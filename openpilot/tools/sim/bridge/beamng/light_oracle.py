#!/usr/bin/env python3
"""The sim's traffic light oracle: the game's own light states ahead of the ego, published as turnpilotLights (simOracle).

SIM ONLY. turnpilotd takes these lights only with SIMULATION=1 and TurnPilotSimLightOracle set (launch_openpilot.sh sets
the param with TURNPILOT_LIGHT_ORACLE=1); anything else that publishes them is rejected out loud and the lights are
unknown, so on the car every mapped signal is held for the driver's go (map_controls.py). The scenario drivers also read
it as the ground truth for their go at a signal (driver.GasTaps), with or without turnpilotd taking it.

It runs on its own BeamNGpy connection (never the bridge's world lock), at RATE_HZ: the ego's pose, then the signal
instances whose stop line (wcusa_map_node_signals.json) is from PAST_M behind the ego (negative distance: the car standing
just past the line still sees the far-side head, as a camera would; BeamNG s14_red_right_frames: dropped 1.3 m past it,
the held red went unknown) to MAX_AHEAD_M ahead, within MAX_LATERAL_M of its axis, facing it (the instance's direction, from
the game, dotted with the ego's heading, over FACING_DOT), nearest first either side of the line, at most MAX_LIGHTS of
them, each read with get_instance_state. A failed poll publishes an empty list: unknown, a hold. The lights go out
--light-latency s late (default 0.3), as a camera's would.

The directions are read once, at the start, and the game has no core_trafficSignals until the level has loaded (BeamNG
ctl_ped_in_path, 2026-10-30: started with the bridge, which then loaded the level in 49 s, the oracle died on its first
read). They are read again every STARTUP_RETRY_S until they come, for STARTUP_TIMEOUT_S; then the oracle exits with
EXIT_NOT_READY having published nothing, and the scenario runner, which waits for its lights (run.ORACLE_WAIT_S), ends the
run 'error'.

  python -m openpilot.tools.sim.bridge.beamng.light_oracle [--host H --port P --light-latency 0.3]
Unverified against the game since it was written (2026-10-27, no game access): run it once and check the published
states against the HUD before trusting a batch on it.
"""
import argparse
import collections
import json
import math
import os
import sys
import time

import openpilot.cereal.messaging as messaging

SIGNALS = os.path.join(os.path.dirname(__file__), "wcusa_map_node_signals.json")
RATE_HZ = 4.
MAX_AHEAD_M = 200.
MAX_LATERAL_M = 25.
FACING_DOT = 0.7
MAX_LIGHTS = 4
PAST_M = 15.
STARTUP_TIMEOUT_S = 180.  # past a level load (about a minute) and the scenario's start
STARTUP_RETRY_S = 2.
EXIT_NOT_READY = 2


def light_state(raw) -> str:
  """A game signal state ('greenTrafficLight', 'red', 2, ...) as TurnpilotLights.State; anything else is unknown."""
  s = str(raw).lower()
  for name in ('red', 'yellow', 'green'):
    if name in s:
      return name
  if 'amber' in s:
    return 'yellow'
  return 'unknown'


def stop_lines(path: str = SIGNALS) -> dict[str, tuple[float, float]]:
  """{traffic light instance: its stop line (x, y)} from the mapNodeSignals dump (the first link it controls)."""
  with open(path) as f:
    mns = json.load(f)
  mns = mns.get("mapNodeSignals", mns)
  out = {}
  for inner in mns.values():
    for lst in inner.values():
      for s in lst:
        if s["instance"].startswith("trafficLight") and s["instance"] not in out:
          out[s["instance"]] = (s["pos"]["x"], s["pos"]["y"])
  return out


def lights_ahead(ego_xy, ego_fwd, lines: dict, dirs: dict) -> list[tuple[str, float]]:
  """[(instance, m ahead to its stop line, negative past it)] facing the ego, from PAST_M past the line to MAX_AHEAD_M
  ahead, nearest first either side of it, at most MAX_LIGHTS."""
  fx, fy = ego_fwd
  n = math.hypot(fx, fy) or 1.
  fx, fy = fx / n, fy / n
  out = []
  for name, (x, y) in lines.items():
    dx, dy = x - ego_xy[0], y - ego_xy[1]
    ahead, lateral = dx * fx + dy * fy, -dx * fy + dy * fx
    d = dirs.get(name)
    if d is None or not (-PAST_M <= ahead <= MAX_AHEAD_M) or abs(lateral) > MAX_LATERAL_M:
      continue
    dn = math.hypot(d[0], d[1]) or 1.
    if (d[0] * fx + d[1] * fy) / dn > FACING_DOT:
      out.append((name, ahead))
  return sorted(out, key=lambda e: abs(e[1]))[:MAX_LIGHTS]


def make_msg(lights: list[tuple[str, float, str]]):
  """turnpilotLights from [(instance, distance, state)]."""
  msg = messaging.new_message('turnpilotLights')
  msg.turnpilotLights.source = 'simOracle'
  msg.turnpilotLights.lights = [{'state': state, 'distance': float(dist), 'instance': name} for name, dist, state in lights]
  return msg


def read_dirs(get_state, names) -> dict[str, tuple[float, float]]:
  """{instance: its direction (x, y)} from the game (get_instance_state), for each that has one."""
  dirs = {}
  for name in names:
    st = get_state(name)
    d = st.get('dir') if isinstance(st, dict) else None
    if d is not None:
      dirs[name] = (d[0], d[1]) if isinstance(d, (list, tuple)) else (d['x'], d['y'])
  return dirs


def wait_for_dirs(get_state, names, timeout: float = STARTUP_TIMEOUT_S, retry: float = STARTUP_RETRY_S, clock=time.monotonic,
                  sleep=time.sleep, log=print) -> dict | None:
  """The lights' directions, read again every retry s while the game has no signals (core_trafficSignals not available
  before the level has loaded) or no light with a direction, for timeout s; None if they never come."""
  deadline, said = clock() + timeout, -math.inf
  while True:
    try:
      dirs = read_dirs(get_state, names)
      why = None if dirs else "no light has a direction"
    except Exception as e:
      dirs, why = {}, repr(e)
    if why is None:
      return dirs
    if clock() >= deadline:
      log(f"light_oracle: no traffic signals after {timeout:.0f} s ({why}): exiting, nothing published")
      return None
    if clock() - said >= 10.:
      log(f"light_oracle: the game's signals aren't ready ({why}); reading again every {retry:.0f} s for up to {timeout:.0f} s")
      said = clock()
    sleep(retry)


def main():
  from beamngpy import BeamNGpy
  from openpilot.tools.sim.bridge.beamng.scenario import ego_pose

  ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
  ap.add_argument('--host', default='192.168.137.1')
  ap.add_argument('--port', type=int, default=25252)
  ap.add_argument('--light-latency', type=float, default=0.3)
  args = ap.parse_args()

  lines = stop_lines()
  bng = BeamNGpy(args.host, args.port).open(launch=False)
  try:
    dirs = wait_for_dirs(bng.traffic_signals.get_instance_state, lines, clock=time.monotonic, sleep=time.sleep,
                         log=lambda m: print(m, flush=True))
    if dirs is None:
      sys.exit(EXIT_NOT_READY)
    print(f"light_oracle: {len(dirs)} of {len(lines)} lights with a direction", flush=True)
    pm = messaging.PubMaster(['turnpilotLights'])
    queue: collections.deque = collections.deque()
    while True:
      t = time.monotonic()
      try:
        pos, heading = ego_pose(bng)
        ahead = lights_ahead(pos[:2], (math.cos(heading), math.sin(heading)), lines, dirs)
        lights = [(name, dist, light_state(bng.traffic_signals.get_instance_state(name).get('state'))) for name, dist in ahead]
      except Exception as e:  # unknown: an empty list, which holds every signal
        print(f"light_oracle: poll failed: {e!r}", flush=True)
        lights = []
      queue.append((t + args.light_latency, lights))
      while queue and queue[0][0] <= time.monotonic():
        pm.send('turnpilotLights', make_msg(queue.popleft()[1]))
      time.sleep(max(1. / RATE_HZ - (time.monotonic() - t), 0.))
  finally:
    bng.disconnect()


if __name__ == "__main__":
  main()
