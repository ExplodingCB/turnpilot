#!/usr/bin/env python3
"""The sim's traffic light oracle: the game's own light states ahead of the ego, published as turnpilotLights (simOracle).

SIM ONLY. turnpilotd takes these lights only with SIMULATION=1 and TurnPilotSimLightOracle set (launch_openpilot.sh sets
the param with TURNPILOT_LIGHT_ORACLE=1); anything else that publishes them is rejected out loud and the lights are
unknown, so on the car every mapped signal is held for the driver's go (map_controls.py). The scenario drivers also read
it as the ground truth for their go at a signal (driver.GasTaps), with or without turnpilotd taking it.

It runs on its own BeamNGpy connection (never the bridge's world lock), at RATE_HZ: the ego's pose, then the signal
instances whose stop line (wcusa_map_node_signals.json) is 0 to MAX_AHEAD_M ahead and within MAX_LATERAL_M of the ego's
axis, facing it (the instance's direction, from the game, dotted with the ego's heading, over FACING_DOT), nearest first,
at most MAX_LIGHTS of them, each read with get_instance_state. A failed poll publishes an empty list: unknown, a hold. The
lights go out --light-latency s late (default 0.3), as a camera's would.

It is also the sim's cross-traffic oracle (STOP_SIGNS.md v3 8.2, SG4c): every vehicle in the game but the ego, and the
scenario's mannequins, within --cross-range m (default ORACLE_RANGE_M, the sim survey's reach; 0: off), in the ego's frame,
in the same message (cross, crossRangeM), which the stop-sign decider takes in place of its cameras' tracks and reach under
the same gate. Logic checks only, never range evidence. A failed poll sends none and range 0 (no oracle: the decider's own
cameras, which hand the stop back in the sim), said here every time.

  python -m openpilot.tools.sim.bridge.beamng.light_oracle [--host H --port P --light-latency 0.3 --cross-range 60]
Unverified against the game since it was written (2026-10-27, no game access; the cross-traffic poll 2026-10-30, the same):
run it once and check the published states against the HUD, and the cross list against the scenario cars, before trusting
a batch on it.
"""
import argparse
import collections
import json
import math
import os
import time

import openpilot.cereal.messaging as messaging
from openpilot.tools.sim.bridge.beamng.sim_stop_files import ORACLE_RANGE_M

SIGNALS = os.path.join(os.path.dirname(__file__), "wcusa_map_node_signals.json")
RATE_HZ = 4.
MAX_AHEAD_M = 200.
MAX_LATERAL_M = 25.
FACING_DOT = 0.7
MAX_LIGHTS = 4
EGO = 'ego'
WALKERS = ('tp_sc_ped',)  # the scenario mannequins (catalog.PREFIX): TSStatics, no velocity of their own
# every vehicle but the ego: name, position and velocity, ';'-separated
VEHICLES_LUA = ("local out = {} for i = 0, be:getObjectCount() - 1 do local o = be:getObject(i) local n = o:getName() " +
                f"if n ~= '{EGO}' then local p = o:getPosition() local v = o:getVelocity() " +
                "table.insert(out, string.format('%s,%f,%f,%f,%f', n, p.x, p.y, v.x, v.y)) end end return table.concat(out, ';')")


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
  """[(instance, m ahead to its stop line)] facing the ego, nearest first, at most MAX_LIGHTS."""
  fx, fy = ego_fwd
  n = math.hypot(fx, fy) or 1.
  fx, fy = fx / n, fy / n
  out = []
  for name, (x, y) in lines.items():
    dx, dy = x - ego_xy[0], y - ego_xy[1]
    ahead, lateral = dx * fx + dy * fy, -dx * fy + dy * fx
    d = dirs.get(name)
    if d is None or not (0. <= ahead <= MAX_AHEAD_M) or abs(lateral) > MAX_LATERAL_M:
      continue
    dn = math.hypot(d[0], d[1]) or 1.
    if (d[0] * fx + d[1] * fy) / dn > FACING_DOT:
      out.append((name, ahead))
  return sorted(out, key=lambda e: e[1])[:MAX_LIGHTS]


def cross_traffic(ego_xy, heading: float, others, range_m: float) -> list[tuple]:
  """[(id, x, y, vx, vy, cls)] in the car frame (x forward, y left, from the ego's pose) of the road users within range_m of
  it, from others [(id, (x, y), (vx, vy), cls)] in the world; heading: the ego's, rad CCW from east. x, y are each one's
  position in the game (a vehicle's reference point, about its centre) from the ego's, not the nearest ground point
  TurnpilotLights.CrossCar names: within a car's half length, which a logic check's geometry takes (STOP_SIGNS.md 8.2)."""
  c, s = math.cos(heading), math.sin(heading)
  out = []
  for ident, (x, y), (vx, vy), cls in others:
    dx, dy = x - ego_xy[0], y - ego_xy[1]
    if math.hypot(dx, dy) > range_m:
      continue
    out.append((int(ident), dx * c + dy * s, -dx * s + dy * c, vx * c + vy * s, -vx * s + vy * c, cls))
  return out


def make_msg(lights: list[tuple[str, float, str]], cross=(), range_m: float = 0.):
  """turnpilotLights from [(instance, distance, state)], with the cross-traffic oracle's road users [(id, x, y, vx, vy, cls)]
  and its range (0: none)."""
  msg = messaging.new_message('turnpilotLights')
  msg.turnpilotLights.source = 'simOracle'
  msg.turnpilotLights.lights = [{'state': state, 'distance': float(dist), 'instance': name} for name, dist, state in lights]
  msg.turnpilotLights.cross = [{'id': int(i), 'x': float(x), 'y': float(y), 'vx': float(vx), 'vy': float(vy), 'cls': cls}
                               for i, x, y, vx, vy, cls in cross]
  msg.turnpilotLights.crossRangeM = float(range_m)
  return msg


class RoadUsers:
  """The game's road users for the cross-traffic oracle: every vehicle but the ego (its velocity from the game) and the
  scenario's mannequins (theirs from their last two poses), each with an id kept for its name."""

  def __init__(self):
    self.ids: dict[str, int] = {}
    self.walker_last: dict[str, tuple] = {}

  def ident(self, name: str) -> int:
    return self.ids.setdefault(name, len(self.ids) + 1)

  def poll(self, bng, lua) -> list[tuple]:
    out = []
    r = str(lua(bng, VEHICLES_LUA, response=True) or '')
    for item in filter(None, r.split(';')):
      name, x, y, vx, vy = item.split(',')
      out.append((self.ident(name), (float(x), float(y)), (float(vx), float(vy)), 'car'))
    now = time.monotonic()
    for name in WALKERS:
      p = str(lua(bng, f"local o = scenetree.findObject('{name}') if not o then return '' end local p = o:getPosition() " +
                       "return string.format('%f,%f', p.x, p.y)", response=True) or '')
      if not p:
        self.walker_last.pop(name, None)
        continue
      x, y = (float(v) for v in p.split(','))
      last = self.walker_last.get(name)
      dt = now - last[0] if last is not None else 0.
      v = ((x - last[1]) / dt, (y - last[2]) / dt) if last is not None and dt > 1e-3 else (0., 0.)
      self.walker_last[name] = (now, x, y)
      out.append((self.ident(name), (x, y), v, 'pedestrian'))
    return out


def main():
  from beamngpy import BeamNGpy
  from openpilot.tools.sim.bridge.beamng.scenario import ego_pose, lua

  ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
  ap.add_argument('--host', default='192.168.137.1')
  ap.add_argument('--port', type=int, default=25252)
  ap.add_argument('--light-latency', type=float, default=0.3)
  ap.add_argument('--cross-range', type=float, default=ORACLE_RANGE_M, help="the cross-traffic oracle's range, m (0: off)")
  args = ap.parse_args()

  pm = messaging.PubMaster(['turnpilotLights'])
  lines = stop_lines()
  bng = BeamNGpy(args.host, args.port).open(launch=False)
  try:
    dirs = {}
    for name in lines:
      st = bng.traffic_signals.get_instance_state(name)
      d = st.get('dir') if isinstance(st, dict) else None
      if d is not None:
        dirs[name] = (d[0], d[1]) if isinstance(d, (list, tuple)) else (d['x'], d['y'])
    print(f"light_oracle: {len(dirs)} of {len(lines)} lights with a direction", flush=True)
    queue: collections.deque = collections.deque()
    users = RoadUsers()
    while True:
      t = time.monotonic()
      pos = heading = None
      try:
        pos, heading = ego_pose(bng)
        ahead = lights_ahead(pos[:2], (math.cos(heading), math.sin(heading)), lines, dirs)
        lights = [(name, dist, light_state(bng.traffic_signals.get_instance_state(name).get('state'))) for name, dist in ahead]
      except Exception as e:  # unknown: an empty list, which holds every signal
        print(f"light_oracle: poll failed: {e!r}", flush=True)
        lights = []
      cross, cross_range = [], 0.
      if args.cross_range > 0.:
        try:
          if pos is None:
            raise RuntimeError("no ego pose")
          cross, cross_range = cross_traffic(pos[:2], heading, users.poll(bng, lua), args.cross_range), args.cross_range
        except Exception as e:  # no oracle (range 0): the decider's own cameras, which hand the stop back in the sim
          print(f"light_oracle: cross-traffic poll failed, sent none (range 0): {e!r}", flush=True)
      queue.append((t + args.light_latency, (lights, cross, cross_range)))
      while queue and queue[0][0] <= time.monotonic():
        pm.send('turnpilotLights', make_msg(*queue.popleft()[1]))
      time.sleep(max(1. / RATE_HZ - (time.monotonic() - t), 0.))
  finally:
    bng.disconnect()


if __name__ == "__main__":
  main()
