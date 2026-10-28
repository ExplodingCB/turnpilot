#!/usr/bin/env python3
"""Gate 1: does one desire pulse make the driving model take a 90-degree junction?

Spawns the ego on the approach to a junction from the road-graph export, engages, and publishes turnpilotPlan at 20 Hz:
desire none, then turnLeft or turnRight for a few frames when the distance to the junction drops to the lead distance,
then none. modeld only uses a desire on the frame it first appears, so the pulse is the only lever. Logs the heading
change, the path, and whether the car left on the exit road. One bridge session runs every lead in --leads.

Start openpilot with turnpilotd blocked, since only one process may publish turnpilotPlan:
  BLOCK=turnpilotd openpilot/tools/sim/bridge/beamng/launch_openpilot.sh
  python -m openpilot.tools.sim.bridge.beamng.gate1 --junction group_80_81_82_83 --turn left --leads 10,20,30,40
"""
import argparse
import json
import math
import time
from multiprocessing import Queue

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.tools.sim.bridge.beamng.bridge import BeamNGBridge
from openpilot.tools.sim.bridge.beamng.roadgraph import RoadGraph, wrap
from openpilot.tools.sim.bridge.beamng.run_bridge import config_from_args, parse_args
from openpilot.tools.sim.bridge.beamng.scenario import heading_quat, load_signals
from openpilot.tools.sim.bridge.common import control_cmd_gen

SETTLE = 4.0  # s after the teleport before engaging
ENGAGE_TIMEOUT = 20.0
PASS_DIST = 35.0  # m from the junction, moving away, ends a run
RUN_TIMEOUT = 90.0
STUCK_TIME = 15.0
# s between cancelling and the next teleport: a teleport stops the car dead, and if openpilot's longitudinal is still
# active then, selfdrived latches Offroad_ExcessiveActuation and nothing engages again until it's cleared
CANCEL_WAIT = 1.5
TOOK_TURN_HEADING = 30.0  # deg from the expected 90
TOOK_TURN_EXIT = 6.0  # m from the exit road's centreline


def polyline_dist(p, pts) -> float:
  """Distance from point p to a polyline, in the plane."""
  best = math.inf
  for a, b in zip(pts[:-1, :2], pts[1:, :2], strict=True):
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-9), 0.0, 1.0))
    best = min(best, float(np.linalg.norm(p - (a + t * ab))))
  return best


class Gate1:
  """Runs in the bridge process, called at 100 Hz with the bridge (BeamNGBridge scenario hook)."""

  def __init__(self, turn, leads, approach: float, speed_kph: float, pulse_frames: int, out: str):
    self.turn, self.leads, self.approach, self.speed_kph = turn, list(leads), approach, speed_kph
    self.pulse_frames, self.out = pulse_frames, out
    self.desire = log.Desire.turnLeft if turn.direction == "left" else log.Desire.turnRight
    if pulse_frames == 0:  # the baseline: what the model does at this junction with no desire
      self.desire, self.pulse_frames = log.Desire.none, 1
    self.results, self.run, self.phase, self.t_end = [], -1, "next", 0.0
    self.pm = None

  def start_run(self, b):
    self.run += 1
    self.lead = self.leads[self.run]
    pos, heading = self.turn.spawn(self.approach)
    b.car.cancel()
    b.world.teleport(pos, heading_quat(heading))
    self.t0, self.phase, self.pulses_sent, self.engaged_t = time.monotonic(), "settle", 0, None
    self.rows, self.t_pulse, self.disengaged, self.min_dist = [], None, False, math.inf
    print(f"gate1 run {self.run + 1}/{len(self.leads)}: {self.turn.direction} at {self.turn.junction}, lead {self.lead} m", flush=True)

  def publish(self, desire, dist):
    msg = messaging.new_message('turnpilotPlan', valid=True)  # hooks.fresh_plan drops an invalid plan
    p = msg.turnpilotPlan
    p.state = custom.TurnpilotPlan.State.turning if desire else custom.TurnpilotPlan.State.approaching
    p.desire = desire
    p.accelCap, p.speedCap = 10.0, 100.0
    p.maneuverDistance = float(dist)
    p.maneuverType, p.maneuverModifier = "turn", self.turn.direction
    p.routeValid = True
    p.reason = "gate1"
    self.pm.send('turnpilotPlan', msg)

  def __call__(self, b) -> bool:
    if self.pm is None:
      self.pm = messaging.PubMaster(['turnpilotPlan'])
    s = b.simulator_state
    now = time.monotonic()
    if self.phase == "next":
      if now - self.t_end < CANCEL_WAIT:
        return False
      if self.run + 1 >= len(self.leads):
        self.finish(b)
        return True
      self.start_run(b)
      return False

    pos = np.array([s.position.x, s.position.y])
    dist = float(np.linalg.norm(pos - self.turn.center[:2]))
    b.car.sm.update(0)
    enabled = b.car.sm['selfdriveState'].enabled

    if self.phase == "settle":
      if now - self.t0 > SETTLE:
        b.set_speed_kph, b.rolling_start_kph, b.auto_engage, self.phase = self.speed_kph, self.speed_kph * 0.8, True, "engage"
    elif self.phase == "engage":
      if enabled:
        self.phase, self.engaged_t = "approach", now
      elif now - self.t0 > SETTLE + ENGAGE_TIMEOUT:
        self.end_run(b, "no engagement")
    elif self.phase in ("approach", "pulse", "after"):
      if not enabled and not self.disengaged:
        self.disengaged = True
      self.min_dist = min(self.min_dist, dist)
      if self.phase == "approach" and dist <= self.lead:
        self.phase, self.t_pulse = "pulse", now
      desire = self.desire if self.phase == "pulse" else log.Desire.none
      if b.rk_frame % 5 == 0:  # 20 Hz, like turnpilotd
        self.publish(desire, dist)
        if self.phase == "pulse":
          self.pulses_sent += 1
          if self.pulses_sent >= self.pulse_frames:
            self.phase = "after"
      if b.rk_frame % 10 == 0:
        self.rows.append((round(now - self.t0, 2), round(float(pos[0]), 2), round(float(pos[1]), 2),
                          round(math.degrees(math.atan2(s.velocity.y, s.velocity.x)), 1), round(s.wheel_speed, 2), round(dist, 1),
                          int(desire), round(s.steering_angle, 1), bool(enabled)))
      moving_away = dist > PASS_DIST and self.min_dist < PASS_DIST and self.phase == "after"
      stuck = len(self.rows) > 150 and max(r[4] for r in self.rows[-150:]) < 0.2
      if moving_away or now - self.t0 > RUN_TIMEOUT or stuck:
        self.end_run(b, "passed" if moving_away else ("stuck" if stuck else "timeout"))
    return False

  def end_run(self, b, reason):
    b.car.cancel()
    b.auto_engage = False
    self.t_end = time.monotonic()
    rows = self.rows
    res = {"run": self.run + 1, "lead": self.lead, "direction": self.turn.direction, "junction": self.turn.junction, "end": reason,
           "disengaged": self.disengaged, "expected_deg": round(math.degrees(self.turn.angle), 1)}
    moving = [r for r in rows if r[4] > 1.0]
    if self.t_pulse is not None and moving:
      t_pulse = self.t_pulse - self.t0
      before = [r for r in moving if t_pulse - 1.5 <= r[0] <= t_pulse] or moving[:1]
      h_in = math.radians(np.mean([r[3] for r in before]))
      h_out = math.radians(np.mean([r[3] for r in moving[-5:]]))
      dh = math.degrees(wrap(h_out - h_in))
      end = np.array(rows[-1][1:3])
      exit_dist = polyline_dist(end, self.turn.exit)
      res.update(heading_change_deg=round(dh, 1), exit_dist_m=round(exit_dist, 2), speed_at_pulse=next((r[4] for r in rows if r[0] >= t_pulse), None),
                 took_turn=bool(abs(dh - math.degrees(self.turn.angle)) < TOOK_TURN_HEADING and exit_dist < TOOK_TURN_EXIT))
    res['path'] = rows
    self.results.append(res)
    print(f"gate1 run {res['run']}: {json.dumps({k: v for k, v in res.items() if k != 'path'})}", flush=True)
    self.phase = "next"

  def finish(self, b):
    took = sum(r.get('took_turn', False) for r in self.results)
    summary = {"turns_taken": took, "runs": len(self.results), "junction": self.turn.junction, "direction": self.turn.direction,
               "desire": int(self.desire), "pulse_frames": self.pulse_frames, "speed_kph": self.speed_kph, "runs_detail": self.results}
    with open(self.out, 'w') as f:
      json.dump(summary, f, indent=1)
    print(f"gate1: {took}/{len(self.results)} turns taken, results in {self.out}", flush=True)
    b.shutdown()


def pick_turn(graph: RoadGraph, junction: str, direction: str):
  sig = {j['group']: j for j in load_signals()}
  node = graph.nearest_junction(sig[junction]['x'], sig[junction]['y']) if junction in sig else junction
  turns = [t for t in graph.turns(node) if t.direction == direction]
  if not turns:
    raise SystemExit(f"no legal {direction} turn of about 90 degrees at {junction}")
  return max(turns, key=lambda t: float(np.linalg.norm(t.approach[-1][:2] - t.approach[0][:2])))  # the longest approach


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--junction', default='group_80_81_82_83', help='signal group from wcusa_signals.json or a road-graph node')
  ap.add_argument('--turn', choices=('left', 'right'), default='left')
  ap.add_argument('--leads', default='20', help='comma-separated lead distances, m (one run each)')
  ap.add_argument('--approach', type=float, default=140.0, help='spawn this far before the junction, m')
  ap.add_argument('--speed', type=float, default=25.0, help='cruise set speed, km/h')
  ap.add_argument('--pulse-frames', type=int, default=4, help='turnpilotPlan messages (20 Hz) that carry the desire; 0 for a no-desire baseline')
  ap.add_argument('--out', default='gate1_results.json')
  ap.add_argument('--list', action='store_true', help='list signalized junctions with 90-degree turns and exit')
  args, rest = ap.parse_known_args()
  graph = RoadGraph()
  if args.list:
    for j in load_signals():
      node = graph.nearest_junction(j['x'], j['y'])
      for t in graph.turns(node):
        n = float(np.linalg.norm(t.approach[-1][:2] - t.approach[0][:2]))
        if n > 60:
          print(f"{j['group']:28s} {node:16s} {t.direction:5s} {math.degrees(t.angle):6.1f} deg, approach {n:5.0f} m, lanes {t.lanes_in}")
    return

  turn = pick_turn(graph, args.junction, args.turn)
  gate = Gate1(turn, [float(x) for x in args.leads.split(',')], args.approach, args.speed, args.pulse_frames, args.out)
  bargs = parse_args(rest)
  q: Queue = Queue()
  bridge = BeamNGBridge(config_from_args(bargs), scenario=gate)
  p = bridge.run(q)
  try:
    p.join()
  except KeyboardInterrupt:
    q.put(control_cmd_gen("quit"))
    p.join()


if __name__ == "__main__":
  main()
