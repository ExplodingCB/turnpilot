#!/usr/bin/env python3
"""Run a scenario on the BeamNG bridge and score it.

Each run: teleport the car to the scenario's start, place its mannequin or cars, send the destination to turnpilot_web
the way the phone does, engage, then watch turnpilotPlan, selfdriveState, carState, navInstruction and modelV2 until
the car arrives, disengages, stops (where the scenario ends on a stop) or times out. A result JSON per invocation holds
every run: outcome, maneuvers taken of attempted, disengagements with cause, holds with reason and duration, the
closest the car came to the mannequin or another car, the rlog segments, the 20 Hz rows it was scored from, and a
20 Hz pose log of the car and the actors for chase_render.py. Everything a run spawns is removed after it, and again
from here after the bridge exits (a sweep by name).

Start openpilot first (navd, turnpilotd and turnpilot_web need NavigationEnabled and TurnPilotEnabled, which this sets):
  OPENPILOT_PREFIX=tpsim openpilot/tools/sim/bridge/beamng/launch_openpilot.sh
  OPENPILOT_PREFIX=tpsim python -m openpilot.tools.sim.bridge.beamng.scenarios.run ped_crosswalk_right --runs 3 --record ~/turnpilot-artifacts/demo
Bridge options (--host, --signals, ...) pass through. --record records the UI while the runs drive and renders each run's
chase view after the bridge exits, from the pose log: a live chase camera (--capture) costs the driving cameras 6 to 9
pairs a second, at which the model's turns fail.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
import traceback
import urllib.request
from multiprocessing import Queue
from pathlib import Path

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.selfdrive.turnpilot.web.pairing import PORT
from openpilot.selfdrive.turnpilot.web.store import ensure_token
from openpilot.tools.sim.bridge.beamng import frame
from openpilot.tools.sim.bridge.beamng.driver import GasTaps, ground_truth_green
from openpilot.tools.sim.bridge.beamng.pose import beamng_quat
from openpilot.tools.sim.bridge.beamng.scenario import heading_quat
from openpilot.tools.sim.bridge.beamng.scenarios import score
from openpilot.tools.sim.bridge.beamng.scenarios.catalog import CAR_HALF_M, DATA_JSON, ROLL_FRACTION, SCENARIOS, Plan, build, junction_distance
from openpilot.tools.sim.bridge.beamng.scenarios.director import Director, sweep
from openpilot.tools.sim.bridge.beamng.scenarios.timing import GO_NEAR_M, STOOD_V, GoWatch

SETTLE_S = 4.         # after the teleport: the car is back in D and the GPS has caught up
CLEAR_TIMEOUT_S = 5.  # wait this long for navd to drop the old route
# TurnPilot is turned off at each run's start and on again this long after (turnpilotd reads TurnPilotEnabled once a second),
# before the destination goes out: the teleport back would leave its route follower dead-reckoning the last run's junction
# (gaps are dead-reckoned through) and its latch and left on it (BeamNG unprotected_left_gap6: a hold 13 m early)
TP_RESET_S = 2.5
ROUTE_TIMEOUT_S = 30.
ENGAGE_TIMEOUT_S = 30.

TURN_DESIRES = (int(log.Desire.turnLeft), int(log.Desire.turnRight))
SERVICES = ['turnpilotPlan', 'selfdriveState', 'carState', 'navInstruction', 'navRoute', 'modelV2', 'turnpilotLights']
OUT_ROOT = os.path.expanduser("~/turnpilot-artifacts/scenarios")
POSE_FIELDS = ["wall_s", "t_s", "x", "y", "z", "qx", "qy", "qz", "qw"]  # route_drive's, which chase_render.py reads
ACTOR_FIELDS = ["wall_s", "x", "y", "z", "heading"]


def cross_due(params: dict, held_t: float | None, went: bool, t: float, key: str = 'release', j_dist: float | None = None) -> bool:
  """A stop-sign scenario's cross car's leg (catalog.Entry.cross) is due: params[key] (its 'arrive' or its 'release') s after
  the car's stand at the junction (held_t), on the car's go there ('go'), once the car is within m of the junction
  (('near', m), j_dist: the car's), or never (None)."""
  when = params.get(key)
  if when is None:
    return False
  if when == 'go':
    return went
  if isinstance(when, (list, tuple)) and len(when) == 2 and when[0] == 'near':
    return j_dist is not None and j_dist <= float(when[1])
  return held_t is not None and t - held_t >= float(when)


def wall() -> float:
  return time.time()  # noqa: TID251  the recordings are aligned on wall-clock time


def web_call(url: str, token: str, method: str, body: dict | None = None) -> dict:
  data = None if body is None else json.dumps(body).encode()
  req = urllib.request.Request(url, data=data, method=method, headers={'Authorization': f'Bearer {token}', 'Content-Type': 'application/json'})
  with urllib.request.urlopen(req, timeout=5) as r:
    return json.loads(r.read() or b'{}')


def rlog_segments(since: float) -> list[str]:
  root = Path(Paths.log_root())
  if not root.is_dir():
    return []
  return sorted(str(d) for d in root.iterdir() if d.is_dir() and d.stat().st_mtime >= since)


def write_sidecar(video: str, start: float, fps: float, source: str):
  """`<video>.start.json`: the wall-clock time of the video's first frame, which demo_video aligns on."""
  with open(f"{video}.start.json", 'w') as f:
    json.dump({'start': start, 'fps': fps, 'source': source}, f)


class Runner:
  """The scenario, run in the bridge process at 100 Hz (BeamNGBridge scenario hook). Built in the parent and pickled into
  the bridge process, so everything live (SubMaster, the director thread and its client) starts on the first call."""

  def __init__(self, plan: Plan, runs: int, out_dir: str, web_url: str, token: str, host: str, port: int, signal_tap: bool = True,
               params=None):
    """params: Params, for TurnPilot's reset between runs (TP_RESET_S); None leaves TurnPilot be."""
    if plan.road is None:
      raise ValueError(f"scenario {plan.name} has no road model: its runs could not be checked for the oncoming lane")
    self.plan, self.runs, self.out_dir, self.web_url, self.token = plan, runs, out_dir, web_url, token
    self.signal_tap = signal_tap
    self.params = params
    self.tp_off_t: float | None = None  # when this run turned TurnPilot off (TP_RESET_S)
    self.host, self.port = host, port
    self.sm = self.director = None
    self.results: list[dict] = []
    self.run_idx = -1
    self.phase = 'next'
    self.sidecar_done = False
    self.started_wall = wall()

  def __call__(self, b) -> bool:
    try:
      if self.sm is None:
        self.sm = messaging.SubMaster(SERVICES)
        self.director = Director(self.host, self.port)
        self.director.start()
      self.sidecar(b)
      return self.step(b)
    except Exception:
      traceback.print_exc()
      if self.phase not in ('next', 'done'):
        self.end_run(b, 'error', traceback.format_exc(limit=3))
      self.finish(b)
      return True

  def sidecar(self, b):
    cap = getattr(b.world, 'capture', None)
    if cap is not None and not self.sidecar_done and cap.frames >= 10:  # the capture writes one frame a tick from its first
      from openpilot.tools.sim.bridge.beamng.capture import FPS
      write_sidecar(b.cfg.capture, wall() - cap.frames / FPS, FPS, 'beamng chase')
      self.sidecar_done = True

  def step(self, b) -> bool:
    now = time.monotonic()
    self.sm.update(0)
    if self.sm.updated['navRoute']:
      c = self.sm['navRoute'].coordinates
      self.route_pts, self.route_id = len(c), score.route_id(c)
    if self.sm.updated['navInstruction'] and not self.sm.valid['navInstruction']:
      self.nav_cleared = True
    plan = self.plan
    if self.phase not in ('next', 'done') and b.rk_frame % 5 == 0:  # 20 Hz, from the teleport to the end of the run
      self.log_pose(b)
    if not self.director.is_alive() and self.phase not in ('next', 'done'):
      self.end_run(b, 'error', "the scenario director stopped (is the game reachable?)")
      self.finish(b)
      return True
    if self.phase == 'next':
      if self.run_idx + 1 >= self.runs:
        self.finish(b)
        return True
      self.start_run(b)
    elif self.phase == 'settle':
      if now - self.t0 > max(SETTLE_S, TP_RESET_S) and self.setup.done():
        if self.tp_off_t is not None:  # TurnPilot back on, a fresh one, before the destination goes out (TP_RESET_S)
          self.params.put_bool('TurnPilotEnabled', True)
          self.tp_off_t = None
        if self.setup.exception() is not None:
          self.end_run(b, 'error', f"setup: {self.setup.exception()!r}")
        elif plan.dest is None:
          self.engage(b)
        else:
          self.web = self.director.submit(web_call, f"{self.web_url}/api/destination", self.token, 'DELETE')
          self.nav_cleared, self.phase, self.t_phase = False, 'clear', now
    elif self.phase == 'clear':
      # navd drops the old route within its 1 Hz tick; posting before it does could leave navd on the old one
      if self.web.done() and (self.nav_cleared or now - self.t_phase > CLEAR_TIMEOUT_S):
        lat, lon, _ = frame.to_latlon(*plan.dest)
        self.web = self.director.submit(web_call, f"{self.web_url}/api/destination", self.token, 'POST',
                                        {'latitude': lat, 'longitude': lon, 'name': f"scenario {plan.name}"})
        self.route_pts, self.phase, self.t_phase = None, 'route', now
    elif self.phase == 'route':
      if self.web.done() and self.web.exception() is not None:
        self.end_run(b, 'no destination', f"turnpilot_web at {self.web_url}: {self.web.exception()!r}")
      elif self.web.done() and self.route_pts and self.sm.valid['navInstruction'] and self.sm['navInstruction'].distanceRemaining > 0:
        self.engage(b)
      elif now - self.t_phase > ROUTE_TIMEOUT_S:
        self.end_run(b, 'no route', "navd sent no route (is navd running, with a route source for the sim map?)")
    elif self.phase == 'engage':
      if self.sm['selfdriveState'].enabled:
        self.phase, self.t_engaged, self.t_engaged_wall = 'run', now, wall()
      elif now - self.t_phase > ENGAGE_TIMEOUT_S:
        self.end_run(b, 'no engagement', "openpilot never engaged")
    elif self.phase == 'run' and b.rk_frame % 5 == 0:  # 20 Hz
      r = self.row(b, now)
      self.rows.append(r)
      self.triggers(b, r)
      self.driver(b)
      end = self.end_check.update(r)
      if end is not None:
        self.end_run(b, end)
    return False

  def start_run(self, b):
    self.run_idx += 1
    plan = self.plan
    print(f"scenario {plan.name} run {self.run_idx + 1}/{self.runs}", flush=True)
    b.auto_engage = False
    b.car.cancel()
    b.blinker = (False, False)
    b.world.teleport(plan.spawn, heading_quat(plan.heading))
    self.setup = self.director.submit(self.director.setup, plan.actors)
    self.t0, self.t0_wall, self.phase = time.monotonic(), wall(), 'settle'
    self.rows: list[dict] = []
    self.poses: list[tuple] = []
    self.actor_poses: dict[str, dict] = {}
    self.fired: set[str] = set()
    self.arrived: set[str] = set()  # cross cars sent up to their line (their first leg)
    self.stood_t: float | None = None  # the car's first stand near the junction, held by TurnPilot or not (a queue)
    self.jobs: list = []  # trigger jobs: a failed one explains a failed run
    self.route_pts, self.route_id, self.nav_cleared = None, 0, False
    self.t_engaged = self.t_engaged_wall = None
    self.end_check = score.EndCheck(plan.timeout_s, plan.end_on_stop_s)
    self.taps = GasTaps(self.signal_tap)
    self.watch = GoWatch()
    if self.params is not None:
      self.params.put_bool('TurnPilotEnabled', False)
      self.tp_off_t = time.monotonic()

  def engage(self, b):
    b.set_speed_kph, b.rolling_start_kph, b.auto_engage = self.plan.speed_kph, self.plan.speed_kph * ROLL_FRACTION, True
    self.phase, self.t_phase = 'engage', time.monotonic()

  def row(self, b, now) -> dict:
    s, sm = b.simulator_state, self.sm
    x, y = s.position.x, s.position.y
    h = math.radians(90.0 - s.bearing)  # compass to CCW from east
    r = {
      't': round(now - self.t_engaged, 3), 'wall': round(wall(), 3), 'x': round(x, 2), 'y': round(y, 2), 'heading': round(h, 3),
      'v': round(s.wheel_speed, 2), 'route': self.route_id,
      'dest_dist': None if self.plan.dest is None else round(math.hypot(self.plan.dest[0] - x, self.plan.dest[1] - y), 1),
      'blinker': 'left' if b.blinker[0] else 'right' if b.blinker[1] else '',
      'ped_gap': None, 'car_gap': None, 'pacer_dx': None, 'j_dist': junction_distance(self.plan, x, y), 'car_j_dist': None,
      'onc': None, 'cross': None,
      **score.msg_fields(sm['selfdriveState'], sm['carState'], sm['turnpilotPlan'] if sm.seen['turnpilotPlan'] else None,
                         sm['navInstruction'] if sm.seen['navInstruction'] and sm.valid['navInstruction'] else None,
                         sm['modelV2']),
    }
    for a in self.plan.actors:
      pose = self.director.poses.get(a.name)
      if pose is None:
        continue
      d = math.hypot(pose[0] - x, pose[1] - y)
      key = 'ped_gap' if a.kind == 'walker' else 'car_gap'
      r[key] = round(d if r[key] is None else min(r[key], d), 2)
      if a.trigger == 'oncoming':  # where the oncoming traffic is, for the score's "presented" (score.oncoming_presented)
        dj = junction_distance(self.plan, pose[0], pose[1])
        r['car_j_dist'] = round(dj if r['car_j_dist'] is None else min(r['car_j_dist'], dj), 2)
        # and each car's, with how far the junction is ahead of it along its heading, for the left's PET and false gos
        hc = pose[2]
        ahead = (self.plan.junction[0] - pose[0]) * math.cos(hc) + (self.plan.junction[1] - pose[1]) * math.sin(hc)
        r['onc'] = (r['onc'] or []) + [[round(dj, 2), round(ahead, 2)]]
      if a.trigger == 'cross' and a.params.get('side') != 'lead':  # a stop sign's cross car, for its PET (score.stop_sign)
        hc = pose[2]
        ahead = (self.plan.junction[0] - pose[0]) * math.cos(hc) + (self.plan.junction[1] - pose[1]) * math.sin(hc)
        r['cross'] = (r['cross'] or []) + [[round(junction_distance(self.plan, pose[0], pose[1]), 2), round(ahead, 2),
                                            round(a.params['line_m'] + CAR_HALF_M, 2)]]  # and where it stands at its line
      if a.trigger == 'pace':
        r['pacer_dx'] = round((pose[0] - x) * math.cos(h) + (pose[1] - y) * math.sin(h), 2)
    r['j_ahead'] = None
    if r['j_dist'] is not None:
      r['j_dist'] = round(r['j_dist'], 2)
      r['j_ahead'] = round((self.plan.junction[0] - x) * math.cos(h) + (self.plan.junction[1] - y) * math.sin(h), 2)
    return r

  def log_pose(self, b):
    """The car's IMU pose as route_drive logs it, and each actor's pose from the director, for chase_render.py. An
    actor's pose that holds keeps its first and last rows only, so a car standing until it is released starts on time."""
    pose = b.world.pose
    if pose is not None:
      self.poses.append((round(wall(), 3), round(time.monotonic() - self.t0, 2), *(round(float(v), 3) for v in pose.pos),
                         *(round(v, 5) for v in beamng_quat(pose.fwd, pose.up))))
    for a in self.plan.actors:
      p = self.director.poses.get(a.name)
      if p is None:
        continue
      new = [round(wall(), 3), round(p[0], 3), round(p[1], 3), round(p[3], 3), round(p[2], 4)]
      rows = self.actor_poses.setdefault(a.name, {'kind': a.kind, 'rows': []})['rows']
      if len(rows) >= 2 and rows[-1][1:] == new[1:] and rows[-2][1:] == new[1:]:
        rows[-1][0] = new[0]
      else:
        rows.append(new)

  def triggers(self, b, r):
    """Set the actors going on the car at its junction (timing.GoWatch): the walker steps out on the car's go there, the
    oncoming cars go a fixed time after it stands at its hold there, so the first reaches the junction arrive_after_s
    after the go (or at once on a go that came sooner); the pacer goes once the car is engaged; a stop sign's cross cars
    (cross_due) drive up to their line on their arrive and on across on their release, timed from the car's first stand
    near the junction (held or not: behind a queue's lead) or on its go."""
    plan, d = self.plan, self.director
    if r['j_dist'] is not None:
      self.watch.update(r['t'], r['j_dist'], r['v'], r['state'] == 'holding', r['desire'] in TURN_DESIRES)
      if self.stood_t is None and r['j_dist'] <= GO_NEAR_M and r['v'] < STOOD_V:
        self.stood_t = r['t']
    w = self.watch
    for a in plan.actors:
      if a.name in self.fired or a.trigger == 'setup':
        continue
      go = False
      if a.trigger == 'cross' and a.params['start'] != 'line' and a.name not in self.arrived:
        if cross_due(a.params, self.stood_t, w.went, r['t'], 'arrive', r['j_dist']):
          self.arrived.add(a.name)
          self.jobs.append((a.name, d.submit(d.arrive, a.name)))
          print(f"scenario: {a.name} drives up to its line at t={r['t']:.1f} s", flush=True)
        continue
      if a.trigger == 'cross':
        go, job = cross_due(a.params, self.stood_t, w.went, r['t'], j_dist=r['j_dist']), (d.cross_on, a.name)
      elif a.trigger == 'walk':
        go, job = w.went, (d.walk, a.name)
      elif a.trigger == 'oncoming':
        # a gap stream's car only on its own timer (catalog: timed); the others at the car's go too
        go = (w.went and not a.params.get('timed')) or (w.held_t is not None and r['t'] - w.held_t >= a.params['release_after_stop_s'])
        job = (d.release, a.name)
      elif a.trigger == 'pace':
        go, job = True, (d.release_pacer, a.name, (r['x'], r['y']), r['v'])
      if go:
        self.fired.add(a.name)
        self.jobs.append((a.name, d.submit(*job)))
        where = '' if r['j_dist'] is None else f", {r['j_dist']:.0f} m from the junction"
        print(f"scenario: {a.name} goes at t={r['t']:.1f} s{where}", flush=True)

  def driver(self, b):
    """The driver taps the stalk when TurnPilot asks for a signal (PLAN: Tesla can't signal for openpilot), and lets go
    when it stops asking, which it does as soon as the lane change starts. And taps the gas where a driver gives the go
    (driver.GasTaps): at a stop sign TurnPilot holds past its dwell, at a signal it hasn't checked when the light really
    is green, and standing with nothing held (the model not pulling away). Never on a stale plan or a handback."""
    plan = self.sm['turnpilotPlan'] if self.sm.seen['turnpilotPlan'] else None
    p = str(plan.signalPrompt) if plan is not None else 'none'
    b.blinker = (p == 'left', p == 'right')
    now = time.monotonic()
    b.driver_gas = self.taps.update(now, plan, self.sm['selfdriveState'].enabled, b.simulator_state.wheel_speed,
                                    ground_truth_green(b.cfg, self.sm, now), plan_age_s=now - self.sm.recv_time['turnpilotPlan'])

  def end_run(self, b, outcome: str, detail: str = ''):
    b.auto_engage = False
    b.driver_gas = 0.
    b.car.cancel()
    b.blinker = (False, False)
    end_wall = wall()
    failed = [f"{name}: {f.exception()!r}" for name, f in getattr(self, 'jobs', []) if f.done() and f.exception() is not None]
    detail = '; '.join([detail] * bool(detail) + failed)
    res = score.summarize(self.plan.name, self.rows, outcome, road=self.plan.road, junction_control=self.plan.notes.get('junction_control'))
    # every go the sim driver gave, with its reason (the score's gas_taps are the presses openpilot saw, from the rows)
    res['driver_taps'] = getattr(self, 'taps', GasTaps()).log
    res.update(run=self.run_idx + 1, detail=detail, t_start_wall=self.t0_wall, t_engaged_wall=self.t_engaged_wall, t_end_wall=end_wall,
               duration_s=round(end_wall - (self.t_engaged_wall or self.t0_wall), 1), rlog_segments=rlog_segments(self.t0_wall - 60.),
               rows=self.rows, pose_fields=POSE_FIELDS, poses=self.poses, actor_fields=ACTOR_FIELDS, actor_poses=self.actor_poses)
    self.results.append(res)
    print(score.report(f"scenario {self.plan.name} run {res['run']}", res) + (f"\n  {detail}" if detail else ''), flush=True)
    self.director.submit(self.director.cleanup)
    self.save()
    self.phase = 'next'

  def save(self):
    os.makedirs(self.out_dir, exist_ok=True)
    plan = self.plan
    out = {'scenario': plan.name, 'started_wall': self.started_wall, 'spawn': plan.spawn, 'heading': plan.heading, 'dest': plan.dest,
           'junction': plan.junction, 'speed_kph': plan.speed_kph, 'notes': plan.notes, 'road': plan.road,
           'actors': [{'name': a.name, 'kind': a.kind, 'trigger': a.trigger, 'path': a.path, 'params': a.params} for a in plan.actors],
           'passed': sum(r['passed'] for r in self.results), 'runs': self.results}
    with open(os.path.join(self.out_dir, 'result.json'), 'w') as f:
      json.dump(out, f, indent=1)

  def finish(self, b):
    self.phase = 'done'
    if self.director is not None:
      self.director.close()
    self.save()
    passed = sum(r['passed'] for r in self.results)
    print(f"scenario {self.plan.name}: {passed}/{len(self.results)} passed, results in {os.path.join(self.out_dir, 'result.json')}", flush=True)
    b.shutdown()


def sweep_game(host: str, port: int):
  from beamngpy import BeamNGpy
  bng = BeamNGpy(host, port).open(launch=False)
  try:
    n = sweep(bng)
    if n:
      print(f"cleanup: removed {n} leftover scenario actor(s)", flush=True)
  finally:
    bng.disconnect()


def start_ui_recording(path: str):
  env = dict(os.environ, RECORD_OUTPUT=path)
  return subprocess.Popen([sys.executable, '-m', 'openpilot.tools.turnpilot.record_ui', path], env=env)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('name', nargs='?', choices=sorted(SCENARIOS), help='scenario')
  ap.add_argument('--runs', type=int, default=1)
  ap.add_argument('--record', default='', help="record the UI, and render each run's chase view after, into DIR/<scenario>-<time>/")
  ap.add_argument('--no-ui', action='store_true', help='with --record, render the chase view only')
  ap.add_argument('--no-chase', action='store_true', help='with --record, record the UI only (chase_render.py can render it later)')
  ap.add_argument('--out', default=OUT_ROOT, help='results go to OUT/<scenario>-<time>/result.json (or the --record directory)')
  ap.add_argument('--data', default=DATA_JSON, help='junction list for lane_change and occupied_lane')
  ap.add_argument('--entry', default='', help='entry of --data by name (default: the first)')
  ap.add_argument('--web', default=f'http://127.0.0.1:{PORT}', help='turnpilot_web, where the destination goes')
  ap.add_argument('--cleanup', action='store_true', help='only remove leftover scenario actors from the game')
  ap.add_argument('--signal-tap', choices=('on', 'off'), default='on',
                  help="off: the driver never gives the go at a signal TurnPilot hasn't checked (the stop-at-the-line runs)")
  args, rest = ap.parse_known_args()

  from openpilot.tools.sim.bridge.beamng.run_bridge import config_from_args, light_oracle, parse_args
  bargs = parse_args(rest)
  if args.cleanup:
    sweep_game(bargs.host, bargs.port)
    return
  if args.name is None:
    ap.error("name a scenario")

  plan = build(args.name, data_path=args.data, entry=args.entry)
  moved = plan.notes.get('approach_m')
  if moved is not None and moved != SCENARIOS[args.name].approach_m:
    print(f"scenario {args.name}: spawn moved to {moved:.0f} m before the junction (from {SCENARIOS[args.name].approach_m:.0f}), " +
          f"clear of the mapped controls {plan.notes['controls_from_spawn']} (m from the spawn)", flush=True)
  run_dir = os.path.join(os.path.expanduser(args.record or args.out), f"{args.name}-{time.strftime('%Y%m%d-%H%M%S')}")
  os.makedirs(run_dir, exist_ok=True)

  from openpilot.tools.sim.bridge.beamng.bridge import BeamNGBridge
  cfg = config_from_args(bargs)
  cfg.spawn_pos, cfg.spawn_rot = plan.spawn, heading_quat(plan.heading)

  params = Params()
  token = ensure_token(params)
  for key in ('NavigationEnabled', 'TurnPilotEnabled'):
    if not params.get_bool(key):
      params.put_bool(key, True)
      print(f"set {key}: navd, turnpilotd and turnpilot_web run on it", flush=True)

  sweep_game(bargs.host, bargs.port)
  ui = start_ui_recording(os.path.join(run_dir, 'ui.mp4')) if args.record and not args.no_ui else None
  q: Queue = Queue()
  bridge = BeamNGBridge(cfg, scenario=Runner(plan, args.runs, run_dir, args.web.rstrip('/'), token, bargs.host, bargs.port,
                                             signal_tap=args.signal_tap == 'on', params=params))
  with light_oracle(bargs):
    drive(bridge, q, ui, bargs)
  result = os.path.join(run_dir, 'result.json')
  if args.record and not args.no_chase and os.path.exists(result):
    render_chases(result, run_dir, bargs.host, bargs.port)
  print(f"results: {result}")


def drive(bridge, q, ui, bargs):
  """The bridge runs the scenario until it's done or Ctrl-C; then the UI recording is closed and the game swept."""
  from openpilot.tools.sim.bridge.common import control_cmd_gen
  p = bridge.run(q)
  try:
    p.join()
  except KeyboardInterrupt:
    q.put(control_cmd_gen("quit"))
    p.join()
  finally:
    if ui is not None:
      try:
        ui.wait(timeout=3)  # a Ctrl-C reached it too and it is closing its mp4: don't interrupt that
      except subprocess.TimeoutExpired:
        ui.send_signal(signal.SIGINT)  # SIGINT lets the recorder finish the mp4
        try:
          ui.wait(timeout=30)
        except subprocess.TimeoutExpired:
          ui.kill()
    try:
      sweep_game(bargs.host, bargs.port)  # the bridge process may have died before its own cleanup
    except Exception as e:
      print(f"warning: the final cleanup sweep failed ({e!r}); run with --cleanup once the game answers", flush=True)


def render_chases(result: str, run_dir: str, host: str, port: int):
  """Each run's chase view, chase_run<N>.mp4 with its sidecar, once the bridge has let go of the game."""
  with open(result) as f:
    runs = json.load(f)['runs']
  for r in runs:
    if len(r.get('poses', [])) < 2:
      continue
    out = os.path.join(run_dir, f"chase_run{r['run']}.mp4")
    cmd = [sys.executable, '-m', 'openpilot.tools.sim.bridge.beamng.chase_render', result, '--run', str(r['run']), '-o', out,
           '--host', host, '--port', str(port)]
    if subprocess.run(cmd).returncode != 0:
      print(f"warning: the chase render of run {r['run']} failed; rerun: {' '.join(cmd)}", flush=True)


if __name__ == "__main__":
  main()
