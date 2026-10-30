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

The demo's options (each run's result says what it did and when):
  --end-after-handback N   N s after TurnPilot hands back (a take-control prompt, turnpilotPlan.handback, a soft disable)
                           the sim driver brakes, as a driver taking over does: the run ends 'taken over', not on its timeout
  --gas-press-at [S]       the sim driver presses the gas once (GAS_PRESS for GAS_PRESS_S), S s after engagement
  --drop-detections-at [S] S s after engagement the sim's detector goes silent for DROP_S (DROP_PARAM, honoured only with
                           SIMULATION=1: modeld_detector.DetectorHost); the run clears it at its start and end
  --expect E               what the run must show (score.EXPECTS), in place of the scenario's own; the press implies
                           gasDisengage and the drop handback:detectorStopped (within score.DROP_HANDBACK_S of the
                           detector going silent after the drop: the rows' det_age, from turnpilotDetections)
Without S, the scenario's fault_at_s (catalog: straight_cruise's lands the fault short of its junction).

Each invocation writes tree.json in its directory before it touches the game and checks it again after the bridge exits
(tree.py): the stack's checkout (the running manager's for this OPENPILOT_PREFIX, or --stack-dir), its HEAD, uncommitted
edits, untracked sources and submodules (each read where the stack imports it from), whether that is clean, and whether
it changed during the runs. initData.dirty is set on every sim run here, so this is what says which code a run drove on.

A scenario with a forced light (catalog.junction_lights) has its junction's signal forced at each run's setup, after the
actors, and green green_after_s after the car first stands within GO_NEAR_M before the junction; the director logs both
(light_events), the rows carry the state (forced_light), and the sim driver's ground truth never reads it green while red.
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
import urllib.error
import urllib.request
from multiprocessing import Queue
from pathlib import Path

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params, UnknownKeyName
from openpilot.selfdrive.turnpilot.web.pairing import PORT
from openpilot.selfdrive.turnpilot.web.store import ensure_token
from openpilot.tools.sim.bridge.beamng import frame
from openpilot.tools.sim.bridge.beamng.driver import GasTaps, ground_truth_green
from openpilot.tools.sim.bridge.beamng.pose import beamng_quat
from openpilot.tools.sim.bridge.beamng.scenario import heading_quat
from openpilot.tools.sim.bridge.beamng.scenarios import score, tree
from openpilot.tools.sim.bridge.beamng.scenarios.catalog import DATA_JSON, ROLL_FRACTION, SCENARIOS, Plan, build, junction_distance
from openpilot.tools.sim.bridge.beamng.scenarios.director import Director, sweep
from openpilot.tools.sim.bridge.beamng.scenarios.timing import GO_NEAR_M, STOOD_V, GoWatch, step_out_distance

SETTLE_S = 4.         # after the teleport: the car is back in D and the GPS has caught up
CLEAR_TIMEOUT_S = 5.  # wait this long for navd to drop the old route
# TurnPilot is turned off at each run's start and on again this long after (turnpilotd reads TurnPilotEnabled once a second),
# before the destination goes out: the teleport back would leave its route follower dead-reckoning the last run's junction
# (gaps are dead-reckoned through) and its latch and left on it (BeamNG unprotected_left_gap6: a hold 13 m early)
TP_RESET_S = 2.5
ROUTE_TIMEOUT_S = 30.
# turnpilot_web refusing the connection (a fresh stack whose web server isn't listening yet: BeamNG smoke 2026-10-29, a run
# ended 'no destination' 5 s in): the destination call again this often, for ROUTE_TIMEOUT_S from its first try
WEB_RETRY_S = 1.
ENGAGE_TIMEOUT_S = 30.

TURN_DESIRES = (int(log.Desire.turnLeft), int(log.Desire.turnRight))
SERVICES = ['turnpilotPlan', 'selfdriveState', 'carState', 'navInstruction', 'navRoute', 'modelV2', 'turnpilotLights', 'longitudinalPlan',
            'turnpilotDetections']
OUT_ROOT = os.path.expanduser("~/turnpilot-artifacts/scenarios")
POSE_FIELDS = ["wall_s", "t_s", "x", "y", "z", "qx", "qy", "qz", "qw"]  # route_drive's, which chase_render.py reads
GAS_PRESS, GAS_PRESS_S = 0.3, 0.5  # --gas-press-at: the driver's press (a GasTaps tap is 0.25 for 1 s)
TAKEOVER_BRAKE = 1.0       # --end-after-handback: the driver's brake, which disengages (bridge.driver_brake)
SOFT_DISABLE_ALERT = '/softDisable'  # selfdriveState.alertType of a soft disable: '<event>/softDisable'
DROP_PARAM = 'TurnPilotSimDropDetectionsS'
# the detector silent this long (--drop-detections-at): past turnpilotd's 1.5 s deadline and openpilot's 3 s soft disable,
# so the disengagement completes and engaging again is refused while it lasts
DROP_S = 20.
LIGHTS_FRESH_S = 1.  # a row's lights: the light oracle's turnpilotLights this fresh (driver.LIGHTS_FRESH_S)
# --light-oracle: the oracle is a process of its own (run_bridge.light_oracle), which reads the game's signals once the level
# has loaded (light_oracle.STARTUP_TIMEOUT_S) and publishes at 4 Hz. A run waits this long from its teleport for its lights
# before the destination goes out, and ends 'error' without them, or once they are ORACLE_STALE_S old: a run asked for with
# the oracle never drives with every light unknown (BeamNG ctl_ped_in_path 2026-10-30, the oracle dead before the level loaded)
ORACLE_WAIT_S = 30.
ORACLE_STALE_S = 3.
# the speed check (score.speed_problems): the speed openpilot was given, and electrics.wheelspeed beside it, integrated over
# the pose log's travel from each run's teleport (the rolling start before engagement is where the wheels over-read most:
# BeamNG diag S02, 19.5%); a pose step longer than this in one 20 Hz row is the teleport, not travel, and one shorter than
# ODO_STAND_M with the speed at 0 both ends is a standing body's wander (smoke S14: 0.41 m over its 108 s at the red)
ODO_JUMP_M = 2.
ODO_STAND_M = 0.02
ACTOR_FIELDS = ["wall_s", "x", "y", "z", "heading"]


def wall() -> float:
  return time.time()  # noqa: TID251  the recordings are aligned on wall-clock time


def web_call(url: str, token: str, method: str, body: dict | None = None) -> dict:
  data = None if body is None else json.dumps(body).encode()
  req = urllib.request.Request(url, data=data, method=method, headers={'Authorization': f'Bearer {token}', 'Content-Type': 'application/json'})
  with urllib.request.urlopen(req, timeout=5) as r:
    return json.loads(r.read() or b'{}')


def refused(e: BaseException | None) -> bool:
  """A web call's exception is turnpilot_web not listening (yet)."""
  return isinstance(e, ConnectionRefusedError) or \
    (isinstance(e, urllib.error.URLError) and isinstance(getattr(e, 'reason', None), ConnectionRefusedError))


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
               params=None, end_after_handback: float = 0., gas_press_at: float | None = None,
               drop_detections_at: float | None = None, expect=None, light_oracle: bool = False):
    """params: Params, for TurnPilot's reset between runs (TP_RESET_S) and the detection drop; None leaves TurnPilot be.
    end_after_handback, gas_press_at, drop_detections_at: the demo's options (module docstring; a negative time: the
    scenario's fault_at_s); expect: in place of the plan's expectations (each option adds its own); light_oracle: the bridge
    runs --light-oracle, whose lights each run waits for and ends 'error' without (ORACLE_WAIT_S, ORACLE_STALE_S)."""
    if plan.road is None:
      raise ValueError(f"scenario {plan.name} has no road model: its runs could not be checked for the oncoming lane")
    if drop_detections_at is not None and params is None:
      raise ValueError("--drop-detections-at writes a param: the runner needs params")

    def at(t, option):
      if t is None or t >= 0.:
        return t
      if plan.fault_at_s <= 0.:
        raise ValueError(f"{option} without a time: scenario {plan.name} has no fault_at_s")
      return plan.fault_at_s
    self.end_after_handback = end_after_handback
    self.gas_press_at, self.drop_at = at(gas_press_at, '--gas-press-at'), at(drop_detections_at, '--drop-detections-at')
    implied = ('gasDisengage',) * (self.gas_press_at is not None) + ('handback:detectorStopped',) * (self.drop_at is not None)
    self.expect = tuple(dict.fromkeys((plan.expect if expect is None else tuple(expect)) + implied))
    self.handback = self.takeover = self.press = self.drop = None  # the demo options' (demo_options), per run
    self.press_until = -1.
    self.light_state: str | None = None  # the forced light's state now (None: no forced light)
    self.signals_running: bool | None = None  # the bridge runs --signals (from its config, at the first run)
    self.plan, self.runs, self.out_dir, self.web_url, self.token = plan, runs, out_dir, web_url, token
    self.signal_tap = signal_tap
    self.light_oracle = light_oracle
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
      self.odometer(b, now)
    if not self.director.is_alive() and self.phase not in ('next', 'done'):
      self.end_run(b, 'error', "the scenario director stopped (is the game reachable?)")
      self.finish(b)
      return True
    if (why := self.oracle_down(now)) is not None:  # the next run would have no oracle either
      self.end_run(b, 'error', why)
      self.finish(b)
      return True
    if self.phase == 'next':
      if self.run_idx + 1 >= self.runs:
        self.finish(b)
        return True
      self.start_run(b)
    elif self.phase == 'settle':
      lights_done = self.setup_lights is None or self.setup_lights.done()
      if now - self.t0 > max(SETTLE_S, TP_RESET_S) and self.setup.done() and lights_done and self.oracle_up(now):
        if self.tp_off_t is not None:  # TurnPilot back on, a fresh one, before the destination goes out (TP_RESET_S)
          self.params.put_bool('TurnPilotEnabled', True)
          self.tp_off_t = None
        if self.setup.exception() is not None:
          self.end_run(b, 'error', f"setup: {self.setup.exception()!r}")
        elif self.setup_lights is not None and self.setup_lights.exception() is not None:  # never a red the game didn't show
          self.end_run(b, 'error', f"the forced light: {self.setup_lights.exception()!r}")
        elif plan.dest is None:
          self.engage(b)
        else:
          self.web_send(now, 'DELETE')
          self.nav_cleared, self.phase, self.t_phase = False, 'clear', now
    elif self.phase == 'clear':
      # navd drops the old route within its 1 Hz tick; posting before it does could leave navd on the old one
      if self.web_retry(now):
        self.t_phase = now
      elif self.web.done() and refused(self.web.exception()):  # still refused after ROUTE_TIMEOUT_S: no POST will get through
        self.end_run(b, 'no destination', f"turnpilot_web at {self.web_url}: {self.web.exception()!r}")
      elif self.web.done() and (self.nav_cleared or now - self.t_phase > CLEAR_TIMEOUT_S):
        lat, lon, _ = frame.to_latlon(*plan.dest)
        self.web_send(now, 'POST', {'latitude': lat, 'longitude': lon, 'name': f"scenario {plan.name}"})
        self.route_pts, self.phase, self.t_phase = None, 'route', now
    elif self.phase == 'route':
      if self.web_retry(now):
        self.t_phase = now
      elif self.web.done() and self.web.exception() is not None:
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
      if not self.demo_options(b, r):
        return False
      self.driver(b)
      end = self.end_check.update(r)
      if end == 'disengaged' and self.takeover is not None:
        end = 'taken over'
      if end is not None:
        self.end_run(b, end)
    return False

  def oracle_up(self, now: float) -> bool:
    """The light oracle's lights are fresh, or it isn't asked for."""
    sm = self.sm
    return not self.light_oracle or (bool(sm.seen.get('turnpilotLights')) and now - sm.recv_time['turnpilotLights'] <= ORACLE_STALE_S)

  def oracle_down(self, now: float) -> str | None:
    """Why the run can't go on without the light oracle it asked for: none of its lights ORACLE_WAIT_S into the run's
    settle, or none for ORACLE_STALE_S since (module docstring); None while they come, or with no oracle asked for."""
    if self.oracle_up(now) or self.phase in ('next', 'done') or (self.phase == 'settle' and now - self.t0 <= ORACLE_WAIT_S):
      return None
    if not self.sm.seen.get('turnpilotLights'):
      return f"--light-oracle: no turnpilotLights from the light oracle {now - self.t0:.0f} s into the run (did it exit? its log says why)"
    return f"--light-oracle: the light oracle's last turnpilotLights is {now - self.sm.recv_time['turnpilotLights']:.1f} s old " + \
           "(it stopped publishing)"

  def web_send(self, now: float, method: str, body: dict | None = None):
    """The destination call to turnpilot_web, on the director's thread; web_retry repeats it while it is refused."""
    self.web_req, self.web_t, self.web_first_t = (method, body), now, now
    self.web = self.director.submit(web_call, f"{self.web_url}/api/destination", self.token, method, *([body] if body is not None else []))

  def web_retry(self, now: float) -> bool:
    """True while the last destination call was refused (refused: turnpilot_web not up yet) and is being retried,
    WEB_RETRY_S apart for ROUTE_TIMEOUT_S from its first try; it is sent again here when due. After that the refusal stands
    and the phase ends the run as before."""
    if not self.web.done() or not refused(self.web.exception()) or now - self.web_first_t > ROUTE_TIMEOUT_S:
      return False
    if now - self.web_t >= WEB_RETRY_S - 1e-6:
      method, body = self.web_req
      self.web_t = now
      self.web = self.director.submit(web_call, f"{self.web_url}/api/destination", self.token, method, *([body] if body is not None else []))
    return True

  def start_run(self, b):
    self.run_idx += 1
    plan = self.plan
    print(f"scenario {plan.name} run {self.run_idx + 1}/{self.runs}", flush=True)
    b.auto_engage = False
    b.car.cancel()
    b.blinker = (False, False)
    b.world.teleport(plan.spawn, heading_quat(plan.heading))
    self.setup = self.director.submit(self.director.setup, plan.actors)
    self.signals_running = not getattr(b.cfg, 'green', True)
    self.light_state = self.stand_t = self.green_t = self.setup_lights = None
    if plan.lights is not None:
      self.setup_lights = self.director.submit(self.director.set_lights, dict(plan.lights['states_at_setup']), not self.signals_running)
      self.light_state = plan.lights['state']
    self.t0, self.t0_wall, self.phase = time.monotonic(), wall(), 'settle'
    self.rows: list[dict] = []
    self.poses: list[tuple] = []
    self.actor_poses: dict[str, dict] = {}
    self.fired: set[str] = set()
    self.jobs: list = []  # trigger jobs: a failed one explains a failed run
    self.route_pts, self.route_id, self.nav_cleared = None, 0, False
    self.t_engaged = self.t_engaged_wall = None
    self.end_check = score.EndCheck(plan.timeout_s, plan.end_on_stop_s, plan.end_past_m)
    self.taps = GasTaps(self.signal_tap)
    self.watch = GoWatch()
    self.handback = self.takeover = self.press = self.drop = None  # the demo options' (demo_options)
    self.press_until = -1.
    self.step_out = None  # ped_step_out's walker: when she set off, the car's speed and distance, the stop it then needed
    self.odo = {'v_m': 0., 'wheel_m': None, 'travel_m': 0.}  # the speed check's integrals (odometer), from the teleport
    self.odo_last = None
    if self.params is not None:
      self.params.put_bool('TurnPilotEnabled', False)
      self.tp_off_t = time.monotonic()
    if self.drop_at is not None:
      self.clear_drop()

  def engage(self, b):
    b.set_speed_kph, b.rolling_start_kph, b.auto_engage = self.plan.speed_kph, self.plan.speed_kph * ROLL_FRACTION, True
    self.phase, self.t_phase = 'engage', time.monotonic()

  def row(self, b, now) -> dict:
    s, sm = b.simulator_state, self.sm
    x, y = s.position.x, s.position.y
    h = math.radians(90.0 - s.bearing)  # compass to CCW from east
    r = {
      't': round(now - self.t_engaged, 3), 'wall': round(wall(), 3), 'x': round(x, 2), 'y': round(y, 2), 'heading': round(h, 3),
      'v': round(s.wheel_speed, 2), 'v_wheel': None if (vw := getattr(s, 'wheel_speed_electrics', None)) is None else round(vw, 2),
      'route': self.route_id,
      'dest_dist': None if self.plan.dest is None else round(math.hypot(self.plan.dest[0] - x, self.plan.dest[1] - y), 1),
      'blinker': 'left' if b.blinker[0] else 'right' if b.blinker[1] else '',
      'ped_gap': None, 'car_gap': None, 'pacer_dx': None, 'j_dist': junction_distance(self.plan, x, y), 'car_j_dist': None,
      'onc': None, 'forced_light': self.light_state, 'lights': self.oracle_lights(now),
      'det_age': round(now - sm.recv_time['turnpilotDetections'], 3) if sm.seen['turnpilotDetections'] else None,
      **score.msg_fields(sm['selfdriveState'], sm['carState'], sm['turnpilotPlan'] if sm.seen['turnpilotPlan'] else None,
                         sm['navInstruction'] if sm.seen['navInstruction'] and sm.valid['navInstruction'] else None,
                         sm['modelV2'], sm['longitudinalPlan'] if sm.seen['longitudinalPlan'] else None),
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
      if a.trigger == 'pace':
        r['pacer_dx'] = round((pose[0] - x) * math.cos(h) + (pose[1] - y) * math.sin(h), 2)
    r['j_ahead'] = None
    if r['j_dist'] is not None:
      r['j_dist'] = round(r['j_dist'], 2)
      r['j_ahead'] = round((self.plan.junction[0] - x) * math.cos(h) + (self.plan.junction[1] - y) * math.sin(h), 2)
    return r

  def oracle_lights(self, now: float) -> list | None:
    """[[instance, state, m to its line], ...] of the light oracle's fresh turnpilotLights, else None."""
    sm = self.sm
    if not sm.seen.get('turnpilotLights') or now - sm.recv_time['turnpilotLights'] > LIGHTS_FRESH_S:
      return None
    return [[str(lt.instance), str(lt.state), round(float(lt.distance), 1)] for lt in sm['turnpilotLights'].lights]

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

  def odometer(self, b, now: float):
    """The speed check's integrals (module constants): the speed openpilot was given and the wheels' over the pose's travel
    (3D: the speed is along the road, and unprotected_left's ~21% hill reads 2.2% short on the level), a pose step over
    ODO_JUMP_M (the teleport) and a standing car's wander (under ODO_STAND_M with the speed at 0) left out."""
    pose, s = b.world.pose, b.simulator_state
    if pose is None:
      return
    xyz, v, vw = tuple(float(c) for c in pose.pos[:3]), abs(float(s.wheel_speed)), getattr(s, 'wheel_speed_electrics', None)
    last, self.odo_last = self.odo_last, (now, xyz, v, vw)
    if last is None:
      return
    step, dt = math.dist(xyz, last[1]), now - last[0]
    if step > ODO_JUMP_M or dt <= 0. or (v == 0. and last[2] == 0. and step < ODO_STAND_M):
      return
    o = self.odo
    o['travel_m'] += step
    o['v_m'] += dt * (v + last[2]) / 2.
    if vw is not None and last[3] is not None:
      o['wheel_m'] = (o['wheel_m'] or 0.) + dt * (abs(vw) + abs(last[3])) / 2.

  def speed_check(self) -> dict:
    """The run's speed check, for score.speed_problems: m the given speed and the wheels' integrate to, m of travel."""
    o = getattr(self, 'odo', None) or {'v_m': 0., 'wheel_m': None, 'travel_m': 0.}
    travel = o['travel_m']
    return {'v_m': round(o['v_m'], 1), 'wheel_m': None if o['wheel_m'] is None else round(o['wheel_m'], 1), 'travel_m': round(travel, 1),
            'ratio': round(o['v_m'] / travel, 3) if travel > 0. else None,
            'wheel_ratio': round(o['wheel_m'] / travel, 3) if travel > 0. and o['wheel_m'] is not None else None, 'from': 'teleport'}

  def triggers(self, b, r):
    """Set the actors going on the car at its junction (timing.GoWatch): the walker steps out on the car's go there, the
    oncoming cars go a fixed time after it stands at its hold there, so the first reaches the junction arrive_after_s
    after the go (or at once on a go that came sooner); the pacer goes once the car is engaged."""
    plan, d = self.plan, self.director
    if r['j_dist'] is not None:
      self.watch.update(r['t'], r['j_dist'], r['v'], r['state'] == 'holding', r['desire'] in TURN_DESIRES)
    self.light_trigger(r)
    w = self.watch
    for a in plan.actors:
      if a.name in self.fired or a.trigger == 'setup':
        continue
      go = False
      if a.trigger == 'walk':
        go, job = w.went, (d.walk, a.name)
      elif a.trigger == 'step_out':  # ped_step_out: once the car is step_out_distance from her, along its heading
        p0, h = a.path[0], r['heading']
        ahead = (p0[0] - r['x']) * math.cos(h) + (p0[1] - r['y']) * math.sin(h)
        go, job = r['v'] >= a.params['min_v'] and ahead <= step_out_distance(r['v'], a.params), (d.walk, a.name)
        if go:
          react = r['v'] * (a.params['enter_s'] + a.params['react_s'])
          self.step_out = {'t': r['t'], 'v': r['v'], 'ahead_m': round(ahead, 2),
                           'needs_decel': round(r['v'] ** 2 / (2. * max(ahead - react - a.params['gap_m'], 0.1)), 2)}
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

  def light_trigger(self, r):
    """The forced light's swap (module docstring): green_after_s after the car first stands within GO_NEAR_M before the
    junction, its phase green and the cross phase red."""
    lt = self.plan.lights
    if lt is None or self.green_t is not None or lt['green_after_s'] <= 0.:
      return
    if self.stand_t is None and r['v'] < STOOD_V and r['j_dist'] is not None and r['j_dist'] <= GO_NEAR_M and (r['j_ahead'] or 0.) > 0.:
      self.stand_t = r['t']
      print(f"scenario: the car stands at the forced {lt['state']} at t={r['t']:.1f} s; green in {lt['green_after_s']:.0f} s", flush=True)
    if self.stand_t is not None and r['t'] - self.stand_t >= lt['green_after_s'] - 1e-6:
      states = {i: ('red' if s == 'green' else 'green') for i, s in lt['states_at_setup'].items()}
      self.jobs.append(('lights', self.director.submit(self.director.set_lights, states, not self.signals_running)))
      self.green_t, self.light_state = r['t'], states[lt['instance']]
      print(f"scenario: the forced light goes {self.light_state} at t={r['t']:.1f} s", flush=True)

  def demo_options(self, b, r) -> bool:
    """The demo's options (module docstring) on this row. False: the run ended here."""
    t = r['t']
    if self.end_after_handback > 0. and self.takeover is None:
      cp, hb = r.get('control_prompt', 'none'), r.get('handback', 'none')
      what = hb if hb not in ('', 'none') else cp if cp.endswith('TakeControl') else \
             r['alert'].split('/')[0] if SOFT_DISABLE_ALERT in r.get('alert', '') else None
      if self.handback is None and what is not None:
        self.handback = {'t': t, 'what': what}
        print(f"scenario: TurnPilot handed back ({what}) at t={t:.1f} s; the driver takes over in {self.end_after_handback:.0f} s", flush=True)
      if self.handback is not None and t - self.handback['t'] >= self.end_after_handback - 1e-6:
        b.driver_brake = TAKEOVER_BRAKE
        self.takeover = {'t': t, 'after': self.handback['what'], 'handback_t': self.handback['t'], 'brake': TAKEOVER_BRAKE}
        print(f"scenario: the driver brakes at t={t:.1f} s, taking over", flush=True)
    if self.gas_press_at is not None and self.press is None and t >= self.gas_press_at - 1e-6:
      self.press, self.press_until = {'t': t, 'gas': GAS_PRESS, 's': GAS_PRESS_S}, time.monotonic() + GAS_PRESS_S
      print(f"scenario: the driver presses the gas at t={t:.1f} s", flush=True)
    if self.drop_at is not None and self.drop is None and t >= self.drop_at - 1e-6:
      try:
        self.params.put(DROP_PARAM, DROP_S)
      except UnknownKeyName:
        self.end_run(b, 'error', f"{DROP_PARAM} unknown to this build's params: rebuild the stack (scons) at the demoed tip")
        return False
      self.drop = {'t': t, 's': DROP_S, 'param': DROP_PARAM}
      print(f"scenario: the sim's detector goes silent for {DROP_S:.0f} s at t={t:.1f} s ({DROP_PARAM})", flush=True)
    return True

  def clear_drop(self):
    try:
      self.params.remove(DROP_PARAM)
    except UnknownKeyName:
      pass  # a build without it: the drop itself says so

  def driver(self, b):
    """The driver taps the stalk when TurnPilot asks for a signal (PLAN: Tesla can't signal for openpilot), and lets go
    when it stops asking, which it does as soon as the lane change starts. And taps the gas where a driver gives the go
    (driver.GasTaps): at a stop sign TurnPilot holds past its dwell, at a signal it hasn't checked when the light really
    is green, and standing with nothing held (the model not pulling away). Never on a stale plan or a handback."""
    plan = self.sm['turnpilotPlan'] if self.sm.seen['turnpilotPlan'] else None
    p = str(plan.signalPrompt) if plan is not None else 'none'
    b.blinker = (p == 'left', p == 'right')
    now = time.monotonic()
    green = False if self.light_state in ('red', 'yellow') else ground_truth_green(b.cfg, self.sm, now)  # a forced red: never
    gas = self.taps.update(now, plan, self.sm['selfdriveState'].enabled, b.simulator_state.wheel_speed, green,
                           plan_age_s=now - self.sm.recv_time['turnpilotPlan'])
    b.driver_gas = max(gas, GAS_PRESS if now < self.press_until else 0.)  # --gas-press-at's, over any tap

  def end_run(self, b, outcome: str, detail: str = ''):
    b.auto_engage = False
    b.driver_gas = b.driver_brake = 0.
    self.press_until = -1.
    if self.drop_at is not None:
      self.clear_drop()
    b.car.cancel()
    b.blinker = (False, False)
    end_wall = wall()
    failed = [f"{name}: {f.exception()!r}" for name, f in getattr(self, 'jobs', []) if f.done() and f.exception() is not None]
    detail = '; '.join([detail] * bool(detail) + failed)
    # the detection drop, for the score's timing of its handback; asked for and never set (the run ended first): said so
    drop = getattr(self, 'drop', None)
    drop = drop if drop is not None or self.drop_at is None else {'t': None, 'at': self.drop_at}
    res = score.summarize(self.plan.name, self.rows, outcome, road=self.plan.road, junction_control=self.plan.notes.get('junction_control'),
                          expect=self.expect, stop_line_m=self.plan.notes.get('stop_line_m'), drop=drop, speed=self.speed_check())
    # what the demo's options did, and when (t: s since engagement); the drop's is the score's detection_drop
    res.update(driver_takeover=getattr(self, 'takeover', None), gas_press=getattr(self, 'press', None),
               step_out=getattr(self, 'step_out', None))
    if self.plan.lights is not None:  # the forced light: what was set when (wall, from the director), and the swap's times
      res['light_events'] = [e for e in self.director.light_log if e[0] >= self.t0_wall - 1.]
      res['forced_light'] = {'instance': self.plan.lights['instance'], 'stand_t': getattr(self, 'stand_t', None),
                             'green_t': getattr(self, 'green_t', None), 'green_after_s': self.plan.lights['green_after_s']}
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
           'expect': list(self.expect), 'lights': plan.lights, 'signals_running': self.signals_running,
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
  ap.add_argument('--end-after-handback', type=float, default=0., metavar='N',
                  help="N s after TurnPilot hands back the driver brakes, taking over, and the run ends (0: stand to the timeout)")
  ap.add_argument('--gas-press-at', type=float, nargs='?', const=-1., default=None, metavar='S',
                  help="the driver presses the gas once, S s after engagement (no S: the scenario's fault_at_s)")
  ap.add_argument('--drop-detections-at', type=float, nargs='?', const=-1., default=None, metavar='S',
                  help=f"the sim's detector goes silent for {DROP_S:.0f} s, S s after engagement (SIMULATION=1 only; no S: fault_at_s)")
  ap.add_argument('--expect', action='append', default=None, metavar='E',
                  help=f"what the run must show, in place of the scenario's own: {', '.join(score.EXPECTS)} (handback:<kind>); repeatable")
  ap.add_argument('--stack-dir', default='', metavar='DIR',
                  help="the checkout the stack was launched from, for tree.json, when no running manager for OPENPILOT_PREFIX says")
  args, rest = ap.parse_known_args()

  from openpilot.tools.sim.bridge.beamng.run_bridge import config_from_args, light_oracle, parse_args
  bargs = parse_args(rest)
  if args.cleanup:
    sweep_game(bargs.host, bargs.port)
    return
  if args.name is None:
    ap.error("name a scenario")

  score.parse_expect(args.expect)  # an unknown expectation is refused before anything is written or the game is touched
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

  stack = tree.start(run_dir, os.environ.get('OPENPILOT_PREFIX', ''), option=args.stack_dir, git_commit=stack_commit(params))
  try:
    sweep_game(bargs.host, bargs.port)
    ui = start_ui_recording(os.path.join(run_dir, 'ui.mp4')) if args.record and not args.no_ui else None
    q: Queue = Queue()
    bridge = BeamNGBridge(cfg, scenario=Runner(plan, args.runs, run_dir, args.web.rstrip('/'), token, bargs.host, bargs.port,
                                               signal_tap=args.signal_tap == 'on', params=params, end_after_handback=args.end_after_handback,
                                               gas_press_at=args.gas_press_at, drop_detections_at=args.drop_detections_at, expect=args.expect,
                                               light_oracle=bargs.light_oracle))
    with light_oracle(bargs):
      drive(bridge, q, ui, bargs)
  finally:
    tree.finish(run_dir, stack)  # the end check: whatever ended the drive, tree.json says whether the code changed under it
  result = os.path.join(run_dir, 'result.json')
  if args.record and not args.no_chase and os.path.exists(result):
    render_chases(result, run_dir, bargs.host, bargs.port)
  print(f"results: {result}")


def stack_commit(params):
  """The manager's GitCommit param (manager_init writes its checkout's HEAD), for tree.json's cross-check; None unread."""
  try:
    return params.get('GitCommit')
  except Exception as e:
    print(f"tree.json: the GitCommit param could not be read ({e!r})", flush=True)
    return None


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
