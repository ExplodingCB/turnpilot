#!/usr/bin/env python3
"""Routed drives with the full stack: start on a route from routes.py, set the destination through turnpilot_web the
way the phone app does, engage, and let navd, turnpilotd and the model drive to arrival.

The bridge plays the driver: it pulls away and engages (--rolling-start), signals when turnpilotd prompts for a nav lane
change (driver.py), sets each leg's destination once the one before is reached, re-engages after a disengagement (each
one is logged with the events behind it), and brakes at the destination. Several runs share one bridge session. Per-run
stats come from the rlog afterwards (route_stats.py); this writes the run windows, legs and the path to --out.

Start openpilot with navd, turnpilotd and turnpilot_web (NavigationEnabled, TurnPilotEnabled, TurnPilotRouteSource),
then:
  python -m openpilot.tools.sim.bridge.beamng.route_drive --route loop --runs 3 --out loop.json --no-keyboard < /dev/null
"""
import argparse
import json
import math
import signal
import threading
import time
from multiprocessing import Queue

import numpy as np
import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.tools.sim.bridge.beamng import scenario
from openpilot.tools.sim.bridge.beamng.bridge import BeamNGBridge
from openpilot.tools.sim.bridge.beamng.frame import from_latlon
from openpilot.tools.sim.bridge.beamng.driver import GasTaps, ground_truth_green
from openpilot.tools.sim.bridge.beamng.gate1 import CANCEL_WAIT
from openpilot.tools.sim.bridge.beamng.pose import beamng_quat
from openpilot.tools.sim.bridge.beamng.routes import ROUTES
from openpilot.tools.sim.bridge.beamng.run_bridge import config_from_args, light_oracle, parse_args
from openpilot.tools.sim.bridge.common import control_cmd_gen

WEB = "http://127.0.0.1:8088"  # turnpilot_web (selfdrive/turnpilot/web/pairing.py PORT)
SETTLE = 4.0          # s after the teleport before the destination goes in
ROUTE_TIMEOUT = 20.0  # s for navd to publish a valid route
ENGAGE_TIMEOUT = 25.0
ARRIVE_M = 25.0       # navInstruction.distanceRemaining under this is the leg's arrival (navd drops a passed destination at 25 m)
STUCK_S = 45.0        # standing this long, not holding for anything, ends the run
MIN_SPEED = 2.5       # m/s, the slowest average a run may take before it times out
REENGAGE_WAIT = 2.0   # s after a disengagement before the driver engages again


def post(method, path, body=None):
  token = Params().get("TurnPilotWebToken")
  try:
    r = requests.request(method, WEB + path, json=body, headers={"Authorization": f"Bearer {token}"}, timeout=5)
    return r.status_code, r.text[:200]
  except requests.RequestException as e:
    return None, repr(e)


class RouteDrive:
  """Runs in the bridge process, called at 100 Hz (BeamNGBridge scenario hook)."""

  def __init__(self, routes: list, speed_kph: float, rolling_kph: float, traffic: int, out: str, signal_tap: bool = True):
    self.signal_tap = signal_tap
    """routes: the route of each run, in order."""
    self.plan, self.runs, self.speed_kph, self.rolling_kph, self.traffic, self.out = routes, len(routes), speed_kph, rolling_kph, traffic, out
    self.route = routes[0]
    self.screen = False  # end a run at its first reroute, and sooner when stuck
    self.results, self.run, self.phase, self.t_end = [], -1, "init", 0.0
    self.sm = None
    self.web: list = []

  def web_call(self, method, path, body=None):
    """turnpilot_web off the bridge loop; the answer lands in self.web."""
    def go():
      self.web.append((round(time.monotonic() - self.t0, 2), method, path, *post(method, path, body)))
    threading.Thread(target=go, daemon=True).start()

  def set_leg(self, i):
    lat, lon = self.route.legs[i]
    self.leg, self.leg_t, self.rem_min = i, time.monotonic(), float('inf')
    self.web_call("POST", "/api/destination", {"latitude": lat, "longitude": lon, "name": f"{self.route.name} leg {i + 1}"})
    print(f"route_drive: leg {i + 1}/{len(self.route.legs)} destination set", flush=True)

  def start_run(self, b) -> bool:
    """False when the session can't go on."""
    if Params().get("Offroad_ExcessiveActuation") is not None:
      # a collision while engaged latches this in selfdrived and nothing engages again until it restarts; the run
      # that collided is logged, and a new stack (route_drive.sh) picks up from the next run
      print(f"route_drive: excessive actuation latched, stopping before run {self.run + 2}", flush=True)
      return False
    self.run += 1
    self.route = self.plan[self.run]
    b.car.cancel()
    b.driver_brake = 0.0
    x, y, z = self.route.start
    with b.world.bng_lock:  # the road under the start, from just above the given height (tunnels and bridges cross)
      z = scenario.ground_z(b.world.bng, x, y, z + 4.0) + 0.3
    b.world.teleport((x, y, z), scenario.heading_quat(self.route.heading))
    self.t0, self.phase, self.leg = time.monotonic(), "settle", -1
    self.wall0 = time.time()  # noqa: TID251
    self.rows, self.disengagements, self.legs_done, self.web, self.poses = [], [], [], [], []
    self.engaged_once, self.enabled_prev, self.dis_t, self.still_t = False, False, None, 0.0
    self.dist, self.last_xy, self.rem_min = 0.0, None, float('inf')
    self.nav_lost_t = None
    self.taps, self.stuck_ref = GasTaps(self.signal_tap), None
    print(f"route_drive run {self.run + 1}/{self.runs}: {self.route.name} ({self.route.maneuvers})", flush=True)
    return True

  def __call__(self, b) -> bool:
    if self.sm is None:
      self.sm = messaging.SubMaster(['selfdriveState', 'onroadEvents', 'navInstruction', 'turnpilotPlan', 'carState', 'turnpilotLights'])
      if self.traffic:
        with b.world.bng_lock:
          scenario.remove_traffic(b.world.bng)  # leftovers from a session that died
          scenario.spawn_traffic(b.world.bng, self.traffic)
        print(f"route_drive: {self.traffic} traffic cars spawned", flush=True)
      self.phase, self.t_end, self.t0 = "next", time.monotonic(), time.monotonic()
      self.web_call("DELETE", "/api/destination")
    sm, s, now = self.sm, b.simulator_state, time.monotonic()
    sm.update(0)

    if self.phase == "next":
      if now - self.t_end < CANCEL_WAIT:
        return False
      if self.run + 1 >= self.runs or not self.start_run(b):
        self.finish(b)
        return True
      return False

    t = now - self.t0
    enabled = sm['selfdriveState'].enabled
    nav = sm['navInstruction']
    nav_ok = sm.valid['navInstruction'] and sm.recv_frame['navInstruction'] > 0
    if self.phase == "settle":
      if t > SETTLE:
        self.set_leg(0)
        self.phase = "route"
    elif self.phase == "route":
      if nav_ok and sm.updated['navInstruction'] and now - self.leg_t > 1.5:
        b.set_speed_kph, b.rolling_start_kph, b.auto_engage, self.phase = self.speed_kph, self.rolling_kph, True, "drive"
        self.route_t = t
      elif now - self.leg_t > ROUTE_TIMEOUT:
        self.end_run(b, "no route")
    elif self.phase == "drive":
      self.track(b, s, sm, t, enabled, nav, nav_ok)
    return False

  def track(self, b, s, sm, t, enabled, nav, nav_ok):
    xy = np.array([s.position.x, s.position.y])
    if self.last_xy is not None:
      self.dist += float(np.linalg.norm(xy - self.last_xy))
    self.last_xy = xy
    plan = sm['turnpilotPlan']

    if enabled and not self.engaged_once:
      self.engaged_once, self.engage_t = True, t
    if self.engaged_once and self.enabled_prev and not enabled:
      ev = [str(e.name) for e in sm['onroadEvents']]
      sd = sm['selfdriveState']
      d = {"t": round(t, 2), "x": round(float(xy[0]), 1), "y": round(float(xy[1]), 1), "v": round(s.wheel_speed, 2),
           "events": ev, "alert": str(sd.alertType), "alert_text": f"{sd.alertText1} {sd.alertText2}".strip(),
           "plan_state": str(plan.state), "plan_reason": plan.reason, "leg": self.leg + 1}
      self.disengagements.append(d)
      self.dis_t = t
      print(f"route_drive: DISENGAGED {json.dumps(d)}", flush=True)
      if "excessiveActuation" in ev:  # latched until selfdrived restarts: nothing engages again this session
        self.end_run(b, "excessive actuation")
        return
    self.enabled_prev = enabled
    if not enabled and self.dis_t is not None and t - self.dis_t > REENGAGE_WAIT and not b.auto_engage:
      b.auto_engage = True  # the driver takes it again and engages

    pose = b.world.pose
    if b.rk_frame % 5 == 0 and pose is not None:  # 20 Hz, to re-render a chase view offline: wall s, x y z m, BeamNG quat
      self.poses.append((round(time.time(), 3), round(t, 2), *(round(float(v), 3) for v in pose.pos),  # noqa: TID251
                         *(round(v, 5) for v in beamng_quat(pose.fwd, pose.up))))
    if b.rk_frame % 10 == 0:
      self.rows.append((round(t, 2), round(float(xy[0]), 2), round(float(xy[1]), 2), round(s.wheel_speed, 2), bool(enabled),
                        str(plan.state), str(plan.desire), str(plan.laneChange), str(plan.signalPrompt), plan.reason,
                        round(nav.maneuverDistance, 1) if nav_ok else None, f"{nav.maneuverType} {nav.maneuverModifier}" if nav_ok else "",
                        round(nav.distanceRemaining, 1) if nav_ok else None, self.leg + 1))

    if self.screen and self.dist > 30.0:  # off the route (navd invalid): missed, before it drives into anything
      self.nav_lost_t = None if nav_ok else (self.nav_lost_t or t)
      if self.nav_lost_t is not None and t - self.nav_lost_t > 1.5:
        self.end_run(b, "off route")
        return
    if nav_ok and time.monotonic() - self.leg_t > 10.0:
      if self.screen and nav.distanceRemaining > self.rem_min + 40.0:
        self.end_run(b, "rerouted")
        return
      self.rem_min = min(self.rem_min, nav.distanceRemaining)

    # arrival of this leg: navd's distance left, once the car is well under way on this leg
    if nav_ok and nav.distanceRemaining < ARRIVE_M and time.monotonic() - self.leg_t > 10.0 and self.dist > 50.0:
      self.legs_done.append({"leg": self.leg + 1, "t": round(t, 2), "x": round(float(xy[0]), 1), "y": round(float(xy[1]), 1)})
      if self.leg + 1 < len(self.route.legs):
        self.set_leg(self.leg + 1)
      else:
        self.end_run(b, "arrived")
        return

    holding = str(plan.state) == 'holding'
    # stuck: not 5 m from where it was STUCK_S ago, unless TurnPilot holds it (gas taps at a wall don't count as going)
    if self.stuck_ref is None or holding or float(np.linalg.norm(xy - self.stuck_ref[1])) > 5.0:
      self.stuck_ref = (t, xy)
    self.still_t = t - self.stuck_ref[0]
    # the driver's go (driver.GasTaps): at a stop sign TurnPilot holds past its dwell, at a signal it hasn't checked when
    # the light really is green, and standing with nothing held (the model stopped on its own): a gas tap, an override,
    # not a disengagement. Each logged with its reason and where
    n = len(self.taps.log)
    now = time.monotonic()
    b.driver_gas = self.taps.update(t, plan, enabled, s.wheel_speed, ground_truth_green(b.cfg, sm, now),
                                    plan_age_s=now - sm.recv_time['turnpilotPlan'])
    for tap in self.taps.log[n:]:
      tap.update(x=round(float(xy[0]), 1), y=round(float(xy[1]), 1), leg=self.leg + 1)
    timeout = SETTLE + ROUTE_TIMEOUT + 60.0 + route_length(self.route) / MIN_SPEED
    stuck_s = 25.0 if self.screen else STUCK_S
    if self.still_t > stuck_s or t > timeout:
      self.end_run(b, "stuck" if self.still_t > stuck_s else "timeout")

  def end_run(self, b, reason):
    b.auto_engage = False
    b.driver_gas = 0.0
    b.driver_brake = 1.0 if reason == "arrived" else 0.0  # the driver brakes at the destination, which also cancels cruise
    b.car.cancel()
    self.t_end = time.monotonic()
    self.web_call("DELETE", "/api/destination")
    t = self.t_end - self.t0
    res = {"run": self.run + 1, "route": self.route.name, "end": reason, "wall_start": self.wall0, "mono_start": self.t0,
           "mono_end": self.t_end, "duration_s": round(t, 1), "distance_m": round(self.dist, 1),
           "engaged_at_s": getattr(self, "engage_t", None), "legs_reached": self.legs_done, "legs": len(self.route.legs),
           "disengagements": self.disengagements, "stop_confirmations": self.taps.log, "lane_change_prompts_signalled": b.driver.signals,
           "turn_prompts_signalled": b.driver.turn_signals, "web": self.web,
           "traffic": self.traffic, "speed_kph": self.speed_kph, "path": self.rows,
           "pose_fields": ["wall_s", "t_s", "x", "y", "z", "qx", "qy", "qz", "qw"], "poses": self.poses}
    self.results.append(res)
    print(f"route_drive run {res['run']}: {reason} after {t:.0f} s, {self.dist:.0f} m, {len(self.disengagements)} disengagements, " +
          f"legs {len(self.legs_done)}/{len(self.route.legs)}", flush=True)
    with open(self.out, 'w') as f:
      json.dump({"route": self.plan[0].name, "runs": self.results}, f, indent=1)
    self.phase = "next"

  def quit(self, b):
    if self.phase in ("settle", "route", "drive"):
      self.end_run(b, "quit")
    self.finish(b)

  def finish(self, b):
    if self.traffic:
      with b.world.bng_lock:  # never leave AI cars in the shared game
        scenario.remove_traffic(b.world.bng)
      print("route_drive: traffic removed", flush=True)
    arrived = sum(r['end'] == 'arrived' for r in self.results)
    clean = sum(r['end'] == 'arrived' and not r['disengagements'] for r in self.results)
    print(f"route_drive: {arrived}/{len(self.results)} arrived, {clean} with no disengagement, results in {self.out}", flush=True)
    b.shutdown()


def route_length(route) -> float:
  """Straight-line length of the legs, a floor for the timeout (the drive is longer)."""
  pts = [route.start[:2]] + [from_latlon(lat, lon)[:2] for lat, lon in route.legs]
  return 2.0 * sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:], strict=True))


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--route', choices=sorted(ROUTES), default='short')
  ap.add_argument('--runs', type=int, default=1)
  ap.add_argument('--speed', type=float, default=40.0, help='cruise set speed, km/h')
  ap.add_argument('--rolling', type=float, default=20.0, help='rolling start speed before engaging, km/h')
  ap.add_argument('--traffic', type=int, default=0, help='BeamNG traffic cars to spawn (removed at the end)')
  ap.add_argument('--screen', action='store_true', help="one short drive per maneuver of the route instead (screen.py)")
  ap.add_argument('--only', default='', help='with --screen, only maneuvers whose name contains one of these (comma-separated)')
  ap.add_argument('--start-at', type=int, default=0, help='skip this many runs of the plan (a session cut short)')
  ap.add_argument('--out', default='route_drive.json')
  ap.add_argument('--signal-tap', choices=('on', 'off'), default='on',
                  help="off: the driver never gives the go at a signal TurnPilot hasn't checked (the stop-at-the-line runs)")
  args, rest = ap.parse_known_args()
  route = ROUTES[args.route]
  if args.screen:
    from openpilot.tools.sim.bridge.beamng.screen import screen_routes
    plan = [r for r in screen_routes(route) if not args.only or any(k in r.name for k in args.only.split(','))] * args.runs
  else:
    plan = [route] * args.runs
  plan = plan[args.start_at:]
  drive = RouteDrive(plan, args.speed, args.rolling, args.traffic, args.out, signal_tap=args.signal_tap == 'on')
  drive.screen = args.screen
  bargs = parse_args(rest)
  bargs.spawn, bargs.rot = plan[0].start, scenario.heading_quat(plan[0].heading)
  q: Queue = Queue()
  bridge = BeamNGBridge(config_from_args(bargs), scenario=drive)
  with light_oracle(bargs):
    p = bridge.run(q)
    # a background shell ignores SIGINT: SIGTERM (or SIGINT) ends the session cleanly, sensors and traffic removed
    for sig in (signal.SIGINT, signal.SIGTERM):
      signal.signal(sig, lambda *_: q.put(control_cmd_gen("quit")))
    try:
      p.join()
    except KeyboardInterrupt:
      q.put(control_cmd_gen("quit"))
      p.join()
    finally:
      if args.traffic:
        remove_traffic_after(bargs.host, bargs.port)


def remove_traffic_after(host, port):
  """Traffic cars out of the shared game even when the bridge died before finish()."""
  from beamngpy import BeamNGpy
  try:
    game = BeamNGpy(host, port).open(launch=False)
    try:
      scenario.remove_traffic(game)
      print("route_drive: traffic removed after the bridge", flush=True)
    finally:
      game.disconnect()
  except Exception as e:
    print(f"route_drive: traffic removal failed: {e!r}", flush=True)


if __name__ == "__main__":
  main()
