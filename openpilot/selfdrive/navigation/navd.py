#!/usr/bin/env python3
import datetime
import os
import threading
import time

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import system_time_valid
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.route import along_path, local_meters
from openpilot.selfdrive.turnpilot.routing.controls import haversine
from openpilot.selfdrive.turnpilot.routing.guidance import (CONTROLS_AHEAD_M, CONTROLS_BEHIND_M, MAX_CONTROLS, control_msg, fill_controls,
                                                            fill_guidance)
from openpilot.selfdrive.turnpilot.maps.client import MapError
from openpilot.selfdrive.turnpilot.maps import surveys
from openpilot.selfdrive.turnpilot import stop_rules, visibility_table
from openpilot.selfdrive.turnpilot.routing import avoid, certify
from openpilot.selfdrive.turnpilot.routing.sources import MapRouter

REROUTE_DISTANCE = 25
OFF_ROUTE_DISTANCE = 20  # TurnPilot: further than this off the step, its distances mean nothing until the reroute
NEXT_STEP_NEAR = 6  # TurnPilot: this near the next step and further from this one by as much again, the car is on it
MANEUVER_TRANSITION_THRESHOLD = 10
REROUTE_COUNTER_MIN = 3
ROUTE_RESEND_PERIOD = 10  # updates (seconds); a restarted mapsd gets the route within this
RECOMPUTE_HOLD = 10  # updates (seconds) between route requests
SKIP_MATCH_M = 50.  # TurnPilot: a skipped turn is this step's maneuver if turnpilotd's distance to it is this close to navd's
OFF_ROUTE_MATCH_M = 20.  # TurnPilot: a lookup's control this close to one of the last lookup's, of its kind, is that one
# TurnPilot (stop signs, STOP_SIGNS.md 3.3): the avoidance loop's time bound, checked between its reroutes (navd sends invalid
# instructions while a route job runs, and a long loop on a reroute can trip off_route_unknown's 5 m/s cap). Measured
# 2026-10-28 on Kellen's Mac (Apple silicon; the comma four at 192.168.43.1 didn't answer): one iteration (a reroute with its
# exclusions, then the route's controls: trace_attributes and two batched locates) on the Indianapolis region tiles took a
# median 0.051 s, p90 0.074 s, max 0.080 s over 101 iterations on 36 trips. Allowing the comma four 6x the Mac's worst, all
# MAX_AVOID_ITER (8) iterations take 3.8 s.
AVOID_TIME_S = 4.0
AVOID_PARAM = 'TurnPilotAvoidStops'


def wall_clock() -> float | None:
  """TurnPilot: Unix time for the sun's position at a control's ETA, or None while the system clock isn't valid yet (before
  GPS time), when the certifier can't place the sun and fails the light check."""
  return datetime.datetime.now(datetime.UTC).timestamp() if system_time_valid() else None


def eta_of(steps, control, now: float | None) -> float | None:
  """Unix time the car starting the route steps now reaches control (its step and the m along it); None without a clock."""
  if now is None:
    return None
  t = sum(float(st.get('duration', 0.)) for st in steps[:control.step_idx])
  if 0 <= control.step_idx < len(steps):
    st = steps[control.step_idx]
    t += float(st.get('duration', 0.)) * min(max(control.along_step_m / max(float(st.get('distance', 0.)), 1.), 0.), 1.)
  return now + t


def set_banner(instruction, banners, distance_to_maneuver):
  """Fill the banner fields from the banner to show now: the last one still ahead of the ego, else the first."""
  if not banners:
    return
  banner = next((b for b in reversed(banners) if distance_to_maneuver < b['distanceAlongGeometry']), banners[0])
  instruction.showFull = distance_to_maneuver < banner['distanceAlongGeometry']
  primary = banner['primary']
  instruction.maneuverPrimaryText = primary.get('text') or ''
  instruction.maneuverType = primary.get('type') or ''
  instruction.maneuverModifier = primary.get('modifier') or ''
  if banner.get('secondary') is not None:
    instruction.maneuverSecondaryText = banner['secondary']['text']


class RouteJob(threading.Thread):
  """One router.route() call off navd's 1 Hz loop: a corridor download takes minutes over cellular. navd keeps
  publishing meanwhile and installs the result itself, on its own thread, if the destination is still the same.

  The router puts every failure in TurnPilotMapStatus; one that isn't a MapError is put there here."""

  def __init__(self, engine: 'RouteEngine', destination, language):
    super().__init__(name='navd-route', daemon=True)
    self.engine, self.destination, self.language = engine, destination, language
    self.start_position, self.start_bearing = engine.last_position, engine.last_bearing
    self.cancel = threading.Event()
    self.steps: list[dict] | None = None
    self.controls: list = []  # TurnPilot: the route's stop signs, yields and signals (turnpilot/routing/controls.py)
    self.controls_status = 'none'
    self.opposing: dict = {}  # TurnPilot: {step index: (m/s, lanes)} of the roads across its maneuvers (routing/controls.py)
    # TurnPilot (stop signs): the destination's exclusions, whether to avoid, the runtime inputs the route is certified
    # with, and what the avoidance loop did (routing/avoid.py)
    self.avoid = engine.avoid_set(destination)
    self.avoid_on = engine.avoid_stops_on()
    self.runtime = engine.runtime_inputs()
    self.avoid_status, self.avoided, self.avoid_added_s = 'none', 0, 0.

  def run(self):
    router, destination = self.engine.router, self.destination
    try:
      self.steps = router.route(self.start_position, destination, self.start_bearing, self.language,
                                position=self.engine.position, cancelled=self.cancel.is_set)
    except MapError:
      pass
    except Exception as e:
      cloudlog.exception("failed to get route")
      if not self.cancel.is_set():
        router.status.error(MapError('routing_failed', f"Routing failed: {type(e).__name__}: {e}", 'Routing failed'),
                            destination=list(destination))
    if self.steps:
      # TurnPilot: the route's controls, here off navd's loop (two Valhalla calls). Any failure is the status 'failed',
      # which turnpilotd says out loud, and the route still goes out
      try:
        self.controls, self.controls_status = router.controls(self.steps)
      except Exception:
        cloudlog.exception("navd: route controls failed")
        self.controls, self.controls_status = [], 'failed'
      # TurnPilot: every stop and yield the certifier can't certify is routed around (routing/avoid.py), here off navd's
      # loop; a failure keeps the route and says so (avoidStatus failed)
      try:
        self.avoid_stops(router)
      except Exception:
        cloudlog.exception("navd: stop-sign avoidance failed")
        self.avoid_status = 'failed'
      # the opposing roads' mapped speeds, for the unprotected left's required clear range: unknown on a failure (the left then
      # takes 45 mph and is skipped past the detector's range), never a guess lower
      try:
        self.opposing = router.opposing(self.steps)
      except Exception:
        cloudlog.exception("navd: route opposing roads failed")
        self.opposing = {}


  def avoid_stops(self, router):
    engine, destination, language = self.engine, self.destination, self.language
    now = engine.clock()

    def certify_one(control, steps):
      eta = eta_of(steps, control, now)
      result = engine.certifier(control, eta, self.runtime)
      control.eta_window = (eta - self.runtime.window[0], eta + self.runtime.window[1]) if result[0] == 'certifiable' and eta is not None else None
      return result

    def reroute(exclude):
      if not hasattr(router, 'reroute'):
        return None
      position, bearing = engine.position()
      return router.reroute(position, destination, bearing, language, exclude)

    def own_edge():
      position, bearing = engine.position()
      return router.own_edge(position, bearing) if hasattr(router, 'own_edge') and position is not None else None

    def headings(points):
      return router.exclusion_headings(points) if hasattr(router, 'exclusion_headings') else [None for _ in points]

    result = avoid.run(self.steps, self.controls, self.controls_status, self.avoid, certify=certify_one, reroute=reroute,
                       route_controls=router.controls, own_edge=own_edge, cancelled=self.cancel.is_set,
                       deadline=engine.monotonic() + AVOID_TIME_S, monotonic=engine.monotonic, enabled=self.avoid_on, headings=headings)
    self.steps, self.controls, self.controls_status = result.steps, result.controls, result.controls_status
    self.avoid_status, self.avoided, self.avoid_added_s = result.status, result.avoided, result.added_s


class AheadJob(threading.Thread):
  """TurnPilot: one router.controls_ahead() off navd's loop (a few Valhalla calls; a valhalla_service's over HTTP can take
  seconds): the stop signs, yields and signals on the road ahead of the car, off the route. Any failure is the status
  'failed', never no controls."""

  def __init__(self, router, position, bearing):
    super().__init__(name='navd-ahead', daemon=True)
    self.router, self.position, self.bearing = router, position, bearing
    self.result: tuple = ([], 'failed', [])

  def run(self):
    try:
      self.result = self.router.controls_ahead(self.position, self.bearing)
    except Exception:
      cloudlog.exception("navd: controls ahead failed")


class RouteEngine:
  def __init__(self, sm, pm, params=None, router=None, background=True, clock=wall_clock, monotonic=time.monotonic, certifier=None):
    self.sm = sm
    self.pm = pm
    self.params = params if params is not None else Params()
    # TurnPilot (stop signs, STOP_SIGNS.md 3.2-3.3): the wall clock (the sun's position at each control's ETA), the monotonic
    # one (AVOID_TIME_S), the certifier (certifier(control, eta, runtime) -> (certification, reason); routing/certify.py on
    # maps/visibility.json and maps/surveyed_junctions.json by default), the exclusions per destination, what the last
    # route's avoidance did, the runtime inputs it was certified with, and a re-run pending (an ETA left its window)
    self.clock, self.monotonic = clock, monotonic
    self.certifier = certifier if certifier is not None else self.certify_default
    self.avoid_sets: dict = {}
    self.avoid_status, self.avoided, self.avoid_added_s = 'none', 0, 0.
    self.route_runtime = None
    self.avoid_rerun = False

    self.last_position = None  # (longitude, latitude)
    self.last_bearing = None

    self.nav_destination = None
    self.step_idx = None
    self.route = None
    self.route_geometry = None  # an (n, 2) array of (longitude, latitude) per step
    self.route_count = 0  # TurnPilot: routes calculated, so a replaced route is told apart from the one it replaces
    self.controls: list = []  # TurnPilot: the route's stop signs, yields and signals, and Valhalla's status for them
    self.controls_status = 'none'
    self.opposing: dict = {}  # TurnPilot: the route's opposing roads' speeds and lanes (RouteJob.opposing)
    # TurnPilot: off the route with a destination, the controls ahead of the car (AheadJob), [(id, Control)] on the road
    # looked up (an (n, 2) array of (longitude, latitude)), and the lookup's status: navInstruction.offRouteControls
    self.off_controls: list = []
    self.off_road = None
    self.off_status = 'none'
    self.off_next_id = 0
    self.ahead_job: AheadJob | None = None

    self.recompute_hold = 0
    self.reroute_counter = 0
    self.maneuver_distance = None  # TurnPilot: m to the current step's maneuver, as last sent
    self.skipped = None  # TurnPilot: (route_count, step_idx) of a turn turnpilotd skipped (too fast): reroute once past it
    self.missed_seen = False  # TurnPilot: the last turnpilotPlan said a turn was missed (turnMissed)
    self.missed = False  # ...and it newly did: reroute at once, as past a skipped turn

    # TurnPilot: routes are computed by a RouteJob, in a thread unless background is False (tests that step navd)
    self.background = background
    self.job: RouteJob | None = None
    self.requested_destination = None  # the destination of the last route request, which a new one skips the hold for

    # TurnPilot: routes come from TurnPilot's map server (or local tiles), never Mapbox; see turnpilot/routing/sources.py
    self.router = router if router is not None else MapRouter(self.params)

  def update(self):
    self.sm.update(0)

    source = latest_gps_source(self.sm)
    if source is not None:
      gps = self.sm[source]
      self.last_position, self.last_bearing = (gps.longitude, gps.latitude), gps.bearingDeg
    self.note_skip()
    route, route_count = self.route, self.route_count
    try:
      self.recompute_route(gps_ok=source is not None)
      if route is not None and self.route is not None and self.route_count != route_count:
        # TurnPilot: one invalid instruction between two routes, so no maneuver (and its turn desire) carries over to
        # a new route that starts with one of the same type
        msg = messaging.new_message('navInstruction', valid=False)
        msg.navInstruction.routeCount = self.route_count
        self.fill_off_route(msg)
        self.pm.send('navInstruction', msg)
      else:
        self.send_instruction()
    except Exception:
      cloudlog.exception("navd.failed_to_compute")
    # mapsd dedupes repeats, so a periodic resend is how a restarted mapsd gets the route
    if self.route is not route or self.sm.frame % ROUTE_RESEND_PERIOD == 0:
      self.send_route()

  def note_skip(self):
    """TurnPilot: turnpilotd skipped the turn ahead (too fast to make it; turnpilot/turn_skip.py). navd doesn't move on
    to the turn's step once past the junction, it reroutes from there at once (should_recompute)."""
    if 'turnpilotPlan' not in self.sm.data or not self.sm.updated['turnpilotPlan'] or not self.sm.valid['turnpilotPlan']:
      return
    plan = self.sm['turnpilotPlan']
    # a turn from a standstill that went straight past its junction (turnpilotd: turnMissed, held about 3 s): navd has
    # usually moved on to the next step by then (10 m past the step's end, before turnpilotd's pass at 15 m), so it is
    # matched to no step: rerouted at once, once, unless the car is on the step navd is on (it turned after all)
    missed = plan.turnMissed and not plan.shadow
    if missed and not self.missed_seen:
      self.missed = True
      cloudlog.event("navd.turn_missed", step_idx=self.step_idx)
    self.missed_seen = missed
    # a shadow plan is for scoring drives: nothing acts on it (the human drives, and may brake and take the turn)
    if plan.shadow or not plan.tooFastToTurn or self.step_idx is None or self.maneuver_distance is None:
      return
    if abs(plan.maneuverDistance - self.maneuver_distance) < SKIP_MATCH_M and self.skipped != (self.route_count, self.step_idx):
      self.skipped = (self.route_count, self.step_idx)
      cloudlog.event("navd.turn_skipped", step_idx=self.step_idx, maneuver_distance=round(self.maneuver_distance, 1))

  def skip_here(self) -> bool:
    return self.skipped == (self.route_count, self.step_idx)

  def position(self):
    """(position, bearing) now, for a route job that finishes a download."""
    return self.last_position, self.last_bearing

  def avoid_set(self, destination) -> avoid.AvoidSet:
    """TurnPilot: the stop-sign exclusions for destination, kept across every route to it; a new destination clears them."""
    if destination not in self.avoid_sets:
      self.avoid_sets = {destination: avoid.AvoidSet()}
    return self.avoid_sets[destination]

  def avoid_stops_on(self) -> bool:
    """TurnPilot: TurnPilotAvoidStops (on unless set off; step S3 sets it off to keep the junctions it validates). A build
    whose params don't know it avoids, said."""
    try:
      value = self.params.get(AVOID_PARAM)
    except Exception as e:
      cloudlog.warning(f"navd: {AVOID_PARAM} unreadable ({type(e).__name__}): avoiding stops")
      return True
    if isinstance(value, bytes):
      value = value.decode(errors='replace')
    return value not in (False, 0, '0', 'false', 'False', 'off')

  def runtime_inputs(self) -> certify.Runtime:
    """TurnPilot: what the certifier reads at route time besides the files: the mode turnpilotd acts in and whether its side
    views are live (turnpilotPlan.stopGo, SG4a; until it publishes them, the mode from TurnPilotStopSignGo and the side views
    off), and TurnPilotStopLaunch."""
    mode, side = certify.stop_sign_mode(self.params), False
    if 'turnpilotPlan' in self.sm.data and self.sm.valid.get('turnpilotPlan', False):
      try:
        go = self.sm['turnpilotPlan'].stopGo
        side, mode = bool(go.sideViewsLive), str(go.mode) or mode
      except Exception:
        pass
    return certify.Runtime(mode=mode, side_views=side, a_launch=stop_rules.a_launch_plan(self.params))

  @staticmethod
  def runtime_key(runtime: certify.Runtime) -> tuple:
    return runtime.mode, runtime.side_views, runtime.a_launch

  def certify_default(self, control, eta, runtime):
    """TurnPilot: routing/certify.py on the shipped visibility and survey files (a sim table only where the sim says so)."""
    visibility = visibility_table.cached(visibility_table.SHIPPED, sim_ok=self.sim_tables_ok())
    return certify.certify_control(control, visibility, surveys.cached(surveys.SHIPPED).entries, eta, runtime)

  def sim_tables_ok(self) -> bool:
    """TurnPilot: a sim visibility table is read only with SIMULATION=1, TurnPilotSimLightOracle and a sim car."""
    if os.environ.get('SIMULATION') != '1':
      return False
    car_params = None
    try:
      from openpilot.cereal import car
      raw = self.params.get('CarParams')
      if raw:
        with car.CarParams.from_bytes(raw) as cp:
          car_params = cp
      return visibility_table.sim_accepted(os.environ, self.params.get(visibility_table.SIM_PARAM), car_params)
    except Exception:
      cloudlog.exception("navd: the sim's CarParams don't read: no sim visibility table")
      return False

  def avoid_rerun_due(self) -> bool:
    """TurnPilot: the stop-sign avoidance must run again for this route (STOP_SIGNS.md 3.2): a runtime input it was
    certified with changed (N11), or a certified control's ETA left the window its light check passed for (N14). Once."""
    if self.route is None or self.job is not None:
      return False
    if self.avoid_rerun:
      self.avoid_rerun = False
      return True
    key = self.runtime_key(self.runtime_inputs())
    if self.route_runtime is not None and key != self.route_runtime:
      cloudlog.event("navd.stop_runtime_changed", was=list(self.route_runtime), now=list(key))
      self.route_runtime = key
      return True
    return False

  def check_eta_drift(self, car_along: float) -> None:
    """TurnPilot: N14, each certified control still ahead: its ETA now (the route's durations, as timeRemaining) outside the
    window its light check passed for queues the avoidance loop again (a re-run route certifies at the new ETAs)."""
    now = self.clock()
    if self.avoid_rerun or self.route is None or now is None:
      return
    t_car = avoid.time_at(self.route, car_along)
    for c in self.controls:
      window = getattr(c, 'eta_window', None)
      if window is None or c.route_m < car_along:
        continue
      eta = now + avoid.time_at(self.route, c.route_m) - t_car
      if not window[0] <= eta <= window[1]:
        cloudlog.event("navd.stop_eta_drift", control=c.id, eta=round(eta, 1), window=[round(w, 1) for w in window])
        self.avoid_rerun = True
        return

  def recompute_route(self, gps_ok: bool):
    if self.last_position is None:
      return

    destination = self.params.get("NavDestination")
    new_destination = None if destination is None else (destination['longitude'], destination['latitude'])
    self.collect_route(new_destination)
    if new_destination != self.nav_destination and self.route is not None:
      self.clear_route()  # never guidance to a destination that was replaced
    if self.job is not None:
      if self.job.destination == new_destination:
        return  # still routing to it
      # a new destination (or none) cancels the job and doesn't wait for it: a download can't stop while the server is
      # still building its corridor. The job is left to finish on its own, and the router drops what it brings back.
      self.job.cancel.set()
      self.job = None

    if new_destination is None:
      self.clear_route()
      self.router.clear()
      self.recompute_hold = 0
      self.requested_destination = None
      return

    # Don't recompute when GPS drifts in tunnels
    if not gps_ok and self.step_idx is not None:
      return

    self.recompute_hold -= 1
    skipped_past = self.skipped_turn_passed() or self.missed_off_route()  # TurnPilot: a reroute past a skipped or missed
                                                                          # turn waits for nothing
    rerun = self.avoid_rerun_due()  # TurnPilot: nor does a stop-sign certification that changed
    if skipped_past or rerun or new_destination != self.requested_destination or (self.should_recompute() and self.recompute_hold <= 0):
      self.recompute_hold = RECOMPUTE_HOLD
      self.calculate_route(new_destination)
      self.reroute_counter = 0

  def calculate_route(self, destination):
    """Start routing to destination. In the background the route arrives in a later update(); else it is here now."""
    assert self.last_position is not None
    cloudlog.warning(f"Calculating route {self.last_position} -> {destination}")
    self.requested_destination = destination

    lang = self.params.get('LanguageSetting')
    if lang is not None:
      lang = lang.replace('main_', '')

    self.skipped = None
    # TurnPilot: the router checks the geofence, downloads the drive's map and routes on it; any failure raises a
    # MapError that it has already put in TurnPilotMapStatus for the web app and the UI, and there is no route
    self.job = RouteJob(self, destination, lang)
    if self.background:
      self.job.start()
    else:
      self.job.run()
      self.collect_route(destination)

  def collect_route(self, destination):
    """Take a finished job's route, if it is for destination (the current NavDestination) and wasn't cancelled."""
    job = self.job
    if job is None or job.is_alive():
      return
    self.job = None
    if job.cancel.is_set() or job.destination != destination:
      cloudlog.warning(f"navd: dropped the route to {job.destination}, the destination is now {destination}")
      return
    if job.steps is None:
      self.clear_route()
      return
    self.route = job.steps
    self.controls, self.controls_status = job.controls, job.controls_status
    self.opposing = job.opposing
    self.avoid_status, self.avoided, self.avoid_added_s = job.avoid_status, job.avoided, job.avoid_added_s
    self.route_runtime, self.avoid_rerun = self.runtime_key(job.runtime), False
    self.route_count += 1
    self.route_geometry = [np.array(step['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for step in self.route]
    self.step_idx = 0
    self.nav_destination = job.destination

  def send_instruction(self):
    msg = messaging.new_message('navInstruction', valid=True)
    # TurnPilot: on every message, so an invalid one for this route (off it for a fix, rerouting) is told apart from a
    # new route, which a turn under way isn't held through (turnpilot/route_follower.py)
    msg.navInstruction.routeCount = self.route_count
    # and the controls' status on every message too; an invalid one carries no controls
    msg.navInstruction.controlsStatus = self.controls_status if self.route is not None else 'none'
    # and what the stop-sign avoidance did for this route (routing/avoid.py)
    msg.navInstruction.avoidStatus = self.avoid_status if self.route is not None else 'none'
    msg.navInstruction.avoidedControls = min(self.avoided, 65535) if self.route is not None else 0

    # TurnPilot: a job while there is a route is a reroute for the same destination (a new one clears the route), as when
    # the car left the corridor and a new one downloads for minutes. The car is off this route meanwhile: its distances
    # mean nothing, and following them would run through its steps to 'Destination reached' and delete the destination.
    if self.step_idx is None or self.job is not None:
      msg.valid = False
      self.fill_off_route(msg)
      self.pm.send('navInstruction', msg)
      return

    assert self.route is not None and self.route_geometry is not None and self.last_position is not None
    step = self.route[self.step_idx]
    along_geometry, lateral = along_path(self.route_geometry[self.step_idx], self.last_position)
    distance_to_maneuver_along_geometry = step['distance'] - along_geometry
    # a sharp turn leaves the step sideways, and along it the car never gets past its end: it moved on once it is on
    # the next step's geometry and off this one's (otherwise navd rerouted after every such turn)
    if self.on_next_step(lateral, distance_to_maneuver_along_geometry):
      self.step_idx += 1
      self.recompute_hold = 0
      return self.send_instruction()
    # a missed turn: no instruction until rerouted (past the step's end the lateral is the distance from it, and it advances)
    if lateral > OFF_ROUTE_DISTANCE and distance_to_maneuver_along_geometry >= -MANEUVER_TRANSITION_THRESHOLD and \
       self.step_idx < len(self.route) - 1:
      msg.valid = False
      self.fill_off_route(msg)
      self.pm.send('navInstruction', msg)
      return
    self.reset_off_route()

    # Banner instructions are for the following maneuver step, don't use empty last step
    banner_step = step
    if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
      banner_step = self.route[max(self.step_idx - 1, 0)]

    # Current instruction
    self.maneuver_distance = distance_to_maneuver_along_geometry
    msg.navInstruction.maneuverDistance = distance_to_maneuver_along_geometry
    set_banner(msg.navInstruction, banner_step['bannerInstructions'], distance_to_maneuver_along_geometry)
    fill_guidance(msg.navInstruction, self.route, self.step_idx, distance_to_maneuver_along_geometry, self.opposing)
    # TurnPilot: the controls in maneuverDistance's frame, the steps' distances and then along this step's geometry
    car_along = sum(s['distance'] for s in self.route[:self.step_idx]) + along_geometry
    fill_controls(msg.navInstruction, self.controls, self.controls_status, car_along)
    self.check_eta_drift(car_along)

    # Total remaining distance and time: the rest of this step, then every step after it
    remaining = 1.0 - along_geometry / max(step['distance'], 1)
    future = self.route[self.step_idx + 1:]
    msg.navInstruction.distanceRemaining = step['distance'] * remaining + sum(s['distance'] for s in future)
    msg.navInstruction.timeRemaining = step['duration'] * remaining + sum(s['duration'] for s in future)

    self.pm.send('navInstruction', msg)

    # Transition to next route segment
    if distance_to_maneuver_along_geometry < -MANEUVER_TRANSITION_THRESHOLD:
      if self.skip_here():
        pass  # TurnPilot: past a skipped turn the car is off route; the next update reroutes, the turn's step never starts
      elif self.step_idx + 1 < len(self.route):
        self.step_idx += 1
        self.recompute_hold = 0
      else:
        cloudlog.warning("Destination reached")

        # Clear route if driving away from destination
        if np.linalg.norm(local_meters(self.last_position, self.nav_destination)) > REROUTE_DISTANCE:
          self.params.remove("NavDestination")
          self.clear_route()

  def fill_off_route(self, msg):
    """TurnPilot: an invalid navInstruction's offRouteControls and offRouteControlsStatus. With a destination (rerouting, no
    route yet, off it for a fix), the last lookup ahead of the car, measured from where the car is now along the road it
    looked at, and a new lookup started (one at a time, off this loop). pending until the first is in; none with no
    destination, and nothing looked up."""
    if self.requested_destination is None and self.nav_destination is None:
      self.reset_off_route()
      return
    if self.off_status == 'none':
      self.off_status = 'pending'
    self.collect_ahead()
    if self.ahead_job is None and self.last_position is not None:
      self.ahead_job = AheadJob(self.router, self.last_position, self.last_bearing)
      if self.background:
        self.ahead_job.start()
      else:
        self.ahead_job.run()
        self.collect_ahead()
    ni = msg.navInstruction
    ni.offRouteControlsStatus = self.off_status
    if self.off_road is None or self.last_position is None or not self.off_controls:
      return
    along, _ = along_path(self.off_road, self.last_position)
    near = sorted(((c.route_m - along, i, c) for i, c in self.off_controls if -CONTROLS_BEHIND_M < c.route_m - along < CONTROLS_AHEAD_M),
                  key=lambda dic: dic[0])[:MAX_CONTROLS]
    ni.offRouteControls = [control_msg(c, d, i) for d, i, c in near]

  def collect_ahead(self):
    """TurnPilot: a finished lookup ahead becomes the off-route controls; one the last lookup had (its kind, its stop point
    within OFF_ROUTE_MATCH_M) keeps its id, so turnpilotd tracks it as the same control."""
    job = self.ahead_job
    if job is None or job.is_alive():
      return
    self.ahead_job = None
    controls, status, road = job.result
    if len(road) < 2:
      self.off_controls, self.off_road, self.off_status = [], None, status
      return
    old, out = list(self.off_controls), []
    for c in controls:
      same = [(haversine(o.point, c.point), i) for i, o in old if o.kind == c.kind and o.point is not None and c.point is not None and
              haversine(o.point, c.point) < OFF_ROUTE_MATCH_M and all(i != j for j, _ in out)]
      if same:
        i = min(same)[1]
      else:
        i, self.off_next_id = self.off_next_id, (self.off_next_id + 1) & 0xFFFF
      out.append((i, c))
    self.off_controls, self.off_status = out, status
    self.off_road = np.array([[lon, lat] for lat, lon in road], dtype=np.float64)

  def reset_off_route(self):
    """TurnPilot: on a route, or no destination: nothing off it. A lookup under way is left to finish, and dropped."""
    self.off_controls, self.off_road, self.off_status, self.ahead_job = [], None, 'none', None

  def send_route(self):
    coords = np.concatenate(self.route_geometry) if self.route_geometry is not None else np.empty((0, 2))
    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = [{'longitude': lon, 'latitude': lat} for lon, lat in coords.tolist()]
    self.pm.send('navRoute', msg)

  def clear_route(self):
    self.route = None
    self.route_geometry = None
    self.step_idx = None
    self.nav_destination = None

  def on_next_step(self, lateral: float, distance_to_maneuver: float) -> bool:
    """The car is on the next step's geometry and off this one's: a sharp turn leaves the step sideways, and along it the
    car never gets past its end."""
    assert self.route is not None and self.route_geometry is not None and self.step_idx is not None
    if self.step_idx + 1 >= len(self.route) or distance_to_maneuver >= MANEUVER_TRANSITION_THRESHOLD:
      return False
    _, next_lateral = along_path(self.route_geometry[self.step_idx + 1], self.last_position)
    return next_lateral < NEXT_STEP_NEAR and lateral > next_lateral + NEXT_STEP_NEAR

  def skipped_turn_passed(self) -> bool:
    """TurnPilot: the car is past the junction of a turn turnpilotd skipped. If it took the turn after all (on the next
    step's road), it moves on to that step instead: a reroute from a spot on the route replaces it for nothing."""
    if not self.skip_here() or self.step_idx is None or self.last_position is None:
      return False
    assert self.route is not None and self.route_geometry is not None
    along, lateral = along_path(self.route_geometry[self.step_idx], self.last_position)
    distance_to_maneuver = self.route[self.step_idx]['distance'] - along
    if self.on_next_step(lateral, distance_to_maneuver):
      cloudlog.event("navd.skipped_turn_taken", step_idx=self.step_idx)
      self.skipped = None
      self.step_idx += 1
      self.recompute_hold = 0
      return False
    return distance_to_maneuver < -MANEUVER_TRANSITION_THRESHOLD

  def missed_off_route(self) -> bool:
    """TurnPilot: turnpilotd said a turn was missed (note_skip), and the car is off the step navd is on: reroute now."""
    if not self.missed:
      return False
    self.missed = False
    if self.step_idx is None or self.last_position is None or self.route_geometry is None:
      return False
    return along_path(self.route_geometry[self.step_idx], self.last_position)[1] > NEXT_STEP_NEAR

  def should_recompute(self):
    if self.step_idx is None:
      return True

    assert self.route is not None and self.route_geometry is not None and self.last_position is not None
    # Don't recompute in last segment, assume destination is reached
    if self.step_idx == len(self.route) - 1:
      return False

    # TODO: Check for going wrong way in segment
    if along_path(self.route_geometry[self.step_idx], self.last_position)[1] > REROUTE_DISTANCE:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0
    return self.reroute_counter > REROUTE_COUNTER_MIN


def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute'])
  # TurnPilot: turnpilotPlan says when a turn was skipped (too fast), so navd reroutes past it at once
  sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal', 'turnpilotPlan'], ignore_alive=['turnpilotPlan'],
                           ignore_avg_freq=['turnpilotPlan'], ignore_valid=['turnpilotPlan'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngine(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
