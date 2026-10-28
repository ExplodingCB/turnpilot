#!/usr/bin/env python3
import threading

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.route import along_path, local_meters
from openpilot.selfdrive.turnpilot.routing.controls import haversine
from openpilot.selfdrive.turnpilot.routing.guidance import CONTROLS_AHEAD_M, CONTROLS_BEHIND_M, MAX_CONTROLS, fill_controls, fill_guidance
from openpilot.selfdrive.turnpilot.maps.client import MapError
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
  def __init__(self, sm, pm, params=None, router=None, background=True):
    self.sm = sm
    self.pm = pm
    self.params = params if params is not None else Params()

    self.last_position = None  # (longitude, latitude)
    self.last_bearing = None

    self.nav_destination = None
    self.step_idx = None
    self.route = None
    self.route_geometry = None  # an (n, 2) array of (longitude, latitude) per step
    self.route_count = 0  # TurnPilot: routes calculated, so a replaced route is told apart from the one it replaces
    self.controls: list = []  # TurnPilot: the route's stop signs, yields and signals, and Valhalla's status for them
    self.controls_status = 'none'
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
    if skipped_past or new_destination != self.requested_destination or (self.should_recompute() and self.recompute_hold <= 0):
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
    fill_guidance(msg.navInstruction, self.route, self.step_idx, distance_to_maneuver_along_geometry)
    # TurnPilot: the controls in maneuverDistance's frame, the steps' distances and then along this step's geometry
    car_along = sum(s['distance'] for s in self.route[:self.step_idx]) + along_geometry
    fill_controls(msg.navInstruction, self.controls, self.controls_status, car_along)

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
    ni.offRouteControls = [{'id': i, 'kind': c.kind, 'distance': float(d), 'lineMapped': c.line_mapped, 'junction': c.junction,
                            'crossSuspect': c.cross_suspect} for d, i, c in near]

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
