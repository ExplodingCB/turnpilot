#!/usr/bin/env python3
import datetime
import math
import os
import threading
import time

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import system_time_valid
from openpilot.selfdrive.navigation.hold import (GPS, MAX_DT, YAW_STEP_MAX_DEG, Fix, Motion, PositionHold, RoutePath, car_motion,
                                                 course_good, gps_fix, wrap_deg)
from openpilot.selfdrive.navigation.route import ROUTE_TURNS_STEP_M, along_path, local_meters, nearest_segment, route_turns
from openpilot.selfdrive.turnpilot.route_follower import TURN, classify
from openpilot.selfdrive.turnpilot.routing.controls import haversine
from openpilot.selfdrive.turnpilot.routing.guidance import (CONTROLS_AHEAD_M, CONTROLS_BEHIND_M, MAX_CONTROLS, control_msg, fill_controls,
                                                            fill_guidance, speed_limit_at)
from openpilot.selfdrive.turnpilot.maps.client import MapError
from openpilot.selfdrive.turnpilot.maps import surveys
from openpilot.selfdrive.turnpilot import stop_rules, visibility_table
from openpilot.selfdrive.turnpilot.routing import avoid, certify, lefts
from openpilot.selfdrive.turnpilot.routing.sources import MapRouter
from openpilot.selfdrive.turnpilot.web.store import last_gps_position

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
# ...and parked (offroad, the route planned before the drive), where nothing waits on the route but the driver at the screen.
# On the car, 2026-11-02, an iteration took 1.5-2 s, not the Mac's 0.05 s: 4 s held 4 of the 8 reroutes ("out of time after
# 4 reroutes: the best route is kept"), so a route planned parked kept stops and lefts it could have gone around. Parked it
# gets room for all 8 at the car's pace
AVOID_TIME_PARKED_S = 20.0
AVOID_PARAM = 'TurnPilotAvoidStops'
ALWAYS_DRIVE_PARAM = 'TurnPilotAlwaysDrive'  # TurnPilot: read at route time for R1 to R3 (ALWAYS_DRIVE.md section 1, step S6)

# TurnPilot (route start, the 2026-11-06 drives 0000001d and 0000001e): a route planned standing has no start heading (hold.py
# gives one only moving at 3 m/s with a bearing accuracy within 20 deg; a parked u-blox reports 37-45), and Valhalla takes
# either direction of the nearest edge. 1e's first route pointed 122 deg with an 'end of road left' 44 m on while the car was
# parked at about 29 deg and needed a RIGHT to join: turnpilotd pulsed the left at 0.34 m/s, the model turned about 110 deg
# the wrong way out of the spot, and the driver reversed out ("said left when it should go right"). 1d's first route from
# its first fix (standing, bacc 32) said right where left was correct. So a route made with no heading is unverified: no
# valid instruction (so no turn desire) goes out from it until the car's first good moving heading is checked against the
# route's bearing at the car (heading_due), within HEADING_MISMATCH_DEG it is kept, past it navd reroutes at once with that
# heading (1e replayed: nothing sent from 0:32.4, so no left at 1:23.5; at 1:31.4, 3.0 m/s with bacc 5.6, heading 349 against
# the route's 122, a reroute with it. 1d replayed: the 'turn right' withheld through the 47 s stand, and at 4:27.2 heading 258
# against 92 rerouted 7 s before navd's off-route reroute did). With no good heading ever (a receiver reporting no bearing
# accuracy) the car's own travel verifies it: VERIFY_TRAVEL_M on along the step within VERIFY_LATERAL_M of it, so guidance
# can't be withheld for the whole drive
# Only an informative heading decides (review of 11-06): within HEADING_MISMATCH_DEG of the route's road with the car on it
# (VERIFY_LATERAL_M: on the street, not a parking row beside it) verifies; HEADING_REROUTE_DEG or more off it within
# OFF_ROUTE_DISTANCE is the car driving against the route, rerouted with that heading (1e 133 deg, 1d 166). In between the car
# is crossing a lot or turning in: a heading near-perpendicular to the street lets Valhalla (HEADING_TOLERANCE 90) take
# either direction again, so it decides nothing and the route stays unverified (a car 10 m off an east street heading 180
# rerouted with 180 and was sent 'Turn left 30.6 m' still in the lot). A reroute for the heading, or any route made while
# the one it replaces was unverified, is unverified too (heading_due checks it in the update that would send its first
# instruction, so one Valhalla snapped right costs nothing), and a mismatch on a reroute that came back the same way
# (Valhalla ignoring the heading, a one-way the car is heading against) reroutes again only once per RECOMPUTE_HOLD
HEADING_MISMATCH_DEG = 60.
HEADING_REROUTE_DEG = 120.
VERIFY_TRAVEL_M = 15.
VERIFY_LATERAL_M = 10.
VERIFY_STREET_V = 8.  # m/s: a matching heading this fast verifies within OFF_ROUTE_DISTANCE (a wide road's far lane, 10 m or
                      # more off its mapped centre line; no car park is driven at 18 mph)
# TurnPilot (parked heading, review of 11-06): navd runs offroad between drives, and offroad there is no carState or
# deviceMotion, so the hold's last heading would stand for the whole stop: the car's orientation in its space, turned with
# its yaw through the parking manoeuvre (perpendicular to the street nose-in), and a route planned parked for the next trip
# went with it as a known heading, never checked. Offroad navd routes with no heading (route_heading), and HEADING_PARKED_S
# offroad drops the hold's (an onroad cycle is about a second offroad, hardwared.py ONROAD_CYCLE_TIME, with the car moving)
HEADING_PARKED_S = 5.
# TurnPilot (parked onroad, review of 11-06): the occupied Tesla can park nose-in and stay onroad, so carState and deviceMotion
# stay live, and the hold's heading, turned with the car's yaw through the manoeuvre, is the car's orientation in its space:
# perpendicular to the street nose-in, and a destination set then was routed with it as a known heading, never checked (the
# 11-06 wrong-side-of-the-street failure by another road; Valhalla's HEADING_TOLERANCE 90 then takes the street's direction
# nearer the car's nose, whichever way the driver will leave). 1d ended so: standing 80 s at 82 deg from its last GPS course,
# after a 5.5 m crawl in and a reverse (1e's start: 55 deg turned reversing out of its spot). So navd routes with the hold's
# heading only while it rests on a GPS course (hold.py's own test, course_good: a trusted fix moving HEADING_MIN_V or faster
# with a bearing accuracy within HEADING_ACCURACY_MAX_DEG) that the car has since turned less than COURSE_TURN_MAX_DEG from at
# its furthest (a turn in and back out nets nothing), with under COURSE_UNTRACKED_M driven with no yaw to turn it by
# (HeadingBasis). Otherwise a route is made with no heading and goes unverified, as at a fresh start (heading_due), where the
# car's own heading verifies it once it points the route's way on the street, and only a course-backed one reroutes. The 11-06
# stands on the street (10, of 5-54 s, the course up to 58 s behind) had turned at most 11.9 deg from it at their furthest (9
# of them under 1 deg), so a reroute at a red light keeps its heading. No time bound: standing the car can't turn (yaw drifted
# under 0.5 deg over those stands), and a long red is no less the street's. Moving, a turn made under HEADING_MIN_V leaves it
# until the next course, a second (a right from a stop: 2.3-2.9 m/s and 36-40 deg, three times on 11-06; a route made in that
# second is checked against the car's own heading the update after). COURSE_UNTRACKED_M: at full lock the Model 3 turns about
# 13 deg a metre (its 11.8 m kerb circle puts the rear axle on about 4.4 m); on 11-06 deviceMotion was out only for its first
# second after locationd came up (about 55 s into each log), long before the car moved
COURSE_TURN_MAX_DEG = 30.
COURSE_UNTRACKED_M = 2.
# TurnPilot (route start, the straight-in case: review of 11-06): a GPS course the car hasn't turned from can itself point
# across the street: taken driving straight at it across a lot or down a driveway at HEADING_MIN_V or faster, or into a
# pull-through space. Standing at the exit, nose perpendicular to the street, a destination was routed with it as a known
# heading and trusted, so valid guidance (and a turn desire) went out at once (the reviewer's probe: a course of 180 at an east
# street 6 m on, 'left 239.7' the update after), with Valhalla's HEADING_TOLERANCE 90 having taken the street's direction
# nearer the nose, which says nothing about the way the driver will turn out. So under VERIFY_STREET_V a route made with a
# heading is checked against that heading where the car is (RouteEngine.routed_across): more than HEADING_MISMATCH_DEG across
# the route's bearing there, or the car further than OFF_ROUTE_DISTANCE off the route (a course up a lot row), and it goes
# unverified as one made with none does (heading_due decides, on the car's own heading). On the street a routed heading is the
# road's: of the 18 routes the 11-06 drives installed that navd now makes with a heading, the 16 with the car within
# OFF_ROUTE_DISTANCE of them were within 22 deg of their bearing there (15 within 5), and none changes. At VERIFY_STREET_V or
# faster the car is on a road, and a route that doesn't go its way where it is lands past its first turn: the other two, 30-41
# m off at 11.5 and 9.8 m/s carState (1e 13:23.3 and 13:29.3), and in the 1e 5:56.4 replay (the drive's old route given back
# after the car went straight through its right: at 10.5 m/s, 19.7 m off it and 95 deg across its bearing there) checking it
# withheld the guidance and with it the missed-turn reroute (send_instruction: went_straight_past) for up to RECOMPUTE_HOLD.
# There stale_start and the missed-turn reroutes, made for a route navd trusts, decide as before. The check is on the angle
# only, never VERIFY_LATERAL_M: a reroute at a red light in a wide road's far lane goes the heading's way and stays trusted
# (heading_due standing there would withhold guidance until 18 mph)
# TurnPilot (stale reroutes, 1e): a reroute lands 4-7 s (37-101 m) after 'Calculating route', so its first turn can already
# be behind the car (1e 9:43.4: a right at -4.8 m, skipped tooLate, rerouted again) or too close to make (rc=13 at 11:19.4:
# a right 37 m on at 12 m/s). A route whose first step ends in a junction turn nearer the live position than STALE_MIN_M or
# STALE_LEAD_S at the car's speed (turnpilotd's first pulse is due by 25 m: turn_skip's tooLate) is routed again once from
# where the car is now; the second result is installed as it is, so this can't loop
STALE_MIN_M = 25.
STALE_LEAD_S = 3.
STALE_MOVED_M = 10.  # ...for a car that moved this far while the job ran (from where it started, the same route comes back)
# TurnPilot (step advance by heading, 1e): navd moved past a step only 10 m beyond its end or with the car within NEXT_STEP_NEAR
# of the next step's line. At 8:35.1, after a right the car had finished (desire cleared at 8:32.6), it was 20.7 m off the
# step at -8.5 m along and 6.4 m off the next step's line on a wide road: one invalid message, and the valid one a second
# later at -12.1 m was skipped tooLate ('Too Fast to Turn / Rerouting' for a turn already made). The car's heading had taken
# the new road by 8:32.1 (351 deg, 9-13 at 8:33.4 against the next step's 14-16). So inside MANEUVER_TRANSITION_THRESHOLD of a
# step's end, a heading within TURNED_ONTO_DEG of the next step's road and more than LEFT_ROAD_DEG off this step's (it has
# left this road: on a slight turn a match on the next road alone would advance before the junction), with the car within
# NEXT_STEP_HEADING_NEAR of the next step's line, moves navd on
TURNED_ONTO_DEG = 30.
LEFT_ROAD_DEG = 30.
NEXT_STEP_HEADING_NEAR = 15.
# ...and the mirror case (1e 5:55.4): the car drove straight through a right (GPS 127-129 throughout, the step 128, the next
# road 201) and navd moved on to the next step at -10 m all the same, so turnpilotd armed an unprotected-left approach for a
# road the car wasn't on. Past the step's end, a maneuver turning more than MISSED_TURN_MIN_DEG with the car's heading more
# than MISSED_OFF_NEXT_DEG off the next road and within MISSED_ON_STEP_DEG of this one is a missed turn: rerouted at the next
# update (missed_off_route), never the next step
MISSED_TURN_MIN_DEG = 30.
MISSED_OFF_NEXT_DEG = 45.
MISSED_ON_STEP_DEG = 20.
BEARING_SPAN_M = 20.  # a road's bearing into or out of a junction: the chord over this much of the step (chord_bearing)
# TurnPilot (destination lifetime): NavDestination was CLEAR_ON_OFFROAD_TRANSITION, so any onroad cycle deleted it: modeld's
# big-model retry is one (modeld.py, 'cycling onroad to retry'; on 9 downloaded routes), and the occupied Tesla stays
# onroad through it, so a destination set on the phone was gone seconds into the drive. It is CLEAR_ON_MANAGER_START only
# now (params_keys.h), and navd ends the trip itself: offroad this long without a break after a drive (expire_destination)
DESTINATION_OFFROAD_S = 30. * 60.


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


def step_place(geometry, point) -> tuple[float, float, float] | None:
  """TurnPilot: (m along, m off, bearing deg) of the place on a step's (longitude, latitude) polyline nearest point: projected
  onto its segments with a length (a step's last point repeats as the next one's first), so it runs from 0 at the step's start
  to its length at its end, never past (along_path's along measures straight to the point, and grows with the lateral too),
  and the bearing of the segment it is on. None without a segment with a length."""
  g = np.asarray(geometry, dtype=np.float64).reshape((-1, 2))
  if len(g) < 2:
    return None
  local, p = local_meters(g, g[0]), local_meters(point, g[0])
  lengths = np.linalg.norm(np.diff(local, axis=0), axis=1)
  keep = np.flatnonzero(lengths > 1e-6)
  if not len(keep):
    return None
  j, t, lateral = nearest_segment(local[keep], local[keep + 1], p)
  k = int(keep[j])
  east, north = local[k + 1] - local[k]
  return float(np.sum(lengths[:k]) + t * lengths[k]), lateral, math.degrees(math.atan2(east, north)) % 360.


def chord_bearing(geometry, from_end: bool, span: float = BEARING_SPAN_M) -> float | None:
  """TurnPilot: deg of a step's road into its end (from_end) or out of its start: the chord over its last or first span m
  (all of a shorter step), steadier than one segment (a corner's polyline can start or end with a stub of a metre or two).
  None for a step with no length (the arrival's)."""
  g = np.asarray(geometry, dtype=np.float64).reshape((-1, 2))
  if len(g) < 2:
    return None
  if from_end:
    g = g[::-1]
  local = local_meters(g, g[0])
  cum = np.r_[0., np.cumsum(np.linalg.norm(np.diff(local, axis=0), axis=1))]
  if cum[-1] < 1e-6:
    return None
  s = min(span, float(cum[-1]))
  i = min(int(np.searchsorted(cum, s)), len(cum) - 1)
  t = (s - cum[i - 1]) / max(cum[i] - cum[i - 1], 1e-9) if i > 0 else 0.
  east, north = local[i - 1] + t * (local[i] - local[i - 1]) if i > 0 else local[i]
  bearing = math.degrees(math.atan2(east, north)) % 360.
  return (bearing + 180.) % 360. if from_end else bearing


class HeadingBasis:
  """TurnPilot (parked onroad, COURSE_TURN_MAX_DEG): what the hold's heading rests on, updated once per navd update right after
  the hold, with the fix and motion it was given: whether it is a GPS course's (hold.py course_good, the very test the hold
  takes its heading from a fix by), and since that course, how far the car turned from it (net and at its furthest, read off
  the hold's heading itself, which turns by deviceMotion's yaw) and how far it drove with no yaw to turn it by (hold.py takes
  a yaw step only between two valid yaws YAW_STEP_MAX_DEG or less apart). routable: navd may route with it
  (RouteEngine.route_heading)."""

  def __init__(self):
    self.course = False    # the hold's heading is a course's, turned with the car since
    self.turn = 0.         # deg it turned since the course, clockwise, net
    self.peak = 0.         # deg, the largest |turn| since the course
    self.untracked_m = 0.  # m driven since the course with no yaw to turn it by
    self._heading: float | None = None
    self._yaw: float | None = None
    self._t: float | None = None

  def update(self, heading: float | None, fix: Fix | None, motion: Motion | None, now: float) -> None:
    dt = min(max(now - self._t, 0.), MAX_DT) if self._t is not None else 0.
    self._t = now
    yaw = motion.yaw if motion is not None else None
    tracked = yaw is not None and self._yaw is not None and abs(wrap_deg(yaw - self._yaw)) <= YAW_STEP_MAX_DEG
    self._yaw = yaw
    if heading is None:  # none (stale, or dropped parked): the next one is a course's
      self.course = False
    elif course_good(fix, motion):
      self.course, self.turn, self.peak, self.untracked_m = True, 0., 0., 0.
    elif self.course and self._heading is not None:
      self.turn += wrap_deg(heading - self._heading)
      self.peak = max(self.peak, abs(self.turn))
      if not tracked and motion is not None:  # without carState nothing knows whether the car moves (hold.py)
        self.untracked_m += max(motion.v, 0.) * dt
    self._heading = heading

  @property
  def routable(self) -> bool:
    return self.course and self.peak < COURSE_TURN_MAX_DEG and self.untracked_m < COURSE_UNTRACKED_M

  def why_not(self) -> str | None:
    """Why the hold's heading isn't routed with: 'noCourse' (not a course's: set by hand), 'turned' (COURSE_TURN_MAX_DEG),
    'untracked' (COURSE_UNTRACKED_M); None when it is routable."""
    if not self.course:
      return 'noCourse'
    if self.peak >= COURSE_TURN_MAX_DEG:
      return 'turned'
    return 'untracked' if self.untracked_m >= COURSE_UNTRACKED_M else None


class RouteJob(threading.Thread):
  """One router.route() call off navd's 1 Hz loop: a corridor download takes minutes over cellular. navd keeps
  publishing meanwhile and installs the result itself, on its own thread, if the destination is still the same.

  The router puts every failure in TurnPilotMapStatus; one that isn't a MapError is put there here."""

  def __init__(self, engine: 'RouteEngine', destination, language, stale_retry: bool = False, heading_retry: bool = False):
    super().__init__(name='navd-route', daemon=True)
    self.engine, self.destination, self.language = engine, destination, language
    self.start_position, self.start_bearing = engine.last_position, engine.route_heading()
    self.seeded = engine.seeded  # TurnPilot: started from LastGPSPosition, not a live fix (RouteEngine.reseed_due)
    # TurnPilot (route start): the heading the route was made with. The router routes from position() after a corridor
    # download (minutes), and so does each of the avoidance loop's reroutes, so it is the bearing of the last such call; a
    # reroute made with none leaves the kept route unverified however it started (RouteEngine.heading_due)
    self.routed_bearing = self.start_bearing
    self.headingless_reroute = False
    # TurnPilot (parked onroad, COURSE_TURN_MAX_DEG): why the car's own heading wasn't routed with at the start, if it had one
    # (HeadingBasis.why_not: 'turned' nose-in in a space), for the route_heading_unverified log line
    self.heading_dropped = engine.heading_dropped()
    self.stale_retry = stale_retry  # TurnPilot: routed again because the last route's first turn was stale (collect_route)
    # TurnPilot (route start, HEADING_REROUTE_DEG): routed again for a heading against the route, or while the route it
    # replaces was unverified (a reroute from a lot): its heading may be a lot's, or ignored, so it is checked too
    self.heading_retry = heading_retry
    self.replaces_unverified = engine.heading_unverified
    self.cancel = threading.Event()
    self.steps: list[dict] | None = None
    self.controls: list = []  # TurnPilot: the route's stop signs, yields and signals (turnpilot/routing/controls.py)
    self.controls_status = 'none'
    self.opposing: dict = {}  # TurnPilot: {step index: (m/s, lanes)} of the roads across its maneuvers (routing/controls.py)
    self.limits: list = []  # TurnPilot: [(end m, m/s)] of its edges, tagged else the class default (navInstruction.speedLimit)
    # TurnPilot (stop signs): the destination's exclusions, whether to avoid, the runtime inputs the route is certified
    # with, and what the avoidance loop did (routing/avoid.py)
    self.avoid = engine.avoid_set(destination)
    self.avoid_on = engine.avoid_stops_on()
    self.always_drive = engine.always_drive_on()
    self.runtime = engine.runtime_inputs()
    self.avoid_status, self.avoided, self.avoid_added_s = 'none', 0, 0.
    # TurnPilot (unprotected lefts): what the route's lefts are judged under for routing/lefts.py's log line (TurnPilotAutoLeft,
    # the detector's proven ranges, the far crop), and the avoidance's lefts counts (0 since 11-06: none is routed around)
    self.left_config = lefts.left_config(engine.params)
    self.lefts_avoided, self.lefts_unavoidable = 0, 0

  @property
  def heading_known(self) -> bool:
    """TurnPilot: whether the route was made with a start heading (route start: RouteEngine.heading_due checks it if not)."""
    return self.routed_bearing is not None and not self.headingless_reroute

  @property
  def unchecked(self) -> str | None:
    """TurnPilot: why the route's start direction must be checked before guidance goes out (RouteEngine.heading_due), None
    when it was routed with a heading from the car's own travel on a checked route. Such a route is still checked once it
    is installed under VERIFY_STREET_V when it came back across that heading where the car is (RouteEngine.routed_across:
    'headingAcross')."""
    if not self.heading_known:
      return 'noHeading'
    if self.heading_retry:
      return 'headingReroute'
    return 'replacesUnverified' if self.replaces_unverified else None

  def live_position(self):
    """The engine's (position, bearing) now, for a router that routes from it after a download; the bearing is noted."""
    position, bearing = self.engine.position()
    self.routed_bearing = bearing
    return position, bearing

  def run(self):
    router, destination = self.engine.router, self.destination
    try:
      self.steps = router.route(self.start_position, destination, self.start_bearing, self.language,
                                position=self.live_position, cancelled=self.cancel.is_set)
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
      # the kept route's speed limits (navInstruction.speedLimit): unknown on a failure, so the signal leads 300 ft
      try:
        self.limits = router.limits(self.steps) if hasattr(router, 'limits') else []
      except Exception:
        cloudlog.exception("navd: route speed limits failed")
        self.limits = []


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
      self.headingless_reroute |= bearing is None
      return router.reroute(position, destination, bearing, language, exclude)

    def own_edge():
      position, bearing = engine.position()
      return router.own_edge(position, bearing) if hasattr(router, 'own_edge') and position is not None and bearing is not None else None

    def headings(points):
      return router.exclusion_headings(points) if hasattr(router, 'exclusion_headings') else [None for _ in points]

    def opposing_of(steps):
      try:
        return router.opposing(steps)
      except Exception:
        cloudlog.exception("navd: the route's opposing roads failed: its lefts take 45 mph, not known straight (out of range)")
        return {}

    def route_lefts(steps, controls):
      # the route's U-turns (routing/lefts.py: the driving model can't drive one), routed around by their exit edges. Its
      # unprotected lefts never are, nor skipped for range (Kellen, 11-06): each is driven to its hold line and judged there
      # (unprotected_left: a gap the camera certifies, else held with a loud handback), and only logged here. A failure routes
      # no U-turn around, said: route_follower sends nothing at one, the car drives on past it and navd reroutes
      try:
        return lefts.route_lefts(steps, controls, self.left_config, lambda: opposing_of(steps),
                                 lambda: router.exits(steps) if hasattr(router, 'exits') else {})
      except Exception:
        cloudlog.exception("navd: the route's U-turns failed: none is routed around")
        return []

    deadline = engine.monotonic() + (AVOID_TIME_S if engine.onroad() else AVOID_TIME_PARKED_S)
    result = avoid.run(self.steps, self.controls, self.controls_status, self.avoid, certify=certify_one, reroute=reroute,
                       route_controls=router.controls, own_edge=own_edge, cancelled=self.cancel.is_set, deadline=deadline,
                       monotonic=engine.monotonic, enabled=self.avoid_on, headings=headings, lefts=route_lefts)
    self.steps, self.controls, self.controls_status = result.steps, result.controls, result.controls_status
    self.avoid_status, self.avoided, self.avoid_added_s = result.status, result.avoided, result.added_s
    self.lefts_avoided, self.lefts_unavoidable = result.lefts_avoided, result.lefts_unavoidable
    if hasattr(router, 'serve_again'):  # a kept earlier route is the served one again, for its opposing roads and limits
      router.serve_again(self.steps)


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
    self.lefts_avoided, self.lefts_unavoidable = 0, 0  # TurnPilot: the route's lefts routed around and left on it (RouteJob)
    self.route_runtime = None
    self.avoid_rerun = False
    self.route_always_drive = False  # TurnPilotAlwaysDrive as the route was made under it (always_drive_on; R1 to R3, S6)

    self.last_position = None  # (longitude, latitude)
    self.last_bearing = None
    # TurnPilot: until the first live fix, LastGPSPosition stands in for the car's position (seed_position), and the last
    # route job started from it is checked against that fix (reseed_due)
    self.seeded = False
    self.route_seeded = False
    # TurnPilot (GPS loss): where last_position comes from: a trusted fix, held standing, carried along the route without one
    # (navigation/hold.py), and navInstruction.positionSource
    self.hold = PositionHold()
    # TurnPilot (parked onroad, COURSE_TURN_MAX_DEG): what the hold's heading rests on, and whether navd routes with it
    self.basis = HeadingBasis()
    self.basis_routable = False  # ...as of the last update (navd.heading_unroutable is logged as it stops)

    self.nav_destination = None
    self.step_idx = None
    self.route = None
    self.route_geometry = None  # an (n, 2) array of (longitude, latitude) per step
    self.route_count = 0  # TurnPilot: routes calculated, so a replaced route is told apart from the one it replaces
    self.controls: list = []  # TurnPilot: the route's stop signs, yields and signals, and Valhalla's status for them
    self.controls_status = 'none'
    self.opposing: dict = {}  # TurnPilot: the route's opposing roads' speeds and lanes (RouteJob.opposing)
    self.limits: list = []  # TurnPilot: the route's speed limits along it (RouteJob.limits)
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
    self.missed = False  # ...and it newly did (or the car's heading says so at the step's end): reroute at once
    # TurnPilot (route start): the route was made with no start heading and the car's direction isn't checked against it yet
    # (heading_due); verify_from is the least m along the step the car has been on it since, for the check by travel
    self.heading_unverified = False
    self.verify_from: float | None = None
    self.route_heading_retry = False  # ...and the route was itself a reroute for the heading (HEADING_REROUTE_DEG)
    # TurnPilot (parked heading, HEADING_PARKED_S): deviceState.started is False (heard, and False), and since when
    self.parked = False
    self.parked_since: float | None = None
    self.v_ego = 0.  # TurnPilot: carState's vEgo at the last update, 0 without one (a stale route's lead: collect_route)
    # TurnPilot (destination lifetime): deviceState.started at the last update (None: not heard), and since when (monotonic)
    # the car has been offroad after a drive with the destination it had then (expire_destination)
    self.was_started: bool | None = None
    self.offroad_since: float | None = None
    self.offroad_destination = None

    # TurnPilot: routes are computed by a RouteJob, in a thread unless background is False (tests that step navd)
    self.background = background
    self.job: RouteJob | None = None
    self.requested_destination = None  # the destination of the last route request, which a new one skips the hold for

    # TurnPilot: routes come from TurnPilot's map server (or local tiles), never Mapbox; see turnpilot/routing/sources.py
    self.router = router if router is not None else MapRouter(self.params)
    self.seed_position()

  def seed_position(self) -> None:
    """TurnPilot: LastGPSPosition (ubloxd's or qcomgpsd's last fix, saved once a minute; with the receiver on offroad, where
    the car is parked) is the car's position until the first live fix, so the geofence check, the corridor download and
    Valhalla start with navd, at boot, instead of after the receiver's fix. It is a guess, of an unknown age with the clock
    not yet set: no valid instruction is sent from it (send_instruction), nothing is looked up ahead of it (fill_off_route),
    and the first live fix checks the route made from it (reseed_due). It carries no heading: a parked car's says nothing."""
    try:
      seed = last_gps_position(self.params)
    except Exception:
      cloudlog.exception("navd: LastGPSPosition unreadable")
      return
    if seed is not None:
      self.last_position, self.last_bearing, self.seeded = (seed[1], seed[0]), None, True
      cloudlog.event("navd.seeded_from_last_position")  # never the position itself

  def onroad(self) -> bool:
    """TurnPilot: whether the car is started (deviceState.started). navd runs offroad too while TurnPilot is enabled
    (process_config.navigation), where it plans the route and stops there: no reroutes for a parked fix's wander, no
    lookups ahead and no ETA drift (turnpilotd isn't running, and the drive's ETAs start at departure). Before deviceState
    is heard, and where navd isn't given it (tests), it is onroad, as navd always was."""
    if 'deviceState' not in self.sm.data or not self.sm.seen['deviceState']:
      return True
    return bool(self.sm['deviceState'].started)

  def device_started(self) -> bool | None:
    """TurnPilot: deviceState.started, None before it is heard (or where navd isn't given it: tests)."""
    if 'deviceState' not in self.sm.data or not self.sm.seen['deviceState']:
      return None
    return bool(self.sm['deviceState'].started)

  def expire_destination(self) -> None:
    """TurnPilot (destination lifetime, DESTINATION_OFFROAD_S): the trip ends DESTINATION_OFFROAD_S offroad without a break
    after a drive, with the destination it had: NavDestination is removed (recompute_route then clears the route), once.
    An onroad cycle (modeld's big-model retry, a second or so offroad) keeps it, the count starting over at the next stop,
    and a destination set or changed while parked is for the next drive: kept until that drive ends. A real parking stop
    still ends the trip. navd started parked (a restart of it, or TurnPilot switched on) has seen no drive end: kept until
    the next one does, or manager start."""
    started = self.device_started()
    if started is None:
      return
    now = self.monotonic()
    if started:
      self.offroad_since = None
    elif self.was_started:  # the drive ended: a parking stop, or the start of an onroad cycle
      self.offroad_since, self.offroad_destination = now, self.params.get("NavDestination")
    self.was_started = started
    if self.offroad_since is None:
      return
    destination = self.params.get("NavDestination")
    if destination is None or destination != self.offroad_destination:
      self.offroad_since = None  # cleared, or set for the next drive
      return
    if now - self.offroad_since >= DESTINATION_OFFROAD_S:
      cloudlog.event("navd.destination_expired", offroad_s=round(now - self.offroad_since))
      self.params.remove("NavDestination")
      self.offroad_since = None

  def update(self):
    self.sm.update(0)

    self.expire_destination()
    self.note_skip()
    # TurnPilot (GPS loss): a fix moves the car only when trusted and the car moves; without one the car's own travel carries
    # it along the route (navigation/hold.py). A turn missed while carried means the car isn't where the hold put it: lost
    motion = car_motion(self.sm)
    self.v_ego = max(motion.v, 0.) if motion is not None else 0.
    fix, now = gps_fix(self.sm), self.monotonic()
    self.hold.update(fix, motion, now, self.route_geometry, self.step_idx, missed=self.missed and self.step_idx is not None)
    self.note_parked()
    self.note_basis(fix, motion, now)
    if self.hold.position is not None:  # a live fix (held or carried from one) replaces LastGPSPosition's seed
      self.last_position, self.last_bearing, self.seeded = self.hold.position, self.hold.bearing, False
    route, route_count = self.route, self.route_count
    try:
      self.recompute_route(gps_ok=self.hold.source == GPS)
      if route is not None and self.route is not None and self.route_count != route_count:
        # TurnPilot: one invalid instruction between two routes, so no maneuver (and its turn desire) carries over to
        # a new route that starts with one of the same type
        msg = messaging.new_message('navInstruction', valid=False)
        msg.navInstruction.routeCount = self.route_count
        self.fill_position(msg.navInstruction)
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
    return self.last_position, self.route_heading()

  def own_heading(self) -> float | None:
    """TurnPilot: the car's own heading, what navd looks ahead with (AheadJob) and checks a route's start direction against
    (heading_due): the hold's, a fix's bearing only moving with a good bearing accuracy, else the last good one turned with
    the car's yaw since, else none (hold.py; a standing or just-reacquired u-blox heading is stale or 0). A position set by
    hand, with no hold, has its bearing. Offroad none (HEADING_PARKED_S)."""
    if self.parked:
      return None
    return self.hold.heading if self.hold.position is not None else self.last_bearing

  def route_heading(self) -> float | None:
    """TurnPilot: the heading navd routes with: the car's own (own_heading) while it rests on a GPS course the car has barely
    turned from since (HeadingBasis.routable, COURSE_TURN_MAX_DEG), else none. Offroad none (HEADING_PARKED_S), and onroad
    nose-in in a space none either: where the car points in its space says nothing about the way it will leave it. The
    route job reads it here at its start, after a download (live_position) and in the avoidance loop's reroutes alike."""
    heading = self.own_heading()
    if heading is None or self.hold.position is None or self.basis.routable:
      return heading
    return None

  def heading_dropped(self) -> str | None:
    """TurnPilot: why the car's own heading isn't routed with (HeadingBasis.why_not), None when it is or there is none."""
    if self.own_heading() is None or self.hold.position is None:
      return None
    return self.basis.why_not()

  def note_basis(self, fix, motion, now: float) -> None:
    """TurnPilot (parked onroad, COURSE_TURN_MAX_DEG): HeadingBasis updated with the hold's inputs and the heading it left, and
    navd.heading_unroutable logged once as the car's heading stops being one navd routes with (a turn under HEADING_MIN_V:
    into a space, out of one, a right from a stop)."""
    self.basis.update(self.hold.heading, fix, motion, now)
    routable = self.basis.routable
    if self.basis_routable and not routable and self.hold.heading is not None:
      cloudlog.event("navd.heading_unroutable", why=self.basis.why_not(), turn=round(self.basis.turn, 1), peak=round(self.basis.peak, 1),
                     untracked_m=round(self.basis.untracked_m, 1), standing=self.hold.standing, v_ego=round(self.v_ego, 1))
    self.basis_routable = routable

  def note_parked(self) -> None:
    """TurnPilot (parked heading, HEADING_PARKED_S): whether deviceState says offroad, and HEADING_PARKED_S of it drops the
    hold's heading, every update (a parked fix's speed noise could set one again), so the next drive starts with none, as
    a navd started fresh does: its first route goes unverified until the car's first good moving heading checks it."""
    if self.device_started() is not False:
      self.parked, self.parked_since = False, None
      return
    now = self.monotonic()
    if not self.parked:
      self.parked, self.parked_since = True, now
    if self.hold.heading is not None and self.parked_since is not None and now - self.parked_since >= HEADING_PARKED_S:
      cloudlog.event("navd.parked_heading_dropped", offroad_s=round(now - self.parked_since, 1))
      self.hold.heading = None

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

  def always_drive_on(self) -> bool:
    """TurnPilot: TurnPilotAlwaysDrive, read at route time (ALWAYS_DRIVE.md section 1). Off unless set to 1; a build whose
    params don't know it is off, said."""
    try:
      value = self.params.get(ALWAYS_DRIVE_PARAM)
    except Exception as e:
      cloudlog.warning(f"navd: {ALWAYS_DRIVE_PARAM} unreadable ({type(e).__name__}): off")
      return False
    if isinstance(value, (bytes, str)):
      return (value.decode(errors='replace') if isinstance(value, bytes) else value).strip() == '1'
    return bool(value)

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
    return certify.Runtime(mode=mode, side_views=side, a_launch=stop_rules.a_launch_plan(self.params), clock_valid=system_time_valid())

  @staticmethod
  def runtime_key(runtime: certify.Runtime) -> tuple:
    return runtime.mode, runtime.side_views, runtime.a_launch, runtime.clock_valid

  def certify_default(self, control, eta, runtime):
    """TurnPilot: routing/certify.py on the shipped visibility and survey files (a sim table only where the sim says so)."""
    visibility = visibility_table.cached(visibility_table.SHIPPED, sim_ok=self.sim_tables_ok())
    return certify.certify_control(control, visibility, surveys.cached(surveys.SHIPPED).entries, eta, runtime)

  def sim_tables_ok(self) -> bool:
    """TurnPilot: a sim visibility table is read only with SIMULATION=1, TurnPilotSimLightOracle and a sim car."""
    if os.environ.get('SIMULATION') != '1':
      return False
    try:
      from opendbc.car.structs import car  # card.py's CarParams (openpilot.cereal has no car)
      oracle = self.params.get(visibility_table.SIM_PARAM)
      raw = self.params.get('CarParams')
      if not raw:
        return visibility_table.sim_accepted(os.environ, oracle, None)
      with car.CarParams.from_bytes(raw) as cp:  # read inside: the reader is gone once the message is closed
        return visibility_table.sim_accepted(os.environ, oracle, cp)
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
    if self.avoid_rerun or self.route is None or now is None or not self.onroad():  # TurnPilot: offroad the ETAs start at departure
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

    # Don't recompute when GPS drifts in tunnels (TurnPilot: nor while the route is held without a trusted fix, hold.py)
    if not gps_ok and self.step_idx is not None:
      return

    self.recompute_hold -= 1
    skipped_past = self.skipped_turn_passed() or self.missed_off_route()  # TurnPilot: a reroute past a skipped or missed
                                                                          # turn waits for nothing
    rerun = self.avoid_rerun_due()  # TurnPilot: nor does a stop-sign certification that changed
    reseed = self.reseed_due()  # nor a route from the seed that the first live fix finds the car off
    heading = self.heading_due()  # nor a route made with no heading that the car's first good one says points the wrong way
    # TurnPilot: offroad, a route is kept as planned. And while seeded (no live fix yet: 1d drove 3:40 out of a garage with
    # none) the seed doesn't move, so should_recompute rerouted from the same guess every RECOMPUTE_HOLD (seven times, 11 s
    # apart, each re-running the controls and the avoidance, none ever sent): one route until the first fix (reseed_due)
    parked = self.step_idx is not None and (not self.onroad() or self.seeded)
    if skipped_past or rerun or reseed or heading or new_destination != self.requested_destination or \
       (not parked and self.should_recompute() and self.recompute_hold <= 0):
      self.recompute_hold = RECOMPUTE_HOLD
      self.calculate_route(new_destination, heading_retry=heading)
      self.reroute_counter = 0

  def reseed_due(self) -> bool:
    """TurnPilot: the last route job started from the seed (seed_position) and navd has a live fix now. Once: route again at
    once if that job found no route (a seed outside the mapped area, say) or the car is more than REROUTE_DISTANCE off the
    step it is on, rather than wait out RECOMPUTE_HOLD or REROUTE_COUNTER_MIN. On a corridor that covers the car that is a
    reroute on it, with no download. A job that downloaded routed from the live fix if it had come by then (RouteJob's
    position), and the car is on that route: nothing to do."""
    if not self.route_seeded or self.seeded or self.job is not None or self.last_position is None:
      return False
    self.route_seeded = False
    if self.route is None or self.route_geometry is None or self.step_idx is None:
      return True
    off = along_path(self.route_geometry[self.step_idx], self.last_position)[1] > REROUTE_DISTANCE
    if off:
      cloudlog.event("navd.reroute_from_first_fix", step_idx=self.step_idx)
    return off

  def heading_against_route(self, heading: float) -> tuple[float, float] | None:
    """TurnPilot (route start): (deg the car's heading is off the route's bearing at the car, m off the route there: this step
    or the next), or None with the car further than OFF_ROUTE_DISTANCE off both (its heading says nothing about this road,
    and should_recompute reroutes from there). The route's bearing at the car is the route's own where the car is on it, the
    next step's once the car is nearer that (hold.RoutePath.along, as the hold places the car): a car first reaching a good
    heading in the route's first turn (navd sends nothing meanwhile, so it doesn't move on) is checked against the road it
    is turning onto, not the one it left (1e 8:33.4 replayed from a standing start: heading 12 against the exit's 16, the
    approach's 285)."""
    assert self.route_geometry is not None and self.step_idx is not None and self.last_position is not None
    place = step_place(self.route_geometry[self.step_idx], self.last_position)
    if place is None:
      return None
    _, lateral, bearing = place
    following = step_place(self.route_geometry[self.step_idx + 1], self.last_position) if self.step_idx + 1 < len(self.route_geometry) else None
    near = min(lateral, following[1]) if following is not None else lateral
    if near > OFF_ROUTE_DISTANCE:
      return None
    path = RoutePath(self.route_geometry)
    at = path.along(self.step_idx, self.last_position)
    if at is not None:
      bearing = path.at(at)[1]
    return wrap_deg(heading - bearing), near

  def heading_due(self) -> bool:
    """TurnPilot (route start, HEADING_MISMATCH_DEG): the route's start direction isn't checked (collect_route: made with no
    heading, a reroute for the heading, made while the route it replaced was unverified, or made under VERIFY_STREET_V with a
    heading it came back across where the car is, routed_across: straight at the street from a lot), and navd sends no
    guidance from it until the car's direction is checked against it, on an informative heading only (heading_against_route;
    the car's own, own_heading):
    - within HEADING_MISMATCH_DEG of the route's road and on it (VERIFY_LATERAL_M; a wide road's far lane further off at
      VERIFY_STREET_V or faster, a speed no car park is driven at): verified. Standing too: the hold's heading standing is
      the car's own, turned with its yaw (on a street, its lane's direction; nose-in in a space, 90 deg off, decides nothing),
      and so is one the car turned from its last course since (parked onroad, COURSE_TURN_MAX_DEG): pointing the route's way
      on the street it verifies, whatever navd may route with.
    - HEADING_REROUTE_DEG or more off it, moving, on a heading navd routes with (route_heading: a course's, barely turned
      from since): the car drives against the route, rerouted now (True) with that heading. One the car turned from its
      course since (out of a space at a crawl) decides nothing: navd would route with none, and the same guess comes back.
      A reroute for the heading that came back the same way (Valhalla ignoring it, a one-way the car heads against) is
      rerouted again only once per RECOMPUTE_HOLD, never every update.
    - in between: the car is crossing a lot or turning in, and its heading says nothing about the street's direction (a
      reroute with a near-perpendicular one lets Valhalla's HEADING_TOLERANCE take either again): nothing is decided.
    With no good heading, VERIFY_TRAVEL_M along the step within VERIFY_LATERAL_M of it verifies it: the car went the route's
    way. Travel the wrong way along it never does (verify_from follows the car back). Only onroad: parked there is no
    carState, the hold takes every trusted fix, and a parked receiver's wander along the street would pass for travel; a
    route planned parked stays unverified until the drive."""
    if not self.heading_unverified or self.job is not None or self.step_idx is None or self.route_geometry is None or \
       self.last_position is None or not self.onroad():
      return False
    place = step_place(self.route_geometry[self.step_idx], self.last_position)
    if place is None:  # a step with no length: nothing to check against
      self.heading_unverified = False
      return False
    along, lateral, _ = place
    heading = self.own_heading()
    if heading is not None:
      self.verify_from = None  # a heading decides, or says the car isn't going along the route yet
      against = self.heading_against_route(heading)
      if against is None:
        return False
      diff, near = against
      if abs(diff) <= HEADING_MISMATCH_DEG and (near <= VERIFY_LATERAL_M or self.v_ego >= VERIFY_STREET_V):
        self.heading_unverified = False
        cloudlog.event("navd.route_heading_verified", how='heading', diff=round(diff, 1), lateral=round(near, 1),
                       standing=self.hold.standing)
        return False
      if abs(diff) < HEADING_REROUTE_DEG or self.hold.standing or self.route_heading() is None:
        return False
      if self.route_heading_retry and self.recompute_hold > 0:
        return False
      cloudlog.event("navd.reroute_heading_mismatch", step_idx=self.step_idx, diff=round(diff, 1), lateral=round(near, 1),
                     again=self.route_heading_retry)
      return True
    if lateral >= VERIFY_LATERAL_M:
      self.verify_from = None
      return False
    if self.verify_from is None or along < self.verify_from:
      self.verify_from = along
    if along - self.verify_from >= VERIFY_TRAVEL_M:
      self.heading_unverified, self.verify_from = False, None
      cloudlog.event("navd.route_heading_verified", how='travel', lateral=round(lateral, 1))
    return False

  def routed_across(self, job: RouteJob) -> tuple[float | None, float | None] | None:
    """TurnPilot (route start, the straight-in case: review of 11-06, the comment after COURSE_UNTRACKED_M): for a route just
    installed under VERIFY_STREET_V that was made with a heading (RouteJob.unchecked None), whether it came back across that
    heading where the car is. (deg the heading is off the route's bearing at the car, m off the route there:
    heading_against_route) when that is more than HEADING_MISMATCH_DEG; (None, None) with the car further than
    OFF_ROUTE_DISTANCE off the route's step and the next (a lot row: the heading says nothing about the route's road); None
    when the route goes the heading's way (a red light on the street: about 0 deg, in any lane), with nothing to check
    against (a step with no length), or at VERIFY_STREET_V or faster: a car on a road, where a route that doesn't go its
    way where it is (a reroute landing past its first turn) is stale_start's and the missed-turn reroutes' (send_instruction:
    went_straight_past), which act on a route navd trusts."""
    if self.v_ego >= VERIFY_STREET_V or job.routed_bearing is None or self.route_geometry is None or self.step_idx is None or \
       self.last_position is None or step_place(self.route_geometry[self.step_idx], self.last_position) is None:
      return None
    against = self.heading_against_route(job.routed_bearing)
    if against is None:
      return None, None
    return against if abs(against[0]) > HEADING_MISMATCH_DEG else None

  def stale_start(self, job: RouteJob) -> float | None:
    """TurnPilot (stale reroutes, STALE_MIN_M): m from the live position to the just-installed route's first maneuver when that
    is a junction turn (route_follower.classify of its banner, as turnpilotd reads it) too close to make at the car's speed,
    behind the car included, else None. Only for a car that moved STALE_MOVED_M or more while the job ran: from where it
    started, routing again gives the same route. Standing, a near turn is just near: a route planned at a red light isn't
    stale, nor one planned parked (offroad, where a parked fix's wander isn't the car moving). d0 is navd's own distance
    (along_path), the one turnpilotd will be sent, less the second until it is sent."""
    if self.route is None or self.route_geometry is None or self.last_position is None or len(self.route) < 2 or \
       self.hold.standing or job.start_position is None or not self.onroad():
      return None
    if float(np.linalg.norm(local_meters(self.last_position, job.start_position))) < STALE_MOVED_M:
      return None
    banners = self.route[0].get('bannerInstructions') or []
    primary = (banners[0].get('primary') or {}) if banners else {}
    if classify(primary.get('type') or '', primary.get('modifier') or '')[0] != TURN:
      return None
    d0 = float(self.route[0]['distance']) - along_path(self.route_geometry[0], self.last_position)[0]
    # the separator goes out this update and the route's first instruction the next (update), a second on at the car's speed
    return d0 if d0 - self.v_ego * 1. < max(STALE_MIN_M, STALE_LEAD_S * self.v_ego) else None

  def calculate_route(self, destination, stale_retry: bool = False, heading_retry: bool = False):
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
    self.job = RouteJob(self, destination, lang, stale_retry=stale_retry, heading_retry=heading_retry)
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
    self.route_seeded = job.seeded  # TurnPilot: checked against the first live fix (reseed_due)
    if job.steps is None:
      self.clear_route()
      return
    self.route = job.steps
    self.controls, self.controls_status = job.controls, job.controls_status
    self.opposing = job.opposing
    self.limits = job.limits
    self.avoid_status, self.avoided, self.avoid_added_s = job.avoid_status, job.avoided, job.avoid_added_s
    self.lefts_avoided, self.lefts_unavoidable = job.lefts_avoided, job.lefts_unavoidable
    self.note_lefts(job.destination)
    self.route_always_drive = job.always_drive
    self.route_runtime, self.avoid_rerun = self.runtime_key(job.runtime), False
    self.route_count += 1
    self.route_geometry = [np.array(step['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for step in self.route]
    self.step_idx = 0
    self.nav_destination = job.destination
    # TurnPilot (route start): a route made with no heading sends nothing until the car's direction is checked (heading_due),
    # nor one rerouted for the heading or from an unverified route (RouteJob.unchecked), nor one made with a heading that it
    # came back across where the car is (routed_across: the heading said nothing about the street's direction, straight at
    # it across a lot). A reroute's first instruction goes out the update after this one (this one sends the separator), and
    # heading_due runs before it: one that points the car's way costs nothing
    why, across = job.unchecked, None
    if why is None:
      across = self.routed_across(job)
      if across is not None:
        why = 'headingAcross'
    self.heading_unverified, self.verify_from, self.route_heading_retry = why is not None, None, job.heading_retry
    if self.heading_unverified:
      diff, near = across if across is not None else (None, None)
      cloudlog.event("navd.route_heading_unverified", seeded=job.seeded, why=why, dropped=job.heading_dropped,
                     diff=None if diff is None else round(diff, 1), lateral=None if near is None else round(near, 1))
      place = step_place(self.route_geometry[0], self.last_position) if self.last_position is not None else None
      if place is not None and place[1] < VERIFY_LATERAL_M:
        self.verify_from = place[0]  # travel along it is counted from where the car is now
    # TurnPilot (stale reroutes): a first turn the car can't make any more is routed again from where it is now, once (the
    # retry's result is installed as it is). The stale route stays installed meanwhile, sending nothing while the retry runs
    # (send_instruction: a job). Last here: in the foreground calculate_route collects the retry inside this call
    d0 = None if job.stale_retry else self.stale_start(job)
    if d0 is not None:
      cloudlog.event("navd.route_stale", d0=round(d0, 1), v_ego=round(self.v_ego, 1))
      self.recompute_hold, self.reroute_counter = RECOMPUTE_HOLD, 0
      self.calculate_route(destination, stale_retry=True)

  def note_lefts(self, destination) -> None:
    """TurnPilot: the route's lefts in TurnPilotMapStatus, for the phone page and the comma: how many the route goes around and
    how many it couldn't, never where; both 0 since 11-06 (routing/lefts.py: each unprotected left is driven to its line, none
    routed around or skipped for range; its U-turns are no lefts to the counts). Here on navd's loop, for a job whose route
    it takes, so a route dropped for a new destination never writes the new one's status."""
    status = getattr(self.router, 'status', None)
    if status is None or not hasattr(status, 'note'):
      return
    try:
      status.note(destination, lefts={'avoided': int(self.lefts_avoided), 'unavoidable': int(self.lefts_unavoidable)})
    except Exception:
      cloudlog.exception("navd: the route's lefts weren't written to the map status")

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
    msg.navInstruction.avoidAddedS = float(max(self.avoid_added_s, 0.)) if self.route is not None else 0.
    # the whole route's stops the avoidance couldn't route around, and whether the first is on its first edge (route start's
    # "M stop signs on this route need you", "First stop needs you": STOP_SIGNS.md 3.4)
    stuck = sorted((c for c in (self.controls or []) if c.kind in ('stop', 'yield') and getattr(c, 'certification', '') == 'unavoidable'),
                   key=lambda c: c.route_m) if self.route is not None else []
    msg.navInstruction.unavoidableControls = min(len(stuck), 65535)
    msg.navInstruction.firstUnavoidableOnOrigin = bool(stuck) and bool(getattr(stuck[0], 'first_edge', False))
    self.fill_position(msg.navInstruction)

    # TurnPilot: a job while there is a route is a reroute for the same destination (a new one clears the route), as when
    # the car left the corridor and a new one downloads for minutes. The car is off this route meanwhile: its distances
    # mean nothing, and following them would run through its steps to 'Destination reached' and delete the destination.
    # Nor is there guidance before the first live fix: the seed (seed_position) is only where the car was last seen. And
    # with the position lost (GPS gone past the hold's limits, hold.py): the route is kept for the fix that comes back,
    # and turnpilotd hands back on positionSource lost. Nor from a route made with no heading until the car's direction is
    # checked against it (heading_due: 1e's first left went out from a route pointing the way the car wasn't)
    if self.step_idx is None or self.job is not None or self.seeded or self.hold.lost or self.heading_unverified:
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
    # TurnPilot (step advance by heading, TURNED_ONTO_DEG): the car's heading has taken the next step's road; on a wide road the
    # car can be 20 m off this step and 6 m off the next one's line at -8.5 m, which was an invalid message (below) and a
    # phantom tooLate for a turn already made (1e 8:35.4)
    if self.turned_onto_next(distance_to_maneuver_along_geometry):
      if self.skip_here():
        cloudlog.event("navd.skipped_turn_taken", step_idx=self.step_idx)
        self.skipped = None
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
    # TurnPilot (off-route turn, F3): the route's own shape around the car, so a bend the road makes is no off-route turn
    turns, msg.navInstruction.routeTurnsStart = route_turns(self.route_geometry, self.step_idx, along_geometry)
    msg.navInstruction.routeTurns, msg.navInstruction.routeTurnsStep = turns, ROUTE_TURNS_STEP_M
    # TurnPilot: the controls in maneuverDistance's frame, the steps' distances and then along this step's geometry
    car_along = sum(s['distance'] for s in self.route[:self.step_idx]) + along_geometry
    # TurnPilot: the limit of the edge the car is on, tagged else its class default (turnpilotd's signal lead: 200 ft under 50 mph)
    msg.navInstruction.speedLimit = speed_limit_at(self.limits, car_along)
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
      elif self.step_idx + 1 < len(self.route) and self.went_straight_past():
        # TurnPilot (MISSED_TURN_MIN_DEG): the car is still on this step's road, past its turn: rerouted at the next update
        # (missed_off_route), never moved on to a step on a road it isn't on (1e 5:55.4)
        if not self.missed:
          cloudlog.event("navd.turn_missed_by_heading", step_idx=self.step_idx, distance=round(distance_to_maneuver_along_geometry, 1))
        self.missed = True
      elif self.step_idx + 1 < len(self.route):
        self.step_idx += 1
        self.recompute_hold = 0
      else:
        cloudlog.warning("Destination reached")

        # Clear route if driving away from destination
        if np.linalg.norm(local_meters(self.last_position, self.nav_destination)) > REROUTE_DISTANCE:
          self.params.remove("NavDestination")
          self.clear_route()

  def fill_position(self, ni):
    """TurnPilot (GPS loss): navInstruction's positionSource, positionUncertainty and gpsLostS (hold.py), on every message.
    And (route start, 2026-11-06) routeHeadingUnverified: the route waits on its start direction check (heading_unverified,
    heading_due), so turnpilotd can tell 'the route starts once the car moves its way' from off the route (a destination set
    standing, parked or nose-in in a space: invalid messages with the route kept, as a reroute's or a lost fix's are)."""
    ni.positionSource = self.hold.source
    ni.positionUncertainty = float(self.hold.uncertainty_m)
    ni.gpsLostS = float(self.hold.lost_s)
    ni.routeHeadingUnverified = bool(self.heading_unverified)

  def fill_off_route(self, msg):
    """TurnPilot: an invalid navInstruction's offRouteControls and offRouteControlsStatus. With a destination (rerouting, no
    route yet, off it for a fix), the last lookup ahead of the car, measured from where the car is now along the road it
    looked at, and a new lookup started (one at a time, off this loop). pending until the first is in; none with no
    destination, and nothing looked up. With the position lost (hold.py) there is no place to look ahead from: failed, the
    stops ahead unknown, never clear."""
    if self.requested_destination is None and self.nav_destination is None:
      self.reset_off_route()
      return
    if self.hold.lost:
      msg.navInstruction.offRouteControlsStatus = 'failed'
      return
    if self.off_status == 'none':
      self.off_status = 'pending'
    self.collect_ahead()
    # TurnPilot: nothing looked up ahead of the seed, a guess, nor offroad, where turnpilotd doesn't run (onroad)
    if self.ahead_job is None and self.last_position is not None and not self.seeded and self.onroad():
      self.ahead_job = AheadJob(self.router, self.last_position, self.own_heading())  # the road ahead of the car's nose
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
    self.missed = False  # TurnPilot: a turn missed on this route, never the next one's (it would lose a GPS hold, hold.py)
    self.heading_unverified, self.verify_from, self.route_heading_retry = False, None, False

  def on_next_step(self, lateral: float, distance_to_maneuver: float) -> bool:
    """The car is on the next step's geometry and off this one's: a sharp turn leaves the step sideways, and along it the
    car never gets past its end."""
    assert self.route is not None and self.route_geometry is not None and self.step_idx is not None
    if self.step_idx + 1 >= len(self.route) or distance_to_maneuver >= MANEUVER_TRANSITION_THRESHOLD:
      return False
    _, next_lateral = along_path(self.route_geometry[self.step_idx + 1], self.last_position)
    return next_lateral < NEXT_STEP_NEAR and lateral > next_lateral + NEXT_STEP_NEAR

  def car_heading(self) -> float | None:
    """TurnPilot: the car's own heading, for telling a turn taken from one missed: the hold's (hold.py: a moving fix's bearing
    with a good accuracy, turned with the car's yaw since), None without a live fix or a good heading, when navd moves on as
    it always did. A position set by hand has none."""
    return self.hold.heading if self.hold.position is not None else None

  def junction_bearings(self) -> tuple[float, float] | None:
    """TurnPilot: (deg into, deg out of) the junction that ends the current step: this step's road into its end and the next
    step's out of its start (chord_bearing), None at the route's last step or for a step with no length."""
    assert self.route_geometry is not None and self.step_idx is not None
    if self.step_idx + 1 >= len(self.route_geometry):
      return None
    before = chord_bearing(self.route_geometry[self.step_idx], from_end=True)
    after = chord_bearing(self.route_geometry[self.step_idx + 1], from_end=False)
    return None if before is None or after is None else (before, after)

  def turned_onto_next(self, distance_to_maneuver: float) -> bool:
    """TurnPilot (TURNED_ONTO_DEG): within MANEUVER_TRANSITION_THRESHOLD of the step's end, the car heads the next step's road
    and has left this one's, within NEXT_STEP_HEADING_NEAR of the next step's line: it made the turn."""
    heading = self.car_heading()
    if heading is None or distance_to_maneuver >= MANEUVER_TRANSITION_THRESHOLD:
      return False
    bearings = self.junction_bearings()
    if bearings is None:
      return False
    before, after = bearings
    if abs(wrap_deg(heading - after)) > TURNED_ONTO_DEG or abs(wrap_deg(heading - before)) <= LEFT_ROAD_DEG:
      return False
    place = step_place(self.route_geometry[self.step_idx + 1], self.last_position)
    if place is None or place[1] >= NEXT_STEP_HEADING_NEAR:
      return False
    cloudlog.event("navd.advanced_by_heading", step_idx=self.step_idx, distance=round(distance_to_maneuver, 1),
                   off_next=round(wrap_deg(heading - after), 1), next_lateral=round(place[1], 1))
    return True

  def went_straight_past(self) -> bool:
    """TurnPilot (MISSED_TURN_MIN_DEG): past the step's end, its maneuver turns and the car still heads this step's road, not the
    next one's: the turn was missed. Without the car's heading, or junction bearings, never (navd moves on as before)."""
    heading = self.car_heading()
    bearings = self.junction_bearings()
    if heading is None or bearings is None:
      return False
    before, after = bearings
    return abs(wrap_deg(after - before)) > MISSED_TURN_MIN_DEG and abs(wrap_deg(heading - after)) > MISSED_OFF_NEXT_DEG and \
      abs(wrap_deg(heading - before)) <= MISSED_ON_STEP_DEG

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
    # TurnPilot (GPS loss): standing, the car can't leave its route, so no fix counts toward a reroute (hold.py: GPS jitter at
    # a light rerouted from a wandering spot with a standstill bearing)
    if self.hold.standing:
      return False

    # TODO: Check for going wrong way in segment
    if along_path(self.route_geometry[self.step_idx], self.last_position)[1] > REROUTE_DISTANCE:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0
    return self.reroute_counter > REROUTE_COUNTER_MIN


def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute'])
  # TurnPilot: turnpilotPlan says when a turn was skipped (too fast), so navd reroutes past it at once; deviceState whether
  # the car is started (navd runs offroad too: RouteEngine.onroad); carState and deviceMotion carry the route through a GPS
  # loss (hold.py), absent offroad
  sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal', 'turnpilotPlan', 'deviceState', 'carState', 'deviceMotion'],
                           ignore_alive=['turnpilotPlan'],
                           ignore_avg_freq=['turnpilotPlan'], ignore_valid=['turnpilotPlan'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngine(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
