"""TurnPilot: navd's position through a GPS loss, and the route held with it (navInstruction.positionSource).

On the first real drive the GPS went at a red light before a left (an eGPU cable near the antenna is the suspect) and
TurnPilot dropped back to plain openpilot. navd took any fix with hasFix set, however poor, and a degraded fix at a standstill
wanders tens of metres: past OFF_ROUTE_DISTANCE off the step its message went invalid and turnpilotd dropped the left (only a
committed turn is held through a gap), REROUTE_COUNTER_MIN of them rerouted from the wandering spot with a standstill bearing,
and a fix past the step's end moved navd on to the next step with the car still at the line. With no fix at all the position
froze, so once the car pulled away navd's distances stood still while it drove the turn. So:
- A fix is used only when trusted: latest_gps_source's (valid, hasFix, under 2 s old) and a horizontalAccuracy of at most
  GPS_ACCURACY_MAX_M (0, not reported, is used as before). Past that a fix can't tell the car on its road from beside it.
- Standing still (carState.vEgo under STANDSTILL_V) the car's position can't change, so no fix moves it once there is one:
  GPS jitter at a light never puts the car off its route, past its maneuver or on another step. The only fix taken standing
  is the first (navd routes from nothing else), or one trusted when the position came from an untrusted first fix.
- Moving with no trusted fix, the position is carried along the route by the car's own travel (vEgo, trapezoid over navd's
  update interval, measured), on the route's own polyline and with its bearing: navd's steps advance as the car drives
  them, the turn included. Along the route and not by integrating yaw: a yaw-integrated point drifts off the step's
  polyline, and off it navd's distances go invalid, the very dropout here. Along it, the error is the odometer's and the car's
  line through a junction against the polyline's corner, DRIFT per m. The car's own heading (deviceMotion's yaw: an integrated
  angle, so navd's 1 Hz sample of it loses nothing) is checked against the route's: turned MISMATCH_DEG or more away from it
  over MISMATCH_M of travel, the car has left the route (a turn missed, a driveway) and the hold is lost. So it is on
  turnpilotd's turnMissed (navd's note_skip): the car went straight past the turn the hold carried it round.
- The hold ends lost past HOLD_MAX_M of travel or HOLD_MAX_S of moving time since the last trusted fix. HOLD_MAX_M is where
  its along-route error (DRIFT per m, from the last fix's accuracy) reaches route_follower.UNCERTAIN_M, the spread at which
  turnpilotd already says a junction is uncertain; HOLD_MAX_S bounds a crawl, where the travel stays short but the GPS is out
  for good (a garage, the interference), and the driver should hear it before the next turn, not at it. Standing time never
  counts: standing, the position is exact. Lost, the position stays where the hold put it, navd's message is invalid with the
  route kept, and turnpilotd hands back loudly (gpsLostTakeControl): never a silent drop to stock.
- Fixes coming back: REACQUIRE_FIXES trusted ones in a row before the position comes from them again (the first after an
  outage are the receiver's worst), then at once, taken while moving: navd's own off-route and reroute checks judge where it
  puts the car (a turn missed in the outage reroutes there).
- The heading navd routes with (heading: Valhalla's start heading, HEADING_TOLERANCE 90 deg either side, the lookups ahead and
  the car's own edge) is a fix's bearing only when the car moves HEADING_MIN_V or faster (carState's vEgo, else the fix's own
  speed) and the fix reports a bearingAccuracyDeg of at most HEADING_ACCURACY_MAX_DEG: standing, or just after a fix comes
  back, a u-blox heading of motion is stale or 0 (north), which put the start on the wrong edge and routed oddly after a
  dropout. Between such fixes the last good one stands (standing, the car can't turn) and turns with the car's own yaw
  (deviceMotion); with no yaw to turn it by, HEADING_STALE_M of travel ends it, and navd routes with no heading at all.
Without a live carState (offroad, a replay without it) nothing here knows whether the car moves: every trusted fix is taken,
and without one the position stays, as navd did before."""
import math
from typing import NamedTuple

import numpy as np

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import accuracy_of, latest_gps_source
from openpilot.selfdrive.navigation.route import cumulative_distance, local_meters, nearest_segment

GPS_ACCURACY_MAX_M = 25.  # m, a fix's horizontalAccuracy past this isn't used (navd's OFF_ROUTE_DISTANCE is 20)
STANDSTILL_V = 0.3        # m/s, under this the car stands: its position can't change
DRIFT = 0.025             # m of along-route error per m carried: the odometer's scale and a junction's line against the corner
UNCERTAINTY_MIN_M = 3.    # m, the floor of a fix's own accuracy (0 is not reported)
HOLD_MAX_M = 480.         # m carried before lost: 3 m + DRIFT * 480 = 15 m, route_follower.UNCERTAIN_M
HOLD_MAX_S = 120.         # s moving without a trusted fix before lost
MISMATCH_DEG = 50.        # the car's heading this far from the route's (a normal turn's peaks at half its angle, at its node)...
MISMATCH_M = 30.          # ...over this much travel: the car left the route (a 90 deg turn's span over 50 deg is a few metres)
REACQUIRE_FIXES = 2       # trusted fixes in a row that end a hold
MAX_DT = 5.               # s, a longer gap between navd's updates is carried as this (a stalled loop, not a drive)
YAW_STEP_MAX_DEG = 90.    # a yaw step larger than this between two updates is locationd restarting, not the car
NEXT_STEP_M = 5.          # a hold starting this much nearer the next step's polyline than its own starts on the next
HEADING_MIN_V = 3.        # m/s, a fix's bearing (heading of motion) is used for routing only this fast or faster...
HEADING_ACCURACY_MAX_DEG = 20.  # ...reporting a bearing accuracy within this (0, or qcomgpsd's 180, is none reported)
HEADING_STALE_M = 30.     # m travelled with no yaw to turn the last good heading by, and it is no heading

GPS, DEAD_RECKONED, LOST = 'gps', 'deadReckoned', 'lost'  # navInstruction.positionSource


class Fix(NamedTuple):
  position: tuple  # (longitude, latitude)
  bearing: float   # deg
  accuracy: float  # m, 0 not reported
  speed: float = 0.             # m/s, the fix's own
  bearing_accuracy: float = 0.  # deg, 0 not reported


class Motion(NamedTuple):
  v: float               # m/s, carState.vEgo
  yaw: float | None      # deg clockwise, deviceMotion's orientationNED yaw, None without a valid one


def gps_fix(sm) -> Fix | None:
  """The freshest trusted fix of navd's two GPS services, else the freshest fix at all (untrusted: it counts only for navd's
  first position), else None."""
  source = latest_gps_source(sm, GPS_ACCURACY_MAX_M) or latest_gps_source(sm)
  if source is None:
    return None
  gps = sm[source]
  return Fix((gps.longitude, gps.latitude), gps.bearingDeg, accuracy_of(gps), float(getattr(gps, 'speed', 0.) or 0.),
             float(getattr(gps, 'bearingAccuracyDeg', 0.) or 0.))


def trusted(fix: Fix | None) -> bool:
  return fix is not None and all(math.isfinite(x) for x in fix.position) and fix.accuracy <= GPS_ACCURACY_MAX_M


def course_good(fix: Fix | None, motion: Motion | None) -> bool:
  """Whether fix's bearing is a GPS course the heading navd routes with is taken from (module docstring): a trusted fix, the
  car moving HEADING_MIN_V or faster (carState's vEgo, else the fix's own speed without a live carState), a finite bearing
  and a bearing accuracy within HEADING_ACCURACY_MAX_DEG (0 is none reported). PositionHold's heading becomes the fix's bearing
  exactly when this says so, and navd's HeadingBasis (parked onroad, COURSE_TURN_MAX_DEG) starts a course on it: one test,
  so the two can't drift apart (review of 11-06: navd had mirrored it as course_taken, kept in step by a test alone)."""
  if not trusted(fix):
    return False
  assert fix is not None
  v = motion.v if motion is not None else fix.speed
  return v >= HEADING_MIN_V and math.isfinite(fix.bearing) and 0. < fix.bearing_accuracy <= HEADING_ACCURACY_MAX_DEG


def car_motion(sm) -> Motion | None:
  """The car's speed and heading from navd's SubMaster, None without a live, valid carState."""
  if 'carState' not in sm.data or not sm.alive['carState'] or not sm.valid['carState']:
    return None
  yaw = None
  if 'deviceMotion' in sm.data and sm.alive['deviceMotion'] and sm.valid['deviceMotion'] and sm['deviceMotion'].orientationNED.valid:
    yaw = math.degrees(float(sm['deviceMotion'].orientationNED.z))
  return Motion(float(sm['carState'].vEgo), yaw)


def wrap_deg(a: float) -> float:
  return (a + 180.) % 360. - 180.


class RoutePath:
  """A route's steps end to end, for a place along it: m along it of a point on a step, and the point and bearing at m."""

  def __init__(self, geometry):
    self.geometry = geometry
    self.starts = np.cumsum([0] + [len(g) for g in geometry[:-1]])  # each step's first point in the joined path
    self.path = np.concatenate(geometry) if len(geometry) else np.empty((0, 2))
    self.cum = cumulative_distance(self.path) if len(self.path) >= 2 else np.zeros(len(self.path))
    # the segments with a length: a step's last point repeats as the next one's first, and the arrival's step is one point twice
    self.segments = np.flatnonzero(np.diff(self.cum) > 1e-6)

  @property
  def length(self) -> float:
    return float(self.cum[-1]) if len(self.cum) else 0.

  def along(self, step_idx: int, point) -> float | None:
    """m along the route of the place nearest point on step step_idx's polyline, or on the next step's when that is
    NEXT_STEP_M nearer (navd moves on only 10 m past a step's end, and the car may be that far on into the turn); None
    without a segment on either."""
    best = None
    for i, margin in ((step_idx, 0.), (step_idx + 1, NEXT_STEP_M)):
      g = self.geometry[i] if i < len(self.geometry) else ()
      if len(g) < 2:
        continue
      local, p = local_meters(g, g[0]), local_meters(point, g[0])
      j, t, lateral = nearest_segment(local[:-1], local[1:], p)
      k = int(self.starts[i]) + j
      if best is None or lateral + margin < best[1]:
        best = (float(self.cum[k] + t * (self.cum[k + 1] - self.cum[k])), lateral)
    return best[0] if best is not None and len(self.segments) else None

  def at(self, along: float) -> tuple[tuple, float]:
    """((longitude, latitude), bearing deg) along m along the route, clipped to its ends; the bearing is the segment's under
    it (the last one with a length at the end)."""
    along = min(max(along, 0.), self.length)
    k = int(self.segments[min(max(int(np.searchsorted(self.cum[self.segments], along, side='right')) - 1, 0), len(self.segments) - 1)])
    a, b = self.path[k], self.path[k + 1]
    t = min(max((along - self.cum[k]) / (self.cum[k + 1] - self.cum[k]), 0.), 1.)
    point = a + t * (b - a)
    east, north = local_meters(b, a)
    return (float(point[0]), float(point[1])), math.degrees(math.atan2(east, north)) % 360.


class PositionHold:
  """navd's position (module docstring): a trusted fix's, held standing, carried along the route without one, lost past its
  limits. update() once per navd update; position, bearing, source and uncertainty_m are its result."""

  def __init__(self):
    self.position: tuple | None = None
    self.bearing: float | None = None
    self.source = GPS
    self.accuracy = UNCERTAINTY_MIN_M  # m, the last trusted fix's
    self.held_m = self.held_s = 0.     # carried and moving time since the last trusted fix
    self.lost_s = 0.                   # s since the last trusted fix, standing included (gpsLostS)
    self.why = ''                      # what lost the hold: 'limit', 'leftRoute' or 'turnMissed'
    self.standing = False              # the car stood since the last update (a live carState under STANDSTILL_V, then and now)
    self.motion_known = False          # a live carState: whether the car moves is known
    self.reacquire = 0                 # trusted fixes in a row since the hold began
    self.untrusted_start = False       # the position came from an untrusted first fix
    self.heading: float | None = None  # deg, what navd routes with (module docstring), None: no good one
    self._heading_m = 0.               # m travelled since it was last good with no yaw to turn it by
    self._route: RoutePath | None = None
    self._along: float | None = None   # m along _route of the carried place
    self._turn = 0.                    # deg the car turned since the hold began, less what the route turned
    self._route_bearing: float | None = None
    self._mismatch_m = 0.
    self._t: float | None = None
    self._v: float | None = None
    self._yaw: float | None = None

  @property
  def uncertainty_m(self) -> float:
    return max(self.accuracy, UNCERTAINTY_MIN_M) + DRIFT * self.held_m

  @property
  def lost(self) -> bool:
    return self.source == LOST

  def update(self, fix: Fix | None, motion: Motion | None, now: float, geometry=None, step_idx: int | None = None,
             missed: bool = False) -> None:
    """fix: the freshest GPS fix (gps_fix), trusted or not; motion: car_motion's; now: monotonic s; geometry, step_idx: navd's
    route (route_geometry) and its step, None without one; missed: turnpilotd said a turn was missed (navd's note_skip)."""
    dt = min(max(now - self._t, 0.), MAX_DT) if self._t is not None else 0.
    self._t = now
    ds, yaw_step = 0., None
    self.motion_known = motion is not None
    if motion is not None:
      v = max(motion.v, 0.)
      v_was = self._v if self._v is not None else v
      ds = (v + v_was) / 2. * dt
      if motion.yaw is not None and self._yaw is not None and abs(wrap_deg(motion.yaw - self._yaw)) <= YAW_STEP_MAX_DEG:
        yaw_step = wrap_deg(motion.yaw - self._yaw)
      # standing only once the car stood the whole interval: the fix that ends the stop's last second of travel is where it stopped
      self.standing = v < STANDSTILL_V and v_was < STANDSTILL_V
      self._v, self._yaw = v, motion.yaw
    else:
      self._v = self._yaw = None
      self.standing = False
    ok = trusted(fix)
    self._update_heading(fix, motion, ds, yaw_step)

    if self.position is None:
      if fix is not None:  # the first position, from whatever there is: navd routes from nothing else
        self._take(fix, trusted_fix=ok)
      return
    if ok and (motion is None or self.untrusted_start):
      self._take(fix)  # nothing knows whether the car moves (navd before the hold), or the first fix was untrusted
      return
    if ok and self.source == GPS:
      self.lost_s = 0.
      if not self.standing:
        self._take(fix)
      return  # standing: the position can't change, whatever the fix says
    if ok:
      self.reacquire += 1
      if self.reacquire >= REACQUIRE_FIXES and not self.standing:
        self._reacquired(fix)
        return
    else:
      self.reacquire = 0
    self._carry(ds, dt, yaw_step, geometry, step_idx, missed)

  def _update_heading(self, fix: Fix | None, motion: Motion | None, ds: float, yaw_step: float | None) -> None:
    """heading (module docstring): a good fix's bearing (course_good), else the last good one turned with the car, or none."""
    if course_good(fix, motion):
      assert fix is not None
      self.heading, self._heading_m = fix.bearing % 360., 0.
    elif self.heading is not None and yaw_step is not None:
      self.heading = (self.heading + yaw_step) % 360.
    elif self.heading is not None:
      self._heading_m += ds
      if self._heading_m > HEADING_STALE_M:
        self.heading = None

  def _take(self, fix: Fix, trusted_fix: bool = True) -> None:
    self.position, self.bearing = fix.position, fix.bearing
    self.accuracy = fix.accuracy if trusted_fix else max(fix.accuracy, GPS_ACCURACY_MAX_M)
    self.untrusted_start = not trusted_fix
    self.source, self.why = GPS, ''
    self.held_m = self.held_s = self.lost_s = 0.
    self.reacquire = 0
    self._route, self._along, self._turn, self._route_bearing, self._mismatch_m = None, None, 0., None, 0.

  def _reacquired(self, fix: Fix) -> None:
    jump = float(np.linalg.norm(local_meters(fix.position, self.position)))
    cloudlog.event("navd.gps_reacquired", was=self.source, why=self.why, held_m=round(self.held_m, 1), held_s=round(self.held_s, 1),
                   lost_s=round(self.lost_s, 1), jump_m=round(jump, 1), accuracy=round(fix.accuracy, 1))
    self._take(fix)

  def _carry(self, ds: float, dt: float, yaw_step: float | None, geometry, step_idx: int | None, missed: bool) -> None:
    """No trusted fix: the position carried along the route by the car's travel (module docstring), or lost."""
    if self.source == GPS:
      self.source = DEAD_RECKONED
      cloudlog.event("navd.gps_lost", accuracy=round(self.accuracy, 1), standing=self.standing, route=geometry is not None)
    self.lost_s += dt
    if self.source == LOST:
      return  # where the hold left it, until fixes come back
    if missed:
      return self._lose('turnMissed')
    if not self.motion_known:
      return  # nothing knows whether the car moves: the place stays, as navd's did before the hold
    self.held_m += ds
    if not self.standing:
      self.held_s += dt
    if geometry is not None and step_idx is not None:
      if self._route is None or self._route.geometry is not geometry:  # a new route (a reroute that landed): placed on it
        self._route = RoutePath(geometry)
        self._along = self._route.along(step_idx, self.position)
        self._route_bearing = self._route.at(self._along)[1] if self._along is not None else None
      if self._along is not None:
        self._along += ds
        self.position, bearing = self._route.at(self._along)
        if yaw_step is not None and self._route_bearing is not None:
          self._turn = wrap_deg(self._turn + yaw_step - wrap_deg(bearing - self._route_bearing))
          self._mismatch_m = self._mismatch_m + ds if abs(self._turn) >= MISMATCH_DEG else 0.
        self.bearing = self._route_bearing = bearing
    if self._mismatch_m >= MISMATCH_M:
      return self._lose('leftRoute')
    if self.held_m > HOLD_MAX_M or self.held_s > HOLD_MAX_S:
      return self._lose('limit')

  def _lose(self, why: str) -> None:
    self.source, self.why = LOST, why
    cloudlog.event("navd.gps_hold_lost", error=True, why=why, held_m=round(self.held_m, 1), held_s=round(self.held_s, 1),
                   lost_s=round(self.lost_s, 1), turn_deg=round(self._turn, 1), uncertainty_m=round(self.uncertainty_m, 1))
