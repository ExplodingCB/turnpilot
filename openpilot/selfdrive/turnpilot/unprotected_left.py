"""The autonomous unprotected left (DESIGN phase 4 section 2, G4; CRITIQUE REQUIRED 2-5): a routed turn across the oncoming
traffic of the road ahead (map_controls.unprotected_turn), taken from its hold line with no gas tap.

Kellen, 2026-10-28: "No it has to ship taking the turn", and for this version no intervention. So while the car can judge
the junction it waits at the hold line for a gap it can certify, however long the traffic takes, as a careful driver does,
and never hands back for ordinary waiting. A handback (UNDECIDED: leftUndecidedTakeControl, the handback chime, the car
held) is only for a fault: road camera frames stale, the junction's position unknown, its reach past the detector's proven
range (route time should have rerouted it: turnpilotd), the car standing yawed past YAW_UNDECIDED_DEG, or the view still
blocked BLOCKED_UNDECIDED_S after the full creep. A missing frame is never clear.

TurnPilotAutoLeft (turnpilotd) picks the mode: auto (the default), announce (the car stops at the hold line and says
whether it would go, then hands back as before: debugging only), or off (the hold line's handback, G1's).

States: approach (no turn pulse before the hold, so the car stands straight with the road camera up the oncoming
approach), waitGap (held at the line), creep (for a view), go, committed, undecided; announce in that mode.

The gap rule (CRITIQUE REQUIRED 2) is on raw detections, never tracks: a distant car that never scores NEW_TRACK_CONF, or
is caught one run in three, never becomes a track, so track absence certifies nothing. Every road camera run's vehicle
boxes at RAW_CONF or more whose bearing lies in the oncoming band (band_blockers), in the road's frame (the car's turned
back by its heading change toward the turn), within R_req by their nearest plausible range, or unranged on the turn side
of straight ahead, break the clear window. It is velocity-free (one pixel of box bottom at 100 m is 7 m of range):
  T_req = GO_LATENCY_S + t_clear(path through the oncoming lanes on the left's post-go profile) + PET_S
          + EXTRA_LANE_S per opposing lane past the first (unknown: two)
  R_req = v_limit x SPEED_MARGIN x T_req + the conflict point's distance (the maneuver point + CONFLICT_PAST_M)
v_limit is the opposing road's mapped limit (navInstruction maneuverOpposingSpeed), DEFAULT_OPPOSING_SPEED when unknown.
A left whose R_req is past the detector's proven range (TurnPilotLeftRangeM) is never attempted: turnpilotd skips it at
route time (turn_skip: unprotectedRange, chimed, navd reroutes) and it is undecided here should it reach its hold.
The window (CLEAR_WINDOW_S) counts only road camera runs: a run gap over ROAD_FRESH_S, or the scene's holds, start it over.

The view is blocked by an occluder: a velocity-trusted vehicle track standing still in the junction (the opposing
left-turner, a parked truck) whose box hides the oncoming lanes beyond it out to R_req. Its own box isn't traffic (it
would never clear). Blocked, with the band otherwise clear, the car creeps (map_controls: CREEP_SPEED, CREEP_ACCEL) up to
CREEP_MAX_M past its line, never nearer than CREEP_SHORT_M short of the oncoming lanes (ONCOMING_ENTRY_M), and stops
creeping where the view clears or a vehicle comes into the band. The exit crosswalk and the junction box (a car standing
in the exit lane would leave the car in the oncoming lanes) must be clear too, and the scene holding nothing.

The go (map_controls releases its latch, cause gap; turnpilotd pulses the turn and drives it under the left's post-go
profile): the road camera only from the go to the commit (CRITIQUE REQUIRED 3; the wide camera one run in WIDE_EVERY
while held, for the exit crosswalk). Until the car can no longer stop short of the oncoming lanes on the comfortable curve
(scene_policy.committed_past_line), every road run is checked again for the remaining clear time, and at its first motion
(STANDSTILL_SPEED: CRITIQUE REQUIRED 4) with the latency gone: a car in the band then withdraws the go, the car stops
where the comfortable curve can (oncomingHolding, a chime) and waits again. Committed, nothing here ever stops the car (a
pedestrian in the path still does: scene_policy). A go the car hasn't moved on within turnpilotd's LEFT_NO_MOVE_S is
withdrawn and handed back there (the model holding the car; G5's bounded go is the remedy).
"""
import math
from dataclasses import dataclass, field

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.scene_policy import (CROSSWALK_X_AFTER_M, CROSSWALK_X_BEFORE_M, ONCOMING_ENTRY_M, ONCOMING_PAST_M,
                                                        ONCOMING_Y_FAR_M, ONCOMING_Y_NEAR_M, STOP_DECEL, VEHICLES, box_overlap,
                                                        committed_past_line, stop, velocity_trusted)

# T_req: the left's post-go profile (turn_speed.py: 'left'), the path from the line through the oncoming lanes
A_GO = 1.5                 # m/s^2
V_GO = 4.3                 # m/s
CLEAR_PATH_PAST_M = 8.     # the path is the distance to the maneuver point plus this (17 m from the 9 m line)
GO_LATENCY_S = 1.          # go to first motion: turnpilotd's LEFT_NO_MOVE_S withdraws a go not moving by then
PET_S = 2.                 # post-encroachment margin
EXTRA_LANE_S = 0.5         # per opposing lane past the first (AASHTO Case F)
UNKNOWN_LANES = 2          # opposing lanes when the map doesn't say
SPEED_MARGIN = 1.3         # x the mapped limit (CRITIQUE REQUIRED 4: 15 mph over a 35 mph limit needs ~1.4)
DEFAULT_OPPOSING_SPEED = 20.1  # m/s (45 mph) when the opposing road's limit isn't mapped
CONFLICT_PAST_M = 3.       # the first oncoming lane's conflict point, past the maneuver point
# the detector's proven range for oncoming traffic on the road camera at the YOLO input (TurnPilotLeftRangeM). PROVISIONAL
# and unmeasured on real comma frames: a car 1.8 m wide is ~6.5 px at the 640x384 input at 150 m. It takes a 25 mph left
# (R_req 134-141 m) and no faster one; 35 mph needs ~190 m (the far crop, or a measured range)
ROAD_RANGE_M = 150.

RAW_CONF = 0.35            # a raw box at the decoder's floor confidence counts (CRITIQUE REQUIRED 2)
BAND_TOL_DEG = 2.          # the band's near edge widens with range by this (heading error, the road's curve)
CLEAR_WINDOW_S = 2.5       # the band clear on road runs this long (CRITIQUE REQUIRED 2: 2-3 s, 3x the longest dropout)
ROAD_FRESH_S = 0.3         # road runs further apart than this start the window over
STALE_UNDECIDED_S = 2.     # no road run this long while waiting: a fault
YAW_UNDECIDED_DEG = 15.    # standing yawed further into the turn: the road camera isn't up the oncoming approach

STAND_SPEED = 1.           # m/s: a velocity-trusted track slower than this stands
OCCLUDER_PAST_M = 30.      # an occluder stands within this past the maneuver point...
OCCLUDER_Y_FAR_M = 12.     # ...and this far to the turn side
OCCLUDER_WIDTH_M = 2.
LANE_NEAR_M, LANE_FAR_M = 1.8, 9.   # the oncoming lanes it may hide, to the turn side
MATCH_IOU = 0.3            # a raw box is a track's in the same camera at this overlap...
MATCH_GATE_M = 2.          # ...or within this (plus MATCH_GATE_FRAC of its range) of its ground position
MATCH_GATE_FRAC = 0.15

CREEP_MAX_M = 4.           # past the line
CREEP_SHORT_M = 1.         # never nearer the oncoming lanes (ONCOMING_ENTRY_M) than this
CREEP_DONE_M = 0.3         # crept to within this of its target: the full creep
BLOCKED_UNDECIDED_S = 15.  # the view still blocked this long after the full creep: a fault

EXIT_Y_NEAR_M, EXIT_Y_FAR_M = 3., 18.  # the exit crosswalk, to the turn side, CROSSWALK_X_* around the maneuver point
BOX_X_BEFORE_M, BOX_X_AFTER_M = ONCOMING_PAST_M, 8.  # the junction box a vehicle mustn't be in...
BOX_Y_RIGHT_M, BOX_Y_LEFT_M = 12., 18.               # ...from this far to the other side to this far to the turn side

GO_PROMPT_S = 1.5          # leftGo is shown this long
ALERT_S = 2.               # oncomingHolding
VIEW_FROM_M = 60.          # the road camera from this far out (the window starts on the approach)

OFF, AUTO, ANNOUNCE = 'off', 'auto', 'announce'
MODES = (OFF, ANNOUNCE, AUTO)


@dataclass(frozen=True)
class RawBox:
  camera: str              # 'road' or 'wide'
  cls: str                 # TurnpilotScene.ObjectClass name
  conf: float
  bearing_deg: float       # the box's bottom centre, left of the calibrated forward axis
  span_deg: tuple          # (right edge, left edge) bearings
  xy: tuple | None         # ground point (x forward, y left), None unranged
  near_m: float | None     # nearest plausible range (the box bottom NEAR_PX lower), None unranged
  box: tuple = ()          # x0, y0, x1, y1 normalized in its camera's frame


@dataclass
class RawFrame:
  boxes: list
  camera: str
  t: float                 # the run's camera time, s


@dataclass
class LeftInput:
  mode: str
  maneuver_id: int
  side: int                # LEFT or RIGHT: the turn's side
  distance: float          # m to the maneuver point
  v_ego: float
  held: bool               # the hold line's latch has the car (map_controls)
  raw: RawFrame | None     # a detector run that arrived this frame, None if none did
  objects: list            # the scene's tracks (TurnpilotScene.TrackedObject), car frame
  heading_deg: float       # the car's heading change toward the turn since its approach (turnpilotd.track_yaw)
  v_limit: float | None    # m/s, the opposing road's mapped limit, None unknown
  opposing_lanes: int | None
  range_m: float = ROAD_RANGE_M
  scene_clear: bool = True       # the scene held nothing last frame
  junction_uncertain: bool = False
  go_withdrawn: bool = False     # turnpilotd withdrew the go (not moving): its handback
  committed: bool = False        # the scene committed to (or was released into) this junction


@dataclass
class LeftDecision:
  state: str = 'off'
  go: bool = False                # release the hold now
  creep_m: float = 0.             # the hold's stop point this far past where it latched
  view: str = 'none'              # TurnpilotPlan.DetectorView
  prompt: str = 'none'            # TurnpilotPlan.ControlPrompt
  alert: str = 'none'             # TurnpilotPlan.ControlAlert
  reason: str = ''
  stops: list = field(default_factory=list)  # its own hold after a withdrawn go
  t_req: float = 0.
  r_req: float = 0.
  v_limit: float = 0.
  clear_s: float = 0.
  view_blocked: bool = False
  crosswalk_clear: bool = True
  blockers: int = 0
  verdict: str = ''               # announce: 'go' or 'wait'
  why: str = ''                   # undecided: its fault


def t_clear(path_m: float) -> float:
  """s from first motion to cover path_m at A_GO up to V_GO."""
  d_ramp = V_GO ** 2 / (2. * A_GO)
  if path_m <= d_ramp:
    return math.sqrt(2. * max(path_m, 0.) / A_GO)
  return V_GO / A_GO + (path_m - d_ramp) / V_GO


def t_req(distance: float, lanes: int | None, latency: bool = True) -> float:
  """s the oncoming lanes must stay clear for a go from distance m before the maneuver point (module docstring)."""
  lanes = UNKNOWN_LANES if lanes is None or lanes < 1 else lanes
  return (GO_LATENCY_S if latency else 0.) + t_clear(max(distance, 0.) + CLEAR_PATH_PAST_M) + PET_S + EXTRA_LANE_S * (lanes - 1)


def r_req(v_limit: float, t: float, distance: float) -> float:
  """m up the oncoming lanes that must be seen clear: what a car at SPEED_MARGIN x v_limit covers in t, from the conflict point."""
  return v_limit * SPEED_MARGIN * t + max(distance, 0.) + CONFLICT_PAST_M


def turn_bearing(bearing_deg: float, heading_deg: float, side: int) -> float:
  """A car-frame bearing in the road's frame, positive toward the turn."""
  return bearing_deg * side + heading_deg


def band_blockers(boxes, heading_deg: float, side: int, reach_m: float, distance: float, exempt=()) -> list:
  """The road camera's raw vehicle boxes at RAW_CONF or more in the oncoming band within reach_m (module docstring).
  exempt: boxes that are an occluder's (not traffic)."""
  out = []
  for b in boxes:
    if b.camera != 'road' or b.cls not in VEHICLES or b.conf < RAW_CONF or b in exempt:
      continue
    edges = sorted(turn_bearing(e, heading_deg, side) for e in b.span_deg)
    tol = math.radians(BAND_TOL_DEG)
    if b.xy is None or b.near_m is None:
      if edges[1] >= -BAND_TOL_DEG:  # unranged: anywhere on the turn side of straight ahead may be the oncoming lanes
        out.append(b)
      continue
    rng = math.hypot(*b.xy)
    lat_far, lat_near = rng * math.sin(math.radians(edges[1])), rng * math.sin(math.radians(edges[0]))
    x = rng * math.cos(math.radians(turn_bearing(b.bearing_deg, heading_deg, side)))
    if b.near_m <= reach_m and x > distance - ONCOMING_PAST_M and lat_far > ONCOMING_Y_NEAR_M - rng * math.tan(tol) and \
       lat_near < ONCOMING_Y_FAR_M + rng * math.tan(tol):
      out.append(b)
  return out


def road_xy(obj, heading_deg: float, side: int) -> tuple[float, float]:
  """A track's ground position in the road's frame: (x along the approach, y toward the turn)."""
  a = math.radians(heading_deg) * side
  x, y = obj.x * math.cos(a) - obj.y * math.sin(a), obj.x * math.sin(a) + obj.y * math.cos(a)
  return x, y * side


def occluders(objects, heading_deg: float, side: int, distance: float) -> list:
  """Vehicle tracks standing in the junction on the turn side (module docstring): velocity trusted, slower than STAND_SPEED."""
  out = []
  for obj in objects:
    if str(obj.cls) not in VEHICLES or not velocity_trusted(obj) or math.hypot(obj.vx, obj.vy) >= STAND_SPEED:
      continue
    x, y = road_xy(obj, heading_deg, side)
    if distance - ONCOMING_PAST_M <= x <= distance + OCCLUDER_PAST_M and ONCOMING_Y_NEAR_M <= y <= OCCLUDER_Y_FAR_M:
      out.append(obj)
  return out


def matches(b: RawBox, obj) -> bool:
  """A raw box is this track's: overlapping boxes in the same camera, or ground positions within the gate."""
  if str(obj.camera) == b.camera and len(obj.box) == 4 and len(b.box) == 4 and box_overlap(tuple(obj.box), b.box) >= MATCH_IOU:
    return True
  if b.xy is None:
    return False
  return math.hypot(b.xy[0] - obj.x, b.xy[1] - obj.y) <= MATCH_GATE_M + MATCH_GATE_FRAC * math.hypot(*b.xy)


def view_blocked(occ, heading_deg: float, side: int, reach_m: float) -> bool:
  """An occluder's bearings cover some of the oncoming lanes beyond it, out to reach_m."""
  for obj in occ:
    x, y = road_xy(obj, heading_deg, side)
    if x <= 0.:
      continue
    lo, hi = math.atan2(y - OCCLUDER_WIDTH_M / 2., x), math.atan2(y + OCCLUDER_WIDTH_M / 2., x)
    lanes_lo, lanes_hi = math.atan2(LANE_NEAR_M, reach_m), math.atan2(LANE_FAR_M, x)
    if lo < lanes_hi and hi > lanes_lo:
      return True
  return False


def exit_clear(objects, heading_deg: float, side: int, distance: float, occ) -> bool:
  """No pedestrian in the exit crosswalk, and no vehicle but an occluder in the junction box (module docstring)."""
  ids = {o.trackId for o in occ}
  for obj in objects:
    x, y = road_xy(obj, heading_deg, side)
    rel = x - distance
    if str(obj.cls) == 'pedestrian' and -CROSSWALK_X_BEFORE_M <= rel <= CROSSWALK_X_AFTER_M and EXIT_Y_NEAR_M <= y <= EXIT_Y_FAR_M:
      return False
    if str(obj.cls) in VEHICLES and obj.trackId not in ids and -BOX_X_BEFORE_M <= rel <= BOX_X_AFTER_M and \
       -BOX_Y_RIGHT_M <= y <= BOX_Y_LEFT_M:
      return False
  return True


class UnprotectedLeft:
  def __init__(self):
    self.maneuver_id: int | None = None
    self._reset()

  def _reset(self):
    self.state = 'approach'
    self.clear_s = 0.
    self.road_age = math.inf      # s since the last road camera run
    self.blocked = False          # the last road run had a car in the band
    self.latch_d: float | None = None  # m to the maneuver point where the hold latched
    self.creep_to = 0.
    self.blocked_s = 0.           # the view blocked with the full creep done
    self.went = False             # gone (not withdrawn since)
    self.moved = False            # ...and moved
    self.go_t = 0.
    self.alert_t = 0.
    self.why = ''
    self.verdict = ''
    self.hold_here = False        # its own hold, after a withdrawn go

  def update(self, inp: LeftInput, dt: float) -> LeftDecision:
    if inp.mode not in (AUTO, ANNOUNCE):
      self.maneuver_id = None
      return LeftDecision()
    if inp.maneuver_id != self.maneuver_id:
      self.maneuver_id = inp.maneuver_id
      self._reset()
    side = inp.side
    v_limit = inp.v_limit if inp.v_limit is not None and inp.v_limit > 0. else DEFAULT_OPPOSING_SPEED
    latency = not self.moved
    treq = t_req(inp.distance, inp.opposing_lanes, latency)
    rreq = r_req(v_limit, treq, inp.distance)
    occ = occluders(inp.objects, inp.heading_deg, side, inp.distance)
    self.road_age += dt
    raw_blockers = None
    if inp.raw is not None and inp.raw.camera == 'road':
      exempt = [b for b in inp.raw.boxes if any(matches(b, o) for o in occ)]
      raw_blockers = band_blockers(inp.raw.boxes, inp.heading_deg, side, rreq, inp.distance, exempt)
      fresh_run = self.road_age <= ROAD_FRESH_S
      self.road_age = 0.
      self.blocked = bool(raw_blockers)
      self.clear_s = 0. if self.blocked or not fresh_run else self.clear_s + dt
    elif self.road_age > ROAD_FRESH_S or self.blocked or not inp.scene_clear:
      self.clear_s = 0.
    else:
      self.clear_s += dt
    if not inp.scene_clear:
      self.clear_s = 0.
    blocked_view = view_blocked(occ, inp.heading_deg, side, rreq)
    if blocked_view:
      self.clear_s = 0.  # the lanes behind the occluder weren't seen: the window counts only with the view open
    crosswalk = exit_clear(inp.objects, inp.heading_deg, side, inp.distance, occ)
    out = LeftDecision(t_req=treq, r_req=rreq, v_limit=v_limit, clear_s=self.clear_s, view_blocked=blocked_view,
                       crosswalk_clear=crosswalk, blockers=len(raw_blockers or ()))
    self.alert_t = max(self.alert_t - dt, 0.)
    if self.alert_t > 0.:
      out.alert = 'oncomingHolding'

    if self.state == 'committed' or inp.committed or (self.went and committed_past_line(inp.distance, inp.v_ego, True)):
      if self.state != 'committed':
        cloudlog.event("turnpilot.left_committed", distance=round(inp.distance, 1), v=round(inp.v_ego, 2))
      self.state, self.hold_here = 'committed', False
      out.state, out.alert = 'committed', 'none'
      return out
    if self.state == 'undecided':
      return self._undecided(out, self.why)

    if self.went:
      return self._after_go(inp, out, raw_blockers, dt)

    certified = self.clear_s >= CLEAR_WINDOW_S - 1e-6 and not blocked_view and crosswalk and inp.scene_clear
    standing = inp.held or self.hold_here
    if not standing:
      out.state = self.state = 'approach'
      out.view = 'road' if inp.distance <= VIEW_FROM_M else 'none'
      return out
    if self.latch_d is None:
      self.latch_d = inp.distance
    out.view = 'roadAndWide'
    # faults: never a go it can't judge
    if rreq > inp.range_m:
      return self._undecided(out, 'range')
    if inp.junction_uncertain:
      return self._undecided(out, 'position')
    if self.road_age > STALE_UNDECIDED_S:
      return self._undecided(out, 'stale')
    if inp.heading_deg > YAW_UNDECIDED_DEG:
      return self._undecided(out, 'yaw')

    if inp.mode == ANNOUNCE:
      verdict = 'go' if certified else 'wait'
      if verdict != self.verdict:
        self.verdict = verdict
        cloudlog.event("turnpilot.left_announce", verdict=verdict, clear_s=round(self.clear_s, 1), r_req=round(rreq, 1),
                       view_blocked=blocked_view, crosswalk=crosswalk)
      out.state, out.verdict = 'announce', verdict
      out.reason = "Unprotected left: would go" if verdict == 'go' else "Unprotected left: would wait"
      return out

    crept = self.latch_d - inp.distance
    creep_max = min(CREEP_MAX_M, max(self.latch_d - (ONCOMING_ENTRY_M + CREEP_SHORT_M), 0.))
    if certified:
      return self._go(inp, out, rreq)
    if blocked_view and not self.blocked and not self.hold_here:
      self.creep_to = creep_max
      done = crept >= creep_max - CREEP_DONE_M
      self.blocked_s = self.blocked_s + dt if done and inp.v_ego < 0.3 else 0.
      if self.blocked_s >= BLOCKED_UNDECIDED_S - 1e-6:
        return self._undecided(out, 'blocked')
      out.state = self.state = 'creep'
      out.prompt, out.reason, out.creep_m = 'creepingForView', "Creeping for a view", self.creep_to
      return out
    # waiting: crept no further than it has (a car in the band stops the creep where it is)
    self.blocked_s = 0.
    self.creep_to = min(self.creep_to, max(crept, 0.))
    out.state = self.state = 'waitGap'
    out.prompt, out.reason, out.creep_m = 'waitingForGap', "Waiting for a gap", self.creep_to
    if self.hold_here:
      out.stops = [stop(inp.v_ego, 0., "Oncoming, holding", confirmed=False)]
    return out

  def _go(self, inp: LeftInput, out: LeftDecision, rreq: float) -> LeftDecision:
    self.went, self.moved, self.hold_here, self.go_t = True, False, False, 0.
    cloudlog.event("turnpilot.left_go", distance=round(inp.distance, 1), r_req=round(rreq, 1), clear_s=round(self.clear_s, 1),
                   v_limit=round(out.v_limit, 1), heading_deg=round(inp.heading_deg, 1))
    out.state = self.state = 'go'
    out.go, out.view, out.prompt, out.reason = True, 'road', 'leftGo', "Turning"
    return out

  def _after_go(self, inp: LeftInput, out: LeftDecision, raw_blockers, dt: float) -> LeftDecision:
    """Gone, not committed: the road camera only, and the band checked on every road run for the remaining clear time; at
    the first motion with the latency gone (module docstring)."""
    self.go_t += dt
    out.state, out.view = 'go', 'road'
    if self.go_t < GO_PROMPT_S:
      out.prompt, out.reason = 'leftGo', "Turning"
    if inp.go_withdrawn:  # turnpilotd's: not moving, held and handed back there
      return out
    first_motion = not self.moved and inp.v_ego >= 0.3
    self.moved = self.moved or first_motion
    if not raw_blockers:
      return out
    stop_m = inp.v_ego ** 2 / (2. * STOP_DECEL)
    if stop_m >= inp.distance - ONCOMING_ENTRY_M:
      return out  # can't stop short of the oncoming lanes: on (committed_past_line commits it next)
    cloudlog.event("turnpilot.left_go_withdrawn", error=True, first_motion=first_motion, distance=round(inp.distance, 1),
                   v=round(inp.v_ego, 2), blockers=len(raw_blockers), r_req=round(out.r_req, 1))
    self.went, self.moved, self.hold_here, self.clear_s = False, False, True, 0.
    self.alert_t = ALERT_S
    self.latch_d = inp.distance
    out.state = self.state = 'waitGap'
    out.alert, out.prompt, out.reason, out.view, out.clear_s = 'oncomingHolding', 'waitingForGap', "Oncoming, holding", 'roadAndWide', 0.
    out.stops = [stop(inp.v_ego, stop_m, "Oncoming, holding", confirmed=False)]
    return out

  def _undecided(self, out: LeftDecision, why: str) -> LeftDecision:
    if self.state != 'undecided':
      cloudlog.event("turnpilot.left_undecided", error=True, why=why, r_req=round(out.r_req, 1), clear_s=round(out.clear_s, 1))
    self.state, self.why = 'undecided', why
    out.state, out.why, out.view = 'undecided', why, 'roadAndWide'
    out.prompt, out.reason = 'leftUndecidedTakeControl', UNDECIDED_REASON.get(why, "Can't judge oncoming traffic: take control")
    if self.hold_here:
      out.stops = [stop(0., 0., "Oncoming, holding", confirmed=False)]
    return out


UNDECIDED_REASON = {
  'range': "Oncoming traffic too fast to judge: take control",
  'position': "Junction position unsure: take control",
  'stale': "Camera not seeing oncoming traffic: take control",
  'yaw': "Not facing oncoming traffic: take control",
  'blocked': "View of oncoming traffic blocked: take control",
}


def left_mode_for(value) -> str:
  """TurnPilotAutoLeft's value as a mode; AUTO for anything unset or unknown (turnpilotd logs it)."""
  value = value.decode() if isinstance(value, bytes) else value
  return value if value in MODES else AUTO

