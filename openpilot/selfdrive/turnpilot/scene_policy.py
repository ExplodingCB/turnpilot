"""Scene policy: tracked pedestrians and cars into longitudinal limits.

The driving model keeps lane keeping, leads, lights and the turn itself. This covers what it doesn't: a
pedestrian in or entering the path, a pedestrian at the crosswalk a curb-side turn crosses, and oncoming
traffic at a turn across it (every such turn is treated as unprotected). Each becomes a stop point; the
tightest one wins and is turned into a speed cap on a comfortable braking curve, plus an accel cap when that
curve can't make it. shouldStop is only set at the stop point near standstill, because longcontrol's stopping
state ignores the accel target and ramps the brakes on its own.

A track is evidence, not fact: a stop that needs harder braking than the curve only gets it from a confirmed track
(enough hits at a high enough confidence, or many hits at any, CONFIRM_RECENT_HITS of them in the last CONFIRM_WINDOW_S,
and published on every frame since: ScenePolicy.update_evidence), so a two-hit false positive slows the car on the curve
instead of stopping it hard, and the accel cap ramps in at a bounded jerk rather than stepping. A confirmed person moving
along the path is followed like a lead, not stopped for, and one on a confirmed, moving bicycle or motorcycle is its
rider, left to the driving model like any vehicle; an unconfirmed track's velocity is mostly range noise, so it never
turns a stop into a follow. A crosswalk hold needs a steadily detected track with enough hits. It holds for anyone who
might step into the turn's crosswalk, standing or moving: only a confirmed track walking away from it (on past the
maneuver point, or away down the cross street) is let go. A track's evidence is kept while the tracker may still match
it again under the same id (KEEP_EVIDENCE_S), not just while it is published.

No scene is no evidence, never a reason to hold: a missing detector is turnpilotd's to hand back (a soft disable), not
this policy's to stop for. Nor is it a reason to release: ScenePolicy keeps the stops in force when the scene went
away, dead-reckoned with the car's speed, until it comes back. The driver taking over at the hold line (a gas press with
openpilot disengaged: DisengageOnAccelerator, turnpilotd.py) releases the junction hold for that maneuver, not just
while it's held: otherwise re-engaging brings the hold back, inside the junction if the car went past the line. A gas
press with openpilot still engaged is never a go (no gas go, 2026-10-28). Past the hold line a turn is committed for good once stopping in the
junction is worse than finishing it (committed_past_line): a curb-side turn faster than COMMIT_SPEED, a far-side one
once it can't stop short of the oncoming lanes on the comfortable curve, so the oncoming hold still stops a car pulling
away from the line on its go (BeamNG: it held only because the car happened to be under 2 m/s there). Slowing
later in the junction never brings its junction hold back (pedestrians in the path still stop it). ScenePolicy keeps
those and the rest of what one frame can't know; scene_policy() is one frame on its own.
A turnpilotd fault (fault()) loses the route, and with it the maneuver a junction hold is for: the hold in force then
stays where it was, dead-reckoned like a carried pedestrian stop, until a live scene judges the junction again with the
route back, the car is committed past it, or the driver takes over at it. A junction the car was committed to or released
into stays so when the route is back under a new id, the car at it.

Objects are TurnpilotScene.TrackedObject (or anything with the same fields): x m forward, y m LEFT in the
calibrated frame, vx/vy over-ground velocity in that frame (ego motion removed), cls compared by name, inPath
set by the tracker, age its hits. If a tracker publishes velocity relative to the car instead, a parked car ahead
reads as oncoming and the junction hold is conservative, not missed. Nor is a new track's vx trusted for the oncoming
hold: until VEL_TRUST_HITS a vehicle in the oncoming lanes is taken as coming at ONCOMING_ASSUMED_SPEED, and held for on
the comfortable curve only: a parked car or a ghost of a few frames never brakes the car hard.

Hard braking (Kellen, 2026-10-28: confirmed in-path pedestrian and oncoming stops may brake as hard as they need, "but be
careful, phantom emergency braking is really dangerous"). Only these stops ever ask for more than -2.0 m/s^2, the floor
of every map, light and speed-cap stop, and only from a confirmed track: CONFIRM_HITS matched detections at CONFIRM_CONF
or more, or CONFIRM_STEADY_HITS at any (a track starts only at the tracker's NEW_TRACK_CONF), CONFIRM_RECENT_HITS of them within
CONFIRM_WINDOW_S and the track published on every frame since (a phantom seen once a second never confirms), ranged (the tracker
publishes only tracks with a ground position), in the path (in_path), or for the oncoming hold also VEL_TRUST_HITS so
its motion is the tracker's, not range noise. A track the tracker published for one detector frame (MIN_HITS) or a
few never does: its stop brakes on the comfortable curve (no accel cap, a speed cap no lower than STOP_DECEL takes).
Every hard brake is logged at its start with the track's evidence (turnpilot.hard_brake) and is raised to the driver
(SceneDecision.hard_brake: turnpilotd's controlAlert hardBrake).
"""
import math
from collections import deque
from dataclasses import dataclass, field

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.hooks import SPEED_CAP_ACCEL_MIN, SPEED_CAP_TAU, plan_accel
from openpilot.selfdrive.turnpilot.route_follower import LEFT, TURN
from openpilot.selfdrive.turnpilot.scene.tracker import CLOSE_HOLD_S, MAX_AGE_S
from openpilot.selfdrive.turnpilot.turn_speed import NO_ACCEL_CAP, NO_SPEED_CAP

VEHICLES = ("car", "truck", "bus", "motorcycle", "cyclist")
TWO_WHEELERS = ("cyclist", "motorcycle")

STOP_DECEL = 1.5            # m/s^2, the braking curve to a stop point
STOP_LAG_S = 1.0            # s the curve leads the car, the planner's speed-cap time constant
STOP_SPEED = 0.3            # m/s: slower than this...
STOP_NEAR_M = 1.5           # ...and this close to the stop point, shouldStop holds the car
HARD_STOP_MIN_M = 0.5
ACCEL_CAP_JERK = 5.0        # m/s^3: an accel cap tightens from -STOP_DECEL no faster than this (-3.5 in 0.4 s)

CONFIRM_HITS = 5            # braking harder than STOP_DECEL needs a track that has had this many hits...
CONFIRM_CONF = 0.6          # ...at this confidence (once: it stays confirmed while it's tracked),
CONFIRM_STEADY_HITS = 10    # or this many at any: a person YOLO sees at 0.45-0.59 on every run is still a person
# ...and steadily: CONFIRM_RECENT_HITS of them in the last CONFIRM_WINDOW_S (half the detector's 10 Hz runs), and confirmed
# only while the tracker keeps publishing it. Skeptic G1: a stationary in-path phantom on one detector run in 8-14 (about
# 1 Hz) built up 5-12 hits, never braked between them, then on one new frame asked for -2.6 to -8.8 m/s^2 cold
CONFIRM_RECENT_HITS = 5
CONFIRM_WINDOW_S = 1.0

PATH_HALF_WIDTH_M = 1.5     # corridor around the car's axis for objects the tracker hasn't flagged inPath
NEAR_PATH_M = 15.           # ...used for anything this close ahead
ENTER_T_S = 1.5             # a pedestrian walking into the corridor within this counts as in it,
ENTER_MAX_X_M = 40.         # this close: farther out a mono track's lateral speed is mostly range noise
PED_STOP_GAP_M = 5.0        # stop this far (camera to pedestrian) short of a pedestrian in the path
MOVING_PED_VX = 1.0         # m/s: a person moving along the path faster than this is followed, not stopped for,
FOLLOW_GAP_S = 1.0          # with this many seconds of their speed added to the gap
RIDER_OVERLAP = 0.3         # a person box this much inside a cyclist or motorcycle box (of the smaller box) is its rider,
RIDER_MIN_SPEED = 2.0       # m/s, when the two-wheeler moves faster than this: someone walking a bike stays a pedestrian

HOLD_LINE_M = 8.0           # m before the maneuver point where the car waits at a junction
HOLD_ENGAGE_MARGIN_M = 15.  # the junction hold engages within braking distance of the hold line plus this
COMMIT_SPEED = 2.0          # m/s: past the hold line and faster than this, the car is committed to its turn
TURN_ZONE_PAST_M = 5.       # m past the maneuver point the junction rules still apply
GAS_RELEASE_M = 5.          # the driver taking over releases a junction hold only this close to the hold line (or past
                            # it): further out the hold still has an oncoming car to wait for once re-engaged

CROSSWALK_X_BEFORE_M = 6.   # crosswalk zone on the turn side, along the car's axis relative to the maneuver point
CROSSWALK_X_AFTER_M = 15.
CROSSWALK_Y_NEAR_M = 1.5    # and across it, to the turn side
CROSSWALK_Y_FAR_M = 15.
# A crosswalk hold needs a track that has had this many hits at this confidence, and is steady: this many of them in the
# last this many seconds. BeamNG, (-311, 337): a wide camera false positive 10 m to the side held a right turn on 2-4
# hits at 0.42-0.63, one new hit in any second; a pedestrian there is detected on most of the detector's 10 Hz runs.
CROSSWALK_HITS = 5
CROSSWALK_CONF = 0.5
CROSSWALK_STEADY_HITS = 3
CROSSWALK_STEADY_S = 1.0
# A confirmed walker moving away from the turn's crosswalk faster than this doesn't hold: on along the car's axis past the
# maneuver point by more than CROSSWALK_LEAVING_PAST_M (the far half of the cross street's crosswalk, away from the lane
# the turn ends in), or down the cross street's sidewalk, away from the car's road. Either way mostly in that direction
# (the other component under CROSSWALK_LEAVING_RATIO of it). Standing, crossing either way, or walking toward either road
# still holds.
CROSSWALK_LEAVING_SPEED = 0.8
CROSSWALK_LEAVING_PAST_M = 3.
CROSSWALK_LEAVING_RATIO = 0.5
CROSSWALK_LEAVING_FRAMES = 10  # ScenePolicy: leaving on this many frames in a row (0.5 s), not a frame of velocity noise
KEEP_EVIDENCE_S = max(MAX_AGE_S, CLOSE_HOLD_S)  # the longest the tracker keeps a track it no longer matches
RECENT_S = max(CROSSWALK_STEADY_S, CONFIRM_WINDOW_S)  # TrackEvidence.recent keeps the hits of this long

ONCOMING_TTA_S = 5.         # hold while an oncoming vehicle reaches the junction sooner than this
ONCOMING_MIN_SPEED = 1.     # m/s toward the car
ONCOMING_Y_NEAR_M = 1.5     # oncoming lanes, to the turn side
ONCOMING_Y_FAR_M = 20.
ONCOMING_PAST_M = 5.        # a vehicle this far short of the maneuver point has cleared it
ONCOMING_ENTRY_M = 4.       # m (of the car's position) before the maneuver point it must stop by, short of the oncoming lanes
# A new track's velocity isn't trusted: BeamNG unprotected_left_visible (2026-10-27), the wide camera's new track of a car
# closing at about 11 m/s read vx +3.6, +6.1, +5.5, +1.5 (moving away) for its first 2-2.5 s, and the hold came after it
# had crossed. Until a track has this many hits (2.5 s of the detector's 10 Hz) a vehicle in the oncoming lanes is taken
# as coming at ONCOMING_ASSUMED_SPEED, whatever its vx says
VEL_TRUST_HITS = 25
ONCOMING_ASSUMED_SPEED = 15.  # m/s, 54 km/h: within 75 m of the junction an untrusted track holds


@dataclass
class SceneDecision:
  accel_cap: float = NO_ACCEL_CAP
  speed_cap: float = NO_SPEED_CAP
  should_stop: bool = False
  holding: bool = False   # the limits bite now
  reason: str = ""
  hard_brake: bool = False  # the accel cap asks for more than SPEED_CAP_ACCEL_MIN: a confirmed track's hard stop
  evidence: dict | None = None  # ...the track it is for (Stop.evidence)


@dataclass
class Stop:
  accel_cap: float
  speed_cap: float
  should_stop: bool
  reason: str
  d_stop: float = 0.      # where it was, for ScenePolicy to carry it while the scene is gone
  v_lead: float = 0.
  confirmed: bool = True
  evidence: dict | None = None  # the track it is for (track_evidence()), logged with a hard brake


def stop_command(v_ego: float, d_stop: float) -> tuple[float, float, bool]:
  """(accel cap, speed cap, should stop) to stop d_stop m ahead."""
  d = max(d_stop, 0.)
  speed_cap = math.sqrt(2. * STOP_DECEL * max(d - v_ego * STOP_LAG_S, 0.))
  a_need = -v_ego ** 2 / (2. * max(d, HARD_STOP_MIN_M))
  accel_cap = a_need if a_need < -STOP_DECEL else NO_ACCEL_CAP
  return accel_cap, speed_cap, v_ego < STOP_SPEED and d < STOP_NEAR_M


def track_evidence(obj) -> dict:
  """What a stop for this track rests on, for the log of a hard brake."""
  return {'track_id': int(obj.trackId), 'cls': str(obj.cls), 'camera': str(obj.camera), 'hits': int(obj.age),
          'conf': round(float(obj.conf), 3), 'x': round(float(obj.x), 2), 'y': round(float(obj.y), 2),
          'vx': round(float(obj.vx), 2), 'vy': round(float(obj.vy), 2), 'in_path': bool(obj.inPath)}


def stop(v_ego: float, d_stop: float, reason: str, confirmed: bool = True, v_lead: float = 0., evidence: dict | None = None) -> Stop:
  """A stop point d_stop m ahead, moving away at v_lead (then the car only has to slow to v_lead by there). Unconfirmed,
  it brakes no harder than the curve: no accel cap, and a speed cap no further below v_ego than STOP_DECEL takes."""
  accel_cap, speed_cap, should_stop = stop_command(max(v_ego - v_lead, 0.), d_stop)
  speed_cap += v_lead
  if not confirmed and accel_cap < NO_ACCEL_CAP:
    accel_cap, speed_cap = NO_ACCEL_CAP, max(speed_cap, v_ego - STOP_DECEL * SPEED_CAP_TAU)
  return Stop(accel_cap, speed_cap, should_stop and v_lead <= 0., reason, d_stop, v_lead, confirmed, evidence)


def velocity_trusted(obj) -> bool:
  """Its velocity is the tracker's, not range noise: VEL_TRUST_HITS hits."""
  return obj.age >= VEL_TRUST_HITS


def confirmed_track(obj) -> bool:
  return (obj.age >= CONFIRM_HITS and obj.conf >= CONFIRM_CONF) or obj.age >= CONFIRM_STEADY_HITS


def crosswalk_walker(obj) -> bool:
  return obj.age >= CROSSWALK_HITS and obj.conf >= CROSSWALK_CONF


def in_path(obj) -> bool:
  """The tracker's inPath follows the planned path; the straight-ahead corridor here only backs it up close in
  and for pedestrians walking into it."""
  if obj.inPath or (obj.x < NEAR_PATH_M and abs(obj.y) < PATH_HALF_WIDTH_M):
    return True
  return obj.x < ENTER_MAX_X_M and obj.y * obj.vy < 0 and abs(obj.y + obj.vy * ENTER_T_S) < PATH_HALF_WIDTH_M


def box_overlap(a, b) -> float:
  """Intersection of two boxes (x0, y0, x1, y1) over the smaller one's area."""
  w, h = min(a[2], b[2]) - max(a[0], b[0]), min(a[3], b[3]) - max(a[1], b[1])
  smaller = min((a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1]))
  return w * h / smaller if w > 0 and h > 0 and smaller > 0 else 0.


def is_rider(person, objects, is_confirmed=confirmed_track) -> bool:
  """YOLO calls a cyclist's rider a person, and the rider's box ranges badly (it ends above the ground): a person box
  inside a moving two-wheeler's box in the same camera is the two-wheeler. Moving means a confirmed track's velocity: a
  new one's is range noise, and a person beside a parked bike must not read as its rider."""
  return len(person.box) == 4 and any(
    str(o.cls) in TWO_WHEELERS and str(o.camera) == str(person.camera) and len(o.box) == 4 and math.hypot(o.vx, o.vy) > RIDER_MIN_SPEED and
    box_overlap(person.box, o.box) >= RIDER_OVERLAP and is_confirmed(o) for o in objects)


def pedestrian_stops(objects, v_ego: float, is_confirmed=confirmed_track) -> list[Stop]:
  """Pedestrians in the path: a stop short of a standing or crossing one, a following gap behind a confirmed one moving
  along it (an unconfirmed track's vx is range noise: at 30 m two positions 0.1 s apart can be several m/s off)."""
  stops = []
  for obj in objects:
    if str(obj.cls) != "pedestrian" or obj.x <= 0. or not in_path(obj) or is_rider(obj, objects, is_confirmed):
      continue
    if obj.vx > MOVING_PED_VX and is_confirmed(obj):
      stops.append(stop(v_ego, obj.x - PED_STOP_GAP_M - obj.vx * FOLLOW_GAP_S, "following pedestrian", is_confirmed(obj), obj.vx,
                        track_evidence(obj)))
    else:
      stops.append(stop(v_ego, obj.x - PED_STOP_GAP_M, "pedestrian in path", is_confirmed(obj), evidence=track_evidence(obj)))
  return stops


def committed_past_line(distance: float, v_ego: float, far_side: bool) -> bool:
  """Past the hold line and committed to the turn: stopping in the junction is worse than finishing it. A curb-side turn
  faster than COMMIT_SPEED. A far-side one once it can't stop short of the oncoming lanes (ONCOMING_ENTRY_M) on the
  comfortable curve: pulling away from the line on its go is no commitment while it still can."""
  if distance >= HOLD_LINE_M:
    return False
  if not far_side:
    return v_ego > COMMIT_SPEED
  return distance < ONCOMING_ENTRY_M or v_ego ** 2 / (2. * STOP_DECEL) > distance - ONCOMING_ENTRY_M


def hold_line_stop(maneuver, distance, v_ego, reason: str, far_side: bool = False, confirmed: bool = True,
                   evidence: dict | None = None) -> list[Stop]:
  """A stop at the hold line before a turn, while the junction rules apply. Past it and not committed, a far-side turn
  stops where the comfortable curve can (standing, where it is), which is short of the oncoming lanes: never a hard stop
  in the junction, never creeping on toward them. confirmed: as stop()'s, the evidence for it earned a hard stop."""
  if maneuver is None or maneuver.kind != TURN or distance < -TURN_ZONE_PAST_M:
    return []
  d_hold = distance - HOLD_LINE_M
  if committed_past_line(distance, v_ego, far_side):
    return []
  if d_hold < 0. and far_side:
    d_hold = v_ego ** 2 / (2. * STOP_DECEL)
  if d_hold > v_ego ** 2 / (2. * STOP_DECEL) + HOLD_ENGAGE_MARGIN_M:
    return []
  return [stop(v_ego, d_hold, reason, confirmed, evidence=evidence)]


def far_side_turn(maneuver, right_hand_traffic: bool) -> bool:
  return maneuver is not None and maneuver.kind == TURN and (maneuver.side == LEFT) == right_hand_traffic


def leaving_crosswalk(obj, distance: float, side: int) -> bool:
  """A walker moving away from the turn's crosswalk (its velocity trusted: a confirmed track's): on along the car's axis
  well past the maneuver point, or down the cross street away from the car's road."""
  along, away = obj.vx, obj.vy * side
  on_past = obj.x - distance > CROSSWALK_LEAVING_PAST_M and along > CROSSWALK_LEAVING_SPEED and \
    abs(away) < CROSSWALK_LEAVING_RATIO * along
  down_cross_street = away > CROSSWALK_LEAVING_SPEED and abs(along) < CROSSWALK_LEAVING_RATIO * away
  return on_past or down_cross_street


class RoadFrame:
  """A tracked object seen in the road's frame: the car's calibrated frame turned back by the car's heading change toward the
  turn (x along the approach, y left of it). Every other field is the object's own."""
  __slots__ = ('obj', 'x', 'y', 'vx', 'vy')

  def __init__(self, obj, a: float):
    c, s = math.cos(a), math.sin(a)
    self.obj = obj
    self.x, self.y = obj.x * c - obj.y * s, obj.x * s + obj.y * c
    self.vx, self.vy = obj.vx * c - obj.vy * s, obj.vx * s + obj.vy * c

  def __getattr__(self, name):
    return getattr(self.obj, name)


def road_frame(objects, heading_deg: float, side: int) -> list:
  """objects in the road's frame, for a car that has turned heading_deg toward side (LEFT positive) since its turn's first pulse: the
  oncoming lanes are the road's, whichever way the car stands (DESIGN phase 4 section 2: standing 30 deg into a left, an
  oncoming car 50 m up the road is 22 m to the car's right)."""
  a = math.radians(heading_deg) * side
  return list(objects) if a == 0. else [RoadFrame(obj, a) for obj in objects]


def junction_stops(objects, maneuver, distance, v_ego, right_hand_traffic, is_walker=crosswalk_walker,
                   may_leave=confirmed_track, may_harden=confirmed_track, heading_deg: float = 0.) -> list[Stop]:
  """Holds at the hold line before a turn: the crosswalk of a curb-side turn, oncoming traffic across a far-side one.
  may_leave: whether a walker's velocity is trusted to let it go when it is leaving the crosswalk; may_harden: whether a
  walker's or an oncoming vehicle's track is confirmed, to brake harder than the curve for it. heading_deg: the car's
  heading change toward the turn since its first pulse (turnpilotd.track_yaw); the oncoming band is the road's frame (road_frame)."""
  far_side = far_side_turn(maneuver, right_hand_traffic)
  if not hold_line_stop(maneuver, distance, v_ego, "", far_side):
    return []
  side = maneuver.side  # the crosswalk and the oncoming lanes are both on the turn side; y is positive left
  if not far_side:
    walkers = [obj for obj in objects if str(obj.cls) == "pedestrian" and is_walker(obj) and
               distance - CROSSWALK_X_BEFORE_M < obj.x < distance + CROSSWALK_X_AFTER_M and
               CROSSWALK_Y_NEAR_M < obj.y * side < CROSSWALK_Y_FAR_M and
               not (may_leave(obj) and leaving_crosswalk(obj, distance, side))]
    if not walkers:
      return []
    # the hold hardens only for a confirmed walker, and says which one (the most confirmed, then the nearest)
    walker = min(walkers, key=lambda obj: (not may_harden(obj), obj.x))
    return hold_line_stop(maneuver, distance, v_ego, "pedestrian in crosswalk", far_side, may_harden(walker), track_evidence(walker))
  holds = []
  for obj in road_frame(objects, heading_deg, side):
    # a new track's vx is range noise, its sign included (VEL_TRUST_HITS): taken as coming, never as going away. Taken so,
    # it holds on the comfortable curve only: a hard stop needs a track whose existence and motion are both trusted (a
    # parked car or a ghost of a few frames otherwise brakes the car at the planner's emergency floor)
    trusted = velocity_trusted(obj)
    toward = -obj.vx if trusted else max(-obj.vx, ONCOMING_ASSUMED_SPEED)
    if str(obj.cls) in VEHICLES and ONCOMING_Y_NEAR_M < obj.y * side < ONCOMING_Y_FAR_M and \
       obj.x > distance - ONCOMING_PAST_M and toward > ONCOMING_MIN_SPEED and \
       max(obj.x - distance, 0.) / toward < ONCOMING_TTA_S:
      hard = trusted and may_harden(obj)
      holds = hold_line_stop(maneuver, distance, v_ego, "waiting for oncoming", far_side, hard, track_evidence(obj))
      if hard:
        return holds
  return holds


def decide(stops: list[Stop], v_ego: float) -> SceneDecision:
  if not stops:
    return SceneDecision()
  accel_cap, speed_cap = min(s.accel_cap for s in stops), min(s.speed_cap for s in stops)
  should_stop = any(s.should_stop for s in stops)
  reason = min(stops, key=lambda s: (not s.should_stop, plan_accel(s.accel_cap, s.speed_cap, v_ego))).reason
  holding = should_stop or accel_cap < NO_ACCEL_CAP or speed_cap < v_ego + 0.5
  return SceneDecision(accel_cap, speed_cap, should_stop, holding, reason)


def scene_policy(objects, available: bool, maneuver, distance: float, v_ego: float,
                 right_hand_traffic: bool = True, heading_deg: float = 0.) -> SceneDecision:
  """One frame on its own: evidence from this frame's tracks alone, no ramp, no gas release, nothing carried through a
  gap. objects: tracked objects (ignored when not available); maneuver: RouteFollower.upcoming or None; heading_deg: the
  car's heading change toward its turn since its first pulse (junction_stops)."""
  objects = list(objects) if available else []
  stops = pedestrian_stops(objects, v_ego) + junction_stops(objects, maneuver, distance, v_ego, right_hand_traffic,
                                                            heading_deg=heading_deg)
  return decide(stops, v_ego)


@dataclass
class TrackEvidence:
  hits: int                                     # the track's age when last seen
  first_t: float = 0.                           # ScenePolicy.t when first seen, with its first hits (age then)
  first_hits: int = 0
  seen_t: float = 0.                            # ScenePolicy.t when last seen
  recent: deque = field(default_factory=deque)  # times of its hits in the last RECENT_S
  earned: bool = False                          # confirmed_track() with CONFIRM_RECENT_HITS recent hits, once
  confirmed: bool = False                       # earned a hard stop, published on every frame since (or steady again)
  walker: bool = False                          # earned a crosswalk hold
  leaving: int = 0                              # frames in a row leaving the turn's crosswalk


class ScenePolicy:
  """scene_policy() with what one frame can't know: which tracks earned a hard stop or a crosswalk hold (kept until the
  tracker can no longer match them again), the accel cap ramping in, a gas press that released a junction hold, the
  maneuver the car committed to, and the stops in force when the scene went away, carried until it's back."""

  def __init__(self):
    self.t = 0.
    self.tracks: dict[int, TrackEvidence] = {}
    self.accel_cap = NO_ACCEL_CAP
    self.released_id: int | None = None  # the maneuver whose junction hold a gas press released
    self.committed_id: int | None = None  # the maneuver the car passed the hold line for faster than COMMIT_SPEED
    self.carried: list[Stop] = []        # the last available frame's pedestrian stops, moved with the car since
    self.carried_junction: tuple[int | None, Stop] | None = None  # (maneuver id, its junction hold): reason, confirmed, evidence
    self.junction: Stop | None = None    # the junction hold in force on the last frame
    self.fault_junction: Stop | None = None  # ...kept through a fault, which lost the route it was for
    self.fault_open = False  # a fault lost the route with the car committed to (or released into) its junction
    self.hard = False  # the last frame braked harder than SPEED_CAP_ACCEL_MIN (hard_brake())

  def fault(self, maneuver_id: int | None = None) -> None:
    """turnpilotd faulted and dropped the route (maneuver_id: its maneuver): the junction hold in force stays where it
    was, and a junction the car was committed to or released into stays open when the route is back under a new id
    (module docstring)."""
    if self.junction is not None:
      self.fault_junction, self.junction = self.junction, None
    self.fault_open = self.fault_open or (maneuver_id is not None and maneuver_id in (self.released_id, self.committed_id))

  def _fault_hold(self, available: bool, maneuver, distance: float, v_ego: float, gas_pressed: bool, dt: float) -> list[Stop]:
    h = self.fault_junction
    if h is None:
      return []
    d_stop = h.d_stop - v_ego * dt
    route_back = available and maneuver is not None and distance < float('inf')
    committed = d_stop < 0. and v_ego > COMMIT_SPEED
    if route_back or committed or d_stop < -(HOLD_LINE_M + TURN_ZONE_PAST_M) or (gas_pressed and d_stop < GAS_RELEASE_M):
      self.fault_junction = None
      return []
    self.fault_junction = stop(v_ego, d_stop, h.reason, h.confirmed, evidence=h.evidence)
    return [self.fault_junction]

  def update(self, objects, available: bool, maneuver, distance: float, v_ego: float, right_hand_traffic: bool = True,
             maneuver_id: int | None = None, gas_pressed: bool = False, dt: float = DT_MDL, extra_stops=(),
             heading_deg: float = 0.) -> SceneDecision:
    """As scene_policy(), plus maneuver_id (RouteFollower.maneuver_id), once per frame dt apart. gas_pressed: the driver
    taking over (turnpilotd: carState.gasPressed with openpilot disengaged), never a go. extra_stops: the map's stop signs
    and signals (map_controls.py), composed with the scene's like any other stop. heading_deg: as scene_policy()'s."""
    far_side = far_side_turn(maneuver, right_hand_traffic)
    self.t += dt
    if available:
      objects = list(objects)
      self.update_evidence(objects)
      for obj in objects:
        ev = self.tracks[obj.trackId]
        leaving = maneuver is not None and maneuver.kind == TURN and leaving_crosswalk(obj, distance, maneuver.side)
        ev.leaving = ev.leaving + 1 if leaving else 0
      peds = pedestrian_stops(objects, v_ego, lambda obj: self.tracks[obj.trackId].confirmed)
      junction = junction_stops(objects, maneuver, distance, v_ego, right_hand_traffic, lambda obj: self.tracks[obj.trackId].walker,
                                lambda obj: self.tracks[obj.trackId].confirmed and self.tracks[obj.trackId].leaving >= CROSSWALK_LEAVING_FRAMES,
                                lambda obj: self.tracks[obj.trackId].confirmed, heading_deg)
      self.carried = peds
      self.carried_junction = (maneuver_id, junction[0]) if junction else None
    else:
      # no evidence either way: keep what was in force, the pedestrians where they were relative to the road
      self.carried = [stop(v_ego, s.d_stop - (v_ego - s.v_lead) * dt, s.reason, s.confirmed, s.v_lead, s.evidence) for s in self.carried]
      peds = self.carried
      junction = []
      if self.carried_junction is not None and self.carried_junction[0] == maneuver_id:
        # the hold keeps what its evidence earned: an unconfirmed track's hold stays on the comfortable curve
        h = self.carried_junction[1]
        junction = hold_line_stop(maneuver, distance, v_ego, h.reason, far_side, h.confirmed, h.evidence)

    if junction and gas_pressed and maneuver_id is not None and distance - HOLD_LINE_M < GAS_RELEASE_M:
      self.released_id = maneuver_id  # the driver took over and goes: stopping again further in would be worse
    # a standing car a latch holds (map_controls: position-free) is where the latch put it, short of its line: a reading that
    # puts it past (GPS while standing) never commits it (skeptic G4: the unprotected left stopped judging, held forever)
    latched = v_ego < STOP_SPEED and any(s.should_stop for s in extra_stops)
    if maneuver_id is not None and maneuver is not None and maneuver.kind == TURN and not latched and \
       committed_past_line(distance, v_ego, far_side):
      self.committed_id = maneuver_id  # never stopped in the junction for it again, however slow the car gets in there
    if self.fault_open and maneuver_id is not None and maneuver is not None and maneuver.kind == TURN:
      self.fault_open = False
      if distance < HOLD_LINE_M + GAS_RELEASE_M:  # the same junction, the car at it
        self.committed_id = maneuver_id
    if maneuver_id is not None and maneuver_id in (self.released_id, self.committed_id):
      junction = []
    self.junction = junction[0] if junction else None
    stops = peds + junction + self._fault_hold(available, maneuver, distance, v_ego, gas_pressed, dt) + list(extra_stops)
    decision = decide(stops, v_ego)
    decision.accel_cap = self.ramp(decision.accel_cap, dt)
    self.hard_brake(decision, stops, v_ego)
    return decision

  def hard_brake(self, decision: SceneDecision, stops: list[Stop], v_ego: float) -> None:
    """decision.hard_brake while its accel cap asks for more than SPEED_CAP_ACCEL_MIN, with the evidence of the stop that
    asks for the most; logged once at its start (module docstring)."""
    hard = decision.accel_cap < SPEED_CAP_ACCEL_MIN
    if hard:
      worst = min(stops, key=lambda s: s.accel_cap)
      decision.hard_brake, decision.evidence = True, worst.evidence
      if not self.hard:
        cloudlog.event("turnpilot.hard_brake", error=True, reason=worst.reason, accel_cap=round(decision.accel_cap, 2),
                       need=round(worst.accel_cap, 2), v_ego=round(v_ego, 2), d_stop=round(worst.d_stop, 2),
                       confirmed=worst.confirmed, **(worst.evidence or {}))
    self.hard = hard

  def update_evidence(self, objects) -> None:
    """This frame's tracks into their evidence. A track missing from this frame keeps its evidence for KEEP_EVIDENCE_S:
    the tracker stops publishing a track it hasn't matched for a moment, and may match it again under the same id. An id
    back with fewer hits than it had is another track (a restarted tracker), and starts afresh.
    Confirmation (a hard stop) needs confirmed_track() and CONFIRM_RECENT_HITS in the last CONFIRM_WINDOW_S, and is lost
    once the track is missing from a frame: the tracker stopped publishing it (no hit for MAX_COAST_S; a close pedestrian
    in the path stays published CLOSE_HOLD_S). A track back from that is confirmed again once it has CONFIRM_RECENT_HITS
    in the window (its confidence then needn't be CONFIRM_CONF again), which one occluded for half a second has at once
    and a 1 Hz phantom never does. For confirmation a track's hits at first sight count as recent ones then
    (the tracker publishes a track from MIN_HITS on, so a real one is first seen with two); the crosswalk's steady hits
    count only those seen since."""
    tracks = {tid: ev for tid, ev in self.tracks.items() if self.t - ev.seen_t <= KEEP_EVIDENCE_S}
    present = {obj.trackId for obj in objects}
    for tid, ev in tracks.items():
      if tid not in present:
        ev.confirmed = False
    for obj in objects:
      ev = tracks.get(obj.trackId)
      if ev is None or obj.age < ev.hits:
        ev = TrackEvidence(obj.age, self.t, obj.age)
      ev.recent.extend([self.t] * max(obj.age - ev.hits, 0))
      ev.hits, ev.seen_t = obj.age, self.t
      while ev.recent and ev.recent[0] <= self.t - RECENT_S:
        ev.recent.popleft()
      first = ev.first_hits if self.t - ev.first_t < CONFIRM_WINDOW_S else 0
      steady = first + self.recent_hits(ev, CONFIRM_WINDOW_S) >= CONFIRM_RECENT_HITS
      ev.earned = ev.earned or (confirmed_track(obj) and steady)
      ev.confirmed = ev.confirmed or (ev.earned and steady)
      ev.walker = ev.walker or (crosswalk_walker(obj) and self.recent_hits(ev, CROSSWALK_STEADY_S) >= CROSSWALK_STEADY_HITS)
      tracks[obj.trackId] = ev
    self.tracks = tracks

  def recent_hits(self, ev: TrackEvidence, window_s: float) -> int:
    return sum(t > self.t - window_s for t in ev.recent)

  def ramp(self, target: float, dt: float) -> float:
    """The accel cap toward target: at once when it eases, from -STOP_DECEL at ACCEL_CAP_JERK when it tightens."""
    self.accel_cap = target if target >= self.accel_cap else max(target, min(self.accel_cap, -STOP_DECEL) - ACCEL_CAP_JERK * dt)
    return self.accel_cap
