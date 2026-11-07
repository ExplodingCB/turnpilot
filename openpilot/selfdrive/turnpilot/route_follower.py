"""Route following: the next maneuver and the distance to it, between navd's 1 Hz updates.

navd publishes navInstruction once a second: maneuverDistance is the distance to the end of the current step,
and it runs down to about -10 m before navd advances to the next step. Between updates the distance is
dead-reckoned from carState.vEgo, which at highway speed is up to 30 m better than the last message.
Maneuver type and modifier are Mapbox/OSRM strings, copied through by navd unchanged.
What counts as a turn: only a junction maneuver, where the route leaves the road at an intersection that other roads
meet (navInstruction.maneuverBranches > 0, from Valhalla's intersection bearings). Where the road just bends and no
other road meets it, the maneuver is NONE: the driving model follows the bend (0.11.3's Autospeed slows for curves), and
TurnPilot sends no desire and no speed cap there. navd moves past such a step only 10 m beyond its end, so the turn or
keep after it (RouteFollower.ahead, from allManeuvers with its junction) is looked ahead to for its speed cap: a junction
turn 30-50 m past a bend was otherwise seen at road speed, too late to slow for, and skipped. Its desire waits until the
car is past the step: a turn desire before a straight-on junction would turn the car there. That is when the step is
dead-reckoned NAVD_ADVANCE_M past, where navd moves on, not up to a second later at navd's next update: the turn after
it becomes the maneuver then, under its own id, which navd's update moving on to it keeps (an update still on the step
only brings the turn's distance). Even so a turn under turn_skip.MIN_FIRST_PULSE_M past the step is skipped out loud.
The angle is the real one, bearing_after less bearing_before. A route
without intersection data (maneuverGeometryValid false; Valhalla always has it) leaves every turn a junction at the
modifier's nominal angle, which is logged once per maneuver and shown in the plan's reason.
A new route (a destination set from the phone, a reroute) can start with a maneuver of the same type and modifier as
the old one, so it is told apart by navd marking it (one invalid navInstruction between the two routes, and a new
routeCount) or by the distance jumping up by more than GPS noise or dead reckoning explain. Either way the old maneuver
was dropped, not passed, and RouteFollower.replaced says so for the one update: its turn pulse must not carry over to the
new route.
Where a junction is: no navd reading of it is trusted over another, however many agree. GPS bias lasts (urban multipath
over a long approach, a fix off while the car stands in a queue, a reroute's snap), and every rule that let readings
outvote each other let a biased track win somewhere and lifted a committed turn's cap short of its junction. So every
reading of the maneuver's junction is dead-reckoned on (JunctionSpan), and what is done with them holds whichever one
is right:
- RouteFollower.distance, and with it the turn's cap, its desire's lead and an unprotected turn's hold line, is the
  NEAREST of them from JunctionSpan.FROM_M in: slow early, never late. A jump up is never taken up for it, so it only ever runs down. Where the
  furthest is more than UNCERTAIN_M beyond it while a committed unprotected turn's hold is ahead, turnpilotd says so out
  loud (controlAlert junctionUncertain) rather than guess, and a hold line read behind where the car can stop at the
  floor stops the car as soon as it can (map_controls.py);
- the maneuver counts as passed (its cap lifted, its id bumped, CLEAR sent) on navd's distance only once the car is
  dead-reckoned PASSED_DIST_M past the FURTHEST of them, never on a vote: of every one since the approach began, however
  far out, since a bias already there as the car comes in FROM_M can last past the commit, its true readings from before
  it the only ones that hold the truth. A reading dead-reckoned that far can only hold the pass later than the rest would
  (and widen the span, saying the junction is uncertain sooner), never earlier. The car having turned it is the other evidence
  (DesireTiming clears it at its angle);
- a curb-side turn's one pulse goes by the furthest (never early), and at the latest by navd's LATEST reading, dead-reckoned
  (JunctionSpan.conf, RouteFollower.junction_latest; desire_timing.pulse_distance): the furthest holds readings from 43-114 m
  out, 8-26 s old, and on 1e's 91 deg right at 8:32 (replayed through one follower for the whole drive) it was still
  8.8 m out with the nearest at the junction, where the latest agreed with the nearest. The latest counts only CONFIRMED:
  a reading within LATEST_AGREE_M of the one before it, dead-reckoned (2026-11-06). Taken alone, one outlier 9 m short about
  20 m out brought the pulse out at 14.8 m (13 m short: 17.1 m), before navd's next true reading, the early turn-in that
  clipped 1e's rights; one 15 m short there had it skipped tooLate 17 m out. On 1d and 1e every latest that brought a right's
  pulse out agreed with the reading before it: 1d 5:02 and 1e 7:15 within 0.4 m, 1e 8:30's jump in from 12.2 m within 4.0 m
  (its next reading 0.1 m off it); no reading inside 45 m of a right jumped in by more than 4.4 m. A reading that doesn't agree
  leaves the last confirmed one, dead-reckoned, so one outlier either way never moves it, and two that agree (a bias that
  lasts) are taken as before;
- a curb-side turn's first pulse is never let go past the junction by the NEAREST reading, one reading alone included, and a
  turn whose nearest crosses its junction unpulsed is skipped out loud there (desire_timing, turn_skip). So one reading 8-10 m
  short 20-45 m out, which the nearest keeps to the end, has the nearest cross the junction before the furthest reaches the
  pulse point, and that right is skipped tooLate 7.7-9.8 m short of its junction with every later reading saying it is ahead
  (HEAD, pulsing at 40 m, took it), its corner cap kept to its pass (turnpilotd keep_hand_cap: 3.0 m/s through it). Kept on
  purpose (2026-11-06 review): the same readings are one TRUE reading inside a long bias of 8-10 m that lasts, and nothing in
  them tells the two apart. Judged by the second-nearest (tried that round, never one reading alone), that right was pulsed
  0.3 m past its junction under an 8 m bias and 2.3 m past under 10 m, nothing said, the late pulse being the dangerous one
  (desire_timing); by the nearest it is skipped tooLate at its junction, out loud, the car held to its cap. A long bias that
  lasts with no true reading in it pulses that late by any reading (the furthest and the latest are as long), and so does one of
  12 m or more with a true reading in it, the next long reading's jump up from inside ADVANCE_NEAR_M taken for navd moving on
  (below): navd's readings alone can't say so. The cap, the hold lines and the pass keep the nearest and the furthest as above.
A committed turn (turnpilotd: its turn desire is out) is never dropped that way (Kellen: never cancel a turn once
committed, and a car slowing for a turn must not speed up into it). The junction is where it was whatever navd says:
- an invalid navInstruction for the same route (navd off the step for a fix, rerouting, the destination removed) or navd
  going quiet past NAV_STALE_S is a gap: the maneuver stays, dead-reckoned, under the same id, until navd's next valid
  update carries on with it or the turn is over (cleared at its angle, or dead-reckoned PASSED_DIST_M past the furthest
  of its readings);
- navd on another step (not same_junction) is a gap too, however close in: navd moving on early to the next step (a
  sharp turn's, up to 10 m before its end, or a GPS jump forward) doesn't mean the car has made the turn. It is over
  only on real evidence: the car has turned it (DesireTiming clears it at its angle) or is dead-reckoned PASSED_DIST_M
  past the furthest of its readings. Until then its id and its cap stay and no CLEAR goes; navd's next update after that takes up the next step;
- a jump in the distance to the same junction, up or down, is GPS noise: one more reading of it (above), however close
  in, and a reroute's readings too. One more than SAME_TURN_M outside its readings so far is navd on another junction (a
  GPS jump, navd moving on early to the next turn the same way on a route without bearings): a gap, as above;
- a new route (a new routeCount) whose first maneuver is the same junction (same_junction) within SAME_TURN_M of the
  dead-reckoned one is the same maneuver (a reroute onto the road the car is on, its origin snapped up to a reroute's
  25 m off), its readings one more of the junction's. A new route that doesn't turn there (a reroute whose origin snapped past
  the junction, a destination changed from the phone) is a gap: the turn is held to its end, and the new route taken up
  after.
After a reroute or a jump navd's distance to the junction can stay off by that much: the turn's cap would lift, or the
turn read as passed, before the car gets there. So from then on that junction is held, while the turn is committed and
after it while navd is still on its step (by its bearings), and never taken up again as a new maneuver.
A curb-side turn is committed only at its one pulse 8 m out (desire_timing, 2026-11-06), where at HEAD its 40 m pulse had
it held from there, so from turn_speed.TURN_CAP_LEAD_M in (its cap's lead) a turn turnpilotd has in hand, acting on it, not
skipped or dropped, its desire not done (update's junction_hold: turnpilotd.turn_in_hand), is held as a committed one is
through an invalid run, navd going quiet past NAV_STALE_S, navd on another step of the same route (moved on early: a route
without bearings moving on 10-14 m short of the first of two rights 50 m apart dropped it 2.9 m short, unpulsed, with nothing
said), a jump past the span's plausible readings, and a jump up from close in beyond every reading of it. But not as a
committed one in two ways, both of which committed=True broke for a turn whose pulse hasn't gone: a new route that doesn't
turn there is taken (its turn dropped, replaced, its cap with it: a gap held to the pass would cap and pulse a right 35 m out
on a dead route), one that does carries its readings over as for any turn (a new id); and its span is never held (_held_id),
so a jump up from inside ADVANCE_NEAR_M that its own readings bear out (within NEW_MANEUVER_JUMP_M of the furthest: the
nearest one short outlier) still re-baselines it as a new maneuver at the same junction, its readings fresh, which the rights
fix relies on for outliers 12-20 m short. Without bearings, two turns the same way closer than SAME_TURN_M read as one
junction: navd moving on early to the second from further out than ADVANCE_NEAR_M is then one more reading of the first,
which no reading can tell from a GPS jump (turnpilotd says a turn in hand the follower drops for another maneuver).
A turn the car has made or is past is never a new maneuver (2026-11-06, drive 1e):
- an invalid navInstruction for the same route (the same routeCount) with the car at or past the maneuver's junction is a gap
  too, committed or not: at 8:36 the right was done (its desire cleared 4 s before, so no longer committed) when navd sent
  one invalid message (off its step for a fix, the car on the exit road), and the valid one after it, the same step at
  -12.1 m, became a new maneuver behind the car, skipped tooLate and announced 'Too Fast to Turn / Rerouting'. Dead-reckoned
  through the gap under the same id, the valid one after it is one more reading of the junction and the turn is passed at
  PASSED_DIST_M; NAV_STALE_S still ends a real loss, and a new route (its separator carries the new routeCount) still resets.
  Only a TURN's or a KEEP's, the maneuvers the car makes: past a bend step (NONE) or at the arrival the invalid message is a
  drop as before, since dead-reckoned on through a route cancelled just past a bend the follower would move on to the turn
  after it (_move_on) and cap and pulse it for a route that no longer exists;
- a new maneuver whose first reading is already behind the car (a reroute that arrived stale: at 9:43 a fresh route's first
  step was 4.8 m behind) is passed from the start (upcoming None: no pulse, no skip, no skip mark for navd), logged
  (turnpilot.maneuver_behind); navd moves on from it as the car drives. Behind by its own reading and by the furthest of its
  span: a reroute's same junction carries the old span over, and one short outlier the nearest kept below zero must not pass
  that turn, uncommitted (a right is until its 8 m pulse), with no pulse, skip or word at all.
A slight turn across oncoming traffic is a turn, not a keep (make_maneuver; 1e seg 5 and 1d seg 9: slight lefts of 29 and
38 deg with an oncoming approach were keeps, refused at night for no lane line (keepNoLine), driven straight, 'Fork Missed'
and rerouted into a left; a keepLeft there would have crossed the oncoming lanes with no hold and no gap check): a
"slight" turn (not a fork or ramp) at a junction whose bearings turn SLIGHT_TURN_MIN_DEG or more to the far side, with an
opposing approach present, is a TURN, so an autonomous unprotected left (unprotected_left.py: driven to its hold line, then
a gap the camera sees or a loud handback). True forks under that angle, slight turns to the curb side, ramps, and slight
turns with no opposing approach (or one not known) stay keeps.
navd's "present" is any other road within 45 deg of straight on (routing/guidance.py: OPPOSING_DEG), one 26-45 deg off the
far side included, and that road's bearing decides nothing (2026-11-07). The 11-06 rights review had a cone on it (a road
further than 25 deg off the other side taken for a side road past a main road bending left, the slight turn left a keep, so
as not to hold the car at a line on a through road); with navd sending the bearing (maneuverOpposingRelBearing, + right of
straight on, NaN unknown; Maneuver.opposing_rel_deg, None unknown), on the local tiles around the 11-06 drives (2500 routes:
140 distinct slight lefts, 66 TURNs on present alone) the cone made keeps of 6, and none was that bend: in 5 the far-side
road was the approach's own road going on (4 carrying its name, a 5th its class and speed, each at the approach's own mapped
speed), whose oncoming lanes the left crosses, as a keep with no hold line and no gap check; the 6th service roads with no
speed mapped. A route that stays on a bending main road comes as 'continue' (11 of the 140), which classify leaves to the
model (NONE) before any of this, and at an end of road there is no road of the car's own to keep along. So a slight far-side
turn across a present approach is a TURN whatever that road's bearing: a hold too many, never one too few (routing/guidance.py's
rule for opposing). The bearing stays on navd's message (so in the logs) and on the maneuver, for a minor-road rule (that
road's mapped speed, maneuverOpposingSpeed, under the approach's) should a keep there ever be shown safe.
"""
import math
from dataclasses import dataclass, replace

from openpilot.common.swaglog import cloudlog

LEFT, RIGHT = 1, -1

# kinds of maneuver, by what TurnPilot does about them
TURN = "turn"      # a junction turn: turn desire, turn speed cap, lane choice
KEEP = "keep"      # slight turns, forks, ramps and exits: keep desire
UTURN = "uturn"    # the driving model can't U-turn; nothing is sent
ARRIVE = "arrive"
NONE = "none"

TURN_MODIFIERS = {"left": LEFT, "sharp left": LEFT, "right": RIGHT, "sharp right": RIGHT}
SLIGHT_MODIFIERS = {"slight left": LEFT, "slight right": RIGHT}
KEEP_TYPES = ("fork", "off ramp")  # a keep whatever the modifier says: their branches part gently
# with a slight modifier, nothing: the road itself bends that way. A keep desire there swerves the car, at highway speed
CONTINUE_TYPES = ("continue", "new name")
RAMP_TYPES = ("fork", "off ramp", "on ramp", "merge")
NO_ACTION_TYPES = ("depart", "notification", "merge", "roundabout", "rotary", "roundabout turn",
                   "exit roundabout", "exit rotary")
NOMINAL_ANGLE_DEG = {"slight left": 45., "slight right": 45., "left": 90., "right": 90.,
                     "sharp left": 135., "sharp right": 135., "uturn": 180.}

SIDE_MIN_DEG = 20.         # a heading change under this has no side to check the modifier against
SLIGHT_TURN_MIN_DEG = 25.  # a slight turn to the far side this sharp or more, across an opposing approach, is a TURN (module docstring)

NEW_MANEUVER_JUMP_M = 10.  # navd's distance jumping up by more than this...
ADVANCE_NEAR_M = 5.        # ...from this close means it moved on to the next maneuver (it advances past -10 m),
                           # while a jump from further out is GPS noise
ROUTE_JUMP_M = 50.         # a jump up by more than this is a new maneuver from anywhere...
REPLACED_FROM_M = 15.      # ...and from further out than this, where navd never moves on (it does on a sharp turn's
                           # next step up to 10 m early), a new route
PASSED_NEAR_M = 30.        # a new maneuver means the last one was passed if it was this close
PASSED_DIST_M = 15.        # dead-reckoned this far past a maneuver navd hasn't advanced from, it counts as passed
NAVD_ADVANCE_M = 10.       # navd moves past a step this far beyond its end (navd.py MANEUVER_TRANSITION_THRESHOLD)
NAV_STALE_S = 3.           # no navInstruction for this long, the route is lost
SAME_TURN_M = 40.          # a committed turn is the first maneuver of a new route this close to where it was dead-reckoned
                           # (navd reroutes 25 m off the route; the next junction the same way is a block on)
SAME_BEARING_DEG = 15.     # the same junction's bearings, from two routes or two fixes, are this close
ARRIVE_DIST_M = 15.
UNCERTAIN_M = 15.          # a held junction's readings further apart than this: where it is, is said to be uncertain
LATEST_AGREE_M = 5.        # a reading of the junction within this of the one before it (dead-reckoned) confirms it as the latest
                           # (JunctionSpan.conf; module docstring: 1e 8:30's true latest jumped in 4.0 m, 1d/1e's others 1-4.4)


@dataclass(frozen=True)
class Maneuver:
  kind: str
  side: int            # LEFT, RIGHT, or 0
  type: str
  modifier: str
  angle_deg: float     # unsigned turn angle: from the junction's bearings, else the modifier's nominal one
  lanes: tuple = ()    # ((directions, active), ...) left to right, from NavInstruction.lanes; empty when navd has none
  geometry: bool = False  # angle_deg and the junction test came from the route's bearings
  bend: bool = False      # the road bends here and no other road meets it: left to the model (kind NONE)
  bearing_side: int = 0   # LEFT or RIGHT as the bearings turn, 0 without them or under SIDE_MIN_DEG
  bearings: tuple = ()    # (bearing_before, bearing_after) with geometry
  opposing: str = 'unknown'  # NavInstruction.Opposing: a road straight ahead whose traffic a far-side turn crosses
  opposing_speed: float = 0.  # m/s, that road's mapped limit (maneuverOpposingSpeed), 0 unknown
  opposing_lanes: int = 0     # its lanes toward the junction, 0 unknown
  opposing_straight_m: float = 0.  # m out it stays straight on from our approach (maneuverOpposingStraightM), 0 unknown
  wrong_way_m: float = 0.  # m before the junction of an opening on its side the car may not enter (maneuverWrongWayM), 0 none
  opposing_rel_deg: float | None = None  # deg, that road's leg off straight on, + right (maneuverOpposingRelBearing), None unknown

  @property
  def ramp(self) -> bool:
    return self.type in RAMP_TYPES


def classify(mtype: str, modifier: str) -> tuple[str, int]:
  """(kind, side) of an OSRM maneuver type and modifier."""
  if mtype == "arrive":
    return ARRIVE, TURN_MODIFIERS.get(modifier, 0)
  if mtype in NO_ACTION_TYPES or modifier in ("", "straight"):
    return NONE, 0
  if modifier == "uturn":
    return UTURN, 0
  if mtype in CONTINUE_TYPES and modifier in SLIGHT_MODIFIERS:
    return NONE, 0
  side = TURN_MODIFIERS.get(modifier) or SLIGHT_MODIFIERS.get(modifier, 0)
  if side == 0:
    return NONE, 0
  if mtype in KEEP_TYPES or modifier in SLIGHT_MODIFIERS:
    return KEEP, side
  return TURN, side


def lanes_from_instruction(instruction) -> tuple:
  return lanes_from_list(instruction.lanes)


def lanes_from_list(lanes) -> tuple:
  return tuple((tuple(str(d) for d in lane.directions), bool(lane.active)) for lane in lanes)


def turn_angle(bearing_before: float, bearing_after: float) -> float:
  """Signed heading change in degrees, (-180, 180]: positive to the right (bearings run clockwise)."""
  return -((bearing_before - bearing_after + 180.) % 360. - 180.)


def rel_bearing(value) -> float | None:
  """The opposing road's bearing off straight on (navd's maneuverOpposingRelBearing / opposingRelBearing), None when it is
  not known: NaN, or a navd before the field (getattr's default)."""
  value = float(value)
  return value if math.isfinite(value) else None


def maneuver_from_instruction(instruction, right_hand_traffic: bool = True) -> Maneuver:
  return replace(make_maneuver(instruction.maneuverType, instruction.maneuverModifier, lanes_from_instruction(instruction),
                               instruction.maneuverGeometryValid, instruction.maneuverBearingBefore, instruction.maneuverBearingAfter,
                               instruction.maneuverBranches, str(instruction.maneuverOpposing), right_hand_traffic,
                               rel_bearing(getattr(instruction, 'maneuverOpposingRelBearing', math.nan))),
                 opposing_speed=float(instruction.maneuverOpposingSpeed), opposing_lanes=int(instruction.maneuverOpposingLanes),
                 opposing_straight_m=float(instruction.maneuverOpposingStraightM), wrong_way_m=float(instruction.maneuverWrongWayM))


def maneuver_from_ahead(ahead, right_hand_traffic: bool = True) -> Maneuver:
  """A Maneuver from a NavInstruction.Maneuver (allManeuvers), which has no lanes."""
  return replace(make_maneuver(ahead.type, ahead.modifier, (), ahead.geometryValid, ahead.bearingBefore, ahead.bearingAfter, ahead.branches,
                               str(ahead.opposing), right_hand_traffic, rel_bearing(getattr(ahead, 'opposingRelBearing', math.nan))),
                 opposing_speed=float(ahead.opposingSpeed), opposing_lanes=int(ahead.opposingLanes),
                 opposing_straight_m=float(ahead.opposingStraightM), wrong_way_m=float(ahead.wrongWayM))


def slight_far_turn(kind: str, side: int, mtype: str, bearing_side: int, angle_deg: float, branches: int, opposing: str,
                    right_hand_traffic: bool = True) -> bool:
  """A "slight" keep that is a turn across oncoming traffic (module docstring): a slight turn or end of road (not a fork, ramp
  or merge) at a junction (branches) whose bearings turn SLIGHT_TURN_MIN_DEG or more its way, to the far side
  (map_controls.unprotected_turn's side rule: a left where traffic keeps right), with an opposing approach present, whatever
  that road's bearing off straight on (2026-11-07: the cone on it made keeps across the approach's own road going on)."""
  return kind == KEEP and mtype not in KEEP_TYPES and mtype not in RAMP_TYPES and branches > 0 and opposing == 'present' and \
    (side == LEFT) == right_hand_traffic and bearing_side == side and angle_deg >= SLIGHT_TURN_MIN_DEG


def make_maneuver(mtype: str, modifier: str, lanes: tuple, geometry_valid: bool, bearing_before: float, bearing_after: float,
                  branches: int, opposing: str = 'unknown', right_hand_traffic: bool = True,
                  opposing_rel_deg: float | None = None) -> Maneuver:
  kind, side = classify(mtype, modifier)
  if not geometry_valid:
    return Maneuver(kind, side, mtype, modifier, NOMINAL_ANGLE_DEG.get(modifier, 0.), lanes)  # opposing unknown
  angle = turn_angle(bearing_before, bearing_after)
  bearing_side = 0 if abs(angle) < SIDE_MIN_DEG else RIGHT if angle > 0 else LEFT
  bearings = (float(bearing_before), float(bearing_after))
  if kind in (TURN, KEEP) and branches == 0:
    return Maneuver(NONE, 0, mtype, modifier, abs(angle), lanes, geometry=True, bend=True, bearing_side=bearing_side, bearings=bearings,
                    opposing=opposing, opposing_rel_deg=opposing_rel_deg)
  if slight_far_turn(kind, side, mtype, bearing_side, abs(angle), branches, opposing, right_hand_traffic):
    kind = TURN  # across the oncoming lanes: an unprotected turn, held at its line, never a keep (module docstring)
  return Maneuver(kind, side, mtype, modifier, abs(angle), lanes, geometry=True, bearing_side=bearing_side, bearings=bearings,
                  opposing=opposing, opposing_rel_deg=opposing_rel_deg)


def same_bearings(a: Maneuver, b: Maneuver) -> bool:
  """Both have junction geometry, and it is the same junction's: bearings before and after within SAME_BEARING_DEG.
  Two junctions in a row the same way can't share them: the heading changes at the first."""
  return a.geometry and b.geometry and len(a.bearings) == len(b.bearings) == 2 and \
    all(abs(turn_angle(x, y)) <= SAME_BEARING_DEG for x, y in zip(a.bearings, b.bearings, strict=True))


def same_junction(a: Maneuver, b: Maneuver) -> bool:
  """The same maneuver at the same junction, as far as the route says: type and modifier, and the bearings when both
  have them."""
  return (a.type, a.modifier) == (b.type, b.modifier) and (same_bearings(a, b) or not (a.geometry and b.geometry))


class JunctionSpan:
  """The nearest and the furthest navd's readings of one junction put it, each dead-reckoned on (module docstring), the
  latest (last: what navd says now) and the latest CONFIRMED (conf: the deadline a curb-side turn's pulse goes by at the
  latest, desire_timing.pulse_distance; module docstring, 2026-11-06).
  The furthest is over every reading since the span began, however far out: a bias already there as the car comes in
  FROM_M can last past the commit, and the true readings from before it are the only ones that hold the truth.
  conf is a reading that agreed with the one before it (dead-reckoned) within LATEST_AGREE_M; a reading that didn't leaves the
  last confirmed one, dead-reckoned, until the next reading agrees with it. The span's first reading is its own.
  The nearest is any one reading, never only one a second corroborates: a curb-side turn is past its junction by it (module
  docstring: by the second-nearest, one true reading inside a lasting long bias had the right pulsed past its junction)."""
  FROM_M = 200.  # for the nearest, readings from further out are dropped once a nearer one comes: dead reckoning that
                 # far drifts meters, and the nearest only ever runs down (a drifted one would hold the car short)

  def __init__(self, distance: float | None = None):
    self.near = self.far = self.last = self.conf = float('nan')
    if distance is not None:
      self.take(distance)

  def __bool__(self) -> bool:
    return self.near == self.near  # not nan

  @property
  def spread(self) -> float:
    return self.far - self.near if self else 0.

  def travel(self, travelled: float):
    self.near -= travelled
    self.far -= travelled
    self.last -= travelled
    self.conf -= travelled

  def plausible(self, distance: float) -> bool:
    """A reading of this junction, not of another one: within SAME_TURN_M of the span."""
    return not self or self.near - SAME_TURN_M <= distance <= self.far + SAME_TURN_M

  def take(self, distance: float):
    # the latest confirmed: this reading once it agrees with the one before it (class docstring); a span's first is its own
    if not self or abs(distance - self.last) <= LATEST_AGREE_M:
      self.conf = distance
    self.last = distance
    if not self:
      self.near = self.far = distance
      return
    # the nearest: still further out than FROM_M, the latest reading
    self.near = distance if self.near > self.FROM_M else min(self.near, distance)
    self.far = max(self.far, distance)


class RouteFollower:
  def __init__(self, maneuver_id: int = 0):
    """maneuver_id: where the ids start. A follower that replaces another one (turnpilotd after a fault) starts past the
    old one's, so no id repeats: what was latched for an old maneuver (ScenePolicy's commitment) can't match a new one."""
    self.maneuver_id = maneuver_id
    self.route_count = 0  # navInstruction.routeCount of the last valid update
    self.right_hand_traffic = True  # the side traffic keeps to (update()): a slight turn to the far side may be a TURN
    self.reset()

  def reset(self):
    self.valid = False
    self.maneuver: Maneuver | None = None
    self.distance = float('inf')            # m to the next maneuver, dead-reckoned
    self.distance_remaining = float('inf')  # m to the destination
    self.passed = False                     # true for the one update in which the last maneuver was passed
    self.replaced = False                   # true for the one update in which a new route dropped the last maneuver
    self.through_lanes: tuple = ()          # NavInstruction.throughLanes: the next intersection the route goes straight through
    self.through_distance = float('inf')    # m to it, dead-reckoned
    self.ahead: Maneuver | None = None      # past a NONE maneuver, the next turn or keep (allManeuvers), for its speed cap
    self.ahead_distance = float('inf')      # m to it, dead-reckoned
    self._passed_id = -1
    self._stale_t = 0.
    self._moved_on_from: tuple[str, str] | None = None  # (type, modifier) of the NONE step left for the turn after it
    self._held_id = -1  # a committed maneuver: held to its readings' span to its end, not on navd's distance
    self._span = JunctionSpan()        # navd's readings of the maneuver's junction
    self._ahead_span = JunctionSpan()  # ...and of the turn or keep ahead's, past a NONE maneuver

  @property
  def upcoming(self) -> Maneuver | None:
    """The next maneuver, or None once it is passed and navd hasn't moved on yet."""
    return self.maneuver if self.valid and self._passed_id != self.maneuver_id else None

  @property
  def junction_spread(self) -> float:
    """m between the furthest and the nearest navd's readings put the maneuver's junction (module docstring)."""
    return self._span.spread if self.valid else 0.

  @property
  def junction_latest(self) -> float:
    """m to the maneuver's junction by navd's latest CONFIRMED reading of it, dead-reckoned (JunctionSpan.conf; distance
    without one): between the nearest and the furthest, the fresh one a curb-side turn's pulse is due by at the latest
    (desire_timing: pulse_distance, CURB_LATEST_PULSE_M), when the furthest holds a reading from far out long past the
    junction. Confirmed, never one reading alone (module docstring: one outlier 9 m short 20 m out brought the pulse out at
    14.8 m)."""
    return self._span.conf if self.valid and self._span else self.distance

  @property
  def arrived(self) -> bool:
    return self.valid and self.maneuver is not None and self.maneuver.kind == ARRIVE and self.distance < ARRIVE_DIST_M

  def update(self, instruction, updated: bool, valid: bool, v_ego: float, dt: float, committed: bool = False,
             right_hand_traffic: bool | None = None, junction_hold: bool = False):
    """instruction: the latest navInstruction; updated: it arrived since the last call; valid: its valid flag;
    committed: the current maneuver's turn desire is out, so it is held through a gap in the route (module docstring);
    right_hand_traffic: the side traffic keeps to (the last one given when None), for make_maneuver's slight far-side turns;
    junction_hold: the current maneuver is a turn in hand, its desire not out yet (turnpilotd.turn_in_hand: a curb-side TURN
    TurnPilot acts on inside turn_speed.TURN_CAP_LEAD_M): held through a gap as a committed one is, but never against a new
    route that doesn't turn there, and with a short outlier's re-baselining kept (module docstring)."""
    self.passed = self.replaced = False
    if right_hand_traffic is not None:
      self.right_hand_traffic = bool(right_hand_traffic)
    hold = committed or junction_hold
    if updated and not valid:
      # a gap, not a drop: the turn committed or in hand, or the car at or past the junction of this route's maneuver (a turn
      # it has made, navd off its step for a fix: 1e 8:36, module docstring). A new route's separator carries the new
      # routeCount; a turn in hand is held through it too, the new route's first valid update deciding (_on_instruction)
      if self.valid and (hold or (int(instruction.routeCount) == self.route_count and self.distance <= 0. and
                                  self.maneuver is not None and self.maneuver.kind in (TURN, KEEP))):
        updated = False  # dead-reckoned on as between updates; NAV_STALE_S still ends a real loss
      else:
        dropped = self.valid
        self.reset()
        if dropped:  # navd dropped the route: a new one comes next, or the car is off it
          self.maneuver_id += 1
          self.replaced = True
        return
    if updated:
      self._stale_t = 0.
      self._on_instruction(instruction, committed, max(v_ego, 0.) * dt, junction_hold and not committed)
    elif self.valid:
      self._stale_t += dt
      if self._stale_t > NAV_STALE_S and not hold:  # navd went quiet: the maneuver is over, as when navd drops the
        self.reset()  # route, but not replaced (a turn under way finishes and its desire is cleared as for a pass)
        self.maneuver_id += 1
        return
      self._travel(max(v_ego, 0.) * dt)
    if committed and self.valid and self.maneuver is not None:
      self._hold()
      self._at_nearest()
    self._check_passed()

  def _hold(self):
    """The committed maneuver is held from here on, on the route it was committed on."""
    if self._held_id != self.maneuver_id:
      self._held_id = self.maneuver_id
      if not self._span:
        self._span = JunctionSpan(self.distance)

  def _at_nearest(self):
    """A held maneuver's junction, for its cap, desire and hold line: the nearest its readings put it (module docstring)."""
    if self._held_id == self.maneuver_id and self._span:
      self.distance = self._span.near

  def dead_reckon(self, v_ego: float, dt: float):
    """A frame turnpilotd faulted on with a turn committed (turnpilotd.py): the distances run down with the car, as between
    navd's updates, so the turn is over where it would have been, not held on past its junction."""
    self.passed = self.replaced = False
    if self.valid:
      self._travel(max(v_ego, 0.) * dt)
      self._check_passed()

  def _travel(self, travelled: float):
    self.distance -= travelled
    self._span.travel(travelled)
    self._ahead_span.travel(travelled)
    self.distance_remaining -= travelled
    self.through_distance -= travelled
    self.ahead_distance -= travelled

  def _check_passed(self):
    if self.valid and self.maneuver is not None and self.maneuver.kind == NONE and self.ahead is not None and \
       self.distance < -NAVD_ADVANCE_M:
      self._move_on()
    # passed only past the furthest any reading put it (module docstring)
    distance = max(self.distance, self._span.far) if self._span else self.distance
    if self.valid and distance < -PASSED_DIST_M and self._passed_id != self.maneuver_id:
      self._passed_id = self.maneuver_id
      self.passed = True

  @staticmethod
  def _log_new(maneuver: Maneuver, distance: float):
    """A log line for each maneuver TurnPilot would act on but has no junction data for, and for a turn's bearings
    that disagree with its modifier (the side comes from the modifier)."""
    if maneuver.kind not in (TURN, KEEP):
      return
    if not maneuver.geometry:
      cloudlog.warning(f"turnpilot: no junction data for {maneuver.type} {maneuver.modifier} {distance:.0f} m ahead, " +
                       f"taken as a junction at the nominal {maneuver.angle_deg:.0f} deg")
    elif maneuver.bearing_side and maneuver.bearing_side != maneuver.side:
      cloudlog.warning(f"turnpilot: {maneuver.type} {maneuver.modifier} {distance:.0f} m ahead, but its bearings turn " +
                       f"{maneuver.angle_deg:.0f} deg the other way")

  def _move_on(self):
    """Past a NONE step where navd moves on from it: the turn or keep after it is the maneuver now (module docstring)."""
    self._moved_on_from = (self.maneuver.type, self.maneuver.modifier)
    self.maneuver, self.distance = self.ahead, self.ahead_distance
    self._span = self._ahead_span if self._ahead_span else JunctionSpan(self.distance)
    self.ahead, self.ahead_distance, self._ahead_span = None, float('inf'), JunctionSpan()
    self.maneuver_id += 1

  def _following(self, instruction) -> tuple[Maneuver | None, float]:
    """Past a NONE step, the next turn or keep in allManeuvers and its distance."""
    for ahead in list(instruction.allManeuvers)[1:]:
      following = maneuver_from_ahead(ahead, self.right_hand_traffic)
      if following.kind != NONE:
        return (following, float(ahead.distance)) if following.kind in (TURN, KEEP) else (None, float('inf'))
    return None, float('inf')

  def _on_instruction(self, instruction, committed: bool, travelled: float, junction_hold: bool = False):
    distance = float(instruction.maneuverDistance)
    maneuver = maneuver_from_instruction(instruction, self.right_hand_traffic)
    prev = self.maneuver
    self._span.travel(travelled)
    self._ahead_span.travel(travelled)
    route_count, new_route = int(instruction.routeCount), self.valid and int(instruction.routeCount) != self.route_count
    if self._moved_on_from is not None:
      if not new_route and maneuver.kind == NONE and (maneuver.type, maneuver.modifier) == self._moved_on_from:
        # navd still on the step the car is past: the turn after it, by navd's distance to it when it's that one
        following, following_distance = self._following(instruction)
        same = following is not None and prev is not None and (following.type, following.modifier) == (prev.type, prev.modifier)
        self.distance -= travelled
        if same and self._held_id == self.maneuver_id:
          if self._span.plausible(following_distance):
            self._span.take(following_distance)
          self._at_nearest()
        elif same:
          self._span.take(following_distance)
          self.distance = self._span.near
        self.distance_remaining = float(instruction.distanceRemaining)
        return
      self._moved_on_from = None
    advanced = self.distance < ADVANCE_NEAR_M and distance > self.distance + NEW_MANEUVER_JUMP_M
    jumped = distance > self.distance + ROUTE_JUMP_M
    same = self.valid and prev is not None and (maneuver.type, maneuver.modifier) == (prev.type, prev.modifier)
    here = self.valid and prev is not None and same_junction(maneuver, prev)
    if committed and self.valid and prev is not None:
      if not here or not self._span.plausible(distance):
        # navd on another step (moved on early, a new route that doesn't turn there, a jump past the junction's span): a
        # gap, the turn held to its end (module docstring), navd's updates new until then
        self.distance -= travelled
        return
      # the same junction, by the route: a reroute onto the road the car is on, GPS noise either way, navd moving on to
      # the next turn the same way on a route without bearings: one more of its readings (module docstring)
      self._hold()
    if junction_hold and self.valid and prev is not None and not new_route and \
       (not here or not self._span.plausible(distance) or ((advanced or jumped) and distance > self._span.far + NEW_MANEUVER_JUMP_M)):
      # a turn in hand, its desire not out yet (module docstring): navd on another step of the same route (moved on early, a
      # GPS jump past the junction's span, a jump up from close in to beyond every reading of it) is a gap, the turn held. A
      # jump up that its own readings bear out (within NEW_MANEUVER_JUMP_M of the furthest: one short outlier the nearest kept)
      # still re-baselines it below, and a new route is the new route's to decide (a reroute that doesn't turn there: taken)
      self.distance -= travelled
      return
    held = here and self._held_id == self.maneuver_id and (committed or (not new_route and same_bearings(maneuver, prev)))
    # the same junction's readings: a held one's (a reroute's too: its snap only widens the span), and a new route's
    # when its bearings say it is that junction and it is close to where it was
    junction = held or (here and not (advanced or jumped) and
                        (not new_route or (same_bearings(maneuver, prev) and abs(distance - self.distance) <= SAME_TURN_M)))
    if junction:
      if not held or self._span.plausible(distance):
        self._span.take(distance)
    else:
      self._span = JunctionSpan(distance)
    if held:
      # the junction is where its readings put it: while the turn is committed, and after it to its pass while navd is
      # still on its step (by its bearings: without them the next step may be another turn the same way)
      advanced, jumped = False, False
    self.route_count = route_count
    new = not same or advanced or jumped or (new_route and not committed)
    if new:
      self._log_new(maneuver, distance)
      self.replaced = self.valid and (new_route or (jumped and self.distance > REPLACED_FROM_M))
      if self.valid and not self.replaced and self.distance < PASSED_NEAR_M and self._passed_id != self.maneuver_id:
        self.passed = True
      self.maneuver_id += 1
    self.valid = True
    self.maneuver = maneuver
    self.distance = self._span.near  # the nearest its readings put it (module docstring)
    if new and distance < 0. and self._span.far < 0. and maneuver.kind in (TURN, KEEP):
      # a turn or keep already behind the car (a stale reroute's first step, 1e 9:43): passed from the start, so no pulse, no
      # tooLate skip, no skip mark for navd; navd moves on from it as the car drives on (module docstring). By its own
      # reading and every one of a span carried over (a reroute's same junction): one short outlier the nearest kept is no pass
      self._passed_id = self.maneuver_id
      cloudlog.event('turnpilot.maneuver_behind', distance=round(distance, 1), near=round(self._span.near, 1),
                     far=round(self._span.far, 1), type=maneuver.type, modifier=maneuver.modifier, route_count=route_count)
    self.distance_remaining = float(instruction.distanceRemaining)
    self.through_lanes = lanes_from_list(instruction.throughLanes)
    self.through_distance = float(instruction.throughLanesDistance) if self.through_lanes else float('inf')
    prev_ahead = self.ahead
    self.ahead, self.ahead_distance = None, float('inf')
    if maneuver.kind == NONE:  # a bend or a road going on: the turn after it may be close
      self.ahead, self.ahead_distance = self._following(instruction)
    if self.ahead is not None and prev_ahead is not None and not new and same_junction(self.ahead, prev_ahead):
      self._ahead_span.take(self.ahead_distance)
      self.ahead_distance = self._ahead_span.near
    else:
      self._ahead_span = JunctionSpan(self.ahead_distance) if self.ahead is not None else JunctionSpan()
