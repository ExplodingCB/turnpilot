"""Lane planner: get into the lane a turn leaves from, one nav lane change at a time, each warned of, confirmed and
checked until it starts.

Which lane the car is in comes from what the model sees: lanes either side counted from the room between the
ego lane's line and the road edge, at about 3.6 m a lane, but only where the next line out is painted (room
alone counts shoulders and parking strips as lanes). modelV2 can't tell a yellow line from a white one, and on
an undivided road its far road edge lies beyond the oncoming lanes; the lane paint read off the road camera
(scene/lane_paint.py, one entry per modelV2 lane line) can. US markings say what is beyond a line: broken white
is a lane going the same way, solid white an edge, shoulder, bike or parking lane, yellow oncoming traffic or a
divided road's left edge.

Where to go (desired_changes): with the map's lane list (NavInstruction.lanes) the car's lane is counted from the curb
side, which is always its own direction's, and lanes beyond a yellow line count 0. That count is known only where the model
can make it: a curb lane often has no edge line, and one lane in from it the model sees room for a lane but no line, so it
can't tell the curb lane from the one beside it. The changes made for the maneuver are counted too (LanePlanner.done), and
the lane dead reckoned from where the first of them started: no more changes than the lane list needs from there, and none
from a lane it can't place. A change is counted only once the model's lines show the car got over (LineWatch), or its lane
counts moved over: DesireHelper ends a change the same way whether or not it did. One that ends without, or a line crossed
with no change of DesireHelper's (openpilot off, a manual change), leaves the lane unknown, and a lane the model places is
never overridden by the count (nav_lane_span). A lane it can't place is said out loud (lane_unknown:
turnpilotPlan.laneMismatch laneUnknown) instead of changed from. On the far side from the curb only the list's own lanes
are lanes (nav_room): on an undivided road the model's far road edge lies
beyond the oncoming lanes (BeamNG, 2026-10-27: a second change left across a double yellow into the oncoming lane). Without it (the usual case: OSM tags
lanes on few roads) an exit moves toward the curb by the model's lane count, but not across a curb-side line read solid
white, and a turn, or an exit away from the curb, moves one lane toward its side at a time, looked at again after each
change.

Whether the next change may go (change_blocker), on every path, map lanes or not: only into a valid travel lane, which
means all of
- the ego line that way reads broken white at BROKEN_WHITE_CONF_MIN or better: solid, yellow, no line, unknown, or no
  paint read at all (no detector, the wide camera, a stale scene) is no change. A yellow read keeps vetoing for
  YELLOW_HOLD_S and YELLOW_HOLD_M after the last one, so a worn stretch doesn't open a gap; the hold moves with the lines
  in a lane change, and covers all four lines when the direction of one is unknown;
- the model sees a lane that way, its next line out at least LANE_MIN lane widths beyond the ego line (an 8 ft parking
  strip beside an 11 ft lane is 0.73, a 6 ft bike lane beside a 10 ft one 0.61), not beyond a yellow line;
- nothing stopped in that lane ahead (parked cars: a parking strip);
- under US markings (MUTCD) only: in left-hand traffic the center line is broken white, so nothing goes away from the curb
  there, and nothing tells right-hand Europe (white center lines too) from the US.
Before the paint, a change toward the curb before a junction turn put the car in a parking strip (BeamNG, 2026-10-27).

How a change goes (LanePlanner): once the same change is wanted for LC_CONFIRM_FRAMES, the driver is warned LC_WARN_S
ahead: laneChangeOffer says the side (selfdrived raises turnpilotLaneChangeLeft/Right, a chime) and laneChangeCountdown the
seconds left. On cars where openpilot can't signal (the Tesla: opendbc has no blinker command), the driver's stalk is the
go-ahead, at any point in the countdown or up to LC_SIGNAL_TIMEOUT_S after it; elsewhere the request goes when the
countdown ends. The request stands in for the nudge (and, where openpilot signals, the blinker) and DesireHelper still
waits for the car's own blind spot signal (Tesla DAS_status). Until DesireHelper starts the change, every frame checks it
again, and any failure aborts it, with laneChangeAbort saying why for ABORT_HOLD_S (selfdrived raises
turnpilotLaneChangeAborted): the line no longer broken white or the lane no longer valid, the blind spot occupied, a car in
the target lane, no signal in time (or the blinker off before the change started), the route changed, the change no
longer needed, the car too slow or too close to the maneuver, DesireHelper waiting too long. Once the change has started
nothing cancels it: the car is already moving over. Where the car works its own turn signal (turnpilot/car_signals.py,
a verified Tesla), the lamp goes on with the warning and the request waits, past the countdown, until it has been lit for
the legal distance (litM >= leadM, IC 9-21-8-25), and the driver's lamp the other way or hazards (the port's block) are
the driver declining the change: aborted noSignal. Who signals is latched per offer: one made with the car signalling
keeps the car's path unless the car's signal fails (then the driver's path, prompted); one made with the driver
signalling keeps it to its end. A change the driver makes alone gets the same cooldown before the next
offer, for the lines to settle. TurnPilotLanePaintActive off plans no change at all; turnpilotd then publishes what this
would have asked for in laneChangePaintRule.

A map stop between the car and the maneuver (stop_d: MapControlPolicy.stop_ahead) inside the window (BeamNG lane_change on
the signal tiles, 2026-10-28: a signal 164 m before the turn, and braking for it kept the car under DesireHelper's speed
minimum from 227 m in, so no change was offered). Its braking curve takes the car under that minimum slow_zone(v) before
it, so no change starts there (the window's end moves out to it), and the window opens early enough before that for the
offer, its warning and DesireHelper's start (LC_PRE_LEAD_S). With less room than LC_OFFER_ROOM_S at speed before the slow
zone, the change goes after the stop's go when the road from the stop to the window's end can take the car up to speed
and through an offer (AFTER_GO_M): wait afterStop until then. With neither, the lane is unreachable: said (wait
unreachable, and the reason), and where the map's lane list places the car in a lane the maneuver doesn't leave from,
LaneDecision.unreachable: turnpilotd skips the maneuver by the too-fast rules (turn_skip.py: out loud, navd reroutes),
never a turn from the wrong lane or a change too late. Every frame LaneDecision.wait says why a change the route wants
isn't offered (turnpilotPlan.laneChangeWait), and want how many it wants (laneChangeWant), whatever the car's speed.
"""
import math
from dataclasses import dataclass, field, replace
from enum import IntEnum
from typing import NamedTuple

from openpilot.cereal import custom, log
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import ABORT_HOLD_S, ABORT_TEXT
from openpilot.selfdrive.turnpilot.route_follower import KEEP, LEFT, RIGHT, TURN
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint

LaneChange = custom.TurnpilotPlan.LaneChange
LaneMismatch = custom.TurnpilotPlan.LaneMismatch
LaneChangeState = log.LaneChangeState

LANE_WIDTH = 3.6
SAMPLE_IDX = 8              # modelV2 point ~12 m ahead (X_IDXS[8])
LINE_PROB_MIN = 0.5
EDGE_STD_MAX = 1.2          # a road edge is trusted when its std is under this
SHOULDER_LANES = 0.75       # room narrower than this many lane widths beyond the ego line is a shoulder
LANE_MIN = 0.85             # a lane beside the ego lane has its next line out this many lane widths away or more (to 1.5)
KEEP_MIN_ROOM_M = 1.0       # a keep desire toward a trusted edge closer than this beyond the ego line drives off the road
BROKEN_WHITE_CONF_MIN = 0.6  # the ego line that way read broken white at this confidence or better allows a change
SOLID_WHITE_CONF_MIN = 0.6   # a curb-side line read solid white at this confidence stops an exit's changes across it
YELLOW_CONF_MIN = 0.3        # a line read yellow at this confidence vetoes changes across it (the cautious answer, so a lower bar)
YELLOW_HOLD_S, YELLOW_HOLD_M = 5., 100.  # and keeps vetoing until this long and this far after its last yellow read
STOPPED_V = 2.               # m/s: an object this slow in the target lane, up to STOPPED_X_M ahead, makes it no travel lane
STOPPED_X_M = 80.

LC_START_S, LC_START_MIN_M, LC_START_MAX_M = 20., 150., 400.  # lane changes start this far before the maneuver...
LC_END_S, LC_END_MIN_M, LC_END_MAX_M = 6., 60., 120.          # ...and none start closer than this
LC_EXTRA_LANE_S = 8.        # each change past the first starts this much earlier (an exit across a highway)
LC_SPEED_MARGIN = 0.5       # m/s above DesireHelper's minimum, so a request never waits below it
LC_CONFIRM_FRAMES = 10      # the same need this many frames in a row before the warning
LC_WARN_S = 3.              # the warning (chime, countdown) comes this long before the change may go
LC_SIGNAL_TIMEOUT_S = 8.    # after the countdown, the driver has this long to signal before the change is aborted
LC_COOLDOWN_S = 3.          # after any lane change, ours or the driver's, for the model's lane lines to settle
LC_ABORT_COOLDOWN_S = 6.    # after an abort, before the same change is offered again
LC_REQUEST_TIMEOUT_S = 8.   # DesireHelper waiting this long (blindspot) aborts the request
# an abort once the request is out waits this long, the request withdrawn, to see whether DesireHelper started the change
# already: modeld runs it on the plan it last had, and turnpilotd sees its state a modelV2 or two later
LC_ABORT_SETTLE_S = 0.2
LC_BLOCK_X_M = 15.          # a vehicle in the target lane within this, ahead or behind, blocks a request...
LC_CLOSING_T_S = 3.         # ...as does one that closes to that distance within this
LC_LANE_MARGIN_M = 0.5
# a map stop in the window (module docstring)
LC_SPEED_MIN = LANE_CHANGE_SPEED_MIN + LC_SPEED_MARGIN  # m/s: the change's speed
STOP_CURVE_DECEL = 1.5      # m/s^2: map_controls.MAP_STOP_DECEL, the braking curve to a map stop...
STOP_CURVE_LAG_S = 1.       # ...after a second's lag (scene_policy.STOP_LAG_S)
LC_DH_START_S = 1.5         # s from the request to DesireHelper starting the change, blind spot clear
LC_OFFER_ROOM_S = LC_WARN_S + LC_DH_START_S  # s at speed an offer needs before the slow zone: the warning and the start
LC_PRE_LEAD_S = LC_CONFIRM_FRAMES * DT_MDL + LC_OFFER_ROOM_S + 2.  # s: the window before a stop opens this long ahead of its
                                                                    # slow zone: the confirming frames, the offer, 2 s spare
GO_ACCEL = 1.0              # m/s^2: the model pulls away from a stop at about this, or faster
# m from the stop to the window's end the change needs after the go: up to LC_SPEED_MIN at GO_ACCEL (49 m), then the
# confirming frames and the offer at that speed (50 m)
AFTER_GO_M = LC_SPEED_MIN ** 2 / (2. * GO_ACCEL) + LC_SPEED_MIN * (LC_CONFIRM_FRAMES * DT_MDL + LC_OFFER_ROOM_S)

LANE_UNKNOWN_REASON = "can't confirm lane"
AFTER_STOP_REASON = "lane change after the stop"
UNREACHABLE_REASON = "can't reach the turn lane before the stop"

SIDE_DIRECTIONS = {LEFT: ("left", "slightLeft"), RIGHT: ("right", "slightRight")}


def offer_room_s(v_ego: float, lead_m: float | None) -> float:
  """s at speed an offer needs: the warning and DesireHelper's start, or, where the car signals, the lamp's legal lead
  if that is longer than the warning (at 25 mph, 300 ft is 8.2 s)."""
  if lead_m is None:
    return LC_OFFER_ROOM_S
  return max(LC_WARN_S, lead_m / max(v_ego, LC_SPEED_MIN)) + LC_DH_START_S


@dataclass(frozen=True)
class ModelLanes:
  left: int = 0               # lanes the model sees beside the ego lane
  right: int = 0
  keep_left_ok: bool = False  # a keep desire that way has a lane line and road beside it
  keep_right_ok: bool = False
  width: float = LANE_WIDTH
  measured: bool = field(default=False, compare=False)  # the model reads both ego lane lines: the counts mean something
  # the count that way is what is there: every lane counted from a painted line and a trusted edge, or no room for one. Not
  # where the next line out is unpainted but the trusted edge leaves room for a lane (a curb lane with no edge line, seen
  # from the lane beside it), nor where the edge is unsure (the paint alone says at least one)
  left_known: bool = field(default=True, compare=False)
  right_known: bool = field(default=True, compare=False)
  # the ego lane lines' y at SAMPLE_IDX (modelV2: positive right), nan unknown: LineWatch sees the car cross a line by them
  left_y: float = field(default=math.nan, compare=False)
  right_y: float = field(default=math.nan, compare=False)


def _side(ego_y, ego_prob, outer_y, outer_prob, edge_y, edge_std, sign, width):
  """(lanes counted on one side, keep ok that way, the count known: ModelLanes); sign is +1 for the right, -1 for the left
  (modelV2 y is right)."""
  room = sign * (edge_y - ego_y)
  painted = outer_prob > LINE_PROB_MIN and LANE_MIN * width <= sign * (outer_y - ego_y) <= 1.5 * width
  edge_ok = edge_std < EDGE_STD_MAX
  if not painted or (edge_ok and room < SHOULDER_LANES * width):
    lanes = 0
  else:
    lanes = max(int(math.floor(room / width + 0.25)), 1) if edge_ok else 1
  keep_ok = ego_prob > LINE_PROB_MIN and not (lanes == 0 and edge_ok and room < KEEP_MIN_ROOM_M)
  known = edge_ok and (painted or room < SHOULDER_LANES * width)
  return lanes, keep_ok, known


def model_lanes(line_ys, line_probs, edge_ys, edge_stds) -> ModelLanes:
  """From modelV2 laneLines [far left, left, right, far right] and roadEdges [left, right] at one distance ahead."""
  measured = line_ys[2] - line_ys[1]
  width = measured if min(line_probs[1], line_probs[2]) > LINE_PROB_MIN and 2.5 <= measured <= 4.8 else LANE_WIDTH
  left, keep_left, left_known = _side(line_ys[1], line_probs[1], line_ys[0], line_probs[0], edge_ys[0], edge_stds[0], -1., width)
  right, keep_right, right_known = _side(line_ys[2], line_probs[2], line_ys[3], line_probs[3], edge_ys[1], edge_stds[1], 1., width)
  measured = min(line_probs[1], line_probs[2]) > LINE_PROB_MIN
  return ModelLanes(left, right, keep_left, keep_right, width, measured, left_known, right_known, float(line_ys[1]), float(line_ys[2]))


def model_lanes_from_msg(model) -> ModelLanes:
  lines, edges = model.laneLines, model.roadEdges
  if len(lines) < 4 or len(edges) < 2 or len(model.laneLineProbs) < 4 or len(model.roadEdgeStds) < 2 or \
     any(len(e.y) <= SAMPLE_IDX for e in list(lines) + list(edges)):
    return ModelLanes(left_known=False, right_known=False)
  return model_lanes([ln.y[SAMPLE_IDX] for ln in lines], list(model.laneLineProbs),
                     [e.y[SAMPLE_IDX] for e in edges], list(model.roadEdgeStds))


def ego_lane_index(lane_count: int, lanes: ModelLanes, right_hand_traffic: bool) -> int | None:
  """The car's lane in a nav lane list of lane_count, from the left, counted from the curb side (always its own
  direction's): modelV2's far road edge on an undivided road lies beyond the oncoming lanes. None when the model's count on
  the curb side isn't known (ModelLanes.right_known)."""
  if not (lanes.right_known if right_hand_traffic else lanes.left_known):
    return None
  idx = lane_count - 1 - lanes.right if right_hand_traffic else lanes.left
  return min(max(idx, 0), lane_count - 1)


def _nav_lanes(maneuver) -> tuple:
  """The maneuver's lane list when the lane planner goes by it: a turn's or an exit's, of two lanes or more."""
  if maneuver is None or not (maneuver.kind == TURN or (maneuver.kind == KEEP and maneuver.type == "off ramp")):
    return ()
  return maneuver.lanes if len(maneuver.lanes) >= 2 else ()


def _valid_lanes(maneuver, nav) -> list[int]:
  return ([i for i, (_, active) in enumerate(nav) if active] or
          [i for i, (dirs, _) in enumerate(nav) if any(d in SIDE_DIRECTIONS[maneuver.side] for d in dirs)] or
          [0 if maneuver.side == LEFT else len(nav) - 1])


def nav_lane_span(lane_count: int, lanes: ModelLanes, right_hand_traffic: bool, done: int | None = 0, start=None) -> tuple[int, int]:
  """(first, last): the lanes of a nav lane list of lane_count, from the left, the car may be in. done: lane changes
  completed for this maneuver, positive left, None when one went a way that wasn't seen; start: (lane, done) where the
  first change of ours for it started, None before. From the model's count (ego_lane_index) and the lane dead reckoned
  from start; where they disagree the car may be in either, and with neither, anywhere done allows. Either one outside
  where done changes can have taken the car from anywhere in the list is a contradiction (a change counted that the car
  never made, or the list not describing the road here): the car may be anywhere. A lane the model places is never moved
  into the counted range. lanes: paint_lanes' already."""
  lo, hi = (max(-done, 0), min(lane_count - 1 - done, lane_count - 1)) if done is not None else (0, lane_count - 1)
  if lo > hi:  # more changes one way than the list has lanes: it doesn't describe the road here
    lo, hi = 0, lane_count - 1
  found = []
  idx = ego_lane_index(lane_count, lanes, right_hand_traffic)
  if idx is not None:
    found.append(idx)
  if start is not None and done is not None:
    found.append(start[0] - (done - start[1]))
  if any(not lo <= i <= hi for i in found):
    return 0, lane_count - 1
  return (min(found), max(found)) if found else (lo, hi)


def nav_room(maneuver, lanes: ModelLanes, direction: int, right_hand_traffic: bool = True, paint=(), done: int | None = 0,
             start=None) -> int | None:
  """Lanes of the maneuver's lane list beyond the car toward direction (LEFT or RIGHT), wherever in it the car may be
  (nav_lane_span): the only lanes that way known to go its way. None without a lane list."""
  nav = _nav_lanes(maneuver)
  if not nav or direction not in (LEFT, RIGHT):
    return None
  lo, hi = nav_lane_span(len(nav), paint_lanes(lanes, paint), right_hand_traffic, done, start)
  return lo if direction == LEFT else len(nav) - 1 - hi


def lane_unconfirmed(maneuver, lanes: ModelLanes, right_hand_traffic: bool = True, paint=(), done: int | None = 0, start=None) -> bool:
  """The maneuver has a lane list, and the car may be in a lane it doesn't leave from or in one it does, which can't be
  told apart (nav_lane_span): no change is planned, and the driver is told (LanePlanner.lane_unknown)."""
  nav = _nav_lanes(maneuver)
  if not nav:
    return False
  lo, hi = nav_lane_span(len(nav), paint_lanes(lanes, paint), right_hand_traffic, done, start)
  valid = _valid_lanes(maneuver, nav)
  return lo < hi and not all(i in valid for i in range(lo, hi + 1))


def lane_mismatch(maneuver, distance: float, through_lanes: tuple, through_distance: float, lanes: ModelLanes, v_ego: float,
                  right_hand_traffic: bool = True):
  """The car is in a turn-only lane where the route goes straight through, or not in a lane that turns where the route
  turns, closer than the lane planner still changes lanes (lane_change_window's end). Only flagged, never acted on:
  LaneMismatch.none without the map's lane list, without the model reading the ego lane's lines or counting the lanes on
  the curb side."""
  if not lanes.measured:
    return LaneMismatch.none
  near = lane_change_window(v_ego)[1]
  turn = maneuver is not None and maneuver.kind == TURN
  if len(through_lanes) >= 2 and 0. < through_distance < near and not (turn and distance < through_distance):
    idx = ego_lane_index(len(through_lanes), lanes, right_hand_traffic)
    directions, active = through_lanes[idx] if idx is not None else ((), True)
    if not active and directions and "straight" not in directions:
      return LaneMismatch.turnOnlyGoingStraight
  if turn and len(maneuver.lanes) >= 2 and 0. < distance < near:
    idx = ego_lane_index(len(maneuver.lanes), lanes, right_hand_traffic)
    directions, active = maneuver.lanes[idx] if idx is not None else ((), True)
    if not active and not any(d in SIDE_DIRECTIONS[maneuver.side] for d in directions):
      return LaneMismatch.wrongLaneForTurn
  return LaneMismatch.none


def _yellow(p: LinePaint) -> bool:
  return p.kind != 'none' and p.color == 'yellow' and p.confidence >= YELLOW_CONF_MIN


LATCHED_YELLOW = LinePaint('unknown', 'yellow', 1., 0.)


def paint_lanes(lanes: ModelLanes, paint) -> ModelLanes:
  """The model's lane counts with every lane beyond a yellow line taken out. paint: one LinePaint per modelV2 lane line,
  [far left, left, right, far right], or empty when it wasn't read."""
  if len(paint) != 4:
    return lanes
  def side(n, ego, outer):
    return 0 if _yellow(ego) else min(n, 1) if _yellow(outer) else n
  return replace(lanes, left=side(lanes.left, paint[1], paint[0]), right=side(lanes.right, paint[2], paint[3]))


def broken_white_toward(paint, side) -> bool:
  """The ego lane's line toward side (LEFT or RIGHT) reads broken white: a lane going the same way is beyond it."""
  if len(paint) != 4 or side not in (LEFT, RIGHT):
    return False
  return paint[1 if side == LEFT else 2].is_('broken', 'white', BROKEN_WHITE_CONF_MIN)


def stopped_in_lane(objects, direction: int, width: float = LANE_WIDTH) -> bool:
  """Something stopped up to STOPPED_X_M ahead in the lane one over toward direction (LEFT or RIGHT): a parked car says
  that lane is a parking strip. objects: TurnpilotScene.TrackedObject-like, y positive left, vx over ground."""
  return any(0. < obj.x < STOPPED_X_M and abs(obj.vx) < STOPPED_V and 0.5 * width < obj.y * direction < 1.5 * width + LC_LANE_MARGIN_M
             for obj in objects)


def _toward(direction: int):
  return LaneChange.left if direction == LEFT else LaneChange.right if direction == RIGHT else LaneChange.none


def _direction(lc) -> int:
  return LEFT if lc == LaneChange.left else RIGHT if lc == LaneChange.right else 0


def desired_changes(maneuver, lanes: ModelLanes, right_hand_traffic: bool = True, paint=(), done: int | None = 0, start=None) -> int:
  """Lane changes the route wants before the maneuver, positive left, negative right, before change_blocker has a say.
  paint: as for paint_lanes; done, start: as for nav_lane_span. With a lane list, the fewest changes any lane the car may
  be in needs, and none when it may already be in one the maneuver leaves from."""
  if maneuver is None or not (maneuver.kind == TURN or (maneuver.kind == KEEP and maneuver.type == "off ramp")):
    return 0
  lanes = paint_lanes(lanes, paint)
  curb = RIGHT if right_hand_traffic else LEFT
  nav = _nav_lanes(maneuver)
  if nav:
    valid = _valid_lanes(maneuver, nav)
    lo, hi = nav_lane_span(len(nav), lanes, right_hand_traffic, done, start)
    return lo - max(valid) if lo > max(valid) else hi - min(valid) if hi < min(valid) else 0
  if maneuver.side == curb and maneuver.kind == KEEP:  # an exit: over to the curb lane by the model's count
    n = lanes.left if curb == LEFT else lanes.right
    if len(paint) == 4:  # not across a curb-side line read solid white: a shoulder or a bike lane is beyond it
      ego, outer = (paint[1], paint[0]) if curb == LEFT else (paint[2], paint[3])
      n = 0 if ego.is_('solid', 'white', SOLID_WHITE_CONF_MIN) else min(n, 1) if outer.is_('solid', 'white', SOLID_WHITE_CONF_MIN) else n
    return n if curb == LEFT else -n
  return 1 if maneuver.side == LEFT else -1  # a turn, or an exit away from the curb: one toward its side at a time


def change_blocker(direction: int, lanes: ModelLanes, paint=(), objects=(), right_hand_traffic: bool = True,
                   nav_room: int | None = None) -> str:
  """'' when one lane change toward direction (LEFT or RIGHT) is into a valid travel lane, else the abort reason why not
  (a TurnpilotPlan.LaneChangeAbort name). paint: as for paint_lanes, held yellow included; objects: as for stopped_in_lane;
  nav_room: as nav_room() returns it, None without the map's lane list."""
  if direction not in (LEFT, RIGHT):
    return 'noLane'
  if not right_hand_traffic and direction == RIGHT:
    return 'noLane'  # away from the curb in left-hand traffic: the center line is broken white there
  if not broken_white_toward(paint, direction):
    return 'lineNotBroken'
  if (paint_lanes(lanes, paint).left if direction == LEFT else paint_lanes(lanes, paint).right) == 0:
    return 'noLane'  # the model sees no lane that way, or only beyond a yellow line
  if nav_room is not None and nav_room <= 0:
    return 'noLane'  # the map's lane list has no lane that way: the model's are oncoming, or not where it can place the car
  if stopped_in_lane(objects, direction, lanes.width):
    return 'laneParked'
  return ''


def lane_changes_needed(maneuver, lanes: ModelLanes, right_hand_traffic: bool = True, paint=(), objects=(), done: int | None = 0,
                        start=None) -> int:
  """Lane changes the maneuver still needs, positive left, negative right, or 0 when the next one may not go
  (change_blocker). paint: as for paint_lanes; objects: as for stopped_in_lane, empty when unknown; done, start: as for
  nav_lane_span."""
  need = desired_changes(maneuver, lanes, right_hand_traffic, paint, done, start)
  direction = LEFT if need > 0 else RIGHT
  room = nav_room(maneuver, lanes, direction, right_hand_traffic, paint, done, start)
  if need and change_blocker(direction, lanes, paint, objects, right_hand_traffic, room):
    return 0
  return need


def lane_change_window(v_ego: float, changes: int = 1) -> tuple[float, float]:
  """(start, end) m before the maneuver, for this many lane changes still to make."""
  start = min(max(LC_START_S * v_ego, LC_START_MIN_M), LC_START_MAX_M) + max(changes - 1, 0) * LC_EXTRA_LANE_S * v_ego
  end = min(max(LC_END_S * v_ego, LC_END_MIN_M), LC_END_MAX_M)
  return start, end


def slow_zone(v_ego: float) -> float:
  """m before a map stop where its braking curve takes the car under LC_SPEED_MIN, approaching at v_ego."""
  return LC_SPEED_MIN ** 2 / (2. * STOP_CURVE_DECEL) + max(v_ego, LC_SPEED_MIN) * STOP_CURVE_LAG_S


def stop_window(v_ego: float, distance: float, stop_d: float | None, changes: int = 1,
                room_s: float = LC_OFFER_ROOM_S) -> tuple[float, float, str]:
  """(start, end, wait) of the lane change window for this many changes with a map stop stop_d m ahead (module docstring),
  the maneuver distance m ahead. wait: '' (offer in the window), 'afterStop' or 'unreachable'. A stop past the maneuver, or
  one before the window at the change's speed (the car is up to speed again by its start), changes nothing."""
  start, end = lane_change_window(v_ego, changes)
  if stop_d is None or not 0. <= stop_d < distance:
    return start, end, ''
  before = distance - stop_d  # m before the maneuver the stop is
  v = max(v_ego, LC_SPEED_MIN)
  if before >= lane_change_window(v, changes)[0]:  # slowing for it shrinks the window: not the window at speed
    return start, end, ''
  end_stop = max(end, before + slow_zone(v))
  if distance - end_stop >= v * room_s:
    return max(start, end_stop + v * (LC_PRE_LEAD_S + room_s - LC_OFFER_ROOM_S)), end_stop, ''
  if before <= end:
    return start, end_stop, ''  # the junction's own stop, past the window's end: its braking ends the window, nothing more
  # after the go the car is at about LC_SPEED_MIN through the window's end
  after_go = AFTER_GO_M + LC_SPEED_MIN * (room_s - LC_OFFER_ROOM_S)
  return start, end, 'afterStop' if before - lane_change_window(LC_SPEED_MIN)[1] >= after_go else 'unreachable'


def target_lane_blocked(objects, direction: int, v_ego: float, width: float = LANE_WIDTH) -> bool:
  """A vehicle in the lane one over toward direction (LEFT or RIGHT) within LC_BLOCK_X_M, or closing to it fast.
  objects: TurnpilotScene.TrackedObject-like, y positive left, vx over ground."""
  for obj in objects:
    lateral = obj.y * direction
    if not (0.5 * width < lateral < 1.5 * width + LC_LANE_MARGIN_M):
      continue
    rel = obj.vx - v_ego
    if abs(obj.x) < LC_BLOCK_X_M:
      return True
    if obj.x > LC_BLOCK_X_M and rel < 0 and (obj.x - LC_BLOCK_X_M) / -rel < LC_CLOSING_T_S:
      return True
    if obj.x < -LC_BLOCK_X_M and rel > 0 and (-LC_BLOCK_X_M - obj.x) / rel < LC_CLOSING_T_S:
      return True
  return False


class YellowHold:
  """Per modelV2 lane line, how long and how far since it was last read yellow; it reads yellow until both pass
  YELLOW_HOLD_S and YELLOW_HOLD_M. Lane changes move the holds with the lines."""
  def __init__(self):
    self.since = [(math.inf, math.inf)] * 4

  def update(self, paint, v_ego: float, dt: float, frozen: bool) -> tuple:
    """paint as for paint_lanes; frozen: a lane change is under way, so the lines may be renumbering. Returns the paint with
    every held line read yellow (four entries whenever anything is held)."""
    self.since = [(t + dt, d + v_ego * dt) for t, d in self.since]
    if len(paint) == 4 and not frozen:
      self.since = [(0., 0.) if _yellow(p) else hold for p, hold in zip(paint, self.since, strict=True)]
    held = [t < YELLOW_HOLD_S or d < YELLOW_HOLD_M for t, d in self.since]
    if not any(held):
      return tuple(paint)
    paint = tuple(paint) if len(paint) == 4 else (UNKNOWN,) * 4
    return tuple(p if _yellow(p) or not h else LATCHED_YELLOW for p, h in zip(paint, held, strict=True))

  def shift(self, direction: int) -> None:
    """The car moved one lane toward direction (LEFT or RIGHT): each line is now one index further the other way. With the
    direction unknown, any hold covers all four lines, since it could be any of them now."""
    none = (math.inf, math.inf)
    if direction == LEFT:
      self.since = [none] + self.since[:3]
    elif direction == RIGHT:
      self.since = self.since[1:] + [none]
    else:
      latest = min(self.since)
      self.since = [latest] * 4


class LineWatch:
  """The car crossing a lane line, by the model's ego lane lines: they slide the other way as the car moves over, and the
  model relabels them once it is across (the line it crossed becomes the far one). Moving toward a side, the line that way
  slides in past CROSS_FRAC lane widths from where it lies, then is seen WRAP_FRAC lane widths out the far side of it: or
  the line on the other side slides out, then back in past the middle. Either, within WATCH_S, is one lane moved.
  DesireHelper's state alone can't say a change was made: it ends one the same way whether the car got over or not (lateral
  dropped, the model's lane change probability stayed low, LANE_CHANGE_TIME_MAX)."""
  CROSS_FRAC, WRAP_FRAC = 0.3, 0.15
  WATCH_S = 4.

  def __init__(self):
    self.reset()

  def reset(self):
    self.moved = 0  # lanes moved by the lines, positive left
    self._crossed = {LEFT: [-1., -1.], RIGHT: [-1., -1.]}  # s since the line on that side, the other, slid past crossing

  def update(self, lanes: ModelLanes, dt: float) -> None:
    if math.isnan(lanes.left_y) or math.isnan(lanes.right_y):
      return
    w = lanes.width
    for k in (LEFT, RIGHT):
      # u: each line's y the way the lines slide when the car moves toward k; the line on that side lies at -w/2, the other +w/2
      lead, trail = (lanes.left_y, lanes.right_y) if k == LEFT else (-lanes.right_y, -lanes.left_y)
      timers = self._crossed[k]
      for i, (u, nominal) in enumerate(((lead, -w / 2), (trail, w / 2))):
        if timers[i] >= 0.:
          timers[i] += dt
          if timers[i] > self.WATCH_S:
            timers[i] = -1.
        if u >= nominal + self.CROSS_FRAC * w:
          timers[i] = 0.
        elif timers[i] >= 0. and u <= nominal - self.WRAP_FRAC * w:
          self.moved += k
          self._crossed = {LEFT: [-1., -1.], RIGHT: [-1., -1.]}
          return


def counts_shifted(before: ModelLanes | None, after: ModelLanes, direction: int) -> bool:
  """The model's lane counts moved one lane toward direction (LEFT or RIGHT) on a side it counts both times."""
  if before is None:
    return False
  near, far = ('left', 'right') if direction == LEFT else ('right', 'left')
  return (getattr(before, near + '_known') and getattr(after, near + '_known') and getattr(after, near) == getattr(before, near) - 1) or \
    (getattr(before, far + '_known') and getattr(after, far + '_known') and getattr(after, far) == getattr(before, far) + 1)


class Phase(IntEnum):
  IDLE = 0
  WARN = 1      # the countdown: the driver has been told which way and when
  WAIT = 2      # the countdown is over, waiting for the driver's blinker (cars where openpilot can't signal)
  REQUEST = 3   # DesireHelper has the request; it starts the change when the blind spot is clear


class LaneDecision(NamedTuple):
  lane_change: int   # TurnpilotPlan.laneChange: the request for DesireHelper
  prompt: int        # TurnpilotPlan.signalPrompt: ask the driver to signal
  reason: str        # one short line for the status line
  offer: int         # TurnpilotPlan.laneChangeOffer: the side of the change warned of, until it starts or aborts
  countdown: float   # TurnpilotPlan.laneChangeCountdown: s of the warning left
  abort: str         # TurnpilotPlan.laneChangeAbort: why the last change was aborted, for ABORT_HOLD_S after
  wait: str = 'none'  # TurnpilotPlan.laneChangeWait: why a change the route wants isn't offered now
  want: int = 0       # TurnpilotPlan.laneChangeWant: the changes the route wants, positive left, at any speed
  unreachable: bool = False  # the lane list places the car in a lane the maneuver doesn't leave from, and a map stop leaves
                             # no room to change before it nor after its go: turnpilotd skips the maneuver


class LanePlanner:
  def __init__(self, paint_rule: bool = True):
    self.paint_rule = paint_rule      # TurnPilotLanePaintActive: plan lane changes at all
    self.phase = Phase.IDLE
    self.offer = LaneChange.none      # the change warned of
    self.countdown = 0.
    self.abort = 'none'               # the last abort's reason, held for ABORT_HOLD_S
    self.ours = False                 # a DesireHelper lane change we asked for is under way
    self.changes = 0                  # nav lane changes completed
    self.done: int | None = 0         # lane changes completed for this maneuver, ours or the driver's, positive left; None
                                      # once one went a way that wasn't seen
    self.lane_unknown = False         # in or past the window, the car's lane in the maneuver's lane list can't be confirmed
    self._done_id = None              # the maneuver done counts for
    self._start = None                # (lane, done) where our first change for it started: nav_lane_span
    self._from_lane: int | None = None  # the lane in the lane list the change wanted now is counted from
    self.yellow = YellowHold()
    self.lines = LineWatch()          # the car crossing a line, by the model's lines: a change is counted only on that
    self._before: ModelLanes | None = None  # the model's lanes the frame before the change under way started
    self._last_lanes: ModelLanes | None = None
    self._lc_dir: int | None = None   # direction of the DesireHelper lane change under way, 0 when unknown
    self._maneuver_id = None          # the maneuver the offer is for
    self._cooldown = 0.
    self._abort_t = 0.
    self._candidate = LaneChange.none
    self._confirm = 0
    self._wait_t = 0.
    self._pre_t = 0.
    self._pending = ''                # an abort in REQUEST, said once DesireHelper is seen not to have started
    self._pending_t = 0.
    self._wait, self._want, self._unreachable = 'none', 0, False  # this frame's LaneDecision.wait, want, unreachable
    self._unreachable_logged = None   # the maneuver whose unreachable lane was logged
    self._offer_driver = True         # who signals this offer, latched at its warning (module docstring)

  @property
  def active(self) -> bool:
    return self.phase != Phase.IDLE or self.ours

  @property
  def signal_side(self) -> int:
    """The side to signal for the planner: the offer's from its warning, and ours until the change under way is over."""
    if self.phase in (Phase.WARN, Phase.REQUEST, Phase.WAIT):
      return self.offer
    if self.ours and self._lc_dir:
      return _toward(self._lc_dir)
    return LaneChange.none

  def update(self, maneuver, distance: float, lanes: ModelLanes, lc_state, v_ego: float, left_blinker: bool,
             right_blinker: bool, objects, driver_signals: bool, right_hand_traffic: bool, dt: float, paint=(),
             left_blindspot: bool = False, right_blindspot: bool = False, maneuver_id=None, inputs_valid: bool = True,
             stop_d: float | None = None, lit_m: float = math.inf, lead_m: float | None = None, car_block: bool = False,
             signals_failed: bool = False) -> LaneDecision:
    """lc_state: modelV2.meta.laneChangeState; objects: tracked scene objects, empty when the scene is unavailable
    (DesireHelper's blindspot check still applies); paint: as for paint_lanes, empty when not read; blindspots: carState's;
    maneuver_id: RouteFollower.maneuver_id, a new one is a changed route; inputs_valid: carState and modelV2 pass their
    checks, else nothing is offered and an offer aborts; stop_d: m to the map stop the car will stop at next
    (MapControlPolicy.stop_ahead), None without one. Where the car signals (driver_signals False): lit_m, the m its lamp
    has been lit for the offer's side (car_signals.py), lead_m the legal lead, car_block the port holding off (the
    driver's other side, hazards), signals_failed the car's signal failed mid-offer: the driver's path takes over."""
    self._wait, self._want, self._unreachable = 'none', 0, False
    if self._abort_t > 0.:
      self._abort_t -= dt
      if self._abort_t <= 0.:
        self.abort = 'none'
    if maneuver_id != self._done_id:  # a new maneuver: its changes are counted from here
      self._done_id, self.done, self._start = maneuver_id, 0, None
    changing = lc_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
    paint = self.yellow.update(paint, v_ego, dt, changing)
    if changing and self._lc_dir is None:
      self.lines.reset()
      self._before = self._last_lanes
    self.lines.update(lanes, dt)
    self._last_lanes = lanes
    if changing:  # never cancelled once started: the car is already moving over
      if self._lc_dir is None:  # ours, or the driver's by the blinker
        req = self.offer if self.phase == Phase.REQUEST else \
          LaneChange.left if left_blinker and not right_blinker else LaneChange.right if right_blinker and not left_blinker else LaneChange.none
        self._lc_dir = _direction(req)
        if self.phase == Phase.REQUEST and self._start is None and self._from_lane is not None and self.done is not None:
          self._start = (self._from_lane, self.done)  # the first of ours for this maneuver: dead reckoned from here
      self.ours = self.ours or self.phase == Phase.REQUEST
      self.lane_unknown = False
      self._reset()
      return self._decision()
    if self._lc_dir is not None:  # a change just ended, ours or the driver's
      made = self._lc_dir != 0 and (self.lines.moved == self._lc_dir or counts_shifted(self._before, lanes, self._lc_dir))
      self.yellow.shift(self._lc_dir if made else 0)
      if made and self.done is not None:
        self.done += self._lc_dir
      else:  # a way not seen, or the car not seen to get there: nothing to dead reckon from for the rest of this maneuver
        self.done, self._start = None, None
      self._lc_dir = None
      self.lines.reset()
      self._cooldown = max(self._cooldown, LC_COOLDOWN_S)
    elif self.lines.moved:  # the car crossed a line with no change of DesireHelper's (openpilot off, a manual change)
      self.yellow.shift(0)
      self.done, self._start = None, None
      self.lines.reset()
    if self.ours:  # DesireHelper is done with it, whether or not a held blinker keeps it in preLaneChange
      self.ours = False
      self.changes += 1

    fast = v_ego > LC_SPEED_MIN
    wanted = desired_changes(maneuver, lanes, right_hand_traffic, paint, self.done, self._start)
    want = wanted if fast else 0
    start, end = lane_change_window(v_ego, abs(wanted))
    room = offer_room_s(v_ego, lead_m if not driver_signals else None)
    win_start, win_end, stop_wait = stop_window(v_ego, distance, stop_d, abs(wanted), room)
    self._want = wanted
    want_dir = LEFT if want > 0 else RIGHT if want < 0 else 0
    nav = _nav_lanes(maneuver)
    if nav and want_dir:
      lo, hi = nav_lane_span(len(nav), paint_lanes(lanes, paint), right_hand_traffic, self.done, self._start)
      self._from_lane = lo if want_dir == LEFT else hi
    else:
      self._from_lane = None
    self.lane_unknown = self.paint_rule and maneuver is not None and 0. < distance < start and \
      lane_unconfirmed(maneuver, lanes, right_hand_traffic, paint, self.done, self._start)
    idle_reason = LANE_UNKNOWN_REASON if self.lane_unknown else ""
    # an offer under way was made with room: its own checks (tooSlow, tooLate) see it through or abort it. Only with
    # nothing offered is the lane unreachable, or would the maneuver it is changing lanes for be skipped mid-offer
    if stop_wait and wanted and not self.lane_unknown and self.phase == Phase.IDLE and not self._pending:
      nav_known = bool(nav) and not lane_unconfirmed(maneuver, lanes, right_hand_traffic, paint, self.done, self._start)
      self._unreachable = stop_wait == 'unreachable' and nav_known and self.paint_rule
      if stop_wait == 'unreachable' and self._unreachable_logged != maneuver_id:
        self._unreachable_logged = maneuver_id
        cloudlog.event("turnpilot.lane_unreachable", error=True, want=wanted, distance=round(distance, 1), stop_d=round(stop_d, 1),
                       v_ego=round(v_ego, 2), lane_list=nav_known)
      idle_reason = idle_reason or (AFTER_STOP_REASON if stop_wait == 'afterStop' else UNREACHABLE_REASON)

    if self._pending:  # the request is withdrawn; DesireHelper didn't start the change (else `changing` above): abort
      self._pending_t -= dt
      if self._pending_t > 1e-6:
        return self._decision(ABORT_TEXT[self._pending])
      reason = self._pending
      self._reset()
      return self._abort(reason)
    if self.phase != Phase.IDLE:
      if not self._offer_driver and signals_failed:
        self._offer_driver = True  # the car's signal failed mid-offer: the driver's path, prompted
      return self._follow_up(maneuver, maneuver_id, distance, end, want_dir, fast, lanes, paint, objects, lc_state, v_ego,
                             left_blinker, right_blinker, left_blindspot, right_blindspot, self._offer_driver, right_hand_traffic, dt,
                             inputs_valid, lit_m, lead_m, car_block)

    wanted_dir = LEFT if wanted > 0 else RIGHT if wanted < 0 else 0
    blocker = change_blocker(wanted_dir, lanes, paint, objects, right_hand_traffic,
                             nav_room(maneuver, lanes, wanted_dir, right_hand_traffic, paint, self.done, self._start)) \
      if wanted_dir else ''
    self._wait = 'none' if maneuver is None else 'paintRuleOff' if not self.paint_rule else \
      'inputsInvalid' if not inputs_valid else 'laneUnknown' if self.lane_unknown else 'none' if not wanted else \
      stop_wait or ('cooldown' if self._cooldown > 0. else 'beforeWindow' if distance >= win_start else
                    'pastWindow' if distance <= win_end else 'tooSlow' if not fast else blocker or 'confirming')
    if self._cooldown > 0.:
      self._cooldown -= dt
      self._confirm = 0
      return self._decision(idle_reason)
    if not self.paint_rule or not inputs_valid or stop_wait or not (win_end < distance < win_start):
      want_dir = 0
    if want_dir and blocker:
      want_dir = 0
    direction = _toward(want_dir)
    self._confirm = self._confirm + 1 if direction == self._candidate else 1
    self._candidate = direction
    if direction == LaneChange.none or self._confirm < LC_CONFIRM_FRAMES:
      return self._decision(idle_reason)
    if left_blindspot if want_dir == LEFT else right_blindspot:
      self._wait = 'blindspot'
      return self._decision("car in blind spot")
    if target_lane_blocked(objects, want_dir, v_ego, lanes.width):
      self._wait = 'carInLane'
      return self._decision("car in target lane")
    self._wait = 'none'
    self.phase, self.offer, self.countdown = Phase.WARN, direction, LC_WARN_S
    self._offer_driver = driver_signals
    self._maneuver_id, self._wait_t, self._pre_t = maneuver_id, 0., 0.
    self.abort, self._abort_t = 'none', 0.
    return self._decision(prompt=driver_signals)

  def _follow_up(self, maneuver, maneuver_id, distance, end, want_dir, fast, lanes, paint, objects, lc_state, v_ego,
                 left_blinker, right_blinker, left_blindspot, right_blindspot, driver_signals, right_hand_traffic, dt,
                 inputs_valid, lit_m=math.inf, lead_m=None, car_block=False) -> LaneDecision:
    """An offered change, every frame until DesireHelper starts it: still wanted, still valid, still clear, still signaled."""
    d = _direction(self.offer)
    if not self.paint_rule:
      return self._abort('turnedOff')
    if not inputs_valid:
      return self._abort('badInputs')
    if maneuver is None or maneuver_id != self._maneuver_id:
      return self._abort('routeChanged')
    if not fast:
      return self._abort('tooSlow')
    if distance <= end:
      return self._abort('tooLate')
    reason = change_blocker(d, lanes, paint, objects, right_hand_traffic,
                            nav_room(maneuver, lanes, d, right_hand_traffic, paint, self.done, self._start))
    if reason:
      return self._abort(reason)
    if want_dir != d:
      return self._abort('notNeeded')  # the model's lane count says the car is in the lane it wants
    if left_blindspot if d == LEFT else right_blindspot:
      return self._abort('blindspot')
    if target_lane_blocked(objects, d, v_ego, lanes.width):
      return self._abort('carInLane')

    signaled = left_blinker if d == LEFT else right_blinker
    if not driver_signals and car_block and self.phase in (Phase.WARN, Phase.REQUEST):
      return self._abort('noSignal')  # the driver's lamp the other way, or hazards: the driver declines the change
    if self.phase == Phase.WARN:
      self.countdown = max(self.countdown - dt, 0.)
      if driver_signals and signaled:  # the driver's go-ahead, any time in the countdown
        self.phase = Phase.REQUEST
      elif self.countdown <= 0. and driver_signals:
        self.phase = Phase.WAIT
      elif self.countdown <= 0. and (lead_m is None or lit_m >= lead_m):
        self.phase = Phase.REQUEST  # the car's lamp has led the change by the legal distance
    elif self.phase == Phase.WAIT:
      self._wait_t += dt
      if signaled:
        self.phase = Phase.REQUEST
      elif self._wait_t > LC_SIGNAL_TIMEOUT_S:
        return self._abort('noSignal')
    elif self.phase == Phase.REQUEST:
      if driver_signals and not signaled:
        return self._abort('noSignal')  # the blinker went off before the change started: never change unsignaled
      if lc_state == LaneChangeState.preLaneChange:
        self._pre_t += dt
        if self._pre_t > LC_REQUEST_TIMEOUT_S:
          return self._abort('timedOut')
    if self.phase == Phase.REQUEST:
      self.countdown = 0.
      return self._decision(request=True, prompt=driver_signals)
    return self._decision(prompt=driver_signals)

  def cancel(self, reason: str) -> bool:
    """TurnPilot stopped acting (reason 'handedBack' or 'fault'): an offered change that DesireHelper hasn't started is
    aborted with reason, held ABORT_HOLD_S as any abort. A change under way goes on: nothing cancels it once started, and
    one DesireHelper may have started on the request already isn't announced as aborted (_abort).
    True when there was an offer."""
    if self.phase == Phase.IDLE:
      return False
    self._abort(reason)
    return True

  def _abort(self, reason: str) -> LaneDecision:
    """Abort the offered change. Once the request is out, DesireHelper may have started it in modeld's last step, which
    turnpilotd sees a modelV2 or two later: the request is withdrawn at once, and the abort said only after
    LC_ABORT_SETTLE_S with no change starting. One that does start is ours, and goes on: no abort contradicts it."""
    if self.phase == Phase.REQUEST:  # (a second abort while one waits, a handback's say, keeps the first reason)
      if not self._pending:
        self._pending, self._pending_t = reason, LC_ABORT_SETTLE_S
      return self._decision(ABORT_TEXT[self._pending])
    self._reset()
    self.abort, self._abort_t = reason, ABORT_HOLD_S
    self._cooldown = max(self._cooldown, LC_ABORT_COOLDOWN_S)
    return self._decision(ABORT_TEXT[reason])

  def _reset(self) -> None:
    self.phase, self.offer, self.countdown = Phase.IDLE, LaneChange.none, 0.
    self._confirm, self._candidate, self._wait_t, self._pre_t = 0, LaneChange.none, 0., 0.
    self._pending, self._pending_t = '', 0.

  def _decision(self, reason: str = "", request: bool = False, prompt: bool = False) -> LaneDecision:
    wait = 'none' if self.phase != Phase.IDLE or self.ours or self._lc_dir is not None else self._wait
    return LaneDecision(self.offer if request else LaneChange.none, self.offer if prompt else LaneChange.none,
                        reason, self.offer, self.countdown, self.abort, wait, self._want, self._unreachable)
