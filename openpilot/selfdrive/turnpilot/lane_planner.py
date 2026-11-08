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
change. A far-side turn whose list's lanes are its far-most (far_bay: [left (active), straight, straight]) wants no change
while its bay hasn't opened: the far-side ego line read yellow, uncontested by white reads of it, the car beside the centre
line, until that line reads white (the bay rule, desired_changes, BayLatch; drive 1e seg 5, 2026-11-06, wanted +2 against
the yellow on a one-lane approach), and nothing says it is in the wrong lane for the turn (lane_mismatch). Only on a list
that marks the turn's lanes itself, never on a guess for one that marks none (far_bay).

Whether the next change may go (change_blocker), on every path, map lanes or not: only into a valid travel lane, which
means all of
- the ego line that way reads broken white at BROKEN_WHITE_CONF_MIN or better, or the reader read it but couldn't make it
  out and the rest agrees a lane of ours is beyond it (lane_agreed: the model sees the line and counts a painted lane beyond
  it, and the line read white, the line beyond read yellow, or the map's lane list has a lane of ours there, toward the far
  side only over a line its reads have cleared; only on a frame whose paint was read, and only while the ego line has read
  neither yellow nor solid since the car's last lane change, unless WHITE_CLEAR_FRAMES white reads of it or the yellow
  moving out came after, LineMarks). Solid, yellow, no line, or no paint read at all (no detector, the wide camera, a stale
  scene) is no change. A yellow read keeps vetoing for YELLOW_HOLD_S and YELLOW_HOLD_M after the last one, so a worn stretch
  doesn't open a gap, unless the yellow moves out a lane (the ego line read white and the line beyond yellow,
  YELLOW_OUT_FRAMES frames of the two since its last yellow or solid read: a turn bay opening, YellowHold); the
  hold moves with the lines in a lane change, and covers all four lines when the direction of one is unknown. After it a
  white read keeps its colour across reads of unknown colour for WHITE_HOLD_S or WHITE_HOLD_M, whichever ends first
  (WhiteHold: a dashed line's colour drops out before its kind does); never over solid, yellow or no line. A frame valid by
  that hold alone keeps an offer's confirm count and keeps an offer up, but nothing is offered, requested or placed for
  LC-03 on it (LanePlanner._held_only);
- the model sees a lane that way, its next line out at least LANE_MIN lane widths beyond the ego line (an 8 ft parking
  strip beside an 11 ft lane is 0.73, a 6 ft bike lane beside a 10 ft one 0.61), not beyond a yellow line; or, with that
  line unseen (at night the usual case), the ego line read broken white and a trusted road edge LANE_MIN lane widths beyond
  it: one lane (lanes_toward), never more, so never one beyond the next line out. The lane width there is the ego lines'
  own spacing when one of them is sure (model_lanes: at night the curb-side line rarely is);
- nothing stopped in that lane ahead (parked cars: a parking strip), and nothing coming toward the car in it (an oncoming
  lane, whatever its line reads: oncoming_in_lane);
- under US markings (MUTCD) only: in left-hand traffic the center line is broken white, so nothing goes away from the curb
  there, and nothing tells right-hand Europe (white center lines too) from the US.
Before the paint, a change toward the curb before a junction turn put the car in a parking strip (BeamNG, 2026-10-27).

How a change goes (LanePlanner): once the same change is wanted for LC_CONFIRM_FRAMES, the driver is warned LC_WARN_S
ahead: laneChangeOffer says the side (selfdrived raises turnpilotLaneChangeLeft/Right, a chime) and laneChangeCountdown the
seconds left. On cars where openpilot can't signal (the Tesla: opendbc has no blinker command), the driver's stalk is the
go-ahead, at any point in the countdown or up to LC_SIGNAL_TIMEOUT_S after it; elsewhere the request goes when the
countdown ends. The request is the lane change itself: DesireHelper takes it with no blinker or nudge and none of
the driver's 20 mph floor, only a moving car (NAV_LANE_CHANGE_SPEED_MIN), and still waits for the car's own blind spot
signal (Tesla DAS_status). Until DesireHelper starts the change, every frame checks it
again, and any failure aborts it, with laneChangeAbort saying why for ABORT_HOLD_S (selfdrived raises
turnpilotLaneChangeAborted): the line read solid, yellow (held included) or none, or the lane no longer valid (a line read
short of broken white with nothing contrary, its kind or colour unknown, or broken white by the white hold alone, a frame
with no paint read at all, or no lane by the model's count alone with the count unknown, waits instead, nothing requested
on that frame, for LC_UNSURE_MAX_S at most: soft_block, _follow_up; before the offer the same frames keep its confirm
count), the blind spot
occupied, a car in
the target lane, no signal in time (or the blinker off before the change started), the route changed, the change no
longer needed, the car too slow or too close to the maneuver, DesireHelper waiting too long. Once the change has started
nothing cancels it: the car is already moving over. Where the car works its own turn signal (turnpilot/car_signals.py,
a verified Tesla), the lamp goes on with the warning and the request waits, past the countdown, until it has been lit for
the legal distance (litM >= leadM, IC 9-21-8-25), and the driver's lamp the other way or hazards (the port's block) are
the driver declining the change: aborted noSignal. The window opens early enough for that lead and the change itself, so
the car is in its lane by the window's end (lane_change_window's room_s), and between two changes the same way the lamp
stays lit (signal_side), the second's lead its whole run, but no more than LC_UNSURE_MAX_S past the cooldown with no frame
passing change_blocker that way by its own read. Who signals is latched per offer: one made with the car signalling
keeps the car's path unless the car's signal fails (then the driver's path, prompted); one made with the driver
signalling keeps it to its end. A change the driver makes alone gets the same cooldown before the next
offer, for the lines to settle. TurnPilotLanePaintActive off plans no change at all; turnpilotd then publishes what this
would have asked for in laneChangePaintRule.

Under TurnPilotAlwaysDrive (ALWAYS_DRIVE.md step S4; LanePlanner.always_drive, handed over every frame), Kellen's blind spot
rule. A blind spot that reads occupied once a change is offered holds it instead of aborting it (LC-01): the offer stays up
(laneChangeHold blindspot, its line silent), any request is withdrawn, so DesireHelper, which starts a requested change the
instant the blind spot clears, never goes on its own, and neither LC_REQUEST_TIMEOUT_S nor LC_SIGNAL_TIMEOUT_S runs while it
reads occupied (LC-02). Once it has read clear LC_HOLD_CLEAR_S the change is said again, with the lane change chime, and
counted down LC_RELEASE_S (laneChangeHold clear) before the request: the driver hears it when a held change goes. Occupied
again inside the countdown, it is held again with nothing sent. With the driver signalling, the stalk let go while only the
blind spot holds the change asks for it again (WAIT) instead of aborting noSignal. The hold lasts to the window's end
(tooLate, then LC-03) or anything else's abort: the line, the lane, a car TurnPilot's cameras see in it (R13), the route, the
speed. Before an offer a blind spot still keeps it back. Nothing tells a blind spot signal stuck at SNA from a car beside it
(opendbc's Tesla carstate reads DAS_blindSpotRearLeft != 0, LC-04), so SNA holds every change to its window's end and LC-03
skips the turn, said each time; a stale DAS_status fails carState's CAN check (inputs_valid): no change. Saying 'Blind Spot
Monitor Unavailable' once waits on opendbc carrying SNA (ALWAYS_DRIVE.md S7). The owner's yes to 4.10: toward the curb only,
a change may go on less than paint (curb_lane_unpainted): the line that way read short of broken white but not solid, yellow
or no line, on no frame of the window solid or yellow, the model's next line out a lane's width beyond it with a trusted
edge, the lane not parked, no car in it and the blind spot clear, the countdown said first as for any change. Only on a
frame whose paint the reader read: turnpilotd hands over none unless the detector's lanePaintStatus is read (plan_paint),
and the planner looks before the yellow hold, which gives a frame with none read four lines, the held one yellow and the
rest unknown (YellowHold.apply). An offer of it waits on a frame with none read, and on one read short of it with nothing
contrary, nothing requested (_follow_up), so it goes only on paint read. Toward the far side the paint rule stands
(lane_agreed's included), and so does a far-side keep's (keep_blocker).

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

A change the route still wants at the window's end (LC-03, ALWAYS_DRIVE.md 4.11): whatever held it (the blind spot, a car in
the target lane, the car too slow for DesireHelper and so never offered one, no signal, DesireHelper waiting too long, the
lane parked, the line solid), with the car placed in a lane the maneuver doesn't leave from, the turn isn't pulsed from that
lane across the next one: LaneDecision.blocked, and turnpilotd skips the maneuver out loud as for an unreachable lane
(skipReason laneBlocked: "Can't Reach Turn Lane", navd reroutes), unless its desire has gone out. Judged once a maneuver, on
the first frame at or past lane_change_window's end with nothing offered or under way, and held for it. Placed: by the map's
lane list (every lane the car may be in is one the maneuver doesn't leave from, lane_unconfirmed false); without one, by a
frame of the window on which the model read both ego lines and change_blocker found a travel lane that way (since the car's
last lane change: the lane it changed from was no evidence about the one it is in). A window that only ever read the line
solid or the lane parked doesn't place it: a bike lane behind solid white, a parking strip; and a lane found by the room
beyond the line alone (no line seen beyond it) places nothing once a stopped car is seen in it on any frame of the window,
the room between parked cars a parking strip's (2026-11-07: 1e 722-737 s, replayed through the planner before it, that
right skipped). The one exception is noLane at the
window's end (the model sees no lane that way, or one only beyond a yellow line): the lane list may be wrong, or the car is
at the edge already, and the turn goes as before. The bay rule's turn wants none there (desired_changes), so is never blocked
for a bay that hasn't opened. That makes the bay's latch (BayLatch) the permissive answer: one shut by a lane divider misread
yellow would pulse the turn from the curb lane across the through lanes, so it is judged on the line's own reads, at a higher
bar, and a white read of the line near them contests it.

At night (drive 1e, 2026-11-06, after dark): 0 offers on 16,798 plan frames, every change of the drive the driver's own;
waits lineNotBroken 777 frames, noLane 314, pastWindow 3293. The model's next line out is what goes unseen in the dark
(lanes_toward), a dashed line's colour drops out between dashes (WhiteHold), and an offer died on one such frame
(_follow_up). The yellow rail is as it was: the yellow hold is applied before the white, an ego line read or held yellow is
never crossed (and aborts an offer at once), the room beyond broken white counts one lane, never one past the next line, and
a request, an offer or LC-03's placement goes only on a frame whose own read is broken white, never on the white hold's.

Night, the second pass (2026-11-07, Kellen: "we wanna get over to the right before a right turn and the left before a
left"; no give-ups for lack of a read; a safety driver signals every change, the car's lamps beyond it). The working tree's
TurnPilot.plan replayed open loop over 1d and 1e, the lane planner's inputs recorded and run through 2bd8bc25c's planner and
this one, a driver answering each prompt 1 s on: 2bd8bc25c's offered 4 changes, none on an engaged frame (1e segs 12-13 and 1d seg 4,
driven by hand), and none for the turns the driver changed lanes for himself; this offers 3 on engaged frames, each at a
left's turn bay as it opened: 1d seg 9's at 96.8 m (requested at 84.3 m, the driver's own blinker on from 106 m and his
change at 81 m), 1e seg 5's at 226.6 m (requested from 215.6 m, then held by a car stopped in the bay), 1d seg 8's at 63 m
(3.2 m/s in a queue, aborted tooSlow). What held them: the lane width the next line out was measured against
(model_lanes), the centre line's yellow hold across the bay opening (YellowHold's yellow moving out), the bay's line read
but not made out (lane_agreed), and every frame short of a pass starting the confirm count over (soft_block). Where nothing
changed: the curb-side rights, the car at the curb on all of them (the right edge 0.1-0.9 m beyond a line the model is
unsure of: no lane); a sharp left's lane beyond a line read solid with its traffic oncoming (1e 364-367 s, held by the rail,
the driver crossing it himself); a left whose line read yellow on 115 of 314 frames (1e 589-614 s, the yellow rail).

Its review (2026-11-07) found three ways that pass agreed a lane over a centre line: a frame nobody read, padded to four
lines by the yellow hold, agreed by a held yellow beyond or the list (lane_agreed now only on paint read); the far side's
list over a line never read white (now only over one cleared, LineMarks); and one white read clearing a mark and, with one
yellow read beyond, ending the hold (1e 243.35-244.85 s; now YELLOW_OUT_FRAMES and WHITE_CLEAR_FRAMES frames). The replays
of 1d and 1e offer and request the same changes at the same distances as before them; 5 frames a change left may go by
the frame's own read are gone from the two drives (the four of 1e 244.38-244.79 s, and 1d 576.25 s, the bay's yellow beyond
read once, a frame before its tenth frame of evidence)."""
import math
from dataclasses import dataclass, field, replace
from enum import IntEnum
from typing import NamedTuple

from openpilot.cereal import custom, log
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.desire_helper import NAV_LANE_CHANGE_SPEED_MIN
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
# a line read white at BROKEN_WHITE_CONF_MIN keeps its white across later reads of unknown colour until this long or this far
# after it, whichever comes first (WhiteHold; the yellow hold lasts until both, being the cautious answer)
WHITE_HOLD_S, WHITE_HOLD_M = 3., 50.
STOPPED_V = 2.               # m/s: an object this slow in the target lane, up to STOPPED_X_M ahead, makes it no travel lane
STOPPED_X_M = 80.

LC_START_S, LC_START_MIN_M, LC_START_MAX_M = 20., 150., 400.  # lane changes start this far before the maneuver...
LC_END_S, LC_END_MIN_M, LC_END_MAX_M = 6., 60., 120.          # ...and none start closer than this (and, with the offer's
                                                              # room, the window has the car in the lane by then)
LC_EXTRA_LANE_S = 8.        # each change past the first starts this much earlier (an exit across a highway)
LC_SPEED_MARGIN = 0.5       # m/s above DesireHelper's minimum, so a request never waits below it
LC_CONFIRM_FRAMES = 10      # the same need this many frames in a row before the warning
LC_WARN_S = 3.              # the warning (chime, countdown) comes this long before the change may go
LC_SIGNAL_TIMEOUT_S = 8.    # after the countdown, the driver has this long to signal before the change is aborted
LC_COOLDOWN_S = 3.          # after any lane change, ours or the driver's, for the model's lane lines to settle
LC_ABORT_COOLDOWN_S = 6.    # after an abort, before the same change is offered again
LC_REQUEST_TIMEOUT_S = 8.   # DesireHelper waiting this long (blindspot) aborts the request
LC_UNSURE_MAX_S = 3.        # an offer up this long with no frame since read broken white by its own read aborts (_follow_up);
                            # the lamp carried between two changes goes out this long past the cooldown without one (signal_side)
# under TurnPilotAlwaysDrive a blind spot holding an offered change reads clear this long, then the change is said again (its
# chime) and counted down this long before the request (LC-01, module docstring)
LC_HOLD_CLEAR_S, LC_RELEASE_S = 0.5, 1.5
# an abort once the request is out waits this long, the request withdrawn, to see whether DesireHelper started the change
# already: modeld runs it on the plan it last had, and turnpilotd sees its state a modelV2 or two later
LC_ABORT_SETTLE_S = 0.2
LC_BLOCK_X_M = 15.          # a vehicle in the target lane within this, ahead or behind, blocks a request...
LC_CLOSING_T_S = 3.         # ...as does one that closes to that distance within this
LC_LANE_MARGIN_M = 0.5
# the bay rule's latch (BayLatch): the far-side ego line read yellow at this confidence (for LC-03 yellow is the permissive
# answer there, so a higher bar than YELLOW_CONF_MIN's) this many times, none of them this long after a white read of it
BAY_YELLOW_CONF_MIN, BAY_YELLOW_READS, BAY_CONTEST_S = 0.5, LC_CONFIRM_FRAMES, 10.
# frames of reads, not one, for the yellow moving out a lane (YellowHold: the ego line read white and the line beyond it read
# yellow, the two together, since the ego line's last yellow or solid read) and for a line's mark to clear (LineMarks: white
# reads since it). One of each did both before (2026-11-07 review): 1e 243.35-244.85 s, an ego left line read yellow 3.3 s
# earlier, then unknown kind white 0.61 on 2 frames and the far-left line yellow on 3, crossable on 4 frames until it read
# solid. The logged bays: 1d 575.5-576.3 s, 8 white frames then the yellow beyond (the tenth frame at 576.30 s); 1e
# 306.82 s, 1 white frame with the yellow beyond read on 33 frames since the ego line's last yellow. Between two marks of an
# ego line on 1d and 1e, moving, its white frames were 10 or more on 16 stretches, and 1-9 on three: those two bays and
# 1e 601.2 s (7, the 589-614 s left whose line read yellow on 115 of 314 frames)
YELLOW_OUT_FRAMES = WHITE_CLEAR_FRAMES = LC_CONFIRM_FRAMES
# a map stop in the window (module docstring)
LC_SPEED_MIN = NAV_LANE_CHANGE_SPEED_MIN + LC_SPEED_MARGIN  # m/s: the change's speed
STOP_CURVE_DECEL = 1.5      # m/s^2: map_controls.MAP_STOP_DECEL, the braking curve to a map stop...
STOP_CURVE_LAG_S = 1.       # ...after a second's lag (scene_policy.STOP_LAG_S)
LC_DH_START_S = 1.5         # s from the request to DesireHelper starting the change, blind spot clear
LC_OFFER_ROOM_S = LC_WARN_S + LC_DH_START_S  # s at speed an offer needs before the slow zone: the warning and the start
LC_SPARE_S = 2.
LC_CHANGE_S = 5.            # s a change takes once DesireHelper starts it (the model moving the car over, its end)
LC_PRE_LEAD_S = LC_CONFIRM_FRAMES * DT_MDL + LC_OFFER_ROOM_S + LC_SPARE_S  # s: the window before a stop opens this long
                                                                           # ahead of its slow zone: the confirming frames,
                                                                           # the offer, LC_SPARE_S spare
GO_ACCEL = 1.0              # m/s^2: the model pulls away from a stop at about this, or faster
# m from the stop to the window's end the change needs after the go: up to LC_SPEED_MIN at GO_ACCEL (49 m), then the
# confirming frames and the offer at that speed (50 m)
AFTER_GO_M = LC_SPEED_MIN ** 2 / (2. * GO_ACCEL) + LC_SPEED_MIN * (LC_CONFIRM_FRAMES * DT_MDL + LC_OFFER_ROOM_S)

LANE_UNKNOWN_REASON = "can't confirm lane"
HOLD_REASON = "car in blind spot"  # an offer held for it (LC-01), as the wait before one
LINE_UNSURE_REASON = "checking lane line"  # an offer waiting on a line read short of broken white, nothing contrary
AFTER_STOP_REASON = "lane change after the stop"
UNREACHABLE_REASON = "can't reach the turn lane before the stop"

SIDE_DIRECTIONS = {LEFT: ("left", "slightLeft"), RIGHT: ("right", "slightRight")}
UNJUDGED = object()  # LanePlanner's LC-03 verdict for no maneuver yet (maneuver ids may be None)


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
  # room for a lane that way with its next line out unseen: a trusted road edge LANE_MIN lane widths or more beyond the ego
  # line, and no line the model is sure of beyond it (one it sees nearer than a lane is a strip's: noLane). Not a lane by
  # itself (room alone counts shoulders and parking strips), only with the ego line read broken white (lanes_toward)
  left_open: bool = field(default=False, compare=False)
  right_open: bool = field(default=False, compare=False)
  # the model sees the ego line that way, its prob over LINE_PROB_MIN: lane_agreed's "the model's lines"
  left_seen: bool = field(default=False, compare=False)
  right_seen: bool = field(default=False, compare=False)


def _side(ego_y, ego_prob, outer_y, outer_prob, edge_y, edge_std, sign, width, spacing=None):
  """(lanes counted on one side, keep ok that way, the count known, open: ModelLanes); sign is +1 for the right, -1 for the
  left (modelV2 y is right). spacing: the ego lines' own spacing the next line out is measured against (model_lanes),
  width when None."""
  room = sign * (edge_y - ego_y)
  near = LANE_MIN * min(width, width if spacing is None else spacing)
  painted = outer_prob > LINE_PROB_MIN and near <= sign * (outer_y - ego_y) <= 1.5 * width
  edge_ok = edge_std < EDGE_STD_MAX
  if not painted or (edge_ok and room < SHOULDER_LANES * width):
    lanes = 0
  else:
    lanes = max(int(math.floor(room / width + 0.25)), 1) if edge_ok else 1
  keep_ok = ego_prob > LINE_PROB_MIN and not (lanes == 0 and edge_ok and room < KEEP_MIN_ROOM_M)
  known = edge_ok and (painted or room < SHOULDER_LANES * width)
  open_ = edge_ok and room >= LANE_MIN * width and outer_prob <= LINE_PROB_MIN
  return lanes, keep_ok, known, open_


def model_lanes(line_ys, line_probs, edge_ys, edge_stds) -> ModelLanes:
  """From modelV2 laneLines [far left, left, right, far right] and roadEdges [left, right] at one distance ahead.

  The next line out counts a lane (painted) from LANE_MIN of the ego lane's width beyond the ego line, the width there the
  model's two ego lines' spacing whenever one of them is sure and the spacing is a lane's (2.5-4.8 m), if that is narrower
  than ModelLanes.width (which stays LANE_WIDTH unless both are sure, for the shoulder, the room and the lane bands: a
  narrower width there would make a 2.5 m strip to the edge no shoulder, and count more lanes in the same room; the BeamNG
  crossing, test_lane_planner's XING, the curb lane then unplaced). The model places a line it is unsure of where the lane's
  edge is all the same: drives 1d and 1e (2026-11-06, after dark), moving, the spacing with one ego line unsure had a median
  of 3.23 and 3.27 m (10th-90th 2.9-3.6, 2.9-4.3), with both sure 3.09 and 3.27 m, and it lay in 2.5-4.8 m on 99% and 92% of
  the frames. Before, both had to be sure, and at night the curb side's ego line is unsure on most frames (its prob a median
  0.08 moving, 1e: the car at the curb, no line painted there), so the bar was LANE_WIDTH's 3.6 m, wider than nine in ten of
  the lanes measured, and a lane beside the car needed its next line out 3.06 m away: a city lane the model put 2.6-2.8 m out
  was no lane. 1d 575.9-577.4 s, a left's bay opened (the far-left line read yellow at 0.39-1.0, 2.6-2.8 m beyond the ego
  line read broken white, the lane 2.7-3.0 m wide), counted none (noLane), and the driver signalled and changed lanes
  himself 104 m out; 1e 364-367 s, the far-left line 2.8 m out at p 0.5-0.72 the same."""
  measured = line_ys[2] - line_ys[1]
  lane = 2.5 <= measured <= 4.8
  width = measured if min(line_probs[1], line_probs[2]) > LINE_PROB_MIN and lane else LANE_WIDTH
  spacing = measured if max(line_probs[1], line_probs[2]) > LINE_PROB_MIN and lane else LANE_WIDTH
  left, keep_left, left_known, left_open = _side(line_ys[1], line_probs[1], line_ys[0], line_probs[0], edge_ys[0], edge_stds[0], -1.,
                                                 width, spacing)
  right, keep_right, right_known, right_open = _side(line_ys[2], line_probs[2], line_ys[3], line_probs[3], edge_ys[1], edge_stds[1],
                                                     1., width, spacing)
  measured = min(line_probs[1], line_probs[2]) > LINE_PROB_MIN
  return ModelLanes(left, right, keep_left, keep_right, width, measured, left_known, right_known, float(line_ys[1]), float(line_ys[2]),
                    left_open, right_open, line_probs[1] > LINE_PROB_MIN, line_probs[2] > LINE_PROB_MIN)


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


def _marked_lanes(maneuver, nav) -> list[int]:
  """The lanes of the lane list nav the list itself says the maneuver leaves from: the active ones, else those marked for its
  side; none when it marks neither (an incomplete OSM turn:lanes)."""
  return ([i for i, (_, active) in enumerate(nav) if active] or
          [i for i, (dirs, _) in enumerate(nav) if any(d in SIDE_DIRECTIONS[maneuver.side] for d in dirs)])


def _valid_lanes(maneuver, nav) -> list[int]:
  """_marked_lanes, or with none marked the outer lane on the maneuver's side: a guess, never the bay rule's (far_bay)."""
  return _marked_lanes(maneuver, nav) or [0 if maneuver.side == LEFT else len(nav) - 1]


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
                  right_hand_traffic: bool = True, paint=(), bay_shut: bool | None = None):
  """The car is in a turn-only lane where the route goes straight through, or not in a lane that turns where the route
  turns, closer than the lane planner still changes lanes (lane_change_window's end). Only flagged, never acted on:
  LaneMismatch.none without the map's lane list, without the model reading the ego lane's lines or counting the lanes on
  the curb side.

  Never wrongLaneForTurn while the bay rule holds (_bay_waits, the same judgement desired_changes makes; paint and bay_shut
  as for it, and turnpilotd hands over LanePlanner's latch, bay.shut, as the planner's own desired_changes goes by it: at
  night the centre line's own read is unknown on 41% of frames, 1e): the car beside the centre line is then in the far-most
  lane of its direction, the one the turn bay opens from, and TurnPilot drives the turn from it. Counted from the curb by
  the list it reads one or two lanes short of the bay, so before this 'Wrong Lane: This lane doesn't turn with the route'
  was said inside the window's end on every such approach while the plan wanted no change (2026-11-06 review: [left
  (active), straight, straight], one lane, 50 m out, the left line yellow). turnOnlyGoingStraight is as before: the bay
  rule is a turn's."""
  if not lanes.measured:
    return LaneMismatch.none
  near = lane_change_window(v_ego)[1]
  turn = maneuver is not None and maneuver.kind == TURN
  if len(through_lanes) >= 2 and 0. < through_distance < near and not (turn and distance < through_distance):
    idx = ego_lane_index(len(through_lanes), lanes, right_hand_traffic)
    directions, active = through_lanes[idx] if idx is not None else ((), True)
    if not active and directions and "straight" not in directions:
      return LaneMismatch.turnOnlyGoingStraight
  if turn and len(maneuver.lanes) >= 2 and 0. < distance < near and not _bay_waits(maneuver, right_hand_traffic, paint, bay_shut):
    idx = ego_lane_index(len(maneuver.lanes), lanes, right_hand_traffic)
    directions, active = maneuver.lanes[idx] if idx is not None else ((), True)
    if not active and not any(d in SIDE_DIRECTIONS[maneuver.side] for d in directions):
      return LaneMismatch.wrongLaneForTurn
  return LaneMismatch.none


def _yellow(p: LinePaint) -> bool:
  return p.kind != 'none' and p.color == 'yellow' and p.confidence >= YELLOW_CONF_MIN


def _white(p: LinePaint) -> bool:
  return p.kind != 'none' and p.color == 'white' and p.confidence >= BROKEN_WHITE_CONF_MIN


def _bay_yellow(p: LinePaint) -> bool:
  return p.kind != 'none' and p.color == 'yellow' and p.confidence >= BAY_YELLOW_CONF_MIN


def line_contrary(paint, side) -> bool:
  """The ego lane's line toward side (LEFT or RIGHT) reads against a lane going our way beyond it: solid (an edge, a bike or
  parking lane, a change discouraged at a junction), yellow at any confidence (oncoming traffic; held yellow included, so
  paint as for change_blocker), or no line. Kind or colour unknown, or broken white short of BROKEN_WHITE_CONF_MIN, is no
  contrary read: nothing says the line changed, only that this frame didn't confirm it (LanePlanner._follow_up). No paint
  read at all is the caller's to judge."""
  if len(paint) != 4 or side not in (LEFT, RIGHT):
    return False
  p = paint[1 if side == LEFT else 2]
  return p.kind in ('solid', 'none') or p.color == 'yellow'


LATCHED_YELLOW = LinePaint('unknown', 'yellow', 1., 0.)


def paint_lanes(lanes: ModelLanes, paint) -> ModelLanes:
  """The model's lane counts with every lane beyond a yellow line taken out. paint: one LinePaint per modelV2 lane line,
  [far left, left, right, far right], or empty when it wasn't read."""
  if len(paint) != 4:
    return lanes
  def side(n, ego, outer):
    return 0 if _yellow(ego) else min(n, 1) if _yellow(outer) else n
  return replace(lanes, left=side(lanes.left, paint[1], paint[0]), right=side(lanes.right, paint[2], paint[3]))


KEEP_NO_LINE = 'keepNoLine'  # TurnpilotPlan.skipReason: a keep the model sees no lane line for (ModelLanes.keep_*_ok)
KEEP_FAR_SIDE = 'keepFarSide'  # ...one toward the far side with no lane of ours beyond the ego line read broken white


def keep_blocker(lanes: ModelLanes, direction: int, right_hand_traffic: bool = True, paint=()) -> str:
  """Why a keep desire toward direction (LEFT or RIGHT) may not go now, '' when it may (turnpilotd starts one only when it
  may, and says a keep that couldn't go by keep_last: BUILD_FINDINGS D2-keep):
  - KEEP_NO_LINE: the model sees no lane line and road that way (ModelLanes.keep_*_ok: a keep with no line drove the car off
    the road);
  - KEEP_FAR_SIDE: toward the far side (left where traffic keeps right), no lane counted beyond the ego line once the lane
    paint has taken out every lane beyond a yellow line (paint_lanes; paint as for it, held yellow included: YellowHold.apply).
    The room beyond the ego line is no lane there: on an undivided road it is the oncoming lane, the far road edge beyond it
    (BUILD_FINDINGS D5, BeamNG S19: keepLeft at 19 m/s on a two-way road, 2.5 m over the centre line). So a far-side keep
    needs the next line out painted a lane width on (a lane going our way) and positive evidence of it: the ego line read
    broken white this frame (broken_white_toward, never held yellow). The model's count alone is no lane of ours: an
    undivided road whose oncoming lane has its edge line painted a lane out counts a lane there, so with the paint unread
    (night, worn paint, a frame the reader skips; it reads only the road camera) or the ego line's colour unknown the keep
    is refused, said and rerouted (skeptic SIMFIX D5: an uncapped keepLeft over the centre line at highway-to-town speed;
    the wrong-side handback comes only after WRONG_SIDE_S of reads, past the crossing). More skips at night: the safe
    side. Toward the curb side the room is the road's own (a shoulder, an exit's lane), as before."""
  ok = lanes.keep_left_ok if direction == LEFT else lanes.keep_right_ok if direction == RIGHT else False
  if not ok:
    return KEEP_NO_LINE
  if direction == (LEFT if right_hand_traffic else RIGHT):
    counted = paint_lanes(lanes, paint)
    if (counted.left if direction == LEFT else counted.right) == 0 or not broken_white_toward(paint, direction):
      return KEEP_FAR_SIDE
  return ''


def broken_white_toward(paint, side) -> bool:
  """The ego lane's line toward side (LEFT or RIGHT) reads broken white: a lane going the same way is beyond it."""
  if len(paint) != 4 or side not in (LEFT, RIGHT):
    return False
  return paint[1 if side == LEFT else 2].is_('broken', 'white', BROKEN_WHITE_CONF_MIN)


def lanes_toward(direction: int, lanes: ModelLanes, paint=()) -> int:
  """Travel lanes beyond the ego line toward direction (LEFT or RIGHT) for a lane change (change_blocker): the model's count
  with every lane beyond a yellow line taken out (paint_lanes), or one where that is none but the ego line that way reads
  broken white in paint and the model leaves a lane's room to a trusted road edge with its next line out unseen
  (ModelLanes.left_open, right_open). On the white hold's paint (WhiteHold) that room keeps an offer's confirm count and an
  offer up at night, nothing more: LanePlanner asks again on the frame's own read (_held_only), and a request, an offer or
  LC-03's placement needs the lane there too. Broken white is a lane going our way beyond it (MUTCD), and the room
  says it is a lane's width; the line beyond it is what the model rarely sees at night. Drive 1e (2026-11-06, after dark):
  the far-left line was above LINE_PROB_MIN on 17% of moving frames and the far-right on none, so the count by painted
  lines put no lane anywhere and no change was offered all night (noLane 314 frames). On the 1e seg 8-9 approach (a two-lane
  one-way) the left ego line read broken white on 474 frames of 518-569 s; the model counted the lane on 80 of them, this
  on 222 more, the edge a median 5.5 m out (the rest: the edge unsure, 153, or a line seen 0.79-0.85 lane widths out, 19).
  Never across a yellow: the ego line read or held yellow isn't broken white, and the room gives one lane at most, the one
  before the next line out whatever that is, as paint_lanes gives one before a line read yellow (a one-way road's left edge
  line is yellow: 1e seg 8 534.7 s, the far-left line read solid yellow with the model's count flicking 1/0 under it). Not
  for keeps: keep_blocker's far-side rule wants the next line out painted."""
  counted = paint_lanes(lanes, paint)
  n = counted.left if direction == LEFT else counted.right if direction == RIGHT else 0
  if n or not broken_white_toward(paint, direction):
    return n
  return int(lanes.left_open if direction == LEFT else lanes.right_open)


def lane_agreed(direction: int, lanes: ModelLanes, paint, nav_room: int | None = None, right_hand_traffic: bool = True,
                cleared: bool = False) -> bool:
  """A lane change toward direction (LEFT or RIGHT) across an ego line the reader read but couldn't make out (its kind unknown,
  or broken white under BROKEN_WHITE_CONF_MIN; its colour white or unknown), where the rest agrees a lane of ours is beyond it
  (Kellen, 2026-11-07: "a line the reader can't read is not a reason to stay put when the model's lane lines, road edges and
  the map's lane count agree a lane is there"). Paint as for change_blocker (the yellow hold applied), four lines read; never
  across a contrary read (line_contrary: solid, yellow at any confidence read or held, no line). All of:
  - the model sees the ego line that way (ModelLanes.left_seen, right_seen) and counts a painted lane beyond it: its next line
    out LANE_MIN to 1.5 lane widths on, a trusted road edge (the count known), after paint_lanes takes out every lane beyond a
    yellow line. Never by room to the edge alone (left_open, right_open): beside a divided road's yellow left edge misread,
    the room beyond is the median, and room is what a shoulder or a parking strip leaves too;
  - and one sign the lane goes our way: the ego line read white at BROKEN_WHITE_CONF_MIN on this frame's own read (white is no
    centre line; kind unknown is a dash's gaps unresolved, the usual night read); or, toward the far side, the next line out
    read or held yellow (the centre line beyond the lane, as a turn bay opens: paint_lanes counts one lane before it); or the
    map's lane list has a lane of ours that way from wherever the car may be in it (nav_room >= 1, None without a list),
    toward the far side only over an ego line cleared (cleared: LineMarks, read white WHITE_CLEAR_FRAMES times since its last
    yellow or solid read, or the yellow moved out a lane from it). Toward the far side the model counts the oncoming lane
    beyond a centre line as readily as a lane of ours, and on a far_bay approach the list is the junction's (desired_changes'
    bay rule), so there the list alone is no sign over a line whose colour was never read white.
  The planner asks this only with that way's ego line unmarked (LineMarks, LanePlanner._unread_ok: no yellow or solid read
  of it since the car's last lane change, unless WHITE_CLEAR_FRAMES white reads of it or the yellow moving out came after),
  so a centre line read yellow once is never crossed on unread frames after the yellow hold runs out; and only on a frame
  whose paint the reader read (turnpilotd's plan_paint hands over none unless it did, and the planner asks on four lines as
  they came in, before the yellow hold pads a frame nobody read to four: 2026-11-07 review, a far-left line read yellow once
  and every frame after it unread was agreed by its hold, offered and requested), so a frame nobody read is no agreement.

  Drive 1d (2026-11-06, after dark) 575.5-577.7 s: a left's bay opened 127 m out, its lane list [left, straight] placing the
  car one lane right of it; the ego left line read white 0.58-0.71 (kind unknown, 3 frames broken), then broken of unknown
  colour, then unknown at 0 from 576.4 s, the far-left line read yellow 0.39-1.0 2.6-2.8 m out, and the driver signalled at
  577.7 s and changed lanes himself, the planner waiting lineNotBroken and noLane throughout. On both drives the ego lines
  read kind unknown on 41% (left, 1e) and 90% (right) of moving frames."""
  if len(paint) != 4 or direction not in (LEFT, RIGHT) or line_contrary(paint, direction) or (nav_room is not None and nav_room <= 0):
    return False  # (the list with no lane of ours that way: no agreement, whatever else reads)
  ego, outer = (paint[1], paint[0]) if direction == LEFT else (paint[2], paint[3])
  counted = paint_lanes(lanes, paint)
  seen, n, known = (lanes.left_seen, counted.left, lanes.left_known) if direction == LEFT else \
    (lanes.right_seen, counted.right, lanes.right_known)
  if not (seen and n >= 1 and known):
    return False
  far = direction == (LEFT if right_hand_traffic else RIGHT)
  return (ego.color == 'white' and ego.confidence >= BROKEN_WHITE_CONF_MIN) or (far and _yellow(outer)) or \
    (nav_room is not None and nav_room >= 1 and (cleared or not far))


def oncoming_in_lane(objects, direction: int, width: float = LANE_WIDTH) -> bool:
  """Something coming toward the car, faster than STOPPED_V over ground, up to STOPPED_X_M ahead in the lane one over toward
  direction (LEFT or RIGHT): that lane is an oncoming one, whatever its line reads. objects: as for stopped_in_lane. Drive 1e
  (2026-11-06) 364.2-367.4 s, a sharp left's approach: cars at 2.7-3.1 m left closing at 3-6 m/s over ground, 7-25 m ahead,
  beyond an ego line read solid of unknown colour; target_lane_blocked counts only one within LC_CLOSING_T_S of the car."""
  return any(0. < obj.x < STOPPED_X_M and obj.vx < -STOPPED_V and 0.5 * width < obj.y * direction < 1.5 * width + LC_LANE_MARGIN_M
             for obj in objects)


def far_bay(maneuver, right_hand_traffic: bool = True) -> bool:
  """A turn toward the far side (left where traffic keeps right) whose lane list has the lanes it leaves from all on the far
  side of the others ([left (active), straight, straight]). The list describes the junction, its turn bay open; on the
  approach before the bay opens the car beside the yellow centre line is already in the far-most lane of its direction, and
  counted from the curb by the list it reads one or two lanes short of the bay: desired_changes' bay rule.

  Only the lanes the list itself marks (_marked_lanes: active, else marked for the turn's side), never _valid_lanes' guess of
  the outer lane for a list that marks none (2026-11-06 review): the bay rule is the permissive answer for LC-03 (no change
  wanted, never skipped for its lane) and that guess is no evidence of a bay. An unmarked list (every lane's directions
  empty or none, none active: an incomplete OSM turn:lanes) goes by the normal rule, as before the bay rule. Every list
  logged on drives 1d and 1e (2026-11-06) marks its turn's lanes, so this changes none of their approaches."""
  nav = _nav_lanes(maneuver)
  far = LEFT if right_hand_traffic else RIGHT
  if not nav or maneuver.kind != TURN or maneuver.side != far:
    return False
  valid = _marked_lanes(maneuver, nav)
  rest = [i for i in range(len(nav)) if i not in valid]
  return bool(valid) and bool(rest) and (max(valid) < min(rest) if far == LEFT else min(valid) > max(rest))


def _bay_waits(maneuver, right_hand_traffic: bool = True, paint=(), bay_shut: bool | None = None) -> bool:
  """The bay rule holds, one judgement for desired_changes (no change wanted) and lane_mismatch (no wrong lane said): a
  far_bay turn whose bay hasn't opened, by bay_shut (LanePlanner's latch, BayLatch.shut) when given, else the far-side ego
  line read or held yellow at BAY_YELLOW_CONF_MIN in paint as given."""
  if bay_shut is None:
    bay_shut = len(paint) == 4 and _bay_yellow(paint[1 if right_hand_traffic else 2])
  return bool(bay_shut) and far_bay(maneuver, right_hand_traffic)


def stopped_in_lane(objects, direction: int, width: float = LANE_WIDTH) -> bool:
  """Something stopped up to STOPPED_X_M ahead in the lane one over toward direction (LEFT or RIGHT): a parked car says
  that lane is a parking strip. objects: TurnpilotScene.TrackedObject-like, y positive left, vx over ground."""
  return any(0. < obj.x < STOPPED_X_M and abs(obj.vx) < STOPPED_V and 0.5 * width < obj.y * direction < 1.5 * width + LC_LANE_MARGIN_M
             for obj in objects)


def _toward(direction: int):
  return LaneChange.left if direction == LEFT else LaneChange.right if direction == RIGHT else LaneChange.none


def _direction(lc) -> int:
  return LEFT if lc == LaneChange.left else RIGHT if lc == LaneChange.right else 0


def desired_changes(maneuver, lanes: ModelLanes, right_hand_traffic: bool = True, paint=(), done: int | None = 0, start=None,
                    bay_shut: bool | None = None) -> int:
  """Lane changes the route wants before the maneuver, positive left, negative right, before change_blocker has a say.
  paint: as for paint_lanes; done, start: as for nav_lane_span. With a lane list, the fewest changes any lane the car may
  be in needs, and none when it may already be in one the maneuver leaves from.

  The bay rule: none for a far-side turn whose lanes are the list's far-most (far_bay) while its turn bay hasn't opened,
  the far-side ego line read or held yellow at BAY_YELLOW_CONF_MIN (paint's own, as given) or, from LanePlanner, bay_shut
  (its latch, BayLatch: from uncontested yellow reads until the line reads white; _bay_waits, which lane_mismatch shares so
  no 'Wrong Lane' is said where none is wanted). The car beside the centre line is in the
  far-most lane of its direction, and the turn desire takes the bay when it opens. Drive 1e (2026-11-06) seg 5: a 29 deg
  left with lanes [left (active), straight, straight] on a one-lane approach, counted from the curb as the list's curb lane,
  wanted +2 the whole approach against the yellow line (lineNotBroken), and as a turn LC-03 would have skipped it "Can't
  Reach Turn Lane"."""
  if maneuver is None or not (maneuver.kind == TURN or (maneuver.kind == KEEP and maneuver.type == "off ramp")):
    return 0
  lanes = paint_lanes(lanes, paint)
  curb = RIGHT if right_hand_traffic else LEFT
  nav = _nav_lanes(maneuver)
  if nav:
    valid = _valid_lanes(maneuver, nav)
    lo, hi = nav_lane_span(len(nav), lanes, right_hand_traffic, done, start)
    want = lo - max(valid) if lo > max(valid) else hi - min(valid) if hi < min(valid) else 0
    if want and _bay_waits(maneuver, right_hand_traffic, paint, bay_shut):
      return 0  # the bay rule: the bay not open yet
    return want
  if maneuver.side == curb and maneuver.kind == KEEP:  # an exit: over to the curb lane by the model's count
    n = lanes.left if curb == LEFT else lanes.right
    if len(paint) == 4:  # not across a curb-side line read solid white: a shoulder or a bike lane is beyond it
      ego, outer = (paint[1], paint[0]) if curb == LEFT else (paint[2], paint[3])
      n = 0 if ego.is_('solid', 'white', SOLID_WHITE_CONF_MIN) else min(n, 1) if outer.is_('solid', 'white', SOLID_WHITE_CONF_MIN) else n
    return n if curb == LEFT else -n
  return 1 if maneuver.side == LEFT else -1  # a turn, or an exit away from the curb: one toward its side at a time


def change_blocker(direction: int, lanes: ModelLanes, paint=(), objects=(), right_hand_traffic: bool = True,
                   nav_room: int | None = None, unpainted_curb: bool = False, unread_ok: bool = False,
                   line_cleared: bool = False) -> str:
  """'' when one lane change toward direction (LEFT or RIGHT) is into a valid travel lane, else the abort reason why not
  (a TurnpilotPlan.LaneChangeAbort name). paint: as for paint_lanes, held yellow included; objects: as for stopped_in_lane;
  nav_room: as nav_room() returns it, None without the map's lane list; unpainted_curb: a curb-side change may go on less
  than paint (curb_lane_unpainted; LanePlanner, under TurnPilotAlwaysDrive); unread_ok: a line the reader couldn't make out
  may be crossed where the rest agrees a lane is there (lane_agreed; LanePlanner, that way's line unmarked and this frame's
  paint read); line_cleared: that way's ego line cleared (lane_agreed's cleared, LineMarks). Anything coming toward the car
  in that lane makes it no lane of ours (oncoming_in_lane: noLane), on every path."""
  if direction not in (LEFT, RIGHT):
    return 'noLane'
  if not right_hand_traffic and direction == RIGHT:
    return 'noLane'  # away from the curb in left-hand traffic: the center line is broken white there
  if not broken_white_toward(paint, direction) and not (unpainted_curb and curb_lane_unpainted(direction, lanes, paint, right_hand_traffic)) \
     and not (unread_ok and lane_agreed(direction, lanes, paint, nav_room, right_hand_traffic, line_cleared)):
    return 'lineNotBroken'
  if lanes_toward(direction, lanes, paint) == 0:
    return 'noLane'  # the model sees no lane that way, nor room for one beyond broken white, or only beyond a yellow line
  if nav_room is not None and nav_room <= 0:
    return 'noLane'  # the map's lane list has no lane that way: the model's are oncoming, or not where it can place the car
  if oncoming_in_lane(objects, direction, lanes.width):
    return 'noLane'  # an oncoming lane, whatever its line reads
  if stopped_in_lane(objects, direction, lanes.width):
    return 'laneParked'
  return ''


def soft_block(direction: int, blocker: str, lanes: ModelLanes, paint, objects=(), nav_room: int | None = None,
               right_hand_traffic: bool = True) -> bool:
  """change_blocker's blocker toward direction is the want of a read or of the model's count, nothing read against the lane:
  lineNotBroken with no contrary read of the ego line that way (line_contrary: solid, yellow read or held, no line), a frame
  nobody read included; or noLane by the model's count alone with the count not known (its next line out under
  LINE_PROB_MIN this frame with room beyond, the edge's std over EDGE_STD_MAX: ModelLanes.left_known, right_known), never a
  road edge the model is sure lies beside the line, the lane list's, an oncoming car's or the far side's in left-hand
  traffic. Such a frame keeps an offer's confirm count and an offer up, for LC_UNSURE_MAX_S past the last frame that passed,
  and nothing is offered or requested on it (LanePlanner.update, _follow_up). Before 2026-11-07 every one reset the count, and at night
  nothing reached LC_CONFIRM_FRAMES in a row: 1d 576.4-577.4 s, the far-left line's prob 0.15-0.74 around LINE_PROB_MIN,
  the lane counted on at most eight frames running, then not; 1e seg 8-9, the left edge's std over EDGE_STD_MAX once."""
  if direction not in (LEFT, RIGHT) or line_contrary(paint, direction):
    return False
  if blocker == 'lineNotBroken':
    return True
  known = lanes.left_known if direction == LEFT else lanes.right_known
  return blocker == 'noLane' and not known and (right_hand_traffic or direction == LEFT) and \
    not (nav_room is not None and nav_room <= 0) and not oncoming_in_lane(objects, direction, lanes.width)


def curb_lane_unpainted(direction: int, lanes: ModelLanes, paint, right_hand_traffic: bool = True) -> bool:
  """A curb-side lane change on less than paint (ALWAYS_DRIVE.md 4.10, the owner's yes for the curb side only, under
  TurnPilotAlwaysDrive): the ego line toward the curb read short of broken white but never solid, yellow or no line (its kind
  unknown, or broken under BROKEN_WHITE_CONF_MIN; its colour white or unknown: held yellow is yellow), and the model's next line
  out painted 0.85-1.5 lane widths beyond it with a trusted road edge (a lane counted that way, the count known). The paint has
  to have been read: with none (no detector, the wide camera, a stop sweep, the paint off for the drive, a road frame the
  reader had no modelV2 for, a stale scene) nothing read the line. Four lines of kind and colour unknown can be paint nobody
  read, so the caller says whether this frame's was read: turnpilotd passes () unless the reader's lanePaintStatus is read
  (plan_paint), and LanePlanner asks this only on a frame whose paint came in four lines, before the yellow hold pads it.
  Never toward the far side, where the room beyond the centre line is the oncoming lane (R11), nor a far-side keep."""
  if direction != (RIGHT if right_hand_traffic else LEFT) or len(paint) != 4:
    return False
  ego = paint[1 if direction == LEFT else 2]
  if ego.kind not in ('unknown', 'broken') or ego.color == 'yellow':
    return False
  return lanes.left >= 1 and lanes.left_known if direction == LEFT else lanes.right >= 1 and lanes.right_known


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


def lane_change_window(v_ego: float, changes: int = 1, room_s: float | None = None) -> tuple[float, float]:
  """(start, end) m before the maneuver, for this many lane changes still to make. room_s: the offer's room (offer_room_s:
  the warning, or where the car signals the lamp's legal lead, and DesireHelper's start), and the window opens early enough
  for the first change to be confirmed, offered, started and made with LC_SPARE_S to spare by its end, each one after it
  LC_EXTRA_LANE_S on: the car in the lane before the end, not just under way at it. Never past LC_START_MAX_M (and the
  extra lanes), so a highway's window is as it was. The lamp's 300 ft took most of a city window (at 18 mph the offer at
  181 m, the request at 84 m, in the turn lane 50 m out), and a second change none was left for: the first drive's car
  into its turn lane only at the light. A driver's warning (LC_OFFER_ROOM_S) fits the windows as they were at every speed."""
  extra = max(changes - 1, 0) * LC_EXTRA_LANE_S * v_ego
  start = min(max(LC_START_S * v_ego, LC_START_MIN_M), LC_START_MAX_M) + extra
  end = min(max(LC_END_S * v_ego, LC_END_MIN_M), LC_END_MAX_M)
  if room_s is not None:
    made = end + v_ego * (LC_CONFIRM_FRAMES * DT_MDL + room_s + LC_CHANGE_S + LC_SPARE_S) + extra
    start = max(start, min(made, LC_START_MAX_M + extra))
  return start, end


def slow_zone(v_ego: float) -> float:
  """m before a map stop where its braking curve takes the car under LC_SPEED_MIN, approaching at v_ego."""
  return LC_SPEED_MIN ** 2 / (2. * STOP_CURVE_DECEL) + max(v_ego, LC_SPEED_MIN) * STOP_CURVE_LAG_S


def stop_window(v_ego: float, distance: float, stop_d: float | None, changes: int = 1,
                room_s: float = LC_OFFER_ROOM_S) -> tuple[float, float, str]:
  """(start, end, wait) of the lane change window for this many changes with a map stop stop_d m ahead (module docstring),
  the maneuver distance m ahead. wait: '' (offer in the window), 'afterStop' or 'unreachable'. A stop past the maneuver, or
  one before the window at the change's speed (the car is up to speed again by its start), changes nothing."""
  start, end = lane_change_window(v_ego, changes, room_s)
  if stop_d is None or not 0. <= stop_d < distance:
    return start, end, ''
  before = distance - stop_d  # m before the maneuver the stop is
  v = max(v_ego, LC_SPEED_MIN)
  if before >= lane_change_window(v, changes, room_s)[0]:  # slowing for it shrinks the window: not the window at speed
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
  YELLOW_HOLD_S and YELLOW_HOLD_M. Lane changes move the holds with the lines.

  The yellow moving out a lane ends an ego line's hold: since the ego line's last yellow (any confidence) or solid read, it
  read white at BROKEN_WHITE_CONF_MIN (any kind but none) and the next line out that way read yellow, YELLOW_OUT_FRAMES
  frames of the two together with at least one of each, the line beyond read yellow this frame and the ego line's last white
  read at most WHITE_HOLD_S and WHITE_HOLD_M before (moved_out, which clears the line's mark: LineMarks). That is a turn bay
  opening beside the centre line, or a lane added on the far side: the centre line now lies beyond the lane, a white line
  where it was. The hold is there so a worn stretch of a centre line opens no gap, and a worn stretch reads no white with a
  yellow line a lane further out. One frame of each did it before (2026-11-07 review), and 1e 243.35-244.30 s had them: a
  centre line read white 0.61 on 2 frames, the line beyond yellow on 1, released, the lane beyond the centre line crossable on
  4 frames. A pair of frames can be two misreads; ten are the two lines' reads agreeing. Drive 1d (2026-11-06, after dark), a left
  with the lane list [left, straight]: the ego left line read solid and unknown yellow to 574.0 s, white 0.67-0.71 over
  575.5-576.0 s (kind unknown, then broken), and from 576.25 s the far-left line read yellow 0.39-1.0 2.6-2.8 m beyond it;
  held to 582.5 s (100 m at 11.7 m/s), past the window's end, the bay was no lane all its length, and the driver signalled
  at 577.7 s and changed lanes himself 104 m out. 1e 305-314 s (a left, [left (active), straight, straight]) the same: the
  far-left line yellow 0.75-1.0 2.5-3.3 m out from 305 s, the ego line broken white 0.64-0.75 at 310.2-311.4 s."""
  def __init__(self):
    self.since = [(math.inf, math.inf)] * 4
    self.white = [(math.inf, math.inf)] * 4  # how long and how far since each line's last white read (_white): moved out
    self.whites = [0] * 4                    # ...frames each line read white since its last yellow or solid read
    self.outs = [0] * 4                      # ...frames the line beyond an ego line read yellow since then
    self.moved_out = [False] * 4             # this frame, the yellow moved out a lane from that ego line (LineMarks)

  def update(self, paint, v_ego: float, dt: float, frozen: bool) -> tuple:
    """paint as for paint_lanes; frozen: a lane change is under way, so the lines may be renumbering. Returns the paint with
    every held line read yellow (four entries whenever anything is held)."""
    self.since = [(t + dt, d + v_ego * dt) for t, d in self.since]
    self.white = [(t + dt, d + v_ego * dt) for t, d in self.white]
    self.moved_out = [False] * 4
    if len(paint) == 4 and not frozen:
      self.since = [(0., 0.) if _yellow(p) else hold for p, hold in zip(paint, self.since, strict=True)]
      self.white = [(0., 0.) if _white(p) else w for p, w in zip(paint, self.white, strict=True)]
      for i, p in enumerate(paint):
        if p.color == 'yellow' or p.kind == 'solid':  # (as LineMarks marks a line) the evidence starts over
          self.whites[i] = self.outs[i] = 0
        elif _white(p):
          self.whites[i] += 1
      for ego, outer in ((1, 0), (2, 3)):  # the yellow moved out a lane: the ego line's hold ends
        p = paint[ego]
        if not _yellow(paint[outer]) or p.color == 'yellow' or p.kind == 'solid':
          continue
        self.outs[ego] += 1
        wt, wd = self.white[ego]
        if self.whites[ego] and self.whites[ego] + self.outs[ego] >= YELLOW_OUT_FRAMES and wt < WHITE_HOLD_S and wd < WHITE_HOLD_M:
          self.since[ego], self.moved_out[ego] = (math.inf, math.inf), True
    return self.apply(paint)

  def apply(self, paint) -> tuple:
    """paint with every line held yellow read yellow, as update() returns it, without counting a frame: the holds as of the
    last update (turnpilotd's keep gate, before the lane planner's update this frame)."""
    held = [t < YELLOW_HOLD_S or d < YELLOW_HOLD_M for t, d in self.since]
    if not any(held):
      return tuple(paint)
    paint = tuple(paint) if len(paint) == 4 else (UNKNOWN,) * 4
    return tuple(p if _yellow(p) or not h else LATCHED_YELLOW for p, h in zip(paint, held, strict=True))

  def shift(self, direction: int) -> None:
    """The car moved one lane toward direction (LEFT or RIGHT): each line is now one index further the other way. With the
    direction unknown, any hold covers all four lines, since it could be any of them now. A line's white reads move with it;
    the yellow beyond an ego line is counted afresh (the line beyond the new one wasn't watched)."""
    none = (math.inf, math.inf)
    self.outs = [0] * 4
    if direction == LEFT:
      self.since, self.white, self.whites = [none] + self.since[:3], [none] + self.white[:3], [0] + self.whites[:3]
    elif direction == RIGHT:
      self.since, self.white, self.whites = self.since[1:] + [none], self.white[1:] + [none], self.whites[1:] + [0]
    else:
      latest = min(self.since)
      self.since, self.white, self.whites = [latest] * 4, [none] * 4, [0] * 4  # white is the permissive answer: none assumed


class WhiteHold:
  """Per modelV2 lane line, how long and how far since it was last read white (at BROKEN_WHITE_CONF_MIN, any kind but none):
  until either passes WHITE_HOLD_S or WHITE_HOLD_M, a later read of it of unknown colour, kind broken or unknown, reads white,
  at no more than the white read's confidence. It never touches a line read solid, none or yellow, and LanePlanner applies
  it after the yellow hold, so a held yellow (LATCHED_YELLOW) is never white. Only on paint read: a frame nobody read stays
  as it came. Lane changes move the holds with the lines; one of unknown direction drops them (white is the permissive
  answer, so none is assumed for a line that may be any of the four).

  The colour of a dashed line comes from WINDOW_S of painted samples and drops to unknown before the kind does (a dash out of
  the window, too few painted samples: lane_paint COLOR_MIN_PAINT). Drive 1e (2026-11-06, after dark), the seg 8-9 approach
  (route seconds, 60 a segment): the left ego line read broken white at 0.8-1.0 from 523.8 s, then from 525.5 s for 2 s broken
  at 1.0 of unknown colour, and that one change aborted the offer replayed there mid-countdown (lineNotBroken). Over 518-569 s
  the line read broken white 480 times, unknown kind white 152 and broken of unknown colour 55.

  The hold bridges a dropout, it isn't a read: the colour also reads unknown on mixed votes, as a line turns from white to
  yellow (LanePlanner._held_only, drive 1d). So a frame valid by it alone keeps an offer's confirm count and an offer up, and
  is never the frame an offer, a request (the irreversible one) or an LC-03 placement is made on."""
  def __init__(self):
    self.since = [(math.inf, math.inf)] * 4
    self.confidence = [0.] * 4

  def update(self, paint, v_ego: float, dt: float, frozen: bool, read: bool = True) -> tuple:
    """paint as for paint_lanes, after the yellow hold; frozen: a lane change is under way (YellowHold.update); read: this
    frame's paint was read (four lines before the yellow hold padded it). Returns the paint with every held line's unknown
    colour read white."""
    self.since = [(t + dt, d + v_ego * dt) for t, d in self.since]
    if read and len(paint) == 4 and not frozen:
      for i, p in enumerate(paint):
        if _white(p):
          self.since[i], self.confidence[i] = (0., 0.), p.confidence
    return self.apply(paint, read)

  def apply(self, paint, read: bool = True) -> tuple:
    if not read or len(paint) != 4:
      return tuple(paint)
    held = [t < WHITE_HOLD_S and d < WHITE_HOLD_M for t, d in self.since]
    return tuple(replace(p, color='white', confidence=min(p.confidence, c))
                 if h and p.color == 'unknown' and p.kind in ('broken', 'unknown') else p
                 for p, h, c in zip(paint, held, self.confidence, strict=True))

  def shift(self, direction: int) -> None:
    """As YellowHold.shift; with the direction unknown every hold is dropped."""
    none = (math.inf, math.inf)
    if direction == LEFT:
      self.since, self.confidence = [none] + self.since[:3], [0.] + self.confidence[:3]
    elif direction == RIGHT:
      self.since, self.confidence = self.since[1:] + [none], self.confidence[1:] + [0.]
    else:
      self.since, self.confidence = [none] * 4, [0.] * 4


class LineMarks:
  """Per modelV2 lane line, what its own reads have said of its colour, three ways: marked, read yellow (any confidence) or
  solid; cleared, read white (at BROKEN_WHITE_CONF_MIN, any kind but none) on WHITE_CLEAR_FRAMES frames since its last mark,
  or the yellow moved out a lane from it since (YellowHold.moved_out: a turn bay opening); or neither, as every line is from
  the start. lane_agreed crosses a line the reader couldn't make out only while it is unmarked (LanePlanner._unread_ok), and
  toward the far side takes the map's lane list as a sign only once it is cleared (LanePlanner._cleared).

  Marked: the yellow hold keeps a yellow read for YELLOW_HOLD_S and YELLOW_HOLD_M; a centre line can go unread longer than
  that at night (1d and 1e, 2026-11-06: three of the thirty unsure stretches of an ego line between two reads of it 3.6-7.3
  s), and on an undivided road the model counts the oncoming lane beyond it (its edge line painted a lane out), so with the
  hold run out the agreement would be the oncoming lane's. One white read cleared it before (2026-11-07 review): 1e 243.35
  s, a centre line read yellow 3.3 s earlier read unknown kind white 0.61 on 2 frames, and lane_agreed crossed it on 4
  frames 1.0-1.4 s on until it read solid; 1e 601.2 s, a line that read yellow on 115 of 314 frames read white on 7 between
  two of them. A bay opening beside the centre line puts a white line where the yellow was and the yellow a lane out, and
  clears it as it ends the hold (1d 575.5-576.3 s; 1e 306.82 s).

  Cleared, for the far side's map-list sign: there the model's count can't tell our lane from the oncoming one beyond a
  centre line, and on a far_bay approach the list is the junction's, placing the car a lane or two short of its bay
  (desired_changes' bay rule); so the list says a lane of ours is there only over a line read white. Before (2026-11-07
  review), a centre line never read yellow or white (worn, or its colour unknown at night) on a [left (active), straight]
  approach, the oncoming lane counted beyond it, was offered left at the window's start, 293 m out.

  Lane changes move the reads with the lines; one of unknown direction marks every line any was and clears none (the
  cautious answer)."""
  def __init__(self):
    self.marked = [False] * 4
    self.cleared = [False] * 4
    self.whites = [0] * 4  # frames each line read white since its last mark (or the start)

  def update(self, paint, frozen: bool, moved_out=(False,) * 4) -> None:
    """paint: this frame's reads as handed in, before either hold; empty when not read. frozen: a lane change under way.
    moved_out: YellowHold.moved_out, this frame's."""
    if len(paint) != 4 or frozen:
      return
    for i, p in enumerate(paint):
      if p.kind == 'solid' or p.color == 'yellow':
        self.marked[i], self.cleared[i], self.whites[i] = True, False, 0
      elif _white(p):
        self.whites[i] += 1
      if moved_out[i] or self.whites[i] >= WHITE_CLEAR_FRAMES:
        self.marked[i], self.cleared[i] = False, True

  def new_maneuver(self) -> None:
    """A new maneuver: no line is cleared for it yet (marked kept, the cautious side). Review 2026-11-07 (LC-1): cleared never
    aged nor reset, so a clear earned by one road's line (1d 370.75 s: the junction's own paint read unknown-kind white 10 m
    out, 1 of the 8 turns of 1d and 1e) carried through the turn onto the next road's centre line, and the far side's list
    alone agreed a change over a line the reader never made out, into the oncoming lane (offered at 293 m in the probe). Its
    white reads count afresh: a real lane line clears again in WHITE_CLEAR_FRAMES (0.5 s). Never aged on its own: a line read
    white, then unknown at night, is still the one read white (test_lane_planner_night)."""
    self.cleared, self.whites = [False] * 4, [0] * 4

  def shift(self, direction: int) -> None:
    if direction == LEFT:
      self.marked, self.cleared, self.whites = [False] + self.marked[:3], [False] + self.cleared[:3], [0] + self.whites[:3]
    elif direction == RIGHT:
      self.marked, self.cleared, self.whites = self.marked[1:] + [False], self.cleared[1:] + [False], self.whites[1:] + [0]
    else:
      self.marked, self.cleared, self.whites = [any(self.marked)] * 4, [False] * 4, [0] * 4


class BayLatch:
  """The bay rule's latch (desired_changes, far_bay; LanePlanner.bay): shut, the far-side ego line taken for the centre line
  and the car for beside it, once that line has read yellow at BAY_YELLOW_CONF_MIN BAY_YELLOW_READS times, none of those
  reads within BAY_CONTEST_S after a white read of the same line; open again on any white read of it at BROKEN_WHITE_CONF_MIN
  (any kind but none), while the model counts a painted lane of ours that way (the count known, after paint_lanes on the
  yellow hold's paint), the maneuver no far_bay, or the car's lanes changing (shift).

  For LC-03 yellow is the permissive answer here: shut, the turn wants no change and is never skipped for its lane, so it
  goes from the lane the car is in. A lane divider taken for the centre line sends the turn from the curb lane across the
  through lanes beside it. So the latch is judged on each frame's own reads as handed in, before the yellow hold (which
  writes a line read white under it yellow, hiding exactly the reads that contest it), at a higher bar than the yellow hold's
  YELLOW_CONF_MIN, and on reads that nothing white near them contradicts. Drive 1e (2026-11-06, after dark): on the seg 8-9
  approach (a two-lane one-way) the left lane divider read broken yellow 0.55-0.75 on 28 frames over 536.0-537.3 s, white
  on 621 of the 1041 frames of 518-570 s, and shut from that one misread the old latch wanted no change for 10.5 s; on
  the seg 5 bay approach the centre line read yellow on 332 frames over 270-327 s, 329 of them at 0.5 or better, and white on
  none of them before 304.8 s, after its last yellow. The counted lane is the yellow hold's paint's, not the model's raw
  count: on an undivided road the model counts the oncoming lane beyond the centre line (seg 5: 106 frames of 270-327 s, most
  inside the yellow, 276-296 s), and the held yellow takes every one of them out."""
  def __init__(self):
    self.shut = False
    self.reads = 0                    # uncontested yellow reads of the far-side ego line since the latch was last open
    self.white_t = [math.inf] * 4     # s since each line's last white read (this frame's own, any kind but none)

  def reset(self) -> None:
    self.shut, self.reads = False, 0

  def update(self, bay: bool, paint, i: int, counted: bool, dt: float, frozen: bool) -> bool:
    """bay: far_bay(maneuver); paint: this frame's as handed in, before either hold, empty when not read; i: the far-side
    ego line's index (as for paint_lanes); counted: the model counts a painted lane that way, its count known, after
    paint_lanes on the yellow hold's paint; frozen: a lane change under way (no read counts). Returns shut."""
    self.white_t = [t + dt for t in self.white_t]
    read = len(paint) == 4 and not frozen
    if read:
      self.white_t = [0. if _white(p) else t for p, t in zip(paint, self.white_t, strict=True)]
    if not bay or counted or (read and _white(paint[i])):
      self.reset()
    elif read and _bay_yellow(paint[i]) and self.white_t[i] >= BAY_CONTEST_S:
      self.reads += 1
      self.shut = self.shut or self.reads >= BAY_YELLOW_READS
    return self.shut

  def shift(self, direction: int) -> None:
    """The car's lanes changed (a change ended, or a line crossed): open, its reads counted afresh from the lane the car is
    in. The white reads move with the lines as the holds do; with the direction unknown, the latest covers all four lines
    (any of them may be the line it was read on: the cautious answer for the latch, as YellowHold.shift's is for a change)."""
    if direction == LEFT:
      self.white_t = [math.inf] + self.white_t[:3]
    elif direction == RIGHT:
      self.white_t = self.white_t[1:] + [math.inf]
    else:
      self.white_t = [min(self.white_t)] * 4
    self.reset()


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
  blocked: bool = False      # the change the maneuver needs hadn't gone by the window's end, the car placed in a lane the
                             # maneuver doesn't leave from (LC-03, module docstring): turnpilotd skips the maneuver
  hold: str = 'none'         # TurnpilotPlan.laneChangeHold: the offer held for the blind spot, or its release (LC-01)


class LanePlanner:
  def __init__(self, paint_rule: bool = True):
    self.paint_rule = paint_rule      # TurnPilotLanePaintActive: plan lane changes at all
    self.always_drive = False         # TurnPilotAlwaysDrive, handed over every frame (turnpilotd.hand_always_drive; ALWAYS_DRIVE.md section 1)
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
    self.white = WhiteHold()          # applied after the yellow hold; never in keeps (turnpilotd's keep gate: yellow.apply)
    self.bay = BayLatch()             # the bay rule's latch (desired_changes), on each frame's own reads
    self.marks = LineMarks()          # lines read yellow or solid: never crossed on less than a read (lane_agreed)
    self._parked = False              # something stopped in the lane that way on a frame of the window, since the car's last
                                      # lane change: the room beyond the line places nothing for LC-03
    self._lane_painted = False        # ...LC-03 placed by a painted lane counted that way (not the room beyond the line)
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
    self._unsure_t = 0.               # s an offer has waited on the line read short of broken white since its last frame read it
    self._soft_t = 0.                 # s the change being confirmed has waited on soft_block frames since its last that passed
    self._pending = ''                # an abort in REQUEST, said once DesireHelper is seen not to have started
    self._pending_t = 0.
    self._wait, self._want, self._unreachable = 'none', 0, False  # this frame's LaneDecision.wait, want, unreachable
    # this frame a soft one inside a live confirm count (soft_block or the white hold alone, the count kept): the change still
    # pending, whatever its wait says (turnpilotd's lane_pending, LC-4)
    self.confirm_live = False
    self._unreachable_logged = None   # the maneuver whose unreachable lane was logged
    self._offer_driver = True         # who signals this offer, latched at its warning (module docstring)
    self._lane_seen = False           # a travel lane read the way wanted on a frame of the window, since the car's last lane
                                      # change (LC-03's placement without a lane list)
    self._room_seen = 0               # ...frames in a row it was there by the room beyond broken white only (lanes_toward)
    self._judged_id = UNJUDGED        # the maneuver whose window's end LC-03 judged
    self._blocked = False             # ...and its verdict, held for it: LaneDecision.blocked
    self._last_wait = 'none'          # the last decision's wait (LC-03's log: what held the change)
    self._hold = 'none'               # under the switch, the offer held for the blind spot ('blindspot') or its release counting
                                      # down ('clear'): LaneDecision.hold (LC-01)
    self._clear_t = 0.                # ...s the blind spot holding it has read clear
    self._curb_marked = False         # the curb-side ego line read solid or yellow on a frame of the window, since the car's last
                                      # lane change: no change on less than paint there for the rest of it (4.10)
    self._carry = LaneChange.none     # a change of ours just made, with the route wanting another that way: signal_side
    self._carry_t = 0.                # ...s past the cooldown with no frame passing change_blocker that way by its own read

  @property
  def active(self) -> bool:
    return self.phase != Phase.IDLE or self.ours

  @property
  def signal_side(self) -> int:
    """The side to signal for the planner: the offer's from its warning, ours until the change under way is over, and on
    from it through the cooldown while the route wants another change that way, until that one is offered (or anything
    ends it: an abort, the window's end, the driver's lamp the other way). A driver crossing two lanes signals throughout;
    the car's lamp going dark between them (the turn not armed yet) started its 300 ft lead over, and the second change of a
    three-lane road had no window left for it. The carry lengthens one continuous request by a cooldown and an offer (tens
    of seconds at most), well inside the port's own cap on one (opendbc car_signal.REQUEST_CAP_S, 300 s). Past the cooldown
    it lasts while the next change could be offered: LC_UNSURE_MAX_S with no frame passing change_blocker that way by its
    own read puts it out (its clock stopped while that way's blind spot reads occupied; update), so with the line unread
    the lamp is dark LC_COOLDOWN_S + LC_UNSURE_MAX_S after a change, not lit to the window's end (2026-11-06 review: 13.35 s)."""
    if self.phase in (Phase.WARN, Phase.REQUEST, Phase.WAIT):
      return self.offer
    if self.ours and self._lc_dir:
      return _toward(self._lc_dir)
    return self._carry

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
    self.confirm_live = False
    if self._abort_t > 0.:
      self._abort_t -= dt
      if self._abort_t <= 0.:
        self.abort = 'none'
    if maneuver_id != self._done_id:  # a new maneuver: its changes are counted from here
      self._done_id, self.done, self._start = maneuver_id, 0, None
      self._lane_seen = self._curb_marked = self._parked = self._lane_painted = False
      self._room_seen = 0
      self.bay.reset()
      self.marks.new_maneuver()
      self._carry = LaneChange.none
    if maneuver_id != self._judged_id:
      self._blocked = False
    changing = lc_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
    read = len(paint) == 4  # this frame's paint read (turnpilotd.plan_paint), before the hold gives a frame with none four lines
    if changing and self._lc_dir is None:
      self.lines.reset()
      self._before = self._last_lanes
    self.lines.update(lanes, dt)
    self._last_lanes = lanes
    # a change just ended, ours or the driver's; or the car crossed a line with no change of DesireHelper's (openpilot off, a
    # manual change). The yellow holds move with the lines before this frame's paint goes into them: by this frame the model
    # has relabelled its lines, so this frame's yellow is already the lane the car is in. Shifted after it, a far-left centre
    # line read on the frame a change left ended came to sit on the ego line: the next change toward it read lineNotBroken
    # for YELLOW_HOLD_M (a three-lane road's second change never went), and the curb side's count went to 0 (the car placed
    # in the curb lane, the turn lane "Can't Confirm Lane")
    ended = not changing and self._lc_dir is not None
    crossed = not changing and self._lc_dir is None and self.lines.moved
    made = ended and self._lc_dir != 0 and (self.lines.moved == self._lc_dir or counts_shifted(self._before, lanes, self._lc_dir))
    if ended or crossed:
      self.yellow.shift(self._lc_dir if made else 0)
      self.white.shift(self._lc_dir if made else 0)
      self.bay.shift(self._lc_dir if made else 0)  # read from the lane the car left: open
      self.marks.shift(self._lc_dir if made else 0)
    # the yellow hold first, then the white: a line held yellow is never made white. own: this frame's own reads, the yellow
    # hold applied but not the white, for what the white hold may not do (a request, LC-03's placement: held_only below)
    raw, own = paint, self.yellow.update(paint, v_ego, dt, changing)
    paint = self.white.update(own, v_ego, dt, changing, read)
    # this frame's yellow or solid marks its line, its white reads or the yellow moving out a lane clear it, before any check
    self.marks.update(raw, changing, self.yellow.moved_out)
    # the bay rule's latch (BayLatch), on this frame's reads as handed in. The lane it counts that way only on paint read: with
    # none (and no hold to pad it) paint_lanes gives the model's raw count, the oncoming lane beyond the centre line
    far = LEFT if right_hand_traffic else RIGHT
    counted = paint_lanes(lanes, own)
    counted = read and (counted.left >= 1 and lanes.left_known if far == LEFT else counted.right >= 1 and lanes.right_known)
    self.bay.update(far_bay(maneuver, right_hand_traffic), raw, 1 if far == LEFT else 2, counted, dt, changing)
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
    if ended:
      if made and self.done is not None:
        self.done += self._lc_dir
      else:  # a way not seen, or the car not seen to get there: nothing to dead reckon from for the rest of this maneuver
        self.done, self._start = None, None
      self._carry = _toward(self._lc_dir) if made and self.ours else LaneChange.none  # its lamp on for the next (signal_side)
      self._carry_t = 0.
      self._lc_dir = None
      self.lines.reset()
      self._cooldown = max(self._cooldown, LC_COOLDOWN_S)
      self._lane_seen = self._curb_marked = self._parked = self._lane_painted = False  # read from the lane the car left
      self._room_seen = 0
    elif crossed:
      self.done, self._start = None, None
      self.lines.reset()
      self._lane_seen = self._curb_marked = self._parked = self._lane_painted = False
      self._room_seen = 0
    if self.ours:  # DesireHelper is done with it, whether or not a held blinker keeps it in preLaneChange
      self.ours = False
      self.changes += 1

    # the bay rule (desired_changes) by its latch (BayLatch, updated above): latched past the yellow hold, since at night the
    # centre line reads yellow on a third of frames and unknown on the rest (1e: ego left yellow 31%, unknown 41%), and the
    # hold running out with the bay still shut would want the changes back, and LC-03 skip the turn at the window's end
    fast = v_ego > LC_SPEED_MIN
    wanted = desired_changes(maneuver, lanes, right_hand_traffic, paint, self.done, self._start, self.bay.shut)
    want = wanted if fast else 0
    room = offer_room_s(v_ego, lead_m if not driver_signals else None)
    start, end = lane_change_window(v_ego, abs(wanted), room)
    if len(paint) == 4 and 0. < distance < start:  # 4.10: a curb-side line read solid or yellow is never less than paint
      ego = paint[2 if right_hand_traffic else 1]
      self._curb_marked = self._curb_marked or _yellow(ego) or (ego.kind == 'solid' and ego.confidence >= SOLID_WHITE_CONF_MIN)
    # a curb-side change on less than paint (curb_lane_unpainted), only on paint read this frame
    unpainted = self.always_drive and not self._curb_marked and read
    win_start, win_end, stop_wait = stop_window(v_ego, distance, stop_d, abs(wanted), room)
    self._want = wanted
    want_dir = LEFT if want > 0 else RIGHT if want < 0 else 0
    if _toward(LEFT if wanted > 0 else RIGHT if wanted < 0 else 0) != self._carry or distance <= win_end or \
       not self.paint_rule or not inputs_valid or car_block:
      self._carry = LaneChange.none
    nav = _nav_lanes(maneuver)
    if nav and want_dir:
      lo, hi = nav_lane_span(len(nav), paint_lanes(lanes, paint), right_hand_traffic, self.done, self._start)
      self._from_lane = lo if want_dir == LEFT else hi
    else:
      self._from_lane = None
    # the bay shut places the car: beside the centre line, the far-most lane of its direction, the bay to open beyond it
    self.lane_unknown = self.paint_rule and maneuver is not None and 0. < distance < start and not self.bay.shut and \
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
    wanted_dir = LEFT if wanted > 0 else RIGHT if wanted < 0 else 0
    wanted_room = nav_room(maneuver, lanes, wanted_dir, right_hand_traffic, paint, self.done, self._start) if wanted_dir else None
    # lane_agreed only on paint read this frame, as curb_lane_unpainted: the yellow hold pads a frame nobody read to four lines,
    # the held one yellow and the rest unknown, and a held far-left yellow (its sign) or the list agreed every such frame
    unread_ok, cleared = self._unread_ok(wanted_dir, right_hand_traffic) and read, self._cleared(wanted_dir)
    blocker = change_blocker(wanted_dir, lanes, paint, objects, right_hand_traffic, wanted_room, unpainted, unread_ok, cleared) \
      if wanted_dir else ''
    held_only = self._held_only(wanted_dir, blocker, lanes, own, objects, right_hand_traffic, wanted_room, unpainted, unread_ok, cleared)
    # a frame nobody read (no paint, and no hold padding it: a wide camera run, a stop sweep) is no evidence either way: it
    # keeps an offer's confirm count and LC-03's streak, as a frame valid by the white hold alone does, and nothing is offered,
    # requested or placed on it (Kellen, 2026-11-07: no give-ups for lack of a read). 1e and 1d (2026-11-06) moving, the wide
    # camera's runs read no paint on 18% and 26% of frames, 13 and 13 of their stretches over 3 s, and one unread frame reset
    # a confirm count of nine
    unread = bool(wanted_dir) and not read and not paint and blocker == 'lineNotBroken'
    soft = bool(wanted_dir) and bool(blocker) and soft_block(wanted_dir, blocker, lanes, paint, objects, wanted_room, right_hand_traffic)
    keep = held_only or unread  # LC-03's room streak: a frame valid by the white hold alone, or one nobody read
    in_window = end < distance < start
    if wanted_dir and in_window and stopped_in_lane(objects, wanted_dir, lanes.width):
      self._parked = True  # something stopped in that lane on a frame of the window: the room beyond the line is a parking strip
    if wanted_dir and in_window and lanes.measured and not blocker and not held_only:
      # a travel lane that way, offered or not (LC-03): on one frame by the model's painted count, as ever; by the room beyond
      # broken white only on LC_CONFIRM_FRAMES in a row, the bar an offer needs. With no line seen beyond, one frame of broken
      # white is weak evidence for a skip: the whole-drive replay of 1e with a right every 40 s placed one by a single frame.
      # Never on a frame valid by the white hold alone (held_only): that keeps the streak, as it keeps an offer's confirm count.
      # A room placement is undone by a stopped car seen in that lane on any frame of the window (_parked, _judge_end)
      painted = (paint_lanes(lanes, paint).left if wanted_dir == LEFT else paint_lanes(lanes, paint).right) > 0
      self._lane_painted = self._lane_painted or painted
      self._room_seen = 0 if painted else self._room_seen + 1
      self._lane_seen = self._lane_seen or painted or self._room_seen >= LC_CONFIRM_FRAMES
    elif not (keep and in_window and lanes.measured):
      self._room_seen = 0
    if self._judged_id != maneuver_id and maneuver is not None and 0. < distance <= end and self.phase == Phase.IDLE and \
       not self._pending:
      self._judge_end(maneuver, maneuver_id, distance, lanes, paint, v_ego, right_hand_traffic, wanted, blocker, nav)

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
                             inputs_valid, lit_m, lead_m, car_block, unpainted, read, own)

    self._wait = 'none' if maneuver is None else 'paintRuleOff' if not self.paint_rule else \
      'inputsInvalid' if not inputs_valid else 'laneUnknown' if self.lane_unknown else 'none' if not wanted else \
      stop_wait or ('cooldown' if self._cooldown > 0. else 'beforeWindow' if distance >= win_start else
                    'pastWindow' if distance <= win_end else 'tooSlow' if not fast else blocker or 'confirming')
    if self._cooldown > 0.:
      self._cooldown -= dt
      self._confirm = 0
      return self._decision(idle_reason)
    if self._carry != LaneChange.none:
      # the carry's clock (signal_side), past the cooldown with nothing offered: the lamp a change of ours left lit for the
      # next one the same way goes out once LC_UNSURE_MAX_S have passed with no frame that passes change_blocker that way by
      # its own read (the carry is cleared at once whenever the route wants another way, so blocker is that way's), as an
      # offer's wait (_follow_up). Before, nothing about paint ended it: a three-lane left at 11.2 m/s with the 300 ft lead,
      # the new left ego line read unknown after the first change, kept the lamp lit 13.35 s with no offer and no change to
      # the window's end (2026-11-06 review, HEAD the same). Looser than the offer's wait, deliberately: every frame that
      # doesn't pass counts, contrary reads, no paint read and noLane alike, and none ends it at once. Nothing was offered,
      # so 3 s of lamp is all that is at stake, and at night the road edge's std flickers over EDGE_STD_MAX (1e 535.67 s),
      # a single noLane frame, and a wide camera run reads no paint for a stretch. Its clock stops while the blind spot that
      # way reads occupied, the car beside hiding the line, as LC-01's hold: a human's lamp stays on waiting for the gap
      if left_blindspot if _direction(self._carry) == LEFT else right_blindspot:
        pass
      elif blocker or held_only:
        self._carry_t += dt
        if self._carry_t >= LC_UNSURE_MAX_S - 1e-6:
          self._carry, self._carry_t = LaneChange.none, 0.  # dark, nothing said: no offer was made (the wait says why)
      else:
        self._carry_t = 0.
    if not self.paint_rule or not inputs_valid or stop_wait or not (win_end < distance < win_start):
      want_dir = 0
    if want_dir and blocker and not soft:
      want_dir = 0
    direction = _toward(want_dir)
    # a frame valid by the white hold alone (held_only), or one whose blocker is the want of a read or of the model's count
    # (soft_block: a frame nobody read included), keeps the confirm count, neither counted nor a break in it, for
    # LC_UNSURE_MAX_S past the last frame that passed, and is never the one an offer is made on: the same need
    # LC_CONFIRM_FRAMES frames that pass by their own read, nothing read against it between them
    held_only = (held_only or soft) and direction != LaneChange.none
    if not held_only:
      self._confirm = self._confirm + 1 if direction == self._candidate else 1
      self._soft_t = 0.
    else:
      self._soft_t += dt
      if direction != self._candidate or self._soft_t >= LC_UNSURE_MAX_S - 1e-6:
        self._confirm = 0
    self._candidate = direction
    # review 2026-11-07 (LC-4): such a frame's wait is its blocker (lineNotBroken, noLane), not one turnpilotd counts as a
    # change pending, so the wait alternated with 'confirming' frame to frame and the turn's prompt to signal flickered (1d
    # 576.65-577.45 s, a left 119-110 m out: every 50-400 ms), its chime cut to a blip, and a lamp lit for it was taken as the
    # change's go-ahead at the offer's first frame (WARN's rule, D15's case). The count live, the change is pending
    self.confirm_live = held_only and self._confirm > 0
    if direction == LaneChange.none or self._confirm < LC_CONFIRM_FRAMES:
      return self._decision(idle_reason)
    if left_blindspot if want_dir == LEFT else right_blindspot:
      self._wait = 'blindspot'
      return self._decision("car in blind spot")
    if target_lane_blocked(objects, want_dir, v_ego, lanes.width):
      self._wait = 'carInLane'
      return self._decision("car in target lane")
    if held_only:
      return self._decision(idle_reason)
    self._wait = 'none'
    self.phase, self.offer, self.countdown = Phase.WARN, direction, LC_WARN_S
    self._offer_driver = driver_signals
    self._maneuver_id, self._wait_t, self._pre_t, self._unsure_t = maneuver_id, 0., 0., 0.
    self.abort, self._abort_t = 'none', 0.
    return self._decision(prompt=driver_signals)

  def _judge_end(self, maneuver, maneuver_id, distance, lanes, paint, v_ego, right_hand_traffic, wanted, blocker, nav) -> None:
    """LC-03 at the window's end, once for the maneuver (module docstring): blocked when a change is still wanted (wanted,
    desired_changes': a car never offered one for its speed counts), the car is placed, the planner is on and the model sees a
    lane that way (blocker isn't noLane)."""
    self._judged_id = maneuver_id
    # without a list: a painted lane counted that way places the car; the room beyond the line only with nothing seen stopped
    # in that lane all window (drive 1e 722-737 s: a right, the room 3.2-4.6 m beyond the right ego line, parked cars 4.4-4.7 m
    # right 7-33 m ahead, placed by the room between them, and the right skipped "Can't Reach Turn Lane" for a parking strip)
    placed = not lane_unconfirmed(maneuver, lanes, right_hand_traffic, paint, self.done, self._start) if nav else \
      self._lane_painted or (self._lane_seen and not self._parked)
    self._blocked = bool(wanted) and placed and self.paint_rule and blocker != 'noLane'
    if wanted:
      cloudlog.event("turnpilot.lane_blocked" if self._blocked else "turnpilot.lane_not_blocked", error=self._blocked, want=wanted,
                     distance=round(distance, 1), v_ego=round(v_ego, 2), lane_list=bool(nav), placed=placed, blocker=blocker,
                     abort=self.abort, wait=self._last_wait)

  def _unread_ok(self, direction: int, right_hand_traffic: bool = True) -> bool:
    """A line the reader couldn't make out may be crossed toward direction (lane_agreed): the ego line that way unmarked, and
    toward the curb side never read solid white or yellow on a frame of the window (4.10's latch, _curb_marked, as
    curb_lane_unpainted has it). Review 2026-11-07 (LC-2): the latch gated only the 4.10 change, so a curb-side line read
    solid, then unknown-kind white on WHITE_CLEAR_FRAMES frames (a solid line part-lit, worn or in dappled shade: lane_paint
    gives kind unknown at 0.5-0.8 coverage), was crossed on lane_agreed's white sign, the switch on or off; HEAD offered
    nothing there. The caller adds that this frame's paint was read."""
    if direction not in (LEFT, RIGHT) or self.marks.marked[1 if direction == LEFT else 2]:
      return False
    curb = RIGHT if right_hand_traffic else LEFT
    return not (direction == curb and self._curb_marked)

  def _cleared(self, direction: int) -> bool:
    """The ego line toward direction cleared by its white reads or the yellow moving out (LineMarks): lane_agreed's far side."""
    return direction in (LEFT, RIGHT) and self.marks.cleared[1 if direction == LEFT else 2]

  @staticmethod
  def _held_only(direction, blocker, lanes, own, objects, right_hand_traffic, room, unpainted, unread_ok=False,
                 cleared=False) -> bool:
    """A change toward direction valid (blocker '', change_blocker's on the white hold's paint) by the white hold alone: on
    own, this frame's reads with the yellow hold but not the white, change_blocker says no (the ego line broken of unknown
    colour, so not broken white; lanes_toward's room beyond it uncounted). Such a frame keeps an offer's confirm count and
    LC-03's room streak, and an offer up waits on it (_follow_up), but nothing irreversible goes on it: no request, no
    placement for LC-03 (2026-11-06 review). The colour reads unknown on mixed votes as well as on too few painted samples
    (lane_paint._color), so the hold bridges just the frames where a line turns from white to yellow: drive 1d, the left ego
    line broken white 0.70-0.87 over 315.6-321.4 s, broken of unknown colour 321.52-322.41 s (held white, with room to the
    edge and on half of them no lane the model counted), broken yellow 0.40-0.57 from 322.67 s; 23 of the 26 frames the hold
    made broken white on that line in all of 1d lay within 5 s of a yellow read of it (1e: 10 of 67)."""
    return bool(direction) and not blocker and \
      change_blocker(direction, lanes, own, objects, right_hand_traffic, room, unpainted, unread_ok, cleared) != ''

  def _follow_up(self, maneuver, maneuver_id, distance, end, want_dir, fast, lanes, paint, objects, lc_state, v_ego,
                 left_blinker, right_blinker, left_blindspot, right_blindspot, driver_signals, right_hand_traffic, dt,
                 inputs_valid, lit_m=math.inf, lead_m=None, car_block=False, unpainted=False, read=True, own=None) -> LaneDecision:
    """An offered change, every frame until DesireHelper starts it: still wanted, still valid, still clear, still signaled.
    unpainted: a curb-side change may go on less than paint this frame (update's: under the switch, on paint read); read:
    this frame's paint was read (update's); own: this frame's reads before the white hold (update's; paint when None).

    The line that way short of broken white aborts the offer (lineNotBroken) only on a contrary read (line_contrary: solid,
    yellow read or held, no line). Read but unsure (its kind unknown, broken of unknown colour, broken white short of
    BROKEN_WHITE_CONF_MIN, with nothing agreeing a lane is there, lane_agreed; or valid by the white hold alone, _held_only),
    or a frame with no paint read at all (nothing looked at the line: a wide camera run, a stop sweep, a stale scene; it
    aborted the offer before 2026-11-07, Kellen: no give-ups for lack of a read; 1e 776.8-782.5 s, a right's offer out on
    broken white read and requested, died on the wide camera's first unread frames at 4 m/s), the offer waits: up, its
    countdown and timeouts running, but nothing requested on such a frame, so DesireHelper starts the change only on one that
    passes by its own read (broken white, or lane_agreed). The rest of change_blocker (the lane list's room, a parked lane)
    is checked on it as on any frame; the model's lane count isn't, lanes_toward counting one beyond broken white only. Drive
    1e seg 8-9 (2026-11-06): the left ego line's reads flicked between broken white and unknown kind (white) every few tenths
    of a second, and an offer replayed there died on its first unsure frame.

    The wait lasts LC_UNSURE_MAX_S from the last frame read broken white, then aborts lineNotBroken, the lamp out and the
    abort said; its clock stops while the blind spot reads occupied (the car beside hides the line; LC-01's hold goes on). A
    withheld request takes DesireHelper out of preLaneChange, so LC_REQUEST_TIMEOUT_S doesn't run while it waits, and before
    only tooLate bounded it: an offer confirmed on broken white whose line then read unknown kept the car's own lamp lit 14.2 s
    with no change to the window's end (2026-11-06 review), against the drive's 'working indicators' ask. On drives 1d and 1e
    (2026-11-06, after dark) 27 of the 30 unsure stretches between two broken-white reads of an ego line, moving, ended within
    2 s (a dash's colour or kind dropping out, the longest 1.95 s: 1e 525.5-527.4 s, broken of unknown colour); the three
    longer, 3.6-7.3 s, read nothing of the line (kind and colour unknown at 0 confidence) on 86-90% of their frames. Nothing
    goes on an unsure frame, so the model's lane count isn't asked on it (the edge's std flickering over EDGE_STD_MAX at night
    would abort the offer on noLane there too); the request frame asks it, passing by its own read."""
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
    occupied = left_blindspot if d == LEFT else right_blindspot
    room = nav_room(maneuver, lanes, d, right_hand_traffic, paint, self.done, self._start)
    unread_ok, cleared = self._unread_ok(d, right_hand_traffic) and read, self._cleared(d)  # (agreement only on paint read: update's)
    reason = change_blocker(d, lanes, paint, objects, right_hand_traffic, room, unpainted, unread_ok, cleared)
    held_only = self._held_only(d, reason, lanes, paint if own is None else own, objects, right_hand_traffic, room, unpainted,
                                unread_ok, cleared)
    # read but unsure, nobody read this frame, or no lane by the model's count alone (soft_block): waits, nothing requested
    unsure = held_only or (bool(reason) and soft_block(d, reason, lanes, paint, objects, room, right_hand_traffic))
    if unsure:
      waited = reason or 'lineNotBroken'
      # no lane by what is read for sure: the list's none, an oncoming car, or a road edge the model is sure lies beside the
      # line (the count known, none: the lane ended), aborted at once as on any frame
      edge = lanes.left == 0 and lanes.left_known if d == LEFT else lanes.right == 0 and lanes.right_known
      reason = 'noLane' if (room is not None and room <= 0) or oncoming_in_lane(objects, d, lanes.width) or edge else \
        'laneParked' if stopped_in_lane(objects, d, lanes.width) else ''
      if not occupied:  # a car in the blind spot hides the line beside it: the clock stops while it holds the change (LC-02)
        self._unsure_t += dt
      if not reason and self._unsure_t >= LC_UNSURE_MAX_S - 1e-6:
        reason = waited  # no frame has passed for LC_UNSURE_MAX_S: the lamp goes out, said
    elif not reason:
      self._unsure_t = 0.
    if reason:
      return self._abort(reason)
    if want_dir != d:
      return self._abort('notNeeded')  # the model's lane count says the car is in the lane it wants
    if occupied and not self.always_drive:
      return self._abort('blindspot')  # under the switch the blind spot holds the change instead (LC-01)
    if target_lane_blocked(objects, d, v_ego, lanes.width):
      return self._abort('carInLane')

    signaled = left_blinker if d == LEFT else right_blinker
    if not driver_signals and car_block and self.phase in (Phase.WARN, Phase.REQUEST):
      return self._abort('noSignal')  # the driver's lamp the other way, or hazards: the driver declines the change
    if self._hold != 'none':
      held = self._held(occupied, driver_signals, signaled, dt)
      if held is not None:
        return held
    if self.phase == Phase.WARN:
      self.countdown = max(self.countdown - dt, 0.)
      if driver_signals and signaled:  # the driver's go-ahead, any time in the countdown
        self.phase = Phase.REQUEST
      elif self.countdown <= 0. and driver_signals:
        self.phase = Phase.WAIT
      elif self.countdown <= 0. and (lead_m is None or lit_m >= lead_m):
        self.phase = Phase.REQUEST  # the car's lamp has led the change by the legal distance
    elif self.phase == Phase.WAIT:
      if not occupied:  # (occupied only under the switch here) no noSignal while the blind spot holds the change (LC-01)
        self._wait_t += dt
      if signaled:
        self.phase = Phase.REQUEST
      elif self._wait_t > LC_SIGNAL_TIMEOUT_S:
        return self._abort('noSignal')
    elif self.phase == Phase.REQUEST:
      if driver_signals and not signaled:
        return self._abort('noSignal')  # the blinker went off before the change started: never change unsignaled
      if lc_state == LaneChangeState.preLaneChange and not occupied:  # LC-02: held, the request is withdrawn, its clock stops
        self._pre_t += dt
        if self._pre_t > LC_REQUEST_TIMEOUT_S:
          return self._abort('timedOut')
    if occupied and self.phase in (Phase.WARN, Phase.REQUEST):  # LC-01 (off the switch it aborted above): held
      self._hold, self._clear_t, self.countdown = 'blindspot', 0., 0.
      return self._decision(HOLD_REASON, prompt=driver_signals)
    if self.phase == Phase.REQUEST:
      self.countdown = 0.
      if unsure:  # waits for a frame that reads the line broken white: nothing requested on this one
        return self._decision(LINE_UNSURE_REASON, prompt=driver_signals)
      return self._decision(request=True, prompt=driver_signals)
    return self._decision(prompt=driver_signals)

  def _held(self, occupied: bool, driver_signals: bool, signaled: bool, dt: float) -> LaneDecision | None:
    """LC-01, under TurnPilotAlwaysDrive: an offer held for the blind spot ('blindspot', nothing requested, so DesireHelper,
    which starts a requested change the instant it clears, never goes on its own) or its release counting down ('clear'). The
    decision this frame, or None once released: the phase's own rules go on this frame (the request; the driver's blinker or
    the lamp's lead from a held WARN, its countdown spent)."""
    if driver_signals and not signaled and self.phase == Phase.REQUEST:
      # the driver's blinker ended while only the blind spot held the change: asked again (WAIT's prompt, its chime), never
      # aborted noSignal; WAIT's clock waits while the blind spot reads occupied
      self.phase, self._wait_t, self._hold, self.countdown = Phase.WAIT, 0., 'none', 0.
      return self._decision(prompt=True)
    if occupied:  # (back) in the blind spot, the release countdown too: held, nothing requested
      self._hold, self._clear_t, self.countdown = 'blindspot', 0., 0.
      return self._decision(HOLD_REASON, prompt=driver_signals)
    if self._hold == 'blindspot':
      self._clear_t += dt
      if self._clear_t < LC_HOLD_CLEAR_S - 1e-6:
        return self._decision(HOLD_REASON, prompt=driver_signals)
      self._hold, self.countdown = 'clear', LC_RELEASE_S  # said again, its chime, then counted down: when the driver looks
      return self._decision(prompt=driver_signals)
    self.countdown = max(self.countdown - dt, 0.)
    if self.countdown > 1e-6:
      return self._decision(prompt=driver_signals)
    self._hold, self.countdown = 'none', 0.
    return None

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
    self.phase, self.offer, self.countdown, self._carry, self._carry_t = Phase.IDLE, LaneChange.none, 0., LaneChange.none, 0.
    self._confirm, self._candidate, self._wait_t, self._pre_t, self._unsure_t = 0, LaneChange.none, 0., 0., 0.
    self._soft_t = 0.
    self._pending, self._pending_t = '', 0.
    self._hold, self._clear_t = 'none', 0.

  def _decision(self, reason: str = "", request: bool = False, prompt: bool = False) -> LaneDecision:
    wait = 'none' if self.phase != Phase.IDLE or self.ours or self._lc_dir is not None else self._wait
    self._last_wait = wait
    return LaneDecision(self.offer if request else LaneChange.none, self.offer if prompt else LaneChange.none,
                        reason, self.offer, self.countdown, self.abort, wait, self._want, self._unreachable, self._blocked,
                        self._hold)
