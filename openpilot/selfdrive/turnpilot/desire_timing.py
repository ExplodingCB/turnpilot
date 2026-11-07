"""Desire timing: which desire to send the driving model for the next maneuver, and when.

modeld feeds the model only the rising edge of a desire, so the desire is held through the maneuver and the
lead distance is the timing lever. Gate 1 (big model, 25 km/h, 2026-10-27) took 90-degree turns from a pulse
40 m before the junction and not from 10-20 m; a keep desire goes within max(2.5 s * v, 20 m).
Where the lead is depends on the side the turn goes to (turn_lead; 2026-11-06, drive 1e after dark):
- A curb-side turn (a right where traffic keeps right; curb_side_turn) gets ONE pulse, CURB_TURN_PULSE_M (8 m) before the
  junction, with nothing before it: no pulse at the 40 m lead and no repeats. In 1e the model started each right 0.3-1.5 s
  after every fresh rising edge, wherever the car then was: the 40 m pulse, its repeat (a pulse lasts 6.6 s, and the 3 m/s
  crawl from 20 m lands the repeat about 20 m out) and the 10 m entry pulse each started the right 11-25 m before the
  junction on a 12-15 m radius, cutting the inside curb (the driver took over twice, one curb clip), where the human turned
  in about 6.7 m out on an 8 m radius. Gate 1's "not from 10-20 m" was the far side's wider arc at 6.9 m/s; the curb
  side's cap has the car at 3 m/s by 20 m (turn_speed.py), and one pulse at 8 m, 6.6 s long, covers the turn at that
  speed through about 12 m past the junction. Being inside ENTRY_M it is the entry pulse, so no repeat or entry follows it
  (the standstill re-arm, a hold's go, a re-engage still do). A hold's go standing further out than 8 m (a stop line, a red,
  the stop-sign decider's go) waits with no desire until the car is inside 8 m; turn_skip's tooLate deadline for it is
  5 m (CURB_FIRST_PULSE_M, turn_skip's). The junction's own hold line is 8 m out too (scene_policy.HOLD_LINE_M: a walker on
  the turn's crosswalk), and the car stands a little short of it (8.26 m in test_turnpilotd's ped_crosswalk_right): a go
  standing within CURB_GO_SLACK_M outside the pulse point is the pulse, at once, as every hold's go is (BeamNG
  ped_crosswalk_right: the model didn't pull away on its own with no fresh edge). Outside it by the reading the pulse goes
  by (pulse_distance, below), never the nearest alone (2026-11-07 review): the nearest keeps one reading 3-5 m short, and
  closed loop under 1d 5:02's readings (the furthest 2 m long, one 4.7 m short about 28 m out, the rest 2.5 m short) the go
  from a red line 9-13 m out pulsed standing 9.3-13.3 m from the junction by the nearest's 4.6-8.6, and the walker hold's
  (placed by the nearest) 13.0 m out: the early turn-in that clipped 1e's curbs. By pulse_distance the gos from 10-13 m and
  the walker's go with no desire and pulse rolling at 8.4-8.5 m (from 9 m, 8.76 m by it, the go is the pulse, 9.3 m out).
  The cost: a stand placed by the nearest (the walker's hold line, 8.26 m by it) with the furthest 9 m or more out, 0.74 m
  beyond the nearest, which on the drives was nearly every right (spread 0.7-6.7 m), is let go with no desire too, since the
  readings can't say which of them is right (the furthest 2 m long, the nearest true: pulsed rolling at 6.0 m by the
  latest). The car then pulls away on the model alone, as from every line further out since 2026-11-06 (that change's
  premise), or turnpilotd's no-move clock for a curb-side go (curb_release) hands back loudly, "Can't start the turn: take
  control" (cantStartTakeControl; NO_MOVE_S, under TurnPilotAlwaysDrive NO_MOVE_MAX_S), never with a turn started 13 m out.
  The pulse goes by the FURTHEST reading of the junction (far: RouteFollower's distance plus junction_spread), with the
  nearest still short of it: a pulse that starts the turn where the car is must never come early, and the nearest reading,
  right for a cap or a hold line (slow early, never late), is the early one. In 1d and 1e the readings of a right's junction
  spread 0.7-6.7 m where the nearest first came inside 8 m (3 rights of 8 by 5 m or more), and on the 90 deg rights whose
  arc said where the junction was (its midpoint 2.3 m short of the node on an 8 m radius: 1d 5:05, driven by hand; 1e 7:17)
  the nearest was 3-5 m short of it. But the furthest is over every reading since the step began (taken 43-114 m out, 8-26
  s old, for these rights) and holds long: 1.5-2 m beyond the arc's junction on those two, and on 1e's 91 deg right at 8:32
  (one follower replayed over the whole drive) still 8.8 m out with the nearest at the junction, about 10 m long, so by it
  alone that right got no pulse at all. So the pulse is due at the latest by navd's latest CONFIRMED reading of the junction
  (last: RouteFollower.junction_latest, dead-reckoned: a reading within route_follower.LATEST_AGREE_M, 5 m, of the one before
  it), CURB_LATEST_PULSE_M (6 m) out, whichever comes first (pulse_distance); on those three the latest was within 2.5 m of
  the arc's junction (2.5 m short, 0.8 m short, 1.5 m long). Replayed with it (open loop, engaged throughout), 1d's and 1e's
  rights pulse at the furthest's 8 m or the latest's 6 m, 4.2-6.8 m out by the nearest (1e 8:32 at 5.4 m, its latest a jump
  in of 4.0 m from the reading before it, so confirmed at once). The nearest can't be that deadline: a minimum, it keeps one
  reading 20 m short to the end. Nor can one reading alone (2026-11-06): one 9 m short about 20 m out was navd's latest until
  its next true reading, and the pulse went 14.8 m out on it (13 m short: 17.1 m), the early turn-in that clipped 1e's rights;
  one 15 m short there had the right skipped tooLate 17 m out. Confirmed, such an outlier never moves the latest, and the
  right pulses at 8 m by the furthest; one that the next reading bears out (a jump in of 5 m or more that lasts) is due a
  second later.
  A persistent bias short (two readings or more that agree, the furthest from before them) brings the pulse in by that bias
  less 2 m: which reading is right can't be told then, and the latest is the one that was right on the road.
  No first pulse goes once the junction is behind the car (below), whatever the others say: the late pulse is the dangerous
  one, a turn started past the corner into the exit road's far lanes. Behind by the NEAREST reading, one reading alone
  included, and turn_skip skips the turn out loud as that reading crosses the junction (tooLate, navd reroutes), never in
  silence, as it does any turn that reaches its junction with no first pulse out (1e 8:32, its furthest 8.8 m long). So one
  reading 8-10 m short 20-45 m out (an outlier the nearest keeps to the end) has the nearest past the junction before the
  furthest or the latest is inside the pulse point, and that right gets no pulse and is skipped out loud 7.7-9.8 m short of
  its junction, every later reading saying it is ahead (HEAD, pulsing at 40 m, took it), its corner cap kept (turnpilotd).
  That is kept on purpose (2026-11-06 review): the same readings are one TRUE reading inside a long bias of 8-10 m that lasts,
  and nothing in them tells the two apart. Behind by the second-nearest (that round's JunctionSpan.firm, since removed),
  that right was pulsed 0.3 m past its junction under an 8 m bias and 2.3 m past under 10 m, nothing said; by the nearest it
  is skipped at its junction, out loud. A missed right is rerouted; a turn started past the corner is not undone. One 4-7 m
  short is pulsed at 8 m by the furthest; one 11-20 m short (test_turnpilotd's GPS outlier, which by the nearest pulsed the
  right 28 m out) has the next true reading jump up from inside RouteFollower's ADVANCE_NEAR_M, taken for navd moving on: a
  new maneuver, its readings fresh, pulsed at 8 m (test_desire_timing). turn_skip judges a curb-side turn on the same
  readings (pulse_distance: its feasibility, tooLate and standing checks; the nearest: its crossing).
  That one pulse waits while the model already steers into the turn at a turn's scale (STEER_IN_TURN_CURV, 0.04 1/m, a 25 m
  radius) with the car rolling: being the turn's only edge it was exempt from the guard the entry pulse has, and a fresh rising
  edge into a model already turning restarts its turn (BeamNG right_green, below: unwound within 0.4 s, then round late into
  the oncoming half of the exit road). The rights most exposed are the ones the model starts by itself before 8 m: an
  end-of-road T, a right-turn-only lane, a slip lane. The wait's bar is a turn's, not STEER_IN_CURV (the repeat and entry
  guard's 0.005): at that bar every right off a road bending its way or with the model leaning to the curb waited to its
  deadline and was skipped 'Too Fast to Turn' (2026-11-06 review probes: 0.006-0.04 1/m from 40-60 m out, the car yawing with
  it, all skipped at 5 m, all pulsed at 8 m before the wait). In 1d and 1e, openpilot steering at 0.3-6 m/s with no desire in
  the model's queue, its curvature toward the right never reached 0.04 (peak 0.038; its episodes of 0.02 or more ran up to
  2.75 s, 7 m of road), and a rise over the approach's own mean told them apart no better (one of 2.5 s, 7 m, peaking at
  0.036); a city road's bend is under it (over a 25 m radius) and the model's own right at the cap's 3 m/s over it (0.05-0.125,
  an 8-20 m radius). right_green's steer (46-75 deg of wheel at the game car's 12:1 on a 2.9 m wheelbase: 0.023-0.038) was a
  model executing a pulse already in its queue, which a curb-side turn's one pulse never follows; a model ramping into its own
  turn through 0.005-0.04 on the very frame the pulse is due gets it (a fraction of a second of road).
  From the model's turn-scale steer-in inside APPROACH_M (20 m, the nearest reading) the car's turn toward it counts, before
  the pulse point as after it (_approach), until the model lets go (under STEER_IN_CURV): a right the model starts early on a
  wide arc (0.04-0.055 from 12-16 m) has the car 20-26 deg round by the 5 m deadline, and counted only from the wait's start
  (8.6 deg) it was skipped there mid-turn, its cap and its crosswalk's walker hold dropped with it. A steer-in already under
  way when the turn came up (the tail of the turn before it, a right 15 m on) is not its: the count begins only once the model
  has been seen not steering in for it. A reroute's new step for the same junction (same_bearings within SAME_TURN_M, as
  RouteFollower carries its span) carries the count on. Then: the car
  OWN_TURN_DEG round, on the approach or at the pulse point, the turn is the model's own and counts as started (own: no pulse
  ever goes into it, as for any turn under way; turnpilotd's in-turn watches, its caps and holds, its clear at its angle and
  turn_skip's never skipping a turn out all apply); once the pulse waits, the car following the model OWN_FOLLOW_DEG round
  is enough (over 3 m of road at 1.5-4 m/s with the model straight the car's heading by its yaw moved under 1 deg in 1d and
  1e, 0.89 p99, and with the model at 0.04 or more throughout, 9.3 deg or more at p1 in 1e); the model lets go of the corner (under
  STEER_IN_TURN_CURV) short of that, the pulse goes at once (no steer to unwind); it is still steering in at a turn's scale
  with the car not following it round when turn_skip's tooLate deadline comes (5 m by the furthest, 3 m by the latest),
  turn_skip skips it out loud there and no pulse goes at or after the deadline: a pulse then is the right_green failure (the
  model steering hard, the car barely round) made later still, past the point where the model's 0.3-1.5 s start lands it at
  the corner's apex. Nor does a first pulse go into a car TURN_STARTED_DEG round since that steer-in. OWN_TURN_DEG is 10 on
  the approach: a 17 m mean radius over 3 m of road, a turn, where an abrupt 0.5 m lane correction in 6 m peaks at 7.5 deg,
  and TurnPilot's own lane changes are done by 60 m out (lane_planner.LC_END_MIN_M). Replayed through one follower over 1e
  with the logged model's curvature (it ran under the old 40 m pulse, so it was already turning: a stress case), the engaged
  rights it steered into are its own and none is pulsed into: 6:41 (52 deg, 0.054 1/m at 8 m) once its pulse waited, the
  car following it 3.7 deg round 6.7 m out; 8:30 (91 deg, 0.08-0.09 1/m) on its approach, 10 deg round 12 m out (counted
  from the wait, 3.5 m out); engaged throughout, 7:15 too (5.5 m out), and 1d 5:02, driven by hand (the logged curvature
  follows the human), is pulsed as the model's 0.097 eases to 0.012 at 0.7 m/s (4.2 m out by the nearest, 6 by the latest).
  A bend of a 25 m radius or tighter that runs into the junction from inside 20 m reads as the model's
  own turn too, and gets no pulse (only the map's road heading would tell them apart). Taken as its own, the model can still
  let go: straightening (under STEER_IN_CURV) with the car under TURN_STARTED_DEG round and the nearest OWN_DROP_MIN_M or
  more short of the junction, the turn was not its own after all and goes back to the wait, so its pulse goes then if it is
  short of the deadline (a fresh edge into a model not steering in, as the entry pulse's let-go) and else none goes and
  turn_skip skips it out loud on its next update. Closed loop under turnpilotd, an own turn the model straightened out of at
  10 deg and drove through was otherwise said by nothing: R8b's straight watch broke on the steer-in's curvature,
  turn_abandoned needs it to point away, check_missed needs a go. Within OWN_DROP_MIN_M of the junction or past it it stays
  the model's own, and since 2026-11-06 turnpilotd says it: a steer-in no longer breaks R8b's watch, which misses it at the
  pass (turnpilotd.watch_straight). Nor does a pulse go once a waiting pulse is past its deadline (CURB_FIRST_PULSE_M by the
  reading it goes by, moved in past a wrong-way opening as turn_skip's is), turn_skip's skip saying it, where turn_skip
  spares it (the decider's turn crawling, a map stop the car makes short of the junction). Standing (a hold's go, its hold
  line's go, a queue) the pulse goes whatever the model steers: the car isn't turning, and the model doesn't pull away on its
  own without the edge (ped_crosswalk_right).
- A far-side turn (a left across the road) keeps the 40 m lead at turning speed, scaled by speed below it: FAR_LEAD_S
  before the junction, never under FAR_LEAD_MIN_M (15 m) nor over TURN_DESIRE_LEAD_M. A crawl no longer holds a desire 10-30
  s and takes the first opening that way (1e: a 40 m desire at 0.7 mph took a parking-lot opening 25 m early); at the 6.9
  m/s the turn cap leaves the car at by then it is the 40 m it was (turn_skip's deadline scales with it, short of it).
Once a desire
starts it is latched until the maneuver is passed (a keep: or the timeout; a turn is over only when the car has turned it
or is past it, never on a clock: a slow approach in a queue to a hold took longer, and the go then had no turn to pulse,
silently): navd's 1 Hz distance carries a few meters of noise, and dropping the desire whenever it read past the lead
again sent the model a new rising edge each time.
A keep toward a side where the model sees no lane line drove the car off the road there, so it doesn't go (the caller's
keep_left_ok/keep_right_ok); one still unsent keep_last before its point is said and skipped, never left to the model in
silence nor sent late (turnpilotd.keep_unsent: BUILD_FINDINGS D2-keep).
The model forgets a pulse 132 frames after it (6.6 s at 20 Hz, its desire history; see below): where the slow
approach took longer than that, it planned the turn and then dropped it 10 m out, and a late retry pulse brought
the turn back at once. So a far-side turn desire is pulsed again every REPULSE_S on the approach, and once more ENTRY_M
before the junction, so the last pulse is fresh for the whole turn: at 4 m/s a turn outlasts one pulse, and the
model let go of it halfway round (a curb-side turn's one pulse at 8 m is that entry pulse, above). Crossing ENTRY_M
within ENTRY_GAP_S of a repeat, the entry pulse goes once that gap is
past (it used to be used up then, and never went); a pulse that itself goes inside ENTRY_M is the entry pulse.
None once the car has turned TURN_STARTED_DEG (by its yaw rate): pulses during the turn turned the car on past the exit
road, into the curb. That holds for a repeat and one on engaging. Nor a repeat or the entry pulse while the model is steering
into the turn (modelV2.action.desiredCurvature STEER_IN_CURV or more toward its side: a 200 m radius, tightened from 100 m
on 2026-11-06 so that no pulse lands while the model has begun to steer in at all; a curb-side turn's one pulse, which no
pulse comes before, waits rolling only on a turn's steer-in, STEER_IN_TURN_CURV: above), however little the car has turned:
crawling into a right at 1-2 m/s (BeamNG right_green at cbdf5f4, both runs) the entry pulse went with the model at -46 and
-75 deg of steering but under 15 deg of heading, and within 0.4 s the model unwound to -1 and -23 deg and started the
corner again late, swinging into the oncoming half of the exit road for 31-32 m: a fresh rising edge restarts the model's
turn. The entry pulse waits while it steers in, and goes if it lets go of the corner short of TURN_STARTED_DEG. One as the car
pulls away after a standstill (STANDSTILL_REARM_S) goes while it has turned less than STANDSTILL_REPULSE_MAX_DEG: waiting
in the junction for oncoming traffic, 15-25 degrees into a left, longer than the model's queue, the turn would lapse
(H2, below: with the pulse gone at 5-16 degrees the model gave up 3 turns of 3); further round (stopped halfway for a
pedestrian) none goes, and the model finishes it (H2: 9 of 10 from 25-42 degrees). While a hold has the car standing
(turnpilotd: shouldStop) the turn desire is none, its edge still in the model's queue, so the pulse on the hold's go is
a fresh edge in the very plan that lets the car go; nor does a turn's first pulse go then (a right whose one 8 m pulse falls
due where a hold stands it: the stop-sign decider's handback 7.25 m out got a fresh turnRight with the car held, 2026-11-06
review), it goes on the go (BeamNG unprotected_left_visible: let go a frame or more before its
pulse, the car drove straight across the junction). A turn with no pulse before its hold (the autonomous unprotected left:
turnpilotd holds may_start off on its approach, so the car stands straight with the road camera up the oncoming approach)
takes its go as its first pulse, at once. That left's go withdrawn to its own hold while the car still rolls (withdraw())
takes its pulse out of the model's queue on the withdrawal (CLEAR_DESIRE) and sends none while held, so the model plans no
left as the hold brakes the car; the next go pulses a fresh edge.
A repeat pulse is a short drop to none, long enough that modeld (unsynchronized, conflated subscriber) can't miss it.
The model keeps the rising edges of the last 132 frames (6.6 s at 20 Hz, the big model's state_desire_q), and
forgets a pulse when it leaves that queue. A pulse still in the queue once the car is round the corner turns it
on: at a 55 degree "left" the car turned 122-126 degrees into the curb, the last pulse 6.6 s from leaving.
So once the turn is done, CLEAR_DESIRE goes out and modeld drops the turn desires from the queue: at the
junction (navd past it) once the car has turned CLEAR_MIN_DEG, or anywhere within CLEAR_MARGIN_DEG of the turn's
angle (80 degrees for a 90 degree left or right), but not before CLEAR_MIN_FRACTION of it (a 40 degree left at 30).
The angle is the junction's real one from Valhalla's bearings (route_follower.py), else the modifier's nominal one.
Short of CLEAR_MIN_DEG it waits: in H2's single-pulse runs, with the pulse gone at 5-16 degrees the model gave up
3 turns of 3, and at 25-42 degrees it finished 9 of 10 on its own (the one it gave up was at 32).
No turn pulse starts while the car is too fast to make the turn (turn_skip.py, may_start false): the turn speed cap
has the car at 6.9 m/s by a far-side turn's 40 m lead and 3 m/s by 20 m before a curb-side one, and faster than the check
allows something else holds the speed (the driver's foot), so the model would turn from road speed. Too fast for long,
the turn is skipped and no pulse goes at all. A pulse
already out is held and never cancelled, since dropping it and sending it again would be a new rising edge, and a turn
under way is finished. A keep goes at any speed.
Nor does any desire go out while openpilot isn't steering (carControl.latActive): modeld would not feed it to the
model, and a pulse, repeat or timeout spent then is lost. Once openpilot engages, a fresh pulse goes, unless the driver
has turned the car TURN_STARTED_DEG toward a turn meanwhile. A curb-side turn's re-engage pulse (openpilot steering again, a
fault's hold_off) is bounded as its first pulse is: never with the nearest reading at or past the junction, nor rolling past
CURB_FIRST_PULSE_M by the reading it goes by (2026-11-06 review: openpilot's steering off and back, with no handback and no
take-up, pulsed a started right at 1.95 m and 0.45 m, the late pulse into the exit road's far lanes its first pulse's deadline
is for). None goes (repeat_refused), and turnpilotd skips the turn out loud (desireDropped: its pulse cleared from the model's
queue, its corner cap kept to its pass), as its take-up after a handback does. Standing (a stand inside the deadline) it goes,
as any standing pulse does. That skip is for a car that hasn't been following the first pulse: openpilot off (or turnpilotd
faulted) for REPEAT_BLIP_FRAMES or more since it went, the car turned under OWN_FOLLOW_DEG toward the turn. A blip shorter than
that (one fault frame, a frame or three of latActive gone), or a gap the car came out of turning on the turn, leaves the first
pulse in the model's queue to drive the turn on, as it would have with no gap: none goes, and nothing is said or cleared (the
review of that skip, 2026-11-06: one fault frame or one latActive frame lost 2.5-4.5 m out skipped a right the car was 0-12.9
degrees into on its 7.7 m pulse, its pulse cleared from the model's queue mid-entry and its crosswalk hold gone with the
maneuver; the gaps it is for were 27 and 37 frames, 6 m to 1.95 m and 0.45 m at the 3 m/s cap, the car straight). Rolling, no
re-engage pulse goes for it after that (a fresh edge inside the deadline is the late pulse); from a standstill, as for any turn
under way. A far-side turn's is as before: its arc runs past the junction's node (a 12 m corner tangent across
the road), HEAD's re-engage pulse drove it there, and its first pulse's 10-25 m deadline on it would make lefts drive less.
When a new route replaces the maneuver (RouteFollower.replaced), a turn pulse the car hasn't started turning on is
cleared at once: the old junction may be just ahead, and the pulse would turn the car there.
Nor does a turn's pulse go while the car is short of an opening on the turn's side it may not enter (navd's
maneuverWrongWayM: a one-way toward it, a divided road's near carriageway, its left at the far one; routing/guidance.py:
wrong_way_before), by the furthest reading of the junction (RouteFollower.junction_spread): the model takes a turn desire at
the first opening that way, and from 40 m out, or pulling away from a red light standing at that opening, that is the
one-way, the wrong way (the first drive: a left onto a one-way planned at a red light). The first pulse goes once the car
is WRONG_WAY_PAST_M past the opening, closer in than the lead (turn_skip moves its tooLate deadline in with it); a turn
the model doesn't take from there is missed and rerouted, never driven into the one-way. An autonomous left whose hold
line stands at such an opening (one under about 11 m before its junction) is let go from its line with no desire, its first
pulse past the opening: turnpilotd's go bookkeeping for a pulsed go (its stall watch) isn't armed for that left.
"""
import math

from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE
from openpilot.selfdrive.turnpilot.route_follower import KEEP, LEFT, SAME_TURN_M, TURN, same_bearings

Desire = log.Desire

TURN_DESIRE_LEAD_M = 40.      # gate 1's best lead on the big model: a far-side turn's at turning speed, its most
FAR_LEAD_S = 6.               # s before the junction a far-side turn's first pulse goes below that speed...
FAR_LEAD_MIN_M = 15.          # ...and never later than this (module docstring)
CURB_TURN_PULSE_M = 8.        # m: a curb-side turn's one pulse, nothing before it (module docstring: 1e's clipped rights)
CURB_GO_SLACK_M = 1.          # ...or on a go standing this little further out (by pulse_distance): its own hold line's stand
                              # (scene_policy.HOLD_LINE_M)
CURB_LATEST_PULSE_M = 6.      # ...and at the latest once navd's latest reading has the junction this close (pulse_distance)
CURB_FIRST_PULSE_M = 5.       # ...and never later than this, by that reading (turn_skip's tooLate deadline for it)
KEEP_DESIRE_LEAD_S = 2.5
KEEP_DESIRE_LEAD_MIN_M = 20.
KEEP_DESIRE_PAST_M = 5.       # a keep desire lasts this far past the maneuver point
KEEP_LAST_FRACTION = 0.5      # a keep whose desire hasn't gone out by this share of its lead isn't sent: said (turnpilotd)
DESIRE_TIMEOUT_S = 15.        # moving time a keep desire may be held before it is dropped for good
REPULSE_S = 3.5               # a turn desire pulses again this often on the approach...
ENTRY_M = 10.                 # ...and once more this far before the junction (not within ENTRY_GAP_S of the last)...
ENTRY_GAP_S = 1.5
TURN_STARTED_DEG = 15.        # ...until the car has turned this far toward the maneuver
STEER_IN_TURN_CURV = 0.04     # 1/m: a curb-side turn's one pulse waits only on a steer-in this hard: a turn's (25 m radius),
                              # never a road's bend or a lean's (module docstring: 1d/1e's no-desire steer peaked at 0.038)
APPROACH_M = 20.              # ...and from such a steer-in this close (the nearest reading) the car's turn toward it counts
OWN_TURN_DEG = 10.            # the model's own curb-side turn on its approach, the car this far round since that steer-in...
OWN_FOLLOW_DEG = 3.           # ...or once its pulse waits on it, this far: the car following the model round
REPEAT_BLIP_FRAMES = 6        # frames openpilot off (or turnpilotd faulted) since a curb-side turn's pulse under which a refused
                              # re-engage repeat is a blip, the first pulse still driving the turn: nothing said (module docstring;
                              # 0.3 s, 0.9 m at the 3 m/s cap: the review's blips 1-3 frames, the skip's own cases 27 and 37)
OWN_DROP_MIN_M = 1.           # ...given back to the wait if the model lets go of it short of TURN_STARTED_DEG and this short
STEER_IN_CURV = 0.005         # 1/m: nor while the model steers into the turn this hard (a 200 m radius toward its side)
STANDSTILL_REPULSE_MAX_DEG = 25.  # a pulse as the car pulls away after a standstill goes while it has turned less than this
MAX_PULSES = 5
PULSE_GAP_FRAMES = 3          # frames of none before a repeat pulse
STANDSTILL_SPEED = 0.3        # m/s
STANDSTILL_REARM_S = 2.       # stopped this long (a queue, a red light), the pulse goes again once the car moves
CLEAR_MIN_DEG = 45.           # past the junction, the turn desire is cleared once the car has turned this far...
CLEAR_MARGIN_DEG = 10.        # ...and before it, once it has turned the maneuver's angle less this...
CLEAR_MIN_FRACTION = 0.75     # ...and at least this much of it
CLEAR_WAIT_S = 6.             # past the junction short of CLEAR_MIN_DEG, the turn may get there this long after
CLEAR_FRAMES = 5              # frames CLEAR_DESIRE is published
WRONG_WAY_PAST_M = 2.         # m past an opening it may not enter (maneuverWrongWayM) the car is before its turn may pulse:
                              # the device about 2 m behind the bumper, the bumper into the opening's far lanes
WRONG_WAY_GRACE_M = 3.        # m the car may run past that point before turn_skip calls the first pulse too late

DESIRES = {(TURN, LEFT): Desire.turnLeft, (TURN, -LEFT): Desire.turnRight,
           (KEEP, LEFT): Desire.keepLeft, (KEEP, -LEFT): Desire.keepRight}


def curb_side_turn(maneuver, right_hand_traffic: bool = True) -> bool:
  """A junction turn to the curb side: a right where traffic keeps right, a left where it keeps left (scene_policy.far_side_turn's
  side rule, the other way)."""
  return maneuver is not None and maneuver.kind == TURN and (maneuver.side == LEFT) != right_hand_traffic


def pulse_distance(maneuver, distance: float, far: float | None, right_hand_traffic: bool = True,
                   last: float | None = None) -> float:
  """m to the junction a turn's first pulse is placed by (module docstring), against its lead: for a curb-side turn the
  furthest reading (far), or navd's latest (last) CURB_TURN_PULSE_M - CURB_LATEST_PULSE_M further in, whichever comes first,
  so its pulse goes at CURB_TURN_PULSE_M by the furthest or at CURB_LATEST_PULSE_M by the latest; the nearest (distance) for
  a far-side turn, and for a curb-side one with no furthest given. turn_skip judges a curb-side turn by the same reading."""
  if far is None or not curb_side_turn(maneuver, right_hand_traffic):
    return distance
  d = max(far, distance)
  return d if last is None else min(d, max(last, distance) + CURB_TURN_PULSE_M - CURB_LATEST_PULSE_M)


def turn_lead(maneuver, v_ego: float, right_hand_traffic: bool = True) -> float:
  """m before its junction a turn's first pulse goes (module docstring): CURB_TURN_PULSE_M for a curb-side turn; for a
  far-side one FAR_LEAD_S at v_ego, TURN_DESIRE_LEAD_M at most and FAR_LEAD_MIN_M at least."""
  if curb_side_turn(maneuver, right_hand_traffic):
    return CURB_TURN_PULSE_M
  return min(max(FAR_LEAD_S * max(v_ego, 0.), FAR_LEAD_MIN_M), TURN_DESIRE_LEAD_M)


def wrong_way_wait(maneuver, far: float) -> bool:
  """A turn's pulse waits: an opening on its side the car may not enter (maneuver.wrong_way_m before its junction) is still
  ahead of, or under WRONG_WAY_PAST_M behind, the car far m short of the junction (its furthest reading)."""
  w = getattr(maneuver, 'wrong_way_m', 0.) if maneuver is not None else 0.
  return w > 0. and maneuver.kind == TURN and far > w - WRONG_WAY_PAST_M


def first_pulse_by(maneuver, latest: float) -> float:
  """m before the junction a turn's first pulse must have gone by (turn_skip's tooLate, latest by default), moved in for an
  opening it waits past (wrong_way_wait), less the second it may take to go once the car is past it at turning speed."""
  w = getattr(maneuver, 'wrong_way_m', 0.) if maneuver is not None else 0.
  return min(latest, max(w - WRONG_WAY_PAST_M - WRONG_WAY_GRACE_M, 0.)) if w > 0. else latest


def keep_lead(v_ego: float) -> float:
  return max(KEEP_DESIRE_LEAD_S * v_ego, KEEP_DESIRE_LEAD_MIN_M)


def keep_last(v_ego: float) -> float:
  """m before its point a keep the model can't be sent (no lane line that way) is given up on and said (turnpilotd)."""
  return KEEP_LAST_FRACTION * keep_lead(v_ego)


class DesireTiming:
  def __init__(self):
    self.maneuver_id = -1
    self.clear_frames = 0     # CLEAR_DESIRE goes out while this counts down
    self._pending = None      # [turned deg, side, s] of a passed turn not yet turned CLEAR_MIN_DEG
    self._seen = None         # (maneuver, distance) of the last update: a reroute's same junction carries its approach (_carry)
    self._reset()

  def _reset(self):
    self.active = False       # a desire is being held for the current maneuver
    self.started = False      # the desire went out for this maneuver: latched until passed or timed out
    self.done = False         # passed or timed out: nothing more for this maneuver
    self.pulses = 0
    self._since_pulse = 0.
    self.turned_deg = 0.      # heading change toward the turn since the first pulse
    self.entry = False        # the pulse at ENTRY_M went out
    self._gap = 0
    self._moving_t = 0.
    self._stopped_t = 0.
    self.side = 0             # set once a turn desire went out: the turn to clear once it is done
    self._held = False        # held off while openpilot wasn't steering, since the last pulse
    self._rearmed = False     # re-armed by a standstill, since the last pulse
    self._abandoned = False   # abandon()ed: TurnPilot handed back
    self._held_none = 0       # frames of none while a hold had the car standing, since the last pulse
    self._stall_pulse = False  # stall_pulse() asked for one fresh pulse at the next standing update
    self._steer_wait = False  # a curb-side turn's one pulse is due and waits while the model steers in (module docstring)
    self._wait_t = 0.         # s it has waited
    self._run = False         # the model's turn-scale steer-in on this curb-side turn's approach is on (_approach)...
    self._run_deg = 0.        # ...and the car's turn toward it since that steer-in began
    self._armed = False       # ...which begins only once the model has been seen not steering in, for this maneuver
    self.own = False          # the model took this curb-side turn on its own, no pulse sent: started, never pulsed into
    self.repeat_refused = False  # this curb-side turn's re-engage repeat came due past its first pulse's bounds: none went
                                 # (module docstring), for turnpilotd to say
    self._off_frames = 0      # frames openpilot wasn't steering (or turnpilotd faulted: hold_off), since the last pulse
    self._queued = False      # ...and a refused repeat that was a blip, or with the car following: the first pulse drives on

  def stall_pulse(self) -> bool:
    """One fresh pulse of the turn in progress for a car standing partway round it, whatever it has turned, at the next
    update that stands the car with openpilot steering (turnpilotd's stall watch under TurnPilotAlwaysDrive, ALWAYS_DRIVE.md
    LEFT-21: a new rising edge has restarted the model before). None for a turn not started or done (its clear out: a pulse
    then turns the car on past the exit road). Whether it is asked."""
    if not self.started or self.done or self.side == 0:
      return False
    self._stall_pulse = True
    return True

  def _turn_over(self):
    """The maneuver was passed or replaced: clear its turn desire now, or once the car has turned CLEAR_MIN_DEG."""
    if self.side:
      self._pending = [self.turned_deg, self.side, 0.]
      self.side = 0

  def adopt(self, maneuver, maneuver_id: int, turned_deg: float):
    """A curb-side turn skipped tooLate with no pulse out (turn_skip) that the car has turned TURN_STARTED_DEG into anyway
    since (the model taking it on its own after a mild steer-in; turnpilotd.adopt_skipped): the model's own turn from here
    (own: started, never pulsed into), turned_deg so far, so its clear at its angle and turnpilotd's in-turn watches, caps and
    holds all apply to it, as to one taken as its own on its approach (module docstring)."""
    if maneuver_id != self.maneuver_id:
      self.maneuver_id = maneuver_id
      self._reset()
    self.started = self.own = self.entry = True
    self.done = self.active = self._steer_wait = self._run = False
    self.side = maneuver.side
    self.turned_deg = max(self.turned_deg, turned_deg)

  def abandon(self):
    """TurnPilot handed back: no more desires for this maneuver. A turn desire already out is cleared from the model's
    queue once the car has turned CLEAR_MIN_DEG, as for a passed turn; the caller sends None as the maneuver from now on."""
    self._turn_over()
    self.done, self.active = True, False
    self._abandoned = True

  @property
  def approach_deg(self) -> float:
    """deg the car has turned toward a curb-side turn not started since the model's turn-scale steer-in into it began on its
    approach (_approach; module docstring), 0 with none on."""
    return self._run_deg if self._run and not self.started else 0.

  @property
  def held_off(self) -> bool:
    """The desire was dropped while openpilot wasn't steering, and no pulse has gone since."""
    return self._held

  def resume(self):
    """TurnPilot acts again after a handback: a maneuver abandoned before any desire went out for it is taken up again,
    as on a fresh engage. One whose desire went out stays done (turnpilotd decides about a held turn)."""
    if self._abandoned and not self.started:
      self.done = self._abandoned = False

  def hold_off(self):
    """The desire was dropped for a frame (turnpilotd faulted): as while openpilot wasn't steering, it pulses again only
    if the car hasn't turned TURN_STARTED_DEG. Each fault frame counts as one openpilot wasn't steering (REPEAT_BLIP_FRAMES)."""
    self.active, self._held, self._gap = False, True, 0
    self._off_frames += 1

  def track_yaw(self, yaw_rate: float, dt: float):
    """A frame turnpilotd faulted on, with this turn's desire out: the car's turn toward it still counts, so its desire is
    cleared at its angle once TurnPilot runs again, as if there had been no fault."""
    if self.started and self.side and not self.done:
      self.turned_deg += math.degrees(yaw_rate * dt) * self.side

  def carry_clear(self, old: "DesireTiming"):
    """Take over old's turn desire to clear (turnpilotd replaced it: a fault or TurnPilot off, its turn over meanwhile):
    cleared from the model's queue now if the car has turned CLEAR_MIN_DEG, else once it has, as for a passed turn."""
    old._turn_over()
    self._pending, self.clear_frames = old._pending, old.clear_frames
    if self._pending is not None and self._pending[0] >= CLEAR_MIN_DEG:
      self._clear()

  def cancel(self):
    """A turn skipped with its desire out but not yet turned on (turnpilotd: a dropped turn too close to pulse again):
    its desire comes out of the model's queue now, and nothing more goes for it."""
    self._clear()
    self.done, self.active = True, False

  def withdraw(self):
    """A turn's go withdrawn short of its junction with its pulse out (turnpilotd: the autonomous left's go withdrawn to its
    own hold, under TURN_STARTED_DEG into it): its pulse comes out of the model's queue now (CLEAR_DESIRE), so the model
    plans no turn while the hold brakes the car, and none goes while it is held; the next go pulses a fresh edge, as after a
    standstill (STANDSTILL_REPULSE_MAX_DEG). The turn stays the maneuver's (started, side and done kept)."""
    self.clear_frames, self._pending = CLEAR_FRAMES, None
    self.active, self._gap, self._rearmed = False, 0, True

  def _clear(self):
    self.clear_frames, self._pending, self.side = CLEAR_FRAMES, None, 0

  def _approach(self, maneuver, distance: float, v_ego: float, curvature: float, yaw_rate: float, dt: float, lat_active: bool):
    """A curb-side turn not started: the model's turn-scale steer-in on its approach (module docstring). From the first frame
    inside APPROACH_M that the model steers into the turn STEER_IN_TURN_CURV or more, rolling with openpilot steering, the car's
    turn toward it counts (_run_deg), until the model lets go of it (under STEER_IN_CURV) or openpilot stops steering. A
    steer-in already under way when the maneuver came up (the tail of the turn before it, a right 15 m on) is not this turn's:
    one begins only once the model has been seen not steering in for it (_armed)."""
    toward = -maneuver.side * curvature  # desiredCurvature is positive right, side positive left
    self._armed = self._armed or toward < STEER_IN_CURV
    if not lat_active or distance > APPROACH_M or toward < STEER_IN_CURV:
      self._run, self._run_deg = False, 0.
      return
    if not self._run and self._armed and toward >= STEER_IN_TURN_CURV and v_ego >= STANDSTILL_SPEED:
      self._run = True
    if self._run:
      self._run_deg += math.degrees(yaw_rate * dt) * maneuver.side

  def _carry(self, maneuver, distance: float, right_hand_traffic: bool):
    """The approach of the last maneuver, for a new one that is its junction again (a reroute's first step: RouteFollower's span
    carry, same_bearings within SAME_TURN_M): a curb-side turn not started, so the model's steer-in into it and the car's turn
    toward it since then still count (module docstring). None otherwise."""
    if self._seen is None or self.started:
      return None
    old, old_d = self._seen
    if not (curb_side_turn(old, right_hand_traffic) and curb_side_turn(maneuver, right_hand_traffic) and old.side == maneuver.side and
            same_bearings(old, maneuver) and abs(distance - old_d) <= SAME_TURN_M):
      return None
    return self._run, self._run_deg, self._steer_wait, self._wait_t, self._armed

  def update(self, maneuver, distance: float, maneuver_id: int, passed: bool, v_ego: float,
             steering_angle_deg: float, keep_left_ok: bool, keep_right_ok: bool, dt: float, yaw_rate: float = 0.,
             lat_active: bool = True, replaced: bool = False, may_start: bool = True, go: bool = False,
             holding: bool = False, far: float | None = None, curvature: float = 0., right_hand_traffic: bool = True,
             last: float | None = None) -> int:
    """The desire to publish, CLEAR_DESIRE once a turn is done. maneuver is RouteFollower.upcoming (None when there is
    none); keep_left_ok/keep_right_ok: the model sees a lane line and road that way (lane_planner.ModelLanes);
    yaw_rate: rad/s, left positive; lat_active: carControl.latActive; replaced: RouteFollower.replaced;
    may_start: the car can still make the turn (turn_skip.py); no turn pulse starts while it can't; go: a hold let go of
    the car at standstill (turnpilotd): the turn pulses again now, not once the car moves, since the model won't pull
    away from a standstill with its pulse forgotten; holding: a hold has the car (shouldStop): standing, the turn desire
    drops to none (its edge stays in the model's queue), so the go's pulse is a fresh edge in the very frame it lets go;
    far: the furthest reading of the junction's distance (distance plus RouteFollower.junction_spread; distance when not
    given), which a turn's pulse waits past an opening it may not enter by (wrong_way_wait) and a curb-side turn's one pulse
    goes by (pulse_distance); last: navd's latest reading of it, dead-reckoned (RouteFollower.junction_latest; none when not
    given), by which a curb-side turn's pulse is due at the latest (pulse_distance, CURB_LATEST_PULSE_M); curvature: modelV2's
    action.desiredCurvature (1/m, positive right): no repeat or entry pulse while it steers into the turn (STEER_IN_CURV), nor
    a curb-side turn's one pulse with the car rolling while it steers in STEER_IN_TURN_CURV or more (it waits; module
    docstring); right_hand_traffic:
    the side traffic keeps to, for where a turn's first pulse goes (turn_lead: a curb-side turn's one pulse at 8 m). No turn's
    first pulse goes past the junction by distance, the nearest reading (module docstring)."""
    if maneuver_id != self.maneuver_id:
      if replaced and self.side and self.turned_deg < TURN_STARTED_DEG:
        self._clear()
      else:
        self._turn_over()
    if self._pending is not None:
      self._pending[0] += math.degrees(yaw_rate * dt) * self._pending[1]
      self._pending[2] += dt
      if self._pending[0] >= CLEAR_MIN_DEG:
        self._clear()
      elif self._pending[2] > CLEAR_WAIT_S:
        self._pending = None
    may_start = may_start and not wrong_way_wait(maneuver, distance if far is None else far)
    desire = self._update(maneuver, distance, maneuver_id, passed, v_ego, keep_left_ok, keep_right_ok, dt, yaw_rate, lat_active,
                          may_start, go, holding, curvature, right_hand_traffic, far, last)
    if desire != Desire.none:
      self._pending = None  # the next maneuver's pulse is in the model's queue now
    if self.clear_frames > 0:
      self.clear_frames -= 1
      return CLEAR_DESIRE
    return desire

  def _update(self, maneuver, distance: float, maneuver_id: int, passed: bool, v_ego: float,
              keep_left_ok: bool, keep_right_ok: bool, dt: float, yaw_rate: float, lat_active: bool, may_start: bool,
              go: bool = False, holding: bool = False, curvature: float = 0., right_hand_traffic: bool = True,
              far: float | None = None, last: float | None = None) -> int:
    if maneuver_id != self.maneuver_id:
      carry = self._carry(maneuver, distance, right_hand_traffic) if maneuver is not None else None
      self.maneuver_id = maneuver_id
      self._reset()
      if carry is not None:  # a reroute's same junction: the model's steer-in into it, and the car's turn since, go on
        self._run, self._run_deg, self._steer_wait, self._wait_t, self._armed = carry
      passed = False  # that was the maneuver before this one
    self._seen = (maneuver, distance) if maneuver is not None else None
    if passed or maneuver is None:
      self.done = self.done or passed
      self.active = False
      if passed:
        self._turn_over()
      return Desire.none
    desire = DESIRES.get((maneuver.kind, maneuver.side), Desire.none)
    if desire == Desire.none or self.done:
      return Desire.none

    curb = curb_side_turn(maneuver, right_hand_traffic)
    if curb and not self.started:
      self._approach(maneuver, distance, v_ego, curvature, yaw_rate, dt, lat_active)
    pulse_d = distance
    if maneuver.kind == TURN:  # a curb-side turn's one pulse by the furthest reading, at the latest by the latest (module docstring)
      lead = turn_lead(maneuver, v_ego, right_hand_traffic)
      pulse_d = pulse_distance(maneuver, distance, far, right_hand_traffic, last)
      # ...and a go standing at its junction's own hold line, a little outside the pulse point by that same reading, is that
      # pulse: never by the nearest alone, which keeps one reading 3-5 m short (2026-11-07 review: a go 10-13 m out pulsed
      # standing; module docstring)
      line_go = go and v_ego < STANDSTILL_SPEED and curb and pulse_d < CURB_TURN_PULSE_M + CURB_GO_SLACK_M
      # ...and one the model has the car OWN_TURN_DEG into by itself on its approach is under way, wherever (module docstring)
      want = self.started or line_go or pulse_d < lead or (curb and self._run and self._run_deg >= OWN_TURN_DEG)
    else:
      if distance < -KEEP_DESIRE_PAST_M:
        self.done, self.active = True, False
        return Desire.none
      want = self.started or (distance < keep_lead(v_ego) and (keep_left_ok if maneuver.side == LEFT else keep_right_ok))
    if not want:
      self.active = False
      return Desire.none

    turn = maneuver.kind == TURN
    if not lat_active:
      self.active, self._held, self._gap = False, True, 0
      self._off_frames += 1
    # a stop after a re-arm is still in the turn (a curb-side turn's approach counts the car's turn toward it from the model's
    # turn-scale steer-in: _approach, module docstring)
    if turn and (self.active or self._held or self.started):
      self.turned_deg += math.degrees(yaw_rate * dt) * maneuver.side
      if self.turned_deg >= max(maneuver.angle_deg - CLEAR_MARGIN_DEG, CLEAR_MIN_FRACTION * maneuver.angle_deg):
        if self.side:
          self._clear()
        self.done, self.active = True, False
        return Desire.none
    if not lat_active:
      return Desire.none

    if v_ego < STANDSTILL_SPEED:
      self._stopped_t += dt
      if self._stall_pulse and turn and self.started:
        # stall_pulse(): a fresh edge now, the standstill clock started over (STANDSTILL_REARM_S of it, then none again)
        self._stall_pulse, self._stopped_t = False, 0.
        self.pulses += 1
        self._since_pulse = 0.
        if self.active:
          self._gap = PULSE_GAP_FRAMES  # out still: none for the gap, then the desire again
        else:
          self.active, self._rearmed, self._held, self._held_none = True, False, False, 0
          self._off_frames, self._queued = 0, False
          return desire
      if go and turn and not self.started and may_start:
        # a turn whose first pulse is its go (an unprotected left taken from its hold line with no pulse on the approach,
        # turnpilotd): now, standing, and the car gets STANDSTILL_REARM_S to move off on it
        self._stopped_t = 0.
      elif holding and turn and self.started:
        # held standing: none until the go, which then pulses a fresh edge at once; the edge already sent stays queued
        self.active, self._rearmed, self._gap = False, True, 0
        self._held_none += 1
        return Desire.none
      elif holding and turn:
        # nor a first pulse while a hold has the car standing: it goes on the hold's go, the fresh edge in the plan that lets
        # the car go (2026-11-06 review: a right handed back at its stop sign 7.25 m out, its one 8 m pulse due where the car
        # stood, got a fresh turnRight on the handback frame with the car held standing, into the model's 6.6 s queue)
        self.active = False
        return Desire.none
      if go and turn and self.started and self.turned_deg < STANDSTILL_REPULSE_MAX_DEG and may_start and \
         self._held_none and self._held_none < PULSE_GAP_FRAMES:
        self._gap = PULSE_GAP_FRAMES - self._held_none  # held under PULSE_GAP_FRAMES: the drop lasts that long all the same
        self._held_none = 0
      elif go and turn and self.started and self.turned_deg < STANDSTILL_REPULSE_MAX_DEG and may_start:
        # a hold let go of the car here: pulse now, at standstill (GO_PULSE). A fresh one if the standstill re-armed the
        # turn, else a repeat; then the standstill clock starts over, so the car gets STANDSTILL_REARM_S to move off on it
        self._stopped_t = 0.
        if self.active:
          self._gap, self._since_pulse = PULSE_GAP_FRAMES, 0.
          self.pulses += 1
          self.entry = self.entry or distance < ENTRY_M
      elif self._stopped_t > STANDSTILL_REARM_S:
        self.active = False  # the model has forgotten the pulse by now; send it again once the car moves
        self._rearmed = True
        return Desire.none
    else:
      self._stopped_t, self._stall_pulse = 0., False  # a stall pulse is for a standing car only
      self._moving_t += dt
      if self._moving_t > DESIRE_TIMEOUT_S and not turn:
        self.done, self.active = True, False
        return Desire.none

    steering_in = turn and -maneuver.side * curvature >= STEER_IN_CURV  # desiredCurvature is positive right, side positive left
    if self.active and turn:
      self._since_pulse += dt
      entry = not self.entry and distance < ENTRY_M and self._since_pulse >= ENTRY_GAP_S and not steering_in
      # used up once it can go: crossing ENTRY_M within ENTRY_GAP_S of a repeat, or while the model steers in, it waits and
      # then goes
      self.entry = self.entry or entry
      if (entry or (self._since_pulse >= REPULSE_S and distance > ENTRY_M)) and self.turned_deg < TURN_STARTED_DEG and \
         self.pulses < MAX_PULSES and may_start and not steering_in:
        self._gap, self._since_pulse = PULSE_GAP_FRAMES, 0.
        self.pulses += 1
    if self._gap > 0:
      self._gap -= 1
      return Desire.none

    if not self.active:
      if curb and self.own and not self._rearmed and may_start and not steering_in and v_ego >= STANDSTILL_SPEED and \
         self.turned_deg < TURN_STARTED_DEG and distance >= OWN_DROP_MIN_M:
        # the model let go of its own turn short of TURN_STARTED_DEG, short of the junction: not its own after all, back to
        # the wait, its steer-in over (module docstring)
        cloudlog.event("turnpilot.curb_turn_own_dropped", distance=round(distance, 1), pulse_distance=round(pulse_d, 1),
                       turned_deg=round(self.turned_deg, 1), curvature=round(curvature, 4), v_ego=round(v_ego, 2))
        self.started, self.own, self.side, self.entry, self._steer_wait = False, False, 0, False, True
        self._run, self._run_deg = False, 0.
      if curb and not self.started and may_start:
        if self._steer_wait:
          self._wait_t += dt
        if self._run and (self._run_deg >= OWN_TURN_DEG or (self._steer_wait and self._run_deg >= OWN_FOLLOW_DEG)):
          # the model has the car turning into it on its own, OWN_TURN_DEG round on its approach or following it OWN_FOLLOW_DEG
          # round while its pulse waits: under way, so no pulse goes into it, and it counts as started (module docstring)
          self.started, self.own, self.entry, self.side, self._steer_wait = True, True, True, maneuver.side, False
          self.turned_deg = max(self.turned_deg, self._run_deg)
          cloudlog.event("turnpilot.curb_turn_own", distance=round(distance, 1), pulse_distance=round(pulse_d, 1),
                         turned_deg=round(self.turned_deg, 1), waited_s=round(self._wait_t, 2), v_ego=round(v_ego, 2),
                         curvature=round(curvature, 4))
          return Desire.none
        if turn and -maneuver.side * curvature >= STEER_IN_TURN_CURV and v_ego >= STANDSTILL_SPEED and not go and distance >= 0.:
          # its one pulse is due with the model already steering in at a turn's scale, the car rolling: it waits (module
          # docstring)
          if not self._steer_wait:
            cloudlog.event("turnpilot.curb_pulse_wait", distance=round(distance, 1), pulse_distance=round(pulse_d, 1),
                           curvature=round(curvature, 4), turned_deg=round(self._run_deg, 1), v_ego=round(v_ego, 2))
          self._steer_wait = True
          return Desire.none
        if self._steer_wait and v_ego >= STANDSTILL_SPEED and pulse_d < first_pulse_by(maneuver, CURB_FIRST_PULSE_M):
          # past its deadline still waiting, rolling: none goes late, turn_skip says it; standing (braked to a line inside the
          # deadline, its go) it goes as any standing pulse does (module docstring)
          return Desire.none
      started_deg = STANDSTILL_REPULSE_MAX_DEG if self._rearmed and not self._held else TURN_STARTED_DEG
      # a curb-side turn's first pulse counts the car's turn toward it since the model's turn-scale steer-in too (_approach)
      turned = max(self.turned_deg, self._run_deg) if curb and not self.started else self.turned_deg
      if turn and (not may_start or turned >= started_deg):
        return Desire.none  # a pulse in the turn (re-engaging, pulling away) turns the car on past the exit road
      if turn and not self.started and distance < 0.:
        # a first pulse past the junction (by the nearest, one reading alone included, whatever the furthest and the latest say:
        # module docstring) turns the car into the curb or the exit road's far lanes: none, and turn_skip skips the turn out
        # loud as that reading crosses the junction unpulsed
        return Desire.none
      if turn and self.own and not self._rearmed:
        return Desire.none  # the model's own turn gets no pulse (a re-engage in it), but from a standstill as any turn under way
      if curb and self.started and self._held and \
         (distance <= 0. or (v_ego >= STANDSTILL_SPEED and pulse_d < first_pulse_by(maneuver, CURB_FIRST_PULSE_M))):
        # a started curb-side turn's re-engage repeat (openpilot steering again, a fault's hold_off) is bounded as its first
        # pulse is (module docstring): never once the nearest reading has the junction at or behind the car, nor rolling past
        # the pulse's deadline by the reading it goes by. None goes, and turnpilotd says so (repeat_refused)...
        if not self.repeat_refused and (self._off_frames < REPEAT_BLIP_FRAMES or self.turned_deg >= OWN_FOLLOW_DEG):
          # ...but for a blip, or a car following the first pulse round: that pulse, still in the model's queue, drives the
          # turn on as with no gap, nothing said or cleared (module docstring); rolling, no re-engage edge for it after this.
          # Judged on the frame openpilot steers again: a refusal stands (turnpilotd skips and clears it on that frame)
          if not self._queued:
            cloudlog.event("turnpilot.curb_repeat_spared", distance=round(distance, 1), pulse_distance=round(pulse_d, 1),
                           turned_deg=round(self.turned_deg, 1), off_frames=self._off_frames, v_ego=round(v_ego, 2))
          self._held, self._queued = False, True
          return Desire.none
        if not self.repeat_refused:
          self.repeat_refused = True
          cloudlog.event("turnpilot.curb_repeat_refused", distance=round(distance, 1), pulse_distance=round(pulse_d, 1),
                         turned_deg=round(self.turned_deg, 1), v_ego=round(v_ego, 2))
        return Desire.none
      if turn and self._queued and not self._rearmed:
        return Desire.none  # a spared repeat's turn: its first pulse drives it, and from a standstill as any turn under way
      if self._steer_wait:  # the model let go of the corner, the car short of OWN_FOLLOW_DEG round: its pulse goes now
        cloudlog.event("turnpilot.curb_pulse_waited", distance=round(distance, 1), pulse_distance=round(pulse_d, 1),
                       turned_deg=round(turned, 1), waited_s=round(self._wait_t, 2), v_ego=round(v_ego, 2),
                       curvature=round(curvature, 4))
        self._steer_wait = False
      if curb and not self.started:
        self.turned_deg = turned  # from the steer-in it went on (module docstring)
      self.active = self.started = True
      if turn and distance < ENTRY_M:
        self.entry = True  # a pulse inside ENTRY_M (on engaging, pulling away) is the entry pulse
      self._held = self._rearmed = self._queued = False
      self._held_none = self._off_frames = 0
      self.pulses += 1
      self._since_pulse = 0.
      if turn:
        self.side = maneuver.side
    return desire
