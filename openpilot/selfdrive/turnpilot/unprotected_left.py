"""The autonomous unprotected left (DESIGN phase 4 section 2, G4; CRITIQUE REQUIRED 2-5): a routed turn across the oncoming
traffic of the road ahead (map_controls.unprotected_turn), taken from its hold line with no gas tap.

Kellen, 2026-10-28: "No it has to ship taking the turn", and for this version no intervention. So while the car can judge
the junction it waits at the hold line for a gap it can certify, as a careful driver does, for as long as it takes, the car
held, never let go and never handed back for the wait (Kellen, 2026-11-07: "no, no handbacks"; test builds are "bleeding
edge, not maximal caution", a safety driver on every drive). A handback (UNDECIDED: leftUndecidedTakeControl, the handback
chime, the car held) is only for a fault: road camera frames stale, the junction's position unknown, the driver's lamp the
other way (driverSignal). The car standing yawed past what the road camera frames of the band (YAW_VIEW_DEG, after a
withdrawn go) is judged at its measured heading on what it does frame, said once with a chime (turnpilot.left_yaw_view,
LONG_WAIT_REASON['yaw']): the 'yaw' handback until 2026-11-08. Until 2026-11-07 the waits ended in one too,
the car held: no gap in STAND_UNDECIDED_S (noGap; standing where the tracker says something stands in the band or the exit
zones; DESIGN's UNDECIDED row, owner decision (ii), 2026-10-30), the far crop's boxes alone for FAR_BLOCKED_UNDECIDED_S
(farBlocked), the view still blocked BLOCKED_UNDECIDED_S after the full creep (blocked), a creep not made in CREEP_STALL_S
(creepStall), no lamp on its side for LEFT_SIGNAL_WAIT_S (unsignalled), the light read unknown LIGHT_UNKNOWN_S or red past
LIGHT_RED_MAX_S (light), and a red or yellow read after it stood on a green (lightChange). Each is now a wait at the line,
logged once a wait as its old bound passes (_said: turnpilot.left_no_gap, left_far_wait, left_view_wait, left_unsignalled,
left_light_wait, left_light_changed) with its status line saying what the car waits for (LONG_WAIT_REASON), a car the
detector sees still holding the left. Never an endless silent wait (skeptic g4c: 240 s on 'waitingForGap' alone): the
line says why, the log has it once, and the safety driver supervises.
A missing frame is never clear. Never a handback for what the camera can't see (Kellen, 2026-11-07: "no, no handbacks. that
kinda thing is for the final public build, we just NEED to be ready to test"; a safety driver supervises every drive): an
R_req past the camera's reach, by day or after dark, or past where the opposing road bends out of the band, is judged on the
band the camera covers (out to the smaller of its reach, R_req and the straight run), a car seen there holding the left. A
left in a lane it doesn't leave from (LC-03) is skipped out loud and rerouted by turnpilotd (Kellen, 2026-11-08, drive 29:
never taken across a lane from the wrong one; driven from the lane it was in from 2026-11-07, LN-5), but for one LC-03
flags only once the car stands held at its line (wrong_lane, crossed_lanes, a car beside holding it: LS-1, LEFTS-2).
At a mapped signal the hold is in the junction's mouth (map_controls.SIGNAL_LEFT_HOLD_M, LeftInput.line_m; drive 29: every
left at a green ball was held 9 m short, the driver driving them all): the approach is the model's, the signal's own control
stops the car at its line for a red or yellow read, and on a green, read or not, the car rolls up to the mouth and the gap
rule judges the oncoming lanes from there. Its stop line is behind it there, so a red read while it waits is a red in the
junction (LS-4's words), and the car clears the junction on its gap (2026-11-08). turnpilot.left_reach logs, once per hold at
the line, how far that band reached against R_req and what it fell short for. The paragraphs below that say 'hands back'
for range, night or curve describe the build before.

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
A left whose R_req is past the detector's proven range (TurnPilotLeftRangeM; unset, unmeasured: none, so every left; never
past FULL_PASS_RANGE_M, what the letterboxed full pass can box at all: turnpilotd caps it there, and past it the band is the
far slot's, the native schedule's at TurnPilotLeftRangeM or a measured far crop's) is never gone blind: it is driven to its
hold line like any other, and there, judged from where the car stands, it hands back loudly with the car held (undecided,
'range'), never skipped or routed around (Kellen, 2026-11-06: "drive to the line; go where the camera covers the oncoming
road, else hold and hand back loudly; never route around or skip a left for range").
Until then route time refused it (route_range, turn_skip's unprotectedRange) 58-394 m out, before the car could judge
anything, and navd rerouted past the junction onto a route with another left: the 11-06 drives skipped five lefts while
engaged (unprotectedLeft 'skipped' on 2710 frames, never waitGap, creep or go), Chase's "kept wanting to turn left", and a
lane change the driver made toward one of them with no turn behind it ("tried a left and went straight", 1e seg 6).
route_range still says, for the logs, what route time would take R_req as where the car may stand (reach_at_line's setback:
the junction's mapped stop line's, else ROUTE_REACH_MARGIN_M, UNCERTAIN_M of readings' spread: BUILD_FINDINGS D4).
After dark (LeftInput.dark: turnpilotd's darkness.DarkFilter on the road camera's exposure) a left past NIGHT_LEFT_RANGE_M,
what the detector was seen to reach at night (1e: raw vehicle boxes p99 50 m, max 78 m; oncoming tracks first published at
p90 28-53 m), rolled up to its line and handed back there ('night') until 2026-11-07. No night-only gating since (Kellen:
"why the hell do we care about dark? ... the streets are very lit up"): the band is judged at night as by day, and 'night'
is only reach_fault's label in turnpilot.left_reach's short_of.
The far crop (DESIGN phase 4 G5; modeld's FarCrop, tracker.update_far) reaches further: a native-resolution window of the
road frame, aimed (far_aim) at the band's bearings past the full pass's range. While it is on (far_live) its own proven
range (TurnPilotLeftFarRangeM, unset: unmeasured, none) extends the reach, the full pass's measured (it covers the band only
past that range). An R_req past the full pass's range (far_needed)
is then judged on its runs too: one covering those bearings (far_covers, its cover_deg) every FAR_FRESH_S or the window
starts over, STALE_UNDECIDED_S without one a fault (farStale), and any vehicle box of its in the band blocks at any range
(at 230-280 m one pixel of box bottom is 40-56 m and 0.15 deg of pitch is NEAR_PX's whole margin: it never clears a car by
its range). A far slot that runs but can't hold FAR_FRESH_S (1-3 runs a second: a model 2.5-3 ms slower, review 2026-11-07
R2) is behind (far_behind) once the full pass's band has been clear a whole window with the far runs' age alone holding it:
judged from then on that band, as with no far slot, a far box in the band within FAR_BLOCK_RECENT_S still holding it, until
its runs hold FAR_FRESH_S for FAR_BEHIND_CLEAR_S (before, an empty road's left waited at its line forever on them, saying
"Waiting for a gap", where one that didn't run went at once). So waiting on them alone, no band box of a road run within
the full pass's range (no traffic reaching it) and nothing else holding the car, for FAR_BLOCKED_UNDECIDED_S (a far box
FAR_BLOCK_RECENT_S apart keeps the count going) is
said (left_far_wait, its status line), never a handback since 2026-11-07: a car the detector sees holds the left, standing or
coming, and the left keeps waiting for it. Not needed, a far box blocks as the full pass's do. A full pass box
blocks whatever the far runs say. Past both ranges, the far range unmeasured, or the far crop off, it is undecided, saying
which (range, farUnmeasured, farOff; said alike to the driver: the car can't see oncoming far enough), or night after dark.
The band assumes the opposing road straight on from our approach out to R_req: navd's map check of it (straight_m:
routing/controls.py's opposing_straight, the road coming in found on that line every 25 m) must reach R_req from the
junction, or the left is never gone: driven to its hold, it is undecided ('curve') there, the car held; not known is not
straight (skeptic G4: on a road curving away from the turn the band missed a car 80-135 m up).
Nothing here checks the junction's own sightline (a crest, a hedge short of R_req): the demo junction is chosen by eye for
a clear sightline of R_req or more. Held
short of its line (the latch reaches LATCH_NEAR_M short), where the longer crossing puts R_req past the range, the car rolls
up to the line first (ROLL_UP_M; undecided only after ROLL_UP_STALL_S standing in it).
The window (CLEAR_WINDOW_S) counts only road camera runs, from the hold (hold, then judge: never carried from the approach):
a run gap over ROAD_FRESH_S, the scene's holds, an exit crosswalk or view not clear, another of TurnPilot's stops holding the
car at or before the junction (a signal's red, a stop sign: other_hold), or its junction's signal not read green, start it
over (DESIGN G4 go condition 5; skeptic G4). A red or yellow any source reads at the junction (map_controls.turn_light: the
camera's in shadow too, since keeping the car where it stands never brakes it) keeps it waiting, a signal mapped there or not;
at a signal the go needs a green (SIGNAL_NEEDS_GREEN): the left doesn't check the cross traffic, and a model that ran the red
leaves the car at its line in the junction with the oncoming lanes empty at their own red. A green the camera's shadow reader
alone reads lets this go release TurnPilot's latch only: plannerd's lift needs an accepted source's green (turnpilotd.certify_go,
map_controls.release_light), so the model's own red-light hold stays and a shadow green never removes a stop (the light net may
only add one). A signal mapped there with no colour read is 'none' to the left (turn_light, Kellen, 2026-11-07: an unread light
is no reason to wait), so SIGNAL_NEEDS_GREEN holds only where navd can't say whether a signal is there ('unknown'). A red or
yellow read keeps the left waiting at its line for as long as it is read, the light changing from the green it waited on too
(lightChange): held where it stands, short of the oncoming lanes, its window starting over; but one first read with the car
past its line clears the junction on its gap (below: 2026-11-08). A red or yellow that raised
TurnPilot's own signal stop there (the camera's add-only, the car's only source, or an accepted source's) stays the left's
light once no longer read, for as long as that stop is in force (turn_light; a light gone unread keeps a stop): gone on the
green that releases it (a steady camera green for the camera's, an accepted one for an accepted red's). At a junction the map
has no signal for, where no such stop is made (the left's own hold is the junction's), a red or yellow the camera acting or
an accepted source reads there is latched the same (map_controls._left_red_latch, review 2026-11-07 HR-1: it was forgotten
the moment it went unread, and the left went 2.6-3.5 s later on the band empty at its own red, lifted over the model's red
hold), until CAMERA_GREEN_STEADY_S of green there. Only a red the camera's shadow reader alone reads (it adds no stop
anywhere) is gone on the gap once no longer read. Until 2026-11-07 that change, no read for LIGHT_UNKNOWN_S and a red past
LIGHT_RED_MAX_S handed back; now each is said once (left_light_changed, left_light_wait) and waited through. A red held with
nothing read (LeftInput.light_read false: the stop in force, the latch) is said as a through signal's light unseen is,
SIGNAL_UNSEEN_S from the loss, its line SIGNAL_UNSEEN_REASON and one chime (left_light_unseen; review 2026-11-07 AL-2: it
was "Waiting for the light" and silent to LIGHT_RED_MAX_S from the hold, 124 s after the loss); LIGHT_RED_MAX_S is a red still
read. A red or yellow first read with the car past its line (in a signal's junction mouth, or crept into the junction for a
view) is said and heard once, and the car clears the junction as a driver does (Kellen, 2026-11-08; held where it stood
until then): the light is no hold of the left's for that spell (LeftDecision.clearing: turnpilotd lifts its go over the
light and the model's red hold), the go on its gap, a car seen in the band holding it until it passes. "Light changed:
clearing the junction" after a green it stood on, "Red light: clearing the junction" on a light it never read green (LS-4,
LEFTS-3). A red read with the car short of its line, or at it, holds it there for the green as before. Since the review of
2026-11-08 only a car that stood past the line (the mapped signal's own: signal_line_m) on a light not red before the spell
clears (a steady green read there, or CLEAR_STAND_S with no red read); a red first read as it rolls in holds it where its
hold is, for the green; and the clearing go waits for the cross road too (cross_closing: nothing closing within the time it
needs to be out; with no wide camera run, no view of it, a plain-words wait in the mouth, never a handback).
Where plannerd wouldn't lift the go over the model's own hold (turnpilotd.left_lift_refused: at a signal with no accepted
green, which on the car is every signal; where the camera boxes a light with none mapped, HR-2; too near the oncoming lanes
for the lift's nudge; TurnPilotGoOverride off), the go only lets TurnPilot's latch go, so the model has to pull away itself:
there the model's own stop (modelV2.action.shouldStop: a red it sees; LeftInput.model_holds; until 2026-11-08 its takeover
predicate not held, which kept the window from counting at a green where the model waits for the turn the go pulses: drive
29, 461-467 s) holds the left at its line, its window starting over, its line the light's ("Waiting at the light") where a
light is known or boxed at the junction, else
"Waiting for the car to pull away" (a head boxed on the approach alone: LEFTS-4), and its go comes once the model's plan
moves, with the model (review 2026-11-07, LS-2: the go came at the model's red and cantStartTakeControl held the car 1 s later
until a disengage, at every mapped signal on the car). A go of the hold the model then didn't follow is retried the same
way, never bounded, each retry silent and saying what holds the car until the car moves on it, then said and heard as a go
(AL-1: every retry chimed "Turning" on a car that never moved, every 3.85 s).

The view is blocked by an occluder: a velocity-trusted vehicle track standing still in the junction (the opposing
left-turner, a parked truck) whose box hides the oncoming lanes beyond it out to R_req. Its own box isn't traffic (it
would never clear): the raw box overlapping its box in the same camera, or within MATCH_LATERAL_M of it across the ray.
Blocked, with the band otherwise clear, the car creeps (map_controls: CREEP_SPEED, CREEP_ACCEL) up to
CREEP_MAX_M past its line, never nearer than CREEP_SHORT_M short of the oncoming lanes (ONCOMING_ENTRY_M), and stops
creeping where the view clears or a vehicle comes into the band. The exit crosswalk and the junction box (a car standing
in the exit lane would leave the car in the oncoming lanes) must be clear too, and the scene holding nothing.

The hold line is at the nearest of navd's readings of the junction (route_follower: JunctionSpan); the path, the conflict
point and the exit zones take the furthest (spread_m), so a car stopped early never certifies a shorter crossing than it has.
The go (map_controls releases its latch, cause gap; turnpilotd pulses the turn and drives it under the left's post-go
profile): the road camera only from the go to the commit (CRITIQUE REQUIRED 3; the wide camera one run in WIDE_EVERY
while held, for the exit crosswalk). Until the car can no longer stop short of the oncoming lanes on the comfortable curve
(scene_policy.committed_past_line), every road run is checked again for the remaining clear time, and at its first motion
(STANDSTILL_SPEED: CRITIQUE REQUIRED 4) with the latency gone: a car in the band then withdraws the go, the car stops
where the comfortable curve can (oncomingHolding, a chime) and waits again, the go's pulse out of the model's queue as
it is withdrawn (turnpilotd.clear_withdrawn_left), a fresh one at the next go; standing where it was held, it can always
stay. So does anything else the go was certified on no longer holding (the exit crosswalk, the view, another hold, the
light gone red or yellow): a go never outlives them to run stale when they let go. A person overruling the car's left (the
driver's lamp the other way, hazards, our lamp put out: car_signals.py, DESIGN phase4/blinker 6.6) withdraws it the same
while it can (withdrawable: a frame on, VETO_MARGIN_M to spare), and hands back (undecided, driverSignal), the car held
where the comfortable curve stops it, short of the oncoming lanes. It is committed only by its own go and motion (or the
driver's release of the junction, driving it), never by a reading: a scene commit before its go, the car rolling up to its
line or standing held, is a 'position' fault.
Committed, nothing here ever stops the car (a pedestrian in the path still does: scene_policy); committed short of a launch
after a slow one judged on (LEFT_LAUNCH_SPEED), it is carried across the oncoming lanes at its post-go profile (carry,
go_override's: plannerd's lift, never steering, never more than the profile). A lifted go the car hasn't
moved on within turnpilotd's LEFT_NO_MOVE_S is withdrawn and handed back there (the model holding the car under G5's bounded
go). An unlifted one with the model planning no motion that long goes back to a wait at its line (go_unfollowed: its own hold,
a fresh window, never a handback, LS-2); with the model planning motion, the go still out and nothing holding the car,
standing NO_MOVE_S, the car isn't answering: the handback, said. A go it withdraws itself before the car moves ends that
go's clocks there (LEFTS-1).
The left's own hold after a withdrawn go (hold_here: the band, the position fault, a veto) has the latch's release rule
(map_controls._held; owner decisions (a), (b), (c), 2026-10-30), position-free: the car moved by the driver (openpilot
disengaged: the gas, or the brake let off) past where TurnPilot's stop asked it to stand by more than LATCH_PAST_M (metres
inside the ask are TurnPilot's, whoever drove them) or into the oncoming lanes (the room to ONCOMING_ENTRY_M read where the
hold began), or moved by the driver and re-engaged so far on, or so fast, that the hold taken up there could no longer
stand it short of the oncoming lanes (from the re-engage's own speed as the hold's braking takes it, hold_caps through
map_controls.reengage_stop_m), is the driver's: never held again (a re-engage there is never a hold in the oncoming lanes or
the junction), and its crossing is said (crossed: map_controls raises its Driven, the take-control up until the car is
through, turnpilotd's turnMissed for one driven straight on). The lanes are judged at the re-engage, never on the driver's
frames before it: a car the driver brakes to a stand short of the lanes is held there on the re-engage, the full check
going on (c) (skeptic round 2: judged on the driver's frame from the speed then, it was let go for good, and a re-engage
standing short of the lanes pulled the car away across them). Engaged past the re-engage, the plan is TurnPilot's: however
far its own stop carries the car is never the driver's and nothing is let go on it, so a re-engage short of all that is
held there and the full check goes on (c) (skeptic round 1: a re-engage inside the ask was let go a frame or two on, on
TurnPilot's braking, and openpilot drove the car into the junction). So driven, or its hold line's latch let go as the
driver's (driven: the map's Driven for its maneuver), the left is done: undecided ('driven'), no stop, no go, no turn pulse
(TurnPilot never pulses a left nobody checked, and a pulse of a withdrawn go still out comes out of the model's queue under
TURN_STARTED_DEG into it: turnpilotd.clear_driven_left, the car going straight on, missed), the crossing's prompt said.
"""
import math
from dataclasses import dataclass, field

from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.map_controls import (CAMERA_GREEN_STEADY_S, LATCH_PAST_M, SIGNAL_UNSEEN_REASON, SIGNAL_UNSEEN_S,
                                                        reengage_stop_m, unprotected_turn)
from openpilot.selfdrive.turnpilot.route_follower import UNCERTAIN_M
from openpilot.selfdrive.turnpilot.scene_policy import (CROSSWALK_X_AFTER_M, CROSSWALK_X_BEFORE_M, ONCOMING_ENTRY_M, ONCOMING_PAST_M,
                                                        ONCOMING_Y_FAR_M, ONCOMING_Y_NEAR_M, STOP_DECEL, VEHICLES, box_overlap,
                                                        committed_past_line, stop, velocity_trusted)
from openpilot.selfdrive.turnpilot.scene.tracker import RawBox, RawFrame

# T_req: the left's post-go profile (turn_speed.py: 'left'), the path from the line through the oncoming lanes
A_GO = 1.5                 # m/s^2
V_GO = 4.3                 # m/s
# the go from its line (turnpilotd's left_launch_armed: a go LEFT_LIFT_ROOM_M or more short of the oncoming lanes) has its clear
# time for A_GO, and a model that 'takes over' at the speed plannerd's lift gave it (0.8 m/s) would coast 10 s across them.
# Short of LEFT_LAUNCH_SPEED while the car can still stop LEFT_LAUNCH_ROOM_M short of HELD_MIN_M (at STOP_DECEL), the go is
# judged again for the launch the car showed (LAUNCH_A_MIN at least: a mere floor for the arithmetic), as the go itself is:
# on the band the camera covers out to that launch's R_req, kept driving where nothing is seen there, withdrawn to the left's
# own wait short of the lanes where a car is; that launch is its clear time's for the rest of the left (review 2026-11-07, L3:
# handed back there until then, a give-up handback; round 2, 2026-11-08: kept on only where that R_req was within the camera's
# reach, stricter than the go, so on the car's 98 m full pass every judged launch at 25 mph or faster was withdrawn). From
# LINE_D an even launch reaches that point short of LEFT_LAUNCH_SPEED below 1.2 m/s^2 (one at 0.8 m/s^2 at 1.8 m/s); the
# left's own A_GO at 2.1 m/s, never judged. Kept on, it still crossed at the car's own pace: a coast on the lift's speed spent
# about 15 s in the oncoming lanes where R_req assumed under 3, so a car outside the camera's band at the go could arrive while
# it was still in them (review 2026-11-08, round 2's trade 'G5's coast'). So such a launch committing short of
# LEFT_LAUNCH_SPEED (it can no longer stop short of the lanes) is carried across (LeftDecision.carry: go_override's carry,
# plannerd's lift to the 'left' post-go profile, A_GO to V_GO, never more); this car's own launches (11-06, four engaged from
# a stand: committing at 2.6-3.0 m/s from LINE_D) never are
LEFT_LAUNCH_SPEED = 2.
LEFT_LAUNCH_ROOM_M = 1.5
LAUNCH_A_MIN = 0.1
CLEAR_PATH_PAST_M = 8.     # the path is the distance to the maneuver point plus this (17 m from the 9 m line)
# go to first motion (STANDSTILL_SPEED): this car's own launch, 0.92-1.6 s from the plan's go (11-06, 1d and 1e: five engaged
# launches from a stand), and the frame or two from the left's go to the plan's. Review 2026-11-07 (L4): 1 s, thin against
# it, so a go certified a gap 0.6 s short; turnpilotd's LEFT_NO_MOVE_S (2 s) withdraws a go not moving by then
GO_LATENCY_S = 1.7
PET_S = 2.                 # post-encroachment margin
EXTRA_LANE_S = 0.5         # per opposing lane past the first (AASHTO Case F)
UNKNOWN_LANES = 2          # opposing lanes when the map doesn't say
SPEED_MARGIN = 1.3         # x the mapped limit (CRITIQUE REQUIRED 4: 15 mph over a 35 mph limit needs ~1.4)
DEFAULT_OPPOSING_SPEED = 20.1  # m/s (45 mph) when the opposing road's limit isn't mapped
CONFLICT_PAST_M = 3.       # the first oncoming lane's conflict point, past the maneuver point
# the detector's proven range for oncoming traffic on the road camera (TurnPilotLeftRangeM): unmeasured, so none (skeptic G4,
# CRITIQUE SEV2): every autonomous left is past it, driven to its line and handed back there, never gone blind. A drive sets
# the param from a measurement of vehicle recall against range on this car's real frames (DESIGN section 6's replays: the
# range where recall is 95% or more). PROVISIONAL_RANGE_M is only pixel arithmetic (a car 1.8 m wide is ~6.5 px at the
# 640x384 input at 150 m), never a default: 25 mph needs R_req 144-151 m, 35 mph ~197 m (the far crop; 134-141 and ~183 at
# GO_LATENCY_S 1 s, before this car's own launch was measured: L4)
ROAD_RANGE_M = 0.
PROVISIONAL_RANGE_M = 150.
# what the full pass can box at all, whatever TurnPilotLeftRangeM says (turnpilotd caps LeftInput.range_m to it; review
# 2026-11-07, LN-2): the pass is the 1344x760 road frame letterboxed into the 640x384 input, so a car 1.8 m wide (f 1141.5 px)
# is 1.8 x 1141.5 x 640/1344 / R = 978/R px there, and the detector's smallest boxes are about 10 px (maps/defaults' own
# floor for the 210 m default, which it put on the native full pass): 97.8 m. Measured, it is no further. Daylight comma four
# footage through this very 640x384 graph (2,853 vehicle boxes at RAW_CONF or more): the narrowest box 12.2 px, p99 88 m,
# 0.2% past 100 m, per-frame recall against the native strip 0.27 at 75-100 m and 0.05 at 100-125 m, a 2.5 s window with no
# box on a tracked car 15% of the time at 75-100 m and 57% at 100-125 m; the 11-06 night drives: narrowest 12.6 px (1e). On
# the car the native schedule never ran (11-06, both drives) and the far crop went off in minutes, so this pass alone judged
# the band, and a 210 m range certified a 35 mph left (R_req 182.6 m at its line) on a band no box could reach past ~100 m.
# Past it a left is judged on the far slot's runs (far_needed): the native schedule's (TurnPilotLeftRangeM, Kellen's 210 m,
# its pixel arithmetic), or the far crop's own measured range; with neither it hands back at its line ('farUnmeasured':
# "Can't see oncoming far enough"), the car held, never skipped. Replace it from a day recall run on the car
FULL_PASS_RANGE_M = 98.
# the detector's night ceiling on the 11-06 night drive (1e: the full pass's raw vehicle boxes, ranged by width, p99 50 m, max
# 78 m; oncoming tracks first published at p90 28-53 m), the observed band, not a recall measurement. Until 2026-11-07 the
# reach was capped here after dark (LeftInput.dark), so every road over about 8 mph handed back at its line at night (R_req
# 12 m plus 10.9 m per m/s of the opposing limit at GO_LATENCY_S 1 s, 11.8 since L4: 25 mph, 134 m, now 144 m); since, no
# night-only gating (Kellen), and it only labels a
# left past it after dark ('night', reach_fault) in turnpilot.left_reach
NIGHT_LEFT_RANGE_M = 50.

RAW_CONF = 0.35            # a raw box at the decoder's floor confidence counts (CRITIQUE REQUIRED 2)
# a box's width floors its nearest plausible range (nearest_range): the narrowest vehicle of its class, m across. On comma's
# public demo route (a comma four on a Ford whose radar gave the true range), near_m alone (NEAR_PX, 3 px, about 0.15 deg)
# read past the true range on 27-39% of the full pass's runs on oncoming cars at 20-120 m, since the car's pitch and the
# road's grade move a far box's bottom by up to about 20 px; floored by a 1.5 m car's width, on 0-6.5% (none past 60 m).
# YOLO's car boxes ran 1.2-1.4 times a 1.8 m car's width there, so the floor reads 0.3-0.65 of the true range: more waiting
# for far traffic, never a car inside R_req read outside it by its box bottom alone
WIDTH_FLOOR_M = {'car': 1.5, 'truck': 1.5, 'bus': 1.5, 'motorcycle': 0.6, 'cyclist': 0.4}
WIDTH_FLOOR_DEFAULT_M = 0.4
BAND_TOL_DEG = 2.          # the band's near edge widens with range by this (heading error, the road's curve)
CLEAR_WINDOW_S = 2.5       # the band clear on road runs this long (CRITIQUE REQUIRED 2: 2-3 s, 3x the longest dropout)
# the window counts on the approach too, so a band clear through the last CLEAR_WINDOW_S of the roll-in goes as the car comes
# to its stand at the line. False (the public build's): it starts over at the hold, a CLEAR_WINDOW_S stand at every left
WINDOW_FROM_APPROACH = True
# road runs further apart than this start the window over; needed, far runs covering the band too. What the gap rule can spare
# between two runs: a car at SPEED_MARGIN x the limit just past R_req on the last run is inside it by that travel at the go,
# out of PET_S's margin, so a quarter of PET_S (13 m at 45 mph x 1.3), the window still 5 runs or more and the gap under a
# third of it (CRITIQUE REQUIRED 2's longest dropout, 0.83 s). Review 2026-11-07 (L2): 0.3 s, one frame past far first's
# deadlines (ROAD_DUE_S 0.25), so a run a frame late started the window over: a model 2 ms slower than 11-06's held an empty
# road's 35 mph left at a signalized line p50 28-35 s (the real left on the 11-06 spread, 2 road heads at 1.4x)
ROAD_FRESH_S = PET_S / 4.
FAR_FRESH_S = PET_S / 4.
FAR_AIM_MAX_DEG = 14.      # the far crop's aim, either side of straight ahead (its half-width is 15.7 deg on a comma four)
STALE_UNDECIDED_S = 2.     # no road run this long while waiting: a fault
# waiting at its line on the far crop's boxes alone this long (no band box of a road run within the full pass's range: no
# traffic reaching it), with nothing else holding the car: said once a wait (turnpilot.left_far_wait) and on its status line
# (LONG_WAIT_REASON['farBlocked']). Any far box in the band blocks at any range, so a car standing far up the road in it
# (parked on the oncoming side, queued at the next light in the car's own lanes) holds a far-needed left for as long as it is
# seen. The far crop finds a car out to about 400 m (1.8 m wide is ~5 px at its native 1141 px focal length); one closing at
# 8 m/s (18 mph, under half a far-needed road's 45-55 mph) reaches the full pass's 120-150 m from there in 31-35 s. Past
# that, on far boxes alone, the car can't tell standing from coming. Until 2026-11-07 that was the farBlocked handback; now
# the car the detector sees holds the left (the hard rail) and the left keeps waiting (Kellen: "no, no handbacks")
FAR_BLOCKED_UNDECIDED_S = 35.
# a far slot judged behind (far_behind: the full pass's band clear a whole window, its runs' age alone holding the window)
# catches up once its runs have held FAR_FRESH_S this long (R2, review 2026-11-07)
FAR_BEHIND_CLEAR_S = CLEAR_WINDOW_S
FAR_BLOCK_RECENT_S = CLEAR_WINDOW_S  # a far box in the band this recently keeps that count going (the far crop finds a far
                                    # car on one run in several; further apart, the window fills between them instead)
# the band is judged at the car's measured heading (band_blockers, far_aim), standing at its line or, after a withdrawn go,
# on its own hold yawed into the turn. Past this either way the band's far edge (straight up the oncoming road, BAND_TOL_DEG
# wide) leaves the road camera's frame, its half-width on the comma four (os04c10: 1344 px at f 1141.5, 30.5 deg): judged on
# what the camera does frame (out to yaw_view_m up the road), said once a hold with a chime (turnpilot.left_yaw_view), its
# line LONG_WAIT_REASON['yaw']. The far crop covers that band to 27.7 deg (FAR_AIM_MAX_DEG plus its 15.7 deg half-width),
# past which the left is judged on the full pass's band once far_stale passes STALE_UNDECIDED_S. Review 2026-11-08 (L3,
# round 2): the 'yaw' handback past 15 deg until then, and a go withdrawn short of the oncoming lanes stood past it (the Sim
# 25-29 deg; this car's own lefts from a stand, 11-06 and before, 0-39 deg 4 m from first motion), handed back. Never a
# handback for what the camera can't see (Kellen, 2026-11-07)
ROAD_CAMERA = DEVICE_CAMERAS[('mici', 'os04c10')].narrow_road
ROAD_HALF_FOV_DEG = math.degrees(math.atan2(ROAD_CAMERA.width / 2., ROAD_CAMERA.focal_length))
YAW_VIEW_DEG = ROAD_HALF_FOV_DEG - BAND_TOL_DEG
# waiting at its line for a gap this long, whatever the tracker says: a long wait, said once (turnpilot.left_no_gap)
# and on its status line (LONG_WAIT_REASON), the car held, never let go, never handed back (Kellen, 2026-11-07: "no, no
# handbacks"). Until then it was the noGap handback (DESIGN's UNDECIDED row, 'no gap in 45 s'; owner decision (ii),
# 2026-10-30, Kellen's delegate), never an endless silent wait: skeptic g4c, SIMFIX, a car parked 50 m up the oncoming curb,
# or at the curb 6 m past the junction in its box, held the car at the line for a 240 s run, 'waitingForGap' only, no sound;
# 154bdaa bounded only what the tracker says stands, so the same car seen only as raw boxes (a box nothing tracks can't be
# told standing from coming), a stream with no gap, or a car moving in the junction box, was still waited on with no limit.
# Now the wait goes on, saying so. Said as 'standing' where the tracker says something stands (stands_still(): velocity
# trusted, under STAND_SPEED): a band box matched to such a vehicle track (a dropout of its box up to CLEAR_WINDOW_S keeps
# it), or such tracks alone in the exit crosswalk and the junction box. Counted waiting or creeping at its line or its own
# hold, from the hold and from a go; another hold keeping the car where it is (a red, a stop sign, no lamp) is no wait for
# a gap and pauses it (its own line says that wait: LIGHT_WAIT_REASON, NO_SIGNAL_REASON)
STAND_UNDECIDED_S = 45.

STAND_SPEED = 1.           # m/s: a velocity-trusted track slower than this stands
OCCLUDER_PAST_M = 30.      # an occluder stands within this past the maneuver point...
OCCLUDER_Y_FAR_M = 12.     # ...and this far to the turn side
OCCLUDER_WIDTH_M = 2.
LANE_NEAR_M, LANE_FAR_M = 1.8, 9.   # the oncoming lanes it may hide, to the turn side
MATCH_IOU = 0.3            # a raw box is a track's in the same camera at this overlap...
MATCH_GATE_M = 2.          # ...or within this (plus MATCH_GATE_FRAC of its range) of its ground position along the ray,
MATCH_GATE_FRAC = 0.15
MATCH_LATERAL_M = 1.       # and this across it: a car passing in the lane beside a standing turner is traffic

CREEP_MAX_M = 4.           # past the line
CREEP_SHORT_M = 1.         # never nearer the oncoming lanes (ONCOMING_ENTRY_M) than this
CREEP_DONE_M = 0.3         # crept to within this of its target: the full creep
# standing for the left (held at its line, crept, or on its own hold after a withdrawn go) or gone and not yet moved, a reading
# that puts the junction nearer than this is the position fault, said, the car held (skeptic SIMFIX P2, CRITIQUE REQUIRED 4):
# the go's first-motion re-check withdraws only while the car can stop short of the oncoming lanes by the live reading, so a
# reading 5.5-8 m nearer at the line (the follower 1-4 m out, the spread under UNCERTAIN_M) ignored a car coming into the band
# at first motion and committed. Half the creep's margin: past where its full creep stands the car (CREEP_SHORT_M short of the
# lanes, its stop's overshoot under half that), and room at first motion to stop from 0.85 m/s at STOP_DECEL
HELD_MIN_M = ONCOMING_ENTRY_M + 0.5 * CREEP_SHORT_M
# the view still blocked this long after the full creep, or standing this long in a creep it hasn't made with nothing else
# holding the car (the model holding a standstill: G0's tapless stand): said once a wait (turnpilot.left_view_wait), its
# status line the wait for the view (LONG_WAIT_REASON['view']), the creep still asked, never a silent stand. Until
# 2026-11-07 the 'blocked' and 'creepStall' handbacks; an occluder (the opposing left-turner) leaves, and the view it opens
# is judged from where the car stands
BLOCKED_UNDECIDED_S = 15.
CREEP_STALL_S = 10.
ROLL_UP_M = 1.5            # held further short of the line than this with R_req past the range: rolled up to it first
ROLL_UP_STALL_S = 10.      # standing that long in the roll-up: judged where it stands (the band the camera covers)

EXIT_Y_NEAR_M, EXIT_Y_FAR_M = 3., 18.  # the exit crosswalk, to the turn side, CROSSWALK_X_* around the maneuver point
# a left driven from a lane it doesn't leave from (LC-03's wrong_lane) crosses the lanes of ours between the car's and the
# turn's before the junction: a vehicle track there, from this far behind the car to the junction box, holds it
# (crossed_lane_cars, LS-1), each lane LANE_W_M wide past the first's far edge (LEFTS-2: two lanes off, the further one is
# the left's own turn lane, where left-turners queue)
CROSSED_LANE_BEHIND_M = 10.
CROSSED_LANE_Y_FAR_M = 5.4     # m to the turn side: the next lane's far edge, a lane and a half (3.6 m lanes) from the car's centre
LANE_W_M = 3.6
BOX_X_BEFORE_M, BOX_X_AFTER_M = ONCOMING_PAST_M, 8.  # the junction box a vehicle mustn't be in...
BOX_Y_RIGHT_M, BOX_Y_LEFT_M = 12., 18.               # ...from this far to the other side to this far to the turn side

# DESIGN G4 go condition 5 (skeptic G4 blocker 2): at a signal (map_controls.turn_light) the go needs its light read green by
# some source, the camera's in shadow too: a model that ran the red leaves the car at its line in the junction, the oncoming
# lanes empty at their own red, the cross traffic going, which nothing here checks. A camera green only releases TurnPilot's
# own latch (plannerd's lift needs an accepted green: turnpilotd.certify_go). False: only a red or yellow read keeps it waiting
# (unknown goes on the band alone). Since 2026-11-07 turn_light gives 'none' for a mapped signal with no colour read (Kellen:
# an unread light is no reason to wait), so this holds only 'unknown': navd's controls failing, whether a signal is there not
# known. Kept there: release_blocked lifts the left's go wherever no signal is known, so an unknown light gone on the band
# would lift the car over the model's own red-light hold, across cross traffic it can't see, at a red that empties the
# oncoming lanes. It waits instead (LIGHT_WAIT_REASON['unknown']), never handed back
SIGNAL_NEEDS_GREEN = True
# the autonomous left goes only with a lamp lit on its side, the car's or the driver's (car_signals.py, DESIGN phase4/blinker
# 6.7): a car standing in a left-turn position with no lamp invites a following driver to pass on the left, into the path
# the go takes, and IC 9-21-8-25 has every turn signalled (the last 200 ft, 300 ft at 50 mph or more: a car that reached its
# line dark is already short of it, and lit there is the most the law can still have). On this car only the driver can light it (no
# vehicle-CAN tap): car_signals prompts the side ("Signal LEFT Now", its own chime on repeat, hooks.CarSignalEvents) for as
# long as the lamp is dark, and the left waits for it at its line, said once at LEFT_SIGNAL_WAIT_S (turnpilot.left_unsignalled),
# its status line NO_SIGNAL_REASON, never a handback and never a go unsignalled (Kellen, 2026-11-07: no give-up handbacks; a
# go unsignalled would break the law and invite the pass this rule is for, and the chime asks the one person who can light
# it). Until 2026-11-07 it handed back here. False restores STOP_SIGNS D15's rule (the prompt never gates the go)
LEFT_GO_NEEDS_SIGNAL = True
# ...for this long standing at its line (the side's prompt has chimed since the approach): then it goes on its gap, said
# once (turnpilot.left_unsignalled). The public build's: no go unsignalled, ever
LEFT_SIGNAL_WAIT_S = 3.
NO_SIGNAL_REASON = "Unprotected left: waiting for the turn signal"
# s at its line with its light read unknown (navd's controls failing: SIGNAL_NEEDS_GREEN), or red or yellow: said once a wait
# (turnpilot.left_light_wait), waited through, never handed back (until 2026-11-07: 'light', "Can't read the light"). A red
# read holds the left for as long as it is read (the hard rail), a red past a cycle (a red read that never ends) included
LIGHT_UNKNOWN_S = 8.
LIGHT_RED_MAX_S = 150.
# what the left's line says while a light holds it at its line (inp.light): a red or yellow read, or not known whether there
# is a signal (navd's controls failing)
LIGHT_WAIT_REASON = {'red': "Waiting for the light", 'yellow': "Waiting for the light",
                     'unknown': "Waiting: can't tell if there's a light"}
# ...and while a red or yellow read after it stood on a green, with the car past its line, has it clear the junction on its
# gap (LS-4; Kellen, 2026-11-08: no longer held there): heard once as it starts
LIGHT_CHANGED_REASON = "Light changed: clearing the junction"
# ...or one read with it past its line on a light it never read green (LEFTS-3: no change was read, so none is said)
LIGHT_IN_JUNCTION_REASON = "Red light: clearing the junction"
# The review of 2026-11-08: the clearing fired on a red first read late, the car still rolling over the line (a red it never
# stood on a green for), and its go judged the oncoming band alone, the cross road (which has the green on our red) unlooked
# at. Now the car clears only when it stood past the line on a light not red before the spell: a steady green read there
# (CAMERA_GREEN_STEADY_S, the camera stop's own release: a one-frame green flicker between reds arms nothing), or
# CLEAR_STAND_S standing there with no red or yellow read and the model not holding. CLEAR_STAND_S: the camera reads at ~5 Hz
# on road runs and a head's state is VOTE_N of its last VOTE_M reads (light_state: 4 of 5, a red in view is said within
# about 1 s), carried CAMERA_STALE_S (1 s) between runs: a steady red in view the whole stand would have been read by 2 s;
# 3 s of stand with no red read is a light that changed during it, not one read late. Only before any red this hold: after
# one, only a steady green there arms it again (an unread spell between reds is the same red; on the car map_controls keeps
# such a red in force anyway). Once armed a creep for a view keeps it; a red first read otherwise (rolling in, short of the
# stand) holds the car where its hold is, for the green
CLEAR_STAND_S = 3.
CLEAR_STAND_V = 0.3        # m/s: standing, for that stand (the decider's own standing speed, under the creep's)
# ...and before its go (and while it can still stop after it) nothing closing in the cross lanes: a vehicle track within
# CROSS_X_M of the maneuver point along our approach, moving across our path (|vy| over CROSS_MOVING_V and over |vx|) toward
# it, there within the time the car needs to be out of the junction (t_req on one lane: the go's latency, the launch through
# the junction, PET_S), or already across it. Judged only with a view of the cross road: a wide camera run (the full wide or a
# side view: the left asks roadAndWide, one wide run in WIDE_EVERY of the detector's) within CROSS_VIEW_FRESH_S, three of
# those runs missed; without one the car holds where it stands, said in plain words, never a handback
CROSS_X_M = 12.            # a four-lane cross road's half width (7 m), the junction's spread and our position's error
CROSS_MOVING_V = 1.
CROSS_PATH_HALF_M = 1.5    # m either side of our path a crossing vehicle's centre is in it
CROSS_VIEW_FRESH_S = 1.
CROSS_REASON = {'view': "Red light: no view of the cross road, waiting", 'traffic': "Red light: cross traffic coming, waiting"}
GO_PROMPT_S = 1.5          # leftGo is shown this long
WAIT_CHIME_S = 2.          # a long wait's chime is due this long (LeftDecision.chime: hooks.ABORT_HOLD_S, the house onset)
# m to spare short of the oncoming lanes for a veto of the go (DESIGN phase4/blinker 6.6): the intent reads this frame's
# decision (withdrawable) and the veto reaches the left a frame later (signal_veto), when the car has gone on v*dt and its
# stop grown by v*a*dt/STOP_DECEL: v*dt*(1 + a/STOP_DECEL), 0.43 m at V_GO under the left's post-go cap (1.5 m/s^2), so twice
# that for an accel past the cap or a step in the follower's distance. Inside the margin a veto finishes the turn, said
VETO_MARGIN_M = 1.
ALERT_S = 2.               # oncomingHolding
DRIVEN = 'driven'          # undecided's why once the driver took the car past its line or its own hold (module docstring)
DRIVER_MOVING_SPEED = 0.3  # m/s: a disengaged car the scene committed moving this fast is the driver's (update(): driven)
DRIVEN_REASON = {1: "Unprotected left not checked: take control", -1: "Unprotected right not checked: take control"}  # by side
VIEW_FROM_M = 60.          # the road camera from this far out (the window starts on the approach)

OFF, AUTO, ANNOUNCE = 'off', 'auto', 'announce'
MODES = (OFF, ANNOUNCE, AUTO)


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
  heading_deg: float       # the car's heading change toward the turn since its first pulse, moving (turnpilotd.track_yaw)
  v_limit: float | None    # m/s, the opposing road's mapped limit, None unknown
  opposing_lanes: int | None
  range_m: float = ROAD_RANGE_M
  scene_clear: bool = True       # the scene held nothing last frame
  junction_uncertain: bool = False
  go_withdrawn: bool = False     # turnpilotd withdrew the go (not moving): its handback
  committed: bool = False        # the scene committed to this junction (scene_policy: committed_id)
  released: bool = False         # the driver released the scene's hold here (a gas press at the line: released_id)
  spread_m: float = 0.           # m between the furthest and the nearest of navd's readings of the junction (route follower)
  other_hold: bool = False       # another of TurnPilot's stops holds the car at or before the junction (a signal's red, a
                                 # stop sign: map_controls.stop_ahead), last frame
  light: str = 'none'            # its junction's signal's light (map_controls.turn_light): none (no signal), red, yellow, green,
                                 # unknown
  signal_lit: bool = True        # the turn's side lit (ours or the driver's), the other side dark (car_signals.py)
  signal_veto: str = ''          # 'driverSignal': the driver overruled the car's signal at the line, or in a go still
                                 # withdrawable (hands back, car held)
  far: RawFrame | None = None    # a far crop run that arrived this frame (tracker.update_far: view far), None if none did
  far_range_m: float = 0.        # the far crop's proven range (TurnPilotLeftFarRangeM): 0 unmeasured
  far_live: bool = False         # the far crop on (the full pass's latest farStatus read, noRoom or noCalibration): its runs'
                                 # freshness is judged here (FAR_FRESH_S, farStale)
  straight_m: float = math.inf   # m from the junction the opposing road stays in the band straight on from our approach
                                 # (navInstruction maneuverOpposingStraightM; turnpilotd passes 0 when not known)
  enabled: bool = True           # openpilot engaged (carControl.enabled): disengaged (the driver's pedal) it never goes, and
                                 # the car's metres on its own hold then are the driver's
  driven: bool = False           # its hold line's latch was let go as the driver's (map_controls.Driven for this maneuver)
  dark: bool = False             # after dark (turnpilotd's darkness.DarkFilter): logged (left_reach), never a cap (2026-11-07)
  wrong_lane: bool = False       # the lane planner's LC-03 placed the car in a lane the left doesn't leave from (its change
                                 # blocked or unreachable) once it stood held at its line (turnpilotd skips and reroutes one
                                 # flagged before, 2026-11-08): judged from the lane it is in, logged (left_reach)
  crossed_lanes: int = 1         # ...the lanes of ours it crosses to the turn's: the changes LC-03 left unmade (its want; one
                                 # with no lane list, where LC-03 wants one change at a time), LEFTS-2
  launch_armed: bool = False     # turnpilotd: this go from LEFT_LIFT_ROOM_M or more short of the oncoming lanes is followed
                                 # into them only by a launch (LEFT_LAUNCH_SPEED, L3)
  model_holds: str = ''          # the go wouldn't be lifted over the model's own hold (turnpilotd.left_lift_refused: 'light'
                                 # at a signal, 'room', 'param') and the model stops (its shouldStop, its takeover predicate
                                 # not held, last frame): a hold, the left waits with the model (review 2026-11-07 LS-2;
                                 # 2026-11-08: a model standing with no stop of its own waits for the go's pulse, no hold)
  go_unfollowed: bool = False    # turnpilotd: its unlifted go stood LEFT_NO_MOVE_S with the model planning no motion: the go
                                 # goes back to a wait at the line (its own hold, a fresh window), never a handback (LS-2)
  light_read: bool = True        # light's red or yellow read at the junction now (map_controls.turn_light_read); False: the
                                 # stop it raised still in force, or the left's own red latch, with nothing read (AL-2)
  line_m: float = 9.             # m before the maneuver point its hold is (map_controls.left_hold_m): LINE_D, or in the
                                 # junction's mouth where a signal is mapped (SIGNAL_LEFT_HOLD_M, 2026-11-08)
  signal_line_m: float | None = None  # m before the maneuver point of the mapped signal's own stop line (map_controls.
                                 # signal_line_m: its control's d), None with none mapped: past it the car is in the junction


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
  far_needed: bool = False        # R_req past the full pass's range: judged on the far crop's runs too
  far_blockers: int = 0           # the far run's boxes in the band this frame
  far_bearing: float = 0.         # where the far crop is to look (far_aim): deg left of the calibrated axis
  far_range_m: float = 0.         # the far crop's proven range while it reads, else 0
  withdrawable: bool = False      # go: a veto read next frame still withdraws it short of the oncoming lanes (VETO_MARGIN_M)
  chime: bool = False             # a long wait's chime is due (WAIT_CHIME_S from its _said; SF-4)
  crossed: str = ''               # the frame the driver took the car past its own hold: the cause (map_controls.Driven's)
  carry: bool = False             # committed: carried across the oncoming lanes at the left's post-go profile (CARRY_SPEED)
  clearing: bool = False          # a red or yellow read with the car past its line: it clears the junction on its gap, the
                                  # light no hold of its own (turnpilotd lifts its go over the light and the model's red hold)


def hold_caps(v: float) -> tuple[float, float]:
  """Its own hold's (accel cap, speed cap) at v m/s, as a re-engage takes it up (map_controls.reengage_stop_m): waitGap's
  and _undecided's stop(v, 0, confirmed=False), STOP_DECEL down to STOP_DECEL x SPEED_CAP_TAU, its speed cap's -v/tau under it."""
  s = stop(v, 0., '', confirmed=False)
  return s.accel_cap, s.speed_cap


def t_clear(path_m: float, a: float = A_GO) -> float:
  """s from first motion to cover path_m at a (A_GO, or the slower launch the car showed) up to V_GO."""
  d_ramp = V_GO ** 2 / (2. * a)
  if path_m <= d_ramp:
    return math.sqrt(2. * max(path_m, 0.) / a)
  return V_GO / a + (path_m - d_ramp) / V_GO


def t_req(distance: float, lanes: int | None, latency: bool = True, a: float = A_GO) -> float:
  """s the oncoming lanes must stay clear for a go from distance m before the maneuver point (module docstring), on a launch
  at a (A_GO, or the slower one the car showed: LEFT_LAUNCH_SPEED)."""
  lanes = UNKNOWN_LANES if lanes is None or lanes < 1 else lanes
  return (GO_LATENCY_S if latency else 0.) + t_clear(max(distance, 0.) + CLEAR_PATH_PAST_M, a) + PET_S + EXTRA_LANE_S * (lanes - 1)


def r_req(v_limit: float, t: float, distance: float) -> float:
  """m up the oncoming lanes that must be seen clear: what a car at SPEED_MARGIN x v_limit covers in t, from the conflict point."""
  return v_limit * SPEED_MARGIN * t + max(distance, 0.) + CONFLICT_PAST_M


LINE_D = 9.                # m from the hold line to the maneuver point (map_controls: HOLD_LINE_M + MAP_MARGIN_M)
# route time takes R_req this far past LINE_D where the junction has no mapped stop line (one that has: the line's setback,
# MapControlPolicy.line_setback): a junction readings' spread just short of the junctionUncertain alert, which the live check
# at the line takes (d_far). BUILD_FINDINGS D4: at LINE_D alone it was a best case. Logged only since 2026-11-06 (route_range):
# the live check at the line decides, from where the car stands, after rolling it up to the line (ROLL_UP_M). As a refusal
# this margin alone refused the two lefts of the 11-06 night that the line could judge (1e L1: 267.8 m at route time, 182.1 m
# at the line, under a 210 m range; L3: 198.4 m, bent, at the line 133.0 m, straight enough)
ROUTE_REACH_MARGIN_M = UNCERTAIN_M


def reach_at_line(v_limit: float | None, lanes: int | None, setback: float = 0.) -> float:
  """R_req for a go from the hold line, or setback m further back, the opposing road's mapped limit (DEFAULT_OPPOSING_SPEED
  when unknown): what route_range logs against the detector's proven range, at LINE_D plus its margin for a set-back stop line
  and the junction readings' spread (BUILD_FINDINGS D4: the live check at the line takes where the car stands and the furthest
  reading, d_far, which LINE_D alone was a best case of). Never a refusal: the left is judged at its line (route_range)."""
  v = v_limit if v_limit is not None and v_limit > 0. else DEFAULT_OPPOSING_SPEED
  d = LINE_D + max(setback, 0.)
  return r_req(v, t_req(d, lanes), d)


LEFT_RANGE_MAX_M = 400.  # TurnPilotLeftRangeM and TurnPilotLeftFarRangeM are clamped to 0..this


def range_param(value, unset: float) -> float:
  """A TurnPilotLeftRangeM or TurnPilotLeftFarRangeM value as m, clamped to 0..LEFT_RANGE_MAX_M; unset for None. Not a number
  raises (TypeError, ValueError), for the reader to say: turnpilotd's left_range_for and navd's left config read it alike."""
  value = value.decode() if isinstance(value, bytes) else value
  return min(max(float(value), 0.), LEFT_RANGE_MAX_M) if value is not None else unset


def left_reach(range_m: float, far_range_m: float, far_on: bool) -> float:
  """The range an unprotected left's R_req may reach: the full pass's proven range (TurnPilotLeftRangeM), or the far crop's
  (TurnPilotLeftFarRangeM) where further, while the far crop is on and the full pass's range is measured (the far crop covers
  the band only past it). turnpilotd's far_on is the full pass's farStatus (far_live, route time's: on, reading or waiting);
  navd reads no farStatus and takes TurnPilotFarCrop. Both ranges unmeasured (0, the car today): none, either way. Only logged
  at route time (route_range); the left's own check at its line takes the ranges as they are there, capped after dark."""
  return max(range_m, far_range_m) if far_on and range_m > 0. else range_m


@dataclass(frozen=True)
class RangeCheck:
  """route_range's verdict on one maneuver."""
  unprotected: bool = False   # an autonomous unprotected left (map_controls.unprotected_turn, under TurnPilotAutoLeft auto)
  out_of_range: bool = False  # never set since 2026-11-06 (route_range): no left is refused at route time any more
  reach: float = 0.           # m, its R_req where the car may stand (reach_at_line at LINE_D plus margin)
  margin: float = 0.          # m past LINE_D that is: the mapped stop line's setback, else ROUTE_REACH_MARGIN_M
  bent: bool = False          # its opposing road isn't known straight on out to reach (maneuverOpposingStraightM)


def route_range(maneuver, setback: float | None, reach_m: float, right_hand_traffic: bool = True, mode: str = AUTO) -> RangeCheck:
  """Route time's look at a maneuver (route_follower.Maneuver) against the detector's reach_m (left_reach): for an autonomous
  unprotected left (mode auto, the default), its R_req where the car may stand (reach) and whether its opposing road isn't known
  straight on out to that R_req (bent: the band is straight on from our approach, module docstring). setback: m the junction's
  mapped stop line lies back of the hold line (map_controls.line_setback_at), None without one: then ROUTE_REACH_MARGIN_M
  (BUILD_FINDINGS D4). For the logs only (turnpilotd's left_not_straight, navd's 'turnpilot lefts:' line): no left is out of
  range here, ever (Kellen, 2026-11-06: never route around or skip a left for range). Until then this was the predicate
  turnpilotd skipped by (turn_skip: unprotectedRange, as soon as the left was the maneuver, 58-394 m out) and navd routed around
  (routing/lefts.py), and on the 11-06 drives it refused every left, including the two the line could judge, and fed the
  reroute loop. Every left is now driven to its hold line, where UnprotectedLeft judges it from where the car stands: a gap the
  camera can certify, or a loud handback with the car held ('range', 'curve', 'night', ...)."""
  if mode != AUTO or not unprotected_turn(maneuver, right_hand_traffic):
    return RangeCheck()
  margin = ROUTE_REACH_MARGIN_M if setback is None else setback
  reach = reach_at_line(maneuver.opposing_speed, maneuver.opposing_lanes or None, margin)
  bent = reach > LINE_D + margin + maneuver.opposing_straight_m
  return RangeCheck(True, False, reach, margin, bent)


def turn_bearing(bearing_deg: float, heading_deg: float, side: int) -> float:
  """A car-frame bearing in the road's frame, positive toward the turn."""
  return bearing_deg * side + heading_deg


def nearest_range(b) -> float:
  """A ranged vehicle box's nearest plausible range for the reach test: its near_m (the box bottom NEAR_PX lower on flat
  ground), or the range at which its angular width (span_deg) is the narrowest vehicle of its class (WIDTH_FLOOR_M),
  whichever is nearer. It is only ever nearer than near_m, so it only adds blockers."""
  lo, hi = sorted(b.span_deg)
  ang = math.radians(hi - lo)
  if ang <= 1e-6:
    return b.near_m
  return min(b.near_m, WIDTH_FLOOR_M.get(b.cls, WIDTH_FLOOR_DEFAULT_M) / (2. * math.tan(ang / 2.)))


def band_blockers(boxes, heading_deg: float, side: int, reach_m: float, distance: float, exempt=(), lanes=None) -> list:
  """The road camera's raw vehicle boxes at RAW_CONF or more in the oncoming band within reach_m (module docstring), by
  their nearest plausible range (nearest_range). exempt: boxes that are an occluder's (not traffic). A box the tracker
  says is going away up the road (RawBox.motion receding: our own direction's traffic) is no oncoming car, and neither is
  one standing wholly outside the opposing lanes: beyond their far edge (lanes_far_m of lanes, view_blocked's bound: parked
  at the far curb) or wholly on our side of the car's own centre line (our direction's traffic standing ahead, in the band
  only by BAND_TOL_DEG). One standing in the opposing lanes blocks as any other: a queue there launches, and the tracker
  reads a launched car standing for 0.6-3 s more. The far edge is navd's lane count (two lanes where it has none)."""
  out = []
  lane_far = lanes_far_m(lanes)
  for b in boxes:
    if b.camera != 'road' or b.cls not in VEHICLES or b.conf < RAW_CONF or b in exempt or b.motion == 'receding':
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
    if b.motion == 'standing' and (lat_near >= lane_far or lat_far <= 0.):
      continue
    if nearest_range(b) <= reach_m and x > distance - ONCOMING_PAST_M and lat_far > ONCOMING_Y_NEAR_M - rng * math.tan(tol) and \
       lat_near < ONCOMING_Y_FAR_M + rng * math.tan(tol):
      out.append(b)
  return out


def yaw_view_m(heading_deg: float) -> float:
  """m up the oncoming road the road camera still frames the band's near edge (ONCOMING_Y_NEAR_M) from heading_deg (the
  car's yaw off its approach): all of it within ROAD_HALF_FOV_DEG; past it 167 m at 31 deg, 57 m at 32, 19 m at 35, 9 m at
  40 (YAW_VIEW_DEG: turnpilot.left_yaw_view logs it)."""
  past = abs(heading_deg) - ROAD_HALF_FOV_DEG
  return math.inf if past <= 0. else ONCOMING_Y_NEAR_M / math.tan(math.radians(past))


def far_need_deg(range_m: float) -> tuple[float, float]:
  """The band's bearings past the full pass's range (range_m), in the road's frame toward the turn: from straight up the road
  far out to its far lane edge at range_m, each widened by BAND_TOL_DEG."""
  return -BAND_TOL_DEG, math.degrees(math.atan2(ONCOMING_Y_FAR_M, max(range_m, 0.))) + BAND_TOL_DEG


def far_covers(cover_deg, heading_deg: float, side: int, range_m: float) -> bool:
  """A far run (its cover_deg: right and left bearings at the horizon, None for none) covers the band past range_m."""
  if cover_deg is None:
    return False
  lo, hi = sorted((t - heading_deg) * side for t in far_need_deg(range_m))
  return cover_deg[0] <= lo and hi <= cover_deg[1]


def far_aim(heading_deg: float, side: int, range_m: float) -> float:
  """Where the far crop is to look (turnpilotPlan.farCropBearing): the middle of the band's bearings past range_m, in the
  car's frame, within FAR_AIM_MAX_DEG."""
  lo, hi = far_need_deg(range_m)
  return min(max(((lo + hi) / 2. - heading_deg) * side, -FAR_AIM_MAX_DEG), FAR_AIM_MAX_DEG)


def road_xy(obj, heading_deg: float, side: int) -> tuple[float, float]:
  """A track's ground position in the road's frame: (x along the approach, y toward the turn)."""
  a = math.radians(heading_deg) * side
  x, y = obj.x * math.cos(a) - obj.y * math.sin(a), obj.x * math.sin(a) + obj.y * math.cos(a)
  return x, y * side


def occluders(objects, heading_deg: float, side: int, distance: float, wrong_lane: bool = False, lanes: int = 1) -> list:
  """Vehicle tracks standing in the junction on the turn side (module docstring): velocity trusted, slower than STAND_SPEED.
  Driven from a lane it doesn't leave from (wrong_lane), never one in the lanes of ours it crosses short of the maneuver point
  (in_crossed_lane, lanes of them): a car there is in the left's path, never a view to creep past (LS-1, LEFTS-2)."""
  out = []
  for obj in objects:
    if str(obj.cls) not in VEHICLES or not velocity_trusted(obj) or math.hypot(obj.vx, obj.vy) >= STAND_SPEED:
      continue
    x, y = road_xy(obj, heading_deg, side)
    if wrong_lane and x < distance and in_crossed_lane(y, lanes):
      continue
    if distance - ONCOMING_PAST_M <= x <= distance + OCCLUDER_PAST_M and ONCOMING_Y_NEAR_M <= y <= OCCLUDER_Y_FAR_M:
      out.append(obj)
  return out


def in_crossed_lane(y: float, lanes: int = 1) -> bool:
  """y m toward the turn is in the lanes of ours a left lanes off its own crosses (CROSSED_LANE_Y_FAR_M, a lane and a half
  out, and LANE_W_M more per lane past the first; LEFTS-2)."""
  return ONCOMING_Y_NEAR_M <= y <= CROSSED_LANE_Y_FAR_M + LANE_W_M * (max(lanes, 1) - 1)


def crossed_lane_cars(objects, heading_deg: float, side: int, distance: float, lanes: int = 1) -> list:
  """The vehicle tracks, any speed, trusted or not, in the lanes of ours (lanes of them: LeftInput.crossed_lanes, LEFTS-2) a
  left driven from a lane it doesn't leave from (LeftInput.wrong_lane) crosses before the junction: on the turn side
  (in_crossed_lane), from CROSSED_LANE_BEHIND_M behind
  the car to where the junction box starts (BOX_X_BEFORE_M short of the maneuver point; the box is exit_blockers'). Review
  2026-11-07 (LS-1): nothing of the go looked there, so a car standing level with ours in that lane (the bay's first
  left-turner, or one going straight) held nothing (the go at 31.8 s with or without it), and one moving off with ours at
  the same rate was never seen as a conflict: the car turned across its lane. The car's blind spot monitor is no help here:
  on this car it never read a car standing (0 of 8,307 standing carState frames over 27 stops; its lowest read 1.68 m/s)."""
  out = []
  for obj in objects:
    if str(obj.cls) not in VEHICLES:
      continue
    x, y = road_xy(obj, heading_deg, side)
    if -CROSSED_LANE_BEHIND_M <= x < distance - BOX_X_BEFORE_M and in_crossed_lane(y, lanes):
      out.append(obj)
  return out


def matches(b: RawBox, obj) -> bool:
  """A raw box is this track's: overlapping boxes in the same camera, or ground positions within the gate."""
  if str(obj.camera) == b.camera and len(obj.box) == 4 and len(b.box) == 4 and box_overlap(tuple(obj.box), b.box) >= MATCH_IOU:
    return True
  if b.xy is None:
    return False
  rng = math.hypot(*b.xy)
  ux, uy = b.xy[0] / max(rng, 1e-6), b.xy[1] / max(rng, 1e-6)
  dx, dy = obj.x - b.xy[0], obj.y - b.xy[1]
  along, across = dx * ux + dy * uy, -dx * uy + dy * ux  # mono range error is along the ray; across it, a lane is 3.6 m
  return abs(along) <= MATCH_GATE_M + MATCH_GATE_FRAC * rng and abs(across) <= MATCH_LATERAL_M


def lanes_far_m(lanes=None) -> float:
  """m to the turn side of the oncoming lanes' far edge: lanes of them (navd's opposing_lanes) LANE_W_M each past
  LANE_NEAR_M, LANE_FAR_M (two) where the count isn't known."""
  return LANE_NEAR_M + LANE_W_M * lanes if lanes else LANE_FAR_M


def view_blocked(occ, heading_deg: float, side: int, reach_m: float, lanes=None) -> bool:
  """An occluder's bearings cover some of the oncoming lanes beyond it, out to reach_m: lanes of them (the opposing road's,
  navd's opposing_lanes) LANE_W_M each past LANE_NEAR_M, LANE_FAR_M's two where the count isn't known. Drive 29 (2026-11-08,
  seg 7): a car parked at the far curb of a one-lane opposing road, 26 m past the maneuver point and 7.2 m to the turn
  side, read as hiding the outer of two lanes the road doesn't have: the view blocked 25 s at an empty junction."""
  lane_far = lanes_far_m(lanes)
  for obj in occ:
    x, y = road_xy(obj, heading_deg, side)
    if x <= 0.:
      continue
    lo, hi = math.atan2(y - OCCLUDER_WIDTH_M / 2., x), math.atan2(y + OCCLUDER_WIDTH_M / 2., x)
    lanes_lo, lanes_hi = math.atan2(LANE_NEAR_M, reach_m), math.atan2(lane_far, x)
    if lo < lanes_hi and hi > lanes_lo:
      return True
  return False


def exit_blockers(objects, heading_deg: float, side: int, distance: float, occ, spread_m: float = 0., lanes=None) -> list:
  """The pedestrians in the exit crosswalk and the vehicles but an occluder in the junction box (module docstring), each zone
  stretched over the readings of the junction (spread_m past the nearest). An occluder is exempt only standing in the
  oncoming lanes (to lanes_far_m: the opposing left-turner, whose hidden lanes view_blocked waits on): one standing beyond
  them in the box (the exit lane) is an exit blocker, as it was only by view_blocked's two lanes until drive 29's lanes."""
  lane_far = lanes_far_m(lanes)
  ids = {o.trackId for o in occ if road_xy(o, heading_deg, side)[1] <= lane_far}
  spread = max(spread_m, 0.)
  out = []
  for obj in objects:
    x, y = road_xy(obj, heading_deg, side)
    rel = x - distance
    if str(obj.cls) == 'pedestrian' and -CROSSWALK_X_BEFORE_M <= rel <= CROSSWALK_X_AFTER_M + spread and \
       EXIT_Y_NEAR_M <= y <= EXIT_Y_FAR_M:
      out.append(obj)
    elif str(obj.cls) in VEHICLES and obj.trackId not in ids and -BOX_X_BEFORE_M <= rel <= BOX_X_AFTER_M + spread and \
       -BOX_Y_RIGHT_M <= y <= BOX_Y_LEFT_M:
      out.append(obj)
  return out


def exit_clear(objects, heading_deg: float, side: int, distance: float, occ, spread_m: float = 0., lanes=None) -> bool:
  """No pedestrian in the exit crosswalk, and no vehicle but an occluder in the junction box (exit_blockers)."""
  return not exit_blockers(objects, heading_deg, side, distance, occ, spread_m, lanes)


def cross_closing(objects, distance: float, t_bound: float) -> list:
  """The vehicle tracks closing on our path in the cross road (CROSS_X_M of the maneuver point distance m ahead, moving
  across our path toward it) that reach it within t_bound s, or are across it now (the clearing's go: CROSS_REASON)."""
  out = []
  for o in objects:
    if str(o.cls) not in VEHICLES:
      continue
    x, y, vx, vy = float(o.x), float(o.y), float(o.vx), float(o.vy)
    if abs(x - distance) > CROSS_X_M or abs(vy) < CROSS_MOVING_V or abs(vy) < abs(vx):
      continue
    gap = abs(y) - CROSS_PATH_HALF_M
    if gap <= 0. or (y * vy < 0. and gap / abs(vy) <= t_bound):
      out.append(o)
  return out


def stands_still(obj) -> bool:
  """A track standing still by the tracker: its velocity trusted (VEL_TRUST_HITS), slower than STAND_SPEED."""
  return velocity_trusted(obj) and math.hypot(obj.vx, obj.vy) < STAND_SPEED



class UnprotectedLeft:
  def __init__(self):
    self.maneuver_id: int | None = None
    self.always_drive = False  # TurnPilotAlwaysDrive, handed over every frame (turnpilotd.hand_always_drive; ALWAYS_DRIVE.md section 1)
    self.window_from_approach = WINDOW_FROM_APPROACH
    self._reset()

  def _reset(self):
    self.state = 'approach'
    self.clear_s = 0.
    self.road_age = math.inf      # s since the last road camera run
    self.blocked = False          # the last road run had a car in the band
    self.latch_d: float | None = None  # m to the maneuver point where the hold latched
    self.creep_to = 0.
    self.blocked_s = 0.           # the view blocked with the full creep done
    self.creep_stall = 0.         # s standing in a creep not yet made, nothing else holding the car
    self.went = False             # gone (not withdrawn since)
    self.moved = False            # ...and moved
    self.go_t = 0.
    self.alert_t = 0.
    self.why = ''
    self.verdict = ''
    self.hold_here = False        # its own hold, after a withdrawn go
    self.hold_reason = WITHDRAWN_REASON['band']  # ...and what it says
    self.hold_m = 0.              # ...m the car moved since it began (position-free)...
    self.hold_drv_m = 0.          # ...of them with openpilot disengaged (the driver's)...
    self.hold_drv_past = 0.       # ...of those, the ones past where its stop asked the car to stand...
    self.hold_asked = 0.          # ...m on to where its stop asked the car to stand (the comfortable curve's from there)...
    self.hold_entry = 0.          # ...and to the oncoming lanes (ONCOMING_ENTRY_M), read where it began...
    self.hold_drv_frame = False   # ...and a driver's frame (disengaged) since the last engaged one: the next is a re-engage
    self.roll_stall = 0.          # s standing in a roll-up to the line
    self.light_s = 0.             # s at its line with its signal's light not green (unknown, or red and yellow)
    self.unsignalled_s = 0.       # s standing at its line with no lamp on its side (LEFT_GO_NEEDS_SIGNAL)
    self.saw_green = False        # it stood at its line on a green (a red or yellow read after it: left_light_changed, once)
    self.said: set = set()        # the long waits said this hold (_said: once each, never a frame's log)
    self.far_age = math.inf       # s since the last far run covering the band
    self.far_run_age = math.inf   # s since the last far run
    self.far_blocked = False      # the last far run had a car in the band
    self.far_stale = 0.           # s at its line without a far run covering the band
    self.far_block_age = math.inf # s since the last far run with a car in the band
    self.far_only_s = 0.          # s waiting at its line on the far crop's boxes alone (FAR_BLOCKED_UNDECIDED_S)
    self.band_stand_age = math.inf  # s since a road run's band box matched a standing track (STAND_UNDECIDED_S)
    self.band_stand_s = 0.        # s waiting at its line with the band so blocked
    self.exit_stand_s = 0.        # s waiting at its line with the exit zones blocked by standing tracks alone
    self.wait_s = 0.              # s waiting at its line (or its own hold) for a gap, nothing else holding the car
    self.near_clear_s = 0.        # clear_s on the full pass's band alone: every reset of clear_s's but the far slot's age (R2)
    self.far_behind = False       # the far slot's runs can't hold FAR_FRESH_S: judged on the full pass's band (R2)
    self.far_fresh_s = 0.         # s the far slot's runs have held FAR_FRESH_S (FAR_BEHIND_CLEAR_S ends far_behind)
    self.chime_t = 0.             # s of a long wait's chime still due (WAIT_CHIME_S, _said)
    self.light_kind = ''          # the light wait counted (light_s): 'red' (or yellow), 'unknown' or 'model'
    self.changed_crept = False    # a red or yellow read with the car crept past its line (LS-4, LEFTS-3), said once a red
    self.changed_read = False     # ...after it stood on a green (a change read: LIGHT_CHANGED_REASON)
    self.clearing = False         # a red or yellow spell first read with the car past its line: it clears the junction (LS-4)
    self.red_prev = False         # last frame's light a red or yellow
    self.mouth_green_s = 0.       # s standing past its line on a green read (CAMERA_GREEN_STEADY_S arms the clearing)
    self.mouth_stand_s = 0.       # s standing past its line with no red or yellow read (CLEAR_STAND_S arms it)
    self.mouth_armed = False      # ...armed: a red or yellow first read now clears the junction (the review of 2026-11-08)
    self.red_this_hold = False    # a red or yellow read this maneuver: only a steady green arms the clearing from here
    self.wide_age = math.inf      # s since the last wide camera run (the cross road's view, CROSS_VIEW_FRESH_S)
    self.cross = ''               # this frame's cross-road hold on the clearing: 'view' (none fresh), 'traffic' (closing)
    self.line_d: float | None = None  # m to the maneuver point of its line this hold: the nearer of its first latch and LINE_D
    self.unseen_s = 0.            # s its light's red has held it with nothing read (SIGNAL_UNSEEN_S, AL-2)
    self.quiet_go = False         # a go of this hold the model didn't follow: its retries silent until the car moves (AL-1)
    self.launch_a = A_GO          # the launch its clear time is for: A_GO, or the slower one the car showed (L3)
    self.go_d = 0.                # m to the maneuver point at its go
    self.launch_on = False        # ...a slow launch judged on for this go: the band it covers clear for it (L3)
    self.carry = False            # ...and committed short of a launch after it: carried across (LeftDecision.carry)
    self.road_boxes: list = []    # the last road run's boxes, its occluders' own out
    self.range_eff = 0.           # the band's reach this frame: the full pass's, or the far slot's past it

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
    light = inp.light
    # the path and the conflict point at the furthest reading of the junction: the hold line is at the nearest (BeamNG: 13 m
    # apart, the car 22 m out certifying a 9 m line's path)
    d_far = inp.distance + max(inp.spread_m, 0.)
    treq = t_req(d_far, inp.opposing_lanes, latency, self.launch_a)
    rreq = r_req(v_limit, treq, d_far)
    occ = occluders(inp.objects, inp.heading_deg, side, inp.distance, inp.wrong_lane, inp.crossed_lanes)
    # LS-1: from a lane it doesn't leave from, a car in the lane it crosses holds it (no go, no creep, a fresh window), and
    # withdraws a go while it can still stop (_after_go): a car the detector sees holds the left
    beside = crossed_lane_cars(inp.objects, inp.heading_deg, side, inp.distance, inp.crossed_lanes) if inp.wrong_lane else []
    self.road_age += dt
    self.band_stand_age += dt
    raw_blockers = None
    if inp.raw is not None and inp.raw.camera == 'road':
      exempt = [b for b in inp.raw.boxes if any(matches(b, o) for o in occ)]
      raw_blockers = band_blockers(inp.raw.boxes, inp.heading_deg, side, rreq, inp.distance, exempt, inp.opposing_lanes)
      self.road_boxes = [b for b in inp.raw.boxes if b not in exempt]  # the last road run's, for a slow launch's look (L3)
      stands = [o for o in inp.objects if str(o.cls) in VEHICLES and stands_still(o)]
      if any(matches(b, o) for b in raw_blockers for o in stands):
        self.band_stand_age = 0.
      fresh_run = self.road_age <= ROAD_FRESH_S
      self.road_age = 0.
      self.blocked = bool(raw_blockers)
      self.clear_s = 0. if self.blocked or not fresh_run else self.clear_s + dt
      self.near_clear_s = 0. if self.blocked or not fresh_run else self.near_clear_s + dt
      if any(b.xy is not None and math.hypot(*b.xy) <= inp.range_m for b in raw_blockers):
        self.far_only_s = 0.  # traffic within the full pass's range: not waiting on far boxes alone
    elif self.road_age > ROAD_FRESH_S or self.blocked or not inp.scene_clear:
      self.clear_s = self.near_clear_s = 0.
    else:
      self.clear_s += dt
      self.near_clear_s += dt
    # the far crop covers the band past the full pass's range only (far_covers): an unmeasured full range, it extends nothing
    range_eff = self.range_eff = max(inp.range_m, inp.far_range_m) if inp.far_live and inp.range_m > 0. else inp.range_m
    # after dark nothing is capped (Kellen, 2026-11-07: "no handbacks ... we are pushing bleeding edge here, not maximal
    # caution"; a safety driver on every drive): the band is judged on what the camera boxes, a car it sees holding the left.
    # The far slot gates the window only where it extends the band (live, its range past the full pass's): without it the
    # left is judged on the full pass's band, never held for the stretch it can't see
    far_needed = rreq > inp.range_m and range_eff > inp.range_m
    self.far_age += dt
    self.far_run_age += dt
    self.far_stale += dt
    self.far_block_age += dt
    far_blockers = None
    if inp.far is not None and inp.far.view == 'far' and inp.far.camera == 'road':
      exempt = [b for b in inp.far.boxes if any(matches(b, o) for o in occ)]
      far_blockers = band_blockers(inp.far.boxes, inp.heading_deg, side, math.inf if far_needed else rreq, inp.distance, exempt,
                                   inp.opposing_lanes)
      self.far_blocked, self.far_run_age = bool(far_blockers), 0.
      if far_blockers:
        self.far_block_age = 0.
      if far_covers(inp.far.cover_deg, inp.heading_deg, side, inp.range_m):
        self.far_age = self.far_stale = 0.
    if far_needed and self.far_stale > STALE_UNDECIDED_S:
      # the far slot said live but has seen nothing of the band this long (11-06: nativeStatus full, farStatus noRoom on
      # every message): judged on the full pass's band, never a farStale handback (Kellen, 2026-11-07: no handbacks)
      far_needed = False
    # a far slot whose runs can't hold FAR_FRESH_S (far_behind, R2): judged on the full pass's band as one gone stale is,
    # until its runs hold FAR_FRESH_S for FAR_BEHIND_CLEAR_S again; a far box it does see holds the window FAR_BLOCK_RECENT_S
    self.far_fresh_s = self.far_fresh_s + dt if self.far_age <= FAR_FRESH_S else 0.
    if self.far_behind and self.far_fresh_s >= FAR_BEHIND_CLEAR_S - 1e-6:
      self.far_behind = False
      cloudlog.event("turnpilot.left_far_caught_up", distance=round(inp.distance, 1), maneuver_id=inp.maneuver_id)
    if self.far_behind:
      far_needed = False
    if (self.far_behind and self.far_block_age <= FAR_BLOCK_RECENT_S) or \
       (not self.far_behind and self.far_blocked and self.far_run_age <= FAR_FRESH_S):
      self.clear_s = self.near_clear_s = 0.
    if self.far_block_age <= FAR_BLOCK_RECENT_S:
      self.near_clear_s = 0.  # a far box this recent: never judged behind on it (the far_behind trigger, below)
    elif far_needed and self.far_age > FAR_FRESH_S:
      self.clear_s = 0.  # the far slot's age alone: near_clear_s goes on (R2)
    if not inp.scene_clear:
      self.clear_s = self.near_clear_s = 0.
    blocked_view = view_blocked(occ, inp.heading_deg, side, rreq, inp.opposing_lanes)
    if blocked_view:
      self.clear_s = self.near_clear_s = 0.  # the lanes behind the occluder weren't seen: the window counts only with the view open
    # a red or yellow first read with the car past its line (in the junction's mouth at a mapped signal, or crept past it
    # for a view): it clears the junction as a driver does, on its gap (Kellen, 2026-11-08: the oncoming band clear on what
    # the camera sees, a car seen there holding it until it passes), never held there through the cross road's green nor
    # handed back. Until 2026-11-08 held where it stood (LS-4's option b). A red read with the car short of its line, or at
    # it, still holds it there for the green (the hard rail): the spell's first frame decides, for as long as it is read
    # The review of 2026-11-08: past the line is judged from the mapped signal's own line (signal_line_m), else from its
    # first latch or LINE_D, and only a car that stood there on a light not red before the spell clears (CLEAR_STAND_S)
    red_now = light in ('red', 'yellow')
    past = self._past_line_m(inp) > CREEP_DONE_M
    standing_past = past and self.latch_d is not None and inp.v_ego < CLEAR_STAND_V
    if red_now:
      if not self.red_prev:
        self.clearing = past and self.mouth_armed  # armed only standing there: a creep for a view since keeps it
        if past and not self.clearing:
          cloudlog.event("turnpilot.left_red_unstood", past_m=round(self._past_line_m(inp), 2), v=round(inp.v_ego, 2),
                         stand_s=round(self.mouth_stand_s, 1), green_s=round(self.mouth_green_s, 1), red_before=self.red_this_hold,
                         distance=round(inp.distance, 1), maneuver_id=inp.maneuver_id)
      self.red_this_hold, self.mouth_armed = True, False
      self.mouth_green_s = self.mouth_stand_s = 0.
    else:
      self.clearing = False
      if standing_past:
        self.mouth_green_s = self.mouth_green_s + dt if light == 'green' else 0.
        self.mouth_stand_s = self.mouth_stand_s + dt if not inp.model_holds else 0.
        if self.mouth_green_s >= CAMERA_GREEN_STEADY_S - 1e-6 or (not self.red_this_hold and self.mouth_stand_s >= CLEAR_STAND_S - 1e-6):
          self.mouth_armed = True
      else:
        self.mouth_green_s = self.mouth_stand_s = 0.
    self.red_prev = red_now
    self.wide_age += dt
    if inp.raw is not None and inp.raw.camera == 'wide':
      self.wide_age = 0.
    closing = cross_closing(inp.objects, inp.distance, t_req(d_far, 1, latency, self.launch_a)) if self.clearing else []
    self.cross = '' if not self.clearing else 'view' if self.wide_age > CROSS_VIEW_FRESH_S else 'traffic' if closing else ''
    if self.cross:
      self._said('cross:' + self.cross, "turnpilot.left_clearing_cross", inp, why=self.cross, wide_age=round(min(self.wide_age, 99.), 2),
                 closing=[(str(o.cls), round(float(o.x), 1), round(float(o.y), 1), round(float(o.vy), 1)) for o in closing])
    light_go = light in ('none', 'green') or (light == 'unknown' and not SIGNAL_NEEDS_GREEN) or self.clearing
    # no lamp on its side: its go waits LEFT_SIGNAL_WAIT_S at the line for it (counted below), the window counting meanwhile
    dark = LEFT_GO_NEEDS_SIGNAL and inp.mode == AUTO and not inp.signal_lit
    no_signal = dark and self.unsignalled_s < LEFT_SIGNAL_WAIT_S - 1e-6
    # something else keeps it where it is: no go, no creep. The model's own hold where the go wouldn't be lifted over it
    # (model_holds) is one (review 2026-11-07, LS-2): a go there only lets TurnPilot's latch go, so with the model planning no
    # motion it stood LEFT_NO_MOVE_S and handed back (cantStartTakeControl) at every mapped signal on the car, the model holding
    # for its red; it now waits for the model's plan to move, then judges a fresh window (the turn goes with the model)
    # clearing, the model's hold for the light (or the mouth's short room) is lifted by turnpilotd (left_clearing) from the
    # frame after the red is first read: never a window started over for that one frame; with the lift off ('param') it holds
    model_hold = '' if self.clearing and inp.model_holds != 'param' else inp.model_holds
    holds = inp.other_hold or not light_go or bool(model_hold) or bool(self.cross)
    held_else = holds or no_signal
    if holds:
      self.clear_s = self.near_clear_s = 0.  # the window counts only while nothing else holds the car
    exits = exit_blockers(inp.objects, inp.heading_deg, side, inp.distance, occ, inp.spread_m, inp.opposing_lanes) + beside
    crosswalk = not exits
    if not crosswalk:
      self.clear_s = self.near_clear_s = 0.  # everything the go is certified on holds for the whole window (DESIGN G4)
    if far_needed and not self.far_behind and self.near_clear_s >= CLEAR_WINDOW_S - 1e-6 and self.clear_s < CLEAR_WINDOW_S - 1e-6:
      # review 2026-11-07 (R2): the full pass's band clear a whole window, the far slot's age alone keeping it from counting.
      # far_needed dropped only past one far gap over STALE_UNDECIDED_S and came back on its next run, so a far crop running
      # 1-3 times a second (gaps 0.3-2 s: the chestnut 2.5 ms slower than 11-06, or the light reads at a signal) held an
      # empty road's left at its line forever, saying "Waiting for a gap", where one that didn't run at all went at once on
      # the full pass's band. Judged on that band from here, as with no far slot, until the far runs hold FAR_FRESH_S again
      self.far_behind, far_needed = True, False
      self.clear_s = self.near_clear_s
      cloudlog.event("turnpilot.left_far_behind", distance=round(inp.distance, 1), far_age=round(min(self.far_age, 99.), 2),
                     r_req=round(rreq, 1), range_m=round(inp.range_m, 1), maneuver_id=inp.maneuver_id)
    out = LeftDecision(t_req=treq, r_req=rreq, v_limit=v_limit, clear_s=self.clear_s, view_blocked=blocked_view,
                       crosswalk_clear=crosswalk, blockers=len(raw_blockers or ()), far_needed=far_needed,
                       far_blockers=len(far_blockers or ()), far_bearing=far_aim(inp.heading_deg, side, inp.range_m),
                       far_range_m=inp.far_range_m if inp.far_live else 0., clearing=self.clearing)
    self.alert_t = max(self.alert_t - dt, 0.)
    if self.alert_t > 0.:
      out.alert = 'oncomingHolding'
    out.chime = self.chime_t > 0.
    self.chime_t = max(self.chime_t - dt, 0.)

    # the driver took the car past its own hold, or past its line (the latch let go as theirs): theirs, never held or gone
    # again (owner decisions (a), (b); module docstring)
    if self.hold_here:
      cause = self._hold_driven(inp, dt)
      if cause:
        return self._driven(inp, out, cause)
    if inp.driven and self.why != DRIVEN:
      return self._driven(inp, out, '')
    if self.why == DRIVEN:
      out = self._undecided(out, DRIVEN)
      out.reason = DRIVEN_REASON.get(inp.side, '')  # every frame until the map's Driven says it (no 'waiting for a gap' flicker)
      return out

    standing = inp.held or self.hold_here
    # committed by its own go and motion, or by the driver driving it (their release, moving): never by a reading before its
    # go (skeptic G4: a reading 6 m nearer while held at the line committed the scene, and the left stopped judging with the
    # latch holding the car, silently, for good; the same while the car still rolled up to its line)
    past = inp.committed or inp.released or committed_past_line(inp.distance, inp.v_ego, True)
    if self.state == 'committed' or (self.went and self.moved and past) or (inp.released and not self.went and not standing):
      if self.state != 'committed':
        # a slow launch judged on (launch_on: the band it needs clear on every road run while the car could stop) committing
        # short of a launch: carried across at the left's post-go profile (CARRY_SPEED), not left to cross at its crawl
        self.carry = self.went and self.moved and self.launch_on and inp.v_ego < LEFT_LAUNCH_SPEED and not inp.released
        cloudlog.event("turnpilot.left_committed", distance=round(inp.distance, 1), v=round(inp.v_ego, 2), carry=self.carry,
                       band=self.blocked)
      self.state, self.hold_here = 'committed', False
      out.state, out.alert, out.carry = 'committed', 'none', self.carry
      return out
    if self.state == 'undecided':
      return self._undecided(out, self.why, inp.v_ego)
    drivers_move = not inp.enabled and inp.v_ego >= DRIVER_MOVING_SPEED  # openpilot disengaged, the car moving: their pedals
    if not self.went and not self.hold_here and \
       (inp.released or (drivers_move and (inp.committed or (standing and inp.distance < HELD_MIN_M)))):
      # the driver took it: their gas at its line (the scene's release, the latch still holding the car this frame), or the
      # car moving into the junction under their own pedals (openpilot disengaged: a scene commit, or the latch's car taken
      # under HELD_MIN_M, the replay's 511 s). Driven, said as the crossing it is (the
      # latch lets go as theirs a frame or more later: map_controls' Driven), never a fault of TurnPilot's (2026-11-08, drive
      # 29 cause 4: 'Junction position unsure: take control' at every left the driver took, on the frame the scene released
      # the hold, the latch letting go as driverWent 0.9 s later). Its own hold after a withdrawn go has its own release rule
      # (_hold_driven: the owner decisions' metres), above
      return self._driven(inp, out, '')
    if inp.committed and not self.went:
      # the scene committed to the junction before the left's own go by a reading, engaged, the car rolling up to its line or
      # standing held short of it: where the car is isn't known. Out loud, never a freeze
      return self._undecided(out, 'position')

    if self.went:
      why = 'band' if raw_blockers or far_blockers else 'beside' if beside else 'cross' if closing else 'crosswalk' if not crosswalk else \
        'view' if blocked_view else 'hold' if inp.other_hold or not inp.scene_clear else \
        'light' if light in ('red', 'yellow') and not self.clearing else ''
      return self._after_go(inp, out, why, len(raw_blockers or ()) + len(far_blockers or ()), dt)

    if not standing:
      out.state = self.state = 'approach'
      out.view = 'road' if inp.distance <= VIEW_FROM_M else 'none'
      return out
    if self.latch_d is None:
      # held at the line. The window is the approach's too (WINDOW_FROM_APPROACH): a band clear on every road run of the last
      # CLEAR_WINDOW_S of the roll-in, nothing else holding the car, goes as it comes to its stand
      self.latch_d, self.far_stale, self.far_only_s = inp.distance, 0., 0.
      if not self.window_from_approach:
        self.clear_s = self.near_clear_s = out.clear_s = 0.
        self.far_behind = False
      # its stop line: at a left held in the junction's mouth (line_m, a mapped signal) the car stands past it, and a red read
      # there is said as a red in the junction is (LS-4's words, heard once), and the car clears the junction on its gap
      self.line_d = LINE_D if inp.line_m < LINE_D else min(inp.distance, LINE_D)
      self.band_stand_s = self.exit_stand_s = self.wait_s = 0.
      self.said = set()
      # once per hold at its line: how far the band it is judged on reaches against R_req (the camera's reach, the road's
      # straight run), and what it would once have handed back for ('night', 'range', 'farUnmeasured', 'farOff', 'curve')
      band_m = min(range_eff, rreq, inp.distance + max(inp.straight_m, 0.))
      short_of = '' if rreq <= range_eff else reach_fault(inp, rreq)
      if not short_of and inp.distance + max(inp.straight_m, 0.) < rreq:
        short_of = 'curve'
      cloudlog.event("turnpilot.left_reach", r_req=round(rreq, 1), band_m=round(band_m, 1), range_m=round(inp.range_m, 1),
                     far_range_m=round(inp.far_range_m, 1), far_live=inp.far_live, far_needed=far_needed, dark=inp.dark,
                     straight_m=round(inp.straight_m, 1), short_of=short_of, wrong_lane=inp.wrong_lane,
                     distance=round(inp.distance, 1), maneuver_id=inp.maneuver_id)
    certified = self.clear_s >= CLEAR_WINDOW_S - 1e-6 and not blocked_view and crosswalk and inp.scene_clear and not held_else
    out.view = 'roadAndWide'
    # a left LC-03 placed in a lane it doesn't leave from (wrong_lane) once held at its line is judged from the lane it is in,
    # never handed back for it; one flagged before that turnpilotd skips and reroutes (2026-11-08, drive 29)
    if inp.signal_veto:  # the driver's lamp the other way, hazards or our lamp put out, at the line: a handback, car held
      return self._undecided(out, 'driverSignal')
    # no lamp lit on its side: it waits at its line, the side's chime repeating (car_signals' prompt), and goes on a gap once
    # the driver lights it; said once a dark spell past LEFT_SIGNAL_WAIT_S, never handed back nor gone unsignalled (the constants)
    self.unsignalled_s = self.unsignalled_s + dt if dark else 0.
    if self.unsignalled_s >= LEFT_SIGNAL_WAIT_S - 1e-6:
      self._said('unsignalled', "turnpilot.left_unsignalled", inp, unsignalled_s=round(self.unsignalled_s, 1))
    elif not dark:
      self.said.discard('unsignalled')  # lit: a later dark spell as long is another, said again
    if inp.junction_uncertain or inp.distance < HELD_MIN_M:
      # where the car is isn't known: before anything is judged from it (a roll-up included); or a reading puts the junction
      # too near for the go's first-motion re-check to stop the car short of the oncoming lanes (HELD_MIN_M)
      return self._undecided(out, 'position')
    # held short of its line (the latch reaches LATCH_NEAR_M short of it), where the longer crossing puts R_req past the
    # range: rolled up to the line first (the latch's creep), where it is judged (BeamNG: held 16 m out, R_req 164 m)
    short = inp.distance - inp.line_m
    roll_up = inp.mode == AUTO and rreq > range_eff and short > ROLL_UP_M and not self.hold_here
    if roll_up and not held_else:
      self.roll_stall = self.roll_stall + dt if inp.v_ego < 0.3 else 0.
      if self.roll_stall < ROLL_UP_STALL_S - 1e-6:
        self.creep_to = max(self.latch_d - inp.line_m, 0.)
        out.state = self.state = 'creep'
        out.prompt, out.reason, out.creep_m = 'creepingForView', "Rolling up to the line", self.creep_to
        return out
    # R_req past the camera's reach (range, the far slot off or unmeasured, after dark) or past where the opposing road bends
    # out of the band: judged on the band the camera covers, out to min(range_eff, R_req, the straight run), a car it sees there
    # holding the left; never a handback for what it can't see (Kellen, 2026-11-07; the safety driver supervises). Logged per
    # left at its line (turnpilot.left_reach)
    if self.road_age > STALE_UNDECIDED_S:
      return self._undecided(out, 'stale')
    # yawed past what the road camera frames of the band (after a withdrawn go): judged at the measured heading on what it does
    # frame, said once a hold, heard (YAW_VIEW_DEG; the 'yaw' handback until 2026-11-08)
    yaw_view = abs(inp.heading_deg) > YAW_VIEW_DEG
    if yaw_view:
      self._said('yaw', "turnpilot.left_yaw_view", inp, chime=True, heading_deg=round(inp.heading_deg, 1),
                 view_m=round(min(yaw_view_m(inp.heading_deg), 999.), 1), r_req=round(rreq, 1), hold=self.hold_here)
    # a light holding it at its line (a red or yellow read, or the stop one raised still in force: turn_light; unknown where
    # SIGNAL_NEEDS_GREEN): waited through for as long as it holds, its window starting over (held_else), gone on the green
    # that ends it, or on the gap once a red that raised no stop (the camera's in shadow) is no longer read. Until 2026-11-07 a
    # change from the green it stood on (lightChange: "the car is in the junction as it does") handed back at once, and an
    # unknown past LIGHT_UNKNOWN_S or a red past LIGHT_RED_MAX_S ('light'); now each is said once and the car waits where it
    # stands: at its line or its creep, CREEP_SHORT_M or more short of the oncoming lanes, never into them on a red
    # the model's own hold (model_holds) is a light wait too, said once at STAND_UNDECIDED_S (the plain long wait's bound): a
    # wait that changes kind (a red, then the model holding at the green) is another, counted afresh
    light_wait_now = light in ('red', 'yellow') or (light == 'unknown' and SIGNAL_NEEDS_GREEN)
    red_now = light in ('red', 'yellow')
    if light == 'green':
      self.saw_green, self.changed_crept, self.changed_read = True, False, False
    if light_wait_now or inp.model_holds:
      kind = ('red' if red_now else light) if light_wait_now else 'model'
      if kind != self.light_kind:
        self.light_kind, self.light_s = kind, 0.
        self.said.discard('light')
      changed = red_now and self.saw_green
      if changed:
        self.saw_green = False  # said once a change: the next green, then a red, is another
        crept_m = self.latch_d - inp.distance if self.latch_d is not None else 0.
        cloudlog.event("turnpilot.left_light_changed", light=light, distance=round(inp.distance, 1), creep_m=round(self.creep_to, 1),
                       crept_m=round(crept_m, 2), v=round(inp.v_ego, 2), maneuver_id=inp.maneuver_id)
      past_m = self._past_line_m(inp)
      if red_now and not self.changed_crept and self.clearing:
        # review 2026-11-07 (LS-4): crept for a view on the green, the car's nose 3-7 m from the junction's centre, held
        # where it stands through the red (the read red's rail) with "Waiting for a Gap / Waiting for the light", silent:
        # on a cross road of 2 lanes or more each way that is in the near cross lanes through their green. Said in plain
        # words with a chime for the safety driver, never a handback. Kellen, 2026-11-08: it clears the junction on its gap
        # (self.clearing: the light no hold, its go lifted over it), a car seen in the band holding it until it passes.
        # LEFTS-3: on a light it never read green too (a mapped signal with no colour read is 'none': it creeps on it, at
        # night the norm), the first red or yellow with the car past its line (line_d, never the metres since the latch: a
        # roll-up to the line is no creep into the junction); its words then say what was read, no change
        self.changed_crept, self.changed_read = True, changed
        self.chime_t = WAIT_CHIME_S
        cloudlog.event("turnpilot.left_light_in_junction", light=light, changed=changed, past_m=round(past_m, 2),
                       distance=round(inp.distance, 1), v=round(inp.v_ego, 2), maneuver_id=inp.maneuver_id)
      self.light_s += dt
      # review 2026-11-07 (AL-2): its red no longer read (the stop it raised in force, or the left's own latch: turn_light) is
      # said as a through signal's is (map_controls' SIGNAL_UNSEEN), SIGNAL_UNSEEN_S from the loss, once a loss, its line the
      # unseen words while it lasts, the car still held for a green; LIGHT_RED_MAX_S is for a red still read
      self.unseen_s = self.unseen_s + dt if red_now and not inp.light_read else 0.
      if self.unseen_s >= SIGNAL_UNSEEN_S - 1e-6:
        self._said('unseen', "turnpilot.left_light_unseen", inp, chime=True, light=light, unseen_s=round(self.unseen_s, 1),
                   light_s=round(self.light_s, 1))
      elif self.unseen_s == 0.:
        self.said.discard('unseen')  # read again: a later loss as long is another, said again
      bound = LIGHT_RED_MAX_S if kind == 'red' else LIGHT_UNKNOWN_S if kind == 'unknown' else STAND_UNDECIDED_S
      if self.light_s >= bound - 1e-6 and (kind != 'red' or inp.light_read):
        self._said('light', "turnpilot.left_light_wait", inp, chime=True, light=light if light_wait_now else 'model',
                   light_s=round(self.light_s, 1))
    else:
      self.light_s, self.light_kind, self.unseen_s = 0., '', 0.
      self.said -= {'light', 'unseen'}  # a light wait as long after this is another, said again

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
    if certified and inp.enabled:
      # never while openpilot is disengaged (the driver's pedal): a go then would let go of the latch under the driver, a
      # crossing nobody certified (owner decision (b), 2026-10-30). The window goes on counting: re-engaged, it goes on it
      return self._go(inp, out, rreq)
    # every wait at its line for a gap, whatever the tracker says, is said once at STAND_UNDECIDED_S and goes on, the
    # car held, its line saying what it waits for ('standing' where the tracker says something stands); another hold keeping
    # the car pauses it. Until 2026-11-07 the noGap or standing handback there (owner decision (ii))
    long_wait = ''
    if not held_else:
      self.wait_s += dt
      band_standing = self.blocked and self.band_stand_age <= CLEAR_WINDOW_S
      self.band_stand_s = self.band_stand_s + dt if band_standing else 0.
      self.exit_stand_s = self.exit_stand_s + dt if exits and all(stands_still(o) for o in exits) else 0.
      if self.wait_s >= STAND_UNDECIDED_S - 1e-6:
        long_wait = 'standing' if max(self.band_stand_s, self.exit_stand_s) > 0. else 'noGap'
        # heard only where something stands (it won't end on its own); a stream with no gap is the busy road's ordinary wait
        self._said('noGap', "turnpilot.left_no_gap", inp, chime=long_wait == 'standing', why=long_wait, wait_s=round(self.wait_s, 1),
                   band_s=round(self.band_stand_s, 1), exit_s=round(self.exit_stand_s, 1), blockers=len(raw_blockers or ()),
                   hold=self.hold_here, exits=[(str(o.cls), round(float(o.x), 1), round(float(o.y), 1)) for o in exits])
    if blocked_view and not self.blocked and not self.hold_here and not held_else:
      self.creep_to = creep_max
      done = crept >= creep_max - CREEP_DONE_M
      standing = inp.v_ego < 0.3
      self.blocked_s = self.blocked_s + dt if done and standing else 0.
      # asked to creep and standing short of it with nothing else holding the car: the model won't pull away (skeptic G4: 270 s
      # in 'creep', visual only). Either way the creep stays asked (the model may yet take it) and the line says the wait
      self.creep_stall = self.creep_stall + dt if not done and standing and inp.scene_clear else 0.
      view_wait = self.blocked_s >= BLOCKED_UNDECIDED_S - 1e-6 or self.creep_stall >= CREEP_STALL_S - 1e-6
      if view_wait:
        why = 'blocked' if done else 'creepStall'
        self._said(f'view:{why}', "turnpilot.left_view_wait", inp, chime=True, why=why, crept=round(crept, 1), creep_to=round(self.creep_to, 1),
                   blocked_s=round(self.blocked_s, 1), stall_s=round(self.creep_stall, 1))
      self.far_only_s = 0.
      out.state = self.state = 'creep'
      out.prompt, out.reason = ('waitingForGap', LONG_WAIT_REASON['view']) if view_wait else ('creepingForView', "Creeping for a view")
      out.creep_m = self.creep_to
      return out
    # waiting: crept no further than it has (a car in the band stops the creep where it is)
    self.blocked_s = self.creep_stall = 0.
    self.said -= {'view:blocked', 'view:creepStall'}  # out of the creep: a later view wait as long is another, said again
    # on the far crop's boxes alone (traffic within the full pass's range starts it over), nothing else holding the car: a
    # car standing far up the road in the band is never an endless silent wait (skeptic G6): said, and waited on
    far_only = far_needed and self.far_block_age <= FAR_BLOCK_RECENT_S and not held_else
    self.far_only_s = self.far_only_s + dt if far_only else 0.
    if not far_only:
      self.said.discard('farBlocked')
    if self.far_only_s >= FAR_BLOCKED_UNDECIDED_S - 1e-6:
      long_wait = 'farBlocked' if long_wait in ('', 'noGap') else long_wait  # what holds it, over the plain long wait
      self._said('farBlocked', "turnpilot.left_far_wait", inp, chime=True, far_only_s=round(self.far_only_s, 1),
                 far_blockers=len(far_blockers or ()), r_req=round(rreq, 1))
    self.creep_to = min(self.creep_to, max(crept, 0.))
    out.state = self.state = 'waitGap'
    light_wait = light in ('red', 'yellow') or (light == 'unknown' and SIGNAL_NEEDS_GREEN)
    # waiting for the driver's lamp, the left's own prompt stands aside for car_signals' "Signal LEFT Now" and its chime on
    # repeat: the waitingForGap prompt is selfdrived's newer alert at the same priority, and over a wait at the line it hid the
    # side's prompt and silenced its chime (a Sim run through selfdrived's AlertManager, 2026-11-07: "Signal LEFT Now" from
    # the 300 ft arm, then the silent "Unprotected left: waiting for the turn signal" from the line on). Its line says the wait
    out.prompt, out.creep_m = 'none' if no_signal else 'waitingForGap', self.creep_to
    in_junction = LIGHT_CHANGED_REASON if self.changed_read else LIGHT_IN_JUNCTION_REASON
    out.reason = NO_SIGNAL_REASON if no_signal else CROSS_REASON[self.cross] if self.cross else \
      in_junction if light_wait and self.changed_crept else \
      SIGNAL_UNSEEN_REASON if light_wait and self.unseen_s >= SIGNAL_UNSEEN_S - 1e-6 else \
      LIGHT_WAIT_REASON[light] if light_wait else \
      MODEL_HOLD_REASON.get(inp.model_holds, MODEL_HOLD_REASON['']) if inp.model_holds and not inp.other_hold else \
      BESIDE_REASON[side] if beside else LONG_WAIT_REASON['yaw'] if yaw_view and long_wait in ('', 'noGap') else \
      LONG_WAIT_REASON.get(long_wait, "Waiting for a gap")
    if self.hold_here:
      # its own hold says the withdrawal's words (turnpilotd's line is the stop's reason first), but for the turned car's view:
      # what the driver needs to know while it is judged on part of the oncoming road (YAW_VIEW_DEG)
      words = out.reason if out.reason == LONG_WAIT_REASON['yaw'] else self.hold_reason
      out.stops = [stop(inp.v_ego, 0., words, confirmed=False)]
    return out

  def _past_line_m(self, inp: LeftInput) -> float:
    """m the car is past its stop line: the mapped signal's own (signal_line_m), else the nearer of its first latch and LINE_D
    (a left held in a signal's mouth, or crept past its line for a view)."""
    if inp.signal_line_m is not None:
      return inp.signal_line_m - inp.distance
    line_d = self.line_d if self.latch_d is not None and self.line_d is not None else LINE_D if inp.line_m < LINE_D else \
      min(max(inp.line_m, inp.distance), LINE_D)
    return line_d - inp.distance

  def _said(self, key: str, event: str, inp: LeftInput, chime: bool = False, **fields) -> None:
    """A long wait at the line (the module docstring: what handed back until 2026-11-07), logged once for key as its old bound
    passes: never a frame's log, never a handback. A new hold at the line, a go or its own hold after a withdrawn go says it
    afresh, and so does the same wait come round again once what held it has let go (the key discarded there: the lamp lit,
    the light no longer holding, out of the creep, the far wait over). chime: heard too, once (LeftDecision.chime for
    WAIT_CHIME_S under the silent waitingForGap or creepingForView: events.py), for a wait that won't end on its own: something
    standing in the way, the far crop's car, the view, the light (review 2026-11-07, SF-4: each was shown and never heard, the
    state skeptic g4c rejected with different words). Never the plain wait for a gap in a moving stream: on a busy road that
    is the ordinary wait, and a chime there would nag."""
    if key in self.said:
      return
    self.said.add(key)
    if chime:
      self.chime_t = WAIT_CHIME_S
    cloudlog.event(event, error=True, distance=round(inp.distance, 1), v=round(inp.v_ego, 2), maneuver_id=inp.maneuver_id, **fields)

  def _go(self, inp: LeftInput, out: LeftDecision, rreq: float) -> LeftDecision:
    self.went, self.moved, self.hold_here, self.go_t, self.far_only_s = True, False, False, 0., 0.
    self.band_stand_s = self.exit_stand_s = self.wait_s = 0.
    self.go_d, self.launch_on = inp.distance, False
    self.said = set()
    cloudlog.event("turnpilot.left_go", distance=round(inp.distance, 1), r_req=round(rreq, 1), clear_s=round(self.clear_s, 1),
                   v_limit=round(out.v_limit, 1), heading_deg=round(inp.heading_deg, 1))
    out.state = self.state = 'go'
    out.go, out.view, out.prompt, out.reason = True, 'road', 'leftGo', "Turning"
    if self.quiet_go:
      # review 2026-11-07 (AL-1): a go of this hold the model didn't follow (go_unfollowed) is retried once its plan moves,
      # never bounded (Kellen: no arbitrary limits), and each retry chimed "Turning / Turning across oncoming traffic" with
      # the car standing (15 in 60 s at a model that holds once the pulse is out). A retry is silent and says what holds the
      # car until it moves; then it is said and heard as any go (_after_go: the car's first motion)
      out.prompt, out.reason = 'waitingForGap', MODEL_HOLD_REASON['']
    return out

  def _after_go(self, inp: LeftInput, out: LeftDecision, why: str, blockers: int, dt: float) -> LeftDecision:
    """Gone, not committed: the road camera only, and on every frame until the commit what the go was certified on checked
    again (why: what no longer holds, '' if it all does): the band on every road run for the remaining clear time (at the
    first motion with the latency gone), the exit crosswalk, the view, nothing else holding the car, the light not gone red
    or yellow (skeptic G4: a go that stood through a red and a walker in the crosswalk ran stale when they let go). Any of
    them withdraws it while the car can stop short of the oncoming lanes: its own hold, a fresh window (module docstring)."""
    self.go_t += dt
    out.state, out.view = 'go', 'road'
    if self.quiet_go and not self.moved and inp.v_ego >= 0.3:
      self.quiet_go, self.go_t = False, dt  # a quiet retry the car moves on: said and heard from here, as a go (AL-1)
    if self.quiet_go:
      out.prompt, out.reason = 'waitingForGap', MODEL_HOLD_REASON['']
    elif self.go_t < GO_PROMPT_S:
      out.prompt, out.reason = 'leftGo', "Turning"
    stop_m = inp.v_ego ** 2 / (2. * STOP_DECEL)
    if inp.go_withdrawn and not inp.signal_veto:  # turnpilotd's: not moving, held and handed back there
      out.withdrawable = True
      return out
    if not self.moved and (inp.junction_uncertain or inp.distance < HELD_MIN_M):
      # not yet moved on the go, the junction's position unsure or a reading too near for the first-motion re-check
      # (HELD_MIN_M): never on, withdrawn where the car stands (it can always stay) and handed back, held
      cloudlog.event("turnpilot.left_go_withdrawn", error=True, why='position', first_motion=False, distance=round(inp.distance, 1),
                     v=round(inp.v_ego, 2), spread=round(inp.spread_m, 1), r_req=round(out.r_req, 1))
      self._hold(inp, inp.v_ego ** 2 / (2. * STOP_DECEL), UNDECIDED_REASON['position'])
      return self._undecided(out, 'position', inp.v_ego)
    first_motion = not self.moved and inp.v_ego >= 0.3
    self.moved = self.moved or first_motion
    # can't stop short of the oncoming lanes: on (committed_past_line commits it next). Not yet moved, it stands where it was
    # held: it can always stay (a reading nearer than the line never makes it go on)
    can_stop = not self.moved or stop_m < inp.distance - ONCOMING_ENTRY_M
    out.withdrawable = inp.go_withdrawn or not self.moved or stop_m + VETO_MARGIN_M < inp.distance - ONCOMING_ENTRY_M
    if inp.signal_veto:
      # the driver overruled the car's left while this go could still be withdrawn (last frame's withdrawable, the request for
      # the left ended there: car_signals.py): withdrawn and handed back, held where the comfortable curve stops the car
      if can_stop or inp.go_withdrawn:
        cloudlog.event("turnpilot.left_go_withdrawn", error=True, why='driverSignal', veto=inp.signal_veto,
                       first_motion=first_motion, distance=round(inp.distance, 1), v=round(inp.v_ego, 2), r_req=round(out.r_req, 1))
        self._hold(inp, stop_m, UNDECIDED_REASON['driverSignal'])
        return self._undecided(out, 'driverSignal', inp.v_ego)
      # past VETO_MARGIN_M's frame: never a stop in the oncoming lanes, the turn finishes (its request already ended)
      cloudlog.event("turnpilot.left_veto_past_return", error=True, veto=inp.signal_veto, distance=round(inp.distance, 1),
                     v=round(inp.v_ego, 2), stop_m=round(stop_m, 2))
    room = inp.distance - ONCOMING_ENTRY_M - stop_m
    if not why and not self.launch_on and self.moved and inp.launch_armed and inp.v_ego < LEFT_LAUNCH_SPEED and \
       0. <= room < LEFT_LAUNCH_ROOM_M + HELD_MIN_M - ONCOMING_ENTRY_M:
      # review 2026-11-07 (L3): rolling toward the oncoming lanes short of a launch with only LEFT_LAUNCH_ROOM_M left to stop
      # short of HELD_MIN_M (so a car stopped here stands where its next go is judged: a go from nearer is the position
      # fault): its clear time was for its own launch, not a gentle one nor a coast on the lift's speed. Judged again
      # for the launch the car shows, kept for the rest of the left (its later road runs and gos: a longer R_req), as the go
      # is: on the band the camera covers out to that R_req (a car the last road run or a recent far run saw there). Nothing
      # seen: the car keeps driving, that band checked on every road run while it can still stop; a car seen: withdrawn to
      # its own wait where the car stops short of the lanes. turnpilotd handed back here until then (cantStartTakeControl, a
      # give-up). Round 2 (2026-11-08): an R_req past the camera's reach withdrew it too, stricter than the go's own rule
      # (judged on the band it covers: the module docstring), so on the car's 98 m full pass a 0.8 m/s^2 launch HEAD turned
      # was withdrawn and, standing yawed past 15 deg, handed back ('yaw'); a coast is judged the same way
      moved_m = max(self.go_d - inp.distance, 0.05)
      launch_a = inp.v_ego ** 2 / (2. * moved_m)
      self.launch_a = min(self.launch_a, max(launch_a, LAUNCH_A_MIN))
      d_far = inp.distance + max(inp.spread_m, 0.)
      rreq = r_req(out.v_limit, t_req(d_far, inp.opposing_lanes, False, self.launch_a), d_far)
      seen = band_blockers(self.road_boxes, inp.heading_deg, inp.side, rreq, inp.distance, lanes=inp.opposing_lanes) or \
        (self.far_blocked and self.far_run_age <= CLEAR_WINDOW_S)
      self.launch_on = not seen
      cloudlog.event("turnpilot.left_not_launched", error=not self.launch_on, on=self.launch_on, v=round(inp.v_ego, 2),
                     distance=round(inp.distance, 1), room=round(room, 2), launch_a=round(launch_a, 2), moved_m=round(moved_m, 2),
                     r_req=round(rreq, 1), range_m=round(self.range_eff, 1), seen=bool(seen))
      if not self.launch_on:
        why = 'launch'
    if not why and inp.go_unfollowed and not self.moved:
      # turnpilotd's: its unlifted go stood LEFT_NO_MOVE_S with the model planning no motion (review 2026-11-07, LS-2: the
      # cantStartTakeControl handback here until then): back to a wait at its line, a fresh window, and model_holds keeps the
      # next go until the model's plan moves
      why = 'model'
    if not why:
      return out
    if not can_stop:
      return out
    # the model's own hold: what holds it, the light only where one is known or boxed (MODEL_HOLD_REASON, AL-1)
    reason = MODEL_HOLD_REASON.get(inp.model_holds, MODEL_HOLD_REASON['']) if why == 'model' else WITHDRAWN_REASON[why]
    cloudlog.event("turnpilot.left_go_withdrawn", error=True, why=why, first_motion=first_motion, distance=round(inp.distance, 1),
                   v=round(inp.v_ego, 2), blockers=blockers, r_req=round(out.r_req, 1))
    self._hold(inp, stop_m, reason)
    if why == 'model':
      self.quiet_go = True  # its retries silent until the car moves (AL-1)
    out.state = self.state = 'waitGap'
    out.prompt, out.reason, out.view, out.clear_s = 'waitingForGap', reason, 'roadAndWide', 0.
    if why == 'band':
      self.alert_t = ALERT_S
      out.alert = 'oncomingHolding'
    out.stops = [stop(inp.v_ego, stop_m, reason, confirmed=False)]
    return out

  def _hold(self, inp: LeftInput, stop_m: float, reason: str) -> None:
    """Its own hold from here (hold_here: after a withdrawn go), its stop stop_m on, saying reason; never a go standing on
    it (a fresh window), its release rule's metres from here (_hold_driven)."""
    self.went, self.moved, self.hold_here, self.clear_s, self.hold_reason = False, False, True, 0., reason
    self.latch_d = inp.distance
    self.hold_m = self.hold_drv_m = self.hold_drv_past = 0.
    self.hold_asked, self.hold_entry = max(stop_m, 0.), inp.distance - ONCOMING_ENTRY_M
    self.hold_drv_frame = False
    self.said = set()

  def _hold_driven(self, inp: LeftInput, dt: float) -> str:
    """Its own hold's release rule (module docstring; map_controls._held's, owner decisions (a), (c)): the driver's crossing's
    cause, on the driver's own frames (openpilot disengaged), once they have moved the car past where TurnPilot's stop asked
    it to stand by more than LATCH_PAST_M or into the oncoming lanes; or at the re-engage after them, judged from its frame's
    speed and where the car is, the driver having moved it, once the hold taken up there could no longer stand it short of
    the oncoming lanes (reengage_stop_m, its own braking); '' while the hold stands. Position-free (the car's metres, never a
    reading). Engaged past the re-engage, the plan is TurnPilot's: however far its own stop carries the car is never the
    driver's, and nothing is let go on it (skeptic round 1: a re-engage inside the ask was let go a frame or two later on
    TurnPilot's braking metres, and openpilot drove the car on into the junction, the full check (c) skipped)."""
    v = max(inp.v_ego, 0.)
    self.hold_m += v * dt
    cause = 'driverWent' if self.state == 'undecided' else 'drivenOff'
    if not inp.enabled:
      self.hold_drv_m += v * dt
      self.hold_drv_frame = True
      # only what the driver moves it past the ask: metres inside it are TurnPilot's, whoever drove them, and so are those
      # TurnPilot's own stop carried it past the ask, engaged (never counted here); the driver's own from there on count
      self.hold_drv_past += max(min(v * dt, self.hold_m - self.hold_asked), 0.)
      # past the ask by the reach of the driver's own metres, or taken into the oncoming lanes by them (a hold there could
      # only stand the car in them, and it never comes back short of them): theirs, for good
      if self.hold_drv_past > LATCH_PAST_M or (self.hold_drv_m > 0. and self.hold_m > self.hold_entry):
        return cause
      return ''
    if not self.hold_drv_frame:
      return ''
    # the re-engage, the first frame TurnPilot's plan applies after the driver's: the invariant, never a TurnPilot hold that
    # stands the car in the oncoming lanes on a re-engage, judged from this frame's speed and where the car is, before the hold
    # brakes it (never on TurnPilot's own metres: skeptic round 1). Too fast to stand short of them (keeping the speed inside
    # a stop asked late in a launch: e93750b stood it 0.5 m inside them): let go; short of that, held for the full check (c),
    # whatever the speed on the driver's frames before (skeptic round 2: judged there, a car braked to a stand short of the
    # lanes after it was let go, and a re-engage pulled it away from rest across them uncertified)
    self.hold_drv_frame = False
    if self.hold_drv_m > 0. and self.hold_m + reengage_stop_m(v, hold_caps) > self.hold_entry:
      return cause
    return ''

  def _driven(self, inp: LeftInput, out: LeftDecision, cause: str) -> LeftDecision:
    """The driver took the car past its own hold (cause: its crossing, said by map_controls' Driven) or past its line (the
    latch let go as theirs, already said): done, undecided (DRIVEN), never a stop, a go or a turn pulse again."""
    cloudlog.event("turnpilot.left_driven", error=True, cause=cause or 'line', hold=self.hold_here, distance=round(inp.distance, 1),
                   v=round(inp.v_ego, 2), hold_m=round(self.hold_m, 2), driver_m=round(self.hold_drv_m, 2),
                   driver_past_m=round(self.hold_drv_past, 2), asked_m=round(self.hold_asked, 2), was=self.state)
    out.crossed = cause
    self.went = self.moved = self.hold_here = False
    self.alert_t = 0.
    out = self._undecided(out, DRIVEN)
    # the crossing's own words (map_controls' Driven says them from its latch's let-go on), on any frame the latch still has
    # the car under the driver's gas: never a fault's take-control (turnpilotd: no take_control for DRIVEN)
    out.reason = DRIVEN_REASON.get(inp.side, '')
    return out

  def _undecided(self, out: LeftDecision, why: str, v_ego: float = 0.) -> LeftDecision:
    """A fault's handback (UNDECIDED_FAULTS: never a wait, since 2026-11-07), the car held. After a withdrawn go (hold_here)
    its own hold where the comfortable curve stops the car from v_ego (a car still rolling is never told shouldStop), as
    waitGap's after a withdrawal. DRIVEN: the driver's crossing, no hold, its prompt the crossing's (map_controls.Driven)."""
    if self.state != 'undecided':
      cloudlog.event("turnpilot.left_undecided", error=True, why=why, r_req=round(out.r_req, 1), clear_s=round(out.clear_s, 1))
    self.state, self.why = 'undecided', why
    out.state, out.why, out.view = 'undecided', why, 'roadAndWide'
    if why == DRIVEN:
      out.alert = 'none'
      return out
    out.prompt, out.reason = 'leftUndecidedTakeControl', UNDECIDED_REASON.get(why, "Can't judge oncoming traffic: take control")
    if self.hold_here:
      # the fault's words on its hold's stop too (turnpilotd's line is the stop's reason first): under take control, never
      # the withdrawal's ('Car started too slowly, holding' under leftUndecidedTakeControl until 2026-11-08)
      out.stops = [stop(v_ego, 0., out.reason, confirmed=False)]
    return out


def reach_fault(inp: LeftInput, rreq: float) -> str:
  """Why R_req is past the reach (the module docstring): after dark past NIGHT_LEFT_RANGE_M ('night', whatever the ranges
  measured by day), the far crop's range unmeasured, past it too, or the far crop off. The full pass's range unmeasured is
  'range': the far crop covers the band only past it."""
  if inp.dark and rreq > NIGHT_LEFT_RANGE_M:
    return 'night'
  if inp.range_m <= 0.:
    return 'range'
  if inp.far_range_m <= 0.:
    return 'farUnmeasured'
  if rreq > inp.far_range_m:
    return 'range'
  return 'farOff' if not inp.far_live else 'range'


WITHDRAWN_REASON = {
  'band': "Oncoming, holding",
  'crosswalk': "Crosswalk not clear, holding",
  'view': "View of oncoming traffic blocked, holding",
  'hold': "Holding",
  'light': "Light changing, holding",
  'model': "Waiting for the car to pull away",  # _after_go says MODEL_HOLD_REASON's for what holds it (AL-1)
  'beside': "Car beside, holding",
  'launch': "Car started too slowly, holding",
  'cross': "Cross traffic coming, holding",
}
# the left's line while a car in the lane it crosses holds it (crossed_lane_cars, LS-1), by the turn's side (LEFT 1, RIGHT -1)
BESIDE_REASON = {1: "Waiting: car beside on the left", -1: "Waiting: car beside on the right"}
# what the left's line says while the model's own hold keeps it at its line (LeftInput.model_holds: the go wouldn't be lifted
# over that hold, and the model plans no motion): at a signal, or a junction the camera boxes a light at, the light; else the
# car's own hold (plannerd's lift refused for the room to the oncoming lanes, TurnPilotGoOverride off, or only a head boxed on
# the approach: 'heads', no light known at the junction, LEFTS-4)
MODEL_HOLD_REASON = {'light': "Waiting at the light", 'heads': "Waiting for the car to pull away", '': "Waiting for the car to pull away"}

# what the driver is told at the line for a fault, the car held (selfdrived: "Unprotected Turn: Take Control", the handback
# chime repeating). The reach faults say what it means for them, never the camera's internals: on the car TurnPilotLeftFarRangeM
# is unmeasured, so every reach handback by day read 'Far camera range not measured', which meant nothing to Chase (11-06).
# Raised since 2026-11-07 (Kellen: "no, no handbacks") only for 'position', 'stale', 'farStale' and 'driverSignal': the
# reach's ('night', 'range', 'farUnmeasured', 'farOff', 'curve'), LC-03's 'wrongLane', 'yaw' (since 2026-11-08: YAW_VIEW_DEG)
# and the waits ('noGap', 'standing', 'farBlocked', 'blocked', 'creepStall', 'unsignalled', 'light', 'lightChange') hand back
# no more, the car judged
# on what the camera covers or waiting at its line saying so (LONG_WAIT_REASON, LIGHT_WAIT_REASON, NO_SIGNAL_REASON). Their
# words stay for the readers of older logs
UNDECIDED_REASON = {
  'night': "Dark, can't see oncoming far enough: take control",
  'range': "Can't see oncoming far enough: take control",
  'farUnmeasured': "Can't see oncoming far enough: take control",
  'farOff': "Can't see oncoming far enough: take control",
  'farStale': "Far camera not seeing oncoming traffic: take control",
  'farBlocked': "Can't tell if far traffic is coming: take control",
  'standing': "Blocked by something standing: take control",
  'noGap': "No gap in oncoming traffic: take control",
  'position': "Junction position unsure: take control",
  'curve': "Oncoming road bends out of view: take control",
  'stale': "Camera not seeing oncoming traffic: take control",
  'yaw': "Not facing oncoming traffic: take control",
  'blocked': "View of oncoming traffic blocked: take control",
  'creepStall': "Can't creep for a view: take control",
  'unsignalled': "No turn signal at the line: take control",
  'driverSignal': "Turn signal changed at the line: take control",
  'light': "Can't read the light: take control",
  'lightChange': "Light changing: take control",
  'wrongLane': "Not in the turn lane: take control",
}
UNDECIDED_FAULTS = ('position', 'stale', 'farStale', 'driverSignal')  # the whys still raised (DRIVEN is no handback)

# what the left's line says while it waits at its line past a long wait's old bound (STAND_UNDECIDED_S, FAR_BLOCKED_UNDECIDED_S,
# BLOCKED_UNDECIDED_S, CREEP_STALL_S): plain words for what holds it, a status line under the silent waitingForGap prompt,
# never take control. 'noGap' says what the wait always said
LONG_WAIT_REASON = {
  'noGap': "Waiting for a gap",
  'standing': "Waiting: something stopped is in the way",
  'farBlocked': "Waiting for traffic far up the road",
  'view': "Waiting for the view to clear",
  'yaw': "Waiting: oncoming road partly out of view",  # standing yawed past YAW_VIEW_DEG, nothing else said
}


def left_mode_for(value) -> str:
  """TurnPilotAutoLeft's value as a mode; AUTO for anything unset or unknown (turnpilotd logs it)."""
  value = value.decode() if isinstance(value, bytes) else value
  return value if value in MODES else AUTO

