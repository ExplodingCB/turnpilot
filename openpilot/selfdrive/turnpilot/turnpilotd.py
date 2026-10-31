#!/usr/bin/env python3
"""turnpilotd: turns the active route and the scene into desire pulses, lane-change requests and longitudinal limits.

modeld takes the plan's desire and lane change, plannerd its accel and speed caps (see hooks.py). The lane paint the
detector reads comes in with its detections, and is used only while those are fresh (the scene is available) and the
message's lanePaintStatus is read (plan_paint); none read is unknown paint, and no lane change. The lane planner plans
lane changes only with TurnPilotLanePaintActive set; without it a second planner publishes what it would have asked for
in laneChangePaintRule, so drives can be scored. In shadow mode
(TurnPilotShadowMode) everything is computed and published with shadow=True, and nothing acts on it.
It runs onroad only while TurnPilotEnabled. The manager doesn't restart a process that died, and a dead one
soft-disables openpilot for the rest of the drive (processNotRunning), so an exception in the plan or the scene is
logged and a disabled plan or an empty scene goes out instead.

TurnPilot acts only while it can do its job (Kellen, 2026-10-27: no degraded mode, and every failure loud):
- Not until the detector's first fresh results after TurnPilot is turned on (state unavailable, "waiting for the
  detector"): modeld loads it only while disengaged, so a mid-drive enable can't be handed back before it ever ran.
  A detector with no results DETECTOR_START_DEADLINE_S of modelV2 frames after that (no pkl, a failed load, none for the
  driving model) never started: handback noDetector. modeld publishes no modelV2 while it loads the detector, so the
  load itself doesn't count against it. Nor do frames with openpilot engaged (carControl.enabled) before modeld had
  its chance to load it, LOAD_CHANCE_S disengaged with TurnPilot on: turned on and engaged within modeld's param check,
  it waits for a disengage, and TurnPilot waits with it. Never silently, though: DETECTOR_START_DEADLINE_S from the
  first engaged frame with no detection yet (whatever modeld publishes, engaged or not since), it is noDetector too. A
  load that failed while the driver engaged during it (modeld publishes nothing while it loads) is handed back so.
- Once it has run, DETECTOR_DEADLINE_S without fresh results is a stopped detector. modeld's own skips stay inside it: a
  slow spell forces a run every 0.8 s however long it lasts (modeld_detector.FORCE_RUN_S), a failed run retries after
  1 s. A second failure in a row, or a forced run that itself takes too long, is past it.
- A modelV2 from the small model after one from the big model is a fallback (modelV2.big), for the rest of the drive.
  So is modeld's ChestnutActive False (read in main()): it outlives this process, which TurnPilot turned off and on
  restarts, and covers a big model that failed to load.
On either, plan.handback is set and selfdrived soft-disables (handback.py). TurnPilot stops at once: no new desire,
lane change, signal prompt, skipped-turn reroute or wrong-lane flag, and the maneuver in hand is dropped. A turn whose
desire already went out is never cancelled (desire_timing.py): its desire is held, with no new pulse, and cleared from
the model at the turn's angle as usual; a keep desire is dropped. On a fallback the destination is removed, which
cancels navd's route, once such a turn is over. The scene's stops in force stay, and so does the turn speed cap of a turn
whose desire already went out: a handback must not speed the car up. Nor does the route going away under a committed
turn cancel it (route_follower.py: gaps, navd going quiet, reroutes are dead-reckoned through). Nor does a cap the car
was slowing for going away with the maneuver or the route: from the handback until openpilot stops steering (the soft
disable's end) the speed cap holds the car at no more than its speed then, and at no more than it has slowed to since
(hold_v), or than the driver's gas has taken it to.
A fault (plan()) is a disabled plan; it carries the handback, so a fault never hides one, and the detector's deadlines
and the scene's stops in force run on through it (only turning TurnPilot off resets them). So does a committed turn:
its route and desire state are kept (its desire, dropped by the disabled plan, pulses again only as on re-engaging), its
junction dead-reckoned and its yaw counted through the fault (fault_track), and
the car is held to its speed while faulted (hold_v), as on a handback. A faulted plan carries the
scene's limits (live pedestrians, or those carried through a detector gap, and the junction hold in force where it
was; scene_policy.py), which plannerd acts on (hooks.limits_plan), so a turnpilotd failing every frame still never
drives the car on through a pedestrian. Its desire and lane change are never acted on.
Once the detector is back and TurnPilot acts again, the dropped maneuver is taken up or skipped out loud, never left
half driven (_take_up_dropped).
No gas go anywhere (Kellen, 2026-10-28; DESIGN phase 4, G1): any gas press disengages (DisengageOnAccelerator, on by
default; preflight fails without it, and TurnPilot faults while it is off: Fault.disengageOnAccelerator), which is the
driver taking over, never TurnPilot's go. A gas press with openpilot disengaged (the driver driving) is what releases the
scene's junction hold at its line (scene_policy.py).
A turn the car pulls away into from a standstill (a hold's go, the turn's pulse in that frame) is unverified: in BeamNG it
was taken about half the time (2026-10-28). Past its junction (the follower's pass) having turned under MISSED_TURN_DEG,
its desire not cleared at the turn's angle (a turn under 20 deg clears under MISSED_TURN_DEG, and that one was taken),
it was missed: turnMissed for MISSED_HOLD_S, which selfdrived chimes as a skip ("Turn Missed", "Rerouting") and on which
navd reroutes at once. Nothing brakes for it. Standing more than STALL_PROMPT_S in any route turn in progress (its desire
out and not done; after a go of TurnPilot's for it, done or not), STALL_MIN_DEG to STALL_MAX_DEG into it (from its start,
after a go of TurnPilot's that came in the junction), whoever stopped the car (TurnPilot's go, or the model on its own:
BUILD_FINDINGS D2; under STALL_MIN_DEG too once past the maneuver point, where a red's stop line never is), with no
hold of TurnPilot's and no lead within STOPPED_LEAD_M that the live scene corroborates
(watch_lead: a vehicle track or box at it; one it corroborates holds the car LEAD_STAND_MAX_S at most), it stalled in the
junction: handed back loudly (controlPrompt stalledTakeControl, STALLED_REASON), never held there, and TurnPilot never
goes itself. Moving in such a turn past its hold
line, under half of it turned, the model steering away from it (modelV2.action.desiredCurvature, TURN_ABANDON_S) is a turn
abandoned (BUILD_FINDINGS N1): handed back loudly (turnAbandonedTakeControl) until the driver takes over, nothing steered
or held, the turn's caps kept.
Right-hand traffic, engaged and moving, a yellow line read on the car's right (the lane paint: under US markings the car is
left of the centre line; BUILD_FINDINGS D5-lane, the model keeping to the oncoming lane after wide turns) for WRONG_SIDE_S
of reads is the wrong side of the road: handed back loudly (wrongSideTakeControl) until the driver takes over, nothing
steered or held.
From the go such a turn is driven under its post-go profile (turn_speed.py: PostGo; DESIGN phase 4 section 3): accel and
speed capped by the heading it has turned, DesireTiming.turned_deg at the go and the car's yaw after, until 60 deg or
POST_GO_MAX_M on, and dropped at the pass of a turn missed straight on. TurnPilotPostGoProfile names the profile
(graduated by default; none, flat3, flat43 for the BeamNG sweep), and an unprotected turn's go (unprotected_turn: a left
across the oncoming road) takes TurnPilotLeftGoProfile's (left: 1.5 m/s^2, 4.3 m/s). The caps only lower speed.
A go from the hold line (go_at_line: the car GO_LINE_M or more before the maneuver point, the scene not committed to the
junction nor released into it) the car hasn't pulled away on (STANDSTILL_SPEED) within NO_MOVE_S (TurnPilotNoMoveS, up to
NO_MOVE_MAX_S, for the sweep), LEFT_NO_MOVE_S for an unprotected turn (CRITIQUE
REQUIRED 4: its gap runs out while it stands), counting only while nothing else holds it (no TurnPilot hold, no close lead
the scene corroborates, NO_MOVE_MAX_S from the go at most behind one, no pedal), is the model holding the car: TurnPilot
withdraws the go before the car moves, holding it standing (shouldStop, a 0 speed cap: braking at -2.0 at most, from
under 0.3 m/s; no turn pulse), and hands back loudly (controlPrompt cantStartTakeControl, NO_MOVE_REASON) until the
driver takes over (openpilot disengaged: any gas press); a scene hold that comes and goes meanwhile (a walker crossing)
is no new go. The turn isn't cancelled: its desire state stays, and the
driver drives it. A go in the junction (a walker's hold released partway round) is never withdrawn: stopping there is worse
than finishing the turn, so TurnPilot never holds the car in it; it keeps the post-go caps, which only lower speed, and the
stall's handback without a hold.
The route's unprotected left (map_controls.unprotected_turn) is driven by unprotected_left.py (DESIGN phase 4 G4) under
TurnPilotAutoLeft: auto (the default: Kellen, 2026-10-28, "No it has to ship taking the turn"), announce or off (G1's
handback at the line). Each frame it is fed the hold line's latch and the scene from the last frame, this frame's raw
detector run (main(): SceneTracker.raw), the car's yaw toward the turn (track_yaw) and the opposing road's mapped speed and
lanes, and checked against TurnPilotLeftRangeM (the detector's proven range; unset: unmeasured, ROAD_RANGE_M 0, so every
left is skipped and rerouted). Its command goes to
the latch (map_controls.LeftCommand: hold, creep, go, or a fault's handback), its own hold after a withdrawn go joins the
scene's stops, and no turn pulse goes on its approach (may_start off; turn_skip's pulse_at_go): its go from the line is the
first. It asks modeld for the road camera (turnpilotPlan.detectorView) and publishes its state (unprotectedLeft); its wait
is silent (waitingForGap, creepingForView), its go a chime, and it hands back (leftUndecidedTakeControl) only for a fault.
Its reach past the full pass's range is the far crop's (DESIGN phase 4 G5): modeld's far crop runs (main(): SceneTracker.raw_far)
are fed to it with TurnPilotLeftFarRangeM (the far crop's proven range; unset: unmeasured, 0) while the full pass's latest
farStatus says it reads (watch_far), at route time too; turnpilotPlan.farCropBearing aims it up the oncoming approach while
the left is in hand (unprotected_left.far_aim), else straight ahead. The far crop going off for the drive (farStatus
disabled) is said once (controlAlert farViewOff, a status line) and logged; a left needing it then hands back (farOff).
Its class-9 heads are logged in shadow only (turnpilot.far_light_shadow), never acted on.
With TurnPilotFarPeds on (off by default), main() runs tracker B beside tracker A (scene/tracker_b.py): the same runs, and the
far crop's people starting tracks of its own. Its pedestrians tracker A doesn't hold reach the scene policy alone, as objects
that never confirm: a "pedestrian in path" stop on the comfortable curve at most. turnpilotScene and everything else here are A's.
Its go is TurnPilot's like any hold's: the left's post-go profile and LEFT_NO_MOVE_S below.
TurnPilot's go from a standstill (any of the above: a turn its hold let go of, at the line or in the junction, or the left's)
is certified for plannerd's bounded lift (go_override.py; DESIGN phase 4 decision 1, G5) under TurnPilotGoOverride (on by
default): turnpilotPlan.goCertified, cause turnRelease or unprotectedLeft, asserted on every frame of the go. Not while the
driver is on a pedal or disengaged, the scene isn't live, a hold of TurnPilot's is in force, the model has a lead within
GO_CLEAR_M, or a road user is in the path within it (path_blocker: a track, or a raw box of this frame's run); any of them
coming during the go, TurnPilot handing back, or the left withdrawing its own go withdraws it that frame (withdraw_go), and
it ends on its own at GO_WINDOW_S, GO_END_SPEED or the model's takeover (go_override.GoCertifier). plannerd then lifts the
model's hold, +0.5 m/s^2 at most, while the model alone holds the car. A turnRelease rests on its junction's own
certification, as the left's go rests on its gap: the junction's signal read green by an accepted source, never the
camera's shadow green (release_blocked, RELEASE_UNSIGNALIZED; skeptic G5: a walker clearing at a red the model alone held
for had the lift roll the car into the junction), so on the car this week none. The left's lift needs the same of its light
where a signal is (a green from an accepted source; skeptic G5 residual: the camera's shadow green certified it and the lift
rolled the car through the model's red), so on the car this week a signalized left gets no lift, only an unsignalized one;
a red or yellow any source reads at the junction, mapped or not, keeps either off. Never for a turn across the oncoming road
(its go is the left's) nor for a far-side turn released in the junction (a committed left lifted into the oncoming lane as
an oncoming car closed: nothing the lift checks sees those lanes; the stall's handback covers that stand). Uncertified (the
param off, a blocker, no green), the go is as before: the model pulls away, or the no-move handback below comes. The left's
go is lifted only from LEFT_LIFT_ROOM_M or more short of the oncoming lanes (its line, not a creep), and is followed into
them only by a launch (left_launch): short of LEFT_LAUNCH_SPEED with LEFT_LAUNCH_ROOM_M left to stop short of them, it is
withdrawn and handed back, never a coast across them on the lift's speed.
Standing engaged with nothing of TurnPilot's holding the car (no shouldStop, no cap biting), no lead from the model within
STOPPED_LEAD_M that the live scene corroborates (one further on doesn't hold a standing car, nor one nothing in the scene
shows: a phantom, a mailbox), gas and brake off, the model stopped on its own: said, with one info chime a stand (carStoppedNote; none
within STOPPED_CONTROL_M of a mapped signal, where the model stands at every red), on the
status line after STOPPED_NOTE_S (STOPPED_NOTE), and handed back loudly (controlPrompt carStoppedTakeControl) after
STOPPED_HANDBACK_S, or NEAR_CONTROL_STAND_S with a map control within STOPPED_CONTROL_M (CRITIQUE REQUIRED 6: a model
stall at a mapped signal TurnPilot doesn't hold for is never silent, and a red the map doesn't know, the car first in
line, isn't handed back before it can turn green). TurnPilot never goes itself; the driver taking over (a gas press
disengages) starts the wait over.
A person TurnPilot saw who may still be in front (WARGAME_FIXES WF-13, BeamNG SAT validation row 6; scene_policy.person_front: a
pedestrian track whose stop brought the car to rest, or a confirmed one, lost within PERSON_FRONT_X_M ahead in or near the path
with the car at 2 m/s or slower): the scene's stop holds the car standing, or brings it to rest on the comfortable curve, and
"Person may be in front" is the status line from that frame. Every go of TurnPilot's is barred on it (go_blocked: 'personFront',
so S2's releases and relifts, the unstick and clearBox lifts and the stop-sign decider's certified go; the decider's own go
through its front zone, the left's through its scene_clear). It ends on the driver (openpilot disengaged, or the gas) or once
she is seen again where she was lost and seen walking clear of the corridor (someone else crossing in front is not her), then
nothing person-like in front 2 s; PERSON_FRONT_MAX_S on it is handed back loudly (personFrontTakeControl, PERSON_FRONT_REASON)
with the car still held. TurnPilotAlwaysDrive changes none of it.
A lane change a map stop leaves no room for, before it or after its go, with the map's lane list placing the car in a lane
the maneuver doesn't leave from (lane_planner.py: unreachable), skips the maneuver as too fast to turn does: said
(turnpilotTooFastToTurn, "Can't Reach Turn Lane" by laneChangeWait unreachable) and rerouted, unless its desire went out.
A handback or a fault cancels a lane change that is offered and not yet started, with laneChangeAbort handedBack or
fault for ABORT_HOLD_S (selfdrived raises turnpilotLaneChangeAborted); TurnPilot turned off cancels it as turnedOff.
A change DesireHelper has started goes on, and one it may have started on the request already isn't announced as
aborted (lane_planner.LanePlanner._abort).
Alert priority (hooks.plan_events): handback, then fault (turnpilotUnavailable), then too fast to turn, then a lane
change, then wrong lane. While TurnPilot isn't acting or is faulted, too fast to turn, wrong lane and lane change offers
are never set here; the rest of the order is selfdrived's (hooks.plan_events, hooks.lane_change_events).
"""
import dataclasses
import datetime
import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from opendbc.car.structs import car
from openpilot.cereal import custom, log, messaging
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL, Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.desire_timing import ENTRY_M, TURN_STARTED_DEG, DesireTiming, keep_last
from openpilot.selfdrive.turnpilot.control_text import STOPPED_NOTE
from openpilot.selfdrive.turnpilot.go_override import GO_ACCEL, GO_CLEAR_M, GO_WINDOW_S, GoCertifier, Takeover
from openpilot.selfdrive.turnpilot.handback import Handback
from openpilot.selfdrive.turnpilot.hooks import ABORT_HOLD_S, CLEAR_DESIRE, CONTROL_STATUS_ALERTS, KEEP_FOLLOW, KEEP_REROUTED, TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.detector.light_state import LIGHT_MODE_DEFAULT, camera_lights_mode
from openpilot.selfdrive.turnpilot.map_controls import (CAMERA_STALE_S, KEEP_TURN, LATCH_PAST_M, LIGHT_STALE_S, MAP_MARGIN_M, PROTECT_PAST_M, YELLOW_DECEL,
                                                         YELLOW_DECEL_MAX, LeftCommand, MapControlPolicy, StopCommand, camera_light_input,
                                                         is_stop, light_input, turn_stop_feasible, unprotected_turn)
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.control_tracker import OFF_ROUTE
from openpilot.selfdrive.turnpilot.stop_go import MODE_DEFAULT, STOP_SWEEP_M, Frame, StopGo, StopGoDecision, mode_for, side_views_live
from openpilot.selfdrive.turnpilot.car_signals import (TurnSignalIntent, TurnSignalOut, driver_signals_for, identity_for,
                                                        read_mode, record_matches)
from openpilot.selfdrive.turnpilot.lane_planner import (SAMPLE_IDX, LanePlanner, Phase, keep_blocker, lane_change_window,
                                                        lane_mismatch, model_lanes_from_msg)
from openpilot.selfdrive.turnpilot.route_follower import (KEEP, LEFT, RIGHT, SAME_TURN_M, TURN, UNCERTAIN_M, Maneuver, RouteFollower,
                                                           same_junction, turn_angle)
from openpilot.selfdrive.turnpilot.route_notice import RouteNotices
from openpilot.selfdrive.turnpilot.scene.lane_paint import lane_paint_from_msg
from openpilot.selfdrive.turnpilot.scene.tracker import (MAX_COAST_S, STALE_S, SWEEP_COAST_S, RawFrame, SceneTracker, default_yaw_rate,
                                                          predicted_path)
from openpilot.selfdrive.turnpilot.scene.tracker_b import TrackerB
from openpilot.selfdrive.turnpilot.visibility import OracleLive
from openpilot.selfdrive.turnpilot.junction_watch import JunctionWatch, detector_view, junction_from_info, sweep_control, sweep_wanted
from openpilot.selfdrive.turnpilot import visibility_table
from openpilot.selfdrive.turnpilot.maps import surveys as survey_file
from openpilot.common.time_helpers import system_time_valid
from openpilot.selfdrive.turnpilot.scene_policy import (CONFIRM_STEADY_HITS, CROSSWALK_REASON, CROSSWALK_X_AFTER_M, CROSSWALK_X_BEFORE_M,
                                                         CROSSWALK_Y_FAR_M, CROSSWALK_Y_NEAR_M, CURB_PED_CAP_DEFAULT, CURB_PED_EDGE_M, CURB_PED_X_M,
                                                         HOLD_LINE_M, NO_ACCEL_CAP, ONCOMING_ENTRY_M, PATH_HALF_WIDTH_M, PERSON_FRONT_BUMPER_M,
                                                         PERSON_FRONT_HALF_M, PERSON_FRONT_NOTE, PERSON_FRONT_X_M, STOP_DECEL, VEHICLES, ScenePolicy,
                                                         far_side_turn, in_path, velocity_trusted)
from openpilot.selfdrive.turnpilot.turn_skip import SkipState, TurnSkip, can_make_turn
from openpilot.selfdrive.turnpilot.turn_speed import LEFT_GO_PROFILE, NO_SPEED_CAP, POST_GO_DEFAULT, POST_GO_PROFILES, PostGo, turn_speed_cap
from openpilot.selfdrive.turnpilot.unprotected_left import (AUTO, DEFAULT_OPPOSING_SPEED, DRIVEN, LIGHT_RED_MAX_S, LINE_D, OFF, ROAD_RANGE_M,
                                                           STAND_UNDECIDED_S,
                                                           ROUTE_REACH_MARGIN_M, LeftInput, UnprotectedLeft, left_mode_for,
                                                           reach_at_line)

FAR_OFF_ALERT_S = ABORT_HOLD_S  # farViewOff is shown this long, once, when the far crop goes off for the drive
FAR_STATUS_STALE_S = 1.         # the full pass's farStatus older than this: the far crop isn't known to read
# farStatus of a far crop that is on (route time: far_live); a stop sweep suspends it (sweep) without turning it off
FAR_ON = ('read', 'noRoom', 'noCalibration', 'sweep')
SIDE_OFF_ALERT_S = ABORT_HOLD_S  # sideCamerasOff is shown this long, once, when the side views go off for the drive
OVERRIDE_FAILED_ALERT_S = 2. * ABORT_HOLD_S  # overrideWriteFailed is shown this long when an override can't be written

Desire = log.Desire
State = custom.TurnpilotPlan.State
Fault = custom.TurnpilotPlan.Fault
LaneMismatch = custom.TurnpilotPlan.LaneMismatch
LaneChange = custom.TurnpilotPlan.LaneChange

APPROACH_M = 150.  # a turn or keep this close (or inside its lane change window) is being approached
NO_LONG_REASON = "openpilot longitudinal is off"
ERROR_REASON = "TurnPilot error"
CAR_UNKNOWN_REASON = "waiting for the car's parameters"
# DisengageOnAccelerator off: a gas press while engaged doesn't disengage, so the driver driving off a handed-back stop sign
# or unprotected left would do it still engaged. TurnPilot refuses to run, out loud (Fault.disengageOnAccelerator)
GAS_DISENGAGE_REASON = "turn on Disengage on accelerator pedal"
ERROR_LOG_PERIOD_S = 10.  # s between logged exceptions from one place, so one that fails every frame doesn't flood the log
# the plan is invalid once carState or modelV2 have failed their checks this many frames in a row (0.5 s at 20 Hz, as
# hooks.PLAN_MAX_AGE_S), not at the first: a plan invalid for one frame drops its desire for that frame, and modeld takes
# the desire coming back as a new pulse
PLAN_INPUT_GRACE_FRAMES = 10
MISMATCH_CONFIRM_FRAMES = 20  # a lane mismatch this many frames in a row (1 s) is flagged: the model's lane count flickers
NO_JUNCTION_DATA_REASON = "no junction data: nominal turn angle"
# hazards hold the car's signal off while it signals a maneuver (car_signals.py; DESIGN phase4/blinker 7.7): the reason line
# says so while it lasts, under the scene's, a missed turn's, the lane planner's and the map's reasons
HAZARDS_REASON = "Hazards on: car not signalling"
JUNCTION_UNCERTAIN_REASON = "junction position uncertain: stopping early"
# no fresh detections for this long (the scene goes unavailable STALE_S after the last) and the detector has stopped:
# past modeld's forced runs (every 0.8 s) and its first retry (1 s), short of its second (2 s)
DETECTOR_DEADLINE_S = 1.5
# no fresh detections this long (in modelV2 frames) after TurnPilot is turned on: the detector never started. modeld
# loads it at startup, or within a second of a mid-drive enable, and runs it from the next frame
DETECTOR_START_DEADLINE_S = 15.
# modeld reads TurnPilotEnabled once a second, as turnpilotd does, and loads the detector only while disengaged: this long
# disengaged with TurnPilot on, it has tried
LOAD_CHANCE_S = 1.5
HANDBACK_REASON = {Handback.detectorStopped: "detector stopped: take control",
                   Handback.noDetector: "detector not running: take control",
                   Handback.modelFellBack: "driving model fell back: take control"}
WAITING_REASON = "waiting for the detector"
STANDSTILL_SPEED = 0.3  # m/s: a hold let go slower than this pulls the car away with a fresh turn pulse
# standing engaged with nothing of TurnPilot's holding the car and no lead, the model stopped on its own (BeamNG route_demo,
# 2026-10-28: 470 s on an empty road): said after STOPPED_NOTE_S, handed back after STOPPED_HANDBACK_S (longer than most
# reds: one the map doesn't know, the car first in line), or NEAR_CONTROL_STAND_S with a map control within
# STOPPED_CONTROL_M (a mapped signal no light source reads: longer than a long red)
STOPPED_NOTE_S = 5.
STOPPED_HANDBACK_S = 60.
NEAR_CONTROL_STAND_S = 90.
STOPPED_CONTROL_M = 30.
STOPPED_LEAD_PROB = 0.5  # modelV2.leadsV3[0].prob over this is a lead (plannerd's own threshold)
STOPPED_LEAD_M = 15.  # ...and it holds the car only this close: openpilot stands long_mpc.STOP_DISTANCE (6 m) behind a
                      # lead it is queued for, and closes up to one further on (route_demo: a ~0.6 lead at 21-28 m, 470 s)
STOPPED_REASON = "Car stopped: take control"
SIGNAL_STOPPED_REASON = "Stopped at the signal: take control"
# TurnPilotAlwaysDrive (ALWAYS_DRIVE.md ST-07 (stops), ST-01 (stalls)): at a mapped signal the model owns the light, so a stand
# there is said as waiting at it (LIGHT_WAIT_NOTE, the status line alone); while the camera's shadow reader reads red there the
# NEAR_CONTROL_STAND_S handback waits, never past LIGHT_RED_MAX_S from the stand's start (a red read that outlasts a cycle is a
# misread or a stuck model); a camera green read LIGHT_GREEN_STEADY_S, none of red or yellow, with the model still standing and
# nothing in the path is said once a green with a chime (lightGreenNotMoving, LIGHT_GREEN_REASON). Never a lift (section 4.4)
LIGHT_WAIT_NOTE = "Waiting at the light"
LIGHT_GREEN_REASON = "Light green: car not moving"
LIGHT_GREEN_STEADY_S = 2.
# a turn released from a standstill (a hold's go: BeamNG 2026-10-28, taken about half the time) is checked, never trusted:
MISSED_TURN_DEG = 15.  # past its junction (the follower's pass) turned less than this, it was missed: said and rerouted
MISSED_HOLD_S = 3.     # turnMissed is held this long, for the alert and for navd (1 Hz) to see it
# the stop-sign ladder's other turn (stop_go.py: 7.1 rung 2, O12) is a maneuver of its own for DesireTiming, never a route's id, so
# navd's reroute after it (turnMissed) never clears its pulse as a replaced maneuver's
ALT_MANEUVER_ID = -7
# ...and for the scene's junction holds (scene_policy: its committed and released ids) each other turn is a maneuver of its own,
# ALT_SCENE_ID less one per encounter: the last one's commitment never opens the next one's junction
ALT_SCENE_ID = -1000
# the sim's cross-traffic oracle (cross_oracle): its road users' track ids, and their classes as the tracker's (COCO) indexes
ORACLE_ID = 1_000_000
CROSS_ORACLE_REFUSED_REASON = "Sim cross traffic refused: not a sim car"  # the light oracle's gate without a sim car (cross_oracle)
SIM_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps', 'sim')
SIM_VISIBILITY = Path(SIM_DIR) / 'visibility_sim.json'  # the sim's files (set_sim), read only under its gate
SIM_SURVEYS = Path(SIM_DIR) / 'surveyed_junctions_sim.json'
SIM_SUN_UTC_H = 17     # the sim's sun: 13:00 EDT at the sim tiles' Indianapolis frame
ORACLE_CLASS = {'pedestrian': 0, 'cyclist': 1, 'car': 2, 'motorcycle': 3, 'bus': 5, 'truck': 7}
ALT_LANES_M = 30.      # navd's lane guidance is our approach's when its maneuver is this close to the held stop's junction
TURN_MISSED_REASON = "Turn missed — rerouting"
STALL_PROMPT_S = 3.    # standing this long after the go, STALL_MIN_DEG to STALL_MAX_DEG into the turn, nothing holding it:
STALL_MIN_DEG = 20.    # handed back (the stop line it left is within STOPPED_CONTROL_M, so the empty-road handback
STALL_MAX_DEG = 80.    # comes only after the long stand; the sweep's stalls stood 20 s at 46-78 deg)
STALLED_REASON = "Stalled in the turn: take control"
# TurnPilotAlwaysDrive (ALWAYS_DRIVE.md LEFT-21, ST-03 (stalls)): before that handback, a stand in a curb-side route turn
# (never a far-side one: ST-10) gets one fresh turn pulse at STALL_REPULSE_S (DesireTiming.stall_pulse) and, from
# STALL_PROMPT_S, a clearBox lift with the car standing (relift: its speed end isn't followed), up to STALL_LIFTS lifts
# STALL_LIFT_GAP_S apart, all inside the release's RELEASE_LIFT_M, each said (stallLift, STALL_LIFT_REASON); none while
# go_blocked reads anything (a model lead within GO_CLEAR_M among it: the g62 phantom, the s20 mailbox), a red or yellow is
# read there, or a person is near the turn (person_near_turn: path_blocker's straight corridor and the model's lead don't see
# the people a model stops for partway round), and one out is withdrawn when any of that comes. Then the handback, no hold
STALL_REPULSE_S = 2.
STALL_LIFTS = 3
STALL_LIFT_GAP_S = 3.
STALL_LIFT_REASON = "Stalled: finishing the turn"
PEOPLE = ('pedestrian', 'cyclist')
STALL_PERSON_PATH_M = PATH_HALF_WIDTH_M + CURB_PED_EDGE_M  # m of the model's predicted path a person bars the lift within
PERSON_RUN_HOLD_S = 1.  # a detector run's person boxes stand for that camera's view this long (the last run of each)
# a model lead (close_lead) holds a standing car only as far as the live scene corroborates it (BUILD_FINDINGS D1, D2: a phantom
# 10-17 m dead ahead on empty road the moment a turn's pulse went out, a mailbox on a corner read at p 0.99, each kept a car
# standing silently): a vehicle track in the path within LEAD_CORROBORATE_M of the lead's x, or a vehicle box of a detector run
# at it (ranged within LEAD_CORROBORATE_M of its x across the lead's place, or unranged with its bearing within
# LEAD_BEARING_TOL_DEG of the lead's: a lead's bumper under the hood ranges nothing), held LEAD_SEEN_HOLD_S between runs (10 Hz,
# the wide camera's turns included). A lead nothing corroborates exempts no stand; a corroborated one keeps a stand in a turn
# LEAD_STAND_MAX_S at most (stalled_in_turn), a go's stand NO_MOVE_MAX_S from the go (no_move), and a stand off any turn as long
# as it is there (stopped_unheld: a queue). Never a go through it: the go's own lead check (go_blocked) is close_lead's
# a route turn the model abandons in the junction (BUILD_FINDINGS N1, BeamNG S20: steering unwound from -97 to +28 deg with
# 24 deg of a 90 deg right turned, turnRight still out, onto the corner's sidewalk; the smoke run hit a fixed object): past
# the hold line, under TURN_ABANDON_FRACTION of its angle turned, moving, modelV2.action.desiredCurvature (positive right)
# away from the turn's side by more than TURN_ABANDON_CURV for TURN_ABANDON_S is handed back (turnAbandonedTakeControl,
# TURN_ABANDONED_REASON) until the driver takes over: no steering, no new hold, the turn's caps stay. S20's unwinding read
# -0.012 to -0.019 for 0.5-0.75 s; a 200 m radius is the dead band, past the exit's own straightening noise
TURN_ABANDON_FRACTION = 0.5
TURN_ABANDON_CURV = 0.005
TURN_ABANDON_S = 0.5
TURN_ABANDONED_REASON = "Turn abandoned: take control"
# R8b (ALWAYS_DRIVE.md S3): a route turn driven straight through. From the hold line to the follower's pass, turned under
# MISSED_TURN_DEG toward the turn and under TURN_MISSED_AWAY_DEG away from it (turned_deg is signed toward the turn: a car
# yawed away from it in the junction is heading for the oncoming lanes or the far kerb, a mess-up, never a straight drive;
# the plan's test has 10 deg away break it), and desiredCurvature under TURN_MISSED_CURV either way on every frame (inside
# S20's -0.012 to -0.019 unwind above, so a curvature read once proves nothing): missed at the pass (turnMissed) and
# rerouted, its last pulse cleared from the model's queue on that frame (WARGAME_FIXES WF-11). Under TurnPilotAlwaysDrive
# R8's fire inside both bounds is that too: its desire cleared at the fire (CLEAR_DESIRE), no handback, the bounds then held
# on every frame to the pass, and either broken on the way is R8's handback, latched as R8's own (turn_abandoned)
TURN_MISSED_CURV = 0.015
TURN_MISSED_AWAY_DEG = 5.
# the car left of the centre line (BUILD_FINDINGS D5-lane: BeamNG S18, S19, the model kept to the oncoming lane for 82-216 m
# after a wide turn and on a straight street, and nothing in TurnPilot noticed). Under US markings (MUTCD) a yellow line on
# the car's right means it is left of the centre line (or of a divided road's left edge): right-hand traffic, engaged,
# moving WRONG_SIDE_MIN_SPEED or more, the first painted line to the car's right (the ego line; the next one out where the
# ego line reads unpainted) read yellow by the lane paint reader at WRONG_SIDE_CONF or more for WRONG_SIDE_S of reads
# (frames it doesn't read, the wide camera's, neither count nor reset; a read of anything else resets) is handed back
# (wrongSideTakeControl, WRONG_SIDE_REASON) until the driver takes over: no steering, no hold. Not a two-way left-turn lane
# (the car's own lines both yellow), nor a yellow-painted kerb (the right road edge, seen: its std under
# WRONG_SIDE_EDGE_STD_MAX, at least WRONG_SIDE_BEYOND_M beyond the yellow line, the oncoming lane: none past a kerb;
# BeamNG's streets read std 1.3-1.4 there, past lane_planner's EDGE_STD_MAX, with the edge 3.7 m beyond). The finding's
# geometry fallback (the left road edge close to the left line with a lane to the right, paint unknown) is no rule here: on
# openpilot's demo route it read so for 62 frames of the highway's left lane (segment 13), the car on its own side
WRONG_SIDE_S = 1.
WRONG_SIDE_MIN_SPEED = 2.
WRONG_SIDE_CONF = 0.5
WRONG_SIDE_BEYOND_M = 1.
WRONG_SIDE_EDGE_STD_MAX = 2.  # an edge the model sees at all (the demo route's right edge 0.7-0.8; none there reads 20-60)
WRONG_SIDE_GAP_S = 1.  # no read this long and the count starts over
WRONG_SIDE_REASON = "Wrong side of the road: take control"
# an off-route turn is a mess-up said loudly (SAT_VALIDATION row 1, route_demo: the route went straight on, TurnPilot sent no
# desire, and the model turned right on its own, 6.6 m into the oncoming lane with every paint read unknown, into a roadside
# object; the follower dead-reckons, so the turn read as on route). Engaged, moving, with a route: the car's heading changing
# OFF_ROUTE_TURN_DEG or more within OFF_ROUTE_TURN_M of travel, none of that travel within OFF_ROUTE_NEAR_M of a route maneuver
# (ahead or just passed) or with a turn desire of TurnPilot's out, is handed back (offRouteTakeControl, OFF_ROUTE_TURN_REASON)
# until the driver takes over: no braking, no steering. A bend the route itself makes that way in the step (the last
# maneuver's bearing out to the next one's bearing in, OFF_ROUTE_BEND_MARGIN_DEG short of the threshold) explains it: no
# handback. Checked offline on the logs on this Mac: 286 BeamNG logs with their routes (the gate's and the validation's turns,
# 17 km) fire on none but row 1; the real Tesla and comma four routes (49 km engaged, segments joined, 36 junction turns taken
# as their routes' maneuvers) fired once at 35 deg (a turn's exit 46 m past it), none at 45. On row 1 it fires 16 m after the
# pull-away, about 1.5 s before the car was on the wrong side and 3 s before the impact
OFF_ROUTE_TURN_DEG = 45.
OFF_ROUTE_TURN_M = 30.
OFF_ROUTE_NEAR_M = 40.
OFF_ROUTE_BEND_MARGIN_DEG = 10.
# the route's own shape (navInstruction.routeTurns, navd's route_turns): a heading change the road makes with no maneuver (an
# S-bend, a street turning 90 deg with no junction) is no off-route turn when the route turns that way, within the margin,
# over some OFF_ROUTE_SHAPE_WINDOW_M of it from OFF_ROUTE_TURN_M + OFF_ROUTE_SHAPE_BEHIND_M behind the car's place on it to
# OFF_ROUTE_SHAPE_AHEAD_M ahead (the place dead-reckoned from navd's 1 Hz fix, which lags the car: its latency, and the
# shape is of where the fix was). The place is as often ahead of the car: GPS's along-track error at suburban speeds is more
# than the lag's few metres, and along_path (the nearest segment's start plus the straight line to the fix) puts it ahead
# by up to the lateral error at a bend; BEHIND_M 15 keeps a no-maneuver bend excused with the place up to 10 m ahead (at
# 5, 29 of 48 synthetic bends fired with it 5 m ahead), and row 1 fires at the same metre from 10 m ahead to 15 m behind.
# No shape (a navd before it, an invalid message): the step's bend alone, as before
OFF_ROUTE_SHAPE_BEHIND_M = 15.
OFF_ROUTE_SHAPE_AHEAD_M = 20.
OFF_ROUTE_SHAPE_WINDOW_M = OFF_ROUTE_TURN_M + 10.
OFF_ROUTE_TURN_REASON = "Off route: take control"
LEAD_CORROBORATE_M = 3.
LEAD_BEARING_TOL_DEG = 3.
LEAD_SEEN_HOLD_S = 1.
LEAD_STAND_MAX_S = 20.
# TurnPilotAlwaysDrive (ALWAYS_DRIVE.md ST-09 (stalls)): a queue is the model's, so in a turn such a lead keeps the stand with no
# bound while a vehicle track at it, its velocity trusted, has moved (QUEUE_MOVING_SPEED or faster) within QUEUE_STILL_MAX_S; one
# that stood still that long (something parked in the turn's path), or nothing moving at it, hands back at QUEUE_STILL_MAX_S
QUEUE_STILL_MAX_S = 60.
QUEUE_MOVING_SPEED = 1.  # m/s: unprotected_left.STAND_SPEED, slower than which a velocity-trusted track stands
# not at STANDSTILL_SPEED this long after TurnPilot's go, nothing else holding the car: the model is holding it (DESIGN phase 4
# section 2-3; G0, 2026-10-28: 14 of 24 tapless goes stood 20 s). Withdrawn and handed back; an unprotected turn's sooner
NO_MOVE_S = 2.
NO_MOVE_MAX_S = 20.  # TurnPilotNoMoveS is clamped to NO_MOVE_S..this (the sweep's V1 tapless column measures to 20 s)
LEFT_NO_MOVE_S = 1.
NO_MOVE_REASON = "Can't start the turn: take control"
# TurnPilotAlwaysDrive (ALWAYS_DRIVE.md ST-04, ST-11, ST-13): a go from the line that isn't the unprotected left's (it keeps
# its own hold, LEFT-06) nor the stop-sign decider's (its own watchdog) is a release, from TurnPilot's go to the model's
# takeover predicate with the car moving, or the driver. The model not following it is never held: the turn's desire stays,
# a lift is certified again (a relift: GoCertifier's relift) with the car standing (under STANDSTILL_SPEED) RELIFT_GAP_S after
# the release's last go, up to RELEASE_GOES goes, all of its lifts moving the car RELEASE_LIFT_M in all by odometer (a go out
# withdrawn, 'budget', once it is spent: GoLift's 2.25 m and 1.5 m/s hold per release, not per lift); after NO_MOVE_MAX_S
# standing with nothing holding the car, the loud cantStartTakeControl, still no hold, cleared once the car moves on the
# model's takeover predicate (a WARNING the model drives on under)
RELEASE_LIFT_M = 0.5 * GO_ACCEL * GO_WINDOW_S ** 2
RELEASE_GOES = 3
RELIFT_GAP_S = 2.
RELIFT_MIN_M = 0.5 * GO_ACCEL * 1. ** 2  # m: a relift needs this much of RELEASE_LIFT_M left, a lift's first second (the odometer
                                         # reads a full window from rest a few cm short of the 2.25 m it moved the car)
# ...but only a go from the hold line: the car standing this far or further before the maneuver point (the scene's hold line,
# HOLD_LINE_M; an unprotected turn's map stop, MAP_MARGIN_M short of it, latches up to LATCH_PAST_M past its point), the
# scene not committed to the junction nor the driver having taken over into it. Nearer, the car is in the junction: never
# held there, the stall's handback covers it (skeptic G3: a walker's hold released 40-50 deg in froze the car there)
GO_LINE_M = HOLD_LINE_M + MAP_MARGIN_M - LATCH_PAST_M
LEFT_RANGE_MAX_M = 400.  # TurnPilotLeftRangeM is clamped to 0..this
LEFT_HOLDS_PULSE = ('approach', 'waitGap', 'creep', 'undecided', 'announce')  # the left's states with no turn pulse yet
# the junction rules' road frame (scene_policy.road_frame): the car's heading change toward its turn, counted only once the
# turn's desire has gone out (before it the model follows its lane: its yaw is the road's bend, not the car turning; skeptic
# G4: a 6 deg bend away from the turn over the last 40 m turned the band off the oncoming lane), and only moving (a standing
# car doesn't yaw; a biased yaw rate over minutes at the line would)
YAW_MIN_SPEED = STANDSTILL_SPEED
# a change the route wants that may still be offered: the turn's prompt waits (STOP_SIGNS D15), lest the driver's lamp be
# read as the change's go-ahead. One past its window, unreachable or unplaceable will never be offered: it doesn't hold it
LANE_PENDING_WAITS = ('beforeWindow', 'confirming', 'cooldown', 'blindspot', 'carInLane', 'afterStop')
# TurnPilotGoOverride (main()): TurnPilot's go from a standstill certified for plannerd's bounded lift (go_override.py; DESIGN
# phase 4 decision 1, G5). On by default: the autonomous left and the stop sign's go depend on it (Kellen, 2026-10-28)
GO_OVERRIDE_DEFAULT = True
# TurnPilotAlwaysDrive (main(); ALWAYS_DRIVE.md section 1): TurnPilot drives on where it is only unsure (a NERVOUS handback,
# skip, wait or slow-down) and hands back where it messed up. Off by default: with it off every behaviour is today's. Handed
# every frame to the lane planner, the unprotected left, the map controls, the turn skip and the stop-sign decider's Frame
# (hand_always_drive), never in shadow mode; steps S2 to S5 act on it there
ALWAYS_DRIVE_DEFAULT = False
LEFT_GO_STATES = ('go', 'committed')  # the unprotected left's states its certified go stays out in
# the unprotected left's go is followed into the oncoming lanes only by a launch: its clear time (unprotected_left.t_req) is
# for its 1.5 m/s^2 launch, and a model that 'takes over' at the speed the lift gave it (0.8 m/s) would coast 10 s across
# them. Short of LEFT_LAUNCH_SPEED while the car can still stop LEFT_LAUNCH_ROOM_M short of them (at STOP_DECEL), the go is
# withdrawn and handed back. A car on the left's own launch reaches that point at 2.3 m/s; one launching at 1.0 m/s^2 at 2.05.
# Checked (and the lift certified for the left) only for a go with LEFT_LIFT_ROOM_M or more to the oncoming lanes: the lift's
# whole nudge (2.25 m, to 1.5 m/s), its stop at STOP_DECEL and LEFT_LAUNCH_ROOM_M, so the lift alone never carries the car
# past where it can stop short of them. A go nearer (crept for a view: ONCOMING_ENTRY_M + 1 m) gets no lift, and is G4's go
LEFT_LAUNCH_SPEED = 2.
LEFT_LAUNCH_ROOM_M = 1.5
LEFT_LIFT_ROOM_M = 0.5 * GO_ACCEL * GO_WINDOW_S ** 2 + (GO_ACCEL * GO_WINDOW_S) ** 2 / (2. * STOP_DECEL) + LEFT_LAUNCH_ROOM_M
# turnRelease (a turn TurnPilot's own hold let go of at a standstill) certifies only that TurnPilot's blocker left the path,
# never why the model still holds the car: a red no source TurnPilot accepts reads (the car's, this week), cross traffic, an
# oncoming car, which plannerd can't tell from G0's spurious stand (skeptic G5). So it rests on its junction's own
# certification: the junction's signal read green by an accepted source (map_controls.release_light: the camera's green, in
# shadow, never certifies; its red or yellow keeps the go off), checked at the go and on every frame of it. On the car this
# week no source is accepted, so no turnRelease: the model pulls away or the no-move handback comes, as before G5. True also
# certifies it at a junction navd maps with no signal (controls ok); an unmapped signal or stop sign, and cross traffic, then
# go unchecked
RELEASE_UNSIGNALIZED = False
# ...under TurnPilotAlwaysDrive (ALWAYS_DRIVE.md LEFT-20) it is True for a curb-side or straight release where release_light reads
# 'none' (navd's controls ok: the sidecar served) and section 4.5's conditions hold, since 'ok' means the sidecar came, not that
# OSM has every signal and stop sign (unsignalized_why): no map control within STOPPED_CONTROL_M, the camera's shadow reader
# live and reading no head of any colour, no detector stop-sign box in a forward run in the last SIGN_CLEAR_M driven (and the
# camera's signs read at all). A red or yellow from any source still blocks, a mapped signal still needs an accepted green
# The stop-sign memory is in metres driven, never seconds standing (watch_signs, every frame, TurnPilot planning or not): an
# unmapped stop sign at the line stands beside the car, outside the road camera's view (half of it is 20 deg on the AR/OX
# cameras, 30 deg on the OS; a sign 3-8 m to the side leaves it 5-22 m before the car is level with it), so its last box comes
# on the approach and none while the car stands there. Any forward run counts, the road camera's full pass or the wide
# camera's (both cameras' full passes: tracker.view_key 'road', 'full'; map_controls' C11 evidence reads the same). SIGN_CLEAR_M
# covers those 22 m and the release's RELEASE_LIFT_M, with room for a queue or missed runs hiding the sign nearer the line.
# Metres driven at STANDSTILL_SPEED or more with no forward run for SIGN_RUN_FRESH_S (the detector forces one at least every
# 0.8 s, FORCE_RUN_S) count as a sign seen there: road the runs didn't see never clears it; standing with none changes
# nothing. turnpilotd's start counts as a sign just seen
SIGN_CLEAR_M = 50.
SIGN_RUN_FRESH_S = 1.
# TurnPilotAlwaysDrive, section 4.5 (the owner's yes): a model stop off any hold (stopped_unheld's stand) gets one lift of its own
# cause (unstick) from UNSTICK_S, only where release_light reads 'none' (controls ok), unsignalized_why reads nothing, nothing
# is in the path and no model lead within GO_CLEAR_M (go_blocked), and no far-side turn or the left is near; said with a chime
# (unstickLift), inside the release's RELEASE_LIFT_M, no second try until the model drives on (unstick_done)
UNSTICK_S = 10.
# the right-turn crosswalk hold's standoff (ALWAYS_DRIVE.md SC-02, unconditional): held standing by it PED_NOTE_S, one chime and
# PED_WAITING_NOTE; once every walker holding it has stood still (scene_policy.walker_standing) STAND_UNDECIDED_S, the left's
# same standoff, pedestrianWaitingTakeControl with the car held, never a go past them; walkers who move are waited through
PED_NOTE_S = 10.
PED_WAITING_NOTE = "Waiting for pedestrian"
PED_WAITING_REASON = "Pedestrian waiting: take control"
UNSTICK_REASON = "Nothing ahead is holding the car: going on"
# a person TurnPilot saw who may still be in front (WARGAME_FIXES WF-13; BeamNG SAT validation row 6; scene_policy.person_front):
# its hold and its status line are the scene's, every go of TurnPilot's is barred on it (go_blocked: 'personFront'; the left's
# scene_clear, the decider's front zone), and nobody seen leaving it this long it hands back loudly with the car still held
PERSON_FRONT_MAX_S = 30.
PERSON_FRONT_REASON = "Person may be in front: take control"
# the stop-sign decider's go causes (stop_go.py, go_override.CAUSES): its own watchdog (the gate, G5's window, GO_RETRIES, H5)
# follows them, never the no-move clock, and turnRelease never certifies alongside one
STOP_GO_CAUSES = ('stopSign', 'rollUp', 'creep', 'clearBox')
OWN_CAP_TOL = 0.05  # m/s a creep go's own cap may be undercut (its ramp near its end) before something else's counts as a hold
YAW20_M = 20.  # m of travel the approach yaw (go condition 3) is integrated over
# a stop_go.update that raises with the other turn under way (going, committed, inLane) carries its last decision on for up to
# this many frames, logged (skeptic SG4c A: a bare decision for one frame dropped the turn's desire for good); past that, a
# failing decider is as before: shadow, its go withdrawn
STOP_GO_CARRY_FRAMES = 20
STOP_GO_CARRY_STATES = ('going', 'committed', 'inLane')
# a veto at the unprotected left's line is the car's at that line: its record (VetoedTurn.line) is never forgotten before the
# car has driven this far from where the veto found it (TurnPilot.carry_veto), whatever the follower's nearest reading says.
# More than a standing car's odometer creeps, and under the 4.3 m on which, the driver driving the left, navd moves on to the
# next step at the earliest (NAVD_ADVANCE_M past a nearest reading UNCERTAIN_M short of the 9.3 m the car stands from the
# junction, the most spread without junctionUncertain): the record is gone before the next left is taken up
LINE_VETO_MOVED_M = 2.
# ...and it is for the left at that line only: carried to navd's new id for a left whose junction its readings put within this
# of the recorded one (UNCERTAIN_M of spread without junctionUncertain, and a margin for the reroute's own reading: K2's
# reroute at 0 m, N1's nearer reading at 10 m), never to the next left on along the road (skeptic m-behaviour3 P1: a left 25
# m past the vetoed one, same bearings, was taken for it within SAME_TURN_M, its request ended with nothing said)
LINE_VETO_SAME_M = UNCERTAIN_M + 5.
STOP_GO_HOLDS_PULSE = ('approach', 'stopped', 'rollUp', 'observe')  # the decider's states a turn's pulse waits for its go in


def path_blocker(objects, raw, limit_m: float = GO_CLEAR_M) -> str:
  """A road user in the car's path within limit_m, '' if none: a track by scene_policy.in_path, or a box of this frame's
  detector run (scene.tracker.RawFrame, tracked or not, at any confidence the decoder keeps) whose bottom spans the corridor
  ahead (PATH_HALF_WIDTH_M either side). What blocks TurnPilot's certified go (go_override.py), and withdraws one out."""
  for obj in objects:
    if 0. < obj.x <= limit_m and in_path(obj):
      return f"track {obj.trackId} {obj.cls} at {obj.x:.1f} m"
  for b in (raw.boxes if raw is not None else ()):
    if b.xy is None or not 0. < b.xy[0] <= limit_m:
      continue
    rng = math.hypot(*b.xy)
    lats = sorted(rng * math.sin(math.radians(e)) for e in b.span_deg)
    if lats[0] < PATH_HALF_WIDTH_M and lats[-1] > -PATH_HALF_WIDTH_M:
      return f"{b.camera} box {b.cls} at {b.xy[0]:.1f} m"
  return ''


def close_lead(model) -> bool:
  """modelV2 has a lead within STOPPED_LEAD_M: what holds a standing car (one further on doesn't)."""
  return len(model.leadsV3) > 0 and model.leadsV3[0].prob > STOPPED_LEAD_PROB and len(model.leadsV3[0].x) > 0 and \
    model.leadsV3[0].x[0] < STOPPED_LEAD_M


def straight_inside(turned_deg: float, curvature: float) -> bool:
  """R8b's bounds (module constants TURN_MISSED_*): turned under MISSED_TURN_DEG toward the turn and under TURN_MISSED_AWAY_DEG
  away from it, desiredCurvature under TURN_MISSED_CURV either way."""
  return -TURN_MISSED_AWAY_DEG < turned_deg < MISSED_TURN_DEG and abs(curvature) < TURN_MISSED_CURV


def yellow_on_the_right(paint, line_ys, line_probs, edge_ys, edge_stds) -> tuple[bool, bool]:
  """(the first painted line to the car's right reads yellow: the car left of the centre line, the line's paint read at all)
  from the lane paint of modelV2's four lines and the lines' and road edges' y (positive right), probs and stds at one
  distance ahead (WRONG_SIDE_*). The ego right line, or the next one out where the ego line reads unpainted (kind none).
  Not with the car's left line yellow too (a two-way left-turn lane), nor without a road edge seen WRONG_SIDE_BEYOND_M
  beyond it (a yellow kerb has no road past it). Unknown paint, or fewer than four lines, is no read."""
  if len(paint) != 4 or len(line_ys) != 4 or len(edge_ys) != 2:
    return False, False
  i = 3 if paint[2].kind == 'none' else 2
  p = paint[i]
  if p.color == 'unknown':
    return False, False
  yellow = p.color == 'yellow' and p.confidence >= WRONG_SIDE_CONF and not (i == 2 and paint[1].color == 'yellow') and \
    edge_stds[1] < WRONG_SIDE_EDGE_STD_MAX and edge_ys[1] - line_ys[i] >= WRONG_SIDE_BEYOND_M
  return yellow, True


def model_lines_at(model):
  """modelV2's (line ys, line probs, edge ys, edge stds) at SAMPLE_IDX (lane_planner's), empty when it hasn't them all."""
  lines, edges = model.laneLines, model.roadEdges
  if len(lines) < 4 or len(edges) < 2 or len(model.laneLineProbs) < 4 or len(model.roadEdgeStds) < 2 or \
     any(len(e.y) <= SAMPLE_IDX for e in list(lines) + list(edges)):
    return (), (), (), ()
  return ([float(ln.y[SAMPLE_IDX]) for ln in lines], [float(p) for p in model.laneLineProbs], [float(e.y[SAMPLE_IDX]) for e in edges],
          [float(s) for s in model.roadEdgeStds])


def map_handback(mapd) -> bool:
  """A map control's own take-control holds or explains the car this frame (its latch's handback), so the stand handbacks
  wait. A line crossed with no go (map_controls.Driven: no hold) isn't one: a stall or a stand in it is said for what it is."""
  return mapd.prompt in TAKE_CONTROL_PROMPTS and not mapd.driven


def lead_corroboration(model, objects, raw) -> str:
  """What in the live scene shows modelV2's lead, '' if nothing (LEAD_CORROBORATE_M): a vehicle track in the path near its x
  (scene_policy.in_path), or a vehicle box of this frame's detector run (scene.tracker.RawFrame) at it, ranged near its x
  across its place, or unranged, its bearings spanning the lead's. A pedestrian, or a vehicle off the path or past it, isn't."""
  lead = model.leadsV3[0]
  x = float(lead.x[0])
  y = -float(lead.y[0]) if len(lead.y) else 0.  # modelV2's y is right, the scene's left
  for obj in objects:
    if str(obj.cls) in VEHICLES and abs(obj.x - x) <= LEAD_CORROBORATE_M and in_path(obj):
      return f"track {obj.trackId} {obj.cls} at {obj.x:.1f} m"
  bearing = math.degrees(math.atan2(y, max(x, 0.1)))
  for b in (raw.boxes if raw is not None else ()):
    if str(b.cls) not in VEHICLES:
      continue
    if b.xy is None:
      if min(b.span_deg) - LEAD_BEARING_TOL_DEG <= bearing <= max(b.span_deg) + LEAD_BEARING_TOL_DEG:
        return f"{b.camera} box {b.cls} unranged"
      continue
    if abs(b.xy[0] - x) > LEAD_CORROBORATE_M:
      continue
    rng = math.hypot(*b.xy)
    lats = sorted(rng * math.sin(math.radians(e)) for e in b.span_deg)
    if lats[0] < y + PATH_HALF_WIDTH_M and lats[-1] > y - PATH_HALF_WIDTH_M:
      return f"{b.camera} box {b.cls} at {b.xy[0]:.1f} m"
  return ''


class ErrorLog:
  """cloudlog.exception for the exception being handled, at most once per ERROR_LOG_PERIOD_S for each event."""
  def __init__(self):
    self.last: dict[str, float] = {}

  def __call__(self, event: str):
    now = time.monotonic()
    if now - self.last.get(event, -ERROR_LOG_PERIOD_S) >= ERROR_LOG_PERIOD_S:
      self.last[event] = now
      cloudlog.exception(event)


log_error = ErrorLog()


def alt_maneuver(alt: dict) -> Maneuver | None:
  """The stop-sign ladder's other turn as a route maneuver for DesireTiming (stop_go: 7.1 rung 2, 3.5): a turn to its side,
  its angle from its approach's relBearing; None going straight (no desire)."""
  if alt['maneuver'] == 'straight':
    return None
  side = RIGHT if alt['maneuver'] == 'right' else LEFT
  return Maneuver(TURN, side, 'turn', alt['maneuver'], float(alt['angle']), geometry=True, bearing_side=side)


@dataclass
class StraightDrive:
  """R8b's watch over a committed route turn from its hold line (TurnPilot.watch_straight): its maneuver id and side, deg
  turned toward it (DesireTiming's while its desire is out, the car's yaw after R8b's fire cleared it), whether R8 fired
  inside the bounds under TurnPilotAlwaysDrive (fired), and whether both bounds have held on every frame since (ok)."""
  id: int
  side: int
  deg: float = 0.
  fired: bool = False
  ok: bool = True


@dataclass
class VetoedTurn:
  """A turn or keep the driver overruled (a signal veto's skip): its maneuver (type, modifier, bearings), where its
  junction is on turnpilotd's odometer (odo_m plus the follower's distance at the veto), the veto's reason, the id it is
  skipped under now, and whether its desire was out (its cap kept to its pass, skip_cap_id). line: a veto at the
  unprotected left's hold line or in its go while withdrawable (the intent's left_veto), the left's handback and the end
  of the request for the left, never a skip; carried as that end. veto_odo: the car's own place on the odometer at the
  veto. TurnPilot.carry_veto."""
  maneuver: object
  junction_odo: float
  reason: str
  maneuver_id: int
  capped: bool
  line: bool = False
  veto_odo: float = field(kw_only=True)


class TurnPilot:
  def __init__(self):
    self.big_seen = False  # modelV2 came from the big model this drive (kept when TurnPilot is turned off and on)
    self.fell_back = False  # ...and then from the small one
    self.cancel_route = False  # set once on a fallback: main() removes NavDestination, once no turn is committed
    self.stop_abort, self._stop_abort_t = 'none', 0.  # a lane change cancelled by a disabled plan: reset() can't hold it
    self.handback_now: int | None = None  # this frame's handback, once handback() has run for it
    self.hold_v: float | None = None  # handed back with openpilot steering: the speed the car is held to (module docstring)
    self.scene_now = None  # this frame's scene decision, once update() has made it
    self.limits = (NO_ACCEL_CAP, NO_SPEED_CAP, False)  # the last plan's (accelCap, speedCap, shouldStop)
    self.oracle_ok = False  # the sim light oracle is accepted: SIMULATION=1 and TurnPilotSimLightOracle (main())
    self.yellow_decel = YELLOW_DECEL  # TurnPilotYellowDecel (main())
    self.camera_mode = LIGHT_MODE_DEFAULT  # TurnPilotCameraLights (main()): the camera's lights, shadow unless addOnly
    self.map_now = None  # this frame's map control decision, once update() has made it
    self.post_go_profile = POST_GO_DEFAULT  # TurnPilotPostGoProfile (main()): a standstill turn's post-go profile
    self.left_go_profile = LEFT_GO_PROFILE  # TurnPilotLeftGoProfile (main()): an unprotected turn's
    self.no_move_s = NO_MOVE_S  # TurnPilotNoMoveS (main()): a standstill turn's no-move limit (an unprotected turn's is fixed)
    self.auto_left = AUTO  # TurnPilotAutoLeft (main()): the unprotected left's mode (unprotected_left.py)
    self.left_range_m = ROAD_RANGE_M  # TurnPilotLeftRangeM (main()): the detector's proven range its R_req is checked against
    self.left_far_range_m = 0.  # TurnPilotLeftFarRangeM (main()): the far crop's proven range (unmeasured: 0, none)
    self.far_status = 'off'     # the full pass's latest farStatus (watch_far)
    self.far_status_age = float('inf')
    self.far_off_t = 0.         # s farViewOff has left to show
    self.side_status = 'unset'  # the detector's latest sideViewStatus (watch_far)
    self.side_off_t = 0.        # s sideCamerasOff has left to show
    self.far_lights: tuple = ()  # the far crop's governing heads last logged (watch_far)
    self.left_now = None  # this frame's unprotected left decision (unprotected_left.LeftDecision), once update() has made it
    # the turn signal (car_signals.py): who signals, the car (a verified Tesla) or the driver (prompted), and which side
    self.turn_signal = TurnSignalIntent()
    self.signal_out = TurnSignalOut()  # the last frame's, for the lane planner and the unprotected left
    self.car_signal_flags = 0  # CarParams.flags on a Tesla (main()): CAR_SIGNALS, CAR_SIGNALS_CHECK
    self.car_signal_mode = 'off'  # TurnPilotCarSignals (main())
    self.unverified_why = ''  # why mode on found no verified record for this car (main())
    self._rf_ticked = self._yaw_ticked = False  # update() dead-reckoned the route, and counted the yaw, this frame
    self.go_override = GO_OVERRIDE_DEFAULT  # TurnPilotGoOverride (main())
    self.curb_ped_cap = CURB_PED_CAP_DEFAULT  # TurnPilotCurbPedCap (main()): the scene's curb-pedestrian speed cap (scene_policy.py)
    self.go_cert = GoCertifier()  # the go certified for plannerd's lift (go_override.py): its ids never repeat while this runs
    self.go_own_cap: float | None = None  # m/s: a creep go's own creep cap, which isn't a hold that blocks it (go_blocked)
    self.sweeping = False  # the last plan asked modeld for a stop sweep (stopSweep, stopSweepGo): the tracker coasts longer
    self.sweep_on = False  # the held cycle is wanted (junction_watch.sweep_wanted: walking pace, with hysteresis)
    self.watch = JunctionWatch()  # step S3's evidence at the swept stop or yield (junction_watch.py): logged, never acted on
    self.tracks_now: list = []    # the tracker's tracks this frame (observe_runs), for the watch
    self.runs_now: list = []      # ...and its runs (the stop-sign decider's signal-head check)
    self.camera_signs = True      # TurnPilotCameraStopSigns isn't off (main()): modeld reads class-11 boxes (map_controls, C11)
    self.lenses: dict = {}        # consumer: the wide lens it reports (the tracker's lens_spec, the side views' message lens)
    # the stop-sign decider (stop_go.py, STOP_SIGNS.md v3 SG4a): TurnPilotStopSignGo (main()), and its inputs' sources
    self.stop_sign_mode, self.stop_sign_known = MODE_DEFAULT, True
    self.vis_source = visibility_table.cached
    self.survey_source = survey_file.cached
    self.sim_files = False        # the sim's own visibility and survey files and sun (set_sim: SIMULATION=1, the param, a sim car)
    self.cross_refused = False    # this frame the sim's cross-traffic oracle sent cross traffic and wasn't a sim car's (cross_oracle)
    self.wall_clock = lambda: datetime.datetime.now(datetime.UTC).timestamp() if system_time_valid() else None
    self.cp_delay = 0.15          # CarParams.longitudinalActuatorDelay (main()); T_ABORT takes the larger of it and the file's
    self.odo_m = 0.               # m driven since turnpilotd started (faults included): the decider's position after its latch
    self.yaw20: list = []         # (odometer, deg) of the car's heading change, the last YAW20_M
    self.sg_now: StopGoDecision | None = None  # this frame's decision, once update() has made it
    self.sg_last: StopGoDecision | None = None  # the last frame's (stop_go_update), for a failed frame to carry on
    self.sg_carried = 0  # failed frames in a row carried on from it
    self.sg_turned = 0.           # deg the car has turned since the decider's go
    self.sg_heading = 0.          # deg the car has turned since turnpilotd started, never reset (the decider's in-path rail)
    self.alt_scene_id = ALT_SCENE_ID  # the scene's maneuver id for the ladder's other turn in hand (a new one per encounter)
    self.alt_was_on = False
    self.alt_route_id: int | None = None  # the route's maneuver the ladder's other turn was taken in place of (turn_skip's)
    self.not_straight_id: int | None = None  # the left last logged as skipped for an opposing road not known straight
    self.route_reach_m = 0.  # route time's last R_req for the unprotected left (its plan's rReq when skipped for it)
    self.last_holding = False     # the last plan's scene held the car (the decider's clearBox waits it out)
    self.params = None            # Params (main()): TurnPilotJunctionOverrides is written through it (signal_seen_override)
    self.override_failed_t = 0.   # s overrideWriteFailed has left to show
    self.always_drive = ALWAYS_DRIVE_DEFAULT  # TurnPilotAlwaysDrive (main())
    self.shadow = False           # the last plan's shadow mode: the switch does nothing in it (always_drive_on)
    self.rht = True               # this frame's right_hand_traffic (update())
    self.sign_clear_m = 0.        # m driven, forward runs seeing the road, since one had a stop-sign box (watch_signs), 0 at the start
    self.forward_age = 0.         # s since a forward detector run came (watch_signs)
    self.rel_takeover = Takeover()  # the model's takeover predicate frame by frame, a go out or not (watch_release)
    self.rel_takeover_held = False  # ...held this frame
    self.reset()

  def reset(self, detector: bool = True, keep_turn: bool = False):
    """Back to TurnPilot just turned on. detector False (a fault) keeps what the detector and the scene have shown: the
    detector's deadlines and the scene's stops in force run on through a fault. keep_turn (a fault with a turn committed)
    keeps the route, the turn and its desire state: a committed turn is never dropped, nor pulsed afresh in the turn."""
    if detector:
      self.scene = ScenePolicy()
      # the map stops' latch is position-free and outlives faults, nav gaps, reroutes and handbacks: only turning
      # TurnPilot off clears it (map_controls.py)
      self.map_controls = MapControlPolicy(self.yellow_decel)
      # an unprotected turn's line crossed with no go of TurnPilot's (map_controls.Driven): [its maneuver id, its side, deg
      # turned toward it since], checked at its pass for a turn driven straight through (check_driven_left)
      self.driven_left: list | None = None
      self.driven_checked: tuple | None = None  # ...the crossing (map_controls.Driven's key) last checked: once each
      self.lights_age = float('inf')  # s since the last turnpilotLights
      self.camera_read = None         # the camera's lights from the last road run (or a reader off), camera_light_input's
      self.camera_age = float('inf')  # s since it
      self.was_stopping = False    # the last plan held the car (shouldStop)
      self.release_go = False      # ...and let go of it at standstill: the turn pulses again now
      self.detector_seen = False   # fresh detections since TurnPilot was turned on
      self.stale_s = 0.            # how long the scene has been unavailable
      self.waited_s = 0.           # how long modeld has run since TurnPilot was turned on, before the first detection
      self.disengaged_s = 0.       # ...of it with openpilot disengaged
      self.engaged_wait_s = 0.     # time since the first engaged frame, before the first detection
      self.left = UnprotectedLeft()  # the autonomous unprotected left (unprotected_left.py); kept through a fault, as its latch
      self.left_hold_was = False     # ...its own hold (hold_here) last frame: a withdrawn go's is its rising edge
      self.stop_go = StopGo()  # the stop-sign decider: kept through a fault, as the latch it decides for
      self.stop_go.on_signal_seen = self.signal_seen_override
      self.stop_go.on_downgrade = self.downgrade_override
      self.sg_skip_token = None  # the decider's encounter in which the route's turn at its control was skipped (stop_go_update)
      self.sg_veto = ''          # a signal veto that skipped the turn while the decider's go could be withdrawn: Frame.signal_veto
      self.sg_route_maneuver: str | None = None  # the route's maneuver at its control, last frame (before a skip's union)
    self.go_cert.withdraw('reset')  # a disabled plan carries no go
    self.decider_turn_id: int | None = None  # the route's turn whose stop sign the decider acted for last frame (update())
    self.stopped_s = 0.            # standing engaged with nothing holding the car (carStoppedTakeControl)
    self.stopped_signal = False    # ...within STOPPED_CONTROL_M of a mapped signal (stopped_unheld): the note doesn't chime
    self.stopped_said = False      # ...handed back for it: until the stand ends
    self.green_s = 0.              # ...under the switch, s the camera has read that signal steady green (LIGHT_GREEN_STEADY_S)
    self.green_said = False        # ...said for this green
    self.green_alert_t = 0.        # ...s left of its lightGreenNotMoving alert
    self.stall_s = 0.              # standing in a route turn, nothing holding the car (stalledTakeControl)
    self.stall_stand_s = 0.        # ...standing there, a corroborated lead or not (LEAD_STAND_MAX_S)
    self.stall_said = False        # ...handed back for it (its log once)
    self.stall_mid: int | None = None  # under the switch, the maneuver the stall's pulse and lifts are for (LEFT-21)...
    self.stall_pulsed = False      # ...its one fresh turn pulse asked for
    self.stall_lifts = 0           # ...its clearBox lifts certified
    self.stall_lift_id: int | None = None  # ...the one out, or the last (GoCertifier.id)
    self.stall_gap_s = math.inf    # ...s since the last one ended
    self.stall_why = ''            # ...why none may go now, as last logged
    self.stall_handed = False      # ...handed back for a refused lift: stays so until the car moves, a pedal or the turn changes
    self.stall_alert_t = 0.        # ...s left of its stallLift alert
    self.unsig_why = ''            # under the switch, why a release where no signal is mapped isn't lifted, as last logged
    self.unstick_done = False      # under the switch, section 4.5's lift went for this release: no second try (stopped_unheld)
    self.unstick_id: int | None = None  # ...as this go (GoCertifier.id)
    self.unstick_alert_t = 0.      # ...s left of its unstickLift alert
    self.ped_hold_s = 0.           # the crosswalk hold has stood the car this long (SC-02, crosswalk_standoff)...
    self.ped_stand_s = 0.          # ...with every walker it is for standing
    self.ped_said = False          # ...its note chimed
    self.ped_handed = False        # ...handed back, held, while the hold stands the car
    self.ped_alert_t = 0.          # ...s left of its pedestrianWaiting alert
    self.person_handed = False     # the person-may-be-in-front latch handed back (WF-13: person_front_handback)
    self.people_runs: dict = {}    # (camera, view) -> [s old, the person boxes] of each camera's last detector run
    self.abandon_s = 0.            # the model steering away from the route's turn in the junction (turn_abandoned)
    self.wrong_s = 0.              # s of reads of a yellow line on the car's right (wrong_side)
    self.wrong_unread_s = 0.       # ...since the last read of that line
    self.wrong_side_on = False     # ...handed back for it, until the driver takes over
    self.or_win: list = []         # (odometer, deg) of the car's heading change off the route's maneuvers (off_route_turn)
    self.or_shape: tuple = ()      # navInstruction's (routeTurns, routeTurnsStart, routeTurnsStep, odometer at receipt), or ()
    self.or_near_odo = -1e9        # odometer where a route maneuver or a turn desire last was near
    self.or_id, self.or_after, self.or_prev_after = None, None, None  # the maneuver, its bearing out, the last one's
    self.or_on = False             # handed back for an off-route turn, until the driver takes over
    self.abandoned_id: int | None = None  # ...the maneuver handed back for it, until the driver takes over
    self.straight: StraightDrive | None = None  # R8b's watch over a committed turn driven straight (watch_straight)
    self.straight_cleared = False  # ...R8b's fire cleared the turn's desire this frame (CLEAR_DESIRE in this very plan)
    self.lead_seen_s = float('inf')  # since the live scene last corroborated modelV2's close lead (watch_lead)
    self.lead_why = ''             # ...what did
    self.lead_held = False         # this frame: a close lead the scene corroborates, within LEAD_SEEN_HOLD_S (watch_lead)
    self.lead_still_s = 0.         # ...s since a vehicle track at it was last seen moving (QUEUE_MOVING_SPEED), while close
    self.missed_t = 0.             # turnMissed is held while this counts down
    self.notices = RouteNotices()  # the route's stop signs said at its start and 300 m before each it couldn't avoid (3.4)
    if not keep_turn:
      # maneuver ids never repeat: the scene kept through a fault holds ids (its commitment, a gas release, a carried hold)
      self.follower = RouteFollower(self.follower.maneuver_id + 1 if hasattr(self, 'follower') else 0)
      old = getattr(self, 'desires', None)
      self.desires = DesireTiming()
      if old is not None and not detector:
        # a turn desire still in the model's queue (a turn passed during a fault) comes out of it as for any passed turn
        self.desires.carry_clear(old)
      self.skip = TurnSkip()
      self.dropped_id: int | None = None  # the maneuver in hand at a handback: not taken up again
      self.turn_cap_id: int | None = None  # ...whose turn speed cap stays, its desire having gone out
      self.turn_held_id: int | None = None  # ...whose turn desire was out: held to its clear, never cancelled
      self.skip_cap_id: int | None = None  # a dropped turn skipped on taking it up: its cap stays until it is passed
      self.keep_follow_id: int | None = None  # a keep that couldn't be sent, the model driving the fork (keep_unsent, R7)
      self.keep_rerouted_t = 0.  # ...navd recomputed past it: the skip's chime and 'Rerouting' while this counts down
      self.veto_hold_id: int | None = None  # a turn vetoed standing at a hold: the hold keeps it until it lets go
      self.vetoed: VetoedTurn | None = None  # the turn the driver overruled, by its junction: skipped across a reroute
      self.line_deferred_id: int | None = None  # the maneuver last logged as waiting on a line veto's record (line_veto_defers)
      self.go_id: int | None = None  # the turn a hold let go of at standstill: checked for a miss and a stall
      self.go_rolled = False         # ...the car has rolled since that go (STANDSTILL_SPEED or more)
      self.go_spent = False          # ...and then come to rest again short of the hold line: no longer under way (went)
      self.go_clock_again = False    # ...its release's no-move clock run again from that stand (go_spent): a bound, no relift
      self.post_go: PostGo | None = None  # ...and driven under its post-go profile (start_post_go)
      self.no_move_limit = NO_MOVE_S       # ...withdrawn if the car hasn't moved this long after it (no_move)
      self.go_wait_s: float | None = None  # s stood since TurnPilot's go, nothing else holding the car; None: moved, or no go
      self.go_lead_s = 0.                  # ...and behind a corroborated lead (NO_MOVE_MAX_S with go_wait_s)
      self.go_withdrawn = False            # the go withdrawn: the car held standing, handed back (cantStartTakeControl)
      self.go_in_junction = False          # the go came with the car in the junction (go_at_line): never withdrawn
      self.go_lifted = False               # ...certified for plannerd's lift (certify_go)
      self.go_cert_id: int | None = None   # ...as this go (GoCertifier.id)
      self.go_moved = False                # ...and the car has rolled since (no_move)
      self.left_launch_armed = False       # ...an unprotected left's, checked for its launch until its commit (left_launch)
      self.go_left = False                 # ...is the unprotected left's: it keeps its own hold (LEFT-06), never a release's rules
      self.go_straight = False             # ...is a straight release's (no turn): judged at the junction it stands at
      self.rel_lift_m = 0.                 # under the switch, the release's lifts' metres by odometer (RELEASE_LIFT_M)...
      self.rel_goes = 0                    # ...its certified goes (RELEASE_GOES)...
      self.rel_gap_s = 0.                  # ...and s since its last go ended, or since TurnPilot's go (RELIFT_GAP_S)
      self.cant_start_unheld = False       # the switch's no-hold cantStartTakeControl, until the model drives off (no_move)
      self.approach_yaw_deg = 0.           # the car's heading change toward the route's turn since its first pulse (track_yaw)
      self._yaw_id: int | None = None      # ...for this maneuver
    else:
      self.desires.hold_off()  # the disabled plan dropped the desire: as on re-engaging, a fresh pulse only before the turn
    self.lanes = LanePlanner()
    self.paint_rule_lanes = LanePlanner(paint_rule=True)  # what the lane planner would ask for, while it's turned off
    self.mismatch = LaneMismatch.none
    self._mismatch_candidate, self._mismatch_frames = LaneMismatch.none, 0
    self._unknown_onset = self._unknown_heard = None  # (maneuver id,): laneUnknown's onset under way, the last one said (R12)
    self._unknown_quiet = False
    self.acting = False
    self.last_desire = Desire.none
    self.hand_always_drive()  # the components made afresh here have the switch at once, mid-frame too (a fault's reset)

  @property
  def always_drive_on(self) -> bool:
    """TurnPilotAlwaysDrive as the components act on it: the param, and never in shadow mode (ALWAYS_DRIVE.md section 1)."""
    return self.always_drive and not self.shadow

  def hand_always_drive(self) -> None:
    """The switch to every component that acts on it (always_drive_on); the stop-sign decider has it in its Frame."""
    on = self.always_drive_on
    for part in (self.lanes, self.paint_rule_lanes, self.left, self.map_controls, self.skip):
      part.always_drive = on

  @property
  def release_stand(self) -> bool:
    """Under the switch, the stand is a release's (its no-move clock running, or its no-hold handback up): neither the stall
    nor the stopped watch counts it (no_move owns it, ALWAYS_DRIVE.md ST-04)."""
    return self.always_drive_on and not self.go_left and (self.go_wait_s is not None or self.cant_start_unheld)

  @property
  def turn_committed(self) -> bool:
    """The route's maneuver is a turn whose desire went out and isn't over: it is held through a gap in the route
    (route_follower.py), and a fallback's route cancel waits for it."""
    d, rf = self.desires, self.follower
    return d.maneuver_id == rf.maneuver_id and d.started and not d.done and d.side != 0 and rf.upcoming is not None

  def fault_track(self, sm) -> None:
    """A fault (plan()) with a turn committed: the frame's dead-reckoning and yaw, if update() didn't get to them, so the
    turn is over where it would have been (passed, or cleared at its angle). Frozen through the fault, the junction stayed
    ahead of the car and the turn was held, and pulsed again, out on the road after it."""
    if not self.turn_committed:
      return
    try:
      cs = sm['carState']
      if not self._rf_ticked:
        self.follower.dead_reckon(max(cs.vEgo, 0.), DT_MDL)
      if not self._yaw_ticked:
        self.desires.track_yaw(default_yaw_rate(cs, sm['modelV2']), DT_MDL)
    except Exception:
      log_error("turnpilotd.fault_track_failed")

  def watch_model(self, model, enabled: bool) -> None:
    """Note a fallback from the big driving model, whether or not TurnPilot is on: it lasts the drive."""
    if model.big:
      self.big_seen = True
    elif self.big_seen and not self.fell_back:
      self.fell_back = True
      self.cancel_route = enabled  # the route TurnPilot was driving; with it off, navd's route isn't TurnPilot's

  def big_model_failed(self, enabled: bool) -> None:
    """modeld's ChestnutActive is False: the big model isn't running this drive (it fell back, or failed to load). A
    fallback as watch_model's, but it outlives this process. The route is cancelled only by the process that saw the
    big model drive it."""
    if not self.fell_back:
      self.fell_back = True
      self.cancel_route = enabled and self.big_seen

  def handback(self, available: bool, model_running: bool = True, engaged: bool = False) -> int:
    """Why TurnPilot can't act now, as a Handback: none while it can, and while the detector hasn't started yet, short of
    DETECTOR_START_DEADLINE_S of modeld running (model_running: a modelV2 this frame) once it could load the detector
    (engaged: carControl.enabled; module docstring), nor past DETECTOR_START_DEADLINE_S from the first engaged frame.
    Once a frame: a fault after update() already ran it takes this frame's."""
    if self.handback_now is not None:
      return self.handback_now
    self.detector_seen = self.detector_seen or available
    self.stale_s = 0. if available else self.stale_s + DT_MDL
    if not self.detector_seen and model_running:
      if not engaged:
        self.disengaged_s += DT_MDL
      if not engaged or self.disengaged_s >= LOAD_CHANCE_S - DT_MDL / 2:
        self.waited_s += DT_MDL
    if not self.detector_seen and (engaged or self.engaged_wait_s > 0.):
      self.engaged_wait_s += DT_MDL
    if self.fell_back:
      self.handback_now = Handback.modelFellBack
    elif self.detector_seen and self.stale_s > DETECTOR_DEADLINE_S - STALE_S:
      self.handback_now = Handback.detectorStopped
    elif not self.detector_seen and max(self.waited_s, self.engaged_wait_s) >= DETECTOR_START_DEADLINE_S - DT_MDL / 2:
      self.handback_now = Handback.noDetector
    else:
      self.handback_now = Handback.none
    return self.handback_now

  def _lane_mismatch(self, found) -> int:
    """found, once it has held MISMATCH_CONFIRM_FRAMES; logged when it changes."""
    self._mismatch_frames = self._mismatch_frames + 1 if found == self._mismatch_candidate else 1
    self._mismatch_candidate = found
    if (found == LaneMismatch.none or self._mismatch_frames >= MISMATCH_CONFIRM_FRAMES) and found != self.mismatch:
      if found != LaneMismatch.none:
        rf = self.follower
        cloudlog.event("turnpilot_lane_mismatch", error=True, mismatch=str(found), maneuver_distance=round(rf.distance, 1),
                       through_distance=round(rf.through_distance, 1))
      self.mismatch = found
    return self.mismatch

  def unknown_sound(self, mismatch, maneuver_id) -> int:
    """The plan's laneMismatch from _lane_mismatch's (R12, ALWAYS_DRIVE.md step S4): under the switch 'Can't Confirm Lane' chimes
    once a maneuver, its first onset of laneUnknown for a maneuver id published as it is (selfdrived's chime) for as long as it
    lasts, every later onset for that maneuver as laneUnknownQuiet (the same line, no sound), the next maneuver's chiming again;
    it is the one sound saying the car may turn from a lane it can't place, so never silent for a maneuver. Off it, or in
    shadow mode, every onset as today; every other mismatch as it is."""
    if mismatch != LaneMismatch.laneUnknown:
      self._unknown_onset = None
      return mismatch
    if self._unknown_onset != (maneuver_id,):  # an onset, or the next maneuver under the line
      self._unknown_quiet = self.always_drive_on and self._unknown_heard == (maneuver_id,)
      self._unknown_onset = self._unknown_heard = (maneuver_id,)
    return LaneMismatch.laneUnknownQuiet if self._unknown_quiet else mismatch

  def stop(self, reason: str) -> None:
    """TurnPilot goes disabled (turned off, or a fault: reason turnedOff or fault) and resets: an offered lane change is
    cancelled out loud, its abort held ABORT_HOLD_S on the disabled plans (fill_stop_abort), as any abort. A fault keeps
    the detector's and the scene's state (reset())."""
    if self.lanes.cancel(reason):
      self.stop_abort, self._stop_abort_t = reason, ABORT_HOLD_S
    if reason == 'turnedOff':
      self.hold_v = None
    keep_turn = reason == 'fault' and self.turn_committed
    if reason == 'fault':
      self.scene.fault(self.follower.maneuver_id)
    self.reset(detector=reason == 'turnedOff', keep_turn=keep_turn)

  def fill_stop_abort(self, plan) -> None:
    if self._stop_abort_t > 0.:
      self._stop_abort_t -= DT_MDL
      plan.laneChangeAbort = self.stop_abort

  def update(self, sm, scene, enabled: bool, shadow: bool, driver_signals: bool | None = None, right_hand_traffic: bool = True, paint=(),
             paint_rule: bool = True, inputs_valid: bool = True, raw=None, raw_far=None, extra_objects=()):
    """A turnpilotPlan message from sm's carState, navInstruction and modelV2, a TurnpilotScene, and the lane paint of
    modelV2's four lane lines (empty when unknown). paint_rule: TurnPilotLanePaintActive; inputs_valid: carState and
    modelV2 pass their checks; raw: the detector run that arrived this frame (scene.tracker.RawFrame), or None; raw_far:
    the far crop's run that arrived this frame (view far), or None; extra_objects: tracker B's b_only pedestrians (main():
    TurnPilotFarPeds, scene/tracker_b.py), for the scene policy alone (ScenePolicy.update): everything else reads the scene's.
    driver_signals: None (main) takes it from the turn signal's status, the driver signalling unless the car's own signal
    is ready (car_signals.py); a bool forces it, False then being the openpilot-signals path with no lamp lead gate."""
    self.handback_now = self.scene_now = self.map_now = self.left_now = self.sg_now = None
    msg = messaging.new_message('turnpilotPlan')
    plan = msg.turnpilotPlan
    plan.shadow = shadow
    plan.accelCap = NO_ACCEL_CAP
    plan.speedCap = NO_SPEED_CAP
    self.watch_model(sm['modelV2'], enabled)
    if not enabled:
      self.stop('turnedOff')
      plan.state = State.disabled
      self.fill_stop_abort(plan)
      self.sweeping = self.sweep_on = False
      self.watch_junction(None, 0.)
      return msg
    self._stop_abort_t = 0.

    cs, model = sm['carState'], sm['modelV2']
    v_ego = max(cs.vEgo, 0.)
    port = cs.carSignal
    signal_status = self.turn_signal.status_for(self.car_signal_flags, self.car_signal_mode, port, DT_MDL, self.unverified_why)
    signal_forced = driver_signals is not None
    if driver_signals is None:
      driver_signals = driver_signals_for(signal_status)
    self.watch_far(sm)
    rf = self.follower
    rf.update(sm['navInstruction'], sm.updated['navInstruction'], sm.valid['navInstruction'], v_ego, DT_MDL,
              committed=self.turn_committed or self.straight_fired)
    self._rf_ticked = True
    upcoming = None if rf.arrived else rf.upcoming
    available = scene.available
    objects = list(scene.objects) if available else []

    handback = self.handback(available, sm.updated['modelV2'], sm['carControl'].enabled)
    plan.handback = handback
    self.watch_people(raw, raw_far)
    self.rht = right_hand_traffic
    self.watch_lead(model, objects, raw)
    acting = self.detector_seen and handback == Handback.none
    lat_active = sm['carControl'].latActive
    if self.acting and not acting and lat_active:
      self.hold_v = v_ego
    if acting or not lat_active:
      self.hold_v = None
    elif self.hold_v is not None:
      self.hold_v = v_ego if cs.gasPressed else min(self.hold_v, v_ego)  # the driver's gas takes the car on, not back
    if self.acting and not acting:
      # handing back: drop the maneuver in hand, keep its turn cap if its desire went out (the car may be in the turn),
      # and a turn desire that went out is held to its clear, never cancelled; anything else is abandoned
      d = self.desires
      in_turn = d.maneuver_id == rf.maneuver_id and d.started and not d.done and d.side != 0
      self.dropped_id = rf.maneuver_id
      self.turn_cap_id = rf.maneuver_id if d.started else None
      self.turn_held_id = rf.maneuver_id if in_turn else None
      if not in_turn:
        d.abandon()
      # an offered lane change is cancelled out loud, its abort held on this planner; one under way goes on
      self.lanes.cancel('handedBack')
      self.paint_rule_lanes.cancel('handedBack')
    elif acting and not self.acting and self.dropped_id is not None:
      self._take_up_dropped(upcoming, rf, v_ego, right_hand_traffic)
    self.acting = acting
    self.check_straight(rf, v_ego, acting, said=self.check_missed(rf, v_ego, acting))
    self.check_keep_followed(rf)
    self.check_driven_left(rf, default_yaw_rate(cs, model), v_ego, acting)
    self.track_yaw(upcoming, rf, default_yaw_rate(cs, model), v_ego)
    self.track_yaw20(default_yaw_rate(cs, model), v_ego)

    # the map's stop signs and signals: every frame, whatever the maneuver (a skipped turn keeps its stop). And the route's
    # unprotected turn while TurnPilot drives it: not skipped (too fast: skipped out loud and rerouted, never braked for at
    # the floor; last frame's skip, a frame late at most), not dropped at a handback. A held latch stays whatever
    # Its stop is engaged only where it can be made at -2.0; one that can't (first seen too close at speed) is skipped
    skipped = self.skip.skipped and self.skip.maneuver_id == rf.maneuver_id
    driving = acting and rf.maneuver_id != self.dropped_id and not skipped
    # an engaged one stays through a handback and after it, as a stop sign's stop does: the stops in force stay, and the
    # detector back is no reason to cross the blind junction without its stop and its handback
    engaged = self.map_controls.turn_engaged(rf.maneuver_id) and not skipped
    # the stop-sign decider acting for this turn's junction (last frame's decision: it runs after the left, below) judges the
    # turn's crossing itself, a left's oncoming lanes among its conflicts (stop_rules.TURN_SETS), and its go is the turn's first
    # pulse: the autonomous left stands aside there, its approach's pulse hold and its hold line's feasibility with it. A left
    # whose pulse waited for the left's own go had none at the decider's go, and rolling past the left's hold line it was
    # skipped (unprotectedStop): the decider launched for the left, the car crossed straight and its lamp went dark
    decided = self.decider_turn_id == rf.maneuver_id and not engaged
    turn, stop_infeasible = None, False
    if (driving or engaged) and not decided and unprotected_turn(upcoming, right_hand_traffic):
      if engaged or turn_stop_feasible(v_ego, rf.distance):
        turn = (rf.maneuver_id, rf.distance, 'left' if upcoming.side == LEFT else 'right')
      else:
        stop_infeasible = True
    elif engaged and upcoming is None:
      turn = KEEP_TURN  # the route gone with the handback: it carries on, dead-reckoned
    left_cmd = self.unprotected_left(turn, upcoming, rf, v_ego, acting, objects, raw, raw_far, enabled=bool(sm['carControl'].enabled))
    # the stop-sign decider before map_update, so its release, the end of shouldStop and its go land in one plan
    sg = self.sg_now = self.stop_go_update(sm, cs, model, v_ego, acting, objects, rf)
    # the stop-sign ladder's other turn in hand (stop_go: 7.1 rung 2, O12), or done in place of the route's turn here: the route's
    # turn isn't the car's here, so turn_skip never judges it (skeptic SG4c A': its unprotected hold-line stop infeasible from
    # the other turn's launch, its desire never out, it was skipped mid-junction, "Too Fast to Turn / Rerouting" over the go)
    alt_on = sg.alt is not None and sg.state != 'off'
    if alt_on and upcoming is not None and upcoming.kind == TURN and \
       abs(rf.distance - (float(sg.alt['toNode']) - sg.position_m)) <= ALT_LANES_M:
      self.alt_route_id = rf.maneuver_id  # the route turns at this junction: its turn is missed here once the other is taken
    alt_replaced = alt_on or rf.maneuver_id == self.alt_route_id
    if alt_replaced:
      stop_infeasible = False
    turn_side = None if upcoming is None or upcoming.kind != TURN or upcoming.side == 0 else 'left' if upcoming.side == LEFT else 'right'
    self.decider_turn_id = rf.maneuver_id if sg.acting and sg.state not in ('off', 'done') and self.sg_route_maneuver == turn_side \
      else None
    mapd = self.map_update(sm, v_ego, cs.gasPressed, turn, left_cmd, release=sg.release, creep=sg.creep,
                           stop=StopCommand(sg.latch, sg.reason, sg.action))
    took_over = cs.gasPressed and not sm['carControl'].enabled  # the driver driving, never a go (module docstring)
    # the autonomous left holds its turn's pulse until its go from the line (unprotected_left.py: approach), so its first pulse
    # is that go, never too late (turn_skip.py: pulse_at_go); so does a turn at a stop sign the decider acts for (2.4 item 3:
    # no turn desire before its go, a pulse curls the car 39-49 deg and throws away a side's reach)
    sg_turn = sg.acting and sg.state in STOP_GO_HOLDS_PULSE and (sg.maneuver in ('left', 'right') or sg.alt is not None)
    hold_pulse = (self.left_now is not None and self.left_now.state in LEFT_HOLDS_PULSE) or sg_turn
    # ...and its first pulse is the decider's go, from the go on too (the go frame is 'going'): never skipped tooLate there
    sg_go_turn = sg.acting and sg.state not in ('off', 'done') and (sg.maneuver in ('left', 'right') or sg.alt is not None)
    pulse_at_go = (self.left_now is not None and self.left_now.state != 'off') or sg_go_turn

    # too fast to make the turn: skipped, so nothing below acts on it (no desire, no cap, no turn holds or lane changes)
    desire_out = self.desires.maneuver_id == rf.maneuver_id and self.desires.started
    # an autonomous left the detector can't see far enough for (its R_req from the opposing road's mapped speed past
    # TurnPilotLeftRangeM): never attempted, skipped as soon as it is the maneuver and rerouted (turn_skip.py)
    # The far crop counts here while it is on (reading, or waiting for room or calibration), and once the left's stop is engaged
    # nothing here skips it: a reach lost there (the far crop off for the drive) is the left's own handback at its line
    # (unprotected_left: farOff), the car held, never a skip and a reroute from the line
    # So is one whose opposing road isn't known straight out to that R_req (navd's maneuverOpposingStraightM: the band is straight
    # on from our approach, skeptic G4), said by its own log line
    # R_req is taken where the car may stand: LINE_D plus the junction's mapped stop line's setback, or ROUTE_REACH_MARGIN_M
    # without one (BUILD_FINDINGS D4); the band straight on from our approach to that reach past where it stands
    out_of_range = False
    if acting and self.auto_left == AUTO and unprotected_turn(upcoming, right_hand_traffic) and not engaged:
      setback = self.map_controls.line_setback(rf.distance)
      margin = ROUTE_REACH_MARGIN_M if setback is None else setback
      reach = self.route_reach_m = reach_at_line(upcoming.opposing_speed, upcoming.opposing_lanes or None, margin)
      bent = reach > LINE_D + margin + upcoming.opposing_straight_m
      out_of_range = reach > self.left_reach_m(route=True) or bent
      if bent and self.not_straight_id != rf.maneuver_id:
        self.not_straight_id = rf.maneuver_id
        cloudlog.event("turnpilot.left_not_straight", error=True, r_req=round(reach, 1), margin=round(margin, 1),
                       straight_m=round(float(upcoming.opposing_straight_m), 1), distance=round(rf.distance, 1))
    self.carry_veto(upcoming, rf, v_ego)
    skip = self.skip.update(None if alt_replaced else upcoming, rf.distance, rf.maneuver_id, v_ego, desire_out, right_hand_traffic, lat_active,
                            rf.ahead, rf.ahead_distance, stop_d=self.map_controls.stop_ahead(), stop_infeasible=stop_infeasible,
                            pulse_at_go=pulse_at_go, out_of_range=out_of_range)
    # a turn whose bearings turn the other way from its modifier: the side the car would signal and steer is in doubt,
    # so it is skipped out loud and rerouted as soon as it is the maneuver, in every mode (car_signals.py), never a
    # committed one
    if acting and upcoming is not None and upcoming.kind in (TURN, KEEP) and upcoming.bearing_side and \
       upcoming.bearing_side != upcoming.side and not skip.skipped and not desire_out:
      self.skip.skip(upcoming, rf.distance, rf.maneuver_id, v_ego, 'sideUncertain')
      skip = SkipState(True, False)
    maneuver = None if skip.skipped else upcoming
    # navd reroutes on it and selfdrived alerts: only while TurnPilot is driving the route (handback outranks it). A turn
    # just past a bend that will be skipped once it is the maneuver is announced now, before its junction (turn_skip.py)
    plan.tooFastToTurn = (skip.skipped or (skip.ahead_skipped and rf.maneuver_id != self.dropped_id)) and acting

    act_on = maneuver if acting and rf.maneuver_id != self.dropped_id else None
    held = act_on is None and rf.maneuver_id == self.turn_held_id
    desire_on = maneuver if held else act_on
    cap_on = act_on if act_on is not None else (maneuver if rf.maneuver_id == self.turn_cap_id else None)
    if cap_on is None and rf.maneuver_id == self.skip_cap_id:
      cap_on = upcoming  # the skipped turn's pulse may have been taken before it was cleared: never sped up into it
    cap_d = rf.distance

    # the scene's holds only ever brake, so they keep the real maneuver through a handback, and through a veto's skip
    # while a stop brings the car to rest short of the junction, until no such stop is in force: the veto never pulls the
    # car away, the hold's go does (straight on)
    if self.veto_hold_id is not None and self.veto_hold_id != rf.maneuver_id:
      self.veto_hold_id = None
    scene_m = upcoming if self.veto_hold_id is not None else maneuver
    scene_d, scene_id, scene_yaw = rf.distance, rf.maneuver_id, self.approach_yaw_deg
    # the stop-sign ladder's other turn (stop_go: 7.1 rung 2, O12), from when it is taken until its encounter is over, is the
    # turn the car makes here: the scene's junction holds (a right's crosswalk, a left's oncoming hold, its road frame turned by
    # the car's yaw since the decider's go, its pulse) and the turn's speed cap are its, never the route's maneuver here (a route
    # left's oncoming hold through a right's launch; no oncoming hold for a left taken instead of a right). None going straight
    if alt_on:
      if not self.alt_was_on:
        self.alt_scene_id -= 1
        self.turn_signal.forget(ALT_MANEUVER_ID)  # another encounter's other turn: armed afresh, whatever the last one's end
      scene_m, scene_d, scene_id = alt_maneuver(sg.alt), float(sg.alt['toNode']) - sg.position_m, self.alt_scene_id
      scene_yaw = self.sg_turned * scene_m.side if scene_m is not None else 0.
      cap_on, cap_d = scene_m, scene_d
    self.alt_was_on = alt_on
    left_stops = self.left_now.stops if self.left_now is not None else []
    self.scene.curb_cap = self.curb_ped_cap
    decision = self.scene_now = self.scene.update(objects, available, scene_m, scene_d, v_ego, right_hand_traffic, scene_id,
                                                 took_over, extra_stops=mapd.stops + left_stops + sg.stops, heading_deg=scene_yaw,
                                                 path=predicted_path(model), extra_objects=extra_objects if available else (),
                                                 driver=not sm['carControl'].enabled or cs.gasPressed,
                                                 raw_people=self.people_in_front(model))
    self.last_holding = decision.holding
    # a TurnPilot stop in force brings the car to rest at or short of the junction (shouldStop at its end: a walker's
    # hold, a red, a stop sign), braking toward it on the speed cap's curve (shouldStop false until about standstill) or
    # standing at it. Keyed on a stop, never a cap (decision.holding is also a yield's): a veto then is decided as standing
    # at the stop (car_signals.py, DESIGN 6.6)
    stopping = decision.should_stop or (decision.rest_d is not None and decision.rest_d <= rf.distance + PROTECT_PAST_M)
    if not stopping:
      self.veto_hold_id = None
    # a hold let go with the car standing (the walker cleared, a stop's go): the model won't pull away on its own, and the
    # turn's pulse is long forgotten, so it goes again in this very plan, the one that lets the car go: held, the desire
    # was dropped to none, so it is a fresh edge (desire_timing.py). BeamNG: let go a frame before its pulse, the car drove
    # straight across the junction
    # Under the switch a hold that only caps the speed counts once the car has come to rest under it (ALWAYS_DRIVE.md ST-08),
    # and its go is the frame the cap lets go
    stood = decision.should_stop or (self.always_drive_on and decision.holding and v_ego < STANDSTILL_SPEED)
    held_now = decision.should_stop or (self.always_drive_on and decision.holding)
    self.release_go = self.was_stopping and not held_now and v_ego < STANDSTILL_SPEED
    self.was_stopping = stood or (self.was_stopping and held_now)
    # the certified go out, a frame on (its window, the model's takeover), and withdrawn the frame what it rests on goes
    self.go_cert.update(model, v_ego, DT_MDL)
    self.watch_release(sm, cs, v_ego, model)
    self.withdraw_go(sm, cs, acting, available, objects, raw, decision, took_over, self.release_distance(rf.distance))
    if sg.withdraw and self.go_cert.active and self.go_cert.cause in STOP_GO_CAUSES and self.go_cert.id != self.stall_lift_id:
      self.go_cert.withdraw(f"stopGo {sg.withdraw}")
    decider_go = sg.go_cause != 'none' or sg.release is not None
    if sg.go_cause != 'none' and acting:
      self.certify_stop_go(sm, cs, available, objects, raw, decision, took_over, sg)
    withdrawn = self.no_move(sm, cs, v_ego, acting, decision) or self.left_launch(sm, rf, v_ego)
    if withdrawn:
      self.go_cert.withdraw('noMove')
    self.relift(sm, cs, v_ego, acting, available, objects, raw, decision, took_over, self.release_distance(rf.distance),
                None if self.go_straight else upcoming, right_hand_traffic)
    self.release_go = self.release_go and not withdrawn  # handed back: a hold that comes and goes is no new go

    lanes = model_lanes_from_msg(model)
    # a keep goes only where it may (lane_planner.keep_blocker: a lane line that way, and toward the far side a lane of ours
    # beyond it, no yellow line read there now or held); one that can't go by keep_last is said
    keep_paint = self.lanes.yellow.apply(paint)
    keep_why = {side: keep_blocker(lanes, side, right_hand_traffic, keep_paint) for side in (LEFT, RIGHT)}
    # the stop-sign ladder's other turn (stop_go: 7.1 rung 2, O12) decides this junction: its synthesized turn is the desire
    # (from the decider's go, as a stop-sign turn's pulse is), never the route's maneuver here
    d_on, d_dist, d_id, d_passed, d_replaced = desire_on, rf.distance, rf.maneuver_id, rf.passed, rf.replaced
    if sg.acting and sg.alt is not None and sg.state != 'off':
      d_on, d_dist, d_id, d_passed, d_replaced = alt_maneuver(sg.alt), max(sg.alt['toNode'] - sg.position_m, 0.), ALT_MANEUVER_ID, False, False
      if sg.alt.get('vetoed'):
        d_on = None  # the driver overruled it (stop_go: handed back, held): no desire for it, ever
    if sg.turn_missed:
      self.missed_t = MISSED_HOLD_S  # the other turn taken: the route's planned turn is missed, navd reroutes at once (3.5)
    self.clear_driven_left(rf)
    self.clear_withdrawn_left(rf)
    # no turn on red, ever (SAT_VALIDATION row 14; Kellen: "it should always stop on red"): a red read at the turn's junction
    # from any source (release_light: the oracle's, a mapped signal's, the camera's in shadow too, as it vetoes lifts) holds
    # the desire at none, standing as moving, until the car is TURN_STARTED_DEG into the turn (a turn under way is never
    # taken back). DesireTiming keeps its pulse's timing, so the turn isn't judged too late, and the green's go pulses a fresh
    # edge as after any hold. Only short of the line: a car that went (its yellow committed, its signal's line crossed faster
    # than COMMIT_SPEED, the scene committed to or the driver released into the junction) finishes the turn on a later red
    went = self.map_controls.signal_went(d_dist) or d_id in (self.scene.committed_id, self.scene.released_id)
    red_here = not went and self.map_controls.release_light(d_dist) == 'red'
    desire = self.desires.update(d_on, d_dist, d_id, d_passed, v_ego, cs.steeringAngleDeg,
                                 not keep_why[LEFT], not keep_why[RIGHT], DT_MDL, default_yaw_rate(cs, model),
                                 lat_active=lat_active, replaced=d_replaced,
                                 may_start=skip.feasible_now and not held and not hold_pulse, go=self.release_go,
                                 holding=decision.should_stop or withdrawn or red_here)
    if red_here and desire in (Desire.turnLeft, Desire.turnRight) and self.desires.turned_deg < TURN_STARTED_DEG:
      desire = Desire.none
    self._yaw_ticked = True
    self.keep_unsent(act_on, rf, v_ego, lat_active, keep_why, model, plan)
    d = self.desires
    turn_go = self.release_go and d.maneuver_id == rf.maneuver_id and d.started and d.side != 0 and not d.done and upcoming is not None
    if turn_go:
      self.go_id, self.stall_s, self.go_rolled, self.go_spent, self.go_clock_again = rf.maneuver_id, 0., False, False, False
      self.go_in_junction = not self.go_at_line(rf.distance)
      if sm['carControl'].enabled and not took_over:  # TurnPilot's go; the driver driving off a hold is the driver's
        self.start_post_go(upcoming, rf.distance, d.turned_deg, right_hand_traffic)
        if decider_go:
          self.go_wait_s = None  # the stop-sign decider's go: its own watchdog (the gate, G5's window, its retries, H5)
        else:
          self.certify_go(sm, cs, available, objects, raw, decision, took_over, rf.distance, upcoming, right_hand_traffic)
    elif self.release_go and not decider_go and sm['carControl'].enabled and not took_over and \
         (mapd.released_stopped or (self.always_drive_on and (self.left_now is None or self.left_now.state == 'off'))):
      # any other go of TurnPilot's from a standstill (a map control let go, a straight road's too): the no-move clock (gap B).
      # Under the switch a scene hold let go on a straight too, and certified as a turn's go is, at the junction it stands at
      # (ALWAYS_DRIVE.md ST-12: release_distance)
      self.go_wait_s, self.go_withdrawn, self.go_moved, self.go_lifted, self.go_in_junction = 0., False, False, False, False
      self.go_lead_s, self.go_left, self.rel_gap_s, self.go_straight = 0., False, 0., True
      self.no_move_limit, self.go_cert_id, self.go_clock_again = self.no_move_s, None, False
      cloudlog.event("turnpilot.post_go", profile='none', unprotected=False, turned_deg=0., distance=round(rf.distance, 1), at_line=True)
      if self.always_drive_on:
        self.certify_go(sm, cs, available, objects, raw, decision, took_over, self.release_distance(rf.distance), None,
                        right_hand_traffic)
    if self.post_go is not None and not turn_go:
      self.post_go.update(v_ego, default_yaw_rate(cs, model), DT_MDL)
    if held and desire not in (Desire.none, CLEAR_DESIRE) and desire != self.last_desire:
      desire = Desire.none  # held: the desire out stays out, but never a new rising edge (a repeat pulse under way)
    plan.desire = self.last_desire = desire
    # carState or modelV2 failing their checks: no change is offered, and one offered aborts. Not acting: act_on is None,
    # so nothing is offered (an offer at the handback was cancelled above, as handedBack)
    lane_kw = {'left_blindspot': cs.leftBlindspot, 'right_blindspot': cs.rightBlindspot, 'maneuver_id': rf.maneuver_id,
               'inputs_valid': inputs_valid, 'stop_d': self.map_controls.stop_ahead()}
    if not signal_forced:  # the car's lamp leads its own lane changes by the legal distance (lane_planner.py)
      lane_kw.update(lit_m=self.signal_out.lit_m, lead_m=self.signal_out.lead_m,
                     car_block=str(port.block) in ('otherSide', 'hazard') or port.hazard,
                     signals_failed=signal_status == custom.TurnpilotPlan.TurnSignal.Status.failed)
    lane_args = (act_on, rf.distance, lanes, model.meta.laneChangeState, v_ego, cs.leftBlinker, cs.rightBlinker, objects,
                 driver_signals, right_hand_traffic, DT_MDL, paint)
    self.lanes.paint_rule = paint_rule
    lc = self.lanes.update(*lane_args, **lane_kw)
    plan.laneChange, plan.signalPrompt, lane_reason = lc.lane_change, lc.prompt, lc.reason
    plan.laneChangeOffer, plan.laneChangeCountdown, plan.laneChangeAbort = lc.offer, lc.countdown, lc.abort
    plan.laneChangeWait, plan.laneChangeWant = lc.wait, max(min(lc.want, 127), -128)
    plan.laneChangeHold = lc.hold  # held for the blind spot, or its release (LC-01, under the switch)
    committed = self.desires.maneuver_id == rf.maneuver_id and self.desires.started  # this frame's desire included
    if (lc.unreachable or lc.blocked) and act_on is not None and not committed and not self.skip.skipped:
      # a map stop leaves no room to get into the maneuver's lane before it nor after its go, or the change it needs hadn't
      # gone by the window's end, the car placed in a lane it doesn't leave from (LC-03, lane_planner.py): skipped out loud
      # and rerouted (turn_skip.py), never a turn from the wrong lane. Never one whose desire went out: a committed turn goes on
      self.skip.skip(act_on, rf.distance, rf.maneuver_id, v_ego, 'laneUnreachable' if lc.unreachable else 'laneBlocked')
      plan.tooFastToTurn = True
    rule = lc if paint_rule else self.paint_rule_lanes.update(*lane_args, **lane_kw)  # its offers and aborts are never raised
    plan.laneChangePaintRule = rule.lane_change if rule.lane_change != LaneChange.none else rule.prompt
    found = lane_mismatch(act_on, rf.distance, rf.through_lanes, rf.through_distance, lanes, v_ego, right_hand_traffic) \
      if acting else LaneMismatch.none  # handback outranks it, and with the maneuver dropped it would misread the lane
    if found == LaneMismatch.none and acting and self.lanes.lane_unknown:
      found = LaneMismatch.laneUnknown  # no change from a lane it can't place: said instead
    plan.laneMismatch = self.unknown_sound(self._lane_mismatch(found), rf.maneuver_id)
    plan.accelCap = decision.accel_cap
    # past a bend or a straight-on step, the turn after it is slowed for already (route_follower.py: ahead), and still
    # once the step is dead-reckoned past, until navd moves on to the turn
    ahead_on = acting and rf.valid and rf.maneuver_id != self.dropped_id
    ahead_cap = turn_speed_cap(rf.ahead, rf.ahead_distance, v_ego, right_hand_traffic) if ahead_on else NO_SPEED_CAP
    plan.speedCap = min(decision.speed_cap, turn_speed_cap(cap_on, cap_d, v_ego, right_hand_traffic), ahead_cap,
                        NO_SPEED_CAP if self.hold_v is None else self.hold_v)
    plan.shouldStop = decision.should_stop
    if self.post_go is not None and self.post_go.over:
      self.post_go = None
    if self.post_go is not None:  # only ever lowers: a positive accel cap, a speed cap braking at -2.0 at most
      go_accel, go_speed = self.post_go.caps(v_ego)
      plan.accelCap, plan.speedCap = min(plan.accelCap, go_accel), min(plan.speedCap, go_speed)
    plan.speedCap = min(plan.speedCap, sg.speed_cap)  # the stop-sign launch's STOP_GO_V_MAX until its commit: only ever lowers
    if withdrawn:  # held: a 0 speed cap, the speed cap's -2.0 floor at most, never harder, and shouldStop once standing;
      plan.speedCap = 0.  # rolling (a lift the model never took over: 1.5 m/s at most), stopped at STOP_DECEL
      if v_ego < STANDSTILL_SPEED:
        plan.shouldStop = True
      else:
        plan.accelCap = min(plan.accelCap, -STOP_DECEL)
    self.limits = (plan.accelCap, plan.speedCap, plan.shouldStop)
    no_junction_data = maneuver is not None and maneuver.kind in (TURN, KEEP) and not maneuver.geometry
    plan.turnMissed = acting and self.missed_t > 0.
    self.missed_t = max(self.missed_t - DT_MDL, 0.)

    # the turn signal (car_signals.py): the side, the prompt, the driver overruling the car, the lamp's lead
    left_d = self.left_now
    # the unprotected left at its hold line: the autonomous left's wait there, or the line's latch holding the car (G1's
    # handback, TurnPilotAutoLeft off): a veto there is the left's handback, never a skip that lets go of the latch. So is
    # its go while the car can still stop short of the oncoming lanes (withdrawable, a frame on: the left reads the veto
    # next frame and withdraws the go, DESIGN 6.6, 6.7); past that, the turn finishes and says so
    tc = self.map_controls.turn
    at_left_line = (left_d is not None and (left_d.state in ('waitGap', 'creep', 'undecided', 'announce') or
                                            (left_d.state == 'go' and left_d.withdrawable))) or \
      (tc is not None and tc.held and self.map_controls.turn_engaged(rf.maneuver_id))
    # the car at rest short of the junction under TURN_STARTED_DEG into its turn hasn't started it, whatever stopped it (a
    # TurnPilot stop: stopping; a lead in the queue, the model: openpilot's own longitudinal), so a veto there skips the turn
    # out loud (DESIGN 6.6's standing clause, B9). Never once TurnPilot's own go from a standstill is out into the turn (a
    # hold's go, the stop-sign decider's, the left's: go_id, this frame's included, and not withdrawn since; the left's go
    # while it can still stop is at_left_line's) nor once the scene committed or was released into the junction: the turn
    # is under way there and a skip would send the car straight on a go made for the turn (6.6: finished, and said). A go
    # the car has rolled on from and come to rest again short of the hold line (GO_LINE_M out or more, go_at_line's own
    # test: a mid-block walker's hold let go, then a lead in the queue) is spent: the car stands short of the junction as
    # in B9, and the next go is a new one (M1). Rolled into the junction and stopped there, the go's launch is under way
    if self.go_id == rf.maneuver_id and not self.go_spent:
      if v_ego >= STANDSTILL_SPEED:
        self.go_rolled = True
      elif self.go_rolled and rf.distance >= GO_LINE_M:
        self.go_spent = True
        # ALWAYS_DRIVE.md ST-04 under the switch (SAT_VALIDATION run 2, red_right at 263065a): a release the model crept on (its
        # no-move clock ended as followed, no_move) and came back to rest short of the junction wasn't followed; its clock runs
        # again from this stand to the same loud no-hold handback at NO_MOVE_MAX_S, never only the signal's NEAR_CONTROL_STAND_S
        # (the run stood 90 s at a green behind the model's phantom lead with nothing in the road). A bound only: no relift
        # from it (go_clock_again), as none could come once the clock had ended
        again = self.always_drive_on and not self.go_left and self.go_wait_s is None and not self.go_withdrawn and \
          not self.go_in_junction and not self.cant_start_unheld
        if again:
          self.go_wait_s, self.go_lead_s, self.go_moved, self.go_clock_again, self.go_rolled = 0., 0., False, True, False
        cloudlog.event("turnpilot.go_spent", distance=round(rf.distance, 1), turned_deg=round(self.desires.turned_deg, 1),
                       clock_again=again)
    elif self.go_id == rf.maneuver_id and self.go_clock_again and self.go_wait_s is None and not self.go_withdrawn and \
        not self.go_in_junction and not self.cant_start_unheld:
      # per stand, not once a go: the model crept on again from that stand (its clock ended as followed) and came back to rest
      # short of the junction again, its line included (in the junction, STALL_MIN_DEG turned or the scene committed or released
      # into it, is the stall's handback); the clock runs again from this stand too, to the same handback at NO_MOVE_MAX_S
      in_junction = rf.maneuver_id in (self.scene.committed_id, self.scene.released_id) or self.desires.turned_deg >= STALL_MIN_DEG
      if v_ego >= STANDSTILL_SPEED:
        self.go_rolled = True
      elif self.go_rolled and rf.distance > 0. and not in_junction:
        self.go_wait_s, self.go_lead_s, self.go_moved, self.go_rolled = 0., 0., False, False
        cloudlog.event("turnpilot.go_spent", distance=round(rf.distance, 1), turned_deg=round(self.desires.turned_deg, 1),
                       clock_again=True)
    went = self.go_id == rf.maneuver_id and not self.go_withdrawn and not self.go_spent
    at_rest = v_ego < STANDSTILL_SPEED and rf.distance > -PROTECT_PAST_M and self.desires.turned_deg < TURN_STARTED_DEG and \
      not went and rf.maneuver_id not in (self.scene.committed_id, self.scene.released_id)
    # the stop-sign decider's go for this turn while it can still be withdrawn (going, not committed: its capped abort stops
    # short of dEdgeMin with VETO_MARGIN_M to spare, the go's own frame included): a veto skips the turn as at the line, and
    # the decider withdraws the go on it next frame (sg_veto), never "Finishing the turn" across a stop-sign left's oncoming
    # lanes with the other lamp, hazards or no lamp lit (M2: B10's rule, the decider's go being the turn's there)
    # the stop-sign ladder's other turn in hand (stop_go: 7.1 rung 2, O12) is the turn the car signals, or asks the driver to,
    # from the frame it is taken, armed at once (DESIGN phase4/blinker 8, item 2): its side under DesireTiming's id for it, none
    # going straight or once vetoed; the route's turn it replaced at this junction is never signalled again (the merge with
    # tp/integrate left the intent on the route's turn: the car turned right instead of the route's left with its left lamp lit)
    alt_sig = d_id == ALT_MANEUVER_ID
    sig_on = d_on if alt_sig else None if rf.maneuver_id == self.alt_route_id else desire_on
    if not alt_sig and self.line_veto_defers(rf):
      sig_on = None  # another turn ahead of the left the driver overruled at its line, the car still there: not signalled yet
    sg_withdrawable = sg.acting and sg.state == 'going' and sg.withdrawable and (self.decider_turn_id == rf.maneuver_id or alt_sig)
    short_of_turn = stopping or at_rest or sg_withdrawable  # one boolean for the intent's started and the skip gate below
    ts = self.turn_signal.update(status=signal_status, port=port, gates_ok=acting and not shadow, engaged=sm['carControl'].enabled,
                                 lane_side=self.lanes.signal_side, lane_pending=self.lanes.phase != Phase.IDLE or
                                 (lc.want != 0 and lc.wait in LANE_PENDING_WAITS),
                                 maneuver=sig_on, maneuver_id=d_id, distance=d_dist, desires=self.desires,
                                 missed=plan.turnMissed and not alt_sig, v_ego=v_ego, dt=DT_MDL, limit_ms=sm['navInstruction'].speedLimit,
                                 left_lamp=cs.leftBlinker or port.leftLamp, right_lamp=cs.rightBlinker or port.rightLamp,
                                 at_left_line=at_left_line, standing=decision.should_stop or at_left_line,
                                 stopping=short_of_turn, go_now=self.release_go,
                                 skipped=self.skip.skipped and self.skip.maneuver_id == rf.maneuver_id and not alt_sig,
                                 passed=rf.valid and rf.upcoming is None and not alt_sig)
    self.signal_out = ts
    ts.fill(plan.turnSignal)
    # a turn whose desire is out is skipped only while a stop brings the car to rest short of the junction, or it stands at
    # rest short of it, under TURN_STARTED_DEG into it (the intent's own rule, on the same short_of_turn): a right turn
    # pulses 40 m out, so every hold short of it, and every queue standing there, finds its desire out
    stop_veto = short_of_turn and not at_left_line and self.desires.turned_deg < TURN_STARTED_DEG
    if ts.skip != 'none' and alt_sig:
      # the other turn overruled before it starts (standing, or on its go while the decider can still withdraw it): the
      # decider reads the veto next frame (sg_veto) and never takes it, withdrawing a go out for it and handing back with the
      # car held (stop_go: H3, "turn cancelled"); its request ends now, never handed over, and a pulse of its go comes out of
      # the model's queue. Never a skip of the route's maneuver, which may be a turn further on
      self.sg_veto = ts.skip
      self.turn_signal.end(ALT_MANEUVER_ID)
      if self.desires.maneuver_id == ALT_MANEUVER_ID and self.desires.started:
        self.desires.cancel()
      cloudlog.event("turnpilot.turn_signal_alt_veto", error=True, veto=ts.skip, maneuver=str(sg.alt['maneuver']), state=sg.state)
    elif ts.skip != 'none' and act_on is not None and not self.skip.skipped:
      if not committed or stop_veto:
        # the driver overruled the car's signal: skipped out loud, navd reroutes. A desire out from the approach comes
        # out of the model's queue now (CLEAR_DESIRE) and the turn's cap stays until it is passed, as for a dropped turn
        # too close to pulse again: the hold's go is never the vetoed turn
        self.skip.skip(act_on, rf.distance, rf.maneuver_id, v_ego, ts.skip)
        plan.tooFastToTurn = True
        self.vetoed = VetoedTurn(act_on, self.odo_m + rf.distance, ts.skip, rf.maneuver_id, committed, veto_odo=self.odo_m)
        if sg_withdrawable:
          self.sg_veto = ts.skip  # the decider withdraws its go on it next frame (stop_go._going)
        if committed:
          self.desires.cancel()
          self.skip_cap_id = rf.maneuver_id
        if stopping:
          self.veto_hold_id = rf.maneuver_id
      else:
        cloudlog.event("turnpilot.turn_signal_veto_dropped", error=True, why=ts.skip, turned_deg=round(self.desires.turned_deg, 1))
    if ts.left_veto and desire_on is not None:
      # a veto at the left's line, or in its go while withdrawable, is the left's handback and ends the request for the left
      # (car_signals.py), never a skip (6.6's exception). It is the junction's too: carry_veto keeps that request ended for
      # the same left there under a new id, for as long as the car is short of it (carry_veto)
      self.vetoed = VetoedTurn(desire_on, self.odo_m + rf.distance, ts.left_veto, rf.maneuver_id, False, line=True,
                               veto_odo=self.odo_m)
    signal_skip = self.skip.skipped and self.skip.maneuver_id == rf.maneuver_id and \
      self.skip.reason in ('driverSignal', 'hazards', 'signalLost', 'sideUncertain')
    plan.turnSignal.skip = self.skip.reason if signal_skip else 'none'
    plan.skipReason = self.skip.reason if plan.tooFastToTurn and self.skip.skipped and self.skip.maneuver_id == rf.maneuver_id else ''
    if self.keep_rerouted_t > 0. and acting and not plan.tooFastToTurn:
      plan.skipReason = KEEP_REROUTED  # R7: navd recomputed past the fork the model drove: the skip's chime now (keep_unsent)
    self.keep_rerouted_t = max(self.keep_rerouted_t - DT_MDL, 0.)
    hazards_hold = ts.side != LaneChange.none and str(port.block) == 'hazard'
    plan.reason = decision.reason or (TURN_MISSED_REASON if plan.turnMissed else "") or lane_reason or mapd.reason or \
      (HAZARDS_REASON if hazards_hold else "") or (NO_JUNCTION_DATA_REASON if no_junction_data else "")
    fill_map_control(plan, mapd)
    self.fill_left(plan, decision)
    self.fill_stop_go(plan, sg, decision)
    if mapd.driven and str(plan.controlPrompt) == mapd.prompt:
      plan.reason = mapd.driven  # a line crossed with no go: said for what it is (map_controls.Driven)
    # the stop sweep at every engaged stop and yield, engaged or not, at walking pace only, the left's own view first
    # (junction_watch.py), the stop-sign decider's too while it holds or launches; the watch logs the control's whole window
    # whatever the speed. From the decider's commit point the crossing cycle (stopSweepGo) at any speed
    sweep = sweep_control(self.map_controls.tracker.controls.values())
    self.sweep_on = sweep_wanted(sweep if sweep is not None else (True if sg.detector_view == 'stopSweep' else None), v_ego, self.sweep_on)
    plan.detectorView = detector_view(str(plan.detectorView), self.sweep_on)
    if sg.detector_view == 'stopSweepGo' and str(plan.detectorView) not in ('road', 'roadAndWide'):
      plan.detectorView = 'stopSweepGo'
    self.sweeping = str(plan.detectorView) in ('stopSweep', 'stopSweepGo')
    self.watch_junction(sweep, v_ego)
    if acting:
      self.go_cert.fill(plan)
    if skip.skipped and self.skip.reason == 'unprotectedRange' and upcoming is not None:
      ul = plan.unprotectedLeft
      ul.state, ul.rangeM = 'skipped', float(self.left_range_m)
      ul.vLimit = float(upcoming.opposing_speed or DEFAULT_OPPOSING_SPEED)
      ul.rReq = float(self.route_reach_m or reach_at_line(upcoming.opposing_speed, upcoming.opposing_lanes or None, ROUTE_REACH_MARGIN_M))
      ul.farRangeM, ul.farNeeded = float(self.left_far_range_m if self.far_live(route=True) else 0.), ul.rReq > self.left_range_m
    if self.far_off_t > 0. and str(plan.controlAlert) == 'none':  # the far crop off for the drive: said once, a status line
      plan.controlAlert = 'farViewOff'
    self.far_off_t = max(self.far_off_t - DT_MDL, 0.)
    if self.side_off_t > 0. and str(plan.controlAlert) == 'none':  # the side views off for the drive: said once, a status line,
      plan.controlAlert = 'sideCamerasOff'                            # its time running only while it is shown
      self.side_off_t = max(self.side_off_t - DT_MDL, 0.)
    if self.cross_refused and str(plan.controlAlert) == 'none':  # the sim's cross traffic refused on a car that isn't the sim's
      plan.controlAlert = 'lightSourceRejected'
      plan.reason = plan.reason or CROSS_ORACLE_REFUSED_REASON
    if self.override_failed_t > 0. and str(plan.controlAlert) in ('none',) + CONTROL_STATUS_ALERTS:
      plan.controlAlert = 'overrideWriteFailed'  # a signal seen or a downgrade at a stop sign not saved for navd: chimed (O16)
    self.override_failed_t = max(self.override_failed_t - DT_MDL, 0.)
    if decision.hard_brake:  # a confirmed track's hard stop outranks the map's alerts (scene_policy.py)
      plan.controlAlert = 'hardBrake'
    ped_handed = self.crosswalk_standoff(sm, v_ego, acting, decision)
    stalled = self.stalled_in_turn(sm, cs, v_ego, acting, decision, mapd, objects=objects, raw=raw, available=available,
                                   took_over=took_over, right_hand_traffic=right_hand_traffic)
    stopped = self.stopped_unheld(sm, cs, v_ego, acting, decision, mapd, objects=objects, raw=raw)
    if self.always_drive_on and stopped and self.stopped_s >= UNSTICK_S - 1e-6 and not self.stopped_said and not stalled and \
       not self.turn_committed:  # a stand with a turn's desire out is the turn's (its stall, LEFT-21's person veto), never this
      self.unstick(sm, cs, v_ego, decision, objects, raw, available, took_over, right_hand_traffic)
    abandoned = self.turn_abandoned(sm, cs, v_ego, acting)
    if self.straight_cleared:  # R8b's fire: the turn's desire out of the model's queue from this very plan on
      plan.desire = self.last_desire = CLEAR_DESIRE
    wrong_side = self.wrong_side(sm, cs, v_ego, acting, paint, right_hand_traffic)
    person_front = self.scene.person_front
    off_route = self.off_route_turn(sm, cs, v_ego, acting, rf, upcoming, default_yaw_rate(cs, model))
    if self.person_front_handback(acting, person_front):  # WF-13: held by it PERSON_FRONT_MAX_S, nobody seen leaving
      plan.controlPrompt, plan.reason = 'personFrontTakeControl', PERSON_FRONT_REASON
    elif withdrawn or self.cant_start_unheld:  # the switch's has no hold: the model drives on under it (no_move)
      plan.controlPrompt, plan.reason = 'cantStartTakeControl', NO_MOVE_REASON
    elif ped_handed:  # the crosswalk hold's standoff (SC-02): held, take control
      plan.controlPrompt, plan.reason = 'pedestrianWaitingTakeControl', PED_WAITING_REASON
    elif abandoned:
      plan.controlPrompt, plan.reason = 'turnAbandonedTakeControl', TURN_ABANDONED_REASON
    elif wrong_side:
      plan.controlPrompt, plan.reason = 'wrongSideTakeControl', WRONG_SIDE_REASON
    elif off_route:
      plan.controlPrompt, plan.reason = 'offRouteTakeControl', OFF_ROUTE_TURN_REASON
    elif stalled:
      plan.controlPrompt, plan.reason = 'stalledTakeControl', STALLED_REASON
    elif stopped in (STOPPED_NOTE, LIGHT_WAIT_NOTE):
      plan.reason = plan.reason or stopped
      # PF-14: the note's info chime, once a stand (stopped_s starts over with it); never within STOPPED_CONTROL_M of a mapped
      # signal, where the model stands first in line at every red (ALWAYS_DRIVE.md ST-07 (stops)): the status line alone there
      if self.stopped_s < STOPPED_NOTE_S + ABORT_HOLD_S - 1e-6 and str(plan.controlAlert) in ('none',) + CONTROL_STATUS_ALERTS and \
         not self.stopped_signal:
        plan.controlAlert = 'carStoppedNote'
    elif stopped:
      plan.controlPrompt, plan.reason = 'carStoppedTakeControl', stopped
    if self.ped_hold_s >= PED_NOTE_S - 1e-6 and str(plan.controlPrompt) == 'none':
      plan.reason = PED_WAITING_NOTE  # the crosswalk hold standing the car past PED_NOTE_S (SC-02), its chime once
      if self.ped_alert_t > 0. and str(plan.controlAlert) in ('none',) + CONTROL_STATUS_ALERTS:
        plan.controlAlert = 'pedestrianWaiting'
    self.ped_alert_t = max(self.ped_alert_t - DT_MDL, 0.)
    if self.unstick_alert_t > 0. and str(plan.controlPrompt) == 'none' and str(plan.controlAlert) in ('none',) + CONTROL_STATUS_ALERTS:
      plan.controlAlert, plan.reason = 'unstickLift', UNSTICK_REASON  # section 4.5's lift: said with a chime
    self.unstick_alert_t = max(self.unstick_alert_t - DT_MDL, 0.)
    if self.green_alert_t > 0. and str(plan.controlPrompt) == 'none' and str(plan.controlAlert) in ('none',) + CONTROL_STATUS_ALERTS:
      plan.controlAlert, plan.reason = 'lightGreenNotMoving', LIGHT_GREEN_REASON  # the model not going at a camera green: said
    self.green_alert_t = max(self.green_alert_t - DT_MDL, 0.)
    if self.stall_alert_t > 0. and str(plan.controlPrompt) == 'none' and str(plan.controlAlert) in ('none',) + CONTROL_STATUS_ALERTS:
      plan.controlAlert, plan.reason = 'stallLift', STALL_LIFT_REASON  # a stall's lift (LEFT-21): said with a chime
    self.stall_alert_t = max(self.stall_alert_t - DT_MDL, 0.)
    if person_front is not None and str(plan.controlPrompt) not in TAKE_CONTROL_PROMPTS:
      plan.reason = PERSON_FRONT_NOTE  # WF-13: the status line's from the frame it is set, until it ends or hands back
    # the route's stop signs (route_notice.py, STOP_SIGNS.md 3.4): at its start, and 300 m before each it couldn't avoid; said
    # engaged or not (selfdrived's turnpilotRouteNotice), only where the slot is free (no handback, no alert but a status line,
    # no take-control prompt: hooks.plan_events would show those instead), and used up only on frames it is said, so held
    # back (not lost) behind another. In shadow nothing is shown by selfdrived, and the plan carries it as before
    # nor while the turn signal's own line is up (turnpilotCarSignal: the prompt to signal, a notice, "Signal Off"), a permanent
    # alert at the same priority, which the later one covers
    free = handback == Handback.none and str(plan.controlAlert) in ('none',) + CONTROL_STATUS_ALERTS and \
      str(plan.controlPrompt) not in TAKE_CONTROL_PROMPTS and plan.turnSignal.prompt == LaneChange.none and \
      str(plan.turnSignal.notice) == 'none' and plan.turnSignal.handedOver == LaneChange.none
    notice, notice_text = self.notices.update(sm['navInstruction'], sm.updated['navInstruction'], sm.valid['navInstruction'], v_ego,
                                              bool(sm['carControl'].enabled), free=free)
    if notice != 'none':
      plan.controlAlert, plan.routeNotice = notice, notice_text
    # an unprotected turn's hold line is at the nearest of navd's readings of its junction (route_follower.py): readings
    # that far apart, it may be well short of the line, said out loud rather than guessed (never over the infeasible stop)
    if isinstance(turn, tuple) and self.turn_committed and rf.junction_spread > UNCERTAIN_M and \
       plan.controlAlert in ('none',) + CONTROL_STATUS_ALERTS:
      plan.controlAlert = 'junctionUncertain'
      plan.reason = decision.reason or JUNCTION_UNCERTAIN_REASON

    plan.routeValid = rf.valid
    if rf.valid and rf.maneuver is not None:
      plan.maneuverDistance = rf.distance
      plan.maneuverType = rf.maneuver.type
      plan.maneuverModifier = rf.maneuver.modifier
      plan.maneuverAngle = rf.maneuver.angle_deg
      plan.maneuverGeometryValid = rf.maneuver.geometry
      plan.maneuverBend = rf.maneuver.bend

    if not acting:
      plan.state = State.unavailable
      plan.reason = HANDBACK_REASON.get(handback, WAITING_REASON)
    elif decision.holding or withdrawn:
      plan.state = State.holding
    elif not rf.valid:
      plan.state = State.noRoute
    elif rf.arrived:
      plan.state = State.arrived
    elif self.lanes.active:
      plan.state = State.laneChange
    elif self.desires.active:
      plan.state = State.turning
    elif maneuver is not None and maneuver.kind in (TURN, KEEP) and \
         rf.distance < max(APPROACH_M, lane_change_window(v_ego)[0]):
      plan.state = State.approaching
    else:
      plan.state = State.following
    return msg

  def carry_veto(self, upcoming, rf, v_ego: float) -> None:
    """The turn the driver overruled (a signal veto's skip, DESIGN 6.6) stays skipped at its junction whatever id the route
    gives it: navd's reroute before the junction (an off-route drift, the stop-avoidance re-run, an ETA drift) that turns the
    same way there again is a new maneuver id (route_follower: a new route, the turn not committed after the veto's cancel),
    and taken up fresh it would be prompted, signalled and taken, with our lamp put out by nothing that could veto it again.
    A keep is one too: the intent signals and vetoes a TURN or a KEEP (car_signals: signals_turn), and every 'slight'
    modifier is a keep (a city Y junction's slight right, a fork, an off ramp). The same junction: same_junction (type,
    modifier, bearings when both have them) and within SAME_TURN_M of where the veto put it on the odometer; skipped for the
    veto's reason, out loud (tooFastToTurn, navd's skip), its cap kept if the vetoed turn's was. Forgotten once the car is
    SAME_TURN_M past it. Before this frame's TurnSkip.update, which then keeps it.
    A veto at the unprotected left's line (v.line) is the left's handback there, never a skip (a skip lets go of the latch
    with the car in a left-turn position, 6.6's exception), and it ended the request for the left. Carried, that request
    stays ended under the new id (TurnSignalIntent.end: never armed, signalled, prompted or handed over), before this frame's
    intent update, so the car never relights the left the driver overruled, never prompts 'Signal Left' for it with our
    lamp put out, and a takeover never asks for it. It is for the left at the line the car stands at: carried only to a left
    within LINE_VETO_SAME_M of where the veto put it (the readings' spread, never the next left on), and forgotten
    once the car is past that junction (the driver took over and drove it): a left of a new id after it, which without
    junction data same_junction can't tell from it, would otherwise lose its lamp and prompt with nothing said. Past it
    takes both: beyond where the veto put the junction, and LINE_VETO_MOVED_M on from where the veto found the car. The
    junction's place is the follower's nearest reading, which only runs down, so one reading off nearer while the car
    stands at the line (a fix off at a standstill, under UNCERTAIN_M of spread) can put it at or behind the car; standing
    where it was vetoed, the car is short of the junction whatever that reading says."""
    v = self.vetoed
    if v is None:
      return
    past_line = v.line and self.odo_m > v.junction_odo and self.odo_m > v.veto_odo + LINE_VETO_MOVED_M
    if self.odo_m > v.junction_odo + SAME_TURN_M or past_line:
      self.vetoed = None
      return
    if upcoming is None or upcoming.kind not in (TURN, KEEP) or rf.maneuver_id == v.maneuver_id or \
       (self.skip.skipped and self.skip.maneuver_id == rf.maneuver_id):
      return
    window = LINE_VETO_SAME_M if v.line else SAME_TURN_M
    if not same_junction(upcoming, v.maneuver) or abs(self.odo_m + rf.distance - v.junction_odo) > window:
      return
    cloudlog.event("turnpilot.veto_carried", error=True, why=v.reason, handback=v.line, vetoed_id=v.maneuver_id,
                   maneuver_id=rf.maneuver_id, distance=round(rf.distance, 1), off_m=round(self.odo_m + rf.distance - v.junction_odo, 1))
    if v.line:
      self.turn_signal.end(rf.maneuver_id)
      v.maneuver_id = rf.maneuver_id
      return
    self.skip.skip(upcoming, rf.distance, rf.maneuver_id, v_ego, v.reason)
    if v.capped:
      self.skip_cap_id = rf.maneuver_id
    v.maneuver_id = rf.maneuver_id

  def line_veto_defers(self, rf) -> bool:
    """A veto's record at the unprotected left's line (VetoedTurn.line) is live, the car still short of that junction where
    the veto found it, and the route's maneuver now is another turn (navd's reroute to a left further on: never the vetoed
    one, which carry_veto keeps ended): its request waits, never armed, signalled or prompted, until the record is forgotten
    (carry_veto: the car past the vetoed junction). Lit at the vetoed left's line, our lamp would say the car turns there,
    where the driver just overruled it; ended instead, as carry_veto's line branch did within SAME_TURN_M (skeptic
    m-behaviour3 P1), the later left would never be signalled at all. Logged once per maneuver."""
    v = self.vetoed
    if v is None or not v.line or rf.maneuver_id == v.maneuver_id or rf.upcoming is None or rf.upcoming.kind not in (TURN, KEEP):
      return False
    if self.line_deferred_id != rf.maneuver_id:
      self.line_deferred_id = rf.maneuver_id
      cloudlog.event("turnpilot.veto_line_defers", vetoed_id=v.maneuver_id, maneuver_id=rf.maneuver_id, distance=round(rf.distance, 1),
                     off_m=round(self.odo_m + rf.distance - v.junction_odo, 1))
    return True

  def keep_unsent(self, keep, rf, v_ego: float, lat_active: bool, why: dict, model, plan) -> None:
    """A route keep TurnPilot is acting on (act_on) whose desire couldn't start (why: lane_planner.keep_blocker's reason that
    way) and still hasn't keep_last before its point is given up on out loud (BUILD_FINDINGS D2-keep, S23's slight right with no
    lane line seen): skipped (turn_skip: tooFastToTurn, skipReason its reason; selfdrived's skip chime and line, navd's reroute
    or its move on to the next step if the model took the route's branch anyway), logged once, never sent after (a late
    swerve), its cap kept to its pass (a car slowing for it never speeds up into it). Never one whose desire went out, nor with
    openpilot not steering (no desire goes then: the driver steers).
    Under TurnPilotAlwaysDrive (ALWAYS_DRIVE.md R7) the model drives the fork: skipped all the same, navd's skip mark and its
    cap as above, but said as a status line (skipReason KEEP_FOLLOW), never the chime and 'Rerouting' while the model may yet
    take the branch; those come only if navd recomputes past it (check_keep_followed)."""
    d = self.desires
    reason = why.get(keep.side, '') if keep is not None and keep.kind == KEEP else ''
    if not reason or not lat_active or self.skip.skipped or d.maneuver_id != rf.maneuver_id or d.started or d.done or \
       rf.distance >= keep_last(v_ego):
      return
    follow = self.always_drive_on
    self.skip.skip(keep, rf.distance, rf.maneuver_id, v_ego, KEEP_FOLLOW if follow else reason)
    self.skip_cap_id = rf.maneuver_id
    self.keep_follow_id = rf.maneuver_id if follow else None
    plan.tooFastToTurn = True
    cloudlog.event("turnpilot.keep_not_sent", error=True, side='left' if keep.side == LEFT else 'right', reason=reason, follow=follow,
                   probs=[round(float(p), 2) for p in model.laneLineProbs], type=keep.type, modifier=keep.modifier,
                   angle_deg=round(keep.angle_deg, 1), distance=round(rf.distance, 1), v=round(v_ego, 2))

  def check_keep_followed(self, rf) -> None:
    """R7: the keep the model was left to drive (keep_unsent, KEEP_FOLLOW) is over. navd moved on to the next step (the model
    took the route's branch, or navd's skipped_turn_passed found the car on it): nothing to say. navd dropped the route or
    sent a new one (RouteFollower.replaced: off it, rerouting): the skip's chime and 'Rerouting' now (KEEP_REROUTED, for
    MISSED_HOLD_S), never as a skip mark of the new route (tooFastToTurn) nor a turnMissed, which would end its turn's signal.
    Right after this frame's route update."""
    k = self.keep_follow_id
    if k is None or rf.maneuver_id == k:
      return
    self.keep_follow_id = None
    if rf.replaced:
      self.keep_rerouted_t = MISSED_HOLD_S
      cloudlog.event("turnpilot.keep_rerouted", error=True, maneuver_id=k)

  def check_missed(self, rf, v_ego: float, acting: bool) -> bool:
    """The turn a hold let go of at standstill is passed (the follower's pass, this update) having turned under
    MISSED_TURN_DEG: missed (module docstring). Read before this frame's DesireTiming update, which drops the turn. Whether
    it was said missed."""
    d = self.desires
    if not (rf.passed and self.go_id is not None and self.go_id == d.maneuver_id == rf.maneuver_id):
      return False
    self.go_id = None
    if acting and not d.done and d.turned_deg < MISSED_TURN_DEG:  # done before the pass: cleared at its angle, taken
      self.missed_t = MISSED_HOLD_S
      self.post_go = None  # missed straight on: not held to the profile's speed down the road
      cloudlog.event("turnpilot_turn_missed", error=True, turned_deg=round(d.turned_deg, 1), v_ego=round(v_ego, 2),
                     distance=round(rf.distance, 1))
      return True
    return False

  @property
  def straight_fired(self) -> bool:
    """R8b's fire cleared the route's turn now in hand (watch_straight): the follower holds its junction to the pass as for
    a committed turn, so R8b's bounds, and R8's handback if they break, keep the maneuver until then."""
    w = self.straight
    return w is not None and w.fired and w.id == self.follower.maneuver_id

  def watch_straight(self, curvature: float, yaw_rate: float, live: bool) -> None:
    """R8b's bounds (module constants TURN_MISSED_*) on every frame of a committed route turn from its hold line (the
    follower's distance under HOLD_LINE_M) to the follower's pass (check_straight): turned under MISSED_TURN_DEG toward it
    and under TURN_MISSED_AWAY_DEG away from it, and modelV2's desiredCurvature under TURN_MISSED_CURV either way. Turned as
    DesireTiming counts it while the desire is out, and by the car's own yaw once R8b's fire has cleared it. live: TurnPilot
    acting with openpilot engaged; a frame without it ends the watch (the driver's drive, or a handback). A turn cleared at
    its angle (taken) or dropped ends it. A bound broken after R8b's fire is R8's handback, latched (turn_abandoned)."""
    d, rf = self.desires, self.follower
    m, w = rf.upcoming, self.straight
    if w is not None and w.id != rf.maneuver_id:
      self.straight = w = None
    if w is None:
      if not (live and self.turn_committed and m.kind == TURN and rf.distance < HOLD_LINE_M):
        return
      self.straight = w = StraightDrive(rf.maneuver_id, m.side)
    if not w.ok:
      return
    if w.fired:
      w.deg += math.degrees(yaw_rate * DT_MDL) * w.side
    elif self.turn_committed:
      w.deg = d.turned_deg
    else:
      w.ok = False
      return
    if live and straight_inside(w.deg, curvature):
      return
    w.ok = False
    if w.fired and live:
      self.abandoned_id = rf.maneuver_id
      cloudlog.event("turnpilot.turn_abandoned", error=True, straight=True, turned_deg=round(w.deg, 1), curvature=round(curvature, 4),
                     distance=round(rf.distance, 1))

  def check_straight(self, rf, v_ego: float, acting: bool, said: bool = False) -> None:
    """R8b at the follower's pass (module constants TURN_MISSED_*): a route turn whose watch held both bounds on every
    frame, from its hold line with its desire still out and R8 never fired (unconditional: R9's detector catching more), or
    from R8b's fire under TurnPilotAlwaysDrive, was driven straight through: missed (turnMissed: "Turn Missed",
    "Rerouting", navd's reroute at once), its post-go profile dropped, and a desire still out comes out of the model's queue
    on this very frame (CLEAR_DESIRE: WF-11). said: check_missed said it already (a go's turn): cleared, not said again.
    Read before this frame's DesireTiming update, which drops the turn."""
    w = self.straight
    if w is None or not rf.passed or w.id != rf.maneuver_id:
      return
    self.straight = None
    d = self.desires
    out = d.maneuver_id == w.id and d.started and not d.done and d.side != 0
    if not (acting and w.ok and (w.fired or out)):
      return
    if out:
      d.cancel()
    if not said:
      self.missed_t = MISSED_HOLD_S
      self.post_go = None
      cloudlog.event("turnpilot_turn_missed", error=True, straight=True, fired=w.fired, turned_deg=round(w.deg, 1),
                     v_ego=round(v_ego, 2), distance=round(rf.distance, 1))

  def check_driven_left(self, rf, yaw_rate: float, v_ego: float, acting: bool) -> None:
    """An unprotected turn whose line was crossed with no go of TurnPilot's (map_controls.Driven: the driver's gas, or an
    overrun; owner decision (b), 2026-10-30) passed by the follower (not rerouted away from) having turned under MISSED_TURN_DEG
    toward it since: driven straight through, so missed (turnMissed: "Turn Missed", "Rerouting", navd's reroute at once),
    as a go missed straight on is. Last frame's crossing; yaw counted moving (YAW_MIN_SPEED)."""
    dr = self.map_controls.driven
    if dr is not None and dr.maneuver_id is not None and self.driven_checked != dr.key:
      self.driven_checked = dr.key
      side = rf.upcoming.side if rf.maneuver_id == dr.maneuver_id and rf.upcoming is not None else LEFT
      self.driven_left = [dr.maneuver_id, side, 0.]
    dl = self.driven_left
    if dl is None:
      return
    if v_ego >= YAW_MIN_SPEED:
      dl[2] += math.degrees(yaw_rate * DT_MDL) * dl[1]
    if rf.maneuver_id == dl[0] and not rf.passed:
      return
    # passed (the follower's pass moves it on to the next maneuver that update), or gone from the route without one (navd's
    # reroute already away from it: nothing to say)
    self.driven_left = None
    if acting and rf.passed and dl[2] < MISSED_TURN_DEG:
      self.missed_t = MISSED_HOLD_S
      self.post_go = None
      cloudlog.event("turnpilot_turn_missed", error=True, driven=True, turned_deg=round(dl[2], 1), v_ego=round(v_ego, 2),
                     distance=round(rf.distance, 1), passed=bool(rf.passed))

  def clear_driven_left(self, rf) -> None:
    """The autonomous left done as the driver's (unprotected_left: DRIVEN, its own hold or its line taken past with no
    certified go) with its go's pulse still out and the car under TURN_STARTED_DEG into it: the pulse comes out of the
    model's queue now (CLEAR_DESIRE, in this very plan) and nothing more goes for it, so the car goes straight on and the
    left is missed (check_driven_left: turnMissed, navd's reroute), never an uncertified left across the lanes its go was
    withdrawn for (owner decision (b); skeptic round 2: let go as the driver's, the go's turnLeft stayed out on the
    re-engage, the model steering the left). Turned further, the turn is under way and is finished, as a committed one is.
    Before this frame's desire (update())."""
    left, d = self.left_now, self.desires
    if left is None or left.state != 'undecided' or left.why != DRIVEN:
      return
    if d.maneuver_id == rf.maneuver_id and d.started and d.side != 0 and not d.done and d.turned_deg < TURN_STARTED_DEG:
      cloudlog.event("turnpilot.left_driven_desire_cleared", error=True, turned_deg=round(d.turned_deg, 1),
                     distance=round(rf.distance, 1), cause=left.crossed or 'line')
      d.cancel()

  def clear_withdrawn_left(self, rf) -> None:
    """The autonomous left's go withdrawn to its own hold this frame (unprotected_left: hold_here rising, for the band, the
    exit crosswalk, the view, another hold, the light, the position fault or the driver's signal) with its pulse out and the
    car under TURN_STARTED_DEG into it: the pulse comes out of the model's queue now (CLEAR_DESIRE, in this very plan) and
    none goes while the car is held (DesireTiming.withdraw), so the model plans no left while the hold brakes the car to its
    stand short of the oncoming lanes; the next go pulses a fresh edge (tp/left-hold's recheck, follow-up 2: the go's
    turnLeft stayed out on 40 of 44 and 44 of 48 frames the hold braked the car, dropping only once it stood). The turn stays
    the maneuver's, so a left the driver then takes from the hold is cleared and missed as before (clear_driven_left), or
    finished, turned TURN_STARTED_DEG or more. turnpilotd's own no-move withdrawal stands the car already: never here.
    Before this frame's desire (update())."""
    here = self.left.hold_here
    rising, self.left_hold_was = here and not self.left_hold_was, here
    d = self.desires
    if not rising or self.left_now is None:
      return
    if d.maneuver_id == rf.maneuver_id and d.started and d.side != 0 and not d.done and d.turned_deg < TURN_STARTED_DEG:
      cloudlog.event("turnpilot.left_withdrawn_desire_cleared", turned_deg=round(d.turned_deg, 1), distance=round(rf.distance, 1),
                     state=str(self.left_now.state), why=str(self.left_now.why))
      d.withdraw()

  def watch_far(self, sm) -> None:
    """The far crop's state from the full pass's latest farStatus: read, or off for the drive (said once, farViewOff,
    and logged). Its shadow heads are logged on every change of what they read (never acted on). The side views going off
    for the drive (sideViewStatus disabled, STOP_SIGNS.md row E9) is said the same way (sideCamerasOff)."""
    data = getattr(sm, 'data', {})
    self.far_status_age += DT_MDL
    if 'turnpilotDetections' in data and sm.updated['turnpilotDetections']:
      status = str(sm['turnpilotDetections'].farStatus)
      if status == 'disabled' and self.far_status != 'disabled':
        self.far_off_t = FAR_OFF_ALERT_S
        cloudlog.event("turnpilot.far_view_off", error=True, was=self.far_status)
      self.far_status, self.far_status_age = status, 0.
      side = str(sm['turnpilotDetections'].sideViewStatus)
      if side == 'disabled' and self.side_status != 'disabled':
        self.side_off_t = SIDE_OFF_ALERT_S
        cloudlog.event("turnpilot.side_cameras_off", error=True, was=self.side_status)
      self.side_status = side
    if 'turnpilotFarDetections' in data and sm.updated['turnpilotFarDetections']:
      run = sm['turnpilotFarDetections'].run
      heads = tuple(sorted((str(h.state), round(float(h.distance))) for h in run.lights.lights if h.governs))
      if heads != self.far_lights:
        cloudlog.event("turnpilot.far_light_shadow", heads=[list(h) for h in heads], status=str(run.lightStatus))
        self.far_lights = heads

  def far_live(self, route: bool = False) -> bool:
    """The far crop reads: the full pass's latest farStatus, fresh. route: on (reading, or waiting for room or calibration),
    not off nor off for the drive: what route time and the left take, the left judging its runs' own freshness (a spell
    without room at its line starts its window over and, past STALE_UNDECIDED_S, is its farStale fault, never farOff)."""
    return self.far_status in (FAR_ON if route else ('read',)) and self.far_status_age <= FAR_STATUS_STALE_S

  def left_reach_m(self, route: bool = False) -> float:
    """The range an unprotected left's R_req may reach: the full pass's proven range, or the far crop's while it reads (route:
    while it is on, far_live) and the full pass's is measured (the far crop covers the band only past it)."""
    far = self.far_live(route) and self.left_range_m > 0.
    return max(self.left_range_m, self.left_far_range_m) if far else self.left_range_m

  def unprotected_left(self, turn, upcoming, rf, v_ego: float, acting: bool, objects, raw, raw_far=None,
                       enabled: bool = True) -> LeftCommand | None:
    """This frame's autonomous unprotected left (unprotected_left.py; left_now) for the route's unprotected turn the car is
    taking (turn, map_controls' tuple), and its command for the hold line's latch. TurnPilotAutoLeft off, or TurnPilot not
    acting (handed back): off, the latch handed back as G1's. Fed the last frame's latch and scene (whether they held)."""
    mode = self.auto_left if acting else OFF
    if not isinstance(turn, tuple) or mode == OFF:
      self.left.update(LeftInput(OFF, rf.maneuver_id, 0, 0., v_ego, False, None, [], 0., None, None), DT_MDL)
      return None if not isinstance(turn, tuple) else LeftCommand(OFF)
    tc = self.map_controls.turn
    held = tc is not None and tc.held and tc.key[1] == int(turn[0])
    # the latch holds the car at another maneuver's line than the route's now (a reading far off then back read as a pass
    # and a new maneuver by the follower, which a turn whose desire isn't out doesn't hold): the car is held where its
    # junction isn't known. Said for as long as it lasts, never a silent hold (skeptic G4)
    stray = tc is not None and tc.held and not held
    # A reading that puts the junction at or behind the car held at its line (skeptic m-behaviour3 P2: 10 m nearer, the
    # follower at -0.75 m with the car 9.25 m short, the go putting out no turnLeft), or too near for the go's first-motion
    # re-check (skeptic SIMFIX P2), is the left's own position fault (unprotected_left.HELD_MIN_M), said, the car held
    scene = self.scene
    # DESIGN G4 go condition 5: never a go while another of TurnPilot's stops holds the car at or before the junction (a
    # signal's red, a stop sign), nor at a signal whose light isn't read green (unprotected_left.py: SIGNAL_NEEDS_GREEN)
    stop_d = self.map_controls.stop_ahead()
    # its line crossed with no go (map_controls.Driven: the latch let go as the driver's, last frame's): the left is done,
    # never a go nor a pulse of a left nobody checked (owner decision (b))
    driven = self.map_controls.driven
    inp = LeftInput(mode, rf.maneuver_id, upcoming.side, rf.distance, v_ego, held or stray, raw, objects, self.approach_yaw_deg,
                    upcoming.opposing_speed or None, upcoming.opposing_lanes or None, self.left_range_m,
                    scene_clear=scene.junction is None and not scene.carried and scene.person_front is None,
                    junction_uncertain=rf.junction_spread > UNCERTAIN_M or stray,
                    go_withdrawn=self.go_withdrawn, committed=rf.maneuver_id == scene.committed_id,
                    released=rf.maneuver_id == scene.released_id,
                    spread_m=rf.junction_spread, other_hold=stop_d is not None and stop_d <= rf.distance + PROTECT_PAST_M,
                    light=self.map_controls.turn_light(int(turn[0]), turn[1]), far=raw_far, far_range_m=self.left_far_range_m,
                    far_live=self.far_live(route=True),  # on: a spell without room is waited through on its runs' freshness
                    straight_m=float(upcoming.opposing_straight_m),
                    signal_lit=self.signal_out.signal_lit, signal_veto=self.signal_out.left_veto,
                    enabled=enabled, driven=driven is not None and driven.maneuver_id == rf.maneuver_id)
    d = self.left_now = self.left.update(inp, DT_MDL)
    return LeftCommand(mode, d.go, d.creep_m, d.state == 'undecided', d.reason, crossed=d.crossed)

  def observe_runs(self, tracker) -> None:
    """This tick's detector runs and tracks, for the junction watch (main(), after the tracker)."""
    try:
      last_recv, last_now = getattr(tracker, 'last_recv', None), getattr(tracker, 'last_now', None)
      recv_age = last_now - last_recv if last_recv is not None and last_now is not None else 0.
      self.watch.observe_runs(tracker.new_runs, bool(tracker.available), recv_age)
      self.tracks_now = list(tracker.tracks)
      self.runs_now = list(tracker.new_runs)
      if getattr(tracker, 'lens_spec', None):
        self.lenses['tracker'] = tracker.lens_spec
    except Exception:
      log_error("turnpilotd.watch_failed")
      self.tracks_now, self.runs_now = [], []

  def watch_junction(self, sweep, v_ego: float) -> None:
    """The swept control's evidence and visibility, logged (junction_watch.py); a failure there never touches the plan."""
    try:
      if sweep is None and self.watch.key is None:
        return
      wall = datetime.datetime.now(datetime.UTC).timestamp() if system_time_valid() else None  # as navd.wall_clock
      vis = visibility_table.cached() if sweep is not None else None
      surveys = survey_file.cached() if sweep is not None else None
      self.watch.update(sweep, v_ego, self.tracks_now, vis, surveys, wall)
    except Exception:
      log_error("turnpilotd.watch_failed")
      self.watch = JunctionWatch()

  def fill_left(self, plan, decision) -> None:
    """turnpilotPlan.detectorView and unprotectedLeft, and the left's own prompt and alert where the map's say nothing."""
    d = self.left_now
    if d is None:
      return
    plan.detectorView = d.view if d.view != 'none' else 'auto'
    if d.state != 'off':
      plan.farCropBearing = float(d.far_bearing)  # up the oncoming approach while the left is in hand
    ul = plan.unprotectedLeft
    ul.state, ul.tReq, ul.rReq, ul.vLimit, ul.rangeM = d.state, float(d.t_req), float(d.r_req), float(d.v_limit), float(self.left_range_m)
    ul.clearS, ul.blockers, ul.viewBlocked, ul.crosswalkClear = float(d.clear_s), min(d.blockers, 255), d.view_blocked, d.crosswalk_clear
    ul.creepM, ul.why = float(d.creep_m), d.why or d.verdict
    ul.farRangeM, ul.farNeeded, ul.farBlockers = float(d.far_range_m), d.far_needed, min(d.far_blockers, 255)
    if str(plan.controlPrompt) == 'none' and d.prompt != 'none':
      plan.controlPrompt = d.prompt
      plan.reason = decision.reason or d.reason
    if d.alert != 'none' and str(plan.controlAlert) in ('none',) + CONTROL_STATUS_ALERTS:
      plan.controlAlert = d.alert

  def track_yaw(self, maneuver, rf, yaw_rate: float, v_ego: float) -> None:
    """approach_yaw_deg: how far the car has turned toward the route's turn since the turn's desire first went out for it,
    for the junction rules' road frame (scene_policy.road_frame): a turn pulsed on the approach that left the car standing
    at its hold line 30-45 deg in, or the autonomous left's from its go (its first pulse). Before the desire the model
    follows its lane, so the car stands aligned with it and the yaw read meanwhile is the road's bend: never counted (the
    band would turn off the oncoming lane). Counted only moving (YAW_MIN_SPEED). Last frame's desire state. A turn that still
    pulses on its approach (TURN_DESIRE_LEAD_M: TurnPilotAutoLeft off or announce, a left a stop sign protects) still counts
    a bend between its pulse and its line: nothing here knows the road's own heading (the map's would)."""
    d = self.desires
    pulsed = d.maneuver_id == rf.maneuver_id and d.started
    if maneuver is None or maneuver.kind != TURN or rf.maneuver_id != self._yaw_id or not pulsed:
      self._yaw_id, self.approach_yaw_deg = rf.maneuver_id, 0.
      return
    if v_ego >= YAW_MIN_SPEED:
      self.approach_yaw_deg += math.degrees(yaw_rate * DT_MDL) * maneuver.side

  def track_yaw20(self, yaw_rate: float, v_ego: float) -> None:
    """The car's heading change over its last YAW20_M (the stop-sign decider's go condition 3, the yaw off its approach and
    ANGLE_BOUND's measured offset), counted moving only; and how far it has turned since the decider's go."""
    dyaw = math.degrees(yaw_rate * DT_MDL)
    if v_ego >= YAW_MIN_SPEED:
      self.yaw20.append((self.odo_m, dyaw))
      while self.yaw20 and self.yaw20[0][0] < self.odo_m - YAW20_M:
        self.yaw20.pop(0)
    going = self.sg_now is not None and self.sg_now.state in ('going', 'committed', 'inLane')
    self.sg_turned = self.sg_turned + dyaw if going else 0.
    self.sg_heading += dyaw

  def stop_go_update(self, sm, cs, model, v_ego: float, acting: bool, objects, rf) -> StopGoDecision:
    """This frame's stop-sign decision (stop_go.py) for the held stop sign (map_controls.held_control()) or the engaged one
    within STOP_SWEEP_M ahead. Not acting (a handback): as shadow. A failure is logged and is shadow too (the latch's 1 s
    handback), its go withdrawn: never a go on an exception. But with the ladder's other turn under way (going, committed,
    inLane: the latch let go, the turn's desire out) the last frame's decision is carried on, dead-reckoned, for up to
    STOP_GO_CARRY_FRAMES, said in the log (turnpilot.stop_go_carried, error) and in stopGo.fault: a bare decision there would
    drop the turn's desire and the scene's hold on its maneuver mid-turn (skeptic SG4c A), and a committed turn is never
    cancelled."""
    try:
      mc = self.map_controls
      ctl = mc.held_control()
      ctl = ctl if is_stop(ctl) else None  # a stop sign, or a yield that became one (yieldStop: STOP_SIGNS.md 2.7)
      ahead = None
      if ctl is None:
        cands = [c for c in mc.tracker.controls.values() if is_stop(c) and c.engaged and not c.held and
                 not (c.state is not None and c.state.done) and -LATCH_PAST_M <= c.d <= STOP_SWEEP_M]
        ahead = min(cands, key=lambda c: c.d) if cands else None
      cur = ctl or ahead
      # what the encounter is about: the decider's own control while it keeps its encounter. Once the latch has let it go
      # (the 'clear'), neither held_control() nor a control ahead is it (the tracker may have dropped it as passed), and
      # another control ahead is the next one's: its junction, maneuver, route certification and fix spread are still its
      # own, for the launch's re-checks, the band and every retry (a right's blind spot and GO_ROOM_RIGHT_M, a left's T_GAP)
      sg = self.stop_go
      own = sg.control if sg.token is not None and not sg.takes(ctl, ahead) else None
      src = own if own is not None and (cur is None or cur.state is not sg.token) else cur
      info = (src.junction_info or {}) if src is not None else {}
      junction = junction_from_info('stop', src.junction_info) if src is not None else None
      m = str(info.get('maneuver', 'straight'))
      maneuver = 'unknown' if src is not None and src.key[0] == OFF_ROUTE else m if m in ('straight', 'left', 'right') else 'unknown'
      self.sg_route_maneuver = maneuver
      skip_union = False
      if maneuver in ('left', 'right') and ((self.skip.skipped and self.skip.maneuver_id == rf.maneuver_id) or
                                            (sg.token is not None and sg.token is self.sg_skip_token)):
        # the route's turn here skipped (a veto of the turn signal at the stop, too fast, a lane it can't reach; last frame's
        # skip): no turn desire goes out and the hold's go is straight on, so the decider judges every maneuver's conflicts
        # (unknown, as off the route), never the skipped turn's alone, for the rest of this encounter (navd moving on to the
        # next maneuver as the car leaves the junction ends the skip, never the launch's union). A reroute that goes straight
        # here is judged straight
        maneuver, self.sg_skip_token, skip_union = 'unknown', sg.token, True
      vis = self.vis_source()
      live, tracks, runs, objs, side_status = self.watch.live, tuple(self.tracks_now), tuple(self.runs_now), objects, self.side_status
      oracle = self.cross_oracle(sm)
      lenses = dict(self.lenses)
      if oracle is not None:
        # the sim's cross-traffic oracle (8.2): ground truth for the decider in the cameras' place, under the gate only (no
        # camera's lens then decides anything, so none is checked against the file's)
        live, tracks, runs, objs, side_status = oracle
        lenses = {}
      lead = model.leadsV3[0] if len(model.leadsV3) else None
      lead_m = float(lead.x[0]) if lead is not None and lead.prob > STOPPED_LEAD_PROB and len(lead.x) else None
      d = self.desires
      alt_id = ALT_MANEUVER_ID if sg.alt is not None else rf.maneuver_id
      model_lanes = model_lanes_from_msg(model)
      up = rf.upcoming
      to_node = (float(src.d) + float(info.get('stopPointDistance', 0.) or 0.)) if src is not None else None
      ours = len(up.lanes) if up is not None and up.lanes and to_node is not None and abs(rf.distance - to_node) <= ALT_LANES_M else 0
      f = Frame(control=ctl, approach=ahead, junction=junction, route_cert=str(info.get('certification', 'unchecked')), maneuver=maneuver,
                objects=objs, tracks=tracks, runs=runs, live=live, vis=vis,
                surveys=self.survey_source(), wall_t=self.decider_clock(), side_status=side_status, lenses=lenses,
                position_spread=src.fix_spread if src is not None else 0., v_ego=v_ego, a_ego=float(cs.aEgo), odo_m=self.odo_m,
                yaw20_deg=sum(y for _, y in self.yaw20), approach_yaw_deg=self.approach_yaw_deg, turned_deg=self.sg_turned,
                heading_deg=self.sg_heading,
                steer_deg=float(cs.steeringAngleDeg), desire_out=d.maneuver_id == alt_id and d.started and d.side != 0 and not d.done,
                model_v2=model, lead_m=lead_m, t_abort=sr.t_abort(self.cp_delay, vis.actuator_delay_s if vis is not None else None),
                enabled=bool(sm['carControl'].enabled), right_blindspot=bool(cs.rightBlindspot), tp_holding=self.last_holding,
                go_active=self.go_cert.cause, go_taken_over=self.go_cert.taken_over, go_end=self.go_cert.end,
                mode=self.stop_sign_mode if acting else 'shadow', mode_known=self.stop_sign_known, approach_lanes=ours,
                rightmost=model_lanes.measured and model_lanes.right_known and model_lanes.right == 0, signal_veto=self.sg_veto,
                skip_union=skip_union, always_drive=self.always_drive_on, person_front=self.scene.person_front is not None)
      self.sg_veto = ''  # read once: the go it vetoed is withdrawn on it
      d = self.stop_go.update(f)
      self.sg_last, self.sg_carried = d, 0
      return d
    except Exception:
      log_error("turnpilotd.stop_go_failed")
      prev = self.sg_last
      if prev is not None and prev.alt is not None and prev.state in STOP_GO_CARRY_STATES and self.sg_carried < STOP_GO_CARRY_FRAMES:
        self.sg_carried += 1
        carried = dataclasses.replace(prev, release=None, stops=[], creep=None, go_cause='none', withdraw='error', chime=False,
                                      turn_missed=False, position_m=prev.position_m + max(v_ego, 0.) * DT_MDL, fault='error',
                                      log={})
        cloudlog.event("turnpilot.stop_go_carried", error=True, frames=self.sg_carried, state=prev.state,
                       alt=str(prev.alt.get('maneuver', '')), position_m=round(carried.position_m, 2))
        self.sg_last = carried
        return carried
      self.sg_last, self.sg_carried = None, 0
      self.sg_route_maneuver = None
      return StopGoDecision(latch='shadow', withdraw='error')

  def set_sim(self, ok: bool) -> None:
    """The sim's files for the stop-sign decider (STOP_SIGNS.md 8.2), only under visibility_table.sim_accepted (SIMULATION=1,
    TurnPilotSimLightOracle and a sim car in CarParams): the sim visibility file (camera fields, a passing launchVerified, all
    synthetic) and the sim survey of the scenario junctions (maps/sim/), and the sim's own sun (SIM_SUN_UTC_H on the day).
    Off it the shipped files and the wall clock; the sim's are never read."""
    if ok == self.sim_files:
      return
    self.sim_files = ok
    cloudlog.event("turnpilot.sim_stop_files", on=ok, visibility=str(SIM_VISIBILITY), surveys=str(SIM_SURVEYS))
    if ok:
      self.vis_source = lambda: visibility_table.cached(SIM_VISIBILITY, sim_ok=True)
      self.survey_source = lambda: survey_file.cached(SIM_SURVEYS)
    else:
      self.vis_source, self.survey_source = visibility_table.cached, survey_file.cached

  def decider_clock(self) -> float | None:
    """The stop-sign decider's wall clock (its light check): the sim's own sun under the sim's files, SIM_SUN_UTC_H of today
    (BeamNG renders its own daylight, not Indiana's), else the wall clock while valid."""
    t = self.wall_clock()
    if not self.sim_files or t is None:
      return t
    day = datetime.datetime.fromtimestamp(t, datetime.UTC).replace(hour=SIM_SUN_UTC_H, minute=0, second=0, microsecond=0)
    return day.timestamp()

  def cross_oracle(self, sm):
    """The sim's cross-traffic oracle for the decider (STOP_SIGNS.md 8.2), or None: turnpilotLights from the simOracle with a
    crossRangeM, fresh (LIGHT_STALE_S), and only under the sim's whole gate: the light oracle's (oracle_ok: SIMULATION=1 and
    TurnPilotSimLightOracle) and a sim car in CarParams (sim_files, set_sim's: visibility_table.sim_accepted). Off the light
    oracle's gate map_controls refuses the message out loud (lightSourceRejected) and nothing here reads it; under it without a
    sim car (or before CarParams is in) its cross traffic is refused, logged and said (cross_refused: lightSourceRejected with
    CROSS_ORACLE_REFUSED_REASON), and the decider reads the cameras. (OracleLive, its tracks as the tracker's, the runs of every
    view, its road users as the scene's objects, 'running')."""
    was, self.cross_refused = self.cross_refused, False
    data = getattr(sm, 'data', {})
    if 'turnpilotLights' not in data:
      return None
    self.oracle_age = 0. if sm.updated['turnpilotLights'] else getattr(self, 'oracle_age', float('inf')) + DT_MDL
    msg = sm['turnpilotLights']
    if not self.oracle_ok or self.oracle_age > LIGHT_STALE_S or str(msg.source) != 'simOracle' or float(msg.crossRangeM) <= 0.:
      return None
    if not self.sim_files:
      self.cross_refused = True
      if not was:
        cloudlog.event("turnpilot.cross_oracle_refused", error=True, reason="not a sim car", cars=len(msg.cross),
                       range_m=round(float(msg.crossRangeM), 1))
      return None
    self.oracle_t = getattr(self, 'oracle_t', 0.) + DT_MDL
    t = self.oracle_t
    tracks, objs, hits = [], [], {}
    for c in msg.cross:
      x, y, vx, vy, cls = float(c.x), float(c.y), float(c.vx), float(c.vy), str(c.cls)
      hits[int(c.id)] = getattr(self, 'oracle_hits', {}).get(int(c.id), CONFIRM_STEADY_HITS - 1) + 1
      r = max(math.hypot(x, y), 0.5)
      b = math.degrees(math.atan2(y, x))
      half = math.degrees(math.atan2(0.9, r))
      view = 'full' if abs(b) < 40. else 'sideLeft' if b > 0. else 'sideRight'
      tracks.append(SimpleNamespace(track_id=ORACLE_ID + int(c.id), t_seen=t, bearing_seen=b, width_seen=1.8 * 904. / r, view=view,
                                    cls=ORACLE_CLASS.get(cls, 2), x=np.array([x, y, vx, vy]), span_seen=(b - half, b + half), hits=5))
      # ground truth, so a confirmed track's hits for the decider's in-path rail (SAT-11): scene_policy.confirmed_track's at
      # first sight, one more each frame it is published, so its evidence stays steady (TrackEvidence's CONFIRM_RECENT_HITS)
      objs.append(SimpleNamespace(trackId=ORACLE_ID + int(c.id), cls=cls, x=x, y=y, vx=vx, vy=vy, age=hits[int(c.id)], conf=1.,
                                  missedRuns=0))
    self.oracle_hits = hits  # a car gone from the message and back starts afresh
    runs = tuple((v, t, RawFrame([], 'road' if v == 'road' else 'wide', t, v if v.startswith('side') else 'full'), None)
                 for v in ('road', 'full', 'sideLeft', 'sideRight'))
    return OracleLive(float(msg.crossRangeM)), tuple(tracks), runs, objs, 'running'

  def signal_seen_override(self, j) -> None:
    """A signal head at a stop-sign junction (stop_go: 2.4 item 9, C20): TurnPilotJunctionOverrides names it signalSeen so
    navd's certifier (check 8, any approach overridden) routes around it from the next lookup on; the junction may have been
    converted to a signal since it was surveyed. The bus carries an approach's way but not its direction, so both directions of
    every listed approach's way are written (only the real ones ever match navd's keys). Not written (no params, a param this
    build doesn't know, a value that doesn't read, the write failing): logged and said (overrideWriteFailed), never silently
    lost (row O16)."""
    from openpilot.selfdrive.turnpilot.routing.junctions import approach_key
    ways = sorted({int(a.way_id) for a in (j.approaches if j is not None else []) if a.way_id})
    node = int(j.osm_node_id) if j is not None else 0
    keys = [approach_key(node, w, fwd) for w in ways for fwd in (True, False)]
    evidence = {'source': 'stopGo signal head', 'wallTime': self.wall_clock(), 'ways': ways, 'written': keys}  # one event (route_audit)
    entries = [{'approach': k, 'kind': 'signalSeen', 'evidence': evidence} for k in keys]
    self.write_overrides('signalSeen', node, entries if node else [], "the junction's node or ways unknown")

  def downgrade_override(self, j, approach, evidence: dict) -> None:
    """An approach downgraded on site (stop_go: 4.4, decision D11, row V3: a motor vehicle through its line at speed):
    TurnPilotJunctionOverrides names it no with the decider's evidence, so navd's certifier (check 8, any approach
    overridden) routes around the junction from the next lookup on, until Kellen has looked at the evidence and cleared it
    (route_audit --list-overrides, --clear). The bus carries the approach's way but not its direction, so both directions of
    that way at this node are written, each naming both (written): only the real one matches navd's key, and the certifier
    fails a junction on any overridden approach either way. Not written: logged and said (overrideWriteFailed), never
    silently lost (row O16); the decider's own downgrade for this encounter doesn't wait on it."""
    from openpilot.selfdrive.turnpilot.routing.junctions import approach_key
    node = int(j.osm_node_id) if j is not None else 0
    way = int(getattr(approach, 'way_id', 0) or 0)
    keys = [approach_key(node, way, fwd) for fwd in (True, False)] if node and way else []
    ev = dict(evidence, source='stopGo downgrade', wallTime=self.wall_clock(), written=keys)
    self.write_overrides('no', node, [{'approach': k, 'kind': 'no', 'evidence': ev} for k in keys],
                         "the junction's node or the approach's way unknown")

  def write_overrides(self, kind: str, node: int, entries: list, missing: str) -> None:
    """Add entries to TurnPilotJunctionOverrides (routing/junctions.write_override); a write that fails, or nothing to name
    (missing says why), is logged and said (overrideWriteFailed, with a chime), never silently lost."""
    from openpilot.selfdrive.turnpilot.routing.junctions import write_override
    ok, why = write_override(self.params, entries) if entries else (False, missing)
    if ok:
      cloudlog.event("turnpilot.override_written", kind=kind, osmNodeId=node, approaches=[e['approach'] for e in entries])
      return
    log_error(f"turnpilotd: {kind} at junction {node} not saved to TurnPilotJunctionOverrides: {why}")
    cloudlog.event("turnpilot.override_write_failed", error=True, kind=kind, osmNodeId=node, why=why,
                   approaches=[e['approach'] for e in entries])
    self.override_failed_t = OVERRIDE_FAILED_ALERT_S

  def certify_stop_go(self, sm, cs, available: bool, objects, raw, decision, took_over: bool, sg: StopGoDecision) -> None:
    """The stop-sign decider's go (sg.go_cause: stopSign, rollUp, clearBox) certified for plannerd's bounded lift (go_override.py)
    under TurnPilotGoOverride: the decider's own certification replaces turnRelease's light rule, go_blocked's (the driver, the
    scene, a hold, a lead, the path) still apply. A go of its own still out (a roll-up's) ends first: one go at a time, each
    its own id and window. Uncertified (the param off, a blocker), the decider's go stands on the model alone; its watchdog
    withdraws it if the model never takes over."""
    own = float(sg.creep[1]) if sg.go_cause == 'creep' and sg.creep is not None else None
    why = 'param' if not self.go_override else self.go_blocked(sm, cs, available, objects, raw, decision, took_over, own)
    if why:
      cloudlog.event("turnpilot.go_not_certified", cause=sg.go_cause, why=why, blocker=path_blocker(objects, raw) if why == 'inPath' else '',
                     distance=round(-sg.position_m, 1))
      return
    if self.go_cert.active and self.go_cert.cause in STOP_GO_CAUSES:
      self.go_cert.withdraw('replaced')
    if self.go_cert.start(sg.go_cause, position=round(sg.position_m, 2), state=sg.state, retries=sg.retries):
      self.go_lifted, self.go_cert_id, self.go_own_cap = True, self.go_cert.id, own

  def fill_stop_go(self, plan, sg: StopGoDecision, decision) -> None:
    """turnpilotPlan.stopGo, and the decider's prompt, reason and alert where the latch's (map_controls) say nothing: its wait
    (waitingForCross), its go (stopGo), and once the latch has let go its holds and faults (stopSignTakeControl)."""
    g = plan.stopGo
    g.state, g.reason, g.ledger, g.wouldGo, g.maneuver = sg.state, sg.reason, sg.ledger, sg.would_go, sg.maneuver
    lanes = g.init('lanes', len(sg.lanes))
    for out, ln in zip(lanes, sg.lanes, strict=True):
      out.role, out.index, out.approach, out.rSeen, out.rReq = ln.role, min(ln.index, 255), min(ln.approach, 255), float(ln.r_seen), float(ln.r_req)
      out.binding, out.shielded = ln.binding, ln.shielded
    g.arrivalRank, g.retries = min(sg.rank, 255), min(sg.retries, 255)
    if sg.edges is not None:
      g.dEdgeMin, g.dEdgeMax = float(sg.edges.d_min), float(sg.edges.d_max)
    g.committed, g.inLane, g.creptM = sg.committed, sg.in_lane, float(sg.crept_m)
    g.clearS, g.goT, g.takeoverS, g.withdrawn = float(sg.clear_s), float(sg.go_t), float(sg.takeover_s), sg.withdrawn
    g.positionM, g.chime, g.fault = float(sg.position_m), sg.chime, sg.fault
    g.alternate = sg.alt is not None
    if sg.state == 'off':
      return
    if str(plan.controlPrompt) == 'none' and sg.prompt != 'none':
      plan.controlPrompt = sg.prompt
      plan.reason = sg.reason if sg.prompt in ('stopGo', 'waitingForCross') or not decision.reason else decision.reason
    elif sg.state == 'approach' and str(sg.ledger) == 'static':
      # standing short of a line it never latched, nothing else of TurnPilot's holding the car, waiting on its roll-up
      # (its bound's wait, stop_go._short_wait): said over the map control's approach text
      plan.reason = sg.reason
    if sg.alert != 'none' and (str(plan.controlAlert) in ('none',) + CONTROL_STATUS_ALERTS or sg.alert == 'carCrossing'):
      plan.controlAlert = sg.alert  # the in-path rail's "Car crossing" over any map alert (a hard brake for it outranks it below)
    if sg.state in ('going', 'committed', 'inLane') or (sg.state in ('observe', 'ladder') and self.map_controls.held_control() is None):
      mc = plan.mapControl
      mc.kind, mc.distance, mc.action = 'stop', float(-sg.position_m), sg.action if sg.action != 'none' else 'held'

  def go_at_line(self, distance: float) -> bool:
    """A go from the car standing at or short of the route's junction's hold line, distance (the follower's) GO_LINE_M or
    more before it, the scene neither committed to that junction nor released into it by the driver: only such a go is
    withdrawn (no_move). Otherwise the car is in the junction, where stopping is worse than finishing the turn
    (scene_policy.py: committed_past_line); a stop line mapped nearer than GO_LINE_M (a sim sidecar's) is taken so too.
    The autonomous left's go is always from its line (it goes only standing held there), whatever a reading says."""
    if self.left_now is not None and self.left_now.go:
      return True
    mid = self.follower.maneuver_id
    return distance >= GO_LINE_M and mid not in (self.scene.committed_id, self.scene.released_id)

  def start_post_go(self, maneuver, distance: float, turned_deg: float, right_hand_traffic: bool = True) -> None:
    """The go of a turn from a standstill: its post-go profile from now (module docstring), the unprotected turn's own for a
    turn across the oncoming road, and its no-move clock if it goes from the hold line (go_at_line)."""
    left = unprotected_turn(maneuver, right_hand_traffic)
    name = self.left_go_profile if left else self.post_go_profile
    self.post_go = PostGo(POST_GO_PROFILES[name], maneuver, distance, turned_deg, right_hand_traffic)
    at_line = self.go_at_line(distance)
    self.no_move_limit, self.go_withdrawn = (LEFT_NO_MOVE_S if left else self.no_move_s), False
    self.go_wait_s = 0. if at_line else None  # in the junction: never withdrawn, the stall's handback covers it
    self.go_lead_s, self.go_left, self.rel_gap_s, self.go_straight = 0., left, 0., False
    self.go_moved, self.go_clock_again = False, False
    # the left's go from its line followed into the oncoming lanes only by a launch (left_launch)
    self.left_launch_armed = left and at_line and distance - ONCOMING_ENTRY_M >= LEFT_LIFT_ROOM_M
    cloudlog.event("turnpilot.post_go", profile=name, unprotected=left, turned_deg=round(turned_deg, 1), distance=round(distance, 1),
                   at_line=at_line)

  def go_blocked(self, sm, cs, available: bool, objects, raw, decision, took_over: bool, own_cap: float | None = None) -> str:
    """Why TurnPilot's go can't be certified for plannerd's lift now, '' if it can (go_override.py): the driver on a pedal or
    disengaged, no live scene to see the path by, a hold of TurnPilot's in force (a walker's, a map stop's), a lead within
    GO_CLEAR_M, or a road user in the path within it (path_blocker). own_cap: a creep go's own cap (the stop-sign decider's
    deadlock creep, map_controls' creep path): that cap is what bounds the creep's move, so only shouldStop or a cap of
    something else's below it is a hold."""
    if not sm['carControl'].enabled or took_over or cs.gasPressed or cs.brakePressed:
      return 'driver'
    if self.scene.person_front is not None:
      return 'personFront'  # a person TurnPilot saw may be in front (scene_policy.person_front): no go of TurnPilot's at all
    if not available:
      return 'scene'
    if decision.should_stop or (decision.holding and (own_cap is None or decision.speed_cap < own_cap - OWN_CAP_TOL)):
      return 'hold'
    if close_lead(sm['modelV2']):
      return 'lead'
    return 'inPath' if path_blocker(objects, raw) else ''

  def certify_go(self, sm, cs, available: bool, objects, raw, decision, took_over: bool, distance: float, maneuver=None,
                 right_hand_traffic: bool = True) -> None:
    """TurnPilot's go from a standstill (start_post_go's, at the hold line or in the junction: finishing a turn beats standing
    in it) certified for plannerd's bounded lift (turnpilotPlan.goCertified), cause unprotectedLeft for the autonomous left's,
    turnRelease for any other turn released from TurnPilot's hold at a junction whose light is read green (release_blocked);
    the left's too where a signal is, and neither under a red or yellow any source reads there (release_blocked); or,
    TurnPilotGoOverride off, go_blocked or its light, not, logged with why. Never turnRelease for a turn across the
    oncoming road (unprotected_turn: its go is the left's, on its gap), nor for a far-side turn released in the junction
    (go_in_junction: its lanes crossed or to cross, which nothing the lift checks sees; the stall's handback covers that
    stand). Uncertified, the go is as before G5: the model pulls away or the no-move handback comes. maneuver: the route's
    turn."""
    self.go_lifted = False
    cause = 'unprotectedLeft' if self.left_now is not None and self.left_now.go else 'turnRelease'
    why = self.go_refused(sm, cs, available, objects, raw, decision, took_over, distance, cause, maneuver, right_hand_traffic)
    if why:
      cloudlog.event("turnpilot.go_not_certified", cause=cause, why=why, blocker=path_blocker(objects, raw) if why == 'inPath' else '',
                     distance=round(distance, 1), light=self.map_controls.release_light(distance))
      return
    self.go_lifted = self.go_cert.start(cause, distance=round(distance, 1), at_line=self.go_wait_s is not None,
                                        turned_deg=round(self.desires.turned_deg, 1))
    self.go_cert_id = self.go_cert.id if self.go_lifted else None
    if self.go_lifted and cause == 'turnRelease':
      self.rel_goes += 1

  def go_refused(self, sm, cs, available: bool, objects, raw, decision, took_over: bool, distance: float, cause: str, maneuver=None,
                 right_hand_traffic: bool = True) -> str:
    """Why TurnPilot's go of cause (certify_go's, a relift's) can't be certified now, '' if it can (certify_go). Under the
    switch a turnRelease also needs the release's goes and lifted metres left (RELEASE_GOES, RELEASE_LIFT_M)."""
    why = 'param' if not self.go_override else self.go_blocked(sm, cs, available, objects, raw, decision, took_over)
    if not why and cause == 'unprotectedLeft' and distance - ONCOMING_ENTRY_M < LEFT_LIFT_ROOM_M:
      why = 'room'  # nearer the oncoming lanes than the lift's nudge and its stop: never lifted into them (LEFT_LIFT_ROOM_M)
    if not why and cause == 'turnRelease' and unprotected_turn(maneuver, right_hand_traffic):
      why = 'unprotected'  # skeptic G5: the oncoming lanes' certification is the left's gap, which a release never has
    if not why and cause == 'turnRelease' and self.go_in_junction and far_side_turn(maneuver, right_hand_traffic):
      why = 'junction'  # skeptic G5: a committed left lifted into the oncoming lane as an oncoming car closed
    if not why and cause == 'turnRelease' and self.always_drive_on and \
       (self.rel_goes >= RELEASE_GOES or self.rel_lift_m > RELEASE_LIFT_M - RELIFT_MIN_M):
      why = 'budget'
    if not why:
      why = self.release_blocked(distance, cause)
    return why

  def release_distance(self, distance: float) -> float:
    """m to the junction a go of TurnPilot's is judged at (release_blocked): the route's, distance; a straight release's
    (go_straight), the nearest map control ahead within STOPPED_CONTROL_M, where it stands, or the car's place without one."""
    if not self.go_straight:
      return distance
    ds = [c.d for c in self.map_controls.tracker.controls.values() if -LATCH_PAST_M <= c.d < STOPPED_CONTROL_M]
    return min(ds) if ds else 0.

  def watch_release(self, sm, cs, v_ego: float, model) -> None:
    """Under the switch, the release's bounds (RELEASE_LIFT_M; ALWAYS_DRIVE.md ST-13): its lifted metres by odometer while
    one of its goes is out, the go out withdrawn ('budget') once they are spent; the release over (its goes and metres back to
    none) once the car moves on the model's takeover predicate, or the driver takes over."""
    self.rel_takeover_held = self.rel_takeover.update(model, DT_MDL)
    if not self.always_drive_on:
      return
    if (self.rel_takeover_held and v_ego >= STANDSTILL_SPEED) or not sm['carControl'].enabled or cs.gasPressed:
      self.rel_lift_m, self.rel_goes, self.unstick_done = 0., 0, False
    gc = self.go_cert
    if gc.active and ((gc.cause == 'turnRelease' and gc.id == self.go_cert_id) or gc.id in (self.stall_lift_id, self.unstick_id)):
      self.rel_lift_m += v_ego * DT_MDL
      if self.rel_lift_m >= RELEASE_LIFT_M - 1e-6:
        cloudlog.event("turnpilot.release_budget", go_id=gc.id, lifted_m=round(self.rel_lift_m, 2), goes=self.rel_goes)
        gc.withdraw('budget')

  def relift(self, sm, cs, v_ego: float, acting: bool, available: bool, objects, raw, decision, took_over: bool, distance: float,
             maneuver=None, right_hand_traffic: bool = True) -> None:
    """Under the switch, a release's later go (ALWAYS_DRIVE.md ST-04, ST-11, ST-06's follow-up): with its no-move clock
    running (a go from the line, not the left's), no go out, the car standing (under STANDSTILL_SPEED) RELIFT_GAP_S after the
    release's last go ended, its goes and lifted metres left and nothing refusing it (go_refused), turnRelease again under a
    new id, a relift (GoCertifier: its speed end isn't followed)."""
    if not (self.always_drive_on and acting and self.go_wait_s is not None and not self.go_left) or self.go_clock_again:
      return
    gc = self.go_cert
    if gc.active:
      self.rel_gap_s = 0.
      return
    self.rel_gap_s += DT_MDL
    if v_ego >= STANDSTILL_SPEED or self.rel_gap_s < RELIFT_GAP_S - 1e-6:
      return
    if self.go_refused(sm, cs, available, objects, raw, decision, took_over, distance, 'turnRelease', maneuver, right_hand_traffic):
      return
    if gc.start('turnRelease', relift=True, distance=round(distance, 1), at_line=True, turned_deg=round(self.desires.turned_deg, 1),
                lifted_m=round(self.rel_lift_m, 2), goes=self.rel_goes):
      self.go_lifted, self.go_cert_id = True, gc.id
      self.rel_goes += 1

  def release_blocked(self, distance: float, cause: str = 'turnRelease') -> str:
    """Why a go of cause can't be certified at the junction distance m ahead, or stay so, '' if it can: its light
    (map_controls.release_light) a red or yellow from any source, or at a signal not green from an accepted source (the camera's
    shadow green never lifts the model's hold: the light net may only add a stop, skeptic G5). A turnRelease also needs a
    signal there unless RELEASE_UNSIGNALIZED (module constants); the autonomous left's (unprotectedLeft) goes where none is,
    on its gap."""
    light = self.map_controls.release_light(distance)
    if light in ('red', 'yellow'):
      return 'light'
    if cause == 'unprotectedLeft':
      # its own go read its junction (turn_light: no signal there, or a green from some source): at a signal the lift needs
      # the green from an accepted source; with none there it goes on its gap
      return '' if light == 'green' or not self.map_controls.signal_known(self.follower.maneuver_id, distance) else 'light'
    if light == 'none' and not RELEASE_UNSIGNALIZED and self.always_drive_on and \
       not far_side_turn(None if self.go_straight else self.follower.upcoming, self.rht):
      why = self.unsignalized_why(distance)
      if why != self.unsig_why:
        cloudlog.event("turnpilot.release_unsignalized", ok=not why, why=why, distance=round(distance, 1),
                       sign_clear_m=round(self.sign_clear_m, 1))
        self.unsig_why = why
      return 'light' if why else ''
    return '' if light == 'green' or (light == 'none' and RELEASE_UNSIGNALIZED) else 'light'

  def unsignalized_why(self, distance: float) -> str:
    """Why release_light's 'none' at the junction distance m ahead can't stand for no signal there under the switch, '' if
    it can (ALWAYS_DRIVE.md LEFT-20 and section 4.5's conditions): a map control within STOPPED_CONTROL_M of the car or the
    junction, the camera's shadow reader not live or reading a head of any colour, or a stop-sign box seen in a forward run
    in the last SIGN_CLEAR_M driven (watch_signs; or the camera's signs not read)."""
    mc = self.map_controls
    controls = [c for c in list(mc.tracker.controls.values()) + [mc.turn] if c is not None]
    if any(abs(c.d) < STOPPED_CONTROL_M or abs(c.d - distance) < STOPPED_CONTROL_M for c in controls):
      return 'control'
    camera = mc._lights.camera  # the camera's reader as the last map update had it (None: off, stale or not read)
    if camera is None or camera.status != 'read' or camera.heads:
      return 'camera'
    if not self.camera_signs or self.sign_clear_m < SIGN_CLEAR_M - 1e-6:
      return 'stopSign'
    return ''

  def watch_signs(self, raw, v_ego: float) -> None:
    """unsignalized_why's stop-sign memory, every frame (module constants, SIGN_CLEAR_M): the metres driven since a forward run
    (raw, and this tick's runs from the tracker, of either camera's full pass) had a stop-sign box, 0 on a box or on a frame
    driven at STANDSTILL_SPEED or more with no forward run for SIGN_RUN_FRESH_S; a frame standing adds nothing."""
    forward = [r for r in [raw] + [run for key, _t, run, _s in self.runs_now if key in ('road', 'full')]
               if r is not None and r.view == 'full']
    self.forward_age = 0. if forward else self.forward_age + DT_MDL
    if any(getattr(r, 'signs', ()) for r in forward):
      self.sign_clear_m = 0.
    elif self.forward_age <= SIGN_RUN_FRESH_S + 1e-6:
      self.sign_clear_m += v_ego * DT_MDL
    elif v_ego >= STANDSTILL_SPEED:
      self.sign_clear_m = 0.

  def withdraw_go(self, sm, cs, acting: bool, available: bool, objects, raw, decision, took_over: bool, distance: float) -> None:
    """The certified go out withdrawn this frame, plannerd's lift with it, once anything it was certified on stops holding
    (go_blocked; its light, release_blocked, for a turnRelease and the left's), TurnPilot stops acting (a handback), or the autonomous left
    withdraws its own go (left out of its go). distance: m to the route's junction."""
    gc = self.go_cert
    if not gc.active:
      return
    own = self.go_own_cap if gc.cause == 'creep' else None
    why = 'handback' if not acting else self.go_blocked(sm, cs, available, objects, raw, decision, took_over, own)
    if not why and gc.cause == 'unprotectedLeft' and (self.left_now is None or self.left_now.state not in LEFT_GO_STATES):
      why = 'left'
    if not why and gc.cause in ('turnRelease', 'unprotectedLeft'):
      why = self.release_blocked(distance, gc.cause)
    if not why and gc.id == self.stall_lift_id:
      why = self.stall_veto(sm['modelV2'], objects)
      if why == 'person':
        cloudlog.event("turnpilot.stall_lift_person", go_id=gc.id, person=self.person_near_turn(objects, sm['modelV2']))
    if not why and gc.id == self.unstick_id:
      why = self.unstick_veto()
    if why:
      if why == 'inPath':
        cloudlog.event("turnpilot.go_path_blocked", go_id=gc.id, blocker=path_blocker(objects, raw))
      gc.withdraw(why)

  def no_move(self, sm, cs, v_ego: float, acting: bool, decision) -> bool:
    """TurnPilot's go from the hold line (go_at_line) is withdrawn: the model holding the car (module docstring). An
    uncertified go is followed by the car's first motion (STANDSTILL_SPEED), and withdrawn standing no_move_limit before it,
    nothing else holding the car. A certified one (go_lifted: plannerd's lift rolls the car on whether the model goes or not)
    is followed only once the model takes over (GoCertifier.taken_over: the takeover predicate, or GO_END_SPEED) and the car
    has moved: a takeover the car doesn't move on is still the no-move withdrawal. Until then it
    is withdrawn at once when its window ends with the model not having taken over (rolling: stopped at STOP_DECEL), or once
    the car stands no_move_limit, first or again, nothing else holding it (the lift over early: a phantom in the path, a lead
    come and gone). Rolled into the junction on the lift (the scene committed to it or released into it, or STALL_MIN_DEG
    turned), it is never held there: the stall's handback without a hold covers it (go_in_junction). Latched until openpilot
    is disengaged (the driver taking over); read before this frame's go starts a new clock (start_post_go). A go in the
    junction starts none: it is never withdrawn."""
    enabled = sm['carControl'].enabled
    moving = v_ego >= STANDSTILL_SPEED
    if self.cant_start_unheld:  # the switch's no-hold handback: until the car moves on the model's takeover, or the driver
      self.cant_start_unheld = enabled and not (moving and self.rel_takeover_held)
    if self.go_withdrawn:
      self.go_withdrawn = enabled
      return self.go_withdrawn
    if self.go_wait_s is None:
      return False
    unheld = self.always_drive_on and not self.go_left  # a release (RELEASE_LIFT_M): never held, its stand bound NO_MOVE_MAX_S
    gc = self.go_cert
    mine = gc.id == self.go_cert_id and not gc.active  # the go certified for this turn is over
    followed = (mine and gc.taken_over and (self.go_moved or moving)) if self.go_lifted else moving
    if unheld:
      followed = followed or (moving and self.rel_takeover_held)
    if followed or not enabled:  # it pulled away (the model took over), or the driver took over
      self.go_wait_s = None
      return False
    window_out = self.go_lifted and mine and gc.end == 'window' and not unheld  # its lift's 3 s out, the model never taking over
    mid, turned = self.follower.maneuver_id, self.desires.turned_deg
    if unheld and self.go_moved and (mid in (self.scene.committed_id, self.scene.released_id) or turned >= STALL_MIN_DEG):
      cloudlog.event("turnpilot.go_not_followed", error=True, in_junction=True, window_out=False, v=round(v_ego, 2),
                     turned_deg=round(turned, 1))
      self.go_in_junction, self.go_wait_s = True, None  # lifted into the junction: the stall's (stalled_in_turn), no hold
      return False
    if moving and unheld:
      self.go_moved = True  # rolling on a lift: the stand's clock runs from TurnPilot's go, its lifts no reason to start it over
    elif moving:
      self.go_moved, self.go_wait_s, self.go_lead_s = True, 0., 0.  # rolling on the lift: the stand's clock starts over
    elif acting and not decision.should_stop and not decision.holding and not cs.gasPressed and not cs.brakePressed:
      # a lead the live scene corroborates holds the car, NO_MOVE_MAX_S at most; one it doesn't holds nothing (watch_lead:
      # BUILD_FINDINGS D1, the phantom at a green that ended the go)
      if self.lead_held:
        self.go_lead_s += DT_MDL
      else:
        self.go_wait_s += DT_MDL
    limit = NO_MOVE_MAX_S if unheld else self.no_move_limit
    if not window_out and self.go_wait_s < limit - 1e-6 and self.go_wait_s + self.go_lead_s < NO_MOVE_MAX_S - 1e-6:
      return False
    if unheld:
      cloudlog.event("turnpilot.go_not_moving", error=True, stood_s=round(self.go_wait_s, 1), limit_s=limit, turned_deg=round(turned, 1),
                     lifted=self.go_lifted, moved=self.go_moved, unheld=True, goes=self.rel_goes, lifted_m=round(self.rel_lift_m, 2),
                     v=round(v_ego, 2), lead_s=round(self.go_lead_s, 1), **self.lead_log(sm['modelV2']))
      self.cant_start_unheld, self.go_wait_s = True, None  # loud, no hold: the model may still drive off under it
      gc.withdraw('noMove')
      return False
    if self.go_moved and (mid in (self.scene.committed_id, self.scene.released_id) or turned >= STALL_MIN_DEG):
      cloudlog.event("turnpilot.go_not_followed", error=True, in_junction=True, window_out=window_out, v=round(v_ego, 2),
                     turned_deg=round(turned, 1))
      self.go_in_junction, self.go_wait_s = True, None  # never held in the junction: the stall's handback, no hold
      return False
    cloudlog.event("turnpilot.go_not_moving", error=True, stood_s=round(self.go_wait_s, 1), limit_s=self.no_move_limit,
                   turned_deg=round(turned, 1), lifted=self.go_lifted, moved=self.go_moved, window_out=window_out, v=round(v_ego, 2),
                   lead_s=round(self.go_lead_s, 1), **self.lead_log(sm['modelV2']))
    self.go_withdrawn, self.go_wait_s = True, None
    return True

  def left_launch(self, sm, rf, v_ego: float) -> bool:
    """The unprotected left's go from its line (LEFT_LIFT_ROOM_M or more out) withdrawn (handed back, held: no_move's
    withdrawal) when the car rolls toward the oncoming lanes short of LEFT_LAUNCH_SPEED with only LEFT_LAUNCH_ROOM_M left to
    stop short of them (ONCOMING_ENTRY_M, at STOP_DECEL): its clear time was certified for its own launch, not a coast on the
    lift's speed (module constants). Checked from its go until the left commits or withdraws its go; never once committed (a
    committed turn is never cancelled)."""
    if not self.left_launch_armed or self.go_withdrawn:
      return False
    left = self.left_now
    if not sm['carControl'].enabled or left is None or left.state != 'go':  # the driver, its commit, or its own withdrawal
      self.left_launch_armed = False
      return False
    room = rf.distance - ONCOMING_ENTRY_M - v_ego ** 2 / (2. * STOP_DECEL)
    # room under 0: a reading nearer than the car is (the follower), which the left commits on (unprotected_left: past), never
    # a stop in the lanes the reading puts it in
    if v_ego < STANDSTILL_SPEED or not 0. <= room < LEFT_LAUNCH_ROOM_M or v_ego >= LEFT_LAUNCH_SPEED:
      return False
    cloudlog.event("turnpilot.left_not_launched", error=True, v=round(v_ego, 2), distance=round(rf.distance, 1), room=round(room, 2),
                   lifted=self.go_lifted, taken_over=self.go_cert.taken_over)
    self.left_launch_armed, self.go_withdrawn, self.go_wait_s = False, True, None
    return True

  def stalled_in_turn(self, sm, cs, v_ego: float, acting: bool, decision, mapd, objects=(), raw=None, available: bool = False,
                      took_over: bool = False, right_hand_traffic: bool = True) -> bool:
    """The car has stood STALL_PROMPT_S in a route turn in progress (its desire out, not done: turn_committed, which stays
    set while DesireTiming stops pulsing a standing car), with nothing holding it: hand back (module docstring), whoever
    stopped the car (BUILD_FINDINGS D2: the model stopping partway round with no hold, or after a hold that only capped the
    speed, stood silently for 60-103 s). STALL_MIN_DEG to STALL_MAX_DEG into the turn; or, after TurnPilot's go for it, from
    its start if the go came in the junction (go_in_junction: never withdrawn, never silent) or the car stands in it now (not
    go_at_line: a go from the line it crept on from into the cross street's lanes, skeptic G3, 89.95 s silent at 5.5 deg).
    After TurnPilot's go its desire may be done already (DesireTiming clears a turn at max(angle - CLEAR_MARGIN_DEG,
    CLEAR_MIN_FRACTION * angle) and turned_deg stops there): a stand after the go is the stall's done or not, as before D2
    (skeptic SIMFIX D2: a 60 deg turn stood at 56 and 70 deg, a 45 deg one at 39 and 60, were silent for 90 s), its
    STALL_MAX_DEG bound on the angle the desire cleared at, as it was.
    Under STALL_MIN_DEG with no go of TurnPilot's the car may stand at the line (a red the map doesn't know): stopped_unheld's.
    Not past the maneuver point (the follower's distance under 0), where a red's line never is: a stand there, the turn's
    desire out, is in the junction from its start (BUILD_FINDINGS D2 fix (1); skeptic SIMFIX: 10-15 deg into a right 8-10 m
    past its point, a status note at 5 s and carStoppedTakeControl at 60 s).
    A route turn R8b's fire cleared under TurnPilotAlwaysDrive (straight_fired, to the follower's pass) is the stall's as a
    committed one is, though its desire is done: the model stood in the junction box after choosing straight, where with the
    switch off R8's handback has it at once (ad-s3 skeptic: past the point, the note at 5 s and the handback only at 60 s).
    turned there is the angle the fire froze, under MISSED_TURN_DEG while straight_fired holds, so it counts past the point.
    The stop line it left is close behind it, so stopped_unheld would wait NEAR_CONTROL_STAND_S. A model lead holds it only
    as far as the scene corroborates it, LEAD_STAND_MAX_S at most (watch_lead)."""
    d, rf = self.desires, self.follower
    turned = d.turned_deg
    went = self.go_id is not None and self.go_id == rf.maneuver_id
    # TurnPilot's go for this turn (49359b2's gate): its desire out or already cleared at its angle
    go_turn = went and self.go_id == d.maneuver_id and \
      (self.go_in_junction or not self.go_at_line(rf.distance) or STALL_MIN_DEG <= turned)
    # the model's own stand in a turn in progress (or one R8b's fire cleared): partway round, or past the maneuver point
    model_turn = (self.turn_committed or self.straight_fired) and (STALL_MIN_DEG <= turned or rf.distance < 0.)
    in_turn = turned <= STALL_MAX_DEG and (go_turn or model_turn)
    standing = acting and in_turn and sm['carControl'].enabled and v_ego < STANDSTILL_SPEED and not decision.should_stop and \
      not self.go_withdrawn and not self.release_stand and \
      not decision.holding and not cs.gasPressed and not cs.brakePressed and not map_handback(mapd)
    self.stall_stand_s = self.stall_stand_s + DT_MDL if standing else 0.
    # a lead the scene corroborates keeps the stand LEAD_STAND_MAX_S; under the switch while its queue has moved within
    # QUEUE_STILL_MAX_S (ALWAYS_DRIVE.md ST-09 (stalls))
    lead_bound = (self.lead_still_s >= QUEUE_STILL_MAX_S - 1e-6) if self.always_drive_on else self.stall_stand_s >= LEAD_STAND_MAX_S - 1e-6
    lead_keeps = self.lead_held and not lead_bound
    self.stall_s = self.stall_s + DT_MDL if standing and not lead_keeps else 0.
    now = standing and (self.stall_s > STALL_PROMPT_S or (self.lead_held and lead_bound) or
                        (not self.always_drive_on and self.stall_stand_s >= LEAD_STAND_MAX_S - 1e-6))
    if self.always_drive_on:
      now = self.finish_turn(now, standing and not lead_keeps, sm, cs, v_ego, decision, objects, raw, available, took_over,
                             right_hand_traffic)
    if now and not self.stall_said:
      cloudlog.event("turnpilot.stalled_in_turn", error=True, stood_s=round(self.stall_stand_s, 1), turned_deg=round(d.turned_deg, 1),
                     go=went, distance=round(rf.distance, 1), **self.lead_log(sm['modelV2']))
    self.stall_said = now
    return now

  def finish_turn(self, now: bool, counting: bool, sm, cs, v_ego: float, decision, objects, raw, available: bool, took_over: bool,
                  right_hand_traffic: bool) -> bool:
    """Under the switch, a stall in a curb-side route turn (ALWAYS_DRIVE.md LEFT-21; module constants): its fresh pulse at
    STALL_REPULSE_S, and from STALL_PROMPT_S its clearBox lifts while one may go (stall_refused), the handback (now) waiting on
    them; once none may, now as it was. counting: the stall clock runs this frame. A turn R8b's fire cleared (straight_fired:
    the model drove it straight, its desire done) has no turn to finish: its stand is S3's stall handback at STALL_PROMPT_S."""
    rf, gc = self.follower, self.go_cert
    mid = rf.maneuver_id
    if self.stall_mid != mid or not sm['carControl'].enabled:  # another turn, or the driver: a stall of its own
      self.stall_mid, self.stall_pulsed, self.stall_lifts, self.stall_gap_s, self.stall_why = mid, False, 0, math.inf, ''
      self.stall_handed = False
    if v_ego >= STANDSTILL_SPEED or cs.gasPressed or cs.brakePressed:
      self.stall_handed = False  # the car moved or the driver acted: the stand it handed back is over
    mine = gc.active and gc.id == self.stall_lift_id
    if not mine and self.stall_lift_id is not None:
      self.stall_gap_s += DT_MDL
    m = rf.upcoming
    if m is None or m.kind != TURN or far_side_turn(m, right_hand_traffic) or self.decider_turn_id == mid or self.straight_fired:
      # a far-side turn's stall is the handback's alone (ST-10); a stop-sign turn's, the decider's (its clearBox); a turn
      # R8b's fire cleared, S3's stall handback (no turn to finish)
      return now
    if counting and self.stall_s >= STALL_REPULSE_S - 1e-6 and not self.stall_pulsed:
      self.stall_pulsed = True
      if self.desires.stall_pulse():
        cloudlog.event("turnpilot.stall_pulse", turned_deg=round(self.desires.turned_deg, 1), distance=round(rf.distance, 1))
    if mine:
      return False  # finishing the turn on its lift: nothing to hand back
    if not now:
      return False
    if self.stall_handed:
      return True  # handed back (SAT_RERUN R4/R5: a model lead read one frame and not the next): no lift into it after
    why = self.stall_refused(sm, cs, v_ego, decision, objects, raw, available, took_over)
    if why:
      if why != self.stall_why:
        cloudlog.event("turnpilot.stall_lift_refused", why=why, lifts=self.stall_lifts, lifted_m=round(self.rel_lift_m, 2),
                       turned_deg=round(self.desires.turned_deg, 1),
                       person=self.person_near_turn(objects, sm['modelV2']) if why == 'person' else '')
      self.stall_why = why
      self.stall_handed = self.stall_handed or why in ('lead', 'person', 'personFront', 'inPath')  # something seen: never lifted into after
      return True
    if self.stall_gap_s < STALL_LIFT_GAP_S - 1e-6 or gc.active:
      return False  # the next lift is due (or another go is out, for a frame or two): no handback meanwhile
    if gc.start('clearBox', relift=True, stall=True, distance=round(rf.distance, 1), turned_deg=round(self.desires.turned_deg, 1),
                lifts=self.stall_lifts, lifted_m=round(self.rel_lift_m, 2)):
      self.stall_lift_id, self.stall_lifts, self.stall_gap_s, self.stall_why = gc.id, self.stall_lifts + 1, 0., ''
      self.stall_alert_t = ABORT_HOLD_S
      return False
    return True

  def stall_refused(self, sm, cs, v_ego: float, decision, objects, raw, available: bool, took_over: bool) -> str:
    """Why the stall's clearBox lift may not go now, '' if it may (finish_turn): TurnPilotGoOverride off, go_blocked (the
    driver, the scene, a hold, a model lead within GO_CLEAR_M, a road user in the path), its lifts or the release's metres
    spent, not standing, or stall_veto."""
    if not self.go_override:
      return 'param'
    why = self.go_blocked(sm, cs, available, objects, raw, decision, took_over)
    if why:
      return why
    if self.stall_lifts >= STALL_LIFTS:
      return 'lifts'
    if self.rel_lift_m > RELEASE_LIFT_M - RELIFT_MIN_M:
      return 'budget'
    if v_ego >= STANDSTILL_SPEED:
      return 'moving'
    return self.stall_veto(sm['modelV2'], objects)

  def stall_veto(self, model, objects) -> str:
    """What bars a stall's lift, or withdraws one out, '' if nothing: a red or yellow read at the junction (release_light), or a
    person near the turn (person_near_turn)."""
    if self.map_controls.release_light(self.follower.distance) in ('red', 'yellow'):
      return 'light'
    return 'person' if self.person_near_turn(objects, model) else ''

  def unstick(self, sm, cs, v_ego: float, decision, objects, raw, available: bool, took_over: bool, right_hand_traffic: bool) -> None:
    """Section 4.5's lift under the switch (module constants): once per release, a model stop off any hold standing UNSTICK_S,
    nothing refusing it."""
    gc = self.go_cert
    if self.unstick_done or gc.active or not self.go_override or v_ego >= STANDSTILL_SPEED:
      return
    m, rf = self.follower.upcoming, self.follower
    far_turn = m is not None and far_side_turn(m, right_hand_traffic) and rf.distance < STOPPED_CONTROL_M
    if self.go_blocked(sm, cs, available, objects, raw, decision, took_over) or far_turn or \
       (self.left_now is not None and self.left_now.state != 'off') or self.unstick_veto() or \
       self.rel_lift_m > RELEASE_LIFT_M - RELIFT_MIN_M:
      return
    if gc.start('unstick', relift=True, stood_s=round(self.stopped_s, 1), lifted_m=round(self.rel_lift_m, 2)):
      self.unstick_done, self.unstick_id, self.unstick_alert_t = True, gc.id, ABORT_HOLD_S

  def unstick_veto(self) -> str:
    """What bars section 4.5's lift, or withdraws one out, '' if nothing: the car's place not read as no signal with navd's
    controls ok (release_light 'none'), or unsignalized_why."""
    light = self.map_controls.release_light(0.)
    return f"light {light}" if light != 'none' else self.unsignalized_why(0.)

  def crosswalk_standoff(self, sm, v_ego: float, acting: bool, decision) -> bool:
    """The right-turn crosswalk hold's standoff (ALWAYS_DRIVE.md SC-02, unconditional; module constants): its clocks, the note's
    chime at PED_NOTE_S, and the handback once every walker it is for has stood STAND_UNDECIDED_S, held while it stands the car."""
    j = self.scene.junction
    held = acting and sm['carControl'].enabled and decision.should_stop and v_ego < STANDSTILL_SPEED and j is not None and \
      j.reason == CROSSWALK_REASON
    if not held:
      self.ped_hold_s = self.ped_stand_s = 0.
      self.ped_said = self.ped_handed = False
      return False
    self.ped_hold_s += DT_MDL
    self.ped_stand_s = self.ped_stand_s + DT_MDL if self.scene.holders_standing else 0.
    if self.ped_hold_s >= PED_NOTE_S - 1e-6 and not self.ped_said:
      self.ped_said, self.ped_alert_t = True, ABORT_HOLD_S
      cloudlog.event("turnpilot.crosswalk_wait", held_s=round(self.ped_hold_s, 1), standing=bool(self.scene.holders_standing))
    if self.ped_stand_s >= STAND_UNDECIDED_S - 1e-6 and not self.ped_handed:
      self.ped_handed = True
      cloudlog.event("turnpilot.crosswalk_standoff", error=True, held_s=round(self.ped_hold_s, 1), **(j.evidence or {}))
    return self.ped_handed

  def person_front_handback(self, acting: bool, pf) -> bool:
    """The person-may-be-in-front latch (scene_policy.person_front, WARGAME_FIXES WF-13) set PERSON_FRONT_MAX_S: handed back loudly
    (personFrontTakeControl, PERSON_FRONT_REASON), the car still held by it, until the driver or her clearance ends it; logged once."""
    on = acting and pf is not None and self.scene.t - pf.t >= PERSON_FRONT_MAX_S - 1e-6
    if on and not self.person_handed:
      cloudlog.event("turnpilot.person_front_handback", error=True, held_s=round(self.scene.t - pf.t, 1), seen_leaving=pf.left,
                     **pf.evidence)
    self.person_handed = on
    return on

  def people_in_front(self, model) -> bool:
    """A person box of each camera's last detector run (watch_people) in front of the car: up to PERSON_FRONT_X_M ahead, its bottom
    across PATH_HALF_WIDTH_M of straight ahead or of the model's path, PERSON_FRONT_HALF_M ahead of the bumper (scene_policy.in_front's
    zone); an unranged one can't be placed, so it is. The person-may-be-in-front latch's clearance waits while one is
    (ScenePolicy.update's raw_people)."""
    px, py = predicted_path(model)
    for _, boxes in self.people_runs.values():
      for b in boxes:
        if b.xy is None:
          return True
        x = float(b.xy[0])
        if not 0. < x <= PERSON_FRONT_X_M:
          continue
        rng = math.hypot(*b.xy)
        lats = sorted(rng * math.sin(math.radians(e)) for e in b.span_deg)
        half = PERSON_FRONT_HALF_M if x > PERSON_FRONT_BUMPER_M else PATH_HALF_WIDTH_M
        if any(lats[0] < c + half and lats[-1] > c - half for c in (0., float(np.interp(x, px, py)))):
          return True
    return False

  def watch_people(self, *runs) -> None:
    """The person boxes of each camera's last detector run (PEOPLE), PERSON_RUN_HOLD_S at most (person_near_turn)."""
    for key in list(self.people_runs):
      self.people_runs[key][0] += DT_MDL
      if self.people_runs[key][0] > PERSON_RUN_HOLD_S + 1e-6:
        del self.people_runs[key]
    for raw in runs:
      if raw is not None:
        self.people_runs[(raw.camera, raw.view)] = [0., [b for b in raw.boxes if str(b.cls) in PEOPLE]]

  def person_near_turn(self, objects, model) -> str:
    """A person near the route's turn the car stands in, '' if none (ALWAYS_DRIVE.md LEFT-21's person veto): any person
    track (any hits) or person box of each camera's last detector run in the turn side's crosswalk zone (scene_policy's,
    CROSSWALK_*) taken in the road's frame (the car's yaw since the turn's first pulse, approach_yaw_deg), or within
    STALL_PERSON_PATH_M of the model's predicted path out to CURB_PED_X_M (the path curb_ped reads); an unranged person box
    can't be placed, so it is near."""
    m, distance = self.follower.upcoming, self.follower.distance
    side = m.side if m is not None and m.side else RIGHT
    a = math.radians(self.approach_yaw_deg) * side
    c, s = math.cos(a), math.sin(a)
    px, py = predicted_path(model)

    def near(x: float, y: float) -> bool:
      xr, yr = x * c - y * s, x * s + y * c
      zone = distance - CROSSWALK_X_BEFORE_M < xr < distance + CROSSWALK_X_AFTER_M and CROSSWALK_Y_NEAR_M < yr * side < CROSSWALK_Y_FAR_M
      return zone or (0. < x <= CURB_PED_X_M and abs(y - float(np.interp(x, px, py))) < STALL_PERSON_PATH_M)
    for obj in objects:
      if str(obj.cls) in PEOPLE and near(float(obj.x), float(obj.y)):
        return f"track {obj.trackId} {obj.cls} at {obj.x:.1f}, {obj.y:.1f} m"
    for _, boxes in self.people_runs.values():
      for b in boxes:
        if b.xy is None:
          return f"{b.camera} box {b.cls} unranged"
        if near(float(b.xy[0]), float(b.xy[1])):
          return f"{b.camera} box {b.cls} at {b.xy[0]:.1f}, {b.xy[1]:.1f} m"
    return ''

  def turn_abandoned(self, sm, cs, v_ego: float, acting: bool) -> bool:
    """The model is steering away from the route's turn in the junction (TURN_ABANDON_*: a route turn in progress,
    turn_committed, the follower's distance under HOLD_LINE_M, under TURN_ABANDON_FRACTION of its angle turned, moving,
    modelV2.action.desiredCurvature away from its side): handed back from TURN_ABANDON_S on (module constants), held until
    the driver takes over (openpilot disengaged) or the turn is over. Nothing here steers, holds or lifts a cap.
    R8b (module constants TURN_MISSED_*): R8b's watch first; under TurnPilotAlwaysDrive R8's fire inside its bounds clears
    the desire (straight_cleared: CLEAR_DESIRE in this very plan) and hands nothing back, the turn missed at the pass
    (check_straight), and a bound broken after it is this handback, latched for the maneuver to its pass."""
    d, rf = self.desires, self.follower
    m = rf.upcoming
    enabled = sm['carControl'].enabled
    curvature = float(sm['modelV2'].action.desiredCurvature)
    self.straight_cleared = False
    self.watch_straight(curvature, default_yaw_rate(cs, sm['modelV2']), acting and enabled)
    if self.abandoned_id is not None:
      if enabled and self.abandoned_id == rf.maneuver_id and (self.turn_committed or (self.straight_fired and m is not None)):
        return True
      self.abandoned_id = None
    in_turn = acting and enabled and self.turn_committed and m is not None and m.kind == TURN and rf.distance < HOLD_LINE_M and \
      d.turned_deg < TURN_ABANDON_FRACTION * m.angle_deg
    away = float(sm['modelV2'].action.desiredCurvature) * m.side if m is not None else 0.  # positive right: away from a right
    leaving = in_turn and v_ego >= STANDSTILL_SPEED and away > TURN_ABANDON_CURV and not cs.gasPressed
    self.abandon_s = self.abandon_s + DT_MDL if leaving else 0.
    if self.abandon_s < TURN_ABANDON_S - 1e-6:
      return False
    self.abandon_s = 0.
    w = self.straight
    if w is not None and w.id == rf.maneuver_id:
      if self.always_drive_on and straight_inside(d.turned_deg, curvature):
        # R8b: the model chose straight: no handback, its desire cleared now so the queued pulse can't swing the car later
        w.fired, w.ok, w.deg = True, True, d.turned_deg
        d.cancel()
        self.straight_cleared = True
        cloudlog.event("turnpilot.turn_straight", turned_deg=round(d.turned_deg, 1), curvature=round(curvature, 4),
                       v=round(v_ego, 2), distance=round(rf.distance, 1))
        return False
      w.ok = False  # R8's handback: no miss at the pass
    self.abandoned_id = rf.maneuver_id
    cloudlog.event("turnpilot.turn_abandoned", error=True, turned_deg=round(d.turned_deg, 1), angle_deg=round(float(m.angle_deg), 1),
                   curvature=round(float(sm['modelV2'].action.desiredCurvature), 4), v=round(v_ego, 2), distance=round(rf.distance, 1),
                   steer_deg=round(float(cs.steeringAngleDeg), 1))
    return True

  def wrong_side(self, sm, cs, v_ego: float, acting: bool, paint, right_hand_traffic: bool) -> bool:
    """The car left of the centre line by the lane paint (yellow_on_the_right, WRONG_SIDE_*): handed back from WRONG_SIDE_S of
    reads on, held until the driver takes over (openpilot disengaged). Nothing here steers or holds."""
    enabled = sm['carControl'].enabled
    if self.wrong_side_on:
      self.wrong_side_on = enabled
      return self.wrong_side_on
    if not (acting and enabled and right_hand_traffic and v_ego >= WRONG_SIDE_MIN_SPEED and not cs.gasPressed):
      self.wrong_s = self.wrong_unread_s = 0.
      return False
    model = sm['modelV2']
    geom = model_lines_at(model)
    yellow, read = yellow_on_the_right(paint, *geom)
    if read:
      self.wrong_unread_s = 0.
      self.wrong_s = self.wrong_s + DT_MDL if yellow else 0.
    else:
      self.wrong_unread_s += DT_MDL
      if self.wrong_unread_s > WRONG_SIDE_GAP_S:
        self.wrong_s = 0.
    if self.wrong_s < WRONG_SIDE_S - 1e-6:
      return False
    self.wrong_side_on, self.wrong_s = True, 0.
    i = 3 if paint[2].kind == 'none' else 2
    cloudlog.event("turnpilot.wrong_side", error=True, v=round(v_ego, 2), line=i, paint=[[str(q.kind), str(q.color), round(q.confidence, 2)] for q in paint],
                   line_y=round(geom[0][i], 2), edge_y=round(geom[2][1], 2), edge_std=round(geom[3][1], 2))
    return True

  def off_route_turn(self, sm, cs, v_ego: float, acting: bool, rf, upcoming, yaw_rate: float) -> bool:
    """The car turning off its route with no desire of TurnPilot's (OFF_ROUTE_*): handed back from the frame its heading has
    changed OFF_ROUTE_TURN_DEG within OFF_ROUTE_TURN_M of travel away from any route maneuver, held until the driver takes over
    (openpilot disengaged). Nothing here steers or brakes."""
    enabled = sm['carControl'].enabled
    if sm.updated['navInstruction']:
      ni = sm['navInstruction']
      ok = sm.valid['navInstruction'] and len(ni.routeTurns) and ni.routeTurnsStep > 0.
      self.or_shape = (list(ni.routeTurns), float(ni.routeTurnsStart), float(ni.routeTurnsStep), self.odo_m) if ok else ()
    if self.or_on:
      self.or_on = enabled
      return self.or_on
    if rf.maneuver_id != self.or_id:
      self.or_id, self.or_prev_after = rf.maneuver_id, self.or_after
    bearings = upcoming.bearings if upcoming is not None and upcoming.geometry and len(upcoming.bearings) == 2 else None
    self.or_after = bearings[1] if bearings is not None else None
    d = self.desires
    desire_out = (d.started and not d.done and d.side != 0) or self.last_desire in (Desire.turnLeft, Desire.turnRight)
    if upcoming is None or abs(rf.distance) <= OFF_ROUTE_NEAR_M or desire_out:
      self.or_near_odo = self.odo_m
    if not (acting and enabled and upcoming is not None and not cs.gasPressed) or self.odo_m - self.or_near_odo <= OFF_ROUTE_NEAR_M:
      self.or_win = []
      return False
    if v_ego >= YAW_MIN_SPEED:
      self.or_win.append((self.odo_m, math.degrees(yaw_rate * DT_MDL)))
      while self.or_win and self.or_win[0][0] < self.odo_m - OFF_ROUTE_TURN_M:
        self.or_win.pop(0)
    right = -sum(y for _, y in self.or_win)  # deg, + right (the bearings' sense)
    if abs(right) < OFF_ROUTE_TURN_DEG - 1e-6:
      return False
    bend = turn_angle(self.or_prev_after, bearings[0]) if bearings is not None and self.or_prev_after is not None else 0.
    if bend * right > 0. and abs(bend) >= OFF_ROUTE_TURN_DEG - OFF_ROUTE_BEND_MARGIN_DEG:
      return False  # the route bends that way in this step
    shape = self.route_shape_turn(right)
    if shape is not None and abs(shape) >= OFF_ROUTE_TURN_DEG - OFF_ROUTE_BEND_MARGIN_DEG:
      return False  # the road itself turns that way here
    self.or_on, self.or_win = True, []
    cloudlog.event("turnpilot.off_route_turn", error=True, v=round(v_ego, 2), turned_deg=round(right, 1), bend_deg=round(bend, 1),
                   shape_deg=None if shape is None else round(shape, 1), distance=round(rf.distance, 1), maneuver_id=rf.maneuver_id)
    return True

  def route_shape_turn(self, right: float) -> float | None:
    """The route's largest heading change right's way (deg, its sign; 0 none that way) over any OFF_ROUTE_SHAPE_WINDOW_M of it
    around the car's window (OFF_ROUTE_SHAPE_*), from navd's routeTurns dead-reckoned by the odometer; None with no shape."""
    if not self.or_shape:
      return None
    turns, start, step, odo = self.or_shape
    car = self.odo_m - odo  # m along the route from the fix's place
    lo, hi = car - OFF_ROUTE_TURN_M - OFF_ROUTE_SHAPE_BEHIND_M, car + OFF_ROUTE_SHAPE_AHEAD_M
    heading = [0.]
    for t in turns:
      heading.append(heading[-1] + t)
    edges = [(start + i * step, h) for i, h in enumerate(heading) if lo - 1e-6 <= start + i * step <= hi + 1e-6]
    sign = 1. if right > 0. else -1.
    best = 0.
    for i, (a, ha) in enumerate(edges):
      for b, hb in edges[i + 1:]:
        if b - a > OFF_ROUTE_SHAPE_WINDOW_M + 1e-6:
          break
        best = max(best, sign * (hb - ha))
    return sign * best

  def watch_lead(self, model, objects, raw) -> None:
    """This frame's close lead (close_lead) and whether the live scene corroborates it (lead_corroboration), held
    LEAD_SEEN_HOLD_S between detector runs: lead_held, what the stand handbacks let hold the car (module constants)."""
    close = close_lead(model)
    why = lead_corroboration(model, objects, raw) if close else ''
    if why:
      self.lead_seen_s, self.lead_why = 0., why
    elif close:
      self.lead_seen_s += DT_MDL
    else:
      self.lead_seen_s, self.lead_why = float('inf'), ''
    self.lead_held = close and self.lead_seen_s <= LEAD_SEEN_HOLD_S + 1e-6
    if close:  # a queue moving on ahead (ST-09 (stalls)): a velocity-trusted vehicle track at the lead, moving
      x = float(model.leadsV3[0].x[0])
      moving = any(str(o.cls) in VEHICLES and abs(o.x - x) <= LEAD_CORROBORATE_M and in_path(o) and velocity_trusted(o) and
                   math.hypot(o.vx, o.vy) >= QUEUE_MOVING_SPEED for o in objects)
      self.lead_still_s = 0. if moving else self.lead_still_s + DT_MDL
    else:
      self.lead_still_s = 0.

  def lead_log(self, model) -> dict:
    """modelV2's lead and its corroboration, for a stand handback's log line."""
    if not close_lead(model):
      return {'lead': False}
    lead = model.leadsV3[0]
    return {'lead': True, 'lead_x': round(float(lead.x[0]), 1), 'lead_prob': round(float(lead.prob), 2), 'lead_held': self.lead_held,
            'corroborated': self.lead_why if self.lead_held else ''}

  def stopped_unheld(self, sm, cs, v_ego: float, acting: bool, decision, mapd, objects=(), raw=None) -> str:
    """The car standing engaged with nothing holding it (module docstring): '' while it isn't, or not for long; STOPPED_NOTE
    after STOPPED_NOTE_S (LIGHT_WAIT_NOTE at a mapped signal under the switch); the handback's reason after STOPPED_HANDBACK_S,
    NEAR_CONTROL_STAND_S near a map control (under the switch at a signal the camera reads red, LIGHT_RED_MAX_S at most), and
    on until the stand ends."""
    lead = self.lead_held  # a lead the scene corroborates (a queue); one it doesn't holds nothing (watch_lead)
    near = [c for c in list(self.map_controls.tracker.controls.values()) + [self.map_controls.turn]
            if c is not None and abs(c.d) < STOPPED_CONTROL_M]
    unheld = acting and sm['carControl'].enabled and v_ego < STANDSTILL_SPEED and not decision.should_stop and \
      not self.go_withdrawn and not self.release_stand and \
      not decision.holding and not lead and not cs.gasPressed and not cs.brakePressed and not map_handback(mapd)
    limit = NEAR_CONTROL_STAND_S if near else STOPPED_HANDBACK_S
    self.stopped_signal = any(c.kind == 'signal' for c in near)
    light_wait = self.always_drive_on and self.stopped_signal  # the model owns the light (ALWAYS_DRIVE.md ST-07 (stops))
    camera = self.map_controls.junction_lights(min(c.d for c in near if c.kind == 'signal'))[1] if light_wait else set()
    if 'red' in camera:
      limit = LIGHT_RED_MAX_S  # the camera explains the stand, a cycle at most; it never brakes the car
    self.stopped_s = self.stopped_s + DT_MDL if unheld else 0.
    green = light_wait and unheld and camera == {'green'} and not path_blocker(objects, raw)
    self.green_s = self.green_s + DT_MDL if green else 0.
    if not (light_wait and unheld and 'green' in camera):
      self.green_said = False  # another green, or another stand: said again
    if self.green_s >= LIGHT_GREEN_STEADY_S - 1e-6 and not self.green_said:
      self.green_said, self.green_alert_t = True, ABORT_HOLD_S
      cloudlog.event("turnpilot.light_green_not_moving", stood_s=round(self.stopped_s, 1), **self.lead_log(sm['modelV2']))
    was = self.stopped_said
    self.stopped_said = unheld and (self.stopped_said or self.stopped_s >= limit - 1e-6)
    if not self.stopped_said:
      note = LIGHT_WAIT_NOTE if light_wait else STOPPED_NOTE
      return note if self.stopped_s >= STOPPED_NOTE_S - 1e-6 else ''
    if not was:
      cloudlog.event("turnpilot.stopped_unheld", error=True, stood_s=round(self.stopped_s, 1), reason=decision.reason,
                     near=[c.kind for c in near], **self.lead_log(sm['modelV2']))
    return SIGNAL_STOPPED_REASON if self.stopped_signal else STOPPED_REASON

  def map_update(self, sm, v_ego: float, gas_pressed: bool, turn=None, left: LeftCommand | None = None, release=None, creep=None,
                 stop: StopCommand | None = None):
    """This frame's MapControlPolicy decision, from navInstruction's controls and turnpilotLights (once a frame), the
    route's unprotected turn (MapControlPolicy.update's turn) and the unprotected left's command for it, and the stop-sign
    decider's release, creep and command for its latch (a fault's call carries none: the latch as it was told last)."""
    if self.map_now is not None:
      return self.map_now
    data = getattr(sm, 'data', {})
    lights_msg = None
    if 'turnpilotLights' in data:
      self.lights_age = 0. if sm.updated['turnpilotLights'] else self.lights_age + DT_MDL
      lights_msg = sm['turnpilotLights'] if self.lights_age < float('inf') else None
    lights = light_input(lights_msg, self.lights_age, self.oracle_ok, True)
    lights.camera = self.camera_lights(sm, data)
    self.map_controls.yellow_decel = self.yellow_decel
    if stop is None and self.sg_now is not None:
      stop = StopCommand(self.sg_now.latch if self.sg_now.latch != 'none' else 'wait', self.sg_now.reason, self.sg_now.action)
    # this tick's forward runs and their class-11 boxes (row C11's evidence), None with the camera's stop signs not read
    signs = None
    if self.camera_signs:
      forward = [raw for key, _t, raw, _s in self.runs_now if key in ('road', 'full') and raw is not None]
      signs = (len(forward), [s for raw in forward for s in getattr(raw, 'signs', ())])
    self.map_now = self.map_controls.update(sm['navInstruction'], sm.updated['navInstruction'], sm.valid['navInstruction'], v_ego,
                                            gas_pressed, lights, DT_MDL, turn=turn, left=left, release=release, creep=creep, stop=stop,
                                            signs=signs, engaged=bool(sm['carControl'].enabled))
    return self.map_now

  def camera_lights(self, sm, data):
    """The camera's lights (map_controls.camera_light_input) from the last turnpilotDetections of a road camera run, or one
    saying the reader is off; a wide camera run keeps the last road read, which ages out after CAMERA_STALE_S."""
    self.camera_age += DT_MDL
    if 'turnpilotDetections' in data and sm.updated['turnpilotDetections']:
      td = sm['turnpilotDetections']
      status = str(td.lightStatus)
      if status != 'wideCamera':
        self.camera_read, self.camera_age = camera_light_input(td.lights, status, 0., self.camera_mode), 0.
    cam = self.camera_read
    if cam is None or (cam.status != 'disabled' and self.camera_age > CAMERA_STALE_S):
      return None
    return cam

  def _take_up_dropped(self, upcoming, rf, v_ego: float, right_hand_traffic: bool = True) -> None:
    """TurnPilot acts again after a handback (the detector is back): the maneuver dropped at the handback is never left
    half driven, capped with no desire and no alert. A held turn desire still out (openpilot stayed engaged) drives on,
    as does a turn the driver has already started and one a standstill re-armed (it pulses again as the car pulls away);
    a maneuver whose desire never went out is taken up as on a fresh engage. A turn whose desire was dropped while
    openpilot was off gets a fresh pulse when the car can still make it and is ENTRY_M or more out (the entry pulse's
    lead: the first pulse may still be in the model's queue). Otherwise it is skipped out loud (tooFastToTurn, navd
    reroutes), and as its first pulse may still be in the model's queue, that is cleared (CLEAR_DESIRE) and the turn's
    cap kept until it is passed: the model never turns from it at speed, and the car never speeds up into the junction.
    So is an unprotected turn whose stop at the line never engaged and can't be made now: never crossed blind."""
    d = self.desires
    dropped = self.dropped_id == rf.maneuver_id and d.maneuver_id == rf.maneuver_id
    self.dropped_id = self.turn_cap_id = self.turn_held_id = None
    if not dropped or upcoming is None:
      return
    stop_d = self.map_controls.stop_ahead()
    blind = unprotected_turn(upcoming, right_hand_traffic) and not self.map_controls.turn_engaged(rf.maneuver_id) and \
      not turn_stop_feasible(v_ego, rf.distance) and not (stop_d is not None and stop_d <= rf.distance + PROTECT_PAST_M)
    if blind and d.started and d.side != 0 and not d.done and d.turned_deg < TURN_STARTED_DEG:
      # an unprotected turn whose stop never engaged, too close now to stop for at its line: never crossed blind on the
      # desire still out, skipped out loud like one too close to pulse again (its desire cleared, its cap kept)
      self.skip.skip(upcoming, rf.distance, rf.maneuver_id, v_ego, 'unprotectedStop')
      d.cancel()
      self.skip_cap_id = rf.maneuver_id
      return
    if d.started and d.side != 0 and not d.done and d.held_off and d.turned_deg < TURN_STARTED_DEG:
      curb_side = (upcoming.side == LEFT) != right_hand_traffic
      if rf.distance >= ENTRY_M and can_make_turn(v_ego, rf.distance, upcoming.angle_deg, curb_side):
        return  # pulses again on its own, as on re-engaging
      self.skip.skip(upcoming, rf.distance, rf.maneuver_id, v_ego, 'desireDropped')
      d.cancel()
      self.skip_cap_id = rf.maneuver_id
    else:
      d.resume()

  def plan(self, sm, scene, enabled: bool, shadow: bool, *args, long_control: bool | None, disengage_on_accel: bool = True,
           **kwargs):
    """The plan (plan_inner()), every one of them carrying navd's runtime inputs (fill_runtime)."""
    self.shadow = shadow
    self.hand_always_drive()
    msg = self.plan_inner(sm, scene, enabled, shadow, *args, long_control=long_control, disengage_on_accel=disengage_on_accel,
                          **kwargs)
    self.fill_runtime(msg.turnpilotPlan)
    return msg

  def fill_runtime(self, plan) -> None:
    """turnpilotPlan.stopGo's runtime inputs, which navd's certifier reads and re-runs its avoidance on when they change
    (STOP_SIGNS.md 3.2, N11): the mode TurnPilotStopSignGo sets for the drive (shadow when this build doesn't know it), never
    the decider's shadow on a handback; the side views there for the drive; their maps valid (a calibration move, transient).
    On every plan, a disabled one's too, so navd never reads a message's defaults."""
    g = plan.stopGo
    g.mode = self.stop_sign_mode if self.stop_sign_known and self.stop_sign_mode in ('off', 'shadow', 'trial', 'auto', 'vision') else 'shadow'
    g.sideViewsLive, g.mapsValid = side_views_live(self.side_status), self.side_status != 'mapsStale'

  def plan_inner(self, sm, scene, enabled: bool, shadow: bool, *args, long_control: bool | None, disengage_on_accel: bool = True,
                 **kwargs):
    """update(), but a disabled plan while openpilot doesn't control speed (on a Tesla, without alpha longitudinal):
    its speed caps would do nothing there, and the turn desires would go out at whatever speed stock ACC holds.
    long_control None: card hasn't written CarParams yet, so it isn't known. A disabled plan while TurnPilot is on
    carries its fault, which selfdrived shows (turnpilotUnavailable): TurnPilot never stops working unannounced.
    It also carries a model fallback's handback, which outranks the fault (see the module docstring): a fallback lasts
    the drive, so it is noted here too, where update() isn't reached.
    disengage_on_accel: the DisengageOnAccelerator param. Off, any gas press no longer disengages (Kellen, 2026-10-28: it
    must), so TurnPilot faults (Fault.disengageOnAccelerator, turnpilotUnavailable) instead of running with it off
    unannounced; the holds in force stay on the faulted plan (fault_limits), so no held car is let go."""
    self.handback_now = self.scene_now = self.map_now = self.sg_now = None
    self._rf_ticked = self._yaw_ticked = False
    try:
      self.odo_m += max(float(sm['carState'].vEgo), 0.) * DT_MDL
    except Exception:
      log_error("turnpilotd.odometer_failed")
    try:
      self.watch_signs(kwargs.get('raw'), max(float(sm['carState'].vEgo), 0.))
    except Exception:
      log_error("turnpilotd.watch_signs_failed")
      self.sign_clear_m = 0.  # nothing known of the road behind: as a sign seen
    if enabled and long_control and not disengage_on_accel:
      self.fault_track(sm)
      self.stop('fault')
      self.watch_model(sm['modelV2'], enabled)
      msg = disabled_plan(shadow, GAS_DISENGAGE_REASON, Fault.disengageOnAccelerator,
                          Handback.modelFellBack if self.fell_back else Handback.none)
      self.fill_stop_abort(msg.turnpilotPlan)
      self.fault_limits(msg.turnpilotPlan, sm, scene)
      return msg
    if enabled and not long_control:
      self.fault_track(sm)
      self.stop('fault')
      self.watch_model(sm['modelV2'], enabled)
      handback = Handback.modelFellBack if self.fell_back else Handback.none
      msg = disabled_plan(shadow, CAR_UNKNOWN_REASON, handback=handback) if long_control is None else \
        disabled_plan(shadow, NO_LONG_REASON, Fault.noLongitudinal, handback)
      self.fill_stop_abort(msg.turnpilotPlan)
      return msg
    try:
      return self.update(sm, scene, enabled, shadow, *args, **kwargs)
    except Exception:
      log_error("turnpilotd.plan_failed")
      self.fault_track(sm)
      self.stop('fault')
      if self.turn_committed and self.hold_v is None:
        try:  # the turn's cap is gone with the plan: the car is held to its speed (module docstring), not sped up in the turn
          if sm['carControl'].latActive:
            self.hold_v = max(sm['carState'].vEgo, 0.)
        except Exception:
          log_error("turnpilotd.fault_hold_failed")
      # the detector's deadlines run on: a turnpilotd failing every frame still hands back a stopped detector
      try:
        handback = self.handback(scene.available, sm.updated['modelV2'], sm['carControl'].enabled)
      except Exception:
        log_error("turnpilotd.fault_handback_failed")
        handback = Handback.modelFellBack if self.fell_back else Handback.none
      msg = disabled_plan(shadow, ERROR_REASON, Fault.error, handback)
      self.fill_stop_abort(msg.turnpilotPlan)
      self.fault_limits(msg.turnpilotPlan, sm, scene)
      return msg

  def fault_limits(self, plan, sm, scene) -> None:
    """A faulted plan's limits: the scene's, with no route (scene_policy.py: ScenePolicy.fault), and the map stops'
    (their latch above all). The ones update() made this frame before it failed, if it got that far; the last plan's if
    the scene fails too."""
    try:
      decision = self.scene_now
      cs = sm['carState']
      if decision is None:
        try:  # the map stops in force, the latch above all (map_controls.py): a fault never lets a held car go
          # the route is lost with the fault: an unprotected turn's stop carries on, dead-reckoned (map_controls.py)
          stops = self.map_update(sm, max(cs.vEgo, 0.), cs.gasPressed, KEEP_TURN).stops
        except Exception:
          log_error("turnpilotd.fault_map_controls_failed")
          stops = self.map_controls.held_stops()
        try:  # the stop-sign decider's: a go in hand withdrawn (its capped abort), its own hold kept
          stops = stops + self.stop_go.fault_stops(max(cs.vEgo, 0.), self.odo_m)
        except Exception:
          log_error("turnpilotd.fault_stop_go_failed")
        took_over = cs.gasPressed and not sm['carControl'].enabled
        decision = self.scene.update(list(scene.objects) if scene.available else [], scene.available, None, float('inf'),
                                     max(cs.vEgo, 0.), gas_pressed=took_over, extra_stops=stops,
                                     driver=not sm['carControl'].enabled or cs.gasPressed)
        self.was_stopping = decision.should_stop
      if self.hold_v is not None:
        self.hold_v = None if not sm['carControl'].latActive else \
          max(cs.vEgo, 0.) if cs.gasPressed else min(self.hold_v, max(cs.vEgo, 0.))
      speed_cap = decision.speed_cap if self.hold_v is None else min(decision.speed_cap, self.hold_v)
      self.limits = (decision.accel_cap, speed_cap, decision.should_stop)
    except Exception:
      log_error("turnpilotd.fault_limits_failed")
    plan.accelCap, plan.speedCap, plan.shouldStop = self.limits


def fill_map_control(plan, mapd) -> None:
  """turnpilotPlan.mapControl, controlPrompt and controlAlert from MapControlPolicy's decision."""
  plan.controlPrompt, plan.controlAlert = mapd.prompt, mapd.alert
  shown = mapd.shown
  if shown is None:
    return
  mc = plan.mapControl
  mc.id, mc.kind, mc.distance, mc.action = shown.id, shown.kind, float(shown.distance), shown.action
  mc.lightState, mc.lightSource, mc.controlsStatus, mc.lineMapped = shown.light_state, shown.light_source, shown.controls_status, \
    shown.line_mapped


def oracle_accepted(environ, oracle_param: bool) -> bool:
  """The sim's light oracle is taken only in the sim (SIMULATION=1) with TurnPilotSimLightOracle set: never on the car."""
  return environ.get('SIMULATION') == '1' and bool(oracle_param)


def read_bool(params, key: str, default: bool) -> bool:
  """A param this build's params library may not know yet (rebuild common/ after a merge): default, said out loud."""
  try:
    return params.get_bool(key)
  except UnknownKeyName:
    log_error(f"turnpilotd: {key} unknown to this build's params; run scons")
    return default


def yellow_decel_for(params) -> float:
  """TurnPilotYellowDecel, 2.0 to 2.5 m/s^2 (2.5 needs Kellen's sign-off); 2.0 if unset or unknown to this build."""
  try:
    value = params.get("TurnPilotYellowDecel")
  except UnknownKeyName:
    log_error("turnpilotd: TurnPilotYellowDecel unknown to this build's params; run scons")
    return YELLOW_DECEL
  try:
    return min(max(float(value), YELLOW_DECEL), YELLOW_DECEL_MAX) if value is not None else YELLOW_DECEL
  except (TypeError, ValueError):
    return YELLOW_DECEL


def post_go_profile_for(params, key: str, default: str) -> str:
  """TurnPilotPostGoProfile or TurnPilotLeftGoProfile: a turn_speed.POST_GO_PROFILES name; default if unset, unknown (said
  out loud) or unknown to this build's params."""
  try:
    value = params.get(key)
  except UnknownKeyName:
    log_error(f"turnpilotd: {key} unknown to this build's params, post-go profile {default}; run scons")
    return default
  value = value.decode() if isinstance(value, bytes) else value
  if value is None:
    return default
  if value not in POST_GO_PROFILES:
    log_error(f"turnpilotd: {key} {value!r} is no post-go profile ({', '.join(POST_GO_PROFILES)}), {default} instead")
    return default
  return value


def auto_left_for(params) -> str:
  """TurnPilotAutoLeft: off, announce or auto (unprotected_left.py); auto if unset, unknown (said) or unknown to this build."""
  try:
    value = params.get("TurnPilotAutoLeft")
  except UnknownKeyName:
    log_error(f"turnpilotd: TurnPilotAutoLeft unknown to this build's params, {AUTO}; run scons")
    return AUTO
  mode = left_mode_for(value)
  raw = value.decode() if isinstance(value, bytes) else value
  if raw is not None and raw != mode:
    log_error(f"turnpilotd: TurnPilotAutoLeft {raw!r} is no mode (off, announce, auto), {AUTO} instead")
  return mode


def left_range_for(params) -> float:
  """TurnPilotLeftRangeM, the detector's proven range for oncoming traffic, clamped to 0..LEFT_RANGE_MAX_M; ROAD_RANGE_M
  (unmeasured: 0, no autonomous left) if unset, not a number (said) or unknown to this build."""
  try:
    value = params.get("TurnPilotLeftRangeM")
  except UnknownKeyName:
    log_error("turnpilotd: TurnPilotLeftRangeM unknown to this build's params; run scons")
    return ROAD_RANGE_M
  try:
    return min(max(float(value), 0.), LEFT_RANGE_MAX_M) if value is not None else ROAD_RANGE_M
  except (TypeError, ValueError):
    log_error(f"turnpilotd: TurnPilotLeftRangeM {value!r} isn't a number, {ROAD_RANGE_M} m instead")
    return ROAD_RANGE_M


def left_far_range_for(params) -> float:
  """TurnPilotLeftFarRangeM, the far crop's proven range for oncoming traffic, clamped to 0..LEFT_RANGE_MAX_M; 0 (unmeasured:
  the far crop extends no left's reach) if unset, not a number (said) or unknown to this build."""
  try:
    value = params.get("TurnPilotLeftFarRangeM")
  except UnknownKeyName:
    log_error("turnpilotd: TurnPilotLeftFarRangeM unknown to this build's params; run scons")
    return 0.
  try:
    value = value.decode() if isinstance(value, bytes) else value
    return min(max(float(value), 0.), LEFT_RANGE_MAX_M) if value is not None else 0.
  except (TypeError, ValueError):
    log_error(f"turnpilotd: TurnPilotLeftFarRangeM {value!r} isn't a number, unmeasured instead")
    return 0.


def no_move_for(params) -> float:
  """TurnPilotNoMoveS, clamped to NO_MOVE_S..NO_MOVE_MAX_S; NO_MOVE_S if unset, unreadable or unknown to this build."""
  try:
    value = params.get("TurnPilotNoMoveS")
  except UnknownKeyName:
    log_error("turnpilotd: TurnPilotNoMoveS unknown to this build's params; run scons")
    return NO_MOVE_S
  try:
    return min(max(float(value), NO_MOVE_S), NO_MOVE_MAX_S) if value is not None else NO_MOVE_S
  except (TypeError, ValueError):
    log_error(f"turnpilotd: TurnPilotNoMoveS {value!r} isn't a number, {NO_MOVE_S} s instead")
    return NO_MOVE_S


def go_override_for(params) -> bool:
  """TurnPilotGoOverride: whether TurnPilot's go is certified for plannerd's bounded lift (go_override.py); GO_OVERRIDE_DEFAULT
  (on) if unset or unknown to this build's params (said)."""
  try:
    value = params.get("TurnPilotGoOverride")
  except UnknownKeyName:
    log_error(f"turnpilotd: TurnPilotGoOverride unknown to this build's params, {'on' if GO_OVERRIDE_DEFAULT else 'off'}; run scons")
    return GO_OVERRIDE_DEFAULT
  if value is None:
    return GO_OVERRIDE_DEFAULT
  if isinstance(value, (bytes, str)):
    value = value.decode() if isinstance(value, bytes) else value
    return value.strip() == '1'
  return bool(value)


def always_drive_for(params) -> bool:
  """TurnPilotAlwaysDrive (ALWAYS_DRIVE.md section 1): ALWAYS_DRIVE_DEFAULT (off, today's behaviour) if unset, or unknown to
  this build's params (said); on only for a 1."""
  try:
    value = params.get("TurnPilotAlwaysDrive")
  except UnknownKeyName:
    log_error("turnpilotd: TurnPilotAlwaysDrive unknown to this build's params, off; run scons")
    return ALWAYS_DRIVE_DEFAULT
  if value is None:
    return ALWAYS_DRIVE_DEFAULT
  if isinstance(value, (bytes, str)):
    value = value.decode() if isinstance(value, bytes) else value
    return value.strip() == '1'
  return bool(value)


def curb_ped_cap_for(params) -> bool:
  """TurnPilotCurbPedCap: the scene's curb-pedestrian speed cap (scene_policy.py; owner decision, BUILD_FINDINGS D3-real
  option 2); CURB_PED_CAP_DEFAULT (on) if unset or unknown to this build's params (said)."""
  try:
    value = params.get("TurnPilotCurbPedCap")
  except UnknownKeyName:
    log_error(f"turnpilotd: TurnPilotCurbPedCap unknown to this build's params, {'on' if CURB_PED_CAP_DEFAULT else 'off'}; run scons")
    return CURB_PED_CAP_DEFAULT
  if value is None:
    return CURB_PED_CAP_DEFAULT
  if isinstance(value, (bytes, str)):
    value = value.decode() if isinstance(value, bytes) else value
    return value.strip() == '1'
  return bool(value)


def far_peds_for(params) -> bool:
  """TurnPilotFarPeds: tracker B (scene/tracker_b.py) beside tracker A; off if unset or unknown to this build's params (said)."""
  return read_bool(params, "TurnPilotFarPeds", False)


def camera_signs_read(params) -> bool:
  """TurnPilotCameraStopSigns isn't off: modeld decodes the class-11 boxes of every forward run (detector/modeld_detector.py
  camera_stop_signs_mode: off reads none; shadow, the default, or anything else, or a build that doesn't know the key, reads
  them), so map_controls can say a mapped stop the camera didn't see (row C11). Off, nothing is claimed either way."""
  try:
    value = params.get("TurnPilotCameraStopSigns")
  except UnknownKeyName:
    return True
  value = value.decode() if isinstance(value, bytes) else value
  return value != 'off'


def stop_sign_mode_for(params) -> tuple[str, bool]:
  """TurnPilotStopSignGo (off, shadow, trial, auto, vision; stop_go.MODE_DEFAULT unset) and whether this build knows it: a value it
  doesn't know, or a build whose params predate the key, acts as shadow, said out loud (STOP_SIGNS.md row O16, H6)."""
  try:
    value = params.get("TurnPilotStopSignGo")
  except UnknownKeyName:
    log_error("turnpilotd: TurnPilotStopSignGo unknown to this build's params, acting as shadow; run scons")
    return 'shadow', False
  mode, known = mode_for(value)
  if not known:
    log_error(f"turnpilotd: TurnPilotStopSignGo {value!r} is no mode (off, shadow, trial, auto, vision), acting as shadow")
  return mode, known


def disabled_plan(shadow: bool, reason: str, fault=Fault.none, handback=Handback.none):
  """A plan nothing acts on: no desire, no lane change, no limits. handback: a model fallback, which selfdrived still
  raises through a disabled plan (hooks.fresh_plan)."""
  msg = messaging.new_message('turnpilotPlan')
  plan = msg.turnpilotPlan
  plan.state, plan.shadow, plan.reason, plan.fault, plan.handback = State.disabled, shadow, reason, fault, handback
  plan.accelCap, plan.speedCap = NO_ACCEL_CAP, NO_SPEED_CAP
  return msg


def car_params(params):
  """CarParams as card last wrote them, or None: not yet written, or unreadable."""
  cp_bytes = params.get("CarParams")
  if cp_bytes is None:
    return None
  try:
    return messaging.log_from_bytes(cp_bytes, car.CarParams)
  except Exception:
    log_error("turnpilotd.car_params_unreadable")
    return None


def unverified_why_for(params, cp) -> str:
  """With TurnPilotCarSignals on, why no verified record matched this car (car_signals.record_matches), '' when one did:
  with TurnPilotCarSignalsVinless on, on the identity card's resolver formed from the same VIN and firmware."""
  if cp is None:
    return "waiting for the car's parameters"
  try:
    record = params.get("TurnPilotCarSignalsVerified")
  except UnknownKeyName:
    return "this build's params lack the car signal keys: run scons"
  return record_matches(record, cp.carVin, cp.carFw, "SIMULATION" in os.environ, identity_for(params, cp.carVin, cp.carFw))[1]


def long_control_for(params) -> bool | None:
  """CarParams.openpilotLongitudinalControl; None until card has written CarParams, False if they don't read."""
  if params.get("CarParams") is None:
    return None
  cp = car_params(params)
  return cp is not None and cp.openpilotLongitudinalControl


def lane_paint_active(params) -> bool:
  """TurnPilotLanePaintActive; off when this build's params library predates the key (rebuild common/ after a merge)."""
  try:
    return params.get_bool("TurnPilotLanePaintActive")
  except UnknownKeyName:
    cloudlog.error("turnpilotd: TurnPilotLanePaintActive unknown to this build's params, lane paint rule off; run scons")
    return False


INVALID_RUNS: dict[str, int] = {}  # detector_run: invalid runs in a row, by service


def run_invalid(service: str) -> None:
  """An invalid run of service: counted, the first of a spell logged."""
  INVALID_RUNS[service] = INVALID_RUNS.get(service, 0) + 1
  if INVALID_RUNS[service] == 1:
    cloudlog.event("turnpilot.detector_run_invalid", error=True, service=service)


def run_valid(service: str) -> None:
  """A valid run of service: how many invalid ones came before it, logged."""
  if INVALID_RUNS.get(service):
    cloudlog.event("turnpilot.detector_run_valid_again", service=service, invalid_runs=INVALID_RUNS.pop(service))


def detector_run(sm, service: str):
  """The latest run of a detector pass (turnpilotFarDetections) for the tracker, None before the first and while the latest
  is invalid. modeld publishes a run whose output wasn't finite valid False, its non-finite rows dropped (no boxes, a far
  run's crop still set): taken as a run it is a clear one, the far crop covering the band. SubMaster sets sm.valid from
  msg.valid even for services in ignore_valid, so an invalid run is no run (skeptic G6): the left sees none, never clear
  (stale is its fault), and a detector that stays invalid leaves the scene unavailable (the detector's handback). The first
  invalid run of a spell is logged, and how many there were when a valid one comes back. turnpilotDetections comes through
  DetectionDrain by the same rule."""
  if not sm.seen[service]:
    return None
  if not sm.valid[service]:
    if sm.updated[service]:
      run_invalid(service)
    return None
  run_valid(service)
  return sm[service]


class DetectionDrain:
  """turnpilotDetections for the tracker, off turnpilotd's own non-conflated socket (STOP_SIGNS.md 1.3, row E13). modeld
  publishes it at 20 Hz under a stop sweep, one view a step, and turnpilotd's 20 Hz loop isn't in step with it: SubMaster's
  socket is conflated, so whenever two runs land between polls it keeps the latest and the other view's run is lost. Every
  message is drained here each frame; an invalid one is no run (detector_run's rule, logged the same way), and the rest go
  to the tracker in camera-time order (SceneTracker.update_runs). The SubMaster keeps its subscription for what reads the
  latest message (the far crop's and side views' status, the camera's lights, lane paint)."""
  SERVICE = 'turnpilotDetections'

  def __init__(self, sock):
    self.sock = sock

  def runs(self) -> list:
    out = []
    for evt in messaging.drain_sock(self.sock):
      if not evt.valid:
        run_invalid(self.SERVICE)
        continue
      run_valid(self.SERVICE)
      out.append(evt.turnpilotDetections)
    return sorted(out, key=lambda d: d.timestampEof)


def plan_paint(det, scene_available: bool) -> tuple:
  """The lane paint for the plan from the latest turnpilotDetections (None before the first): its four lines only while the
  scene is available and this message's lanePaintStatus is read, else () (none read: unknown paint, no lane change). A
  LinePaint can't tell a line read unknown from one nobody read: a run on the wide camera, a stop sweep, the paint off for
  the drive (a budget overrun, an exception) and a road camera frame with no modelV2 for it all come as four unknowns
  (lane_paint_from_msg) or none, and only the status says which. A curb-side change on less than paint (ALWAYS_DRIVE.md
  4.10, lane_planner.curb_lane_unpainted) goes only on paint the reader read; every other rule plans () and four unknowns
  the same."""
  if det is None or not scene_available or det.lanePaintStatus != custom.TurnpilotDetections.LanePaintStatus.read or \
     len(det.lanePaint) != 4:
    return ()
  return lane_paint_from_msg(det.lanePaint)


def main():
  params = Params()
  pm = messaging.PubMaster(['turnpilotPlan', 'turnpilotScene'])
  # no route, no detector (TurnPilotEnabled off, no pkl), no light source (only the sim's oracle publishes one; the
  # camera's lights ride in turnpilotDetections, shadow unless TurnPilotCameraLights is addOnly)
  optional = ['navInstruction', 'turnpilotDetections', 'turnpilotLights', 'turnpilotFarDetections']
  sm = messaging.SubMaster(['carState', 'carControl', 'navInstruction', 'modelV2', 'turnpilotDetections', 'extrinsicsCalibration',
                            'deviceState', 'narrowRoadCameraState', 'turnpilotLights', 'turnpilotFarDetections'],
                           ignore_alive=optional, ignore_avg_freq=optional, ignore_valid=optional)
  drain = DetectionDrain(messaging.sub_sock(DetectionDrain.SERVICE, conflate=False))
  rk = Ratekeeper(1. / DT_MDL, print_delay_threshold=None)
  tp = TurnPilot()
  tracker = None
  tracker_b: TrackerB | None = None  # TurnPilotFarPeds: tracker B beside tracker A (scene/tracker_b.py)
  far_peds = b_failed = False        # ...b_failed: B raised, dropped for the drive
  enabled = shadow = paint_rule = False
  driver_signals = None  # from the turn signal's status, every frame (car_signals.py)
  right_hand_traffic, disengage_on_accel = True, True
  long_control: bool | None = None
  invalid_frames = PLAN_INPUT_GRACE_FRAMES
  while True:
    sm.update(0)
    if sm.frame % int(1. / DT_MDL) == 0:
      enabled = params.get_bool("TurnPilotEnabled")
      shadow = params.get_bool("TurnPilotShadowMode")
      paint_rule = lane_paint_active(params)
      right_hand_traffic = not params.get_bool("IsRhdDetected")
      cp = car_params(params)
      tp.car_signal_flags = int(cp.flags) if cp is not None and cp.brand == 'tesla' else 0
      tp.car_signal_mode = read_mode(params)
      tp.unverified_why = unverified_why_for(params, cp) if tp.car_signal_mode == 'on' else ''
      long_control = long_control_for(params)
      disengage_on_accel = params.get_bool("DisengageOnAccelerator")
      tp.oracle_ok = oracle_accepted(os.environ, read_bool(params, "TurnPilotSimLightOracle", False))
      tp.set_sim(visibility_table.sim_accepted(os.environ, read_bool(params, visibility_table.SIM_PARAM, False), car_params(params)))
      tp.yellow_decel = yellow_decel_for(params)
      tp.camera_mode = camera_lights_mode(params)
      tp.post_go_profile = post_go_profile_for(params, "TurnPilotPostGoProfile", POST_GO_DEFAULT)
      tp.left_go_profile = post_go_profile_for(params, "TurnPilotLeftGoProfile", LEFT_GO_PROFILE)
      tp.no_move_s = no_move_for(params)
      tp.auto_left = auto_left_for(params)
      tp.left_range_m = left_range_for(params)
      tp.left_far_range_m = left_far_range_for(params)
      tp.go_override = go_override_for(params)
      tp.always_drive = always_drive_for(params)
      tp.curb_ped_cap = curb_ped_cap_for(params)
      far_peds = far_peds_for(params)
      tp.stop_sign_mode, tp.stop_sign_known = stop_sign_mode_for(params)
      tp.camera_signs = camera_signs_read(params)
      tp.params = params
      cp = car_params(params)
      tp.cp_delay = float(cp.longitudinalActuatorDelay) if cp is not None else tp.cp_delay
      if params.get("ChestnutActive") is False:  # None: no big model expected this drive
        tp.big_model_failed(enabled)

    scene_msg = messaging.new_message('turnpilotScene')
    runs, far = [], None
    try:
      runs = drain.runs()  # every frame, the tracker or not, so nothing piles up
      for r in runs:  # the side views' lens, for the stop-sign decider's mismatch check (stop_go.py: H4)
        if str(r.view).startswith('side') and r.lens:
          tp.lenses['sideViews'] = str(r.lens)
      if tracker is None and sm.seen['deviceState'] and sm.seen['narrowRoadCameraState']:
        device_type = str(sm['deviceState'].deviceType)
        tracker = SceneTracker(DEVICE_CAMERAS[(device_type, str(sm['narrowRoadCameraState'].sensor))], device_type)
      if tracker is not None:
        tracker.coast_s = SWEEP_COAST_S if tp.sweeping else MAX_COAST_S
        tracker.update_runs(runs, sm['extrinsicsCalibration'], sm['carState'], sm['modelV2'])
        tp.observe_runs(tracker)
        far = detector_run(sm, 'turnpilotFarDetections')
        tracker.update_far(far, sm['extrinsicsCalibration'])
        tracker.fill_scene(scene_msg.turnpilotScene)
    except Exception:
      log_error("turnpilotd.scene_failed")
      tracker, scene_msg = None, messaging.new_message('turnpilotScene')  # unavailable: as with no detector, lefts hold
    pm.send('turnpilotScene', scene_msg)

    scene = scene_msg.as_reader().turnpilotScene
    # tracker B (TurnPilotFarPeds): the same runs and the far run, after A; its B-only people go to the scene policy alone. Built
    # with A and dropped with it; one that raises is said once and dropped for the drive, A carrying on
    extra = []
    if far_peds and tracker is not None and not b_failed:
      try:
        if tracker_b is None:
          device_type = str(sm['deviceState'].deviceType)
          tracker_b = TrackerB(SceneTracker(DEVICE_CAMERAS[(device_type, str(sm['narrowRoadCameraState'].sensor))], device_type))
        extra = tracker_b.step(runs, far, sm['extrinsicsCalibration'], sm['carState'], sm['modelV2'], tracker, scene)
      except Exception:
        log_error("turnpilotd.tracker_b_failed")
        tracker_b, b_failed, extra = None, True, []
    else:
      tracker_b = None
    paint = plan_paint(sm['turnpilotDetections'] if sm.seen['turnpilotDetections'] else None, scene.available)
    # the lane planner takes this frame's checks (an offered change aborts at once); the plan's valid has a grace
    valid = sm.all_checks(['carState', 'modelV2'])
    # this frame's run for the unprotected left: its road camera run when one came (the left reads only the road's), else
    # the newest of whatever came (the go's path check reads any)
    raw = None
    if tracker is not None:
      raw = tracker.raw_by_view.get('road') if 'road' in tracker.new_views else tracker.raw if tracker.raw_new else None
    raw_far = tracker.raw_far if tracker is not None and tracker.raw_far_new else None  # ...and the far crop's
    msg = tp.plan(sm, scene, enabled, shadow, driver_signals, right_hand_traffic, paint, paint_rule, valid, long_control=long_control,
                  disengage_on_accel=disengage_on_accel, raw=raw, raw_far=raw_far, extra_objects=extra)
    if tracker_b is not None:
      try:  # the B-only people the plan stopped for, counted and logged (turnpilot.tracker_b, turnpilot.tracker_b_stop)
        tracker_b.after_plan(tp.scene.carried if tp.scene_now is not None else (), max(float(sm['carState'].vEgo), 0.))
      except Exception:
        log_error("turnpilotd.tracker_b_failed")
        tracker_b, b_failed = None, True
    invalid_frames = 0 if valid else invalid_frames + 1
    msg.valid = invalid_frames < PLAN_INPUT_GRACE_FRAMES
    pm.send('turnpilotPlan', msg)
    if tp.cancel_route and not tp.turn_committed:  # a turn under way finishes first, its cap and desire held
      tp.cancel_route = False
      cloudlog.warning(f"turnpilot: driving model fell back to the small model, handing back{'' if shadow else ' and cancelling the route'}")
      if not shadow:
        params.remove("NavDestination")
    rk.keep_time()


if __name__ == "__main__":
  main()
