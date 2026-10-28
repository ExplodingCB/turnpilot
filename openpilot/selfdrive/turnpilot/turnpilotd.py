#!/usr/bin/env python3
"""turnpilotd: turns the active route and the scene into desire pulses, lane-change requests and longitudinal limits.

modeld takes the plan's desire and lane change, plannerd its accel and speed caps (see hooks.py). The lane paint the
detector reads comes in with its detections, and is used only while those are fresh (the scene is available); none read
is unknown paint, and no lane change. The lane planner plans lane changes only with TurnPilotLanePaintActive set; without
it a second planner publishes what it would have asked for in laneChangePaintRule, so drives can be scored. In shadow mode
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
navd reroutes at once. Nothing brakes for it. Standing more than STALL_PROMPT_S after the go, STALL_MIN_DEG to
STALL_MAX_DEG into the turn (from its start, for a go that came in the junction), with no hold of TurnPilot's and no lead
within STOPPED_LEAD_M, it stalled in the junction: handed back loudly (controlPrompt stalledTakeControl, STALLED_REASON),
never held there, and TurnPilot never goes itself.
From the go such a turn is driven under its post-go profile (turn_speed.py: PostGo; DESIGN phase 4 section 3): accel and
speed capped by the heading it has turned, DesireTiming.turned_deg at the go and the car's yaw after, until 60 deg or
POST_GO_MAX_M on, and dropped at the pass of a turn missed straight on. TurnPilotPostGoProfile names the profile
(graduated by default; none, flat3, flat43 for the BeamNG sweep), and an unprotected turn's go (unprotected_turn: a left
across the oncoming road) takes TurnPilotLeftGoProfile's (left: 1.5 m/s^2, 4.3 m/s). The caps only lower speed.
A go from the hold line (go_at_line: the car GO_LINE_M or more before the maneuver point, the scene not committed to the
junction nor released into it) the car hasn't pulled away on (STANDSTILL_SPEED) within NO_MOVE_S (TurnPilotNoMoveS, up to
NO_MOVE_MAX_S, for the sweep), LEFT_NO_MOVE_S for an unprotected turn (CRITIQUE
REQUIRED 4: its gap runs out while it stands), counting only while nothing else holds it (no TurnPilot hold, no close lead,
no pedal), is the model holding the car: TurnPilot withdraws the go before the car moves, holding it standing (shouldStop,
a 0 speed cap: braking at -2.0 at most, from under 0.3 m/s; no turn pulse), and hands back loudly (controlPrompt
cantStartTakeControl, NO_MOVE_REASON) until the driver takes over (openpilot disengaged: any gas press); a scene hold
that comes and goes meanwhile (a walker crossing) is no new go. The turn isn't cancelled: its desire state stays, and the
driver drives it. A go in the junction (a walker's hold released partway round) is never withdrawn: stopping there is worse
than finishing the turn, so TurnPilot never holds the car in it; it keeps the post-go caps, which only lower speed, and the
stall's handback without a hold.
The route's unprotected left (map_controls.unprotected_turn) is driven by unprotected_left.py (DESIGN phase 4 G4) under
TurnPilotAutoLeft: auto (the default: Kellen, 2026-10-28, "No it has to ship taking the turn"), announce or off (G1's
handback at the line). Each frame it is fed the hold line's latch and the scene from the last frame, this frame's raw
detector run (main(): SceneTracker.raw), the car's yaw toward the turn (track_yaw) and the opposing road's mapped speed and
lanes, and checked against TurnPilotLeftRangeM (the detector's proven range, provisional: ROAD_RANGE_M). Its command goes to
the latch (map_controls.LeftCommand: hold, creep, go, or a fault's handback), its own hold after a withdrawn go joins the
scene's stops, and no turn pulse goes on its approach (may_start off; turn_skip's pulse_at_go): its go from the line is the
first. It asks modeld for the road camera (turnpilotPlan.detectorView) and publishes its state (unprotectedLeft); its wait
is silent (waitingForGap, creepingForView), its go a chime, and it hands back (leftUndecidedTakeControl) only for a fault.
Its go is TurnPilot's like any hold's: the left's post-go profile and LEFT_NO_MOVE_S below.
Standing engaged with nothing of TurnPilot's holding the car (no shouldStop, no cap biting), no lead from the model within
STOPPED_LEAD_M (one further on doesn't hold a standing car), gas and brake off, the model stopped on its own: said on the
status line after STOPPED_NOTE_S (STOPPED_NOTE), and handed back loudly (controlPrompt carStoppedTakeControl) after
STOPPED_HANDBACK_S, or NEAR_CONTROL_STAND_S with a map control within STOPPED_CONTROL_M (CRITIQUE REQUIRED 6: a model
stall at a mapped signal TurnPilot doesn't hold for is never silent, and a red the map doesn't know, the car first in
line, isn't handed back before it can turn green). TurnPilot never goes itself; the driver taking over (a gas press
disengages) starts the wait over.
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
import math
import os
import time

from opendbc.car.structs import car
from openpilot.cereal import custom, log, messaging
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL, Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.desire_timing import ENTRY_M, TURN_STARTED_DEG, DesireTiming
from openpilot.selfdrive.turnpilot.control_text import STOPPED_NOTE
from openpilot.selfdrive.turnpilot.handback import Handback
from openpilot.selfdrive.turnpilot.hooks import ABORT_HOLD_S, CLEAR_DESIRE, CONTROL_STATUS_ALERTS, TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.detector.light_state import LIGHT_MODE_DEFAULT, camera_lights_mode
from openpilot.selfdrive.turnpilot.map_controls import (CAMERA_STALE_S, KEEP_TURN, LATCH_PAST_M, MAP_MARGIN_M, PROTECT_PAST_M, YELLOW_DECEL,
                                                         YELLOW_DECEL_MAX, LeftCommand, MapControlPolicy, camera_light_input, light_input,
                                                         turn_stop_feasible, unprotected_turn)
from openpilot.selfdrive.turnpilot.lane_planner import (OPENPILOT_SIGNALS_BRANDS, LanePlanner, lane_change_window, lane_mismatch,
                                                         model_lanes_from_msg)
from openpilot.selfdrive.turnpilot.route_follower import KEEP, LEFT, TURN, UNCERTAIN_M, RouteFollower
from openpilot.selfdrive.turnpilot.scene.lane_paint import lane_paint_from_msg
from openpilot.selfdrive.turnpilot.scene.tracker import STALE_S, SceneTracker, default_yaw_rate
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M, NO_ACCEL_CAP, ScenePolicy
from openpilot.selfdrive.turnpilot.turn_skip import TurnSkip, can_make_turn
from openpilot.selfdrive.turnpilot.turn_speed import LEFT_GO_PROFILE, NO_SPEED_CAP, POST_GO_DEFAULT, POST_GO_PROFILES, PostGo, turn_speed_cap
from openpilot.selfdrive.turnpilot.unprotected_left import (AUTO, DEFAULT_OPPOSING_SPEED, OFF, ROAD_RANGE_M, LeftInput, UnprotectedLeft,
                                                           left_mode_for, reach_at_line)

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
# a turn released from a standstill (a hold's go: BeamNG 2026-10-28, taken about half the time) is checked, never trusted:
MISSED_TURN_DEG = 15.  # past its junction (the follower's pass) turned less than this, it was missed: said and rerouted
MISSED_HOLD_S = 3.     # turnMissed is held this long, for the alert and for navd (1 Hz) to see it
TURN_MISSED_REASON = "Turn missed — rerouting"
STALL_PROMPT_S = 3.    # standing this long after the go, STALL_MIN_DEG to STALL_MAX_DEG into the turn, nothing holding it:
STALL_MIN_DEG = 20.    # handed back (the stop line it left is within STOPPED_CONTROL_M, so the empty-road handback
STALL_MAX_DEG = 80.    # comes only after the long stand; the sweep's stalls stood 20 s at 46-78 deg)
STALLED_REASON = "Stalled in the turn: take control"
# not at STANDSTILL_SPEED this long after TurnPilot's go, nothing else holding the car: the model is holding it (DESIGN phase 4
# section 2-3; G0, 2026-10-28: 14 of 24 tapless goes stood 20 s). Withdrawn and handed back; an unprotected turn's sooner
NO_MOVE_S = 2.
NO_MOVE_MAX_S = 20.  # TurnPilotNoMoveS is clamped to NO_MOVE_S..this (the sweep's V1 tapless column measures to 20 s)
LEFT_NO_MOVE_S = 1.
NO_MOVE_REASON = "Can't start the turn: take control"
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


def close_lead(model) -> bool:
  """modelV2 has a lead within STOPPED_LEAD_M: what holds a standing car (one further on doesn't)."""
  return len(model.leadsV3) > 0 and model.leadsV3[0].prob > STOPPED_LEAD_PROB and len(model.leadsV3[0].x) > 0 and \
    model.leadsV3[0].x[0] < STOPPED_LEAD_M


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
    self.left_now = None  # this frame's unprotected left decision (unprotected_left.LeftDecision), once update() has made it
    self._rf_ticked = self._yaw_ticked = False  # update() dead-reckoned the route, and counted the yaw, this frame
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
    self.stopped_s = 0.            # standing engaged with nothing holding the car (carStoppedTakeControl)
    self.stall_s = 0.              # standing in the turn after its go, nothing holding the car (stalledTakeControl)
    self.missed_t = 0.             # turnMissed is held while this counts down
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
      self.go_id: int | None = None  # the turn a hold let go of at standstill: checked for a miss and a stall
      self.post_go: PostGo | None = None  # ...and driven under its post-go profile (start_post_go)
      self.no_move_limit = NO_MOVE_S       # ...withdrawn if the car hasn't moved this long after it (no_move)
      self.go_wait_s: float | None = None  # s stood since TurnPilot's go, nothing else holding the car; None: moved, or no go
      self.go_withdrawn = False            # the go withdrawn: the car held standing, handed back (cantStartTakeControl)
      self.go_in_junction = False          # the go came with the car in the junction (go_at_line): never withdrawn
      self.approach_yaw_deg = 0.           # the car's heading change toward the route's turn since its first pulse (track_yaw)
      self._yaw_id: int | None = None      # ...for this maneuver
    else:
      self.desires.hold_off()  # the disabled plan dropped the desire: as on re-engaging, a fresh pulse only before the turn
    self.lanes = LanePlanner()
    self.paint_rule_lanes = LanePlanner(paint_rule=True)  # what the lane planner would ask for, while it's turned off
    self.mismatch = LaneMismatch.none
    self._mismatch_candidate, self._mismatch_frames = LaneMismatch.none, 0
    self.acting = False
    self.last_desire = Desire.none

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

  def update(self, sm, scene, enabled: bool, shadow: bool, driver_signals: bool = True, right_hand_traffic: bool = True, paint=(),
             paint_rule: bool = True, inputs_valid: bool = True, raw=None):
    """A turnpilotPlan message from sm's carState, navInstruction and modelV2, a TurnpilotScene, and the lane paint of
    modelV2's four lane lines (empty when unknown). paint_rule: TurnPilotLanePaintActive; inputs_valid: carState and
    modelV2 pass their checks; raw: the detector run that arrived this frame (scene.tracker.RawFrame), or None."""
    self.handback_now = self.scene_now = self.map_now = self.left_now = None
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
      return msg
    self._stop_abort_t = 0.

    cs, model = sm['carState'], sm['modelV2']
    v_ego = max(cs.vEgo, 0.)
    rf = self.follower
    rf.update(sm['navInstruction'], sm.updated['navInstruction'], sm.valid['navInstruction'], v_ego, DT_MDL,
              committed=self.turn_committed)
    self._rf_ticked = True
    upcoming = None if rf.arrived else rf.upcoming
    available = scene.available
    objects = list(scene.objects) if available else []

    handback = self.handback(available, sm.updated['modelV2'], sm['carControl'].enabled)
    plan.handback = handback
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
    self.check_missed(rf, v_ego, acting)
    self.track_yaw(upcoming, rf, default_yaw_rate(cs, model), v_ego)

    # the map's stop signs and signals: every frame, whatever the maneuver (a skipped turn keeps its stop). And the route's
    # unprotected turn while TurnPilot drives it: not skipped (too fast: skipped out loud and rerouted, never braked for at
    # the floor; last frame's skip, a frame late at most), not dropped at a handback. A held latch stays whatever
    # Its stop is engaged only where it can be made at -2.0; one that can't (first seen too close at speed) is skipped
    skipped = self.skip.skipped and self.skip.maneuver_id == rf.maneuver_id
    driving = acting and rf.maneuver_id != self.dropped_id and not skipped
    # an engaged one stays through a handback and after it, as a stop sign's stop does: the stops in force stay, and the
    # detector back is no reason to cross the blind junction without its stop and its handback
    engaged = self.map_controls.turn_engaged(rf.maneuver_id) and not skipped
    turn, stop_infeasible = None, False
    if (driving or engaged) and unprotected_turn(upcoming, right_hand_traffic):
      if engaged or turn_stop_feasible(v_ego, rf.distance):
        turn = (rf.maneuver_id, rf.distance, 'left' if upcoming.side == LEFT else 'right')
      else:
        stop_infeasible = True
    elif engaged and upcoming is None:
      turn = KEEP_TURN  # the route gone with the handback: it carries on, dead-reckoned
    left_cmd = self.unprotected_left(turn, upcoming, rf, v_ego, acting, objects, raw)
    mapd = self.map_update(sm, v_ego, cs.gasPressed, turn, left_cmd)
    took_over = cs.gasPressed and not sm['carControl'].enabled  # the driver driving, never a go (module docstring)
    # the autonomous left holds its turn's pulse until its go from the line (unprotected_left.py: approach), so its first pulse
    # is that go, never too late (turn_skip.py: pulse_at_go)
    hold_pulse = self.left_now is not None and self.left_now.state in LEFT_HOLDS_PULSE
    pulse_at_go = self.left_now is not None and self.left_now.state != 'off'

    # too fast to make the turn: skipped, so nothing below acts on it (no desire, no cap, no turn holds or lane changes)
    desire_out = self.desires.maneuver_id == rf.maneuver_id and self.desires.started
    # an autonomous left the detector can't see far enough for (its R_req from the opposing road's mapped speed past
    # TurnPilotLeftRangeM): never attempted, skipped as soon as it is the maneuver and rerouted (turn_skip.py)
    out_of_range = acting and self.auto_left == AUTO and unprotected_turn(upcoming, right_hand_traffic) and \
      reach_at_line(upcoming.opposing_speed, upcoming.opposing_lanes or None) > self.left_range_m
    skip = self.skip.update(upcoming, rf.distance, rf.maneuver_id, v_ego, desire_out, right_hand_traffic, lat_active,
                            rf.ahead, rf.ahead_distance, stop_d=self.map_controls.stop_ahead(), stop_infeasible=stop_infeasible,
                            pulse_at_go=pulse_at_go, out_of_range=out_of_range)
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

    # the scene's holds only ever brake, so they keep the real maneuver through a handback
    left_stops = self.left_now.stops if self.left_now is not None else []
    decision = self.scene_now = self.scene.update(objects, available, maneuver, rf.distance, v_ego, right_hand_traffic, rf.maneuver_id,
                                                 took_over, extra_stops=mapd.stops + left_stops, heading_deg=self.approach_yaw_deg)
    # a hold let go with the car standing (the walker cleared, a stop's go): the model won't pull away on its own, and the
    # turn's pulse is long forgotten, so it goes again in this very plan, the one that lets the car go: held, the desire
    # was dropped to none, so it is a fresh edge (desire_timing.py). BeamNG: let go a frame before its pulse, the car drove
    # straight across the junction
    self.release_go = self.was_stopping and not decision.should_stop and v_ego < STANDSTILL_SPEED
    self.was_stopping = decision.should_stop
    withdrawn = self.no_move(sm, cs, v_ego, acting, decision)
    self.release_go = self.release_go and not withdrawn  # handed back: a hold that comes and goes is no new go

    lanes = model_lanes_from_msg(model)
    desire = self.desires.update(desire_on, rf.distance, rf.maneuver_id, rf.passed, v_ego, cs.steeringAngleDeg,
                                 lanes.keep_left_ok, lanes.keep_right_ok, DT_MDL, default_yaw_rate(cs, model),
                                 lat_active=lat_active, replaced=rf.replaced,
                                 may_start=skip.feasible_now and not held and not hold_pulse, go=self.release_go,
                                 holding=decision.should_stop or withdrawn)
    self._yaw_ticked = True
    d = self.desires
    if self.release_go and d.maneuver_id == rf.maneuver_id and d.started and d.side != 0 and not d.done and upcoming is not None:
      self.go_id, self.stall_s = rf.maneuver_id, 0.
      self.go_in_junction = not self.go_at_line(rf.distance)
      if sm['carControl'].enabled and not took_over:  # TurnPilot's go; the driver driving off a hold is the driver's
        self.start_post_go(upcoming, rf.distance, d.turned_deg, right_hand_traffic)
    elif self.post_go is not None:
      self.post_go.update(v_ego, default_yaw_rate(cs, model), DT_MDL)
    if held and desire not in (Desire.none, CLEAR_DESIRE) and desire != self.last_desire:
      desire = Desire.none  # held: the desire out stays out, but never a new rising edge (a repeat pulse under way)
    plan.desire = self.last_desire = desire
    # carState or modelV2 failing their checks: no change is offered, and one offered aborts. Not acting: act_on is None,
    # so nothing is offered (an offer at the handback was cancelled above, as handedBack)
    lane_kw = {'left_blindspot': cs.leftBlindspot, 'right_blindspot': cs.rightBlindspot, 'maneuver_id': rf.maneuver_id,
               'inputs_valid': inputs_valid, 'stop_d': self.map_controls.stop_ahead()}
    lane_args = (act_on, rf.distance, lanes, model.meta.laneChangeState, v_ego, cs.leftBlinker, cs.rightBlinker, objects,
                 driver_signals, right_hand_traffic, DT_MDL, paint)
    self.lanes.paint_rule = paint_rule
    lc = self.lanes.update(*lane_args, **lane_kw)
    plan.laneChange, plan.signalPrompt, lane_reason = lc.lane_change, lc.prompt, lc.reason
    plan.laneChangeOffer, plan.laneChangeCountdown, plan.laneChangeAbort = lc.offer, lc.countdown, lc.abort
    plan.laneChangeWait, plan.laneChangeWant = lc.wait, max(min(lc.want, 127), -128)
    committed = self.desires.maneuver_id == rf.maneuver_id and self.desires.started  # this frame's desire included
    if lc.unreachable and act_on is not None and not committed and not self.skip.skipped:
      # a map stop leaves no room to get into the maneuver's lane before it nor after its go: skipped out loud and
      # rerouted (turn_skip.py), never a turn from the wrong lane. Never one whose desire went out: a committed turn goes on
      self.skip.skip(act_on, rf.distance, rf.maneuver_id, v_ego, 'laneUnreachable')
      plan.tooFastToTurn = True
    rule = lc if paint_rule else self.paint_rule_lanes.update(*lane_args, **lane_kw)  # its offers and aborts are never raised
    plan.laneChangePaintRule = rule.lane_change if rule.lane_change != LaneChange.none else rule.prompt
    found = lane_mismatch(act_on, rf.distance, rf.through_lanes, rf.through_distance, lanes, v_ego, right_hand_traffic) \
      if acting else LaneMismatch.none  # handback outranks it, and with the maneuver dropped it would misread the lane
    if found == LaneMismatch.none and acting and self.lanes.lane_unknown:
      found = LaneMismatch.laneUnknown  # no change from a lane it can't place: said instead
    plan.laneMismatch = self._lane_mismatch(found)
    plan.accelCap = decision.accel_cap
    # past a bend or a straight-on step, the turn after it is slowed for already (route_follower.py: ahead), and still
    # once the step is dead-reckoned past, until navd moves on to the turn
    ahead_on = acting and rf.valid and rf.maneuver_id != self.dropped_id
    ahead_cap = turn_speed_cap(rf.ahead, rf.ahead_distance, v_ego, right_hand_traffic) if ahead_on else NO_SPEED_CAP
    plan.speedCap = min(decision.speed_cap, turn_speed_cap(cap_on, rf.distance, v_ego, right_hand_traffic), ahead_cap,
                        NO_SPEED_CAP if self.hold_v is None else self.hold_v)
    plan.shouldStop = decision.should_stop
    if self.post_go is not None and self.post_go.over:
      self.post_go = None
    if self.post_go is not None:  # only ever lowers: a positive accel cap, a speed cap braking at -2.0 at most
      go_accel, go_speed = self.post_go.caps(v_ego)
      plan.accelCap, plan.speedCap = min(plan.accelCap, go_accel), min(plan.speedCap, go_speed)
    if withdrawn:  # standing (under STANDSTILL_SPEED when withdrawn): the speed cap's -2.0 floor at most, never harder
      plan.shouldStop, plan.speedCap = True, 0.
    self.limits = (plan.accelCap, plan.speedCap, plan.shouldStop)
    no_junction_data = maneuver is not None and maneuver.kind in (TURN, KEEP) and not maneuver.geometry
    plan.turnMissed = acting and self.missed_t > 0.
    self.missed_t = max(self.missed_t - DT_MDL, 0.)
    plan.reason = decision.reason or (TURN_MISSED_REASON if plan.turnMissed else "") or lane_reason or mapd.reason or \
      (NO_JUNCTION_DATA_REASON if no_junction_data else "")
    fill_map_control(plan, mapd)
    self.fill_left(plan, decision)
    if skip.skipped and self.skip.reason == 'unprotectedRange' and upcoming is not None:
      ul = plan.unprotectedLeft
      ul.state, ul.rangeM = 'skipped', float(self.left_range_m)
      ul.vLimit = float(upcoming.opposing_speed or DEFAULT_OPPOSING_SPEED)
      ul.rReq = float(reach_at_line(upcoming.opposing_speed, upcoming.opposing_lanes or None))
    if decision.hard_brake:  # a confirmed track's hard stop outranks the map's alerts (scene_policy.py)
      plan.controlAlert = 'hardBrake'
    stalled = self.stalled_in_turn(sm, cs, v_ego, acting, decision, mapd)
    stopped = self.stopped_unheld(sm, cs, v_ego, acting, decision, mapd)
    if withdrawn:
      plan.controlPrompt, plan.reason = 'cantStartTakeControl', NO_MOVE_REASON
    elif stalled:
      plan.controlPrompt, plan.reason = 'stalledTakeControl', STALLED_REASON
    elif stopped == STOPPED_NOTE:
      plan.reason = plan.reason or STOPPED_NOTE
    elif stopped:
      plan.controlPrompt, plan.reason = 'carStoppedTakeControl', stopped
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

  def check_missed(self, rf, v_ego: float, acting: bool) -> None:
    """The turn a hold let go of at standstill is passed (the follower's pass, this update) having turned under
    MISSED_TURN_DEG: missed (module docstring). Read before this frame's DesireTiming update, which drops the turn."""
    d = self.desires
    if not (rf.passed and self.go_id is not None and self.go_id == d.maneuver_id == rf.maneuver_id):
      return
    self.go_id = None
    if acting and not d.done and d.turned_deg < MISSED_TURN_DEG:  # done before the pass: cleared at its angle, taken
      self.missed_t = MISSED_HOLD_S
      self.post_go = None  # missed straight on: not held to the profile's speed down the road
      cloudlog.event("turnpilot_turn_missed", error=True, turned_deg=round(d.turned_deg, 1), v_ego=round(v_ego, 2),
                     distance=round(rf.distance, 1))

  def unprotected_left(self, turn, upcoming, rf, v_ego: float, acting: bool, objects, raw) -> LeftCommand | None:
    """This frame's autonomous unprotected left (unprotected_left.py; left_now) for the route's unprotected turn the car is
    taking (turn, map_controls' tuple), and its command for the hold line's latch. TurnPilotAutoLeft off, or TurnPilot not
    acting (handed back): off, the latch handed back as G1's. Fed the last frame's latch and scene (whether they held)."""
    mode = self.auto_left if acting else OFF
    if not isinstance(turn, tuple) or mode == OFF:
      self.left.update(LeftInput(OFF, rf.maneuver_id, 0, 0., v_ego, False, None, [], 0., None, None), DT_MDL)
      return None if not isinstance(turn, tuple) else LeftCommand(OFF)
    tc = self.map_controls.turn
    held = tc is not None and tc.held and tc.key[1] == int(turn[0])
    scene = self.scene
    inp = LeftInput(mode, rf.maneuver_id, upcoming.side, rf.distance, v_ego, held, raw, objects, self.approach_yaw_deg,
                    upcoming.opposing_speed or None, upcoming.opposing_lanes or None, self.left_range_m,
                    scene_clear=scene.junction is None and not scene.carried, junction_uncertain=rf.junction_spread > UNCERTAIN_M,
                    go_withdrawn=self.go_withdrawn, committed=rf.maneuver_id in (scene.committed_id, scene.released_id),
                    spread_m=rf.junction_spread)
    d = self.left_now = self.left.update(inp, DT_MDL)
    return LeftCommand(mode, d.go, d.creep_m, d.state == 'undecided', d.reason)

  def fill_left(self, plan, decision) -> None:
    """turnpilotPlan.detectorView and unprotectedLeft, and the left's own prompt and alert where the map's say nothing."""
    d = self.left_now
    if d is None:
      return
    plan.detectorView = d.view if d.view != 'none' else 'auto'
    ul = plan.unprotectedLeft
    ul.state, ul.tReq, ul.rReq, ul.vLimit, ul.rangeM = d.state, float(d.t_req), float(d.r_req), float(d.v_limit), float(self.left_range_m)
    ul.clearS, ul.blockers, ul.viewBlocked, ul.crosswalkClear = float(d.clear_s), min(d.blockers, 255), d.view_blocked, d.crosswalk_clear
    ul.creepM, ul.why = float(d.creep_m), d.why or d.verdict
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
    band would turn off the oncoming lane). Counted only moving (YAW_MIN_SPEED). Last frame's desire state."""
    d = self.desires
    pulsed = d.maneuver_id == rf.maneuver_id and d.started
    if maneuver is None or maneuver.kind != TURN or rf.maneuver_id != self._yaw_id or not pulsed:
      self._yaw_id, self.approach_yaw_deg = rf.maneuver_id, 0.
      return
    if v_ego >= YAW_MIN_SPEED:
      self.approach_yaw_deg += math.degrees(yaw_rate * DT_MDL) * maneuver.side

  def go_at_line(self, distance: float) -> bool:
    """A go from the car standing at or short of the route's junction's hold line, distance (the follower's) GO_LINE_M or
    more before it, the scene neither committed to that junction nor released into it by the driver: only such a go is
    withdrawn (no_move). Otherwise the car is in the junction, where stopping is worse than finishing the turn
    (scene_policy.py: committed_past_line); a stop line mapped nearer than GO_LINE_M (a sim sidecar's) is taken so too."""
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
    cloudlog.event("turnpilot.post_go", profile=name, unprotected=left, turned_deg=round(turned_deg, 1), distance=round(distance, 1),
                   at_line=at_line)

  def no_move(self, sm, cs, v_ego: float, acting: bool, decision) -> bool:
    """TurnPilot's go from the hold line (go_at_line) is withdrawn: the car not at STANDSTILL_SPEED no_move_limit after it,
    nothing else holding it (module docstring). Latched until openpilot is disengaged (the driver taking over); read before
    this frame's go starts a new clock (start_post_go). A go in the junction starts none: it is never withdrawn."""
    enabled = sm['carControl'].enabled
    if self.go_withdrawn:
      self.go_withdrawn = enabled
      return self.go_withdrawn
    if self.go_wait_s is None:
      return False
    if v_ego >= STANDSTILL_SPEED or not enabled:  # it pulled away, or the driver took over
      self.go_wait_s = None
      return False
    if acting and not decision.should_stop and not decision.holding and not close_lead(sm['modelV2']) and \
       not cs.gasPressed and not cs.brakePressed:
      self.go_wait_s += DT_MDL
    if self.go_wait_s < self.no_move_limit - 1e-6:
      return False
    cloudlog.event("turnpilot.go_not_moving", error=True, stood_s=round(self.go_wait_s, 1), limit_s=self.no_move_limit,
                   turned_deg=round(self.desires.turned_deg, 1))
    self.go_withdrawn, self.go_wait_s = True, None
    return True

  def stalled_in_turn(self, sm, cs, v_ego: float, acting: bool, decision, mapd) -> bool:
    """The car has stood STALL_PROMPT_S in the turn a hold let go of, with nothing holding it (module docstring): hand
    back. The stop line it left is close behind it, so stopped_unheld would wait NEAR_CONTROL_STAND_S. From STALL_MIN_DEG
    into the turn, or from its start if the go came in the junction (go_in_junction: never withdrawn, never silent)."""
    d, rf = self.desires, self.follower
    in_turn = self.go_id is not None and self.go_id == d.maneuver_id == rf.maneuver_id and \
      (self.go_in_junction or STALL_MIN_DEG <= d.turned_deg) and d.turned_deg <= STALL_MAX_DEG
    stalled = acting and in_turn and sm['carControl'].enabled and v_ego < STANDSTILL_SPEED and not decision.should_stop and \
      not self.go_withdrawn and \
      not decision.holding and not close_lead(sm['modelV2']) and not cs.gasPressed and not cs.brakePressed and \
      mapd.prompt not in TAKE_CONTROL_PROMPTS
    was = self.stall_s > STALL_PROMPT_S
    self.stall_s = self.stall_s + DT_MDL if stalled else 0.
    now = self.stall_s > STALL_PROMPT_S
    if now and not was:
      cloudlog.event("turnpilot.stalled_in_turn", error=True, stood_s=round(self.stall_s, 1), turned_deg=round(d.turned_deg, 1))
    return now

  def stopped_unheld(self, sm, cs, v_ego: float, acting: bool, decision, mapd) -> str:
    """The car standing engaged with nothing holding it (module docstring): '' while it isn't, or not for long; STOPPED_NOTE
    after STOPPED_NOTE_S; the handback's reason after STOPPED_HANDBACK_S, NEAR_CONTROL_STAND_S near a map control."""
    lead = close_lead(sm['modelV2'])
    near = [c for c in list(self.map_controls.tracker.controls.values()) + [self.map_controls.turn]
            if c is not None and abs(c.d) < STOPPED_CONTROL_M]
    unheld = acting and sm['carControl'].enabled and v_ego < STANDSTILL_SPEED and not decision.should_stop and \
      not self.go_withdrawn and \
      not decision.holding and not lead and not cs.gasPressed and not cs.brakePressed and mapd.prompt not in TAKE_CONTROL_PROMPTS
    limit = NEAR_CONTROL_STAND_S if near else STOPPED_HANDBACK_S
    was = self.stopped_s >= limit - 1e-6
    self.stopped_s = self.stopped_s + DT_MDL if unheld else 0.
    if self.stopped_s < limit - 1e-6:
      return STOPPED_NOTE if self.stopped_s >= STOPPED_NOTE_S - 1e-6 else ''
    if not was:
      cloudlog.event("turnpilot.stopped_unheld", error=True, stood_s=round(self.stopped_s, 1), reason=decision.reason,
                     near=[c.kind for c in near])
    return SIGNAL_STOPPED_REASON if any(c.kind == 'signal' for c in near) else STOPPED_REASON

  def map_update(self, sm, v_ego: float, gas_pressed: bool, turn=None, left: LeftCommand | None = None):
    """This frame's MapControlPolicy decision, from navInstruction's controls and turnpilotLights (once a frame), the
    route's unprotected turn (MapControlPolicy.update's turn) and the unprotected left's command for it."""
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
    self.map_now = self.map_controls.update(sm['navInstruction'], sm.updated['navInstruction'], sm.valid['navInstruction'], v_ego,
                                            gas_pressed, lights, DT_MDL, turn=turn, left=left)
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
    """update(), but a disabled plan while openpilot doesn't control speed (on a Tesla, without alpha longitudinal):
    its speed caps would do nothing there, and the turn desires would go out at whatever speed stock ACC holds.
    long_control None: card hasn't written CarParams yet, so it isn't known. A disabled plan while TurnPilot is on
    carries its fault, which selfdrived shows (turnpilotUnavailable): TurnPilot never stops working unannounced.
    It also carries a model fallback's handback, which outranks the fault (see the module docstring): a fallback lasts
    the drive, so it is noted here too, where update() isn't reached.
    disengage_on_accel: the DisengageOnAccelerator param. Off, any gas press no longer disengages (Kellen, 2026-10-28: it
    must), so TurnPilot faults (Fault.disengageOnAccelerator, turnpilotUnavailable) instead of running with it off
    unannounced; the holds in force stay on the faulted plan (fault_limits), so no held car is let go."""
    self.handback_now = self.scene_now = self.map_now = None
    self._rf_ticked = self._yaw_ticked = False
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
        took_over = cs.gasPressed and not sm['carControl'].enabled
        decision = self.scene.update(list(scene.objects) if scene.available else [], scene.available, None, float('inf'),
                                     max(cs.vEgo, 0.), gas_pressed=took_over, extra_stops=stops)
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
  """TurnPilotLeftRangeM, the detector's proven range for oncoming traffic, clamped to 0..LEFT_RANGE_MAX_M; ROAD_RANGE_M (the
  provisional, unmeasured value) if unset, not a number (said) or unknown to this build."""
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


def driver_signals_for(params) -> bool:
  """True unless the car's port signals for openpilot's lane changes; unknown cars prompt the driver."""
  cp = car_params(params)
  return cp is None or cp.brand not in OPENPILOT_SIGNALS_BRANDS


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


def main():
  params = Params()
  pm = messaging.PubMaster(['turnpilotPlan', 'turnpilotScene'])
  # no route, no detector (TurnPilotEnabled off, no pkl), no light source (only the sim's oracle publishes one; the
  # camera's lights ride in turnpilotDetections, shadow unless TurnPilotCameraLights is addOnly)
  optional = ['navInstruction', 'turnpilotDetections', 'turnpilotLights']
  sm = messaging.SubMaster(['carState', 'carControl', 'navInstruction', 'modelV2', 'turnpilotDetections', 'extrinsicsCalibration',
                            'deviceState', 'narrowRoadCameraState', 'turnpilotLights'],
                           ignore_alive=optional, ignore_avg_freq=optional, ignore_valid=optional)
  rk = Ratekeeper(1. / DT_MDL, print_delay_threshold=None)
  tp = TurnPilot()
  tracker = None
  enabled = shadow = paint_rule = False
  driver_signals, right_hand_traffic, disengage_on_accel = True, True, True
  long_control: bool | None = None
  invalid_frames = PLAN_INPUT_GRACE_FRAMES
  while True:
    sm.update(0)
    if sm.frame % int(1. / DT_MDL) == 0:
      enabled = params.get_bool("TurnPilotEnabled")
      shadow = params.get_bool("TurnPilotShadowMode")
      paint_rule = lane_paint_active(params)
      right_hand_traffic = not params.get_bool("IsRhdDetected")
      driver_signals = driver_signals_for(params)
      long_control = long_control_for(params)
      disengage_on_accel = params.get_bool("DisengageOnAccelerator")
      tp.oracle_ok = oracle_accepted(os.environ, read_bool(params, "TurnPilotSimLightOracle", False))
      tp.yellow_decel = yellow_decel_for(params)
      tp.camera_mode = camera_lights_mode(params)
      tp.post_go_profile = post_go_profile_for(params, "TurnPilotPostGoProfile", POST_GO_DEFAULT)
      tp.left_go_profile = post_go_profile_for(params, "TurnPilotLeftGoProfile", LEFT_GO_PROFILE)
      tp.no_move_s = no_move_for(params)
      tp.auto_left = auto_left_for(params)
      tp.left_range_m = left_range_for(params)
      if params.get("ChestnutActive") is False:  # None: no big model expected this drive
        tp.big_model_failed(enabled)

    scene_msg = messaging.new_message('turnpilotScene')
    try:
      if tracker is None and sm.seen['deviceState'] and sm.seen['narrowRoadCameraState']:
        device_type = str(sm['deviceState'].deviceType)
        tracker = SceneTracker(DEVICE_CAMERAS[(device_type, str(sm['narrowRoadCameraState'].sensor))], device_type)
      if tracker is not None:
        tracker.update(sm['turnpilotDetections'] if sm.seen['turnpilotDetections'] else None, sm['extrinsicsCalibration'],
                       sm['carState'], sm['modelV2'])
        tracker.fill_scene(scene_msg.turnpilotScene)
    except Exception:
      log_error("turnpilotd.scene_failed")
      tracker, scene_msg = None, messaging.new_message('turnpilotScene')  # unavailable: as with no detector, lefts hold
    pm.send('turnpilotScene', scene_msg)

    scene = scene_msg.as_reader().turnpilotScene
    paint = lane_paint_from_msg(sm['turnpilotDetections'].lanePaint) if scene.available and sm.seen['turnpilotDetections'] else ()
    # the lane planner takes this frame's checks (an offered change aborts at once); the plan's valid has a grace
    valid = sm.all_checks(['carState', 'modelV2'])
    raw = tracker.raw if tracker is not None and tracker.raw_new else None  # this frame's run, for the unprotected left
    msg = tp.plan(sm, scene, enabled, shadow, driver_signals, right_hand_traffic, paint, paint_rule, valid, long_control=long_control,
                  disengage_on_accel=disengage_on_accel, raw=raw)
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
