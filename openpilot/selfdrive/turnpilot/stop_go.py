"""The stop-sign decider (STOP_SIGNS.md v3 sections 2, 4.5, 4.6, 6, 7.1; SG4a, the core): at a held stop sign, whether and
when the car goes by itself, and its launch across the junction. turnpilotd runs it every frame between the turn tuple and
map_update (StopGo.update), so its release, the end of shouldStop and its go reach one plan.

What it decides from (Frame): the stop control in hand (map_controls.held_control(), keyed on its state object, which a
reroute's adopt carries over, so the same control keeps its decider; O8), or the engaged stop sign ahead within STOP_SWEEP_M
(a queue); its junction (stop_rules.Junction from junctionInfo); the tracker's tracks (its own StopEvidence) and the scene's
objects; the live visibility turnpilotd's junction watch feeds every run (visibility.LiveVisibility); this tick's runs (the
signal-head check); the visibility file, the surveys and the wall clock; the car's speed, acceleration, odometer, yaw and
steering; the model (the takeover predicate: go_override.Takeover, imported, never restated); and G5's go in hand.

Modes (TurnPilotStopSignGo, decision D4). auto and trial act. off and shadow (H6), a value or a param this build doesn't know
(H6, said), auto without step S1's camera fields or without a passing launchVerified (H4; trial skips only the latter), a lens
a consumer reports differently from the file's (refused, loudly: H4), and no current stop-sign sidecar at the junction (H8)
act as shadow: map_controls keeps its 1 s stopSignTakeControl handback exactly as before, the reason saying why, and the
decider computes would_go only (logged once a stop). Acting:

- approach: the engaged stop within STOP_SWEEP_M. Standing in a queue with no lead and short of the line (V1, O2), up to
  ROLLUP_GOES rollUp goes (G5) while the model holds; the map curve rolls the car up, never a stop ahead of a standing car.
- stopped: latched; the full stop's dwell (STOP_DWELL_S, within LINE_M of the stop point). Latched more than LINE_M short (O2)
  is rollUp: the latch's stop point moved on to the stop point on map_controls' creep path (creep), the map curve its cap up
  to ROLLUP_V_MAX, with rollUp goes while the model holds. The arrival is the full stop: a vehicle already standing at another
  approach's line is earlier (V5), in the order first seen stopped; one stopping within TIE_S after it is a tie (V6: yield to
  the right; facing, straight or right before left; another car assumed straight). One standing at its line, or up to
  PARKED_BACK_M back, PARKED_S while others take turns is parked (V18): no rank, never a shield, still an occluder.
- observe: section 2.4's go conditions every frame (go_holds: each one held a Hold with its reason and kind, the wait ledger of
  4.6). All of them for CLEAR_S is the go: map_controls' latch let go (release, cause 'clear', no span release past it), G5's
  go (go_cause stopSign) and the stopGo chime. Waiting is silent (waitingForCross, its reason) with an info chime at
  WAIT_INFO_S and every WAIT_INFO_S after while dynamic; a static wait has STATIC_WAIT_S, then the ladder (7.1): rung 1, the
  creep for a view (2.5, G3: VIEW_CREEP_M steps on the latch's creep path where the walk predicted from there gains the
  binding lane and the creep floor allows it, measured CLEAR_S after each), rung 2, another maneuver synthesized from the
  junction's approaches that certifies here (3.5: a right from a lane that can turn right, straight or a left only from a
  single-lane approach; said, otherTurn, and the route's planned turn published missed once done), rung 3, H3 with the
  reason. A dynamic wait past WAIT_ALT_S takes another maneuver whose every go condition holds now (O12). A person standing
  at a crosswalk's curb holds the go PED_CURB_S, then the car goes at STOP_GO_V_MAX until past that crosswalk (D3, P3, P15).
  A control the route couldn't avoid (junctionInfo.certification
  unavoidable: announced at route start and 300 m before) is H3 "this stop needs you" right after the full stop whatever the
  line reads, and one the route found uncertifiable (avoidance off) is H3 there once the line can't certify it either (7.1).
  The deadlock (4.5 step 5, V9, V10): an earlier arrival that holds its turn DEADLOCK_S with nothing in the
  junction moving gets one signalling creep of DEADLOCK_CREEP_M on the latch's creep path (DEADLOCK_CREEP_V, a G5 go with
  cause creep, never closer than GO_ROOM_M to the closest plausible edge, never while creep_blocked: V12, V13), then the
  order is waived and every other go condition still holds for CLEAR_S; it moving before our commit stops us short and we
  yield (during the creep the latch holds again; during the launch the re-check's capped abort); a creep something else cut
  short (anything moving in the junction, the corner no longer clear: the earlier arrival didn't move) is given back and
  taken again DEADLOCK_S after nothing moves, up to DEADLOCK_CREEP_TRIES creeps an encounter. With nothing moving
  DEADLOCK_S and no creep able to run (another go condition holding the go, or the encounter's creeps spent), the turn is a
  static cause (4.6: only traffic taking turns waits with no bound): STATIC_WAIT_S, then H3. Off the route (O9) the
  maneuver is unknown, the union of the conflict sets: navd's reroute is waited for up to OFFROUTE_WAIT_S from the full stop
  (a dynamic wait, its static causes' clock running on), then the union decides and the ladder runs if it fails.
- going: TurnPilot's cap STOP_GO_V_MAX until the commit point, every object, view and sector re-checked (lane reach on the
  go position's geometry, decision D13; a track known at the go judged by its ground motion since, as bearings from a moving
  car restart its trust; any new one in a conflicting lane, or a raw box no known track explains, withdraws the go); the
  takeover gate (the model must have taken over by the gate point) and G5's window. A withdrawn go is the capped abort (an
  accel cap of max(needed, -2.0) to M_EDGE short of dEdgeMin and a speed cap of 0, the way map_controls._approach stops; never
  scene_policy.stop(confirmed=False), whose exponential stop overruns), then observe again with the rank kept, a new go while
  GO_ROOM_M remains, GO_RETRIES retries, then H5. A model that took over and stands again before commit is the same (O5).
- committed (the band, to the in-lane point): a full stop un-commits (V26: observe, never clearBox); a vehicle that would reach
  our conflict point before we clear its lane plus BAND_ABORT_MARGIN_S is the capped abort only where it ends short of the
  band midpoint, else carried on (said: crossTrafficAfterCommit).
- inLane: carried on, never braked for a cross car (crossTrafficAfterCommit); standing STALL_BOX_S with nothing of TurnPilot's
  holding it, up to CLEAR_BOX_GOES clearBox goes once the lanes left to cross pass their object tests (stalledInJunction),
  then H5.
- The views: stopSweep from STOP_SWEEP_M before the control to the commit point (turnpilotd's speed gate on it), stopSweepGo
  from the commit point until the rear bumper is past the farthest conflicting lane or the car has turned 60 deg.

Vision (TurnPilotStopSignGo=vision; owner decision 2026-10-30 18:20, "vision based go"; tp/vision-go's spec): every piece of
the live machinery above runs, and only the certification gates are bypassed: the survey (and its caps), the validation
stamp, S1's camera fields (the noMask zero) and launchVerified. In their place the vision gate (geometry, skew, unknown legs,
an override, the side views on for the drive, the light at the line: certify._assess_vision) and a reach floor per lane
(stop_rules.reach_needed_vision on the walk's raw reach, PV1's optimistic row, no mask needed). Every lane is judged as
possibly not stopping (the tracked test for every track, no on-site downgrade), a car at its line on an approach the map says
stops is a shield (stop_evidence's vision_shield), and the go comes after VISION_CLEAR_S, announced VISION_ANNOUNCE_S before
("Going"). A conflicting approach over 25 mph or of unknown speed is the long look (owner direction 2026-10-30 19:15-19:55):
its floor judged at 25 mph, the window VISION_LONG_CLEAR_S, the creep to the best view first, "Look both ways" announced. A
gate that fails before the first go is the 1 s handback saying which ("Stop sign: side cameras off: take control", H9); after
it, a held fault (H9), never shadow (C14). ALWAYS_DRIVE's amendments V1-V10 hold under vision alone (each marked V<n> below);
the other four modes are as they were (tests/test_stop_go_modes_golden.py).

Faults (7.1), each stopSignTakeControl with its reason, the car held (by the latch, or by the decider's own stop once the latch
has let go): H1 a needed view without a run for DET_FAULT_S (held: cameraPaused for FAULT_GRACE_S first; moving before commit,
the capped abort first), H2 the position unknown (no or a failed junctionInfo, navd's fixes spread over 2 x POS_ERR_M on the
last 20 m before the latch, or the latch more than H2_LINE_M from a surveyed stop point with no queue; held: junctionUncertain
for FAULT_GRACE_S first), H3, H5, H7 (a person in the 0-3 m front zone not
seen leaving, across views, FRONT_ZONE_MAX_S). Every hold on a standing car is scene_policy.stop(confirmed=False); every abort
is the capped stop; nothing here asks for less than -2.0 m/s^2. The decider survives faults as the latch does (turnpilotd
rebuilds it only in reset(detector=True)); the driver disengaging during the launch ends its encounter (O6, O7).

A one-way cross road (G9, V22): only its legal approach conflicts for cars, and its illegal side (a leg cars only leave by,
stop_rules.wrong_way_lanes) is watched: a track there not yet trusted, or coming toward our path, blocks the go (a driver
going the wrong way, or a cyclist, on a contraflow bike lane or not); during the launch any new track there, or a known one
the tracker's ground motion brings nearer, withdraws it, and in the band it is the in-band rule's threat.

Evidence on site (4.4, decision D11): a motor vehicle through a stopping approach's line at DOWNGRADE_V or more (its speed
trusted, never under it in the DOWNGRADE_WINDOW_M before; never a bicycle) makes that approach no for the rest of the
encounter at once (go_holds: "cross traffic doesn't stop", a static wait, then H3; before commit a go out is withdrawn for it,
never once committed), and on_downgrade (turnpilotd) writes it to TurnPilotJunctionOverrides with its evidence; a vehicle that
stops at its line and goes on is a stop event, logged only.

Phantom clears are what the go conditions are about: no detections in a lane says nothing (visibility.py's positive walk), an
untrusted track is never stopping, a raw box blocks before it is a track, a stale view resets CLEAR_S, the shield needs a
vehicle seen stopped on every cycle at a surveyed stopping approach, and the junction must certify by routing/certify.assess
(navd's own function; its reach check replaced by the live walk), with the side views live if its reach relied on them.
"""
import math
from collections import deque
from dataclasses import dataclass, field, replace
from typing import Any

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.go_override import GO_WINDOW_S, Takeover
from openpilot.selfdrive.turnpilot.map_controls import CREEP_AT_M, CREEP_SPEED, CROSSED_CAUSES, is_stop
from openpilot.selfdrive.turnpilot.maps import surveys as survey_file
from openpilot.selfdrive.turnpilot.routing import certify as ce
from openpilot.selfdrive.turnpilot.routing.junctions import survey_approach
from openpilot.selfdrive.turnpilot.scene_policy import PATH_HALF_WIDTH_M, STOP_SPEED, Stop, stop
from openpilot.selfdrive.turnpilot.stop_evidence import APPROACH_V, DOWNGRADE_V, DOWNGRADE_WINDOW_M, StopEvidence, class_name, junction_lanes
from openpilot.selfdrive.turnpilot.turn_speed import NO_SPEED_CAP

MODES = ce.MODES                # off, shadow, trial, auto, vision (TurnPilotStopSignGo)
MODE_DEFAULT = 'auto'           # decision D4: acting only with its data, else as shadow, said
ACTING = ce.ACTING
VISION = ce.VISION

# section 9
STOP_DWELL_S, LINE_M = 1.0, 3.0
STAND_V = 0.1                   # m/s: standing (a full stop)
DET_FAULT_S, FAULT_GRACE_S = 2.0, 10.0
LEAD_GO_M = 8.
BIKE_PASS_MEMORY_S = 10.
HIDE_S = 2.0
TIE_S = 1.0
PARKED_S = 10.                  # s a vehicle stands at its line, or 3-15 m back, while others take turns: parked (4.5 step 2)
PARKED_BACK_M = 15.             # m back from its line a stopped vehicle may stand and still be watched for it
DEADLOCK_S = 3.0                # s an earlier arrival may hold its turn, nothing in the junction moving, before we take ours
DEADLOCK_CREEP_M = 0.5          # m of the one signalling creep (4.5 step 5; map_controls' creep path at CREEP_SPEED)
DEADLOCK_CREEP_V = min(0.4, CREEP_SPEED)  # m/s, its cap: the creep path's limit or under, slow enough to end 0.5 m on (8.1)
DEADLOCK_CYCLES = 1             # creeps an encounter takes before the order is waived (the turn passes to us after one)
DEADLOCK_CREEP_TRIES = 3        # creeps an encounter may start: one something else cut short is given back, up to this many
DEADLOCK_CREEP_MAX_S = GO_WINDOW_S + 1.  # s the creep may take (G5's window, a second to stop): then it is over, moved or not
CREEP_LANE_HOLDS = ('reach', 'raw', 'u2', 'tracked', 'wrongWay', 'stale', 'hide')  # a creep in hand isn't ended by these
VIEW_CREEP_M = 0.5              # m a creep for a view takes (the ladder's rung 1, 2.5, row G3), on the latch's creep path...
VIEW_CREEP_V = DEADLOCK_CREEP_V  # ...at the deadlock creep's cap, its target short by the same run-on (a bounded move)
VIEW_CREEP_GAIN_M = 0.25        # m the binding lane's R_seen less R_req must be predicted to gain from the step, or measured to
CORNER_M = 15.                  # m down a cross lane a long vehicle coming toward our path is at the corner (no creep, V13)...
CREEP_LOOK_S = 5.               # ...or will be within this (its speed toward us): the creep and the next CLEAR_S
FRONT_ZONE_M, FRONT_ZONE_MAX_S = 3.0, 30.
FRONT_ZONE_HALF_W = 1.2         # m either side of the axis the front zone reaches (a 1.85 m car and a margin)
FRONT_EDGE_M = 0.6              # m inside its side edge a track that ended there ended at the edge...
FRONT_LEAVE_V = 0.5             # ...moving sideways over this...
FRONT_LEAVE_S = 1.0             # ...and a person track starting outside on that side within this: the same person, left
EXIT_ROOM_M = 4.7 + 2.0         # the car and 2 m past the exit crosswalk
CROSSWALK_M = 3.0               # m a crosswalk is deep
EXIT_TURN_M = 10.               # m a right turn's path runs from the cross road's near edge to the exit leg's crosswalk
CAR_LEN_M = 4.7
BOX_HOLD_M = 6.                 # any box overlapping 0-6 m ahead of the bumper holds the go (V13)
ROLLUP_GOES = 4
ROLLUP_V_MAX = 3.0              # m/s: a roll-up's cap is the map curve alone, up to this
ROLLUP_WAIT_S = 15.             # s standing short of the line with no lead within LEAD_GO_M, its rollUp goes spent: H5 (4.6)
GO_RETRIES = 2
UNCOMMIT_STOP_S = 0.5
# m to spare short of dEdgeMin for a turn signal veto of the go (DESIGN phase4/blinker 6.6): turnpilotd reads this frame's
# withdrawable and skips the turn on it, and the decider reads the veto a frame later (Frame.signal_veto), the car v*dt on and
# its abort grown by (a T + (v + a T) a / A_ABORT) dt: 0.23 m at STOP_GO_V_MAX under a 1.5 m/s^2 launch, so four times that for
# a launch past it or a step in the position. Inside the margin the go is the turn's: a veto finishes it, said
VETO_MARGIN_M = 1.0
BAND_ABORT_MARGIN_S = 1.0
STALL_BOX_S, CLEAR_BOX_GOES = 2.0, 2
STATIC_WAIT_S, WAIT_INFO_S, CAL_WAIT_S = 15., 60., 30.
WAIT_ALT_S = 45.                # s of a wait (from the arrival) after which another maneuver whose go holds now is taken (O12)
OFFROUTE_WAIT_S = 10.           # s off the route a stopped car waits for navd's reroute to give it a maneuver (3.5, O9)
INFO_CHIME_S = 2.0              # s the info chime's flag (stopGo.chime) is up
ALERT_S = 2.0                   # s an alert of the decider's is held (hooks.ABORT_HOLD_S)
STOP_SWEEP_M = 30.
SIGNAL_HEAD_RUNS, SIGNAL_HEAD_OF = 3, 5
SIGNAL_SPAN_M = 8.              # m either side of our path the junction's region reaches, for a class-9 head
YAW_GO_DEG = 3.                 # deg the car may be yawed off its approach at a go (2.4 item 3)
STEER_GO_DEG = 10.              # deg of steering wheel angle still straight
H2_SPREAD_M = 2. * sr.POS_ERR_M
H2_LINE_M = sr.POS_ERR_M + 1.   # m the latch may lie from a surveyed stop point with no queue to explain it (7.1 H2)
CROSS_DONE_YAW_DEG = 60.
DRIVE_OFF_M, DRIVE_OFF_V = 2., 2.  # the driver drove a decider's own hold off (map_controls' DRIVE_OFF_M, COMMIT_SPEED)
PED_STILL_V = 0.4               # m/s: a person moving slower stands (D3's measure, as a speed)
PED_STILL_M, PED_STILL_S = 0.4, 1.0  # m of displacement in the last s under which a person stands still (D3; the arrival clause)
PED_ARRIVE_MARGIN_S = 2.0       # s our rear bumper must clear a crosswalk on our path before a person heading for it arrives
PED_V_UNTRUSTED = 2.0           # m/s a person is taken to walk at until their speed is trusted (VEL_TRUST_S, 5 samples)
PED_HISTORY_S = 2.0             # s of each person's ground positions kept (the arrival clause's fit)
RAW_PED_RUNS, RAW_PED_OF = 2, 3  # a raw person box counts for the arrival clause seen on 2 of the last 3 runs of its view
RAW_PED_SAME_M = 1.5            # m: a raw box this near a person track, or a box of another run, is the same person
PED_RANGE_BAND = 0.3            # a person's ranging band, plus or minus this of range
PED_CURB_S = 5.                 # s a person standing at a crosswalk's curb holds the go (decision D3, rows P3, P15)
CURB_M = 1.0                    # m past a crosswalk's end (onto the sidewalk) its curb reaches
BAND_KS = (1. - PED_RANGE_BAND, 1. - PED_RANGE_BAND / 2., 1., 1. + PED_RANGE_BAND / 2., 1. + PED_RANGE_BAND)  # along the ray
RAW_PERSON_CONF = 0.35
MOVING_V = 0.5                  # m/s: a vehicle moving
RAW_KNOWN_DEG = 3.              # deg a raw box may lie outside a known track's live span and still be its box (raw_known)
UPGRADE_V = 0.5                 # m/s or less, within UPGRADE_BEFORE_M before its stop point to UPGRADE_PAST_M past it, then on:
UPGRADE_BEFORE_M, UPGRADE_PAST_M = 8., 3.  # a stop event on that approach, logged only (4.4's upgrade, never a tier)
UPGRADE_LEAD_M = 10.            # m: with another vehicle that close ahead of it in its lane it stopped for that one, not its sign
HALF_LEFT_M = 1.85 + sr.LANE_W_MAX   # our half of the road plus one lane, left of our axis (RCW 46.61.235)
HALF_RIGHT_M = 1.85 + 3.0       # ...and right of it to the curb
BOX_LEFT_M, BOX_RIGHT_M = 1.85 + sr.LANE_W_MAX + 1., 1.85 + 1.  # the junction box: our road's width, a metre either side
LARGE = ('truck', 'bus')
MOTOR = ('car', 'truck', 'bus', 'motorcycle')
CYCLISTS = ('cyclist', 'bicycle')
PERSON = 'pedestrian'
SIDE_LIVE = ('running', 'noRoom')  # sideViewStatus with the side views run (the speed gate's)
SIDE_OFF = ('unset', 'off', 'disabled')  # sideViewStatus with no side views for the drive (navd's runtime input, side_views_live)
SIDE_VIEWS = ('sideLeft', 'sideRight')
FORWARD = ('road', 'full')
WAITING = ('stopped', 'rollUp', 'observe')
IDLE = ('off', 'approach', 'done')  # an encounter another stop control ahead may take over (StopGo.takes)

STOP_SIGN_REASON = "Stop sign: take control to proceed"  # map_controls' handback (H6), kept as it was
# junctionInfo.certification of a stop the route couldn't avoid (routing/avoid.py): announced, then handed back at the full stop
UNAVOIDABLE = 'unavoidable'
ROUTE_KNOWN_REASON = "this stop needs you"
ALT_VETOED_REASON = "turn cancelled"  # H3's detail: a turn signal veto of the other turn (Frame.signal_veto with self.alt)
VIEW_CREEP_REASON = "Creeping for view"
STALLED_REASON = "Stalled in junction"
ALT_REASON = {'right': "Turning right instead", 'left': "Turning left instead", 'straight': "Going straight instead"}
# go conditions the same for every maneuver at the junction (the other-turn rung reads them from this frame's own evaluation)
SHARED_HOLDS = ('signal', 'yaw', 'camera', 'stale', 'calibration', 'box', 'near', 'nearRaw', 'hide', 'lead', 'front', 'turn',
                'deadlockCreep')
REASONS = {
  'H1': "Cross-traffic camera unavailable: take control",
  'H2': "Stop sign position uncertain: take control",
  'H4camera': "Stop-sign go off: no camera measurements",
  'H4launch': "Stop-sign go off: launch not verified",
  'H4lens': "Stop-sign go off: camera lens mismatch",
  'H5': "Can't start: take control",
  'H6': STOP_SIGN_REASON,
  'H6unknown': "Stop-sign go off: its param unknown to this build",
  'H7': "Something may be in front of the car: take control",
  'H8': "Stop-sign map data unavailable",
  'H9': "Stop-sign go off: take control",  # vision's gate (its reason says which: StopGo.gate_reason)
}
DYNAMIC, STATIC, FAULT = 'dynamic', 'static', 'fault'

# vision (module docstring): ALWAYS_DRIVE's amendments, under vision alone
VISION_YAW_GO_DEG = 8.          # V8: deg of yaw off the approach still straight enough for a go (the walk's rotation bound takes it in)
VISION_STEER_GO_DEG = 30.       # V8: deg of steering wheel angle
VISION_STALL_H5_S = 30.         # V4: s standing in the junction past the in-lane point, no lift taken over, before H5 (no hold)
VISION_BOX_LIFT_M = 2.25        # V4: m all of a stand's clearBox lifts may move the car, until the model's takeover predicate
VISION_ROLLUP_PROGRESS_M = 1.0  # V3: m a roll-up's goes must move the car for their count to start over (progress)
TAKEOVER_WITHDRAWALS = ('gate', 'window', 'stoodAgain')  # V3: the model not taking over, which still meets GO_RETRIES
LOOK_REASON = "Look both ways"  # the long look's line before its go
GOING_REASON = "Going"
NO_LIFT_REASON = "signal at the junction: the car's own go"  # V7
VISION_GATE_TEXT = {'table': "no visibility file", 'sideViews': "side cameras off", 'H8': "map data unavailable",
                    'lens': "camera lens mismatch"}


@dataclass
class Frame:
  """One frame of what the decider reads (turnpilotd fills it; the tests build it)."""
  control: Any = None            # map_controls.held_control(): the latched stop control (TrackedControl), or None
  approach: Any = None           # the engaged stop sign within STOP_SWEEP_M not yet held, or None
  junction: Any = None           # stop_rules.Junction of the control in hand (junction_watch.junction_from_info)
  route_cert: str = 'unchecked'  # its junctionInfo.certification at route time ('uncertifiable': the route couldn't avoid it)
  maneuver: str = 'straight'     # at the control: straight, left, right, unknown (off the route)
  objects: tuple = ()            # the scene's tracks (TrackedObject: trackId, cls, x, y, vx, vy)
  tracks: tuple = ()             # the tracker's Tracks (stop_evidence's input)
  runs: tuple = ()               # this tick's runs (view key, camera time, RawFrame, sectors)
  live: Any = None               # visibility.LiveVisibility, fed every run
  vis: Any = None                # visibility_table.Visibility, or None
  surveys: Any = None            # maps.surveys' entries
  wall_t: float | None = None    # the wall clock while valid
  side_status: str = 'unset'     # turnpilotDetections.sideViewStatus, latest
  lenses: dict = field(default_factory=dict)  # consumer: the Lens.spec it reports (the tracker, the side views)
  position_spread: float = 0.    # m, navd's fixes of the control less the odometer, over the last 20 m before the latch (H2)
  v_ego: float = 0.
  a_ego: float = 0.
  odo_m: float = 0.              # turnpilotd's odometer, m (faults included)
  yaw20_deg: float = 0.          # the car's heading change over its last 20 m (its yaw off its approach)
  approach_yaw_deg: float = 0.   # its yaw into the route's turn since the turn's desire (the union rule)
  turned_deg: float = 0.         # its yaw since the decider's go (the crossing cycle's end)
  steer_deg: float = 0.
  desire_out: bool = False       # a turn desire has gone out for the maneuver at this control
  model_v2: Any = None
  lead_m: float | None = None    # the model's lead, m, while it has one (prob over its threshold)
  t_abort: float = sr.t_abort(0.15)
  enabled: bool = True           # carControl.enabled
  right_blindspot: bool = False
  tp_holding: bool = False       # something else of TurnPilot's holds the car (a scene stop): no clearBox
  go_active: str = 'none'        # G5's go in hand (GoCertifier.cause)
  go_taken_over: bool = False    # ...the last one ended on the model's takeover
  go_end: str = ''               # ...why it ended
  signal_veto: str = ''          # a turn signal veto turnpilotd skipped the turn for last frame on this go while withdrawable
  # maneuver is the union because the route's turn at this control was skipped (turnpilotd: a turn signal veto, too fast, a
  # lane it can't reach), straight on for the rest of the encounter whatever navd says: no reroute is waited for (O9)
  skip_union: bool = False
  mode: str = MODE_DEFAULT
  mode_known: bool = True        # the build's params know TurnPilotStopSignGo, and its value is a mode
  approach_lanes: int = 0        # our approach's lanes toward the junction (navd's lane guidance for a turn here), 0 unknown
  rightmost: bool = False        # the car is in the rightmost lane (the model sees no lane to its right)
  dt: float = DT_MDL


@dataclass
class LaneStatus:
  role: str
  index: int
  approach: int
  r_seen: float
  r_req: float
  binding: str
  shielded: bool


@dataclass
class StopGoDecision:
  release: tuple | None = None    # (control key, 'clear'): map_controls lets its latch go this frame
  stops: list = field(default_factory=list)  # its holds and aborts (scene_policy.Stop), never below -2.0 m/s^2
  speed_cap: float = NO_SPEED_CAP  # the launch's cap (STOP_GO_V_MAX until commit): a plan cap, only ever lowering
  creep: tuple | None = None      # (m past where the latch held, m/s cap): the latch's stop point moved on (a roll-up)
  go_cause: str = 'none'          # G5's go to start this frame (stopSign, rollUp, clearBox), 'none'
  withdraw: str = ''              # G5's go out withdrawn this frame, why ('' none)
  withdrawable: bool = False      # going: a veto read next frame still withdraws the go short of dEdgeMin (VETO_MARGIN_M)
  detector_view: str = 'auto'     # auto, stopSweep (turnpilotd's speed gate on it), stopSweepGo
  latch: str = 'none'             # map_controls' latch: none, shadow (its 1 s handback), wait (held, no handback), fault
  action: str = 'none'            # MapControlStatus.Action name for the control in hand
  reason: str = ''
  alert: str = 'none'
  prompt: str = 'none'            # waitingForCross, stopGo, stopSignTakeControl (its faults), none
  would_go: bool = False
  state: str = 'off'
  ledger: str = 'none'
  fault: str = ''
  chime: bool = False
  lanes: list = field(default_factory=list)  # LaneStatus
  rank: int = 0
  edges: Any = None
  committed: bool = False
  in_lane: bool = False
  retries: int = 0
  crept_m: float = 0.
  side_live: bool = False
  maps_valid: bool = True
  mode: str = 'shadow'
  clear_s: float = 0.
  go_t: float = 0.
  takeover_s: float = -1.
  withdrawn: str = ''
  position_m: float = 0.
  maneuver: str = 'straight'
  holds: list = field(default_factory=list)  # this frame's Holds (the ledger's inputs)
  acting: bool = False            # the mode acts (auto with its data, trial): not as shadow
  alt: dict | None = None         # the ladder's other turn in hand (rung 2, O12): maneuver, side (+1 right), angle, relBearing, toNode
  turn_missed: bool = False       # this frame: the encounter done on the other turn, the route's planned turn missed (navd reroutes)
  log: dict = field(default_factory=dict)


@dataclass
class Hold:
  """One go condition not met: what to say, whether waiting ends it by itself (dynamic), needs a bound (static) or is a
  fault's, and its key (the same cause frame to frame)."""
  key: str
  reason: str
  kind: str = DYNAMIC


def capped_abort(v: float, d_remaining: float, reason: str) -> Stop:
  """The capped stop d_remaining m ahead (map_controls._approach's way): an accel cap of max(needed, -2.0) and a speed cap of 0,
  never a hard stop, shouldStop once slow. A stop that brings the car to rest (Stop.rest): a turn signal veto while it brakes
  is decided as standing at it (turnpilotd's stopping, car_signals.py)."""
  need = -v * v / (2. * max(d_remaining, 0.5))
  return Stop(max(need, -sr.A_ABORT), 0., v < STOP_SPEED, reason, max(d_remaining, 0.), confirmed=False, rest=True)


def hold_here(v: float, reason: str) -> Stop:
  """A hold on a standing car: scene_policy.stop(confirmed=False) at its own position (shouldStop, a speed cap of 0)."""
  return stop(v, 0., reason, confirmed=False)


def side_views_live(status: str) -> bool:
  """turnpilotPlan.stopGo.sideViewsLive, navd's runtime input (STOP_SIGNS.md 3.2): the side views there for the drive, whatever
  the sweep and the speed gate are doing; navd re-runs its avoidance when it changes, so never a per-stop cycle."""
  return status not in SIDE_OFF


def mode_for(value) -> tuple[str, bool]:
  """TurnPilotStopSignGo's value as (mode, known): unset is the default; anything else acts as shadow, said by the caller."""
  value = value.decode(errors='replace') if isinstance(value, bytes) else value
  if value is None:
    return MODE_DEFAULT, True
  return (value, True) if value in MODES else ('shadow', False)


def file_lens(vis) -> str:
  """The lens spec every consumer must report: visibility.json's fit as scene.ranging.wide_lens builds it (Lens.spec)."""
  lens = vis.raw.get('lens') if vis is not None else None
  if not lens:
    return ''
  k = float(lens.get('k') or 0.) if lens.get('model') == 'equisolid' else 0.
  return f"{'equidistant' if k == 0. else 'equisolid'} f={float(lens['f'])} k={float(k)} visibility.json"


class StopGo:
  def __init__(self):
    self.evidence = StopEvidence()
    self._said: set = set()
    self._cert_key = None
    self._cert = None
    self._certs: dict = {}         # the other turns' verdicts, keyed as certified() keys its own
    self.reset()

  # ---------------------------------------------------------------------------------------------------------- encounter

  def reset(self) -> None:
    """No stop in hand: everything of the encounter cleared."""
    self.token = None              # the control's state object (adopt carries it): the encounter's identity
    self.key = None                # its key now
    self.control = None            # the control itself, last seen (turnpilotd reads its junction info once it's gone)
    self.state = 'off'
    self.junction: sr.Junction | None = None
    self.lanes_all: list = []
    self.watch_lanes: list = []    # a one-way's illegal side, watched for a track coming the wrong way (stop_rules.wrong_way_lanes)
    self.entry = None
    self.lane_edge = None
    self.edges = sr.edge_band(13., None)
    self.setback = 1.9
    self.evidence.reset()
    self.ev: dict = {}
    self.t = 0.                    # s on the decider's clock this encounter
    self.pos_ref, self.odo_ref = 0., 0.
    self.latched = False           # the latch held the car last frame (fault_stops leaves a latched hold to it)
    self.dwell_s = 0.
    self.stand_s = 0.
    self.arrived_t: float | None = None
    self.earlier: dict[int, float] = {}    # trackId: when first seen stopped at its line (an earlier arrival)
    self.later: set = set()
    self.stopped_since: dict[int, float] = {}
    self.gone: set = set()                 # earlier arrivals seen leaving their line
    self.standing: dict[int, float] = {}   # trackId: since when it stands at its line or up to PARKED_BACK_M back (V18)
    self.went_t: dict[int, float] = {}     # trackId: when it last went on through its line (a turn taken)
    self.parked: set = set()               # standing PARKED_S while others took turns: no rank, never a shield, an occluder
    self.rank = 0
    self.turn_wait_s = 0.
    self.route_wait_s = 0.          # s waited off the route for navd's maneuver (OFFROUTE_WAIT_S)
    self.creep: dict | None = None  # the deadlock's signalling creep in hand: its target, start, the earlier arrivals then
    self.creeps = 0                 # creeps taken this encounter (DEADLOCK_CYCLES)
    self.creep_tries = 0            # creeps started this encounter, given back or not (DEADLOCK_CREEP_TRIES)
    self.order_waived = False       # the deadlock passed the turn to us: go condition 5 waived, and only it
    self.creep_end = None           # (why, creep): the creep just ended (its G5 go withdrawn)
    self.view_creep: dict | None = None  # the ladder's creep for a view (rung 1): phase creep, measure or over, its target...
    self.view_steps = 0             # creeps for a view taken this encounter
    self.alt: dict | None = None    # the other turn taken (7.1 rung 2, O12): the decider's maneuver from then on
    self._shared: list = []         # this frame's holds that are the same for every maneuver (SHARED_HOLDS)...
    self._shared_t = -1.            # ...and the decider's clock when they were read (go_holds' early returns read none)
    self.clear_s = 0.
    self.clear_for = ''            # the maneuver clear_s counts for: a new one (a reroute, a skipped turn) counts from 0
    self.static_s = 0.
    self.dynamic_s = 0.
    self.next_chime = WAIT_INFO_S
    self.chime_t = 0.
    self.cal_wait_s = 0.
    self.view_age = dict.fromkeys(FORWARD + SIDE_VIEWS, 0.)  # s since each view's last run (the encounter's start a grace)
    self.needed: set = set()
    self.paused_s = 0.
    self.h2_s = 0.
    self.line_fault = ''           # the latch and the surveyed stop point disagree (H2's third trigger), why
    self.front: dict[int, dict] = {}       # person trackId: in the front zone, its y and vy, lost
    self.front_s = 0.
    self.front_ended: list = []            # (t, y) of front-zone tracks that ended at its side moving sideways
    self.peds: dict[int, deque] = {}       # person trackId: (t, x ground, y) the last PED_HISTORY_S (the arrival clause)
    self.raw_peds: dict[str, deque] = {}   # view key: the person boxes' (x ground, y) of its last RAW_PED_OF runs
    self.ped_carry_on = False              # the launch carries on across the near crosswalk (the arrival clause, 2.6)
    self.curb: dict[int, dict] = {}        # person trackId at a crosswalk's curb, still toward the road (D3): since, which, until
    self.curb_until: float | None = None   # m past the stop point the go's STOP_GO_V_MAX holds to (D3: past that crosswalk)
    self.bike_t: float | None = None
    self.large_left: dict[str, float] = {}  # lane role: when a large vehicle last left it
    self.large_seen: dict[int, str] = {}
    self.lights = deque(maxlen=SIGNAL_HEAD_OF)
    self.signal_seen = False
    self.downgraded: dict[int, dict] = {}  # id(approach): the evidence that it doesn't stop (4.4: no for this encounter)
    self.stop_events: dict[int, dict] = {}  # trackId: its stop at its line seen, logged (4.4's upgrade: a log only)
    self.retries = 0
    self.went = False              # a stopSign go has gone out this encounter (its turn's pulse with it)
    self.rollups = 0
    self.short_s = 0.              # s standing short of the line, no lead (the roll-up's bound)
    self.clear_boxes = 0
    self.stall_s = 0.
    self.uncommit_s = 0.
    self.go: dict | None = None            # the go in hand: its position, clock, lane reach and known tracks at the go
    self.aborting: dict | None = None      # the capped abort in hand: its end point (m past the stop point)
    self.fault = ''
    self.fault_reason = ''
    self.withdrawn = ''
    self.takeover = Takeover()
    self.takeover_s = -1.
    self.would_go = False
    self.would_go_logged = False
    self.alert_t, self.alert = 0., 'none'
    self.crept_m = 0.
    self.rollup_target: float | None = None
    self.last_lanes: list = []
    self.disengaged_odo: float | None = None
    # vision (module docstring)
    self.vision = False             # TurnPilotStopSignGo=vision this encounter
    self.gate = None                # (key, text, verdict) the vision gate failed on, None passing
    self.gate_reason = ''           # its handback's words ("Stop sign: side cameras off: take control")
    self.gate_logged = None         # the gate's last logged result
    self.need_s = sr.CLEAR_S        # s the go conditions must hold for the go
    self.long_look: list = []       # (role, limit) of the conflicting approaches over 25 mph or of unknown speed
    self.announced = False          # this go's line and chime said
    self.vision_top = ''            # the top hold's key last logged (stop_go_vision_wait)
    self.vision_creep_tried = False  # V9: the creep for a view started on a reach-only wait
    self.look_crept = False         # the long look's creep to the best view tried
    self.no_lift = False            # V7: a signal head here: the go releases the latch with no G5 lift
    self.no_lift_said = False       # ...its first go said, the later ones silent (the info chime)
    self.box_lift_m = 0.            # V4: m the stand's clearBox lifts moved the car since the model's last takeover
    self.box_stand_s = 0.           # V4: s standing in the junction since the model's last takeover
    self.box_takeover = Takeover()
    self.fault_in_lane = False      # V4: the fault raised past the in-lane point: no hold there
    self.rollup_odo: float | None = None  # V3: the odometer at the roll-ups' count's start
    self.takeover_fails = 0         # V3: goes withdrawn because the model didn't take over (GO_RETRIES bounds these alone)
    self.box_next = STALL_BOX_S     # V4: s of the stand at which the next clearBox may go

  def _take(self, ctl, f: Frame) -> None:
    """A new encounter with control ctl (held, or engaged ahead)."""
    self.reset()
    self.token, self.key, self.control = ctl.state, ctl.key, ctl
    self.junction = f.junction
    if f.vis is not None and getattr(f.vis, 'setback', None) is not None:
      self.setback = float(f.vis.setback)
    self._set_lanes(f, f.mode == VISION)
    j = self.junction
    cloudlog.event("turnpilot.stop_go_control", key=list(ctl.key), d=round(float(ctl.d), 2), held=bool(getattr(ctl, 'held', False)),
                   junction=j.status if j is not None else 'none', osmNodeId=j.osm_node_id if j is not None else 0)

  def _set_lanes(self, f: Frame, vision: bool) -> None:
    """The encounter's lanes and evidence. Under vision (module docstring) no survey is read (the edge band unsurveyed, every
    line DEFAULT_LINE_M), every lane is built not stopping and stop_evidence's vision shield is on."""
    self.vision = vision
    j = self.junction
    if j is None or j.status != 'ok':
      return
    entries = list(getattr(f.surveys, 'entries', f.surveys) or []) if not vision else []
    self.entry = survey_file.match(entries, j.osm_node_id, (j.lat, j.lon)) if entries else None
    ours = survey_approach(self.entry, j.our_way, j.heading_in) if self.entry is not None else None
    self.lane_edge = (ours or {}).get('laneEdgeM')
    self.edges = sr.edge_band(j.stop_point_distance, ce.cross_lanes(j), self.lane_edge)
    self.lanes_all = junction_lanes(j, 'unknown', self.setback, self.lane_edge)
    if vision:
      self.lanes_all = [ce.vision_lane(ln) for ln in self.lanes_all]
    self.watch_lanes = sr.wrong_way_lanes(j, self.edges, self.setback)
    lines = {}
    for ln in self.lanes_all + self.watch_lanes:
      a = ln.approach
      sv = survey_approach(self.entry, a.way_id, (a.bearing + 180.) % 360.) if self.entry is not None and a is not None else None
      line = (sv or {}).get('stopPointM') or (sv or {}).get('lineM')
      if line:
        lines[ln.role] = float(line)
    if vision:
      self.evidence.set_junction(self.lanes_all + self.watch_lanes, sr.angle_bound(self.edges.nominal is not None), lines, vision_shield=True)
    else:
      self.evidence.set_junction(self.lanes_all + self.watch_lanes, sr.angle_bound(self.edges.nominal is not None), lines)

  # ------------------------------------------------------------------------------------------------------------- update

  def takes(self, ctl, ahead) -> bool:
    """Whether this frame starts a new encounter: a stop control the latch holds now always does; one engaged ahead only
    while the encounter in hand is idle (none, still approaching, or done). Once the latch has let go the decider's own hold
    or its handback is all that holds the car (observe, ladder), and while it launches it is crossing: another stop sign
    within STOP_SWEEP_M (a frontage stop 15-30 m on, a divided road's far line) waits until this one is done."""
    cur = ctl or ahead
    return cur is not None and cur.state is not self.token and (ctl is not None or self.state in IDLE)

  def update(self, f: Frame) -> StopGoDecision:
    # a stop sign, or a yield that became one (yieldStop, map_controls: STOP_SIGNS.md 2.7, row C5): its go a stopSign go
    ctl = f.control if is_stop(f.control) else None
    ahead = f.approach if is_stop(f.approach) else None
    cur = ctl or ahead
    if self.takes(ctl, ahead):
      self._take(cur, f)
    elif cur is None and self.token is not None and self.state in ('approach', 'done') and ctl is None:
      self.reset()
    if self.token is None:
      return StopGoDecision(state='off', mode=self._mode(f)[0])
    if cur is not None and cur.state is self.token:
      self.key, self.control = cur.key, cur
    if (f.mode == VISION) != self.vision and self.state in WAITING + ('approach', 'off'):
      self._set_lanes(f, f.mode == VISION)  # the mode changed before any go: the lanes as the mode now judges them
    self.t += f.dt
    if self.alt is not None and f.signal_veto and not self.alt.get('vetoed') and self.state not in ('committed', 'inLane', 'done', 'ladder'):
      # a turn signal veto of the other turn before it starts (turnpilotd, with its lamp armed: the driver's lamp the other way,
      # hazards, our lamp put out): never taken. A go out for it is withdrawn (_going: signalVeto, its capped abort) and the car,
      # standing, is handed back (_observe: H3), held: the route's maneuver here is not retried, nor another turn taken in its
      # place (DESIGN phase4/blinker 6.6 for the turn the ladder made, which tp/blinker's vetoes predate)
      self.alt['vetoed'] = f.signal_veto
      cloudlog.event("turnpilot.stop_go_other_turn_vetoed", error=True, key=list(self.key), maneuver=self.alt['maneuver'],
                     veto=f.signal_veto, state=self.state)
    if self.alt is not None:
      f = replace(f, maneuver=self.alt['maneuver'])  # the other turn decides from here on (7.1 rung 2)
    d = StopGoDecision(maneuver=f.maneuver, edges=self.edges)
    mode, why = self._mode(f)
    d.mode = mode
    acting = mode in ACTING and not why
    pos = self._position(f, ctl, ahead)
    d.position_m = pos
    self._feed(f, pos)
    if self.state == 'off':
      self.state = 'stopped' if ctl is not None else 'approach'
    if self.state == 'approach' and ctl is not None:
      self.state, self.dwell_s = 'stopped', 0.
    self._driver(f, d, ctl)
    if self.state == 'approach':
      self._approach(f, d, pos, acting)
    elif self.state == 'stopped':
      self._stopped(f, d, ctl, pos, acting, why)
    elif self.state == 'rollUp':
      self._rollup(f, d, ctl, pos, acting, why)
    elif self.state == 'observe':
      self._observe(f, d, ctl, pos, acting, why)
    elif self.state == 'going':
      self._going(f, d, pos)
    elif self.state == 'committed':
      self._committed(f, d, pos)
    elif self.state == 'inLane':
      self._inlane(f, d, pos)
    elif self.state == 'ladder':
      self._ladder(f, d, ctl)
    self._finish(f, d, acting, why)
    return d

  def _driver(self, f: Frame, d: StopGoDecision, ctl) -> None:
    """The driver taking over (openpilot disengaged: a pedal, O6 and O7). During the launch the encounter is theirs at once; a
    hold of the decider's own (the latch already let go) ends once they drive off, or at once when it was the car's motion
    under them past the line that let the latch go (map_controls.CROSSED_CAUSES, owner decision (a), 2026-10-30): never a
    hold of the decider's own in the crossing lanes on a re-engage, the crossing said by the latch (map_controls.Driven: its
    take-control until through), never silent."""
    if ctl is None and self.state in ('observe', 'ladder') and \
       getattr(getattr(self.control, 'state', None), 'cause', '') in CROSSED_CAUSES:
      cloudlog.event("turnpilot.stop_go_driver", key=list(self.key), state=self.state, crossed=True)
      self.go = self.aborting = None
      self.state = 'done'
      return
    if f.enabled:
      self.disengaged_odo = None
      return
    if self.state in ('going', 'committed', 'inLane'):
      cloudlog.event("turnpilot.stop_go_driver", key=list(self.key), state=self.state)
      d.withdraw = 'driver' if f.go_active != 'none' else ''
      self.go = self.aborting = None
      self.state = 'done'
      return
    if ctl is None and self.state in ('observe', 'ladder'):
      self.disengaged_odo = f.odo_m if self.disengaged_odo is None else self.disengaged_odo
      if f.odo_m - self.disengaged_odo > DRIVE_OFF_M and f.v_ego > DRIVE_OFF_V:
        cloudlog.event("turnpilot.stop_go_driver", key=list(self.key), state=self.state, drove_off=True)
        self.go = self.aborting = None
        self.state = 'done'

  # ---------------------------------------------------------------------------------------------------------- the mode

  def _mode(self, f: Frame) -> tuple[str, str]:
    """(the mode, why it acts as shadow: an H4, H6 or H8 key; '' when acting)."""
    if not f.mode_known:
      self._say('mode', "turnpilot stop_go: TurnPilotStopSignGo unknown to this build or not a mode: acting as shadow")
      return 'shadow', 'H6unknown'
    if f.mode == VISION:
      return VISION, self._vision_mode(f)
    if f.mode not in ACTING:
      return f.mode if f.mode in MODES else 'shadow', 'H6'
    vis = f.vis
    if vis is None or not vis.camera_ready()[0]:
      return f.mode, 'H4camera'
    if f.mode == 'auto' and not vis.launch_passed:
      return f.mode, 'H4launch'
    want = file_lens(vis)
    bad = {k: s for k, s in (f.lenses or {}).items() if s and s != want}
    if bad:
      self._say('lens', f"turnpilot stop_go: lens mismatch with visibility.json's {want!r}: {bad}: acting as shadow")
      return f.mode, 'H4lens'
    j = f.junction if f.junction is not None else self.junction
    if j is not None and j.status == 'ok' and not j.sidecar:
      return f.mode, 'H8'
    return f.mode, ''

  def _what(self) -> str:
    """'Yield' for a yield that became a stop (yieldStop), else 'Stop sign': the first words of vision's handbacks (V10)."""
    return 'Yield' if getattr(self.control, 'kind', 'stop') == 'yield' else 'Stop sign'

  def _vision_gate(self, f: Frame):
    """Vision's gate (module docstring) for the encounter in hand: (key, text, verdict) of the first check that fails, None
    when it passes. The file (none: table), a lens a consumer reports against a fitted file's (lens), no current sidecar at the
    junction (H8), the side views off for the drive (sideViews), then certify's vision checks at the line (geometry, skew,
    unknown legs, an override, the light); certify's route-time reach is the live walk's here (never a gate)."""
    if f.vis is None:
      return 'table', VISION_GATE_TEXT['table'], None
    want = file_lens(f.vis)
    bad = {k: s for k, s in (f.lenses or {}).items() if s and s != want} if want else {}
    if bad:
      self._say('lens', f"turnpilot stop_go: lens mismatch with visibility.json's {want!r}: {bad}: vision off")
      return 'lens', VISION_GATE_TEXT['lens'], None
    j = self.junction
    if self.token is None or j is None or j.status != 'ok':
      return None
    if not j.sidecar:
      return 'H8', VISION_GATE_TEXT['H8'], None
    if not side_views_live(f.side_status):
      return 'sideViews', VISION_GATE_TEXT['sideViews'], None
    v = self.certified(f)
    if v.certification != 'certifiable' and v.reason != 'reach':
      return v.reason, v.detail or v.reason, v
    return None

  def _vision_mode(self, f: Frame) -> str:
    """_mode's why under vision: before the first go a gate that fails is its handback (H9 with its reason, H8 at a junction
    without its sidecar), said once in the log; after it '' (go_holds holds the car with the fault: C14)."""
    gate = self._vision_gate(f)
    self.gate = gate
    j = self.junction
    if self.token is not None and j is not None:
      self.long_look = ce.long_look(j, f.maneuver, f.approach_yaw_deg) if j.status == 'ok' else []
      logged = (gate[0] if gate else '', tuple(r for r, _ in self.long_look), f.maneuver)
      if logged != self.gate_logged:
        self.gate_logged = logged
        v = gate[2] if gate else None
        cloudlog.event("turnpilot.stop_go_vision_gate", key=list(self.key) if self.key else [], verdict='fail' if gate else 'pass',
                       reason=gate[0] if gate else '', detail=gate[1] if gate else '', maneuver=f.maneuver, sideViewStatus=f.side_status,
                       sidecar=bool(getattr(j, 'sidecar', False)), longLook=[r for r, _ in self.long_look],
                       approaches=[(sr.effective_role(m.approach), round(m.approach.assumed_limit / sr.MPH, 1))
                                   for m in sr.conflict_movements(j, f.maneuver, f.approach_yaw_deg)] if j.status == 'ok' else [],
                       rows=sorted(v.rows) if v is not None else [], wallT=f.wall_t)
    if gate is None:
      self.gate_reason = ''
      return ''
    key, text, _ = gate
    self.gate_reason = REASONS['H8'] if key == 'H8' and self._what() == 'Stop sign' else f"{self._what()}: {text}: take control"
    if self.went:
      return ''  # after the first go: a held fault (go_holds), never shadow
    return 'H8' if key == 'H8' else 'H4lens' if key == 'lens' else 'H9' + key

  def _say(self, key: str, message: str) -> None:
    if key not in self._said:
      self._said.add(key)
      cloudlog.error(message)

  # ------------------------------------------------------------------------------------------------------ the evidence

  def _position(self, f: Frame, ctl, ahead) -> float:
    """m the car's bumper is past the stop point: the held control's frozen distance and what the latch has moved since, the
    engaged one's distance ahead, and once neither holds it, the odometer from the last of those."""
    if ctl is not None:
      pos = -float(ctl.d) + float(getattr(ctl.state, 'moved_m', 0.))
    elif ahead is not None and self.state in ('off', 'approach'):
      pos = -float(ahead.d)
    else:
      return self.pos_ref + (f.odo_m - self.odo_ref)
    self.pos_ref, self.odo_ref = pos, f.odo_m
    return pos

  def _feed(self, f: Frame, pos: float) -> None:
    """The tracks into the decider's evidence (once a frame), the runs into the views' ages and the signal-head check."""
    self.evidence.observe(f.tracks or (), d_stop=-pos)
    # only the tracks the tracker publishes now (it coasts a missed one SWEEP_COAST_S): StopEvidence ages a history against
    # its newest sample, so with no tracks left nothing would age out. An earlier arrival that vanishes stays waiting
    # (_turn_holds), never presumed gone
    live = {getattr(t, 'track_id', None) for t in f.tracks or ()}
    self.ev = {tid: e for tid, e in self.evidence.evaluate().items() if tid in live}
    self._parked()
    for tid, e in self.ev.items():
      if e.stopped_at_line:
        self.stopped_since.setdefault(tid, self.t - e.stopped_s)
      else:
        self.stopped_since.pop(tid, None)
    self._on_site(f, pos)
    for k in self.view_age:
      self.view_age[k] += f.dt
    for key, _t, raw, _sectors in f.runs or ():
      if key in self.view_age:
        self.view_age[key] = 0.
      if key in FORWARD and raw is not None:
        self.lights.append(self._head_seen(raw, pos))
    if not self.signal_seen and sum(self.lights) >= SIGNAL_HEAD_RUNS:
      self.signal_seen = True
      j = self.junction
      cloudlog.event("turnpilot.stop_go_signal_head", error=True, key=list(self.key), osmNodeId=j.osm_node_id if j is not None else 0,
                     runs=list(self.lights))
      self.on_signal_seen(j)
    seen_peds = set()
    for o in f.objects or ():
      if class_name(o.cls) == PERSON:
        h = self.peds.setdefault(o.trackId, deque())
        h.append((self.t, float(o.x) + pos, float(o.y)))
        while h and h[0][0] < self.t - PED_HISTORY_S - 1e-9:
          h.popleft()
        seen_peds.add(o.trackId)
    for tid in [k for k, h in self.peds.items() if k not in seen_peds and (not h or h[-1][0] < self.t - PED_STILL_S)]:
      del self.peds[tid]
    self._curbs(f, pos)
    for key, _t, raw, _sectors in f.runs or ():
      if raw is not None and key != 'far':
        boxes = [(float(b.xy[0]) + pos, float(b.xy[1])) for b in raw.boxes
                 if b.cls == PERSON and b.conf >= RAW_PERSON_CONF and b.xy is not None]
        self.raw_peds.setdefault(key, deque(maxlen=RAW_PED_OF)).append(boxes)
    for o in f.objects or ():
      # a cyclist beside or behind us on the right (P7): a right turn waits until it has reappeared ahead or BIKE_PASS_MEMORY_S
      if class_name(o.cls) in CYCLISTS and float(o.y) < -0.9:
        if float(o.x) < self.setback + 3.:
          self.bike_t = self.t
        elif self.bike_t is not None and float(o.x) > self.setback + 3.:
          self.bike_t = None

  def _parked(self) -> None:
    """4.5 step 2, row V18: a vehicle standing at its line (its line region) or up to PARKED_BACK_M back that doesn't move for
    PARKED_S while others take turns (another vehicle went on through its line meanwhile) is parked: it loses its rank (no
    more an earlier arrival we wait for), is never its lane's shield (one parked at the corner may not be in the lane at all,
    and the lane behind it is unseen), and stays an occluder (visibility caps reach behind any track's box). Moving again
    (its speed trusted over SHIELD_V) ends it. Untrusted (our own creep restarts every track's bearings) changes nothing."""
    for tid, e in self.ev.items():
      if e.v_trusted and e.v_nom > MOVING_V and e.front_from_line_m > sr.SHIELD_PAST_M:
        self.went_t[tid] = self.t
      if not e.v_trusted:
        continue
      if e.v_hi < sr.SHIELD_V and -PARKED_BACK_M <= e.front_from_line_m <= sr.SHIELD_PAST_M:
        self.standing.setdefault(tid, self.t - e.stopped_s)
      else:
        self.standing.pop(tid, None)
        if tid in self.parked:
          self.parked.discard(tid)
          cloudlog.event("turnpilot.stop_go_unparked", key=list(self.key), track=int(tid))
    for tid, since in self.standing.items():
      if tid not in self.parked and self.t - since >= PARKED_S - 1e-6 and any(t >= since for o, t in self.went_t.items() if o != tid):
        self.parked.add(tid)
        cloudlog.event("turnpilot.stop_go_parked", key=list(self.key), track=int(tid), standing_s=round(self.t - since, 1),
                       earlier=tid in self.earlier)
    for tid in self.parked:
      e = self.ev.get(tid)
      if e is not None and e.shield:
        self.ev[tid] = replace(e, shield=False)

  def on_signal_seen(self, j) -> None:
    """A signal head at the junction (2.4 item 9, C20): turnpilotd's hook writes TurnPilotJunctionOverrides (signalSeen)."""

  def on_downgrade(self, j, approach, evidence: dict) -> None:
    """An approach downgraded on site (4.4, decision D11): turnpilotd's hook writes TurnPilotJunctionOverrides (no) with
    the evidence, so navd avoids it from the next lookup on until Kellen clears it (route_audit --clear)."""

  def _on_site(self, f: Frame, pos: float) -> None:
    """4.4's evidence on site, every frame from the tracks' evidence. Downgrade: a motor vehicle (never a bicycle: those are
    non-stopping already) on a stopping approach, its speed trusted, through its line at DOWNGRADE_V or more without dropping
    under it in the DOWNGRADE_WINDOW_M before (stop_evidence's through at speed, at the lane's nominal offset: at a surveyed
    junction the surveyed one). That approach is no for the rest of this encounter at once (go_holds: cross traffic doesn't
    stop), the safe direction for a runner as for a wrong survey, and on_downgrade writes it. The heading clause (a change under
    DOWNGRADE_HEADING_DEG 20 deg) isn't applied: the bearing ray assumes the lane line and ground-plane heading at the side is
    30 deg or more off with the ranging's 1.5-2.5x error, so a turning car through at speed downgrades too, the safe
    direction, its evidence saying 'heading unmeasured'. Upgrade (_stop_event): a log only."""
    j = self.junction
    for tid, e in self.ev.items():
      ln = e.lane
      if ln is None or ln.movement == sr.WRONG_WAY or ln.approach is None:
        continue
      a = ln.approach
      if e.through_at_speed and e.cls in MOTOR and ln.stopping and id(a) not in self.downgraded:
        ev = {'kind': 'no', 'trackId': int(tid), 'cls': e.cls, 'role': sr.effective_role(a), 'wayId': int(a.way_id),
              'bearing': round(float(a.bearing), 1), 'speedMs': round(float(e.through_v), 2), 'minSpeedMs': round(float(e.through_min_v), 2),
              'thresholdMs': DOWNGRADE_V, 'windowM': DOWNGRADE_WINDOW_M, 'heading': 'unmeasured', 'cameraTimeS': round(float(e.t), 3),
              'view': e.view, 'osmNodeId': int(j.osm_node_id) if j is not None else 0, 'log': 'turnpilot.stop_go_downgrade'}
        self.downgraded[id(a)] = ev
        self._cert_key = None
        cloudlog.event("turnpilot.stop_go_downgrade", error=True, key=list(self.key), state=self.state, pos=round(pos, 2), **ev)
        self.on_downgrade(j, a, ev)
      self._stop_event(f, tid, e, pos)

  def _stop_event(self, f: Frame, tid: int, e, pos: float) -> None:
    """4.4's upgrade, logged only: a vehicle that reaches UPGRADE_V or less within UPGRADE_BEFORE_M before to UPGRADE_PAST_M
    past its stop point, no vehicle within UPGRADE_LEAD_M ahead of it in its lane and no person in front of it (the junction
    and the crosswalks on its side), and then goes on, is a stop event on its approach for route_audit to show. It never
    changes a tier: while the car waits, courtesy stops and stops for people it can't see would count."""
    st = self.stop_events.setdefault(tid, {'stopped': False, 'logged': False})
    if st['logged'] or e.cls not in MOTOR:
      return
    if e.v_trusted and e.v_nom <= UPGRADE_V and -UPGRADE_BEFORE_M <= e.front_from_line_m <= UPGRADE_PAST_M:
      lead = any(o is not e and (o.role, o.index) == (e.role, e.index) and 0. < o.front_from_line_m - e.front_from_line_m <= UPGRADE_LEAD_M
                 for o in self.ev.values())
      x0, x1 = self._x(self.edges.d_min, pos) - CROSSWALK_M - 1., self._x(self._far_edge(), pos) + CROSSWALK_M + 1.
      side = 1. if e.role == 'left' else -1. if e.role == 'right' else 0.
      person = any(class_name(o.cls) == PERSON and x0 <= float(o.x) <= x1 and (side == 0. or float(o.y) * side >= 0.)
                   for o in f.objects or ())
      if not lead and not person:
        st['stopped'] = True
    elif st['stopped'] and ((e.v_trusted and e.v_nom > MOVING_V) or e.front_from_line_m > UPGRADE_PAST_M):
      st['logged'] = True
      cloudlog.event("turnpilot.stop_go_stop_event", key=list(self.key), track=int(tid), cls=e.cls, role=e.role,
                     wayId=int(e.lane.approach.way_id) if e.lane is not None and e.lane.approach is not None else 0, cameraTimeS=round(float(e.t), 3))

  def _head_seen(self, raw, pos: float) -> bool:
    """A class-9 head in the junction's region on this forward run: above the horizon, its bearing within SIGNAL_SPAN_M of
    our path at the cross road's near edge (RawFrame.lights: (bearing deg, elevation deg, conf))."""
    lights = getattr(raw, 'lights', None) or ()
    if not lights:
      return False
    s = max(self.edges.d_min - pos + self.setback, 1.)
    span = math.degrees(math.atan2(SIGNAL_SPAN_M, s))
    return any(elev > 0. and abs(bearing) <= span for bearing, elev, _conf in lights)

  # ------------------------------------------------------------------------------------------------------------ states

  def _approach(self, f: Frame, d: StopGoDecision, pos: float, acting: bool) -> None:
    standing = f.v_ego < STAND_V
    self.stand_s = self.stand_s + f.dt if standing else 0.
    d.detector_view = 'stopSweep' if -pos <= STOP_SWEEP_M else 'auto'
    free = f.lead_m is None or f.lead_m > LEAD_GO_M
    if acting and -pos > LINE_M and self._short_wait(f, d, pos, standing and free and not f.tp_holding):
      d.stops.append(hold_here(f.v_ego, self.fault_reason))  # never latched: the decider's own hold from this frame
      return
    # a queue's lead gone, the model holding the car short of the line (V1, O2): a rollUp go (G5), the map curve rolling it up
    self._rollup_progress(f)
    if acting and standing and self.stand_s >= STOP_DWELL_S and free and -pos > LINE_M and \
       f.go_active == 'none' and self.rollups < ROLLUP_GOES and f.enabled:
      self.rollups += 1
      self.stand_s = 0.
      d.go_cause = 'rollUp'
      cloudlog.event("turnpilot.stop_go_rollup", key=list(self.key), d=round(-pos, 2), n=self.rollups, latched=False)

  def _rollup_progress(self, f: Frame) -> None:
    """V3 under vision: the roll-ups repeat while they move the car (VISION_ROLLUP_PROGRESS_M since their count started, the
    count starting over), so ROLLUP_GOES and its H5 bound only a roll-up that makes no progress (TurnPilotGoOverride off, a
    model that never follows): never a silent stand."""
    if not self.vision:
      return
    if self.rollup_odo is None or self.rollups == 0:
      self.rollup_odo = f.odo_m
    elif f.odo_m - self.rollup_odo >= VISION_ROLLUP_PROGRESS_M:
      self.rollups, self.rollup_odo = 0, f.odo_m

  def _short_wait(self, f: Frame, d: StopGoDecision, pos: float, waiting: bool) -> bool:
    """4.6's bound on a roll-up (V1, O2): standing short of the line with nothing ahead within LEAD_GO_M is waiting on the
    roll-up's goes, a static wait said as such; ROLLUP_WAIT_S of it, the ROLLUP_GOES goes spent (TurnPilotGoOverride off, or
    each go blocked) and none out, is H5. A lead within LEAD_GO_M, or the car moving, starts it over. True: the fault."""
    self.short_s = self.short_s + f.dt if waiting and f.enabled else 0.
    if self.short_s <= 0.:
      return False
    d.ledger, d.reason = STATIC, "Rolling up to the stop sign"
    if self.short_s >= ROLLUP_WAIT_S - 1e-6 and self.rollups >= ROLLUP_GOES and f.go_active == 'none':
      self._fault('H5', f, d, f"{self.rollups} roll-up goes left the car {-pos:.1f} m short of the line")
      return True
    return False

  def _stopped(self, f, d, ctl, pos, acting, why) -> None:
    d.latch, d.detector_view, d.action, d.reason = 'wait', 'stopSweep', 'held', "Stopped at stop sign"
    j = self.junction
    if acting and ctl is not None and not self.line_fault and j is not None and j.stop_point_source == 'survey' and \
       abs(pos) > H2_LINE_M and (f.lead_m is None or f.lead_m > LEAD_GO_M):
      # H2's third trigger (7.1): at a surveyed stop point, the latch more than POS_ERR_M + 1 m from it with no queue to
      # explain it says navd's position and where the car stopped (its line) disagree, and the band's geometry rests on
      # that position: never a roll-up onto it. Observe, the position fault's grace (junctionUncertain), then H2
      self.line_fault = f"latched {abs(pos):.1f} m {'short of' if pos < 0. else 'past'} the surveyed stop point"
      cloudlog.event("turnpilot.stop_go_line_disagrees", error=True, key=list(self.key), pos=round(pos, 2), osmNodeId=j.osm_node_id)
      self.state, self.clear_s = 'observe', 0.
      self._observe(f, d, ctl, pos, acting, why)
      return
    if acting and ctl is not None and -pos > LINE_M:
      # latched short of the stop point (O2): its stop point moved on to the stop point, rolling up on the map curve
      self.rollup_target = float(ctl.d)
      self.state, self.stand_s = 'rollUp', 0.
      cloudlog.event("turnpilot.stop_go_rollup", key=list(self.key), d=round(-pos, 2), latched=True)
      self._rollup(f, d, ctl, pos, acting, why)
      return
    standing = f.v_ego < STAND_V
    self.dwell_s = self.dwell_s + f.dt if standing and abs(pos) <= LINE_M + 1e-6 else 0.
    if self.dwell_s >= STOP_DWELL_S - 1e-6:
      # the full stop made (2.4 item 1): the arrival, and the other go conditions are counted from the next frame
      self._arrive()
      self.state, self.clear_s = 'observe', 0.
      d.prompt, d.action, d.reason = 'waitingForCross', 'waitingCross', "Stopped at stop sign: looking for cross traffic"
      if acting and f.route_cert == UNAVOIDABLE and not self.vision:  # V1: under vision the line's own gate decides
        # 7.1: a control navd's avoidance couldn't route around (routing/avoid.py), announced at route start and 300 m before
        # ("Stop sign ahead needs you"), skips the ladder's rungs: the handback right after the 1 s full stop, whatever the
        # line reads (the driver was told this stop needs them)
        self._fault('H3', f, d, ROUTE_KNOWN_REASON)

  def _rollup(self, f, d, ctl, pos, acting, why) -> None:
    d.latch, d.detector_view, d.action, d.reason = 'wait', 'stopSweep', 'creeping', "Rolling up to the stop sign"
    if ctl is None or not acting or self.rollup_target is None:
      self.state, self.rollup_target, self.dwell_s = 'stopped', None, 0.
      return
    self.crept_m = float(ctl.state.moved_m)
    target = self.rollup_target
    standing = f.v_ego < STAND_V
    if target - self.crept_m <= 0.1 + 1e-6 or (standing and abs(pos) <= LINE_M and self.crept_m > 0.5):
      # there: the full stop again, its dwell from now (the arrival is this stop)
      self.state, self.rollup_target, self.dwell_s, self.short_s = 'stopped', None, 0., 0.
      d.creep = (target, ROLLUP_V_MAX)
      return
    d.creep = (target, ROLLUP_V_MAX)
    if self._short_wait(f, d, pos, standing and (f.lead_m is None or f.lead_m > LEAD_GO_M)):
      d.creep = None
      return
    self.stand_s = self.stand_s + f.dt if standing else 0.
    self._rollup_progress(f)
    if standing and self.stand_s >= STOP_DWELL_S and f.go_active == 'none' and self.rollups < ROLLUP_GOES and f.enabled:
      self.rollups += 1
      self.stand_s = 0.
      d.go_cause = 'rollUp'
      cloudlog.event("turnpilot.stop_go_rollup", key=list(self.key), d=round(-pos, 2), n=self.rollups, latched=True)

  def _arrive(self) -> None:
    """Our arrival, at the full stop: every vehicle standing at another approach's line is earlier (V5)."""
    self.arrived_t = self.t
    for tid, e in self.ev.items():
      if e.stopped_at_line and tid not in self.parked:
        self.earlier[tid] = self.stopped_since.get(tid, self.t)
    cloudlog.event("turnpilot.stop_go_arrived", key=list(self.key), earlier=sorted(self.earlier), t=round(self.t, 2))

  def _observe(self, f, d, ctl, pos, acting, why) -> None:
    d.latch, d.detector_view = 'wait', 'stopSweep'
    d.action = 'waitingCross' if acting else 'held'
    standing = f.v_ego < STAND_V
    if self.aborting is not None:
      d.stops.append(self._abort_stop(f, pos))
      if not standing:
        d.reason = f"Stopping short: {self.aborting['why']}"
        return
      self.aborting = None
      room, need = self.edges.d_min - pos, sr.go_room_m(f.t_abort, right=f.maneuver == 'right')
      if acting and room < need - 1e-6:
        # a withdrawn go come to rest with no room for another (2.6): H5, never a wait from where no go can start
        self._fault('H5', f, d, f"no room for another go: {room:.2f} m to the closest edge, {need:.2f} needed")
        return
    if self.alt is not None and self.alt.get('vetoed'):
      self._fault('H3', f, d, ALT_VETOED_REASON)  # the other turn overruled, the car standing: handed back, held
      return
    if f.maneuver != self.clear_for:
      # another path to judge (navd's reroute while held, or turnpilotd's skip of the turn here: the union, 'unknown'): its
      # go conditions hold CLEAR_S from now, never on the time the old path's lanes were clear
      self.clear_s, self.clear_for = 0., f.maneuver
    holds = self.go_holds(f, pos, launch=False)
    d.holds = holds
    self.clear_s = self.clear_s + f.dt if not holds else 0.
    if self.vision and self.view_creep is not None and self.view_creep.get('phase') == 'creep':
      self.clear_s = 0.  # a creep in hand: the window runs from where it stands
    self._ledger(f, d, holds, acting, pos)
    if self.state != 'observe':
      return
    if self.view_creep is not None:
      self._view_creeping(f, d, holds, pos, ctl, acting, standing)
    elif self.vision and acting and f.enabled and not holds and self.long_look and not self.look_crept and ctl is not None and standing:
      # the long look (module docstring): the creep to the best view first, wherever the walk predicts a step gains the lane
      # with the least margin (on a perpendicular cross road the stop point usually is the best view: no creep), then the window
      self.look_crept = True
      if self._view_creep_start(f, d, holds, pos, look=True):
        self.clear_s = 0.
    if ctl is None:
      d.stops.append(hold_here(f.v_ego, d.reason or "Stopped at stop sign"))  # the decider's own hold: the latch let go already
    # the deadlock's signalling creep (4.5 step 5): the latch's stop point moved on at CREEP_SPEED, and one G5 go (creep) at its
    # start for a model that holds; ended (reached, or we yield), its go withdrawn
    if self.creep is not None and ctl is not None:
      d.creep, d.action = (self.creep['target'], DEADLOCK_CREEP_V), 'creeping'
      if not self.creep['went'] and acting and f.enabled and f.go_active == 'none' and standing:
        self.creep['went'] = True
        d.go_cause = 'creep'
    if self.creep_end is not None:
      if f.go_active == 'creep':
        d.withdraw = self.creep_end[0]
      self.creep_end = None
    need_s = self._need_s()
    # vision's: the window runs from where a creep stands; auto and trial judge it as base did
    creeping = self.vision and self.view_creep is not None and self.view_creep.get('phase') == 'creep'
    go = not holds and self.clear_s >= need_s - 1e-6 and not creeping
    self.would_go = go
    if go and not self.would_go_logged:
      self.would_go_logged = True
      cloudlog.event("turnpilot.stop_go_would_go", key=list(self.key), acting=acting, why=why, t=round(self.t, 2), pos=round(pos, 2))
    if not go or not acting or not f.enabled:
      # openpilot disengaged (the driver's pedal: DisengageOnAccelerator): never a go of TurnPilot's, which would let go of
      # the latch while the driver drives and leave a crossing nobody certified silent (owner decision (b), 2026-10-30).
      # The window goes on counting: re-engaged short of the line, the go comes on it
      if self.vision and acting and f.enabled and not holds and not creeping and \
         self.clear_s >= need_s - sr.VISION_ANNOUNCE_S - 1e-6 and not (self.no_lift and self.no_lift_said):
        self._announce(d, need_s)  # the go's line and chime, VISION_ANNOUNCE_S before it
      elif holds:
        self.announced = False
      return
    room = self.edges.d_min - pos
    need = sr.go_room_m(f.t_abort, right=f.maneuver == 'right')
    if room < need - 1e-6:
      self._fault('H5', f, d, f"no room for a go: {room:.2f} m to the closest edge, {need:.2f} needed")
      return
    self.go = {'pos': pos, 't': self.t, 'reach': {(x.role, x.index): x.r_seen for x in self.last_lanes},
               'tracks': self._known_tracks(f), 'said': not (self.no_lift and self.no_lift_said)}
    self.curb_until = self.curb_cap_until()  # D3: past a curb person's crosswalk at STOP_GO_V_MAX at most
    if self.curb_until is not None:
      cloudlog.event("turnpilot.stop_go_curb_cap", key=list(self.key), until=round(self.curb_until, 2), people=sorted(self.curb))
    self.takeover.reset()
    self.takeover_s, self.uncommit_s = -1., 0.
    self.state, self.clear_s, self.went, self.ped_carry_on = 'going', 0., True, False
    silent = self.no_lift and self.no_lift_said  # V7: a no-lift go after the first, the wait's info chime its only sound
    if not silent:
      self.static_s = self.dynamic_s = 0.
      self.next_chime = WAIT_INFO_S
    d.release = (self.key, 'clear') if ctl is not None else None
    d.go_cause = 'none' if self.no_lift else 'stopSign'  # V7: at a signal head no G5 lift, the model goes or holds by itself
    d.prompt, d.reason, d.action, d.ledger = 'stopGo', GOING_REASON, 'going', 'none'
    if silent:
      d.prompt, d.reason = 'waitingForCross', "Waiting: " + NO_LIFT_REASON
    self.no_lift_said = self.no_lift_said or self.no_lift
    self.announced = False
    d.stops = []
    d.speed_cap = sr.STOP_GO_V_MAX
    d.withdrawable = True  # standing where it was held, GO_ROOM_M or more short of dEdgeMin
    cloudlog.event("turnpilot.stop_go_go", key=list(self.key), pos=round(pos, 2), room=round(room, 2), retries=self.retries,
                   rank=self.rank, lanes=[(x.role, x.index, round(x.r_seen, 1), round(x.r_req, 1), x.shielded) for x in self.last_lanes],
                   vision=self.vision, need_s=need_s, noLift=self.no_lift, longLook=[r for r, _ in self.long_look],
                   margins=[(x.role, x.index, round(x.r_seen - x.r_req, 1)) for x in self.last_lanes])

  def _need_s(self) -> float:
    """s every go condition must hold for the go: VISION_LONG_CLEAR_S for the long look, VISION_CLEAR_S under vision, else
    CLEAR_S."""
    self.need_s = (sr.VISION_LONG_CLEAR_S if self.long_look else sr.VISION_CLEAR_S) if self.vision else sr.CLEAR_S
    return self.need_s

  def _announce(self, d: StopGoDecision, need_s: float) -> None:
    """Vision's go said before it comes (guard 5): the stopGo prompt (its chime) with "Going", or "Look both ways" for the
    long look so the supervisor checks both ways; a hold inside it starts the window again and the wait's words come back."""
    d.prompt, d.reason = 'stopGo', LOOK_REASON if self.long_look else GOING_REASON
    if not self.announced:
      self.announced = True
      cloudlog.event("turnpilot.stop_go_vision_announce", key=list(self.key), clear_s=round(self.clear_s, 2), need_s=need_s,
                     reason=d.reason, longLook=[(r, round(lim / sr.MPH, 1)) for r, lim in self.long_look])

  def _going(self, f: Frame, d: StopGoDecision, pos: float) -> None:
    g = self.go
    d.detector_view, d.action, d.prompt, d.reason = 'stopSweep', 'going', 'stopGo', GOING_REASON
    d.speed_cap = sr.STOP_GO_V_MAX
    d.go_t = self.t - g['t']
    if f.model_v2 is not None and self.takeover.update(f.model_v2, f.dt) and self.takeover_s < 0.:
      self.takeover_s = d.go_t
    if f.go_taken_over and self.takeover_s < 0.:
      self.takeover_s = d.go_t
    taken_over = self.takeover_s >= 0.
    if self.no_lift and not taken_over and not g.get('said', True):
      d.prompt, d.reason = 'waitingForCross', "Waiting: " + NO_LIFT_REASON  # V7: silent until the model goes
    a_now = max(f.a_ego, 0.)
    if f.signal_veto:
      # the driver overruled the turn's signal while this go could still be withdrawn (last frame's withdrawable, turnpilotd
      # skipped the turn on it and cleared its desire): one more thing the go was made on that no longer holds. Withdrawn
      # ahead of the commit check, its capped abort short of dEdgeMin, and observe again on every path (turnpilotd's union,
      # unknown) for the next go, straight on (DESIGN 6.6; B10's rule for the autonomous left, unprotected_left._after_go)
      self._withdraw(f, d, pos, 'signalVeto', "turn cancelled")
      return
    if sr.committed(pos, f.v_ego, f.t_abort, self.edges, a_now):
      self.state, self.uncommit_s = 'committed', 0.
      d.detector_view, d.action, d.speed_cap, d.prompt, d.committed = 'stopSweepGo', 'committed', NO_SPEED_CAP, 'none', True
      cloudlog.event("turnpilot.stop_go_committed", key=list(self.key), pos=round(pos, 2), v=round(f.v_ego, 2), taken_over=taken_over,
                     go_t=round(d.go_t, 2))
      return
    d.withdrawable = sr.abort_distance(f.v_ego, f.t_abort, a_now) + sr.M_EDGE + VETO_MARGIN_M < self.edges.d_min - pos
    holds = self.go_holds(f, pos, launch=True)
    d.holds = holds
    # a person heading for the near crosswalk (the arrival clause, P16): it lies between the stop point and dEdgeMin, so the
    # capped stop targets its earliest plausible near edge less M_EDGE where it can end there at -2.0 or gentler; where it
    # can't, the car carries on across at STOP_GO_V_MAX or less rather than stop on it (scene_policy's in-path stop armed)
    near_ped = [h for h in holds if h.key.startswith('pedArrive') and h.key.endswith(':near')]
    others = [h for h in holds if h not in near_ped]
    why, detail, to = '', '', None
    if others:
      why, detail = others[0].key, others[0].reason
    elif near_ped:
      edge = self.edges.d_min - CROSSWALK_M - sr.M_EDGE
      if not self.ped_carry_on and sr.abort_distance(f.v_ego, f.t_abort, a_now) <= edge - pos:
        why, detail, to = near_ped[0].key, near_ped[0].reason, edge
      elif not self.ped_carry_on:
        self.ped_carry_on = True
        cloudlog.event("turnpilot.stop_go_ped_carry_on", key=list(self.key), pos=round(pos, 2), v=round(f.v_ego, 2), why=near_ped[0].key)
    if why:
      pass  # a go condition failed: withdrawn for it
    elif not taken_over and sr.gate_reached(pos, f.v_ego, f.t_abort, self.edges, a_now):
      why = 'gate'
    elif not taken_over and (d.go_t >= GO_WINDOW_S - 1e-6 or (f.go_active == 'none' and f.go_end == 'window' and not self.no_lift)):
      why = 'window'
    elif taken_over and f.v_ego < STAND_V:
      self.uncommit_s += f.dt
      why = 'stoodAgain' if self.uncommit_s >= UNCOMMIT_STOP_S - 1e-6 else ''
    else:
      self.uncommit_s = 0.
    if why:
      self._withdraw(f, d, pos, why, detail, to)

  def _committed(self, f: Frame, d: StopGoDecision, pos: float) -> None:
    d.detector_view, d.action, d.committed = 'stopSweepGo', 'committed', True
    standing = f.v_ego < STAND_V
    if self.aborting is not None:
      d.stops.append(self._abort_stop(f, pos))
    if sr.in_lane(pos, self.edges) and self.aborting is None:
      self.state = 'inLane'
      cloudlog.event("turnpilot.stop_go_in_lane", key=list(self.key), pos=round(pos, 2), v=round(f.v_ego, 2))
      self._inlane(f, d, pos)
      return
    self.uncommit_s = self.uncommit_s + f.dt if standing else 0.
    if self.uncommit_s >= UNCOMMIT_STOP_S - 1e-6:
      # a full stop in the band un-commits (V26): observe from here, the rank kept, a fresh go only after CLEAR_S, never clearBox
      cloudlog.event("turnpilot.stop_go_uncommitted", key=list(self.key), pos=round(pos, 2), aborted=self.aborting is not None)
      self.aborting = self.go = None
      self.state, self.clear_s = 'observe', 0.
      d.withdraw = 'uncommitted' if f.go_active != 'none' else ''
      d.committed, d.action, d.detector_view = False, 'waitingCross', 'stopSweep'
      d.stops = [hold_here(f.v_ego, "Stopped short of the cross road: looking again")]
      return
    if self.aborting is None:
      threat = self._threat(f, pos)
      if threat:
        mid = sr.band_midpoint(self.edges)
        room = mid - sr.M_EDGE - pos
        if sr.abort_distance(f.v_ego, f.t_abort, max(f.a_ego, 0.)) <= room:
          self.aborting = {'to': mid - sr.M_EDGE, 'why': threat}
          d.withdraw = 'bandAbort'
          d.stops.append(self._abort_stop(f, pos))
          cloudlog.event("turnpilot.stop_go_band_abort", key=list(self.key), pos=round(pos, 2), v=round(f.v_ego, 2), threat=threat,
                         room=round(room, 2))
        else:
          self._after_commit(threat)

  def _inlane(self, f: Frame, d: StopGoDecision, pos: float) -> None:
    d.action, d.committed, d.in_lane, d.detector_view = 'committed', True, True, 'stopSweepGo'
    curb_past = self.curb_until is None or pos > self.curb_until  # D3's cap held until past that crosswalk
    if (pos - CAR_LEN_M > self._far_edge() or abs(f.turned_deg) >= CROSS_DONE_YAW_DEG) and curb_past:
      self.state, self.go = 'done', None
      d.detector_view = 'auto'
      cloudlog.event("turnpilot.stop_go_done", key=list(self.key), pos=round(pos, 2), turned=round(f.turned_deg, 1))
      if self.alt is not None and not self.alt.get('missed'):
        # the other turn taken: the route's planned turn is missed, and navd reroutes at once (turnMissed, 3.5)
        self.alt['missed'] = d.turn_missed = True
        cloudlog.event("turnpilot.stop_go_turn_missed", key=list(self.key), alt=self.alt['maneuver'], planned=self.alt['from'])
      return
    threat = self._threat(f, pos)
    if threat:
      self._after_commit(threat)  # never braked for: stopping in a lane is worse than going on (V8)
    standing = f.v_ego < STAND_V
    free = not f.tp_holding and (f.lead_m is None or f.lead_m > LEAD_GO_M)
    self.stall_s = self.stall_s + f.dt if standing and free else 0.
    if self.stall_s >= STALL_BOX_S - 1e-6:
      d.reason = STALLED_REASON  # said on the status line (control_text) while it lasts
    if self.vision:
      self._vision_stall(f, d, pos, standing, threat)
      return
    if self.stall_s >= STALL_BOX_S - 1e-6 and f.go_active == 'none':
      if self.clear_boxes >= CLEAR_BOX_GOES:
        self._fault('H5', f, d, "stalled in the junction after its clearBox goes")
        return
      if threat:
        return  # the lanes left to cross don't pass their object tests: no clearBox yet
      self.clear_boxes += 1
      self.stall_s = 0.
      d.go_cause = 'clearBox'
      self._show_alert('stalledInJunction')
      cloudlog.event("turnpilot.stop_go_clear_box", key=list(self.key), pos=round(pos, 2), n=self.clear_boxes)

  def _vision_stall(self, f: Frame, d: StopGoDecision, pos: float, standing: bool, threat: str) -> None:
    """V4 past the in-lane point under vision: clearBox goes with no count, each from a standstill once STALL_BOX_S stood with
    nothing of TurnPilot's holding the car and the lanes left to cross passing their object tests, all of them moving the
    car VISION_BOX_LIFT_M at most until the model's takeover predicate holds (which, and only which, starts the budget and the
    clock over: a lift ending at GO_END_SPEED isn't the model following); the stall alert each STALL_BOX_S meanwhile; H5 after
    VISION_STALL_H5_S standing in the junction, with no hold added in the lane (stopping there is worse)."""
    if f.model_v2 is not None and self.box_takeover.update(f.model_v2, f.dt):
      self.box_lift_m = self.box_stand_s = 0.
    if f.go_active == 'clearBox':
      self.box_lift_m += max(f.v_ego, 0.) * f.dt
      if self.box_lift_m >= VISION_BOX_LIFT_M - 1e-6:
        d.withdraw = 'budget'  # the stand's lifts have moved the car VISION_BOX_LIFT_M: the rest is the model's
    self.box_stand_s = self.box_stand_s + f.dt if standing else self.box_stand_s
    if self.box_stand_s >= VISION_STALL_H5_S - 1e-6:
      self.fault_in_lane = True
      self._fault('H5', f, d, f"stalled in the junction {self.box_stand_s:.0f} s")
      return
    if self.stall_s < STALL_BOX_S - 1e-6:
      self.box_next = STALL_BOX_S
      return
    if self.stall_s < self.box_next - 1e-6 or f.go_active != 'none':
      return
    self.box_next = self.stall_s + STALL_BOX_S  # the next try STALL_BOX_S on, standing still
    self._show_alert('stalledInJunction')
    if threat or self.box_lift_m >= VISION_BOX_LIFT_M - 1e-6:
      cloudlog.event("turnpilot.stop_go_clear_box_no", key=list(self.key), pos=round(pos, 2), threat=threat, lifted=round(self.box_lift_m, 2))
      return
    self.clear_boxes += 1
    d.go_cause = 'clearBox'
    cloudlog.event("turnpilot.stop_go_clear_box", key=list(self.key), pos=round(pos, 2), n=self.clear_boxes, lifted=round(self.box_lift_m, 2))

  def _ladder(self, f: Frame, d: StopGoDecision, ctl) -> None:
    """A fault's handback: held, said, until the driver takes over (openpilot disengaged) and drives off."""
    d.latch, d.detector_view, d.action = 'fault', 'stopSweep', 'awaitingDriver'
    d.prompt, d.reason, d.fault, d.ledger = 'stopSignTakeControl', self.fault_reason, self.fault, FAULT
    if ctl is None and not self.fault_in_lane:  # V4: a handback past the in-lane point holds nothing there
      pos = self.pos_ref + (f.odo_m - self.odo_ref)
      if self.aborting is not None and f.v_ego >= STAND_V:
        d.stops.append(self._abort_stop(f, pos))  # still the capped stop while it rolls (never the exponential hold)
      else:
        self.aborting = None
        d.stops.append(hold_here(f.v_ego, self.fault_reason))

  # ------------------------------------------------------------------------------------------------------- go conditions

  def junction_lanes(self, f: Frame) -> list:
    """The maneuver's conflicting lanes from the stop point (2.1): the turn's set and the straight set until UNION_YAW_DEG of
    yaw into it, the right's swing zone, every lane off the route (unknown)."""
    j = self.junction
    if j is None or j.status != 'ok':
      return []
    lanes = junction_lanes(j, f.maneuver, self.setback, self.lane_edge, f.approach_yaw_deg)
    return [ce.vision_lane(ln) for ln in lanes] if self.vision else lanes

  def certified(self, f: Frame, probe: bool = False):
    """routing/certify.assess at the line (the same function navd runs at route time; its mode and launch checks are _mode's):
    the static checks, cached while their inputs stand still."""
    vision = f.mode == VISION
    # under vision the side views' term is the drive's (side_views_live): idle and disabled are both outside SIDE_LIVE
    side = side_views_live(f.side_status) if vision else f.side_status in SIDE_LIVE
    key = (f.maneuver, round(f.approach_yaw_deg), round(max(abs(f.yaw20_deg), sr.ROUTE_YAW_DEG), 1), side,
           None if f.wall_t is None else int(f.wall_t // 60.), id(f.vis), id(f.surveys), id(self.junction), len(self.downgraded), vision)
    rt = ce.Runtime(mode=VISION if vision else 'trial', side_views=side, window=(0., 0.), yaw_deg=f.approach_yaw_deg,
                    yaw_offset_deg=max(abs(f.yaw20_deg), sr.ROUTE_YAW_DEG))
    if probe:
      if key not in self._certs:
        if len(self._certs) > 8:
          self._certs.clear()
        self._certs[key] = ce.assess(self.junction, f.maneuver, f.vis, f.surveys, f.wall_t, rt)
      return self._certs[key]
    if key != self._cert_key:
      self._cert_key, self._cert = key, ce.assess(self.junction, f.maneuver, f.vis, f.surveys, f.wall_t, rt)
    return self._cert

  def go_holds(self, f: Frame, pos: float, launch: bool) -> list[Hold]:
    """Every go condition of 2.4 not met now, a Hold each (the wait ledger reads them). launch: re-checked during the launch
    (2.6): no full stop or turn order, reach on the go position's geometry, tracks known at the go by their ground motion."""
    holds: list[Hold] = []
    j = self.junction
    # off the route (O9, 3.5): the maneuver unknown, the union of the three conflict sets; navd's reroute is waited for up to
    # OFFROUTE_WAIT_S from the full stop (a dynamic wait, said, its static causes' clock running meanwhile), then the union decides.
    # Never for a skipped turn's union (skip_union): the car goes straight on whatever the reroute says, so nothing is gained
    if not launch and f.maneuver == 'unknown' and not f.skip_union and not self.vision and self.route_wait_s < OFFROUTE_WAIT_S - 1e-6:
      self.route_wait_s += f.dt
      holds.append(Hold("route", "the reroute (off the route)", DYNAMIC))
    # H2's triggers, and 10: the junction, the data
    if j is None or j.status != 'ok':
      return holds + [Hold('position', "the junction's position isn't known", FAULT)]
    if f.position_spread > H2_SPREAD_M:
      return holds + [Hold('position', f"its distance read {f.position_spread:.1f} m apart", FAULT)]
    if self.line_fault:
      return holds + [Hold('position', self.line_fault, FAULT)]
    # an approach downgraded on site this encounter (4.4): not stopping, so no maneuver here certifies (the union rule
    # leaves none that conflicts with no approach); before commit a go out is withdrawn for it, never once committed
    if self.downgraded:
      conflicting = {id(m.approach) for m in sr.conflict_movements(j, f.maneuver, f.approach_yaw_deg)}
      if conflicting & set(self.downgraded):
        return holds + [Hold('downgrade', "cross traffic doesn't stop", STATIC)]
    if self.vision:
      # vision's gate (_vision_gate): before the first go _mode made it the 1 s handback; after it a fault, held (C14)
      gate = self._vision_gate(f)
      if gate is not None:
        return holds + [Hold('vision:' + gate[0], gate[1], FAULT)]
    else:
      verdict = self.certified(f)
      if verdict.certification != 'certifiable' and verdict.reason != 'reach':
        return holds + [Hold('cert:' + verdict.reason, f"can't certify ({verdict.reason}: {verdict.detail})" if verdict.detail else
                     f"can't certify ({verdict.reason})", STATIC)]
    # 9 a signal head at the junction (C20); under vision (V7) no hold: the go releases the latch with no G5 lift, the model's
    if self.signal_seen and not self.vision:
      holds.append(Hold('signal', "signal at junction", STATIC))
    self.no_lift = self.vision and self.signal_seen
    # 3 straight on its approach, no turn desire before our first go (our own go's pulse is that go's: after a withdrawn go the
    # yaw and the wheel say whether it curled the car); V8: wider under vision
    yaw_go, steer_go = (VISION_YAW_GO_DEG, VISION_STEER_GO_DEG) if self.vision else (YAW_GO_DEG, STEER_GO_DEG)
    if not launch and (abs(f.yaw20_deg) > yaw_go or abs(f.steer_deg) > steer_go or (f.desire_out and not self.went)):
      holds.append(Hold('yaw', "the car isn't straight on its approach", STATIC))
    # 2 the views the maneuver needs, fresh on camera time (the side views while the speed gate runs them); E7, E8
    holds += self._view_holds(f)
    # 4 every conflicting lane: reach or a shield, its tracks, its raw boxes
    holds += self._lane_holds(f, pos, launch)
    # 6 the box; 7 people, the front zone, the exit; 8 a lead, a right's blind spot and a cyclist passed on the right
    box = self._box_holds(f, pos)
    rest = self._people_holds(f, pos, launch) + self._exit_holds(f, pos)
    if f.lead_m is not None and f.lead_m <= LEAD_GO_M:
      rest.append(Hold('lead', "a queue ahead", DYNAMIC))
    if f.maneuver == 'right':
      if f.right_blindspot:
        rest.append(Hold('blindspot', "something in the right blind spot", DYNAMIC))
      if self.bike_t is not None and self.t - self.bike_t <= BIKE_PASS_MEMORY_S:
        rest.append(Hold('bikePassed', "a cyclist passed on the right", DYNAMIC))
    # 5 our turn, and the deadlock: nothing in the junction moving is its condition, and the creep starts only with every other
    # go condition met (we would go now were it our turn; never a creep toward a person in the crosswalk), and once started
    # ends on any but its lanes' (our own creep restarts their trust: its earlier arrivals judged by ground motion)
    if not launch:
      lanes_only = all(h.key.split(':')[0] in CREEP_LANE_HOLDS for h in box + rest)
      turn = self._turn_holds(f, pos, any(h.kind == DYNAMIC for h in box + rest), not holds and not box and not rest, lanes_only)
      self._shared = [h for h in holds + box + rest + turn if h.key.split(':')[0] in SHARED_HOLDS]
      self._shared_t = self.t
      if any(h.kind == STATIC for h in turn):
        return holds + box + rest + turn  # a standing cause beside it is the wait's reason, and H3's
      holds += turn
    return holds + box + rest

  def _view_holds(self, f: Frame) -> list[Hold]:
    """2.4 item 2, E7, E8: a needed view older than DET_FRESH_S of camera time resets CLEAR_S; none for DET_FAULT_S on the
    decider's clock is the camera paused (held: FAULT_GRACE_S, then H1); the side views' maps waiting for a calibration move,
    a dynamic wait of up to CAL_WAIT_S."""
    out = []
    # under vision the side views are needed whenever they exist for the drive (a view that stops running is stale, then
    # dead, then H1: never dropped from needed), but while their maps wait for a calibration (its own hold and bound)
    side = f.v_ego <= sr.STOP_GO_V_MAX and (f.side_status in SIDE_LIVE or (self.vision and f.side_status not in ('mapsStale', 'noCalibration')))
    needed = set(FORWARD) | (set(SIDE_VIEWS) if side else set())
    for k in needed - self.needed:
      self.view_age[k] = min(self.view_age[k], 0.)  # needed from now: its grace starts now
    self.needed = needed
    live = f.live
    dead = sorted(k for k in needed if self.view_age[k] >= DET_FAULT_S - 1e-6)
    stale = sorted(k for k in needed if live is None or not live.fresh_key(k))
    if dead:
      out.append(Hold('camera', "cross-traffic camera paused (" + ', '.join(dead) + ")", FAULT))
    elif stale:
      out.append(Hold('stale', "views catching up (" + ', '.join(stale) + ")", DYNAMIC))
    if f.side_status == 'mapsStale':
      self.cal_wait_s += f.dt
      out.append(Hold('calibration', "side cameras recalibrating", DYNAMIC if self.cal_wait_s < CAL_WAIT_S else STATIC))
    else:
      self.cal_wait_s = 0.
    return out

  def _lighting(self, f: Frame):
    from openpilot.selfdrive.turnpilot.junction_watch import lighting_at
    return lighting_at(self.junction, f.wall_t, f.vis.veil if f.vis is not None else None)

  def _lane_holds(self, f: Frame, pos: float, launch: bool, record: bool = True) -> list[Hold]:
    """2.4 item 4: each conflicting lane's R_seen at least its R_req (or a qualified shield), its tracks passing U2 (a stopping
    approach's) or the tracked test (another's, and a cyclist on any), and no raw box blocking it. During the launch reach is
    walked on the go position's geometry and held to the smaller of its go-time and live value (D13), shields and the tracks
    known at the go are judged by their ground motion, and any other track or raw box in a lane withdraws the go."""
    holds: list[Hold] = []
    lanes = self.junction_lanes(f)
    seen_lanes: list = []
    if record:
      self.last_lanes = seen_lanes
    if not lanes:
      return [Hold('noLanes', "no conflicting approach to judge by", STATIC)]
    if f.live is None or f.vis is None:
      return [Hold('camera', "no live view", FAULT)]
    at = self.go['pos'] if launch and self.go is not None else pos
    rot = sr.angle_bound(self.edges.nominal is not None, max(abs(f.yaw20_deg), sr.ROUTE_YAW_DEG))
    rep = self._reach_report(f, lanes, at)
    known = self.go['tracks'] if launch and self.go is not None else {}
    still = self._still_spans(f, known) if launch else []
    for ln, seen in zip(lanes, rep.lanes, strict=True):
      if self.vision:
        # the walk's raw reach against the vision floor (D-V1): a car seen in the last T_LAT is the re-check's and the band's
        r_req, r_seen = sr.reach_needed_vision(ln.limit, ln.approach), seen.reach_m
      else:
        r_req = max(seen.need_m - sr.V_CROSS_FACTOR * ln.limit * sr.T_LAT, 0.)
        r_seen = seen.r_seen_m
      if launch and self.go is not None:
        r_seen = min(r_seen, self.go['reach'].get((ln.role, ln.index), r_seen))
      if launch:
        shielded = any(k['shield'] and (k['role'], k['index']) == (ln.role, ln.index) and not self._known_moved(f, tid, k)
                       for tid, k in known.items())
      else:
        shielded = bool(seen.shields)
      idx = next((i for i, a in enumerate(self.junction.approaches) if a is ln.approach), 0)
      seen_lanes.append(LaneStatus(ln.role, ln.index, idx, r_seen, r_req, seen.why, shielded))
      if r_seen + 1e-9 < r_req and not shielded:
        moving = any((e.role, e.index) == (ln.role, ln.index) and e.v_nom > MOVING_V for e in self.ev.values())
        holds.append(Hold(f'reach:{ln.role}{ln.index}', f"can't see far enough {side_word(ln.role)} ({seen.why})",
                          DYNAMIC if moving else STATIC))
      if launch:
        # objects are judged from where the car is now (only reach keeps the go position's geometry, D13)
        raw = [b for b in self._blockers(f, ln, seen.need_m, rot, pos) if not raw_known(b, still)]
      else:
        raw = seen.blockers
      if raw:
        holds.append(Hold(f'raw:{ln.role}{ln.index}', f"something {side_word(ln.role)}", DYNAMIC))
    want = {(ln.role, ln.index): ln for ln in lanes}
    cue = sr.t_cue(max(self.edges.d_max - pos, 0.))
    for tid, e in self.ev.items():
      if e.lane is not None and e.lane.movement == sr.WRONG_WAY:
        if self._wrong_way(f, tid, e, known if launch else None):
          holds.append(Hold(f'wrongWay:{tid}', f"a {e.cls} coming the wrong way {side_word(e.role)}", DYNAMIC))
        continue
      ln = want.get((e.role, e.index))
      if ln is None:
        continue
      if launch:
        if tid not in known:
          holds.append(Hold(f'new:{tid}', f"a {e.cls} {side_word(ln.role)}", DYNAMIC))
        elif self._known_moved(f, tid, known[tid]):
          holds.append(Hold(f'outOfTurn:{tid}', f"a {e.cls} {side_word(ln.role)} moving", DYNAMIC))
        continue
      if ln.stopping and e.cls not in CYCLISTS:
        if e.u2 not in ('stopped', 'far', 'stopping'):
          holds.append(Hold(f'u2:{tid}', f"a {e.cls} {side_word(ln.role)} not stopping", DYNAMIC))
      elif not sr.tracked_ok(sr.TrackFit(e.d_lo, e.v_hi, e.v_trusted), f.maneuver if f.maneuver in ('left',) else 'straight', cue,
                             sr.V_CROSS_FACTOR * ln.limit):
        holds.append(Hold(f'tracked:{tid}', f"a {e.cls} {side_word(ln.role)}", DYNAMIC))
    return holds

  def _reach_report(self, f: Frame, lanes: list, at: float, ahead_m: float = 0.):
    """visibility.LiveVisibility.lanes for lanes from at (m past the stop point), with this frame's lighting, survey, stamp,
    tracks and evidence; ahead_m: predicted from that much further on (the creep for a view, 2.5)."""
    row, low = self._lighting(f)
    stamp = ce.pick_stamp(self.entry, row, self.junction.heading_in) if self.entry is not None and not self.vision else None
    rot = sr.angle_bound(self.edges.nominal is not None, max(abs(f.yaw20_deg), sr.ROUTE_YAW_DEG))
    kw = {'vision': True} if self.vision else {}
    return f.live.lanes(lanes, -(at + ahead_m), f.vis, rot_deg=rot, edges=self.edges, lighting=row if row in ('day', 'sunAhead') else 'night',
                        low_sun_sides=low, entry=None if self.vision else self.entry, stamp=stamp, tracks=f.tracks or (), evidence=self.ev,
                        ahead_m=ahead_m, **kw)

  @staticmethod
  def _blockers(f: Frame, ln, need: float, rot: float, pos: float) -> list:
    """The raw boxes blocking lane ln (visibility.LiveVisibility.blockers), the lane placed from where the car is now."""
    here = replace(ln, s_min=ln.s_min - pos, s_max=ln.s_max - pos)
    height = f.vis.height if f.vis is not None and f.vis.height is not None else 1.3
    return f.live.blockers(here, need, rot, height)

  def _known_tracks(self, f: Frame) -> dict:
    """The conflicting lanes' tracks at the go: their lane, U2 state and shield, their ground point and our odometer then."""
    out = {}
    by_id = {t.track_id: t for t in f.tracks or () if getattr(t, 'x', None) is not None}
    for tid, e in self.ev.items():
      trk = by_id.get(tid)
      if trk is not None:
        out[tid] = {'role': e.role, 'index': e.index, 'shield': e.shield, 'u2': e.u2, 'x': float(trk.x[0]), 'y': float(trk.x[1]),
                    'odo': f.odo_m, 'span': getattr(trk, 'span_seen', None), 'd_lo': e.d_lo, 'v_hi': e.v_hi, 't': self.t}
    return out

  def _still(self, f: Frame) -> dict:
    """The tracks known at the go that haven't moved since (_known_moved): the go stands on them as it found them."""
    known = self.go['tracks'] if self.go is not None else {}
    return {tid: k for tid, k in known.items() if not self._known_moved(f, tid, k)}

  def _still_spans(self, f: Frame, known: dict) -> list:
    """The bearing spans, from where the car is now, of the known tracks that haven't moved: the tracker's live span of
    each (its newest match), which a raw box of the same vehicle shares however the launch swings its bearing."""
    by_id = {t.track_id: t for t in f.tracks or ()}
    out = []
    for tid, k in known.items():
      trk = by_id.get(tid)
      span = getattr(trk, 'span_seen', None) if trk is not None else None
      if span is not None and not self._known_moved(f, tid, k):
        out.append(tuple(span))
    return out

  def _known_moved(self, f: Frame, tid: int, k: dict) -> bool:
    """A track known at the go, during the launch (its bearings from a moving car aren't comparable, so stop_evidence restarts
    its trust): one stopped at its line moved once the tracker's ground motion (ego-compensated) since the go passes
    SHIELD_MOVE_M or its ground speed SHIELD_V, each with the ranging's allowance; one that was far stays so only while its
    go-time distance less its go-time speed carried on still lets it stop at A_U2; anything else is moving."""
    trk = next((t for t in f.tracks or () if t.track_id == tid), None)
    if k['u2'] == 'far':
      d_lo = k['d_lo'] - k['v_hi'] * (self.t - k['t'])
      room = d_lo - sr.U2_LINE_M
      return room <= 0. or k['v_hi'] ** 2 / (2. * room) > sr.A_U2
    if trk is None or getattr(trk, 'x', None) is None:
      return False  # unmatched this frame: coasting; it reappears or its lane's reach judges it
    if k['u2'] == 'stopped' or k['shield']:
      moved = math.hypot(float(trk.x[0]) + (f.odo_m - k['odo']) - k['x'], float(trk.x[1]) - k['y'])
      speed = math.hypot(float(trk.x[2]), float(trk.x[3])) if len(trk.x) > 3 else 0.
      return speed > sr.SHIELD_V + MOVING_V or moved > sr.SHIELD_MOVE_M + 1.0
    return True

  def _wrong_way(self, f: Frame, tid: int, e, known: dict | None) -> bool:
    """A track on a one-way's illegal side (rows V22, G9) that may be coming the wrong way: before the go, one whose speed
    isn't trusted yet (its direction unknown) or that moves toward our path; legal traffic leaving by that side, or a car
    standing there, doesn't. During the launch (known: the tracks at the go), where our car's motion restarts every track's
    trust, a track there that wasn't known at the go, or a known one the tracker's ground motion brings nearer our path (a
    car that stood there starting toward us). A cyclist on a contraflow bike lane is no exception: it comes in toward us."""
    if known is None:
      return not e.v_trusted or e.v_nom > APPROACH_V
    k = known.get(tid)
    if k is None:
      return True
    trk = next((t for t in f.tracks or () if t.track_id == tid), None)
    if trk is None or getattr(trk, 'x', None) is None:
      return False  # coasting: it reappears or is gone
    return abs(float(trk.x[1])) < abs(k['y']) - (sr.SHIELD_MOVE_M + 1.0)

  def _turn_holds(self, f: Frame, pos: float = 0., others_moving: bool = False, ready: bool = True,
                  creep_ok: bool = True) -> list[Hold]:
    """2.4 item 5 (4.5 steps 2, 4): every earlier arrival gone (seen leaving its line: then the box and its lane judge it), no
    tie lost. A vehicle stopping at another line within TIE_S after our arrival is a tie: we yield to the right and, facing
    each other, a left yields to the other's straight or right (others are assumed straight); one later waits for us."""
    for tid, e in self.ev.items():
      if e.stopped_at_line and tid not in self.earlier and tid not in self.later and tid not in self.parked and self.arrived_t is not None:
        since = self.stopped_since.get(tid, self.t)
        if since - self.arrived_t <= TIE_S and (e.role == 'right' or (e.role == 'opposite' and f.maneuver == 'left')):
          self.earlier[tid] = since
          cloudlog.event("turnpilot.stop_go_tie", key=list(self.key), track=tid, role=e.role, yielded=True)
        else:
          self.later.add(tid)
    waiting = []
    for tid in sorted(self.earlier, key=lambda k: self.earlier[k]):
      if tid in self.gone or tid in self.parked:
        continue
      e = self.ev.get(tid)
      if e is not None and not e.stopped_at_line and (e.v_nom > sr.SHIELD_V or e.front_from_line_m > sr.SHIELD_PAST_M):
        self.gone.add(tid)
        continue
      waiting.append(tid)  # still at its line, or unseen without having been seen leaving it: never presumed gone
    self.rank = len(waiting)
    if not waiting or self.order_waived:
      self.turn_wait_s, self.creep = 0., None
      return []
    moving = others_moving or any(e.v_nom > MOVING_V for e in self.ev.values())
    if self.creep is not None:
      return self._creeping(f, pos, waiting, moving or not creep_ok)
    self.turn_wait_s = 0. if moving else self.turn_wait_s + f.dt
    if self.turn_wait_s < DEADLOCK_S - 1e-6:
      return [Hold('turn', "an earlier arrival's turn", DYNAMIC)]
    if self.creeps < DEADLOCK_CYCLES and ready:
      return self._deadlock(f, pos, waiting)
    if self.vision and ready:
      # V5: the creeps spent (or a standoff), nothing in the junction moving DEADLOCK_S and every other go condition met: the
      # order is waived; one moving later stops us short (the launch's re-check)
      return self._waive(waiting, f"vision: {self.creep_tries} creeps, nothing moving")
    # nothing in the junction has moved for DEADLOCK_S and no creep can run (another go condition holds the go, so we
    # wouldn't go were it our turn, or the encounter's creeps are spent): traffic isn't taking turns, so the turn is a static
    # cause (4.6), its STATIC_WAIT_S then the ladder (H3); anything moving makes it dynamic again, and the creep comes once
    # every other go condition is met
    return [Hold('turn', "an earlier arrival isn't taking its turn", STATIC)]

  def creep_blocked(self, f: Frame, pos: float, known: dict | None = None) -> str:
    """Why the car mustn't creep now ('' it may; 2.5, rows V12, V13): a vehicle whose path ends in our road's oncoming lane
    (in the junction box or the oncoming lane ahead of us, coming toward our car: a car turning into our road may cut the
    corner toward our bumper), or a long vehicle approaching the corner in a cross lane (coming toward our path, or its
    direction not known yet): its swing may cross our line. One standing at its line isn't approaching. known: the tracks
    when the creep began, judged by the tracker's ground motion once our own creep restarts their bearings' trust."""
    x0, x1 = self._x(self.edges.d_min, pos) - CROSSWALK_M, self._x(self._far_edge(), pos) + CROSSWALK_M
    for o in f.objects or ():
      cls = class_name(o.cls)
      if cls in MOTOR and x0 <= float(o.x) <= x1 and -BOX_RIGHT_M <= float(o.y) <= BOX_LEFT_M + sr.LANE_W_MAX and float(o.vx) < -MOVING_V:
        return f"a {cls} turning into our road"
    for tid, e in self.ev.items():
      if e.cls not in LARGE or e.lane is None or e.lane.movement == sr.WRONG_WAY:
        continue
      k = (known or {}).get(tid)
      coming = self._known_moved(f, tid, k) if k is not None else not e.v_trusted or e.v_nom > APPROACH_V
      if coming and e.d_nom < CORNER_M + max(e.v_nom, 0.) * CREEP_LOOK_S:
        return f"a {e.cls} coming to the corner {side_word(e.role)}"
    return ''

  def _deadlock(self, f: Frame, pos: float, waiting: list) -> list[Hold]:
    """4.5 step 5: an earlier arrival has held its turn DEADLOCK_S with nothing in the junction moving: we take ours as a
    driver would, with one signalling creep of DEADLOCK_CREEP_M on the latch's creep path (CREEP_SPEED, a G5 go with cause
    creep for a model that holds). Never closer than GO_ROOM_M to the closest plausible edge (the creep floor, 2.5; G14), never
    while creep_blocked, never from the decider's own hold (the latch let go: no creep path), so then the turn passes with
    no creep, said in the log."""
    self.creeps += 1
    room = self.edges.d_min - pos - DEADLOCK_CREEP_M
    need = sr.go_room_m(f.t_abort, right=f.maneuver == 'right')
    blocked = self.creep_blocked(f, pos)
    if blocked:
      self.creeps -= 1  # not yet: the creep waits for the corner to clear
      self.turn_wait_s = DEADLOCK_S
      return [Hold('turn', f"an earlier arrival's turn ({blocked})", DYNAMIC)]
    if f.control is None or room < need - 1e-6:
      why = 'no latch to creep from' if f.control is None else f"no room to creep: {room:.2f} m to the closest edge, {need:.2f} needed"
      return self._waive(waiting, why)
    self.creep_tries += 1
    moved = float(getattr(f.control.state, 'moved_m', 0.))
    # the latch holds again within CREEP_AT_M of its target, and the car runs on through the actuator delay and its stop: the
    # target is that much short, so the car comes to rest about DEADLOCK_CREEP_M on (8.1's bounded move)
    v = DEADLOCK_CREEP_V
    run_on = max(v * (f.t_abort - DT_MDL - 0.1) + v * v / 2. - CREEP_AT_M, 0.)
    self.creep = {'target': moved + max(DEADLOCK_CREEP_M - run_on, CREEP_AT_M + 0.1), 't': self.t, 'went': False,
                  'known': self._known_tracks(f), 'moved': False, 'from': moved}
    cloudlog.event("turnpilot.stop_go_deadlock_creep", key=list(self.key), waiting=sorted(waiting), pos=round(pos, 2),
                   target=round(self.creep['target'], 2), room=round(room, 2))
    return [Hold('deadlockCreep', "taking our turn: creeping forward", DYNAMIC)]

  def _creeping(self, f: Frame, pos: float, waiting: list, moving: bool) -> list[Hold]:
    """The deadlock's creep in hand: the earlier arrival moving (its ground motion, our own creep restarting its bearings'
    trust), or anything moving in the junction, or the corner no longer clear, ends it where the car is and we yield (the
    latch holds again, the creep's go withdrawn); reached, or its time up, the turn passes to us."""
    c = self.creep
    standing = f.v_ego < STAND_V
    stirred = [tid for tid in waiting if tid in c['known'] and self._known_moved(f, tid, c['known'][tid])]
    blocked = self.creep_blocked(f, pos, c['known'])
    if stirred or moving or blocked:
      why = f"track {stirred[0]} moved" if stirred else blocked or 'something moved in the junction'
      # the earlier arrival didn't move (something else did, or the corner stopped being clear): the creep is given back,
      # taken again once nothing moves DEADLOCK_S, up to DEADLOCK_CREEP_TRIES; spent, or the earlier arrival stirring and
      # standing again (a standoff), the turn is a static cause with its bound once nothing moves (_turn_holds)
      again = not stirred and self.creep_tries < DEADLOCK_CREEP_TRIES
      if again:
        self.creeps -= 1
      cloudlog.event("turnpilot.stop_go_deadlock_yield", key=list(self.key), why=why, pos=round(pos, 2), again=again,
                     tries=self.creep_tries)
      self.creep, self.turn_wait_s = None, 0.
      c['yield'] = why
      self.creep_end = ('deadlockYield', c)
      return [Hold('turn', "an earlier arrival's turn", DYNAMIC)]
    moved = float(getattr(f.control.state, 'moved_m', 0.)) if f.control is not None else c['target']
    c['moved'] = c['moved'] or f.v_ego > STAND_V
    there = moved >= c['target'] - CREEP_AT_M - 1e-6
    if (standing and (there or (c['moved'] and self.t - c['t'] > STOP_DWELL_S))) or self.t - c['t'] >= DEADLOCK_CREEP_MAX_S - 1e-6:
      self.creep = None
      self.creep_end = ('creepDone', c)
      return self._waive(waiting, f"crept {moved - c['from']:.2f} m")
    return [Hold('deadlockCreep', "taking our turn: creeping forward", DYNAMIC)]

  def _waive(self, waiting: list, why: str) -> list[Hold]:
    """The turn passes to us (4.5 step 5): go condition 5 waived, every other still applies; one moving later stops us short
    (the launch's re-check)."""
    self.order_waived = True
    cloudlog.event("turnpilot.stop_go_order_waived", key=list(self.key), waiting=sorted(waiting), why=why, t=round(self.t, 2))
    return []

  # --------------------------------------------------------------------------------------------- the other turn (rung 2)

  def _our_lanes(self, f: Frame) -> int:
    """Our approach's lanes toward the junction: navd's lane guidance for a turn here (Frame.approach_lanes), else 0, not
    known. Never the road straight across's lanes out (skeptic SG4c B: a turn bay's two-lane approach opposite a one-lane far
    side read as single-lane, and the other turn was taken across a lane)."""
    return int(f.approach_lanes) if f.approach_lanes > 0 else 0

  def _alternatives(self, f: Frame) -> list:
    """The maneuvers the ladder may synthesize instead of the route's (3.5), fewest conflicts first: a right from a lane that
    can turn right (a single-lane approach, or the rightmost lane), straight or a left only from a single-lane approach (a
    lane's own arrows aren't known), each where a road leaves the junction that way: (maneuver, its approach). Our lane count
    unknown (no navd lane guidance) is never single-lane: no straight or left, a right only from the rightmost lane."""
    j = self.junction
    if self.alt is not None or j is None or j.status != 'ok' or f.maneuver not in ('straight', 'left', 'right'):
      return []
    lanes = self._our_lanes(f)
    out = []
    for m, role in (('right', 'right'), ('straight', 'opposite'), ('left', 'left')):
      a = next((a for a in j.approaches if sr.effective_role(a) == role and a.outbound), None)
      if m == f.maneuver or a is None:
        continue
      if (m == 'right' and not (lanes == 1 or f.rightmost)) or (m != 'right' and lanes != 1):
        continue
      out.append((m, a))
    return out

  def _alt_holds(self, f: Frame, pos: float, m: str) -> list[Hold]:
    """Maneuver m's go conditions now, certified like any other (routing/certify.assess for its maneuver, its conflict set,
    its zones, exit and a right's guards), with this frame's holds that are the same for every maneuver (SHARED_HOLDS); no
    clock of the decider's moves for it."""
    fm = replace(f, maneuver=m)
    j = self.junction
    if self.downgraded:
      conflicting = {id(mv.approach) for mv in sr.conflict_movements(j, m, 0.)}
      if conflicting & set(self.downgraded):
        return [Hold('downgrade', "cross traffic doesn't stop", STATIC)]
    verdict = self.certified(fm, probe=True)
    if verdict.certification != 'certifiable' and verdict.reason != 'reach':
      return [Hold('cert:' + verdict.reason, f"can't certify ({verdict.reason})", STATIC)]
    holds = list(self._shared) + self._lane_holds(fm, pos, False, record=False) + self._people_holds(fm, pos, probe=True)
    holds += self._exit_holds(fm, pos)
    if m == 'right':
      if f.right_blindspot:
        holds.append(Hold('blindspot', "something in the right blind spot", DYNAMIC))
      if self.bike_t is not None and self.t - self.bike_t <= BIKE_PASS_MEMORY_S:
        holds.append(Hold('bikePassed', "a cyclist passed on the right", DYNAMIC))
    return holds

  def _other_turn(self, f: Frame, d: StopGoDecision, pos: float, empty: bool) -> bool:
    """7.1 rung 2 (empty False: at a static wait's bound, a maneuver with no static or fault cause, its dynamic ones waited
    for) and O12 (empty True: a wait past WAIT_ALT_S, a maneuver whose every go condition holds now): the first of
    _alternatives that qualifies is taken, said once (otherTurn, "Turning right instead"), and decides from then on; its side
    and angle from its approach's relBearing for turnpilotd's synthesized turn (desire from the go), the route's planned turn
    published missed once the encounter is done. True: taken. Never on a frame whose shared conditions (the box, the views,
    the yaw, the front zone, the turn) go_holds didn't read (its early returns: a position fault, a downgrade, the route's
    maneuver failing certification): the probe would have none of them, so the ladder goes on to H3."""
    if abs(self._shared_t - self.t) > 1e-9:
      cloudlog.event("turnpilot.stop_go_other_turn_no", key=list(self.key), why='shared conditions not read this frame')
      return False
    for m, a in self._alternatives(f):
      holds = self._alt_holds(f, pos, m)
      if (empty and holds) or any(h.kind in (STATIC, FAULT) for h in holds):
        continue
      rel = float(a.rel_bearing)
      self.alt = {'maneuver': m, 'side': 1 if m == 'right' else -1 if m == 'left' else 0, 'angle': abs(rel) if m != 'straight' else 0.,
                  'relBearing': rel, 'from': f.maneuver, 'why': 'wait' if empty else 'static', 'toNode': float(self.junction.stop_point_distance)}
      self.clear_s = self.static_s = 0.
      self.view_creep = None
      cloudlog.event("turnpilot.stop_go_other_turn", key=list(self.key), maneuver=m, planned=f.maneuver, why=self.alt['why'],
                     rel=round(rel, 1), waited_s=round(self.dynamic_s, 1), holds=[h.key for h in holds])
      self._show_alert('otherTurn')
      d.reason, d.maneuver, d.ledger = ALT_REASON[m], m, DYNAMIC if holds else 'none'
      return True
    return False

  def _x(self, d_along: float, pos: float) -> float:
    """Camera x (m ahead of the camera) of the point d_along m past the stop point, the bumper pos m past it."""
    return self.setback + d_along - pos

  def _far_edge(self) -> float:
    """m past the stop point to the far side of the farthest conflicting cross lane, at its largest plausible offset."""
    lanes = [ln for ln in self.lanes_all if ln.role in ('left', 'right')]
    if not lanes:
      return self.edges.d_max + 2. * sr.LANE_W_MAX
    return max(ln.s_max for ln in lanes) - self.setback + sr.LANE_W_MAX / 2.

  def _box_holds(self, f: Frame, pos: float) -> list[Hold]:
    """2.4 item 6 and V13: nothing moving in the junction box (a vehicle standing there is a static wait, V4); a large vehicle
    gone from a conflicting lane within HIDE_S (V17: it may hide one behind it); nothing at all in the 0-6 m ahead of the
    bumper (a track or a raw box of a fresh view)."""
    holds = []
    x0, x1 = self._x(self.edges.d_min, pos), self._x(self._far_edge(), pos)
    for o in f.objects or ():
      cls = class_name(o.cls)
      x, y = float(o.x), float(o.y)
      v = math.hypot(float(o.vx), float(o.vy))
      if cls in MOTOR + CYCLISTS and x0 <= x <= x1 and -BOX_RIGHT_M <= y <= BOX_LEFT_M:
        holds.append(Hold(f'box:{o.trackId}', f"a {cls} in the junction" + ('' if v > MOVING_V else ", stopped"),
                          DYNAMIC if v > MOVING_V else STATIC))
      elif 0. <= x - self.setback <= BOX_HOLD_M and abs(y) <= PATH_HALF_WIDTH_M + 0.5:
        holds.append(Hold(f'near:{o.trackId}', f"a {cls} just ahead", DYNAMIC if v > MOVING_V else STATIC))
    if f.live is not None:
      for key, frame in f.live.raw.items():
        if key == 'far' or not f.live.fresh_key(key):
          continue
        if any(b.xy is not None and 0. <= float(b.xy[0]) - self.setback <= BOX_HOLD_M and abs(float(b.xy[1])) <= PATH_HALF_WIDTH_M + 0.5
               for b in frame.boxes):
          holds.append(Hold('nearRaw', "something just ahead", DYNAMIC))
          break
    for tid, e in self.ev.items():
      if e.cls in LARGE:
        self.large_seen[tid] = e.role
    for tid, role in list(self.large_seen.items()):
      e = self.ev.get(tid)
      if e is None or e.role != role:
        self.large_left[role] = self.t
        del self.large_seen[tid]
    for role, t in self.large_left.items():
      if self.t - t <= HIDE_S:
        holds.append(Hold(f'hide:{role}', f"a large vehicle just passed {side_word(role)}", DYNAMIC))
    return holds

  def _people_holds(self, f: Frame, pos: float, launch: bool = False, probe: bool = False) -> list[Hold]:
    """2.4 item 7, the conservative zones (P1, P2, P10): any person track or raw person box in the near or far crosswalk band
    (our half of the road plus one lane, its ranging band of plus or minus PED_RANGE_BAND included) or in the box blocks the
    go; moving, a dynamic wait, standing, a static one. A person standing at a crosswalk's curb (D3, P3, P15: _curbs) is the
    curb rule's: PED_CURB_S, then the go at STOP_GO_V_MAX past that crosswalk (_curb_holds). A right turn (and off the route, the union)
    has its exit leg's crosswalk too, across the right road beside the box, and the sidewalk users heading for it (P4; a left's
    is scene_policy's junction hold's, P2). A person heading for a crosswalk on our path (the arrival clause, P16:
    _arrival_holds). The front zone (P6)."""
    holds = []
    near0, near1 = self._x(0., pos) - 1., self._x(self.edges.d_max, pos)
    box0, box1 = self._x(self.edges.d_min, pos), self._x(self._far_edge(), pos)
    far0, far1 = box1, box1 + CROSSWALK_M + 1.
    # a right's exit leg's crosswalk (P4): across the right road beside the box, its sidewalk ends at the corners
    exit_on = f.maneuver in ('right', 'unknown')
    ex0, ex1, ey0, ey1 = box0 - 1., box1 + 1., -(BOX_RIGHT_M + CROSSWALK_M + 1.), -BOX_RIGHT_M + 0.5

    def zone(x_lo, x_hi, y) -> str:
      if near0 <= x_hi and x_lo <= near1 and -HALF_RIGHT_M <= y <= HALF_LEFT_M:
        return 'the crosswalk'
      if box0 <= x_hi and x_lo <= box1 and -BOX_RIGHT_M <= y <= BOX_LEFT_M:
        return 'the junction'
      if f.maneuver in ('straight', 'unknown') and far0 <= x_hi and x_lo <= far1 and -HALF_RIGHT_M <= y <= HALF_LEFT_M:
        return 'the far crosswalk'
      if exit_on and ex0 <= x_hi and x_lo <= ex1 and ey0 <= y <= ey1:
        return 'the exit crosswalk'
      return ''

    curb_xy = []  # the curb people's own positions: their track decides (D3), so their raw box never blocks as a stranger's
    for o in f.objects or ():
      if class_name(o.cls) != PERSON:
        continue
      x, y = float(o.x), float(o.y)
      if self.curb.get(o.trackId, {}).get('standing'):
        curb_xy.append((x, y))
        continue  # at a curb, standing (D3, P15): the curb rule's, never a person standing in the road
      where = zone(x * (1. - PED_RANGE_BAND), x * (1. + PED_RANGE_BAND), y)
      if where:
        v = math.hypot(float(o.vx), float(o.vy))
        holds.append(Hold(f'ped:{o.trackId}', f"a person in {where}", DYNAMIC if v > PED_STILL_V else STATIC))
    holds += self._curb_holds(launch)
    if f.live is not None:
      for key, frame in f.live.raw.items():
        if key == 'far' or not f.live.fresh_key(key):
          continue
        for b in frame.boxes:
          if b.cls == PERSON and b.conf >= RAW_PERSON_CONF and b.xy is not None:
            x, y = float(b.xy[0]), float(b.xy[1])
            if any(math.hypot(x - cx, y - cy) <= RAW_PED_SAME_M for cx, cy in curb_xy):
              continue
            if zone(x * (1. - PED_RANGE_BAND), x * (1. + PED_RANGE_BAND), y):
              holds.append(Hold(f'pedRaw:{key}', "a person at the crosswalk", DYNAMIC))
              break
    walks = [(near0, near1, -HALF_RIGHT_M, HALF_LEFT_M, self.edges.d_min, 'near')]
    if f.maneuver in ('straight', 'unknown'):
      walks.append((far0, far1, -HALF_RIGHT_M, HALF_LEFT_M, self._far_edge() + CROSSWALK_M + 1., 'far'))
    if exit_on:
      walks.append((ex0, ex1, ey0, ey1, self.edges.d_min + EXIT_TURN_M + CROSSWALK_M, 'exit'))
    holds += self._arrival_holds(f, pos, walks)
    if not probe:
      holds += self._front_zone(f)  # the same for every maneuver: the other turn's probe reads this frame's (SHARED_HOLDS)
    return holds

  def _crosswalks(self, f: Frame, pos: float) -> list:
    """The crosswalks on our path from where the car is now (2.4 item 7's zones): (which, x0, x1, y0, y1 in camera x and y, the
    axis a person crosses it along, m past the stop point to its far edge along our path). The near one across our road
    (our half plus one lane, to the right curb), the far one going straight (and off the route), the exit leg's across the
    right road for a right (P4)."""
    near0, near1 = self._x(0., pos) - 1., self._x(self.edges.d_max, pos)
    box0, box1 = self._x(self.edges.d_min, pos), self._x(self._far_edge(), pos)
    out = [('near', near0, near1, -HALF_RIGHT_M, HALF_LEFT_M, 'y', self.edges.d_max)]
    if f.maneuver in ('straight', 'unknown'):
      out.append(('far', box1, box1 + CROSSWALK_M + 1., -HALF_RIGHT_M, HALF_LEFT_M, 'y', self._far_edge() + CROSSWALK_M + 1.))
    if f.maneuver in ('right', 'unknown'):
      out.append(('exit', box0 - 1., box1 + 1., -(BOX_RIGHT_M + CROSSWALK_M + 1.), -BOX_RIGHT_M + 0.5, 'x',
                  self.edges.d_min + EXIT_TURN_M + CROSSWALK_M))
    return out

  @staticmethod
  def _at_curb(x: float, y: float, cw) -> bool:
    """A person read at (x, y) is at crosswalk cw's curb (D3, P15): read outside it with their ranging band (along the ray)
    reaching it or its curb (CURB_M past either end, across its width), or read inside it with that band reaching past an
    end, onto the curb. Read inside with the band inside, they stand in the road."""
    _which, x0, x1, y0, y1, axis, _until = cw

    def inside(px, py):
      return x0 <= px <= x1 and y0 <= py <= y1

    def curb(px, py):
      if axis == 'y':
        return x0 <= px <= x1 and (y0 - CURB_M <= py < y0 or y1 < py <= y1 + CURB_M)
      return y0 <= py <= y1 and (x0 - CURB_M <= px < x0 or x1 < px <= x1 + CURB_M)

    def past_end(px, py):
      return (py < y0 or py > y1) and x0 <= px <= x1 if axis == 'y' else (px < x0 or px > x1) and y0 <= py <= y1

    band = [(k * x, k * y) for k in BAND_KS]
    if inside(x, y):
      return any(past_end(px, py) for px, py in band)
    beyond = (y < y0 or y > y1) if axis == 'y' else (x < x0 or x > x1)  # read out on the sidewalk past one of its ends
    return beyond and any(inside(px, py) or curb(px, py) for px, py in band)

  @staticmethod
  def _inside(x: float, y: float, cw) -> bool:
    return cw[1] <= x <= cw[2] and cw[3] <= y <= cw[4]

  def _stepped(self, tid: int, cw, pos: float) -> bool:
    """D3's measure: the person moved PED_STILL_M or more toward the road (along cw's crossing axis, toward its middle) in
    the last PED_STILL_S, from their ground positions (the arrival clause's history)."""
    h = self.peds.get(tid)
    if not h:
      return False
    then = next((s for s in h if s[0] >= self.t - PED_STILL_S - 1e-9), h[-1])
    now = h[-1]
    _which, x0, x1, y0, y1, axis, _until = cw
    if axis == 'y':
      mid = (y0 + y1) / 2.
      toward = abs(then[2] - mid) - abs(now[2] - mid)
    else:
      mid = (x0 + x1) / 2. + pos  # ground x
      toward = abs(then[1] - mid) - abs(now[1] - mid)
    return toward >= PED_STILL_M - 1e-9

  def _curbs(self, f: Frame, pos: float) -> None:
    """Decision D3 (rows P3, P15), every frame: each person track at a crosswalk's curb (_at_curb) who hasn't stepped toward
    the road (_stepped) keeps a clock from when they stand there (first seen standing, or the frame they stand again after
    walking: D3's 5 s are of standing still, never of walking up); a step starts it again once they stand (meanwhile the
    arrival clause judges them), and one gone from the curb for PED_STILL_S is forgotten."""
    cws = self._crosswalks(f, pos)
    for o in f.objects or ():
      if class_name(o.cls) != PERSON:
        continue
      x, y = float(o.x), float(o.y)
      # read inside a crosswalk (or the box): at a curb only by that crosswalk's own band (P15), else in the road
      box = ('box', self._x(self.edges.d_min, pos), self._x(self._far_edge(), pos), -BOX_RIGHT_M, BOX_LEFT_M, 'y', 0.)
      home = [c for c in cws + [box] if self._inside(x, y, c)]
      cw = next((c for c in (home or cws) if c[0] != 'box' and self._at_curb(x, y, c)), None)
      if cw is None:
        continue
      if self._stepped(o.trackId, cw, pos):
        if self.curb.pop(o.trackId, None) is not None:
          cloudlog.event("turnpilot.stop_go_curb_step", key=list(self.key), track=int(o.trackId), which=cw[0])
        continue
      # standing (the arrival clause's measure: under PED_STILL_M in PED_STILL_S; too new to tell, taken as standing): one
      # walking along the curb or away from it is the arrival clause's while it moves, and its clock starts once it stands
      h = self.peds.get(o.trackId) or ()
      then = next((s for s in h if s[0] >= self.t - PED_STILL_S - 1e-9), None)
      known = bool(h) and h[-1][0] - h[0][0] >= PED_STILL_S - 1e-9
      standing = not known or then is None or math.hypot(h[-1][1] - then[1], h[-1][2] - then[2]) < PED_STILL_M
      rec = self.curb.setdefault(o.trackId, {'since': self.t, 'logged': False, 'standing': standing})
      if standing and not rec['standing']:
        rec['since'], rec['logged'] = self.t, False  # standing again after walking: D3's clock from now
      rec.update(which=cw[0], until=cw[6], seen=self.t, standing=standing)
      if not rec['logged'] and self.t - rec['since'] >= PED_CURB_S - 1e-6:
        rec['logged'] = True
        cloudlog.event("turnpilot.stop_go_curb", key=list(self.key), track=int(o.trackId), which=cw[0], waited_s=round(self.t - rec['since'], 1),
                       x=round(float(o.x), 2), y=round(float(o.y), 2))
    for tid in [k for k, r in self.curb.items() if r['seen'] < self.t - PED_STILL_S - 1e-9]:
      del self.curb[tid]

  def _curb_holds(self, launch: bool) -> list[Hold]:
    """D3: a person at a crosswalk's curb holds the go PED_CURB_S from when first seen there standing (a dynamic wait, bounded by
    itself); then the go may come, at STOP_GO_V_MAX at most until our rear bumper is past that crosswalk (curb_until), with
    scene_policy's in-path pedestrian stop armed. During the launch a curb person (however new) withdraws nothing: they arm
    the cap; a step toward the road is the arrival clause's, re-checked to commit."""
    if launch:
      return []
    return [Hold(f'curb:{tid}', f"a person at the curb of {({'far': 'the far ', 'exit': 'the exit '}).get(rec['which'], 'the ')}crosswalk",
                 DYNAMIC) for tid, rec in self.curb.items() if rec['standing'] and self.t - rec['since'] < PED_CURB_S - 1e-6]

  def curb_cap_until(self) -> float | None:
    """m past the stop point the go's STOP_GO_V_MAX holds to for the curb people seen (D3), None with none."""
    ends = [r['until'] + CAR_LEN_M for r in self.curb.values()]
    return max(ends) if ends else None

  def t_clear(self, f: Frame, pos: float, far_edge: float) -> float:
    """s until our rear bumper is past far_edge (m past the stop point; 2.4 item 7's t_clear): T_GO_LAT from rest, then the G5
    nudge's A_LAUNCH_PLAN (the slowest launch a go can produce) up to STOP_GO_V_MAX, from the car's speed now once moving."""
    d = far_edge - pos + CAR_LEN_M
    if d <= 0.:
      return 0.
    a, vmax = sr.A_LAUNCH_PLAN, sr.STOP_GO_V_MAX
    v0 = min(max(f.v_ego, 0.), vmax)
    lat = sr.T_GO_LAT if f.v_ego < STAND_V else 0.
    d_acc = (vmax * vmax - v0 * v0) / (2. * a)
    if d <= d_acc:
      return lat + (math.sqrt(v0 * v0 + 2. * a * d) - v0) / a
    return lat + (vmax - v0) / a + (d - d_acc) / vmax

  def _arrival_holds(self, f: Frame, pos: float, walks) -> list[Hold]:
    """2.4 item 7's arrival clause (P16): a person not standing still, a track or a raw box seen on RAW_PED_RUNS of the last
    RAW_PED_OF runs of its view, that would reach a crosswalk on our path (walks: (x0, x1 camera x, y0, y1 camera y, its far
    edge m past the stop point, which)) before our rear bumper has cleared it plus PED_ARRIVE_MARGIN_S blocks the go: t_arr = d_lo / v_hi, d_lo its
    distance to the crosswalk at the near end of its ranging band, v_hi its speed toward it plus 2 sigma once trusted
    (VEL_TRUST_S with VEL_TRUST_SAMPLES), PED_V_UNTRUSTED before. One moving away doesn't block; one standing still (under
    PED_STILL_M in PED_STILL_S) is D3's (P3). A dynamic wait; during the launch the re-check withdraws the go (_going)."""
    holds = []
    clears = [(x0, x1, y0, y1, self.t_clear(f, pos, edge) + PED_ARRIVE_MARGIN_S, which) for x0, x1, y0, y1, edge, which in walks]

    def dist(x, y, x0, x1, y0, y1) -> float:
      dx = max(x0 - x, 0., x - x1)
      dy = max(y0 - y, 0., y - y1)
      return math.hypot(dx, dy)

    def d_lo(x, y, x0, x1, y0, y1) -> float:
      return min(dist(k * x, k * y, x0, x1, y0, y1) for k in (1. - PED_RANGE_BAND, 1. - PED_RANGE_BAND / 2., 1., 1. + PED_RANGE_BAND / 2.,
                                                       1. + PED_RANGE_BAND))

    people = []
    for o in f.objects or ():
      if class_name(o.cls) != PERSON:
        continue
      h = list(self.peds.get(o.trackId, ()))
      window = [s for s in h if s[0] >= self.t - sr.VEL_TRUST_S - 1e-9]
      trusted = len(window) >= sr.VEL_TRUST_SAMPLES and h and h[-1][0] - h[0][0] >= sr.VEL_TRUST_S - 1e-9
      if trusted:
        then = next(s for s in h if s[0] >= self.t - PED_STILL_S - 1e-9)
        if math.hypot(h[-1][1] - then[1], h[-1][2] - then[2]) < PED_STILL_M:
          continue  # standing still: D3's (the curb rule), not an arrival
      people.append((f'pedArrive:{o.trackId}', float(o.x), float(o.y), window if trusted else None))
    tracked = [(x, y) for _, x, y, _ in people] + [(float(o.x), float(o.y)) for o in f.objects or () if class_name(o.cls) == PERSON]
    for key, runs in self.raw_peds.items():
      if not runs or (f.live is not None and not f.live.fresh_key(key)):
        continue
      for xg, y in runs[-1]:
        x = xg - pos
        if any(math.hypot(x - tx, y - ty) <= RAW_PED_SAME_M for tx, ty in tracked):
          continue  # a person track's own box: the track decides
        seen = sum(any(math.hypot(xg - bx, y - by) <= RAW_PED_SAME_M for bx, by in boxes) for boxes in runs)
        if seen >= RAW_PED_RUNS:
          people.append((f'pedArriveRaw:{key}', x, y, None))
    for key, x, y, window in people:
      for x0, x1, y0, y1, need, which in clears:
        lo = d_lo(x, y, x0, x1, y0, y1)
        name = {'far': 'far ', 'exit': 'exit '}.get(which, '')
        if lo <= 0.:
          # its ranging band (along the ray: range and offset together) reaches into the crosswalk, wherever it walks
          holds.append(Hold(f'{key}:{which}', f"a person at the {name}crosswalk", DYNAMIC))
          break
        if window is None:
          v_hi = PED_V_UNTRUSTED
        else:
          ts = [s[0] for s in window]
          ds = [dist(s[1] - pos, s[2], x0, x1, y0, y1) for s in window]
          t_bar = sum(ts) / len(ts)
          stt = sum((t - t_bar) ** 2 for t in ts)
          slope = sum((t - t_bar) * (d - sum(ds) / len(ds)) for t, d in zip(ts, ds, strict=True)) / max(stt, 1e-9)
          res = [d - (sum(ds) / len(ds) + slope * (t - t_bar)) for t, d in zip(ts, ds, strict=True)]
          sigma = math.sqrt(sum(r * r for r in res) / max(len(res) - 2, 1) / max(stt, 1e-9))
          v_hi = -slope + 2. * sigma
        if v_hi <= 0.05:
          continue  # moving away from it, or along it
        if lo / v_hi < need:
          holds.append(Hold(f'{key}:{which}', f"a person heading for the {name}crosswalk", DYNAMIC))
          break
    return holds

  def _front_zone(self, f: Frame) -> list[Hold]:
    """P6: a person track that entered the 0-3 m zone ahead of the bumper and wasn't seen leaving, across views: a track that
    ended at its side edge moving sideways over FRONT_LEAVE_V, then within FRONT_LEAVE_S a person track starting outside the
    zone on that side, is the same person leaving. FRONT_ZONE_MAX_S of it is H7."""
    now = {}
    for o in f.objects or ():
      if class_name(o.cls) == PERSON:
        x, y = float(o.x), float(o.y)
        now[o.trackId] = (0. <= x - self.setback <= FRONT_ZONE_M and abs(y) <= FRONT_ZONE_HALF_W, y, float(o.vy))
    for tid, (inside, y, vy) in now.items():
      rec = self.front.get(tid)
      if rec is None and not inside:
        for t_end, side in list(self.front_ended):
          if (y > 0.) == (side > 0.):
            self.front_ended.remove((t_end, side))
            lost = [k for k, r in self.front.items() if r['lost'] and (r['y'] > 0.) == (side > 0.)]
            if lost:
              del self.front[lost[0]]
            break
      if rec is None:
        self.front[tid] = {'in': inside, 'y': y, 'vy': vy, 'lost': False}
      else:
        rec.update({'in': inside, 'y': y, 'vy': vy, 'lost': False})
    for tid, rec in list(self.front.items()):
      if tid in now:
        continue
      if not rec['in']:
        del self.front[tid]
      elif not rec['lost']:
        rec['lost'] = True
        if abs(rec['y']) >= FRONT_ZONE_HALF_W - FRONT_EDGE_M and rec['vy'] * rec['y'] > 0. and abs(rec['vy']) > FRONT_LEAVE_V:
          self.front_ended.append((self.t, rec['y']))
    self.front_ended = [(t, s) for t, s in self.front_ended if self.t - t <= FRONT_LEAVE_S]
    occupied = any(r['in'] for r in self.front.values())
    self.front_s = self.front_s + f.dt if occupied else 0.
    return [Hold('front', "a person may be in front of the car", FAULT)] if occupied else []

  def _exit_holds(self, f: Frame, pos: float) -> list[Hold]:
    """2.4 item 7: the exit leg has room for the car plus 2 m past its crosswalk: no stopped vehicle there (a static wait)."""
    holds = []
    far = self._x(self._far_edge(), pos)
    for o in f.objects or ():
      cls = class_name(o.cls)
      if cls not in MOTOR or math.hypot(float(o.vx), float(o.vy)) > MOVING_V:
        continue
      x, y = float(o.x), float(o.y)
      if f.maneuver == 'right':
        x0 = self._x(self.edges.d_min, pos)
        inside = x0 <= x <= x0 + 2. * sr.LANE_W_MAX and -(CROSSWALK_M + EXIT_ROOM_M + 4.) <= y <= -BOX_RIGHT_M
      elif f.maneuver == 'left':
        x0 = self._x(self.edges.d_max, pos) + sr.LANE_W_MAX
        inside = x0 <= x <= x0 + 2. * sr.LANE_W_MAX and BOX_LEFT_M <= y <= CROSSWALK_M + EXIT_ROOM_M + 6.
      else:
        inside = far <= x <= far + CROSSWALK_M + EXIT_ROOM_M and abs(y) <= PATH_HALF_WIDTH_M + 0.5
      if inside:
        holds.append(Hold(f'exit:{o.trackId}', "no room past the junction", DYNAMIC if self.vision else STATIC))  # V6
    return holds

  # ---------------------------------------------------------------------------------------------------------- the ledger

  def _ledger(self, f: Frame, d: StopGoDecision, holds: list[Hold], acting: bool, pos: float = 0.) -> None:
    """4.6: every held state says why (waitingForCross, its reason) and has a bound or a dynamic cause. Dynamic: no bound, an
    info chime at WAIT_INFO_S and every WAIT_INFO_S after. Static (every hold static): STATIC_WAIT_S, then the ladder (H3).
    A fault's: its grace, then its handback. As shadow, only said (the latch's own handback stands)."""
    d.prompt = 'waitingForCross'
    # the wait's clock (the info chime's) runs from the arrival to the go, whatever comes and goes meanwhile
    self.dynamic_s += f.dt
    if self.dynamic_s >= self.next_chime - 1e-6:
      self.next_chime += WAIT_INFO_S
      self.chime_t = INFO_CHIME_S
    if not holds:
      # clear, CLEAR_S not yet run: the static clock keeps its count (a static cause that flickers still meets its bound)
      d.ledger, d.reason = 'none', "Cross traffic clear"
      self.paused_s = self.h2_s = 0.
      return
    top = holds[0]
    d.reason = "Waiting: " + top.reason
    if self.vision and top.key != self.vision_top:
      self.vision_top = top.key
      cloudlog.event("turnpilot.stop_go_vision_wait", key=list(self.key), holds=[(h.key, h.kind) for h in holds], clear_s=round(self.clear_s, 2),
                     need_s=self._need_s(), lanes=[(x.role, x.index, round(x.r_seen, 1), round(x.r_req, 1), x.binding) for x in self.last_lanes])
    # the wait for the route is bounded by its own OFFROUTE_WAIT_S: a static cause's clock runs on through it, so a union that
    # can't certify meets the ladder STATIC_WAIT_S from the full stop (O9: wait for the reroute, then decide and the ladder)
    kinds = {h.kind for h in holds if h.key != 'route'} or {DYNAMIC}
    cam = next((h for h in holds if h.key == 'camera'), None)
    position = next((h for h in holds if h.key == 'position'), None)
    front = next((h for h in holds if h.key == 'front'), None)
    self.paused_s = self.paused_s + f.dt if cam is not None else 0.
    self.h2_s = self.h2_s + f.dt if position is not None else 0.
    if cam is not None:
      d.ledger, d.reason = FAULT, "Waiting: " + cam.reason
      self._show_alert('cameraPaused')
      if acting and self.paused_s >= FAULT_GRACE_S - 1e-6:
        self._fault('H1', f, d, cam.reason)
      return
    if position is not None:
      d.ledger, d.reason = FAULT, "Waiting: " + position.reason
      self._show_alert('junctionUncertain')
      if acting and self.h2_s >= FAULT_GRACE_S - 1e-6:
        self._fault('H2', f, d, position.reason)
      return
    if front is not None:
      d.ledger, d.reason = FAULT, "Waiting: " + front.reason
      if acting and self.front_s >= FRONT_ZONE_MAX_S - 1e-6:
        self._fault('H7', f, d, front.reason)
      return
    gate = next((h for h in holds if h.key.startswith('vision:')), None)
    if gate is not None:
      # vision's gate failing after the first go (C14): the handback at once, the decider's own hold kept (_ladder)
      d.ledger, d.reason = FAULT, "Waiting: " + gate.reason
      if acting:
        self._fault('H9', f, d, gate.reason)
      return
    if DYNAMIC in kinds:
      # something moving: waiting is right, with no bound; a static cause starts its clock again once nothing moves
      d.ledger = DYNAMIC
      self.static_s = 0.
      if acting and self.alt is None and self.dynamic_s >= WAIT_ALT_S - 1e-6 and self._other_turn(f, d, pos, empty=True):
        return  # O12: a long wait for a gap, another maneuver whose go conditions hold now taken
      if self.chime_t > 0.:
        self.chime_t = max(self.chime_t - f.dt, 0.)
        d.chime, d.reason = True, "Waiting for cross traffic"
      return
    self.chime_t = 0.
    d.ledger = STATIC
    self.static_s += f.dt
    route_known = f.route_cert == 'uncertifiable' and top.key.startswith('cert:') and not self.vision  # V1
    if acting and route_known:
      self._fault('H3', f, d, ROUTE_KNOWN_REASON)
    elif acting and self.vision and self.view_creep is None and not self.vision_creep_tried and \
        self.static_s >= sr.VISION_CREEP_AFTER_S - 1e-6 and all(h.key.startswith('reach:') for h in holds):
      # V9 (spec C11): the creep for a view after VISION_CREEP_AFTER_S of a wait on reach alone, not STATIC_WAIT_S
      self.vision_creep_tried = True
      self._view_creep_start(f, d, holds, pos)
    elif acting and self.static_s >= STATIC_WAIT_S - 1e-6:
      self._ladder_rungs(f, d, holds, top, pos)

  def _ladder_rungs(self, f: Frame, d: StopGoDecision, holds: list[Hold], top: Hold, pos: float) -> None:
    """7.1, the static wait's bound: rung 1, the creep for a view where 2.5 predicts a closer point helps the binding lane
    and the creep floor allows it (_view_creep_start; while one is in hand, crept or measuring, the ladder waits on it); rung
    3, H3 with the reason."""
    vc = self.view_creep
    if vc is not None and vc['phase'] in ('creep', 'measure'):
      return
    if vc is None and self._view_creep_start(f, d, holds, pos):
      return
    self.view_creep = {'phase': 'over'}
    if self._other_turn(f, d, pos, empty=False):
      return
    self._fault('H3', f, d, top.reason)

  # ------------------------------------------------------------------------------------------------- the creep for a view

  def _binding(self) -> LaneStatus | None:
    """The lane short of its reach with the least margin (R_seen less R_req) this frame, None with none short."""
    short = [x for x in self.last_lanes if x.r_seen + 1e-9 < x.r_req and not x.shielded]
    return min(short, key=lambda x: x.r_seen - x.r_req) if short else None

  def _view_creep_start(self, f: Frame, d: StopGoDecision, holds: list[Hold], pos: float, look: bool = False) -> bool:
    """2.5's creep for a corner occluder, the ladder's first rung (row G3): only for a wait whose every cause is a lane short
    of its reach (a parked car, an occluder; standing back extends frame-edge reach, so the walk decides), from the latch (its
    creep path), with the car straight and no turn desire, never while creep_blocked (V12, V13), never closer than GO_ROOM_M
    (GO_ROOM_RIGHT_M more for a right) to the closest plausible edge after the step (at a surveyed junction measured to the
    surveyed travel-lane edge: edge_band), and only where the walk predicted from VIEW_CREEP_M on (every ranged occluder
    re-seen from there: visibility's ahead_m) gains the binding lane VIEW_CREEP_GAIN_M and, from the furthest point the floor
    allows, meets its requirement (so no run of pointless steps). True: a step started. look: the long look's creep to the best
    view (_look_lanes) with nothing holding the go."""
    fast = self._look_lanes() if look else []
    if look:
      b = min(fast, key=lambda x: x.r_seen) if fast else None
    else:
      b = self._binding()
    if b is None or f.control is None or (not look and (not holds or any(not h.key.startswith('reach:') for h in holds))):
      return False
    yaw_go, steer_go = (VISION_YAW_GO_DEG, VISION_STEER_GO_DEG) if self.vision else (YAW_GO_DEG, STEER_GO_DEG)
    if abs(f.yaw20_deg) > yaw_go or abs(f.steer_deg) > steer_go or (f.desire_out and not self.went):
      return False
    need = sr.go_room_m(f.t_abort, right=f.maneuver == 'right')
    room = self.edges.d_min - pos - VIEW_CREEP_M
    blocked = self.creep_blocked(f, pos)
    if room < need - 1e-6 or blocked:
      cloudlog.event("turnpilot.stop_go_view_creep_no", key=list(self.key), why=blocked or 'room', room=round(room, 2), need=round(need, 2))
      return False
    lanes = self.junction_lanes(f)
    key = (b.role, b.index)

    def margin(ahead):
      rep = self._reach_report(f, lanes, pos, ahead)
      if look:
        # the best view: the least reach of the fast cross lanes from there, every lane still meeting its vision floor (a step
        # that gains the range-bound far lane loses frame-edge reach on the near one)
        if any(seen.reach_m + 1e-9 < sr.reach_needed_vision(ln.limit, ln.approach) for ln, seen in zip(lanes, rep.lanes, strict=True)):
          return -math.inf
        want = {(x.role, x.index) for x in fast}
        return min((seen.reach_m for ln, seen in zip(lanes, rep.lanes, strict=True) if (ln.role, ln.index) in want), default=-math.inf)
      for ln, seen in zip(lanes, rep.lanes, strict=True):
        if (ln.role, ln.index) == key:
          if self.vision:
            return seen.reach_m - sr.reach_needed_vision(ln.limit, ln.approach)
          return seen.r_seen_m - max(seen.need_m - sr.V_CROSS_FACTOR * ln.limit * sr.T_LAT, 0.)
      return -math.inf
    now = b.r_seen if look else b.r_seen - b.r_req
    step = margin(VIEW_CREEP_M)
    furthest = math.floor((self.edges.d_min - pos - need) / VIEW_CREEP_M + 1e-6) * VIEW_CREEP_M
    best = margin(furthest) if furthest > VIEW_CREEP_M else step
    if step < now + VIEW_CREEP_GAIN_M or best < 0.:
      cloudlog.event("turnpilot.stop_go_view_creep_no", key=list(self.key), why='notPredicted', lane=list(key), now=round(now, 2),
                     step=round(step, 2), best=round(best, 2), furthest=round(furthest, 2))
      return False
    moved = float(getattr(f.control.state, 'moved_m', 0.))
    v = VIEW_CREEP_V
    run_on = max(v * (f.t_abort - DT_MDL - 0.1) + v * v / 2. - CREEP_AT_M, 0.)
    self.view_creep = {'phase': 'creep', 'target': moved + max(VIEW_CREEP_M - run_on, CREEP_AT_M + 0.1), 't': self.t, 'went': False,
                       'from': moved, 'moved': False, 'margin0': now, 'lane': key, 'known': self._known_tracks(f), 'look': look}
    cloudlog.event("turnpilot.stop_go_view_creep", key=list(self.key), lane=list(key), pos=round(pos, 2), now=round(now, 2),
                   predicted=round(step, 2), best=round(best, 2), room=round(room, 2), steps=self.view_steps)
    d.creep, d.action, d.reason = (self.view_creep['target'], VIEW_CREEP_V), 'creeping', VIEW_CREEP_REASON
    return True

  def _look_lanes(self) -> list:
    """The long look's lanes a creep could help (this frame's LaneStatus): the cross lanes (left, right) of an approach over 25
    mph or of unknown speed whose walk reaches less than that approach's own floor would ask (unknown: any). The opposite lane
    is straight ahead, where a step forward only adds what a range bin already gives."""
    roles = {r: lim for r, lim in self.long_look if r in ('left', 'right')}
    out = []
    for x in self.last_lanes:
      if x.role in roles and not x.shielded:
        lim = roles[x.role]
        own = x.r_seen < (lim * sr.T_R + lim * lim / (2. * sr.A_PHANTOM)) if lim > 0. else True
        if own:
          out.append(x)
    return out

  def _view_creeping(self, f: Frame, d: StopGoDecision, holds: list[Hold], pos: float, ctl, acting: bool, standing: bool) -> None:
    """The creep for a view in hand. creep: on the latch's creep path at VIEW_CREEP_V, one G5 go (cause creep) at its start
    for a model that holds; anything moving that isn't its lanes' (a person, the box, a queue) or creep_blocked gives it back
    where the car is (the latch holding again; the ladder may take it again at the next bound); reached, or its time up,
    measure. measure: CLEAR_S from where it stands (the go comes on its own if every condition holds); then a margin gained
    VIEW_CREEP_GAIN_M on the binding lane may take the next step where it is predicted to help, else the rung is over and
    the ladder goes on."""
    vc = self.view_creep
    if vc['phase'] == 'creep':
      moving = [h for h in holds if h.kind == DYNAMIC and h.key.split(':')[0] not in CREEP_LANE_HOLDS]
      blocked = self.creep_blocked(f, pos, vc['known'])
      if ctl is None or moving or blocked:
        why = 'no latch' if ctl is None else blocked or moving[0].reason
        cloudlog.event("turnpilot.stop_go_view_creep_yield", key=list(self.key), why=why, pos=round(pos, 2))
        if f.go_active == 'creep':
          d.withdraw = 'viewCreepYield'
        self.view_creep = None
        return
      d.creep, d.action, d.reason = (vc['target'], VIEW_CREEP_V), 'creeping', VIEW_CREEP_REASON
      if not vc['went'] and acting and f.enabled and f.go_active == 'none' and standing:
        vc['went'] = True
        d.go_cause = 'creep'
      moved = float(getattr(ctl.state, 'moved_m', 0.))
      vc['moved'] = vc['moved'] or f.v_ego > STAND_V
      there = moved >= vc['target'] - CREEP_AT_M - 1e-6
      if (standing and (there or (vc['moved'] and self.t - vc['t'] > STOP_DWELL_S))) or self.t - vc['t'] >= DEADLOCK_CREEP_MAX_S - 1e-6:
        self.view_steps += 1
        vc.update(phase='measure', t_measure=self.t)
        if f.go_active == 'creep':
          d.withdraw = 'creepDone'
        cloudlog.event("turnpilot.stop_go_view_creep_done", key=list(self.key), crept=round(moved - vc['from'], 2), pos=round(pos, 2))
      return
    if vc['phase'] == 'measure' and vc.get('look'):
      # the long look's creep: stood, the next step where the walk predicts it gains (a step that never moved the car ends
      # it), else the window from here
      self.view_creep = None
      if not vc['moved'] or not self._view_creep_start(f, d, holds, pos, look=True):
        cloudlog.event("turnpilot.stop_go_look_crept", key=list(self.key), pos=round(pos, 2), steps=self.view_steps)
      return
    if vc['phase'] != 'measure' or self.t - vc['t_measure'] < self._need_s() - 1e-6 or not holds:
      return
    b = self._binding()
    if b is None or any(not h.key.startswith('reach:') or h.kind != STATIC for h in holds):
      self.view_creep = None  # something else now: the ladder starts over at the next bound
      return
    gained = (b.r_seen - b.r_req) - vc['margin0']
    cloudlog.event("turnpilot.stop_go_view_creep_measured", key=list(self.key), lane=[b.role, b.index], gained=round(gained, 2))
    self.view_creep = None
    # V9: under vision the steps go on while the walk predicts a gain (down to the creep floor), whatever the last measured,
    # but never after a step that didn't move the car
    if (gained < VIEW_CREEP_GAIN_M and not self.vision) or (self.vision and not vc['moved']) or not self._view_creep_start(f, d, holds, pos):
      self.view_creep = {'phase': 'over'}

  # -------------------------------------------------------------------------------------------------- withdraw and abort

  def _withdraw(self, f: Frame, d: StopGoDecision, pos: float, why: str, detail: str = '', to: float | None = None) -> None:
    """The go withdrawn before commit: G5's go ends this frame, the capped abort to M_EDGE short of dEdgeMin (or to, m past
    the stop point: short of the near crosswalk), and observe again, the rank kept (2.6); GO_RETRIES retries while GO_ROOM_M
    remains, then H5."""
    self.retries += 1
    self.withdrawn = why
    cloudlog.event("turnpilot.stop_go_withdrawn", key=list(self.key), why=why, detail=detail, pos=round(pos, 2), v=round(f.v_ego, 2),
                   go_t=round(self.t - self.go['t'], 2), retries=self.retries, takeover_s=round(self.takeover_s, 2))
    self.go = None
    self.aborting = {'to': self.edges.d_min - sr.M_EDGE if to is None else to, 'why': detail or why}
    self.state, self.clear_s = 'observe', 0.
    d.withdraw, d.prompt, d.speed_cap, d.action = why, 'waitingForCross', NO_SPEED_CAP, 'waitingCross'
    d.reason = f"Stopping short: {detail or why}"
    d.stops = [self._abort_stop(f, pos)]
    # V3: under vision a go withdrawn for traffic (or a no-lift go the model didn't take: V7) counts toward no bound, each
    # next go after a fresh window; the model not taking over a lifted go still meets GO_RETRIES (a mess-up, said)
    if self.vision and why in TAKEOVER_WITHDRAWALS and not self.no_lift:
      self.takeover_fails += 1
    n = self.takeover_fails if self.vision else self.retries
    if n > GO_RETRIES:
      self._fault('H5', f, d, f"the go was withdrawn {n} times ({why})")

  def _abort_stop(self, f: Frame, pos: float) -> Stop:
    return capped_abort(f.v_ego, self.aborting['to'] - pos, f"Stopping short: {self.aborting['why']}")

  def _threat(self, f: Frame, pos: float) -> str:
    """A tracked vehicle, or a raw vehicle box, in a conflicting lane the car hasn't cleared that reaches our conflict point (at
    v_hi, v_c while untrusted) before we clear its lane plus BAND_ABORT_MARGIN_S (2.6's in-band abort; past the in-lane point
    only said)."""
    lanes = {(ln.role, ln.index): ln for ln in self.junction_lanes(f)}
    v = max(f.v_ego, 0.5)
    # a track known at the go that hasn't moved since (a shield, a later arrival waiting at its line, a car far enough to
    # stop) is as the go found it: once our car moves stop_evidence restarts its trust, and untrusted it would read as a car
    # not stopping at v_c, so a car standing at its line turned the commonest all-way crossing into an in-band abort
    still = self._still(f)
    known = self.go['tracks'] if self.go is not None else {}
    for tid, e in self.ev.items():
      if e.lane is not None and e.lane.movement == sr.WRONG_WAY:
        if e.cls in MOTOR + CYCLISTS and self._wrong_way(f, tid, e, known):
          return f"a {e.cls} coming the wrong way {side_word(e.role)}"
        continue
      ln = lanes.get((e.role, e.index))
      if ln is None or e.cls not in MOTOR + CYCLISTS or (e.u2 == 'stopped' and e.v_trusted) or tid in still:
        continue
      far_side = ln.s_max - self.setback + sr.LANE_W_MAX / 2.
      if pos - CAR_LEN_M > far_side:
        continue
      t_clear = (far_side + CAR_LEN_M - pos) / v
      v_in = e.v_hi if e.v_trusted else sr.V_CROSS_FACTOR * ln.limit
      if v_in > 0.1 and e.d_lo / v_in < t_clear + BAND_ABORT_MARGIN_S:
        return f"a {e.cls} {side_word(ln.role)}"
    if f.live is not None:
      for ln in lanes.values():
        if pos - CAR_LEN_M > ln.s_max - self.setback + sr.LANE_W_MAX / 2.:
          continue
        need = sr.reach_needed_allway(ln.limit) if ln.stopping else sr.reach_needed_two_way(ln.limit, self.edges.d_max)
        spans = self._still_spans(f, still)
        blockers = [b for b in self._blockers(f, ln, need, sr.angle_bound(self.edges.nominal is not None), pos)
                    if any(m in b for m in MOTOR) and not raw_known(b, spans)]
        if blockers:
          return f"a vehicle {side_word(ln.role)}"
    return ''

  def _after_commit(self, threat: str) -> None:
    if self.alert != 'crossTrafficAfterCommit' or self.alert_t <= 0.:
      cloudlog.event("turnpilot.stop_go_after_commit", key=list(self.key), threat=threat, state=self.state)
    self._show_alert('crossTrafficAfterCommit')

  def _show_alert(self, alert: str) -> None:
    self.alert, self.alert_t = alert, ALERT_S

  # ------------------------------------------------------------------------------------------------------------- faults

  def _fault(self, h: str, f: Frame, d: StopGoDecision, detail: str = '') -> None:
    """A handback (7.1): held, stopSignTakeControl with its reason, until the driver takes over."""
    if self.state == 'ladder':
      return
    self.fault = h
    what = self._what() if self.vision else 'Stop sign'
    self.fault_reason = f"{what}: {detail}: take control" if h in ('H3', 'H9') else REASONS.get(h, STOP_SIGN_REASON)
    self.state = 'ladder'
    if f.go_active != 'none' and not d.withdraw:
      d.withdraw = 'fault'
    cloudlog.event("turnpilot.stop_go_fault", error=True, fault=h, detail=detail, key=list(self.key), retries=self.retries,
                   t=round(self.t, 2))
    d.prompt, d.reason, d.fault, d.ledger, d.latch, d.action = 'stopSignTakeControl', self.fault_reason, h, FAULT, 'fault', 'awaitingDriver'

  def fault_stops(self, v: float, odo: float) -> list:
    """A faulted plan's (turnpilotd.fault_limits: update() isn't run): a go in hand withdrawn (its capped abort; a fault is
    no time to cross), a capped abort in hand carried on, the decider's own hold on a standing car kept. The latch keeps its
    own (map_controls)."""
    if self.token is None:
      return []
    pos = self.pos_ref + (odo - self.odo_ref)
    if self.state == 'going':
      self.retries += 1
      self.withdrawn, self.go = 'fault', None
      self.aborting = {'to': self.edges.d_min - sr.M_EDGE, 'why': 'fault'}
      self.state, self.clear_s = 'observe', 0.
      cloudlog.event("turnpilot.stop_go_withdrawn", key=list(self.key), why='fault', pos=round(pos, 2),
                     v=round(v, 2), retries=self.retries)
    if self.aborting is not None and v >= STAND_V:
      return [capped_abort(v, self.aborting['to'] - pos, "Stopping short: fault")]
    if self.state in ('observe', 'ladder') and not self.latched and not self.fault_in_lane:
      return [hold_here(v, self.fault_reason or "Stopped at stop sign")]
    return []

  # ------------------------------------------------------------------------------------------------------------- output

  def _finish(self, f: Frame, d: StopGoDecision, acting: bool, why: str) -> None:
    if self.state in ('going', 'committed', 'inLane'):
      # D3: a person at a crosswalk's curb, seen at the go or since, keeps the launch at STOP_GO_V_MAX until our rear bumper is
      # past that crosswalk, past commit too (scene_policy's in-path stop armed; never braked for here)
      end = self.curb_cap_until()
      if end is not None:
        self.curb_until = end if self.curb_until is None else max(self.curb_until, end)
      if self.curb_until is not None and d.position_m <= self.curb_until:
        d.speed_cap = min(d.speed_cap, sr.STOP_GO_V_MAX)
    d.state, d.acting = self.state, acting
    d.alt = dict(self.alt) if self.alt is not None else None
    self.latched = f.control is not None
    d.clear_s, d.rank, d.retries, d.crept_m = self.clear_s, self.rank, self.retries, self.crept_m
    d.side_live = f.side_status in SIDE_LIVE
    d.maps_valid = f.side_status != 'mapsStale'
    d.would_go, d.takeover_s, d.withdrawn = self.would_go, self.takeover_s, self.withdrawn
    d.lanes, d.edges = list(self.last_lanes), self.edges
    if self.alert_t > 0.:
      self.alert_t = max(self.alert_t - f.dt, 0.)
      if d.alert == 'none':
        d.alert = self.alert
    if not acting and self.state in WAITING + ('approach',):
      # as shadow: the latch's own 1 s handback with the reason saying why (H4, H6, H8); nothing released, held or gone here
      d.release, d.go_cause, d.stops, d.creep, d.withdraw = None, 'none', [], None, ''
      if self.state in WAITING:
        d.latch, d.prompt, d.action = 'shadow', 'none', 'held'
        # under vision its gate's words, "Yield: ..." at a yield (V10); an H9 never falls back to STOP_SIGN_REASON
        d.reason = self.gate_reason if self.vision and self.gate_reason else REASONS.get(why, STOP_SIGN_REASON)
        d.fault = why[:2]
    d.log = {'state': self.state, 'mode': d.mode, 'why': why, 'clear_s': round(self.clear_s, 2), 'rank': self.rank,
             'retries': self.retries, 'pos': round(d.position_m, 2), 'reason': d.reason, 'would_go': d.would_go}
    if self.vision:
      d.log.update(vision=True, need_s=self.need_s, gate=self.gate[0] if self.gate else '', longLook=[r for r, _ in self.long_look])


def side_word(role: str) -> str:
  return {'left': 'from the left', 'right': 'from the right', 'opposite': 'ahead'}.get(role, role)


def raw_known(desc: str, spans: list) -> bool:
  """A raw blocker (visibility.LiveVisibility.blockers' description) that is a known, unmoved track's own box: its bearing
  within RAW_KNOWN_DEG of that track's live span (StopGo._still_spans, from where the car is now). The launch's own motion
  swings every box's bearing (a car 9 m down the near lane by 15 deg over 5 m of ours), so against the span at the go a
  standing shield's box blocked as a new one, and an 8 deg allowance on it excused other boxes beside it."""
  try:
    bearing = float(desc.split(' at ', 1)[1].split(' deg', 1)[0])
  except (IndexError, ValueError):
    return False
  return any(min(span) - RAW_KNOWN_DEG <= bearing <= max(span) + RAW_KNOWN_DEG for span in spans)
