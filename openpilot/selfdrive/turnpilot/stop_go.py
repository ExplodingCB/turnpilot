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
  the right; facing, straight or right before left; another car assumed straight).
- observe: section 2.4's go conditions every frame (go_holds: each one held a Hold with its reason and kind, the wait ledger of
  4.6). All of them for CLEAR_S is the go: map_controls' latch let go (release, cause 'clear', no span release past it), G5's
  go (go_cause stopSign) and the stopGo chime. Waiting is silent (waitingForCross, its reason) with an info chime at
  WAIT_INFO_S and every WAIT_INFO_S after while dynamic; a static wait has STATIC_WAIT_S, then the ladder, which in this group
  is H3 (its creep and other-turn rungs are SG4c's); an earlier arrival that holds its turn 3 x DEADLOCK_S is H3 (until SG4b's
  deadlock creep); a junction the route already found uncertifiable is H3 right after the full stop (7.1).
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
from openpilot.selfdrive.turnpilot.maps import surveys as survey_file
from openpilot.selfdrive.turnpilot.routing import certify as ce
from openpilot.selfdrive.turnpilot.routing.junctions import survey_approach
from openpilot.selfdrive.turnpilot.scene_policy import PATH_HALF_WIDTH_M, STOP_SPEED, Stop, stop
from openpilot.selfdrive.turnpilot.stop_evidence import APPROACH_V, DOWNGRADE_V, DOWNGRADE_WINDOW_M, StopEvidence, class_name, junction_lanes
from openpilot.selfdrive.turnpilot.turn_speed import NO_SPEED_CAP

MODES = ce.MODES                # off, shadow, trial, auto (TurnPilotStopSignGo)
MODE_DEFAULT = 'auto'           # decision D4: acting only with its data, else as shadow, said
ACTING = ce.ACTING

# section 9
STOP_DWELL_S, LINE_M = 1.0, 3.0
STAND_V = 0.1                   # m/s: standing (a full stop)
DET_FAULT_S, FAULT_GRACE_S = 2.0, 10.0
LEAD_GO_M = 8.
BIKE_PASS_MEMORY_S = 10.
HIDE_S = 2.0
TIE_S = 1.0
DEADLOCK_S = 3.0
DEADLOCK_H3_S = 3. * DEADLOCK_S  # an earlier arrival holding its turn this long, nothing moving: H3 (until SG4b's creep)
FRONT_ZONE_M, FRONT_ZONE_MAX_S = 3.0, 30.
FRONT_ZONE_HALF_W = 1.2         # m either side of the axis the front zone reaches (a 1.85 m car and a margin)
FRONT_EDGE_M = 0.6              # m inside its side edge a track that ended there ended at the edge...
FRONT_LEAVE_V = 0.5             # ...moving sideways over this...
FRONT_LEAVE_S = 1.0             # ...and a person track starting outside on that side within this: the same person, left
EXIT_ROOM_M = 4.7 + 2.0         # the car and 2 m past the exit crosswalk
CROSSWALK_M = 3.0               # m a crosswalk is deep
CAR_LEN_M = 4.7
BOX_HOLD_M = 6.                 # any box overlapping 0-6 m ahead of the bumper holds the go (V13)
ROLLUP_GOES = 4
ROLLUP_V_MAX = 3.0              # m/s: a roll-up's cap is the map curve alone, up to this
ROLLUP_WAIT_S = 15.             # s standing short of the line with no lead within LEAD_GO_M, its rollUp goes spent: H5 (4.6)
GO_RETRIES = 2
UNCOMMIT_STOP_S = 0.5
BAND_ABORT_MARGIN_S = 1.0
STALL_BOX_S, CLEAR_BOX_GOES = 2.0, 2
STATIC_WAIT_S, WAIT_INFO_S, CAL_WAIT_S = 15., 60., 30.
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
}
DYNAMIC, STATIC, FAULT = 'dynamic', 'static', 'fault'


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
  mode: str = MODE_DEFAULT
  mode_known: bool = True        # the build's params know TurnPilotStopSignGo, and its value is a mode
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
  never a hard stop, shouldStop once slow."""
  need = -v * v / (2. * max(d_remaining, 0.5))
  return Stop(max(need, -sr.A_ABORT), 0., v < STOP_SPEED, reason, max(d_remaining, 0.), confirmed=False)


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
    self.rank = 0
    self.turn_wait_s = 0.
    self.clear_s = 0.
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

  def _take(self, ctl, f: Frame) -> None:
    """A new encounter with control ctl (held, or engaged ahead)."""
    self.reset()
    self.token, self.key, self.control = ctl.state, ctl.key, ctl
    self.junction = f.junction
    if f.vis is not None and getattr(f.vis, 'setback', None) is not None:
      self.setback = float(f.vis.setback)
    j = self.junction
    if j is not None and j.status == 'ok':
      entries = list(getattr(f.surveys, 'entries', f.surveys) or [])
      self.entry = survey_file.match(entries, j.osm_node_id, (j.lat, j.lon)) if entries else None
      ours = survey_approach(self.entry, j.our_way, j.heading_in) if self.entry is not None else None
      self.lane_edge = (ours or {}).get('laneEdgeM')
      self.edges = sr.edge_band(j.stop_point_distance, ce.cross_lanes(j), self.lane_edge)
      self.lanes_all = junction_lanes(j, 'unknown', self.setback, self.lane_edge)
      self.watch_lanes = sr.wrong_way_lanes(j, self.edges, self.setback)
      lines = {}
      for ln in self.lanes_all + self.watch_lanes:
        a = ln.approach
        sv = survey_approach(self.entry, a.way_id, (a.bearing + 180.) % 360.) if self.entry is not None and a is not None else None
        line = (sv or {}).get('stopPointM') or (sv or {}).get('lineM')
        if line:
          lines[ln.role] = float(line)
      self.evidence.set_junction(self.lanes_all + self.watch_lanes, sr.angle_bound(self.edges.nominal is not None), lines)
    cloudlog.event("turnpilot.stop_go_control", key=list(ctl.key), d=round(float(ctl.d), 2), held=bool(getattr(ctl, 'held', False)),
                   junction=j.status if j is not None else 'none', osmNodeId=j.osm_node_id if j is not None else 0)

  # ------------------------------------------------------------------------------------------------------------- update

  def takes(self, ctl, ahead) -> bool:
    """Whether this frame starts a new encounter: a stop control the latch holds now always does; one engaged ahead only
    while the encounter in hand is idle (none, still approaching, or done). Once the latch has let go the decider's own hold
    or its handback is all that holds the car (observe, ladder), and while it launches it is crossing: another stop sign
    within STOP_SWEEP_M (a frontage stop 15-30 m on, a divided road's far line) waits until this one is done."""
    cur = ctl or ahead
    return cur is not None and cur.state is not self.token and (ctl is not None or self.state in IDLE)

  def update(self, f: Frame) -> StopGoDecision:
    ctl = f.control if f.control is not None and f.control.kind == 'stop' else None
    ahead = f.approach if f.approach is not None and f.approach.kind == 'stop' else None
    cur = ctl or ahead
    if self.takes(ctl, ahead):
      self._take(cur, f)
    elif cur is None and self.token is not None and self.state in ('approach', 'done') and ctl is None:
      self.reset()
    if self.token is None:
      return StopGoDecision(state='off', mode=self._mode(f)[0])
    if cur is not None and cur.state is self.token:
      self.key, self.control = cur.key, cur
    self.t += f.dt
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
    hold of the decider's own (the latch already let go) ends once they drive off."""
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
    if acting and standing and self.stand_s >= STOP_DWELL_S and free and -pos > LINE_M and \
       f.go_active == 'none' and self.rollups < ROLLUP_GOES and f.enabled:
      self.rollups += 1
      self.stand_s = 0.
      d.go_cause = 'rollUp'
      cloudlog.event("turnpilot.stop_go_rollup", key=list(self.key), d=round(-pos, 2), n=self.rollups, latched=False)

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
    if standing and self.stand_s >= STOP_DWELL_S and f.go_active == 'none' and self.rollups < ROLLUP_GOES and f.enabled:
      self.rollups += 1
      self.stand_s = 0.
      d.go_cause = 'rollUp'
      cloudlog.event("turnpilot.stop_go_rollup", key=list(self.key), d=round(-pos, 2), n=self.rollups, latched=True)

  def _arrive(self) -> None:
    """Our arrival, at the full stop: every vehicle standing at another approach's line is earlier (V5)."""
    self.arrived_t = self.t
    for tid, e in self.ev.items():
      if e.stopped_at_line:
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
    holds = self.go_holds(f, pos, launch=False)
    d.holds = holds
    self.clear_s = self.clear_s + f.dt if not holds else 0.
    self._ledger(f, d, holds, acting)
    if self.state != 'observe':
      return
    if ctl is None:
      d.stops.append(hold_here(f.v_ego, d.reason or "Stopped at stop sign"))  # the decider's own hold: the latch let go already
    go = not holds and self.clear_s >= sr.CLEAR_S - 1e-6
    self.would_go = go
    if go and not self.would_go_logged:
      self.would_go_logged = True
      cloudlog.event("turnpilot.stop_go_would_go", key=list(self.key), acting=acting, why=why, t=round(self.t, 2), pos=round(pos, 2))
    if not go or not acting:
      return
    room = self.edges.d_min - pos
    need = sr.go_room_m(f.t_abort, right=f.maneuver == 'right')
    if room < need - 1e-6:
      self._fault('H5', f, d, f"no room for a go: {room:.2f} m to the closest edge, {need:.2f} needed")
      return
    self.go = {'pos': pos, 't': self.t, 'reach': {(x.role, x.index): x.r_seen for x in self.last_lanes},
               'tracks': self._known_tracks(f)}
    self.takeover.reset()
    self.takeover_s, self.uncommit_s = -1., 0.
    self.state, self.clear_s, self.went, self.ped_carry_on = 'going', 0., True, False
    self.static_s = self.dynamic_s = 0.
    self.next_chime = WAIT_INFO_S
    d.release = (self.key, 'clear') if ctl is not None else None
    d.go_cause = 'stopSign'
    d.prompt, d.reason, d.action, d.ledger = 'stopGo', "Going", 'going', 'none'
    d.stops = []
    d.speed_cap = sr.STOP_GO_V_MAX
    cloudlog.event("turnpilot.stop_go_go", key=list(self.key), pos=round(pos, 2), room=round(room, 2), retries=self.retries,
                   rank=self.rank, lanes=[(x.role, x.index, round(x.r_seen, 1), round(x.r_req, 1), x.shielded) for x in self.last_lanes])

  def _going(self, f: Frame, d: StopGoDecision, pos: float) -> None:
    g = self.go
    d.detector_view, d.action, d.prompt, d.reason = 'stopSweep', 'going', 'stopGo', "Going"
    d.speed_cap = sr.STOP_GO_V_MAX
    d.go_t = self.t - g['t']
    if f.model_v2 is not None and self.takeover.update(f.model_v2, f.dt) and self.takeover_s < 0.:
      self.takeover_s = d.go_t
    if f.go_taken_over and self.takeover_s < 0.:
      self.takeover_s = d.go_t
    taken_over = self.takeover_s >= 0.
    a_now = max(f.a_ego, 0.)
    if sr.committed(pos, f.v_ego, f.t_abort, self.edges, a_now):
      self.state, self.uncommit_s = 'committed', 0.
      d.detector_view, d.action, d.speed_cap, d.prompt, d.committed = 'stopSweepGo', 'committed', NO_SPEED_CAP, 'none', True
      cloudlog.event("turnpilot.stop_go_committed", key=list(self.key), pos=round(pos, 2), v=round(f.v_ego, 2), taken_over=taken_over,
                     go_t=round(d.go_t, 2))
      return
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
    elif not taken_over and (d.go_t >= GO_WINDOW_S - 1e-6 or (f.go_active == 'none' and f.go_end == 'window')):
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
    if pos - CAR_LEN_M > self._far_edge() or abs(f.turned_deg) >= CROSS_DONE_YAW_DEG:
      self.state, self.go = 'done', None
      d.detector_view = 'auto'
      cloudlog.event("turnpilot.stop_go_done", key=list(self.key), pos=round(pos, 2), turned=round(f.turned_deg, 1))
      return
    threat = self._threat(f, pos)
    if threat:
      self._after_commit(threat)  # never braked for: stopping in a lane is worse than going on (V8)
    standing = f.v_ego < STAND_V
    free = not f.tp_holding and (f.lead_m is None or f.lead_m > LEAD_GO_M)
    self.stall_s = self.stall_s + f.dt if standing and free else 0.
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

  def _ladder(self, f: Frame, d: StopGoDecision, ctl) -> None:
    """A fault's handback: held, said, until the driver takes over (openpilot disengaged) and drives off."""
    d.latch, d.detector_view, d.action = 'fault', 'stopSweep', 'awaitingDriver'
    d.prompt, d.reason, d.fault, d.ledger = 'stopSignTakeControl', self.fault_reason, self.fault, FAULT
    if ctl is None:
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
    return junction_lanes(j, f.maneuver, self.setback, self.lane_edge, f.approach_yaw_deg)

  def certified(self, f: Frame):
    """routing/certify.assess at the line (the same function navd runs at route time; its mode and launch checks are _mode's):
    the static checks, cached while their inputs stand still."""
    key = (f.maneuver, round(f.approach_yaw_deg), round(max(abs(f.yaw20_deg), sr.ROUTE_YAW_DEG), 1), f.side_status in SIDE_LIVE,
           None if f.wall_t is None else int(f.wall_t // 60.), id(f.vis), id(f.surveys), id(self.junction))
    if key != self._cert_key:
      rt = ce.Runtime(mode='trial', side_views=f.side_status in SIDE_LIVE, window=(0., 0.), yaw_deg=f.approach_yaw_deg,
                      yaw_offset_deg=max(abs(f.yaw20_deg), sr.ROUTE_YAW_DEG))
      self._cert_key, self._cert = key, ce.assess(self.junction, f.maneuver, f.vis, f.surveys, f.wall_t, rt)
    return self._cert

  def go_holds(self, f: Frame, pos: float, launch: bool) -> list[Hold]:
    """Every go condition of 2.4 not met now, a Hold each (the wait ledger reads them). launch: re-checked during the launch
    (2.6): no full stop or turn order, reach on the go position's geometry, tracks known at the go by their ground motion."""
    holds: list[Hold] = []
    j = self.junction
    # H2's triggers, and 10: the junction, the data
    if j is None or j.status != 'ok':
      return [Hold('position', "the junction's position isn't known", FAULT)]
    if f.position_spread > H2_SPREAD_M:
      return [Hold('position', f"its distance read {f.position_spread:.1f} m apart", FAULT)]
    if self.line_fault:
      return [Hold('position', self.line_fault, FAULT)]
    # an approach downgraded on site this encounter (4.4): not stopping, so no maneuver here certifies (the union rule
    # leaves none that conflicts with no approach); before commit a go out is withdrawn for it, never once committed
    if self.downgraded:
      conflicting = {id(m.approach) for m in sr.conflict_movements(j, f.maneuver, f.approach_yaw_deg)}
      if conflicting & set(self.downgraded):
        return [Hold('downgrade', "cross traffic doesn't stop", STATIC)]
    verdict = self.certified(f)
    if verdict.certification != 'certifiable' and verdict.reason != 'reach':
      return [Hold('cert:' + verdict.reason, f"can't certify ({verdict.reason}: {verdict.detail})" if verdict.detail else
                   f"can't certify ({verdict.reason})", STATIC)]
    # 9 a signal head at the junction (C20)
    if self.signal_seen:
      holds.append(Hold('signal', "signal at junction", STATIC))
    # 3 straight on its approach, no turn desire before our first go (our own go's pulse is that go's: after a withdrawn go the
    # yaw and the wheel say whether it curled the car)
    if not launch and (abs(f.yaw20_deg) > YAW_GO_DEG or abs(f.steer_deg) > STEER_GO_DEG or (f.desire_out and not self.went)):
      holds.append(Hold('yaw', "the car isn't straight on its approach", STATIC))
    # 2 the views the maneuver needs, fresh on camera time (the side views while the speed gate runs them); E7, E8
    holds += self._view_holds(f)
    # 4 every conflicting lane: reach or a shield, its tracks, its raw boxes
    holds += self._lane_holds(f, pos, launch)
    # 5 our turn
    if not launch:
      holds += self._turn_holds(f)
    # 6 the box; 7 people, the front zone, the exit; 8 a lead, a right's blind spot and a cyclist passed on the right
    holds += self._box_holds(f, pos)
    holds += self._people_holds(f, pos)
    holds += self._exit_holds(f, pos)
    if f.lead_m is not None and f.lead_m <= LEAD_GO_M:
      holds.append(Hold('lead', "a queue ahead", DYNAMIC))
    if f.maneuver == 'right':
      if f.right_blindspot:
        holds.append(Hold('blindspot', "something in the right blind spot", DYNAMIC))
      if self.bike_t is not None and self.t - self.bike_t <= BIKE_PASS_MEMORY_S:
        holds.append(Hold('bikePassed', "a cyclist passed on the right", DYNAMIC))
    return holds

  def _view_holds(self, f: Frame) -> list[Hold]:
    """2.4 item 2, E7, E8: a needed view older than DET_FRESH_S of camera time resets CLEAR_S; none for DET_FAULT_S on the
    decider's clock is the camera paused (held: FAULT_GRACE_S, then H1); the side views' maps waiting for a calibration move,
    a dynamic wait of up to CAL_WAIT_S."""
    out = []
    side = f.v_ego <= sr.STOP_GO_V_MAX and f.side_status in SIDE_LIVE
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

  def _lane_holds(self, f: Frame, pos: float, launch: bool) -> list[Hold]:
    """2.4 item 4: each conflicting lane's R_seen at least its R_req (or a qualified shield), its tracks passing U2 (a stopping
    approach's) or the tracked test (another's, and a cyclist on any), and no raw box blocking it. During the launch reach is
    walked on the go position's geometry and held to the smaller of its go-time and live value (D13), shields and the tracks
    known at the go are judged by their ground motion, and any other track or raw box in a lane withdraws the go."""
    holds: list[Hold] = []
    lanes = self.junction_lanes(f)
    self.last_lanes = []
    if not lanes:
      return [Hold('noLanes', "no conflicting approach to judge by", STATIC)]
    if f.live is None or f.vis is None:
      return [Hold('camera', "no live view", FAULT)]
    row, low = self._lighting(f)
    stamp = ce.pick_stamp(self.entry, row, self.junction.heading_in) if self.entry is not None else None
    at = self.go['pos'] if launch and self.go is not None else pos
    rot = sr.angle_bound(self.edges.nominal is not None, max(abs(f.yaw20_deg), sr.ROUTE_YAW_DEG))
    rep = f.live.lanes(lanes, -at, f.vis, rot_deg=rot, edges=self.edges, lighting=row if row in ('day', 'sunAhead') else 'night',
                       low_sun_sides=low, entry=self.entry, stamp=stamp, tracks=f.tracks or (), evidence=self.ev)
    known = self.go['tracks'] if launch and self.go is not None else {}
    still = self._still_spans(f, known) if launch else []
    for ln, seen in zip(lanes, rep.lanes, strict=True):
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
      self.last_lanes.append(LaneStatus(ln.role, ln.index, idx, r_seen, r_req, seen.why, shielded))
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

  def _turn_holds(self, f: Frame) -> list[Hold]:
    """2.4 item 5 (4.5 steps 2, 4): every earlier arrival gone (seen leaving its line: then the box and its lane judge it), no
    tie lost. A vehicle stopping at another line within TIE_S after our arrival is a tie: we yield to the right and, facing
    each other, a left yields to the other's straight or right (others are assumed straight); one later waits for us."""
    for tid, e in self.ev.items():
      if e.stopped_at_line and tid not in self.earlier and tid not in self.later and self.arrived_t is not None:
        since = self.stopped_since.get(tid, self.t)
        if since - self.arrived_t <= TIE_S and (e.role == 'right' or (e.role == 'opposite' and f.maneuver == 'left')):
          self.earlier[tid] = since
          cloudlog.event("turnpilot.stop_go_tie", key=list(self.key), track=tid, role=e.role, yielded=True)
        else:
          self.later.add(tid)
    waiting = []
    for tid in sorted(self.earlier, key=lambda k: self.earlier[k]):
      if tid in self.gone:
        continue
      e = self.ev.get(tid)
      if e is not None and not e.stopped_at_line and (e.v_nom > sr.SHIELD_V or e.front_from_line_m > sr.SHIELD_PAST_M):
        self.gone.add(tid)
        continue
      waiting.append(tid)  # still at its line, or unseen without having been seen leaving it: never presumed gone
    self.rank = len(waiting)
    if not waiting:
      self.turn_wait_s = 0.
      return []
    moving = any(e.v_nom > MOVING_V for e in self.ev.values())
    self.turn_wait_s = 0. if moving else self.turn_wait_s + f.dt
    if self.turn_wait_s >= DEADLOCK_H3_S - 1e-6:
      return [Hold('deadlock', "an earlier arrival isn't taking its turn", STATIC)]
    return [Hold('turn', "an earlier arrival's turn", DYNAMIC)]

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

  def _people_holds(self, f: Frame, pos: float) -> list[Hold]:
    """2.4 item 7, the conservative zones (P1, P2, P10): any person track or raw person box in the near or far crosswalk band
    (our half of the road plus one lane, its ranging band of plus or minus PED_RANGE_BAND included) or in the box blocks the
    go; moving, a dynamic wait, standing, a static one (the curb rule is SG4c's). A person heading for a crosswalk on our path
    (the arrival clause, P16: _arrival_holds). The front zone (P6)."""
    holds = []
    near0, near1 = self._x(0., pos) - 1., self._x(self.edges.d_max, pos)
    box0, box1 = self._x(self.edges.d_min, pos), self._x(self._far_edge(), pos)
    far0, far1 = box1, box1 + CROSSWALK_M + 1.

    def zone(x_lo, x_hi, y) -> str:
      if near0 <= x_hi and x_lo <= near1 and -HALF_RIGHT_M <= y <= HALF_LEFT_M:
        return 'the crosswalk'
      if box0 <= x_hi and x_lo <= box1 and -BOX_RIGHT_M <= y <= BOX_LEFT_M:
        return 'the junction'
      if f.maneuver in ('straight', 'unknown') and far0 <= x_hi and x_lo <= far1 and -HALF_RIGHT_M <= y <= HALF_LEFT_M:
        return 'the far crosswalk'
      return ''

    for o in f.objects or ():
      if class_name(o.cls) != PERSON:
        continue
      x, y = float(o.x), float(o.y)
      where = zone(x * (1. - PED_RANGE_BAND), x * (1. + PED_RANGE_BAND), y)
      if where:
        v = math.hypot(float(o.vx), float(o.vy))
        holds.append(Hold(f'ped:{o.trackId}', f"a person in {where}", DYNAMIC if v > PED_STILL_V else STATIC))
    if f.live is not None:
      for key, frame in f.live.raw.items():
        if key == 'far' or not f.live.fresh_key(key):
          continue
        for b in frame.boxes:
          if b.cls == PERSON and b.conf >= RAW_PERSON_CONF and b.xy is not None:
            x, y = float(b.xy[0]), float(b.xy[1])
            if zone(x * (1. - PED_RANGE_BAND), x * (1. + PED_RANGE_BAND), y):
              holds.append(Hold(f'pedRaw:{key}', "a person at the crosswalk", DYNAMIC))
              break
    holds += self._arrival_holds(f, pos, ((near0, near1, self.edges.d_min, 'near'),) +
                                 (((far0, far1, self._far_edge() + CROSSWALK_M + 1., 'far'),) if f.maneuver in ('straight', 'unknown') else ()))
    holds += self._front_zone(f)
    return holds

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
    RAW_PED_OF runs of its view, that would reach a crosswalk on our path (walks: (x0, x1 camera x, its far edge m past the stop
    point, which)) before our rear bumper has cleared it plus PED_ARRIVE_MARGIN_S blocks the go: t_arr = d_lo / v_hi, d_lo its
    distance to the crosswalk at the near end of its ranging band, v_hi its speed toward it plus 2 sigma once trusted
    (VEL_TRUST_S with VEL_TRUST_SAMPLES), PED_V_UNTRUSTED before. One moving away doesn't block; one standing still (under
    PED_STILL_M in PED_STILL_S) is D3's (P3). A dynamic wait; during the launch the re-check withdraws the go (_going)."""
    holds = []
    clears = [(x0, x1, self.t_clear(f, pos, edge) + PED_ARRIVE_MARGIN_S, which) for x0, x1, edge, which in walks]

    def dist(x, y, x0, x1) -> float:
      dx = max(x0 - x, 0., x - x1)
      dy = max(-HALF_RIGHT_M - y, 0., y - HALF_LEFT_M)
      return math.hypot(dx, dy)

    def d_lo(x, y, x0, x1) -> float:
      return min(dist(k * x, k * y, x0, x1) for k in (1. - PED_RANGE_BAND, 1. - PED_RANGE_BAND / 2., 1., 1. + PED_RANGE_BAND / 2.,
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
      for x0, x1, need, which in clears:
        lo = d_lo(x, y, x0, x1)
        if lo <= 0.:
          # its ranging band (along the ray: range and offset together) reaches into the crosswalk, wherever it walks
          holds.append(Hold(f'{key}:{which}', f"a person at the {'far ' if which == 'far' else ''}crosswalk", DYNAMIC))
          break
        if window is None:
          v_hi = PED_V_UNTRUSTED
        else:
          ts = [s[0] for s in window]
          ds = [dist(s[1] - pos, s[2], x0, x1) for s in window]
          t_bar = sum(ts) / len(ts)
          stt = sum((t - t_bar) ** 2 for t in ts)
          slope = sum((t - t_bar) * (d - sum(ds) / len(ds)) for t, d in zip(ts, ds, strict=True)) / max(stt, 1e-9)
          res = [d - (sum(ds) / len(ds) + slope * (t - t_bar)) for t, d in zip(ts, ds, strict=True)]
          sigma = math.sqrt(sum(r * r for r in res) / max(len(res) - 2, 1) / max(stt, 1e-9))
          v_hi = -slope + 2. * sigma
        if v_hi <= 0.05:
          continue  # moving away from it, or along it
        if lo / v_hi < need:
          holds.append(Hold(f'{key}:{which}', f"a person heading for the {'far ' if which == 'far' else ''}crosswalk", DYNAMIC))
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
        holds.append(Hold(f'exit:{o.trackId}', "no room past the junction", STATIC))
    return holds

  # ---------------------------------------------------------------------------------------------------------- the ledger

  def _ledger(self, f: Frame, d: StopGoDecision, holds: list[Hold], acting: bool) -> None:
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
    kinds = {h.kind for h in holds}
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
    deadlock = next((h for h in holds if h.key == 'deadlock'), None)
    if deadlock is not None and acting:
      self._fault('H3', f, d, deadlock.reason)
      return
    if DYNAMIC in kinds:
      # something moving: waiting is right, with no bound; a static cause starts its clock again once nothing moves
      d.ledger = DYNAMIC
      self.static_s = 0.
      if self.chime_t > 0.:
        self.chime_t = max(self.chime_t - f.dt, 0.)
        d.chime, d.reason = True, "Waiting for cross traffic"
      return
    self.chime_t = 0.
    d.ledger = STATIC
    self.static_s += f.dt
    route_known = f.route_cert == 'uncertifiable' and top.key.startswith('cert:')
    if acting and (route_known or self.static_s >= STATIC_WAIT_S - 1e-6):
      self._fault('H3', f, d, "this stop needs you" if route_known else top.reason)

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
    if self.retries > GO_RETRIES:
      self._fault('H5', f, d, f"the go was withdrawn {self.retries} times ({why})")

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
    self.fault_reason = f"Stop sign: {detail}: take control" if h == 'H3' else REASONS.get(h, STOP_SIGN_REASON)
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
    if self.state in ('observe', 'ladder') and not self.latched:
      return [hold_here(v, self.fault_reason or "Stopped at stop sign")]
    return []

  # ------------------------------------------------------------------------------------------------------------- output

  def _finish(self, f: Frame, d: StopGoDecision, acting: bool, why: str) -> None:
    d.state, d.acting = self.state, acting
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
        d.reason = REASONS.get(why, STOP_SIGN_REASON)
        d.fault = why[:2]
    d.log = {'state': self.state, 'mode': d.mode, 'why': why, 'clear_s': round(self.clear_s, 2), 'rank': self.rank,
             'retries': self.retries, 'pos': round(d.position_m, 2), 'reason': d.reason, 'would_go': d.would_go}


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
