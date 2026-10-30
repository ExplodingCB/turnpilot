"""Map stop control: stop signs, yields and traffic signals on the route (navInstruction.controls), turned into stops.

The driving model doesn't stop for them (BeamNG, 2026-10-27: 0 of 31 reds, full stops at 2 of 8 stop signs), so
TurnPilot does, from the map. Each control becomes a scene_policy.Stop, composed with the pedestrian, oncoming and turn
limits by ScenePolicy (its reason, holding and the accel cap's jerk ramp). It runs every frame, whatever the route's
maneuver is: a skipped turn or a straight-through junction keeps its stop.

The rules (Kellen, 2026-10-27; /private/tmp/claude-501/stop_design.md):
- The approach: a control engages within engage_distance (braking at MAP_STOP_DECEL from v, a second's lag, a margin).
  From then on a speed cap on a MAP_STOP_DECEL curve to the stop point, which only ever comes down (a cap that rises and
  falls with a GPS fix hunts), re-seeded only at standstill short of the latch (stopped in a queue: the lead leaves, the
  car creeps up to the line). An accel cap where the curve is late, never past MAP_DECEL_MAX (-2.0 m/s^2, the speed
  cap's own floor): a map stop never takes stop()'s hard-stop path. A stop that needs more than that, checked when the
  control is first seen, when the driver's gas comes off, and after a jump in its distance, brakes at -2.0 and says so
  (cannotStopComfortably, repeating while it lasts). Never harder, never silent.
- The latch (HELD): slower than LATCH_SPEED within LATCH_NEAR_M of the stop point (LATCH_PAST_M past it), the car is held:
  shouldStop, speed cap 0. The latch is the policy's and position-free: GPS fixes, nav gaps, reroutes, a skipped turn,
  a handback and a fault leave it be (turnpilotd keeps this policy through a fault). Only its release rule lets go, and
  only TurnPilot turned off clears it. A reroute's same control takes the latch over under its own key. A held car gone
  more than DRIVE_OFF_M at more than COMMIT_SPEED is let go: the driver's, once the hold has handed back (driverWent), or
  under the driver's gas (drivenOff); otherwise, with no go at all (the plan not applied: shadow mode turned on), an
  overrun. The latch is never replayed down the road. A car stopped further out (a queue) isn't latched: the MPC holds
  it behind the lead.
- No gas go anywhere (Kellen, 2026-10-28; DESIGN phase 4, G1). Any gas press disengages (DisengageOnAccelerator, on by
  default): it is the driver taking over, never TurnPilot's go, and releases nothing. A hold TurnPilot has no rule to
  release ends in a loud handback, the car held while it lasts: a stop sign after STOP_SIGN_HANDBACK_S stopped
  (stopSignTakeControl, "Stop sign: take control to proceed"), an unprotected turn at once at its hold line
  (unprotectedTurnTakeControl) unless the autonomous left holds it. Releasing a control releases those RELEASE_SPAN_M
  past it too (the far side of a divided road), but never past the stop-sign decider's 'clear': each line has its own.
- A held stop sign under the stop-sign decider (stop_go.py, STOP_SIGNS.md v3 SG4a, through turnpilotd; update()'s stop, a
  StopCommand): wait holds it with no handback while the decider waits (its reason and action shown), fault hands it back
  now (stopSignTakeControl, the decider's reason: its faults only), shadow (TurnPilotStopSignGo shadow or off, auto without
  its data, H4, H6, H8) is the 1 s handback above exactly as before, its reason saying why. Without a command it is shadow.
- One release path (held_control(), update()'s release, a (key, cause)): the autonomous left's go (cause gap) and the
  decider's (cause clear) let the latch go the same way, in the frame they are given. The latch's stop point moves on by
  update()'s creep, (m past where it latched, its speed cap): the left's creep for a view (CREEP_SPEED) and the decider's
  roll-up to its stop point (the map curve, up to its cap), shouldStop false, held again within CREEP_AT_M.
- A signal adds nothing (Kellen, 2026-10-28: through greens without slowing; DESIGN phase 4, G1): no stop, no cap, no
  prompt, unless an accepted light source reads red or yellow for it (the autonomous left standing at its line is the one
  exception: it goes only on a green read, turn_light). The driving model's own braking is the car's
  red-light handling; the map signal is only where a light is looked for. On the car this week no light source is
  accepted (the camera's reader ships in shadow), so a mapped signal never slows the car; in the sim the light oracle
  stands in for it, so a sim red still stops. A red is a stop on the MAP_STOP_DECEL curve, seeded where it is first read
  (the curve from engage_distance if it is further out), never harder than MAP_DECEL_MAX: one that needs more brakes at
  that floor, cannotStopComfortably. A yellow is decided once: stop when the stopping distance at YELLOW_DECEL after T_LAT
  fits (YELLOW_DECEL, TurnPilotYellowDecel, no more than -MAP_DECEL_MAX); otherwise committed, with proceedingOnYellow,
  and a later red never reverses it. A stop is released only by a steady green (GREEN_STEADY_S): a light gone unknown
  keeps it. The driver's gas never releases one.
- The camera's light reader (DESIGN phase 4 G2, detector/light_state.py, its heads in turnpilotDetections.lights) is a
  second source that may only ever add a stop, and this week it adds none: in shadow (TurnPilotCameraLights shadow, the
  default) its governing heads are associated with the signals (a head from CAMERA_ASSOC_BEFORE_M short of a signal's
  line to CAMERA_ASSOC_PAST_M past it is that signal's, the last line before it winning) and what it would do is logged
  on every change (turnpilot.camera_light_shadow; turnpilot.camera_light_unmapped for a red or yellow with no mapped
  signal: no synthetic control, CRITIQUE cut), and nothing it reads reaches a stop, a cap, a prompt or a release: the one
  thing it gates is the autonomous left's go (turn_light, its red too: keeping the car standing where it is never brakes
  it), which at a signal needs a green read from some source (unprotected_left.SIGNAL_NEEDS_GREEN). Its
  reader turned off for the drive is said once (lightSourceRejected, "Camera light reader off for this drive"), on its
  own hold and only where no other alert is said, so it never mutes a take-control, overrun or yellow alert. Add-only
  needs both keys, modeld's message not shadow and turnpilotd's own TurnPilotCameraLights addOnly: then a camera red
  raises a signal's light to red, and a camera yellow to yellow only while a yellow stop at it is still feasible (a
  yellow too late to stop for would commit the signal to go, and a commit outlasts a later red: a removed stop). A
  camera green or unknown never enters, so a camera red can hold a stop the oracle's green would have released, never
  the reverse; and a stop the camera added is released only by an accepted green once the camera's red has gone (on the
  car, where no other source is accepted, that is never: the stand handback ends it, which is why add-only stays off).
- A mapped stop the camera never saw (row C11; STOP_SIGNS.md v3 SG4b) is stopped for anyway, the gentle error, and the
  decider certifies it as mapped; at its hold its approach's class-11 shadow evidence is logged (update()'s signs: the
  forward runs counted from SIGN_LOOK_M, their stop-sign boxes, the best conf; turnpilot.map_stop_signs), and SIGN_UNSEEN_RUNS
  or more with none is said as unseen (turnpilot.map_stop_unseen) for the replay gate. Nothing here acts on a sign.
- A yield caps the speed to YIELD_SPEED at its line, never stops the car, and goes once passed. One whose
  road outranks every crossing one (crossSuspect: likely mapped for the cross road) is dropped and logged. Such a stop
  sign still stops, and says the approach is unverified.
- The driver's gas before a stop sign is an override; when it comes off past the line moving faster than COMMIT_SPEED,
  or past where the latch reaches (LATCH_PAST_M) at any speed, the stop is dropped (stopOverrun), else the curve resumes
  from there (re-seeded, checked for feasibility). Past the line beyond the latch's reach with no release, at any speed,
  is an overrun: dropped, logged, alerted, never a stop in the junction with nothing to release it. Driven off a held stop
  under the driver's foot is the same.
- An unprotected turn (Kellen, 2026-10-27: the same rule as stop signs and lights) is a routed far-side turn (a left on the
  right) across an oncoming approach (navInstruction.maneuverOpposing present, or unknown: no junction data is never
  clear) with no stop sign of its own at the junction (one within PROTECT_BEFORE_M before it to PROTECT_PAST_M past it: a
  stop sign already stops; the maneuver stays protected once seen so, after its control drops out behind the car). A
  signal there doesn't protect it (SIGNAL_PROTECTS_LEFT): a left on a green ball is unprotected. turnpilotd hands it in each frame
  (update()'s turn) and it becomes a control of its own, kind unprotectedTurn, at the junction's hold line less
  MAP_MARGIN_M: the curve, the latch and the stop sign's rules. Held, it does what the unprotected left asks (update()'s
  left, a LeftCommand from unprotected_left.py through turnpilotd; TurnPilotAutoLeft): auto (the default; Kellen,
  2026-10-28, "No it has to ship taking the turn", no intervention) holds it, never handing back but for the left's fault
  (leftUndecidedTakeControl), creeps its stop point on to where the left asks (CREEP_SPEED, CREEP_ACCEL; still the latch,
  never past it), and releases it on the left's go (cause gap); announce and off hand it back at once
  (unprotectedTurnTakeControl), announce saying whether it would have gone.
  A failing or missing controls status leaves the turn unprotected. One the
  car isn't held at goes with its maneuver (skipped too fast, rerouted away, passed), never with a fault (it carries on,
  dead-reckoned); a held one is position-free like any latch. Its junction is the nearest of navd's readings
  (route_follower.py), so a new one can put the line behind where the car can stop at MAP_DECEL_MAX: it is never dropped
  as an overrun then (crossed blind), but moved up to where the car stops braking at that floor, no harder, said out
  loud (cannotStopComfortably), and handed back there like any other. scene_policy.py's oncoming and pedestrian holds
  only add holds.
- Off the route with a destination, the controls are navd's lookup ahead of the car (control_tracker.py), under the same
  rules. With nothing known ahead past the tracker's grace (off_route_unknown: the lookup pending, failed, unsupported,
  partial, or stale), the car never drives on silently: a speed cap comes down from its speed at OFF_ROUTE_DECEL to
  OFF_ROUTE_SPEED, and offRouteStopsUnknown says so ("Off route, map stops unknown"), for as long as it lasts. Never a stop:
  nothing is known to stop for, and the driver is asked to take control or wait for the reroute.
- navd's controlsStatus other than ok (navd always sets it; none is a navd from before controls) raises
  mapControlsUnavailable and keeps "Map stops/signals: <status>" in the reason: no controls on tiles without control
  data is never shown as clear.
"""
import math
from dataclasses import dataclass, field

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.control_tracker import JUMP_M, ControlTracker, TrackedControl
from openpilot.selfdrive.turnpilot.hooks import ABORT_HOLD_S, SPEED_CAP_ACCEL_MIN
from openpilot.selfdrive.turnpilot.route_follower import LEFT, TURN
from openpilot.selfdrive.turnpilot.scene_policy import HARD_STOP_MIN_M, HOLD_LINE_M, NO_ACCEL_CAP, STOP_LAG_S, Stop
from openpilot.selfdrive.turnpilot.turn_speed import NO_SPEED_CAP

MAP_STOP_DECEL = 1.5          # m/s^2, the braking curve to a map stop point
MAP_DECEL_MAX = SPEED_CAP_ACCEL_MIN  # -2.0 m/s^2: a map stop never brakes harder
YELLOW_DECEL = 2.0            # m/s^2 for a stop decided on a yellow (TurnPilotYellowDecel)
YELLOW_DECEL_MAX = -MAP_DECEL_MAX  # 2.0: a light stop never brakes harder than any map stop (Kellen, 2026-10-28)
CONTROL_LINE_M = 10.          # without a mapped line, the stop line is the junction less this (routing/controls.py)
LATCH_SPEED = 0.3             # m/s: slower than this...
LATCH_NEAR_M = 5.             # ...this close to the stop point...
LATCH_PAST_M = 2.             # ...or this far past it, the latch holds the car
HOLD_ACCEL = -0.5             # m/s^2, the held car's accel cap
STOP_SIGN_HANDBACK_S = 1.0    # s stopped at a stop sign before it hands back (DESIGN phase 4: a 1 s full stop)
GREEN_STEADY_S = 0.5          # s of green before it counts (the sim oracle; a camera would need 1.0)
T_LAT = 0.8                   # s from a yellow to braking
COMMIT_SPEED = 2.0            # m/s: past the line faster than this, the car went
YIELD_SPEED = 5.0             # m/s at a yield's line
RELEASE_SPAN_M = 30.          # releasing a control releases those this far past it
OVERRUN_PAST_M = 2.           # m past the line, moving faster than COMMIT_SPEED with no release: an overrun
DRIVE_OFF_M = 2.              # m a held car moved under the driver's gas: driven off
LIGHT_ASSOC_M = 25.           # a light belongs to the signal control whose stop point is this close to its own
LIGHT_STALE_S = 0.6           # s: an older light message is stale (unknown)
CAMERA_STALE_S = 1.0          # s: a camera light read older than this is none (it reads on road camera runs only, ~5 Hz)
CAMERA_ASSOC_BEFORE_M = 10.   # a camera head from this far short of a signal's line...
CAMERA_ASSOC_PAST_M = 40.     # ...to this far past it is that signal's (far-side heads hang ~18-35 m past the line)
CAMERA_OFF_REASON = "Camera light reader off for this drive"
SIGNAL_GO, SIGNAL_STOP, SIGNAL_COMMITTED = 'go', 'stop', 'committed'  # a signal's ControlState.mode
OFF_ROUTE_SPEED = 5.          # m/s: off the route with the stops ahead unknown, the speed is capped to this...
OFF_ROUTE_DECEL = 1.0         # m/s^2: ...coming down from the car's speed this gently
OFF_ROUTE_REASON = "Off route, map stops unknown — take control or wait for reroute"

UNPROTECTED = 'unprotectedTurn'  # MapControlStatus.Kind's name for it
UNPROTECTED_ROUTE = -1        # its key is (UNPROTECTED_ROUTE, maneuver id): a maneuver's, not a route's control
KEEP_TURN = 'keep'            # update()'s turn when the route isn't known this frame (a fault): the turn's stop carries on
MAP_MARGIN_M = 1.             # m short of the hold line an unprotected turn stops, as navd's controls stop short of their line
PROTECT_BEFORE_M = 30.        # a stop sign or signal this far before the junction (its line, mapped or the junction less 11)...
PROTECT_PAST_M = 2.           # ...to this far past it is the junction's own: the turn isn't unprotected
# A signal at the junction doesn't protect a left (DESIGN phase 4, decision 4): signals don't hold now, and nothing here can
# tell a green arrow from a green ball, so a left at a signal is unprotected. True: a signal protects it, as a stop sign does
SIGNAL_PROTECTS_LEFT = False

STOP_SIGN_REASON = "Stop sign: take control to proceed"
# The autonomous unprotected left (unprotected_left.py, DESIGN phase 4 G4) creeps its held stop point on for a view
CREEP_SPEED = 0.7             # m/s
CREEP_ACCEL = 0.5             # m/s^2
CREEP_AT_M = 0.1              # crept to within this of its target: held there again
# A mapped stop the camera never saw (row C11; STOP_SIGNS.md v3 SG4b): stopped for anyway, logged at its hold with the
# class-11 shadow evidence of its approach (TurnPilotCameraStopSigns shadow: nothing acts on a sign this week)
SIGN_LOOK_M = 40.             # m before its stop point its forward runs are counted
SIGN_SEEN_HITS = 2            # stop-sign boxes in them that say the camera saw a sign
SIGN_UNSEEN_RUNS = 10         # forward runs with none, and the stop is logged as unseen (the replay gate's)


@dataclass
class StopCommand:
  """What the stop-sign decider (stop_go.py, through turnpilotd) asks of a held stop sign's latch (module docstring). latch:
  shadow (the 1 s stopSignTakeControl handback, as before), wait (held, no handback), fault (handed back now); reason: what to
  say (shadow's default is STOP_SIGN_REASON); action: the latch's MapControlStatus.Action while it waits (waitingCross,
  creeping)."""
  latch: str = 'shadow'
  reason: str = ''
  action: str = ''


@dataclass
class LeftCommand:
  """What the unprotected left (unprotected_left.py, through turnpilotd) asks of its hold line's latch. mode:
  TurnPilotAutoLeft's (off: handed back at once, as G1; announce: handed back, the reason its verdict; auto: held for its
  go, no handback); go: release it now (cause gap); creep_m: its stop point this far past where it latched;
  take_control: a fault (undecided), handed back held (leftUndecidedTakeControl); reason: what to say."""
  mode: str = 'off'
  go: bool = False
  creep_m: float = 0.
  take_control: bool = False
  reason: str = ''


def engage_distance(v_ego: float) -> float:
  """m from the stop point a control engages: braking at MAP_STOP_DECEL, a second's lag, 15 m to spare."""
  return v_ego ** 2 / (2. * MAP_STOP_DECEL) + v_ego * STOP_LAG_S + 15.


def curve(v_ego: float, d: float) -> float:
  return math.sqrt(2. * MAP_STOP_DECEL * max(d - v_ego * STOP_LAG_S, 0.))


def needed(v_ego: float, d: float) -> float:
  return -v_ego ** 2 / (2. * max(d, HARD_STOP_MIN_M))


def turn_stop_feasible(v_ego: float, distance: float) -> bool:
  """An unprotected turn distance m ahead can be stopped for at its hold line at MAP_DECEL_MAX or gentler."""
  return needed(max(v_ego, 0.), distance - HOLD_LINE_M - MAP_MARGIN_M) >= MAP_DECEL_MAX - 1e-9


def unprotected_turn(maneuver, right_hand_traffic: bool = True) -> bool:
  """A routed turn across the oncoming traffic of the road straight ahead: a junction turn to the far side (a left on the
  right) whose junction has an oncoming approach, or might have (opposing unknown: no junction data is never clear). A
  keep, a bend and a T's stem (opposing none) aren't. The junction's own stop sign or signal is MapControlPolicy's to
  check (protected())."""
  return maneuver is not None and maneuver.kind == TURN and (maneuver.side == LEFT) == right_hand_traffic and \
    maneuver.opposing != 'none'


def protected(controls, distance: float) -> bool:
  """A stop sign of the junction distance m ahead stops the car there already (a signal too, with SIGNAL_PROTECTS_LEFT)."""
  kinds = ('stop', 'signal') if SIGNAL_PROTECTS_LEFT else ('stop',)
  return any(c.kind in kinds and distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M for c in controls)


@dataclass
class CameraLights:
  heads: list                  # [(voted state, m to the head)] of the governing heads (light_state.py)
  shadow: bool = True          # logged, never acted on; False only with both keys (camera_light_input)
  status: str = 'read'         # turnpilotDetections.lightStatus


@dataclass
class Lights:
  lights: list | None = None   # [(state, distance)] from an accepted, fresh source; None without one
  source: str = 'none'         # TurnpilotLights.Source of the last message
  rejected: bool = False       # a source that isn't accepted here sent lights (lightSourceRejected)
  camera: CameraLights | None = None  # the camera's reader, fresh (camera_light_input); never in lights


NO_LIGHTS = Lights()


def camera_light_input(msg, status: str, age_s: float, mode: str) -> CameraLights | None:
  """The camera's lights from the last road camera run's turnpilotDetections.lights and lightStatus, age_s old; mode:
  turnpilotd's TurnPilotCameraLights. None when off, stale, or not read (no calibration yet). A reader off for the drive
  (disabled) comes through, with no heads, to be said."""
  if mode == 'off' or msg is None:
    return None
  if status == 'disabled':
    return CameraLights([], True, status)
  if status != 'read' or age_s > CAMERA_STALE_S or str(msg.source) != 'camera':
    return None
  heads = [(str(light.state), float(light.distance)) for light in msg.lights if light.governs]
  return CameraLights(heads, bool(msg.shadow) or mode != 'addOnly', status)


def camera_owner(dist: float, signals):
  """The signal control a camera head dist m ahead belongs to: the last line before it within the association window."""
  owners = [o for o in signals if o.d - CAMERA_ASSOC_BEFORE_M <= dist <= o.d + CAMERA_ASSOC_PAST_M]
  return max(owners, key=lambda o: o.d, default=None)


def camera_state(c, signals, camera: CameraLights | None) -> str:
  """The camera's state for signal c: its heads' states, if they agree."""
  if camera is None:
    return 'unknown'
  states = {st for st, dist in camera.heads if st != 'unknown' and camera_owner(dist, signals) is c}
  return states.pop() if len(states) == 1 else 'unknown'


def light_input(msg, age_s: float, simulation: bool, oracle_param: bool) -> Lights:
  """Lights from a turnpilotLights message age_s old. The sim oracle only with SIMULATION=1 and TurnPilotSimLightOracle
  set: anything else that sends lights is rejected, out loud, and its lights are unknown. Stale is unknown."""
  if msg is None or str(msg.source) == 'none':
    return Lights()
  source = str(msg.source)
  if not (source == 'simOracle' and simulation and oracle_param):
    return Lights(None, source, rejected=True)
  if age_s > LIGHT_STALE_S:
    return Lights(None, source)
  return Lights([(str(light.state), float(light.distance)) for light in msg.lights], source)


@dataclass
class Shown:
  id: int
  kind: str
  distance: float
  action: str
  light_state: str = 'unknown'
  light_source: str = 'none'
  controls_status: str = 'none'
  line_mapped: bool = False


@dataclass
class MapDecision:
  stops: list = field(default_factory=list)
  prompt: str = 'none'
  alert: str = 'none'
  shown: Shown | None = None
  reason: str = ''               # the controls' status to show when nothing holds
  released_stopped: bool = False  # a hold let go this frame with the car stopped: turnpilotd re-pulses the turn


@dataclass
class ControlState:
  cap: float = NO_SPEED_CAP
  infeasible: bool = False
  override: bool = False        # the driver's gas on the approach
  mode: str = 'stop'            # signals: SIGNAL_GO (the default: nothing to stop for), SIGNAL_STOP (a red, or a yellow
                                # decided so), SIGNAL_COMMITTED (a yellow too late to stop for)
  yellow: bool = False          # a stop decided on a yellow
  green_t: float = 0.
  light: str = 'unknown'
  dwell_t: float = 0.
  moved_m: float = 0.
  done: str = ''                # 'released', 'passed', 'overrun', 'dropped'
  action: str = 'none'
  sign_runs: int = 0            # a stop sign's approach, from SIGN_LOOK_M: forward runs counted (the camera's signs read)...
  sign_hits: int = 0            # ...the stop-sign boxes in them...
  sign_conf: float = 0.         # ...and the best conf (row C11's evidence)


def description(c: TrackedControl, turn_name: str = 'left') -> str:
  if c.kind == UNPROTECTED:
    return f"unprotected {turn_name}"
  if c.kind == 'stop':
    return "map stop sign (approach unverified)" if c.cross_suspect else "stop sign"
  return "signal" if c.junction else "signal (crossing)"


class MapControlPolicy:
  def __init__(self, yellow_decel: float = YELLOW_DECEL):
    self.tracker = ControlTracker()
    self.yellow_decel = yellow_decel
    self.held: tuple[int, int] | None = None  # the latched control's key
    self.prev_gas = False
    self._alert, self._alert_t = 'none', 0.
    self.turn: TrackedControl | None = None  # the unprotected turn's control (module docstring)
    self._turn_floor: float | None = None  # m, dead-reckoned: its stop point read behind where the car can stop at the floor
    self._turn_raw = 0.  # m to its stop point as last read, before that floor
    self.turn_name = 'left'
    self._protected_id: int | None = None  # the maneuver a stop sign or signal of its junction protected: it stays so
    self._off_cap: float | None = None  # m/s: the off-route cap while the stops ahead are unknown (module docstring)
    self._camera_would: dict = {}  # signal key -> what the camera would have it do, as last logged
    self._camera_unmapped: tuple = ()  # the unmapped camera states last logged
    self._camera_off_said = False
    self._camera_off_t = 0.  # s of the reader's off line still to say; it runs down only while shown (never masks)
    self._lights = NO_LIGHTS  # the last update's lights (turn_light)
    self._signal_turn_id: int | None = None  # the unprotected turn a signal at its junction was seen for (turn_light)

  def stop_ahead(self) -> float | None:
    """m to the nearest stop sign or signal the car is going to stop at, or None: a stop sign not released, a signal only
    while it is stopping the car (a red, or a yellow decided a stop), never one it goes through. A turn just past one is
    taken from a stop, never too fast (turn_skip.py), and the lane change window plans around it (lane_planner.py)."""
    def stopping(c):
      st = c.state
      if c.kind == 'signal':
        return st is not None and not st.done and st.mode == SIGNAL_STOP
      return c.kind == 'stop' and not (st is not None and st.done)
    ds = [0. if c.held else c.d for c in self.tracker.controls.values() if stopping(c) and (c.held or c.d > -LATCH_PAST_M)]
    return min(ds) if ds else None

  def turn_engaged(self, maneuver_id: int) -> bool:
    """The unprotected turn of this maneuver is engaged (stopping for it, held at it, or released from it)."""
    tc = self.turn
    return tc is not None and tc.key == (UNPROTECTED_ROUTE, int(maneuver_id)) and tc.engaged

  def turn_light(self, maneuver_id: int, distance: float) -> str:
    """The light of the signal at the junction of the unprotected turn maneuver_id, distance m ahead, for the autonomous
    left's go (unprotected_left.py: DESIGN G4 go condition 5): 'none' with no signal there (one from PROTECT_BEFORE_M
    before it to PROTECT_PAST_M past it, once seen the junction's for good: it drops out 15 m behind the car), 'unknown'
    with navd's controls failing or unsupported (whether there is one isn't known); else what
    every source reads around the junction (PROTECT_BEFORE_M before it to CAMERA_ASSOC_PAST_M past it), the camera's in
    shadow too: it only ever keeps the car where it stands, never brakes it. Red over yellow over green; 'unknown' with no
    read. The last update's lights."""
    if any(c.kind == 'signal' and distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M for c in self.tracker.controls.values()):
      self._signal_turn_id = maneuver_id
    if self._signal_turn_id != maneuver_id:
      # navd's controls failing or unsupported: a signal there isn't known not to be (no junction data is never clear)
      return 'unknown' if self.tracker.route and self.tracker.status not in ('ok', 'none') else 'none'
    lo, hi = distance - PROTECT_BEFORE_M, distance + CAMERA_ASSOC_PAST_M
    lights = self._lights
    states = {st for st, d in (lights.lights or []) if lo <= d <= hi}
    if lights.camera is not None:
      states |= {st for st, d in lights.camera.heads if lo <= d <= hi}
    return next((st for st in ('red', 'yellow', 'green') if st in states), 'unknown')

  def release_light(self, distance: float) -> str:
    """The light of the route's junction distance m ahead for a go TurnPilot's own hold let go of (turnpilotd: turnRelease,
    skeptic G5): 'none' with no signal there (PROTECT_BEFORE_M before it to PROTECT_PAST_M past it) and navd's controls ok,
    'unknown' with them anything else (whether there is one isn't known). At a signal, around it as turn_light reads: a red
    or a yellow from any source, the camera's in shadow too (here it only ever keeps a go off); a green only from an accepted
    source, never the camera's (G2 is shadow: a phantom green would lift the car into a red); else 'unknown'. Latches
    nothing (turn_light's latch is the unprotected left's). The last update's lights."""
    if not any(c.kind == 'signal' and distance - PROTECT_BEFORE_M <= c.d <= distance + PROTECT_PAST_M
               for c in self.tracker.controls.values()):
      return 'none' if self.tracker.status == 'ok' else 'unknown'
    lo, hi = distance - PROTECT_BEFORE_M, distance + CAMERA_ASSOC_PAST_M
    lights = self._lights
    accepted = {st for st, d in (lights.lights or []) if lo <= d <= hi}
    camera = {st for st, d in lights.camera.heads if lo <= d <= hi} if lights.camera is not None else set()
    return next((st for st in ('red', 'yellow') if st in accepted | camera), 'green' if 'green' in accepted else 'unknown')

  def held_control(self) -> TrackedControl | None:
    """The control the latch holds the car at, or None: the stop-sign decider's (turnpilotd), keyed on its state object, which
    a reroute's adopt carries to the new route's same control."""
    if self.held is None:
      return None
    if self.turn is not None and self.turn.key == self.held:
      return self.turn
    return self.tracker.controls.get(self.held)

  def held_stops(self) -> list:
    """The latch alone, for a faulted plan whose map update failed: a held car stays held."""
    return [Stop(HOLD_ACCEL, 0., True, "Stopped at a map stop", 0.)] if self.held is not None else []

  def _transient(self, alert: str) -> None:
    self._alert, self._alert_t = alert, ABORT_HOLD_S

  def _light(self, c: TrackedControl, controls, lights: Lights) -> str:
    """The light state for signal control c: the lights within LIGHT_ASSOC_M of it that have no nearer signal control,
    if they agree."""
    if lights.lights is None:
      return 'unknown'
    signals = [o for o in controls if o.kind == 'signal' and not (o.state is not None and o.state.done)]
    states = {s for s, dist in lights.lights
              if abs(dist - c.d) < LIGHT_ASSOC_M and min(signals, key=lambda o: abs(dist - o.d), default=c) is c}
    return states.pop() if len(states) == 1 else 'unknown'

  def _turn_control(self, turn, controls, v: float, dt: float) -> TrackedControl | None:
    """The unprotected turn's control this frame. turn: (maneuver id, m to its junction, 'left' or 'right') for an
    unprotected_turn() the car is taking (not skipped), None, or KEEP_TURN: not known this frame (turnpilotd faulted
    before it knew the route), the one there carries on, dead-reckoned."""
    tc = self.turn
    if tc is not None and tc.held:
      if isinstance(turn, tuple):
        self._turn_raw = turn[1] - HOLD_LINE_M - MAP_MARGIN_M  # kept current: a release must not read the car's moves as a jump
      return tc  # position-free: only its release lets go
    if isinstance(turn, str):
      if tc is not None:
        tc.d -= v * dt
      return tc
    if turn is not None and protected(controls, turn[1]):
      # once protected, protected for good: a signal mapped well before a wide junction drops out of the controls 15 m
      # behind the car (control_tracker.DROP_PAST_M), still short of the hold line: never a second stop for the same go
      self._protected_id = int(turn[0])
    want = turn is not None and int(turn[0]) != self._protected_id
    key = (UNPROTECTED_ROUTE, int(turn[0])) if turn is not None else None
    if tc is not None and (not want or tc.key != key):
      if tc.engaged and not (tc.state is not None and tc.state.done):
        cloudlog.event("turnpilot.unprotected_turn_dropped", key=list(tc.key), d=round(tc.d, 1), v=round(v, 2),
                       why='protected' if turn is not None and key == tc.key else 'maneuver')
      tc = None
    if tc is None:
      self._turn_floor = None
    if want:
      d = turn[1] - HOLD_LINE_M - MAP_MARGIN_M
      raw, was_raw = d, self._turn_raw - v * dt
      self._turn_raw = raw
      if tc is None:
        tc = TrackedControl(key, UNPROTECTED, d)
      else:
        was = tc.d - v * dt
        if self._turn_floor is not None:
          self._turn_floor -= v * dt
        stop_d = v ** 2 / (2. * -MAP_DECEL_MAX)
        if tc.engaged and tc.state is not None and not tc.state.override and not tc.state.done and raw < was_raw - 2. and \
           raw < stop_d - LATCH_PAST_M:
          # its junction read nearer (route_follower.py: the nearest of navd's readings), its line now where the car,
          # braking at the floor, would stop past the latch's reach: never dropped as an overrun and crossed blind, the
          # car stops as soon as it can at the floor, no harder, and is held there and handed back. Said out loud.
          # Nearer than that the latch holds it as for any stop
          self._turn_floor = max(self._turn_floor or 0., stop_d)
          self._transient('cannotStopComfortably')
          cloudlog.event("turnpilot.unprotected_turn_line_behind", error=True, key=list(tc.key), d=round(d, 1),
                         v=round(v, 2), stop_d=round(stop_d, 1))
        if self._turn_floor is not None:
          d = max(d, self._turn_floor)
        tc.jumped = abs(d - was) > JUMP_M
        tc.d = d
      self.turn_name = turn[2]
    self.turn = tc
    return tc

  def _release(self, c: TrackedControl, controls, cause: str, v: float, out: MapDecision) -> None:
    st = c.state
    st.done = 'released'
    if self.held == c.key:
      self.held, c.held = None, False
      out.released_stopped = out.released_stopped or v < LATCH_SPEED
    for o in controls:
      if c.kind == UNPROTECTED:
        break  # past a turn's hold line the route is on another road: nothing there is the same junction's
      if cause == 'clear':
        break  # the stop-sign decider's go certifies its own line only: a second line (a divided road) has its own (G11)
      if o is not c and c.d <= o.d <= c.d + RELEASE_SPAN_M and o.kind not in ('yield', UNPROTECTED):
        o.state = o.state or ControlState()
        o.state.done = 'released'
    cloudlog.event("turnpilot.map_stop_released", kind=c.kind, key=list(c.key), cause=cause, d=round(c.d, 1), v=round(v, 2))

  def _overrun(self, c: TrackedControl, v: float) -> None:
    c.state.done = 'overrun'
    if self.held == c.key:
      self.held, c.held = None, False
    self._transient('stopOverrun')
    cloudlog.event("turnpilot.map_stop_overrun", error=True, kind=c.kind, key=list(c.key), d=round(c.d, 1), v=round(v, 2))

  def update(self, nav, updated: bool, valid: bool, v_ego: float, gas_pressed: bool, lights: Lights = NO_LIGHTS,
             dt: float = DT_MDL, turn=None, left: LeftCommand | None = None, release: tuple | None = None,
             creep: tuple | None = None, stop: StopCommand | None = None, signs: tuple | None = None) -> MapDecision:
    """nav: the latest navInstruction, updated: it arrived this frame, valid: its valid flag; carState's vEgo and
    gasPressed; lights: light_input()'s; turn: (maneuver id, m to its junction, 'left' or 'right') of the route's
    unprotected_turn() the car is taking, or None; left: the unprotected left's command for its latch (None: off); release:
    (control key, cause) to let the latch go this frame (the stop-sign decider's 'clear'); creep: (m past where it latched,
    m/s cap) for a held stop sign's stop point (the decider's roll-up); stop: the decider's StopCommand for a held stop sign
    (None: shadow, as before); signs: (this tick's forward runs, their stop-sign boxes [(bearing, elevation, conf)]) for row
    C11's evidence, None with the camera's signs not read."""
    self.left = left if left is not None else LeftCommand()
    self.stop_cmd = stop if stop is not None else StopCommand()
    self._creep = creep
    self._lights = lights
    # one release path (module docstring): the autonomous left's go is its turn's release, cause gap
    self._release_req = release
    if self.left.mode == 'auto' and self.left.go and self.turn is not None:
      self._release_req = (self.turn.key, 'gap')
    v = max(v_ego, 0.)
    controls = self.tracker.update(nav, updated, valid, v, dt)
    if self.held is not None and all(c.key != self.held for c in controls) and \
       not (self.turn is not None and self.turn.key == self.held):
      # a reroute's same control took over the held one under its own key (control_tracker.py): the latch follows it
      moved = next((c for c in controls if c.held), None)
      cloudlog.event("turnpilot.map_stop_latch_moved", key=list(self.held), to=list(moved.key) if moved else None)
      self.held = moved.key if moved is not None else None
    tc = self._turn_control(turn, controls, v, dt)
    if tc is not None:
      controls = sorted(controls + [tc], key=lambda c: c.d)
    gas_lift = self.prev_gas and not gas_pressed
    self.prev_gas = gas_pressed
    out = MapDecision()
    prompts = []
    infeasible = False
    if any(c.engaged for c in self.tracker.lost):
      self._transient('mapControlsUnavailable')
    camera = lights.camera
    if camera is not None and camera.status == 'disabled' and not self._camera_off_said:
      self._camera_off_said = True
      self._camera_off_t = ABORT_HOLD_S  # its own hold, never _transient: it must not overwrite a live one-shot alert
      cloudlog.event("turnpilot.camera_lights_off", error=True)
    live_signals = [c for c in controls if c.kind == 'signal' and not (c.state is not None and c.state.done)]
    self._signs = signs
    if signs is not None:
      # each forward run counted for the nearest stop sign ahead within SIGN_LOOK_M not yet held (row C11)
      ahead = [c for c in controls if c.kind == 'stop' and not c.held and not (c.state is not None and c.state.done) and
               -LATCH_PAST_M < c.d <= SIGN_LOOK_M]
      if ahead:
        st = min(ahead, key=lambda c: c.d)
        st.state = st.state or ControlState()
        runs, boxes = signs
        st.state.sign_runs += int(runs)
        st.state.sign_hits += len(boxes)
        st.state.sign_conf = max([st.state.sign_conf] + [float(b[2]) for b in boxes])
    for c in controls:
      st = c.state = c.state or ControlState(mode=SIGNAL_GO if c.kind == 'signal' else 'stop')
      if st.done:
        continue
      if c.kind == 'signal':
        st.light = self._light(c, controls, lights)
        if camera is not None and not camera.shadow:
          st.light = self._camera_adds(c, st, camera_state(c, live_signals, camera), v)
        st.green_t = st.green_t + dt if st.light == 'green' else 0.
      if c.held:
        self._held(c, controls, v, gas_pressed, dt, out, prompts)
      elif c.kind == 'yield':
        self._yield(c, v, out, prompts)
      else:
        infeasible = self._approach(c, controls, v, gas_pressed, gas_lift, out, prompts) or infeasible

    unknown = self.tracker.off_route_unknown
    if unknown:
      if self._off_cap is None:
        self._off_cap = max(v, OFF_ROUTE_SPEED)
        cloudlog.event("turnpilot.off_route_stops_unknown", error=True, status=self.tracker.off_status, v=round(v, 2))
      self._off_cap = max(self._off_cap - OFF_ROUTE_DECEL * dt, OFF_ROUTE_SPEED)
      out.stops.append(Stop(NO_ACCEL_CAP, self._off_cap, False, OFF_ROUTE_REASON, 0.))
    elif self._off_cap is not None:
      self._off_cap = None
      cloudlog.event("turnpilot.off_route_stops_known", route=self.tracker.route, status=self.tracker.off_status)

    # what the driver sees and hears
    out.prompt = prompts[0] if prompts else 'none'
    # 'none' is a navd that predates controls: nothing to say about a route it never looked at
    status_bad = self.tracker.route and self.tracker.status not in ('ok', 'none')
    if status_bad:
      out.reason = f"Map stops/signals: {self.tracker.status}"
    if unknown:
      out.reason = OFF_ROUTE_REASON
    self._alert_t = max(self._alert_t - dt, 0.)
    if infeasible:
      out.alert = 'cannotStopComfortably'
    elif self._alert_t > 0.:
      out.alert = self._alert
    elif unknown:
      out.alert = 'offRouteStopsUnknown'
    elif status_bad:
      out.alert = 'mapControlsUnavailable'
    elif lights.rejected:
      out.alert = 'lightSourceRejected'
    elif self._camera_off_t > 0.:
      # a shadow component's status line: said only where nothing else is, held back (not lost) behind a take-control,
      # overrun or yellow alert, and said in full once that one is done
      out.alert = 'lightSourceRejected'
      self._camera_off_t = max(self._camera_off_t - dt, 0.)
      if not out.reason:
        out.reason = CAMERA_OFF_REASON
    live = [c for c in controls if c.state is not None and not c.state.done]
    pick = next((c for c in live if c.held), None) or next((c for c in live if c.engaged), None) or \
      next((c for c in live if c.d > -OVERRUN_PAST_M), None)
    if pick is not None:
      st = pick.state
      out.shown = Shown(pick.key[1] & 0xFFFF, pick.kind, pick.d, st.action, st.light, lights.source, self.tracker.status, pick.line_mapped)
      if not out.reason and pick.kind == 'signal' and st.mode == SIGNAL_GO and st.light == 'green':
        out.reason = "Green (sim oracle)"
    elif self.tracker.route:
      out.shown = Shown(0, 'none', 0., 'none', 'unknown', lights.source, self.tracker.status)
    self._camera_log(live_signals, camera, v)
    return out

  def _camera_adds(self, c, st, cam: str, v: float) -> str:
    """Add-only: signal c's light raised by the camera's (module docstring). Never lowered."""
    if cam == 'red':
      return 'red'
    if cam == 'yellow' and st.light != 'red' and st.mode == SIGNAL_GO and \
       c.d >= v * T_LAT + v ** 2 / (2. * min(self.yellow_decel, YELLOW_DECEL_MAX)):
      return 'yellow'
    return st.light

  def _camera_log(self, signals, camera: CameraLights | None, v: float) -> None:
    """What the camera would have each signal do, logged on a change; it reads nothing back into the policy."""
    keys = set()
    for c in signals:
      keys.add(c.key)
      cam = camera_state(c, signals, camera)
      st = c.state
      if cam == 'red':
        need = needed(v, c.d)
        would = 'held' if c.held else 'stop' if need >= -MAP_STOP_DECEL else 'stopFirm' if need >= MAP_DECEL_MAX else \
          'cannotStopComfortably'
      elif cam == 'yellow':
        would = 'stop' if c.d >= v * T_LAT + v ** 2 / (2. * min(self.yellow_decel, YELLOW_DECEL_MAX)) else 'proceedOnYellow'
      else:
        would = 'none'
      if self._camera_would.get(c.key, ('unknown', 'none')) != (cam, would):
        self._camera_would[c.key] = (cam, would)
        cloudlog.event("turnpilot.camera_light_shadow", key=list(c.key), d=round(c.d, 1), v=round(v, 2), camera=cam, would=would,
                       acting=camera is not None and not camera.shadow, policy_light=st.light if st else 'unknown',
                       policy_mode=st.mode if st else '', needed=round(needed(v, c.d), 2),
                       heads=[(h, round(dist, 1)) for h, dist in camera.heads] if camera is not None else [])
    for k in [k for k in self._camera_would if k not in keys]:
      del self._camera_would[k]
    unmapped = tuple(sorted({h for h, dist in camera.heads if h in ('red', 'yellow') and camera_owner(dist, signals) is None})) \
      if camera is not None else ()
    if unmapped != self._camera_unmapped:
      self._camera_unmapped = unmapped
      if unmapped:
        cloudlog.event("turnpilot.camera_light_unmapped", states=list(unmapped), v=round(v, 2),
                       heads=[(h, round(dist, 1)) for h, dist in camera.heads])

  def _held(self, c, controls, v, gas_pressed, dt, out, prompts) -> None:
    st = c.state
    st.dwell_t += dt
    st.moved_m += v * dt
    what = description(c, self.turn_name)
    cause, prompt = None, None
    left = getattr(self, 'left', LeftCommand())
    req = getattr(self, '_release_req', None)
    stop_cmd = getattr(self, 'stop_cmd', StopCommand())
    if c.kind == UNPROTECTED and left.mode == 'auto':
      # the autonomous left's (unprotected_left.py): held for its go, never handed back but for its fault
      if left.take_control:
        prompt, reason = 'leftUndecidedTakeControl', left.reason or f"Unprotected {self.turn_name}: take control"
      elif req is not None and req[0] == c.key:
        cause = req[1]
        reason = left.reason
      else:
        reason = left.reason or f"Unprotected {self.turn_name}: waiting for a gap"
    elif c.kind == UNPROTECTED:
      # off (G1): handed back at once, held while the driver takes over; announce: the same, saying whether it would go
      prompt = 'unprotectedTurnTakeControl'
      reason = f"{left.reason}: take control" if left.mode == 'announce' and left.reason else \
        f"Unprotected {self.turn_name}: take control"
    elif c.kind == 'stop' and req is not None and req[0] == c.key and stop_cmd.latch == 'wait':
      cause, reason = req[1], stop_cmd.reason or "Going"  # the stop-sign decider's go (cause clear)
    elif c.kind == 'stop' and stop_cmd.latch == 'wait':
      reason = stop_cmd.reason or f"Stopped at {what}"  # the stop-sign decider waits: held, no handback
    elif c.kind == 'stop' and stop_cmd.latch == 'fault':
      prompt, reason = 'stopSignTakeControl', stop_cmd.reason or STOP_SIGN_REASON  # its fault: handed back now
    elif c.kind == 'stop':
      # as shadow: a full stop, then handed back (module docstring), the decider's reason saying why
      prompt = 'stopSignTakeControl' if st.dwell_t >= STOP_SIGN_HANDBACK_S - 1e-9 else None
      reason = (stop_cmd.reason or STOP_SIGN_REASON) if prompt is not None else f"Stopped at {what}"
    else:
      if st.green_t >= GREEN_STEADY_S - 1e-9:
        cause = 'green'
      reason = f"Red light (sim oracle){'' if c.junction else ' (crossing)'}" if st.light == 'red' else \
        f"Stopped at {what}: waiting for green"
    # moved where it was asked (a creep, a roll-up) isn't driven off: only past that
    asked = left.creep_m if c.kind == UNPROTECTED and left.mode == 'auto' else \
      (getattr(self, '_creep', None) or (0., 0.))[0] if c.kind == 'stop' and stop_cmd.latch == 'wait' else 0.
    if cause is None and st.moved_m - max(asked, 0.) > DRIVE_OFF_M and v > COMMIT_SPEED:
      # handed back, the driver took over and went: theirs, said already. Driven off under the driver's foot: lifting must
      # not stop the car in the junction. Or gone with no go at all, the plan not applied (shadow mode turned on): the
      # latch must not stop the car wherever the plan applies again
      if prompt is not None:
        cause = 'driverWent'
      else:
        if not gas_pressed:
          self._transient('stopOverrun')
        cause = 'drivenOff' if gas_pressed else 'movedOff'
    if cause == 'green':  # a red's stop, or one decided on a yellow, is undone by the next steady green: back to going on it
      self._signal_go(c)
      st.action = 'released'
      self.held, c.held = None, False
      out.released_stopped = out.released_stopped or v < LATCH_SPEED
      cloudlog.event("turnpilot.map_stop_released", kind=c.kind, key=list(c.key), cause=cause, v=round(v, 2))
      return
    if cause is not None:
      st.action = 'released'
      self._release(c, controls, cause, v, out)
      return
    if prompt is not None:
      prompts.insert(0, prompt)  # the held control's prompt first
    st.action = 'awaitingDriver' if prompt is not None else (stop_cmd.action or 'held') if c.kind == 'stop' and \
      stop_cmd.latch == 'wait' else 'held'
    # the latch's stop point moved on (module docstring): the left's creep for a view, the stop-sign decider's roll-up
    creep_m, creep_v, accel = 0., CREEP_SPEED, CREEP_ACCEL
    if c.kind == UNPROTECTED and left.mode == 'auto' and prompt is None:
      creep_m = left.creep_m
    elif c.kind == 'stop' and stop_cmd.latch == 'wait' and getattr(self, '_creep', None) is not None:
      creep_m, creep_v = self._creep
      accel = CREEP_ACCEL if creep_v <= CREEP_SPEED else NO_ACCEL_CAP  # a roll-up: the map curve alone, no accel cap
    creep = creep_m - st.moved_m
    if creep > CREEP_AT_M:
      # on slowly to where it is asked: still the latch's (position-free), never past its target; a roll-up on the map curve
      ramp = curve(v, creep) if creep_v > CREEP_SPEED else math.sqrt(2. * MAP_STOP_DECEL * max(creep - CREEP_AT_M, 0.))
      out.stops.append(Stop(accel, min(creep_v, ramp), False, reason, creep))
      return
    out.stops.append(Stop(HOLD_ACCEL, 0., True, reason, 0.))

  def _yield(self, c, v, out, prompts) -> None:
    st = c.state
    if c.cross_suspect:
      st.done = 'dropped'
      cloudlog.event("turnpilot.map_yield_dropped", kind=c.kind, key=list(c.key), reason="crossSuspect", d=round(c.d, 1))
      return
    if not c.engaged:
      if c.d > engage_distance(v):
        return
      c.engaged = True
    if c.d < 0.:
      st.done, st.action = 'passed', 'released'
      return
    # a speed at a fixed point, not a stop: the curve down to YIELD_SPEED, no accel cap, never shouldStop
    st.cap = min(st.cap, math.sqrt(YIELD_SPEED ** 2 + 2. * MAP_STOP_DECEL * max(c.d - v * STOP_LAG_S, 0.)))
    st.action = 'braking' if st.cap < v + 0.5 else 'approaching'
    out.stops.append(Stop(NO_ACCEL_CAP, st.cap, False, "Yield ahead", c.d))
    prompts.append('yieldAhead')

  def _approach(self, c, controls, v, gas_pressed, gas_lift, out, prompts) -> bool:
    """A stop sign or signal not held: the curve, the decisions, the latch. True while it is infeasible."""
    st = c.state
    if c.kind == 'signal':
      self._signal_mode(c, v)
      if st.mode != SIGNAL_STOP:  # nothing to stop for: no stop, no cap, no prompt (module docstring)
        st.action = 'proceedingOnYellow' if st.mode == SIGNAL_COMMITTED else 'none'
        if c.d < -OVERRUN_PAST_M:
          st.done = 'passed'
        return False
    if not c.engaged:
      if c.d > engage_distance(v) or c.d < -OVERRUN_PAST_M:
        return False
      c.engaged = True
      st.cap = curve(v, c.d)
      self._check(c, v, 'engaged')
    elif c.jumped:
      st.cap = curve(v, c.d)
      self._check(c, v, 'jumped')

    if c.kind == 'signal':
      lit = {'red': "Red light (sim oracle)", 'yellow': "Yellow light (sim oracle)"}.get(st.light, "Stopping for the signal")
      reason = lit + ('' if c.junction else ' (crossing)')
    else:
      if c.kind == UNPROTECTED:
        reason = f"Unprotected {self.turn_name} ahead"
      else:
        reason = "Map stop sign, approach unverified" if c.cross_suspect else "Stop sign ahead"
      if gas_pressed:
        st.override = True
      elif st.override and gas_lift:
        st.override = False
        if c.d < -LATCH_PAST_M or (c.d < 0. and v > COMMIT_SPEED):
          # past the line under the driver's foot, beyond where the latch can hold it: the driver went. Stopping the car
          # in the junction with nothing to release it would be worse
          st.action = 'overrun'
          self._overrun(c, v)
          return False
        st.cap = curve(v, c.d)
        self._check(c, v, 'gasLifted')

    if c.d < -OVERRUN_PAST_M and not gas_pressed:
      # past the line and not held, however slow: never a stop in the junction with no hold and no go to release it
      st.action = 'overrun'
      self._overrun(c, v)
      return False

    cv = curve(v, c.d)
    st.cap = cv if v < LATCH_SPEED else min(st.cap, cv)  # re-seeded only at standstill (a queue)
    need = needed(v, c.d)
    floor = MAP_DECEL_MAX  # a yellow's stop too (YELLOW_DECEL_MAX)
    accel = max(need, floor) if need < -MAP_STOP_DECEL else NO_ACCEL_CAP
    if st.infeasible and need >= floor:
      st.infeasible = False
    if v < LATCH_SPEED and -LATCH_PAST_M < c.d < LATCH_NEAR_M and not gas_pressed and self.held is None:
      self.held, c.held = c.key, True
      st.dwell_t, st.moved_m, st.infeasible = 0., 0., False
      cloudlog.event("turnpilot.map_stop_held", kind=c.kind, key=list(c.key), d=round(c.d, 1), light=st.light)
      if c.kind == 'stop' and getattr(self, '_signs', None) is not None:
        self._sign_evidence(c)
      self._held(c, controls, v, gas_pressed, 0., out, prompts)
      return False
    st.action = 'infeasible' if st.infeasible else 'braking' if (st.cap < v + 0.5 or accel < NO_ACCEL_CAP) else 'approaching'
    out.stops.append(Stop(accel, st.cap, False, reason, c.d))
    return st.infeasible

  def _sign_evidence(self, c) -> None:
    """Row C11 at a stop sign's hold: its approach's class-11 shadow evidence logged (the forward runs counted from
    SIGN_LOOK_M, the stop-sign boxes in them, the best conf), and with SIGN_UNSEEN_RUNS or more and no box, said as a mapped
    stop the camera didn't see (a sign missing, turned or hidden, or the map's own error) for the replay gate. The stop
    stands either way (the gentle error) and the decider certifies it as mapped: nothing here acts on a sign."""
    st = c.state
    seen = st.sign_hits >= SIGN_SEEN_HITS
    cloudlog.event("turnpilot.map_stop_signs", key=list(c.key), runs=st.sign_runs, hits=st.sign_hits, conf=round(st.sign_conf, 2),
                   seen=seen, lineMapped=bool(c.line_mapped), crossSuspect=bool(c.cross_suspect))
    if not seen and st.sign_runs >= SIGN_UNSEEN_RUNS:
      cloudlog.event("turnpilot.map_stop_unseen", key=list(c.key), runs=st.sign_runs, hits=st.sign_hits, conf=round(st.sign_conf, 2),
                     junction=bool(c.junction), osmNodeId=int((getattr(c, 'junction_info', None) or {}).get('osmNodeId', 0) or 0))

  def _signal_go(self, c) -> None:
    """A signal back to nothing to stop for (a steady green): its curve gone, re-seeded if it stops the car again."""
    st = c.state
    st.mode, st.yellow, st.infeasible, st.cap = SIGNAL_GO, False, False, NO_SPEED_CAP
    c.engaged = False

  def _signal_mode(self, c, v: float) -> None:
    """A signal's light into its mode (module docstring): a steady green undoes a stop, a red going is a stop, a yellow
    going is decided once. A stop is engaged where it is first read (or at engage_distance), its curve seeded there."""
    st = c.state
    if st.mode == SIGNAL_STOP and st.green_t >= GREEN_STEADY_S - 1e-9:
      self._signal_go(c)
      cloudlog.event("turnpilot.map_stop_released", kind=c.kind, key=list(c.key), cause='green', d=round(c.d, 1), v=round(v, 2))
    elif st.mode == SIGNAL_GO and st.light == 'red':
      st.mode, st.yellow = SIGNAL_STOP, False
      cloudlog.event("turnpilot.map_signal_decided", decision='stop', light=st.light, d=round(c.d, 1), v=round(v, 2))
    elif st.mode == SIGNAL_GO and st.light == 'yellow':
      # decided once: a later red never reverses a commit, and only a steady green undoes a stop
      if c.d >= v * T_LAT + v ** 2 / (2. * min(self.yellow_decel, YELLOW_DECEL_MAX)):
        st.mode, st.yellow = SIGNAL_STOP, True
        cloudlog.event("turnpilot.map_signal_decided", decision='stop', light=st.light, d=round(c.d, 1), v=round(v, 2))
      else:
        st.mode = SIGNAL_COMMITTED
        self._transient('proceedingOnYellow')
        cloudlog.event("turnpilot.map_signal_decided", decision='go', light=st.light, d=round(c.d, 1), v=round(v, 2))

  def _check(self, c, v, when: str) -> None:
    """Can the car stop at -MAP_DECEL_MAX from here? If not it brakes at the floor, alerted (cannotStopComfortably)."""
    floor = MAP_DECEL_MAX
    c.state.infeasible = needed(v, c.d) < floor - 1e-9
    if c.state.infeasible:
      cloudlog.event("turnpilot.map_stop_infeasible", error=True, kind=c.kind, key=list(c.key), when=when, d=round(c.d, 1),
                     v=round(v, 2), needed=round(needed(v, c.d), 2))
