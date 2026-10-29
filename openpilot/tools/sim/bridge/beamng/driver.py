"""The driver's part in a nav lane change on a Tesla. openpilot can't signal on that port, so turnpilotd prompts
"signal left" (turnpilotPlan.signalPrompt) and the driver works the stalk; DesireHelper then makes the change. The
stalk stays on until the change is made, then cancels as the wheel straightens, or goes off if the prompt does.
The prompt comes with the warning (turnpilotPlan.laneChangeCountdown); the driver waits for the countdown to run out
before working the stalk, as a driver who lets it run would, so the countdown runs in the sim. A blinker during the
countdown is the go-ahead early (lane_planner.py), which would skip it.

A turn while the car's own signal isn't ready prompts "Signal Left/Right" (turnpilotPlan.turnSignal.prompt, DESIGN
phase4/blinker 6.5): the driver works that side's stalk and holds it until the turn is over (turnSignal.side moves on), as
the wheel's return cancels it, though the prompt goes as soon as the lamp is lit."""
import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.selfdrive.turnpilot.hooks import PLAN_MAX_AGE_S

LaneChange = custom.TurnpilotPlan.LaneChange
Handback, State, Fault = custom.TurnpilotPlan.Handback, custom.TurnpilotPlan.State, custom.TurnpilotPlan.Fault
STARTING = log.LaneChangeState.laneChangeStarting


class PromptedDriver:
  def __init__(self):
    self.sm = messaging.SubMaster(['turnpilotPlan', 'modelV2'])
    self.side = LaneChange.none
    self.started = False
    self.signals = 0  # prompts answered, for the logs
    self.turn_side = LaneChange.none
    self.turn_signals = 0  # turn prompts answered

  def update(self) -> tuple[bool, bool]:
    """(left, right) blinker the driver holds now."""
    self.sm.update(0)
    plan = self.sm['turnpilotPlan']
    prompt = plan.signalPrompt if self.sm.alive['turnpilotPlan'] else LaneChange.none
    lc_state = self.sm['modelV2'].meta.laneChangeState
    if self.side == LaneChange.none:
      if prompt != LaneChange.none and plan.laneChangeCountdown <= 0.:
        self.side, self.started = prompt, False
        self.signals += 1
    elif lc_state == STARTING:
      self.started = True
    elif self.started or prompt != self.side:
      self.side = LaneChange.none

    alive = self.sm.alive['turnpilotPlan']
    turn = plan.turnSignal
    if not alive or turn.side != self.turn_side:
      self.turn_side = LaneChange.none  # the turn is over (or skipped, or another): the stalk goes back
    if alive and self.turn_side == LaneChange.none and turn.prompt != LaneChange.none and turn.prompt == turn.side:
      self.turn_side = turn.prompt
      self.turn_signals += 1
    side = self.side if self.side != LaneChange.none else self.turn_side
    return side == LaneChange.left, side == LaneChange.right


TAP_GAS, TAP_S = 0.25, 1.0  # the driver's go: this much gas for this long, an override (DisengageOnAccelerator off)
STOPPED_TAP_S = 2.0         # TurnPilot asks for a go (controlPrompt): the driver taps once the car has stood this long
STALL_S = 4.0               # standing engaged this long with nothing held (the model stopped on its own): a tap to go on
TAP_GAP_S = 2.0             # s after a tap ends before another
STANDING = 0.3              # m/s


class GasTaps:
  """The sim driver's gas taps, where a driver would give one, each logged with its reason and how long the car had
  stood ('stopped_s'):
  - stopSignGo: TurnPilot holds the car at a stop sign past its dwell and asks for the driver's go: once the car has
    stood STOPPED_TAP_S (counted from the stop, not the prompt: a dwell longer than that gets its tap with the prompt);
  - unprotectedTurnGo: held at an unprotected turn's hold line for the driver's go: once the car has stood
    STOPPED_TAP_S, as a driver who has looked across (this driver doesn't look: the detector's oncoming hold after the
    go is what it exercises);
  - signalGo: a signal whose light TurnPilot hasn't checked: only when the ground-truth light (the bridge's own reading
    of the game) is green, once the car has stood STOPPED_TAP_S, never on red, yellow or a light it can't read;
    signal_tap False never taps it (the stop-at-the-line runs);
  - stall, only with stall_tap (exploring by hand, never a scored run): standing engaged STALL_S with nothing held and no
    prompt (plan not holding, no shouldStop): the model stopped on its own. TurnPilot's own carStoppedGo prompt for that
    (turnpilotd.py) is never tapped otherwise: a scored run's car left standing stays a failure to see. Off, a car left standing (no pull-away after a
    hold let go, as BeamNG ped_crosswalk_right did) is the run's failure to see, not the driver's to push on, and a push
    in a queue would drive the car at the lead (scenarios/score.py fails a press TurnPilot asked no go for).
  Never while TurnPilot holds the car for anything else: a pedestrian, the stop sign's dwell. And never while TurnPilot
  isn't acting, whatever its prompt says: a handback (plan.handback), unavailable (the detector not started, or handed
  back), a fault, or no fresh plan (older than hooks.PLAN_MAX_AGE_S, as selfdrived takes it): a tap under way stops, and
  the car's standing counts again from when it is back."""

  def __init__(self, signal_tap: bool = True, stall_tap: bool = False):
    self.signal_tap = signal_tap
    self.stall_tap = stall_tap
    self.log: list[dict] = []
    self._since = None      # (reason, t) the current reason to tap began
    self._stopped = None    # t the car came to a stand, engaged (None: moving or not engaged)
    self._tap_until = -1.
    self._next_ok = -1.

  def update(self, t: float, plan, engaged: bool, v: float, green: bool | None = None, plan_age_s: float = 0.) -> float:
    """The gas to hold now (0..1). plan: turnpilotPlan (None if none); green: the ground-truth light ahead is green
    (None: unknown); plan_age_s: s since the plan was received."""
    if plan is None or plan_age_s > PLAN_MAX_AGE_S or plan.handback != Handback.none or plan.state == State.unavailable or \
       plan.fault != Fault.none:
      self._tap_until, self._since, self._stopped = -1., None, None
      return 0.
    if t < self._tap_until:
      return TAP_GAS
    standing = engaged and v < STANDING
    self._stopped = (t if self._stopped is None else self._stopped) if standing else None
    reason = None
    if standing and plan is not None:
      prompt = str(plan.controlPrompt)
      if prompt in ('stopSignGo', 'unprotectedTurnGo'):
        reason = prompt
      elif prompt == 'signalGo':
        reason = 'signalGo' if self.signal_tap and green is True else None
      elif self.stall_tap and str(plan.state) != 'holding' and not plan.shouldStop:
        reason = 'stall'
    if reason is None:
      self._since = None
      return 0.
    if self._since is None or self._since[0] != reason:
      self._since = (reason, t)
    waited = t - self._since[1] >= STALL_S - 1e-6 if reason == 'stall' else t - self._stopped >= STOPPED_TAP_S - 1e-6
    if waited and t >= self._next_ok:
      self._tap_until, self._next_ok, self._since = t + TAP_S, t + TAP_S + TAP_GAP_S, None
      self.log.append({'t': round(t, 2), 'reason': reason, 'stopped_s': round(t - self._stopped, 2)})
      print(f"driver: gas tap ({reason}, stood {t - self._stopped:.1f} s)", flush=True)
      return TAP_GAS
    return 0.


LIGHTS_FRESH_S = 1.  # the light oracle's reading (light_oracle.py) is the driver's ground truth this long


def ground_truth_green(cfg, sm, now: float) -> bool | None:
  """The light ahead really is green: every light is, forced and held, unless the bridge runs with --signals; then the
  light oracle's fresh reading of the nearest one ahead, if it runs. None: not known, so never tapped on."""
  if getattr(cfg, 'green', False):
    return True
  if not sm.seen.get('turnpilotLights') or now - sm.recv_time['turnpilotLights'] > LIGHTS_FRESH_S:
    return None
  lights = [lt for lt in sm['turnpilotLights'].lights if lt.distance >= -2.]
  return None if not lights else str(min(lights, key=lambda lt: lt.distance).state) == 'green'
