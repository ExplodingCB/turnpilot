"""What upstream processes take from turnpilotPlan: modeld its desire and lane change, plannerd its limits, selfdrived
its alerts and lane change events.

A plan is acted on only while it is fresh by its receive time. SubMaster.alive can't say: under SIMULATION it stays
true for good once a service was seen, so a dead turnpilotd's last speed cap or desire would hold for the rest of a
sim drive.
"""
import math
import time

from openpilot.cereal import custom, log

SPEED_CAP_TAU = 1.0           # s, from a speed cap to an accel
SPEED_CAP_ACCEL_MIN = -2.0    # m/s^2: a speed cap alone brakes no harder than this; accelCap carries hard stops
PLAN_MAX_AGE_S = 0.5          # s since the last turnpilotPlan (20 Hz): SubMaster's own 10 periods, outside simulation too
# turnpilotPlan.desire past log.Desire: the turn is done, modeld drops the turn desires from the model's desire history
CLEAR_DESIRE = 255


def fresh_plan(sm, now: float | None = None):
  """The turnpilotPlan while it is fresh and valid, whatever its state (a disabled plan may carry a handback), or None.
  sm is a SubMaster subscribed to turnpilotPlan; anything without its recv_time and valid dicts (a test's plain dict)
  gives None."""
  recv_t = getattr(sm, 'recv_time', {}).get('turnpilotPlan')
  if recv_t is None or not getattr(sm, 'valid', {}).get('turnpilotPlan', False):
    return None
  if (time.monotonic() if now is None else now) - recv_t > PLAN_MAX_AGE_S:
    return None
  return sm['turnpilotPlan']


def shown_plan(sm, now: float | None = None):
  """The fresh turnpilotPlan selfdrived alerts from, disabled ones too (a fault's plan carries its abort), or None while
  it is stale, invalid or in shadow mode."""
  plan = fresh_plan(sm, now)
  return None if plan is None or plan.shadow else plan


def active_plan(sm, now: float | None = None):
  """The turnpilotPlan to act on, or None while it is stale, invalid, disabled or in shadow mode."""
  plan = fresh_plan(sm, now)
  return None if plan is None or plan.shadow or plan.state == custom.TurnpilotPlan.State.disabled else plan


def limits_plan(sm, now: float | None = None):
  """The turnpilotPlan whose accelCap, speedCap and shouldStop plannerd acts on: active_plan's, or a faulted one, which
  carries the scene's stops in force (turnpilotd.py), or None. Every other disabled plan has no limits."""
  plan = fresh_plan(sm, now)
  if plan is None or plan.shadow:
    return None
  return plan if plan.state != custom.TurnpilotPlan.State.disabled or plan.fault != custom.TurnpilotPlan.Fault.none else None


CONTROL_ALERT_FIRST = ('hardBrake', 'cannotStopComfortably', 'offRouteStopsUnknown', 'oncomingHolding')  # above too fast to turn
# handbacks: the car held while the driver is told to take control (map_controls.py, turnpilotd.py), above too fast to turn
TAKE_CONTROL_PROMPTS = ('stopSignTakeControl', 'unprotectedTurnTakeControl', 'carStoppedTakeControl', 'stalledTakeControl',
                        'cantStartTakeControl', 'leftUndecidedTakeControl')
# shown, never heard: the unprotected left waiting at its line or creeping for a view is ordinary (no intervention)
SILENT_PROMPTS = ('none', 'yieldAhead', 'waitingForGap', 'creepingForView')
# never sent since gas go went (2026-10-28), kept in the append-only schema: the driver's gas tap as a go, and going on red
RETIRED_PROMPTS = ('stopSignGo', 'signalGo', 'unprotectedTurnGo', 'carStoppedGo')
RETIRED_ALERTS = ('goingOnRed',)
CONTROL_ALERT_EVENTS = ('stopOverrun', 'proceedingOnYellow', 'junctionUncertain')  # below the prompt, above a lane change
CONTROL_STATUS_ALERTS = ('mapControlsUnavailable', 'lightSourceRejected')  # last of all, a status line


def control_outranks_lane_change(plan) -> bool:
  """A map control's prompt or alert that outranks a lane change (plan_events' order)."""
  return str(plan.controlPrompt) != 'none' or str(plan.controlAlert) in CONTROL_ALERT_FIRST + CONTROL_ALERT_EVENTS


def plan_events(sm, now: float | None = None) -> list[str]:
  """selfdrived's onroad event names from a fresh, valid turnpilotPlan (none while it is stale: a dead turnpilotd is
  processNotRunning already, and StaleHoldEvents' while it was holding the car). sm is a SubMaster subscribed to
  turnpilotPlan.
  TurnPilot's alerts share one slot, at most one of them a frame, by this priority:
  1. a handback (turnpilotDetectorStopped, turnpilotNoDetector, turnpilotModelFellBack; handback.py): nothing here while one is set;
  2. a fault (turnpilotUnavailable);
  3. a hard brake for a confirmed pedestrian or oncoming vehicle (turnpilotControlAlert, hardBrake), then a handback
     with the car held (turnpilotControlPrompt, TAKE_CONTROL_PROMPTS: a stop sign, an unprotected turn, the car stopped
     or stalled in a turn, or not pulling away after TurnPilot's go), then a map stop it can't make gently (turnpilotControlAlert, cannotStopComfortably: take
     control), or off the route with the map stops ahead unknown (offRouteStopsUnknown: the speed capped, take control
     or wait for the reroute), or an unprotected left's go withdrawn for oncoming traffic (oncomingHolding);
  4. too fast to turn, or a turn from a standstill missed (turnpilotTooFastToTurn, turnMissed);
  5. a map control's other prompt (turnpilotControlPrompt: yield ahead, an unprotected left waiting for a gap or creeping for
     a view, visual only; its go, a short chime);
  6. a map control's other alerts (turnpilotControlAlert: a stop line overrun, going on a yellow, an unprotected turn's
     junction uncertain);
  7. a planned lane change, offered or just aborted (turnpilotLaneChangeLeft/Right/Aborted, LaneChangeEvents);
  8. the wrong lane, or a lane it can't confirm (turnpilotWrongLane);
  9. the map controls' status (turnpilotControlAlert: map stops unavailable, light source rejected).
  The alerts of 4 to 9 are all Priority.LOW or lower, where AlertManager shows the newest, so the order is kept by not
  raising the lower ones: the wrong lane isn't raised while a lane change is offered or its abort holds (the car is in a
  turn-only lane at an intersection before the turn the change is for), and lane_change_events raises nothing while the
  plan is too fast to turn, faulted or has a map control's prompt or alert up (the skip drops the maneuver, so its offer
  aborts that same frame, and the skip says why).
  turnpilotd keeps the lower ones unset while it isn't acting or is faulted (turnpilotd.py). A handback cancels an
  offered lane change with its abort (turnpilotLaneChangeAborted, reason handedBack), raised with it: on screen the
  engaged handback's soft disable (Priority.MID) outranks it, and while disengaged the abort (a WARNING) isn't shown.
  A fault is a WARNING with a chime while engaged, held TURNPILOT_FAULT_ALERT_S so a one-frame fault is heard and read
  (selfdrived/events.py), and PERMANENT at Priority.LOWER while not."""
  if not getattr(sm, 'recv_time', {}).get('turnpilotPlan'):
    return []
  plan = fresh_plan(sm, now)
  if plan is None or plan.handback != custom.TurnpilotPlan.Handback.none:
    return []
  if plan.fault != custom.TurnpilotPlan.Fault.none:
    return ['turnpilotUnavailable']
  if plan.shadow:  # in shadow mode the driver drives, and TurnPilot acts on nothing
    return []
  alert = str(plan.controlAlert)
  if alert == 'hardBrake':
    return ['turnpilotControlAlert']
  if str(plan.controlPrompt) in TAKE_CONTROL_PROMPTS:
    return ['turnpilotControlPrompt']
  if alert in CONTROL_ALERT_FIRST:
    return ['turnpilotControlAlert']
  if plan.tooFastToTurn or plan.turnMissed:
    return ['turnpilotTooFastToTurn']
  if str(plan.controlPrompt) != 'none':
    return ['turnpilotControlPrompt']
  if alert in CONTROL_ALERT_EVENTS:
    return ['turnpilotControlAlert']
  if lane_change_events(plan):
    return []  # the lane change outranks the wrong lane
  if plan.laneMismatch != custom.TurnpilotPlan.LaneMismatch.none:
    return ['turnpilotWrongLane']
  if alert in CONTROL_STATUS_ALERTS:
    return ['turnpilotControlAlert']
  return []


# turnpilotPlan.controlPrompt / controlAlert -> what the driver is told (selfdrived/events.py)
CONTROL_PROMPT_TEXT = {
  'yieldAhead': 'Yield ahead',
  'stopSignTakeControl': 'TurnPilot can\'t check cross traffic',
  'unprotectedTurnTakeControl': 'TurnPilot can\'t judge oncoming traffic',
  'carStoppedTakeControl': 'Nothing ahead is holding the car',
  'stalledTakeControl': 'The car stopped in the turn',
  'cantStartTakeControl': 'The car didn\'t pull away',
  'waitingForGap': 'Waiting for a gap in oncoming traffic',
  'creepingForView': 'Creeping forward for a view of oncoming traffic',
  'leftGo': 'Turning across oncoming traffic',
  'leftUndecidedTakeControl': 'TurnPilot can\'t judge oncoming traffic',
}
CONTROL_ALERT_TEXT = {
  'cannotStopComfortably': 'Cannot stop gently for the stop/signal',
  'stopOverrun': 'Passed the stop line without stopping',
  'proceedingOnYellow': 'Too close to stop for the yellow',
  'mapControlsUnavailable': 'Map stops and signals unavailable',
  'lightSourceRejected': 'Light source not accepted: lights unknown',
  'junctionUncertain': 'GPS unsure where the junction is: slow, may stop short of it',
  'offRouteStopsUnknown': 'Map stops unknown: take control or wait for reroute',
  'hardBrake': 'Braking hard',
  'oncomingHolding': 'Oncoming traffic: holding short of its lanes',
}


class StaleHoldEvents:
  """turnpilotd stopped while its plan held the car (shouldStop): plannerd dropped the hold with the stale plan
  (limits_plan), so the driver is told at once and for as long as it lasts (turnpilotControlAlert, which says take
  control while there is no fresh plan), not only once the manager sees the process gone."""
  def __init__(self):
    self.holding = False

  def update(self, limits, fresh) -> tuple[str, ...]:
    """limits: limits_plan's; fresh: fresh_plan's, None while stale. A fresh plan with no limits (TurnPilot turned off,
    shadow mode) isn't a lost hold: the driver turned it off."""
    if fresh is not None:
      self.holding = limits is not None and bool(limits.shouldStop)
      return ()
    return ('turnpilotControlAlert',) if self.holding else ()


ABORT_HOLD_S = 2.             # turnpilotPlan.laneChangeAbort holds its reason this long, for selfdrived's alert
# turnpilotPlan.laneChangeAbort -> what the driver is told
ABORT_TEXT = {
  'lineNotBroken': 'Lane line not broken white', 'noLane': 'No lane there', 'laneParked': 'Cars parked in that lane',
  'blindspot': 'Car in blind spot', 'carInLane': 'Car in target lane', 'noSignal': 'No signal from driver',
  'routeChanged': 'Route changed', 'notNeeded': 'Already in the lane', 'tooLate': 'Too close to the turn',
  'tooSlow': 'Too slow to change lanes', 'timedOut': 'Lane change timed out', 'turnedOff': 'Lane changes turned off',
  'badInputs': 'openpilot inputs invalid', 'stale': 'TurnPilot not responding',
  'handedBack': 'TurnPilot handed back', 'fault': 'TurnPilot unavailable',
}


def lane_change_events(plan) -> tuple[str, ...]:
  """The onroad event names (log.OnroadEvent.EventName) a turnpilotPlan raises: the coming lane change, left or right,
  from its warning until it starts, and an abort while laneChangeAbort holds its reason. Nothing while the plan is too
  fast to turn, faulted, or has a map control's prompt or alert above it: those outrank it (plan_events). plan:
  shown_plan's, or None."""
  if plan is None or plan.tooFastToTurn or plan.turnMissed or plan.fault != custom.TurnpilotPlan.Fault.none or control_outranks_lane_change(plan):
    return ()
  offer = str(plan.laneChangeOffer)
  events = ('turnpilotLaneChangeLeft',) if offer == 'left' else ('turnpilotLaneChangeRight',) if offer == 'right' else ()
  return events + (('turnpilotLaneChangeAborted',) if str(plan.laneChangeAbort) != 'none' else ())


class LaneChangeEvents:
  """selfdrived's lane change events: the plan's, and an abort when the plan goes away (turnpilotd stopped, shadow mode
  turned on) while a change was offered, so a warned-of change never just disappears."""
  def __init__(self):
    self.offered = False
    self._stale_t = 0.

  def update(self, plan, dt: float) -> tuple[str, ...]:
    """plan: shown_plan's, None when stale or in shadow mode."""
    if plan is None:
      if self.offered:
        self.offered, self._stale_t = False, ABORT_HOLD_S
      if self._stale_t > 0.:
        self._stale_t -= dt
        return ('turnpilotLaneChangeAborted',)
      return ()
    self._stale_t = 0.
    self.offered = str(plan.laneChangeOffer) != 'none'
    return lane_change_events(plan)


def hides_stock_lane_change(names, direction) -> bool:
  """Whether TurnPilot's lane change events (LaneChangeEvents.update's names) stand in for stock's preLaneChange alert
  for DesireHelper's direction (modelV2.meta.laneChangeDirection): only when the change offered is to that side. The
  driver signalling the other way is their own change, and gets stock's prompt."""
  side = {'turnpilotLaneChangeLeft': log.LaneChangeDirection.left, 'turnpilotLaneChangeRight': log.LaneChangeDirection.right}
  return any(name in side and direction == side[name] for name in names)


def lane_change_text(plan, side: str) -> str:
  """The coming lane change's alert line: the countdown, then the prompt to signal, then waiting for the blind spot."""
  if plan is None:
    return f'Lane Change {side.capitalize()}'
  if plan.laneChangeCountdown > 0.:
    return f'Lane Change {side.capitalize()} in {math.ceil(plan.laneChangeCountdown)}'
  if str(plan.laneChange) == side:
    return f'Changing Lanes {side.capitalize()} Once Clear'
  return f'Signal {side.capitalize()} to Change Lanes'


def plan_desire(desire: int, lat_active: bool) -> tuple[int, bool]:
  """(desire to feed the model, clear the turn desire history) from turnpilotPlan.desire. No desire while openpilot
  isn't steering (carControl.latActive), so the model never plans a turn the driver didn't engage for; turnpilotd
  pulses afresh once it is. A clear goes through regardless: it only takes turn desires out of the model's queue."""
  if desire == CLEAR_DESIRE:
    return 0, True
  return (desire if lat_active else 0), False


def plan_accel(accel_cap: float, speed_cap: float, v_ego: float) -> float:
  """The longitudinal planner's accel candidate from a plan's caps."""
  return min(accel_cap, max((speed_cap - v_ego) / SPEED_CAP_TAU, SPEED_CAP_ACCEL_MIN))


# TurnPilot's turn signal (car_signals.py, DESIGN phase4/blinker 7.7): one event, turnpilotCarSignal, whose line is chosen
# here. The skips go through the tooFastToTurn alert (too_fast_title). (title, text, sound) with sound an AudibleAlert name
CAR_SIGNAL_FAULT_TEXT = {
  'lampTimeout': "Lamp didn't light", 'lampLost': "Signal went out: stalk touched?", 'lampStuck': "Lamp stuck on: cancel the signal",
  'wrongSide': "Car signalled the wrong side", 'noStockFrame': "No body-control message", 'capped': "Signal on too long",
  'recheckFailed': "Re-check failed",
}
CAR_SIGNAL_FAULT_S = 4.
CAR_SIGNAL_ONCE_S = 6.
CAR_SIGNAL_NOTICE_S = 3.
PROMPT_QUIET_S = 10.  # a prompt gone this long is a new one: chimed again
SKIP_TITLE = {'hazards': "Turn Cancelled: Hazards On", 'signalLost': "Turn Cancelled: Signal Went Out",
              'sideUncertain': "Turn Side Uncertain"}
CHECK_TITLE = {'waiting': None, 'reason': "Turn Signal Check", 'release': "Checking Left Signal", 'left': "Checking Left Signal",
               'right': "Checking Right Signal", 'pattern': "Checking Turn Signals", 'driverPress': "Signal Left Yourself",
               'passed': "Turn Signal Check Passed", 'failed': "Turn Signal Check Failed", 'aborted': "Turn Signal Check Stopped"}


def too_fast_title(plan) -> str | None:
  """The tooFastToTurn alert's title for a turn the turn signal's rules skipped (turnSignal.skip), or None."""
  skip = str(plan.turnSignal.skip)
  if skip == 'driverSignal':
    other = 'Left' if 'right' in str(plan.maneuverModifier) else 'Right'
    return f"Turn Cancelled: You Signalled {other}"
  return SKIP_TITLE.get(skip)


class CarSignalEvents:
  """selfdrived's turnpilotCarSignal: the turn signal's lines, their holds and once-per-drive logic (DESIGN 7.7). line is
  the (title, text, sound) to show while the event is raised; events.py reads it."""
  def __init__(self):
    self.line: tuple[str, str, str] | None = None
    self._held: tuple[str, str, str] | None = None
    self._held_t = 0.
    self._once: set[str] = set()
    self._fault_said = False
    self._still_said = False
    self._prompt_side = 'none'
    self._prompt_quiet = PROMPT_QUIET_S
    self._check_stage = 'idle'
    self._started = 0.

  def _hold(self, line, seconds: float) -> None:
    self._held, self._held_t = line, seconds

  def update(self, plan, car_signal, flags: int, dt: float) -> tuple[str, ...]:
    """plan: shown_plan's (None while stale); car_signal: carState.carSignal; flags: CarParams.flags on a Tesla."""
    from opendbc.car.tesla.values import TeslaFlags
    self._started += dt
    self.line = None
    ts = plan.turnSignal if plan is not None else None
    status = str(ts.status) if ts is not None else 'off'
    # the check and the re-check, from the port (carState.carSignal)
    stage = str(car_signal.check)
    if stage != self._check_stage and flags & TeslaFlags.CAR_SIGNALS_CHECK:
      title = CHECK_TITLE.get(stage)
      if title is not None:
        why = str(car_signal.checkWhy) or "Hands off the signal, stay stopped"
        sound = 'prompt' if stage in ('reason', 'passed', 'failed', 'aborted') else 'none'
        self._hold((title, why, sound), CAR_SIGNAL_FAULT_S if stage in ('passed', 'failed', 'aborted') else 1e9)
    self._check_stage = stage
    if flags & TeslaFlags.CAR_SIGNALS_RECHECK and not flags & TeslaFlags.CAR_SIGNALS_CHECK and stage in ('left', 'right', 'waiting') \
       and str(car_signal.status) == 'verifying':
      self.line = ("Turn Signal Re-check", "Stay in Park" if stage != 'waiting' else str(car_signal.checkWhy), 'none')
    # a fault, said once when it happens
    if status == 'failed' and not self._fault_said:
      self._fault_said = True
      fault = str(car_signal.fault)
      text = CAR_SIGNAL_FAULT_TEXT.get(fault, "Car signal failed")
      if fault == 'lampTimeout' and car_signal.rejected > 0:
        text = "Lamp didn't light: the panda refused it"
      if fault == 'recheckFailed' and str(car_signal.checkWhy):
        text = str(car_signal.checkWhy)
      self._hold(("Car Signal Failed", text + ". You'll be asked to signal", 'prompt'), CAR_SIGNAL_FAULT_S)
    # once per drive
    if status == 'unverified' and 'unverified' not in self._once:
      self._once.add('unverified')
      self._hold(("Car Signals Not Verified", str(ts.why) or "Run the turn signal check", 'prompt'), CAR_SIGNAL_ONCE_S)
    if status == 'unavailable' and 'unavailable' not in self._once:
      self._once.add('unavailable')
      self._hold(("Car Can't Signal", "No body-control message: you signal", 'prompt'), CAR_SIGNAL_ONCE_S)
    if status == 'ready' and flags & TeslaFlags.CAR_SIGNALS and not flags & TeslaFlags.CAR_SIGNALS_RECHECK and \
       'notRechecked' not in self._once:
      self._once.add('notRechecked')
      self._hold(("Car Signals Not Re-checked", "Verified in Drive: watch the first turn", 'prompt'), CAR_SIGNAL_ONCE_S)
    # notices
    if ts is not None and str(ts.handedOver) != 'none' and 'handedOver' not in self._once:
      self._once.add('handedOver')
      side = str(ts.handedOver)
      self._hold(("Signal Off", f"Signal {side} yourself", 'prompt'), CAR_SIGNAL_NOTICE_S)
    elif ts is not None and str(ts.handedOver) == 'none':
      self._once.discard('handedOver')
    notice = str(ts.notice) if ts is not None else 'none'
    if notice != 'none' and f'notice:{notice}' not in self._once:
      self._once.add(f'notice:{notice}')
      self._hold(("Turn Signal Changed", "Finishing the turn", 'prompt') if notice == 'changedMidTurn' else
                 ("Turn Signal Ended", "Signal ran past the turn", 'prompt'), CAR_SIGNAL_NOTICE_S)
    elif notice == 'none':
      self._once = {o for o in self._once if not o.startswith('notice:')}
    # a masked lamp lit past its release
    if car_signal.stillOn:
      if not self._still_said:
        self._still_said = True
        self.line = ("Turn Signal Still On", "Cancel it if you're not turning", 'prompt')
      else:
        self.line = ("Turn Signal Still On", "Cancel it if you're not turning", 'none')
    else:
      self._still_said = False
    # the prompt (car_signals.py 6.5): while it holds, chimed once per prompt
    prompt = str(ts.prompt) if ts is not None else 'none'
    if prompt != 'none':
      sound = 'none'
      if prompt != self._prompt_side or self._prompt_quiet >= PROMPT_QUIET_S:
        sound = 'tpLaneChangeLeft' if prompt == 'left' else 'tpLaneChangeRight'
      self._prompt_side, self._prompt_quiet = prompt, 0.
      if self.line is None:
        self.line = (f"Signal {prompt.capitalize()}", "For the turn ahead", sound)
    else:
      self._prompt_quiet += dt
    if self._held_t > 0.:
      self._held_t -= dt
      if self.line is None or self._held[2] != 'none':
        self.line = self._held
        self._held = (self._held[0], self._held[1], 'none') if self._held_t > 0. else self._held
    return ('turnpilotCarSignal',) if self.line is not None else ()


# the line selfdrived's turnpilotCarSignal shows: CarSignalEvents.update sets it each frame (one selfdrived per process)
CAR_SIGNAL_EVENTS = CarSignalEvents()
