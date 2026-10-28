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


CONTROL_ALERT_FIRST = ('hardBrake', 'cannotStopComfortably', 'offRouteStopsUnknown')  # above too fast to turn
# handbacks: the car held while the driver is told to take control (map_controls.py, turnpilotd.py), above too fast to turn
TAKE_CONTROL_PROMPTS = ('stopSignTakeControl', 'unprotectedTurnTakeControl', 'carStoppedTakeControl', 'stalledTakeControl')
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
     or stalled in a turn), then a map stop it can't make gently (turnpilotControlAlert, cannotStopComfortably: take
     control), or off the route with the map stops ahead unknown (offRouteStopsUnknown: the speed capped, take control
     or wait for the reroute);
  4. too fast to turn, or a turn from a standstill missed (turnpilotTooFastToTurn, turnMissed);
  5. a map control's other prompt (turnpilotControlPrompt: yield ahead, visual only);
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
