"""TurnPilot's lane change events: the alerts selfdrived builds from turnpilotPlan."""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.selfdrive.selfdrived.events import ET, Events

EventName = log.OnroadEvent.EventName
AudibleAlert = log.SelfdriveState.AudibleAlert
VisualAlert = car.CarControl.HUDControl.VisualAlert


class FakeSM(dict):
  """SubMaster's dicts: the plan received just now and valid, or never received (hooks.fresh_plan reads recv_time)."""
  def __init__(self, plan=None, alive=True):
    super().__init__(turnpilotPlan=plan)
    received = alive and plan is not None
    self.alive = {'turnpilotPlan': received}
    self.valid = {'turnpilotPlan': received}
    self.recv_time = {'turnpilotPlan': time.monotonic() if received else None}


def plan(**fields):
  msg = messaging.new_message('turnpilotPlan')
  for k, v in fields.items():
    setattr(msg.turnpilotPlan, k, v)
  return msg.as_reader().turnpilotPlan


def alerts(name, sm):
  events = Events()
  events.add(name)
  return events.create_alerts([ET.WARNING], [car.CarParams.new_message(), car.CarState.new_message(), sm, False, 0, 0])


def test_lane_change_warning_left_and_right():
  for name, side, chime in ((EventName.turnpilotLaneChangeLeft, 'left', AudibleAlert.tpLaneChangeLeft),
                            (EventName.turnpilotLaneChangeRight, 'right', AudibleAlert.tpLaneChangeRight)):
    a, = alerts(name, FakeSM(plan(laneChangeOffer=side, laneChangeCountdown=2.5)))
    assert a.alert_text_1 == f'Lane Change {side.capitalize()} in 3' and a.audible_alert == chime
    a, = alerts(name, FakeSM(plan(laneChangeOffer=side)))
    assert a.alert_text_1 == f'Signal {side.capitalize()} to Change Lanes'
    assert a.alert_type == f'turnpilotLaneChange{side.capitalize()}/warning'  # separate names: a chime per side later


def test_lane_change_aborted():
  a, = alerts(EventName.turnpilotLaneChangeAborted, FakeSM(plan(laneChangeAbort='blindspot')))
  assert a.alert_text_1 == 'Lane Change Cancelled' and a.alert_text_2 == 'Car in blind spot'
  assert a.audible_alert == AudibleAlert.tpLaneChangeAbort and a.duration >= 100  # 1 s at least, so it is seen
  a, = alerts(EventName.turnpilotLaneChangeAborted, FakeSM(None, alive=False))
  assert a.alert_text_2 == 'TurnPilot not responding'


def test_turnpilot_chimes_on_skip_and_handback():
  a, = alerts(EventName.turnpilotTooFastToTurn, FakeSM(plan()))
  assert a.audible_alert == AudibleAlert.tpTooFastTurn and a.alert_text_1 == 'Too Fast to Turn'
  # skipped because a map stop left no room to get into the turn's lane: said so
  a, = alerts(EventName.turnpilotTooFastToTurn, FakeSM(plan(laneChangeWait='unreachable')))
  assert a.alert_text_1 == "Can't Reach Turn Lane" and a.alert_text_2 == 'Rerouting' and a.audible_alert == AudibleAlert.tpTooFastTurn
  # a turn from a standstill driven straight past its junction: missed, said so, with the same chime
  a, = alerts(EventName.turnpilotTooFastToTurn, FakeSM(plan(turnMissed=True)))
  assert a.alert_text_1 == "Turn Missed" and a.alert_text_2 == 'Rerouting' and a.audible_alert == AudibleAlert.tpTooFastTurn
  for name in (EventName.turnpilotDetectorStopped, EventName.turnpilotNoDetector, EventName.turnpilotModelFellBack):
    events = Events()
    events.add(name)
    a, = events.create_alerts([ET.SOFT_DISABLE], [car.CarParams.new_message(), car.CarState.new_message(), FakeSM(plan()), False, 300, 0])
    assert a.alert_text_1 == 'TAKE CONTROL IMMEDIATELY' and a.audible_alert == AudibleAlert.tpHandback


class AlertSlot:
  """selfdrived's TurnPilot alerts frame by frame, as it raises them (plan_events, LaneChangeEvents) into AlertManager,
  engaged (WARNING alerts shown)."""
  def __init__(self):
    from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
    from openpilot.selfdrive.turnpilot.hooks import LaneChangeEvents
    self.am, self.lane_events, self.frame = AlertManager(), LaneChangeEvents(), 0

  def step(self, p):
    from openpilot.selfdrive.turnpilot.hooks import plan_events, shown_plan
    sm = FakeSM(p)
    events = Events()
    for name in plan_events(sm) + list(self.lane_events.update(shown_plan(sm), 0.01)):
      events.add(getattr(EventName, name))
    self.am.add_many(self.frame, events.create_alerts([ET.PERMANENT, ET.WARNING], [car.CarParams.new_message(), car.CarState.new_message(),
                                                                                 sm, False, 0, 0]))
    self.am.process_alerts(self.frame, set())
    self.frame += 1
    return self.am.current_alert


def test_lane_change_outranks_wrong_lane():
  # a change offered for a turn beyond an intersection where the car is in a turn-only lane: the wrong lane is confirmed a
  # second after the offer, and must not hide its countdown and its prompt to signal
  slot = AlertSlot()
  for frame in range(300):
    mismatch = 'turnOnlyGoingStraight' if frame >= 100 else 'none'
    a = slot.step(plan(laneChangeOffer='right', laneChangeCountdown=max(3. - frame * 0.01, 0.), laneMismatch=mismatch))
    assert a.alert_type == 'turnpilotLaneChangeRight/warning', frame
  # the change gone, the wrong lane is said
  for _ in range(300):
    a = slot.step(plan(laneMismatch='turnOnlyGoingStraight'))
  assert a.alert_type == 'turnpilotWrongLane/warning'


def test_too_fast_outranks_the_abort_it_causes():
  # a lane change aborted earlier in the drive, then an offered one dropped by a too-fast skip on the skip's frame: the
  # driver is told why (too fast, rerouting) at once, not "Lane Change Cancelled: Route changed"
  slot = AlertSlot()
  for _ in range(10):
    slot.step(plan(laneChangeOffer='left'))
  for _ in range(400):
    slot.step(plan(laneChangeAbort='blindspot'))
  for _ in range(10):
    slot.step(plan(laneChangeOffer='left'))
  a = slot.step(plan(tooFastToTurn=True, laneChangeAbort='routeChanged'))
  assert a.alert_type == 'turnpilotTooFastToTurn/warning' and a.audible_alert == AudibleAlert.tpTooFastTurn


def test_a_fault_while_engaged_is_heard_and_seen():
  # turnpilotd fails one frame (its plan disabled, the fault on it) and recovers: the driver hears it, and it stays up
  # long enough to read, over the abort of the change it cancelled
  slot = AlertSlot()
  for _ in range(10):
    slot.step(plan(laneChangeOffer='left'))
  a = slot.step(plan(state='disabled', fault='error', reason='TurnPilot error', laneChangeAbort='fault'))
  assert a.alert_type == 'turnpilotUnavailable/warning' and a.audible_alert != AudibleAlert.none
  for _ in range(200):
    a = slot.step(plan())
  assert a.alert_type == 'turnpilotUnavailable/warning'  # 2 s after a single bad frame


def test_offer_hides_only_the_stock_prompt_for_its_own_side():
  from openpilot.selfdrive.turnpilot.hooks import hides_stock_lane_change
  Dir = log.LaneChangeDirection
  assert hides_stock_lane_change(('turnpilotLaneChangeLeft',), Dir.left)
  assert not hides_stock_lane_change(('turnpilotLaneChangeLeft',), Dir.right)  # the driver's own change the other way
  assert hides_stock_lane_change(('turnpilotLaneChangeRight',), Dir.right)
  assert not hides_stock_lane_change(('turnpilotLaneChangeRight',), Dir.left)
  assert not hides_stock_lane_change(('turnpilotLaneChangeAborted',), Dir.left)
  assert not hides_stock_lane_change((), Dir.right)


def test_lane_not_confirmed_is_said_with_a_chime():
  a, = alerts(EventName.turnpilotWrongLane, FakeSM(plan(laneMismatch='laneUnknown')))
  assert a.alert_text_1 == "Can't Confirm Lane" and a.audible_alert == AudibleAlert.prompt
  a, = alerts(EventName.turnpilotWrongLane, FakeSM(plan(laneMismatch='wrongLaneForTurn')))
  assert a.alert_text_1 == "Wrong Lane" and a.audible_alert == AudibleAlert.none


def test_map_control_prompts_chime_and_yield_is_silent():
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='stopSignGo', reason='Stopped at stop sign — tap gas to go')))
  assert a.alert_text_1 == 'Stopped at stop sign — tap gas to go' and 'press accelerator to go' in a.alert_text_2
  assert a.audible_alert == AudibleAlert.prompt
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='signalGo', reason='Signal ahead: light not checked')))
  assert 'press accelerator to proceed' in a.alert_text_2 and a.audible_alert == AudibleAlert.prompt
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='yieldAhead')))
  assert a.audible_alert == AudibleAlert.none
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='unprotectedTurnGo',
                                                           reason='Stopped at unprotected left — tap gas to go')))
  assert a.alert_text_1 == 'Stopped at unprotected left — tap gas to go' and 'check oncoming traffic' in a.alert_text_2
  assert a.audible_alert == AudibleAlert.prompt
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='unprotectedTurnGo')))
  assert a.alert_text_1 == 'Unprotected Turn'
  # the model stopped on its own, nothing holding the car: the driver's go, with a chime
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='carStoppedGo', reason='Car stopped — tap gas to continue')))
  assert a.alert_text_1 == 'Car stopped — tap gas to continue' and 'press accelerator to continue' in a.alert_text_2
  assert a.audible_alert == AudibleAlert.prompt


def test_map_control_alerts():
  a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert='cannotStopComfortably')))
  assert a.alert_text_1 == 'Take Control' and a.audible_alert == AudibleAlert.tpHandback
  for kind in ('stopOverrun', 'proceedingOnYellow', 'goingOnRed', 'junctionUncertain'):
    a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert=kind)))
    assert a.audible_alert == AudibleAlert.prompt and a.duration >= 200, kind
  for kind in ('mapControlsUnavailable', 'lightSourceRejected'):
    a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert=kind, reason='Map stops/signals: unsupported')))
    assert a.audible_alert == AudibleAlert.prompt, kind
  # off the route with the map stops ahead unknown: up while it lasts, with a chime and the take-control hint
  a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert='offRouteStopsUnknown')))
  assert a.alert_text_1 == 'Off Route' and 'take control or wait for reroute' in a.alert_text_2
  assert a.audible_alert == AudibleAlert.prompt and a.visual_alert == VisualAlert.steerRequired
  # turnpilotd gone while holding: the hold is lost, said as take control
  a, = alerts(EventName.turnpilotControlAlert, FakeSM(None, alive=False))
  assert a.alert_text_1 == 'TAKE CONTROL' and a.audible_alert == AudibleAlert.tpHandback


def test_cannot_stop_outranks_and_the_prompt_chimes_once():
  slot = AlertSlot()
  a = slot.step(plan(controlAlert='cannotStopComfortably', tooFastToTurn=True))
  assert a.alert_type == 'turnpilotControlAlert/warning'
  sounds = []
  for _ in range(300):
    a = slot.step(plan(controlPrompt='stopSignGo', reason='Stopped at stop sign — tap gas to go'))
    sounds.append(a.audible_alert)
  assert a.alert_type == 'turnpilotControlPrompt/warning'
  # once the take-control alert's own short hold is over, one prompt alert held up: soundd plays its one-shot sound once
  assert all(s == AudibleAlert.prompt for s in sounds[20:])


def test_a_hard_brake_for_a_confirmed_pedestrian_is_raised_first_with_the_handback_chime():
  a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert='hardBrake', reason='pedestrian in path')))
  assert a.alert_text_1 == 'Braking Hard' and a.alert_text_2 == 'pedestrian in path'
  assert a.audible_alert == AudibleAlert.tpHandback and a.visual_alert == VisualAlert.fcw
  from openpilot.selfdrive.turnpilot.hooks import plan_events
  assert plan_events(FakeSM(plan(controlAlert='hardBrake', tooFastToTurn=True))) == ['turnpilotControlAlert']
