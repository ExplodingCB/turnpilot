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


def test_map_control_prompts_hand_back_loudly_and_yield_is_silent():
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='stopSignTakeControl', reason='Stop sign: take control to proceed')))
  assert a.alert_text_1 == 'Stop sign: take control to proceed' and a.alert_text_2 == "TurnPilot can't check cross traffic"
  assert a.audible_alert == AudibleAlert.tpHandback
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='yieldAhead')))
  assert a.audible_alert == AudibleAlert.none
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='unprotectedTurnTakeControl')))
  assert a.alert_text_1 == 'Unprotected Turn: Take Control' and a.audible_alert == AudibleAlert.tpHandback


def test_map_control_alerts():
  a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert='cannotStopComfortably')))
  assert a.alert_text_1 == 'Take Control' and a.audible_alert == AudibleAlert.tpHandback
  for kind in ('stopOverrun', 'proceedingOnYellow', 'junctionUncertain'):
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


def test_cannot_stop_outranks_and_the_handback_chimes_while_it_lasts():
  slot = AlertSlot()
  a = slot.step(plan(controlAlert='cannotStopComfortably', tooFastToTurn=True))
  assert a.alert_type == 'turnpilotControlAlert/warning'
  sounds = []
  for _ in range(300):
    a = slot.step(plan(controlPrompt='stopSignTakeControl', reason='Stop sign: take control to proceed'))
    sounds.append(a.audible_alert)
  assert a.alert_type == 'turnpilotControlPrompt/warning'
  assert all(s == AudibleAlert.tpHandback for s in sounds[20:])


def test_a_hard_brake_for_a_confirmed_pedestrian_is_raised_first_with_the_handback_chime():
  a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert='hardBrake', reason='pedestrian in path')))
  assert a.alert_text_1 == 'Braking Hard' and a.alert_text_2 == 'pedestrian in path'
  assert a.audible_alert == AudibleAlert.tpHandback and a.visual_alert == VisualAlert.fcw
  from openpilot.selfdrive.turnpilot.hooks import plan_events
  assert plan_events(FakeSM(plan(controlAlert='hardBrake', tooFastToTurn=True))) == ['turnpilotControlAlert']


def test_take_control_prompts_are_loud_and_outrank_a_skip():
  from openpilot.selfdrive.turnpilot.hooks import plan_events
  for prompt, title in (('stopSignTakeControl', 'Stop sign: take control to proceed'),
                        ('unprotectedTurnTakeControl', 'Unprotected left: take control'),
                        ('cantStartTakeControl', "Can't start the turn: take control")):
    a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt=prompt, reason=title)))
    assert a.alert_text_1 == title and a.audible_alert == AudibleAlert.tpHandback
    assert a.visual_alert == VisualAlert.steerRequired and 'accelerator' not in a.alert_text_2
    assert plan_events(FakeSM(plan(controlPrompt=prompt, tooFastToTurn=True))) == ['turnpilotControlPrompt']


def test_a_go_the_car_never_moved_on_hands_back_loudly():
  # turnpilotd's no-move handback (G3): the car didn't pull away after TurnPilot's go, which is withdrawn
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='cantStartTakeControl')))
  assert a.alert_text_1 == "Can't Start Turn: Take Control" and a.alert_text_2 == "The car didn't pull away"
  assert a.audible_alert == AudibleAlert.tpHandback and a.visual_alert == VisualAlert.steerRequired


def test_the_unprotected_lefts_wait_is_silent_its_go_chimes_and_its_fault_hands_back():
  # DESIGN phase 4 section 2, G4: waiting for a gap and creeping for a view are visual only (no intervention: waiting is
  # ordinary); the go is a short chime; undecided (a fault) is the handback; a withdrawn go is an alert above the wait
  from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS, plan_events
  for prompt, title in (('waitingForGap', 'Waiting for a gap'), ('creepingForView', 'Creeping for a view')):
    a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt=prompt, reason=title)))
    assert a.alert_text_1 == title and a.audible_alert == AudibleAlert.none and a.visual_alert == VisualAlert.none
    assert prompt not in TAKE_CONTROL_PROMPTS
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='leftGo', reason='Turning')))
  assert a.alert_text_1 == 'Turning' and a.audible_alert == AudibleAlert.prompt
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='leftUndecidedTakeControl',
                                                            reason="View of oncoming traffic blocked: take control")))
  assert a.alert_text_1 == "View of oncoming traffic blocked: take control" and a.audible_alert == AudibleAlert.tpHandback
  assert a.visual_alert == VisualAlert.steerRequired and 'leftUndecidedTakeControl' in TAKE_CONTROL_PROMPTS
  assert plan_events(FakeSM(plan(controlPrompt='leftUndecidedTakeControl', tooFastToTurn=True))) == ['turnpilotControlPrompt']
  a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert='oncomingHolding')))
  assert a.alert_text_1 == 'Oncoming: Holding' and a.audible_alert == AudibleAlert.prompt
  assert plan_events(FakeSM(plan(controlAlert='oncomingHolding', controlPrompt='waitingForGap'))) == ['turnpilotControlAlert']


def test_the_detector_view_and_the_lefts_status_are_in_the_plan():
  p = plan(detectorView='road')
  assert str(p.detectorView) == 'road' and str(plan().detectorView) == 'auto'
  msg = messaging.new_message('turnpilotPlan')
  ul = msg.turnpilotPlan.unprotectedLeft
  ul.state, ul.rReq, ul.tReq, ul.clearS, ul.why = 'waitGap', 134., 8.4, 1.2, ''
  assert str(msg.as_reader().turnpilotPlan.unprotectedLeft.state) == 'waitGap'


def test_a_left_skipped_for_the_detectors_range_says_so():
  msg = messaging.new_message('turnpilotPlan')
  msg.turnpilotPlan.tooFastToTurn = True
  msg.turnpilotPlan.unprotectedLeft.state = 'skipped'
  a, = alerts(EventName.turnpilotTooFastToTurn, FakeSM(msg.as_reader().turnpilotPlan))
  assert a.alert_text_1 == "Can't See Far Enough to Turn" and a.alert_text_2 == 'Rerouting' and a.audible_alert == AudibleAlert.tpTooFastTurn


def test_side_cameras_off_is_a_status_line():
  from openpilot.selfdrive.turnpilot.hooks import CONTROL_ALERT_TEXT, CONTROL_STATUS_ALERTS
  a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert='sideCamerasOff')))
  assert a.alert_text_1 == 'Side Cameras Off' and a.alert_text_2 == CONTROL_ALERT_TEXT['sideCamerasOff']
  assert 'sideCamerasOff' in CONTROL_STATUS_ALERTS


def test_the_stop_sign_deciders_schema_is_appended_in_order():
  # STOP_SIGNS.md v3 SG4a, appended only: the plan's stopGo @34 after farCropBearing @33, MapControlStatus.Action @9-@12 after
  # infeasible @8, ControlPrompt @15-@16 after leftUndecidedTakeControl @14, ControlAlert @13-@15 after sideCamerasOff @12
  from openpilot.cereal import custom
  tp = custom.TurnpilotPlan
  assert tp.schema.fields['stopGo'].proto.ordinal.explicit == 34
  actions = tp.MapControlStatus.Action.schema.enumerants
  assert [actions[a] for a in ('waitingCross', 'creeping', 'going', 'committed')] == [9, 10, 11, 12]
  prompts = tp.ControlPrompt.schema.enumerants
  assert (prompts['waitingForCross'], prompts['stopGo']) == (15, 16)
  alerts_ = tp.ControlAlert.schema.enumerants
  assert [alerts_[a] for a in ('crossTrafficAfterCommit', 'cameraPaused', 'stalledInJunction')] == [13, 14, 15]
  fields = list(tp.StopGoStatus.schema.fields)
  for f in ('state', 'reason', 'ledger', 'wouldGo', 'maneuver', 'lanes', 'arrivalRank', 'dEdgeMin', 'dEdgeMax', 'committed', 'inLane',
            'retries', 'creptM', 'sideViewsLive', 'mapsValid', 'mode'):
    assert f in fields, f
  assert list(tp.StopGoStatus.Lane.schema.fields)[:7] == ['role', 'index', 'approach', 'rSeen', 'rReq', 'binding', 'shielded']


def test_the_stop_sign_wait_is_silent_its_go_chimes_and_its_faults_hand_back():
  # SG4a: waiting for cross traffic is ordinary (visual only), with an info chime at WAIT_INFO_S (stopGo.chime); the go is a
  # short chime as the left's; a fault is stopSignTakeControl with its reason line; the alerts after commit and while the
  # camera is paused are said, never a handback
  from openpilot.selfdrive.turnpilot.hooks import CONTROL_ALERT_TEXT, SILENT_PROMPTS, TAKE_CONTROL_PROMPTS, plan_events
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='waitingForCross', reason='Waiting: car from the left')))
  assert a.alert_text_1 == 'Waiting: car from the left' and a.audible_alert == AudibleAlert.none and a.visual_alert == VisualAlert.none
  assert 'waitingForCross' in SILENT_PROMPTS and 'waitingForCross' not in TAKE_CONTROL_PROMPTS
  msg = messaging.new_message('turnpilotPlan')
  msg.turnpilotPlan.controlPrompt, msg.turnpilotPlan.reason = 'waitingForCross', 'Waiting for cross traffic'
  msg.turnpilotPlan.stopGo.chime = True
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(msg.as_reader().turnpilotPlan))
  assert a.audible_alert == AudibleAlert.prompt
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='stopGo', reason='Going')))
  assert a.alert_text_1 == 'Going' and a.audible_alert == AudibleAlert.prompt and 'stopGo' not in TAKE_CONTROL_PROMPTS
  a, = alerts(EventName.turnpilotControlPrompt, FakeSM(plan(controlPrompt='stopSignTakeControl',
                                                            reason='Cross-traffic camera unavailable: take control')))
  assert a.alert_text_1 == 'Cross-traffic camera unavailable: take control' and a.audible_alert == AudibleAlert.tpHandback
  for kind, title in (('crossTrafficAfterCommit', 'Cross Traffic'), ('cameraPaused', 'Camera Paused'),
                      ('stalledInJunction', 'Stalled in Junction')):
    a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert=kind)))
    assert a.alert_text_1 == title and a.audible_alert == AudibleAlert.prompt and a.alert_text_2 == CONTROL_ALERT_TEXT[kind], kind
    assert plan_events(FakeSM(plan(controlAlert=kind))) == ['turnpilotControlAlert']


def test_a_failed_junction_override_write_is_appended_and_said_with_a_chime():
  # STOP_SIGNS.md O16: a TurnPilotJunctionOverrides entry turnpilotd couldn't write (a signal head seen at a stop sign) is
  # never silently lost: ControlAlert overrideWriteFailed @16, after stalledInJunction @15, said with a chime, never a handback
  from openpilot.cereal import custom
  from openpilot.selfdrive.turnpilot.hooks import CONTROL_ALERT_FIRST, CONTROL_ALERT_TEXT, plan_events
  assert custom.TurnpilotPlan.ControlAlert.schema.enumerants['overrideWriteFailed'] == 16
  a, = alerts(EventName.turnpilotControlAlert, FakeSM(plan(controlAlert='overrideWriteFailed')))
  assert a.alert_text_1 == 'Junction Not Saved' and a.alert_text_2 == CONTROL_ALERT_TEXT['overrideWriteFailed']
  assert a.audible_alert == AudibleAlert.prompt and 'overrideWriteFailed' in CONTROL_ALERT_FIRST
  assert plan_events(FakeSM(plan(controlAlert='overrideWriteFailed'))) == ['turnpilotControlAlert']


def test_the_stop_sign_ladders_other_turn_is_appended_and_said_with_a_chime_naming_the_turn():
  # STOP_SIGNS.md v3 SG4c, 7.1 rung 2 and O12: ControlAlert otherTurn @17 after overrideWriteFailed @16, StopGoStatus.alternate
  # @23 after fault @22; said with a chime, its title the turn the decider took (stopGo.maneuver), never a handback
  from openpilot.cereal import custom
  from openpilot.selfdrive.turnpilot.hooks import CONTROL_ALERT_FIRST, CONTROL_ALERT_TEXT, TAKE_CONTROL_PROMPTS, plan_events
  tp = custom.TurnpilotPlan
  assert tp.ControlAlert.schema.enumerants['otherTurn'] == 17
  assert tp.StopGoStatus.schema.fields['alternate'].proto.ordinal.explicit == 23
  for maneuver, title in (('right', 'Turning Right Instead'), ('left', 'Turning Left Instead'), ('straight', 'Going Straight Instead')):
    msg = messaging.new_message('turnpilotPlan')
    msg.turnpilotPlan.controlAlert = 'otherTurn'
    msg.turnpilotPlan.stopGo.maneuver, msg.turnpilotPlan.stopGo.alternate = maneuver, True
    a, = alerts(EventName.turnpilotControlAlert, FakeSM(msg.as_reader().turnpilotPlan))
    assert a.alert_text_1 == title and a.alert_text_2 == CONTROL_ALERT_TEXT['otherTurn'] and a.audible_alert == AudibleAlert.prompt
  assert 'otherTurn' in CONTROL_ALERT_FIRST and 'otherTurn' not in TAKE_CONTROL_PROMPTS
  assert plan_events(FakeSM(plan(controlAlert='otherTurn'))) == ['turnpilotControlAlert']
