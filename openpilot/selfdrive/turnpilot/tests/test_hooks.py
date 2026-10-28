"""The upstream hunks: what modeld, DesireHelper and the longitudinal planner do with turnpilotPlan."""
import time

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.turnpilot.hooks import (ABORT_HOLD_S, ABORT_TEXT, CLEAR_DESIRE, PLAN_MAX_AGE_S, LaneChangeEvents, active_plan,
                                               lane_change_events, lane_change_text, plan_desire, plan_events)
from openpilot.selfdrive.turnpilot.tests.synthetic import car_state

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
State = custom.TurnpilotPlan.State


class FakeSM(dict):
  """SubMaster's dicts, every message received just now and valid. alive stays true, as SubMaster keeps it in simulation."""
  def __init__(self, msgs, valid=True, age=0.):
    super().__init__(msgs)
    self.alive = dict.fromkeys(msgs, True)
    self.valid = dict.fromkeys(msgs, valid)
    self.recv_time = dict.fromkeys(msgs, time.monotonic() - age)


def turnpilot_plan(shadow, desire=log.Desire.turnLeft, lane_change='left', accel_cap=-3., speed_cap=2., should_stop=True,
                   state=State.turning, fault=custom.TurnpilotPlan.Fault.none):
  msg = messaging.new_message('turnpilotPlan')
  p = msg.turnpilotPlan
  p.fault = fault
  p.desire, p.laneChange, p.accelCap, p.speedCap, p.shouldStop, p.shadow = desire, lane_change, accel_cap, speed_cap, should_stop, shadow
  p.state = state
  return msg.as_reader().turnpilotPlan


def test_active_plan():
  assert active_plan(FakeSM({'turnpilotPlan': turnpilot_plan(False)})) is not None
  assert active_plan(FakeSM({'turnpilotPlan': turnpilot_plan(True)})) is None
  assert active_plan({'turnpilotPlan': turnpilot_plan(False)}) is None  # the maneuver tests' plain dict


def test_active_plan_needs_a_fresh_valid_enabled_plan():
  # alive is true throughout, as it stays in simulation once seen: freshness is by receive time alone
  assert active_plan(FakeSM({'turnpilotPlan': turnpilot_plan(False)}, age=PLAN_MAX_AGE_S + 0.1)) is None
  assert active_plan(FakeSM({'turnpilotPlan': turnpilot_plan(False)}, age=PLAN_MAX_AGE_S - 0.1)) is not None
  sm = FakeSM({'turnpilotPlan': turnpilot_plan(False)})
  assert active_plan(sm, now=sm.recv_time['turnpilotPlan'] + 60.) is None
  assert active_plan(FakeSM({'turnpilotPlan': turnpilot_plan(False)}, valid=False)) is None
  assert active_plan(FakeSM({'turnpilotPlan': turnpilot_plan(False, state=State.disabled)})) is None
  assert active_plan(messaging.SubMaster(['turnpilotPlan'])) is None  # a real SubMaster that never got one


def test_plan_events():
  Fault = custom.TurnpilotPlan.Fault
  for fault, events in ((Fault.none, []), (Fault.error, ['turnpilotUnavailable']), (Fault.noLongitudinal, ['turnpilotUnavailable']),
                        (Fault.disengageOnAccelerator, ['turnpilotUnavailable'])):
    msg = messaging.new_message('turnpilotPlan')
    msg.turnpilotPlan.state, msg.turnpilotPlan.fault = State.disabled, fault
    plan = msg.as_reader().turnpilotPlan
    assert plan_events(FakeSM({'turnpilotPlan': plan})) == events
    assert plan_events(FakeSM({'turnpilotPlan': plan}, age=PLAN_MAX_AGE_S + 0.1)) == []  # a dead turnpilotd is processNotRunning
  assert plan_events(messaging.SubMaster(['turnpilotPlan'])) == []
  # every event name exists, and has an alert
  from openpilot.selfdrive.selfdrived.events import EVENTS
  assert log.OnroadEvent.EventName.turnpilotUnavailable in EVENTS


def modeld_inputs(sm, dh_desire=log.Desire.none, lat_active=True):
  """modeld's turnpilot reads, as in its main loop: the desire fed to the model and DesireHelper's nav lane change."""
  tp = active_plan(sm)
  tp_desire = plan_desire(tp.desire, lat_active)[0] if tp is not None else log.Desire.none
  desire = tp_desire if dh_desire == log.Desire.none else dh_desire
  return desire, tp.laneChange.raw if tp is not None else LaneChangeDirection.none


def test_modeld_reads():
  assert modeld_inputs(FakeSM({'turnpilotPlan': turnpilot_plan(False)})) == (log.Desire.turnLeft, LaneChangeDirection.left)
  assert modeld_inputs(FakeSM({'turnpilotPlan': turnpilot_plan(True)})) == (log.Desire.none, LaneChangeDirection.none)
  # DesireHelper's own lane change desire wins
  assert modeld_inputs(FakeSM({'turnpilotPlan': turnpilot_plan(False)}), log.Desire.laneChangeRight)[0] == log.Desire.laneChangeRight
  # no desire while openpilot isn't steering; DesireHelper gates the lane change on latActive itself
  assert modeld_inputs(FakeSM({'turnpilotPlan': turnpilot_plan(False)}), lat_active=False) == (log.Desire.none, LaneChangeDirection.left)


def test_clear_desire():
  # the clear is fed to the model as no desire, and asks modeld to clear the turn desire history
  assert plan_desire(CLEAR_DESIRE, True) == (log.Desire.none, True)
  assert plan_desire(log.Desire.turnRight, True) == (log.Desire.turnRight, False)
  # with openpilot not steering, no desire, but the clear still goes
  assert plan_desire(log.Desire.turnRight, False) == (log.Desire.none, False)
  assert plan_desire(CLEAR_DESIRE, False) == (log.Desire.none, True)
  assert CLEAR_DESIRE >= ModelConstants.DESIRE_LEN and CLEAR_DESIRE not in log.Desire.schema.enumerants.values()


def torque_state(v, left=True):
  msg = messaging.new_message('carState')
  msg.carState.vEgo = v
  msg.carState.leftBlinker = left
  msg.carState.steeringPressed = True
  msg.carState.steeringTorque = 1. if left else -1.
  return msg.as_reader().carState


def blindspot_state(v):
  msg = messaging.new_message('carState')
  msg.carState.vEgo = v
  msg.carState.leftBlindspot = True
  return msg.as_reader().carState


def test_desire_helper_without_nav_is_upstream():
  dh = DesireHelper()
  for _ in range(3):
    dh.update(car_state(15.), True, 0.5)
  assert dh.lane_change_state == LaneChangeState.off
  dh.update(car_state(15., left_blinker=True), True, 0.5)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  dh.update(car_state(15., left_blinker=True), True, 0.5)
  assert dh.lane_change_state == LaneChangeState.preLaneChange  # waits for the nudge
  dh.update(torque_state(15.), True, 0.5)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting and dh.desire == log.Desire.laneChangeLeft


def test_nav_lane_change_is_blinker_plus_nudge():
  dh = DesireHelper()
  dh.update(car_state(15.), True, 0.5, LaneChangeDirection.right)
  assert (dh.lane_change_state, dh.lane_change_direction) == (LaneChangeState.preLaneChange, LaneChangeDirection.right)
  dh.update(car_state(15.), True, 0.5, LaneChangeDirection.right)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting and dh.desire == log.Desire.laneChangeRight


def test_nav_lane_change_keeps_blindspot_and_speed_checks():
  dh = DesireHelper()
  for _ in range(5):
    dh.update(blindspot_state(15.), True, 0.5, LaneChangeDirection.left)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  dh = DesireHelper()
  for _ in range(5):
    dh.update(car_state(7.), True, 0.5, LaneChangeDirection.left)
  assert dh.lane_change_state == LaneChangeState.off


def planner_sm(v=15., plan=None, age=0.):
  msgs = {s: messaging.new_message(s) for s in ('radarState', 'controlsState', 'selfdriveState', 'carState', 'modelV2')}
  msgs['controlsState'].controlsState.longControlState = LongCtrlState.pid
  msgs['carState'].carState.vEgo = v
  msgs['carState'].carState.vCruise = 30. * 3.6
  model = msgs['modelV2'].modelV2
  model.action.desiredAcceleration = 0.5
  for field in ('velocity', 'acceleration', 'position'):
    getattr(model, field).x = [float(v)] * ModelConstants.IDX_N
  if plan is not None:
    msgs['turnpilotPlan'] = plan
  return FakeSM({s: m if s == 'turnpilotPlan' else m.as_reader().__getattr__(s) for s, m in msgs.items()}, age=age)


def run_planner(plan, age=0.):
  from opendbc.car.honda.interface import CarInterface
  from opendbc.car.honda.values import CAR
  lp = LongitudinalPlanner(CarInterface.get_non_essential_params(CAR.HONDA_CIVIC), init_v=15.)
  out = []
  for _ in range(20):
    lp.update(planner_sm(plan=plan, age=age))
    out.append((float(lp.output_a_target), lp.output_should_stop))
  return out


def test_shadow_never_changes_planner_output():
  baseline = run_planner(None)
  assert run_planner(turnpilot_plan(shadow=True)) == baseline
  active = run_planner(turnpilot_plan(shadow=False))
  assert all(a < b for (a, _), (b, _) in zip(active, baseline, strict=True))
  assert all(stop for _, stop in active) and not any(stop for _, stop in baseline)
  # an active plan with no caps changes nothing either
  assert np.allclose(run_planner(turnpilot_plan(shadow=False, accel_cap=10., speed_cap=100., should_stop=False)), baseline)


def test_stale_or_disabled_plan_never_changes_planner_output():
  baseline = run_planner(None)
  assert run_planner(turnpilot_plan(shadow=False), age=PLAN_MAX_AGE_S + 0.1) == baseline
  assert run_planner(turnpilot_plan(shadow=False, state=State.disabled)) == baseline


def test_faulted_plan_keeps_its_stops():
  # a fault (turnpilotd failing a frame, or every frame) still carries the scene's stops in force: a car held for a
  # pedestrian stays held. Its desire and lane change are never acted on (modeld takes active_plan's)
  baseline = run_planner(None)
  faulted = turnpilot_plan(shadow=False, state=State.disabled, fault=custom.TurnpilotPlan.Fault.error)
  out = run_planner(faulted)
  assert all(a < b for (a, _), (b, _) in zip(out, baseline, strict=True)) and all(stop for _, stop in out)
  assert active_plan(FakeSM({'turnpilotPlan': faulted})) is None
  assert run_planner(turnpilot_plan(shadow=True, state=State.disabled, fault=custom.TurnpilotPlan.Fault.error)) == baseline


def test_plan_events_one_slot_by_priority():
  # handback, then fault, then too fast to turn, then a lane change, then wrong lane: at most one of TurnPilot's alerts a frame
  Fault, Handback, LaneMismatch = custom.TurnpilotPlan.Fault, custom.TurnpilotPlan.Handback, custom.TurnpilotPlan.LaneMismatch
  def events(handback=Handback.none, fault=Fault.none, too_fast=False, mismatch=LaneMismatch.none, offer='none', abort='none'):
    msg = messaging.new_message('turnpilotPlan')
    p = msg.turnpilotPlan
    p.state, p.handback, p.fault, p.tooFastToTurn, p.laneMismatch = State.turning, handback, fault, too_fast, mismatch
    p.laneChangeOffer, p.laneChangeAbort = offer, abort
    plan = msg.as_reader().turnpilotPlan
    return plan_events(FakeSM({'turnpilotPlan': plan})) + list(lane_change_events(plan))
  everything = {'fault': Fault.error, 'too_fast': True, 'mismatch': LaneMismatch.wrongLaneForTurn}
  assert events(Handback.detectorStopped, **everything) == []  # handback.py raises the handback's own event
  assert events(Handback.modelFellBack, **everything) == []
  assert events(**everything) == ['turnpilotUnavailable']
  assert events(too_fast=True, mismatch=LaneMismatch.wrongLaneForTurn) == ['turnpilotTooFastToTurn']
  assert events(mismatch=LaneMismatch.wrongLaneForTurn) == ['turnpilotWrongLane']
  # a lane change, offered or just aborted, outranks the wrong lane; too fast to turn and a fault outrank the lane change
  assert events(mismatch=LaneMismatch.turnOnlyGoingStraight, offer='left') == ['turnpilotLaneChangeLeft']
  assert events(mismatch=LaneMismatch.turnOnlyGoingStraight, abort='noSignal') == ['turnpilotLaneChangeAborted']
  assert events(too_fast=True, abort='routeChanged') == ['turnpilotTooFastToTurn']
  assert events(fault=Fault.error, abort='fault') == ['turnpilotUnavailable']


def lane_change_plan(offer='none', countdown=0., lane_change='none', abort='none', shadow=False):
  msg = messaging.new_message('turnpilotPlan')
  p = msg.turnpilotPlan
  p.laneChangeOffer, p.laneChangeCountdown, p.laneChange, p.laneChangeAbort, p.shadow = offer, countdown, lane_change, abort, shadow
  return msg.as_reader().turnpilotPlan


def test_lane_change_events_and_text():
  assert lane_change_events(None) == ()
  assert lane_change_events(lane_change_plan()) == ()
  assert lane_change_events(lane_change_plan('left', 2.)) == ('turnpilotLaneChangeLeft',)
  assert lane_change_events(lane_change_plan('right')) == ('turnpilotLaneChangeRight',)
  assert lane_change_events(lane_change_plan(abort='blindspot')) == ('turnpilotLaneChangeAborted',)
  # a new offer can follow an abort while its reason still holds
  assert lane_change_events(lane_change_plan('left', 3., abort='noSignal')) == ('turnpilotLaneChangeLeft', 'turnpilotLaneChangeAborted')
  assert lane_change_text(lane_change_plan('left', 2.2), 'left') == 'Lane Change Left in 3'
  assert lane_change_text(lane_change_plan('left', 0.), 'left') == 'Signal Left to Change Lanes'
  assert lane_change_text(lane_change_plan('right', 0., 'right'), 'right') == 'Changing Lanes Right Once Clear'
  assert all(reason in ABORT_TEXT for reason in custom.TurnpilotPlan.LaneChangeAbort.schema.enumerants if reason != 'none')


def test_lane_change_events_abort_when_the_plan_goes_away():
  ev = LaneChangeEvents()
  assert ev.update(lane_change_plan('left', 2.), 0.01) == ('turnpilotLaneChangeLeft',)
  # turnpilotd stops (or shadow mode comes on) under the offer: an abort, held for the alert, not a silent vanish
  n = 0
  while ev.update(None, 0.01) == ('turnpilotLaneChangeAborted',):
    n += 1
  assert abs(n * 0.01 - ABORT_HOLD_S) < 0.02 and ev.update(None, 0.01) == ()
  # gone with nothing offered: nothing to say
  ev = LaneChangeEvents()
  ev.update(lane_change_plan(), 0.01)
  assert ev.update(None, 0.01) == ()


def test_plan_events_map_controls_by_priority():
  # handback > fault > cannotStopComfortably > tooFastToTurn > controlPrompt > other controlAlerts > lane change >
  # wrongLane > the map controls' status (unavailable, light source rejected)
  Fault, Handback, LaneMismatch = custom.TurnpilotPlan.Fault, custom.TurnpilotPlan.Handback, custom.TurnpilotPlan.LaneMismatch
  def events(handback=Handback.none, fault=Fault.none, too_fast=False, mismatch=LaneMismatch.none, offer='none', prompt='none',
             alert='none'):
    msg = messaging.new_message('turnpilotPlan')
    p = msg.turnpilotPlan
    p.state, p.handback, p.fault, p.tooFastToTurn, p.laneMismatch = State.holding, handback, fault, too_fast, mismatch
    p.laneChangeOffer, p.controlPrompt, p.controlAlert = offer, prompt, alert
    plan = msg.as_reader().turnpilotPlan
    return plan_events(FakeSM({'turnpilotPlan': plan})) + list(lane_change_events(plan))
  assert events(Handback.detectorStopped, alert='cannotStopComfortably') == []
  assert events(fault=Fault.error, alert='cannotStopComfortably') == ['turnpilotUnavailable']
  assert events(too_fast=True, alert='cannotStopComfortably') == ['turnpilotControlAlert']
  assert events(too_fast=True, prompt='stopSignGo') == ['turnpilotTooFastToTurn']
  assert events(prompt='signalGo', alert='stopOverrun', offer='left') == ['turnpilotControlPrompt']
  assert events(alert='proceedingOnYellow', offer='left', mismatch=LaneMismatch.wrongLaneForTurn) == ['turnpilotControlAlert']
  assert events(alert='mapControlsUnavailable', offer='left') == ['turnpilotLaneChangeLeft']
  assert events(alert='mapControlsUnavailable', mismatch=LaneMismatch.wrongLaneForTurn) == ['turnpilotWrongLane']
  assert events(alert='lightSourceRejected') == ['turnpilotControlAlert']
  # an unprotected turn's junction read far apart: under its go prompt, over a lane change
  assert events(prompt='unprotectedTurnGo', alert='junctionUncertain') == ['turnpilotControlPrompt']
  assert events(alert='junctionUncertain', offer='left') == ['turnpilotControlAlert']
  assert events(prompt='yieldAhead') == ['turnpilotControlPrompt']
  # off the route with the stops ahead unknown: over too fast to turn and every prompt, under a fault
  assert events(too_fast=True, prompt='stopSignGo', alert='offRouteStopsUnknown', offer='left') == ['turnpilotControlAlert']
  assert events(fault=Fault.error, alert='offRouteStopsUnknown') == ['turnpilotUnavailable']
  # shadow: nothing acts, nothing is said
  msg = messaging.new_message('turnpilotPlan')
  msg.turnpilotPlan.shadow, msg.turnpilotPlan.controlPrompt = True, 'stopSignGo'
  assert plan_events(FakeSM({'turnpilotPlan': msg.as_reader().turnpilotPlan})) == []


def test_control_texts():
  from openpilot.selfdrive.turnpilot.hooks import CONTROL_ALERT_TEXT, CONTROL_PROMPT_TEXT, RETIRED_ALERTS, RETIRED_PROMPTS
  prompts = [p for p in custom.TurnpilotPlan.ControlPrompt.schema.enumerants if p not in ('none',) + RETIRED_PROMPTS]
  alerts = [a for a in custom.TurnpilotPlan.ControlAlert.schema.enumerants if a not in ('none',) + RETIRED_ALERTS]
  assert all(p in CONTROL_PROMPT_TEXT for p in prompts) and all(a in CONTROL_ALERT_TEXT for a in alerts)
  assert not any('accelerator' in t or 'tap gas' in t for t in list(CONTROL_PROMPT_TEXT.values()) + list(CONTROL_ALERT_TEXT.values()))


def test_no_gas_go_is_ever_sent():
  # the retired gas-go prompts and goingOnRed appear nowhere in what TurnPilot sends from (no gas go, 2026-10-28)
  from pathlib import Path
  from openpilot.selfdrive.turnpilot.hooks import RETIRED_ALERTS, RETIRED_PROMPTS
  root = Path(__file__).parents[1]
  for name in ('map_controls.py', 'turnpilotd.py', 'scene_policy.py', 'control_text.py'):
    text = (root / name).read_text()
    assert not any(f"'{p}'" in text for p in RETIRED_PROMPTS + RETIRED_ALERTS), name
    assert 'tap gas' not in text and 'press accelerator' not in text, name


def test_stale_hold_is_loud():
  # turnpilotd dies with the car held at a stop line: plannerd drops shouldStop at once (PLAN_MAX_AGE_S), and the driver is
  # told at once, not when the manager notices the process is gone
  from openpilot.selfdrive.turnpilot.hooks import StaleHoldEvents, fresh_plan, limits_plan

  def update(ev, sm):
    return ev.update(limits_plan(sm), fresh_plan(sm))
  held = turnpilot_plan(False, desire=0, lane_change='none', accel_cap=-0.5, speed_cap=0., should_stop=True, state=State.holding)
  ev = StaleHoldEvents()
  assert update(ev, FakeSM({'turnpilotPlan': held})) == ()
  stale = FakeSM({'turnpilotPlan': held}, age=PLAN_MAX_AGE_S + 0.05)
  assert limits_plan(stale) is None
  assert update(ev, stale) == ('turnpilotControlAlert',)
  assert update(ev, stale) == ('turnpilotControlAlert',)  # until it is back
  assert update(ev, FakeSM({'turnpilotPlan': held})) == ()
  # a plan that wasn't holding going stale is processNotRunning's alone
  free = turnpilot_plan(False, should_stop=False, accel_cap=10., speed_cap=100.)
  ev = StaleHoldEvents()
  update(ev, FakeSM({'turnpilotPlan': free}))
  assert update(ev, FakeSM({'turnpilotPlan': free}, age=PLAN_MAX_AGE_S + 0.05)) == ()
  # held, then TurnPilot turned off or shadow mode on: a fresh plan with no limits is not a stale one, nothing to say
  for off in (turnpilot_plan(False, state=State.disabled, should_stop=False), turnpilot_plan(True, should_stop=True)):
    ev = StaleHoldEvents()
    update(ev, FakeSM({'turnpilotPlan': held}))
    assert update(ev, FakeSM({'turnpilotPlan': off})) == ()
    assert update(ev, FakeSM({'turnpilotPlan': off}, age=PLAN_MAX_AGE_S + 0.05)) == ()


def test_a_missed_turn_takes_the_too_fast_to_turn_slot():
  # a turn missed from a standstill (turnMissed) is said where a skipped one is: after a fault and a cannot-stop, before a
  # map prompt, and over a lane change
  def events(**fields):
    msg = messaging.new_message('turnpilotPlan')
    for k, v in fields.items():
      setattr(msg.turnpilotPlan, k, v)
    plan = msg.as_reader().turnpilotPlan
    return plan_events(FakeSM({'turnpilotPlan': plan})) + list(lane_change_events(plan))
  assert events(turnMissed=True) == ['turnpilotTooFastToTurn']
  assert events(turnMissed=True, laneChangeOffer='left', laneMismatch='wrongLaneForTurn') == ['turnpilotTooFastToTurn']
  assert events(turnMissed=True, controlPrompt='carStoppedGo') == ['turnpilotTooFastToTurn']
  assert events(turnMissed=True, fault='error') == ['turnpilotUnavailable']
  assert events(turnMissed=True, shadow=True) == []
