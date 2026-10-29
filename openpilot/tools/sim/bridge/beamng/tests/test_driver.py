"""The sim's driver waits out TurnPilot's lane change countdown before working the stalk."""
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.tools.sim.bridge.beamng.driver import PromptedDriver

LaneChange = custom.TurnpilotPlan.LaneChange


class FakeSM(dict):
  def __init__(self):
    super().__init__()
    self.alive = {'turnpilotPlan': True}
    self['modelV2'] = messaging.new_message('modelV2').modelV2

  def update(self, timeout=0):
    pass

  def plan(self, prompt, countdown):
    msg = messaging.new_message('turnpilotPlan')
    msg.turnpilotPlan.signalPrompt, msg.turnpilotPlan.laneChangeOffer = prompt, prompt
    msg.turnpilotPlan.laneChangeCountdown = countdown
    self['turnpilotPlan'] = msg.as_reader().turnpilotPlan

  def turn(self, prompt, side):
    """turnpilotPlan.turnSignal: the turn's side, and the prompt to signal it while the car's own signal isn't ready."""
    msg = messaging.new_message('turnpilotPlan')
    msg.turnpilotPlan.turnSignal.prompt, msg.turnpilotPlan.turnSignal.side = prompt, side
    self['turnpilotPlan'] = msg.as_reader().turnpilotPlan


def new_driver():
  driver = PromptedDriver.__new__(PromptedDriver)
  driver.sm, driver.side, driver.started, driver.signals = FakeSM(), LaneChange.none, False, 0
  driver.turn_side, driver.turn_signals = LaneChange.none, 0
  return driver


def test_stalk_waits_for_the_countdown():
  driver = new_driver()
  for countdown in (3., 2., 0.5):  # the warning: prompted, but the countdown is still running
    driver.sm.plan(LaneChange.left, countdown)
    assert driver.update() == (False, False)
  driver.sm.plan(LaneChange.left, 0.)
  assert driver.update() == (True, False) and driver.signals == 1
  # held until the change starts, then let go once the prompt is gone
  msg = messaging.new_message('modelV2')
  msg.modelV2.meta.laneChangeState = log.LaneChangeState.laneChangeStarting
  driver.sm['modelV2'] = msg.as_reader().modelV2
  assert driver.update() == (True, False)
  driver.sm['modelV2'] = messaging.new_message('modelV2').as_reader().modelV2  # the change is done
  driver.sm.plan(LaneChange.none, 0.)
  assert driver.update() == (False, False)


def test_stalk_answers_the_turn_prompt_and_holds_it_through_the_turn():
  """turnSignal.prompt ("Signal Left") is answered with that side's stalk, held until the turn is over (the wheel's
  return), though the prompt goes as soon as the lamp is lit."""
  driver = new_driver()
  driver.sm.turn(LaneChange.none, LaneChange.left)  # armed, the car's signal ready: no prompt, no stalk
  assert driver.update() == (False, False)
  driver.sm.turn(LaneChange.left, LaneChange.left)
  assert driver.update() == (True, False) and driver.turn_signals == 1
  driver.sm.turn(LaneChange.none, LaneChange.left)  # the lamp is lit: the prompt goes, the stalk stays
  assert driver.update() == (True, False)
  driver.sm.turn(LaneChange.none, LaneChange.none)  # the turn is done
  assert driver.update() == (False, False) and driver.turn_signals == 1
  driver.sm.turn(LaneChange.right, LaneChange.right)
  assert driver.update() == (False, True) and driver.turn_signals == 2
  driver.sm.alive['turnpilotPlan'] = False  # turnpilotd gone: the driver lets go
  assert driver.update() == (False, False)


# --- the driver's go at a stop (driver.GasTaps) ---

def plan_msg(prompt='none', state='following', should_stop=False, handback='none', fault='none'):
  msg = messaging.new_message('turnpilotPlan')
  p = msg.turnpilotPlan
  p.controlPrompt, p.state, p.shouldStop, p.handback, p.fault = prompt, state, should_stop, handback, fault
  return msg.as_reader().turnpilotPlan


def run_taps(taps, plan, seconds, v=0., engaged=True, green=None, t0=0., plan_age_s=0.):
  out = []
  for k in range(int(seconds / 0.05)):
    out.append(taps.update(t0 + k * 0.05, plan, engaged, v, green, plan_age_s=plan_age_s))
  return out


def test_stop_sign_go_is_tapped_once_the_car_has_stood_2_s():
  from openpilot.tools.sim.bridge.beamng.driver import TAP_GAS, GasTaps
  taps = GasTaps()
  out = run_taps(taps, plan_msg('stopSignGo', 'holding', True), 4.)
  first = out.index(TAP_GAS)
  assert abs(first * 0.05 - 2.0) < 0.06 and taps.log[0]['reason'] == 'stopSignGo'
  assert taps.log[0]['stopped_s'] >= 2.0 - 1e-6


def test_unprotected_turn_go_is_tapped_once_the_car_has_stood_2_s():
  from openpilot.tools.sim.bridge.beamng.driver import TAP_GAS, GasTaps
  taps = GasTaps()
  out = run_taps(taps, plan_msg('unprotectedTurnGo', 'holding', True), 4.)
  first = out.index(TAP_GAS)
  assert abs(first * 0.05 - 2.0) < 0.06 and taps.log[0]['reason'] == 'unprotectedTurnGo'


def test_the_2_s_count_from_when_the_car_stopped_not_from_the_prompt():
  from openpilot.tools.sim.bridge.beamng.driver import TAP_GAS, GasTaps
  # the prompt comes up while the car still rolls to its stop: no tap until it has stood 2 s
  taps = GasTaps()
  out = run_taps(taps, plan_msg('unprotectedTurnGo', 'holding', True), 1., v=1.)
  out += run_taps(taps, plan_msg('unprotectedTurnGo', 'holding', True), 4., t0=1.)
  first = out.index(TAP_GAS)
  assert abs(first * 0.05 - 3.0) < 0.06
  # the car stands 1 s in the stop sign's dwell before the prompt: the tap comes 2 s after the stop, 1 s after the prompt
  taps = GasTaps()
  out = run_taps(taps, plan_msg('none', 'holding', True), 1.)
  out += run_taps(taps, plan_msg('stopSignGo', 'holding', True), 4., t0=1.)
  first = out.index(TAP_GAS)
  assert abs(first * 0.05 - 2.0) < 0.06 and taps.log[0]['stopped_s'] == pytest.approx(2.0, abs=0.06)
  # a creep in the hold restarts the count
  taps = GasTaps()
  out = run_taps(taps, plan_msg('stopSignGo', 'holding', True), 1.5)
  out += run_taps(taps, plan_msg('stopSignGo', 'holding', True), 0.1, v=1., t0=1.5)
  out += run_taps(taps, plan_msg('stopSignGo', 'holding', True), 4., t0=1.6)
  first = out.index(TAP_GAS)
  assert abs(first * 0.05 - 3.6) < 0.06


def test_signal_go_only_on_a_ground_truth_green():
  from openpilot.tools.sim.bridge.beamng.driver import GasTaps
  for green in (False, None):  # red or yellow, or not known: never
    taps = GasTaps()
    assert max(run_taps(taps, plan_msg('signalGo', 'holding', True), 5., green=green)) == 0. and not taps.log
  taps = GasTaps()
  assert max(run_taps(taps, plan_msg('signalGo', 'holding', True), 3., green=True)) > 0. and taps.log[0]['reason'] == 'signalGo'
  taps = GasTaps(signal_tap=False)  # the stop-at-the-line path: never tapped
  assert max(run_taps(taps, plan_msg('signalGo', 'holding', True), 5., green=True)) == 0.


def test_standing_with_nothing_held_is_never_tapped_unless_asked_and_a_hold_never_is():
  # the driver taps only where TurnPilot asks for a go: a car the model stopped on its own (a queue, a hold let go with no
  # pull-away) is the run's to fail, not the driver's to push on. Only with stall_tap on (exploring by hand) does it
  from openpilot.tools.sim.bridge.beamng.driver import STALL_S, GasTaps
  taps = GasTaps()
  assert max(run_taps(taps, plan_msg('none', 'turning'), STALL_S + 10.)) == 0. and not taps.log
  taps = GasTaps(stall_tap=True)
  out = run_taps(taps, plan_msg('none', 'turning'), STALL_S + 1.)
  assert max(out) > 0. and taps.log[0]['reason'] == 'stall'
  # a pedestrian hold, or a stop sign's dwell before its prompt: the car waits, no tap
  taps = GasTaps()
  assert max(run_taps(taps, plan_msg('none', 'holding', True), 20.)) == 0.
  taps = GasTaps()
  assert max(run_taps(taps, plan_msg('none', 'turning'), 20., engaged=False)) == 0.
  taps = GasTaps()
  assert max(run_taps(taps, plan_msg('none', 'turning'), 20., v=3.)) == 0.


def test_taps_are_short_and_spaced():
  from openpilot.tools.sim.bridge.beamng.driver import TAP_S, GasTaps
  taps = GasTaps()
  out = run_taps(taps, plan_msg('stopSignGo', 'holding', True), 10.)
  pressed = sum(1 for g in out if g > 0.) * 0.05
  assert len(taps.log) >= 2 and abs(pressed - len(taps.log) * TAP_S) < 0.11


def test_never_a_tap_while_turnpilot_hands_back_is_out_or_its_plan_is_stale():
  # a prompt left over from before a handback, a fault or a stale plan is no go: the driver doesn't press on it (a tap
  # under the detector-stopped alert read as the driver's disengagement in BeamNG)
  from openpilot.tools.sim.bridge.beamng.driver import GasTaps
  for plan, age in ((plan_msg('signalGo', 'unavailable', True, handback='detectorStopped'), 0.),
                    (plan_msg('signalGo', 'holding', True, handback='detectorStopped'), 0.),
                    (plan_msg('stopSignGo', 'unavailable', True), 0.),
                    (plan_msg('stopSignGo', 'disabled', True, fault='error'), 0.),
                    (plan_msg('unprotectedTurnGo', 'holding', True), 0.6)):
    taps = GasTaps(stall_tap=True)
    assert max(run_taps(taps, plan, 6., green=True, plan_age_s=age)) == 0. and not taps.log


def test_a_tap_under_way_stops_when_turnpilot_hands_back():
  from openpilot.tools.sim.bridge.beamng.driver import STOPPED_TAP_S, TAP_GAP_S, TAP_GAS, TAP_S, GasTaps
  taps = GasTaps()
  out = run_taps(taps, plan_msg('stopSignGo', 'holding', True), STOPPED_TAP_S + 0.1)
  assert out[-1] == TAP_GAS
  out = run_taps(taps, plan_msg('stopSignGo', 'unavailable', True, handback='detectorStopped'), 2., t0=STOPPED_TAP_S + 0.1)
  assert max(out) == 0.
  # back again: the 2 s standing starts over from the handback's end, no tap the instant it clears
  t_back = STOPPED_TAP_S + 2.1
  out = run_taps(taps, plan_msg('stopSignGo', 'holding', True), 6., t0=t_back)
  first = out.index(TAP_GAS)
  assert t_back + STOPPED_TAP_S > STOPPED_TAP_S + TAP_S + TAP_GAP_S  # past the usual spacing after the cut tap
  assert abs(first * 0.05 - STOPPED_TAP_S) < 0.06
