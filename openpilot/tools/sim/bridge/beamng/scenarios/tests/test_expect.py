"""A scenario's expectations (Entry.expect, or the runner's): the handback, skip, missed turn, gas press or hard braking a
shot exists to show is required of the run, and only the generic failure it causes is explained by it, so result.json's
verdict means what the shot shows."""
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.tools.sim.bridge.beamng.scenarios import score
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_score import drive

SOFT = 'turnpilotDetectorStopped/softDisable'


def detector_drop(handback=True, early_disengage=False):
  """A straight cruise: engaged, the detector stops, TurnPilot hands back (handback detectorStopped, openpilot's soft
  disable), and openpilot disengages."""
  segs = [(4.0, {'nav_type': 'arrive', 'nav_mod': ''})]
  if early_disengage:
    segs += [(0.5, {'enabled': False, 'nav_type': 'arrive', 'nav_mod': ''}), (1.0, {'nav_type': 'arrive', 'nav_mod': ''})]
  if handback:
    segs += [(3.0, {'handback': 'detectorStopped', 'state': 'unavailable', 'alert': SOFT, 'nav_type': 'arrive', 'nav_mod': ''})]
  segs += [(1.0, {'enabled': False, 'handback': 'detectorStopped' if handback else 'none', 'state': 'unavailable',
                  'nav_type': 'arrive', 'nav_mod': ''})]
  return drive(segs)


class TestHandback:
  def test_without_the_expectation_the_handback_fails_the_run(self):
    res = score.summarize('straight_cruise', detector_drop(), 'disengaged')
    assert not res['passed'] and res['failures'][0].startswith('1 disengagement(s)')

  def test_the_expected_handback_passes_and_explains_its_disengagement(self):
    res = score.summarize('straight_cruise', detector_drop(), 'disengaged', expect=('handback:detectorStopped',))
    assert res['passed'], res['failures']
    (e,) = res['expected']
    assert e['expect'] == 'handback:detectorStopped' and e['met'] and e['t'] == pytest.approx(4.0)
    assert any(x.startswith('1 disengagement(s)') for x in e['explains'])
    assert 'as expected' in score.report('straight_cruise run 1', res)

  def test_an_expected_handback_that_never_comes_fails(self):
    res = score.summarize('straight_cruise', detector_drop(handback=False), 'disengaged', expect=('handback:detectorStopped',))
    assert not res['passed']
    assert "expected handback detectorStopped: none came" in res['failures']

  def test_a_disengagement_before_the_handback_still_fails(self):
    res = score.summarize('straight_cruise', detector_drop(early_disengage=True), 'disengaged', expect=('handback:detectorStopped',))
    assert not res['passed'] and any(f.startswith('1 disengagement(s) before the expected') for f in res['failures']), res['failures']

  def test_a_take_control_prompt_is_a_handback_and_explains_the_stall_and_the_drivers_takeover(self):
    # S02: the go withdrawn (cantStartTakeControl), the car held; the sim driver brakes (--end-after-handback): disengaged
    rows = drive([(3.0, {'v': 0., 'state': 'holding', 'left_state': 'waitGap', 'left_treq': 6.0, 'j_dist': 20., 'j_ahead': 20.,
                         'reason': 'Waiting for a gap', 'control_prompt': 'waitingForGap'}),
                  (1.0, {'v': 1.0, 'left_state': 'go', 'control_prompt': 'leftGo', 'j_dist': 19., 'j_ahead': 19.}),
                  (5.0, {'v': 0., 'state': 'holding', 'left_state': 'go', 'control_prompt': 'cantStartTakeControl', 'j_dist': 17.,
                         'j_ahead': 17., 'reason': "Can't start the turn: take control"}),
                  (0.5, {'v': 0., 'enabled': False, 'brake': True, 'j_dist': 17., 'j_ahead': 17.})])
    plain = score.summarize('unprotected_left_visible', rows, 'taken over')
    assert any('cantStartTakeControl' in f for f in plain['failures'])
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=('handback:cantStartTakeControl',))
    assert res['passed'], res['failures']
    explained = res['expected'][0]['explains']
    assert any('cantStartTakeControl' in x for x in explained) and any('driver brake' in x for x in explained)
    assert any('not arrived' in x for x in explained)


class TestGasDisengage:
  def press(self, gap_rows=0, prompt_after=False):
    """Engaged on a straight; the driver presses the gas and openpilot disengages gap_rows rows later."""
    segs = [(3.0, {'nav_type': 'arrive', 'nav_mod': ''})]
    if gap_rows:
      segs.append((gap_rows * 0.05, {'gas': True, 'nav_type': 'arrive', 'nav_mod': ''}))
    segs.append((0.5, {'gas': True, 'enabled': False, 'nav_type': 'arrive', 'nav_mod': ''}))
    segs.append((2.0, {'enabled': False, 'nav_type': 'arrive', 'nav_mod': '',
                       'control_prompt': 'carStoppedTakeControl' if prompt_after else 'none'}))
    return drive(segs)

  def test_a_press_that_disengages_at_once_passes_only_as_expected(self):
    plain = score.summarize('straight_cruise', self.press(), 'disengaged')
    assert not plain['passed'] and plain['disengagements'][0]['cause'] == 'driver gas'
    res = score.summarize('straight_cruise', self.press(), 'disengaged', expect=('gasDisengage',))
    assert res['passed'], res['failures']
    assert res['expected'][0]['t'] == pytest.approx(3.0)

  def test_a_press_openpilot_rode_through_and_a_slow_disengage(self):
    # engaged with the gas held: a tap (gas_taps) that the expectation explains only if the disengagement came quickly
    res = score.summarize('straight_cruise', self.press(gap_rows=2), 'disengaged', expect=('gasDisengage',))
    assert res['passed'], res['failures']
    res = score.summarize('straight_cruise', self.press(gap_rows=10), 'disengaged', expect=('gasDisengage',))
    assert not res['passed'] and any('after the gas press' in f for f in res['failures']), res['failures']

  def test_no_turnpilot_prompt_after_the_press(self):
    res = score.summarize('straight_cruise', self.press(prompt_after=True), 'disengaged', expect=('gasDisengage',))
    assert not res['passed'] and any('carStoppedTakeControl' in f for f in res['failures']), res['failures']

  def test_no_press_no_pass(self):
    rows = drive([(3.0, {'nav_type': 'arrive', 'nav_mod': ''}), (1.0, {'enabled': False, 'nav_type': 'arrive', 'nav_mod': ''})])
    res = score.summarize('straight_cruise', rows, 'disengaged', expect=('gasDisengage',))
    assert not res['passed'] and "expected gasDisengage: no disengagement on the gas" in res['failures']


class TestSkip:
  def skipped(self, desire_after=False):
    """S05: the left skipped at route time (unprotectedRange): no hold, the car straight on past the junction, a reroute."""
    return drive([(4.0, {'left_state': 'skipped', 'too_fast': True, 'j_dist': lambda tau: 40. - 8. * tau,
                         'j_ahead': lambda tau: 40. - 8. * tau, 'nav_mod': 'left'}),
                  (3.0, {'left_state': 'skipped', 'too_fast': False, 'nav_mod': 'left', 'route': 2, 'j_dist': lambda tau: 8. * tau,
                         'j_ahead': lambda tau: -8. * tau, 'desire': int(log.Desire.turnLeft) if desire_after else 0})])

  def test_the_left_checks_fail_a_skip_unless_it_is_expected(self):
    plain = score.summarize('unprotected_left_visible', self.skipped(), 'timeout')
    assert any('never went' in f for f in plain['failures'])
    res = score.summarize('unprotected_left_visible', self.skipped(), 'timeout', expect=('skip',))
    assert res['passed'], res['failures']

  def test_a_turn_desire_after_the_skip_fails(self):
    res = score.summarize('unprotected_left_visible', self.skipped(desire_after=True), 'timeout', expect=('skip',))
    assert not res['passed'] and any('turn desire after the skip' in f for f in res['failures']), res['failures']

  def test_an_expected_skip_that_never_comes_fails(self):
    rows = drive([(4.0, {'nav_mod': 'left'})])
    res = score.summarize('too_fast_right', rows, 'timeout', expect=('skip',))
    assert "expected skip: no turn skipped (tooFastToTurn)" in res['failures']


class TestTurnMissed:
  def test_the_missed_turn_is_required_and_explains_the_maneuver(self):
    # S18: stood at the forced red 3 m short of its line (12 m before the junction), then green, pulled away straight on
    rows = drive([(3.0, {'v': 0., 'state': 'holding', 'reason': 'Red light (sim oracle)', 'j_dist': 15., 'j_ahead': 15.,
                         'forced_light': 'red'}),
                  (3.0, {'turn_missed': True, 'nav_dist': lambda tau: 20. - 8. * tau, 'forced_light': 'green',
                         'j_ahead': lambda tau: 15. - 8. * tau, 'j_dist': lambda tau: abs(15. - 8. * tau)}),
                  (3.0, {'route': 2, 'nav_dist': 150., 'nav_remaining': 400., 'forced_light': 'green'})])
    plain = score.summarize('red_right_g62', rows, 'timeout', stop_line_m=12.)
    assert not plain['passed']
    res = score.summarize('red_right_g62', rows, 'timeout', expect=('turnMissed',), stop_line_m=12.)
    assert res['passed'], res['failures']
    assert res['red_light']['stood_at_red'] and res['red_light']['green_t'] == pytest.approx(3.0)
    res = score.summarize('red_right_g62', drive([(3.0, {'forced_light': 'green', 'j_ahead': 30.})]), 'arrived', expect=('turnMissed',),
                          stop_line_m=12.)
    assert "expected turnMissed: none came (no turnMissed)" in res['failures']


class TestHardBrake:
  def stop(self, gap=4.0, alert=True):
    return drive([(2.0, {'ped_gap': lambda tau: 25. - 8. * tau}),
                  (1.5, {'control_alert': 'hardBrake' if alert else 'none', 'v': lambda tau: max(8. - 5.5 * tau, 0.),
                         'ped_gap': lambda tau: max(9. - 6. * tau, gap)}),
                  (4.0, {'v': 0., 'ped_gap': gap})])

  def test_hard_braking_for_the_walker_without_contact(self):
    res = score.summarize('ped_step_out', self.stop(), 'stopped', expect=('hardBrake',))
    assert res['passed'], res['failures']

  def test_contact_fails_whatever_was_expected(self):
    res = score.summarize('ped_step_out', self.stop(gap=1.5), 'stopped', expect=('hardBrake',))
    assert any('came within 1.5 m of the pedestrian' in f for f in res['failures'])

  def test_no_hard_braking_fails_the_expectation(self):
    res = score.summarize('ped_step_out', self.stop(alert=False), 'stopped', expect=('hardBrake',))
    assert "expected hardBrake: no hardBrake alert" in res['failures']


def test_an_unknown_expectation_is_refused():
  with pytest.raises(ValueError, match="unknown expectation"):
    score.summarize('straight_cruise', drive([(1.0, {})]), 'arrived', expect=('handbak:detectorStopped',))


def test_rows_carry_the_plans_handback_alert_and_skip():
  tp = messaging.new_message('turnpilotPlan')
  tp.turnpilotPlan.handback = custom.TurnpilotPlan.Handback.detectorStopped
  tp.turnpilotPlan.controlAlert = custom.TurnpilotPlan.ControlAlert.hardBrake
  tp.turnpilotPlan.tooFastToTurn = True
  ss, cs, model = (messaging.new_message(s) for s in ('selfdriveState', 'carState', 'modelV2'))
  cs.carState.rightBlinker = True
  f = score.msg_fields(ss.selfdriveState, cs.carState, tp.as_reader().turnpilotPlan, None, model.modelV2)
  assert (f['handback'], f['control_alert'], f['too_fast'], f['lamp']) == ('detectorStopped', 'hardBrake', True, 'right')
  f = score.msg_fields(ss.selfdriveState, cs.carState, None, None, model.modelV2)
  assert (f['handback'], f['control_alert'], f['too_fast']) == ('none', 'none', False)


class TestForcedRed:
  def approach(self, stop=True, line=12.):
    """red_right: up to its line on the forced red, standing 2 m short of it if stop, green after 8 s, then the turn."""
    segs = [(4.0, {'forced_light': 'red', 'j_ahead': lambda tau: 46. - 8. * tau, 'j_dist': lambda tau: 46. - 8. * tau})]
    if stop:
      segs += [(8.0, {'forced_light': 'red', 'v': 0., 'state': 'holding', 'reason': 'Red light (sim oracle)', 'j_ahead': line + 2.,
                      'j_dist': line + 2.})]
    else:
      segs += [(3.0, {'forced_light': 'red', 'j_ahead': lambda tau: 14. - 8. * tau, 'j_dist': lambda tau: abs(14. - 8. * tau)})]
    segs += [(3.0, {'forced_light': 'green', 'j_ahead': lambda tau: line + 2. - 6. * tau, 'j_dist': lambda tau: abs(line + 2. - 6. * tau)})]
    return drive(segs)

  def test_stood_at_the_red_and_never_past_its_line(self):
    res = score.summarize('red_right', self.approach(), 'arrived', stop_line_m=12.)
    red = res['red_light']
    assert red['stood_at_red'] and red['past_line_on_red_t'] is None and red['green_t'] == pytest.approx(12.0)
    assert not any('red' in f for f in res['failures']), res['failures']

  def test_running_the_red_fails_red_right_but_is_only_reported_for_red_straight(self):
    res = score.summarize('red_right', self.approach(stop=False), 'arrived', stop_line_m=12.)
    assert "never stood at the forced red" in res['failures'] and any('past the stop line on the forced red' in f for f in res['failures'])
    res = score.summarize('red_straight', self.approach(stop=False), 'arrived', stop_line_m=12.)
    assert not any('red' in f for f in res['failures']) and res['red_light']['v_at_line'] == 8.0
