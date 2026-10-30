"""A scenario's expectations (Entry.expect, or the runner's): the handback, skip, missed turn, gas press or hard braking a
shot exists to show is required of the run, and only the generic failure it causes is explained by it, so result.json's
verdict means what the shot shows."""
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.tools.sim.bridge.beamng.scenarios import score
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_score import drive, route

SOFT = 'turnpilotDetectorStopped/softDisable'
A = {'nav_type': 'arrive', 'nav_mod': ''}


def silent_from(rows, last_det_t):
  """The rows' det_age (turnpilotDetections' age, 20 Hz) with the detector's last detections at last_det_t."""
  for r in rows:
    r['det_age'] = 0.05 if r['t'] <= last_det_t + 0.05 else round(r['t'] - last_det_t, 3)
  return rows


def detector_drop(handback=True, early_disengage=False, cruise_s=4.0, last_det_t=2.45, driver=False):
  """A straight cruise: engaged, the detector stops (its last detections at last_det_t), TurnPilot hands back
  (handback detectorStopped, openpilot's soft disable) cruise_s in, and openpilot disengages (or the driver brakes)."""
  segs = [(cruise_s, dict(A))]
  if early_disengage:
    segs += [(0.5, {'enabled': False, **A}), (1.0, dict(A))]
  if handback:
    segs += [(3.0, {'handback': 'detectorStopped', 'state': 'unavailable', 'alert': SOFT, **A})]
  segs += [(1.0, {'enabled': False, 'handback': 'detectorStopped' if handback else 'none', 'state': 'unavailable', 'brake': driver,
                  **A})]
  return silent_from(drive(segs), last_det_t)


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
    res = score.summarize('straight_cruise', detector_drop(early_disengage=True, last_det_t=4.0), 'disengaged', expect=('handback:detectorStopped',))
    assert not res['passed'] and any(f.startswith('1 disengagement(s) before the expected') for f in res['failures']), res['failures']

  def test_another_handback_before_the_expected_one_fails(self):
    rows = detector_drop()
    for r in rows:
      if 1.0 <= r['t'] < 2.0:
        r['control_prompt'] = 'carStoppedTakeControl'
    res = score.summarize('straight_cruise', rows, 'disengaged', expect=('handback:detectorStopped',))
    assert "handed back carStoppedTakeControl at t=1.0 s, before the expected handback detectorStopped" in res['failures'], res['failures']

  def test_the_disengagement_after_a_fault_handback_is_its_soft_disable(self):
    res = score.summarize('straight_cruise', detector_drop(driver=True), 'disengaged', expect=('handback:detectorStopped',))
    assert "the disengagement after the detectorStopped handback was the driver's (driver brake), not its soft disable" in \
      res['failures'], res['failures']
    rows = [r for r in detector_drop() if r['enabled']]  # the run ended before the soft disable did
    res = score.summarize('straight_cruise', rows, 'timeout', expect=('handback:detectorStopped',))
    assert "no disengagement after the detectorStopped handback: its soft disable never ended the drive" in res['failures'], res['failures']

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


class TestDetectionDrop:
  """S01's 'handback within 2.0 s of the drop (rlog: last turnpilotDetections to turnpilotPlan.handback)' (demo skeptic
  P1): the drop's handback within score.DROP_HANDBACK_S of the detector's last detections before it, which come after the
  runner's drop; without the detections' age in the rows, within that and the detector's read of the param of the drop."""
  DROP = {'t': 3.0, 's': 20.0, 'param': 'TurnPilotSimDropDetectionsS'}

  def late(self):
    """The skeptic's P1: dropped at t=3, the handback 12 s later, at t=15."""
    return detector_drop(cruise_s=15.0, last_det_t=3.0)

  def test_a_handback_12_s_after_the_drop_fails(self):
    res = score.summarize('straight_cruise', self.late(), 'disengaged', expect=('handback:detectorStopped',), drop=self.DROP)
    assert not res['passed']
    assert "handed back 12.0 s after the detector's last detections (t=3.0 s), over 2.0 s" in res['failures'], res['failures']
    assert any(f.startswith('1 disengagement(s)') for f in res['failures'])  # unmet: the soft disable isn't explained
    # the same rows without det_age (a result from before it): held to the drop's time and the detector's read of it
    for r in (old := self.late()):
      del r['det_age']
    res = score.summarize('straight_cruise', old, 'disengaged', expect=('handback:detectorStopped',), drop=self.DROP)
    want = "handed back 12.0 s after the drop at t=3.0 s, over 3.0 s (2.0 s, and up to 1.0 s for the detector to read the drop's param)"
    assert want in res['failures'], res['failures']

  def test_the_smoke_runs_timing_passes(self):
    # BeamNG smoke 2026-10-29 (s01_detector_drop): the param written at 3.04 s, the handback at 5.29 s, the soft disable 3 s on
    rows = detector_drop(cruise_s=5.3, last_det_t=3.79)
    drop = {**self.DROP, 't': 3.04}
    res = score.summarize('straight_cruise', rows, 'disengaged', expect=('handback:detectorStopped',), drop=drop)
    assert res['passed'], res['failures']
    for r in rows:
      del r['det_age']
    res = score.summarize('straight_cruise', rows, 'disengaged', expect=('handback:detectorStopped',), drop=drop)
    assert res['passed'], res['failures']

  def test_a_detector_silent_before_the_drop_isnt_the_drops(self):
    rows = detector_drop(cruise_s=2.5, last_det_t=1.0)
    res = score.summarize('straight_cruise', rows, 'disengaged', expect=('handback:detectorStopped',), drop=self.DROP)
    assert "the detector went silent at t=1.0 s, before the drop at t=3.0 s" in res['failures'], res['failures']
    for r in rows:
      del r['det_age']
    res = score.summarize('straight_cruise', rows, 'disengaged', expect=('handback:detectorStopped',), drop=self.DROP)
    assert "handed back detectorStopped at t=2.5 s, before the drop at t=3.0 s" in res['failures'], res['failures']

  def test_a_drop_asked_for_that_never_came_fails(self):
    res = score.summarize('straight_cruise', detector_drop(), 'disengaged', expect=('handback:detectorStopped',),
                          drop={'t': None, 'at': 30.0})
    assert "the detection drop asked for at t=30.0 s never came (the run ended first)" in res['failures'], res['failures']

  def test_neither_the_detections_age_nor_the_drop_is_not_checked(self):
    rows = detector_drop()
    for r in rows:
      del r['det_age']
    res = score.summarize('straight_cruise', rows, 'disengaged', expect=('handback:detectorStopped',))
    assert "no detector times in the rows and no drop: the detectorStopped handback's 2.0 s was not checked" in res['failures']

  def test_a_recorded_drives_rows_carry_the_detections_age(self):
    from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_score import msg
    out = []
    for i in range(80):  # 4 s at 20 Hz, the detector silent from 1.5 s
      t = i * 0.05
      out += [msg('selfdriveState', t, enabled=True), msg('carState', t, vEgo=8.0), msg('modelV2', t)]
      if t < 1.5:
        out.append(msg('turnpilotDetections', t))
      out.append(msg('turnpilotPlan', t + 0.01))
    rows = score.rows_from_log(out)
    assert rows[10]['det_age'] == pytest.approx(0.01, abs=1e-6) and rows[-1]['det_age'] == pytest.approx(3.95 + 0.01 - 1.45, abs=1e-6)
    assert score.rows_from_log(out[:3] + [msg('turnpilotPlan', 0.01)])[0]['det_age'] is None

  def test_the_param_read_is_the_detectors(self):
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import PARAM_CHECK_S
    from openpilot.selfdrive.turnpilot.turnpilotd import DETECTOR_DEADLINE_S
    assert score.DETECTOR_PARAM_CHECK_S == PARAM_CHECK_S
    assert DETECTOR_DEADLINE_S < score.DROP_HANDBACK_S  # the shot's 2.0 s holds turnpilotd's deadline


def left_go(after):
  """S02: waiting at the left's line, the go, withdrawn (cantStartTakeControl) with the car standing 17 m out; then the
  `after` segments, and the sim driver's brake (--end-after-handback)."""
  return drive([(3.0, {'v': 0., 'state': 'holding', 'left_state': 'waitGap', 'left_treq': 6.0, 'j_dist': 20., 'j_ahead': 20.,
                       'reason': 'Waiting for a gap', 'control_prompt': 'waitingForGap'}),
                (1.0, {'v': 1.0, 'left_state': 'go', 'control_prompt': 'leftGo', 'j_dist': 19., 'j_ahead': 19.})] + after +
               [(0.5, {'v': 0., 'enabled': False, 'brake': True, 'j_dist': 5., 'j_ahead': 5.})])


class TestHeldAfterHandback:
  """A take-control handback holds the car (hooks.TAKE_CONTROL_PROMPTS): from it, engaged, the car goes no further than a
  stop from its speed then (score.HELD_*). S02 'stopped short of the oncoming lanes and was held after', S03 'nothing
  pushes the car on', S23 off 'full stop, handback, no go' (demo skeptic P2, P3)."""
  HB = {'state': 'holding', 'left_state': 'go', 'control_prompt': 'cantStartTakeControl', 'reason': "Can't start the turn: take control"}

  def test_rolling_on_after_the_lefts_handback_fails(self):
    # the skeptic's P3: handed back standing, then 3 s rolling to 7 m/s, about 12 m into the junction, before the driver brakes
    rows = left_go([(1.0, {'v': 0., 'j_dist': 17., 'j_ahead': 17., **self.HB}),
                    (3.0, {'v': lambda tau: 1.0 + 2.0 * tau, 'left_state': 'go', 'control_prompt': 'cantStartTakeControl',
                           'j_dist': lambda tau: 17. - 4. * tau, 'j_ahead': lambda tau: 17. - 4. * tau})])
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=('handback:cantStartTakeControl',))
    assert not res['passed']
    assert any(f.startswith("moved 11.7 m") and "after the cantStartTakeControl handback (up to 6.9 m/s)" in f and "not held" in f
               for f in res['failures']), res['failures']
    assert not res['expected'][0]['met']
    assert any('cantStartTakeControl' in f and 'stalled' in f for f in res['failures'])  # nothing explained

  def test_a_go_withdrawn_rolling_stops_and_is_held(self):
    # withdrawn at the lift's 3 m/s, stopped at 1.5 m/s^2 (3 m) and held 4 s: the shot
    rows = left_go([(2.0, {'v': lambda tau: max(3.0 - 1.5 * tau, 0.), 'j_dist': lambda tau: 17. - 3. * tau + 0.75 * tau * tau,
                           'j_ahead': lambda tau: 17. - 3. * tau + 0.75 * tau * tau, **self.HB}),
                    (4.0, {'v': 0., 'j_dist': 14., 'j_ahead': 14., **self.HB})])
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=('handback:cantStartTakeControl',))
    assert res['passed'], res['failures']
    held = res['expect_facts']['held'][('cantStartTakeControl')]
    assert held['went_m'] == pytest.approx(3.0, abs=0.2) and held['v0'] == 3.0

  def test_creeping_on_slowly_after_standing_fails_too(self):
    rows = left_go([(1.0, {'v': 0., 'j_dist': 17., 'j_ahead': 17., **self.HB}),
                    (6.0, {'v': 1.0, 'j_dist': lambda tau: 17. - tau, 'j_ahead': lambda tau: 17. - tau, **self.HB})])
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=('handback:cantStartTakeControl',))
    assert any(f.startswith("moved 6.") and "not held" in f for f in res['failures']), res['failures']

  def test_the_model_pulling_away_after_the_stall_handback_fails(self):
    # S03: stood 3 s in the turn after the walker, stalledTakeControl, then the model drives on through the turn
    rows = drive([(4.0, {'v': 4., 'nav_dist': lambda tau: 30. - 4. * tau}),
                  (3.0, {'v': 0., 'nav_dist': 14., 'reason': 'pedestrian in crosswalk', 'state': 'holding'}),
                  (3.0, {'v': 0., 'nav_dist': 14., 'control_prompt': 'stalledTakeControl', 'reason': 'Stalled in the turn: take control'}),
                  (4.0, {'v': lambda tau: min(2. * tau, 6.), 'nav_dist': lambda tau: 14. - tau * tau}),
                  (0.5, {'v': 0., 'enabled': False, 'brake': True})])
    res = score.summarize('ped_crosswalk_right', rows, 'taken over', expect=('handback:stalledTakeControl',))
    assert any("after the stalledTakeControl handback" in f and "not held" in f for f in res['failures']), res['failures']

  def test_driving_on_through_the_stop_after_its_handback_fails(self):
    # the skeptic's P2 (S23, avoidance off): the full stop, stopSignTakeControl, then on through the stop sign engaged, arrived
    segs = route([(90., 'turn', 'straight'), (120., 'arrive', '')], v=8.)
    rows = drive([(5.0, {'map_kind': 'stop', 'v': 8., **segs[0][1]}),
                  (2.0, {'map_kind': 'stop', 'v': 0., 'state': 'holding', 'reason': 'Stop sign'}),
                  (2.0, {'map_kind': 'stop', 'v': 0., 'state': 'holding', 'reason': 'Stop sign', 'control_prompt': 'stopSignTakeControl'}),
                  (10.0, {'map_kind': 'none', 'v': 8., 'control_prompt': 'none'}),
                  (1.0, {'v': 0., 'state': 'arrived', 'dest_dist': 3.})])
    res = score.summarize('twoway_route_pair', rows, 'arrived', expect=('handback:stopSignTakeControl',))
    assert not res['passed']
    assert any(f.startswith("moved 80.") and "after the stopSignTakeControl handback" in f for f in res['failures']), res['failures']
    assert any('took 0 of 0 maneuvers' in f for f in res['failures'])  # an unmet expectation explains nothing
    # stood at the sign and held there until the driver took over: the shot
    held = drive([(5.0, {'map_kind': 'stop', 'v': 8., **segs[0][1]}),
                  (2.0, {'map_kind': 'stop', 'v': 0., 'state': 'holding', 'reason': 'Stop sign'}),
                  (3.0, {'map_kind': 'stop', 'v': 0., 'state': 'holding', 'reason': 'Stop sign', 'control_prompt': 'stopSignTakeControl'}),
                  (0.5, {'map_kind': 'stop', 'v': 0., 'enabled': False, 'brake': True})])
    res = score.summarize('twoway_route_pair', held, 'taken over', expect=('handback:stopSignTakeControl',))
    assert res['passed'], res['failures']

  def test_a_stop_sign_handback_without_the_full_stop_fails(self):
    # handed back rolling at 1.5 m/s and stopped after: not the 'full stop, handback'
    rows = drive([(5.0, {'map_kind': 'stop', 'v': 8.}),
                  (1.0, {'map_kind': 'stop', 'v': 1.5, 'control_prompt': 'stopSignTakeControl'}),
                  (3.0, {'map_kind': 'stop', 'v': 0., 'control_prompt': 'stopSignTakeControl'}),
                  (0.5, {'v': 0., 'enabled': False, 'brake': True})])
    res = score.summarize('twoway_route_pair', rows, 'taken over', expect=('handback:stopSignTakeControl',))
    assert "handed back stopSignTakeControl at t=5.0 s at 1.5 m/s: no full stop before it" in res['failures'], res['failures']


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

  def test_the_lefts_skip_is_the_range_skip(self):
    # the skeptic's P4 (S05): tooFastToTurn set, but the left never 'skipped': a tooLate or laneUnreachable skip, not the
    # range skip the shot is about ("Can't See Far Enough to Turn")
    rows = drive([(4.0, {'v': 8., 'left_state': 'approach', 'j_dist': 60., 'j_ahead': 60.}),
                  (6.0, {'v': 8., 'too_fast': True, 'left_state': 'approach', 'j_dist': 30., 'j_ahead': 30.}),
                  (4.0, {'v': 8., 'j_dist': 10., 'j_ahead': -10.})])
    res = score.summarize('unprotected_left_visible', rows, 'timeout', expect=('skip',))
    assert not res['passed']
    want = ("expected skip: the left never skipped for range (unprotectedLeft.state skipped); tooFastToTurn at t=4.0 s is " +
            "another skip (tooLate, laneUnreachable or tooFast)")
    assert want in res['failures'], res['failures']
    assert any('never went' in f for f in res['failures'])  # unmet: nothing explained
    # elsewhere tooFastToTurn is the skip (S19)
    assert score.summarize('too_fast_right', rows, 'timeout', expect=('skip',))['expected'][0]['met']

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
