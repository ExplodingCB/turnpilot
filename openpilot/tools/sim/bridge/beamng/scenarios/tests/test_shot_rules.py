"""The shots' pass rules the expectations hold a run to (the demo skeptic's recheck at afadad3, B1 to B9): each test is a run
that broke its shot's rule and passed, and the run that keeps it."""
from openpilot.tools.sim.bridge.beamng.scenarios import score
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_score import drive

SC = {'state': 'holding', 'control_prompt': 'stalledTakeControl', 'reason': 'Stalled in the turn: take control'}


class TestTakeControlBrakesNoHarder:
  """S03 'no TurnPilot accel below -2.0 m/s^2' (B8): a run expecting a take-control handback has no confirmed track's hard
  stop to show, so hardBrake past -2.0 fails it even in a ped_* scenario."""

  def stall_after_hard_stop(self, accel=-3.5, alert='hardBrake'):
    """ped_crosswalk_right: a hard stop for the walker, held for her 5 s, then standing 3 s with nothing holding the car,
    the stall's handback, held, and the driver's brake."""
    return drive([(3.0, {'v': 6., 'tp_accel': 0.}),
                  (1.5, {'v': lambda tau: max(6. - 4. * tau, 0.), 'tp_accel': accel, 'control_alert': alert}),
                  (5.0, {'v': 0., 'state': 'holding', 'reason': 'pedestrian in crosswalk', 'tp_accel': 0.}),
                  (3.0, {'v': 0., 'tp_accel': 0.}),
                  (3.0, {'v': 0., **SC, 'tp_accel': 0.}),
                  (0.5, {'v': 0., 'enabled': False, 'brake': True})])

  def test_a_hard_stop_fails_a_run_expecting_the_stall_handback(self):
    res = score.summarize('ped_crosswalk_right', self.stall_after_hard_stop(), 'taken over', expect=('handback:stalledTakeControl',))
    assert not res['passed']
    assert any(f.startswith("TurnPilot braked hard (-3.5 m/s^2, hardBrake) at t=3.0 s") and 'take-control handback' in f
               for f in res['failures']), res['failures']

  def test_the_same_stall_without_the_hard_stop_passes(self):
    res = score.summarize('ped_crosswalk_right', self.stall_after_hard_stop(accel=-1.8, alert='none'), 'taken over',
                          expect=('handback:stalledTakeControl',))
    assert res['passed'], res['failures']

  def test_a_hard_stop_is_still_ped_step_outs_and_ped_crosswalk_rights_own(self):
    assert score.hard_brake_ok('ped_step_out', ('hardBrake',))
    assert score.hard_brake_ok('ped_crosswalk_right', ())
    for hb in ('stalledTakeControl', 'cantStartTakeControl', 'stopSignTakeControl', 'carStoppedTakeControl'):
      assert not score.hard_brake_ok('ped_crosswalk_right', (f'handback:{hb}',))
      assert not score.hard_brake_ok('unprotected_left_visible', (f'handback:{hb}',))
    assert score.hard_brake_ok('straight_cruise', ('handback:detectorStopped', 'hardBrake'))  # a fault's handback isn't one
