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


HB = {'state': 'holding', 'left_state': 'go', 'control_prompt': 'cantStartTakeControl', 'reason': "Can't start the turn: take control"}


def withdrawn_go(v0, decel, j0, go_s=2.0, extra=None):
  """S02: standing at the left's line j0 + 3 m from the junction, the go (left_state go, the lift to v0 over go_s), withdrawn
  at v0 with the junction j0 ahead (cantStartTakeControl), stopped at decel, held 3 s, and the driver's brake. Returns the
  rows and where the car stopped (j_dist)."""
  j_line = j0 + 0.5 * v0 * go_s
  end = j0 - v0 ** 2 / (2 * decel)
  segs = [(3.0, {'v': 0., 'state': 'holding', 'left_state': 'waitGap', 'left_treq': 6.0, 'j_dist': j_line, 'j_ahead': j_line,
                 'reason': 'Waiting for a gap', 'control_prompt': 'waitingForGap'}),
          (go_s, {'v': lambda tau: v0 * tau / go_s, 'left_state': 'go', 'control_prompt': 'leftGo', **(extra or {}),
                  'j_dist': lambda tau: j_line - 0.5 * v0 * tau * tau / go_s, 'j_ahead': lambda tau: j_line - 0.5 * v0 * tau * tau / go_s}),
          (v0 / decel, {'v': lambda tau: max(v0 - decel * tau, 0.), **HB,
                        'j_dist': lambda tau: j0 - v0 * tau + 0.5 * decel * tau * tau, 'j_ahead': lambda tau: j0 - v0 * tau + 0.5 * decel * tau * tau}),
          (3.0, {'v': 0., **HB, 'j_dist': end, 'j_ahead': end}),
          (0.5, {'v': 0., 'enabled': False, 'brake': True, 'j_dist': end, 'j_ahead': end})]
  return drive(segs), end


class TestWithdrawnGoShortOfTheOncomingLanes:
  """S02 'the car stopped short of the oncoming lanes and was held after' (B1): from the withdrawn go's handback
  (cantStartTakeControl) on a left, engaged, the car stays outside the junction's conflict zone (score.CONFLICT_M), and goes
  no further than a stop at turnpilotd's STOP_DECEL (score.HELD_DECEL) from its speed then, nor than the zone allows."""
  EXPECT = ('handback:cantStartTakeControl',)

  def test_the_hold_is_turnpilots_stop(self):
    from openpilot.selfdrive.turnpilot.scene_policy import STOP_DECEL
    assert score.HELD_DECEL == STOP_DECEL

  def test_a_slow_stop_into_the_conflict_zone_fails(self):
    # the skeptic's B1: withdrawn at the lift's 3 m/s 14 m out, stopping at 0.7 m/s^2: 6.4 m, ending 7.6 m out
    rows, end = withdrawn_go(3.0, 0.7, 14.)
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=self.EXPECT)
    assert not res['passed'] and not res['expected'][0]['met']
    held = res['expect_facts']['held']['cantStartTakeControl']
    assert held['allowed_m'] == 5.5 and held['zone_m'] == 6.0 and held['min_j_dist'] < score.CONFLICT_M
    assert any(f.startswith("moved 6.4 m engaged after the cantStartTakeControl handback") and "past the 5.5 m a stop from 3.0 m/s allows" in f
               for f in res['failures']), res['failures']
    assert any(f.startswith("came within 7.6 m of the junction engaged after the cantStartTakeControl handback") and "conflict zone" in f
               for f in res['failures']), res['failures']

  def test_a_stop_at_turnpilots_decel_that_ends_in_the_zone_fails(self):
    # the shot's own edge: withdrawn under 2.0 m/s with 1.5 m left to the zone, stopping at 1.0 m/s^2 (1.8 m), ending 7.7 m out
    rows, end = withdrawn_go(1.9, 1.0, 9.5)
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=self.EXPECT)
    assert not res['passed']
    assert any("past the 1.5 m to the junction's 8 m conflict zone" in f for f in res['failures']), res['failures']

  def test_handed_back_inside_the_zone_fails(self):
    rows, end = withdrawn_go(1.0, 1.5, 7.0)
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=self.EXPECT)
    assert any(f.startswith("handed back cantStartTakeControl 7.0 m from the junction, inside its 8 m conflict zone") for f in res['failures']), \
      res['failures']

  def test_withdrawn_and_held_short_of_the_zone_passes(self):
    # withdrawn at 3 m/s 14 m out, stopped at 1.5 m/s^2 (3 m): 11 m out, held
    rows, end = withdrawn_go(3.0, 1.5, 14.)
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=self.EXPECT)
    assert res['passed'], res['failures']

  def test_a_left_without_the_junctions_distance_isnt_checked(self):
    rows, end = withdrawn_go(3.0, 1.5, 14.)
    for r in rows:
      r['j_dist'] = r['j_ahead'] = None
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=self.EXPECT)
    assert "no junction distance in the rows at the cantStartTakeControl handback: 'short of the oncoming lanes' was not checked" in \
      res['failures'], res['failures']

  def test_only_the_withdrawn_go_is_held_to_the_zone(self):
    # a stall in the left's turn is handed back inside the junction, never held there: the zone is no part of it
    rows = drive([(4.0, {'v': 3., 'left_state': 'committed', 'j_dist': lambda tau: 15. - 3. * tau, 'j_ahead': lambda tau: 15. - 3. * tau}),
                  (3.0, {'v': 0., 'left_state': 'committed', 'j_dist': 3., 'j_ahead': 3.}),
                  (3.0, {'v': 0., 'left_state': 'committed', **SC, 'state': 'following', 'j_dist': 3., 'j_ahead': 3.}),
                  (0.5, {'v': 0., 'enabled': False, 'brake': True, 'j_dist': 3., 'j_ahead': 3.})])
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=('handback:stalledTakeControl',))
    assert not any('conflict zone' in f for f in res['failures']), res['failures']


class TestWithdrawnGoTimed:
  """S02 'handback within 3.5 s of the go (1.5 s with the lift off)' (B2b): the cantStartTakeControl handback comes within
  score.LIFT_HANDBACK_S of the left's go (left_state into go) when plannerd's lift was certified for it (the rows' go_cert,
  turnpilotPlan.goCertified.cause unprotectedLeft), within NO_LIFT_HANDBACK_S when it wasn't, and within the former in rows
  from before go_cert."""
  EXPECT = ('handback:cantStartTakeControl',)

  def test_the_bounds_hold_turnpilots_timers(self):
    from openpilot.selfdrive.turnpilot.go_override import GO_WINDOW_S
    from openpilot.selfdrive.turnpilot.turnpilotd import LEFT_NO_MOVE_S
    assert (score.LIFT_HANDBACK_S, score.NO_LIFT_HANDBACK_S) == (3.5, 1.5)
    assert GO_WINDOW_S < score.LIFT_HANDBACK_S and LEFT_NO_MOVE_S < score.NO_LIFT_HANDBACK_S

  def test_a_handback_12_s_after_the_go_fails(self):
    # the skeptic's B2b: the go, the car standing 12 s, then cantStartTakeControl, held, the driver's brake
    rows = drive([(3.0, {'v': 0., 'state': 'holding', 'left_state': 'waitGap', 'left_treq': 6.0, 'j_dist': 17., 'j_ahead': 17.,
                         'reason': 'Waiting for a gap', 'control_prompt': 'waitingForGap'}),
                  (12.0, {'v': 0., 'left_state': 'go', 'control_prompt': 'leftGo', 'j_dist': 17., 'j_ahead': 17.}),
                  (3.0, {'v': 0., 'j_dist': 17., 'j_ahead': 17., **HB}),
                  (0.5, {'v': 0., 'enabled': False, 'brake': True, 'j_dist': 17., 'j_ahead': 17.})])
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=self.EXPECT)
    assert not res['passed'] and not res['expected'][0]['met']
    assert "handed back cantStartTakeControl 12.0 s after the left's go at t=3.0 s, over 3.5 s (the lift not in the rows)" in \
      res['failures'], res['failures']
    assert any('never went' in f or 'stalled' in f for f in res['failures'])  # unmet: nothing explained

  def test_the_lifts_go_withdrawn_at_its_window_passes(self):
    rows, _ = withdrawn_go(3.0, 1.5, 14., go_s=3.0, extra={'go_cert': 'unprotectedLeft'})
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=self.EXPECT)
    assert res['passed'], res['failures']
    assert res['expect_facts']['go_before']['cantStartTakeControl'] == {'go_t': 3.0, 'after_s': 3.0, 'lift': True, 'bound_s': 3.5}

  def test_without_the_lift_the_handback_comes_within_1_5_s(self):
    rows, _ = withdrawn_go(0.2, 1.5, 14., go_s=2.5, extra={'go_cert': 'none'})
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=self.EXPECT)
    assert "handed back cantStartTakeControl 2.5 s after the left's go at t=3.0 s, over 1.5 s (without the lift: no goCertified)" in \
      res['failures'], res['failures']
    rows, _ = withdrawn_go(0.2, 1.5, 14., go_s=1.2, extra={'go_cert': 'none'})
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=self.EXPECT)
    assert res['passed'], res['failures']

  def test_a_handback_with_no_go_before_it_fails(self):
    rows, _ = withdrawn_go(0.2, 1.5, 14., go_s=1.0)
    for r in rows:
      if r.get('left_state') == 'go':
        r['left_state'] = 'waitGap'
    res = score.summarize('unprotected_left_visible', rows, 'taken over', expect=self.EXPECT)
    assert "no go of the left (left_state go) before the cantStartTakeControl handback" in res['failures'], res['failures']

  def test_rows_carry_the_certified_gos_cause(self):
    import openpilot.cereal.messaging as messaging
    from openpilot.cereal import custom
    tp = messaging.new_message('turnpilotPlan')
    tp.turnpilotPlan.goCertified.cause = custom.TurnpilotPlan.GoCertified.Cause.unprotectedLeft
    ss, cs, model = (messaging.new_message(s) for s in ('selfdriveState', 'carState', 'modelV2'))
    assert score.msg_fields(ss.selfdriveState, cs.carState, tp.as_reader().turnpilotPlan, None, model.modelV2)['go_cert'] == 'unprotectedLeft'
    assert score.msg_fields(ss.selfdriveState, cs.carState, None, None, model.modelV2)['go_cert'] == 'none'


def walker_stall(unheld_s, stalled_s=3.0):
  """S03 (ped_crosswalk_right): into the turn, held for the walker 5 s standing, then standing unheld_s with nothing
  holding the car, the stall's handback (stalledTakeControl) held stalled_s, and the driver's brake."""
  return drive([(4.0, {'v': lambda tau: max(6. - 1.5 * tau, 0.)}),
                (5.0, {'v': 0., 'state': 'holding', 'reason': 'pedestrian in crosswalk'})] +
               ([(unheld_s, {'v': 0.})] if unheld_s else []) +
               [(stalled_s, {'v': 0., **SC, 'state': 'following'}),
                (0.5, {'v': 0., 'enabled': False, 'brake': True})])


class TestStallTimed:
  """S03 'handback 3 s after the stand' (B2c): the stall's handback (stalledTakeControl) comes STALL_PROMPT_S (turnpilotd's)
  into the car's stand with nothing holding it: from the later of the stand's start and the end of TurnPilot's hold in
  it, the clock turnpilotd's stall counts on, to the handback, within a row early and STALL_LATE_S late."""
  EXPECT = ('handback:stalledTakeControl',)

  def test_the_prompt_is_turnpilots(self):
    from openpilot.selfdrive.turnpilot.turnpilotd import STALL_PROMPT_S
    assert score.STALL_PROMPT_S == STALL_PROMPT_S

  def test_a_stall_handback_12_s_after_the_stand_fails(self):
    # the skeptic's B2c: held for the walker, released, standing 12 s, then stalledTakeControl
    res = score.summarize('ped_crosswalk_right', walker_stall(12.0), 'taken over', expect=self.EXPECT)
    assert not res['passed'] and not res['expected'][0]['met']
    assert "handed back stalledTakeControl 12.0 s into the car's stand with nothing holding it (from t=9.0 s), not 3.0 s" in \
      res['failures'], res['failures']
    assert "did not move on after the crosswalk cleared" in res['failures']  # unmet: the stall isn't explained

  def test_three_s_after_the_walkers_hold_lets_go_passes(self):
    res = score.summarize('ped_crosswalk_right', walker_stall(3.0), 'taken over', expect=self.EXPECT)
    assert res['passed'], res['failures']
    assert res['expect_facts']['stand_before']['stalledTakeControl'] == {'from_t': 9.0, 'stood_s': 3.0}
    assert score.summarize('ped_crosswalk_right', walker_stall(3.25), 'taken over', expect=self.EXPECT)['passed']

  def test_a_stall_handback_with_no_unheld_stand_before_it_fails(self):
    for unheld_s in (0., 2.0, 3.5):
      res = score.summarize('ped_crosswalk_right', walker_stall(unheld_s), 'taken over', expect=self.EXPECT)
      assert any(f.startswith(f"handed back stalledTakeControl {unheld_s:.1f} s into the car's stand") for f in res['failures']), \
        (unheld_s, res['failures'])


class TestStopSignFullStop:
  """S23 with avoidance off, 'full stop, handback, no go' (B3): the stop sign's handback (stopSignTakeControl) comes with the
  car standing at its own first row, never on a roll up after a stand some time back."""
  EXPECT = ('handback:stopSignTakeControl',)

  def rows(self, v_at_handback):
    SS = {'map_kind': 'stop', 'state': 'holding', 'reason': 'Stop sign'}
    return drive([(5.0, {'map_kind': 'stop', 'v': 8.}),
                  (2.0, {**SS, 'v': 0.}),
                  (8.0, {**SS, 'v': 1.5}),
                  (1.0, {**SS, 'v': lambda tau: max(v_at_handback - 1.5 * tau, 0.), 'control_prompt': 'stopSignTakeControl'}),
                  (3.0, {**SS, 'v': 0., 'control_prompt': 'stopSignTakeControl'}),
                  (0.5, {'v': 0., 'enabled': False, 'brake': True})])

  def test_a_handback_rolling_after_an_old_stand_fails(self):
    # the skeptic's B3: stood 12 m short, rolled 8 s at 1.5 m/s, handed back still rolling at 1.5 m/s
    res = score.summarize('twoway_route_pair', self.rows(1.5), 'taken over', expect=self.EXPECT)
    assert not res['passed'] and not res['expected'][0]['met']
    assert "handed back stopSignTakeControl at t=15.0 s at 1.5 m/s: no full stop before it" in res['failures'], res['failures']
    assert res['expect_facts']['stood_at'] == {'stopSignTakeControl': False}

  def test_standing_at_the_handback_is_the_full_stop(self):
    res = score.summarize('twoway_route_pair', self.rows(0.), 'taken over', expect=self.EXPECT)
    assert res['expect_facts']['stood_at'] == {'stopSignTakeControl': True}
    assert not any('no full stop' in f for f in res['failures']), res['failures']
