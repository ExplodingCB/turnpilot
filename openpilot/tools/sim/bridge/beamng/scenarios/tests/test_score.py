import math

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.tools.sim.bridge.beamng.scenarios import score

DT = 0.05


def row(t, **kw):
  r = {'t': t, 'x': None, 'y': None, 'heading': None, 'v': 8.0, 'enabled': True, 'alert': '', 'alert_text': '', 'steer': False, 'gas': False, 'brake': False,
       'state': 'following', 'reason': '', 'lc': 'none', 'prompt': 'none', 'lc_state': 'off', 'nav_valid': True, 'nav_type': 'turn',
       'nav_mod': 'right', 'nav_dist': 100.0, 'nav_remaining': 300.0, 'route': 1, 'dest_dist': 200.0, 'ped_gap': None, 'car_gap': None,
       'pacer_dx': None, 'blinker': '', 'control_prompt': 'none'}
  r.update(kw)
  return r


def drive(segments):
  """Rows at 20 Hz from (duration, fields) segments; a callable field gets the time into its segment."""
  rows, t = [], 0.0
  for dur, fields in segments:
    for k in range(int(round(dur / DT))):
      tau = k * DT
      rows.append(row(round(t + tau, 3), **{f: (v(tau) if callable(v) else v) for f, v in fields.items()}))
    t += dur
  return rows


def route(steps, v=10.0, route_id=1):
  """navInstruction fields for driving a route at v: steps are (length m, type, modifier), the last one an arrive."""
  total = sum(s[0] for s in steps)
  segs, done = [], 0.0
  for i, (length, mtype, mod) in enumerate(steps):
    future = total - done - length
    # navd moves on 10 m past a maneuver, so a step's distance runs down to -10
    span = length + (10.0 if i + 1 < len(steps) else -14.0) - (10.0 if i else 0.0)
    start = length - (10.0 if i else 0.0)
    segs.append((span / v, {'nav_type': mtype, 'nav_mod': mod, 'route': route_id,
                            'nav_dist': lambda tau, s=start: round(s - v * tau, 1),
                            'nav_remaining': lambda tau, s=start, f=future: round(s - v * tau + f, 1)}))
    done += length
  return segs


class TestDisengagements:
  def test_cause_is_the_last_alert(self):
    rows = drive([(2.0, {}), (1.0, {'alert': 'steerTempUnavailable/softDisabling', 'alert_text': 'TAKE CONTROL'}),
                  (1.0, {'enabled': False, 'alert': ''})])
    d = score.disengagements(rows)
    assert len(d) == 1 and d[0]['t'] == pytest.approx(3.0) and d[0]['cause'].startswith('steerTempUnavailable')

  def test_driver_override(self):
    d = score.disengagements(drive([(1.0, {}), (1.0, {'enabled': False, 'brake': True})]))
    assert [x['cause'] for x in d] == ['driver brake']

  def test_not_before_the_first_engagement(self):
    assert score.disengagements(drive([(2.0, {'enabled': False}), (2.0, {})])) == []


class TestHolds:
  def test_merges_flicker_and_sees_the_car_stop_and_go(self):
    rows = drive([(2.0, {}),
                  (3.0, {'state': 'holding', 'reason': 'pedestrian in crosswalk', 'v': lambda tau: max(8.0 - 4.0 * tau, 0.0)}),
                  (0.1, {'state': 'following', 'v': 0.0}),
                  (4.0, {'state': 'holding', 'reason': 'pedestrian in crosswalk', 'v': 0.0}),
                  (3.0, {'v': lambda tau: 2.0 * tau})])
    hs = score.holds(rows)
    assert len(hs) == 1
    h = hs[0]
    assert h['duration_s'] == pytest.approx(7.05, abs=0.06) and h['stopped'] and h['resumed']

  def test_a_hold_that_never_lets_go(self):
    hs = score.holds(drive([(1.0, {}), (5.0, {'state': 'holding', 'reason': 'waiting for oncoming', 'v': 0.0})]))
    assert hs[0]['stopped'] and not hs[0]['resumed']


class TestManeuvers:
  def test_route_taken(self):
    rows = drive(route([(150.0, 'turn', 'right'), (200.0, 'turn', 'left'), (80.0, 'arrive', '')]))
    ms = score.maneuvers(rows)
    assert [(m['modifier'], m['attempted'], m['taken']) for m in ms] == [('right', True, True), ('left', True, True)]

  def test_missed_turn_reroutes(self):
    # the car goes straight past the right turn; navd reroutes: new route id, a longer way round
    rows = drive(route([(150.0, 'turn', 'right'), (100.0, 'arrive', '')])[:1] +
                 [(2.0, {'nav_type': 'turn', 'nav_mod': 'right', 'nav_dist': lambda tau: round(-10.0 * tau, 1),
                         'nav_remaining': lambda tau: round(100.0 - 10.0 * tau, 1)})] +
                 route([(120.0, 'turn', 'right'), (90.0, 'turn', 'right'), (100.0, 'arrive', '')], route_id=2))
    ms = score.maneuvers(rows)
    assert (ms[0]['attempted'], ms[0]['taken']) == (True, False)
    assert all(m['taken'] for m in ms[1:])

  def test_instruction_before_route_still_a_reroute(self):
    # navd sends the new route's first instruction a tick before the new navRoute: the step still ends as a reroute
    rows = drive([(3.0, {'nav_dist': lambda tau: round(25.0 - 10.0 * tau, 1), 'nav_remaining': lambda tau: round(325.0 - 10.0 * tau, 1)}),
                  (0.05, {'nav_dist': 80.0, 'nav_remaining': 400.0, 'nav_mod': 'left'}),
                  (2.0, {'nav_dist': 80.0, 'nav_remaining': 400.0, 'nav_mod': 'left', 'route': 2})])
    ms = score.maneuvers(rows)
    assert (ms[0]['attempted'], ms[0]['taken']) == (True, False)

  def straight_through(self, heading=None):
    """The car drives straight on past a right turn. navd projects it past the step's end and moves on to the next step
    10 m past the junction, so the step changes like a taken turn; then the car is off the next step's road: navd sends
    no instruction, and reroutes 4 s later, well outside the reroute window around the step change."""
    h = {} if heading is None else {'heading': heading}
    return drive(route([(150.0, 'turn', 'right'), (200.0, 'turn', 'left'), (80.0, 'arrive', '')])[:1] +
                 [(1.5, {**h, 'nav_type': 'turn', 'nav_mod': 'left', 'nav_dist': lambda tau: round(200.0 - 10.0 * tau, 1),
                         'nav_remaining': lambda tau: round(280.0 - 10.0 * tau, 1)}),
                  (4.0, {**h, 'nav_valid': False, 'nav_dist': 0.0, 'nav_remaining': 0.0}),
                  (3.0, {**h, 'route': 2, 'nav_type': 'turn', 'nav_mod': 'right', 'nav_dist': lambda tau: round(150.0 - 10.0 * tau, 1),
                         'nav_remaining': lambda tau: round(420.0 - 10.0 * tau, 1)})])

  def test_straight_through_a_turn_is_not_taken(self):
    ms = score.maneuvers(self.straight_through())
    assert (ms[0]['modifier'], ms[0]['attempted'], ms[0]['taken']) == ('right', True, False)
    assert ms[0]['left_route_after']
    res = score.summarize('route_demo', self.straight_through(), 'arrived')
    assert any('maneuvers' in f for f in res['failures'])

  def test_a_turn_without_the_heading_change_is_not_taken(self):
    # navd stays on the route (the next step runs along the road the car went on), but the car never turned
    rows = drive(route([(150.0, 'turn', 'right'), (200.0, 'turn', 'left'), (80.0, 'arrive', '')]))
    for r in rows:
      r['heading'] = 0.0
    ms = score.maneuvers(rows)
    assert [m['taken'] for m in ms] == [False, False] and ms[0]['turn_deg'] == pytest.approx(0.0)

  def test_a_turn_with_its_heading_change_is_taken(self):
    rows = drive(route([(150.0, 'turn', 'right'), (200.0, 'turn', 'left'), (80.0, 'arrive', '')]))
    # 90 degrees right through the first junction (t 14-16 s), then 90 left through the second (t 34-36 s)
    for r in rows:
      t = r['t']
      r['heading'] = -math.pi / 2 * min(max((t - 14.0) / 2.0, 0.0), 1.0) + math.pi / 2 * min(max((t - 34.0) / 2.0, 0.0), 1.0)
    ms = score.maneuvers(rows)
    assert [m['taken'] for m in ms] == [True, True]
    assert ms[0]['turn_deg'] == pytest.approx(90.0, abs=1.0) and ms[1]['turn_deg'] == pytest.approx(90.0, abs=1.0)
    rows_wrong = [dict(r, heading=-r['heading']) for r in rows]  # turned the wrong way each time
    assert [m['taken'] for m in score.maneuvers(rows_wrong)] == [False, False]

  def test_not_attempted_when_disengaged(self):
    rows = drive([(3.0, {'enabled': False, 'nav_dist': lambda tau: round(40.0 - 10.0 * tau, 1),
                         'nav_remaining': lambda tau: round(340.0 - 10.0 * tau, 1)})])
    assert not score.maneuvers(rows)[0]['attempted']


class TestEnd:
  def test_arrival_disengagement_timeout(self):
    e = score.EndCheck(60.0)
    assert e.update(row(0.0)) is None
    assert e.update(row(1.0, dest_dist=14.0)) == 'arrived'
    e = score.EndCheck(60.0)
    e.update(row(0.0))
    assert e.update(row(1.0, enabled=False)) == 'disengaged'
    assert score.EndCheck(60.0).update(row(60.0)) == 'timeout'

  def test_stop(self):
    e = score.EndCheck(60.0, end_on_stop_s=4.0)
    ends = [e.update(r) for r in drive([(2.0, {'v': 0.0}), (3.0, {}), (5.0, {'v': 0.0})])]
    first = next(i for i, x in enumerate(ends) if x)
    assert ends[first] == 'stopped' and first * DT == pytest.approx(9.0, abs=DT)


class TestChecks:
  def crosswalk_run(self, hold=True):
    hold_fields = {'state': 'holding', 'reason': 'pedestrian in crosswalk'} if hold else {}
    rows = drive(route([(120.0, 'turn', 'right'), (60.0, 'arrive', '')])[:1] +
                 [(4.0, {**hold_fields, 'v': 0.0, 'ped_gap': 9.0, 'nav_dist': 9.0, 'nav_remaining': 69.0})] +
                 [(2.0, {'v': 5.0, 'nav_dist': lambda tau: round(9.0 - 10.0 * tau, 1), 'nav_remaining': lambda tau: round(69.0 - 10.0 * tau, 1)})] +
                 [(4.0, {'v': 8.0, 'nav_type': 'arrive', 'nav_mod': '', 'nav_dist': lambda tau: round(50.0 - 10.0 * tau, 1),
                         'nav_remaining': lambda tau: round(50.0 - 10.0 * tau, 1), 'dest_dist': lambda tau: 50.0 - 10.0 * tau})])
    return rows

  def test_crosswalk_pass_and_fail(self):
    res = score.summarize('ped_crosswalk_right', self.crosswalk_run(), 'arrived')
    assert res['passed'], res['failures']
    assert res['maneuvers_taken'] == res['maneuvers_attempted'] == 1 and res['min_gap_ped_m'] == 9.0
    res = score.summarize('ped_crosswalk_right', self.crosswalk_run(hold=False), 'arrived')
    assert not res['passed'] and 'never held' in res['failures'][0]

  @pytest.mark.parametrize("gap,ok", [(5.0, True), (2.0, False), (9.5, False)])
  def test_ped_in_path_gap(self, gap, ok):
    # a pause at the rolling start 80 m out is not the stop for the pedestrian
    rows = drive([(2.0, {'v': 0.0, 'ped_gap': 80.0}), (6.0, {'v': 8.0, 'ped_gap': lambda tau, g=gap: g + 64.0 - 8.0 * tau}),
                  (4.0, {'v': lambda tau: 8.0 - 2.0 * tau, 'ped_gap': lambda tau, g=gap: g + 16.0 - 8.0 * tau + tau * tau}),
                  (4.0, {'v': 0.0, 'ped_gap': gap})])
    res = score.summarize('ped_in_path', rows, 'stopped')
    assert res['stop_gap_m'] == pytest.approx(gap, abs=0.05) and res['passed'] == ok
    assert score.stop_gap(rows, 'timeout') == pytest.approx(gap, abs=0.05)  # without the end, the first stop after moving

  def test_occupied_lane_waits_for_the_pacer(self):
    blocked = drive([(4.0, {'reason': 'car in target lane', 'pacer_dx': 1.0, 'v': 12.0})])
    late = drive([(1.0, {'lc_state': 'laneChangeStarting', 'pacer_dx': -22.0, 'v': 12.0})])
    early = drive([(1.0, {'lc_state': 'laneChangeStarting', 'pacer_dx': 3.0, 'v': 12.0})])
    assert score.summarize('occupied_lane', blocked + late, 'arrived')['passed']
    res = score.summarize('occupied_lane', blocked + early, 'arrived')
    assert not res['passed'] and 'alongside' in res['failures'][0]
    assert not score.summarize('occupied_lane', drive([(1.0, {})]) + late, 'arrived')['passed']  # never blocked


class TestGasTaps:
  """No gas go anywhere (Kellen, 2026-10-28): the score records every press while engaged, and any fails the run."""
  def left_run(self, tap=False, stop=True, go=True):
    """An autonomous unprotected left: waits at its hold line, goes by itself, turns, arrives."""
    at_line = {'nav_dist': 9.0, 'nav_remaining': 69.0}
    stand = {'state': 'holding', 'reason': 'Waiting for a gap', 'control_prompt': 'waitingForGap', 'left_state': 'waitGap',
             'left_treq': 8.4, 'v': 0.0 if stop else 3.0, **at_line}
    return drive(route([(120.0, 'turn', 'left'), (60.0, 'arrive', '')])[:1] +
                 [(2.5, stand),
                  (1.0, {**stand, 'gas': tap}),
                  (3.0, stand)] +
                 ([(2.0, {'v': 5.0, 'control_prompt': 'leftGo', 'left_state': 'go', 'nav_dist': lambda tau: round(9.0 - 10.0 * tau, 1),
                          'nav_remaining': lambda tau: round(69.0 - 10.0 * tau, 1)}),
                   (4.0, {'v': 8.0, 'left_state': 'committed', 'nav_type': 'arrive', 'nav_mod': '',
                          'nav_dist': lambda tau: round(50.0 - 10.0 * tau, 1), 'nav_remaining': lambda tau: round(50.0 - 10.0 * tau, 1),
                          'dest_dist': lambda tau: 50.0 - 10.0 * tau})] if go else [(20., stand)]))

  def test_a_left_that_waits_and_goes_by_itself_passes(self):
    res = score.summarize('unprotected_left', self.left_run(), 'arrived')
    assert res['passed'], res['failures']
    assert res['gas_taps'] == [] and res['left']['gos']

  def test_any_press_is_recorded_and_fails_the_run(self):
    res = score.summarize('unprotected_left', self.left_run(tap=True), 'arrived')
    assert len(res['gas_taps']) == 1
    tap = res['gas_taps'][0]
    assert tap['prompt'] == 'waitingForGap' and tap['reason'] == 'Waiting for a gap'
    assert tap['stopped_s'] == pytest.approx(2.5, abs=0.06) and tap['t'] == pytest.approx(15.5, abs=0.06)
    assert any('no taps anywhere' in f for f in res['failures']), res['failures']
    # whatever the scenario
    rows = drive([(2.0, {}), (1.0, {'gas': True}), (2.0, {})])
    assert any('no taps anywhere' in f for f in score.check('route_demo', score.summarize('route_demo', rows, 'arrived')))

  def test_engaging_on_the_gas_is_no_tap(self):
    rows = drive([(1.0, {'enabled': False, 'gas': True}), (1.0, {})])
    assert score.summarize('route_demo', rows, 'arrived')['gas_taps'] == []

  def test_a_left_that_never_goes_fails(self):
    res = score.summarize('unprotected_left', self.left_run(go=False), 'timeout')
    assert "never went at the unprotected left" in res['failures']


class TestOncomingPresented:
  """The left scenarios require the wait only where an oncoming car came to the junction while the car was at it, before
  it: one that came after the car had turned tested nothing, and fails the run as untested."""

  def run_with(self, car_at, waited=True):
    rows = TestGasTaps().left_run()
    for i, r in enumerate(rows):
      t = i * DT
      r['j_dist'], r['j_ahead'] = (20., 20.) if t < 18. else (5., -5.)
      r['car_j_dist'] = 60. - 10. * max(t - car_at, 0.)
      if not waited and r['reason'] == 'Waiting for a gap':
        r['reason'] = 'Stopped at signal'
    return rows

  def test_a_car_presented_and_waited_for_passes(self):
    res = score.summarize('unprotected_left', self.run_with(car_at=12.), 'arrived', junction_control='signal')
    assert res['passed'], res['failures']
    assert res['oncoming_presented'] is True

  def test_a_car_presented_and_not_waited_for_fails(self):
    res = score.summarize('unprotected_left', self.run_with(car_at=12., waited=False), 'arrived', junction_control='signal')
    assert "never waited for oncoming traffic" in res['failures']

  def test_a_car_that_came_after_the_turn_fails_as_untested(self):
    res = score.summarize('unprotected_left', self.run_with(car_at=30.), 'arrived', junction_control='signal')
    assert res['oncoming_presented'] is False
    assert any("the wait went untested" in f for f in res['failures']), res['failures']


class TestUnprotectedLeft:
  """The autonomous left's gate numbers (CRITIQUE REQUIRED 5, G6): post-encroachment time, false gos, missed gaps,
  undecided only without a gap of tReq offered, no stall, no missed turn."""
  T_REQ = 8.4

  def stream(self, go_at, cars, end=40., v_car=11., stall=False, undecided_at=None, missed=False):
    """The car waits at its line (j_dist 9 m) and goes at go_at (None: never), through the zone 1.5 s later for 1.5 s; cars
    pass the junction at the times in cars at v_car."""
    rows = []
    for k in range(int(end / DT)):
      t = round(k * DT, 3)
      went = go_at is not None and t >= go_at
      j = 9. if not went else max(9. - 6. * (t - go_at), -30.)
      state = 'undecided' if undecided_at is not None and t >= undecided_at else 'go' if went and t < go_at + 4. else \
        'committed' if went else 'waitGap'
      onc = [[abs(v_car * (tc - t)), v_car * (tc - t)] for tc in cars]
      rows.append(row(t, v=0. if not went else 6., j_dist=abs(j), j_ahead=j, onc=onc, car_j_dist=min((o[0] for o in onc), default=None),
                      left_state=state, left_treq=self.T_REQ, state='holding' if not went else 'turning',
                      reason='Waiting for a gap' if not went else '', turn_missed=missed and t > (go_at or 0.) + 5.,
                      control_prompt='stalledTakeControl' if stall and went and t > go_at + 8. else 'none'))
    return rows

  def test_a_go_after_the_last_car_passes(self):
    lr = score.unprotected_left(self.stream(go_at=20., cars=[5., 10., 15.]))
    assert lr['gos'] == [20.] and not lr['false_goes'] and lr['pet_s'] >= score.PET_MIN_S

  def test_a_go_into_a_close_car_is_a_false_go_and_too_little_pet(self):
    lr = score.unprotected_left(self.stream(go_at=12., cars=[5., 15.]))
    assert lr['false_goes'] and lr['false_goes'][0]['tta_s'] < self.T_REQ
    assert lr['pet_s'] < score.PET_MIN_S
    res = score.summarize('unprotected_left', self.stream(go_at=12., cars=[5., 15.]), 'arrived')
    assert any('false go' in f for f in res['failures']) and any('post-encroachment' in f for f in res['failures'])

  def test_a_long_gap_waited_through_is_counted_missed_not_failed(self):
    # cars 5 s and 25 s through: a 20 s gap the car stood through; it went after the last
    lr = score.unprotected_left(self.stream(go_at=30., cars=[5., 25.], end=45.))
    assert lr['missed_gaps'] == 1 and any(g >= self.T_REQ for g in lr['gaps_offered_s'])
    res = score.summarize('unprotected_left', self.stream(go_at=30., cars=[5., 25.], end=45.), 'arrived')
    assert not any('gap' in f for f in res['failures'])

  def test_undecided_passes_only_without_a_gap_offered(self):
    # a dense stream (4 s apart) the whole time: undecided is no failure of the left's rule
    rows = self.stream(go_at=None, cars=[4. * k for k in range(1, 12)], end=45., undecided_at=20.)
    res = score.summarize('unprotected_left', rows, 'timeout')
    assert res['left']['undecided_t'] == 20. and not res['left']['gap_before_undecided']
    assert not any('undecided' in f or 'arrived' in f or 'maneuvers' in f for f in res['failures']), res['failures']
    # with a 15 s gap offered before it: a failure
    rows = self.stream(go_at=None, cars=[2., 17.], end=45., undecided_at=40.)
    res = score.summarize('unprotected_left', rows, 'timeout')
    assert any('undecided' in f for f in res['failures']), res['failures']

  def test_a_stall_or_a_missed_turn_fails(self):
    res = score.summarize('unprotected_left', self.stream(go_at=20., cars=[5.], stall=True), 'arrived')
    assert any('stalled' in f for f in res['failures'])
    res = score.summarize('unprotected_left', self.stream(go_at=20., cars=[5.], missed=True), 'arrived')
    assert "missed the turn" in res['failures']


class TestOncomingLane:
  """A two-way road east-west along y = 0, two 3.5 m lanes ('-+' for eastbound travel: the westbound lane on the left,
  north), with a junction at x = 0 whose box reaches 11.5 m."""
  ROAD = {'links': [[-500.0, 0.0, 0.0, 0.0, 3.5, 3.5, '-+'], [0.0, 0.0, 500.0, 0.0, 3.5, 3.5, '-+'],
                    [0.0, 0.0, 0.0, 500.0, 3.5, 3.5, '-+']],
          'junctions': [[0.0, 0.0, 11.5]]}

  def lane_run(self, y, heading=0.0, x0=-400.0, metres=100.0, road=None):
    sign = 1.0 if math.cos(heading) > 0 else -1.0
    return drive([(metres / 10.0, {'x': lambda tau: x0 + sign * 10.0 * tau, 'y': y, 'heading': heading, 'v': 10.0})])

  def test_driving_in_the_oncoming_lane_fails(self):
    res = score.summarize('lane_change', self.lane_run(1.75), 'arrived', road=self.ROAD)
    assert res['wrong_side_m'] == pytest.approx(100.0, abs=1.0)
    assert any(f.startswith("drove 100 m in the oncoming lane") for f in res['failures']), res['failures']
    for name in ('route_demo', 'ped_crosswalk_right', 'unprotected_left', 'occupied_lane', 'ped_in_path'):
      assert any('oncoming lane' in f for f in score.summarize(name, self.lane_run(1.75), 'arrived', road=self.ROAD)['failures'])

  def test_straddling_the_divider_fails(self):
    # the car's centre short of the divider by less than half its width less the road model's error: its left wheels are
    # in the oncoming lane, as with its centre over the divider. Every one fails, however far it drives so
    for y in (-0.3, 0.0, 0.5, 0.9):
      res = score.summarize('lane_change', self.lane_run(y), 'arrived', road=self.ROAD)
      assert res['wrong_side_m'] == pytest.approx(100.0, abs=1.0), y
      assert any('oncoming lane' in f for f in res['failures']), y
    # wandering in its own lane, its left side still short of the divider, passes
    assert score.summarize('lane_change', self.lane_run(-1.2), 'arrived', road=self.ROAD)['wrong_side_m'] == 0.0

  def test_its_own_lane_passes(self):
    res = score.summarize('lane_change', self.lane_run(-1.75), 'arrived', road=self.ROAD)
    assert res['wrong_side_m'] == 0.0 and not any('oncoming' in f for f in res['failures'])

  def test_travel_the_other_way_flips_the_sides(self):
    west = score.summarize('lane_change', self.lane_run(-1.75, heading=math.pi, x0=-100.0), 'arrived', road=self.ROAD)
    assert west['wrong_side_m'] == pytest.approx(100.0, abs=1.0)
    assert score.summarize('lane_change', self.lane_run(1.75, heading=math.pi, x0=-100.0), 'arrived', road=self.ROAD)['wrong_side_m'] == 0.0

  def test_inside_the_junction_box_a_turn_may_cross(self):
    rows = self.lane_run(1.75, x0=-11.0, metres=22.0)
    assert score.summarize('lane_change', rows, 'arrived', road=self.ROAD)['wrong_side_m'] == 0.0

  def test_the_box_starts_at_the_junctions_mapped_stop_line(self):
    # BeamNG smoke 2026-10-28: the car pulled away from a signal line out past the box (hold line plus the widest road)
    # and its first metres across counted as the oncoming lane. With a mapped stop line 20 m out the box starts there for
    # a car heading into the junction, and not on the way out of it
    lined = {**self.ROAD, 'junctions': [[0.0, 0.0, 11.5, 20.0]]}
    approach = self.lane_run(1.75, x0=-19.0, metres=8.0)
    assert score.summarize('lane_change', approach, 'arrived', road=self.ROAD)['wrong_side_m'] == pytest.approx(8.0, abs=1.0)
    assert score.summarize('lane_change', approach, 'arrived', road=lined)['wrong_side_m'] == 0.0
    leaving = self.lane_run(1.75, x0=11.5, metres=8.0)
    assert score.summarize('lane_change', leaving, 'arrived', road=lined)['wrong_side_m'] == pytest.approx(8.0, abs=1.0)

  def test_a_one_way_road_has_no_oncoming_lane_but_against_it_all_of_it_is(self):
    ours = {'links': [[-500.0, 0.0, 500.0, 0.0, 3.5, 3.5, '++']], 'junctions': []}
    assert score.summarize('lane_change', self.lane_run(1.75), 'arrived', road=ours)['wrong_side_m'] == 0.0
    against = {'links': [[-500.0, 0.0, 500.0, 0.0, 3.5, 3.5, '--']], 'junctions': []}
    assert score.summarize('lane_change', self.lane_run(-1.75), 'arrived', road=against)['wrong_side_m'] == pytest.approx(100.0, abs=1.0)

  def test_off_the_road_or_across_it_is_not_judged(self):
    assert score.summarize('lane_change', self.lane_run(30.0), 'arrived', road=self.ROAD)['wrong_side_m'] == 0.0
    across = drive([(5.0, {'x': 100.0, 'y': lambda tau: -2.0 + tau, 'heading': math.pi / 2, 'v': 1.0})])  # crossing at 90 degrees
    assert score.summarize('lane_change', across, 'arrived', road={'links': self.ROAD['links'][:2], 'junctions': []})['wrong_side_m'] == 0.0

  def test_positions_without_a_road_model_fail_loudly(self):
    res = score.summarize('lane_change', self.lane_run(-1.75), 'arrived')
    assert res['wrong_side_m'] is None
    assert "no road model: the oncoming lane was not checked" in res['failures']
    res = score.summarize('lane_change', self.lane_run(-1.75), 'arrived', road={'links': [], 'junctions': []})
    assert res['wrong_side_m'] is None and "no road model: the oncoming lane was not checked" in res['failures']

  def test_a_recorded_drive_without_positions_is_not_judged(self):
    res = score.summarize('route_demo', drive([(2.0, {})]), 'arrived')
    assert res['wrong_side_m'] is None and not any('road model' in f or 'oncoming' in f for f in res['failures'])


def msg(service, t, **fields):
  m = messaging.new_message(service, valid=True)  # navd marks a navInstruction without a route invalid
  m.logMonoTime = int(t * 1e9)
  body = getattr(m, service)
  for k, v in fields.items():
    setattr(body, k, v)
  return m.as_reader()


class TestFromMessages:
  def stream(self):
    """A recorded stream: route from navd, TurnPilot holding at a crosswalk, then a disengagement."""
    out = []
    for i in range(200):  # 10 s at 20 Hz
      t = i * DT
      holding = 2.0 <= t < 5.0
      out.append(msg('selfdriveState', t, enabled=t < 8.0, alertType='' if t < 7.0 else 'steerSaturated/warning'))
      out.append(msg('carState', t, vEgo=0.0 if holding else 6.0, steeringPressed=False))
      model = messaging.new_message('modelV2', valid=True)
      model.logMonoTime = int(t * 1e9)
      model.modelV2.meta.laneChangeState = log.LaneChangeState.laneChangeStarting if 6.0 <= t < 6.5 else log.LaneChangeState.off
      out.append(model.as_reader())
      if i % 20 == 0:
        out.append(msg('navInstruction', t, maneuverType='turn', maneuverModifier='right', maneuverDistance=60.0 - 6.0 * t,
                       distanceRemaining=160.0 - 6.0 * t))
      plan = msg('turnpilotPlan', t, state=custom.TurnpilotPlan.State.holding if holding else custom.TurnpilotPlan.State.approaching,
                 reason='pedestrian in crosswalk' if holding else '')
      out.append(plan)
    return out

  def test_scores_a_log(self):
    rows = score.rows_from_log(self.stream())
    assert len(rows) == 200 and rows[0]['state'] == 'approaching' and rows[50]['state'] == 'holding'
    assert rows[0]['lc_state'] == 'off'
    assert [lc['t'] for lc in score.lane_changes(rows)] == [pytest.approx(6.0)]  # the scorer's enum names match the readers'
    res = score.summarize('ped_crosswalk_right', rows, 'log ended')
    assert [(h['reason'], h['stopped'], h['resumed']) for h in res['holds']] == [('pedestrian in crosswalk', True, True)]
    assert len(res['disengagements']) == 1 and res['disengagements'][0]['cause'].startswith('steerSaturated')
    assert res['maneuvers_attempted'] == 1 and res['maneuvers_taken'] == 0  # it disengaged before navd moved on


def test_undecided_at_once_with_an_empty_road_after_the_stream_fails():
  # BeamNG unprotected_left_gap4 (2026-10-28): handed back undecided at the line the moment it latched (never in waitGap),
  # the stream passed, and the road stood empty for 170 s: an offered gap, so a failure, never a pass for want of a tReq
  rows = []
  for k in range(int(60. / DT)):
    t = round(k * DT, 3)
    onc = [[abs(11. * (tc - t)), 11. * (tc - t)] for tc in (12., 16., 20., 24.)]
    rows.append(row(t, v=0., j_dist=22., j_ahead=22., onc=onc, car_j_dist=min(o[0] for o in onc),
                    left_state='undecided' if t >= 2. else 'approach', left_treq=11.7, state='holding',
                    reason='Oncoming traffic too fast to judge: take control', control_prompt='leftUndecidedTakeControl'))
  res = score.summarize('unprotected_left_gap4', rows, 'timeout')
  assert res['left']['t_req'] == 11.7 and res['left']['gaps_offered_s']
  assert any('undecided' in f for f in res['failures']), res['failures']
  assert not res['passed']
  # with no tReq known at all, undecided is a failure too
  for r in rows:
    r['left_treq'] = 0.
  res = score.summarize('unprotected_left_gap4', rows, 'timeout')
  assert any('undecided' in f for f in res['failures']), res['failures']
