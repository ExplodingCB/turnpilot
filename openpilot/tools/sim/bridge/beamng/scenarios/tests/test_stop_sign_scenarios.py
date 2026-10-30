"""The stop-sign logic gate's scenarios (STOP_SIGNS.md v3 8.2, SG4c): the six of section 8.2 on the sim's mapped stop
instances (catalog.py), their cross cars' triggers (run.py), their score (score.py: goes, withdrawn goes, retries, handbacks
with their reason, the minimum post-encroachment time, any TurnPilot accel below -2.0) and the sim's cross-traffic oracle
(light_oracle.cross_traffic). Logic checks only, never range evidence."""
import json
import math

import numpy as np
import pytest

from openpilot.tools.sim.bridge.beamng.scenarios import catalog, score

STOP_SCENARIOS = ('stop_allway_empty', 'stop_allway_cross_first', 'stop_allway_queue', 'stop_allway_out_of_turn', 'stop_allway_shield',
                  'stop_two_way')


@pytest.fixture(scope='module')
def plans():
  return {name: catalog.build(name) for name in STOP_SCENARIOS}


def test_the_six_scenarios_are_on_the_sims_mapped_stops(plans):
  stops = [c for c in catalog.mapped_controls(catalog.RoadGraph()) if c.kind == 'stop']
  for name, p in plans.items():
    assert p.notes['junction_control'] == 'stop', name  # the car's own approach stops
    assert p.notes['stop_scenario'] == name and p.junction is not None
    near = [c for c in stops if math.hypot(c.x - p.junction[0], c.y - p.junction[1]) < 40.]
    json.dumps([a.params for a in p.actors])  # run.save writes them with the result
    if name == 'stop_two_way':
      assert p.notes['all_way'] is False
    else:
      assert p.notes['all_way'] is True and len(near) >= 3, name


def cross(p):
  return [a for a in p.actors if a.trigger == 'cross']


def to_line(a, p):
  """m from the car's last point to the junction (its path ends at its line, or goes on through)."""
  end = np.array(a.path[-1][:2])
  return float(np.hypot(*(end - np.array(p.junction))))


def test_each_scenarios_cross_cars_start_and_go_as_section_8_2_says(plans):
  assert cross(plans['stop_allway_empty']) == [] and cross(plans['stop_two_way']) == []
  # 2: a cross car standing at its line before ours arrives, released 3 s after our stand, across and away
  a, = cross(plans['stop_allway_cross_first'])
  assert a.params['start'] == 'line' and a.params['release'] == pytest.approx(3.) and a.params['through']
  assert to_line(a, plans['stop_allway_cross_first']) > 20.
  # 3: a lead in our lane standing at our line, released 3 s after we stand behind it
  a, = cross(plans['stop_allway_queue'])
  assert a.params['side'] == 'lead' and a.params['start'] == 'line' and a.params['through']
  # 4: a cross car that arrived after us standing at its line goes on our go (out of turn), across
  a, = cross(plans['stop_allway_out_of_turn'])
  assert a.params['release'] == 'go' and a.params['through'] and a.params['side'] == 'left'
  # 5: a cross car arrives after us at the left line and waits there (its path ends at its line), sent up to it as the car
  # nears its own line so it stands there before the car's go (else the go never meets the shield)
  a, = cross(plans['stop_allway_shield'])
  assert a.params['side'] == 'left' and not a.params['through'] and a.params['start'] > 15.
  assert to_line(a, plans['stop_allway_shield']) < 20. and list(a.params['arrive']) == ['near', pytest.approx(30.)]


def test_the_cross_trigger_releases_on_its_delay_after_the_stand_or_on_our_go():
  from openpilot.tools.sim.bridge.beamng.scenarios.run import cross_due
  assert not cross_due({'release': 3.}, held_t=None, went=False, t=10.)
  assert not cross_due({'release': 3.}, held_t=8., went=False, t=10.) and cross_due({'release': 3.}, held_t=7., went=False, t=10.)
  assert not cross_due({'release': 'go'}, held_t=1., went=False, t=10.) and cross_due({'release': 'go'}, held_t=1., went=True, t=10.)
  assert not cross_due({'release': None}, held_t=1., went=True, t=100.)
  # ('near', m): once the car is within m of the junction, before any stand
  assert not cross_due({'arrive': ('near', 30.)}, held_t=None, went=False, t=1., key='arrive', j_dist=31.)
  assert cross_due({'arrive': ['near', 30.]}, held_t=None, went=False, t=1., key='arrive', j_dist=29.)


def row(t, **kw):
  base = {'t': t, 'v': 0., 'j_dist': 12., 'control_prompt': 'none', 'reason': '', 'go_cause': 'none', 'accel_cap': 10.,
          'sg_withdrawn': '', 'sg_retries': 0, 'sg_state': 'observe', 'cross': None, 'gas': False}
  base.update(kw)
  return base


def test_the_stop_sign_score_counts_goes_withdrawals_retries_handbacks_pet_and_hard_accel():
  rows = [row(k * 0.05) for k in range(40)]
  rows += [row(2. + k * 0.05, go_cause='stopSign' if k < 3 else 'none', sg_state='going', v=0.5 * k * 0.05) for k in range(20)]
  rows += [row(3. + k * 0.05, sg_withdrawn='new:7' if k > 2 else '', sg_retries=1 if k > 2 else 0, accel_cap=-1.9) for k in range(20)]
  rows += [row(4. + k * 0.05, control_prompt='stopSignTakeControl' if k > 5 else 'none', reason='Stop sign: view blocked: take control')
           for k in range(20)]
  res = score.stop_sign(rows)
  assert res['goes'] == 1 and res['withdrawn'] == 1 and res['retries'] == 1
  assert res['handbacks'] == [{'t': 4.3, 'reason': 'Stop sign: view blocked: take control'}]
  assert res['min_accel_cap'] == pytest.approx(-1.9) and res['below_2'] == 0
  # the ego in the conflict zone 5-7 s, a cross car 9-10 s: 2 s apart
  rows = [row(k * 0.1, j_dist=4. if 5. <= k * 0.1 < 7. else 20., cross=[[4. if 9. <= k * 0.1 < 10. else 30., 0.]]) for k in range(150)]
  assert score.stop_sign(rows)['min_pet_s'] == pytest.approx(2., abs=0.11)
  rows = [row(k * 0.05, accel_cap=-2.5 if k == 3 else 10.) for k in range(10)]
  assert score.stop_sign(rows)['below_2'] == 1


def test_the_shield_is_untested_unless_its_car_stood_at_its_line_before_the_go():
  # the shield car's rows carry its at-line distance (run.py: line_m + CAR_HALF_M): at 12.3 m from 2 s on, our go at 4 s
  at = [row(k * 0.1, cross=[[12.3 if k * 0.1 >= 2. else 30., 1., 12.3]], go_cause='stopSign' if 4. <= k * 0.1 < 4.3 else 'none')
        for k in range(80)]
  res = score.stop_sign(at)
  assert res['present_t'] == pytest.approx(2.) and res['first_go_t'] == pytest.approx(4.)
  good = dict(res, handbacks=[], below_2=0, min_pet_s=None)
  assert score.check_stop('stop_allway_shield', good) == []
  late = [dict(r, cross=[[12.3 if r['t'] >= 6. else 30., 1., 12.3]]) for r in at]  # it came after the go: untested
  assert score.check_stop('stop_allway_shield', dict(score.stop_sign(late), handbacks=[], below_2=0))


def test_check_fails_any_go_at_the_two_way_any_hard_accel_and_a_pet_under_2_s():
  ok = {'goes': 0, 'withdrawn': 0, 'retries': 0, 'handbacks': [{'t': 3., 'reason': 'Stop sign: this stop needs you: take control'}],
        'min_pet_s': None, 'below_2': 0, 'min_accel_cap': 10., 'gas': 0}
  assert score.check_stop('stop_two_way', ok) == []
  assert score.check_stop('stop_two_way', dict(ok, goes=1))
  good = dict(ok, goes=1, handbacks=[], min_pet_s=3.)
  assert score.check_stop('stop_allway_cross_first', good) == []
  assert score.check_stop('stop_allway_cross_first', dict(good, min_pet_s=1.5))
  assert score.check_stop('stop_allway_empty', dict(good, below_2=1))
  assert score.check_stop('stop_allway_out_of_turn', dict(good, withdrawn=1)) == []
  assert score.check_stop('stop_allway_out_of_turn', good)  # the out-of-turn car must make us stop short
  assert score.check_stop('stop_allway_shield', dict(good, handbacks=ok['handbacks']))


def test_the_cross_traffic_oracle_puts_the_sims_road_users_in_the_cars_frame_within_its_range():
  from openpilot.tools.sim.bridge.beamng.light_oracle import cross_traffic
  others = [(1, (0., 20.), (0., -5.), 'car'), (2, (10., 0.), (0., 0.), 'pedestrian'), (3, (100., 0.), (0., 0.), 'car')]
  east = cross_traffic((0., 0.), 0., others, 60.)  # heading east: north is left
  assert [c[0] for c in east] == [1, 2]
  assert east[0][1:5] == pytest.approx((0., 20., 0., -5.)) and east[1][1:3] == pytest.approx((10., 0.)) and east[1][5] == 'pedestrian'
  north = cross_traffic((0., 0.), math.pi / 2., others, 60.)  # heading north: east is right
  assert north[1][1:3] == pytest.approx((0., -10.), abs=1e-9) and north[0][1:5] == pytest.approx((20., 0., -5., 0.), abs=1e-9)
