"""twoway_route_pair (S23): an origin and destination whose fastest route crosses a two-way stop (we stop, the cross road
doesn't) and whose avoided route meets no stop or yield, found offline with route_audit on the stop-sign sim tiles (the
audit is in the plan's notes). Run with TurnPilotAvoidStops at its default (the route goes around) and off (the car stops
at the sign and hands back)."""
import math
import os

import numpy as np
import pytest

from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON, RoadGraph
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, score
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_score import drive
from openpilot.tools.sim.bridge.beamng.scenarios.timing import arc_lengths

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


@pytest.fixture(scope="module")
def graph():
  return RoadGraph()


def test_the_spawn_is_on_a_stop_approach_to_a_two_way_stop_clear_of_every_control(graph):
  plan = catalog.build('twoway_route_pair', graph)
  tw = plan.notes['twoway_stop']
  toward, s_spawn, off = catalog.approach_arm(graph, plan.spawn, plan.heading, tw['xy'])
  assert abs(off) < 4. and graph.nearest_junction(*tw['xy']) == tw['node']
  along = catalog.controls_along(toward, catalog.mapped_controls(graph))
  assert not catalog.spawn_conflicts(s_spawn, along, plan.speed_kph)
  ahead = [(round(float(s - s_spawn), 1), c) for s, c in along if s > s_spawn]
  assert ahead and ahead[0][1].kind == 'stop' and ahead[0][0] > catalog.spawn_clear_m(plan.speed_kph)
  total = float(arc_lengths(toward)[-1])
  assert catalog.own_control(along, total)[1].kind == 'stop'  # the sign is the junction's own
  # a two-way stop: of its four approaches, two stop (ours and the one across) and the cross road doesn't
  j = tw['node']
  kinds = []
  for a in graph.adj[j]:
    if j in graph.out[a]:
      t = catalog.walk_straight(graph, j, a, 80.)[::-1]
      own = catalog.own_control(catalog.controls_along(t, catalog.mapped_controls(graph)), float(arc_lengths(t)[-1]))
      kinds.append(own[1].kind if own else 'none')
  assert sorted(kinds) == ['none', 'none', 'stop', 'stop']
  near = min(np.hypot(*(graph.pos[k][:2] - np.asarray(plan.spawn[:2]))) for k in graph.junctions())
  assert near > catalog.CLEAR_OF_JUNCTION_M and math.hypot(plan.dest[0] - plan.spawn[0], plan.dest[1] - plan.spawn[1]) > 200.
  audit = tw['route_audit']
  assert audit['fastest']['stops'] == [['stop', 183, 'uncertifiable']] and audit['avoided']['stops'] == []
  assert audit['avoidance'] == {'status': 'clean', 'avoided': 1, 'added_s': 19.0}


def stop_and_handback():
  return drive([(5.0, {'map_kind': 'stop', 'nav_type': 'arrive', 'nav_mod': ''}),
                (3.0, {'map_kind': 'stop', 'v': 0., 'state': 'holding', 'reason': 'Stop sign: take control to proceed',
                       'control_prompt': 'stopSignTakeControl', 'nav_type': 'arrive', 'nav_mod': ''}),
                (0.5, {'map_kind': 'stop', 'v': 0., 'enabled': False, 'brake': True, 'nav_type': 'arrive', 'nav_mod': ''})])


def test_the_avoided_route_must_not_meet_the_stop():
  res = score.summarize('twoway_route_pair', stop_and_handback(), 'taken over')
  assert any('met a stop sign' in f for f in res['failures'])
  clean = drive([(8.0, {'map_kind': 'signal', 'nav_type': 'arrive', 'nav_mod': '', 'dest_dist': lambda tau: 100. - 12. * tau})])
  assert not any('stop sign' in f for f in score.summarize('twoway_route_pair', clean, 'arrived')['failures'])


def test_with_avoidance_off_the_stop_and_its_handback_are_the_run():
  res = score.summarize('twoway_route_pair', stop_and_handback(), 'taken over', expect=('handback:stopSignTakeControl',))
  assert res['passed'], res['failures']


def test_the_finder_reads_a_two_way_stop_and_finds_the_catalogs_origin(graph):
  from openpilot.tools.sim.bridge.beamng.scenarios import twoway_pair
  assert twoway_pair.is_two_way({'a': 'stop', 'b': 'none', 'c': 'stop', 'd': 'none'})
  assert not twoway_pair.is_two_way({'a': 'stop', 'b': 'stop'})                # all-way
  assert not twoway_pair.is_two_way({'a': 'stop', 'b': 'none', 'c': 'signal'})  # a signal there
  found = [x for x in twoway_pair.candidates(graph) if x['node'] == 'DR687_1' and x['approach'] == 'DR558_7']
  e = catalog.SCENARIOS['twoway_route_pair']
  assert found and found[0]['spawn'] == list(e.spawn) and found[0]['dest'] == list(e.dest) and found[0]['heading_deg'] == e.heading_deg
