"""right_green and left_t_junction (S20): route_demo's route (Valhalla on the sim tiles, 2026-10-29) has neither a right on
green (its right, at DR405_1, is uncontrolled and 58 deg) nor a left with no oncoming road (its left, at DR314_2, has a
two-way road straight across: unprotected), so each gets an entry: ped_crosswalk_right's signal without the walker, and the
stem of a T with no road straight across. Both must be taken without TurnPilot holding the car to a stop."""
import math
import os

import pytest

from openpilot.selfdrive.turnpilot.turn_skip import MIN_FIRST_PULSE_M
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON, RoadGraph
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, score
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_score import drive, route

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


@pytest.fixture(scope='module')
def graph():
  return RoadGraph()


def test_right_green_is_the_crosswalk_junctions_right_with_nothing_on_it(graph):
  plan = catalog.build('right_green', graph)
  assert plan.notes['junction_control'] == 'signal' and not plan.actors and plan.lights is None and plan.expect == ()
  assert plan.junction == catalog.build('ped_crosswalk_right', graph).junction


def test_left_t_junction_turns_off_the_stem_of_a_t(graph):
  e = catalog.SCENARIOS['left_t_junction']
  jn = catalog.find_turn(graph, e.junction, 'left', from_heading=math.radians(e.from_heading_deg))
  assert jn.straight is None and len(graph.adj[jn.node]) == 3  # a T: no road straight across, nothing oncoming
  plan = catalog.build('left_t_junction', graph)
  assert plan.notes['junction_control'] == 'none' and not plan.actors and plan.notes['controls_from_spawn'] == []


@pytest.mark.parametrize("name", ['right_green', 'left_t_junction'])
def test_a_hold_to_a_stop_fails_a_protected_turn(name):
  mod = 'right' if name == 'right_green' else 'left'
  clean = drive(route([(150.0, 'turn', mod), (80.0, 'arrive', '')]))
  assert score.summarize(name, clean, 'arrived')['passed']
  held = drive([(2.0, {'nav_mod': mod}), (3.0, {'state': 'holding', 'reason': 'Waiting for a gap', 'v': 0., 'nav_mod': mod})]) + clean
  res = score.summarize(name, held, 'arrived')
  assert any('held the car to a stop' in f for f in res['failures']), res['failures']


def roll_drift_m(graph, name: str) -> tuple[float, float]:
  """(m the approach's centreline strays from the spawn's heading over the roll-up and 5 m on, m before the junction the
  car engages): the bridge's rolling start never steers (bridge.engage_step), so a bend in it runs the car off the road."""
  e = catalog.SCENARIOS[name]
  jn = catalog.find_turn(graph, e.junction, e.turn, from_heading=None if e.from_heading_deg is None else math.radians(e.from_heading_deg))
  s0 = catalog.build(name, graph).notes['approach_m']
  v = catalog.ROLL_FRACTION * e.speed_kph / 3.6
  roll = v ** 2 / (2. * catalog.ROLL_ACCEL) + catalog.ENGAGE_LAG_S * v
  p0, h0 = catalog.point_at(jn.approach, s0)
  t = h0 + math.pi  # point_at's heading points away from the junction
  worst, d = 0., 0.
  while d <= roll + 5.:
    p, _ = catalog.point_at(jn.approach, max(s0 - d, 0.))
    worst = max(worst, abs(-(p[0] - p0[0]) * math.sin(t) + (p[1] - p0[1]) * math.cos(t)))
    d += 1.
  return worst, s0 - roll


@pytest.mark.parametrize("name", ['right_green', 'left_t_junction'])
def test_the_rolling_start_is_on_a_straight(graph, name):
  # BeamNG smoke 2026-10-29: left_t_junction spawned 140 m up the T's stem, on an S-bend; the roll-up ran straight into the
  # roadside 12 m on and the car never engaged ('no engagement')
  drift, engaged_m = roll_drift_m(graph, name)
  assert drift < 0.5, drift
  assert engaged_m >= MIN_FIRST_PULSE_M + 10., engaged_m  # its first pulse well before turn_skip's tooLate point
