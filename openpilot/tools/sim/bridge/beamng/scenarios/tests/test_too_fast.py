"""too_fast_right (S19): a right at a junction with no control of its own, at the end of the map's one 240 m+ straight with
no mapped control on it, at 65 km/h: the car engages already inside turn_skip's infeasible region, at its rolling start's
speed and at the set speed, so the turn is skipped (tooFastToTurn), never braked hard for, and well before the first
pulse's latest point (MIN_FIRST_PULSE_M), so the skip is the too-fast one, not tooLate."""
import os

import pytest

from openpilot.selfdrive.turnpilot.turn_skip import MIN_FIRST_PULSE_M, can_make_turn
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON
from openpilot.tools.sim.bridge.beamng.scenarios import catalog

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


def test_engaged_inside_the_infeasible_region_of_an_uncontrolled_right():
  plan = catalog.build('too_fast_right')
  e = catalog.SCENARIOS['too_fast_right']
  assert e.turn == 'right' and e.speed_kph >= 60. and plan.expect == ('skip',) and not plan.actors
  assert plan.notes['junction_control'] == 'none' and plan.notes['controls_from_spawn'] == []
  v_roll, v = catalog.ROLL_FRACTION * e.speed_kph / 3.6, e.speed_kph / 3.6
  engaged_m = plan.notes['approach_m'] - v_roll ** 2 / (2. * catalog.ROLL_ACCEL) - catalog.ENGAGE_LAG_S * v_roll
  for speed in (v_roll, v):
    assert not can_make_turn(speed, engaged_m, 90., True), (speed, engaged_m)
  assert engaged_m > MIN_FIRST_PULSE_M + 10.
  # and the spawn itself is where the catalog put it: not moved (clear of junctions and controls)
  assert plan.notes['approach_m'] == e.approach_m


def test_the_run_ends_past_the_skipped_junction():
  # BeamNG smoke 2026-10-29: run on to its 60 s timeout, the reroute's next turns (a desire, the model 168 m in the
  # oncoming lane on one of them) failed a run whose skip had come at engagement; the shot is the skip at this junction
  plan = catalog.build('too_fast_right')
  assert 20. <= plan.end_past_m <= 40.
  assert catalog.build('ped_in_path').end_past_m == 0.
