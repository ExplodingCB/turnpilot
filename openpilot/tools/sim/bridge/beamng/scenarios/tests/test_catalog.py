import json
import math
import os

import numpy as np
import pytest

from openpilot.selfdrive.turnpilot import scene_policy
from openpilot.selfdrive.turnpilot.route_follower import RIGHT
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON, RoadGraph, wrap
from openpilot.tools.sim.bridge.beamng.scenarios import catalog
from openpilot.tools.sim.bridge.beamng.scenarios.timing import arc_lengths

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")
CROSSWALK_JUNCTIONS = [(-768.6, 555.2), (-759.2, 376.7), (-650.1, 481.7), (-600.7, 220.9), (-445.3, 147.0), (-32.5, 403.8)]


@pytest.fixture(scope="module")
def graph():
  return RoadGraph()


def car_frame(p, ego, heading):
  """(x forward, y left) of world point p for a car at ego facing heading."""
  d = np.asarray(p[:2]) - np.asarray(ego[:2])
  return float(d[0] * math.cos(heading) + d[1] * math.sin(heading)), float(-d[0] * math.sin(heading) + d[1] * math.cos(heading))


def test_every_scenario_builds(graph):
  for name in catalog.SCENARIOS:
    plan = catalog.build(name, graph)
    assert plan.dest is not None, name
    near = min(math.hypot(*(graph.pos[j][:2] - np.asarray(plan.spawn[:2]))) for j in graph.junctions())
    assert near > 20.0, f"{name} spawns {near:.0f} m from a junction"
    if plan.junction is not None:  # the car starts on its approach, facing the junction (along it: approaches bend)
      toward, s, off = catalog.approach_arm(graph, plan.spawn, plan.heading, plan.junction)
      _, _, h = catalog.project(toward, np.asarray(plan.spawn))
      assert abs(math.degrees(wrap(h - plan.heading))) < 10.0 and abs(off) < 8.0, name


def test_crosswalk_junctions(graph):
  for xy in CROSSWALK_JUNCTIONS[:4]:
    for direction in ('left', 'right'):
      jn = catalog.find_turn(graph, xy, direction)
      assert np.hypot(*(jn.center[:2] - np.asarray(xy))) < 5.0
      assert jn.straight is not None
  for xy in CROSSWALK_JUNCTIONS[4:]:  # no 90-degree turn in the road graph there
    with pytest.raises(ValueError):
      catalog.find_turn(graph, xy, 'right')


def test_walker_crosses_the_policy_crosswalk_zone(graph):
  plan = catalog.build('ped_crosswalk_right', graph)
  ped = plan.actors[0]
  path = np.array(ped.path)
  # the car waiting at the hold line, in its lane, looking down the approach
  ego = np.asarray(plan.junction) - (scene_policy.HOLD_LINE_M + 4.0) * np.array([math.cos(plan.heading), math.sin(plan.heading)])
  ego += 1.8 * np.array([math.sin(plan.heading), -math.cos(plan.heading)])
  dist = math.hypot(*(np.asarray(plan.junction) - ego))

  def in_zone(p):
    x, y = car_frame(p, ego, plan.heading)
    return dist - scene_policy.CROSSWALK_X_BEFORE_M < x < dist + scene_policy.CROSSWALK_X_AFTER_M and \
      scene_policy.CROSSWALK_Y_NEAR_M < y * RIGHT < scene_policy.CROSSWALK_Y_FAR_M

  walk = [path[0] + f * (path[1] - path[0]) for f in np.linspace(0.15, 0.85, 8)]
  assert all(in_zone(p) for p in walk), "the crossing is outside scene_policy's crosswalk zone"
  assert not in_zone(path[-1]), "the walker never leaves the zone, so the car would hold forever"
  assert car_frame(path[0], ego, plan.heading)[0] < car_frame(path[1], ego, plan.heading)[0]  # starts on the near side


def test_oncoming_cars(graph):
  plan = catalog.build('unprotected_left', graph)
  cars = [a for a in plan.actors if a.trigger == 'oncoming']
  assert len(cars) == 2
  ego_lane = car_frame(plan.spawn, plan.junction, plan.heading)[1]  # left of the junction centre, along the car's heading
  for k, car in enumerate(cars):
    path = np.array(car.path)
    start = math.hypot(*(path[0, :2] - np.asarray(plan.junction)))
    assert start == pytest.approx(35.0 + 25.0 * k, abs=3.0)
    assert abs(math.degrees(wrap(car.heading - plan.heading - math.pi))) < 25.0  # coming the other way
    lane = car_frame(path[0], plan.junction, plan.heading)[1]
    assert lane * ego_lane < 0 and abs(lane) > 1.0  # on the other side of the road
    assert arc_lengths(path)[-1] > start + 40.0  # through the junction and away


def test_standing_pedestrian_in_the_lane(graph):
  plan = catalog.build('ped_in_path', graph)
  ped = np.array(plan.actors[0].path[0])
  x, y = car_frame(ped, plan.spawn, plan.heading)
  assert 60.0 < x < 120.0 and abs(y) < 0.5


@pytest.mark.parametrize("entry", ["wpTown_5_left", "autojunction_1311_left"])
def test_pacer_lane(graph, entry):
  plan = catalog.build('occupied_lane', graph, entry=entry)
  pacer = plan.actors[0]
  lane = np.array(pacer.path)
  s, off, h = catalog.project(lane, np.asarray(plan.spawn))
  assert off == pytest.approx(3.6, abs=0.1)  # the car is one lane right of the pacer: the lane a left turn changes into
  assert abs(math.degrees(wrap(h - plan.heading))) < 5.0
  end_to_junction = math.hypot(*(lane[-1, :2] - np.asarray(plan.junction)))
  assert end_to_junction == pytest.approx(catalog.PACER_END_BEFORE_M, abs=15.0)
  assert pacer.params['s_junction'] == pytest.approx(arc_lengths(lane)[-1] + catalog.PACER_END_BEFORE_M, abs=1.0)


def test_unprotected_lefts_give_the_car_3_to_4_s_after_its_go_before_the_oncoming_cars(graph):
  # the car stands at the junction's signal line and goes on the driver's tap 2 s later: the first oncoming car, released
  # a fixed time after the stand, reaches the junction 3-4 s after that go, as the car pulls away toward its lanes
  from openpilot.tools.sim.bridge.beamng.driver import STOPPED_TAP_S
  from openpilot.tools.sim.bridge.beamng.scenarios.timing import travel_time
  lefts = {n: e for n, e in catalog.SCENARIOS.items() if e.actor == 'oncoming' and not e.stream_headway_s}
  assert set(lefts) >= {'unprotected_left', 'unprotected_left_visible'}
  for n, e in lefts.items():
    car = catalog.build(n, graph).actors[0]
    arrive = car.params['release_after_stop_s'] + travel_time(car.params['start_m'], car.speed, car.accel) - STOPPED_TAP_S
    assert 3.0 <= e.arrive_after_s <= 4.0 and arrive == pytest.approx(e.arrive_after_s, abs=0.05), n


def test_every_scenario_carries_a_road_model_and_starts_on_its_own_side(graph):
  from openpilot.tools.sim.bridge.beamng.scenarios import score
  for name in catalog.SCENARIOS:
    plan = catalog.build(name, graph)
    assert plan.road is not None and plan.road['links'], name
    past = score.oncoming_offsets([{'x': plan.spawn[0], 'y': plan.spawn[1], 'heading': plan.heading}], plan.road)[0]
    if past is None and plan.notes.get('data_entry') == 'autojunction_1311_left':
      continue  # a divided road, two lanes our way: no oncoming lane beside it
    if past is None and name == 'twoway_route_pair':
      continue  # a one-way street to its stop (test_twoway): no oncoming lane beside it
    assert past is not None and past < -0.5, f"{name}: the spawn is {past} m left of the road's divider"


# *** the signal tiles' controls (wcusa_map_node_signals.json, which the sim tiles were built from) ***
def data_entries():
  with open(catalog.DATA_JSON) as f:
    return [d['name'] for d in json.load(f)]


def every_plan(graph):
  for name, e in catalog.SCENARIOS.items():
    for entry in (data_entries() if e.kind == 'data' else ['']):
      if e.kind != 'route':  # route_demo drives a whole route from the bridge's default spawn
        yield name, entry, catalog.build(name, graph, entry=entry)


def test_mapped_controls_load_with_their_direction(graph):
  controls = catalog.mapped_controls(graph)
  assert len(controls) > 300 and {c.kind for c in controls} == {'signal', 'stop'}
  tl46 = next(c for c in controls if c.instance == 'trafficLight 46')  # unprotected_left's own signal, DR297_2 -> DR307_1
  assert math.hypot(tl46.x + 655.7, tl46.y - 470.6) < 0.5
  b, a = graph.pos['DR307_1'][:2], graph.pos['DR297_2'][:2]
  assert abs(wrap(tl46.heading - math.atan2(b[1] - a[1], b[0] - a[0]))) < 1e-6


def test_no_spawn_starts_inside_braking_distance_of_a_mapped_control(graph):
  # BeamNG smoke 2026-10-28: lane_change, occupied_lane and unprotected_left_visible spawned 20-37 m before a mapped signal,
  # and the rolling start ran over its line ("Stop Line Passed") before openpilot was engaged to stop for it. Every spawn
  # is more than the roll plus a map stop's engage distance before each control on its approach, or past it
  for name, entry, plan in every_plan(graph):
    clear = catalog.spawn_clear_m(plan.speed_kph)
    assert clear > 60.
    for d, c in plan.notes['controls_from_spawn']:
      assert d >= clear or d <= -catalog.SPAWN_PAST_M, f"{name} {entry}: {c} {d:.0f} m from the spawn"


def test_turn_scenarios_know_their_junctions_own_control(graph):
  for name in ('ped_crosswalk_right', 'unprotected_left', 'unprotected_left_visible'):
    plan = catalog.build(name, graph)
    assert plan.notes['junction_control'] == 'signal', name
    assert 10. < plan.notes['stop_line_m'] < 14., name  # the mapped line, m before the junction along the approach


def test_lane_change_entries_have_no_mapped_control_in_the_window(graph):
  # BeamNG smoke 2026-10-28: wpTown_5_left's approach has a signal 160 m before the turn, inside the 250-75 m lane change
  # window at 45 km/h: the car stood at it and was below DesireHelper's minimum speed for most of the window. The default
  # entry (the one a plain run takes) and every entry but the one kept for the lane planner's stop-in-window work are clear
  first = data_entries()[0]
  for name in ('lane_change', 'occupied_lane'):
    plan = catalog.build(name, graph)
    assert plan.notes['data_entry'] == first and plan.notes['controls_in_window'] == [], name
  for entry in data_entries():
    plan = catalog.build('lane_change', graph, entry=entry)
    assert (plan.notes['controls_in_window'] == []) != (entry == 'wpTown_5_left'), entry


def test_a_data_entry_spawned_before_a_control_is_refused(graph, tmp_path):
  with open(catalog.DATA_JSON) as f:
    items = json.load(f)
  bad = dict(items[0], name='too_close')
  near = next(d for d in data_entries() if d == 'wpTown_5_left')
  bad.update(next(i for i in items if i['name'] == near))
  bad['spawn'] = [-357.39, 472.27, 89.98]  # 37 m before trafficLight 14
  bad['heading_deg'] = -134.3
  p = tmp_path / 'data.json'
  p.write_text(json.dumps([bad]))
  with pytest.raises(ValueError, match="trafficLight 14"):
    catalog.build('lane_change', graph, data_path=str(p))


def test_the_spawn_clearance_rolls_as_the_bridge_does():
  pytest.importorskip("beamngpy")
  from openpilot.tools.sim.bridge.beamng.bridge import ROLLING_START_ACCEL
  assert catalog.ROLL_ACCEL == ROLLING_START_ACCEL


def test_the_road_models_junctions_carry_their_farthest_mapped_stop_line(graph):
  # the score's junction box starts at the mapped stop line for a car heading in (score.oncoming_offsets)
  plan = catalog.build('unprotected_left', graph)
  j = min(plan.road['junctions'], key=lambda j: math.hypot(j[0] - plan.junction[0], j[1] - plan.junction[1]))
  assert len(j) == 4 and 12.0 < j[3] < 16.0  # trafficLight 46 at 12.5 m, 49 at 13.0 m, and the junction's others
  assert all(len(k) == 4 and k[3] >= 0. for k in plan.road['junctions'])


def test_gap_streams_queue_across_the_junction_and_come_at_their_headway(graph):
  # G6 for G4's gate (CRITIQUE REQUIRED 5): a queue standing in the oncoming lane across the junction, the car arriving to
  # it, so it stands at its line; released car by car on timers from the car's stand (never on its go), each reaches the
  # junction its headway after the one before, the first STREAM_FIRST_S after the stand; after the last, an empty road
  from openpilot.tools.sim.bridge.beamng.scenarios.timing import travel_time
  streams = {n: e for n, e in catalog.SCENARIOS.items() if e.stream_headway_s}
  assert {e.stream_headway_s for e in streams.values()} == {4., 6., 8., 10.}
  for n, e in streams.items():
    plan = catalog.build(n, graph)
    cars = [a for a in plan.actors if a.trigger == 'oncoming']
    assert len(cars) == e.oncoming >= 3, n
    starts = [c.params['start_m'] for c in cars]
    assert starts == sorted(starts) and min(b - a for a, b in zip(starts, starts[1:], strict=False)) >= catalog.QUEUE_GAP_M - 1e-6
    arrive = [c.params['release_after_stop_s'] + travel_time(c.params['start_m'], c.speed, c.accel) for c in cars]
    assert arrive[0] == pytest.approx(e.stream_first_s, abs=0.05), n
    assert all(b - a == pytest.approx(e.stream_headway_s, abs=0.05) for a, b in zip(arrive, arrive[1:], strict=False)), n
    assert all(c.params['timed'] and c.params['release_after_stop_s'] >= 0. for c in cars)
    assert all(c.speed == pytest.approx(11.18, abs=0.01) for c in cars)  # the mapped 25 mph
