"""ped_step_out (S10): a walker standing at the road edge on the car's side steps into the middle of its lane when the
car is step_out_distance(v) away, so that, at whatever speed the car comes, the stop needs harder braking than -2.0 m/s^2
(scene_policy's comfortable floor, only a confirmed track gets more) and no more than the Tesla's -3.48, whether the
policy takes her as in the path at her first step (ENTER_T_S) or only inside the corridor.

She faces the car, standing and through her step, as ped_in_path's mannequin does: BeamNG's walker is a low-poly block
figure, and side-on (facing across the road, the way she walks) it never scores as a person (BeamNG build findings D3:
best 0.197 from 56 m to contact, 0 frames at 0.35; facing the car on the same street up to 0.84). ped_step_out_side is the
same step side-on, kept as the detector-limit clip, scored the same (it fails on contact)."""
import json
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.tesla.values import CarControllerParams
from openpilot.selfdrive.turnpilot.hooks import SPEED_CAP_ACCEL_MIN
from openpilot.selfdrive.turnpilot.scene.tracker import CORRIDOR_HALF_WIDTH
from openpilot.selfdrive.turnpilot.scene_policy import ENTER_T_S, PED_STOP_GAP_M
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON, wrap
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, director, run, score, timing
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_runner import DT, Clock, FakeDirector, FakeSM, World, bridge

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


def car_frame(p, plan):
  d = np.asarray(p[:2]) - np.asarray(plan.spawn[:2])
  h = plan.heading
  return float(d[0] * math.cos(h) + d[1] * math.sin(h)), float(-d[0] * math.sin(h) + d[1] * math.cos(h))


STEP_OUTS = ('ped_step_out', 'ped_step_out_side')


@pytest.mark.parametrize("name", STEP_OUTS)
def test_she_stands_at_the_road_edge_and_steps_into_the_middle_of_the_lane(name):
  plan = catalog.build(name)
  (ped,) = plan.actors
  assert ped.kind == 'walker' and ped.trigger == 'step_out'
  (x0, y0), (x1, y1) = car_frame(ped.path[0], plan), car_frame(ped.path[-1], plan)
  assert abs(x1 - x0) < 0.5                       # straight across the lane
  assert y0 < -CORRIDOR_HALF_WIDTH < y1 + 0.1    # from outside the car's corridor on its right...
  assert abs(y1) < 0.3                           # ...to the middle of its lane, where she stays: the stop is forced
  assert ped.params['enter_s'] == pytest.approx((-y0 - CORRIDOR_HALF_WIDTH) / timing.WALK_SPEED, abs=0.05)
  assert ped.params['enter_s'] < ENTER_T_S        # she is "entering" (scene_policy) from her first step
  assert plan.expect == ('hardBrake',) and plan.end_on_stop_s > 0.
  # she stands well before the junction's signal line, past where the car engages
  assert plan.notes['stop_line_m'] + 20. < catalog.SCENARIOS[name].ped_before_m < plan.notes['approach_m'] - 45.


def test_s10s_walker_faces_the_car_and_the_side_on_one_is_the_detector_limit():
  plan, side = catalog.build('ped_step_out'), catalog.build('ped_step_out_side')
  (ped,), (ped_side,) = plan.actors, side.actors
  assert ped.path == ped_side.path and {k: v for k, v in ped.params.items() if k != 'face'} == ped_side.params
  # facing the car (against its travel, as ped_in_path's standing mannequin), and kept so through the step
  assert abs(wrap(ped.heading - (plan.heading + math.pi))) < math.radians(10.) and ped.params['face'] == ped.heading
  # side-on: facing the way she walks, across the road, turning with her path
  across = math.atan2(ped_side.path[1][1] - ped_side.path[0][1], ped_side.path[1][0] - ped_side.path[0][0])
  assert abs(wrap(ped_side.heading - across)) < 1e-6 and 'face' not in ped_side.params
  assert plan.notes['step_out']['faces'] == 'car' and side.notes['step_out']['faces'] == 'across'
  assert 'mannequin' in plan.notes['step_out']['label'] and 'detector limit' in side.notes['step_out']['label']
  assert side.expect == plan.expect == ('hardBrake',)


@pytest.mark.parametrize("name", [*STEP_OUTS, 'ped_crosswalk_right'])
def test_the_director_keeps_a_facing_walker_facing_and_turns_the_others_with_their_path(monkeypatch, name):
  plan = catalog.build(name)
  (ped,) = plan.actors
  moves = []
  monkeypatch.setattr(director.scenario, 'move_pedestrian', lambda bng, n, x, y, z, h: moves.append(h))
  d = director.Director('host', 0)
  path = np.array(ped.path, dtype=float)
  d.walkers[ped.name] = {'actor': ped, 'path': path, 'done': False, 't0': None}
  length = float(timing.arc_lengths(path)[-1])
  for f in (0.1, 0.5, 0.95):  # along the walk
    d.walkers[ped.name]['t0'] = director.time.monotonic() - f * length / ped.speed
    d.tick()
    _, want = timing.walker_pose(path, f * length / ped.speed, ped.speed)
    if 'face' in ped.params:
      want = ped.params['face']
    assert abs(wrap(moves[-1] - want)) < 0.05, (name, f)
    assert d.poses[ped.name][2] == moves[-1]  # the pose log (chase render) faces the same way
  if name == 'ped_crosswalk_right':  # across the road, then away along the far sidewalk: she turns
    assert abs(wrap(moves[0] - moves[-1])) > math.radians(45.)


@pytest.mark.parametrize("name", STEP_OUTS)
def test_both_are_scored_as_the_step_out(name):
  rows = [{'t': i * 0.05, 'v': max(8. - 0.4 * i, 0.), 'ped_gap': 1.2, 'enabled': True, 'alert': '', 'x': None, 'y': None,
           'a_target': -1.0, 'accel_cap': 0., 'tp_accel': 0., 'control_alert': 'none'} for i in range(40)]
  res = score.summarize(name, rows, 'stopped', expect=('hardBrake',))
  assert "came within 1.2 m of the pedestrian" in res['failures'] and not res['passed']
  assert any('hardBrake' in f for f in res['failures'])


@pytest.mark.parametrize("v", [5., 6.67, 8.33, 10.])
def test_the_stop_needs_more_than_the_comfortable_floor_and_no_more_than_the_tesla_has(v):
  p = catalog.build('ped_step_out').actors[0].params
  d = timing.step_out_distance(v, p)
  react = p['react_s']
  # in the path at her first step, braking react_s later; or only once in the corridor, enter_s after it
  early = v ** 2 / (2. * (d - v * react - PED_STOP_GAP_M))
  late = v ** 2 / (2. * (d - v * (react + p['enter_s']) - PED_STOP_GAP_M))
  assert -SPEED_CAP_ACCEL_MIN < early <= late <= -CarControllerParams.ACCEL_MIN, (v, d, early, late)


def test_at_30_kph_she_steps_out_about_20_to_25_m_ahead():
  p = catalog.build('ped_step_out').actors[0].params
  assert 20. <= timing.step_out_distance(30. / 3.6, p) <= 25.


@pytest.mark.parametrize("name", STEP_OUTS)
def test_the_runner_sends_her_when_the_car_is_that_far(tmp_path, monkeypatch, name):
  clock = Clock()
  plan = catalog.build(name)
  world = World(plan, clock)
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  monkeypatch.setattr(run, 'wall', clock.wall)
  monkeypatch.setattr(run, 'Director', FakeDirector)
  monkeypatch.setattr(FakeDirector, 'clock', clock)
  monkeypatch.setattr(run, 'web_call', world.web)
  monkeypatch.setattr(run.messaging, 'SubMaster', lambda services: FakeSM(world))
  runner = run.Runner(plan, 1, str(tmp_path), 'http://127.0.0.1:8088', 'tok', 'host', 0)
  b = bridge()
  b.world.teleport = lambda pos, rot: (b.teleports.append(pos), setattr(world, 's', 0.0), setattr(world, 'hold', None))
  b.simulator_state.position, b.simulator_state.bearing, b.simulator_state.wheel_speed = SimpleNamespace(x=0, y=0, z=0), 0.0, 0.0
  for _ in range(int(200 / DT)):
    world.step(b)
    if runner(b):
      break
    b.rk_frame += 1
    clock.t += DT
  (r,) = json.load(open(tmp_path / 'result.json'))['runs']
  walks = [t for (fn, _), t in zip(runner.director.calls, runner.director.times, strict=True) if fn == 'walk']
  assert len(walks) == 1
  so = r['step_out']
  want = timing.step_out_distance(so['v'], plan.actors[0].params)
  assert want - 8. * 0.05 - 0.01 <= so['ahead_m'] <= want + 0.01  # on the first 20 Hz row inside it, at the fake car's 8 m/s
  assert so['needs_decel'] == pytest.approx(plan.actors[0].params['decel'], abs=0.25)
