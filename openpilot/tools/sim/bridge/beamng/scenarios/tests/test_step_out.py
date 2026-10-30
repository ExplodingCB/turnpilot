"""ped_step_out (S10): a walker standing at the road edge on the car's side steps into the middle of its lane when the
car is step_out_distance(v) away, so that, at whatever speed the car comes, the stop needs harder braking than -2.0 m/s^2
(scene_policy's comfortable floor, only a confirmed track gets more) and no more than the Tesla's -3.48, whether the
policy takes her as in the path at her first step (ENTER_T_S) or only inside the corridor."""
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
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, run, timing
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_runner import DT, Clock, FakeDirector, FakeSM, World, bridge

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


def car_frame(p, plan):
  d = np.asarray(p[:2]) - np.asarray(plan.spawn[:2])
  h = plan.heading
  return float(d[0] * math.cos(h) + d[1] * math.sin(h)), float(-d[0] * math.sin(h) + d[1] * math.cos(h))


def test_she_stands_at_the_road_edge_and_steps_into_the_middle_of_the_lane():
  plan = catalog.build('ped_step_out')
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
  assert plan.notes['stop_line_m'] + 20. < catalog.SCENARIOS['ped_step_out'].ped_before_m < plan.notes['approach_m'] - 45.


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


def test_the_runner_sends_her_when_the_car_is_that_far(tmp_path, monkeypatch):
  clock = Clock()
  plan = catalog.build('ped_step_out')
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
