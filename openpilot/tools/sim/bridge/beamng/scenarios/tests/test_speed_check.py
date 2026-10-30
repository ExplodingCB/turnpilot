"""The speed openpilot was given against the car's travel, from each run's teleport (BeamNG build findings D4-speed): the
bridge's speed came from electrics.wheelspeed, which over-read on S08's braking approach (14%) and S02's rolling start
(19.5%, before engagement, where the result's rows don't reach), so turnpilotd's route follower put the junction 6-13 m too
near and the left went undecided at the line. The runner integrates the speed it fed openpilot (and the wheels' own, logged
beside it) over the pose log's travel; a run whose speed is off by more than SPEED_TOL fails, as harness, not TurnPilot."""
import json
import os
from types import SimpleNamespace

import pytest

from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, run, score
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_runner import DT, Clock, FakeDirector, FakeSM, World, bridge

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


class MovingWorld(World):
  """The fake world with the IMU pose following the car, the speed openpilot gets k times the car's, the wheels' wheel_k."""
  def __init__(self, plan, clock, k: float, wheel_k: float):
    super().__init__(plan, clock)
    self.k, self.wheel_k = k, wheel_k

  def step(self, b):
    super().step(b)
    v = b.simulator_state.wheel_speed
    b.simulator_state.wheel_speed, b.simulator_state.wheel_speed_electrics = v * self.k, v * self.wheel_k
    p = b.simulator_state.position
    b.world.pose = SimpleNamespace(pos=(p.x, p.y, 0.), fwd=(1., 0., 0.), up=(0., 0., 1.))


def drive(tmp_path, monkeypatch, k=1.0, wheel_k=1.15):
  clock = Clock()
  plan = catalog.build('right_green')
  world = MovingWorld(plan, clock, k, wheel_k)
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  monkeypatch.setattr(run, 'wall', clock.wall)
  monkeypatch.setattr(run, 'Director', FakeDirector)
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
  return json.load(open(tmp_path / 'result.json'))


def test_the_speed_given_matches_the_travel(tmp_path, monkeypatch):
  out = drive(tmp_path, monkeypatch)
  r = out['runs'][0]
  sc = r['speed_check']
  assert r['outcome'] == 'arrived' and sc['travel_m'] > 100.
  assert abs(sc['ratio'] - 1.) < 0.01 and abs(sc['wheel_ratio'] - 1.15) < 0.01  # the wheels' over-read, logged, not fed
  assert not any('speed' in f for f in r['failures'])
  assert all(row['v_wheel'] is not None for row in r['rows'])


def test_a_speed_off_by_more_than_the_tolerance_fails_the_run(tmp_path, monkeypatch):
  out = drive(tmp_path, monkeypatch, k=1.10)
  r = out['runs'][0]
  assert abs(r['speed_check']['ratio'] - 1.10) < 0.01
  why = next(f for f in r['failures'] if f.startswith("the sim's speed"))
  assert 'harness' in why and '+10%' in why and not r['passed']
  # a rescore keeps it
  res = score.summarize(out['scenario'], r['rows'], r['outcome'], road=out['road'], speed=r['speed_check'])
  assert why in res['failures']


def test_the_check_bounds():
  ok = {'v_m': 102., 'wheel_m': 120., 'travel_m': 100., 'ratio': 1.02, 'wheel_ratio': 1.2}
  assert not score.speed_problems(ok)
  assert score.speed_problems(dict(ok, v_m=96.5, ratio=0.965))
  assert not score.speed_problems(dict(ok, v_m=15., travel_m=10., ratio=1.5))  # too short a run to say
  assert not score.speed_problems(None) and score.SPEED_TOL == 0.03
