"""A run asked for with --light-oracle never drives without it (build findings H2): the oracle is a process of its own, and
one that died before the level loaded (BeamNG ctl_ped_in_path, 2026-10-30: 'core_trafficSignals not available') or stops
publishing leaves every light unknown, every signal held for the driver's go, with no word why. So the runner waits for the
oracle's first turnpilotLights before the destination goes out (ORACLE_WAIT_S from the teleport), and ends the run 'error'
when none comes or the oracle goes quiet (ORACLE_STALE_S), and the invocation with it."""
import json
import os
from types import SimpleNamespace

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, run
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_runner import DT, Clock, FakeDirector, FakeSM, World, bridge

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


class OracleWorld(World):
  """The fake world with the light oracle publishing at 4 Hz from up_s to down_s s after the start."""
  def __init__(self, plan, clock, up_s: float, down_s: float):
    super().__init__(plan, clock)
    self.t0, self.up_s, self.down_s, self.next_light = clock.t, up_s, down_s, 0.

  def step(self, b):
    super().step(b)
    t = self.clock.t - self.t0
    if self.up_s <= t < self.down_s and self.clock.t >= self.next_light:
      self.next_light = self.clock.t + 0.25
      m = messaging.new_message('turnpilotLights', valid=True)
      m.turnpilotLights.source = 'simOracle'
      self.msgs['turnpilotLights'] = m
      self.updated.add('turnpilotLights')


def drive(tmp_path, monkeypatch, up_s=0., down_s=1e9, oracle=True, runs=1):
  clock = Clock()
  plan = catalog.build('right_green')
  world = OracleWorld(plan, clock, up_s, down_s)
  posts = []

  def web(url, token, method, body=None):
    posts.append((round(clock.t - world.t0, 2), method))
    return world.web(url, token, method, body)
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  monkeypatch.setattr(run, 'wall', clock.wall)
  monkeypatch.setattr(run, 'Director', FakeDirector)
  monkeypatch.setattr(run, 'web_call', web)
  monkeypatch.setattr(run.messaging, 'SubMaster', lambda services: FakeSM(world))
  runner = run.Runner(plan, runs, str(tmp_path), 'http://127.0.0.1:8088', 'tok', 'host', 0, light_oracle=oracle)
  b = bridge()
  b.world.teleport = lambda pos, rot: (b.teleports.append(pos), setattr(world, 's', 0.0), setattr(world, 'hold', None))
  b.simulator_state.position, b.simulator_state.bearing, b.simulator_state.wheel_speed = SimpleNamespace(x=0, y=0, z=0), 0.0, 0.0
  for _ in range(int(300 / DT)):
    world.step(b)
    if runner(b):
      break
    b.rk_frame += 1
    clock.t += DT
  assert b.done
  out = json.load(open(tmp_path / 'result.json'))
  return out['runs'], posts, world


def test_an_oracle_that_never_publishes_ends_the_run_before_it_drives(tmp_path, monkeypatch):
  runs, posts, world = drive(tmp_path, monkeypatch, up_s=1e9, runs=2)
  assert [r['outcome'] for r in runs] == ['error']  # the invocation ends with it: the next run would have no oracle either
  r = runs[0]
  assert 'light oracle' in r['detail'] and r['t_engaged_wall'] is None and not r['rows']
  assert ('POST' not in [m for _, m in posts]) and not world.enabled
  assert run.ORACLE_WAIT_S - 0.1 <= r['t_end_wall'] - r['t_start_wall'] <= run.ORACLE_WAIT_S + 0.5


def test_an_oracle_that_comes_up_late_waits_and_drives(tmp_path, monkeypatch):
  # the oracle's own retry outlasts the level load, so its first lights can come some seconds after the bridge's first run
  runs, posts, _ = drive(tmp_path, monkeypatch, up_s=run.ORACLE_WAIT_S - 5.)
  assert [r['outcome'] for r in runs] == ['arrived']
  assert min(t for t, m in posts if m == 'POST') >= run.ORACLE_WAIT_S - 5.


def test_an_oracle_that_goes_quiet_mid_run_ends_it(tmp_path, monkeypatch):
  runs, _, _ = drive(tmp_path, monkeypatch, down_s=20.)
  r = runs[0]
  assert r['outcome'] == 'error' and 'light oracle' in r['detail'] and r['rows']
  quiet = 20. - (r['t_engaged_wall'] - r['t_start_wall'])  # s since engagement when the oracle's last lights went out
  assert quiet - 0.3 + run.ORACLE_STALE_S <= r['rows'][-1]['t'] <= quiet + run.ORACLE_STALE_S + 0.2


def test_the_oracle_up_throughout_changes_nothing(tmp_path, monkeypatch):
  runs, _, _ = drive(tmp_path, monkeypatch)
  assert [r['outcome'] for r in runs] == ['arrived']


def test_without_the_flag_no_lights_is_no_error(tmp_path, monkeypatch):
  runs, _, _ = drive(tmp_path, monkeypatch, up_s=1e9, oracle=False)
  assert [r['outcome'] for r in runs] == ['arrived']


def test_run_main_hands_the_flag_to_the_runner(monkeypatch, tmp_path):
  from openpilot.tools.sim.bridge.beamng.scenarios.tests import test_light_oracle_wiring as wiring
  got = []
  monkeypatch.setattr(run_bridge_mod(), 'start_light_oracle', lambda args: wiring.FakeProc())
  wiring.FakeBridge.joined = False
  monkeypatch.setattr(run, 'build', lambda *a, **k: SimpleNamespace(spawn=(0., 0., 0.), heading=0., notes={}))
  monkeypatch.setattr(run, 'Params', lambda: SimpleNamespace(get_bool=lambda k: True, get=lambda k: None))
  monkeypatch.setattr(run.tree, 'running_managers', lambda prefix: [])
  monkeypatch.setattr(run, 'ensure_token', lambda params: 'tok')
  monkeypatch.setattr(run, 'sweep_game', lambda host, port: None)
  monkeypatch.setattr(run, 'Runner', lambda *a, **k: got.append(k.get('light_oracle')))
  monkeypatch.setattr('openpilot.tools.sim.bridge.beamng.bridge.BeamNGBridge', wiring.FakeBridge)
  for flags, want in ((['--signals', '--light-oracle'], True), ([], False)):
    monkeypatch.setattr('sys.argv', ['run.py', 'ped_in_path', '--out', str(tmp_path), *flags])
    run.main()
    assert got[-1] is want


def run_bridge_mod():
  from openpilot.tools.sim.bridge.beamng import run_bridge
  return run_bridge
