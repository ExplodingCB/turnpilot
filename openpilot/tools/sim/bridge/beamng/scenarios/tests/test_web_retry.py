"""turnpilot_web refusing the runner's destination calls while the stack is still starting (BeamNG smoke 2026-10-29:
unprotected_left_visible's run ended 'no destination', ConnectionRefusedError, 5 s into it, the fresh stack's web server
not yet listening): each call is retried WEB_RETRY_S apart for ROUTE_TIMEOUT_S, then the run ends 'no destination'."""
import json
import os
import urllib.error
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, run
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_runner import DT, Clock, FakeDirector, FakeSM, World, bridge

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


class Director(FakeDirector):
  """As the real one: a job's exception goes in its future, never out of submit."""
  def submit(self, fn, *args):
    self.calls.append((getattr(fn, '__name__', str(fn)), args))
    f: Future = Future()
    try:
      f.set_result(fn(*args))
    except Exception as e:
      f.set_exception(e)
    return f


def drive(tmp_path, monkeypatch, up_at_s: float):
  clock = Clock()
  plan = catalog.build('right_green')
  world = World(plan, clock)
  calls = []
  t0 = clock.t

  def web(url, token, method, body=None):
    calls.append((round(clock.t - t0, 2), method))
    if clock.t - t0 < up_at_s:
      raise urllib.error.URLError(ConnectionRefusedError(61, 'Connection refused'))
    return world.web(url, token, method, body)
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  monkeypatch.setattr(run, 'wall', clock.wall)
  monkeypatch.setattr(run, 'Director', Director)
  monkeypatch.setattr(run, 'web_call', web)
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
  return json.load(open(tmp_path / 'result.json'))['runs'][0], calls


def test_a_web_server_still_starting_is_waited_for(tmp_path, monkeypatch):
  r, calls = drive(tmp_path, monkeypatch, up_at_s=12.)
  assert r['outcome'] == 'arrived', (r['outcome'], r['detail'])
  refused = [c for c in calls if c[0] < 12.]
  assert len(refused) >= 3 and {m for _, m in refused} == {'DELETE'}
  gaps = [b[0] - a[0] for a, b in zip(refused, refused[1:], strict=False)]
  assert min(gaps) >= run.WEB_RETRY_S - DT
  assert [m for _, m in calls if _ >= 12.][:2] == ['DELETE', 'POST']


def test_one_that_never_comes_up_ends_the_run_no_destination(tmp_path, monkeypatch):
  r, calls = drive(tmp_path, monkeypatch, up_at_s=1e9)
  assert r['outcome'] == 'no destination' and 'Connection refused' in r['detail']
  assert calls[-1][0] - calls[0][0] <= run.ROUTE_TIMEOUT_S + run.WEB_RETRY_S + DT
