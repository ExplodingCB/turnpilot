"""--light-oracle starts the light oracle from the scenario runner and route_drive.py, as it does from run_bridge.py, and
stops it when the drive ends: run_bridge.parse_args takes the flag for all three, so taking it and starting nothing would
leave turnpilotd holding every signal for the driver's go with no word why."""
import subprocess
import sys
from types import SimpleNamespace

import pytest

from openpilot.tools.sim.bridge.beamng import route_drive, run_bridge
from openpilot.tools.sim.bridge.beamng.scenarios import run


class FakeProc:
  def __init__(self, hangs=False):
    self.terminated, self.killed, self.hangs = False, False, hangs

  def terminate(self):
    self.terminated = True

  def kill(self):
    self.killed = True

  def wait(self, timeout=None):
    if self.hangs:
      raise subprocess.TimeoutExpired('light_oracle', timeout)
    return 0


class FakeBridge:
  joined = False

  def __init__(self, *args, **kwargs):
    pass

  def run(self, q):
    return SimpleNamespace(join=lambda: setattr(FakeBridge, 'joined', True))


@pytest.fixture
def oracle(monkeypatch):
  started = []

  def start(args):
    assert FakeBridge.joined is False  # up before the drive
    started.append((args.light_oracle, args.light_latency, FakeProc()))
    return started[-1][2]
  FakeBridge.joined = False
  monkeypatch.setattr(run_bridge, 'start_light_oracle', start)
  return started


def run_main(monkeypatch, tmp_path, flags):
  monkeypatch.setattr(sys, 'argv', ['run.py', 'ped_in_path', '--out', str(tmp_path), *flags])
  monkeypatch.setattr(run, 'build', lambda *a, **k: SimpleNamespace(spawn=(0., 0., 0.), heading=0., notes={}))
  monkeypatch.setattr(run, 'Params', lambda: SimpleNamespace(get_bool=lambda k: True, get=lambda k: None))
  monkeypatch.setattr(run.tree, 'manager_dirs', lambda prefix: [])  # tree.json: no stack found, said in it
  monkeypatch.setattr(run, 'ensure_token', lambda params: 'tok')
  monkeypatch.setattr(run, 'sweep_game', lambda host, port: None)
  monkeypatch.setattr(run, 'Runner', lambda *a, **k: None)
  monkeypatch.setattr('openpilot.tools.sim.bridge.beamng.bridge.BeamNGBridge', FakeBridge)
  run.main()


def drive_main(monkeypatch, flags):
  monkeypatch.setattr(sys, 'argv', ['route_drive.py', *flags])
  monkeypatch.setattr(route_drive, 'BeamNGBridge', FakeBridge)
  monkeypatch.setattr(route_drive, 'RouteDrive', lambda *a, **k: SimpleNamespace())
  monkeypatch.setattr(route_drive.signal, 'signal', lambda *a: None)
  route_drive.main()


def test_scenario_runner_starts_and_stops_the_oracle(monkeypatch, tmp_path, oracle):
  run_main(monkeypatch, tmp_path, ['--signals', '--light-oracle', '--light-latency', '0.5'])
  assert FakeBridge.joined and [(o, lat) for o, lat, _ in oracle] == [(True, 0.5)] and oracle[0][2].terminated


def test_scenario_runner_without_the_flag_starts_none(monkeypatch, tmp_path, oracle):
  run_main(monkeypatch, tmp_path, [])
  assert FakeBridge.joined and oracle == []


def test_route_drive_starts_and_stops_the_oracle(monkeypatch, oracle):
  drive_main(monkeypatch, ['--signals', '--light-oracle'])
  assert FakeBridge.joined and len(oracle) == 1 and oracle[0][2].terminated
  oracle.clear()
  FakeBridge.joined = False
  drive_main(monkeypatch, [])
  assert FakeBridge.joined and oracle == []


def test_an_oracle_that_wont_stop_is_killed_and_the_run_still_ends(monkeypatch):
  proc = FakeProc(hangs=True)
  monkeypatch.setattr(run_bridge, 'start_light_oracle', lambda args: proc)
  with run_bridge.light_oracle(SimpleNamespace(light_oracle=True)):
    pass
  assert proc.terminated and proc.killed
