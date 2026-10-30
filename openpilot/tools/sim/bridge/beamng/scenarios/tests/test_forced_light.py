"""The forced light (S13, S14, S18): the scenario junction's own signal forced red from the run's setup and held, green
green_after_s after the car stands at the junction, the opposing approach with it and the cross approaches the other
way; logged for the chase render; the sim driver never taps on it red; put back after the run."""
import json
import math
import os
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.tools.sim.bridge.beamng import scenario
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, director, run
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_runner import DT, Clock, FakeDirector, FakeSM, World, bridge

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")

LIGHTS = {  # scenario: (its own instance, the opposing one, the cross ones), from wcusa_map_node_signals.json's headings
  'red_straight': ('trafficLight 64', ['trafficLight 65'], ['trafficLight 66', 'trafficLight 67']),
  'red_right': ('trafficLight 62', ['trafficLight 63'], ['trafficLight 60', 'trafficLight 61']),
  'red_right_g62': ('trafficLight 46', ['trafficLight 47'], ['trafficLight 48', 'trafficLight 49']),
}


@pytest.mark.parametrize("name", sorted(LIGHTS))
def test_the_junctions_own_signal_is_forced_with_its_phase(name):
  plan = catalog.build(name)
  own, opposing, cross = LIGHTS[name]
  assert plan.notes['junction_control'] == 'signal' and plan.notes['junction_instance'] == own
  lt = plan.lights
  assert (lt['instance'], sorted(lt['with']), sorted(lt['cross'])) == (own, opposing, cross)
  assert lt['state'] == 'red' and lt['green_after_s'] == catalog.SCENARIOS[name].green_after_s > 0.
  assert lt['states_at_setup'] == {own: 'red', **dict.fromkeys(opposing, 'red'), **dict.fromkeys(cross, 'green')}


def test_red_right_g62_comes_from_the_south_west_and_expects_the_miss():
  plan = catalog.build('red_right_g62')
  assert abs(math.degrees(plan.heading) - 45.) < 10. and plan.expect == ('turnMissed',)


def test_scenarios_without_a_light_force_none():
  for name in ('ped_crosswalk_right', 'straight_cruise', 'unprotected_left_visible', 'route_demo'):
    assert catalog.build(name).lights is None


class LightDirector(FakeDirector):
  def __init__(self, *a, **k):
    super().__init__(*a, **k)
    self.states: dict[str, str] = {}
    self.light_log: list[list] = []

  def set_lights(self, states, green_held):
    self.states.update(states)
    self.light_log += [[round(self.clock.wall(), 3), i, st] for i, st in states.items()]


class LightWorld(World):
  """test_runner's World, standing at the junction's line while its forced light is red (the oracle and turnpilotd's red)."""
  def __init__(self, plan, clock, director_ref):
    super().__init__(plan, clock)
    self.director_ref = director_ref

  def step(self, b):
    d = self.director_ref()
    red = d is not None and d.states.get(self.plan.lights['instance']) == 'red'
    if self.hold is not None and red:
      self.hold = (self.hold[0], max(self.hold[1], self.clock.t + DT))
    super().step(b)


def test_the_light_goes_green_after_the_stand_and_the_run_logs_it(tmp_path, monkeypatch):
  clock = Clock()
  plan = catalog.build('red_right')
  holder = {}
  world = LightWorld(plan, clock, lambda: holder.get('d'))
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  monkeypatch.setattr(run, 'wall', clock.wall)
  monkeypatch.setattr(run, 'Director', LightDirector)
  monkeypatch.setattr(FakeDirector, 'clock', clock)
  monkeypatch.setattr(run, 'web_call', world.web)
  monkeypatch.setattr(run.messaging, 'SubMaster', lambda services: FakeSM(world))
  runner = run.Runner(plan, 1, str(tmp_path), 'http://127.0.0.1:8088', 'tok', 'host', 0)
  b = bridge()
  b.cfg.green = True
  b.world.teleport = lambda pos, rot: (b.teleports.append(pos), setattr(world, 's', 0.0), setattr(world, 'hold', None))
  b.simulator_state.position, b.simulator_state.bearing, b.simulator_state.wheel_speed = SimpleNamespace(x=0, y=0, z=0), 0.0, 0.0
  for _ in range(int(200 / DT)):
    if runner.director is not None:
      holder['d'] = runner.director
    world.step(b)
    if runner(b):
      break
    b.rk_frame += 1
    clock.t += DT
  out = json.load(open(tmp_path / 'result.json'))
  (r,) = out['runs']
  calls = [(c[0], c[1]) for c in runner.director.calls if c[0] == 'set_lights']
  own = plan.lights['instance']
  assert calls[0][1][0] == plan.lights['states_at_setup'] and calls[0][1][1] is True
  assert calls[1][1][0][own] == 'green' and all(calls[1][1][0][i] == 'red' for i in plan.lights['cross'])
  fl = r['forced_light']
  stand = next(row['t'] for row in r['rows'] if row['v'] < 0.3 and row['j_dist'] < 35.)
  assert fl['stand_t'] == pytest.approx(stand) and fl['green_t'] == pytest.approx(stand + plan.lights['green_after_s'], abs=0.06)
  assert {row['forced_light'] for row in r['rows'] if row['t'] < fl['green_t']} == {'red'}
  assert {row['forced_light'] for row in r['rows'] if row['t'] >= fl['green_t'] + 0.05} == {'green'}
  assert [e[1:] for e in r['light_events']][:1] == [[own, 'red']]
  assert r['outcome'] == 'arrived' and r['red_light']['stood_at_red'] and r['red_light']['past_line_on_red_t'] is None
  assert out['signals_running'] is False and out['lights'] == plan.lights
  assert r['passed'], r['failures']


def test_the_sim_driver_never_taps_on_the_forced_red(monkeypatch):
  # without --signals every light reads green to the driver's ground truth (driver.ground_truth_green): not this one
  from openpilot.tools.sim.bridge.beamng.driver import TAP_GAS, GasTaps
  clock = Clock()
  runner = run.Runner(catalog.build('red_right'), 1, '/tmp', 'http://127.0.0.1:8088', 'tok', 'host', 0)
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  runner.taps, runner.light_state = GasTaps(), 'red'
  msgs = {}
  ss = messaging.new_message('selfdriveState')
  ss.selfdriveState.enabled = True
  tp = messaging.new_message('turnpilotPlan')
  tp.turnpilotPlan.controlPrompt, tp.turnpilotPlan.state = 'signalGo', 'holding'
  msgs['selfdriveState'], msgs['turnpilotPlan'] = ss, tp
  runner.sm = type('SM', (), {'seen': {'turnpilotPlan': True, 'turnpilotLights': False}, 'recv_time': {'turnpilotPlan': clock.t},
                              '__getitem__': lambda self, s: getattr(msgs[s].as_reader(), s)})()
  b = bridge()
  b.cfg.green, b.simulator_state.wheel_speed = True, 0.
  gas = []
  for k in range(120):
    if k == 60:
      runner.light_state = 'green'
    runner.sm.recv_time['turnpilotPlan'] = clock.t
    runner.driver(b)
    gas.append(b.driver_gas)
    clock.t += 0.05
  assert max(gas[:60]) == 0. and TAP_GAS in gas[60:]


class FakeSignals:
  def __init__(self, log):
    self.log = log

  def set_instance_strict_state(self, name, state):
    self.log.append(('strict', name, state))


def fake_director(monkeypatch):
  d = director.Director('host', 0)
  calls = []
  d.bng = SimpleNamespace(traffic_signals=FakeSignals(calls))
  monkeypatch.setattr(director.scenario, 'lua', lambda bng, chunk, response=False: calls.append(('lua', chunk)))
  monkeypatch.setattr(director, 'wall', lambda: 5.0)
  return d, calls


def test_the_director_forces_holds_logs_and_puts_back(monkeypatch):
  d, calls = fake_director(monkeypatch)
  d.set_lights({'trafficLight 62': 'red', 'trafficLight 60': 'green'}, True)
  assert ('strict', 'trafficLight 62', scenario.RED) in calls and ('strict', 'trafficLight 60', scenario.GREEN) in calls
  assert ('lua', "core_trafficSignals.setActive(false)") in calls  # held: the sequence would overwrite a strict state
  assert d.light_log == [[5.0, 'trafficLight 62', 'red'], [5.0, 'trafficLight 60', 'green']]
  calls.clear()
  d.cleanup()  # the bridge holds every light green (no --signals): green again, still held
  assert sorted(c for c in calls if c[0] == 'strict') == [('strict', 'trafficLight 60', scenario.GREEN),
                                                          ('strict', 'trafficLight 62', scenario.GREEN)]
  assert not d.forced
  d.set_lights({'trafficLight 62': 'red'}, False)
  calls.clear()
  d.cleanup()  # --signals: back on their sequences
  assert ('strict', 'trafficLight 62', None) in calls and ('lua', "core_trafficSignals.setActive(true)") in calls


class FailingLights(LightDirector):
  """The game refused the forced state: the job's future carries the exception, as the real director's does."""
  def submit(self, fn, *args):
    if getattr(fn, '__name__', '') == 'set_lights':
      self.calls.append(('set_lights', args))
      self.times.append(None if self.clock is None else self.clock.t)
      f: Future = Future()
      f.set_exception(RuntimeError("no such signal instance"))
      return f
    return super().submit(fn, *args)


def test_a_forced_light_that_fails_to_set_ends_the_run_loudly(tmp_path, monkeypatch):
  # otherwise the rows would say forced_light red for a light the game never showed red
  clock = Clock()
  plan = catalog.build('red_right')
  world = World(plan, clock)
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  monkeypatch.setattr(run, 'wall', clock.wall)
  monkeypatch.setattr(run, 'Director', FailingLights)
  monkeypatch.setattr(FakeDirector, 'clock', clock)
  monkeypatch.setattr(run, 'web_call', world.web)
  monkeypatch.setattr(run.messaging, 'SubMaster', lambda services: FakeSM(world))
  runner = run.Runner(plan, 1, str(tmp_path), 'http://127.0.0.1:8088', 'tok', 'host', 0)
  b = bridge()
  b.cfg.green = True
  b.world.teleport = lambda pos, rot: (b.teleports.append(pos), setattr(world, 's', 0.0), setattr(world, 'hold', None))
  b.simulator_state.position, b.simulator_state.bearing, b.simulator_state.wheel_speed = SimpleNamespace(x=0, y=0, z=0), 0.0, 0.0
  for _ in range(int(60 / DT)):
    world.step(b)
    if runner(b):
      break
    b.rk_frame += 1
    clock.t += DT
  (r,) = json.load(open(tmp_path / 'result.json'))['runs']
  assert r['outcome'] == 'error' and 'forced light' in r['detail'] and 'no such signal instance' in r['detail']
  assert not r['rows']  # never engaged on it
