"""The runner's state machine, dry: a fake bridge, director, SubMaster and clock around a car that drives the scenario's
approach, turns at the junction and stops at the destination. Catches the sequencing the live game can't be used for."""
import dataclasses
import json
import math
import os
from concurrent.futures import Future
from types import SimpleNamespace

import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, run

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")
DT = 0.01


class Clock:
  def __init__(self):
    self.t = 1000.0

  def monotonic(self):
    return self.t

  def wall(self):
    return 1.7e9 + self.t


class FakeDirector:
  clock = None  # the test's Clock: each call is logged with its time

  def __init__(self, *args, **kwargs):
    self.calls, self.poses, self.times = [], {}, []

  def start(self):
    pass

  def is_alive(self):
    return True

  def submit(self, fn, *args):
    self.calls.append((getattr(fn, '__name__', str(fn)), args))
    self.times.append(None if self.clock is None else self.clock.t)
    f: Future = Future()
    f.set_result(fn(*args))
    return f

  def setup(self, actors):
    for a in actors:
      self.poses[a.name] = (a.path[0][0], a.path[0][1], a.heading, a.path[0][2])

  def walk(self, name):
    pass

  def release(self, name):
    pass

  def release_pacer(self, name, xy, v):
    pass

  def cleanup(self):
    self.poses.clear()

  def close(self):
    self.calls.append(('close', ()))
    self.times.append(None if self.clock is None else self.clock.t)


LANE_OFFSET_M = 1.75  # the fake car drives this far right of the line between the plan's points: in its own lane


HOLD_S = 3.0  # the fake car stands this long at the junction's stop line, TurnPilot holding it, before its go


class World:
  """The car and navd: drives spawn -> junction -> destination at 8 m/s once engaged, standing HOLD_S at the junction's
  stop line (the mapped one, as on the signal tiles) with TurnPilot holding, the turn's desire out from its go."""
  def __init__(self, plan, clock):
    self.plan, self.clock = plan, clock
    self.path = np.array([plan.spawn[:2], plan.junction, plan.dest[:2]])
    self.seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.path, axis=0), axis=1))]
    self.s, self.enabled, self.dest_set, self.route_t = 0.0, False, False, None
    self.msgs, self.updated, self.next_nav = {}, set(), 0.0
    self.s_stop = self.seg[1] - (plan.notes.get('stop_line_m') or 8.0) - 1.0
    self.hold = None  # (start, end) of this run's stand at the stop line
    self.holds = []   # every run's

  def pos(self):
    """On the way from one point to the next, kept a lane's width (LANE_OFFSET_M) right of the line between them: driving
    the straight line to the junction's centre would put the car on the divider the last 40 m of the way."""
    i = min(int(np.searchsorted(self.seg, self.s, side='right')), len(self.path) - 1)
    a, b = self.path[i - 1], self.path[i]
    f = (self.s - self.seg[i - 1]) / (self.seg[i] - self.seg[i - 1])
    h = math.atan2(b[1] - a[1], b[0] - a[0])
    return a + min(f, 1.0) * (b - a) + LANE_OFFSET_M * np.array([math.sin(h), -math.cos(h)]), h

  def web(self, url, token, method, body=None):
    assert token == 'tok' and url.endswith('/api/destination')
    self.dest_set = method == 'POST'
    self.route_t = self.clock.t + 1.0 if self.dest_set else None
    return {'ok': True}

  def step(self, b):
    v = 8.0 if self.enabled else 0.0
    if self.enabled and self.hold is None and self.s >= self.s_stop:
      self.hold = (self.clock.t, self.clock.t + HOLD_S)
      self.holds.append(self.hold)
    holding = self.hold is not None and self.clock.t < self.hold[1]
    v = 0.0 if holding else v
    self.s = min(self.s + v * DT, self.seg[-1])
    (x, y), h = self.pos()
    b.simulator_state.position = SimpleNamespace(x=float(x), y=float(y), z=0.0)
    b.simulator_state.bearing = 90.0 - math.degrees(h)
    b.simulator_state.wheel_speed = v
    if b.auto_engage and not self.enabled:
      self.enabled = True
      b.auto_engage = False
    if b.car.cancelled:
      self.enabled, b.car.cancelled = False, False
    self.updated = {'selfdriveState', 'carState', 'turnpilotPlan', 'modelV2'}
    ss = messaging.new_message('selfdriveState', valid=True)
    ss.selfdriveState.enabled = self.enabled
    self.msgs['selfdriveState'] = ss
    self.msgs['carState'] = messaging.new_message('carState', valid=True)
    tp = messaging.new_message('turnpilotPlan', valid=True)
    tp.turnpilotPlan.state = custom.TurnpilotPlan.State.holding if holding else custom.TurnpilotPlan.State.following
    if self.hold is not None and not holding and self.s < self.seg[1] + 10.:
      tp.turnpilotPlan.desire = log.Desire.turnRight
    self.msgs['turnpilotPlan'] = tp
    self.msgs['modelV2'] = messaging.new_message('modelV2', valid=True)
    if self.clock.t >= self.next_nav:  # navd at 1 Hz
      self.next_nav = self.clock.t + 1.0
      routed = self.route_t is not None and self.clock.t >= self.route_t
      nav = messaging.new_message('navInstruction', valid=routed)
      if routed:
        to_turn, left = float(self.seg[1] - self.s), float(self.seg[-1] - self.s)
        n = nav.navInstruction
        n.maneuverType, n.maneuverModifier = ('turn', 'right') if to_turn > -10 else ('arrive', '')
        n.maneuverDistance = to_turn if to_turn > -10 else left
        n.distanceRemaining = left
      route = messaging.new_message('navRoute', valid=True)
      route.navRoute.coordinates = [{'latitude': 39.0, 'longitude': -86.0}, {'latitude': 39.1, 'longitude': -86.1}] if routed else []
      self.msgs['navInstruction'], self.msgs['navRoute'] = nav, route
      self.updated |= {'navInstruction', 'navRoute'}


class FakeSM:
  def __init__(self, world):
    self.world = world
    self.seen = dict.fromkeys(run.SERVICES, False)
    self.updated = dict.fromkeys(run.SERVICES, False)
    self.valid = dict.fromkeys(run.SERVICES, True)
    self.recv_time = dict.fromkeys(run.SERVICES, 0.)

  def update(self, timeout=0):
    for s in run.SERVICES:
      self.updated[s] = s in self.world.updated
      if self.updated[s]:
        self.seen[s] = True
        self.recv_time[s] = self.world.clock.t
        self.valid[s] = self.world.msgs[s].valid
    self.world.updated = set()

  def __getitem__(self, s):
    m = self.world.msgs.get(s) or messaging.new_message(s)
    return getattr(m.as_reader(), s)


def bridge():
  car = SimpleNamespace(cancelled=False)
  car.cancel = lambda: setattr(car, 'cancelled', True)
  b = SimpleNamespace(simulator_state=SimpleNamespace(), car=car, blinker=(False, False), auto_engage=False, rk_frame=0,
                      set_speed_kph=0.0, rolling_start_kph=0.0, cfg=SimpleNamespace(capture=''), teleports=[], done=False)
  pose = SimpleNamespace(pos=(1.0, 2.0, 3.0), fwd=(1.0, 0.0, 0.0), up=(0.0, 0.0, 1.0))
  b.world = SimpleNamespace(capture=None, pose=pose, teleport=lambda pos, rot: b.teleports.append(pos))
  b.shutdown = lambda: setattr(b, 'done', True)
  return b


# the fake world plays no turnpilotd and no detector, so each scenario fails on what only they would do
CASES = [
  ('ped_crosswalk_right', 2, 'walk', "never held to a stop for the pedestrian in the crosswalk"),
  ('unprotected_left', 1, 'release', "no oncoming car came within 30 m of the junction while the car was at it: the wait went untested"),
  ('occupied_lane', 1, 'release_pacer', "the planner never saw the target lane as occupied"),
]


class FakeParams:
  def __init__(self, clock):
    self.clock, self.writes = clock, []

  def put_bool(self, key, value):
    self.writes.append((round(self.clock.t, 2), key, value))


def test_each_run_starts_turnpilot_afresh(tmp_path, monkeypatch):
  # BeamNG unprotected_left_gap6 (2026-10-28): the teleport back to the spawn left turnpilotd's route follower dead-reckoning
  # the last run's junction (gaps are dead-reckoned through), so the left's hold latched 13 m early. Each run turns TurnPilot
  # off at its start and on again RESET_S later, before its destination goes out: a fresh follower, latch and left
  clock = Clock()
  plan = catalog.build('unprotected_left')
  world = World(plan, clock)
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  monkeypatch.setattr(run, 'wall', clock.wall)
  monkeypatch.setattr(run, 'Director', FakeDirector)
  monkeypatch.setattr(FakeDirector, 'clock', clock)
  monkeypatch.setattr(run, 'web_call', world.web)
  monkeypatch.setattr(run.messaging, 'SubMaster', lambda services: FakeSM(world))
  params = FakeParams(clock)
  runner = run.Runner(plan, 2, str(tmp_path), 'http://127.0.0.1:8088', 'tok', 'host', 0, params=params)
  b = bridge()
  b.world.teleport = lambda pos, rot: (b.teleports.append(pos), setattr(world, 's', 0.0), setattr(world, 'hold', None))
  b.simulator_state.position, b.simulator_state.bearing, b.simulator_state.wheel_speed = SimpleNamespace(x=0, y=0, z=0), 0.0, 0.0
  posts = []
  web = world.web

  def recording_web(url, token, method, body=None):
    posts.append((round(clock.t, 2), method))
    return web(url, token, method, body)
  monkeypatch.setattr(run, 'web_call', recording_web)
  for _ in range(int(200 / DT)):
    world.step(b)
    if runner(b):
      break
    b.rk_frame += 1
    clock.t += DT
  offs = [t for t, k, v in params.writes if k == 'TurnPilotEnabled' and not v]
  ons = [t for t, k, v in params.writes if k == 'TurnPilotEnabled' and v]
  assert len(offs) == len(ons) == 2
  for off, on in zip(offs, ons, strict=True):
    assert on - off >= run.TP_RESET_S - 1e-6
    post = next(t for t, m in posts if m == 'POST' and t >= off)
    assert on <= post


@pytest.mark.parametrize("name,runs,trigger,failure", CASES)
def test_runs_end_to_end(tmp_path, monkeypatch, name, runs, trigger, failure):
  clock = Clock()
  plan = catalog.build(name)
  world = World(plan, clock)
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  monkeypatch.setattr(run, 'wall', clock.wall)
  monkeypatch.setattr(run, 'Director', FakeDirector)
  monkeypatch.setattr(FakeDirector, 'clock', clock)
  monkeypatch.setattr(run, 'web_call', world.web)
  monkeypatch.setattr(run.messaging, 'SubMaster', lambda services: FakeSM(world))
  runner = run.Runner(plan, runs, str(tmp_path), 'http://127.0.0.1:8088', 'tok', 'host', 0)
  b = bridge()
  b.world.teleport = lambda pos, rot: (b.teleports.append(pos), setattr(world, 's', 0.0), setattr(world, 'hold', None))
  b.simulator_state.position, b.simulator_state.bearing, b.simulator_state.wheel_speed = SimpleNamespace(x=0, y=0, z=0), 0.0, 0.0
  for _ in range(int(200 / DT)):
    world.step(b)
    if runner(b):
      break
    b.rk_frame += 1
    clock.t += DT
  assert b.done and len(b.teleports) == runs
  out = json.load(open(tmp_path / 'result.json'))
  assert [r['outcome'] for r in out['runs']] == ['arrived'] * runs
  names = [c[0] for c in runner.director.calls]
  actors = sum(a.trigger == {'walk': 'walk', 'release': 'oncoming', 'release_pacer': 'pace'}[trigger] for a in plan.actors)
  assert names.count('setup') == runs and names.count(trigger) == runs * actors and names.count('cleanup') >= runs and names[-1] == 'close'
  assert [c[1][2] for c in runner.director.calls if c[0] == 'web'] == ['DELETE', 'POST'] * runs
  assert out['road'] == plan.road
  # the actors go on the car at the junction: the walker on its go, the oncoming cars a set time after it stands there
  d = runner.director
  assert len(world.holds) == runs
  for (fn, args), t in zip(d.calls, d.times, strict=True):
    if fn == 'walk':
      assert any(end <= t <= end + 0.1 for _, end in world.holds), (t, world.holds)
    elif fn == 'release':
      delay = next(a for a in plan.actors if a.name == args[0]).params['release_after_stop_s']
      assert any(start + delay <= t <= start + delay + 0.1 for start, _ in world.holds), (t, world.holds, delay)
  for r in out['runs']:
    assert r['maneuvers_attempted'] == 1 and not r['disengagements']
    assert r['wrong_side_m'] == 0.0  # checked against the plan's road model, saved beside the runs for a rescore
    assert r['failures'][0] == failure
    if name == 'unprotected_left':  # no turnpilotd here: the left never went by itself
      assert "never went at the unprotected left" in r['failures']
    gap = 'ped_gap' if name.startswith('ped') else 'car_gap'
    assert any(row[gap] is not None for row in r['rows'])
    assert r['pose_fields'] == run.POSE_FIELDS and len(r['poses']) > 20 * 10
    assert r['poses'][0][0] <= r['rows'][0]['wall']  # from the teleport on, not only once engaged
    assert set(r['actor_poses']) == {a.name for a in plan.actors}
    for a in r['actor_poses'].values():
      assert len(a['rows'][0]) == len(run.ACTOR_FIELDS)


def test_a_plan_without_a_road_model_is_refused():
  plan = catalog.build('unprotected_left')
  plan.road = None
  with pytest.raises(ValueError, match="no road model"):
    run.Runner(plan, 1, '/tmp', 'http://127.0.0.1:8088', 'tok', 'host', 0)


def test_driver_taps_the_gas_where_turnpilot_asks_for_a_go(monkeypatch):
  # held at an unprotected turn's line with its go prompt (G1's, kept for the left's announce mode), the scenario driver gives
  # it, logged; a stop sign is never tapped (SG4c: the decider goes by itself or hands back; test_driver has that)
  from openpilot.tools.sim.bridge.beamng.driver import TAP_GAS, GasTaps
  clock = Clock()
  runner = run.Runner(catalog.build('ped_crosswalk_right'), 1, '/tmp', 'http://127.0.0.1:8088', 'tok', 'host', 0)
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  runner.taps = GasTaps()
  msgs = {}
  ss = messaging.new_message('selfdriveState')
  ss.selfdriveState.enabled = True
  tp = messaging.new_message('turnpilotPlan')
  tp.turnpilotPlan.controlPrompt, tp.turnpilotPlan.state, tp.turnpilotPlan.shouldStop = 'unprotectedTurnGo', 'holding', True
  msgs['selfdriveState'], msgs['turnpilotPlan'] = ss, tp
  runner.sm = type('SM', (), {'seen': {'turnpilotPlan': True, 'turnpilotLights': False}, 'recv_time': {'turnpilotPlan': clock.t},
                              '__getitem__': lambda self, s: getattr(msgs[s].as_reader(), s)})()
  b = bridge()
  b.simulator_state.wheel_speed = 0.
  gas = []
  for _ in range(60):
    runner.driver(b)
    gas.append(b.driver_gas)
    clock.t += 0.05
  assert max(gas) == 0.  # turnpilotd went quiet: its last prompt is no go
  gas = []
  for _ in range(60):
    runner.sm.recv_time['turnpilotPlan'] = clock.t  # fresh at 20 Hz
    runner.driver(b)
    gas.append(b.driver_gas)
    clock.t += 0.05
  assert TAP_GAS in gas and runner.taps.log[0]['reason'] == 'unprotectedTurnGo'


def row_runner(monkeypatch, name):
  """A runner on the scenario's plan, its car at the spawn heading for the junction, for a row taken mid-run."""
  clock = Clock()
  plan = catalog.build(name)
  world = World(plan, clock)
  monkeypatch.setattr(run, 'wall', clock.wall)
  runner = run.Runner(plan, 1, '/tmp', 'http://127.0.0.1:8088', 'tok', 'host', 0)
  runner.sm, runner.director, runner.t_engaged, runner.route_id = FakeSM(world), FakeDirector(), clock.t, 'r'
  b = bridge()
  world.step(b)
  runner.sm.update()
  return runner, b, plan


def test_a_row_on_a_plan_without_actors():
  """lane_change has a junction and no actors: its rows have no pacer and no gaps, and no run ends 'error' on them."""
  with pytest.MonkeyPatch.context() as mp:
    runner, b, plan = row_runner(mp, 'lane_change')
    assert not plan.actors and plan.junction is not None
    r = runner.row(b, runner.t_engaged + 1.0)
  assert r['pacer_dx'] is None and r['car_gap'] is None and r['j_ahead'] is not None


def test_a_row_once_the_pacer_is_gone():
  """The director drops the pacer's pose when its script ends, while the car may still be driving: the row says so."""
  with pytest.MonkeyPatch.context() as mp:
    runner, b, plan = row_runner(mp, 'occupied_lane')
    runner.director.setup(plan.actors)
    r = runner.row(b, runner.t_engaged + 1.0)
    pacer = next(a for a in plan.actors if a.trigger == 'pace')
    assert r['pacer_dx'] is not None
    runner.director.poses.pop(pacer.name)
    r = runner.row(b, runner.t_engaged + 1.0)
  assert r['pacer_dx'] is None


def test_the_pacer_dx_is_the_pacers():
  """With an actor after the pacer in the plan, pacer_dx is still the pacer's, not the last actor's."""
  with pytest.MonkeyPatch.context() as mp:
    runner, b, plan = row_runner(mp, 'occupied_lane')
    pacer = next(a for a in plan.actors if a.trigger == 'pace')
    other = dataclasses.replace(pacer, name='other', trigger='setup', path=[(p[0] + 500., p[1], p[2]) for p in pacer.path])
    plan.actors = [pacer, other]
    runner.director.setup(plan.actors)
    r = runner.row(b, runner.t_engaged + 1.0)
  x, y = b.simulator_state.position.x, b.simulator_state.position.y
  h = math.radians(90.0 - b.simulator_state.bearing)
  p = runner.director.poses[pacer.name]
  assert r['pacer_dx'] == round((p[0] - x) * math.cos(h) + (p[1] - y) * math.sin(h), 2)


def test_lane_change_runs_end_to_end(tmp_path, monkeypatch):
  """The scenario without actors, through the whole run: it arrives, its rows taken on the way."""
  clock = Clock()
  plan = catalog.build('lane_change')
  world = World(plan, clock)
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
    clock.t += DT
  out = json.load(open(tmp_path / 'result.json'))
  assert [r['outcome'] for r in out['runs']] == ['arrived']
  assert out['runs'][0]['rows'] and all(row['pacer_dx'] is None for row in out['runs'][0]['rows'])
