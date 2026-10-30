"""The runner's demo options, dry (test_runner's fake bridge, director, SubMaster and clock): the driver taking over a
handback (--end-after-handback), the driver's one gas press (--gas-press-at), and the sim's detection drop
(--drop-detections-at), each with the expectation it implies, and straight_cruise's timing for them."""
import json
import math
import os
from types import SimpleNamespace

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.selfdrived.state import SOFT_DISABLE_TIME
from openpilot.selfdrive.turnpilot.turnpilotd import DETECTOR_DEADLINE_S
from openpilot.tools.sim.bridge.beamng.roadgraph import GRAPH_JSON
from openpilot.tools.sim.bridge.beamng.scenarios import catalog, run
from openpilot.tools.sim.bridge.beamng.scenarios.tests.test_runner import DT, Clock, FakeDirector, FakeSM, World, bridge

pytestmark = pytest.mark.skipif(not os.path.exists(GRAPH_JSON), reason="needs piston's road-graph export")


class DriverWorld(World):
  """test_runner's World, with openpilot's answer to the driver's pedals: the brake disengages, and so does the gas
  (DisengageOnAccelerator, on as on the car). carState shows both. take_control: at the stop line TurnPilot holds the car
  for good and shows this take-control prompt from a second into the stand."""
  def __init__(self, plan, clock, take_control: str = ''):
    super().__init__(plan, clock)
    self.take_control = take_control

  def step(self, b):
    gas, brake = getattr(b, 'driver_gas', 0.) > 0., getattr(b, 'driver_brake', 0.) > 0.
    if self.take_control and self.hold is not None:
      self.hold = (self.hold[0], math.inf)
    super().step(b)
    if gas or brake:
      self.enabled = False
      self.msgs['selfdriveState'].selfdriveState.enabled = False
    cs = messaging.new_message('carState', valid=True)
    cs.carState.gasPressed, cs.carState.brakePressed = gas, brake
    self.msgs['carState'] = cs
    if self.take_control and self.hold is not None and self.clock.t >= self.hold[0] + 1.0:
      tp = self.msgs['turnpilotPlan'].turnpilotPlan
      tp.controlPrompt = self.take_control
      tp.reason = 'pedestrian in crosswalk'


class DetectorWorld(DriverWorld):
  """DriverWorld with a detector: turnpilotDetections every step until the drop's param is written, silent after."""
  def __init__(self, plan, clock, take_control: str = '', params=None):
    super().__init__(plan, clock, take_control)
    self.params, self.silent_t = params, None

  def step(self, b):
    super().step(b)
    if self.silent_t is None and any(k == run.DROP_PARAM and v for _, k, v in self.params.writes):
      self.silent_t = self.clock.t
    if self.silent_t is None:
      self.msgs['turnpilotDetections'] = messaging.new_message('turnpilotDetections', valid=True)
      self.updated.add('turnpilotDetections')


class FakeParams:
  def __init__(self, clock, known=True):
    self.clock, self.known, self.writes = clock, known, []

  def put_bool(self, key, value):
    self.writes.append((round(self.clock.t, 2), key, value))

  def put(self, key, value):
    if not self.known:
      raise UnknownKeyName(key)
    self.writes.append((round(self.clock.t, 2), key, value))

  def remove(self, key):
    if not self.known:
      raise UnknownKeyName(key)
    self.writes.append((round(self.clock.t, 2), key, None))


def drive(tmp_path, monkeypatch, name, take_control='', runs=1, params=None, world_cls=None, **options):
  clock = Clock()
  plan = catalog.build(name)
  params = params(clock) if callable(params) else params
  world = DriverWorld(plan, clock, take_control) if world_cls is None else world_cls(plan, clock, take_control, params)
  monkeypatch.setattr(run, 'time', SimpleNamespace(monotonic=clock.monotonic, strftime=lambda *a: 'now'))
  monkeypatch.setattr(run, 'wall', clock.wall)
  monkeypatch.setattr(run, 'Director', FakeDirector)
  monkeypatch.setattr(FakeDirector, 'clock', clock)
  monkeypatch.setattr(run, 'web_call', world.web)
  monkeypatch.setattr(run.messaging, 'SubMaster', lambda services: FakeSM(world))
  runner = run.Runner(plan, runs, str(tmp_path), 'http://127.0.0.1:8088', 'tok', 'host', 0, params=params, **options)
  b = bridge()
  b.world.teleport = lambda pos, rot: (b.teleports.append(pos), setattr(world, 's', 0.0), setattr(world, 'hold', None))
  b.simulator_state.position, b.simulator_state.bearing, b.simulator_state.wheel_speed = SimpleNamespace(x=0, y=0, z=0), 0.0, 0.0
  gas, brake = [], []
  for _ in range(int(200 / DT)):
    world.step(b)
    if runner(b):
      break
    gas.append((round(clock.t, 2), getattr(b, 'driver_gas', 0.)))
    brake.append((round(clock.t, 2), getattr(b, 'driver_brake', 0.)))
    b.rk_frame += 1
    clock.t += DT
  out = json.load(open(tmp_path / 'result.json'))
  return out, runner, gas, brake, params


def test_the_driver_takes_over_a_handback_after_n_seconds(tmp_path, monkeypatch):
  # S03: the stall handback holds the car engaged; without the hook the run stands to its timeout (130 s measured)
  out, runner, _, brake, _ = drive(tmp_path, monkeypatch, 'ped_crosswalk_right', take_control='stalledTakeControl',
                                   end_after_handback=3., expect=('handback:stalledTakeControl',))
  (r,) = out['runs']
  assert r['outcome'] == 'taken over' and r['disengagements'][0]['cause'] == 'driver brake'
  first = next(row['t'] for row in r['rows'] if row['control_prompt'] == 'stalledTakeControl')
  assert r['driver_takeover']['after'] == 'stalledTakeControl'
  assert r['driver_takeover']['t'] == pytest.approx(first + 3., abs=0.06)
  assert r['duration_s'] < 60.
  assert r['expect'] == out['expect'] == ['handback:stalledTakeControl']
  assert r['passed'], r['failures']
  assert brake[-1][1] == 0.  # off again once the run ended


def test_without_the_hook_a_handback_stands_engaged(tmp_path, monkeypatch):
  out, *_ = drive(tmp_path, monkeypatch, 'ped_crosswalk_right', take_control='stalledTakeControl')
  (r,) = out['runs']
  assert r['outcome'] == 'timeout' and r.get('driver_takeover') is None


def test_one_gas_press_disengages_and_is_what_the_run_expects(tmp_path, monkeypatch):
  # S04: 0.3 gas for 0.5 s at 3 s after engagement (straight_cruise's fault_at_s, the option without a value)
  out, runner, gas, _, _ = drive(tmp_path, monkeypatch, 'straight_cruise', gas_press_at=-1.)
  (r,) = out['runs']
  assert r['outcome'] == 'disengaged' and r['disengagements'][0]['cause'] == 'driver gas'
  assert r['gas_press']['t'] == pytest.approx(catalog.SCENARIOS['straight_cruise'].fault_at_s, abs=0.06)
  pressed = [t for t, g in gas if g == run.GAS_PRESS]
  assert pressed and pressed[-1] - pressed[0] <= run.GAS_PRESS_S
  assert out['expect'] == ['gasDisengage'] and r['passed'], r['failures']


def test_the_detection_drop_is_set_once_and_cleared(tmp_path, monkeypatch):
  # S01: the param the detector honours only in the sim, set 3 s after engagement, cleared at the run's start and end
  out, runner, _, _, params = drive(tmp_path, monkeypatch, 'straight_cruise', drop_detections_at=-1., params=FakeParams)
  (r,) = out['runs']
  drops = [(t, v) for t, k, v in params.writes if k == run.DROP_PARAM]
  assert [v for _, v in drops] == [None, run.DROP_S, None]
  assert r['detection_drop']['t'] == pytest.approx(catalog.SCENARIOS['straight_cruise'].fault_at_s, abs=0.06)
  assert out['expect'] == ['handback:detectorStopped']
  assert r['outcome'] == 'arrived'  # the fake world has no detector to drop: the expectation fails it
  assert "expected handback detectorStopped: none came" in r['failures']
  assert all(row['det_age'] is None for row in r['rows'])  # nor any detections


def test_the_rows_carry_the_detectors_silence_from_the_drop(tmp_path, monkeypatch):
  # the S01 shot's clock: the rows' det_age, from the runner's own turnpilotDetections, runs from the drop's write
  out, *_ = drive(tmp_path, monkeypatch, 'straight_cruise', drop_detections_at=-1., params=FakeParams, world_cls=DetectorWorld)
  (r,) = out['runs']
  drop_t = r['detection_drop']['t']
  before = [row['det_age'] for row in r['rows'] if row['t'] < drop_t - 0.1]
  assert before and max(before) <= 0.06
  last = r['rows'][-1]
  assert last['t'] - last['det_age'] == pytest.approx(drop_t, abs=0.06)


def test_a_drop_the_run_never_reached_is_said_and_fails(tmp_path, monkeypatch):
  out, *_ = drive(tmp_path, monkeypatch, 'straight_cruise', drop_detections_at=500., params=FakeParams)
  (r,) = out['runs']
  assert r['detection_drop'] == {'t': None, 'at': 500.} and not r['passed']
  assert "the detection drop asked for at t=500.0 s never came (the run ended first)" in r['failures'], r['failures']


def test_a_build_without_the_param_ends_the_run_loudly(tmp_path, monkeypatch):
  out, *_ = drive(tmp_path, monkeypatch, 'straight_cruise', drop_detections_at=3., params=lambda c: FakeParams(c, known=False))
  (r,) = out['runs']
  assert r['outcome'] == 'error' and 'rebuild' in r['detail']


def test_the_drop_needs_params_and_an_option_without_seconds_needs_the_scenarios():
  plan = catalog.build('straight_cruise')
  with pytest.raises(ValueError, match="params"):
    run.Runner(plan, 1, '/tmp', 'http://127.0.0.1:8088', 'tok', 'host', 0, drop_detections_at=3.)
  with pytest.raises(ValueError, match="fault_at_s"):
    run.Runner(catalog.build('ped_in_path'), 1, '/tmp', 'http://127.0.0.1:8088', 'tok', 'host', 0, gas_press_at=-1.)


def test_straight_cruise_faults_land_short_of_the_junctions_line():
  # engaged after the roll and the stalk (catalog.spawn_clear_m's arithmetic), at the set speed from there on (the most
  # ground it can cover): the press disengages at once, the drop hands back DETECTOR_DEADLINE_S later and openpilot's
  # soft disable ends SOFT_DISABLE_TIME after that, all short of the junction's own stop line with room to spare
  plan = catalog.build('straight_cruise')
  e = catalog.SCENARIOS['straight_cruise']
  v_roll, v = catalog.ROLL_FRACTION * e.speed_kph / 3.6, e.speed_kph / 3.6
  engaged_m = plan.notes['approach_m'] - v_roll ** 2 / (2. * catalog.ROLL_ACCEL) - catalog.ENGAGE_LAG_S * v_roll
  disengaged_m = engaged_m - v * (e.fault_at_s + DETECTOR_DEADLINE_S + SOFT_DISABLE_TIME)
  assert plan.notes['junction_control'] == 'signal' and not plan.actors
  assert disengaged_m - plan.notes['stop_line_m'] >= 10., (engaged_m, disengaged_m, plan.notes)
  # and the road goes on straight past the junction to the destination, with no mapped control on it
  assert math.hypot(plan.dest[0] - plan.junction[0], plan.dest[1] - plan.junction[1]) > 100.


def test_the_plans_expect_and_the_options_reach_the_result(tmp_path, monkeypatch):
  out, runner, *_ = drive(tmp_path, monkeypatch, 'lane_change', expect=('turnMissed',))
  assert out['expect'] == ['turnMissed'] and "expected turnMissed: none came (no turnMissed)" in out['runs'][0]['failures']
  assert custom.TurnpilotPlan.Handback.detectorStopped  # the kind the drop's expectation names exists
