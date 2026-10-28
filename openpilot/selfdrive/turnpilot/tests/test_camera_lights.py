"""The camera light source in map_controls (DESIGN phase 4 G2): shadow logs what it would do and never acts; add-only may
only ever add a stop, never raise an accel candidate or remove one, and never past -2.0 m/s^2."""
import random
from dataclasses import replace

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import map_controls
from openpilot.selfdrive.turnpilot.map_controls import (CAMERA_STALE_S, GREEN_STEADY_S, MAP_DECEL_MAX, NO_LIGHTS, CameraLights, Lights,
                                                         MapControlPolicy, camera_light_input)
from openpilot.selfdrive.turnpilot.scene_policy import decide
from openpilot.selfdrive.turnpilot.tests.test_map_controls import Car, held, lights, oracle

STATES = ('red', 'yellow', 'green', 'unknown')


def camera(state, distance, shadow=True, status='read'):
  return CameraLights([(state, distance)], shadow, status)


def cam_at(state, past=20., shadow=True, oracle_state=None):
  """The camera reading the first control's head `past` m beyond its line, with the sim oracle's light if given."""
  def make(car):
    d = car.controls[0][0] - car.s
    base = lights(oracle_state, d) if oracle_state is not None else Lights()
    return replace(base, camera=camera(state, d + past, shadow))
  return make


class Events:
  def __init__(self, monkeypatch):
    self.rows = []
    monkeypatch.setattr(map_controls.cloudlog, 'event', lambda name, **kw: self.rows.append((name, kw)))

  def named(self, name):
    return [kw for n, kw in self.rows if n == name]


class Light:
  def __init__(self, state, distance, governs=True):
    self.state, self.distance, self.governs = state, distance, governs


class Msg:
  def __init__(self, lights, shadow=True, source='camera'):
    self.lights, self.shadow, self.source = lights, shadow, source


def test_camera_light_input():
  msg = Msg([Light('red', 40.), Light('green', 45., governs=False)])
  cam = camera_light_input(msg, 'read', 0.1, 'shadow')
  assert cam.heads == [('red', 40.)] and cam.shadow and cam.status == 'read'  # governing heads only
  assert camera_light_input(msg, 'read', CAMERA_STALE_S + 0.1, 'shadow') is None  # stale: nothing
  assert camera_light_input(None, 'read', 0., 'shadow') is None
  assert camera_light_input(msg, 'noCalibration', 0.1, 'shadow') is None
  assert camera_light_input(msg, 'read', 0.1, 'off') is None
  off = camera_light_input(Msg([]), 'disabled', 0.1, 'shadow')
  assert off.status == 'disabled' and off.heads == []
  # add-only takes both keys: modeld's message not shadow, and turnpilotd's own param addOnly
  assert camera_light_input(Msg([Light('red', 40.)], shadow=False), 'read', 0.1, 'shadow').shadow
  assert camera_light_input(Msg([Light('red', 40.)], shadow=True), 'read', 0.1, 'addOnly').shadow
  assert not camera_light_input(Msg([Light('red', 40.)], shadow=False), 'read', 0.1, 'addOnly').shadow


def test_shadow_camera_red_changes_nothing_and_says_what_it_would_do(monkeypatch):
  ev = Events(monkeypatch)
  base, car = Car([(150., 'signal')]), Car([(150., 'signal')])
  for _ in range(int(20. / DT_MDL)):
    a = base.step()
    b = car.step(lights=cam_at('red'))
    assert a[0].stops == b[0].stops and a[2] == b[2] and a[0].alert == b[0].alert == 'none'
  assert car.s > 200. and min(r[2] for r in car.rows) >= 13.4 - 1e-6  # through at speed
  would = ev.named('turnpilot.camera_light_shadow')
  assert would and would[0]['camera'] == 'red' and would[0]['would'] in ('stop', 'stopFirm')
  assert would[0]['acting'] is False and would[0]['policy_light'] == 'unknown'
  assert len(would) < 6  # on a change, not every frame


def test_shadow_camera_logs_an_unmapped_red_and_does_nothing(monkeypatch):
  ev = Events(monkeypatch)
  car = Car([])
  out, dec, _ = car.run(3., lights=lambda car: Lights(camera=camera('red', 40.)))
  assert out.stops == [] and not dec.should_stop
  assert len(ev.named('turnpilot.camera_light_unmapped')) == 1


def test_camera_light_reader_off_is_said_once():
  car = Car([(300., 'signal')])
  off = Lights(camera=CameraLights([], True, 'disabled'))
  out, dec, _ = car.step(lights=off)
  assert out.alert == 'lightSourceRejected' and 'Camera light reader off' in out.reason
  out, dec, _ = car.run(10., lights=off)
  assert out.alert == 'none'  # once for the drive, not a status line for the rest of it


def test_a_camera_source_is_never_rejected():
  car = Car([(300., 'signal')])
  out, dec, _ = car.step(lights=Lights(camera=camera('red', 40.)))
  assert out.alert == 'none'


# --- add-only (TurnPilotCameraLights addOnly, not on this week) ---

def test_add_only_camera_red_stops_at_the_line_never_below_minus_2():
  car = Car([(150., 'signal')])
  out, dec = car.run_until(held, lights=cam_at('red', shadow=False))
  assert -2. < 150. - car.s < 5. and all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)


def test_add_only_camera_never_releases_an_oracle_red():
  # held on the oracle's red; the camera reads green (or a phantom red over the oracle's green holds it on)
  car = Car([(150., 'signal')])
  car.run_until(held, lights=oracle('red'))
  out, dec, _ = car.run(3., lights=cam_at('green', shadow=False, oracle_state='red'))
  assert held(out, dec)
  out, dec, _ = car.run(GREEN_STEADY_S + 1., lights=cam_at('red', shadow=False, oracle_state='green'))
  assert held(out, dec)  # the camera added this hold: only its red going lets the oracle's green release it
  out, dec, _ = car.run(GREEN_STEADY_S + 0.1, lights=cam_at('unknown', shadow=False, oracle_state='green'))
  assert not dec.should_stop


def test_add_only_camera_green_adds_nothing_and_unknown_nothing():
  for state in ('green', 'unknown'):
    car = Car([(150., 'signal')])
    out, dec, _ = car.run(20., lights=cam_at(state, shadow=False))
    assert car.s > 200. and min(r[2] for r in car.rows) >= 13.4 - 1e-6


# --- the property: random drives, the camera never raises any candidate or removes a stop ---

def random_camera(rng, d_signal):
  status = rng.choice(['read'] * 6 + ['disabled', 'noCalibration'])
  if status != 'read':
    return CameraLights([], rng.random() < 0.5, status)
  heads = [(rng.choice(STATES), d_signal + rng.uniform(-30., 60.)) for _ in range(rng.randint(0, 3))]
  heads += [(rng.choice(STATES), rng.uniform(0., 150.)) for _ in range(rng.randint(0, 1))]  # anywhere, unmapped too
  return CameraLights(heads, rng.random() < 0.5, status)


def lockstep(seed: int, add_only: bool, seconds: float = 30.):
  """A car driven by a policy fed the sim oracle plus random camera lights, and the baseline policy fed the same frames
  with the oracle alone. The car follows the camera's policy: where it holds, the car is held (as on the road)."""
  rng = random.Random(seed)
  signals = sorted(rng.uniform(60., 300.) for _ in range(rng.randint(1, 3)))
  car = Car([(s, 'signal') for s in signals] + ([(rng.uniform(60., 300.), 'stop')] if rng.random() < 0.3 else []),
            v=rng.uniform(5., 20.))
  car.controls.sort()
  base_policy = MapControlPolicy()
  oracle_state, cam = rng.choice(STATES), None
  for frame in range(int(seconds / DT_MDL)):
    if frame % 10 == 0:
      oracle_state = rng.choice(STATES) if rng.random() < 0.3 else oracle_state
    d0 = next((s for s, k in car.controls if k == 'signal' and s > car.s - 5.), signals[-1]) - car.s
    base = lights(oracle_state, d0) if oracle_state != 'unknown' else NO_LIGHTS
    if frame % 2 == 0:
      cam = replace(random_camera(rng, d0), shadow=not add_only)
    due = car.frame % 20 == 0
    nav = car.nav() if due else None
    v = car.v
    out_a = base_policy.update(nav, due, True, v, False, base, DT_MDL)
    out_b, _, _ = car.step(lights=replace(base, camera=cam))
    yield v, out_a, decide(out_a.stops, v), out_b, decide(out_b.stops, v)  # unramped, both


@pytest.mark.parametrize("seed", range(40))
def test_shadow_camera_never_changes_a_plan(seed):
  for _, a, _, b, _ in lockstep(seed, add_only=False):
    assert a.stops == b.stops and a.prompt == b.prompt and a.released_stopped == b.released_stopped
    assert b.alert in (a.alert, 'lightSourceRejected')  # the reader's loud off, once


@pytest.mark.parametrize("seed", range(40))
def test_add_only_camera_never_raises_a_candidate_or_removes_a_stop(seed):
  for v, a, dec_a, b, dec_b in lockstep(seed, add_only=True):
    assert dec_b.accel_cap <= dec_a.accel_cap + 1e-9 and dec_b.speed_cap <= dec_a.speed_cap + 1e-9, (v, a.stops, b.stops)
    assert dec_b.should_stop or not dec_a.should_stop
    added = [s for s in b.stops if s not in a.stops]
    assert all(s.accel_cap >= MAP_DECEL_MAX - 1e-9 for s in added)  # what it adds is never past -2.0


# --- through turnpilotd: the camera's lights from turnpilotDetections ---

def detections(state, distance, shadow=True, status='read', camera='road', governs=True):
  from openpilot.cereal import messaging
  msg = messaging.new_message('turnpilotDetections')
  td = msg.turnpilotDetections
  td.camera, td.lightStatus = camera, status
  if status == 'read' and camera == 'road':
    td.lights.source, td.lights.shadow = 'camera', shadow
    td.lights.lights = [{'state': state, 'distance': distance, 'governs': governs, 'votes': 4, 'conf': 0.7}]
  return msg.as_reader().turnpilotDetections


def camera_sim(mode, make):
  from openpilot.cereal import messaging
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(200., 'signal')])
  sim.tp.camera_mode = mode
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights  # no oracle (source none)

  def lights(sim):
    # Sim sends turnpilotLights each frame after it resets sm.updated: the detections ride along here, a road camera run
    # every other frame, as modeld does
    if make is not None and int(round(sim.t / DT_MDL)) % 2 == 0:
      sim.sm.put('turnpilotDetections', make(sim))
    return none
  sim.lights = lights
  return sim


def test_turnpilotd_shadow_camera_red_never_changes_the_plan(monkeypatch):
  ev = Events(monkeypatch)
  base, sim = camera_sim('shadow', None), camera_sim('shadow', lambda sim: detections('red', 220. - sim.s, shadow=True))
  for _ in range(int(25. / DT_MDL)):
    a, b = base.step(), sim.step()
    assert (a.accelCap, a.speedCap, a.shouldStop, a.controlAlert, a.controlPrompt, a.reason) == \
      (b.accelCap, b.speedCap, b.shouldStop, b.controlAlert, b.controlPrompt, b.reason)
  assert sim.s > 300. and min(v for _, v, _ in sim.trace) >= 13. - 1e-6
  assert any(kw['camera'] == 'red' for kw in ev.named('turnpilot.camera_light_shadow'))


def test_turnpilotd_add_only_needs_both_keys():
  # modeld's message not shadow, but turnpilotd's own param shadow: nothing acts
  sim = camera_sim('shadow', lambda sim: detections('red', 220. - sim.s, shadow=False))
  sim.run(25.)
  assert sim.s > 300. and not any(p.shouldStop for _, _, p in sim.trace)
  # both: the camera's red stops the car at the line, never braking past -2.0
  sim = camera_sim('addOnly', lambda sim: detections('red', 220. - sim.s, shadow=False))
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import stop_at
  plan = stop_at(sim, 200.)
  assert plan.shouldStop and -2. < 200. - sim.s < 5. and all(p.accelCap >= MAP_DECEL_MAX - 1e-6 for _, _, p in sim.trace)


def test_turnpilotd_keeps_the_last_road_read_through_wide_runs_and_ages_it_out():
  sim = camera_sim('addOnly', lambda sim: detections('red', 220. - sim.s, shadow=False) if sim.t < 1. else
                   detections('unknown', 0., camera='wide', status='wideCamera'))
  sim.run(1. + CAMERA_STALE_S - 0.2)
  assert sim.tp.map_update(sim.sm, sim.v, False).shown.light_state == 'red'  # the road read still holds
  sim.run(0.5)
  # aged out: the camera says nothing; the stop it added stays until an accepted green (map_controls docstring)
  assert sim.tp.map_update(sim.sm, sim.v, False).shown.light_state == 'unknown'
  assert sim.tp.map_controls.stop_ahead() is not None


def test_turnpilotd_says_a_camera_reader_off_once():
  sim = camera_sim('shadow', lambda sim: detections('unknown', 0., status='disabled'))
  sim.run(10.)
  alerts = [p.controlAlert for _, _, p in sim.trace]
  assert 'lightSourceRejected' in alerts and alerts[-1] == 'none'
  assert any(p.reason == "Camera light reader off for this drive" for _, _, p in sim.trace if p.controlAlert == 'lightSourceRejected')
