"""The camera light source in map_controls (DESIGN phase 4 G2): shadow logs what it would do and never acts; add-only may
only ever add a stop, never raise an accel candidate or remove one, and never past -2.0 m/s^2."""
import random
from dataclasses import replace

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import map_controls
from openpilot.selfdrive.turnpilot.map_controls import (CAMERA_OFF_AGAIN_S, CAMERA_OFF_REASON, CAMERA_OFF_STILL_REASON,
                                                         CAMERA_STALE_S, GREEN_STEADY_S, MAP_DECEL_MAX, NO_LIGHTS, CameraLights,
                                                         Lights, MapControlPolicy, camera_light_input)
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


def test_camera_light_reader_failing_is_a_status_line_for_its_whole_spell():
  # S2 (review 2026-11-07): a reader failing every read was said once for 2.0 s and never again (probe_camera_off: 40 frames in
  # 600 s), and for the rest of the drive no camera red could stop the car with nothing saying so. Its line stays up the whole
  # spell: the chimed onset's words for ABORT_HOLD_S, then the same line silent, down at the reader's first good read
  car = Car([])
  off = Lights(camera=CameraLights([], True, 'disabled'))
  out, dec, _ = car.step(lights=off)
  assert out.alert == 'lightSourceRejected' and out.reason == CAMERA_OFF_REASON and out.reader_line == 'onset'
  outs = [car.step(lights=off)[0] for _ in range(int(120. / DT_MDL))]
  assert all(o.alert == 'lightSourceRejected' for o in outs)
  onset = 1 + sum(o.reason == CAMERA_OFF_REASON for o in outs)
  assert abs(onset * DT_MDL - map_controls.ABORT_HOLD_S) <= 2 * DT_MDL
  assert outs[-1].reason == CAMERA_OFF_STILL_REASON and outs[-1].reader_line == 'still'
  # a read with no input at all (a sweep, the wide runs, a retry's backoff) is still the spell: only a good read ends it
  out, _, _ = car.run(5., lights=NO_LIGHTS)
  assert out.alert == 'lightSourceRejected' and out.reason == CAMERA_OFF_STILL_REASON
  out, _, _ = car.step(lights=Lights(camera=CameraLights([], False, 'read')))
  assert out.alert == 'none' and out.reader_line == '' and out.reason == ''


def test_a_reader_failing_before_the_driver_engages_is_said_at_the_engagement():
  # S2 (skeptic): the reader failing from the drive's start, parked, openpilot off: its 2 s ran down while nothing could show
  # it (turnpilotControlAlert is a WARNING, engaged only), and engaged at 10 s the driver heard nothing for the rest of the
  # drive. Its onset runs down only on frames it could be shown: said in full at the engagement, then the line stays
  car = Car([])
  for _ in range(int(10. / DT_MDL)):
    out, _, _ = car.step(lights=OFF, engaged=False)
    assert out.alert == 'lightSourceRejected' and out.reason == CAMERA_OFF_REASON
  outs = [car.step(lights=OFF, engaged=True)[0] for _ in range(int(5. / DT_MDL))]
  onset = [i for i, o in enumerate(outs) if o.reason == CAMERA_OFF_REASON]
  assert onset == list(range(len(onset))) and abs(len(onset) * DT_MDL - map_controls.ABORT_HOLD_S) <= 2 * DT_MDL
  assert all(o.alert == 'lightSourceRejected' for o in outs) and outs[-1].reason == CAMERA_OFF_STILL_REASON


def test_the_camera_light_readers_words_say_it_is_failing_not_off_for_the_drive():
  # detector-2 (2026-11-06): disabled is three failed reads in a row, still retried and read again at its first good read.
  # "Camera light reader off for this drive" was false from then on
  assert CAMERA_OFF_REASON == "Camera light reader failing: lights unknown"
  for text in (CAMERA_OFF_REASON, map_controls.__doc__, map_controls.camera_light_input.__doc__):
    assert 'off for this drive' not in text and 'off for the drive (disabled)' not in text
  assert 'no green in 667 reads, and no colour on any' in ' '.join(map_controls.__doc__.split())
  assert '667 governing heads' not in map_controls.__doc__ and "Saturday's log" not in map_controls.__doc__
  assert 'G2 is shadow' not in map_controls.MapControlPolicy.release_light.__doc__


def spell(car, lights, seconds):
  """seconds of car steps with lights: the frames the reader's failing line is said."""
  outs = [car.step(lights=lights)[0] for _ in range(int(round(seconds / DT_MDL)))]
  return [i for i, o in enumerate(outs) if o.alert == 'lightSourceRejected' and o.reason == CAMERA_OFF_REASON]


OFF = Lights(camera=CameraLights([], True, 'disabled'))
READING = Lights(camera=CameraLights([], False, 'read'))


def test_a_camera_light_reader_failing_again_later_in_the_night_is_said_again(monkeypatch):
  # a failing spell, the reader reading again for a minute, a second failing spell: said both times, each its full hold.
  # Before: said once for the drive, the second spell silent (and "off for this drive" untrue after the first read)
  ev = Events(monkeypatch)
  car = Car([])
  first = spell(car, OFF, 5.)
  back = spell(car, READING, CAMERA_OFF_AGAIN_S)
  second = spell(car, OFF, 5.)
  assert first and not back and second
  assert len(first) * DT_MDL >= map_controls.ABORT_HOLD_S - 2 * DT_MDL and len(second) == len(first)
  assert len(ev.named('turnpilot.camera_lights_off')) == 2 and len(ev.named('turnpilot.camera_lights_back')) == 1


def test_a_camera_light_reader_never_reading_again_is_never_said_again(monkeypatch):
  # disabled, then no read at all for over a minute (a sweep, wide runs, a retry's backoff: no camera input), then disabled:
  # one spell, said once. Only a good read ends a spell
  ev = Events(monkeypatch)
  car = Car([])
  first = spell(car, OFF, 3.)
  gap = spell(car, NO_LIGHTS, CAMERA_OFF_AGAIN_S + 5.)
  again = spell(car, OFF, 5.)
  assert first and not gap and not again
  assert len(ev.named('turnpilot.camera_lights_off')) == 1 and not ev.named('turnpilot.camera_lights_back')


def test_a_camera_light_reader_flapping_is_said_at_most_once_a_minute(monkeypatch):
  # three failed reads (about 3 s, disabled), one good read, again: as a reader that throws on a head in view and reads a run
  # without one would at every signal. Said once, then again only CAMERA_OFF_AGAIN_S after it, the spell still on
  ev = Events(monkeypatch)
  car = Car([])
  said = []
  t = 0.
  while t < CAMERA_OFF_AGAIN_S + 6.:
    said += [t + i * DT_MDL for i in spell(car, OFF, 3.)]
    spell(car, READING, DT_MDL)
    t += 3. + DT_MDL
  starts = [s for k, s in enumerate(said) if k == 0 or s - said[k - 1] > 2 * DT_MDL]
  assert len(starts) == 2 and CAMERA_OFF_AGAIN_S - 0.1 <= starts[1] - starts[0] <= CAMERA_OFF_AGAIN_S + 3.5
  assert len(ev.named('turnpilot.camera_lights_off')) == 2


@pytest.mark.parametrize("prior", ['cannotStopComfortably', 'stopOverrun', 'proceedingOnYellow', 'mapControlsUnavailable'])
def test_camera_light_reader_off_never_masks_a_live_alert(prior):
  """The reader going off at a signal (large heads, the slow device CPU) is the moment overrun and turn-line alerts fire:
  its status line waits for them and is still said, never in their place."""
  car = Car([(300., 'signal')])
  car.step()
  car.policy._transient(prior)
  off = Lights(camera=CameraLights([], True, 'disabled'))
  out, _, _ = car.step(lights=off)
  assert out.alert == prior
  outs = [car.step(lights=off)[0] for _ in range(int(6. / DT_MDL))]
  alerts = [o.alert for o in outs]
  assert alerts[0] == prior and 'lightSourceRejected' in alerts and alerts[-1] == 'lightSourceRejected'  # the spell's line stays
  first = alerts.index('lightSourceRejected')
  assert all(a == prior for a in alerts[:first])  # never in place of it
  onset = sum(o.reason == CAMERA_OFF_REASON for o in outs[first:])
  assert onset * DT_MDL >= map_controls.ABORT_HOLD_S - 2 * DT_MDL  # its chimed onset in full once the prior alert is done


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
    # the reader's loud off, once, only where nothing else is said: never in place of a take-control or overrun alert
    assert b.alert == a.alert or (a.alert == 'none' and b.alert == 'lightSourceRejected'), (a.alert, b.alert)


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
  # aged out: the camera says nothing; the stop it added stays until a steady green, the camera's or an accepted one
  # (map_controls docstring)
  assert sim.tp.map_update(sim.sm, sim.v, False).shown.light_state == 'unknown'
  assert sim.tp.map_controls.stop_ahead() is not None


def test_turnpilotd_keeps_a_failing_camera_readers_line_up_and_says_its_onset_engaged():
  # S2: the plan carried the line 2 s, all of it disengaged (the Sim's default), and selfdrived showed it on 0 frames. Now the
  # plan carries it the whole spell, its onset's words until it has been up ABORT_HOLD_S engaged, then the silent line
  sim = camera_sim('shadow', lambda sim: detections('unknown', 0., status='disabled'))
  sim.run(10.)
  plans = [p for _, _, p in sim.trace]
  first = next(i for i, p in enumerate(plans) if p.controlAlert == 'lightSourceRejected')
  assert all(p.controlAlert == 'lightSourceRejected' and p.reason == CAMERA_OFF_REASON for p in plans[first:])
  engaged = [sim.step(engaged=True) for _ in range(int(5. / DT_MDL))]
  assert all(p.controlAlert == 'lightSourceRejected' for p in engaged)
  onset = [i for i, p in enumerate(engaged) if p.reason == CAMERA_OFF_REASON]
  assert onset == list(range(len(onset))) and abs(len(onset) * DT_MDL - map_controls.ABORT_HOLD_S) <= 3 * DT_MDL
  assert engaged[-1].reason == CAMERA_OFF_STILL_REASON


def test_turnpilotd_says_a_failing_camera_readers_onset_in_its_own_words_over_a_holds():
  # S2: turnpilotd's plan.reason put a decision's reason first, so the reader failing on a stop sign's approach said "Stop sign
  # ahead" under the reader's title and its onset's words never reached the driver. Its chimed onset's words now go over a
  # hold's (no map prompt, no hard brake); after it the hold's reason is the plan's again (the line then shows its own plain
  # words, events.py)
  from openpilot.cereal import messaging
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(150., 'stop')])
  sim.tp.camera_mode, sim.go_taps = 'addOnly', False
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

  def lights(sim):
    if int(round(sim.t / DT_MDL)) % 2 == 0:
      sim.sm.put('turnpilotDetections', detections('unknown', 0., status='disabled' if sim.s > 80. else 'read'))
    return none
  sim.lights = lights
  plans = [sim.step(engaged=True) for _ in range(int(14. / DT_MDL))]
  said = [i for i, p in enumerate(plans) if p.controlAlert == 'lightSourceRejected']
  assert said and all(plans[i].controlAlert == 'lightSourceRejected' for i in range(said[0], len(plans)))
  onset = [i for i in said if plans[i].reason == CAMERA_OFF_REASON]
  assert onset[0] == said[0] and abs(len(onset) * DT_MDL - map_controls.ABORT_HOLD_S) <= 3 * DT_MDL
  assert plans[said[0] - 1].reason == 'Stop sign ahead' and plans[-1].reason == 'Stop sign ahead'  # a hold's reason around it


# --- the route turn's junction the map has no signal for (2026-11-06: a right rolled on red at an unmapped signal) ---

def cam_junction(state, s_head, shadow=False):
  """The camera reading a governing head at s_head (road position) as the car moves: add-only unless shadow."""
  return lambda car: Lights(camera=camera(state, s_head - car.s, shadow))


def right_at(s_junction=200., mid=7, v=13.4):
  car = Car([], v=v)
  car.junction = (s_junction, mid)
  return car


def test_unmapped_right_camera_red_stops_at_the_junction_line_and_holds_until_a_steady_camera_green():
  from openpilot.selfdrive.turnpilot.map_controls import CAMERA_GREEN_STEADY_S, CONTROL_LINE_M
  car = right_at()
  out, dec = car.run_until(held, lights=cam_junction('red', 215.))
  line = 200. - CONTROL_LINE_M
  assert -2. < line - car.s < 5. and all(r[3] >= MAP_DECEL_MAX - 1e-6 for r in car.rows)  # never past -2.0
  assert out.shown.kind == 'signal' and car.policy.camera_signal is not None
  assert out.stops[0].reason == "Red light (camera)"
  s_held = car.s
  # waiting at the red: held, no handback while the camera reads it, never a roll
  for _ in range(int(30. / DT_MDL)):
    out, dec, _ = car.step(lights=cam_junction('red', 215.))
    assert held(out, dec) and out.prompt == 'none'
  # the camera reads green: the stop it added is TurnPilot's own, let go on its steady green (2026-11-06: until then every
  # camera stop ended in the stand handback), never before CAMERA_GREEN_STEADY_S
  for _ in range(int((CAMERA_GREEN_STEADY_S - 0.1) / DT_MDL)):
    out, dec, _ = car.step(lights=cam_junction('green', 215.))
    assert held(out, dec) and out.prompt == 'none'
  assert abs(car.s - s_held) < 0.05
  out, dec = car.run_until(lambda out, dec: not dec.should_stop, max_s=0.3, lights=cam_junction('green', 215.))
  assert out.released_stopped and out.prompt == 'none'


def test_unmapped_right_camera_red_released_by_an_accepted_green():
  # the sim oracle (an accepted source) reading green there once the camera's red is gone lets the car go
  car = right_at()
  car.run_until(held, lights=cam_junction('red', 215.))

  def green(car):
    return replace(lights('green', 215. - car.s), camera=camera('green', 215. - car.s, False))
  out, dec, _ = car.run(GREEN_STEADY_S + 0.2, lights=green)
  assert not dec.should_stop
  car.run(10., lights=green)
  assert car.s > 220.


def test_unmapped_right_camera_green_read_before_the_stop_adds_nothing():
  # the junction's head read green from far out: its signal is made (a light is there), and its steady green lets the turn
  # go before its stop engages, so nothing slows the car
  base, car = right_at(), right_at()
  for _ in range(int(25. / DT_MDL)):
    a = base.step()
    b = car.step(lights=cam_junction('green', 215.))
    assert a[0].stops == b[0].stops == [] and a[2] == b[2]
  assert car.s > 300.


def test_unmapped_right_camera_head_of_unknown_colour_goes_with_the_model():
  # Kellen, 2026-11-07: a colourless head makes no stop (no want of a green), and no handback for the unread light
  car = right_at()
  for _ in range(int(30. / DT_MDL)):
    out, dec, _ = car.step(lights=cam_junction('unknown', 215.))
    assert not held(out, dec) and out.prompt == 'none' and car.policy.camera_signal is None


def test_unmapped_right_shadow_camera_red_adds_nothing():
  base, car = right_at(), right_at()
  for _ in range(int(25. / DT_MDL)):
    a = base.step()
    b = car.step(lights=cam_junction('red', 215., shadow=True))
    assert a[0].stops == b[0].stops and a[2] == b[2]
  assert car.policy.camera_signal is None


def test_no_route_turn_no_camera_signal():
  # a red no mapped signal owns with no route turn there (a straight road): logged, as before (camera_light_unmapped)
  car = Car([])
  out, dec, _ = car.run(3., lights=lambda car: Lights(camera=camera('red', 40., False)))
  assert out.stops == [] and not dec.should_stop and car.policy.camera_signal is None


def test_a_camera_red_head_far_from_the_turn_junction_is_not_its():
  # a head 80 m past the turn's junction (the next junction's) makes no signal at the turn
  car = right_at()
  car.run(5., lights=cam_junction('red', 280.))
  assert car.policy.camera_signal is None


def test_a_mapped_signal_at_the_right_turn_stops_on_the_camera_red_at_its_own_line():
  car = Car([(192., 'signal', True)])
  car.junction = (200., 7)
  out, dec = car.run_until(held, lights=cam_junction('red', 215.))
  assert -2. < 192. - car.s < 5. and car.policy.camera_signal is None  # the mapped signal's own add-only stop
  assert out.shown.kind == 'signal' and out.shown.line_mapped


def test_unmapped_right_yellow_far_enough_stops_too_late_goes():
  car = right_at()
  out, dec = car.run_until(held, lights=cam_junction('yellow', 215.))
  assert -2. < 190. - car.s < 5.
  # first read at the line's 9 m at 13.4 m/s: too late to stop for at -2.0, the car goes (a yellow decided so)
  car = Car([], v=13.4)
  car.junction = (200., 7)
  car.s = 181.
  for _ in range(int(5. / DT_MDL)):
    out, dec, _ = car.step(lights=cam_junction('yellow', 215.))
    assert not dec.should_stop
  assert car.s > 220. and car.policy.camera_signal is None


def test_unmapped_unprotected_left_keeps_its_own_hold():
  # an unprotected left's own hold line holds the car at its junction already; no second stop is made there
  car = Car([])
  car.turn = (200., 7, 'left')
  car.junction = (200., 7)
  out, dec = car.run_until(held, lights=cam_junction('red', 215.))
  assert car.policy.camera_signal is None and out.shown.kind == 'unprotectedTurn'


def test_unmapped_right_camera_red_read_past_the_line_makes_no_stop_in_the_junction():
  car = right_at()
  car.v, car.s = 5., 192.  # past the junction's line, turning
  for _ in range(int(3. / DT_MDL)):
    out, dec, _ = car.step(lights=cam_junction('red', 215.))
    assert out.stops == []


def test_an_unengaged_camera_signal_drops_when_the_maneuver_moves_on():
  car = right_at()
  car.run(3., lights=cam_junction('red', 215.))  # read far out, not engaged yet at 13.4 m/s 150 m out
  cs = car.policy.camera_signal
  assert cs is not None
  car.junction = (600., 8)  # the route's maneuver moved on (a reroute) before the curve engaged: dropped
  car.step(lights=lambda car: NO_LIGHTS)
  assert car.policy.camera_signal is None


def test_an_engaged_camera_signal_dead_reckons_through_a_frame_without_the_junction():
  car = right_at()
  car.run_until(lambda out, dec: car.policy.camera_signal is not None and car.policy.camera_signal.engaged,
                lights=cam_junction('red', 215.))
  d = car.policy.camera_signal.d
  car.junction = None  # a fault's frame (KEEP_TURN): no route this frame
  out, dec, _ = car.step(lights=cam_junction('red', 215.))
  cs = car.policy.camera_signal
  assert cs is not None and abs(cs.d - (d - car.rows[-2][2] * DT_MDL)) < 0.05 and out.stops


# --- through turnpilotd: an unmapped right on a camera red ---

def right_sim(mode, state, controls=(), s_head=215.):
  from openpilot.cereal import messaging
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
  sim = Sim([(200., "turn", "right"), (1500., "turn", "left")], controls=controls)
  sim.tp.camera_mode = mode
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

  def lights(sim):
    if int(round(sim.t / DT_MDL)) % 2 == 0:
      sim.sm.put('turnpilotDetections', detections(state, s_head - sim.s, shadow=mode != 'addOnly'))
    return none
  sim.lights = lights
  return sim


def test_turnpilotd_unmapped_right_on_a_camera_red_stops_at_the_line_and_holds():
  from openpilot.cereal import log
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import stop_at
  sim = right_sim('addOnly', 'red')
  plan = stop_at(sim, 190.)
  assert plan.shouldStop and -2. < 190. - sim.s < 6. and all(p.accelCap >= MAP_DECEL_MAX - 1e-6 for _, _, p in sim.trace)
  assert str(plan.mapControl.kind) == 'signal'
  s_held = sim.s
  sim.run(20.)
  assert sim.s - s_held < 0.1 and all(p.shouldStop for _, _, p in sim.trace[-int(19. / DT_MDL):])
  # no turn desire out while held at the red (the desire waits for the go)
  assert all(p.desire != log.Desire.turnRight for s, _, p in sim.trace if s < 190.)


def test_turnpilotd_unmapped_right_camera_green_goes():
  # as with no head there at all: through the junction on the turn, never stopped (its steady green read from far out)
  base, sim = right_sim('addOnly', 'green', s_head=1000.), right_sim('addOnly', 'green')  # base: the head no junction's
  for _ in range(int(30. / DT_MDL)):
    a, b = base.step(), sim.step()
    assert (a.accelCap, a.speedCap, a.shouldStop, a.desire) == (b.accelCap, b.speedCap, b.shouldStop, b.desire)
  assert sim.s > 210. and not any(p.shouldStop for _, _, p in sim.trace)




def test_turnpilotd_mapped_signal_right_on_a_camera_red_stops_at_its_line_first():
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import stop_at
  sim = right_sim('addOnly', 'red', controls=[(192., 'signal')])
  plan = stop_at(sim, 192.)
  assert plan.shouldStop and -2. < 192. - sim.s < 6. and str(plan.mapControl.kind) == 'signal'
  assert sim.tp.map_controls.camera_signal is None


def test_turnpilotd_shadow_camera_red_at_an_unmapped_right_is_logged_only():
  # the 7c37f66 behaviour, kept under shadow (set by hand): nothing stops the car
  sim = right_sim('shadow', 'red')
  sim.run(25.)
  assert not any(p.shouldStop for _, _, p in sim.trace) and sim.tp.map_controls.camera_signal is None


def test_a_mapped_signal_set_far_back_of_the_turn_junction_keeps_its_heads():
  # its line 35 m short of the junction (outside PROTECT_BEFORE_M), the head 5 m past the junction its own: the stop is at the
  # mapped line, never moved up to the junction's
  car = Car([(165., 'signal', True)])
  car.junction = (200., 7)
  out, dec = car.run_until(held, lights=cam_junction('red', 195.))
  assert -2. < 165. - car.s < 5. and car.policy.camera_signal is None
