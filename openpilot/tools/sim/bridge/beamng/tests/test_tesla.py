import hashlib
import random
from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.structs import car
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.car_signal import CarSignalsConfig
from opendbc.car.tesla.body_model import BodyModel, LEVEL_HOLD_S, STOCK_IDLE
from opendbc.car.tesla.teslacan import BODY_CONTROLS_ADDR, body_controls_request, body_controls_valid, create_body_controls
from opendbc.car.tesla.values import CAR, TeslaFlags
from opendbc.car.vin import VIN_UNKNOWN
from openpilot.selfdrive.turnpilot.car_signals import record_matches, resolver
from openpilot.tools.sim.bridge.beamng.tesla import BODY_OWN_S, GEAR_D, GameLights, TeslaCan, sim_car_signals_record, \
  write_sim_car_signals

CANDIDATE = CAR.TESLA_MODEL_3


def car_state(**kw):
  s = {"wheel_speed": 10.0, "steering_angle": 15.0, "user_gas": 0.0, "user_brake": 0.0, "user_torque_nm": 0.0, "left_blinker": False,
           "right_blinker": False, "gear": GEAR_D}
  s.update(kw)
  return SimpleNamespace(**s)


def packets(car, state, ticks, start=0):
  return [(int((start + t) * 1e7), [CanData(*m) for m in car.step(state, start + t)]) for t in range(ticks)]


def fingerprint(car, state):
  finger = gen_empty_fingerprint()
  for _, frames in packets(car, state, 200):
    for f in frames:
      finger[f.src][f.address] = len(f.dat)
  return finger


class TestTeslaCan:
  def setup_method(self):
    self.car = TeslaCan()
    finger = fingerprint(self.car, car_state())
    self.CP = interfaces[CANDIDATE].get_params(CANDIDATE, finger, [], alpha_long=True, is_release=False, docs=False)
    self.CI = interfaces[CANDIDATE](self.CP)

  def update(self, state, ticks=100, start=1000):
    cs = None
    for nanos, frames in packets(self.car, state, ticks, start):
      cs = self.CI.update([(nanos, frames)])
    return cs

  def test_fingerprint_not_dashcam(self):
    finger = fingerprint(TeslaCan(), car_state())
    assert 0x054 in finger[0], "DI_autonomyHealth must be on bus 0 from the first frames"
    assert 0x293 in finger[2]
    assert not self.CP.dashcamOnly
    assert not self.CP.flags & TeslaFlags.MISSING_DAS_SETTINGS
    assert self.CP.openpilotLongitudinalControl
    assert self.CP.steerControlType == structs.CarParams.SteerControlType.angle

  def test_carstate(self):
    cs = self.update(car_state(wheel_speed=10.0, steering_angle=15.0, left_blinker=True))
    assert cs.canValid and not cs.canTimeout
    assert abs(cs.vEgoRaw - 10.0) < 0.1
    assert abs(cs.steeringAngleDeg - 15.0) < 0.11
    assert cs.gearShifter == structs.CarState.GearShifter.drive
    assert cs.leftBlinker and not cs.rightBlinker
    assert not (cs.seatbeltUnlatched or cs.doorOpen or cs.brakePressed or cs.gasPressed or cs.steeringPressed)
    assert not (cs.steerFaultPermanent or cs.steerFaultTemporary or cs.accFaulted or cs.invalidLkasSetting or cs.stockAeb)
    assert cs.cruiseState.available and not cs.cruiseState.enabled

  def test_engage_and_brake_cancel(self):
    self.update(car_state())
    self.car.engage(10.0, set_speed_kph=50.0)
    cs = self.update(car_state())
    assert cs.cruiseState.enabled
    assert abs(cs.cruiseState.speed - 50.0 / 3.6) < 0.2
    cs = self.update(car_state(user_brake=0.5))
    assert cs.brakePressed and not cs.cruiseState.enabled

  def test_pedals(self):
    cs = self.update(car_state(user_gas=0.3))
    assert cs.gasPressed

  def test_commands_round_trip(self):
    self.car.engage(10.0)
    self.update(car_state(steering_angle=0.0))
    msg = car.CarControl.new_message(enabled=True, latActive=True, longActive=True)
    msg.actuators.steeringAngleDeg = 30.0
    msg.actuators.accel = 1.2
    CC = msg.as_reader()
    sent = []
    for i in range(20):
      new_actuators, can_sends = self.CI.CC.update(CC, self.CI.CS, int(i * 1e7))
      sent.append((int(i * 1e7), [CanData(*m) for m in can_sends]))
    self.car.parse_sendcan(sent, now=0.0)
    # the angle the car sees is openpilot's rate-limited one, not the raw request
    assert abs(self.car.steer_cmd - new_actuators.steeringAngleDeg) < 0.11
    assert 0.0 < self.car.steer_cmd < 30.0
    assert abs(self.car.accel_cmd - 1.2) < 0.05
    assert self.car.lat_active(now=0.05) and self.car.long_active(now=0.05)
    assert not self.car.lat_active(now=1.0)

    msg.cruiseControl.cancel = True
    _, can_sends = self.CI.CC.update(msg.as_reader(), self.CI.CS, int(30 * 1e7))
    self.car.parse_sendcan([(int(30 * 1e7), [CanData(*m) for m in can_sends])], now=0.3)
    assert not self.car.engaged


# --- TurnPilot's commanded turn signal (DESIGN phase4/blinker 9.2, 11.5) ---

DT = 0.01
REQ_NONE, REQ_LEFT, REQ_RIGHT, REQ_CANCEL = 0, 1, 2, 3
UI_WARNING = 0x311
DBC_PARTY = "tesla_model3_party"
# today's frames (before the car_signals option) over golden_digest's drive, so the option off changes nothing
GOLDEN_FRAMES = "ab15019a5334948b1c791ed6a7dc92e7aa149741c11322f06381ae3561485d30"


def golden_digest(car) -> str:
  h = hashlib.sha256()
  for tick in range(400):
    if tick == 50:
      car.engage(10.0, 50.0)
    st = car_state(wheel_speed=10.0 + tick * 0.01, steering_angle=(tick % 37) - 18.0, left_blinker=100 <= tick < 200,
                   right_blinker=250 <= tick < 300, user_gas=0.3 if 320 <= tick < 330 else 0.0)
    for addr, dat, bus in car.step(st, tick):
      h.update(addr.to_bytes(4, 'little') + bytes([bus]) + bytes(dat))
  return h.hexdigest()


def test_frames_unchanged_with_car_signals_off():
  assert golden_digest(TeslaCan()) == GOLDEN_FRAMES
  assert golden_digest(TeslaCan(car_signals=False)) == GOLDEN_FRAMES


def quiet_body(**kw):
  return BodyModel(**{'semantics': 'level', 'on_latency': 0.3, 'rng': random.Random(0), 'glitch_rate': 0., 'early_rate': 0.,
                      'jitter': False, **kw})


class SignalDrive:
  """The bridge's car with car_signals on, and a stand-in for the port: after each stock frame, one copy of it carrying
  the request (as the port sends, one per stock frame), back through parse_sendcan."""

  def __init__(self, car, start=1000):
    self.car = car
    self.tick = start
    self.ui = CANParser(DBC_PARTY, [], 0)
    self.ui.vl["UI_warning"]
    self.rows = []  # per UI frame: (t, left_blinking, right_blinking)
    self.stock = []  # (t, dat) on bus 2
    self.echoes = []  # (t, dat, src)
    self.sent = []  # (t, dat)
    self.lights = []  # every change of the game's lights: (t, (left, right, hazard))

  @property
  def t(self) -> float:
    return self.tick * DT

  def run(self, seconds, request=None, reason=8, state=None, once=False):
    state = state or car_state()
    for _ in range(int(round(seconds / DT))):
      t = self.t
      frames = self.car.step(state, self.tick, now=t)
      ui = [(a, d, b) for a, d, b in frames if a == UI_WARNING]
      if ui:
        self.ui.update([(int(t * 1e9), [CanData(*ui[0])])])
        vl = self.ui.vl["UI_warning"]
        self.rows.append((t, int(vl["leftBlinkerBlinking"]), int(vl["rightBlinkerBlinking"])))
      for a, d, b in frames:
        if a == BODY_CONTROLS_ADDR and b == 2:
          self.stock.append((t, bytes(d)))
          if request is not None:
            ours = create_body_controls(bytes(d), request, reason if request != REQ_CANCEL else 15)
            self.sent.append((t, ours))
            self.car.parse_sendcan([(int(t * 1e9), [CanData(BODY_CONTROLS_ADDR, ours, 0)])], now=t)
            if once:
              request = None
        elif a == BODY_CONTROLS_ADDR:
          self.echoes.append((t, bytes(d), b))
      lights = self.car.lights
      if not self.lights or self.lights[-1][1] != lights:
        self.lights.append((t, lights))
      self.tick += 1
    return self

  def lit(self, side, since=-1e9):
    """UI rows since t with that side's lamp blinking (1 or 2)."""
    return [r for r in self.rows if r[0] >= since and r[side] in (1, 2)]



class TestTeslaCanCarSignals:
  def test_idle_body_controls_on_bus_2(self):
    car = TeslaCan(car_signals=True, body=BodyModel(rng=random.Random(1)))  # the DAS's jitter and early frames on
    d = SignalDrive(car).run(20.)
    assert 36 <= len(d.stock) <= 46  # 2 Hz, with the measured early frames
    counters = [dat[6] >> 4 for _, dat in d.stock]
    assert all((b - a) % 16 == 1 for a, b in zip(counters, counters[1:], strict=False))
    assert all(body_controls_valid(dat) and body_controls_request(dat) == (REQ_NONE, 0) for _, dat in d.stock)
    assert all(dat[:6] == STOCK_IDLE[:6] for _, dat in d.stock)
    gaps = [b[0] - a[0] for a, b in zip(d.stock, d.stock[1:], strict=False)]
    assert max(gaps) < 0.52 and sum(0.495 < g < 0.52 for g in gaps) >= 0.8 * len(gaps)
    assert len({round(g, 2) for g in gaps if g > 0.495}) >= 2, "the DAS's jitter"
    assert d.lights == [(1000 * DT, (False, False, False))] and not d.lit(1) and not d.lit(2)

  def test_left_lights_after_the_latency_and_one_set_lights(self):
    car = TeslaCan(car_signals=True, body=quiet_body())
    car.engage(10.0)
    d = SignalDrive(car).run(1.).run(4., request=REQ_LEFT)
    first = d.sent[0][0]
    lit = d.lit(1)
    assert lit and first + 0.3 - 1e-6 <= lit[0][0] <= first + 0.3 + 0.1 + 1e-6
    assert not d.lit(2)
    assert {r[1] for r in d.rows if r[0] > lit[0][0]} == {1, 2}, "the bridge's lamp blinks 2 and 1"
    # the game's indicator: one SetLights for the request, however the lamp blinks
    assert [x[1] for x in d.lights] == [(False, False, False), (True, False, False)]
    # every frame of ours was accepted and echoed on 128, the stock frames blocked while we own the message
    assert len(d.echoes) == len(d.sent) and {e[2] for e in d.echoes} == {128}
    assert all(body_controls_request(dat)[0] == REQ_LEFT for t, dat in car.body.frames_received if t > first)

  def test_cancel_puts_it_out_and_clears_the_stalk(self):
    car = TeslaCan(car_signals=True, body=quiet_body())
    car.engage(10.0)
    car.toggle_stalk('left')  # the keyboard's blinker is the driver's stalk
    d = SignalDrive(car).run(1.)
    assert d.lit(1) and car.stalk == (True, False)
    d.run(1.5, request=REQ_LEFT).run(0.6, request=REQ_CANCEL, once=True)
    cancel_t = [t for t, dat in d.sent if body_controls_request(dat)[0] == REQ_CANCEL][0]
    assert not d.lit(1, since=cancel_t + 0.11)
    assert car.stalk == (False, False) and car.body.cancels_received == 1
    assert d.lights[-1][1] == (False, False, False)

  def test_none_puts_it_out(self):
    car = TeslaCan(car_signals=True, body=quiet_body())
    car.engage(10.0)
    d = SignalDrive(car).run(0.5).run(2., request=REQ_LEFT)
    assert d.lit(1)
    d.run(0.6, request=REQ_NONE, once=True)
    none_t = [t for t, dat in d.sent if body_controls_request(dat)[0] == REQ_NONE][0]
    assert not d.lit(1, since=none_t + 0.11)
    assert d.echoes[-1][2] == 128

  def test_none_after_the_ownership_window_still_ends_it(self):
    """As the panda: a NONE is accepted while ours is the last request, however long ago (only the stock frame's
    block times out at 750 ms)."""
    car = TeslaCan(car_signals=True, body=quiet_body())
    car.engage(10.0)
    d = SignalDrive(car).run(0.5).run(1.5, request=REQ_LEFT).run(0.6, request=REQ_CANCEL, once=True).run(1.0)
    cancel_t = [t for t, dat in d.sent if body_controls_request(dat)[0] == REQ_CANCEL][0]
    d.run(0.6, request=REQ_NONE, once=True)
    none_t, none = [(t, dat) for t, dat in d.sent if body_controls_request(dat)[0] == REQ_NONE][0]
    assert none_t - cancel_t > BODY_OWN_S
    assert [e[2] for e in d.echoes if e[1] == none] == [128] and not car.body_owned
    d.run(0.6, request=REQ_NONE, once=True)  # but not once ours has ended
    assert d.echoes[-1][2] == 192

  def test_disengaged_request_lights_nothing(self):
    car = TeslaCan(car_signals=True, body=quiet_body())
    d = SignalDrive(car).run(3., request=REQ_LEFT)
    assert d.sent and not d.lit(1) and not d.lit(2)
    assert {e[2] for e in d.echoes} == {192}, "refused, as the panda refuses it"
    assert car.body.frames_received and all(body_controls_request(dat)[0] == REQ_NONE for _, dat in car.body.frames_received)

  def test_request_750ms_stale_is_dropped(self):
    car = TeslaCan(car_signals=True, body=quiet_body())
    car.engage(10.0)
    d = SignalDrive(car).run(0.6).run(2., request=REQ_LEFT)
    last = d.sent[-1][0]
    d.run(2.)
    assert BODY_OWN_S == 0.75
    stock_after = [t for t, _ in d.stock if t > last]
    received = [t for t, dat in car.body.frames_received if t > last]
    # the stock frame inside 750 ms of our last is still blocked; the first after it reaches the body
    assert stock_after[0] - last < BODY_OWN_S and stock_after[0] not in received
    assert stock_after[1] - last >= BODY_OWN_S and received[0] == stock_after[1]
    # so the lamp is out on the stock NONE, before the level body's own hold would have dropped it
    assert not d.lit(1, since=stock_after[1] + 0.11) and stock_after[1] + 0.11 < last + LEVEL_HOLD_S

  def test_hazard_key_lights_both(self):
    car = TeslaCan(car_signals=True, body=quiet_body())
    car.toggle_hazards()
    d = SignalDrive(car).run(1.)
    assert d.lit(1) and d.lit(2) and all(r[1] == r[2] for r in d.rows)
    assert d.lights[-1][1] == (True, True, True)
    car.toggle_hazards()
    d.run(0.5)
    assert d.lights[-1][1] == (False, False, False)

  def test_stalk_toggles_like_the_keyboard(self):
    car = TeslaCan(car_signals=True, body=quiet_body())
    car.toggle_stalk('left')
    assert car.stalk == (True, False)
    car.toggle_stalk('right')
    assert car.stalk == (False, True)
    car.toggle_stalk('right')
    assert car.stalk == (False, False)

  def test_driver_hold_sets_the_stalk_on_a_change_only(self):
    car = TeslaCan(car_signals=True, body=quiet_body())
    car.driver_hold((True, False))
    assert car.stalk == (True, False)
    car.body.driver_stalk(1, False)  # a CANCEL cleared it: the held stalk isn't latched again
    car.driver_hold((True, False))
    assert car.stalk == (False, False)
    car.driver_hold((False, False))
    car.driver_hold((False, True))
    assert car.stalk == (False, True)

  def test_port_masks_its_own_lamp_through_the_bridge(self):
    """openpilot's Tesla port with the car signal on, over the bridge's body: the lamp it lights isn't the driver's."""
    car = TeslaCan(car_signals=True, body=quiet_body())
    finger = fingerprint(TeslaCan(), car_state())
    CP = interfaces[CANDIDATE].get_params(CANDIDATE, finger, [], alpha_long=True, is_release=False, docs=False,
                                          car_signals=CarSignalsConfig(mode='on'))
    CI = interfaces[CANDIDATE](CP)
    car.engage(10.0)
    cs, lit_cs, blinkers = None, [], []
    for k in range(1200):  # 6 s wanting left, then 6 s not: the port's own CANCEL and NONE through the stand-in
      tick = 3000 + k
      t = tick * DT
      frames = car.step(car_state(), tick, now=t)
      cs = CI.update([(int(t * 1e9), [CanData(*f) for f in frames])])
      _, can_sends = CI.CC.update(car_cc(left=k < 600), CI.CS, int(t * 1e9))
      car.parse_sendcan([(int(t * 1e9), [CanData(*m) for m in can_sends])], now=t)
      blinkers.append(cs.leftBlinker or cs.rightBlinker)
      if car.body.lit(1):
        lit_cs.append(cs)
      if k == 599:
        assert str(cs.carSignal.status) == 'lit' and cs.carSignal.leftLamp and str(cs.carSignal.fault) == 'none'
    assert lit_cs, "the port's request lit the bridge's lamp"
    assert not any(blinkers), "and carState never reads it as the driver's, lit or released"
    assert str(cs.carSignal.status) == 'ready' and not cs.carSignal.leftLamp and str(cs.carSignal.fault) == 'none'
    assert cs.carSignal.rejected == 0 and not car.body_owned and not car.body.lit(1)
    assert car.body.cancels_received == 1, "the port's release: one CANCEL, then NONE"
    assert body_controls_request(car.body.frames_received[-1][1])[0] == REQ_NONE


def car_cc(left=False, right=False):
  return car.CarControl.new_message(enabled=True, latActive=True, longActive=True, leftBlinker=left, rightBlinker=right).as_reader()


def test_sim_record_matches_the_sim_car_only_in_the_sim():
  record = sim_car_signals_record()
  assert record['sim'] and record['gear'] == 'D' and record['vin'] == VIN_UNKNOWN and record['fw'] == []
  assert record_matches(record, VIN_UNKNOWN, [], simulation=True) == (True, "")
  assert not record_matches(record, VIN_UNKNOWN, [], simulation=False)[0]


class FakeParams(dict):
  """Params as card's resolver reads them: an unset key reads None, or its params_keys.h default with return_default (as
  car_signals.forced_on asks for TurnPilotCarSignalsForceOn, default 1). Until 2026-11-06 this read every key None, so a
  forced-on car read as off here and the bridge's off path passed while card, on the real Params, signalled."""
  DEFAULTS = {"TurnPilotCarSignalsForceOn": True}

  def get(self, key, block=False, return_default=False):
    return super().get(key, self.DEFAULTS.get(key) if return_default else None)

  def get_bool(self, key, block=False):
    return bool(self.get(key))

  def put(self, key, value, *a, **kw):
    self[key] = value

  def put_bool(self, key, value, *a, **kw):
    self[key] = bool(value)

  def remove(self, key):
    self.pop(key, None)


def test_car_signals_flag_writes_on_and_the_sim_record():
  # TurnPilotCarSignalsForceOn 0, so the record decides: the sim record only with SIMULATION (under ForceOn none is read)
  params = FakeParams(TurnPilotCarSignalsForceOn=False)
  write_sim_car_signals(params)
  assert params["TurnPilotCarSignals"] == "on"
  config = resolver(params, simulation=True)(VIN_UNKNOWN, [])
  assert config == CarSignalsConfig(mode='on', on_reason=8, recheck=False, on_separates=False)
  assert resolver(params, simulation=False)(VIN_UNKNOWN, []) == CarSignalsConfig()


def test_the_sim_runs_with_the_cars_own_turn_signal_on_unless_told_not_to():
  """The sim's default (merging tp/blinker into tp/integrate): the bridge writes TurnPilotCarSignals on and the sim verified
  record before the car's first CAN, so card's port signals TurnPilot's turns, the body model lights the game's indicator
  and the autonomous left's gate runs on a signalled left (LEFT_GO_NEEDS_SIGNAL). --no-car-signals writes it off: today's
  frames, the driver prompted."""
  from openpilot.tools.sim.bridge.beamng.bridge import car_signals_at_start
  from openpilot.tools.sim.bridge.beamng.run_bridge import config_from_args, parse_args
  from openpilot.tools.sim.bridge.beamng.world import BeamNGConfig
  assert BeamNGConfig().car_signals and parse_args([]).car_signals and config_from_args(parse_args([])).car_signals
  assert not parse_args(['--no-car-signals']).car_signals and not config_from_args(parse_args(['--no-car-signals'])).car_signals
  params = FakeParams()
  assert car_signals_at_start(params, True)
  assert params["TurnPilotCarSignals"] == "on" and params["TurnPilotCarSignalsVerified"] == sim_car_signals_record()
  assert params["TurnPilotCarSignalsForceOn"] is True  # the car's default, put back after an earlier --no-car-signals
  assert resolver(params, simulation=True)(VIN_UNKNOWN, []).mode == 'on'
  assert not car_signals_at_start(params, False) and params["TurnPilotCarSignals"] == "off"
  assert params["TurnPilotCarSignalsForceOn"] is False
  assert resolver(params, simulation=True)(VIN_UNKNOWN, []) == CarSignalsConfig()


def test_no_car_signals_turns_force_on_off_so_card_and_the_bridge_agree():
  """--no-car-signals on a comma at TurnPilotCarSignalsForceOn's default (unset, 1): until 2026-11-06 it wrote the mode off
  alone, card's resolver still put the car's signal on (forced) and sent DAS_bodyControls, and the bridge, reading the mode
  off, ran no body: neither path. Now the off path puts ForceOn 0, card resolves off, and the body state is card's."""
  from openpilot.tools.sim.bridge.beamng.bridge import car_signals_at_start
  for on in (True, False):
    params = FakeParams()
    assert params.get("TurnPilotCarSignalsForceOn", return_default=True) is True  # unset: the default, on
    body = car_signals_at_start(params, on)
    card = resolver(params, simulation=True)(VIN_UNKNOWN, [])
    assert body == on and (card.mode == 'on') == on and (card == CarSignalsConfig()) == (not on), (on, body, card)
  # a comma left at mode off by hand, ForceOn at its default: the bridge reads card's forced on and runs the body
  from openpilot.selfdrive.turnpilot.car_signals import forced_on
  params = FakeParams(TurnPilotCarSignals="off")
  assert forced_on(params) and resolver(params, simulation=True)(VIN_UNKNOWN, []).mode == 'on'
  # FakeParams writes as it's told, at once: this checks the logic only. The real Params' put returns before its write
  # lands, which test_the_bridges_body_is_what_card_reads_on_the_real_params checks.


@pytest.mark.parametrize('on', [True, False], ids=['on', 'off'])
def test_the_bridges_body_is_what_card_reads_on_the_real_params(on):
  """car_signals_at_start on the real Params, after an earlier run that left the other choice (launch_openpilot.sh's
  TURNPILOT_CAR_SIGNALS and the bridge's --no-car-signals told apart, or the sim's last run): the bridge's body is what
  card's resolver reads when it fingerprints the car. Until 2026-11-06 its writes didn't block: Params.put without
  block=True queues the write on params.cc's async thread, and the read-back on the next line read the disk before it
  landed, so the bridge decided its body from the earlier run's values (review probe: 150 of 150 calls on a fresh handle
  read the previous state). Launcher off and bridge on, the bridge ran no body under a card that sent DAS_bodyControls;
  launcher on and bridge off, a body under a card that resolved off."""
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  from openpilot.tools.sim.bridge.beamng.bridge import car_signals_at_start
  with OpenpilotPrefix():
    for _ in range(5):
      earlier = Params()  # the other choice, landed (blocking), as an earlier run leaves it
      earlier.put("TurnPilotCarSignals", "off" if on else "on", block=True)
      earlier.put_bool("TurnPilotCarSignalsForceOn", not on, block=True)
      if on:
        earlier.remove("TurnPilotCarSignalsVerified")
      else:
        earlier.put("TurnPilotCarSignalsVerified", sim_car_signals_record(), block=True)
      del earlier
      body = car_signals_at_start(Params(), on)  # a fresh handle, as BeamNGBridge's run does
      card = resolver(Params(), simulation=True)(VIN_UNKNOWN, [])
      assert body == on and (card.mode == 'on') == body, (on, body, card)


def test_game_lights_one_set_lights_per_change():
  """world.py's vehicle loop sends what GameLights.take() gives: beamngpy's Vehicle.set_lights message, on a change only."""
  lights = GameLights()
  assert lights.take() is None  # nothing wanted yet
  lights.set((False, False, False))
  assert lights.take() == {"type": "SetLights", "leftSignal": False, "rightSignal": False, "hazardSignal": False}
  lights.set((False, False, False))
  assert lights.take() is None
  lights.set((True, False, False))
  assert lights.take() == {"type": "SetLights", "leftSignal": True, "rightSignal": False, "hazardSignal": False}
  assert lights.take() is None
  lights.set((True, True, True))  # hazards: the game's hazard mode, not both indicators
  assert lights.take() == {"type": "SetLights", "leftSignal": False, "rightSignal": False, "hazardSignal": True}
  lights.failed()  # the vehicle link dropped it: sent again
  assert lights.take()["hazardSignal"]


class FakeLink:
  """The ego's BeamNGpy connection: records what the vehicle loop sends, ends the loop after `iterations`."""

  def __init__(self, world, iterations):
    self.world, self.left, self.sent = world, iterations, []

  def send(self, msg):
    self.sent.append(msg)
    if msg["type"] == "SensorRequest":
      self.left -= 1
      if self.left <= 0:
        self.world.stop.set()
    return SimpleNamespace(ack=lambda kind: None, recv=lambda kind: {"data": {"electrics": {}}})


def test_vehicle_loop_sends_set_lights_on_a_change_only():
  import threading
  from openpilot.tools.sim.bridge.beamng.world import BeamNGWorld
  world = BeamNGWorld.__new__(BeamNGWorld)
  world.stop, world.vlock, world.ctrl, world.el, world.log = threading.Event(), threading.Lock(), (0., 0., 0.), {'gear': 'D'}, print
  world.vehicle_rate, world.game_lights = 0., GameLights()
  world.electrics = SimpleNamespace(encode_vehicle_request=dict, decode_response=lambda r: {'gear': 'D'})
  link = FakeLink(world, 3)
  world.ego = SimpleNamespace(connection=link)
  world.game_lights.set((True, False, False))
  world.vehicle_loop()
  lights = [m for m in link.sent if m["type"] == "SetLights"]
  assert lights == [{"type": "SetLights", "leftSignal": True, "rightSignal": False, "hazardSignal": False}]
  assert sum(m["type"] == "Control" for m in link.sent) == 3


def test_bridge_keys_work_the_bodys_stalk_and_hazards():
  from openpilot.tools.sim.bridge.beamng.bridge import BeamNGBridge
  bridge = BeamNGBridge.__new__(BeamNGBridge)
  bridge.car, bridge.blinker = TeslaCan(car_signals=True, body=quiet_body()), (False, False)
  bridge.handle_command("blinker_left", None)
  assert bridge.car.stalk == (True, False) and bridge.blinker == (False, False)
  bridge.handle_command("blinker_left", None)
  assert bridge.car.stalk == (False, False)
  bridge.handle_command("hazard", None)
  assert bridge.car.body.hazard_button
  # without the car's signal: today's keyboard blinker, and no hazards
  bridge.car = TeslaCan()
  bridge.handle_command("blinker_right", None)
  assert bridge.blinker == (False, True)
  bridge.handle_command("hazard", None)
