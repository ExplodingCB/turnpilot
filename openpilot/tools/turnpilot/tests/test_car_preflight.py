"""car_preflight (CHASE_CAR.md, "Preflight tool spec") on synthetic logs built with cereal's messaging: a good car and Chase's
startup, an unknown EPS string, a 2-bit car, longitudinal off, a refusal after the tesla time, a panda that never reaches
tesla, no 0x3E9, a zero VIN, the big model down, the detector at its real cadence, slow and stopped, calibration
incomplete, a param this build's library lacks, a qlog and a sunnypilot log. Each case states the exact status of the lines
it is about and the exit status, live (the device's Params, faked) and from an rlog written to disk."""
import ast
import json
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.car.structs import car
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.values import CAR
from openpilot.cereal import messaging
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.turnpilot.detector.modeld_detector import FORCE_RUN_FRAMES, RUN_EVERY
from openpilot.tools.lib.logreader import save_log
from openpilot.tools.turnpilot import car_preflight as cpf
from openpilot.tools.turnpilot.car_preflight import GO, INFO, NO_GO, UNKNOWN, WARN

TOOL = Path(cpf.__file__)
REPO = Path(__file__).parents[4]

CHASE_EPS = b'TeM3_E014p10_0.0.0 (35),E014.21.2'
UNLISTED_EPS = b'TeM3_E014p10_0.0.0 (35),EL014.21.2'   # patch 0001's string: in no list at this pin
MODEL_Y_EPS = b'TeM3_E014p10_0.0.0 (35),Y002.22.2'
STOCK_3E9 = bytes.fromhex('f1882e000000f083')        # Chase's stock DAS_bodyControls, request NONE (ROUTE_FACTS.md)
STOCK_2B9 = b'\xaa' * 8                               # the Autopilot computer's own DAS_control, which the panda blocks
PARTY, VEHICLE, AP, RETURNED, REFUSED = 0, 1, 2, 128, 192
TESLA_T = 4.      # s: the panda enters tesla safety
LOADED_T = 3.     # s: the big model has loaded
FAIL_T = 6.       # s: the big model fails, in the 'failed' case
END_T = 12.       # s of log
CAN_DT = 0.05     # can, sendcan, carState and modelV2 at 20 Hz; the detections on every RUN_EVERY-th of those frames
DET_START_T = 0.5  # s: the detector's first run
OFFSET_S = 100.   # logMonoTime of t 0
LAT, LON = 39.765432, -86.123456  # a GPS position in every log, and in a param: never printed

TURNPILOT = {"branch": "tp/chase-car", "remote": "https://github.com/kaileh57/turnpilot.git", "commit": "be3c0ed7b81181d1b4daca5ed25aeb400a72f28f"}
SUNNYPILOT = {"branch": "staging-chestnut", "remote": "https://github.com/sunnypilot/openpilot.git", "commit": "820eaf287daf" + "0" * 28}

GOOD_PARAMS = {
  "TurnPilotEnabled": True, "NavigationEnabled": True, "AlphaLongitudinalEnabled": True, "DisengageOnAccelerator": True,
  "TurnPilotCarSignals": "off", "TurnPilotCameraLights": "shadow", "TurnPilotCameraStopSigns": "shadow",
  "TurnPilotStopSignGo": "vision", "TurnPilotAvoidStops": True, "TurnPilotCurbPedCap": True, "TurnPilotLeftRangeM": 0.,
  "TurnPilotShadowMode": False, "DisableUpdates": False, "TurnPilotAutoLeft": "auto", "TurnPilotAlwaysDrive": True,
  "TurnPilotMapHost": "maps.turnpilot.test", "TurnPilotMapKey": "not-a-real-key", "TurnPilotMapCert": "not-a-real-cert",
  "IsReleaseBranch": False,
}
SUNNYPILOT_PARAMS = {"AlphaLongitudinalEnabled": True, "DisengageOnAccelerator": False, "Mads": "1"}

REQUIRED_KEYS = [key for key, _, _ in cpf.REQUIRED]
MAP_KEYS = ["TurnPilotMapHost", "TurnPilotMapKey", "TurnPilotMapCert"]
GO_LINES = ["branch", "car", "EPS", "3-bit firmware", "DAS_settings", "longitudinal", "panda safety", "panda honours long",
            "driver's blinker", "car signals", "calibration", "road camera", "big model", "detector", "TurnPilot faults",
            "car health", *REQUIRED_KEYS, *MAP_KEYS]
INFO_LINES = ["fingerprint source", "turn-signal frame", "X179 add-on", "VIN", "TurnPilotShadowMode", "DisableUpdates",
              "TurnPilotLeftFarRangeM", "TurnPilotAutoLeft", "TurnPilotGoOverride", "TurnPilotNoMoveS", "TurnPilotCarSignalsVinless",
              "TurnPilotAlwaysDrive",
              "panda build", "Tesla software", "outside lamps", "mount"]
CAN_LINES = ["3-bit firmware", "panda honours long", "turn-signal frame", "driver's blinker", "X179 add-on"]
TURNPILOT_LINES = ["car signals", "detector", "TurnPilot faults"]

DEFAULTS = {
  "git": TURNPILOT, "params": GOOD_PARAMS, "eps": CHASE_EPS, "brand": "tesla", "fingerprint": "TESLA_MODEL_3", "source": "fw",
  "fuzzy": False, "dashcam": False, "flags": 8, "alpha_available": True, "long": True, "safety_param": 1, "vin": "0" * 17,
  "car_params": True,
  "tesla_t": TESLA_T,         # None: the panda never enters tesla safety
  "panda_param": None,        # the panda's safetyParam, when it isn't carParams'
  "three_bit": True, "body_controls": True, "ui_warning": True, "refuse_before": 0, "refuse_after": 0, "src0_before": False,
  "rx_invalid": False, "can": True, "can_keep": 1, "sendcan": True, "can_valid_drop": False,
  "big": "ok",                # ok, false (never up), failed (bigModelFailed at FAIL_T), loading (still loading at the end)
  "supply_fault": False, "calibration": "calibrated", "sensor": "os04c10", "events": (),
  "model": True, "detections": True, "plan_fault": "none",
  "det_every": RUN_EVERY,     # the detector's full pass on every this-many model frames (modeld_detector.RUN_EVERY parked)
  "det_skip": (),             # (start, end) s: frames where the detector doesn't run (room() refusing, a retry): the run waits
}


def stamped(msg, t):
  msg.logMonoTime = int(round((t + OFFSET_S) * 1e9))
  return msg.as_reader()


def ticks(dt, start=0., end=END_T):
  return [round(start + i * dt, 6) for i in range(int(round((end - start) / dt)))]


def raw_param(key, value):
  if key in cpf.DONT_LOG:
    return b""  # loggerd logs a DONT_LOG key with no value (logger.cc)
  if isinstance(value, bool):
    return b"1" if value else b"0"
  if isinstance(value, dict):
    return json.dumps(value).encode()  # a JSON param, as Params stores it
  return str(value).encode()


def fill_car_params(cp, c):
  cp.brand, cp.carFingerprint, cp.fingerprintSource, cp.fuzzyFingerprint = c.brand, c.fingerprint, c.source, c.fuzzy
  cp.dashcamOnly, cp.flags, cp.carVin = c.dashcam, c.flags, c.vin
  cp.alphaLongitudinalAvailable, cp.openpilotLongitudinalControl = c.alpha_available, c.long
  safety = cp.init('safetyConfigs', 1)
  safety[0].safetyModel = 'noOutput' if c.dashcam else 'tesla'
  safety[0].safetyParam = c.safety_param
  fw = cp.init('carFw', 1 if c.eps else 0)
  if c.eps:
    fw[0].ecu, fw[0].address, fw[0].fwVersion, fw[0].brand = 'eps', 0x730, c.eps, 'tesla'


def can_msg(service, frames, t):
  msg = messaging.new_message(service, len(frames))
  for f, (address, dat, src) in zip(getattr(msg, service), frames, strict=True):
    f.address, f.dat, f.src = address, dat, src
  return stamped(msg, t)


def drive(**kw) -> list:
  """A parked car switched on: the panda in elm327 (the harness relay puts the Autopilot side on party CAN) until TESLA_T,
  openpilot's 0x2B9 from 0.2 s before it, the big model up at LOADED_T."""
  c = SimpleNamespace(**{**DEFAULTS, **kw})
  tt = c.tesla_t

  def tesla(t):
    return tt is not None and t >= tt - 1e-9

  turnpilot_log = c.git is not SUNNYPILOT
  msgs = []

  init = messaging.new_message('initData')
  d = init.initData
  d.gitBranch, d.gitRemote, d.gitCommit = c.git["branch"], c.git["remote"], c.git["commit"]
  items = list(c.params.items()) + [("LastGPSPosition", json.dumps({"latitude": LAT, "longitude": LON}))]
  entries = d.init('params').init('entries', len(items))
  for e, (key, value) in zip(entries, items, strict=True):
    e.key, e.value = key, raw_param(key, value)
  msgs.append(stamped(init, 0.))

  if c.car_params:
    cp = messaging.new_message('carParams')
    fill_car_params(cp.carParams, c)
    msgs.append(stamped(cp, 0.5))

  for t in ticks(0.1):
    msg = messaging.new_message('pandaStates', 1)
    p = msg.pandaStates[0]
    p.safetyModel, p.safetyParam = ('tesla', c.safety_param if c.panda_param is None else c.panda_param) if tesla(t) else ('elm327', 0)
    p.ignitionCan, p.harnessStatus = True, 'normal'
    p.safetyRxChecksInvalid = c.rx_invalid and tesla(t)
    msgs.append(stamped(msg, t))

  sends = refused_before = refused_after = 0
  since_run = c.det_every  # modeld_detector: the first frame runs, then every det_every-th; a frame it can't run on delays the next
  for i, t in enumerate(ticks(CAN_DT)):
    ap = AP if tesla(t) else PARTY
    frames = [(0x257, b'\x00' * 8, PARTY)]
    if c.three_bit:
      frames += [(0x489, b'\x00' * 8, ap), (0x054, b'\x00' * 8, PARTY)]
    if i % 10 == 0:
      frames.append((0x293, b'\x00' * 8, ap))
    if c.ui_warning and i % 2 == 0:
      frames.append((0x311, b'\x00' * 7, PARTY))
    if c.body_controls and i % 10 == 0:
      frames.append((0x3E9, STOCK_3E9, AP if tesla(t) or not c.src0_before else PARTY))
      if tesla(t):
        frames.append((0x3E9, STOCK_3E9, RETURNED))  # the panda forwarding the stock frame from bus 2 to bus 0
    if tesla(t):
      frames.append((0x2B9, STOCK_2B9, AP))
    if c.sendcan and c.long and tt is not None and t >= tt - 0.2 - 1e-9:
      dat = bytes([sends & 0xff, sends >> 8 & 0xff]) + b'\x00' * 6
      sends += 1
      msgs.append(can_msg('sendcan', [(0x2B9, dat, PARTY)], t))
      if not tesla(t):
        if refused_before < c.refuse_before:
          frames.append((0x2B9, dat, REFUSED))
          refused_before += 1
      elif refused_after < c.refuse_after:
        frames.append((0x2B9, dat, REFUSED))
        refused_after += 1
      else:
        frames.append((0x2B9, dat, RETURNED))
    if c.can and i % c.can_keep == 0:
      msgs.append(can_msg('can', frames, t))

    cs = messaging.new_message('carState')
    cs.carState.canValid = t >= 1. and not (c.can_valid_drop and 8. <= t < 8.5)  # false at startup, before the tesla time
    cs.carState.carSignal.status = 'off'
    msgs.append(stamped(cs, t))

    if c.model:
      m = messaging.new_message('modelV2')
      up = {"ok": t >= LOADED_T, "false": False, "failed": LOADED_T <= t < FAIL_T, "loading": False}[c.big]
      m.modelV2.big, m.modelV2.modelExecutionTime = up, 0.035
      msgs.append(stamped(m, t))
    if turnpilot_log and c.detections and t >= DET_START_T - 1e-9:
      since_run += 1
      if since_run >= c.det_every and not any(a - 1e-9 <= t < b - 1e-9 for a, b in c.det_skip):
        since_run = 0
        det = messaging.new_message('turnpilotDetections')
        det.turnpilotDetections.executionTime, det.turnpilotDetections.camera = 0.021, 'road'
        det.turnpilotDetections.lanePaintStatus, det.turnpilotDetections.lightStatus = 'read', 'read'
        msgs.append(stamped(det, t))
    plan = messaging.new_message('turnpilotPlan')  # on a sunnypilot log, sunnypilot's own message on this slot
    plan.turnpilotPlan.state, plan.turnpilotPlan.fault, plan.turnpilotPlan.shadow = 'following', c.plan_fault, True
    msgs.append(stamped(plan, t))

  for t in ticks(1.):
    names = list(c.events)
    if t < LOADED_T or c.big == "loading":
      names.append('bigModelLoading')
    if c.big == "failed" and t == FAIL_T:
      names.append('bigModelFailed')
    ev = messaging.new_message('onroadEvents', len(names))
    for e, n in zip(ev.onroadEvents, names, strict=True):
      e.name = n
    msgs.append(stamped(ev, t))
  for t in ticks(0.5):
    ds = messaging.new_message('deviceState')
    ds.deviceState.chestnutPresent = True
    ch = messaging.new_message('chestnutState')
    ch.chestnutState.supplyVoltage, ch.chestnutState.pcieLtssm, ch.chestnutState.tempC = 14300, 0x78, 45.
    ch.chestnutState.supplyFault = c.supply_fault
    cam = messaging.new_message('narrowRoadCameraState')
    cam.narrowRoadCameraState.sensor = c.sensor
    gps = messaging.new_message('gpsLocationExternal')
    gps.gpsLocationExternal.latitude, gps.gpsLocationExternal.longitude = LAT, LON
    msgs += [stamped(ds, t), stamped(ch, t), stamped(cam, t), stamped(gps, t)]
  for t in ticks(0.25):
    cal = messaging.new_message('extrinsicsCalibration')
    x = cal.extrinsicsCalibration
    x.calStatus = c.calibration
    x.calPerc, x.validBlocks = (100, 35) if c.calibration == 'calibrated' else (40, 14)
    x.rpyCalib, x.height = [0., 0.0349, -0.0175], [1.22]
    msgs.append(stamped(cal, t))
  if turnpilot_log and c.detections:
    for t in ticks(0.1, 0.5):
      msgs.append(stamped(messaging.new_message('turnpilotFarDetections'), t))
  return sorted(msgs, key=lambda m: m.logMonoTime)


class FakeParams:
  """The device's Params for check_live: values as Params.get returns them, types and defaults from this build's library,
  keys a stale library wouldn't know, and no way to write."""
  def __init__(self, tmp_path, values, unknown=()):
    self.lib = Params(str(tmp_path / "params"))
    self.values, self.unknown = dict(values), set(unknown)

  def check_key(self, key):
    if key in self.unknown:
      raise UnknownKeyName(key)
    return self.lib.check_key(key)

  def get_type(self, key):
    self.check_key(key)
    return self.lib.get_type(key)

  def get_default_value(self, key):
    self.check_key(key)
    return self.lib.get_default_value(key)

  def get(self, key, block=False, return_default=False):
    self.check_key(key)
    return self.values.get(key)

  def get_bool(self, key, block=False):
    return bool(self.get(key))

  def cpp2python(self, key, value):
    self.check_key(key)
    return self.lib.cpp2python(key, value)

  def put(self, *args, **kwargs):
    raise AssertionError("car_preflight wrote a param")

  put_bool = put_nonblocking = remove = clear_all = put


def git_params(git):
  return {"GitBranch": git["branch"], "GitCommit": git["commit"], "GitRemote": git["remote"]}


def run_live(tmp_path, params=None, unknown=(), extra=None, **kw):
  git = kw.get("git", TURNPILOT)
  values = {**(GOOD_PARAMS if params is None else params), **git_params(git), **(extra or {})}
  return cpf.check_live(END_T, params=FakeParams(tmp_path, values, unknown), messages=drive(**kw))


def write(tmp_path, msgs, name="rlog.zst"):
  path = tmp_path / name
  save_log(str(path), msgs)
  return path


def run_route(tmp_path, name="rlog.zst", lib=None, **kw):
  return cpf.check_route([str(write(tmp_path, drive(**kw), name))], lib=lib)


def statuses(lines):
  out = {}
  for line in lines:
    assert line.name not in out, f"two lines named {line.name}"
    assert line.status in (GO, NO_GO, WARN, UNKNOWN, INFO)
    out[line.name] = line.status
  return out


def line(lines, name):
  return next(x for x in lines if x.name == name)


def assert_good(lines, unknown=()):
  s = statuses(lines)
  for name in GO_LINES:
    assert s[name] == (UNKNOWN if name in unknown else GO), (name, line(lines, name))
  for name in INFO_LINES:
    assert s[name] == INFO, (name, line(lines, name))
  assert set(s) <= set(GO_LINES) | set(INFO_LINES) | {"EPS since route", "0x2B9 before tesla"}, set(s) - set(GO_LINES) - set(INFO_LINES)


# --------------------------------------------------------------------------------------------------------- a good car

def test_good_car_live(tmp_path):
  lines = run_live(tmp_path)
  assert_good(lines)
  assert cpf.exit_code(lines) == 0
  assert "TESLA_MODEL_3" in line(lines, "car").detail and "fw" in line(lines, "car").detail
  assert repr(CHASE_EPS) in line(lines, "EPS").detail
  assert "EPS since route" not in statuses(lines)
  assert "not-a-real-key" not in cpf.render(lines) and "not-a-real-cert" not in cpf.render(lines)  # DONT_LOG: never printed
  assert "(default)" in line(lines, "TurnPilotLeftFarRangeM").detail  # unset on the device: the library's default, said


def test_good_car_route(tmp_path):
  """From a route the map key and certificate are DONT_LOG (params_keys.h): UNKNOWN, so a route never exits 0."""
  lines = run_route(tmp_path)
  assert_good(lines, unknown=("TurnPilotMapKey", "TurnPilotMapCert"))
  assert "not logged" in line(lines, "TurnPilotMapKey").detail
  assert cpf.exit_code(lines) == 2


def test_chase_startup(tmp_path):
  """Chase's route: three 0x2B9 refused and the stock 0x3E9 on src 0 before the tesla time, carState.canValid false at card's
  start, and none of it after: GO, with an INFO line for the refusals."""
  lines = run_live(tmp_path, refuse_before=3, src0_before=True)
  s = statuses(lines)
  assert s["panda honours long"] == GO
  assert "back on src 128" in line(lines, "panda honours long").detail
  assert s["0x2B9 before tesla"] == INFO and "3 refused" in line(lines, "0x2B9 before tesla").detail
  assert s["turn-signal frame"] == INFO and "2.0 Hz" in line(lines, "turn-signal frame").detail
  assert s["car health"] == GO and s["panda safety"] == GO
  assert cpf.exit_code(lines) == 0


def test_listed_eps_other_than_chases(tmp_path):
  lines = run_live(tmp_path, eps=b'TeM3_E014p10_0.0.0 (24),E014.20.2')
  assert statuses(lines)["EPS"] == GO
  assert statuses(lines)["EPS since route"] == INFO and repr(CHASE_EPS) in line(lines, "EPS since route").detail
  assert cpf.exit_code(lines) == 0


# ------------------------------------------------------------------------------------------------- the car and its EPS

def test_unknown_eps(tmp_path):
  """An unlisted string: opendbc picks MOCK, dashcam mode, safety noOutput, so the panda never enters tesla."""
  lines = run_route(tmp_path, brand="mock", fingerprint="MOCK", dashcam=True, eps=UNLISTED_EPS, tesla_t=None, long=False,
                    alpha_available=False, safety_param=0, flags=0)
  s = statuses(lines)
  assert s["car"] == NO_GO and repr(UNLISTED_EPS) in line(lines, "car").detail
  assert "fingerprints.py" in line(lines, "car").fix
  assert s["EPS"] == NO_GO and repr(UNLISTED_EPS) in line(lines, "EPS").detail and "no fingerprint list" in line(lines, "EPS").detail
  assert "fingerprints.py" in line(lines, "EPS").fix
  assert cpf.exit_code(lines) == 1


def test_eps_listed_for_another_platform(tmp_path):
  lines = run_live(tmp_path, eps=MODEL_Y_EPS)
  assert statuses(lines)["EPS"] == NO_GO and "TESLA_MODEL_Y" in line(lines, "EPS").detail
  assert cpf.exit_code(lines) == 1


def test_no_eps_answer(tmp_path):
  lines = run_live(tmp_path, eps=b"")
  assert statuses(lines)["EPS"] == NO_GO and "no EPS answer" in line(lines, "EPS").detail


def test_two_bit_car(tmp_path):
  """No 0x489 on src 2 or 0x054 on src 0 in 8 s of can carrying DI_speed after the tesla time: NO-GO."""
  lines = run_live(tmp_path, three_bit=False)
  assert statuses(lines)["3-bit firmware"] == NO_GO and "of can with DI_speed" in line(lines, "3-bit firmware").detail
  assert cpf.exit_code(lines) == 1
  short = run_live(tmp_path, three_bit=False, tesla_t=9.)  # 3 s of can after the tesla time: too little to call it absent
  assert statuses(short)["3-bit firmware"] == UNKNOWN


def test_two_bit_car_in_dashcam_mode(tmp_path):
  """As opendbc leaves it: dashcamOnly, passive, so the panda stays out of tesla and the can lines can't judge."""
  lines = run_live(tmp_path, three_bit=False, dashcam=True, tesla_t=None)
  s = statuses(lines)
  assert s["car"] == NO_GO and "dashcam" in line(lines, "car").detail and "software" in line(lines, "car").fix
  assert s["3-bit firmware"] == UNKNOWN and "never entered tesla" in line(lines, "3-bit firmware").detail
  assert cpf.exit_code(lines) == 1


def test_fixed_fingerprint_on_this_fork(tmp_path):
  lines = run_live(tmp_path, source="fixed")
  assert statuses(lines)["fingerprint source"] == WARN and "FINGERPRINT" in line(lines, "fingerprint source").detail
  assert statuses(lines)["car"] == GO
  assert cpf.exit_code(lines) == 0


def test_car_params_from_the_last_drive(tmp_path):
  """Live with no carParams in the window (card logs it every 50 s): CarParams, else CarParamsPersistent, said."""
  cp = car.CarParams.new_message()
  fill_car_params(cp, SimpleNamespace(**DEFAULTS))
  lines = run_live(tmp_path, car_params=False, extra={"CarParamsPersistent": cp.to_bytes()})
  assert statuses(lines)["car"] == GO and "last drive" in line(lines, "car").detail
  assert cpf.exit_code(lines) == 0
  none = run_live(tmp_path, car_params=False)
  assert statuses(none)["car"] == UNKNOWN and statuses(none)["EPS"] == UNKNOWN and statuses(none)["longitudinal"] == UNKNOWN


# ----------------------------------------------------------------------------------------------- longitudinal, the panda

def test_longitudinal_off(tmp_path):
  lines = run_live(tmp_path, params={**GOOD_PARAMS, "AlphaLongitudinalEnabled": False}, long=False, safety_param=0,
                   plan_fault="noLongitudinal")
  s = statuses(lines)
  assert s["longitudinal"] == NO_GO and "AlphaLongitudinalEnabled" in line(lines, "longitudinal").detail
  assert "alpha" in line(lines, "longitudinal").fix
  assert s["AlphaLongitudinalEnabled"] == NO_GO
  assert s["panda honours long"] == UNKNOWN
  assert s["TurnPilot faults"] == NO_GO and "noLongitudinal" in line(lines, "TurnPilot faults").detail
  assert cpf.exit_code(lines) == 1


def test_refused_after_tesla(tmp_path):
  lines = run_route(tmp_path, refuse_after=2)
  assert statuses(lines)["panda honours long"] == NO_GO
  assert "refused 2" in line(lines, "panda honours long").detail and line(lines, "panda honours long").fix
  assert cpf.exit_code(lines) == 1


def test_panda_never_reaches_tesla(tmp_path):
  lines = run_live(tmp_path, tesla_t=None)
  s = statuses(lines)
  assert s["panda safety"] == UNKNOWN and "elm327" in line(lines, "panda safety").detail
  for name in CAN_LINES + ["car health"]:
    assert s[name] == UNKNOWN, name
  assert cpf.exit_code(lines) == 2


def test_panda_safety_param_and_rx_checks(tmp_path):
  mismatch = run_live(tmp_path, panda_param=0)
  assert statuses(mismatch)["panda safety"] == NO_GO and "carParams" in line(mismatch, "panda safety").detail
  rx = run_live(tmp_path, rx_invalid=True)
  assert statuses(rx)["panda safety"] == NO_GO and "safetyRxChecksInvalid" in line(rx, "panda safety").detail


def test_can_valid_drop_after_tesla(tmp_path):
  lines = run_live(tmp_path, can_valid_drop=True)
  assert statuses(lines)["car health"] == NO_GO and "canValid false" in line(lines, "car health").detail


def test_health_events(tmp_path):
  lines = run_live(tmp_path, events=("invalidLkasSetting",))
  assert statuses(lines)["car health"] == NO_GO and "Autosteer" in line(lines, "car health").fix


def live_from(tmp_path, msgs):
  return cpf.check_live(END_T, params=FakeParams(tmp_path, {**GOOD_PARAMS, **git_params(TURNPILOT)}), messages=msgs)


def test_health_needs_its_evidence(tmp_path):
  """car health is GO only on carState after the tesla time and onroadEvents to read the events from: a missing fact is
  UNKNOWN, never GO (CHASE_CAR.md, Output). selfdrived sends onroadEvents every second while onroad, empty on a healthy car
  (selfdrived.py:574-579), and an empty one is evidence."""
  tesla_ns = (OFFSET_S + TESLA_T) * 1e9
  good = drive()
  assert all(len(m.onroadEvents) == 0 for m in good if m.which() == "onroadEvents" and m.logMonoTime >= tesla_ns)
  assert statuses(live_from(tmp_path, good))["car health"] == GO

  early = live_from(tmp_path, [m for m in good if not (m.which() == "carState" and m.logMonoTime >= tesla_ns)])
  assert statuses(early)["car health"] == UNKNOWN and "no carState after the tesla time" in line(early, "car health").detail
  assert cpf.exit_code(early) == 2

  for title, msgs in (("none", [m for m in good if m.which() != "onroadEvents"]),
                      ("only before the tesla time", [m for m in good if not (m.which() == "onroadEvents" and m.logMonoTime >= tesla_ns)])):
    quiet = live_from(tmp_path, msgs)
    assert statuses(quiet)["car health"] == UNKNOWN, (title, line(quiet, "car health"))
    assert "no onroadEvents" in line(quiet, "car health").detail and "none of" not in line(quiet, "car health").detail
    assert cpf.exit_code(quiet) == 2, title

  dropped = drive(can_valid_drop=True)
  bad = live_from(tmp_path, [m for m in dropped if m.which() != "onroadEvents"])  # a fault seen still wins over the events unread
  assert statuses(bad)["car health"] == NO_GO and "canValid false" in line(bad, "car health").detail


# ----------------------------------------------------------------------------------------------------- the turn signal

def test_no_body_controls(tmp_path):
  lines = run_live(tmp_path, body_controls=False)
  assert statuses(lines)["turn-signal frame"] == INFO and "absent" in line(lines, "turn-signal frame").detail
  assert cpf.exit_code(lines) == 0


def test_zero_vin(tmp_path):
  lines = run_live(tmp_path)
  assert statuses(lines)["VIN"] == INFO and "all zeros" in line(lines, "VIN").detail
  assert "VIN is unreadable" in line(lines, "turn-signal frame").detail
  assert cpf.exit_code(lines) == 0


def valid_vin(stem="5YJ3E1EA_HF000337"):
  from openpilot.selfdrive.turnpilot.car_signals import vin_valid
  return next(v for v in (stem.replace("_", d) for d in "0123456789X") if vin_valid(v))


def test_valid_vin(tmp_path):
  vin = valid_vin()
  lines = run_live(tmp_path, vin=vin)
  assert statuses(lines)["VIN"] == INFO and "valid" in line(lines, "VIN").detail
  assert vin not in cpf.render(lines)  # never printed
  assert "VIN is unreadable" not in line(lines, "turn-signal frame").detail
  assert cpf.exit_code(lines) == 0


def test_car_signals_on(tmp_path):
  """Mode on with a verified record: record_matches' reason, on a Tesla's zero VIN "Car's VIN unreadable"."""
  record = {"vin": "0" * 17, "fw": [], "exteriorConfirmed": "2026-10-30T12:00:00+00:00"}
  lines = run_live(tmp_path, params={**GOOD_PARAMS, "TurnPilotCarSignals": "on", "TurnPilotCarSignalsVerified": record})
  s = statuses(lines)
  assert s["car signals"] == WARN and "VIN unreadable" in line(lines, "car signals").detail
  assert s["TurnPilotCarSignals"] == NO_GO


DONGLE = "0123456789abcdef"
UPDATED_EPS = b'TeM3_E014p10_0.0.0 (24),E014.20.2'  # another TESLA_MODEL_3 string: a Tesla update
DID = "Car's VIN unreadable: can't match the record"


def signal_record(vin="0" * 17, eps=CHASE_EPS, **kw):
  """A verified record as promote() writes it from a check on this car (fill_car_params' carFw): with the identity a check
  keeps under TurnPilotCarSignalsVinless."""
  from openpilot.selfdrive.turnpilot.car_signals import fw_list
  cp = car.CarParams.new_message()
  fill_car_params(cp, SimpleNamespace(**{**DEFAULTS, "eps": eps, "vin": vin}))
  r = {"onReason": 9, "gear": "P", "onSeparates": False, "vin": vin, "fw": fw_list(cp.carFw), "checkId": "c0ffee00",
       "exteriorConfirmed": "2026-10-30T12:00:00+00:00", "when": "2026-10-30T11:58:00+00:00", "opendbc": "",
       "dongleId": DONGLE, "carFingerprint": "TESLA_MODEL_3"}
  r.update(kw)
  return r


def signals_on(record, vinless=None, **extra):
  return {**GOOD_PARAMS, "TurnPilotCarSignals": "on", "TurnPilotCarSignalsVerified": record, "DongleId": DONGLE, **extra,
          **({} if vinless is None else {"TurnPilotCarSignalsVinless": vinless})}


def test_car_signals_say_which_way_the_record_matched(tmp_path):
  """Mode on: the car signals line says whether the verified record matches and which way, by VIN or without one
  (TurnPilotCarSignalsVinless), live and from a route, and never prints the DongleId or the VIN."""
  vin = valid_vin()
  cases = [
    (signals_on(signal_record()), {}, f"does not match: {DID} (TurnPilotCarSignalsVinless off: a record matches by VIN only)"),
    (signals_on(signal_record(), False), {}, f"does not match: {DID} (TurnPilotCarSignalsVinless off: a record matches by VIN only)"),
    (signals_on(signal_record(), True), {}, "matches without a VIN (TurnPilotCarSignalsVinless): this comma's DongleId, " +
                                             "TESLA_MODEL_3 by its firmware and the exact carFw list"),
    (signals_on(signal_record(vin)), {"vin": vin}, "matches by VIN and firmware"),
    (signals_on(signal_record(vin), True), {"vin": vin}, "matches by VIN and firmware"),
    (signals_on(signal_record(), True), {"vin": vin}, "does not match: Recorded for another car: run the check"),
    (signals_on(signal_record(), True, DongleId="fedcba9876543210"), {}, "does not match: Recorded on another comma: run the check"),
    (signals_on(signal_record(), True), {"eps": UPDATED_EPS}, "does not match: Car software changed: run the check again"),
    (signals_on(signal_record(gear="D"), True), {}, "does not match: Checked in Drive: without a VIN, check in P"),
  ]
  for params, kw, said in cases:
    for lines in (run_live(tmp_path, params=params, **kw), run_route(tmp_path, params=params, **kw)):
      s = statuses(lines)
      detail = line(lines, "car signals").detail
      assert s["car signals"] == WARN and detail.startswith(f"on: Saturday plans for off; the verified record {said}"), (said, detail)
      out = cpf.render(lines)
      assert DONGLE not in out and "fedcba9876543210" not in out and vin not in out


def test_the_vinless_opt_in_line(tmp_path):
  """TurnPilotCarSignalsVinless: INFO while off (the default), WARN on (an owner's opt-in that loosens which car a record
  fits), NO-GO for a library that lacks it; the VIN and turn-signal frame lines say what a zero VIN can match."""
  lines = run_live(tmp_path)
  assert statuses(lines)["TurnPilotCarSignalsVinless"] == INFO
  assert line(lines, "TurnPilotCarSignalsVinless").detail.startswith("0 (default): a car-signal record matches by VIN only")
  assert "a car-signal record can't match" in line(lines, "VIN").detail
  on = run_live(tmp_path, params={**GOOD_PARAMS, "TurnPilotCarSignalsVinless": True})
  s = statuses(on)
  assert s["TurnPilotCarSignalsVinless"] == WARN and "without a VIN" in line(on, "TurnPilotCarSignalsVinless").detail
  assert "TurnPilotCarSignalsVinless=0" in line(on, "TurnPilotCarSignalsVinless").fix
  assert s["VIN"] == INFO and "matches only without a VIN (TurnPilotCarSignalsVinless on)" in line(on, "VIN").detail
  assert "VIN is unreadable" not in line(on, "turn-signal frame").detail
  assert "only from a record made without a VIN" in line(on, "turn-signal frame").detail
  assert cpf.exit_code(on) == 0  # a WARN: read and decided, as every WARN
  stale = run_live(tmp_path, unknown={"TurnPilotCarSignalsVinless"})
  assert statuses(stale)["TurnPilotCarSignalsVinless"] == NO_GO


def test_a_check_on_a_zero_vin_says_whether_its_record_can_match(tmp_path):
  off = run_live(tmp_path, params={**GOOD_PARAMS, "TurnPilotCarSignals": "check"})
  assert "its record can't match this car's zero VIN (TurnPilotCarSignalsVinless off)" in line(off, "car signals").detail
  on = run_live(tmp_path, params={**GOOD_PARAMS, "TurnPilotCarSignals": "check", "TurnPilotCarSignalsVinless": True})
  assert "its record keeps this comma's DongleId and the firmware's fingerprint (TurnPilotCarSignalsVinless on)" in \
    line(on, "car signals").detail
  assert statuses(on)["car signals"] == WARN


def test_no_ui_warning(tmp_path):
  lines = run_live(tmp_path, ui_warning=False)
  assert statuses(lines)["driver's blinker"] == NO_GO


# --------------------------------------------------------------------------------------------- the big model, the camera

def test_big_model_false_after_loading(tmp_path):
  lines = run_live(tmp_path, big="false")
  assert statuses(lines)["big model"] == NO_GO and "false" in line(lines, "big model").detail
  assert "restart the car" in line(lines, "big model").fix and "12 V" in line(lines, "big model").fix
  assert cpf.exit_code(lines) == 1


def test_big_model_failed(tmp_path):
  lines = run_route(tmp_path, big="failed")
  assert statuses(lines)["big model"] == NO_GO and "bigModelFailed" in line(lines, "big model").detail


def test_big_model_still_loading(tmp_path):
  lines = run_live(tmp_path, big="loading")
  assert statuses(lines)["big model"] == UNKNOWN and "loading" in line(lines, "big model").detail
  assert cpf.exit_code(lines) == 2


def test_chestnut_supply_fault(tmp_path):
  lines = run_live(tmp_path, supply_fault=True)
  assert statuses(lines)["big model"] == NO_GO and "supplyFault" in line(lines, "big model").detail


def test_other_camera(tmp_path):
  lines = run_live(tmp_path, sensor="ar0231")
  assert statuses(lines)["road camera"] == NO_GO


# ------------------------------------------------------------------------------------------------------------ the detector

def test_detector_at_its_cadence(tmp_path):
  """Parked, modeld runs the detector on every RUN_EVERY-th 20 Hz model frame (modeld_detector.py:93, 10 Hz): GO at exactly
  that rate."""
  lines = run_live(tmp_path)
  assert statuses(lines)["detector"] == GO, line(lines, "detector")
  assert "10.0 Hz" in line(lines, "detector").detail
  assert cpf.exit_code(lines) == 0


def test_detector_one_run_put_off(tmp_path):
  """A frame room() refuses puts the run off to the next frame (modeld_detector.py:606-616): one in the window is GO, not the
  9.9 Hz NO-GO an 18 / 10 Hz band gave."""
  lines = run_live(tmp_path, det_skip=((8., 8. + CAN_DT),))
  assert statuses(lines)["detector"] == GO, line(lines, "detector")
  assert cpf.exit_code(lines) == 0


def test_detector_slow(tmp_path):
  """Runs put off on many frames: WARN under 0.9 of the detector's rate, NO-GO under half of it. room()'s own floor, a forced
  run every FORCE_RUN_FRAMES with no gap near the deadline, is NO-GO."""
  slow = run_live(tmp_path, det_every=RUN_EVERY + 1)
  assert statuses(slow)["detector"] == WARN, line(slow, "detector")
  assert cpf.exit_code(slow) == 0
  floor = run_live(tmp_path, det_every=FORCE_RUN_FRAMES)
  assert statuses(floor)["detector"] == NO_GO, line(floor, "detector")
  assert "under" in line(floor, "detector").detail and line(floor, "detector").fix
  assert cpf.exit_code(floor) == 1


def test_detector_gap(tmp_path):
  """No detections for over DETECTOR_DEADLINE_S, turnpilotd's handback: NO-GO."""
  lines = run_route(tmp_path, det_skip=((6., 6. + cpf.DETECTOR_DEADLINE_S + 0.1),))
  assert statuses(lines)["detector"] == NO_GO and "gap" in line(lines, "detector").detail
  assert cpf.exit_code(lines) == 1


# ------------------------------------------------------------------------------------------------------- calibration

@pytest.mark.parametrize("status, words", [("uncalibrated", "won't engage"), ("recalibrating", "won't engage"), ("invalid", "mount")])
def test_calibration_incomplete(tmp_path, status, words):
  lines = run_live(tmp_path, calibration=status, events=("calibrationIncomplete",))
  s = statuses(lines)
  assert s["calibration"] == NO_GO and words in line(lines, "calibration").detail + line(lines, "calibration").fix
  assert s["car health"] == NO_GO and "calibrationIncomplete" in line(lines, "car health").detail
  assert cpf.exit_code(lines) == 1


# ---------------------------------------------------------------------------------------------------------- params

def test_param_missing_from_the_library(tmp_path):
  """FK-16: a libparams built before the key. Live, the device's; from a route, the library this tool runs with."""
  lines = run_live(tmp_path, unknown={"TurnPilotCurbPedCap"})
  assert statuses(lines)["TurnPilotCurbPedCap"] == NO_GO and "unknown" in line(lines, "TurnPilotCurbPedCap").detail
  assert cpf.exit_code(lines) == 1
  route = run_route(tmp_path, lib=FakeParams(tmp_path, {}, {"TurnPilotCurbPedCap"}))
  assert statuses(route)["TurnPilotCurbPedCap"] == NO_GO


def test_param_wrong_or_default(tmp_path):
  params = GOOD_PARAMS | {"TurnPilotStopSignGo": "auto", "TurnPilotValhallaTiles": "/data/tiles", "TurnPilotShadowMode": True,
                          "DisableUpdates": True}
  lines = run_live(tmp_path, params=params)
  s = statuses(lines)
  assert s["TurnPilotStopSignGo"] == NO_GO and "auto, Saturday needs vision" in line(lines, "TurnPilotStopSignGo").detail
  assert "TurnPilotStopSignGo=vision" in line(lines, "TurnPilotStopSignGo").fix
  assert s["TurnPilotValhallaTiles"] == NO_GO
  assert s["TurnPilotShadowMode"] == WARN and "nothing acts" in line(lines, "TurnPilotShadowMode").detail
  assert "TurnPilotShadowMode=0" in line(lines, "TurnPilotShadowMode").fix
  assert s["DisableUpdates"] == WARN and "no fix pushed" in line(lines, "DisableUpdates").detail
  assert "DisableUpdates=0" in line(lines, "DisableUpdates").fix
  assert cpf.exit_code(lines) == 1


def test_the_saturday_values_are_the_defaults(tmp_path):
  """A fresh install's params (params_keys.h's defaults, test_saturday_defaults.py) read as Saturday's: TurnPilotStopSignGo
  vision and TurnPilotShadowMode 0 among them, with no NO-GO that Saturday needs off and no WARN that live is wrong."""
  unset = dict.fromkeys(("TurnPilotEnabled", "NavigationEnabled", "AlphaLongitudinalEnabled", "DisengageOnAccelerator",
                         "TurnPilotStopSignGo", "TurnPilotShadowMode", "DisableUpdates", "TurnPilotAlwaysDrive"))
  lines = run_live(tmp_path, params=GOOD_PARAMS | unset)
  s = statuses(lines)
  for key in ("TurnPilotEnabled", "NavigationEnabled", "AlphaLongitudinalEnabled", "DisengageOnAccelerator", "TurnPilotStopSignGo"):
    assert s[key] == GO and "(default)" in line(lines, key).detail, line(lines, key)
  assert line(lines, "TurnPilotStopSignGo").detail.startswith("vision")
  for key in ("TurnPilotShadowMode", "DisableUpdates", "TurnPilotAlwaysDrive"):
    assert s[key] == INFO and "(default)" in line(lines, key).detail, line(lines, key)
  assert "live" in line(lines, "TurnPilotShadowMode").detail
  assert line(lines, "DisableUpdates").detail.startswith("0 (default): updates on")


def test_stop_sign_go_off_is_saturdays_fallback_warned(tmp_path):
  """off is Saturday's value only when preflight's stop-sign go line WARNs (the side views missing): a WARN, never a NO-GO."""
  lines = run_live(tmp_path, params=GOOD_PARAMS | {"TurnPilotStopSignGo": "off"})
  got = line(lines, "TurnPilotStopSignGo")
  assert got.status == WARN and "hands back" in got.detail and "TurnPilotStopSignGo=vision" in got.fix


def test_always_drive_is_printed_and_off_warns(tmp_path):
  """TurnPilotAlwaysDrive (ALWAYS_DRIVE.md section 1; CHASE_CAR_CHECKLIST step 14's read-back): printed, INFO on (set, or unset
  and so the default, 1), and a WARN off, with the fix. Never NO-GO: off is today's behaviour."""
  lines = run_live(tmp_path)
  assert statuses(lines)["TurnPilotAlwaysDrive"] == INFO and line(lines, "TurnPilotAlwaysDrive").detail.startswith("1:")
  lines = run_live(tmp_path, params={k: v for k, v in GOOD_PARAMS.items() if k != "TurnPilotAlwaysDrive"})
  assert statuses(lines)["TurnPilotAlwaysDrive"] == INFO and line(lines, "TurnPilotAlwaysDrive").detail.startswith("1 (default):")
  lines = run_live(tmp_path, params={**GOOD_PARAMS, "TurnPilotAlwaysDrive": False})
  off = line(lines, "TurnPilotAlwaysDrive")
  assert off.status == WARN and "TurnPilotAlwaysDrive=1" in off.fix, off
  assert cpf.exit_code(lines) == 0


def test_map_key_unset_live(tmp_path):
  params = {k: v for k, v in GOOD_PARAMS.items() if k != "TurnPilotMapKey"}
  lines = run_live(tmp_path, params=params)
  assert statuses(lines)["TurnPilotMapKey"] == NO_GO


# ------------------------------------------------------------------------------------------------- a qlog, a sunnypilot log

def test_qlog_without_can(tmp_path):
  """A qlog keeps no modelV2 or detections and about 3 can messages a segment (services.py): those items UNKNOWN, never GO."""
  for kw in ({"can": False, "sendcan": False}, {"can_keep": 100, "sendcan": False}):
    lines = run_route(tmp_path, name="qlog.zst", model=False, detections=False, **kw)
    s = statuses(lines)
    for name in CAN_LINES + ["big model", "detector"]:
      assert s[name] == UNKNOWN, (kw, name, line(lines, name))
    assert NO_GO not in s.values()
    assert cpf.exit_code(lines) == 2


def test_sunnypilot_log(tmp_path):
  """Sunnypilot's own messages sit on turnpilotPlan's slot (a fault there is garbage), events are numbered differently, the
  vehicle selector fixes the fingerprint and flags read 0: TurnPilot's items not applicable, the car items judged."""
  lines = run_route(tmp_path, git=SUNNYPILOT, params=SUNNYPILOT_PARAMS, source="fixed", flags=0, plan_fault="noLongitudinal",
                    events=("canError",))
  s = statuses(lines)
  for name in TURNPILOT_LINES + ["branch", "TurnPilot params", "fingerprint source"]:
    assert s[name] == INFO, (name, line(lines, name))
  assert "not applicable" in line(lines, "TurnPilot faults").detail
  assert "vehicle selector" in line(lines, "fingerprint source").detail
  assert "DisengageOnAccelerator 0" in line(lines, "TurnPilot params").detail
  assert not set(REQUIRED_KEYS + MAP_KEYS) & set(s)
  for name in ["car", "EPS", "3-bit firmware", "DAS_settings", "longitudinal", "panda safety", "panda honours long",
               "driver's blinker", "calibration", "road camera", "big model", "car health"]:
    assert s[name] == GO, (name, line(lines, name))
  assert "not judged" in line(lines, "car health").detail and "canError" in line(lines, "car health").detail
  assert "none of" not in line(lines, "car health").detail  # it read no events, so it claims none absent
  assert cpf.exit_code(lines) == 0
  quiet = cpf.check_route([str(write(tmp_path, [m for m in drive(git=SUNNYPILOT, params=SUNNYPILOT_PARAMS, source="fixed", flags=0)
                                                if m.which() != "onroadEvents"], "quiet_rlog.zst"))])
  assert statuses(quiet)["car health"] == GO  # its events aren't judged, so canValid alone decides
  down = run_route(tmp_path, git=SUNNYPILOT, params=SUNNYPILOT_PARAMS, source="fixed", flags=0, big="false")  # Chase's route
  assert statuses(down)["big model"] == NO_GO and cpf.exit_code(down) == 1


# ------------------------------------------------------------------------------------------------- the command, the rules

def test_live_reads_the_sockets(tmp_path):
  """--live's own path on real msgq sockets: this test publishes (as card and pandad would), the tool only subscribes."""
  sent = [m for m in drive() if m.which() in ("pandaStates", "carState") and m.logMonoTime < (OFFSET_S + 2.) * 1e9]
  values = {**GOOD_PARAMS, **git_params(TURNPILOT)}
  out = {}
  with OpenpilotPrefix():
    pubs = {s: messaging.pub_sock(s) for s in ("pandaStates", "carState")}
    reader = threading.Thread(target=lambda: out.update(f=cpf.collect_live(2., params=FakeParams(tmp_path, values))[0]))
    reader.start()
    time.sleep(0.5)  # the subscriber is up: msgq hands a new subscriber only what is published after it joins
    for m in sent:
      pubs[m.which()].send(m.as_builder().to_bytes())
      time.sleep(0.002)
    reader.join()
  f = out["f"]
  assert len(f.car_state) == sum(m.which() == "carState" for m in sent) and len(f.panda) == sum(m.which() == "pandaStates" for m in sent)
  assert f.live and f.init["branch"] == TURNPILOT["branch"]



def test_main_prints_and_exits(tmp_path, capsys):
  path = write(tmp_path, drive(refuse_after=1))
  assert cpf.main(["--route", str(path)]) == 1
  out = capsys.readouterr().out
  assert "NO-GO" in out and "fix:" in out and "panda tesla from 4.00 s" in out
  assert str(LAT)[:6] not in out and str(LON)[:7] not in out  # no GPS position, from a message or a param
  assert cpf.main(["--route", str(tmp_path)]) == 1  # a directory: its rlogs


def test_route_directory(tmp_path):
  """A device segment directory holds rlog.zst, qlog.zst and, while it records, rlog.lock: the rlog alone is read."""
  segment = tmp_path / "route--0"
  segment.mkdir()
  msgs = drive()
  write(segment, msgs, "rlog.zst")
  write(segment, [m for i, m in enumerate(msgs) if m.which() != "can" or i % 100 == 0], "qlog.zst")
  (segment / "rlog.lock").write_bytes(b"")
  assert cpf.route_files([str(tmp_path)]) == [segment / "rlog.zst"]
  f, _ = cpf.collect_route(cpf.route_files([str(tmp_path)]))
  assert len(f.can_t) == sum(m.which() == "can" for m in msgs) and not f.qlog
  qlogs = tmp_path / "route--1"
  qlogs.mkdir()
  write(qlogs, msgs, "qlog.zst")  # no rlog there: its qlog
  assert cpf.route_files([str(tmp_path)]) == [segment / "rlog.zst", qlogs / "qlog.zst"]
  assert cpf.route_files([str(segment / "qlog.zst")]) == [segment / "qlog.zst"]  # a file named is read as given


def test_never_prints_a_position(tmp_path, capsys):
  for name in ("rlog.zst", "qlog.zst"):
    cpf.main(["--route", str(write(tmp_path, drive(), name))])
    out = capsys.readouterr().out
    assert "39.76" not in out and "86.12" not in out and "LastGPSPosition" not in out


def test_read_only():
  """No param write, no publisher, no send, no panda: the tool reads."""
  tree = ast.parse(TOOL.read_text())
  writes = {"put", "put_bool", "put_nonblocking", "remove", "clear_all", "send", "pub_sock", "PubMaster", "Panda"}
  found = []
  for node in ast.walk(tree):
    if isinstance(node, ast.Call):
      fn = node.func
      called = fn.attr if isinstance(fn, ast.Attribute) else fn.id if isinstance(fn, ast.Name) else ""
      if called in writes:
        found.append(f"{called} at line {node.lineno}")
    if isinstance(node, (ast.Import, ast.ImportFrom)):
      mods = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module or ""]
      found += [f"import {m} at line {node.lineno}" for m in mods if m.split(".")[0] == "panda"]
  assert not found, found
  assert not {s for s in cpf.SERVICES if "gps" in s.lower() or "location" in s.lower() or "gnss" in s.lower()}


def test_runs_on_the_device_python():
  """The comma four runs this tree's python (pyproject: >= 3.12.3, < 3.13). At import the tool loads only the stdlib, this
  repo and opendbc; LogReader (openpilot.tools) only for --route, messaging only for --live."""
  source = TOOL.read_text()
  tree = ast.parse(source, feature_version=(3, 12))
  top = []
  for node in tree.body:
    if isinstance(node, ast.Import):
      top += [a.name for a in node.names]
    elif isinstance(node, ast.ImportFrom):
      top.append(node.module)
  for module in top:
    root = module.split(".")[0]
    assert root in sys.stdlib_module_names or root in ("openpilot", "opendbc"), module
    assert not module.startswith(("openpilot.tools", "openpilot.cereal.messaging")), module


def test_constants_follow_their_sources():
  from openpilot.selfdrive.turnpilot import car_signals, turnpilotd
  assert cpf.DETECTOR_DEADLINE_S == turnpilotd.DETECTOR_DEADLINE_S
  assert cpf.VINLESS == car_signals.VINLESS_KEY
  # the detector's bands come from its real rate: the model's 20 Hz over RUN_EVERY (10 Hz), never the 20 Hz of a stop sweep
  assert cpf.MODEL_HZ == ModelConstants.MODEL_RUN_FREQ and cpf.DETECTOR_RUN_EVERY == RUN_EVERY
  nominal = ModelConstants.MODEL_RUN_FREQ / RUN_EVERY
  assert cpf.DETECTOR_HZ == pytest.approx(nominal)
  assert cpf.DETECTOR_GO_HZ == pytest.approx(0.9 * nominal) and cpf.DETECTOR_MIN_HZ == pytest.approx(0.5 * nominal)
  assert cpf.DETECTOR_MIN_HZ > ModelConstants.MODEL_RUN_FREQ / FORCE_RUN_FRAMES  # room()'s forced-run floor is NO-GO
  keys = (REPO / "openpilot/common/params_keys.h").read_text()
  dont_log = set()
  for key in cpf.PARAM_KEYS:
    m = next((ln for ln in keys.splitlines() if f'{{"{key}",' in ln), None)
    if m is not None and "DONT_LOG" in m:
      dont_log.add(key)
  assert dont_log == set(cpf.DONT_LOG)
  assert CHASE_EPS in FW_VERSIONS[CAR.TESLA_MODEL_3][(car.CarParams.Ecu.eps, 0x730, None)]
  assert UNLISTED_EPS not in sum((v for ecus in FW_VERSIONS.values() for v in ecus.values()), [])


def test_every_key_is_this_builds(tmp_path):
  lib = Params(str(tmp_path / "params"))
  for key in cpf.PARAM_KEYS:
    lib.get_type(key)  # raises UnknownKeyName for a key this build doesn't have


def test_suite_catches_a_broken_check(tmp_path, monkeypatch):
  """Break the EPS check on purpose (every string GO): the unknown-EPS case must fail."""
  monkeypatch.setattr(cpf, "check_eps", lambda f: [cpf.Line("EPS", GO, "listed")])
  with pytest.raises(AssertionError):
    test_unknown_eps(tmp_path)
  monkeypatch.undo()
  test_unknown_eps(tmp_path)


def test_suite_catches_a_broken_detector(tmp_path, monkeypatch):
  """Turn the detector's NO-GO off (no rate floor, no deadline): the slow and gap cases must fail."""
  monkeypatch.setattr(cpf, "DETECTOR_MIN_HZ", 0.)
  monkeypatch.setattr(cpf, "DETECTOR_DEADLINE_S", float("inf"))
  for case in (test_detector_slow, test_detector_gap):
    with pytest.raises(AssertionError):
      case(tmp_path)
  monkeypatch.undo()
  test_detector_slow(tmp_path)
  test_detector_gap(tmp_path)
