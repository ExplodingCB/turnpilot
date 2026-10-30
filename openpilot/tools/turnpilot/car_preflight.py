#!/usr/bin/env python3
"""car_preflight: GO, NO-GO, WARN, UNKNOWN or INFO for every car-dependent item on Chase's Model 3, read from a running
comma four or from a route pulled off one (turnpilot-artifacts phase4/chase_car/CHASE_CAR.md, "Preflight tool spec").

  on the device, parked with the car on:
    cd /data/openpilot && python3 -m openpilot.tools.turnpilot.car_preflight --live --seconds 30
  on rlogs pulled off the device (files, or directories holding them):
    python3 -m openpilot.tools.turnpilot.car_preflight --route <rlog.zst> [<rlog.zst> ...]

selfdrive/turnpilot/preflight.py checks the install; this checks the car. It reads only: it subscribes for the window, reads
params and log files, never writes a param, sends on no socket and never opens the panda (pandad owns it). It reads no
location service and none of initData's params but the ones it checks, so it never prints a GPS position.

A missing fact (no message in the window, no such event in the log) prints UNKNOWN, never GO. A param that is unset, or
absent from initData.params, takes this build's default, said. A qlog keeps no modelV2 or detections and about 3 can
messages a segment (services.py), so the items that need them print UNKNOWN. The panda's tesla time is the first
pandaStates in tesla safety: before it the panda is in elm327, the harness relay puts the Autopilot computer straight on
party CAN and openpilot's first sends come back refused, so every CAN item counts only frames after it.

On a sunnypilot log (initData.gitRemote names sunnypilot) TurnPilot's own items print INFO "not applicable": sunnypilot's own
messages sit on turnpilotPlan's, turnpilotDetections' and turnpilotFarDetections' union slots, so those are never read, nor
carState.carSignal; its events are numbered differently, so their names are listed, not judged; fingerprintSource fixed is
its vehicle selector, and the EPS line decides whether this fork would match the car.

Exit 0 when every line is GO, WARN or INFO; 1 on any NO-GO; 2 on any UNKNOWN without a NO-GO. A route can't show the map key
and certificate (DONT_LOG), so a route never exits 0.

It lives in tools/turnpilot, not beside preflight.py: --route reads with openpilot.tools' LogReader, and test_preflight's
import scan keeps openpilot.tools off selfdrive/turnpilot (the car path). --live imports none of it.
"""
import argparse
import math
import re
import statistics
import sys
import tempfile
import time
from collections import Counter, defaultdict, namedtuple
from dataclasses import dataclass, field
from pathlib import Path

from opendbc.car.structs import CarParams
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.values import CAR, TeslaFlags
from openpilot.common.version import RELEASE_BRANCHES

GO, NO_GO, WARN, UNKNOWN, INFO = "GO", "NO-GO", "WARN", "UNKNOWN", "INFO"

CHASE_EPS = b'TeM3_E014p10_0.0.0 (35),E014.21.2'  # the EPS answer on Chase's sunnypilot route of 2026-10-30 (ROUTE_FACTS.md)
EPS_ADDR = 0x730
M3_EPS = (CarParams.Ecu.eps, EPS_ADDR, None)

# can src: 0 party CAN on the car's side, 2 party CAN on the Autopilot computer's side, 1 vehicle CAN (an X179 add-on only),
# +128 a frame the panda put on that bus and got back (its own send, or a frame it forwarded), +192 a send it refused
PARTY, VEHICLE, AP = 0, 1, 2
RETURNED, REFUSED = 128, 192
DI_SPEED = 0x257
DAS_REDUNDANT_BRAKING = 0x489  # only the 3-bit firmware sends these two (opendbc tesla/interface.py's gate)
DI_AUTONOMY_HEALTH = 0x054
DAS_SETTINGS = 0x293
DAS_CONTROL = 0x2B9
DAS_BODY_CONTROLS = 0x3E9
UI_WARNING = 0x311
X179 = {0x249: "SCCM_leftStalk", 0x3F5: "VCFRONT_lighting", 0x3DF: "UI_status2"}
WATCH = {DI_SPEED, DAS_REDUNDANT_BRAKING, DI_AUTONOMY_HEALTH, DAS_SETTINGS, DAS_CONTROL, DAS_BODY_CONTROLS, UI_WARNING, *X179}
SENT = {DAS_CONTROL, DAS_BODY_CONTROLS}

MIN_CAN_HZ = 10.          # can slower than this is decimated (a qlog), not a full log (100 Hz)
MIN_ABSENT_S = 5.         # s of can carrying DI_speed after the tesla time before a missing message counts as absent
LONG_BACK_FRAC = 0.9      # openpilot's 0x2B9 that must come back on src 128 for the panda to be honouring them
MODEL_HZ = 20.            # ModelConstants.MODEL_RUN_FREQ, kept equal by a test
DETECTOR_RUN_EVERY = 2    # modeld_detector.RUN_EVERY, kept equal by a test: the full pass on every 2nd model frame
DETECTOR_HZ = MODEL_HZ / DETECTOR_RUN_EVERY  # 10 Hz parked or driving; only a stop sweep's side views add messages between
DETECTOR_GO_HZ = 0.9 * DETECTOR_HZ   # a run room() puts off waits one frame, so a few in the window stay GO
DETECTOR_MIN_HZ = 0.5 * DETECTOR_HZ  # under it, room() refuses most runs (its forced-run floor is 1.25 Hz): NO-GO
DETECTOR_DEADLINE_S = 1.5  # turnpilotd.DETECTOR_DEADLINE_S, kept equal by a test
LOADING_TAIL_S = 2.       # a bigModelLoading this close to the window's end: still loading
CAMERA_SENSOR = "os04c10"  # preflight.py's CAR_CAMERA, the 1344x760 sensor the detector pkl is built for

SERVICES = ("can", "sendcan", "pandaStates", "carState", "carParams", "modelV2", "deviceState", "chestnutState", "onroadEvents",
            "extrinsicsCalibration", "narrowRoadCameraState", "turnpilotDetections", "turnpilotFarDetections", "turnpilotPlan")
TURNPILOT_SERVICES = ("turnpilotDetections", "turnpilotFarDetections", "turnpilotPlan")
HEALTH_EVENTS = ("canError", "carUnrecognized", "invalidLkasSetting", "calibrationIncomplete")
MODEL_EVENTS = ("bigModelLoading", "bigModelFailed", "turnpilotModelFellBack")
DETECTOR_EVENTS = ("turnpilotNoDetector", "turnpilotDetectorStopped")
NOT_APPLICABLE = "not applicable: a sunnypilot log, whose own messages share TurnPilot's slots"

PREFLIGHT_SET = "python3 -m openpilot.selfdrive.turnpilot.preflight --set"
REQUIRED = (  # (key, the value Saturday needs, None for unset, how to set it)
  ("TurnPilotEnabled", True, f"{PREFLIGHT_SET} TurnPilotEnabled=1, then turn the car off and on"),
  ("NavigationEnabled", True, "Settings > Developer > navigation (alpha)"),
  ("AlphaLongitudinalEnabled", True, "Settings > Developer > openpilot longitudinal control (alpha), then turn the car off and on"),
  ("DisengageOnAccelerator", True, f"Settings > Toggles > Disengage on accelerator pedal, or {PREFLIGHT_SET} DisengageOnAccelerator=1"),
  ("TurnPilotCarSignals", "off", f"{PREFLIGHT_SET} TurnPilotCarSignals=off"),
  ("TurnPilotCameraLights", "shadow", f"{PREFLIGHT_SET} TurnPilotCameraLights=shadow"),
  ("TurnPilotCameraStopSigns", "shadow", f"{PREFLIGHT_SET} TurnPilotCameraStopSigns=shadow"),
  ("TurnPilotStopSignGo", "off", f"{PREFLIGHT_SET} TurnPilotStopSignGo=off"),
  ("TurnPilotAvoidStops", True, f"{PREFLIGHT_SET} TurnPilotAvoidStops=1"),
  ("TurnPilotCurbPedCap", True, f"{PREFLIGHT_SET} TurnPilotCurbPedCap=1"),
  ("TurnPilotLeftRangeM", 0., f"{PREFLIGHT_SET} TurnPilotLeftRangeM=0"),
  ("TurnPilotValhallaTiles", None, f"{PREFLIGHT_SET} TurnPilotValhallaTiles= (empty clears it)"),
)
NOTED = ("TurnPilotShadowMode", "DisableUpdates", "TurnPilotLeftFarRangeM", "TurnPilotAutoLeft", "TurnPilotGoOverride", "TurnPilotNoMoveS")
MAP_PARAMS = ("TurnPilotMapHost", "TurnPilotMapKey", "TurnPilotMapCert")
DONT_LOG = ("TurnPilotMapKey", "TurnPilotMapCert")  # params_keys.h: loggerd logs them with no value; kept equal by a test
OTHER_PARAMS = ("IsReleaseBranch", "TurnPilotCarSignalsVerified", "ChestnutLoading")
PARAM_KEYS = tuple(k for k, _, _ in REQUIRED) + NOTED + MAP_PARAMS + OTHER_PARAMS
SUNNYPILOT_CONTEXT = ("AlphaLongitudinalEnabled", "DisengageOnAccelerator", "Mads")  # how sunnypilot was set, said on its log

BY_HAND = (  # in no message or param, so never GO: where to read each
  ("panda build", "in no message: on the device run cat /data/openpilot/panda/board/obj/version, expect DEV-<commit>-DEBUG; " +
                  "the panda honours long line is the log's stand-in"),
  ("Tesla software", "in no message: the car's Controls > Software screen; an update can move the EPS string off the list"),
  ("outside lamps", "whether a lamp check's outside lamps flashed: only a person watching can say"),
  ("mount", "whether the comma moved between mounts or cars since it calibrated: ask"),
)

PandaRow = namedtuple("PandaRow", "t model param rx_checks_invalid rx_invalid tx_blocked harness ignition controls_allowed")
ModelRow = namedtuple("ModelRow", "t big execution_time frame_drop")
ChestnutRow = namedtuple("ChestnutRow", "t supply_mv fault pcie temp gpu")
CalibRow = namedtuple("CalibRow", "t status perc blocks rpy height")
DetectionRow = namedtuple("DetectionRow", "t execution_time camera lane_paint light")
PlanRow = namedtuple("PlanRow", "t state fault shadow handback signal signal_why")


@dataclass(frozen=True)
class Line:
  name: str
  status: str
  detail: str = ""
  fix: str = ""


@dataclass(frozen=True)
class Param:
  key: str
  state: str          # set, default (this build's), unset, unknown (this build's library lacks the key), unlogged (DONT_LOG, a route)
  value: object = None


class ParamView:
  """Params as the checks see them: the device's, live, or a route's initData.params decoded with this build's library.
  Types and defaults always come from the library."""
  def __init__(self, lib, raw: dict | None = None, keep=None):
    self.lib, self.raw, self._keep = lib, raw, keep  # keep: the temporary directory a route's library lives in

  def get(self, key: str) -> Param:
    try:
      kind = self.lib.get_type(key)
    except Exception:  # UnknownKeyName
      return Param(key, "unknown")
    if self.raw is not None and key in DONT_LOG:
      return Param(key, "unlogged", key in self.raw)
    try:
      if self.raw is None:
        value = self.lib.get(key)
      else:
        dat = self.raw.get(key)
        value = self.lib.cpp2python(key, dat) if dat else None
    except Exception:
      value = None
    if value is not None and value != "":
      return Param(key, "set", value)
    default = self.lib.get_default_value(key)
    if default is not None:
      return Param(key, "default", default)
    return Param(key, "unset", False if getattr(kind, "name", "") == "BOOL" else None)


@dataclass
class Facts:
  """What the window or the log held, as plain values; times are logMonoTime, ns."""
  where: str = ""
  live: bool = False
  qlog: bool = False
  sunnypilot: bool = False
  init: dict | None = None
  params_raw: dict | None = None
  t0: int | None = None
  t1: int | None = None
  cp: object = None
  cp_from: str = ""
  extra: dict = field(default_factory=dict)
  panda: list = field(default_factory=list)
  can_t: list = field(default_factory=list)
  frames: dict = field(default_factory=lambda: defaultdict(list))   # address -> [(t, src, dat)]
  sent: dict = field(default_factory=lambda: defaultdict(list))     # address -> [(t, dat)], openpilot's sendcan
  car_state: list = field(default_factory=list)                     # (t, canValid, canErrorCounter)
  signal_status: Counter = field(default_factory=Counter)
  signal_fault: Counter = field(default_factory=Counter)
  signal_rejected: int = 0
  signal_why: str = ""
  model: list = field(default_factory=list)
  chestnut_present: list = field(default_factory=list)
  chestnut: list = field(default_factory=list)
  events: list = field(default_factory=list)                        # (t, name)
  events_t: list = field(default_factory=list)                      # every onroadEvents message's t, an empty one too
  calib: list = field(default_factory=list)
  sensors: Counter = field(default_factory=Counter)
  detections: list = field(default_factory=list)
  far_t: list = field(default_factory=list)
  plans: list = field(default_factory=list)

  def feed(self, msg) -> None:
    which, t = msg.which(), msg.logMonoTime
    self._time(t)  # every message: times are said from the first message of the log (as tesla_rlog_hw.py does) or the window
    if which == "initData":
      if not self.live:
        self._init_data(msg.initData)
      return
    if which not in SERVICES or (self.sunnypilot and which in TURNPILOT_SERVICES):
      return
    getattr(self, "_" + which)(t, getattr(msg, which))

  def _time(self, t: int) -> None:
    if t > 0:
      self.t0 = t if self.t0 is None else self.t0
      self.t1 = t if self.t1 is None else max(self.t1, t)

  def _init_data(self, d) -> None:
    if self.init is not None:
      return
    self.init = {"branch": d.gitBranch, "remote": d.gitRemote, "commit": d.gitCommit}
    keys = set(PARAM_KEYS) | set(SUNNYPILOT_CONTEXT)  # only these: initData.params also holds positions
    self.params_raw = {e.key: bytes(e.value) for e in d.params.entries if e.key in keys}
    self.sunnypilot = "sunnypilot" in d.gitRemote.lower()

  def _can(self, t, can) -> None:
    self.can_t.append(t)
    for c in can:
      if c.address in WATCH:
        self.frames[c.address].append((t, c.src, bytes(c.dat)))

  def _sendcan(self, t, can) -> None:
    for c in can:
      if c.address in SENT:
        self.sent[c.address].append((t, bytes(c.dat)))

  def _pandaStates(self, t, states) -> None:
    if len(states):
      p = states[0]
      self.panda.append(PandaRow(t, str(p.safetyModel), p.safetyParam, p.safetyRxChecksInvalid, p.safetyRxInvalid,
                                 p.safetyTxBlocked, str(p.harnessStatus), p.ignitionLine or p.ignitionCan, p.controlsAllowed))

  def _carState(self, t, cs) -> None:
    self.car_state.append((t, cs.canValid, cs.canErrorCounter))
    if not self.sunnypilot:
      s = cs.carSignal
      self.signal_status[str(s.status)] += 1
      self.signal_fault[str(s.fault)] += 1
      self.signal_rejected = max(self.signal_rejected, s.rejected)
      self.signal_why = s.checkWhy or self.signal_why

  def _carParams(self, t, cp) -> None:
    if self.cp is None:
      self.cp, self.cp_from = cp.as_builder(), "carParams in the log" if not self.live else "this drive's carParams"

  def _modelV2(self, t, m) -> None:
    self.model.append(ModelRow(t, m.big, m.modelExecutionTime, m.frameDropPerc))

  def _deviceState(self, t, d) -> None:
    self.chestnut_present.append(d.chestnutPresent)

  def _chestnutState(self, t, c) -> None:
    self.chestnut.append(ChestnutRow(t, c.supplyVoltage, c.supplyFault, c.pcieLtssm, c.tempC, c.gpuUsagePercent))

  def _onroadEvents(self, t, events) -> None:
    self.events_t.append(t)  # selfdrived sends the whole list every second while onroad: empty is a healthy car's
    for e in events:
      try:
        self.events.append((t, str(e.name)))
      except Exception:  # an event this schema doesn't number
        self.events.append((t, "unknown"))

  def _extrinsicsCalibration(self, t, c) -> None:
    self.calib.append(CalibRow(t, str(c.calStatus), c.calPerc, c.validBlocks, list(c.rpyCalib), list(c.height)))

  def _narrowRoadCameraState(self, t, c) -> None:
    self.sensors[str(c.sensor)] += 1

  def _turnpilotDetections(self, t, d) -> None:
    self.detections.append(DetectionRow(t, d.executionTime, str(d.camera), str(d.lanePaintStatus), str(d.lightStatus)))

  def _turnpilotFarDetections(self, t, d) -> None:
    self.far_t.append(t)

  def _turnpilotPlan(self, t, p) -> None:
    self.plans.append(PlanRow(t, str(p.state), str(p.fault), p.shadow, str(p.handback), str(p.turnSignal.status), p.turnSignal.why))


# ---------------------------------------------------------------------------------------------------------------- helpers

def secs(f: Facts, t: int) -> float:
  return (t - f.t0) / 1e9


def span_s(ts: list) -> float:
  return (max(ts) - min(ts)) / 1e9 if len(ts) > 1 else 0.


def tesla_time(f: Facts) -> int | None:
  return next((r.t for r in sorted(f.panda) if r.model == "tesla"), None)


def mode_timeline(f: Facts) -> str:
  out, last = [], None
  for r in sorted(f.panda):
    if r.model != last:
      out.append((r.model, r.t))
      last = r.model
  if len(out) == 1:
    return f"{out[0][0]} throughout"
  return ", ".join(f"{m} from {max(0., secs(f, t)):.2f} s" for m, t in out)  # a message logged a hair before the first reads 0


def can_window(f: Facts) -> tuple[int | None, str]:
  """(the tesla time, '') when the CAN items can judge the can, else (None, why not)."""
  if not f.can_t:
    return None, "no can" + (": a qlog keeps about 3 messages a segment" if f.qlog else " in the window")
  hz = (len(f.can_t) - 1) / span_s(f.can_t) if span_s(f.can_t) > 0 else 0.
  if hz < MIN_CAN_HZ:
    return None, f"can at {hz:.1f} Hz: decimated, as in a qlog (about 3 messages a segment), not a full log"
  tt = tesla_time(f)
  if tt is None:
    return None, f"the panda never entered tesla safety ({mode_timeline(f) if f.panda else 'no pandaStates'})"
  if max(f.can_t) < tt:
    return None, "no can after the panda entered tesla safety"
  return tt, ""


def after(f: Facts, address: int, tt: int, src: int | None = None) -> list:
  return [x for x in f.frames[address] if x[0] >= tt and (src is None or x[1] == src)]


def can_span_after(f: Facts, tt: int) -> float:
  return span_s([t for t in f.can_t if t >= tt])


def di_span(f: Facts, tt: int) -> float:
  return span_s([t for t, _, _ in after(f, DI_SPEED, tt, PARTY)])


def vin_valid(vin: str) -> bool:
  from openpilot.selfdrive.turnpilot.car_signals import vin_valid as valid
  return valid(vin)


def fw_text(cp) -> str:
  fw = [f"{x.ecu} 0x{x.address:x} {bytes(x.fwVersion)!r}" for x in cp.carFw if not x.logging]
  return ", ".join(fw) or "none: no ECU answered the firmware query"


def flag_names(flags: int) -> str:
  names = [f.name for f in TeslaFlags if f.value and flags & f.value and f.value & (f.value - 1) == 0]
  return f"{flags} ({', '.join(names) or 'none'})"


def shown(p: Param) -> str:
  if p.state == "unknown":
    return "unknown key"
  if p.state == "unlogged":
    return "not logged"
  v = p.value
  if v is None:
    text = "unset"
  elif isinstance(v, bool):
    text = "1" if v else "0"
  elif isinstance(v, float):
    text = f"{v:g}"
  elif isinstance(v, (dict, list, bytes)):
    text = "set"
  else:
    text = str(v)
  if p.state == "default":
    text += " (default)"
  elif p.state == "unset" and v is not None:
    text += " (unset)"
  return text


def unknown_param(key: str, live: bool) -> Line:
  where = "the device's params library" if live else "the params library this tool runs with"
  return Line(key, NO_GO, f"unknown to {where}: libparams was built before the key (FK-16)",
              "rebuild (scons) on the device, then restart" if live else "update this checkout and rebuild (scons), then read the route again")


# ------------------------------------------------------------------------------------------------------------ the items

def check_branch(f: Facts, pv: ParamView) -> Line:
  name = "branch"
  if f.init is None or not f.init.get("branch"):
    return Line(name, UNKNOWN, "GitBranch unset" if f.live else "no initData")
  branch, commit, remote = f.init["branch"], f.init.get("commit") or "", f.init.get("remote") or ""
  if f.sunnypilot:
    return Line(name, INFO, f"sunnypilot {branch} at {commit[:10]} ({remote}): not this fork, so TurnPilot's items are not applicable")
  release = pv.get("IsReleaseBranch")
  if branch in RELEASE_BRANCHES or (release.state == "set" and release.value):
    why = f"{branch} is a release branch" if branch in RELEASE_BRANCHES else "the manager reports a release branch"
    return Line(name, NO_GO, why + ": no alpha longitudinal toggle, and the panda gets the release build",
                "install the frozen TurnPilot build from its non-release branch")
  return Line(name, GO, f"{branch} at {commit[:10] or 'an unknown commit'}: compare with the frozen commit")


def check_car(f: Facts) -> Line:
  name = "car"
  cp = f.cp
  if cp is None:
    return Line(name, UNKNOWN, "no carParams" + (", and no CarParams or CarParamsPersistent param" if f.live else " in the log"))
  where = f" ({f.cp_from})"
  if cp.brand == "mock":
    return Line(name, NO_GO, f"not recognised, so MOCK in dashcam mode{where}: firmware {fw_text(cp)}",
                "add the EPS string exactly as printed to opendbc_repo/opendbc/car/tesla/fingerprints.py; a Tesla update won't fix this")
  car = f"{cp.brand} {cp.carFingerprint}"
  if cp.brand != "tesla" or cp.carFingerprint != str(CAR.TESLA_MODEL_3):
    return Line(name, NO_GO, f"{car}{where}: these checks are for Chase's Model 3 (tesla TESLA_MODEL_3)",
                "run it on the Model 3, or check which car this comma was in")
  if cp.dashcamOnly:
    return Line(name, NO_GO, f"{car} in dashcam mode{where}: neither 0x489 nor 0x054 while fingerprinting, the 2-bit " +
                             "DAS_steeringControlType of software older than the 3-bit builds (opendbc tesla/interface.py)",
                "update the Tesla's software (Controls > Software), then turn the car off and on")
  if cp.fuzzyFingerprint:
    return Line(name, NO_GO, f"{car} by a fuzzy match{where}: a Tesla is matched only by its exact EPS string", "see the EPS line")
  flags = flag_names(cp.flags)
  if f.sunnypilot:
    flags += ", as sunnypilot's port leaves them" if cp.flags == 0 else ", unexpected from sunnypilot's port"
  elif not cp.flags & TeslaFlags.HAS_BODY_CONTROLS:
    flags += ": HAS_BODY_CONTROLS clear, so 0x3E9 missed the fingerprint window or is absent"
  return Line(name, GO, f"{car}, fingerprint source {cp.fingerprintSource}, flags {flags}{where}")


def check_eps(f: Facts) -> list[Line]:
  name = "EPS"
  cp = f.cp
  if cp is None:
    return [Line(name, UNKNOWN, "no carParams")]
  eps = [bytes(x.fwVersion) for x in cp.carFw if str(x.ecu) == "eps" and x.address == EPS_ADDR and not x.logging]
  if not eps or not any(eps):
    return [Line(name, NO_GO, f"no EPS answer at 0x730 in carFw ({fw_text(cp)})",
                 "turn the car off and on; if the EPS still doesn't answer, check the harness")]
  ours = FW_VERSIONS[CAR.TESLA_MODEL_3][M3_EPS]
  for s in eps:
    if s in ours:
      continue
    others = [str(platform) for platform, ecus in FW_VERSIONS.items() if any(s in versions for versions in ecus.values())]
    if others:
      return [Line(name, NO_GO, f"{s!r} is listed only for {', '.join(others)}: not this fork's TESLA_MODEL_3",
                   "check which car this comma was in")]
    return [Line(name, NO_GO, f"{s!r} is in no fingerprint list: the car comes up MOCK and drives as a stock Tesla",
                 "add it exactly as printed to opendbc_repo/opendbc/car/tesla/fingerprints.py under TESLA_MODEL_3 " +
                 "(patch 0001 carries (35),EL014.21.2)")]
  lines = [Line(name, GO, f"{eps[0]!r} is listed for TESLA_MODEL_3 (opendbc tesla/fingerprints.py)")]
  if eps[0] != CHASE_EPS:
    lines.append(Line("EPS since route", INFO, f"differs from Chase's route of 2026-10-30, {CHASE_EPS!r}: has the Tesla's software changed?"))
  return lines


def check_source(f: Facts) -> Line:
  name = "fingerprint source"
  if f.cp is None:
    return Line(name, UNKNOWN, "no carParams")
  source = str(f.cp.fingerprintSource)
  if source == "fixed":
    if f.sunnypilot:
      return Line(name, INFO, "fixed: sunnypilot's vehicle selector picked the platform; the EPS line says whether this fork would")
    return Line(name, WARN, "fixed: FINGERPRINT was set, so an EPS string in no list would be hidden",
                "unset FINGERPRINT so the car is matched by its EPS string")
  if source == "fw":
    return Line(name, INFO, "fw: matched by the exact EPS string")
  return Line(name, WARN, f"{source}: a Tesla has no CAN fingerprint, only the EPS string")


def check_three_bit(f: Facts) -> Line:
  name = "3-bit firmware"
  tt, why = can_window(f)
  if tt is None:
    return Line(name, UNKNOWN, why)
  n489, n054 = len(after(f, DAS_REDUNDANT_BRAKING, tt, AP)), len(after(f, DI_AUTONOMY_HEALTH, tt, PARTY))
  if n489 or n054:
    return Line(name, GO, f"0x489 on src 2: {n489}, 0x054 on src 0: {n054}, after the tesla time")
  span = di_span(f, tt)
  if span < MIN_ABSENT_S:
    return Line(name, UNKNOWN, f"no 0x489 or 0x054, but only {span:.1f} s of can with DI_speed 0x257 on src 0 ({MIN_ABSENT_S:.0f} s needed)")
  return Line(name, NO_GO, f"neither 0x489 on src 2 nor 0x054 on src 0 in {span:.1f} s of can with DI_speed: the 2-bit " +
                           "DAS_steeringControlType, which this fork doesn't steer", "update the Tesla's software (Controls > Software)")


def check_das_settings(f: Facts) -> Line:
  name = "DAS_settings"
  if f.cp is None:
    return Line(name, UNKNOWN, "no carParams")
  tt, _ = can_window(f)
  seen = f", 0x293 on src 2: {len(after(f, DAS_SETTINGS, tt, AP))}" if tt is not None else ""
  if f.cp.flags & TeslaFlags.MISSING_DAS_SETTINGS:
    return Line(name, WARN, f"MISSING_DAS_SETTINGS set{seen}: the Autosteer-on check is off", "confirm by hand that Autosteer is off (TACC only)")
  return Line(name, GO, f"MISSING_DAS_SETTINGS clear{seen}: openpilot refuses to engage while Autosteer is on")


def check_longitudinal(f: Facts, pv: ParamView) -> Line:
  name = "longitudinal"
  cp = f.cp
  if cp is None:
    return Line(name, UNKNOWN, "no carParams")
  p = pv.get("AlphaLongitudinalEnabled")
  if p.state == "unknown":
    return unknown_param(name, f.live)
  bit = bool(len(cp.safetyConfigs) and cp.safetyConfigs[0].safetyParam & 1)
  facts = {"alphaLongitudinalAvailable": cp.alphaLongitudinalAvailable, "openpilotLongitudinalControl": cp.openpilotLongitudinalControl,
           "safetyParam's LONG_CONTROL bit": bit, "AlphaLongitudinalEnabled": bool(p.value)}
  off = [k for k, v in facts.items() if not v]
  if not off:
    return Line(name, GO, "on: " + ", ".join(facts))
  if not cp.alphaLongitudinalAvailable:
    fix = "this car's port has no alpha longitudinal"
  elif not p.value:
    fix = REQUIRED[2][2]
  else:
    fix = "turn the car off and on: the drive started without it"
  return Line(name, NO_GO, f"off: {', '.join(off)} false (AlphaLongitudinalEnabled {shown(p)})", fix)


def check_panda(f: Facts) -> Line:
  name = "panda safety"
  if not f.panda:
    return Line(name, UNKNOWN, "no pandaStates")
  rows, tt = sorted(f.panda), tesla_time(f)
  if tt is None:
    if not any(r.ignition for r in rows):
      return Line(name, UNKNOWN, f"offroad, no ignition: {mode_timeline(f)}")
    return Line(name, UNKNOWN, f"never entered tesla safety: {mode_timeline(f)}")
  on = [r for r in rows if r.t >= tt and r.ignition]
  if not on:
    return Line(name, UNKNOWN, f"no ignition after the tesla time: {mode_timeline(f)}")
  if f.cp is None or not len(f.cp.safetyConfigs):
    return Line(name, UNKNOWN, f"no carParams to compare safetyParam with: {mode_timeline(f)}")
  want = f.cp.safetyConfigs[0].safetyParam
  wrong = [r for r in on if r.model != "tesla" or r.param != want]
  rx = [r for r in on if r.rx_checks_invalid]
  unplugged = [r for r in on if r.harness == "notConnected"]
  problems, fixes = [], []
  if wrong:
    problems.append(f"{len(wrong)} pandaStates read {wrong[0].model} safetyParam {wrong[0].param}, carParams wants tesla {want}")
    fixes.append("turn the car off and on; if it stays, the panda's build doesn't match this one")
  if rx:
    problems.append(f"safetyRxChecksInvalid on {len(rx)} of {len(on)} after the tesla time, first at {secs(f, rx[0].t):.2f} s")
    fixes.append("the panda sees the car's messages out of spec: check the harness, then turn the car off and on")
  if unplugged:
    problems.append("harness notConnected")
    fixes.append("reseat the harness at the comma")
  if problems:
    return Line(name, NO_GO, "; ".join(problems) + f" ({mode_timeline(f)})", "; ".join(fixes))
  rx_invalid = max(r.rx_invalid for r in on) - min(r.rx_invalid for r in on)
  tx_blocked = max(r.tx_blocked for r in on) - min(r.tx_blocked for r in on)
  allowed = "seen" if any(r.controls_allowed for r in on) else "never"
  over = span_s([r.t for r in on])
  return Line(name, GO, f"tesla, safetyParam {want} as carParams, RX checks valid on all {len(on)} pandaStates over {over:.1f} s, " +
                        f"harness {on[-1].harness}; safetyRxInvalid +{rx_invalid}, safetyTxBlocked +{tx_blocked}, " +
                        f"controlsAllowed {allowed}; {mode_timeline(f)}")


def check_panda_long(f: Facts) -> list[Line]:
  name = "panda honours long"
  tt, why = can_window(f)
  if tt is None:
    return [Line(name, UNKNOWN, why)]
  frames = f.frames[DAS_CONTROL]
  sent = [d for t, d in f.sent[DAS_CONTROL] if t >= tt]
  back = [d for t, s, d in frames if t >= tt and s == RETURNED]
  refused_after = [t for t, s, _ in frames if t >= tt and s == REFUSED]
  refused_before = [t for t, s, _ in frames if t < tt and s == REFUSED]
  stock = {d for t, s, d in frames if t >= tt and s == AP}
  lines = []
  if refused_after:
    lines.append(Line(name, NO_GO, f"the panda refused {len(refused_after)} 0x2B9 after entering tesla safety at {secs(f, tt):.2f} s " +
                                   f"(first at {secs(f, refused_after[0]):.2f} s): a release panda build, or a safetyParam it doesn't honour",
                      "install the non-release TurnPilot build (its panda gets the debug build), turn alpha longitudinal on, restart the car"))
  elif f.cp is not None and not f.cp.openpilotLongitudinalControl:
    lines.append(Line(name, UNKNOWN, "openpilot longitudinal is off, so there is nothing to honour: see the longitudinal line"))
  elif not sent:
    lines.append(Line(name, UNKNOWN, "openpilot sent no 0x2B9 after the panda entered tesla safety"))
  else:
    sent_set = set(sent)
    equal = sum(d in sent_set for d in back)
    forwarded = sum(d in stock and d not in sent_set for d in back)
    frac = equal / len(sent)
    detail = (f"{len(sent)} sent after the tesla time, {equal} back on src 128 byte-equal to a sent frame ({frac:.1%}), " +
              f"{forwarded} equal to the stock DAS_control, none refused")
    if forwarded:
      lines.append(Line(name, WARN, detail + ": the panda forwarded the stock DAS_control", "the panda's build doesn't block it: check the panda build"))
    elif frac >= LONG_BACK_FRAC:
      lines.append(Line(name, GO, detail))
    else:
      lines.append(Line(name, UNKNOWN, detail + f": under {LONG_BACK_FRAC:.0%} came back, so this can't tell"))
  if refused_before:
    when = f"{secs(f, refused_before[0]):.2f}-{secs(f, refused_before[-1]):.2f} s"
    lines.append(Line("0x2B9 before tesla", INFO, f"{len(refused_before)} refused on src 192 at {when}, before the panda entered tesla at " +
                                                  f"{secs(f, tt):.2f} s: normal startup, card sends in the step it sets ControlsReady"))
  return lines


def check_turn_signal_frame(f: Facts) -> Line:
  name = "turn-signal frame"
  tt, why = can_window(f)
  if tt is None:
    return Line(name, UNKNOWN, why)
  span = can_span_after(f, tt)
  src2, src0 = after(f, DAS_BODY_CONTROLS, tt, AP), after(f, DAS_BODY_CONTROLS, tt, PARTY)
  src128 = after(f, DAS_BODY_CONTROLS, tt, RETURNED)
  stock = {d for _, _, d in src2}
  sent = {d for t, d in f.sent[DAS_BODY_CONTROLS] if t >= tt}
  odd = [d for _, _, d in src128 if d not in stock and d not in sent]
  warns = []
  if src0:
    warns.append(f"{len(src0)} on src 0 after the tesla time: the Autopilot computer's frame reaches party CAN past the panda")
  if odd:
    warns.append(f"{len(odd)} on src 128 equal to neither a stock nor a sent frame")
  if src2:
    body = f.cp is not None and f.cp.flags & TeslaFlags.HAS_BODY_CONTROLS
    vin_ok = f.cp is not None and vin_valid(f.cp.carVin)
    dlc = ", ".join(str(n) for n in sorted({len(d) for d in stock}))
    hz = len(src2) / span if span > 0 else 0.
    equal = ", each equal to a stock frame" if src128 and not odd else ""
    text = (f"present on src 2 at {hz:.1f} Hz ({len(src2)} frames, dlc {dlc}); {len(src128)} forwarded on src 128{equal}; " +
            f"openpilot sent {len(f.sent[DAS_BODY_CONTROLS])}; HAS_BODY_CONTROLS {'set' if body else 'clear'}" +
            (" (sunnypilot's port never sets it)" if f.sunnypilot else "") + ": a lamp check can run")
    if not vin_ok:
      text += ", but car signals can't turn on while the VIN is unreadable"
  else:
    text = f"absent on src 2 in {span:.0f} s: the car can't signal, the driver signals"
  return Line(name, WARN if warns else INFO, "; ".join(warns + [text]))


def check_stalk(f: Facts) -> list[Line]:
  tt, why = can_window(f)
  if tt is None:
    return [Line("driver's blinker", UNKNOWN, why), Line("X179 add-on", UNKNOWN, why)]
  ui, span = after(f, UI_WARNING, tt, PARTY), di_span(f, tt)
  if ui:
    blinker = Line("driver's blinker", GO, f"UI_warning 0x311 on src 0: {len(ui)} frames, carState's blinker read")
  elif span < MIN_ABSENT_S:
    blinker = Line("driver's blinker", UNKNOWN, f"no UI_warning 0x311 on src 0, but only {span:.1f} s of can with DI_speed")
  else:
    blinker = Line("driver's blinker", NO_GO, f"no UI_warning 0x311 on src 0 in {span:.1f} s: carState can't read the driver's blinker",
                   "check the harness's party CAN connection")
  by_src = {name: Counter(s for _, s, _ in after(f, address, tt)) for address, name in X179.items()}
  on_bus1 = [name for name, srcs in by_src.items() if srcs[VEHICLE]]
  elsewhere = {name: dict(srcs) for name, srcs in by_src.items() if srcs and not srcs[VEHICLE]}
  if on_bus1:
    x179 = Line("X179 add-on", INFO, f"{', '.join(on_bus1)} on src 1: an X179 vehicle-CAN add-on is fitted; this fork reads bus 1 only for the radar's 0x410")
  elif elsewhere:
    x179 = Line("X179 add-on", INFO, f"none on src 1, so no X179 add-on; seen by src: {elsewhere}")
  else:
    x179 = Line("X179 add-on", INFO, "no 0x249, 0x3F5 or 0x3DF on any bus: no X179 add-on")
  return [blinker, x179]


def check_car_signals(f: Facts, pv: ParamView) -> Line:
  name = "car signals"
  if f.sunnypilot:
    return Line(name, INFO, NOT_APPLICABLE)
  p = pv.get("TurnPilotCarSignals")
  if p.state == "unknown":
    return unknown_param(name, f.live)
  mode = p.value or "off"
  seen = []
  if f.signal_status:
    seen.append(f"carState.carSignal {dict(f.signal_status.most_common(3))}")
  if f.plans:
    seen.append(f"turnpilotPlan.turnSignal {dict(Counter(r.signal for r in f.plans).most_common(3))}")
  if f.signal_rejected:
    seen.append(f"the panda refused {f.signal_rejected} of our turn-signal frames")
  tail = f"; {'; '.join(seen)}" if seen else ""
  if mode == "off":
    status = WARN if f.signal_rejected else GO
    default = " (default)" if p.state == "default" else ""
    return Line(name, status, f'off{default}: the driver signals when prompted ("Signal Left", "Signal Right"){tail}')
  fix = f"{PREFLIGHT_SET} TurnPilotCarSignals=off"
  if mode == "check":
    return Line(name, WARN, f"check: this ignition runs the lamp check; Saturday plans for off{tail}", fix)
  if mode == "on":
    from openpilot.selfdrive.turnpilot.car_signals import record_matches
    record = pv.get("TurnPilotCarSignalsVerified").value
    ok, why = record_matches(record, f.cp.carVin, f.cp.carFw) if f.cp is not None else (False, "no carParams")
    return Line(name, WARN, f"on: Saturday plans for off; the verified record {'matches' if ok else 'does not match: ' + why}{tail}", fix)
  return Line(name, WARN, f"{mode!r} isn't off, check or on: read as off{tail}", fix)


def check_vin(f: Facts) -> Line:
  name = "VIN"
  if f.cp is None:
    return Line(name, UNKNOWN, "no carParams")
  vin = f.cp.carVin
  if vin_valid(vin):
    return Line(name, INFO, "valid, check digit passes (not printed)")
  if not vin or set(vin) <= {"0"}:
    return Line(name, INFO, "all zeros, as on every Tesla (opendbc has no Tesla VIN query): a car-signal record can't match")
  return Line(name, INFO, "unreadable, the check digit fails: a car-signal record can't match")


def check_calibration(f: Facts) -> Line:
  name = "calibration"
  stored = f"; CalibrationParams {'stored' if f.extra.get('CalibrationParams') else 'absent'}" if f.live else ""
  if not f.calib:
    return Line(name, UNKNOWN, "no extrinsicsCalibration" + stored)
  c = max(f.calib)
  rpy = ", ".join(f"{math.degrees(x):.2f}" for x in c.rpy)
  height = f", height {c.height[0]:.2f} m" if c.height else ""
  detail = f"{c.status}, {c.perc}%, {c.blocks} valid blocks, rpyCalib [{rpy}] deg{height}{stored}"
  if c.status == "calibrated":
    return Line(name, GO, detail)
  if c.status == "invalid":
    return Line(name, NO_GO, detail + ": the mount is out of range", "re-mount the comma level and centred, then drive to calibrate")
  return Line(name, NO_GO, detail + ": openpilot won't engage",
              "the calibration drive: straight roads above 15 mph, at least 25 s, until it reads calibrated")


def check_camera(f: Facts) -> Line:
  name = "road camera"
  if not f.sensors:
    return Line(name, UNKNOWN, "no narrowRoadCameraState")
  if set(f.sensors) == {CAMERA_SENSOR}:
    return Line(name, GO, f"{CAMERA_SENSOR} on all {f.sensors[CAMERA_SENSOR]} frames: the 1344x760 sensor preflight and the detector assume")
  return Line(name, NO_GO, f"{dict(f.sensors)}: preflight and the detector assume the {CAMERA_SENSOR}'s 1344x760",
              "another comma or camera: the detector pkl won't match it")


def check_big_model(f: Facts) -> Line:
  name = "big model"
  if not f.model:
    return Line(name, UNKNOWN, "no modelV2" + (": a qlog doesn't keep it" if f.qlog else " in the window"))
  rows = sorted(f.model)
  judged = not f.sunnypilot
  named = Counter(n for _, n in f.events if n in MODEL_EVENTS)
  loading = [t for t, n in f.events if n == "bigModelLoading"] if judged else []
  failed = [t for t, n in f.events if n == "bigModelFailed"] if judged else []
  fell = [t for t, n in f.events if n == "turnpilotModelFellBack"] if judged else []
  loaded = max(loading) if loading else None
  rest = [r for r in rows if loaded is None or r.t > loaded]
  up = next((r.t for r in rest if r.big), None)
  dropped = [r for r in rest if up is not None and r.t > up and not r.big]
  faults = [r for r in f.chestnut if r.fault]

  present = f.chestnut_present
  bits = [f"chestnutPresent {'true' if present and all(present) else 'false' if present and not any(present) else 'mixed' if present else 'unknown'}"]
  volts = [r.supply_mv / 1000 for r in f.chestnut if r.supply_mv]
  if volts:
    bits.append(f"supply {min(volts):.1f}-{max(volts):.1f} V")
  if f.chestnut:
    bits.append("supplyFault" if faults else "no supply fault")
    bits.append("pcie " + "/".join(f"0x{v:02x}" for v in sorted({r.pcie for r in f.chestnut})))
    temp, gpu = max(r.temp for r in f.chestnut), max(r.gpu for r in f.chestnut)
    bits.append(f"GPU max {temp:.0f} C, {gpu}%" if temp or gpu else "the GPU reported no temperature or load")
  bits.append(f"modelExecutionTime median {statistics.median(r.execution_time for r in rows) * 1000:.0f} ms, " +
              f"frameDropPerc max {max(r.frame_drop for r in rows):.0f}%")
  if not judged and named:
    bits.append(f"sunnypilot events (numbered differently, not judged): {dict(named)}")
  context = "; " + ", ".join(bits)

  problems = []
  if failed:
    problems.append(f"bigModelFailed at {secs(f, failed[0]):.2f} s")
  if fell:
    problems.append(f"turnpilotModelFellBack at {secs(f, fell[0]):.2f} s")
  if faults:
    problems.append(f"chestnut supplyFault on {len(faults)} chestnutState")
  if up is None and not problems:
    still = (f.extra.get("ChestnutLoading") or (loaded is not None and (f.t1 - loaded) / 1e9 <= LOADING_TAIL_S))
    if still:
      return Line(name, UNKNOWN, f"still loading: modelV2.big false on all {len(rest)} frames so far{context}")
  if up is None:
    since = f" after loading at {secs(f, loaded):.2f} s" if loaded is not None else ""
    problems.append(f"modelV2.big false on all {len(rest)} frames{since}: the small model is running")
  elif dropped:
    problems.append(f"modelV2.big false on {len(dropped)} frames after it came up at {secs(f, up):.2f} s")
  if problems:
    return Line(name, NO_GO, "; ".join(problems) + context, "restart the car; if it fails again, check the chestnut's seating and 12 V supply")
  return Line(name, GO, f"modelV2.big true on every frame from {secs(f, up):.2f} s{context}")


def check_detector(f: Facts, pv: ParamView) -> Line:
  name = "detector"
  if f.sunnypilot:
    return Line(name, INFO, NOT_APPLICABLE)
  events = Counter(n for _, n in f.events if n in DETECTOR_EVENTS)
  if events:
    return Line(name, NO_GO, f"{dict(events)}: TurnPilot handed back", "restart the car; if it repeats, read modeld's log for the detector's load")
  enabled = pv.get("TurnPilotEnabled")
  if enabled.state == "unknown" or not enabled.value:
    return Line(name, UNKNOWN, f"TurnPilot off (TurnPilotEnabled {shown(enabled)}): the detector doesn't run")
  if len(f.detections) < 2:
    return Line(name, UNKNOWN, "no turnpilotDetections" + (": a qlog doesn't keep them" if f.qlog else " in the window"))
  ts = sorted(r.t for r in f.detections)
  hz = (len(ts) - 1) / span_s(ts) if span_s(ts) > 0 else 0.
  gaps = [(b - a) / 1e9 for a, b in zip(ts, ts[1:], strict=False)]
  if f.model:
    gaps.append(max(0., (max(r.t for r in f.model) - ts[-1]) / 1e9))  # the detector stopped while the model ran on
  gap = max(gaps)
  ms = [r.execution_time * 1000 for r in f.detections]
  bits = [f"{hz:.1f} Hz", f"longest gap {gap:.2f} s", f"executionTime median {statistics.median(ms):.0f} ms, max {max(ms):.0f} ms",
          f"cameras {dict(Counter(r.camera for r in f.detections))}",
          f"lanePaint {Counter(r.lane_paint for r in f.detections).most_common(1)[0][0]}",
          f"lights {Counter(r.light for r in f.detections).most_common(1)[0][0]}"]
  if len(f.far_t) > 1:
    bits.append(f"far crop {(len(f.far_t) - 1) / span_s(f.far_t):.1f} Hz")
  if f.plans:
    bits.append(f"plan {Counter(r.state for r in f.plans).most_common(1)[0][0]}")
    handbacks = Counter(r.handback for r in f.plans if r.handback != "none")
    if handbacks:
      bits.append(f"handbacks {dict(handbacks)}")
  detail = ", ".join(bits)
  cadence = f"it runs at {DETECTOR_HZ:g} Hz, one run in {DETECTOR_RUN_EVERY} model frames at {MODEL_HZ:g} Hz"
  if hz < DETECTOR_MIN_HZ or gap > DETECTOR_DEADLINE_S:
    return Line(name, NO_GO, f"{detail}: under {DETECTOR_MIN_HZ:g} Hz or a gap over {DETECTOR_DEADLINE_S} s (turnpilotd's deadline); {cadence}",
                "restart the car; if it repeats, check the chestnut and modeld's log")
  if hz < DETECTOR_GO_HZ:
    return Line(name, WARN, f"{detail}: under {DETECTOR_GO_HZ:g} Hz, so runs are being put off (no room in the step) or model frames dropped; {cadence}",
                "read the big model line's frameDropPerc and modeld's log: the driving model or the detector is running slow")
  return Line(name, GO, f"{detail}; {cadence}")


def check_faults(f: Facts) -> Line:
  name = "TurnPilot faults"
  if f.sunnypilot:
    return Line(name, INFO, NOT_APPLICABLE)
  if not f.plans:
    return Line(name, UNKNOWN, "no turnpilotPlan")
  faults = Counter(r.fault for r in f.plans)
  shadow = Counter(r.shadow for r in f.plans)
  context = f"; shadow on {shadow[True]} of {len(f.plans)} plans"
  if faults["noLongitudinal"]:
    return Line(name, NO_GO, f"noLongitudinal on {faults['noLongitudinal']} plans: openpilot doesn't control speed, so TurnPilot does nothing{context}",
                "turn alpha longitudinal on (see the longitudinal line), then turn the car off and on")
  if faults["disengageOnAccelerator"]:
    return Line(name, NO_GO, f"disengageOnAccelerator on {faults['disengageOnAccelerator']} plans: TurnPilot refuses with gas not disengaging{context}",
                REQUIRED[3][2])
  if faults["error"]:
    return Line(name, WARN, f"error on {faults['error']} plans: turnpilotd failed those frames (logged){context}", "read turnpilotd's log")
  return Line(name, GO, f"fault none on all {len(f.plans)} plans{context}")


def check_health(f: Facts) -> Line:
  name = "car health"
  if not f.car_state:
    return Line(name, UNKNOWN, "no carState")
  tt = tesla_time(f)
  if tt is None:
    return Line(name, UNKNOWN, "the panda never entered tesla safety, so startup can't be told from a fault")
  before = [r for r in f.car_state if r[0] < tt and not r[1]]
  on = [r for r in f.car_state if r[0] >= tt]
  bad = [r for r in on if not r[1]]
  errors = max((r[2] for r in on), default=0)
  names = Counter(n for _, n in f.events)
  read = sum(t >= tt for t in f.events_t)  # onroadEvents after the tesla time: what "none of the events" rests on
  bits = [f"canErrorCounter max {errors}"]
  if before:
    bits.append(f"canValid false on {len(before)} frames before the tesla time (startup)")
  problems, fixes, missing = [], [], []
  if not on:
    missing.append(f"no carState after the tesla time ({len(f.car_state)} before it), so canValid can't be judged")
  if bad:
    problems.append(f"canValid false on {len(bad)} of {len(on)} carState after the tesla time, first at {secs(f, bad[0][0]):.2f} s")
    fixes.append("check the harness, then turn the car off and on")
  if f.sunnypilot:
    if names:
      bits.append(f"events (sunnypilot numbers some differently, not judged): {dict(names.most_common(8))}")
  else:
    found = {n: names[n] for n in HEALTH_EVENTS if names[n]}
    if found:
      problems.append(f"events {found}")
      fixes += [{"canError": "check the harness", "carUnrecognized": "see the car and EPS lines",
                 "invalidLkasSetting": "turn Autosteer off in the Tesla's Autopilot settings (TACC only)",
                 "calibrationIncomplete": "see the calibration line"}[n] for n in found]
    elif not read:
      before_tt = f", {len(f.events_t)} before it" if f.events_t else ""
      missing.append(f"no onroadEvents after the tesla time{before_tt} (selfdrived sends them every second onroad), " +
                     f"so {', '.join(HEALTH_EVENTS)} can't be read")
    other = {n: c for n, c in names.most_common(6) if n not in HEALTH_EVENTS}
    if other:
      bits.append(f"other events {other}")
  if problems:
    return Line(name, NO_GO, "; ".join(problems + missing) + "; " + ", ".join(bits), "; ".join(fixes))
  if missing:
    return Line(name, UNKNOWN, "; ".join(missing) + "; " + ", ".join(bits))
  valid = f"canValid true on all {len(on)} carState after the tesla time"
  if f.sunnypilot:
    return Line(name, GO, f"{valid}; " + ", ".join(bits))
  return Line(name, GO, f"{valid}, none of {', '.join(HEALTH_EVENTS)} in {read} onroadEvents after it; " + ", ".join(bits))


def noted(p: Param) -> Line:
  key, v, text = p.key, p.value, shown(p)
  if key == "TurnPilotShadowMode":
    return Line(key, INFO, f"{text}: TurnPilot computes and nothing acts") if v else \
      Line(key, WARN, f"{text}: TurnPilot is live, it takes turns, changes lanes and brakes", f"{PREFLIGHT_SET} TurnPilotShadowMode=1 for the first drives")
  if key == "DisableUpdates":
    return Line(key, INFO, f"{text}: no update can land mid-test") if v else \
      Line(key, WARN, f"{text}: the updater can fetch another build on a test day", f"{PREFLIGHT_SET} DisableUpdates=1")
  if key == "TurnPilotLeftFarRangeM":
    return Line(key, INFO, f"{text}: the far crop adds no reach") if not v else \
      Line(key, WARN, f"{text} m: it must come from a measurement on this car", f"{PREFLIGHT_SET} TurnPilotLeftFarRangeM=0 until it is measured")
  if key == "TurnPilotAutoLeft":
    return Line(key, INFO, f"{text}: with TurnPilotLeftRangeM 0 every unprotected left is skipped and rerouted") if v == "auto" else \
      Line(key, WARN, f"{text}: off turns the skip into a handback at the line", f"{PREFLIGHT_SET} TurnPilotAutoLeft=auto")
  if key == "TurnPilotGoOverride":
    return Line(key, INFO, f"{text}: TurnPilot's certified go from a standstill (plannerd's bounded lift)")
  return Line(key, INFO, f"{text}: seconds a standstill turn waits for the car to pull away before it hands back")


def check_params(f: Facts, pv: ParamView) -> list[Line]:
  if f.sunnypilot:
    raw = f.params_raw or {}
    had = ", ".join(f"{k} {raw[k].decode(errors='replace')}" for k in SUNNYPILOT_CONTEXT if k in raw) or "none of them set"
    return [Line("TurnPilot params", INFO, f"{NOT_APPLICABLE}; sunnypilot had {had}; this fork needs DisengageOnAccelerator 1 (no MADS)")]
  lines = []
  for key, want, fix in REQUIRED:
    p = pv.get(key)
    if p.state == "unknown":
      lines.append(unknown_param(key, f.live))
      continue
    ok = p.value in (None, "") if want is None else p.value == want and type(p.value) is type(want)
    if ok:
      lines.append(Line(key, GO, shown(p)))
    else:
      need = "unset" if want is None else shown(Param(key, "set", want))
      lines.append(Line(key, NO_GO, f"{shown(p)}, Saturday needs {need}", fix))
  for key in NOTED:
    p = pv.get(key)
    lines.append(unknown_param(key, f.live) if p.state == "unknown" else noted(p))
  for key in MAP_PARAMS:
    p = pv.get(key)
    if p.state == "unknown":
      lines.append(unknown_param(key, f.live))
    elif p.state == "unlogged":
      lines.append(Line(key, UNKNOWN, f"not logged (DONT_LOG), {'set' if p.value else 'absent'} on the route: check it on the device",
                        "run this with --live, or preflight's map server line"))
    elif p.value:
      lines.append(Line(key, GO, p.value if key == "TurnPilotMapHost" else "set (not printed)"))
    else:
      lines.append(Line(key, NO_GO, "unset: navd can't route from the map server", "enter it in the web app's Map server section"))
  return lines


def evaluate(f: Facts, pv: ParamView) -> list[Line]:
  lines = [check_branch(f, pv), check_car(f), *check_eps(f), check_source(f), check_three_bit(f), check_das_settings(f),
           check_longitudinal(f, pv), check_panda(f), *check_panda_long(f), check_turn_signal_frame(f), *check_stalk(f),
           check_car_signals(f, pv), check_vin(f), check_calibration(f), check_camera(f), check_big_model(f),
           check_detector(f, pv), check_faults(f), check_health(f), *check_params(f, pv)]
  return lines + [Line(name, INFO, how) for name, how in BY_HAND]


def exit_code(lines: list[Line]) -> int:
  statuses = {line.status for line in lines}
  return 1 if NO_GO in statuses else 2 if UNKNOWN in statuses else 0


def render(lines: list[Line]) -> str:
  width = max(len(line.name) for line in lines)
  out = []
  for line in lines:
    out.append(f"{line.status:<7}  {line.name:<{width}}  {line.detail}".rstrip())
    if line.fix and line.status in (NO_GO, WARN):
      out.append(f"{'':<7}  {'':<{width}}  fix: {line.fix}")
  count = Counter(line.status for line in lines)
  out.append("\n" + ", ".join(f"{count[s]} {s}" for s in (GO, NO_GO, WARN, UNKNOWN, INFO)) + f": exit {exit_code(lines)}")
  return "\n".join(out)


def header(f: Facts) -> str:
  parts = [f.where, f"{(f.t1 - f.t0) / 1e9:.1f} s" if f.t0 is not None else "no messages"]
  if f.sunnypilot:
    parts.append(f"a sunnypilot log ({(f.init or {}).get('branch', '')}): TurnPilot's items not applicable")
  if f.qlog:
    parts.append("a qlog")
  if f.t0 is not None:
    tt = tesla_time(f)
    parts.append(f"panda tesla from {secs(f, tt):.2f} s" if tt is not None else "the panda never entered tesla safety")
  return "car_preflight: " + "; ".join(parts)


# ------------------------------------------------------------------------------------------------------------ reading

LOG_NAME = re.compile(r"(?:^|[^A-Za-z])(rlog|qlog)(?:\.zst|\.bz2)?$")  # rlog.zst, seg0_rlog.zst, <route>--0--rlog.zst; never rlog.lock


def route_files(args: list[str]) -> list[Path]:
  """Files as given; a directory's logs in segment order, its rlogs where it has any, else its qlogs (a segment directory
  holds both, and reading both would count every message twice)."""
  def natural(p: Path):
    return [int(s) if s.isdigit() else s for s in re.split(r"(\d+)", str(p))]
  out = []
  for a in args:
    p = Path(a)
    if p.is_dir():
      by_dir = defaultdict(lambda: defaultdict(list))
      for q in p.rglob("*"):
        if q.is_file() and (m := LOG_NAME.search(q.name)):
          by_dir[q.parent][m.group(1)].append(q)
      out += sorted((q for kinds in by_dir.values() for q in kinds.get("rlog") or kinds.get("qlog") or []), key=natural)
    elif p.is_file():
      out.append(p)
    else:
      raise FileNotFoundError(a)
  return out


def collect_route(paths: list, lib=None) -> tuple[Facts, ParamView]:
  """A route's rlogs (or qlogs), read one file at a time with LogReader(only_union_types=True): a fork's own events are
  skipped. lib: a Params-like library for types and defaults; by default this build's, in a fresh temporary directory, so
  nothing on this machine is read or written."""
  from openpilot.tools.lib.logreader import LogReader
  keep = None
  if lib is None:
    from openpilot.common.params import Params
    keep = tempfile.TemporaryDirectory(prefix="car_preflight_")
    lib = Params(keep.name)
  paths = [Path(p) for p in paths]
  f = Facts(where=f"route, {len(paths)} file{'s' if len(paths) != 1 else ''} ({', '.join(p.name for p in paths)})",
            qlog=any("qlog" in p.name for p in paths))
  for p in paths:
    for msg in LogReader(str(p), only_union_types=True):
      f.feed(msg)
  return f, ParamView(lib, f.params_raw or {}, keep)


def subscribe(seconds: float):
  """Every message on SERVICES for the window. Subscribing only: nothing is published."""
  from openpilot.cereal import messaging
  poller = messaging.Poller()
  socks = [messaging.sub_sock(service, poller=poller, conflate=False) for service in SERVICES]  # held for the window
  end = time.monotonic() + seconds
  while time.monotonic() < end:
    for sock in poller.poll(100):
      for dat in messaging.drain_sock_raw(sock):
        yield messaging.log_from_bytes(dat)
  del socks


def safe_get(params, key):
  try:
    return params.get(key)
  except Exception:
    return None


def collect_live(seconds: float, params=None, messages=None) -> tuple[Facts, ParamView]:
  """The device for the window: params now, then every message (messages: the tests' stand-in for the sockets). carParams
  is logged every 50 s, so a window without one takes CarParams (this drive's), else CarParamsPersistent (the last drive)."""
  if params is None:
    from openpilot.common.params import Params
    params = Params()
  f = Facts(where=f"live, {seconds:g} s window", live=True)
  branch = safe_get(params, "GitBranch")
  if branch:
    f.init = {"branch": branch, "commit": safe_get(params, "GitCommit") or "", "remote": safe_get(params, "GitRemote") or ""}
    f.sunnypilot = "sunnypilot" in f.init["remote"].lower()
  f.extra["CalibrationParams"] = safe_get(params, "CalibrationParams") is not None
  f.extra["ChestnutLoading"] = bool(safe_get(params, "ChestnutLoading"))
  for msg in (subscribe(seconds) if messages is None else messages):
    f.feed(msg)
  if f.cp is None:
    from opendbc.car.structs import car
    from openpilot.cereal import messaging
    for key, label in (("CarParams", "this drive's CarParams param"), ("CarParamsPersistent", "CarParamsPersistent: the last drive")):
      raw = safe_get(params, key)
      if raw:
        f.cp, f.cp_from = messaging.log_from_bytes(raw, car.CarParams).as_builder(), label
        break
  return f, ParamView(params)


def check_route(paths: list, lib=None) -> list[Line]:
  return evaluate(*collect_route(paths, lib))


def check_live(seconds: float, params=None, messages=None) -> list[Line]:
  return evaluate(*collect_live(seconds, params, messages))


def main(argv=None) -> int:
  parser = argparse.ArgumentParser(description="TurnPilot's car checklist for Chase's Model 3: read only, from the device or a route")
  source = parser.add_mutually_exclusive_group(required=True)
  source.add_argument("--live", action="store_true", help="read the running device (cd /data/openpilot first), parked with the car on")
  source.add_argument("--route", nargs="+", metavar="RLOG", help="rlog files, or directories holding them")
  parser.add_argument("--seconds", type=float, default=30., help="the --live window (default 30)")
  args = parser.parse_args(argv)
  try:
    f, pv = collect_live(args.seconds) if args.live else collect_route(route_files(args.route))
  except FileNotFoundError as e:
    print(f"car_preflight: no such file or directory: {e}", file=sys.stderr)
    return 2
  lines = evaluate(f, pv)
  print(header(f))
  print(render(lines))
  return exit_code(lines)


if __name__ == "__main__":
  sys.exit(main())
