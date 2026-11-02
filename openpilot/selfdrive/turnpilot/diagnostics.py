"""Diagnostics: one plain-text report Chase copies from the phone page (Diagnostics, Copy) and texts to Kellen.

It says the build (GitCommit, GitBranch), the AGNOS version, the driving model and the detector as they run now (modelV2.big:
the big model on the chestnut; the step's times; nativeStatus), preflight's lines (python -m openpilot.selfdrive.turnpilot.
preflight, run apart so turnpilot_web never imports the planner), and the last few TurnPilot events: its handbacks and
faults (turnpilotPlan) and the errors its code logged (errorLogMessage), kept in TurnPilotDiagEvents across reboots. Its GPS
section says which GPS daemon runs, each GPS message's state and quality and whether LastGPSPosition is saved.

It never says the map key, the web token or the pairing URL that carries it, or any position: preflight's pairing and token
lines are dropped, and every line is scrubbed of the saved key and token and of any number with four or more decimals (a
latitude or longitude), which reads [position].
"""
import json
import os
import re
import subprocess
import sys
import threading
import time
import traceback

from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.turnpilot.maps.paths import wall_time

EVENTS_PARAM = "TurnPilotDiagEvents"
EVENTS_KEPT = 10       # the last events the report lists, newest first
REPEAT_S = 60.         # the same event again within this is counted on the last one, not listed again
WHY_MAX = 200          # characters of an event's text
BIG_WHY_MAX = 600      # a big model failure's: its exception, its timings and its traceback's last frames
BIG_WHAT = "big model"  # the prefix of modeld's big model events (note_big_model)
BIG_TB_FRAMES = 3      # the traceback's last frames a big model failure keeps
CHESTNUT_PARAMS = ("ChestnutLoading", "ChestnutActive", "ChestnutRetry")
PREFLIGHT_TIMEOUT_S = 60.
PREFLIGHT_CACHE_S = 30.  # a second Copy or refresh within this reuses the last run
DROPPED_CHECKS = ("pairing URL", "web token")  # they carry or concern the web token
POSITION = re.compile(r"-?\d{1,3}\.\d{4,}")
# turnpilotPlan.controlAlert values that say something failed or TurnPilot acted hard, as against its notes and route notices
FAULT_ALERTS = frozenset((
  "cannotStopComfortably", "stopOverrun", "mapControlsUnavailable", "lightSourceRejected", "junctionUncertain",
  "offRouteStopsUnknown", "hardBrake", "farViewOff", "sideCamerasOff", "cameraPaused", "stalledInJunction",
  "overrideWriteFailed", "stopRoutingFailed", "carCrossing",
))
TURNPILOT_DAEMONS = frozenset(("turnpilotd", "turnpilot_web"))
GPS_SERVICES = ("gpsLocation", "gpsLocationExternal")
GPS_DAEMONS = ("ubloxd", "pigeond", "qcomgpsd")
QCOM_NO_FIX_V_ACC = 500.  # qcomgpsd: the modem clips verticalAccuracy at 500, and hasFix is False there
NATIVE_WORDS = {"off": "off (the far crop)", "strip": "strip", "full": "full (the whole frame natively)",
                "fallback": "fell back to the far crop for the drive", "unavailable": "on, but not loaded: the far crop"}


def scrub(text: str, secrets=()) -> str:
  for secret in secrets:
    if secret:
      text = text.replace(secret, "[redacted]")
  return POSITION.sub("[position]", text)


def secrets_of(params) -> tuple[str, ...]:
  return tuple(s for s in (params.get("TurnPilotMapKey"), params.get("TurnPilotWebToken")) if isinstance(s, str) and len(s) >= 4)


def log_event(record) -> tuple[str, str] | None:
  """(what, text) for an errorLogMessage record logged by TurnPilot's code, else None."""
  try:
    rec = json.loads(record) if isinstance(record, (str, bytes)) else record
  except ValueError:
    return None
  if not isinstance(rec, dict) or not isinstance(rec.get("levelnum"), int) or rec["levelnum"] < 40:
    return None
  msg = rec.get("msg")
  msg = msg if isinstance(msg, str) else json.dumps(msg, default=str)
  daemon = str((rec.get("ctx") or {}).get("daemon") or "")
  if daemon not in TURNPILOT_DAEMONS and "turnpilot" not in msg.lower():
    return None
  text = msg.strip().splitlines()[0] if msg.strip() else ""
  if isinstance(rec.get("exc_info"), str) and rec["exc_info"].strip():
    text += f" ({rec['exc_info'].strip().splitlines()[-1]})"  # the exception's type and message
  return f"error {daemon or 'log'}", text


class EventLog:
  """The last EVENTS_KEPT TurnPilot events, saved in TurnPilotDiagEvents as they come. Fed by the web app's status reader:
  see_plan with each turnpilotPlan snapshot (a handback's or fault's onset), see_log with each errorLogMessage."""

  def __init__(self, params, now=wall_time):
    self.params, self.now = params, now
    self.lock = threading.Lock()
    self.last_prompt = self.last_alert = None
    self.last_missed = False
    try:
      saved = params.get(EVENTS_PARAM)
    except Exception:  # Params raises UnknownKeyName on a build whose params predate the key
      saved = None
    self.events: list[dict] = [e for e in saved if isinstance(e, dict)][-EVENTS_KEPT:] if isinstance(saved, list) else []

  def see_plan(self, plan: dict | None) -> None:
    """plan: the snapshot's plan (status.read_services), None while turnpilotd isn't publishing."""
    if plan is None:  # the next drive's first handback is an onset again
      self.last_prompt, self.last_alert, self.last_missed = None, None, False
      return
    prompt, alert, missed, reason = plan.get("prompt"), plan.get("alert"), bool(plan.get("missed")), plan.get("reason") or ""
    if prompt != self.last_prompt and isinstance(prompt, str) and prompt.endswith("TakeControl"):
      self.add(f"handback {prompt}", reason)
    if alert != self.last_alert and alert in FAULT_ALERTS:
      self.add(f"alert {alert}", reason)
    if missed and not self.last_missed:
      self.add("turn missed", reason)
    self.last_prompt, self.last_alert, self.last_missed = prompt, alert, missed

  def see_log(self, record) -> None:
    if (found := log_event(record)) is not None:
      self.add(*found)

  def merge_saved(self) -> None:
    """Take in the events another process saved since (modeld's big model failures, note_big_model). Under self.lock."""
    try:
      saved = self.params.get(EVENTS_PARAM)
    except Exception:
      return
    if not isinstance(saved, list):
      return
    mine = {(e.get("t"), e.get("what")) for e in self.events}
    theirs = [e for e in saved if isinstance(e, dict) and (e.get("t"), e.get("what")) not in mine and str(e.get("what", "")).startswith(BIG_WHAT)]
    if theirs:
      self.events = sorted([*self.events, *theirs], key=lambda e: e.get("t", 0.))[-EVENTS_KEPT:]

  def add(self, what: str, why: str) -> None:
    why = scrub(why)[:WHY_MAX]  # never a position in the saved param
    with self.lock:
      self.merge_saved()  # a write from modeld isn't lost to this one
      now, last = self.now(), self.events[-1] if self.events else None
      if last is not None and last.get("what") == what and last.get("why") == why and now - last.get("t", 0.) < REPEAT_S:
        last["n"], last["t"] = last.get("n", 1) + 1, now
      else:
        self.events = [*self.events, {"t": now, "what": what, "why": why}][-EVENTS_KEPT:]
      events = [dict(e) for e in self.events]
    try:
      self.params.put(EVENTS_PARAM, events, block=True)
    except Exception:
      pass  # the report still has them until a reboot

  def get(self) -> list[dict]:
    with self.lock:
      self.merge_saved()
      return [dict(e) for e in self.events]


def short_traceback(exc: BaseException, frames: int = BIG_TB_FRAMES) -> str:
  """The exception's last frames as 'file:line in func', paths from the repo (or the bare file name outside it)."""
  out = []
  for fr in traceback.extract_tb(exc.__traceback__)[-frames:]:
    path = fr.filename
    if path.startswith(BASEDIR):
      path = os.path.relpath(path, BASEDIR)
    elif "site-packages/" in path:
      path = path.split("site-packages/", 1)[1]
    else:
      path = os.path.basename(path)
    out.append(f"{path}:{fr.lineno} in {fr.name}")
  return " < ".join(reversed(out))


def big_model_why(kind: str, exc: BaseException | None, took_s: float, retry: bool, waited_s: float | None, wait_error: str | None) -> str:
  """kind: 'timed out', 'failed to load' or 'failed while running'. Durations at one decimal, never read as a position."""
  head = f"{type(exc).__name__}: {str(exc).strip().splitlines()[0] if str(exc).strip() else ''}".rstrip(": ") if exc is not None else ""
  how = f"timed out after {took_s:.1f} s" if kind == "timed out" else f"raised after {took_s:.1f} s"
  wait = (f"chestnut enumerated in {waited_s:.1f} s" if waited_s is not None else
          f"wait_for_chestnut raised {wait_error}" if wait_error else "wait_for_chestnut didn't return")
  parts = [p for p in (head, how, "the ChestnutRetry try" if retry else "the first try", wait) if p]
  tb = short_traceback(exc) if exc is not None and exc.__traceback__ is not None else ""
  return "; ".join(parts) + (f"; at {tb}" if tb else "")


def note_big_model(params, kind: str, exc: BaseException | None = None, took_s: float = 0., retry: bool = False,
                   waited_s: float | None = None, wait_error: str | None = None, now=wall_time) -> None:
  """modeld's record of a big model failure in TurnPilotDiagEvents, for the report's events and its driving model line.
  Called only from modeld's except and timeout paths; never raises."""
  try:
    why = scrub(big_model_why(kind, exc, took_s, retry, waited_s, wait_error), secrets_of(params))[:BIG_WHY_MAX]
    saved = params.get(EVENTS_PARAM)
    events = [e for e in saved if isinstance(e, dict)] if isinstance(saved, list) else []
    events = [*events, {"t": now(), "what": f"{BIG_WHAT} {kind}", "why": why}][-EVENTS_KEPT:]
    params.put(EVENTS_PARAM, events, block=True)
  except Exception:
    pass


class Preflight:
  """preflight's lines, run as its own process (its collect() imports the planner and the map engine), at most every
  PREFLIGHT_CACHE_S. run: a stand-in for tests."""

  def __init__(self, run=None):
    self.run = run or self.run_module
    self.lock = threading.Lock()
    self.at, self.lines = -1e9, []

  @staticmethod
  def run_module() -> list[str]:
    try:
      out = subprocess.run([sys.executable, "-m", "openpilot.selfdrive.turnpilot.preflight"], cwd=BASEDIR, env=dict(os.environ),
                           capture_output=True, text=True, timeout=PREFLIGHT_TIMEOUT_S)
    except subprocess.TimeoutExpired:
      return [f"preflight took over {PREFLIGHT_TIMEOUT_S:.0f} s"]
    except OSError as e:
      return [f"preflight didn't start: {e}"]
    lines = out.stdout.splitlines()
    if out.returncode not in (0, 1) or not lines:
      tail = (out.stderr.strip().splitlines() or [""])[-1]
      lines.append(f"preflight exited {out.returncode}: {tail}")
    return lines

  def get(self, monotonic=time.monotonic) -> list[str]:
    with self.lock:
      if monotonic() - self.at >= PREFLIGHT_CACHE_S:
        self.lines, self.at = keep_lines(self.run()), monotonic()
      return list(self.lines)


def keep_lines(lines: list[str]) -> list[str]:
  """preflight's output without the checks that carry the token (DROPPED_CHECKS) and their fix lines."""
  kept, dropping = [], False
  for line in lines:
    words = line.split(None, 1)
    if words and words[0] in ("PASS", "FAIL", "WARN", "INFO"):
      dropping = len(words) > 1 and words[1].startswith(DROPPED_CHECKS)
    elif not line.startswith(" "):
      dropping = False
    if not dropping:
      kept.append(line)
  return kept


def agnos_version() -> str | None:
  try:
    from openpilot.common.hardware import HARDWARE
    return HARDWARE.get_os_version()
  except Exception:
    return None


def build_text(params) -> str:
  commit, branch = params.get("GitCommit") or "", params.get("GitBranch") or ""
  return f"{commit[:10] or 'unknown'}" + (f" on {branch}" if branch else "")


def settings_line(params, agnos: str | None) -> str:
  """settings > turnpilot's diagnostics card: the build and AGNOS, and where the full report is."""
  return f"build {build_text(params)}, AGNOS {agnos or 'unknown'}. The full report, with a copy button, is on the phone page."


def read_param(params, key):
  try:
    return params.get(key)
  except Exception:
    return None


def chestnut_line(params) -> str:
  words = {True: "1", False: "0", None: "unset"}
  return "chestnut params: " + ", ".join(f"{k} {words.get(read_param(params, k), read_param(params, k))}" for k in CHESTNUT_PARAMS)


def small_model_why(params, events: list[dict]) -> str:
  """Why the small model drives: modeld's ChestnutActive (False: it had a chestnut and the big model isn't running) and
  its newest big model event."""
  if params is None:
    return "(no chestnut)"
  active = read_param(params, "ChestnutActive")
  if active is not False:
    if read_param(params, "ChestnutRetry") and read_param(params, "ChestnutLoading") is not True:
      return "(the big model failed once, retrying after an onroad cycle; see events)"
    return "(modeld saw no chestnut when it started)" if active is None else "(no chestnut)"
  last = next((e for e in reversed(events or []) if str(e.get("what", "")).startswith(BIG_WHAT)), None)
  if last is None:
    return "(big model not running, no reason recorded)"
  what, why = last["what"], str(last.get("why", ""))
  if what.endswith("timed out"):
    took = re.search(r"timed out after (\d+(?:\.\d)?) s", why)
    return f"(big model timed out after {took.group(1) if took else '?'} s; see events)"
  if what.endswith("failed while running"):
    return "(big model failed while driving, fell back; see events)"
  return "(big model failed to load; see events)"


def model_text(live: dict | None, params=None, events: list[dict] | None = None) -> list[str]:
  """The driving model and the detector as they run now: live is the status reader's (status.read_live)."""
  if not live or live.get("model_big") is None:
    return ["driving model: not running (the car is off or modeld is down)"]
  where = "the big model on the chestnut" if live["model_big"] else f"the small model on the comma {small_model_why(params, events or [])}"
  out = [f"driving model: {where}"]
  if live.get("native") is not None:
    out.append(f"native detect: {NATIVE_WORDS.get(live['native'], live['native'])}; far crop: {live.get('far') or 'unknown'}")
  times = [f"{name} {live[key] * 1e3:.1f} ms" for name, key in (("model", "model_s"), ("full pass", "full_s"), ("far slot", "far_s"))
           if isinstance(live.get(key), (int, float))]
  if times:
    out.append(f"step times (median, last {live.get('samples', 0)}): " + ", ".join(times) + "; budget 45 ms")
  return out


def event_lines(events: list[dict]) -> list[str]:
  if not events:
    return ["  none since the last install"]
  out = []
  for e in reversed(events):
    when = time.strftime("%m-%d %H:%M:%S", time.localtime(e.get("t", 0.)))
    count = f" (x{e['n']})" if e.get("n", 1) > 1 else ""
    out.append(f"  {when}  {e.get('what', '')}{count}" + (f": {e['why']}" if e.get("why") else ""))
  return out


def gps_daemon_rule() -> bool | None:
  """process_config's rule: True runs ubloxd and pigeond (/dev/ttyHS0 and no use-quectel-gps), False qcomgpsd, None unknown."""
  try:
    from openpilot.system.manager.process_config import ublox_available
    return bool(ublox_available())
  except Exception:
    return None


def clock_valid() -> bool | None:
  try:
    from openpilot.common.time_helpers import system_time_valid
    return bool(system_time_valid())
  except Exception:
    return None


def saved_position_text(params, now: float) -> str:
  """LastGPSPosition present or not, which message it holds and how old its fix is: never its value."""
  raw = read_param(params, "LastGPSPosition")
  if not raw:
    return "none (only a saved fix fills the offroad position)"
  try:
    from openpilot.cereal import log
    with log.Event.from_bytes(raw) as evt:
      which = evt.which()
      ms = getattr(evt, which).unixTimestampMillis if which in GPS_SERVICES else 0
  except Exception:
    return "present, unreadable"
  if not ms:
    return f"present ({which})"
  return f"present ({which}, {source_age_text(now - ms / 1e3)} old)"


def source_age_text(seconds: float) -> str:
  if seconds < 120:
    return f"{seconds:.0f} s"
  if seconds < 2 * 3600:
    return f"{seconds / 60:.0f} min"
  return f"{seconds / 3600:.0f} h" if seconds < 2 * 86400 else f"{seconds / 86400:.0f} days"


def gps_service_text(name: str, g: dict | None) -> str:
  if not g or not g.get("seen"):
    return f"  {name}: never received since the web app started"
  state = "publishing" if g["up"] else f"stopped, last {source_age_text(g['age_s'])} ago"
  parts = [state, f"valid {'yes' if g['valid'] else 'no'}", f"hasFix {'yes' if g['fix'] else 'no'}", f"age {g['age_s']:.1f} s",
           f"source {g.get('source', '?')}"]
  qcom = g.get("source") == "qcomdiag"  # qcomgpsd sets neither satelliteCount nor horizontalAccuracy
  parts.append("satellites not reported" if qcom else f"satellites {g['satellites']}")
  if not qcom:
    parts.append(f"horizontal accuracy {g['h_acc']:.1f} m")
  vertical = f"vertical accuracy {g['v_acc']:.1f}"
  if qcom and g["v_acc"] == QCOM_NO_FIX_V_ACC:
    vertical += " (qcomgpsd's no-fix value)"
  parts += [vertical, f"speed accuracy {g['speed_acc']:.2f} m/s", f"bearing accuracy {g['bearing_acc']:.0f} deg"]
  return f"  {name}: " + ", ".join(parts)


def radio_text(radio: dict, now=time.monotonic) -> list[str]:
  """The u-blox's radio readings (status.see_ublox): what separates no signal at the antenna from noise drowning it."""
  if not radio:
    return ["  u-blox radio: no ubloxGnss messages read"]
  t = now()
  out = []
  if "hw" in radio:
    h = radio["hw"]
    out.append(f"  u-blox radio: noise {h['noise']} per ms, AGC {h['agc']} of 8191, antenna {h['antenna']}, antenna power " +
               f"{h['antenna_power']}, jamming {h['jamming']} ({t - h['at']:.0f} s ago)")
  if "sats" in radio:
    v = radio["sats"]
    out.append(f"  u-blox satellites: hears {v['heard']} of {v['listed']} it expects, best signal {v['best_cno']} dB-Hz " +
               f"({t - v['at']:.0f} s ago)")
  if "meas" in radio:
    out.append(f"  u-blox measurements: {radio['meas']['count']} ({t - radio['meas']['at']:.0f} s ago)")
  return out


def gps_text(gps: dict | None, params, ublox: bool | None, clock: bool | None, now: float) -> list[str]:
  """The GPS section: which daemon runs, each GPS message's state and quality, the assistance qcomgpsd gets and
  LastGPSPosition, never a coordinate. gps is the status reader's (status.read_gps); ublox is gps_daemon_rule()'s."""
  rule = {True: "ubloxd and pigeond (/dev/ttyHS0 is there)", False: "qcomgpsd (no /dev/ttyHS0, or use-quectel-gps)",
          None: "unknown"}[ublox]
  saved = read_param(params, "UbloxAvailable")
  out = ["GPS:", f"  daemon by process_config's rule: {rule}; UbloxAvailable {({True: '1', False: '0'}).get(saved, 'unset')}"]
  if gps is None:
    out.append("  status reader not running: no GPS messages read")
  else:
    procs = gps.get("procs") or {}
    if not procs:
      out.append("  running: unknown (managerState isn't publishing)")
    else:
      words = {True: "running", False: "not running", None: "not managed"}
      out.append("  running: " + ", ".join(f"{d} {words[procs.get(d)]}" for d in GPS_DAEMONS))
    services = gps.get("services") or {}
    out += [gps_service_text(s, services.get(s)) for s in GPS_SERVICES]
    out += radio_text(gps.get("radio") or {})
  if ublox is not True:
    valid = {True: 'yes', False: 'no'}.get(clock, 'unknown')
    out.append("  assistance (qcomgpsd): this build downloads no assistance data; at its start it sets the modem's time only " +
               f"if the clock is valid (clock valid now: {valid}), so a fix can be a cold start")
  out.append(f"  LastGPSPosition: {saved_position_text(params, now)}")
  return out


def report(params, live: dict | None, events: list[dict], preflight_lines: list[str], agnos: str | None, now: float | None = None,
           gps: dict | None = None, ublox=gps_daemon_rule, clock=clock_valid) -> str:
  now = wall_time() if now is None else now
  lines = [f"TurnPilot diagnostics, {time.strftime('%Y-%m-%d %H:%M:%S %Z', time.localtime(now))}",
           f"build: {build_text(params)}" + (f" ({params.get('Version')})" if params.get("Version") else ""),
           f"AGNOS: {agnos or 'unknown'}",
           *model_text(live, params, events),
           chestnut_line(params),
           "", *gps_text(gps, params, ublox(), clock(), now),
           "", "preflight:", *(f"  {ln}" for ln in preflight_lines if ln.strip()),
           "", "recent TurnPilot events (newest first):", *event_lines(events)]
  secrets = secrets_of(params)
  return "\n".join(scrub(ln, secrets) for ln in lines) + "\n"
