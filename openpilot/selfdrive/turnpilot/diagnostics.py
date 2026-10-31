"""Diagnostics: one plain-text report Chase copies from the phone page (Diagnostics, Copy) and texts to Kellen.

It says the build (GitCommit, GitBranch), the AGNOS version, the driving model and the detector as they run now (modelV2.big:
the big model on the chestnut; the step's times; nativeStatus), preflight's lines (python -m openpilot.selfdrive.turnpilot.
preflight, run apart so turnpilot_web never imports the planner), and the last few TurnPilot events: its handbacks and
faults (turnpilotPlan) and the errors its code logged (errorLogMessage), kept in TurnPilotDiagEvents across reboots.

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

from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.turnpilot.maps.paths import wall_time

EVENTS_PARAM = "TurnPilotDiagEvents"
EVENTS_KEPT = 10       # the last events the report lists, newest first
REPEAT_S = 60.         # the same event again within this is counted on the last one, not listed again
WHY_MAX = 200          # characters of an event's text
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

  def add(self, what: str, why: str) -> None:
    why = scrub(why)[:WHY_MAX]  # never a position in the saved param
    with self.lock:
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
      return [dict(e) for e in self.events]


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


def model_text(live: dict | None) -> list[str]:
  """The driving model and the detector as they run now: live is the status reader's (status.read_live)."""
  if not live or live.get("model_big") is None:
    return ["driving model: not running (the car is off or modeld is down)"]
  where = "the big model on the chestnut" if live["model_big"] else "the small model on the comma (no chestnut)"
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


def report(params, live: dict | None, events: list[dict], preflight_lines: list[str], agnos: str | None, now: float | None = None) -> str:
  now = wall_time() if now is None else now
  lines = [f"TurnPilot diagnostics, {time.strftime('%Y-%m-%d %H:%M:%S %Z', time.localtime(now))}",
           f"build: {build_text(params)}" + (f" ({params.get('Version')})" if params.get("Version") else ""),
           f"AGNOS: {agnos or 'unknown'}",
           *model_text(live),
           "", "preflight:", *(f"  {ln}" for ln in preflight_lines if ln.strip()),
           "", "recent TurnPilot events (newest first):", *event_lines(events)]
  secrets = secrets_of(params)
  return "\n".join(scrub(ln, secrets) for ln in lines) + "\n"
