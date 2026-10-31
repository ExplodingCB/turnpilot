"""TurnPilot's guided setup: its steps, what each one waits for, and the one line the home screen shows until it is ready.

Plain params in, words out, so the comma screen (ui/mici/layouts/turnpilot_setup.py), the phone and the readiness alert
(Offroad_TurnPilotSetup, set by valhalla_install.SetupThread) all say the same thing. No raylib, no network.

The steps, in order: intro, phone (the phone opened the TurnPilot page with the pairing token: TurnPilotPhoneSeen,
written by the web server), key (the map server answered the saved key: TurnPilotMapCoverageStatus ok), engine (the
map engine, pyvalhalla, is installed: navd routes each drive's corridor with it, so driving needs it), done.
"""
from dataclasses import dataclass

from openpilot.selfdrive.turnpilot.maps.coverage import STATUS_PARAM as COVERAGE_STATUS
from openpilot.selfdrive.turnpilot.maps.geofence import join_names, place_name

STEPS = ('intro', 'phone', 'key', 'engine', 'done')
SETUP_DONE = 'TurnPilotSetupDone'
SETUP_STEP = 'TurnPilotSetupStep'  # the furthest step the driver moved past, so a reboot resumes there or earlier
PHONE_SEEN = 'TurnPilotPhoneSeen'  # epoch s of the last request the phone made with the pairing token
INSTALL_PARAM = 'TurnPilotValhallaInstall'
RETRY_PARAM = 'TurnPilotValhallaRetry'
ALERT = 'Offroad_TurnPilotSetup'
PHONE_LIVE_S = 30.  # the page polls every 2 s while it is open

GOOD, WAITING, BUSY, ERROR = 'good', 'waiting', 'busy', 'error'


@dataclass(frozen=True)
class StepState:
  state: str  # GOOD, WAITING, BUSY or ERROR
  title: str  # short and lowercase, for the card's title
  detail: str  # one short line for the card's value


def phone_state(params, now: float) -> StepState:
  seen = params.get(PHONE_SEEN)
  if not isinstance(seen, (int, float)) or seen <= 0:
    return StepState(WAITING, "waiting for phone", "scan the code, then\nopen the page")
  if now - seen < PHONE_LIVE_S:
    return StepState(GOOD, "phone connected", "the TurnPilot page\nis open")
  return StepState(GOOD, "phone paired", "open the page again\nany time")


def area_names(status: dict) -> str:
  regions = status.get('regions') if isinstance(status, dict) else None
  names = [place_name(str(r.get('name') or r.get('id') or '')) for r in regions or [] if isinstance(r, dict)]
  return join_names([n for n in names if n]) if names else ''


def key_state(params) -> StepState:
  if not (params.get('TurnPilotMapKey') or '').strip():
    return StepState(WAITING, "waiting for the key", "paste it on the\nphone page")
  status = params.get(COVERAGE_STATUS)
  if not isinstance(status, dict):
    return StepState(BUSY, "checking the key", "asking the\nmap server")
  if status.get('ok'):
    names = area_names(status)
    return StepState(GOOD, "map key works", f"maps: {names}" if names else "the map server answered")
  code = status.get('code')
  if code == 'unauthorized':
    return StepState(ERROR, "key not accepted", "paste the key again\non the phone page")
  if code == 'unreachable':
    return StepState(ERROR, "map server offline", "check the comma's\ninternet, then retry")
  if code == 'not_configured':
    return StepState(ERROR, "map server not set", "paste the key again\non the phone page")
  return StepState(ERROR, "map server problem", "retry, or paste the\nkey again")


def engine_state(params, installed: bool) -> StepState:
  if installed:
    return StepState(GOOD, "map engine installed", "ready for routes")
  status = params.get(INSTALL_PARAM)
  status = status if isinstance(status, dict) else {}
  state = status.get('state')
  if state == 'installing':
    progress = status.get('progress')
    pct = f" {progress * 100:.0f}%" if isinstance(progress, (int, float)) else ""
    return StepState(BUSY, f"installing{pct}", status.get('stage') or "downloading")
  if state == 'failed':
    return StepState(ERROR, "install failed", status.get('short') or "tap retry")
  if state == 'offline':
    return StepState(WAITING, "waiting for internet", "it installs by itself\nonce online")
  return StepState(BUSY, "starting install", "this takes a minute")


def step_good(step: str, params, now: float, installed: bool) -> bool:
  if step == 'intro':
    return (params.get(SETUP_STEP) or 0) >= 1 or bool(params.get_bool(SETUP_DONE))
  if step == 'phone':
    return phone_state(params, now).state == GOOD
  if step == 'key':
    return key_state(params).state == GOOD
  if step == 'engine':
    return engine_state(params, installed).state == GOOD
  return False


def first_unfinished(params, now: float, installed: bool) -> int:
  """Where setup opens: the first step whose state isn't good, so a reboot resumes there."""
  return next((i for i, step in enumerate(STEPS) if not step_good(step, params, now, installed)), len(STEPS) - 1)


def readiness_line(params, now: float, installed: bool) -> str:
  """What TurnPilot still needs and where to fix it, in one sentence, or '' when it is ready to drive.
  Only while TurnPilot is on: with it off nothing is missing."""
  if not params.get_bool('TurnPilotEnabled') or (params.get('TurnPilotValhallaTiles') or '').strip():
    return ''  # off, or a developer's local tiles (the sim): no map server, no setup
  key = key_state(params)
  if key.state == WAITING:
    return "Paste the map key on the TurnPilot phone page."
  if key.state == ERROR:
    return {"key not accepted": "The map server refused the key. Paste it again on the TurnPilot phone page.",
            "map server offline": "Can't reach the map server. Connect the comma to the internet."}.get(
      key.title, "The map server had a problem. Open settings, turnpilot, run setup again.")
  engine = engine_state(params, installed)
  if engine.state == ERROR:
    return "The map engine install failed. Open settings, turnpilot, and tap retry."
  if engine.state != GOOD:
    return "The map engine is still installing. Keep the comma online."
  if not params.get_bool(SETUP_DONE):
    return "Finish setup: open settings, turnpilot, run setup again."
  return ''
