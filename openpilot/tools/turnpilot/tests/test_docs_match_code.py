"""The docs the car is installed and driven by (docs/turnpilot: INSTALL.md for the install, SHADOW_CHECKLIST.md for each
drive) against the code they describe: the params' defaults as common/params_keys.h writes them, preflight's checks, and
the times and words the car says. Review 2026-11-06, the night's fixes in: INSTALL.md still said the car's own turn
signal "is off by default, and it stays off on a car until a lamp check passes", a day after TurnPilotCarSignalsForceOn
(default 1, 7c37f664a) put it on for every drive with no check, and named no ForceOn at all; it read a preflight WARN as
"TurnPilot is live, not in shadow mode", the opposite of check_shadow; and SHADOW_CHECKLIST.md put the night's 667 light
reads at one turn and had the driver expect a take-control at "some" turns at a light, where no light governing the car's
lane was read in any colour all night (wf_analysis: 32 reds, every one non-governing, no green)."""
import re
from pathlib import Path

from openpilot.selfdrive.turnpilot.map_controls import CAMERA_OFF_REASON, TURN_LIGHT_REASON, TURN_UNSEEN_S
from openpilot.selfdrive.turnpilot.preflight import INFO, WARN, check_shadow
from openpilot.selfdrive.turnpilot.unprotected_left import LIGHT_WAIT_REASON, LONG_WAIT_REASON, UNDECIDED_FAULTS, UNDECIDED_REASON

HEADER = Path(__file__).parents[3] / 'common' / 'params_keys.h'
DOCS = Path(__file__).parents[4] / 'docs' / 'turnpilot'
ENTRY = re.compile(r'\{"(\w+)",\s*\{[^}]*?(?:,\s*"([^"]*)")?\}\}')


def defaults() -> dict[str, str | None]:
  """Every key params_keys.h declares, with its default as written (None: no default)."""
  return {m.group(1): m.group(2) for ln in HEADER.read_text().splitlines() if (m := ENTRY.search(ln.split('//')[0]))}


def section(doc: str, heading: str) -> str:
  """The text under a '## heading' up to the next '## ' heading."""
  text = (DOCS / doc).read_text()
  start = text.index(f"\n## {heading}\n")
  end = text.find("\n## ", start + 1)
  return text[start:end if end >= 0 else None]


def table(text: str) -> dict[str, str]:
  """A params table's rows: key -> its Value cell, backticks stripped."""
  return {m.group(1): m.group(2).strip().strip('`') for m in re.finditer(r'^\| `(\w+)` \| ([^|]*) \|', text, re.M)}


def test_install_names_every_car_signal_param_at_its_default():
  keys = {k: v for k, v in defaults().items() if k.startswith('TurnPilotCarSignals')}
  assert 'TurnPilotCarSignalsForceOn' in keys and keys['TurnPilotCarSignalsForceOn'] == '1'
  rows = table(section('INSTALL.md', 'Let the car signal its own turns'))
  assert set(keys) <= set(rows), sorted(set(keys) - set(rows))
  for k, v in keys.items():
    if v is not None and not rows[k].startswith(('written', 'set by')):
      assert rows[k] == v, (k, rows[k], v)


def test_install_says_the_car_signals_every_drive_while_force_on_defaults_on():
  assert defaults()['TurnPilotCarSignalsForceOn'] == '1'
  sec = section('INSTALL.md', 'Let the car signal its own turns')
  first = sec.strip().split('\n\n')[1]  # the heading, then the section's opening paragraph
  assert 'TurnPilotCarSignalsForceOn' in first and 'on every drive' in first, first
  assert 'off by default' not in first, first
  setup = section('INSTALL.md', 'Install like stock')  # the guided setup's car turn signals step
  step = next(ln for ln in setup.splitlines() if ln.startswith('5. **car turn signals**'))
  assert 'With car signals off, the comma chimes' not in step and 'TurnPilotCarSignalsForceOn' in step, step
  assert 'TurnPilotCarSignalsForceOn' in (DOCS / 'AGENTS.md').read_text()


def test_a_preflight_warn_on_shadow_mode_means_shadow_mode_is_on():
  assert check_shadow(True, True).status == WARN and check_shadow(True, False).status == INFO
  sec = section('INSTALL.md', 'Run preflight before every drive')
  assert 'WARN means TurnPilot is live' not in sec
  assert re.search(r'WARN[^.]*`shadow mode` line[^.]*shadow mode is on', sec), sec[:600]
  assert '`shadow mode` warns when it is on' in sec


def test_the_checklists_lights_are_the_codes():
  sec = section('SHADOW_CHECKLIST.md', 'Traffic lights')
  assert f"(`TurnPilotCameraLights`, `{defaults()['TurnPilotCameraLights']}` by default)" in sec
  # since 2026-11-07 a route turn's signal is a through signal's (map_controls): no stop for want of a green, so no
  # TURN_UNSEEN_S handback and its words (TURN_LIGHT_REASON) are never said; the page says what the turn now does
  assert TURN_LIGHT_REASON not in sec and f"{TURN_UNSEEN_S:.0f} seconds at the line" not in sec
  assert 'A route turn at a signal is driven as a through signal is' in sec and 'With no colour read the turn goes with the driving model' in sec
  # the 11-06 night: 667 reads over both drives, no green, nothing governing the car's lane read
  assert 'At one turn' not in sec and 'at some turns at a light' not in sec
  assert 'no green in 667 light reads' in sec and 'expect every turn at a light on the map to stop at its line' not in sec
  # the reader failing (2026-11-06): its line as the comma says it, once per failing spell, at most once a minute; and the
  # red map_controls refused past a line gone through on a go, an error the log check triages against the rider's notes
  assert f'reading "{CAMERA_OFF_REASON}"' in sec and 'Camera light reader off for this drive' not in sec, sec
  assert 'once per failing spell' in sec and 'no more than once a minute' in sec
  assert '`turnpilot.camera_red_refused`' in sec


def test_the_checklists_unprotected_lefts_are_the_codes():
  """SHADOW_CHECKLIST.md's unprotected lefts against unprotected_left (2026-11-07, Kellen: "no, no handbacks"): the page
  had the driver expect a take-control at every left past the camera's reach and at every left after dark, which the build
  no longer says. It names each wait's line as the car says it, the handbacks still raised (UNDECIDED_FAULTS) and no other."""
  sec = section('SHADOW_CHECKLIST.md', 'What to watch in shadow mode')
  for words in list(LONG_WAIT_REASON.values()) + [LIGHT_WAIT_REASON['red'], 'Signal LEFT Now']:
    assert f'`{words}`' in sec, words
  for why, words in UNDECIDED_REASON.items():
    assert (f'`{words}`' in sec) == (why in UNDECIDED_FAULTS), (why, words)
