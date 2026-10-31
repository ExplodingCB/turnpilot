"""A fresh install drives on Saturday's params with nothing set (SATURDAY_PARAMS.md's table; Kellen, 2026-10-31 10:25: "it
should start in ready to drive"). Chase installs from installer.comma.ai and never opens a shell, so every value the drive
needs is a params_keys.h default.

Params.get reads no default: a default reaches the device through manager_init, which writes every unset key's default at
each boot, before any process starts. These tests read the header, the library built from it (skipped where the built
library predates a key: run scons), and a fresh params directory after manager_init's loop."""
import inspect
import re
from pathlib import Path

import pytest

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.version import RELEASE_BRANCHES

HEADER = Path(__file__).parents[3] / 'common' / 'params_keys.h'

# key: (type, the default as params_keys.h writes it, as Params reads it once manager_init has written it)
SATURDAY = {
  'TurnPilotEnabled': ('BOOL', '1', True),
  'TurnPilotShadowMode': ('BOOL', '0', False),        # live from the first drive, no shadow drive
  'TurnPilotAlwaysDrive': ('BOOL', '1', True),
  'TurnPilotNoMoveS': ('FLOAT', '20.0', 20.),
  'TurnPilotStopSignGo': ('STRING', 'vision', 'vision'),
  'TurnPilotFarPeds': ('BOOL', '1', True),
  'TurnPilotAvoidStops': ('BOOL', '1', True),
  'TurnPilotCurbPedCap': ('BOOL', '1', True),
  'TurnPilotLeftRangeM': ('FLOAT', '0.0', 0.),        # only ever a measurement on this car
  'TurnPilotLeftFarRangeM': ('FLOAT', '0.0', 0.),
  'TurnPilotAutoLeft': ('STRING', 'auto', 'auto'),
  'TurnPilotGoOverride': ('BOOL', '1', True),
  'TurnPilotCameraLights': ('STRING', 'shadow', 'shadow'),
  'TurnPilotCameraStopSigns': ('STRING', 'shadow', 'shadow'),
  'TurnPilotFarCrop': ('BOOL', '1', True),
  'TurnPilotNativeDetect': ('STRING', 'full', 'full'),  # the whole frame natively in the far crop's slot (35.7 ms step)
  'TurnPilotLanePaintActive': ('BOOL', '1', True),
  'TurnPilotYellowDecel': ('FLOAT', '2.0', 2.),
  'TurnPilotStopLaunch': ('FLOAT', '0.5', 0.5),
  'TurnPilotPostGoProfile': ('STRING', 'graduated', 'graduated'),
  'TurnPilotLeftGoProfile': ('STRING', 'left', 'left'),
  'TurnPilotCarSignals': ('STRING', 'off', 'off'),    # on only after the parked lamp check
  'TurnPilotCarSignalsVinless': ('BOOL', '0', False),
  # the stock toggles the drive needs, on in this fork with no settings screen visited
  'NavigationEnabled': ('BOOL', '1', True),
  'AlphaLongitudinalEnabled': ('BOOL', '1', True),
  'DisengageOnAccelerator': ('BOOL', '1', True),
  'DisableUpdates': ('BOOL', '0', False),             # on: a fix pushed to the turnpilot branch reaches the car (11:50)
}
# no default: routes come from the map server, not local tiles, and the map key is the one secret, never in this public repo
UNSET = ('TurnPilotValhallaTiles', 'TurnPilotMapKey')


def header_line(key):
  lines = [ln.split('//')[0].strip() for ln in HEADER.read_text().splitlines() if ln.strip().startswith(f'{{"{key}",')]
  assert len(lines) == 1, key
  return lines[0]


@pytest.mark.parametrize("key", SATURDAY)
def test_the_header_defaults_each_saturday_value(key):
  kind, text, _ = SATURDAY[key]
  assert re.fullmatch(rf'\{{"{key}", \{{PERSISTENT( \| DEVELOPMENT_ONLY)?, {kind}, "{re.escape(text)}"\}}\}},', header_line(key))


@pytest.mark.parametrize("key", UNSET)
def test_no_default_for_the_tiles_override_or_the_map_key(key):
  assert re.fullmatch(rf'\{{"{key}", \{{PERSISTENT( \| DONT_LOG)?, STRING\}}\}},', header_line(key))


def fresh(tmp_path, keys):
  params = Params(str(tmp_path))
  for key in keys:
    try:
      params.check_key(key)
    except UnknownKeyName:
      pytest.skip(f"this build's libparams predates {key}: run scons")
  return params


def test_the_library_holds_the_saturday_defaults(tmp_path):
  params = fresh(tmp_path, SATURDAY)
  for key, (kind, _, value) in SATURDAY.items():
    assert params.get_type(key).name == kind, key
    got = params.get_default_value(key)
    assert got == value and type(got) is type(value), key
  for key in UNSET:
    assert params.get_default_value(key) is None, key


def test_a_fresh_device_reads_the_saturday_values_after_manager_init(tmp_path):
  """manager_init's own loop, run over a fresh params directory: every reader (get_bool, get) then sees Saturday's value."""
  from openpilot.system.manager import manager
  assert "params.get_default_value(k)" in inspect.getsource(manager.manager_init)
  params = fresh(tmp_path, SATURDAY)
  assert all(params.get(key) is None for key in SATURDAY)  # nothing written yet: Params.get reads no default
  for k in params.all_keys():  # manager_init's "set unset params to their default value"
    default_value = params.get_default_value(k)
    if default_value is not None and params.get(k) is None:
      params.put(k, default_value, block=True)
  for key, (kind, _, value) in SATURDAY.items():
    assert params.get(key) == value, key
    if kind == 'BOOL':
      assert params.get_bool(key) is value, key
  assert all(params.get(key) is None for key in UNSET)


def test_alpha_longitudinal_stays_on_on_this_branch():
  """AlphaLongitudinalEnabled is DEVELOPMENT_ONLY: manager_init clears it on a release channel, and the settings screen removes
  it there. The branch the installer clones is not one, so its default stands (the Tesla interface offers alpha longitudinal
  on every car, so selfdrived's cleanup and the toggle's CarParams gate keep it too)."""
  assert 'turnpilot' not in RELEASE_BRANCHES  # installer.comma.ai/kaileh57/turnpilot
