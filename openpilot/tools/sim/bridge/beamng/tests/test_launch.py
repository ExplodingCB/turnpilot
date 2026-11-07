"""launch_openpilot.sh's BLOCK: logmessaged runs with loggerd, so a run's rlog carries cloudlog's records as logMessage
(TurnPilot's reasons: turnpilot.go_not_certified, turnpilot.stalled_in_turn, ...). Upstream's sim blocks both because it
doesn't log; this one keeps loggerd, and blocked logmessaged anyway, so the reasons never reached the rlogs (BeamNG smoke
2026-10-29: S14's start approved for 0.1 s and withdrawn, with no word why)."""
import os
import subprocess
import sys

import pytest

from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch_openpilot.sh')


def block(extra: str = '') -> list[str]:
  """The processes launch_openpilot.sh blocks with BLOCK=extra: its lines before it touches anything (the exports),
  sourced in a clean shell."""
  with open(LAUNCH) as f:
    head = f.read().split('# msgq needs the prefix directory')[0]
  out = subprocess.run(['bash', '-c', head + '\necho "$BLOCK"'], capture_output=True, text=True, check=True,
                       env={'PATH': os.environ.get('PATH', '/usr/bin:/bin'), 'BLOCK': extra}).stdout.strip()
  return [p for p in out.split(',') if p]


def test_logmessaged_runs_with_loggerd():
  procs = block()
  assert 'loggerd' not in procs and 'logmessaged' not in procs
  assert {'camerad', 'encoderd', 'micd', 'manage_athenad'} <= set(procs)


@pytest.mark.parametrize('extra', ['loggerd', 'ui,loggerd', 'loggerd,ui'])
def test_a_block_without_loggerd_drops_logmessaged_with_it(extra):
  # upstream's reason for blocking it: no rlog to carry what it publishes
  assert 'logmessaged' in block(extra)


def test_other_blocks_leave_it_running():
  assert 'logmessaged' not in block('turnpilotd')
  assert 'logmessaged' not in block('ui')


def param_setup() -> str:
  """launch_openpilot.sh's python -c param block, as bash hands it to python."""
  with open(LAUNCH) as f:
    text = f.read()
  body = text.split('python3 -c "', 1)[1].split('\n"\n', 1)[0]
  return body.replace('\\"', '"')


@pytest.mark.parametrize('signals, mode, force_on', [('1', 'on', True), ('0', 'off', False)])
def test_the_car_signal_params_match_the_bridge(tmp_path, signals, mode, force_on):
  """TURNPILOT_CAR_SIGNALS=0 (with the bridge's --no-car-signals) writes TurnPilotCarSignals off and
  TurnPilotCarSignalsForceOn 0, as bridge.car_signals_at_start does: until 2026-11-06 it wrote the mode alone, and under
  ForceOn's default, 1, card signalled while the bridge ran no body. The default writes on, the sim record and ForceOn 1."""
  root = os.path.abspath(os.path.join(os.path.dirname(LAUNCH), *[os.pardir] * 5))
  with OpenpilotPrefix() as prefix:
    params = Params()
    params.put('TurnPilotCarSignals', 'check' if signals == '0' else 'off', block=True)  # whatever an earlier run left
    params.put_bool('TurnPilotCarSignalsForceOn', signals == '0', block=True)  # landed before the launcher's own writes
    env = {**os.environ, 'OPENPILOT_PREFIX': prefix.prefix, 'TURNPILOT_CAR_SIGNALS': signals, 'TURNPILOT_TILES': str(tmp_path),
           'PYTHONPATH': root + os.pathsep + os.environ.get('PYTHONPATH', '')}
    out = subprocess.run([sys.executable, '-c', param_setup()], capture_output=True, text=True, env=env, cwd=root, timeout=60)
    assert out.returncode == 0, out.stderr
    assert params.get('TurnPilotCarSignals') == mode and params.get_bool('TurnPilotCarSignalsForceOn') is force_on
    assert f"TurnPilotCarSignals: {mode}, TurnPilotCarSignalsForceOn: {int(force_on)}" in out.stdout
    assert (params.get('TurnPilotCarSignalsVerified') is not None) == (signals == '1')
