"""launch_openpilot.sh's BLOCK: logmessaged runs with loggerd, so a run's rlog carries cloudlog's records as logMessage
(TurnPilot's reasons: turnpilot.go_not_certified, turnpilot.stalled_in_turn, ...). Upstream's sim blocks both because it
doesn't log; this one keeps loggerd, and blocked logmessaged anyway, so the reasons never reached the rlogs (BeamNG smoke
2026-10-29: S14's start approved for 0.1 s and withdrawn, with no word why)."""
import os
import subprocess

import pytest

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
