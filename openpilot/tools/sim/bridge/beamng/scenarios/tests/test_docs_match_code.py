"""The sim's docs (beamng/README.md, scenarios/README.md, the catalog's comments) against the code they describe. Review
2026-11-06: since 5dcb363 the camera's light reader is addOnly by default (TurnPilotCameraLights), on the car and in the
sim, so a camera red stops the car; S13's row and comment and the README's light-source paragraph still said TurnPilot
adds nothing for a red "with no accepted light source, as on the car this week", and named "the camera's shadow reader".
S05's scoring rule (score.LINE_HANDBACKS) names the reach's whys it is met for (score.REACH_WHYS); since 7c37f664a
TurnPilotCarSignalsForceOn (default 1) puts the car's signal on whatever --no-car-signals writes."""
import re
from pathlib import Path

from openpilot.tools.sim.bridge.beamng.scenarios import score

BEAMNG = Path(__file__).parents[2]
HEADER = Path(__file__).parents[6] / 'common' / 'params_keys.h'
SIM_DOCS = (BEAMNG / 'README.md', BEAMNG / 'scenarios' / 'README.md', BEAMNG / 'scenarios' / 'catalog.py')


def default(key: str) -> str:
  (ln,) = [ln.split('//')[0] for ln in HEADER.read_text().splitlines() if ln.strip().startswith(f'{{"{key}",')]
  return re.search(r',\s*"([^"]*)"\}\}', ln).group(1)


def test_no_sim_doc_says_the_camera_reader_is_shadow():
  assert default('TurnPilotCameraLights') == 'addOnly'
  for doc in SIM_DOCS:
    text = ' '.join(doc.read_text().split())  # across the comments' line breaks
    for stale in ("shadow reader", "this week", "TurnPilot adds nothing for it"):
      assert stale not in text, (doc.name, stale)
  readme = ' '.join((BEAMNG / 'README.md').read_text().split())
  assert "The camera's reader is one, on the car and in the sim alike (`TurnPilotCameraLights` `addOnly`" in readme
  s13 = next(ln for ln in (BEAMNG / 'scenarios' / 'README.md').read_text().splitlines() if ln.startswith('| `red_straight` |'))
  assert '`TurnPilotCameraLights` `addOnly`' in s13 and "a red it misses is the driving model's alone" in s13, s13


def test_the_s05_rule_names_the_reach_whys_it_is_met_for():
  rule = next(ln for ln in (BEAMNG / 'scenarios' / 'README.md').read_text().splitlines()
              if '(`leftUndecidedTakeControl`, S05) at its line' in ln)
  for why in score.REACH_WHYS:
    assert f'`{why}`' in rule, why
  assert 'explains nothing' in rule and '`light`' in rule


def test_the_car_signal_section_names_force_on():
  assert default('TurnPilotCarSignalsForceOn') == '1'
  readme = (BEAMNG / 'README.md').read_text()
  sec = readme[readme.index("### The car's own turn signal"):readme.index("With it on:")]
  assert '`--no-car-signals`' in sec and "put_bool('TurnPilotCarSignalsForceOn', False)" in sec, sec
  # the bridge writes it itself (bridge.car_signals_at_start, 2026-11-06): no manual step for the driver's path
  assert '`bridge.car_signals_at_start`' in sec, sec
