"""chase_render's sidecar says which lights and whether the indicator it replayed, and whether any other lamp is the run's."""
import json

import pytest

pytest.importorskip('beamngpy')
from openpilot.tools.sim.bridge.beamng import chase_render as cr
from openpilot.tools.sim.bridge.beamng import chase_track as ct


@pytest.mark.parametrize("running,word", [(False, "green and held"), (True, "not"), (None, "doesn't say")])
def test_the_sidecar_says_what_was_replayed(tmp_path, running, word):
  run = {'rows': [{'wall': 1000., 'lamp': 'left'}], 'light_events': [[1000.1, 'trafficLight 62', 'red']]}
  replay = ct.Replay(ct.light_timeline(run), ct.lamp_timeline(run['rows']), lambda i, s: None, lambda side: None)
  doc = {} if running is None else {'signals_running': running}
  video = str(tmp_path / 'chase.mp4')
  cr.write_sidecar(video, 1000., 40, 0.16, 'test', cr.replayed_note(doc, replay))
  side = json.load(open(video + '.start.json'))
  assert side['lights_replayed'] == ['trafficLight 62'] and side['indicator_replayed'] and side['signals_running'] == running
  assert word in side['lights_note'] and side['frames'] == 40
