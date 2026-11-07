"""The setup screens' car-signal words under TurnPilotCarSignalsForceOn (1 by default since 2026-11-06), without a window.
Until then the signals step said "Skip it, and the comma asks you to tap the stalk at each turn", the 'failed' card "car
signals are off" and the done screen told the driver to tap the stalk at every turn, all while the car signalled."""
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase


class BlockingParams(Params):
  def put(self, key, dat, block=True):
    super().put(key, dat, block=True)

  def put_bool(self, key, val, block=True):
    super().put_bool(key, val, block=True)


class TestSignalWords(OpenpilotTestCase):
  def test_the_done_screens_stalk_line_follows_force_on(self):
    from openpilot.selfdrive.ui.mici.layouts.turnpilot_setup import (DONE_LINES_MAX, FORCED_STALK_LINE, STALK_LINE, done_lines,
                                                                     stalk_line)
    params = BlockingParams()
    # a fresh install: the car signals with no check, and the stalk is only for a lamp that doesn't light
    assert stalk_line(params) == FORCED_STALK_LINE and FORCED_STALK_LINE in done_lines(params)
    assert FORCED_STALK_LINE.startswith("The car signals its own turns.") and "Signal Left" in FORCED_STALK_LINE
    assert STALK_LINE not in done_lines(params)
    assert len(done_lines(params)) == DONE_LINES_MAX
    params.put("TurnPilotCarSignals", "on")  # a passed, confirmed check: no stalk line
    assert stalk_line(params) is None
    params.put("TurnPilotCarSignals", "off")
    params.put_bool("TurnPilotCarSignalsForceOn", False)  # the driver signals every turn, prompted
    assert stalk_line(params) == STALK_LINE and STALK_LINE in done_lines(params)

  def test_the_signals_step_says_skip_leaves_the_car_signalling(self):
    from openpilot.selfdrive.ui.mici.layouts.turnpilot_setup import signals_sentences
    params = BlockingParams()
    forced = signals_sentences(params)
    assert len(forced) == 2 and "on every drive" in forced[0] and "Skip it, and the car still signals" in forced[1]
    assert not any("at each turn" in s for s in forced)
    params.put_bool("TurnPilotCarSignalsForceOn", False)
    off = signals_sentences(params)
    assert len(off) == 2 and off[1] == "Skip it, and the comma asks you to tap the stalk at each turn."

  def test_the_failed_card_says_the_car_still_signals_while_forced(self):
    from openpilot.selfdrive.ui.mici.layouts.turnpilot_setup import signals_failed
    params = BlockingParams()
    header, text = signals_failed(params)
    assert header == "check didn't pass" and "The car still signals at each turn" in text and "are off" not in text
    params.put_bool("TurnPilotCarSignalsForceOn", False)
    assert signals_failed(params) == ("car signals off",
                                      "The check didn't pass, so car signals are off. The comma asks you to tap the stalk.")

  def test_the_step_reads_its_words_from_the_params(self):
    """SignalsStep builds and refreshes its cards from signals_sentences and signals_failed, so a ForceOn change shows on the
    next poll; no fixed sentence is left in the class."""
    from openpilot.selfdrive.ui.mici.layouts import turnpilot_setup as ts
    source = Path(ts.__file__).read_text()
    step = source[source.index("class SignalsStep"):source.index("class DoneStep")]
    assert "signals_sentences(ui_state.params)" in step and "signals_failed(ui_state.params)[1]" in step
    assert "self._failed.set_value(failed_text)" in step and "signals_sentences(params)" in step
    assert "Skip it, and the comma asks you" not in step and "so car signals are off" not in step
    assert 'failed' not in ts.SignalsStep.HEADER
