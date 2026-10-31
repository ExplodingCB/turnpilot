"""TurnPilot setup's and settings > turnpilot's wiring to their params, without a window."""
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase


class BlockingParams(Params):
  def put(self, key, dat, block=True):
    super().put(key, dat, block=True)

  def put_bool(self, key, val, block=True):
    super().put_bool(key, val, block=True)


class TestSettingsWiring(OpenpilotTestCase):
  def test_each_toggle_writes_a_known_bool_param(self):
    from openpilot.selfdrive.ui.mici.layouts.settings.turnpilot import TOGGLES
    params = Params()
    assert [p for _, p, _ in TOGGLES] == ["TurnPilotShadowMode", "TurnPilotAlwaysDrive", "TurnPilotFarPeds"]
    for title, param, description in TOGGLES:
      assert title == title.lower() and description.endswith(".")
      params.put_bool(param, True, block=True)  # a BOOL key: anything else raises
      assert params.get_bool(param)

  def test_stop_sign_go_is_vision_or_off(self):
    from openpilot.selfdrive.ui.mici.layouts.settings.turnpilot import stop_sign_checked, stop_sign_value
    params = BlockingParams()
    assert stop_sign_value(True) == "vision" and stop_sign_value(False) == "off"
    for mode, checked in (("vision", True), ("off", False), ("auto", False), ("shadow", False)):
      params.put("TurnPilotStopSignGo", mode)
      assert stop_sign_checked(params) is checked, mode

  def test_engine_text(self):
    from openpilot.selfdrive.turnpilot.setup_state import INSTALL_PARAM
    from openpilot.selfdrive.ui.mici.layouts.settings.turnpilot import engine_text
    params = BlockingParams()
    assert engine_text(params, True) == "map engine installed"
    params.put(INSTALL_PARAM, {'state': 'failed', 'short': 'storage full', 'at': 0.})
    assert engine_text(params, False) == "install failed: storage full"


class TestSetupWiring(OpenpilotTestCase):
  def test_opens_until_done_and_never_on_local_tiles(self):
    from openpilot.selfdrive.ui.mici.layouts.turnpilot_setup import setup_needed
    params = BlockingParams()
    assert setup_needed(params)
    params.put_bool("TurnPilotSetupDone", True)
    assert not setup_needed(params)
    params.put_bool("TurnPilotSetupDone", False)
    params.put("TurnPilotValhallaTiles", "/data/turnpilot/valhalla/indiana")  # the sim, a developer's tiles
    assert not setup_needed(params)

  def test_done_screen_says_live_or_watch_only_and_the_stalk(self):
    from openpilot.selfdrive.ui.mici.layouts.turnpilot_setup import done_lines
    params = BlockingParams()
    lines = done_lines(params)
    # in the car the comma reaches the internet (each drive's map) and the phone only on the phone's hotspot, and the
    # page's address changes there (INSTALL.md, First drive): the done screen says both, with no document to read
    assert "phone's hotspot" in lines[0] and "settings > network" in lines[0]
    assert "scan the code again" in lines[1].lower() and "settings > turnpilot > pair phone" in lines[1]
    assert "15 mph" in lines[2] and "cruise stalk" in lines[3] and "destination on the phone" in lines[4]
    assert [ln[:3] for ln in lines[:5]] == ["1. ", "2. ", "3. ", "4. ", "5. "]
    assert lines[5] == "TurnPilot starts live: it drives the route."
    assert "Signal Left" in lines[6]  # car signals off: the car asks for the stalk
    params.put_bool("TurnPilotShadowMode", True)
    params.put("TurnPilotCarSignals", "on")
    lines = done_lines(params)
    assert lines[5].startswith("TurnPilot starts watch only") and len(lines) == 7
    from openpilot.selfdrive.ui.mici.layouts.turnpilot_setup import DONE_LINES_MAX
    params.put("TurnPilotCarSignals", "off")
    assert len(done_lines(params)) == DONE_LINES_MAX  # the done screen has a card for every line it can show

  def test_done_screen_and_settings_say_how_to_update(self):
    # DisableUpdates 0: a fix pushed to the turnpilot branch reaches the car through settings > software, applied at a boot
    from openpilot.selfdrive.turnpilot.setup_state import UPDATE_LINE
    from openpilot.selfdrive.ui.mici.layouts import turnpilot_setup
    from openpilot.selfdrive.ui.mici.layouts.settings import turnpilot
    assert "settings > software > check for update" in UPDATE_LINE and "next starts" in UPDATE_LINE
    params = BlockingParams()
    for signals in ("off", "on"):
      params.put("TurnPilotCarSignals", signals)
      assert turnpilot_setup.done_lines(params)[-1] == UPDATE_LINE
    assert "GreyBigButton(\"updates\", ss.UPDATE_LINE)" in Path(turnpilot.__file__).read_text()
    assert Params().get_default_value("DisableUpdates") is False

  def test_steps_match_the_state_module(self):
    from openpilot.selfdrive.turnpilot.setup_state import STEPS
    from openpilot.selfdrive.ui.mici.layouts import turnpilot_setup as ts
    classes = [ts.IntroStep, ts.PhoneStep, ts.KeyStep, ts.EngineStep, ts.SignalsStep, ts.DoneStep]
    assert [c.STEP for c in classes] == list(STEPS)
    assert all(c.STEP in ts.ICON for c in classes)


class TestScreenshots(OpenpilotTestCase):
  def test_the_screenshots_draw_a_fresh_install(self):
    """The device's manager_init writes every unset param's default before the UI starts, so a fresh install shows
    always drive, stop-sign go and far pedestrians on; the screenshots draw what Chase sees, not empty params."""
    import os
    from unittest import mock
    with mock.patch.dict(os.environ):  # the script sets SCALE for its own window; keep it out of this process
      from openpilot.selfdrive.ui.tests.turnpilot_setup_screenshots import fresh_install
    params = BlockingParams()
    assert not params.get_bool("TurnPilotAlwaysDrive")  # nothing written yet: Params.get reads no default
    fresh_install(params)
    for key in ("TurnPilotEnabled", "NavigationEnabled", "TurnPilotAlwaysDrive", "TurnPilotFarPeds", "AlphaLongitudinalEnabled"):
      assert params.get_bool(key), key
    assert params.get("TurnPilotStopSignGo") == "vision" and not params.get_bool("TurnPilotShadowMode")
    assert params.get("TurnPilotMapKey") is None and not params.get_bool("TurnPilotSetupDone")
