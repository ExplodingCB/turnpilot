import pyray as rl

from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.turnpilot.maps.coverage import CoverageCache
from openpilot.selfdrive.turnpilot.web.pairing import pairing_url
from openpilot.selfdrive.turnpilot.web.store import last_gps_position
from openpilot.selfdrive.ui.lib.turnpilot import mapped_area_text
from openpilot.selfdrive.turnpilot import setup_state as ss
from openpilot.selfdrive.turnpilot.maps.paths import wall_time
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigParamControl, BigToggle, GreyBigButton
from openpilot.selfdrive.ui.mici.widgets.qr import QR
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller

URL_REFRESH_INTERVAL = 5  # seconds; the address changes when the device joins the phone's hotspot
STATUS_REFRESH_INTERVAL = 1  # seconds between reads of the map engine's install state

# settings > turnpilot's switches: (title, param, description). Each writes its param as the stock toggles do.
TOGGLES = (
  ("watch only", "TurnPilotShadowMode", "TurnPilot shows what it would do, but you drive. Off: it drives the route."),
  ("always drive", "TurnPilotAlwaysDrive", "Keep driving through what TurnPilot isn't sure of, and hand back only when something is wrong."),
  ("far pedestrians", "TurnPilotFarPeds", "Also slow for people far ahead, seen in the zoomed camera."),
)
STOP_SIGN_PARAM = "TurnPilotStopSignGo"
STOP_SIGN_DESCRIPTION = "On: at a stop sign, TurnPilot looks both ways with the cameras and goes when clear. Off: it stops and you go."


def stop_sign_checked(params) -> bool:
  return (params.get(STOP_SIGN_PARAM) or "") == "vision"


def stop_sign_value(checked: bool) -> str:
  """The TurnPilotStopSignGo value the toggle writes: vision or off, the two modes a driver picks between."""
  return "vision" if checked else "off"


def engine_text(params, installed: bool) -> str:
  state = ss.engine_state(params, installed)
  return f"{state.title}: {state.detail}".replace("\n", " ") if state.state != ss.GOOD else state.title


def shown_url(url: str) -> str:
  """The URL for the page, or 'no network' when pairing_url fell back to loopback, which no phone can reach."""
  return "no network" if url.startswith("http://127.") else url


class PairPhoneScroller(NavScroller):
  """The QR code and URL that pair a phone with the TurnPilot web app."""

  def __init__(self):
    super().__init__()
    self._url = pairing_url(ui_state.params)
    self._last_refresh = rl.get_time()
    self._qr = QR(self._url)
    self._url_button = GreyBigButton("", shown_url(self._url))
    self._coverage = CoverageCache()
    self._area_button = GreyBigButton("mapped area", self._area_text())
    self._scroller.scroll_indicator_start_after = self._qr
    self._scroller.add_widgets([
      self._qr,
      GreyBigButton("pair phone", "scan with the\nphone camera",
                    gui_app.texture("icons_mici/settings/device/phone.png", 85, 64)),
      self._url_button,
      GreyBigButton("", "The phone must be on the same network, such as its own hotspot."),
      self._area_button,
    ])

  def _area_text(self) -> str:
    """Which of TurnPilot's mapped areas the car is in, from the live fix onroad or the last saved one."""
    source = latest_gps_source(ui_state.sm)
    position = (ui_state.sm[source].latitude, ui_state.sm[source].longitude) if source else last_gps_position(ui_state.params)
    return mapped_area_text(self._coverage, position, (ui_state.params.get("TurnPilotValhallaTiles") or "").strip())

  def _update_state(self):
    super()._update_state()
    if rl.get_time() - self._last_refresh < URL_REFRESH_INTERVAL:
      return
    self._last_refresh = rl.get_time()
    self._area_button.set_value(self._area_text())
    url = pairing_url(ui_state.params)
    if url != self._url:
      self._url = url
      self._url_button.set_value(shown_url(url))
      self._qr._url = url
      if self._qr._texture and self._qr._texture.id != 0:
        rl.unload_texture(self._qr._texture)
      self._qr._texture = self._qr._generate_qr_code()

  def _update_layout_rects(self):
    super()._update_layout_rects()
    self._qr.set_rect(rl.Rectangle(self._qr.rect.x, self._qr.rect.y, self._rect.height, self._rect.height))


if __name__ == "__main__":
  gui_app.init_window("pair phone")
  page = PairPhoneScroller()
  gui_app.push_widget(page)
  try:
    for _ in gui_app.render():
      pass
  finally:
    del page


class TurnPilotSettingsLayout(NavScroller):
  """settings > turnpilot: the phone, the driving switches, the map engine and run setup again."""

  def __init__(self):
    super().__init__()
    from openpilot.selfdrive.turnpilot.valhalla_install import installed
    self._installed = installed
    self._pair_btn = BigButton("pair phone", icon=gui_app.texture("icons_mici/settings/device/phone.png", 85, 64),
                               description="Pair a phone to set TurnPilot destinations and add the map key.")
    self._pair_btn.set_click_callback(lambda: gui_app.push_widget(PairPhoneScroller()))
    self._toggles = [(param, BigParamControl(title, param, description=description)) for title, param, description in TOGGLES]
    self._stop_sign = BigToggle("stop-sign go", initial_state=stop_sign_checked(ui_state.params), toggle_callback=self._on_stop_sign,
                                description=STOP_SIGN_DESCRIPTION)
    self._engine_btn = BigButton("map engine", engine_text(ui_state.params, installed()),
                                 description="The map engine plans each drive's route on the comma. It installs itself while online.")
    self._engine_btn.set_click_callback(self._on_engine)
    self._setup_btn = BigButton("run setup again", icon=gui_app.texture("icons_mici/setup/green_info.png", 64, 64),
                                description="Go through TurnPilot setup again: the phone, the map key and the map engine.")
    self._setup_btn.set_click_callback(self._on_setup)
    self._last_refresh = -1e9

    toggles = [t for _, t in self._toggles]
    for item in (*toggles, self._stop_sign):
      item.set_enabled(lambda: not ui_state.engaged)
    self._scroller.add_widgets([self._pair_btn, toggles[0], toggles[1], self._stop_sign, toggles[2], self._engine_btn, self._setup_btn])

  def show_event(self):
    super().show_event()
    for param, toggle in self._toggles:
      toggle.set_checked(ui_state.params.get_bool(param))
    self._stop_sign.set_checked(stop_sign_checked(ui_state.params))
    self._last_refresh = -1e9

  def _update_state(self):
    super()._update_state()
    if rl.get_time() - self._last_refresh >= STATUS_REFRESH_INTERVAL:
      self._last_refresh = rl.get_time()
      self._engine_btn.set_value(engine_text(ui_state.params, self._installed()))

  def _on_stop_sign(self, checked: bool):
    ui_state.params.put(STOP_SIGN_PARAM, stop_sign_value(checked), block=True)

  def _on_engine(self):
    if ss.engine_state(ui_state.params, self._installed()).state == ss.ERROR:
      ui_state.params.put_bool(ss.RETRY_PARAM, True)
      ui_state.params.put(ss.INSTALL_PARAM, {'state': 'installing', 'progress': 0., 'stage': 'starting', 'at': wall_time()})

  def _on_setup(self):
    from openpilot.selfdrive.ui.mici.layouts.turnpilot_setup import setup_window
    setup_window().open(restart=True)
