"""TurnPilot setup: the guided screens after the stock terms and training, until TurnPilot is ready to drive.

Built like the stock onboarding (onboarding.py): each step is a horizontal Scroller of GreyBigButton cards ending in a
BigPillButton, as TrainingGuideAttentionNotice and NetworkSetupPage are. A step whose state isn't good yet shows a
disabled "waiting..." pill, as the stock network page does; once it is good the pill turns into a green "next" and grows,
one tap away. Errors say what to do on the status card and add a "retry" pill. "set up later" (a slide to confirm, like
the stock decline) closes it until the next boot, and the home screen's Offroad_TurnPilotSetup alert says what is missing.

What each step waits for is in selfdrive/turnpilot/setup_state.py. It opens at the first unfinished step, so a reboot
resumes there; TurnPilotSetupDone marks it finished, and settings > turnpilot > run setup again opens it at the start.
"""
import threading
from collections.abc import Callable

import pyray as rl

from openpilot.selfdrive.turnpilot import setup_state as ss
from openpilot.selfdrive.turnpilot.maps.defaults import ensure_map_defaults
from openpilot.selfdrive.turnpilot.maps.paths import wall_time
from openpilot.selfdrive.turnpilot.valhalla_install import installed as engine_installed
from openpilot.selfdrive.turnpilot.web.pairing import pairing_url
from openpilot.selfdrive.ui.mici.layouts.settings.turnpilot import shown_url
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationCircleButton
from openpilot.selfdrive.ui.mici.widgets.qr import QR
from openpilot.selfdrive.ui.ui_state import device, ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.mici_setup import BigPillButton, GreyBigButton
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.scroller import Scroller

POLL_S = 0.5  # s between param reads; the states change on the phone's and the installer's time, not per frame
URL_REFRESH_S = 5.  # the address changes when the comma joins the phone's hotspot

ICON = {
  'intro': ("icons_mici/setup/green_info.png", 64, 64),
  'phone': ("icons_mici/settings/device/phone.png", 85, 64),
  'key': ("icons_mici/settings/developer/ssh.png", 56, 64),
  'engine': ("icons_mici/settings/software.png", 64, 64),
  'done': ("icons_mici/offroad_alerts/green_wheel.png", 64, 64),
  'error': ("icons_mici/setup/warning.png", 64, 58),
}


def icon(name: str) -> rl.Texture:
  path, w, h = ICON[name]
  return gui_app.texture(path, w, h)


def setup_needed(params) -> bool:
  """Whether setup opens by itself at boot: not finished, and not a developer's local tiles (the sim), which need no
  map server and no phone."""
  return not params.get_bool(ss.SETUP_DONE) and not (params.get('TurnPilotValhallaTiles') or '').strip()


def done_lines(params) -> list[str]:
  """The last screen's sentences, from the params as they are now."""
  live = "watch only: it shows what it would do, and you drive" if params.get_bool('TurnPilotShadowMode') else \
         "live: it drives the route"
  lines = ["1. Drive straight above 15 mph for a few minutes. The camera calibrates.",
           "2. Engage with the cruise stalk.",
           "3. Enter a destination on the phone.",
           f"TurnPilot starts {live}."]
  if (params.get('TurnPilotCarSignals') or 'off') != 'on':
    lines.append("When the comma chimes \"Signal Left\" or \"Signal Right\", tap the stalk that way.")
  return lines


class SetupStep(Scroller):
  """One step: a header card, one sentence, the live status, and the pill that moves on."""

  STEP = ''

  def __init__(self, title: str, sentences: list[str], next_callback: Callable[[], None], later_callback: Callable[[], None] | None,
               waiting_text: str = "waiting...", next_text: str = "next"):
    super().__init__()
    self._next_callback = next_callback
    self._header = GreyBigButton(title, "scroll to continue", icon(self.STEP))
    self._sentences = [GreyBigButton("", s) for s in sentences]
    self._status = GreyBigButton("checking", "...")  # a title from the start, so the value sits at the bottom
    self._status.set_visible(False)
    self._retry = BigPillButton("retry")
    self._retry.set_click_callback(self._on_retry)
    self._retry.set_visible(False)
    self._waiting = BigPillButton(waiting_text, disabled_background=True)
    self._waiting.set_click_callback(self._scroll_to_status)
    self._next = BigPillButton(next_text, green=True)
    self._next.set_click_callback(next_callback)
    self._next.set_visible(False)
    self._later = None
    if later_callback is not None:
      self._later = BigConfirmationCircleButton("set up\nlater", gui_app.texture("icons_mici/setup/cancel.png", 64, 64), later_callback,
                                                exit_on_confirm=False)
    self._last_poll = -1e9
    self._was_good: bool | None = None
    self._pending_grow = False
    self.state: ss.StepState | None = None

  def widgets(self) -> list[Widget]:
    return [self._header, *self._extra_widgets(), *self._sentences, self._status, self._retry, self._next, self._waiting,
            *([self._later] if self._later else [])]

  def build(self):
    self._scroller.add_widgets(self.widgets())

  def _extra_widgets(self) -> list[Widget]:
    return []

  def step_state(self) -> ss.StepState | None:
    """This step's state from the params, or None for a step with nothing to wait for."""
    return None

  def _on_retry(self):
    pass

  def show_event(self):
    super().show_event()
    self._last_poll = -1e9
    self._was_good = None
    self.refresh()

  def refresh(self):
    state = self.step_state()
    self.state = state
    good = state is None or state.state == ss.GOOD
    if state is not None:
      self._status.set_text(state.title)
      self._status.set_value(state.detail)
      self._status.set_icon(icon('error') if state.state == ss.ERROR else None)
      self._status.set_visible(True)
    self._retry.set_visible(state is not None and state.state == ss.ERROR)
    self._next.set_visible(good)
    self._waiting.set_visible(not good)
    if good and self._was_good is False:
      self._pending_grow = True  # it just went good: scroll to "next" and grow it, as the stock network page does
      self._scroll_to(self._next)
    self._was_good = good

  def _scroll_to(self, widget: Widget):
    self._scroller._layout()
    offset = (widget.rect.x + widget.rect.width / 2) - (self._rect.x + self._rect.width / 2)
    self._scroller.scroll_to(offset, smooth=True, block_interrupt=True)

  def _scroll_to_status(self):
    self._scroll_to(self._status)

  def _update_state(self):
    super()._update_state()
    now = rl.get_time()
    if now - self._last_poll >= POLL_S:
      self._last_poll = now
      self.refresh()
    if self._pending_grow and self._next.rect.x + self._next.rect.width < self._rect.x + self._rect.width + 50:
      self._pending_grow = False
      self._next.trigger_grow_animation()

  def _render(self, _):
    rl.draw_rectangle_rec(self._rect, rl.BLACK)
    super()._render(_)


class IntroStep(SetupStep):
  STEP = 'intro'

  def __init__(self, next_callback, later_callback):
    super().__init__("what is turnpilot?", [
      "TurnPilot drives your navigation route: the turns, the lane changes and the stops.",
      "Next, pair your phone and add the map key. Then it is ready to drive.",
    ], next_callback, later_callback)
    self.build()


class PhoneStep(SetupStep):
  STEP = 'phone'

  def __init__(self, next_callback, later_callback):
    self._url = pairing_url(ui_state.params)
    self._qr = QR(self._url)
    self._url_card = GreyBigButton("", shown_url(self._url))
    self._last_url = -1e9
    super().__init__("pair your phone", [
      "The phone must be on the comma's Wi-Fi, or the comma on the phone's hotspot.",
    ], next_callback, later_callback, waiting_text="waiting for\nphone...")
    self._header.set_value("scan the code with\nthe phone camera")
    self.build()

  def _extra_widgets(self):
    return [self._qr, self._url_card]

  def step_state(self):
    return ss.phone_state(ui_state.params, wall_time())

  def _update_state(self):
    super()._update_state()
    if rl.get_time() - self._last_url < URL_REFRESH_S:
      return
    self._last_url = rl.get_time()
    url = pairing_url(ui_state.params)
    if url != self._url:
      self._url = url
      self._url_card.set_value(shown_url(url))
      self._qr._url = url
      if self._qr._texture and self._qr._texture.id != 0:
        rl.unload_texture(self._qr._texture)
      self._qr._texture = self._qr._generate_qr_code()

  def _update_layout_rects(self):
    super()._update_layout_rects()
    self._qr.set_rect(rl.Rectangle(self._qr.rect.x, self._qr.rect.y, self._rect.height, self._rect.height))


class KeyStep(SetupStep):
  STEP = 'key'

  def __init__(self, next_callback, later_callback):
    super().__init__("add the map key", [
      "Open the TurnPilot page on your phone and paste the map key Kellen sent you.",
    ], next_callback, later_callback, waiting_text="waiting for\nthe key...")
    self._checking = False
    self.build()

  def step_state(self):
    if self._checking:
      return ss.StepState(ss.BUSY, "checking the key", "asking the\nmap server")
    return ss.key_state(ui_state.params)

  def _on_retry(self):
    """Ask the map server again with the saved key, off the UI thread; the answer lands in TurnPilotMapCoverageStatus."""
    if self._checking:
      return
    self._checking = True

    def check():
      from openpilot.selfdrive.turnpilot.maps import coverage
      try:
        coverage.refresh(ui_state.params)
      except Exception:
        pass  # refresh() records why in the status param, which the card shows
      finally:
        self._checking = False

    threading.Thread(target=check, daemon=True).start()


class EngineStep(SetupStep):
  STEP = 'engine'

  def __init__(self, next_callback, later_callback, installed: Callable[[], bool]):
    self._installed = installed
    super().__init__("map engine", [
      "TurnPilot installs its map engine by itself while the comma is online.",
    ], next_callback, later_callback, waiting_text="installing...")
    self.build()

  def step_state(self):
    return ss.engine_state(ui_state.params, self._installed())

  def _on_retry(self):
    ui_state.params.put_bool(ss.RETRY_PARAM, True)  # turnpilot_web's installer takes it within LOOP_S


class DoneStep(SetupStep):
  STEP = 'done'

  def __init__(self, done_callback):
    super().__init__("ready to drive", done_lines(ui_state.params), done_callback, None, next_text="done")
    self.build()

  def show_event(self):
    for card, line in zip(self._sentences, done_lines(ui_state.params), strict=False):
      card.set_value(line)
    super().show_event()


class TurnPilotSetupWindow(Widget):
  """The setup steps, one at a time, over everything, as OnboardingWindow is."""

  def __init__(self, installed: Callable[[], bool] = engine_installed):
    super().__init__()
    self._installed = installed
    self._below: object | None = None
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    self._steps: list[SetupStep] = [
      IntroStep(self._on_intro_next, self.later),
      PhoneStep(lambda: self._advance(2), self.later),
      KeyStep(lambda: self._advance(3), self.later),
      EngineStep(lambda: self._advance(4), self.later, installed),
      DoneStep(self._on_done),
    ]
    for step in self._steps:
      step.set_enabled(lambda: self.enabled)  # for nav stack
    self._current = 0

  @property
  def current(self) -> int:
    return self._current

  @property
  def step(self) -> SetupStep:
    return self._steps[self._current]

  def open(self, restart: bool = False):
    """Push this over the active widget, at the first unfinished step (or the start)."""
    params = ui_state.params
    if restart:
      params.put_bool(ss.SETUP_DONE, False, block=True)
      params.put(ss.SETUP_STEP, 0, block=True)
    self.go_to(ss.first_unfinished(params, wall_time(), self._installed()))
    if not gui_app.widget_in_stack(self):
      self._below = gui_app.get_active_widget()
      gui_app.push_widget(self)

  def go_to(self, index: int):
    self._current = max(0, min(index, len(self._steps) - 1))
    self.step.show_event()

  def _advance(self, index: int):
    params = ui_state.params
    params.put(ss.SETUP_STEP, max(index, params.get(ss.SETUP_STEP) or 0), block=True)
    # a step already good (the phone paired last time) is skipped over, so setup resumes where it is needed
    while index < len(self._steps) - 1 and ss.step_good(ss.STEPS[index], params, wall_time(), self._installed()):
      index += 1
    self.go_to(index)

  def _on_intro_next(self):
    params = ui_state.params
    # the phone page (turnpilot_web) and navd only run with these on
    params.put_bool('TurnPilotEnabled', True, block=True)
    params.put_bool('NavigationEnabled', True, block=True)
    ui_state.turnpilot_enabled = True
    ensure_map_defaults(params)
    self._advance(1)

  def _on_done(self):
    ui_state.params.put_bool(ss.SETUP_DONE, True, block=True)
    self.close()

  def later(self):
    self.close()

  def close(self):
    below = self._below
    self._below = None
    if below is not None and gui_app.widget_in_stack(below):
      gui_app.pop_widgets_to(below)

  def show_event(self):
    super().show_event()
    device.set_override_interactive_timeout(300)
    device.set_offroad_brightness(100)
    self.step.show_event()

  def hide_event(self):
    super().hide_event()
    device.set_override_interactive_timeout(None)
    device.set_offroad_brightness(None)

  def _render(self, _):
    rl.draw_rectangle_rec(self._rect, rl.BLACK)
    self.step.render(self._rect)


_window: TurnPilotSetupWindow | None = None


def setup_window() -> TurnPilotSetupWindow:
  global _window
  if _window is None:
    _window = TurnPilotSetupWindow()
  return _window
