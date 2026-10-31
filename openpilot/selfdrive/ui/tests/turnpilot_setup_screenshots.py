#!/usr/bin/env python3
"""Screenshots of TurnPilot setup on the comma four (mici) UI, with the real widgets, in a fresh OPENPILOT_PREFIX.

  python -m openpilot.selfdrive.ui.tests.turnpilot_setup_screenshots OUT_DIR

The params start as a fresh install's (fresh_install: every default written, as manager_init does at boot). Each setup
step in its waiting, good and error states (set through the params the steps read), settings > turnpilot,
the home screen's readiness alert, and the stock terms and toggles pages beside them for comparison. One 536x240 PNG per
scene, named in flow order. Nothing here talks to a network: the map key's states are the status param the web app's
refresh writes, and the map engine's are the installer's.
"""
import argparse
import os
import tempfile

os.environ.setdefault("SCALE", "1")  # the mici resolution, not the Mac's auto scale

import pyray as rl

from openpilot.common.prefix import OpenpilotPrefix

FPS = 60
SETTLE = 75  # frames: scroll and grow animations settle within about a second
NOW_REGIONS = [{'id': 'indianapolis', 'name': 'Indianapolis area'}, {'id': 'seattle', 'name': 'Seattle area'}]


def fresh_install(params):
  """A fresh install's params as the UI first reads them: manager_init's own loop writes every unset key's default before
  any process starts, so the switches show this fork's defaults (TurnPilot, always drive, stop-sign go vision, far
  pedestrians and the stock navigation and alpha longitudinal on), not empty params."""
  for k in params.all_keys():  # manager_init's "set unset params to their default value"
    default_value = params.get_default_value(k)
    if default_value is not None and params.get(k) is None:
      params.put(k, default_value, block=True)


def screenshots(out_dir: str):
  from openpilot.common.version import terms_version, training_version
  from openpilot.selfdrive.turnpilot import setup_state as ss
  from openpilot.selfdrive.turnpilot.maps.coverage import STATUS_PARAM
  from openpilot.selfdrive.turnpilot.maps.paths import wall_time
  from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
  from openpilot.selfdrive.ui.ui_state import ui_state
  from openpilot.system.ui.lib.application import gui_app

  params = ui_state.params
  params.put("HasAcceptedTerms", terms_version, block=True)
  params.put("CompletedTrainingVersion", training_version, block=True)
  fresh_install(params)

  rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
  gui_app.init_window("turnpilot setup screenshots", fps=FPS)

  from openpilot.selfdrive.ui.mici.layouts.home import MiciHomeLayout
  from openpilot.selfdrive.ui.mici.layouts.offroad_alerts import MiciOffroadAlerts
  from openpilot.selfdrive.ui.mici.layouts.onboarding import TermsPage
  from openpilot.selfdrive.ui.mici.layouts.settings.settings import SettingsLayout
  from openpilot.selfdrive.ui.mici.layouts.settings.toggles import TogglesLayoutMici
  from openpilot.selfdrive.ui.mici.layouts.settings.turnpilot import TurnPilotSettingsLayout
  from openpilot.selfdrive.ui.mici.layouts.turnpilot_setup import TurnPilotSetupWindow

  from openpilot.system.ui.widgets import Widget

  class Black(Widget):  # the bottom of the stack: each scene is drawn on black, as the device's screen is
    def _render(self, rect):
      rl.draw_rectangle_rec(rect, rl.BLACK)

  base = Black()
  base.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
  gui_app.push_widget(base)
  frames = gui_app.render()
  names: list[str] = []

  def settle(n: int = SETTLE):
    for _ in range(n):
      ui_state.update()
      next(frames)

  def shot(name: str):
    rl.rl_draw_render_batch_active()
    image = rl.load_image_from_screen()
    path = os.path.join(out_dir, f"{len(names) + 1:02d}_{name}.png")
    rl.export_image(image, path)
    rl.unload_image(image)
    names.append(os.path.basename(path))
    print(path, flush=True)

  def center(scroller_owner, widget):
    sc = scroller_owner._scroller
    sc._layout()
    sc.scroll_to((widget.rect.x + widget.rect.width / 2) - (scroller_owner.rect.x + scroller_owner.rect.width / 2))
    settle()

  def show(widget, n: int = SETTLE):
    gui_app.push_widget(widget)
    settle(n)

  def pop():
    gui_app.pop_widgets_to(base, instant=True)
    settle(5)

  # the stock terms page, for comparison
  terms = TermsPage(on_accept=lambda: None, on_decline=lambda: None)
  show(terms)
  shot("stock_terms")
  pop()

  # TurnPilot setup, step by step
  engine = {'installed': False}
  window = TurnPilotSetupWindow(installed=lambda: engine['installed'])
  show(window)
  window.go_to(0)
  settle()
  shot("setup_1_intro")
  center(window.step, window.step._sentences[0])
  shot("setup_1_intro_sentence")
  center(window.step, window.step._next)
  shot("setup_1_intro_next")

  params.put(ss.SETUP_STEP, 1, block=True)
  window.go_to(1)
  settle(3 * FPS)  # the QR texture and the nav bar fade
  shot("setup_2_phone_qr")
  phone = window.step
  center(phone, phone._header)
  shot("setup_2_phone_header")
  center(phone, phone._url_card)
  shot("setup_2_phone_url")
  center(phone, phone._waiting)
  shot("setup_2_phone_waiting")
  params.put(ss.PHONE_SEEN, wall_time(), block=True)
  settle(SETTLE * 2)
  shot("setup_2_phone_connected_next")
  center(phone, phone._status)
  shot("setup_2_phone_connected_status")

  window.go_to(2)
  settle()
  shot("setup_3_key_header")
  key = window.step
  center(key, key._sentences[0])
  shot("setup_3_key_sentence")
  center(key, key._status)
  shot("setup_3_key_waiting")
  params.put("TurnPilotMapKey", "x" * 32, block=True)
  params.put(STATUS_PARAM, {'ok': False, 'code': 'unauthorized', 'error': 'refused', 'at': wall_time()}, block=True)
  settle()
  center(key, key._status)
  shot("setup_3_key_refused")
  center(key, key._retry)
  shot("setup_3_key_refused_retry")
  params.put(STATUS_PARAM, {'ok': False, 'code': 'unreachable', 'error': 'offline', 'at': wall_time()}, block=True)
  settle()
  center(key, key._status)
  shot("setup_3_key_offline")
  params.put(STATUS_PARAM, {'ok': True, 'version': 'v1', 'regions': NOW_REGIONS, 'at': wall_time()}, block=True)
  settle()
  center(key, key._status)
  shot("setup_3_key_works")
  center(key, key._next)
  shot("setup_3_key_next")

  window.go_to(3)
  settle()
  eng = window.step
  shot("setup_4_engine_header")
  for name, status in (("installing", {'state': 'installing', 'progress': 0.42, 'stage': 'downloading'}),
                       ("offline", {'state': 'offline', 'why': 'no route', 'short': 'no internet'}),
                       ("failed", {'state': 'failed', 'why': 'less than 300 MB free', 'short': 'storage full'})):
    params.put(ss.INSTALL_PARAM, {**status, 'at': wall_time()}, block=True)
    settle()
    center(eng, eng._status)
    shot(f"setup_4_engine_{name}")
  center(eng, eng._retry)
  shot("setup_4_engine_failed_retry")
  engine['installed'] = True
  settle()
  center(eng, eng._status)
  shot("setup_4_engine_installed")
  center(eng, eng._later)
  shot("setup_4_set_up_later")

  # the optional turn signal check: skipped by default, or run in Park and answered on the phone
  window.go_to(4)
  settle()
  sig = window.step
  shot("setup_5_signals_header")
  for i, card in enumerate(sig._sentences):
    center(sig, card)
    shot(f"setup_5_signals_line{i + 1}")
  center(sig, sig._next)
  shot("setup_5_signals_skip")
  center(sig, sig._check_btn)
  shot("setup_5_signals_check_button")
  sig._start()
  settle()
  shot("setup_5_signals_checking_header")
  for i, card in enumerate(sig._steps):
    center(sig, card)
    shot(f"setup_5_signals_checking_step{i + 1}")
  center(sig, sig._status)
  shot("setup_5_signals_checking_waiting")
  params.put('TurnPilotCarSignalsCheck', {'pass': True, 'checkId': 'abcd1234', 'exterior': 'pending', 'when': 'now'}, block=True)
  settle()
  center(sig, sig._status)
  shot("setup_5_signals_checking_answer_on_phone")
  params.put('TurnPilotCarSignalsExterior', 'yes:abcd1234', block=True)
  settle()
  center(sig, sig._try)
  shot("setup_5_signals_try_controls")
  center(sig, sig._works)
  shot("setup_5_signals_they_work")
  center(sig, sig._broken)
  shot("setup_5_signals_something_wrong")
  sig._on_works()
  settle()
  center(sig, sig._cycle)
  shot("setup_5_signals_on_cycle_again")
  sig._fail()
  settle()
  center(sig, sig._failed)
  shot("setup_5_signals_failed_off")
  params.put('TurnPilotCarSignals', 'off', block=True)

  window.go_to(5)
  settle()
  done = window.step
  shot("setup_6_done_header")
  for i, card in enumerate(done._sentences):
    center(done, card)
    shot(f"setup_6_done_line{i + 1}")
  center(done, done._next)
  shot("setup_6_done_button")
  pop()

  # settings: the stock list with turnpilot first, then its page, then a stock settings page for comparison
  settings = SettingsLayout()
  show(settings)
  shot("settings_list")
  pop()
  tp = TurnPilotSettingsLayout()
  show(tp)
  for item in tp._scroller.items:
    center(tp, item)
    shot(f"settings_turnpilot_{item.text.replace(' ', '_').replace('-', '_')}")
  pop()
  toggles = TogglesLayoutMici()
  show(toggles)
  shot("stock_settings_toggles")
  center(toggles, toggles._scroller.items[2])
  shot("stock_settings_toggles_2")
  pop()

  # the home screen: the readiness alert in the stock alerts list, and the pill on home
  params.remove("TurnPilotMapKey")
  set_offroad_alert(ss.ALERT, True, ss.readiness_line(params, wall_time(), False))
  alerts = MiciOffroadAlerts()
  alerts.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
  pending = ({"UpdaterNewDescription": None} | {a.key: params.get(a.key) for a in alerts.sorted_alerts})
  alerts._refresh(pending)
  show(alerts)
  shot("home_alerts_not_ready")
  pop()
  home = MiciHomeLayout()
  home.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
  home.set_callbacks(alert_count_callback=alerts.active_alerts, alert_icon_callback=alerts.highest_severity_icon)
  show(home)
  shot("home_pill_not_ready")
  pop()
  set_offroad_alert(ss.ALERT, False)
  alerts._refresh({"UpdaterNewDescription": None} | {a.key: params.get(a.key) for a in alerts.sorted_alerts})
  show(alerts)
  shot("home_alerts_ready")
  pop()
  gui_app.close()
  return names


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('out_dir', help='where the PNGs go')
  args = parser.parse_args()
  os.makedirs(args.out_dir, exist_ok=True)
  with tempfile.TemporaryDirectory() as maps, OpenpilotPrefix():
    os.environ['TURNPILOT_MAPS_DIR'] = maps
    screenshots(os.path.abspath(args.out_dir))


if __name__ == "__main__":
  main()
