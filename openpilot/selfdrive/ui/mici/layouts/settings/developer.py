from collections.abc import Callable
import pyray as rl
from openpilot.common.time_helpers import system_time_valid
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigToggle, BigParamControl, BigCircleParamControl, GreyBigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialog, BigInputDialog, BigConfirmationCircleButton
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.mici.layouts.settings.turnpilot import PairPhoneScroller
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.widgets.ssh_key import SshKeyFetcher


class AlphaLongConfirmPage(NavScroller):
  def __init__(self, on_confirm: Callable[[], None]):
    super().__init__()

    accept = BigConfirmationCircleButton("enable alpha\nlongitudinal",
                                         gui_app.texture("icons_mici/setup/driver_monitoring/dm_check.png", 64, 64),
                                         lambda: self.dismiss(on_confirm))

    self._scroller.add_widgets([
      GreyBigButton("enabling alpha longitudinal", "scroll to continue",
                    gui_app.texture("icons_mici/setup/warning.png", 64, 64)),
      GreyBigButton("", "WARNING: alpha longitudinal control may disable Automatic Emergency Braking (AEB)"),
      GreyBigButton("", "On this car, openpilot defaults to the stock system's built-in ACC."),
      GreyBigButton("", "Enabling this will switch to openpilot longitudinal control."),
      accept,
    ])


class DeveloperLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._ssh_fetcher = SshKeyFetcher(ui_state.params)
    self._alpamayo_status_t = 0.0

    def github_username_callback(username: str):
      if username:
        self._ssh_keys_btn.set_value("Loading...")
        self._ssh_keys_btn.set_enabled(False)

        def on_response(error):
          self._ssh_keys_btn.set_enabled(True)
          if error is None:
            self._ssh_keys_btn.set_value(username)
          else:
            self._ssh_keys_btn.set_value("Not set")
            gui_app.push_widget(BigDialog("", error))

        self._ssh_fetcher.fetch(username, on_response)
      else:
        self._ssh_fetcher.clear()
        self._ssh_keys_btn.set_value("Not set")

    def ssh_keys_callback():
      github_username = ui_state.params.get("GithubUsername") or ""
      dlg = BigInputDialog("enter GitHub username...", github_username, minimum_length=0, confirm_callback=github_username_callback)
      if not system_time_valid():
        dlg = BigDialog("", "Please connect to Wi-Fi to fetch your key.")
        gui_app.push_widget(dlg)
        return
      gui_app.push_widget(dlg)

    txt_ssh = gui_app.texture("icons_mici/settings/developer/ssh.png", 56, 64)
    github_username = ui_state.params.get("GithubUsername") or ""
    self._ssh_keys_btn = BigButton("SSH keys", "Not set" if not github_username else github_username, icon=txt_ssh,
                                   description="Grant SSH access to all public keys in your GitHub settings. Only enter your own username.")
    self._ssh_keys_btn.set_click_callback(ssh_keys_callback)

    # adb, ssh, ssh keys, debug mode, joystick debug mode, longitudinal maneuver mode, ip address
    # ******** Main Scroller ********
    self._adb_toggle = BigCircleParamControl(gui_app.texture("icons_mici/adb_short.png", 82, 82), "AdbEnabled", icon_offset=(0, 12),
                                             description="Use Android Debug Bridge (ADB) over USB or the network.", title="enable ADB")
    self._ssh_toggle = BigCircleParamControl(gui_app.texture("icons_mici/ssh_short.png", 82, 82), "SshEnabled", icon_offset=(0, 12),
                                             description="Access the device remotely using your SSH keys.", title="enable SSH")
    self._joystick_toggle = BigToggle("joystick debug\nmode", initial_state=ui_state.params.get_bool("JoystickDebugMode"),
                                      toggle_callback=self._on_joystick_debug_mode, description="Control the car with a joystick for debugging.")
    self._long_maneuver_toggle = BigToggle("longitudinal maneuver mode", initial_state=ui_state.params.get_bool("LongitudinalManeuverMode"),
                                           toggle_callback=self._on_long_maneuver_mode,
                                           description="Run longitudinal maneuvers for testing gas and brake control.")
    self._lat_maneuver_toggle = BigToggle("lateral maneuver mode", initial_state=ui_state.params.get_bool("LateralManeuverMode"),
                                          toggle_callback=self._on_lat_maneuver_mode,
                                          description="Run lateral maneuvers for testing steering control.")
    self._alpha_long_toggle = BigToggle("alpha longitudinal", initial_state=ui_state.params.get_bool("AlphaLongitudinalEnabled"),
                                        toggle_callback=self._on_alpha_long_enabled,
                                        description="Use alpha openpilot longitudinal control instead of stock ACC. This may disable Automatic Emergency " +
                                                    "Braking (AEB).")
    self._navigation_toggle = BigParamControl("navigation (alpha)", "NavigationEnabled",
                                              description="Turn-by-turn navigation with a route map. Set a destination from comma connect.")
    self._turnpilot_toggle = BigParamControl("turnpilot", "TurnPilotEnabled", toggle_callback=self._on_turnpilot,
                                             description="Drive the navigation route: turns, lane changes for the route, " +
                                                         "and holds for pedestrians and oncoming traffic. Turns on navigation.")
    self._turnpilot_shadow_toggle = BigParamControl("turnpilot shadow mode", "TurnPilotShadowMode",
                                                    description="TurnPilot plans and shows what it would do, but doesn't act on it.")
    self._alpamayo_toggle = BigToggle("alpamayo steering", initial_state=self._alpamayo_on(),
                                      toggle_callback=self._on_alpamayo,
                                      description="Experimental: NVIDIA Alpamayo 2 Edge on the chestnut sets the steering target below " +
                                                  "20 mph. Stock steering takes over whenever its plan is late or disagrees. " +
                                                  "Turns TurnPilot off. Downloads 5.3 GB on Wi-Fi first.")
    self._pair_phone_btn = BigButton("pair phone", icon=gui_app.texture("icons_mici/settings/device/phone.png", 85, 64),
                                     description="Pair a phone to set TurnPilot destinations.")
    self._pair_phone_btn.set_click_callback(lambda: gui_app.push_widget(PairPhoneScroller()))
    self._pair_phone_btn.set_enabled(lambda: ui_state.turnpilot_enabled)
    self._debug_mode_toggle = BigParamControl("ui debug mode", "ShowDebugInfo",
                                              toggle_callback=lambda checked: (gui_app.set_show_touches(checked), gui_app.set_show_fps(checked)),
                                              description="Show touch locations and the UI frame rate.")

    self._scroller.add_widgets([
      self._adb_toggle,
      self._ssh_toggle,
      self._ssh_keys_btn,
      self._joystick_toggle,
      self._long_maneuver_toggle,
      self._lat_maneuver_toggle,
      self._alpha_long_toggle,
      self._debug_mode_toggle,
      self._navigation_toggle,
      self._turnpilot_toggle,
      self._turnpilot_shadow_toggle,
      self._pair_phone_btn,
      self._alpamayo_toggle,
    ])

    # Toggle lists
    self._refresh_toggles = (
      ("AdbEnabled", self._adb_toggle),
      ("SshEnabled", self._ssh_toggle),
      ("JoystickDebugMode", self._joystick_toggle),
      ("LongitudinalManeuverMode", self._long_maneuver_toggle),
      ("LateralManeuverMode", self._lat_maneuver_toggle),
      ("AlphaLongitudinalEnabled", self._alpha_long_toggle),
      ("ShowDebugInfo", self._debug_mode_toggle),
      ("NavigationEnabled", self._navigation_toggle),
      ("TurnPilotEnabled", self._turnpilot_toggle),
      ("TurnPilotShadowMode", self._turnpilot_shadow_toggle),
    )
    onroad_blocked_toggles = (self._adb_toggle, self._joystick_toggle)
    release_blocked_toggles = (self._joystick_toggle, self._long_maneuver_toggle, self._lat_maneuver_toggle, self._alpha_long_toggle, self._navigation_toggle,
                               self._turnpilot_toggle, self._turnpilot_shadow_toggle, self._pair_phone_btn, self._alpamayo_toggle)
    engaged_blocked_toggles = (self._long_maneuver_toggle, self._lat_maneuver_toggle, self._alpha_long_toggle,
                               self._turnpilot_toggle, self._turnpilot_shadow_toggle, self._alpamayo_toggle)

    # Hide non-release toggles on release builds
    for item in release_blocked_toggles:
      item.set_visible(not ui_state.is_release)

    # Disable toggles that require offroad
    for item in onroad_blocked_toggles:
      item.set_enabled(lambda: ui_state.is_offroad())

    # Disable toggles that require not engaged
    for item in engaged_blocked_toggles:
      item.set_enabled(lambda: not ui_state.engaged)

    # Set initial state
    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()
    self._ssh_fetcher.update()
    if self._alpamayo_toggle.is_visible and rl.get_time() - self._alpamayo_status_t > 1.0:
      self._alpamayo_status_t = rl.get_time()
      on = self._alpamayo_on()
      self._alpamayo_toggle.set_value((ui_state.params.get("AlpamayoStatus") or "on") if on else "")

  @staticmethod
  def _alpamayo_on() -> bool:
    from openpilot.selfdrive.alpamayo.gate import alpamayo_mode
    return alpamayo_mode(ui_state.params) == "active"

  def _on_alpamayo(self, state: bool):
    ui_state.params.put("AlpamayoMode", "active" if state else "off")
    if state:  # one model on the chestnut at a time
      ui_state.params.put_bool("TurnPilotEnabled", False, block=True)
      ui_state.turnpilot_enabled = False
      self._turnpilot_toggle.set_checked(False)

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # CP gating
    if ui_state.CP is not None:
      alpha_avail = ui_state.CP.alphaLongitudinalAvailable
      if not alpha_avail or ui_state.is_release:
        self._alpha_long_toggle.set_visible(False)
        ui_state.params.remove("AlphaLongitudinalEnabled")
      else:
        self._alpha_long_toggle.set_visible(True)

      long_man_enabled = ui_state.has_longitudinal_control and ui_state.is_offroad()
      self._long_maneuver_toggle.set_enabled(long_man_enabled)
      self._lat_maneuver_toggle.set_enabled(ui_state.is_offroad())
    else:
      self._long_maneuver_toggle.set_enabled(False)
      self._lat_maneuver_toggle.set_enabled(False)
      self._alpha_long_toggle.set_visible(False)

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
    self._alpamayo_toggle.set_checked(self._alpamayo_on())

  def _on_joystick_debug_mode(self, state: bool):
    ui_state.params.put_bool("JoystickDebugMode", state, block=True)
    ui_state.params.put_bool("LongitudinalManeuverMode", False, block=True)
    self._long_maneuver_toggle.set_checked(False)
    ui_state.params.put_bool("LateralManeuverMode", False, block=True)
    self._lat_maneuver_toggle.set_checked(False)

  def _on_long_maneuver_mode(self, state: bool):
    ui_state.params.put_bool("LongitudinalManeuverMode", state, block=True)
    ui_state.params.put_bool("JoystickDebugMode", False, block=True)
    self._joystick_toggle.set_checked(False)
    ui_state.params.put_bool("LateralManeuverMode", False, block=True)
    self._lat_maneuver_toggle.set_checked(False)
    restart_needed_callback()

  def _on_lat_maneuver_mode(self, state: bool):
    ui_state.params.put_bool("LateralManeuverMode", state, block=True)
    ui_state.params.put_bool("JoystickDebugMode", False, block=True)
    self._joystick_toggle.set_checked(False)
    ui_state.params.put_bool("LongitudinalManeuverMode", False, block=True)
    self._long_maneuver_toggle.set_checked(False)
    restart_needed_callback()

  def _on_turnpilot(self, state: bool):
    if state:
      ui_state.params.put("AlpamayoMode", "off")
      self._alpamayo_toggle.set_checked(False)
    # BigParamControl writes the param after this callback, so set the flag here and "pair phone" enables at once
    ui_state.turnpilot_enabled = state
    if not state:  # turnpilot_web, which keeps the readiness alert, stops with TurnPilot: nothing is missing now
      from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
      from openpilot.selfdrive.turnpilot.setup_state import ALERT
      set_offroad_alert(ALERT, False)
    # TurnPilot follows navd's route
    if state:
      ui_state.params.put_bool("NavigationEnabled", True, block=True)
      self._navigation_toggle.set_checked(True)

  def _on_alpha_long_enabled(self, state: bool):
    def do_toggle(_state: bool):
      ui_state.params.put_bool("AlphaLongitudinalEnabled", _state, block=True)
      restart_needed_callback()
      self._update_toggles()

    if state:
      # Don't show enabled state until confirm
      self._alpha_long_toggle.set_checked(False)
      gui_app.push_widget(AlphaLongConfirmPage(lambda: do_toggle(True)))
    else:
      do_toggle(False)
