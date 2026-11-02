#!/usr/bin/env python3
"""Screenshots of TurnPilot on the comma four (mici) UI, driven by a fake publisher in a fresh OPENPILOT_PREFIX.

  python -m openpilot.selfdrive.ui.tests.turnpilot_screenshots OUT_DIR

Publishes deviceState (started), pandaStates (ignition), carState, selfdriveState, driver monitoring (a face, so the
DM icon draws), navInstruction and turnpilotPlan, plus a synthetic map (mapsd) and road camera (camerad) over
VisionIPC, and saves one 536x240 PNG per scene. The nav view's scenes (nav_*) add a synthetic route (navRoute) near
latitude and longitude 0, a GPS fix on it and calibration; --nav renders only those.

To watch the real UI instead, run both in one prefix:
  OPENPILOT_PREFIX=tp python -m openpilot.selfdrive.ui.tests.turnpilot_screenshots --publish 4
  OPENPILOT_PREFIX=tp SCALE=1 python openpilot/selfdrive/ui/ui.py
"""
import argparse
import os
import time
from dataclasses import dataclass, field

os.environ.setdefault("SCALE", "1")  # the mici resolution, not the Mac's auto scale

import numpy as np
import pyray as rl

from openpilot.common.prefix import OpenpilotPrefix

FPS = 60
SETTLE_FRAMES = 90  # filters, fades and nav animations settle within 1.5 s
NAV_SETTLE_FRAMES = 150  # 2.5 s: a GPS fix counts for 2 s (latest_gps_source), so the GPS-lost scene needs the last one gone

ROUTE = {'maneuverPrimaryText': 'W Washington St', 'maneuverDistance': 180., 'maneuverType': 'turn', 'maneuverModifier': 'left',
         'distanceRemaining': 5200., 'timeRemaining': 610.}


@dataclass
class Scene:
  name: str
  plan: dict
  car: dict = field(default_factory=dict)
  nav: dict | None = field(default_factory=lambda: dict(ROUTE))
  params: dict = field(default_factory=dict)
  alert: tuple | None = None  # (text1, text2, size, status) from selfdriveState
  started: bool = True
  engaged: bool = True
  # the nav view's: navRoute's (longitude, latitude) points, a GPS fix (longitude, latitude, bearing, speed), extrinsicsCalibration
  # (status, percent). None publishes nothing, as before the nav view
  route: np.ndarray | None = None
  gps: tuple | None = None
  calibration: tuple | None = None


def onroad_scenes() -> list[Scene]:
  turning = {'state': 'turning', 'desire': 1, 'maneuverDistance': 12., 'maneuverModifier': 'left', 'routeValid': True}
  return [
    Scene('01_off', turning, nav=dict(ROUTE, maneuverDistance=12.), params={'TurnPilotEnabled': False}),
    Scene('02_following', {'state': 'following', 'routeValid': True}),
    Scene('03_turning_left', turning, nav=dict(ROUTE, maneuverDistance=12.)),
    Scene('04_keeping_right', {'state': 'turning', 'desire': 6, 'maneuverModifier': 'slight right', 'routeValid': True},
          nav=dict(ROUTE, maneuverPrimaryText='I-65 N', maneuverModifier='slight right', maneuverType='fork', maneuverDistance=40.)),
    Scene('05_signal_prompt_right', {'state': 'laneChange', 'signalPrompt': 'right', 'routeValid': True},
          nav=dict(ROUTE, maneuverModifier='right', maneuverPrimaryText='N Meridian St', maneuverDistance=420.)),
    Scene('06_lane_change_right_signaled', {'state': 'laneChange', 'signalPrompt': 'right', 'laneChange': 'right', 'routeValid': True},
          car={'rightBlinker': True}, nav=dict(ROUTE, maneuverModifier='right', maneuverPrimaryText='N Meridian St', maneuverDistance=380.)),
    Scene('07_holding_pedestrian', {'state': 'holding', 'reason': 'pedestrian in crosswalk', 'shouldStop': True, 'routeValid': True},
          car={'vEgo': 0., 'standstill': True}, nav=dict(ROUTE, maneuverModifier='right', maneuverDistance=8.)),
    Scene('08_holding_oncoming', {'state': 'holding', 'reason': 'waiting for oncoming', 'shouldStop': True, 'routeValid': True},
          car={'vEgo': 0., 'standstill': True}, nav=dict(ROUTE, maneuverDistance=6.)),
    Scene('09_holding_no_route', {'state': 'holding', 'reason': 'pedestrian in path', 'routeValid': False},
          car={'vEgo': 3.}, nav=None, params={'NavDestination': None}),
    Scene('10_arrived', {'state': 'arrived', 'routeValid': True},
          nav=dict(ROUTE, maneuverType='arrive', maneuverModifier='straight', maneuverPrimaryText='Your destination',
                   maneuverDistance=0., distanceRemaining=0., timeRemaining=0.)),
    Scene('11_shadow_turning', dict(turning, shadow=True), nav=dict(ROUTE, maneuverDistance=12.), params={'TurnPilotShadowMode': True}),
    Scene('12_shadow_signal_prompt_left', {'state': 'laneChange', 'signalPrompt': 'left', 'shadow': True, 'routeValid': True},
          params={'TurnPilotShadowMode': True}),
    Scene('13_metric_following', {'state': 'following', 'routeValid': True}, nav=dict(ROUTE, maneuverDistance=640.),
          params={'IsMetric': True}),
    Scene('14_openpilot_alert_wins', {'state': 'holding', 'reason': 'pedestrian in path', 'routeValid': True},
          alert=('Take Control', 'Steering Temporarily Unavailable', 'mid', 'userPrompt')),
    Scene('15_enabled_turnpilotd_down', {}, params={}),  # TurnPilotEnabled but no turnpilotPlan: same as upstream
    # the driver steering: no status (nothing is holding the car), except in shadow mode, which is for exactly this
    Scene('16_disengaged_holding', {'state': 'holding', 'reason': 'pedestrian in path', 'routeValid': True}, engaged=False),
    Scene('17_disengaged_shadow_turning', dict(turning, shadow=True), nav=dict(ROUTE, maneuverDistance=12.),
          params={'TurnPilotShadowMode': True}, engaged=False),
  ]


def synthetic_route(step_m: float = 10.) -> np.ndarray:
  """(longitude, latitude) of a made-up 3 km route out of (0, 0), in the sea: 600 m north, left 400 m, right 800 m, right 700 m,
  left 500 m to the destination. Never a real place."""
  from openpilot.selfdrive.navigation.route import METERS_PER_DEGREE
  corners = np.array([(0, 0), (0, 600), (-400, 600), (-400, 1400), (300, 1400), (300, 1900)], dtype=np.float64)
  legs = []
  for a, b in zip(corners[:-1], corners[1:], strict=True):
    length = np.linalg.norm(b - a)
    legs.append(a + (b - a) * (np.arange(0, length, step_m) / length)[:, None])
  meters = np.vstack(legs + [corners[-1:]])
  return meters / METERS_PER_DEGREE  # east, north at the equator: degrees of longitude and latitude alike


def nav_scenes() -> list[Scene]:
  """The nav view: the car 420 m up the synthetic route's first leg, 180 m before its left turn, 2580 m from the end."""
  from openpilot.selfdrive.navigation.route import METERS_PER_DEGREE
  route = synthetic_route()
  car = (0., 420. / METERS_PER_DEGREE, 0., 12.)
  nav = {'maneuverPrimaryText': 'Example Ave', 'maneuverDistance': 180., 'maneuverType': 'turn', 'maneuverModifier': 'left',
         'distanceRemaining': 2580., 'timeRemaining': 230.}
  params = {'NavDestination': {'latitude': 1900. / METERS_PER_DEGREE, 'longitude': 300. / METERS_PER_DEGREE, 'place_name': 'Example'}}
  calibrated = ('calibrated', 100)
  stop_ahead = {'kind': 'stop', 'action': 'approaching', 'distance': 95.}
  red_held = {'kind': 'signal', 'action': 'held', 'lightState': 'red', 'lightSource': 'camera', 'distance': 2.}

  def scene(name, plan, **kw):
    return Scene(name, plan, **({'nav': dict(nav), 'params': dict(params), 'route': route, 'gps': car, 'calibration': calibrated} | kw))
  return [
    scene('nav_01_following', {'state': 'following', 'routeValid': True, 'laneChangeOffer': 'left', 'laneChangeCountdown': 2.3,
                               'mapControl': stop_ahead}),
    scene('nav_02_red_light', {'state': 'holding', 'reason': 'Red light', 'shouldStop': True, 'routeValid': True, 'mapControl': red_held},
          car={'vEgo': 0., 'standstill': True}, gps=car[:3] + (0.,)),
    scene('nav_03_gps_lost_calibrating', {'state': 'approaching', 'routeValid': True, 'laneChangeWant': 1, 'laneChangeWait': 'blindspot'},
          gps=None, calibration=('uncalibrated', 40)),
    scene('nav_04_disengaged_turning', {'state': 'turning', 'desire': 1, 'maneuverModifier': 'left', 'routeValid': True},
          nav=dict(nav, maneuverDistance=12.), engaged=False),
    scene('nav_05_unavailable', {'state': 'unavailable', 'reason': 'detector stopped: take control'}),
    # navd's invalid instruction with no route (navInstruction stays alive 10 s after the last valid one, so one is sent)
    scene('nav_06_no_destination', {'state': 'noRoute'}, nav={'valid': False}, route=np.empty((0, 2)), params={'NavDestination': None}),
    scene('nav_07_arriving', {'state': 'following', 'routeValid': True},
          nav=dict(nav, maneuverType='arrive', maneuverModifier='straight', maneuverPrimaryText='Example', maneuverDistance=60.,
                   distanceRemaining=60., timeRemaining=8.), gps=(300. / METERS_PER_DEGREE, 1840. / METERS_PER_DEGREE, 0., 8.)),
  ]


def map_frame() -> np.ndarray:
  """A road grid and a route that turns left ahead of the ego, in mapsd's grayscale format."""
  from openpilot.selfdrive.navigation.renderer import EGO, ROAD_INTENSITY, ROUTE_INTENSITY, SIZE
  img = np.zeros((SIZE, SIZE), dtype=np.uint8)
  ex, ey = int(EGO[0]), int(EGO[1])
  for x in (ex - 90, ex, ex + 80):
    img[:, x - 3:x + 4] = ROAD_INTENSITY
  for y in (ey - 150, ey - 70, ey + 40):
    img[y - 3:y + 4, :] = ROAD_INTENSITY
  img[ey - 70:ey + 1, ex - 2:ex + 3] = ROUTE_INTENSITY
  img[ey - 72:ey - 67, ex - 90:ex + 1] = ROUTE_INTENSITY
  return img


def camera_frame(width: int, height: int) -> bytes:
  """NV12: light sky over a darker road, so the overlays are seen over something other than black."""
  y = np.interp(np.arange(height), [0, height * 0.45, height * 0.55, height], [200, 170, 90, 60]).astype(np.uint8)
  luma = np.repeat(y[:, None], width, axis=1)
  chroma = np.full((height // 2, width), 128, dtype=np.uint8)
  return np.concatenate([luma, chroma]).tobytes()


BASE_PARAMS = {'TurnPilotEnabled': True, 'TurnPilotShadowMode': False, 'NavigationEnabled': True, 'IsMetric': False,
               'NavDestination': {'latitude': 39.7684, 'longitude': -86.1581, 'place_name': 'Monument Circle'}}


class FakePublisher:
  """Everything the onroad UI reads for one scene, sent once per call."""

  def __init__(self):
    from msgq.visionipc import VisionIpcServer
    from openpilot.cereal import messaging
    from openpilot.cereal.visionipc import VisionStreamType
    from openpilot.common.params import Params
    from openpilot.selfdrive.navigation.renderer import SIZE

    self.params = Params()
    self.pm = messaging.PubMaster(['deviceState', 'pandaStates', 'carState', 'selfdriveState', 'driverStateV2', 'driverMonitoringState',
                                   'navInstruction', 'turnpilotPlan', 'navRoute', 'gpsLocationExternal', 'extrinsicsCalibration'])
    cam_w, cam_h = 1928, 1208
    self.camera = VisionIpcServer('camerad')
    self.camera.create_buffers(VisionStreamType.VISION_STREAM_NARROW_ROAD, 4, cam_w, cam_h)
    self.map = VisionIpcServer('mapsd')
    self.map.create_buffers_with_sizes(VisionStreamType.VISION_STREAM_MAP, 4, SIZE, SIZE, SIZE * SIZE, SIZE, SIZE * SIZE)
    self.camera.start_listener()
    self.map.start_listener()
    self.camera_bytes, self.map_bytes = camera_frame(cam_w, cam_h), map_frame().tobytes()
    self.frame_id = 0

  def apply_params(self, scene: Scene):
    for k, v in (BASE_PARAMS | scene.params).items():
      if v is None:
        self.params.remove(k)
      elif isinstance(v, bool):
        self.params.put_bool(k, v, block=True)
      else:
        self.params.put(k, v, block=True)

  def send(self, scene: Scene):
    from openpilot.cereal import log, messaging
    from openpilot.cereal.visionipc import VisionStreamType

    self.frame_id += 1
    ds = messaging.new_message('deviceState', valid=True)
    ds.deviceState.started = scene.started
    ps = messaging.new_message('pandaStates', 1, valid=True)
    ps.pandaStates[0].pandaType = log.PandaState.PandaType.dos
    ps.pandaStates[0].ignitionLine = scene.started
    cs = messaging.new_message('carState', valid=True)
    cs.carState.vEgo = 12.
    cs.carState.vCruiseCluster = 72.
    for k, v in scene.car.items():
      setattr(cs.carState, k, v)
    ss = messaging.new_message('selfdriveState', valid=True)
    if scene.started and scene.engaged:
      ss.selfdriveState.enabled = True
      ss.selfdriveState.active = True
      ss.selfdriveState.state = log.SelfdriveState.OpenpilotState.enabled
    if scene.alert is not None:
      ss.selfdriveState.alertText1, ss.selfdriveState.alertText2, ss.selfdriveState.alertSize, ss.selfdriveState.alertStatus = scene.alert
    dsv = messaging.new_message('driverStateV2', valid=True)
    dsv.driverStateV2.leftDriverData.faceProb = 1.
    dms = messaging.new_message('driverMonitoringState', valid=True)
    dms.driverMonitoringState.activePolicy = log.DriverMonitoringState.MonitoringPolicy.vision
    dms.driverMonitoringState.visionPolicyState.faceDetected = True
    dms.driverMonitoringState.visionPolicyState.awarenessPercent = 100.
    for service, msg in (('deviceState', ds), ('pandaStates', ps), ('carState', cs), ('selfdriveState', ss),
                         ('driverStateV2', dsv), ('driverMonitoringState', dms)):
      self.pm.send(service, msg)
    if scene.nav is not None:
      ni = messaging.new_message('navInstruction', valid=scene.nav.get('valid', True))
      for k, v in scene.nav.items():
        if k != 'valid':
          setattr(ni.navInstruction, k, v)
      self.pm.send('navInstruction', ni)
    if scene.route is not None:
      nr = messaging.new_message('navRoute', valid=True)
      nr.navRoute.coordinates = [{'longitude': lon, 'latitude': lat} for lon, lat in scene.route.tolist()]
      self.pm.send('navRoute', nr)
    if scene.gps is not None:
      gps = messaging.new_message('gpsLocationExternal', valid=True)
      g = gps.gpsLocationExternal
      g.longitude, g.latitude, g.bearingDeg, g.speed = scene.gps
      g.hasFix = True
      self.pm.send('gpsLocationExternal', gps)
    if scene.calibration is not None:
      cal = messaging.new_message('extrinsicsCalibration', valid=True)
      cal.extrinsicsCalibration.calStatus, cal.extrinsicsCalibration.calPerc = scene.calibration
      self.pm.send('extrinsicsCalibration', cal)
    if scene.plan:
      tp = messaging.new_message('turnpilotPlan', valid=True)
      tp.turnpilotPlan.accelCap = 10.
      tp.turnpilotPlan.speedCap = 100.
      for k, v in scene.plan.items():
        setattr(tp.turnpilotPlan, k, v)
      self.pm.send('turnpilotPlan', tp)
    now = time.monotonic_ns()
    self.camera.send(VisionStreamType.VISION_STREAM_NARROW_ROAD, self.camera_bytes, self.frame_id, now, now)
    self.map.send(VisionStreamType.VISION_STREAM_MAP, self.map_bytes, self.frame_id, now, now)


def publish(seconds_per_scene: float):
  """Cycles the onroad scenes forever, for the real UI (selfdrive/ui/ui.py) running in the same OPENPILOT_PREFIX."""
  from openpilot.common.params import Params
  from openpilot.common.version import terms_version, training_version
  params = Params()
  params.put("HasAcceptedTerms", terms_version, block=True)
  params.put("CompletedTrainingVersion", training_version, block=True)
  pub = FakePublisher()
  while True:
    for scene in onroad_scenes():
      print(f"{time.monotonic():.1f} {scene.name}", flush=True)
      pub.apply_params(scene)
      end = time.monotonic() + seconds_per_scene
      while time.monotonic() < end:
        pub.send(scene)
        time.sleep(1 / 20)


def screenshots(out_dir: str, nav_only: bool = False):
  from openpilot.common.qrcode import decode
  from openpilot.selfdrive.ui.ui_state import ui_state
  from openpilot.system.ui.lib.application import gui_app
  from openpilot.selfdrive.ui.mici.onroad.augmented_road_view import AugmentedRoadView
  from openpilot.selfdrive.ui.mici.layouts.settings.developer import DeveloperLayoutMici
  from openpilot.selfdrive.ui.mici.layouts.settings.turnpilot import PairPhoneScroller

  pub = FakePublisher()
  rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
  gui_app.init_window("turnpilot screenshots", fps=FPS)
  gui_app.push_widget(AugmentedRoadView(lambda: None))
  frames = gui_app.render()

  def settle(scene: Scene, n: int = SETTLE_FRAMES):
    pub.apply_params(scene)
    ui_state.update_params()
    for _ in range(n):
      pub.send(scene)
      ui_state.update()
      next(frames)  # renders the nav stack and stops before the frame is presented

  def shot(name: str) -> np.ndarray:
    """Saves the frame just drawn and returns it as grayscale."""
    rl.rl_draw_render_batch_active()
    image = rl.load_image_from_screen()
    path = os.path.join(out_dir, f"{name}.png")
    rl.export_image(image, path)
    rgba = np.frombuffer(rl.ffi.buffer(image.data, image.width * image.height * 4), dtype=np.uint8).reshape(image.height, image.width, 4)
    gray = rgba[..., :3].mean(axis=2).astype(np.uint8)
    rl.unload_image(image)
    print(path)
    return gray

  def nav_shots():
    """The nav view's scenes, then the swipe to it from the camera view caught halfway, as the main scroller moves the pages."""
    from openpilot.selfdrive.ui.mici.onroad.nav_view import NavView
    from openpilot.system.ui.widgets.scroller import Scroller
    gui_app.push_widget(NavView())
    for scene in nav_scenes():
      settle(scene, NAV_SETTLE_FRAMES)
      shot(scene.name)
    pages = Scroller(snap_items=False, spacing=0, pad=0, scroll_indicator=False, edge_shadows=False)
    for page in (NavView(), AugmentedRoadView(lambda: None)):
      page.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
      pages._scroller.add_widget(page)
    gui_app.push_widget(pages)
    pages._scroller.scroll_panel.set_offset(-gui_app.width * 0.6)
    settle(nav_scenes()[0], NAV_SETTLE_FRAMES)
    shot('nav_08_swipe_from_camera')

  if nav_only:
    nav_shots()
    gui_app.close()
    return

  for scene in onroad_scenes():
    settle(scene)
    shot(scene.name)

  # offroad settings: the developer page scrolled to the TurnPilot items, then the pairing page
  offroad = Scene('offroad', {}, nav=None, started=False)
  settle(offroad)
  developer = DeveloperLayoutMici()
  gui_app.push_widget(developer)
  settle(offroad)
  for name, item in (('18_settings_turnpilot', developer._turnpilot_toggle), ('19_settings_shadow', developer._turnpilot_shadow_toggle),
                     ('20_settings_pair_phone', developer._pair_phone_btn)):
    developer._scroller.scroll_to(item.rect.x - (gui_app.width - item.rect.width) / 2)
    settle(offroad, 30)
    shot(name)
  pair = PairPhoneScroller()
  gui_app.push_widget(pair)
  settle(offroad, 3 * FPS)  # the nav bar over the top of the code fades after 2 s
  # mici draws the code light-on-dark and relies on the black bezel for the quiet zone
  scanned = decode(255 - np.pad(shot('21_pair_phone_qr'), 20))
  print(f"QR on screen decodes to {scanned!r}, pairing URL {pair._url!r}: {'OK' if scanned == pair._url else 'MISMATCH'}")
  pair._scroller.scroll_to(pair._url_button.rect.x - (gui_app.width - pair._url_button.rect.width) / 2)
  settle(offroad, 30)
  shot('22_pair_phone_url')
  nav_shots()  # last, so the GPS and calibration it publishes change none of the shots above
  gui_app.close()


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('out_dir', nargs='?', help='where the PNGs go')
  parser.add_argument('--publish', type=float, metavar='SECONDS',
                      help='only publish, cycling the scenes every SECONDS, in the current OPENPILOT_PREFIX')
  parser.add_argument('--nav', action='store_true', help="only the nav view's scenes")
  args = parser.parse_args()
  if args.publish:
    # fake started/ignition must never reach a real device's namespace
    if not os.environ.get('OPENPILOT_PREFIX'):
      parser.error('--publish needs OPENPILOT_PREFIX, shared with the UI')
    with OpenpilotPrefix(os.environ['OPENPILOT_PREFIX'], clean_dirs_on_exit=False):
      publish(args.publish)
  elif args.out_dir:
    os.makedirs(args.out_dir, exist_ok=True)
    with OpenpilotPrefix():
      screenshots(os.path.abspath(args.out_dir), nav_only=args.nav)
  else:
    parser.error('give OUT_DIR or --publish')


if __name__ == "__main__":
  main()
