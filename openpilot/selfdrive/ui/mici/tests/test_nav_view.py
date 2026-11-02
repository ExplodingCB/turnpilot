"""The nav view's pure parts: its text (ui/lib/turnpilot_nav.py), which onroad page the swipe left the screen on, and the route map's
framing and caching (ui/lib/route_map.py). The routes are synthetic, near latitude and longitude 0."""
import numpy as np

from openpilot.cereal import log, messaging
from openpilot.selfdrive.navigation.route import METERS_PER_DEGREE
from openpilot.selfdrive.ui.lib import route_map
from openpilot.selfdrive.ui.lib.route_map import RouteMap, fit, rotated, simplify
from openpilot.selfdrive.ui.lib.turnpilot_nav import (INFO, MAX_NOTES, PLAN, WARN, OnroadPage, calibration_text, control_line,
                                                      lane_change_line, maneuver_lines, notes, remaining_text, resting_page,
                                                      route_status, state_lines)


def plan(**fields):
  msg = messaging.new_message('turnpilotPlan', valid=True)
  for k, v in fields.items():
    setattr(msg.turnpilotPlan, k, v)
  return msg.as_reader().turnpilotPlan


def instruction(**fields):
  msg = messaging.new_message('navInstruction', valid=True)
  for k, v in fields.items():
    setattr(msg.navInstruction, k, v)
  return msg.as_reader().navInstruction


def route(*corners, step=10.):
  """(longitude, latitude) every step m along straight legs between corners given in meters east and north of (0, 0)."""
  corners = np.asarray(corners, dtype=np.float64)
  legs = []
  for a, b in zip(corners[:-1], corners[1:], strict=True):
    length = np.linalg.norm(b - a)
    legs.append(a + (b - a) * (np.arange(0, length, step) / length)[:, None])
  return np.vstack(legs + [corners[-1:]]) / METERS_PER_DEGREE


def lonlat(east, north):
  return east / METERS_PER_DEGREE, north / METERS_PER_DEGREE


class TestStateLines:
  def test_always_a_title(self):
    assert state_lines(None, True) == ('TurnPilot not running', 'no plan from turnpilotd')
    assert state_lines(plan(state='following'), True) == ('Following route', '')
    assert state_lines(plan(state='approaching'), True) == ('Maneuver ahead', '')  # the phone's label wraps on the comma
    assert state_lines(plan(state='noRoute'), True) == ('No route', '')
    assert state_lines(plan(state='following', reason='lead car slowing'), True) == ('Following route', 'lead car slowing')

  def test_what_it_would_do_when_not_engaged(self):
    """The camera view's status line is blank disengaged; the nav view says what TurnPilot is thinking, and that it isn't acting."""
    assert state_lines(plan(state='turning', desire=log.Desire.turnLeft), False) == ('Turning left', 'openpilot not engaged')
    assert state_lines(plan(state='holding', reason='pedestrian in path'), False) == ('Holding', 'pedestrian in path, not engaged')
    assert state_lines(plan(state='turning', desire=log.Desire.turnLeft), True) == ('Turning left', '')
    # shadow mode is the driver steering on purpose, and unavailable isn't acting either way
    assert state_lines(plan(state='turning', desire=log.Desire.turnLeft, shadow=True), False) == ('Turning left', 'shadow')
    assert state_lines(plan(state='following', shadow=True), False) == ('Following route', 'shadow')
    assert state_lines(plan(state='unavailable', reason='detector stopped: take control'), False) == \
      ('TurnPilot unavailable', 'detector stopped: take control')

  def test_holding_at_a_stop_sign_or_light(self):
    stop = {'kind': 'stop', 'action': 'held'}
    assert state_lines(plan(state='holding', reason='Stopped at stop sign', mapControl=stop), True) == ('Stopped at stop sign', '')
    red = {'kind': 'signal', 'action': 'held', 'lightState': 'red', 'lightSource': 'camera'}
    assert state_lines(plan(state='holding', reason='Red light', mapControl=red), True) == ('Red light', '')
    # another hold in force with it is the detail
    assert state_lines(plan(state='holding', reason='pedestrian in crosswalk', mapControl=stop), True) == \
      ('Stopped at stop sign', 'pedestrian in crosswalk')

  def test_fault(self):
    assert state_lines(plan(state='disabled', fault='noLongitudinal'), True) == ('TurnPilot off', "openpilot doesn't control speed on this car")
    assert state_lines(plan(state='disabled', reason='why'), True) == ('TurnPilot off', 'why')


class TestNotes:
  def test_lane_change(self):
    assert lane_change_line(plan(laneChangeOffer='right', laneChangeCountdown=2.3)) == 'Lane change right in 3'
    assert lane_change_line(plan(laneChangeOffer='left', signalPrompt='left')) == 'Signal left to change lanes'
    assert lane_change_line(plan(laneChangeOffer='left', laneChange='left')) == 'Changing lanes left once clear'
    assert lane_change_line(plan(laneChangeAbort='blindspot')) == 'Lane change cancelled: car in blind spot'
    assert lane_change_line(plan(laneChangeWant=1, laneChangeWait='beforeWindow')) == 'Lane change left planned'
    assert lane_change_line(plan(laneChangeWant=-2, laneChangeWait='carInLane')) == '2 lane changes right planned: car in that lane'
    assert lane_change_line(plan()) == ''

  def test_map_control_ahead(self):
    stop = {'kind': 'stop', 'action': 'approaching', 'distance': 95.}
    assert control_line(plan(mapControl=stop).mapControl, False) == 'Stop sign in 310 ft'
    assert control_line(plan(mapControl=stop).mapControl, True) == 'Stop sign in 100 m'
    red = {'kind': 'signal', 'action': 'braking', 'lightState': 'red', 'distance': 40.}
    assert control_line(plan(mapControl=red).mapControl, False) == 'Red light in 130 ft: stopping'
    assert control_line(plan(mapControl={'kind': 'signal', 'distance': 300.}).mapControl, True) == 'Signal in 300 m'
    assert control_line(plan(mapControl={'kind': 'yield', 'action': 'released'}).mapControl, False) == 'Yield: going'
    # holding at it, or the stop-sign decider at it: the title says so
    for action in ('held', 'awaitingDriver', 'waitingCross', 'creeping', 'going', 'committed'):
      assert control_line(plan(mapControl={'kind': 'stop', 'action': action, 'distance': 1.}).mapControl, False) == ''
    assert control_line(plan().mapControl, False) == ''

  def test_calibration(self):
    assert calibration_text('uncalibrated', 40) == 'Calibrating 40%'
    assert calibration_text('recalibrating', 75) == 'Recalibrating 75%'
    assert calibration_text('invalid', 0) == 'Calibration invalid: check the mount'
    assert calibration_text('calibrated', 100) == ''

  def test_order_and_room(self):
    busy = plan(state='laneChange', laneChangeOffer='right', laneChangeCountdown=1., mapControl={'kind': 'stop', 'distance': 95.})
    assert notes(busy, gps_ok=True, routing=True, calibration='', metric=False) == \
      [('Lane change right in 1', PLAN), ('Stop sign in 310 ft', INFO)]
    # GPS and calibration first, and only MAX_NOTES of them
    lost = notes(busy, gps_ok=False, routing=True, calibration='Calibrating 40%', metric=False)
    assert lost == [('Holding route, GPS lost', WARN), ('Calibrating 40%', WARN)] and len(lost) == MAX_NOTES
    assert notes(None, gps_ok=False, routing=False, calibration='', metric=False) == [('No GPS fix', WARN)]
    assert notes(plan(state='approaching', tooFastToTurn=True), gps_ok=True, routing=True, calibration='', metric=False) == \
      [('Skipping the turn: too fast, rerouting', WARN)]
    assert notes(plan(state='following', laneChangeAbort='noSignal'), gps_ok=True, routing=True, calibration='', metric=False)[0][1] == WARN
    assert notes(plan(state='disabled', laneChangeWant=1), gps_ok=True, routing=True, calibration='', metric=False) == []


class TestManeuver:
  def test_maneuver_and_remaining(self):
    turn = instruction(maneuverPrimaryText='Example Ave', maneuverType='turn', maneuverModifier='left', maneuverDistance=180.,
                       distanceRemaining=2580., timeRemaining=230.)
    assert maneuver_lines(turn, False) == ('590 ft', 'Example Ave')
    assert maneuver_lines(turn, True) == ('180 m', 'Example Ave')
    assert remaining_text(turn, False) == '4 min / 1.6 mi'
    assert remaining_text(turn, True) == '4 min / 2.6 km'
    arrive = instruction(maneuverPrimaryText='Example', maneuverType='arrive', maneuverDistance=-3.)
    assert maneuver_lines(arrive, False) == ('0 ft', 'Arrive at Example')
    assert maneuver_lines(instruction(maneuverType='turn', maneuverModifier='right'), False)[1] == 'Turn right'

  def test_route_status(self):
    here = {'latitude': 0.01, 'longitude': 0.02}
    assert route_status(None, None) == ('No destination', 'set one from your phone')
    assert route_status(here, None) == ('Waiting for route', '')
    refused = {'state': 'error', 'short': 'Destination outside mapped area', 'destination': [0.02, 0.01]}
    assert route_status(here, refused) == ('No route', 'Destination outside mapped area')


class TestSwipe:
  """The swipe itself is the main scroller's; what the nav view adds is which onroad page the driver rests on, so main.py's scrolls
  back to onroad (the drive's start, leaving a standstill, the interactive timeout) don't swipe the nav view away."""

  def test_resting_page(self):
    pages = {OnroadPage.NAV: 536., OnroadPage.CAMERA: 1072.}
    assert resting_page(0., pages, True) is None  # on home
    assert resting_page(0., {OnroadPage.NAV: 0., OnroadPage.CAMERA: 536.}, True) == OnroadPage.NAV
    assert resting_page(0., {OnroadPage.NAV: -536., OnroadPage.CAMERA: 0.4}, True) == OnroadPage.CAMERA
    # mid-swipe, or a scroll_to passing the nav view on its way from the camera to home
    assert resting_page(0., {OnroadPage.NAV: 0., OnroadPage.CAMERA: 536.}, False) is None
    assert resting_page(0., {OnroadPage.NAV: 210., OnroadPage.CAMERA: 746.}, True) is None

  def test_choice_sticks(self):
    page = OnroadPage()
    assert page.page == OnroadPage.CAMERA  # upstream's onroad page until the driver swipes
    page.see(OnroadPage.NAV)
    for elsewhere in (None, None):  # home, settings, a swipe under way
      page.see(elsewhere)
    assert page.page == OnroadPage.NAV
    page.see(OnroadPage.CAMERA)
    assert page.page == OnroadPage.CAMERA


class TestRouteMap:
  ROUTE = route((0, 0), (0, 600), (-400, 600), (-400, 1400), (300, 1400), (300, 1900))

  def test_simplify(self):
    px = np.array([(0, 0), (5, 0.1), (10, 0), (10, 10), (10, 20)], dtype=np.float64)
    assert simplify(px, 0.5).tolist() == [0, 2, 4]  # the corner kept, the near-collinear points dropped
    # out along a street and back: the far end is kept though it lies on the chord's line
    out_back = np.array([(0, 0), (5, 0), (10, 0), (5, 0.01), (1, 0.01)], dtype=np.float64)
    assert 2 in simplify(out_back, 0.5)
    assert simplify(px[:2], 0.5).tolist() == [0, 1]

  def test_arrow_turns_with_bearing(self):
    tip = np.array([[0., -9.]])
    assert np.allclose(rotated(tip, 0.), [[0, -9]])
    assert np.allclose(rotated(tip, 90.), [[9, 0]])  # east is right
    assert np.allclose(rotated(tip, 180.), [[0, 9]])
    assert np.allclose(rotated(tip, 270.), [[-9, 0]])

  def test_frame_fits_route_north_up(self):
    rm = RouteMap(240, 240)
    rm.set_route(self.ROUTE)
    assert rm.update(None, 0.)
    px = rm.frame.to_px(rm.meters)
    pad = route_map.PAD_PX
    assert px.min() >= pad - 1e-6 and px.max() <= 240 - pad + 1e-6
    # north up, east right: the end (300 m east, 1900 m north) is up and right of the start
    start, end = px[0], px[-1]
    assert end[0] > start[0] and end[1] < start[1]
    assert rm.image.shape == (240, 240, 4) and rm.image[..., 3].max() == 255
    assert rm.image[0, 0, 3] == 0  # the rounded corner is transparent

  def test_rebuilt_only_on_change(self):
    rm = RouteMap(240, 240)
    rm.set_route(self.ROUTE)
    rm.update(None, 0.)
    version = rm.version
    for i, north in enumerate(range(0, 600, 20)):  # driving up the first leg at 10 fixes a second
      rm.update(lonlat(0, north), 0.1 * i)
    assert rm.version == version
    rm.set_route(self.ROUTE.copy())  # navd's resend every 10 s
    assert rm.version == version
    assert abs(rm.progress - 580.) < 1e-6

  def test_maneuver_point(self):
    rm = RouteMap(240, 240)
    rm.set_route(self.ROUTE)
    rm.update(lonlat(0, 420), 0.)
    assert abs(rm.progress - 420.) < 1e-6
    assert np.allclose(rm.point_at(rm.progress + 180.), rm.car_meters(lonlat(0, 600)), atol=1e-6)  # the left turn's corner
    assert np.allclose(rm.point_at(1e9), rm.meters[-1])

  def test_reframes_when_the_car_leaves(self):
    rm = RouteMap(240, 240)
    rm.set_route(self.ROUTE)
    rm.update(lonlat(0, 100), 0.)
    version = rm.version
    away = lonlat(-1500, 100)  # 1.5 km off the route
    assert rm.update(away, 0.1) and rm.version == version + 1
    assert rm.frame.inside(rm.frame.to_px(rm.car_meters(away))[0], route_map.EDGE_PX)

  def test_zooms_in_near_the_end(self):
    rm = RouteMap(240, 240)
    rm.set_route(self.ROUTE)
    rm.update(lonlat(0, 0), 0.)
    whole = rm.frame.scale
    for i, point in enumerate(self.ROUTE[::2]):  # the whole drive, 20 m a fix
      rm.update(point, 0.1 * i)
    assert rm.frame.scale > 2 * whole
    assert rm.frame.scale <= (240 - 2 * route_map.PAD_PX) / route_map.MIN_SPAN_M + 1e-9  # never past MIN_SPAN_M across

  def test_progress_found_again_after_a_gap(self):
    """Projection searches near the last progress; off screen (or without GPS) the car may have gone far beyond it."""
    rm = RouteMap(240, 240)
    rm.set_route(self.ROUTE)
    rm.update(lonlat(0, 50), 0.)
    rm.update(lonlat(-400, 1000), 0.5)  # 1350 m on, within STALE_S: the window can't reach it
    assert rm.progress < 200.
    rm.update(lonlat(-400, 1000), 0.5 + route_map.STALE_S + 0.1)
    assert abs(rm.progress - 1400.) < 1e-3

  def test_no_route(self):
    rm = RouteMap(240, 240)
    rm.set_route(self.ROUTE)
    rm.update(None, 0.)
    rm.set_route(np.empty((0, 2)))  # destination cleared: navd sends an empty route
    assert not rm.has_route and rm.image is None and rm.frame is None
    assert not rm.update(lonlat(0, 0), 1.)

  def test_short_route_not_blown_up(self):
    frame = fit(np.array([(0., 0.), (0., 20.)]), (0., 0.), 240, 240)
    assert frame.scale * route_map.MIN_SPAN_M <= 240 - 2 * route_map.PAD_PX + 1e-9
