"""TurnPilot's fast start: navd routes from LastGPSPosition before the first fix and checks that route against the fix,
navd and mapsd run offroad (plan the route and fetch its tiles, nothing more), and the GPS and nav daemons' process rules."""
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.navigation.mapsd import MapsD
from openpilot.common.params import ParamKeyFlag, Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.navigation.navd import DESTINATION_OFFROAD_S, HEADING_PARKED_S, RECOMPUTE_HOLD, REROUTE_COUNTER_MIN, RouteEngine
from openpilot.selfdrive.navigation.tests import test_navigation as nav_tests
from openpilot.selfdrive.turnpilot.maps.client import MapError
from openpilot.system.manager import process_config

METER = 1 / 111319.49079327358  # degrees at the equator


def saved_position(lon, lat) -> bytes:
  """LastGPSPosition as ubloxd saves it: a gpsLocationExternal event."""
  msg = messaging.new_message('gpsLocationExternal', valid=True)
  msg.gpsLocationExternal.hasFix, msg.gpsLocationExternal.longitude, msg.gpsLocationExternal.latitude = True, lon, lat
  return msg.to_bytes()


class FakeSM:
  """navd's and mapsd's SubMaster: a live fix at .fix ((lon, lat) or None) heading .bearing at .speed with .bearing_accuracy
  (standing, none reported: no heading, hold.py), deviceState.started from .started (None: never heard), and navRoute's
  coordinates from .route."""

  def __init__(self):
    self.fix, self.started, self.route, self.frame = None, None, None, 0
    self.bearing, self.speed, self.bearing_accuracy = 90., 0., 0.
    self.data, self.seen, self.valid, self.recv_time, self.logMonoTime, self.updated = {}, {}, {}, {}, {}, {}

  def update(self, timeout=0):
    self.frame += 1
    now = time.monotonic()
    for s in ('gpsLocation', 'gpsLocationExternal'):
      self.valid[s], self.recv_time[s], self.logMonoTime[s] = self.fix is not None, now, int(now * 1e9)
    if self.started is not None:
      self.data['deviceState'], self.seen['deviceState'] = SimpleNamespace(started=self.started), True
    self.updated['navRoute'] = self.route is not None

  def __getitem__(self, s):
    if s == 'deviceState':
      return self.data[s]
    if s == 'navRoute':
      return SimpleNamespace(coordinates=[SimpleNamespace(longitude=lon, latitude=lat) for lon, lat in self.route])
    lon, lat = self.fix or (0., 0.)
    return SimpleNamespace(hasFix=self.fix is not None, longitude=lon, latitude=lat, bearingDeg=self.bearing, speed=self.speed,
                           bearingAccuracyDeg=self.bearing_accuracy)

  def drive(self, bearing=90., speed=10.):
    """From here on the fix moves, heading bearing with a good bearing accuracy."""
    self.bearing, self.speed, self.bearing_accuracy = bearing, speed, 2.


class NavdCase(unittest.TestCase):
  def setUp(self):
    self.values = {'NavDestination': {'latitude': 0, 'longitude': 0.002}, 'LanguageSetting': 'main_en'}
    self.sm, self.pm = FakeSM(), MagicMock()

  def engine(self, seed=(0.0001, 0.)):
    if seed is not None:
      self.values['LastGPSPosition'] = saved_position(*seed)
    params = MagicMock()
    params.get.side_effect = self.values.get
    params.remove.side_effect = lambda key: self.values.pop(key, None)
    self.t = 0.
    self.nav = RouteEngine(self.sm, self.pm, params=params, router=nav_tests.StubRouter(nav_tests.TestRouteEngine.steps()), background=False,
                           monotonic=lambda: self.t)
    return self.nav

  def tick(self):
    self.sm.update()
    self.nav.update()
    return next(c.args[1] for c in reversed(self.pm.send.call_args_list) if c.args[0] == 'navInstruction')


class TestSeededStart(NavdCase):
  def test_the_route_starts_from_the_last_position_before_the_first_fix(self):
    engine = self.engine()
    self.assertFalse(self.tick().valid)  # never guidance from a guess
    self.assertEqual(engine.router.calls, [((0.0001, 0.), (0.002, 0), None, 'en')])  # no heading for a parked car
    self.assertIsNotNone(engine.route)
    for _ in range(3):
      self.assertFalse(self.tick().valid)
    self.sm.fix = (0.0002, 2 * METER)  # on the route, standing
    # TurnPilot (route start, 1d 3:39): the route made from the seed had no heading, so nothing goes out until the car's
    # direction is checked against it; the first good moving heading is the route's (east): kept, guidance at once
    self.assertFalse(self.tick().valid)
    self.sm.drive(bearing=91.)
    self.sm.fix = (0.0003, 2 * METER)
    self.assertTrue(self.tick().valid)
    self.assertEqual(len(engine.router.calls), 1)
    self.assertFalse(engine.heading_unverified)

  def test_a_first_fix_off_the_seeded_route_routes_again_at_once(self):
    engine = self.engine()
    self.tick()
    self.sm.fix = (0.0005, 100 * METER)  # the car was moved, or the seed is old
    self.tick()
    # no heading: this fix stands still, and a standing fix's bearing is stale (hold.py: route heading)
    self.assertEqual(engine.router.calls[1], ((0.0005, 100 * METER), (0.002, 0), None, 'en'))
    self.tick()
    self.assertEqual(len(engine.router.calls), 2)  # once

  def test_a_seeded_attempt_that_failed_is_tried_again_at_the_first_fix(self):
    engine = self.engine()
    engine.router.error = MapError('outside_coverage', "The car is outside TurnPilot's mapped area.")
    self.tick()
    self.assertIsNone(engine.route)
    engine.router.error = None
    self.sm.fix = (0.0001, 0.)
    self.tick()  # not after RECOMPUTE_HOLD
    self.assertEqual(len(engine.router.calls), 2)
    self.assertFalse(self.tick().valid)  # made standing with no heading: guidance once the car's direction is checked
    self.sm.drive()
    self.assertTrue(self.tick().valid)
    self.assertEqual(len(engine.router.calls), 2)

  def test_no_saved_position_waits_for_the_fix_as_before(self):
    engine = self.engine(seed=None)
    self.tick()
    self.assertEqual(engine.router.calls, [])
    self.sm.fix = (0.0001, 0.)
    self.tick()
    self.assertEqual(len(engine.router.calls), 1)

  def test_no_reroutes_from_the_unmoving_seed_until_the_first_fix(self):
    """1d 1:28-3:30: no fix for minutes out of a garage, and navd rerouted from the same seed every 11 s (seven routes, each
    with its controls and avoidance, none sent). One route from the seed; the first fix, off it, routes again at once."""
    engine = self.engine(seed=(0.0005, 100 * METER))  # the seed off the route it gets (100 m beside it)
    for _ in range(4 * RECOMPUTE_HOLD):
      self.assertFalse(self.tick().valid)
    self.assertEqual(len(engine.router.calls), 1)
    self.sm.fix = (0.0006, 100 * METER)
    self.tick()
    self.assertEqual(len(engine.router.calls), 2)  # reseed_due
    self.assertFalse(engine.seeded)

  def test_nothing_is_looked_up_ahead_of_the_seed(self):
    engine = self.engine(seed=(0.0005, 100 * METER))  # off the route it gets: invalid instructions, with no lookup
    for _ in range(3):
      self.tick()
    self.assertEqual(engine.router.ahead_calls, [])


class TestOffroad(NavdCase):
  def test_offroad_the_route_is_planned_and_kept(self):
    engine = self.engine(seed=None)
    self.sm.started, self.sm.fix = False, (0.0001, 0.)
    self.tick()
    self.assertEqual(len(engine.router.calls), 1)  # planned while parked
    self.sm.fix = (0.0005, 100 * METER)  # a parked fix's wander, or a car park beside the road
    for _ in range(RECOMPUTE_HOLD + REROUTE_COUNTER_MIN + 5):
      self.assertFalse(self.tick().valid)
    self.assertEqual(len(engine.router.calls), 1)  # no reroutes offroad
    self.assertEqual(engine.router.ahead_calls, [])  # nor lookups ahead: turnpilotd isn't running
    self.sm.started = True
    for _ in range(REROUTE_COUNTER_MIN + 1):
      self.tick()
    self.assertEqual(len(engine.router.calls), 2)  # in drive, off the route: rerouted as always
    self.assertTrue(engine.router.ahead_calls)

  def test_a_route_planned_parked_stays_unverified_however_the_parked_fix_wanders(self):
    """Offroad there is no carState, so the hold takes every trusted fix: a parked receiver's wander along the street must
    not pass for the car driving the route's way (navd heading_due, by travel), nor a parked fix's speed noise for a heading.
    Onroad, the first good moving heading checks it."""
    engine = self.engine(seed=None)
    self.sm.started, self.sm.fix = False, (0.0001, 0.)
    self.tick()
    self.assertTrue(engine.heading_unverified)
    for k in range(1, 7):  # wandering 20 m east, the route's own way, and back
      self.sm.fix = (0.0001 + (k if k <= 4 else 8 - k) * 5. * METER, 0.)
      self.assertFalse(self.tick().valid)
    self.sm.drive(bearing=90., speed=4.)  # even "moving" with a good bearing accuracy: offroad nothing is decided
    self.tick()
    self.assertTrue(engine.heading_unverified)
    self.assertEqual(len(engine.router.calls), 1)
    self.sm.started = True  # in drive, pulling away east
    self.sm.fix = (0.0003, 0.)
    self.assertTrue(self.tick().valid)
    self.assertFalse(engine.heading_unverified)
    self.assertEqual(len(engine.router.calls), 1)

  def test_offroad_the_etas_do_not_drift(self):
    engine = self.engine(seed=None)
    self.sm.started, self.sm.fix = False, (0.0001, 0.)
    self.tick()
    engine.clock = lambda: 1000.
    engine.controls = [SimpleNamespace(id=1, route_m=150., eta_window=(0., 1.))]
    engine.check_eta_drift(0.)
    self.assertFalse(engine.avoid_rerun)  # the drive's ETAs start at departure
    self.sm.started = True
    self.sm.update()
    engine.check_eta_drift(0.)
    self.assertTrue(engine.avoid_rerun)

  def test_a_new_destination_offroad_is_routed_at_once(self):
    engine = self.engine(seed=None)
    self.sm.started, self.sm.fix = False, (0.0001, 0.)
    self.tick()
    self.values['NavDestination'] = {'latitude': 0, 'longitude': 0.0015}
    self.tick()
    self.assertEqual([c[1] for c in engine.router.calls], [(0.002, 0), (0.0015, 0)])


class TestParkedHeading(NavdCase):
  """Review of 11-06: navd keeps running offroad between drives, where there is no carState or deviceMotion, so the hold's
  last heading stood for the whole stop: the car's orientation in its space (nose-in, perpendicular to the street), and a
  route planned parked for the next trip went with it as a known heading, never checked, its guidance out as the car pulled
  away. Offroad navd routes with no heading, and HEADING_PARKED_S offroad drops the hold's."""
  NEXT = {'latitude': 0, 'longitude': 0.0015}

  def drive_then_park(self, offroad_s):
    """A short drive east (a good heading of 90), then parked offroad_s with the u-blox's standstill bearing (300, bacc 45)."""
    engine = self.engine(seed=None)
    self.sm.fix, self.sm.started = (0.0001, 0.), True
    self.sm.drive(bearing=90., speed=10.)
    for _ in range(3):
      self.t += 1.
      self.assertTrue(self.tick().valid)
    self.assertEqual(engine.hold.heading, 90.)
    self.sm.started = False
    self.sm.bearing, self.sm.speed, self.sm.bearing_accuracy = 300., 0., 45.
    self.tick()
    self.t += offroad_s
    self.tick()
    return engine

  def test_a_route_planned_parked_after_a_drive_has_no_heading_and_waits_for_the_cars_direction(self):
    engine = self.drive_then_park(HEADING_PARKED_S)
    self.assertIsNone(engine.hold.heading)  # dropped: where the car points in its space isn't where it will drive
    self.values['NavDestination'] = dict(self.NEXT)  # set on the phone for the next trip
    self.t += 60.
    self.tick()
    self.assertEqual(engine.router.calls[-1][1:3], ((0.0015, 0), None))
    self.assertTrue(engine.heading_unverified)
    self.sm.started = True  # the next drive, still in the space: nothing goes out
    for _ in range(3):
      self.t += 1.
      self.assertFalse(self.tick().valid)
    self.sm.drive(bearing=91.)  # pulled away along the route: its first good moving heading checks it
    self.t += 1.
    self.assertTrue(self.tick().valid)
    self.assertFalse(engine.heading_unverified)
    self.assertEqual([c[1] for c in engine.router.calls], [(0.002, 0), (0.0015, 0)])

  def test_a_destination_set_just_after_parking_is_routed_with_no_heading_too(self):
    engine = self.drive_then_park(1.)
    self.assertEqual(engine.hold.heading, 90.)  # not dropped yet (an onroad cycle is about a second offroad)...
    self.assertIsNone(engine.route_heading())  # ...but offroad nothing routes with it
    self.values['NavDestination'] = dict(self.NEXT)
    self.t += 1.
    self.tick()
    self.assertIsNone(engine.router.calls[-1][2])
    self.assertTrue(engine.heading_unverified)

  def test_an_onroad_cycle_keeps_the_heading(self):
    """modeld's big-model retry: started False for about a second with the car driving on: the heading stays, and the
    route with it."""
    engine = self.drive_then_park(0.)
    self.sm.started = True
    self.sm.drive(bearing=90., speed=10.)
    self.t += 1.
    self.assertTrue(self.tick().valid)
    self.assertEqual(engine.route_heading(), 90.)
    self.assertEqual(len(engine.router.calls), 1)
    self.assertFalse(engine.heading_unverified)

class TestMapsdOffroad(unittest.TestCase):
  def test_offroad_only_the_tiles_are_fetched(self):
    sm = FakeSM()
    sm.route, sm.fix = [(0., 0.), (0.001, 0.)], (0., 0.)
    mapsd = MapsD()
    mapsd.renderer = MagicMock()
    mapsd.renderer.render.return_value = (np.full((256, 256), 7, dtype=np.uint8), True)
    sm.update()
    image, timestamp = mapsd.update(sm, render=False)
    mapsd.renderer.prefetch.assert_called_once()
    mapsd.renderer.render.assert_not_called()
    self.assertFalse(image.any() or timestamp)
    sm.route = None  # the same route, not resent
    sm.update()
    image, timestamp = mapsd.update(sm)  # in drive: the tiles are there, the first frame renders
    mapsd.renderer.prefetch.assert_called_once()
    self.assertEqual((image[0, 0], timestamp), (7, sm.logMonoTime['gpsLocationExternal']))


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key, block=False):
    return bool(self.values.get(key, False))

  def put_bool(self, key, value, block=False):
    self.values[key] = value


class TestProcessRules(unittest.TestCase):
  def test_gps_runs_offroad_while_turnpilot_is_enabled(self):
    for ublox, rule, other in ((True, process_config.ublox, process_config.qcomgps), (False, process_config.qcomgps, process_config.ublox)):
      with patch.object(process_config, 'ublox_available', return_value=ublox):
        self.assertTrue(rule(False, FakeParams(TurnPilotEnabled=True), None))
        self.assertFalse(rule(False, FakeParams(TurnPilotEnabled=False), None))  # comma's: onroad only
        self.assertTrue(rule(True, FakeParams(TurnPilotEnabled=False), None))
        self.assertFalse(other(True, FakeParams(TurnPilotEnabled=True), None))  # one receiver's daemons, never both
        params = FakeParams(TurnPilotEnabled=True)
        process_config.ublox(False, params, None)
        self.assertEqual(params.get_bool('UbloxAvailable'), ublox)  # still kept up to date

  def test_navd_and_mapsd_run_offroad_while_turnpilot_is_enabled(self):
    nav = process_config.navigation
    self.assertTrue(nav(False, FakeParams(NavigationEnabled=True, TurnPilotEnabled=True), None))
    self.assertFalse(nav(False, FakeParams(NavigationEnabled=True, TurnPilotEnabled=False), None))
    self.assertTrue(nav(True, FakeParams(NavigationEnabled=True, TurnPilotEnabled=False), None))
    self.assertFalse(nav(True, FakeParams(NavigationEnabled=False, TurnPilotEnabled=True), None))
    procs = process_config.managed_processes
    self.assertIs(procs['navd'].should_run, nav)
    self.assertIs(procs['mapsd'].should_run, nav)
    self.assertFalse(procs['turnpilotd'].should_run(False, FakeParams(TurnPilotEnabled=True), None))  # still onroad only


class TestDestinationLifetime(NavdCase):
  """NavDestination was CLEAR_ON_OFFROAD_TRANSITION: any onroad cycle deleted the phone's destination, and modeld's big-model
  retry is one (on 9 downloaded routes, seconds into the drive, with the occupied Tesla staying onroad). It is
  CLEAR_ON_MANAGER_START only, and navd ends the trip DESTINATION_OFFROAD_S offroad after a drive."""
  DEST = {'latitude': 0, 'longitude': 0.002}

  def test_the_destination_survives_an_onroad_cycle_through_the_managers_clears(self):
    with OpenpilotPrefix():
      params = Params()
      params.put('NavDestination', dict(self.DEST), block=True)
      params.put_bool('LateralManeuverMode', True, block=True)  # still CLEAR_ON_OFFROAD_TRANSITION: shows the clear ran
      # manager.py's loop, started True -> False -> True (OnroadCycleRequested: started False for a second, hardwared.py)
      params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
      params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
      self.assertFalse(params.get_bool('LateralManeuverMode'))
      self.assertEqual(params.get('NavDestination'), self.DEST)
      params.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)  # manager_init, a reboot: still a fresh start
      self.assertIsNone(params.get('NavDestination'))

  def drive(self):
    engine = self.engine(seed=None)
    self.sm.fix, self.sm.started = (0.0001, 0.), True
    self.sm.drive()
    for _ in range(3):
      self.t += 1.
      self.assertTrue(self.tick().valid)
    return engine

  def test_navd_ends_the_trip_after_30_min_offroad_after_a_drive(self):
    engine = self.drive()
    self.sm.started = False  # parked
    start = self.t
    for t in (0., 15. * 60., DESTINATION_OFFROAD_S - 1.):  # the stop is still part of the trip
      self.t = start + t
      self.tick()
      self.assertEqual(self.values['NavDestination'], self.DEST)
    self.assertIsNotNone(engine.route)
    self.t = start + DESTINATION_OFFROAD_S
    self.tick()
    self.assertNotIn('NavDestination', self.values)
    self.assertIsNone(engine.route)  # and the route with it
    calls = len(engine.router.calls)
    for _ in range(3):
      self.t += 60.
      self.assertFalse(self.tick().valid)
    self.assertEqual(len(engine.router.calls), calls)

  def test_an_onroad_cycle_keeps_the_destination_and_restarts_nothing(self):
    engine = self.drive()
    self.sm.started = False  # modeld's retry: started False for about a second
    self.t += 1.
    self.tick()
    self.sm.started = True
    self.t += 1.
    self.assertTrue(self.tick().valid)
    self.t += 2 * DESTINATION_OFFROAD_S  # a long drive after it
    self.tick()
    self.assertEqual(self.values['NavDestination'], self.DEST)
    self.assertEqual(len(engine.router.calls), 1)

  def test_a_destination_set_parked_is_kept_for_the_next_drive(self):
    self.drive()
    self.sm.started = False
    self.tick()
    self.t += 60.
    self.values['NavDestination'] = {'latitude': 0, 'longitude': 0.0015}  # set on the phone for the next trip
    self.tick()
    self.t += 2 * DESTINATION_OFFROAD_S
    self.tick()
    self.assertEqual(self.values['NavDestination'], {'latitude': 0, 'longitude': 0.0015})
    # and navd starting parked (no drive since it started): nothing to end
    self.setUp()
    self.engine(seed=None)
    self.sm.fix, self.sm.started = (0.0001, 0.), False
    self.tick()
    self.t += 2 * DESTINATION_OFFROAD_S
    self.tick()
    self.assertEqual(self.values['NavDestination'], self.DEST)
