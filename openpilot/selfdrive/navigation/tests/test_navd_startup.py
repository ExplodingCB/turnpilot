"""TurnPilot's fast start: navd routes from LastGPSPosition before the first fix and checks that route against the fix,
navd and mapsd run offroad (plan the route and fetch its tiles, nothing more), and the GPS and nav daemons' process rules."""
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.navigation.mapsd import MapsD
from openpilot.selfdrive.navigation.navd import RECOMPUTE_HOLD, REROUTE_COUNTER_MIN, RouteEngine
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
  """navd's and mapsd's SubMaster: a live fix at .fix ((lon, lat) or None), deviceState.started from .started (None: never
  heard), and navRoute's coordinates from .route."""

  def __init__(self):
    self.fix, self.started, self.route, self.frame = None, None, None, 0
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
    return SimpleNamespace(hasFix=self.fix is not None, longitude=lon, latitude=lat, bearingDeg=90.)


class NavdCase(unittest.TestCase):
  def setUp(self):
    self.values = {'NavDestination': {'latitude': 0, 'longitude': 0.002}, 'LanguageSetting': 'main_en'}
    self.sm, self.pm = FakeSM(), MagicMock()

  def engine(self, seed=(0.0001, 0.)):
    if seed is not None:
      self.values['LastGPSPosition'] = saved_position(*seed)
    params = MagicMock()
    params.get.side_effect = self.values.get
    self.nav = RouteEngine(self.sm, self.pm, params=params, router=nav_tests.StubRouter(nav_tests.TestRouteEngine.steps()), background=False)
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
    self.sm.fix = (0.0002, 2 * METER)  # on the route
    self.assertTrue(self.tick().valid)  # at once, with the route made from the seed
    self.assertEqual(len(engine.router.calls), 1)

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
    self.assertTrue(self.tick().valid)

  def test_no_saved_position_waits_for_the_fix_as_before(self):
    engine = self.engine(seed=None)
    self.tick()
    self.assertEqual(engine.router.calls, [])
    self.sm.fix = (0.0001, 0.)
    self.tick()
    self.assertEqual(len(engine.router.calls), 1)

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
