"""Routes on real Valhalla tiles. Skipped without pyvalhalla; the west_coast_usa tests also need the BeamNG road-graph export
(~/pc_relay/incoming/osm_export/west_coast_usa.osm, or TURNPILOT_WCUSA_OSM), and the Indiana test needs the tiles that
tools/turnpilot/build_valhalla_tiles.sh builds into ~/turnpilot_data/valhalla/indiana."""
import importlib.util
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np

from openpilot.selfdrive.navigation.navd import MANEUVER_TRANSITION_THRESHOLD, RouteEngine
from openpilot.selfdrive.navigation.route import local_meters
from openpilot.selfdrive.turnpilot.routing.guidance import MAX_MANEUVERS, approach_lanes, fill_guidance, upcoming_maneuvers
from openpilot.selfdrive.turnpilot.routing.valhalla_source import ValhallaSource

ROOT = Path(__file__).parents[5]
BUILD_TILES = ROOT / 'tools/turnpilot/build_valhalla_tiles.sh'
TURN_LANES_OSM = Path(__file__).parent / 'fixtures/turn_lanes.osm'
WEST_COAST_OSM = Path(os.environ.get('TURNPILOT_WCUSA_OSM', '~/pc_relay/incoming/osm_export/west_coast_usa.osm')).expanduser()
INDIANA_TILES = Path('~/turnpilot_data/valhalla/indiana').expanduser()

# Two public-road points in the 4506-node public component of west_coast_usa (OSM nodes 1014 and 8131), (longitude, latitude).
# Private service roads are destination_only there, so the readiness check kept route endpoints on public roads.
NORTH_WEST, SOUTH_EAST = (-86.17128968, 39.78649561), (-86.14482423, 39.7580146)
# The route between them, measured on tiles built from the export: (metres, steps) by the start's bearing. Bearings 90 and 180 give
# the longer route, none, 0 and 270 the shorter one.
ROUTE_WITH_BEARING_90, ROUTE_WITHOUT = (6762, 13), (6389, 10)
DISTANCE_TOLERANCE = 10  # m

try:
  HAVE_VALHALLA = importlib.util.find_spec('valhalla') is not None
except ValueError:
  HAVE_VALHALLA = False


def build_tiles(osm, out):
  subprocess.run([str(BUILD_TILES), str(osm), str(out)], env={**os.environ, 'VALHALLA_PYTHON': sys.executable}, check=True, capture_output=True)
  return str(out)


def meters_between(a, b):
  return float(np.linalg.norm(local_meters(a, b)))


def route_engine(tiles, start, bearing):
  values = {'TurnPilotValhallaTiles': tiles, 'LanguageSetting': 'main_en'}
  params = MagicMock()
  params.get.side_effect = values.get
  pm = MagicMock()
  engine = RouteEngine(MagicMock(), pm, params=params, background=False)
  engine.last_position, engine.last_bearing = start, bearing
  return engine, pm


@unittest.skipUnless(HAVE_VALHALLA, 'pyvalhalla is not installed')
class TestTurnLanes(unittest.TestCase):
  """A three-lane one-way street, left | through | through;right, ending in a crossroads."""

  @classmethod
  def setUpClass(cls):
    cls.directory = tempfile.TemporaryDirectory()
    cls.addClassCleanup(cls.directory.cleanup)
    cls.tiles = build_tiles(TURN_LANES_OSM, Path(cls.directory.name) / 'tiles')
    cls.source = ValhallaSource(cls.tiles)
    cls.start = (-86.1699, 39.77)

  def test_left_then_right(self):
    steps = self.source.route(self.start, (-86.1600, 39.7740), 90, 'en')
    self.assertEqual(steps[0]['driving_side'], 'right')  # from the country polygon the build adds
    self.assertEqual([s['maneuver']['type'] for s in (steps[0], steps[-1])], ['depart', 'arrive'])
    self.assertEqual(steps[1]['maneuver']['modifier'], 'left')
    self.assertAlmostEqual(sum(s['distance'] for s in steps), 1292, delta=3)
    left_lane = [{'directions': ['left'], 'active': True, 'activeDirection': 'left'},
                 {'directions': ['straight'], 'active': False, 'activeDirection': 'none'},
                 {'directions': ['straight', 'right'], 'active': False, 'activeDirection': 'none'}]
    for distance in (steps[0]['distance'], 300, 10):  # from the start of the street, and in the 400 m with the sub banner
      self.assertEqual(approach_lanes(steps, 0, distance), left_lane)
    self.assertEqual(approach_lanes(steps, 1, 100), [])  # no lanes on the way to the second turn
    ahead = np.cumsum([590, steps[1]['distance'], steps[2]['distance']])  # from 590 m short of the left turn
    np.testing.assert_allclose([m['distance'] for m in upcoming_maneuvers(steps, 0, 590)], ahead)

  def test_right_turn(self):
    steps = self.source.route(self.start, (-86.1630, 39.7660), 90, 'en')
    self.assertEqual((steps[1]['maneuver']['type'], steps[1]['maneuver']['modifier']), ('turn', 'right'))
    self.assertEqual(approach_lanes(steps, 0, 500), [{'directions': ['left'], 'active': False, 'activeDirection': 'none'},
                                                     {'directions': ['straight'], 'active': False, 'activeDirection': 'none'},
                                                     {'directions': ['straight', 'right'], 'active': True, 'activeDirection': 'right'}])

  def test_exclude_locations_take_200_points(self):
    # Valhalla refuses more than its max_exclude_locations (50 by default): ValhallaSource raises it to 200 (3.3)
    far = [{'lat': 39.70 + k * 1e-4, 'lon': -86.30, 'heading': 0.} for k in range(120)]
    self.assertTrue(self.source.route(self.start, (-86.1600, 39.7740), 90, 'en', exclude=far))

  def test_navd_sends_the_lanes(self):
    engine, pm = route_engine(self.tiles, self.start, 90)
    engine.calculate_route((-86.1600, 39.7740))
    self.assertEqual(engine.step_idx, 0)
    engine.last_position = (-86.1690, 39.77)
    engine.send_instruction()
    instruction = pm.send.call_args.args[1].navInstruction.to_dict()
    self.assertEqual(instruction['maneuverModifier'], 'left')
    self.assertEqual([lane['active'] for lane in instruction['lanes']], [True, False, False])
    self.assertEqual(instruction['lanes'][0]['activeDirection'], 'left')
    self.assertEqual(len(instruction['allManeuvers']), 3)


@unittest.skipUnless(HAVE_VALHALLA and WEST_COAST_OSM.is_file(), 'needs pyvalhalla and the west_coast_usa export')
class TestWestCoastUSA(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.directory = tempfile.TemporaryDirectory()
    cls.addClassCleanup(cls.directory.cleanup)
    cls.tiles = build_tiles(WEST_COAST_OSM, Path(cls.directory.name) / 'west_coast_usa')

  def test_route(self):
    steps = ValhallaSource(self.tiles).route(NORTH_WEST, SOUTH_EAST, None, 'en')
    self.assertAlmostEqual(sum(s['distance'] for s in steps), ROUTE_WITHOUT[0], delta=DISTANCE_TOLERANCE)
    self.assertEqual(len(steps), ROUTE_WITHOUT[1])
    self.assertEqual([steps[0]['maneuver']['type'], steps[-1]['maneuver']['type']], ['depart', 'arrive'])
    self.assertEqual(steps[0]['driving_side'], 'right')
    self.assertLess(meters_between(steps[0]['geometry']['coordinates'][0], NORTH_WEST), 30)
    self.assertLess(meters_between(steps[-1]['geometry']['coordinates'][-1], SOUTH_EAST), 30)
    for step, following in zip(steps, steps[1:], strict=False):  # one continuous line
      self.assertLess(meters_between(step['geometry']['coordinates'][-1], following['geometry']['coordinates'][0]), 2)
    for step in steps[:-1]:
      self.assertTrue(step['bannerInstructions'] and step['duration'] > 0 and step['distance'] > 0)
    self.assertEqual(steps[-1]['bannerInstructions'], [])
    self.assertTrue(all('voiceInstructions' in step for step in steps))

  def test_bearing_picks_the_direction(self):
    source = ValhallaSource(self.tiles)
    for bearing, (distance, count) in ((90, ROUTE_WITH_BEARING_90), (None, ROUTE_WITHOUT)):
      steps = source.route(NORTH_WEST, SOUTH_EAST, bearing, None)
      self.assertAlmostEqual(sum(s['distance'] for s in steps), distance, delta=DISTANCE_TOLERANCE, msg=bearing)
      self.assertEqual(len(steps), count, bearing)
    for bearing in (0, 180, 270):  # the other headings still reach the destination
      steps = source.route(NORTH_WEST, SOUTH_EAST, bearing, None)
      self.assertTrue(5000 < sum(s['distance'] for s in steps) < 9000, bearing)

  def test_no_route_raises(self):
    with self.assertRaises(Exception):  # noqa: B017  # pyvalhalla raises RuntimeError; a service answers with an HTTP error
      ValhallaSource(self.tiles).route(NORTH_WEST, (-87.6298, 41.8781), None, None)  # Chicago

  def test_drive_the_route_in_navd(self):
    engine, pm = route_engine(self.tiles, NORTH_WEST, 90)
    engine.calculate_route(SOUTH_EAST)
    self.assertIsNotNone(engine.route)
    steps = engine.route
    self.assertEqual(len(steps), ROUTE_WITH_BEARING_90[1])  # the engine's start bearing is 90
    path = np.concatenate(engine.route_geometry)
    sent = lambda service: [call.args[1] for call in pm.send.call_args_list if call.args[0] == service]  # noqa: E731
    engine.send_route()
    self.assertEqual(len(sent('navRoute')[-1].navRoute.coordinates), len(path))

    # A fix every 25 m along the route.
    cumulative = np.r_[0, np.cumsum(np.linalg.norm(np.diff(local_meters(path, path[0]), axis=0), axis=1))]
    visited, previous, last_distance = [], None, 0.0
    for along in np.arange(0, cumulative[-1], 25.0):
      engine.last_position = tuple(np.interp(along, cumulative, path[:, axis]) for axis in (0, 1))
      step_idx = engine.step_idx
      engine.send_instruction()
      instruction = sent('navInstruction')[-1]
      self.assertTrue(instruction.valid)
      self.assertIn(engine.step_idx, (step_idx, step_idx + 1))  # never skips a step
      distance = instruction.navInstruction.maneuverDistance
      # past a step's end navd sends it once more, then moves on; a car already on the next step gets that step's
      step_idx = step_idx if distance < -MANEUVER_TRANSITION_THRESHOLD else engine.step_idx
      if step_idx == previous:
        self.assertLessEqual(distance, last_distance + 1)  # and closes in on the maneuver
      previous, last_distance = step_idx, distance
      maneuvers = instruction.navInstruction.allManeuvers
      self.assertEqual(len(maneuvers), min(len(steps) - 1 - step_idx, MAX_MANEUVERS))
      self.assertEqual([m.distance for m in maneuvers], sorted(m.distance for m in maneuvers))
      visited.append(step_idx)
    # Every step was driven, up to the arrival, which is only entered by driving past the destination.
    self.assertEqual(sorted(set(visited)), list(range(len(steps) - 1)))
    final = sent('navInstruction')[-1].navInstruction
    self.assertLess(final.distanceRemaining, 60)
    self.assertEqual([m.type for m in final.allManeuvers], ['arrive'])
    self.assertLess(final.allManeuvers[0].distance, 60)


@unittest.skipUnless(HAVE_VALHALLA and INDIANA_TILES.is_dir(), 'needs pyvalhalla and the Indiana tiles')
class TestIndiana(unittest.TestCase):
  def test_downtown_to_carmel(self):
    steps = ValhallaSource(str(INDIANA_TILES)).route((-86.1581, 39.7684), (-86.1180, 39.9784), 90, 'en')
    self.assertTrue(30000 < sum(s['distance'] for s in steps) < 45000)
    self.assertEqual(steps[0]['driving_side'], 'right')
    with_lanes = 0
    for idx, step in enumerate(steps):  # real OSM lane tags, at every distance along every step
      for distance in (step['distance'], 300, 0):
        instruction = MagicMock()
        fill_guidance(instruction, steps, idx, distance)
        with_lanes += bool(approach_lanes(steps, idx, distance))
    self.assertGreater(with_lanes, 0)


if __name__ == '__main__':
  unittest.main()
