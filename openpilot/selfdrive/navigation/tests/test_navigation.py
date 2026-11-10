import gzip
import re
import struct
import tempfile
import threading
import time
import unittest
from contextlib import closing, contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.navigation.mapsd import MapsD
from openpilot.selfdrive.turnpilot.routing import certify
from openpilot.selfdrive.navigation.navd import (RECOMPUTE_HOLD, REROUTE_COUNTER_MIN, ROUTE_RESEND_PERIOD, RouteEngine,
                                                 chord_bearing, set_banner, step_place)
from openpilot.selfdrive.navigation.pmtiles import PMTiles, decode_directory, tile_id
from openpilot.selfdrive.navigation.renderer import MAJOR, MEDIUM, MINOR, SIZE, STREET, TRACK, MapRenderer, draw_segments, mercator, project, stroke_widths
from openpilot.selfdrive.navigation.route import (METERS_PER_DEGREE, ROUTE_TURNS_AHEAD_M, ROUTE_TURNS_BEHIND_M, ROUTE_TURNS_STEP_M, RouteTracker,
                                                  along_path, cumulative_distance, route_turns)
from openpilot.selfdrive.navigation.tiles import RemoteTiles
from openpilot.selfdrive.navigation.vector_tile import decode_lines, decode_roads
from openpilot.selfdrive.turnpilot.maps.client import MapError

METER = 1 / 111319.49079327358  # degrees of latitude


def draw_line(image, points, intensity, width, **kwargs):
  draw_segments(image, points[:-1], points[1:], width / 2, intensity, **kwargs)


def varint(value):
  output = bytearray()
  while value > 127:
    output.append((value & 127) | 128)
    value >>= 7
  output.append(value)
  return bytes(output)


def field(number, value):
  if isinstance(value, bytes):
    return varint(number << 3 | 2) + varint(len(value)) + value
  return varint(number << 3) + varint(value)


def road_tile(crossing=None):
  # One vertical line, from (2048, 0) to (2048, 4096), and one non-road layer. With crossing='bridge' or 'tunnel',
  # a horizontal line from (1024, 2048) to (3072, 2048) at that level.
  geometry = b''.join(varint(v) for v in [9, 4096, 0, 10, 0, 8192])
  feature = field(2, b'\x00\x00') + field(3, 2) + field(4, geometry)
  roads = field(15, 2) + field(1, b'roads') + field(2, feature)
  if crossing is not None:
    geometry = b''.join(varint(v) for v in [9, 2048, 4096, 10, 4096, 0])
    roads += field(2, field(2, b'\x00\x00\x01\x01') + field(3, 2) + field(4, geometry))
  roads += field(3, b'kind') + field(3, f'is_{crossing}'.encode()) + field(4, field(1, b'minor_road')) + field(4, field(7, 1)) + field(5, 4096)
  return field(3, field(1, b'buildings')) + field(3, roads)


def make_archive(payload=None, leaf=False, run=1, max_zoom=0):
  """A PMTiles archive with one zoom-zero tile, in the root directory or in a leaf directory."""
  tile = gzip.compress(road_tile() if payload is None else payload)
  directory = gzip.compress(varint(1) + varint(0) + varint(run) + varint(len(tile)) + varint(1))
  child = directory if leaf else b''
  root = gzip.compress(varint(1) + varint(0) + varint(0) + varint(len(child)) + varint(1)) if leaf else directory
  metadata = gzip.compress(b'{"vector_layers":[{"id":"roads"}]}')
  header = bytearray(127)
  header[:8] = b'PMTiles\x03'
  offsets = [127, 127 + len(root), 127 + len(root) + len(metadata), 127 + len(root) + len(metadata) + len(child)]
  struct.pack_into('<11Q', header, 8, offsets[0], len(root), offsets[1], len(metadata), offsets[2], len(child), offsets[3], len(tile), 1, 1, 1)
  header[96:102] = bytes([1, 2, 2, 1, 0, max_zoom])
  return bytes(header + root + metadata + child + tile)


@contextmanager
def local_archive(data):
  """A PMTiles reader over in-memory bytes, bypassing the tile server."""
  source = MagicMock(size=len(data))
  source.read.side_effect = lambda offset, length: data[offset:offset + length]
  with closing(PMTiles(source)) as archive:
    yield archive


class StubRouter:
  """Stands in for navd's MapRouter (turnpilot/routing/sources.py, tested there): the steps it is given, or its error.
  With gate set to an Event, route() blocks until it is set, as a corridor download does, and stops when cancelled."""
  def __init__(self, steps=None):
    self.steps, self.error, self.calls = steps, None, []
    self.controls_result: tuple = ([], 'ok')  # what controls() gives, or an exception it raises
    self.opposing_result: dict | Exception = {}  # what opposing() gives ({step index: (m/s, lanes)}), or an exception it raises
    self.opposing_calls: list = []  # the thread of each call
    self.controls_calls: list = []  # (steps, thread name) per call
    self.status = MagicMock()
    self.gate: threading.Event | None = None
    self.positions = []  # what position() said once the "download" was done
    self.ahead_result: tuple = ([], 'ok', [])  # what controls_ahead() gives (controls, status, road), or an exception it raises
    self.ahead_calls: list = []  # (position, bearing, thread name) per call
    # TurnPilot (stop-sign avoidance): what each reroute() gives in turn (steps, or None for a failed iteration; None once
    # they run out), its calls, the controls of given steps by identity, the car's own edge, and a hook run on each reroute
    self.reroute_results: list = []
    self.reroute_calls: list = []
    self.controls_by_steps: dict = {}
    self.own = None
    self.on_reroute = None

  @property
  def call_count(self):
    return len(self.calls)

  def route(self, start, destination, bearing, language, position=None, cancelled=None):
    self.calls.append((start, destination, bearing, language))
    if self.gate is not None:
      while not self.gate.wait(0.01):
        if cancelled is not None and cancelled():
          raise MapError('cancelled', 'The destination changed during the map download.')
    if position is not None:
      self.positions.append(position())
    if self.error is not None:
      raise self.error
    return self.steps

  def controls(self, steps):
    self.controls_calls.append((steps, threading.current_thread().name))
    if id(steps) in self.controls_by_steps:
      return self.controls_by_steps[id(steps)]
    if isinstance(self.controls_result, Exception):
      raise self.controls_result
    return self.controls_result

  def reroute(self, start, destination, bearing, language, exclude):
    self.reroute_calls.append((start, destination, bearing, language, exclude))
    if self.on_reroute is not None:
      self.on_reroute()
    return self.reroute_results.pop(0) if self.reroute_results else None

  def own_edge(self, position, bearing):
    return self.own

  def exclusion_headings(self, points):
    return [self.headings.get(p[2]) for p in points] if hasattr(self, 'headings') else [None for _ in points]

  def opposing(self, steps):
    self.opposing_calls.append(threading.current_thread().name)
    if isinstance(self.opposing_result, Exception):
      raise self.opposing_result
    return self.opposing_result

  def limits(self, steps):
    """TurnPilot: the route's speed limits [(end m, m/s)] (routing/controls.py: limit_segments), or an exception it raises."""
    result = getattr(self, 'limits_result', [])
    if isinstance(result, Exception):
      raise result
    return result

  def controls_ahead(self, position, bearing):
    self.ahead_calls.append((position, bearing, threading.current_thread().name))
    if isinstance(self.ahead_result, Exception):
      raise self.ahead_result
    return self.ahead_result

  def clear(self):
    pass


@contextmanager
def serving(engine, steps):
  """The router answers with steps; yields it, with the calls made inside the block."""
  engine.router.steps = steps
  engine.router.calls = []
  yield engine.router


def route_engine(sm, pm, values, steps=None):
  params = MagicMock()
  params.get.side_effect = values.get
  params.remove.side_effect = lambda key: values.pop(key, None)
  return RouteEngine(sm, pm, params=params, router=StubRouter(steps), background=False)


class RangeHandler(BaseHTTPRequestHandler):
  def do_GET(self):
    self.server.requests.append(dict(self.headers))
    if self.headers.get('If-Match', self.server.etag) != self.server.etag:
      self.send_error(412)
      return
    if self.server.ignore_range:
      self.send_response(200)
      self.end_headers()
      return
    offset, end = map(int, re.fullmatch(r'bytes=(\d+)-(\d+)', self.headers['Range']).groups())
    data = self.server.archive[offset:end + 1]
    self.send_response(206)
    self.send_header('Content-Range', f'bytes {offset}-{end}/{len(self.server.archive)}')
    self.send_header('Content-Length', str(len(data)))
    self.send_header('ETag', self.server.etag)
    self.end_headers()
    self.wfile.write(data)

  def log_message(self, format: str, *args: object) -> None:  # noqa: A002  # stdlib override
    pass


class TileServerCase(unittest.TestCase):
  """Serves a leaf-directory archive over HTTP range requests, with the comma API and tile cache stubbed."""

  def setUp(self):
    self.directory = tempfile.TemporaryDirectory()
    self.addCleanup(self.directory.cleanup)
    self.cache = Path(self.directory.name) / 'navigation' / 'tiles'
    self.server = ThreadingHTTPServer(('127.0.0.1', 0), RangeHandler)
    self.server.archive = make_archive(leaf=True)
    self.server.etag = '"test-version"'
    self.server.ignore_range = False
    self.server.requests = []
    threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True).start()
    self.addCleanup(self.server.server_close)
    self.addCleanup(self.server.shutdown)
    self.api = MagicMock(dongle_id='test-dongle')
    self.api.get_token.return_value = 'device-token'
    self.url = f'http://127.0.0.1:{self.server.server_port}/planet.pmtiles?sig=secret'
    self.api.get.return_value.json.return_value = {'url': self.url, 'expires': time.time() + 86400}  # noqa: TID251
    for name, value in [('Api', MagicMock(return_value=self.api)), ('Params', MagicMock()), ('PC', True)]:
      self.enterContext(patch(f'openpilot.selfdrive.navigation.tiles.{name}', value))
    self.enterContext(patch('openpilot.selfdrive.navigation.tiles.Paths.comma_home', return_value=self.directory.name))


class TestEndToEnd(TileServerCase):
  def test_navd_to_map_frames(self):
    """navd routes a destination over messaging; mapsd fetches tiles over HTTP and renders map frames."""
    destination = {'latitude': 0.01, 'longitude': 0}
    steps = [{'geometry': {'coordinates': [[0, -0.01], [0, 0.01]]}, 'distance': 2224, 'duration': 100, 'bannerInstructions': []}]
    with OpenpilotPrefix():
      pm = messaging.PubMaster(['navInstruction', 'navRoute', 'gpsLocationExternal'])
      sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal', 'navRoute'])
      values = {'NavDestination': dict(destination)}
      navd = route_engine(messaging.SubMaster(['gpsLocation', 'gpsLocationExternal']), pm, values, steps)
      mapsd = MapsD()
      self.addCleanup(mapsd.close)

      def frame(fix=True):
        gps = messaging.new_message('gpsLocationExternal', valid=True)
        gps.gpsLocationExternal.hasFix = fix
        pm.send('gpsLocationExternal', gps)
        navd.update()
        sm.update(0)
        return mapsd.update(sm)

      image, timestamp = frame()
      # The route is centered over the white road, with the ego arrow three quarters down.
      self.assertEqual((image[128, 128], image[128, 131], image[188, 128], image[195, 128]), (128, 255, 255, 0))
      self.assertEqual(timestamp, sm.logMonoTime['gpsLocationExternal'])
      fetched = len(self.server.requests)
      tracker = mapsd.tracker
      navd.send_route()  # the same route again, as a restarted mapsd would get it
      frame()
      self.assertIs(mapsd.tracker, tracker)  # progress along it is kept
      values.pop('NavDestination')
      self.assertFalse(frame()[0].any())  # black without a route
      values['NavDestination'] = dict(destination)
      self.assertEqual(frame()[0][128, 128], 128)
      self.assertEqual(len(self.server.requests), fetched)  # the second route is served from the cache
      image, timestamp = frame(fix=False)
      self.assertFalse(image.any() or timestamp)  # black, not stale, without a fix
      self.api.get.assert_called_once()
      self.assertTrue(all('Range' in headers for headers in self.server.requests))
      self.assertTrue(all(headers['If-Match'] == self.server.etag for headers in self.server.requests[1:]))
      self.assertTrue(all(b'secret' not in p.read_bytes() for p in self.cache.iterdir()))
      # The archive is replaced on the server and the cache evicted: the next fetch fails, the map goes black,
      # and after a delay a fresh reader starts over on the new version.
      self.server.etag = '"new-version"'
      for cached in self.cache.iterdir():
        cached.unlink()
      values.pop('NavDestination')
      frame()
      values['NavDestination'] = dict(destination)
      image, timestamp = frame()
      self.assertFalse(image.any() or timestamp)
      fetched = len(self.server.requests)
      self.assertFalse(frame()[1])  # not retried yet
      self.assertEqual(len(self.server.requests), fetched)
      mapsd.retry_at = 0
      self.assertEqual(frame()[0][128, 128], 128)
      self.assertEqual((self.cache / 'version').read_text().split()[0], '"new-version"')


class TestRemoteTiles(TileServerCase):
  def test_shared_cache_and_refresh(self):
    with closing(PMTiles(RemoteTiles())) as archive:
      archive.source.expires = 0  # the signed URL is refreshed before the next fetch
      self.assertEqual(archive.get(0, 0, 0), road_tile())
      self.assertEqual(archive.get(0, 0, 0), road_tile())
      self.assertEqual(self.api.get.call_count, 2)
    count = len(self.server.requests)
    with closing(PMTiles(RemoteTiles())) as archive:
      self.assertEqual(archive.get(0, 0, 0), road_tile())
    # A second reader revalidates only the header; directories and tiles are shared on disk.
    self.assertEqual(len(self.server.requests), count + 1)
    with patch('openpilot.selfdrive.navigation.tiles.CACHE_LIMIT', 1), closing(RemoteTiles()) as source:
      source.read(1, 2)  # an uncached range is downloaded, then the cache is trimmed to the limit
    self.assertEqual([p.name for p in self.cache.iterdir()], ['version'])

  def test_prefetch_merges_nearby_ranges(self):
    with patch('openpilot.selfdrive.navigation.tiles.MERGE_GAP', 16), closing(RemoteTiles()) as source:
      count = len(self.server.requests)
      source.prefetch([(20, 10), (0, 10), (60, 10), (20, 10)])
      self.assertEqual(len(self.server.requests), count + 2)  # 0-29 in one request, 60-69 in another
      self.assertEqual(self.server.requests[-2]['Range'], 'bytes=0-29')
      for offset in (0, 20, 60):
        self.assertEqual(source.read(offset, 10), self.server.archive[offset:offset + 10])
      self.assertEqual(len(self.server.requests), count + 2)

  def test_failures(self):
    with closing(PMTiles(RemoteTiles())) as archive:
      self.server.etag = '"new-version"'
      with self.assertRaisesRegex(OSError, '412'):  # the archive changed under an open reader
        archive.get(0, 0, 0)
    with closing(PMTiles(RemoteTiles())) as archive:  # a new reader starts over on the new version
      self.assertEqual(archive.get(0, 0, 0), road_tile())
    self.server.ignore_range = True
    with self.assertRaisesRegex(OSError, '200'):
      RemoteTiles()
    self.api.get.return_value.raise_for_status.side_effect = requests.HTTPError(self.url)
    with self.assertRaisesRegex(OSError, '^Unable to fetch map tiles$'):  # the signed URL stays out of the error
      RemoteTiles()


class TestRouteEngine(unittest.TestCase):
  def setUp(self):
    self.pm = MagicMock()
    self.values = {'NavDestination': {'latitude': 0, 'longitude': 0.002}, 'LanguageSetting': 'main_en'}
    self.sm = MagicMock()
    self.sm.valid = dict.fromkeys(('gpsLocation', 'gpsLocationExternal'), True)
    self.sm.logMonoTime = dict.fromkeys(self.sm.valid, time.monotonic_ns())
    self.sm.recv_time = dict.fromkeys(self.sm.valid, time.monotonic())
    # TurnPilot: a car driving east with a good heading of motion (speed and bearing accuracy: hold.py), so a route made from
    # its fix has a start heading and isn't held back for a check of the car's direction (navd heading_due)
    self.sm.__getitem__.return_value = SimpleNamespace(hasFix=True, latitude=0, longitude=0, bearingDeg=90, speed=10., bearingAccuracyDeg=2.)
    self.engine = route_engine(self.sm, self.pm, self.values)
    self.engine.last_position, self.engine.last_bearing = (0, 0), 90
    self.response = self.steps()

  @staticmethod
  def steps():
    distance = 111.195  # Mapbox's length of 0.001 degrees at the equator
    return [
      {'geometry': {'coordinates': [[0, 0], [0.0005, 0], [0.001, 0]]}, 'distance': distance, 'duration': 10,
       'bannerInstructions': [{'distanceAlongGeometry': 100, 'primary': {'text': 'Turn right', 'type': 'turn', 'modifier': 'right'}}]},
      {'geometry': {'coordinates': [[0.001, 0], [0.002, 0]]}, 'distance': distance, 'duration': 10, 'bannerInstructions': []},
    ]

  def last_message(self, service):
    return next(call.args[1] for call in reversed(self.pm.send.call_args_list) if call.args[0] == service)

  def load_route(self):
    with serving(self.engine, self.response) as get:
      self.engine.calculate_route((0.002, 0))
    self.engine.send_route()
    return get

  def test_request_and_instruction(self):
    get = self.load_route()
    self.assertEqual(get.calls, [((0, 0), (0.002, 0), 90, 'en')])  # the ego's position and bearing, and the language
    self.assertEqual(len(self.last_message('navRoute').navRoute.coordinates), 5)
    self.engine.send_instruction()
    message = self.last_message('navInstruction')
    self.assertTrue(message.valid)
    instruction = message.navInstruction.to_dict()
    self.assertEqual((instruction['maneuverPrimaryText'], instruction['maneuverModifier'], instruction['timeRemaining']), ('Turn right', 'right', 20))
    self.engine.last_position = (0.0007, 0)  # on the step's second segment
    self.engine.send_instruction()
    self.assertAlmostEqual(self.last_message('navInstruction').navInstruction.timeRemaining, 13, delta=0.05)
    self.engine.last_position = (0.001, 0)  # at the maneuver
    self.engine.send_instruction()
    self.assertAlmostEqual(self.last_message('navInstruction').navInstruction.maneuverDistance, 0, delta=0.01)

  def test_the_routes_shape_around_the_car(self):
    # TurnPilot (off-route turn, F3): the route's heading change per bin around the car's matched place, on every valid message
    self.load_route()
    self.engine.last_position = (0.0004, 0)  # 44.5 m along the first step, the route straight east
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    self.assertEqual((ni.routeTurnsStart, ni.routeTurnsStep), (-40., ROUTE_TURNS_STEP_M))  # clipped at the route's start
    self.assertEqual(len(ni.routeTurns), 8 + int(ROUTE_TURNS_AHEAD_M / ROUTE_TURNS_STEP_M))
    self.assertTrue(all(abs(t) < 1e-6 for t in ni.routeTurns))

  def test_route_turns_find_a_bend_with_no_maneuver(self):
    # two steps, the first ending at a street that turns right 90 deg 100 m in, the second's first point repeating it
    m = 1 / METERS_PER_DEGREE
    first = [[0, 0], [0, 50 * m], [0, 100 * m]]
    second = [[0, 100 * m], [50 * m, 100 * m], [200 * m, 100 * m]]
    turns, start = route_turns([first, second], 0, 80.)
    self.assertEqual(start, -ROUTE_TURNS_BEHIND_M)
    self.assertEqual(len(turns), int((ROUTE_TURNS_BEHIND_M + ROUTE_TURNS_AHEAD_M) / ROUTE_TURNS_STEP_M))
    corner = [i for i, t in enumerate(turns) if abs(t) > 1e-3]
    self.assertEqual(len(corner), 1)
    self.assertAlmostEqual(turns[corner[0]], 90., delta=0.1)  # + right
    self.assertAlmostEqual(start + corner[0] * ROUTE_TURNS_STEP_M, 20., delta=ROUTE_TURNS_STEP_M)  # 20 m ahead of the car
    turns, start = route_turns([first, second], 1, 30.)  # on the second step, past the bend: 30 m behind the car
    corner = [i for i, t in enumerate(turns) if abs(t) > 1e-3]
    self.assertAlmostEqual(start + corner[0] * ROUTE_TURNS_STEP_M, -30., delta=ROUTE_TURNS_STEP_M)
    self.assertEqual(route_turns([[[0, 0], [0, 0]]], 0, 0.), ([], 0.))

  def test_controls_in_the_maneuver_distance_frame(self):
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    step = 111.195
    # a stop sign 50 m into the second step, and a signal 30 m into the first
    self.engine.router.controls_result = ([Control(0, 'signal', 30., 0, 30.), Control(1, 'stop', step + 50., 1, 50., True)], 'ok')
    self.load_route()
    self.engine.last_position = (0.0004, 0)  # 44.5 m along the first step
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    self.assertEqual(str(ni.controlsStatus), 'ok')
    self.assertEqual([(c.id, str(c.kind)) for c in ni.controls], [(0, 'signal'), (1, 'stop')])
    self.assertAlmostEqual(ni.controls[0].distance, 30. - 0.4 * step, delta=0.1)  # passed, still sent within 15 m
    self.assertAlmostEqual(ni.controls[1].distance, step + 50. - 0.4 * step, delta=0.1)
    self.assertTrue(ni.controls[1].lineMapped)
    # on the second step: the same frame as maneuverDistance (the steps' distances, then along the step)
    self.engine.last_position = (0.0012, 0)
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    self.assertEqual(self.engine.step_idx, 1)
    self.assertEqual([c.id for c in ni.controls], [1])  # the signal is 52 m behind now
    self.assertAlmostEqual(ni.controls[0].distance, 50. - 0.0002 * step / 0.001, delta=0.1)

  def test_navd_fills_junction_info_on_the_route_and_off_it(self):
    """A stop or yield's junction goes out with it (MapControl.junctionInfo, STOP_SIGNS.md 3.1): on the route and in the
    lookups ahead off it; a signal has none (status none)."""
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    from openpilot.selfdrive.turnpilot.routing.junctions import Approach, Junction

    def junction(osm, tier):
      return Junction(osm_node_id=osm, lat=0.001, lon=0., stop_point_distance=13., heading_in=90., maneuver='left', turn_angle=-90.,
                      rail_distance=-1., sidecar=True, approaches=[Approach(0., -90., 'exit', 'secondary', 'road', True, True, 1, 1,
                                                                            'none', 'none', tier, 0., 20.1, 'classDefault', 812)])
    self.engine.router.controls_result = ([Control(0, 'signal', 30., 0, 30.),
                                           Control(1, 'stop', 60., 0, 60., junction_info=junction(900, 'no'))], 'ok')
    self.load_route()
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    signal, stop = ni.controls
    self.assertEqual(str(signal.junctionInfo.status), 'none')
    j = stop.junctionInfo
    self.assertEqual((str(j.status), j.osmNodeId, str(j.maneuver), round(j.stopPointDistance, 1), j.sidecar), ('ok', 900, 'left', 13., True))
    self.assertEqual([(str(a.role), str(a.roadClass), str(a.tier), a.wayId) for a in j.approaches], [('exit', 'secondary', 'no', 812)])
    off, status, road = self.ahead([(0, 'stop', 80.)])
    off[0].junction_info = junction(901, 'unknown')
    off[0].junction_info.maneuver = 'unknown'
    self.engine.router.ahead_result = (off, status, road)
    self.engine.last_position, self.engine.last_bearing = (0.0001, 0.001), 92.
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    j = ni.offRouteControls[0].junctionInfo
    self.assertEqual((j.osmNodeId, str(j.maneuver), str(j.approaches[0].tier)), (901, 'unknown', 'unknown'))

  def test_controls_computed_by_the_route_job_once(self):
    self.engine.background = True
    self.engine.router.steps = self.response
    self.engine.calculate_route((0.002, 0))
    self.engine.job.join(5.)
    self.engine.collect_route((0.002, 0))
    for _ in range(3):
      self.engine.send_instruction()
    self.assertEqual([name for _, name in self.engine.router.controls_calls], ['navd-route'])

  def test_the_opposing_roads_speed_rides_with_its_maneuver(self):
    # the unprotected left's R_req (turnpilot/unprotected_left.py) is from the opposing road's mapped limit: the route job looks
    # them up once (router.opposing, off navd's loop) and each instruction carries its maneuver's
    self.engine.background = True
    self.engine.router.steps = self.response
    self.engine.router.opposing_result = {0: (11.11, 1)}
    self.engine.calculate_route((0.002, 0))
    self.engine.job.join(5.)
    self.engine.collect_route((0.002, 0))
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    self.assertAlmostEqual(ni.maneuverOpposingSpeed, 11.11, places=3)
    self.assertEqual(ni.maneuverOpposingLanes, 1)
    self.assertEqual(self.engine.router.opposing_calls, ['navd-route'])
    # a failure keeps the route, its opposing roads unknown (0: the left takes 45 mph)
    self.engine.background = False
    self.engine.router.opposing_result = RuntimeError("synthetic")
    self.load_route()
    self.engine.send_instruction()
    message = self.last_message('navInstruction')
    self.assertTrue(message.valid)
    self.assertEqual(message.navInstruction.maneuverOpposingSpeed, 0.)

  def test_speed_limit_is_the_edge_the_car_is_on(self):
    # navInstruction.speedLimit: navd never filled it, so turnpilotd took every road as 50 mph or more and lit the signal 300 ft
    # before every turn and lane change (car_signals.lead_m). The route job reads the kept route's limits once (router.limits:
    # tagged, else the road class's default), and each instruction carries the limit of the edge the car is on
    self.engine.router.limits_result = [(60., 13.41), (222.39, 8.94)]
    self.load_route()
    self.engine.send_instruction()
    self.assertAlmostEqual(self.last_message('navInstruction').navInstruction.speedLimit, 13.41, places=2)
    self.engine.last_position = (0.0015, 0)  # on the second step, past the first edge's end
    self.engine.send_instruction()
    self.assertAlmostEqual(self.last_message('navInstruction').navInstruction.speedLimit, 8.94, places=2)
    # a failure keeps the route, its limit unknown (0: the signal leads 300 ft)
    self.engine.last_position = (0, 0)
    self.engine.router.limits_result = RuntimeError("synthetic")
    self.load_route()
    self.engine.send_instruction()
    message = self.last_message('navInstruction')
    self.assertTrue(message.valid)
    self.assertEqual(message.navInstruction.speedLimit, 0.)

  def test_controls_failure_keeps_the_route(self):
    self.engine.router.controls_result = RuntimeError("synthetic")
    self.load_route()
    self.engine.send_instruction()
    message = self.last_message('navInstruction')
    self.assertTrue(message.valid)
    self.assertEqual(str(message.navInstruction.controlsStatus), 'failed')
    self.assertEqual(len(message.navInstruction.controls), 0)

  def test_controls_window_and_cap(self):
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    self.engine.router.controls_result = ([Control(i, 'stop', 20. * i, 0, 20. * i) for i in range(12)] +
                                          [Control(12, 'signal', 500., 1, 500.)], 'partial')
    self.load_route()
    self.engine.last_position = (0.0002, 0)  # 22 m along
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    self.assertEqual(str(ni.controlsStatus), 'partial')
    ds = [c.distance for c in ni.controls]
    self.assertEqual(len(ds), 8)
    self.assertEqual([c.id for c in ni.controls], list(range(1, 9)))  # -15 m to 400 m, nearest first
    self.assertTrue(all(-15. < d < 400. for d in ds) and ds == sorted(ds))

  def test_invalid_instruction_carries_no_controls(self):
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    self.engine.router.controls_result = ([Control(0, 'stop', 60., 0, 60.)], 'ok')
    self.load_route()
    self.engine.last_position = (0.0001, 0.001)  # 111 m off the route
    self.engine.send_instruction()
    message = self.last_message('navInstruction')
    self.assertFalse(message.valid)
    self.assertEqual(len(message.navInstruction.controls), 0)
    self.assertEqual(str(message.navInstruction.controlsStatus), 'ok')

  @staticmethod
  def ahead(controls, status='ok', lat=0.001, lon0=0.0001):
    """controls_ahead()'s answer for a road east from (lat, lon0): controls [(id, kind, m along)], their stop points on it."""
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    road = [(lat, lon0), (lat, lon0 + 0.002)]
    return [Control(i, kind, m, 0, m, point=(lat, lon0 + m / 111195.)) for i, kind, m in controls], status, road

  def test_off_route_instruction_carries_the_controls_ahead_of_the_car(self):
    """Off the route, the stop signs and signals on the road the car is on, looked up from where it is and its heading
    (BeamNG route_demo, 2026-10-28: a signal driven through at speed while rerouting)."""
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    self.engine.router.controls_result = ([Control(0, 'stop', 60., 0, 60.)], 'ok')
    self.load_route()
    self.engine.router.ahead_result = self.ahead([(0, 'signal', 80.), (1, 'stop', 300.)])
    self.engine.last_position, self.engine.last_bearing = (0.0001, 0.001), 92.  # 111 m off the route, heading east
    self.engine.send_instruction()
    message = self.last_message('navInstruction')
    self.assertFalse(message.valid)
    ni = message.navInstruction
    self.assertEqual(len(ni.controls), 0)  # the route's are nowhere the car is
    self.assertEqual(str(ni.offRouteControlsStatus), 'ok')
    self.assertEqual(self.engine.router.ahead_calls[-1][:2], ((0.0001, 0.001), 92.))
    self.assertEqual([(str(c.kind), round(c.distance, 1)) for c in ni.offRouteControls], [('signal', 80.), ('stop', 300.)])
    first = [c.id for c in ni.offRouteControls]
    # 20 m on along that road: the next lookup finds them again from there, their ids kept
    self.engine.last_position = (0.0001 + 20. / 111195., 0.001)
    self.engine.router.ahead_result = self.ahead([(5, 'signal', 60.), (6, 'stop', 280.), (7, 'yield', 30.)],
                                                  lon0=0.0001 + 20. / 111195.)
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    by_kind = {str(c.kind): c for c in ni.offRouteControls}
    self.assertEqual([by_kind['signal'].id, by_kind['stop'].id], first)
    self.assertNotIn(by_kind['yield'].id, first)
    self.assertAlmostEqual(by_kind['signal'].distance, 60., delta=0.1)
    # back on the route: none of it
    self.engine.last_position = (0.0004, 0)
    self.engine.send_instruction()
    message = self.last_message('navInstruction')
    self.assertTrue(message.valid)
    self.assertEqual((len(message.navInstruction.offRouteControls), str(message.navInstruction.offRouteControlsStatus)), (0, 'none'))

  def test_off_route_controls_measured_from_the_car_between_lookups_and_pending_before_the_first(self):
    self.load_route()
    self.engine.background = True
    self.engine.router.ahead_result = self.ahead([(0, 'signal', 80.)])
    self.engine.last_position = (0.0001, 0.001)
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    self.assertEqual((str(ni.offRouteControlsStatus), len(ni.offRouteControls)), ('pending', 0))
    self.engine.ahead_job.join(5.)
    self.assertEqual(self.engine.router.ahead_calls[-1][2], 'navd-ahead')  # off navd's loop
    self.engine.router.ahead_result = RuntimeError("valhalla went away")
    self.engine.last_position = (0.0001 + 10. / 111195., 0.001)  # 10 m on
    self.engine.send_instruction()  # the first lookup's, measured from where the car is now; a second lookup starts
    ni = self.last_message('navInstruction').navInstruction
    self.assertEqual(str(ni.offRouteControlsStatus), 'ok')
    self.assertAlmostEqual(ni.offRouteControls[0].distance, 70., delta=0.1)
    self.engine.ahead_job.join(5.)
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    self.assertEqual((str(ni.offRouteControlsStatus), len(ni.offRouteControls)), ('failed', 0))  # never "clear"

  def test_rerouting_carries_the_controls_ahead_and_no_destination_none(self):
    self.load_route()
    self.engine.router.ahead_result = self.ahead([(0, 'stop', 40.)], lat=0., lon0=0.)  # the car's at (0, 0) heading east
    self.engine.job = SimpleNamespace(destination=(0.002, 0), cancel=threading.Event(), is_alive=lambda: True)  # a reroute
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    self.assertEqual((str(ni.offRouteControlsStatus), [str(c.kind) for c in ni.offRouteControls]), ('ok', ['stop']))
    self.engine.job = None
    self.engine.clear_route()
    self.engine.requested_destination = None
    calls = len(self.engine.router.ahead_calls)
    self.engine.send_instruction()
    ni = self.last_message('navInstruction').navInstruction
    self.assertEqual((str(ni.offRouteControlsStatus), len(ni.offRouteControls)), ('none', 0))
    self.assertEqual(len(self.engine.router.ahead_calls), calls)  # no destination: nothing looked up

  def test_banner_selection(self):
    banners = [{'distanceAlongGeometry': 100, 'primary': {'text': 'Turn right', 'type': 'turn', 'modifier': 'right'}},
               {'distanceAlongGeometry': 50, 'primary': {'text': 'Turn right'}, 'secondary': {'text': 'onto Main St'}}]
    # The last banner still ahead of the ego, else the first: 120 m, 70 m and 30 m from the maneuver.
    for distance, expected in [(120, ('Turn right', '', False)), (70, ('Turn right', '', True)), (30, ('Turn right', 'onto Main St', True))]:
      instruction = messaging.new_message('navInstruction').navInstruction
      set_banner(instruction, banners, distance)
      self.assertEqual((instruction.maneuverPrimaryText, instruction.maneuverSecondaryText, instruction.showFull), expected)

  def test_route_sent_on_change_and_periodically(self):
    sent = lambda: sum(call.args[0] == 'navRoute' for call in self.pm.send.call_args_list)  # noqa: E731
    self.sm.frame = 1
    with serving(self.engine, self.response):
      self.engine.update()  # the route to a new destination is sent once
      self.engine.update()
    self.assertEqual(sent(), 1)
    self.sm.frame = ROUTE_RESEND_PERIOD  # and again periodically, for a mapsd that started after it
    self.engine.update()
    self.assertEqual(sent(), 2)

  def skip_route(self):
    """A right turn 111 m ahead, onto a road north; the plan turnpilotd publishes, skipping it or not."""
    distance = 111.195
    self.response = [
      {'geometry': {'coordinates': [[0, 0], [0.0005, 0], [0.001, 0]]}, 'distance': distance, 'duration': 10,
       'bannerInstructions': [{'distanceAlongGeometry': 100, 'primary': {'text': 'Turn left', 'type': 'turn', 'modifier': 'left'}}]},
      {'geometry': {'coordinates': [[0.001, 0], [0.001, 0.001]]}, 'distance': distance, 'duration': 10, 'bannerInstructions': []},
    ]
    self.skip_route_plan()

  def skip_route_plan(self):
    """Load self.response and the plan turnpilotd publishes."""
    self.load_route()
    msg = messaging.new_message('turnpilotPlan')
    self.plan = msg.turnpilotPlan
    # turnpilotd's runtime inputs ride on every plan (SG4a, STOP_SIGNS.md 3.2): the mode the param says (on a device manager
    # writes TurnPilotStopSignGo's default, so navd and turnpilotd read the same) and the side views off: nothing re-runs
    self.plan.stopGo.mode = certify.stop_sign_mode(self.engine.params)
    gps = self.sm.__getitem__.return_value
    self.sm.data = {'turnpilotPlan': None}
    self.sm.updated = {'turnpilotPlan': True}
    self.sm.valid['turnpilotPlan'] = True
    self.sm.__getitem__.side_effect = lambda s: self.plan if s == 'turnpilotPlan' else gps

  def drive_past_junction(self, too_fast, plan_distance=55.):
    """The car 55 m before the turn, turnpilotd's plan seen, then straight on past the junction. Returns the router."""
    self.skip_route()
    self.engine.last_position = (0.0005, 0)
    self.engine.send_instruction()
    self.plan.tooFastToTurn, self.plan.maneuverDistance = too_fast, plan_distance
    self.engine.note_skip()
    with serving(self.engine, self.response) as router:
      self.engine.recompute_hold = RECOMPUTE_HOLD  # just routed: a reroute would normally wait
      self.engine.last_position = (0.00105, 0)  # 5.6 m past the junction: not yet
      self.engine.recompute_route(gps_ok=True)
      self.engine.send_instruction()
      self.assertEqual(router.call_count, 0)
      self.engine.last_position = (0.00115, 0)  # 16.7 m past it, straight on
      self.engine.recompute_route(gps_ok=True)
      self.engine.send_instruction()
    return router

  def test_reroute_at_once_past_a_skipped_turn(self):
    # TurnPilot: too fast to turn, the turn was skipped; past the junction navd reroutes at once, ignoring the hold and
    # the off-route counter, and never starts the turn's step
    router = self.drive_past_junction(too_fast=True)
    self.assertEqual(router.call_count, 1)
    self.assertEqual(router.calls[0][0], (0.00115, 0))
    self.assertEqual(self.engine.route_count, 2)
    self.assertIsNone(self.engine.skipped)

  def test_no_skip_moves_on_as_before(self):
    router = self.drive_past_junction(too_fast=False)
    self.assertEqual(router.call_count, 0)
    self.assertEqual(self.engine.step_idx, 1)

  def test_skip_for_another_maneuver_is_ignored(self):
    # turnpilotd's flag still on the previous maneuver, 300 m off what navd has: not this step's turn
    router = self.drive_past_junction(too_fast=True, plan_distance=355.)
    self.assertEqual(router.call_count, 0)
    self.assertIsNone(self.engine.skipped)

  def test_shadow_skip_is_ignored(self):
    # TurnPilot in shadow mode: nothing acts on its plan, navd included (the human drives, and may brake late and turn)
    self.skip_route()
    self.engine.last_position = (0.0005, 0)
    self.engine.send_instruction()
    self.plan.tooFastToTurn, self.plan.maneuverDistance, self.plan.shadow = True, 55., True
    self.engine.note_skip()
    self.assertIsNone(self.engine.skipped)

  def miss_past_junction(self, at=(0.00115, 0), shadow=False):
    """The car 55 m before the turn, then straight on past the junction; there turnpilotd says the turn was missed (a turn
    from a standstill that went straight: turnMissed). Returns the router."""
    self.skip_route()
    self.engine.last_position = (0.0005, 0)
    self.engine.send_instruction()
    self.plan.maneuverDistance = 55.
    self.engine.note_skip()
    with serving(self.engine, self.response) as router:
      self.engine.recompute_hold = RECOMPUTE_HOLD  # just routed: a reroute would normally wait
      self.engine.last_position = at
      self.engine.recompute_route(gps_ok=True)
      self.engine.send_instruction()  # navd moves on to the next step here, past the turn's end
      self.assertEqual(router.call_count, 0)
      self.plan.turnMissed, self.plan.maneuverDistance, self.plan.shadow = True, -15., shadow
      self.engine.note_skip()
      self.engine.recompute_route(gps_ok=True)
      self.engine.send_instruction()
      for _ in range(3):  # the plan says so for about 3 s: one reroute for it
        self.engine.note_skip()
        self.engine.recompute_route(gps_ok=True)
        self.engine.send_instruction()
    return router

  def test_reroute_at_once_on_a_missed_turn(self):
    # TurnPilot: a turn from a standstill went straight past its junction; navd reroutes at once, as past a skipped turn,
    # ignoring the hold and the off-route counter, and only once
    router = self.miss_past_junction()
    self.assertEqual(router.call_count, 1)
    self.assertEqual(router.calls[0][0], (0.00115, 0))
    self.assertEqual(self.engine.route_count, 2)

  def test_missed_turn_on_the_route_after_all_or_in_shadow_is_not_rerouted(self):
    # on the road the route turns onto (the car turned, its yaw misread), navd is on that step already: no reroute
    router = self.miss_past_junction(at=(0.001, 0.0002))
    self.assertEqual(router.call_count, 0)
    self.assertEqual(self.engine.step_idx, 1)
    router = self.miss_past_junction(shadow=True)
    self.assertEqual(router.call_count, 0)

  def test_skipped_turn_taken_after_all_moves_on(self):
    # the turn was flagged skipped, but the car turned (a driver who braked, or a desire after turnpilotd's skip state
    # reset): on the next step's road, navd moves on to it rather than rerouting from a spot on the route
    distance = 111.195
    self.response = [
      {'geometry': {'coordinates': [[0, 0], [0.0009, 0], [0.001, 0]]}, 'distance': distance, 'duration': 10,
       'bannerInstructions': [{'distanceAlongGeometry': 100, 'primary': {'text': 'Turn left', 'type': 'turn', 'modifier': 'left'}}]},
      {'geometry': {'coordinates': [[0.001, 0], [0.001, 0.002]]}, 'distance': 2 * distance, 'duration': 20, 'bannerInstructions': []},
    ]
    self.skip_route_plan()
    self.engine.last_position = (0.0005, 0)
    self.engine.send_instruction()
    self.plan.tooFastToTurn, self.plan.maneuverDistance = True, 55.
    self.engine.note_skip()
    self.assertIsNotNone(self.engine.skipped)
    with serving(self.engine, self.response) as router:
      self.engine.recompute_hold = RECOMPUTE_HOLD
      self.engine.last_position = (0.001, 0.0002)  # 22 m up the road the route turns onto
      self.engine.recompute_route(gps_ok=True)
      self.engine.send_instruction()
      self.assertEqual(router.call_count, 0)
    self.assertEqual(self.engine.step_idx, 1)
    self.assertTrue(self.last_message('navInstruction').valid)

  def test_new_route_is_marked_by_an_invalid_instruction(self):
    # TurnPilot: a new route can start with a maneuver like the old one's, so one invalid instruction goes between them
    valid = lambda: [call.args[1].valid for call in self.pm.send.call_args_list if call.args[0] == 'navInstruction']  # noqa: E731
    self.sm.frame = 1
    with serving(self.engine, self.response):
      self.engine.update()
      self.values['NavDestination'] = {'latitude': 0, 'longitude': 0.003}
      self.engine.recompute_hold = 0
      self.engine.update()
      self.engine.update()
    self.assertEqual(valid(), [True, False, True])

  def test_every_instruction_carries_the_route_count(self):
    # TurnPilot: an invalid instruction for the same route is a gap in it (off it for a fix, rerouting), which a turn
    # under way is held through; only a new routeCount is a new route
    counts = lambda: [(call.args[1].valid, call.args[1].navInstruction.routeCount)  # noqa: E731
                      for call in self.pm.send.call_args_list if call.args[0] == 'navInstruction']
    self.load_route()
    self.engine.send_instruction()
    self.engine.last_position = (0.0005, 0.0003)  # 33 m beside the first step: off it
    self.engine.send_instruction()
    self.engine.job = MagicMock()  # rerouting for the same destination
    self.engine.send_instruction()
    self.engine.job = None
    self.engine.last_position = (0, 0)
    self.load_route()  # the new route, from where the car is (navd routed_across: one the car stands 33 m off is checked)
    self.engine.send_instruction()
    self.assertEqual(counts(), [(True, 1), (False, 1), (False, 1), (True, 2)])

  def test_maneuvers_and_rerouting(self):
    self.load_route()
    # driving (setUp's fixes: 10 m/s), so the reroute that lands with the car off its start is a road's and kept as before
    # (navd routed_across: VERIFY_STREET_V)
    self.engine.v_ego = 10.
    self.engine.last_position = (0, 0.001)  # 111 m off route: reroute after a few consecutive misses
    for _ in range(REROUTE_COUNTER_MIN):
      self.assertFalse(self.engine.should_recompute())
    self.assertTrue(self.engine.should_recompute())
    with serving(self.engine, self.response) as get:
      self.engine.recompute_route(gps_ok=False)  # but not while the GPS drifts in a tunnel
      self.assertEqual(get.call_count, 0)
    with serving(self.engine, self.response) as get:
      self.engine.recompute_route(gps_ok=True)
      self.engine.reroute_counter = REROUTE_COUNTER_MIN  # still off route, but an immediate second recompute is held back
      self.engine.recompute_route(gps_ok=True)
      self.assertEqual(get.call_count, 1)
    self.engine.last_position = (0.0012, 0)  # past the first maneuver
    self.engine.send_instruction()
    self.assertEqual(self.engine.step_idx, 1)
    self.engine.last_position = (0.00215, 0)  # 17 m past the destination: arrived, but the route is kept nearby
    self.engine.send_instruction()
    self.assertIsNotNone(self.engine.route)
    self.engine.last_position = (0.0024, 0)  # driving away from the destination
    self.engine.send_instruction()
    self.assertIsNone(self.engine.route)
    self.assertNotIn('NavDestination', self.values)

  def test_off_route_instruction_is_invalid(self):
    self.load_route()
    self.engine.last_position = (0.0005, 0.0003)  # 33 m beside the first step: a missed turn, before the reroute
    self.engine.send_instruction()
    self.assertFalse(self.last_message('navInstruction').valid)
    self.assertEqual(self.engine.step_idx, 0)
    self.engine.last_position = (0.0005, 0.00005)  # back on it
    self.engine.send_instruction()
    self.assertTrue(self.last_message('navInstruction').valid)

  def test_sharp_turn_moves_on_to_the_next_step(self):
    distance = 111.195
    self.response = [
      {'geometry': {'coordinates': [[0, 0], [0.001, 0]]}, 'distance': distance, 'duration': 10,
       'bannerInstructions': [{'distanceAlongGeometry': 100, 'primary': {'text': 'Turn left', 'type': 'turn', 'modifier': 'left'}}]},
      {'geometry': {'coordinates': [[0.001, 0], [0.001, 0.002]]}, 'distance': 2 * distance, 'duration': 20, 'bannerInstructions': []},
    ]
    self.load_route()
    # 22 m up the road it turned into: along the first step that is short of its end + 10 m, but it is on the second
    self.engine.last_position = (0.001, 0.0002)
    self.engine.send_instruction()
    self.assertEqual(self.engine.step_idx, 1)
    self.assertTrue(self.last_message('navInstruction').valid)

  def test_failed_request_clears_route(self):
    self.load_route()
    self.engine.router.error = MapError('unreachable', "Can't reach the TurnPilot map server")
    self.load_route()
    self.assertIsNone(self.engine.route)
    self.assertEqual(len(self.last_message('navRoute').navRoute.coordinates), 0)
    self.engine.send_instruction()
    self.assertFalse(self.last_message('navInstruction').valid)

  def test_gps_source(self):
    fixes = {'gpsLocation': SimpleNamespace(hasFix=True, latitude=1, longitude=1, bearingDeg=180),
             'gpsLocationExternal': SimpleNamespace(hasFix=True, latitude=2, longitude=2, bearingDeg=0)}
    self.sm.__getitem__.side_effect = fixes.get
    self.sm.logMonoTime = {'gpsLocation': 2, 'gpsLocationExternal': 1}  # replayed logs are judged by receipt time
    with serving(self.engine, self.response):
      self.engine.update()
      self.assertEqual((self.engine.last_position, self.engine.last_bearing), ((1, 1), 180))  # the newest fix wins
      self.sm.valid['gpsLocation'] = False
      self.engine.update()
      self.assertEqual(self.engine.last_position, (2, 2))  # unless it is invalid
      fixes['gpsLocationExternal'] = SimpleNamespace(hasFix=True, latitude=3, longitude=3, bearingDeg=0)
      self.sm.recv_time = dict.fromkeys(self.sm.valid, time.monotonic() - 3)  # or stale
      self.engine.update()
      self.assertEqual(self.engine.last_position, (2, 2))


class TestBackgroundRouting(unittest.TestCase):
  """navd routes in a worker thread: a corridor download takes minutes, and the 1 Hz loop must keep running meanwhile."""
  A, B = (0.002, 0), (0.004, 0)

  def setUp(self):
    self.pm = MagicMock()
    self.values = {'NavDestination': {'latitude': 0, 'longitude': 0.002}, 'LanguageSetting': 'main_en'}
    self.sm = MagicMock()
    self.sm.valid = dict.fromkeys(('gpsLocation', 'gpsLocationExternal'), True)
    self.sm.logMonoTime = dict.fromkeys(self.sm.valid, time.monotonic_ns())
    self.sm.recv_time = dict.fromkeys(self.sm.valid, time.monotonic())
    self.sm.frame = 1
    self.fix = SimpleNamespace(hasFix=True, latitude=0, longitude=0, bearingDeg=90, speed=10., bearingAccuracyDeg=2.)  # heading east
    self.sm.__getitem__.side_effect = lambda service: self.fix
    params = MagicMock()
    params.get.side_effect = self.values.get
    params.remove.side_effect = lambda key: self.values.pop(key, None)
    self.router = StubRouter(TestRouteEngine.steps())
    self.router.gate = threading.Event()
    self.engine = RouteEngine(self.sm, self.pm, params=params, router=self.router)
    self.addCleanup(self.router.gate.set)

  def tick(self):
    """One navd update, which must not block on the download."""
    t = threading.Thread(target=self.engine.update, daemon=True)
    t.start()
    t.join(2)
    self.assertFalse(t.is_alive(), "navd's update blocked on the route")
    return next(c.args[1] for c in reversed(self.pm.send.call_args_list) if c.args[0] == 'navInstruction')

  def finish(self):
    self.router.gate.set()
    job = self.engine.job
    if job is not None:
      job.join(2)
      self.assertFalse(job.is_alive())

  def test_navd_keeps_publishing_while_the_map_downloads_and_routes_from_where_the_car_is(self):
    for _ in range(3):
      self.assertFalse(self.tick().valid)  # no route yet: invalid, but published every second
    self.assertEqual(len(self.router.calls), 1)
    # the car moved on meanwhile (TurnPilot: a moving fix with a good bearing accuracy, the only heading navd routes with, hold.py)
    self.fix = SimpleNamespace(hasFix=True, latitude=0, longitude=0.0005, bearingDeg=80, speed=10., bearingAccuracyDeg=2.)
    self.tick()
    self.finish()
    self.assertTrue(self.tick().valid)
    self.assertEqual(self.router.positions, [((0.0005, 0), 80)])
    self.assertEqual(self.engine.nav_destination, self.A)

  def test_a_new_destination_during_the_download_cancels_it_and_starts_at_once(self):
    self.tick()
    self.values['NavDestination'] = {'latitude': 0, 'longitude': 0.004}
    self.tick()  # cancels A's download
    deadline = time.monotonic() + 2
    while self.router.calls[-1][1] != self.B and time.monotonic() < deadline:
      self.assertFalse(self.tick().valid)
    self.assertEqual([c[1] for c in self.router.calls], [self.A, self.B])  # B right away, not after RECOMPUTE_HOLD
    self.finish()
    self.assertTrue(self.tick().valid)
    self.assertEqual(self.engine.nav_destination, self.B)

  def test_a_route_for_a_replaced_destination_is_dropped(self):
    self.tick()
    self.finish()  # A's route is done, but navd hasn't picked it up when the destination changes
    self.values['NavDestination'] = {'latitude': 0, 'longitude': 0.004}
    self.router.gate = threading.Event()
    self.assertFalse(self.tick().valid)  # never guidance to A
    self.assertIsNone(self.engine.route)
    self.assertEqual(self.router.calls[-1][1], self.B)
    self.finish()
    self.assertTrue(self.tick().valid)
    self.assertEqual(self.engine.nav_destination, self.B)

  def test_a_new_destination_does_not_wait_for_a_download_that_cannot_stop_yet(self):
    """The server builds a corridor before it answers, and a cancel can't reach the download until then: B starts at once."""
    a_gate, route = threading.Event(), self.router.route
    self.addCleanup(a_gate.set)

    def a_blocks_regardless(start, destination, *args, **kwargs):
      if destination == self.A:
        self.router.calls.append((start, destination))
        a_gate.wait(5)  # deaf to the cancel, like a request still waiting for its status line
        raise MapError('unreachable', 'A failed after it was cancelled')
      return route(start, destination, *args, **kwargs)

    self.router.route = a_blocks_regardless
    self.router.gate = None
    self.tick()
    a_job = self.engine.job
    self.values['NavDestination'] = {'latitude': 0, 'longitude': 0.004}
    self.tick()
    self.assertEqual([c[1] for c in self.router.calls], [self.A, self.B])
    self.assertTrue(a_job.is_alive() and a_job.cancel.is_set())
    self.engine.job.join(2)
    self.assertTrue(self.tick().valid)
    self.assertEqual(self.engine.nav_destination, self.B)
    a_gate.set()
    a_job.join(2)
    self.assertTrue(self.tick().valid)  # A's late failure changes nothing
    self.assertEqual(self.engine.nav_destination, self.B)

  def test_leaving_the_corridor_is_no_guidance_until_the_new_route_and_keeps_the_destination(self):
    """While the new corridor downloads, the old route stays installed. The car is off it: following its distances ran
    through its steps to 'Destination reached', which deleted NavDestination with no error shown."""
    d = 0.01  # about 1.1 km per step along the equator
    steps = [{'geometry': {'coordinates': [[i * d, 0], [i * d + d / 2, 0], [(i + 1) * d, 0]]}, 'distance': 1112., 'duration': 60,
              'bannerInstructions': [{'distanceAlongGeometry': 300, 'primary': {'text': f'Turn {i}', 'type': 'turn', 'modifier': 'right'}}]}
             for i in range(6)]
    self.values['NavDestination'] = {'latitude': 0, 'longitude': 6 * d}
    self.router.steps = steps
    self.fix = SimpleNamespace(hasFix=True, latitude=0, longitude=0.001, bearingDeg=90, speed=30., bearingAccuracyDeg=2.)
    self.tick()
    self.finish()
    self.assertTrue(self.tick().valid)
    self.router.gate = threading.Event()  # the next route call is the new corridor's download: minutes
    calls = len(self.router.calls)
    for t in range(1, 120):  # the car leaves the route northward at 30 m/s
      self.fix = SimpleNamespace(hasFix=True, latitude=t * 30 / 111319.5, longitude=0.001, bearingDeg=0, speed=30., bearingAccuracyDeg=2.)
      msg = self.tick()
      self.assertIn('NavDestination', self.values, f"navd deleted the destination at {t} s")
      if len(self.router.calls) > calls:
        self.assertFalse(msg.valid, f"guidance on the old route at {t} s: {msg.navInstruction.maneuverDistance} m")
        self.assertEqual(self.engine.step_idx, 0)
    self.assertGreater(len(self.router.calls), calls, "navd never rerouted")
    self.fix = SimpleNamespace(hasFix=True, latitude=0, longitude=0.001, bearingDeg=90, speed=30., bearingAccuracyDeg=2.)
    self.finish()
    self.assertFalse(self.tick().valid)  # one invalid instruction between two routes, so no turn desire carries over
    self.assertTrue(self.tick().valid)
    self.assertEqual(self.engine.step_idx, 0)
    self.assertIn('NavDestination', self.values)

  def test_clearing_the_destination_cancels_the_download(self):
    self.tick()
    job = self.engine.job
    del self.values['NavDestination']
    self.assertFalse(self.tick().valid)
    job.join(2)
    self.assertFalse(job.is_alive())
    self.assertIsNone(self.engine.route)


DEG = 1 / METERS_PER_DEGREE  # degrees per m at the equator, in navd's own frame (route.local_meters)


def ll(east: float, north: float) -> tuple:
  """(longitude, latitude) east and north m of (0, 0): synthetic positions, never a drive's."""
  return east * DEG, north * DEG


def heading_to(bearing: float, m: float, origin=(0., 0.)) -> tuple:
  """(east, north) m bearing deg and m from origin (east, north)."""
  return origin[0] + m * np.sin(np.radians(bearing)), origin[1] + m * np.cos(np.radians(bearing))


def road_step(points, text='', mtype='', modifier=''):
  """A route step (Mapbox's shape) along points [(east, north) m], its banner the maneuver at its end (none for the arrival)."""
  coords = [list(ll(*p)) for p in points]
  distance = float(cumulative_distance(np.array(coords))[-1])
  banners = [{'distanceAlongGeometry': min(distance, 200.), 'primary': {'text': text, 'type': mtype, 'modifier': modifier}}] if text else []
  return {'geometry': {'coordinates': coords}, 'distance': distance, 'duration': distance / 10., 'bannerInstructions': banners}


def arrival(point):
  return road_step([point, point])


class TestRouteStartAndReroutes(unittest.TestCase):
  """TurnPilot, the 2026-11-06 drives (1d: 0000001d, 1e: 0000001e), replayed in synthetic geometry around (0, 0): a route
  planned parked with no heading (1e's first left out of the parking spot, the wrong way), a reroute that lands with its first
  turn already behind the car (1e 9:43.4), a right navd couldn't tell was made (1e 8:35.4) or was missed (1e 5:55.4). navd
  steps at 1 Hz with fixes as each tick gives them (no carState: every trusted fix is taken, hold.py)."""

  def setUp(self):
    self.pm = MagicMock()
    self.values = {'NavDestination': {'latitude': 0., 'longitude': 0.01}, 'LanguageSetting': 'main_en'}
    self.sm = MagicMock()
    self.sm.valid = dict.fromkeys(('gpsLocation', 'gpsLocationExternal'), True)
    self.sm.frame = 1
    self.fix = None
    self.sm.__getitem__.side_effect = lambda service: self.fix
    params = MagicMock()
    params.get.side_effect = self.values.get
    params.remove.side_effect = lambda key: self.values.pop(key, None)
    self.router = StubRouter()
    self.engine = RouteEngine(self.sm, self.pm, params=params, router=self.router, background=False)

  def tick(self, place, bearing=90., speed=0., bearing_accuracy=0.):
    """One navd update with a fix at place (east, north m) heading bearing at speed, reporting bearing_accuracy (0: none, so
    no heading: hold.py), received just now. The navInstruction message navd sent."""
    lon, lat = ll(*place)
    self.fix = SimpleNamespace(hasFix=True, longitude=lon, latitude=lat, bearingDeg=bearing, speed=speed, bearingAccuracyDeg=bearing_accuracy)
    self.sm.logMonoTime = dict.fromkeys(self.sm.valid, time.monotonic_ns())
    self.sm.recv_time = dict.fromkeys(self.sm.valid, time.monotonic())
    self.engine.update()
    return next(c.args[1] for c in reversed(self.pm.send.call_args_list) if c.args[0] == 'navInstruction')

  # --- the route's start direction (navd heading_due) ---

  def parking_lot_start(self):
    """1e 0:32.4: the route's first edge heads 122 deg with an 'end of road left' 44 m on; the car is parked 8 m off that
    street, facing about 29 deg, and needs a RIGHT to join it. Returns the car's parked place (east, north m)."""
    start, corner = (0., 0.), heading_to(122., 44.)
    end = heading_to(32., 200., corner)
    self.router.steps = [road_step([start, heading_to(122., 22.), corner], 'End of road left', 'end of road', 'left'),
                         road_step([corner, end], 'Arrive', 'arrive', ''), arrival(end)]
    return heading_to(32., 8.)

  def test_a_route_made_parked_with_no_heading_sends_nothing_until_the_cars_direction_is_checked(self):
    parked = self.parking_lot_start()
    # parked: the u-blox's standstill bearing (295 deg, 45 deg accuracy) is no heading, so the route goes without one
    sent = [self.tick(parked, bearing=295., bearing_accuracy=45.) for _ in range(5)]
    self.assertEqual([c[2] for c in self.router.calls], [None])
    self.assertTrue(self.engine.heading_unverified)
    # invalid, so no 'end of road left' maneuver and no turn desire 40 m out at a crawl (1e: turnLeft at 0.34 m/s)
    self.assertTrue(all(not m.valid for m in sent))
    # creeping out of the spot under HEADING_MIN_V (3 m/s): still no heading, still nothing
    self.assertFalse(self.tick(heading_to(29., 1., parked), bearing=29., speed=1.5, bearing_accuracy=12.).valid)
    # pulling out onto the street the route's way (121 deg against its 122): verified, guidance from here on, no reroute
    sent = [self.tick(heading_to(122., 4. * i, (2., 1.)), bearing=121., speed=4., bearing_accuracy=5.) for i in range(1, 4)]
    self.assertTrue(all(m.valid for m in sent))
    self.assertEqual(str(sent[0].navInstruction.maneuverModifier), 'left')
    self.assertEqual(len(self.router.calls), 1)
    self.assertFalse(self.engine.heading_unverified)

  def test_route_heading_unverified_is_said_on_every_message_while_the_start_waits(self):
    """routeHeadingUnverified (2026-11-06): while the route's start direction waits on its check (heading_unverified), every
    message says so, the invalid ones it withholds guidance on and the separator before a reroute made for the heading, so
    turnpilotd can say 'the route starts once moving' instead of off the route; a verified route, and no route, say false."""
    parked = self.parking_lot_start()
    sent = [self.tick(parked, bearing=295., bearing_accuracy=45.) for _ in range(3)]
    self.assertTrue(all(not m.valid and m.navInstruction.routeHeadingUnverified for m in sent))
    # the reroute for a heading against the route on the street: its separator says the new route is unverified too
    sent = self.tick(heading_to(122., 10.), bearing=349., speed=3., bearing_accuracy=5.6)
    self.assertEqual([c[2] for c in self.router.calls], [None, 349.])
    self.assertFalse(sent.valid)
    self.assertTrue(sent.navInstruction.routeHeadingUnverified and self.engine.heading_unverified)
    # the same route back (Valhalla ignoring the heading): driving it the route's way verifies it, false from then on
    sent = [self.tick(heading_to(122., 10. + 4. * i), bearing=121., speed=4., bearing_accuracy=5.) for i in range(1, 4)]
    self.assertTrue(all(m.valid and not m.navInstruction.routeHeadingUnverified for m in sent))
    # a route made with the car's own course on the street is never unverified, nor is there anything to wait on without one
    self.setUp()
    self.parking_lot_start()
    sent = [self.tick(heading_to(122., 10. + 10. * i), bearing=122., speed=10., bearing_accuracy=2.) for i in range(3)]
    self.assertEqual([c[2] for c in self.router.calls], [122.])
    self.assertTrue(all(m.valid and not m.navInstruction.routeHeadingUnverified for m in sent))
    self.values.pop('NavDestination')
    sent = self.tick(heading_to(122., 40.), bearing=122., speed=10., bearing_accuracy=2.)
    self.assertFalse(sent.valid or sent.navInstruction.routeHeadingUnverified)

  def test_a_lot_heading_across_the_street_decides_nothing_and_never_reroutes(self):
    """Review of 11-06: a heading near-perpendicular to the street says nothing about its direction, and a reroute with it
    let Valhalla (HEADING_TOLERANCE 90) take either again, trusted at once: 'Turn left 30.6 m' went out with the car still
    in the lot. 1e 1:30: out of the spot 107 deg off the route's 122, 13 m off the street: nothing decided, nothing sent."""
    parked = self.parking_lot_start()
    for _ in range(3):
      self.tick(parked, bearing=295., bearing_accuracy=45.)
    off = (122. - 107.) % 360.
    sent = [self.tick(heading_to(off, 5. + 2. * i, parked), bearing=off, speed=3.1, bearing_accuracy=5.4) for i in range(3)]
    self.assertEqual([c[2] for c in self.router.calls], [None])
    self.assertTrue(all(not m.valid for m in sent))
    self.assertTrue(self.engine.heading_unverified)

  def test_a_car_crossing_a_lot_to_an_east_street_waits_then_its_heading_on_the_street_verifies(self):
    """The reviewer's probe: an east street with a left 40 m on, the car 10 m north of it heading 180 (toward it) at 3.2 m/s
    with a good bearing accuracy: no reroute with 180, nothing sent; on the street heading east, verified, the left goes out."""
    start, corner, end = (0., 0.), (40., 0.), (40., 300.)
    self.router.steps = [road_step([start, (20., 0.), corner], 'Turn left', 'turn', 'left'),
                         road_step([corner, end], 'Arrive', 'arrive', ''), arrival(end)]
    for _ in range(3):
      self.tick((5., 14.), bearing=180., bearing_accuracy=45.)
    sent = [self.tick((5., north), bearing=180., speed=3.2, bearing_accuracy=6.) for north in (10., 8., 6.)]
    self.assertTrue(all(not m.valid for m in sent))
    self.assertEqual(len(self.router.calls), 1)
    sent = self.tick((9., 1.), bearing=92., speed=4., bearing_accuracy=4.)  # turned onto the street, the route's way
    self.assertTrue(sent.valid)
    self.assertEqual(str(sent.navInstruction.maneuverModifier), 'left')
    self.assertEqual(len(self.router.calls), 1)
    self.assertFalse(self.engine.heading_unverified)

  def against_on_the_street(self, heading=349., speed=3.0):
    """The parking-lot start, then the car on the street 10 m along it heading `heading` (1e 1:31.4: 349, bacc 5.6, 3.0 m/s,
    133 deg off the route's 122). The message navd sent."""
    parked = self.parking_lot_start()
    for _ in range(3):
      self.tick(parked, bearing=295., bearing_accuracy=45.)
    return self.tick(heading_to(122., 10.), bearing=heading, speed=speed, bearing_accuracy=5.6)

  def test_a_heading_against_the_route_on_the_street_reroutes_once_with_it_and_checks_the_new_route(self):
    """1e 1:31.4: 133 deg off the route on the street: a reroute at once with that heading. The new route is checked too
    (a reroute for the heading may come back the same way): here Valhalla took the car's way, so the update after the
    separator verifies it and guidance goes out, with no third route."""
    route, car = self.router.route, heading_to(122., 10.)

    def the_cars_way(start, destination, bearing, language, position=None, cancelled=None):
      if bearing is not None:  # routed with the car's heading: its road, its way
        corner = heading_to(349., 60., car)
        self.router.steps = [road_step([car, heading_to(349., 30., car), corner], 'Turn right', 'turn', 'right'),
                             road_step([corner, heading_to(79., 300., corner)], 'Arrive', 'arrive', ''), arrival(heading_to(79., 300., corner))]
      return route(start, destination, bearing, language, position=position, cancelled=cancelled)
    self.router.route = the_cars_way
    sent = self.against_on_the_street()
    self.assertEqual([c[2] for c in self.router.calls], [None, 349.])
    self.assertFalse(sent.valid)  # the separator between the two routes
    self.assertTrue(self.engine.heading_unverified and self.engine.route_heading_retry)
    sent = [self.tick(heading_to(349., 3. * i, car), bearing=349., speed=3.5, bearing_accuracy=5.) for i in range(1, 4)]
    self.assertTrue(all(m.valid for m in sent))
    self.assertEqual(str(sent[0].navInstruction.maneuverModifier), 'right')
    self.assertFalse(self.engine.heading_unverified)
    self.assertEqual(len(self.router.calls), 2)

  def test_a_heading_reroute_that_comes_back_the_same_way_is_tried_again_only_once_per_recompute_hold(self):
    """Valhalla ignoring the heading (or a one-way the car heads against) brings back the same route: still unverified,
    nothing sent, and the next reroute for it after RECOMPUTE_HOLD, never one every update."""
    self.against_on_the_street(heading=302., speed=3.5)  # driving back down the street, 180 deg off
    self.assertEqual([c[2] for c in self.router.calls], [None, 302.])
    sent = [self.tick(heading_to(122., 9. - 0.5 * i), bearing=302., speed=3.5, bearing_accuracy=5.) for i in range(1, RECOMPUTE_HOLD)]
    self.assertTrue(all(not m.valid for m in sent))
    self.assertEqual(len(self.router.calls), 2)
    self.assertTrue(self.engine.heading_unverified)
    self.tick(heading_to(122., 4.), bearing=302., speed=3.5, bearing_accuracy=5.)
    self.assertEqual(len(self.router.calls), 3)

  def test_a_heading_against_the_route_anywhere_within_off_route_distance_reroutes(self):
    """Reversed is informative however far into OFF_ROUTE_DISTANCE: 15 m off the street (a wide road's far side), 180 deg off
    the route's way, a reroute with it; VERIFY_LATERAL_M bounds only verification."""
    parked = self.parking_lot_start()
    for _ in range(3):
      self.tick(parked, bearing=295., bearing_accuracy=45.)
    self.tick(heading_to(32., 15., heading_to(122., 20.)), bearing=302., speed=3.5, bearing_accuracy=5.)
    self.assertEqual([c[2] for c in self.router.calls], [None, 302.])

  def test_a_reroute_made_while_the_route_is_unverified_is_unverified_too(self):
    """A car driving round a lot more than REROUTE_DISTANCE off the route is rerouted (should_recompute) with its lot heading:
    that route is checked like the one it replaced, never trusted for having a heading (the reviewer's probe, by another
    road). On the street the route's way, it is verified."""
    parked = self.parking_lot_start()
    self.tick(parked, bearing=295., bearing_accuracy=45.)
    for i in range(RECOMPUTE_HOLD + REROUTE_COUNTER_MIN):
      self.tick(heading_to(32., 30. + i), bearing=32., speed=3.5, bearing_accuracy=5.)
    self.assertEqual([c[2] for c in self.router.calls], [None, 32.])
    self.assertTrue(self.engine.heading_unverified)
    self.assertFalse(self.engine.route_heading_retry)
    self.assertFalse(self.tick(heading_to(32., 20.), bearing=212., speed=3.5, bearing_accuracy=5.).valid)  # back across the lot
    self.assertTrue(self.tick(heading_to(122., 6.), bearing=120., speed=4., bearing_accuracy=5.).valid)
    self.assertEqual(len(self.router.calls), 2)

  def test_a_wide_roads_far_lane_verifies_at_speed_and_waits_at_a_crawl(self):
    """14 m off the step's mapped centre line, heading its way: at a crawl that is a parking row's place too, so it waits; at
    VERIFY_STREET_V it is the street. (carState given here: vEgo is the car's speed.)"""
    from openpilot.selfdrive.navigation.hold import Motion
    self.router.steps = [road_step([(0., 0.), (100., 0.), (300., 0.)], 'Turn right', 'turn', 'right'),
                         road_step([(300., 0.), (300., -300.)], 'Arrive', 'arrive', ''), arrival((300., -300.))]
    speed = {'v': 0.}
    with patch('openpilot.selfdrive.navigation.navd.car_motion', lambda sm: Motion(speed['v'], None)):
      self.tick((0., 14.), bearing=295., bearing_accuracy=45.)
      self.assertTrue(self.engine.heading_unverified)
      speed['v'] = 4.
      self.assertFalse(self.tick((4., 14.), bearing=90., speed=4., bearing_accuracy=4.).valid)
      self.assertTrue(self.engine.heading_unverified)
      speed['v'] = 9.  # 20 mph (VERIFY_STREET_V 8)
      self.assertTrue(self.tick((12., 14.), bearing=90., speed=9., bearing_accuracy=3.).valid)
    self.assertEqual(len(self.router.calls), 1)

  def test_standing_a_heading_only_verifies(self):
    """Standing, the hold's heading is the car's own, turned with its yaw (hold.py): pointing the route's way on the street it
    verifies; nose-in in a space, 90 deg off, decides nothing; reversed, nothing either (a car standing never reroutes for
    its heading: it may yet reverse out)."""
    from openpilot.selfdrive.navigation.hold import Motion
    with patch('openpilot.selfdrive.navigation.navd.car_motion', lambda sm: Motion(0., None)):
      for place, heading, verified in (('parked', 32., False), (heading_to(122., 10.), 302., False), (heading_to(122., 10.), 121., True)):
        with self.subTest(heading=heading):
          self.setUp()
          parked = self.parking_lot_start()
          place = parked if place == 'parked' else place
          for _ in range(2):
            self.tick(place, bearing=heading, bearing_accuracy=45.)
          self.engine.hold.heading = heading  # as an earlier drive's moving fixes and the car's yaw since left it
          self.assertTrue(self.engine.hold.standing)
          self.assertEqual(self.tick(place, bearing=heading, bearing_accuracy=45.).valid, verified)
          self.assertEqual(self.engine.heading_unverified, not verified)
          self.assertEqual(len(self.router.calls), 1)

  def test_a_heading_within_the_tolerance_keeps_the_route_and_a_wrong_one_far_off_it_waits(self):
    parked = self.parking_lot_start()
    self.tick(parked, bearing=295., bearing_accuracy=45.)
    # 30 m off the street (a car park beside it): its heading says nothing about the street's direction, so nothing is decided
    self.tick(heading_to(32., 30.), bearing=300., speed=4., bearing_accuracy=5.)
    self.assertTrue(self.engine.heading_unverified)
    self.assertEqual(len(self.router.calls), 1)
    # on the street, 1 deg off the route's bearing: kept
    self.assertTrue(self.tick(heading_to(122., 10.), bearing=123., speed=4., bearing_accuracy=5.).valid)
    self.assertEqual(len(self.router.calls), 1)

  def test_with_no_heading_ever_travel_along_the_route_verifies_it_and_travel_against_it_never(self):
    """A receiver that reports no bearing accuracy gives no heading at all: VERIFY_TRAVEL_M along the step on it verifies the
    route, so guidance isn't withheld for the drive; driving back down the step the wrong way never does."""
    self.router.steps = [road_step([(0., 0.), (100., 0.), (200., 0.)], 'Turn right', 'turn', 'right'),
                         road_step([(200., 0.), (200., -300.)], 'Arrive', 'arrive', ''), arrival((200., -300.))]
    sent = [self.tick((60. - 5. * i, 2.), bearing=270., speed=5.) for i in range(6)]  # 25 m back toward the route's start
    self.assertTrue(all(not m.valid for m in sent) and self.engine.heading_unverified)
    sent = [self.tick((35. + 5. * i, 2.), bearing=90., speed=5.) for i in range(1, 5)]  # then 20 m the route's way
    self.assertEqual([m.valid for m in sent], [False, False, True, True])  # valid from 15 m on
    self.assertEqual(len(self.router.calls), 1)

  def test_a_route_made_with_a_heading_is_never_held_back(self):
    self.router.steps = [road_step([(0., 0.), (200., 0.)], 'Turn right', 'turn', 'right'), arrival((200., 0.))]
    self.assertTrue(self.tick((0., 1.), bearing=90., speed=10., bearing_accuracy=3.).valid)
    self.assertFalse(self.engine.heading_unverified)

  def test_a_download_that_routes_from_a_later_moving_fix_routes_with_its_heading(self):
    # the router routes from position() after a download (sources.py: route_server); that bearing is the route's heading
    job_steps = [road_step([(0., 0.), (200., 0.)], 'Turn right', 'turn', 'right'), arrival((200., 0.))]
    self.router.steps = job_steps
    route = self.router.route

    def downloading(start, destination, bearing, language, position=None, cancelled=None):
      self.assertIsNone(bearing)  # the job started with the car standing
      self.tick_fix((10., 1.), bearing=91., speed=8., bearing_accuracy=3.)  # meanwhile it drove off east
      return route(start, destination, bearing, language, position=position, cancelled=cancelled)
    self.router.route = downloading
    self.tick_fix((0., 1.), bearing=200., bearing_accuracy=40.)
    self.engine.update()
    self.assertFalse(self.engine.heading_unverified)

  def tick_fix(self, place, bearing=90., speed=0., bearing_accuracy=0.):
    """The fix tick() would give, and the hold (and what its heading rests on, navd's note_basis) updated with it, without the
    rest of navd's update."""
    from openpilot.selfdrive.navigation.hold import car_motion, gps_fix
    lon, lat = ll(*place)
    self.fix = SimpleNamespace(hasFix=True, longitude=lon, latitude=lat, bearingDeg=bearing, speed=speed, bearingAccuracyDeg=bearing_accuracy)
    self.sm.logMonoTime = dict.fromkeys(self.sm.valid, time.monotonic_ns())
    self.sm.recv_time = dict.fromkeys(self.sm.valid, time.monotonic())
    fix, motion, now = gps_fix(self.sm), car_motion(self.sm), time.monotonic()
    self.engine.hold.update(fix, motion, now)
    self.engine.note_basis(fix, motion, now)
    self.engine.last_position = self.engine.hold.position

  # --- a reroute that lands stale (navd collect_route, stale_start) ---

  def stale_route(self, first_turn_m, text='Turn right', mtype='turn', modifier='right'):
    """A route east from (0, 0) to a turn first_turn_m on, then south to the end."""
    corner = (first_turn_m, 0.)
    self.router.steps = [road_step([(0., 0.), (first_turn_m / 2., 0.), corner], text, mtype, modifier),
                         road_step([corner, (first_turn_m, -300.)], 'Arrive', 'arrive', ''), arrival((first_turn_m, -300.))]

  def route_while_driving(self, travelled, v_ego=11.):
    """navd routes from (0, 0) at v_ego; while each route job runs the car drives on: to travelled[i] m east by the time the
    i-th route comes back (1e: 4-7 s, 37-101 m, from 'Calculating route' to the first instruction)."""
    self.engine.last_position, self.engine.last_bearing, self.engine.v_ego = ll(0., 0.), 90., v_ego
    route, places = self.router.route, list(travelled)

    def driving(*args, **kwargs):
      if places:
        self.engine.last_position = ll(places.pop(0), 0.)
      return route(*args, **kwargs)
    self.router.route = driving
    self.engine.calculate_route((0.01, 0.))

  def test_a_reroute_whose_first_right_is_already_behind_the_car_is_routed_again_once_then_installed(self):
    """1e 9:39.4-9:43.4: the reroute's job started 38 m back; it landed with its first right 4.8 m behind the car, skipped
    tooLate and rerouted again a second later. navd routes it again at once from where the car is, and the second result
    goes in as it is (here the stub's same route, its right still behind: never a third)."""
    self.stale_route(30.)
    self.route_while_driving([34.8, 78.8])
    self.assertEqual(len(self.router.calls), 2)
    self.assertEqual(self.router.calls[0][0], ll(0., 0.))
    self.assertEqual(self.router.calls[1][0], ll(34.8, 0.))  # the retry from the live position
    self.assertIsNone(self.engine.job)
    self.assertEqual((self.engine.route_count, self.engine.step_idx), (2, 0))

  def test_a_first_turn_too_close_at_speed_is_stale_and_one_far_enough_or_a_keep_or_standing_is_not(self):
    """1e rc=13 (11:19.4): its first right 37 m on at 12 m/s, too close to make. Stale under max(25 m, 3 s at the car's
    speed) when its first instruction goes out, a second after the route lands (the separator's update); a keep (no junction
    turn), a car that didn't move while it routed (the same route would come back) and a car standing (a route made at a red
    light: the turn is near, not stale) are installed as they are."""
    # (first turn m from the route's start, m the car drove while it routed, v_ego, banner, standing, routed again)
    for name, first, travelled, v_ego, banner, standing, again in (
        ('20 m ahead at 11 m/s', 58., 38., 11., ('Turn right', 'turn', 'right'), False, True),
        ('40 m ahead at 11 m/s: 29 m, under 3 s, a second on', 78., 38., 11., ('Turn left', 'turn', 'left'), False, True),
        ('50 m ahead at 11 m/s: 39 m a second on', 88., 38., 11., ('Turn right', 'turn', 'right'), False, False),
        ('30 m ahead at 4 m/s: 26 m a second on', 68., 38., 4., ('Turn right', 'turn', 'right'), False, False),
        ('20 m ahead at 5 m/s, a slight-right keep', 58., 38., 5., ('Keep right', 'fork', 'slight right'), False, False),
        ('20 m ahead, not having moved while it routed', 20., 0., 11., ('Turn right', 'turn', 'right'), False, False),
        ('standing at a red 20 m short', 58., 38., 0., ('Turn right', 'turn', 'right'), True, False)):
      with self.subTest(name):
        self.setUp()
        self.stale_route(first, *banner)
        self.engine.hold.standing = standing
        self.route_while_driving([travelled, travelled], v_ego=v_ego)
        self.assertEqual(len(self.router.calls), 2 if again else 1)
        self.assertEqual(self.engine.route_count, 2 if again else 1)

  # --- the step's end: a turn made or missed, by the car's heading (navd turned_onto_next, went_straight_past) ---

  def wide_right(self):
    """1e 8:16-8:36: a right from a wide road's curb lane. The step runs east to the junction at (0, 0), its last segment
    13.3 m, and the next one south. At the 1e moment the car is 20.7 m off the step at -8.5 m along and 6.4 m off the next
    step's line (its right lane), heading the exit road."""
    self.router.steps = [road_step([(-200., 0.), (-13.33, 0.), (0., 0.)], 'Turn right', 'turn', 'right'),
                         road_step([(0., 0.), (0., -400.)], 'Arrive', 'arrive', ''), arrival((0., -400.))]
    self.assertTrue(self.tick((-100., -3.), bearing=90., speed=8., bearing_accuracy=2.).valid)
    return (-6.4, -20.7)

  def test_a_right_made_on_a_wide_road_moves_navd_on_by_the_cars_heading(self):
    after_turn = self.wide_right()
    g0, g1 = self.engine.route_geometry[:2]
    # the 1e geometry: past OFF_ROUTE_DISTANCE off the step at -8.5 m along, and past NEXT_STEP_NEAR off the next one
    self.assertAlmostEqual(along_path(g0, ll(*after_turn))[1], 20.7, delta=0.05)
    self.assertAlmostEqual(self.engine.route[0]['distance'] - along_path(g0, ll(*after_turn))[0], -8.5, delta=0.1)
    self.assertAlmostEqual(along_path(g1, ll(*after_turn))[1], 6.4, delta=0.05)
    sent = [self.tick((-20., -4.), bearing=95., speed=5., bearing_accuracy=2.),
            self.tick((-9., -8.), bearing=125., speed=4., bearing_accuracy=2.),    # turning, not onto the exit road yet
            self.tick(after_turn, bearing=181., speed=3.8, bearing_accuracy=2.1)]  # on it (1e: 15 deg against its 16)
    self.assertEqual([m.valid for m in sent], [True, True, True])  # never the invalid blip turnpilotd dropped the turn on
    self.assertEqual([self.engine.step_idx, str(sent[1].navInstruction.maneuverModifier)], [1, 'right'])
    self.assertEqual(str(sent[-1].navInstruction.maneuverType), 'arrive')
    self.assertEqual(len(self.router.calls), 1)

  def test_without_a_heading_the_wide_right_goes_invalid_as_before(self):
    after_turn = self.wide_right()
    self.tick((-20., -4.), bearing=95., speed=5., bearing_accuracy=2.)
    self.engine.hold.heading = None  # no heading (a dropout ended it), and these fixes report no bearing accuracy
    self.assertFalse(self.tick(after_turn, bearing=181., speed=3.8).valid)  # navd can only go by the geometry
    self.assertEqual(self.engine.step_idx, 0)

  def test_straight_through_a_right_is_a_missed_turn_rerouted_at_once_never_the_next_step(self):
    """1e 5:49-5:56: the car went straight through the right (GPS 127-129, the step 128, the next road 201); navd moved on
    at -10.4 m to the next step's left 80 m on a road the car wasn't on, and turnpilotd armed an unprotected left for 1 s."""
    self.wide_right()
    self.tick((-5., -3.), bearing=90., speed=9., bearing_accuracy=2.)
    sent = self.tick((10.4, -3.), bearing=89., speed=9., bearing_accuracy=2.)  # just past the junction, straight on
    self.assertLess(sent.navInstruction.maneuverDistance, -10.)
    self.assertEqual(self.engine.step_idx, 0)  # not moved on
    self.assertTrue(self.engine.missed)
    self.assertEqual(len(self.router.calls), 1)
    self.tick((19.4, -3.), bearing=90., speed=9., bearing_accuracy=2.)  # the next update: rerouted from where the car is
    self.assertEqual(len(self.router.calls), 2)
    self.assertEqual(self.router.calls[1][0], ll(19.4, -3.))
    self.assertFalse(self.engine.missed)

  def test_without_a_heading_past_the_end_navd_moves_on_as_before(self):
    self.wide_right()
    self.tick((-5., -3.), bearing=90., speed=9., bearing_accuracy=2.)
    self.engine.hold.heading = None
    self.tick((10.4, -3.), bearing=90., speed=9.)  # no bearing accuracy: no heading
    self.assertEqual(self.engine.step_idx, 1)
    self.assertFalse(self.engine.missed)

  def test_a_slight_turn_is_never_moved_on_before_its_junction(self):
    # a slight right (20 deg): the car still on the approach heads within 30 deg of the next road, but hasn't left its own
    corner = (0., 0.)
    self.router.steps = [road_step([(-200., 0.), corner], 'Turn slight right', 'turn', 'slight right'),
                         road_step([corner, heading_to(110., 300.)], 'Arrive', 'arrive', ''), arrival(heading_to(110., 300.))]
    self.tick((-100., 0.), bearing=90., speed=8., bearing_accuracy=2.)
    sent = self.tick((-6., 0.5), bearing=95., speed=8., bearing_accuracy=2.)
    self.assertEqual(self.engine.step_idx, 0)
    self.assertAlmostEqual(sent.navInstruction.maneuverDistance, 6., delta=0.5)

  def test_chord_bearings_of_a_step(self):
    g = np.array([ll(0., 0.), ll(1., 1.), ll(1., 30.)])  # a 1.4 m stub north-east, then north
    self.assertAlmostEqual(chord_bearing(g, from_end=False), np.degrees(np.arctan2(1., 1. + 20. - np.hypot(1., 1.))), delta=1e-6)
    self.assertAlmostEqual(chord_bearing(g, from_end=True), 0., delta=1e-6)
    self.assertIsNone(chord_bearing(np.array([ll(5., 5.), ll(5., 5.)]), from_end=True))
    along, lateral, bearing = step_place(g, ll(3., 10.))
    self.assertAlmostEqual(lateral, 2., delta=0.01)
    self.assertAlmostEqual(bearing, 0., delta=1e-6)
    self.assertAlmostEqual(along, np.hypot(1., 1.) + 9., delta=0.01)


class TestSlightLeftAcrossTheOpposingRoad(unittest.TestCase):
  """The rights review (2026-11-06): navd's maneuverOpposing present is any other road within 45 deg of straight on, so a main
  road bending 28 deg left past a lone side road 40 deg to the right read present, and route_follower's slight_far_turn made
  the slight left a TURN: an autonomous unprotected left held at a line on a through road, handed back. A cone on the
  opposing road's leg off straight on was to tell the two apart, which navd never sent. navd now sends it
  (maneuverOpposingRelBearing, and each allManeuvers entry's opposingRelBearing: routing/guidance.py), and RouteFollower reads
  it off navd's own message beside that road's mapped speed (maneuverOpposingSpeed, opposingSpeed) and the approach's
  (speedLimit). By its bearing alone the road off the far side was more often the approach's own road going on, whose
  oncoming traffic the left does cross (review of 11-06: 5 of the 6 keeps the bare cone made on the local tiles around the
  11-06 drives, every one at the approach's own mapped speed; routing/guidance.py docstring), so the cone is gone
  (2026-11-07): the slight left across the road straight ahead, one on its own side (1e seg 5, 1d seg 9: 29 and 38 deg lefts
  with an oncoming approach) or one off the far side at the approach's speed is a TURN, on navd's message and on the look
  ahead. navd on a route of synthetic geometry around (0, 0), the car on the street east of the junction with a good course
  (routed with it: valid)."""

  setUp = TestRouteStartAndReroutes.setUp
  tick = TestRouteStartAndReroutes.tick

  def route(self, other: float, bend_first: bool = False):
    """East 150 m to a slight left onto 62 deg (28 deg left) at a junction with one other road leaving at `other` deg, then the
    arrival; bend_first: a 'continue straight' at 60 m first (no other road: NONE), so the slight left is allManeuvers[1], the
    maneuver RouteFollower looks ahead to past it."""
    corner, end = (150., 0.), heading_to(62., 200., (150., 0.))
    junction = {'type': 'turn', 'modifier': 'slight left', 'bearing_before': 90., 'bearing_after': 62.}
    steps = [road_step([(0., 0.), (75., 0.), corner], 'Bear left', 'turn', 'slight left'), road_step([corner, end], 'Arrive', 'arrive', ''),
             arrival(end)]
    steps[1].update(maneuver=junction, intersections=[{'location': list(ll(*corner)), 'bearings': [62., 270., other],
                                                       'entry': [True, False, True], 'in': 1, 'out': 0}])
    if bend_first:
      first, rest = road_step([(0., 0.), (60., 0.)], 'Continue', 'continue', 'straight'), road_step([(60., 0.), (105., 0.), corner],
                                                                                                    'Bear left', 'turn', 'slight left')
      rest.update(maneuver={'type': 'continue', 'modifier': 'straight', 'bearing_before': 90., 'bearing_after': 90.},
                  intersections=[{'location': list(ll(60., 0.)), 'bearings': [90., 270.], 'entry': [True, False], 'in': 1, 'out': 0}])
      steps = [first, rest] + steps[1:]
    self.router.steps = steps

  def follow(self):
    """navd's messages for the car driving east 10 m/s on a good course from 20 m to 50 m, each into a RouteFollower as
    turnpilotd gives them: the follower and the last message."""
    from openpilot.selfdrive.turnpilot.route_follower import RouteFollower
    rf = RouteFollower()
    for east in (20., 30., 40., 50.):
      sent = self.tick((east, 0.), bearing=90., speed=10., bearing_accuracy=2.)
      rf.update(sent.navInstruction, True, sent.valid, 10., 1.)
    self.assertTrue(sent.valid)
    return rf, sent.navInstruction

  def test_the_opposing_roads_bearing_and_speed_reach_the_follower(self):
    import math
    from openpilot.selfdrive.turnpilot.route_follower import LEFT, NONE, TURN, make_maneuver
    limit = 13.41  # m/s, 30 mph: the approach's and the opposing road's alike, a through road's
    # off the far side 40 deg (the approach's own road going on at its speed: the class docstring), ahead, the turn's side
    for other, rel, kind in ((130., 40., TURN), (95., 5., TURN), (75., -15., TURN), (45., -45., TURN)):
      with self.subTest(other=other):
        self.setUp()
        self.route(other)
        self.router.opposing_result, self.router.limits_result = {0: (limit, 2)}, [(1000., limit)]
        rf, ni = self.follow()
        self.assertEqual(str(ni.maneuverOpposing), 'present')
        self.assertAlmostEqual(ni.maneuverOpposingRelBearing, rel, places=3)
        self.assertAlmostEqual(ni.allManeuvers[0].opposingRelBearing, rel, places=3)
        self.assertAlmostEqual(ni.maneuverOpposingSpeed, limit, places=3)  # the road's speed and the approach's, on one message
        self.assertAlmostEqual(ni.speedLimit, limit, places=3)
        m = rf.upcoming
        self.assertEqual((m.side, m.type, m.modifier), (LEFT, 'turn', 'slight left'))
        self.assertEqual(m.kind, kind)
        self.assertAlmostEqual(m.angle_deg, 28.)
        self.assertAlmostEqual(m.opposing_rel_deg, rel, places=3)
        self.assertAlmostEqual(m.opposing_speed, limit, places=3)
        # past a 'continue straight' first, the slight left is RouteFollower's look ahead (allManeuvers[1]): the same reading
        self.setUp()
        self.route(other, bend_first=True)
        self.router.opposing_result, self.router.limits_result = {1: (limit, 2)}, [(1000., limit)]
        rf, ni = self.follow()
        self.assertEqual(rf.upcoming.kind, NONE)
        self.assertAlmostEqual(ni.allManeuvers[1].opposingRelBearing, rel, places=3)
        self.assertAlmostEqual(ni.allManeuvers[1].opposingSpeed, limit, places=3)
        self.assertEqual(rf.ahead.side, LEFT)
        self.assertEqual(rf.ahead.kind, kind)
        self.assertAlmostEqual(rf.ahead.opposing_rel_deg, rel, places=3)
        self.assertAlmostEqual(rf.ahead.opposing_speed, limit, places=3)
    # what the car did before: no bearing (a navd before the field: NaN, not known), present alone decides, a TURN past the road off the far side
    self.assertEqual(make_maneuver('turn', 'slight left', (), True, 90., 62., 1, 'present', True, None).kind, TURN)
    # an invalid message (the separator, a route not yet verified) carries none: NaN, never a road dead ahead
    self.setUp()
    self.route(130.)
    sent = self.tick((20., 0.), bearing=295., bearing_accuracy=45.)  # standing with no course: routed with none, unverified
    self.assertFalse(sent.valid)
    self.assertTrue(math.isnan(sent.navInstruction.maneuverOpposingRelBearing))


class TestParkedOnroadHeading(unittest.TestCase):
  """Review of 11-06, the residual of the route-start fix: the occupied Tesla can park nose-in and stay onroad, so carState and
  deviceMotion stay live and the hold's heading, turned with the car's yaw through the manoeuvre, is its orientation in the
  space: perpendicular to the street. A destination set then was routed with it as a known heading and never checked (1d
  ended standing 80 s at 82 deg from its last GPS course). navd routes with the hold's heading only while it rests on a GPS
  course the car has turned less than COURSE_TURN_MAX_DEG from since (navd HeadingBasis), so a red light after driving
  straight keeps its heading (the 11-06 stands: at most 11.9 deg). navd itself on navd_loop's synthetic route (east 300 m to a
  left) with carState and deviceMotion, no deviceState: onroad throughout, as the occupied car is."""

  def loop(self):
    from openpilot.selfdrive.turnpilot.tests.navd_loop import NavdLoop
    return NavdLoop()

  @staticmethod
  def into_a_space(loop, s=100.):
    """Driven east at 10 m/s on good GPS to s, then a right at 2 m/s (under HEADING_MIN_V, so no GPS course) on a 5 m radius
    into a space beside the street, nose-in 8 m south of it, heading 180, and standing there: the last tick's instruction."""
    from openpilot.selfdrive.navigation.hold import HEADING_MIN_V
    for i in range(int(s / 10.)):
      loop.tick(10. * i, 10.)
    assert 2. < HEADING_MIN_V
    for turned in (20., 40., 60., 80., 90.):
      a = np.radians(turned)
      loop.tick(s, 2., fix=(5. * np.sin(a), -5. + 5. * np.cos(a), 3.), car_heading=90. + turned)
    loop.tick(s, 2., fix=(5., -8., 3.), car_heading=180.)
    out = None
    for _ in range(5):
      out = loop.tick(s, 0., fix=(5., -8., 3.), car_heading=180.)
    return out

  def test_the_course_test_is_the_holds_own(self):
    """hold.py's course_good is the one course test (review of 11-06: navd had mirrored it as course_taken, kept in step by
    this test alone): the hold's heading becomes the fix's bearing exactly when it says so, and navd's HeadingBasis starts a
    course (its turn since reset, routable again) on the very same updates, at each edge (HEADING_MIN_V,
    HEADING_ACCURACY_MAX_DEG, none reported, no carState, a NaN bearing, a poor fix, no fix)."""
    from openpilot.selfdrive.navigation import navd
    from openpilot.selfdrive.navigation.hold import (GPS_ACCURACY_MAX_M, HEADING_ACCURACY_MAX_DEG, HEADING_MIN_V, Fix, Motion, PositionHold,
                                                     course_good)
    from openpilot.selfdrive.navigation.navd import HeadingBasis
    self.assertFalse(hasattr(navd, 'course_taken'))  # no copy to drift from it
    cases = [(Fix((0., 0.), 200., 3., 0., 5.), Motion(HEADING_MIN_V, None)),
             (Fix((0., 0.), 200., 3., 10., 5.), Motion(HEADING_MIN_V - 0.01, None)),
             (Fix((0., 0.), 200., 3., 10., HEADING_ACCURACY_MAX_DEG), Motion(10., None)),
             (Fix((0., 0.), 200., 3., 10., HEADING_ACCURACY_MAX_DEG + 0.01), Motion(10., None)),
             (Fix((0., 0.), 200., 3., 10., 0.), Motion(10., None)),
             (Fix((0., 0.), 560., 3., HEADING_MIN_V, 5.), None),
             (Fix((0., 0.), 200., 3., HEADING_MIN_V - 0.01, 5.), None),
             (Fix((0., 0.), float('nan'), 3., 10., 5.), Motion(10., None)),
             (Fix((0., 0.), 200., GPS_ACCURACY_MAX_M + 1., 10., 5.), Motion(10., None)),
             (Fix((0., 0.), 200., 0., 10., 5.), Motion(10., None)),
             (None, Motion(10., None))]
    taken = []
    for fix, motion in cases:
      hold = PositionHold()
      hold.update(Fix((0., 0.), 10., 3., 10., 5.), Motion(10., None), 0.)
      self.assertEqual(hold.heading, 10.)
      hold.update(fix, motion, 1.)
      self.assertEqual(hold.heading != 10., course_good(fix, motion), (fix, motion))
      # HeadingBasis on a course the car then turned 40 deg from at a crawl (not routable), given this update's fix and motion
      # and the heading the hold left: a course again (routable) exactly when the hold took the fix's bearing
      basis = HeadingBasis()
      basis.update(10., Fix((0., 0.), 10., 3., 10., 5.), Motion(10., None), 0.)
      basis.update(50., None, Motion(1., None), 1.)
      self.assertFalse(basis.routable)
      basis.update(hold.heading, fix, motion, 2.)
      self.assertEqual(basis.routable, course_good(fix, motion), (fix, motion))
      taken.append(course_good(fix, motion))
    self.assertEqual(taken, [True, False, True, False, False, True, False, False, False, True, False])

  def test_the_heading_rests_on_a_course_barely_turned_from_since(self):
    """HeadingBasis: the furthest it turned from its course, not the net (a turn in and back out), a new course starts over,
    no heading (stale, or dropped parked) ends it, and COURSE_UNTRACKED_M driven with no yaw does too."""
    from openpilot.selfdrive.navigation.hold import Fix, Motion
    from openpilot.selfdrive.navigation.navd import COURSE_TURN_MAX_DEG, COURSE_UNTRACKED_M, HeadingBasis
    course, crawl = Fix((0., 0.), 90., 3., 10., 5.), Fix((0., 0.), 90., 3., 1., 0.)
    basis, t = HeadingBasis(), 0.
    self.assertFalse(basis.routable)
    self.assertEqual(basis.why_not(), 'noCourse')

    def step(heading, fix=crawl, v=1., yaw=0.):
      nonlocal t
      t += 1.
      basis.update(heading, fix, Motion(v, yaw), t)
      return basis.routable
    self.assertTrue(step(90., course, 10.))
    self.assertTrue(step(90. + COURSE_TURN_MAX_DEG - 1.))
    self.assertFalse(step(90. + COURSE_TURN_MAX_DEG + 1.))
    self.assertEqual(basis.why_not(), 'turned')
    self.assertFalse(step(90.))  # turned back: still turned at its furthest
    self.assertAlmostEqual(basis.turn, 0.)
    self.assertTrue(step(95., course, 10.))  # a new course
    self.assertTrue(step(85. + 360.))  # wraps: -10 deg net
    self.assertAlmostEqual(basis.turn, -10.)
    self.assertFalse(step(None))
    self.assertFalse(step(95.))  # not a course's: what turned it is unknown
    self.assertTrue(step(95., course, 10.))
    for _ in range(int(COURSE_UNTRACKED_M / 0.5) - 1):
      self.assertTrue(step(95., v=0.5, yaw=None))  # crawling with no deviceMotion: nothing turns the heading
    self.assertFalse(step(95., v=0.5, yaw=None))
    self.assertEqual(basis.why_not(), 'untracked')

  def test_parked_nose_in_onroad_a_destination_is_routed_with_no_heading_and_waits_for_the_cars_direction(self):
    loop = self.loop()
    looked, controls_ahead = [], loop.router.controls_ahead
    loop.router.controls_ahead = lambda position, bearing: looked.append(bearing) or controls_ahead(position, bearing)
    self.into_a_space(loop)
    engine = loop.engine
    self.assertEqual(len(loop.router.calls), 1)  # the drive's route, made moving with its heading
    self.assertAlmostEqual(loop.router.calls[0][1], 90.)
    self.assertTrue(engine.onroad() and engine.hold.standing)
    self.assertAlmostEqual(engine.own_heading(), 180., delta=0.5)  # the car's orientation in the space
    self.assertGreaterEqual(engine.basis.peak, 89.)
    self.assertIsNone(engine.route_heading())
    self.assertEqual(engine.heading_dropped(), 'turned')
    # a new destination set on the phone in the space: routed with no heading (the earlier code routed with 180, trusted it,
    # and sent valid guidance at once), and nothing goes out standing there
    loop.values['NavDestination'] = {'latitude': 0.003, 'longitude': 0.004}
    loop.tick(100., 0., fix=(5., -8., 3.), car_heading=180.)
    self.assertEqual(len(loop.router.calls), 2)
    self.assertIsNone(loop.router.calls[1][1])
    self.assertTrue(engine.heading_unverified)
    self.assertTrue(all(not loop.tick(100., 0., fix=(5., -8., 3.), car_heading=180.)[1] for _ in range(5)))
    self.assertAlmostEqual(looked[-1], 180., delta=0.5)  # the road ahead of the car's nose is still looked up with its own
    # out of the space at a crawl, turning onto the street the route's way: 75 deg off it decides nothing; 40 deg off, 3 m off
    # its line, the car's own heading verifies it (no GPS course yet, so navd would still route with none)
    self.assertFalse(loop.tick(100., 1.5, fix=(6., -6., 3.), car_heading=165.)[1])
    self.assertTrue(engine.heading_unverified)
    self.assertTrue(loop.tick(100., 1.5, fix=(8., -3., 3.), car_heading=130.)[1])
    self.assertFalse(engine.heading_unverified)
    self.assertIsNone(engine.route_heading())
    ni, valid = loop.tick(112., 3.5, car_heading=95.)  # away on the street: a course again
    self.assertTrue(valid)
    self.assertAlmostEqual(engine.route_heading(), 95.)
    self.assertEqual(len(loop.router.calls), 2)

  def test_out_of_the_space_against_the_route_waits_for_a_course_and_then_reroutes_with_it(self):
    """Leaving west, against the route: at a crawl the car's heading the space turned decides nothing (navd would route with
    none); its first GPS course on the street reroutes with it."""
    loop = self.loop()
    self.into_a_space(loop)
    engine = loop.engine
    loop.values['NavDestination'] = {'latitude': 0.003, 'longitude': 0.004}
    loop.tick(100., 0., fix=(5., -8., 3.), car_heading=180.)
    self.assertTrue(engine.heading_unverified)
    for turned in (30., 60., 90.):  # a right out of the space onto the street, heading west under HEADING_MIN_V
      a = np.radians(turned)
      loop.tick(100., 2., fix=(5. - 5. * np.sin(a), -8. + 5. * (1. - np.cos(a)) + 3. * turned / 90., 3.), car_heading=180. + turned)
    self.assertFalse(loop.tick(97., 2., car_heading=270.)[1])  # on the street, 180 deg off the route: no course yet
    self.assertEqual(len(loop.router.calls), 2)
    self.assertIsNone(engine.route_heading())
    loop.tick(92., 4., car_heading=270.)  # 4 m/s with a good bearing accuracy: a course, against the route
    self.assertEqual(len(loop.router.calls), 3)
    self.assertAlmostEqual(loop.router.calls[2][1], 270.)

  def test_a_red_light_stand_after_driving_straight_keeps_its_heading_for_a_reroute(self):
    """Standing 60 s at a red after driving east (deviceMotion's yaw jittering a few tenths, as at the 11-06 stands): the heading
    is still the course's, so a reroute there (a new destination, the stop-sign avoidance re-run for an ETA that left its
    window) is made with it and trusted, as before: valid guidance the update after the separator."""
    from openpilot.selfdrive.navigation.tests.test_gps_hold import STOP_M, approach
    for why in ('destination', 'avoidRerun'):
      with self.subTest(why=why):
        loop = self.loop()
        approach(loop, STOP_M)
        for i in range(60):
          loop.tick(STOP_M, 0., car_heading=90. + 0.3 * float(np.sin(i)))
        engine = loop.engine
        self.assertTrue(engine.hold.standing and engine.basis.routable)
        self.assertLess(engine.basis.peak, 1.)
        if why == 'destination':
          loop.values['NavDestination'] = {'latitude': 0.003, 'longitude': 0.004}
        else:
          engine.avoid_rerun = True
        ni, valid = loop.tick(STOP_M, 0., car_heading=90.)
        self.assertEqual(len(loop.router.calls), 2)
        self.assertAlmostEqual(loop.router.calls[1][1], 90., delta=0.5)
        self.assertFalse(engine.heading_unverified)
        self.assertFalse(valid)  # the separator between the two routes
        self.assertTrue(loop.tick(STOP_M, 0., car_heading=90.)[1])

  def test_a_route_made_mid_turn_at_a_crawl_is_checked_by_the_cars_own_heading_the_update_after(self):
    """A turn from a stop at 2 m/s, under HEADING_MIN_V (here navd_loop's left; on 11-06 three rights from a stop, a second each
    at 2.3-2.9 m/s and 36-40 deg): a route made in that second goes with no heading, unverified, and the car's own heading, on
    the road the route takes, verifies it the update after the separator: no guidance lost but the separator's."""
    from openpilot.selfdrive.navigation.tests.test_gps_hold import STOP_M, approach
    from openpilot.selfdrive.turnpilot.tests.navd_loop import JUNCTION_M
    loop = self.loop()
    approach(loop, STOP_M)
    for s in (STOP_M + 2., STOP_M + 4., STOP_M + 6., STOP_M + 8.):
      loop.tick(s, 2.)
    engine = loop.engine
    self.assertFalse(engine.basis.routable)
    self.assertGreaterEqual(engine.basis.peak, 30.)
    loop.values['NavDestination'] = {'latitude': 0.003, 'longitude': 0.004}
    self.assertFalse(loop.tick(JUNCTION_M + 2., 2.)[1])
    self.assertIsNone(loop.router.calls[-1][1])
    self.assertTrue(engine.heading_unverified)
    self.assertTrue(loop.tick(JUNCTION_M + 4., 2.)[1])
    self.assertFalse(engine.heading_unverified)
    self.assertEqual(len(loop.router.calls), 2)


class TestStraightAtTheStreet(unittest.TestCase):
  """Review of 11-06, the residual of the parked-onroad gate: a GPS course the car hasn't turned from can itself point across
  the street, taken driving straight at it across a lot or down a driveway at HEADING_MIN_V or faster, or into a pull-through
  space. Standing at the exit, nose perpendicular to the street, a destination was routed with it as a known heading and
  trusted: valid guidance (and a turn desire) at once, the 11-06 wrong-way-out-of-the-spot class by another road (Valhalla's
  HEADING_TOLERANCE 90 takes the street's direction nearer the nose, which says nothing about the way the driver will turn
  out). navd checks a route made with a heading against that heading where the car is (navd routed_across), and one more
  than HEADING_MISMATCH_DEG across it, or with the car under VERIFY_STREET_V further than OFF_ROUTE_DISTANCE off it, goes
  unverified as a headingless one does (heading_due decides). navd itself on navd_loop's synthetic route (east along y=0 to
  a left at 300 m: StepsRouter answers every route so, whatever the heading) with carState and deviceMotion, onroad, and
  fixes placed where each tick says with the bearing accuracy it says (NavdLoop.tick's fixes off the route report none)."""

  def loop(self):
    from openpilot.selfdrive.turnpilot.tests.navd_loop import NavdLoop  # lazily, as TestParkedOnroadHeading does

    class Loop(NavdLoop):
      def at(self, east, north, v, heading, bacc=0.):
        """One navd update with a trusted fix (3 m accuracy) at (east, north) m moving v heading `heading` (the fix's bearing and
        deviceMotion's yaw) and reporting bearing accuracy bacc (0: none), carState's vEgo v: (instruction, valid)."""
        self.t += 1.
        self.sm.updated = dict.fromkeys(self.sm.data, False)
        msg = messaging.new_message('gpsLocationExternal')
        gps = msg.gpsLocationExternal
        gps.hasFix, gps.horizontalAccuracy, gps.speed, gps.bearingDeg, gps.bearingAccuracyDeg = True, 3., float(v), heading, bacc
        gps.longitude, gps.latitude = east / METERS_PER_DEGREE, north / METERS_PER_DEGREE
        self.sm.put('gpsLocationExternal', msg.as_reader().gpsLocationExternal)
        cs = messaging.new_message('carState')
        cs.carState.vEgo = float(v)
        self.sm.put('carState', cs.as_reader().carState)
        dm = messaging.new_message('deviceMotion')
        dm.deviceMotion.orientationNED.z, dm.deviceMotion.orientationNED.valid = float(np.radians(heading)), True
        self.sm.put('deviceMotion', dm.as_reader().deviceMotion)
        self.engine.update()
        sent = next(c.args[1] for c in reversed(self.pm.send.call_args_list) if c.args[0] == 'navInstruction')
        return sent.as_reader().navInstruction, sent.valid
    return Loop()

  def routed_standing(self, loop, place, heading):
    """A new destination set on the phone with the car standing at place heading `heading`: (navd.route_heading_unverified
    events, the separator's validity)."""
    from openpilot.selfdrive.navigation import navd
    loop.values['NavDestination'] = {'latitude': 0.003, 'longitude': 0.004}
    with patch.object(navd.cloudlog, 'event') as said:
      _, valid = loop.at(*place, 0., heading)
    return [c.kwargs for c in said.call_args_list if c.args[0] == 'navd.route_heading_unverified'], valid

  def straight_at_the_street(self, loop):
    """The probe: driven east on the street at 10 m/s (the drive's route, made with its heading), a left at 2 m/s into a lot
    north of it, north across the lot at 4 m/s, a U-turn at its top at a crawl (within REROUTE_DISTANCE of the street, so
    the drive's route stays), then straight south at the street at 4 m/s with a good bearing accuracy (a GPS course, 180),
    slowing to stand at the lot's exit 6 m north of the street's centre line, nose perpendicular to it. Returns the place
    it stands."""
    for i in range(6):
      loop.at(10. * i, 0., 10., 90., bacc=2.)
    for east, north, heading in ((52., 2., 60.), (54., 5., 30.), (55., 8., 0.)):
      loop.at(east, north, 2., heading)
    for north in (11., 14., 17.):
      loop.at(55., north, 4., 0., bacc=3.)
    for east, north, heading in ((57., 20., 45.), (60., 21., 90.), (63., 20., 135.), (64., 18., 180.)):
      loop.at(east, north, 2., heading)
    for north in (15., 12., 9.):
      loop.at(64., north, 4., 180., bacc=3.)
    loop.at(64., 7., 1., 180.)
    for _ in range(4):
      loop.at(64., 6., 0., 180.)
    assert len(loop.router.calls) == 1 and not loop.engine.heading_unverified
    return 64., 6.

  def test_standing_at_the_exit_a_route_made_with_the_course_across_the_street_waits_for_the_cars_direction(self):
    loop = self.loop()
    place = self.straight_at_the_street(loop)
    engine = loop.engine
    self.assertTrue(engine.hold.standing and engine.basis.routable)  # a course, not turned from: navd routes with it
    self.assertAlmostEqual(engine.route_heading(), 180., delta=0.5)
    said, valid = self.routed_standing(loop, place, 180.)
    self.assertAlmostEqual(loop.router.calls[-1][1], 180., delta=0.5)  # routed with it, as before
    self.assertFalse(valid)  # the separator
    # ...and the route came back along the street, 90 deg across it: checked, as a route with no heading is (the earlier code
    # trusted it and sent 'left 239.7' the update after)
    self.assertTrue(engine.heading_unverified)
    self.assertFalse(engine.route_heading_retry)
    self.assertEqual([(s['why'], s['dropped']) for s in said], [('headingAcross', None)])
    self.assertAlmostEqual(abs(said[0]['diff']), 90., delta=1.)
    self.assertAlmostEqual(said[0]['lateral'], 6., delta=1.)
    self.assertTrue(all(not loop.at(*place, 0., 180.)[1] for _ in range(5)))
    self.assertEqual(len(loop.router.calls), 2)

  def test_pulling_out_the_routes_way_verifies_it_with_no_reroute(self):
    loop = self.loop()
    place = self.straight_at_the_street(loop)
    self.routed_standing(loop, place, 180.)
    engine = loop.engine
    self.assertFalse(loop.at(64., 5., 1.5, 160.)[1])  # turning out, 70 deg off the route's way: nothing decided
    self.assertTrue(engine.heading_unverified)
    ni, valid = loop.at(66., 3., 1.5, 125.)  # 35 deg off it, 3 m off its line: the car's own heading verifies it
    self.assertTrue(valid)
    self.assertEqual(str(ni.maneuverModifier), 'left')
    self.assertFalse(engine.heading_unverified)
    self.assertEqual(len(loop.router.calls), 2)

  def test_pulling_out_against_the_route_waits_for_a_course_and_then_reroutes_with_it(self):
    loop = self.loop()
    place = self.straight_at_the_street(loop)
    self.routed_standing(loop, place, 180.)
    engine = loop.engine
    for east, north, heading in ((63., 4., 210.), (61., 2., 240.), (58., 1., 265.)):  # a right out at a crawl, west
      self.assertFalse(loop.at(east, north, 2., heading)[1])
    self.assertIsNone(engine.route_heading())  # turned 85 deg from its course at a crawl: nothing navd routes with
    self.assertEqual(len(loop.router.calls), 2)
    loop.at(54., 0.5, 4., 270., bacc=3.)  # a course on the street, 180 deg off the route: rerouted with it, and checked
    self.assertEqual(len(loop.router.calls), 3)
    self.assertAlmostEqual(loop.router.calls[2][1], 270., delta=0.5)
    self.assertTrue(engine.heading_unverified and engine.route_heading_retry)

  def test_a_course_up_a_lot_row_further_than_off_route_distance_from_the_route_is_checked_and_a_road_at_speed_is_not(self):
    """A course along a lot row 23 m north of the street (the street's own direction, so not across it; within
    REROUTE_DISTANCE, so the drive's route stays) says nothing about the street the route starts on: under VERIFY_STREET_V
    further than OFF_ROUTE_DISTANCE off it, the route is checked. At VERIFY_STREET_V or faster the car is on a road, and a
    route made with its heading is kept as before however it lies (here a parallel road 23 m off and a crossing one, 90 deg
    across: the stub routes along y=0 whatever the car's place; in a drive, a reroute that lands past its first turn, 1e
    13:23.3 and 13:29.3 at 11.5 and 9.8 m/s carState 30-41 m off, which stale_start and the missed-turn reroutes handle)."""
    with self.subTest('a lot row'):
      loop = self.loop()
      for i in range(6):
        loop.at(10. * i, 0., 10., 90., bacc=2.)
      for east, north, heading in ((52., 3., 30.), (54., 8., 0.), (55., 13., 0.), (56., 18., 0.), (58., 21., 30.), (61., 23., 60.)):
        loop.at(east, north, 2., heading)
      for east in (65., 69., 73.):
        loop.at(east, 23., 4., 90., bacc=3.)
      loop.at(75., 23., 1., 90.)
      for _ in range(3):
        loop.at(76., 23., 0., 90.)
      self.assertEqual(len(loop.router.calls), 1)
      self.assertTrue(loop.engine.basis.routable)
      said, _ = self.routed_standing(loop, (76., 23.), 90.)
      self.assertAlmostEqual(loop.router.calls[-1][1], 90., delta=0.5)
      self.assertTrue(loop.engine.heading_unverified)
      self.assertEqual([(s['why'], s['diff'], s['lateral']) for s in said], [('headingAcross', None, None)])
      self.assertFalse(loop.at(78., 18., 2., 150.)[1])  # crossing the lot to the street
      self.assertTrue(loop.at(84., 2., 4., 95., bacc=3.)[1])  # on it, the route's way: verified
      self.assertEqual(len(loop.router.calls), 2)
    for name, places, heading in (('a parallel road at speed', [(10. * i, 23.) for i in range(7)], 90.),
                                  ('a crossing road at speed', [(150., -60. + 10. * i) for i in range(7)], 0.)):
      with self.subTest(name):
        loop = self.loop()
        for place in places[:-1]:
          loop.at(*place, 10., heading, bacc=2.)
        self.assertFalse(loop.engine.heading_unverified)
        loop.values['NavDestination'] = {'latitude': 0.003, 'longitude': 0.004}
        loop.at(*places[-1], 10., heading, bacc=2.)
        self.assertEqual(len(loop.router.calls), 2)
        self.assertAlmostEqual(loop.router.calls[1][1], heading, delta=0.5)
        self.assertFalse(loop.engine.heading_unverified)

  def test_a_red_light_in_a_wide_roads_far_lane_keeps_a_reroutes_heading(self):
    """14 m off the mapped centre line (a wide road's far lane), standing at a red after driving its way: the reroute's route
    goes the heading's way there, so it is trusted and guidance goes out the update after the separator. The check is on the
    angle only: heading_due wouldn't verify that place standing (VERIFY_LATERAL_M), and a reroute at every such red would
    withhold guidance until 18 mph."""
    loop = self.loop()
    for i in range(10):
      loop.at(10. * i, 14., 10., 90., bacc=2.)
    self.assertFalse(loop.engine.heading_unverified)
    for _ in range(5):
      loop.at(95., 14., 0., 90.)
    said, valid = self.routed_standing(loop, (95., 14.), 90.)
    self.assertEqual(len(loop.router.calls), 2)
    self.assertAlmostEqual(loop.router.calls[1][1], 90., delta=0.5)
    self.assertEqual(said, [])
    self.assertFalse(loop.engine.heading_unverified)
    self.assertFalse(valid)  # the separator
    self.assertTrue(loop.at(95., 14., 0., 90.)[1])


class TestTiles(unittest.TestCase):
  def test_pmtiles_format(self):
    self.assertEqual([tile_id(1, x, y) for x, y in [(0, 0), (0, 1), (1, 1), (1, 0)]], [1, 2, 3, 4])
    self.assertEqual(sorted(tile_id(3, x, y) for x in range(8) for y in range(8)), list(range(21, 85)))
    # IDs 5, 10; runs 3, 1; lengths 4, 6; the second offset follows the first blob.
    data = bytes([2, 5, 5, 3, 1, 4, 6, 1, 0])
    self.assertEqual(decode_directory(data), ([5, 10], [(3, 4, 0), (1, 6, 4)]))
    for invalid in (b'\x80', b'\x01\x00\x01\x01\x00', data + b'\x00'):
      with self.assertRaises(ValueError):
        decode_directory(invalid)
    with local_archive(make_archive(run=3, max_zoom=1)) as archive:  # the zoom-zero tile is also addressed by IDs 1 and 2
      self.assertEqual(archive.get(1, 0, 1), road_tile())
      self.assertIsNone(archive.get(1, 1, 1))
    with self.assertRaises(ValueError), local_archive(make_archive()[:-1]):
      pass

  def test_vector_tiles(self):
    roads = decode_roads(road_tile())
    self.assertEqual((len(roads), roads[0].kind), (1, 'minor_road'))
    np.testing.assert_array_equal(roads[0].points, [[0.5, 0], [0.5, 1]])
    lines = decode_lines([9, 20, 20, 10, 10, 0, 9, 9, 9, 10, 0, 10])
    np.testing.assert_array_equal(lines[0], [[10, 10], [15, 10]])
    np.testing.assert_array_equal(lines[1], [[10, 5], [10, 10]])
    for data in (b'\x1a\xff', b'\x00', b'\x1a\x02\x0a\x04'):
      with self.assertRaises(ValueError):
        decode_roads(data)
    for commands in ([10, 1, 1], [9, 0], [9, 0, 0, 15]):
      with self.assertRaises(ValueError):
        decode_lines(commands)


class TestRendering(unittest.TestCase):
  def test_projection(self):
    # At the equator, 64m ahead is the image center and the car is 64px below it, at 1m per pixel.
    points = mercator(np.array([[0, 0], [0, 64 * METER], [64 * METER, 0]]))
    np.testing.assert_allclose(project(points, 0, 0, 0), [[128, 192], [128, 128], [192, 192]], atol=1e-5)
    np.testing.assert_allclose(project(points, 0, 0, 90), [[128, 192], [64, 192], [128, 128]], atol=1e-5)
    pixels = project(mercator(np.array([[-179.999, 0]])), 179.999, 0, 90)  # across the antimeridian
    self.assertAlmostEqual(pixels[0, 1], 192 - 222.6389816, places=5)

  def test_strokes(self):
    image = np.zeros((SIZE, SIZE), dtype=np.uint8)
    draw_line(image, np.array([[-1000, 128], [1000, 128]]), 255, 8)
    self.assertTrue(np.all(image[128] == 255))
    draw_line(image, np.array([[128, 0], [128, 256]]), 128, 5, overlay=True)  # the grey route replaces the white road
    self.assertEqual((image[128, 128], image[128, 131]), (128, 255))
    before = image.copy()
    draw_line(image, np.array([[-1000, -1000], [-500, -500]]), 255, 5)
    np.testing.assert_array_equal(image, before)
    draw_line(image, np.array([[40, 40], [40, 40]]), 255, 5)  # a zero-length segment is still a dot
    self.assertEqual(image[40, 40], 255)

  def test_crossings(self):
    route = np.array([[0.01, 0], [0.01, 0.01]])  # off to the side, out of the way of the probes
    images = {}
    for crossing in ('bridge', 'tunnel'):
      with local_archive(make_archive(road_tile(crossing))) as archive:
        images[crossing] = MapRenderer(archive).render(0, -40 * METER, 0, route)[0]  # the crossing 40m ahead
    # The horizontal road crosses the vertical one 40px above the ego, at row 152. A bridge severs the road beneath
    # just outside its own stroke; a tunnel passes underneath untouched. The cut has butt caps, so the
    # bridge's own end is not wrapped in black.
    self.assertEqual((images['bridge'][146, 128], images['bridge'][152, 128], images['bridge'][152, 200], images['bridge'][140, 128]), (0, 255, 255, 255))
    self.assertEqual((images['tunnel'][146, 128], images['tunnel'][152, 200]), (255, 255))
    # The ground road cuts over the tunnel beneath it, and the tunnel is drawn without its casing.
    self.assertEqual((images['tunnel'][152, 133], images['tunnel'][147, 200]), (0, 0))

  def test_road_widths(self):
    # Pinned at the style zoom of 1 m per pixel: 16.26 at the equator and 15.26 at 60 degrees. Tunnels have no casing.
    styles, levels = np.array([MAJOR, MEDIUM, STREET, MINOR, TRACK, STREET]), np.array([0, 0, 0, 0, 0, -1])
    np.testing.assert_allclose(stroke_widths(16.256, styles, levels), [19.02, 15.48, 9.86, 6.70, 7.02, 7.89], atol=0.01)
    np.testing.assert_allclose(stroke_widths(15.256, styles, levels), [13.52, 10.78, 6.39, 4.16, 4.58, 4.62], atol=0.01)

  def test_route_tiles(self):
    route = np.array([[0, -0.01], [0, 0.01]])  # 2.2 km north along the prime meridian
    with local_archive(make_archive(max_zoom=15)) as archive:
      renderer = MapRenderer(archive)
      tiles = renderer.route_tiles(route)
      # Both columns beside the meridian, and the rows within the view radius of the route's ends.
      self.assertEqual({x for x, _ in tiles}, {16383, 16384})
      self.assertEqual({y for _, y in tiles}, set(range(16382, 16386)))

  def test_validity_and_display(self):
    route = np.array([[0, -0.01], [0, 0.01]])
    with local_archive(make_archive()) as archive:
      renderer = MapRenderer(archive)
      self.assertTrue(renderer.render(0, 0, 0, route)[1])
      self.assertFalse(renderer.render(0.01, 0, 0, route)[1])  # no road near the ego
      with patch.object(archive, 'get', return_value=None):  # a missing tile
        image, valid = MapRenderer(archive).render(0, 0, 0, route)
        self.assertFalse(valid)
        self.assertEqual(image[128, 128], 128)  # the route is still drawn
    with local_archive(make_archive(field(3, field(1, b'buildings')))) as archive:  # tiles without roads
      self.assertFalse(MapRenderer(archive).render(0, 0, 0, route)[1])


class TestRouteTracking(unittest.TestCase):
  def test_snap_heading_and_window(self):
    m = METER
    tracker = RouteTracker(np.array([[0, 0], [0, 200 * m], [200 * m, 200 * m]]))  # north 200m, then east 200m
    longitude, latitude, bearing = tracker.snap(3 * m, 100 * m)  # 3m right of the first leg
    self.assertAlmostEqual(tracker.progress, 100, delta=0.5)
    self.assertAlmostEqual(longitude, 0)  # lateral error removed
    self.assertAlmostEqual(latitude, 100 * m)
    self.assertAlmostEqual(bearing, 0, places=3)
    self.assertAlmostEqual(tracker.snap(0, 200 * m)[2], 45, places=2)  # at the corner the +-50m chord is mid-turn
    self.assertAlmostEqual(tracker.snap(0, 150 * m)[2], 45, places=2)  # progress never decreases
    tracker.snap(100 * m, 200 * m)  # progress 300: the window ahead no longer includes the first leg
    self.assertAlmostEqual(tracker.snap(150 * m, 200 * m)[0], 150 * m)  # so the window's offset is applied
    self.assertAlmostEqual(tracker.snap(200 * m, 200 * m)[2], 90, places=2)  # the chord clips at the route's end
    # Out and back along the same road: a fix near the far leg stays on the outbound leg within the window,
    # while a fresh tracker searches anywhere and lands on the return leg.
    route = np.array([[0, 0], [0, 1000 * m], [10 * m, 1000 * m], [10 * m, 0]])
    tracker = RouteTracker(route)
    tracker.snap(1 * m, 100 * m)
    self.assertAlmostEqual(tracker.snap(9 * m, 150 * m)[2], 0, places=3)
    self.assertAlmostEqual(tracker.progress, 150, delta=0.5)
    self.assertAlmostEqual(RouteTracker(route).snap(9 * m, 150 * m)[2], 180, places=3)


if __name__ == '__main__':
  unittest.main()


class TestStopAvoidance(unittest.TestCase):
  """navd's stop-sign avoidance loop (STOP_SIGNS.md 3.3, rows C2, C21, N1-N3, N9-N14): every stop and yield the certifier
  (routing/certify.py) can't certify is excluded by its approach edge and the route recomputed around it, with the
  router's status-neutral reroute(). A fake router; the certifier is the real one where named, else a stand-in."""
  STEP = 111.195

  def setUp(self):
    self.pm = MagicMock()
    self.values = {'NavDestination': {'latitude': 0, 'longitude': 0.002}, 'LanguageSetting': 'main_en', 'TurnPilotStopSignGo': 'auto'}
    self.sm = MagicMock()
    self.sm.valid = dict.fromkeys(('gpsLocation', 'gpsLocationExternal'), True)
    self.now, self.mono = 1_800_000_000., 1000.
    self.certified: set = set()  # approach keys the stand-in certifier certifies
    self.engine = self.make_engine()

  def make_engine(self, certifier='fake'):
    params = MagicMock()
    params.get.side_effect = self.values.get
    params.remove.side_effect = lambda key: self.values.pop(key, None)
    engine = RouteEngine(self.sm, self.pm, params=params, router=StubRouter(), background=False, clock=lambda: self.now,
                         monotonic=lambda: self.mono, certifier=self.certify if certifier == 'fake' else None)
    engine.last_position, engine.last_bearing = (0, 0), 90
    return engine

  def certify(self, control, eta, runtime):
    ok = control.approach_key in self.certified and runtime.mode == 'auto'
    control.certification, control.cert_reason = ('certifiable', 'none') if ok else ('uncertifiable', 'notSurveyed')
    return control.certification, control.cert_reason

  def steps(self):
    """A new route object each time (the router tells routes apart by identity), 2 x 111 m, 10 s each."""
    return [{'geometry': {'coordinates': [[0, 0], [0.0005, 0], [0.001, 0]]}, 'distance': self.STEP, 'duration': 10,
             'bannerInstructions': [{'distanceAlongGeometry': 100, 'primary': {'text': 'Turn right', 'type': 'turn', 'modifier': 'right'}}]},
            {'geometry': {'coordinates': [[0.001, 0], [0.002, 0]]}, 'distance': self.STEP, 'duration': 10, 'bannerInstructions': []}]

  @staticmethod
  def stop(n, route_m, kind='stop', first=False, last=False, junction=True):
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    from openpilot.selfdrive.turnpilot.routing.junctions import Junction
    step = 0 if route_m < TestStopAvoidance.STEP else 1
    return Control(n, kind, route_m, step, route_m - step * TestStopAvoidance.STEP, junction_info=Junction() if junction else None,
                   approach_key=(500 + n, True, (0.0001 * n, 0.001)), exclusion=(0.0001 * n, 0.0009, 90. + n), first_edge=first, last_edge=last)

  def route_with(self, controls, steps=None):
    """Steps the router gives, with these controls."""
    steps = steps if steps is not None else self.steps()
    self.engine.router.controls_by_steps[id(steps)] = (controls, 'ok')
    return steps

  def start(self, controls, reroutes=(), destination=(0.002, 0)):
    """Route to destination: the first route has controls; reroutes are (controls or None) the reroutes give in turn."""
    router = self.engine.router
    router.steps = self.route_with(controls)
    router.reroute_results = [None if c is None else self.route_with(c) for c in reroutes]
    self.engine.calculate_route(destination)
    return router

  def instruction(self):
    self.engine.send_instruction()
    return next(call.args[1] for call in reversed(self.pm.send.call_args_list) if call.args[0] == 'navInstruction').navInstruction

  def test_an_avoidance_reroute_made_with_no_heading_leaves_the_route_unverified(self):
    """TurnPilot (route start): the avoidance loop reroutes from the car's position and heading then (engine.position); a
    reroute with none may be the route kept, so the route waits for the car's direction to be checked however it began."""
    engine, certify = self.engine, self.engine.certifier

    def heading_gone(control, eta, runtime):
      engine.last_bearing = None  # the heading went while the job ran
      return certify(control, eta, runtime)
    engine.certifier = heading_gone
    router = self.start([self.stop(1, 60.)], reroutes=[[]])
    self.assertEqual(router.calls[0][2], 90)  # the route itself had one
    self.assertEqual([c[2] for c in router.reroute_calls], [None])
    self.assertTrue(engine.heading_unverified)
    self.instruction()
    self.assertFalse(next(c.args[1] for c in reversed(self.pm.send.call_args_list) if c.args[0] == 'navInstruction').valid)

  def test_C2_noTable_floor_every_stop_and_yield_is_routed_around(self):
    """No camera measurements yet: the real certifier says noTable for every stop and yield, so the route avoids all of
    them (the stop-free floor), excluding each approach edge by its local bearing (N9)."""
    self.engine = self.make_engine(certifier=None)
    a, y = self.stop(1, 60.), self.stop(2, 150., kind='yield')
    router = self.start([a, y], reroutes=[[]])
    self.assertEqual((a.junction_info.certification, a.junction_info.cert_reason), ('uncertifiable', 'noTable'))
    self.assertEqual(y.junction_info.cert_reason, 'noTable')
    self.assertEqual([c[4] for c in router.reroute_calls], [[{'lat': 0.0001, 'lon': 0.0009, 'heading': 91.}, {'lat': 0.0002, 'lon': 0.0009, 'heading': 92.}]])
    self.assertEqual(self.engine.controls, [])
    ni = self.instruction()
    self.assertEqual((str(ni.avoidStatus), ni.avoidedControls), ('clean', 2))
    # and the time it added, for the route-start announcement (STOP_SIGNS.md 3.4, SG4c)
    self.engine.avoid_added_s = 240.
    self.assertAlmostEqual(self.instruction().avoidAddedS, 240.)

  def test_N9_the_exclusion_takes_valhallas_own_heading_at_the_point(self):
    a, b = self.stop(1, 60.), self.stop(2, 150.)
    self.engine.router.headings = {501: 349.}  # by way id: a's way
    router = self.start([a, b], reroutes=[[]])
    self.assertEqual([p['heading'] for p in router.reroute_calls[0][4]], [349., 92.])  # b: none found, the shape's bearing

  def test_a_certifiable_stop_stays_and_signals_are_never_excluded(self):
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    a = self.stop(1, 60.)
    self.certified = {a.approach_key}
    router = self.start([Control(0, 'signal', 30., 0, 30.), a])
    self.assertEqual(router.reroute_calls, [])
    ni = self.instruction()
    self.assertEqual((str(ni.avoidStatus), ni.avoidedControls), ('clean', 0))

  def test_N3_terminates_within_max_avoid_iter(self):
    from openpilot.selfdrive.turnpilot.stop_rules import MAX_AVOID_ITER
    router = self.start([self.stop(0, 60.)], reroutes=[[self.stop(k, 60.)] for k in range(1, 20)])
    self.assertEqual(len(router.reroute_calls), MAX_AVOID_ITER)
    self.assertEqual(self.engine.avoid_status, 'keptBest')
    self.assertEqual([c.certification for c in self.engine.controls], ['unavoidable'])  # what is left needs the driver

  def test_N3_terminates_within_avoid_time(self):
    from openpilot.selfdrive.navigation.navd import AVOID_TIME_S

    def slow():
      self.mono += AVOID_TIME_S * 0.6
    router = self.start([self.stop(0, 60.)], reroutes=[[self.stop(k, 60.)] for k in range(1, 20)])
    self.assertGreater(len(router.reroute_calls), 2)  # (without the clock moving: many)
    self.engine = self.make_engine()
    self.engine.router.on_reroute = slow
    router = self.start([self.stop(0, 60.)], reroutes=[[self.stop(k, 60.)] for k in range(1, 20)])
    self.assertEqual(len(router.reroute_calls), 2)
    self.assertEqual(self.engine.avoid_status, 'keptBest')

  def test_N3_honours_cancel(self):
    router = self.engine.router
    router.on_reroute = lambda: self.engine.job.cancel.set()
    self.start([self.stop(0, 60.)], reroutes=[[self.stop(k, 60.)] for k in range(1, 20)])
    self.assertEqual(len(router.reroute_calls), 1)
    self.assertIsNone(self.engine.route)  # a cancelled job's route is dropped, as before

  def test_N3_a_failed_iteration_keeps_the_best_route_and_its_controls_as_a_pair(self):
    b = self.stop(2, 150.)
    router = self.start([self.stop(1, 60.)], reroutes=[[b], None])
    self.assertEqual(len(router.reroute_calls), 2)
    first_reroute = [steps for steps in router.controls_calls if steps[0] is not router.steps][0][0]
    self.assertIs(self.engine.route, first_reroute)  # the steps of the last good iteration, and its controls with them
    self.assertEqual(self.engine.controls, [b])
    self.assertEqual((b.certification, self.engine.avoid_status), ('unavoidable', 'keptBest'))
    avoid = self.engine.avoid_sets[(0.002, 0)]
    self.assertEqual(avoid.keys(), {self.stop(1, 60.).approach_key})  # the failed batch (b) is dropped from the set

  def test_the_detour_is_bounded_against_the_plain_route(self):
    """Avoidance took any route around an uncertifiable stop: on the region's real tiles, every stop uncertifiable (the car
    with no surveys), its kept route was up to 74% slower (+242 s) and 47% longer than the plain one. A reroute adding more
    than detour_budget_s over the plain route (10% of it, from 30 s to 180 s) is a failed iteration: its batch is dropped
    and unavoidable, and the best route within the budget is kept (keptBest)."""
    from openpilot.selfdrive.turnpilot.routing.avoid import detour_budget_s
    self.assertEqual((detour_budget_s(20.), detour_budget_s(900.), detour_budget_s(3600.)), (30., 90., 180.))
    a, b = self.stop(1, 60.), self.stop(2, 150.)
    near, far = self.steps(), self.steps()
    for s in near:
      s['duration'] = 15  # +10 s over the plain route's 20 s: within the 30 s floor
    for s in far:
      s['duration'] = 60  # +100 s: past it
    router = self.engine.router
    router.steps = self.route_with([a])
    router.reroute_results = [self.route_with([b], near), self.route_with([], far)]
    self.engine.calculate_route((0.002, 0))
    self.assertEqual(len(router.reroute_calls), 2)
    self.assertIs(self.engine.route, near)
    self.assertEqual(self.engine.controls, [b])
    self.assertEqual((b.certification, self.engine.avoid_status), ('unavoidable', 'keptBest'))
    self.assertAlmostEqual(self.engine.avoid_added_s, 10.)
    self.assertEqual(self.engine.avoid_sets[(0.002, 0)].keys(), {a.approach_key})  # the over-budget batch (b) is dropped

  def test_N1_N10_first_last_and_own_edge_are_unavoidable_and_never_excluded(self):
    own = self.stop(3, 100.)
    self.engine.router.own = (own.approach_key[0], own.approach_key[1], (own.approach_key[2][0] + 1e-6, own.approach_key[2][1]))
    first, last = self.stop(1, 20., first=True), self.stop(2, 200., last=True)
    router = self.start([first, own, last])
    self.assertEqual(router.reroute_calls, [])
    self.assertEqual([c.certification for c in (first, own, last)], ['unavoidable'] * 3)
    ni = self.instruction()
    self.assertEqual(str(ni.avoidStatus), 'unavoidable')
    # the whole route's count and the first on the origin street (its first edge), for the route-start announcement (SG4c)
    self.assertEqual((ni.unavoidableControls, ni.firstUnavoidableOnOrigin), (3, True))

  def test_N10_a_point_already_in_the_set_on_the_cars_own_edge_is_not_sent(self):
    a, b = self.stop(1, 60.), self.stop(2, 150.)
    self.start([a], reroutes=[[]])
    self.engine.router.own = a.approach_key  # the car is now on a's approach edge
    a2 = self.stop(1, 60.)
    router = self.start([a2, b], reroutes=[[a2]])
    self.assertEqual([p['heading'] for p in router.reroute_calls[-1][4]], [92.])  # b only
    self.assertEqual((a2.certification, self.engine.avoid_status), ('unavoidable', 'unavoidable'))

  def test_N2_a_banned_control_still_on_the_route_is_failed(self):
    a = self.stop(1, 60.)
    self.start([a], reroutes=[[self.stop(1, 60.)]])
    ni = self.instruction()
    self.assertEqual((str(ni.avoidStatus), ni.avoidedControls), ('failed', 0))
    self.assertEqual([c.certification for c in self.engine.controls], ['unavoidable'])

  def test_N12_C21_every_navd_reroute_reuses_the_set_and_a_new_destination_clears_it(self):
    a, b = self.stop(1, 60.), self.stop(2, 150.)
    self.start([a], reroutes=[[]])
    router = self.start([b], reroutes=[[]])  # a reroute for the same destination (a missed turn, off the route, a skip)
    self.assertEqual(sorted(p['heading'] for p in router.reroute_calls[-1][4]), [91., 92.])
    router = self.start([self.stop(3, 60.)], reroutes=[[]], destination=(0.003, 0))
    self.assertEqual([p['heading'] for p in router.reroute_calls[-1][4]], [93.])

  def test_N11_a_runtime_input_change_reruns_the_loop_once(self):
    a = self.stop(1, 60.)
    self.certified = {a.approach_key}
    router = self.start([a])
    self.engine.recompute_hold = RECOMPUTE_HOLD
    self.engine.recompute_route(gps_ok=True)
    self.assertEqual(router.call_count, 1)
    self.values['TurnPilotStopSignGo'] = 'shadow'  # the mode changed (side cameras off would too, from turnpilotPlan)
    router.steps = self.route_with([self.stop(1, 60.)])
    router.reroute_results = [self.route_with([])]
    self.engine.recompute_route(gps_ok=True)
    self.assertEqual(router.call_count, 2)
    self.assertEqual(self.engine.avoid_status, 'clean')
    self.engine.recompute_route(gps_ok=True)
    self.assertEqual(router.call_count, 2)  # once

  def test_a_route_planned_before_the_clock_was_set_reruns_once_it_is(self):
    # 2026-11-02: the comma boots on a clock months behind; a route planned then failed every light check and was never
    # certified again
    from unittest import mock
    import openpilot.selfdrive.navigation.navd as navd_mod
    a = self.stop(1, 60.)
    self.certified = {a.approach_key}
    with mock.patch.object(navd_mod, 'system_time_valid', return_value=False):
      router = self.start([a])
      self.engine.recompute_hold = RECOMPUTE_HOLD
      self.engine.recompute_route(gps_ok=True)
    self.assertEqual(router.call_count, 1)
    router.steps = self.route_with([self.stop(1, 60.)])
    router.reroute_results = [self.route_with([])]
    with mock.patch.object(navd_mod, 'system_time_valid', return_value=True):
      self.engine.recompute_route(gps_ok=True)
      self.assertEqual(router.call_count, 2)
      self.engine.recompute_route(gps_ok=True)
      self.assertEqual(router.call_count, 2)  # once

  def test_N14_eta_leaving_its_window_reruns_once_and_a_slip_inside_does_not(self):
    a = self.stop(1, 60.)
    self.certified = {a.approach_key}
    router = self.start([a])
    self.assertEqual(a.certification, 'certifiable')
    self.engine.recompute_hold = RECOMPUTE_HOLD
    self.now += 600.  # stuck 10 min at the start: still inside the window (ETA - 5 min to + 20 min)
    self.engine.send_instruction()
    self.engine.recompute_route(gps_ok=True)
    self.assertEqual(router.call_count, 1)
    self.now += 700.  # 21.7 min late: outside
    router.steps = self.route_with([self.stop(1, 60.)])
    self.engine.send_instruction()
    self.engine.recompute_route(gps_ok=True)
    self.assertEqual(router.call_count, 2)
    self.engine.send_instruction()
    self.engine.recompute_route(gps_ok=True)
    self.assertEqual(router.call_count, 2)  # the new route's window is from now

  def test_the_loop_never_sets_the_map_status(self):
    # the reroutes are status-neutral: navd only notes the taken route's lefts (counts, routing/lefts.py) once it has it
    router = self.start([self.stop(1, 60.)], reroutes=[[self.stop(2, 150.)], None])
    self.assertEqual(len(router.reroute_calls), 2)
    self.assertEqual([(c[0], c.args, c.kwargs) for c in router.status.method_calls],
                     [('note', ((0.002, 0),), {'lefts': {'avoided': 0, 'unavoidable': 0}})])

  def test_the_route_keeps_the_always_drive_switch_it_was_made_under(self):
    """TurnPilotAlwaysDrive (ALWAYS_DRIVE.md section 1, step S1): read at route time for R1 to R3 (step S6), the route
    keeping the value it was made under; unset, off or unreadable is off."""
    self.start([])
    self.assertIs(self.engine.route_always_drive, False)
    for value, on in ((True, True), (b'1', True), ('1', True), (False, False), (b'0', False), ('0', False), (None, False)):
      self.values['TurnPilotAlwaysDrive'] = value
      self.start([])
      self.assertIs(self.engine.route_always_drive, on, value)
    self.values['TurnPilotAlwaysDrive'] = True
    get = self.engine.params.get.side_effect
    self.engine.params.get.side_effect = lambda key: (_ for _ in ()).throw(KeyError(key)) if key == 'TurnPilotAlwaysDrive' else get(key)
    self.start([])
    self.assertIs(self.engine.route_always_drive, False)

  def test_avoid_stops_off_certifies_but_excludes_nothing(self):
    self.values['TurnPilotAvoidStops'] = False
    a = self.stop(1, 60.)
    router = self.start([a])
    self.assertEqual((router.reroute_calls, a.certification), ([], 'uncertifiable'))
    self.assertEqual(str(self.instruction().avoidStatus), 'none')

  def test_a_control_without_its_exclusion_point_is_unavoidable(self):
    a = self.stop(1, 60.)
    a.exclusion = None
    router = self.start([a])
    self.assertEqual((router.reroute_calls, a.certification, self.engine.avoid_status), ([], 'unavoidable', 'unavoidable'))

  def test_no_valid_clock_fails_the_light_check_and_is_avoided(self):
    self.engine = self.make_engine(certifier=None)
    self.engine.clock = lambda: None
    a = self.stop(1, 60.)
    with patch('openpilot.selfdrive.turnpilot.routing.certify.sr.light_window') as light:
      router = self.start([a], reroutes=[[]])
    # noTable comes first with the shipped file; the clock is checked without reaching the sun
    self.assertEqual((a.cert_reason, light.call_count, len(router.reroute_calls)), ('noTable', 0, 1))
    from openpilot.selfdrive.navigation.navd import eta_of, wall_clock
    self.assertIsNone(eta_of(self.steps(), a, None))
    with patch('openpilot.selfdrive.navigation.navd.system_time_valid', return_value=False):
      self.assertIsNone(wall_clock())
    with patch('openpilot.selfdrive.navigation.navd.system_time_valid', return_value=True):
      self.assertGreater(wall_clock(), 1.7e9)

  def test_a_sim_visibility_table_is_read_in_the_sim_with_the_param_and_a_sim_car(self):
    import os

    from opendbc.car.structs import car

    from openpilot.selfdrive.turnpilot import visibility_table
    from openpilot.selfdrive.turnpilot.routing.certify import Runtime
    sim_car = car.CarParams.new_message(fingerprintSource='fixed').to_bytes()
    real_car = car.CarParams.new_message(fingerprintSource='can', brand='tesla').to_bytes()
    with patch.object(visibility_table.cloudlog, 'exception') as said, patch.object(visibility_table, 'cached', return_value=None) as cached:
      cases = []
      for env, oracle, cp in (({'SIMULATION': '1'}, True, sim_car), ({}, True, sim_car), ({'SIMULATION': '1'}, False, sim_car),
                              ({'SIMULATION': '1'}, True, real_car), ({'SIMULATION': '1'}, True, None)):
        self.values.update({'TurnPilotSimLightOracle': oracle, 'CarParams': cp})
        with patch.dict(os.environ, env, clear=False):
          if 'SIMULATION' not in env:
            os.environ.pop('SIMULATION', None)
          cases.append(self.engine.sim_tables_ok())
          self.engine.certify_default(self.stop(1, 60.), self.now, Runtime())
          self.assertEqual(cached.call_args.kwargs['sim_ok'], cases[-1])
    self.assertEqual(cases, [True, False, False, False, False])
    said.assert_not_called()

  def test_the_params_are_declared(self):
    # TurnPilotAvoidStops (on by default; step S3 sets it off) and TurnPilotStopLaunch (A_LAUNCH_PLAN, 0.5 m/s^2 until S4a)
    header = (Path(__file__).parents[3] / 'common' / 'params_keys.h').read_text()
    self.assertRegex(header, r'\{"TurnPilotAvoidStops", \{PERSISTENT, BOOL, "1"\}\}')
    self.assertRegex(header, r'\{"TurnPilotStopLaunch", \{PERSISTENT, FLOAT, "0.5"\}\}')

  def test_eta_of_each_control_from_the_step_durations(self):
    seen = []

    def certify(control, eta, runtime):
      seen.append((control.id, eta))
      return 'certifiable', 'none'
    self.engine.certifier = certify
    self.start([self.stop(1, 55.6), self.stop(2, self.STEP + 55.6)])
    self.assertEqual([(i, round(t - self.now, 1)) for i, t in seen], [(1, 5.), (2, 15.)])

  # --- TurnPilotStopSignGo=vision (tp/vision-go): navd's avoidance keeps every stop and yield vision can take ---

  def vision_stop(self, n, route_m, mph=25, kind='stop', divided=False, rail_distance=-1.):
    """A control on the shipped files as Saturday's car meets it: no survey, the map's tiers (an all-way), each approach mph."""
    from openpilot.selfdrive.turnpilot.routing.junctions import Approach, Junction
    from openpilot.selfdrive.turnpilot.stop_rules import MPH
    c = self.stop(n, route_m, kind=kind)
    legs = [(270., -90., 'left', 11), (90., 90., 'right', 12), (0., 0., 'opposite', 13)]
    c.junction_info = Junction(osm_node_id=4242, lat=39.7684, lon=-86.1581, stop_point_distance=13., rail_distance=rail_distance, surveyed=False,
                               sidecar=True, divided=divided, approaches=[
                                 Approach(b, rel, role, 'residential', 'road', True, True, 1, 1, 'stop', 'stop', 'mapStrong', mph * MPH,
                                          mph * MPH, 'tagged', way) for b, rel, role, way in legs])
    return c

  def vision_engine(self, side_views=True):
    """The real certifier on the shipped files, 15:00 EDT, turnpilotd publishing mode vision and its side views."""
    from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA
    self.values['TurnPilotStopSignGo'] = 'vision'
    self.now = ETA - 30.
    self.engine = self.make_engine(certifier=None)
    plan = SimpleNamespace(stopGo=SimpleNamespace(mode='vision', sideViewsLive=side_views))
    self.sm.data = {'turnpilotPlan': plan}
    self.sm.valid['turnpilotPlan'] = True
    self.sm.__getitem__.side_effect = lambda key: plan if key == 'turnpilotPlan' else MagicMock()

  def test_vision_keeps_a_25_mph_stop_and_yield_and_the_long_looks_35_mph_one_on_the_route(self):
    self.vision_engine()
    self.assertEqual(self.engine.runtime_inputs().mode, 'vision')
    a, y, fast = self.vision_stop(1, 60.), self.vision_stop(2, 150., kind='yield'), self.vision_stop(3, 170., mph=35)
    router = self.start([a, y, fast])
    self.assertEqual(router.reroute_calls, [])
    self.assertEqual([c.certification for c in (a, y, fast)], ['certifiable'] * 3)
    self.assertIn('long look', fast.junction_info.cert_detail)
    ni = self.instruction()
    self.assertEqual((str(ni.avoidStatus), ni.avoidedControls), ('clean', 0))
    self.assertEqual(len(self.engine.controls), 3)

  def test_vision_routes_around_no_stop_and_says_the_one_its_line_would_ask_for(self):
    # route 29 (11-08): the loop took a 13 s detour round one stop and tried a 43 s one round two more, under vision, where
    # the line drives the stop itself. The route stays Valhalla's; the one stop the line would still ask the driver for (a
    # mapped rail crossing: stop_go._vision_gate) is said, a divided cross road it drives is not
    self.vision_engine()
    keep, divided, rail = self.vision_stop(1, 60.), self.vision_stop(2, 150., divided=True), self.vision_stop(3, 170., rail_distance=12.)
    router = self.start([keep, divided, rail], reroutes=[[keep]])
    self.assertEqual(router.reroute_calls, [])
    self.assertEqual((divided.certification, divided.cert_reason), ('certifiable', 'none'))
    self.assertEqual((rail.certification, rail.cert_reason), ('unavoidable', 'geometry'))
    ni = self.instruction()
    self.assertEqual((str(ni.avoidStatus), ni.avoidedControls, ni.unavoidableControls), ('unavoidable', 0, 1))
    self.assertEqual(len(self.engine.controls), 3)

  def test_vision_with_the_side_views_off_routes_around_no_stop(self):
    self.vision_engine(side_views=False)
    a, y = self.vision_stop(1, 60.), self.vision_stop(2, 150., kind='yield')
    router = self.start([a, y], reroutes=[[]])
    # driven on the forward views: certified, nothing counted for the driver
    self.assertEqual([(c.certification, c.cert_reason) for c in (a, y)], [('certifiable', 'none')] * 2)
    self.assertEqual(router.reroute_calls, [])
    self.assertEqual(self.instruction().unavoidableControls, 0)

  def test_vision_counts_only_a_stop_its_line_hands_back_at_as_needing_the_driver(self):
    # whatever a certifier marked a stop for, under vision the route-start count goes by the line's own rule
    # (stop_rules.vision_needs_driver): a mapped rail crossing or no visibility file. Reach, glare, skew are driven
    def certifier(control, eta, runtime):
      control.certification, control.cert_reason = 'uncertifiable', {1: 'reach', 2: 'light', 3: 'skew', 4: 'geometry', 5: 'noTable'}[control.id]
      return control.certification, control.cert_reason
    self.vision_engine()
    self.engine.certifier = certifier
    stops = [self.vision_stop(1, 60.), self.vision_stop(2, 150.), self.vision_stop(3, 170.), self.vision_stop(4, 190., rail_distance=12.),
             self.vision_stop(5, 210.)]
    router = self.start(stops, reroutes=[[]])
    self.assertEqual(router.reroute_calls, [])
    self.assertEqual([c.certification for c in stops], ['unavoidable'] * 5)
    self.assertEqual(self.instruction().unavoidableControls, 2)  # the rail crossing and the missing file

  def test_a_plan_invalid_for_a_moment_is_no_runtime_change_and_no_reroute(self):
    # turnpilotPlan goes invalid whenever carState or modelV2 fail their checks for half a second: read as "side views off",
    # each such blip rerouted the whole route twice (off, then on again), navd's messages invalid while each job ran
    self.vision_engine()
    router = self.start([self.vision_stop(1, 60.)])
    self.engine.recompute_hold = RECOMPUTE_HOLD
    self.engine.recompute_route(gps_ok=True)
    self.assertEqual(router.call_count, 1)
    self.sm.valid['turnpilotPlan'] = False
    router.steps = self.route_with([self.vision_stop(1, 60.)])
    self.engine.recompute_route(gps_ok=True)
    self.assertEqual((router.call_count, self.engine.runtime_inputs().side_views), (1, True))
    self.sm.valid['turnpilotPlan'] = True
    self.engine.recompute_route(gps_ok=True)
    self.assertEqual(router.call_count, 1)
    # the side views really going off is still a change
    self.sm.data['turnpilotPlan'].stopGo.sideViewsLive = False
    self.engine.recompute_route(gps_ok=True)
    self.assertEqual(router.call_count, 2)

  def test_off_vision_the_loop_still_routes_around_a_stop_that_hands_back(self):
    a = self.stop(1, 60.)
    router = self.start([a], reroutes=[[]])
    self.assertEqual(len(router.reroute_calls), 1)
    self.assertEqual((self.engine.avoid_status, self.engine.avoided), ('clean', 1))

  def test_always_drive_changes_nothing_navd_certifies_keeps_or_avoids_under_vision_or_auto(self):
    """TurnPilotAlwaysDrive implies nothing about TurnPilotStopSignGo: at route time navd certifies, keeps and avoids the same
    stops with the switch on as off, under vision and under auto (the mode turnpilotd publishes, never the switch)."""
    seen = {}
    for mode in ('vision', 'auto'):
      for always in ('0', '1'):
        self.values['TurnPilotAlwaysDrive'] = always
        self.vision_engine()
        if mode == 'auto':
          self.values['TurnPilotStopSignGo'] = 'auto'
          self.sm.data['turnpilotPlan'].stopGo.mode = 'auto'
        keep, fast, divided = self.vision_stop(1, 60.), self.vision_stop(2, 150., mph=35), self.vision_stop(3, 170., divided=True)
        router = self.start([keep, fast, divided], reroutes=[[keep, fast], [keep], []])
        self.assertIs(self.engine.route_always_drive, always == '1')
        seen[mode, always] = (self.engine.runtime_inputs().mode, [(c.certification, c.cert_reason) for c in (keep, fast, divided)],
                              [len(r[4]) for r in router.reroute_calls])
      self.assertEqual(seen[mode, '1'], seen[mode, '0'], seen)
    self.assertEqual((seen['vision', '1'][0], seen['auto', '1'][0]), ('vision', 'auto'))
    self.assertEqual(seen['vision', '1'][1][0], ('certifiable', 'none'))
    self.assertNotEqual(seen['vision', '1'][1], seen['auto', '1'][1])  # the mode, not the switch, decides
