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
from openpilot.selfdrive.navigation.navd import RECOMPUTE_HOLD, REROUTE_COUNTER_MIN, ROUTE_RESEND_PERIOD, RouteEngine, set_banner
from openpilot.selfdrive.navigation.pmtiles import PMTiles, decode_directory, tile_id
from openpilot.selfdrive.navigation.renderer import MAJOR, MEDIUM, MINOR, SIZE, STREET, TRACK, MapRenderer, draw_segments, mercator, project, stroke_widths
from openpilot.selfdrive.navigation.route import RouteTracker
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
    self.controls_calls: list = []  # (steps, thread name) per call
    self.status = MagicMock()
    self.gate: threading.Event | None = None
    self.positions = []  # what position() said once the "download" was done
    self.ahead_result: tuple = ([], 'ok', [])  # what controls_ahead() gives (controls, status, road), or an exception it raises
    self.ahead_calls: list = []  # (position, bearing, thread name) per call

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
    if isinstance(self.controls_result, Exception):
      raise self.controls_result
    return self.controls_result

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
    self.sm.__getitem__.return_value = SimpleNamespace(hasFix=True, latitude=0, longitude=0, bearingDeg=90)
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

  def test_controls_computed_by_the_route_job_once(self):
    self.engine.background = True
    self.engine.router.steps = self.response
    self.engine.calculate_route((0.002, 0))
    self.engine.job.join(5.)
    self.engine.collect_route((0.002, 0))
    for _ in range(3):
      self.engine.send_instruction()
    self.assertEqual([name for _, name in self.engine.router.controls_calls], ['navd-route'])

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
    self.load_route()  # the new route
    self.engine.last_position = (0, 0)
    self.engine.send_instruction()
    self.assertEqual(counts(), [(True, 1), (False, 1), (False, 1), (True, 2)])

  def test_maneuvers_and_rerouting(self):
    self.load_route()
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
    self.fix = SimpleNamespace(hasFix=True, latitude=0, longitude=0, bearingDeg=90)
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
    self.fix = SimpleNamespace(hasFix=True, latitude=0, longitude=0.0005, bearingDeg=80)  # the car moved on meanwhile
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
    self.fix = SimpleNamespace(hasFix=True, latitude=0, longitude=0.001, bearingDeg=90)
    self.tick()
    self.finish()
    self.assertTrue(self.tick().valid)
    self.router.gate = threading.Event()  # the next route call is the new corridor's download: minutes
    calls = len(self.router.calls)
    for t in range(1, 120):  # the car leaves the route northward at 30 m/s
      self.fix = SimpleNamespace(hasFix=True, latitude=t * 30 / 111319.5, longitude=0.001, bearingDeg=0)
      msg = self.tick()
      self.assertIn('NavDestination', self.values, f"navd deleted the destination at {t} s")
      if len(self.router.calls) > calls:
        self.assertFalse(msg.valid, f"guidance on the old route at {t} s: {msg.navInstruction.maneuverDistance} m")
        self.assertEqual(self.engine.step_idx, 0)
    self.assertGreater(len(self.router.calls), calls, "navd never rerouted")
    self.fix = SimpleNamespace(hasFix=True, latitude=0, longitude=0.001, bearingDeg=90)
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
