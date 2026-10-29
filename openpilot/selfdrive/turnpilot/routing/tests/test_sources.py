import errno
import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import requests

from openpilot.selfdrive.navigation.navd import RouteEngine
from openpilot.selfdrive.turnpilot.maps import corridor
from openpilot.selfdrive.turnpilot.maps.client import MapError, storage_error
from openpilot.selfdrive.turnpilot.maps.corridor import KEEP_PACKS
from openpilot.selfdrive.turnpilot.maps.coverage import coverage_path, save
from openpilot.selfdrive.turnpilot.maps.status import STATUS_PARAM, read_status
from openpilot.selfdrive.turnpilot.maps.tests.fakes import (CHICAGO, INDY_EAST, KEY, SEATTLE, MapServer, coverage_doc, level2_tile, make_pack,
                                                            self_signed, tile_files)
from openpilot.selfdrive.turnpilot.routing import valhalla_source
from openpilot.selfdrive.turnpilot.routing.osm_sidecar import NO_DIRECTORY
from openpilot.selfdrive.turnpilot.routing.sources import PACKS_DIR, REFUSED_RETRY_SECONDS, RETRY_SECONDS, MapRouter
from openpilot.selfdrive.turnpilot.routing.tests.test_valhalla_routing import HAVE_VALHALLA, TURN_LANES_OSM, build_tiles
from openpilot.selfdrive.turnpilot.routing.valhalla_source import ValhallaSource, locate, tiles_usable

FIXTURES = Path(__file__).parent / 'fixtures'
RESPONSE = json.loads((FIXTURES / 'valhalla_left_then_right.json').read_text())
STEPS = RESPONSE['routes'][0]['legs'][0]['steps']
START, DESTINATION = (-86.1699, 39.77), (-86.16, 39.774)


class FakeActor:
  def __init__(self, config):
    self.config, self.requests = config, []

  def route(self, request):
    self.requests.append(request)
    return RESPONSE


def fake_valhalla():
  return SimpleNamespace(Actor=FakeActor, get_config=lambda **kwargs: kwargs)


def params_of(values):
  params = MagicMock()
  params.get.side_effect = values.get
  return params


class TestLocate(unittest.TestCase):
  def setUp(self):
    self.directory = tempfile.TemporaryDirectory()
    self.addCleanup(self.directory.cleanup)
    self.tiles = Path(self.directory.name)
    (self.tiles / 'tiles').mkdir()
    (self.tiles / 'tiles.tar').write_bytes(b'')

  def test_locate(self):
    root = self.tiles
    self.assertEqual(locate(str(root)), {'extract': str(root / 'tiles.tar'), 'dir': str(root / 'tiles')})
    (root / 'tiles.tar').unlink()
    self.assertEqual(locate(str(root)), {'extract': '', 'dir': str(root / 'tiles')})
    (root / 'tiles').rmdir()
    self.assertIsNone(locate(str(root)))
    (root / 'valhalla.json').write_text('{}')
    self.assertEqual(locate(str(root)), {'config': str(root / 'valhalla.json')})
    self.assertEqual(locate(str(root / 'valhalla.json')), {'config': str(root / 'valhalla.json')})
    (root / 'valhalla.json').unlink()
    (root / '2').mkdir()  # a bare tile hierarchy
    self.assertEqual(locate(str(root)), {'extract': '', 'dir': str(root)})
    (root / 'extract.tar').write_bytes(b'')
    self.assertEqual(locate(str(root / 'extract.tar')), {'extract': str(root / 'extract.tar'), 'dir': ''})
    (root / 'notes.txt').write_text('')
    self.assertIsNone(locate(str(root / 'notes.txt')))
    self.assertEqual(locate('http://127.0.0.1:8002/'), {'url': 'http://127.0.0.1:8002'})
    self.assertFalse(tiles_usable(None) or tiles_usable(''))


class TestValhalla(unittest.TestCase):
  def test_request(self):
    with patch.object(valhalla_source, 'import_valhalla', fake_valhalla), tempfile.TemporaryDirectory() as tiles:
      (Path(tiles) / 'tiles').mkdir()
      source = ValhallaSource(tiles)
      self.assertEqual(source.route(START, DESTINATION, 92.4, 'en'), STEPS)
    request = source.actor.requests[0]
    self.assertEqual(request['locations'], [{'lon': -86.1699, 'lat': 39.77, 'heading': 92, 'heading_tolerance': 90}, {'lon': -86.16, 'lat': 39.774}])
    expected = {'costing': 'auto', 'format': 'osrm', 'shape_format': 'geojson', 'banner_instructions': True, 'voice_instructions': True, 'language': 'en'}
    self.assertEqual({k: request[k] for k in expected}, expected)
    request = source.request(START, DESTINATION, -0.4, None)  # no language without one, and the heading wraps
    self.assertEqual(request['locations'][0]['heading'], 0)
    self.assertNotIn('language', request)
    self.assertNotIn('heading', source.request(START, DESTINATION, None, None)['locations'][0])
    for bearing in (float('nan'), float('inf')):
      self.assertNotIn('heading', source.request(START, DESTINATION, bearing, None)['locations'][0])
    self.assertEqual(source.request(START, DESTINATION, 300.2, None)['locations'][0]['heading'], 300)

  def test_config_file(self):
    with patch.object(valhalla_source, 'import_valhalla', fake_valhalla), tempfile.TemporaryDirectory() as directory:
      config = Path(directory) / 'valhalla.json'
      config.write_text('{}')
      self.assertEqual(ValhallaSource(str(config)).actor.config, str(config))

  def test_http_fallback(self):
    response = MagicMock(status_code=200)
    response.json.return_value = RESPONSE
    with patch.object(valhalla_source, 'import_valhalla', return_value=None), patch('requests.post', return_value=response) as post:
      source = ValhallaSource('http://localhost:8002/')
      self.assertEqual(source.route(START, DESTINATION, None, None), STEPS)
      self.assertEqual(post.call_args.args[0], 'http://localhost:8002/route')
      self.assertEqual(post.call_args.kwargs['json']['format'], 'osrm')
      with tempfile.TemporaryDirectory() as tiles:  # tiles but no pyvalhalla: a local valhalla_service
        (Path(tiles) / 'tiles').mkdir()
        source = ValhallaSource(tiles)
        self.assertIsNone(source.actor)
        self.assertEqual(source.route(START, DESTINATION, None, None), STEPS)
        self.assertEqual(post.call_args.args[0], 'http://127.0.0.1:8002/route')
        with patch.dict('os.environ', {'TURNPILOT_VALHALLA_URL': 'http://127.0.0.1:9999'}):
          ValhallaSource(tiles).route(START, DESTINATION, None, None)
          self.assertEqual(post.call_args.args[0], 'http://127.0.0.1:9999/route')
      response.raise_for_status.side_effect = requests.HTTPError('no route')
      with self.assertRaises(requests.HTTPError):
        source.route(START, DESTINATION, None, None)
      response.raise_for_status.side_effect = None
      response.json.return_value = {'routes': []}
      self.assertEqual(source.route(START, DESTINATION, None, None), [])

  def test_device_install_on_path(self):
    marker = object()
    with tempfile.TemporaryDirectory() as device, patch.dict('os.environ', {'TURNPILOT_PYVALHALLA': device}):
      with patch('importlib.import_module', side_effect=[ImportError, marker]), patch.object(sys, 'path', list(sys.path)):
        self.assertIs(valhalla_source.import_valhalla(), marker)
        self.assertEqual(sys.path[-1], device)
      with patch('importlib.import_module', side_effect=ImportError), patch.object(sys, 'path', list(sys.path)):
        self.assertIsNone(valhalla_source.import_valhalla())  # an install that doesn't import
    with patch.dict('os.environ', {'TURNPILOT_PYVALHALLA': '/nonexistent/pyvalhalla'}), patch('importlib.import_module', side_effect=ImportError):
      self.assertIsNone(valhalla_source.import_valhalla())
    with patch('importlib.import_module', return_value=marker), patch.object(sys, 'path', list(sys.path)) as path:
      self.assertIs(valhalla_source.import_valhalla(), marker)  # a system install wins and the path is left alone
      self.assertEqual(path, sys.path)


class TestValhallaJunctions(unittest.TestCase):
  """ValhallaSource's side of the junctions (SG1b): a route's opposing roads come from its controls' one batched Q2 locate,
  read alone only when that didn't run, and its lookups ahead keep Q1 and Q2 in the source's own LocateCache."""

  def source(self):
    from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import StubJunction, leg
    stub = StubJunction([(0., 100., {'stop': True}), (100., 200., {}), (200., 400., {})],
                        {100.: [leg(90.), leg(270.)], 200.: [leg(0., kph=56, **{'in': 2}), leg(90.), leg(270., exit=True)]})
    with patch.object(valhalla_source, 'import_valhalla', return_value=None):
      source = ValhallaSource('http://127.0.0.1:1')
    source._call = lambda: stub
    return source, stub

  def test_the_opposing_roads_ride_in_the_controls_q2(self):
    from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import opposing_steps
    source, stub = self.source()
    steps = opposing_steps()
    controls, status = source.controls(steps, sidecar_dir=None)
    self.assertEqual((status, [c.kind for c in controls]), ('ok', ['stop']))
    before = len(stub.requests)
    self.assertEqual(source.opposing(steps), {0: (56 / 3.6, 2)})
    self.assertEqual(len(stub.requests), before)  # no third call
    self.assertEqual(source.opposing(list(steps)), {0: (56 / 3.6, 2)})  # other steps: read alone
    self.assertEqual(len(stub.requests), before + 1)
    with tempfile.TemporaryDirectory() as unmarked:  # unsupported tiles: no Q2, the opposing roads read alone all the same
      steps = opposing_steps()
      self.assertEqual(source.controls(steps, unmarked, sidecar_dir=None), ([], 'unsupported'))
      self.assertEqual(source.opposing(steps), {0: (56 / 3.6, 2)})

  def test_lookups_ahead_keep_q1_and_q2_in_the_sources_cache(self):
    from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import StubRoadJunction, leg, pt
    source, _ = self.source()
    stub = StubRoadJunction([(0., 100., {'stop': True}), (100., 400., {})], road=[(0., 100.), (100., 400.)],
                            junctions={100.: [leg(90.), leg(270.)]})
    source._call = lambda: stub
    first = source.controls_ahead(pt(0.), 3., sidecar_dir=None)
    stub.requests.clear()
    again = source.controls_ahead(pt(4.), 3., sidecar_dir=None)
    self.assertEqual([c.kind for c in again[0]], [c.kind for c in first[0]])
    self.assertEqual([r for e, r in stub.requests if e == 'locate' and 'heading' not in r['locations'][0]], [])


class FakeValhalla:
  """Stands in for ValhallaSource on a corridor or the local tiles: records what it was built on and routed."""

  def __init__(self, test):
    self.test = test

  def __call__(self, tiles):
    if self.test.broken_tiles:
      raise RuntimeError('tiles do not load')
    # as ValhallaSource: a valhalla_service URL has no tiles directory
    source = SimpleNamespace(tiles=tiles, calls=[], tiles_dir=None if tiles.startswith('http') else tiles, controls_calls=[], contexts=[])

    def controls(steps, tiles_dir=None, *, sidecar_dir, context=None):
      source.controls_calls.append((steps, tiles_dir, sidecar_dir))
      source.contexts.append(context)
      return [], 'ok'

    source.controls = controls

    def controls_ahead(position, heading, tiles_dir=None, *, sidecar_dir, context=None):
      source.controls_calls.append(('ahead', position, heading, tiles_dir, sidecar_dir))
      source.contexts.append(context)
      return [], 'ok', [position, position]

    source.controls_ahead = controls_ahead

    def opposing(steps):
      source.controls_calls.append(('opposing', steps))
      return {0: (11.11, 1)}

    source.opposing = opposing

    def route(start, destination, bearing, language):
      source.calls.append((start, destination, bearing, language))
      if self.test.route_error is not None:
        raise self.test.route_error
      return self.test.steps

    source.route = route
    self.test.built.append(source)
    return source


class RouterCase(unittest.TestCase):
  """MapRouter against a fake map server over real TLS, with Valhalla stubbed."""

  def setUp(self):
    self.server = MapServer()
    self.addCleanup(self.server.close)
    directory = tempfile.TemporaryDirectory()
    self.addCleanup(directory.cleanup)
    self.maps = Path(directory.name)
    save(coverage_doc(), self.maps)
    self.server.corridor = (200, make_pack({level2_tile(*START): b'tile'}), 'application/x-tar')
    self.params = self.server.params()
    self.steps, self.route_error, self.broken_tiles, self.built = STEPS, None, False, []
    self.router = MapRouter(self.params, self.maps, valhalla=FakeValhalla(self))
    # Mapbox went through requests.get; the map client only uses requests.request
    self.enterContext(patch('requests.get', side_effect=AssertionError('nothing may call requests.get (Mapbox)')))

  def status(self):
    return read_status(self.params)

  def states(self):
    return [value['state'] for key, value in self.params.writes if key == STATUS_PARAM]

  def refused(self, code, start=START, destination=DESTINATION):
    with self.assertRaises(MapError) as e:
      self.router.route(start, destination, 90, 'en')
    self.assertEqual(e.exception.code, code, e.exception.message)
    status = self.status()
    self.assertEqual((status['state'], status['code'], status['message']), ('error', code, e.exception.message))
    return e.exception


class TestMapRouter(RouterCase):
  def test_routes_on_the_downloaded_corridor(self):
    self.assertEqual(self.router.route(START, DESTINATION, 90, 'en'), STEPS)
    self.assertEqual(self.server.corridor_requests(), [{'start': list(START), 'destination': list(DESTINATION), 'osm_controls': True}])
    (source,) = self.built
    pack = Path(source.tiles)
    self.assertEqual(pack.parent, self.maps / PACKS_DIR)
    self.assertEqual((pack / level2_tile(*START)).read_bytes(), b'tile')
    self.assertEqual(source.calls, [(START, DESTINATION, 90, 'en')])
    self.assertEqual(self.states(), ['checking', 'downloading', 'downloading', 'ready'])
    status = self.status()
    self.assertEqual((status['region'], status['source'], status['short']), ('Indianapolis area', 'server', 'Ready'))
    downloads = [v for k, v in self.params.writes if k == STATUS_PARAM and v['state'] == 'downloading']
    self.assertEqual(downloads[0]['message'], 'Downloading map for this drive…')
    self.assertEqual(downloads[-1]['progress'], 1.0)
    self.assertEqual(list((self.maps / PACKS_DIR).glob('.download-*')), [])  # the tar is gone once unpacked

  def test_corridor_controls_need_no_marker_and_read_the_packs_sidecar(self):
    # tiles from real OSM carry their controls (no marker); the OSM stop-sign sidecar is the pack's own osm_controls/,
    # checked against its manifest's osm_timestamp (routing/osm_sidecar.py), for the route and the road ahead off it
    steps = self.router.route(START, DESTINATION, 90, 'en')
    self.router.controls(steps)
    pack = self.router.corridor.pack.path
    self.assertEqual(self.built[0].controls_calls, [(steps, None, pack)])
    self.router.controls_ahead(START, 90.)
    self.assertEqual(self.built[0].controls_calls[-1], ('ahead', START[::-1], 90., None, pack))

  def test_reroutes_on_the_same_corridor(self):
    self.router.route(START, DESTINATION, 90, 'en')
    elsewhere = (-86.165, 39.771)
    self.assertEqual(self.router.route(elsewhere, DESTINATION, 0, 'en'), STEPS)
    self.assertEqual(len(self.server.corridor_requests()), 1)  # no download for a reroute
    self.assertEqual(self.built[0].calls[-1], (elsewhere, DESTINATION, 0, 'en'))

  def test_leaving_the_corridor_downloads_a_new_one(self):
    """Valhalla snaps a start far outside the corridor's level-2 tiles to a highway on its level 0/1 tiles and answers;
    that must not count as a reroute on the corridor."""
    self.router.route(START, DESTINATION, 90, 'en')
    outside = (-86.4, 40.25)  # inside the Indianapolis fence, outside the corridor's tiles
    self.server.corridor = (200, make_pack({level2_tile(*START): b'tile', level2_tile(*outside): b'more'}), 'application/x-tar')
    self.params.writes.clear()
    self.assertEqual(self.router.route(outside, DESTINATION, 0, 'en'), STEPS)
    self.assertEqual(self.server.corridor_requests()[-1], {'start': list(outside), 'destination': list(DESTINATION), 'osm_controls': True})
    self.assertEqual(self.built[-1].calls, [(outside, DESTINATION, 0, 'en')])
    self.assertEqual(self.states(), ['downloading', 'downloading', 'ready'])  # the driver sees the new download
    self.assertEqual(self.status()['destination'], list(DESTINATION))

  def test_a_reroute_that_leaves_the_corridor_downloads_a_new_one(self):
    self.router.route(START, DESTINATION, 90, 'en')
    far = [{**STEPS[0], 'geometry': {'type': 'LineString', 'coordinates': [list(START), [-85.9, 39.8]]}}] + STEPS[1:]
    self.steps = far
    with self.assertRaises(MapError) as e:  # the new corridor (the same tiles here) still doesn't hold the route: loud
      self.router.route(START, DESTINATION, 90, 'en')
    self.assertEqual(e.exception.code, 'route_off_map')
    self.assertEqual(len(self.server.corridor_requests()), 2)
    self.refused('route_off_map')  # and not downloaded again within the retry wait
    self.assertEqual(len(self.server.corridor_requests()), 2)

  def test_routing_failure_inside_the_corridor_is_an_error_not_a_download(self):
    self.router.route(START, DESTINATION, 90, 'en')
    self.route_error = RuntimeError('No suitable edges near location')
    self.refused('routing_failed', start=(-86.1, 39.8))  # still on the corridor's tiles
    self.assertEqual(len(self.server.corridor_requests()), 1)
    self.route_error = None
    self.steps = []
    self.refused('no_route')
    self.assertEqual(len(self.server.corridor_requests()), 1)

  def test_routes_from_where_the_car_is_once_the_download_is_done(self):
    later = (-86.165, 39.771)
    self.assertEqual(self.router.route(START, DESTINATION, 90, 'en', position=lambda: (later, 45)), STEPS)
    self.assertEqual(self.server.corridor_requests(), [{'start': list(START), 'destination': list(DESTINATION), 'osm_controls': True}])
    self.assertEqual(self.built[0].calls, [(later, DESTINATION, 45, 'en')])

  def test_a_cancelled_download_stops_quietly(self):
    with self.assertRaises(MapError) as e:
      self.router.route(START, DESTINATION, 90, 'en', cancelled=lambda: True)
    self.assertEqual(e.exception.code, 'cancelled')
    self.assertNotEqual(self.status()['state'], 'error')
    self.assertIsNone(self.router.failure)
    self.assertEqual(corridor.packs(self.maps / PACKS_DIR), [])
    self.assertEqual(self.router.route(START, DESTINATION, 90, 'en'), STEPS)  # the same destination again goes at once

  def test_every_status_names_its_destination(self):
    self.router.route(START, DESTINATION, 90, 'en')
    self.assertTrue(all(v['destination'] == list(DESTINATION) for k, v in self.params.writes if k == STATUS_PARAM))

  def test_new_destinations_keep_the_last_few_packs(self):
    for i in range(KEEP_PACKS + 2):
      self.router.route(START, (-86.16 + i * 0.001, 39.774), 90, 'en')
    self.assertEqual(len(self.server.corridor_requests()), KEEP_PACKS + 2)
    kept = corridor.packs(self.maps / PACKS_DIR)
    self.assertEqual(len(kept), KEEP_PACKS)
    self.assertIn(self.router.corridor.pack.path, kept)

  def test_destination_outside_the_mapped_area(self):
    e = self.refused('outside_coverage', destination=CHICAGO)
    self.assertEqual(e.message, "Outside TurnPilot's mapped area. TurnPilot can only navigate around Indianapolis and Seattle.")
    self.assertEqual(self.server.requests, [])  # refused before any download
    self.assertEqual(self.built, [])

  def test_car_outside_or_in_another_area(self):
    self.assertIn('The car is outside', self.refused('outside_coverage', start=CHICAGO).message)
    self.refused('different_regions', start=SEATTLE)
    self.assertEqual(self.server.requests, [])

  def test_no_coverage_is_fetched_once_or_is_a_loud_error(self):
    coverage_path(self.maps).unlink()
    self.assertEqual(self.router.route(START, DESTINATION, 90, 'en'), STEPS)  # online: navd fetches it first
    self.assertTrue(coverage_path(self.maps).is_file())
    coverage_path(self.maps).unlink()
    self.params.values['TurnPilotMapKey'] = 'wrong'
    e = self.refused('no_coverage', destination=INDY_EAST)
    self.assertIn('never downloaded', e.message)
    self.assertIn('refused the API key', e.message)  # and why fetching it failed

  def test_each_422(self):
    for body, code in [({'error': 'outside_coverage', 'which': 'destination'}, 'outside_coverage'),
                       ({'error': 'outside_coverage', 'which': 'start'}, 'outside_coverage'),
                       ({'error': 'different_regions'}, 'different_regions'),
                       ({'error': 'no_route'}, 'no_route')]:
      self.router.failure = None
      self.server.corridor = (422, json.dumps(body).encode(), 'application/json')
      self.refused(code)
      self.assertIsNone(self.router.corridor)
    self.assertEqual(self.built, [])

  def test_bad_key_bad_tls_bad_pack(self):
    self.params.values['TurnPilotMapKey'] = 'wrong'
    self.assertIn('API key', self.refused('unauthorized').message)
    self.params.values['TurnPilotMapKey'] = KEY
    self.params.values['TurnPilotMapCert'] = self_signed()[0]  # a certificate that isn't the server's
    self.refused('tls')
    self.params.values['TurnPilotMapCert'] = self.server.cert
    self.server.corridor = (200, make_pack({'tiles/2/0.gph': b'tile'}, manifest_files={'tiles/2/0.gph': '0' * 64}), 'application/x-tar')
    self.refused('bad_pack')
    self.assertEqual(corridor.packs(self.maps / PACKS_DIR), [])
    self.assertEqual(self.built, [])

  def test_unreachable_server_is_an_error_and_retried_later(self):
    self.server.close()
    e = self.refused('unreachable')
    self.assertIn("Can't reach", e.message)
    self.assertIsNone(self.router.corridor)
    with patch('openpilot.selfdrive.turnpilot.maps.client.requests.request') as request:
      self.refused('unreachable')  # within RETRY_SECONDS the same error, with no new attempt
      request.assert_not_called()
    with patch('openpilot.selfdrive.turnpilot.routing.sources.time.monotonic', return_value=time.monotonic() + RETRY_SECONDS + 1):
      with patch('openpilot.selfdrive.turnpilot.maps.client.requests.request', side_effect=requests.ConnectionError) as request:
        self.refused('unreachable')
        request.assert_called_once()

  def test_a_cut_off_download_backs_off_before_downloading_again(self):
    """A clean hang-up mid-stream with no Content-Length: one download, a visible error, then the retry wait."""
    self.server.corridor = (200, make_pack({f'tiles/2/000/000/{i:03d}.gph': os.urandom(4000) for i in range(10)}), 'application/x-tar')
    for gz in (True, False):
      self.router.failure = None
      self.server.requests.clear()
      self.server.gzip, self.server.send_length, self.server.cut_at = gz, False, 20000
      e = self.refused('unreachable' if gz else 'bad_pack')
      self.assertTrue(e.retry)
      self.refused(e.code)
      self.assertEqual(len(self.server.corridor_requests()), 1)  # the second try waits for RETRY_SECONDS
      with patch('openpilot.selfdrive.turnpilot.routing.sources.time.monotonic', return_value=time.monotonic() + RETRY_SECONDS + 1):
        self.refused(e.code)
      self.assertEqual(len(self.server.corridor_requests()), 2)

  def test_any_download_failure_is_a_map_error_with_the_backoff(self):
    with patch.object(corridor, 'unpack', side_effect=RuntimeError('surprise')):
      e = self.refused('download_failed')
      self.assertIn('RuntimeError: surprise', e.message)
      self.refused('download_failed')
    self.assertEqual(len(self.server.corridor_requests()), 1)

  def test_a_full_disk_is_said_and_not_downloaded_again_every_30_s(self):
    """The comma's own storage: a new download won't fix it, so it waits out the refusal time, not RETRY_SECONDS. The real
    unpack's ENOSPC is tested in maps/tests/test_corridor.py; here, its error and a full disk under the router itself."""
    full = OSError(errno.ENOSPC, 'No space left on device')
    for target in (patch.object(corridor, 'unpack', side_effect=storage_error(full)), patch.object(Path, 'mkdir', side_effect=full)):
      self.router.failure, self.router.corridor = None, None
      self.server.requests.clear()
      with target:
        e = self.refused('storage')
        self.assertFalse(e.retry)
        self.assertIn('storage is full', e.message)
        with patch('openpilot.selfdrive.turnpilot.routing.sources.time.monotonic', return_value=time.monotonic() + RETRY_SECONDS + 1):
          self.refused('storage')  # no new download 30 s later
      self.assertLessEqual(len(self.server.corridor_requests()), 1)
      with patch('openpilot.selfdrive.turnpilot.routing.sources.time.monotonic', return_value=time.monotonic() + REFUSED_RETRY_SECONDS + 1):
        self.assertEqual(self.router.route(START, DESTINATION, 90, 'en'), STEPS)  # space was freed meanwhile

  def test_a_job_cancelled_while_waiting_on_the_server_leaves_no_trace(self):
    """navd doesn't wait for a cancelled job: it can fail or finish after the next destination started, and then it must
    not write the status, a failure to wait out, or the corridor."""
    real = self.router.client_factory
    cancel = []

    def cancelled_while_the_server_builds(*args):
      client = real(*args)
      download = client.corridor

      def corridor_call(start, destination, out, progress=None, coverage_names=''):
        cancel.append(True)  # navd moved on to INDY_EAST, which is the status now
        self.router.status.set('checking', 'Checking the mapped area', destination=list(INDY_EAST))
        return download(start, destination, out, None, coverage_names)
      client.corridor = corridor_call
      return client

    self.router.client_factory = cancelled_while_the_server_builds
    for reply in ((503, b'{}', 'application/json'), self.server.corridor):  # A's request fails, or completes
      self.server.corridor = reply
      cancel.clear()
      with self.assertRaises(MapError) as e:
        self.router.route(START, DESTINATION, 90, 'en', cancelled=lambda: bool(cancel))
      self.assertEqual(self.status()['destination'], list(INDY_EAST), e.exception.code)
      self.assertIsNone(self.router.failure)
      self.assertIsNone(self.router.corridor)

  def test_routing_that_fails_on_a_fresh_corridor_is_not_downloaded_again(self):
    self.route_error = RuntimeError('Locations are in unconnected regions')
    self.refused('routing_failed')
    for _ in range(3):
      self.refused('routing_failed')
    self.assertEqual(len(self.server.corridor_requests()), 1)
    self.route_error = None
    self.assertEqual(self.router.route(START, DESTINATION, 90, 'en'), STEPS)  # it works once routing does

  def test_settings_changes_retry_at_once(self):
    self.params.values['TurnPilotMapKey'] = 'wrong'
    self.refused('unauthorized')
    self.params.values['TurnPilotMapKey'] = KEY
    self.assertEqual(self.router.route(START, DESTINATION, 90, 'en'), STEPS)

  def test_not_set_up(self):
    del self.params.values['TurnPilotMapKey']
    self.assertIn('not set up', self.refused('not_configured').message)

  def test_mapbox_is_gone(self):
    self.params.values['TurnPilotRouteSource'] = 'mapbox'
    self.refused('mapbox_removed')
    self.params.values['TurnPilotRouteSource'] = 'valhalla'  # what the BeamNG scripts set: harmless
    self.assertEqual(self.router.route(START, DESTINATION, 90, 'en'), STEPS)

  def test_clear(self):
    self.router.route(START, DESTINATION, 90, 'en')
    self.router.clear()
    self.assertEqual(self.status()['state'], 'idle')


class TestLocalOverride(RouterCase):
  def test_local_tiles_skip_the_geofence_and_the_download(self):
    self.params.values['TurnPilotValhallaTiles'] = '/sim/tiles'
    coverage_path(self.maps).unlink()  # the sim has no coverage
    self.assertEqual(self.router.route(CHICAGO, CHICAGO, 90, 'en'), STEPS)
    self.assertEqual([s.tiles for s in self.built], ['/sim/tiles'])
    self.assertEqual(self.server.requests, [])
    status = self.status()
    self.assertEqual((status['state'], status['source']), ('ready', 'local'))
    self.router.route(CHICAGO, CHICAGO, 90, 'en')
    self.assertEqual(len(self.built), 1)  # the actor is kept

  def test_opposing_roads_from_the_source_that_routed(self):
    # the unprotected left's opposing speeds (routing/controls.py: opposing_roads) come from the source that routed the
    # route; another route's steps get none (unknown), said
    self.params.values['TurnPilotValhallaTiles'] = '/sim/tiles'
    coverage_path(self.maps).unlink()
    steps = self.router.route(CHICAGO, CHICAGO, 90, 'en')
    self.assertEqual(self.router.opposing(steps), {0: (11.11, 1)})
    self.assertEqual(self.built[0].controls_calls, [('opposing', steps)])
    self.assertEqual(self.router.opposing(list(steps)), {})

  def test_controls_from_the_source_that_routed(self):
    # local tiles: checked for their controls (marker, sidecar); a corridor from real OSM: not; another route: failed
    self.params.values['TurnPilotValhallaTiles'] = '/sim/tiles'
    coverage_path(self.maps).unlink()
    steps = self.router.route(CHICAGO, CHICAGO, 90, 'en')
    self.assertEqual(self.router.controls(steps), ([], 'ok'))
    self.assertEqual(self.built[0].controls_calls, [(steps, '/sim/tiles', '/sim/tiles')])  # the local tiles hold their sidecar
    self.assertEqual(self.router.controls(list(steps)), ([], 'failed'))

  def test_the_junctions_read_the_surveys_and_the_overrides_param(self):
    """The controls' junctions are read against surveyed_junctions.json and TurnPilotJunctionOverrides, on the route and
    off it; a param this build doesn't know is not readable, which controls.py says (every junction failed, partial)."""
    from openpilot.common.params import UnknownKeyName
    from openpilot.selfdrive.turnpilot.maps import surveys
    self.params.values['TurnPilotValhallaTiles'] = '/sim/tiles'
    self.params.values['TurnPilotJunctionOverrides'] = [{'approach': '900:790:f', 'kind': 'no', 'evidence': 'a runner at 8 m/s'}]
    coverage_path(self.maps).unlink()
    entry = {'osmNodeId': 900, 'lat': 39.8, 'lon': -86.15, 'date': '2026-10-20', 'source': 'inPerson', 'confirmedBy': [],
             'approaches': [{'wayId': 1, 'bearingDeg': 0, 'stops': True, 'plaque': True, 'postedMph': 20, 'laneEdgeM': 4.,
                             'lineM': 7., 'nearestParkingM': {'left': 40., 'right': 40.}, 'sightM': 60.}], 'validations': []}
    with tempfile.TemporaryDirectory() as d:
      path = Path(d) / 'surveyed_junctions.json'
      path.write_text(json.dumps([entry]))
      with patch.object(surveys, 'SHIPPED', path):
        steps = self.router.route(CHICAGO, CHICAGO, 90, 'en')
        self.router.controls(steps)
        self.router.controls_ahead((-87.6, 41.8), 90.)
    for ctx in self.built[0].contexts:
      self.assertEqual((ctx.overrides_ok, set(ctx.overrides), [e['osmNodeId'] for e in ctx.surveys]), (True, {'900:790:f'}, [900]))

    def unknown(key, *args, **kwargs):
      if key == 'TurnPilotJunctionOverrides':
        raise UnknownKeyName(key)
      return self.params.values.get(key)
    with patch.object(self.params, 'get', side_effect=unknown):
      self.router.controls(steps)
    ctx = self.built[0].contexts[-1]
    self.assertFalse(ctx.overrides_ok)
    self.assertIn('unknown to this build', ctx.overrides_error)

  def test_controls_ahead_on_the_local_tiles_before_any_route(self):
    """Off the route before the first route is in (navd's off-route lookup), the local tiles answer: the sim's first
    seconds aren't unknown for want of a route. A corridor's source exists only once it is downloaded: failed till then."""
    self.assertEqual(self.router.controls_ahead((-87.6, 41.8), 90.)[:2], ([], 'failed'))
    self.params.values['TurnPilotValhallaTiles'] = '/sim/tiles'
    self.assertEqual(self.router.controls_ahead((-87.6, 41.8), 90.)[:2], ([], 'ok'))
    self.assertEqual(self.built[0].controls_calls, [('ahead', (41.8, -87.6), 90., '/sim/tiles', '/sim/tiles')])
    self.router.route(CHICAGO, CHICAGO, 90, 'en')
    self.assertEqual(len(self.built), 1)  # the same actor routes

  def test_a_valhalla_service_has_no_directory_for_a_sidecar_which_is_said(self):
    # a sidecar is expected everywhere: a valhalla_service URL has none to read, which controls.py says (partial)
    self.params.values['TurnPilotValhallaTiles'] = 'http://127.0.0.1:8002'
    coverage_path(self.maps).unlink()
    steps = self.router.route(CHICAGO, CHICAGO, 90, 'en')
    self.router.controls(steps)
    self.assertEqual(self.built[0].controls_calls, [(steps, None, NO_DIRECTORY)])

  def test_local_tiles_fail_loudly(self):
    self.params.values['TurnPilotValhallaTiles'] = '/sim/tiles'
    self.broken_tiles = True
    self.refused('local_tiles')
    self.broken_tiles = False
    self.steps = []
    self.refused('no_route')


class TestRouteEngine(RouterCase):
  def engine(self):
    pm = MagicMock()
    engine = RouteEngine(MagicMock(), pm, params=self.params, router=self.router, background=False)
    engine.last_position, engine.last_bearing = START, 90
    return engine, pm

  def test_routes_and_guides_on_the_corridor(self):
    self.params.values['LanguageSetting'] = 'main_en'
    engine, pm = self.engine()
    engine.calculate_route(DESTINATION)
    self.assertEqual((engine.step_idx, len(engine.route_geometry)), (0, 4))
    self.assertEqual(self.built[0].calls, [(START, DESTINATION, 90, 'en')])
    engine.last_position = (-86.1675, 39.77)
    engine.send_instruction()
    instruction = pm.send.call_args.args[1].navInstruction.to_dict()
    self.assertEqual((instruction['maneuverType'], instruction['maneuverModifier']), ('turn', 'left'))

  def test_refused_destination_means_no_route(self):
    engine, pm = self.engine()
    engine.calculate_route(CHICAGO)
    self.assertIsNone(engine.route)
    self.assertIsNone(engine.nav_destination)  # asked again later, as the car may drive into the area
    self.assertEqual(self.status()['code'], 'outside_coverage')
    self.assertEqual(self.server.requests, [])
    engine.send_instruction()
    self.assertFalse(pm.send.call_args.args[1].valid)

  def test_unreachable_server_means_no_route_and_no_mapbox(self):
    self.server.close()
    engine, _ = self.engine()
    engine.calculate_route(DESTINATION)
    self.assertIsNone(engine.route)
    self.assertEqual(self.status()['code'], 'unreachable')

  def test_an_unexpected_failure_is_reported(self):
    self.route_error = ValueError('boom')
    engine, _ = self.engine()
    engine.router.route = MagicMock(side_effect=ValueError('boom'))
    engine.calculate_route(DESTINATION)
    self.assertIsNone(engine.route)
    self.assertEqual((self.status()['code'], self.status()['message']), ('routing_failed', 'Routing failed: ValueError: boom'))

  def test_no_destination_clears_the_status(self):
    engine, _ = self.engine()
    engine.calculate_route(DESTINATION)
    engine.recompute_route(gps_ok=True)  # NavDestination isn't set
    self.assertEqual(self.status()['state'], 'idle')


@unittest.skipUnless(HAVE_VALHALLA, 'pyvalhalla is not installed')
class TestRealCorridor(unittest.TestCase):
  """Real tiles in a corridor pack, served over TLS, verified, unpacked and routed by navd with real pyvalhalla."""

  def test_navd_routes_on_a_served_corridor(self):
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      tiles = Path(build_tiles(TURN_LANES_OSM, root / 'build'))
      tile_dir = tiles / 'tiles' if (tiles / 'tiles').is_dir() else tiles
      server = MapServer()
      self.addCleanup(server.close)
      server.corridor = (200, make_pack(tile_files(tile_dir)), 'application/x-tar')
      maps = root / 'maps'
      save(coverage_doc(), maps)
      params = server.params(LanguageSetting='main_en')
      pm = MagicMock()
      engine = RouteEngine(MagicMock(), pm, params=params, router=MapRouter(params, maps), background=False)
      engine.last_position, engine.last_bearing = (-86.1699, 39.77), 90
      with patch('requests.get', side_effect=AssertionError('no Mapbox')):
        engine.calculate_route((-86.1600, 39.7740))
      self.assertIsNotNone(engine.route)
      self.assertEqual(engine.route[1]['maneuver']['modifier'], 'left')
      self.assertEqual(read_status(params)['state'], 'ready')


if __name__ == '__main__':
  unittest.main()
