"""TurnPilot, 2026-11-06: map tile errors that say which call failed and its HTTP status (never the signed URL), and mapsd
trying every 5 minutes, logging the status once, when comma's nav/tiles refuses the device. Both drives (0000001d,
0000001e) logged a bare 'Unable to fetch map tiles' every 10 s from the first route to the end, and the comma's map panel
stayed black."""
import re
import tempfile
import threading
import time
import traceback
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import requests

from openpilot.selfdrive.navigation import mapsd as mapsd_module
from openpilot.selfdrive.navigation.mapsd import REFUSED_RETRY_S, RETRY_SECONDS, MapsD, refusal
from openpilot.selfdrive.navigation.tests.test_navigation import make_archive
from openpilot.selfdrive.navigation.tiles import RemoteTiles, TileFetchError

SECRET = 'sig=secret-signature'


class Refuse(BaseHTTPRequestHandler):
  """The archive host: every range read answered with the server's status, or, at 206, with the archive's bytes."""
  def do_GET(self):
    if self.server.status == 206:
      offset, end = map(int, re.fullmatch(r'bytes=(\d+)-(\d+)', self.headers['Range']).groups())
      data = self.server.archive[offset:end + 1]
      self.send_response(206)
      self.send_header('Content-Range', f'bytes {offset}-{end}/{len(self.server.archive)}')
      self.send_header('Content-Length', str(len(data)))
      self.send_header('ETag', '"test-version"')
      self.end_headers()
      self.wfile.write(data)
      return
    self.send_response(self.server.status)
    self.send_header('Content-Length', '0')
    self.end_headers()

  def log_message(self, format, *args):  # noqa: A002
    pass


def http_error(status, url):
  """What response.raise_for_status() raises for a status: its message holds the URL."""
  response = requests.Response()
  response.status_code, response.url, response.reason = status, url, 'Forbidden'
  return requests.HTTPError(f'{status} Client Error: Forbidden for url: {url}', response=response)


class TilesCase(unittest.TestCase):
  """RemoteTiles with comma's API stubbed and the tile cache in a temporary directory, as on a PC."""

  def setUp(self):
    directory = tempfile.TemporaryDirectory()
    self.addCleanup(directory.cleanup)
    self.server = ThreadingHTTPServer(('127.0.0.1', 0), Refuse)
    self.server.status = 403
    self.server.archive = make_archive(leaf=True)  # one zoom-zero road tile, served once status is 206
    threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True).start()
    self.addCleanup(self.server.server_close)
    self.addCleanup(self.server.shutdown)
    self.url = f'http://127.0.0.1:{self.server.server_port}/planet.pmtiles?{SECRET}'
    self.api = MagicMock(dongle_id='test-dongle')
    self.api.get_token.return_value = 'device-token'
    self.api.get.return_value.json.return_value = {'url': self.url, 'expires': time.time() + 86400}  # noqa: TID251
    for name, value in (('Api', MagicMock(return_value=self.api)), ('Params', MagicMock()), ('PC', True)):
      self.enterContext(patch(f'openpilot.selfdrive.navigation.tiles.{name}', value))
    self.enterContext(patch('openpilot.selfdrive.navigation.tiles.Paths.comma_home', return_value=directory.name))

  def refuse_metadata(self, status):
    nav_url = f'https://api.commadotai.com/v1/test-dongle/nav/tiles?{SECRET}'
    self.api.get.return_value.raise_for_status.side_effect = http_error(status, nav_url)

  def fetch_error(self) -> TileFetchError:
    with self.assertRaises(TileFetchError) as raised:
      RemoteTiles()
    return raised.exception

  def assert_no_url(self, e):
    logged = ''.join(traceback.format_exception(e))  # what cloudlog.exception writes: message, notes and traceback
    for secret in (SECRET, '://', 'api.commadotai.com', '127.0.0.1', 'device-token'):
      self.assertNotIn(secret, logged)


class TestTileErrors(TilesCase):
  def test_a_refused_metadata_call_says_its_status_and_no_url(self):
    self.refuse_metadata(403)
    e = self.fetch_error()
    self.assertIsInstance(e, OSError)  # mapsd and PMTiles catch OSError, as before
    self.assertEqual(str(e), 'Unable to fetch map tiles (nav/tiles HTTP 403)')
    self.assertEqual((e.step, e.status), ('nav/tiles', 403))
    self.assertIsNone(e.__cause__)
    self.assertTrue(e.__suppress_context__)  # raised from None: requests' exception, with the URL, isn't chained
    self.assert_no_url(e)
    self.assertEqual(refusal(e), 403)
    for status in (401, 404):
      self.refuse_metadata(status)
      self.assertEqual(refusal(self.fetch_error()), status)

  def test_a_range_read_error_is_not_a_refusal(self):
    e = self.fetch_error()  # nav/tiles answers; the archive host says 403, as an expired signature would
    self.assertEqual(str(e), 'Unable to fetch map tiles (range HTTP 403)')
    self.assertEqual((e.step, e.status), ('range', 403))
    self.assert_no_url(e)
    self.assertIsNone(refusal(e))  # a fresh reader gets a fresh signed URL: retried
    self.refuse_metadata(500)
    self.assertIsNone(refusal(self.fetch_error()))  # comma's server failing isn't a refusal either

  def test_without_a_status_the_note_names_the_call(self):
    nav_url = f'https://api.commadotai.com/v1/test-dongle/nav/tiles?{SECRET}'
    self.api.get.side_effect = requests.ConnectionError(f'Max retries exceeded with url: {nav_url}')
    e = self.fetch_error()
    self.assertEqual(str(e), 'Unable to fetch map tiles')
    self.assertEqual((e.step, e.status), ('nav/tiles', None))
    self.assertIn('nav/tiles: ConnectionError', ''.join(traceback.format_exception(e)))
    self.assert_no_url(e)
    self.assertIsNone(refusal(e))
    self.api.get.side_effect = None
    self.api.get.return_value.json.return_value = {'expires': 0}  # answered 200 without the signed URL
    e = self.fetch_error()
    self.assertIn('nav/tiles: KeyError', ''.join(traceback.format_exception(e)))
    self.api.get.return_value.json.return_value = {'url': self.url, 'expires': time.time() + 86400}  # noqa: TID251
    self.server.shutdown()  # the archive host gone mid-drive
    self.server.server_close()
    e = self.fetch_error()
    self.assertEqual((str(e), e.step), ('Unable to fetch map tiles', 'range'))
    self.assertIn('range: ConnectionError', ''.join(traceback.format_exception(e)))
    self.assert_no_url(e)


class StubSM:
  """What MapsD.update reads: a route on navRoute, and a GPS fix only when fix is set (without one no frame renders, so
  only the fetch runs)."""
  def __init__(self, route):
    self.route = SimpleNamespace(coordinates=[SimpleNamespace(longitude=lon, latitude=lat) for lon, lat in route])
    self.updated = {'navRoute': True}
    self.fix = None  # (longitude, latitude) on gpsLocationExternal
    self.recv_time = dict.fromkeys(('gpsLocation', 'gpsLocationExternal'), 0.)
    self.logMonoTime = dict.fromkeys(self.recv_time, 1)

  @property
  def valid(self):
    return {'gpsLocation': False, 'gpsLocationExternal': self.fix is not None}

  def __getitem__(self, service):
    if service == 'navRoute':
      return self.route
    if service == 'gpsLocationExternal' and self.fix is not None:
      return SimpleNamespace(hasFix=True, longitude=self.fix[0], latitude=self.fix[1], horizontalAccuracy=0.)
    return SimpleNamespace(hasFix=False)


class TestMapsdRefusal(TilesCase):
  """mapsd against the real RemoteTiles, its clock and cloudlog patched: the 1e failure path, mapsd.py:43 -> tiles.py:30."""

  def setUp(self):
    super().setUp()
    self.now = 1000.
    self.enterContext(patch.object(mapsd_module, 'time', SimpleNamespace(monotonic=lambda: self.now)))
    self.log = self.enterContext(patch.object(mapsd_module, 'cloudlog'))
    self.mapsd = MapsD()
    self.addCleanup(self.mapsd.close)
    self.sm = StubSM([(0., -0.01), (0., 0.01)])

  def drive(self, seconds, render=True, step=1., black=True):
    """update() every step s, as mapsd's loop (20 Hz onroad, 1 Hz offroad, coarser here); the route arrives once. The
    clock of each frame that rendered."""
    end, lit = self.now + seconds, []
    while self.now < end:
      self.sm.recv_time['gpsLocationExternal'] = time.monotonic()  # helpers.latest_gps_source reads the real clock: just heard
      image, timestamp = self.mapsd.update(self.sm, render=render)
      if image.any():
        lit.append(self.now)
      else:
        self.assertFalse(timestamp)  # black, never stale
      if black:
        self.assertFalse(image.any())
      self.sm.updated['navRoute'] = False
      self.now += step
    return lit

  def refused_logs(self):
    return [c for c in self.log.event.call_args_list if c.args[0] == 'mapsd.tiles_refused']

  def test_a_refusal_is_logged_once_and_tried_every_five_minutes(self):
    self.refuse_metadata(403)
    self.drive(14 * 60)  # 1e: 14 segments, a route from seg0 t=36 s, and a failure every RETRY_SECONDS
    self.assertEqual(self.api.get.call_count, 3)  # at 0, 5 and 10 minutes, not every 10 s
    self.assertEqual(len(self.refused_logs()), 1)  # the same status: logged once
    self.assertEqual(self.refused_logs()[0].kwargs, {'error': True, 'step': 'nav/tiles', 'status': 403, 'retry_s': REFUSED_RETRY_S})
    self.log.exception.assert_not_called()  # no traceback every 10 s
    self.assertEqual(self.mapsd.refused, 403)
    # 1e rerouted in loops: a new route on the same drive doesn't start the tries over (the next is at 15 minutes)
    self.sm.route.coordinates.append(SimpleNamespace(longitude=0.001, latitude=0.01))
    self.sm.updated['navRoute'] = True
    self.drive(50)
    self.assertEqual(self.api.get.call_count, 3)
    # a different refusal is a change: logged
    self.refuse_metadata(401)
    self.drive(REFUSED_RETRY_S)
    self.assertEqual(self.api.get.call_count, 4)
    self.assertEqual([c.kwargs['status'] for c in self.refused_logs()], [403, 401])
    self.log.exception.assert_not_called()

  def test_a_later_answer_is_picked_up_within_five_minutes(self):
    """Review, 2026-11-06: the first version held a refusal for the rest of the drive. A 401 is also a device JWT from a
    skewed clock, which a time sync fixes mid-drive, and a Tesla's drive lasts as long as the car is awake."""
    self.refuse_metadata(401)
    self.server.status = 206  # the archive host would serve; only nav/tiles refuses
    self.sm.fix = (0., 0.)
    refused_at = self.now
    self.assertEqual(self.drive(60), [])
    self.assertEqual(self.api.get.call_count, 1)
    self.api.get.return_value.raise_for_status.side_effect = None  # the clock synced: nav/tiles answers 200
    lit = self.drive(REFUSED_RETRY_S, black=False)
    self.assertEqual(self.api.get.call_count, 2)
    self.assertTrue(lit)
    self.assertLessEqual(lit[0] - refused_at, REFUSED_RETRY_S)  # the map is back within 5 minutes of the refusal
    self.assertEqual(lit, list(range(int(lit[0]), int(self.now))))  # and every frame after it renders
    self.log.exception.assert_not_called()  # no traceback in between
    self.assertEqual(len(self.refused_logs()), 1)
    over = [c for c in self.log.event.call_args_list if c.args[0] == 'mapsd.tiles_refusal_over']
    self.assertEqual([c.kwargs for c in over], [{'step': 'nav/tiles', 'status': 401}])
    self.assertIsNone(self.mapsd.refused)

  def test_a_drive_start_tries_a_refusal_at_once(self):
    self.refuse_metadata(404)
    self.drive(120, render=False)  # navd plans the route while parked; mapsd fetches its tiles offroad
    self.assertEqual(self.api.get.call_count, 1)
    self.drive(120)  # the drive starts: tried at once, and logged again, in this drive's log; then held REFUSED_RETRY_S
    self.assertEqual(self.api.get.call_count, 2)
    self.assertEqual([c.kwargs['status'] for c in self.refused_logs()], [404, 404])
    # parked again, the account fixed (prime bought): the next drive's first try works, and says the refusal is over
    self.drive(30, render=False)
    self.api.get.return_value.raise_for_status.side_effect = None
    self.server.status = 206
    self.drive(10)
    self.assertEqual(self.api.get.call_count, 3)
    over = [c.kwargs for c in self.log.event.call_args_list if c.args[0] == 'mapsd.tiles_refusal_over']
    self.assertEqual(over, [{'step': 'nav/tiles', 'status': 404}])
    self.assertIsNone(self.mapsd.refused)
    self.log.exception.assert_not_called()

  def test_other_failures_still_retry_every_ten_seconds(self):
    for setup in (lambda: self.refuse_metadata(500), lambda: None):  # comma's server failing; the archive host's 403
      with self.subTest(setup=setup):
        self.api.get.reset_mock()
        self.log.reset_mock()
        self.api.get.return_value.raise_for_status.side_effect = None
        setup()
        self.mapsd.retry_at = 0.
        self.drive(10 * RETRY_SECONDS)
        self.assertEqual(self.log.exception.call_count, 10)
        self.assertTrue(all(c.args == ('mapsd.render_failed',) for c in self.log.exception.call_args_list))
        self.assertEqual(self.refused_logs(), [])
        self.assertIsNone(self.mapsd.refused)

  def test_a_failure_after_a_refusal_ends_the_hold(self):
    self.refuse_metadata(403)
    self.drive(10)
    self.refuse_metadata(500)  # comma's server failing is not a refusal: retried every 10 s, with its traceback
    self.drive(REFUSED_RETRY_S + 10 * RETRY_SECONDS)
    self.assertEqual(self.api.get.call_count, 1 + 1 + 10)  # the refusal, the try 5 minutes on (500), then every 10 s
    self.assertIsNone(self.mapsd.refused)
    self.refuse_metadata(403)  # refused again: a change, logged again
    self.drive(RETRY_SECONDS)
    self.assertEqual(len(self.refused_logs()), 2)


if __name__ == '__main__':
  unittest.main()
