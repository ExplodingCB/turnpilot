import errno
import gzip
import json
import socket
from unittest.mock import patch

import pytest
import requests

from openpilot.selfdrive.turnpilot.maps import client as map_client
from openpilot.selfdrive.turnpilot.maps.client import MapClient, MapError, cert_fingerprint, check_key, normalize_host
from openpilot.selfdrive.turnpilot.maps.coverage import STATUS_PARAM, CoverageCache, refresh
from openpilot.selfdrive.turnpilot.maps.tests.fakes import INDY, INDY_EAST, KEY, FakeParams, MapServer, coverage_doc, make_pack, self_signed


@pytest.fixture
def server():
  s = MapServer()
  yield s
  s.close()


def client_for(server, tmp_path, **overrides):
  return MapClient.from_params(server.params(**overrides), tmp_path)


def test_health_and_coverage_over_pinned_tls(server, tmp_path):
  c = client_for(server, tmp_path)
  assert c.health() == {'ok': True, 'regions': ['indianapolis', 'seattle']}
  assert c.coverage()['version'] == coverage_doc()['version']
  assert (tmp_path / 'server-cert.pem').read_text().strip() == server.cert.strip()
  health, cov = server.requests
  assert 'Authorization' not in health[2]  # health needs no key
  assert cov[2]['Authorization'] == f'Bearer {KEY}'


def test_requests_always_verify_against_the_saved_certificate(server, tmp_path):
  c = client_for(server, tmp_path)
  real = requests.request
  seen = []

  def spy(*args, **kwargs):
    seen.append(kwargs['verify'])
    return real(*args, **kwargs)

  with patch.object(map_client.requests, 'request', side_effect=spy):
    c.health()
    c.coverage()
  assert seen == [str(tmp_path / 'server-cert.pem')] * 2  # a path, never False


def test_bad_key(server, tmp_path):
  with pytest.raises(MapError) as e:
    client_for(server, tmp_path, TurnPilotMapKey='wrong').coverage()
  assert e.value.code == 'unauthorized' and 'API key' in e.value.message


def test_wrong_certificate_is_a_tls_error(server, tmp_path):
  other, _ = self_signed()
  with pytest.raises(MapError) as e:
    client_for(server, tmp_path, TurnPilotMapCert=other).health()
  assert e.value.code == 'tls'
  assert server.requests == []  # the handshake failed before any request


def test_certificate_for_another_address_is_a_tls_error(server, tmp_path):
  with pytest.raises(MapError) as e:
    client_for(server, tmp_path, TurnPilotMapHost=server.host.replace('127.0.0.1', 'localhost')).health()
  assert e.value.code == 'tls'  # the SAN is the IP, so the name doesn't match


def test_unreachable(tmp_path):
  cert, _ = self_signed()
  with socket.socket() as s:
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]  # closed again before the request
  c = MapClient(f'https://127.0.0.1:{port}', KEY, cert, tmp_path)
  with pytest.raises(MapError) as e:
    c.coverage()
  assert e.value.code == 'unreachable' and e.value.retry


def test_settings_are_checked(tmp_path):
  with pytest.raises(MapError) as e:
    MapClient.from_params(FakeParams({'TurnPilotMapHost': 'https://1.2.3.4'}), tmp_path)
  assert e.value.code == 'not_configured' and 'API key' in e.value.message and 'certificate' in e.value.message
  assert normalize_host('136.113.85.41') == 'https://136.113.85.41'
  assert normalize_host('https://136.113.85.41:8443/') == 'https://136.113.85.41:8443'
  with pytest.raises(MapError):
    normalize_host('http://136.113.85.41')  # the key would go in the clear
  with pytest.raises(MapError):
    MapClient('https://1.2.3.4', KEY, 'not a certificate', tmp_path)
  cert, _ = self_signed()
  assert len(cert_fingerprint(cert).split(':')) == 32


@pytest.mark.parametrize('compressed', [True, False])
def test_corridor_download(server, tmp_path, compressed):
  pack = make_pack({f'tiles/2/{i}.gph': b'tile ' * 2000 for i in range(20)})
  server.corridor, server.gzip = (200, pack, 'application/x-tar'), compressed
  seen = []
  size = client_for(server, tmp_path).corridor(INDY, INDY_EAST, tmp_path / 'out.tar', lambda done, total: seen.append((done, total)))
  assert size == len(pack) and (tmp_path / 'out.tar').read_bytes() == pack  # the tar itself, decoded
  wire = len(gzip.compress(pack)) if compressed else len(pack)
  assert seen[-1] == (wire, wire)  # progress counts bytes off the wire against Content-Length
  assert (wire < len(pack)) == compressed
  # the OSM stop-sign sidecar is asked for: an older car, which can't unpack it, doesn't get it
  assert server.corridor_requests() == [{'start': list(INDY), 'destination': list(INDY_EAST), 'osm_controls': True}]


@pytest.mark.parametrize('body,code,words', [
  ({'error': 'outside_coverage', 'which': 'start'}, 'outside_coverage', 'The car is outside'),
  ({'error': 'outside_coverage', 'which': 'destination'}, 'outside_coverage', 'The destination is outside'),
  ({'error': 'outside_coverage', 'which': 'both'}, 'outside_coverage', 'The car and the destination are outside'),
  ({'error': 'different_regions'}, 'different_regions', 'different mapped areas'),
  ({'error': 'no_route'}, 'no_route', 'no route'),
  ({'error': 'something new'}, 'bad_response', 'refused the route'),
])
def test_corridor_422(server, tmp_path, body, code, words):
  server.corridor = (422, json.dumps(body).encode(), 'application/json')
  with pytest.raises(MapError) as e:
    client_for(server, tmp_path).corridor(INDY, INDY_EAST, tmp_path / 'out.tar', coverage_names='Indianapolis and Seattle')
  assert e.value.code == code and words in e.value.message and not e.value.retry
  if code == 'outside_coverage':
    assert 'Indianapolis and Seattle' in e.value.message


def test_refresh_caches_and_records_status(server, tmp_path):
  params = server.params()
  refresh(params, tmp_path)
  assert CoverageCache(tmp_path).get().names() == 'Indianapolis and Seattle'
  assert params.values[STATUS_PARAM]['ok'] and params.values[STATUS_PARAM]['version'] == '2026-10-27'
  params.values['TurnPilotMapKey'] = 'wrong'
  with pytest.raises(MapError):
    refresh(params, tmp_path)
  assert params.values[STATUS_PARAM]['ok'] is False and params.values[STATUS_PARAM]['code'] == 'unauthorized'
  assert CoverageCache(tmp_path).get() is not None  # a failed refresh keeps the last good cache
  server.coverage = {'type': 'FeatureCollection', 'version': 'x', 'features': []}
  params.values['TurnPilotMapKey'] = KEY
  with pytest.raises(MapError) as e:
    refresh(params, tmp_path)
  assert e.value.code == 'bad_response'
  assert CoverageCache(tmp_path).get().version == '2026-10-27'


def test_real_map_server_corridor_through_the_client_and_unpack(tmp_path):
  """tools/turnpilot/mapserver/server.py's real handler over TLS, as main() serves it, into MapClient and corridor.unpack:
  the headers the car relies on (Content-Length for progress and truncation) come from the real server, not the fake."""
  import threading
  from openpilot.selfdrive.turnpilot.maps import corridor
  from tools.turnpilot.mapserver.server import Maps, make_server
  from tools.turnpilot.mapserver.tests.test_server import SQUARE, FakeRouter, make_region, tile_rel

  from tools.turnpilot.mapserver.tests.test_server import shard_rel
  near, shard = tile_rel(2, -86.1, 39.8), shard_rel(-86.1, 39.8)
  root = tmp_path / 'regions'
  root.mkdir()
  region = make_region(root, 'indianapolis', SQUARE, {near: b'near tile ' * 5000, tile_rel(0, -86, 40): b'hw', shard: b'{"osm": 1}'},
                       FakeRouter([[-86.2, 39.7], [-86.0, 39.9]]))
  cert, key = self_signed()
  (tmp_path / 'cert.pem').write_text(cert)
  (tmp_path / 'cert.key').write_text(key)
  httpd = make_server(Maps([region], KEY), '127.0.0.1', 0, tmp_path / 'cert.pem', tmp_path / 'cert.key')
  threading.Thread(target=httpd.serve_forever, kwargs={'poll_interval': 0.05}, daemon=True).start()
  try:
    c = MapClient(f"https://127.0.0.1:{httpd.server_address[1]}", KEY, cert, tmp_path)
    seen = []
    c.corridor((-86.2, 39.7), (-86.0, 39.9), tmp_path / 'out.tar', lambda done, total: seen.append((done, total)))
    assert seen[-1][1] is not None and seen[-1][0] == seen[-1][1]  # the progress has a total, and reached it
    pack = corridor.unpack(tmp_path / 'out.tar', tmp_path / 'packs')
    assert (pack.tiles / near.removeprefix('tiles/')).read_bytes() == b'near tile ' * 5000
    assert (pack.path / shard).read_bytes() == b'{"osm": 1}' and pack.manifest['osm_timestamp'] == 't'
  finally:
    httpd.shutdown()
    httpd.server_close()


@pytest.mark.parametrize('send_length', [True, False])
def test_a_gzip_download_cut_off_is_caught(server, tmp_path, send_length):
  """The server hangs up partway (OOM kill, restart): with or without Content-Length, the download is not complete."""
  import os
  server.corridor = (200, make_pack({f'tiles/2/{i}.gph': os.urandom(5000) for i in range(20)}), 'application/x-tar')
  server.send_length, server.cut_at = send_length, 30000
  with pytest.raises(MapError) as e:
    client_for(server, tmp_path).corridor(INDY, INDY_EAST, tmp_path / 'out.tar')
  assert (e.value.code, e.value.short, e.value.retry) == ('unreachable', 'Map download broke off', True)


def test_a_plain_download_cut_off_is_caught_by_its_length(server, tmp_path):
  server.corridor, server.gzip, server.cut_at = (200, make_pack({'tiles/2/0.gph': b'x' * 50000}), 'application/x-tar'), False, 20000
  with pytest.raises(MapError) as e:
    client_for(server, tmp_path).corridor(INDY, INDY_EAST, tmp_path / 'out.tar')
  assert e.value.code == 'unreachable' and e.value.retry


def test_a_download_that_cannot_be_saved_is_a_map_error(server, tmp_path):
  server.corridor = (200, make_pack({'tiles/2/0.gph': b'x'}), 'application/x-tar')
  (tmp_path / 'out.tar').mkdir()  # as a full or read-only /data would fail the write
  with pytest.raises(MapError) as e:
    client_for(server, tmp_path).corridor(INDY, INDY_EAST, tmp_path / 'out.tar')
  assert e.value.code == 'storage' and not e.value.retry  # the comma's own disk: not downloaded again every 30 s


def test_a_full_disk_says_so_and_is_not_a_retry(server, tmp_path):
  server.corridor = (200, make_pack({'tiles/2/0.gph': b'x'}), 'application/x-tar')
  real_open = open

  def full(path, *args, **kwargs):
    if str(path).endswith('out.tar'):
      raise OSError(errno.ENOSPC, 'No space left on device')
    return real_open(path, *args, **kwargs)

  with patch('builtins.open', full), pytest.raises(MapError) as e:
    client_for(server, tmp_path).corridor(INDY, INDY_EAST, tmp_path / 'out.tar')
  assert (e.value.code, e.value.retry) == ('storage', False)
  assert 'full' in e.value.short and 'storage is full' in e.value.message



@pytest.mark.parametrize('host', ['https://1.2.3.4:abc', '1.2.3.4:99999', '1.2.3.4 8443', 'https://1.2.3.4/maps', 'https://u@1.2.3.4',
                                  'https://1.2.3.4?x=1', 'https://1.2.3.4#x', 'https://', 'https://[zz]', 'https://[::1',
                                  'https://h\\x', 'https://h;x', 'https://host:', 'host:', 'https://[1.2.3.4]', 'https://-h.com',
                                  'https://h_x.com', 'https://h..com'])
def test_bad_hosts_are_map_errors(host):
  with pytest.raises(MapError) as e:
    normalize_host(host)
  assert e.value.code == 'not_configured'


def test_good_hosts_are_kept():
  assert normalize_host('https://[::1]:8443') == 'https://[::1]:8443'
  assert normalize_host('maps.example.com') == 'https://maps.example.com'
  assert normalize_host('https://maps-1.example.com:443/') == 'https://maps-1.example.com:443'


def test_a_key_that_is_not_printable_ascii_is_refused():
  for key in ('clé-ключ-12345678', 'has space 123456', 'tab\there12345'):
    with pytest.raises(MapError) as e:
      check_key(key)
    assert e.value.code == 'not_configured'
  assert check_key('  abc-123_XYZ.def  ') == 'abc-123_XYZ.def'


def test_any_request_failure_is_a_map_error(server, tmp_path):
  c = client_for(server, tmp_path)
  for error in (requests.exceptions.InvalidURL('Failed to parse'), UnicodeEncodeError('latin-1', 'é', 0, 1, 'x'), requests.exceptions.TooManyRedirects()):
    with patch.object(map_client.requests, 'request', side_effect=error), pytest.raises(MapError) as e:
      c.coverage()
    assert e.value.code == 'bad_request' and type(error).__name__ in e.value.message


def test_refresh_records_any_failure(server, tmp_path):
  params = server.params()
  with patch.object(map_client.MapClient, 'coverage', side_effect=RuntimeError('surprise')), pytest.raises(MapError) as e:
    refresh(params, tmp_path)
  assert e.value.code == 'refresh_failed'
  assert params.values[STATUS_PARAM]['ok'] is False and 'RuntimeError: surprise' in params.values[STATUS_PARAM]['error']



def test_the_corridor_waits_longer_for_the_server_to_build_it(server, tmp_path):
  """The server builds the whole corridor before its status line: the default 30 s read timeout would cut off a big one."""
  server.corridor = (200, make_pack({'tiles/2/0.gph': b'x'}), 'application/x-tar')
  real, seen = requests.request, []

  def spy(*args, **kwargs):
    seen.append((args[1].rsplit('/', 1)[1], kwargs['timeout']))
    return real(*args, **kwargs)

  c = client_for(server, tmp_path)
  with patch.object(map_client.requests, 'request', side_effect=spy):
    c.coverage()
    c.corridor(INDY, INDY_EAST, tmp_path / 'out.tar')
  assert seen == [('coverage', map_client.TIMEOUT), ('corridor', map_client.CORRIDOR_TIMEOUT)]
  assert map_client.CORRIDOR_TIMEOUT[1] >= 120
