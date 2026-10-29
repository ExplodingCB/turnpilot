import hashlib
import io
import json
import os
import shutil
import socket
import ssl
import subprocess
import sys
import tarfile
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest
import requests
import urllib3

from openpilot.selfdrive.turnpilot.maps.tests.fakes import self_signed
from tools.turnpilot.mapserver import geo, server as mapserver, tiles
from tools.turnpilot.mapserver.server import Maps, Refusal, Region, load_regions, make_handler, make_server

KEY = 'k' * 40
MAPSERVER = Path(mapserver.__file__).parent
SQUARE = {'type': 'Polygon', 'coordinates': [[[-87, 39], [-85, 39], [-85, 41], [-87, 41], [-87, 39]]]}


class FakeRouter:
  def __init__(self, line, fail=False):
    self.line, self.fail = line, fail

  def route(self, request):
    if self.fail:
      raise RuntimeError('No path could be found for input')
    return {'routes': [{'distance': 12345.0, 'geometry': {'type': 'LineString', 'coordinates': self.line}}]}


def make_region(root, rid, geometry, files, router):
  d = root / rid
  d.mkdir()
  index = {}
  for rel, data in files.items():
    p = d / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(data)
    # a sidecar shard (osm_controls/<level-2 tile>.json) is indexed as the level-2 tile it covers, as build_region.py does
    level, idx = (2, int(Path(rel).stem)) if rel.startswith('osm_controls/') else tiles.parse_path(rel.removeprefix('tiles/'))
    index[rel] = [level, idx, hashlib.sha256(data).hexdigest(), len(data)]
  (d / 'fence.geojson').write_text(json.dumps({'type': 'Feature', 'geometry': geometry, 'properties': {}}))
  (d / 'meta.json').write_text(json.dumps({'id': rid, 'name': rid.title(), 'osm_timestamp': 't', 'built_at': 'b'}))
  (d / 'index.json').write_text(json.dumps(index))
  return Region(d, router=router)


def tile_rel(level, lon, lat):
  size = tiles.LEVEL_SIZE[level]
  index = int((lat + 90) // size) * round(360 / size) + int((lon + 180) // size)
  digits = str(index).zfill(3 * -(-len(str(round(360 / size) * round(180 / size) - 1)) // 3))
  return f"tiles/{level}/" + '/'.join(digits[i:i + 3] for i in range(0, len(digits), 3)) + '.gph'


@pytest.fixture
def server(tmp_path):
  near = tile_rel(2, -86.1, 39.8)
  far = tile_rel(2, -80.1, 45.8)
  indy = make_region(tmp_path, 'indianapolis', SQUARE, {near: b'near', far: b'far', tile_rel(0, -86, 40): b'hw'},
                     FakeRouter([[-86.2, 39.7], [-86.0, 39.9]]))
  sea_geom = {'type': 'MultiPolygon', 'coordinates': [[[[-123, 47], [-121, 47], [-121, 48], [-123, 48], [-123, 47]]]]}
  sea = make_region(tmp_path, 'seattle', sea_geom, {}, FakeRouter([], fail=True))
  maps = Maps([indy, sea], KEY)
  httpd = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(maps))
  threading.Thread(target=httpd.serve_forever, daemon=True).start()
  yield f"http://127.0.0.1:{httpd.server_address[1]}", maps, near, far
  httpd.shutdown()


def auth():
  return {'Authorization': f'Bearer {KEY}'}


def test_tile_bbox_matches_real_indiana_tile():
  assert tiles.parse_path('0/002/813.gph') == (0, 2813)
  assert tiles.tile_bbox(0, 2813) == (-88.0, 34.0, -84.0, 38.0)
  assert tiles.parse_path('2/000/756/425.gph') == (2, 756425)
  assert tiles.parse_path('2/000/756/425.txt') is None
  assert tiles.parse_path('../2/000.gph') is None


def test_point_in_polygon_with_hole_and_multipolygon():
  holed = {'type': 'Polygon', 'coordinates': [SQUARE['coordinates'][0], [[-86.5, 39.5], [-85.5, 39.5], [-85.5, 40.5], [-86.5, 40.5]]]}
  assert geo.in_geometry(-86.9, 39.1, holed)
  assert not geo.in_geometry(-86.0, 40.0, holed)
  assert not geo.in_geometry(-84.9, 40.0, holed)
  multi = {'type': 'MultiPolygon', 'coordinates': [SQUARE['coordinates'], [[[0, 0], [1, 0], [1, 1], [0, 1]]]]}
  assert geo.in_geometry(0.5, 0.5, multi)
  assert not geo.in_geometry(2, 2, multi)


def test_health_needs_no_key(server):
  base, *_ = server
  r = requests.get(base + '/v1/health', timeout=5)
  assert r.json() == {'ok': True, 'regions': ['indianapolis', 'seattle']}


def test_everything_else_needs_the_key(server):
  base, *_ = server
  assert requests.get(base + '/v1/coverage', timeout=5).status_code == 401
  bad = {'Authorization': 'Bearer ' + 'x' * 40}
  assert requests.get(base + '/v1/coverage', headers=bad, timeout=5).status_code == 401
  assert requests.post(base + '/v1/corridor', json={}, timeout=5).status_code == 401


def test_coverage(server):
  base, maps, *_ = server
  c = requests.get(base + '/v1/coverage', headers=auth(), timeout=5).json()
  assert c['version'] == maps.version
  assert [f['properties']['id'] for f in c['features']] == ['indianapolis', 'seattle']
  assert c['features'][1]['geometry']['type'] == 'MultiPolygon'


def test_corridor_returns_nearby_tiles_with_checksums(server):
  base, _, near, far = server
  r = requests.post(base + '/v1/corridor', json={'start': [-86.2, 39.7], 'destination': [-86.0, 39.9]},
                    headers=auth(), timeout=5)
  assert r.status_code == 200
  assert r.headers['X-TurnPilot-Region'] == 'indianapolis'
  assert r.headers['Content-Encoding'] == 'gzip'  # requests asks for it and decodes it
  with tarfile.open(fileobj=io.BytesIO(r.content)) as tar:
    names = tar.getnames()
    manifest = json.load(tar.extractfile('manifest.json'))
    assert names[0] == 'manifest.json'
    assert near in names and far not in names
    for rel, sha in manifest['files'].items():
      assert hashlib.sha256(tar.extractfile(rel).read()).hexdigest() == sha
  assert set(manifest['files']) == set(names[1:])
  assert manifest['region'] == 'indianapolis' and manifest['route_km'] == 12.35


def shard_rel(lon, lat):
  return f"osm_controls/{tiles.parse_path(tile_rel(2, lon, lat).removeprefix('tiles/'))[1]}.json"


def test_corridor_carries_the_sidecar_shards_of_its_tiles_when_asked(tmp_path):
  """The OSM stop-sign sidecar's shards go with the corridor's own level-2 tiles, listed with their SHA-256, and the
  manifest carries the region's osm_timestamp the car checks them against. Only a car that asks gets them: an older car's
  corridor.py refuses any file outside tiles/, so shards sent unasked would fail every download it makes."""
  near, far = tile_rel(2, -86.1, 39.8), tile_rel(2, -80.1, 45.8)
  files = {near: b'near', far: b'far', shard_rel(-86.1, 39.8): b'{"near": 1}', shard_rel(-80.1, 45.8): b'{"far": 1}'}
  maps = Maps([make_region(tmp_path, 'indianapolis', SQUARE, files, FakeRouter([[-86.2, 39.7], [-86.0, 39.9]]))], KEY)
  _, manifest, paths = maps.corridor((-86.2, 39.7), (-86.0, 39.9), osm_controls=True)
  assert paths == sorted([near, shard_rel(-86.1, 39.8)])
  assert manifest['files'][shard_rel(-86.1, 39.8)] == hashlib.sha256(b'{"near": 1}').hexdigest()
  assert manifest['osm_timestamp'] == 't'
  _, manifest, paths = maps.corridor((-86.2, 39.7), (-86.0, 39.9))
  assert paths == [near] and manifest['osm_timestamp'] == 't'


def test_corridor_request_asks_for_the_sidecar_over_http(server):
  base, maps, near, _ = server
  shard = shard_rel(-86.1, 39.8)
  region = maps.regions[0]
  (region.path / 'osm_controls').mkdir()
  (region.path / shard).write_bytes(b'{}')
  region.index[shard] = (2, int(Path(shard).stem), hashlib.sha256(b'{}').hexdigest(), 2)
  for ask in (True, False):
    body = {'start': [-86.2, 39.7], 'destination': [-86.0, 39.9]} | ({'osm_controls': True} if ask else {})
    r = requests.post(base + '/v1/corridor', json=body, headers=auth(), timeout=5)
    with tarfile.open(fileobj=io.BytesIO(r.content)) as tar:
      names = tar.getnames()
      assert near in names and (shard in names) == ask and [n for n in names if n.startswith('osm_controls/')] == ([shard] if ask else [])


@pytest.mark.parametrize('start,dest,which', [
  ([-90, 30], [-86, 40], 'start'),
  ([-86, 40], [-90, 30], 'destination'),
  ([-90, 30], [-91, 31], 'both'),
])
def test_corridor_refuses_outside_the_fence(server, start, dest, which):
  base, *_ = server
  r = requests.post(base + '/v1/corridor', json={'start': start, 'destination': dest}, headers=auth(), timeout=5)
  assert r.status_code == 422
  assert r.json() == {'error': 'outside_coverage', 'which': which}


def test_corridor_refuses_between_regions(server):
  base, *_ = server
  r = requests.post(base + '/v1/corridor', json={'start': [-86, 40], 'destination': [-122, 47.5]}, headers=auth(), timeout=5)
  assert r.status_code == 422 and r.json()['error'] == 'different_regions'


def test_corridor_no_route(server):
  base, *_ = server
  r = requests.post(base + '/v1/corridor', json={'start': [-122.5, 47.5], 'destination': [-122, 47.6]}, headers=auth(), timeout=5)
  assert r.status_code == 422 and r.json()['error'] == 'no_route'


@pytest.mark.parametrize('body', [{'start': 'x', 'destination': [0, 0]}, {'start': [0, 99], 'destination': [0, 0]}, [1, 2]])
def test_corridor_bad_requests(server, body):
  base, *_ = server
  r = requests.post(base + '/v1/corridor', json=body, headers=auth(), timeout=5)
  assert r.status_code == 400


def test_region_for_prefers_a_shared_region(server):
  _, maps, *_ = server
  with pytest.raises(Refusal):
    maps.region_for((-86, 40), (-122, 47.5))
  assert maps.region_for((-86, 40), (-86.5, 40.5)).id == 'indianapolis'


CORRIDOR = {'start': [-86.2, 39.7], 'destination': [-86.0, 39.9]}


@pytest.fixture
def tls_server(server, tmp_path):
  """The same maps over TLS as main() serves them, with short timeouts and a small connection cap."""
  _, maps, *_ = server
  cert, key = self_signed()
  (tmp_path / 'cert.pem').write_text(cert)
  (tmp_path / 'cert.key').write_text(key)
  httpd = make_server(maps, '127.0.0.1', 0, tmp_path / 'cert.pem', tmp_path / 'cert.key',
                      handshake_timeout=1., request_timeout=1., max_connections=3, header_timeout=1.5, max_per_address=2)
  threading.Thread(target=httpd.serve_forever, kwargs={'poll_interval': 0.05}, daemon=True).start()
  yield f"https://127.0.0.1:{httpd.server_address[1]}", httpd.server_address, str(tmp_path / 'cert.pem')
  httpd.shutdown()
  httpd.server_close()


def test_an_idle_tcp_connection_does_not_stall_the_server(tls_server):
  base, address, cert = tls_server
  idle = socket.create_connection(address)  # never sends a ClientHello, as a port scanner does
  try:
    t = time.monotonic()
    assert requests.get(base + '/v1/health', verify=cert, timeout=(0.5, 0.5)).status_code == 200
    assert time.monotonic() - t < 0.5
  finally:
    idle.close()


def test_a_silent_client_is_dropped_after_the_handshake(tls_server):
  _, address, cert = tls_server
  context = ssl.create_default_context(cafile=cert)
  with socket.create_connection(address) as raw, context.wrap_socket(raw, server_hostname='127.0.0.1') as tls:
    tls.settimeout(5)
    t = time.monotonic()
    assert tls.recv(1) == b''  # the server hangs up on a request that never comes
    assert time.monotonic() - t < 3


def test_connections_beyond_the_cap_are_turned_away(tls_server):
  """Here the cap per address (2): the total cap is test_one_address_cannot_take_every_connection's."""
  base, address, cert = tls_server
  idle = [socket.create_connection(address) for _ in range(2)]
  try:
    time.sleep(0.2)
    with pytest.raises(requests.ConnectionError):
      requests.get(base + '/v1/health', verify=cert, timeout=(0.5, 0.5))
  finally:
    for s in idle:
      s.close()
  time.sleep(0.3)
  assert requests.get(base + '/v1/health', verify=cert, timeout=2).status_code == 200


def test_one_address_cannot_take_every_connection():
  """A total cap alone turns idle sockets from one host into a lockout of every car: each address gets a few."""
  httpd = make_server(Maps([], KEY), '127.0.0.1', 0, max_connections=5, max_per_address=2)
  try:
    assert [httpd.take('6.6.6.6') for _ in range(3)] == ['', '', 'over 2 connections from it']
    assert httpd.take('10.0.0.1') == '' and httpd.take('10.0.0.2') == ''  # the cars still get in
    httpd.give_back('6.6.6.6')
    assert httpd.take('6.6.6.6') == ''
    assert httpd.take('10.0.0.3') == ''
    assert httpd.take('10.0.0.4') == 'over 5 connections'
    for a in ('6.6.6.6', '6.6.6.6', '10.0.0.1', '10.0.0.2', '10.0.0.3'):
      httpd.give_back(a)
    assert httpd.open == {}
  finally:
    httpd.server_close()


def drip(sock, data, every, until):
  """Send data a byte at a time; True if the server hung up before until seconds."""
  t = time.monotonic()
  for byte in data:
    if time.monotonic() - t > until:
      return False
    try:
      sock.sendall(bytes([byte]))
    except OSError:
      return True
    time.sleep(every)
    sock.setblocking(False)
    try:
      if sock.recv(1) == b'':
        return True
    except (BlockingIOError, ssl.SSLWantReadError):
      pass
    except OSError:
      return True
    finally:
      sock.setblocking(True)
  return False


def test_a_request_sent_a_byte_at_a_time_is_cut_off_by_its_deadline(tls_server):
  """Each byte comes well within the 1 s per-read timeout, but the whole request must be in within 1.5 s."""
  _, address, cert = tls_server
  context = ssl.create_default_context(cafile=cert)
  for request in (b'GET /v1/health HTTP/1.1\r\nHost: x\r\n' + b'X-Pad: ' + b'a' * 200 + b'\r\n\r\n',
                  b'POST /v1/corridor HTTP/1.1\r\nContent-Length: 4000\r\n\r\n' + b' ' * 4000):  # a body as slow
    with socket.create_connection(address) as raw, context.wrap_socket(raw, server_hostname='127.0.0.1') as tls:
      start = time.monotonic()
      if request.startswith(b'POST'):
        head, _, body = request.partition(b'\r\n\r\n')
        tls.sendall(head + b'\r\n\r\n')
        request = body
      assert drip(tls, request, 0.25, 6), "the server kept a dripping request"
      assert time.monotonic() - start < 3


def test_a_client_hello_sent_a_byte_at_a_time_is_cut_off(tls_server):
  _, address, _ = tls_server
  incoming, outgoing = ssl.MemoryBIO(), ssl.MemoryBIO()
  client = ssl.create_default_context().wrap_bio(incoming, outgoing, server_hostname='127.0.0.1')
  with pytest.raises(ssl.SSLWantReadError):
    client.do_handshake()
  hello = outgoing.read()
  with socket.create_connection(address) as raw:
    start = time.monotonic()
    assert drip(raw, hello, 0.25, 6), "the server kept a dripping handshake"
    assert time.monotonic() - start < 2.5


def test_corridor_has_a_content_length(server):
  base, *_ = server
  r = requests.post(base + '/v1/corridor', json=CORRIDOR, headers=auth(), timeout=5, stream=True)
  wire = r.raw.read(decode_content=False)
  assert r.headers['Content-Encoding'] == 'gzip'
  assert int(r.headers['Content-Length']) == len(wire)
  r = requests.post(base + '/v1/corridor', json=CORRIDOR, headers={**auth(), 'Accept-Encoding': 'identity'}, timeout=5)
  assert 'Content-Encoding' not in r.headers and int(r.headers['Content-Length']) == len(r.content)


def test_a_missing_tile_is_a_clean_error_before_any_tar(server):
  base, maps, near, _ = server
  (maps.regions[0].path / near).unlink()
  r = requests.post(base + '/v1/corridor', json=CORRIDOR, headers=auth(), timeout=5)
  assert r.status_code == 503
  assert r.json() == {'error': 'region_unavailable'}


def test_a_tile_changed_under_the_index_is_a_clean_error(server):
  base, maps, near, _ = server
  (maps.regions[0].path / near).write_bytes(b'rebuilt, a different size')
  r = requests.post(base + '/v1/corridor', json=CORRIDOR, headers=auth(), timeout=5)
  assert r.status_code == 503 and r.json() == {'error': 'region_unavailable'}


def test_an_error_while_streaming_cuts_the_body_short_with_no_json_in_it(server, monkeypatch):
  base, *_ = server

  def broken(src, dst):
    dst.write(src.read(10))
    raise OSError('disk went away')

  monkeypatch.setattr(mapserver, 'send_file', broken)
  r = requests.post(base + '/v1/corridor', json=CORRIDOR, headers={**auth(), 'Accept-Encoding': 'identity'}, timeout=5, stream=True)
  assert r.status_code == 200 and int(r.headers['Content-Length']) > 10
  with pytest.raises(urllib3.exceptions.ProtocolError, match=r'IncompleteRead\(10 bytes read'):  # nothing after the 10 bytes
    r.raw.read(decode_content=False)


def test_load_regions_skips_leftovers_from_a_deploy(tmp_path):
  root = tmp_path / 'regions'
  root.mkdir()
  make_region(root, 'indianapolis', SQUARE, {tile_rel(0, -86, 40): b'hw'}, None)
  shutil.copytree(root / 'indianapolis', root / 'indianapolis.old')  # an old swap's leftover: the same id
  half = root / '.indianapolis.new'  # a copy cut off before index.json
  half.mkdir()
  shutil.copy(root / 'indianapolis' / 'meta.json', half / 'meta.json')
  (root / 'notes').mkdir()
  (root / 'README').write_text('not a region')
  assert [r.path.name for r in load_regions(root)] == ['indianapolis']


def test_load_regions_refuses_a_broken_live_region(tmp_path):
  root = tmp_path / 'regions'
  root.mkdir()
  make_region(root, 'indianapolis', SQUARE, {}, None)
  (root / 'indianapolis' / 'index.json').write_text('{"tiles/2/0')  # truncated
  with pytest.raises(SystemExit, match='indianapolis'):
    load_regions(root)
  (root / 'indianapolis').rename(root / 'other')
  with pytest.raises(SystemExit, match='no regions'):
    load_regions(root)


def run_swap(tmp_path, region):
  code = (f"import sys; sys.path.insert(0, '{MAPSERVER}'); from server import load_regions; " +
          f"print([r.id for r in load_regions(__import__('pathlib').Path('{tmp_path / 'regions'}'))])")
  check = f'{sys.executable} -c "{code}"'
  env = {**os.environ, 'SUDO': '', 'RESTART': 'true', 'CHECK': check, 'SETTLE': '0', 'WAIT': '3'}
  return subprocess.run(['bash', str(MAPSERVER / 'swap_region.sh'), str(tmp_path / 'regions'), str(tmp_path / 'staging'), region],
                        env=env, capture_output=True, text=True, timeout=60)


def test_swap_region_replaces_the_live_region_in_one_rename(tmp_path):
  (tmp_path / 'regions').mkdir()
  (tmp_path / 'staging').mkdir()
  make_region(tmp_path / 'regions', 'indianapolis', SQUARE, {tile_rel(0, -86, 40): b'old'}, None)
  make_region(tmp_path / 'staging', 'indianapolis', SQUARE, {tile_rel(0, -86, 40): b'new tile'}, None)
  r = run_swap(tmp_path, 'indianapolis')
  assert r.returncode == 0, r.stderr
  assert "['indianapolis']" in r.stdout
  assert (tmp_path / 'regions' / 'indianapolis' / tile_rel(0, -86, 40)).read_bytes() == b'new tile'
  assert sorted(p.name for p in (tmp_path / 'regions').iterdir()) == ['indianapolis']  # nothing left beside it


def test_swap_region_rolls_back_when_the_new_region_does_not_load(tmp_path):
  (tmp_path / 'regions').mkdir()
  (tmp_path / 'staging').mkdir()
  make_region(tmp_path / 'regions', 'indianapolis', SQUARE, {tile_rel(0, -86, 40): b'old'}, None)
  make_region(tmp_path / 'staging', 'indianapolis', SQUARE, {}, None)
  (tmp_path / 'staging' / 'indianapolis' / 'index.json').write_text('{')
  r = run_swap(tmp_path, 'indianapolis')
  assert r.returncode != 0 and 'rolling back' in r.stderr
  assert (tmp_path / 'regions' / 'indianapolis' / tile_rel(0, -86, 40)).read_bytes() == b'old'
  assert sorted(p.name for p in (tmp_path / 'regions').iterdir()) == ['.indianapolis.failed', 'indianapolis']


def test_a_running_server_serves_the_region_it_loaded_through_a_swap(server):
  """Between swap_region.sh's renames and the restart, the old process must not mix its index with the new files."""
  base, maps, near, _ = server
  live = maps.regions[0].path
  live.rename(live.with_name('.indianapolis.old'))
  shutil.copytree(live.with_name('.indianapolis.old'), live)
  (live / near).write_bytes(b'nope')  # rebuilt: same size, other bytes
  r = requests.post(base + '/v1/corridor', json=CORRIDOR, headers=auth(), timeout=5)
  assert r.status_code == 200
  with tarfile.open(fileobj=io.BytesIO(r.content)) as tar:
    manifest = json.load(tar.extractfile('manifest.json'))
    assert tar.extractfile(near).read() == b'near'
    assert hashlib.sha256(b'near').hexdigest() == manifest['files'][near]


def test_the_server_will_not_start_without_a_disk_tmp_dir(tmp_path):
  """Corridors built in the system /tmp may land on a tmpfs that counts against MemoryMax: no default, and checked."""
  r = subprocess.run([sys.executable, str(MAPSERVER / 'server.py'), '--regions', str(tmp_path)], capture_output=True, text=True,
                     cwd=MAPSERVER, timeout=60)
  assert r.returncode != 0 and '--tmp-dir' in r.stderr
  with pytest.raises(SystemExit, match='not a directory'):
    mapserver.check_tmp_dir(tmp_path / 'missing')
  (tmp_path / 'ro').mkdir()
  os.chmod(tmp_path / 'ro', 0o500)
  try:
    if os.access(tmp_path / 'ro', os.W_OK):
      pytest.skip('running as root')
    with pytest.raises(SystemExit, match="can't be written"):
      mapserver.check_tmp_dir(tmp_path / 'ro')
  finally:
    os.chmod(tmp_path / 'ro', 0o700)
  mapserver.check_tmp_dir(tmp_path)
