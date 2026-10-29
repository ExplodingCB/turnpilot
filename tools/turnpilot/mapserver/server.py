#!/usr/bin/env python3
"""TurnPilot's map server: the drivable areas, and the Valhalla tiles a drive needs.

  GET  /v1/health     {"ok": true, "regions": [...]}, no key needed
  GET  /v1/coverage   the drivable areas as a GeoJSON FeatureCollection, with a "version"
  POST /v1/corridor   {"start": [lon, lat], "destination": [lon, lat], "osm_controls": true}
                      -> a tar of manifest.json, tiles/ and, when asked, osm_controls/

Every /v1 call but health needs "Authorization: Bearer <key>". A corridor is every tile within BUFFER_KM of
the route, so the car can reroute locally without a connection. With "osm_controls": true it also carries the OSM
stop-sign sidecar's shards for its level-2 tiles (osm_controls.py), which the index lists beside the tiles; a car from
before the sidecar doesn't ask, and gets the tiles alone, which its pack check accepts. The manifest carries the region's
osm_timestamp, the stamp the car checks the shards against. Start and destination must both be inside
one region's fence, or the server refuses with a 422 that says why. The corridor is built in a temp file before
anything is sent, so it goes out with a Content-Length the car checks, and any failure is a clean 5xx.

The TLS handshake runs in each connection's own thread, and it and the request (line, headers and body) each have a
deadline for the whole of it, not per read, so a client that drips a byte at a time is dropped all the same. Connections
are capped in total and per client address, and corridors in flight are capped, so idle or slow clients can't stall the
server, lock the cars out, or exhaust its 1 GB.

Each region is a directory under --regions, written by build_region.py:
  fence.geojson  the drivable area, a Feature with a Polygon or MultiPolygon
  meta.json      {"id", "name", "osm_timestamp", "built_at"}
  index.json     {"tiles/2/000/756/425.gph": [level, index, sha256, size], ...,
                  "osm_controls/756425.json": [2, level-2 index, sha256, size], ...}
  tiles/         the tile hierarchy
  osm_controls/  the sidecar's shards, one per level-2 tile
  tiles.tar      the same tiles as one extract, which Valhalla routes from
"""
import argparse
import gzip
import hashlib
import hmac
import io
import json
import logging
import os
import shutil
import socket
import ssl
import tarfile
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
  from tools.turnpilot.mapserver import geo, tiles
except ImportError:  # run as a plain script on the server
  import geo  # type: ignore[no-redef]
  import tiles  # type: ignore[no-redef]

BUFFER_KM = 15.0  # with 28 km level-2 tiles this keeps every road within 15-40 km of the route
GZIP_LEVEL = 3
MAX_BODY = 4096
CACHE_BYTES = 150 * 1024 * 1024  # Valhalla's tile cache; the server has 1 GB
HANDSHAKE_TIMEOUT = 10.  # s for a client to finish the whole TLS handshake, in its own thread
HEADER_TIMEOUT = 20.  # s for the whole request (line, headers and body) once the handshake is done
REQUEST_TIMEOUT = 60.  # s a client may stall on one socket write while a response goes out (cellular stalls)
MAX_CONNECTIONS = 32  # open connections; more are closed at once, so idle sockets can't exhaust threads or memory
MAX_PER_ADDRESS = 4  # open connections from one address: a car needs one or two, and one host can't take all 32
MAX_CORRIDORS = 2  # corridors built or sent at once; each is a temp file of up to a few hundred MB
SHARDS = 'osm_controls'  # the OSM stop-sign sidecar's shards (osm_controls.py)
SEND_CHUNK = 256 * 1024

log = logging.getLogger("turnpilot-maps")


class Refusal(Exception):
  def __init__(self, code: int, body: dict):
    super().__init__(body.get('error', ''))
    self.code, self.body = code, body


class Region:
  """A region as loaded. Tiles are read through a descriptor of the directory opened at load, so a deploy that renames
  a new copy into place can't mix new files with the index this process loaded; it restarts the server instead."""

  def __init__(self, path: Path, router=None):
    self.path = path
    self.dir_fd = os.open(path, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
    self.fence = json.loads(self.read('fence.geojson'))
    self.meta = json.loads(self.read('meta.json'))
    self.id = self.meta['id']
    self.index = {rel: tuple(v) for rel, v in json.loads(self.read('index.json')).items()}
    self.router = router
    self.lock = threading.Lock()

  def open(self, rel: str):
    return os.fdopen(os.open(rel, os.O_RDONLY, dir_fd=self.dir_fd), 'rb')

  def read(self, rel: str) -> bytes:
    with self.open(rel) as f:
      return f.read()

  def load_router(self):
    """Valhalla on tiles.tar. main() does this at start, while the directory still holds the files this index lists."""
    if self.router is None:
      import valhalla
      config = valhalla.get_config(tile_extract=str(self.path / 'tiles.tar'), tile_dir=str(self.path / 'tiles'))
      config['mjolnir']['max_cache_size'] = CACHE_BYTES
      self.router = valhalla.Actor(config)

  def contains(self, lon, lat) -> bool:
    return geo.in_geometry(lon, lat, self.fence['geometry'])

  def feature(self) -> dict:
    return {'type': 'Feature', 'geometry': self.fence['geometry'],
            'properties': {k: self.meta.get(k) for k in ('id', 'name', 'osm_timestamp', 'built_at')}}

  def route_line(self, start, destination) -> tuple[list[tuple[float, float]], float]:
    """The route's line and its length in km."""
    self.load_router()
    request = {
      'locations': [{'lon': start[0], 'lat': start[1]}, {'lon': destination[0], 'lat': destination[1]}],
      'costing': 'auto', 'format': 'osrm', 'shape_format': 'geojson',
    }
    with self.lock:
      try:
        r = self.router.route(request)
      except RuntimeError as e:  # Valhalla raises for "no path" and "no suitable edges near location"
        raise Refusal(422, {'error': 'no_route', 'detail': str(e)[:200]}) from e
    if isinstance(r, str):
      r = json.loads(r)
    routes = r.get('routes') or []
    if not routes:
      raise Refusal(422, {'error': 'no_route'})
    line = [tuple(c) for c in routes[0]['geometry']['coordinates']]
    km = routes[0].get('distance', 0) / 1000
    return line, km


class Maps:
  def __init__(self, regions: list[Region], key: str, tmp_dir: Path | None = None, max_corridors: int = MAX_CORRIDORS):
    self.regions = regions
    self.key = key
    self.tmp_dir = tmp_dir  # main() requires it; None (tests) is the system's
    self.corridor_slots = threading.BoundedSemaphore(max_corridors)
    blob = json.dumps([[r.meta, r.fence] for r in regions], sort_keys=True).encode()
    self.version = hashlib.sha256(blob).hexdigest()[:16]

  def coverage(self) -> dict:
    return {'type': 'FeatureCollection', 'version': self.version, 'features': [r.feature() for r in self.regions]}

  def region_for(self, start, destination) -> Region:
    start_in = [r for r in self.regions if r.contains(*start)]
    dest_in = [r for r in self.regions if r.contains(*destination)]
    if not start_in and not dest_in:
      raise Refusal(422, {'error': 'outside_coverage', 'which': 'both'})
    if not start_in:
      raise Refusal(422, {'error': 'outside_coverage', 'which': 'start'})
    if not dest_in:
      raise Refusal(422, {'error': 'outside_coverage', 'which': 'destination'})
    shared = [r for r in start_in if r in dest_in]
    if not shared:
      raise Refusal(422, {'error': 'different_regions'})
    return shared[0]

  def corridor(self, start, destination, osm_controls: bool = False) -> tuple[Region, dict, list[str]]:
    """osm_controls: the sidecar's shards too, for the same level-2 tiles."""
    region = self.region_for(start, destination)
    line, km = region.route_line(start, destination)
    grid = {rel: (v[0], v[1]) for rel, v in region.index.items() if osm_controls or not rel.startswith(SHARDS + '/')}
    paths = tiles.needed(grid, line, BUFFER_KM)
    manifest = {'region': region.id, 'version': self.version, 'route_km': round(km, 2), 'buffer_km': BUFFER_KM,
                'osm_timestamp': region.meta.get('osm_timestamp'), 'files': {rel: region.index[rel][2] for rel in paths}}
    return region, manifest, paths


def load_regions(root: Path) -> list[Region]:
  """Every region directory under root. Only a directory named for its meta.json id is a region: deploy leftovers
  ('.indianapolis.new' half copied, '.indianapolis.old', 'indianapolis.old') are skipped. A region that is there but
  doesn't load stops the server with its name, rather than serving the others without it."""
  regions = []
  for path in sorted(root.iterdir()):
    if path.name.startswith('.') or not path.is_dir():
      continue
    try:
      meta = json.loads((path / 'meta.json').read_text())
    except FileNotFoundError:
      log.warning("skipping %s: no meta.json", path)
      continue
    except ValueError as e:
      raise SystemExit(f"region {path} does not load: meta.json: {e}") from None
    if not isinstance(meta, dict) or meta.get('id') != path.name:
      log.warning("skipping %s: it is not named for its region id (%r)", path, meta.get('id') if isinstance(meta, dict) else meta)
      continue
    try:
      regions.append(Region(path))
    except (OSError, ValueError, KeyError, TypeError) as e:
      raise SystemExit(f"region {path} does not load: {type(e).__name__}: {e}") from None
  if not regions:
    raise SystemExit(f"no regions under {root}")
  return regions


def point(value) -> tuple[float, float]:
  """(lon, lat) from a JSON [lon, lat], or a 400."""
  try:
    lon, lat = (float(v) for v in value)
  except (TypeError, ValueError) as e:
    raise Refusal(400, {'error': 'bad_request', 'detail': 'points are [lon, lat]'}) from e
  if not (-180 <= lon <= 180 and -90 <= lat <= 90):
    raise Refusal(400, {'error': 'bad_request', 'detail': 'point out of range'})
  return lon, lat


def write_corridor(out, region: Region, manifest: dict, paths: list[str], gz: bool):
  """The corridor tar (gzipped if gz) into the file out. Every tile must still be the size the index says: the files
  are read by path, so one replaced under a running server (a deploy) would not match the manifest's checksums."""
  for rel in paths:
    try:
      size = os.stat(rel, dir_fd=region.dir_fd).st_size
    except OSError as e:
      raise Refusal(503, {'error': 'region_unavailable'}) from e
    if size != region.index[rel][3]:
      log.error("%s is %d bytes, the index says %d: the region changed under the server", rel, size, region.index[rel][3])
      raise Refusal(503, {'error': 'region_unavailable'})
  dest = gzip.GzipFile(fileobj=out, mode='wb', compresslevel=GZIP_LEVEL) if gz else out
  try:
    with tarfile.open(fileobj=dest, mode='w|') as tar:
      data = json.dumps(manifest).encode()
      info = tarfile.TarInfo('manifest.json')
      info.size = len(data)
      tar.addfile(info, io.BytesIO(data))
      for rel in paths:
        with region.open(rel) as tile:
          info = tarfile.TarInfo(rel)
          info.size = os.fstat(tile.fileno()).st_size
          info.mtime = int(os.fstat(tile.fileno()).st_mtime)
          tar.addfile(info, tile)
  except OSError as e:
    raise Refusal(503, {'error': 'region_unavailable'}) from e
  finally:
    if gz:
      dest.close()


def send_file(src, dst):
  shutil.copyfileobj(src, dst, SEND_CHUNK)


class DeadlineIO(socket.SocketIO):
  """A socket reader whose every recv waits only as long as is left of a deadline, so the deadline bounds a whole
  readline (http.server reads the request line and headers with readline), however slowly the bytes come."""

  def __init__(self, sock, remaining):
    super().__init__(sock, 'rb')
    self.remaining = remaining

  def readinto(self, b):
    left = self.remaining()
    if left is not None:
      if left <= 0:
        raise TimeoutError('the request took too long')
      self._sock.settimeout(left)
    return super().readinto(b)


def make_handler(maps: Maps, request_timeout: float = REQUEST_TIMEOUT, header_timeout: float = HEADER_TIMEOUT):
  class Handler(BaseHTTPRequestHandler):
    server_version = 'turnpilot-maps'
    sys_version = ''
    timeout = request_timeout  # StreamRequestHandler puts it on the socket: a stalled client lets go of its thread
    headers_sent = False
    deadline: float | None = None

    def setup(self):
      super().setup()
      # the request line, headers and body must all arrive by the deadline: a timeout per read alone lets a client that
      # sends a byte every timeout - 1 s hold its connection forever
      self.deadline = time.monotonic() + header_timeout
      self.rfile.close()
      raw = DeadlineIO(self.connection, self.remaining)
      self.connection._io_refs += 1  # as makefile() counts it, so closing the socket waits for this reader
      self.rfile = io.BufferedReader(raw)

    def handle(self):
      try:
        super().handle()
      except OSError as e:  # the client hung up or broke TLS mid-request: its own problem, not a server error
        log.info("%s: connection lost (%s: %s)", self.address_string(), type(e).__name__, e)
        self.close_connection = True

    def remaining(self) -> float | None:
      return None if self.deadline is None else self.deadline - time.monotonic()

    def request_read(self):
      """The request is in: from here on only the per-write timeout applies, as the response goes out."""
      self.deadline = None
      self.connection.settimeout(request_timeout)

    def log_message(self, format, *args):  # noqa: A002
      log.info("%s %s", self.address_string(), format % args)

    def send_json(self, code, obj):
      self.request_read()  # a refusal before the body was read writes with the per-write timeout, not the deadline's rest
      body = json.dumps(obj).encode()
      self.headers_sent = True
      self.send_response(code)
      self.send_header('Content-Type', 'application/json')
      self.send_header('Content-Length', str(len(body)))
      self.end_headers()
      self.wfile.write(body)

    def authorized(self) -> bool:
      header = self.headers.get('Authorization', '')
      given = header[7:] if header.startswith('Bearer ') else ''
      return bool(maps.key) and hmac.compare_digest(given.encode(), maps.key.encode())

    def handle_request(self, method):
      try:
        if method != 'POST':
          self.request_read()
        if method == 'GET' and self.path == '/v1/health':
          return self.send_json(200, {'ok': True, 'regions': [r.id for r in maps.regions]})
        if not self.path.startswith('/v1/'):
          raise Refusal(404, {'error': 'not_found'})
        if not self.authorized():
          raise Refusal(401, {'error': 'unauthorized'})
        if method == 'GET' and self.path == '/v1/coverage':
          return self.send_json(200, maps.coverage())
        if method == 'POST' and self.path == '/v1/corridor':
          return self.corridor()
        raise Refusal(404, {'error': 'not_found'})
      except TimeoutError as e:
        log.info("%s: %s (%s); closing the connection", self.address_string(), 'timed out' if self.headers_sent else 'request too slow', e)
        self.close_connection = True
      except Exception as e:
        if self.headers_sent:  # a status line is out: an error now must not write a second response into the body
          log.warning("request failed after its headers were sent (%s: %s); closing the connection", type(e).__name__, e)
          self.close_connection = True
          return
        if isinstance(e, Refusal):
          return self.send_json(e.code, e.body)
        log.exception("request failed")
        self.send_json(500, {'error': 'server_error'})

    def corridor(self):
      length = int(self.headers.get('Content-Length') or 0)
      if not 0 < length <= MAX_BODY:
        raise Refusal(400, {'error': 'bad_request', 'detail': 'body missing or too large'})
      data = self.rfile.read(length)
      self.request_read()
      try:
        body = json.loads(data)
      except ValueError as e:
        raise Refusal(400, {'error': 'bad_request', 'detail': 'body is not JSON'}) from e
      if not isinstance(body, dict):
        raise Refusal(400, {'error': 'bad_request'})
      region, manifest, paths = maps.corridor(point(body.get('start')), point(body.get('destination')), body.get('osm_controls') is True)
      size = sum(region.index[rel][3] for rel in paths)
      shards = sum(1 for rel in paths if rel.startswith(SHARDS + '/'))
      log.info("corridor %s: %.1f km, %d tiles, %d sidecar shards, %.1f MB", region.id, manifest['route_km'], len(paths) - shards, shards,
               size / 1e6)

      # built in a temp file first, so a failure is a proper error response, and the car gets a Content-Length that
      # shows it whether the download broke off
      gz = 'gzip' in self.headers.get('Accept-Encoding', '')
      if not maps.corridor_slots.acquire(blocking=False):
        raise Refusal(503, {'error': 'busy'})
      try:
        with tempfile.TemporaryFile(dir=maps.tmp_dir, prefix='corridor-') as f:
          write_corridor(f, region, manifest, paths, gz)
          length = f.tell()
          f.seek(0)
          self.headers_sent = True
          self.send_response(200)
          self.send_header('Content-Type', 'application/x-tar')
          if gz:  # tiles compress about 2.8 to 1, and the car downloads them over cellular
            self.send_header('Content-Encoding', 'gzip')
          self.send_header('Content-Length', str(length))
          self.send_header('X-TurnPilot-Region', region.id)
          self.send_header('Connection', 'close')
          self.end_headers()
          self.close_connection = True
          send_file(f, self.wfile)
      finally:
        maps.corridor_slots.release()

    def do_GET(self):
      self.handle_request('GET')

    def do_POST(self):
      self.handle_request('POST')

  return Handler


class MapHTTPServer(ThreadingHTTPServer):
  """ThreadingHTTPServer with the TLS handshake in each connection's thread, with a deadline, and caps on connections,
  in total and per client address.

  Wrapping the listening socket instead would run the handshake inside accept() on the one serving thread: a single
  client that connects and sends nothing would stall every other request. The cap per address keeps one host from
  taking every connection, which would turn the total cap into a lockout of every car."""
  daemon_threads = True
  request_queue_size = 64

  def __init__(self, address, handler, context: ssl.SSLContext | None = None, handshake_timeout: float = HANDSHAKE_TIMEOUT,
               max_connections: int = MAX_CONNECTIONS, max_per_address: int = MAX_PER_ADDRESS):
    super().__init__(address, handler)
    self.context, self.handshake_timeout = context, handshake_timeout
    self.max_connections, self.max_per_address = max_connections, max_per_address
    self.lock = threading.Lock()
    self.open: dict[str, int] = {}  # open connections by client address

  def take(self, address: str) -> str:
    """'' and a slot taken for the connection, or why it is turned away."""
    with self.lock:
      if sum(self.open.values()) >= self.max_connections:
        return f"over {self.max_connections} connections"
      if self.open.get(address, 0) >= self.max_per_address:
        return f"over {self.max_per_address} connections from it"
      self.open[address] = self.open.get(address, 0) + 1
      return ''

  def give_back(self, address: str):
    with self.lock:
      self.open[address] -= 1
      if not self.open[address]:
        del self.open[address]

  def process_request(self, request, client_address):
    if why := self.take(client_address[0]):
      log.warning("%s: %s, closing it", client_address[0], why)
      self.shutdown_request(request)
      return
    try:
      super().process_request(request, client_address)
    except BaseException:
      self.give_back(client_address[0])
      raise

  def process_request_thread(self, request, client_address):
    try:
      super().process_request_thread(request, client_address)
    finally:
      self.give_back(client_address[0])

  def finish_request(self, request, client_address):
    if self.context is None:
      return super().finish_request(request, client_address)
    # the handshake runs with this as one deadline for all of it (CPython's do_handshake counts down the socket timeout
    # across its reads), so a ClientHello sent a byte at a time is cut off all the same
    request.settimeout(self.handshake_timeout)
    try:
      tls = self.context.wrap_socket(request, server_side=True)
    except (OSError, ValueError) as e:  # ssl.SSLError and TimeoutError are OSErrors
      log.info("%s: TLS handshake failed: %s", client_address[0], e)
      return
    try:
      super().finish_request(tls, client_address)
    finally:
      self.shutdown_request(tls)


def make_server(maps: Maps, host: str, port: int, cert=None, cert_key=None, handshake_timeout: float = HANDSHAKE_TIMEOUT,
                request_timeout: float = REQUEST_TIMEOUT, max_connections: int = MAX_CONNECTIONS, header_timeout: float = HEADER_TIMEOUT,
                max_per_address: int = MAX_PER_ADDRESS) -> MapHTTPServer:
  """The server over TLS with cert and cert_key, or plain HTTP without them (tests)."""
  context = None
  if cert is not None:
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(cert, cert_key)
  return MapHTTPServer((host, port), make_handler(maps, request_timeout, header_timeout), context, handshake_timeout, max_connections,
                       max_per_address)


def check_tmp_dir(path: Path):
  """Refuse to start without a directory corridors can be built in, rather than fail the first one."""
  if not path.is_dir():
    raise SystemExit(f"--tmp-dir {path} is not a directory")
  try:
    with tempfile.TemporaryFile(dir=path, prefix='corridor-check-'):
      pass
  except OSError as e:
    raise SystemExit(f"--tmp-dir {path}: corridors can't be written there: {e}") from None


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('--regions', type=Path, default=Path('/srv/turnpilot-maps/regions'))
  ap.add_argument('--key-file', type=Path, default=Path('/etc/turnpilot-maps/key'))
  ap.add_argument('--cert', type=Path, default=Path('/etc/turnpilot-maps/cert.pem'))
  ap.add_argument('--cert-key', type=Path, default=Path('/etc/turnpilot-maps/cert.key'))
  ap.add_argument('--host', default='0.0.0.0')
  ap.add_argument('--port', type=int, default=443)
  ap.add_argument('--tmp-dir', type=Path, required=True,
                  help="where corridors are built before they are sent, on disk (setup_server.sh: /var/cache/turnpilot-maps). " +
                       "Not /tmp, which may be a tmpfs that counts against the unit's MemoryMax")
  args = ap.parse_args()
  logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
  check_tmp_dir(args.tmp_dir)

  key = args.key_file.read_text().strip()
  if len(key) < 32:
    raise SystemExit(f"{args.key_file} must hold a key of at least 32 characters")
  regions = load_regions(args.regions)
  for region in regions:
    region.load_router()
  maps = Maps(regions, key, args.tmp_dir)
  log.info("serving %s, coverage version %s", [r.id for r in regions], maps.version)

  httpd = make_server(maps, args.host, args.port, args.cert, args.cert_key)
  httpd.serve_forever()


if __name__ == '__main__':
  main()
