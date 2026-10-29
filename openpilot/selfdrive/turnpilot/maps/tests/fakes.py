"""A fake TurnPilot map server over real HTTPS with a self-signed certificate, corridor packs, coverage and params."""
import datetime
import gzip
import hashlib
import io
import ipaddress
import json
import ssl
import tarfile
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID

KEY = 'test-map-key-0123456789'

# (lon, lat) rings around the two areas, roughly as the real server draws them
INDY_RING = [[-87.0, 39.3], [-85.4, 39.3], [-85.4, 40.3], [-87.0, 40.3], [-87.0, 39.3]]
SEATTLE_RING = [[-122.7, 47.2], [-121.8, 47.2], [-121.8, 47.9], [-122.7, 47.9], [-122.7, 47.2]]
INDY = (-86.1581, 39.7684)  # Monument Circle
INDY_EAST = (-86.05, 39.80)
SEATTLE = (-122.335, 47.608)
CHICAGO = (-87.63, 41.88)  # in neither


def level2_tile(lon, lat) -> str:
  """The Valhalla level-2 tile (0.25 degrees) that holds (lon, lat), as a pack names it."""
  index = int((lat + 90) // 0.25) * 1440 + int((lon + 180) // 0.25)
  digits = f"{index:09d}"
  return f"tiles/2/{digits[:3]}/{digits[3:6]}/{digits[6:]}.gph"


def coverage_doc(version='2026-10-27', rings=None):
  rings = rings or {'indianapolis': ('Indianapolis area', INDY_RING), 'seattle': ('Seattle area', SEATTLE_RING)}
  return {'type': 'FeatureCollection', 'version': version, 'features': [
    {'type': 'Feature', 'geometry': {'type': 'Polygon', 'coordinates': [ring]},
     'properties': {'id': rid, 'name': name, 'osm_timestamp': '2026-10-20T00:00:00Z', 'built_at': '2026-10-21T00:00:00Z'}}
    for rid, (name, ring) in rings.items()]}


WORLD = coverage_doc(rings={'world': ('Test world', [[-180, -85], [180, -85], [180, 85], [-180, 85], [-180, -85]])})


def self_signed(ip='127.0.0.1'):
  """(cert PEM, key PEM) for a certificate with ip in its SAN, as the real server has."""
  key = ec.generate_private_key(ec.SECP256R1())
  name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, 'turnpilot-maps-test')])
  now = datetime.datetime.now(datetime.UTC)
  cert = (x509.CertificateBuilder().subject_name(name).issuer_name(name).public_key(key.public_key())
          .serial_number(x509.random_serial_number()).not_valid_before(now - datetime.timedelta(days=1))
          .not_valid_after(now + datetime.timedelta(days=30))
          .add_extension(x509.SubjectAlternativeName([x509.IPAddress(ipaddress.ip_address(ip))]), critical=False)
          .add_extension(x509.BasicConstraints(ca=True, path_length=None), critical=True)
          .sign(key, hashes.SHA256()))
  return (cert.public_bytes(serialization.Encoding.PEM).decode(),
          key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8, serialization.NoEncryption()).decode())


def make_pack(files: dict[str, bytes], region='indianapolis', version='2026-10-27', manifest_files=None, extra=(), manifest=True,
              manifest_extra=None):
  """A corridor tar. manifest_files overrides the listed checksums; extra is a list of TarInfo-building callables;
  manifest_extra more manifest keys (the server's osm_timestamp)."""
  listed = manifest_files if manifest_files is not None else {n: hashlib.sha256(d).hexdigest() for n, d in files.items()}
  buf = io.BytesIO()
  with tarfile.open(fileobj=buf, mode='w') as tar:
    def add(name, data):
      info = tarfile.TarInfo(name)
      info.size = len(data)
      tar.addfile(info, io.BytesIO(data))
    if manifest:
      add('manifest.json', json.dumps({'region': region, 'version': version, 'route_km': 4.2, 'files': listed,
                                       **(manifest_extra or {})}).encode())
    for name, data in files.items():
      add(name, data)
    for build in extra:
      info, data = build()
      tar.addfile(info, io.BytesIO(data) if data is not None else None)
  return buf.getvalue()


def tile_files(tile_dir: Path) -> dict[str, bytes]:
  """{'tiles/2/000/...gph': bytes} for a real tile hierarchy."""
  return {f"tiles/{p.relative_to(tile_dir).as_posix()}": p.read_bytes() for p in sorted(tile_dir.rglob('*')) if p.is_file()}


class FakeParams:
  """A dict with the Params calls the map code makes."""

  def __init__(self, values=None):
    self.values = dict(values or {})
    self.writes = []

  def get(self, key, *args, **kwargs):
    return self.values.get(key)

  def put(self, key, value, block=False):
    self.writes.append((key, value))
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


class MapServer:
  """The map server's API on 127.0.0.1 over TLS. Set .corridor to (status, body bytes, content type) to answer
  /v1/corridor (gzipped on the wire, as the real server sends it); every request is in .requests as (method, path, headers, body).

  The corridor goes out as tools/turnpilot/mapserver/server.py sends it: Content-Length, Connection: close and the region
  header. .cut_at sends only that many bytes of the body and hangs up (a dropped connection); .send_length=False leaves
  Content-Length out, as an older server or a proxy might."""

  def __init__(self, coverage=None, corridor=(200, b'', 'application/x-tar')):
    self.cert, key_pem = self_signed()
    self.coverage = coverage if coverage is not None else coverage_doc()
    self.corridor = corridor
    self.gzip = True  # as the real server does when the client accepts it
    self.cut_at: int | None = None
    self.send_length = True
    self.requests = []
    self.directory = tempfile.TemporaryDirectory()
    cert_file, key_file = Path(self.directory.name) / 'cert.pem', Path(self.directory.name) / 'key.pem'
    cert_file.write_text(self.cert)
    key_file.write_text(key_pem)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(cert_file, key_file)
    server = self

    class Handler(BaseHTTPRequestHandler):
      def log_message(self, format, *args):  # noqa: A002
        pass

      def reply(self, code, body: bytes, ctype='application/json', headers=None, length=True, cut_at=None):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        for k, v in (headers or {}).items():
          self.send_header(k, v)
        if length:
          self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body if cut_at is None else body[:cut_at])

      def handle_any(self, method):
        length = int(self.headers.get('Content-Length') or 0)
        body = self.rfile.read(length) if length else b''
        server.requests.append((method, self.path, dict(self.headers), body))
        if self.path == '/v1/health':
          return self.reply(200, json.dumps({'ok': True, 'regions': ['indianapolis', 'seattle']}).encode())
        if self.headers.get('Authorization') != f'Bearer {KEY}':
          return self.reply(401, b'{"error": "unauthorized"}')
        if method == 'GET' and self.path == '/v1/coverage':
          return self.reply(200, json.dumps(server.coverage).encode())
        if method == 'POST' and self.path == '/v1/corridor':
          code, data, ctype = server.corridor
          if code != 200:
            return self.reply(code, data, ctype)
          self.close_connection = True
          headers = {'X-TurnPilot-Region': 'indianapolis', 'Connection': 'close'}
          if server.gzip and 'gzip' in self.headers.get('Accept-Encoding', ''):
            data, headers['Content-Encoding'] = gzip.compress(data), 'gzip'
          return self.reply(code, data, ctype, headers, length=server.send_length, cut_at=server.cut_at)
        self.reply(404, b'{}')

      def do_GET(self):
        self.handle_any('GET')

      def do_POST(self):
        self.handle_any('POST')

    self.httpd = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)
    self.host = f"https://127.0.0.1:{self.httpd.server_address[1]}"
    threading.Thread(target=self.httpd.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True).start()

  def params(self, **overrides) -> FakeParams:
    return FakeParams({'TurnPilotMapHost': self.host, 'TurnPilotMapKey': KEY, 'TurnPilotMapCert': self.cert, **overrides})

  def corridor_requests(self):
    return [json.loads(body) for method, path, _, body in self.requests if path == '/v1/corridor']

  def close(self):
    self.httpd.shutdown()
    self.httpd.server_close()
    self.directory.cleanup()
