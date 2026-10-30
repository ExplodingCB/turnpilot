"""The TurnPilot map server's API, over HTTPS with its self-signed certificate pinned from TurnPilotMapCert.

Every failure is a MapError with a code and a message meant for the driver; nothing here retries or falls back.
"""
import base64
import errno
import ipaddress
import hashlib
import json
import os
import re
import tempfile
import time
import zlib
from pathlib import Path
from urllib.parse import urlsplit

import requests

from openpilot.selfdrive.turnpilot.maps.paths import maps_dir

TIMEOUT = (5., 30.)  # s to connect, s between bytes
# the server builds the whole corridor (up to a few hundred MB, gzipped) before it sends the status line, so the first
# byte can take a minute or more on its small VM; after that, bytes come steadily
CORRIDOR_TIMEOUT = (5., 180.)
CHUNK = 256 * 1024
CERT_FILE = 'server-cert.pem'
PEM_BEGIN = '-----BEGIN CERTIFICATE-----'


class MapError(Exception):
  """A reason TurnPilot can't route. code is stable for tests and the UI, message is for the driver.
  short fits the onroad status line. retry says whether trying again soon could help (a network blip)."""

  def __init__(self, code: str, message: str, short: str = '', retry: bool = False):
    super().__init__(message)
    self.code, self.message, self.short, self.retry = code, message, short or message, retry


def bad_host(host: str, why: str) -> MapError:
  return MapError('not_configured', f"The map server address {host!r} {why}. Fix it in the TurnPilot web app.", 'Map server address is wrong')


def storage_error(e: OSError) -> MapError:
  """The comma couldn't save the map: its own storage, which a new download won't fix, so it isn't a retry."""
  if e.errno in (errno.ENOSPC, errno.EDQUOT):
    return MapError('storage', "The comma's storage is full, so TurnPilot can't save the map for this drive. Free up space " +
                    "on the comma (old drives), then set the destination again.", "Comma storage full: can't save the map")
  return MapError('storage', f"TurnPilot can't save the map on the comma ({type(e).__name__}: {e}).", "Can't save the map")


HOSTNAME = re.compile(r'^(?=.{1,253}$)[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?(?:\.[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?)*\.?$')


def normalize_host(host: str) -> str:
  """https://host[:port] from the TurnPilotMapHost param. Plain http is refused: the key would travel in the clear."""
  host = (host or '').strip().rstrip('/')
  if not host:
    return ''
  if any(c.isspace() for c in host) or not host.isascii() or not host.isprintable():
    raise bad_host(host, "has spaces or characters an address can't have")
  if '://' not in host:
    if re.match(r'^[A-Za-z][A-Za-z0-9+.-]*:(?!\d)', host):  # 'https:' or 'http:/x': a scheme, not host:port
      raise bad_host(host, "must be just https://host or https://host:port")
    host = 'https://' + host
  try:
    parts = urlsplit(host)
  except ValueError:  # a bracketed host that isn't an IPv6 address, as https://[zz] or https://[::1
    raise bad_host(host, "is not a valid address") from None
  if parts.scheme != 'https':
    raise bad_host(host, "must start with https://")
  if not parts.hostname or parts.username is not None or parts.path or parts.query or parts.fragment or '#' in host or '?' in host:
    raise bad_host(host, "must be just https://host or https://host:port")
  try:
    parts.port  # noqa: B018  # raises for a port that isn't a number from 0 to 65535
  except ValueError:
    raise bad_host(host, "has a port that isn't a number from 1 to 65535") from None
  if parts.port == 0 or parts.netloc.endswith(':'):
    raise bad_host(host, "has a port that isn't a number from 1 to 65535")
  name = parts.hostname
  if parts.netloc.startswith('['):
    try:
      ipaddress.IPv6Address(name)
    except ValueError:
      raise bad_host(host, "is not a valid address") from None
  elif not HOSTNAME.match(name):
    raise bad_host(host, "is not a valid host name: letters, digits, dots and dashes only")
  return f"https://{parts.netloc}"


def check_key(key: str) -> str:
  """The API key, which goes in an HTTP header: printable ASCII with no spaces."""
  key = (key or '').strip()
  if not key.isascii() or not key.isprintable() or any(c.isspace() for c in key):
    raise MapError('not_configured', "The map server API key can only have letters, digits and punctuation, with no spaces. " +
                   "Paste it again in the TurnPilot web app.", 'Map server key is wrong')
  return key


def check_cert(pem: str) -> str:
  pem = (pem or '').strip()
  if not pem.startswith(PEM_BEGIN) or '-----END CERTIFICATE-----' not in pem:
    raise MapError('not_configured', "The map server certificate is not a PEM certificate. Paste it again in the TurnPilot web app.",
                   'Map server certificate is wrong')
  return pem + '\n'


def cert_fingerprint(pem: str) -> str:
  """SHA-256 of the certificate's DER bytes, as openssl x509 -fingerprint -sha256 prints it."""
  body = ''.join(line for line in pem.strip().splitlines() if line and not line.startswith('-----'))
  digest = hashlib.sha256(base64.b64decode(body)).hexdigest().upper()
  return ':'.join(digest[i:i + 2] for i in range(0, len(digest), 2))


def missing_config(params) -> list[str]:
  return [name for key, name in (('TurnPilotMapHost', 'address'), ('TurnPilotMapKey', 'API key'), ('TurnPilotMapCert', 'certificate'))
          if not (params.get(key) or '').strip()]


def write_atomic(path: Path, data: bytes):
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
  try:
    with os.fdopen(fd, 'wb') as f:
      f.write(data)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  except BaseException:
    Path(tmp).unlink(missing_ok=True)
    raise


class MapClient:
  def __init__(self, host: str, key: str, cert_pem: str, directory: Path | None = None):
    self.host = normalize_host(host)
    self.key = check_key(key)
    pem = check_cert(cert_pem)
    self.cert_path = (directory or maps_dir()) / CERT_FILE
    if not self.cert_path.is_file() or self.cert_path.read_text() != pem:
      write_atomic(self.cert_path, pem.encode())

  @classmethod
  def from_params(cls, params, directory: Path | None = None) -> 'MapClient':
    missing = missing_config(params)
    if missing:
      raise MapError('not_configured', f"The TurnPilot map server is not set up: no {', '.join(missing)}. Enter it in the TurnPilot web app.",
                     'Map server not set up')
    return cls(params.get('TurnPilotMapHost'), params.get('TurnPilotMapKey'), params.get('TurnPilotMapCert'), directory)

  def request(self, method, path, auth=True, timeout=TIMEOUT, **kwargs) -> requests.Response:
    headers = {'Authorization': f"Bearer {self.key}"} if auth else {}
    try:
      resp = requests.request(method, self.host + path, headers=headers, verify=str(self.cert_path), timeout=timeout, **kwargs)
    except requests.exceptions.SSLError as e:
      raise MapError('tls', f"The map server at {self.host} failed the TLS check against the saved certificate ({type(e).__name__}). " +
                     "Paste the server's current certificate in the TurnPilot web app.", 'Map server TLS check failed') from None
    except (requests.exceptions.ConnectionError, requests.exceptions.Timeout) as e:
      raise MapError('unreachable', f"Can't reach the TurnPilot map server at {self.host} ({type(e).__name__}). " +
                     "Check the phone hotspot or the server.", 'Map server unreachable', retry=True) from None
    except (requests.exceptions.RequestException, ValueError) as e:  # an address or key requests can't send; UnicodeError is a ValueError
      raise MapError('bad_request', f"TurnPilot can't send a request to the map server at {self.host} ({type(e).__name__}: {e}). " +
                     "Check the map server settings in the TurnPilot web app.", 'Map server settings are wrong') from None
    if resp.status_code == 401:
      resp.close()
      raise MapError('unauthorized', "The map server refused the API key. Enter the right key in the TurnPilot web app.",
                     'Map server refused the key')
    return resp

  def unexpected(self, resp, what) -> MapError:
    resp.close()
    return MapError('bad_response', f"The map server answered {what} with HTTP {resp.status_code}.", f"Map server error {resp.status_code}",
                    retry=resp.status_code >= 500)

  def health(self) -> dict:
    resp = self.request('GET', '/v1/health', auth=False)
    if resp.status_code != 200:
      raise self.unexpected(resp, 'the health check')
    return resp.json()

  def coverage(self) -> dict:
    resp = self.request('GET', '/v1/coverage')
    if resp.status_code != 200:
      raise self.unexpected(resp, 'the coverage request')
    try:
      return resp.json()
    except ValueError:
      raise MapError('bad_response', "The map server's coverage is not JSON.", 'Map server sent bad coverage') from None

  def corridor(self, start, destination, out: Path, progress=None, coverage_names: str = '') -> int:
    """Download the corridor tar for (lon, lat) start and destination to out, and return its size.

    The server gzips the tar (Content-Encoding: gzip) when asked, as requests does by default. It is decoded here
    rather than by urllib3, which accepts a gzip stream that stops short of its end. So a download that broke off is
    caught three ways: fewer bytes than Content-Length, a gzip stream that never reached its end, and (in unpack) a tar
    that ends early. progress(received, total or None) counts bytes off the wire against Content-Length."""
    # osm_controls: the OSM stop-sign sidecar's shards for the corridor's tiles (corridor.py, routing/osm_sidecar.py)
    resp = self.request('POST', '/v1/corridor', json={'start': list(start), 'destination': list(destination), 'osm_controls': True},
                        stream=True, timeout=CORRIDOR_TIMEOUT)
    with resp:
      if resp.status_code == 422:
        raise refusal(resp, coverage_names)
      if resp.status_code != 200:
        raise self.unexpected(resp, 'the corridor request')
      total = resp.headers.get('Content-Length')
      total = int(total) if total and total.isascii() and total.isdigit() else None
      encoding = resp.headers.get('Content-Encoding', '').strip().lower()
      if encoding not in ('', 'identity', 'gzip'):
        raise MapError('bad_response', f"The map server sent the map with an encoding TurnPilot can't read ({encoding}).",
                       'Map server sent a bad map')
      gunzip = zlib.decompressobj(wbits=31) if encoding == 'gzip' else None
      received = size = 0

      def broke_off(detail: str) -> MapError:
        return MapError('unreachable', f"The map download from {self.host} broke off ({detail}).", 'Map download broke off', retry=True)

      try:
        with open(out, 'wb') as f:
          chunks = resp.raw.stream(CHUNK, decode_content=False)
          while True:
            try:
              chunk = next(chunks, None)
            except Exception as e:  # urllib3's ProtocolError, ReadTimeoutError, SSLError: the stream broke
              raise broke_off(type(e).__name__) from None
            if chunk is None:
              break
            received += len(chunk)
            if gunzip is not None:
              try:
                chunk = gunzip.decompress(chunk)
              except zlib.error as e:
                raise MapError('bad_response', f"The map download from {self.host} is not valid gzip ({e}).", 'Map server sent a bad map',
                               retry=True) from None
            f.write(chunk)
            size += len(chunk)
            if progress is not None:
              progress(received, total)
          if gunzip is not None:
            if not gunzip.eof:
              raise broke_off(f"the gzip stream ended early, after {received} bytes")
            f.write(gunzip.flush())
      except OSError as e:  # writing the download on the comma: a full disk is still full in 30 s
        raise storage_error(e) from None
      if total is not None and received != total:
        raise broke_off(f"{received} of {total} bytes")
    return size


def refusal(resp, coverage_names: str) -> MapError:
  """A MapError for the map server's 422."""
  try:
    body = resp.json()
  except ValueError:
    body = {}
  error = body.get('error') if isinstance(body, dict) else None
  around = f" TurnPilot can only navigate around {coverage_names}." if coverage_names else ''
  if error == 'outside_coverage':
    which = body.get('which')
    what = {'start': 'The car is', 'destination': 'The destination is', 'both': 'The car and the destination are'}.get(which, 'The route is')
    return MapError('outside_coverage', f"{what} outside TurnPilot's mapped area.{around}", 'Outside mapped area')
  if error == 'different_regions':
    return MapError('different_regions', f"The car and the destination are in different mapped areas. TurnPilot can't route between them.{around}",
                    'Destination in another mapped area')
  if error == 'no_route':
    return MapError('no_route', "The map server found no route to the destination.", 'No route found')
  return MapError('bad_response', f"The map server refused the route: {json.dumps(body)[:200]}", 'Map server refused the route')


class Throttle:
  """Calls fn at most every period seconds, and always for the final call."""

  def __init__(self, fn, period=0.5):
    self.fn, self.period, self.last = fn, period, 0.

  def __call__(self, *args, final=False):
    now = time.monotonic()
    if final or now - self.last >= self.period:
      self.last = now
      self.fn(*args)
