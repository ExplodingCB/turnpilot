#!/usr/bin/env python3
"""turnpilot_web: the phone companion. One page and a small JSON API, standard library only.

Chase's phone joins nothing: the comma joins the phone's hotspot, and this serves one page on port 8088. The page holds
no secrets, so / is open. Every /api/ request needs the token in TurnPilotWebToken, as a Bearer header or as the cookie set
by opening /?t=<token> once.
It is plain HTTP on every interface, so a connection is served only on a loopback or private address, between two such
addresses: the hotspot and a home network are, the LTE modem's address is not (unless the carrier hands out a private
one). The connections handled at once are capped, so a flood can't take a thread each.
"""
import hmac
import ipaddress
import json
import threading
import time
import urllib.parse
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.car_signals import MODES as CAR_SIGNAL_MODES, read_mode
from openpilot.selfdrive.turnpilot.maps import coverage as coverage_cache
from openpilot.selfdrive.turnpilot.maps.client import MapError, cert_fingerprint, check_cert, check_key, missing_config, normalize_host
from openpilot.selfdrive.turnpilot.maps.coverage import CoverageCache, CoverageRefresher, check_trip
from openpilot.selfdrive.turnpilot.maps.status import for_destination, read_status
from openpilot.selfdrive.turnpilot.routing.sources import local_override
from openpilot.selfdrive.turnpilot.web import places
from openpilot.selfdrive.turnpilot.web.pairing import PORT, icon_png, pairing_url, qr_svg
from openpilot.selfdrive.turnpilot.web.status import StatusReader, format_status
from openpilot.selfdrive.turnpilot.web.store import Store, ensure_token, valid_latlon

INDEX = Path(__file__).with_name("index.html")
COOKIE = "tp_token"
MAX_BODY = 16 * 1024
REQUEST_TIMEOUT = 10.  # s a client may stall on one socket read or write; Safari opens sockets it never uses
LOOPBACK = ("127.0.0.1", "::1")
MAX_CONNECTIONS = 32  # sockets handled at once; past this a new one is closed unanswered. A page opens several at once
BIND_RETRY_S = 5.  # s between tries to listen: the manager doesn't restart a dead process, and a dead one soft-disables openpilot


class ApiError(Exception):
  def __init__(self, status: int, message: str, **extra):
    super().__init__(message)
    self.code = status
    self.extra = extra  # more fields for the JSON body, as the geofence's code and regions


def number(value):
  """A coordinate from JSON, which Shortcuts may send as a string."""
  if isinstance(value, bool):
    return None
  try:
    return float(value)
  except (TypeError, ValueError):
    return None


class App:
  """What the handlers share. search and resolve can be swapped for tests."""
  def __init__(self, params, reader=None, search=places.search, resolve=places.resolve_apple_maps, maps_dir=None, refresh=None):
    self.store = Store(params)
    self.reader = reader
    self.search = search
    self.resolve = resolve
    self.maps_dir = maps_dir
    self.coverage = CoverageCache(maps_dir)
    self.refresh = refresh if refresh is not None else (lambda: coverage_cache.refresh(params, maps_dir))

  def valid_token(self, candidates) -> str | None:
    """The stored token if any candidate matches it, else None."""
    expected = self.store.token()
    if expected and any(hmac.compare_digest(c.encode(), expected.encode()) for c in candidates):
      return expected
    return None

  def token_ok(self, candidates) -> bool:
    return self.valid_token(candidates) is not None

  def position(self):
    """(latitude, longitude) to bias searches towards: the live fix, else the last one saved."""
    snap = self.reader.get() if self.reader else None
    return snap["gps"] if snap and snap["gps"] else self.store.last_gps()

  def status(self) -> dict:
    snap = self.reader.get() if self.reader else None
    status = format_status(snap, self.store.destination(), self.store.flag("TurnPilotEnabled"), self.store.flag("IsMetric"))
    status["map"] = self.map_status(status["gps"], status["onroad"])
    return status

  # TurnPilot's mapped area (the geofence) and its map server

  def cached_coverage(self):
    """(coverage or None, error message or None). Never raises: the status must load to show the error."""
    try:
      return self.coverage.get(), None
    except MapError as e:
      return None, e.message

  def map_status(self, gps, onroad: bool = True) -> dict:
    """Where the car is against the mapped area, and navd's download and route status."""
    with self.store.lock:
      params = self.store.params
      override = local_override(params)
      missing = missing_config(params)
      refresh_status = params.get(coverage_cache.STATUS_PARAM)
      nav = for_destination(read_status(params), params.get("NavDestination"), onroad)  # only about the current destination
    coverage, error = self.cached_coverage()
    if coverage is None and error is None:
      error = coverage_cache.NO_COVERAGE_MESSAGE
    here = gps
    if here is None and (last := self.store.last_gps()):
      here = {"latitude": last[0], "longitude": last[1]}
    region = coverage.region_at(here["longitude"], here["latitude"]) if coverage and here else None
    return {
      "override": override,  # TurnPilotValhallaTiles: local tiles, no geofence, no download
      "missing": missing,
      "coverage": coverage and {"version": coverage.version, "fetched_at": self.coverage.fetched_at, "regions": coverage.summary(),
                                "names": coverage.names()},
      "coverage_error": error,
      "refresh": refresh_status if isinstance(refresh_status, dict) else None,
      "car": here,
      "car_region": region.name if region else None,
      "car_outside": bool(coverage and here and region is None),
      "outside_message": coverage.outside_message() if coverage else None,
      "nav": nav,
    }

  def check_destination(self, latitude, longitude):
    """Refuse a destination outside the mapped area, and any destination while there is no coverage to check it against.
    With the local tiles override (TurnPilotValhallaTiles, as in the sim) there is no geofence."""
    with self.store.lock:
      if local_override(self.store.params):
        return
    try:
      coverage = self.coverage.require()
      here = self.position()  # (latitude, longitude)
      check_trip(coverage, (here[1], here[0]) if here else None, (longitude, latitude))
    except MapError as e:
      coverage = self.coverage.coverage
      raise ApiError(422, e.message, code=e.code, regions=coverage.summary() if coverage else []) from None

  def map_settings(self) -> dict:
    """The map server settings for the page. The key is never sent back, only its last 4 characters."""
    with self.store.lock:
      params = self.store.params
      host, key, cert = (params.get(k) or "" for k in ("TurnPilotMapHost", "TurnPilotMapKey", "TurnPilotMapCert"))
    try:
      fingerprint = cert_fingerprint(cert) if cert else ""
    except ValueError:
      fingerprint = "not a valid certificate"
    return {"host": host, "key_set": bool(key), "key_hint": key[-4:] if len(key) >= 12 else "", "cert_set": bool(cert),
            "cert_fingerprint": fingerprint}

  def set_map_settings(self, body) -> dict:
    """Save what the body sets (host, key, cert); a missing or empty field stays as it is. Then fetch the coverage with
    the new settings, so the page shows at once whether they work.

    A new host or certificate needs the key in the same request. The web token travels in the clear on the hotspot, and
    otherwise whoever holds it could point the car at their own server and certificate, and the saved key would go there."""
    values = {}
    try:
      if str(body.get("host") or "").strip():
        values["TurnPilotMapHost"] = normalize_host(str(body["host"]))
      if str(body.get("key") or "").strip():
        values["TurnPilotMapKey"] = check_key(str(body["key"]))
      if str(body.get("cert") or "").strip():
        values["TurnPilotMapCert"] = check_cert(str(body["cert"]))
        try:
          cert_fingerprint(values["TurnPilotMapCert"])
        except ValueError:  # base64's binascii.Error; only the certificate gets this message
          raise ApiError(400, "The certificate is not valid PEM.") from None
    except MapError as e:
      raise ApiError(400, e.message) from None
    if not values:
      raise ApiError(400, "nothing to save: send host, key or cert")
    with self.store.lock:
      params = self.store.params
      saved_host, saved_cert = (params.get("TurnPilotMapHost") or ""), (params.get("TurnPilotMapCert") or "")
      new_host = values.get("TurnPilotMapHost", saved_host) != saved_host
      new_cert = values.get("TurnPilotMapCert", saved_cert).strip() != saved_cert.strip()
      if (new_host or new_cert) and "TurnPilotMapKey" not in values and params.get("TurnPilotMapKey"):
        what = "address" if new_host and not new_cert else "certificate" if new_cert and not new_host else "address and certificate"
        raise ApiError(400, f"Enter the API key again to change the map server {what}. " +
                       "The saved key is only sent to a new server when you type it.")
      for k, v in values.items():
        params.put(k, v, block=True)
    try:
      self.refresh()
      result = {"coverage_ok": True, "coverage_error": None}
    except MapError as e:
      result = {"coverage_ok": False, "coverage_error": e.message}
    return {**self.map_settings(), **result}

  def car_signals(self) -> dict:
    """TurnPilot's turn signal settings for the page (car_signals.py, DESIGN phase4/blinker 7.8): the mode, the last
    lamp check (step, result, why, its id and whether the outside lamps were confirmed), and the verified record's VIN."""
    with self.store.lock:
      params = self.store.params
      mode = read_mode(params)
      check = params.get("TurnPilotCarSignalsCheck")
      record = params.get("TurnPilotCarSignalsVerified")
    last = None
    if check:
      last = {k: check.get(k) for k in ("pass", "step", "why", "checkId", "exterior", "when")}
      last["gear"] = (check.get("record") or {}).get("gear")
    return {"mode": mode, "check": last, "verified": bool(record), "verified_vin": (record or {}).get("vin")}

  def set_car_signals(self, body) -> dict:
    """One of: {"mode": off|check|on}; {"startInDrive": true} (TurnPilotCarSignalsCheckNow, honoured only on a check
    ignition with the car stopped in D and the brake held); {"exterior": yes|no, "checkId": ...}, the answer to "did the
    lamps flash outside?", only for the pending check's id. The page never writes a record: card's resolver does."""
    with self.store.lock:
      params = self.store.params
      if "mode" in body:
        mode = body.get("mode")
        if mode not in CAR_SIGNAL_MODES:
          raise ApiError(400, "mode must be off, check or on")
        params.put("TurnPilotCarSignals", mode, block=True)
      elif body.get("startInDrive") is True:
        params.put_bool("TurnPilotCarSignalsCheckNow", True, block=True)
      elif "exterior" in body:
        verdict = body.get("exterior")
        if verdict not in ("yes", "no"):
          raise ApiError(400, "exterior must be yes or no")
        check = params.get("TurnPilotCarSignalsCheck") or {}
        if not check.get("pass") or check.get("exterior") != "pending" or body.get("checkId") != check.get("checkId"):
          raise ApiError(409, "No pending check with that id: run the check again")
        params.put("TurnPilotCarSignalsExterior", f"{verdict}:{check['checkId']}", block=True)
      else:
        raise ApiError(400, "send mode, startInDrive or exterior")
    return self.car_signals()

  def coverage_body(self) -> dict:
    coverage, error = self.cached_coverage()
    if coverage is None:
      return {"coverage": None, "error": error or coverage_cache.NO_COVERAGE_MESSAGE}
    return {"coverage": coverage.data, "version": coverage.version, "fetched_at": self.coverage.fetched_at,
            "regions": coverage.summary(), "names": coverage.names(), "error": None}


class Handler(BaseHTTPRequestHandler):
  app: App  # set on a subclass by make_handler
  server_version = "turnpilot-web"
  timeout = REQUEST_TIMEOUT  # StreamRequestHandler puts this on the socket, so idle and half-open connections let go

  def log_message(self, format, *args):  # noqa: A002
    pass  # the first request line carries the pairing token

  def do_GET(self):
    self.dispatch("GET")

  def do_POST(self):
    self.dispatch("POST")

  def do_DELETE(self):
    self.dispatch("DELETE")

  def send(self, code, body: bytes, ctype: str, extra=()):
    self.send_response(code)
    self.send_header("Content-Type", ctype)
    self.send_header("Content-Length", str(len(body)))
    self.send_header("Cache-Control", "no-store")
    self.send_header("Referrer-Policy", "no-referrer")
    self.send_header("X-Content-Type-Options", "nosniff")
    for k, v in extra:
      self.send_header(k, v)
    self.end_headers()
    self.wfile.write(body)

  def send_json(self, code, obj):
    self.send(code, json.dumps(obj).encode(), "application/json")

  def body(self) -> dict:
    size = (self.headers.get("Content-Length") or "0").strip()
    if not (size.isascii() and size.isdigit()):  # also catches -1, which read() would treat as "until the client hangs up"
      raise ApiError(400, "bad Content-Length")
    length = int(size)
    if length > MAX_BODY:
      raise ApiError(413, "body too large")
    try:
      raw = self.rfile.read(length)
    except TimeoutError:
      self.close_connection = True
      raise ApiError(408, "request timed out") from None
    try:
      data = json.loads(raw or b"{}")
    except ValueError:
      raise ApiError(400, "body is not JSON") from None
    if not isinstance(data, dict):
      raise ApiError(400, "body must be a JSON object")
    return data

  def credentials(self) -> list[str]:
    """What the request offers as the token. The ?t= in a URL isn't one: it only pairs on the page, so it stays out of API request lines."""
    found = []
    auth = self.headers.get("Authorization", "")
    if auth.startswith("Bearer "):
      found.append(auth[7:].strip())
    if morsel := SimpleCookie(self.headers.get("Cookie", "")).get(COOKIE):
      found.append(morsel.value)
    return found

  def dispatch(self, method):
    url = urllib.parse.urlsplit(self.path)
    path, query = url.path, urllib.parse.parse_qs(url.query)
    try:
      if method == "GET" and path == "/icon.png":  # iOS fetches this without our cookie; it holds nothing private
        return self.send(200, icon_png(), "image/png", [("Cache-Control", "max-age=86400")])
      if method == "GET" and path == "/":  # no secrets in the page, so a launch that lost its ?t= can still load and use the stored token
        return self.page(query)
      authorized = self.app.token_ok(self.credentials())
      if path == "/pair.svg":  # shows the token, so only the token holder or the device itself may see it
        authorized = authorized or self.client_address[0] in LOOPBACK
      if not authorized:
        raise ApiError(401, "unauthorized")
      if method == "POST" and not self.headers.get("Content-Type", "").lower().startswith("application/json"):
        raise ApiError(415, "send application/json")  # a cross-site form can't send this without a preflight
      if method == "GET" and path == "/pair.svg":
        with self.app.store.lock:
          url = pairing_url(self.app.store.params)
        return self.send(200, qr_svg(url).encode(), "image/svg+xml")
      self.send_json(200, self.api(method, path, query))
    except ApiError as e:
      self.send_json(e.code, {"error": str(e), **e.extra})
    except ValueError as e:
      self.send_json(400, {"error": str(e)})
    except Exception:
      cloudlog.exception("turnpilot_web.request_failed")
      self.send_json(500, {"error": "internal error"})

  def page(self, query):
    extra = []
    if token := self.app.valid_token(query.get("t", [])):  # opened with the token: remember it for the next visit
      extra = [("Set-Cookie", f"{COOKIE}={token}; Path=/; Max-Age=31536000; HttpOnly; SameSite=Strict")]  # the stored token, never the query's
    self.send(200, INDEX.read_bytes(), "text/html; charset=utf-8", extra)

  def api(self, method, path, query):
    app, store = self.app, self.app.store
    if method == "GET" and path == "/api/status":
      return app.status()
    if method == "GET" and path == "/api/search":
      return {"results": self.search(query)}
    if method == "GET" and path == "/api/favorites":
      return store.places()
    if method == "GET" and path == "/api/pairing":
      host = self.headers.get("Host") or f"localhost:{PORT}"
      return {"url": f"http://{host}/?t={store.token()}", "endpoint": f"http://{host}/api/destination", "token": store.token()}
    if method == "POST" and path == "/api/destination":
      return {"ok": True, **self.navigate(self.body())}
    if (method, path) in (("DELETE", "/api/destination"), ("POST", "/api/stop")):
      store.stop()
      return {"ok": True}
    if method == "GET" and path == "/api/coverage":
      return app.coverage_body()
    if method == "GET" and path == "/api/map":
      return app.map_settings()
    if method == "POST" and path == "/api/map":
      return {"ok": True, **app.set_map_settings(self.body())}
    if method == "GET" and path == "/api/car-signals":
      return app.car_signals()
    if method == "POST" and path == "/api/car-signals":
      return {"ok": True, **app.set_car_signals(self.body())}
    if method == "POST" and path == "/api/favorites":
      body = self.body()
      return {"ok": True, **store.add_favorite(number(body.get("latitude")), number(body.get("longitude")), body.get("name"))}
    if method == "DELETE" and path.startswith("/api/favorites/"):
      if not store.remove_favorite(path.rsplit("/", 1)[1]):
        raise ApiError(404, "no such favorite")
      return {"ok": True}
    raise ApiError(404, "not found")

  def search(self, query) -> list[dict]:
    text = (query.get("q") or [""])[0].strip()
    if len(text) < 2:
      return []
    try:
      return self.app.search(text, self.app.position())
    except Exception as e:
      cloudlog.warning(f"turnpilot_web.search_failed {e}")
      raise ApiError(502, "place search is not reachable") from None

  def navigate(self, body) -> dict:
    """Set the destination from coordinates, or from a shared Apple Maps link."""
    place = body
    if "url" in body:
      place = self.app.resolve(str(body["url"]))
      if place is None:
        raise ApiError(400, "that is not a maps link")
      if "latitude" not in place:  # the link only names or addresses a place
        found = self.search({"q": [place["query"]]})
        if not found:
          raise ApiError(404, f"no place found for {place['query']!r}")
        place = {**found[0], "name": place["name"] or found[0]["name"]}
    lat, lon = number(place.get("latitude")), number(place.get("longitude"))
    if lat is None or lon is None or not valid_latlon(lat, lon):
      raise ApiError(400, "need latitude and longitude, or a url")
    self.app.check_destination(lat, lon)  # before anything is saved, so a refused place never becomes the destination or a recent
    return self.app.store.navigate(lat, lon, place.get("name"))


def make_handler(app: App):
  return type("BoundHandler", (Handler,), {"app": app})


def private_address(ip: str) -> bool:
  """Loopback, link-local or a private network's, not one the internet can reach (nor carrier-grade NAT's 100.64/10)."""
  try:
    return ipaddress.ip_address(ip).is_private
  except ValueError:
    return False


class Server(ThreadingHTTPServer):
  request_queue_size = 64  # the default of 5 resets connections when a page opens several at once

  def __init__(self, *args, **kwargs):
    self.lock = threading.Lock()
    self.connections: set = set()  # the sockets being handled
    super().__init__(*args, **kwargs)

  def verify_request(self, request, client_address) -> bool:
    """Runs before a handler thread exists; a refused socket is closed unanswered."""
    if not (private_address(request.getsockname()[0]) and private_address(client_address[0])):
      return False
    with self.lock:
      if len(self.connections) >= MAX_CONNECTIONS:
        return False
      self.connections.add(request)
    return True

  def shutdown_request(self, request):
    with self.lock:
      self.connections.discard(request)
    super().shutdown_request(request)


def main():
  params = Params()
  ensure_token(params)
  reader = StatusReader()
  reader.start()
  refresher = CoverageRefresher(params)  # keeps TurnPilot's mapped area fresh while the comma is online
  refresher.start()
  while True:
    try:
      server = Server(("", PORT), make_handler(App(params, reader)))
      break
    except OSError as e:  # the port still held by another instance
      cloudlog.warning(f"turnpilot_web can't listen on :{PORT} ({e}), retrying")
      time.sleep(BIND_RETRY_S)
  cloudlog.info(f"turnpilot_web listening on :{PORT}")
  server.serve_forever()


if __name__ == "__main__":
  main()
