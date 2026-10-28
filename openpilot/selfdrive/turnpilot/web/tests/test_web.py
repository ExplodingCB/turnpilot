import http.client
import json
import re
import socket
import struct
import tempfile
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

import numpy as np

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.qrcode import decode_matrix
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.turnpilot.maps.coverage import coverage_path, save
from openpilot.selfdrive.turnpilot.maps.status import STATUS_PARAM
from openpilot.selfdrive.turnpilot.maps.tests.fakes import CHICAGO, INDY, KEY, SEATTLE, WORLD, MapServer, coverage_doc
from openpilot.selfdrive.turnpilot.web import pairing, places, server, status
from openpilot.selfdrive.turnpilot.web.store import Store, ensure_token, last_gps_position

INDIANAPOLIS = (39.7684, -86.1581)


def photon_feature(name, lat, lon, **props):
  return {"geometry": {"type": "Point", "coordinates": [lon, lat]}, "properties": {"name": name, **props}}


class FakeSearch:
  """Stands in for places.search so no test touches the network."""
  def __init__(self, results=None, error=None):
    self.results, self.error, self.calls = results or [], error, []

  def __call__(self, query, near=None):
    self.calls.append((query, near))
    if self.error:
      raise self.error
    return self.results


class WebTestCase(OpenpilotTestCase):
  """A real server on a free port, backed by the per-test Params that OpenpilotTestCase isolates."""
  HANDLER_TIMEOUT = None  # a test that waits on timeouts sets a short one
  COVERAGE = WORLD

  def setup_method(self):
    self.params = Params()
    self.search = FakeSearch([{"name": "Cafe", "detail": "1 Main St", "latitude": 39.1, "longitude": -86.2}])
    maps = tempfile.TemporaryDirectory()
    self.addCleanup(maps.cleanup)
    self.maps = Path(maps.name)
    save(self.COVERAGE, self.maps)  # most of these tests are about the page, not the geofence: there everywhere is mapped
    self.app = server.App(self.params, reader=None, search=self.search, resolve=places.resolve_apple_maps, maps_dir=self.maps)
    self.token = ensure_token(self.params)
    if self.HANDLER_TIMEOUT:
      patcher = mock.patch.object(server.Handler, "timeout", self.HANDLER_TIMEOUT)
      patcher.start()
      self.addCleanup(patcher.stop)
    self.httpd = server.Server(("127.0.0.1", 0), server.make_handler(self.app))
    self.port = self.httpd.server_address[1]
    threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

  def teardown_method(self):
    self.httpd.shutdown()
    self.httpd.server_close()

  def call(self, method, path, body=None, auth="bearer", headers=None, content_type="application/json"):
    """(status, parsed body or bytes, response headers)."""
    h = dict(headers or {})
    if auth == "bearer":
      h["Authorization"] = f"Bearer {self.token}"
    elif auth == "cookie":
      h["Cookie"] = f"tp_token={self.token}"
    data = None
    if body is not None:
      data = body if isinstance(body, bytes) else json.dumps(body).encode()
    if method == "POST":
      h["Content-Type"] = content_type
    conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
    conn.request(method, path, body=data, headers=h)
    resp = conn.getresponse()
    raw = resp.read()
    conn.close()
    is_json = resp.getheader("Content-Type", "").startswith("application/json")
    return resp.status, json.loads(raw) if is_json else raw, dict(resp.getheaders())

  def raw(self, data: bytes, wait: float = 3.) -> bytes:
    """Send bytes down a plain socket and return everything the server answers before it closes the connection."""
    with socket.create_connection(("127.0.0.1", self.port), timeout=wait) as sock:
      sock.sendall(data)
      out = b""
      try:
        while chunk := sock.recv(4096):
          out += chunk
      except TimeoutError:
        raise AssertionError(f"the server kept the connection open for {wait} s; got {out!r}") from None
      return out


class TestToken(WebTestCase):
  def test_generated_once_and_persisted(self):
    assert len(self.token) >= 20
    assert self.params.get("TurnPilotWebToken") == self.token
    assert ensure_token(self.params) == self.token

  def test_token_ok(self):
    assert self.app.token_ok(["nope", self.token])
    assert not self.app.token_ok([])
    assert not self.app.token_ok(["", "nope", self.token[:-1], self.token + "x"])
    self.params.remove("TurnPilotWebToken")
    assert not self.app.token_ok([""])  # an unset token must not accept an empty credential

  def test_api_needs_the_token(self):
    for method, path in [("GET", "/api/status"), ("GET", "/api/favorites"), ("GET", "/api/search?q=cafe"), ("GET", "/api/pairing"),
                         ("POST", "/api/destination"), ("POST", "/api/stop"), ("DELETE", "/api/destination")]:
      for auth in ("none", "wrong"):
        headers = {"Authorization": "Bearer wrong"} if auth == "wrong" else None
        code, body, _ = self.call(method, path, body={}, auth="none", headers=headers)
        assert code == 401, (method, path, auth)
        assert body == {"error": "unauthorized"}
    assert self.params.get("NavDestination") is None

  def test_bearer_and_cookie(self):
    assert self.call("GET", "/api/status", auth="bearer")[0] == 200
    assert self.call("GET", "/api/status", auth="cookie")[0] == 200
    assert self.call("GET", "/api/status", auth="none", headers={"Authorization": f"Basic {self.token}"})[0] == 401

  def test_page_loads_without_a_token_and_the_api_does_not(self):
    page = server.INDEX.read_bytes()
    for path in ("/", "/?t=wrong", f"/?t={self.token}"):
      code, body, headers = self.call("GET", path, auth="none")
      assert code == 200 and body == page and headers["Content-Type"].startswith("text/html"), path
    assert b'id="unpaired"' in page  # what the page shows when the API says 401
    assert self.call("GET", "/nope", auth="none")[0] == 401  # only the page and the icon are open
    assert self.call("POST", "/", {}, auth="none")[0] == 401
    assert self.call("GET", f"/api/status?t={self.token}", auth="none")[0] == 401

  def test_query_token_sets_the_cookie_and_nothing_else_does(self):
    code, _, headers = self.call("GET", f"/?t={self.token}", auth="none")
    cookie = headers["Set-Cookie"]
    assert code == 200
    assert cookie.startswith(f"tp_token={self.token};") and "HttpOnly" in cookie and "SameSite=Strict" in cookie
    assert self.call("GET", "/api/status", auth="none", headers={"Cookie": cookie.split(";")[0]})[0] == 200

    for path, auth in [("/", "none"), ("/?t=wrong", "none"), ("/?t=", "none"), ("/", "bearer"), ("/?t=wrong", "cookie")]:
      assert "Set-Cookie" not in self.call("GET", path, auth=auth)[2], (path, auth)

  def test_cookie_never_holds_the_raw_query_value(self):
    # a request that offers the valid token too passes the check, so the junk next to it must not reach the cookie or any header
    junk = "wrong%0d%0aX-Injected:%201"
    for path in (f"/?t={junk}&t={self.token}", f"/?t={self.token}&t={junk}"):
      code, _, headers = self.call("GET", path, auth="none")
      assert code == 200 and "X-Injected" not in headers, path
      assert headers["Set-Cookie"].startswith(f"tp_token={self.token};"), path
    code, _, headers = self.call("GET", f"/?t={junk}", auth="cookie")
    assert code == 200 and "X-Injected" not in headers and "Set-Cookie" not in headers

  def test_icon_is_public_and_a_png(self):
    code, body, headers = self.call("GET", "/icon.png", auth="none")
    assert code == 200 and headers["Content-Type"] == "image/png"
    assert body[:8] == b"\x89PNG\r\n\x1a\n"
    width, height = struct.unpack(">II", body[16:24])
    assert (width, height) == (180, 180)
    idat = body[body.index(b"IDAT") + 4:body.index(b"IEND") - 8]
    assert len(zlib.decompress(idat)) == 180 * (1 + 180 * 3)

  def test_pair_svg_is_for_the_token_holder_or_the_device(self):
    assert self.call("GET", "/pair.svg", auth="bearer")[0] == 200
    self.monkeypatch_loopback(())
    assert self.call("GET", "/pair.svg", auth="none")[0] == 401
    assert self.call("GET", "/pair.svg", auth="bearer")[0] == 200
    self.monkeypatch_loopback(("127.0.0.1", "::1"))
    code, body, headers = self.call("GET", "/pair.svg", auth="none")
    assert code == 200 and headers["Content-Type"] == "image/svg+xml" and body.startswith(b"<svg")

  def monkeypatch_loopback(self, value):
    old = server.LOOPBACK
    server.LOOPBACK = value
    self.addCleanup(setattr, server, "LOOPBACK", old)


class TestHandlerDefaults(OpenpilotTestCase):
  def test_sockets_time_out_by_default(self):
    assert server.Handler.timeout == server.REQUEST_TIMEOUT and 0 < server.REQUEST_TIMEOUT <= 30


class TestMain(OpenpilotTestCase):
  def test_a_taken_port_is_retried(self):
    # the manager doesn't restart a dead process, so a failed bind must not end it
    failures, served = [OSError(48, "Address already in use")], []

    def make_server(address, handler):
      if failures:
        raise failures.pop()
      return mock.Mock(serve_forever=lambda: served.append(address))
    with mock.patch.object(server, "Server", side_effect=make_server), mock.patch.object(server, "StatusReader"), \
         mock.patch.object(server.time, "sleep") as sleep:
      server.main()
    assert served == [("", pairing.PORT)] and sleep.call_count == 1


class TestConnections(WebTestCase):
  """Sockets that stall or lie about their length must not hold a handler thread for good."""
  HANDLER_TIMEOUT = 0.4

  def post_head(self, length):
    headers = ["POST /api/destination HTTP/1.1", f"Authorization: Bearer {self.token}", "Content-Type: application/json",
               f"Content-Length: {length}", "", ""]
    return "\r\n".join(headers).encode()

  def test_idle_connection_is_closed(self):
    assert self.raw(b"") == b""

  def test_half_a_request_is_closed(self):
    assert self.raw(b"GET /api/status HTTP/1.1\r\nHost: x\r\n") == b""

  def test_short_body_times_out(self):
    reply = self.raw(self.post_head(50) + b'{"latitude"')
    assert reply.startswith(b"HTTP/1.0 408 ") and b"request timed out" in reply
    assert self.params.get("NavDestination") is None

  def test_bad_content_length_is_rejected_at_once(self):
    for length in ("-1", "-0", "abc", "1.5", "+5", "1_0", "\u00b2"):
      reply = self.raw(self.post_head(length))
      assert reply.startswith(b"HTTP/1.0 400 ") and b"bad Content-Length" in reply, (length, reply)
    assert self.call("POST", "/api/destination", {"latitude": 1, "longitude": 2}, headers={"Content-Length": "-1"})[0] == 400
    assert self.params.get("NavDestination") is None

  def test_too_long_body_is_still_413(self):
    assert self.call("POST", "/api/destination", b" " * (server.MAX_BODY + 1))[0] == 413

  def test_a_request_inside_the_timeout_still_works(self):
    assert self.call("POST", "/api/destination", {"latitude": 1, "longitude": 2})[0] == 200


class FakeSocket:
  def __init__(self, local_ip):
    self.local_ip = local_ip

  def getsockname(self):
    return (self.local_ip, pairing.PORT)


class TestReach(WebTestCase):
  """Plain HTTP on every interface: served on the hotspot and loopback, not on the LTE modem's address."""
  HANDLER_TIMEOUT = 3.

  def test_private_addresses(self):
    for ip in ("127.0.0.1", "::1", "172.20.10.2", "192.168.1.20", "10.0.0.5", "169.254.10.1"):
      assert server.private_address(ip), ip
    for ip in ("100.64.12.1", "8.8.8.8", "2607:f8b0::1", "", "not an address"):
      assert not server.private_address(ip), ip

  def test_only_private_local_and_peer_addresses_are_served(self):
    for local, peer, ok in (("172.20.10.3", "172.20.10.1", True), ("127.0.0.1", "127.0.0.1", True),
                            ("100.64.12.1", "1.1.1.1", False), ("8.8.4.4", "1.1.1.1", False),
                            ("172.20.10.3", "1.1.1.1", False)):
      sock = FakeSocket(local)
      assert self.httpd.verify_request(sock, (peer, 50000)) == ok, (local, peer)
      self.httpd.connections.discard(sock)

  def test_connections_are_capped(self):
    with mock.patch.object(server, "MAX_CONNECTIONS", 2):
      idle = [socket.create_connection(("127.0.0.1", self.port), timeout=5) for _ in range(2)]
      with socket.create_connection(("127.0.0.1", self.port), timeout=2) as third:
        assert third.recv(1) == b""  # closed unanswered at once, not after the handler timeout
      for sock in idle:
        sock.close()
      for _ in range(50):  # the idle sockets' handlers see them close
        if not self.httpd.connections:
          break
        time.sleep(0.05)
      assert self.call("GET", "/api/status")[0] == 200


class TestDestination(WebTestCase):
  def test_write_and_name(self):
    code, body, _ = self.call("POST", "/api/destination", {"latitude": 39.1, "longitude": -86.2, "name": "Cafe"})
    assert code == 200 and body == {"ok": True, "name": "Cafe", "latitude": 39.1, "longitude": -86.2}
    # navd and the UI read exactly these two keys
    assert self.params.get("NavDestination") == {"latitude": 39.1, "longitude": -86.2}
    assert self.call("GET", "/api/favorites")[1]["recents"] == [{"name": "Cafe", "latitude": 39.1, "longitude": -86.2}]
    assert self.call("GET", "/api/status")[1]["destination"] == {"name": "Cafe", "latitude": 39.1, "longitude": -86.2}

  def test_unnamed_destination_shows_coordinates(self):
    self.params.put("NavDestination", {"latitude": 1.5, "longitude": 2.5}, block=True)  # e.g. set over ssh
    assert self.call("GET", "/api/status")[1]["destination"]["name"] == "1.50000, 2.50000"

  def test_numbers_as_strings(self):
    assert self.call("POST", "/api/destination", {"latitude": "39.5", "longitude": "-86.5"})[0] == 200
    assert self.params.get("NavDestination") == {"latitude": 39.5, "longitude": -86.5}

  def test_bad_requests_leave_the_destination_alone(self):
    self.call("POST", "/api/destination", {"latitude": 39.1, "longitude": -86.2})
    good = self.params.get("NavDestination")
    for body in [{"latitude": 91, "longitude": 0}, {"latitude": 0, "longitude": -181}, {"latitude": "x", "longitude": 1},
                 {"latitude": True, "longitude": 1}, {"latitude": 1}, {}, {"url": "not a url"}, {"url": "https://example.com/"}]:
      assert self.call("POST", "/api/destination", body)[0] == 400, body
    assert self.call("POST", "/api/destination", b"{nope")[0] == 400
    assert self.call("POST", "/api/destination", b"[1]")[0] == 400
    assert self.call("POST", "/api/destination", {"latitude": 1, "longitude": 2}, content_type="text/plain")[0] == 415
    assert self.params.get("NavDestination") == good

  def test_stop(self):
    self.call("POST", "/api/destination", {"latitude": 39.1, "longitude": -86.2})
    assert self.call("POST", "/api/stop", {})[0] == 200
    assert self.params.get("NavDestination") is None
    assert self.call("GET", "/api/status")[1]["destination"] is None
    self.call("POST", "/api/destination", {"latitude": 39.1, "longitude": -86.2})
    assert self.call("DELETE", "/api/destination")[0] == 200
    assert self.params.get("NavDestination") is None
    assert self.call("POST", "/api/stop", {})[0] == 200  # clearing nothing is fine

  def test_apple_maps_link_with_coordinates(self):
    url = "https://maps.apple.com/?ll=39.7684%2C-86.1581&q=Monument%20Circle&t=m"
    code, body, _ = self.call("POST", "/api/destination", {"url": url})
    assert code == 200 and body["name"] == "Monument Circle"
    assert self.params.get("NavDestination") == {"latitude": 39.7684, "longitude": -86.1581}
    assert self.search.calls == []

  def test_apple_maps_link_with_only_an_address_is_geocoded(self):
    url = "https://maps.apple.com/?address=1%20Main%20St%2C%20Bloomington%2C%20IN&auid=1"
    code, body, _ = self.call("POST", "/api/destination", {"url": url})
    assert code == 200 and body["name"] == "1 Main St, Bloomington, IN"  # the shared address names it, the match only places it
    assert self.search.calls == [("1 Main St, Bloomington, IN", None)]
    assert self.params.get("NavDestination") == {"latitude": 39.1, "longitude": -86.2}

  def test_apple_maps_link_nothing_found(self):
    self.search.results = []
    assert self.call("POST", "/api/destination", {"url": "https://maps.apple.com/?q=zzzz"})[0] == 404
    assert self.params.get("NavDestination") is None

  def test_search_bias_uses_the_last_gps_fix(self):
    msg = messaging.new_message("gpsLocationExternal", valid=True)
    msg.gpsLocationExternal.latitude, msg.gpsLocationExternal.longitude = INDIANAPOLIS
    self.params.put("LastGPSPosition", msg.to_bytes(), block=True)
    assert last_gps_position(self.params) == INDIANAPOLIS
    code, body, _ = self.call("GET", "/api/search?q=coffee%20shop")
    assert code == 200 and body["results"][0]["name"] == "Cafe"
    assert self.search.calls == [("coffee shop", INDIANAPOLIS)]

  def test_search_edge_cases(self):
    code, body, _ = self.call("GET", "/api/search?q=a")
    assert (code, body) == (200, {"results": []}) and self.search.calls == []
    self.search.error = ConnectionError("offline")
    assert self.call("GET", "/api/search?q=cafe")[0] == 502


class TestGeofence(WebTestCase):
  """The destination endpoint against Indianapolis and Seattle, as the real map server draws them."""
  COVERAGE = coverage_doc()

  def navigate(self, lon_lat, name="Place"):
    return self.call("POST", "/api/destination", {"latitude": lon_lat[1], "longitude": lon_lat[0], "name": name})

  def test_inside_is_accepted(self):
    assert self.navigate(INDY)[0] == 200
    assert self.navigate(SEATTLE)[0] == 200  # the car's position is unknown here, so either area

  def test_outside_is_refused_and_nothing_is_saved(self):
    code, body, _ = self.navigate(CHICAGO, "Chicago")
    assert code == 422
    assert body["error"] == "Outside TurnPilot's mapped area. TurnPilot can only navigate around Indianapolis and Seattle."
    assert body["code"] == "outside_coverage"
    assert [r["id"] for r in body["regions"]] == ["indianapolis", "seattle"]
    assert self.params.get("NavDestination") is None
    assert self.call("GET", "/api/favorites")[1]["recents"] == []

  def test_apple_maps_links_are_checked_too(self):
    code, body, _ = self.call("POST", "/api/destination", {"url": "https://maps.apple.com/?ll=41.88,-87.63&q=Chicago"})
    assert code == 422 and "Outside" in body["error"]

  def test_car_position_is_checked(self):
    with mock.patch.object(self.app, "position", return_value=(CHICAGO[1], CHICAGO[0])):
      code, body, _ = self.navigate(INDY)
    assert code == 422 and body["error"].startswith("The car is outside")
    with mock.patch.object(self.app, "position", return_value=(SEATTLE[1], SEATTLE[0])):
      code, body, _ = self.navigate(INDY)
    assert code == 422 and body["code"] == "different_regions"

  def test_no_coverage_is_a_loud_refusal(self):
    coverage_path(self.maps).unlink()
    code, body, _ = self.navigate(INDY)
    assert code == 422 and body["code"] == "no_coverage" and "never downloaded" in body["error"]
    assert self.params.get("NavDestination") is None
    m = self.call("GET", "/api/status")[1]["map"]
    assert m["coverage"] is None and "never downloaded" in m["coverage_error"]
    assert self.call("GET", "/api/coverage")[1]["coverage"] is None

  def test_local_tiles_override_has_no_geofence(self):
    coverage_path(self.maps).unlink()  # as in the sim
    self.params.put("TurnPilotValhallaTiles", "/sim/tiles")
    assert self.navigate(CHICAGO)[0] == 200
    assert self.call("GET", "/api/status")[1]["map"]["override"] == "/sim/tiles"

  def test_status_map(self):
    assert self.call("GET", "/api/status")[1]["map"]["nav"] is None  # no destination, no navd status
    assert self.navigate(INDY)[0] == 200
    self.params.put(STATUS_PARAM, {"state": "downloading", "message": "Downloading map for this drive…", "short": "Downloading map",
                                   "code": "", "progress": 0.4, "destination": list(INDY), "at": 1})
    with mock.patch.object(self.app.store, "last_gps", return_value=(INDY[1], INDY[0])):
      m = self.call("GET", "/api/status")[1]["map"]
    assert m["car_region"] == "Indianapolis area" and not m["car_outside"]
    assert m["coverage"]["names"] == "Indianapolis and Seattle"
    assert m["nav"]["state"] == "downloading" and m["nav"]["progress"] == 0.4
    assert m["missing"] == ["address", "API key", "certificate"]
    with mock.patch.object(self.app.store, "last_gps", return_value=(CHICAGO[1], CHICAGO[0])):
      m = self.call("GET", "/api/status")[1]["map"]
    assert m["car_outside"] and m["car_region"] is None

  def test_status_of_another_destination_is_not_shown_as_this_ones(self):
    """navd's status still names the previous destination until it starts on the new one."""
    self.params.put(STATUS_PARAM, {"state": "error", "message": "The car is outside TurnPilot's mapped area.", "short": "Car outside",
                                   "code": "outside_coverage", "destination": [SEATTLE[0], SEATTLE[1]], "at": 1})
    assert self.navigate(INDY)[0] == 200
    assert self.call("GET", "/api/status")[1]["map"]["nav"] is None  # offroad navd doesn't run: nothing to check yet
    with mock.patch.object(server, "format_status", side_effect=lambda *a, **k: {**status.format_status(*a, **k), "onroad": True}):
      nav = self.call("GET", "/api/status")[1]["map"]["nav"]
    assert nav["state"] == "checking" and nav["destination"] == [INDY[0], INDY[1]], nav
    self.params.put(STATUS_PARAM, {"state": "ready", "message": "Ready", "short": "Ready", "code": "", "destination": [INDY[0], INDY[1]]})
    assert self.call("GET", "/api/status")[1]["map"]["nav"]["state"] == "ready"

  def test_coverage_endpoint(self):
    body = self.call("GET", "/api/coverage")[1]
    assert body["coverage"]["type"] == "FeatureCollection" and body["names"] == "Indianapolis and Seattle" and body["error"] is None

  def test_page_has_the_map_and_its_error(self):
    page = server.INDEX.read_text()
    assert 'id="mapview"' in page and 'id="maperror"' in page and "Downloading map for this drive" in page
    script = page.split("<script>")[1]
    assert "https://" not in script.replace('"http://www.w3.org/2000/svg"', "")  # no tile servers or other hosts


class TestMapSettings(WebTestCase):
  @property
  def map_server(self):
    """A fake map server, and no coverage yet: saving the settings fetches it."""
    if not hasattr(self, "_map_server"):
      self._map_server = MapServer()
      self.addCleanup(self._map_server.close)
      coverage_path(self.maps).unlink()
    return self._map_server

  def test_save_check_and_never_echo_the_key(self):
    assert self.map_server
    assert self.call("GET", "/api/map")[1] == {"host": "", "key_set": False, "key_hint": "", "cert_set": False, "cert_fingerprint": ""}
    code, body, _ = self.call("POST", "/api/map", {"host": self.map_server.host, "key": KEY, "cert": self.map_server.cert})
    assert code == 200 and body["coverage_ok"], body
    assert body["key_set"] and body["key_hint"] == KEY[-4:] and len(body["cert_fingerprint"]) == 95
    assert self.params.get("TurnPilotMapKey") == KEY
    for path in ("/api/map", "/api/status", "/api/coverage"):
      assert KEY not in json.dumps(self.call("GET", path)[1]), path
    assert self.call("GET", "/api/coverage")[1]["names"] == "Indianapolis and Seattle"  # fetched on save
    assert self.call("POST", "/api/destination", {"latitude": CHICAGO[1], "longitude": CHICAGO[0]})[0] == 422

  def test_an_empty_field_keeps_the_saved_value_and_errors_show(self):
    self.call("POST", "/api/map", {"host": self.map_server.host, "key": KEY, "cert": self.map_server.cert})
    code, body, _ = self.call("POST", "/api/map", {"key": "wrong-key-0000"})
    assert code == 200 and not body["coverage_ok"] and "API key" in body["coverage_error"]
    assert self.params.get("TurnPilotMapHost") == self.map_server.host
    assert self.params.get("TurnPilotMapCert").strip() == self.map_server.cert.strip()

  def test_a_new_host_or_certificate_needs_the_key_again(self):
    """The web token travels in the clear on the hotspot. Whoever has it must not be able to point the car at their own
    server and certificate and have it send the real map key there."""
    self.call("POST", "/api/map", {"host": self.map_server.host, "key": KEY, "cert": self.map_server.cert})
    attacker = MapServer()
    self.addCleanup(attacker.close)
    for body in ({"host": attacker.host, "cert": attacker.cert}, {"host": attacker.host}, {"host": self.map_server.host, "cert": attacker.cert}):
      code, reply, _ = self.call("POST", "/api/map", body)
      assert code == 400 and "API key" in reply["error"], (body, reply)
      assert self.params.get("TurnPilotMapHost") == self.map_server.host
      assert self.params.get("TurnPilotMapCert").strip() == self.map_server.cert.strip()
    assert not any(KEY in json.dumps(headers) for _, _, headers, _ in attacker.requests), "the key went to the new host"
    # the page always sends the saved host back: unchanged settings with a new key are fine, and so is a move typed in full
    assert self.call("POST", "/api/map", {"host": self.map_server.host, "key": KEY})[0] == 200
    code, reply, _ = self.call("POST", "/api/map", {"host": attacker.host, "cert": attacker.cert, "key": "a-new-key-for-the-new-server"})
    assert code == 200 and self.params.get("TurnPilotMapHost") == attacker.host

  def test_settings_the_client_cannot_use_are_refused_before_saving(self):
    for body in ({"host": "https://1.2.3.4:abc"}, {"host": "1.2.3.4 8443"}, {"key": "clé-ключ-12345678"}, {"host": "https://[zz]"},
                 {"host": "https://[::1"}, {"host": "https://h\\x"}, {"host": "https://h;x"}, {"host": "https://host:"}):
      code, reply, _ = self.call("POST", "/api/map", body)
      assert code == 400, reply
      assert "Failed to parse" not in reply["error"] and "codec" not in reply["error"]  # a message for the driver, not Python's
      if "host" in body:
        assert "map server address" in reply["error"] and "certificate" not in reply["error"], reply  # the address, not the cert
      assert self.params.get("TurnPilotMapHost") is None and self.params.get("TurnPilotMapKey") is None

  def test_a_failed_refresh_is_on_the_status_for_the_page(self):
    self.call("POST", "/api/map", {"host": self.map_server.host, "key": KEY, "cert": self.map_server.cert})
    self.call("POST", "/api/map", {"key": "wrong-key-0000"})
    m = self.call("GET", "/api/status")[1]["map"]
    assert m["coverage"] is not None and m["refresh"]["ok"] is False and "API key" in m["refresh"]["error"]
    script = server.INDEX.read_text().split("<script>")[1]
    assert "m.refresh" in script.split("function renderMap")[1].split("\nfunction ")[0]  # renderMap shows it

  def test_bad_settings_are_refused(self):
    assert self.call("POST", "/api/map", {"host": "http://136.113.85.41"})[0] == 400
    assert self.call("POST", "/api/map", {"cert": "hello"})[0] == 400
    assert self.call("POST", "/api/map", {})[0] == 400
    assert self.params.get("TurnPilotMapHost") is None


class TestFavorites(WebTestCase):
  def test_add_list_remove(self):
    code, fav, _ = self.call("POST", "/api/favorites", {"name": "Home", "latitude": 39.1, "longitude": -86.2})
    assert code == 200 and len(fav["id"]) == 8
    self.call("POST", "/api/favorites", {"name": "Work", "latitude": 39.2, "longitude": -86.3})
    names = [p["name"] for p in self.call("GET", "/api/favorites")[1]["favorites"]]
    assert names == ["Home", "Work"]
    assert self.params.get("TurnPilotFavorites")["favorites"][0]["id"] == fav["id"]

    assert self.call("DELETE", f"/api/favorites/{fav['id']}")[0] == 200
    assert [p["name"] for p in self.call("GET", "/api/favorites")[1]["favorites"]] == ["Work"]
    assert self.call("DELETE", f"/api/favorites/{fav['id']}")[0] == 404

  def test_saving_the_same_place_renames_it(self):
    first = self.call("POST", "/api/favorites", {"name": "Home", "latitude": 39.1, "longitude": -86.2})[1]
    again = self.call("POST", "/api/favorites", {"name": "Mom's", "latitude": 39.1, "longitude": -86.2})[1]
    assert again["id"] == first["id"]
    assert [p["name"] for p in self.call("GET", "/api/favorites")[1]["favorites"]] == ["Mom's"]

  def test_invalid_favorite(self):
    assert self.call("POST", "/api/favorites", {"name": "Nowhere", "latitude": 99, "longitude": 0})[0] == 400
    assert self.call("GET", "/api/favorites")[1] == {"favorites": [], "recents": []}

  def test_favorite_name_wins_for_the_destination(self):
    self.call("POST", "/api/favorites", {"name": "Home", "latitude": 39.1, "longitude": -86.2})
    self.call("POST", "/api/destination", {"name": "1 Main St", "latitude": 39.1, "longitude": -86.2})
    assert self.call("GET", "/api/status")[1]["destination"]["name"] == "Home"

  def test_recents_are_newest_first_deduped_and_capped(self):
    store = Store(self.params)
    for i in range(15):
      store.navigate(30 + i, -80, f"Place {i}")
    store.navigate(35, -80, "Place 5 again")
    recents = store.places()["recents"]
    assert len(recents) == 10
    assert recents[0]["name"] == "Place 5 again" and recents[1]["name"] == "Place 14"
    assert sum(p["latitude"] == 35 for p in recents) == 1

  def test_concurrent_requests_do_not_lose_writes(self):
    codes = []

    def add(i):
      body = {"name": f"Place {i}", "latitude": 30 + i, "longitude": -80}
      codes.append(self.call("POST", "/api/favorites", body)[0])
      codes.append(self.call("POST", "/api/destination", body)[0])
    threads = [threading.Thread(target=add, args=(i,)) for i in range(20)]
    for t in threads:
      t.start()
    for t in threads:
      t.join()
    assert codes == [200] * 40
    places = self.call("GET", "/api/favorites")[1]
    assert sorted(p["latitude"] for p in places["favorites"]) == [30 + i for i in range(20)]
    assert len(places["recents"]) == 10

  def test_survives_garbage_in_the_param(self):
    self.params.put("TurnPilotFavorites", ["not", "a", "dict"], block=True)
    assert self.call("GET", "/api/favorites")[1] == {"favorites": [], "recents": []}
    self.params.put("TurnPilotFavorites", {"favorites": [1, {"name": "ok", "latitude": 1, "longitude": 2}]}, block=True)
    assert [p["name"] for p in self.call("GET", "/api/favorites")[1]["favorites"]] == ["ok"]
    self.params.put("TurnPilotFavorites", {"favorites": 5, "recents": "x"}, block=True)
    assert self.call("GET", "/api/favorites")[1] == {"favorites": [], "recents": []}

  BAD_PLACES = [{"name": "no coordinates"}, {"name": "no longitude", "latitude": 1}, {"latitude": None, "longitude": 2},
                {"name": "text", "latitude": "39.1", "longitude": "-86.2"}, {"name": "bool", "latitude": True, "longitude": 1},
                {"name": "range", "latitude": 99, "longitude": 0}, {"name": "list", "latitude": [1], "longitude": 2}, 7, None, "x"]
  GOOD = {"name": "Home", "latitude": 39.1, "longitude": -86.2, "id": "abc12345"}

  def test_places_without_numeric_coordinates_are_dropped(self):
    self.params.put("TurnPilotFavorites", {"favorites": [*self.BAD_PLACES, self.GOOD], "recents": [*self.BAD_PLACES, self.GOOD]}, block=True)
    code, body, _ = self.call("GET", "/api/favorites")
    assert code == 200 and body == {"favorites": [self.GOOD], "recents": [self.GOOD]}
    # these used to raise KeyError in same_place or destination
    self.params.put("NavDestination", {"latitude": 39.1, "longitude": -86.2}, block=True)
    code, body, _ = self.call("GET", "/api/status")
    assert code == 200 and body["destination"]["name"] == "Home"
    assert self.call("POST", "/api/destination", {"latitude": 40, "longitude": -86, "name": "New"})[0] == 200
    assert self.call("POST", "/api/favorites", {"latitude": 41, "longitude": -86, "name": "Fav"})[0] == 200
    saved = self.params.get("TurnPilotFavorites")
    assert [p["name"] for p in saved["recents"]] == ["New", "Home"]
    assert [p["name"] for p in saved["favorites"]] == ["Home", "Fav"]  # writing again also cleans the stored param

  def test_a_place_without_a_name_gets_its_coordinates(self):
    self.params.put("TurnPilotFavorites", {"favorites": [{"latitude": 1.5, "longitude": 2.5, "id": "x"}]}, block=True)
    self.params.put("NavDestination", {"latitude": 1.5, "longitude": 2.5}, block=True)
    assert self.call("GET", "/api/favorites")[1]["favorites"][0]["name"] == "1.50000, 2.50000"
    assert self.call("GET", "/api/status")[1]["destination"]["name"] == "1.50000, 2.50000"


class TestPlaces(OpenpilotTestCase):
  APPLE = [
    # old share link
    ("https://maps.apple.com/?ll=39.7684,-86.1581&q=Monument%20Circle&t=m",
     {"latitude": 39.7684, "longitude": -86.1581, "name": "Monument Circle"}),
    # newer links use coordinate=
    ("https://maps.apple.com/place?coordinate=39.7684,-86.1581&name=Soldiers%20Monument&address=x",
     {"latitude": 39.7684, "longitude": -86.1581, "name": "Soldiers Monument"}),
    ("https://maps.apple.com/?auid=123&ll=39.1%2C-86.2&lsp=9902&q=Cafe%20Roma", {"latitude": 39.1, "longitude": -86.2, "name": "Cafe Roma"}),
    ("http://maps.apple.com/?daddr=40.0,-75.5", {"latitude": 40.0, "longitude": -75.5, "name": ""}),
    ("https://maps.apple.com/?q=40.5,-75.25", {"latitude": 40.5, "longitude": -75.25, "name": ""}),
    ("maps://?ll=-33.86,151.21&q=Sydney", {"latitude": -33.86, "longitude": 151.21, "name": "Sydney"}),
    ("https://maps.apple.com/?sll=39.5,-86.5&q=Pizza", {"latitude": 39.5, "longitude": -86.5, "name": "Pizza"}),
    # no coordinates: only something to geocode
    ("https://maps.apple.com/?q=Starbucks", {"query": "Starbucks", "name": "Starbucks"}),
    ("https://maps.apple.com/?address=1%20Main%20St&q=Cafe", {"query": "1 Main St", "name": "Cafe"}),
    # junk
    ("https://maps.apple.com/", None),
    ("ftp://maps.apple.com/?ll=1,2", None),
    ("hello", None),
    ("https://maps.apple.com/?ll=95,200", None),
  ]

  def test_parse_apple_maps(self):
    for url, expected in self.APPLE:
      assert places.parse_apple_maps(url) == expected, url

  def test_short_link_is_followed_only_on_apple_maps(self):
    short = "https://maps.apple.com/p/abc123"
    seen = []

    def get(url):
      seen.append(url)
      return "https://maps.apple.com/?ll=39.5%2C-86.5&q=Park", b""
    assert places.resolve_apple_maps(short, get) == {"latitude": 39.5, "longitude": -86.5, "name": "Park"}

    def page(url):
      return url, b'<a href="/place?foo=1&amp;coordinate=39.25%2C-86.75">'
    assert places.resolve_apple_maps(short, page) == {"latitude": 39.25, "longitude": -86.75, "name": ""}

    def elsewhere(url):
      return "https://evil.example/x", b"ll=1.5,2.5"
    assert places.resolve_apple_maps(short, elsewhere) is None

    def boom(url):
      raise OSError("offline")
    assert places.resolve_apple_maps(short, boom) is None

    seen.clear()
    assert places.resolve_apple_maps("https://evil.example/p/abc?ll=1,2", get) == {"latitude": 1.0, "longitude": 2.0, "name": ""}
    assert places.resolve_apple_maps("https://evil.example/p/abc", get) is None
    assert seen == []  # never fetches a host other than maps.apple.com

  def test_photon_results(self):
    data = {"features": [
      photon_feature("Cafe Roma", 39.1, -86.2, street="Main St", housenumber="12", city="Bloomington", state="Indiana", country="USA"),
      {"geometry": {"coordinates": [-86.0, 39.0]}, "properties": {"street": "Oak Ave", "housenumber": "3", "city": "Carmel"}},
      photon_feature("Broken", 999, 0),
      {"properties": {"name": "No geometry"}},
    ]}
    assert places.photon_places(data) == [
      {"name": "Cafe Roma", "detail": "12 Main St, Bloomington, Indiana, USA", "latitude": 39.1, "longitude": -86.2},
      {"name": "3 Oak Ave", "detail": "Carmel", "latitude": 39.0, "longitude": -86.0},
    ]

  def test_nominatim_results(self):
    data = [{"lat": "39.1", "lon": "-86.2", "name": "Cafe Roma", "display_name": "Cafe Roma, Main St, Bloomington"},
            {"lat": "39.2", "lon": "-86.3", "name": "", "display_name": "Oak Ave, Carmel"}, {"lat": "x", "lon": "1"}]
    assert places.nominatim_places(data) == [
      {"name": "Cafe Roma", "detail": "Cafe Roma, Main St, Bloomington", "latitude": 39.1, "longitude": -86.2},
      {"name": "Oak Ave", "detail": "Oak Ave, Carmel", "latitude": 39.2, "longitude": -86.3},
    ]

  def test_search_uses_photon_with_the_bias(self):
    urls = []

    def get_json(url):
      urls.append(url)
      return {"features": [photon_feature("Cafe", 39.1, -86.2)]}
    found = places.search("cafe roma", INDIANAPOLIS, get_json)
    assert found[0]["name"] == "Cafe" and len(urls) == 1
    assert urls[0].startswith("https://photon.komoot.io/api/?q=cafe+roma&limit=8&lat=39.76840&lon=-86.15810")

  def test_search_falls_back_to_nominatim(self):
    urls = []

    def photon_down(url):
      urls.append(url)
      if "photon" in url:
        raise OSError("down")
      return [{"lat": "39.1", "lon": "-86.2", "display_name": "Cafe, Bloomington"}]
    assert places.search("cafe", INDIANAPOLIS, photon_down)[0]["name"] == "Cafe"
    assert "nominatim.openstreetmap.org/search?q=cafe&format=jsonv2&limit=8&viewbox=" in urls[1]

    def photon_empty(url):
      return {"features": []} if "photon" in url else [{"lat": "1", "lon": "2", "display_name": "Somewhere"}]
    assert places.search("somewhere", None, photon_empty)[0]["name"] == "Somewhere"

  def test_search_failures(self):
    def down(url):
      raise OSError("down")
    try:
      places.search("cafe", None, down)
      raise AssertionError("expected ConnectionError")
    except ConnectionError:
      pass

    def photon_only_empty(url):
      return {"features": []} if "photon" in url else []
    assert places.search("zzzz", None, photon_only_empty) == []

  def test_user_agent_identifies_the_app(self):
    assert places.USER_AGENT.startswith("turnpilot-web/") and "http" in places.USER_AGENT


class TestAppleFetch(OpenpilotTestCase):
  """fetch_apple against throwaway servers on loopback, with APPLE_HOST pointed at 127.0.0.1."""
  def setup_method(self):
    patcher = mock.patch.object(places, "APPLE_HOST", "127.0.0.1")
    patcher.start()
    self.addCleanup(patcher.stop)

  def serve(self, routes):
    """(base url, hits). A route is (status, headers, body), or a function that answers on the handler itself."""
    hits = []

    class Handler(BaseHTTPRequestHandler):
      def do_GET(self):
        hits.append(self.path)
        route = routes.get(self.path.split("?")[0])
        if route is None:
          return self.send_error(404)
        if callable(route):
          return route(self)
        status, headers, body = route
        self.send_response(status)
        for k, v in headers.items():
          self.send_header(k, v)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

      def log_message(self, format, *args):  # noqa: A002
        pass

    httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, args=(0.05,), daemon=True).start()  # shutdown waits one poll
    self.addCleanup(httpd.server_close)
    self.addCleanup(httpd.shutdown)
    return f"http://127.0.0.1:{httpd.server_address[1]}", hits

  def test_follows_redirects_on_the_apple_host(self):
    base, _ = self.serve({"/p/short": (302, {"Location": "/p/hop"}, b""), "/p/hop": (301, {"Location": "/?ll=39.5%2C-86.5&q=Park"}, b""),
                          "/": (200, {}, b"<html>ok</html>")})
    assert places.fetch_apple(f"{base}/p/short") == (f"{base}/?ll=39.5%2C-86.5&q=Park", b"<html>ok</html>")
    assert places.resolve_apple_maps(f"{base}/p/short") == {"latitude": 39.5, "longitude": -86.5, "name": "Park"}

  def test_a_redirect_off_the_apple_host_is_never_requested(self):
    other, other_hits = self.serve({"/": (200, {}, b"ll=1.5,2.5")})
    port = other.rsplit(":", 1)[1]
    for location in (f"http://localhost:{port}/", f"//localhost:{port}/", f"http://127.0.0.1@localhost:{port}/"):
      base, hits = self.serve({"/p/x": (302, {"Location": location}, b"")})
      with self.assertRaises(OSError):
        places.fetch_apple(f"{base}/p/x")
      assert places.resolve_apple_maps(f"{base}/p/x") is None
      assert hits == ["/p/x", "/p/x"] and other_hits == [], location  # the Apple host was asked, the other host never was

  def test_redirect_loops_end(self):
    base, hits = self.serve({"/p/loop": (302, {"Location": "/p/loop"}, b"")})
    with self.assertRaises(OSError):
      places.fetch_apple(f"{base}/p/loop")
    assert len(hits) <= 5

  def test_gives_up_after_the_total_timeout(self):
    def drip(handler):  # a server that never finishes: each read gets a byte in time, so only the total deadline stops it
      handler.send_response(200)
      handler.end_headers()
      try:
        for _ in range(60):
          handler.wfile.write(b"x")
          handler.wfile.flush()
          time.sleep(0.05)
      except OSError:
        pass

    def stall(handler):  # accepts the request and never answers
      time.sleep(2)

    def slow_hop(handler):  # the redirect arrives late, and the page behind it is slow too
      time.sleep(0.4)
      handler.send_response(302)
      handler.send_header("Location", "/p/stall")
      handler.send_header("Content-Length", "0")
      handler.end_headers()

    base, _ = self.serve({"/p/drip": drip, "/p/stall": stall, "/p/hop": slow_hop})
    for path in ("/p/drip", "/p/stall", "/p/hop"):
      start = time.monotonic()
      with self.assertRaises(OSError):  # TimeoutError and URLError are both OSErrors
        places.fetch_apple(base + path, total=0.7)
      assert time.monotonic() - start < 1.5, path
    assert places.APPLE_TIMEOUT < places.TIMEOUT


class TestPairing(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()

  def test_pairing_url(self):
    url = pairing.pairing_url(self.params)
    token = self.params.get("TurnPilotWebToken")
    assert re.fullmatch(rf"http://\d+\.\d+\.\d+\.\d+:8088/\?t={token}", url), url

  def test_qr_round_trips(self):
    url = pairing.pairing_url(self.params)
    svg = pairing.qr_svg(url)
    border = 4
    side = int(re.findall(r'viewBox="0 0 (\d+) ', svg)[0])
    n = side - 2 * border
    modules = np.zeros((n, n), dtype=bool)
    for x, y in re.findall(r"M(\d+) (\d+)h1v1h-1z", svg):
      modules[int(y) - border, int(x) - border] = True
    assert decode_matrix(modules) == url


class TestStatus(OpenpilotTestCase):
  ROUTE = {"text": "Main St", "type": "turn", "modifier": "left", "maneuver_m": 310., "remaining_m": 6600., "remaining_s": 780.}
  PLAN = {"state": "following", "reason": "", "shadow": False}

  def snap(self, **kw):
    return {"at": 100., "onroad": True, "route": self.ROUTE, "plan": self.PLAN, "gps": (39.7, -86.1), **kw}

  def test_navigating(self):
    dest = {"name": "Home", "latitude": 1, "longitude": 2}
    s = status.format_status(self.snap(), dest, True, False, now=101.)
    assert s == {
      "reading": True, "onroad": True, "destination": dest, "navigating": True,
      "maneuver": {"text": "Turn left onto Main St", "distance": "1020 ft"},
      "remaining": {"distance": "4.1 mi", "duration": "13 min", "seconds": 780.},
      "turnpilot": {"enabled": True, "state": "following", "label": "Following route", "reason": "", "shadow": False},
      "gps": {"latitude": 39.7, "longitude": -86.1},
    }
    metric = status.format_status(self.snap(), dest, True, True, now=101.)
    assert metric["maneuver"]["distance"] == "310 m" and metric["remaining"]["distance"] == "6.6 km"

  def test_no_route(self):
    s = status.format_status(self.snap(route=None, onroad=False, gps=None), None, True, False, now=101.)
    assert s["navigating"] is False and s["maneuver"] is None and s["remaining"] is None and s["gps"] is None
    assert s["onroad"] is False and s["destination"] is None

  def test_turnpilot_states(self):
    plan = {"state": "holding", "reason": "pedestrian in crosswalk", "shadow": True}
    tp = status.format_status(self.snap(plan=plan), None, True, False, now=101.)["turnpilot"]
    assert (tp["state"], tp["label"], tp["reason"], tp["shadow"]) == ("holding", "Holding", "pedestrian in crosswalk", True)
    tp = status.format_status(self.snap(plan=None), None, False, False, now=101.)["turnpilot"]
    assert tp == {"enabled": False, "state": "offline", "label": "Not running", "reason": "TurnPilot is off", "shadow": False}

  def test_offline_reason_names_the_cause(self):
    def reason(onroad, enabled, plan=None, **kw):
      return status.format_status(self.snap(onroad=onroad, plan=plan), None, enabled, False, **{"now": 101., **kw})["turnpilot"]["reason"]
    assert reason(onroad=True, enabled=False) == "TurnPilot is off"
    assert reason(onroad=False, enabled=False) == "TurnPilot is off"
    assert reason(onroad=False, enabled=True) == "car is off"
    assert reason(onroad=True, enabled=True) == "turnpilotd is not running"
    assert reason(onroad=False, enabled=True, plan=self.PLAN) == ""  # a plan speaks for itself, whatever the car says
    # a stale or missing snapshot has no onroad to go by, so it must not claim the car is off
    assert reason(onroad=True, enabled=True, now=200.) == "car data is not available"
    assert status.format_status(None, None, True, False)["turnpilot"]["reason"] == "car data is not available"

  def test_stale_or_missing_snapshot_reads_as_unavailable(self):
    for snap, now in [(None, 0.), (self.snap(), 100. + status.STALE_AFTER + 1)]:
      s = status.format_status(snap, None, True, False, now=now)
      assert s["reading"] is False and s["onroad"] is False and s["navigating"] is False
      assert s["turnpilot"]["state"] == "offline"

  def test_text_helpers(self):
    assert [status.format_distance(m, False) for m in (0, 44, 120, 160, 400, 1609, 8000, 16100, 40000)] == \
           ["0 ft", "140 ft", "390 ft", "520 ft", "0.2 mi", "1.0 mi", "5.0 mi", "10 mi", "25 mi"]
    assert [status.format_distance(m, True) for m in (4, 120, 999, 1500, 9500, 12000)] == ["0 m", "120 m", "1000 m", "1.5 km", "9.5 km", "12 km"]
    assert [status.format_duration(s) for s in (10, 60, 125, 3540, 3600, 4500)] == ["<1 min", "1 min", "2 min", "59 min", "1 h 00 min", "1 h 15 min"]
    assert status.maneuver_text("turn", "slight right", "Oak Ave") == "Turn slight right onto Oak Ave"
    assert status.maneuver_text("arrive", "right", "12 Main St") == "Arrive at 12 Main St"
    assert status.maneuver_text("arrive", "", "") == "Arrive"
    assert status.maneuver_text("off ramp", "right", "I-65 N") == "Take exit right onto I-65 N"
    assert status.maneuver_text("", "", "Main St") == "Main St"

  def test_status_endpoint_without_a_reader(self):
    app = server.App(Params(), reader=None, maps_dir=Path("/nonexistent/turnpilot/maps"))
    assert app.status()["reading"] is False


class TestStatusReader(WebTestCase):
  """Real cereal messages through a real SubMaster, into the /api/status JSON."""
  def test_end_to_end(self):
    reader = status.StatusReader()
    self.app.reader = reader
    reader.start()
    self.addCleanup(reader.stop)

    pm = messaging.PubMaster(["deviceState", "navInstruction", "turnpilotPlan", "gpsLocation"])
    self.call("POST", "/api/destination", {"latitude": 39.1, "longitude": -86.2, "name": "Home"})
    self.params.put_bool("IsMetric", True)

    def publish():
      msg = messaging.new_message("deviceState", valid=True)
      msg.deviceState.started = True
      pm.send("deviceState", msg)
      msg = messaging.new_message("navInstruction", valid=True)
      ni = msg.navInstruction
      ni.maneuverPrimaryText, ni.maneuverType, ni.maneuverModifier = "Main Street", "turn", "left"
      ni.maneuverDistance, ni.distanceRemaining, ni.timeRemaining = 250., 5400., 600.
      pm.send("navInstruction", msg)
      msg = messaging.new_message("turnpilotPlan")  # turnpilotd leaves valid unset
      msg.turnpilotPlan.state = "approaching"
      msg.turnpilotPlan.reason = "waiting for oncoming"
      pm.send("turnpilotPlan", msg)
      msg = messaging.new_message("gpsLocation", valid=True)
      msg.gpsLocation.hasFix, msg.gpsLocation.latitude, msg.gpsLocation.longitude = True, 39.7, -86.1
      pm.send("gpsLocation", msg)

    body = {}
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
      publish()
      time.sleep(0.2)
      body = self.call("GET", "/api/status")[1]
      if body["navigating"] and body["gps"] and body["turnpilot"]["state"] != "offline":
        break
    assert body["reading"] and body["onroad"] and body["navigating"]
    assert body["destination"]["name"] == "Home"
    assert body["maneuver"] == {"text": "Turn left onto Main Street", "distance": "250 m"}
    assert body["remaining"] == {"distance": "5.4 km", "duration": "10 min", "seconds": 600.}
    assert (body["turnpilot"]["state"], body["turnpilot"]["reason"]) == ("approaching", "waiting for oncoming")
    assert body["gps"] == {"latitude": 39.7, "longitude": -86.1}
    assert self.app.position() == (39.7, -86.1)  # the live fix is what searches bias towards
    assert "points" not in reader.get()["route"] and "navRoute" not in status.SERVICES

  def test_snapshots_come_at_most_twice_a_second(self):
    calls = []
    real = status.read_services

    def counting(sm):
      calls.append(time.monotonic())
      return real(sm)
    patcher = mock.patch.object(status, "read_services", counting)
    patcher.start()
    self.addCleanup(patcher.stop)
    reader = status.StatusReader()
    reader.start()
    self.addCleanup(reader.stop)

    pm = messaging.PubMaster(["turnpilotPlan"])
    sent, end = 0, time.monotonic() + 1.6
    while time.monotonic() < end:  # a service at about 100 Hz used to mean a snapshot for every message
      pm.send("turnpilotPlan", messaging.new_message("turnpilotPlan"))
      sent += 1
      time.sleep(0.01)
    assert sent > 60
    assert 2 <= len(calls) <= 5, calls
    assert all(b - a >= 0.45 for a, b in zip(calls, calls[1:], strict=False)), calls
    assert reader.get()["plan"] is not None  # and the newest message still gets through


class TestMapHoldStatus(OpenpilotTestCase):
  def test_hold_and_unavailable_on_the_phone(self):
    snap = {"at": 100., "onroad": True, "route": None, "gps": None,
            "plan": {"state": "holding", "reason": "Stop sign: take control to proceed", "shadow": False,
                     "map": {"kind": "stop", "action": "awaitingDriver", "light": "unknown", "source": "none", "status": "ok"}}}
    tp = status.format_status(snap, None, True, False, now=101.)["turnpilot"]
    assert (tp["label"], tp["reason"]) == ("Stopped at stop sign", "take control")
    snap["plan"] = {"state": "following", "reason": "Map stops/signals: unsupported", "shadow": False,
                    "map": {"kind": "none", "action": "none", "light": "unknown", "source": "none", "status": "unsupported"}}
    tp = status.format_status(snap, None, True, False, now=101.)["turnpilot"]
    assert (tp["label"], tp["reason"]) == ("Following route", "Map stops/signals: unsupported")
    # an old snapshot without map fields still reads
    snap["plan"] = {"state": "holding", "reason": "pedestrian in path", "shadow": False}
    assert status.format_status(snap, None, True, False, now=101.)["turnpilot"]["label"] == "Holding"
