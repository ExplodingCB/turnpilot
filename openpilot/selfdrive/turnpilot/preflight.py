#!/usr/bin/env python3
"""preflight: a pass/fail checklist for a TurnPilot install, run on the comma over SSH.

  cd /data/openpilot && python3 -m openpilot.selfdrive.turnpilot.preflight

  --set KEY=VALUE   set a param first (python3 -m openpilot.common.params can't write bools); repeatable
  --qr              print the pairing URL as a QR code the phone camera can scan from the terminal

Exits 1 if any check fails. The check_* functions are pure: collect() reads the device and hands them plain values.
"""
import argparse
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, is_chestnut_usb_id
from openpilot.common.version import RELEASE_BRANCHES

PASS, FAIL, WARN, INFO = "PASS", "FAIL", "WARN", "INFO"
CAR_CAMERA = (1344, 760)  # the comma four's road and wide cameras, so the detector pkl to look for
MIN_PKL_BYTES = 1024      # a git LFS pointer is about 130 bytes
USB3_MBPS = 5000
WEB_PORT_TIMEOUT_S = 1.

# the first leg of the routing test: Monument Circle to the Indiana Statehouse area, about 2 km on public roads
ROUTE_FROM, ROUTE_TO = (-86.1581, 39.7684), (-86.1465, 39.7792)  # (longitude, latitude)
ROUTE_KM = (0.5, 10.)

SETTINGS_DEVELOPER = "Settings > Developer"


@dataclass(frozen=True)
class Check:
  name: str
  status: str
  detail: str = ""
  fix: str = ""


def check_branch(channel: str, is_release_param: bool) -> Check:
  name = "non-release branch"
  if channel in RELEASE_BRANCHES or is_release_param:
    why = f"{channel} is a release branch" if channel in RELEASE_BRANCHES else "the manager still reports a release branch"
    return Check(name, FAIL, why + ": no alpha longitudinal toggle, and the panda gets the release build",
                 "install from installer.comma.ai/kaileh57/turnpilot, or reboot if the branch just changed")
  return Check(name, PASS, channel)


def check_car(cp) -> Check:
  """cp: CarParams from CarParamsPersistent, or None before the car has been seen once."""
  name = "car"
  if cp is None:
    return Check(name, FAIL, "no car seen yet", "turn the car on once with the comma connected, then turn it off")
  car = f"{cp.brand} {cp.carFingerprint}"
  if cp.dashcamOnly:
    return Check(name, FAIL, f"{car} is in dashcam mode, openpilot won't control it",
                 "for a Tesla this means the car's software is too old for this openpilot")
  return Check(name, PASS, car)


def check_alpha_long(enabled: bool, cp) -> Check:
  name = "alpha longitudinal"
  if cp is not None and not cp.alphaLongitudinalAvailable:
    return Check(name, FAIL, "this car has no alpha longitudinal")
  if not enabled:
    return Check(name, FAIL, "AlphaLongitudinalEnabled is off",
                 f"{SETTINGS_DEVELOPER} > alpha longitudinal (it shows once the car has been seen, on a non-release branch), then restart")
  if cp is not None and not cp.openpilotLongitudinalControl:
    return Check(name, FAIL, "on, but the last drive started without it", "turn the car off and on")
  return Check(name, PASS, "on")


def check_toggle(name: str, key: str, enabled: bool, fix: str) -> Check:
  return Check(name, PASS, key) if enabled else Check(name, FAIL, f"{key} is off", fix)


def check_gas_disengages(enabled: bool) -> Check:
  """DisengageOnAccelerator: any gas press disengages (Kellen, 2026-10-28: no gas go anywhere; the press is the driver
  taking over, never TurnPilot's go)."""
  return check_toggle("gas disengages", "DisengageOnAccelerator", enabled,
                      "Settings > Toggles > Disengage on accelerator pedal, or preflight --set DisengageOnAccelerator=1")


def check_shadow(enabled: bool, shadow: bool) -> Check:
  name = "shadow mode"
  if not enabled:
    return Check(name, INFO, "TurnPilot is off")
  if shadow:
    return Check(name, INFO, "on: TurnPilot computes its plan and nothing acts on it")
  return Check(name, WARN, "OFF: TurnPilot will take turns, change lanes and brake",
               "use shadow mode on the first drives: preflight --set TurnPilotShadowMode=1")


def check_route_source(mode: str | None, override: str | None) -> Check:
  """Routes come from TurnPilot's map server, or from local tiles when TurnPilotValhallaTiles is set. Mapbox is gone."""
  name = "route source"
  if (mode or "auto").strip().lower() == "mapbox":
    return Check(name, FAIL, "TurnPilotRouteSource is mapbox, and Mapbox routing was removed", "preflight --set TurnPilotRouteSource=auto")
  if override:
    return Check(name, WARN, f"local tiles at {override} (developer override: no geofence, no map server)",
                 "for the car, clear TurnPilotValhallaTiles so routes come from the map server")
  return Check(name, INFO, "TurnPilot map server, a corridor downloaded per drive")


def check_map_server(missing: list[str], coverage_names: str | None, error: str | None, override: str | None,
                     refresh: dict | None = None) -> Check:
  """refresh: TurnPilotMapCoverageStatus, how the last coverage fetch went. A cached area with a failing server (a
  rotated key, a new certificate, the server down) would fail the drive's corridor download."""
  name = "map server"
  if override:
    return Check(name, INFO, "not used with TurnPilotValhallaTiles set")
  if missing:
    return Check(name, FAIL, f"not set up: no {', '.join(missing)}", "enter the address, API key and certificate in the web app's Map server section")
  if error:
    return Check(name, FAIL, error, "check the web app's Map server section with the comma online")
  if not coverage_names:
    return Check(name, FAIL, "no mapped area downloaded yet", "open the web app with the comma online; it fetches the mapped area")
  if isinstance(refresh, dict) and refresh.get("ok") is False:
    return Check(name, FAIL, f"mapped area: {coverage_names}, but updating it failed: {refresh.get('error') or refresh.get('code')}",
                 "check the web app's Map server section with the comma online")
  return Check(name, PASS, f"mapped area: {coverage_names}")


def check_pyvalhalla(version: str | None, needed: bool = True) -> Check:
  name = "pyvalhalla"
  if not needed:
    return Check(name, INFO, "not needed")
  if version is None:
    return Check(name, FAIL, "not importable", "bash /data/openpilot/tools/turnpilot/install_valhalla_device.sh")
  return Check(name, PASS, version)


def check_tiles(tiles: str | None, where: dict | None, needed: bool) -> Check:
  """The TurnPilotValhallaTiles override. Without it there are no tiles to check: each drive downloads its own."""
  name = "valhalla tiles"
  if not needed:
    return Check(name, INFO, "downloaded from the map server for each drive")
  if not tiles:
    return Check(name, FAIL, "TurnPilotValhallaTiles is not set", "copy tiles.tar over (see INSTALL.md), then set TurnPilotValhallaTiles")
  if where is None:
    return Check(name, FAIL, f"nothing usable at {tiles}", "the directory needs tiles.tar, or a tiles/ directory")
  if 'url' in where:
    return Check(name, PASS, f"valhalla_service at {where['url']}")
  if where.get('extract'):
    size = Path(where['extract']).stat().st_size / 1e6
    return Check(name, PASS, f"{where['extract']} ({size:.0f} MB)")
  return Check(name, PASS, where.get('config') or where['dir'])


def check_route(error: str | None, meters: float, seconds: float) -> Check:
  name = "routing test"
  if error:
    return Check(name, FAIL, f"routing failed: {error}")
  if not ROUTE_KM[0] * 1000 < meters < ROUTE_KM[1] * 1000:
    return Check(name, FAIL, f"Monument Circle to the Statehouse area gave {meters / 1000:.1f} km, expected about 2 km",
                 "this test assumes Indiana tiles")
  return Check(name, PASS, f"{meters / 1000:.1f} km in Indianapolis, {seconds * 1000:.0f} ms")


def check_big_model(compiled: bool) -> Check:
  name = "big driving model"
  if compiled:
    return Check(name, PASS, "big_driving_tinygrad.pkl and both warps are real files")
  return Check(name, FAIL, "missing, or a git LFS pointer: modeld would run the small model",
               "cd /data/openpilot && git lfs pull")


def check_detector(path: Path, chestnut: bool) -> Check:
  name = "detector pkl"
  size = path.stat().st_size if path.is_file() else 0
  if size > MIN_PKL_BYTES:
    return Check(name, PASS, f"{path.name} ({size / 1e6:.1f} MB)")
  why = "empty or a pointer" if path.is_file() else "missing"
  note = "" if chestnut else " (no chestnut, so this is the plain detector)"
  return Check(name, FAIL, f"{path.name} is {why}{note}: the planner holds every left turn without a detector",
               "update to a branch that carries it (compile_detector.py builds it on the Mac)")


def check_chestnut(devices: list[dict]) -> Check:
  name = "chestnut"
  seen = [d for d in devices if is_chestnut_usb_id(d["vendorId"], d["productId"], include_bootloader=True)]
  running = [d for d in seen if is_chestnut_usb_id(d["vendorId"], d["productId"])]
  if not seen:
    return Check(name, FAIL, "not detected", "check the USB-C cable and the 12 V supply, then power the comma off and on")
  if len(running) != 1 or running[0]["product"] != CHESTNUT_USB_PRODUCT:
    return Check(name, FAIL, f"detected, but not running {CHESTNUT_USB_PRODUCT}", "reboot so the chestnut firmware updater runs")
  speed = running[0]["speedMbps"]
  if speed < USB3_MBPS:
    return Check(name, FAIL, f"USB link is {speed} Mbps, the detector upload needs USB 3", "reseat the cable or try the other USB-C orientation")
  return Check(name, PASS, f"present, USB {speed} Mbps")


def check_token(token: str | None) -> Check:
  name = "web token"
  if token:
    return Check(name, PASS, "set")
  return Check(name, FAIL, "TurnPilotWebToken is not set",
               "turnpilot_web creates it when it starts, a few seconds after TurnPilotEnabled is set: run preflight again")


def check_pairing_url(token: str | None, ip: str, port: int) -> Check:
  name = "pairing URL"
  if not token:
    return Check(name, FAIL, "no token yet")
  if ip.startswith("127."):
    return Check(name, FAIL, "the comma has no network address", "join the phone's Personal Hotspot in Settings > Network")
  return Check(name, PASS, f"http://{ip}:{port}/?t={token}")


def check_web(listening: bool, enabled: bool) -> Check:
  name = "web server"
  if listening:
    return Check(name, PASS, "answering on port 8088")
  if enabled:
    return Check(name, FAIL, "turnpilot_web is not answering on port 8088", "the manager starts it when TurnPilotEnabled is on: reboot if it stays down")
  return Check(name, INFO, "not running, TurnPilotEnabled is off")


def parse_assignment(params, assignment: str):
  """(key, value) for 'KEY=VALUE', with the value cast to the param's type. Raises ValueError if it can't be set this way."""
  key, sep, text = assignment.partition("=")
  if not sep:
    raise ValueError(f"{assignment!r}: expected KEY=VALUE")
  try:
    kind = params.get_type(key).name
  except Exception:  # Params raises UnknownKeyName
    raise ValueError(f"{key} is not a param") from None
  if kind == "BOOL":
    lowered = text.strip().lower()
    if lowered not in ("1", "0", "true", "false", "on", "off", "yes", "no"):
      raise ValueError(f"{key} is a bool: use 1 or 0")
    return key, lowered in ("1", "true", "on", "yes")
  if kind in ("STRING", "INT", "FLOAT"):
    return key, {"STRING": str, "INT": int, "FLOAT": float}[kind](text)
  raise ValueError(f"{key} is a {kind} param, which this tool doesn't set")


def qr_lines(text: str) -> list[str]:
  """A QR code as terminal rows, two modules per row in half blocks, black on white whatever the terminal theme."""
  from openpilot.common.qrcode import _Qr, _capacity
  raw = text.encode()
  version = next(v for v in range(1, 21) if 4 + (8 if v <= 9 else 16) + len(raw) * 8 <= _capacity(v) * 8)
  modules = _Qr(version, raw).modules
  quiet = 4
  side = len(modules) + 2 * quiet
  grid = [[False] * side for _ in range(side)]
  for y, row in enumerate(modules):
    for x, dark in enumerate(row):
      grid[y + quiet][x + quiet] = bool(dark)
  grid.append([False] * side)  # an odd count of rows leaves the last half block blank
  lines = []
  for y in range(0, side, 2):
    cells = [f"\x1b[{30 if top else 37};{40 if bottom else 47}m\u2580" for top, bottom in zip(grid[y], grid[y + 1], strict=True)]
    lines.append("".join(cells) + "\x1b[0m")
  return lines


def collect(params) -> list[Check]:
  """Read the device and run every check."""
  from openpilot.common.git import get_short_branch
  from openpilot.common.hardware.usb import get_usb_state
  from openpilot.selfdrive.modeld.helpers import chestnut_compiled
  from openpilot.selfdrive.turnpilot.detector.graph import detector_pkl_path
  from openpilot.selfdrive.turnpilot.maps.client import MapError, missing_config
  from openpilot.selfdrive.turnpilot.maps.coverage import STATUS_PARAM as COVERAGE_STATUS, CoverageCache
  from openpilot.selfdrive.turnpilot.routing.valhalla_source import ValhallaSource, import_valhalla, locate
  from openpilot.selfdrive.turnpilot.web.pairing import PORT, local_ip

  cp = None
  if cp_bytes := params.get("CarParamsPersistent"):
    from opendbc.car.structs import car
    from openpilot.cereal import messaging
    cp = messaging.log_from_bytes(cp_bytes, car.CarParams)

  devices = get_usb_state()
  chestnut = any(is_chestnut_usb_id(d["vendorId"], d["productId"]) and d["product"] == CHESTNUT_USB_PRODUCT for d in devices)
  enabled, shadow = params.get_bool("TurnPilotEnabled"), params.get_bool("TurnPilotShadowMode")
  mode, tiles = params.get("TurnPilotRouteSource"), (params.get("TurnPilotValhallaTiles") or "").strip()
  needed = bool(tiles)  # the routing test needs local tiles; the map server's corridor only exists during a drive
  try:
    coverage, coverage_error = CoverageCache().get(), None
  except MapError as e:
    coverage, coverage_error = None, e.message
  token = params.get("TurnPilotWebToken")
  valhalla = import_valhalla()
  where = locate(tiles)

  checks = [
    check_branch(get_short_branch(), params.get_bool("IsReleaseBranch")),
    check_car(cp),
    check_alpha_long(params.get_bool("AlphaLongitudinalEnabled"), cp),
    check_toggle("navigation", "NavigationEnabled", params.get_bool("NavigationEnabled"), f"{SETTINGS_DEVELOPER} > navigation (alpha)"),
    check_toggle("TurnPilot", "TurnPilotEnabled", enabled, "preflight --set TurnPilotEnabled=1, then turn the car off and on"),
    check_gas_disengages(params.get_bool("DisengageOnAccelerator")),
    check_shadow(enabled, shadow),
    check_route_source(mode, tiles),
    check_map_server(missing_config(params), coverage.names() if coverage else None, coverage_error, tiles, params.get(COVERAGE_STATUS)),
    check_pyvalhalla(getattr(valhalla, "__version__", "imported") if valhalla else None),
    check_tiles(tiles, where, needed),
  ]
  if needed and valhalla is not None and where is not None:
    try:
      t0 = time.monotonic()
      steps = ValhallaSource(tiles).route(ROUTE_FROM, ROUTE_TO, None, "en")
      checks.append(check_route(None, sum(s["distance"] for s in steps), time.monotonic() - t0))
    except Exception as e:
      checks.append(check_route(f"{type(e).__name__}: {e}", 0., 0.))
  checks += [
    check_big_model(chestnut_compiled()),
    check_detector(detector_pkl_path(*CAR_CAMERA, chestnut), chestnut),
    check_chestnut(devices),
    check_token(token),
    check_pairing_url(token, local_ip(), PORT),
    check_web(listening(PORT), enabled),
  ]
  return checks


def listening(port: int) -> bool:
  try:
    with socket.create_connection(("127.0.0.1", port), timeout=WEB_PORT_TIMEOUT_S):
      return True
  except OSError:
    return False


def render(checks: list[Check]) -> str:
  width = max(len(c.name) for c in checks)
  lines = []
  for c in checks:
    lines.append(f"{c.status}  {c.name:<{width}}  {c.detail}".rstrip())
    if c.fix and c.status in (FAIL, WARN):
      lines.append(f"      {'':<{width}}  fix: {c.fix}")
  count = {s: sum(c.status == s for c in checks) for s in (PASS, FAIL, WARN)}
  lines.append(f"\n{count[PASS]} passed, {count[FAIL]} failed, {count[WARN]} warnings")
  return "\n".join(lines)


def main(argv=None) -> int:
  parser = argparse.ArgumentParser(description="TurnPilot install checklist")
  parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="set a param before checking")
  parser.add_argument("--qr", action="store_true", help="print the pairing URL as a QR code")
  args = parser.parse_args(argv)

  from openpilot.common.params import Params
  params = Params()
  for assignment in args.set:
    try:
      key, value = parse_assignment(params, assignment)
    except ValueError as e:
      print(f"--set {assignment}: {e}", file=sys.stderr)
      return 2
    if isinstance(value, bool):
      params.put_bool(key, value, block=True)
    else:
      params.put(key, value, block=True)
    print(f"set {key} = {value}")

  checks = collect(params)
  print(render(checks))
  url = next((c.detail for c in checks if c.name == "pairing URL" and c.status == PASS), None)
  if args.qr:
    print("\n" + "\n".join(qr_lines(url)) if url else "\nno pairing URL to show")
  return int(any(c.status == FAIL for c in checks))


if __name__ == "__main__":
  sys.exit(main())
