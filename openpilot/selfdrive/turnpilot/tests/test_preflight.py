"""preflight's checks with plain values, and a scan that nothing on the car path imports what the comma lacks."""
import ast
import re
import sys
import tomllib
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT
from openpilot.selfdrive.turnpilot import preflight
from openpilot.selfdrive.turnpilot.preflight import FAIL, INFO, PASS, WARN, Check

PACKAGE = Path(__file__).parents[1]
REPO = Path(__file__).parents[4]


def cp(brand="tesla", fingerprint="TESLA_MODEL_3", dashcam=False, alpha_available=True, alpha_active=True):
  return SimpleNamespace(brand=brand, carFingerprint=fingerprint, dashcamOnly=dashcam,
                         alphaLongitudinalAvailable=alpha_available, openpilotLongitudinalControl=alpha_active)


def chestnut(product=CHESTNUT_USB_PRODUCT, speed=10000, vendor=0xADD1):
  return {"vendorId": vendor, "productId": 0x0001, "product": product, "speedMbps": speed}


def test_branch():
  assert preflight.check_branch("turnpilot", False).status == PASS
  assert preflight.check_branch("tp/carbuild", False).status == PASS
  for release in ("release-mici", "nightly-chestnut"):
    assert preflight.check_branch(release, False).status == FAIL
  assert preflight.check_branch("turnpilot", True).status == FAIL  # the manager still has the old branch


def test_car():
  assert preflight.check_car(None).status == FAIL
  assert preflight.check_car(cp(dashcam=True)).status == FAIL
  ok = preflight.check_car(cp())
  assert ok.status == PASS and "TESLA_MODEL_3" in ok.detail


def test_alpha_long():
  assert preflight.check_alpha_long(True, None).status == PASS  # the car hasn't been seen, the toggle can still be on
  assert preflight.check_alpha_long(True, cp()).status == PASS
  assert preflight.check_alpha_long(False, cp()).status == FAIL
  assert preflight.check_alpha_long(True, cp(alpha_active=False)).status == FAIL  # needs a restart
  assert preflight.check_alpha_long(True, cp(alpha_available=False)).status == FAIL


def test_shadow():
  assert preflight.check_shadow(False, False).status == INFO
  assert preflight.check_shadow(True, True).status == INFO
  live = preflight.check_shadow(True, False)
  assert live.status == WARN and "TurnPilotShadowMode=1" in live.fix


def test_pyvalhalla_is_always_needed():
  assert preflight.check_pyvalhalla(None).status == FAIL
  assert preflight.check_pyvalhalla("3.9.1").status == PASS
  assert preflight.check_tiles(None, None, False).status == INFO  # no override: each drive downloads its tiles


def test_route_source_has_no_mapbox():
  assert preflight.check_route_source("mapbox", None).status == FAIL
  assert preflight.check_route_source(None, None).status == INFO
  assert preflight.check_route_source("auto", "/data/tiles").status == WARN  # the sim override, not for the car


def test_map_server():
  assert preflight.check_map_server(["API key"], None, None, None).status == FAIL
  assert preflight.check_map_server([], None, None, None).status == FAIL  # never fetched
  assert preflight.check_map_server([], None, "unreadable", None).status == FAIL
  ok = preflight.check_map_server([], "Indianapolis and Seattle", None, None)
  assert ok.status == PASS and "Seattle" in ok.detail
  assert preflight.check_map_server(["address"], None, None, "/data/tiles").status == INFO
  # the cached area is there, but the last refresh failed (a rotated key, a new certificate, the server down)
  failed = {"ok": False, "code": "unauthorized", "error": "The map server refused the API key."}
  check = preflight.check_map_server([], "Indianapolis and Seattle", None, None, failed)
  assert check.status == FAIL and "refused the API key" in check.detail
  assert preflight.check_map_server([], "Indianapolis and Seattle", None, None, {"ok": True}).status == PASS


def test_tiles(tmp_path):
  assert preflight.check_tiles(None, None, True).status == FAIL
  assert preflight.check_tiles("/nope", None, True).status == FAIL
  extract = tmp_path / "tiles.tar"
  extract.write_bytes(b"\0" * 2_000_000)
  ok = preflight.check_tiles(str(tmp_path), {"extract": str(extract), "dir": ""}, True)
  assert ok.status == PASS and "2 MB" in ok.detail
  assert preflight.check_tiles("http://127.0.0.1:8002", {"url": "http://127.0.0.1:8002"}, True).status == PASS


def test_route():
  assert preflight.check_route(None, 2100., 0.04).status == PASS
  assert preflight.check_route(None, 0., 0.04).status == FAIL  # no route
  assert preflight.check_route(None, 90_000., 0.04).status == FAIL  # tiles for somewhere else
  assert preflight.check_route("RuntimeError: boom", 0., 0.).status == FAIL


def test_detector(tmp_path):
  path = tmp_path / "big_detector_1344x760_tinygrad.pkl"
  assert preflight.check_detector(path, True).status == FAIL
  path.write_bytes(b"version https://git-lfs.github.com/spec/v1\n")
  assert preflight.check_detector(path, True).status == FAIL
  path.write_bytes(b"\0" * 5000)
  assert preflight.check_detector(path, True).status == PASS
  missing = preflight.check_detector(tmp_path / "detector_1344x760_tinygrad.pkl", False)
  assert missing.status == FAIL and "no chestnut" in missing.detail


def test_far_crop(tmp_path):
  # DESIGN phase 4 G5: the far crop (and its measured range) are what a 45-55 mph left needs; without either the car drives,
  # those lefts skipped and rerouted, so a warning, never a failure
  path = tmp_path / "big_far_detector_1344x760_tinygrad.pkl"
  missing = preflight.check_far_crop(path, 300., True)
  assert missing.status == WARN and "missing" in missing.detail
  path.write_bytes(b"version https://git-lfs.github.com/spec/v1\n")
  assert preflight.check_far_crop(path, 300., True).status == WARN
  path.write_bytes(b"\0" * 5000)
  unmeasured = preflight.check_far_crop(path, 0., True)
  assert unmeasured.status == WARN and "TurnPilotLeftFarRangeM" in unmeasured.detail
  off = preflight.check_far_crop(path, 300., False)
  assert off.status == WARN and "TurnPilotFarCrop" in off.detail
  ok = preflight.check_far_crop(path, 300., True)
  assert ok.status == PASS and "300 m" in ok.detail


def test_big_model():
  assert preflight.check_big_model(True).status == PASS
  assert preflight.check_big_model(False).status == FAIL


def test_chestnut():
  assert preflight.check_chestnut([]).status == FAIL
  assert preflight.check_chestnut([chestnut()]).status == PASS
  assert preflight.check_chestnut([chestnut(product="custom 00000000-CLEAN")]).status == FAIL  # old firmware
  assert preflight.check_chestnut([chestnut(speed=480)]).status == FAIL  # a USB 2 link
  assert preflight.check_chestnut([{"vendorId": 0x174C, "productId": 0x2464, "product": "", "speedMbps": 5000}]).status == FAIL  # bootloader
  assert preflight.check_chestnut([{"vendorId": 0x05AC, "productId": 0x1234, "product": "keyboard", "speedMbps": 12}]).status == FAIL


def test_web():
  assert preflight.check_token(None).status == FAIL
  assert preflight.check_token("abc").status == PASS
  assert preflight.check_pairing_url(None, "172.20.10.2", 8088).status == FAIL
  assert preflight.check_pairing_url("abc", "127.0.0.1", 8088).status == FAIL
  url = preflight.check_pairing_url("abc", "172.20.10.2", 8088)
  assert url.status == PASS and url.detail == "http://172.20.10.2:8088/?t=abc"
  assert preflight.check_web(True, True).status == PASS
  assert preflight.check_web(False, True).status == FAIL
  assert preflight.check_web(False, False).status == INFO


class FakeParams:
  Kind = type("Kind", (), {"__init__": lambda self, name: setattr(self, "name", name)})

  def __init__(self):
    self.kinds = {"TurnPilotEnabled": "BOOL", "TurnPilotValhallaTiles": "STRING", "TurnpilotCount": "INT", "NavDestination": "JSON"}

  def get_type(self, key):
    if key not in self.kinds:
      raise KeyError(key)
    return self.Kind(self.kinds[key])


def test_parse_assignment():
  params = FakeParams()
  assert preflight.parse_assignment(params, "TurnPilotEnabled=1") == ("TurnPilotEnabled", True)
  assert preflight.parse_assignment(params, "TurnPilotEnabled=off") == ("TurnPilotEnabled", False)
  assert preflight.parse_assignment(params, "TurnPilotValhallaTiles=/data/x=y") == ("TurnPilotValhallaTiles", "/data/x=y")
  assert preflight.parse_assignment(params, "TurnpilotCount=3") == ("TurnpilotCount", 3)
  for bad in ("TurnPilotEnabled", "TurnPilotEnabled=maybe", "NavDestination={}", "Unknown=1", "TurnpilotCount=x"):
    with pytest.raises(ValueError):
      preflight.parse_assignment(params, bad)


def test_render():
  checks = [Check("a", PASS, "fine"), Check("b", FAIL, "broken", "do this"), Check("c", WARN, "careful", "and this"), Check("d", INFO, "note", "hidden")]
  text = preflight.render(checks)
  assert "fix: do this" in text and "fix: and this" in text and "hidden" not in text.replace("note", "")
  assert text.endswith("1 passed, 1 failed, 1 warnings")


def test_qr_scans():
  """The terminal QR code decodes to the URL: rebuild the module grid from the escape codes and run openpilot's decoder."""
  from openpilot.common.qrcode import decode_matrix
  url = "http://172.20.10.2:8088/?t=Zk3dQ9vE1xLmN2pR8sT5uA"
  rows = []
  for line in preflight.qr_lines(url):
    cells = re.findall(r"\x1b\[(30|37);(40|47)m▀", line)
    rows.append([fg == "30" for fg, _ in cells])
    rows.append([bg == "40" for _, bg in cells])
  grid = np.array(rows, dtype=bool)
  quiet = 4
  assert not grid[:quiet].any() and not grid[:, :quiet].any()
  modules = grid[quiet:, quiet:]
  size = next(n for n in range(21, 100, 4) if modules[n:].sum() == 0 and not modules[:, n:].any())  # the blank rows and columns after the code
  assert decode_matrix(modules[:size, :size]) == url


class FakeSettings:
  """Params with nothing set, for a run of collect() on whatever machine the tests are on."""
  def get(self, key):
    return None

  def get_bool(self, key):
    return False


def test_collect_runs():
  checks = preflight.collect(FakeSettings())
  names = [c.name for c in checks]
  assert len(names) == len(set(names))
  assert {"non-release branch", "car", "alpha longitudinal", "valhalla tiles", "detector pkl", "far crop", "chestnut", "pairing URL"} <= set(names)
  assert all(c.status in (PASS, FAIL, WARN, INFO) for c in checks)
  assert preflight.render(checks)


# Everything the car path imports has to exist on the comma: the stdlib, the dependencies in pyproject.toml, and the submodules.
# These files are Mac tools (compiling the detector, exporting YOLO26, replaying an rlog) and never run on the device.
DEV_TOOLS = {"detector/compile_detector.py", "detector/replay_route.py", "models/export_yolo26.py"}
SUBMODULE_PACKAGES = {"opendbc", "msgq", "panda", "rednose", "teleoprtc", "tinygrad", "tinygrad_repo"}


def imports(path: Path):
  """(module, line) for every import in the file, including the ones inside functions."""
  for node in ast.walk(ast.parse(path.read_text())):
    if isinstance(node, ast.Import):
      yield from ((a.name, node.lineno) for a in node.names)
    elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
      yield node.module, node.lineno


def test_car_path_imports():
  dependencies = {re.split(r"[<>=\[ ;~!]", d)[0].lower() for d in tomllib.loads((REPO / "pyproject.toml").read_text())["project"]["dependencies"]}
  allowed = {"numpy", "requests"}
  assert allowed <= dependencies  # the third-party modules the car path may use are core dependencies
  allowed |= SUBMODULE_PACKAGES

  car_path = [p for p in sorted(PACKAGE.rglob("*.py")) if "tests" not in p.relative_to(PACKAGE).parts and p.relative_to(PACKAGE).as_posix() not in DEV_TOOLS]
  assert len(car_path) > 20
  dev_modules = {"openpilot.selfdrive.turnpilot." + t[:-3].replace("/", ".") for t in DEV_TOOLS}
  problems = []
  for path in car_path:
    for module, line in imports(path):
      top = module.split(".")[0]
      where = f"{path.relative_to(PACKAGE)}:{line} imports {module}"
      if module.startswith("openpilot.tools") or module in dev_modules:
        problems.append(where)
      elif top != "openpilot" and top not in sys.stdlib_module_names and top not in allowed:
        problems.append(where)
  assert not problems, "\n".join(problems)
