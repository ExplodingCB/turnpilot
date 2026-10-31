"""The map engine (pyvalhalla) installs itself, and the home screen's readiness line, from a thread in turnpilot_web.

navd routes each drive's corridor from the map server with pyvalhalla in its own process (routing/valhalla_source.py),
and the device has no valhalla_service to fall back to, so driving needs it. The system venv is read-only, so it goes
in /data/turnpilot/pyvalhalla, as tools/turnpilot/install_valhalla_device.sh does by hand: the same pinned wheel and
sha256, valhalla/bin dropped, an import test in a fresh interpreter, then an atomic move into place.

The thread never blocks boot, the UI or onroad: it runs inside turnpilot_web at the lowest CPU priority, tries once per
boot and again only when offline (OFFLINE_RETRY_S) or when the driver taps retry (TurnPilotValhallaRetry). Its state is
TurnPilotValhallaInstall: {"state": installing|installed|failed|offline, "progress": 0-1, "stage", "why", "short", "at"}.
"""
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import zipfile
from pathlib import Path

import requests

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.selfdrive.turnpilot.maps.paths import wall_time
from openpilot.selfdrive.turnpilot.routing.valhalla_source import DEVICE_PYVALHALLA
from openpilot.selfdrive.turnpilot.setup_state import ALERT, INSTALL_PARAM, RETRY_PARAM, readiness_line

# the same wheel as tools/turnpilot/install_valhalla_device.sh (test_valhalla_install checks they agree)
WHEEL_NAME = "pyvalhalla-3.9.1-cp312-abi3-manylinux_2_28_aarch64.whl"
WHEEL_URL = ("https://files.pythonhosted.org/packages/e5/3a/087f21f1d3551ed30fed4a8945f631482482183ca9bb69a2fddc9454402a/" + WHEEL_NAME)
WHEEL_SHA256 = "6ebe5f9dd6ce3b16413f939a271d68e64a4addd44df8a802a8eda0b9ca0bf16f"
MIN_FREE_BYTES = 300 * 1000 * 1000
OFFLINE_RETRY_S = 30.  # s between tries while the download can't connect
LOOP_S = 5.  # s between readiness checks
TIMEOUT = (10., 60.)


class InstallError(Exception):
  def __init__(self, short: str, why: str, offline: bool = False):
    super().__init__(why)
    self.short, self.why, self.offline = short, why, offline


def installed(dest: str = DEVICE_PYVALHALLA) -> bool:
  """The install moves into place in one rename, so its package directory there means a whole install."""
  return (Path(dest) / 'valhalla' / '__init__.py').is_file()


def decide(is_installed: bool, status: dict | None, retry: bool, now: float) -> str:
  """'done', 'install' or 'wait'. Installs once per boot (no status yet); after that only on a retry tap, or once
  OFFLINE_RETRY_S has passed since an attempt that couldn't connect. A failure for any other reason (a bad download,
  no space, an import that fails) waits for the tap, since trying again by itself would fail the same way."""
  if is_installed:
    return 'done'
  if retry:
    return 'install'
  state = (status or {}).get('state')
  if state is None:
    return 'install'
  if state == 'offline':
    return 'install' if now - float(status.get('at') or 0.) >= OFFLINE_RETRY_S else 'wait'
  return 'wait'  # installing (another try in flight), or failed


def download(url: str, out: Path, progress) -> None:
  try:
    with requests.get(url, stream=True, timeout=TIMEOUT) as resp:
      if resp.status_code != 200:
        raise InstallError("download failed", f"the download answered HTTP {resp.status_code}", offline=resp.status_code >= 500)
      total = int(resp.headers.get('content-length') or 0)
      got = 0
      with open(out, 'wb') as f:
        for chunk in resp.iter_content(256 * 1024):
          f.write(chunk)
          got += len(chunk)
          progress(got / total if total else None)
  except (requests.exceptions.ConnectionError, requests.exceptions.Timeout) as e:
    raise InstallError("no internet", f"can't download the map engine ({type(e).__name__})", offline=True) from None


def sha256(path: Path) -> str:
  h = hashlib.sha256()
  with open(path, 'rb') as f:
    for block in iter(lambda: f.read(1 << 20), b''):
      h.update(block)
  return h.hexdigest()


def import_ok(path: Path, python: str = sys.executable) -> str:
  """'' if valhalla imports from path in a fresh interpreter, else why not."""
  try:
    res = subprocess.run([python, '-c', 'import valhalla'], env={**os.environ, 'PYTHONPATH': str(path)}, capture_output=True, text=True,
                         timeout=120)
  except (OSError, subprocess.TimeoutExpired) as e:
    return f"{type(e).__name__}: {e}"
  return '' if res.returncode == 0 else (res.stderr.strip().splitlines() or ['import failed'])[-1]


def install(dest: str = DEVICE_PYVALHALLA, url: str = WHEEL_URL, digest: str = WHEEL_SHA256, progress=lambda f, stage: None,
            check_import=import_ok) -> None:
  """install_valhalla_device.sh in Python. Raises InstallError with words for the driver."""
  target = Path(dest)
  target.parent.mkdir(parents=True, exist_ok=True)
  for stale in target.parent.glob('.pyvalhalla-install.*'):  # an attempt killed part way (power off)
    shutil.rmtree(stale, ignore_errors=True)
  if shutil.disk_usage(target.parent).free < MIN_FREE_BYTES:
    raise InstallError("storage full", f"less than 300 MB free in {target.parent}: delete old drives")
  work = Path(tempfile.mkdtemp(prefix='.pyvalhalla-install.', dir=target.parent))
  try:
    wheel = work / WHEEL_NAME
    download(url, wheel, lambda f: progress(f, "downloading"))
    progress(1., "checking")
    got = sha256(wheel)
    if got != digest:
      raise InstallError("bad download", f"the download's sha256 is {got}, not {digest}")
    progress(1., "unpacking")
    unpacked = work / 'pyvalhalla'
    try:
      with zipfile.ZipFile(wheel) as z:
        z.extractall(unpacked)
    except zipfile.BadZipFile as e:
      raise InstallError("bad download", f"the wheel doesn't unzip ({e})") from None
    shutil.rmtree(unpacked / 'valhalla' / 'bin', ignore_errors=True)
    wheel.unlink()
    progress(1., "testing")
    why = check_import(unpacked)
    if why:
      raise InstallError("doesn't run here", f"the map engine doesn't import on this comma: {why}")
    old = target.with_name(target.name + '.old')
    shutil.rmtree(old, ignore_errors=True)
    if target.exists():
      target.rename(old)
    unpacked.rename(target)
    shutil.rmtree(old, ignore_errors=True)
  except OSError as e:
    raise InstallError("can't save it", f"{type(e).__name__}: {e}") from None
  finally:
    shutil.rmtree(work, ignore_errors=True)


class SetupThread(threading.Thread):
  """Installs the map engine when it is missing, and keeps Offroad_TurnPilotSetup saying what TurnPilot still needs."""

  def __init__(self, params, dest: str = DEVICE_PYVALHALLA, installer=install, clock=wall_time, alert=set_offroad_alert):
    super().__init__(name='valhalla_install', daemon=True)
    self.params, self.dest, self.installer, self.clock, self.alert = params, dest, installer, clock, alert
    self.stopped = threading.Event()
    self.shown: str | None = None

  def stop(self):
    self.stopped.set()

  def set_status(self, state: str, **extra):
    self.params.put(INSTALL_PARAM, {'state': state, **extra, 'at': self.clock()})

  def attempt(self):
    last = [0.]

    def progress(fraction, stage):
      now = time.monotonic()
      if now - last[0] > 1. or stage != 'downloading':  # one param write a second at most
        last[0] = now
        self.set_status('installing', progress=fraction, stage=stage)

    self.set_status('installing', progress=0., stage='starting')
    try:
      self.installer(self.dest, progress=progress)
    except InstallError as e:
      cloudlog.warning(f"turnpilot map engine install failed: {e.why}")
      self.set_status('offline' if e.offline else 'failed', why=e.why, short=e.short)
      return
    except Exception as e:
      cloudlog.exception("turnpilot map engine install failed")
      self.set_status('failed', why=f"{type(e).__name__}: {e}", short="install failed")
      return
    self.set_status('installed')
    cloudlog.info(f"turnpilot map engine installed in {self.dest}")

  def step(self):
    """One pass: install if decide says so, then the readiness alert."""
    enabled = self.params.get_bool('TurnPilotEnabled')
    if enabled:
      retry = self.params.get_bool(RETRY_PARAM)
      if retry:
        self.params.remove(RETRY_PARAM)
      if decide(installed(self.dest), self.params.get(INSTALL_PARAM), retry, self.clock()) == 'install':
        self.attempt()
    line = readiness_line(self.params, self.clock(), installed(self.dest))
    if line != self.shown:
      self.alert(ALERT, bool(line), line)
      self.shown = line

  def run(self):
    try:
      os.nice(19)  # the lowest priority: the unzip never takes CPU from the drive
    except OSError:
      pass
    while not self.stopped.is_set():
      try:
        self.step()
      except Exception:
        cloudlog.exception("turnpilot setup thread failed")
      self.stopped.wait(LOOP_S)
