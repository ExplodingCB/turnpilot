"""TurnPilot's guided setup (setup_state.py), the map engine's self-install (valhalla_install.py) and the shipped map server
(maps/defaults.py)."""
import base64
import hashlib
import io
import re
import tempfile
import zipfile
from pathlib import Path

import pytest

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.turnpilot import setup_state as ss
from openpilot.selfdrive.turnpilot import valhalla_install as vi
from openpilot.selfdrive.turnpilot.maps.client import cert_fingerprint, check_cert, normalize_host
from openpilot.selfdrive.turnpilot.maps.coverage import STATUS_PARAM
from openpilot.selfdrive.turnpilot.maps.defaults import DEFAULT_MAP_CERT, DEFAULT_MAP_CERT_SHA256, DEFAULT_MAP_HOST, ensure_map_defaults

NOW = 1_800_000_000.


class BlockingParams(Params):
  """Params whose writes land before the next read, as the readers in these tests need."""
  def put(self, key, dat, block=True):
    super().put(key, dat, block=True)

  def put_bool(self, key, val, block=True):
    super().put_bool(key, val, block=True)
REGIONS = [{'id': 'indianapolis', 'name': 'Indianapolis area'}, {'id': 'seattle', 'name': 'Seattle area'}]


class SetupParams(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.params = BlockingParams()
    self.params.put_bool('TurnPilotEnabled', True)

  def ready(self):
    """Every step good: the phone seen, the key answered, setup finished."""
    self.params.put(ss.SETUP_STEP, 4)
    self.params.put(ss.PHONE_SEEN, NOW - 5)
    self.params.put('TurnPilotMapKey', 'k' * 32)
    self.params.put(STATUS_PARAM, {'ok': True, 'version': 'v1', 'regions': REGIONS, 'at': NOW})
    self.params.put_bool(ss.SETUP_DONE, True)


class TestSteps(SetupParams):
  def test_fresh_install_starts_at_the_intro(self):
    assert ss.first_unfinished(self.params, NOW, installed=False) == 0

  def test_resumes_at_the_first_unfinished_step(self):
    self.params.put(ss.SETUP_STEP, 1)  # moved past the intro, then rebooted
    assert ss.STEPS[ss.first_unfinished(self.params, NOW, False)] == 'phone'
    self.params.put(ss.PHONE_SEEN, NOW - 3600)  # paired an hour ago: still paired
    assert ss.STEPS[ss.first_unfinished(self.params, NOW, False)] == 'key'
    self.params.put('TurnPilotMapKey', 'k' * 32)
    self.params.put(STATUS_PARAM, {'ok': True, 'regions': REGIONS, 'at': NOW})
    assert ss.STEPS[ss.first_unfinished(self.params, NOW, False)] == 'engine'
    assert ss.STEPS[ss.first_unfinished(self.params, NOW, True)] == 'done'

  def test_phone_connected_live_then_paired(self):
    assert ss.phone_state(self.params, NOW).state == ss.WAITING
    self.params.put(ss.PHONE_SEEN, NOW - 2)
    assert ss.phone_state(self.params, NOW).title == "phone connected"
    assert ss.phone_state(self.params, NOW + ss.PHONE_LIVE_S).title == "phone paired"

  def test_key_states_say_what_to_do(self):
    assert ss.key_state(self.params).title == "waiting for the key"
    self.params.put('TurnPilotMapKey', 'k' * 32)
    assert ss.key_state(self.params).state == ss.BUSY  # the refresh hasn't answered yet
    self.params.put(STATUS_PARAM, {'ok': False, 'code': 'unauthorized', 'error': 'refused', 'at': NOW})
    key = ss.key_state(self.params)
    assert key.state == ss.ERROR and "paste the key again" in key.detail
    self.params.put(STATUS_PARAM, {'ok': False, 'code': 'unreachable', 'error': 'x', 'at': NOW})
    assert "internet" in ss.key_state(self.params).detail
    self.params.put(STATUS_PARAM, {'ok': True, 'regions': REGIONS, 'at': NOW})
    key = ss.key_state(self.params)
    assert key.state == ss.GOOD and key.detail == "maps: Indianapolis and Seattle"

  def test_engine_states(self):
    assert ss.engine_state(self.params, True).state == ss.GOOD
    self.params.put(ss.INSTALL_PARAM, {'state': 'installing', 'progress': 0.42, 'stage': 'downloading', 'at': NOW})
    assert ss.engine_state(self.params, False).title == "installing 42%"
    self.params.put(ss.INSTALL_PARAM, {'state': 'failed', 'short': 'storage full', 'why': '...', 'at': NOW})
    assert ss.engine_state(self.params, False) == ss.StepState(ss.ERROR, "install failed", "storage full")
    self.params.put(ss.INSTALL_PARAM, {'state': 'offline', 'at': NOW})
    assert ss.engine_state(self.params, False).title == "waiting for internet"


class TestReadiness(SetupParams):
  def test_nothing_when_ready(self):
    self.ready()
    assert ss.readiness_line(self.params, NOW, installed=True) == ''

  def test_nothing_when_turnpilot_is_off_or_on_local_tiles(self):
    self.params.put_bool('TurnPilotEnabled', False)
    assert ss.readiness_line(self.params, NOW, False) == ''
    self.params.put_bool('TurnPilotEnabled', True)
    self.params.put('TurnPilotValhallaTiles', '/data/tiles')
    assert ss.readiness_line(self.params, NOW, False) == ''

  def test_one_line_for_the_first_thing_missing(self):
    assert ss.readiness_line(self.params, NOW, False) == "Paste the map key on the TurnPilot phone page."
    self.ready()
    self.params.put(STATUS_PARAM, {'ok': False, 'code': 'unreachable', 'error': 'x', 'at': NOW})
    assert "Connect the comma to the internet" in ss.readiness_line(self.params, NOW, True)
    self.params.put(STATUS_PARAM, {'ok': True, 'regions': REGIONS, 'at': NOW})
    assert ss.readiness_line(self.params, NOW, False) == "The map engine is still installing. Keep the comma online."
    self.params.put(ss.INSTALL_PARAM, {'state': 'failed', 'short': 'bad download', 'at': NOW})
    assert "tap retry" in ss.readiness_line(self.params, NOW, False)
    self.params.put_bool(ss.SETUP_DONE, False)
    assert ss.readiness_line(self.params, NOW, True) == "Finish setup: open settings, turnpilot, run setup again."

  def test_the_alert_exists_with_its_extra_text(self):
    from openpilot.selfdrive.selfdrived.alertmanager import OFFROAD_ALERTS
    assert '%1' in OFFROAD_ALERTS[ss.ALERT]['text']


class TestDecide:
  def test_installed_is_done(self):
    assert vi.decide(True, None, retry=True, now=NOW) == 'done'

  def test_once_per_boot(self):
    assert vi.decide(False, None, False, NOW) == 'install'  # no status since this boot (CLEAR_ON_MANAGER_START)
    assert vi.decide(False, {'state': 'installing', 'at': NOW}, False, NOW + 600) == 'wait'

  def test_offline_retries_by_itself(self):
    status = {'state': 'offline', 'at': NOW}
    assert vi.decide(False, status, False, NOW + vi.OFFLINE_RETRY_S - 1) == 'wait'
    assert vi.decide(False, status, False, NOW + vi.OFFLINE_RETRY_S) == 'install'

  def test_a_failure_waits_for_the_retry_tap(self):
    status = {'state': 'failed', 'at': NOW}
    assert vi.decide(False, status, False, NOW + 86400) == 'wait'
    assert vi.decide(False, status, True, NOW + 1) == 'install'


def fake_wheel() -> bytes:
  buf = io.BytesIO()
  with zipfile.ZipFile(buf, 'w') as z:
    z.writestr('valhalla/__init__.py', '__version__ = "3.9.1"\n')
    z.writestr('valhalla/bin/valhalla_service', 'binary')
    z.writestr('pyvalhalla.libs/libvalhalla.so', 'lib')
  return buf.getvalue()


class FakeResponse:
  def __init__(self, body: bytes, status: int = 200):
    self.body, self.status_code, self.headers = body, status, {'content-length': str(len(body))}

  def __enter__(self):
    return self

  def __exit__(self, *a):
    return False

  def iter_content(self, size):
    for i in range(0, len(self.body), 7):
      yield self.body[i:i + 7]


class TestInstall:
  def run_install(self, tmp_path, monkeypatch, body=None, digest=None, check_import=lambda p: ''):
    body = fake_wheel() if body is None else body
    monkeypatch.setattr(vi.requests, 'get', lambda url, **kw: FakeResponse(body))
    stages = []
    dest = tmp_path / 'turnpilot' / 'pyvalhalla'
    vi.install(str(dest), digest=digest or hashlib.sha256(body).hexdigest(), progress=lambda f, s: stages.append(s), check_import=check_import)
    return dest, stages

  def test_installs_like_the_script(self, tmp_path, monkeypatch):
    (tmp_path / 'turnpilot').mkdir()
    (tmp_path / 'turnpilot' / '.pyvalhalla-install.dead').mkdir()  # a try killed part way
    dest, stages = self.run_install(tmp_path, monkeypatch)
    assert vi.installed(str(dest))
    assert (dest / 'pyvalhalla.libs').is_dir() and not (dest / 'valhalla' / 'bin').exists()
    assert list((tmp_path / 'turnpilot').iterdir()) == [dest]  # no work or stale directories left
    assert stages[0] == 'downloading' and stages[-1] == 'testing'

  def test_sha256_mismatch_installs_nothing(self, tmp_path, monkeypatch):
    with pytest.raises(vi.InstallError) as e:
      self.run_install(tmp_path, monkeypatch, digest='0' * 64)
    assert e.value.short == "bad download" and not e.value.offline
    assert not vi.installed(str(tmp_path / 'turnpilot' / 'pyvalhalla'))

  def test_import_failure_installs_nothing(self, tmp_path, monkeypatch):
    with pytest.raises(vi.InstallError) as e:
      self.run_install(tmp_path, monkeypatch, check_import=lambda p: 'GLIBC_2.34 not found')
    assert "GLIBC" in e.value.why and not vi.installed(str(tmp_path / 'turnpilot' / 'pyvalhalla'))

  def test_offline_is_a_retry(self, tmp_path, monkeypatch):
    def offline(url, **kw):
      raise vi.requests.exceptions.ConnectionError("no route")
    monkeypatch.setattr(vi.requests, 'get', offline)
    with pytest.raises(vi.InstallError) as e:
      vi.install(str(tmp_path / 'pyvalhalla'), check_import=lambda p: '')
    assert e.value.offline

  def test_same_wheel_as_the_script(self):
    script = (Path(BASEDIR) / 'tools/turnpilot/install_valhalla_device.sh').read_text()
    assert f'WHEEL_NAME="{vi.WHEEL_NAME}"' in script
    assert f'WHEEL_SHA256="{vi.WHEEL_SHA256}"' in script
    assert vi.WHEEL_URL == re.search(r'WHEEL_URL="(.*)\$WHEEL_NAME"', script).group(1) + vi.WHEEL_NAME


class TestSetupThread(SetupParams):
  def setUp(self):
    super().setUp()
    self._tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self._tmp.cleanup)

  def thread(self, installer):
    tmp_path = Path(self._tmp.name)
    alerts = []
    t = vi.SetupThread(self.params, str(tmp_path / 'pyvalhalla'), installer=installer, clock=lambda: NOW,
                       alert=lambda key, show, extra: alerts.append((key, show, extra)))
    return t, alerts

  def test_installs_once_and_sets_the_alert(self):
    calls = []

    def installer(dest, progress):
      calls.append(dest)
      progress(0.5, 'downloading')
      (Path(dest) / 'valhalla').mkdir(parents=True)
      (Path(dest) / 'valhalla' / '__init__.py').write_text('')

    t, alerts = self.thread(installer)
    t.step()
    t.step()
    assert len(calls) == 1 and self.params.get(ss.INSTALL_PARAM)['state'] == 'installed'
    assert alerts == [(ss.ALERT, True, "Paste the map key on the TurnPilot phone page.")]  # said once, not every pass
    self.ready()
    t.step()
    assert alerts[-1] == (ss.ALERT, False, '')

  def test_failure_then_retry_tap(self):
    tries = []

    def installer(dest, progress):
      tries.append(1)
      raise vi.InstallError("storage full", "less than 300 MB free")

    t, _ = self.thread(installer)
    t.step()
    t.step()
    assert len(tries) == 1 and self.params.get(ss.INSTALL_PARAM)['short'] == "storage full"
    self.params.put_bool(ss.RETRY_PARAM, True)
    t.step()
    assert len(tries) == 2 and not self.params.get_bool(ss.RETRY_PARAM)

  def test_turnpilot_off_installs_nothing(self):
    self.params.put_bool('TurnPilotEnabled', False)
    t, alerts = self.thread(lambda dest, progress: pytest.fail("installed with TurnPilot off"))
    t.step()
    assert alerts == [(ss.ALERT, False, '')]


class TestMapDefaults(OpenpilotTestCase):
  def test_shipped_server(self):
    assert normalize_host(DEFAULT_MAP_HOST) == DEFAULT_MAP_HOST == 'https://136.113.85.41'
    assert check_cert(DEFAULT_MAP_CERT) == DEFAULT_MAP_CERT
    assert cert_fingerprint(DEFAULT_MAP_CERT) == DEFAULT_MAP_CERT_SHA256
    assert 'PRIVATE' not in DEFAULT_MAP_CERT
    der = base64.b64decode(''.join(line for line in DEFAULT_MAP_CERT.splitlines() if not line.startswith('-----')))
    assert bytes([136, 113, 85, 41]) in der  # the server's IP, in the certificate's SAN

  def test_written_only_when_empty(self):
    params = BlockingParams()
    assert ensure_map_defaults(params) == ['TurnPilotMapHost', 'TurnPilotMapCert']
    assert params.get('TurnPilotMapHost') == DEFAULT_MAP_HOST and params.get('TurnPilotMapKey') is None
    params.put('TurnPilotMapHost', 'https://maps.example.test')
    assert ensure_map_defaults(params) == []
    assert params.get('TurnPilotMapHost') == 'https://maps.example.test'
