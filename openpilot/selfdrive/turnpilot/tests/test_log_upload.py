"""TurnPilot's drive logs to our map server (log_upload.py): what goes first, what waits for an unmetered connection, and a real
upload over TLS with the pinned certificate into the server's log store."""
import os
import threading

import pytest

from openpilot.selfdrive.turnpilot import log_upload
from openpilot.selfdrive.turnpilot.maps.client import MapClient
from openpilot.selfdrive.turnpilot.maps.tests.fakes import self_signed

KEY = 'k' * 40
DONGLE = '0123456789abcdef'


@pytest.fixture
def marks(monkeypatch):
  """xattrs as a dict: the device's are Linux's."""
  done: dict[str, bytes] = {}
  monkeypatch.setattr(log_upload, 'getxattr', lambda fn, attr: done.get(fn) if os.path.exists(fn) else (_ for _ in ()).throw(OSError()))
  monkeypatch.setattr(log_upload, 'setxattr', lambda fn, attr, v: done.__setitem__(fn, v))
  return done


def make_segments(root, n=3, lock_last=True):
  for i in range(n):
    d = root / f"00000011--24ea9deaa5--{i}"
    d.mkdir()
    for name in (log_upload.QLOG, log_upload.RLOG):
      (d / name).write_bytes(f"{name} {i}".encode() * 100)
    os.utime(d, (1000 + i, 1000 + i))
  if lock_last:
    (root / f"00000011--24ea9deaa5--{n - 1}" / 'rlog.lock').write_bytes(b'')


def test_qlogs_first_newest_first_rlogs_only_off_metered_and_never_a_segment_being_written(tmp_path, marks):
  make_segments(tmp_path)
  root = str(tmp_path)
  assert log_upload.segments(root) == ['00000011--24ea9deaa5--1', '00000011--24ea9deaa5--0']  # --2 is still being written
  sent = []
  while (nxt := log_upload.next_file(root, metered=True)) is not None:
    sent.append(nxt)
    marks[os.path.join(root, *nxt)] = log_upload.DONE
  assert sent == [('00000011--24ea9deaa5--1', 'qlog.zst'), ('00000011--24ea9deaa5--0', 'qlog.zst')]  # no rlog on a hotspot
  assert log_upload.next_file(root, metered=False) == ('00000011--24ea9deaa5--1', 'rlog.zst')


def test_a_segment_reaches_the_map_servers_log_store_over_the_pinned_tls(tmp_path, marks):
  from tools.turnpilot.mapserver.logstore import LogStore
  from tools.turnpilot.mapserver.server import Maps, make_server
  cert, key = self_signed()
  (tmp_path / 'cert.pem').write_text(cert)
  (tmp_path / 'cert.key').write_text(key)
  store = LogStore(tmp_path / 'store', 10 ** 8)
  httpd = make_server(Maps([], KEY, logs=store), '127.0.0.1', 0, tmp_path / 'cert.pem', tmp_path / 'cert.key')
  threading.Thread(target=httpd.serve_forever, daemon=True).start()
  try:
    logs = tmp_path / 'logs'
    logs.mkdir()
    make_segments(logs, n=1, lock_last=False)
    client = MapClient(f'https://127.0.0.1:{httpd.server_address[1]}', KEY, cert, tmp_path)
    seg = '00000011--24ea9deaa5--0'
    assert log_upload.upload(client, DONGLE, str(logs), seg, 'rlog.zst')
    assert store.path(DONGLE, seg, 'rlog.zst').read_bytes() == (logs / seg / 'rlog.zst').read_bytes()
    assert log_upload.upload(client, DONGLE, str(logs), seg, 'rlog.zst')  # a retry is accepted, not stored twice
  finally:
    httpd.shutdown()
