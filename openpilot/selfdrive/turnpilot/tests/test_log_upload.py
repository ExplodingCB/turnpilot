"""TurnPilot's drive logs to our map server (log_upload.py): what goes first, what waits for an unmetered connection, and a real
upload over TLS with the pinned certificate into the server's log store.

2026-11-06: each segment's qcamera.ts goes too, after every qlog and before any rlog, on a hotspot as well; and a file the
server refuses (it doesn't know qcamera.ts until it is deployed) is held while the rest go up, not retried forever ahead of them.

2026-11-06, the review: Chase's hotspot reads as unmetered wifi, so the videos stream ahead of the drive's rlogs while it
drives. Only a video of a segment closed within 48 h goes, so the ~330 MB of old ones on the car never stand ahead of tonight's
rlogs. A file whose tries keep failing (the server's refusal of an unread 2.4 MB body can be lost to the reset under it) is
set aside after three, so the next file goes. A refused qlog, or a dongle the server can't take, is the server refusing this
car, not one file: no burst of every file on the car. And when every video fails, as when every answer is lost, the
videos stand aside together, or the rlogs never got a turn.

2026-11-06, the review of that: comma's deleter can take a segment while its file goes up from the open file. The server
holds it, the mark finds no file, and that FileNotFoundError killed the uploader for the boot. A file gone when its try ends
is the deleter's (nothing counted, the next goes now); a mark that fails with the file there is a failed try, not a loop."""
import datetime
import errno
import os
import shutil
import threading

import pytest

from openpilot.selfdrive.turnpilot import log_upload
from openpilot.selfdrive.turnpilot.maps.client import MapClient, MapError
from openpilot.selfdrive.turnpilot.maps.paths import wall_time
from openpilot.selfdrive.turnpilot.maps.tests.fakes import self_signed

KEY = 'k' * 40
DONGLE = '0123456789abcdef'
ROUTE = '00000011--24ea9deaa5'
Q, C, R = log_upload.QLOG, log_upload.QCAMERA, log_upload.RLOG
HOUR = 3600.


@pytest.fixture
def marks(monkeypatch):
  """xattrs as a dict: the device's are Linux's. Marking a file that is gone fails as os.setxattr does (ENOENT): a mark that
  always took hid the deleter's race until the 11-06 review."""
  done: dict[str, bytes] = {}

  def gone(fn):
    raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), fn)
  monkeypatch.setattr(log_upload, 'getxattr', lambda fn, attr: done.get(fn) if os.path.exists(fn) else gone(fn))
  monkeypatch.setattr(log_upload, 'setxattr', lambda fn, attr, v: done.__setitem__(fn, v) if os.path.exists(fn) else gone(fn))
  return done


def make_segments(root, n=3, lock_last=True, names=(Q, C, R), first=0, closed=None, size=None):
  """n segments, closed an hour ago a second apart in their order (or at closed, s since the epoch)."""
  start = wall_time() - HOUR
  for i in range(first, first + n):
    d = root / f"{ROUTE}--{i}"
    d.mkdir()
    for name in names:
      (d / name).write_bytes(os.urandom(size) if size and name == C else f"{name} {i}".encode() * 100)
    t = start + i if closed is None else closed
    os.utime(d, (t, t))
  if lock_last:
    (root / f"{ROUTE}--{first + n - 1}" / 'rlog.lock').write_bytes(b'')


def drain(root, marks, metered, wall=None):
  """Every file next_file hands out, in order, each marked sent as the uploader marks it."""
  sent = []
  while (nxt := log_upload.next_file(root, metered=metered, wall=wall)) is not None:
    sent.append((int(nxt[0].rsplit('--', 1)[1]), nxt[1]))
    marks[os.path.join(root, *nxt)] = log_upload.DONE
  return sent


def seg(i):
  return f'{ROUTE}--{i}'


def test_qlogs_first_newest_first_rlogs_only_off_metered_and_never_a_segment_being_written(tmp_path, marks):
  make_segments(tmp_path)
  root = str(tmp_path)
  assert log_upload.segments(root) == [seg(1), seg(0)]  # --2 is still being written
  assert drain(root, marks, metered=True) == [(1, Q), (0, Q), (1, C), (0, C)]  # no rlog on a metered network, the video does go
  assert log_upload.next_file(root, metered=False) == (seg(1), R)


def test_unmetered_every_qlog_then_every_qcamera_then_every_rlog_each_newest_first(tmp_path, marks):
  make_segments(tmp_path, n=4, lock_last=False)
  assert drain(str(tmp_path), marks, metered=False) == [(3, Q), (2, Q), (1, Q), (0, Q), (3, C), (2, C), (1, C), (0, C),
                                                        (3, R), (2, R), (1, R), (0, R)]


def test_a_segment_just_finished_sends_its_qlog_before_the_older_videos(tmp_path, marks):
  """The drive goes on while the videos go up: the qlog of each segment loggerd closes jumps them, as before 11-06."""
  make_segments(tmp_path, n=2, lock_last=False)
  root = str(tmp_path)
  assert drain(root, marks, metered=True)[:3] == [(1, Q), (0, Q), (1, C)]
  for k in [k for k in marks if k.endswith(f'--0/{C}')]:
    del marks[k]  # --0's video still to go when --2 closes
  make_segments(tmp_path, n=1, lock_last=False, first=2)
  assert drain(root, marks, metered=True) == [(2, Q), (2, C), (0, C)]


@pytest.mark.parametrize('lock', ['qcamera.ts.lock', 'rlog.lock', 'fcamera.hevc.lock'])
def test_nothing_of_a_segment_goes_while_any_of_its_files_is_written(tmp_path, marks, lock):
  """loggerd's video writer holds qcamera.ts.lock until it closes the video (system/loggerd/video_writer.cc)."""
  make_segments(tmp_path, n=2, lock_last=False)
  (tmp_path / seg(1) / lock).write_bytes(b'')
  assert drain(str(tmp_path), marks, metered=False) == [(0, Q), (0, C), (0, R)]


def test_a_segment_without_a_video_is_skipped_not_waited_on(tmp_path, marks):
  """A segment can lack its qcamera.ts (no road camera, or loggerd cut it short); its logs still go, in their turn."""
  make_segments(tmp_path, n=1, lock_last=False, names=(Q, R), first=0)
  make_segments(tmp_path, n=1, lock_last=False, first=1)
  assert drain(str(tmp_path), marks, metered=False) == [(1, Q), (0, Q), (1, C), (1, R), (0, R)]


def test_only_loggerds_segment_names_are_looked_at(tmp_path, marks):
  """The store takes only loggerd's <count>--<10 hex>--<n> (logstore.SEGMENT_RE). Any other name would be refused at its
  qlog, which is the server refusing this car (a wait, not a hold), so it would stand ahead of every video and rlog."""
  make_segments(tmp_path, n=1, lock_last=False)
  for d in ('2024-01-01--12-00-00--0', 'boot', 'crash', f'{ROUTE}--x'):
    (tmp_path / d).mkdir()
    (tmp_path / d / Q).write_bytes(b'q')
  assert log_upload.segments(str(tmp_path)) == [seg(0)]
  assert drain(str(tmp_path), marks, metered=False) == [(0, Q), (0, C), (0, R)]


def test_videos_older_than_two_days_stay_on_the_car_so_tonights_rlogs_are_not_behind_them(tmp_path, marks):
  """The store held 136 segments of this car's, each with a video on the car never sent (~330 MB), and the hotspot reads as
  unmetered: a segment closed 72 h ago keeps its video, one closed an hour ago sends it; every log of both still goes."""
  wall = wall_time()
  make_segments(tmp_path, n=1, lock_last=False, first=0, closed=wall - 72 * HOUR)
  make_segments(tmp_path, n=1, lock_last=False, first=1, closed=wall - HOUR)
  root = str(tmp_path)
  assert drain(root, marks, metered=False, wall=wall) == [(1, Q), (0, Q), (1, C), (1, R), (0, R)]
  marks.clear()
  assert drain(root, marks, metered=True, wall=wall) == [(1, Q), (0, Q), (1, C)]
  marks.clear()
  bound = log_upload.QCAMERA_MAX_AGE_S
  assert drain(root, marks, metered=True, wall=wall - 72 * HOUR + bound - 1) == [(1, Q), (0, Q), (1, C), (0, C)]  # just inside 48 h


def test_a_segment_closed_before_the_clock_was_set_sends_its_video(tmp_path, marks):
  """The comma boots at its OS's build date (6 of 23 routes fetched from this car began at 07-28) until NTP or GPS sets it.
  A segment closed before that has no age to read, and its video goes; so does every video while the clock is not set."""
  unset = datetime.datetime(2025, 2, 21).timestamp() - 24 * HOUR  # before common/time_helpers' MIN_DATE: before any min_date
  assert unset < log_upload.clock_set_after()
  make_segments(tmp_path, n=1, lock_last=False, first=0, closed=unset)
  make_segments(tmp_path, n=1, lock_last=False, first=1)
  assert drain(str(tmp_path), marks, metered=True) == [(1, Q), (0, Q), (1, C), (0, C)]
  marks.clear()
  assert drain(str(tmp_path), marks, metered=True, wall=unset + 60) == [(1, Q), (0, Q), (1, C), (0, C)]  # the clock not set now


def test_only_a_dongle_the_server_takes_sends_anything(monkeypatch):
  """registration.py writes 'UnregisteredDevice' when comma's API refuses the device; the store refuses every file of it
  (logstore.DONGLE_RE), so nothing is tried, and the error is said once, not every look."""
  errors = []
  monkeypatch.setattr(log_upload.cloudlog, 'error', errors.append)
  told: set[str] = set()
  assert log_upload.dongle_ok('0123456789abcdef', told)
  for bad in ('UnregisteredDevice', 'UnregisteredDevice', '0123456789ABCDEF', '0123456789abcde', '0123456789abcdef\n'):
    assert not log_upload.dongle_ok(bad, told)
  assert len(errors) == 4 and 'UnregisteredDevice' in errors[0]
  assert not log_upload.dongle_ok('', told) and len(errors) == 4  # not registered yet: nothing to say


def test_main_idles_on_an_unregistered_dongle(monkeypatch):
  class Stop(Exception):
    pass

  class SM:
    seen = {'deviceState': True}

    def update(self, _):
      pass

    def __getitem__(self, _):
      return type('DS', (), {'networkType': log_upload.NetworkType.wifi, 'networkMetered': False})()

  sleeps, sends, errors = [], [], []
  dongle = ['UnregisteredDevice']
  monkeypatch.setattr(log_upload, 'Params', lambda: type('P', (), {'get': lambda self, k: dongle[0]})())
  monkeypatch.setattr(log_upload.messaging, 'SubMaster', lambda services: SM())
  monkeypatch.setattr(log_upload.Paths, 'log_root', staticmethod(lambda: '/nonexistent'))
  monkeypatch.setattr(log_upload, 'send_next', lambda *a, **k: sends.append(a[1]) or log_upload.IDLE_S)
  monkeypatch.setattr(log_upload.cloudlog, 'error', errors.append)

  def sleep(s):
    sleeps.append(s)
    if len(sleeps) == 3:
      dongle[0] = DONGLE  # registered at last
    if len(sleeps) == 4:
      raise Stop
  monkeypatch.setattr(log_upload.time, 'sleep', sleep)
  with pytest.raises(Stop):
    log_upload.main()
  assert sleeps == [log_upload.IDLE_S] * 4 and sends == [DONGLE] and len(errors) == 1


@pytest.fixture
def tls_store(tmp_path):
  from tools.turnpilot.mapserver.logstore import LogStore
  from tools.turnpilot.mapserver.server import Maps, make_server
  cert, key = self_signed()
  (tmp_path / 'cert.pem').write_text(cert)
  (tmp_path / 'cert.key').write_text(key)
  maps = Maps([], KEY, logs=LogStore(tmp_path / 'store', 10 ** 8))
  httpd = make_server(maps, '127.0.0.1', 0, tmp_path / 'cert.pem', tmp_path / 'cert.key')
  threading.Thread(target=httpd.serve_forever, daemon=True).start()
  logs = tmp_path / 'logs'
  logs.mkdir()
  yield maps, logs, lambda: MapClient(f'https://127.0.0.1:{httpd.server_address[1]}', KEY, cert, tmp_path)
  httpd.shutdown()


class Lossy:
  """A client whose POSTs of the files in lost end as a lost connection, as when the server's close resets the connection
  under its answer; every POST it makes is in posts."""

  def __init__(self, client: MapClient, lost: set, posts: list):
    self.client, self.lost, self.posts = client, lost, posts

  def request(self, method, path, **kwargs):
    self.posts.append(tuple(path.split('/')[-2:]))
    if any(path.endswith(f'/{s}/{n}') for s, n in self.lost):
      raise MapError('unreachable', "Can't reach the TurnPilot map server (ConnectionError).", retry=True)
    return self.client.request(method, path, **kwargs)


def test_a_segment_reaches_the_map_servers_log_store_over_the_pinned_tls(tls_store, marks):
  maps, logs, connect = tls_store
  make_segments(logs, n=1, lock_last=False)
  (logs / seg(0) / C).write_bytes(os.urandom(2_400_000))  # a minute of qcamera.ts, over the uploader's and the store's 1 MiB chunks
  for name in (R, C):
    assert log_upload.upload(connect(), DONGLE, str(logs), seg(0), name) == 200
    assert maps.logs.path(DONGLE, seg(0), name).read_bytes() == (logs / seg(0) / name).read_bytes()
    assert log_upload.upload(connect(), DONGLE, str(logs), seg(0), name) == 200  # a retry is accepted, not stored twice


def test_a_server_that_does_not_know_qcamera_holds_the_videos_and_the_rlogs_still_go(tls_store, marks, monkeypatch):
  """The car may update before the server learns qcamera.ts: each refused video waits HOLD_S, the rest go up meanwhile, and
  once the server knows the name the videos follow. The videos are a minute's 2.4 MB, far past the socket buffers, so the
  server refuses with most of the body unread, as it would the car's."""
  from tools.turnpilot.mapserver import logstore
  maps, logs, connect = tls_store
  make_segments(logs, n=2, lock_last=False, size=2_400_000)
  root = str(logs)
  monkeypatch.setattr(logstore, 'NAMES', (Q, R))  # the server as deployed before 11-06
  held: dict = {}
  fails: dict = {}
  now = 1000.
  for _ in range(6):
    assert log_upload.send_next(connect, DONGLE, root, False, held, fails, now) == 0.  # a refusal moves on at once: not a lost connection
  assert log_upload.send_next(connect, DONGLE, root, False, held, fails, now) == log_upload.IDLE_S
  sent = [tuple(fn.split(os.sep)[-2:]) for fn in marks]  # in the order they were marked sent
  assert sent == [(seg(1), Q), (seg(0), Q), (seg(1), R), (seg(0), R)]
  assert held == {(seg(1), C): now + log_upload.HOLD_S, (seg(0), C): now + log_upload.HOLD_S} and fails == {}
  assert not any(fn.endswith(C) for fn in marks)  # refused is not sent: they wait on the car
  assert log_upload.send_next(connect, DONGLE, root, False, held, fails, now + log_upload.HOLD_S - 1) == log_upload.IDLE_S
  monkeypatch.setattr(logstore, 'NAMES', (Q, C, R))  # the server deployed
  later = now + log_upload.HOLD_S + 1
  assert log_upload.send_next(connect, DONGLE, root, True, held, fails, later) == 0.
  assert log_upload.send_next(connect, DONGLE, root, True, held, fails, later) == 0.
  assert log_upload.send_next(connect, DONGLE, root, True, held, fails, later) == log_upload.IDLE_S
  for s in (seg(1), seg(0)):
    assert maps.logs.path(DONGLE, s, C).read_bytes() == (logs / s / C).read_bytes()
  assert held == {}  # spent holds are dropped


@pytest.mark.parametrize('code', [400, 404, 413])
def test_a_video_or_rlog_the_server_refuses_is_held_and_the_next_goes_now(tls_store, marks, monkeypatch, code):
  """400 (a bad digest), 404 (an unknown name) and 413 (past the cap) are about that file."""
  from tools.turnpilot.mapserver.logstore import LogRefusal
  maps, logs, connect = tls_store
  make_segments(logs, n=1, lock_last=False)
  root = str(logs)
  put = maps.logs.put

  def refuse(dongle, segment, name, *a):
    if name in (C, R):
      raise LogRefusal(code, 'refused')
    return put(dongle, segment, name, *a)
  monkeypatch.setattr(maps.logs, 'put', refuse)
  held: dict = {}
  fails: dict = {}
  assert [log_upload.send_next(connect, DONGLE, root, False, held, fails, 0.) for _ in range(4)] == [0., 0., 0., log_upload.IDLE_S]
  assert held == {(seg(0), C): log_upload.HOLD_S, (seg(0), R): log_upload.HOLD_S} and fails == {}
  assert [tuple(fn.split(os.sep)[-2:]) for fn in marks] == [(seg(0), Q)]


@pytest.mark.parametrize('code', [408, 429, 500, 503, 507])
def test_a_try_later_or_a_server_error_retries_the_same_file_after_a_wait(tls_store, marks, monkeypatch, code):
  """408/429 say "later", a 5xx is the server's own trouble, 507 its full store: not about the file, so nothing is held."""
  from tools.turnpilot.mapserver.logstore import LogRefusal
  maps, logs, connect = tls_store
  make_segments(logs, n=1, lock_last=False)
  root = str(logs)
  marks[os.path.join(root, seg(0), Q)] = log_upload.DONE

  def later(*a):
    raise LogRefusal(code, 'later')
  monkeypatch.setattr(maps.logs, 'put', later)
  held: dict = {}
  fails: dict = {}
  assert log_upload.send_next(connect, DONGLE, root, False, held, fails, 0.) == log_upload.RETRY_S
  assert held == {} and fails == {(seg(0), C): 1, (log_upload.EVERY, C): 1} and log_upload.next_file(root, False, held, 0.) == (seg(0), C)


def test_a_full_store_or_a_lost_connection_retries_the_same_file_after_a_wait(tls_store, marks):
  """507 (the store past its quota) and no connection are not about the file: nothing is held, and the next look waits."""
  maps, logs, connect = tls_store
  make_segments(logs, n=1, lock_last=False)
  root = str(logs)
  maps.logs.quota_bytes = 10
  held: dict = {}
  fails: dict = {}
  assert log_upload.send_next(connect, DONGLE, root, False, held, fails, 0.) == log_upload.RETRY_S
  assert held == {} and log_upload.next_file(root, False) == (seg(0), Q)
  posts: list = []
  assert log_upload.send_next(lambda: Lossy(connect(), {(seg(0), Q)}, posts), DONGLE, root, False, held, fails, 60.) == log_upload.RETRY_S
  assert held == {} and marks == {} and fails == {(seg(0), Q): 2}

  def unset():
    raise MapError('not_configured', 'The TurnPilot map server is not set up')
  assert log_upload.send_next(unset, DONGLE, root, False, held, fails, 120.) == log_upload.RETRY_S
  assert fails == {(seg(0), Q): 2}  # no file was tried: not counted against it


def test_a_file_whose_tries_keep_failing_is_set_aside_and_the_next_goes(tls_store, marks):
  """The server refuses a video it doesn't know before reading its 2.4 MB, and its close can reset the connection under
  the 404: the car sees a lost connection. After three in a row that file stands aside 15 min and the rest go; then it is
  tried again with a fresh count, and once it goes up its count is gone."""
  maps, logs, connect = tls_store
  make_segments(logs, n=2, lock_last=False)
  root = str(logs)
  lost = {(seg(1), C)}
  posts: list = []
  lossy = lambda: Lossy(connect(), lost, posts)  # noqa: E731
  held: dict = {}
  fails: dict = {}
  waits = [log_upload.send_next(lossy, DONGLE, root, False, held, fails, 0.) for _ in range(2)]  # the qlogs
  waits += [log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) for t in (0., 60., 120.)]
  assert waits == [0., 0., log_upload.RETRY_S, log_upload.RETRY_S, 0.]
  assert held == {(seg(1), C): 120. + log_upload.SET_ASIDE_S} and fails == {(log_upload.EVERY, C): 3}  # all of one file: not the name
  assert [log_upload.send_next(lossy, DONGLE, root, False, held, fails, 120.) for _ in range(4)] == [0., 0., 0., log_upload.IDLE_S]
  assert [tuple(fn.split(os.sep)[-2:]) for fn in marks] == [(seg(1), Q), (seg(0), Q), (seg(0), C), (seg(1), R), (seg(0), R)]
  back = 120. + log_upload.SET_ASIDE_S
  assert log_upload.send_next(lossy, DONGLE, root, False, held, fails, back) == log_upload.RETRY_S
  assert held == {} and fails == {(seg(1), C): 1, (log_upload.EVERY, C): 1}  # tried again, counting from one (seg(0)'s video went)
  lost.clear()  # the server deployed
  assert log_upload.send_next(lossy, DONGLE, root, False, held, fails, back + 60) == 0.
  assert fails == {} and maps.logs.path(DONGLE, seg(1), C).is_file()
  assert len(posts) == 2 + 3 + 3 + 2  # one POST a try, none in a burst


def test_a_new_segments_qlog_between_the_tries_does_not_restart_the_count(tls_store, marks):
  """Driving, a segment closes every minute and its qlog and video go up between two tries of the stuck video. Were any
  success to restart the count, it would never reach three and the rlogs of the drive would wait behind it till it ends."""
  maps, logs, connect = tls_store
  make_segments(logs, n=1, lock_last=False)
  root = str(logs)
  posts: list = []
  lossy = lambda: Lossy(connect(), {(seg(0), C)}, posts)  # noqa: E731
  held: dict = {}
  fails: dict = {}
  t = 0.
  assert log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) == 0.  # --0's qlog
  for i in (1, 2, 3):
    assert log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) == (log_upload.RETRY_S if i < 3 else 0.)
    if i < 3:
      t += log_upload.RETRY_S
      make_segments(logs, n=1, lock_last=False, first=i)  # closed meanwhile
      assert [log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) for _ in range(2)] == [0., 0.]  # its qlog, its video
  assert held == {(seg(0), C): t + log_upload.SET_ASIDE_S}
  while log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) == 0.:
    pass
  assert [p for p in posts if p[1] == R] == [(seg(2), R), (seg(1), R), (seg(0), R)]  # the drive's rlogs went


def test_when_every_video_fails_the_videos_stand_aside_and_the_rlogs_go(tls_store, marks):
  """The deploy gap with the race lost every time: each video's 404 ends as a lost connection. Setting aside one file at a
  time took two minutes a video, each back in 15: with more than about seven waiting the rlogs never got a turn (simulated,
  0 rlogs in an hour with 28 parked). Three failed tries of the name, not all of one file, set every video aside."""
  maps, logs, connect = tls_store
  make_segments(logs, n=3, lock_last=False)
  root = str(logs)
  posts: list = []
  lossy = lambda: Lossy(connect(), {(seg(i), C) for i in range(3)}, posts)  # noqa: E731
  held: dict = {}
  fails: dict = {}
  assert [log_upload.send_next(lossy, DONGLE, root, False, held, fails, 0.) for _ in range(3)] == [0.] * 3  # the qlogs
  waits = [log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) for t in (0., 60., 120., 120.)]
  assert waits == [log_upload.RETRY_S, log_upload.RETRY_S, 0., 0.]  # --2's video aside, then --1's try makes it the name
  assert held == {(seg(2), C): 120. + log_upload.SET_ASIDE_S, (log_upload.EVERY, C): 120. + log_upload.SET_ASIDE_S}
  assert [log_upload.send_next(lossy, DONGLE, root, False, held, fails, 120.) for _ in range(4)] == [0., 0., 0., log_upload.IDLE_S]
  assert [p for p in posts if p[1] == R] == [(seg(2), R), (seg(1), R), (seg(0), R)]
  assert [p for p in posts if p[1] == C] == [(seg(2), C)] * 3 + [(seg(1), C)]
  back = 120. + log_upload.SET_ASIDE_S
  assert log_upload.send_next(lossy, DONGLE, root, False, held, fails, back) == log_upload.RETRY_S  # the videos are tried again
  assert posts[-1] == (seg(2), C) and held == {}


def test_while_driving_with_every_video_failing_the_videos_stand_aside_and_the_new_qlogs_still_go_first(tls_store, marks):
  """Driving, each try of a video is of a newer one (a segment closes every minute, newest first), so no one file reaches
  three tries: the name's count does, across files, and the drive's rlogs go. A segment closing while the videos stand
  aside still sends its qlog first."""
  maps, logs, connect = tls_store
  make_segments(logs, n=1, lock_last=False)
  root = str(logs)
  posts: list = []

  class AllVideosLost(Lossy):
    def request(self, method, path, **kwargs):
      if path.endswith(f'/{C}'):
        self.posts.append(tuple(path.split('/')[-2:]))
        raise MapError('unreachable', "Can't reach the TurnPilot map server (ConnectionError).", retry=True)
      return super().request(method, path, **kwargs)
  lossy = lambda: AllVideosLost(connect(), set(), posts)  # noqa: E731
  held: dict = {}
  fails: dict = {}
  t = 0.
  assert log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) == 0.  # --0's qlog
  assert log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) == log_upload.RETRY_S  # --0's video
  for i in (1, 2):
    t += log_upload.RETRY_S
    make_segments(logs, n=1, lock_last=False, first=i)  # closed meanwhile
    assert log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) == 0.  # its qlog
    assert log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) == (log_upload.RETRY_S if i < 2 else 0.)  # its video
  assert [p for p in posts if p[1] == C] == [(seg(0), C), (seg(1), C), (seg(2), C)]
  assert held == {(log_upload.EVERY, C): t + log_upload.SET_ASIDE_S}
  make_segments(logs, n=1, lock_last=False, first=3)
  while log_upload.send_next(lossy, DONGLE, root, False, held, fails, t) == 0.:
    pass
  went = [tuple(fn.split(os.sep)[-2:]) for fn in marks]
  assert went[-5:] == [(seg(3), Q), (seg(3), R), (seg(2), R), (seg(1), R), (seg(0), R)]
  assert len([p for p in posts if p[1] == C]) == 3  # no video tried while they stand aside


def test_a_refused_qlog_is_the_server_refusing_this_car_not_a_burst(tls_store, marks):
  """A server with no log store refuses everything; every server version takes qlog.zst, so its refusal is about the car.
  Each look waits RETRY_S, nothing is held, and no video or rlog is POSTed while a qlog is still to go; after three
  refusals the qlog stands aside like any file whose tries keep failing, so a single bad one can't stop the rest."""
  maps, logs, connect = tls_store
  make_segments(logs, n=2, lock_last=False)
  root = str(logs)
  maps.logs = None
  posts: list = []
  counted = lambda: Lossy(connect(), set(), posts)  # noqa: E731
  held: dict = {}
  fails: dict = {}
  waits = [log_upload.send_next(counted, DONGLE, root, False, held, fails, 60. * i) for i in range(3)]
  assert waits == [log_upload.RETRY_S, log_upload.RETRY_S, 0.] and posts == [(seg(1), Q)] * 3
  assert held == {(seg(1), Q): 120. + log_upload.SET_ASIDE_S}
  assert log_upload.send_next(counted, DONGLE, root, False, held, fails, 120.) == log_upload.RETRY_S
  assert posts[-1] == (seg(0), Q) and fails == {(seg(0), Q): 1} and marks == {}


class DeletedWhileRead:
  """The open file a POST reads, its segment taken by the deleter (deleter_step's rmtree) at the first read: the rest still
  comes off the open file, as it does on the device."""

  def __init__(self, f, d: str):
    self.f, self.d = f, d

  def fileno(self):
    return self.f.fileno()

  def tell(self):
    return self.f.tell()

  def __iter__(self):
    return iter(self.f)

  def read(self, n=-1):
    if os.path.isdir(self.d):
      shutil.rmtree(self.d)
    return self.f.read(n)


class DeleterDuringPost(Lossy):
  """A client whose every POST has its segment deleted while the body goes."""

  def __init__(self, client: MapClient, posts: list, root: str):
    super().__init__(client, set(), posts)
    self.root = root

  def request(self, method, path, **kwargs):
    kwargs['data'] = DeletedWhileRead(kwargs['data'], os.path.join(self.root, path.split('/')[-2]))
    return super().request(method, path, **kwargs)


def test_a_segment_the_deleter_takes_while_its_file_goes_up_is_sent_and_the_uploader_lives_on(tls_store, marks):
  """Newest first, the last files to go are the oldest, where the deleter frees space: it took the segment while its rlog
  went, the server answered 200, and the mark's FileNotFoundError killed turnpilot_logs for the boot (main() has no handler,
  and the manager does not start it again). The server holds the file: a success, and the uploader looks again."""
  maps, logs, connect = tls_store
  make_segments(logs, n=1, lock_last=False)
  root = str(logs)
  body = os.urandom(2_400_000)  # past the uploader's 1 MiB reads: the segment is gone while most of it is still to go
  (logs / seg(0) / R).write_bytes(body)
  for name in (Q, C):
    marks[os.path.join(root, seg(0), name)] = log_upload.DONE
  posts: list = []
  held: dict = {}
  fails: dict = {(seg(0), R): 2, (log_upload.EVERY, R): 2}
  assert log_upload.send_next(lambda: DeleterDuringPost(connect(), posts, root), DONGLE, root, False, held, fails, 0.) == 0.
  assert not (logs / seg(0)).exists() and maps.logs.path(DONGLE, seg(0), R).read_bytes() == body
  assert posts == [(seg(0), R)] and held == {} and fails == {}  # as any success: both counts gone
  assert log_upload.send_next(connect, DONGLE, root, False, held, fails, 0.) == log_upload.IDLE_S


def test_a_file_the_deleter_took_before_its_post_counts_nothing_and_the_next_goes_now(tls_store, marks):
  """Taken after next_file picked it and before upload() opened it: nothing is left to send and no try of it failed, so its
  count goes, its name's stays as it was (neither failed nor went up), and the next file goes at once, not after RETRY_S."""
  maps, logs, connect = tls_store
  make_segments(logs, n=2, lock_last=False)
  root = str(logs)
  for s in (seg(1), seg(0)):
    marks[os.path.join(root, s, Q)] = log_upload.DONE
  posts: list = []

  def deleted_meanwhile():
    shutil.rmtree(logs / seg(1))  # the deleter, between next_file and upload()
    return Lossy(connect(), set(), posts)
  held: dict = {}
  fails: dict = {(seg(1), C): 2, (log_upload.EVERY, C): 2}
  assert log_upload.send_next(deleted_meanwhile, DONGLE, root, False, held, fails, 0.) == 0.
  assert posts == [] and held == {} and fails == {(log_upload.EVERY, C): 2}
  assert log_upload.send_next(lambda: Lossy(connect(), set(), posts), DONGLE, root, False, held, fails, 0.) == 0.
  assert posts == [(seg(0), C)] and fails == {} and maps.logs.path(DONGLE, seg(0), C).is_file()


def test_a_file_sent_but_not_marked_is_a_failed_try_not_a_loop(tls_store, marks, monkeypatch):
  """The server holds it (200) but the mark fails with the file still there (EIO): unmarked, next_file hands it out again
  at once and the server takes all of it again ("already stored"), with no wait. It is a failed try: RETRY_S, counted, and
  after three it stands aside like any file whose tries keep failing, and the next goes."""
  maps, logs, connect = tls_store
  make_segments(logs, n=1, lock_last=False)
  root = str(logs)
  marks[os.path.join(root, seg(0), Q)] = log_upload.DONE
  video = os.path.join(root, seg(0), C)
  mark = log_upload.setxattr

  def eio(fn, attr, v):
    if fn == video:
      raise OSError(errno.EIO, os.strerror(errno.EIO), fn)
    mark(fn, attr, v)
  monkeypatch.setattr(log_upload, 'setxattr', eio)
  posts: list = []
  counted = lambda: Lossy(connect(), set(), posts)  # noqa: E731
  held: dict = {}
  fails: dict = {}
  assert [log_upload.send_next(counted, DONGLE, root, False, held, fails, t) for t in (0., 60., 120.)] == \
    [log_upload.RETRY_S, log_upload.RETRY_S, 0.]
  assert posts == [(seg(0), C)] * 3 and maps.logs.path(DONGLE, seg(0), C).is_file()  # held by the server from the first
  assert held == {(seg(0), C): 120. + log_upload.SET_ASIDE_S} and fails == {(log_upload.EVERY, C): 3}
  assert [log_upload.send_next(counted, DONGLE, root, False, held, fails, 120.) for _ in range(2)] == [0., log_upload.IDLE_S]
  assert posts[-1] == (seg(0), R) and video not in marks


def test_no_log_root_to_list_is_nothing_to_send_not_a_dead_uploader(tmp_path, marks, monkeypatch):
  """loggerd makes the log root, and comma's uploader reads none as nothing to send (listdir_by_creation): an OSError from
  the look would kill the uploader for the boot."""
  errors = []
  monkeypatch.setattr(log_upload.cloudlog, 'exception', errors.append)
  root = str(tmp_path / 'realdata')
  assert log_upload.segments(root) == [] and log_upload.next_file(root, False) is None
  assert log_upload.send_next(lambda: pytest.fail('nothing to send'), DONGLE, root, False, {}, {}, 0.) == log_upload.IDLE_S
  assert errors == []  # not made yet: nothing to say
  if os.geteuid() == 0:
    return  # root lists a mode-0 directory: no unlistable root to make
  locked = tmp_path / 'locked'
  locked.mkdir()
  locked.chmod(0)
  try:
    assert log_upload.segments(str(locked)) == [] and len(errors) == 1
  finally:
    locked.chmod(0o755)
