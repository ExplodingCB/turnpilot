"""TurnPilot's drive logs to our own map server (tools/turnpilot/mapserver/logstore.py), so a drive can be looked at by us, not
only by whoever can sign in to the car's comma connect.

2026-11-02, the second drive: TurnPilot did nothing on a shown route, and its segments came over by hand, one at a time.
Now every finished segment's qlog goes up by itself, and its rlog after every qlog is up, off metered connections only
(an rlog is about 8 MB a minute, too much for a metered phone plan; a qlog is about 1 MB). The newest segments go first, so
the drive just done is there before older ones. A file goes up once: user.turnpilot_upload marks it, as comma's uploader
marks its own (system/loggerd/uploader.py), and the server accepts a retry of one it holds. Same server, key and pinned
certificate as the maps (maps/client.py). Nothing here deletes: comma's deleter frees space as ever.

2026-11-06, after the first night drive (routes 1d and 1e): the logs said what the car did, not what it saw. The curb it
clipped, the bus lane it drove in and the lights it read wrong at night could only be told back from Chase's memory, and the
night light reader can't be tuned without real night frames. So each segment's qcamera.ts goes up too: comma's small road
video (526x330 H.264, system/loggerd/loggerd.h), the one comma connect plays. loggerd encodes it at a constant 256 kbit/s,
and both drives bear that out: 1200 qNarrowRoadEncodeIdx frames a segment holding 1.92 MB of H.264 a minute. ffmpeg's
MPEG-TS muxer (loggerd's) makes that 2.4 MB, 2.6 MB with RecordAudio's 32 kbit/s AAC (these drives have none). That is
five qlogs (0.45 MB a minute on these drives) and a quarter of an rlog (8-10 MB). It goes after every qlog is up and before
any rlog, newest first, on a metered connection as well: comma's own uploader sends a qcamera on a metered connection only
for a route someone opened in connect; ours goes because these drives are reviewed after every one. A segment loggerd still
writes holds a .lock, the video's as well (video_writer.cc makes qcamera.ts.lock beside it), so a qcamera goes up only once
it is closed; a segment without one is skipped, not waited on.

2026-11-06, the review of that: on this car the metered gate never binds. Chase's phone hotspot reads as wifi: every
deviceState of both drives says wifi, networkMetered false, started, and the car sent 12 and 13 rlogs while driving 1d and
1e. So qlogs, videos and rlogs all stream over the hotspot while it drives, and the videos stand ahead of the rlogs of the
drive going on. (The gate stays, for a car on a metered network.) What must not stand there is a backlog: the store already
held 136 segments of 31 routes from this car, each with a qcamera.ts still on the car and never sent, about 330 MB before
the first rlog. So a video goes up only for a segment closed within QCAMERA_MAX_AGE_S (48 h), by its directory's mtime:
loggerd removes a segment's .locks as it closes it, the directory's last change. The night before's videos still go at the
next drive. The comma boots with its clock at its OS's build date until NTP or GPS sets it (common/time_helpers.py): 6 of
the 23 routes fetched from this car began at 07-28, each set within its first segment. A segment closed before the clock is
set has an mtime that says nothing of its age, so its video goes.

The server may not know a name yet: it learns qcamera.ts when it is deployed, and the car may update first. A 404 then, a
413 for a file past the server's cap, or a 400, is the server refusing that file, not a lost connection, so the file is held
for HOLD_S while the rest go up, and tried again after: before 11-06 one refused file was retried every minute forever, and
every rlog behind it waited. But every server version takes qlog.zst, so a refused qlog is the server refusing this car, not
the file: it waits RETRY_S and is not held, since holding it would hash and POST every file on the car in one burst, each
refused in turn. A dongle the server can't take ('UnregisteredDevice', which registration.py writes when comma's API refuses
this device) sends nothing at all: main() says so once and waits. A lost connection, a full store (507), a server error or
a try-later (408, 429) waits RETRY_S and tries the same file.

The hold needs the refusal to reach the car, and the server refuses before it reads the body: with 2.4 MB of video unread,
its close can reset the connection under the answer, and the try ends as a lost connection (a MapError) instead. So a file
whose tries fail SET_ASIDE_TRIES times in a row (any failure but a client that can't be made, a refused qlog as well) is set
aside SET_ASIDE_S and the next file goes now. Only that file's own success restarts its count: driving, a new segment's qlog
goes up every minute, as often as a failed try is retried, so a count any success restarted would never reach the limit,
and the rlogs would wait behind the video for the whole drive. When it is the network, the next file fails as well and
stands aside in its turn, one every few minutes, each back after SET_ASIDE_S.

That alone is not enough when the server loses its answer for every video (the deploy gap, with the race lost each time).
Parked, each video takes three tries, two minutes, to stand aside and is back after SET_ASIDE_S, so with more than about
seven waiting the rlogs never get a turn; driving, each try is of a newer video (a segment closes a minute, newest first),
so no one file reaches three at all. Simulated against a server losing every qcamera answer, with 1, 5 or 28 videos waiting
and driving, or 28 parked, that sent 0 rlogs in an hour. So the name keeps its own count too, of failed tries since a file
of it went up: SET_ASIDE_TRIES of them in a row, not all of one file, is the name, not a file, and every file of the name
stands aside SET_ASIDE_S while the next name goes. The same simulations then send every rlog, the first about 2 min in;
with one video lost instead, every other video and every rlog goes. Never the qlogs as a name: every server takes them and
they go first, so qlogs failing one after another is the network or the server refusing this car, which no other file gets
past, and moving on would POST every video and rlog on the car, refused in turn.

2026-11-06, the review of that: comma's deleter frees space by an rmtree of the oldest segment without a .lock
(system/loggerd/deleter.py), and newest first, the uploader's last files are the oldest ones, the 11-01 and 11-02 rlogs still
on the car, just where it deletes. upload() sends from an open file, so the deleter can take the segment while its 8 MB rlog
goes over the hotspot: the server still answers 200, and the mark then finds no file. Before 11-06 main()'s one except
(MapError, OSError) caught that; when send_next took main() apart the mark was left outside it, so the FileNotFoundError
killed turnpilot_logs for the rest of the boot (the manager starts a process once while it should run: process.py), and no
qlog, video or rlog went up. Now a file found gone when its try raises, whatever raised, is the deleter's: nothing is left to send
and no try of it failed, so its count goes, its name's stays as it was, and the next file goes now. Gone after a 200, the
server holds it: a success. A mark that fails with the file still there (EIO) is a failed try, not a success: unmarked, the
same file would be handed out at once and the server would take all 8 MB again ("already stored") in a loop with no wait,
so RETRY_S and the set-aside bound it. And with no log root to list there is nothing to send, as comma's uploader has it
(listdir_by_creation), not a dead uploader.
"""
import hashlib
import os
import re
import time
from collections.abc import Callable
from functools import cache

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import min_date
from openpilot.selfdrive.turnpilot.maps.client import MapClient, MapError
from openpilot.selfdrive.turnpilot.maps.paths import wall_time
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr

NetworkType = log.DeviceState.NetworkType
ATTR = 'user.turnpilot_upload'
DONE = b'1'
QLOG, QCAMERA, RLOG = 'qlog.zst', 'qcamera.ts', 'rlog.zst'
ORDER = (QLOG, QCAMERA, RLOG)  # what goes first: every qlog, then every qcamera, then (off metered connections) every rlog
SEGMENT_RE = re.compile(r'[0-9a-f]{8}--[0-9a-f]{10}--\d{1,4}')  # loggerd's (logger.cc), and the only ones the store takes
DONGLE_RE = re.compile(r'[0-9a-f]{16}')  # a registered comma's, the only ones the store takes (logstore.py)
UPLOAD_TIMEOUT = (10., 120.)  # s to connect, s between bytes
IDLE_S = 30.        # s between looks when there is nothing to send or no network
RETRY_S = 60.       # s after a failed upload before the next try
HOLD_S = 3 * 3600.  # s a file the server refused (a 4xx) waits while the others go up: a server deploy is done well within it
NOT_A_REFUSAL = (408, 429)  # 4xx that say "later", not "not this file"
QCAMERA_MAX_AGE_S = 48 * 3600.  # s since its segment closed after which a video stays on the car (the module docstring)
SET_ASIDE_TRIES = 3     # failed tries in a row of one file after which it stands aside for the next
SET_ASIDE_S = 15 * 60.  # s it stands aside
EVERY = '*'             # the segment in a held or fails key that stands for every file of its name (no segment is named so)


@cache
def clock_set_after() -> float:
  """s since the epoch past which the comma's clock has been set: it boots at its systemd's build date until NTP or GPS
  sets it, and common/time_helpers.py's min_date is a day past that."""
  return min_date().timestamp()


def segment_times(root: str) -> list[tuple[str, float]]:
  """Finished segment directories under root with their mtimes (s since the epoch), newest first. A segment loggerd still
  writes holds a .lock, and loggerd removes the last one as it closes the segment, so the mtime is when it closed. No root,
  or one that can't be listed, has none (system/loggerd/uploader.py's listdir_by_creation)."""
  out = []
  if not os.path.isdir(root):
    return []
  try:
    names = os.listdir(root)
  except OSError:
    cloudlog.exception(f"turnpilot logs: can't list {root}")
    return []
  for d in names:
    path = os.path.join(root, d)
    if not SEGMENT_RE.fullmatch(d) or not os.path.isdir(path):
      continue
    try:
      if any(n.endswith('.lock') for n in os.listdir(path)):
        continue
      out.append((os.path.getmtime(path), d))
    except OSError:
      continue
  return [(d, t) for t, d in sorted(out, reverse=True)]


def segments(root: str) -> list[str]:
  """Finished segment directories under root, newest first."""
  return [d for d, _ in segment_times(root)]


def video_too_old(closed: float, wall: float) -> bool:
  """Whether a segment closed at closed (its directory's mtime) is past QCAMERA_MAX_AGE_S at wall. One closed before the
  comma's clock was set has no age to tell, and is not."""
  return closed > clock_set_after() and wall - closed > QCAMERA_MAX_AGE_S


def uploaded(fn: str) -> bool:
  try:
    return getxattr(fn, ATTR) == DONE
  except OSError:
    return True  # gone (the deleter): nothing to send


def next_file(root: str, metered: bool, held: dict[tuple[str, str], float] | None = None, now: float = 0.,
              wall: float | None = None) -> tuple[str, str] | None:
  """(segment, name) to send next: every qlog first, newest first, then every qcamera the same way of a segment closed
  within QCAMERA_MAX_AGE_S of wall (s since the epoch, the clock's now if None), then off metered connections every rlog.
  A file in held is skipped until its time there (monotonic s) has passed, and every file of a name held as (EVERY, name)."""
  wall = wall_time() if wall is None else wall
  segs = segment_times(root)
  for name in ORDER:
    if name == RLOG and metered:
      return None
    if held and held.get((EVERY, name), 0.) > now:
      continue
    for seg, closed in segs:
      if held and held.get((seg, name), 0.) > now:
        continue
      if name == QCAMERA and video_too_old(closed, wall):
        continue
      fn = os.path.join(root, seg, name)
      if os.path.isfile(fn) and not uploaded(fn):
        return seg, name
  return None


def upload(client: MapClient, dongle: str, root: str, seg: str, name: str) -> int:
  """Send one file. Returns the server's HTTP status: 200 when it holds the file."""
  fn = os.path.join(root, seg, name)
  h = hashlib.sha256()
  with open(fn, 'rb') as f:
    for chunk in iter(lambda: f.read(1 << 20), b''):
      h.update(chunk)
  with open(fn, 'rb') as f:  # the open file's size, not the path's: the deleter can take the path while it goes
    resp = client.request('POST', f'/v1/logs/{dongle}/{seg}/{name}', timeout=UPLOAD_TIMEOUT, data=f,
                          headers={'X-Content-SHA256': h.hexdigest(), 'Content-Length': str(os.fstat(f.fileno()).st_size)})
  if resp.status_code != 200:
    cloudlog.warning(f"turnpilot logs: {seg}/{name} refused with HTTP {resp.status_code}: {resp.text[:200]}")
  resp.close()
  return resp.status_code


def failed(held: dict[tuple[str, str], float], fails: dict[tuple[str, str], int], now: float, seg: str, name: str, why: str) -> float:
  """A try of seg/name that failed (module docstring). fails[(seg, name)] counts the file's failed tries since it went up,
  fails[(EVERY, name)] the name's (not the qlogs'). After SET_ASIDE_TRIES of the name in a row, not all of this file, every
  file of the name is set aside SET_ASIDE_S and the next name goes now; else after SET_ASIDE_TRIES of the file it is, and the
  next file goes now; else it is tried again after RETRY_S. Returns the s to wait before the next look."""
  key, every = (seg, name), (EVERY, name)
  tries = fails[key] = fails.get(key, 0) + 1
  aside = SET_ASIDE_S / 60
  if name != QLOG:
    streak = fails[every] = fails.get(every, 0) + 1
    if streak >= SET_ASIDE_TRIES and streak > tries:
      del fails[every]
      held[every] = now + SET_ASIDE_S
      cloudlog.error(f"turnpilot logs: {streak} tries of {name} in a row failed, of more than one file, the last {seg} ({why}); " +
                     f"every {name} set aside {aside:.0f} min, the next name goes now")
      return 0.
  if tries < SET_ASIDE_TRIES:
    cloudlog.warning(f"turnpilot logs: {seg}/{name} not sent ({why}); try {tries} of {SET_ASIDE_TRIES}, again in {RETRY_S:.0f} s")
    return RETRY_S
  del fails[key]
  held[key] = now + SET_ASIDE_S
  cloudlog.error(f"turnpilot logs: {seg}/{name} failed {SET_ASIDE_TRIES} tries in a row ({why}); set aside {aside:.0f} min, the next file goes now")
  return 0.


def send_next(connect: Callable[[], MapClient], dongle: str, root: str, metered: bool, held: dict[tuple[str, str], float],
              fails: dict[tuple[str, str], int], now: float, wall: float | None = None) -> float:
  """Send the next file, if any. connect() makes the client. held maps a file, or (EVERY, name) every file of a name, to the
  monotonic s it waits until; fails counts failed tries (failed()). Both live only in the process. now is monotonic s, wall
  s since the epoch (next_file). Nothing the disk or the deleter does raises out of it (module docstring): main() has no
  handler, and a process that dies stays dead for the boot.
  Returns the s to wait before the next look: none after a file went up, was refused, was set aside or was deleted meanwhile
  (the next one goes now), RETRY_S after a failed try, a refused qlog or a server that can't take it now."""
  for k in [k for k, t in held.items() if t <= now]:
    del held[k]  # spent: tried again now
  nxt = next_file(root, metered, held, now, wall)
  if nxt is None:
    return IDLE_S
  seg, name = nxt
  key, fn = (seg, name), os.path.join(root, seg, name)
  try:
    client = connect()
  except (MapError, OSError) as e:  # the map server not set up, or its certificate not saved: no file was tried
    cloudlog.warning(f"turnpilot logs: no map server: {type(e).__name__}: {e}")
    return RETRY_S
  try:
    status = upload(client, dongle, root, seg, name)
  except (MapError, OSError) as e:  # OSError: the deleter took the file before it went, or the disk
    if not os.path.isfile(fn):  # as next_file looks: it won't hand the file out again
      fails.pop(key, None)  # nothing left to send, and no try of it failed: its name's count stays as it was
      cloudlog.info(f"turnpilot logs: {seg}/{name} deleted before it went up ({type(e).__name__}: {e})")
      return 0.
    return failed(held, fails, now, seg, name, f"{type(e).__name__}: {e}")
  if status == 200:
    try:
      setxattr(fn, ATTR, DONE)
    except OSError as e:
      if os.path.isfile(fn):  # unmarked, it would go again at once, all of it: a failed try waits and counts
        return failed(held, fails, now, seg, name, f"sent but not marked: {type(e).__name__}: {e}")
      cloudlog.info(f"turnpilot logs: sent {seg}/{name}, deleted while it went")  # the server holds it
    else:
      cloudlog.info(f"turnpilot logs: sent {seg}/{name}")
    fails.pop(key, None)
    fails.pop((EVERY, name), None)
    return 0.
  if 400 <= status < 500 and status not in NOT_A_REFUSAL:
    if name == QLOG:
      cloudlog.error(f"turnpilot logs: the server refuses this car's logs ({seg}/{name}: HTTP {status}); next try in {RETRY_S:.0f} s")
      return failed(held, fails, now, seg, name, f"HTTP {status}")
    fails.pop(key, None)
    held[key] = now + HOLD_S
    cloudlog.error(f"turnpilot logs: the server refuses {seg}/{name} (HTTP {status}); held {HOLD_S / 3600:.0f} h, the rest go first")
    return 0.
  return failed(held, fails, now, seg, name, f"HTTP {status}")


def dongle_ok(dongle: str, told: set[str]) -> bool:
  """Whether dongle is one the server takes. One it can't take ('UnregisteredDevice', which registration.py writes when
  comma's API refuses this device) would have every file refused: an error once per value (told keeps them), and nothing
  goes up. An empty one is a comma not registered yet: no error."""
  if DONGLE_RE.fullmatch(dongle):
    return True
  if dongle and dongle not in told:
    told.add(dongle)
    cloudlog.error(f"turnpilot logs: DongleId {dongle!r} is not a registered comma's; no log goes up until it is")
  return False


def main():
  params = Params()
  sm = messaging.SubMaster(['deviceState'])
  root = Paths.log_root()
  held: dict[tuple[str, str], float] = {}
  fails: dict[tuple[str, str], int] = {}
  told: set[str] = set()
  while True:
    sm.update(0)
    ds = sm['deviceState']
    dongle = (params.get('DongleId') or '').strip()
    if not dongle_ok(dongle, told) or not sm.seen['deviceState'] or ds.networkType == NetworkType.none:
      time.sleep(IDLE_S)
      continue
    wait = send_next(lambda: MapClient.from_params(params), dongle, root, ds.networkMetered, held, fails, time.monotonic())
    if wait:
      time.sleep(wait)


if __name__ == "__main__":
  main()
