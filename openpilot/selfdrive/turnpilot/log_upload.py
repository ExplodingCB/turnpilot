"""TurnPilot's drive logs to our own map server (tools/turnpilot/mapserver/logstore.py), so a drive can be looked at by us, not
only by whoever can sign in to the car's comma connect.

2026-11-02, the second drive: TurnPilot did nothing on a shown route, and its segments came over by hand, one at a time.
Now every finished segment's qlog goes up by itself, and its rlog after every qlog is up, off metered connections only
(an rlog is about 8 MB a minute, too much for a phone's hotspot; a qlog is about 1 MB). The newest segments go first, so
the drive just done is there before older ones. A file goes up once: user.turnpilot_upload marks it, as comma's uploader
marks its own (system/loggerd/uploader.py), and the server accepts a retry of one it holds. Same server, key and pinned
certificate as the maps (maps/client.py). Nothing here deletes: comma's deleter frees space as ever.
"""
import hashlib
import os
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.maps.client import MapClient, MapError
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr

NetworkType = log.DeviceState.NetworkType
ATTR = 'user.turnpilot_upload'
DONE = b'1'
QLOG, RLOG = 'qlog.zst', 'rlog.zst'
UPLOAD_TIMEOUT = (10., 120.)  # s to connect, s between bytes
IDLE_S = 30.        # s between looks when there is nothing to send or no network
RETRY_S = 60.       # s after a failed upload before the next try


def segments(root: str) -> list[str]:
  """Finished segment directories under root, newest first. A segment loggerd still writes holds a .lock."""
  out = []
  for d in os.listdir(root):
    path = os.path.join(root, d)
    if d.count('--') != 2 or not os.path.isdir(path):
      continue
    try:
      if any(n.endswith('.lock') for n in os.listdir(path)):
        continue
      out.append((os.path.getmtime(path), d))
    except OSError:
      continue
  return [d for _, d in sorted(out, reverse=True)]


def uploaded(fn: str) -> bool:
  try:
    return getxattr(fn, ATTR) == DONE
  except OSError:
    return True  # gone (the deleter): nothing to send


def next_file(root: str, metered: bool) -> tuple[str, str] | None:
  """(segment, name) to send next: every qlog first, newest first, then off metered connections every rlog the same way."""
  segs = segments(root)
  for name in (QLOG, RLOG):
    if name == RLOG and metered:
      return None
    for seg in segs:
      fn = os.path.join(root, seg, name)
      if os.path.isfile(fn) and not uploaded(fn):
        return seg, name
  return None


def upload(client: MapClient, dongle: str, root: str, seg: str, name: str) -> bool:
  fn = os.path.join(root, seg, name)
  h = hashlib.sha256()
  with open(fn, 'rb') as f:
    for chunk in iter(lambda: f.read(1 << 20), b''):
      h.update(chunk)
  with open(fn, 'rb') as f:
    resp = client.request('POST', f'/v1/logs/{dongle}/{seg}/{name}', timeout=UPLOAD_TIMEOUT, data=f,
                          headers={'X-Content-SHA256': h.hexdigest(), 'Content-Length': str(os.path.getsize(fn))})
  ok = resp.status_code == 200
  if not ok:
    cloudlog.warning(f"turnpilot logs: {seg}/{name} refused with HTTP {resp.status_code}: {resp.text[:200]}")
  resp.close()
  return ok


def main():
  params = Params()
  sm = messaging.SubMaster(['deviceState'])
  root = Paths.log_root()
  while True:
    sm.update(0)
    ds = sm['deviceState']
    dongle = (params.get('DongleId') or '').strip()
    if not sm.seen['deviceState'] or ds.networkType == NetworkType.none or not dongle:
      time.sleep(IDLE_S)
      continue
    nxt = next_file(root, ds.networkMetered)
    if nxt is None:
      time.sleep(IDLE_S)
      continue
    seg, name = nxt
    try:
      client = MapClient.from_params(params)
      if upload(client, dongle, root, seg, name):
        setxattr(os.path.join(root, seg, name), ATTR, DONE)
        cloudlog.info(f"turnpilot logs: sent {seg}/{name}")
        continue
    except (MapError, OSError) as e:
      cloudlog.warning(f"turnpilot logs: {seg}/{name} not sent: {type(e).__name__}: {e}")
    time.sleep(RETRY_S)


if __name__ == "__main__":
  main()
