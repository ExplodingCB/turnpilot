"""The drive logs the cars send the map server (TurnPilot's log upload, selfdrive/turnpilot/log_upload.py), so a drive can be
looked at without the car's owner sending segments by hand.

  POST /v1/logs/<dongle>/<segment>/<name>   the file's bytes, X-Content-SHA256 its digest -> {"ok": true}
  GET  /v1/logs                             {"<dongle>": {"<segment>": {"<name>": size}}}
  GET  /v1/logs/<dongle>/<segment>/<name>   the file

A file is written to a temp file beside it, checked against its digest, then renamed into place, so a broken upload never
leaves a partial log. One already stored with the same digest is accepted again as is (the car retries after a lost
answer). The store never deletes: past --log-quota-gb, or with under 2 GB left on the disk, it refuses new files with 507, and the car keeps them until there is
room.
"""
import hashlib
import os
import re
import shutil
import tempfile
from pathlib import Path

DONGLE_RE = re.compile(r'^[0-9a-f]{16}$')
SEGMENT_RE = re.compile(r'^[0-9a-f]{8}--[0-9a-f]{10}--\d{1,4}$')
NAMES = ('qlog.zst', 'rlog.zst')
MAX_LOG_BYTES = 200 * 1024 * 1024  # an rlog is about 8 MB a minute and a segment a minute
CHUNK = 1024 * 1024
DIGEST_SUFFIX = '.sha256'
MIN_FREE_BYTES = 2 * 1024 ** 3  # the disk keeps this free whatever the quota: the regions' swaps and corridors need room


class LogRefusal(Exception):
  def __init__(self, code: int, error: str, detail: str = ''):
    super().__init__(detail or error)
    self.code, self.body = code, {'error': error, **({'detail': detail} if detail else {})}


def parse(path: str) -> tuple[str, str, str]:
  """'/v1/logs/<dongle>/<segment>/<name>' -> its three parts, or a 404 for anything else (no path outside the store)."""
  parts = path.split('/')
  if len(parts) != 6 or parts[:3] != ['', 'v1', 'logs']:
    raise LogRefusal(404, 'not_found')
  dongle, segment, name = parts[3:]
  if not DONGLE_RE.match(dongle) or not SEGMENT_RE.match(segment) or name not in NAMES:
    raise LogRefusal(404, 'not_found')
  return dongle, segment, name


class LogStore:
  def __init__(self, root: Path, quota_bytes: int):
    self.root = Path(root)
    self.quota_bytes = quota_bytes
    self.root.mkdir(parents=True, exist_ok=True)

  def path(self, dongle: str, segment: str, name: str) -> Path:
    return self.root / dongle / segment / name

  def used(self) -> int:
    return sum(f.stat().st_size for f in self.root.rglob('*') if f.is_file())

  def listing(self) -> dict:
    out: dict = {}
    for f in sorted(self.root.glob('*/*/*')):
      if f.is_file() and f.name in NAMES:
        out.setdefault(f.parent.parent.name, {}).setdefault(f.parent.name, {})[f.name] = f.stat().st_size
    return out

  def put(self, dongle: str, segment: str, name: str, length: int, digest: str, read) -> bool:
    """Store length bytes from read(n) under the file's name. Returns False if it was already stored with this digest."""
    digest = digest.strip().lower()
    if not re.fullmatch(r'[0-9a-f]{64}', digest):
      raise LogRefusal(400, 'bad_request', 'X-Content-SHA256 missing or malformed')
    if not 0 < length <= MAX_LOG_BYTES:
      raise LogRefusal(413, 'too_large')
    dst = self.path(dongle, segment, name)
    sidecar = dst.with_name(dst.name + DIGEST_SUFFIX)
    if dst.is_file() and sidecar.is_file() and sidecar.read_text().strip() == digest:
      drain(read, length)
      return False
    if self.used() + length > self.quota_bytes or shutil.disk_usage(self.root).free - length < MIN_FREE_BYTES:
      raise LogRefusal(507, 'log_store_full', 'the server log store is past its quota or the disk is nearly full')
    dst.parent.mkdir(parents=True, exist_ok=True)
    h = hashlib.sha256()
    fd, tmp = tempfile.mkstemp(dir=dst.parent, prefix=f'.{name}-')
    try:
      with os.fdopen(fd, 'wb') as f:
        left = length
        while left:
          chunk = read(min(CHUNK, left))
          if not chunk:
            raise LogRefusal(400, 'bad_request', 'body shorter than Content-Length')
          f.write(chunk)
          h.update(chunk)
          left -= len(chunk)
        f.flush()
        os.fsync(f.fileno())
      if h.hexdigest() != digest:
        raise LogRefusal(400, 'bad_request', 'body does not match X-Content-SHA256')
      os.replace(tmp, dst)
      sidecar.write_text(digest + '\n')
    finally:
      if os.path.exists(tmp):
        os.unlink(tmp)
    return True


def drain(read, length: int):
  while length:
    chunk = read(min(CHUNK, length))
    if not chunk:
      return
    length -= len(chunk)
