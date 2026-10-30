import errno
import hashlib
import os
import io
import tarfile
import time
from unittest.mock import patch

import pytest

from openpilot.selfdrive.turnpilot.maps import corridor
from openpilot.selfdrive.turnpilot.maps.client import MapError
from openpilot.selfdrive.turnpilot.maps.tests.fakes import make_pack

FILES = {'tiles/2/000/123/456.gph': b'level two tile', 'tiles/0/000/003.gph': b'level zero', 'tiles/1/000/045.gph': b'one'}


def unpack(tmp_path, data):
  tar = tmp_path / 'pack.tar'
  tar.write_bytes(data)
  return corridor.unpack(tar, tmp_path / 'packs')


def rejected(tmp_path, data, words):
  with pytest.raises(MapError) as e:
    unpack(tmp_path, data)
  assert e.value.code == 'bad_pack' and words in e.value.message, e.value.message
  packs = tmp_path / 'packs'
  assert not packs.exists() or not any(packs.iterdir()), "a rejected pack leaves nothing behind"
  return e.value


def member(name, data=b'x', kind=tarfile.REGTYPE, link=''):
  def build():
    info = tarfile.TarInfo(name)
    info.type, info.linkname = kind, link
    info.size = len(data) if kind == tarfile.REGTYPE else 0
    return info, data if kind == tarfile.REGTYPE else None
  return build


def test_good_pack(tmp_path):
  pack = unpack(tmp_path, make_pack(FILES))
  assert pack.manifest['region'] == 'indianapolis'
  for name, data in FILES.items():
    assert (pack.path / name).read_bytes() == data
  assert (pack.tiles / '2/000/123/456.gph').is_file()
  assert corridor.packs(tmp_path / 'packs') == [pack.path]


def test_directory_members_are_fine(tmp_path):
  pack = unpack(tmp_path, make_pack(FILES, extra=[member('tiles/', kind=tarfile.DIRTYPE), member('tiles/2', kind=tarfile.DIRTYPE)]))
  assert pack.path.is_dir()


def test_bad_checksum(tmp_path):
  listed = {n: hashlib.sha256(d).hexdigest() for n, d in FILES.items()}
  listed['tiles/1/000/045.gph'] = hashlib.sha256(b'something else').hexdigest()
  rejected(tmp_path, make_pack(FILES, manifest_files=listed), 'checksum mismatch for tiles/1/000/045.gph')


def test_extra_file(tmp_path):
  listed = {n: hashlib.sha256(d).hexdigest() for n, d in FILES.items() if n != 'tiles/0/000/003.gph'}
  rejected(tmp_path, make_pack(FILES, manifest_files=listed), 'unlisted file tiles/0/000/003.gph')
  rejected(tmp_path, make_pack(FILES, extra=[member('readme.txt')]), "unexpected file 'readme.txt'")


def test_missing_file(tmp_path):
  listed = {**{n: hashlib.sha256(d).hexdigest() for n, d in FILES.items()}, 'tiles/2/000/999.gph': '0' * 64}
  rejected(tmp_path, make_pack(FILES, manifest_files=listed), 'missing tiles/2/000/999.gph')


@pytest.mark.parametrize('name', ['../evil.gph', 'tiles/../../evil.gph', 'tiles/2/../../../evil', '/etc/passwd', '/tiles/2/0.gph',
                                  'tiles\\..\\evil', 'C:/tiles/x'])
def test_path_traversal_in_the_tar(tmp_path, name):
  rejected(tmp_path, make_pack(FILES, extra=[member(name)]), 'unsafe path')
  assert not (tmp_path / 'evil.gph').exists() and not (tmp_path.parent / 'evil.gph').exists()


def test_path_traversal_in_the_manifest(tmp_path):
  listed = {**{n: hashlib.sha256(d).hexdigest() for n, d in FILES.items()}, '../evil.gph': '0' * 64}
  rejected(tmp_path, make_pack(FILES, manifest_files=listed), 'unsafe path')


@pytest.mark.parametrize('kind,link', [(tarfile.SYMTYPE, '/etc/passwd'), (tarfile.LNKTYPE, 'tiles/0/000/003.gph'),
                                       (tarfile.FIFOTYPE, ''), (tarfile.CHRTYPE, '')])
def test_links_and_devices(tmp_path, kind, link):
  rejected(tmp_path, make_pack(FILES, extra=[member('tiles/2/link.gph', kind=kind, link=link)]), 'not a regular file')


def test_duplicate_member(tmp_path):
  rejected(tmp_path, make_pack(FILES, extra=[member('tiles/1/000/045.gph', b'one')]), 'appears twice')


def test_no_or_bad_manifest(tmp_path):
  rejected(tmp_path, make_pack(FILES, manifest=False), 'no manifest')
  buf = io.BytesIO()
  with tarfile.open(fileobj=buf, mode='w') as tar:
    info = tarfile.TarInfo('manifest.json')
    info.size = 5
    tar.addfile(info, io.BytesIO(b'nope!'))
  rejected(tmp_path, buf.getvalue(), 'manifest is not valid')
  rejected(tmp_path, b'this is not a tar file at all' * 30, 'not a tar file')
  listed = dict.fromkeys(FILES, 'xyz')
  rejected(tmp_path, make_pack(FILES, manifest_files=listed), 'not SHA-256')


def test_prune_keeps_the_newest_and_the_one_in_use(tmp_path):
  made = []
  for i in range(5):
    made.append(unpack(tmp_path, make_pack({'tiles/2/0.gph': bytes([i])})).path)
    time.sleep(0.01)
  packs_dir = tmp_path / 'packs'
  assert corridor.packs(packs_dir) == made  # oldest first, whatever their names
  (packs_dir / '.staging-dead').mkdir()  # left by a crash mid-unpack
  os.utime(packs_dir / '.staging-dead', (1, 1))
  (packs_dir / '.staging-live').mkdir()  # another route job's unpack in progress
  (packs_dir / '.download-dead.tar').write_bytes(b'cut off')
  os.utime(packs_dir / '.download-dead.tar', (1, 1))
  (packs_dir / '.download-live.tar').write_bytes(b'still coming')  # another download in progress
  corridor.prune(packs_dir, keep=3, in_use=made[0])
  assert corridor.packs(packs_dir) == [made[0]] + made[2:]
  assert not (packs_dir / '.staging-dead').exists() and not (packs_dir / '.download-dead.tar').exists()
  assert (packs_dir / '.download-live.tar').exists() and (packs_dir / '.staging-live').exists()
  corridor.prune(packs_dir, keep=2)
  assert corridor.packs(packs_dir) == made[3:]


BIG = {f'tiles/2/000/000/{i:03d}.gph': os.urandom(3000) for i in range(4)}


@pytest.mark.parametrize('compress', [False, True])
@pytest.mark.parametrize('keep', [0.2, 0.4, 0.55])  # the tiles end at about 0.6, then come the tar's zero records
def test_a_cut_off_pack_is_a_retryable_bad_pack(tmp_path, compress, keep):
  """A download that broke off (no Content-Length, a clean FIN) must be a MapError the router can back off on, not a
  raw tarfile.ReadError or EOFError."""
  import gzip
  data = make_pack(BIG)
  data = gzip.compress(data) if compress else data
  e = rejected(tmp_path, data[:int(len(data) * keep)], 'failed its check')
  assert e.retry


def test_a_file_and_a_directory_at_one_path_is_a_bad_pack_not_retried(tmp_path):
  """The server's pack is wrong the same way on every download: it waits out the refusal time, not 30 s."""
  clash = make_pack({'tiles/2/000/000/001.gph': b'a'}, extra=[member('tiles/2/000/000/001.gph/x.gph')],
                    manifest_files={'tiles/2/000/000/001.gph': hashlib.sha256(b'a').hexdigest(),
                                    'tiles/2/000/000/001.gph/x.gph': hashlib.sha256(b'x').hexdigest()})
  e = rejected(tmp_path, clash, 'is a file and a directory')
  assert not e.retry


@pytest.mark.parametrize('error', [errno.ENOSPC, errno.EIO])
def test_a_pack_the_comma_cannot_save_is_a_storage_error_not_a_bad_map(tmp_path, error):
  """A full disk is not a bad map, and downloading it again every 30 s won't help."""
  real_open = open

  def failing(path, mode='r', *args, **kwargs):
    if 'x' in mode:
      raise OSError(error, os.strerror(error))
    return real_open(path, mode, *args, **kwargs)

  with patch('builtins.open', failing), pytest.raises(MapError) as e:
    unpack(tmp_path, make_pack(FILES))
  assert (e.value.code, e.value.retry) == ('storage', False)
  assert ('storage is full' in e.value.message) == (error == errno.ENOSPC)
  assert not any((tmp_path / 'packs').iterdir()), "nothing is left behind"


def test_a_corrupt_gzip_is_retried(tmp_path):
  """gzip.BadGzipFile (a bad CRC, say) is an OSError, but it is a bad download, not the comma's disk."""
  import gzip
  with patch.object(corridor, 'unpack_to', side_effect=gzip.BadGzipFile('CRC check failed')):
    e = rejected(tmp_path, make_pack(FILES), 'failed its check')
  assert e.retry


def test_a_pack_covers_the_places_its_level_2_tiles_do(tmp_path):
  from openpilot.selfdrive.turnpilot.maps.tests.fakes import INDY, INDY_EAST, level2_tile
  from tools.turnpilot.mapserver import tiles
  name = level2_tile(*INDY)
  assert tiles.tile_bbox(*tiles.parse_path(name.removeprefix('tiles/'))) == (-86.25, 39.75, -86.0, 40.0)  # as the server's grid
  pack = unpack(tmp_path, make_pack({name: b'two', 'tiles/0/002/813.gph': b'zero', 'tiles/1/045/876.gph': b'one'}))
  assert pack.covers(*INDY) and pack.covers(-86.01, 39.99)
  assert not pack.covers(-86.4, 40.3)  # inside the 1 and 4 degree level 0 and 1 tiles, which cover nearly anything
  assert not pack.covers(INDY_EAST[0] + 0.3, INDY_EAST[1])


def test_prune_and_packs_ignore_files_another_job_removes(tmp_path, monkeypatch):
  import errno
  from openpilot.selfdrive.turnpilot.maps import corridor
  (tmp_path / '.download-gone.tar').write_bytes(b'x')
  (tmp_path / '.staging-gone').mkdir()
  real_stat = corridor.Path.stat

  def vanishing_stat(self, *a, **kw):
    if self.name in ('.download-gone.tar', '.staging-gone') or self.name == corridor.MANIFEST:
      raise FileNotFoundError(errno.ENOENT, "removed by another job", str(self))
    return real_stat(self, *a, **kw)
  pack = tmp_path / '20260101-000000-abcdef'
  pack.mkdir()
  (pack / corridor.MANIFEST).write_text('{}')
  monkeypatch.setattr(corridor.Path, 'stat', vanishing_stat)
  assert corridor.packs(tmp_path) == []
  corridor.prune(tmp_path)


def test_a_path_component_longer_than_a_filename_is_a_bad_pack():
  import pytest
  from openpilot.selfdrive.turnpilot.maps.client import MapError
  from openpilot.selfdrive.turnpilot.maps.corridor import member_path
  with pytest.raises(MapError) as e:
    member_path('tiles/2/' + 'a' * 300 + '.gph')
  assert e.value.code == 'bad_pack'


SHARD = 'osm_controls/756425.json'


def test_sidecar_shards_unpack_with_their_checksums(tmp_path):
  """The OSM stop-sign sidecar's shards (routing/osm_sidecar.py) come in the pack beside the tiles, listed with their
  SHA-256 like the tiles; they say nothing about where the pack covers."""
  files = {**FILES, SHARD: b'{"osm_timestamp": "t"}'}
  pack = unpack(tmp_path, make_pack(files, manifest_extra={'osm_timestamp': 't'}))
  assert (pack.path / SHARD).read_bytes() == b'{"osm_timestamp": "t"}'
  assert pack.level2 == frozenset({123456}) and pack.manifest['osm_timestamp'] == 't'


def test_a_sidecar_shard_with_a_bad_checksum_rejects_the_pack(tmp_path):
  files = {**FILES, SHARD: b'{}'}
  listed = {n: hashlib.sha256(d).hexdigest() for n, d in files.items()}
  listed[SHARD] = hashlib.sha256(b'something else').hexdigest()
  rejected(tmp_path, make_pack(files, manifest_files=listed), f'checksum mismatch for {SHARD}')


@pytest.mark.parametrize('name', ['osm_controls/x.json', 'osm_controls/1.txt', 'osm_controls/1/2.json', 'osm_controls/-1.json'])
def test_only_numbered_shards_under_osm_controls(tmp_path, name):
  rejected(tmp_path, make_pack({**FILES, name: b'{}'}), 'unexpected file')
