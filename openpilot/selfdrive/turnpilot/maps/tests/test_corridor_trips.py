"""corridor's trip record: which drives a saved pack serves (serves), and touch, which never moves a pack back in time."""
import os
import time

from openpilot.selfdrive.turnpilot.maps import corridor
from openpilot.selfdrive.turnpilot.maps.corridor import REUSE_M, Pack, meters

A, B = (0.02, 0.02), (0.105, 0.05)  # 10 km apart, in one level-2 tile
OFF = (-0.1, 0.02)  # in the next tile west
ON = (0.06, 0.03)  # between them, on the same tile


def tile_name(lon, lat) -> str:
  digits = f"{corridor.level2_index(lon, lat):09d}"
  return f"tiles/2/{digits[:3]}/{digits[3:6]}/{digits[6:]}.gph"


def pack_on(tmp_path, point) -> Pack:
  path = tmp_path / 'pack'
  path.mkdir()
  (path / corridor.MANIFEST).write_text('{}')
  return Pack(path, {'files': {tile_name(*point): '0' * 64}})


def east(point, m):
  """point moved m metres east."""
  return (point[0] + m / 111319.49, point[1])  # at the equator


def test_the_same_trip_from_anywhere_on_its_roads_and_the_way_back(tmp_path):
  pack = pack_on(tmp_path, A)
  assert pack.covers(*B) and not pack.covers(*OFF)
  assert 0.95 * REUSE_M < meters(B, east(B, REUSE_M)) < 1.05 * REUSE_M
  assert corridor.serves(pack, A, B, A, B)
  assert corridor.serves(pack, A, B, ON, east(B, 0.9 * REUSE_M))  # its destination, from elsewhere on it
  assert corridor.serves(pack, A, B, east(B, 0.9 * REUSE_M), east(A, -0.9 * REUSE_M))  # the way back, both ends near
  assert not corridor.serves(pack, A, B, B, east(A, 1.2 * REUSE_M))  # home is somewhere else
  assert not corridor.serves(pack, A, B, ON, A)  # back to its start, but not from its destination
  assert not corridor.serves(pack, A, B, A, east(B, 1.2 * REUSE_M))  # another destination
  assert not corridor.serves(pack, A, B, OFF, B)  # a start off its roads


def test_touch_marks_a_pack_used_and_never_moves_it_back(tmp_path):
  pack = pack_on(tmp_path, A)
  manifest = pack.path / corridor.MANIFEST
  old = time.time_ns() - 86400 * 10**9
  os.utime(manifest, ns=(old, old))
  corridor.touch(pack)
  assert manifest.stat().st_mtime_ns > old + 86000 * 10**9
  ahead = time.time_ns() + 86400 * 10**9  # written while the clock was right; now it is behind, as after a boot
  os.utime(manifest, ns=(ahead, ahead))
  corridor.touch(pack)
  assert manifest.stat().st_mtime_ns == ahead
