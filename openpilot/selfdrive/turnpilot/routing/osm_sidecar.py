"""The OSM stop-sign sidecar on the car: the shards for a route's level-2 tiles, checked against the tiles' own stamp.

tools/turnpilot/mapserver/osm_controls.py writes, beside a region's tiles, osm_controls/<level-2 tile index>.json for every
level-2 tile, stamped with the osm_timestamp of the OSM data the tiles were built from. A corridor carries the shards of its
own tiles (maps/corridor.py) and the region's osm_timestamp in its manifest.json; a region's or the sim tiles' directory
(TurnPilotValhallaTiles) has it in meta.json. Each shard holds
  controls: {"<way_id>:<f|b>": [{kind stop|yield, line [lat, lon] or null, junction_node (0: none), stop_tag, evidence,
                                 node}, ...]}
  junctions: {"<osm node id>": {pos [lat, lon], approaches [...]}}
  rail: [{lat, lon, ways}]
  unplaced: [{node, kind, lat, lon, direction, stop_tag, why}]: the stop and give_way nodes no approach takes (route_audit)
routing/controls.py unions its stops and yields with Valhalla's flags (STOP_SIGNS.md, SG1a).

load() reads the shards of every level-2 tile a route's shape passes through (each must be there: the build writes one per
tile, empty or not) and of the tiles within NEAR_DEG of it (read if there: a line on the route by a tile's edge). Its
status: ok; missing (no directory, no osm_controls/, or a shard of the route's tiles not there or unreadable: a corridor from
a server before the sidecar, or tiles built without it); stale (a shard's osm_timestamp isn't the tiles', or the tiles have
none to check it against). Anything but ok means the stops Valhalla doesn't flag are unknown, which controls.py says out
loud. Shards are cached by file (mtime and size, CACHE_MAX of them), since navd looks up the road ahead every second.
"""
import json
import threading
from dataclasses import dataclass, field
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.maps.corridor import SHARDS, level2_index

STAMP_FILES = ('manifest.json', 'meta.json')  # a corridor's; a region's or the sim tiles'
NO_DIRECTORY = ''   # sources.py: a sidecar is expected, but the tiles have no directory (a valhalla_service URL)
SAMPLE_M = 25.      # m between the route points whose tiles must have a shard
NEAR_DEG = 0.0006   # deg (about 50-65 m) around them: the tiles read too, when there
M_PER_DEG = 111195.
CACHE_MAX = 256     # shards kept parsed: a few corridors' worth; the oldest read goes first

_cache: dict[Path, tuple[int, int, dict | None]] = {}
_lock = threading.Lock()


@dataclass
class OsmSidecar:
  status: str                 # 'ok', 'missing' or 'stale'
  detail: str = ''
  controls: dict[str, list[dict]] = field(default_factory=dict)
  junctions: dict[int, dict] = field(default_factory=dict)
  rail: list[dict] = field(default_factory=list)
  unplaced: list[dict] = field(default_factory=list)

  def entries(self, key: str) -> list[dict]:
    return self.controls.get(key, [])

  def junction_pos(self, node: int) -> tuple[float, float] | None:
    j = self.junctions.get(int(node))
    return (float(j['pos'][0]), float(j['pos'][1])) if j and j.get('pos') else None


def tiles_stamp(directory: Path) -> str | None:
  """The osm_timestamp of the data the tiles in directory were built from, or None."""
  for name in STAMP_FILES:
    path = Path(directory) / name
    if path.is_file():
      try:
        stamp = json.loads(path.read_text()).get('osm_timestamp')
      except (OSError, ValueError, AttributeError):
        return None
      return stamp if isinstance(stamp, str) and stamp else None
  return None


def samples(points) -> list[tuple[float, float]]:
  """The points (lat, lon), with more between any two further apart than SAMPLE_M."""
  out = []
  for a, b in zip(points, points[1:], strict=False):
    d = abs(b[0] - a[0]) * M_PER_DEG + abs(b[1] - a[1]) * M_PER_DEG
    n = max(1, int(d // SAMPLE_M) + 1)
    out.extend((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n))
  return out + list(points[-1:])


def cells(points) -> tuple[set[int], set[int]]:
  """(the level-2 tiles the route passes through, the ones within NEAR_DEG of it besides)."""
  on, near = set(), set()
  for lat, lon in samples(points):
    on.add(level2_index(lon, lat))
    for dlat in (-NEAR_DEG, 0., NEAR_DEG):
      for dlon in (-NEAR_DEG, 0., NEAR_DEG):
        near.add(level2_index(lon + dlon, lat + dlat))
  return on, near - on


def read_shard(path: Path) -> dict | None:
  """The shard at path, or None when it isn't there or doesn't read (logged)."""
  try:
    st = path.stat()
  except OSError:
    return None
  with _lock:
    hit = _cache.get(path)
  if hit is not None and hit[:2] == (st.st_mtime_ns, st.st_size):
    return hit[2]
  try:
    shard = json.loads(path.read_text())
    if not isinstance(shard, dict) or not isinstance(shard.get('controls'), dict):
      raise ValueError("not a shard")
  except (OSError, ValueError) as e:
    cloudlog.error(f"turnpilot osm sidecar: {path} doesn't read: {type(e).__name__}: {e}")
    shard = None
  with _lock:
    _cache.pop(path, None)
    _cache[path] = (st.st_mtime_ns, st.st_size, shard)
    while len(_cache) > CACHE_MAX:
      del _cache[next(iter(_cache))]
  return shard


def load(sidecar_dir, points) -> OsmSidecar:
  """The sidecar for a route's shape points [(lat, lon), ...] from sidecar_dir (the directory holding osm_controls/ and
  the tiles' stamp), or NO_DIRECTORY."""
  if sidecar_dir is None or str(sidecar_dir) == NO_DIRECTORY:
    return OsmSidecar('missing', "the tiles have no directory to hold it")
  root = Path(sidecar_dir)
  if not (root / SHARDS).is_dir():
    return OsmSidecar('missing', f"no {SHARDS}/ in {root}")
  stamp = tiles_stamp(root)
  on, near = cells(points)
  out = OsmSidecar('ok')
  absent, stale = [], []
  for cell in sorted(on | near):
    shard = read_shard(root / SHARDS / f'{cell}.json')
    if shard is None:
      if cell in on:
        absent.append(cell)
      continue
    if stamp is None or shard.get('osm_timestamp') != stamp:
      stale.append((cell, shard.get('osm_timestamp')))
    for key, entries in shard['controls'].items():
      out.controls.setdefault(key, []).extend(entries)
    for node, j in (shard.get('junctions') or {}).items():
      out.junctions[int(node)] = j
    out.rail.extend(shard.get('rail') or [])
    out.unplaced.extend(shard.get('unplaced') or [])
  if absent:
    out.status, out.detail = 'missing', f"no shard for tile {absent[0]}" + (f" and {len(absent) - 1} more" if len(absent) > 1 else '')
  elif stamp is None:
    out.status, out.detail = 'stale', f"the tiles in {root} have no osm_timestamp to check it against"
  elif stale:
    out.status, out.detail = 'stale', f"tile {stale[0][0]}'s shard is from {stale[0][1]}, the tiles from {stamp}"
  return out
