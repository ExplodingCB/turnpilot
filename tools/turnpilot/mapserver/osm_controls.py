#!/usr/bin/env python3
"""The OSM stop-sign sidecar: every stop sign, yield and level crossing in a region's OSM data, by the approach it
governs, sharded on Valhalla's level-2 grid so a corridor carries the shards for its own tiles.

  osm_controls.py <region.osm.pbf | export.osm> <out_dir> --timestamp T --tiles DIR

Valhalla sets no stop_sign flag on about a quarter of the OSM-mapped stop approaches (research_allway.md section 0), drops
stop=all, and puts a phantom stop at the far end of a stop it can't direct. navd's routing/controls.py unions these shards
with Valhalla's flags (STOP_SIGNS.md, SG1a). Written by build_region.py right after build_valhalla_tiles.sh, from the same
PBF, so the way ids match the tiles; build_sim_tiles.sh runs it on the sim level's export.

Writes <out_dir>/osm_controls/<level-2 tile index>.json for EVERY level-2 tile under --tiles (empty or not), so on the car
a missing shard is never mistaken for a tile with no stops. Each shard:
  {"version", "osm_timestamp" (the tiles' own stamp: the car refuses, out loud, a shard whose stamp differs), "tile",
   "controls": {"<way_id>:<f|b>": [{kind stop|yield, line [lat, lon] or null, junction_node, stop_tag all|minor|none,
                                    evidence}, ...]},
   "junctions": {"<osm node id>": {pos [lat, lon], approaches [{way_id, dir f|b, bearing_deg, class, maxspeed, oneway,
                                   stops, kind, evidence, line}, ...]}},
   "rail": [{lat, lon, ways [way_id, ...]}]}
f: traffic travelling along the way's node order (Valhalla's edge.forward), b: against it. A control sits in the shard of
its line (of its junction when it has no line), a junction and a crossing in the shard of their position.

The junction model (from stopsigns/scripts/allway_osm.py):
- drivable ways: DRIVABLE highway classes, not area=yes, not closed to motor vehicles. A junction is a node with at least
  3 drivable legs (2 where a way passes through, 1 where it ends). An approach is (way, f) arriving along the way's order
  or (way, b) against it; a oneway gives only its legal arrival.
- a stop or give_way on a junction node applies to every approach (evidence node_all with stop=all, node_untagged
  without), or with stop=minor only to the approaches of a lower class than the junction's top one (node_minor, as
  Valhalla does); it has no line (null): the node is the junction's middle, not where anyone stops.
- anywhere else (mid-way, or on a node shared only with a footway or a way's continuation) it governs the first junction
  within MAXD m along its own way: direction=forward the one ahead along the way (mid_directed), backward the one behind,
  both each of them (mid_both), and with no or another direction the nearest (mid_undirected, by proximity). Its node is
  the line. A stop with no junction within MAXD along its way is left out and counted (stats).
- an approach's bearing_deg is the bearing of the leg leaving the junction (deg from north, as Valhalla's begin_heading),
  over BEARING_M of it; maxspeed is m/s from the way's maxspeed tag ("25 mph", "40", "40 km/h"), null when untagged or
  unreadable; stops/kind/evidence/line the control on that approach, if any (a mid-way line before a node's).

Needs osmium-tool on the build machine. The PBF pass takes about a minute for the Indianapolis region on the Mac.
"""
import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict
from collections.abc import Callable, Iterable
from pathlib import Path

try:
  from tools.turnpilot.mapserver import tiles
except ImportError:  # run as a plain script beside tiles.py
  sys.path.insert(0, str(Path(__file__).resolve().parent))
  import tiles  # type: ignore[no-redef]

VERSION = 1
SHARDS_DIR = 'osm_controls'
DRIVABLE = {'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link', 'secondary', 'secondary_link',
            'tertiary', 'tertiary_link', 'unclassified', 'residential', 'living_street', 'service', 'road'}
NOT_INGESTED = {'proposed', 'construction', 'abandoned', 'disused', 'razed', 'planned', 'platform', 'rest_area', 'services',
                'elevator', 'bus_stop', 'emergency_bay', 'no'}
RANK = {'motorway': 0, 'trunk': 1, 'primary': 2, 'secondary': 3, 'tertiary': 4, 'unclassified': 5, 'road': 5,
        'residential': 6, 'living_street': 6, 'service': 7}
EVIDENCE_ORDER = ('mid_directed', 'mid_both', 'mid_undirected', 'node_all', 'node_minor', 'node_untagged')
MAXD = 60.      # m along a way from a mid-way stop node to the junction it governs
BEARING_M = 15.  # m of a leg its bearing is taken over (Valhalla's kMinMetersOffsetForHeading)
EARTH_R = 6371000.
MPH = 0.44704
ESC = re.compile(r'%([0-9a-fA-F]+)%')
NODE = re.compile(r'n(\d+)x([-0-9.]*)y([-0-9.]*)')
NODE_ID = re.compile(r'n(\d+)')
KINDS = {'stop': 'stop', 'give_way': 'yield'}


def unesc(s: str) -> str:
  return ESC.sub(lambda m: chr(int(m.group(1), 16)), s)


def parse_tags(field: str) -> dict[str, str]:
  out = {}
  for kv in field.split(',') if field else []:
    if '=' in kv:
      k, v = kv.split('=', 1)
      out[unesc(k)] = unesc(v)
  return out


def fields(line: str) -> dict[str, str]:
  return {tok[0]: tok[1:] for tok in line.rstrip('\n').split(' ') if tok}


def rank(hw: str) -> int:
  return RANK.get(hw.replace('_link', ''), 8)


def dist(a, b) -> float:
  (la1, lo1), (la2, lo2) = a, b
  x = math.radians(lo2 - lo1) * math.cos(math.radians((la1 + la2) / 2))
  return EARTH_R * math.hypot(x, math.radians(la2 - la1))


def bearing(a, b) -> float:
  x = math.radians(b[1] - a[1]) * math.cos(math.radians((a[0] + b[0]) / 2))
  return math.degrees(math.atan2(x, math.radians(b[0] - a[0]))) % 360.


def drivable(t: dict) -> bool:
  if t.get('highway', '') not in DRIVABLE or t.get('area') == 'yes':
    return False
  if t.get('access') == 'no' and t.get('motor_vehicle') not in ('yes', 'destination') and t.get('motorcar') not in ('yes', 'destination'):
    return False
  return t.get('motor_vehicle') != 'no' and t.get('motorcar') != 'no'


def oneway(t: dict) -> int:
  o = t.get('oneway', '')
  if o in ('yes', 'true', '1'):
    return 1
  if o in ('-1', 'reverse'):
    return -1
  if o == 'no':
    return 0
  return 1 if t.get('junction') in ('roundabout', 'circular') or t.get('highway') == 'motorway' else 0


def maxspeed(tag: str | None) -> float | None:
  """m/s from a maxspeed tag, or None."""
  m = re.fullmatch(r'\s*(\d+(?:\.\d+)?)\s*(mph|km/h|kmh|kph)?\s*', tag or '')
  if m is None:
    return None
  v = float(m.group(1))
  return v * MPH if m.group(2) == 'mph' else v / 3.6


def level2_index(lon: float, lat: float) -> int:
  size = tiles.LEVEL_SIZE[2]
  return int((lat + 90) // size) * round(360 / size) + int((lon + 180) // size)


def rounded(p) -> list[float]:
  return [round(p[0], 7), round(p[1], 7)]


def key(way: int, step: int) -> str:
  return f"{way}:{'f' if step > 0 else 'b'}"


def build_sidecar(control_lines: Iterable[str], way_lines: Callable[[], Iterable[str]]) -> dict:
  """The sidecar from osmium OPL: control_lines the stop, give_way and level-crossing nodes; way_lines() a fresh pass
  over the highway ways with their node locations (it is called twice). {'controls', 'junctions' (by int node id),
  'rail', 'stats'}."""
  controls_in = {}  # node id -> (lat, lon, tags)
  rail_pos = {}     # node id -> (lat, lon)
  for line in control_lines:
    if not line.startswith('n'):
      continue
    f = fields(line)
    try:
      nid, t = int(f['n']), parse_tags(f.get('T', ''))
      p = (float(f['y']), float(f['x']), t)
    except (KeyError, ValueError):
      continue
    if t.get('highway') in KINDS:
      controls_in[nid] = p
    if t.get('railway') == 'level_crossing':
      rail_pos[nid] = p[:2]
  wanted = set(controls_in) | set(rail_pos)

  # pass A: the nodes of every drivable way carrying a control or a crossing
  C = set()
  for line in way_lines():
    if not line.startswith('w'):
      continue
    f = fields(line)
    t = parse_tags(f.get('T', ''))
    if not drivable(t):
      continue
    ids = [int(x) for x in NODE_ID.findall(f.get('N', ''))]
    if wanted.intersection(ids):
      C.update(ids)
  # pass B: every ingested way's occurrences of those nodes; the geometry of the drivable ways touching them
  occ = defaultdict(list)  # node -> [(way, index, way length in nodes, drivable)]
  W = {}                   # way -> (tags, [(node id, lat, lon)])
  for line in way_lines():
    if not line.startswith('w'):
      continue
    f = fields(line)
    t = parse_tags(f.get('T', ''))
    hw = t.get('highway', '')
    if not hw or hw in NOT_INGESTED:
      continue
    nf = f.get('N', '')
    ids = [int(x) for x in NODE_ID.findall(nf)]
    hit = [i for i, n in enumerate(ids) if n in C]
    if not hit:
      continue
    d = drivable(t)
    wid = int(f['w'])
    for i in hit:
      occ[ids[i]].append((wid, i, len(ids), d))
    if d:
      nodes = [(int(n), float(y) if y else math.nan, float(x) if x else math.nan) for n, x, y in NODE.findall(nf)]
      W[wid] = ({k: t[k] for k in ('highway', 'oneway', 'junction', 'service', 'maxspeed') if k in t}, nodes)

  def legs_at(n):
    out = []
    for wid, i, L, d in occ.get(n, []):
      if not d or wid not in W:
        continue
      nodes = W[wid][1]
      closed = nodes[0][0] == nodes[-1][0] and L > 2
      if i > 0 or closed:
        out.append((wid, +1, i))
      if i < L - 1 or closed:
        out.append((wid, -1, i))
    return out

  junc = {}

  def junction(n) -> bool:
    if n not in junc:
      junc[n] = len(legs_at(n)) >= 3
    return junc[n]

  def approaches(n):
    """[(way, step, rank, index of n in the way)] for every leg traffic may arrive at n on"""
    out = []
    for wid, step, i in legs_at(n):
      ow = oneway(W[wid][0])
      if (step == +1 and ow == -1) or (step == -1 and ow == 1):
        continue
      out.append((wid, step, rank(W[wid][0]['highway']), i))
    return out

  def walk(wid, i, step):
    """(junction node, m along) for the first junction from index i along the way in direction step within MAXD"""
    nodes = W[wid][1]
    d, j = 0., i
    while 0 <= j + step < len(nodes):
      a, b = nodes[j], nodes[j + step]
      d += dist(a[1:], b[1:])
      j += step
      if d > MAXD:
        return None
      if junction(b[0]):
        return b[0], d
    return None

  stats = Counter()
  controls = defaultdict(list)                # key -> [entry]
  on_approach = defaultdict(dict)             # junction -> {(way, step): entry}
  for sid, (la, lo, t) in sorted(controls_in.items()):
    kind = KINDS[t['highway']]
    stop_tag = t.get('stop' if kind == 'stop' else 'give_way', '')
    stop_tag = stop_tag if stop_tag in ('all', 'minor') else 'none'
    dways = [(w, i) for w, i, _, d in occ.get(sid, []) if d and w in W]
    if not dways:
      stats['not_on_a_drivable_way'] += 1
      continue
    if junction(sid):
      stats['junction_node'] += 1
      apps = approaches(sid)
      top = min((r for *_, r, _ in apps), default=8)
      evidence = 'node_minor' if stop_tag == 'minor' else 'node_all' if stop_tag == 'all' else 'node_untagged'
      for w, step, r, _ in apps:
        if stop_tag == 'minor' and r <= top:
          continue
        entry = {'kind': kind, 'line': None, 'junction_node': sid, 'stop_tag': stop_tag, 'evidence': evidence}
        controls[key(w, step)].append(entry)
        on_approach[sid].setdefault((w, step), entry)
      continue
    w, i = dways[0]
    direction = t.get('direction', '')
    fwd, bwd = walk(w, i, +1), walk(w, i, -1)
    if direction == 'forward':
      targets, evidence = [(fwd, +1)] if fwd else [], 'mid_directed'
    elif direction in ('backward', 'reverse'):
      targets, evidence = [(bwd, -1)] if bwd else [], 'mid_directed'
    elif direction == 'both':
      targets, evidence = [x for x in ((fwd, +1), (bwd, -1)) if x[0]], 'mid_both'
    else:
      near = [x for x in ((fwd, +1), (bwd, -1)) if x[0]]
      targets, evidence = ([min(near, key=lambda x: x[0][1])] if near else []), 'mid_undirected'
    if not targets:
      stats['no_junction_within_maxd'] += 1
      continue
    stats[evidence] += 1
    for (jn, _), step in targets:
      entry = {'kind': kind, 'line': rounded((la, lo)), 'junction_node': jn, 'stop_tag': stop_tag, 'evidence': evidence}
      controls[key(w, step)].append(entry)
      prev = on_approach[jn].get((w, step))
      if prev is None or prev['line'] is None:  # a mid-way line before a node's
        on_approach[jn][(w, step)] = entry

  junctions = {}
  for jn, governed in on_approach.items():
    out = []
    for w, step, _, i in approaches(jn):
      tags, nodes = W[w]
      # the leg leaves the junction the way the traffic on it came from: against step
      far, d, j = nodes[i], 0., i
      while 0 <= j - step < len(nodes) and d < BEARING_M:
        d += dist(nodes[j][1:], nodes[j - step][1:])
        j -= step
        far = nodes[j]
      entry = governed.get((w, step))
      out.append({'way_id': w, 'dir': 'f' if step > 0 else 'b', 'bearing_deg': round(bearing(nodes[i][1:], far[1:]), 1),
                  'class': tags['highway'], 'maxspeed': maxspeed(tags.get('maxspeed')), 'oneway': oneway(tags) != 0,
                  'stops': entry is not None, 'kind': entry['kind'] if entry else None,
                  'evidence': entry['evidence'] if entry else 'none', 'line': entry['line'] if entry else None})
    pos = next(((la, lo) for w, *_ in approaches(jn) for n, la, lo in W[w][1] if n == jn), None)
    if pos is not None:
      junctions[jn] = {'pos': rounded(pos), 'approaches': sorted(out, key=lambda a: (a['way_id'], a['dir']))}

  rail = []
  for rid, (la, lo) in sorted(rail_pos.items()):
    ways = sorted({w for w, _, _, d in occ.get(rid, []) if d and w in W})
    if ways:  # a crossing on a drivable road
      rail.append({'lat': round(la, 7), 'lon': round(lo, 7), 'ways': ways})
  return {'controls': dict(sorted(controls.items())), 'junctions': junctions, 'rail': rail, 'stats': dict(stats)}


def shards(sidecar: dict, cells: set[int], osm_timestamp: str) -> dict[int, dict]:
  """{level-2 tile: shard} for every tile in cells; what falls in no tile there is left out (counted in the stats)."""
  out = {c: {'version': VERSION, 'osm_timestamp': osm_timestamp, 'tile': c, 'controls': {}, 'junctions': {}, 'rail': []}
         for c in sorted(cells)}
  dropped = 0
  junctions = sidecar['junctions']
  for k, entries in sidecar['controls'].items():
    for e in entries:
      at = e['line'] or (junctions.get(e['junction_node']) or {}).get('pos')
      cell = level2_index(at[1], at[0]) if at else None
      if cell in out:
        out[cell]['controls'].setdefault(k, []).append(e)
      else:
        dropped += 1
  for jn, j in junctions.items():
    cell = level2_index(j['pos'][1], j['pos'][0])
    if cell in out:
      out[cell]['junctions'][str(jn)] = j
    else:
      dropped += 1
  for r in sidecar['rail']:
    cell = level2_index(r['lon'], r['lat'])
    if cell in out:
      out[cell]['rail'].append(r)
    else:
      dropped += 1
  sidecar.setdefault('stats', {})['outside_the_tiles'] = dropped
  return out


def tile_cells(tile_dir: Path) -> set[int]:
  """The level-2 tiles under a Valhalla tile hierarchy, by index."""
  cells = set()
  for p in (Path(tile_dir) / '2').rglob('*.gph'):
    parsed = tiles.parse_path(p.relative_to(tile_dir).as_posix())
    if parsed is not None:
      cells.add(parsed[1])
  return cells


def write_shards(out_dir: Path, shard_map: dict[int, dict]) -> Path:
  """Write the shards to out_dir/osm_controls, replacing what is there in one rename (beside a running stack that
  reads it, the old shards stay whole until the new ones are all written)."""
  final = Path(out_dir) / SHARDS_DIR
  staging = Path(out_dir) / f".{SHARDS_DIR}.new"
  shutil.rmtree(staging, ignore_errors=True)
  staging.mkdir(parents=True)
  for cell, shard in shard_map.items():
    (staging / f"{cell}.json").write_text(json.dumps(shard, separators=(',', ':'), sort_keys=True))
  old = Path(out_dir) / f".{SHARDS_DIR}.old"
  shutil.rmtree(old, ignore_errors=True)
  if final.exists():
    final.rename(old)
  staging.rename(final)
  shutil.rmtree(old, ignore_errors=True)
  return final


def run(*cmd, **kw):
  print('+', ' '.join(map(str, cmd)), flush=True)
  return subprocess.run(list(map(str, cmd)), check=True, **kw)


def extract(osm: Path, work: Path) -> tuple[Path, Path]:
  """With osmium: (the control nodes as OPL, the highway ways as a PBF with their nodes)."""
  nodes, ways = work / 'controls.opl', work / 'highways.osm.pbf'
  run('osmium', 'tags-filter', '-R', '-O', '-f', 'opl', '-o', nodes, osm, 'n/highway=stop,give_way', 'n/railway=level_crossing')
  run('osmium', 'tags-filter', '-O', '-o', ways, osm, 'w/highway')
  return nodes, ways


def way_pass(ways_pbf: Path) -> Callable[[], Iterable[str]]:
  def lines():
    proc = subprocess.Popen(['osmium', 'add-locations-to-ways', '-f', 'opl', '-o', '-', str(ways_pbf)], stdout=subprocess.PIPE,
                            text=True, bufsize=1 << 20)
    assert proc.stdout is not None
    try:
      for line in proc.stdout:
        if line.startswith('w'):
          yield line
    finally:
      proc.stdout.close()
      if proc.wait() != 0:
        raise RuntimeError(f"osmium add-locations-to-ways {ways_pbf} failed ({proc.returncode})")
  return lines


def build(osm: Path, out_dir: Path, osm_timestamp: str, tile_dir: Path, work: Path | None = None) -> dict:
  """The sidecar of osm (a .osm.pbf or .osm), written as out_dir/osm_controls/ shards for every level-2 tile under
  tile_dir, stamped osm_timestamp. Returns the stats."""
  if not osm_timestamp:
    raise ValueError("no osm_timestamp: the car could not tell this sidecar's data from its tiles'")
  cells = tile_cells(tile_dir)
  if not cells:
    raise ValueError(f"no level-2 tiles under {tile_dir}")
  with tempfile.TemporaryDirectory(dir=work, prefix='osm-controls-') as tmp:
    nodes, ways = extract(Path(osm), Path(tmp))
    side = build_sidecar(nodes.read_text().splitlines(), way_pass(ways))
  shard_map = shards(side, cells, osm_timestamp)
  write_shards(out_dir, shard_map)
  stats = {**side['stats'], 'shards': len(shard_map), 'controls': sum(len(v) for v in side['controls'].values()),
           'junctions': len(side['junctions']), 'rail': len(side['rail'])}
  print(f"osm_controls: {json.dumps(stats, sort_keys=True)}", flush=True)
  return stats


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('osm', type=Path, help='the .osm.pbf or .osm the tiles were built from')
  ap.add_argument('out', type=Path, help='writes out/osm_controls/')
  ap.add_argument('--timestamp', required=True, help="the tiles' osm_timestamp (meta.json), stamped on every shard")
  ap.add_argument('--tiles', type=Path, required=True, help='the tile hierarchy (out/tiles): one shard per level-2 tile')
  args = ap.parse_args()
  build(args.osm, args.out, args.timestamp, args.tiles, work=Path(os.environ['TMPDIR']) if 'TMPDIR' in os.environ else None)


if __name__ == '__main__':
  main()
