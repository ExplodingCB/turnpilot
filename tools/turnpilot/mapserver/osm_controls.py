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
   "controls": {"<way_id>:<f|b>": [{kind stop|yield, line [lat, lon] or null, junction_node (0: none), stop_tag
                                    all|minor|none, evidence, node (the sign's OSM node)}, ...]},
   "junctions": {"<osm node id>": {pos [lat, lon], approaches [{way_id, dir f|b, bearing_deg, class, maxspeed, oneway,
                                   stops, kind, evidence, line}, ...]}},
   "rail": [{lat, lon, ways [way_id, ...]}],
   "unplaced": [{node, kind, lat, lon, direction, stop_tag, why}]}
f: traffic travelling along the way's node order (Valhalla's edge.forward), b: against it. A control sits in the shard of
its line (of its junction when it has no line), a junction, a crossing and an unplaced sign in the shard of their
position. Every stop and give_way node is in a shard once, as a control or unplaced, so route_audit can list one near a
route that no control placed.

The junction model (from stopsigns/scripts/allway_osm.py):
- drivable ways: DRIVABLE highway classes, not area=yes, not closed to motor vehicles. A junction is a node with at least
  3 drivable legs (2 where a way passes through, 1 where it ends). An approach is (way, f) arriving along the way's order
  or (way, b) against it; a oneway gives only its legal arrival.
- a stop or give_way on a junction node applies to every approach (evidence node_all with stop=all, node_untagged
  without), or with stop=minor only to the approaches of a lower class than the junction's top one (node_minor, as
  Valhalla does); it has no line (null): the node is the junction's middle, not where anyone stops.
- anywhere else (mid-way, or on a node shared only with a footway or a way's continuation) it governs the first junction
  within MAXD m along its way, the walk carrying on across a way split (a 2-leg node where the way ends and one other
  drivable way starts or ends) onto the way carrying on, CARRY_ON_PASSES splits at most: direction=forward the one ahead
  along the way (mid_directed), backward the one behind, both each of them (mid_both), and with no or another direction
  the nearest (mid_undirected, by proximity). Its node is the line, and it is keyed to the way its traffic arrives at it
  on (the way ending there when the sign is on a split). A stop with no junction within MAXD still stops its approach:
  it is kept with junction_node 0 (each direction its way allows when undirected) and counted (no_junction_within_maxd).
- a sign no approach takes is unplaced, with why: not_on_a_drivable_way (mapped beside the road, on a footway), or
  no_lower_approach (stop=minor on a junction whose approaches are all of its top class).
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
CARRY_ON_PASSES = 2  # way splits a walk carries on across (each one more pass over the ways)
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
  over the highway ways with their node locations (it is called twice, and once more per way split the walks carry on
  across, CARRY_ON_PASSES at most). {'controls', 'junctions' (by int node id), 'rail', 'unplaced', 'stats'}."""
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
  gathered = set()         # the nodes occ is complete for

  def gather(nodes):
    new = set(nodes) - gathered
    if not new:
      return
    gathered.update(new)
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
      hit = [i for i, n in enumerate(ids) if n in new]
      if not hit:
        continue
      d = drivable(t)
      wid = int(f['w'])
      for i in hit:
        occ[ids[i]].append((wid, i, len(ids), d))
      if d and wid not in W:
        nodes = [(int(n), float(y) if y else math.nan, float(x) if x else math.nan) for n, x, y in NODE.findall(nf)]
        W[wid] = ({k: t[k] for k in ('highway', 'oneway', 'junction', 'service', 'maxspeed') if k in t}, nodes)

  gather(C)

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

  def carry_on(wid, end):
    """(way, index of end in it, step along it) for the one other drivable way at end, where wid ends and it does (a way
    split: two legs, no junction), or None"""
    legs = legs_at(end)
    others = {w for w, _, _ in legs if w != wid}
    if len(legs) != 2 or len(others) != 1:
      return None
    nxt = others.pop()
    nodes = W[nxt][1]
    if nodes[0][0] == end:
      return nxt, 0, +1
    return (nxt, len(nodes) - 1, -1) if nodes[-1][0] == end else None

  def walk(wid, i, step, need=None):
    """(junction node, m along, the way and step arriving there) for the first junction from index i of way wid in
    direction step within MAXD, across a way split onto the way carrying on (carry_on), or None. need: a set the nodes
    of a way the walk would carry on along are added to while occ isn't complete for them (it stops there)."""
    d, seen = 0., {wid}
    while True:
      nodes = W[wid][1]
      j = i
      while 0 <= j + step < len(nodes):
        a, b = nodes[j], nodes[j + step]
        d += dist(a[1:], b[1:])
        j += step
        if d > MAXD:
          return None
        if junction(b[0]):
          return b[0], d, wid, step
      nxt = carry_on(wid, nodes[j][0])
      if nxt is None or nxt[0] in seen:
        return None
      if any(n not in gathered for n, *_ in W[nxt[0]][1]):
        if need is not None:
          need.update(n for n, *_ in W[nxt[0]][1])
        return None
      wid, i, step = nxt
      seen.add(wid)

  def arriving(wid, i, step):
    """(way, step) of the traffic going `step` along way wid that arrives at its index i: wid's own unless i is where
    that traffic leaves wid's first or last node, which a way split's other way arrives at"""
    if (step > 0 and i > 0) or (step < 0 and i < len(W[wid][1]) - 1):
      return wid, step
    nxt = carry_on(wid, W[wid][1][i][0])
    return (wid, step) if nxt is None else (nxt[0], -nxt[2])

  def mid_way(sid):
    """(way, index) of a mid-way stop node on a drivable way, or None for one on a junction node or on none"""
    dways = [(w, i) for w, i, _, d in occ.get(sid, []) if d and w in W]
    return dways[0] if dways and not junction(sid) else None

  # the ways the walks carry on along, a split at a time (CARRY_ON_PASSES more passes at most)
  for _ in range(CARRY_ON_PASSES):
    need = set()
    for sid in controls_in:
      if (at := mid_way(sid)) is not None:
        walk(*at, +1, need)
        walk(*at, -1, need)
    if not need - gathered:
      break
    gather(need)

  stats = Counter()
  controls = defaultdict(list)                # key -> [entry]
  on_approach = defaultdict(dict)             # junction -> {(way, step): entry}
  unplaced = []                               # the signs no approach takes
  for sid, (la, lo, t) in sorted(controls_in.items()):
    kind = KINDS[t['highway']]
    stop_tag = t.get('stop' if kind == 'stop' else 'give_way', '')
    stop_tag = stop_tag if stop_tag in ('all', 'minor') else 'none'
    sign = {'node': sid, 'kind': kind, 'lat': round(la, 7), 'lon': round(lo, 7), 'direction': t.get('direction', ''), 'stop_tag': stop_tag}
    dways = [(w, i) for w, i, _, d in occ.get(sid, []) if d and w in W]
    if not dways:
      stats['not_on_a_drivable_way'] += 1
      unplaced.append(sign | {'why': 'not_on_a_drivable_way'})
      continue
    if junction(sid):
      stats['junction_node'] += 1
      apps = approaches(sid)
      top = min((r for *_, r, _ in apps), default=8)
      evidence = 'node_minor' if stop_tag == 'minor' else 'node_all' if stop_tag == 'all' else 'node_untagged'
      placed = False
      for w, step, r, _ in apps:
        if stop_tag == 'minor' and r <= top:
          continue
        entry = {'kind': kind, 'line': None, 'junction_node': sid, 'stop_tag': stop_tag, 'evidence': evidence, 'node': sid}
        controls[key(w, step)].append(entry)
        on_approach[sid].setdefault((w, step), entry)
        placed = True
      if not placed:
        stats['no_lower_approach'] += 1
        unplaced.append(sign | {'why': 'no_lower_approach'})
      continue
    w, i = dways[0]
    direction = t.get('direction', '')
    walks = {+1: walk(w, i, +1), -1: walk(w, i, -1)}
    if direction == 'forward':
      steps, evidence = [+1], 'mid_directed'
    elif direction in ('backward', 'reverse'):
      steps, evidence = [-1], 'mid_directed'
    elif direction == 'both':
      steps, evidence = [+1, -1], 'mid_both'
    else:
      near = [st for st in (+1, -1) if walks[st]]
      steps, evidence = ([min(near, key=lambda st: walks[st][1])] if near else [+1, -1]), 'mid_undirected'
    if not any(walks[st] for st in steps):
      stats['no_junction_within_maxd'] += 1
    else:
      stats[evidence] += 1
    for step in steps:
      aw, astep = arriving(w, i, step)
      found = walks[step]
      if found is None and evidence == 'mid_undirected' and (oneway(W[aw][0]) * astep) < 0:
        continue  # a oneway's illegal arrival
      # a stop tied to no junction within MAXD (a mid-block trail or rail crossing, a dead end) still stops its approach
      entry = {'kind': kind, 'line': rounded((la, lo)), 'junction_node': found[0] if found else 0, 'stop_tag': stop_tag,
               'evidence': evidence, 'node': sid}
      controls[key(aw, astep)].append(entry)
      if found is not None:
        jn, _, jw, jstep = found
        prev = on_approach[jn].get((jw, jstep))
        if prev is None or prev['line'] is None:  # a mid-way line before a node's
          on_approach[jn][(jw, jstep)] = entry

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
  return {'controls': dict(sorted(controls.items())), 'junctions': junctions, 'rail': rail, 'unplaced': unplaced, 'stats': dict(stats)}


def shards(sidecar: dict, cells: set[int], osm_timestamp: str) -> dict[int, dict]:
  """{level-2 tile: shard} for every tile in cells; what falls in no tile there is left out (counted in the stats)."""
  out = {c: {'version': VERSION, 'osm_timestamp': osm_timestamp, 'tile': c, 'controls': {}, 'junctions': {}, 'rail': [], 'unplaced': []}
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
  for what in ('rail', 'unplaced'):
    for r in sidecar.get(what, []):
      cell = level2_index(r['lon'], r['lat'])
      if cell in out:
        out[cell][what].append(r)
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
