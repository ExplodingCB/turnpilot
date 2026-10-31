#!/usr/bin/env python3
"""Choose the showcase all-way for step S2: every 3-way all-way stop near a point that a car can arrive at on the stem of the
T with the sun off its nose over a time window (STOP_SIGNS_SATURDAY.md; STOP_SIGNS.md 4.3).

  allway_candidates.py --region DIR [--sidecar DIR] [--osm PBF] --near LAT,LON --radius-km R --start ISO --end ISO
                       [--max-mph 20] [--json]

--region is a map-server region (~/turnpilot_data/mapserver/regions/indianapolis): its meta.json stamps the tiles, its
osm_controls/ holds the stop-sign sidecar's shards (step S0, build_region.py --sidecar-only) and its <id>.osm.pbf the ways.
--sidecar reads the shards from another directory (a scratch build), checked against the region's stamp all the same;
--osm reads the ways from another PBF. --start and --end are ISO times with their zone (2026-10-31T10:00-04:00).

The junctions come from the sidecar's records (every approach arriving at the node, and whether it stops); the legs come
from the ways themselves, since a record lists only the legs traffic arrives on and a one-way leg that only leaves is in
none (181199929's north leg). A junction is listed when:
- every approach the records have stops at a stop sign (no yields), and every leg traffic can arrive on has a record;
- exactly 3 legs carry arriving traffic, and one of them is a stem: the other two within STEM_OPPOSITE_DEG of opposite each
  other and each 75-105 deg from the stem's arriving heading. At most one more leg, one-way away from the node and within
  OPPOSITE_SKEW_DEG of straight on (an exit with nothing arriving from it: stop_rules.leg_skew allows it, no conflict set has
  it) is listed and flagged; any other leg makes it no T;
- the class mix is plausible (junctions.PLAUSIBLE_TOP_RANK), no mapped rail crossing within RAIL_M, no service road, slip
  lane (*_link) or roundabout way among its legs, no leg made of two ways (a divided road);
- the light check certify() makes (stop_rules.light_window, the veil unmeasured: no night, dusk or low sun along the cross
  approaches, the sun not within 90 deg of the arriving heading) passes at every step from --start - ETA_EARLY_S to --end
  + ETA_LATE_S;
- the cross road's limit (the higher of the two tagged maxspeeds, else stop_rules' class default: an untagged residential
  is 30 mph) is --max-mph or less. The tag is OSM's, not the posted sign: the survey's postedMph decides.

Each candidate says whether every approach is mapStrong-like (a directed or stop=all sidecar stop, as junctions.classify
tiers it), the cross limits, the reach an all-way lane needs at that limit (stop_rules.reach_needed_allway), its distance
from --near, and route_audit commands for a trip through the stem (TRIP_M back along it, EXIT_M out along a cross leg, so
the route has to pass the node): the survey template, and the audit of the left and of the right turn (pick the turn from
their predicted reach). Check the audit lists the junction: a trip that misses it says nothing about it. Nothing here is evidence: route_audit, the survey and
validation decide.

Refuses (exit 1, naming why): no shards; a shard whose osm_timestamp isn't the region's meta.json one; a window any part of
which (with ETA_EARLY_S before and ETA_LATE_S after) has the sun under the horizon at --near. It never writes a file.
"""
import argparse
import json
import math
import subprocess
import sys
import tempfile
from collections import defaultdict
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.routing.junctions import PLAUSIBLE_TOP_RANK
from tools.turnpilot.mapserver import osm_controls as oc

SHARDS = 'osm_controls'
STEM_OPPOSITE_DEG = 15.      # deg off opposite the two cross legs may be
SIDE_LO_DEG, SIDE_HI_DEG = 75., 105.  # deg from the arriving heading a cross leg is
LEG_MERGE_DEG = 10.          # deg: legs this close in bearing are one leg (two ways there: a divided road)
RAIL_M = sr.RAIL_M
TRIP_M = 150.                # m back along the stem (across way splits) the audit trip starts
EXIT_M = 40.                 # m out along each cross leg it ends: near enough that the route can only go through the node
CARRY_ON_DEG = 30.           # deg: a way carrying on from a way's end this close to straight is the same road
BBOX_MARGIN_KM = 0.5         # km around --radius-km the ways are extracted over (the trip walks 150 m out)
STRONG_EVIDENCE = ('mid_directed', 'mid_both', 'node_all')
BAD_CLASSES = ('service',)


class Refusal(Exception):
  """The tool refuses: the message says why."""


@dataclass
class Leg:
  bearing: float           # deg, the leg leaving the node, over osm_controls.BEARING_M
  ways: list[int]
  classes: list[str]
  arrives: bool            # traffic can arrive at the node on it
  leaves: bool
  tags: list[dict] = field(default_factory=list)
  path: list = field(default_factory=list)  # (lat, lon) from the node out along its first way


@dataclass
class Candidate:
  node: int
  lat: float
  lon: float
  heading_in: float        # deg, arriving on the stem
  stem_way: int
  cross_bearings: list[float]
  cross_limit_mph: int
  cross_limit_source: str  # 'tagged' or 'classDefault'
  cross_tagged_mph: list   # each cross approach's tagged limit, None untagged
  need_m: float            # reach_needed_allway at the cross limit
  map_strong: bool
  exit_only: list[float]   # deg, a one-way leg leaving straight on
  classes: list[str]
  km: float
  rows: list[str]          # the lighting rows the window needs
  commands: dict = field(default_factory=dict)


def angle_diff(a: float, b: float) -> float:
  return abs((a - b + 180.) % 360. - 180.)


def parse_ways(lines) -> tuple[dict, dict]:
  """From osmium OPL with way node locations: ({way: (tags, [(node, lat, lon)])} of the drivable ways, {node: [(way,
  index)]} over them)."""
  ways, occ = {}, defaultdict(list)
  for line in lines:
    if not line.startswith('w'):
      continue
    f = oc.fields(line)
    t = oc.parse_tags(f.get('T', ''))
    if not oc.drivable(t):
      continue
    nodes = [(int(n), float(y) if y else math.nan, float(x) if x else math.nan) for n, x, y in oc.NODE.findall(f.get('N', ''))]
    wid = int(f['w'])
    ways[wid] = (t, nodes)
    for i, (n, _, _) in enumerate(nodes):
      occ[n].append((wid, i))
  return ways, occ


def _walk(nodes, i: int, step: int, max_m: float) -> list:
  out, d, j = [nodes[i][1:]], 0., i
  while 0 <= j + step < len(nodes) and d < max_m:
    d += oc.dist(nodes[j][1:], nodes[j + step][1:])
    j += step
    out.append(nodes[j][1:])
  return out


def _trace(ways: dict, occ: dict, wid: int, i: int, step: int, max_m: float) -> list:
  """(lat, lon) from index i of way wid along it in direction step for max_m, carrying on at its end onto the drivable way
  that goes on within CARRY_ON_DEG of straight (none: the path ends there)."""
  nodes = ways[wid][1]
  out, seen = _walk(nodes, i, step, max_m), {wid}
  while True:
    d = sum(oc.dist(a, b) for a, b in zip(out, out[1:], strict=False))
    if d >= max_m or len(out) < 2:
      return out
    end = nodes[0][0] if step < 0 else nodes[-1][0]
    here = oc.bearing(out[-2], out[-1])
    best = None
    for w2, j in occ.get(end, []):
      if w2 in seen:
        continue
      n2 = ways[w2][1]
      for s2 in (-1, +1):
        if 0 <= j + s2 < len(n2):
          dev = angle_diff(oc.bearing(n2[j][1:], n2[j + s2][1:]), here)
          if dev <= CARRY_ON_DEG and (best is None or dev < best[0]):
            best = (dev, w2, j, s2)
    if best is None:
      return out
    _, wid, i, step = best
    nodes = ways[wid][1]
    seen.add(wid)
    out += _walk(nodes, i, step, max_m - d)[1:]


def _at(path, m: float):
  """The point m along a polyline of (lat, lon), or its end."""
  d = 0.
  for a, b in zip(path, path[1:], strict=False):
    s = oc.dist(a, b)
    if d + s >= m and s > 0.:
      k = (m - d) / s
      return a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k
    d += s
  return path[-1]


def legs_at(node: int, ways: dict, occ: dict) -> list[Leg]:
  """Every drivable leg at node from the ways, legs within LEG_MERGE_DEG of each other merged."""
  raw = []
  for wid, i in occ.get(node, []):
    t, nodes = ways[wid]
    ow = oc.oneway(t)
    for step in (-1, +1):  # the leg toward the previous node, toward the next (a closed way's node is at both its ends)
      if not (0 <= i + step < len(nodes)):
        continue
      path = _walk(nodes, i, step, oc.BEARING_M)
      # traffic arriving on the leg toward the previous node travels along the way's order (+1), toward the next against it
      arrive_step = -step
      arrives = not ((arrive_step == +1 and ow == -1) or (arrive_step == -1 and ow == 1))
      leaves = not ((step == +1 and ow == -1) or (step == -1 and ow == 1))
      raw.append(Leg(round(oc.bearing(path[0], path[-1]), 1), [wid], [t.get('highway', '')], arrives, leaves, [t],
                     _trace(ways, occ, wid, i, step, TRIP_M + 10.)))
  merged: list[Leg] = []
  for leg in sorted(raw, key=lambda g: g.bearing):
    same = next((m for m in merged if angle_diff(m.bearing, leg.bearing) < LEG_MERGE_DEG), None)
    if same is None:
      merged.append(leg)
    else:
      same.ways += leg.ways
      same.classes += leg.classes
      same.tags += leg.tags
      same.arrives |= leg.arrives
      same.leaves |= leg.leaves
  return merged


def map_strong(record: list[dict]) -> bool:
  """Every approach a directed or stop=all sidecar stop and nothing stop=minor (junctions.classify's mapStrong)."""
  return all(a.get('evidence') in STRONG_EVIDENCE for a in record) and not any(a.get('evidence') == 'node_minor' for a in record)


def plausible(classes) -> bool:
  ranks = [oc.rank(c) for c in classes if oc.rank(c) < 8]
  top = min(ranks, default=8)
  return not (top <= PLAUSIBLE_TOP_RANK and any(r >= top + 2 for r in ranks))


def record_for(record: list[dict], leg: Leg) -> dict | None:
  """The sidecar's arriving approach on a leg: its way, arriving toward the node, at the leg's bearing."""
  fits = [a for a in record if int(a.get('way_id', 0)) in leg.ways and angle_diff(float(a.get('bearing_deg', 0.)), leg.bearing) < LEG_MERGE_DEG]
  return fits[0] if fits else None


def judge(node: int, rec: dict, ways: dict, occ: dict, rails, near, start: float, end: float, max_mph: float) -> tuple[list[Candidate], str]:
  """(the junction's candidate stems, '') or ([], why it isn't one)."""
  record = rec.get('approaches') or []
  lat, lon = float(rec['pos'][0]), float(rec['pos'][1])
  if not record:
    return [], 'no arriving approach in the record'
  if not all(a.get('stops') and a.get('kind') == 'stop' for a in record):
    return [], 'not every approach stops at a stop sign'
  if node not in occ:
    return [], 'the node is on no drivable way in the OSM data'
  legs = legs_at(node, ways, occ)
  if any(len(set(g.ways)) > 1 for g in legs):
    return [], 'a leg made of two ways (a divided road or a split)'
  classes = sorted({c for g in legs for c in g.classes})
  if any(c in BAD_CLASSES or c.endswith('_link') for c in classes):
    return [], f"a {'/'.join(c for c in classes if c in BAD_CLASSES or c.endswith('_link'))} leg"
  if any(t.get('junction') in ('roundabout', 'circular') for g in legs for t in g.tags):
    return [], 'a roundabout'
  arriving = [g for g in legs if g.arrives]
  for g in arriving:
    if record_for(record, g) is None:
      return [], f"traffic arrives on the leg at {g.bearing:.0f} deg, which the sidecar has no approach for"
  if len(arriving) != 3:
    return [], f"{len(arriving)} legs carry arriving traffic, not 3"
  if not plausible(classes):
    return [], f"an implausible all-way ({', '.join(classes)})"
  if any(oc.dist((lat, lon), (float(r['lat']), float(r['lon']))) < RAIL_M for r in rails):
    return [], f"a rail crossing within {RAIL_M:.0f} m"
  out, why = [], 'no stem: no arriving leg has the other two opposite each other and square to it'
  for stem in arriving:
    others = [g for g in arriving if g is not stem]
    h_in = (stem.bearing + 180.) % 360.
    if angle_diff(others[0].bearing, others[1].bearing) < 180. - STEM_OPPOSITE_DEG:
      continue
    if not all(SIDE_LO_DEG <= angle_diff(g.bearing, h_in) <= SIDE_HI_DEG for g in others):
      continue
    extra = [g for g in legs if not g.arrives]
    bad = [g for g in extra if angle_diff(g.bearing, h_in) > sr.OPPOSITE_SKEW_DEG]
    if bad or len(extra) > 1:
      why = f"a leg at {bad[0].bearing if bad else extra[1].bearing:.0f} deg besides the T"
      continue
    looks = sorted(g.bearing for g in others)
    rows, light = sr.light_window(lat, lon, start, h_in, 'unmeasured', looks, early_s=sr.ETA_EARLY_S,
                                  late_s=(end - start) + sr.ETA_LATE_S)
    if light:
      why = f"arriving at {h_in:.0f} deg: {light}"
      continue
    cross = [record_for(record, g) for g in others]
    tagged = [round(float(a['maxspeed']) / sr.MPH) if a.get('maxspeed') else None for a in cross]
    if all(t is not None for t in tagged):
      limit, source = max(tagged), 'tagged'
    else:
      limit = max([t for t in tagged if t is not None] + [sr.class_limit_mph(a.get('class')) for a, t in zip(cross, tagged, strict=True)
                                                          if t is None])
      source = 'classDefault'
    if limit > max_mph + 1e-9:
      why = f"arriving at {h_in:.0f} deg: the cross road is {limit} mph ({source}), over {max_mph:g}"
      continue
    out.append(Candidate(node, lat, lon, round(h_in, 1), stem.ways[0], [g.bearing for g in others], int(limit), source, tagged,
                         round(sr.reach_needed_allway(limit * sr.MPH), 1), map_strong(record), [g.bearing for g in extra],
                         classes, round(oc.dist((lat, lon), near) / 1000., 1), sorted(rows)))
    out[-1].commands = trip_commands(stem, others, h_in)
  return out, ('' if out else why)


def trip_commands(stem: Leg, others: list[Leg], h_in: float) -> dict:
  """{'origin', 'left', 'right'}: TRIP_M back along the stem, EXIT_M out along the left and the right leg."""
  o = _at(stem.path, TRIP_M)
  out = {'origin': f"{o[0]:.6f},{o[1]:.6f}"}
  for g in others:
    side = 'right' if ((g.bearing - h_in) % 360.) < 180. else 'left'
    p = _at(g.path, EXIT_M)
    out[side] = f"{p[0]:.6f},{p[1]:.6f}"
  return out


def read_meta_stamp(region: Path) -> str:
  meta = region / 'meta.json'
  try:
    stamp = json.loads(meta.read_text()).get('osm_timestamp')
  except (OSError, ValueError) as e:
    raise Refusal(f"{meta} doesn't read ({type(e).__name__}): no stamp to check the shards against") from None
  if not isinstance(stamp, str) or not stamp:
    raise Refusal(f"{meta} has no osm_timestamp to check the shards against")
  return stamp


def read_shards(shard_root: Path, stamp: str) -> tuple[dict[int, dict], list[dict]]:
  """({node: junction record}, rail crossings) from every shard, or a Refusal: no shards, or one stamped otherwise."""
  d = shard_root / SHARDS
  files = sorted(d.glob('*.json')) if d.is_dir() else []
  if not files:
    raise Refusal(f"no {SHARDS}/ shards in {shard_root} (step S0: build_region.py --sidecar-only)")
  junctions, rails = {}, []
  for p in files:
    try:
      shard = json.loads(p.read_text())
    except (OSError, ValueError) as e:
      raise Refusal(f"shard {p} doesn't read ({type(e).__name__})") from None
    if shard.get('osm_timestamp') != stamp:
      raise Refusal(f"shard {p.name} is stamped {shard.get('osm_timestamp')!r}, the region's meta.json {stamp!r}: rebuild the sidecar")
    for k, j in (shard.get('junctions') or {}).items():
      junctions[int(k)] = j
    rails.extend(shard.get('rail') or [])
  return junctions, rails


def check_window(lat: float, lon: float, start: float, end: float) -> None:
  """A Refusal when the sun is under the horizon at any ETA_STEP_S step of [start - ETA_EARLY_S, end + ETA_LATE_S]."""
  if end < start:
    raise Refusal("--end is before --start")
  t0, t1 = start - sr.ETA_EARLY_S, end + sr.ETA_LATE_S
  n = max(int(math.ceil((t1 - t0) / sr.ETA_STEP_S)), 1)
  for k in range(n + 1):
    t = t0 + (t1 - t0) * k / n
    el, _ = sr.sun_position(lat, lon, t)
    if el < 0.:
      raise Refusal(f"the window touches dusk or night: the sun is {el:.1f} deg at {datetime.fromtimestamp(t).astimezone().isoformat(timespec='minutes')}")


def osm_ways(pbf: Path, near, radius_km: float) -> tuple[dict, dict]:
  """The drivable ways within radius_km (and BBOX_MARGIN_KM) of near, with osmium."""
  r = radius_km + BBOX_MARGIN_KM
  dlat = r * 1000. / oc.EARTH_R * 180. / math.pi
  dlon = dlat / math.cos(math.radians(near[0]))
  bbox = f"{near[1] - dlon:.6f},{near[0] - dlat:.6f},{near[1] + dlon:.6f},{near[0] + dlat:.6f}"
  with tempfile.TemporaryDirectory(prefix='allway-') as tmp:
    cut, hw = Path(tmp) / 'cut.osm.pbf', Path(tmp) / 'hw.osm.pbf'
    subprocess.run(['osmium', 'extract', '-b', bbox, '-O', '-o', str(cut), str(pbf)], check=True, capture_output=True)
    subprocess.run(['osmium', 'tags-filter', '-O', '-o', str(hw), str(cut), 'w/highway'], check=True, capture_output=True)
    return parse_ways(oc.way_pass(hw)())


def candidates(junctions: dict, rails: list, ways: dict, occ: dict, near, radius_km: float, start: float, end: float,
               max_mph: float) -> tuple[list[Candidate], dict[int, str]]:
  """(the candidates, nearest first; {node: why not} for the all-stop junctions within radius_km that aren't)."""
  out, why = [], {}
  for node, rec in junctions.items():
    pos = rec.get('pos')
    if not pos or oc.dist((float(pos[0]), float(pos[1])), near) > radius_km * 1000.:
      continue
    if not all(a.get('stops') for a in rec.get('approaches') or [{}]):
      continue
    found, reason = judge(node, rec, ways, occ, rails, near, start, end, max_mph)
    out += found
    if reason:
      why[node] = reason
  return sorted(out, key=lambda c: (bool(c.exit_only), c.km)), why


def point(s: str) -> tuple[float, float]:
  lat, lon = (float(x) for x in s.split(','))
  return lat, lon


def iso(s: str) -> float:
  t = datetime.fromisoformat(s)
  if t.tzinfo is None:
    raise argparse.ArgumentTypeError(f"{s}: give the zone (2026-10-31T10:00-04:00)")
  return t.timestamp()


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('--region', required=True, type=Path, help="the map-server region: meta.json, osm_controls/, <id>.osm.pbf")
  ap.add_argument('--sidecar', type=Path, help='read osm_controls/ from here instead (checked against the region stamp)')
  ap.add_argument('--osm', type=Path, help="the ways' PBF (default: the region's own)")
  ap.add_argument('--near', required=True, type=point, help='lat,lon')
  ap.add_argument('--radius-km', required=True, type=float)
  ap.add_argument('--start', required=True, help='ISO time with its zone')
  ap.add_argument('--end', required=True)
  ap.add_argument('--max-mph', type=float, default=20.)
  ap.add_argument('--json', action='store_true')
  args = ap.parse_args(argv)
  start_iso = args.start
  try:
    args.start, args.end = iso(args.start), iso(args.end)
  except (ValueError, argparse.ArgumentTypeError) as e:
    print(f"refused: {e}", file=sys.stderr)
    return 1
  try:
    stamp = read_meta_stamp(args.region)
    junctions, rails = read_shards(args.sidecar or args.region, stamp)
    check_window(args.near[0], args.near[1], args.start, args.end)
    pbf = args.osm or next(iter(sorted(args.region.glob('*.osm.pbf'))), None)
    if pbf is None or not Path(pbf).is_file():
      raise Refusal(f"no .osm.pbf in {args.region} to read the legs from (--osm)")
    ways, occ = osm_ways(Path(pbf), args.near, args.radius_km)
  except Refusal as e:
    print(f"refused: {e}", file=sys.stderr)
    return 1
  found, why = candidates(junctions, rails, ways, occ, args.near, args.radius_km, args.start, args.end, args.max_mph)
  tiles = args.region / 'tiles.tar'
  side = args.sidecar or args.region
  audit = f"python -m openpilot.tools.turnpilot.route_audit --tiles {tiles} --sidecar {side}"
  for c in found:
    o = c.commands['origin']
    c.commands = {'survey': f"{audit} --survey-template {o} {c.commands.get('left') or c.commands.get('right')}",
                  **{f"audit {s}": f"{audit} --eta {start_iso} {o} {c.commands[s]}"
                     for s in ('left', 'right') if s in c.commands}}
  if args.json:
    print(json.dumps({'candidates': [asdict(c) for c in found], 'notCandidates': {str(k): v for k, v in sorted(why.items())}}, indent=1))
    return 0
  print(f"{len(found)} stems at 3-way all-way stops within {args.radius_km:g} km of {args.near[0]},{args.near[1]} " +
        f"(cross road {args.max_mph:g} mph or less, the sun off the nose over the window); {len(why)} all-stop junctions not")
  for c in found:
    tags = ', '.join('untagged' if t is None else f"{t} mph" for t in c.cross_tagged_mph)
    print(f"\n{c.node} at {c.lat:.6f},{c.lon:.6f}, {c.km} km away: arriving {c.heading_in:.0f} deg on way {c.stem_way}; cross " +
          f"{c.cross_limit_mph} mph ({c.cross_limit_source}; tags {tags}), each cross lane needs {c.need_m} m; " +
          f"{'every approach mapStrong-like' if c.map_strong else 'NOT every approach mapStrong-like'}; {', '.join(c.classes)}" +
          ''.join(f"\n  + a one-way exit leg at {b:.0f} deg straight on: nothing arrives on it, check route_audit shows it in no conflict set"
                  for b in c.exit_only))
    for k, cmd in c.commands.items():
      print(f"  {k}: {cmd}")
  return 0


if __name__ == '__main__':
  sys.exit(main())
