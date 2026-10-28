#!/usr/bin/env python3
"""Build one region for the map server: cut it out of Geofabrik state extracts, build its Valhalla tiles and its OSM
stop-sign sidecar, and write its fence, metadata and tile index.

  build_region.py <region> [--osm DIR] [--out DIR]
  build_region.py <region> --sidecar-only [--out DIR]

The fence is the drivable area the car may navigate in: a circle around the city, clipped to the states we
have data for. The data is cut CUT_MARGIN_KM wider than the fence, so a route between two points just inside
the fence still has the roads around it. Needs shapely and osmium-tool on the build machine; the server needs
neither.

The sidecar (osm_controls.py) is built right after the tiles, from the same region PBF, as osm_controls/<level-2 tile>.json,
one shard per level-2 tile, stamped with the region's osm_timestamp. The index lists the shards beside the tiles, as
[2, tile index, sha256, size], so a corridor that asks for them gets the shards of its own tiles.
--sidecar-only rebuilds just the shards from <out>/<region>/<region>.osm.pbf, stamped with the osm_timestamp already in
its meta.json, and rewrites the index's shard entries (a full Indianapolis rebuild takes hours; this takes minutes). Its
meta.json, fence and tiles are left as they are, and it writes the controls_built marker, since the region's tiles were
built from that PBF, which carries its stop signs and signals. Needs osmium-tool only.
"""
import argparse
import datetime
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import osm_controls
import tiles

GEOFABRIK = 'https://download.geofabrik.de/north-america/us'
CUT_MARGIN_KM = 25
SIMPLIFY_DEG = 0.005  # about 500 m; the fence is shrunk by this much after simplifying, so it stays inside

REGIONS = {
  'indianapolis': {
    'name': 'Indianapolis area', 'center': (-86.1581, 39.7684), 'radius_km': 300,
    'states': ['indiana', 'illinois', 'ohio', 'kentucky', 'michigan'],
  },
  'seattle': {
    'name': 'Seattle area', 'center': (-122.3321, 47.6062), 'radius_km': 250,
    'states': ['washington', 'oregon'],
  },
}


def circle(center, radius_km, n=180):
  from shapely.geometry import Polygon  # only a full build needs shapely
  lon0, lat0 = map(math.radians, center)
  d = radius_km / tiles.EARTH_KM
  ring = []
  for i in range(n):
    b = 2 * math.pi * i / n
    lat = math.asin(math.sin(lat0) * math.cos(d) + math.cos(lat0) * math.sin(d) * math.cos(b))
    lon = lon0 + math.atan2(math.sin(b) * math.sin(d) * math.cos(lat0), math.cos(d) - math.sin(lat0) * math.sin(lat))
    ring.append((math.degrees(lon), math.degrees(lat)))
  return Polygon(ring)


def read_poly(path: Path):
  """A Geofabrik .poly file as a shapely geometry: numbered sections are outer rings, '!'-prefixed ones holes."""
  from shapely.geometry import Polygon
  from shapely.ops import unary_union
  outers, holes, ring, hole = [], [], None, False
  for line in path.read_text().splitlines()[1:]:
    line = line.strip()
    if not line:
      continue
    if line == 'END':
      if ring is None:
        break
      (holes if hole else outers).append(Polygon(ring))
      ring = None
    elif ring is None:
      ring, hole = [], line.startswith('!')
    else:
      lon, lat = map(float, line.split()[:2])
      ring.append((lon, lat))
  return unary_union(outers).difference(unary_union(holes)) if holes else unary_union(outers)


def run(*cmd):
  print('+', ' '.join(map(str, cmd)), flush=True)
  subprocess.run(list(map(str, cmd)), check=True)


def entries(out: Path, pattern: str, level_of) -> dict:
  index = {}
  for p in sorted(out.glob(pattern)):
    level, idx = level_of(p)
    index[p.relative_to(out).as_posix()] = [level, idx, hashlib.sha256(p.read_bytes()).hexdigest(), p.stat().st_size]
  return index


def tile_entries(out: Path) -> dict:
  return entries(out, 'tiles/**/*.gph', lambda p: tiles.parse_path(p.relative_to(out / 'tiles').as_posix()))


def shard_entries(out: Path) -> dict:
  """The sidecar's shards, as the level-2 tile each covers: tiles.needed() picks a corridor's shards with its tiles."""
  return entries(out, f'{osm_controls.SHARDS_DIR}/*.json', lambda p: (2, int(p.stem)))


def sidecar_only(region: str, out: Path):
  """Rebuild only the region's sidecar shards, stamped with its osm_timestamp, and their index entries."""
  pbf, meta_path, tile_dir = out / f'{region}.osm.pbf', out / 'meta.json', out / 'tiles'
  for need in (pbf, meta_path, out / 'index.json', tile_dir):
    if not need.exists():
      raise SystemExit(f"{need} is missing: build the region in full first")
  stamp = json.loads(meta_path.read_text()).get('osm_timestamp')
  if not stamp:
    raise SystemExit(f"{meta_path} has no osm_timestamp")
  stats = osm_controls.build(pbf, out, stamp, tile_dir)
  index = {rel: v for rel, v in json.loads((out / 'index.json').read_text()).items()
           if not rel.startswith(osm_controls.SHARDS_DIR + '/')}
  index.update(shard_entries(out))
  (out / 'index.json').write_text(json.dumps(index))
  (out / 'controls_built').touch()  # the tiles came from this PBF, with its stop signs and signals
  print(f"{region}: {stats['shards']} sidecar shards stamped {stamp}, {stats['controls']} controls, {stats['junctions']} junctions")


def main():
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('region', choices=sorted(REGIONS))
  ap.add_argument('--osm', type=Path, default=Path.home() / 'turnpilot_data/osm')
  ap.add_argument('--out', type=Path, default=Path.home() / 'turnpilot_data/mapserver/regions')
  ap.add_argument('--sidecar-only', action='store_true',
                  help="rebuild only the OSM stop-sign sidecar shards, from the region's PBF and with its osm_timestamp")
  args = ap.parse_args()
  spec = REGIONS[args.region]
  out = args.out / args.region
  if args.sidecar_only:
    return sidecar_only(args.region, out)
  from shapely.geometry import mapping
  from shapely.ops import unary_union
  out.mkdir(parents=True, exist_ok=True)

  for state in spec['states']:
    for name in (f'{state}-latest.osm.pbf', f'{state}.poly'):
      if not (args.osm / name).is_file():
        run('curl', '-sSfL', '-o', args.osm / name, f'{GEOFABRIK}/{name}')
  states = unary_union([read_poly(args.osm / f'{s}.poly') for s in spec['states']])

  # Geofabrik's state polygons reach a little past the border, so clipping to them keeps the fence on our data
  fence = circle(spec['center'], spec['radius_km']).intersection(states)
  fence = fence.simplify(SIMPLIFY_DEG).buffer(-SIMPLIFY_DEG)
  cut = circle(spec['center'], spec['radius_km'] + CUT_MARGIN_KM).intersection(states)
  (out / 'fence.geojson').write_text(json.dumps({'type': 'Feature', 'geometry': mapping(fence), 'properties': {}}))
  (out / 'cut.geojson').write_text(json.dumps({'type': 'Feature', 'geometry': mapping(cut), 'properties': {}}))

  pbfs = [args.osm / f'{s}-latest.osm.pbf' for s in spec['states']]
  merged, region_pbf = out / 'merged.osm.pbf', out / f'{args.region}.osm.pbf'
  run('osmium', 'merge', '--overwrite', '-o', merged, *pbfs)
  run('osmium', 'extract', '--overwrite', '--set-bounds', '-s', 'smart', '-p', out / 'cut.geojson', '-o', region_pbf, merged)
  merged.unlink()
  stamps = [subprocess.run(['osmium', 'fileinfo', '-g', 'header.option.osmosis_replication_timestamp', p],
                           capture_output=True, text=True, check=True).stdout.strip() for p in pbfs]
  run(HERE.parent / 'build_valhalla_tiles.sh', region_pbf, out)
  osm_controls.build(region_pbf, out, min(stamps), out / 'tiles')

  index = {**tile_entries(out), **shard_entries(out)}
  (out / 'index.json').write_text(json.dumps(index))
  meta = {'id': args.region, 'name': spec['name'], 'center': spec['center'], 'radius_km': spec['radius_km'],
          'states': spec['states'], 'osm_timestamp': min(stamps),
          'built_at': datetime.datetime.now(datetime.UTC).strftime('%Y-%m-%dT%H:%M:%SZ')}
  (out / 'meta.json').write_text(json.dumps(meta, indent=2))
  total = sum(v[3] for v in index.values())
  shards = sum(1 for rel in index if rel.startswith(osm_controls.SHARDS_DIR + '/'))
  print(f"{args.region}: {len(index) - shards} tiles and {shards} sidecar shards, {total / 1e6:.0f} MB, " +
        f"fence area {fence.area:.1f} sq deg, data from {meta['osm_timestamp']}")


if __name__ == '__main__':
  main()
