#!/usr/bin/env python3
"""Every stop and yield approach on a route, from both map sources: Valhalla's flag, the OSM stop-sign sidecar's, the stop
line's source and the OSM junction node (STOP_SIGNS.md, SG1a; step S2 runs it on each demo trip).

  python -m openpilot.tools.turnpilot.route_audit --tiles DIR [--sidecar DIR] [--json] ORIGIN DESTINATION

ORIGIN and DESTINATION are "lat,lon". --tiles is what TurnPilotValhallaTiles takes (a region's or the sim's tile directory,
tiles.tar, valhalla.json); --sidecar the directory holding osm_controls/ and the tiles' stamp (meta.json or a corridor's
manifest.json), by default the tiles' own. A map-server region without the controls_built marker is read as a corridor
is (tiles built from its PBF carry their controls): build_region.py --sidecar-only writes the marker.

One row per route edge where Valhalla or the sidecar has a stop or yield (routing/controls.py's union): the m along the
route of the edge's end (its node), the edge's way and direction, Valhalla's flag (a phantom it made of a stop it can't
direct is shown dropped), the sidecar's entries (kind, evidence, stop=all|minor|none), the line source of the control there
(sim: the sim's controls.json; default: the node less 10 m), where the OSM stop node is (the sign: data only, it never
moves the stop point in SG1a), the OSM junction node, and the control navd sends (its kind and stop point). An
internal-intersection edge's row says which junction's control it joined.

Then every OSM stop and give_way node within RAW_M of the route that no row covers: neither the sidecar placed it on the
edge it lies on nor Valhalla flagged that edge with its kind. Each says who the sidecar has it for: ours (keyed to our
edge but not placed on it), the other direction of our way (often right, but a reversed direction tag looks the same),
another way, or no approach (unplaced: on no drivable way, or a stop=minor with no lower approach). The car stops for
none of them; the auditor checks each.
"""
import argparse
import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

from openpilot.selfdrive.turnpilot.routing import osm_sidecar
from openpilot.selfdrive.turnpilot.routing.controls import MARKER, cumulative, nearest, route_controls, route_shape

RAW_M = 5.           # m from the route an OSM stop or give_way node is listed within
M_PER_DEG = 111195.


@dataclass
class Row:
  node_m: float        # m along the route to the edge's end
  key: str             # '<way_id>:<f|b>'
  internal: bool
  valhalla: str        # Valhalla's flags on the edge, or '-'
  sidecar: str         # the sidecar's entries for it, '-' for none, 'not checked' without a valid sidecar
  line_source: str     # the control's line source there, '-' with no control
  osm_node: int        # the OSM junction node the sidecar names (0 none)
  control: str         # the control navd sends there, '-' for none
  joins: int = -1      # an internal edge's: the trace edge whose junction its control is
  sign_m: float | None = None  # m along the route of the OSM stop node on the control's edge (data only), None for none


@dataclass
class Sign:
  route_m: float       # m along the route (its shape) of the node's nearest point on it
  off_m: float         # m off the route
  node: int            # the OSM node id (0 for a shard without it)
  kind: str            # 'stop' or 'yield'
  tags: str            # its direction and stop tags
  holder: str          # who the sidecar has it for: 'ours', 'other direction', 'way <keys>', 'no approach (<why>)'
  edge: str            # the route edge it lies on, '-' for none
  to_node_m: float     # m before that edge's end node


@dataclass
class Report:
  status: str          # navInstruction.controlsStatus
  sidecar: str         # the sidecar's status and why
  rows: list[Row] = field(default_factory=list)
  controls: list = field(default_factory=list)
  route_m: float = 0.
  signs: list[Sign] = field(default_factory=list)  # OSM stop and give_way nodes near the route that no row covers


def audit(call, steps, tiles_dir, sidecar_dir) -> Report:
  """The rows for a route's steps, through call (routing/controls.py's actor_call or http_call)."""
  edges: list[dict] = []
  controls, status = route_controls(call, steps, tiles_dir, sidecar_dir, audit=edges)
  side = osm_sidecar.load(sidecar_dir, route_shape(steps)) if sidecar_dir is not None else None
  checked = side is not None and side.status == 'ok'
  by_edge = {c.edge: c for c in controls}
  rows = []
  for e in edges:
    if not (e['osm'] or e['phantom'] or 'stop' in e['valhalla'] or 'yield' in e['valhalla']):
      continue
    valhalla = ' '.join(e['valhalla'] + [f"{k} (phantom: dropped)" for k in e['phantom']]) or '-'
    sidecar = ', '.join(f"{h['kind']} ({h['evidence']}, stop={h['stop_tag']})" for h in e['osm']) or '-' if checked else 'not checked'
    c = by_edge.get(e['anchor'])
    osm_node = next((int(h['junction_node']) for h in e['osm'] if h.get('junction_node')), c.osm_node if c is not None else 0)
    sign_m = None if c is None or c.osm_sign_m is None else round(c.osm_sign_m, 1)
    rows.append(Row(round(e['end_m'], 1), e['key'], e['internal'], valhalla, sidecar, c.line_source if c is not None else '-', osm_node,
                    f"{c.kind} at {c.route_m:.0f} m" if c is not None else '-', e['anchor'] if e['internal'] else -1, sign_m))
  detail = 'none expected' if side is None else side.status + (f" ({side.detail})" if side.detail else '')
  total = sum(float(s.get('distance', 0.)) for s in steps)
  return Report(status, detail, rows, controls, total, uncovered(side, route_shape(steps), edges) if checked else [])


def signs_of(side) -> list[tuple[int, str, tuple[float, float], str, set[str], str]]:
  """Every OSM stop and give_way node the sidecar holds, once: (node, kind, (lat, lon), tags, the keys it is held
  under, why it is unplaced or '')."""
  out = {}
  for k, entries in side.controls.items():
    for x in entries:
      pos = tuple(x['line']) if x.get('line') else side.junction_pos(x.get('junction_node', 0))
      if pos is None:
        continue
      ident = x.get('node') or (round(pos[0], 7), round(pos[1], 7), x.get('kind'))
      tags = f"{x.get('evidence', '')}, stop={x.get('stop_tag', 'none')}"
      out.setdefault(ident, [int(x.get('node') or 0), x.get('kind', ''), (float(pos[0]), float(pos[1])), tags, set(), ''])[4].add(k)
  for u in side.unplaced:
    tags = f"direction={u.get('direction') or '-'}, stop={u.get('stop_tag', 'none')}"
    out.setdefault(u['node'], [int(u['node']), u.get('kind', ''), (float(u['lat']), float(u['lon'])), tags, set(), u.get('why', '')])
  return [tuple(v) for v in out.values()]


def uncovered(side, pts, edges) -> list[Sign]:
  """The OSM stop and give_way nodes within RAW_M of the route shape pts that no row covers (module docstring)."""
  if len(pts) < 2:
    return []
  cum = cumulative(pts)
  pad = RAW_M / M_PER_DEG * 2.
  lat_lo, lat_hi = min(p[0] for p in pts) - pad, max(p[0] for p in pts) + pad
  lon_lo, lon_hi = min(p[1] for p in pts) - pad * 2., max(p[1] for p in pts) + pad * 2.
  out = []
  for node, kind, pos, tags, keys, why in signs_of(side):
    if not (lat_lo <= pos[0] <= lat_hi and lon_lo <= pos[1] <= lon_hi):
      continue
    hit = nearest(pts, cum, pos, 0., cum[-1])
    if hit is None or hit[1] > RAW_M:
      continue
    m, off = hit
    e = next((e for e in edges if e['begin_m'] < m <= e['end_m'] + 1.), None)
    if e is not None and (any(node and h.get('node') == node for h in e['osm']) or kind in e['valhalla'] or kind in e['phantom']):
      continue  # its edge's row has it
    if why:
      holder = f"no approach ({why})"
    elif e is not None and e['key'] in keys:
      holder = 'ours'
    elif e is not None and f"{e['key'][:-1]}{'b' if e['key'].endswith('f') else 'f'}" in keys:
      holder = 'other direction'
    else:
      holder = 'way ' + ' '.join(sorted(keys))
    out.append(Sign(round(m, 1), round(off, 1), node, kind, tags, holder, e['key'] if e is not None else '-',
                    round(e['end_m'] - m, 1) if e is not None else 0.))
  return sorted(out, key=lambda s: s.route_m)


def lines(report: Report) -> list[str]:
  out = [f"Route {report.route_m / 1000:.2f} km: controls status {report.status}, OSM sidecar: {report.sidecar}",
         f"{'node m':>8}  {'way:dir':<16} {'Valhalla':<26} {'sidecar':<40} {'line':<8} {'sign m':>7} {'OSM node':>12}  control"]
  for r in report.rows:
    way = r.key + (f" (median, joins {r.joins})" if r.internal else '')
    sign = '-' if r.sign_m is None else f"{r.sign_m:.0f}"
    out.append(f"{r.node_m:>8.1f}  {way:<16} {r.valhalla:<26} {r.sidecar:<40} {r.line_source:<8} {sign:>7} {r.osm_node or '-':>12}  {r.control}")
  if not report.rows:
    out.append("(no stop or yield approach on the route)")
  out.append(f"{len(report.rows)} stop/yield approaches; navd sends {len(report.controls)} controls: " +
             ', '.join(f"{c.kind} {c.route_m:.0f} m" for c in report.controls))
  out.append(f"OSM stop/give_way nodes within {RAW_M:.0f} m of the route that no row covers (the car stops for none of them): " +
             f"{len(report.signs)}")
  if report.signs:
    out.append(f"{'route m':>8}  {'node':>12}  {'kind':<6} {'tags':<32} {'held for':<40} {'edge':<16} {'to its node':>11}")
  for g in report.signs:
    out.append(f"{g.route_m:>8.1f}  {g.node or '-':>12}  {g.kind:<6} {g.tags:<32} {g.holder:<40} {g.edge:<16} {g.to_node_m:>9.1f} m")
  return out


def as_json(report: Report) -> dict:
  return {'status': report.status, 'sidecar': report.sidecar, 'route_m': report.route_m, 'rows': [asdict(r) for r in report.rows],
          'signs': [asdict(g) for g in report.signs],
          'controls': [{'kind': c.kind, 'route_m': round(c.route_m, 1), 'line_source': c.line_source, 'osm_node': c.osm_node,
                        'osm_sign_m': None if c.osm_sign_m is None else round(c.osm_sign_m, 1), 'junction_status': c.junction_status}
                       for c in report.controls]}


def point(text: str) -> tuple[float, float]:
  lat, lon = (float(x) for x in text.split(','))
  return lat, lon


def main(argv=None):
  from openpilot.selfdrive.turnpilot.routing.valhalla_source import ValhallaSource
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('origin', type=point, help='lat,lon')
  ap.add_argument('destination', type=point, help='lat,lon')
  ap.add_argument('--tiles', required=True, help="what TurnPilotValhallaTiles takes: the tiles' directory, tiles.tar or valhalla.json")
  ap.add_argument('--sidecar', help="the directory with osm_controls/ and the tiles' stamp (default: the tiles' own)")
  ap.add_argument('--json', action='store_true')
  args = ap.parse_args(argv)
  source = ValhallaSource(args.tiles)
  tiles_dir = source.tiles_dir
  if tiles_dir is not None and not (Path(tiles_dir) / MARKER).exists() and (Path(tiles_dir) / 'meta.json').is_file():
    print(f"note: {tiles_dir} has no {MARKER} marker: read as a map-server region built from its PBF", file=sys.stderr)
    tiles_dir = None
  sidecar_dir = args.sidecar or source.tiles_dir or osm_sidecar.NO_DIRECTORY
  if args.sidecar and source.tiles_dir and osm_sidecar.tiles_stamp(Path(source.tiles_dir)) != osm_sidecar.tiles_stamp(Path(args.sidecar)):
    print(f"warning: the tiles' osm_timestamp ({osm_sidecar.tiles_stamp(Path(source.tiles_dir))}) isn't the sidecar directory's " +
          f"({osm_sidecar.tiles_stamp(Path(args.sidecar))})", file=sys.stderr)
  (olat, olon), (dlat, dlon) = args.origin, args.destination
  steps = source.route((olon, olat), (dlon, dlat), None, None)
  if not steps:
    raise SystemExit("no route between them on these tiles")
  report = audit(source._call(), steps, tiles_dir, sidecar_dir)
  print(json.dumps(as_json(report), indent=1) if args.json else '\n'.join(lines(report)))


if __name__ == '__main__':
  main()
