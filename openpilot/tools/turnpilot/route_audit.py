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
(osm: the OSM stop node; sim: the sim's controls.json; default: the node less 10 m), the OSM junction node, and the control
navd sends (its kind and stop point). An internal-intersection edge's row says which junction's control it joined.
"""
import argparse
import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

from openpilot.selfdrive.turnpilot.routing import osm_sidecar
from openpilot.selfdrive.turnpilot.routing.controls import MARKER, route_controls, route_shape


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


@dataclass
class Report:
  status: str          # navInstruction.controlsStatus
  sidecar: str         # the sidecar's status and why
  rows: list[Row] = field(default_factory=list)
  controls: list = field(default_factory=list)
  route_m: float = 0.


def audit(call, steps, tiles_dir, sidecar_dir) -> Report:
  """The rows for a route's steps, through call (routing/controls.py's actor_call or http_call)."""
  edges: list[dict] = []
  controls, status = route_controls(call, steps, tiles_dir, sidecar_dir, audit=edges)
  side = osm_sidecar.load(sidecar_dir, route_shape(steps)) if sidecar_dir is not None else None
  checked = side is not None and side.status == 'ok'
  by_edge = {c.edge: c for c in controls}
  rows = []
  for e in edges:
    valhalla = ' '.join(e['valhalla'] + [f"{k} (phantom: dropped)" for k in e['phantom']]) or '-'
    sidecar = ', '.join(f"{h['kind']} ({h['evidence']}, stop={h['stop_tag']})" for h in e['osm']) or '-' if checked else 'not checked'
    c = by_edge.get(e['anchor'])
    osm_node = next((int(h['junction_node']) for h in e['osm'] if h.get('junction_node')), c.osm_node if c is not None else 0)
    rows.append(Row(round(e['end_m'], 1), e['key'], e['internal'], valhalla, sidecar, c.line_source if c is not None else '-', osm_node,
                    f"{c.kind} at {c.route_m:.0f} m" if c is not None else '-', e['anchor'] if e['internal'] else -1))
  detail = 'none expected' if side is None else side.status + (f" ({side.detail})" if side.detail else '')
  total = sum(float(s.get('distance', 0.)) for s in steps)
  return Report(status, detail, rows, controls, total)


def lines(report: Report) -> list[str]:
  out = [f"Route {report.route_m / 1000:.2f} km: controls status {report.status}, OSM sidecar: {report.sidecar}",
         f"{'node m':>8}  {'way:dir':<16} {'Valhalla':<26} {'sidecar':<40} {'line':<8} {'OSM node':>12}  control"]
  for r in report.rows:
    way = r.key + (f" (median, joins {r.joins})" if r.internal else '')
    out.append(f"{r.node_m:>8.1f}  {way:<16} {r.valhalla:<26} {r.sidecar:<40} {r.line_source:<8} {r.osm_node or '-':>12}  {r.control}")
  if not report.rows:
    out.append("(no stop or yield approach on the route)")
  out.append(f"{len(report.rows)} stop/yield approaches; navd sends {len(report.controls)} controls: " +
             ', '.join(f"{c.kind} {c.route_m:.0f} m" for c in report.controls))
  return out


def as_json(report: Report) -> dict:
  return {'status': report.status, 'sidecar': report.sidecar, 'route_m': report.route_m, 'rows': [asdict(r) for r in report.rows],
          'controls': [{'kind': c.kind, 'route_m': round(c.route_m, 1), 'line_source': c.line_source, 'osm_node': c.osm_node,
                        'junction_status': c.junction_status} for c in report.controls]}


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
