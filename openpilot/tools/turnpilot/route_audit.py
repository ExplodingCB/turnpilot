#!/usr/bin/env python3
"""Every stop and yield approach on a route, from both map sources: Valhalla's flag, the OSM stop-sign sidecar's, the stop
line's source and the OSM junction node (STOP_SIGNS.md, SG1a; step S2 runs it on each demo trip).

  python -m openpilot.tools.turnpilot.route_audit --tiles DIR [--sidecar DIR] [--surveys FILE] [--overrides FILE]
                                                 [--json | --survey-template] ORIGIN DESTINATION

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

Then each stop and yield's junction (SG1b, routing/junctions.py): its OSM node, status, maneuver, where its stop point is
and why (default, mappedLine, survey), the sidecar, the survey it matched (and whether it counts), the plausibility gate,
the rail distance, and one line per road there: its role, bearing, class, use, lanes, Valhalla's control, the OSM stop
tag, the all-way tier, the assumed limit and its source (survey, tagged, classDefault) and the way. --surveys reads a
surveyed_junctions.json other than the shipped one; --overrides a JSON file holding what TurnPilotJunctionOverrides would
(none by default: the car's param isn't on this machine).

--survey-template prints, instead, a surveyed_junctions.json entry for each candidate all-way on the trip (every other
road there tiered mapStrong or better): what the map knows filled in (the OSM node, its position, and per approach the
way, the OSM bearing toward the junction and, as hintTaggedMph, the tagged limit), and every field Kellen measures null,
so the entry doesn't load until it is filled (STOP_SIGNS.md 4.3, step S2).
"""
import argparse
import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

from openpilot.selfdrive.turnpilot.maps import surveys
from openpilot.selfdrive.turnpilot.routing import junctions, osm_sidecar
from openpilot.selfdrive.turnpilot.routing.controls import MARKER, cumulative, nearest, route_controls, route_shape

RAW_M = 5.           # m from the route an OSM stop or give_way node is listed within
M_PER_DEG = 111195.
MPH = 0.44704
CANDIDATE_TIERS = ('mapStrong', 'surveyed')  # a junction whose every other inbound road is one of these is a survey candidate


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
class JunctionRow:
  route_m: float       # m along the route of the control's stop point
  kind: str            # 'stop' or 'yield'
  osm_node: int        # the OSM junction node, 0 not known
  lat: float
  lon: float
  status: str          # MapJunction.status
  maneuver: str
  turn_angle: float    # deg, + right
  line_source: str     # the stop point's source: default, mappedLine or survey
  stop_point_m: float  # m from the stop point to the node
  sidecar: bool
  suspected: bool
  divided: bool
  roundabout: bool
  rail_m: float        # railDistance (-1 none, -2 unknown)
  plausible: bool      # the plausibility gate (STOP_SIGNS.md 4.2)
  survey: str          # 'none', 'matched, counts' or "matched, doesn't count: <why>"
  legs: list[dict] = field(default_factory=list)  # per road: role, bearing, rel, class, use, in/out, lanes, controls, tier, limits, way


@dataclass
class Report:
  status: str          # navInstruction.controlsStatus
  sidecar: str         # the sidecar's status and why
  rows: list[Row] = field(default_factory=list)
  controls: list = field(default_factory=list)
  route_m: float = 0.
  signs: list[Sign] = field(default_factory=list)  # OSM stop and give_way nodes near the route that no row covers
  junctions: list[JunctionRow] = field(default_factory=list)  # each stop and yield's junction


def junction_row(c) -> JunctionRow:
  j = c.junction_info
  survey = 'none' if j.survey is None else 'matched, counts' if j.surveyed else f"matched, doesn't count: {j.survey_note}"
  legs = [{'role': a.role, 'bearing': round(a.bearing, 1), 'rel': round(a.rel_bearing, 1), 'class': a.road_class, 'use': a.use,
           'inbound': a.inbound, 'outbound': a.outbound, 'lanes_in': a.lanes_in, 'lanes_out': a.lanes_out, 'valhalla': a.valhalla_control,
           'osm_stop': a.osm_stop, 'tier': a.tier, 'tagged_mph': round(a.tagged_limit / MPH, 1), 'assumed_mph': round(a.assumed_limit / MPH, 1),
           'speed_source': a.speed_source, 'way_id': a.way_id, 'bike_lane': a.bike_lane, 'shoulder_m': a.shoulder_m} for a in j.approaches]
  return JunctionRow(round(c.route_m, 1), c.kind, j.osm_node_id, round(j.lat, 7), round(j.lon, 7), j.status, j.maneuver,
                     round(j.turn_angle, 1), j.stop_point_source, round(j.stop_point_distance, 1), j.sidecar, j.suspected, j.divided,
                     j.roundabout, round(j.rail_distance, 1), j.plausible, survey, legs)


def survey_template(report: Report) -> list[dict]:
  """A surveyed_junctions.json entry for each candidate all-way among the report's controls (module docstring)."""
  out = []
  for c in report.controls:
    j = c.junction_info
    inbound = [a for a in j.approaches if a.inbound] if j is not None else []
    if j is None or not j.osm_node_id or not inbound or any(a.tier not in CANDIDATE_TIERS for a in inbound):
      continue

    def blank(way, toward, limit):
      return {'wayId': int(way), 'bearingDeg': round(toward % 360., 1), 'stops': None, 'plaque': None, 'postedMph': None, 'laneEdgeM': None,
              'lineM': None, 'nearestParkingM': {'left': None, 'right': None}, 'sightM': None,
              'hintTaggedMph': round(limit / MPH) if limit > 0. else None}
    approaches = [blank(j.our_way, j.heading_in, j.our_limit)] + [blank(a.way_id, a.bearing + 180., a.tagged_limit) for a in inbound]
    out.append({'osmNodeId': j.osm_node_id, 'lat': round(j.lat, 7), 'lon': round(j.lon, 7), 'date': None, 'source': None,
                'imageryDate': None, 'confirmedBy': [], 'approaches': approaches, 'validations': []})
  return out


def audit(call, steps, tiles_dir, sidecar_dir, context: junctions.Context | None = None) -> Report:
  """The rows for a route's steps, through call (routing/controls.py's actor_call or http_call); context: the surveys and
  overrides the junctions are read against (none by default)."""
  edges: list[dict] = []
  controls, status = route_controls(call, steps, tiles_dir, sidecar_dir, audit=edges, context=context)
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
  return Report(status, detail, rows, controls, total, uncovered(side, route_shape(steps), edges) if checked else [],
                [junction_row(c) for c in controls if c.junction_info is not None])


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
  out.append(f"{len(report.junctions)} stop/yield junctions:")
  for j in report.junctions:
    rail = {-1.: 'none', -2.: 'unknown'}.get(j.rail_m, f"{j.rail_m:.0f} m")
    flags = ', '.join(x for x, on in (('suspected', j.suspected), ('divided', j.divided), ('roundabout', j.roundabout),
                                       ('implausible all-way', not j.plausible)) if on)
    out.append(f"  {j.kind} at {j.route_m:.0f} m, node {j.osm_node or '-'} ({j.lat:.6f}, {j.lon:.6f}): status {j.status}, {j.maneuver} " +
               f"{j.turn_angle:+.0f} deg, line {j.line_source} {j.stop_point_m:.1f} m before the node, sidecar {'ok' if j.sidecar else 'no'}, " +
               f"rail {rail}, survey {j.survey}" + (f", {flags}" if flags else ''))
    out.append(f"    {'role':<9} {'bearing':>7} {'rel':>5}  {'class':<13} {'use':<12} {'in/out':<7} {'lanes':<6} {'Valhalla':<9} " +
               f"{'OSM stop':<9} {'tier':<10} {'limit (source)':<24} way")
    for a in j.legs:
      way = '/'.join(x for x, on in (('in', a['inbound']), ('out', a['outbound'])) if on) or '-'
      limit = f"{a['assumed_mph']:.0f} mph ({a['speed_source']})"
      out.append(f"    {a['role']:<9} {a['bearing']:>7.0f} {a['rel']:>+5.0f}  {a['class']:<13} {a['use']:<12} {way:<7} " +
                 f"{a['lanes_in']}/{a['lanes_out']:<4} {a['valhalla']:<9} {a['osm_stop']:<9} {a['tier']:<10} {limit:<24} {a['way_id'] or '-'}")
  return out


def as_json(report: Report) -> dict:
  return {'status': report.status, 'sidecar': report.sidecar, 'route_m': report.route_m, 'rows': [asdict(r) for r in report.rows],
          'signs': [asdict(g) for g in report.signs], 'junctions': [asdict(j) for j in report.junctions],
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
  ap.add_argument('--surveys', help='a surveyed_junctions.json (default: the shipped one)')
  ap.add_argument('--overrides', help="a JSON file holding a TurnPilotJunctionOverrides value (default: none)")
  ap.add_argument('--survey-template', action='store_true', help='print survey entries for the candidate all-ways instead')
  args = ap.parse_args(argv)
  loaded = surveys.load(Path(args.surveys) if args.surveys else surveys.SHIPPED)
  for e in loaded.errors:
    print(f"surveys: {e}", file=sys.stderr)
  overrides, ok, why = ({}, True, '') if not args.overrides else \
    junctions.read_overrides(type('File', (), {'get': staticmethod(lambda key: Path(args.overrides).read_text())})())
  if not ok:
    print(f"overrides: {why}", file=sys.stderr)
  context = junctions.Context(loaded.entries, overrides, ok, why)
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
  report = audit(source._call(), steps, tiles_dir, sidecar_dir, context)
  if args.survey_template:
    print(json.dumps(survey_template(report), indent=1))
  else:
    print(json.dumps(as_json(report), indent=1) if args.json else '\n'.join(lines(report)))


if __name__ == '__main__':
  main()
