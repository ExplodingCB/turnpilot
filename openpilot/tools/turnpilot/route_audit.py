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
another way, no approach (unplaced: on no drivable way, or a stop=minor with no lower approach), or an orphan left out (tied
to no junction, its sign more than ORPHAN_NEAR_M before its edge's end, where the edge's end would put the stop: routing/
controls.py). The car stops for none of them; the auditor checks each. An orphan with a rail crossing within RAIL_M past
its sign is no such sign: it stops the car at its sign, and its edge's row says so, listing that stop beside any at the
edge's end.

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

SG2 (STOP_SIGNS.md 3.2-3.5): each junction's certification and reason (routing/certify.py, against --visibility, by default
the shipped file, with --mode and the side views as navd would have them once SG4a publishes them: auto and on by default,
and the light at the ETA from --eta, by default now); its predicted reach per conflicting lane from PV1's conservative and
optimistic tables with the survey's caps, at the closest stop point section 1.2 allows and at node - 13 m (candidates for a
survey: PV1 nominates, validation decides); navd's avoidance loop on the trip (routing/avoid.py) and the time it added; for
each certified all-way where the route turns, the first stop or yield straight on from it (a missed turn runs straight);
and the TurnPilotJunctionOverrides entries with their evidence.

  python -m openpilot.tools.turnpilot.route_audit --check-survey FILE      the schema check and each junction's predicted
                                                                           reach, per arriving approach, with its caps
  python -m openpilot.tools.turnpilot.route_audit --overrides FILE --list-overrides | --clear APPROACH
                                                  (--params in place of --overrides on the car: the param itself)
--clear removes one override after Kellen has looked at its evidence (decision D11), with the keys written together for the
same event (turnpilotd names both directions of a downgraded approach's way, every way of a junction a signal head was seen
at: one left behind would still fail the junction); an approach not there is said. --list-overrides shows each event once,
a downgrade on site read out: the vehicle, the side it came from, its speed through its line and the least it read in the
20 m before, the track, the time and the log event (turnpilot.stop_go_downgrade) to find it by.
"""
import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from datetime import UTC, datetime
from pathlib import Path

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot import visibility_table
from openpilot.selfdrive.turnpilot.maps import surveys
from openpilot.selfdrive.turnpilot.routing import avoid, certify, junctions, osm_sidecar
from openpilot.selfdrive.turnpilot.routing.controls import MARKER, controls_ahead, cumulative, nearest, route_controls, route_shape

RAW_M = 5.           # m from the route an OSM stop or give_way node is listed within
M_PER_DEG = 111195.
MPH = 0.44704
CANDIDATE_TIERS = ('mapStrong', 'surveyed')  # a junction whose every other inbound road is one of these is a survey candidate
DEFAULT_SETBACK_M = 1.9  # m, the camera setback STOP_SIGNS.md's tables use until step S1 measures it
NODE_13_M = 13.          # m, the default stop point on a 2-lane cross road, where predicted reach is also given
CHECK_MANEUVER = 'straight'  # --check-survey's lanes: both cross lanes and the opposing approach


@dataclass
class Row:
  node_m: float        # m along the route to the edge's end
  key: str             # '<way_id>:<f|b>'
  internal: bool
  valhalla: str        # Valhalla's flags on the edge, or '-'
  sidecar: str         # the sidecar's entries for it, '-' for none, 'not checked' without a valid sidecar
  line_source: str     # the control's line source there, '-' with no control
  osm_node: int        # the OSM junction node the sidecar names (0 none)
  control: str         # the controls navd sends there (a rail orphan's at its sign, then the edge's end's), '-' for none
  joins: int = -1      # an internal edge's: the trace edge whose junction its control is
  sign_m: float | None = None  # m along the route of the OSM stop node on the control's edge (data only), None for none


@dataclass
class Sign:
  route_m: float       # m along the route (its shape) of the node's nearest point on it
  off_m: float         # m off the route
  node: int            # the OSM node id (0 for a shard without it)
  kind: str            # 'stop' or 'yield'
  tags: str            # its direction and stop tags
  holder: str          # who the sidecar has it for: 'ours', 'other direction', 'way <keys>', 'no approach (<why>)',
                       # 'orphan (sign N m before its edge's end)' (left out: routing/controls.py's ORPHAN_NEAR_M; one
                       # before a rail crossing is placed, on its edge's row)
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
  certification: str = 'unchecked'  # routing/certify.py (SG2)
  cert_reason: str = 'none'
  cert_detail: str = ''
  predicted: list[dict] = field(default_factory=list)  # per stop point, bound and conflicting lane: reach, need, cap (PV1)


@dataclass
class Report:
  status: str          # navInstruction.controlsStatus
  sidecar: str         # the sidecar's status and why
  rows: list[Row] = field(default_factory=list)
  controls: list = field(default_factory=list)
  route_m: float = 0.
  signs: list[Sign] = field(default_factory=list)  # OSM stop and give_way nodes near the route that no row covers
  junctions: list[JunctionRow] = field(default_factory=list)  # each stop and yield's junction
  overrides: dict = field(default_factory=dict)  # TurnPilotJunctionOverrides: approach -> {kind, evidence}
  continuations: list[dict] = field(default_factory=list)  # straight on from each certified all-way where the route turns
  visibility: object = None
  runtime: object = None
  eta: float | None = None
  entries: list = field(default_factory=list)


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


def predicted_reach(j: sr.Junction, maneuver: str, visibility, entry: dict | None) -> list[dict]:
  """Each conflicting lane's predicted reach from PV1 (the provisional tables, read as candidates) and the survey's caps,
  conservative and optimistic, at the closest stop point section 1.2 allows (the surveyed edge + 5.8 m, else the default
  rule) and at node - 13 m, with the setback S1 measured or DEFAULT_SETBACK_M (STOP_SIGNS.md 2.8)."""
  setback = visibility.setback if visibility is not None and visibility.setback is not None else DEFAULT_SETBACK_M
  ours = junctions.survey_approach(entry, j.our_way, j.heading_in) if entry else None
  lane_edge = ours.get('laneEdgeM') if ours else None
  n = certify.cross_lanes(j)
  if lane_edge is not None:
    closest = lane_edge + surveys.STOP_CLEAR_M + surveys.POS_ERR_M + surveys.SURVEY_ERR_M
  else:
    closest = max(11., (n or sr.DEFAULT_CROSS_LANES) * sr.LANE_W_MAX / 2. + sr.SHOULDER_M + sr.NODE_ERR_M + sr.POS_ERR_M + surveys.STOP_CLEAR_M)
  rot = sr.angle_bound(lane_edge is not None)
  out = []
  for stop_m in sorted({round(closest, 1), NODE_13_M}):
    edges = sr.edge_band(stop_m, n, lane_edge)
    lanes = sr.conflict_lanes(j, maneuver, edges, float(setback))
    for bound in ('conservative', 'optimistic'):
      views = visibility.views('day', bound, candidates=True) if visibility is not None else {}
      sight = visibility.sight(views) if visibility is not None else sr.Sight({'left': visibility_table.DEFAULT_EDGE_DEG,
                                                                               'right': visibility_table.DEFAULT_EDGE_DEG}, views)
      for ln in lanes:
        cap, why = certify.lane_cap(ln, entry) if entry else (None, 'no survey')
        reach = sr.reach_walk(ln, sight, rot, cap_m=cap)
        out.append({'stop_m': stop_m, 'bound': bound, 'role': ln.role, 'index': ln.index, 'movement': ln.movement,
                    'reach': round(reach, 1), 'need': round(certify.lane_need(ln, edges), 1), 'cap': why,
                    'capped': cap is not None and reach + sr.WALK_STEP_M >= cap})
  return out


def predicted_lines(predicted: list[dict], indent: str = '    ') -> list[str]:
  out = []
  for stop_m in sorted({p['stop_m'] for p in predicted}):
    out.append(f"{indent}predicted reach (PV1) with the stop point at node - {stop_m:.1f} m:")
    rows = [p for p in predicted if p['stop_m'] == stop_m]
    for key in dict.fromkeys((p['role'], p['index'], p['movement']) for p in rows):
      cons = next(p for p in rows if (p['role'], p['index'], p['movement']) == key and p['bound'] == 'conservative')
      opt = next(p for p in rows if (p['role'], p['index'], p['movement']) == key and p['bound'] == 'optimistic')
      capped = f", capped by {cons['cap']}" if cons['capped'] or opt['capped'] else f" ({cons['cap']})"
      out.append(f"{indent}  {key[0]} lane {key[1]} {key[2]}: {cons['reach']:.1f} m conservative, {opt['reach']:.1f} m optimistic, " +
                 f"of {cons['need']:.1f} m needed{capped}")
  return out


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


def audit(call, steps, tiles_dir, sidecar_dir, context: junctions.Context | None = None, visibility=None, runtime=None,
          eta: float | None = None) -> Report:
  """The rows for a route's steps, through call (routing/controls.py's actor_call or http_call); context: the surveys and
  overrides the junctions are read against (none by default); visibility (the shipped file by default), runtime (auto, the
  side views on) and eta (Unix time at the route's start, now by default): what each junction is certified against."""
  edges: list[dict] = []
  controls, status = route_controls(call, steps, tiles_dir, sidecar_dir, audit=edges, context=context)
  if visibility is None:
    try:
      visibility = visibility_table.load(visibility_table.SHIPPED)
    except visibility_table.VisibilityError:
      visibility = None
  runtime = runtime if runtime is not None else certify.Runtime(mode='auto', side_views=True)
  eta = eta if eta is not None else datetime.now(UTC).timestamp()
  entries = list(context.surveys) if context is not None else []
  side = osm_sidecar.load(sidecar_dir, route_shape(steps)) if sidecar_dir is not None else None
  checked = side is not None and side.status == 'ok'
  by_edge = {c.edge: c for c in controls}
  rows = []
  for e in edges:
    if not (e['osm'] or e['rail'] or e['phantom'] or 'stop' in e['valhalla'] or 'yield' in e['valhalla']):
      continue
    valhalla = ' '.join(e['valhalla'] + [f"{k} (phantom: dropped)" for k in e['phantom']]) or '-'
    sidecar = ', '.join([f"{h['kind']} ({h['evidence']}, stop={h['stop_tag']})" for h in e['osm']] +
                        [f"{h['kind']} ({h['evidence']}, stop={h['stop_tag']}, an orphan {h['rail_m'] - h['m']:.0f} m before a rail crossing: " +
                         "stops at its sign)" for h in e['rail']] +
                        [f"{h['kind']} ({h['evidence']}, stop={h['stop_tag']}, an orphan {h['before_m']:.0f} m before its edge's end: left out)"
                         for h in e['orphans']]) or '-' if checked else 'not checked'
    c = by_edge.get(e['anchor'])
    osm_node = next((int(h['junction_node']) for h in e['osm'] if h.get('junction_node')), c.osm_node if c is not None else 0)
    sign_m = None if c is None or c.osm_sign_m is None else round(c.osm_sign_m, 1)
    sent = ', '.join(f"{x.kind} at {x.route_m:.0f} m" for x in controls if x.edge == e['anchor']) or '-'
    rows.append(Row(round(e['end_m'], 1), e['key'], e['internal'], valhalla, sidecar, c.line_source if c is not None else '-', osm_node,
                    sent, e['anchor'] if e['internal'] else -1, sign_m))
  detail = 'none expected' if side is None else side.status + (f" ({side.detail})" if side.detail else '')
  total = sum(float(s.get('distance', 0.)) for s in steps)
  rows_j = []
  for c in controls:
    if c.junction_info is None:
      continue
    row = junction_row(c)
    j = sr.junction_from_control(c)
    v = certify.assess(j, j.maneuver, visibility, entries, eta + avoid.time_at(steps, c.route_m), runtime)
    c.junction_info.certification, c.junction_info.cert_reason, c.junction_info.cert_detail = v.certification, v.reason, v.detail
    c.certification, c.cert_reason = v.certification, v.reason
    row.certification, row.cert_reason, row.cert_detail = v.certification, v.reason, v.detail
    entry = surveys.match(entries, j.osm_node_id, (j.lat, j.lon))
    row.predicted = predicted_reach(j, j.maneuver, visibility, entry)
    rows_j.append(row)
  return Report(status, detail, rows, controls, total, uncovered(side, route_shape(steps), edges) if checked else [], rows_j,
                dict(context.overrides) if context is not None else {}, [], visibility, runtime, eta, entries)


def audit_avoidance(report: Report, steps, reroute, route_controls, eta: float | None = None, headings=None) -> avoid.AvoidResult:
  """navd's avoidance loop (routing/avoid.py) on the report's route, certified as the report was; reroute(exclude) -> steps."""
  eta = report.eta if eta is None else eta

  def certify_one(c, on_steps):
    return certify.certify_control(c, report.visibility, report.entries, eta + avoid.time_at(on_steps, c.route_m), report.runtime)
  return avoid.run(steps, list(report.controls), report.status, avoid.AvoidSet(), certify=certify_one, reroute=reroute,
                   route_controls=route_controls, own_edge=lambda: None, cancelled=lambda: False, deadline=math.inf,
                   headings=headings or (lambda points: [None for _ in points]))


def avoidance_lines(result: avoid.AvoidResult) -> list[str]:
  n = result.avoided
  out = [f"avoidance: {result.status}, {n} stop{'' if n == 1 else 's'} routed around in {result.iterations} reroutes, " +
         f"{result.added_s:+.0f} s on the route"]
  left = [c for c in avoid.stops(result.controls) if c.certification == 'unavoidable']
  if left:
    out.append(f"  {len(left)} left on the route need the driver: " + ', '.join(f"{c.kind} at {c.route_m:.0f} m" for c in left))
  return out


def continuations(controls, lookahead, certify) -> list[dict]:
  """For each certified all-way where the route turns, the first stop or yield straight on from its node (a missed turn
  runs straight, N12): lookahead(position (lat, lon), heading) -> (controls, status, road) as navd's lookup ahead;
  certify(control) -> (certification, reason), for the union of maneuvers."""
  out = []
  for c in controls:
    j = c.junction_info
    if j is None or j.certification != 'certifiable' or j.maneuver not in ('left', 'right'):
      continue
    ahead, _, _ = lookahead((j.lat, j.lon), j.heading_in)
    nxt = next((x for x in sorted(ahead, key=lambda x: x.route_m) if x.kind in ('stop', 'yield')), None)
    row = {'control': c.id, 'route_m': round(c.route_m, 1), 'maneuver': j.maneuver, 'next': 'none', 'next_m': None,
           'certification': 'none', 'reason': 'none'}
    if nxt is not None:
      cert, reason = certify(nxt)
      row |= {'next': nxt.kind, 'next_m': round(nxt.route_m, 1), 'certification': cert, 'reason': reason}
    out.append(row)
  return out


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
    if e is not None and (any(node and h.get('node') == node for h in e['osm'] + e['rail']) or kind in e['valhalla'] or kind in e['phantom']):
      continue  # its edge's row has it
    orphan = next((h for h in e['orphans'] if (node and h.get('node') == node) or (not node and h.get('line') and
                   (round(h['line'][0], 7), round(h['line'][1], 7)) == (round(pos[0], 7), round(pos[1], 7)))), None) if e is not None else None
    if orphan is not None:
      holder = f"orphan (sign {orphan['before_m']:.0f} m before its edge's end)"
    elif why:
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
    out.append(f"    certification: {j.certification} ({j.cert_reason})" + (f": {j.cert_detail}" if j.cert_detail else ''))
    out += predicted_lines(j.predicted)
  for r in report.continuations:
    nxt = 'nothing within the lookup' if r['next'] == 'none' else \
      f"{r['next']} {r['next_m']:.0f} m on, {r['certification']} ({r['reason']})"
    out.append(f"straight on from the certified all-way at {r['route_m']:.0f} m (the route turns {r['maneuver']}): {nxt}")
  out += override_lines(report.overrides)
  return out


def override_event(key: str, o: dict) -> tuple:
  """The keys written together with key for one event (its evidence's written: turnpilotd names both directions of a
  way, or every way of a junction, at once), key alone for an entry that doesn't say."""
  ev = o.get('evidence')
  written = ev.get('written') if isinstance(ev, dict) else None
  return tuple(written) if isinstance(written, list) and key in written else (key,)


def evidence_text(kind: str, ev) -> str:
  """An override's evidence read out: a downgrade on site (stop_go.py: the vehicle, its side, its speeds, the heading clause,
  the track, when, where and the log event to find it by), a signal head; anything else as it is."""
  if not isinstance(ev, dict) or 'source' not in ev:
    return f"evidence {ev!r}"
  when = datetime.fromtimestamp(float(ev['wallTime']), tz=UTC).strftime('%Y-%m-%d %H:%M:%SZ') \
    if isinstance(ev.get('wallTime'), (int, float)) else 'time unknown'
  if kind == 'no' and 'speedMs' in ev:
    return ' '.join([f"a {ev.get('cls', 'vehicle')} through the {ev.get('role', '?')} approach's line at {float(ev['speedMs']):.1f} m/s,",
                     f"no slower than {float(ev.get('minSpeedMs', 0.)):.1f} m/s over the {float(ev.get('windowM', 0.)):.0f} m before it",
                     f"(threshold {float(ev.get('thresholdMs', 0.)):.1f} m/s, heading {ev.get('heading', 'unmeasured')}); track",
                     f"{ev.get('trackId')} on {ev.get('view', '?')}, {when}, junction {ev.get('osmNodeId')} way {ev.get('wayId')},",
                     f"{ev.get('log', 'no log event')} at camera time {ev.get('cameraTimeS')} s ({ev['source']})"])
  return f"{ev['source']}, {when}, ways {ev.get('ways')}" if 'ways' in ev else f"{ev['source']}, {when}: {ev!r}"


def override_lines(overrides: dict) -> list[str]:
  """Every override with its evidence, one event (the keys written together) once (decision D11)."""
  out = [f"TurnPilotJunctionOverrides: {len(overrides)}"]
  done = set()
  for key, o in overrides.items():
    if key in done:
      continue
    keys = [k for k in override_event(key, o) if k in overrides] or [key]
    done.update(keys)
    clear = f"route_audit --clear {keys[0]} once looked at" + (f", which clears {', '.join(keys[1:])} with it" if len(keys) > 1 else '')
    out.append(f"  {' '.join(keys)} {o['kind']}: {evidence_text(o['kind'], o.get('evidence'))}  ({clear})")
  return out


def as_json(report: Report) -> dict:
  return {'status': report.status, 'sidecar': report.sidecar, 'route_m': report.route_m, 'rows': [asdict(r) for r in report.rows],
          'signs': [asdict(g) for g in report.signs], 'junctions': [asdict(j) for j in report.junctions],
          'continuations': report.continuations, 'overrides': report.overrides,
          'controls': [{'kind': c.kind, 'route_m': round(c.route_m, 1), 'line_source': c.line_source, 'osm_node': c.osm_node,
                        'osm_sign_m': None if c.osm_sign_m is None else round(c.osm_sign_m, 1), 'junction_status': c.junction_status}
                       for c in report.controls]}


def point(text: str) -> tuple[float, float]:
  lat, lon = (float(x) for x in text.split(','))
  return lat, lon


def survey_junction(entry: dict, ours: dict) -> sr.Junction:
  """A surveyed junction as the certifier sees it from one arriving approach (ours): the other approaches by their
  bearings, one lane each (a survey has no lane counts), stopping as surveyed, at their posted limit (D8's 30 mph if none)."""
  heading = float(ours['bearingDeg'])
  approaches = []
  for a in entry['approaches']:
    if a is ours:
      continue
    leg = (float(a['bearingDeg']) + 180.) % 360.
    rel = junctions.rel(leg, heading)
    mph = a['postedMph'] if a['postedMph'] is not None else sr.class_limit_mph('residential')
    approaches.append(sr.Approach(leg, rel, junctions.role_of(rel), 'residential', 'road', True, True, 1, 1, 'stop' if a['stops'] else 'none',
                                  'surveyed' if a['stops'] else 'no', mph * sr.MPH, int(a['wayId'])))
  return sr.Junction(osm_node_id=int(entry['osmNodeId']), lat=float(entry['lat']), lon=float(entry['lon']), heading_in=heading,
                     maneuver=CHECK_MANEUVER, rail_distance=-1., surveyed=True, our_way=int(ours['wayId']), approaches=approaches)


def check_survey(path: Path, visibility=None) -> int:
  """--check-survey: 0 when every entry loads, else 1; prints each loaded junction's predicted reach per arriving approach."""
  loaded = surveys.load(path)
  for e in loaded.errors:
    print(f"error: {e}")
  print(f"{path}: {'schema ok' if not loaded.errors else 'errors above'}, {len(loaded.entries)} junctions load")
  for entry in loaded.entries:
    print(f"junction {entry['osmNodeId']} ({entry['lat']:.6f}, {entry['lon']:.6f}), surveyed {entry['date']} ({entry['source']}):")
    for ours in entry['approaches']:
      j = survey_junction(entry, ours)
      limits = sorted({round(a.assumed_limit / sr.MPH) for a in j.approaches if a.tier == 'surveyed'})
      print(f"  arriving heading {ours['bearingDeg']:.0f} (way {ours['wayId']}), cross traffic at {', '.join(f'{m} mph' for m in limits) or '-'}" +
            (f", its stopPointM {ours['stopPointM']:.1f} m" if ours.get('stopPointM') is not None else ''))
      for line in predicted_lines(predicted_reach(j, CHECK_MANEUVER, visibility, entry), '    '):
        print(line)
  return 1 if loaded.errors else 0


def cleared_with(items: list, key: str) -> set:
  """The keys --clear key removes: key and those written with it for the same event (override_event), since one left
  behind still fails the junction (certify: any approach overridden)."""
  o = next((x for x in items if x.get('approach') == key), None)
  return set(override_event(key, o)) | {key} if o is not None else set()


def clear_override_file(path: Path, key: str) -> list[str]:
  """Remove approach key, with those written with it, from a file holding a TurnPilotJunctionOverrides value: the keys
  removed ([]: not there, nothing written)."""
  items = json.loads(Path(path).read_text() or '[]')
  gone = cleared_with(items, key)
  if not gone:
    return []
  Path(path).write_text(json.dumps([o for o in items if o.get('approach') not in gone]))
  return [o['approach'] for o in items if o.get('approach') in gone]


def clear_override_param(params, key: str) -> list[str]:
  """The same on the car's TurnPilotJunctionOverrides param (a STRING key navd parses, json.dumps of the list)."""
  raw = params.get(junctions.OVERRIDES_PARAM)
  items = json.loads(raw.decode() if isinstance(raw, bytes) else raw or '[]')
  gone = cleared_with(items, key)
  if not gone:
    return []
  params.put(junctions.OVERRIDES_PARAM, json.dumps([o for o in items if o.get('approach') not in gone]))
  return [o['approach'] for o in items if o.get('approach') in gone]


def main(argv=None):
  from openpilot.selfdrive.turnpilot.routing.valhalla_source import ValhallaSource
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('origin', type=point, nargs='?', help='lat,lon')
  ap.add_argument('destination', type=point, nargs='?', help='lat,lon')
  ap.add_argument('--tiles', help="what TurnPilotValhallaTiles takes: the tiles' directory, tiles.tar or valhalla.json")
  ap.add_argument('--sidecar', help="the directory with osm_controls/ and the tiles' stamp (default: the tiles' own)")
  ap.add_argument('--json', action='store_true')
  ap.add_argument('--surveys', help='a surveyed_junctions.json (default: the shipped one)')
  ap.add_argument('--overrides', help="a JSON file holding a TurnPilotJunctionOverrides value (default: none)")
  ap.add_argument('--params', action='store_true', help="read (and --clear) the car's TurnPilotJunctionOverrides param itself")
  ap.add_argument('--survey-template', action='store_true', help='print survey entries for the candidate all-ways instead')
  ap.add_argument('--check-survey', metavar='FILE', help='check a surveyed_junctions.json and print its predicted reach')
  ap.add_argument('--list-overrides', action='store_true', help='print the overrides with their evidence')
  ap.add_argument('--clear', metavar='APPROACH', help="remove one override (<OSM node>:<way>:<f|b>) after looking at its evidence")
  ap.add_argument('--visibility', help='a visibility.json (default: the shipped one)')
  ap.add_argument('--mode', default='auto', choices=certify.MODES, help='TurnPilotStopSignGo to certify for (default auto)')
  ap.add_argument('--no-side-views', action='store_true', help='certify as with the side views off')
  ap.add_argument('--eta', help="the route's start, ISO time with its zone (default now)")
  args = ap.parse_args(argv)
  visibility = visibility_table.load(Path(args.visibility)) if args.visibility else None
  if args.check_survey:
    return check_survey(Path(args.check_survey), visibility or visibility_table.load(visibility_table.SHIPPED))
  if args.clear:
    if args.params:
      from openpilot.common.params import Params
      done, where = clear_override_param(Params(), args.clear), 'the TurnPilotJunctionOverrides param'
    elif args.overrides:
      done, where = clear_override_file(Path(args.overrides), args.clear), args.overrides
    else:
      ap.error('--clear needs --overrides FILE or --params')
    if not done:
      print(f"{args.clear} is not in {where}: nothing cleared", file=sys.stderr)
      return 1
    print(f"cleared {' and '.join(done)} from {where}" + (" (written together for one event)" if len(done) > 1 else ''))
    return 0
  if args.list_overrides:
    if args.params:
      from openpilot.common.params import Params
      overrides, ok, why = junctions.read_overrides(Params())
    else:
      overrides, ok, why = junctions.read_overrides(type('File', (), {'get': staticmethod(lambda key: Path(args.overrides).read_text())})())
    if not ok:
      print(f"overrides: {why}", file=sys.stderr)
      return 1
    print('\n'.join(override_lines(overrides)))
    return 0
  if args.origin is None or args.destination is None or not args.tiles:
    ap.error('a route needs ORIGIN, DESTINATION and --tiles')
  loaded = surveys.load(Path(args.surveys) if args.surveys else surveys.SHIPPED)
  for e in loaded.errors:
    print(f"surveys: {e}", file=sys.stderr)
  if args.params:
    from openpilot.common.params import Params
    overrides, ok, why = junctions.read_overrides(Params())
  else:
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
  eta = datetime.fromisoformat(args.eta).timestamp() if args.eta else None
  call = source._call()
  report = audit(call, steps, tiles_dir, sidecar_dir, context, visibility,
                 certify.Runtime(mode=args.mode, side_views=not args.no_side_views), eta)
  if args.survey_template:
    print(json.dumps(survey_template(report), indent=1))
    return 0

  def lookahead(position, heading):
    return controls_ahead(call, position, heading, tiles_dir, sidecar_dir, context=context)

  def certify_ahead(c):
    j = sr.junction_from_control(c)
    return certify.certify(j, 'unknown', report.visibility, report.entries, report.eta, report.runtime)
  report.continuations = continuations(report.controls, lookahead, certify_ahead)
  result = audit_avoidance(report, steps, lambda exclude: source.route((olon, olat), (dlon, dlat), None, None, exclude=exclude),
                           lambda s: route_controls(call, s, tiles_dir, sidecar_dir, context=context), headings=source.exclusion_headings)
  if args.json:
    doc = as_json(report)
    doc['avoidance'] = {'status': result.status, 'avoided': result.avoided, 'iterations': result.iterations, 'added_s': round(result.added_s, 1)}
    print(json.dumps(doc, indent=1))
  else:
    print('\n'.join(lines(report) + avoidance_lines(result)))
  return 0


if __name__ == '__main__':
  sys.exit(main())
