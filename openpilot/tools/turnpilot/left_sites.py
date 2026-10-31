#!/usr/bin/env python3
"""left_sites: junctions near a point where an unprotected left across one oncoming approach passes this fork's own route
time for a given TurnPilotLeftRangeM (and TurnPilotLeftFarRangeM), ranked by margin, with why every other one fails
(LEFT_RANGE.md).

  python3 -m openpilot.tools.turnpilot.left_sites --tiles DIR --range M [--far-range M] [--radius M] [--sidecar DIR] LAT,LON

--tiles is what route_audit takes (a map-server region, a tile directory, tiles.tar or valhalla.json), read the same way:
a region with meta.json and no controls_built marker is real OSM carrying its controls. The car is served corridors cut
from the map server's region, so that region is the one to give it.

The junctions are Valhalla's graph, the one navd routes on: a grid of batched locates over the circle gives the drivable
road edges (classes ROAD_CLASSES), and a node where three or more of them meet is a junction. A left there is a road
coming in (an edge ending at the node), one leaving 30-160 deg to its left (an edge starting there) and one within
OPPOSING_DEG of straight across (guidance.py's oncoming approach). Each is routed for real, from APPROACH_M back on its
approach to EXIT_M down its exit, and the route's steps go through what navd and turnpilotd do with them:
guidance.upcoming_maneuvers and route_follower's Maneuver, map_controls.unprotected_turn, the opposing road's mapped limit,
lanes and straight span (routing/controls.py's opposing_roads, through the source as navd asks it) and the route's controls
(route_controls). judge() then takes route time's checks (turnpilotd: reach_at_line at LINE_D plus ROUTE_REACH_MARGIN_M,
or the junction's mapped line's setback; the reach against TurnPilotLeftRangeM, or the far crop's past it; bent past the
opposing road's straight span) and what the left itself would hand back at the line, all said:
  notUnprotected  a keep, a T's stem, or no left at all: not the autonomous left
  sideUncertain   the bearings turn the other way from the modifier: skipped and rerouted in every mode
  twoOncoming     two roads straight across: the band looks up one, so not one oncoming approach (safer than route time)
  controls        the controls unknown (unsupported tiles, a failed match): a signal or stop there can't be ruled out
  signal          the go needs a green from an accepted source (SIGNAL_NEEDS_GREEN), and the camera's reader is shadow
  stop, yield     ours at the junction: the stop-sign decider's, not the left's (TurnPilotStopSignGo off: a handback)
  range           R_req past the range: skipped at route time (unprotectedRange) and rerouted
  curve           the opposing road not mapped straight past R_req (unknown is not straight): skipped the same way
An unmapped limit is 45 mph and unmapped lanes two, as route time takes them (noted). Controls partial (the OSM sidecar
missing: Valhalla's flags only, which miss about a quarter of OSM's stops) pass, said: check the junction for a stop
sign by eye. Nothing here sees the sightline (a crest, a hedge, a parked truck short of R_req), the signals or signs the
map lacks, or the approach speed's stop at the line (turn_stop_feasible): choose the site by eye on the street.

It reads only: the tiles, and no param, socket or log.
"""
import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

from openpilot.selfdrive.turnpilot.map_controls import (HOLD_LINE_M, MAP_MARGIN_M, PROTECT_BEFORE_M, PROTECT_PAST_M, cross_road_yield,
                                                        rail_between, unprotected_turn)
from openpilot.selfdrive.turnpilot.route_follower import maneuver_from_ahead, turn_angle
from openpilot.selfdrive.turnpilot.routing.controls import angle_diff, bearing, cumulative, decode6, haversine, point_at
from openpilot.selfdrive.turnpilot.routing.guidance import OPPOSING_DEG, upcoming_maneuvers
from openpilot.selfdrive.turnpilot.unprotected_left import DEFAULT_OPPOSING_SPEED, LINE_D, ROUTE_REACH_MARGIN_M, UNKNOWN_LANES, reach_at_line

ROAD_CLASSES = ('trunk', 'primary', 'secondary', 'tertiary', 'unclassified', 'residential')
GRID_M = 40.            # m between the locate points
LOCATE_RADIUS_M = 30    # m each looks around it
LOCATE_BATCH = 100      # locations per call
BEARING_M = 10.         # m along a road its bearing at the node is taken over
LEFT_MIN_DEG, LEFT_MAX_DEG = 30., 160.  # a left by the bearings, before the route says
APPROACH_M = 150.       # m back on the approach the route starts (or the approach edge's far end)
EXIT_M = 40.            # m down the exit it ends
NODE_MATCH_M = 10.      # m: the route's maneuver this close to the node is its left
KNOWN = ('ok', 'walkOrSnap', 'partial')
MPH = 0.44704
EARTH_M_PER_DEG = 111195.


@dataclass
class End:
  """A road at a node: the edge, which way it leaves the node, and whether a car can come in or go out along it."""
  way_id: int
  key: str
  bearing: float      # deg leaving the node
  arrive: bool = False
  leave: bool = False
  cls: str = ''
  name: str = ''
  limit_kph: float = 0.
  pts: list = field(default_factory=list)  # (lat, lon) from the node outward


@dataclass
class Node:
  lat: float
  lon: float
  ends: list = field(default_factory=list)


@dataclass
class Verdict:
  ok: bool = False
  codes: list = field(default_factory=list)
  reasons: list = field(default_factory=list)
  notes: list = field(default_factory=list)
  reach: float = 0.
  margin: float = 0.
  limit_m: float = 0.
  speed: float = 0.
  lanes: int = 0
  straight: float = 0.


@dataclass
class Site:
  point: tuple
  from_name: str
  to_name: str
  verdict: Verdict
  approach_kph: float = 0.
  heading: float = 0.


def nodes_from_locate(answers) -> dict:
  """{node key: Node} from verbose locate answers: every drivable road edge's two ends (the shape's, by its direction)."""
  nodes: dict = {}
  seen = set()
  for loc in answers or []:
    for c in (loc or {}).get('edges') or []:
      info, e = c.get('edge_info') or {}, c.get('edge') or {}
      klass = e.get('classification') or {}
      if klass.get('classification') not in ROAD_CLASSES or klass.get('use', 'road') != 'road' or not info.get('shape'):
        continue
      forward = bool(e.get('forward', True))
      if (info['shape'], forward) in seen:
        continue
      seen.add((info['shape'], forward))
      pts = decode6(info['shape'])
      if len(pts) < 2:
        continue
      names = info.get('names') or []
      common = {'way_id': int(info.get('way_id') or 0), 'key': f"{info.get('way_id')}:{info['shape']}", 'cls': klass.get('classification'),
                'name': names[0] if names else f"way {info.get('way_id')}", 'limit_kph': float(info.get('speed_limit') or 0.)}
      start, end = (pts[0], pts[-1]) if forward else (pts[-1], pts[0])
      for here, outward, role in ((start, pts if start == pts[0] else pts[::-1], 'leave'), (end, pts if end == pts[0] else pts[::-1], 'arrive')):
        k = (round(here[0], 6), round(here[1], 6))
        node = nodes.setdefault(k, Node(here[0], here[1]))
        mine = next((x for x in node.ends if x.key == common['key']), None)
        if mine is None:
          cum = cumulative(outward)
          mine = End(bearing=bearing(outward[0], point_at(outward, cum, min(BEARING_M, cum[-1]))), pts=outward, **common)
          node.ends.append(mine)
        setattr(mine, role, True)
  return nodes


def junctions(nodes: dict) -> list[Node]:
  return [n for n in nodes.values() if len(n.ends) >= 3]


def turn_angle_deg(a: End, b: End) -> float:
  """The turn coming in along a and going out along b, deg, positive to the right (route_follower.turn_angle)."""
  return turn_angle((a.bearing + 180.) % 360., b.bearing)


def left_candidates(node: Node) -> list[tuple[End, End]]:
  """(in, out) for every left at the node across a road straight on (module docstring)."""
  out = []
  for a in node.ends:
    if not a.arrive:
      continue
    ahead = (a.bearing + 180.) % 360.
    for b in node.ends:
      if b is a or not b.leave or not LEFT_MIN_DEG <= -turn_angle_deg(a, b) <= LEFT_MAX_DEG:
        continue
      if any(angle_diff(c.bearing, ahead) <= OPPOSING_DEG for c in node.ends if c is not a and c is not b):
        out.append((a, b))
  return out


def maneuver_of(steps, idx: int, opposing):
  """route_follower's Maneuver for the maneuver ending steps[idx], as turnpilotd gets it from navInstruction.allManeuvers."""
  return maneuver_from_ahead(SimpleNamespace(**upcoming_maneuvers(steps, idx, float(steps[idx].get('distance', 0.)), opposing)[0]))


def as_received(c) -> SimpleNamespace:
  """A navd control as turnpilotd's tracker holds it (control_tracker.py): its junction as the message's dict, which
  map_controls' cross_road_yield and rail_between read."""
  info = getattr(c, 'junction_info', None)
  info = info.to_msg() if hasattr(info, 'to_msg') else info
  return SimpleNamespace(kind=c.kind, route_m=float(c.route_m), line_mapped=bool(c.line_mapped),
                         cross_suspect=bool(getattr(c, 'cross_suspect', False)), junction_info=info)


def fail(v: Verdict, code: str, reason: str) -> None:
  v.codes.append(code)
  v.reasons.append(reason)


def judge(steps, opposing, controls, status: str, range_m: float, far_m: float = 0., idx: int = 0,
          right_hand_traffic: bool = True) -> Verdict:
  """Route time's verdict on the left that ends steps[idx] (module docstring), given the route's opposing roads ({step
  index: (m/s, lanes, straight m)}), its controls and their status, and the ranges."""
  v = Verdict()
  if idx + 1 >= len(steps):
    fail(v, 'notUnprotected', "no maneuver after this step")
    return v
  m = maneuver_of(steps, idx, opposing)
  if not unprotected_turn(m, right_hand_traffic):
    what = f"a {m.kind}" if m.kind != 'turn' else "a right" if m.side != 1 else "no oncoming approach (a T's stem)" \
      if m.opposing == 'none' else "not a far-side turn"
    fail(v, 'notUnprotected', f"not an unprotected left: {what} ({m.type} {m.modifier}, opposing {m.opposing})")
    return v
  if m.bearing_side and m.bearing_side != m.side:
    fail(v, 'sideUncertain', f"the bearings turn the other way from '{m.modifier}': skipped and rerouted (sideUncertain)")
    return v
  first = steps[idx + 1]['intersections'][0]
  bearings, i_in, out = first['bearings'], first.get('in'), first['out']
  ahead = (float(bearings[i_in]) + 180.) % 360. if i_in is not None else m.bearings[0]
  if sum(angle_diff(float(bearings[i]), ahead) <= OPPOSING_DEG for i in range(len(bearings)) if i not in (i_in, out)) > 1:
    fail(v, 'twoOncoming', "two roads straight across: not one oncoming approach")
    return v
  junction_m = sum(float(s.get('distance', 0.)) for s in steps[:idx + 1])
  own = [c for c in map(as_received, controls) if junction_m - PROTECT_BEFORE_M <= c.route_m <= junction_m + PROTECT_PAST_M and
         not cross_road_yield(c) and not rail_between(c)]
  if status not in KNOWN:
    fail(v, 'controls', f"controls {status}: a signal or stop sign there can't be ruled out")
  elif status == 'partial':
    v.notes.append("controls partial (Valhalla's flags only, no OSM sidecar): check for a stop sign by eye")
  for kind, why in (('signal', "a traffic signal: the go needs a green from an accepted source, and the camera's reader is shadow"),
                    ('stop', "a stop sign of ours: the stop-sign decider's, a handback with TurnPilotStopSignGo off"),
                    ('yield', "a yield of ours: the stop-sign decider's, not the left's")):
    if any(c.kind == kind for c in own):
      fail(v, kind, why)
  lines = [junction_m - c.route_m for c in own if c.line_mapped]
  margin = max(max(lines) - HOLD_LINE_M - MAP_MARGIN_M, 0.) if lines else ROUTE_REACH_MARGIN_M  # map_controls.line_setback
  v.speed, v.lanes, v.straight = m.opposing_speed, m.opposing_lanes, m.opposing_straight_m
  if v.speed <= 0.:
    v.notes.append(f"opposing limit not mapped: route time takes {DEFAULT_OPPOSING_SPEED / MPH:.0f} mph")
  if v.lanes <= 0:
    v.notes.append(f"opposing lanes not mapped: {UNKNOWN_LANES} lanes taken")
  v.reach = reach_at_line(v.speed or None, v.lanes or None, margin)
  far = far_m > 0. and range_m > 0.
  v.limit_m = max(range_m, far_m) if far else range_m
  if v.reach > v.limit_m:
    what = "the far crop's range" if far else "TurnPilotLeftRangeM"
    fail(v, 'range', f"R_req {v.reach:.0f} m past {what} {v.limit_m:.0f} m: skipped at route time (unprotectedRange)")
  straight_need = v.reach - LINE_D - margin
  if v.reach > LINE_D + margin + v.straight:
    fail(v, 'curve', f"the opposing road is mapped straight {v.straight:.0f} m on, {straight_need:.0f} m needed: skipped at route time")
  v.margin = min(v.limit_m - v.reach, LINE_D + margin + v.straight - v.reach)
  v.ok = not v.codes
  return v


def ranked(sites: list[Site]) -> list[Site]:
  """The passing first, most margin first; then the rest, nearest to passing first."""
  return sorted(sites, key=lambda s: (not s.verdict.ok, -s.verdict.margin))


def site_line(s: Site) -> str:
  v = s.verdict
  head = f"{s.point[0]:.6f},{s.point[1]:.6f}  left from {s.from_name} onto {s.to_name} (coming in heading {s.heading:.0f} deg" + \
    (f", approach limit {s.approach_kph / 1.609:.0f} mph)" if s.approach_kph else ")")
  if v.reach:
    speed = f"{v.speed / MPH:.0f} mph" if v.speed > 0. else "unmapped"
    lanes = f"{v.lanes} lane{'s' if v.lanes != 1 else ''}" if v.lanes > 0 else "lanes unmapped"
    head += f": R_req {v.reach:.0f} m (opposing {speed}, {lanes}, straight {v.straight:.0f} m), margin {v.margin:.0f} m"
  tail = [f"      fails: {r}" for r in v.reasons] + [f"      note: {n}" for n in v.notes]
  return '\n'.join([head] + tail)


def render(sites: list[Site], range_m: float, far_m: float, facts: dict | None = None) -> list[str]:
  facts = facts or {}
  ok = [s for s in sites if s.verdict.ok]
  rest = [s for s in sites if not s.verdict.ok]
  ranges = f"TurnPilotLeftRangeM {range_m:.0f} m" + (f", TurnPilotLeftFarRangeM {far_m:.0f} m (far crop reading)" if far_m > 0. else "")
  out = [f"left_sites: {facts.get('junctions', '?')} junctions within {facts.get('radius', '?')} m, {len(sites)} lefts across an " +
         f"oncoming approach; {ranges}"]
  if facts.get('status'):
    out.append("controls status: " + ', '.join(f"{k} {n}" for k, n in sorted(facts['status'].items())))
  out.append(f"passing route time ({len(ok)}), most margin first:")
  out += [f"  {i + 1}. " + site_line(s) for i, s in enumerate(ok)] or ["  none"]
  out.append(f"failing ({len(rest)}):")
  counts: dict = {}
  for s in rest:
    for c in s.verdict.codes:
      counts[c] = counts.get(c, 0) + 1
  if counts:
    out.append("  by check: " + ', '.join(f"{k} {n}" for k, n in sorted(counts.items(), key=lambda kv: -kv[1])))
  out += ["  " + site_line(s) for s in rest]
  out.append("not checked here: the sightline to R_req (a crest, a hedge, parked cars), signals and signs the map lacks, and the stop " +
             "at the line from the approach speed. Choose the site by eye on the street.")
  return out


def grid(center: tuple, radius_m: float) -> list[tuple[float, float]]:
  lat0, lon0 = center
  k = EARTH_M_PER_DEG * math.cos(math.radians(lat0))
  n = int(radius_m // GRID_M)
  return [(lat0 + j * GRID_M / EARTH_M_PER_DEG, lon0 + i * GRID_M / k) for i in range(-n, n + 1) for j in range(-n, n + 1)
          if math.hypot(i, j) * GRID_M <= radius_m + GRID_M]


def survey(source, call, center: tuple, radius_m: float, range_m: float, far_m: float, tiles_dir, sidecar_dir, context,
           max_lefts: int = 600) -> tuple[list[Site], dict]:
  """Every left across an oncoming approach within radius_m of center, judged (module docstring)."""
  answers = []
  pts = grid(center, radius_m)
  for i in range(0, len(pts), LOCATE_BATCH):
    locs = [{'lat': la, 'lon': lo, 'radius': LOCATE_RADIUS_M} for la, lo in pts[i:i + LOCATE_BATCH]]
    answers += call('locate', {'locations': locs, 'costing': 'auto', 'verbose': True}) or []
  near = [j for j in junctions(nodes_from_locate(answers)) if haversine(center, (j.lat, j.lon)) <= radius_m]
  facts = {'junctions': len(near), 'radius': f"{radius_m:.0f}", 'status': {}}
  sites: list[Site] = []
  for j in sorted(near, key=lambda j: haversine(center, (j.lat, j.lon))):
    for a, b in left_candidates(j):
      if len(sites) >= max_lefts:
        facts['capped'] = max_lefts
        return sites, facts
      sites.append(route_and_judge(source, j, a, b, range_m, far_m, tiles_dir, sidecar_dir, context, facts))
  return sites, facts


def route_and_judge(source, j: Node, a: End, b: End, range_m: float, far_m: float, tiles_dir, sidecar_dir, context, facts) -> Site:
  cum_a, cum_b = cumulative(a.pts), cumulative(b.pts)
  start = point_at(a.pts, cum_a, min(APPROACH_M, cum_a[-1]))
  toward = point_at(a.pts, cum_a, max(min(APPROACH_M, cum_a[-1]) - 5., 0.))
  heading = bearing(start, toward)
  end = point_at(b.pts, cum_b, min(EXIT_M, cum_b[-1] - 1.) if cum_b[-1] > 2. else cum_b[-1])
  site = Site((j.lat, j.lon), a.name, b.name, Verdict(), a.limit_kph, (a.bearing + 180.) % 360.)
  try:
    steps = source.route((start[1], start[0]), (end[1], end[0]), heading, None)
  except Exception as e:
    fail(site.verdict, 'route', f"Valhalla found no route through it ({type(e).__name__})")
    return site
  idx = next((i for i in range(len(steps) - 1) if (loc := (steps[i + 1].get('maneuver') or {}).get('location')) and
              haversine((loc[1], loc[0]), (j.lat, j.lon)) <= NODE_MATCH_M), None)
  if idx is None:
    fail(site.verdict, 'route', "the route from the approach doesn't turn here (a turn restriction, a one-way, or a shorter way)")
    return site
  controls, status = source.controls(steps, tiles_dir, sidecar_dir=sidecar_dir, context=context)
  facts['status'][status] = facts['status'].get(status, 0) + 1
  site.verdict = judge(steps, source.opposing(steps), controls, status, range_m, far_m, idx)
  return site


def point(text: str) -> tuple[float, float]:
  lat, lon = (float(x) for x in text.split(','))
  return lat, lon


def main(argv=None) -> int:
  from openpilot.selfdrive.turnpilot.maps import surveys
  from openpilot.selfdrive.turnpilot.routing import junctions as jn
  from openpilot.selfdrive.turnpilot.routing import osm_sidecar
  from openpilot.selfdrive.turnpilot.routing.controls import MARKER
  from openpilot.selfdrive.turnpilot.routing.valhalla_source import ValhallaSource
  ap = argparse.ArgumentParser(description="unprotected-left sites that pass TurnPilot's route time near a point: read only")
  ap.add_argument('center', type=point, help='lat,lon')
  ap.add_argument('--tiles', required=True, help="what route_audit takes: a map-server region, tiles directory, tiles.tar or valhalla.json")
  ap.add_argument('--sidecar', help="the directory with osm_controls/ and the tiles' stamp (default: the tiles' own)")
  ap.add_argument('--range', type=float, required=True, dest='range_m', help='TurnPilotLeftRangeM, m (left_range.py measures it)')
  ap.add_argument('--far-range', type=float, default=0., dest='far_m', help='TurnPilotLeftFarRangeM, m (0: none)')
  ap.add_argument('--radius', type=float, default=800., help='m around the center (default 800)')
  ap.add_argument('--max-lefts', type=int, default=600, help='lefts routed at most (default 600)')
  args = ap.parse_args(argv)
  source = ValhallaSource(args.tiles)
  tiles_dir = source.tiles_dir
  if tiles_dir is not None and not (Path(tiles_dir) / MARKER).exists() and (Path(tiles_dir) / 'meta.json').is_file():
    print(f"note: {tiles_dir} has no {MARKER} marker: read as a map-server region built from its PBF", file=sys.stderr)
    tiles_dir = None
  sidecar_dir = args.sidecar or source.tiles_dir or osm_sidecar.NO_DIRECTORY
  context = jn.Context(surveys.load(surveys.SHIPPED).entries, {}, True, '')
  sites, facts = survey(source, source._call(), args.center, args.radius, args.range_m, args.far_m, tiles_dir, sidecar_dir, context,
                        args.max_lefts)
  out = render(ranked(sites), args.range_m, args.far_m, facts)
  if facts.get('capped'):
    out.insert(1, f"stopped after {facts['capped']} lefts (--max-lefts): a smaller --radius covers the rest")
  if facts['status'] and set(facts['status']) == {'unsupported'}:
    out.insert(1, "every route's controls are unsupported: these tiles have no controls_built marker and no meta.json, so a " +
                  "signal or stop can't be ruled out anywhere; give the map server's region (its meta.json) instead")
  print('\n'.join(out))
  return 0 if any(s.verdict.ok for s in sites) else 2


if __name__ == '__main__':
  sys.exit(main())
