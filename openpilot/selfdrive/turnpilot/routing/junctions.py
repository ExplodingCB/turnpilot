"""The junction at each stop or yield control, on the route and off it (navInstruction MapControl.junctionInfo; STOP_SIGNS.md
1.2, 3.1 and 4.1-4.3, SG1b). routing/controls.py runs it; nothing here calls Valhalla.

The legs. trace_attributes gives, at the control's node (the end of its edge) and at the end node of each consecutive
internal-intersection edge after it (a divided cross road's junction spans them), every other road there
(node.intersecting_edge: begin_heading, road_class, use, driveability, cyclability). The route's own next non-internal edge,
which intersecting_edges never lists, is added as the exit leg: for a left it is the left approach, whose near lanes the
left crosses first, and for a straight the opposing one. Off the route (controls_ahead) the exit leg is the edge the road
carries on onto, or, at the end of the road looked up (where intersecting_edges does list it), the leg nearest straight on
within EXIT_AHEAD_TOL. Roles by the leg's bearing from our heading in (relBearing, + right): opposite within ROLE_OPPOSITE_DEG
of straight on, right or left out to ROLE_SIDE_DEG, other behind; exit for the exit leg whatever its bearing.

Q2 (probes(): in the route's one batched verbose locate, beside the opposing roads' probes): a point at each node (radius
NODE_RADIUS_M) and one LEG_PROBE_M out along each leg's begin_heading (radius LEG_RADIUS_M). A leg's inbound edge is the
candidate whose direction-corrected shape ends within LEG_END_M of its node with its last bearing within LEG_HEADING_TOL of
the leg's heading reversed; its outbound edge the one starting there within LEG_HEADING_TOL of the heading. A candidate
from the node's own point counts only for the leg nearest it in bearing (research_map.md 4.1: 97 of 98 legs matched). The
inbound edge gives Valhalla's control (signal over stop over yield), lanes in, the tagged limit, the way and the bike lane;
the outbound edge lanes out. A leg whose inbound edge isn't matched has control unknown, never none, and so does one cars
only leave by (none to match: read the control with inbound).

Per approach: the assumed limit is the survey's posted sign, else the tagged maxspeed (Valhalla's speed_limit, else the
sidecar's), else CLASS_LIMIT_MPH by road class (decision D8), never Valhalla's classified speed. The tier (4.1), against the
OSM stop-sign sidecar's junction record: an override (TurnPilotJunctionOverrides) is no; a counting survey's approach is
surveyed if it stops, else no; a sidecar stop on the approach is mapStrong when directed or stop=all (its own or the
junction node's), mapWeak when a plain junction-node or undirected stop, or the junction's tags contradict (stop=all beside
stop=minor or an unstopped approach), or stop=minor is anywhere; a Valhalla flag alone is mapWeak; an approach the sidecar
shows unstopped, or that it can't place, is no wherever anything at the junction stops or yields (ours does, unless only
suspected), so a plain two-way's cross approaches are never unknown; unknown otherwise, and everywhere when the sidecar is
missing or stale. The plausibility gate (4.2): a junction where an approach (ours included) is primary or above and
another two or more class ranks lower is implausible as an all-way, and a survey of it counts only with both looks
(inPerson and streetImagery) in confirmedBy.
"""
import json
import math
from dataclasses import dataclass, field
from datetime import date

from openpilot.selfdrive.turnpilot.maps import surveys as survey_file

LEG_PROBE_M = 8.        # m out along a leg its probe point goes
LEG_RADIUS_M = 5        # m: the probe's locate radius
NODE_RADIUS_M = 1       # m: the node's own point
LEG_END_M = 1.5         # m: an edge ending (starting) this close to the node ends (starts) there
LEG_HEADING_TOL = 35.   # deg: an edge's last (first) bearing this close to the leg's heading reversed (the heading) is the leg's
LEG_BACK_M = 2.         # m: an edge's end bearing is taken from a shape point at least this far along
EXIT_AHEAD_TOL = 60.    # deg: off the route, at the road's end, the leg nearest straight on within this is the exit
ROLE_OPPOSITE_DEG = 45.  # deg either side of straight on: opposite
ROLE_SIDE_DEG = 135.     # deg: right (+) or left (-) out to this, other past it
MANEUVER_AFTER_M = 30.   # m after the junction node a route step's maneuver is the control's (N8)
RAIL_M = 30.             # m from the node a mapped rail crossing is reported within (C7), besides one on our own edge
RAIL_ON_ROUTE_M = 5.     # m: a crossing this close to the route is on it
RAIL_NONE = -1.          # railDistance: none within RAIL_M nor on our edge
RAIL_UNKNOWN = -2.       # railDistance: no current sidecar to say
SHOULDER_MAPPED_M = 1.5  # m a mapped shoulder is taken as (no width is mapped; 1.5 m is wide enough to pass in, 2.3)
MPH = 0.44704
CLASS_LIMIT_MPH = {'motorway': 55, 'trunk': 55, 'primary': 55, 'secondary': 45, 'tertiary': 35, 'unclassified': 30,
                   'residential': 30, 'service_other': 30}
UNKNOWN_CLASS_MPH = 55   # a class not known: the highest default
UNLIMITED_KPH = 255      # Valhalla's speed_limit for no limit
RANK = {'motorway': 0, 'trunk': 1, 'primary': 2, 'secondary': 3, 'tertiary': 4, 'unclassified': 5, 'residential': 6, 'service_other': 7}
PLAUSIBLE_TOP_RANK = 2   # primary: an approach this class or above beside one two ranks lower is an implausible all-way
ROAD_CLASS = {'motorway': 'motorway', 'trunk': 'trunk', 'primary': 'primary', 'secondary': 'secondary', 'tertiary': 'tertiary',
              'unclassified': 'unclassified', 'residential': 'residential', 'service_other': 'serviceOther'}
ROAD_USE = {'road': 'road', 'ramp': 'ramp', 'turn_channel': 'turnChannel', 'living_street': 'livingStreet', 'service_road': 'serviceRoad',
            'driveway': 'driveway', 'alley': 'alley', 'parking_aisle': 'parkingAisle', 'track': 'track', 'cycleway': 'cycleway',
            'mountain_bike': 'cycleway', 'footway': 'footway', 'sidewalk': 'footway', 'pedestrian': 'footway', 'steps': 'footway',
            'pedestrian_crossing': 'footway', 'elevator': 'footway', 'escalator': 'footway', 'path': 'path', 'bridleway': 'path',
            'culdesac': 'road', 'emergency_access': 'serviceRoad', 'drive_through': 'serviceRoad'}
OVERRIDES_PARAM = 'TurnPilotJunctionOverrides'
OVERRIDE_KINDS = ('no', 'signalSeen')
EARTH_R = 6371000.


def haversine(a, b) -> float:
  (la1, lo1), (la2, lo2) = a, b
  x = math.radians(lo2 - lo1) * math.cos(math.radians((la1 + la2) / 2))
  return EARTH_R * math.hypot(x, math.radians(la2 - la1))


def bearing(a, b) -> float:
  x = math.radians(b[1] - a[1]) * math.cos(math.radians((a[0] + b[0]) / 2))
  return math.degrees(math.atan2(x, math.radians(b[0] - a[0]))) % 360.


def angle_diff(a: float, b: float) -> float:
  return abs((a - b + 180.) % 360. - 180.)


def rel(heading: float, heading_in: float) -> float:
  """deg from heading_in to heading, + right, in [-180, 180)."""
  return (heading - heading_in + 540.) % 360. - 180.


def offset(p, heading: float, m: float) -> tuple[float, float]:
  """The point m from p (lat, lon) heading that way."""
  r = math.radians(heading)
  dlat = m * math.cos(r) / (EARTH_R * math.pi / 180.)
  dlon = m * math.sin(r) / (EARTH_R * math.pi / 180. * math.cos(math.radians(p[0])))
  return p[0] + dlat, p[1] + dlon


def decode6(s: str) -> list[tuple[float, float]]:
  out, i, lat, lon = [], 0, 0, 0
  while i < len(s):
    vals = []
    for _ in range(2):
      shift = res = 0
      while True:
        b = ord(s[i]) - 63
        i += 1
        res |= (b & 0x1f) << shift
        shift += 5
        if b < 0x20:
          break
      vals.append(~(res >> 1) if res & 1 else res >> 1)
    lat, lon = lat + vals[0], lon + vals[1]
    out.append((lat / 1e6, lon / 1e6))
  return out


def role_of(rel_bearing: float) -> str:
  a = abs(rel_bearing)
  if a <= ROLE_OPPOSITE_DEG:
    return 'opposite'
  if a <= ROLE_SIDE_DEG:
    return 'right' if rel_bearing > 0 else 'left'
  return 'other'


def class_limit(road_class: str | None, use: str | None = None) -> float:
  """m/s: decision D8's default for an untagged road of road_class."""
  if use == 'living_street':
    return 30 * MPH
  return CLASS_LIMIT_MPH.get(road_class or '', UNKNOWN_CLASS_MPH) * MPH


def kph(limit) -> float:
  """m/s from Valhalla's speed_limit (km/h), 0 when untagged or unlimited."""
  v = float(limit or 0.)
  return v / 3.6 if 0. < v < UNLIMITED_KPH else 0.


@dataclass
class Leg:
  """A road at the junction, by the edge leaving its node (trace_attributes)."""
  node: int                  # which of the junction's nodes (0 the control's)
  heading: float | None      # deg, begin_heading; None not given (no probe, unknown)
  road_class: str | None = None
  use: str | None = None
  driveability: str | None = None  # forward (cars only leave), backward (only arrive), both; None not drivable
  cyclability: str | None = None
  exit: bool = False
  way_id: int = 0            # the exit leg's own way (the route's edge)
  probe: int = -1            # its point among probes()'


@dataclass
class Approach:
  """MapJunction's JunctionApproach (log.capnp), and what navd keeps besides."""
  bearing: float
  rel_bearing: float
  role: str
  road_class: str = 'unknown'
  use: str = 'unknown'
  inbound: bool = False
  outbound: bool = False
  lanes_in: int = 0
  lanes_out: int = 0
  valhalla_control: str = 'unknown'
  osm_stop: str = 'unknown'
  tier: str = 'unknown'
  tagged_limit: float = 0.
  assumed_limit: float = 0.
  speed_source: str = 'classDefault'
  way_id: int = 0
  bike_lane: bool = False
  shoulder_m: float = -1.
  # navd only
  node: int = 0
  valhalla_class: str = ''     # Valhalla's road_class ('' not known)
  valhalla_use: str = ''
  forward: bool | None = None  # the inbound edge's direction along its way ('f'/'b' in the sidecar's keys); None unmatched
  matched: bool = False        # its inbound edge was matched
  lanes_known: bool = False    # every direction it carries was matched, with its lanes
  rank: int = 8                # its class's rank (RANK), 8 unknown
  osm_evidence: str = ''       # the sidecar's evidence for its stop

  def to_msg(self) -> dict:
    return {'bearing': float(self.bearing), 'relBearing': float(self.rel_bearing), 'role': self.role, 'roadClass': self.road_class,
            'use': self.use, 'inbound': self.inbound, 'outbound': self.outbound, 'lanesIn': min(self.lanes_in, 255),
            'lanesOut': min(self.lanes_out, 255), 'valhallaControl': self.valhalla_control, 'osmStop': self.osm_stop, 'tier': self.tier,
            'taggedLimit': float(self.tagged_limit), 'assumedLimit': float(self.assumed_limit), 'speedSource': self.speed_source,
            'wayId': int(self.way_id), 'bikeLane': self.bike_lane, 'shoulderM': float(self.shoulder_m)}


@dataclass
class Junction:
  """MapJunction (log.capnp), and what navd and route_audit keep besides."""
  status: str = 'ok'
  osm_node_id: int = 0
  lat: float = 0.
  lon: float = 0.
  stop_point_distance: float = 0.
  stop_point_source: str = 'default'
  heading_in: float = 0.
  maneuver: str = 'straight'
  turn_angle: float = 0.
  roundabout: bool = False
  divided: bool = False
  rail_distance: float = RAIL_UNKNOWN
  sidecar: bool = False
  suspected: bool = False
  surveyed: bool = False
  validated: bool = False      # SG2: the validation for the ETA's lighting
  certification: str = 'unchecked'
  cert_reason: str = 'none'
  approaches: list[Approach] = field(default_factory=list)
  # navd and route_audit only
  plausible: bool = True
  survey: dict | None = None   # the matched entry, counting or not
  survey_note: str = ''        # why a matched entry doesn't count
  our_rank: int = 8
  our_way: int = 0
  our_forward: bool = True
  nodes: list = field(default_factory=list)

  def to_msg(self) -> dict:
    return {'status': self.status, 'osmNodeId': int(self.osm_node_id), 'lat': float(self.lat), 'lon': float(self.lon),
            'stopPointDistance': float(self.stop_point_distance), 'stopPointSource': self.stop_point_source,
            'headingIn': float(self.heading_in), 'maneuver': self.maneuver, 'turnAngle': float(self.turn_angle),
            'roundabout': self.roundabout, 'divided': self.divided, 'railDistance': float(self.rail_distance), 'sidecar': self.sidecar,
            'suspected': self.suspected, 'surveyed': self.surveyed, 'validated': self.validated, 'certification': self.certification,
            'certReason': self.cert_reason, 'approaches': [a.to_msg() for a in self.approaches]}


@dataclass
class Context:
  """What junctions are read against besides the map: the surveys (maps/surveys.py), TurnPilotJunctionOverrides
  (read_overrides) and today's date (None: the system's)."""
  surveys: list[dict] = field(default_factory=list)
  overrides: dict[str, dict] = field(default_factory=dict)  # approach_key -> {kind, evidence}
  overrides_ok: bool = True
  overrides_error: str = ''
  today: date | None = None

  def day(self) -> date:
    return self.today or date.today()


def approach_key(osm_node_id: int, way_id: int, forward: bool) -> str:
  """An approach's key in TurnPilotJunctionOverrides: '<OSM junction node>:<way>:<f|b>' (f: arriving along the way's order,
  as the sidecar's keys)."""
  return f"{int(osm_node_id)}:{int(way_id)}:{'f' if forward else 'b'}"


def read_overrides(params) -> tuple[dict[str, dict], bool, str]:
  """(overrides by approach key, readable, why not) from TurnPilotJunctionOverrides: a JSON list of {approach, kind (no or
  signalSeen), evidence}. A param the build doesn't know, or one that doesn't parse (any item: one bad item may be the
  downgrade that matters), is not readable, which navd says out loud (routing/controls.py); unset is empty."""
  from openpilot.common.params import UnknownKeyName
  try:
    raw = params.get(OVERRIDES_PARAM)
  except UnknownKeyName:
    return {}, False, f"{OVERRIDES_PARAM} is unknown to this build's params (run scons)"
  if raw is None:
    return {}, True, ''
  try:
    if isinstance(raw, bytes):
      raw = raw.decode()
    if isinstance(raw, str):
      raw = json.loads(raw) if raw.strip() else []
    if not isinstance(raw, list):
      raise ValueError("not a list")
    out = {}
    for o in raw:
      if not isinstance(o, dict) or not isinstance(o.get('approach'), str) or len(o['approach'].split(':')) != 3 or \
         o.get('kind') not in OVERRIDE_KINDS:
        raise ValueError(f"bad item {o!r}")
      out[o['approach']] = {'kind': o['kind'], 'evidence': o.get('evidence')}
    return out, True, ''
  except (ValueError, UnicodeDecodeError) as e:
    return {}, False, f"{OVERRIDES_PARAM} doesn't parse: {e}"


def junction_legs(edges, tpts, a: int, ahead: bool) -> tuple[list[tuple[float, float]], list[Leg], int]:
  """(the junction's nodes [(lat, lon)], its legs, the index of its last edge) for the control at the end of trace edge a
  (module docstring)."""
  nodes, legs, j = [], [], a
  while True:
    for x in (edges[j].get('end_node') or {}).get('intersecting_edges') or []:
      h = x.get('begin_heading')
      legs.append(Leg(len(nodes), None if h is None else float(h) % 360., x.get('road_class'), x.get('use'), x.get('driveability'),
                      x.get('cyclability')))
    nodes.append(tpts[edges[j]['end_shape_index']])
    if j + 1 < len(edges) and edges[j + 1].get('internal_intersection'):
      j += 1
    else:
      break
  out = edges[j + 1] if j + 1 < len(edges) else None
  if out is not None:
    h = out.get('begin_heading')
    legs.append(Leg(len(nodes) - 1, None if h is None else float(h) % 360., out.get('road_class'), out.get('use'), None, None,
                    exit=True, way_id=int(out.get('way_id') or 0)))
  elif ahead:
    h_in = heading_in(edges[j], tpts)
    last = [x for x in legs if x.node == len(nodes) - 1 and x.heading is not None and x.driveability in ('forward', 'both')]
    if last:
      best = min(last, key=lambda x: angle_diff(x.heading, h_in))
      if angle_diff(best.heading, h_in) <= EXIT_AHEAD_TOL:
        best.exit = True
  return nodes, legs, j


def heading_in(edge, tpts) -> float:
  """deg: the route's heading into an edge's end node, its end_heading, else from its shape's last LEG_BACK_M or more."""
  h = edge.get('end_heading')
  if h is not None:
    return float(h) % 360.
  end = tpts[edge['end_shape_index']]
  k = edge['end_shape_index'] - 1
  while k > edge['begin_shape_index'] and haversine(tpts[k], end) < LEG_BACK_M:
    k -= 1
  return bearing(tpts[max(k, 0)], end) if k >= 0 else 0.


def probes(nodes, legs) -> list[dict]:
  """Q2's locations for one junction: its nodes, then a point LEG_PROBE_M out along each leg with a heading (each leg's
  probe set to its index here)."""
  locs = [{'lat': n[0], 'lon': n[1], 'radius': NODE_RADIUS_M} for n in nodes]
  for leg in legs:
    if leg.heading is None:
      continue
    leg.probe = len(locs)
    lat, lon = offset(nodes[leg.node], leg.heading, LEG_PROBE_M)
    locs.append({'lat': lat, 'lon': lon, 'radius': LEG_RADIUS_M})
  return locs


def shape_of(c) -> list[tuple[float, float]] | None:
  """A locate candidate's shape in its own direction, or None."""
  shape = (c.get('edge_info') or {}).get('shape')
  if not shape:
    return None
  pts = decode6(shape) if isinstance(shape, str) else [(float(q['lat']), float(q['lon'])) for q in shape]
  if not (c.get('edge') or {}).get('forward', True):
    pts = pts[::-1]  # edge_info's shape runs the way's own direction
  return pts if len(pts) >= 2 else None


def end_bearing(pts, at_end: bool) -> float:
  """deg: the shape's bearing arriving at its end (at_end) or leaving its start, over LEG_BACK_M or more of it."""
  if at_end:
    end = pts[-1]
    far = next((p for p in reversed(pts[:-1]) if haversine(p, end) >= LEG_BACK_M), pts[0])
    return bearing(far, end)
  start = pts[0]
  far = next((p for p in pts[1:] if haversine(p, start) >= LEG_BACK_M), pts[-1])
  return bearing(start, far)


def _fits(c, node, heading: float, inbound: bool) -> float | None:
  """deg off the leg if candidate c is its inbound (or outbound) edge at node, else None."""
  pts = shape_of(c)
  if pts is None:
    return None
  if inbound:
    if haversine(pts[-1], node) > LEG_END_M:
      return None
    d = angle_diff(end_bearing(pts, True), (heading + 180.) % 360.)
  else:
    if haversine(pts[0], node) > LEG_END_M:
      return None
    d = angle_diff(end_bearing(pts, False), heading)
  return d if d <= LEG_HEADING_TOL else None


def match_leg(leg: Leg, nodes, legs, located) -> tuple[dict | None, dict | None]:
  """(inbound, outbound) locate candidates for leg, from its probe's answer, else from its node point's (only for the
  leg nearest a candidate in bearing)."""
  node = nodes[leg.node]
  out = []
  for inbound in (True, False):
    probe = (located[leg.probe] or {}).get('edges') or [] if 0 <= leg.probe < len(located) else []
    fits = [(d, k) for k, c in enumerate(probe) if (d := _fits(c, node, leg.heading, inbound)) is not None]
    if fits:
      out.append(probe[min(fits)[1]])
      continue
    at_node = (located[leg.node] or {}).get('edges') or [] if leg.node < len(located) else []
    near = [x for x in legs if x.node == leg.node and x.heading is not None]
    best = None
    for c in at_node:
      d = _fits(c, node, leg.heading, inbound)
      if d is None:
        continue
      pts = shape_of(c)
      b = end_bearing(pts, inbound)
      nearest = min(near, key=lambda x: angle_diff(b, (x.heading + 180.) % 360. if inbound else x.heading))
      if nearest is leg and (best is None or d < best[0]):
        best = (d, c)
    out.append(best[1] if best else None)
  return out[0], out[1]


def control_of(c) -> str:
  e = c.get('edge') or {}
  return 'signal' if e.get('traffic_signal') else 'stop' if e.get('stop_sign') else 'yield' if e.get('yield_sign') else 'none'


def approach(leg: Leg, h_in: float, inb: dict | None, outb: dict | None) -> Approach:
  """A leg's approach from its matched edges (module docstring), before its tier and limits."""
  r = rel(leg.heading, h_in) if leg.heading is not None else 180.
  a = Approach(float(leg.heading if leg.heading is not None else 0.), r, 'exit' if leg.exit else role_of(r), node=leg.node)
  cls = leg.road_class or ((inb or outb or {}).get('edge') or {}).get('classification', {}).get('classification')
  use = leg.use or ((inb or outb or {}).get('edge') or {}).get('classification', {}).get('use')
  a.road_class, a.rank, a.valhalla_class = ROAD_CLASS.get(cls or '', 'unknown'), RANK.get(cls or '', 8), cls or ''
  a.use, a.valhalla_use = (ROAD_USE.get(use, 'other') if use else 'unknown'), use or ''
  if leg.exit and leg.driveability is None:
    a.inbound = a.outbound = True  # the route's own exit has no driveability: cars may come in by it (unknown until matched)
  else:
    a.inbound, a.outbound = leg.driveability in ('backward', 'both'), leg.driveability in ('forward', 'both')
  a.matched = inb is not None
  if inb is not None:
    e = inb.get('edge') or {}
    a.valhalla_control = control_of(inb)
    a.lanes_in = int(e.get('lane_count') or 0)
    a.forward = bool(e.get('forward', True))
    a.shoulder_m = SHOULDER_MAPPED_M if inb.get('shoulder') else 0.
  if outb is not None:
    a.lanes_out = int((outb.get('edge') or {}).get('lane_count') or 0)
  for c in (inb, outb):
    if c is not None and not a.way_id:
      a.way_id = int((c.get('edge_info') or {}).get('way_id') or 0)
    if c is not None and not a.tagged_limit:
      a.tagged_limit = kph((c.get('edge_info') or {}).get('speed_limit'))
  if not a.way_id and leg.exit:
    a.way_id = leg.way_id
  a.bike_lane = any(((c or {}).get('edge') or {}).get('cycle_lane') in ('dedicated', 'separated') for c in (inb, outb)) or \
    (leg.cyclability == 'both' and leg.driveability in ('forward', 'backward'))
  a.lanes_known = (a.inbound or a.outbound) and (not a.inbound or (inb is not None and a.lanes_in > 0)) and \
    (not a.outbound or (outb is not None and a.lanes_out > 0))
  return a


def osm_node_at(osm, node_id: int, pos) -> int:
  """The OSM junction node: the control's from the sidecar, else the sidecar's junction on pos (within SAME_NODE_M)."""
  if node_id or osm is None:
    return int(node_id or 0)
  best = None
  for n, rec in osm.junctions.items():
    p = rec.get('pos')
    if not p or abs(p[0] - pos[0]) > 1e-4 or abs(p[1] - pos[1]) > 1e-4:
      continue
    d = haversine((float(p[0]), float(p[1])), pos)
    if d <= survey_file.SAME_NODE_M and (best is None or d < best[0]):
      best = (d, n)
  return int(best[1]) if best else 0


def osm_record(osm, node_id: int) -> list[dict] | None:
  """The sidecar's approaches at OSM node node_id, each with stop_tag (all, minor or none) from its control, or None when
  the sidecar has no record of it (no stop or yield mapped there)."""
  rec = osm.junctions.get(int(node_id)) if node_id else None
  if rec is None:
    return None
  out = []
  for a in rec.get('approaches') or []:
    tag = 'none'
    if a.get('stops'):
      entries = [x for x in osm.entries(f"{a.get('way_id')}:{a.get('dir')}") if int(x.get('junction_node') or 0) == int(node_id)]
      tag = next((x.get('stop_tag') for x in entries if x.get('stop_tag') in ('all', 'minor')), 'none')
      tag = {'node_all': 'all', 'node_minor': 'minor'}.get(a.get('evidence'), tag)
    out.append(dict(a, stop_tag=tag))
  return out


def survey_approach(entry: dict | None, way_id: int, heading_toward: float) -> dict | None:
  """The survey's approach on way_id (0: any) heading toward the junction within LEG_HEADING_TOL of heading_toward, the
  nearest."""
  if entry is None:
    return None
  fits = [(angle_diff(float(a['bearingDeg']), heading_toward), k) for k, a in enumerate(entry['approaches'])
          if (not way_id or not a['wayId'] or a['wayId'] == way_id) and angle_diff(float(a['bearingDeg']), heading_toward) <= LEG_HEADING_TOL]
  return entry['approaches'][min(fits)[1]] if fits else None


def classify(j: Junction, osm, ours_stops: bool, ctx: Context) -> None:
  """Each approach's osmStop, tier, limits; the junction's survey, surveyed and plausible (module docstring). osm: the
  sidecar (None or not ok: none current); ours_stops: our approach's control is a mapped stop or yield (not suspected)."""
  osm_ok = osm is not None and osm.status == 'ok'
  record = osm_record(osm, j.osm_node_id) if osm_ok else None
  ranks = [a.rank for a in j.approaches if a.inbound and a.rank < 8] + ([j.our_rank] if j.our_rank < 8 else [])
  top = min(ranks) if ranks else 8
  j.plausible = not (top <= PLAUSIBLE_TOP_RANK and any(r >= top + 2 for r in ranks))
  entry = survey_file.match(ctx.surveys, j.osm_node_id, (j.lat, j.lon))
  j.survey, j.surveyed, j.survey_note = entry, False, ''
  if entry is not None:
    ok, why = survey_file.counts(entry, ctx.day())
    if ok and not j.plausible and not {'inPerson', 'streetImagery'} <= set(entry.get('confirmedBy') or []):
      ok, why = False, "an implausible all-way (a primary or above beside a road two ranks lower): needs inPerson and streetImagery in confirmedBy"
    j.surveyed, j.survey_note = ok, why
  counting = entry if j.surveyed else None
  # the sidecar's approaches matched to ours, by way and direction, else by bearing
  used = set()
  sides = []
  for a in j.approaches:
    sa = None
    if record is not None:
      if a.way_id and a.forward is not None:
        sa = next((x for x in record if x.get('way_id') == a.way_id and x.get('dir') == ('f' if a.forward else 'b')), None)
      if sa is None and a.inbound:
        fits = [(angle_diff(float(x.get('bearing_deg', 0.)), a.bearing), k) for k, x in enumerate(record)
                if k not in used and angle_diff(float(x.get('bearing_deg', 0.)), a.bearing) <= LEG_HEADING_TOL]
        sa = record[min(fits)[1]] if fits else None
      if sa is not None:
        used.add(next(k for k, x in enumerate(record) if x is sa))
    sides.append(sa)
  stopping = [x for x in record or [] if x.get('stops') and x.get('kind') == 'stop']
  tags = {x['stop_tag'] for x in stopping}
  minor_any = 'minor' in tags
  contradict = 'all' in tags and (minor_any or any(not x.get('stops') for x in record or []))
  stops_any = ours_stops or bool(stopping) or any(a.valhalla_control == 'stop' for a in j.approaches)
  for a, sa in zip(j.approaches, sides, strict=True):
    # the OSM stop tag
    if not osm_ok:
      a.osm_stop = 'unknown'
    elif record is None:
      a.osm_stop = 'none'
    elif sa is None:
      a.osm_stop = 'unknown'
    elif sa.get('stops') and sa.get('kind') == 'stop':
      a.osm_stop = {'all': 'all', 'minor': 'minor'}.get(sa['stop_tag'], 'stop')
      a.osm_evidence = sa.get('evidence', '')
    else:
      a.osm_stop = 'none'
    # the speed: the survey's sign, else the tagged limit, else the class default
    if not a.tagged_limit and sa is not None and sa.get('maxspeed'):
      a.tagged_limit = float(sa['maxspeed'])
    sv = survey_approach(counting, a.way_id, (a.bearing + 180.) % 360.)
    if sv is not None and sv.get('postedMph') is not None:
      a.assumed_limit, a.speed_source = float(sv['postedMph']) * MPH, 'survey'
    elif a.tagged_limit > 0.:
      a.assumed_limit, a.speed_source = a.tagged_limit, 'tagged'
    else:
      a.assumed_limit, a.speed_source = class_limit(a.valhalla_class, a.valhalla_use), 'classDefault'
    # the tier
    key = approach_key(j.osm_node_id, a.way_id, a.forward) if j.osm_node_id and a.way_id and a.forward is not None else None
    if key is not None and key in ctx.overrides:
      a.tier = 'no'
    elif sv is not None:
      a.tier = 'surveyed' if sv['stops'] else 'no'
    elif not osm_ok:
      a.tier = 'mapWeak' if a.valhalla_control == 'stop' else 'unknown'
    elif a.osm_stop in ('stop', 'all', 'minor'):
      if contradict or minor_any:
        a.tier = 'mapWeak'
      elif a.osm_stop == 'all' or a.osm_evidence in ('mid_directed', 'mid_both'):
        a.tier = 'mapStrong'
      else:
        a.tier = 'mapWeak'
    elif a.valhalla_control == 'stop':
      a.tier = 'mapWeak'
    else:
      a.tier = 'no' if stops_any or minor_any else 'unknown'


def build(edges, tpts, a: int, nodes, legs, last: int, located: list | None) -> Junction:
  """The junction of the control at the end of trace edge a (its nodes and legs from junction_legs, its last edge last),
  from Q2's answers for its probes (None: Q2 failed, every leg unknown and the junction failed), before classify()."""
  h_in = heading_in(edges[a], tpts)
  j = Junction(lat=float(nodes[0][0]), lon=float(nodes[0][1]), heading_in=h_in, divided=len(nodes) > 1, nodes=list(nodes))
  j.our_way, j.our_forward = int(edges[a].get('way_id') or 0), bool(edges[a].get('forward', True))
  j.our_rank = RANK.get(edges[a].get('road_class') or '', 8)
  out = edges[last + 1] if last + 1 < len(edges) else None
  j.roundabout = bool(out is not None and out.get('roundabout'))
  if located is None:
    j.status = 'failed'
  for leg in legs:
    inb, outb = match_leg(leg, nodes, legs, located) if located is not None and leg.heading is not None else (None, None)
    j.approaches.append(approach(leg, h_in, inb, outb))
  exit_leg = next((x for x in j.approaches if x.role == 'exit'), None)
  j.turn_angle = exit_leg.rel_bearing if exit_leg is not None else 0.
  return j
