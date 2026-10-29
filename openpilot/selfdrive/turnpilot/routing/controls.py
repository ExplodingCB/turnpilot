"""The route's stop signs, yields and traffic signals, from Valhalla and the OSM stop-sign sidecar (navInstruction.controls,
map_controls.py).

route_controls() takes navd's route steps and asks Valhalla two things. trace_attributes (edge_walk over the route's own
shape) gives the directed edges the route drives, with each edge's traffic_signal, its end node's traffic_signal and the
roads meeting there. Locate gives what trace_attributes doesn't: a directed edge's stop_sign and yield_sign. One batched
verbose locate, radius 1, a metre back from each edge's end, picks the candidate with the edge's own id. Valhalla puts a
control on the edge that ends at the junction, so every control is placed at the END of its edge:
  signal = the edge's traffic_signal or its end node's; stop = locate's stop_sign; yield = locate's yield_sign.
The OSM sidecar (routing/osm_sidecar.py, from sidecar_dir) adds what Valhalla misses: it sets no stop_sign on about a
quarter of the OSM-mapped stop approaches (research_allway.md section 0). Its entries for the edge's way and direction
("<way_id>:<f|b>", a list, since a way passes several junctions) are the edge's when the entry's line, the OSM stop node,
projects inside the edge's span on the route within LINE_TOL_M of it (past SPAN_START_M from its start, which is the
previous edge's end), or, for a stop on the junction node (no line), when that node is within NODE_TOL_M of the edge's
end. The edge's stops and yields are the union of both, but for an orphan (an entry the sidecar ties to no junction,
junction_node 0: a mid-block trail or rail crossing, or a reversed direction tag) whose sign is more than ORPHAN_NEAR_M
before its anchor edge's end: that end less CONTROL_LINE_M would stop the car where there is no sign, up to a kilometre
past the real one (Indianapolis: 146 of the 176 such orphans more than 15 m past), so it is left out and logged (route_audit
lists it as uncovered), as the base had it; never placed at its sign instead, since a reversed tag would then stop the car
past a junction. It is still ours on the edge for phantom(). One exception: Valhalla flags both directions of a stop on a
graph node it can't direct (a crosswalk's, shared with a footway), so the other direction's directed stop sitting ON the
edge's end node (within PHANTOM_NODE_M) makes a phantom of our flag of its kind there: dropped, and logged, but only when
the sidecar has nothing of ours on the edge (any kind, directed, both-ways, undirected, with a junction or none) and no
undirected or both-ways one keyed the other way inside it (phantom()). A missing entry for us proves nothing by itself
(the sidecar may not have tied the stop to a junction), so every other case keeps Valhalla's flag: an extra stop is
cheap, a dropped real one isn't.
Internal-intersection edges (inside a divided road's junction) are searched too; a control on one is the preceding
junction's (the end of the last edge before them), so a stop mapped in a median stops the car before the first
carriageway. One edge end is one node, and its kinds are one control, the signal over the stop over the yield; controls at
different nodes never merge, however close (STOP_SIGNS.md G11, G18).
junction: other roads meet at the end node. crossSuspect (stop and yield only): the route's road outranks every road
crossing there, so the sign is likely the cross road's.
The stop line: a sim sidecar (tiles_dir/controls.json, {"controls": {"<way_id>:<f|b>": {kind, line: [lat, lon], ...}}} from
tools/sim/bridge/beamng/mapsignals.py) gives the sim's real lines, projected onto the route within the edge (lineMapped,
line source sim). Otherwise the edge's end less CONTROL_LINE_M (line source default), for Valhalla's controls and the
sidecar's alike. Either way less MAP_MARGIN_M. The OSM stop node never sets it: it marks a sign, not a line, 20-40 m back
on some approaches, and a yield moved back to its sign would end map_controls' YIELD_SPEED cap well short of the merge.
It is carried as data only (osm_sign_m: the sign nearest the junction on the control's own edge, for route_audit). A stop or
yield's stop point (never a signal's) then moves back to junctions.stop_offset's rule where the cross road's lanes are known
(node - 13 m for 2 lanes, 16.7 m for 4: SG1b, G15), never forward, or to a counting survey's stopPointM. osm_node is the
OSM junction the sidecar says the control governs (0 when it has none).
Off the route (controls_ahead): the same, over the road ahead of the car instead of a route's shape. Locate (verbose, by
the car's heading) gives the edge the car is on and its shape from the car on; from its end node, the edge that carries on
nearest the bearing it arrives with (within NEXT_HEADING_TOL: the road straight on), and so on to AHEAD_M or AHEAD_EDGES.
That road is walked as a one-step route. A control at its end counts (the route's end is a destination; this road's isn't).
Past a junction the car may turn off it: navd looks again every second, and turnpilotd drops what a lookup no longer lists
(map_controls.py).
Status: ok; walkOrSnap (edge_walk failed, walk_or_snap matched it, logged); partial (more than PARTIAL_UNMATCHED of the
edges unmatched by locate, or the OSM sidecar missing or stale where one is expected: the stops Valhalla misses are
unknown; logged, and each stop and yield's junction_status is failed); failed (nothing: both matches failed, or any
exception); unsupported (tiles_dir given and it has no controls_built marker: tiles built from data with no controls, the
old sim exports). The route publishes whatever the status: navd sends it on every message, and turnpilotd says so out loud.
sidecar_dir None means no sidecar is expected (synthetic tests); navd's sources always pass one (routing/sources.py).
The junction (SG1b): each stop and yield carries its junction (junction_info, routing/junctions.py: the roads there, the
road the route leaves by, their controls, lanes, limits and all-way tiers, the survey, the maneuver, the rail distance),
read from trace_attributes and Q2, one more batched verbose locate per route that also carries the opposing roads' probes
(opposing_roads, for the unprotected left: given an Opposing, a route makes Q1 and Q2 and no third). A failed Q2 leaves
every leg unknown and the junction failed. A suspected unmapped stop (N5: every road crossing outranks ours by two classes
or more, junctions.suspected_stop, and nothing mapped on our whole approach, approach_edges: no signal, stop or yield from
Valhalla or the sidecar back to the previous road junction or JUNCTION_AHEAD_M, and no suspected stop already at a junction
within SAME_JUNCTION_M before it) is emitted as a stop at the default stop point, its junction flagged suspected. Mapped
controls at different nodes still never merge. A TurnPilotJunctionOverrides navd can't read fails every stop and yield junction
and makes the status partial, logged. Off the route a LocateCache keeps Q1 and Q2 by edge id between the 1 Hz lookups.
navd's own Control also carries, for SG2's avoidance loop, its approach edge's key, its exclusion point and whether that
edge is the route's first or last.
"""
import json
import math
import threading
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.routing import junctions as jn
from openpilot.selfdrive.turnpilot.routing import osm_sidecar
from openpilot.selfdrive.turnpilot.routing.guidance import opposing_leg

CONTROL_LINE_M = 10.    # m before the junction node a stop line is taken to be, without a mapped one
MAP_MARGIN_M = 1.       # m short of the line the car stops
ORPHAN_NEAR_M = CONTROL_LINE_M + 5.  # m: a sidecar stop or yield tied to no junction (junction_node 0, an orphan) whose sign
                                     # is further than this before its anchor edge's end is left out (SG1a): that end less
                                     # CONTROL_LINE_M would put it that far past its sign, where there is none
PARTIAL_UNMATCHED = 0.10
LINE_TOL_M = 5.         # m: a sidecar line further than this from the route isn't on it
NODE_TOL_M = 5.         # m: a junction-node control's node this close to an edge's end is that end
PHANTOM_NODE_M = 1.     # m: the other direction's directed stop this close to an edge's end node is on that node (C14)
SPAN_START_M = 0.5      # m: a line this close past an edge's start is on the node it shares with the edge before
EDGE_END_M = 1.         # m: a line this far past an edge's end still projects onto it
LOCATE_BACK_M = 1.      # m back from an edge's end the locate point goes, so it is on the edge, not the junction node
END_OF_ROUTE_M = 1.     # an edge ending this close to the route's end ends at the destination: its control isn't driven to
SIDECAR = 'controls.json'
MARKER = 'controls_built'
KIND_RANK = {'signal': 0, 'stop': 1, 'yield': 2}
ROAD_CLASS_RANK = {'motorway': 0, 'trunk': 1, 'primary': 2, 'secondary': 3, 'tertiary': 4, 'unclassified': 5, 'residential': 6,
                   'service_other': 7}
HTTP_TIMEOUT_S = 5.
AHEAD_M = 250.          # m of road ahead of the car an off-route lookup follows (engage_distance at 25 m/s is 248 m)
AHEAD_EDGES = 8         # edges at most
AHEAD_HEADING_TOL = 45  # deg: the car's edge, by the car's heading
NEXT_HEADING_TOL = 60   # deg: the edge carrying on from a node, by the bearing the road arrives with
AHEAD_RADIUS_M = 25     # m from the car's position its edge is looked for
NODE_GAP_M = 3.         # m: a next edge starting further than this from the last one's end isn't its continuation
EARTH_R = 6371000.

ATTRIBUTES = ['edge.id', 'edge.way_id', 'edge.forward', 'edge.length', 'edge.begin_shape_index', 'edge.end_shape_index',
              'edge.traffic_signal', 'edge.internal_intersection', 'edge.road_class', 'node.traffic_signal', 'node.type',
              'node.intersecting_edge.road_class', 'shape',
              # the junctions (routing/junctions.py)
              'edge.begin_heading', 'edge.end_heading', 'edge.roundabout', 'edge.use', 'edge.lane_count', 'edge.speed_limit',
              'node.intersecting_edge.begin_heading', 'node.intersecting_edge.driveability', 'node.intersecting_edge.use',
              'node.intersecting_edge.cyclability', 'edge.traversability']
EXCLUDE_BACK_M = 12.     # m back from the node an approach's exclusion point goes, at most half its edge (SG2, 3.3)
JUNCTION_AHEAD_M = 60.   # m on along the route the junction the sidecar says a stop governs is looked for (its MAXD), and
                         # back along it a suspected stop's approach is looked over for anything mapped (N5)
SAME_JUNCTION_M = 25.    # m: a road junction (or an internal-intersection edge) this close before a suspected stop's node is
                         # its junction's (a divided road's near carriageway, its median split by a node so that Valhalla
                         # flags neither half internal: Indianapolis routes 60 and 73, 10 m), for suspected stops only
SUSPECTED_STOPS = True   # emit suspected unmapped stops (N5) as stop controls: until SG2 routes around them, each is a stop
CACHE_MAX = 4096         # Q1 and Q2 answers a LocateCache keeps, each
SAME_EDGE_NODE_M = 1.5   # m: an edge's end node this close to another's is the same node (surveys.SAME_NODE_M)


@dataclass
class Control:
  id: int              # index along the route
  kind: str            # 'stop', 'yield' or 'signal'
  route_m: float       # m along the route (navd's step frame: the steps' distances, then along the step) to the stop point
  step_idx: int
  along_step_m: float
  line_mapped: bool = False
  junction: bool = True
  cross_suspect: bool = False
  instance: str = ''   # the sim's signal instance its line came from (the sidecar), for logs and tests
  point: tuple[float, float] | None = None  # (lat, lon) of the stop point: controls_ahead's, which navd measures from
  line_source: str = 'default'  # 'sim' (the sim's controls.json) or 'default' (the edge's end less 10 m)
  osm_node: int = 0    # the OSM junction node the sidecar says it governs; 0 none
  osm_sign_m: float | None = None  # m along the route (navd's frame) of the OSM stop node on its edge, nearest the junction:
                                   # data only (route_audit), never the stop point
  junction_status: str = 'ok'  # 'failed' for a stop or yield when the OSM sidecar was missing or stale (STOP_SIGNS.md N7;
                               # MapJunction.status, SG1b)
  edge: int = -1       # the trace edge whose end it is at (route_audit.py)
  junction_info: jn.Junction | None = None  # a stop or yield's junction (routing/junctions.py; MapControl.junctionInfo)
  # for SG2's avoidance loop (navd's only, not on the bus): the approach edge's key (way id, forward, the node's (lat, lon)
  # to 1e-6 deg), its exclusion point min(EXCLUDE_BACK_M, half the edge) back from the node with the route's bearing there
  # (lat, lon, deg), and whether the edge is the route's first or last
  approach_key: tuple | None = None
  exclusion: tuple[float, float, float] | None = None
  first_edge: bool = False
  last_edge: bool = False
  # navd's, from SG2's certifier and avoidance loop (routing/certify.py, routing/avoid.py): its certification and reason
  # (also on junction_info for the bus), and the Unix-time window its ETA's light check passed for (certifiable only)
  certification: str = 'unchecked'
  cert_reason: str = 'none'
  eta_window: tuple[float, float] | None = None


@dataclass
class Opposing:
  """route_controls' out-parameter: the route's opposing roads ({step index: (m/s, lanes)}, opposing_roads'), read in its
  one batched Q2 locate; None until that locate ran (unsupported tiles, an early failure: opposing_roads alone then)."""
  roads: dict | None = None


class LocateCache:
  """Q1's and Q2's locate answers by edge id, for navd's lookups ahead of the car every second (controls_ahead), which
  mostly repeat: Q1's matched candidate per edge, Q2's answers per junction (its edges' ids). One per ValhallaSource (one
  tile set). Thread-safe (RouteJob and AheadJob run apart); CACHE_MAX of each, the oldest going first."""

  def __init__(self, size: int = CACHE_MAX):
    self.size, self.lock, self.data = size, threading.Lock(), {'q1': OrderedDict(), 'q2': OrderedDict()}

  def get(self, kind: str, key):
    with self.lock:
      return self.data[kind].get(key)

  def put(self, kind: str, key, value) -> None:
    with self.lock:
      d = self.data[kind]
      d.pop(key, None)
      d[key] = value
      while len(d) > self.size:
        d.popitem(last=False)


def decode6(s: str) -> list[tuple[float, float]]:
  """Valhalla's encoded polyline (precision 6) as [(lat, lon), ...]."""
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


def haversine(a, b) -> float:
  (la1, lo1), (la2, lo2) = a, b
  x = math.radians(lo2 - lo1) * math.cos(math.radians((la1 + la2) / 2))
  return EARTH_R * math.hypot(x, math.radians(la2 - la1))


def cumulative(pts) -> list[float]:
  cum = [0.]
  for a, b in zip(pts, pts[1:], strict=False):
    cum.append(cum[-1] + haversine(a, b))
  return cum


def point_back(pts, cum, idx: int, back_m: float):
  """The point back_m along pts before index idx."""
  target = cum[idx] - back_m
  j = idx
  while j > 0 and cum[j - 1] > target:
    j -= 1
  if j == 0:
    return pts[0]
  a, b = pts[j - 1], pts[j]
  seg = cum[j] - cum[j - 1]
  f = 0. if seg <= 0. else (target - cum[j - 1]) / seg
  return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])


def project(pts, cum, p, lo_m: float, hi_m: float) -> float | None:
  """m along pts of the closest spot to p between lo_m and hi_m along, or None."""
  hit = nearest(pts, cum, p, lo_m, hi_m)
  return None if hit is None else hit[0]


def nearest(pts, cum, p, lo_m: float, hi_m: float) -> tuple[float, float] | None:
  """(m along pts, m from p) of the closest spot to p on the segments between lo_m and hi_m along, or None."""
  best = None
  for j in range(len(pts) - 1):
    if cum[j + 1] < lo_m or cum[j] > hi_m:
      continue
    a, b = pts[j], pts[j + 1]
    kx = math.cos(math.radians(a[0]))
    ax, ay, bx, by, px, py = a[1] * kx, a[0], b[1] * kx, b[0], p[1] * kx, p[0]
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0. if L2 <= 0. else min(max(((px - ax) * dx + (py - ay) * dy) / L2, 0.), 1.)
    q = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
    d = haversine(q, p)
    if best is None or d < best[0]:
      best = (d, cum[j] + t * (cum[j + 1] - cum[j]))
  return None if best is None else (best[1], best[0])


def point_at(pts, cum, s: float):
  """The point s m along pts."""
  return point_back(pts, cum, len(pts) - 1, cum[-1] - min(max(s, 0.), cum[-1]))


def bearing(a, b) -> float:
  """deg clockwise from north, from (lat, lon) a to b."""
  x = math.radians(b[1] - a[1]) * math.cos(math.radians((a[0] + b[0]) / 2))
  return math.degrees(math.atan2(x, math.radians(b[0] - a[0]))) % 360.


def angle_diff(a: float, b: float) -> float:
  return abs((a - b + 180.) % 360. - 180.)


def route_shape(steps) -> list[tuple[float, float]]:
  """The steps' geojson geometry as one [(lat, lon), ...], consecutive duplicates dropped."""
  pts = []
  for step in steps:
    for lon, lat in (step.get('geometry') or {}).get('coordinates') or []:
      if not pts or pts[-1] != (lat, lon):
        pts.append((lat, lon))
  return pts


def step_frame(steps, pts, cum):
  """[(start along the shape, start in navd's frame)] per step: navd measures a step by its distance, then along its
  geometry, so a control's route distance is the steps before it plus its distance along its own step's geometry."""
  starts, along, frame, k = [], 0., 0., 0
  for step in steps:
    coords = (step.get('geometry') or {}).get('coordinates') or []
    starts.append((along, frame))
    if coords:
      first = (coords[0][1], coords[0][0])
      while k < len(pts) - 1 and pts[k] != first:
        k += 1
      along = cum[k]
      starts[-1] = (along, frame)
      last = (coords[-1][1], coords[-1][0])
      j = k
      while j < len(pts) - 1 and pts[j] != last:
        j += 1
      along, k = cum[j], j
    frame += float(step.get('distance', 0.))
  return starts


def to_step(starts, s: float) -> tuple[int, float, float]:
  """(step index, m along it, m in navd's route frame) of shape distance s."""
  idx = 0
  for i, (start, _) in enumerate(starts):
    if start <= s + 1e-6:
      idx = i
  start, frame = starts[idx]
  return idx, s - start, frame + s - start


def load_sidecar(tiles_dir) -> dict:
  if tiles_dir is None:
    return {}
  path = Path(tiles_dir) / SIDECAR
  if not path.is_file():
    return {}
  return json.loads(path.read_text()).get('controls', {})


def http_call(url: str, post):
  """call() for a valhalla_service at url; post is requests.post."""
  def call(endpoint, req):
    resp = post(f"{url}/{endpoint}", json=req, timeout=HTTP_TIMEOUT_S)
    resp.raise_for_status()
    return resp.json()
  return call


def actor_call(actor):
  """call() for a pyvalhalla Actor (its methods take and return JSON as str or dict, by version)."""
  def call(endpoint, req):
    out = getattr(actor, endpoint)(req)
    return json.loads(out) if isinstance(out, str) else out
  return call


def trace(call, pts):
  """trace_attributes over the shape: (response, status), edge_walk first, then walk_or_snap."""
  req = {'shape': [{'lat': la, 'lon': lo} for la, lo in pts], 'costing': 'auto', 'shape_match': 'edge_walk',
         'filters': {'attributes': ATTRIBUTES, 'action': 'include'}}
  try:
    t = call('trace_attributes', req)
    if t.get('edges'):
      return t, 'ok'
  except Exception as e:  # seen on 1 of 81 Valhalla-made routes: surfaced in the status, never hidden
    cloudlog.warning(f"turnpilot controls: edge_walk failed ({type(e).__name__}: {e}), trying walk_or_snap")
  req['shape_match'] = 'walk_or_snap'
  t = call('trace_attributes', req)
  if not t.get('edges'):
    raise ValueError("trace_attributes matched no edges")
  cloudlog.warning("turnpilot controls: the route matched only by walk_or_snap")
  return t, 'walkOrSnap'


OPPOSING_HEADING_TOL = 45.  # deg: the opposing road's edge coming into the junction, within this of the leg reversed
OPPOSING_RADIUS_M = 8.
UNLIMITED_KPH = 255         # Valhalla's speed_limit for no limit (kUnlimitedSpeedLimit)


def opposing_probes(steps) -> tuple[dict, list[dict]]:
  """({step index: (lat, lon, bearing)}, their locate locations) for opposing_roads: a point OPPOSING_PROBE_M out along the
  road straight across each maneuver's junction (guidance.opposing_leg), by the heading coming into the junction."""
  legs = {i: leg for i in range(len(steps)) if (leg := opposing_leg(steps, i)) is not None}
  return legs, [{'lat': lat, 'lon': lon, 'heading': round((b + 180.) % 360.) % 360, 'heading_tolerance': OPPOSING_HEADING_TOL,
                 'radius': OPPOSING_RADIUS_M} for lat, lon, b in legs.values()]


def opposing_parse(legs: dict, located) -> dict[int, tuple[float, int]]:
  """opposing_roads' answer from locate's answers for opposing_probes' locations."""
  out = {}
  for (i, (_, _, b)), loc in zip(legs.items(), located, strict=False):
    inbound = (b + 180.) % 360.
    cands = [c for c in (loc or {}).get('edges') or [] if angle_diff(float(c.get('heading', inbound)), inbound) <= OPPOSING_HEADING_TOL]
    if not cands:
      continue
    c = min(cands, key=lambda c: (round(float(c.get('distance', 0.)), 1), angle_diff(float(c.get('heading', inbound)), inbound)))
    limit = float((c.get('edge_info') or {}).get('speed_limit') or 0.)
    out[i] = (limit / 3.6 if 0. < limit < UNLIMITED_KPH else 0., int((c.get('edge') or {}).get('lane_count') or 0))
  return out


def opposing_roads(call, steps) -> dict[int, tuple[float, int]]:
  """{step index: (speed limit m/s, lanes)} for the road straight across the junction of each maneuver that ends a step
  (guidance.opposing_leg), from one batched verbose locate OPPOSING_PROBE_M out along it: the edge coming into the junction,
  its edge_info.speed_limit (tagged maxspeed, km/h; 0 untagged and 255 unlimited are 0, unknown) and edge.lane_count (0
  unknown). A leg with no such edge has no entry; a failure is logged and gives {} (unknown everywhere: the unprotected left
  takes 45 mph, never a guess lower). route_controls reads the same in its own batched locate (Opposing), so a route makes
  one: this is for when it didn't."""

  try:
    legs, locs = opposing_probes(steps)
    if not legs:
      return {}
    return opposing_parse(legs, call('locate', {'locations': locs, 'costing': 'auto', 'verbose': True}) or [])
  except Exception:
    cloudlog.exception("turnpilot opposing roads: failed")
    return {}


def route_controls(call, steps, tiles_dir=None, sidecar_dir=None, audit: list | None = None, opposing: Opposing | None = None,
                   context: jn.Context | None = None) -> tuple[list[Control], str]:
  """(controls along the route nearest first, status). call(endpoint, request) -> response: actor_call or http_call.
  tiles_dir: the local tiles' directory, for the controls_built marker and the sim sidecar; None for tiles built from
  real OSM, which carry their own controls. sidecar_dir: the directory holding the OSM sidecar (osm_controls/) and the
  tiles' stamp (a corridor pack, a region's or the sim tiles' directory), osm_sidecar.NO_DIRECTORY when one is expected
  but the tiles have no directory, None when none is (module docstring). audit: a list every trace edge is appended to,
  as a dict with its span on the route and both sources' stops and yields there (route_audit.py). opposing: given, its
  roads are read in the junctions' batched locate (opposing_roads' answer, one call for both). context: the surveys and
  overrides the junctions are read against (routing/junctions.py), none by default."""
  try:
    if tiles_dir is not None and not (Path(tiles_dir) / MARKER).exists():
      return [], 'unsupported'
    return _route_controls(call, steps, tiles_dir, sidecar_dir, audit=audit, opposing=opposing, context=context)
  except Exception:
    cloudlog.exception("turnpilot controls: failed")
    return [], 'failed'


def _edge_from(call, p, heading: float, tol: float, radius: float, starting: bool):
  """(locate's candidate, its shape [(lat, lon)] in its own direction) for the edge at p heading that way, or None.
  starting: only an edge that starts at p (a node), not one that ends there."""
  resp = call('locate', {'locations': [{'lat': p[0], 'lon': p[1], 'heading': round(heading) % 360, 'heading_tolerance': tol,
                                        'radius': radius}], 'costing': 'auto', 'verbose': True})
  cands = [c for c in ((resp or [None])[0] or {}).get('edges') or [] if (c.get('edge_info') or {}).get('shape')]
  if starting:
    cands = [c for c in cands if c.get('percent_along', 1.) < 0.5]
  if not cands:
    return None
  near = min(c.get('distance', 0.) for c in cands)
  # at a node, the edge leaving it rather than the one ending there; then the nearest the heading
  c = min((c for c in cands if c.get('distance', 0.) < near + 2.),
          key=lambda c: (c.get('percent_along', 0.) > 0.999, angle_diff(c.get('heading', heading), heading)))
  shape = c['edge_info']['shape']
  pts = decode6(shape) if isinstance(shape, str) else [(q['lat'], q['lon']) for q in shape]
  if not (c.get('edge') or {}).get('forward', True):
    pts = pts[::-1]  # edge_info's shape runs the way's own direction
  return c, pts


def own_edge(call, position, heading: float) -> tuple[int, bool, tuple[float, float]] | None:
  """(way id, forward, end node (lat, lon)) of the edge the car at position (lat, lon) heading that way is on (locate, as
  road_ahead's first edge), or None (none found, or the locate failed): SG2's avoidance loop never excludes it."""
  try:
    first = _edge_from(call, position, heading, AHEAD_HEADING_TOL, AHEAD_RADIUS_M, False)
  except Exception:
    cloudlog.exception("turnpilot controls: the car's own edge (locate) failed")
    return None
  if first is None:
    return None
  c, pts = first
  return int((c.get('edge_info') or {}).get('way_id') or 0), bool((c.get('edge') or {}).get('forward', True)), (float(pts[-1][0]), float(pts[-1][1]))


def exclusion_headings(call, points) -> list[float | None]:
  """Valhalla's own heading of each point's edge there, points [(lat, lon, way id, forward)], in one batched verbose locate:
  the candidate on that way and direction, else None. Valhalla correlates an exclude_location against the edge heading it
  computes itself, which on a short curved approach can be far off the route's local bearing (routing/avoid.py)."""
  if not points:
    return []
  try:
    answers = call('locate', {'locations': [{'lat': p[0], 'lon': p[1]} for p in points], 'costing': 'auto', 'verbose': True}) or []
  except Exception:
    cloudlog.exception("turnpilot controls: the exclusion points' locate failed: the route's bearings stay")
    return [None for _ in points]
  out = []
  for k, p in enumerate(points):
    cands = ((answers[k] if k < len(answers) else None) or {}).get('edges') or []
    hit = next((c for c in cands if int((c.get('edge_info') or {}).get('way_id') or 0) == int(p[2]) and
                bool((c.get('edge') or {}).get('forward', True)) == bool(p[3]) and c.get('heading') is not None), None)
    out.append(float(hit['heading']) if hit is not None else None)
  return out


def same_edge(key, own) -> bool:
  """A control's approach_key (way id, forward, its node) is the car's own edge (own_edge's): the way and direction equal
  and the nodes within SAME_EDGE_NODE_M."""
  return own is not None and key is not None and key[0] == own[0] and key[1] == own[1] and haversine(key[2], own[2]) <= SAME_EDGE_NODE_M


def road_ahead(call, position, heading: float) -> list[tuple[float, float]]:
  """The road ahead of the car at position (lat, lon) heading that way, [(lat, lon), ...] from where it is on its edge, edge
  after edge (module docstring); [] when no edge there goes that way."""
  first = _edge_from(call, position, heading, AHEAD_HEADING_TOL, AHEAD_RADIUS_M, False)
  if first is None:
    return []
  c, pts = first
  cum = cumulative(pts)
  s0 = min(max(float(c.get('percent_along', 0.)), 0.), 1.) * cum[-1]
  shape = [point_at(pts, cum, s0)] + [q for q, m in zip(pts, cum, strict=True) if m > s0 + 0.5]
  total = cumulative(shape)[-1]
  for _ in range(AHEAD_EDGES - 1):
    if total >= AHEAD_M:
      break
    # the bearing the road arrives with; the car's own at the very end of its edge
    arrive = bearing(shape[-2], shape[-1]) if len(shape) >= 2 else heading
    nxt = _edge_from(call, shape[-1], arrive, NEXT_HEADING_TOL, 1, True)
    if nxt is None or len(nxt[1]) < 2 or haversine(nxt[1][0], shape[-1]) > NODE_GAP_M:
      break
    shape += nxt[1][1:]
    total = cumulative(shape)[-1]
  return shape if len(shape) >= 2 else []


def controls_ahead(call, position, heading: float, tiles_dir=None, sidecar_dir=None, context: jn.Context | None = None,
                   cache: LocateCache | None = None) -> tuple[list[Control], str, list]:
  """(controls on the road ahead of the car, nearest first, status, the road [(lat, lon), ...]), off the route (module
  docstring). position: (lat, lon); heading: deg; the controls' route_m are m along the road from where the car was on it,
  their point (lat, lon) set. tiles_dir, sidecar_dir and statuses as route_controls'; failed also when no road there goes
  the car's way, or the heading is unknown. context as route_controls'; cache: Q1 and Q2 kept by edge id between
  lookups (LocateCache)."""
  try:
    if tiles_dir is not None and not (Path(tiles_dir) / MARKER).exists():
      return [], 'unsupported', []
    if heading is None or not math.isfinite(heading):
      cloudlog.warning("turnpilot controls ahead: no heading")
      return [], 'failed', []
    shape = road_ahead(call, position, heading)
    if len(shape) < 2:
      cloudlog.warning(f"turnpilot controls ahead: no road heading {heading:.0f} at {position}")
      return [], 'failed', []
    cum = cumulative(shape)
    steps = [{'geometry': {'coordinates': [[lo, la] for la, lo in shape]}, 'distance': cum[-1]}]
    controls, status = _route_controls(call, steps, tiles_dir, sidecar_dir, ahead=True, context=context, cache=cache)
    for c in controls:
      c.point = point_at(shape, cum, c.route_m)
    return controls, status, shape
  except Exception:
    cloudlog.exception("turnpilot controls ahead: failed")
    return [], 'failed', []


def osm_at(osm, entry: dict, pts, cum, begin_m: float, end_m: float, end_point) -> float | None:
  """m along the route of a sidecar entry on the edge from begin_m to end_m (module docstring), or None if it isn't its."""
  if entry.get('line'):
    hit = nearest(pts, cum, (float(entry['line'][0]), float(entry['line'][1])), begin_m - EDGE_END_M, end_m + EDGE_END_M)
    if hit is None or hit[1] > LINE_TOL_M or not begin_m + SPAN_START_M < hit[0] <= end_m + EDGE_END_M:
      return None
    return hit[0]
  pos = osm.junction_pos(entry.get('junction_node', 0))
  return end_m if pos is not None and haversine(pos, end_point) <= NODE_TOL_M else None


def phantom(osm, kind: str, others: list[str], hits, pts, cum, begin_m: float, end_m: float, end_point) -> bool:
  """Valhalla's `kind` on an edge is the phantom it makes of a stop on a graph node it can't direct (C14): only when the
  sidecar has the other direction's (others: its keys) directed `kind` ON the edge's end node (within PHANTOM_NODE_M) and
  nothing of ours on the edge, whatever its kind (hits), nor an undirected or both-ways one keyed to the other direction
  there. The sidecar leaving our direction out proves nothing by itself (a stop it can't tie to a junction, an old
  shard), so every other case keeps the flag: an extra stop is cheap, a dropped real one isn't."""
  if hits:
    return False
  entries = [x for k in others for x in osm.entries(k)]
  if any(x.get('evidence') in ('mid_undirected', 'mid_both') and x.get('kind') in ('stop', 'yield') and
         osm_at(osm, x, pts, cum, begin_m, end_m, end_point) is not None for x in entries):
    return False
  return any(x.get('kind') == kind and x.get('evidence') == 'mid_directed' and x.get('line') and
             haversine((float(x['line'][0]), float(x['line'][1])), end_point) <= PHANTOM_NODE_M for x in entries)


def _q1(call, edges, tpts, tcum, cache: LocateCache | None) -> list[dict | None]:
  """Q1: per trace edge, locate's candidate with its own id (its edge dict: stop_sign, yield_sign), from one batched verbose
  locate, radius 1, LOCATE_BACK_M back from its end; None unmatched. With a cache, only the edges it doesn't hold."""
  out: list[dict | None] = [cache.get('q1', e['id']) if cache is not None else None for e in edges]
  need = [i for i, c in enumerate(out) if c is None]
  locs = [dict(zip(('lat', 'lon'), point_back(tpts, tcum, edges[i]['end_shape_index'], LOCATE_BACK_M), strict=True)) | {'radius': 1}
          for i in need]
  located = call('locate', {'locations': locs, 'costing': 'auto', 'verbose': True}) if locs else []
  for i, loc in zip(need, located, strict=False):
    c = next((c.get('edge') or {} for c in (loc or {}).get('edges') or [] if (c.get('edge_id') or {}).get('value') == edges[i]['id']), None)
    out[i] = c
    if c is not None and cache is not None:
      cache.put('q1', edges[i]['id'], c)
  return out


def governed_node(edges, tpts, ends, a: int, pos) -> int:
  """The trace edge whose end node is the junction a control at the end of edge a governs: the sidecar's junction at pos
  (lat, lon) when it is on the route ahead within JUNCTION_AHEAD_M (a node on the way there: a crosswalk, an alley), else
  edge a's own."""
  if pos is None or haversine(tpts[edges[a]['end_shape_index']], pos) <= jn.survey_file.SAME_NODE_M:
    return a
  for i in range(a + 1, len(edges)):
    if ends[i][0] - ends[a][0] > JUNCTION_AHEAD_M:
      break
    if haversine(tpts[edges[i]['end_shape_index']], pos) <= jn.survey_file.SAME_NODE_M:
      return i
  return a


def road_junction(node) -> bool:
  """Another road meets at this end node: one of jn.CROSSING_USES and a known class, which cars aren't barred from (an
  alley, a driveway, a parking aisle or a path is none, and so is a node only our road passes: a crosswalk's, a signal's
  stop line, a way split). A driveability Valhalla leaves out counts as drivable here: the approach then ends sooner, which
  leaves the suspected stop as it was."""
  return any((x.get('use') or 'road') in jn.CROSSING_USES and x.get('driveability') != 'none' and
             jn.RANK.get(x.get('road_class') or '', 8) < 8 for x in node.get('intersecting_edges') or [])


def approach_edges(edges, ends, a: int) -> list[int]:
  """The trace edges of the approach to the junction at edge a's end, which a suspected stop there (N5) needs nothing
  mapped on: a, then each edge before it back to the previous road junction (road_junction) or JUNCTION_AHEAD_M, through
  the nodes no other road meets (OSM maps a signal on each approach's stop-line node, 10-15 m short of the junction, and
  Valhalla splits an approach at a crosswalk, an alley or a driveway). A road junction or an internal-intersection edge
  within SAME_JUNCTION_M of a's end is a's own junction (a divided road whose median Valhalla doesn't flag internal) and
  the walk goes on past it."""
  out = [a]
  for i in range(a - 1, -1, -1):
    back = ends[a][0] - ends[i][0]
    if back > JUNCTION_AHEAD_M or (back > SAME_JUNCTION_M and (edges[i].get('internal_intersection') or road_junction(ends[i][3]))):
      break
    out.append(i)
  return out


def maneuver_at(steps, starts, stop_s: float, node_s: float) -> str:
  """The route's maneuver at a junction (N8): the first step starting from the stop point stop_s to MANEUVER_AFTER_M past
  the node node_s (m along the route's shape), else straight."""
  for k in range(1, len(steps)):
    if stop_s - 1e-6 <= starts[k][0] <= node_s + jn.MANEUVER_AFTER_M:
      m = steps[k].get('maneuver') or {}
      if m.get('type') in ('arrive', 'depart'):
        return 'straight'
      modifier = m.get('modifier') or ''
      return 'uTurn' if 'uturn' in modifier else 'left' if 'left' in modifier else 'right' if 'right' in modifier else 'straight'
  return 'straight'


def rail_distance(osm, pts, cum, begin_m: float, end_m: float, node) -> float:
  """MapJunction.railDistance: m to the nearest mapped rail crossing (the sidecar's), along the route for one on the
  control's own edge (begin_m to end_m) however far back, else straight-line from the node within RAIL_M; RAIL_NONE without
  one, RAIL_UNKNOWN without a current sidecar."""
  if osm is None or osm.status != 'ok':
    return jn.RAIL_UNKNOWN
  best = None
  for r in osm.rail:
    p = (float(r['lat']), float(r['lon']))
    straight = haversine(p, node)
    if straight > end_m - begin_m + jn.RAIL_M + jn.RAIL_ON_ROUTE_M:
      continue
    hit = nearest(pts, cum, p, begin_m, end_m)
    if hit is not None and hit[1] <= jn.RAIL_ON_ROUTE_M:
      d = max(end_m - hit[0], 0.)
    elif straight <= jn.RAIL_M:
      d = straight
    else:
      continue
    best = d if best is None else min(best, d)
  return jn.RAIL_NONE if best is None else best


def _route_controls(call, steps, tiles_dir, sidecar_dir=None, ahead: bool = False, audit: list | None = None, opposing: Opposing | None = None,
                    context: jn.Context | None = None, cache: LocateCache | None = None) -> tuple[list[Control], str]:
  """ahead: the road ahead of the car (controls_ahead), not a route: its end is no destination, and its junctions have no
  maneuver. opposing: filled from Q2's batched locate (a route's). context: surveys and overrides (none: neither). cache:
  Q1 and Q2 by edge id (controls_ahead's)."""
  context = context if context is not None else jn.Context()
  pts = route_shape(steps)
  if len(pts) < 2:
    return [], 'failed'
  cum = cumulative(pts)
  starts = step_frame(steps, pts, cum)
  t, status = trace(call, pts)
  tpts = decode6(t['shape']) if isinstance(t.get('shape'), str) else [(p['lat'], p['lon']) for p in t.get('shape', [])]
  tcum = cumulative(tpts)
  scale = cum[-1] / tcum[-1] if tcum and tcum[-1] > 0. else 1.
  edges = t['edges']  # internal-intersection edges too: a control in a median is the junction's before it
  located = _q1(call, edges, tpts, tcum, cache)
  sim = load_sidecar(tiles_dir)
  osm = osm_sidecar.load(sidecar_dir, pts) if sidecar_dir is not None else None
  osm_ok = osm is not None and osm.status == 'ok'
  ends = []                   # per edge: (end m on the route, span m, key, its end node)
  at = {}                     # anchor edge -> {kind: [(source edge, osm entry or None, m along or None)]}
  signs = {}                  # anchor edge -> [(source edge, osm entry, m along)]: every sidecar sign there, far orphans too
  unmatched = anchor = 0
  for i, (e, candidate) in enumerate(zip(edges, located, strict=False)):
    internal = bool(e.get('internal_intersection'))
    if not internal or not ends:
      anchor = i
    if candidate is None:
      unmatched += 0 if internal else 1
      candidate = {}
    end_i, begin_i = e['end_shape_index'], e['begin_shape_index']
    span = (tcum[end_i] - tcum[begin_i]) * scale
    # where the edge ends on the route's own shape, near where the trace says
    expect = tcum[end_i] * scale
    end_m = project(pts, cum, tpts[end_i], expect - 30., expect + 30.)
    end_m = expect if end_m is None else end_m
    key = f"{e.get('way_id')}:{'f' if e.get('forward') else 'b'}"
    ends.append((end_m, span, key, e.get('end_node') or {}))
    valhalla = (['signal'] if e.get('traffic_signal') or (e.get('end_node') or {}).get('traffic_signal') else []) + \
      (['stop'] if candidate.get('stop_sign') else []) + (['yield'] if candidate.get('yield_sign') else [])
    hits, phantoms, far = [], [], []
    if osm_ok:
      for entry in osm.entries(key):
        m = osm_at(osm, entry, pts, cum, end_m - span, end_m, tpts[end_i])
        if m is not None and entry.get('kind') in ('stop', 'yield'):
          hits.append((entry, m))
      # an orphan whose sign is far before its anchor edge's end: left out (ORPHAN_NEAR_M), never placed at the sign either
      # (a reversed direction tag would then stop the car past a junction); still ours on the edge for phantom()
      for entry, m in hits:
        before = ends[anchor][0] - m
        if not entry.get('junction_node') and entry.get('line') and before > ORPHAN_NEAR_M:
          far.append((entry, m, before))
          cloudlog.info(f"turnpilot controls: the sidecar's {entry['kind']} on {key} at {m:.0f} m is tied to no junction and " +
                        f"{before:.0f} m before its edge's end: left out (that end would put it {before - CONTROL_LINE_M:.0f} m past its sign)")
      # the other direction: of our way, and, where no other road meets the end node (a way split, never a junction the
      # route may turn at), of the next edge's way (a split's sign is keyed to the way its traffic arrives on)
      split = not (e.get('end_node') or {}).get('intersecting_edges')
      nxt = edges[i + 1] if i + 1 < len(edges) and split else {}
      others = list(dict.fromkeys(f"{x['way_id']}:{'b' if x.get('forward') else 'f'}" for x in (e, nxt) if x.get('way_id') is not None))
      for kind in ('stop', 'yield'):
        if kind in valhalla and phantom(osm, kind, others, hits, pts, cum, end_m - span, end_m, tpts[end_i]):
          valhalla.remove(kind)
          phantoms.append(kind)
          cloudlog.info(f"turnpilot controls: Valhalla's {kind} on {key} at {end_m:.0f} m is a phantom: the OSM sidecar's is " +
                        f"directed {' or '.join(others)}, on the edge's end node")
    placed = [(entry, m) for entry, m in hits if not any(entry is x for x, _, _ in far)]
    if audit is not None:
      audit.append({'edge': i, 'key': key, 'internal': internal, 'anchor': anchor, 'begin_m': end_m - span, 'end_m': end_m,
                    'valhalla': list(valhalla), 'phantom': phantoms, 'osm': [dict(entry, m=m) for entry, m in placed],
                    'orphans': [dict(entry, m=m, before_m=before) for entry, m, before in far]})
    for entry, m in hits:
      signs.setdefault(anchor, []).append((i, entry, m))
    kinds = at.setdefault(anchor, {}) if valhalla or placed else None
    for kind in valhalla:
      kinds.setdefault(kind, []).append((i, None, None))
    for entry, m in placed:
      kinds.setdefault(entry['kind'], []).append((i, entry, m))
  if edges and unmatched > PARTIAL_UNMATCHED * sum(1 for e in edges if not e.get('internal_intersection')) and status == 'ok':
    status = 'partial'
  if osm is not None and not osm_ok:
    cloudlog.error(f"turnpilot controls: the OSM stop-sign sidecar is {osm.status} ({osm.detail}): the stops Valhalla doesn't flag are " +
                   "unknown, said as partial")
    if status in ('ok', 'walkOrSnap'):
      status = 'partial'
  if not context.overrides_ok:
    cloudlog.error(f"turnpilot controls: {context.overrides_error}: every stop and yield junction is failed, said as partial")
    if status in ('ok', 'walkOrSnap'):
      status = 'partial'
  real = [i for i, e in enumerate(edges) if not e.get('internal_intersection')]

  found = []
  for a, kinds in at.items():
    if not kinds:
      continue
    kind = min(kinds, key=KIND_RANK.get)
    end_m, span, key, node = ends[a]
    if end_m > cum[-1] - END_OF_ROUTE_M and not ahead:
      continue  # the route ends on this edge (a destination mid-edge snaps onto either direction): no junction ahead
    crossing = [ROAD_CLASS_RANK.get(x.get('road_class'), 8) for x in node.get('intersecting_edges') or []]
    junction = bool(crossing)
    ours = ROAD_CLASS_RANK.get(edges[a].get('road_class'), 8)
    cross_suspect = junction and all(ours < c for c in crossing) and kind in ('stop', 'yield')
    line_m, mapped, instance, source = end_m - CONTROL_LINE_M, False, '', 'default'
    own = sorted(((m, entry) for i, entry, m in signs.get(a, []) if i == a and entry.get('kind') == kind and entry.get('line')),
                 key=lambda x: x[0])
    osm_node = next((int(entry.get('junction_node') or 0) for _, entry, _ in kinds[kind] if entry is not None), 0)
    sim_entry = sim.get(key)
    if sim_entry is not None and sim_entry.get('kind') == kind and sim_entry.get('line'):
      p = project(pts, cum, tuple(sim_entry['line']), end_m - span - 1., end_m + 1.)
      if p is not None:
        line_m, mapped, instance, source = p, True, sim_entry.get('instance', ''), 'sim'
    sign_m = None
    if own:
      sign_m, entry = own[-1]  # the sign nearest the junction, if the way has two: data, never the stop point
      osm_node = int(entry.get('junction_node') or 0) or osm_node
    failed = osm is not None and not osm_ok and kind in ('stop', 'yield')
    found.append({'s': line_m - MAP_MARGIN_M, 'kind': kind, 'mapped': mapped, 'junction': junction, 'cross_suspect': cross_suspect,
                  'instance': instance, 'source': source, 'osm_node': osm_node, 'status': 'failed' if failed else 'ok', 'a': a,
                  'sign_m': sign_m, 'suspected': False, 'legs': None})
  # suspected unmapped stops (N5): a junction with nothing mapped on its whole approach (approach_edges) where every road
  # crossing outranks ours by two classes or more is emitted as a stop, one per junction
  suspected: set[int] = set()
  for a in real if SUSPECTED_STOPS else []:
    end_m, span, key, node = ends[a]
    if a in at or not node.get('intersecting_edges') or (end_m > cum[-1] - END_OF_ROUTE_M and not ahead):
      continue
    nodes, legs, last = jn.junction_legs(edges, tpts, a, ahead)
    if not jn.suspected_stop(edges[a], tpts, legs):
      continue
    mapped = next((i for i in approach_edges(edges, ends, a) if i in at or i in suspected), None)
    if mapped is not None:
      what = 'a suspected stop' if mapped in suspected else 'a ' + '/'.join(sorted(at[mapped], key=KIND_RANK.get))
      cloudlog.info(f"turnpilot controls: no suspected stop on {key} at {end_m:.0f} m: its approach has {what} at {ends[mapped][0]:.0f} m")
      continue
    suspected.add(a)
    cloudlog.info(f"turnpilot controls: a suspected unmapped stop on {key} at {end_m:.0f} m: every road crossing outranks ours by two classes")
    found.append({'s': end_m - CONTROL_LINE_M - MAP_MARGIN_M, 'kind': 'stop', 'mapped': False, 'junction': True, 'cross_suspect': False,
                  'instance': '', 'source': 'default', 'osm_node': 0, 'status': 'failed' if osm is not None and not osm_ok else 'ok',
                  'a': a, 'sign_m': None, 'suspected': True, 'legs': (nodes, legs, last)})

  # Q2: every stop and yield's junction, in one batched verbose locate with the route's opposing roads. A control whose
  # edge ends short of the junction the sidecar says it governs (Valhalla splits an approach at a crosswalk or an alley) is
  # read at that junction's node, when it is on the route ahead
  jobs = []
  for f in found:
    if f['kind'] not in ('stop', 'yield'):
      continue
    f['node_edge'] = governed_node(edges, tpts, ends, f['a'], osm.junction_pos(f['osm_node']) if osm_ok and f['osm_node'] else None)
    nodes, legs, last = f['legs'] or jn.junction_legs(edges, tpts, f['node_edge'], ahead)
    jobs.append((f, nodes, legs, last))
  opp_legs, locs = opposing_probes(steps) if opposing is not None else ({}, [])
  n_opp, spans = len(locs), []
  for f, nodes, legs, last in jobs:
    mine = jn.probes(nodes, legs)
    qkey = (edges[f['node_edge']].get('id'), tuple(edges[k].get('id') for k in range(f['node_edge'] + 1, last + 1)),
            edges[last + 1].get('id') if last + 1 < len(edges) else None)
    hit = cache.get('q2', qkey) if cache is not None else None
    spans.append((qkey, None, hit) if hit is not None else (qkey, (len(locs), len(mine)), None))
    if hit is None:
      locs += mine
  answers = []
  if locs:
    try:
      answers = call('locate', {'locations': locs, 'costing': 'auto', 'verbose': True}) or []
    except Exception:
      cloudlog.exception("turnpilot controls: the junctions' locate (Q2) failed: their legs are unknown, each junction failed")
      answers = None
  if opposing is not None:
    opposing.roads = opposing_parse(opp_legs, answers[:n_opp]) if answers is not None else {}
  for (f, nodes, legs, last), (qkey, span_, hit) in zip(jobs, spans, strict=True):
    if hit is not None:
      mine = hit
    elif answers is None or len(answers) < span_[0] + span_[1]:
      mine = None
    else:
      mine = answers[span_[0]:span_[0] + span_[1]]
      if cache is not None:
        cache.put('q2', qkey, mine)
    a, k = f['a'], f['node_edge']
    end_m = ends[k][0]  # the junction's node
    begin_m = ends[a][0] - ends[a][1]  # the control's own edge's start
    j = jn.build(edges, tpts, k, nodes, legs, last, mine)
    j.suspected, j.sidecar = f['suspected'], osm_ok
    if f['status'] == 'failed' or not context.overrides_ok:
      j.status = 'failed'
    if len(nodes) == 1 and not ends[k][3].get('intersecting_edges'):
      # no other road at its node and no junction found ahead for it: nothing to judge cross traffic by
      cloudlog.warning(f"turnpilot controls: the {f['kind']} on {ends[a][2]} at {ends[a][0]:.0f} m is at a node no other road meets " +
                       "and its junction wasn't found ahead: its junction failed")
      j.status = 'failed'
    j.osm_node_id = jn.osm_node_at(osm if osm_ok else None, f['osm_node'], nodes[0])
    jn.classify(j, osm, not f['suspected'], context)
    j.rail_distance = rail_distance(osm, pts, cum, begin_m, end_m, nodes[0])
    j.maneuver = 'unknown' if ahead else maneuver_at(steps, starts, f['s'], end_m)
    sv = jn.survey_approach(j.survey if j.surveyed else None, j.our_way, j.heading_in)
    off, j.stop_point_source = jn.stop_offset(j, end_m - f['s'], f['mapped'], sv.get('stopPointM') if sv else None)
    f['s'], j.stop_point_distance = end_m - off, off
    f['status'] = j.status
    f['info'] = j

  controls = []
  for i, f in enumerate(sorted(found, key=lambda c: c['s'])):
    a = f['a']
    end_m, span, key, node = ends[a]
    step_idx, along, route_m = to_step(starts, max(f['s'], 0.))
    back = min(EXCLUDE_BACK_M, span / 2.)
    ex = point_at(pts, cum, end_m - back)
    ex_bearing = bearing(point_at(pts, cum, end_m - back - 1.), point_at(pts, cum, end_m - back + 1.))
    nd = tpts[edges[a]['end_shape_index']]
    controls.append(Control(i, f['kind'], route_m, step_idx, along, f['mapped'], f['junction'], f['cross_suspect'], f['instance'],
                            line_source=f['source'], osm_node=f['osm_node'],
                            osm_sign_m=None if f['sign_m'] is None else to_step(starts, max(f['sign_m'], 0.))[2], junction_status=f['status'],
                            edge=a, junction_info=f.get('info'),
                            approach_key=(int(edges[a].get('way_id') or 0), bool(edges[a].get('forward', True)), (round(nd[0], 6), round(nd[1], 6))),
                            exclusion=(ex[0], ex[1], ex_bearing), first_edge=bool(real) and a == real[0],
                            last_edge=bool(real) and a == real[-1]))
  side = 'none expected' if osm is None else osm.status
  cloudlog.info(f"turnpilot controls: {len(controls)} on the route ({status}), {unmatched}/{len(edges)} edges unmatched, OSM sidecar {side}")
  return controls, status
