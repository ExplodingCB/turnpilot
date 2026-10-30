"""The scenarios: one declarative Entry each, and the builders that turn an entry into a Plan of world positions from the
road-graph export (no game). Positions are BeamNG world meters (x east, y north, z up), headings radians CCW from east.

Turn scenarios name a signalized junction from piston_assets.md; the junctions with marked crosswalks are (-768.6, 555.2),
(-759.2, 376.7), (-650.1, 481.7), (-600.7, 220.9), (-445.3, 147.0) and (-32.5, 403.8). The last two have no 90-degree
turn in the road graph. Data scenarios (lane_change, occupied_lane) read a JSON list of
  {"name", "spawn": [x, y, z], "heading_deg", "dest": [x, y, z], "junction": [x, y], "turn": "left"|"right", "speed_kph"}
(junction, turn and speed_kph optional), from lane_change_junctions.json next to this file unless --data says otherwise.

The sim tiles (west_coast_usa_signals) map a stop line for every signal and stop sign of the level, from the committed
wcusa_map_node_signals.json (mapsignals.py). Every spawn is clear of them: more than the rolling start plus a map stop's
engage distance before each one on its approach (spawn_clear_m), or past it, so the car never runs over a line before
openpilot is engaged to stop for it (the smoke's "Stop Line Passed"). A turn scenario's spawn moves to the nearest
clear spot, a data entry that isn't clear is refused.
"""
import json
import math
import os
from dataclasses import dataclass, field

import numpy as np

from openpilot.selfdrive.turnpilot.control_tracker import DROP_PAST_M
from openpilot.selfdrive.turnpilot.lane_planner import LANE_WIDTH, lane_change_window
from openpilot.selfdrive.turnpilot.map_controls import MAP_STOP_DECEL, PROTECT_BEFORE_M, engage_distance
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M
from openpilot.tools.sim.bridge.beamng.roadgraph import RoadGraph, wrap
from openpilot.tools.sim.bridge.beamng.scenarios.timing import WALK_SPEED, arc_lengths, oncoming_release_delay, point_at, travel_time

DATA_JSON = os.path.join(os.path.dirname(__file__), "lane_change_junctions.json")
SIGNALS_JSON = os.path.join(os.path.dirname(os.path.dirname(__file__)), "wcusa_map_node_signals.json")
PREFIX = "tp_sc_"  # every actor a scenario spawns; the cleanup sweep deletes anything named like this
TURN_TOLERANCE_DEG = 15.
STRAIGHT_TOLERANCE_DEG = 25.
CURB_STEP_M = 1.5        # walker start and end, beyond the road edge
CROSSWALK_PAST_EDGE_M = 2.5  # the crosswalk across the exit road, this far past the edge of the road the car comes from
WALK_AWAY_M = 10.        # after crossing, the walker goes this far along the far sidewalk, out of the crosswalk zone
PACER_END_BEFORE_M = 50.  # the pacer's lane ends this far before the junction
CLEAR_OF_JUNCTION_M = 25.  # a spawn this close to another junction moves up the approach
QUEUE_GAP_M = 8.         # a gap stream's cars stand this far apart, front to front (a queue)
ROAD_MARGIN_M = 150.     # the road model covers the box around spawn, junction and destination, this much bigger
ROLL_FRACTION = 0.8      # the driver rolls up to this much of the set speed before engaging, as gate1 does (run.py)
ROLL_ACCEL = 1.0         # m/s^2 of that roll (bridge.ROLLING_START_ACCEL)
ENGAGE_LAG_S = 1.        # s from rolled to engaged (bridge.engage_step: the stalk once engageable)
SPAWN_PAST_M = DROP_PAST_M  # a spawn more than spawn_clear_m before a control, or this far past it (the tracker drops it here)
MIN_APPROACH_M = 60.     # a turn scenario's spawn no closer to its junction than this
LINE_LATERAL_M = 8.      # a control's stop line within this of the approach's centreline is on it (lanes either side)
LINE_ALIGNED_DEG = 40.   # ...if it controls traffic within this of the approach's direction
LINE_ASSOC_M = 40.       # a stop line this close to the junction nearest it is that junction's...
INTO_DEG = 60.           # ...if it controls traffic heading into it, within this of the bearing to it


@dataclass(frozen=True)
class Entry:
  kind: str                       # 'turn' (at a junction), 'straight' (past one), 'data' (from the JSON), 'route'
  junction: tuple = ()            # (x, y) of a signalized junction
  turn: str = 'right'
  approach_m: float = 140.        # spawn this far before the junction (closer if that is inside another junction)
  from_heading_deg: float | None = None  # approach travelling this way; None: the longest approach
  dest_after_m: float = 60.       # destination this far past it
  speed_kph: float = 30.
  timeout_s: float = 150.
  actor: str = ''                 # 'crosswalk', 'oncoming', 'standing', 'pacer'
  crosswalk_m: float = 0.         # crosswalk centre from the junction along the exit road; 0: road edge + 2.5 m (unverified)
  ped_before_m: float = 40.       # standing mannequin, this far before the junction
  oncoming: int = 2               # cars
  oncoming_start_m: float = 35.   # the first one starts this far from the junction (released at the car's stop: nearer
                                  # than ~36 m at 11 m/s, 2.5 m/s^2, it can still come arrive_after_s after the go)
  oncoming_spacing_m: float = 25.
  oncoming_speed: float = 11.     # m/s
  oncoming_accel: float = 2.5     # m/s^2
  arrive_after_s: float = 3.5     # the first oncoming car reaches the junction this long after the car's go there
  # a gap stream (the autonomous unprotected left's gate, G6): the oncoming cars queue across the junction QUEUE_GAP_M apart
  # from oncoming_start_m, and each is released on its own timer from the car's stand at its line (never on its go), so
  # the first reaches the junction stream_first_s after the stand and each one after it stream_headway_s later; 0: none
  stream_headway_s: float = 0.
  stream_first_s: float = 6.
  pace_block_s: float = 6.        # the pacer keeps the target lane occupied this long into the lane change window
  end_on_stop_s: float = 0.       # end the run once the car has stood this long after moving
  spawn: tuple = ()               # route: (x, y, z), heading_deg, destination (x, y, z)
  heading_deg: float = 0.
  dest: tuple = ()
  # the stop-sign logic gate (STOP_SIGNS.md v3 8.2, SG4c): cross cars, each (side: left, right or lead (ours, in our lane,
  # ahead of the car), start: 'line' (standing at its stop line from the start) or m back from it, arrive: s after the car's
  # stand at its line, or ('near', m) once the car is within m of the junction, a car starting back drives up to its line
  # and stops there (None: it stands), release: s after that
  # stand, 'go' (the car's go: out of turn when it arrived after the car) or None (never) it goes on from its line, through:
  # across the junction and away); from_stop_arm: the car's approach is the junction's arm with a stop line (a two-way)
  cross: tuple = ()
  from_stop_arm: bool = False


SCENARIOS = {
  'ped_crosswalk_right': Entry('turn', junction=(-768.6, 555.2), turn='right', actor='crosswalk', timeout_s=120.),
  # the car stands at the junction's signal line (11-27 m out) for the driver's go, 2 s after the stop; the first oncoming
  # car reaches the junction 3.5 s after that go, as the car pulls away toward the oncoming lanes, so the detector's
  # oncoming hold is what decides when it turns
  'unprotected_left': Entry('turn', junction=(-650.1, 481.7), turn='left', actor='oncoming', arrive_after_s=3.5, timeout_s=150.),
  # unprotected_left comes downhill onto a shelf on a ~21% hill, and the arm straight across drops away past it: the
  # oncoming cars are out of the cameras' view until they top the shelf, about 13 m from the car. This is the same
  # junction from the south-east, up a gentle grade, with the arm across rising about 3 m over its first 85 m.
  'unprotected_left_visible': Entry('turn', junction=(-650.1, 481.7), turn='left', from_heading_deg=137., actor='oncoming',
                                    arrive_after_s=3.5, timeout_s=150.),
  # the autonomous left's gate (CRITIQUE REQUIRED 5: a 9 m hold, no approach pulse, a stand of 10 s or more, a cold go
  # pulse): the visible approach, a queue of cars at the mapped 25 mph through the junction at 4, 6, 8 and 10 s headways,
  # then an empty road. Its R_req (133 m) is past the 106 m straight arm, so every gap here is under what it may take: it
  # stands through the stream and goes after the last car (a missed gap is counted, never failed: score.py)
  **{f'unprotected_left_gap{h}': Entry('turn', junction=(-650.1, 481.7), turn='left', from_heading_deg=137., actor='oncoming',
                                       oncoming=4 if h < 10 else 3, oncoming_start_m=40., oncoming_speed=11.18,
                                       stream_headway_s=float(h), timeout_s=150. + 10. * h) for h in (4, 6, 8, 10)},
  # gate1's approach: eastbound on the 140 m straight from the bridge's default spawn
  'ped_in_path': Entry('straight', junction=(-768.9, 661.4), turn='right', from_heading_deg=0., actor='standing', end_on_stop_s=4.,
                       timeout_s=90.),
  'occupied_lane': Entry('data', actor='pacer', speed_kph=45., timeout_s=150.),
  'lane_change': Entry('data', speed_kph=45., timeout_s=150.),
  # the stop-sign logic gate (STOP_SIGNS.md 8.2), on the sim's mapped stops: DR439_5, a four-way all-way three blocks north
  # of ped_in_path's junction (all four arms stopped, surveyed synthetic in maps/sim), straight across; DR560_2, a T whose stem
  # alone stops (a two-way), from the stem. Run with TurnPilotAvoidStops off (navd keeps its junctions) and the sim gate on
  # (the cross-traffic oracle, the sim's files): logic checks, never range evidence. empty: no one else (a go, no hard
  # braking); cross_first: a car standing at the left line before ours arrives goes 3 s after our stand (we go after it,
  # PET 2 s or more); queue: a lead standing at our line goes 3 s after we stand behind it (a roll-up, then our own stop and
  # go); out_of_turn: a car up to the left line after our stand goes across on our go (a withdrawn go, then ours); shield: a
  # car 16 m back sent up to the left line as the car comes within 30 m of the junction (about 5 s each: it stands there
  # after our stand, before our go) waits there (its lane shielded: a go, no handback; untested if it wasn't there by the
  # go); two_way: only our approach
  # stops (a handback, no go)
  'stop_allway_empty': Entry('straight', junction=(-768.8, 769.2), turn='right', timeout_s=90.),
  'stop_allway_cross_first': Entry('straight', junction=(-768.8, 769.2), turn='right', timeout_s=90.,
                                   cross=(('left', 'line', None, 3., True),)),
  'stop_allway_queue': Entry('straight', junction=(-768.8, 769.2), turn='right', timeout_s=90., cross=(('lead', 'line', None, 3., True),)),
  'stop_allway_out_of_turn': Entry('straight', junction=(-768.8, 769.2), turn='right', timeout_s=90.,
                                   cross=(('left', 10., 0., 'go', True),)),
  'stop_allway_shield': Entry('straight', junction=(-768.8, 769.2), turn='right', timeout_s=90.,
                              cross=(('left', 16., ('near', 30.), None, False),)),
  'stop_two_way': Entry('turn', junction=(-768.2, 872.9), turn='right', approach_m=77., timeout_s=60., from_stop_arm=True, end_on_stop_s=6.),
  # downtown: 20 m on from the bridge's default spawn (clear of the junction behind it), eastbound, to road-graph node
  # DR249_3 about 850 m east-southeast
  'route_demo': Entry('route', spawn=(-888.82, 659.66, 111.4), heading_deg=0., dest=(-84.9, 377.0, 101.5), speed_kph=40., timeout_s=600.),
}


@dataclass(frozen=True)
class MappedControl:
  x: float
  y: float
  heading: float  # rad, the direction of the traffic it controls
  kind: str       # 'signal' or 'stop'
  instance: str   # the level's signal instance ('trafficLight 46', 'stop 71')

  def __str__(self):
    return self.instance


@dataclass
class Actor:
  name: str
  kind: str            # 'walker' (the mannequin) or 'car' (an AI etk800)
  trigger: str         # 'setup' (stands from the start), 'walk', 'oncoming', 'pace'
  path: list           # [[x, y, z], ...]: the walk, or the car's lane (its script starts at the first point)
  speed: float = WALK_SPEED
  accel: float = 0.
  heading: float = 0.  # where it faces at spawn
  params: dict = field(default_factory=dict)


@dataclass
class Plan:
  name: str
  spawn: tuple         # (x, y, z) for the ego
  heading: float
  dest: tuple | None   # (x, y, z), or None for no route
  junction: tuple | None  # (x, y) of the maneuver point the triggers time against
  speed_kph: float
  timeout_s: float
  end_on_stop_s: float
  actors: list
  notes: dict
  road: dict | None = None  # road_model() around the run, for the score's oncoming-lane check


# *** road geometry ***
def heading_of(a, b) -> float:
  return math.atan2(b[1] - a[1], b[0] - a[0])


def unit(h: float) -> np.ndarray:
  return np.array([math.cos(h), math.sin(h)])


def right_of(h: float) -> np.ndarray:
  return np.array([math.sin(h), -math.cos(h)])


def lane_offset(lanes: str, radius: float, which: str) -> float:
  """Offset to the right of the centreline of our rightmost or leftmost lane ('+' in a lane string, left to right)."""
  ours = [k for k, c in enumerate(lanes) if c == '+'] or [len(lanes) - 1]
  k = ours[-1] if which == 'right' else ours[0]
  w = 2 * radius / max(len(lanes), 1)
  return -radius + w * (k + 0.5)


def flip(lanes: str) -> str:
  """A lane string for travel the other way."""
  return ''.join('+' if c == '-' else '-' for c in reversed(lanes))


def offset_polyline(pts, offset: float) -> np.ndarray:
  """The polyline moved `offset` m to the right of its direction of travel (first point to last)."""
  pts = np.asarray(pts, dtype=float)
  d = np.gradient(pts[:, :2], axis=0)
  n = np.stack([d[:, 1], -d[:, 0]], axis=1) / np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
  out = pts.copy()
  out[:, :2] += offset * n
  return out


def section(pts, s0: float, s1: float) -> np.ndarray:
  """The polyline between arc lengths s0 and s1."""
  seg = arc_lengths(pts)
  inner = [p for p, a in zip(pts, seg, strict=True) if s0 < a < s1]
  return np.array([point_at(pts, s0)[0]] + inner + [point_at(pts, s1)[0]])


def cut(pts, s: float) -> np.ndarray:
  """The first s m of a polyline."""
  return section(pts, 0., s)


def walk_straight(graph: RoadGraph, j: str, first: str, length: float) -> np.ndarray:
  """Points from junction j along the arm through `first`, carrying on straight through junctions, until `length` m."""
  pos = graph.pos
  pts, prev, cur, d = [pos[j]], j, first, 0.0
  while True:
    d += float(np.linalg.norm(pos[cur][:2] - pos[prev][:2]))
    pts.append(pos[cur])
    nxt = graph.adj[cur] - {prev}
    if d >= length or not nxt:
      return np.array(pts)
    h = heading_of(pos[prev], pos[cur])
    turn = {n: abs(wrap(heading_of(pos[cur], pos[n]) - h)) for n in nxt}
    best = min(turn, key=lambda n: turn[n])
    if len(nxt) > 1 and math.degrees(turn[best]) > STRAIGHT_TOLERANCE_DEG:
      return np.array(pts)
    prev, cur = cur, best


def arm_out_heading(graph: RoadGraph, j: str, a: str) -> float:
  return graph.arm_heading(graph.walk(j, a, 30.))


def lane_pose(arm, s: float, lanes: str, radius: float, which: str):
  """(x, y, z), heading of a car s m out along an arm (points from the junction outward), in a lane, facing the junction.
  lanes: oriented for travel toward the junction."""
  p, h_out = point_at(arm, s)
  heading = wrap(h_out + math.pi)
  xy = p[:2] + lane_offset(lanes, radius, which) * right_of(heading)
  return (float(xy[0]), float(xy[1]), float(p[2]) + 0.3), heading


def mapped_controls(graph: RoadGraph, path: str = SIGNALS_JSON) -> list[MappedControl]:
  """The level's stop lines as the sim tiles map them (mapsignals.py): mapNodeSignals[a][b] controls traffic a -> b, its
  pos the stop line. A pair whose nodes the road graph lacks has no direction here, and is left out."""
  with open(path) as f:
    signals = json.load(f)['mapNodeSignals']
  out = []
  for a, by_b in signals.items():
    for b, items in by_b.items():
      if a not in graph.pos or b not in graph.pos:
        continue
      h = heading_of(graph.pos[a], graph.pos[b])
      for it in items:
        kind = 'signal' if it['instance'].startswith('trafficLight') else 'stop'
        out.append(MappedControl(float(it['pos']['x']), float(it['pos']['y']), h, kind, it['instance']))
  return out


def controls_along(pts, controls) -> list[tuple[float, MappedControl]]:
  """(arc length, control) of the controls on a polyline in its direction of travel, nearest its start first: the line
  within LINE_LATERAL_M of it, for traffic its way. Lines that project onto an end are past it (another approach's)."""
  pts = np.asarray(pts, dtype=float)
  total = float(arc_lengths(pts)[-1])
  out = []
  for c in controls:
    if min(np.hypot(pts[:, 0] - c.x, pts[:, 1] - c.y)) > total:
      continue
    s, off, h = project(pts, np.array([c.x, c.y]))
    if 0.5 < s < total - 0.5 and abs(off) < LINE_LATERAL_M and abs(math.degrees(wrap(c.heading - h))) < LINE_ALIGNED_DEG:
      out.append((s, c))
  return sorted(out, key=lambda sc: sc[0])


def spawn_clear_m(speed_kph: float) -> float:
  """m a spawn must be before a mapped control: the rolling start up to ROLL_FRACTION of the set speed, ENGAGE_LAG_S at
  that speed until openpilot is engaged, and from there a map stop's engage distance (map_controls.engage_distance: its
  comfortable curve, a second's lag and 15 m to spare). BeamNG smoke 2026-10-28: 20.5 m ran over the line."""
  v = ROLL_FRACTION * speed_kph / 3.6
  return v ** 2 / (2. * ROLL_ACCEL) + ENGAGE_LAG_S * v + engage_distance(v)


def spawn_conflicts(s_spawn: float, along, speed_kph: float) -> list[tuple[float, MappedControl]]:
  """(m from the spawn, control) of the controls along the approach too close to it: ahead within spawn_clear_m, or behind
  by less than SPAWN_PAST_M."""
  clear = spawn_clear_m(speed_kph)
  return [(s - s_spawn, c) for s, c in along if -SPAWN_PAST_M < s - s_spawn < clear]


def junction_control(along, total: float) -> tuple[str, float] | None:
  """(kind, m before the junction) of the junction's own control on its approach: a line within PROTECT_BEFORE_M before
  it (map_controls.protected's reach), or None."""
  own = [(total - s, c) for s, c in along if -2. < total - s <= PROTECT_BEFORE_M]
  if not own:
    return None
  d, c = min(own, key=lambda dc: dc[0])
  return c.kind, round(float(d), 1)


def window_controls(along, total: float, v: float) -> list[tuple[float, str]]:
  """(m before the junction, instance) of the controls that stop the car in the lane change window, or so close before it
  that it isn't back up to speed when the window opens (at MAP_STOP_DECEL, as gently as it braked)."""
  start, end = lane_change_window(v)
  reach = start + v ** 2 / (2. * MAP_STOP_DECEL)
  return [(round(float(total - s), 1), c.instance) for s, c in along if end < total - s <= reach]


@dataclass
class Junction:
  node: str
  center: np.ndarray
  a: str               # approach neighbour
  b: str               # exit neighbour
  approach: np.ndarray  # from the junction out along the approach, carried on straight through junctions
  exit: np.ndarray
  h_in: float          # travel heading on the approach
  straight: str | None  # the arm straight across, if any


def find_turn(graph: RoadGraph, xy, direction: str, approach_len: float = 400., from_heading: float | None = None) -> Junction:
  """The legal ~90 degree turn that way at the junction nearest xy: from the approach travelling closest to
  from_heading (rad), else from the longest approach."""
  j = graph.nearest_junction(*xy)
  best = None
  for a in graph.adj[j]:
    if j not in graph.out[a]:
      continue
    app = walk_straight(graph, j, a, approach_len)
    h_in = wrap(graph.arm_heading(app) + math.pi)
    for b in graph.adj[j]:
      if b == a or b not in graph.out[j]:
        continue
      ex = graph.walk(j, b, 120.)
      angle = wrap(graph.arm_heading(ex) - h_in)
      if abs(abs(math.degrees(angle)) - 90) <= TURN_TOLERANCE_DEG and (angle > 0) == (direction == 'left'):
        n = float(arc_lengths(app)[-1]) if from_heading is None else -abs(wrap(h_in - from_heading))
        if best is None or n > best[0]:
          best = (n, a, b, app, ex, h_in)
  if best is None:
    raise ValueError(f"no legal {direction} turn of about 90 degrees at {j} near {xy}")
  _, a, b, app, ex, h_in = best
  # straight across, two-way: oncoming traffic comes from it and a straight-on destination lies along it
  across = [o for o in graph.adj[j] if o not in (a, b) and j in graph.out[o] and o in graph.out[j] and
            abs(math.degrees(wrap(arm_out_heading(graph, j, o) - h_in))) < STRAIGHT_TOLERANCE_DEG]
  return Junction(j, graph.pos[j], a, b, app, ex, h_in, across[0] if across else None)


def ego_lane(direction: str) -> str:
  return 'left' if direction == 'left' else 'right'


def crosswalk_path(graph: RoadGraph, jn: Junction, crosswalk_m: float = 0.) -> tuple[np.ndarray, float]:
  """The walk across the exit road: from the inside corner of the turn, over the road, then away along the far sidewalk.
  Returns (path, crosswalk distance from the junction)."""
  h_out = graph.arm_heading(jn.exit)
  u, n = unit(h_out), unit(h_out + math.pi / 2)
  c_dist = crosswalk_m or graph.radius[jn.a] + CROSSWALK_PAST_EDGE_M
  c = jn.center[:2] + c_dist * u
  half = graph.radius[jn.b] + CURB_STEP_M
  near = 1.0 if np.dot(n, unit(jn.h_in + math.pi)) > 0 else -1.0  # the side of the exit road the car comes from
  z = float(jn.center[2])
  pts = [c + near * half * n, c - near * half * n, c - near * half * n + WALK_AWAY_M * u]
  return np.array([[p[0], p[1], z] for p in pts]), c_dist


def oncoming_paths(graph: RoadGraph, jn: Junction, count: int, start_m: float, spacing: float, after_m: float = 60.):
  """Lanes for cars coming straight across the junction toward the car: from start_m (+ spacing each) out on the far
  arm, through the junction, and away along the car's approach, each in its rightmost lane."""
  if jn.straight is None:
    raise ValueError(f"no arm straight across {jn.node} for oncoming traffic")
  far = walk_straight(graph, jn.node, jn.straight, start_m + spacing * (count - 1) + 5.)
  lanes_in = graph.lanes_toward(jn.node, jn.straight)
  lanes_out = flip(graph.lanes_toward(jn.node, jn.a))
  leave = offset_polyline(cut(jn.approach, after_m), lane_offset(lanes_out, graph.radius[jn.a], 'right'))
  length = float(arc_lengths(far)[-1])
  paths = []
  for k in range(count):
    s = start_m + k * spacing
    if s > length:
      break
    come = offset_polyline(cut(far, s)[::-1], lane_offset(lanes_in, graph.radius[jn.straight], 'right'))
    paths.append(np.vstack([come, leave[1:]]))
  return paths


def project(pts, p) -> tuple[float, float, float]:
  """(arc length, signed offset to the right, heading of the segment there) of point p against a polyline."""
  pts = np.asarray(pts, dtype=float)
  best = (math.inf, 0.0, 0.0, 0.0)
  seg = arc_lengths(pts)
  for i in range(len(pts) - 1):
    a, b = pts[i, :2], pts[i + 1, :2]
    ab = b - a
    f = float(np.clip(np.dot(p[:2] - a, ab) / max(np.dot(ab, ab), 1e-9), 0.0, 1.0))
    q = a + f * ab
    d = float(np.linalg.norm(p[:2] - q))
    if d < best[0]:
      h = math.atan2(ab[1], ab[0])
      best = (d, seg[i] + f * float(np.linalg.norm(ab)), float(np.dot(p[:2] - q, right_of(h))), h)
  return best[1], best[2], best[3]


def approach_arm(graph: RoadGraph, spawn, heading: float, junction_xy):
  """(the approach the spawn is on, in the car's direction of travel and ending at the junction; the spawn's arc length
  along it; its offset right of the centreline)."""
  j = graph.nearest_junction(*junction_xy)
  best = None
  for a in graph.adj[j]:
    if j not in graph.out[a]:
      continue
    toward = walk_straight(graph, j, a, 800.)[::-1]  # the arm in the car's direction of travel, ending at the junction
    s, off, h = project(toward, np.asarray(spawn, dtype=float))
    if abs(math.degrees(wrap(h - heading))) < 45. and (best is None or abs(off) < abs(best[2])):
      best = (toward, s, off)
  if best is None:
    raise ValueError(f"the spawn isn't on an approach to {j}")
  return best


def pacer_lane(graph: RoadGraph, spawn, heading: float, junction_xy, direction: str):
  """The lane next to the car's toward the turn side, from 30 m behind the spawn to PACER_END_BEFORE_M before the
  junction, and how far along it the junction is."""
  toward, s_spawn, off = approach_arm(graph, spawn, heading, junction_xy)
  lane = offset_polyline(toward, off + (-LANE_WIDTH if direction == 'left' else LANE_WIDTH))
  start, total = max(s_spawn - 30., 0.), float(arc_lengths(lane)[-1])
  return section(lane, start, total - PACER_END_BEFORE_M), total - start


STOP_LINE_NEAR_M = 30.   # a stop line this close before a junction is its arm's
CROSS_SPEED, CROSS_ACCEL = 8., 2.5  # m/s, m/s^2: a cross car's
CAR_HALF_M = 2.4         # m from a car's centre to its front: standing at its line, its front there
AFTER_M = 40.            # m a car going through drives on past the junction


def arm_line_m(graph: RoadGraph, j: str, arm: str, stops) -> float | None:
  """m before junction j its stop line lies on the arm arriving from neighbour arm, or None without one."""
  toward = walk_straight(graph, j, arm, 80.)[::-1]
  total = float(arc_lengths(toward)[-1])
  ds = [total - sa for sa, _ in controls_along(toward, stops) if -2. < total - sa <= STOP_LINE_NEAR_M]
  return min(ds) if ds else None


def all_way(graph: RoadGraph, j: str, stops) -> bool:
  """Every arm cars arrive at j by has its own stop line (the sim's mapped stops: an all-way), three or more."""
  arms = [a for a in graph.adj[j] if j in graph.out[a]]
  return len(arms) >= 3 and all(arm_line_m(graph, j, a, stops) is not None for a in arms)


def stop_arm_heading(graph: RoadGraph, xy, stops) -> float:
  """The travel heading (rad) of an arm with a stop line at the junction nearest xy (a two-way's stopped approach)."""
  j = graph.nearest_junction(*xy)
  for a in graph.adj[j]:
    if j in graph.out[a] and arm_line_m(graph, j, a, stops) is not None:
      return wrap(arm_out_heading(graph, j, a) + math.pi)
  raise ValueError(f"no arm with a stop line at {j}")


def side_arm(graph: RoadGraph, jn: Junction, side: str) -> str:
  """The arm of jn's junction cross traffic comes from on that side of the car's approach (lead: the car's own)."""
  if side == 'lead':
    return jn.a
  for o in graph.adj[jn.node]:
    if o in (jn.a, jn.straight) or jn.node not in graph.out[o]:
      continue
    angle = wrap(arm_out_heading(graph, jn.node, o) - jn.h_in)
    if abs(abs(math.degrees(angle)) - 90.) <= TURN_TOLERANCE_DEG and (angle > 0) == (side == 'left'):
      return o
  raise ValueError(f"no arm on the {side} at {jn.node}")


def across_arm(graph: RoadGraph, j: str, arm: str) -> str | None:
  """The arm straight across junction j from arm (where a car coming from arm goes on to)."""
  h = wrap(arm_out_heading(graph, j, arm) + math.pi)
  out = [o for o in graph.adj[j] if o != arm and o in graph.out[j] and abs(math.degrees(wrap(arm_out_heading(graph, j, o) - h))) < STRAIGHT_TOLERANCE_DEG]
  return out[0] if out else None


def cross_path(graph: RoadGraph, jn: Junction, side: str, start, through: bool, stops) -> tuple[np.ndarray, np.ndarray | None, float]:
  """A cross car's lane (its rightmost) toward jn's junction from its start (its front at its stop line, or start m back
  from that), ending with its front at its line; and, going through, on across the junction AFTER_M (else None); and how
  far before the junction its line is."""
  j = jn.node
  arm = side_arm(graph, jn, side)
  line = arm_line_m(graph, j, arm, stops)
  if line is None:
    raise ValueError(f"the {side} arm at {j} has no stop line")
  back = 0. if start == 'line' else float(start)
  far = walk_straight(graph, j, arm, line + back + CAR_HALF_M + 10.)
  lanes = graph.lanes_toward(j, arm)
  toward = offset_polyline(far[::-1], lane_offset(lanes, graph.radius[arm], 'right'))
  total = float(arc_lengths(toward)[-1])
  at_line = total - line - CAR_HALF_M
  come = section(toward, max(at_line - max(back, 1.), 0.), at_line)  # standing at its line: a 1 m stub to it
  if not through:
    return come, None, line
  opp = across_arm(graph, j, arm)
  if opp is None:
    raise ValueError(f"nothing straight across {j} from the {side} arm")
  leave = offset_polyline(cut(walk_straight(graph, j, opp, AFTER_M + 5.), AFTER_M), lane_offset(flip(graph.lanes_toward(j, opp)), graph.radius[opp], 'right'))
  return come, np.vstack([come[-1:], toward[-1:], leave]), line


def cross_actors(graph: RoadGraph, jn: Junction, e: Entry) -> list:
  """The stop-sign logic gate's cross cars (Entry.cross; STOP_SIGNS.md 8.2), trigger 'cross' (run.cross_due): each standing at
  its start; one starting back drives up to its line on its arrive (path, stopping at its end); from its line it goes on
  across on its release (path2 after an arrive, else path)."""
  stops = [c for c in mapped_controls(graph) if c.kind == 'stop']
  out = []
  for k, (side, start, arrive, release, through) in enumerate(e.cross):
    if (start == 'line') != (arrive is None):
      raise ValueError(f"cross car {k}: a car standing at its line has no arrive, one starting back needs one")
    come, across, line = cross_path(graph, jn, side, start, through, stops)
    params = {'side': side, 'start': start, 'arrive': arrive, 'release': release, 'through': bool(through), 'line_m': round(float(line), 1)}
    if start == 'line':
      path = across if across is not None else come
    else:
      path = come
      if across is not None:
        params['path2'] = across.tolist()
    out.append(Actor(f"{PREFIX}cross{k}", 'car', 'cross', path.tolist(), CROSS_SPEED, CROSS_ACCEL, heading_of(path[0], path[1]), params=params))
  return out


# *** builders ***
def _turn_plan(name: str, e: Entry, graph: RoadGraph) -> Plan:
  from_heading = None if e.from_heading_deg is None else math.radians(e.from_heading_deg)
  if e.from_stop_arm:
    from_heading = stop_arm_heading(graph, e.junction, [c for c in mapped_controls(graph) if c.kind == 'stop'])
  jn = find_turn(graph, e.junction, e.turn, from_heading=from_heading)
  lanes, radius = graph.lanes_toward(jn.node, jn.a), graph.radius[jn.a]
  toward = jn.approach[::-1]  # in the car's direction of travel, ending at the junction
  total = float(arc_lengths(toward)[-1])
  along = controls_along(toward, mapped_controls(graph))
  # the spawn approach_m out, else the nearest clear of other junctions and of the mapped controls: toward the junction
  # first (as far as MIN_APPROACH_M), then further out
  tries = list(np.arange(e.approach_m, MIN_APPROACH_M - 1e-6, -10.)) + list(np.arange(e.approach_m + 10., total - 5., 10.))
  for s in tries:
    spawn, heading = lane_pose(jn.approach, s, lanes, radius, ego_lane(e.turn))
    if not any(np.hypot(*(graph.pos[j][:2] - spawn[:2])) < CLEAR_OF_JUNCTION_M for j in graph.junctions()) and \
       not spawn_conflicts(total - s, along, e.speed_kph):
      break
  else:
    raise ValueError(f"{name}: no spawn on the approach to {jn.node} clear of the junctions and the mapped controls " +
                     f"({[(round(float(total - s), 1), str(c)) for s, c in along]} m before it)")
  s = float(s)
  actors, notes = [], {'junction_node': jn.node, 'approach_m': s,
                       'controls_from_spawn': [(round(float(sc - (total - s)), 1), str(c)) for sc, c in along]}
  own = junction_control(along, total)
  notes['junction_control'], notes['stop_line_m'] = own if own is not None else ('none', None)
  if e.kind == 'straight':
    if jn.straight is None:
      dest = None
    else:
      p, _ = point_at(walk_straight(graph, jn.node, jn.straight, e.dest_after_m + 5.), e.dest_after_m)
      dest = tuple(float(v) for v in p)
  else:
    p, _ = point_at(jn.exit, min(e.dest_after_m, float(arc_lengths(jn.exit)[-1]) - 5.))
    dest = tuple(float(v) for v in p)

  if e.actor == 'crosswalk':
    path, c_dist = crosswalk_path(graph, jn, e.crosswalk_m)
    h = heading_of(path[0], path[1])
    # sets off on the car's go at the junction (timing.GoWatch): 0.9 s to the crosswalk zone, then 15 s in it, while the
    # car, standing 11-27 m out, takes 2 s or more to its hold line
    actors.append(Actor(f"{PREFIX}ped", 'walker', 'walk', path.tolist(), WALK_SPEED, heading=h))
    notes['crosswalk_m'] = round(c_dist, 1)
    notes['crosswalk_unverified'] = "crosswalk placed from road widths, not from the level's crossing decals"
  elif e.actor == 'standing':
    p, _ = lane_pose(jn.approach, e.ped_before_m, lanes, radius, ego_lane(e.turn))
    actors.append(Actor(f"{PREFIX}ped", 'walker', 'setup', [list(p)], 0., heading=wrap(heading + math.pi)))
  elif e.actor == 'oncoming' and e.stream_headway_s:
    # a queue across the junction, each released on its own timer from the car's stand: at its headway (Entry)
    paths = oncoming_paths(graph, jn, e.oncoming, e.oncoming_start_m, QUEUE_GAP_M)
    if len(paths) < e.oncoming:
      raise ValueError(f"{name}: only {len(paths)} of {e.oncoming} queued cars fit on the arm across {jn.node}")
    for k, path in enumerate(paths):
      s_k = e.oncoming_start_m + k * QUEUE_GAP_M
      delay = e.stream_first_s + k * e.stream_headway_s - travel_time(s_k, e.oncoming_speed, e.oncoming_accel)
      if delay < 0.:
        raise ValueError(f"{name}: car {k} {s_k:.0f} m out can't reach the junction {e.stream_first_s + k * e.stream_headway_s:.1f} s " +
                         "after the stand: start the stream later")
      actors.append(Actor(f"{PREFIX}car{k}", 'car', 'oncoming', path.tolist(), e.oncoming_speed, e.oncoming_accel, heading_of(path[0], path[1]),
                          params={'start_m': s_k, 'release_after_stop_s': round(delay, 2), 'timed': True,
                                  'headway_s': e.stream_headway_s}))
    notes['stream'] = {'headway_s': e.stream_headway_s, 'first_s': e.stream_first_s, 'cars': len(paths)}
  elif e.actor == 'oncoming':
    # all released together, a fixed time after the car stands at its hold at the junction (timing.oncoming_release_delay)
    delay = oncoming_release_delay(e.oncoming_start_m, e.oncoming_speed, e.oncoming_accel, e.arrive_after_s)
    for k, path in enumerate(oncoming_paths(graph, jn, e.oncoming, e.oncoming_start_m, e.oncoming_spacing_m)):
      h = heading_of(path[0], path[1])
      actors.append(Actor(f"{PREFIX}car{k}", 'car', 'oncoming', path.tolist(), e.oncoming_speed, e.oncoming_accel, h,
                          params={'start_m': e.oncoming_start_m, 'arrive_after_s': e.arrive_after_s, 'release_after_stop_s': round(delay, 2)}))
  if name.startswith('stop_'):
    stops = [c for c in mapped_controls(graph) if c.kind == 'stop']
    notes['stop_scenario'], notes['all_way'] = name, all_way(graph, jn.node, stops)
    actors += cross_actors(graph, jn, e)
  return Plan(name, spawn, heading, dest, (float(jn.center[0]), float(jn.center[1])), e.speed_kph, e.timeout_s,
              e.end_on_stop_s, actors, notes)


def load_data(path: str = DATA_JSON, entry: str = '') -> dict:
  with open(path) as f:
    items = json.load(f)
  if entry:
    found = [d for d in items if d['name'] == entry]
    if not found:
      raise ValueError(f"no entry {entry!r} in {path}: {[d['name'] for d in items]}")
    return found[0]
  return items[0]


def _data_plan(name: str, e: Entry, graph: RoadGraph, data: dict) -> Plan:
  spawn = tuple(float(v) for v in data['spawn'])
  heading = math.radians(float(data['heading_deg']))
  dest = tuple(float(v) for v in data['dest']) if data.get('dest') else None
  junction = tuple(float(v) for v in data['junction']) if data.get('junction') else None
  speed = float(data.get('speed_kph', e.speed_kph))
  actors, notes = [], {'data_entry': data['name']}
  if junction is not None:
    toward, s_spawn, _ = approach_arm(graph, spawn, heading, junction)
    total = float(arc_lengths(toward)[-1])
    along = controls_along(toward, mapped_controls(graph))
    bad = spawn_conflicts(s_spawn, along, speed)
    if bad:
      raise ValueError(f"{name} entry {data['name']}: spawns {[(round(float(d), 1), str(c)) for d, c in bad]} m from mapped " +
                       f"controls, inside {spawn_clear_m(speed):.0f} m (the roll and a map stop's engage distance): move it")
    notes['controls_from_spawn'] = [(round(float(s - s_spawn), 1), str(c)) for s, c in along]
    notes['controls_in_window'] = window_controls(along, total, speed / 3.6)
    own = junction_control(along, total)
    notes['junction_control'], notes['stop_line_m'] = own if own is not None else ('none', None)
  if e.actor == 'pacer':
    if junction is None:
      raise ValueError("occupied_lane needs a junction in its data entry")
    direction = data.get('turn', 'left')
    lane, s_junction = pacer_lane(graph, spawn, heading, junction, direction)
    v = speed / 3.6
    window_start = lane_change_window(v)[0]
    h = heading_of(lane[0], lane[1])
    actors.append(Actor(f"{PREFIX}pacer", 'car', 'pace', lane.tolist(), v, 3.0, h,
                        params={'s_junction': float(s_junction), 'window_start_m': window_start, 'block_s': e.pace_block_s,
                                'drop_to': 0.4}))
  return Plan(name, spawn, heading, dest, junction, speed, e.timeout_s, e.end_on_stop_s, actors, notes)


def road_model(graph: RoadGraph, points, margin: float = ROAD_MARGIN_M) -> dict:
  """The roads in the box around points, grown by margin, for score.oncoming_offsets: each link as
  [ax, ay, bx, by, ra, rb, lanes] (lanes for travel a -> b, left to right, as RoadGraph.lanes_toward reads them), and each
  junction as [x, y, box, line]: within box m of its node a turn may cross the road's divider (the hold line, plus the
  widest road at it), and heading into it, within its farthest mapped stop line, line m out (stop_lines; 0 without one)."""
  pts = np.array([p[:2] for p in points if p is not None], dtype=float)
  lo, hi = pts.min(axis=0) - margin, pts.max(axis=0) + margin
  def inside(n):
    return bool((graph.pos[n][:2] >= lo).all() and (graph.pos[n][:2] <= hi).all())
  links, seen = [], set()
  for link in graph.links.values():
    a, b = link['inNode'], link['outNode']
    if (a, b) in seen or not (inside(a) or inside(b)):
      continue
    seen.add((a, b))
    links.append([*(round(float(v), 2) for v in graph.pos[a][:2]), *(round(float(v), 2) for v in graph.pos[b][:2]),
                  round(graph.radius[a], 2), round(graph.radius[b], 2), graph.lanes_toward(b, a)])
  lines = stop_lines(graph, mapped_controls(graph))
  junctions = [[round(float(graph.pos[j][0]), 2), round(float(graph.pos[j][1]), 2),
                round(HOLD_LINE_M + max(graph.radius[n] for n in (j, *graph.adj[j])), 2), round(lines.get(j, 0.), 2)]
               for j in graph.junctions() if inside(j)]
  return {'links': links, 'junctions': junctions}


def stop_lines(graph: RoadGraph, controls) -> dict[str, float]:
  """m from each junction node to its farthest mapped stop line: a control within LINE_ASSOC_M of the junction nearest it,
  for traffic heading into it (within INTO_DEG of the bearing from the line to the node)."""
  junctions = graph.junctions()
  xy = np.array([graph.pos[j][:2] for j in junctions])
  out: dict[str, float] = {}
  for c in controls:
    d = np.hypot(xy[:, 0] - c.x, xy[:, 1] - c.y)
    k = int(np.argmin(d))
    if d[k] > LINE_ASSOC_M:
      continue
    bearing = math.atan2(xy[k, 1] - c.y, xy[k, 0] - c.x)
    if d[k] > 1. and abs(math.degrees(wrap(bearing - c.heading))) > INTO_DEG:
      continue
    out[junctions[k]] = max(out.get(junctions[k], 0.), float(d[k]))
  return out


def build(name: str, graph: RoadGraph | None = None, data_path: str = DATA_JSON, entry: str = '') -> Plan:
  e = SCENARIOS[name]
  graph = RoadGraph() if graph is None else graph
  if e.kind == 'route':
    plan = Plan(name, e.spawn, math.radians(e.heading_deg), e.dest, None, e.speed_kph, e.timeout_s, e.end_on_stop_s, [], {})
  elif e.kind == 'data':
    plan = _data_plan(name, e, graph, load_data(data_path, entry))
  else:
    plan = _turn_plan(name, e, graph)
  plan.road = road_model(graph, [plan.spawn, plan.junction, plan.dest])
  return plan


def junction_distance(plan: Plan, x: float, y: float) -> float | None:
  """m from (x, y) to the plan's junction (straight line), or None without one."""
  if plan.junction is None:
    return None
  return math.hypot(plan.junction[0] - x, plan.junction[1] - y)
