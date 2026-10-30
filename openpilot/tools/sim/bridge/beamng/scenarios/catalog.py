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

A forced light (Entry.light, the demo's S13, S14, S18): the junction's own signal on the approach (its stop line's
instance) is forced to that state from the run's setup and held, its opposing approach with it and the cross approaches
the other way (their mapped headings: within PHASE_DEG of ours, or of its reverse, is our phase); green_after_s after the
car stands at the junction the phases swap (run.py). With --signals alone the phase at the car's arrival is chance.
"""
import json
import math
import os
from dataclasses import dataclass, field

import numpy as np

from openpilot.selfdrive.turnpilot.control_tracker import DROP_PAST_M
from openpilot.selfdrive.turnpilot.lane_planner import LANE_WIDTH, lane_change_window
from openpilot.selfdrive.turnpilot.map_controls import MAP_STOP_DECEL, PROTECT_BEFORE_M, engage_distance
from openpilot.selfdrive.turnpilot.scene.tracker import CORRIDOR_HALF_WIDTH
from openpilot.selfdrive.turnpilot.unprotected_left import reach_at_line
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M, PED_STOP_GAP_M
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
PHASE_DEG = 45.          # a signal of the junction's group whose traffic heads within this of ours, or of its reverse: our phase
# ped_step_out (S10): she stands STEP_OUT_CURB_M off the road edge on the car's side (0: on it, 0.5 m outside the car's
# corridor in a 4 m lane) and steps into the middle of its lane when the car is timing.step_out_distance away: the stop
# then needs STEP_OUT_DECEL once she is inside the corridor, STEP_OUT_REACT_S later (a detector run, the pipeline and the
# actuator's lag), and less (2.4 m/s^2 at 30 km/h) if the policy takes her as entering at her first step; both are past
# the comfortable -2.0 floor that only a confirmed track may brake harder than, and short of the Tesla's -3.48
STEP_OUT_CURB_M = 0.
STEP_OUT_DECEL = 3.0
STEP_OUT_REACT_S = 0.5
STEP_OUT_MIN_V = 4.      # m/s: she never steps out in front of a car slower than this


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
  actor: str = ''                 # 'crosswalk', 'oncoming', 'standing', 'pacer', 'step_out'
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
  # an occluder (S08): a car on its parking brake in the opposing inner lane this far past the maneuver point, facing it,
  # where a car waiting to turn its left stands, never released (trigger 'setup'); 0: none
  occluder_past_m: float = 0.
  pace_block_s: float = 6.        # the pacer keeps the target lane occupied this long into the lane change window
  end_on_stop_s: float = 0.       # end the run once the car has stood this long after moving
  spawn: tuple = ()               # route: (x, y, z), heading_deg, destination (x, y, z)
  heading_deg: float = 0.
  dest: tuple = ()
  expect: tuple = ()              # what the run must show (score.EXPECTS): a handback, a skip, ...; the runner's options add theirs
  light: str = ''                 # 'red': the junction's own signal forced red from setup and held (module docstring)
  green_after_s: float = 0.       # ...then green this long after the car stands at the junction (0: red for the whole run)
  fault_at_s: float = 0.          # the runner's --gas-press-at and --drop-detections-at with no value: this long after engagement


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
  # ped_in_path's straight with nothing on it, to a destination 140 m past the junction, straight on: S01's detection drop
  # (run.py --drop-detections-at) and S04's gas press (--gas-press-at). Either at fault_at_s after engagement lands the
  # disengagement, a soft disable's DETECTOR_DEADLINE_S + SOFT_DISABLE_TIME later for the drop, 10 m or more short of the
  # junction's stop line (tests: test_runner_options)
  'straight_cruise': Entry('straight', junction=(-768.9, 661.4), turn='right', from_heading_deg=0., dest_after_m=140., timeout_s=60.,
                           fault_at_s=3.),
  # S08: the left at wpTown_5 (-600.7, 220.9), two lanes each way at the mapped 25 mph (R_req 141 m), from its longest
  # approach: a car standing 10 m past the junction in the opposing inner lane, where one waiting to turn its left would,
  # hides part of the oncoming lanes from the hold line and after the full creep (unprotected_left.view_blocked); a stream
  # at 10 s headways in the outer lane beside it, then an empty road. It creeps, then goes on a clear band or hands back
  'unprotected_left_occluded': Entry('turn', junction=(-600.7, 220.9), turn='left', actor='oncoming', oncoming=3, oncoming_start_m=40.,
                                     oncoming_speed=11.18, stream_headway_s=10., occluder_past_m=10., timeout_s=250.),
  # S19: the right at DR475_2 (-153.7, 148.7), no control of its own, at the end of the map's one straight of 240 m or more
  # with no mapped control on it (357 m, south-west-bound, through two uncontrolled junctions 135-210 m out, where no spawn
  # may stand), at 80 km/h from 230 m out: the rolling start engages about 54 m out at 64 km/h, already past where the turn
  # can be made braking at -2.0 (turn_skip.can_make_turn), so it is skipped, never braked hard for, and rerouted. At 65 km/h
  # from the nearest clear spawn (215 m) it would engage 92 m out, where the turn is still feasible and its cap slows the car
  'too_fast_right': Entry('turn', junction=(-153.7, 148.7), turn='right', from_heading_deg=-135., approach_m=230., speed_kph=80.,
                          timeout_s=60., expect=('skip',)),
  # S20: route_demo's route has no right on green (its right is uncontrolled) and no left without an oncoming road (its left
  # has one straight across), so: ped_crosswalk_right's right at its signal with nothing on it, and a left from the stem of
  # the T at DR295_16 (-586.7, 631.3), no control, no road straight across, two-way all round
  'right_green': Entry('turn', junction=(-768.6, 555.2), turn='right', timeout_s=120.),
  'left_t_junction': Entry('turn', junction=(-586.7, 631.3), turn='left', from_heading_deg=45., timeout_s=120.),
  # gate1's approach: eastbound on the 140 m straight from the bridge's default spawn
  'ped_in_path': Entry('straight', junction=(-768.9, 661.4), turn='right', from_heading_deg=0., actor='standing', end_on_stop_s=4.,
                       timeout_s=90.),
  # S10: on the same straight a walker standing at the road edge 45 m before the junction steps into the middle of the
  # lane when the car is step_out_distance ahead of her (STEP_OUT_*): the stop needs hard braking from a confirmed track
  'ped_step_out': Entry('straight', junction=(-768.9, 661.4), turn='right', from_heading_deg=0., actor='step_out', ped_before_m=45.,
                        end_on_stop_s=4., timeout_s=90., expect=('hardBrake',)),
  # forced reds (the module docstring's forced light). S13: the same straight through gate1's junction (group_80_81_82_83),
  # its signal red from the start: with no accepted light source (the car this week) TurnPilot adds nothing for it, the
  # model alone meets it. S14: ped_crosswalk_right's right turn without the walker, red until 8 s after the car stands.
  # S18: the right at (-650.1, 481.7) from the south-west (G0's 62R: every pull-away at a 20 m hold went straight), its
  # miss expected (likely, not certain)
  'red_straight': Entry('straight', junction=(-768.9, 661.4), turn='right', from_heading_deg=0., light='red', green_after_s=8.,
                        timeout_s=90.),
  'red_right': Entry('turn', junction=(-768.6, 555.2), turn='right', light='red', green_after_s=8., timeout_s=120.),
  'red_right_g62': Entry('turn', junction=(-650.1, 481.7), turn='right', from_heading_deg=45., light='red', green_after_s=8.,
                         timeout_s=120., expect=('turnMissed',)),
  'occupied_lane': Entry('data', actor='pacer', speed_kph=45., timeout_s=150.),
  'lane_change': Entry('data', speed_kph=45., timeout_s=150.),
  # downtown: 20 m on from the bridge's default spawn (clear of the junction behind it), eastbound, to road-graph node
  # DR249_3 about 850 m east-southeast
  'route_demo': Entry('route', spawn=(-888.82, 659.66, 111.4), heading_deg=0., dest=(-84.9, 377.0, 101.5), speed_kph=40., timeout_s=600.),
  # S23: in the right lane 90 m up DR677_5 from the stop sign at DR225_1 (-651.2, 774.2), whose other way in (DR409_3)
  # doesn't stop, to 120 m straight across it. Run with TurnPilotAvoidStops at its default (the route goes around it,
  # ROUTE_NOTES) and off (the car stops at the sign and hands back: --expect handback:stopSignTakeControl)
  'twoway_route_pair': Entry('route', spawn=(-564.58, 779.66, 75.0), heading_deg=-180., dest=(-770.2, 769.21, 87.66), speed_kph=30.,
                             timeout_s=150.),
}

# what a route entry's plan notes carry: for twoway_route_pair, its stop and route_audit's reading of the trip on the
# stop-sign sim tiles (build_sim_tiles.sh at this branch's code, the stop-sign map branch merged, 2026-10-29), routed from
# the spawn's heading as navd sends the car's bearing, certified as navd would (mode auto, side views on); twoway_pair.py
# finds it again. The fastest route (208 m, 14 s, straight on) meets the stop 68 m in, uncertifiable (a stop whose junction
# isn't a surveyed all-way never certifies); navd's avoidance loop goes around it in one reroute, clean, 4.8 s longer
# (348 m), with no stop, yield or signal on it, a keep (slight right) 54 m in and its one junction turn 230 m in, a left at
# DR560_1 with no road straight across. Neither route has an unprotected left or a junction turn inside 100 m of the start
ROUTE_NOTES = {
  'twoway_route_pair': {'twoway_stop': {
    'node': 'DR225_1', 'xy': (-651.2, 774.2), 'approach': 'DR677_5', 'instance': 'stop 41', 'spawn_before_m': 90.,
    'kinds': {'DR677_5': 'stop', 'DR409_3': 'none'},
    'route_audit': {'fastest': {'m': 208, 's': 14, 'stops': [['stop', 68, 'uncertifiable']], 'turns': []},
                    'avoided': {'m': 348, 'stops': [], 'signals_m': [],
                                'turns': [[54, 'slight right', 'DR677_1'], [230, 'left', 'DR560_1']]},
                    'avoidance': {'status': 'clean', 'avoided': 1, 'added_s': 4.8}}}},
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
  expect: tuple = ()        # Entry.expect
  fault_at_s: float = 0.    # Entry.fault_at_s
  lights: dict | None = None  # the forced light (junction_lights), or None


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


def own_control(along, total: float) -> tuple[float, MappedControl] | None:
  """(m before the junction, control) of the junction's own control on its approach: a line within PROTECT_BEFORE_M
  before it (map_controls.protected's reach), the nearest, or None."""
  own = [(total - s, c) for s, c in along if -2. < total - s <= PROTECT_BEFORE_M]
  return min(own, key=lambda dc: dc[0]) if own else None


def junction_control(along, total: float) -> tuple[str, float] | None:
  """(kind, m before the junction) of the junction's own control on its approach (own_control), or None."""
  own = own_control(along, total)
  return None if own is None else (own[1].kind, round(float(own[0]), 1))


def junction_lights(controls, own: MappedControl, state: str, green_after_s: float, signals: list[dict] | None = None) -> dict:
  """The forced light of a junction whose own signal is `own` (module docstring): its instance, the rest of its phase
  ('with', from the level's signal groups, scenario.SIGNALS_JSON, by mapped heading) and the cross phase, and the states
  they are forced to at setup."""
  from openpilot.tools.sim.bridge.beamng.scenario import load_signals
  group = next((g for g in (load_signals() if signals is None else signals) if own.instance in g['instances']), None)
  if group is None:
    raise ValueError(f"{own.instance} is in no signal group of the level")
  heading = {c.instance: c.heading for c in reversed(controls)}  # the first link each controls
  same, cross = [], []
  for inst in group['instances']:
    if inst == own.instance or inst not in heading:
      continue
    d = abs(math.degrees(wrap(heading[inst] - own.heading)))
    (same if min(d, 180. - d) <= PHASE_DEG else cross).append(inst)
  other = 'green' if state == 'red' else 'red'
  return {'instance': own.instance, 'group': group['group'], 'with': same, 'cross': cross, 'state': state,
          'green_after_s': green_after_s, 'states_at_setup': {own.instance: state, **dict.fromkeys(same, state), **dict.fromkeys(cross, other)}}


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


# *** builders ***
def _turn_plan(name: str, e: Entry, graph: RoadGraph) -> Plan:
  jn = find_turn(graph, e.junction, e.turn, from_heading=None if e.from_heading_deg is None else math.radians(e.from_heading_deg))
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
  own_c = own_control(along, total)
  notes['junction_instance'] = None if own_c is None else own_c[1].instance
  lights = None
  if e.light:
    if own_c is None or own_c[1].kind != 'signal':
      raise ValueError(f"{name}: a forced light at {jn.node}, whose approach has no signal of its own ({notes['junction_control']})")
    lights = junction_lights(mapped_controls(graph), own_c[1], e.light, e.green_after_s)
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
  elif e.actor == 'step_out':
    p, h_out = point_at(jn.approach, e.ped_before_m)  # the centreline there; h_out points away from the junction
    travel = wrap(h_out + math.pi)
    o_lane = lane_offset(lanes, radius, ego_lane(e.turn))
    start = p[:2] + (radius + STEP_OUT_CURB_M) * right_of(travel)
    end = p[:2] + o_lane * right_of(travel)
    enter_s = max(radius + STEP_OUT_CURB_M - o_lane - CORRIDOR_HALF_WIDTH, 0.) / WALK_SPEED
    z = float(p[2])
    actors.append(Actor(f"{PREFIX}ped", 'walker', 'step_out', [[*map(float, start), z], [*map(float, end), z]], WALK_SPEED,
                        heading=heading_of(start, end),
                        params={'decel': STEP_OUT_DECEL, 'gap_m': PED_STOP_GAP_M, 'enter_s': round(enter_s, 3),
                                'react_s': STEP_OUT_REACT_S, 'min_v': STEP_OUT_MIN_V}))
    notes['step_out'] = {'before_m': e.ped_before_m, 'lateral_m': round(radius + STEP_OUT_CURB_M - o_lane, 2), 'enter_s': round(enter_s, 2)}
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
  if e.occluder_past_m > 0.:
    if jn.straight is None:
      raise ValueError(f"{name}: no arm straight across {jn.node} for an occluder")
    far = walk_straight(graph, jn.node, jn.straight, e.occluder_past_m + 10.)
    pos, h_occ = lane_pose(far, e.occluder_past_m, graph.lanes_toward(jn.node, jn.straight), graph.radius[jn.straight], 'left')
    actors.append(Actor(f"{PREFIX}occluder", 'car', 'setup', [list(pos)], 0., 0., h_occ, params={'past_m': e.occluder_past_m}))
    notes['occluder'] = {'past_m': e.occluder_past_m, 'lane': 'opposing inner'}
  if e.turn == 'left' and jn.straight is not None:  # the left's R_req at its line, the opposing road's mapped limit (route time's)
    link = graph.links.get((jn.straight, jn.node)) or graph.links.get((jn.node, jn.straight)) or {}
    lanes_in = graph.lanes_toward(jn.node, jn.straight).count('+')
    notes['left_r_req_m'] = round(reach_at_line(link.get('speedLimit'), lanes_in), 1)
  notes['ego_lane_offset_m'] = round(lane_offset(lanes, radius, ego_lane(e.turn)), 2)
  return Plan(name, spawn, heading, dest, (float(jn.center[0]), float(jn.center[1])), e.speed_kph, e.timeout_s,
              e.end_on_stop_s, actors, notes, lights=lights)


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
    plan = Plan(name, e.spawn, math.radians(e.heading_deg), e.dest, None, e.speed_kph, e.timeout_s, e.end_on_stop_s, [],
                json.loads(json.dumps(ROUTE_NOTES.get(name, {}))))
  elif e.kind == 'data':
    plan = _data_plan(name, e, graph, load_data(data_path, entry))
  else:
    plan = _turn_plan(name, e, graph)
  plan.road = road_model(graph, [plan.spawn, plan.junction, plan.dest])
  plan.expect, plan.fault_at_s = tuple(e.expect), e.fault_at_s
  return plan


def junction_distance(plan: Plan, x: float, y: float) -> float | None:
  """m from (x, y) to the plan's junction (straight line), or None without one."""
  if plan.junction is None:
    return None
  return math.hypot(plan.junction[0] - x, plan.junction[1] - y)
