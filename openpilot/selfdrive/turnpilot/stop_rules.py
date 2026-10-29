"""The stop-sign rules navd's certifier and turnpilotd's decider share (STOP_SIGNS.md 1.2, 1.4-1.6, 2.1-2.3, 2.6, 2.8 and 9).

Pure Python, no cereal: navd converts its Control (junction_from_control) and turnpilotd its capnp MapControl reader
(junction_from_capnp) to the plain Junction here, so routing/certify.py's certify() is one function on both sides.

Frames and units. The car frame has the camera at the origin, x forward and y right, in m. A bearing is the angle off the
camera's axis toward the side the point is on, in degrees of calibrated (true) bearing, never signed: the side says which
way. Speeds are m/s, times s, accelerations m/s^2 (a deceleration positive). Every limit is the approach's assumed limit,
times V_CROSS_FACTOR where a requirement takes it.

Uncertain geometry (1.2) is used at its pessimistic end: a lane point's bearing at sLaneMin, rotated toward the frame edge by
the angular bound, decides the frame edge, the mask, the table's bin and a bearing-only occluder; its range at sLaneMax is
what a table bin bounds. A validated entry bounds the distance along the lane instead (validation measures that).

The reach walk (reach_walk) steps out along a lane from our conflict point and stops at the first point that fails: past the
frame edge less EDGE_MARGIN_DEG; zeroed (sun ahead, low sun, a sector that isn't ok); outside the footprint mask; past every
running, fresh view's coverage of its bearing (a view stale past DET_FRESH_S, or a bin of 0 m, covers nothing); inside the
inner band (INNER_BAND_LO_DEG to the side views' inner edge) without the full pass and that side's view both fresh, or past
the side view's first bin; past every covering view's range for its bin, or past a validated distance in place of that range
(never in place of the coverage); past a cap (survey sight distance, the parking cap); or behind an occluder. It returns the
reach from the conflict point, m.
"""
import math
from collections.abc import Callable
from dataclasses import dataclass, field

MPH = 0.44704

# requirements (STOP_SIGNS.md 2.2, 2.3, 9)
V_CROSS_FACTOR = 1.15     # the safety line: cross traffic at 1.15 x its limit
T_R = 1.5                 # s, a surprised driver's reaction (Green 2000)
A_PHANTOM = 3.0           # m/s^2, the hidden car on a non-stopping approach (decision D2)
A_AW = 3.0                # m/s^2, the hidden car on a stopping approach (D2: was AASHTO's 3.4)
A_U2 = 3.0                # m/s^2, a visible car on a stopping approach that must still stop (U2 'far')
A_DETECTED = 2.0          # m/s^2, a tracked car on a non-stopping approach: TurnPilot's own floor
M_STOP = 5.0              # m past the phantom's stop
T_LAT = 1.0               # s, detection latency (R_SEEN.md 5: two hits within 1.0 s at 5 Hz a side)
T_GO_LAT = 0.5            # s, certification to motion
A_LAUNCH_PLAN = 0.5       # m/s^2, the planning launch (the G5 nudge) until step S4a measures the model's: TurnPilotStopLaunch
A_LAUNCH_MIN, A_LAUNCH_MAX = 0.3, 2.0
STOP_LAUNCH_PARAM = 'TurnPilotStopLaunch'
T_GAP_RIGHT_STRAIGHT = 6.5  # s, AASHTO Case B
T_GAP_LEFT = 7.5
T_GAP_PER_LANE = 0.5      # s for each extra lane crossed
CLEAR_S = 2.0             # s every go condition holds (CRITIQUE REQUIRED 2)
VALIDATE_DROPOUT_S = CLEAR_S / 3.  # s, the longest detection gap a validation car may have inside its R_req
VALIDATE_CARS = 10        # cross motor vehicles per binding side a validation needs
DET_FRESH_S = 0.45        # s of camera time a view counts as fresh
VEL_TRUST_S, VEL_TRUST_SAMPLES = 1.0, 5
DECEL_TRUST_S = 1.5
TAU_DOT_MIN = -0.4        # Lee (1976) -0.5 plus margin
TAU_AGREE = 0.30          # box-expansion time to contact within 30% of the bearing TTA
U2_LINE_M = 3.0           # m short of our path a stopping car must be able to stop
SHIELD_STOP_S, SHIELD_V, SHIELD_MOVE_M = 1.0, 0.5, 0.3
SHIELD_BEFORE_M, SHIELD_PAST_M = 3.0, 2.0  # the line region: 3 m before its stop point to 2 m past it
SHIELD_KINDS = ('car', 'truck', 'bus')

# the launch and the band (2.6)
STOP_GO_V_MAX = 2.0       # m/s, TurnPilot's cap from the go to commit
A_ABORT = 2.0             # m/s^2, the capped abort
M_EDGE = 0.3              # m short of the closest plausible edge the abort ends
DT_MDL = 0.05             # s, one model step
T_ABORT_PAD_S = 0.1
TAKEOVER_MARGIN_M = 0.3
GO_NUDGE_M = 1.0          # m of nudge in GO_ROOM_M
GO_ROOM_RIGHT_M = 0.5     # m more for a right, whose swing reaches the far lane
GO_ROOM_V = 1.0           # m/s the nudge reaches

# geometry (1.2)
POS_ERR_M = 2.0           # m, the car's along-track position error
SURVEY_ERR_M = 0.5        # m, a surveyed distance's
LANE_W_MAX = 3.7          # m per cross lane for the closest plausible edge and sLaneMax
LANE_W_MIN = 3.0          # m per cross lane for the farthest plausible edge and sLaneMin
SHOULDER_M = 3.0          # m of shoulder or parking before the closest plausible edge
NODE_ERR_M = 1.0          # m, the node's own position error
FAR_EDGE_MIN_M = 0.5
NEAR_LANE_MIN_M = 1.5     # m from the closest plausible edge to the near lane's centre (sLaneMin)
NEAR_LANE_MAX_M = 1.85    # m from the farthest plausible edge (sLaneMax)
LANE_STEP_MIN_M = 3.0     # m per further lane, sLaneMin
LANE_STEP_MAX_M = 3.7     # sLaneMax
DEFAULT_CROSS_LANES = 2
MAP_ANGLE_DEG = 2.0       # deg, OSM edge headings
SURVEY_ANGLE_DEG = 1.0    # deg, a surveyed approach bearing
CAL_ANGLE_DEG = 0.5
ROUTE_YAW_DEG = 1.0       # deg of yaw offset at the line assumed at route time (section 2.8's 3.5 and 2.5 deg)
SKEW_MIN_DEG, SKEW_MAX_DEG = 75., 105.
OPPOSITE_SKEW_DEG = 15.   # deg off straight on the opposite road may be (the side roads' 90 +- 15)
RAIL_M = 30.
UNION_YAW_DEG = 15.       # deg of yaw before a turn's conflict set drops the straight set (2.1)
SWING_ZONE_M = 15.        # m to the right of our path a wide right reaches into the far lanes
OPPOSITE_LANE_Y_M = 3.7   # m left of our lane centre the opposing approach's first lane is
PARKED_FACE_M = 2.3       # m a parked car's face sits short of the lane centre (v3_tables.py)
CLASS_LIMIT_MPH = {'motorway': 55, 'trunk': 55, 'primary': 55, 'secondary': 45, 'tertiary': 35, 'unclassified': 30,
                   'residential': 30, 'service_other': 30, 'living_street': 30}  # decision D8; never Valhalla's classified speed
UNKNOWN_CLASS_MPH = 55

# what the camera sees (1.1, 1.4, 1.6)
EDGE_MARGIN_DEG = 1.5     # deg a car's bottom centre sits inside the frame edge
INNER_BAND_LO_DEG = 40.   # deg where the full pass's measurement ends
INNER_BAND_HI_DEG = 52.5  # deg, the side views' inner edge (52 deg repo, about 52.5 calibrated)
SUN_AHEAD_ZERO_DEG = 67.  # deg: with the sun ahead, reach is 0 from here to the frame edge on both sides
FOOTPRINT_H_M, FOOTPRINT_W_M = 1.5, 1.8
WALK_STEP_M = 0.25
WALK_MAX_M = 300.

# lighting (1.6, 3.2)
SUN_AHEAD_DEG = 90.       # deg of our heading the sun is "ahead" within
NIGHT_SUN_EL = -6.        # deg: night below, dusk from here to 0
SUN_EL_MAX = 15.          # deg: low sun under this...
SUN_AZ_TOL = 25.          # ...within this of a conflicting approach's look bearing
ETA_EARLY_S, ETA_LATE_S = 300., 1200.  # s, the window the light check passes over
ETA_STEP_S = 60.
VEIL_OK = ('none', 'withCover')

# route-time avoidance (3.3)
MAX_AVOID_ITER = 8
MAX_EXCLUDE_LOCATIONS = 200
EXCLUDE_HEADING_TOL = 30


def class_limit_mph(road_class: str | None, use: str | None = None) -> int:
  """mph, decision D8's default for an untagged road of road_class (Valhalla's names)."""
  if use in ('living_street', 'livingStreet'):
    return 30
  return CLASS_LIMIT_MPH.get(road_class or '', UNKNOWN_CLASS_MPH)


def a_launch_plan(params) -> float:
  """A_LAUNCH_PLAN from TurnPilotStopLaunch (m/s^2), clamped to 0.3-2.0; 0.5 unset, unreadable or unknown to the build."""
  try:
    value = params.get(STOP_LAUNCH_PARAM)
  except Exception:  # UnknownKeyName: a build whose params don't know it yet
    return A_LAUNCH_PLAN
  try:
    v = float(value)
  except (TypeError, ValueError):
    return A_LAUNCH_PLAN
  return min(max(v, A_LAUNCH_MIN), A_LAUNCH_MAX) if math.isfinite(v) else A_LAUNCH_PLAN


# ---------------------------------------------------------------------------------------------------------- requirements

def r_req_allway(v_limit: float) -> float:
  """m (2.3): a phantom at v_c on a stopping approach stops short of our path at A_AW."""
  vc = V_CROSS_FACTOR * v_limit
  return vc * vc / (2. * A_AW) + M_STOP


def reach_needed_allway(v_limit: float) -> float:
  """m of seen lane a stopping approach's lane needs: R_req_AW plus v_c x T_LAT."""
  return r_req_allway(v_limit) + V_CROSS_FACTOR * v_limit * T_LAT


def t_cue(d_edge_max: float, a_launch: float = A_LAUNCH_PLAN) -> float:
  """s from rest until our bumper reaches the farthest plausible edge (2.2)."""
  return T_GO_LAT + math.sqrt(2. * max(d_edge_max, 0.) / a_launch)


def r_req_two_way(v_limit: float, d_edge_max: float, a_launch: float = A_LAUNCH_PLAN) -> float:
  """m (2.2): a phantom at v_c on a non-stopping approach, braking T_R after our cue at A_PHANTOM, stops short of our path."""
  vc = V_CROSS_FACTOR * v_limit
  return vc * (t_cue(d_edge_max, a_launch) + T_R) + vc * vc / (2. * A_PHANTOM) + M_STOP


def reach_needed_two_way(v_limit: float, d_edge_max: float, a_launch: float = A_LAUNCH_PLAN) -> float:
  return r_req_two_way(v_limit, d_edge_max, a_launch) + V_CROSS_FACTOR * v_limit * T_LAT


def allway_limit_mph(reach_m: float) -> int | None:
  """The highest of 10-35 mph (5 mph steps) whose all-way requirement reach_m meets, or None."""
  best = None
  for mph in (10, 15, 20, 25, 30, 35):
    if reach_m + 1e-9 >= reach_needed_allway(mph * MPH):
      best = mph
  return best


def t_gap(maneuver: str, extra_lanes: int = 0) -> float:
  """s, the gap a tracked car on a non-stopping approach must leave to our conflict point (AASHTO Case B)."""
  return (T_GAP_LEFT if maneuver == 'left' else T_GAP_RIGHT_STRAIGHT) + T_GAP_PER_LANE * max(extra_lanes, 0)


def tracked_r_req(v: float, cue_s: float) -> float:
  """m a tracked car at v needs to stop short of our path, braking T_R after our cue at A_DETECTED."""
  return v * (cue_s + T_R) + v * v / (2. * A_DETECTED) + M_STOP


# -------------------------------------------------------------------------------------------------------------- geometry

@dataclass
class Edges:
  """The near edge of the first conflicting lane, from our stop point (m along our path; 1.2)."""
  d_min: float             # closest plausible (dEdgeMin)
  d_max: float             # farthest plausible (dEdgeMax)
  nominal: float | None = None  # the surveyed edge, None unsurveyed


def edge_band(stop_point_m: float, lanes_cross: int | None, lane_edge_m: float | None = None) -> Edges:
  """The edge band from a stop point stop_point_m before the node: from a survey's laneEdgeM (the near travel-lane edge from
  the node) when given, else from the cross road's lanes (DEFAULT_CROSS_LANES when not known)."""
  if lane_edge_m is not None:
    nominal = stop_point_m - lane_edge_m
    return Edges(nominal - SURVEY_ERR_M - POS_ERR_M, nominal + SURVEY_ERR_M + POS_ERR_M, nominal)
  n = lanes_cross if lanes_cross else DEFAULT_CROSS_LANES
  d_min = stop_point_m - (n * LANE_W_MAX / 2. + SHOULDER_M + NODE_ERR_M) - POS_ERR_M
  d_max = stop_point_m - max(n * LANE_W_MIN / 2. - NODE_ERR_M, FAR_EDGE_MIN_M) + POS_ERR_M
  return Edges(d_min, d_max)


def lane_offsets(edges: Edges, setback: float, index: int) -> tuple[float, float]:
  """(sLaneMin, sLaneMax): camera to the centre of the cross road's lane index (1 nearest), at both ends (1.2)."""
  k = max(index, 1) - 1
  return (edges.d_min + setback + NEAR_LANE_MIN_M + LANE_STEP_MIN_M * k,
          edges.d_max + setback + NEAR_LANE_MAX_M + LANE_STEP_MAX_M * k)


def angle_bound(surveyed: bool, yaw_offset_deg: float = ROUTE_YAW_DEG) -> float:
  """deg every lane point is rotated toward the frame edge (1.2): the yaw offset, the map angle allowance, calibration."""
  return abs(yaw_offset_deg) + (SURVEY_ANGLE_DEG if surveyed else MAP_ANGLE_DEG) + CAL_ANGLE_DEG


def parking_cap(s_min: float, parking_m: float) -> float:
  """m along a lane of the first legal parking space parking_m down its binding curb, as a bearing-only occluder at
  sLaneMin (a parked car assumed there, its face PARKED_FACE_M short of the lane centre; 2.8, v3_tables.py)."""
  if parking_m <= 0.:
    return 0.
  s_true = s_min + POS_ERR_M + SURVEY_ERR_M
  return s_min * parking_m / max(s_true - PARKED_FACE_M, 0.1)


# ---------------------------------------------------------------------------------------------------------- the junction

@dataclass
class Approach:
  """A road at the junction, as the certifier reads it (log.capnp JunctionApproach)."""
  bearing: float = 0.          # deg, the leg's heading leaving the node
  rel_bearing: float = 0.      # deg from our heading in (+ right)
  role: str = 'other'          # left, right, opposite, exit, other
  road_class: str = 'unknown'
  use: str = 'unknown'
  inbound: bool = False
  outbound: bool = False
  lanes_in: int = 0
  lanes_out: int = 0
  control: str = 'unknown'     # Valhalla's on its inbound edge: none, stop, yield, signal, unknown
  tier: str = 'unknown'        # unknown, no, mapWeak, mapStrong, surveyed
  assumed_limit: float = 0.    # m/s, before V_CROSS_FACTOR
  way_id: int = 0
  bike_lane: bool = False
  shoulder_m: float = -1.
  overridden: bool = False     # TurnPilotJunctionOverrides names it (navd's; the bus doesn't carry it)


@dataclass
class Junction:
  """A stop or yield control's junction (log.capnp MapJunction), plus the control's kind and our way."""
  kind: str = 'stop'
  status: str = 'ok'
  osm_node_id: int = 0
  lat: float = 0.
  lon: float = 0.
  stop_point_distance: float = 0.  # m from the stop point to the node
  stop_point_source: str = 'default'
  heading_in: float = 0.
  maneuver: str = 'straight'
  turn_angle: float = 0.
  roundabout: bool = False
  divided: bool = False
  rail_distance: float = -2.   # m; -1 none, -2 unknown
  sidecar: bool = False
  suspected: bool = False
  surveyed: bool = False
  our_way: int = 0
  approaches: list[Approach] = field(default_factory=list)


def junction_from_control(control) -> Junction:
  """navd's Control (routing/controls.py) and its junction_info (routing/junctions.py). A control without one is failed."""
  info = getattr(control, 'junction_info', None)
  if info is None:
    return Junction(kind=getattr(control, 'kind', 'stop'), status='failed')
  return Junction(kind=control.kind, status=info.status, osm_node_id=int(info.osm_node_id), lat=float(info.lat), lon=float(info.lon),
                  stop_point_distance=float(info.stop_point_distance), stop_point_source=info.stop_point_source,
                  heading_in=float(info.heading_in), maneuver=info.maneuver, turn_angle=float(info.turn_angle), roundabout=bool(info.roundabout),
                  divided=bool(info.divided), rail_distance=float(info.rail_distance), sidecar=bool(info.sidecar), suspected=bool(info.suspected),
                  surveyed=bool(info.surveyed), our_way=int(getattr(info, 'our_way', 0) or 0),
                  approaches=[Approach(float(a.bearing), float(a.rel_bearing), a.role, a.road_class, a.use, bool(a.inbound), bool(a.outbound),
                                       int(a.lanes_in), int(a.lanes_out), a.valhalla_control, a.tier, float(a.assumed_limit), int(a.way_id),
                                       bool(a.bike_lane), float(a.shoulder_m), bool(getattr(a, 'overridden', False)))
                              for a in info.approaches])


def junction_from_capnp(control) -> Junction:
  """A NavInstruction.MapControl reader (navInstruction.controls or offRouteControls), read by attribute so this module
  needs no cereal. A control whose junctionInfo status is none (a signal, an old navd) is failed."""
  j = control.junctionInfo
  status = str(j.status)
  return Junction(kind=str(control.kind), status='failed' if status == 'none' else status, osm_node_id=int(j.osmNodeId), lat=float(j.lat),
                  lon=float(j.lon), stop_point_distance=float(j.stopPointDistance), stop_point_source=str(j.stopPointSource),
                  heading_in=float(j.headingIn), maneuver=str(j.maneuver), turn_angle=float(j.turnAngle), roundabout=bool(j.roundabout),
                  divided=bool(j.divided), rail_distance=float(j.railDistance), sidecar=bool(j.sidecar), suspected=bool(j.suspected),
                  surveyed=bool(j.surveyed),
                  approaches=[Approach(float(a.bearing), float(a.relBearing), str(a.role), str(a.roadClass), str(a.use), bool(a.inbound),
                                       bool(a.outbound), int(a.lanesIn), int(a.lanesOut), str(a.valhallaControl), str(a.tier),
                                       float(a.assumedLimit), int(a.wayId), bool(a.bikeLane), float(a.shoulderM))
                              for a in j.approaches])


# --------------------------------------------------------------------------------------------------------- conflict sets

ROLE_OPPOSITE_DEG = 45.
ROLE_SIDE_DEG = 135.
TURN_SETS = {
  'right': (('left', 'cross'), ('right', 'swing'), ('opposite', 'leftTurners')),
  'straight': (('left', 'cross'), ('right', 'cross'), ('opposite', 'leftTurners')),
  'left': (('left', 'cross'), ('right', 'cross'), ('opposite', 'straightRight')),
}


def effective_role(a: Approach) -> str:
  """An approach's role by where its traffic comes from: the exit leg by its bearing (for a left it is the left approach,
  for a straight the opposite one)."""
  if a.role != 'exit':
    return a.role
  r = abs(a.rel_bearing)
  if r <= ROLE_OPPOSITE_DEG:
    return 'opposite'
  if r <= ROLE_SIDE_DEG:
    return 'right' if a.rel_bearing > 0 else 'left'
  return 'other'


def leg_skew(j: Junction) -> str | None:
  """Why the junction's roads aren't the shape the conflict sets and the reach model were made for (3.2 check 5), or None.
  Every road a car uses there (inbound or outbound, the exit leg by its bearing; a path no car drives is none) must be the
  left or right one within SKEW_MIN_DEG to SKEW_MAX_DEG of our heading in, or the opposite one within OPPOSITE_SKEW_DEG of
  straight on, and one road per role. A road behind the side bins (role other) is in no conflict set, so its traffic would
  never be checked; a cross road skewed into the opposite and other bins would be walked as an opposing lane parallel to
  ours; a second road in a role would be walked as the first."""
  seen: dict[str, float] = {}
  for a in j.approaches:
    if not (a.inbound or a.outbound):
      continue
    role, rel = effective_role(a), a.rel_bearing
    if role == 'other':
      return f"a road at {rel:+.0f} deg, behind the side bins: its traffic is in no conflict set"
    if role in ('left', 'right') and not SKEW_MIN_DEG <= abs(rel) <= SKEW_MAX_DEG:
      return f"the {role} approach at {rel:+.0f} deg"
    if role == 'opposite' and abs(rel) > OPPOSITE_SKEW_DEG + 1e-9:
      return f"the opposite approach {rel:+.0f} deg off straight on"
    if role in seen:
      return f"two {role} roads, at {seen[role]:+.0f} and {rel:+.0f} deg"
    seen[role] = rel
  return None


@dataclass
class Movement:
  role: str          # the approach's effective role
  movement: str      # cross, swing, leftTurners, straightRight
  approach: Approach


def maneuver_set(maneuver: str, yaw_deg: float = 0.) -> set[tuple[str, str]]:
  """(role, movement) pairs our maneuver conflicts with (2.1): a turn's set and the straight set until the car has yawed
  UNION_YAW_DEG into it; unknown (off the route) the union of all three; a U-turn none (geometry)."""
  if maneuver == 'unknown':
    return set().union(*(set(v) for v in TURN_SETS.values()))
  if maneuver not in TURN_SETS:
    return set()
  out = set(TURN_SETS[maneuver])
  if maneuver != 'straight' and abs(yaw_deg) < UNION_YAW_DEG:
    out |= set(TURN_SETS['straight'])
  return out


def conflict_movements(j: Junction, maneuver: str, yaw_deg: float = 0.) -> list[Movement]:
  """Every inbound approach movement our maneuver conflicts with; an approach cars only leave by carries none."""
  want = maneuver_set(maneuver, yaw_deg)
  out = []
  for a in j.approaches:
    role = effective_role(a)
    if not a.inbound:
      continue
    for r, mv in sorted(want):
      if r == role:
        out.append(Movement(role, mv, a))
  return out


@dataclass
class Lane:
  """One conflicting lane, as the walk sees it: a line from its origin outward along the approach (car frame)."""
  role: str
  index: int
  movement: str
  s_min: float               # camera to its centre where it crosses our path, small end (sLaneMin)
  s_max: float               # large end (sLaneMax)
  rel_bearing: float         # deg the approach leaves the node at, from our heading (+ right)
  conflict_m: float = 0.     # m along the lane from its origin to our conflict point
  limit: float = 0.          # m/s, its approach's assumed limit
  stopping: bool = False     # its approach counts as stopping (surveyed, decision D10)
  y: float = 0.              # m right of the camera its origin is (an opposing lane's offset; 0 for a cross lane)
  approach: Approach | None = None

  def point(self, d: float, which: str) -> tuple[float, float]:
    """(x, y) of the point d m from the origin, at the small (which 'min') or large ('max') end of the lane offset."""
    s = self.s_min if which == 'min' else self.s_max
    r = math.radians(self.rel_bearing)
    return s + d * math.cos(r), self.y + d * math.sin(r)


def conflict_lanes(j: Junction, maneuver: str, edges: Edges, setback: float, yaw_deg: float = 0.) -> list[Lane]:
  """The lanes of every conflicting movement, placed by 1.2: the near (left) approach's lanes first, nearest first, then
  the far (right) approach's; the right's swing zone on its far lanes from SWING_ZONE_M; the opposing approach's lanes
  across the junction, OPPOSITE_LANE_Y_M a lane to our left."""
  moves = conflict_movements(j, maneuver, yaw_deg)
  left = next((a for a in j.approaches if effective_role(a) == 'left' and a.inbound), None)
  right_any = next((a for a in j.approaches if effective_role(a) == 'right'), None)
  n_near = max(left.lanes_in, 1) if left is not None else max(right_any.lanes_out, 1) if right_any is not None else 1
  n_cross = n_near + (max(right_any.lanes_in, 1) if right_any is not None and right_any.inbound else 0)
  out = []
  for m in moves:
    a = m.approach
    n = max(a.lanes_in, 1)
    stopping = a.tier == 'surveyed'
    for i in range(1, n + 1):
      if m.role == 'left':
        s_min, s_max = lane_offsets(edges, setback, i)
        out.append(Lane('left', i, m.movement, s_min, s_max, a.rel_bearing if a.role != 'exit' else -abs(a.rel_bearing), 0., a.assumed_limit,
                        stopping, approach=a))
      elif m.role == 'right':
        s_min, s_max = lane_offsets(edges, setback, n_near + i)
        out.append(Lane('right', i, m.movement, s_min, s_max, a.rel_bearing if a.role != 'exit' else abs(a.rel_bearing),
                        SWING_ZONE_M if m.movement == 'swing' else 0., a.assumed_limit, stopping, approach=a))
      elif m.role == 'opposite':
        far_min, far_max = lane_offsets(edges, setback, n_cross + 1)
        out.append(Lane('opposite', i, m.movement, far_min - NEAR_LANE_MIN_M, far_max - NEAR_LANE_MAX_M, 0., 0., a.assumed_limit,
                        stopping, y=-OPPOSITE_LANE_Y_M * i, approach=a))
  return out


# --------------------------------------------------------------------------------------------------------- the reach walk

@dataclass
class Occluder:
  """A box that hides what lies behind it: its side, bearing span (deg) and range (m; None: bearing-only, depth unknown)."""
  side: str
  lo_deg: float
  hi_deg: float
  range_m: float | None = None


@dataclass
class Sight:
  """What the camera sees, for reach_walk. views: {view: bins} for the views running ('road', 'full', 'side'), bins
  [[bearingLoDeg, metres], ...] in calibrated bearing. zero(side, bearing): reach 0 there (sun ahead, low sun, a sector that
  isn't ok over the window). mask(side, bearing, range): a car-sized footprint there is unobstructed. fresh(view, side): the
  view ran within DET_FRESH_S (None: all fresh, route time). occluders: Occluder list (live)."""
  edge_deg: dict
  views: dict
  zero: Callable | None = None
  mask: Callable | None = None
  fresh: Callable | None = None
  occluders: list = field(default_factory=list)


def bins_at(bins, bearing: float) -> float:
  """metres of the bin bearing falls in (the last bin whose lower bound it reaches), 0 before the first."""
  out = 0.
  for lo, m in bins:
    if bearing + 1e-9 >= lo:
      out = float(m)
  return out


def _fresh(sight: Sight, view: str, side: str) -> bool:
  return sight.fresh is None or bool(sight.fresh(view, side))


def range_limit(sight: Sight, side: str, bearing: float) -> float:
  """m: how far the running, fresh views see at bearing on side (the inner band by its rule)."""
  best = 0.
  for view, bins in sight.views.items():
    if view == 'side':
      if not _fresh(sight, 'side', side):
        continue
      if INNER_BAND_LO_DEG <= bearing < INNER_BAND_HI_DEG:
        # the inner band (1.4 rule 3a): seen only while the full pass and this side's view are both fresh, and the lane passes
        # the side view's first bin just beyond the band
        if 'full' in sight.views and _fresh(sight, 'full', side):
          best = max(best, bins_at(bins, INNER_BAND_HI_DEG))
        continue
    elif not _fresh(sight, view, side):
      continue
    best = max(best, bins_at(bins, bearing))
  return best


def sun_ahead_zero(side: str, bearing: float) -> bool:
  """Sun ahead (1.6): reach 0 from SUN_AHEAD_ZERO_DEG to the frame edge on both sides."""
  return bearing >= SUN_AHEAD_ZERO_DEG


def reach_walk(lane: Lane, sight: Sight, rotation_deg: float, validated_m: float | None = None, cap_m: float | None = None,
               step: float = WALK_STEP_M, max_m: float = WALK_MAX_M) -> float:
  """m of lane seen from our conflict point outward (module docstring). validated_m bounds the distance along the lane
  from its origin (a validation stamp) in place of the views' range bound only: some running, fresh view must still cover
  each point's bearing, the inner band by its rule (1.4 rules 1 and 3a hold at a validated junction too). cap_m bounds the
  distance along the lane as well (survey sight distance, parking)."""
  d = lane.conflict_m
  while d < max_m:
    nd = d + step
    x, y = lane.point(nd, 'min')
    side = 'left' if y < 0. else 'right'
    bearing = math.degrees(math.atan2(abs(y), x)) + rotation_deg
    if bearing > sight.edge_deg.get(side, 0.) - EDGE_MARGIN_DEG:
      break
    if sight.zero is not None and sight.zero(side, bearing):
      break
    xm, ym = lane.point(nd, 'max')
    rng = math.hypot(xm, ym)
    if sight.mask is not None and not sight.mask(side, bearing, rng):
      break
    limit = range_limit(sight, side, bearing)
    if limit <= 0.:
      break  # no running, fresh view covers the bearing (the inner band by its rule), validated or not
    if validated_m is not None:
      if nd > validated_m + 1e-9:
        break
    elif rng > limit + 1e-9:
      break
    if cap_m is not None and nd > cap_m + 1e-9:
      break
    if any(o.side == side and o.lo_deg <= bearing <= o.hi_deg and (o.range_m is None or o.range_m < math.hypot(x, y))
           for o in sight.occluders):
      break
    d = nd
  return max(d - lane.conflict_m, 0.)


def footprint_ok(mask: dict | None, side: str, bearing: float, rng: float, height_m: float) -> bool:
  """The S1 mask test (1.4 rule 1): a car-sized footprint (ground to FOOTPRINT_H_M, FOOTPRINT_W_M wide) at bearing and
  range lies inside the usable elevation band of every 1 deg bearing bin it spans; an unlisted bin is masked."""
  if not mask:
    return False
  rows = {int(math.floor(b)): (lo, hi) for b, lo, hi in mask.get(side) or []}
  half = math.degrees(math.atan2(FOOTPRINT_W_M / 2., max(rng, 0.1)))
  lo_el = math.degrees(math.atan2(-height_m, rng))
  hi_el = math.degrees(math.atan2(FOOTPRINT_H_M - height_m, rng))
  for b in range(int(math.floor(bearing - half)), int(math.floor(bearing + half)) + 1):
    band = rows.get(abs(b))
    if band is None or lo_el < band[0] or hi_el > band[1]:
      return False
  return True


# ---------------------------------------------------------------------------------------------------- the launch and band

def t_abort(cp_delay: float, file_delay: float | None = None) -> float:
  """s, T_ABORT: the larger of CarParams.longitudinalActuatorDelay and visibility.json's actuatorDelayS, plus a model step
  and 0.1 s."""
  return max(float(cp_delay), float(file_delay) if file_delay is not None else 0.) + DT_MDL + T_ABORT_PAD_S


def abort_distance(v: float, t_abort_s: float, a: float = 0.) -> float:
  """m the capped abort (-A_ABORT after T_ABORT) needs from v while the car accelerates at a (m/s^2; a launch keeps going
  through the delay, so the brake starts from v + a T_ABORT): v T + a T^2 / 2 + (v + a T)^2 / (2 A_ABORT). A car already
  braking (a < 0) takes no credit for it."""
  a = max(float(a), 0.)
  v1 = v + a * t_abort_s
  return v * t_abort_s + a * t_abort_s * t_abort_s / 2. + v1 * v1 / (2. * A_ABORT)


def go_room_m(t_abort_s: float, right: bool = False, a_launch: float = A_LAUNCH_PLAN) -> float:
  """m to the closest plausible edge a go (and the creep floor) needs: the nudge, the abort from GO_ROOM_V while still
  accelerating at a_launch (the G5 nudge's A_LAUNCH_PLAN; pass the measured launch where it is larger), M_EDGE and the
  takeover margin (2.6), and GO_ROOM_RIGHT_M more for a right."""
  return GO_NUDGE_M + abort_distance(GO_ROOM_V, t_abort_s, a_launch) + M_EDGE + TAKEOVER_MARGIN_M + (GO_ROOM_RIGHT_M if right else 0.)


def gate_reached(moved_m: float, v: float, t_abort_s: float, edges: Edges, a_now: float) -> bool:
  """The gate point: the model must have taken over by here (moved_m: m moved from the stop point; a_now: the car's
  acceleration now, m/s^2, or the measured maximum launch: never left out, the abort starts from where the launch has
  carried the car after T_ABORT)."""
  return edges.d_min - moved_m <= abort_distance(v, t_abort_s, a_now) + M_EDGE + TAKEOVER_MARGIN_M


def committed(moved_m: float, v: float, t_abort_s: float, edges: Edges, a_now: float) -> bool:
  """The commit point: the capped abort can no longer stop short of the closest plausible edge (a_now as gate_reached)."""
  return edges.d_min - moved_m <= abort_distance(v, t_abort_s, a_now) + M_EDGE


def band_midpoint(edges: Edges) -> float:
  """m from the stop point the in-band abort may stop short of: the surveyed edge, else halfway through the band."""
  return edges.nominal if edges.nominal is not None else (edges.d_min + edges.d_max) / 2.


def in_lane(moved_m: float, edges: Edges) -> bool:
  return moved_m > edges.d_max


# ------------------------------------------------------------------------------------------------------------ the tracks

@dataclass
class TrackFit:
  """A track's position and speed along its lane at the pessimistic ends (1.2, 1.7)."""
  d_lo: float                 # m to our conflict point: sLaneMin, bearing rotated toward the axis, less 2 sigma
  v_hi: float                 # m/s toward it: sLaneMax, bearing rotated toward the edge, plus 2 sigma (0 or more)
  v_trusted: bool             # VEL_TRUST_S of history with VEL_TRUST_SAMPLES samples
  a_lo: float = 0.            # m/s^2 of braking, lower bound; 0 until DECEL_TRUST_S
  a_trusted: bool = False
  tau_dot: float | None = None
  expansion_agrees: bool | None = None  # box-expansion time to contact within TAU_AGREE of the bearing TTA; None no widths


def _linfit(ts, ys):
  """(intercept at the last t, slope, sigma of the last value, sigma of the slope) by least squares."""
  n = len(ts)
  tm, ym = sum(ts) / n, sum(ys) / n
  sxx = sum((t - tm) ** 2 for t in ts)
  slope = sum((t - tm) * (y - ym) for t, y in zip(ts, ys, strict=True)) / sxx if sxx > 0 else 0.
  icpt = ym - slope * tm
  res = [y - (icpt + slope * t) for t, y in zip(ts, ys, strict=True)]
  s2 = sum(r * r for r in res) / max(n - 2, 1)
  last = icpt + slope * ts[-1]
  s_last = math.sqrt(s2 * (1. / n + (ts[-1] - tm) ** 2 / sxx)) if sxx > 0 else math.sqrt(s2)
  s_slope = math.sqrt(s2 / sxx) if sxx > 0 else float('inf')
  return last, slope, s_last, s_slope


def _quadfit(ts, ys):
  """(c2, sigma of c2) of y = c0 + c1 t + c2 t^2 by least squares (normal equations)."""
  n = len(ts)
  t0 = ts[-1]
  x = [t - t0 for t in ts]
  s = [sum(xi ** k for xi in x) for k in range(5)]
  b = [sum(y * xi ** k for xi, y in zip(x, ys, strict=True)) for k in range(3)]
  m = [[s[0], s[1], s[2]], [s[1], s[2], s[3]], [s[2], s[3], s[4]]]

  def det3(a):
    return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
            a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
  dm = det3(m)
  if abs(dm) < 1e-12:
    return 0., float('inf')
  coef = []
  for k in range(3):
    mk = [row[:] for row in m]
    for r in range(3):
      mk[r][k] = b[r]
    coef.append(det3(mk) / dm)
  res = [y - (coef[0] + coef[1] * xi + coef[2] * xi * xi) for xi, y in zip(x, ys, strict=True)]
  s2 = sum(r * r for r in res) / max(n - 3, 1)
  # the variance of c2: s2 times the (2, 2) cofactor of m over its determinant
  cof = m[0][0] * m[1][1] - m[0][1] * m[1][0]
  return coef[2], math.sqrt(max(s2 * cof / dm, 0.))


def fit_track(samples, s_min: float, s_max: float, bound_deg: float, conflict_m: float = 0., pixel_rad: float = 1. / 904.) -> TrackFit:
  """A TrackFit from samples [(t camera time s, bearing deg off the axis toward the car's side, box width px or None)] of
  one track on a lane crossing our path at s_min..s_max (1.7). Distance at sLaneMin with the bearing rotated toward the
  axis by bound_deg; speed and braking at sLaneMax rotated toward the edge; sigma from the fit's residuals, the bearing
  noise floored at pixel_rad (one pixel of the view)."""
  samples = sorted(samples, key=lambda s: s[0])
  t_end = samples[-1][0]
  span = t_end - samples[0][0]

  def d_at(s, psi_deg, sign):
    psi = min(max(psi_deg + sign * bound_deg, 0.), 89.5)
    return s * math.tan(math.radians(psi))

  def floor_sigma(s, psi_deg):
    psi = math.radians(min(max(psi_deg, 0.), 89.5))
    return s * pixel_rad / math.cos(psi) ** 2

  last = samples[-1]
  d_lo_now = d_at(s_min, last[1], -1.) - conflict_m
  sig_floor = floor_sigma(s_max, last[1])
  vel = [s for s in samples if s[0] >= t_end - VEL_TRUST_S - 1e-9]
  v_trusted = span + 1e-9 >= VEL_TRUST_S and len(vel) >= VEL_TRUST_SAMPLES
  if len(vel) >= 2:
    ts = [s[0] for s in vel]
    _, slope, s_last, s_slope = _linfit(ts, [d_at(s_max, s[1], 1.) for s in vel])
    sxx = sum((t - sum(ts) / len(ts)) ** 2 for t in ts)
    s_slope = max(s_slope, sig_floor / math.sqrt(sxx) if sxx > 0 else s_slope)
    v_hi = max(-slope + 2. * s_slope, 0.)
    _, _, s_lo_last, _ = _linfit(ts, [d_at(s_min, s[1], -1.) for s in vel])
    d_lo = d_lo_now - 2. * max(s_lo_last, floor_sigma(s_min, last[1]))
  else:
    v_hi, d_lo = 0., d_lo_now - 2. * floor_sigma(s_min, last[1])
  fit = TrackFit(d_lo, v_hi, v_trusted)
  acc = [s for s in samples if s[0] >= t_end - DECEL_TRUST_S - 1e-9]
  if span + 1e-9 >= DECEL_TRUST_S and len(acc) >= 4:
    ts = [s[0] for s in acc]
    c2, s_c2 = _quadfit(ts, [d_at(s_max, s[1], 1.) for s in acc])
    s_c2 = max(s_c2, sig_floor / max(DECEL_TRUST_S ** 2, 1e-6))
    fit.a_lo, fit.a_trusted = max(2. * (c2 - 2. * s_c2), 0.), True
    # tau-dot over the window: tau = D / v at its two ends, D and v from the linear fits of each half
    half = len(acc) // 2
    taus = []
    for part in (acc[:half + 1], acc[half:]):
      if len(part) >= 2:
        p_ts = [s[0] for s in part]
        dl, sl, _, _ = _linfit(p_ts, [d_at(s_min, s[1], -1.) - conflict_m for s in part])
        taus.append((p_ts[-1], dl / max(-sl, 1e-3)))
    if len(taus) == 2 and taus[1][0] > taus[0][0]:
      fit.tau_dot = (taus[1][1] - taus[0][1]) / (taus[1][0] - taus[0][0])
  widths = [s for s in vel if len(s) > 2 and s[2]]
  if len(widths) >= VEL_TRUST_SAMPLES and len(widths) == len(vel):
    ts = [s[0] for s in widths]
    inv, inv_slope, _, _ = _linfit(ts, [1. / s[2] for s in widths])
    dl, sl, _, _ = _linfit(ts, [d_at(s_min, s[1], 0.) - conflict_m for s in widths])
    if inv_slope < 0. and sl < 0.:
      ttc, tta = inv / -inv_slope, dl / -sl
      fit.expansion_agrees = abs(ttc / max(tta, 1e-3) - 1.) <= TAU_AGREE
    else:
      fit.expansion_agrees = False
  return fit


def u2_state(fit: TrackFit, at_line: bool = False) -> str:
  """The not-stopping test (2.3) for a track on a stopping approach: 'stopped' (at its line, trusted under SHIELD_V),
  'far' (it can still stop short comfortably at A_U2), 'stopping' (proven braking enough, tau-dot and the box expansion
  agreeing), else 'notStopping' (an untrusted speed included)."""
  if not fit.v_trusted:
    return 'notStopping'
  if at_line and fit.v_hi < SHIELD_V:
    return 'stopped'
  room = fit.d_lo - U2_LINE_M
  if room > 0. and fit.v_hi ** 2 / (2. * room) <= A_U2:
    return 'far'
  if (fit.a_trusted and fit.a_lo > 0. and room > 0. and fit.v_hi ** 2 / (2. * fit.a_lo) < room and fit.tau_dot is not None and
      fit.tau_dot >= TAU_DOT_MIN and fit.expansion_agrees):
    return 'stopping'
  return 'notStopping'


def tracked_ok(fit: TrackFit, maneuver: str, cue_s: float, v_c: float, extra_lanes: int = 0) -> bool:
  """The tracked test on a non-stopping approach (2.2): from its worst-case position and speed (v_c while untrusted) it
  stops short of our path at A_DETECTED and leaves T_GAP to our conflict point."""
  v = fit.v_hi if fit.v_trusted else v_c
  if fit.d_lo < tracked_r_req(v, cue_s):
    return False
  return v <= 0. or fit.d_lo / v >= t_gap(maneuver, extra_lanes)


def shield_ok(kind: str, stopped_s: float, v_hi: float, samples: int, front_from_line_m: float, seen_every_cycle: bool,
              moved_m: float) -> bool:
  """The shield (2.3): a motor vehicle stopped SHIELD_STOP_S (trusted speed under SHIELD_V, VEL_TRUST_SAMPLES samples) with
  its front within its line region (front_from_line_m: + past its stop point), seen on every cycle of its view, not moved
  SHIELD_MOVE_M."""
  return (kind in SHIELD_KINDS and stopped_s >= SHIELD_STOP_S and v_hi < SHIELD_V and samples >= VEL_TRUST_SAMPLES and
          -SHIELD_BEFORE_M <= front_from_line_m <= SHIELD_PAST_M and seen_every_cycle and moved_m < SHIELD_MOVE_M)


# --------------------------------------------------------------------------------------------------------------- the sun

def sun_position(lat: float, lon: float, t: float) -> tuple[float, float]:
  """(elevation, azimuth) deg of the sun at (lat, lon) at Unix time t, by NOAA's solar calculator (no refraction)."""
  jd = t / 86400. + 2440587.5
  jc = (jd - 2451545.) / 36525.
  l0 = (280.46646 + jc * (36000.76983 + jc * 0.0003032)) % 360.
  m = 357.52911 + jc * (35999.05029 - 0.0001537 * jc)
  e = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc)
  mr = math.radians(m)
  c = math.sin(mr) * (1.914602 - jc * (0.004817 + 0.000014 * jc)) + math.sin(2 * mr) * (0.019993 - 0.000101 * jc) + \
    math.sin(3 * mr) * 0.000289
  app = l0 + c - 0.00569 - 0.00478 * math.sin(math.radians(125.04 - 1934.136 * jc))
  obliq = 23. + (26. + (21.448 - jc * (46.815 + jc * (0.00059 - jc * 0.001813))) / 60.) / 60.
  obliq += 0.00256 * math.cos(math.radians(125.04 - 1934.136 * jc))
  decl = math.asin(math.sin(math.radians(obliq)) * math.sin(math.radians(app)))
  y = math.tan(math.radians(obliq / 2.)) ** 2
  l0r = math.radians(l0)
  eot = 4. * math.degrees(y * math.sin(2 * l0r) - 2 * e * math.sin(mr) + 4 * e * y * math.sin(mr) * math.cos(2 * l0r) -
                          0.5 * y * y * math.sin(4 * l0r) - 1.25 * e * e * math.sin(2 * mr))
  minutes = (t % 86400.) / 60.
  tst = (minutes + eot + 4. * lon) % 1440.
  ha = tst / 4. - 180. if tst / 4. >= 0. else tst / 4. + 180.
  latr = math.radians(lat)
  cz = math.sin(latr) * math.sin(decl) + math.cos(latr) * math.cos(decl) * math.cos(math.radians(ha))
  zen = math.acos(min(max(cz, -1.), 1.))
  el = 90. - math.degrees(zen)
  den = math.cos(latr) * math.sin(zen)
  if abs(den) < 1e-9:
    return el, 180.
  ca = (math.sin(latr) * math.cos(zen) - math.sin(decl)) / den
  a = math.degrees(math.acos(min(max(ca, -1.), 1.)))
  az = (a + 180.) % 360. if ha > 0. else (540. - a) % 360.
  return el, az


def angle_diff(a: float, b: float) -> float:
  return abs((a - b + 180.) % 360. - 180.)


def lighting(lat: float, lon: float, t: float, heading: float, look_bearings=()) -> str:
  """The lighting at the junction at time t for a car heading that way (1.6): 'night' (sun under NIGHT_SUN_EL), 'dusk'
  (under 0), 'lowSun' (under SUN_EL_MAX within SUN_AZ_TOL of a conflicting approach's look bearing, whatever the veil),
  'sunAhead' (up within SUN_AHEAD_DEG of our heading), else 'day'."""
  el, az = sun_position(lat, lon, t)
  if el < NIGHT_SUN_EL:
    return 'night'
  if el < 0.:
    return 'dusk'
  if el < SUN_EL_MAX and any(angle_diff(az, b) <= SUN_AZ_TOL for b in look_bearings):
    return 'lowSun'
  if angle_diff(az, heading) < SUN_AHEAD_DEG:
    return 'sunAhead'
  return 'day'


def light_window(lat: float, lon: float, eta: float, heading: float, veil: str | None, look_bearings=(),
                 early_s: float = ETA_EARLY_S, late_s: float = ETA_LATE_S) -> tuple[set[str], str]:
  """(the lighting rows the window needs, why it fails or ''): the light check (3.2 check 9) at every ETA_STEP_S from eta
  - early_s to eta + late_s (and both ends). Night, dusk and low sun fail; sun ahead fails unless the veil is none or
  withCover, when its row is 'sunAhead'."""
  rows, n = set(), max(int(math.ceil((early_s + late_s) / ETA_STEP_S)), 1)
  for k in range(n + 1):
    t = eta - early_s + (early_s + late_s) * k / n
    row = lighting(lat, lon, t, heading, look_bearings)
    when = f"{(t - eta) / 60.:+.0f} min"
    if row in ('night', 'dusk'):
      return rows, f"{row} at ETA {when}"
    if row == 'lowSun':
      return rows, f"low sun along an approach at ETA {when}"
    if row == 'sunAhead' and veil not in VEIL_OK:
      return rows, f"sun ahead at ETA {when} (veil {veil})"
    rows.add(row)
  return rows, ''
