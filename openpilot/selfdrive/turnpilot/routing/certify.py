"""The route-time certifier (STOP_SIGNS.md 3.2): whether the car may take a stop or yield control by itself, or the route
must go around it. A pure function over stop_rules.py, which turnpilotd's decider also uses: navd converts its Control
(stop_rules.junction_from_control, certify_control here) and turnpilotd its capnp reader (junction_from_capnp), so the
route-time and drive-time verdicts are the same function on the same files (maps/visibility.json, maps/surveyed_junctions.json).

certify(junction, maneuver, visibility, surveys, eta, runtime) returns (certification, reason): ('certifiable', 'none'),
('uncertifiable', reason) at the first of these checks that fails, in order:
   1 noTable       no visibility file, or its camera fields, lens, mask, darkThreshold or veil are empty (before step S1)
   2 modeOff       TurnPilotStopSignGo (runtime.mode) off or shadow, or a value this build doesn't know
   3 noLaunch      no launchVerified stamp (step S4a), unless the mode is trial
   4 geometry      roundabout, a turn_channel approach, a rail crossing within RAIL_M or on our approach (or not known), a
                   divided cross road, a U-turn
   5 skew          a conflicting left or right approach outside 75-105 deg
   6 unknownLeg    a conflicting leg (the exit leg included) whose control is unknown, the junction failed, or no conflicting
                   approach found at all
   7 notSurveyed   the junction's survey doesn't count, or a conflicting approach isn't surveyed as stopping (decision D10)
   8 override      an approach in TurnPilotJunctionOverrides (a downgrade or a signal seen)
   9 light         night, dusk, low sun along a conflicting approach, or the sun ahead with the veil present or unmeasured, at
                   any time from ETA - ETA_EARLY_S to ETA + ETA_LATE_S (runtime.window; at the line (0, 0)); or no ETA
                   (the clock isn't valid yet)
  10 notValidated  no passing validation stamp for our approach for every lighting row the window needs, or one whose
                   maxDropoutS exceeds CLEAR_S / 3 on a binding lane
  11 reach         a conflicting lane's reach short of its requirement: the stamp's minFirstTrackM bounding the distance along
                   it, capped by the frame edge and the mask, the survey's sight distance and the parking on its binding
                   curb, at the junction's stop point; with the side views off, the road and full-pass tables instead
('unchecked', 'none') for a signal (not this certifier's).

A validation stamp (written by junction_validate.py, SG3b) is {lighting, date, passes, approachBearingDeg (optional: our
approach's heading toward the junction it was measured from; absent, any), cars: {left, right, opposite}, minFirstTrackM and
maxDropoutS: {left, right, opposite} each a list per binding lane, nearest first (or one number for all), allStopped,
verdict: pass}. Lanes are named by where their traffic comes from, relative to our approach.

The binding curb of a lane (1.4 rule 6, nearestParkingM {left, right} along each curb of the approach, left and right as a
car on it toward the junction sees them): the left approach's right curb (the near side, between us and its lanes) and the
right approach's left curb; parking on the far curbs hides neither lane.
"""
from dataclasses import dataclass, field

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.maps import surveys as survey_file
from openpilot.selfdrive.turnpilot.routing.junctions import LEG_HEADING_TOL, survey_approach

MODES = ('off', 'shadow', 'trial', 'auto')
MODE_PARAM = 'TurnPilotStopSignGo'
ACTING = ('trial', 'auto')
BINDING_CURB = {'left': 'right', 'right': 'left'}


@dataclass
class Runtime:
  """What the certifier reads besides the map and the files: turnpilotd's mode and side views (turnpilotPlan; navd takes
  the mode from the param and the side views as off until SG4a publishes them), launchVerified (None: the visibility
  file's), the ETA window (the line: (0, 0)), the planning launch, the yaw into a turn (the union rule) and the yaw offset."""
  mode: str = 'shadow'
  side_views: bool = False
  launch_verified: bool | None = None
  window: tuple = (sr.ETA_EARLY_S, sr.ETA_LATE_S)
  a_launch: float = sr.A_LAUNCH_PLAN
  yaw_deg: float = 0.
  yaw_offset_deg: float = sr.ROUTE_YAW_DEG


@dataclass
class LaneReach:
  role: str
  index: int
  movement: str
  need: float      # m
  reach: float     # m from its conflict point
  stopping: bool


@dataclass
class Verdict:
  certification: str
  reason: str
  detail: str = ''
  lanes: list[LaneReach] = field(default_factory=list)
  rows: set = field(default_factory=set)   # the lighting rows the ETA window needs
  validated: bool = False


def _fail(reason: str, detail: str, **kw) -> Verdict:
  return Verdict('uncertifiable', reason, detail, **kw)


def _entries(surveys) -> list[dict]:
  return list(getattr(surveys, 'entries', surveys) or [])


def cross_lanes(j: sr.Junction) -> int | None:
  """The cross road's lanes: its left or right leg with the most, in and out; None when none is known."""
  known = [a.lanes_in * a.inbound + a.lanes_out * a.outbound for a in j.approaches if sr.effective_role(a) in ('left', 'right')]
  known = [n for n in known if n > 0]
  return max(known) if known else None


def _per_lane(value, role: str, index: int):
  side = (value or {}).get(role) if isinstance(value, dict) else None
  if isinstance(side, (int, float)) and not isinstance(side, bool):
    return float(side)
  if isinstance(side, list) and 0 < index <= len(side) and isinstance(side[index - 1], (int, float)):
    return float(side[index - 1])
  return None


def pick_stamp(entry: dict, row: str, heading_in: float) -> dict | None:
  """The newest passing stamp for a lighting row, measured from our approach (or any)."""
  fits = [v for v in entry.get('validations') or [] if isinstance(v, dict) and v.get('lighting') == row and v.get('verdict') == 'pass' and
          v.get('allStopped') is True and (v.get('approachBearingDeg') is None or
                                          sr.angle_diff(float(v['approachBearingDeg']), heading_in) <= LEG_HEADING_TOL)]
  return max(fits, key=lambda v: str(v.get('date'))) if fits else None


def assess(junction: sr.Junction, maneuver: str, visibility, surveys, eta: float | None, runtime: Runtime) -> Verdict:
  """certify() with its detail: the failing check's why, each conflicting lane's need and reach, the lighting rows."""
  j = junction
  # 1 the camera
  if not visibility:
    return _fail('noTable', 'no visibility file')
  ready, missing = visibility.camera_ready()
  if not ready:
    return _fail('noTable', f"visibility.json has no {', '.join(missing)} (step S1)")
  # 2, 3 the mode and the launch
  if runtime.mode not in ACTING:
    return _fail('modeOff', f"TurnPilotStopSignGo is {runtime.mode}")
  launch = runtime.launch_verified if runtime.launch_verified is not None else visibility.launch_verified is not None
  if not launch and runtime.mode != 'trial':
    return _fail('noLaunch', 'no launchVerified stamp (step S4a)')
  # 4 geometry
  moves = sr.conflict_movements(j, maneuver, runtime.yaw_deg)
  exit_leg = next((a for a in j.approaches if a.role == 'exit'), None)
  channel = [a for a in [m.approach for m in moves] + ([exit_leg] if exit_leg is not None else []) if a.use in ('turnChannel', 'turn_channel')]
  if j.roundabout:
    return _fail('geometry', 'a roundabout')
  if maneuver == 'uTurn':
    return _fail('geometry', 'a U-turn')
  if j.divided:
    return _fail('geometry', 'a divided cross road')
  if channel:
    return _fail('geometry', 'a turn channel (slip lane)')
  if j.rail_distance != -1.:
    return _fail('geometry', 'a rail crossing: not known (no current sidecar)' if j.rail_distance < -1.5 else
                 f"a rail crossing {j.rail_distance:.0f} m away")
  # 5 skew
  for m in moves:
    if m.role in ('left', 'right') and not sr.SKEW_MIN_DEG <= abs(m.approach.rel_bearing) <= sr.SKEW_MAX_DEG:
      return _fail('skew', f"the {m.role} approach at {m.approach.rel_bearing:+.0f} deg")
  # 6 unknown legs
  if j.status != 'ok':
    return _fail('unknownLeg', f"the junction is {j.status}")
  if not moves:
    return _fail('unknownLeg', 'no conflicting approach found: nothing to judge cross traffic by')
  for m in moves:
    if m.approach.control == 'unknown':
      return _fail('unknownLeg', f"the {m.role} approach's control is unknown")
  # 7, 8 surveyed, overrides
  if not j.surveyed:
    return _fail('notSurveyed', 'no counting survey of the junction (decision D10)')
  for m in moves:
    if m.approach.tier != 'surveyed' and not m.approach.overridden:
      return _fail('notSurveyed', f"the {m.role} approach is {m.approach.tier}, not surveyed as stopping")
  if any(a.overridden for a in j.approaches):
    return _fail('override', 'an approach is in TurnPilotJunctionOverrides')
  # 9 light over the ETA window
  if eta is None:
    return _fail('light', "the clock isn't set (no GPS time yet): the sun can't be placed")
  looks = sorted({m.approach.bearing for m in moves})
  rows, why = sr.light_window(j.lat, j.lon, eta, j.heading_in, visibility.veil, looks, *runtime.window)
  if why:
    return _fail('light', why)
  # 10 the validation stamp, per lighting row, for our approach
  entry = survey_file.match(_entries(surveys), j.osm_node_id, (j.lat, j.lon))
  if entry is None:
    return _fail('notSurveyed', 'the survey navd counted is not in the surveys given', rows=rows)
  edges = sr.edge_band(j.stop_point_distance, cross_lanes(j), (survey_approach(entry, j.our_way, j.heading_in) or {}).get('laneEdgeM'))
  lanes = sr.conflict_lanes(j, maneuver, edges, float(visibility.setback), runtime.yaw_deg)
  stamps = {}
  for row in sorted(rows):
    st = pick_stamp(entry, row, j.heading_in)
    if st is None:
      return _fail('notValidated', f"no passing validation stamp for {row} from this approach", rows=rows)
    for ln in lanes:
      first, drop = _per_lane(st.get('minFirstTrackM'), ln.role, ln.index), _per_lane(st.get('maxDropoutS'), ln.role, ln.index)
      if first is None or drop is None:
        return _fail('notValidated', f"the {row} stamp has nothing for the {ln.role} lane {ln.index}", rows=rows)
      if drop > sr.VALIDATE_DROPOUT_S + 1e-9:
        return _fail('notValidated', f"the {row} stamp's {ln.role} lane {ln.index} dropped out {drop:.2f} s (over CLEAR_S / 3)", rows=rows)
    stamps[row] = st
  # 11 reach, per lane, at the stop point
  rot = sr.angle_bound(edges.nominal is not None, runtime.yaw_offset_deg)
  out, short = [], []
  for ln in lanes:
    sv = survey_approach(entry, ln.approach.way_id, (ln.approach.bearing + 180.) % 360.) if ln.approach is not None else None
    caps = [float(sv['sightM'])] if sv is not None else [0.]
    curb = BINDING_CURB.get(ln.role)
    if sv is not None and curb is not None:
      caps.append(sr.parking_cap(ln.s_min, float(sv['nearestParkingM'][curb])))
    need = sr.reach_needed_allway(ln.limit) if ln.stopping else sr.reach_needed_two_way(ln.limit, edges.d_max, runtime.a_launch)
    reach = float('inf')
    for row, st in stamps.items():
      if runtime.side_views:
        sight = visibility.sight({})
        validated = _per_lane(st.get('minFirstTrackM'), ln.role, ln.index)
        r = sr.reach_walk(ln, sight, rot, validated_m=validated, cap_m=min(caps))
      else:
        sight = visibility.sight({k: b for k, b in visibility.views(row, use='allWay').items() if k != 'side'})
        r = sr.reach_walk(ln, sight, rot, cap_m=min(caps))
      reach = min(reach, r)
    reach = 0. if reach == float('inf') else reach
    out.append(LaneReach(ln.role, ln.index, ln.movement, need, reach, ln.stopping))
    if reach + 1e-9 < need:
      short.append(f"{ln.role} lane {ln.index} {ln.movement}: {reach:.1f} of {need:.1f} m")
  if short:
    why = '; '.join(short) + ('' if runtime.side_views else ' (side views off: the full pass only)')
    return _fail('reach', why, lanes=out, rows=rows, validated=True)
  return Verdict('certifiable', 'none', '', out, rows, True)


def stop_sign_mode(params) -> str:
  """TurnPilotStopSignGo (off, shadow, trial, auto), read the way light_state reads its param: a value this build doesn't
  know, or a param it doesn't know (UnknownKeyName, before SG4a adds it), or none at all, acts as shadow, logged."""
  try:
    value = params.get(MODE_PARAM)
  except Exception as e:
    cloudlog.warning(f"turnpilot certify: {MODE_PARAM} unreadable ({type(e).__name__}): acting as shadow")
    return 'shadow'
  if isinstance(value, bytes):
    value = value.decode(errors='replace')
  if value not in MODES:
    if value is not None:
      cloudlog.warning(f"turnpilot certify: {MODE_PARAM} is {value!r}, unknown to this build: acting as shadow")
    return 'shadow'
  return value


def certify(junction: sr.Junction, maneuver: str, visibility, surveys, eta: float, runtime: Runtime) -> tuple[str, str]:
  """(certification, reason) for a stop or yield control's junction (module docstring)."""
  v = assess(junction, maneuver, visibility, surveys, eta, runtime)
  return v.certification, v.reason


def certify_control(control, visibility, surveys, eta: float, runtime: Runtime, maneuver: str | None = None) -> tuple[str, str]:
  """certify() for navd's Control, its verdict written to its junction_info (MapJunction.certification, certReason,
  validated); a signal is unchecked."""
  if control.kind not in ('stop', 'yield'):
    return 'unchecked', 'none'
  j = sr.junction_from_control(control)
  v = assess(j, maneuver or j.maneuver, visibility, surveys, eta, runtime)
  control.certification, control.cert_reason = v.certification, v.reason
  info = getattr(control, 'junction_info', None)
  if info is not None:
    info.certification, info.cert_reason, info.validated = v.certification, v.reason, v.validated
    info.cert_detail = v.detail
  if v.certification != 'certifiable':
    cloudlog.debug(f"turnpilot certify: {control.kind} {control.id} at {control.route_m:.0f} m {v.reason}: {v.detail}")
  return v.certification, v.reason
