"""Route time's refused maneuvers: the route's U-turns, for the avoidance loop to route around (routing/avoid.py), and a log
line of its unprotected lefts, which are never routed around.

Unprotected lefts (Kellen, 2026-11-06: "drive to the line; go where the camera covers the oncoming road, else hold and hand
back loudly; never route around or skip a left for range"). Until then this file returned every left whose required clear range
was past the detector's proven range (TurnPilotLeftRangeM, the far crop's TurnPilotLeftFarRangeM) by unprotected_left.route_range,
the predicate turnpilotd skipped them by (turn_skip: unprotectedRange), and the loop excluded their exits. On the 11-06 drives
it refused every left 58-394 m out, before anything could be judged, and every reroute logged "N of the route's N unprotected
lefts past the reach" and brought another left: Chase's "kept wanting to turn left" (1e: 19 route calculations, one "out of
time after 5 reroutes"). Now no left is a Left here: turnpilotd drives each to its hold line, where unprotected_left judges it
from where the car stands (a gap the camera certifies, or a loud handback with the car held), and the route Valhalla gives is
the route driven. Each is still judged here exactly as turnpilotd would at route time, for the log line only:
  - the maneuver as turnpilotd reads it off allManeuvers (guidance.maneuver_entry, route_follower.maneuver_from_ahead, under
    the config's side of the road as turnpilotd's follower is: a slight turn to the far side across an opposing approach is a
    TURN, so where traffic keeps left a slight right): a junction turn to the far side whose junction has an opposing
    approach, or might have (guidance.maneuver_junction: present, or unknown without junction data; none at a T's stem or a
    bend);
  - its opposing road's mapped limit, lanes and straight span (routing/controls.py: opposing_roads; {} on a failure: 45 mph and
    not known straight, as turnpilotd takes what navd then sends);
  - the junction's mapped stop line's setback from the route's controls (map_controls.line_setback_at, the car at the start);
  - unprotected_left.route_range under TurnPilotAutoLeft and the reach left_reach gives (navd takes the far crop as on unless
    TurnPilotFarCrop is off: it reads no farStatus): its reach and bent say which lefts may hand back at their line ('turnpilot
    lefts:', by step, never a position).

A U-turn on the route (continue/uturn, any maneuver whose modifier is uturn) is refused, whatever TurnPilotAutoLeft says: the
driving model can't U-turn and turnpilotd sends nothing there (route_follower: UTURN), so the car drives on past it and navd
reroutes, or the model stands short of it. The confirm round's route_demo (BeamNG): a reroute asked a U-turn 100.9 m on, and the
model stood 85 s at the green 7 m short of it. A U-turn is a Left of kind 'uturn', carrying its exit edge's exclusion (the road it
turns back onto: routing/controls.py, Exit) in the fields the avoidance loop reads for a stop's approach (approach_key, exclusion,
first_edge, last_edge), under the loop's detour budget. Why the exit edge, directed, not the approach edge a stop is excluded by:
the approach edge can never be excluded while the car is on it (avoid.py: the car's own edge is never sent), and it would close
our own road into the junction to every other movement as well; a point at the node with no heading would close the whole
junction. One whose exit is the route's last edge, with none known, or over the detour budget stays on the route: unavoidable.
It is no unprotected left, so the status's lefts counts (TurnPilotMapStatus lefts: avoided, unavoidable) stay 0.
Positions never leave navd: the status says how many, the logs say steps.
"""
from collections.abc import Callable
from dataclasses import dataclass
from types import SimpleNamespace

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.map_controls import line_setback_at, unprotected_turn
from openpilot.selfdrive.turnpilot.route_follower import UTURN, maneuver_from_ahead
from openpilot.selfdrive.turnpilot.routing.guidance import maneuver_entry
from openpilot.selfdrive.turnpilot.unprotected_left import AUTO, ROAD_RANGE_M, left_mode_for, left_reach, range_param, route_range

AUTO_LEFT_PARAM, RANGE_PARAM, FAR_RANGE_PARAM, FAR_CROP_PARAM = 'TurnPilotAutoLeft', 'TurnPilotLeftRangeM', 'TurnPilotLeftFarRangeM', 'TurnPilotFarCrop'


@dataclass(frozen=True)
class LeftConfig:
  """What route_range is judged under, read at route time (left_config): TurnPilotAutoLeft's mode, the two proven ranges, the
  far crop on (TurnPilotFarCrop), and the side traffic keeps to (IsRhdDetected, as turnpilotd takes it)."""
  mode: str = AUTO
  range_m: float = ROAD_RANGE_M
  far_range_m: float = 0.
  far_on: bool = True
  right_hand_traffic: bool = True

  @property
  def reach_m(self) -> float:
    return left_reach(self.range_m, self.far_range_m, self.far_on)


@dataclass
class Left:
  """One maneuver the car refuses on a route (a U-turn since 2026-11-06: module docstring), in the avoidance loop's fields for an
  exclusion (a stop's Control has the same): approach_key and exclusion are its EXIT edge's (routing/controls.py: Exit), None
  with none known; never on the route's first edge; last_edge: the exit is the route's last. unavoidable: it stays on the route."""
  step_idx: int        # the step its maneuver ends
  route_m: float       # m along the route to its junction (navd's step frame)
  reach: float         # its R_req where the car may stand (route_range; a U-turn: 0)
  bent: bool           # its opposing road isn't known straight out to it (a U-turn: False)
  approach_key: tuple | None = None
  exclusion: tuple[float, float, float] | None = None
  last_edge: bool = False
  unavoidable: bool = False
  first_edge: bool = False
  kind: str = 'left'   # or 'uturn' (module docstring)


def read(params, key: str):
  """params.get(key), or None (said) when the build doesn't know it or it can't be read."""
  try:
    return params.get(key)
  except Exception as e:
    cloudlog.error(f"navd: {key} unreadable ({type(e).__name__}): its default for the route's lefts; run scons")
    return None


def flag(value, unset: bool) -> bool:
  if value is None:
    return unset
  if isinstance(value, (bytes, str)):
    return (value.decode(errors='replace') if isinstance(value, bytes) else value).strip() == '1'
  return value is True or value == 1


def left_config(params) -> LeftConfig:
  """The route's LeftConfig from params, as turnpilotd and modeld read them: TurnPilotAutoLeft (left_mode_for: auto unless off
  or announce), the ranges clamped (range_param; not a number: unmeasured, said), TurnPilotFarCrop on unless set off, and
  IsRhdDetected (a right-hand drive car: left-hand traffic). get_bool is never used: every value is read and interpreted."""
  def ranged(key: str, unset: float) -> float:
    value = read(params, key)
    try:
      return range_param(value, unset)
    except (TypeError, ValueError):
      cloudlog.error(f"navd: {key} {value!r} isn't a number: unmeasured for the route's lefts")
      return unset

  return LeftConfig(left_mode_for(read(params, AUTO_LEFT_PARAM)), ranged(RANGE_PARAM, ROAD_RANGE_M), ranged(FAR_RANGE_PARAM, 0.),
                    flag(read(params, FAR_CROP_PARAM), True), not flag(read(params, 'IsRhdDetected'), False))


def route_maneuver(steps, step_idx: int, opposing=None, right_hand_traffic: bool = True):
  """The maneuver that ends step step_idx as turnpilotd reads it from allManeuvers (route_follower.Maneuver), under the side
  traffic keeps to (LeftConfig.right_hand_traffic): a slight turn to the far side across an opposing approach is a TURN
  (route_follower.slight_far_turn), a slight right where traffic keeps left among them, as turnpilotd's follower takes it."""
  return maneuver_from_ahead(SimpleNamespace(**maneuver_entry(steps, step_idx, 0., opposing)), right_hand_traffic)


def setback_at(junction_m: float, controls) -> float | None:
  """map_controls.line_setback_at for the junction junction_m along the route, over the route's controls (routing/controls.py),
  the car at the route's start: only a mapped line counts, as turnpilotd's tracker's."""
  return line_setback_at(junction_m, [SimpleNamespace(d=c.route_m, line_mapped=True, kind=c.kind, cross_suspect=c.cross_suspect,
                                                      junction_info=c.junction_info.to_msg() if c.junction_info is not None else None)
                                      for c in controls or [] if getattr(c, 'line_mapped', False)])


def route_lefts(steps, controls, config: LeftConfig, opposing: Callable[[], dict], exits: Callable[[], dict]) -> list[Left]:
  """The route's U-turns (module docstring), nearest first: never an unprotected left, each driven to its line. Those are
  logged: how many, and the steps of those route_range says may hand back at their line (reach past config's, or bent).
  opposing() -> {step index: (m/s, lanes, straight m)} is asked only for a route with an unprotected left on it (for that log
  line), exits() -> {step index: Exit} only for one with a U-turn (both cached by the route's controls() on the real router, so
  no second Valhalla call)."""
  ends, at = [], 0.
  for i in range(len(steps) - 1):
    at += float(steps[i].get('distance', 0.))
    ends.append(at)
  rht = config.right_hand_traffic
  candidates = [i for i in range(len(steps) - 1) if config.mode == AUTO and unprotected_turn(route_maneuver(steps, i, None, rht), rht)]
  uturns = [i for i in range(len(steps) - 1) if route_maneuver(steps, i, None, rht).kind == UTURN]
  if candidates:
    # a log line only: a failure here never changes the route (the U-turns below are the one thing decided)
    try:
      roads = opposing() or {}
      checks = [(i, route_range(route_maneuver(steps, i, roads, rht), setback_at(ends[i], controls), config.reach_m, rht,
                                config.mode)) for i in candidates]
      back = [i for i, rc in checks if rc.reach > config.reach_m or rc.bent]
      cloudlog.info(f"turnpilot lefts: {len(candidates)} unprotected lefts on the route, each driven to its line, never routed " +
                    f"around; {len(back)} may hand back there (past the reach of {config.reach_m:.0f} m, or the oncoming road " +
                    f"not known straight: steps {back})")
    except Exception:
      cloudlog.exception("turnpilot lefts: the route's lefts log failed")
  if not uturns:
    return []
  edges, out = exits() or {}, []
  for i in uturns:
    ex = edges.get(i)
    out.append(Left(i, ends[i], 0., False, ex.key if ex else None, ex.exclusion if ex else None, bool(ex and ex.last_edge), kind='uturn'))
  cloudlog.info(f"turnpilot lefts: {len(uturns)} U-turns on the route (steps {uturns}), the car can't drive one")
  return out
