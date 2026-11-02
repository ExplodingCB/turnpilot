"""Route time's unprotected lefts: the route's lefts the car would refuse, for the avoidance loop to route around (routing/avoid.py).

turnpilotd never attempts an autonomous unprotected left whose required clear range is past the detector's proven range
(TurnPilotLeftRangeM, the far crop's TurnPilotLeftFarRangeM): it is skipped out loud as soon as it is the maneuver
(turn_skip: unprotectedRange) and navd reroutes. Both ranges are unmeasured on the car (0), so every left with an opposing
approach was skipped, and Valhalla's routes were full of them: the first drive's routes "very sketchy", navigation giving up at a
red light before a left. So the route plans none it can help: each maneuver is judged here exactly as turnpilotd judges it,
  - the maneuver as turnpilotd reads it off allManeuvers (guidance.maneuver_entry, route_follower.maneuver_from_ahead): a
    junction turn to the far side whose junction has an opposing approach, or might have (guidance.maneuver_junction: present,
    or unknown without junction data; none at a T's stem or a bend);
  - its opposing road's mapped limit, lanes and straight span (routing/controls.py: opposing_roads; {} on a failure: 45 mph and
    not known straight, as turnpilotd takes what navd then sends);
  - the junction's mapped stop line's setback from the route's controls (map_controls.line_setback_at, the car at the start);
  - unprotected_left.route_range, the predicate turnpilotd skips by, under TurnPilotAutoLeft and the reach left_reach gives
    (navd takes the far crop as on unless TurnPilotFarCrop is off: it reads no farStatus);
and a left out of range is a Left, carrying its exit edge's exclusion (routing/controls.py: Exit) in the fields the avoidance
loop reads for a stop's approach (approach_key, exclusion, first_edge, last_edge).
Why the exit edge, directed, not the approach edge a stop is excluded by: the approach edge can never be excluded while the
car is on it (avoid.py: the car's own edge is never sent), and a left the car is standing short of (at a red light before
it, where the first drive's navigation gave up) always is; and it closes our own road into the junction to straight on and
the right as well, where the exit edge leaves every movement from our road open and closes only the way into that one road
away from the junction (the opposite approach's right and the cross road's straight on into it go with it; that road's other
direction stays open). A point at the node with no heading would close the whole junction. On the BeamNG west_coast_usa
tiles the one left the car would skip on the test trip went, its exit excluded, for a route 8 s quicker.
A left whose exit is the route's last edge (onto the destination's street), or with none known, can't be routed around:
unavoidable, as is one over the detour budget; turnpilotd skips it out loud at its junction and navd reroutes, as before. A
measured range keeps the lefts within it, as ever.
Positions never leave navd: the status says how many (TurnPilotMapStatus lefts: avoided, unavoidable), the logs say steps.
"""
from collections.abc import Callable
from dataclasses import dataclass
from types import SimpleNamespace

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.map_controls import line_setback_at, unprotected_turn
from openpilot.selfdrive.turnpilot.route_follower import maneuver_from_ahead
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
  """One left the car would skip on a route, in the avoidance loop's fields for an exclusion (a stop's Control has the same):
  approach_key and exclusion are its EXIT edge's (routing/controls.py: Exit), None with none known; never on the route's first
  edge; last_edge: the exit is the route's last. unavoidable: it stays on the route (skipped out loud at its junction)."""
  step_idx: int        # the step its maneuver ends
  route_m: float       # m along the route to its junction (navd's step frame)
  reach: float         # its R_req where the car may stand (route_range)
  bent: bool           # its opposing road isn't known straight out to it
  approach_key: tuple | None = None
  exclusion: tuple[float, float, float] | None = None
  last_edge: bool = False
  unavoidable: bool = False
  first_edge: bool = False
  kind: str = 'left'


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


def route_maneuver(steps, step_idx: int, opposing=None):
  """The maneuver that ends step step_idx as turnpilotd reads it from allManeuvers (route_follower.Maneuver)."""
  return maneuver_from_ahead(SimpleNamespace(**maneuver_entry(steps, step_idx, 0., opposing)))


def setback_at(junction_m: float, controls) -> float | None:
  """map_controls.line_setback_at for the junction junction_m along the route, over the route's controls (routing/controls.py),
  the car at the route's start: only a mapped line counts, as turnpilotd's tracker's."""
  return line_setback_at(junction_m, [SimpleNamespace(d=c.route_m, line_mapped=True, kind=c.kind, cross_suspect=c.cross_suspect,
                                                      junction_info=c.junction_info.to_msg() if c.junction_info is not None else None)
                                      for c in controls or [] if getattr(c, 'line_mapped', False)])


def route_lefts(steps, controls, config: LeftConfig, opposing: Callable[[], dict], exits: Callable[[], dict]) -> list[Left]:
  """The route's lefts the car would skip (module docstring), nearest first. opposing() -> {step index: (m/s, lanes, straight m)}
  and exits() -> {step index: Exit} are asked only for a route with an unprotected left on it (both cached by the route's
  controls() on the real router, so no second Valhalla call)."""
  ends, at = [], 0.
  for i in range(len(steps) - 1):
    at += float(steps[i].get('distance', 0.))
    ends.append(at)
  candidates = [i for i in range(len(steps) - 1) if config.mode == AUTO and
                unprotected_turn(route_maneuver(steps, i), config.right_hand_traffic)]
  if not candidates:
    return []
  roads, edges, out = opposing() or {}, exits() or {}, []
  for i in candidates:
    rc = route_range(route_maneuver(steps, i, roads), setback_at(ends[i], controls), config.reach_m, config.right_hand_traffic,
                     config.mode)
    if not rc.out_of_range:
      continue
    ex = edges.get(i)
    out.append(Left(i, ends[i], rc.reach, rc.bent, ex.key if ex else None, ex.exclusion if ex else None, bool(ex and ex.last_edge)))
  if out:
    cloudlog.info(f"turnpilot lefts: {len(out)} of the route's {len(candidates)} unprotected lefts past the reach of " +
                  f"{config.reach_m:.0f} m (steps {[t.step_idx for t in out]}), {sum(t.exclusion is None for t in out)} with no exit known")
  return out
