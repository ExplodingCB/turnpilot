"""Route-time stop-sign avoidance (STOP_SIGNS.md 3.3): every stop and yield control the certifier (routing/certify.py) can't
certify is excluded by its approach edge and the route recomputed around it, until none is left, none more can be
excluded, or a bound is hit. navd's RouteJob runs it after a route's controls (and route_audit, for the time it adds).

Valhalla has no costing option for stop signs, so the mechanism is exclude_locations on the control's approach edge,
min(12 m, half the edge) back from the node, with the edge's local bearing at that point and a 30 deg tolerance, so only
the direction that arrives at the stop is excluded (Control.exclusion, routing/controls.py; valhalla_source.py). The set of
points persists per destination (AvoidSet): every reroute navd makes for it (off the route, a skipped or missed turn, a
runtime input or an ETA change) passes the same set, so no reroute brings a banned approach back.

The loop (run), with route(exclude) the router's status-neutral reroute, which never writes TurnPilotMapStatus or downloads:
  - a control on the route's first or last edge, on the car's own edge (located before each call), or with no exclusion
    point can't be avoided: 'unavoidable', and its point is never sent while the car is on that edge;
  - every other uncertifiable control's point joins the set, and the route is recomputed around the whole set;
  - a recomputed route that still has a control whose point was sent (Valhalla dropped an exclusion, as it does with only a
    log warning when a point doesn't correlate) ends the loop 'failed', said loudly;
  - a failed iteration (no route around, a route off the corridor, any routing error) drops the batch it added from the set,
    keeps the previous steps and controls as a pair and ends 'keptBest', as do RouteJob's cancel, AVOID_TIME_S and
    MAX_AVOID_ITER, each checked between iterations;
  - otherwise it ends 'clean' (nothing uncertifiable left) or 'unavoidable' (only unavoidable controls left).
Whatever ends it, every stop and yield still uncertifiable on the kept route becomes unavoidable: the driver needs to know.
"""
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.routing.controls import same_edge
from openpilot.selfdrive.turnpilot.stop_rules import MAX_AVOID_ITER, MAX_EXCLUDE_LOCATIONS

STOP_KINDS = ('stop', 'yield')


def mark(control, certification: str, reason: str | None = None) -> None:
  """A control's verdict, on it and on its junction (MapJunction.certification) when it has one."""
  control.certification = certification
  if reason is not None:
    control.cert_reason = reason
  info = getattr(control, 'junction_info', None)
  if info is not None:
    info.certification = certification
    if reason is not None:
      info.cert_reason = reason


@dataclass
class AvoidPoint:
  key: tuple          # the approach edge (way id, forward, node)
  lat: float
  lon: float
  heading: float      # deg, the edge's local bearing at the point


class AvoidSet:
  """The exclusions for one destination, in the order they were added."""

  def __init__(self):
    self.points: dict[tuple, AvoidPoint] = {}

  def __len__(self) -> int:
    return len(self.points)

  def __contains__(self, key) -> bool:
    return key in self.points

  def keys(self) -> set:
    return set(self.points)

  def add(self, control, heading: float | None = None) -> None:
    """control's exclusion point, with Valhalla's own heading there when given (else the route's local bearing)."""
    lat, lon, bearing = control.exclusion
    self.points[control.approach_key] = AvoidPoint(control.approach_key, float(lat), float(lon), float(bearing if heading is None else heading))

  def drop(self, keys) -> None:
    for k in keys:
      self.points.pop(k, None)

  def exclude(self, own) -> tuple[list[dict], set]:
    """(exclude_locations points, their keys): every point but those on the car's own edge, the newest MAX_EXCLUDE_LOCATIONS."""
    pts = [p for p in self.points.values() if not same_edge(p.key, own)][-MAX_EXCLUDE_LOCATIONS:]
    return [{'lat': p.lat, 'lon': p.lon, 'heading': p.heading} for p in pts], {p.key for p in pts}


@dataclass
class AvoidResult:
  steps: list
  controls: list
  controls_status: str
  status: str                  # NavInstruction.AvoidStatus: none, clean, unavoidable, failed, keptBest
  avoided: int = 0             # stop and yield controls routed around (excluded and not on the kept route)
  iterations: int = 0          # reroutes made
  added_s: float = 0.          # s the kept route takes over the first
  sent: set = field(default_factory=set)


def time_at(steps, route_m: float) -> float:
  """s along the route steps to route_m (navd's step frame), from their durations, as navd reckons timeRemaining."""
  t = acc = 0.
  for st in steps:
    dist, dur = float(st.get('distance', 0.)), float(st.get('duration', 0.))
    if route_m <= acc + dist:
      return t + dur * max(route_m - acc, 0.) / max(dist, 1.)
    t, acc = t + dur, acc + dist
  return t


def duration(steps) -> float:
  return float(sum(float(s.get('duration', 0.)) for s in steps or []))


def stops(controls) -> list:
  return [c for c in controls if c.kind in STOP_KINDS]


def run(steps, controls, controls_status: str, avoid: AvoidSet, *, certify: Callable, reroute: Callable, route_controls: Callable,
        own_edge: Callable, cancelled: Callable, deadline: float, monotonic: Callable = time.monotonic, enabled: bool = True,
        max_iter: int = MAX_AVOID_ITER, headings: Callable = lambda points: [None for _ in points]) -> AvoidResult:
  """The loop (module docstring). certify(control, steps) -> (certification, reason), writing it to the control (steps: the
  route it is on, for its ETA); reroute(exclude)
  -> steps or None; route_controls(steps) -> (controls, status); own_edge() -> the car's edge key or None; cancelled() ->
  bool; deadline: monotonic() time it stops by. enabled False (TurnPilotAvoidStops off): certify only, status none.
  headings([(lat, lon, way id, forward)]) -> Valhalla's own heading at each new exclusion point, or None (the route's
  local bearing stays): Valhalla correlates a point against the heading it computes itself, which a short curved approach
  can put 50 deg off the route's (Indianapolis way 17439639), and drops it, so the stop survives as 'failed'."""
  first = duration(steps)
  for c in stops(controls):
    certify(c, steps)
  if not enabled:
    return AvoidResult(steps, controls, controls_status, 'none')
  if controls_status == 'unsupported':
    return AvoidResult(steps, controls, controls_status, 'none')
  if controls_status == 'failed':
    cloudlog.error("turnpilot avoid: the route's controls failed: no stop sign can be avoided, said as failed")
    return AvoidResult(steps, controls, controls_status, 'failed')
  sent, excluded, status, iterations = set(), set(), None, 0
  for _ in range(max_iter):
    if cancelled() or monotonic() > deadline:
      status = 'keptBest'
      cloudlog.warning(f"turnpilot avoid: {'cancelled' if cancelled() else 'out of time'} after {iterations} reroutes: the best route is kept")
      break
    own = own_edge()
    for c in stops(controls):
      if c.certification == 'uncertifiable' and (c.first_edge or c.last_edge or c.exclusion is None or c.approach_key is None or
                                                 same_edge(c.approach_key, own)):
        mark(c, 'unavoidable')
    bad = [c for c in stops(controls) if c.certification == 'uncertifiable']
    if not bad:
      status = 'unavoidable' if any(c.certification == 'unavoidable' for c in stops(controls)) else 'clean'
      break
    batch = [c for c in bad if c.approach_key not in avoid]
    found = headings([(c.exclusion[0], c.exclusion[1], c.approach_key[0], c.approach_key[1]) for c in batch]) if batch else []
    for c, h in zip(batch, list(found) + [None] * (len(batch) - len(found)), strict=True):
      avoid.add(c, h)
    exclude, keys = avoid.exclude(own)
    if keys == sent:
      for c in bad:
        mark(c, 'unavoidable')
      status = 'unavoidable'
      break
    iterations += 1
    new_steps = reroute(exclude)
    new = None
    if new_steps:
      try:
        new = route_controls(new_steps)
      except Exception:
        cloudlog.exception("turnpilot avoid: the reroute's controls failed")
    if new is None or new[1] == 'failed':
      avoid.drop([c.approach_key for c in batch])
      for c in batch:
        mark(c, 'unavoidable')
      status = 'keptBest'
      cloudlog.warning(f"turnpilot avoid: no route around {len(batch)} more stops: the best route is kept, those are unavoidable")
      break
    excluded |= {c.approach_key for c in bad} & keys
    steps, (controls, controls_status), sent = new_steps, new, keys
    for c in stops(controls):
      certify(c, steps)
    survivors = [c for c in stops(controls) if c.approach_key in sent]
    if survivors:
      status = 'failed'
      cloudlog.error(f"turnpilot avoid: Valhalla dropped {len(survivors)} exclusions (a point that didn't correlate): " +
                     ', '.join(f"{c.kind} at {c.route_m:.0f} m" for c in survivors))
      break
  else:
    status = 'keptBest'
    cloudlog.warning(f"turnpilot avoid: {max_iter} reroutes and stops still left: the best route is kept")
  for c in stops(controls):
    if c.certification == 'uncertifiable':
      mark(c, 'unavoidable')
  on_route = {c.approach_key for c in stops(controls)}
  avoided = len(excluded - on_route)
  cloudlog.info(f"turnpilot avoid: {status}, {avoided} stops routed around in {iterations} reroutes, +{duration(steps) - first:.0f} s")
  return AvoidResult(steps, controls, controls_status, status, avoided, iterations, duration(steps) - first, sent)
