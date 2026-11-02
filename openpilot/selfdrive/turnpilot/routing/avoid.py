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
  - so does a reroute that takes more than detour_budget_s longer than the plain route (the first, before any exclusion): a
    stop is never avoided at any price. With no stop certified yet (no surveys: notSurveyed on every stop), on the region's
    real tiles the unbounded loop kept routes up to 74% slower (+242 s) and 47% longer than the plain one, 1 in 8 over +20%:
    the detours the first drive's owner called routes no sensible driver would pick. Under vision stop-go the line's own
    gate decides an unavoidable stop as any other (stop_go.py V1), so the detour buys little; it is kept to AVOID_ADD_FRAC
    of the plain route, at least AVOID_ADD_MIN_S (a short trip may still go round a block) and at most AVOID_ADD_MAX_S;
  - otherwise it ends 'clean' (nothing uncertifiable left) or 'unavoidable' (only unavoidable controls left).
Whatever ends it, every stop and yield still uncertifiable on the kept route becomes unavoidable: the driver needs to know.

The lefts the car would refuse (routing/lefts.py: an autonomous unprotected left past the detector's proven range, which
turnpilotd skips as soon as it is the maneuver and navd reroutes) go round the same loop, excluded by their exit edge (that
module says why not the approach), in the same set, under the same bounds and the same detour budget, with:
  - lefts first: the route's new lefts are each iteration's batch while it has any, its stops only after. A left the car
    refuses is a certain skip and a reroute at its junction; an unavoidable stop is a stop the line's own gate decides
    (vision stop-go), so the lefts get the first claim on the budget, and a reroute for stops that brings more lefts the car
    would skip is a failed iteration;
  - a reroute around lefts with more of them than the route it would replace is a failed iteration too, so the count never
    goes up; one with as many is taken (excluding a left's exit often moves the route to the next block's left, and
    excluding that one may clear it), and the route with the fewest, then the quickest, of every route the lefts went
    through, plain included, is the one kept, the set's later exclusions dropped. On the region's real tiles (40 random
    downtown trips, both ranges unmeasured, stops not avoided), taking any route within the budget left 6 trips with more
    lefts than their plain route (5 for 3); so ruled, the plain routes' 94 lefts the car would skip were 61 on the kept
    ones, none more than its own plain route's, at most +38 s, the loop at most 0.28 s on a Mac;
  - a batch of lefts that fails is retried one left at a time, nearest first (one too dear never costs the rest theirs); a
    single left that fails is unavoidable, and the loop goes on to the next, then to the stops, where a failed batch still
    ends it 'keptBest';
  - a left already in the set (an earlier route's for this destination) joins no batch, but the whole set goes out with every
    reroute, and once even with no new left, so a reroute after a skip never brings a banned left back;
  - a left still on a recomputed route whose point was sent is unavoidable, said loudly (the stops' 'failed' is about stops:
    the route-start notice says "Stop-sign routing failed"). Only the stops' own reroutes look for a stop's dropped point: one
    a reroute for lefts carried and Valhalla dropped is found by the stops' first pass as nothing more to send, unavoidable;
  - they don't ride TurnPilotAvoidStops (step S3 turns the stops' avoidance off to keep the junctions it validates), nor need
    the route's controls (unsupported tiles), only their exits (the same trace);
  - every left still on the kept route is unavoidable: skipped out loud at its junction and rerouted, as before. The counts
    (AvoidResult: lefts_avoided, the plain route's lefts not on the kept one; lefts_unavoidable, the kept one's) are what
    navd's TurnPilotMapStatus says, never where.
The route's U-turns (routing/lefts.py: a Left of kind 'uturn', which the driving model can't drive) are lefts to the loop,
every rule above theirs too, but no unprotected left to the counts: lefts_avoided and lefts_unavoidable are the lefts' alone,
what the phone page says of unprotected lefts, and the U-turns' are logged.
avoidStatus stays the stops' (none with their avoidance off or unsupported, failed with their controls failed): a route whose
lefts alone went round the loop says nothing about stop signs.
"""
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.routing.controls import same_edge
from openpilot.selfdrive.turnpilot.stop_rules import MAX_AVOID_ITER, MAX_EXCLUDE_LOCATIONS

STOP_KINDS = ('stop', 'yield')
AVOID_ADD_FRAC = 0.10   # the avoidance may add this share of the plain route's time (module docstring)...
AVOID_ADD_MIN_S = 30.   # ...but at least this much...
AVOID_ADD_MAX_S = 180.  # ...and never more than this, however long the trip (route_notice chimes from +50% or +5 min)


def detour_budget_s(base_s: float) -> float:
  """s the avoidance may add over the plain route of base_s s: AVOID_ADD_FRAC of it, within AVOID_ADD_MIN_S..AVOID_ADD_MAX_S."""
  return min(max(AVOID_ADD_FRAC * max(base_s, 0.), AVOID_ADD_MIN_S), AVOID_ADD_MAX_S)


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
  lefts_avoided: int = 0       # lefts the car would skip, routed around (excluded and not on the kept route)
  lefts: list = field(default_factory=list)  # those still on the kept route, every one unavoidable (routing/lefts.py: Left)

  @property
  def lefts_unavoidable(self) -> int:
    return sum(t.kind == 'left' for t in self.lefts)  # its U-turns are no unprotected lefts (module docstring)


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
        max_iter: int = MAX_AVOID_ITER, headings: Callable = lambda points: [None for _ in points],
        lefts: Callable = lambda steps, controls: []) -> AvoidResult:
  """The loop (module docstring). certify(control, steps) -> (certification, reason), writing it to the control (steps: the
  route it is on, for its ETA); reroute(exclude)
  -> steps or None; route_controls(steps) -> (controls, status); own_edge() -> the car's edge key or None; cancelled() ->
  bool; deadline: monotonic() time it stops by. enabled False (TurnPilotAvoidStops off): certify only, status none.
  headings([(lat, lon, way id, forward)]) -> Valhalla's own heading at each new exclusion point, or None (the route's
  local bearing stays): Valhalla correlates a point against the heading it computes itself, which a short curved approach
  can put 50 deg off the route's (Indianapolis way 17439639), and drops it, so the stop survives as 'failed'.
  A reroute past detour_budget_s of the plain route (steps, as given) is a failed iteration: its batch dropped, the best kept.
  lefts(steps, controls) -> [Left], the route's lefts the car would skip (routing/lefts.py: route_lefts)."""
  first = duration(steps)
  budget = detour_budget_s(first)
  for c in stops(controls):
    certify(c, steps)
  if enabled and controls_status == 'failed':
    cloudlog.error("turnpilot avoid: the route's controls failed: no stop sign can be avoided, said as failed")

  def live(status: str) -> bool:  # the stops of a route whose controls say status are routed around
    return enabled and status not in ('unsupported', 'failed')

  turns = lefts(steps, controls)
  if not live(controls_status) and not turns:
    return AvoidResult(steps, controls, controls_status, 'failed' if enabled and controls_status == 'failed' else 'none')
  plain_lefts = {t.approach_key for t in turns if t.approach_key is not None and t.kind == 'left'}
  plain_uturns = {t.approach_key for t in turns if t.approach_key is not None and t.kind == 'uturn'}
  sent, excluded, status, iterations, singly = set(), set(), None, 0, False
  added: list = []  # the keys this run put in the set, in order (never one an earlier route's job put there)

  def bounded() -> bool:  # a bound ends the loop before another reroute: the best route is kept
    if cancelled() or monotonic() > deadline:
      cloudlog.warning(f"turnpilot avoid: {'cancelled' if cancelled() else 'out of time'} after {iterations} reroutes: the best route is kept")
      return True
    if iterations >= max_iter:
      cloudlog.warning(f"turnpilot avoid: {max_iter} reroutes and stops or lefts still left: the best route is kept")
      return True
    return False

  def exclude_with(batch, own):
    found = headings([(c.exclusion[0], c.exclusion[1], c.approach_key[0], c.approach_key[1]) for c in batch]) if batch else []
    for c, h in zip(batch, list(found) + [None] * (len(batch) - len(found)), strict=True):
      if c.approach_key not in avoid:
        added.append(c.approach_key)
      avoid.add(c, h)
    return avoid.exclude(own)

  def drop(batch) -> None:
    keys = {c.approach_key for c in batch}
    avoid.drop(keys)
    added[:] = [k for k in added if k not in keys]

  def attempt(exclude):
    """(steps, (controls, status), lefts) of the route around exclude, or (None, why) for a failed iteration: no route, or
    past the budget, or more lefts the car would skip than the route it would replace."""
    new_steps = reroute(exclude)
    new = None
    if new_steps:
      try:
        new = route_controls(new_steps)
      except Exception:
        cloudlog.exception("turnpilot avoid: the reroute's controls failed")
    if new is None or new[1] == 'failed':
      return None, "no route around"
    more = duration(new_steps) - first
    if more > budget:
      return None, f"{more:.0f} s over the plain route's {first:.0f} s, past the {budget:.0f} s budget, for the route around"
    new_turns = lefts(new_steps, new[0])
    if len(new_turns) > len(turns):
      return None, f"{len(new_turns)} lefts the car would skip, not {len(turns)}, on the route around"
    return (new_steps, new, new_turns), ''

  def take(got, keys) -> None:
    """The reroute is the route: its stops certified, a left of it whose point was sent unavoidable (Valhalla dropped it)."""
    nonlocal steps, controls, controls_status, turns, sent
    steps, (controls, controls_status), turns, sent = got[0], got[1], got[2], keys
    for c in stops(controls):
      certify(c, steps)
    kept = [t for t in turns if t.approach_key in sent]
    for t in kept:
      t.unavoidable = True
    if kept:
      cloudlog.error(f"turnpilot avoid: Valhalla dropped {len(kept)} left exclusions (a point that didn't correlate): " +
                     ', '.join(f"the left ending step {t.step_idx}" for t in kept))

  # the lefts first (module docstring), the route with the fewest of them, then the quickest, kept: plain or a reroute
  best = None
  while turns:
    own = own_edge()
    for t in turns:
      if t.last_edge or t.exclusion is None or t.approach_key is None or same_edge(t.approach_key, own):
        t.unavoidable = True
    bad_lefts = [t for t in turns if not t.unavoidable]
    if not bad_lefts:
      break
    if bounded():
      status = 'keptBest'
      break
    if best is None or (len(turns), duration(steps)) < best[0]:
      best = ((len(turns), duration(steps)), steps, controls, controls_status, turns, sent, set(excluded), len(added))
    # a left already in the set (an earlier route's for this destination) is in no batch, but the set goes out again whole,
    # as the stops' does: a reroute after a skip or off the route never brings a banned left back
    new_lefts = sorted((t for t in bad_lefts if t.approach_key not in avoid), key=lambda t: t.route_m)
    batch = new_lefts[:1] if singly else new_lefts
    bad = [c for c in stops(controls) if c.certification == 'uncertifiable'] if live(controls_status) else []
    exclude, keys = exclude_with(batch, own)
    if keys == sent:  # nothing more to send: what is left on the route stays
      for t in bad_lefts:
        t.unavoidable = True
      break
    iterations += 1
    got, why = attempt(exclude)
    if got is None:
      drop(batch)
      if not batch:  # the set alone routes nowhere within the budget: its lefts still on this route stay
        for t in bad_lefts:
          t.unavoidable = True
        cloudlog.warning(f"turnpilot avoid: {why} the set's {len(bad_lefts)} lefts: unavoidable, skipped at their junctions")
        break
      if len(batch) > 1:
        singly = True
        cloudlog.warning(f"turnpilot avoid: {why} {len(batch)} lefts: they are tried one at a time, nearest first")
      else:
        batch[0].unavoidable = True
        cloudlog.warning(f"turnpilot avoid: {why} the left ending step {batch[0].step_idx}: unavoidable, skipped at its junction")
      continue
    excluded |= {c.approach_key for c in bad} & keys
    take(got, keys)
  if best is not None and (len(turns), duration(steps)) > best[0]:
    _, steps, controls, controls_status, turns, sent, excluded, mark_at = best
    avoid.drop(added[mark_at:])
    del added[mark_at:]
    cloudlog.info(f"turnpilot avoid: an earlier route with {len(turns)} lefts the car would skip is kept: as few, and quicker")

  # then the stops, as before; a reroute for them that brings more lefts the car would skip is a failed iteration
  while status is None:
    if bounded():
      status = 'keptBest'
      break
    own = own_edge()
    on = live(controls_status)
    for c in stops(controls) if on else []:
      if c.certification == 'uncertifiable' and (c.first_edge or c.last_edge or c.exclusion is None or c.approach_key is None or
                                                 same_edge(c.approach_key, own)):
        mark(c, 'unavoidable')
    bad = [c for c in stops(controls) if c.certification == 'uncertifiable'] if on else []
    if not bad:
      status = 'unavoidable' if any(c.certification == 'unavoidable' for c in stops(controls)) else 'clean'
      break
    batch = [c for c in bad if c.approach_key not in avoid]
    exclude, keys = exclude_with(batch, own)
    if keys == sent:
      for c in bad:
        mark(c, 'unavoidable')
      status = 'unavoidable'
      break
    iterations += 1
    got, why = attempt(exclude)
    if got is None:
      drop(batch)
      for c in batch:
        mark(c, 'unavoidable')
      status = 'keptBest'
      cloudlog.warning(f"turnpilot avoid: {why} {len(batch)} more stops: the best route is kept, those are unavoidable")
      break
    excluded |= {c.approach_key for c in bad} & keys
    take(got, keys)
    survivors = [c for c in stops(controls) if c.approach_key in sent]
    if survivors:
      status = 'failed'
      cloudlog.error(f"turnpilot avoid: Valhalla dropped {len(survivors)} exclusions (a point that didn't correlate): " +
                     ', '.join(f"{c.kind} at {c.route_m:.0f} m" for c in survivors))
      break
  if live(controls_status):
    for c in stops(controls):
      if c.certification == 'uncertifiable':
        mark(c, 'unavoidable')
  else:
    status = 'failed' if enabled and controls_status == 'failed' else 'none'
  for t in turns:
    t.unavoidable = True
  on_route = {c.approach_key for c in stops(controls)}
  avoided = len(excluded - on_route)
  lefts_avoided = len(plain_lefts - {t.approach_key for t in turns})
  uturns_avoided = len(plain_uturns - {t.approach_key for t in turns})
  uturns_left = sum(t.kind == 'uturn' for t in turns)
  cloudlog.info(f"turnpilot avoid: {status}, {avoided} stops, {lefts_avoided} lefts and {uturns_avoided} U-turns routed around in " +
                f"{iterations} reroutes, {len(turns) - uturns_left} lefts and {uturns_left} U-turns unavoidable, +{duration(steps) - first:.0f} s")
  return AvoidResult(steps, controls, controls_status, status, avoided, iterations, duration(steps) - first, sent, lefts_avoided, turns)
