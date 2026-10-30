"""The route's stop signs, yields and traffic signals between navd's 1 Hz fixes (navInstruction.controls).

navd sends each control's distance once a second from the GPS projection onto the route. A projection at standstill is
pure noise (2 m of it walked the old junction hold 4 m over its line), and one a second jumps by the GPS error, so the
distance here is dead-reckoned with vEgo every frame, and a fresh fix pulls it at most CORRECTION_M (none while slower
than FREEZE_SPEED). A fix more than JUMP_M off is a re-initialisation, logged (turnpilot.control_jump), and the policy
re-checks whether the stop is still feasible (map_controls.py).

A control's identity is (routeCount, id), never the maneuver: a skipped turn or a straight-through junction keeps its
stop. A new route (a reroute) drops the controls the car wasn't stopping for yet, and keeps an engaged or held one: a
same-kind control of the new route within ADOPT_M of it takes over its state (it is the same one), and one the new route
doesn't list is carried under its old key until passed. A held control is never dropped and its distance is frozen:
the latch is the policy's, position-free.
A gap in the route (an invalid navInstruction, or none for NAV_STALE_S) is dead-reckoned through, for GAP_MAX_M of
travel; then every control but a held one is dropped, and listed in lost for the policy to say so.

Off the route with a destination (navd rerouting, no route yet, off it for a fix), navd looks up the controls on the road
ahead of the car itself once a second (navInstruction.offRouteControls, routing/controls.py: controls_ahead). Those are
the controls then, under (OFF_ROUTE, id), each lookup a fix as above: the stale route's go once one is in, as they're
nowhere the car is going (BeamNG route_demo, 2026-10-28: the old route's signal dead-reckoned 106 m out while the car drove
through the one on its road). An engaged one of theirs is taken over by a lookup's same-kind control within ADOPT_M, at the
lookup's distance (the stale route's dead-reckoned one is on a road the car may have left), a held one stays. One a
lookup no longer lists goes, an engaged one after OFF_ROUTE_MISS lookups in a row (one is a GPS hiccup; two, the car
turned off that road). A route again takes over the engaged off-route ones the same way.
Off the route with a destination and nothing known ahead (off_route_unknown), past OFF_ROUTE_GRACE_S: the lookup pending,
failed, unsupported or partial, or none fresh for OFF_LOOKUP_STALE_S (navd gone quiet). The policy caps the speed and says so.
"""
from dataclasses import dataclass, field
from typing import Any

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.route_follower import NAV_STALE_S

CORRECTION_M = 0.5    # a fix moves the dead-reckoned distance at most this far
FREEZE_SPEED = 0.5    # m/s: slower than this, fixes are ignored
JUMP_M = 20.          # a fix this far off re-initialises the distance
FIX_SPREAD_M = 20.    # m of travel a control's fixes are kept over for their spread (fix_spread)
ADOPT_M = 20.         # a new route's control this close to an engaged one of the same kind is that one
DROP_PAST_M = 15.     # a control this far behind the car is passed
GAP_MAX_M = 150.      # m dead-reckoned without navd before the controls are dropped
OFF_ROUTE = -2        # the route key of navd's lookups ahead of the car off the route
OFF_ROUTE_MISS = 2    # lookups in a row an engaged off-route control is missing from before it goes
OFF_ROUTE_GRACE_S = 2.5  # s off the route before the stops ahead count as unknown: navd sends one invalid message between
                         # two routes, and its first lookup comes within its next second
OFF_LOOKUP_STALE_S = 3.  # s: an older lookup doesn't say what is ahead now
LOOKUP_KNOWN = ('ok', 'walkOrSnap')  # lookup statuses that say what is ahead (partial may miss stop signs)
LOOKUP_USED = LOOKUP_KNOWN + ('partial',)  # ...and those whose controls are used, as far as they go


@dataclass
class TrackedControl:
  key: tuple[int, int]       # (routeCount, id)
  kind: str                  # 'stop', 'yield' or 'signal'
  d: float                   # m to the stop point, dead-reckoned
  line_mapped: bool = False
  junction: bool = True
  cross_suspect: bool = False
  engaged: bool = False      # the policy is stopping (or deciding) for it: carried through a reroute
  held: bool = False         # the policy's latch holds the car at it: never dropped, its distance frozen
  jumped: bool = False       # re-initialised by a jump this frame
  misses: int = 0            # off-route lookups in a row it was missing from
  state: Any = None          # the policy's own state for it
  junction_info: dict | None = None  # a stop or yield's MapControl.junctionInfo as a dict (routing/junctions.py); None a signal's
  fixes: list = field(default_factory=list)  # (odometer m, navd's distance plus it) of its moving fixes over the last FIX_SPREAD_M

  @property
  def fix_spread(self) -> float:
    """m navd's fixes of it disagree by, its odometry removed, over the last FIX_SPREAD_M of fixes (frozen at the latch, as
    fixes are while held): the stop-sign decider's position fault (STOP_SIGNS.md 7.1 H2)."""
    vals = [x for _, x in self.fixes]
    return max(vals) - min(vals) if len(vals) >= 2 else 0.


def from_msg(route_count: int, c) -> TrackedControl:
  """A tracked control from navd's MapControl c; its junction copied out (the message's buffer isn't kept)."""
  return TrackedControl((route_count, int(c.id)), str(c.kind), float(c.distance), bool(c.lineMapped), bool(c.junction),
                        bool(c.crossSuspect), junction_info=c.junctionInfo.to_dict() if c._has('junctionInfo') else None)


def adopt(o: TrackedControl, candidates, measured: bool = False) -> bool:
  """An engaged or held control o handed to the same-kind one of candidates (another route's, or a lookup's) within ADOPT_M
  that has no state of its own: it is the same control. False when there is none. measured: the candidate's own distance
  is kept (a lookup's, on the road the car is on, over the stale route's dead-reckoned one), jumped when it moves the
  stop by more than a fix would, so the policy re-checks it. The candidate keeps its own junction_info: the new message's
  (a reroute may change the maneuver there)."""
  same = [n for n in candidates if n.kind == o.kind and n.state is None and not n.engaged and abs(n.d - o.d) < ADOPT_M]
  if not same:
    return False
  n = min(same, key=lambda n: abs(n.d - o.d))
  if measured:
    n.jumped = abs(n.d - o.d) > CORRECTION_M
  else:
    n.d = o.d
  n.engaged, n.held, n.state = o.engaged, o.held, o.state
  return True


class ControlTracker:
  def __init__(self):
    self.controls: dict[tuple[int, int], TrackedControl] = {}
    self.route_count: int | None = None
    self.status = 'none'     # navInstruction.controlsStatus of the last valid message
    self.route = False       # on a route now: the last navInstruction was valid and fresh
    self.invalid = False     # the last navInstruction was invalid (off the route, rerouting)
    self.stale_t = 0.
    self.gap_m = 0.          # m travelled in a gap
    self.lost: list[TrackedControl] = []  # dropped this frame by a gap too long
    self.off_status = 'none'  # navInstruction.offRouteControlsStatus of the last invalid message; none after a valid one
    self.gap_t = 0.          # s off the route (invalid or stale)
    self.lookup_age = float('inf')  # s since the last lookup ahead that said something
    self.odo = 0.            # m driven (dead reckoned): the odometer TrackedControl.fixes are kept against

  def update(self, nav, updated: bool, valid: bool, v_ego: float, dt: float) -> list[TrackedControl]:
    """nav: the latest navInstruction; updated: it arrived this frame; valid: its valid flag. The controls nearest first."""
    v = max(v_ego, 0.)
    self.lost = []
    self.odo += v * dt
    for c in self.controls.values():
      c.jumped = False
      if not c.held:
        c.d -= v * dt
    self.lookup_age += dt
    if updated and valid:
      self.stale_t, self.gap_m, self.route, self.invalid = 0., 0., True, False
      self.off_status, self.gap_t = 'none', 0.
      self._fix(nav, v)
    else:
      self.stale_t = 0. if updated else self.stale_t + dt
      self.invalid = self.invalid or updated
      if updated:
        self.off_status = str(nav.offRouteControlsStatus)
        if self.off_status in LOOKUP_USED:
          self._off_fix(nav, v)
      gap = self.invalid or self.stale_t > NAV_STALE_S
      if gap:
        self.route = False
        self.gap_t += dt
        self.gap_m += v * dt
        if self.gap_m > GAP_MAX_M:
          self.lost = [c for c in self.controls.values() if not c.held]
          for c in self.lost:
            cloudlog.event("turnpilot.control_lost", kind=c.kind, key=list(c.key), engaged=c.engaged, gap_m=round(self.gap_m, 1))
          self.controls = {k: c for k, c in self.controls.items() if c.held}
    self.controls = {k: c for k, c in self.controls.items() if c.held or c.d >= -DROP_PAST_M}
    return sorted(self.controls.values(), key=lambda c: c.d)

  @property
  def off_route_unknown(self) -> bool:
    """Off the route with a destination, past OFF_ROUTE_GRACE_S, and no fresh lookup that says what is ahead (module
    docstring): rerouting, or before the first route (a corridor still downloading). Not with no destination (navd's
    off-route status none on its fresh invalid messages), nor from a navd never heard from with a route."""
    if self.route or self.gap_t < OFF_ROUTE_GRACE_S:
      return False
    if self.off_status == 'none' and (self.route_count is None or (self.invalid and self.stale_t <= NAV_STALE_S)):
      return False  # no destination: navd is on no route, nor trying to be
    return not (self.off_status in LOOKUP_KNOWN and self.lookup_age <= OFF_LOOKUP_STALE_S)

  def _off_fix(self, nav, v: float) -> None:
    """A lookup ahead of the car off the route (module docstring)."""
    self.lookup_age, self.gap_m = 0., 0.
    measured = {(OFF_ROUTE, int(c.id)): c for c in nav.offRouteControls}
    new = {k: from_msg(OFF_ROUTE, c) for k, c in measured.items() if k not in self.controls}
    for k, c in list(self.controls.items()):
      if k[0] == OFF_ROUTE or c.held:
        continue
      # the stale route's: nowhere the car is going. One the car is stopping for goes on as the lookup's same one
      del self.controls[k]
      if c.engaged and not adopt(c, new.values(), measured=True):
        cloudlog.event("turnpilot.control_stale_dropped", kind=c.kind, key=list(c.key), d=round(c.d, 1))
    for k, c in list(self.controls.items()):
      if k[0] != OFF_ROUTE or k in measured:
        continue
      c.misses += 1
      if not c.held and (not c.engaged or c.misses >= OFF_ROUTE_MISS):
        if c.engaged:
          cloudlog.event("turnpilot.control_off_route_dropped", kind=c.kind, key=list(c.key), d=round(c.d, 1))
        del self.controls[k]
    self.controls.update(new)
    for key, c in measured.items():
      t = self.controls[key]
      t.misses = 0
      if key not in new and not t.held:
        self._correct(t, key, float(c.distance), v)

  def _fix(self, nav, v: float) -> None:
    route_count = int(nav.routeCount)
    self.status = str(nav.controlsStatus)
    measured = {(route_count, int(c.id)): c for c in nav.controls}
    if route_count != self.route_count:
      old, self.controls = self.controls, {k: from_msg(route_count, c) for k, c in measured.items()}
      self.route_count = route_count
      for o in old.values():
        if (o.engaged or o.held) and not adopt(o, self.controls.values()):
          self.controls[o.key] = o
      return
    for key, c in measured.items():
      t = self.controls.get(key)
      if t is None:
        self.controls[key] = from_msg(route_count, c)
        continue
      if t.held:
        continue
      self._correct(t, key, float(c.distance), v, self.odo)
    # back on the route: its own control takes over an engaged off-route one, the same control
    for k, o in list(self.controls.items()):
      if k[0] == OFF_ROUTE and (o.engaged or o.held) and adopt(o, [self.controls[m] for m in measured if m in self.controls]):
        del self.controls[k]
    # out of navd's window (passed, beyond 400 m, past the 8 nearest): only the ones the car is stopping for stay
    self.controls = {k: c for k, c in self.controls.items() if k in measured or c.engaged or c.held}

  @staticmethod
  def _correct(t: TrackedControl, key, measured: float, v: float, odo: float | None = None) -> None:
    """A fix of a dead-reckoned distance: at most CORRECTION_M, none while slow, a jump re-initialises it. A moving fix on
    the route is kept for its spread (fix_spread), the odometer added back."""
    if odo is not None and v >= FREEZE_SPEED:
      t.fixes.append((odo, measured + odo))
      t.fixes[:] = [x for x in t.fixes if x[0] >= odo - FIX_SPREAD_M]
    err = measured - t.d
    if abs(err) > JUMP_M:
      cloudlog.event("turnpilot.control_jump", kind=t.kind, key=list(key), d=round(t.d, 1), measured=round(measured, 1))
      t.d, t.jumped = measured, True
    elif v >= FREEZE_SPEED:
      t.d += min(max(err, -CORRECTION_M), CORRECTION_M)
