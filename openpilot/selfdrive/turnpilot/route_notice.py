"""What the driver hears about the route's stop signs (STOP_SIGNS.md v3 section 3.4; SG4c), from navd's navInstruction, said as
turnpilotPlan.controlAlert with its text in turnpilotPlan.routeNotice (selfdrived's turnpilotRouteNotice, engaged or not).

At route start (a route appearing after ROUTE_GAP_S with none, or a new one while the car stands disengaged: never a reroute
while driving), in this order, each held NOTICE_S:
- avoidStatus failed: "Stop-sign routing failed: stops may need you" (stopRoutingFailed, a chime);
- avoidedControls N: "Route avoids N stop signs (+M min)" with the time avoidance added (avoidAddedS), shown only
  (routeAvoidsStops), or with a chime past DETOUR_WARN, the larger of +50% of the route without it and +5 min (routeDetour),
  so a long loop is never silent; the route still avoids;
- unavoidableControls M: "M stop signs on this route need you" (stopsNeedYou, a chime), and with the first of them on the
  route's first edge (firstUnavoidableOnOrigin) and openpilot not engaged, "First stop needs you" (firstStopNeedsYou, a chime;
  decision D5: the demo starts on a through street). The first one's street isn't on the bus, so it isn't said.
Then, on any route, AHEAD_M before each stop or yield navd marks unavoidable (junctionInfo.certification), or at once where it
is first seen closer: "Stop sign ahead needs you" (stopAheadNeedsYou, a chime), once per junction. At the line the decider
hands it back right after the full stop (stop_go.py: 7.1)."""
from dataclasses import dataclass, field

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog

NOTICE_S = 3.             # s each notice is held, one after another
ROUTE_GAP_S = 5.          # s with no valid route before a route is a route start
START_V = 1.0             # m/s: under this, disengaged, a new route is a route start (the car at its origin)
AHEAD_M = 300.            # m before a stop the route couldn't avoid: "Stop sign ahead needs you"
DETOUR_WARN_FRAC = 0.5    # avoidance adding more than this of the route without it...
DETOUR_WARN_MIN_S = 300.  # ...or this, whichever is larger, is said with a chime
STOP_KINDS = ('stop', 'yield')
CHIMES = ('routeDetour', 'stopsNeedYou', 'firstStopNeedsYou', 'stopRoutingFailed', 'stopAheadNeedsYou')
NOTICES = ('routeAvoidsStops',) + CHIMES


def detour_warn_s(base_s: float) -> float:
  """s of added time past which the route-start notice chimes: the larger of DETOUR_WARN_FRAC of the route without avoidance
  and DETOUR_WARN_MIN_S."""
  return max(DETOUR_WARN_FRAC * max(base_s, 0.), DETOUR_WARN_MIN_S)


def plural(n: int, word: str) -> str:
  return f"{n} {word}{'' if n == 1 else 's'}"


@dataclass
class RouteNotices:
  queue: list = field(default_factory=list)   # [(kind, text)] still to say
  now: tuple = ('none', '')                    # (kind, text) being said
  left_s: float = 0.
  route: int | None = None                     # navd's routeCount of the route last seen
  no_route_s: float = ROUTE_GAP_S              # s since the last valid route
  ahead_said: set = field(default_factory=set)  # junctions already said ahead of
  seen_start: bool = False

  @staticmethod
  def chime(kind: str) -> bool:
    return kind in CHIMES

  def update(self, nav, updated: bool, valid: bool, v_ego: float, engaged: bool, dt: float = DT_MDL) -> tuple[str, str]:
    """(controlAlert kind, text) to say this frame, ('none', '') with none. nav: the latest navInstruction (None when none this
    frame), updated: it arrived this frame, valid: its valid flag."""
    if updated and nav is not None and valid:
      self._route(nav, v_ego, engaged)
      self._ahead(nav)
    else:
      self.no_route_s += dt  # no message this frame, or an invalid one (off the route): the gap counts on
    if self.left_s > 0.:
      self.left_s = max(self.left_s - dt, 0.)
    if self.left_s <= 0.:
      self.now = self.queue.pop(0) if self.queue else ('none', '')
      if self.now[0] != 'none':
        self.left_s = NOTICE_S
        cloudlog.event("turnpilot.route_notice", kind=self.now[0], text=self.now[1])
    return self.now

  def _route(self, nav, v_ego: float, engaged: bool) -> None:
    route = int(nav.routeCount)
    start = route != self.route and (self.no_route_s >= ROUTE_GAP_S or (v_ego < START_V and not engaged))
    self.route, self.no_route_s = route, 0.
    if not start:
      return
    self.seen_start = True
    out = []
    if str(nav.avoidStatus) == 'failed':
      out.append(('stopRoutingFailed', "Stop-sign routing failed: stops may need you"))
    n = int(nav.avoidedControls)
    if n > 0:
      added = max(float(nav.avoidAddedS), 0.)
      base = max(float(nav.timeRemaining) - added, 0.)
      kind = 'routeDetour' if added > detour_warn_s(base) else 'routeAvoidsStops'
      out.append((kind, f"Route avoids {plural(n, 'stop sign')} (+{added / 60.:.0f} min)"))
    m = int(nav.unavoidableControls)
    if m > 0:
      out.append(('stopsNeedYou', f"{plural(m, 'stop sign')} on this route need{'s' if m == 1 else ''} you"))
      if bool(nav.firstUnavoidableOnOrigin) and not engaged:
        out.append(('firstStopNeedsYou', "First stop needs you"))
    if out:
      cloudlog.event("turnpilot.route_start_notices", route=route, notices=[k for k, _ in out], avoided=n, unavoidable=m,
                     added_s=round(float(nav.avoidAddedS), 1), status=str(nav.avoidStatus))
    self.queue = out + [q for q in self.queue if q[0] == 'stopAheadNeedsYou']

  def _ahead(self, nav) -> None:
    for c in nav.controls:
      if str(c.kind) not in STOP_KINDS or not c._has('junctionInfo') or str(c.junctionInfo.certification) != 'unavoidable':
        continue
      ident = int(c.junctionInfo.osmNodeId) or (int(nav.routeCount), int(c.id))
      if ident in self.ahead_said or not (0. < float(c.distance) <= AHEAD_M):
        continue
      self.ahead_said.add(ident)
      cloudlog.event("turnpilot.stop_ahead_needs_you", junction=str(ident), d=round(float(c.distance), 1))
      self.queue.append(('stopAheadNeedsYou', "Stop sign ahead needs you"))  # after a route start's own, said first
