"""Route time's unprotected lefts (routing/lefts.py) and the avoidance loop routing around them (routing/avoid.py): the shared
predicate turnpilotd skips by (unprotected_left.route_range), the exits off the route's trace (routing/controls.py), and a
synthetic street grid routed by a small Dijkstra with Valhalla's exclude_locations semantics (Grid), so a route the loop keeps
is one a real router would give around the exclusions it sends. Synthetic coordinates only: metres from (0, 0)."""
import heapq
import math
from collections import defaultdict
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from openpilot.selfdrive.navigation.navd import RouteEngine
from openpilot.selfdrive.turnpilot.maps.status import NavStatus
from openpilot.selfdrive.turnpilot.map_controls import line_setback_at
from openpilot.selfdrive.turnpilot.route_follower import LEFT, RIGHT, TURN, Maneuver
from openpilot.selfdrive.turnpilot.routing import avoid
from openpilot.selfdrive.turnpilot.routing.controls import EXCLUDE_BACK_M, Exits, route_controls, route_exits
from openpilot.selfdrive.turnpilot.routing.lefts import LeftConfig, left_config, route_lefts, route_maneuver
from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import LAT0, M_PER_DEG, StubValhalla, pt, turn_steps
from openpilot.selfdrive.turnpilot.stop_rules import EXCLUDE_HEADING_TOL
from openpilot.selfdrive.turnpilot.unprotected_left import (LEFT_RANGE_MAX_M, LINE_D, ROUTE_REACH_MARGIN_M, left_reach, range_param,
                                                           reach_at_line, route_range)

V_20 = 8.94  # m/s, 20 mph


def left(**kw) -> Maneuver:
  """A junction left with an oncoming approach, its road 20 mph, one lane, straight 400 m: R_req ~164 m from its line."""
  return Maneuver(**{'kind': TURN, 'side': LEFT, 'type': 'turn', 'modifier': 'left', 'angle_deg': 90., 'geometry': True,
                     'opposing': 'present', 'opposing_speed': V_20, 'opposing_lanes': 1, 'opposing_straight_m': 400.} | kw)


class TestSharedPredicate:
  """unprotected_left.route_range is the one check turnpilotd skips a left by (turn_skip: unprotectedRange) and navd routes
  around: these are its rules, which test_turnpilotd_left.py pins on turnpilotd's side."""

  def test_unmeasured_range_refuses_every_left_with_an_oncoming_approach(self):
    reach = reach_at_line(V_20, 1, ROUTE_REACH_MARGIN_M)
    rc = route_range(left(), None, 0.)
    assert (rc.unprotected, rc.out_of_range, rc.bent, rc.margin) == (True, True, False, ROUTE_REACH_MARGIN_M)
    assert rc.reach == pytest.approx(reach)
    assert route_range(left(opposing='unknown'), None, 0.).out_of_range  # no junction data is never clear

  def test_a_measured_range_keeps_the_lefts_within_it(self):
    reach = route_range(left(), None, 0.).reach
    assert not route_range(left(), None, reach + 1.).out_of_range
    assert route_range(left(), None, reach - 1.).out_of_range
    # its road not known straight out to its R_req is never attempted, whatever the range (the band is straight on)
    rc = route_range(left(opposing_straight_m=reach - LINE_D - ROUTE_REACH_MARGIN_M - 1.), None, LEFT_RANGE_MAX_M)
    assert rc.bent and rc.out_of_range
    # a mapped stop line's setback replaces the margin: a line at the hold line needs less reach
    assert route_range(left(), 0., 0.).reach < reach

  @pytest.mark.parametrize('maneuver, rht, mode', [(left(side=RIGHT, modifier='right'), True, 'auto'),  # a right: curb side
                                                    (left(opposing='none'), True, 'auto'),               # a T's stem
                                                    (left(kind='keep', modifier='slight left'), True, 'auto'),
                                                    (left(), False, 'auto'),                             # left-hand traffic
                                                    (left(), True, 'announce'), (left(), True, 'off')])  # the line hands back
  def test_what_is_never_skipped_for_its_range(self, maneuver, rht, mode):
    rc = route_range(maneuver, None, 0., rht, mode)
    assert not rc.out_of_range and not rc.unprotected

  def test_the_reach_and_the_params(self):
    assert (left_reach(0., 300., True), left_reach(150., 300., True), left_reach(150., 300., False), left_reach(150., 0., True)) == \
      (0., 300., 150., 150.)  # the far crop extends only a measured full pass's range, and only while on
    assert (range_param(None, 0.), range_param(b'120', 0.), range_param(900., 0.), range_param(-5, 0.)) == (0., 120., LEFT_RANGE_MAX_M, 0.)
    with pytest.raises(ValueError):
      range_param('far', 0.)

  def test_line_setback_rule_is_shared(self):
    line = SimpleNamespace(d=80., line_mapped=True, kind='stop', cross_suspect=False, junction_info={'stopPointDistance': 20.})
    assert line_setback_at(100., [line]) == pytest.approx(20. - 8. - 1.)
    assert line_setback_at(100., [SimpleNamespace(**{**vars(line), 'line_mapped': False})]) is None
    rail = SimpleNamespace(**{**vars(line), 'junction_info': {'railDistance': 5., 'stopPointDistance': 20.}})
    assert line_setback_at(100., [rail]) is None  # a rail orphan's stop is the tracks', not the junction's


def params(values):
  return SimpleNamespace(get=lambda k: values[k] if k in values else None)


class TestLeftConfig:
  def test_defaults_are_the_cars(self):
    assert left_config(params({})) == LeftConfig('auto', 0., 0., True, True)

  def test_values_as_turnpilotd_and_modeld_read_them(self):
    cfg = left_config(params({'TurnPilotAutoLeft': 'announce', 'TurnPilotLeftRangeM': 150., 'TurnPilotLeftFarRangeM': b'250',
                              'TurnPilotFarCrop': False, 'IsRhdDetected': True}))
    assert cfg == LeftConfig('announce', 150., 250., False, False) and cfg.reach_m == 150.
    assert left_config(params({'TurnPilotLeftRangeM': 'far', 'TurnPilotAutoLeft': 'sideways'})) == LeftConfig()  # said, unmeasured

  def test_an_unknown_key_is_its_default(self):
    def get(key):
      raise KeyError(key)
    assert left_config(SimpleNamespace(get=get)) == LeftConfig()


# ---- the exits, off the route's own trace (routing/controls.py)

def test_the_exit_is_the_edge_the_route_leaves_the_junction_by():
  """The maneuver ending step 0 (a left at 100 m) leaves by the edge starting there: its key in own_edge's form (way, forward,
  end node), its point EXCLUDE_BACK_M past the node with the route's bearing, not the route's last edge with one after it."""
  v = StubValhalla([(0., 100., {}), (100., 250., {'way_id': 812}), (250., 400., {})])
  out = Exits()
  route_controls(v, turn_steps(100., 'left'), exits=out)
  ex = out.edges[0]
  assert ex.key == (812, True, (round(pt(250.)[0], 6), round(pt(250.)[1], 6))) and not ex.last_edge
  assert (ex.exclusion[0] - LAT0) * M_PER_DEG == pytest.approx(100. + EXCLUDE_BACK_M, abs=0.01)
  assert ex.exclusion[2] == pytest.approx(0., abs=0.1)  # due north, as the route
  assert route_exits(v, turn_steps(100., 'left')) == out.edges  # alone, from a trace of its own (tiles with no controls)
  # past a divided road's median (internal edges) to the road beyond; the destination's edge is the last
  v = StubValhalla([(0., 100., {}), (100., 110., {'internal': True}), (110., 400., {'way_id': 77})])
  ex = route_exits(v, turn_steps(100., 'left'))[0]
  assert ex.key[0] == 77 and ex.last_edge
  assert (ex.exclusion[0] - LAT0) * M_PER_DEG == pytest.approx(110. + EXCLUDE_BACK_M, abs=0.01)
  # a trace that doesn't meet the step's start there gives none (never a guess at the wrong edge)
  assert route_exits(StubValhalla([(0., 80., {}), (80., 400., {})]), turn_steps(100., 'left')) == {}


# ---- a synthetic street grid

RIGHT_S, LEFT_S = 2., 5.  # s a right and a left cost the grid's router, as Valhalla's turn costs make lefts dearer
MATCH_M = 3.


def ll(p):
  return p[1] / M_PER_DEG, p[0] / M_PER_DEG


def xy(lat, lon):
  return lon * M_PER_DEG, lat * M_PER_DEG


def heading(a, b) -> float:
  return math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360.


def turned(a: float, b: float) -> float:
  """Signed heading change, + to the right."""
  return (b - a + 180.) % 360. - 180.


def seg_m(p, a, b) -> float:
  dx, dy = b[0] - a[0], b[1] - a[1]
  t = min(max(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy), 0.), 1.)
  return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


class Grid:
  """Two-way streets [(way id, [(x, y) m, ...], m/s)], every point a node. route() is Dijkstra by time with turn costs (RIGHT_S,
  LEFT_S, no U-turns) around the directed edges an exclude_location closes as Valhalla's does (within MATCH_M of its point,
  heading within EXCLUDE_HEADING_TOL), and gives Valhalla's format=osrm steps: a maneuver at each turn, every node's
  intersection with its roads' bearings, in and out. opposing() and exits() answer for the steps as the real router does."""

  def __init__(self, roads):
    self.out = defaultdict(list)
    for way, points, speed in roads:
      for a, b in zip(points, points[1:], strict=False):
        self.out[a].append((b, way, speed))
        self.out[b].append((a, way, speed))
    self.paths: dict = {}
    self.excludes: list = []

  def closed(self, exclude) -> set:
    shut = set()
    for p in exclude or []:
      q = xy(p['lat'], p['lon'])
      shut |= {(a, b) for a, nbrs in self.out.items() for b, _, _ in nbrs
               if seg_m(q, a, b) <= MATCH_M and abs(turned(heading(a, b), p['heading'])) <= EXCLUDE_HEADING_TOL}
    return shut

  def route(self, start, bearing: float, dest, exclude=None):
    self.excludes.append(exclude)
    shut = self.closed(exclude)
    behind = (start[0] - math.sin(math.radians(bearing)), start[1] - math.cos(math.radians(bearing)))
    queue, done = [(0., start, behind, (start,))], set()
    while queue:
      t, n, prev, path = heapq.heappop(queue)
      if n == dest:
        return self.steps(path)
      if (n, prev) in done:
        continue
      done.add((n, prev))
      for m, _, speed in self.out[n]:
        if m == prev or (n, m) in shut:
          continue
        turn = turned(heading(prev, n), heading(n, m))
        cost = math.dist(n, m) / speed + (0. if abs(turn) < 30. else RIGHT_S if turn > 0. else LEFT_S)
        heapq.heappush(queue, (t + cost, m, n, path + (m,)))
    return None

  def edge(self, a, b):
    return next((way, speed) for m, way, speed in self.out[a] if m == b)

  def intersection(self, path, k) -> dict:
    n = path[k]
    bearings = sorted(round(heading(n, m)) % 360 for m, _, _ in self.out[n])
    x = {'location': [ll(n)[1], ll(n)[0]], 'bearings': bearings, 'entry': [True] * len(bearings),
         'out': bearings.index(round(heading(n, path[k + 1])) % 360)}
    if k > 0:
      x['in'] = bearings.index(round(heading(n, path[k - 1])) % 360)
    return x

  def steps(self, path) -> list[dict]:
    turns = [k for k in range(1, len(path) - 1) if abs(turned(heading(path[k - 1], path[k]), heading(path[k], path[k + 1]))) >= 30.]
    out = []
    for k0, k1 in zip([0] + turns, turns + [len(path) - 1], strict=True):
      seg = list(zip(path[k0:k1], path[k0 + 1:k1 + 1], strict=True))
      dist = sum(math.dist(a, b) for a, b in seg)
      t = sum(math.dist(a, b) / self.edge(a, b)[1] for a, b in seg)
      after = heading(path[k0], path[k0 + 1])
      if k0 == 0:
        maneuver = {'type': 'depart', 'bearing_before': 0, 'bearing_after': round(after)}
      else:
        before = heading(path[k0 - 1], path[k0])
        turn = turned(before, after)
        maneuver = {'type': 'turn', 'modifier': 'right' if turn > 0. else 'left', 'bearing_before': round(before), 'bearing_after': round(after)}
        t += RIGHT_S if turn > 0. else LEFT_S
      maneuver['location'] = [ll(path[k0])[1], ll(path[k0])[0]]
      out.append({'maneuver': maneuver, 'distance': dist, 'duration': t, 'bannerInstructions': [],
                  'geometry': {'coordinates': [[ll(p)[1], ll(p)[0]] for p in path[k0:k1 + 1]]},
                  'intersections': [self.intersection(path, k) for k in range(k0, k1)]})
    end = [ll(path[-1])[1], ll(path[-1])[0]]
    out.append({'maneuver': {'type': 'arrive', 'location': end}, 'distance': 0., 'duration': 0., 'bannerInstructions': [],
                'geometry': {'coordinates': [end, end]}, 'intersections': [{'location': end, 'bearings': [0], 'in': 0, 'entry': [True]}]})
    self.paths[id(out)] = path
    return out

  def junctions(self, steps):
    """[(step index, its turn's node index in the path)] for each turn that ends a step."""
    path, at = self.paths[id(steps)], 0
    out = []
    for i, s in enumerate(steps[:-1]):
      at += len(s['geometry']['coordinates']) - 1
      if steps[i + 1]['maneuver']['type'] == 'turn':
        out.append((i, at))
    return path, out

  def opposing(self, steps) -> dict:
    """{step index: (m/s, lanes, straight m)}: the road straight on through each turn's junction, out as far as it stays so."""
    path, turns = self.junctions(steps)
    out = {}
    for i, k in turns:
      ahead = heading(path[k - 1], path[k])
      n = path[k]
      nxt = next(((m, s) for m, _, s in self.out[n] if abs(turned(ahead, heading(n, m))) < 10.), None)
      if nxt is None:
        continue
      straight, a = 0., n
      while nxt is not None and straight < 400.:
        straight += math.dist(a, nxt[0])
        a = nxt[0]
        nxt = next(((m, s) for m, _, s in self.out[a] if abs(turned(ahead, heading(a, m))) < 10.), None)
      out[i] = (self.edge(n, next(m for m, _, _ in self.out[n] if abs(turned(ahead, heading(n, m))) < 10.))[1], 1, min(straight, 400.))
    return out

  def exits(self, steps) -> dict:
    """{step index: Exit}: the edge each turn leaves its junction by, as routing/controls.py's exit_edges."""
    from openpilot.selfdrive.turnpilot.routing.controls import Exit
    path, turns = self.junctions(steps)
    out = {}
    for i, k in turns:
      a, b = path[k], path[k + 1]
      back = min(EXCLUDE_BACK_M, math.dist(a, b) / 2.)
      h = heading(a, b)
      p = (a[0] + back * math.sin(math.radians(h)), a[1] + back * math.cos(math.radians(h)))
      out[i] = Exit((self.edge(a, b)[0], True, tuple(round(v, 6) for v in ll(b))), (*ll(p), h), k + 1 == len(path) - 1)
    return out

  def own(self, start, bearing):
    """own_edge's key for a car at node start heading bearing: the edge it is on, ahead of it."""
    b = next(m for m, _, _ in self.out[start] if abs(turned(bearing, heading(start, m))) < 10.)
    return self.edge(start, b)[0], True, tuple(round(v, 6) for v in ll(b))


def grid(slow=5., b_ends=600, east_from=200):
  """Streets every 100 m at 10 m/s: columns x 0 and 200 (C) from y 100 to 300, x 300 from east_from, and A (x 100) from the car
  at y -100, its block y 100..200 at slow m/s; rows y 200 (N) and 300 from x 0 to 300, and B (y 100) at 8 m/s out east to
  b_ends: past C its long straight run is the oncoming road of the left off B onto C."""
  roads = [(10, [(0, y) for y in range(100, 301, 100)], 10.), (12, [(200, y) for y in range(100, 301, 100)], 10.),
           (13, [(300, y) for y in range(east_from, 301, 100)], 10.)]
  roads += [(1, [(100, -100), (100, 100)], 10.), (2, [(100, 100), (100, 200)], slow), (3, [(100, 200), (100, 300)], 10.)]
  roads += [(20 + y // 100, [(x, y) for x in range(0, 301, 100)], 10.) for y in (200, 300)]
  roads += [(21, [(0, 100), (100, 100), (150, 100), (200, 100)] + [(x, 100) for x in range(300, b_ends + 1, 100)], 8.)]
  return Grid(roads)


START, NORTH, DEST = (100, -100), 0., (300, 200)
CAR = LeftConfig()  # both ranges unmeasured, TurnPilotAutoLeft auto: the car today


def turns_of(steps):
  return [s['maneuver'].get('modifier') for s in steps[1:-1]]


def skipped(g, steps, config=CAR):
  return route_lefts(steps, [], config, lambda: g.opposing(steps), lambda: g.exits(steps))


def run_grid(g, start=START, bearing=NORTH, dest=DEST, config=CAR, own=None, avoid_set=None, **kw):
  plain = g.route(start, bearing, dest)
  result = avoid.run(plain, [], 'ok', avoid_set if avoid_set is not None else avoid.AvoidSet(), certify=lambda c, s: None,
                     reroute=lambda exclude: g.route(start, bearing, dest, exclude), route_controls=lambda s: ([], 'ok'),
                     own_edge=lambda: own, cancelled=lambda: False, deadline=math.inf, monotonic=lambda: 0.,
                     lefts=lambda s, c: skipped(g, s, config), **kw)
  return plain, result


class TestGrid:
  def test_the_plain_route_turns_left_across_oncoming_traffic(self):
    g = grid()
    plain = g.route(START, NORTH, DEST)
    assert turns_of(plain) == ['right', 'left', 'right']  # right at A x B, left at B x C (B's oncoming lanes), right onto D's row
    m = route_maneuver(plain, 1, g.opposing(plain))
    assert (m.kind, m.side, m.opposing, m.opposing_speed, m.opposing_straight_m) == (TURN, LEFT, 'present', 8., 400.)
    assert [(t.step_idx, t.exclusion is not None, t.last_edge) for t in skipped(g, plain)] == [(1, True, False)]

  def test_routed_around_by_right_turns_within_the_budget(self):
    """Both ranges unmeasured (the car today): the left is excluded by its exit, C northbound out of the junction, and the
    route goes straight on up A and right instead, 0.5 s slower, well inside the detour budget."""
    g = grid()
    plain, r = run_grid(g)
    assert turns_of(r.steps) == ['right'] and skipped(g, r.steps) == []
    assert (r.lefts_avoided, r.lefts_unavoidable, r.iterations, r.status) == (1, 0, 1, 'clean')
    (p,) = g.excludes[1]
    assert xy(p['lat'], p['lon']) == pytest.approx((200., 100. + EXCLUDE_BACK_M), abs=0.01) and p['heading'] == pytest.approx(0.)
    assert 0. < r.added_s <= avoid.detour_budget_s(avoid.duration(plain))

  def test_a_measured_range_keeps_the_left(self):
    g = grid()
    reach = skipped(g, g.route(START, NORTH, DEST))[0].reach
    plain, r = run_grid(g, config=LeftConfig(range_m=reach + 10.))
    assert r.steps is plain and r.iterations == 0 and (r.lefts_avoided, r.lefts_unavoidable) == (0, 0)
    # its road not known straight that far (B ends 100 m past the junction) is refused whatever the range
    g = grid(b_ends=300)
    plain, r = run_grid(g, config=LeftConfig(range_m=LEFT_RANGE_MAX_M))
    assert r.lefts_avoided == 1 and skipped(g, plain, LeftConfig(range_m=LEFT_RANGE_MAX_M))[0].bent

  def test_over_the_budget_the_left_stays_and_is_said_unavoidable(self):
    """A crawling block on A puts the way round past the budget, and every other way round turns left as often: the plain
    route is kept, its left unavoidable (skipped out loud at the junction, rerouted), the set left as it was."""
    g = grid(slow=1.)
    held = avoid.AvoidSet()
    plain, r = run_grid(g, avoid_set=held)
    assert r.steps is plain and (r.lefts_avoided, r.lefts_unavoidable) == (0, 1) and r.lefts[0].unavoidable
    assert len(held) == 0 and r.iterations >= 1

  def test_a_left_onto_the_destinations_street_is_unavoidable_without_a_reroute(self):
    g = grid()
    plain, r = run_grid(g, dest=(200, 200))
    assert turns_of(plain) == ['right', 'left'] and skipped(g, plain)[0].last_edge
    assert (r.iterations, r.lefts_unavoidable, len(g.excludes)) == (0, 1, 1)

  def test_the_left_the_car_is_standing_short_of_is_routed_around_by_its_exit(self):
    """The car on the left's approach edge (at a red light short of it): an approach exclusion could never be sent (the car's
    own edge, never excluded), the exit can. The route goes straight on through the junction and round the corner at B's end,
    a bend no other road meets (no left to skip there)."""
    g = grid(b_ends=300, east_from=100)
    start, east = (150, 100), 90.
    own = g.own(start, east)
    plain, r = run_grid(g, start=start, bearing=east, own=own)
    assert turns_of(plain) == ['left', 'right'] and own == (21, True, tuple(round(v, 6) for v in ll((200, 100))))  # its approach
    assert (r.lefts_avoided, r.lefts_unavoidable) == (1, 0) and turns_of(r.steps) == ['left']  # B's end: a bend, no left
    assert route_maneuver(r.steps, 0).kind == 'none'
    assert skipped(g, r.steps) == [] and all(avoid.same_edge(k, own) is False for k in r.sent)

  def test_navd_routes_around_it_and_says_how_many_never_where(self):
    """The whole of navd: RouteJob's avoidance with the lefts (TurnPilotAutoLeft auto, both ranges unmeasured), and the route's
    counts in TurnPilotMapStatus for the phone page."""
    g = grid()
    store = {}

    class Router:
      status = NavStatus(SimpleNamespace(put=store.__setitem__))

      def route(self, start, destination, bearing, language, position=None, cancelled=None):
        self.status.set('ready', 'Ready', destination=list(destination))
        return self.reroute(start, destination, bearing, language, None)

      def reroute(self, start, destination, bearing, language, exclude):
        return g.route(tuple(round(v) for v in xy(start[1], start[0])), NORTH, tuple(round(v) for v in xy(destination[1], destination[0])),
                       exclude)

      def controls(self, steps):
        return [], 'ok'

      def opposing(self, steps):
        return g.opposing(steps)

      def exits(self, steps):
        return g.exits(steps)

    values = {'LanguageSetting': 'main_en'}
    engine = RouteEngine(MagicMock(), MagicMock(), params=SimpleNamespace(get=values.get), router=Router(), background=False)
    engine.last_position, engine.last_bearing = (ll(START)[1], ll(START)[0]), NORTH
    engine.calculate_route((ll(DEST)[1], ll(DEST)[0]))
    assert turns_of(engine.route) == ['right']
    assert (engine.lefts_avoided, engine.lefts_unavoidable) == (1, 0)
    status = store['TurnPilotMapStatus']
    assert status['lefts'] == {'avoided': 1, 'unavoidable': 0} and set(status) == {'state', 'message', 'short', 'code', 'destination',
                                                                                    'lefts', 'at'}


# ---- the loop's rules, on scripted routes

def lefts_at(*route_ms, exit_=True, last=False):
  from openpilot.selfdrive.turnpilot.routing.lefts import Left
  return [Left(k, m, 160., False, (900 + int(m), True, (m / M_PER_DEG, 0.)) if exit_ else None,
               (m / M_PER_DEG, 0., 0.) if exit_ else None, last) for k, m in enumerate(route_ms)]


class Scripted:
  """Routes in turn for each reroute (None: no route), each with its lefts and stops, by identity."""
  def __init__(self, plain, reroutes):
    self.of, self.calls = {}, []
    self.plain = self.make(*plain)
    self.reroutes = [None if r is None else self.make(*r) for r in reroutes]

  def make(self, duration_s, route_lefts, stops=()):
    steps = [{'distance': 100., 'duration': duration_s}]
    self.of[id(steps)] = (route_lefts, list(stops))
    return steps

  def reroute(self, exclude):
    self.calls.append(exclude)
    return self.reroutes.pop(0) if self.reroutes else None

  def run(self, enabled=True, controls_status='ok', **kw):
    return avoid.run(self.plain, self.of[id(self.plain)][1], controls_status, avoid.AvoidSet(), certify=lambda c, s: None,
                     reroute=self.reroute, route_controls=lambda s: (self.of[id(s)][1], 'ok'), own_edge=lambda: None,
                     cancelled=lambda: False, deadline=math.inf, monotonic=lambda: 0., enabled=enabled,
                     lefts=lambda s, c: [SimpleNamespace(**vars(t)) for t in self.of[id(s)][0]], **kw)


def stop(n, route_m):
  from openpilot.selfdrive.turnpilot.routing.controls import Control
  from openpilot.selfdrive.turnpilot.routing.junctions import Junction
  c = Control(n, 'stop', route_m, 0, route_m, junction_info=Junction(), approach_key=(500 + n, True, (0.0001 * n, 0.001)),
              exclusion=(0.0001 * n, 0.0009, 90.))
  c.certification = 'uncertifiable'
  return c


class TestLoopRules:
  def test_lefts_go_first_then_the_stops(self):
    s = stop(1, 50.)
    sc = Scripted((100., lefts_at(30.), [s]), [(105., [], [s]), (110., [], [])])
    r = sc.run()
    assert [len(x) for x in sc.calls] == [1, 2] and sc.calls[0][0]['lat'] == pytest.approx(30. / M_PER_DEG)
    assert (r.lefts_avoided, r.avoided, r.status) == (1, 1, 'clean')

  def test_a_reroute_with_more_lefts_is_refused_and_the_left_stays(self):
    sc = Scripted((100., lefts_at(30.)), [(105., lefts_at(40., 60.))])
    r = sc.run()
    assert r.steps is sc.plain and (r.lefts_avoided, r.lefts_unavoidable) == (0, 1)

  def test_as_many_lefts_but_slower_reverts_to_the_plain_route_and_its_set(self):
    held = avoid.AvoidSet()
    sc = Scripted((100., lefts_at(30.)), [(110., lefts_at(70.)), None])
    r = avoid.run(sc.plain, [], 'ok', held, certify=lambda c, s: None, reroute=sc.reroute, route_controls=lambda s: ([], 'ok'),
                  own_edge=lambda: None, cancelled=lambda: False, deadline=math.inf, monotonic=lambda: 0.,
                  lefts=lambda s, c: [SimpleNamespace(**vars(t)) for t in sc.of[id(s)][0]])
    assert len(sc.calls) == 2 and r.steps is sc.plain and len(held) == 0
    assert (r.lefts_avoided, r.lefts_unavoidable, r.added_s) == (0, 1, 0.)

  def test_a_batch_too_dear_is_tried_one_left_at_a_time_nearest_first(self):
    sc = Scripted((100., lefts_at(30., 60.)), [(200., []), (110., lefts_at(60.)), None])
    r = sc.run()
    assert [len(x) for x in sc.calls] == [2, 1, 2] and sc.calls[1][0]['lat'] == pytest.approx(30. / M_PER_DEG)
    assert (r.lefts_avoided, r.lefts_unavoidable, r.status) == (1, 1, 'clean')

  def test_with_the_stops_avoidance_off_the_lefts_still_go_round_and_say_nothing_of_stops(self):
    s = stop(1, 50.)
    sc = Scripted((100., lefts_at(30.), [s]), [(105., [], [s])])
    r = sc.run(enabled=False)
    assert len(sc.calls) == 1 and r.status == 'none' and r.lefts_avoided == 1 and s.certification == 'uncertifiable'
    sc = Scripted((100., lefts_at(30.)), [(105., [])])
    assert sc.run(controls_status='unsupported').lefts_avoided == 1

  def test_a_left_with_no_exit_known_is_unavoidable_with_no_reroute(self):
    sc = Scripted((100., lefts_at(30., exit_=False)), [])
    r = sc.run()
    assert sc.calls == [] and r.lefts_unavoidable == 1 and r.status == 'clean'

  def test_every_reroute_for_the_destination_sends_the_set_again(self):
    """The set persists per destination (as the stops', N12): a later route job (off the route, past a skipped turn) whose
    plain route has a left already banned sends the set at once, with no new left to batch, and routes around it again; with
    the stops' avoidance off and no stop on the route, as on the sim's tiles."""
    held = avoid.AvoidSet()
    for k in range(2):
      sc = Scripted((100., lefts_at(30.)), [(104., [])])
      r = avoid.run(sc.plain, [], 'unsupported', held, certify=lambda c, s: None, reroute=sc.reroute,
                    route_controls=lambda s: ([], 'unsupported'), own_edge=lambda: None, cancelled=lambda: False, deadline=math.inf,
                    monotonic=lambda: 0., enabled=False, lefts=lambda s, c, sc=sc: [SimpleNamespace(**vars(t)) for t in sc.of[id(s)][0]])
      assert len(sc.calls) == 1 and sc.calls[0][0]['lat'] == pytest.approx(30. / M_PER_DEG), k
      assert (r.lefts_avoided, r.lefts_unavoidable, len(held)) == (1, 0, 1), k
    # the set alone past the budget: the left stays, unavoidable, and the set keeps it for the next reroute
    sc = Scripted((100., lefts_at(30.)), [(200., [])])
    r = avoid.run(sc.plain, [], 'ok', held, certify=lambda c, s: None, reroute=sc.reroute, route_controls=lambda s: ([], 'ok'),
                  own_edge=lambda: None, cancelled=lambda: False, deadline=math.inf, monotonic=lambda: 0.,
                  lefts=lambda s, c: [SimpleNamespace(**vars(t)) for t in sc.of[id(s)][0]])
    assert len(sc.calls) == 1 and r.steps is sc.plain and (r.lefts_avoided, r.lefts_unavoidable, len(held)) == (0, 1, 1)

  def test_a_dropped_left_exclusion_is_unavoidable_never_the_stops_failed(self):
    kept = lefts_at(30.)
    sc = Scripted((100., kept), [(100., kept)])
    r = sc.run()
    assert (r.lefts_unavoidable, r.status, len(sc.calls)) == (1, 'clean', 1)


class TestStatus:
  def test_the_counts_join_the_ready_status_of_that_destination_only(self):
    store = {}
    st = NavStatus(SimpleNamespace(put=store.__setitem__))
    st.set('ready', 'Ready', destination=[1., 2.])
    assert st.note((1., 2.), lefts={'avoided': 2, 'unavoidable': 1})
    assert store['TurnPilotMapStatus']['lefts'] == {'avoided': 2, 'unavoidable': 1}
    assert not st.note((3., 4.), lefts={'avoided': 9, 'unavoidable': 9})  # another destination's
    st.set('downloading', 'Downloading', destination=[1., 2.])
    assert not st.note((1., 2.), lefts={'avoided': 9, 'unavoidable': 9}) and 'lefts' not in store['TurnPilotMapStatus']
