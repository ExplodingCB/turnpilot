"""left_sites (LEFT_RANGE.md): junctions near a point where a left across one oncoming approach would go from its line for a
given TurnPilotLeftRangeM (one that fails hands back there). judge() is route time's reach on a route's steps, its opposing road and its
controls: the left's reach against the range (and the far crop's), the opposing road straight past it, its speed and lanes
or the unknown rules, no signal, stop or yield of its own, a turn and not a keep, its side not in doubt, and the controls
known. The junctions come from Valhalla's locate (its graph, the one navd routes on): faked here."""
import math

import pytest

from openpilot.selfdrive.turnpilot.map_controls import PROTECT_BEFORE_M
from openpilot.selfdrive.turnpilot.routing.controls import Control
from openpilot.selfdrive.turnpilot.unprotected_left import DEFAULT_OPPOSING_SPEED, LINE_D, ROUTE_REACH_MARGIN_M, reach_at_line
from openpilot.tools.turnpilot import left_sites as ls

LAT0, LON0 = 39.80, -86.10
M_PER_DEG = 111195.
MPH = 0.44704
APPROACH = 200.  # m of the route before the junction


def turn_steps(modifier='left', bearings=(0, 90, 180, 270), i_in=2, out=3, before=0., after=270., kind='turn'):
  """depart northbound APPROACH m, then the maneuver at the junction (LAT0, LON0), then arrive: OSRM's shape of Valhalla's."""
  loc = [LON0, LAT0]
  return [
    {'distance': APPROACH, 'name': 'Main Street', 'maneuver': {'type': 'depart', 'bearing_after': before, 'location': [LON0, LAT0 - 0.0018]},
     'intersections': [{'location': [LON0, LAT0 - 0.0018], 'bearings': [int(before)], 'out': 0}],
     'geometry': {'coordinates': [[LON0, LAT0 - APPROACH / M_PER_DEG], loc]}},
    {'distance': 60., 'name': 'Oak Street', 'maneuver': {'type': kind, 'modifier': modifier, 'bearing_before': before, 'bearing_after': after,
                                                         'location': loc},
     'intersections': [{'location': loc, 'bearings': list(bearings), 'in': i_in, 'out': out}],
     'geometry': {'coordinates': [loc, [LON0 - 0.0007, LAT0]]}},
    {'distance': 0., 'name': 'Oak Street', 'maneuver': {'type': 'arrive', 'location': [LON0 - 0.0007, LAT0]}, 'intersections': [],
     'geometry': {'coordinates': [[LON0 - 0.0007, LAT0]]}},
  ]


def control(kind, before_m=11., **kw):
  """A control of ours before_m short of the junction (navd's default stop point: the node less 10 m, less MAP_MARGIN_M)."""
  return Control(0, kind, APPROACH - before_m, 0, APPROACH - before_m, **kw)


ROAD_25 = {0: (25 * MPH, 1, 400.)}  # the opposing road: 25 mph mapped, one lane, straight 400 m on


def test_a_quiet_four_way_left_passes_with_its_margin():
  """25 mph and one lane mapped: route time's R_req, at LINE_D plus ROUTE_REACH_MARGIN_M (no mapped stop line), is about
  200 m, so it passes under 210 m (and not under DESIGN's 134-141 m, which were at LINE_D alone)."""
  reach = reach_at_line(25 * MPH, 1, ROUTE_REACH_MARGIN_M)
  assert 195. < reach < 205.
  v = ls.judge(turn_steps(), ROAD_25, [], 'ok', 210.)
  assert v.ok and v.reasons == [] and v.reach == pytest.approx(reach)
  assert v.margin == pytest.approx(min(210. - reach, LINE_D + ROUTE_REACH_MARGIN_M + 400. - reach))


def test_a_reach_past_the_range_fails_as_route_time_skips_it():
  v = ls.judge(turn_steps(), ROAD_25, [], 'ok', 150.)
  assert not v.ok and v.codes == ['range']


def test_the_opposing_road_must_be_mapped_straight_past_the_reach():
  v = ls.judge(turn_steps(), {0: (25 * MPH, 1, 50.)}, [], 'ok', 400.)
  assert not v.ok and v.codes == ['curve']
  v = ls.judge(turn_steps(), {0: (25 * MPH, 1, 0.)}, [], 'ok', 400.)  # unknown is not straight
  assert v.codes == ['curve']


def test_an_unmapped_limit_is_45_mph_and_unknown_lanes_are_two():
  v = ls.judge(turn_steps(), {0: (0., 1, 400.)}, [], 'ok', 400.)
  assert v.reach == pytest.approx(reach_at_line(DEFAULT_OPPOSING_SPEED, 1, ROUTE_REACH_MARGIN_M))
  assert any('45 mph' in r for r in v.notes)
  v = ls.judge(turn_steps(), {0: (25 * MPH, 0, 400.)}, [], 'ok', 400.)
  assert v.reach == pytest.approx(reach_at_line(25 * MPH, None, ROUTE_REACH_MARGIN_M))
  assert v.reach > reach_at_line(25 * MPH, 1, ROUTE_REACH_MARGIN_M)
  assert any('lanes' in r for r in v.notes)
  v = ls.judge(turn_steps(), {}, [], 'ok', 400.)  # no opposing road found at all: both unknown, and not straight
  assert v.codes == ['curve']


def test_a_signal_stop_or_yield_of_its_own_fails_and_one_further_back_does_not():
  for kind in ('signal', 'stop', 'yield'):
    v = ls.judge(turn_steps(), ROAD_25, [control(kind)], 'ok', 400.)
    assert not v.ok and v.codes == [kind]
  v = ls.judge(turn_steps(), ROAD_25, [control('signal', before_m=PROTECT_BEFORE_M + 20.)], 'ok', 400.)
  assert v.ok
  # the cross road's yield (the sidecar confirms it governs a crossing approach): not ours
  v = ls.judge(turn_steps(), ROAD_25, [control('yield', cross_suspect=True, junction_info={'crossGoverned': True})], 'ok', 400.)
  assert v.ok


def test_navds_own_junctions_are_read_as_turnpilotd_receives_them():
  """route_controls' Control carries a routing/junctions.py Junction; turnpilotd gets it as the message's dict, which
  map_controls' cross_road_yield and rail_between read: a cross road's yield is not ours, a rail orphan's stop short of the
  tracks is not the junction's, and a plain stop of ours is."""
  from openpilot.selfdrive.turnpilot.routing.junctions import Junction
  cross = control('yield', cross_suspect=True, junction_info=Junction(cross_governed=True))
  assert ls.judge(turn_steps(), ROAD_25, [cross], 'ok', 400.).ok
  rail = control('stop', junction_info=Junction(rail_distance=5., stop_point_distance=20.))
  assert ls.judge(turn_steps(), ROAD_25, [rail], 'ok', 400.).ok
  assert ls.judge(turn_steps(), ROAD_25, [control('stop', junction_info=Junction())], 'ok', 400.).codes == ['stop']


def test_controls_unknown_fail_and_partial_passes_said():
  for status in ('unsupported', 'failed'):
    v = ls.judge(turn_steps(), ROAD_25, [], status, 400.)
    assert not v.ok and v.codes == ['controls']
  v = ls.judge(turn_steps(), ROAD_25, [], 'partial', 400.)
  assert v.ok and any('stop' in n and 'by eye' in n for n in v.notes)


def test_the_far_crop_extends_the_reach_only_past_a_measured_road_range():
  reach = reach_at_line(35 * MPH, 1, ROUTE_REACH_MARGIN_M)
  road = {0: (35 * MPH, 1, 400.)}
  assert not ls.judge(turn_steps(), road, [], 'ok', 120.).ok
  assert ls.judge(turn_steps(), road, [], 'ok', 120., far_m=reach + 5.).ok
  assert ls.judge(turn_steps(), road, [], 'ok', 0., far_m=reach + 5.).codes == ['range']


def test_only_a_left_turn_across_one_oncoming_approach():
  # a T's stem: no road straight on, so no oncoming traffic to judge (route time's unprotected_turn is false)
  assert ls.judge(turn_steps(bearings=(90, 180, 270), i_in=1, out=2), ROAD_25, [], 'ok', 400.).codes == ['notUnprotected']
  # a slight left of SLIGHT_TURN_MIN_DEG (25) or more across the road straight on is a left across oncoming traffic, as
  # turnpilotd takes it since the 11-06 drives (route_follower.slight_far_turn): judged as one; a gentler one is a keep
  assert ls.judge(turn_steps(modifier='slight left', after=330.), ROAD_25, [], 'ok', 400.).ok
  assert ls.judge(turn_steps(modifier='slight left', after=340.), ROAD_25, [], 'ok', 400.).codes == ['notUnprotected']
  assert ls.judge(turn_steps(modifier='right', out=1, after=90.), ROAD_25, [], 'ok', 400.).codes == ['notUnprotected']
  # two roads within OPPOSING_DEG of straight on: which one the band looks up isn't one approach
  assert ls.judge(turn_steps(bearings=(350, 30, 90, 180, 270), i_in=3, out=4), ROAD_25, [], 'ok', 400.).codes == ['twoOncoming']
  # the bearings turn right under a left modifier: turnpilotd skips it, sideUncertain
  assert ls.judge(turn_steps(after=90., out=1), ROAD_25, [], 'ok', 400.).codes == ['sideUncertain']


def test_every_failing_check_is_said():
  v = ls.judge(turn_steps(), {0: (35 * MPH, 1, 50.)}, [control('signal')], 'ok', 100.)
  assert v.codes == ['signal', 'range', 'curve'] and len(v.reasons) == 3


# --- junctions from Valhalla's locate (faked) ---

def encode6(pts):
  out, plat, plon = [], 0, 0
  for lat, lon in pts:
    for v, p in ((round(lat * 1e6), plat), (round(lon * 1e6), plon)):
      d = v - p
      d = ~(d << 1) if d < 0 else d << 1
      while d >= 0x20:
        out.append(chr((0x20 | (d & 0x1f)) + 63))
        d >>= 5
      out.append(chr(d + 63))
    plat, plon = round(lat * 1e6), round(lon * 1e6)
  return ''.join(out)


def at(east_m, north_m):
  return LAT0 + north_m / M_PER_DEG, LON0 + east_m / (M_PER_DEG * math.cos(math.radians(LAT0)))


def edge(way, pts, forward=True, cls='residential', limit=40, end_id=0):
  return {'edge_info': {'way_id': way, 'shape': encode6(pts), 'names': [f'way {way}'], 'speed_limit': limit},
          'edge': {'forward': forward, 'classification': {'classification': cls, 'use': 'road'}, 'end_node': {'value': end_id}},
          'heading': 0., 'distance': 1.}


def both(way, pts, **kw):
  return [edge(way, pts, True, **kw), edge(way, pts, False, **kw)]


def test_junctions_and_their_lefts_from_locate():
  """A four-way (two two-way roads crossing), a T to its east, a bend (two roads meeting, no junction) and a one-way: lefts
  are taken only where a road comes in, another leaves to the left and one goes on straight across."""
  c, tee, bend = at(0, 0), at(300, 0), at(0, 300)
  answers = [{'edges': both(1, [at(0, -150), c]) + both(2, [c, at(0, 150)]) + both(3, [at(-150, 0), c]) + both(4, [c, tee])},
             {'edges': both(5, [tee, at(450, 0)]) + both(6, [tee, at(300, -150)])},
             {'edges': both(7, [at(0, 150), bend]) + both(8, [bend, at(100, 300)])},
             {'edges': [edge(9, [at(-150, 300), at(-150, 150)], True)]}]
  nodes = ls.nodes_from_locate(answers)
  junctions = ls.junctions(nodes)
  assert sorted((round(j.lat, 5), round(j.lon, 5)) for j in junctions) == sorted([(round(c[0], 5), round(c[1], 5)),
                                                                                   (round(tee[0], 5), round(tee[1], 5))])
  four = next(j for j in junctions if abs(j.lon - c[1]) < 1e-6)
  lefts = ls.left_candidates(four)
  assert len(lefts) == 4  # one from each approach, each with a road straight across
  assert all(ls.turn_angle_deg(a, b) < 0 for a, b in lefts)
  t = next(j for j in junctions if abs(j.lon - tee[1]) < 1e-6)
  lefts = ls.left_candidates(t)
  # from the west: left is north, no road there; from the east (way 5 coming in westbound): left is south into the stem, the
  # through road straight on; from the stem: no road straight on
  assert [(a.way_id, b.way_id) for a, b in lefts] == [(5, 6)]


def test_a_one_way_road_is_never_taken_against_its_way():
  c = at(0, 0)
  answers = [{'edges': [edge(1, [at(0, -150), c], True)] + both(2, [c, at(0, 150)]) + [edge(3, [c, at(-150, 0)], True)] +
              both(4, [c, at(150, 0)])}]
  four = ls.junctions(ls.nodes_from_locate(answers))[0]
  lefts = {(a.way_id, b.way_id) for a, b in ls.left_candidates(four)}
  # way 1 northbound only (it may come in, never leave); way 3 westbound away from the junction only (leave, never come in)
  assert (1, 3) in lefts and not any(b == 1 for _, b in lefts) and not any(a == 3 for a, _ in lefts)


def test_sites_rank_by_margin():
  sites = [ls.Site(at(0, 0), 'Alder Road', 'Birch Road', ls.Verdict(ok=True, margin=5.)),
           ls.Site(at(10, 0), 'Cedar Road', 'Dogwood Road', ls.Verdict(ok=True, margin=30.)),
           ls.Site(at(20, 0), 'Elm Road', 'Fir Road', ls.Verdict(ok=False, margin=50., codes=['signal'], reasons=['a traffic signal']))]
  ranked = ls.ranked(sites)
  assert [s.from_name for s in ranked] == ['Cedar Road', 'Alder Road', 'Elm Road']
  text = '\n'.join(ls.render(ranked, 150., 0.))
  assert text.index('Cedar Road') < text.index('Alder Road') < text.index('Elm Road') and 'a traffic signal' in text
