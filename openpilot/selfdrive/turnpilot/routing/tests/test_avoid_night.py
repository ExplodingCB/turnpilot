"""navd's stop-sign avoidance under TurnPilotStopSignGo=vision (routing/avoid.py run; certify's vision checks on the shipped
file), and what route_notice says of the route it keeps.

Kellen, 2026-11-07: the problems are "our nav system", not openpilot, and "why the hell do we care about dark?". On the 11-06
night drive (1e) every stop navd saw failed the light check ('night at ETA -5 min'); the loop tried 7 stop reroutes and kept
1 (the budget refused 4 at +44 to +61 s, 2 brought more lefts), and the stops it couldn't route round were said "Stop sign
ahead needs you" 6 times. Now a dark stop certifies, and the route-time reach walk is the line's own (certify.vision_lanes):
a right whose far lane the line walks as its cross movement certifies and stays on the route unannounced, while a stop the
line is short at too (a cross road of 2 lanes each way: it waits, creeps and hands back 15 s after its full stop) is still
routed round, and said when it can't be, as is what vision's gate hands back (geometry, here a divided cross road). Re-certified
this way, 2 of the 7 stops the 11-06 routes kept certify and 5 are short of reach at their line."""
import math

import pytest

from openpilot.selfdrive.turnpilot.route_notice import RouteNotices
from openpilot.selfdrive.turnpilot.routing import avoid
from openpilot.selfdrive.turnpilot.routing import certify as ce
from openpilot.selfdrive.turnpilot.routing.controls import Control
from openpilot.selfdrive.turnpilot.routing.junctions import Approach, Junction
from openpilot.selfdrive.turnpilot.routing.tests.test_certify import ETA, LAT, LON, NODE
from openpilot.selfdrive.turnpilot.stop_rules import MPH
from openpilot.selfdrive.turnpilot import visibility_table as vt
from openpilot.selfdrive.turnpilot.tests.test_route_notice import nav, said

SHIPPED = vt.load(vt.SHIPPED)
NIGHT = ETA + 8 * 3600.  # 23:00 EDT
VISION = ce.Runtime(mode='vision', side_views=True)
STEPS = [{'distance': 400., 'duration': 60.}, {'distance': 600., 'duration': 90.}]
WHEN = {'day': ETA, 'night': NIGHT}


def stop(n, route_m, mph=25, maneuver='straight', divided=False, left=1, right=1, rail=-1.):
  """A stop on the shipped files as the car meets it: no survey, the map's tiers (an all-way), each approach mph; the exit leg
  by the maneuver; left (right) lanes each way on the cross road's left (right) leg."""
  exit_role = {'straight': 'opposite', 'right': 'right', 'left': 'left'}[maneuver]
  legs = [(270., -90., 'left', 11, left), (90., 90., 'right', 12, right), (0., 0., 'opposite', 13, 1)]
  info = Junction(osm_node_id=NODE + n, lat=LAT, lon=LON, stop_point_distance=13., rail_distance=rail, surveyed=False, sidecar=True,
                  divided=divided, maneuver=maneuver, approaches=[
                    Approach(b, rel, 'exit' if role == exit_role else role, 'residential', 'road', True, True, lanes, lanes, 'stop', 'stop',
                             'mapStrong', mph * MPH, mph * MPH, 'tagged', way) for b, rel, role, way, lanes in legs])
  return Control(n, 'stop', route_m, 0, route_m, junction_info=info, approach_key=(500 + n, True, (0.0001 * n, 0.001)),
                 exclusion=(0.0001 * n, 0.0009, 90. + n))


class Router:
  """reroute(exclude) -> the route around (the same steps, the excluded stops gone; None: no route), as navd's."""
  def __init__(self, controls, around=True):
    self.controls, self.around, self.calls = controls, around, []

  def reroute(self, exclude):
    self.calls.append(exclude)
    return [dict(s) for s in STEPS] if self.around else None

  def route_controls(self, steps):
    sent = {round(p['lat'], 6) for p in self.calls[-1]} if self.calls else set()
    return [c for c in self.controls if round(c.exclusion[0], 6) not in sent], 'ok'


def run(controls, eta, around=True, runtime=VISION):
  router = Router(controls, around)

  def certify(c, steps):
    return ce.certify_control(c, SHIPPED, [], eta, runtime)
  r = avoid.run([dict(s) for s in STEPS], list(controls), 'ok', avoid.AvoidSet(), certify=certify, reroute=router.reroute,
                route_controls=router.route_controls, own_edge=lambda: None, cancelled=lambda: False, deadline=math.inf)
  return r, router


def notices(result, distances):
  """What route_notice says at route start and on the way (each control at its distance), from navd's instruction."""
  ctl = [(c.id, c.kind, distances[c.id], c.certification, c.junction_info.osm_node_id) for c in result.controls]
  return said(RouteNotices(), round(8. / 0.05), nav(status=result.status, avoided=result.avoided, added=result.added_s, controls=ctl))


def test_a_night_stop_under_vision_certifies_stays_on_the_route_and_is_never_said():
  # before 2026-11-07: uncertifiable 'light' (night), excluded, the reroute over budget or none, unavoidable, "Stop sign
  # ahead needs you" 300 m before it and at route start "1 stop sign on this route needs you"
  a = stop(1, 250.)
  r, router = run([a], NIGHT, around=False)
  assert (a.certification, a.cert_reason) == ('certifiable', 'none'), a.junction_info.cert_detail
  assert router.calls == [] and (r.status, r.avoided, r.iterations) == ('clean', 0, 0)
  assert notices(r, {1: 250.}) == []


@pytest.mark.parametrize('when', list(WHEN))
@pytest.mark.parametrize('mph', [20, 25])
def test_a_right_whose_far_lane_the_line_walks_as_its_cross_movement_certifies_and_is_never_routed_round_or_said(mph, when):
  # the route-time walk judged the far lane's swing zone too (23.8 m of 26.7-37.6 m: 'reach'), so navd routed round a right the
  # line drives (it walks that lane as its cross movement under 15 deg of yaw and goes 2.5 s after its full stop:
  # tests/test_turnpilotd_stop_night.py) and, when it couldn't, said "Stop sign ahead needs you"
  a = stop(1, 250., mph=mph, maneuver='right')
  r, router = run([a], WHEN[when], around=False)
  assert (a.certification, a.cert_reason) == ('certifiable', 'none'), a.junction_info.cert_detail
  assert router.calls == [] and (r.status, r.avoided) == ('clean', 0)
  assert notices(r, {1: 250.}) == []


@pytest.mark.parametrize('when', list(WHEN))
@pytest.mark.parametrize('maneuver', ['straight', 'right'])
def test_a_stop_short_of_reach_on_the_route_time_walk_is_driven_never_routed_round_or_said(maneuver, when):
  # a 25 mph stop on a cross road of 2 lanes each way is short on the lanes the line walks (the near lane 17.2 of 37.6 m). The
  # line drives it all the same (the full stop, the creep for a view, the go on what the cameras see: stop_go._vision_gate
  # hands back for no such thing), so it certifies, stays on the route and nothing is said, the short lanes in its detail
  a = stop(1, 250., maneuver=maneuver, left=2, right=2)
  r, router = run([a], WHEN[when], around=False)
  assert (a.certification, a.cert_reason) == ('certifiable', 'none') and 'left lane 1 cross' in a.junction_info.cert_detail
  assert router.calls == [] and (r.avoided, r.status) == (0, 'clean')
  assert notices(r, {1: 250.}) == []


def test_only_what_vision_hands_back_at_its_line_is_routed_round_at_night_and_said_when_it_cant_be():
  # a mapped rail crossing is the one junction the line hands back at; a divided cross road is driven
  keep, divided, rail = stop(1, 150.), stop(2, 300., divided=True), stop(3, 350., rail=12.)
  r, router = run([keep, divided, rail], NIGHT)
  assert (rail.cert_reason, keep.certification, divided.certification) == ('geometry', 'certifiable', 'certifiable')
  assert len(router.calls) == 1 and len(router.calls[0]) == 1 and r.avoided == 1 and r.status == 'clean'
  keep, divided, rail = stop(1, 150.), stop(2, 200., divided=True), stop(3, 250., rail=12.)
  r, router = run([keep, divided, rail], NIGHT, around=False)
  assert rail.certification == 'unavoidable' and keep.certification == divided.certification == 'certifiable'
  out = notices(r, {1: 150., 2: 200., 3: 250.})
  assert ('stopsNeedYou', "1 stop sign on this route needs you") in out
  assert [k for k, _ in out].count('stopAheadNeedsYou') == 1


def test_the_11_06_shape_at_night_every_stop_is_driven_none_routed_round_or_said():
  # 1e's kept stops were all rights at 25 mph stops, 5 of the 7 short at their line on a far or third cross lane (here 3 lanes
  # from the left: its third lane short). The line creeps for those and goes, so all stay on the route with nothing said
  stops = [stop(1, 200.), stop(2, 450., maneuver='right'), stop(3, 700., maneuver='right', mph=20),
           stop(4, 900., maneuver='right', left=3)]
  r, router = run(stops, NIGHT)
  assert [c.certification for c in stops] == ['certifiable'] * 4
  assert 'left lane 3 cross' in stops[3].junction_info.cert_detail
  assert router.calls == [] and r.avoided == 0 and r.status == 'clean'
  assert notices(r, {1: 200., 2: 450., 3: 700., 4: 900.}) == []
