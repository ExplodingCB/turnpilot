"""navd through a GPS loss (navigation/hold.py): the first real drive lost GPS standing at a red before a left, and TurnPilot
dropped back to plain openpilot. navd took any fix with hasFix set: a degraded one at a standstill wandered off the step (an
invalid instruction, the uncommitted left dropped), rerouted from the wandering spot, or moved navd past the left with the
car still at the line; with no fix the position froze while the car drove the turn. These drive navd itself, at its 1 Hz,
on navd_loop's synthetic route (east to a left at JUNCTION_M, north to a right, east to the end)."""
from openpilot.selfdrive.navigation.hold import (DRIFT, GPS_ACCURACY_MAX_M, HOLD_MAX_M, HOLD_MAX_S, MISMATCH_M, REACQUIRE_FIXES,
                                                 UNCERTAINTY_MIN_M, Motion, PositionHold, RoutePath)
from openpilot.selfdrive.turnpilot.tests.navd_loop import JUNCTION_M, NORTH_M, NavdLoop, place, route_steps

import numpy as np

STOP_M = JUNCTION_M - 8.  # first in line at the red, 8 m before the left's node
# a receiver losing its fix standing at the light: fixes reporting accuracy past GPS_ACCURACY_MAX_M wandering tens of metres,
# then none, then fixes claiming a good accuracy still metres off (multipath, the interference ebbing)
WANDERING = [(35., 0., 45.), (-30., 20., 60.), (0., 35., 45.), (25., -25., 80.), (40., 10., 45.), (-10., -35., 50.)]
CLAIMING_GOOD = [(20., -15., 8.), (18., 22., 6.), (-22., 5., 9.), (14., 30., 8.)]


def approach(loop, until_s, v=10.):
  """Driven at v on good GPS from the start to until_s, then stopped there: (s, the last instruction)."""
  s, out = 0., None
  while s < until_s - v:
    out = loop.tick(s, v)
    s += v
  out = loop.tick(until_s, 0.)
  return until_s, out


def pull_away(loop, s, until_s, fix=None, v_max=8.):
  """From standing at s, accelerating 2 m/s^2 to v_max on to until_s, fix as each tick takes it: [(s, instruction, valid)].
  s runs as navd's own trapezoid does, so a hold carries the car exactly where it is."""
  out, v = [], 0.
  while s < until_s:
    v_next = min(v + 2., v_max)
    s += (v + v_next) / 2.
    v = v_next
    out.append((s,) + loop.tick(s, v, fix=fix(s) if callable(fix) else fix))
  return out


def test_gps_lost_standing_before_a_left_keeps_the_route_and_carries_the_turn_after_the_light():
  """The first drive's dropout: standing at the red before the left, the fix wanders, goes, comes back claiming a good
  accuracy but metres off. Every instruction stays valid on the same route with the left still 8 m on, nothing reroutes,
  and nothing moves navd on. Green with the GPS still out: the car's own travel carries navd round the left onto its next
  step, and the fixes coming back take over after REACQUIRE_FIXES with no jump and no reroute."""
  loop = NavdLoop()
  s, (before, valid) = approach(loop, STOP_M)
  assert valid and before.maneuverModifier == 'left' and abs(before.maneuverDistance - 8.) < 0.5
  route_count = before.routeCount
  stand = [loop.tick(s, 0., fix=f) for f in WANDERING] + [loop.tick(s, 0., fix=None) for _ in range(30)] + \
          [loop.tick(s, 0., fix=f) for f in CLAIMING_GOOD]
  assert all(valid for _, valid in stand)
  assert {(ni.routeCount, ni.maneuverModifier, str(ni.positionSource)) for ni, _ in stand} == {(route_count, 'left', 'deadReckoned')}
  assert all(abs(ni.maneuverDistance - 8.) < 0.5 for ni, _ in stand)  # standing: the place never moves
  assert all(ni.positionUncertainty == stand[0][0].positionUncertainty for ni, _ in stand)  # nor grows less sure
  assert stand[-1][0].gpsLostS == len(stand)
  assert loop.engine.step_idx == 0 and len(loop.router.calls) == 1

  # green, the GPS still out through the left: carried along the route, onto the left's step at its node
  turn = pull_away(loop, s, JUNCTION_M + 40.)
  assert all(valid for _, _, valid in turn) and {ni.routeCount for _, ni, _ in turn} == {route_count}
  assert all(str(ni.positionSource) == 'deadReckoned' for _, ni, _ in turn)
  for s_car, ni, _ in turn:
    if s_car < JUNCTION_M:
      assert ni.maneuverModifier == 'left' and abs(ni.maneuverDistance - (JUNCTION_M - s_car)) < 0.5
    elif s_car > JUNCTION_M + 10.:
      assert ni.maneuverModifier == 'right' and abs(ni.maneuverDistance - (JUNCTION_M + NORTH_M - s_car)) < 0.5
  assert loop.engine.step_idx == 1

  # the fixes come back: REACQUIRE_FIXES of them, then navd is on GPS again where the hold had it
  s = turn[-1][0]
  back = [loop.tick(s + 8. * (i + 1), 8.) for i in range(REACQUIRE_FIXES + 1)]
  assert [str(ni.positionSource) for ni, _ in back] == ['deadReckoned'] * (REACQUIRE_FIXES - 1) + ['gps', 'gps']
  assert all(valid for _, valid in back) and back[-1][0].gpsLostS == 0. and back[-1][0].positionUncertainty == UNCERTAINTY_MIN_M
  assert len(loop.router.calls) == 1 and 'NavDestination' in loop.values and loop.engine.step_idx == 1


def test_a_wandering_fix_standing_never_reroutes_or_moves_navd_on():
  """hasFix set the whole time at the light, wandering 20-40 m: never off the route, past the left, or a reroute, however
  long it stands. (Before the hold: 33 m off the step was an invalid instruction at once, and a reroute four seconds on.)"""
  loop = NavdLoop()
  s, _ = approach(loop, STOP_M)
  stand = [loop.tick(s, 0., fix=WANDERING[i % len(WANDERING)]) for i in range(60)] + \
          [loop.tick(s, 0., fix=CLAIMING_GOOD[i % len(CLAIMING_GOOD)]) for i in range(60)]
  assert all(valid and ni.maneuverModifier == 'left' and abs(ni.maneuverDistance - 8.) < 0.5 for ni, valid in stand)
  assert loop.engine.step_idx == 0 and len(loop.router.calls) == 1 and loop.engine.reroute_counter == 0


def test_good_gps_standing_holds_the_place_and_moving_follows_the_fix():
  """With good GPS nothing changes moving (every fix taken, as before the hold); standing, a fix metres off moves nothing."""
  loop = NavdLoop()
  s, _ = approach(loop, STOP_M)
  ni, valid = loop.tick(s, 0., fix=(12., 0., 3.))  # 12 m on, a good accuracy: standing, the car didn't move
  assert valid and str(ni.positionSource) == 'gps' and abs(ni.maneuverDistance - 8.) < 0.5
  ni, valid = loop.tick(s + 2., 2., fix=(5., 0., 3.))  # moving: the fix is the place
  assert abs(ni.maneuverDistance - (8. - 2. - 5.)) < 0.5


def test_moving_without_a_trusted_fix_past_the_holds_limit_is_lost_loudly_with_the_route_kept():
  """GPS out from 100 m on at 15 m/s: carried, valid, its uncertainty growing DRIFT a metre, to HOLD_MAX_M; past it navd's
  position is lost: invalid instructions (positionSource lost, which turnpilotd hands back on), the map's stops ahead
  unknown, never clear; the route and the destination kept. The fixes back, guidance carries on with no reroute."""
  loop = NavdLoop()
  for s in range(0, 100, 15):
    loop.tick(float(s), 15.)
  s, out = 100., []
  while s < 100. + HOLD_MAX_M + 45.:
    s += 15.
    out.append((s,) + loop.tick(s, 15., fix=(0., 0., GPS_ACCURACY_MAX_M + 20.)))  # a fix, but too poor to use
  held = [(s_car, ni) for s_car, ni, valid in out if valid]
  lost = [(s_car, ni) for s_car, ni, valid in out if not valid]
  assert held and lost and max(s_car for s_car, _ in held) < min(s_car for s_car, _ in lost)
  assert all(str(ni.positionSource) == 'deadReckoned' for _, ni in held)
  carried = held[-1][0] - 100.
  assert abs(held[-1][1].positionUncertainty - (UNCERTAINTY_MIN_M + DRIFT * carried)) < 0.5
  assert carried <= HOLD_MAX_M < carried + 15. + 1.
  assert all(str(ni.positionSource) == 'lost' and ni.offRouteControlsStatus == 'failed' for _, ni in lost)
  assert {ni.routeCount for _, ni in held + lost} == {held[0][1].routeCount}
  assert loop.engine.route is not None and 'NavDestination' in loop.values and len(loop.router.calls) == 1
  back = [loop.tick(s + 15. * (i + 1), 15.) for i in range(REACQUIRE_FIXES)]
  assert back[-1][1] and str(back[-1][0].positionSource) == 'gps' and len(loop.router.calls) == 1
  assert abs(back[-1][0].maneuverDistance - (JUNCTION_M + NORTH_M - (s + 15. * REACQUIRE_FIXES))) < 0.5


def test_a_long_crawl_without_gps_is_lost_on_time_and_standing_time_never_counts():
  """Creeping at 1 m/s with no fix: the travel stays short, the hold lasts HOLD_MAX_S of moving. Standing, it never runs out."""
  loop = NavdLoop()
  s, _ = approach(loop, 50.)
  stand = [loop.tick(s, 0., fix=None) for _ in range(int(HOLD_MAX_S) * 3)]
  assert all(valid and str(ni.positionSource) == 'deadReckoned' for ni, valid in stand)
  crawl = []
  for _ in range(int(HOLD_MAX_S) + 2):
    s += 1.
    crawl.append(loop.tick(s, 1., fix=None))
  assert all(valid for _, valid in crawl[:int(HOLD_MAX_S) - 1]) and not crawl[-1][1]
  assert str(crawl[-1][0].positionSource) == 'lost' and loop.engine.hold.why == 'limit'


def test_the_car_straight_past_the_left_while_carried_is_lost_and_rerouted_once_the_fix_is_back():
  """GPS out before the left, and the car goes straight on through the junction: the hold carries navd round the left, the
  car's own heading (deviceMotion) stays east. Off the route's heading MISMATCH_M on, the hold is lost (never guidance down
  a street the car isn't on). The fixes back put it east of the junction, off the route: navd's own reroute."""
  loop = NavdLoop()
  s, _ = approach(loop, 250.)
  out = []
  for _ in range(12):
    s += 8.
    out.append((s,) + loop.tick(s, 8., fix=None, car_heading=90.))
  lost_at = next(s_car for s_car, ni, valid in out if not valid)
  assert str(out[-1][1].positionSource) == 'lost' and loop.engine.hold.why == 'leftRoute'
  assert JUNCTION_M + MISMATCH_M <= lost_at <= JUNCTION_M + MISMATCH_M + 2 * 8.
  assert all(valid for s_car, _, valid in out if s_car < lost_at)
  # back on GPS, east of the junction: off the route, rerouted from where the car really is
  for _ in range(REACQUIRE_FIXES + 8):
    s += 8.
    loop.tick(s, 8., fix=(s - JUNCTION_M, 0. - (s - JUNCTION_M), 3.), car_heading=90.)  # the true place: east of the node
  assert str(loop.engine.hold.source) == 'gps' and len(loop.router.calls) >= 2
  (lon, lat), _ = loop.router.calls[1]
  assert lat == 0. and lon > place(JUNCTION_M + MISMATCH_M)[0]  # from the street the car went straight on along


def test_a_turn_missed_while_carried_is_lost():
  """turnpilotd's turnMissed (the car pulled away from the line straight past its turn) with no fix and no deviceMotion to
  see it by: the hold carried the car round the turn it didn't take, so it is lost at once."""
  loop = NavdLoop(yaw=False)
  s, _ = approach(loop, STOP_M)
  turn = pull_away(loop, s, JUNCTION_M + 20.)
  assert all(valid for _, _, valid in turn)
  ni, valid = loop.tick(turn[-1][0] + 8., 8., fix=None, turn_missed=True)
  assert not valid and str(ni.positionSource) == 'lost' and loop.engine.hold.why == 'turnMissed'


def test_without_car_state_fixes_are_taken_as_before_but_never_a_poor_one():
  """Offroad (no carState) nothing knows whether the car moves: every trusted fix is taken, standing or not, and without one the
  place stays. A fix past GPS_ACCURACY_MAX_M is never taken but as the very first place (navd routes from nothing else)."""
  loop = NavdLoop(motion=False, yaw=False)
  ni, valid = loop.tick(0., 0., fix=(0., 0., 60.))  # the first place: poor, but all there is
  assert valid and loop.engine.last_position == place(0.)
  loop.tick(30., 0., fix='true')
  assert loop.engine.last_position == place(30.)
  loop.tick(60., 0., fix=(0., 0., 60.))
  assert loop.engine.last_position == place(30.) and str(loop.engine.hold.source) == 'deadReckoned'


def test_a_poor_first_place_is_replaced_by_the_first_trusted_fix_even_standing():
  loop = NavdLoop()
  loop.tick(0., 0., fix=(20., 0., 60.))
  assert loop.engine.last_position == place(20.)
  loop.tick(0., 0., fix='true')
  assert loop.engine.last_position == place(0.)
  loop.tick(0., 0., fix=(15., 0., 3.))  # trusted from here: standing, nothing moves it
  assert loop.engine.last_position == place(0.)


def test_route_path_places_and_bearings():
  geometry = [np.array(st['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for st in route_steps()]
  path = RoutePath(geometry)
  assert abs(path.length - 900.) < 0.01
  assert abs(path.along(0, place(120.)) - 120.) < 0.01
  assert abs(path.along(0, place(JUNCTION_M + 10.)) - (JUNCTION_M + 10.)) < 0.01  # 10 m into the next step: on it
  assert abs(path.along(0, place(JUNCTION_M + 3.)) - JUNCTION_M) < 0.01           # 3 m: not yet sure, the step's end
  for s, bearing in ((10., 90.), (JUNCTION_M + 5., 0.), (850., 90.), (900., 90.), (2000., 90.)):  # the arrival's zero length too
    p, b = path.at(s)
    assert np.allclose(p, place(min(s, 900.)), atol=1e-9) and abs(b - bearing) < 1e-6


def test_hold_unit_standing_ignores_fixes_and_moving_carries_and_reacquires():
  hold = PositionHold()
  hold.update(None, Motion(0., None), 0.)
  assert hold.position is None
  from openpilot.selfdrive.navigation.hold import Fix
  hold.update(Fix((0., 0.), 90., 4.), Motion(0., 90.), 1.)
  assert hold.position == (0., 0.) and hold.source == 'gps' and hold.uncertainty_m == 4.
  hold.update(Fix(place(30.), 90., 4.), Motion(0., 90.), 2.)
  assert hold.position == (0., 0.)
  geometry = [np.array(st['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for st in route_steps()]
  hold.update(None, Motion(10., 90.), 3., geometry, 0)
  assert hold.source == 'deadReckoned' and np.allclose(hold.position, place(5.), atol=1e-9)  # (0 + 10) / 2 m in the second
  hold.update(None, Motion(10., 90.), 4., geometry, 0)
  assert np.allclose(hold.position, place(15.), atol=1e-9) and abs(hold.uncertainty_m - (4. + DRIFT * 15.)) < 1e-6
  hold.update(Fix(place(26.), 90., 4.), Motion(10., 90.), 5., geometry, 0)
  assert hold.source == 'deadReckoned' and np.allclose(hold.position, place(25.), atol=1e-9)
  hold.update(Fix(place(36.), 90., 4.), Motion(10., 90.), 6., geometry, 0)
  assert hold.source == 'gps' and hold.position == place(36.)


def test_navd_routes_with_a_heading_only_from_a_moving_fix_with_a_good_bearing_accuracy():
  """The route request's start heading (Valhalla's 90 deg either side): a standing or just-reacquired u-blox heading of motion
  is stale or 0, which put the start on the wrong edge after a dropout. Only a fix moving HEADING_MIN_V or faster reporting a
  bearing accuracy within HEADING_ACCURACY_MAX_DEG sets it; standing the last good one stands, it turns with the car's yaw,
  and with neither navd sends no heading at all."""
  from openpilot.selfdrive.navigation.hold import HEADING_ACCURACY_MAX_DEG, HEADING_MIN_V, HEADING_STALE_M, Fix
  loop = NavdLoop()
  loop.tick(0., 0., fix=(0., 0., 3.))  # navd starts standing: the u-blox heading is a stale 90 here, with no accuracy
  assert loop.router.calls[0][1] is None  # no heading: Valhalla picks the start edge by distance
  # moving, with a good bearing accuracy: that bearing; standing again: it stands, whatever the receiver says
  hold = PositionHold()
  hold.update(Fix((0., 0.), 37., 3., 10., 5.), Motion(10., 37.), 0.)
  assert hold.heading == 37.
  hold.update(Fix((0., 0.), 0., 3., 0., 5.), Motion(0., 37.), 1.)  # standing: a heading of motion of 0 means nothing
  hold.update(Fix((0., 0.), 300., 3., HEADING_MIN_V, HEADING_ACCURACY_MAX_DEG + 1.), Motion(HEADING_MIN_V, 37.), 2.)
  assert hold.heading == 37.
  hold.update(Fix((0., 0.), 300., 3., HEADING_MIN_V - 0.1, 2.), Motion(HEADING_MIN_V - 0.1, 37.), 3.)  # too slow
  assert hold.heading == 37.
  hold.update(None, Motion(5., 127.), 4.)  # the GPS gone, the car turned 90 deg right: so did its heading
  assert hold.heading == 127.
  hold.update(None, Motion(10., None), 5.)  # no yaw: the last good one, until HEADING_STALE_M of travel
  assert hold.heading == 127.
  for t in range(6, 6 + int(HEADING_STALE_M / 10.) + 1):
    hold.update(None, Motion(10., None), float(t))
  assert hold.heading is None
  # navd after a dropout: the first fixes back while standing give no heading; the reroute takes the last good one
  loop = NavdLoop()
  for i in range(10):
    loop.tick(10. * i, 10.)
  assert loop.engine.route_heading() == 90.
  loop.tick(100., 0., fix=None)
  loop.tick(100., 0., fix=(0., 0., 3.))
  assert loop.engine.route_heading() == 90. and loop.engine.position()[1] == 90.
