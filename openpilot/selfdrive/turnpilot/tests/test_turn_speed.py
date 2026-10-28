import math

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.desire_timing import TURN_DESIRE_LEAD_M
from openpilot.selfdrive.turnpilot.route_follower import NOMINAL_ANGLE_DEG, Maneuver, classify
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.scene_policy import NO_ACCEL_CAP
from openpilot.selfdrive.turnpilot.turn_speed import (CORNER_M, LEFT_GO_PROFILE, NO_SPEED_CAP, POST_GO_DEFAULT, POST_GO_MAX_M,
                                                      POST_GO_PROFILES, V_CORNER, V_TURN_MAX, V_TURN_MIN, PostGo, post_go_caps,
                                                      turn_speed_cap, turn_target_speed)


def maneuver(mtype, modifier):
  kind, side = classify(mtype, modifier)
  return Maneuver(kind, side, mtype, modifier, NOMINAL_ANGLE_DEG.get(modifier, 0.))


def test_target_speeds():
  # every junction turn goes at gate 1's speed; only a slight turn across the road is wider and faster
  for angle in (90., 135.):
    for curb_side in (True, False):
      assert math.isclose(turn_target_speed(angle, curb_side), V_TURN_MIN)
  assert V_TURN_MIN < turn_target_speed(45., curb_side=False) < V_TURN_MAX
  assert turn_target_speed(5., curb_side=True) == V_TURN_MAX


def test_which_maneuvers_are_capped():
  assert turn_speed_cap(maneuver("turn", "right"), 0., 4.) == V_CORNER
  assert turn_speed_cap(maneuver("turn", "slight left"), 0., 4.) < 10.
  assert turn_speed_cap(maneuver("off ramp", "slight right"), 0., 25.) == NO_SPEED_CAP
  assert turn_speed_cap(maneuver("fork", "left"), 0., 25.) == NO_SPEED_CAP
  assert turn_speed_cap(maneuver("turn", "uturn"), 0., 4.) == NO_SPEED_CAP
  assert turn_speed_cap(None, 0., 4.) == NO_SPEED_CAP
  # traffic keeping left: the right turn is the wide one
  assert turn_speed_cap(maneuver("turn", "slight right"), 0., 4., right_hand_traffic=False) > \
         turn_speed_cap(maneuver("turn", "slight right"), 0., 4.)


def test_no_cap_where_the_highway_bends():
  # "continue slight right" at 30 m/s braked the car at 2 m/s^2 toward 6.9 m/s from 300 m out
  for mtype, modifier in (("continue", "slight right"), ("new name", "slight left"), ("turn", "slight right")):
    for d in (300., 100., 20., 0.):
      assert turn_speed_cap(maneuver(mtype, modifier), d, 30.) == NO_SPEED_CAP, (mtype, modifier, d)
  # a slight turn in town is still capped, and a junction turn is capped at any speed
  assert turn_speed_cap(maneuver("turn", "slight right"), 20., 14.) < 10.
  assert turn_speed_cap(maneuver("turn", "right"), 100., 30.) < 15.


def test_braking_curve():
  m = maneuver("turn", "right")
  assert math.isclose(turn_speed_cap(m, 100. + TURN_DESIRE_LEAD_M, 0.), math.sqrt(V_TURN_MIN ** 2 + 2 * 1.5 * 100.))
  assert turn_speed_cap(m, 300., 15.) > 25.  # far out it doesn't bite


def test_car_reaches_turn_speed_by_the_desire():
  for modifier in ("right", "left"):
    m = maneuver("turn", modifier)
    d, v, min_a = 300., 15., 0.
    at_desire = None
    while d > CORNER_M:
      a = min(plan_accel(10., turn_speed_cap(m, d, v), v), 0.)  # the car would hold its speed otherwise
      min_a = min(min_a, a)
      v = max(v + a * DT_MDL, 0.)
      d -= v * DT_MDL
      if at_desire is None and d < TURN_DESIRE_LEAD_M:
        at_desire = v
    assert V_TURN_MIN - 0.5 < at_desire < V_TURN_MIN + 0.25, modifier  # gate 1's speed when the desire goes out
    assert v < V_CORNER + 0.5, modifier                                  # and its own turn speed by the corner
    assert min_a > -1.6, modifier


def test_corner_curve_at_real_angles():
  # a right whose bearings turn 70 degrees still slows to V_CORNER by CORNER_M, as the nominal 90 did; a slight keep doesn't
  for angle in (70., 85., 90., 120.):
    real = Maneuver(*classify("turn", "right"), "turn", "right", angle, geometry=True)
    assert turn_speed_cap(real, CORNER_M, 4.) == V_CORNER, angle
  keep = Maneuver(*classify("turn", "slight right"), "turn", "slight right", 30., geometry=True)
  assert turn_speed_cap(keep, CORNER_M, 4.) > V_CORNER


def test_post_go_cap_shape_by_angle():
  """DESIGN phase 4 section 3, the graduated profile: from the go, accel capped at 1.0 m/s^2 until 45 deg turned; speed
  capped at 3.0 m/s until 15 deg, ramping linearly to 4.3 at 45 deg, then blended linearly into the turn's own corner cap by
  60 deg, where the profile ends."""
  g = POST_GO_PROFILES['graduated']
  corner = 5.5  # any corner cap: the blend's far end
  for deg, speed in ((-5., 3.0), (0., 3.0), (10., 3.0), (15., 3.0), (30., 3.65), (44.99, 4.3), (45., 4.3), (52.5, 4.9),
                     (59.99, 5.5)):
    accel, cap = post_go_caps(g, deg, corner)
    assert math.isclose(cap, speed, abs_tol=1e-3), deg
    assert accel == (1.0 if deg < 45. else NO_ACCEL_CAP), deg
  assert post_go_caps(g, 60., corner) == post_go_caps(g, 90., corner) == (NO_ACCEL_CAP, NO_SPEED_CAP)
  # a corner cap under the profile's end: the blend comes down to it, never under it
  assert math.isclose(post_go_caps(g, 52.5, 3.5)[1], 3.9, abs_tol=1e-3)


def test_post_go_profiles_for_the_sweep():
  """The four standstill variants of the BeamNG sweep (V1 tapless: no cap; flat 3.0 to 45 deg; graduated; flat 4.3, the
  capped ones with the 1.0 accel cap) and the unprotected left's own (1.5 m/s^2, 4.3 flat to 45 deg), each by name, the
  default the graduated one."""
  assert POST_GO_DEFAULT == 'graduated' and LEFT_GO_PROFILE == 'left'
  assert set(POST_GO_PROFILES) == {'none', 'flat3', 'graduated', 'flat43', 'left'}
  at = {name: [post_go_caps(p, deg, V_CORNER) for deg in (0., 30., 50.)] for name, p in POST_GO_PROFILES.items()}
  assert at['none'] == [(NO_ACCEL_CAP, NO_SPEED_CAP)] * 3
  assert [c for _, c in at['flat3'][:2]] == [3.0, 3.0] and [c for _, c in at['flat43'][:2]] == [4.3, 4.3]
  assert [round(c, 2) for _, c in at['graduated'][:2]] == [3.0, 3.65]
  assert [c for _, c in at['left'][:2]] == [4.3, 4.3]
  for name in ('flat3', 'graduated', 'flat43'):
    assert [a for a, _ in at[name]] == [1.0, 1.0, NO_ACCEL_CAP], name
  assert [a for a, _ in at['left']] == [1.5, 1.5, NO_ACCEL_CAP]
  # the caps only ever lower speed: the accel cap is positive, and the speed cap's own braking stays at -2.0 or gentler
  for p in POST_GO_PROFILES.values():
    for deg in range(-10, 100, 5):
      for v in (0., 1., 3., 5., 8.):
        accel, cap = post_go_caps(p, deg, V_CORNER)
        assert accel > 0. and plan_accel(accel, cap, v) >= -2.0


def test_post_go_follows_the_turn_from_the_go():
  """PostGo keys the caps on the heading turned toward the maneuver: DesireTiming.turned_deg at the go (a hold reached
  already 40 deg into the turn starts the ramp there), then the car's yaw. It ends once the blend is done, or POST_GO_MAX_M
  from the go however little the car turned (a turn missed straight on is not held to 3 m/s down the road)."""
  m = Maneuver(*classify("turn", "left"), "turn", "left", 90.)
  pg = PostGo(POST_GO_PROFILES['graduated'], m, 10., 0.)
  assert pg.caps(0.) == (1.0, 3.0) and not pg.over
  pg.update(1., math.radians(30.), 1.)  # 30 deg of yaw toward the left
  assert math.isclose(pg.turned_deg, 30.) and math.isclose(pg.caps(1.)[1], 3.65, abs_tol=1e-3)
  pg.update(1., -math.radians(10.), 1.)  # yaw away counts back
  assert math.isclose(pg.turned_deg, 20.)
  pg.update(1., math.radians(45.), 1.)
  assert pg.over and pg.caps(1.) == (NO_ACCEL_CAP, NO_SPEED_CAP)

  right = Maneuver(*classify("turn", "right"), "turn", "right", 90.)
  pg = PostGo(POST_GO_PROFILES['graduated'], right, 10., 40.)
  assert math.isclose(pg.caps(0.)[1], 3.0 + 1.3 * 25. / 30., abs_tol=1e-3)
  pg.update(1., -math.radians(10.), 1.)  # a right turns clockwise: negative yaw
  assert math.isclose(pg.turned_deg, 50.)
  # the blend's far end is the maneuver's own corner cap where the car is, the junction POST_GO's distance from the go
  assert math.isclose(pg.caps(1.)[1], 4.3 + (turn_speed_cap(right, 9., 1.) - 4.3) / 3., abs_tol=1e-3)

  straight = PostGo(POST_GO_PROFILES['graduated'], m, 10., 0.)
  while not straight.over:
    straight.update(3., 0., DT_MDL)
    assert straight.travelled_m <= POST_GO_MAX_M + 1.
  assert straight.caps(3.) == (NO_ACCEL_CAP, NO_SPEED_CAP)
