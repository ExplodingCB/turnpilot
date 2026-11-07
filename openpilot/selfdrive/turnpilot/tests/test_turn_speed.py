import math

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.route_follower import NOMINAL_ANGLE_DEG, Maneuver, classify
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.scene_policy import NO_ACCEL_CAP
from openpilot.selfdrive.turnpilot.turn_speed import (CORNER_CURB_M, CORNER_M, LEFT_GO_PROFILE, NO_SPEED_CAP, POST_GO_DEFAULT,
                                                      POST_GO_MAX_M, POST_GO_PROFILES, TURN_CAP_LEAD_M, V_CORNER, V_CORNER_CURB, V_TURN_MAX,
                                                      V_TURN_MIN, PostGo, post_go_caps, turn_speed_cap, turn_target_speed)


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
  assert turn_speed_cap(maneuver("turn", "right"), 0., 4.) == V_CORNER_CURB
  assert turn_speed_cap(maneuver("turn", "left"), 0., 4.) == V_CORNER
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


def test_a_slight_turn_at_a_junction_is_capped_at_any_speed():
  """BUILD_FINDINGS D5: a keep at a junction (its bearings, so a branch: make_maneuver makes a branchless one a bend) of
  KEEP_CORNER_MIN_DEG or more gets its corner cap at any speed, not only under KEEP_CAP_SPEED_MAX. Under that angle, without
  junction data (the modifier's nominal angle), or a ramp or fork (braking on the main road for its exit) it is as before."""
  def keep(mtype, modifier, angle, geometry=True):
    kind, side = classify(mtype, modifier)
    return Maneuver(kind, side, mtype, modifier, angle, geometry=geometry)
  j = keep("turn", "slight right", 22.)  # S23's DR677_1
  assert math.isclose(turn_speed_cap(j, 20., 19.), turn_target_speed(22., curb_side=True))
  assert math.isclose(turn_speed_cap(j, 20., 30.), turn_speed_cap(j, 20., 14.))
  assert turn_speed_cap(keep("turn", "slight left", 20.), 20., 25.) < 12.
  assert turn_speed_cap(j, 400., 25.) > 25.  # far out it doesn't bite: the same braking curve as a town keep's
  assert turn_speed_cap(keep("turn", "slight left", 14.), 20., 19.) == NO_SPEED_CAP  # S19's reroute: under 20 deg
  assert turn_speed_cap(keep("turn", "slight right", 45., geometry=False), 20., 19.) == NO_SPEED_CAP
  assert turn_speed_cap(keep("off ramp", "slight right", 25.), 20., 25.) == NO_SPEED_CAP
  assert turn_speed_cap(keep("fork", "slight left", 30.), 20., 25.) == NO_SPEED_CAP


def test_braking_curve():
  m = maneuver("turn", "right")
  assert math.isclose(turn_speed_cap(m, 100. + TURN_CAP_LEAD_M, 0.), math.sqrt(V_TURN_MIN ** 2 + 2 * 1.5 * 100.))
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
      if at_desire is None and d < TURN_CAP_LEAD_M:
        at_desire = v
    assert V_TURN_MIN - 0.5 < at_desire < V_TURN_MIN + 0.25, modifier  # gate 1's speed where its desire went out
    assert v < V_CORNER + 0.5, modifier                                  # and its own turn speed by the corner
    assert min_a > -1.6, modifier


def test_corner_curve_at_real_angles():
  # a right whose bearings turn 70 degrees still slows to its corner speed by its corner distance, as the nominal 90 did, a
  # left to its own; a slight keep doesn't
  for angle in (70., 85., 90., 120.):
    real = Maneuver(*classify("turn", "right"), "turn", "right", angle, geometry=True)
    assert turn_speed_cap(real, CORNER_CURB_M, 3.) == V_CORNER_CURB, angle
    left = Maneuver(*classify("turn", "left"), "turn", "left", angle, geometry=True)
    assert turn_speed_cap(left, CORNER_M, 4.) == V_CORNER, angle
  keep = Maneuver(*classify("turn", "slight right"), "turn", "slight right", 30., geometry=True)
  assert turn_speed_cap(keep, CORNER_M, 4.) > V_CORNER


def approach(modifier, until_m, v=15., d=300., right_hand_traffic=True):
  """The car under the turn's cap from d m out at v (test_car_reaches_turn_speed_by_the_desire's): (v at until_m, min accel)."""
  m = maneuver("turn", modifier)
  min_a = 0.
  while d > until_m:
    a = min(plan_accel(10., turn_speed_cap(m, d, v, right_hand_traffic), v), 0.)
    min_a = min(min_a, a)
    v = max(v + a * DT_MDL, 0.)
    d -= v * DT_MDL
  return v, min_a


def test_a_curb_side_turn_comes_to_its_corner_at_the_speed_the_model_takes_it_from():
  """BeamNG right_green and red_right's junction (DR435_1), 2026-10-30 to 11-01: every right taken came to it at 3.5 m/s or
  less from 15 m out and started round 12 m out; every right lost (S3, C5, F3) came at 4.6-4.7 m/s, the 4.3 cap binding,
  started round 4-6 m out and was given up (F3 abandoned at 14 deg), stood in the corner or driven straight. A curb-side
  turn comes to 15 m out at no more than 3.5 m/s on the same gentle braking, and through its corner at V_CORNER_CURB; a
  far-side one keeps V_CORNER (its wider arc); traffic keeping left mirrors it."""
  v, min_a = approach("right", 15.)
  assert v <= 3.5 and min_a > -1.6, (v, min_a)
  assert approach("right", 0.)[0] <= V_CORNER_CURB + 0.1
  v, _ = approach("left", 15.)
  assert V_CORNER - 0.3 < v < V_CORNER + 0.6, v
  assert approach("left", 15., right_hand_traffic=False)[0] <= 3.5


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


def test_post_go_ends_where_a_shallow_turn_is_done():
  """A junction whose bearings turn 50 deg is done at 40 deg (DesireTiming clears its desire at max(angle - 10, 0.75 angle)):
  the profile ends there too, not at 60 deg, so the car isn't crawling down the exit road under the ramp's cap."""
  shallow = Maneuver(*classify("turn", "left"), "turn", "left", 50., geometry=True)
  pg = PostGo(POST_GO_PROFILES['graduated'], shallow, 10., 0.)
  pg.update(1., math.radians(39.), 1.)
  assert not pg.over and pg.caps(1.)[0] == 1.0
  pg.update(1., math.radians(1.5), 1.)
  assert pg.over and pg.caps(1.) == (NO_ACCEL_CAP, NO_SPEED_CAP)
  square = Maneuver(*classify("turn", "left"), "turn", "left", 90., geometry=True)  # done at 80: 60 ends it first
  pg = PostGo(POST_GO_PROFILES['graduated'], square, 10., 59.)
  assert not pg.over
  pg.update(1., math.radians(1.5), 1.)
  assert pg.over


def test_the_caps_braking_curve_is_its_own_not_the_desires_lead(monkeypatch):
  """2026-11-06: a curb-side turn's one pulse goes 8 m out and a far-side one's lead comes in with speed (desire_timing.turn_lead),
  so the cap's braking curve has its own anchor, TURN_CAP_LEAD_M, and the caps are what they were: 6.9 m/s by 40 m for any
  junction turn, the curb side's 3 m/s by 20 m, the far side's 4.3 by 15 m; the car under them reaches those speeds there."""
  import openpilot.selfdrive.turnpilot.desire_timing as dt
  assert TURN_CAP_LEAD_M == 40.
  right, left = maneuver("turn", "right"), maneuver("turn", "left")
  before = [turn_speed_cap(m, d, v) for m in (right, left) for d in (300., 100., 60., 40., 30., 20., 15., 8., 0.) for v in (0., 3., 6.9, 12.)]
  monkeypatch.setattr(dt, 'TURN_DESIRE_LEAD_M', 25.)  # nothing in the cap reads the desire's lead
  assert before == [turn_speed_cap(m, d, v) for m in (right, left) for d in (300., 100., 60., 40., 30., 20., 15., 8., 0.) for v in (0., 3., 6.9, 12.)]
  assert math.isclose(turn_speed_cap(left, 40., 6.9), V_TURN_MIN) and math.isclose(turn_speed_cap(right, 40., 6.9), V_TURN_MIN)
  assert math.isclose(turn_speed_cap(right, CORNER_CURB_M, 3.), V_CORNER_CURB) and CORNER_CURB_M == 20.
  assert math.isclose(turn_speed_cap(left, CORNER_M, 4.), V_CORNER)
  for modifier in ("right", "left"):
    v40, _ = approach(modifier, 40.)
    assert V_TURN_MIN - 0.5 < v40 < V_TURN_MIN + 0.25, (modifier, v40)
  v20, _ = approach("right", 20.)
  assert v20 < V_CORNER_CURB + 0.6, v20
