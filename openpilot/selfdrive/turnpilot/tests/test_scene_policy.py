import math
from types import SimpleNamespace
from unittest import mock

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import SPEED_CAP_ACCEL_MIN as MAP_DECEL_MAX, plan_accel
from openpilot.selfdrive.turnpilot.route_follower import NOMINAL_ANGLE_DEG, Maneuver, classify
from openpilot.selfdrive.turnpilot.scene_policy import (ACCEL_CAP_JERK, COMMIT_SPEED, CONFIRM_HITS, CROSSWALK_LEAVING_FRAMES, GAS_RELEASE_M, HARD_MISSED_RUNS,
                                                        HOLD_LINE_M, LOST_CLOSE_M,
                                                        NO_ACCEL_CAP, NO_SPEED_CAP, ONCOMING_ASSUMED_SPEED, ONCOMING_ENTRY_M, ONCOMING_TTA_S,
                                                        PED_MAX_SPEED, PED_STOP_GAP_M, STOP_DECEL, VEL_TRUST_HITS, ScenePolicy, Stop, scene_policy)
from openpilot.selfdrive.turnpilot.tests.synthetic import scene

LEFT_TURN = Maneuver(*classify("turn", "left"), "turn", "left", NOMINAL_ANGLE_DEG["left"])
RIGHT_TURN = Maneuver(*classify("turn", "right"), "turn", "right", NOMINAL_ANGLE_DEG["right"])


def ped(x, y, vx=0., vy=0., in_path=False, **fields):
  """fields: more TrackedObject fields, e.g. age (hits) and conf; scene() makes it a confirmed track otherwise."""
  return {'cls': 'pedestrian', 'x': x, 'y': y, 'vx': vx, 'vy': vy, 'inPath': in_path, **fields}


def car(x, y, vx=0., vy=0., **fields):
  """fields: more TrackedObject fields, e.g. age (hits); scene() gives 10 otherwise."""
  return {'cls': 'car', 'x': x, 'y': y, 'vx': vx, 'vy': vy, 'inPath': False, **fields}


def policy(objects=(), available=True, maneuver=None, distance=1000., v_ego=10.):
  sc = scene(objects, available)
  return scene_policy(sc.objects, sc.available, maneuver, distance, v_ego)


def test_nothing_there():
  d = policy()
  assert (d.accel_cap, d.speed_cap, d.should_stop, d.holding, d.reason) == (NO_ACCEL_CAP, NO_SPEED_CAP, False, False, "")


def test_pedestrian_in_path_stops():
  d = policy([ped(30., 0.2, in_path=True)], v_ego=10.)
  assert d.holding and d.reason == "pedestrian in path" and d.speed_cap < 10.
  # too close for the comfortable curve: brake harder through accelCap
  d = policy([ped(14., 0.2, in_path=True)], v_ego=10.)
  assert d.accel_cap < -5.
  # at the stop point near standstill: hold
  d = policy([ped(PED_STOP_GAP_M + 0.5, 0.2, in_path=True)], v_ego=0.1)
  assert d.should_stop


def test_closed_loop_stop_short_of_pedestrian():
  x_ped, v, s, stopped = 60., 12., 0., False
  for _ in range(int(20 / DT_MDL)):
    d = policy([ped(x_ped - s, 0., in_path=True)], v_ego=v)
    a = max(min(plan_accel(d.accel_cap, d.speed_cap, v), 0.5), -3.5)  # planner min() with cruise, then ACCEL_MIN
    if d.should_stop:
      a = min(a, -1.)
      stopped = True
    v = max(v + a * DT_MDL, 0.)
    s += v * DT_MDL
  assert stopped and v == 0.
  assert 0. < x_ped - s < PED_STOP_GAP_M + 2.


def test_pedestrian_on_sidewalk_ignored_walking_in_counts():
  assert not policy([ped(20., -5.)]).holding
  d = policy([ped(20., -3., vy=2.5)])
  assert d.holding and d.reason == "pedestrian in path"


def test_crosswalk_hold_on_right_turn():
  d = policy([ped(23., -6.)], maneuver=RIGHT_TURN, distance=20., v_ego=5.)
  assert d.holding and d.reason == "pedestrian in crosswalk"
  assert d.speed_cap < 5.
  # same pedestrian, but the turn is a left: not on the turn side
  assert policy([ped(23., -6.)], maneuver=LEFT_TURN, distance=20., v_ego=5.).reason != "pedestrian in crosswalk"
  # committed: past the hold line and moving
  assert not policy([ped(8., -6.)], maneuver=RIGHT_TURN, distance=HOLD_LINE_M - 3., v_ego=4.).holding


def test_a_stop_to_rest_is_said_while_braking_toward_it():
  """decision.rest_d: where the nearest stop in force brings the car to rest (shouldStop at its end), braking toward it
  or standing at it; never a following gap (turnpilotd: a veto then is decided as standing at the stop, car_signals.py)."""
  assert policy().rest_d is None
  d = policy([ped(36., -6.)], maneuver=RIGHT_TURN, distance=33., v_ego=7.)  # a walker's hold, the car still rolling
  assert d.holding and not d.should_stop and d.rest_d == 33. - HOLD_LINE_M
  d = policy([ped(30., 0.2, in_path=True)], v_ego=10.)
  assert not d.should_stop and d.rest_d == 30. - PED_STOP_GAP_M
  # a confirmed walker moving along the path is followed, never stopped for: no rest at the end of it
  d = policy([ped(30., 0.2, vx=1.5, in_path=True)], v_ego=10.)
  assert d.holding and d.rest_d is None


def test_oncoming_car_holds_unprotected_left():
  d = policy([car(60., 5., vx=-12.)], maneuver=LEFT_TURN, distance=15., v_ego=4.)
  assert d.holding and d.reason == "waiting for oncoming"
  # a gap of more than 5 s, a receding car, a car in the cross street, or one already through: go
  assert not policy([car(60., 5., vx=-5., age=VEL_TRUST_HITS)], maneuver=LEFT_TURN, distance=15., v_ego=4.).holding
  assert not policy([car(60., 5., vx=8., age=VEL_TRUST_HITS)], maneuver=LEFT_TURN, distance=15., v_ego=4.).holding
  assert not policy([car(60., -5., vx=-12.)], maneuver=LEFT_TURN, distance=15., v_ego=4.).holding
  assert not policy([car(5., 5., vx=-12.)], maneuver=LEFT_TURN, distance=15., v_ego=4.).holding
  # far from the junction the hold isn't engaged yet
  assert not policy([car(160., 5., vx=-30.)], maneuver=LEFT_TURN, distance=120., v_ego=10.).holding
  # stopped at the hold line: stays stopped
  d = policy([car(60., 5., vx=-12.)], maneuver=LEFT_TURN, distance=HOLD_LINE_M + 0.5, v_ego=0.)
  assert d.should_stop
  # committed to the turn
  assert not policy([car(40., 5., vx=-12.)], maneuver=LEFT_TURN, distance=HOLD_LINE_M - 3., v_ego=4.).holding


def test_new_tracks_velocity_is_taken_as_approaching():
  # BeamNG unprotected_left_visible run 4: the wide camera's new track of a car closing at about 11 m/s read vx +3.6,
  # +6.1, +5.5, +1.5 for 2 s (as receding), and the hold came only after it had crossed. Until the track's velocity is
  # trusted, a vehicle in the oncoming lanes is taken as coming at ONCOMING_ASSUMED_SPEED
  for vx in (3.6, 6.1, 0., -2.):
    d = policy([car(30., 5., vx=vx, age=10)], maneuver=LEFT_TURN, distance=HOLD_LINE_M + 0.5, v_ego=0.)
    assert d.should_stop and d.reason == "waiting for oncoming", vx
  # trusted, a receding or standing car is let go, as before
  for vx in (6.1, 0.):
    assert not policy([car(30., 5., vx=vx, age=VEL_TRUST_HITS)], maneuver=LEFT_TURN, distance=HOLD_LINE_M + 0.5, v_ego=0.).holding
  # beyond the assumed speed's reach within ONCOMING_TTA_S: not yet
  far = HOLD_LINE_M + 0.5 + ONCOMING_ASSUMED_SPEED * ONCOMING_TTA_S + 5.
  assert not policy([car(far, 5., vx=3., age=3)], maneuver=LEFT_TURN, distance=HOLD_LINE_M + 0.5, v_ego=0.).holding
  # ScenePolicy takes the same rule
  sp = ScenePolicy()
  d = sp.update(scene([car(30., 5., vx=5.5, age=12)]).objects, True, LEFT_TURN, HOLD_LINE_M + 0.5, 0., maneuver_id=1)
  assert d.should_stop and d.reason == "waiting for oncoming"


def yawed(x_road, y_road, vx_road, vy_road, yaw_deg):
  """A road-frame point and velocity (x along the approach, y left) as the car, yawed yaw_deg left, sees them."""
  a = math.radians(yaw_deg)
  def rot(x, y):
    return x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a)
  return (*rot(x_road, y_road), *rot(vx_road, vy_road))


def test_oncoming_band_is_in_the_road_frame_with_the_car_yawed_into_the_turn():
  # DESIGN phase 4 section 2: standing 30 deg into a left at its hold line (BeamNG V1 at a 10 m hold stood 30-45 deg in), an
  # oncoming car 50 m up the road in the first oncoming lane is about 22 m to the car's right in its own frame: outside a
  # car-frame band (y left 1.5-20 m), never counted. The band is the road's: the car's frame turned back by its yaw
  x, y, vx, vy = yawed(50., 3.6, -12., 0., 30.)
  assert y < -20.
  oncoming = scene([car(x, y, vx=vx, vy=vy, age=VEL_TRUST_HITS)]).objects
  d = scene_policy(oncoming, True, LEFT_TURN, HOLD_LINE_M + 0.5, 0., heading_deg=30.)
  assert d.should_stop and d.reason == "waiting for oncoming"
  d = ScenePolicy().update(oncoming, True, LEFT_TURN, HOLD_LINE_M + 0.5, 0., maneuver_id=1, heading_deg=30.)
  assert d.should_stop and d.reason == "waiting for oncoming"
  # and the other way: a car in the car's own lane ahead (road y 0), going away, which the yawed car sees to its right, and
  # one on the cross street far to the right, aren't oncoming
  for road in ((50., 0., 12., 0.), (5., -12., 0., 10.)):
    x, y, vx, vy = yawed(*road, 30.)
    assert not scene_policy(scene([car(x, y, vx=vx, vy=vy, age=VEL_TRUST_HITS)]).objects, True, LEFT_TURN, HOLD_LINE_M + 0.5, 0.,
                            heading_deg=30.).holding
  # a right on the left-hand side of the road (its far side) turns the other way
  right_on_left = Maneuver(*classify("turn", "right"), "turn", "right", NOMINAL_ANGLE_DEG["right"])
  x, y, vx, vy = yawed(50., -3.6, -12., 0., -30.)
  d = scene_policy(scene([car(x, y, vx=vx, vy=vy, age=VEL_TRUST_HITS)]).objects, True, right_on_left, HOLD_LINE_M + 0.5, 0.,
                   right_hand_traffic=False, heading_deg=30.)
  assert d.should_stop and d.reason == "waiting for oncoming"


def test_detector_unavailable_is_no_evidence():
  # no scene is neither a hold nor a reason: the stopped detector is turnpilotd's to hand back
  for maneuver in (LEFT_TURN, RIGHT_TURN, None):
    d = policy([car(60., 5., vx=-12.)], available=False, maneuver=maneuver, distance=15., v_ego=4.)
    assert (d.accel_cap, d.speed_cap, d.should_stop, d.holding, d.reason) == (NO_ACCEL_CAP, NO_SPEED_CAP, False, False, "")


def drive(sp, frames, v, s, objects_at, available=True, maneuver=None, maneuver_at=None, maneuver_id=0, gas=lambda t: False,
          accel_min=-3.5):
  """A point-mass car under a ScenePolicy for `frames` frames from speed v at s; objects_at(s, t) gives the objects.
  Returns (v, s, decisions)."""
  decisions = []
  for k in range(frames):
    t = k * DT_MDL
    sc = scene(objects_at(s, t), available(t) if callable(available) else available)
    distance = maneuver_at - s if maneuver_at is not None else 1000.
    d = sp.update(sc.objects, sc.available, maneuver, distance, v, maneuver_id=maneuver_id, gas_pressed=gas(t))
    decisions.append(d)
    a = max(min(plan_accel(d.accel_cap, d.speed_cap, v), 0.5), accel_min)
    if d.should_stop:
      a = min(a, -1.)
    v = max(v + a * DT_MDL, 0.)
    s += v * DT_MDL
  return v, s, decisions


def test_no_stop_and_wait_with_the_detector_down():
  # the removed hold: approaching an unprotected left with no scene, the car drives through without waiting at the line
  sp = ScenePolicy()
  v, s, ds = drive(sp, int(30. / DT_MDL), 10., 0., lambda s, t: (), available=False, maneuver=LEFT_TURN, maneuver_at=150.)
  assert s > 250. and v >= 10. and not any(d.holding or d.should_stop or d.reason for d in ds)


def test_a_hold_in_force_is_carried_through_a_gap():
  # waiting for an oncoming car at the line when the detector misses a few runs: still waiting, not released
  sp = ScenePolicy()
  oncoming = scene([car(60., 5., vx=-12.)])
  for _ in range(5):
    d = sp.update(oncoming.objects, True, LEFT_TURN, HOLD_LINE_M + 0.5, 0., maneuver_id=7)
  assert d.should_stop and d.reason == "waiting for oncoming"
  for _ in range(int(1. / DT_MDL)):
    d = sp.update([], False, LEFT_TURN, HOLD_LINE_M + 0.5, 0., maneuver_id=7)
  assert d.should_stop and d.reason == "waiting for oncoming"
  # back with nothing coming: released at once
  assert not sp.update([], True, LEFT_TURN, HOLD_LINE_M + 0.5, 0., maneuver_id=7).holding
  # a pedestrian stop is carried where the pedestrian was: the car still stops short of them
  sp, x_ped = ScenePolicy(), 40.
  v, s, ds = drive(sp, int(20. / DT_MDL), 10., 0., lambda s, t: (ped(x_ped - s, 0.1, in_path=True),),
                   available=lambda t: t < 0.5)
  assert v == 0. and ds[-1].should_stop and 0. < x_ped - s < PED_STOP_GAP_M + 2.
  assert not ScenePolicy().update([], False, LEFT_TURN, 15., 4., maneuver_id=0).holding  # nothing was in force


def test_gas_releases_the_hold_at_the_line():
  oncoming = scene([car(60., 5., vx=-12.)])
  # crept past the hold line on the gas, then lifted: the one-frame policy stops it where the comfortable curve can, short
  # of the oncoming lanes (no hard stop in the junction)
  d = policy([car(60., 5., vx=-12.)], maneuver=LEFT_TURN, distance=HOLD_LINE_M - 2., v_ego=1.5)
  assert d.holding and d.speed_cap == 0. and d.accel_cap == NO_ACCEL_CAP
  sp = ScenePolicy()
  d = sp.update(oncoming.objects, True, LEFT_TURN, HOLD_LINE_M - 2., 1.5, maneuver_id=3, gas_pressed=True)
  assert not d.holding
  d = sp.update(oncoming.objects, True, LEFT_TURN, HOLD_LINE_M - 2.1, 1.5, maneuver_id=3, gas_pressed=False)
  assert not d.holding and d.accel_cap == NO_ACCEL_CAP
  # the next maneuver holds again
  assert sp.update(oncoming.objects, True, LEFT_TURN, 15., 4., maneuver_id=4).reason == "waiting for oncoming"


def test_gas_further_out_releases_nothing():
  # the hold engages within braking distance of the line: gas there is the driver keeping up speed, not a go
  sp = ScenePolicy()
  oncoming = scene([car(80., 5., vx=-12.)])
  d = sp.update(oncoming.objects, True, LEFT_TURN, HOLD_LINE_M + GAS_RELEASE_M + 10., 8., maneuver_id=0, gas_pressed=True)
  assert d.reason == "waiting for oncoming"
  closer = scene([car(50., 5., vx=-12.)])
  d = sp.update(closer.objects, True, LEFT_TURN, HOLD_LINE_M + 1., 1., maneuver_id=0)
  assert d.holding and d.reason == "waiting for oncoming"
  # gas never releases a pedestrian in the path
  sc = scene([ped(12., 0.2, in_path=True)])
  for _ in range(3):
    d = sp.update(sc.objects, True, LEFT_TURN, HOLD_LINE_M + 1., 4., maneuver_id=0, gas_pressed=True)
  assert d.holding and d.reason == "pedestrian in path"


def test_unconfirmed_velocity_never_turns_a_stop_into_a_follow():
  # a standing pedestrian on a 2-hit track whose range noise reads as 1.5 m/s away: stopped for, not followed
  d = policy([ped(30., 0.2, vx=1.5, in_path=True, age=2, conf=0.7)], v_ego=10.)
  assert d.reason == "pedestrian in path"
  assert policy([ped(30., 0.2, vx=1.5, in_path=True, age=6, conf=0.7)], v_ego=10.).reason == "following pedestrian"
  # a person beside a 2-hit bicycle track that reads as moving is not its rider
  rider = ped(20., 0.2, in_path=True, camera='road', box=[0.48, 0.40, 0.52, 0.55])
  bike = {'cls': 'cyclist', 'x': 20., 'y': 0.2, 'vx': 6., 'vy': 0., 'camera': 'road', 'box': [0.47, 0.47, 0.53, 0.58]}
  assert policy([rider, bike | {'age': 2, 'conf': 0.6}], v_ego=10.).reason == "pedestrian in path"


def test_steady_low_confidence_pedestrian_is_confirmed():
  # YOLO sees them at 0.5 on every run: after CONFIRM_STEADY_HITS they get a hard stop like any confirmed track
  assert policy([ped(14., 0.2, in_path=True, age=9, conf=0.5)], v_ego=10.).accel_cap == NO_ACCEL_CAP
  assert policy([ped(14., 0.2, in_path=True, age=10, conf=0.5)], v_ego=10.).accel_cap < -3.


def test_far_false_positive_is_not_in_the_path():
  # 150 m out, a 2-hit track drifting toward the lane: was a stop at 145 m, -3.1 m/s^2 at 30 m/s
  d = policy([ped(150., -3., vy=2.5, age=2, conf=0.45)], v_ego=30.)
  assert not d.holding and d.accel_cap == NO_ACCEL_CAP
  assert policy([ped(30., -3., vy=2.5)]).reason == "pedestrian in path"  # walking in close up still counts


def test_unconfirmed_track_brakes_on_the_curve_only():
  weak = policy([ped(45., 0.2, in_path=True, age=2, conf=0.45)], v_ego=30.)
  assert weak.holding and weak.accel_cap == NO_ACCEL_CAP
  assert plan_accel(weak.accel_cap, weak.speed_cap, 30.) >= -STOP_DECEL - 1e-6
  strong = policy([ped(45., 0.2, in_path=True, age=5, conf=0.6)], v_ego=30.)
  assert strong.accel_cap < -5.
  # a stop the curve can make needs no confirmation
  d = policy([ped(40., 0.2, in_path=True, age=2, conf=0.45)], v_ego=10.)
  assert d.accel_cap == NO_ACCEL_CAP and d.speed_cap < 10.


def test_accel_cap_ramps_in_and_confirmation_sticks():
  sp = ScenePolicy()
  caps = []
  for k in range(12):
    conf = 0.7 if k < 2 else 0.5  # confirmed on the first frame, then its confidence dips: still confirmed
    sc = scene([ped(30. - 0.5 * k, 0.2, in_path=True, age=5 + k, conf=conf)])
    caps.append(sp.update(sc.objects, True, None, 1000., 15.).accel_cap)
  assert caps[0] == -STOP_DECEL - ACCEL_CAP_JERK * DT_MDL
  assert all(0. <= a - b <= ACCEL_CAP_JERK * DT_MDL + 1e-6 for a, b in zip(caps, caps[1:], strict=False))
  assert caps[-1] < -3.5
  # and it eases at once
  assert sp.update([], True, None, 1000., 15.).accel_cap == NO_ACCEL_CAP


def test_closed_loop_stop_with_growing_evidence():
  # gate 3: the mannequin in the lane, tracked from about 55 m at 35 km/h on 2 hits, conf 0.5-0.85
  sp, x_ped = ScenePolicy(), 55.

  def mannequin(s, t):
    return (ped(x_ped - s, 0.1, in_path=True, age=2 + int(t / 0.1), conf=min(0.5 + t, 0.85)),)

  v, s, ds = drive(sp, int(20. / DT_MDL), 9.7, 0., mannequin)
  assert v == 0. and ds[-1].should_stop
  assert 0. < x_ped - s < PED_STOP_GAP_M + 2.


def test_cyclist_riding_away_is_followed():
  # a person 60 m ahead riding away at 6 m/s, closing at 25 m/s: a moving lead, not a wall, on one frame on its own
  # (scene_policy). ScenePolicy reads a pedestrian track that fast as standing (PED_MAX_SPEED, peds P2)
  moving = policy([ped(60., 0.2, vx=6., in_path=True)], v_ego=25.)
  standing = policy([ped(60., 0.2, in_path=True)], v_ego=25.)
  assert moving.reason == "following pedestrian" and not moving.should_stop
  assert moving.accel_cap > standing.accel_cap and moving.speed_cap > standing.speed_cap
  # through ScenePolicy at a person's speed, a jogger 3 m/s away, matched on every detector run (10 Hz): confirmation is
  # re-earned every frame from recent hits (update_evidence)
  sp, jog = ScenePolicy(), 3.
  assert jog < PED_MAX_SPEED
  v, s, ds = drive(sp, int(30. / DT_MDL), 15., 0., lambda s, t: (ped(60. + jog * t - s, 0.2, vx=jog, in_path=True, age=10 + int(t / 0.1)),))
  gap = 60. + jog * 30. - s
  assert not any(d.should_stop for d in ds) and abs(v - jog) < 0.5 and PED_STOP_GAP_M < gap < PED_STOP_GAP_M + jog * 3.


def test_rider_on_a_moving_two_wheeler_is_not_a_pedestrian():
  rider = ped(20., 0.2, vx=1.5, in_path=True, camera='road', box=[0.48, 0.40, 0.52, 0.55])
  bike = {'cls': 'cyclist', 'x': 20., 'y': 0.2, 'vx': 6., 'vy': 0., 'camera': 'road', 'box': [0.47, 0.47, 0.53, 0.58]}
  assert policy([rider], v_ego=10.).holding  # alone, a person moving away
  assert not policy([rider, bike], v_ego=10.).holding  # on a moving bicycle: the driving model's lead
  # someone pushing a bike, a bike standing beside a person, or a bike in the other camera: still a pedestrian
  assert policy([rider, bike | {'vx': 1.2}], v_ego=10.).holding
  assert policy([rider, bike | {'box': [0.60, 0.47, 0.66, 0.58]}], v_ego=10.).holding
  assert policy([rider, bike | {'camera': 'wide'}], v_ego=10.).holding


# turnpilotScene at BeamNG (-311, 337), rlog 00000012--39e37873b0--1, plan 22.20-23.75 s: a wide camera false positive the
# crosswalk rule held a right turn on. (t, age, conf, x, y, vx, vy) every 50 ms, None where it wasn't published, and the
# car's (distance to the maneuver point, vEgo).
BEAMNG_FALSE_WALKER = (
  [(22.20 + 0.05 * i, 2, 0.63, 9.9 - 0.1 * i, -10.1 + 0.05 * i, -0.43, 1.83) for i in range(7)] +
  [(22.55 + 0.05 * i, 3, 0.51, 9.6 - 0.02 * i, -10.0 + 0.04 * i, 0.41, 1.17) for i in range(11)] +
  [None] * 4 +
  [(23.30 + 0.05 * i, 4, 0.42, 9.8, -10.8 - 0.01 * i, 0.73, -0.15) for i in range(10)])


def test_beamng_crosswalk_false_positive_does_not_hold():
  sp = ScenePolicy()
  for k, row in enumerate(BEAMNG_FALSE_WALKER):
    distance, v = 0.1 - 1.5 * k / len(BEAMNG_FALSE_WALKER), 1.9 - 1.5 * k / len(BEAMNG_FALSE_WALKER)
    objects = [] if row is None else [ped(*row[3:5], vx=row[5], vy=row[6], age=row[1], conf=row[2], camera='wide')]
    sc = scene(objects)
    d = sp.update(sc.objects, True, RIGHT_TURN, distance, v, maneuver_id=0)
    assert not d.holding and not d.should_stop and d.reason == ""
    d = policy(objects, maneuver=RIGHT_TURN, distance=distance, v_ego=v)
    assert not d.holding and not d.should_stop and d.reason == ""


def test_steady_walker_holds_sporadic_one_does_not():
  def run(hit_every_s, frames):
    sp, d = ScenePolicy(), None
    for k in range(frames):
      sc = scene([ped(23., -6., age=2 + int(k * DT_MDL / hit_every_s), conf=0.7 if k < 30 else 0.45)])
      d = sp.update(sc.objects, True, RIGHT_TURN, 20., 5., maneuver_id=0)
    return d
  assert run(0.1, 10).reason == "pedestrian in crosswalk"  # on every 10 Hz run: holds within half a second...
  assert run(0.1, 60).reason == "pedestrian in crosswalk"  # ...and keeps holding when its confidence dips
  assert run(0.6, 100).reason != "pedestrian in crosswalk"  # 6 hits, but one every 0.6 s


def test_committed_turn_is_never_held_again():
  # past the hold line above COMMIT_SPEED with nothing oncoming, the car is committed to its left. Slowing under
  # COMMIT_SPEED further in (a lead turning ahead) with a car now oncoming doesn't bring the hold back: that would stop
  # it hard inside the junction, cancelling a committed turn
  sp, distance = ScenePolicy(), 15.
  while distance > 3.:
    assert not sp.update([], True, LEFT_TURN, distance, 4., maneuver_id=1).holding
    distance -= 4. * DT_MDL
  oncoming = scene([car(35., 6., vx=-10.)])
  for _ in range(10):
    d = sp.update(oncoming.objects, True, LEFT_TURN, distance, 1.9, maneuver_id=1)
    assert not d.holding and not d.should_stop and d.accel_cap == NO_ACCEL_CAP
  for _ in range(10):  # nor through a detector gap
    assert not sp.update([], False, LEFT_TURN, distance, 1.9, maneuver_id=1).holding
  # a pedestrian in the path still stops it
  d = sp.update(scene([car(35., 6., vx=-10.), ped(8., 0.2, in_path=True)]).objects, True, LEFT_TURN, distance, 1.9, maneuver_id=1)
  assert d.holding and d.reason == "pedestrian in path"
  # crept past the line under COMMIT_SPEED: not committed, so it still waits there
  sp = ScenePolicy()
  assert sp.update(oncoming.objects, True, LEFT_TURN, HOLD_LINE_M - 0.5, 1., maneuver_id=1).holding
  # and the next maneuver holds as usual
  sp = ScenePolicy()
  sp.update([], True, LEFT_TURN, 3., 4., maneuver_id=1)
  assert sp.update(oncoming.objects, True, LEFT_TURN, 15., 4., maneuver_id=2).reason == "waiting for oncoming"


def test_track_evidence_outlives_a_spell_unpublished():
  # the tracker keeps a track it has stopped publishing (MAX_COAST_S) until MAX_AGE_S, CLOSE_HOLD_S close in, and matches
  # it again under the same id: a confirmed pedestrian (seen on every 10 Hz run for a second) unpublished for 0.4 s (occluded
  # for 0.9 s: the tracker coasts it MAX_COAST_S first) and back at a lower confidence is confirmed again on its first
  # frame back, its hits before the occlusion still recent
  sp = ScenePolicy()
  for k in range(20):
    sp.update(scene([ped(20., 0.2, in_path=True, age=2 + k // 2, conf=0.7)]).objects, True, None, 1000., 10.)
  assert sp.tracks[0].confirmed
  for _ in range(8):
    sp.update([], True, None, 1000., 10.)
  assert not sp.tracks[0].confirmed  # coasted out: no hard stop on a frame of it alone (skeptic G1, the 1 Hz phantom)
  back = scene([ped(13., 0.2, in_path=True, age=12, conf=0.55)])
  d = sp.update(back.objects, True, None, 1000., 10.)
  assert d.reason == "pedestrian in path" and d.accel_cap < NO_ACCEL_CAP
  # unpublished for 1.2 s, its hits no longer recent: its stop brakes on the comfortable curve until it is steady again
  sp = ScenePolicy()
  for k in range(20):
    sp.update(scene([ped(20., 0.2, in_path=True, age=2 + k // 2, conf=0.7)]).objects, True, None, 1000., 10.)
  for _ in range(24):
    sp.update([], True, None, 1000., 10.)
  assert sp.update(back.objects, True, None, 1000., 10.).accel_cap == NO_ACCEL_CAP
  for k in range(8):
    d = sp.update(scene([ped(13., 0.2, in_path=True, age=13 + k // 2, conf=0.55)]).objects, True, None, 1000., 10.)
  assert d.accel_cap < NO_ACCEL_CAP
  back = scene([ped(13., 0.2, in_path=True, age=7, conf=0.55)])
  # gone longer than the tracker keeps any track: a new one, which earns its stop afresh
  sp = ScenePolicy()
  sp.update(scene([ped(20., 0.2, in_path=True, age=6, conf=0.7)]).objects, True, None, 1000., 10.)
  for _ in range(int(4. / DT_MDL)):
    sp.update([], True, None, 1000., 10.)
  assert sp.update(back.objects, True, None, 1000., 10.).accel_cap == NO_ACCEL_CAP
  # an id that comes back with fewer hits is another track (a restarted tracker): no evidence carried over
  sp = ScenePolicy()
  sp.update(scene([ped(20., 0.2, in_path=True, age=9, conf=0.7)]).objects, True, None, 1000., 10.)
  assert sp.update(back.objects, True, None, 1000., 10.).accel_cap == NO_ACCEL_CAP
  # a crosswalk walker unpublished for 0.6 s holds again on its first frame back
  sp = ScenePolicy()
  for k in range(10):
    assert sp.update(scene([ped(23., -6., age=2 + k, conf=0.7)]).objects, True, RIGHT_TURN, 20., 5., maneuver_id=0) is not None
  for _ in range(12):
    sp.update([], True, RIGHT_TURN, 20., 5., maneuver_id=0)
  d = sp.update(scene([ped(23., -6., age=12, conf=0.55)]).objects, True, RIGHT_TURN, 20., 5., maneuver_id=0)
  assert d.reason == "pedestrian in crosswalk"


def test_crosswalk_hold_lets_a_walker_leaving_it_go():
  # right turn, stopped at the hold line: someone walking on past the far corner, or away down the cross street, isn't
  # going to step into the turn's crosswalk
  at_line = {'maneuver': RIGHT_TURN, 'distance': HOLD_LINE_M, 'v_ego': 0.}
  assert not policy([ped(15., -10., vx=1.3)], **at_line).holding
  assert not policy([ped(10., -9., vy=-1.3)], **at_line).holding
  # every other case still holds: standing at the corner (waiting to cross either road), crossing the cross street
  # either way, walking toward the car's road, walking back along it, and walking off diagonally
  for p in (ped(5., -6.), ped(HOLD_LINE_M + 1., -6., vx=1.3), ped(HOLD_LINE_M + 2., -6., vx=-1.3), ped(15., -10., vx=-1.3),
            ped(10., -9., vy=1.3), ped(5., -6., vx=-1.3), ped(15., -10., vx=1.3, vy=1.3), ped(15., -10., vx=0.5)):
    d = policy([p], **at_line)
    assert d.should_stop and d.reason == "pedestrian in crosswalk", p
  # an unconfirmed track's velocity is mostly range noise: it never lets the hold go
  assert policy([ped(15., -10., vx=1.3, age=5, conf=0.55)], **at_line).holding
  # left-hand traffic, a left turn: the same, mirrored
  left_line = at_line | {'maneuver': LEFT_TURN}
  sc = scene([ped(15., 10., vx=1.3)])
  assert not scene_policy(sc.objects, True, LEFT_TURN, HOLD_LINE_M, 0., right_hand_traffic=False).holding
  sc = scene([ped(15., 10.)])
  assert scene_policy(sc.objects, True, left_line['maneuver'], HOLD_LINE_M, 0., right_hand_traffic=False).holding


def test_crosswalk_hold_needs_a_walker_steadily_leaving():
  # a Kalman velocity on a person standing at the corner can read a walking pace away for a frame or two: the hold stays.
  # Walking away for CROSSWALK_LEAVING_FRAMES in a row lets it go
  def walker(sp, k, x, y, vy):  # a confirmed walker the detector sees on each 10 Hz run
    sc = scene([ped(x, y, vy=vy, age=10 + k // 2)])
    return sp.update(sc.objects, True, RIGHT_TURN, HOLD_LINE_M, 0., maneuver_id=0)
  warm_up = int(1. / DT_MDL)
  sp, noisy = ScenePolicy(), (-1.2, 0.1, 0.0, -1.1, 0.2)
  for k in range(int(6. / DT_MDL)):
    d = walker(sp, k, 5., -6., 0. if k < warm_up else noisy[k % len(noisy)])
    assert k < warm_up or (d.should_stop and d.reason == "pedestrian in crosswalk"), k
  sp = ScenePolicy()
  for k in range(warm_up):
    walker(sp, k, 10., -9., 0.)
  holds = [walker(sp, warm_up + k, 10., -9., -1.3).holding for k in range(2 * CROSSWALK_LEAVING_FRAMES)]
  assert all(holds[:CROSSWALK_LEAVING_FRAMES - 1]) and not any(holds[CROSSWALK_LEAVING_FRAMES:])


def test_fault_keeps_a_junction_open():
  # committed to a left (id 5) when turnpilotd faults: the route comes back with the same left under a new id (7), the
  # car slow at it with an oncoming car in range. It isn't held in the junction. The next junction, further on, is
  for distance, held in ((2., False), (20., True)):
    sp = ScenePolicy()
    sp.committed_id = 5
    sp.fault(5)
    sp.update([], True, None, float('inf'), 1.)  # the faulted frame: no route
    d = sp.update(scene([car(distance + 4., 5., vx=-1.5)]).objects, True, LEFT_TURN, distance, 1., maneuver_id=7)
    assert (d.reason == "waiting for oncoming") == held, distance


def test_after_the_go_an_oncoming_car_still_holds_short_of_the_oncoming_lanes():
  # pulling away from the hold line past COMMIT_SPEED is no commitment while the car can still stop short of the oncoming
  # lanes on the comfortable curve (BeamNG: the hold depended on the car happening to be under 2 m/s there)
  sp, distance, v = ScenePolicy(), HOLD_LINE_M + 0.5, 0.
  while distance > HOLD_LINE_M - 1.5:
    assert not sp.update([], True, LEFT_TURN, distance, v, maneuver_id=1).holding
    v += 1.5 * DT_MDL
    distance -= v * DT_MDL
  assert v > COMMIT_SPEED and sp.committed_id is None
  oncoming = scene([car(40., 5., vx=-10., age=VEL_TRUST_HITS)])
  stopped = False
  for _ in range(int(5. / DT_MDL)):
    d = sp.update(oncoming.objects, True, LEFT_TURN, distance, v, maneuver_id=1)
    assert d.holding and d.reason == "waiting for oncoming" and d.accel_cap == NO_ACCEL_CAP  # the curve, never harder
    a = plan_accel(d.accel_cap, d.speed_cap, v)
    if d.should_stop:
      a, stopped = min(a, -1.), True
    v = max(v + a * DT_MDL, 0.)
    distance -= v * DT_MDL
  assert stopped and v == 0. and distance > ONCOMING_ENTRY_M - 0.5
  # too far in to stop short of them comfortably: committed, the turn is finished
  sp = ScenePolicy()
  sp.update([], True, LEFT_TURN, HOLD_LINE_M + 0.5, 0., maneuver_id=2)
  d = sp.update(oncoming.objects, True, LEFT_TURN, ONCOMING_ENTRY_M + 2., 3., maneuver_id=2)
  assert not d.holding and sp.committed_id == 2
  # standing past the line (crept there): stays where it is, never creeps on toward the oncoming lanes
  d = ScenePolicy().update(oncoming.objects, True, LEFT_TURN, HOLD_LINE_M - 2., 0., maneuver_id=4)
  assert d.should_stop and d.speed_cap == 0.


def test_an_untrusted_track_in_the_oncoming_lanes_never_brakes_harder_than_the_curve():
  # a parked or waiting car (vx 0), or a ghost a few frames old, in the oncoming lanes 20 m past the junction as the car
  # nears its hold line: taken as coming until its velocity is trusted, so it holds, but only on the comfortable curve,
  # never the hard stop a trusted oncoming car gets (the planner would clip -5 to its emergency floor)
  for age in (1, 3, 12, VEL_TRUST_HITS - 1):
    d = policy([car(31., 5., vx=0., age=age)], maneuver=LEFT_TURN, distance=11., v_ego=4.46)
    assert d.holding and d.reason == "waiting for oncoming", age
    assert d.accel_cap >= -STOP_DECEL, (age, d.accel_cap)
  # a trusted one closing fast still gets the hard stop it needs
  d = policy([car(31., 5., vx=-12., age=VEL_TRUST_HITS)], maneuver=LEFT_TURN, distance=11., v_ego=4.46)
  assert d.holding and d.accel_cap < -STOP_DECEL


# --- hard braking needs a confirmed detection (Kellen, 2026-10-28: "phantom emergency braking is really dangerous") ---

def test_a_one_frame_phantom_pedestrian_at_10_m_never_brakes_harder_than_minus_2():
  # a pedestrian box in the path 10 m ahead at 10 m/s for one detector frame (the tracker's 2 hits, high confidence), then
  # gone: with the scene still up, and with the scene dropping out right after it (its stop carried), the plan never asks
  # for more than -2.0 m/s^2 on any frame
  for available_after in (True, False):
    sp, v = ScenePolicy(), 10.
    for k in range(int(3. / DT_MDL)):
      objects = [ped(10. - v * k * DT_MDL, 0.1, in_path=True, age=2, conf=0.95)] if k < 2 else []
      sc = scene(objects, True if k < 2 else available_after)
      d = sp.update(sc.objects, sc.available, None, 1000., v)
      assert plan_accel(d.accel_cap, d.speed_cap, v) >= MAP_DECEL_MAX - 1e-6, (available_after, k, d)
      assert not d.hard_brake
      v = max(v + max(plan_accel(d.accel_cap, d.speed_cap, v), -3.5) * DT_MDL, 0.)


def test_a_phantom_seen_a_few_frames_never_brakes_hard_until_confirmed():
  # hits growing one per detector frame (10 Hz) at a confidence under CONFIRM_CONF: under -2.0 only once confirmed
  sp, v = ScenePolicy(), 10.
  for k in range(8):
    age = 2 + k // 2
    sc = scene([ped(10., 0.1, in_path=True, age=age, conf=0.55)])
    d = sp.update(sc.objects, True, None, 1000., v)
    assert plan_accel(d.accel_cap, d.speed_cap, v) >= MAP_DECEL_MAX - 1e-6, (k, age)


def test_a_confirmed_pedestrian_at_10_m_brakes_hard_and_says_so_with_its_evidence():
  sp, v = ScenePolicy(), 10.
  with mock.patch('openpilot.selfdrive.turnpilot.scene_policy.cloudlog') as log:
    accels = []
    for _ in range(10):
      sc = scene([ped(10., 0.1, in_path=True, age=CONFIRM_HITS, conf=0.9, trackId=7)])
      d = sp.update(sc.objects, True, None, 1000., v)
      accels.append(plan_accel(d.accel_cap, d.speed_cap, v))
  assert min(accels) < MAP_DECEL_MAX - 1.
  assert d.hard_brake and d.reason == "pedestrian in path"
  events = [c for c in log.event.call_args_list if c.args[0] == "turnpilot.hard_brake"]
  assert len(events) == 1  # once, on its start
  kw = events[0].kwargs
  assert kw['track_id'] == 7 and kw['cls'] == 'pedestrian' and kw['hits'] == CONFIRM_HITS and kw['conf'] == 0.9
  assert kw['x'] == 10. and kw['reason'] == "pedestrian in path" and kw['accel_cap'] < MAP_DECEL_MAX
  # gone: no hard brake, and the next one is logged afresh
  d = sp.update([], True, None, 1000., v)
  assert not d.hard_brake


def test_a_hard_stop_for_a_trusted_oncoming_car_is_logged_too():
  sp = ScenePolicy()
  with mock.patch('openpilot.selfdrive.turnpilot.scene_policy.cloudlog') as log:
    for _ in range(10):
      sc = scene([car(31., 5., vx=-12., age=VEL_TRUST_HITS, trackId=3)])
      d = sp.update(sc.objects, True, LEFT_TURN, 11., 4.46, maneuver_id=1)
  assert d.hard_brake and d.reason == "waiting for oncoming"
  kw = next(c.kwargs for c in log.event.call_args_list if c.args[0] == "turnpilot.hard_brake")
  assert kw['track_id'] == 3 and kw['cls'] == 'car' and kw['vx'] == -12.


def _carried_junction_run(obj, drop_at: float, v0: float = 9., d0: float = 40.):
  """A left turn approached at v0 from d0 m: obj (a car dict) seen over the 1.5 m before drop_at, then the scene gone
  (detector stale) and the junction hold carried. The car takes the plan's brake. (worst plan accel, hard_brake events)"""
  sp, v, dist, worst = ScenePolicy(), v0, d0, 0.
  with mock.patch('openpilot.selfdrive.turnpilot.scene_policy.cloudlog') as log:
    for _ in range(int(8 / DT_MDL)):
      seen = drop_at + 1.5 > dist > drop_at
      if dist > drop_at:
        sc = scene([obj | {'x': dist + 20.}] if seen else [])
        d = sp.update(sc.objects, True, LEFT_TURN, dist, v, maneuver_id=1)
      else:
        d = sp.update([], False, LEFT_TURN, dist, v, maneuver_id=1)
      a = plan_accel(d.accel_cap, d.speed_cap, v)
      worst = min(worst, a)
      v = max(v + max(min(a, 0.), -3.5) * DT_MDL, 0.)
      dist -= v * DT_MDL
      if dist < 0:
        break
  return worst, [c.kwargs for c in log.event.call_args_list if c.args[0] == "turnpilot.hard_brake"]


def test_a_carried_junction_hold_keeps_its_tracks_confirmation_and_evidence():
  # skeptic G1: an untrusted 3-hit oncoming blip held the turn on the comfortable curve; the scene then dropped out and the
  # carried hold was rebuilt confirmed, with no evidence: -2.9 to -8.8 m/s^2, logged confirmed=true
  blip = car(0., 5., vx=-3., age=3, conf=0.5, trackId=9)
  for drop_at in (22., 20., 18., 16., 14.):
    worst, events = _carried_junction_run(blip, drop_at)
    assert worst >= MAP_DECEL_MAX - 1e-6, (drop_at, worst)
    assert not events, drop_at
  # a trusted oncoming car closing fast: its carried hold still brakes as hard as it needs, logged with its evidence
  trusted = car(0., 5., vx=-12., age=VEL_TRUST_HITS, trackId=3)
  worst, events = _carried_junction_run(trusted, 18.)
  assert worst < MAP_DECEL_MAX - 1e-6
  assert events and all(e['confirmed'] and e['track_id'] == 3 and e['cls'] == 'car' for e in events)


def test_a_crosswalk_hold_brakes_hard_only_for_a_confirmed_walker_and_logs_it():
  # a crosswalk walker (5-9 hits at 0.5, steadily seen) that isn't confirmed: the hold stays on the comfortable curve, where it
  # took the hold line's defaults (confirmed, no evidence) and asked for -7
  def run(conf, age0, age_max=100):
    sp, v, dist, worst = ScenePolicy(), 9., HOLD_LINE_M + 10., 0.
    with mock.patch('openpilot.selfdrive.turnpilot.scene_policy.cloudlog') as log:
      for k in range(int(3 / DT_MDL)):
        sc = scene([ped(dist + 2., -5., age=min(age0 + k, age_max), conf=conf, trackId=4)])
        d = sp.update(sc.objects, True, RIGHT_TURN, dist, v, maneuver_id=1)
        a = plan_accel(d.accel_cap, d.speed_cap, v)
        worst = min(worst, a)
        v = max(v + max(min(a, 0.), -3.5) * DT_MDL, 0.)
        dist -= v * DT_MDL
    return worst, [c.kwargs for c in log.event.call_args_list if c.args[0] == "turnpilot.hard_brake"]
  worst, events = run(0.5, 5, 9)  # under CONFIRM_STEADY_HITS, under CONFIRM_CONF
  assert worst >= MAP_DECEL_MAX - 1e-6 and not events
  worst, events = run(0.9, 10)
  assert worst < MAP_DECEL_MAX - 1e-6
  assert events and events[0]['track_id'] == 4 and events[0]['reason'] == "pedestrian in crosswalk" and events[0]['confirmed']


def _flicker_run(every: int, conf: float = 0.7, x0: float = 90., v0: float = 10.):
  """A stationary in-path person x0 m ahead on the road camera, detected on 1 of every `every` detector runs (10 Hz), through
  the real SceneTracker and ScenePolicy, the car taking the plan's brake. (worst plan accel, hard_brake events)"""
  from openpilot.cereal import messaging
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.selfdrive.turnpilot.detector.decoder import PERSON
  from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import calibration, car_state, detections
  from openpilot.selfdrive.turnpilot.scene.tracker import SceneTracker
  tr, sp, cal = SceneTracker(DEVICE_CAMERAS[('mici', 'os04c10')], 'mici'), ScenePolicy(), calibration()
  v, x, t, det_frame, worst = v0, x0, 0., 0, 0.
  with mock.patch('openpilot.selfdrive.turnpilot.scene_policy.cloudlog') as log:
    for k in range(int(12 / DT_MDL)):
      t += DT_MDL
      x -= v * DT_MDL
      if x < 3.:
        break
      if k % 2 == 0:
        det_frame += 1
        objs = [(PERSON, x, 0.2, conf)] if det_frame % every == 0 else []
        tr.update(detections(tr, 'road', t, objs, frame_id=det_frame), cal, car_state(v), now=t)
      msg = messaging.new_message('turnpilotScene')
      tr.available = True
      tr.fill_scene(msg.turnpilotScene)
      sc = msg.as_reader().turnpilotScene
      d = sp.update(sc.objects, sc.available, None, 1000., v)
      a = plan_accel(d.accel_cap, d.speed_cap, v)
      worst = min(worst, a)
      v = max(v + max(min(a, 0.), -3.5) * DT_MDL, 0.)
  return worst, [c.kwargs for c in log.event.call_args_list if c.args[0] == "turnpilot.hard_brake"]


def test_a_phantom_seen_about_once_a_second_never_brakes_hard():
  # skeptic G1: seen on 1 in 8-14 detector runs it built up 5-12 hits with no braking between them, then one new frame
  # asked for -2.6 to -8.8 m/s^2 cold. Sparse hits never confirm: the steady curve only
  for every in (3, 4, 6, 8, 10, 12, 14):
    worst, events = _flicker_run(every)
    assert worst >= MAP_DECEL_MAX - 1e-6, (every, worst)
    assert not events, every
  # seen on every run or every other one, it is slowed for from far out, gently
  for every in (1, 2):
    worst, _ = _flicker_run(every)
    assert MAP_DECEL_MAX - 1e-6 <= worst < -1., (every, worst)


def test_confirmation_needs_recent_hits_and_is_lost_when_the_track_coasts_out():
  sp, v = ScenePolicy(), 10.
  def frame(objs):
    sc = scene(objs)
    return sp.update(sc.objects, True, None, 1000., v)
  # 3 hits at 0.9, then one a second, published for half a second after each: never confirmed, never below -2.0
  for k in range(5):
    frame([ped(30., 0.1, in_path=True, age=2 + k // 4, conf=0.9, trackId=1)])  # 2-3 hits in the first quarter second
  assert not sp.tracks[1].confirmed
  age = 3
  for k in range(int(10. / DT_MDL)):
    if k % int(1. / DT_MDL) == 0:
      age += 1
    d = frame([ped(10., 0.1, in_path=True, age=age, conf=0.9, trackId=1)] if k % int(1. / DT_MDL) < 10 else [])
    assert plan_accel(d.accel_cap, d.speed_cap, v) >= MAP_DECEL_MAX - 1e-6, k
  # confirmed while seen steadily, lost the frame the tracker stops publishing it, back at once with steady recent hits
  sp = ScenePolicy()
  for k in range(4):
    frame([ped(30., 0.1, in_path=True, age=2 + 2 * k, conf=0.9, trackId=2)])
  assert sp.tracks[2].confirmed
  frame([])
  assert not sp.tracks[2].confirmed
  frame([ped(30., 0.1, in_path=True, age=10, conf=0.9, trackId=2)])
  assert sp.tracks[2].confirmed
  # a track first seen with its hits (the tracker publishes from MIN_HITS on) and coasted out a second: one new hit isn't enough
  for _ in range(int(1.2 / DT_MDL)):
    frame([])
  frame([ped(30., 0.1, in_path=True, age=11, conf=0.9, trackId=2)])
  assert not sp.tracks[2].confirmed


def test_a_standing_car_a_latch_holds_is_never_committed_by_a_reading():
  # skeptic G4 (blocker 3): held at an unprotected left's line through a long oncoming stream, one navd reading 6 m nearer put
  # the follower's distance under ONCOMING_ENTRY_M: committed_past_line fired on a car that hadn't moved, the scene committed
  # to the junction, and the left (committed first) stopped judging with the latch still holding the car. A car a latch
  # holds (an extra stop with shouldStop, the car standing) is where the latch put it: a reading never commits it
  latch = Stop(-0.5, 0., True, "Unprotected left: waiting for a gap", 0.)
  sp = ScenePolicy()
  sp.update([], True, LEFT_TURN, HOLD_LINE_M + 1., 0., maneuver_id=3, extra_stops=[latch])
  sp.update([], True, LEFT_TURN, ONCOMING_ENTRY_M - 0.2, 0., maneuver_id=3, extra_stops=[latch])
  assert sp.committed_id is None
  # moving through (its go), the same reading commits it, as before
  sp.update([], True, LEFT_TURN, ONCOMING_ENTRY_M - 0.2, 1., maneuver_id=3)
  assert sp.committed_id == 3
  # and nothing holding it, standing there, it is past the line: committed (a driver who stopped in the junction)
  sp = ScenePolicy()
  sp.update([], True, LEFT_TURN, ONCOMING_ENTRY_M - 0.2, 0., maneuver_id=4)
  assert sp.committed_id == 4


def _burst_run(hits, x0=14., v0=10., conf=0.7, lost_at=None, misses=(), secs=6.):
  """A stationary in-path person x0 m ahead on the road camera at 10 Hz through the real SceneTracker and ScenePolicy, the car
  taking the plan's brake (the car's own floor -3.5): detected on the first `hits` detector runs (all of them with None), or
  while it is further than lost_at m, skipping the runs in misses. Returns ([(s since the last detection, plan accel, v,
  hard_brake, m to the person)] of every frame, hard_brake events)."""
  from openpilot.cereal import messaging
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.selfdrive.turnpilot.detector.decoder import PERSON
  from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import calibration, car_state, detections
  from openpilot.selfdrive.turnpilot.scene.tracker import SceneTracker
  tr, sp, cal = SceneTracker(DEVICE_CAMERAS[('mici', 'os04c10')], 'mici'), ScenePolicy(), calibration()
  v, s, t, run, last, rows = v0, 0., 0., 0, None, []
  with mock.patch('openpilot.selfdrive.turnpilot.scene_policy.cloudlog') as log:
    for k in range(int(secs / DT_MDL)):
      t += DT_MDL
      s += v * DT_MDL
      x = x0 - s
      if k % 2 == 0:
        run += 1
        seen = (hits is None or run <= hits) and (lost_at is None or x > lost_at) and run not in misses and x > 0.5
        if seen:
          last = t
        tr.update(detections(tr, 'road', t, [(PERSON, x, 0.2, conf)] if seen else [], frame_id=run), cal, car_state(v), now=t)
      msg = messaging.new_message('turnpilotScene')
      tr.available = True
      tr.fill_scene(msg.turnpilotScene)
      sc = msg.as_reader().turnpilotScene
      d = sp.update(sc.objects, sc.available, None, 1000., v)
      a = plan_accel(d.accel_cap, d.speed_cap, v)
      rows.append((t - last if last is not None else -1., a, v, d.hard_brake, x))
      v = max(v + max(min(a, 0.), -3.5) * DT_MDL, 0.)
  return rows, [c.kwargs for c in log.event.call_args_list if c.args[0] == "turnpilot.hard_brake"]


def test_a_phantom_burst_the_tracker_coasts_on_never_brakes_hard_once_unseen():
  # skeptic G1 (residual): a stationary in-path 'person' at 0.7 on 5-8 detector runs from 14 m, then never seen, car at 10 m/s.
  # The tracker keeps a close one published CLOSE_HOLD_S and ScenePolicy kept its confirmation while published, so every frame
  # below -2.0 came after the last detection, for up to 1.65 s (worst -5.5 to -8.25). Now the hard tier needs the detector
  # still matching it (HARD_MISSED_RUNS): below -2.0 only within that many missed runs of its last detection, the stop
  # itself still on the curve
  window = (HARD_MISSED_RUNS + 1) * 2 * DT_MDL + 1e-6
  for hits in (5, 6, 8):
    rows, events = _burst_run(hits)
    hard = [r for r in rows if r[1] < MAP_DECEL_MAX - 1e-6]
    assert all(0. <= r[0] <= window for r in hard), (hits, hard[-1:])
    assert all(r[1] >= MAP_DECEL_MAX - 1e-6 and not r[3] for r in rows if r[0] > window), hits
    assert len(events) <= 1, hits
  # under CONFIRM_HITS it never brakes hard at all
  rows, events = _burst_run(4)
  assert min(r[1] for r in rows) >= MAP_DECEL_MAX - 1e-6 and not events


def test_a_person_seen_on_every_run_keeps_the_hard_stop_through_a_short_dropout():
  # a real person in the path, seen on every run but two in a row (HARD_MISSED_RUNS): the hard stop holds through the dropout
  rows, events = _burst_run(None, x0=20., v0=10., misses=(9, 10))
  hard_t = [k for k, r in enumerate(rows) if r[3]]
  assert hard_t and len(events) == 1
  assert all(rows[k][3] for k in range(hard_t[0], hard_t[-1] + 1))  # never lapsed between its first and last frame
  assert min(r[1] for r in rows) < MAP_DECEL_MAX - 1.


def test_a_person_lost_at_the_stop_point_keeps_the_hard_stop_while_the_tracker_holds_them():
  # a real person seen on every run until they drop out of view within LOST_CLOSE_M (they fill the frame or leave it: the
  # BeamNG mannequin at 5.3 m), the car still moving: the hard tier stays while the tracker keeps them (CLOSE_HOLD_S)
  rows, events = _burst_run(None, x0=16., v0=12., lost_at=5.)
  lost = [r for r in rows if r[0] > (HARD_MISSED_RUNS + 1) * 2 * DT_MDL + 1e-6 and r[2] > 0.5 and r[4] > 0.5]
  assert lost and all(r[1] < MAP_DECEL_MAX - 1e-6 and r[3] for r in lost)
  assert len(events) == 1


def test_confirmation_lapses_with_missed_runs_and_comes_back_on_a_match():
  sp, v = ScenePolicy(), 10.

  def frame(age, missed, x=10.):
    sc = scene([ped(x, 0.1, in_path=True, age=age, conf=0.9, trackId=3, missedRuns=missed)])
    return sp.update(sc.objects, True, None, 1000., v)
  for k in range(4):
    frame(2 + 2 * k, 0)
  assert sp.tracks[3].confirmed
  frame(8, HARD_MISSED_RUNS)
  assert sp.tracks[3].confirmed
  d = frame(8, HARD_MISSED_RUNS + 1)
  assert not sp.tracks[3].confirmed and not d.hard_brake and d.accel_cap == NO_ACCEL_CAP
  frame(9, 0)
  assert sp.tracks[3].confirmed
  # last matched within LOST_CLOSE_M confirmed: kept while published, missed runs or not
  frame(10, 0, x=LOST_CLOSE_M - 0.5)
  assert frame(10, 12, x=LOST_CLOSE_M - 2.).hard_brake and sp.tracks[3].confirmed


# --- the curb-pedestrian speed cap (owner decision, 2026-10-30: BUILD_FINDINGS D3-real option 2, TurnPilotCurbPedCap) ---

def curb_run(objects_at, secs=8., v0=13., path=None, curb_cap=True, sp=None):
  """A point-mass car under a ScenePolicy (curb_cap as TurnPilotCurbPedCap) for secs from v0; objects_at(t) gives the scene's
  objects. Returns (sp, [(t, v, decision, the planner's accel from the caps)])."""
  sp = sp or ScenePolicy()
  sp.curb_cap = curb_cap
  v, rows = v0, []
  for k in range(int(round(secs / DT_MDL))):
    t = k * DT_MDL
    sc = scene(objects_at(t))
    d = sp.update(sc.objects, True, None, 1000., v, maneuver_id=0, path=path)
    a = plan_accel(d.accel_cap, d.speed_cap, v)
    rows.append((t, v, d, a))
    v = max(v + min(a, (v0 - v) / 2.) * DT_MDL, 0.)
  return sp, rows


def steady_curb_walker(x, y, every=1, conf=0.7):
  """A person track at (x, y) matched on every `every`-th frame from 2 hits (the tracker publishes from MIN_HITS)."""
  return lambda t: [ped(x, y, age=2 + int(round(t / DT_MDL)) // every, conf=conf, trackId=7)]


def test_a_steady_person_by_the_road_caps_the_speed_on_the_comfortable_curve_only():
  """Owner decision (BUILD_FINDINGS D3-real option 2): a published person track with crosswalk-grade steadiness
  (crosswalk_walker, CROSSWALK_STEADY_HITS within CROSSWALK_STEADY_S) within CURB_PED_EDGE_M of the path's edge
  (PATH_HALF_WIDTH_M + 3 m of the model path) and CURB_PED_X_M ahead caps the speed at CURB_PED_SPEED, 5.6 m/s, through the
  speed cap alone: no accel cap, and nothing from it asks for more than -CURB_PED_DECEL (1.5 m/s^2). Never a stop, never a
  hard brake: the track's confirmation is untouched (it is kept, so a step-out finds it warm)."""
  from openpilot.selfdrive.turnpilot.scene_policy import CURB_PED_DECEL, CURB_PED_SPEED
  sp, rows = curb_run(steady_curb_walker(35., 3.8), secs=6.)
  capped = [(t, v, d, a) for t, v, d, a in rows if d.speed_cap < NO_SPEED_CAP]
  assert capped and capped[0][0] < 1.
  assert all(d.accel_cap == NO_ACCEL_CAP and not d.should_stop and not d.hard_brake for _, _, d, _ in rows)
  assert all(a >= -CURB_PED_DECEL - 1e-6 for _, _, _, a in rows)
  assert all(d.speed_cap >= CURB_PED_SPEED - 1e-6 for _, _, d, _ in capped)
  assert rows[-1][1] < CURB_PED_SPEED + 0.3 and capped[-1][2].reason == "pedestrian by the road"
  assert not any(d.speed_cap < NO_SPEED_CAP for t, _, d, _ in rows if t < 0.1)  # not before its fifth hit


def test_the_curb_cap_never_takes_a_flicker_a_weak_track_or_one_far_off_the_path():
  for objects_at, why in ((lambda t: [ped(30., 3.5, age=min(2 + int(round(t / DT_MDL)), 4), conf=0.8, trackId=7)], "4 hits"),
                          (steady_curb_walker(30., 3.5, conf=0.45), "under CROSSWALK_CONF"),
                          (steady_curb_walker(30., 3.5, every=12), "one hit in 0.6 s"),
                          (steady_curb_walker(30., 5.), "4.5 m or more off the path"),
                          (steady_curb_walker(45., 3.5), "past CURB_PED_X_M"),
                          (lambda t: [car(25., 3.5, age=40)], "not a person")):
    _, rows = curb_run(objects_at, secs=5.)
    assert all(d.speed_cap == NO_SPEED_CAP for _, _, d, _ in rows), why
  # off with TurnPilotCurbPedCap off
  _, rows = curb_run(steady_curb_walker(35., 3.8), secs=5., curb_cap=False)
  assert all(d.speed_cap == NO_SPEED_CAP for _, _, d, _ in rows)


def test_the_curb_cap_follows_the_model_path_and_lets_go_a_second_after_the_last_track():
  import numpy as np
  # the path bends right, 2 m right at 35 m: a person 3.5 m left of the straight line ahead is 5.5 m off the path, one 3.5 m
  # right of it 1.5 m off it (inside the cap's band; not in_path, the tracker's own flag unset, so no stop)
  path = (np.array([0., 50.]), np.array([0., -2.0 * 50. / 35.]))
  _, rows = curb_run(steady_curb_walker(35., 3.5), secs=4., path=path)
  assert all(d.speed_cap == NO_SPEED_CAP for _, _, d, _ in rows)
  _, rows = curb_run(steady_curb_walker(35., -3.5), secs=4., path=path)
  assert any(d.speed_cap < NO_SPEED_CAP for _, _, d, _ in rows)
  # published for 3 s, then gone: the cap holds CURB_PED_HOLD_S more, then lets go
  from openpilot.selfdrive.turnpilot.scene_policy import CURB_PED_HOLD_S
  walker = steady_curb_walker(35., 3.8)
  _, rows = curb_run(lambda t: walker(t) if t < 3. else [], secs=6.)
  held = [t for t, _, d, _ in rows if d.speed_cap < NO_SPEED_CAP and t >= 3.]
  assert held and abs(max(held) - (3. + CURB_PED_HOLD_S)) < 2 * DT_MDL, max(held)


def test_the_curb_cap_is_logged_once_per_track():
  with mock.patch('openpilot.selfdrive.turnpilot.scene_policy.cloudlog') as log:
    curb_run(steady_curb_walker(35., 3.8), secs=4.)
  calls = [c for c in log.event.call_args_list if c.args[0] == 'turnpilot.curb_ped_cap']
  assert len(calls) == 1 and {'x', 'y', 'conf', 'hits', 'track_id'} <= set(calls[0].kwargs), calls


# --- a pedestrian's velocity is trusted only at a person's speed (peds P2) ---

def plain_ped(x, y, vx=0., vy=0., track_id=0, **fields):
  """A tracked pedestrian as a plain object, to see that the policy never writes to the objects it reads."""
  f = {'trackId': track_id, 'cls': 'pedestrian', 'conf': 0.9, 'x': x, 'y': y, 'vx': vx, 'vy': vy, 'age': 10, 'inPath': False, 'camera': 'road',
       'box': [], 'missedRuns': 0, **fields}
  return SimpleNamespace(**f)


def test_a_pedestrian_faster_than_a_run_has_no_velocity_the_policy_trusts():
  # public comma four route 5afee5161ea1dcde/00000003--b8ae16aae0 segment 26: walkers behind a guardrail on a curve read
  # vx -5 to -10 m/s (range noise and ego motion), the entering rule took one as walking into the path and the hard tier ramped
  # to -7.25 m/s^2 at 37 km/h. A pedestrian track faster than PED_MAX_SPEED reads as standing to ScenePolicy.update
  # 20 m out, 6 m to the left, read crossing in at 4 m/s: the entering rule's |6 - 4 x 1.5| < 1.5, no longer
  sc = scene([ped(20., 6., vy=-4.)])
  d = ScenePolicy().update(sc.objects, True, None, 1000., 6.)
  assert not d.holding and d.reason == "" and d.speed_cap == NO_SPEED_CAP
  # a crossing walker at 1.4 m/s still walks into the path, and is stopped for on the comfortable curve
  sc = scene([ped(20., 3., vy=-1.4)])
  d = ScenePolicy().update(sc.objects, True, None, 1000., 6.)
  assert d.holding and d.reason == "pedestrian in path" and d.rest_d == 20. - PED_STOP_GAP_M
  assert d.accel_cap == NO_ACCEL_CAP and not d.hard_brake and plan_accel(d.accel_cap, d.speed_cap, 6.) >= -STOP_DECEL
  # what the policy reads is never written back: turnpilotScene keeps the tracker's velocity for everyone else
  objects = [plain_ped(20., 6., vy=-4.), plain_ped(20., 3., vy=-1.4, track_id=1), plain_ped(12., 0., vx=5., track_id=2)]
  sp = ScenePolicy()
  for _ in range(5):
    sp.update(objects, True, RIGHT_TURN, 20., 6.)
  assert [(o.vx, o.vy) for o in objects] == [(0., -4.), (0., -1.4), (5., 0.)]


def test_a_confirmed_pedestrian_faster_than_a_run_is_stopped_for_never_followed():
  # 12 m ahead in the path, confirmed, read moving on at 5 m/s: a person at that speed is range noise or a rider without a
  # bicycle box, so the policy stops short of them as if they stood, rather than following them into a 2 m gap
  sp = ScenePolicy()
  sc = scene([ped(12., 0., vx=5.)])
  for _ in range(3):
    d = sp.update(sc.objects, True, None, 1000., 8.)
  assert d.reason == "pedestrian in path" and d.rest_d == 12. - PED_STOP_GAP_M
  assert sp.tracks[0].confirmed and d.hard_brake
  # under PED_MAX_SPEED (a jogger) it is followed as today
  sp = ScenePolicy()
  sc = scene([ped(12., 0., vx=3.)])
  for _ in range(3):
    d = sp.update(sc.objects, True, None, 1000., 8.)
  assert d.reason == "following pedestrian" and d.rest_d is None
