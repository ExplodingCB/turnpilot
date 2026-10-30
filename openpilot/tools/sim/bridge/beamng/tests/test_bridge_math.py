import math

import numpy as np

# no beamngpy here: these run without the game's client library installed
from openpilot.tools.sim.bridge.beamng import frame
from openpilot.tools.sim.bridge.beamng.control import AccelController
from openpilot.tools.sim.bridge.beamng.pose import (STANDSTILL_MPS, WHEELS_STOPPED_MPS, Pose, after_last_jump, beamng_quat,
                                                    car_speed)
from openpilot.tools.sim.bridge.beamng.scenario import heading_quat


def imu_reading(t, pos, yaw, acc=(0.0, 0.0, 0.0)):
  """An AdvancedIMU reading: sensor X forward, Y up, Z right, for a level car heading `yaw` (counterclockwise from east)."""
  fwd = [math.cos(yaw), math.sin(yaw), 0.0]
  right = [math.sin(yaw), -math.cos(yaw), 0.0]
  return {'time': t, 'pos': list(pos), 'dirX': fwd, 'dirY': [0.0, 0.0, 1.0], 'dirZ': right, 'accSmooth': list(acc)}


class TestPose:
  def test_left_turn_and_rest(self):
    # 10 m/s north, turning left at 0.2 rad/s, accelerating forward at 1 m/s^2
    v, r, a = 10.0, 0.2, 1.0
    hist = []
    for i in range(10):
      t = i * 0.01
      yaw = math.pi / 2 + r * t
      hist.append(imu_reading(t, (0.0, v * t + a * t * t / 2, 0.0), yaw, acc=(a, 0.0, 0.0)))
    p = Pose(hist, hist[-2:])
    assert abs(p.yaw_rate - r) < 1e-3
    assert abs(p.vel[1] - (v + a * 0.065)) < 0.01  # average over the 50 ms window
    assert abs(p.a_long - a) < 0.05
    assert abs(p.bearing - (90.0 - math.degrees(math.pi / 2 + r * 0.09)) % 360.0) < 1e-6  # compass
    up, left, back = p.accel
    # comma four sensor axes (up, left, back): at rest the accelerometer reads +g up; locationd's meas = [-v2, -v1, -v0]
    assert abs(up - 9.81) < 1e-6 and abs(left) < 1e-6 and abs(back + 1.0) < 1e-6
    gyro_up, _, _ = p.gyro
    assert abs(gyro_up - r) < 1e-3  # device yaw rate (z down) = -gyro_up: negative in a left turn

  def test_teleport_inside_a_batch(self):
    rs = [imu_reading(i * 0.01, (0.1 * i, 0.0, 0.0), 0.0) for i in range(10)]
    assert after_last_jump(rs) == 0
    rs[6:] = [imu_reading(i * 0.01, (500.0 + 0.1 * i, 0.0, 0.0), 0.0) for i in range(6, 10)]
    assert after_last_jump(rs) == 6
    rs[8:] = [imu_reading(i * 0.01, (-300.0, 0.0, 0.0), 0.0) for i in range(8, 10)]
    assert after_last_jump(rs) == 8


class TestGroundSpeed:
  """The car's speed for openpilot (BeamNG build findings D4-speed): electrics.wheelspeed over-read by 10-20% while the car
  sped up or braked (smoke S08 14% on its braking approach, diag S02 19.5% on the rolling start), so turnpilotd's route
  follower, dead-reckoning navd's readings on vEgo, put the junction 6-13 m too near. The pose's ground velocity is what the
  car really does: the forward speed of the road-level point under the IMU, which the body's pitch doesn't swing."""
  def test_forward_speed_on_a_straight_and_in_a_turn(self):
    hist = [imu_reading(i * 0.01, (0.0, 10.0 * i * 0.01, 0.0), math.pi / 2) for i in range(10)]
    assert abs(Pose(hist, hist[-2:], ground_h=1.34).v_fwd - 10.0) < 1e-6
    v, r = 8.0, 0.3  # a left turn: along the arc
    hist = [imu_reading(i * 0.01, (v / r * math.sin(r * i * 0.01), v / r * (1 - math.cos(r * i * 0.01)), 0.0), r * i * 0.01)
            for i in range(10)]
    assert abs(Pose(hist, hist[-2:], ground_h=1.34).v_fwd - v) < 0.01

  def test_backwards_is_negative(self):
    hist = [imu_reading(i * 0.01, (-2.0 * i * 0.01, 0.0, 0.0), 0.0) for i in range(10)]
    assert abs(Pose(hist, hist[-2:]).v_fwd + 2.0) < 1e-6

  def test_a_pitching_body_at_a_standstill_reads_no_speed(self):
    # stopped, the body rocking nose-down at 0.15 rad/s about a point on the road: the windshield mount, 1.34 m up, swings
    # forward at about 0.2 m/s, the road under it doesn't move
    h, q = 1.34, 0.15
    hist = []
    for i in range(10):
      t = i * 0.01
      th = q * t  # pitch, nose down
      fwd, up = [math.cos(th), 0.0, -math.sin(th)], [math.sin(th), 0.0, math.cos(th)]
      hist.append({'time': t, 'pos': [h * math.sin(th), 0.0, h * math.cos(th)], 'dirX': fwd, 'dirY': up, 'dirZ': [0.0, -1.0, 0.0],
                   'accSmooth': [0.0, 0.0, 0.0]})
    p = Pose(hist, hist[-2:], ground_h=h)
    assert float(np.dot(p.vel, p.fwd)) > 0.15  # the mount itself
    assert abs(p.v_fwd) < 1e-3

  def test_car_speed_from_the_ground_and_the_wheels(self):
    assert car_speed(9.3, 8.1) == 8.1  # the wheels over-reading: the ground
    assert car_speed(0.0, 0.004) == 0.0  # standing: no jitter, ESP_vehicleStandstillSts holds
    assert car_speed(WHEELS_STOPPED_MPS / 2, STANDSTILL_MPS * 2) == STANDSTILL_MPS * 2  # sliding: what the car does
    assert car_speed(0.5, 0.4) == 0.4  # creeping
    assert car_speed(2.0, -1.8, reverse=True) == -1.8 and car_speed(2.0, 1.8, reverse=True) == -1.8  # reverse: negative, as before
    assert car_speed(9.3, 8.1, source='wheels') == 9.3 and car_speed(9.3, 8.1, reverse=True, source='wheels') == -9.3
    assert car_speed(9.3, None) == 9.3  # no pose yet: the wheels


class TestFrame:
  def test_round_trip(self):
    for x, y, z in ((-717.1, 101.0, 118.7), (1351.0, 2198.0, 0.0), (-1603.0, -1216.0, 50.0)):
      lat, lon, h = frame.to_latlon(x, y, z)
      assert np.allclose(frame.from_latlon(lat, lon, h), (x, y, z), atol=1e-6)


class TestScenario:
  def test_heading_quat_matches_known_spawn(self):
    # the known-good spawn (0, 0, 0.3826834, 0.9238795) faces south-west
    assert np.allclose(heading_quat(math.radians(-135)), (0.0, 0.0, 0.3826834, 0.9238795), atol=1e-6)


class TestAccelController:
  def test_signs(self):
    c = AccelController()
    th, br = c.update(1.0, 10.0, 0.01)
    assert th > 0 and br == 0
    c.reset()
    th, br = c.update(-2.0, 10.0, 0.01)
    assert br > 0 and th == 0
    th, br = c.update(-0.5, 0.1, 0.01)
    assert th == 0 and br > 0  # standstill hold

  def test_surge_lets_go_of_the_windup(self):
    # the car falls behind a +0.7 request for 3 s (a tight turn), then surges once out of it
    c = AccelController()
    v = 4.0
    for _ in range(300):
      th_wound, _ = c.update(0.7, v, 0.01, -1.0)
      v -= 0.01
    ff = 0.7 * 0.22 + 0.004 * v
    assert th_wound > ff + 0.5  # the PI asks for far more than the request needs
    th, _ = c.update(0.7, v, 0.01, 3.0)
    assert th < ff + 0.01  # measured 3 m/s^2 on a 0.7 request: back to the feedforward


def test_beamng_quat_matches_heading_quat():
  for heading in (0., 0.7, math.pi / 2, 2.5, math.pi, -2.0):
    fwd = (math.cos(heading), math.sin(heading), 0.)
    q, ref = np.array(beamng_quat(fwd, (0., 0., 1.))), np.array(heading_quat(heading))
    assert min(np.abs(q - ref).max(), np.abs(q + ref).max()) < 1e-9, heading
