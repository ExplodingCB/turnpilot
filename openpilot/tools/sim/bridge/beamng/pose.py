"""BeamNG AdvancedIMU readings turned into what a comma four's IMU and GPS would report. Pure math, no game connection."""
import math

import numpy as np

GRAVITY = 9.81
TELEPORT_JUMP = 2.0  # m between 100 Hz IMU readings
# the car's speed for openpilot (car_speed): the pose's, standing when the wheels have stopped and the road under the IMU
# moves slower than STANDSTILL_MPS (a settling body's jitter is no speed: ESP_vehicleStandstillSts holds)
WHEELS_STOPPED_MPS = 0.1
STANDSTILL_MPS = 0.3
SPEED_SOURCES = ('ground', 'wheels')


def car_speed(wheelspeed: float, v_fwd: float | None, reverse: bool = False, source: str = 'ground') -> float:
  """The car's speed for openpilot's wheel speeds, m/s, negative in reverse. source 'ground': the pose's forward speed of
  the road under the IMU (Pose.v_fwd), the wheels' only before the first pose; 'wheels': electrics.wheelspeed, which over-read
  by 10-20% while the car sped up or braked (BeamNG smoke S08's braking approach, diag S02's rolling start), so turnpilotd's
  route follower, dead-reckoning navd's readings on vEgo, put the junction 6-13 m too near (build findings D4-speed)."""
  if source == 'wheels' or v_fwd is None:
    speed = float(wheelspeed)
  elif abs(wheelspeed) < WHEELS_STOPPED_MPS and abs(v_fwd) < STANDSTILL_MPS:
    speed = 0.
  else:
    speed = abs(float(v_fwd))
  return -abs(speed) if reverse else speed


def after_last_jump(readings: list, jump: float = TELEPORT_JUMP) -> int:
  """Index of the first reading after the last position jump (a teleport) in time-ordered readings, or 0 if none."""
  for k in range(len(readings) - 1, 0, -1):
    if np.linalg.norm(np.subtract(readings[k]['pos'], readings[k - 1]['pos'])) > jump:
      return k
  return 0


def beamng_quat(fwd, up) -> tuple:
  """BeamNG's rotation quaternion (x, y, z, w) of a vehicle with these world-space forward and up axes: its local -y
  is forward and z up, and BeamNG turns a quaternion the other way round from the usual (see scenario.heading_quat)."""
  y, z = -np.asarray(fwd, dtype=float), np.asarray(up, dtype=float)
  x = np.cross(y, z)
  m = np.array([x, y, z]).T  # columns: the local axes in world space
  w = math.sqrt(max(1. + m[0, 0] + m[1, 1] + m[2, 2], 1e-12)) / 2
  q = np.array([(m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)])
  if w < 1e-3:  # near a half turn: from the largest diagonal instead
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    r = math.sqrt(max(1. + m[i, i] - m[j, j] - m[k, k], 1e-12))
    q = np.zeros(3)
    q[i], q[j], q[k] = r / 2, (m[j, i] + m[i, j]) / (2 * r), (m[k, i] + m[i, k]) / (2 * r)
    w = (m[k, j] - m[j, k]) / (2 * r)
  q, w = -q, w  # BeamNG's sense: the conjugate
  n = math.sqrt(float(q @ q) + w * w)
  return (float(q[0] / n), float(q[1] / n), float(q[2] / n), float(w / n))


class Pose:
  """Pose, velocity, specific force and angular rate from AdvancedIMU readings (100 Hz sim time, sensor X forward, Y up,
  Z right in world space). The angular rate comes from differencing the sensor axes, not from the readings' angVel:
  live, angVel/angVelSmooth carry the yaw rate in their third component, the documented "right" axis, so their frame is
  not the documented sensor frame, while the axes in dirX/dirY/dirZ are exact."""
  WINDOW = 0.05  # s of readings to difference for velocity and angular rate

  def __init__(self, history, latest, ground_h: float = 0.):
    """ground_h: m from the sensor down its up axis to the road, for v_fwd, the forward speed of the road under it: the
    sensor's own swings with the body's pitch (1.34 m up at the windshield, about 0.2 m/s rocking on a stop)."""
    r1 = history[-1]
    r0 = next((r for r in reversed(history) if r1['time'] - r['time'] >= self.WINDOW), history[0])
    self.t_sim = r1['time']
    self.pos = np.array(r1['pos'])
    fwd, up, right = (np.array(r1[k]) for k in ('dirX', 'dirY', 'dirZ'))
    self.fwd, self.up = fwd, up
    dt = r1['time'] - r0['time']
    if dt > 0:
      self.vel = (self.pos - np.array(r0['pos'])) / dt
      ground0 = np.array(r0['pos']) - ground_h * np.array(r0['dirY'])
      self.v_fwd = float((self.pos - ground_h * up - ground0) @ fwd) / dt
      # world angular velocity from the orientation change: dR/dt = [w]x R
      R0 = np.array([r0['dirX'], r0['dirY'], r0['dirZ']]).T
      R1 = np.array([fwd, up, right]).T
      Wm = (R1 @ R0.T - np.eye(3)) / dt
      omega = np.array([Wm[2, 1] - Wm[1, 2], Wm[0, 2] - Wm[2, 0], Wm[1, 0] - Wm[0, 1]]) / 2
    else:
      self.vel, omega, self.v_fwd = np.zeros(3), np.zeros(3), 0.
    acc = np.mean([r['accSmooth'] for r in latest], axis=0)  # kinematic, in sensor axes (is_using_gravity=False)
    # the car's acceleration from its forward speed across the history (logged as a_long): accSmooth bounces with the
    # suspension (single readings of +-3 m/s^2 at a steady speed)
    t = np.array([r['time'] for r in history])
    if len(t) >= 6 and t[-1] - t[0] > 0.08:
      v_fwd = (np.diff(np.array([r['pos'] for r in history]), axis=0) @ fwd) / np.maximum(np.diff(t), 1e-4)
      self.a_long = float(np.polyfit((t[1:] + t[:-1]) / 2 - t[-1], v_fwd, 1)[0])
    else:
      self.a_long = float(acc[0])
    # an accelerometer reads specific force, a - g; a comma four's sensor axes are (up, left, back), see locationd
    f = acc + GRAVITY * np.array([fwd[2], up[2], right[2]])
    self.accel = (float(f[1]), float(-f[2]), float(-f[0]))
    self.gyro = (float(omega @ up), float(-(omega @ right)), float(-(omega @ fwd)))
    self.yaw_rate = float(omega @ up)

  @property
  def bearing(self) -> float:
    return math.degrees(math.atan2(self.fwd[0], self.fwd[1])) % 360.0  # compass: x east, y north
