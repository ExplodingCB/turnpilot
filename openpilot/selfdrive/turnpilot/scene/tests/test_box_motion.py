"""RawBox.motion: the road camera's raw boxes (full pass and far crop) associated run to run, their over-ground motion fitted
with the car's own taken out (tracker.BoxMotion)."""
import math

import numpy as np

from openpilot.cereal import messaging
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.detector.decoder import CAR
from openpilot.selfdrive.turnpilot.scene.ranging import calib_from_view
from openpilot.selfdrive.turnpilot.scene.tracker import MOTION_MIN_SPAN_S, SceneTracker

CAMS = DEVICE_CAMERAS[('mici', 'os04c10')]
RPY, HEIGHT = [0.0001, -0.0554, 0.0112], 1.238
FAR_CROP = (256, 188, 896, 572)
CAR_W, CAR_H = 1.8, 1.5

# a held left at night (a drive on the test car): the car standing at its line, the far crop and the road camera's full pass
# alternating. Road camera pixels of the car boxes under the horizon's band, per run: (s, view, boxes). The 428-515 box is a car
# parked at the near end of the far curb, 536-576 and 504-547 two more parked along it, 704-737 a car standing up the road
# right of straight ahead; each read as a far blocker on every far run of that hold
HELD_LEFT = [
  (0.000, 'far', [(428, 422, 513, 480), (536, 428, 575, 466), (504, 431, 546, 469), (704, 438, 737, 456)]),
  (0.200, 'full', [(428, 419, 515, 479)]),
  (0.300, 'far', [(429, 422, 511, 479), (504, 431, 546, 469), (536, 429, 576, 465)]),
  (0.400, 'full', [(426, 419, 516, 480)]),
  (0.450, 'far', [(428, 422, 512, 480), (535, 428, 575, 465), (504, 431, 545, 469)]),
  (0.650, 'full', [(426, 419, 518, 479)]),
  (0.750, 'far', [(429, 422, 511, 480), (538, 428, 576, 464), (505, 430, 546, 468), (704, 437, 736, 455)]),
  (0.850, 'full', [(426, 419, 517, 480)]),
  (0.950, 'far', [(428, 422, 510, 480), (506, 431, 547, 469), (537, 428, 576, 464), (704, 439, 737, 456)]),
  (1.100, 'full', [(427, 419, 516, 479)]),
  (1.200, 'far', [(428, 422, 513, 479), (536, 428, 575, 465), (705, 438, 735, 456), (503, 431, 545, 469)]),
  (1.300, 'full', [(426, 419, 515, 480)]),
  (1.350, 'far', [(428, 422, 513, 479), (537, 428, 576, 465), (503, 431, 546, 469)]),
  (1.550, 'full', [(426, 419, 515, 480)]),
  (1.650, 'far', [(428, 422, 512, 479), (506, 430, 547, 468), (705, 438, 736, 455), (536, 428, 575, 466)]),
  (1.750, 'full', [(427, 419, 514, 479)]),
  (1.850, 'far', [(429, 422, 511, 480), (506, 430, 547, 468), (538, 428, 576, 464), (705, 438, 735, 455)]),
  (2.050, 'full', [(427, 419, 518, 479)]),
  (2.150, 'far', [(429, 422, 511, 479), (538, 428, 576, 464), (506, 430, 547, 468), (704, 439, 736, 455)]),
  (2.250, 'full', [(427, 419, 518, 479)]),
  (2.350, 'far', [(428, 422, 513, 481), (704, 437, 736, 456), (505, 431, 545, 469), (537, 428, 575, 464)]),
  (2.500, 'full', [(426, 419, 517, 479)]),
  (2.600, 'far', [(429, 422, 512, 479), (537, 428, 576, 465), (507, 430, 546, 467), (705, 439, 736, 455)]),
  (2.700, 'full', [(426, 419, 514, 480)]),
  (2.750, 'far', [(429, 421, 512, 480), (536, 428, 575, 466), (706, 438, 735, 455), (505, 430, 546, 469)]),
  (2.950, 'full', [(427, 419, 519, 479)]),
]


def calibration():
  msg = messaging.new_message('extrinsicsCalibration')
  cal = msg.extrinsicsCalibration
  cal.rpyCalib, cal.wideFromDeviceEuler, cal.height = RPY, [0., 0., 0.], [HEIGHT]
  cal.calStatus = 'calibrated'
  return msg.as_reader().extrinsicsCalibration


def car_state(v_ego=0.):
  msg = messaging.new_message('carState')
  msg.carState.vEgo = v_ego
  return msg.as_reader().carState


def run_msg(boxes, t, view='full', frame_id=0):
  msg = messaging.new_message('turnpilotDetections')
  td = msg.turnpilotDetections
  td.frameId, td.timestampEof, td.camera, td.view = frame_id, int(t * 1e9), 'road', view
  for d, (x0, y0, x1, y1) in zip(td.init('detections', len(boxes)), boxes, strict=True):
    d.cls, d.conf, d.x0, d.y0, d.x1, d.y1 = CAR, 0.6, float(x0), float(y0), float(x1), float(y1)
  return msg.as_reader().turnpilotDetections


def far_msg(boxes, t, frame_id=0):
  msg = messaging.new_message('turnpilotFarDetections')
  fd = msg.turnpilotFarDetections
  fd.crop = [float(v) for v in FAR_CROP]
  r = fd.run
  r.frameId, r.timestampEof, r.camera, r.view = frame_id, int(t * 1e9), 'road', 'far'
  for d, (x0, y0, x1, y1) in zip(r.init('detections', len(boxes)), boxes, strict=True):
    d.cls, d.conf, d.x0, d.y0, d.x1, d.y1 = CAR, 0.6, float(x0), float(y0), float(x1), float(y1)
  return msg.as_reader().turnpilotFarDetections


def box_of(x, y_left, lens, rot):
  """Road camera pixels of a car whose rear (or front) centre is on the ground at (x forward, y left) m from the camera."""
  u, v = lens.project(rot.T @ np.array([x, -y_left, HEIGHT]))
  half = lens.f * CAR_W / 2 / x
  top = lens.project(rot.T @ np.array([x, -y_left, HEIGHT - CAR_H]))[1]
  return (u - half, top, u + half, v)


def drive(tracker, cal, path, v_ego=0., hz=8., seconds=2.5, noise_px=0., seed=0):
  """Runs of one car on path(t) -> (x, y_left), the full pass and the far crop alternating at hz. Returns [(t, RawBox)]."""
  rng = np.random.default_rng(seed)
  lens, rot = tracker.lenses['road'], calib_from_view(RPY)
  out = []
  for k in range(int(seconds * hz) + 1):
    t = 100. + k / hz
    box = np.array(box_of(*path(t - 100.), lens, rot)) + rng.normal(0., noise_px, 4)
    if k % 2:
      tracker.update_far(far_msg([box], t, k), cal)
      frame = tracker.raw_far
    else:
      tracker.update(run_msg([box], t, frame_id=k), cal, car_state(v_ego), now=t, yaw_rate=0.)
      frame = tracker.raw
    out.append((t, frame.boxes[0]))
  return out


def test_held_left_far_curb_cars_read_standing():
  tracker, cal = SceneTracker(CAMS, 'mici'), calibration()
  last = {}
  for k, (t, view, boxes) in enumerate(HELD_LEFT):
    t += 100.
    if view == 'far':
      tracker.update_far(far_msg(boxes, t, k), cal)
      frame = tracker.raw_far
    else:
      tracker.update(run_msg(boxes, t, frame_id=k), cal, car_state(0.), now=t, yaw_rate=0.)
      frame = tracker.raw
    for b in frame.boxes:
      x0 = b.box[0] * CAMS.narrow_road.width
      last[min((428, 505, 536, 705), key=lambda a: abs(a - x0))] = (t - 100., b)
  # every box seen in the last second: the parked cars read standing, the car up the road standing or receding (115 m out, its
  # fit scatters a few m/s: neither is an oncoming car)
  assert len(last) == 4
  for key, (t, b) in last.items():
    assert t > HELD_LEFT[-1][0] - 1., key
    want = ('standing', 'receding') if key == 705 else ('standing',)
    assert b.motion in want, (key, b.motion, b.radial_mps, b.lateral_mps)


def test_unknown_until_enough_sightings():
  tracker, cal = SceneTracker(CAMS, 'mici'), calibration()
  seen = drive(tracker, cal, lambda t: (60., 7.), seconds=1.5)
  for t, b in seen:
    if t - 100. < MOTION_MIN_SPAN_S - 1e-6:
      assert b.motion == 'unknown'
  assert seen[-1][1].motion == 'standing'


def test_oncoming_car_reads_closing():
  tracker, cal = SceneTracker(CAMS, 'mici'), calibration()
  seen = drive(tracker, cal, lambda t: (150. - 15. * t, 5.), noise_px=0.5)
  b = seen[-1][1]
  assert b.motion == 'closing' and b.radial_mps < -10., (b.motion, b.radial_mps)


def test_same_direction_car_reads_receding():
  tracker, cal = SceneTracker(CAMS, 'mici'), calibration()
  seen = drive(tracker, cal, lambda t: (90. + 10. * t, -1.), noise_px=0.5)
  b = seen[-1][1]
  assert b.motion == 'receding' and b.radial_mps > 6., (b.motion, b.radial_mps)


def test_crossing_car_never_reads_standing():
  # no radial speed at all: only its lateral speed tells it from a parked car
  for y0, vy in ((-12., 10.), (12., -10.), (-6., 4.)):
    tracker, cal = SceneTracker(CAMS, 'mici'), calibration()
    seen = drive(tracker, cal, lambda t, y0=y0, vy=vy: (40., y0 + vy * t), seconds=2.)
    assert all(b.motion != 'standing' for _, b in seen), (y0, vy, [(b.motion, b.lateral_mps) for _, b in seen])


def test_moving_car_clears_nothing():
  # the car itself at 10 m/s passing a parked car: its own motion is taken out, but moving nothing reads standing or receding
  tracker, cal = SceneTracker(CAMS, 'mici'), calibration()
  seen = drive(tracker, cal, lambda t: (80. - 10. * t, 6.), v_ego=10.)
  assert all(b.motion in ('unknown', 'closing') for _, b in seen)


def test_one_noisy_fit_keeps_a_parked_car_standing():
  tracker, cal = SceneTracker(CAMS, 'mici'), calibration()
  seen = drive(tracker, cal, lambda t: (110., -3.5), seconds=6., noise_px=0.7, seed=3)
  late = [b.motion for t, b in seen if t - 100. >= 2.]
  assert late.count('standing') >= 0.9 * len(late), late


def test_wide_and_side_boxes_have_no_motion():
  tracker, cal = SceneTracker(CAMS, 'mici'), calibration()
  for k in range(12):
    t = 100. + k * 0.125
    msg = messaging.new_message('turnpilotDetections')
    td = msg.turnpilotDetections
    td.frameId, td.timestampEof, td.camera, td.view = k, int(t * 1e9), 'wide', 'full'
    d = td.init('detections', 1)[0]
    d.cls, d.conf, d.x0, d.y0, d.x1, d.y1 = CAR, 0.6, 600., 400., 700., 470.
    tracker.update(msg.as_reader().turnpilotDetections, cal, car_state(0.), now=t, yaw_rate=0.)
  assert tracker.raw.boxes[0].motion == 'unknown' and tracker.raw.boxes[0].radial_mps is None
  assert math.isfinite(tracker.raw.boxes[0].bearing_deg)


def launch_after_stand(t, x0=35., stand_s=3., accel=2.):
  """A car standing x0 m out in the oncoming lanes, then launching toward the car at accel m/s^2."""
  dt = max(0., t - stand_s)
  return x0 - 0.5 * accel * dt * dt, 5.


def test_a_stopped_oncoming_car_that_launches_leaves_standing_quickly():
  # an oncoming queue launching as the light turns: the window's slope alone kept it standing 1.8-2.1 s past its launch at
  # 29-37 m on the logged cars' noise, so a left consuming 'standing' would go into it. A step in range and width ends it
  # inside 2 s here (2.75 s on the slope alone)
  for seed in range(4):
    tracker, cal = SceneTracker(CAMS, 'mici'), calibration()
    seen = drive(tracker, cal, launch_after_stand, seconds=6., noise_px=0.5, seed=seed)
    assert any(b.motion == 'standing' for t, b in seen if t - 100. < 3.), seed
    left = next(t - 100. - 3. for t, b in seen if t - 100. > 3. and b.motion != 'standing')
    assert left <= 2.0, (seed, left)
    assert all(b.motion != 'standing' for t, b in seen if t - 100. >= 3. + left), seed
