from unittest import mock

import numpy as np

from openpilot.cereal import messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.detector.decoder import CAR, PERSON, fill_detections
from openpilot.selfdrive.turnpilot.scene.ranging import calib_from_view
from openpilot.selfdrive.turnpilot.scene import tracker as tracker_module
from openpilot.selfdrive.turnpilot.scene.tracker import (CLASS_GROUP, CLOSE_HOLD_MIN_HITS, CLOSE_HOLD_S, GROUND_GATE_FRAC, GROUND_GATE_M,
                                                         HIGH_CONF, IOU_FIRST, IOU_SECOND, MAX_COAST_S, MAX_TRACKS, PATH_X, Measurement,
                                                         SceneTracker, Track, greedy_match, iou, predicted_path)

CAMS = DEVICE_CAMERAS[('mici', 'os04c10')]
RPY, WIDE_EULER, HEIGHT = [0.0, 0.03, 0.01], [0.0, 0.005, -0.003], 1.3
DT = 0.15  # the detector's period at every 3rd model frame
SIZES = {PERSON: (0.6, 1.7), CAR: (1.8, 1.5)}  # width, height in m


# the public demo route (5beb9b58bd12b691/0000010a--a51155e496, a comma four): its calibration at 860-913 s, and detections of it
# as the road camera published them (route s, conf, x0 y0 x1 y1), in the detector's 20 Hz
DEMO_CALIBRATION = ([-0.00024, 0.16399, 0.00554], WIDE_EULER, 1.348)
# the overpass pillar and bush the detector calls a person: h/w 0.46-0.50, cut off by the frame's top, 9.5 m by height, 90-180 m
# by ground contact (a 15-30 m tall person)
PILLAR = [(859.95, 0.47, (285, 0, 697, 200)), (860.10, 0.38, (283, 0, 697, 203)), (860.15, 0.47, (278, 0, 700, 201)),
          (860.20, 0.39, (275, 0, 701, 201)), (860.30, 0.45, (269, 0, 701, 203)), (860.35, 0.51, (268, 0, 699, 203)),
          (860.40, 0.39, (258, 0, 698, 206)), (860.45, 0.57, (253, 0, 699, 206)), (860.50, 0.59, (243, 0, 696, 207))]
# a cyclist, 30 m away, on a rising road (YOLO says person): h/w 1.39-1.64, the box ends 3-11 px under the horizon, so the
# ground contact reads 150-170 m and the box 7.6-9.8 m tall
CYCLIST = [(876.85, 0.50, (969, 136, 1012, 201)), (876.90, 0.43, (968, 135, 1015, 200)), (876.95, 0.40, (979, 140, 1023, 202)),
           (877.20, 0.39, (999, 136, 1037, 200))]
# a pedestrian standing at the curb, 28 m away: h/w 3.1-3.7, 2.3-2.6 m tall by ground contact
CURB_PEDESTRIAN = [(875.50, 0.58, (850, 150, 881, 246)), (875.55, 0.66, (857, 150, 884, 247)), (875.60, 0.43, (862, 153, 889, 248)),
                   (875.65, 0.66, (865, 155, 893, 248)), (875.70, 0.51, (869, 144, 897, 245))]
# a pedestrian walking across the road, 26 m away: h/w 2.0-2.3, 2.2-2.3 m tall
CROSSWALK_PEDESTRIAN = [(912.45, 0.84, (356, 154, 401, 255)), (912.50, 0.81, (323, 159, 372, 256)), (912.55, 0.84, (289, 155, 340, 256)),
                        (912.60, 0.86, (254, 154, 305, 256)), (912.65, 0.81, (216, 154, 269, 257))]


def calibration(demo=False):
  msg = messaging.new_message('extrinsicsCalibration')
  cal = msg.extrinsicsCalibration
  rpy, wide, height = DEMO_CALIBRATION if demo else (RPY, WIDE_EULER, HEIGHT)
  cal.rpyCalib, cal.wideFromDeviceEuler, cal.height = rpy, wide, [height]
  return msg.as_reader().extrinsicsCalibration


def car_state(v_ego=0.0, yaw_rate=0.0):
  msg = messaging.new_message('carState')
  msg.carState.vEgo, msg.carState.yawRate = v_ego, yaw_rate
  return msg.as_reader().carState


def box(tracker: SceneTracker, camera: str, cls: int, x: float, y: float):
  """The box an object standing at ground point (x forward, y left) makes in a camera."""
  lens = tracker.lenses[camera]
  rot = calib_from_view(RPY, WIDE_EULER if camera == 'wide' else None).T
  u, v = lens.project(rot @ np.array([x, -y, HEIGHT]))
  w, h = SIZES[cls]
  scale = lens.f / np.hypot(x, y)
  return [u - w * scale / 2, v - h * scale, u + w * scale / 2, v]


def detections(tracker, camera: str, t: float, objects, frame_id=0):
  """objects: (cls, x, y, conf). Returns a TurnpilotDetections reader at camera time t."""
  rows = np.array([[*box(tracker, camera, cls, x, y), conf, cls] for cls, x, y, conf in objects], dtype=np.float32).reshape(-1, 6)
  msg = messaging.new_message('turnpilotDetections')
  fill_detections(msg.turnpilotDetections, rows, frame_id, int(t * 1e9), camera, 0.005)
  return msg.as_reader().turnpilotDetections


def feed_demo(tracker, rows):
  """Recorded person detections of the demo route, one per road camera frame; returns the published tracks after the last."""
  cal, tracks = calibration(demo=True), []
  for i, (t, conf, bx) in enumerate(rows):
    msg = messaging.new_message('turnpilotDetections')
    fill_detections(msg.turnpilotDetections, np.array([[*bx, conf, PERSON]], dtype=np.float32), i, int(t * 1e9), 'road', 0.005)
    tracks = tracker.update(msg.as_reader().turnpilotDetections, cal, car_state(10.0), now=t)
  return tracks


def nearest_track(tracks, x, y):
  """The track id whose position (in a frame's snapshot) is nearest to (x, y)."""
  return min(tracks, key=lambda tid: np.hypot(tracks[tid]['x'][0] - x, tracks[tid]['x'][1] - y))


class TestSceneTracker(OpenpilotTestCase):
  def setUp(self):
    self.tracker = SceneTracker(CAMS, 'mici')
    self.cal, self.cs = calibration(), car_state()

  def run_frames(self, n, objects_at, camera='road', v_ego=0.0, t0=10.0):
    """objects_at(t) -> [(cls, x, y, conf)]. Returns a snapshot of the published tracks after each frame: {id: state}."""
    out = []
    for i in range(n):
      t = t0 + i * DT
      det = detections(self.tracker, camera(i) if callable(camera) else camera, t, objects_at(t - t0), i)
      tracks = self.tracker.update(det, self.cal, car_state(v_ego), now=t)
      out.append({trk.track_id: {'x': trk.x.copy(), 'camera': trk.camera, 'in_path': trk.in_path} for trk in tracks})
    return out

  def test_crossing_pedestrians_keep_ids(self):
    # A walks right to left at 12 m, B left to right at 16 m; their boxes cross in the image mid-way
    def objects(t):
      return [(PERSON, 12.0, -4.0 + 1.5 * t, 0.8), (PERSON, 16.0, 4.0 - 1.5 * t, 0.7)]
    history = self.run_frames(40, objects)
    ids = set()
    for i, tracks in enumerate(history[1:], start=1):
      assert len(tracks) == 2
      (_, xa, ya, _), (_, xb, yb, _) = objects(i * DT)
      ids.add((nearest_track(tracks, xa, ya), nearest_track(tracks, xb, yb)))
    assert len(ids) == 1 and len(next(iter(ids))) == len(set(next(iter(ids))))
    a, b = next(iter(ids))
    # walking speed over the ground, sideways
    np.testing.assert_allclose(history[-1][a]['x'][3], 1.5, atol=0.3)
    np.testing.assert_allclose(history[-1][b]['x'][3], -1.5, atol=0.3)

  def test_ego_motion_is_removed(self):
    # a parked car ahead while we drive at 10 m/s: it closes in, but over the ground it stands still
    def objects(t):
      return [(CAR, 50.0 - 10.0 * t, 0.0, 0.9)]
    history = self.run_frames(20, objects, v_ego=10.0)
    assert len(history[-1]) == 1
    trk = next(iter(history[-1].values()))
    np.testing.assert_allclose(trk['x'][0], 50.0 - 10.0 * 19 * DT, rtol=0.05)
    assert abs(trk['x'][2]) < 1.0 and abs(trk['x'][3]) < 0.5
    assert trk['in_path']

  def test_camera_switch_keeps_id(self):
    def objects(t):
      return [(PERSON, 9.0, 3.0 - 0.5 * t, 0.8)]
    history = self.run_frames(12, objects, camera=lambda i: 'road' if i < 6 else 'wide')
    ids = {tid for tracks in history[1:] for tid in tracks}
    assert len(ids) == 1 and all(len(tracks) == 1 for tracks in history[1:])
    assert history[-1][ids.pop()]['camera'] == 'wide'

  def test_staleness(self):
    det = detections(self.tracker, 'road', 20.0, [(CAR, 20.0, 0.0, 0.9)])
    self.tracker.update(det, self.cal, self.cs, now=20.0)
    assert self.tracker.available
    self.tracker.update(det, self.cal, self.cs, now=20.4)  # same message again: no new detections
    assert self.tracker.available
    self.tracker.update(det, self.cal, self.cs, now=20.6)
    assert not self.tracker.available
    msg = messaging.new_message('turnpilotScene')
    self.tracker.fill_scene(msg.turnpilotScene)
    assert not msg.turnpilotScene.available and len(msg.turnpilotScene.objects) == 0

  def test_fill_scene(self):
    def objects(t):
      return [(PERSON, 15.0, 0.5, 0.8), (CAR, 25.0, 6.0, 0.9)]
    self.run_frames(3, objects)
    msg = messaging.new_message('turnpilotScene')
    self.tracker.fill_scene(msg.turnpilotScene)
    scene = msg.as_reader().turnpilotScene
    assert scene.available and len(scene.objects) == 2
    by_cls = {str(o.cls): o for o in scene.objects}
    ped, car = by_cls['pedestrian'], by_cls['car']
    np.testing.assert_allclose((ped.x, ped.y), (15.0, 0.5), atol=0.3)
    np.testing.assert_allclose((car.x, car.y), (25.0, 6.0), atol=0.5)
    assert ped.inPath and not car.inPath
    assert all(0 <= v <= 1 for v in ped.box) and ped.box[2] > ped.box[0] and str(ped.camera) == 'road'

  def test_close_pedestrian_out_of_view_is_kept(self):
    # stopping for a pedestrian standing in the path: it drops out of view at 6 m while the car still rolls at 1 m/s
    def objects(t):
      return [(PERSON, 9.0 - 1.0 * t, 0.2, 0.8)] if t < 3.0 else []
    history = self.run_frames(int((3.0 + CLOSE_HOLD_S - 0.2) / DT), objects, v_ego=1.0)
    ids = {tid for tracks in history[2:] for tid in tracks}
    assert len(ids) == 1 and all(len(tracks) == 1 for tracks in history[2:])
    trk = history[-1][ids.pop()]
    assert trk['in_path'] and trk['x'][0] < 6.0 - 0.5 * (CLOSE_HOLD_S - 0.5)  # carried along with the car's motion
    later = self.run_frames(3, lambda t: [], v_ego=1.0, t0=10.0 + len(history) * DT)
    assert later[-1] == {}

  def test_close_hold_needs_matched_detections(self):
    # a pedestrian in the path at 6.5 m that stops being detected: the long hold is for one matched at least CLOSE_HOLD_MIN_HITS
    # times, so a 2-3 hit blip is released after MAX_COAST_S like anything else
    for hits in (CLOSE_HOLD_MIN_HITS - 1, CLOSE_HOLD_MIN_HITS):
      self.tracker = SceneTracker(CAMS, 'mici')

      def objects(t, hits=hits):
        return [(PERSON, 6.5, 0.2, 0.8)] if t < hits * DT - 1e-6 else []
      history = self.run_frames(hits + int((MAX_COAST_S + 0.4) / DT), objects)
      assert len(history[hits - 1]) == (hits >= 2)  # published from its 2nd detection on
      assert len(history[-1]) == (hits >= CLOSE_HOLD_MIN_HITS)

  def test_pillar_and_bush_are_not_a_pedestrian(self):
    # both checks drop it, and each alone does: the box is wider than tall, and 90-180 m of ground contact under a 9.5 m person's height
    assert feed_demo(SceneTracker(CAMS, 'mici'), PILLAR) == []
    for off in ({'MAX_PERSON_GROUND_HEIGHT': np.inf}, {'MIN_PERSON_ASPECT': 0.0}):
      tracker = SceneTracker(CAMS, 'mici')
      with mock.patch.multiple(tracker_module, **off):
        assert feed_demo(tracker, PILLAR) == [] and tracker.tracks == []
    # with both off it was a pedestrian track less than 10 m ahead (in the path, with the driving model's path)
    with mock.patch.multiple(tracker_module, MAX_PERSON_GROUND_HEIGHT=np.inf, MIN_PERSON_ASPECT=0.0):
      tracks = feed_demo(SceneTracker(CAMS, 'mici'), PILLAR)
    assert len(tracks) == 1 and tracks[0].x[0] < 10.0

  def test_real_pedestrians_and_cyclist_stay(self):
    for rows in (CURB_PEDESTRIAN, CYCLIST, CROSSWALK_PEDESTRIAN):
      tracks = feed_demo(SceneTracker(CAMS, 'mici'), rows)
      assert len(tracks) == 1 and tracks[0].cls == PERSON and tracks[0].hits == len(rows)

  def test_far_pedestrian_out_of_view_is_dropped(self):
    def objects(t):
      return [(PERSON, 20.0, 0.2, 0.8)] if t < 1.0 else []
    history = self.run_frames(int((1.0 + MAX_COAST_S + 0.4) / DT), objects)
    assert len(history[3]) == 1 and history[-1] == {}

  def test_no_tracks_above_horizon(self):
    # a box whose bottom is in the sky can't be ranged, so it never starts a track
    for i in range(3):
      msg = messaging.new_message('turnpilotDetections')
      fill_detections(msg.turnpilotDetections, np.array([[600, 10, 700, 80, 0.9, CAR]], dtype=np.float32), i, int((1 + i * DT) * 1e9), 'road', 0)
      tracks = self.tracker.update(msg.as_reader().turnpilotDetections, self.cal, self.cs, now=1 + i * DT)
    assert tracks == [] and self.tracker.tracks == []

  def test_predicted_path(self):
    def model(xs, ys_right):
      msg = messaging.new_message('modelV2')
      msg.modelV2.position.x, msg.modelV2.position.y = xs, ys_right
      return msg.as_reader().modelV2
    x, y = predicted_path(None)
    assert x[-1] == PATH_X and not y.any()
    # stopped: the model path is a few cm long, so straight ahead
    x, y = predicted_path(model([0.0, 0.01, 0.02, 0.02], [0.0, 0.0, 0.0, 0.0]))
    assert x[-1] == PATH_X and not y.any()
    # a right curve that ends at 30 m carries on along its last heading; y is to the left
    xs = [float(v) for v in np.linspace(0, 30, 16)]
    x, y = predicted_path(model(xs, [0.002 * v ** 2 for v in xs]))
    assert x[-1] == PATH_X
    np.testing.assert_allclose(np.interp(20.0, x, y), -0.8, atol=0.05)
    assert y[-1] < np.interp(30.0, x, y) < 0

  def test_turning_ego_stationary_pedestrian(self):
    # we turn left at 0.3 rad/s and 5 m/s past a pedestrian standing still; with the model's yaw rate removed it stays
    # still over the ground, where ignoring the turn would show it crossing at about 3 m/s
    omega, v, world = 0.3, 5.0, np.array([12.0, 4.0])
    msg = messaging.new_message('modelV2')
    msg.modelV2.orientationRate.z = [-omega]  # the model's z is down
    model = msg.as_reader().modelV2
    for i in range(12):
      t = i * DT
      psi = omega * t
      pos = v / omega * np.array([np.sin(psi), 1 - np.cos(psi)])
      c, s = np.cos(psi), np.sin(psi)
      x, y = np.array([[c, s], [-s, c]]) @ (world - pos)
      det = detections(self.tracker, 'road', 10 + t, [(PERSON, float(x), float(y), 0.8)], i)
      tracks = self.tracker.update(det, self.cal, car_state(v), model, now=10 + t)
    assert len(tracks) == 1 and tracks[0].x is not None
    assert np.hypot(*tracks[0].x[2:]) < 0.7

  def test_association_scores_all_pairs_at_once(self):
    # 64 detections against 64 tracks: one IoU call for the lot (a call per pair took 18.7 ms a frame), the same pairs as
    # scoring each pair on its own
    rng = np.random.default_rng(3)
    tracker = SceneTracker(CAMS, 'mici')
    for i in range(64):
      b = np.r_[rng.uniform(0, 1800, 2), 0, 0]
      b[2:] = b[:2] + rng.uniform(20, 200, 2)
      trk = Track(i, 'road' if i % 5 else 'wide', b, b, 0.0, 0.8, {[PERSON, CAR, 1][i % 3]: 1.0})
      if i % 7:
        trk.x, trk.P = np.r_[rng.uniform(5, 60), rng.uniform(-10, 10), 0, 0], np.eye(4)
      tracker.tracks.append(trk)
    meas = []
    for j in range(64):
      trk = tracker.tracks[rng.integers(64)]
      xy = None if j % 6 == 0 or trk.x is None else trk.x[:2] + rng.normal(0, 1.5, 2)
      meas.append(Measurement(trk.box + rng.normal(0, 15, 4), float(rng.uniform(0.35, 0.95)), [PERSON, CAR, 1, 3][j % 4], xy))
    with mock.patch.object(tracker_module, 'iou', wraps=iou) as calls:
      pairs, unmatched = tracker.associate(meas, 'road')
    assert calls.call_count == 1
    assert (pairs, unmatched) == pairwise_associate(tracker.tracks, meas, 'road') and len(pairs) > 20

  def test_track_count_is_capped(self):
    # a burst of false positives, new ones every frame: the count stays capped, and a close-held pedestrian stays
    def objects(t):
      k = int(round(t / DT))
      rng = np.random.default_rng(k)
      noise = [(CAR, float(rng.uniform(15, 120)), float(rng.uniform(-40, 40)), 0.6) for _ in range(60)] if k >= 6 else []
      return ([(PERSON, 6.5, 0.2, 0.8)] if k < 6 else []) + noise
    with mock.patch.object(tracker_module, 'MAX_TRACKS', 10 ** 6):
      self.run_frames(16, objects)
    assert len(self.tracker.tracks) > MAX_TRACKS  # uncapped
    self.tracker = SceneTracker(CAMS, 'mici')
    self.run_frames(16, objects)
    assert len(self.tracker.tracks) <= MAX_TRACKS
    assert any(trk.cls == PERSON and trk.in_path for trk in self.tracker.published())


def pairwise_associate(tracks, meas, camera):
  """The association scored one pair at a time, as before it was vectorized."""
  pairs, free_t, free_m = [], set(range(len(tracks))), set(range(len(meas)))

  def run(ms, score_fn, min_score):
    ts = sorted(free_t)
    if not ts or not ms:
      return
    for a, b in greedy_match(np.array([[score_fn(tracks[i], meas[j]) for j in ms] for i in ts]), min_score):
      pairs.append((ts[a], ms[b]))
      free_t.discard(ts[a])
      free_m.discard(ms[b])

  def iou_score(trk, m):
    if trk.camera != camera or CLASS_GROUP[trk.cls] != CLASS_GROUP[m.cls]:
      return -1.0
    return float(iou(trk.box[None], m.box[None])[0, 0])

  def ground_score(trk, m):
    if trk.x is None or m.xy is None or CLASS_GROUP[trk.cls] != CLASS_GROUP[m.cls]:
      return -np.inf
    d = float(np.hypot(*(trk.x[:2] - m.xy)))
    return -d if d <= GROUND_GATE_M + GROUND_GATE_FRAC * float(np.hypot(*m.xy)) else -np.inf

  run(sorted(j for j in free_m if meas[j].conf >= HIGH_CONF), iou_score, IOU_FIRST)
  run(sorted(j for j in free_m if meas[j].conf < HIGH_CONF), iou_score, IOU_SECOND)
  run(sorted(free_m), ground_score, -1e9)
  return pairs, sorted(free_m)


def random_association(rng):
  """Tracks and measurements built to hit the edge cases: both cameras and class groups, missing ground positions,
  duplicate boxes and positions (exact ties), zero-area boxes, and measurements just inside or outside the ground gate."""
  tracks = []
  for i in range(int(rng.integers(0, 25))):
    b = np.r_[rng.uniform(0, 1800, 2), 0, 0]
    b[2:] = b[:2] + rng.uniform(0, 200, 2) * (rng.random() > 0.05)
    if tracks and rng.random() < 0.15:
      b = tracks[int(rng.integers(len(tracks)))].box.copy()
    trk = Track(i, 'road' if rng.random() < 0.6 else 'wide', b, b, 0.0, 0.8, {int(rng.choice([PERSON, CAR, 1, 3, 5, 7])): 1.0})
    if rng.random() < 0.85:
      placed = [t for t in tracks if t.x is not None]
      xy = placed[int(rng.integers(len(placed)))].x[:2] if placed and rng.random() < 0.1 else np.r_[rng.uniform(3, 80), rng.uniform(-15, 15)]
      trk.x, trk.P = np.r_[xy, 0, 0], np.eye(4)
    tracks.append(trk)
  meas = []
  for _ in range(int(rng.integers(0, 25))):
    src = tracks[int(rng.integers(len(tracks)))] if tracks and rng.random() < 0.7 else None
    bx = (src.box + rng.normal(0, rng.choice([0., 5., 40.]), 4)) if src is not None else np.r_[rng.uniform(0, 1800, 2), 0, 0]
    if src is None:
      bx[2:] = bx[:2] + rng.uniform(1, 200, 2)
    xy = None
    if rng.random() < 0.85:
      if src is not None and src.x is not None:
        d = GROUND_GATE_M + GROUND_GATE_FRAC * float(np.hypot(*src.x[:2]))
        xy = src.x[:2] + rng.choice([0., 0.5, 0.999, 1.001]) * d * np.array([1., 0.])  # exact, inside, at, past the gate
      else:
        xy = np.r_[rng.uniform(3, 80), rng.uniform(-15, 15)]
    meas.append(Measurement(bx, float(rng.choice([0.35, 0.49, 0.5, 0.51, 0.9])), int(rng.choice([PERSON, CAR, 1, 3, 5, 7])), xy))
  return tracks, meas


def test_vectorized_association_matches_the_old_loop():
  # the pre-vectorization associate() (5f9d8c0), verbatim as pairwise_associate, on 3000 random frames of edge cases
  tracker, matched = SceneTracker(CAMS, 'mici'), 0
  for seed in range(3000):
    rng = np.random.default_rng(seed)
    tracker.tracks, meas = random_association(rng)
    camera = 'road' if seed % 2 else 'wide'
    got = tracker.associate(meas, camera)
    assert got == pairwise_associate(tracker.tracks, meas, camera), f"seed {seed}"
    matched += len(got[0])
  assert matched > 10000


def test_vectorized_tracker_keeps_the_old_ids():
  # whole drives, both cameras alternating, crossing and bunched objects: the same track ids frame for frame
  def drive(associate):
    rng = np.random.default_rng(11)
    tracker, cal, ids = SceneTracker(CAMS, 'mici'), calibration(), []
    objs = [(int(rng.choice([PERSON, CAR])), rng.uniform(8, 60), rng.uniform(-12, 12), rng.uniform(-2, 2), rng.uniform(-2, 2))
            for _ in range(30)]
    with mock.patch.object(SceneTracker, 'associate', associate):
      for k in range(200):
        t, camera = 10. + k * DT, 'wide' if (k // 7) % 2 else 'road'
        frame = [(c, x + vx * k * DT, y + vy * k * DT, float(rng.uniform(0.36, 0.95))) for c, x, y, vx, vy in objs
                 if rng.random() < 0.8 and x + vx * k * DT > 4.]
        tracks = tracker.update(detections(tracker, camera, t, frame, k), cal, car_state(), now=t)
        ids.append(sorted((trk.track_id, round(float(trk.x[0]), 9), round(float(trk.x[1]), 9)) for trk in tracks))
    return ids

  old = drive(lambda self, meas, camera: pairwise_associate(self.tracks, meas, camera))
  assert drive(SceneTracker.associate) == old and sum(map(len, old)) > 2000


def test_raw_boxes_of_every_run_with_bearing_and_range():
  # the unprotected left's gap rule reads every run's boxes, tracked or not (CRITIQUE REQUIRED 2): a car 120 m up the
  # oncoming lane at 0.36 (under NEW_TRACK_CONF: never a track) is there with its bearing, its ground point and its nearest
  # plausible range (the box bottom NEAR_PX lower)
  trk = SceneTracker(CAMS, 'mici')
  cal = calibration()
  d = detections(trk, 'road', 1.0, [(CAR, 120., 3.6, 0.36)])
  assert trk.update(d, cal, car_state(), now=1.0) == []
  raw = trk.raw
  assert raw is not None and trk.raw_new and raw.camera == 'road' and raw.t == 1.0 and len(raw.boxes) == 1
  b = raw.boxes[0]
  assert b.camera == 'road' and b.cls == 'car' and abs(b.conf - 0.36) < 1e-6
  assert abs(b.bearing_deg - np.degrees(np.arctan2(3.6, 120.))) < 0.1
  assert b.span_deg[0] < b.bearing_deg < b.span_deg[1]
  assert b.xy is not None and abs(b.xy[0] - 120.) < 2. and abs(b.xy[1] - 3.6) < 0.3
  assert b.near_m is not None and 60. < b.near_m < np.hypot(*b.xy)
  assert all(0. <= v <= 1. for v in b.box) and len(b.box) == 4
  # no new run: the same frame, not new
  trk.update(d, cal, car_state(), now=1.05)
  assert not trk.raw_new and trk.raw is raw
  # far past ranging.MAX_RANGE, still ranged here (the band reads its nearest plausible range, never the cap)
  d = detections(trk, 'road', 1.1, [(CAR, 260., 3.6, 0.5)], frame_id=1)
  trk.update(d, cal, car_state(), now=1.1)
  b = trk.raw.boxes[0]
  assert trk.raw_new and b.xy is not None and b.xy[0] > 200. and b.near_m is not None and b.near_m < b.xy[0]
  # the box bottom above the horizon: unranged, with its bearing
  rows = np.array([[600., 100., 640., 130., 0.6, CAR]], dtype=np.float32)
  msg = messaging.new_message('turnpilotDetections')
  fill_detections(msg.turnpilotDetections, rows, 2, int(1.2e9), 'road', 0.005)
  trk.update(msg.as_reader().turnpilotDetections, cal, car_state(), now=1.2)
  b = trk.raw.boxes[0]
  assert b.xy is None and b.near_m is None and np.isfinite(b.bearing_deg)
  # the wide camera's run is the wide camera's
  d = detections(trk, 'wide', 1.3, [(CAR, 20., 3.6, 0.5)], frame_id=3)
  trk.update(d, cal, car_state(), now=1.3)
  assert trk.raw.camera == 'wide' and trk.raw.boxes[0].camera == 'wide' and abs(trk.raw.boxes[0].xy[0] - 20.) < 1.5


def far_run(trk, crop, t, objects, frame_id=0, bearing=0.):
  """A turnpilotFarDetections reader as modeld publishes it: objects (cls, x, y, conf) standing at ground points, their
  boxes in the road camera's frame taken into the crop's input pixels and back out through decode_far (graph.Crop)."""
  from openpilot.selfdrive.turnpilot.detector.decoder import decode_far, fill_far
  raw = np.zeros((1, 300, 6), np.float32)
  for i, (cls, x, y, conf) in enumerate(objects):
    x0, y0, x1, y1 = box(trk, 'road', cls, x, y)
    raw[0, i] = [*crop.to_input(x0, y0), *crop.to_input(x1, y1), conf, cls]
  msg = messaging.new_message('turnpilotFarDetections')
  fill_far(msg.turnpilotFarDetections, decode_far(raw, crop), crop, bearing, frame_id, int(t * 1e9), 0.006)
  return msg.as_reader().turnpilotFarDetections


def far_crop_at_horizon(trk, x0=None):
  """The far crop with the calibrated horizon straight ahead at its FAR_HORIZON_ROW (as modeld aims it), or at column x0."""
  from openpilot.selfdrive.turnpilot.detector.graph import FAR_HORIZON_ROW, Crop, crop_at
  u, v = trk.lenses['road'].project(calib_from_view(RPY).T @ np.array([1., 0., 0.]))
  crop = crop_at(1344, 760, u, v)
  return crop if x0 is None else Crop(1344, 760, x0, crop.y0), v - FAR_HORIZON_ROW


def test_far_boxes_at_120_to_220_m_range_and_bear_as_the_road_cameras():
  # DESIGN phase 4 G5: the far crop's boxes, in the road camera's full-frame pixels by the crop's offset, range and bear with
  # the road lens exactly as a full pass box at the same pixels would. At 220 m one pixel of box bottom is about 30 m, so
  # this is to the centimetre, never a tolerance a pixel's error could pass
  trk = SceneTracker(CAMS, 'mici')
  cal = calibration()
  for i, x0 in enumerate((288, 352, 416)):
    crop, _ = far_crop_at_horizon(trk, x0)
    assert crop.x0 > 0 and crop.y0 > 0
    for k, (x, y) in enumerate([(120., 3.6), (150., 7.2), (180., 3.6), (200., -3.6), (220., 10.8)]):
      t = 1. + i + 0.1 * k
      fd = far_run(trk, crop, t, [(CAR, x, y, 0.4)], frame_id=10 * i + k)
      trk.update_far(fd, cal, now=t)
      assert trk.raw_far_new and trk.raw_far.view == 'far' and trk.raw_far.camera == 'road' and abs(trk.raw_far.t - t) < 1e-6
      (b,) = trk.raw_far.boxes
      assert b.view == 'far' and b.camera == 'road' and b.cls == 'car' and abs(b.conf - 0.4) < 1e-6
      assert b.xy is not None and abs(np.hypot(*b.xy) - np.hypot(x, y)) < 0.05, (x0, x, y, b.xy)
      assert abs(b.xy[1] - y) < 0.01 and abs(b.bearing_deg - np.degrees(np.arctan2(y, x))) < 0.002
      assert b.near_m is not None and b.near_m < np.hypot(x, y)  # its nearest plausible range: never further
  # the crop's horizon bearings: what the far run covers, straight ahead between them
  lo, hi = trk.raw_far.cover_deg
  assert lo < 0. < hi and abs((hi - lo) - 2 * np.degrees(np.arctan2(320, CAMS.narrow_road.focal_length))) < 0.5


def test_far_run_cover_needs_the_horizon_inside_the_crop():
  from openpilot.selfdrive.turnpilot.detector.graph import Crop
  trk = SceneTracker(CAMS, 'mici')
  cal = calibration()
  crop, _ = far_crop_at_horizon(trk)
  trk.update_far(far_run(trk, crop, 1., []), cal, now=1.)
  assert trk.raw_far.cover_deg is not None and trk.raw_far.boxes == []
  low = Crop(1344, 760, crop.x0, 760 - 384)  # the horizon above the crop's top rows: it sees the road near the car only
  trk.update_far(far_run(trk, low, 1.1, [], frame_id=1), cal, now=1.1)
  assert trk.raw_far_new and trk.raw_far.cover_deg is None
  # no calibration: nothing ranged, no cover
  trk.update_far(far_run(trk, crop, 1.2, [(CAR, 150., 3.6, 0.5)], frame_id=2), None, now=1.2)
  assert trk.raw_far.cover_deg is None and trk.raw_far.boxes == []


def test_far_matches_tag_tracks_and_never_feed_confirmation():
  # a far match tags the track (view far, farHits) and nothing else: no new track, no hit (TrackedObject.age is a hard
  # stop's confirmation: scene_policy), no Kalman update, no time seen, no freshness. A far run alone never makes the
  # scene available, never starts a track
  trk = SceneTracker(CAMS, 'mici')
  cal = calibration()
  for k in range(3):
    trk.update(detections(trk, 'road', 1. + 0.1 * k, [(CAR, 60., 3.6, 0.8)], frame_id=k), cal, car_state(), now=1. + 0.1 * k)
  (car,) = trk.published()
  hits, x, t_seen, key, recv = car.hits, car.x.copy(), car.t_seen, trk.last_key, trk.last_recv
  crop, _ = far_crop_at_horizon(trk)
  trk.update_far(far_run(trk, crop, 1.25, [(CAR, 60., 3.6, 0.8), (PERSON, 70., -3., 0.9)], frame_id=10), cal, now=1.25)
  assert car.far_hits == 1 and car.view == 'far' and len(trk.tracks) == 1  # the person: no track from a far run
  assert car.hits == hits and np.array_equal(car.x, x) and car.t_seen == t_seen and trk.last_key == key and trk.last_recv == recv
  trk.update_far(far_run(trk, crop, 1.25, [(CAR, 60., 3.6, 0.8)], frame_id=10), cal, now=1.3)
  assert car.far_hits == 1 and not trk.raw_far_new  # the same run again: nothing new
  msg = messaging.new_message('turnpilotScene')
  trk.update(None, cal, car_state(), now=1.3)
  trk.fill_scene(msg.turnpilotScene)
  (o,) = msg.as_reader().turnpilotScene.objects
  assert str(o.view) == 'far' and o.farHits == 1 and o.age == hits and str(o.camera) == 'road'
  # the full pass matching it again: view full
  trk.update(detections(trk, 'road', 1.35, [(CAR, 60., 3.6, 0.8)], frame_id=4), cal, car_state(), now=1.35)
  assert car.view == 'full' and car.hits == hits + 1 and car.far_hits == 1
  # far runs alone: nothing tracked, never available
  fresh = SceneTracker(CAMS, 'mici')
  for k in range(10):
    fresh.update_far(far_run(fresh, crop, 2. + 0.1 * k, [(PERSON, 10., 0., 0.9)], frame_id=k), cal, now=2. + 0.1 * k)
    assert fresh.update(None, cal, car_state(), now=2. + 0.1 * k) == [] and not fresh.available
  assert fresh.tracks == [] and fresh.raw is None and fresh.raw_far is not None


def test_stop_signs_never_become_tracks_or_raw_boxes():
  # a stop sign (COCO 11) rides in the message's signs, in shadow: the tracker reads detections only, so no track, raw box,
  # and so no stop, cap or release comes of one
  tracker = SceneTracker(CAMS, 'mici')
  sign = detections(tracker, 'road', 1.0, [(CAR, 12.0, 0.0, 0.8)])
  msg = messaging.new_message('turnpilotDetections')
  td = msg.turnpilotDetections
  fill_detections(td, np.zeros((0, 6), np.float32), 0, int(1e9), 'road', 0.005)
  s = td.init('signs', 1)[0]
  d = sign.detections[0]
  s.box.cls, s.box.conf, s.box.x0, s.box.y0, s.box.x1, s.box.y1 = 11, 0.9, d.x0, d.y0, d.x1, d.y1
  for k in range(5):
    tracks = tracker.update(msg.as_reader().turnpilotDetections, calibration(), car_state(0.), now=1.0 + 0.1 * k)
  assert tracks == [] and tracker.raw is not None and tracker.raw.boxes == []


def test_the_tracker_says_which_lens_it_uses():
  from openpilot.selfdrive.turnpilot.scene.ranging import wide_lens
  tracker = SceneTracker(CAMS, 'mici')
  assert tracker.lens_spec == wide_lens(CAMS.wide_road, 'mici').spec == tracker.lenses['wide'].spec
