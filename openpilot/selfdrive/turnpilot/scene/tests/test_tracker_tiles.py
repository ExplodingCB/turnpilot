"""Tracker B's far crop (SceneTracker.update_tiles, PERCEPTION_PLAN item 5, peds P3 variant g:C): the far crop's people and
two-wheelers start and extend tracks, nothing of freshness is theirs, a box the crop's inner edge cuts is dropped, and a run is
taken once, in camera-time order."""
import numpy as np

from openpilot.cereal import messaging
from openpilot.selfdrive.turnpilot.detector.decoder import BICYCLE, CAR, PERSON, fill_far
from openpilot.selfdrive.turnpilot.detector.graph import Crop
from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import CAMS, box, calibration, car_state, detections, far_crop_at_horizon
from openpilot.selfdrive.turnpilot.scene.tracker import EDGE_PX, MIN_HITS, NEW_TRACK_CONF, STALE_S, SceneTracker

W, H = CAMS.narrow_road.size


def far_rows(crop, rows, t, frame_id=0):
  """A turnpilotFarDetections reader carrying rows (x0, y0, x1, y1 in the road camera's frame pixels, conf, cls) as they are."""
  msg = messaging.new_message('turnpilotFarDetections')
  fill_far(msg.turnpilotFarDetections, np.array(rows, dtype=np.float32).reshape(-1, 6), crop, 0., frame_id, int(t * 1e9), 0.006)
  return msg.as_reader().turnpilotFarDetections


def person_row(trk, x, y, conf=0.6, dx=0.):
  b = box(trk, 'road', PERSON, x, y)
  return [b[0] + dx, b[1], b[2] + dx, b[3], conf, PERSON]


def feed_far(trk, crop, rows_at, n=4, t0=1., cal=None):
  """n far runs 0.1 s apart (the far crop's 10 Hz), rows_at(k) each; returns the published tracks after the last."""
  cal = cal if cal is not None else calibration()
  for k in range(n):
    trk.update_tiles(far_rows(crop, rows_at(k), t0 + 0.1 * k, frame_id=k), cal, car_state())
  return [trk_ for trk_ in trk.tracks if trk_.hits >= MIN_HITS]


def test_far_people_start_and_extend_tracks_and_touch_nothing_of_freshness():
  trk, cal = SceneTracker(CAMS, 'mici'), calibration()
  trk.update(detections(trk, 'road', 1.0, [(CAR, 30., 3.6, 0.8)], frame_id=1), cal, car_state(), now=1.0)
  car = trk.tracks[0]
  before = (trk.last_recv, trk.last_key, trk.last_t, trk.raw, trk.raw_new, dict(trk.raw_by_view), set(trk.new_views), list(trk.new_runs),
            trk.raw_far, trk.raw_far_new, trk.far_key, trk.available, car.misses, car.hits)
  crop, _ = far_crop_at_horizon(trk)
  for k in range(3):
    # a person at 40 m the full pass never scored, and the car: the far crop's car is no tile class, so it's never a hit here
    trk.update_tiles(far_rows(crop, [person_row(trk, 40., 0., 0.55), [*box(trk, 'road', CAR, 30., 3.6), 0.8, CAR]], 1.05 + 0.1 * k, k),
                     cal, car_state())
  person = [t for t in trk.tracks if t.cls == PERSON]
  assert len(person) == 1 and person[0].hits == 3 and person[0].view == 'far' and person[0].camera == 'road'
  assert abs(person[0].x[0] - 40.) < 2. and abs(person[0].x[1]) < 0.5 and person[0].in_path
  after = (trk.last_recv, trk.last_key, trk.last_t, trk.raw, trk.raw_new, dict(trk.raw_by_view), set(trk.new_views), list(trk.new_runs),
           trk.raw_far, trk.raw_far_new, trk.far_key, trk.available, car.misses, car.hits)
  assert after == before
  # published from MIN_HITS on, the full pass's freshness deciding when: the scene goes stale on far runs alone
  assert person[0] in trk.update(None, cal, car_state(), now=1.3)
  assert trk.update(None, cal, car_state(), now=1.0 + STALE_S + 0.01) and not trk.available


def test_far_runs_alone_never_make_the_scene_available():
  trk, cal = SceneTracker(CAMS, 'mici'), calibration()
  crop, _ = far_crop_at_horizon(trk)
  for k in range(10):
    t = 2. + 0.1 * k
    trk.update_tiles(far_rows(crop, [person_row(trk, 25., 0., 0.8)], t, k), cal, car_state())
    trk.update(None, cal, car_state(), now=t)
    assert not trk.available and trk.last_recv is None and trk.raw is None and trk.raw_far is None
  assert len(trk.tracks) == 1 and trk.tracks[0].hits == 10  # tracked all along, never fresh: fill_scene publishes nothing
  msg = messaging.new_message('turnpilotScene')
  trk.fill_scene(msg.turnpilotScene)
  assert not msg.turnpilotScene.available and len(msg.turnpilotScene.objects) == 0


def test_a_box_the_crops_inner_edge_cuts_starts_no_track_and_one_at_the_frames_edge_does():
  # spec test 3: within EDGE_PX of a crop edge that isn't the frame's (left, right, bottom) the body is cut or the feet clipped,
  # so the box ranges wrong: dropped. At the frame's own edge the crop cut nothing the camera saw: taken
  trk = SceneTracker(CAMS, 'mici')
  centre, _ = far_crop_at_horizon(trk)
  assert 0 < centre.x0 and centre.x0 + centre.in_w < W
  b = person_row(trk, 30., 0.)
  at_left = person_row(trk, 30., 0., dx=centre.x0 + EDGE_PX - 1 - b[0])  # x0 3 px inside the crop's left edge
  at_right = person_row(trk, 30., 0., dx=centre.x0 + centre.in_w - EDGE_PX + 1 - b[2])
  assert feed_far(trk, centre, lambda k: [at_left]) == [] and feed_far(trk, centre, lambda k: [at_right], t0=2.) == []
  assert trk.tracks == []
  # 5 px in: taken
  inside = person_row(trk, 30., 0., dx=centre.x0 + EDGE_PX + 1 - b[0])
  assert len(feed_far(SceneTracker(CAMS, 'mici'), centre, lambda k: [inside])) == 1
  # the crop's bottom above the frame's: feet within EDGE_PX of it are clipped
  cut = Crop(W, H, centre.x0, 2 * int((b[3] + 2 - centre.in_h) / 2))
  assert cut.y0 + cut.in_h < H and cut.y0 + cut.in_h - EDGE_PX <= b[3] <= cut.y0 + cut.in_h
  assert feed_far(SceneTracker(CAMS, 'mici'), cut, lambda k: [b]) == []
  # the crop's top cuts the head, never the feet: taken (as the replays measured the rule)
  top = Crop(W, H, centre.x0, 2 * int((b[1] + 2) / 2))
  assert top.y0 >= b[1] and b[3] < top.y0 + top.in_h - EDGE_PX
  assert len(feed_far(SceneTracker(CAMS, 'mici'), top, lambda k: [[b[0], top.y0, b[2], b[3], b[4], b[5]]])) == 1
  # the crop at the frame's right edge: a person there touches the frame's edge, not a cut of the crop's
  right = Crop(W, H, W - 640, centre.y0)
  edge = person_row(trk, 30., 0., dx=W - b[2])
  assert edge[2] == W and len(feed_far(SceneTracker(CAMS, 'mici'), right, lambda k: [edge])) == 1
  # ...and its left edge, inside the frame, still cuts
  cut_left = person_row(trk, 30., 0., dx=right.x0 + 2 - b[0])
  assert feed_far(SceneTracker(CAMS, 'mici'), right, lambda k: [cut_left]) == []
  # and at the frame's left edge
  left = Crop(W, H, 0, centre.y0)
  at0 = person_row(trk, 30., 0., dx=-b[0])
  assert at0[0] == 0. and len(feed_far(SceneTracker(CAMS, 'mici'), left, lambda k: [at0])) == 1


def test_a_far_run_is_taken_once():
  # the far crop's 10 Hz through turnpilotd's 20 Hz conflated SubMaster: the same run comes on two frames, one hit
  trk, cal = SceneTracker(CAMS, 'mici'), calibration()
  crop, _ = far_crop_at_horizon(trk)
  run = far_rows(crop, [person_row(trk, 30., 0.)], 1.0, frame_id=7)
  for _ in range(3):
    trk.update_tiles(run, cal, car_state())
  assert len(trk.tracks) == 1 and trk.tracks[0].hits == 1 and trk.tile_key == (7, int(1e9))
  trk.update_tiles(None, cal, car_state())
  assert trk.tracks[0].hits == 1


def test_a_far_run_older_than_the_newest_full_run_or_uncalibrated_or_cropless_is_no_run():
  trk, cal = SceneTracker(CAMS, 'mici'), calibration()
  crop, _ = far_crop_at_horizon(trk)
  trk.update(detections(trk, 'road', 2.0, [(PERSON, 30., 0., 0.8)], frame_id=1), cal, car_state(), now=2.0)
  (person,) = trk.tracks
  state = (person.hits, person.t_seen, person.x.copy(), person.box.copy())
  trk.update_tiles(far_rows(crop, [person_row(trk, 30., 0.)], 1.95, frame_id=2), cal, car_state())  # older than the full run
  trk.update_tiles(far_rows(crop, [person_row(trk, 30., 0.)], 2.05, frame_id=3), None, car_state())  # no calibration
  uncal = calibration()
  msg = messaging.new_message('extrinsicsCalibration')
  msg.extrinsicsCalibration.rpyCalib = list(uncal.rpyCalib)
  msg.extrinsicsCalibration.calStatus = 'uncalibrated'
  trk.update_tiles(far_rows(crop, [person_row(trk, 30., 0.)], 2.10, frame_id=4), msg.as_reader().extrinsicsCalibration, car_state())
  cropless = messaging.new_message('turnpilotFarDetections')
  cropless.turnpilotFarDetections.run = far_rows(crop, [person_row(trk, 30., 0.)], 2.15, frame_id=5).run
  trk.update_tiles(cropless.as_reader().turnpilotFarDetections, cal, car_state())
  assert len(trk.tracks) == 1 and (person.hits, person.t_seen) == state[:2]
  assert np.array_equal(person.x, state[2]) and np.array_equal(person.box, state[3])
  # a newer one matches it: one more hit, view far, and its missed runs stay the full pass's
  person.misses = 1
  trk.update_tiles(far_rows(crop, [person_row(trk, 30., 0.)], 2.20, frame_id=6), cal, car_state())
  assert person.hits == state[0] + 1 and person.view == 'far' and person.misses == 1 and len(trk.tracks) == 1


def test_only_people_and_two_wheelers_at_new_track_conf_start_tracks_and_a_duplicate_is_one():
  trk = SceneTracker(CAMS, 'mici')
  crop, _ = far_crop_at_horizon(trk)
  bike = [*box(trk, 'road', CAR, 35., -3.), 0.7, BICYCLE]  # a bicycle's box (a car's size will do)
  car = [*box(trk, 'road', CAR, 45., 3.6), 0.9, CAR]
  low = person_row(trk, 30., 0., NEW_TRACK_CONF - 0.01)
  p = person_row(trk, 25., -1.5, 0.7)
  dup = [p[0] + 1., p[1], p[2] + 1., p[3], 0.5, PERSON]  # the same person twice (IoU > 0.5): one box, the more confident
  feed_far(trk, crop, lambda k: [car, bike, low, p, dup], n=1)
  assert sorted(t.cls for t in trk.tracks) == [PERSON, BICYCLE]
  (person,) = [t for t in trk.tracks if t.cls == PERSON]
  assert abs(person.conf - 0.7) < 1e-6 and abs(person.x[0] - 25.) < 1.5
