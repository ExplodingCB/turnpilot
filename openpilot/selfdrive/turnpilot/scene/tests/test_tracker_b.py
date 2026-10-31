"""Tracker B's merge with tracker A and its frame (scene/tracker_b.py): which of B's people reach the scene policy, as what, in
which order a frame's runs go in, and what it logs."""
from types import SimpleNamespace
from unittest import mock

import numpy as np

from openpilot.cereal import messaging
from openpilot.selfdrive.turnpilot.detector.decoder import BICYCLE, CAR, PERSON, fill_detections
from openpilot.selfdrive.turnpilot.scene import tracker_b as TB
from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import CAMS, box, calibration, car_state, far_crop_at_horizon
from openpilot.selfdrive.turnpilot.scene.tests.test_tracker_tiles import far_rows
from openpilot.selfdrive.turnpilot.scene.tracker import SceneTracker
from openpilot.selfdrive.turnpilot.scene_policy import B_ID, CROSSWALK_CONF, Stop

W, H = CAMS.narrow_road.size


def obj(x, y, camera='road', cls='pedestrian', box=(0.4, 0.4, 0.42, 0.48), track_id=1, conf=0.8):
  return SimpleNamespace(trackId=track_id, cls=cls, conf=conf, x=x, y=y, vx=0., vy=0., age=5, inPath=False, camera=camera, view='far',
                         farHits=0, tSeen=1., missedRuns=0, box=list(box))


def test_a_pedestrian_held_by_tracker_a_never_reaches_the_policy_twice():
  b = obj(21.5, 1.5, 'road', track_id=4)
  # spec test 5: within 2 m of an A pedestrian the wide camera sees (a turn: the full pass on the wide camera, the far crop on the road)
  assert TB.b_only_objects([obj(20., 1., 'wide', box=(0.1, 0.1, 0.12, 0.2))], [b], []) == []
  # 2.5 m off on the other camera, boxes not comparable: B's own
  assert len(TB.b_only_objects([obj(24., 1.5, 'wide', box=b.box)], [b], [])) == 1
  # the same camera, boxes overlapping by IoU 0.3 or more: held, however far apart their ranges read
  assert TB.b_only_objects([obj(30., 1.5, 'road', box=(0.401, 0.4, 0.421, 0.48))], [b], []) == []
  assert len(TB.b_only_objects([obj(30., 1.5, 'road', box=(0.418, 0.4, 0.438, 0.48))], [b], [])) == 1
  # A's cyclist or car holds no pedestrian; B's cyclists never reach the policy
  assert len(TB.b_only_objects([obj(21.5, 1.5, cls='cyclist', box=b.box)], [b], [])) == 1
  assert TB.b_only_objects([], [obj(21.5, 1.5, cls='cyclist')], []) == []


def test_a_b_only_object_is_tracker_bs_pedestrian_offset_with_age_0_and_capped_conf():
  (o,) = TB.b_only_objects([], [obj(21.5, 1.5, track_id=4, conf=0.9)], [])
  assert (o.trackId, o.cls, o.age, o.b_only, o.camera, o.view) == (B_ID + 4, 'pedestrian', 0, True, 'road', 'far')
  assert abs(o.conf - (CROSSWALK_CONF - 0.01)) < 1e-9 and (o.x, o.y) == (21.5, 1.5) and o.box == [0.4, 0.4, 0.42, 0.48]
  (o,) = TB.b_only_objects([], [obj(21.5, 1.5, conf=0.46)], [])
  assert abs(o.conf - 0.46) < 1e-9


def test_a_b_person_in_a_two_wheelers_box_is_its_rider():
  person = obj(21.5, 1.5, box=(0.40, 0.40, 0.42, 0.48))
  bike = (0.395, 0.42, 0.43, 0.49)
  assert TB.b_only_objects([], [person], [('road', list(bike))]) == []
  assert len(TB.b_only_objects([], [person], [('wide', list(bike))])) == 1  # another camera's pixels: no overlap to judge
  assert len(TB.b_only_objects([], [person], [('road', [0.6, 0.4, 0.65, 0.5])])) == 1


def pixels(trk, cls, x, y):
  """A box at ground point (x, y) in the road camera: a person's, or a car's size for anything else (a bicycle)."""
  return box(trk, 'road', PERSON if cls == PERSON else CAR, x, y)


def full(trk, t, objects, frame_id):
  rows = np.array([[*pixels(trk, cls, x, y), conf, cls] for cls, x, y, conf in objects], dtype=np.float32).reshape(-1, 6)
  msg = messaging.new_message('turnpilotDetections')
  fill_detections(msg.turnpilotDetections, rows, frame_id, int(t * 1e9), 'road', 0.005)
  return msg.as_reader().turnpilotDetections


def scene_of(trk):
  msg = messaging.new_message('turnpilotScene')
  trk.fill_scene(msg.turnpilotScene)
  return msg.as_reader().turnpilotScene


def run_pair(full_at, far_at, frames=24):
  """Trackers A and B as turnpilotd runs them, over frames 0.05 s apart (the full pass on even ones, the far crop on odd ones);
  returns B's objects on each frame."""
  a, b = SceneTracker(CAMS, 'mici'), TB.TrackerB(SceneTracker(CAMS, 'mici'))
  cal, cs = calibration(), car_state(5.)
  crop, _ = far_crop_at_horizon(a)
  out = []
  for k in range(frames):
    t = 1. + 0.05 * k
    runs = [full(a, t, full_at(k), k)] if k % 2 == 0 else []
    far = far_rows(crop, [[*pixels(a, cls, x, y), conf, cls] for cls, x, y, conf in far_at(k)], t, k) if k % 2 else None
    a.update_runs(runs, cal, cs, None, now=t)
    a.update_far(far, cal, now=t)
    out.append(b.step(runs, far, cal, cs, None, a, scene_of(a), now=t))
  return out


def test_a_b_only_person_on_a_moving_bicycle_is_dropped():
  # spec test 5: the full pass scores the bicycle crossing 22 m ahead, the far crop its rider too (a person box inside its box): the
  # rider is the bicycle's, left to the driving model, never a B-only pedestrian. Without the bicycle the same person is one
  def y(k):
    return 4. - 0.25 * k  # crossing right at 5 m/s
  rider = lambda k: [(BICYCLE, 22., y(k), 0.7), (PERSON, 22., y(k), 0.7)]  # noqa: E731
  out = run_pair(lambda k: [(BICYCLE, 22., y(k), 0.7)], rider)
  assert not any(out)
  out = run_pair(lambda k: [], lambda k: [(PERSON, 22., y(k), 0.7)])
  assert any(out) and all(o.b_only and o.trackId >= B_ID for objs in out for o in objs)


def test_a_far_run_between_a_frames_full_runs_goes_in_at_its_time():
  # the full pass's runs drain together (DetectionDrain) and the far crop's latest comes beside them: a far run older than the
  # newest full run of the frame goes into B before it, never dropped as out of order
  a, b = SceneTracker(CAMS, 'mici'), TB.TrackerB(SceneTracker(CAMS, 'mici'))
  cal, cs = calibration(), car_state()
  crop, _ = far_crop_at_horizon(a)
  person = lambda t, k: far_rows(crop, [[*box(a, 'road', PERSON, 30., 0.), 0.7, PERSON]], t, k)  # noqa: E731
  for k, (runs_t, far_t) in enumerate([((1.0, 1.1), 1.05), ((1.2, 1.3), 1.25), ((1.4,), 1.35)]):
    runs = [full(a, t, [(CAR, 40., 3.6, 0.8)], 10 * k + i) for i, t in enumerate(runs_t)]
    a.update_runs(runs, cal, cs, None, now=runs_t[-1])
    b.step(runs, person(far_t, k), cal, cs, None, a, scene_of(a), now=runs_t[-1])
  (p,) = [t for t in b.tracker.tracks if t.cls == PERSON]
  assert p.hits == 3 and p.view == 'far'
  assert len(b.objects) == 1 and b.objects[0].trackId == B_ID + p.track_id
  # the same far run on the next frame: taken once
  b.step([], person(1.35, 2), cal, cs, None, a, scene_of(a), now=1.42)
  assert p.hits == 3


def test_tracker_b_logs_its_time_and_its_stops():
  b = TB.TrackerB(SceneTracker(CAMS, 'mici'))
  b.objects = TB.b_only_objects([], [obj(20., 0., track_id=3)], [])
  b.times = [0.0004, 0.0006, 0.0005, 0.002]
  stop_b = Stop(0., 7., False, "pedestrian in path", 15., evidence={'track_id': B_ID + 3})
  stop_a = Stop(0., 7., True, "pedestrian in path", 15., evidence={'track_id': 3})
  with mock.patch.object(TB.cloudlog, 'event') as event:
    for k in range(3):
      b.after_plan([stop_b, stop_a], 10., now=100. + k)
    b.after_plan([], 10., now=100. + TB.LOG_PERIOD_S)
    b.after_plan([], 10., now=101. + TB.LOG_PERIOD_S)
  calls = [(c.args[0], c.kwargs) for c in event.call_args_list]
  stops = [kw for name, kw in calls if name == "turnpilot.tracker_b_stop"]
  assert len(stops) == 1 and stops[0]['track_id'] == B_ID + 3 and stops[0]['camera'] == 'road' and stops[0]['box'] == [0.4, 0.4, 0.42, 0.48]
  assert stops[0]['ask'] == -2.0  # the speed cap's own floor here: the policy's is what keeps it to STOP_DECEL
  (period,) = [kw for name, kw in calls if name == "turnpilot.tracker_b"]
  assert period['b_stop_objects'] == 1 and period['frames'] == 4 and period['p50_ms'] == 0.55 and 1.5 < period['p95_ms'] <= 2.0
  assert b.times == [] and b.stop_ids == set()
