"""STOP_SIGNS.md SG3b (1.3, 1.7): the tracker's views. Every run keeps its view (the latest RawFrame per view in raw_by_view),
a track says the view and camera time of its last match (TrackedObject.view, tSeen), runs drained together go in camera-time
order, and a track coasts MAX_COAST_S, or SWEEP_COAST_S while a stop sweep runs (two missed passes of a 5 Hz view)."""
import numpy as np

from openpilot.cereal import custom, messaging
from openpilot.selfdrive.turnpilot.detector.decoder import CAR, fill_detections
from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import CAMS, box, calibration, car_state
from openpilot.selfdrive.turnpilot.scene.tracker import MAX_COAST_S, SWEEP_COAST_S, SceneTracker, view_key

View = custom.TurnpilotDetections.View


def run(trk, camera: str, view: str, t: float, objects, frame_id=0):
  """A TurnpilotDetections reader of one detector run: objects (cls, x, y left, conf) as camera sees them, view as modeld
  says (a side view's boxes are in the wide camera's fisheye pixels)."""
  rows = np.array([[*box(trk, camera, cls, x, y), conf, cls] for cls, x, y, conf in objects], dtype=np.float32).reshape(-1, 6)
  msg = messaging.new_message('turnpilotDetections')
  fill_detections(msg.turnpilotDetections, rows, frame_id, int(t * 1e9), camera, 0.005)
  msg.turnpilotDetections.view = view
  return msg.as_reader().turnpilotDetections


def test_view_keys():
  assert view_key('road', 'full') == 'road' and view_key('wide', 'full') == 'full'
  assert view_key('wide', 'sideLeft') == 'sideLeft' and view_key('wide', 'sideRight') == 'sideRight' and view_key('road', 'far') == 'far'


def test_each_view_keeps_its_latest_raw_frame():
  trk, cal = SceneTracker(CAMS, 'mici'), calibration()
  car_left = [(CAR, 6., 18., 0.6)]   # 72 deg left: the left side view's
  car_right = [(CAR, 6., -18., 0.6)]
  trk.update(run(trk, 'road', 'full', 1.00, [(CAR, 30., 0., 0.6)], 1), cal, car_state(), now=1.00)
  trk.update(run(trk, 'wide', 'full', 1.05, [(CAR, 12., 3., 0.6)], 2), cal, car_state(), now=1.05)
  trk.update(run(trk, 'wide', 'sideLeft', 1.10, car_left, 3), cal, car_state(), now=1.10)
  trk.update(run(trk, 'wide', 'sideRight', 1.15, car_right, 4), cal, car_state(), now=1.15)
  by = trk.raw_by_view
  assert set(by) == {'road', 'full', 'sideLeft', 'sideRight'}
  for v, t, camera, view in (('road', 1.00, 'road', 'full'), ('full', 1.05, 'wide', 'full'), ('sideLeft', 1.10, 'wide', 'sideLeft'),
                             ('sideRight', 1.15, 'wide', 'sideRight')):
    assert abs(by[v].t - t) < 1e-9 and (by[v].camera, by[v].view) == (camera, view)
  assert [b.view for b in by['sideLeft'].boxes] == ['sideLeft'] and by['sideLeft'].boxes[0].bearing_deg > 60.
  assert by['sideRight'].boxes[0].bearing_deg < -60.
  assert trk.raw is by['sideRight']  # the latest run, as before
  trk.update(run(trk, 'road', 'full', 1.20, [], 5), cal, car_state(), now=1.20)
  assert abs(trk.raw_by_view['road'].t - 1.20) < 1e-9 and abs(trk.raw_by_view['sideLeft'].t - 1.10) < 1e-9  # each view its own


def test_a_track_says_its_view_and_camera_time():
  trk, cal = SceneTracker(CAMS, 'mici'), calibration()
  car = [(CAR, 6., 18., 0.7)]
  trk.update(run(trk, 'wide', 'sideLeft', 1.0, car, 1), cal, car_state(), now=1.0)
  trk.update(run(trk, 'wide', 'sideLeft', 1.2, car, 2), cal, car_state(), now=1.2)
  msg = messaging.new_message('turnpilotScene')
  trk.fill_scene(msg.turnpilotScene)
  objs = msg.as_reader().turnpilotScene.objects
  assert len(objs) == 1 and objs[0].view == View.sideLeft and abs(objs[0].tSeen - 1.2) < 1e-9
  # matched next by the full wide pass: its view and its time
  trk.update(run(trk, 'wide', 'full', 1.3, car, 3), cal, car_state(), now=1.3)
  msg = messaging.new_message('turnpilotScene')
  trk.fill_scene(msg.turnpilotScene)
  o = msg.as_reader().turnpilotScene.objects[0]
  assert o.view == View.full and abs(o.tSeen - 1.3) < 1e-9
  # a run that misses it leaves its time at the last match
  trk.update(run(trk, 'wide', 'sideRight', 1.4, [], 4), cal, car_state(), now=1.4)
  msg = messaging.new_message('turnpilotScene')
  trk.fill_scene(msg.turnpilotScene)
  o = msg.as_reader().turnpilotScene.objects[0]
  assert o.view == View.full and abs(o.tSeen - 1.3) < 1e-9


def coasting(coast_s: float | None, gap: float) -> bool:
  """A car matched twice, then a run gap s after its last match that misses it: still published?"""
  trk, cal = SceneTracker(CAMS, 'mici'), calibration()
  if coast_s is not None:
    trk.coast_s = coast_s
  car = [(CAR, 6., 18., 0.7)]
  trk.update(run(trk, 'wide', 'sideLeft', 1.0, car, 1), cal, car_state(), now=1.0)
  trk.update(run(trk, 'wide', 'sideLeft', 1.2, car, 2), cal, car_state(), now=1.2)
  return bool(trk.update(run(trk, 'wide', 'full', 1.2 + gap, [], 3), cal, car_state(), now=1.2 + gap))


def test_a_sweep_coasts_a_track_through_two_missed_passes_of_a_5_hz_view():
  assert MAX_COAST_S == 0.5 and SWEEP_COAST_S == 0.65
  # 0.6 s between hits of one side view: two of its passes missed
  assert not coasting(None, 0.6) and coasting(None, 0.45)
  assert coasting(SWEEP_COAST_S, 0.6) and not coasting(SWEEP_COAST_S, 0.7)


def test_runs_drained_together_go_in_camera_time_order():
  trk, cal = SceneTracker(CAMS, 'mici'), calibration()
  car = [(CAR, 6., 18., 0.7)]
  late = run(trk, 'wide', 'sideLeft', 1.10, car, 3)
  early = run(trk, 'road', 'full', 1.05, [(CAR, 30., 0., 0.6)], 2)
  trk.update_runs([late, early], cal, car_state(), now=1.12)
  assert trk.raw_new and trk.new_views == {'road', 'sideLeft'} and trk.raw is trk.raw_by_view['sideLeft'] and abs(trk.last_t - 1.10) < 1e-9
  # nothing this tick: nothing new, the scene still available until STALE_S
  trk.update_runs([], cal, car_state(), now=1.15)
  assert not trk.raw_new and trk.new_views == set() and trk.available
  # a run older than the newest taken is dropped (the filter only moves forward), the newer one in the same batch taken
  trk.update_runs([run(trk, 'wide', 'full', 1.08, car, 4), run(trk, 'wide', 'sideLeft', 1.30, car, 5)], cal, car_state(), now=1.31)
  assert trk.new_views == {'sideLeft'} and abs(trk.last_t - 1.30) < 1e-9 and 'full' not in trk.raw_by_view
