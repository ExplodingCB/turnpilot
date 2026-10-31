"""turnpilotd main() with tracker B (TurnPilotFarPeds; scene/tracker_b.py, PERCEPTION_PLAN item 5, peds P3 variant g:C): the far
crop's people reach the scene policy as objects that never confirm, tracker A and turnpilotScene are what they are with it off,
far runs alone never keep the scene fresh, and a tracker B that raises is dropped for the drive while A carries on."""
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from openpilot.cereal import messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import turnpilotd
from openpilot.selfdrive.turnpilot.detector.decoder import CAR, PERSON, fill_detections
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.scene import tracker as tracker_module
from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import CAMS, box, calibration, far_crop_at_horizon
from openpilot.selfdrive.turnpilot.scene.tests.test_tracker_tiles import far_rows
from openpilot.selfdrive.turnpilot.scene.tracker import STALE_S, SceneTracker
from openpilot.selfdrive.turnpilot.scene_policy import B_ID, CROSSWALK_CONF, STOP_DECEL
from openpilot.selfdrive.turnpilot.tests import test_turnpilotd_main as tm
from openpilot.selfdrive.turnpilot.turn_speed import NO_ACCEL_CAP, NO_SPEED_CAP

Handback = turnpilotd.Handback
T0 = 100.  # camera time of frame 0, s; frame k's is T0 + k DT_MDL (the full pass on even frames, the far crop on odd ones)
_GEOM = SceneTracker(CAMS, 'mici')  # for the boxes' pixels only
CROP, _ = far_crop_at_horizon(_GEOM)


def full_run(k, objects):
  """Frame k's full pass on the road camera: objects (cls, x, y left, conf) standing at ground points."""
  rows = np.array([[*box(_GEOM, 'road', cls, x, y), conf, cls] for cls, x, y, conf in objects], dtype=np.float32).reshape(-1, 6)
  msg = messaging.new_message('turnpilotDetections')
  fill_detections(msg.turnpilotDetections, rows, k, int((T0 + k * DT_MDL) * 1e9), 'road', 0.005)
  return msg.as_reader().turnpilotDetections


def far_run(k, objects):
  return far_rows(CROP, [[*box(_GEOM, 'road', cls, x, y), conf, cls] for cls, x, y, conf in objects], T0 + k * DT_MDL, frame_id=k)


def stream(frames, full_at=lambda k: [(CAR, 40., 3.6, 0.8)], far_at=lambda k: [(PERSON, 25., 0., 0.7)], full_until=None):
  """[(full run or None, far run or None)] per frame: the full pass on even frames (until full_until), the far crop on odd ones."""
  out = []
  for k in range(frames):
    full = full_run(k, full_at(k)) if k % 2 == 0 and (full_until is None or k < full_until) else None
    far = far_run(k, far_at(k)) if k % 2 == 1 else None
    out.append((full, far))
  return out


class StreamSM(tm.LoopSM):
  """LoopSM with a calibrated car and a detector stream: frames[k] = (full pass run, far run), either None."""
  frames: list = []

  def __init__(self, *a, **k):
    super().__init__(*a, **k)
    self.data['extrinsicsCalibration'] = calibration()

  def update(self, timeout):
    super().update(timeout)
    full, far = self.frames[self.frame] if self.frame < len(self.frames) else (None, None)
    self.updated['turnpilotDetections'] = full is not None
    self.updated['turnpilotFarDetections'] = far is not None
    if full is not None:
      self.data['turnpilotDetections'], self.seen['turnpilotDetections'] = full, True
    if far is not None:
      self.data['turnpilotFarDetections'], self.seen['turnpilotFarDetections'] = far, True


def run_stream(frames, far_peds, step=None, **params):
  """main() over frames (stream()) with real trackers A and B, the trackers' clock the camera's. Returns (plans, scenes, errors,
  rows): rows per frame, what the plan's scene policy decided (A's tracks' evidence and stops, B-only ids, the decision)."""
  sent, made, rows, extras = [], [], [], []
  StreamSM.frames = frames
  sm = StreamSM(0, lambda frame: True)
  pm = mock.MagicMock()
  pm.send.side_effect = lambda s, msg: sent.append((s, msg.as_reader()))
  rk = mock.MagicMock()
  rk.keep_time.side_effect = [None] * (len(frames) - 1) + [tm.Stop()]
  params = tm.FakeParams(**{'TurnPilotEnabled': True, 'CarParams': tm.car_params_bytes(), 'DisengageOnAccelerator': True,
                            'TurnPilotFarPeds': far_peds, **params})
  real_plan = turnpilotd.TurnPilot.plan

  def plan(self, *a, **k):
    extras.append(list(k.get('extra_objects', ())))
    msg = real_plan(self, *a, **k)
    sp = self.scene
    rows.append({
      'a': {tid: (ev.confirmed, ev.earned, ev.walker, ev.hits) for tid, ev in sp.tracks.items() if tid < B_ID},
      'a_stops': [(s.evidence['track_id'], s.accel_cap, s.speed_cap, s.confirmed, s.reason) for s in sp.carried if s.evidence['track_id'] < B_ID],
      'b': {tid: (ev.confirmed, ev.earned, ev.walker) for tid, ev in sp.tracks.items() if tid >= B_ID},
      'b_stops': [(s.accel_cap, s.speed_cap, s.confirmed, s.reason) for s in sp.carried if s.evidence['track_id'] >= B_ID],
      'hard': sp.hard, 'curb_logged': set(sp.curb_logged), 'decided': self.scene_now is not None})
    return msg

  def make_tracker(*a):
    made.append(SceneTracker(CAMS, 'mici'))
    return made[-1]
  clock = SimpleNamespace(monotonic=lambda: T0 + max(sm.frame, 0) * DT_MDL)
  with mock.patch.object(messaging, 'SubMaster', return_value=sm), mock.patch.object(messaging, 'PubMaster', return_value=pm), \
       mock.patch.object(messaging, 'sub_sock', return_value=mock.MagicMock()), \
       mock.patch.object(messaging, 'drain_sock', side_effect=lambda sock, wait_for_one=False: sm.drain()), \
       mock.patch.object(turnpilotd, 'Params', return_value=params), mock.patch.object(turnpilotd, 'Ratekeeper', return_value=rk), \
       mock.patch.object(turnpilotd, 'SceneTracker', side_effect=make_tracker), mock.patch.object(turnpilotd, 'DEVICE_CAMERAS', mock.MagicMock()), \
       mock.patch.object(tracker_module, 'time', clock), mock.patch.object(turnpilotd.TurnPilot, 'plan', plan), \
       mock.patch.object(turnpilotd, 'log_error') as log_error, pytest.raises(tm.Stop):
    if step is not None:
      with mock.patch.object(turnpilotd.TrackerB, 'step', side_effect=step, autospec=True):
        turnpilotd.main()
    else:
      turnpilotd.main()
  plans = [msg.turnpilotPlan for s, msg in sent if s == 'turnpilotPlan']
  scenes = [msg for s, msg in sent if s == 'turnpilotScene']
  return SimpleNamespace(plans=plans, scenes=scenes, errors=[c.args[0] for c in log_error.call_args_list], rows=rows, made=made, extras=extras)


def scene_bytes(scenes):
  """turnpilotScene's content, frame by frame (logMonoTime aside: when it was sent)."""
  return [s.turnpilotScene.as_builder().to_bytes() for s in scenes]


def test_identity_tracker_a_and_turnpilot_scene_are_the_same_with_it_on_and_off():
  # spec test 1: full-pass and far runs with a person only the far crop sees (25 m, in the path) and one both see (a confirmed
  # A pedestrian at 15 m, 2 m right), a car beside: turnpilotScene's bytes and every A track's decisions (confirmed, earned,
  # walker, its stops' confirmation and the hard tier) are the same with TurnPilotFarPeds on and off
  both = [(PERSON, 15., -2., 0.8), (CAR, 40., 3.6, 0.8)]
  frames = stream(60, full_at=lambda k: both, far_at=lambda k: [(PERSON, 25., 0., 0.7), *both])
  off, on = run_stream(frames, False), run_stream(frames, True)
  assert not off.errors and not on.errors and len(off.plans) == len(on.plans) == 60
  assert scene_bytes(off.scenes) == scene_bytes(on.scenes) and any(s.turnpilotScene.available for s in on.scenes)
  for r_off, r_on in zip(off.rows, on.rows, strict=True):
    assert (r_off['a'], r_off['a_stops'], r_off['hard']) == (r_on['a'], r_on['a_stops'], r_on['hard'])
  assert any(r['a'] for r in on.rows) and any(r['b'] for r in on.rows)  # both trackers had people in play
  # off: no tracker B at all, no objects of its to the plan (the parent's path to the byte: ScenePolicy.update gets nothing more)
  assert len(off.made) == 1 and all(e == [] for e in off.extras) and not any(r['b'] for r in off.rows)
  assert len(on.made) == 2 and any(e for e in on.extras)


def test_a_far_only_person_in_the_path_is_a_comfortable_stop_never_confirmed_with_it_on_and_nothing_with_it_off():
  # spec test 2: 25 m ahead in the path for 3 s, seen by the far crop alone, the car at 10 m/s (the harness's carState; open loop)
  frames = stream(60)
  on, off = run_stream(frames, True), run_stream(frames, False)
  v = 10.
  asks = [plan_accel(p.accelCap, p.speedCap, v) for p in on.plans]
  assert min(asks) < -0.3 and all(a >= -STOP_DECEL - 1e-6 for a in asks)  # it stops, on the comfortable curve and no harder
  assert all(p.accelCap == NO_ACCEL_CAP for p in on.plans)
  assert any(r['b_stops'] for r in on.rows) and all(s[2] is False and s[0] == NO_ACCEL_CAP and s[3] == "pedestrian in path"
                                                    for r in on.rows for s in r['b_stops'])
  assert all(not c and not e and not w for r in on.rows for c, e, w in r['b'].values())  # never confirmed, earned or a walker
  assert not any(r['hard'] for r in on.rows) and not any(r['curb_logged'] for r in on.rows)
  b_objects = [o for e in on.extras for o in e]
  assert b_objects and all(o.b_only and o.age == 0 and o.trackId >= B_ID and o.conf <= CROSSWALK_CONF - 0.01 + 1e-9 for o in b_objects)
  # off: nothing (the far crop only tags A's tracks)
  assert all(p.speedCap == NO_SPEED_CAP and p.accelCap == NO_ACCEL_CAP for p in off.plans) and not any(r['b_stops'] for r in off.rows)


def test_far_runs_alone_leave_the_scene_unavailable_and_the_detector_hands_back_as_with_it_off():
  # spec test 4: the full pass stops at frame 20 and the far crop runs on: unavailable STALE_S after the last full run, and
  # detectorStopped at the deadline, frame for frame as with TurnPilotFarPeds off
  frames = stream(80, full_until=20)
  on, off = run_stream(frames, True), run_stream(frames, False)
  avail_on, avail_off = [s.turnpilotScene.available for s in on.scenes], [s.turnpilotScene.available for s in off.scenes]
  assert avail_on == avail_off
  last_full = 18
  assert avail_on[last_full] and not any(avail_on[last_full + int(round(STALE_S / DT_MDL)) + 1:])
  assert [p.handback for p in on.plans] == [p.handback for p in off.plans]
  assert on.plans[-1].handback == Handback.detectorStopped and on.plans[last_full].handback == Handback.none
  assert scene_bytes(on.scenes) == scene_bytes(off.scenes)
  # B's objects stop with the scene: none while it is unavailable
  assert all(not e for e, a in zip(on.extras, avail_on, strict=True) if not a)


def test_an_exception_in_tracker_b_drops_it_for_the_drive_and_the_plan_goes_on_from_a():
  # spec test 6
  calls = []
  real_step = turnpilotd.TrackerB.step

  def step(self, *a, **k):
    calls.append(1)
    if len(calls) == 5:
      raise RuntimeError("synthetic tracker B failure")
    return real_step(self, *a, **k)
  frames = stream(40)
  on, off = run_stream(frames, True, step=step), run_stream(frames, False)
  assert on.errors == ["turnpilotd.tracker_b_failed"] and len(calls) == 5  # said once, and never stepped (nor built) again
  assert len(on.made) == 2 and len(on.plans) == len(on.scenes) == 40
  assert scene_bytes(on.scenes) == scene_bytes(off.scenes)
  assert all(not e for e in on.extras[4:])
  # from the frame after, the plan is tracker A's alone, as with it off: once B's stop has let go, the same plans
  assert [p.speedCap for p in on.plans[-10:]] == [p.speedCap for p in off.plans[-10:]]


@pytest.mark.parametrize("value,on", [(None, False), (True, True), (False, False), (b'1', True)])
def test_main_reads_tracker_bs_param(value, on):
  frames = stream(4)
  r = run_stream(frames, value)
  assert len(r.made) == (2 if on else 1)


def test_tracker_bs_param_unknown_to_the_build_is_off():
  class Old(tm.FakeParams):
    def get_bool(self, key):
      if key == "TurnPilotFarPeds":
        raise turnpilotd.UnknownKeyName(key)
      return super().get_bool(key)
  with mock.patch.object(turnpilotd, 'log_error') as log_error:
    assert turnpilotd.far_peds_for(Old()) is False
  assert log_error.call_count == 1
