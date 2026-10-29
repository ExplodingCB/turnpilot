import time
from collections import deque

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.turnpilot.detector.decoder import PERSON, TRAFFIC_LIGHT
from openpilot.selfdrive.turnpilot.detector.graph import letterbox
from openpilot.selfdrive.turnpilot.detector.light_state import LightHead
from openpilot.selfdrive.turnpilot.detector.modeld_detector import (LIGHT_BUDGET_S, PAINT_BUDGET_S, RUN_EVERY, STEP_BUDGET_S, TIME_WINDOW,
                                                                    WIDE_HOLD_S, LanePaintStatus, LightStatus, ModeldDetector,
                                                                    camera_lights_mode)
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint


class FakeOutput:
  def __init__(self, raw):
    self.raw = raw

  def numpy(self):
    return self.raw


class FakeRun:
  """Stands in for the compiled jit: records which frame it got, returns one person."""
  def __init__(self, lb):
    self.calls, self.fail = [], False
    self.raw = np.zeros((1, 300, 6), np.float32)
    self.raw[0, 0] = [*lb.to_input(100, 200), *lb.to_input(160, 400), 0.9, PERSON]
    self.raw[0, 1] = [*lb.to_input(700, 100), *lb.to_input(715, 145), 0.8, TRAFFIC_LIGHT]

  def __call__(self, input_frame):
    if self.fail:
      raise RuntimeError("gpu fell over")
    self.calls.append(input_frame)
    return FakeOutput(self.raw)


class Meta:
  def __init__(self, frame_id):
    self.frame_id, self.timestamp_eof = frame_id, frame_id * 50_000_000


class FakeLanePaint:
  """Stands in for LanePaintReader: records the frame and boxes it got."""
  def __init__(self):
    self.calls, self.fail, self.delay = [], False, 0.

  def read(self, frame, meta, boxes):
    if self.fail:
      raise RuntimeError("bad frame")
    if self.delay:
      time.sleep(self.delay)
    self.calls.append((frame, meta.frame_id, None if boxes is None else len(boxes)))
    return [UNKNOWN, LinePaint('broken', 'white', 0.9, 0.3), LinePaint('solid', 'yellow', 0.8, 0.95), UNKNOWN], LanePaintStatus.read


class FakeLights:
  """Stands in for CameraLightReader: records the frame and light boxes it got, returns one governing red head."""
  def __init__(self):
    self.calls, self.fail, self.delay = [], False, 0.

  def read(self, frame, meta, boxes):
    if self.fail:
      raise RuntimeError("bad frame")
    if self.delay:
      time.sleep(self.delay)
    self.calls.append((frame, meta.frame_id, boxes[:, 5].tolist()))
    head = LightHead(3, (0.52, 0.13, 0.53, 0.19), 0.8, 'red', 4, 'red', 'read', 1.5, 32., True)
    return [head], LightStatus.read


class FakeModelState:
  frames = ['host0', 'host1']


class PM:
  def __init__(self):
    self.sent = []

  def send(self, name, msg):
    self.sent.append(msg.as_reader().turnpilotDetections)


def make_detector(cameras=None, det_time=0.005):
  det = ModeldDetector.__new__(ModeldDetector)  # skips loading a pkl
  det.lb = letterbox(1344, 760)
  det.run = FakeRun(det.lb)
  det.model_state = FakeModelState()
  det.cameras = cameras or {'road': 0, 'wide': 1}
  det.lane_paint = FakeLanePaint() if 'road' in det.cameras else None
  det.frames = {cam: f'frame{i}' for cam, i in det.cameras.items()}
  det.times = deque([det_time])
  det.paint_times = deque(maxlen=TIME_WINDOW)
  det.light_times = deque(maxlen=TIME_WINDOW)
  det.since_run, det.wide_until = RUN_EVERY, -1.0
  return det


def with_lights(det, mode='shadow'):
  det.light_mode, det.lights = mode, FakeLights()
  return det


class TestModeldDetector(OpenpilotTestCase):
  def test_camera_choice(self):
    det = make_detector()
    assert det.choose_camera(0.001, log.Desire.none, 0.0) == 'road'
    assert det.choose_camera(-0.02, log.Desire.none, 1.0) == 'wide'
    assert det.choose_camera(0.0, log.Desire.none, 1.0 + WIDE_HOLD_S - 0.1) == 'wide'  # held after the turn
    assert det.choose_camera(0.0, log.Desire.none, 1.0 + WIDE_HOLD_S + 0.1) == 'road'
    assert det.choose_camera(0.0, log.Desire.turnRight, 10.0) == 'wide'
    assert det.choose_camera(0.0, log.Desire.laneChangeLeft, 20.0) == 'road'
    assert make_detector({'wide': 0}).choose_camera(0.0, log.Desire.none, 0.0) == 'wide'
    assert make_detector({'road': 0}).choose_camera(0.05, log.Desire.turnLeft, 0.0) == 'road'

  def test_the_plans_view_overrides_the_turns_wide_hold(self):
    # CRITIQUE REQUIRED 3: from an unprotected left's go to its commit the road camera only, whatever the turn desire or the
    # wide hold after it; held at its line, the road camera with the wide camera one run in WIDE_EVERY (its exit crosswalk)
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import WIDE_EVERY
    det = make_detector()
    assert det.choose_camera(0.05, log.Desire.turnLeft, 0.0, 'road') == 'road'
    assert det.choose_camera(0.0, log.Desire.none, 0.5, 'road') == 'road'  # inside the turn desire's wide hold
    picks = [det.choose_camera(0.05, log.Desire.turnLeft, 1.0 + 0.1 * k, 'roadAndWide') for k in range(3 * WIDE_EVERY)]
    assert picks.count('wide') == 3 and picks.count('road') == 3 * (WIDE_EVERY - 1) and WIDE_EVERY >= 3
    assert 'wide' not in picks[:WIDE_EVERY - 1]  # the road camera first
    # auto: as before
    assert det.choose_camera(0.05, log.Desire.turnLeft, 10.0, 'auto') == 'wide'
    # a device with one camera runs it, whatever the view
    assert make_detector({'wide': 0}).choose_camera(0.0, log.Desire.none, 0.0, 'road') == 'wide'

  def test_step_takes_the_view(self):
    det, pm = make_detector(), PM()
    for k in range(3 * RUN_EVERY):
      det.step(pm, det.model_state, 0.010, 0.05, log.Desire.turnLeft, [Meta(100 + k), Meta(200 + k)], view='road')
    assert det.run.calls == ['frame0'] * 3 and all(str(td.camera) == 'road' for td in pm.sent)

  def test_every_nth_frame_and_message(self):
    det, pm = make_detector(), PM()
    for k in range(3 * RUN_EVERY):
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(100 + k), Meta(200 + k)])
    assert len(pm.sent) == 3 and det.run.calls == ['frame0'] * 3
    td = pm.sent[-1]
    assert str(td.camera) == 'road' and td.frameId == 100 + 2 * RUN_EVERY and len(td.detections) == 1
    np.testing.assert_allclose([td.detections[0].x0, td.detections[0].y1], [100, 400], atol=1e-3)

  def test_wide_uses_extra_frame(self):
    det, pm = make_detector(), PM()
    det.step(pm, det.model_state, 0.010, 0.03, log.Desire.none, [Meta(1), Meta(2)])
    assert det.run.calls == ['frame1'] and str(pm.sent[0].camera) == 'wide' and pm.sent[0].frameId == 2

  def test_budget(self):
    det, pm = make_detector(det_time=0.006), PM()
    for _ in range(10):
      det.step(pm, det.model_state, STEP_BUDGET_S - 0.005, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert pm.sent == []  # no room this step, every step
    det.step(pm, det.model_state, STEP_BUDGET_S - 0.007, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert len(pm.sent) == 1  # the first frame with room runs, even if it's not on the N-frame grid

  def test_failure_retries_and_a_wrong_model_raises(self):
    det, pm = make_detector(), PM()
    det.run.fail = True
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert det.failures == 1 and det.retry_at > 0. and pm.sent == []  # test_detector_recovery.py has the backoff
    det = make_detector()
    with pytest.raises(ValueError):  # no quiet off: DetectorHost logs it and retries
      det.step(pm, object(), 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert pm.sent == []

  def test_lane_paint_rides_along_on_the_road_frame(self):
    det, pm = make_detector(), PM()
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(7), Meta(8)])
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(9), Meta(10)])  # not on the N-frame grid
    det.since_run = RUN_EVERY
    det.step(pm, det.model_state, 0.010, 0.03, log.Desire.none, [Meta(11), Meta(12)])  # the detector looks through the wide
    # the road camera's host frame, behind the detector's boxes on it; nothing while it looks through the wide camera, where
    # there are no boxes for the road frame
    assert det.lane_paint.calls == [('host0', 7, 1)]
    assert len(pm.sent) == 2 and str(pm.sent[1].camera) == 'wide' and len(pm.sent[1].lanePaint) == 0
    paint = pm.sent[0].lanePaint
    assert [(str(p.kind), str(p.color)) for p in paint] == [('unknown', 'unknown'), ('broken', 'white'), ('solid', 'yellow'),
                                                           ('unknown', 'unknown')]
    assert abs(paint[1].confidence - 0.9) < 1e-6
    assert pm.sent[0].lanePaintStatus == LanePaintStatus.read and pm.sent[1].lanePaintStatus == LanePaintStatus.wideCamera
    assert pm.sent[0].executionTime > 0. and pm.sent[0].lanePaintTime >= 0.

  def test_lane_paint_failure_leaves_the_detector_on(self):
    det, pm = make_detector(), PM()
    det.lane_paint.fail = True
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert det.lane_paint is None and len(pm.sent) == 1 and len(pm.sent[0].detections) == 1
    assert len(pm.sent[0].lanePaint) == 0 and pm.sent[0].lanePaintStatus == LanePaintStatus.disabled  # unknown, said out loud
    det.since_run = RUN_EVERY
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(3), Meta(4)])
    assert len(pm.sent) == 2 and pm.sent[1].lanePaintStatus == LanePaintStatus.disabled
    # no road camera, no paint
    det, pm = make_detector({'wide': 0}), PM()
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(1)])
    assert len(pm.sent) == 1 and len(pm.sent[0].lanePaint) == 0 and pm.sent[0].lanePaintStatus == LanePaintStatus.disabled

  def test_slow_lane_paint_turns_off_and_the_detector_keeps_going(self):
    det, pm = make_detector(), PM()
    det.lane_paint.delay = PAINT_BUDGET_S * 1.5
    for k in range(TIME_WINDOW):
      det.since_run = RUN_EVERY
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)])
    assert det.lane_paint is None and len(pm.sent) == TIME_WINDOW
    assert all(m.lanePaintTime > PAINT_BUDGET_S for m in pm.sent)
    det.since_run = RUN_EVERY
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(100), Meta(101)])
    assert pm.sent[-1].lanePaintStatus == LanePaintStatus.disabled and len(pm.sent[-1].detections) == 1

  def test_paint_time_counts_toward_the_step_budget(self):
    det, pm = make_detector(det_time=0.006), PM()
    det.paint_times.extend([0.003] * 3)
    det.step(pm, det.model_state, STEP_BUDGET_S - 0.008, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert pm.sent == []  # 6 ms detector + 3 ms paint don't fit in 8 ms
    det.lane_paint = None  # paint off: its time no longer counts
    det.step(pm, det.model_state, STEP_BUDGET_S - 0.008, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert len(pm.sent) == 1


class TestCameraLights(OpenpilotTestCase):
  def test_lights_ride_along_on_the_road_frame(self):
    det, pm = with_lights(make_detector()), PM()
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(7), Meta(8)])
    det.since_run = RUN_EVERY
    det.step(pm, det.model_state, 0.010, 0.03, log.Desire.none, [Meta(11), Meta(12)])  # through the wide camera
    # the road camera's host frame and its class-9 boxes alone; the detections and the lane paint's mask never see a light
    assert det.lights.calls == [('host0', 7, [TRAFFIC_LIGHT])]
    assert det.lane_paint.calls == [('host0', 7, 1)]
    td = pm.sent[0]
    assert [d.cls for d in td.detections] == [PERSON]
    assert td.lightStatus == LightStatus.read and td.lightTime >= 0.
    lights = td.lights
    assert str(lights.source) == 'camera' and lights.shadow
    (light,) = lights.lights
    assert str(light.state) == 'red' and abs(light.distance - 32.) < 1e-4 and light.votes == 4 and light.governs
    assert abs(light.conf - 0.8) < 1e-6 and abs(light.bearing - 1.5) < 1e-6 and len(light.box) == 4
    assert pm.sent[1].lightStatus == LightStatus.wideCamera and len(pm.sent[1].lights.lights) == 0

  def test_add_only_mode_is_not_shadow(self):
    det, pm = with_lights(make_detector(), 'addOnly'), PM()
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert str(pm.sent[0].lights.source) == 'camera' and not pm.sent[0].lights.shadow

  def test_off_reads_nothing(self):
    det, pm = with_lights(make_detector(), 'off'), PM()
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert det.lights.calls == [] and pm.sent[0].lightStatus == LightStatus.off and str(pm.sent[0].lights.source) == 'none'
    det, pm = make_detector({'wide': 0}), PM()  # no road camera
    det.light_mode = 'shadow'
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(1)])
    assert pm.sent[0].lightStatus == LightStatus.off

  def test_light_failure_turns_them_off_loudly_and_the_detector_keeps_going(self):
    det, pm = with_lights(make_detector()), PM()
    det.lights.fail = True
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert det.lights is None and det.lights_failed and len(pm.sent) == 1 and len(pm.sent[0].detections) == 1
    assert pm.sent[0].lightStatus == LightStatus.disabled and len(pm.sent[0].lights.lights) == 0
    assert len(pm.sent[0].lanePaint) == 4  # the paint is its own
    det.since_run = RUN_EVERY
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(3), Meta(4)])
    assert pm.sent[1].lightStatus == LightStatus.disabled  # for the drive, never back to off

  def test_slow_lights_turn_off_and_the_detector_keeps_going(self):
    det, pm = with_lights(make_detector()), PM()
    det.lights.delay = LIGHT_BUDGET_S * 1.5
    for k in range(TIME_WINDOW):
      det.since_run = RUN_EVERY
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)])
    assert det.lights is None and all(m.lightTime > LIGHT_BUDGET_S for m in pm.sent)
    det.since_run = RUN_EVERY
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(100), Meta(101)])
    assert pm.sent[-1].lightStatus == LightStatus.disabled and len(pm.sent[-1].detections) == 1

  def test_light_time_counts_toward_the_step_budget(self):
    det, pm = with_lights(make_detector(det_time=0.006)), PM()
    det.lane_paint = None
    det.light_times.extend([0.003] * 3)
    det.step(pm, det.model_state, STEP_BUDGET_S - 0.008, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert pm.sent == []
    det.light_mode = 'off'  # off: its time no longer counts
    det.step(pm, det.model_state, STEP_BUDGET_S - 0.008, 0.0, log.Desire.none, [Meta(1), Meta(2)])
    assert len(pm.sent) == 1


class LightParams:
  def __init__(self, value=None, unknown=False):
    self.value, self.unknown = value, unknown

  def get(self, key):
    assert key == "TurnPilotCameraLights"
    if self.unknown:
      raise UnknownKeyName(key)
    return self.value


def test_camera_lights_mode():
  assert camera_lights_mode(LightParams(None)) == 'shadow'  # the default: shadow, logged, never acted on
  assert camera_lights_mode(LightParams('off')) == 'off'
  assert camera_lights_mode(LightParams('addOnly')) == 'addOnly'
  assert camera_lights_mode(LightParams('loud')) == 'shadow'
  assert camera_lights_mode(LightParams(unknown=True)) == 'shadow'  # a build whose params predate the key


class TestLanePaintReader(OpenpilotTestCase):
  def test_reads_this_frames_lines_off_the_bus(self):
    from openpilot.cereal import messaging
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import LanePaintReader
    from openpilot.selfdrive.turnpilot.scene.tests.test_lane_paint import DT, HEIGHT, LINE_YS, RPY, V_EGO, dashed, render, solid, WHITE
    pm = messaging.PubMaster(['modelV2', 'extrinsicsCalibration', 'deviceState', 'narrowRoadCameraState'])
    reader = LanePaintReader(1344, 760)
    msg = messaging.new_message('deviceState')
    msg.deviceState.deviceType = 'mici'
    pm.send('deviceState', msg)
    msg = messaging.new_message('narrowRoadCameraState')
    msg.narrowRoadCameraState.sensor = 'os04c10'
    pm.send('narrowRoadCameraState', msg)
    msg = messaging.new_message('extrinsicsCalibration')
    msg.extrinsicsCalibration.rpyCalib, msg.extrinsicsCalibration.calStatus = RPY, 'calibrated'
    pm.send('extrinsicsCalibration', msg)
    x = np.linspace(0., 100., 33)
    paint = None
    for k in range(20):
      msg = messaging.new_message('modelV2')
      msg.modelV2.frameId = 100 + k
      for line, y in zip(msg.modelV2.init('laneLines', 4), LINE_YS, strict=True):
        line.x, line.y, line.z = x.tolist(), [y] * len(x), [HEIGHT] * len(x)
      msg.modelV2.laneLineProbs = [0.9] * 4
      pm.send('modelV2', msg)
      frame, _ = render([None, dashed(WHITE), solid(WHITE), None], V_EGO * DT * k, seed=k)
      meta = Meta(100 + k)
      meta.timestamp_eof = int(k * DT * 1e9)
      if k == 19:
        assert reader.read(frame, Meta(99), None) == ([UNKNOWN] * 4, LanePaintStatus.noModel)  # another frame's lines
      paint, status = reader.read(frame, meta, None)
      assert status == LanePaintStatus.read
    assert [(p.kind, p.color) for p in paint[1:3]] == [('broken', 'white'), ('solid', 'white')], paint


class TestCameraLightReader(OpenpilotTestCase):
  def test_reads_on_the_calibrated_road_frame(self):
    from openpilot.cereal import messaging
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import CameraLightReader, frame_bus
    from openpilot.selfdrive.turnpilot.detector.light_state import VOTE_N
    from openpilot.selfdrive.turnpilot.detector.tests.test_light_state import head, scene
    pm = messaging.PubMaster(['modelV2', 'extrinsicsCalibration', 'deviceState', 'narrowRoadCameraState'])
    reader = CameraLightReader(frame_bus())
    frame, boxes = scene([head(0, 'red')])
    assert reader.read(frame, Meta(1), boxes) == ([], LightStatus.noCalibration)  # no device yet
    msg = messaging.new_message('deviceState')
    msg.deviceState.deviceType = 'mici'
    pm.send('deviceState', msg)
    msg = messaging.new_message('narrowRoadCameraState')
    msg.narrowRoadCameraState.sensor = 'os04c10'
    pm.send('narrowRoadCameraState', msg)
    msg = messaging.new_message('extrinsicsCalibration')
    msg.extrinsicsCalibration.rpyCalib, msg.extrinsicsCalibration.calStatus = [0., 0., 0.], 'uncalibrated'
    pm.send('extrinsicsCalibration', msg)
    assert reader.read(frame, Meta(2), boxes) == ([], LightStatus.noCalibration)  # no horizon to gate by
    msg = messaging.new_message('extrinsicsCalibration')
    msg.extrinsicsCalibration.rpyCalib, msg.extrinsicsCalibration.calStatus = [0., 0., 0.], 'calibrated'
    pm.send('extrinsicsCalibration', msg)
    for k in range(VOTE_N):
      heads, status = reader.read(frame, Meta(10 + k), boxes)
    assert status == LightStatus.read and [(h.state, h.governs) for h in heads] == [('red', True)]


# --- the stop sweep and the side views (STOP_SIGNS.md sections 1.1, 1.3 and 6; catalogue rows G17, E7-E11 and E14) ---
CAL_RPY, CAL_EULER = (-0.0004, 0.16485, 0.004535), (-2.1e-05, 0.00551, -0.00276)  # the demo route's comma four


class SyncExecutor:
  """Runs a submitted recompute at once (the side views' maps)."""
  def submit(self, fn, *args):
    from concurrent.futures import Future
    f = Future()
    f.set_result(fn(*args))
    return f


class HeldExecutor:
  """Holds a submitted recompute until finish(): the side views stay invalid until then."""
  def __init__(self):
    self.pending = []

  def submit(self, fn, *args):
    from concurrent.futures import Future
    f = Future()
    self.pending.append((f, fn, args))
    return f

  def finish(self):
    for f, fn, args in self.pending:
      f.set_result(fn(*args))
    self.pending = []


class FakeSideRun:
  """Stands in for the side-view jit: records which view's maps it got, returns a car and our own dash in view pixels."""
  def __init__(self):
    self.calls, self.fail, self.delay = [], False, 0.
    self.raw = np.zeros((1, 300, 6), np.float32)
    self.raw[0, 0] = [300, 160, 380, 215, 0.7, 2]   # a car on the horizon
    self.raw[0, 1] = [20, 380, 760, 512, 0.9, 2]    # the dash: wider than 0.9 of the view, at its bottom

  def __call__(self, input_frame, luma_idx, chroma_idx, weights):
    if self.fail:
      raise RuntimeError("gpu fell over")
    if self.delay:
      time.sleep(self.delay)
    self.calls.append((input_frame, luma_idx.split('.')[0]))
    return FakeOutput(self.raw)


def fake_upload(name, side_map):
  return f'{name}.luma', f'{name}.chroma', f'{name}.weights'


class SweepPM:
  def __init__(self):
    self.msgs, self.far = [], []

  def send(self, name, msg):
    r = msg.as_reader()
    (self.msgs if name == 'turnpilotDetections' else self.far).append(getattr(r, name))

  @property
  def runs(self):
    return [(str(m.camera), str(m.view)) for m in self.msgs]


def side_detector(rpy=CAL_RPY, side_time=0.007, det_time=0.005, executor=None, ready=True):
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import SideViews
  from openpilot.selfdrive.turnpilot.detector.sideview import SIDE_VIEWS
  from openpilot.selfdrive.turnpilot.scene.ranging import wide_lens
  det = make_detector(det_time=det_time)
  det.model_state.frames = [np.zeros(1, np.uint8), np.zeros(1, np.uint8)]  # sector quality reads nothing here
  side = SideViews(FakeSideRun(), 1344, 760, 'frame1', executor=executor or SyncExecutor(), upload=fake_upload)
  side.lens = wide_lens(DEVICE_CAMERAS[('mici', 'os04c10')].wide_road, 'mici')
  side.calibration = lambda: None if rpy is None else (list(rpy), list(CAL_EULER))
  side.quality = None  # E11 tests it on a real frame
  side.warmup_s = side_time
  det.side = side
  if ready:
    for _ in range(len(SIDE_VIEWS)):  # the maps for this calibration, on the device a view at a time
      side.prepare(True)
    assert side.geometry is not None and set(side.device) == {'sideLeft', 'sideRight'}
  return det


def sweep_steps(det, pm, n, view='stopSweep', v_ego=0., model_time=0.010, start=0, desire=log.Desire.none, curvature=0.):
  for k in range(n):
    det.step(pm, det.model_state, model_time, curvature, desire, [Meta(start + 2 * k), Meta(start + 2 * k + 1)], view, 0., v_ego)


HELD = [('wide', 'full'), ('wide', 'sideLeft'), ('road', 'full'), ('wide', 'sideRight')]
CROSSING = [('wide', 'full'), ('wide', 'sideLeft'), ('wide', 'full'), ('wide', 'sideRight')]


class TestStopSweep(OpenpilotTestCase):
  def test_E10_a_car_standing_straight_at_a_stop_runs_the_held_cycle(self):
    # today a car standing straight at a stop line (no curvature, no turn desire) runs the road camera alone, which can't see
    # cross traffic at 60-85 deg; under stopSweep the even steps alternate the full wide pass and the road camera
    det = make_detector()
    assert [det.choose_camera(0.0, log.Desire.none, 0.1 * k, 'stopSweep') for k in range(4)] == ['wide', 'road'] * 2
    assert [det.choose_camera(0.0, log.Desire.none, 1. + 0.1 * k, 'stopSweepGo') for k in range(3)] == ['wide'] * 3
    # and the odd steps alternate the side views: [full wide, left, road, right], 5 Hz a view
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SideViewStatus
    det, pm = side_detector(), SweepPM()
    sweep_steps(det, pm, 8)
    assert pm.runs == HELD * 2
    assert [c[1] for c in det.side.run.calls] == ['sideLeft', 'sideRight'] * 2 and all(c[0] == 'frame1' for c in det.side.run.calls)
    assert all(m.sideViewStatus == SideViewStatus.running for m in pm.msgs)
    # the crossing cycle: the full wide pass on every even step, the sides on the odd ones
    det, pm = side_detector(), SweepPM()
    sweep_steps(det, pm, 8, view='stopSweepGo')
    assert pm.runs == CROSSING * 2
    # out of the sweep, as before: no side view, the road camera straight
    sweep_steps(det, pm, 4, view='auto', start=100)
    assert pm.runs[8:] == [('road', 'full')] * 2 and len(det.side.run.calls) == 4

  def test_E10_side_boxes_go_back_to_fisheye_pixels_on_the_wide_camera_without_the_dash(self):
    det, pm = side_detector(), SweepPM()
    sweep_steps(det, pm, 2)
    td = pm.msgs[1]
    assert str(td.camera) == 'wide' and str(td.view) == 'sideLeft' and td.frameId == 3  # the wide camera's frame of the step
    (d,) = td.detections  # the dash's box is dropped
    geom = det.side.geometry
    u, v = geom.to_fisheye('sideLeft', np.array([300., 380., 340.]), np.array([160., 215., 215.]))
    assert d.x0 <= min(u) + 0.5 and d.x1 >= max(u) - 0.5 and d.y0 <= min(v) + 0.5 and d.y1 >= max(v) - 0.5
    assert d.x1 < 1344 / 2  # left of the frame's centre: the left view
    assert td.lens == det.side.lens.spec  # the lens the views were mapped with, for SG4a's mismatch check

  def test_E14_no_side_view_at_road_speed(self):
    # the speed gate: above SIDE_VIEW_V_MAX the odd steps stay idle, so the approach costs what the detector costs today
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SIDE_VIEW_V_MAX, SideViewStatus
    assert SIDE_VIEW_V_MAX == 2.0
    for view in ('stopSweep', 'stopSweepGo'):
      det, pm = side_detector(), SweepPM()
      sweep_steps(det, pm, 8, view=view, v_ego=5.)
      assert det.side.run.calls == [] and all(m.view == 'full' for m in pm.msgs) and len(pm.msgs) == 4
      assert all(m.sideViewStatus == SideViewStatus.speedGated for m in pm.msgs)  # the decider counts only views the gate runs
      sweep_steps(det, pm, 8, view=view, v_ego=1.5, start=100)
      assert [c[1] for c in det.side.run.calls] == ['sideLeft', 'sideRight'] * 2
      assert pm.msgs[-1].sideViewStatus == SideViewStatus.running
    # no speed known: no side view
    det, pm = side_detector(), SweepPM()
    sweep_steps(det, pm, 4, v_ego=None)
    assert det.side.run.calls == []

  def test_G17_a_stop_sign_just_after_a_turn_still_gets_the_sweep(self):
    # a stop 30 m after a turn: the turn's wide hold has run out and its desire is off, or its desire is still out. Either
    # way the sweep runs its own cycle: the wide camera at the hold, whatever the turn-desire state
    for desire, curvature in ((log.Desire.none, 0.0), (log.Desire.turnRight, 0.05)):
      det, pm = side_detector(), SweepPM()
      det.wide_until = -1.0 if desire == log.Desire.none else 1e9
      sweep_steps(det, pm, 8, desire=desire, curvature=curvature)
      assert pm.runs == HELD * 2, desire

  def test_a_shadow_plan_still_sweeps(self):
    # modeld reads the view through fresh_plan, whatever the plan's state: a shadow drive (and step S3's dwell with
    # openpilot off) sweeps too; a signal mapped within SWEEP_SIGNAL_M is what lets the light reader read during a sweep
    from openpilot.cereal import messaging
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SWEEP_SIGNAL_M, plan_inputs
    from openpilot.selfdrive.turnpilot.hooks import fresh_plan
    msg = messaging.new_message('turnpilotPlan')
    p = msg.turnpilotPlan
    p.shadow, p.state, p.detectorView, p.farCropBearing = True, 'disabled', 'stopSweep', 3.
    p.mapControl.kind, p.mapControl.distance = 'signal', SWEEP_SIGNAL_M - 1.

    class SM:
      recv_time, valid = {'turnpilotPlan': time.monotonic()}, {'turnpilotPlan': True}

      def __getitem__(self, s):
        return msg.as_reader().turnpilotPlan
    assert plan_inputs(fresh_plan(SM())) == ('stopSweep', 3., True)
    p.mapControl.distance = SWEEP_SIGNAL_M + 1.
    assert plan_inputs(fresh_plan(SM())) == ('stopSweep', 3., False)
    p.mapControl.kind, p.mapControl.distance = 'stop', 10.
    assert plan_inputs(fresh_plan(SM()))[2] is False
    assert plan_inputs(None) == ('auto', 0., False)

  def test_a_sweep_suspends_the_far_crop_lane_paint_and_unmapped_lights(self):
    # G6's far crop never runs during a sweep (the odd steps are the side views'), and says so; lane paint is off; the
    # light reader's colour read waits for a signal mapped within SWEEP_SIGNAL_M, but every forward run's class-9 boxes
    # are published
    from openpilot.selfdrive.turnpilot.detector.tests.test_far_crop import FakeFarRun
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import FarCrop
    from openpilot.selfdrive.turnpilot.scene.ranging import road_lens
    from openpilot.common.transformations.camera import DEVICE_CAMERAS
    FarStatus = __import__('openpilot.cereal', fromlist=['custom']).custom.TurnpilotDetections.FarStatus
    det, pm = with_lights(side_detector()), SweepPM()
    far = FarCrop(FakeFarRun(), 1344, 760, lambda crop: ('luma', 'chroma'))
    far.lens, far.calibration, far.warmup_s = road_lens(DEVICE_CAMERAS[('mici', 'os04c10')].narrow_road), lambda: [0., 0., 0.], 0.006
    det.far = far
    sweep_steps(det, pm, 8, v_ego=5.)  # odd steps idle at speed: still no far crop
    sweep_steps(det, pm, 8, start=100)
    assert far.run.calls == [] and pm.far == []
    assert all(m.farStatus == FarStatus.sweep for m in pm.msgs)
    road = [m for m in pm.msgs if str(m.camera) == 'road']
    assert road and det.lane_paint.calls == [] and all(m.lanePaintStatus == LanePaintStatus.sweep for m in road)
    assert det.lights.calls == [] and all(m.lightStatus == LightStatus.sweep for m in road)
    forward = [m for m in pm.msgs if str(m.view) == 'full']
    assert all([b.cls for b in m.lightBoxes] == [TRAFFIC_LIGHT] for m in forward)  # wide runs' too
    # a signal mapped within 60 m: the colour read runs on the road runs
    for k in range(4):
      det.step(pm, det.model_state, 0.010, 0., log.Desire.none, [Meta(300 + k), Meta(400 + k)], 'stopSweep', 0., 0., True)
    assert len(det.lights.calls) == 1 and det.lane_paint.calls == []
    # out of the sweep the far crop comes back
    sweep_steps(det, pm, 4, view='auto', start=500)
    assert len(far.run.calls) == 2 and pm.msgs[-1].farStatus == FarStatus.read

  def test_E7_a_side_view_without_room_is_skipped_and_said(self):
    # room() governs each side-view run on its own estimate: one that doesn't fit is skipped (its view goes stale, which
    # the decider counts against a go) and the messages say noRoom; the full pass's runs are untouched
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SIDE_STATUS_S, SideViewStatus
    det, pm = side_detector(side_time=0.012), SweepPM()
    sweep_steps(det, pm, 8, model_time=STEP_BUDGET_S - 0.008)  # room for the full pass (5 ms), not a side view (12 ms)
    assert det.side.run.calls == [] and pm.runs == [('wide', 'full'), ('road', 'full')] * 2
    det.side.started_t -= SIDE_STATUS_S + 0.01
    sweep_steps(det, pm, 2, model_time=STEP_BUDGET_S - 0.008, start=100)
    assert pm.msgs[-1].sideViewStatus == SideViewStatus.noRoom
    det.side.times['sideLeft'].extend([0.004] * TIME_WINDOW)  # its own estimate: the left fits, the right doesn't
    sweep_steps(det, pm, 4, model_time=STEP_BUDGET_S - 0.008, start=200)
    assert [c[1] for c in det.side.run.calls] == ['sideLeft']

  def test_E9_an_overrunning_side_view_is_off_for_the_drive_loudly(self):
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SIDE_BUDGET_S, SideViewStatus
    det, pm = side_detector(side_time=0.001), SweepPM()
    det.side.run.delay = SIDE_BUDGET_S * 1.3
    with mock.patch.object(modeld_detector.cloudlog, 'error') as error:
      sweep_steps(det, pm, 4 * TIME_WINDOW, model_time=0.002)
    assert det.side.disabled and len(det.side.run.calls) == TIME_WINDOW
    assert sum('side views' in c.args[0] for c in error.call_args_list) == 1  # said once
    assert pm.msgs[-1].sideViewStatus == SideViewStatus.disabled
    # both cycles fall back to the full wide pass and the road camera, the odd steps idle, for the drive
    det.side.run.delay = 0.
    n = len(pm.msgs)
    sweep_steps(det, pm, 8, start=1000)
    assert pm.runs[n:] == [('wide', 'full'), ('road', 'full')] * 2 and len(det.side.run.calls) == TIME_WINDOW
    sweep_steps(det, pm, 4, view='stopSweepGo', start=2000)
    assert pm.runs[n + 4:] == [('wide', 'full')] * 2 and all(m.sideViewStatus == SideViewStatus.disabled for m in pm.msgs[n:])
    # a failure is the same, and the full pass's own backoff never moves
    det, pm = side_detector(), SweepPM()
    det.side.run.fail = True
    sweep_steps(det, pm, 4)
    assert det.side.disabled and det.failures == 0 and det.retry_at == 0. and len(pm.msgs) == 2

  def test_E9_step_overruns_turn_the_side_views_off(self):
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SIDE_OVERRUNS
    det, pm = side_detector(side_time=0.001), SweepPM()
    det.side.run.delay = 0.007
    for k in range(4 * TIME_WINDOW):
      for t in det.side.times.values():
        t.clear()  # each run fits its (low) estimate
      sweep_steps(det, pm, 1, model_time=STEP_BUDGET_S - 0.005, start=10 * k)
    assert det.side.disabled and len(det.side.run.calls) == SIDE_OVERRUNS

  def test_E8_a_calibration_change_invalidates_the_side_views_until_their_maps_are_recomputed(self):
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SideViewStatus
    from openpilot.selfdrive.turnpilot.detector.sideview import RECALIB_DEG
    held = HeldExecutor()
    det, pm = side_detector(executor=held, ready=False), SweepPM()
    sweep_steps(det, pm, 4)  # no maps yet: computing, nothing runs
    assert det.side.run.calls == [] and len(held.pending) == 1 and pm.msgs[-1].sideViewStatus == SideViewStatus.mapsStale
    held.finish()
    sweep_steps(det, pm, 2, start=50)  # one odd step: one view's maps up (27 ms for both on the chestnut), no view run
    assert list(det.side.staged) == ['sideLeft'] and det.side.geometry is None and det.side.run.calls == []
    sweep_steps(det, pm, 8, start=100)
    # the steps after the recompute upload the maps a view at a time (and run no view), then the views run
    assert {c[1] for c in det.side.run.calls} == {'sideLeft', 'sideRight'} and pm.msgs[-1].sideViewStatus == SideViewStatus.running
    first = det.side.geometry
    # under RECALIB_DEG of movement: the same maps
    rpy = np.array(CAL_RPY) + [0., np.radians(RECALIB_DEG - 0.1), 0.]
    det.side.calibration = lambda: (list(rpy), list(CAL_EULER))
    sweep_steps(det, pm, 4, start=200)
    assert held.pending == [] and det.side.geometry is first and pm.msgs[-1].sideViewStatus == SideViewStatus.running
    # past it: invalid at once, recomputed off the step, then back on the new maps
    rpy = np.array(CAL_RPY) + [0., np.radians(RECALIB_DEG + 0.1), 0.]
    det.side.calibration = lambda: (list(rpy), list(CAL_EULER))
    n = len(det.side.run.calls)
    sweep_steps(det, pm, 4, start=300)
    assert len(det.side.run.calls) == n and len(held.pending) == 1 and pm.msgs[-1].sideViewStatus == SideViewStatus.mapsStale
    held.finish()
    sweep_steps(det, pm, 8, start=400)
    assert len(det.side.run.calls) > n and det.side.geometry is not first
    np.testing.assert_allclose(det.side.geometry.rpy, rpy)
    # no calibration: none
    det, pm = side_detector(rpy=None, ready=False), SweepPM()
    sweep_steps(det, pm, 4)
    assert det.side.run.calls == [] and pm.msgs[-1].sideViewStatus == SideViewStatus.noCalibration

  def test_E11_dark_frozen_and_saturated_sectors_are_not_ok(self):
    # sector quality per 5 deg of bearing on each side view, from the host copy of the wide frame: a black sector reads
    # dark, a stuck one frozen, a saturated one washed, and textured ones ok; within about 1 ms
    from openpilot.selfdrive.turnpilot.detector.sideview import SectorQuality
    from openpilot.selfdrive.turnpilot.detector.tests.test_sideview import geometry, textured_frame
    geom = geometry()
    q = SectorQuality()
    rng = np.random.default_rng(5)
    left = geom.maps['sideLeft'].sectors
    dark, frozen, sat = left[2], left[4], geom.maps['sideRight'].sectors[3]
    stuck = None
    times = []
    for k in range(6):  # 5 Hz a side, 1.2 s
      frame = textured_frame(geom, rng)
      frame[dark.idx] = rng.integers(16, 30, len(dark.idx))
      frame[sat.idx] = 235  # full range 255
      if stuck is None:
        stuck = frame[frozen.idx].copy()
      frame[frozen.idx] = stuck
      t0 = time.perf_counter()
      got = {name: q.update(geom, name, frame, 0.2 * k) for name in ('sideLeft', 'sideRight')}
      times.append((time.perf_counter() - t0) / 2)
      quality = {(name, s[0]): s[2] for name, ss in got.items() for s in ss}
      if k == 0:
        assert quality[('sideLeft', dark.inner)] == 'dark' and quality[('sideRight', sat.inner)] == 'washed'
        assert quality[('sideLeft', frozen.inner)] == 'ok'
    assert quality[('sideLeft', frozen.inner)] == 'frozen' and quality[('sideRight', sat.inner)] == 'washed'
    assert quality[('sideLeft', dark.inner)] in ('dark', 'blocked')
    others = [v for key, v in quality.items() if key not in (('sideLeft', dark.inner), ('sideLeft', frozen.inner), ('sideRight', sat.inner))]
    assert others and all(v == 'ok' for v in others), quality
    print(f"sector quality: {1e3 * np.median(times):.2f} ms a side (median)")
    assert np.median(times) < 0.003

  def test_E11_a_side_run_carries_its_sectors(self):
    from openpilot.selfdrive.turnpilot.detector.sideview import SectorQuality
    from openpilot.selfdrive.turnpilot.detector.tests.test_sideview import textured_frame
    det, pm = side_detector(), SweepPM()
    det.side.quality = SectorQuality()
    frame = textured_frame(det.side.geometry, np.random.default_rng(1))
    det.model_state.frames = [frame, frame]
    sweep_steps(det, pm, 4)
    left, right = pm.msgs[1], pm.msgs[3]
    assert str(left.view) == 'sideLeft' and len(left.sectorQuality) >= 6 and all(s.inner > 45. for s in left.sectorQuality)
    assert all(s.outer < -45. for s in right.sectorQuality) and all(str(s.quality) == 'ok' for s in left.sectorQuality)
    assert left.sectorTime > 0. and len(pm.msgs[0].sectorQuality) == 0  # forward runs carry none

  def test_side_views_off_without_a_side_pkl_or_a_wide_camera(self):
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import SideViewStatus
    det, pm = make_detector(), SweepPM()
    det.side = None
    sweep_steps(det, pm, 8)
    assert pm.runs == [('wide', 'full'), ('road', 'full')] * 2 and pm.msgs[-1].sideViewStatus == SideViewStatus.off
    sweep_steps(det, pm, 2, view='auto', start=100)
    assert pm.msgs[-1].sideViewStatus == SideViewStatus.off
    det, pm = side_detector(), SweepPM()
    sweep_steps(det, pm, 2, view='auto')
    assert pm.msgs[-1].sideViewStatus == SideViewStatus.idle

  def test_stop_signs_are_decoded_apart_in_shadow_with_their_plaque(self):
    # COCO 11 never reaches the detections (nothing stops, caps or releases on it); under TurnPilotCameraStopSigns shadow
    # (the default) each sign rides in signs with the luma below it for plaque labelling; off decodes none
    from openpilot.selfdrive.turnpilot.detector.decoder import STOP_SIGN
    det, pm = make_detector(), SweepPM()
    lb = det.lb
    det.run.raw[0, 2] = [*lb.to_input(600, 300), *lb.to_input(640, 340), 0.8, STOP_SIGN]
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    stride, y_height, uv_height, _ = get_nv12_info(1344, 760)
    frame = np.zeros(stride * (y_height + uv_height), np.uint8)
    frame.reshape(-1, stride)[330:390, 570:670] = 200  # a plaque below the sign
    det.model_state.frames = [frame, frame]
    det.step(pm, det.model_state, 0.010, 0., log.Desire.none, [Meta(1), Meta(2)])
    td = pm.msgs[0]
    assert [d.cls for d in td.detections] == [PERSON]
    (s,) = td.signs
    assert s.box.cls == STOP_SIGN and abs(s.box.x0 - 600) < 1 and abs(s.box.y1 - 340) < 1
    x0, y0, x1, y1 = s.plaqueRect
    assert y0 >= 339 and x0 < 600 < 640 < x1 and s.plaqueW * s.plaqueH == len(s.plaque) and min(s.plaque) == 200
    det.signs_mode = 'off'
    det.step(pm, det.model_state, 0.010, 0., log.Desire.none, [Meta(3), Meta(4)])
    det.step(pm, det.model_state, 0.010, 0., log.Desire.none, [Meta(5), Meta(6)])
    assert len(pm.msgs[-1].signs) == 0 and [d.cls for d in pm.msgs[-1].detections] == [PERSON]


class StopSignParams(LightParams):
  def get(self, key):
    assert key == "TurnPilotCameraStopSigns"
    if self.unknown:
      raise UnknownKeyName(key)
    return self.value


def test_camera_stop_signs_mode():
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import camera_stop_signs_mode
  assert camera_stop_signs_mode(StopSignParams(None)) == 'shadow'
  assert camera_stop_signs_mode(StopSignParams('off')) == 'off'
  assert camera_stop_signs_mode(StopSignParams(b'shadow')) == 'shadow'
  assert camera_stop_signs_mode(StopSignParams('auto')) == 'shadow'  # nothing acts on a stop sign seen this week
  assert camera_stop_signs_mode(StopSignParams(unknown=True)) == 'shadow'


def test_detections_are_published_at_20_hz():
  # the sweep runs a view on every model frame while it holds the car (STOP_SIGNS.md section 1.3)
  from openpilot.cereal.services import SERVICE_LIST
  assert SERVICE_LIST['turnpilotDetections'].frequency == 20.


def test_a_side_pkl_is_checked_at_load():
  # modeld loads the side pkl only if its input_specs and metadata are what it gives it: the wide frame's view and a
  # 768x512 view's maps (graph.side_input_specs, which compile_detector writes)
  from openpilot.selfdrive.turnpilot.detector.graph import side_input_specs
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import check_side_specs
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  stride, y_height, uv_height, _ = get_nv12_info(1344, 760)
  size = stride * (y_height + uv_height)
  meta = {'kind': 'side', 'camera_size': (1344, 760), 'input_size': (768, 512)}
  check_side_specs({'metadata': meta, 'input_specs': side_input_specs(1344, 760, 'AMD')}, size)
  bad = side_input_specs(1344, 760, 'AMD')
  bad['weights'] = ((2, 768 * 512), bad['weights'][1], 'AMD')
  for art, frame_size in (({'metadata': meta, 'input_specs': bad}, size),
                          ({'metadata': {**meta, 'kind': 'far'}, 'input_specs': side_input_specs(1344, 760, 'AMD')}, size),
                          ({'metadata': {**meta, 'input_size': (640, 384)}, 'input_specs': side_input_specs(1344, 760, 'AMD')}, size),
                          ({'metadata': meta, 'input_specs': side_input_specs(1344, 760, 'AMD')}, size + 128)):
    with pytest.raises(ValueError):
      check_side_specs(art, frame_size)
