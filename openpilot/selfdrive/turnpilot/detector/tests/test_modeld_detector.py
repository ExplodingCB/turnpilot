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
