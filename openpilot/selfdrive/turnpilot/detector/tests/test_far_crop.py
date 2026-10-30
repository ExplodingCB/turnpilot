"""The far crop pass in modeld (DESIGN phase 4 G5): a native-resolution window of the road frame, run on the model frames
between the full pass's runs, aimed by turnpilotPlan.farCropBearing, its own budget and a loud off for the drive."""
import time

import numpy as np

from openpilot.cereal import custom, log
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.detector.decoder import CAR, TRAFFIC_LIGHT
from openpilot.selfdrive.turnpilot.detector.graph import FAR_HORIZON_ROW, Crop
from openpilot.selfdrive.turnpilot.detector.modeld_detector import (FAR_BUDGET_S, FAR_STATUS_S, FORCE_RUN_FRAMES, RUN_EVERY,
                                                                    STEP_BUDGET_S, TIME_WINDOW, FarCrop, LightStatus)
from openpilot.selfdrive.turnpilot.detector.tests.test_modeld_detector import FakeLights, FakeOutput, Meta, make_detector
from openpilot.selfdrive.turnpilot.scene.ranging import road_lens

FarStatus = custom.TurnpilotDetections.FarStatus
ROAD = DEVICE_CAMERAS[('mici', 'os04c10')].narrow_road  # the comma four: 1344x760, f 1141.5


class FarPM:
  def __init__(self):
    self.full, self.far = [], []

  def send(self, name, msg):
    r = msg.as_reader()
    (self.full if name == 'turnpilotDetections' else self.far).append(getattr(r, name))

  @property
  def sent(self):
    return self.full


class FakeFarRun:
  """Stands in for the far jit: records the views it got, returns a car and a traffic light in crop input pixels."""
  def __init__(self):
    self.calls, self.fail, self.delay = [], False, 0.
    self.raw = np.zeros((1, 300, 6), np.float32)
    self.raw[0, 0] = [300, 250, 312, 258, 0.6, CAR]
    self.raw[0, 1] = [400, 100, 408, 124, 0.7, TRAFFIC_LIGHT]

  def __call__(self, luma, chroma):
    if self.fail:
      raise RuntimeError("gpu fell over")
    if self.delay:
      time.sleep(self.delay)
    self.calls.append((luma, chroma))
    return FakeOutput(self.raw)


def far_detector(rpy=(0., 0., 0.), far_time=0.006, det_time=0.005):
  det = make_detector(det_time=det_time)
  far = FarCrop(FakeFarRun(), 1344, 760, lambda crop: (f'luma{crop.x0},{crop.y0}', f'chroma{crop.x0},{crop.y0}'))
  far.lens = road_lens(ROAD)
  far.calibration = lambda: None if rpy is None else list(rpy)
  far.warmup_s = far_time
  det.far = far
  return det


def steps(det, pm, n, model_time=0.010, bearing=0., start=0):
  for k in range(n):
    det.step(pm, det.model_state, model_time, 0.0, log.Desire.none, [Meta(start + 2 * k), Meta(start + 2 * k + 1)], 'auto', bearing)


class TestFarCrop(OpenpilotTestCase):
  def test_runs_between_the_full_runs_on_the_road_frame(self):
    det, pm = far_detector(), FarPM()
    steps(det, pm, 3 * RUN_EVERY)
    # full, far, full, far...: the full pass's grid is unchanged, the far crop takes the frames between
    assert len(pm.full) == 3 and len(pm.far) == 3 and det.run.calls == ['frame0'] * 3
    fd = pm.far[0]
    assert str(fd.run.camera) == 'road' and str(fd.run.view) == 'far' and fd.run.frameId == 2  # its frame, the road camera's
    # aimed straight ahead at zero calibration: centred on the principal point, the horizon at FAR_HORIZON_ROW
    x0, y0, x1, y1 = fd.crop
    assert abs((x0 + x1) / 2 - 672) <= 16 and abs(y0 + FAR_HORIZON_ROW - 380) <= 1 and (x1 - x0, y1 - y0) == (640, 384)
    assert det.far.run.calls[0] == (f'luma{int(x0)},{int(y0)}', f'chroma{int(x0)},{int(y0)}')
    # its boxes in the road camera's full-frame pixels; class 9 is not a detection
    (d,) = fd.run.detections
    assert d.cls == CAR and abs(d.x0 - (x0 + 300)) < 1e-3 and abs(d.y1 - (y0 + 258)) < 1e-3
    assert all(str(td.view) == 'full' for td in pm.full) and pm.full[-1].farStatus == FarStatus.read

  def test_aim_follows_the_plans_bearing(self):
    det, pm = far_detector(), FarPM()
    steps(det, pm, 2, bearing=8.)
    x0, _, x1, _ = pm.far[0].crop
    u = 672 - 1141.5 * np.tan(np.radians(8.))  # left of centre
    assert abs((x0 + x1) / 2 - u) <= 16 and abs(pm.far[0].bearing - 8.) < 1e-6
    # a calibrated pitch moves the horizon, and the crop with it
    det, pm = far_detector(rpy=(0., np.radians(1.), 0.)), FarPM()
    steps(det, pm, 2)
    v = road_lens(ROAD).project(det.far.view_ray(0., [0., np.radians(1.), 0.]))[1]
    assert abs(pm.far[0].crop[1] + FAR_HORIZON_ROW - v) <= 1 and abs(v - 380) > 15

  def test_never_on_a_frame_the_full_pass_is_due_or_forced(self):
    # no room for the full pass (5 ms), room for the far crop (3 ms): the full pass is due on every frame and the far crop
    # never takes them, up to the full pass's forced run; on the frame after it, the far crop runs
    det, pm = far_detector(far_time=0.003), FarPM()
    steps(det, pm, FORCE_RUN_FRAMES - RUN_EVERY - 1, model_time=STEP_BUDGET_S - 0.004)
    assert pm.full == [] and pm.far == [] and det.since_run == FORCE_RUN_FRAMES - 1
    steps(det, pm, 2, model_time=STEP_BUDGET_S - 0.004, start=100)
    assert len(pm.full) == 1 and len(pm.far) == 1 and pm.far[0].run.frameId == 102
    det, pm = far_detector(far_time=0.003), FarPM()
    det.since_run = 0  # the full pass just ran
    det.step(pm, det.model_state, STEP_BUDGET_S - 0.004, 0.0, log.Desire.none, [Meta(1), Meta(2)], 'auto', 0.)
    assert len(pm.far) == 1 and pm.full == []  # room for the far crop alone, on the frame between

  def test_no_room_and_no_calibration_are_said(self):
    det, pm = far_detector(far_time=0.012), FarPM()
    steps(det, pm, 4, model_time=STEP_BUDGET_S - 0.008)  # room for the full pass (5 ms), not the far crop (12 ms)
    assert pm.far == [] and len(pm.full) == 2 and pm.full[-1].farStatus == FarStatus.noRoom
    det, pm = far_detector(rpy=None), FarPM()
    steps(det, pm, 4)
    assert pm.far == [] and pm.full[-1].farStatus == FarStatus.noCalibration and det.far.run.calls == []

  def test_read_ages_out(self):
    det, pm = far_detector(), FarPM()
    steps(det, pm, 2)
    assert det.far_status() == FarStatus.read
    det.far.last_run_t -= FAR_STATUS_S + 0.01
    det.far.why = FarStatus.noRoom
    assert det.far_status() == FarStatus.noRoom

  def test_off_without_a_far_pkl_or_with_the_param_off(self):
    det, pm = make_detector(), FarPM()
    det.far = None
    steps(det, pm, 4)
    assert pm.far == [] and pm.full[-1].farStatus == FarStatus.off
    det, pm = far_detector(), FarPM()
    det.far_on = False
    steps(det, pm, 4)
    assert pm.far == [] and det.far.run.calls == [] and pm.full[-1].farStatus == FarStatus.off

  def test_failure_turns_only_the_far_crop_off_for_the_drive(self):
    det, pm = far_detector(), FarPM()
    det.far.run.fail = True
    steps(det, pm, 2 * RUN_EVERY)
    assert det.far.disabled and pm.far == [] and len(pm.full) == 2
    assert det.failures == 0 and det.retry_at == 0.  # the full pass's backoff is its own
    assert pm.full[-1].farStatus == FarStatus.disabled
    det.far.run.fail = False
    steps(det, pm, 2 * RUN_EVERY, start=100)
    assert pm.far == [] and pm.full[-1].farStatus == FarStatus.disabled  # for the drive, never back

  def test_slow_far_crop_turns_off_and_the_full_pass_keeps_going(self):
    det, pm = far_detector(far_time=0.001), FarPM()
    det.far.run.delay = FAR_BUDGET_S * 1.3
    steps(det, pm, 2 * TIME_WINDOW + 4, model_time=0.002)
    assert det.far.disabled and len(pm.far) == TIME_WINDOW and all(fd.run.executionTime > FAR_BUDGET_S for fd in pm.far)
    assert len(pm.full) == TIME_WINDOW + 2 and pm.full[-1].farStatus == FarStatus.disabled

  def test_step_overruns_turn_it_off(self):
    # runs that fit their estimate but, with the model's time, overrun the step's budget (the estimate low, the model slow):
    # FAR_OVERRUNS of the last TIME_WINDOW and it is off for the drive
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import FAR_OVERRUNS
    det, pm = far_detector(far_time=0.001), FarPM()
    det.far.run.delay = 0.007
    for k in range(2 * TIME_WINDOW):
      det.far.times.clear()  # each run fits the (low) estimate
      steps(det, pm, 1, model_time=STEP_BUDGET_S - 0.005, start=10 * k)
    assert det.far.disabled and len(pm.far) == FAR_OVERRUNS and pm.full[-1].farStatus == FarStatus.disabled
    det, pm = far_detector(far_time=0.001), FarPM()
    det.far.run.delay = 0.007
    steps(det, pm, 2 * TIME_WINDOW, model_time=0.010)
    assert not det.far.disabled and len(pm.far) == TIME_WINDOW

  def test_far_lights_have_a_reader_of_their_own_always_in_shadow(self):
    det, pm = far_detector(), FarPM()
    det.light_mode, det.lights = 'addOnly', FakeLights()
    det.far.lights = FakeLights()
    steps(det, pm, 2)
    # the far crop's class-9 boxes, in frame pixels, on the road camera's host frame, to its own reader
    assert det.lights.calls == [('host0', 0, [TRAFFIC_LIGHT])] and det.far.lights.calls == [('host0', 2, [TRAFFIC_LIGHT])]
    fl = pm.far[0].run
    assert fl.lightStatus == LightStatus.read and str(fl.lights.source) == 'camera' and fl.lights.shadow  # addOnly or not
    assert not pm.full[0].lights.shadow
    det.light_mode = 'off'
    steps(det, pm, 2, start=10)
    assert pm.far[-1].run.lightStatus == LightStatus.off and len(det.far.lights.calls) == 1

  def test_far_light_failure_turns_off_only_its_lights(self):
    det, pm = far_detector(), FarPM()
    det.light_mode, det.far.lights = 'shadow', FakeLights()
    det.far.lights.fail = True
    steps(det, pm, 2 * RUN_EVERY)
    assert not det.far.disabled and len(pm.far) == 2 and all(fd.run.lightStatus == LightStatus.disabled for fd in pm.far)
    assert all(len(fd.run.detections) == 1 for fd in pm.far)


def test_crop_views_of_modelds_frame_buffer():
  # the far crop's two views are byte windows of the road camera's frame in modeld's packed input: its luma rows and chroma
  # rows at the crop (graph.crop_views), past the npy inputs and the frame before it
  from openpilot.selfdrive.turnpilot.detector.graph import crop_views
  from openpilot.selfdrive.turnpilot.detector.modeld_detector import crop_view_offsets
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  stride, y_height, uv_height, _ = get_nv12_info(1344, 760)
  size = stride * (y_height + uv_height)
  packed = 1000 + 2 * size
  crop = Crop(1344, 760, 352, 124)
  (lo, ln), (co, cn) = crop_views(crop)
  assert crop_view_offsets(packed, size, 0, crop) == ((1000 + lo, ln), (1000 + co, cn))
  assert crop_view_offsets(packed, size, 1, crop) == ((1000 + size + lo, ln), (1000 + size + co, cn))
