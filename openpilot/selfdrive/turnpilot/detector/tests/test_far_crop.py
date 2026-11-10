"""The far crop pass in modeld (DESIGN phase 4 G5): a native-resolution window of the road frame, run on the model frames
between the full pass's runs, aimed by turnpilotPlan.farCropBearing, its own budget and a loud off for the drive."""
import time

import numpy as np

from openpilot.cereal import custom, log
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.detector.decoder import CAR, TRAFFIC_LIGHT
from openpilot.selfdrive.turnpilot.detector.graph import FAR_HORIZON_ROW, Crop
from openpilot.selfdrive.turnpilot.detector.modeld_detector import (DEBT_CAP_S, FAR_BUDGET_S, FAR_STATUS_S, FORCE_RUN_FRAMES,
                                                                    NATIVE_NO_RUN_S, PAINT_BUDGET_S, RETRY_S, RUN_EVERY, STEP_BUDGET_S,
                                                                    STEP_GAP_S, TIME_WINDOW, FarCrop, LanePaintStatus, LightStatus,
                                                                    NativeStatus, NativeStrips)
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
    # no room for the full pass (20 ms beside a model 1 ms under the budget: a 19 ms debt, past FORCE_SKIP_LATE_S), room for
    # the far crop (3 ms: 2): the full pass is due on every frame and the far crop never takes them, up to the full pass's
    # forced run; on the frame after it, the far crop runs
    det, pm = far_detector(far_time=0.003, det_time=0.020), FarPM()
    steps(det, pm, FORCE_RUN_FRAMES - RUN_EVERY - 1, model_time=STEP_BUDGET_S - 0.001)
    assert pm.full == [] and pm.far == [] and det.since_run == FORCE_RUN_FRAMES - 1
    steps(det, pm, 2, model_time=STEP_BUDGET_S - 0.001, start=100)
    assert len(pm.full) == 1 and len(pm.far) == 1 and pm.far[0].run.frameId == 102
    det, pm = far_detector(far_time=0.003, det_time=0.010), FarPM()
    det.since_run = 0  # the full pass just ran
    det.step(pm, det.model_state, STEP_BUDGET_S - 0.001, 0.0, log.Desire.none, [Meta(1), Meta(2)], 'auto', 0.)
    assert len(pm.far) == 1 and pm.full == []  # room for the far crop alone, on the frame between

  def test_no_room_and_no_calibration_are_said(self):
    det, pm = far_detector(far_time=0.012), FarPM()
    steps(det, pm, 4, model_time=STEP_BUDGET_S)  # room for the full pass (a 5 ms debt), not the far crop (12 ms, and 5 to come)
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

  def test_a_failure_is_retried_and_never_turns_the_far_crop_off_for_the_drive(self):
    # 2026-11-07 (no features off): a failed run is retried after a backoff, logged, the full pass's own backoff untouched;
    # FAR_DISABLED_FAILS in a row say farStatus disabled (turnpilotd's farViewOff) until a run succeeds, which reads again.
    # Until then one failure was the far crop off for the drive
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import FAR_DISABLED_FAILS
    det, pm = far_detector(), FarPM()
    det.far.run.fail = True
    steps(det, pm, 2 * RUN_EVERY)
    assert not det.far.disabled and det.far.failures == 1 and pm.far == [] and len(pm.full) == 2
    assert det.failures == 0 and det.retry_at == 0.  # the full pass's backoff is its own
    assert RETRY_S - 0.5 < det.far.retry_at - time.monotonic() <= RETRY_S
    assert pm.full[-1].farStatus == FarStatus.noRoom  # one failure, waiting out its retry: not yet said off
    for k in range(FAR_DISABLED_FAILS - 1):
      det.far.retry_at = 0.
      steps(det, pm, RUN_EVERY, start=100 + 10 * k)
    steps(det, pm, RUN_EVERY, start=150)  # the next full run's message, the retry not yet due
    assert det.far.failures == FAR_DISABLED_FAILS and pm.full[-1].farStatus == FarStatus.disabled and pm.far == []
    det.far.run.fail, det.far.retry_at = False, 0.
    steps(det, pm, RUN_EVERY + 1, start=200)
    assert len(pm.far) == 1 and det.far.failures == 0 and pm.full[-1].farStatus == FarStatus.read  # back, never off

  def test_a_slow_far_crop_is_warned_and_runs_where_it_fits(self):
    # a median run over FAR_BUDGET_S: warned once, never off for the drive (11-06: the far crop's rules turned it off in
    # minutes on both drives); its runs go where its estimate, its measured step cost, fits
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    det, pm = far_detector(far_time=0.001), FarPM()
    det.far.run.delay = FAR_BUDGET_S * 1.3
    with mock.patch.object(modeld_detector.cloudlog, 'warning') as warning:
      steps(det, pm, 2 * TIME_WINDOW + 4, model_time=0.002)
    assert not det.far.disabled and len(pm.far) == TIME_WINDOW + 2 and all(fd.run.executionTime > FAR_BUDGET_S for fd in pm.far)
    assert len(pm.full) == TIME_WINDOW + 2 and pm.full[-1].farStatus == FarStatus.read
    said = [c.args[0] for c in warning.call_args_list if 'over budget' in c.args[0]]
    assert len(said) == 1 and said[0].startswith("turnpilot far crop over budget")
    assert det.far.estimate(False) >= FAR_BUDGET_S * 1.3  # its step's cost, the run's time in it

  def test_step_overruns_are_warned_never_off(self):
    # runs that fit their estimate but, with the model's time, push the debt past DEBT_CAP_S (the estimate low, the model
    # slow): FAR_OVERRUNS of the last TIME_WINDOW are warned, never the far crop off for the drive (11-06: three of five did
    # it 79.5 s into 1d's log and 190.6 s into 1e's); each run's step cost is its estimate after it, so it fits only where it does
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    det, pm = far_detector(far_time=0.001, det_time=0.001), FarPM()
    det.far.run.delay = 0.009
    with mock.patch.object(modeld_detector.cloudlog, 'warning') as warning:
      for k in range(2 * TIME_WINDOW):
        det.far.costs.clear()  # each run fits the (low) warm-up estimate from a paid-back cycle, but beside a model 1 ms over
        det.debt = 0.          # the budget it leaves 10 ms of debt
        steps(det, pm, 1, model_time=STEP_BUDGET_S + 0.001, start=10 * k)
    assert not det.far.disabled and len(pm.far) == TIME_WINDOW and all(det.far.overruns)
    assert pm.full[-1].farStatus == FarStatus.read
    assert len([c for c in warning.call_args_list if 'pushed the debt' in c.args[0]]) == 1  # once in BUDGET_WARN_S
    # its measured cost kept: beside that model it has no room again (noRoom), never off
    det.debt = 0.
    steps(det, pm, 2, model_time=STEP_BUDGET_S + 0.001, start=500)
    assert len(pm.far) == TIME_WINDOW and det.far.why == FarStatus.noRoom and not det.far.disabled

  def test_a_far_run_is_judged_on_its_steps_cost(self):
    # 11-06: the room check took the far run's own 10.5 ms, the step around it (decode, the message, the send) cost 2.4 ms
    # more, and runs it fit left 6.5-7.1 ms of debt. Its estimate is now the step's: before a run the warm-up's plus
    # FAR_STEP_OVERHEAD_S, after, the measured costs' median (its lights apart), its executionTime still the run's alone
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import FAR_STEP_OVERHEAD_S
    det = far_detector(far_time=0.0105)
    assert abs(det.far.estimate(False) - (0.0105 + FAR_STEP_OVERHEAD_S)) < 1e-9
    det.far.ran(time.monotonic(), 0.0105, 0.0129, False)
    det.far.light_times.append(0.0007)
    det.far.lights = FakeLights()
    assert abs(det.far.estimate(False) - 0.0129) < 1e-9 and abs(det.far.estimate(True) - 0.0136) < 1e-9
    assert list(det.far.times) == [0.0105]

  def test_a_far_crop_without_room_measures_again(self):
    # a slow burst's median can't keep it from ever running: FAR_FORCE_S wanting a frame with none and its measured costs
    # are dropped, the estimate back to the warm-up's, so a frame with room for that measures it again
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import FAR_FORCE_S, FAR_STEP_OVERHEAD_S
    det = far_detector(far_time=0.010)
    det.far.costs.extend([0.030] * TIME_WINDOW)  # the burst
    t = time.monotonic()
    det.far.no_room(t)
    det.far.no_room(t + FAR_FORCE_S - 0.1)
    assert len(det.far.costs) == TIME_WINDOW
    det.far.no_room(t + FAR_FORCE_S)
    assert not det.far.costs and abs(det.far.estimate(False) - (0.010 + FAR_STEP_OVERHEAD_S)) < 1e-9

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

  def test_far_light_failure_is_retried_and_touches_only_its_lights(self):
    # its lights' rule is the full pass's lights' (never off for the drive): a failure is logged and retried after a backoff,
    # its runs inside it saying retrying, and the far crop itself runs on
    det, pm = far_detector(), FarPM()
    det.light_mode, det.far.lights = 'shadow', FakeLights()
    det.far.lights.fail = True
    steps(det, pm, 2 * RUN_EVERY)
    assert not det.far.disabled and len(pm.far) == 2 and all(fd.run.lightStatus == LightStatus.retrying for fd in pm.far)
    assert all(len(fd.run.detections) == 1 for fd in pm.far) and det.far.light_failures == 1 and det.far.lights is not None
    assert RETRY_S - 0.5 < det.far.light_retry_at - time.monotonic() <= RETRY_S
    det.far.lights.fail, det.far.light_retry_at = False, 0.
    steps(det, pm, RUN_EVERY, start=100)
    assert pm.far[-1].run.lightStatus == LightStatus.read and det.far.light_failures == 0
    (light,) = pm.far[-1].run.lights.lights
    assert str(light.read) == 'red' and light.reason == 'read'  # its heads carry this frame's read too
    # LIGHT_DISABLED_FAILS in a row: disabled until a read succeeds, as the full pass's lights (light_failed)
    from openpilot.selfdrive.turnpilot.detector.modeld_detector import LIGHT_DISABLED_FAILS
    det.far.lights.fail = True
    for k in range(LIGHT_DISABLED_FAILS):
      det.far.light_retry_at = 0.
      steps(det, pm, RUN_EVERY, start=200 + 10 * k)
      want = LightStatus.disabled if k == LIGHT_DISABLED_FAILS - 1 else LightStatus.retrying
      assert pm.far[-1].run.lightStatus == want and not det.far.disabled
    steps(det, pm, RUN_EVERY, start=300)
    assert pm.far[-1].run.lightStatus == LightStatus.disabled  # inside its backoff
    det.far.lights.fail, det.far.light_retry_at = False, 0.
    steps(det, pm, RUN_EVERY, start=400)
    assert pm.far[-1].run.lightStatus == LightStatus.read and det.far.light_failures == 0

  def test_slow_far_lights_stay_on(self):
    det, pm = far_detector(), FarPM()
    det.light_mode, det.far.lights = 'shadow', FakeLights()
    det.far.light_times.extend([0.006] * TIME_WINDOW)  # over LIGHT_BUDGET_S: warned, never off
    steps(det, pm, 2 * RUN_EVERY)
    assert det.far.lights is not None and all(fd.run.lightStatus == LightStatus.read for fd in pm.far)
    assert det.far.light_warn_t is not None


class TestOddPaint(OpenpilotTestCase):
  """PF-03: while the full pass looks through the wide camera, the lane paint is read on the far crop's road frame."""
  def turn(self, det, pm, n, start=0, model_time=0.010):
    # turning: the curvature of a turn (a turn desire alone, before the car turns, alternates the cameras: choose_camera)
    for k in range(n):
      det.step(pm, det.model_state, model_time, 0.05, log.Desire.turnLeft, [Meta(start + 2 * k), Meta(start + 2 * k + 1)], 'auto', 0.)

  def test_a_turn_reads_the_paint_on_the_far_crops_frame(self):
    det, pm = far_detector(), FarPM()
    self.turn(det, pm, 3 * RUN_EVERY)
    assert [str(td.camera) for td in pm.full] == ['wide'] * 3 and len(pm.far) == 3
    # each far run's road frame, behind the far crop's one box (its car), in frame pixels
    assert det.lane_paint.calls == [('host0', 2, 1), ('host0', 6, 1), ('host0', 10, 1)]
    # the first wide run had no read before it; each later one carries the read of the step before
    assert pm.full[0].lanePaintStatus == LanePaintStatus.wideCamera and len(pm.full[0].lanePaint) == 0
    for td in pm.full[1:]:
      assert td.lanePaintStatus == LanePaintStatus.read and len(td.lanePaint) == 4 and str(td.lanePaint[2].color) == 'yellow'
      assert td.lanePaintTime >= 0.

  def test_straight_on_the_road_camera_the_far_crop_reads_no_paint(self):
    det, pm = far_detector(), FarPM()
    steps(det, pm, 2 * RUN_EVERY)
    assert det.lane_paint.calls == [('host0', 0, 1), ('host0', 4, 1)]  # the full runs' own, as before
    assert all(str(td.camera) == 'road' and td.lanePaintStatus == LanePaintStatus.read for td in pm.full)

  def test_no_room_no_read_and_a_stale_read_is_not_sent(self):
    det = far_detector()
    det.last_cam = 'wide'
    det.paint_times.extend([0.004] * 3)
    det.step_t0 = time.perf_counter()  # the far crop's step, its own time so far nothing
    # the read (4 ms) beside a model 2 ms under the budget leaves 2 ms of debt, and the next full pass (5 ms, 4 of paint
    # with the camera unknown) 7 more: past DEBT_CAP_S. 4 ms under, 0 + 5
    det.read_odd_paint(det.model_state, [Meta(2), Meta(3)], np.zeros((0, 6), np.float32), STEP_BUDGET_S - 0.002)
    assert det.lane_paint.calls == [] and det.odd_paint is None
    det.read_odd_paint(det.model_state, [Meta(2), Meta(3)], np.zeros((0, 6), np.float32), STEP_BUDGET_S - 0.004)
    assert det.lane_paint.calls == [('host0', 2, 0)] and det.odd_paint is not None
    # a read older than the step before (the full run it was for had no room) is dropped, not sent
    det, pm = far_detector(), FarPM()
    self.turn(det, pm, 2)
    det.steps += 1
    self.turn(det, pm, 1, start=20)
    assert len(pm.full) == 2 and pm.full[1].lanePaintStatus == LanePaintStatus.wideCamera and len(pm.full[1].lanePaint) == 0

  def test_the_first_read_is_checked_against_the_paint_budget(self):
    # no paint time yet (the drive started in a turn): the room check takes PAINT_BUDGET_S, the most a read may take
    det = far_detector()
    det.last_cam = 'wide'
    assert len(det.paint_times) == 0
    det.step_t0 = time.perf_counter()
    # the read's debt (model + PAINT_BUDGET_S past the budget) and the next full pass's (model + 5 ms) fit DEBT_CAP_S at
    # this model time, and no later
    edge = STEP_BUDGET_S + (DEBT_CAP_S - PAINT_BUDGET_S - 0.005) / 2
    det.read_odd_paint(det.model_state, [Meta(2), Meta(3)], np.zeros((0, 6), np.float32), edge + 0.001)
    assert det.lane_paint.calls == [] and det.odd_paint is None
    det.read_odd_paint(det.model_state, [Meta(2), Meta(3)], np.zeros((0, 6), np.float32), edge - 0.001)
    assert det.lane_paint.calls == [('host0', 2, 0)] and det.odd_paint is not None and len(det.paint_times) == 1

  def test_paint_failure_on_the_far_frame_is_the_paints_retry(self):
    # the paint's own rule (read_lane_paint's): retried after a backoff, never off for the drive; the far crop runs on
    det, pm = far_detector(), FarPM()
    det.lane_paint.fail = True
    self.turn(det, pm, 3 * RUN_EVERY)
    assert det.lane_paint is not None and det.paint_failures == 1 and not det.far.disabled and len(pm.far) == 3
    assert len(pm.full) == 3 and all(td.lanePaintStatus == LanePaintStatus.wideCamera and len(td.lanePaint) == 0 for td in pm.full)
    det.lane_paint.fail, det.paint_retry_at = False, 0.
    self.turn(det, pm, 2 * RUN_EVERY, start=100)
    assert pm.full[-1].lanePaintStatus == LanePaintStatus.read and det.paint_failures == 0


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


def native_slot(det, mode='full', native_time=0.015):
  """det with the native schedule of mode loaded and on in the far crop's slot (test_native_detect's, without its import)."""
  native = NativeStrips(FakeFarRun(), 1344, 760, lambda crop: (f'luma{crop.x0},{crop.y0}', f'chroma{crop.x0},{crop.y0}'), None, mode)
  native.lens, native.calibration, native.warmup_s = road_lens(ROAD), lambda: [0., 0., 0.], native_time
  det.native, det.native_mode = native, mode
  return native


NO_ROOM_NATIVE_S = 0.030  # a native estimate that never fits beside view_steps' model (DEBT_CAP_S)


class TestTruthfulFarView(OpenpilotTestCase):
  """11-06, both drives: the native schedule never ran beside the car's 38.5 ms model and the far crop went off for the drive
  in its first minutes (79.5 s into 1d's log, 190.6 s into 1e's), yet every message said nativeStatus full and farStatus noRoom, so turnpilotd
  took the far view for alive (FAR_ON) and never said farViewOff."""

  def test_a_dead_slot_says_disabled(self):
    det = far_detector()
    native = native_slot(det)
    det.far.disabled = True
    native.why = FarStatus.noRoom
    assert det.far_status() == FarStatus.noRoom  # just loaded: on, waiting for room
    native.load_t -= NATIVE_NO_RUN_S + 1.
    assert det.far_status() == FarStatus.disabled  # far crop off for the drive, native never run: the slot is dead
    det.far = None  # no far crop at all: the same
    assert det.far_status() == FarStatus.disabled

  def test_a_native_that_ran_lately_is_on(self):
    det = far_detector()
    native = native_slot(det)
    det.far.disabled = True
    native.load_t -= 10 * NATIVE_NO_RUN_S
    native.last_run_t, native.why = time.monotonic() - 0.6, FarStatus.noRoom
    assert det.far_status() == FarStatus.noRoom  # ran 0.6 s ago: on, no room just now (never flaps to disabled)
    native.last_run_t = time.monotonic() - 0.1
    assert det.far_status() == FarStatus.read
    native.last_run_t = time.monotonic() - NATIVE_NO_RUN_S - 1.
    assert det.far_status() == FarStatus.disabled  # nothing in the slot for NATIVE_NO_RUN_S
    # with the far crop alive the slot reads through it, as before
    det.far.disabled = False
    det.far.last_run_t = time.monotonic()
    assert det.far_status() == FarStatus.read

  def test_a_native_schedule_that_never_ran_says_unavailable_once(self):
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    det = far_detector()
    native = native_slot(det)
    assert det.native_status() == NativeStatus.full
    native.load_t -= NATIVE_NO_RUN_S + 1.
    det.model_time = 0.0385
    with mock.patch.object(modeld_detector.cloudlog, 'error') as error:
      assert det.native_status() == NativeStatus.unavailable and det.native_status() == NativeStatus.unavailable
    assert [c.args[0] for c in error.call_args_list] == [
      "turnpilot native schedule: no room in 30 s of its free, calibrated slot beside a 38 ms model"]
    native.last_run_t = time.monotonic() - 100.  # it ran once: on, as before
    assert det.native_status() == NativeStatus.full

  def test_on_the_cars_budget_every_message_says_the_slot_is_dead(self):
    # the car's frames: the strips (15 ms) never fit beside a model at the budget, the far crop off for the drive. Every
    # full pass message says nativeStatus unavailable and farStatus disabled (which turnpilotd.watch_far raises as farViewOff)
    det, pm = far_detector(), FarPM()
    native = native_slot(det, native_time=0.015)
    det.far.disabled = True
    native.load_t -= NATIVE_NO_RUN_S + 1.
    steps(det, pm, 4 * RUN_EVERY, model_time=STEP_BUDGET_S)
    assert native.run.calls == [] and pm.far == [] and len(pm.full) == 4
    assert all(td.farStatus == FarStatus.disabled and td.nativeStatus == NativeStatus.unavailable for td in pm.full)

  @staticmethod
  def view_steps(det, pm, n, view, start, model_time=STEP_BUDGET_S - 0.010):
    """n steps under turnpilotPlan.detectorView view, standing (the side views' gate open), the model leaving the full pass
    room (10 ms of slack: the debt never builds with the host's own step time, whatever its load) and none for a native of
    NO_ROOM_NATIVE_S."""
    for k in range(n):
      det.step(pm, det.model_state, model_time, 0.0, log.Desire.none, [Meta(start + 2 * k), Meta(start + 2 * k + 1)], view, 0.,
               v_ego=0.)

  @staticmethod
  def far_view_offs(statuses) -> int:
    """turnpilotd.watch_far's rule: farViewOff (a chime and an error event) on every change into disabled."""
    offs, prev = 0, None
    for s in statuses:
      offs += s == FarStatus.disabled and prev != FarStatus.disabled
      prev = s
    return offs

  def test_a_dead_slot_stays_disabled_through_a_stop_sweep(self):
    # 11-06 review: on the car's dead slot (far crop off for the drive, native never run) a stop sweep turned farStatus
    # disabled into sweep, which turnpilotd takes for a live far view (FAR_ON), and back: farViewOff chimed after every hold
    det, pm = far_detector(), FarPM()
    native = native_slot(det, native_time=NO_ROOM_NATIVE_S)
    det.far.disabled = True
    native.load_t -= NATIVE_NO_RUN_S + 1.
    assert det.far_status() == FarStatus.disabled
    det.sweep = 'stopSweep'
    assert det.far_status() == FarStatus.disabled  # as the far crop's own disabled outlasts a sweep (far_status)
    det.sweep = None
    for k, view in enumerate(('auto', 'stopSweep', 'stopSweepGo', 'auto', 'stopSweep', 'auto')):
      self.view_steps(det, pm, 2 * RUN_EVERY, view, start=4 * RUN_EVERY * k)
    assert native.run.calls == [] and len(pm.full) == 12
    assert [td.farStatus for td in pm.full] == [FarStatus.disabled] * 12
    assert all(td.nativeStatus == NativeStatus.unavailable for td in pm.full)
    assert self.far_view_offs([td.farStatus for td in pm.full]) == 1  # once a drive

  def test_a_stop_sweep_is_off_the_native_no_run_clock(self):
    # a native that ran 5 s before a 35 s hold at a stop sign: the sweep's time was the side views', never the native's to
    # miss, so it is on (not disabled, not unavailable) after the sweep, and the clock goes on from 5 s, not from zero
    det, pm = far_detector(), FarPM()
    native = native_slot(det, native_time=NO_ROOM_NATIVE_S)
    det.far.disabled = True
    native.load_t -= 100.
    native.last_run_t, native.why = time.monotonic() - 5., FarStatus.noRoom
    self.view_steps(det, pm, RUN_EVERY, 'stopSweep', start=0)
    assert det.hold_t0 is not None and det.native_holds == {'sweep'}
    det.hold_t0 -= 35.  # the hold has lasted 35 s, the native's run 5 s before it
    native.last_run_t -= 35.
    assert abs(det.native_idle_s(native, time.monotonic()) - 5.) < 0.5
    assert det.far_status() == FarStatus.sweep  # 40 s since its run, 35 of them the sweep's: no farViewOff
    self.view_steps(det, pm, 2 * RUN_EVERY, 'auto', start=4 * RUN_EVERY)
    assert det.hold_t0 is None and not det.native_holds and abs(native.held_s - 35.) < 0.5 and native.run.calls == []
    assert pm.full[-1].farStatus == FarStatus.noRoom and pm.full[-1].nativeStatus == NativeStatus.full
    assert self.far_view_offs([td.farStatus for td in pm.full]) == 0
    native.last_run_t -= NATIVE_NO_RUN_S - 4.  # 26 s more with its slot free and no run: 31 s on its clock
    assert det.far_status() == FarStatus.disabled
    native.ran(time.monotonic(), 0.010, 0.012, False)  # a run starts the clock over, the sweep's time with it
    assert native.held_s == 0. and det.far_status() == FarStatus.read

  def test_a_slot_that_dies_in_a_sweep_says_so_after_it(self):
    # never run, loaded 29 s before a 20 s sweep: the clock stands at 29 s through it (sweep, unavailable not yet said), and
    # the slot is said dead on its free time after, once
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    det, pm = far_detector(), FarPM()
    native = native_slot(det, native_time=NO_ROOM_NATIVE_S)
    det.far.disabled = True
    native.load_t -= NATIVE_NO_RUN_S - 1.
    self.view_steps(det, pm, 2 * RUN_EVERY, 'stopSweep', start=0)
    det.hold_t0 -= 20.
    native.load_t -= 20.
    with mock.patch.object(modeld_detector.cloudlog, 'error') as error:
      self.view_steps(det, pm, 2 * RUN_EVERY, 'stopSweep', start=4 * RUN_EVERY)
      assert error.call_args_list == []
    assert all(td.farStatus == FarStatus.sweep and td.nativeStatus == NativeStatus.full for td in pm.full)
    self.view_steps(det, pm, 2 * RUN_EVERY, 'auto', start=8 * RUN_EVERY)
    assert pm.full[-1].farStatus == FarStatus.noRoom and pm.full[-1].nativeStatus == NativeStatus.full
    native.load_t -= 2.  # 2 s of free slot later: 31 s
    self.view_steps(det, pm, 2 * RUN_EVERY, 'stopSweep', start=12 * RUN_EVERY)
    assert pm.full[-1].farStatus == FarStatus.disabled and pm.full[-1].nativeStatus == NativeStatus.unavailable
    assert self.far_view_offs([td.farStatus for td in pm.full]) == 1

  def test_the_no_run_clock_counts_only_the_slots_free_calibrated_time(self):
    # 11-06 review: the clock ran from the load whatever kept the native from running, so a fresh install (minutes with no
    # calibration) said nativeStatus unavailable 30 s after the load and logged 'no room', blaming the budget for the
    # calibration. On the simulated clock at 20 Hz, the car's dead slot (far crop off for the drive, no room for the native):
    # 40 s with no calibration, 20 s with TurnPilotFarCrop off and a 60 s gap in the steps count nothing; the slot's free,
    # calibrated time does (and STEP_GAP_S of the gap's step: native_gap), and its 30 s say unavailable and disabled, logged
    # once, as no room
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    from openpilot.selfdrive.turnpilot.detector.tests.test_modeld_detector import sim_clock
    with sim_clock() as clock, mock.patch.object(modeld_detector.cloudlog, 'error') as error:
      det, pm = far_detector(), FarPM()
      native = native_slot(det, native_time=NO_ROOM_NATIVE_S)
      det.far.disabled = True
      calibrated = {'rpy': None}
      native.calibration = det.far.calibration = lambda: calibrated['rpy']
      k = 0

      def drive(seconds):
        nonlocal k
        out = []
        for _ in range(int(round(seconds / 0.05))):
          clock.tick()
          n = len(pm.full)
          self.view_steps(det, pm, 1, 'auto', start=2 * k)
          k += 1
          if len(pm.full) > n:
            out.append((clock.t, pm.full[-1].nativeStatus, pm.full[-1].farStatus))
        return out
      start = clock.t
      uncal = drive(40.)
      # (the first message comes before its first look: noRoom, its initial why)
      assert native.load_t == start and all(ns == NativeStatus.full and fs == FarStatus.noCalibration for _, ns, fs in uncal[1:])
      assert det.native_holds == {'noCalibration'}
      calibrated['rpy'] = [0., 0., 0.]
      det.far_on = False
      off = drive(20.)
      assert all(ns == NativeStatus.full and fs == FarStatus.off for _, ns, fs in off)
      det.far_on = True
      free = drive(10.)  # calibrated, the slot free: its clock runs (its first message before its first look, as above)
      assert not det.native_holds and all(ns == NativeStatus.full and fs == FarStatus.noRoom for _, ns, fs in free[1:])
      clock.tick(60.)  # nothing stepped the detector (TurnPilot off a while): the native missed nothing past STEP_GAP_S
      after = drive(25.)
      assert native.run.calls == []
      dead = [t for t, ns, fs in after if ns == NativeStatus.unavailable]
      assert dead and all(ns == NativeStatus.unavailable and fs == FarStatus.disabled for t, ns, fs in after if t >= dead[0])
      assert all(ns == NativeStatus.full and fs == FarStatus.noRoom for t, ns, fs in after if t < dead[0])
      # 30 s of the free slot's time: 10 s before the gap, STEP_GAP_S of it and the rest after it, two frames late for the
      # calibration's (the gap off the clock whole would say it STEP_GAP_S later; on it whole, at once)
      after_s = NATIVE_NO_RUN_S - 10. - STEP_GAP_S
      assert after_s - 1e-6 <= dead[0] - after[0][0] <= after_s + 0.15
      assert [c.args[0] for c in error.call_args_list] == [
        "turnpilot native schedule: no room in 30 s of its free, calibrated slot beside a 37 ms model"]

  def test_a_host_stepping_below_10_hz_still_says_a_dead_slot(self):
    # 11-06 review: native_gap took every step more than STEP_GAP_S after the one before off the clock whole, so a host
    # stepping steadily below 10 Hz (a loaded one) never ran it: 0.125 s steps for 120 s on the car's dead slot said
    # nativeStatus full and farStatus noRoom to the end, held_s 119.875 s, though every other step offered the native the
    # slot. Only a gap's time past STEP_GAP_S is off now, so such a host's clock runs STEP_GAP_S a step and the dead slot is
    # said after NATIVE_NO_RUN_S / STEP_GAP_S steps (8 Hz 37.5 s, 5 Hz 60 s), on the simulated clock, logged once
    from unittest import mock
    from openpilot.selfdrive.turnpilot.detector import modeld_detector
    from openpilot.selfdrive.turnpilot.detector.tests.test_modeld_detector import sim_clock
    for dt in (0.125, 0.2):
      with sim_clock() as clock, mock.patch.object(modeld_detector.cloudlog, 'error') as error:
        det, pm = far_detector(), FarPM()
        native = native_slot(det, native_time=NO_ROOM_NATIVE_S)
        det.far.disabled = True
        start, out = clock.t, []
        for k in range(int(round(90. / dt))):
          clock.tick(dt)
          n = len(pm.full)
          self.view_steps(det, pm, 1, 'auto', start=2 * k)
          if len(pm.full) > n:
            out.append((clock.t - start, pm.full[-1].nativeStatus, pm.full[-1].farStatus))
        said_s = NATIVE_NO_RUN_S / STEP_GAP_S * dt  # 300 steps
        assert native.run.calls == [] and out[-1][0] > said_s + 20., dt
        dead = [t for t, ns, fs in out if ns == NativeStatus.unavailable]
        # (the first step counts its whole time since the load, with no step before it; a message every other step)
        assert dead and said_s - dt - 1e-6 <= dead[0] <= said_s + 2 * dt, (dt, dead[:1])
        assert all(ns == NativeStatus.full and fs == FarStatus.noRoom for t, ns, fs in out[1:] if t < dead[0]), dt
        assert all(ns == NativeStatus.unavailable and fs == FarStatus.disabled for t, ns, fs in out if t >= dead[0]), dt
        assert self.far_view_offs([fs for _, _, fs in out]) == 1, dt
        assert [c.args[0] for c in error.call_args_list] == [
          "turnpilot native schedule: no room in 30 s of its free, calibrated slot beside a 37 ms model"], dt

  def test_the_words_say_a_loaded_native_may_never_run(self):
    # 11-06 review: diagnostics said nativeStatus unavailable meant 'not loaded', and preflight's PASS promised the strips run
    # in the far crop's place; on the car it was loaded and never had room beside the big model. Both now say that can be
    import tempfile
    from pathlib import Path
    from openpilot.selfdrive.turnpilot import preflight
    from openpilot.selfdrive.turnpilot.diagnostics import NATIVE_WORDS
    words = NATIVE_WORDS['unavailable']
    assert 'not loaded' in words and 'no room beside the driving model' in words and 'far crop' in words
    with tempfile.TemporaryDirectory() as d:
      path = Path(d) / "big_native_detector_1344x760_1344x512_tinygrad.pkl"
      path.write_bytes(b"\0" * 5000)
      ok = preflight.check_native('full', path, True)
    assert ok.status == preflight.PASS and ok.detail.startswith("full: the whole frame natively") and path.name in ok.detail
    assert 'where it has room' in ok.detail and 'never runs' in ok.detail and 'nativeStatus unavailable' in ok.detail
