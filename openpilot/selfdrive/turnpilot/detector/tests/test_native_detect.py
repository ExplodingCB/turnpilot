"""The native schedule (TurnPilotNativeDetect, modeld_detector NativeStrips): full-width native strips of the road frame in the
far crop's slot, their boxes in frame pixels, the strips' seam, the fallback to the far crop, and the off path as today."""
import hashlib
import json
import time
from pathlib import Path

import numpy as np

from openpilot.cereal import custom, log, messaging
from openpilot.common.params import UnknownKeyName
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.detector.decoder import CAR, PERSON, TRAFFIC_LIGHT, decode_far, fill_far
from openpilot.selfdrive.turnpilot.detector.graph import (FAR_HORIZON_ROW, MODELS_DIR, NATIVE_SIZES, native_detector_pkl_path)
from openpilot.selfdrive.turnpilot.detector.modeld_detector import (FAR_OVERRUNS, NATIVE_BUDGET_S, RUN_EVERY, STEP_BUDGET_S, TIME_WINDOW,
                                                                    DetectorHost, NativeStatus, NativeStrips, load_native,
                                                                    native_detect_mode, native_strips)
from openpilot.selfdrive.turnpilot.detector.tests import native_golden
from openpilot.selfdrive.turnpilot.detector.tests.test_far_crop import FarPM, far_detector, steps
from openpilot.selfdrive.turnpilot.detector.tests.test_modeld_detector import FakeLights, FakeOutput, Meta
from openpilot.selfdrive.turnpilot.scene.ranging import road_lens
from openpilot.selfdrive.turnpilot.scene.tracker import EDGE_PX, SceneTracker

FarStatus = custom.TurnpilotDetections.FarStatus
CAMS = DEVICE_CAMERAS[('mici', 'os04c10')]  # the comma four: 1344x760
ROAD = CAMS.narrow_road
W, H = 1344, 760
# native_golden's drive on 263065a, before the native schedule: (full messages, far messages, sha256)
GOLDEN_263065A = (20, 20, 'fa519195deed4894e02ffc8d0bc8adaca05cf5a4e79b481658eca971f89fcfd7')


class Params:
  def __init__(self, value=None, unknown=False):
    self.value, self.unknown = value, unknown

  def get(self, key):
    assert key == "TurnPilotNativeDetect"
    if self.unknown:
      raise UnknownKeyName(key)
    return self.value


class FakeNativeRun:
  """Stands in for a native strip's jit: records the views it got; returns rows (strip input pixels) per view key."""
  def __init__(self, rows=None):
    self.calls, self.fail, self.delay = [], False, 0.
    self.rows = rows or [[600., 300., 640., 330., 0.7, CAR], [900., 100., 908., 124., 0.7, TRAFFIC_LIGHT]]

  def __call__(self, luma, chroma):
    if self.fail:
      raise RuntimeError("gpu fell over")
    if self.delay:
      time.sleep(self.delay)
    self.calls.append((luma, chroma))
    raw = np.zeros((1, 300, 6), np.float32)
    raw[0, :len(self.rows)] = self.rows
    return FakeOutput(raw)


def native_detector(mode='full', rpy=(0., 0., 0.), native_time=0.012, far_time=0.006, rows=None):
  """test_far_crop's detector (a far crop, the full pass) with the native schedule of mode loaded and on."""
  det = far_detector(rpy=rpy, far_time=far_time)
  native = NativeStrips(FakeNativeRun(rows), W, H, lambda crop: (f'luma{crop.x0},{crop.y0}', f'chroma{crop.x0},{crop.y0}'), None, mode)
  native.lens = road_lens(ROAD)
  native.calibration = lambda: None if rpy is None else list(rpy)
  native.warmup_s = native_time
  det.native, det.native_mode = native, mode
  return det


def statuses(pm):
  return [td.nativeStatus for td in pm.full]


class TestNativeDetect(OpenpilotTestCase):
  def test_param(self):
    assert native_detect_mode(Params()) == 'off'  # unset: the default
    assert native_detect_mode(Params('strip')) == 'strip' and native_detect_mode(Params(b'full')) == 'full'
    assert native_detect_mode(Params('full\n')) == 'full'
    assert native_detect_mode(Params('off')) == native_detect_mode(Params('both')) == native_detect_mode(Params('')) == 'off'
    assert native_detect_mode(Params(unknown=True)) == 'off'  # a build whose params predate the key

  def test_the_strips_cover_the_frame(self):
    top, bottom = native_strips(W, H, 'full')
    assert (top.rect, bottom.rect) == ((0, 0, 1344, 512), (0, 248, 1344, 760))
    assert top.rect[3] - bottom.rect[1] == 264  # rows 248-511 in both, the horizon (row ~380) among them
    assert native_strips(W, H, 'strip') is None and native_strips(W, H, 'off') is None
    assert native_strips(1928, 1208, 'full') is None  # no native strips for the comma 3x's frames
    # strip: full width at any aim, the horizon FAR_HORIZON_ROW down it, moving with the calibrated pitch
    native = native_detector('strip').native
    crops = {native.aim(b, [0., 0., 0.]).rect for b in (-30., 0., 8., 30.)}
    assert crops == {(0, 380 - FAR_HORIZON_ROW, 1344, 380 - FAR_HORIZON_ROW + 384)}
    pitched = native.aim(0., [0., np.radians(2.), 0.])
    v = road_lens(ROAD).project(native.view_ray(0., [0., np.radians(2.), 0.]))[1]
    assert pitched.x0 == 0 and pitched.in_w == 1344 and abs(pitched.y0 + FAR_HORIZON_ROW - v) <= 1 and abs(v - 380) > 30

  def test_box_mapping_per_strip(self):
    # a box in a strip's input pixels is the frame's by the strip's offset alone, exactly, clipped to the strip
    raw = np.zeros((1, 300, 6), np.float32)
    raw[0, 0] = [10.25, 20.5, 50.75, 80., 0.9, PERSON]
    raw[0, 1] = [1300., 300., 1360., 400., 0.8, CAR]  # past the strip's right edge (and the 384 strip's bottom)
    for crop in [*native_strips(W, H, 'full'), native_detector('strip').native.aim(0., [0., 0., 0.])]:
      dets = decode_far(raw, crop)
      person = dets[dets[:, 5] == PERSON][0]
      np.testing.assert_array_equal(person[:4], np.array([10.25, 20.5 + crop.y0, 50.75, 80. + crop.y0], np.float32))
      car = dets[dets[:, 5] == CAR][0]
      assert car[2] == 1344 and car[3] == min(400. + crop.y0, crop.y0 + crop.in_h)
      msg = messaging.new_message('turnpilotFarDetections')
      fill_far(msg.turnpilotFarDetections, dets, crop, 0., 7, 350_000_000, 0.01)
      fd = msg.as_reader().turnpilotFarDetections
      assert list(fd.crop) == [float(v) for v in crop.rect] and str(fd.run.view) == 'far' and str(fd.run.camera) == 'road'

  def test_full_alternates_the_strips_in_the_far_crops_slot(self):
    det, pm = native_detector('full'), FarPM()
    steps(det, pm, 4 * RUN_EVERY)
    assert len(pm.full) == 4 and det.run.calls == ['frame0'] * 4  # the full pass's grid is unchanged
    assert [tuple(fd.crop) for fd in pm.far] == [(0, 0, 1344, 512), (0, 248, 1344, 760)] * 2
    assert det.native.run.calls[:2] == [('luma0,0', 'chroma0,0'), ('luma0,248', 'chroma0,248')]
    assert det.far.run.calls == []  # the far crop never ran
    # the car at strip row 300: frame row 300 in the top strip, 548 in the bottom; class 9 is not a detection
    assert [fd.run.detections[0].y0 for fd in pm.far[:2]] == [300., 548.]
    assert all(len(fd.run.detections) == 1 and fd.run.detections[0].cls == CAR for fd in pm.far)
    assert all(td.farStatus == FarStatus.read for td in pm.full[1:]) and statuses(pm) == [NativeStatus.full] * 4

  def test_strip_takes_the_far_crops_slot_at_the_horizon(self):
    det, pm = native_detector('strip'), FarPM()
    steps(det, pm, 3 * RUN_EVERY, bearing=12.)
    assert len(pm.far) == 3 and det.far.run.calls == []
    y0 = 380 - FAR_HORIZON_ROW
    assert all(tuple(fd.crop) == (0, y0, 1344, y0 + 384) and abs(fd.bearing - 12.) < 1e-6 for fd in pm.far)
    assert pm.far[0].run.detections[0].y0 == 300. + y0 and statuses(pm) == [NativeStatus.strip] * 3

  def test_the_seam_between_the_strips(self):
    # where the strips meet: tracker B (tile_boxes) drops a box cut by the top strip's bottom edge, which is no frame edge,
    # and keeps one cut by the bottom strip's top edge (its feet in view); every person's feet are whole in one strip
    tracker = SceneTracker(CAMS, 'mici')
    top, bottom = native_strips(W, H, 'full')

    def taken(frame_box, crop):
      x0, y0, x1, y1 = frame_box
      raw = np.zeros((1, 300, 6), np.float32)
      raw[0, 0] = [x0 - crop.x0, y0 - crop.y0, x1 - crop.x0, y1 - crop.y0, 0.8, PERSON]
      msg = messaging.new_message('turnpilotFarDetections')
      fill_far(msg.turnpilotFarDetections, decode_far(raw, crop), crop, 0., 1, 50_000_000, 0.01)
      fd = msg.as_reader().turnpilotFarDetections
      return [(d.x0, d.y0, d.x1, d.y1) for d in tracker.tile_boxes(fd.run.detections, tuple(fd.crop))]

    near = (600., 400., 660., 600.)  # a person at about 8 m, feet below the top strip
    assert taken(near, top) == [] and taken(near, bottom) == [near]
    tall = (600., 150., 700., 700.)  # taller than the 264 shared rows: cut in both
    assert taken(tall, top) == [] and taken(tall, bottom) == [(600., 248., 700., 700.)]
    at_seam = (600., 300., 640., 512. - EDGE_PX / 2)  # feet at the top strip's edge, whole in the bottom strip
    assert taken(at_seam, top) == [] and taken(at_seam, bottom) == [at_seam]
    shared = (600., 300., 640., 420.)  # in the shared rows: whole in both, the same frame pixels from either
    assert taken(shared, top) == taken(shared, bottom) == [shared]
    hood_edge = (600., 600., 660., 760.)  # at the frame's own bottom: kept
    assert taken(hood_edge, bottom) == [hood_edge]
    far_side = (0., 350., 30., 420.)  # at the frame's left edge, which every strip shares: kept
    assert taken(far_side, top) == taken(far_side, bottom) == [far_side]

  def test_slow_native_falls_back_to_the_far_crop_for_the_drive(self):
    det, pm = native_detector('full'), FarPM()
    det.native.run.delay = NATIVE_BUDGET_S + 0.002
    steps(det, pm, TIME_WINDOW * RUN_EVERY)
    assert det.native.disabled and len(pm.far) == TIME_WINDOW and det.far.run.calls == []
    assert statuses(pm) == [NativeStatus.full] * TIME_WINDOW and det.native_status() == NativeStatus.fallback
    steps(det, pm, 2 * RUN_EVERY, start=100)  # then the far crop, never nothing
    assert len(det.far.run.calls) == 2 and [tuple(fd.crop)[2] - tuple(fd.crop)[0] for fd in pm.far[-2:]] == [640, 640]
    assert pm.full[-1].farStatus == FarStatus.read and pm.full[-1].nativeStatus == NativeStatus.fallback
    assert len(det.native.run.calls) == TIME_WINDOW and det.failures == 0 and not det.far.disabled

  def test_step_overruns_fall_back(self):
    det, pm = native_detector('full'), FarPM()
    det.native.estimate = lambda lights_on: 0.001  # fits by its estimate every time...
    det.native.run.delay = 0.006                   # ...but with the model's 40 ms the run takes the step past STEP_BUDGET_S
    steps(det, pm, FAR_OVERRUNS * RUN_EVERY, model_time=STEP_BUDGET_S - 0.005)
    assert det.native.disabled and len(det.native.run.calls) == FAR_OVERRUNS and det.native_status() == NativeStatus.fallback
    assert len(det.native.times) == FAR_OVERRUNS and float(np.median(det.native.times)) < NATIVE_BUDGET_S  # the step's rule, not the median's
    steps(det, pm, RUN_EVERY, start=100)
    assert len(det.far.run.calls) == 1 and statuses(pm)[-1] == NativeStatus.fallback

  def test_a_failure_falls_back_at_once(self):
    det, pm = native_detector('strip'), FarPM()
    det.native.run.fail = True
    steps(det, pm, 3 * RUN_EVERY)
    assert det.native.disabled and len(det.far.run.calls) == 2 and len(pm.far) == 2  # the far crop from the next odd step
    assert det.failures == 0 and det.retry_at == 0. and statuses(pm)[-1] == NativeStatus.fallback

  def test_a_frame_without_room_for_the_strip_is_the_far_crops(self):
    det, pm = native_detector('full', native_time=0.012, far_time=0.006), FarPM()
    steps(det, pm, 2 * RUN_EVERY, model_time=STEP_BUDGET_S - 0.008)  # room for the far crop (6 ms), not a strip (12 ms)
    assert det.native.run.calls == [] and len(det.far.run.calls) == 2 and not det.native.disabled
    assert pm.full[-1].farStatus == FarStatus.read and statuses(pm)[-1] == NativeStatus.full
    steps(det, pm, 2 * RUN_EVERY, start=100, model_time=STEP_BUDGET_S - 0.020)  # room again: the strips
    assert len(det.native.run.calls) == 2 and len(det.far.run.calls) == 2

  def test_far_crop_param_off_turns_the_slot_off(self):
    det, pm = native_detector('full'), FarPM()
    det.far_on = False
    steps(det, pm, 2 * RUN_EVERY)
    assert pm.far == [] and det.native.run.calls == [] and pm.full[-1].farStatus == FarStatus.off

  def test_a_stop_sweep_suspends_it(self):
    det, pm = native_detector('full'), FarPM()
    for k in range(2 * RUN_EVERY):
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, [Meta(2 * k), Meta(2 * k + 1)], 'stopSweep', 0., 5.)
    assert pm.far == [] and det.native.run.calls == [] and pm.full[-1].farStatus == FarStatus.sweep

  def test_native_lights_are_its_own_reader(self):
    det, pm = native_detector('full'), FarPM()
    det.light_mode, det.native.lights, det.far.lights = 'shadow', FakeLights(), FakeLights()
    steps(det, pm, RUN_EVERY)
    assert len(det.native.lights.calls) == 1 and det.far.lights.calls == [] and pm.far[0].run.lights.shadow

  def test_the_param_changed_since_the_load(self):
    det, pm = native_detector('full'), FarPM()
    det.native_mode = 'strip'  # loaded for full: the far crop until the next load
    steps(det, pm, RUN_EVERY)
    assert det.native.run.calls == [] and len(det.far.run.calls) == 1 and statuses(pm) == [NativeStatus.unavailable]
    det.native_mode = 'off'  # off takes effect at once
    steps(det, pm, RUN_EVERY, start=10)
    assert det.native.run.calls == [] and len(det.far.run.calls) == 2 and statuses(pm)[-1] == NativeStatus.off
    det.native_mode = 'full'
    steps(det, pm, RUN_EVERY, start=20)
    assert len(det.native.run.calls) == 1 and statuses(pm)[-1] == NativeStatus.full

  def test_off_is_todays_far_crop(self):
    # native_golden's drive gives 263065a's messages to the field: with the param off and nothing loaded, and with a native
    # schedule loaded but the param since turned off; nativeStatus is off on every full pass message
    assert native_golden.digest(native_golden.drive(native_golden.golden_detector(), native_golden.RecordingPM())) == GOLDEN_263065A
    det = native_golden.golden_detector()
    det.native = NativeStrips(FakeNativeRun(), W, H, lambda crop: ('l', 'c'), None, 'full')
    det.native.calibration = lambda: [0., 0., 0.]
    pm = native_golden.drive(det, native_golden.RecordingPM())
    assert native_golden.digest(pm) == GOLDEN_263065A and det.native.run.calls == []
    assert all(td.nativeStatus == NativeStatus.off for td in pm.full)
    assert all(json.loads(text).get('nativeStatus', 'off') == 'off' for name, text in pm.sent if name == 'turnpilotDetections')

  def test_the_golden_drive_is_deterministic(self):
    a = native_golden.digest(native_golden.drive(native_golden.golden_detector(), native_golden.RecordingPM()))
    b = native_golden.digest(native_golden.drive(native_golden.golden_detector(), native_golden.RecordingPM()))
    assert a == b

  def test_host_loads_the_mode_at_load(self):
    from openpilot.selfdrive.turnpilot.detector.tests.test_detector_recovery import FakeLoad, FakeParams

    class Model:
      chestnut = True
    for mode, want in ((None, []), ('off', []), ('full', ['full']), ('strip', ['strip'])):
      model, loaded = Model(), []
      load = FakeLoad({model})
      host = DetectorHost(FakeParams(True, native=mode), 1344, 760, False, True, [model], load_fn=load)
      host.load_fn = lambda m, *args, load=load, loaded=loaded: setattr(det := load(m, *args), 'load_native', loaded.append) or det
      host.step(FarPM(), model, 0.01, 0., 0, [Meta(0), Meta(1)], False)
      assert loaded == want and host.native_mode == (mode or 'off')

  def test_no_native_pkl_is_the_far_crop(self):
    class Model:
      chestnut = False  # no native pkl for the small model's device
    assert load_native(Model(), W, H, 0, 'full', None) is None
    assert load_native(Model(), 1928, 1208, 0, 'full', None) is None  # no strip for these frames

  def test_the_committed_pkls(self):
    # the chestnut's native strips are in git (plain blobs, not LFS) and match models/README.md's table
    readme = (MODELS_DIR / 'README.md').read_text()
    for mode in ('strip', 'full'):
      path = native_detector_pkl_path(W, H, NATIVE_SIZES[(W, H)][mode], True)
      assert path.is_file() and path.stat().st_size > 10e6, path
      assert hashlib.sha256(Path(path).read_bytes()).hexdigest() in readme
