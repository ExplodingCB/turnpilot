"""The detector inside modeld: after modelV2 is sent, on frames modeld already uploaded, it runs YOLO26 on one camera and
publishes turnpilotDetections. The road camera sees far ahead on straights; the wide one sees the crosswalk and cross
traffic in turns. turnpilotPlan.detectorView (read through fresh_plan, shadow too) overrides that pick: the unprotected left
(unprotected_left.py) asks for the road camera from its go to its commit, whatever its turn desire (CRITIQUE REQUIRED 3).
It runs in modeld because the chestnut's USB interface is claimed by one process.

Each run on the road camera also reads the lane paint (scene/lane_paint.py) on that frame, from the host copy modeld made
of it, with the frame's modelV2 and calibration read back off the bus and the detections masking what they hide, and
publishes it in the same message, with lanePaintStatus saying why when it isn't read. A run on the wide camera reads none:
unmasked, a white car over a yellow line reads as broken white paint. It costs about 1 ms on a Mac (lane_paint_video.py
measures it) and has its own budget: a read slower than PAINT_BUDGET_S (median) turns the paint off for the drive, loudly,
and the detector, which the pedestrian holds need, keeps running.

The traffic lights are read the same way (DESIGN phase 4 G2, detector/light_state.py): on road camera runs, from the host
copy of the frame, on the detector's class-9 boxes (decode_lights, never in the detections), with the frame's calibration
and modelV2 off the bus, into the message's lights (source camera, shadow unless TurnPilotCameraLights is addOnly). Its
own budget too: a failure, or a median read slower than LIGHT_BUDGET_S, turns the lights off for the drive
(lightStatus disabled, which turnpilotd raises once), and the detector keeps running. TurnPilotCameraLights off reads none.

The far crop (DESIGN phase 4 G5, FarCrop): a FAR_CROP_SIZE window of the road camera's frame at native resolution
(graph.Crop: twice the full pass's pixel density, a car 1.8 m wide is 9 px at 230 m), run on the model frames between the
full pass's runs and published apart (turnpilotFarDetections), so it never takes a frame the full pass is due on, forced or
not: the full pass always wins. It is aimed by turnpilotPlan.farCropBearing (read through fresh_plan, shadow too: the
unprotected left aims it up the oncoming approach, else straight ahead) at the calibrated horizon (FAR_HORIZON_ROW down
the crop), and one compiled graph serves every aim: it takes two byte windows of modeld's device copy of the frame
(crop_view_offsets). Its own budget: a run only where the model's time and its estimate fit STEP_BUDGET_S; a failure, a
median run over FAR_BUDGET_S, or FAR_OVERRUNS of its last TIME_WINDOW runs pushing the step past STEP_BUDGET_S turn it off
for the drive, logged, said by every full pass message (farStatus disabled, which turnpilotd raises once), and nothing of
the full pass's (its runs, its backoff) changes. Its class-9 boxes go to a light reader of its own, always in shadow.
TurnPilotFarCrop off, or no far pkl, runs none (farStatus off).

Nothing here runs unless TurnPilotEnabled is set and the compiled pkl for this device and camera size exists
(compile_detector.py builds it; the pkls are not in git).

Once it's on it stays on for the drive, and nothing here turns it off quietly: without fresh detections turnpilotd hands
control back to the driver (a soft disable), so every gap is loud. A slow spell only skips frames until a forced run,
inside turnpilotd's deadline, measures the time again, a failed run is retried with a backoff and logged, a mid-drive
TurnPilotEnabled starts it, and a fallback to the small driving model switches to that model's detector, where one
exists.
"""
import time
from collections import deque

import numpy as np

from openpilot.cereal import custom, log, messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.turnpilot.detector.decoder import decode, decode_far, decode_far_lights, decode_lights, fill_detections, fill_far
from openpilot.selfdrive.turnpilot.detector.graph import (Crop, crop_at, crop_views, detector_pkl_path, far_detector_pkl_path,
                                                         letterbox)
from openpilot.selfdrive.turnpilot.detector.light_state import LIGHT_MODE_DEFAULT, LightHead, LightReader, camera_lights_mode
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LanePaintClassifier, LinePaint, fill_lane_paint
from openpilot.selfdrive.turnpilot.scene.ranging import Lens, calib_from_view, camera_height, road_lens
from openpilot.selfdrive.turnpilot.scene.tracker import predicted_path
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

# chestnut, 2026-10-27, yolo26n at 1344x760: big model 25-27 ms, detector 7.9 ms in modeld (5.9 ms alone). Every frame
# fit with no drops; every 2nd keeps headroom for a slower host and in-stack load.
RUN_EVERY = 2              # at most every 2nd model frame (10 Hz)
STEP_BUDGET_S = 0.045      # driving model + detector must finish within this of the 50 ms cycle (the rest takes ~1 ms)
WIDE_CURVATURE = 0.01      # 1/m (a 100 m radius): at or above this, look through the wide camera
WIDE_HOLD_S = 2.0          # and keep looking through it this long after the turn ends
TURN_DESIRES = (log.Desire.turnLeft, log.Desire.turnRight)
# turnpilotPlan.detectorView (DESIGN phase 4 G4, CRITIQUE REQUIRED 3): road is the road camera only, overriding a turn's wide
# hold (an unprotected left from its go to its commit); roadAndWide the road camera with the wide one run in WIDE_EVERY (held
# at its line: its exit crosswalk); auto, choose_camera's own pick
WIDE_EVERY = 3
TIME_WINDOW = 5            # the detector time estimate is the median of this many runs, so one slow run can't stop it
# The estimate only changes when it runs, so after this many model frames skipped in a row, or this long by the clock
# (a loaded host runs the driving model below 20 Hz), it runs anyway and the estimate starts over from that run: a burst
# of slow runs under load can't keep it off once the load has gone. The wait never grows: turnpilotd hands back
# DETECTOR_DEADLINE_S (1.5 s) after the last detections, so a forced run must land well inside (DETECTOR_DEADLINE_S -
# STALE_S) of the one before; each costs one step past the budget, one in 16 frames while nothing fits. A detector that
# can't run in time even so (a forced run itself too slow, failures) is a loud handback in turnpilotd, never quietly off.
FORCE_RUN_FRAMES = 16      # 0.8 s at 20 Hz
FORCE_RUN_S = 0.8
RETRY_S = 1.0              # a failed run is retried after this, doubled for each failure in a row...
RETRY_MAX_S = 60.          # ...to this
PARAM_CHECK_S = 1.0        # TurnPilotEnabled is read this often
PAINT_BUDGET_S = 0.005     # the lane paint read's median over TIME_WINDOW reads above this turns it off for the drive
LIGHT_BUDGET_S = 0.004     # the light read's median over TIME_WINDOW reads above this turns the lights off for the drive
# the far crop (FarCrop). Chestnut, 2026-10-28, yolo26n on the 640x384 crop alone: 6.0 ms (the full pass 5.9 ms alone, 7.9 ms
# beside the big model); a median run over FAR_BUDGET_S is more than the frames between the full runs were measured to hold
FAR_BUDGET_S = 0.012
FAR_OVERRUNS = 3           # of its last TIME_WINDOW runs pushing the step past STEP_BUDGET_S: off for the drive
FAR_STATUS_S = 0.5         # farStatus read: it ran within this
FAR_VIEW_CACHE = 64        # device views kept, one per crop position (graph.crop_at aligns them to a coarse grid)
FAR_CROP_DEFAULT = True    # TurnPilotFarCrop unset or unknown to this build
LanePaintStatus = custom.TurnpilotDetections.LanePaintStatus
LightStatus = custom.TurnpilotDetections.LightStatus
FarStatus = custom.TurnpilotDetections.FarStatus


def frame_view(model_state, index: int):
  """A uint8 device view of packed frame `index` (0 main camera, 1 extra) in the input modeld uploads every step."""
  from tinygrad import Tensor, dtypes
  from tinygrad.uop.ops import UOp
  size = model_state.frame_copy_size
  offset = model_state.packed_input.nbytes - 2 * size + index * size
  view = model_state.input_device.view(size, offset).ensure_allocated()
  return Tensor(UOp.from_buffer(view, dtypes.uint8))


def crop_view_offsets(packed_nbytes: int, frame_size: int, index: int, crop: Crop) -> tuple[tuple[int, int], tuple[int, int]]:
  """(offset, size) in modeld's packed input of the far crop's two views (graph.crop_views) of packed frame `index`."""
  base = packed_nbytes - 2 * frame_size + index * frame_size
  (lo, ln), (co, cn) = crop_views(crop)
  return (base + lo, ln), (base + co, cn)


def crop_frame_views(model_state, index: int):
  """Crop -> (luma, chroma) uint8 device views of packed frame `index`, what the far crop's graph takes."""
  from tinygrad import Tensor, dtypes
  from tinygrad.uop.ops import UOp

  def views(crop: Crop):
    out = []
    for offset, size in crop_view_offsets(model_state.packed_input.nbytes, model_state.frame_copy_size, index, crop):
      out.append(Tensor(UOp.from_buffer(model_state.input_device.view(size, offset).ensure_allocated(), dtypes.uint8)))
    return tuple(out)
  return views


def far_crop_on(params) -> bool:
  """TurnPilotFarCrop: FAR_CROP_DEFAULT (on) if unset or unknown to this build's params (said)."""
  try:
    value = params.get("TurnPilotFarCrop")
  except UnknownKeyName:
    cloudlog.error(f"turnpilot: TurnPilotFarCrop unknown to this build's params, far crop {'on' if FAR_CROP_DEFAULT else 'off'}; run scons")
    return FAR_CROP_DEFAULT
  if value is None:
    return FAR_CROP_DEFAULT
  if isinstance(value, (bytes, str)):
    return (value.decode() if isinstance(value, bytes) else value).strip() == '1'
  return bool(value)


def frame_bus():
  return messaging.SubMaster(['modelV2', 'extrinsicsCalibration', 'deviceState', 'narrowRoadCameraState'])


class LanePaintReader:
  """Lane paint on the road camera's frame: modelV2 and calibration for that very frame come back off the bus (modeld has
  just sent the modelV2), and the lens from the device type."""
  def __init__(self, cam_w: int, cam_h: int, sm=None):
    self.sm = sm if sm is not None else frame_bus()
    self.stride, self.y_height, _, _ = get_nv12_info(cam_w, cam_h)
    self.classifier: LanePaintClassifier | None = None

  def read(self, frame: np.ndarray, meta, boxes) -> tuple[list[LinePaint], int]:
    """(paint, lanePaintStatus). frame: the road camera's padded NV12, flat; boxes: this frame's detections (x0 y0 x1 y1 ...),
    or None."""
    sm = self.sm
    sm.update(0)
    if self.classifier is None:
      if not (sm.seen['deviceState'] and sm.seen['narrowRoadCameraState']):
        return [UNKNOWN] * 4, LanePaintStatus.noModel
      cam = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))]
      self.classifier = LanePaintClassifier(road_lens(cam.narrow_road))
    model, cal = sm['modelV2'], sm['extrinsicsCalibration']
    if not sm.seen['modelV2'] or model.frameId != meta.frame_id % 2**32:
      return [UNKNOWN] * 4, LanePaintStatus.noModel  # not this frame's lines
    lines = [(np.array(ln.x), np.array(ln.y), np.array(ln.z)) for ln in model.laneLines]
    calibrated = sm.seen['extrinsicsCalibration'] and cal.calStatus == log.ExtrinsicsCalibration.Status.calibrated
    return self.classifier.update(meta.timestamp_eof * 1e-9, frame, self.stride, self.y_height, list(cal.rpyCalib), calibrated,
                                  lines, list(model.laneLineProbs), boxes), LanePaintStatus.read


class CameraLightReader:
  """The lights on the road camera's frame (light_state.py), with the lane paint's bus: calibration for the horizon, and
  modelV2 for the lane's bearing when it is this very frame's."""
  def __init__(self, sm):
    self.sm = sm
    self.reader: LightReader | None = None

  def read(self, frame: np.ndarray, meta, boxes: np.ndarray) -> tuple[list[LightHead], int]:
    sm = self.sm
    sm.update(0)
    if self.reader is None:
      if not (sm.seen['deviceState'] and sm.seen['narrowRoadCameraState']):
        return [], LightStatus.noCalibration
      cam = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))]
      self.reader = LightReader(road_lens(cam.narrow_road))
      self.stride, self.y_height, _, _ = get_nv12_info(cam.narrow_road.width, cam.narrow_road.height)
    cal = sm['extrinsicsCalibration']
    if not (sm.seen['extrinsicsCalibration'] and cal.calStatus == log.ExtrinsicsCalibration.Status.calibrated):
      return [], LightStatus.noCalibration
    model = sm['modelV2']
    path = predicted_path(model) if sm.seen['modelV2'] and model.frameId == meta.frame_id % 2**32 else None
    heads = self.reader.read(meta.timestamp_eof * 1e-9, frame, self.stride, self.y_height, list(cal.rpyCalib), camera_height(cal),
                             boxes, path)
    return heads, LightStatus.read


class FarCrop:
  """The far crop pass (module docstring): its compiled graph run(luma=, chroma=), views (Crop -> the two device views),
  its aim from the bus's calibration, its own time estimate and budget, and its own light reader."""
  def __init__(self, run, cam_w: int, cam_h: int, views, sm=None):
    self.run, self.cam_w, self.cam_h, self.views_fn, self.sm = run, cam_w, cam_h, views, sm
    self.lens: Lens | None = None
    self.times: deque[float] = deque(maxlen=TIME_WINDOW)
    self.overruns: deque[bool] = deque(maxlen=TIME_WINDOW)
    self.light_times: deque[float] = deque(maxlen=TIME_WINDOW)
    self.warmup_s = 0.              # the load's warm-up runs' time, the estimate before its first run
    self.disabled = False           # failed or over budget: off for the drive
    self.last_run_t: float | None = None  # time.monotonic() of its last run
    self.why = FarStatus.noRoom     # what kept it from running last
    self.lights: CameraLightReader | None = None  # its own (None: off, or failed)
    self.lights_failed = False
    self._views: dict = {}

  def calibration(self) -> list | None:
    """rpyCalib when calibrated (and the road lens, once the device is known), else None."""
    sm = self.sm
    sm.update(0)
    if self.lens is None:
      if not (sm.seen['deviceState'] and sm.seen['narrowRoadCameraState']):
        return None
      self.lens = road_lens(DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))].narrow_road)
    cal = sm['extrinsicsCalibration']
    if not (sm.seen['extrinsicsCalibration'] and cal.calStatus == log.ExtrinsicsCalibration.Status.calibrated):
      return None
    return list(cal.rpyCalib)

  @staticmethod
  def view_ray(bearing_deg: float, rpy) -> np.ndarray:
    """The road camera's view-frame ray to the horizon bearing_deg left of the calibrated forward axis."""
    b = np.radians(bearing_deg)
    return calib_from_view(list(rpy)).T @ np.array([np.cos(b), -np.sin(b), 0.])  # calib frame: x forward, y right, z down

  def aim(self, bearing_deg: float, rpy) -> Crop:
    """The crop centred on the horizon bearing_deg left of the calibrated forward axis (graph.crop_at)."""
    assert self.lens is not None
    u, v = self.lens.project(self.view_ray(bearing_deg, rpy))
    return crop_at(self.cam_w, self.cam_h, float(u), float(v))

  def views(self, crop: Crop):
    key = (crop.x0, crop.y0)
    if key not in self._views:
      if len(self._views) >= FAR_VIEW_CACHE:
        self._views.clear()
      self._views[key] = self.views_fn(crop)
    return self._views[key]

  def estimate(self, lights_on: bool) -> float:
    est = float(np.median(self.times)) if self.times else self.warmup_s
    if lights_on and self.lights is not None and self.light_times:
      est += float(np.median(self.light_times))
    return est

  def status(self, now: float) -> int:
    if self.disabled:
      return FarStatus.disabled
    if self.last_run_t is not None and now - self.last_run_t <= FAR_STATUS_S:
      return FarStatus.read
    return self.why

  def disable(self, why: str) -> None:
    if not self.disabled:
      cloudlog.error(f"turnpilot far crop {why}, off for this drive")
    self.disabled = True

  def ran(self, now: float, dt: float, overran: bool) -> None:
    """A run of dt s at now; overran: with the model's time it pushed the step past STEP_BUDGET_S."""
    self.last_run_t = now
    self.times.append(dt)
    self.overruns.append(overran)
    if len(self.times) == TIME_WINDOW and float(np.median(self.times)) > FAR_BUDGET_S:
      self.disable(f"over budget ({np.median(self.times) * 1e3:.1f} ms)")
    elif sum(self.overruns) >= FAR_OVERRUNS:
      self.disable(f"pushed the step past {STEP_BUDGET_S * 1e3:.0f} ms {sum(self.overruns)} times in {len(self.overruns)} runs")


def fill_lights(tl, heads: list[LightHead], shadow: bool) -> None:
  """A TurnpilotLights builder from the camera's heads."""
  tl.source, tl.shadow = custom.TurnpilotLights.Source.camera, shadow
  out = tl.init('lights', len(heads))
  for o, h in zip(out, heads, strict=True):
    o.state, o.distance, o.instance = h.state, float(h.distance), f"head{h.id}"
    o.conf, o.box, o.bearing, o.votes, o.governs = float(h.conf), [float(b) for b in h.box], float(h.bearing), int(h.votes), h.governs


class ModeldDetector:
  failures = 0                    # failed runs in a row
  retry_at = 0.0                  # time.monotonic() a failed detector waits for
  last_run_t: float | None = None  # time.monotonic() of the last run (None: none yet)
  light_mode = LIGHT_MODE_DEFAULT  # TurnPilotCameraLights, set by DetectorHost
  lights: CameraLightReader | None = None  # None: no road camera, or failed (lights_failed)
  lights_failed = False
  view_runs = 0                   # runs under the roadAndWide view, for its one wide run in WIDE_EVERY
  far: FarCrop | None = None      # the far crop (None: no road camera, no far pkl, or it failed to load)
  far_on = FAR_CROP_DEFAULT       # TurnPilotFarCrop, set by DetectorHost

  def __init__(self, model_state, cam_w: int, cam_h: int, cameras: dict[str, int]):
    """cameras: 'road' and/or 'wide' -> index of its frame in modeld's packed input."""
    from tinygrad.engine.realize import lower_and_compile
    from openpilot.selfdrive.modeld.helpers import load_oob
    art = load_oob(detector_pkl_path(cam_w, cam_h, model_state.chestnut), model_state.chestnut)
    if art['input_specs']['input_frame'][0] != (model_state.frame_copy_size,):
      raise ValueError(f"detector compiled for frames of {art['input_specs']['input_frame'][0]}, modeld has {model_state.frame_copy_size}")
    self.run = art['run']
    self.run.captured._linear = lower_and_compile(self.run.captured._linear)
    self.lb = letterbox(cam_w, cam_h, tuple(art['metadata']['input_size']))
    self.model_state = model_state
    self.cameras = cameras
    self.frames = {cam: frame_view(model_state, i) for cam, i in cameras.items()}
    self.times: deque[float] = deque(maxlen=TIME_WINDOW)
    self.paint_times: deque[float] = deque(maxlen=TIME_WINDOW)
    self.light_times: deque[float] = deque(maxlen=TIME_WINDOW)
    for _ in range(3):  # warm up, and a first time estimate
      t0 = time.perf_counter()
      self.run(input_frame=self.frames[next(iter(self.frames))]).numpy()
      self.times.append(time.perf_counter() - t0)
    self.since_run, self.last_run_t = RUN_EVERY, time.monotonic()
    self.wide_until = -1.0
    bus = frame_bus() if 'road' in cameras else None
    self.lane_paint = LanePaintReader(cam_w, cam_h, bus) if 'road' in cameras else None
    self.lights = CameraLightReader(bus) if 'road' in cameras else None
    self.far = load_far(model_state, cam_w, cam_h, cameras['road'], bus) if 'road' in cameras else None


  def choose_camera(self, desired_curvature: float, desire: int, now: float, view: str = 'auto') -> str:
    """The camera for this run: the wide one in a turn or on a curve and WIDE_HOLD_S after, else the road one; the plan's
    view (WIDE_EVERY) overrides that. A device with one camera runs it."""
    if abs(desired_curvature) >= WIDE_CURVATURE or desire in TURN_DESIRES:
      self.wide_until = now + WIDE_HOLD_S
    if view == 'road':
      want = 'road'
    elif view == 'roadAndWide':
      self.view_runs = (self.view_runs + 1) % WIDE_EVERY
      want = 'wide' if self.view_runs == 0 else 'road'
    else:
      want = 'wide' if now < self.wide_until else 'road'
    return want if want in self.frames else next(iter(self.frames))

  def room(self, model_execution_time: float) -> bool:
    """Whether this frame has room for a run: the driving model's time plus the estimate (the detector's, and the lane
    paint read's while it is on) fit the budget, or the detector has gone FORCE_RUN_FRAMES or FORCE_RUN_S without and
    runs to measure again."""
    estimate = float(np.median(self.times)) if self.times else 0.
    if self.lane_paint is not None and self.paint_times:
      estimate += float(np.median(self.paint_times))
    if self.lights is not None and self.light_mode != 'off' and self.light_times:
      estimate += float(np.median(self.light_times))
    if model_execution_time + estimate <= STEP_BUDGET_S:
      return True
    waited_s = time.monotonic() - self.last_run_t if self.last_run_t is not None else 0.
    if self.since_run < FORCE_RUN_FRAMES and waited_s < FORCE_RUN_S:
      return False
    cloudlog.info(f"turnpilot detector: no room for {waited_s:.2f} s at {1e3 * estimate:.0f} ms, measuring again")
    self.times.clear()
    return True

  def step(self, pm, model_state, model_execution_time: float, desired_curvature: float, desire: int, metas: list,
           view: str = 'auto', far_bearing: float = 0.) -> None:
    """Called once per model frame after modelV2 is sent, with the driving model this detector was loaded for.
    metas: FrameMeta of packed frames 0 and 1; view: turnpilotPlan.detectorView (choose_camera); far_bearing:
    turnpilotPlan.farCropBearing (FarCrop)."""
    if model_state is not self.model_state:
      # DetectorHost picks the detector by model, so this is a bug: raise for the host to log and retry, never a quiet off
      raise ValueError("turnpilot detector stepped with a driving model it wasn't loaded for")
    self.since_run += 1
    if self.since_run < RUN_EVERY:
      self.far_step(pm, model_state, model_execution_time, metas, far_bearing)  # a frame between the full runs
      return
    if time.monotonic() < self.retry_at or not self.room(model_execution_time):
      return
    try:
      cam = self.choose_camera(desired_curvature, desire, time.monotonic(), view)
      t0 = time.perf_counter()
      raw = self.run(input_frame=self.frames[cam]).numpy()
      dt = time.perf_counter() - t0
      self.since_run, self.last_run_t = 0, time.monotonic()
      meta = metas[self.cameras[cam]]
      dets = decode(raw, self.lb)
      msg = messaging.new_message('turnpilotDetections', valid=bool(np.isfinite(raw).all()))
      fill_detections(msg.turnpilotDetections, dets, meta.frame_id, meta.timestamp_eof, cam, dt)
      self.times.append(dt)
      if cam == 'road':
        self.read_lane_paint(msg.turnpilotDetections, model_state, metas, dets)
        self.read_lights(msg.turnpilotDetections, model_state, metas, raw)
      else:
        msg.turnpilotDetections.lanePaintStatus = LanePaintStatus.wideCamera if 'road' in self.cameras else LanePaintStatus.disabled
        msg.turnpilotDetections.lightStatus = self.light_status() if self.light_status() != LightStatus.read else LightStatus.wideCamera
      msg.turnpilotDetections.farStatus = self.far_status()
      pm.send('turnpilotDetections', msg)
      self.failures = 0
    except Exception:
      self.failures += 1
      delay = min(RETRY_S * 2 ** (self.failures - 1), RETRY_MAX_S)
      self.retry_at = time.monotonic() + delay
      cloudlog.exception(f"turnpilot detector failed ({self.failures} in a row), retrying in {delay:.0f} s")

  def far_status(self) -> int:
    """farStatus for the full pass's messages: off with TurnPilotFarCrop off or no far crop, else the far crop's own."""
    if self.far is None or (not self.far_on and not self.far.disabled):
      return FarStatus.off
    return self.far.status(time.monotonic())

  def far_step(self, pm, model_state, model_execution_time: float, metas: list, bearing: float) -> None:
    """The far crop on this frame between the full runs, where it has room (module docstring). Its failures turn it off for
    the drive and touch nothing of the full pass's."""
    far = self.far
    if far is None or far.disabled or not self.far_on:
      return
    try:
      rpy = far.calibration()
      if rpy is None:
        far.why = FarStatus.noCalibration
        return
      lights_on = far.lights is not None and self.light_mode != 'off'
      if model_execution_time + far.estimate(lights_on) > STEP_BUDGET_S:
        far.why = FarStatus.noRoom
        return
      crop = far.aim(bearing, rpy)
      luma, chroma = far.views(crop)
      t0 = time.perf_counter()
      raw = far.run(luma=luma, chroma=chroma).numpy()
      dt = time.perf_counter() - t0
      i = self.cameras['road']
      meta = metas[i]
      msg = messaging.new_message('turnpilotFarDetections', valid=bool(np.isfinite(raw).all()))
      fill_far(msg.turnpilotFarDetections, decode_far(raw, crop), crop, bearing, meta.frame_id, meta.timestamp_eof, dt)
      light_dt = self.read_far_lights(msg.turnpilotFarDetections.run, model_state, metas, raw, crop)
      pm.send('turnpilotFarDetections', msg)
      far.ran(time.monotonic(), dt, model_execution_time + dt + light_dt > STEP_BUDGET_S)
    except Exception:
      cloudlog.exception("turnpilot far crop failed")
      far.disable("failed")

  def read_far_lights(self, td, model_state, metas: list, raw: np.ndarray, crop: Crop) -> float:
    """The far crop's class-9 boxes through its own reader, always shadow: td.lights, lightStatus, lightTime. A failure or
    reads over LIGHT_BUDGET_S turn off only the far crop's lights, for the drive. Returns the time it took."""
    far = self.far
    assert far is not None
    if far.lights_failed:
      td.lightStatus = LightStatus.disabled
      return 0.
    if far.lights is None or self.light_mode == 'off':
      td.lightStatus = LightStatus.off
      return 0.
    t0 = time.perf_counter()
    try:
      i = self.cameras['road']
      heads, status = far.lights.read(model_state.frames[i], metas[i], decode_far_lights(raw, crop))
    except Exception:
      cloudlog.exception("turnpilot far crop lights failed, off for this drive")
      far.lights, far.lights_failed = None, True
      td.lightStatus = LightStatus.disabled
      return time.perf_counter() - t0
    dt = time.perf_counter() - t0
    fill_lights(td.lights, heads, True)
    td.lightStatus, td.lightTime = status, dt
    far.light_times.append(dt)
    if len(far.light_times) == TIME_WINDOW and float(np.median(far.light_times)) > LIGHT_BUDGET_S:
      cloudlog.error(f"turnpilot far crop lights over budget ({np.median(far.light_times) * 1e3:.1f} ms), off for this drive")
      far.lights, far.lights_failed = None, True
    return dt

  def read_lane_paint(self, td, model_state, metas: list, boxes: np.ndarray) -> None:
    """Fill td.lanePaint, lanePaintStatus and lanePaintTime from the road camera's frame, behind this run's detections on
    it. A failure, or reads over PAINT_BUDGET_S, turn only the paint off, for the drive; turnpilotd then plans no lane
    change (no paint is unknown paint)."""
    if self.lane_paint is None:
      td.lanePaintStatus = LanePaintStatus.disabled
      return
    t0 = time.perf_counter()
    try:
      i = self.cameras['road']
      paint, status = self.lane_paint.read(model_state.frames[i], metas[i], boxes)
    except Exception:
      cloudlog.exception("turnpilot lane paint failed, off for this drive")
      self.lane_paint = None
      td.lanePaintStatus = LanePaintStatus.disabled
      return
    dt = time.perf_counter() - t0
    fill_lane_paint(td.init('lanePaint', 4), paint)
    td.lanePaintStatus, td.lanePaintTime = status, dt
    self.paint_times.append(dt)
    if len(self.paint_times) == TIME_WINDOW and float(np.median(self.paint_times)) > PAINT_BUDGET_S:
      cloudlog.error(f"turnpilot lane paint over budget ({np.median(self.paint_times) * 1e3:.1f} ms), off for this drive")
      self.lane_paint = None


  def light_status(self) -> int:
    """What a run without a read says: disabled after a failure, off with the param off or no road camera, else read."""
    if self.lights_failed:
      return LightStatus.disabled
    return LightStatus.off if self.lights is None or self.light_mode == 'off' else LightStatus.read

  def read_lights(self, td, model_state, metas: list, raw: np.ndarray) -> None:
    """Fill td.lights, lightStatus and lightTime from the road camera's frame and this run's class-9 boxes. A failure, or
    reads over LIGHT_BUDGET_S, turn only the lights off, for the drive, said in lightStatus (disabled) and the log."""
    status = self.light_status()
    if status != LightStatus.read:
      td.lightStatus = status
      return
    t0 = time.perf_counter()
    try:
      i = self.cameras['road']
      heads, status = self.lights.read(model_state.frames[i], metas[i], decode_lights(raw, self.lb))
    except Exception:
      cloudlog.exception("turnpilot lights failed, off for this drive")
      self.lights, self.lights_failed = None, True
      td.lightStatus = LightStatus.disabled
      return
    dt = time.perf_counter() - t0
    fill_lights(td.lights, heads, self.light_mode != 'addOnly')
    td.lightStatus, td.lightTime = status, dt
    self.light_times.append(dt)
    if len(self.light_times) == TIME_WINDOW and float(np.median(self.light_times)) > LIGHT_BUDGET_S:
      cloudlog.error(f"turnpilot lights over budget ({np.median(self.light_times) * 1e3:.1f} ms), off for this drive")
      self.lights, self.lights_failed = None, True


def load_far(model_state, cam_w: int, cam_h: int, index: int, bus) -> FarCrop | None:
  """The far crop for the road camera, packed frame `index`, or None: no far pkl for this device (logged), or it failed to
  load (logged); the full pass runs either way."""
  try:
    from tinygrad.engine.realize import lower_and_compile
    from openpilot.selfdrive.modeld.helpers import load_oob
    path = far_detector_pkl_path(cam_w, cam_h, model_state.chestnut)
    if not path.is_file():
      cloudlog.warning(f"turnpilot far crop: no {path.name}, far crop off")
      return None
    art = load_oob(path, model_state.chestnut)
    centre = crop_at(cam_w, cam_h, cam_w / 2, cam_h / 2)
    (_, ln), (_, cn) = crop_views(centre)
    specs = art['input_specs']
    if art['metadata'].get('kind') != 'far' or specs['luma'][0] != (ln,) or specs['chroma'][0] != (cn,):
      raise ValueError(f"far crop compiled for {art['metadata']}, {specs}; this camera's views are {ln} and {cn} bytes")
    run = art['run']
    run.captured._linear = lower_and_compile(run.captured._linear)
    far = FarCrop(run, cam_w, cam_h, crop_frame_views(model_state, index), bus)
    times = []
    for _ in range(3):  # warm up, and a first time estimate
      t0 = time.perf_counter()
      run(**dict(zip(('luma', 'chroma'), far.views(centre), strict=True))).numpy()
      times.append(time.perf_counter() - t0)
    far.warmup_s = float(np.median(times))
    far.lights = CameraLightReader(bus)
    cloudlog.warning(f"turnpilot far crop loaded ({1e3 * far.warmup_s:.1f} ms warm)")
    return far
  except Exception:
    cloudlog.exception("turnpilot far crop failed to load, far crop off")
    return None


def load(model_state, cam_w: int, cam_h: int, main_wide_camera: bool, use_extra_client: bool) -> ModeldDetector | None:
  """The detector for one driving model's device, or None. DetectorHost only calls this with TurnPilotEnabled set."""
  try:
    path = detector_pkl_path(cam_w, cam_h, model_state.chestnut)
    if not path.is_file():
      cloudlog.warning(f"turnpilot detector: no {path.name}, so none for the {'big' if model_state.chestnut else 'small'} driving model")
      return None
    cameras = {'wide': 0} if main_wide_camera else ({'road': 0, 'wide': 1} if use_extra_client else {'road': 0})
    return ModeldDetector(model_state, cam_w, cam_h, cameras)
  except Exception:
    cloudlog.exception("turnpilot detector failed to load, detector off")
    return None


class DetectorHost:
  """What modeld holds: a ModeldDetector for each driving model it can run, loaded once TurnPilotEnabled is set and stepped
  while it stays set. With it unset or no pkl, nothing runs and modeld runs as without TurnPilot.

  Loading takes seconds (the pkl, its compile, warm-up runs), so after startup it waits for openpilot to be disengaged,
  when a late modelV2 steers nothing. The small model's detector loads with the big one's so a fallback switches over at
  once. That needs a pkl compiled on the small model's device, and models/ has only the chestnut's, so today a fallback
  leaves the detector off (logged), which turnpilotd sees as an unavailable scene."""

  def __init__(self, params, cam_w: int, cam_h: int, main_wide_camera: bool, use_extra_client: bool, models: list, load_fn=None):
    """models: the driving models modeld may run (the active one first); None and repeats are skipped."""
    self.params = params
    self.args = (cam_w, cam_h, main_wide_camera, use_extra_client)
    self.models = [m for i, m in enumerate(models) if m is not None and all(m is not n for n in models[:i])]
    self.load_fn = load_fn or load
    self.detectors: list[tuple] | None = None  # (driving model, its detector or None), once loaded
    self.enabled = False
    self.light_mode = LIGHT_MODE_DEFAULT  # TurnPilotCameraLights, read with TurnPilotEnabled
    self.far_on = FAR_CROP_DEFAULT        # TurnPilotFarCrop, read with TurnPilotEnabled
    self.next_check = 0.0
    self.missing = None  # the driving model last reported as having no detector
    self.failures = 0    # host failures in a row

  def check_enabled(self, now: float) -> bool:
    if now >= self.next_check:
      self.next_check = now + PARAM_CHECK_S
      self.enabled = self.params.get_bool("TurnPilotEnabled")
      self.light_mode = camera_lights_mode(self.params)
      self.far_on = far_crop_on(self.params)
    return self.enabled

  def load(self) -> None:
    self.detectors = [(m, self.load_fn(m, *self.args)) for m in self.models]

  def step(self, pm, model_state, model_execution_time: float, desired_curvature: float, desire: int, metas: list,
           engaged: bool, view: str = 'auto', far_bearing: float = 0.) -> None:
    """Called once per model frame after modelV2 is sent, with the model that made it. view: turnpilotPlan.detectorView;
    far_bearing: turnpilotPlan.farCropBearing."""
    try:
      if not self.check_enabled(time.monotonic()):
        return
      if self.detectors is None:
        if engaged:
          return
        cloudlog.warning("turnpilot detector: TurnPilot turned on, loading the detector")
        self.load()
      det = next((d for m, d in self.detectors or () if m is model_state), None)
      if det is None:
        if self.missing is not model_state:
          self.missing = model_state
          cloudlog.warning("turnpilot detector: none for the driving model modeld is running, detector off")
        return
      det.light_mode, det.far_on = self.light_mode, self.far_on
      det.step(pm, model_state, model_execution_time, desired_curvature, desire, metas, view, far_bearing)
      self.failures = 0
    except Exception:
      # never take modeld down: off until the next TurnPilotEnabled check, after a wait doubling from RETRY_S. turnpilotd
      # hands back to the driver if that outlasts its deadline
      self.failures += 1
      delay = min(RETRY_S * 2 ** (self.failures - 1), RETRY_MAX_S)
      cloudlog.exception(f"turnpilot detector host failed ({self.failures} in a row), retrying in {delay:.0f} s")
      self.enabled, self.next_check = False, time.monotonic() + delay
