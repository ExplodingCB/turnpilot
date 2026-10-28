"""The detector inside modeld: after modelV2 is sent, on frames modeld already uploaded, it runs YOLO26 on one camera and
publishes turnpilotDetections. The road camera sees far ahead on straights; the wide one sees the crosswalk and cross
traffic in turns. It runs in modeld because the chestnut's USB interface is claimed by one process.

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
from openpilot.selfdrive.turnpilot.detector.decoder import decode, decode_lights, fill_detections
from openpilot.selfdrive.turnpilot.detector.graph import detector_pkl_path, letterbox
from openpilot.selfdrive.turnpilot.detector.light_state import LIGHT_MODE_DEFAULT, LightHead, LightReader, camera_lights_mode
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LanePaintClassifier, LinePaint, fill_lane_paint
from openpilot.selfdrive.turnpilot.scene.ranging import camera_height, road_lens
from openpilot.selfdrive.turnpilot.scene.tracker import predicted_path
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

# chestnut, 2026-10-27, yolo26n at 1344x760: big model 25-27 ms, detector 7.9 ms in modeld (5.9 ms alone). Every frame
# fit with no drops; every 2nd keeps headroom for a slower host and in-stack load.
RUN_EVERY = 2              # at most every 2nd model frame (10 Hz)
STEP_BUDGET_S = 0.045      # driving model + detector must finish within this of the 50 ms cycle (the rest takes ~1 ms)
WIDE_CURVATURE = 0.01      # 1/m (a 100 m radius): at or above this, look through the wide camera
WIDE_HOLD_S = 2.0          # and keep looking through it this long after the turn ends
TURN_DESIRES = (log.Desire.turnLeft, log.Desire.turnRight)
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
LanePaintStatus = custom.TurnpilotDetections.LanePaintStatus
LightStatus = custom.TurnpilotDetections.LightStatus


def frame_view(model_state, index: int):
  """A uint8 device view of packed frame `index` (0 main camera, 1 extra) in the input modeld uploads every step."""
  from tinygrad import Tensor, dtypes
  from tinygrad.uop.ops import UOp
  size = model_state.frame_copy_size
  offset = model_state.packed_input.nbytes - 2 * size + index * size
  view = model_state.input_device.view(size, offset).ensure_allocated()
  return Tensor(UOp.from_buffer(view, dtypes.uint8))


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

  def choose_camera(self, desired_curvature: float, desire: int, now: float) -> str:
    if abs(desired_curvature) >= WIDE_CURVATURE or desire in TURN_DESIRES:
      self.wide_until = now + WIDE_HOLD_S
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

  def step(self, pm, model_state, model_execution_time: float, desired_curvature: float, desire: int, metas: list) -> None:
    """Called once per model frame after modelV2 is sent, with the driving model this detector was loaded for.
    metas: FrameMeta of packed frames 0 and 1."""
    if model_state is not self.model_state:
      # DetectorHost picks the detector by model, so this is a bug: raise for the host to log and retry, never a quiet off
      raise ValueError("turnpilot detector stepped with a driving model it wasn't loaded for")
    self.since_run += 1
    if self.since_run < RUN_EVERY or time.monotonic() < self.retry_at or not self.room(model_execution_time):
      return
    try:
      cam = self.choose_camera(desired_curvature, desire, time.monotonic())
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
      pm.send('turnpilotDetections', msg)
      self.failures = 0
    except Exception:
      self.failures += 1
      delay = min(RETRY_S * 2 ** (self.failures - 1), RETRY_MAX_S)
      self.retry_at = time.monotonic() + delay
      cloudlog.exception(f"turnpilot detector failed ({self.failures} in a row), retrying in {delay:.0f} s")

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
    self.next_check = 0.0
    self.missing = None  # the driving model last reported as having no detector
    self.failures = 0    # host failures in a row

  def check_enabled(self, now: float) -> bool:
    if now >= self.next_check:
      self.next_check = now + PARAM_CHECK_S
      self.enabled = self.params.get_bool("TurnPilotEnabled")
      self.light_mode = camera_lights_mode(self.params)
    return self.enabled

  def load(self) -> None:
    self.detectors = [(m, self.load_fn(m, *self.args)) for m in self.models]

  def step(self, pm, model_state, model_execution_time: float, desired_curvature: float, desire: int, metas: list,
           engaged: bool) -> None:
    """Called once per model frame after modelV2 is sent, with the model that made it."""
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
      det.light_mode = self.light_mode
      det.step(pm, model_state, model_execution_time, desired_curvature, desire, metas)
      self.failures = 0
    except Exception:
      # never take modeld down: off until the next TurnPilotEnabled check, after a wait doubling from RETRY_S. turnpilotd
      # hands back to the driver if that outlasts its deadline
      self.failures += 1
      delay = min(RETRY_S * 2 ** (self.failures - 1), RETRY_MAX_S)
      cloudlog.exception(f"turnpilot detector host failed ({self.failures} in a row), retrying in {delay:.0f} s")
      self.enabled, self.next_check = False, time.monotonic() + delay
