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

The stop sweep (STOP_SIGNS.md section 1.3, SideViews): turnpilotPlan.detectorView stopSweep (held at a stop or yield) or
stopSweepGo (crossing, from the commit point), read through fresh_plan, shadow too. Its even steps (the full pass's, every
RUN_EVERY) run the forward views: the full wide pass and the road camera alternating when held, the full wide pass every
time when crossing, whatever the turn desire or the wide hold after a turn. Its odd steps (the far crop's otherwise)
alternate the two side views (sideview.py: level pinholes at yaw 75 deg left and right of the calibrated axis, 768x512,
gathered from the wide frame through maps that are graph inputs), but only with the car at or under SIDE_VIEW_V_MAX: above
it the odd steps idle, so the approach costs what the detector costs today. The side views' boxes go back to the wide
camera's fisheye pixels (camera wide, view sideLeft or sideRight), each with its sectors' image quality (sectorQuality) from
the host copy of the wide frame. Their maps follow the bus's calibration: recomputed off the step (a thread) when it moves
more than sideview.RECALIB_DEG, uploaded on an odd step, the views invalid until then (sideViewStatus mapsStale). Each
view's run is governed by room() on its own estimate (noRoom); a failure, a median run over SIDE_BUDGET_S, or SIDE_OVERRUNS
of the last TIME_WINDOW runs pushing the step past STEP_BUDGET_S turn them off for the drive (sideViewStatus disabled,
which turnpilotd says once, sideCamerasOff), and both cycles fall back to the forward views. While a sweep runs, the lane
paint isn't read (lanePaintStatus sweep), the light reader reads colour only with a signal mapped within SWEEP_SIGNAL_M
(lightStatus sweep otherwise), every forward run publishes its class-9 boxes (lightBoxes), and the far crop is suspended
(farStatus sweep). Every message says what the side views are doing (sideViewStatus).

Stop signs (COCO 11), on every forward run under TurnPilotCameraStopSigns shadow (the default; off reads none): decoded
apart into the message's signs, each with the luma below it for plaque labelling (decoder.fill_signs), never into its
detections, so nothing stops, caps or releases on one.

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
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from openpilot.cereal import custom, log, messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.turnpilot.detector.decoder import (decode, decode_far, decode_far_lights, decode_lights, decode_side,
                                                           decode_signs, fill_detections, fill_far, fill_signs)
from openpilot.selfdrive.turnpilot.detector.graph import (SIDE_INPUT_SIZE, Crop, crop_at, crop_views, detector_pkl_path,
                                                         far_detector_pkl_path, letterbox, side_detector_pkl_path)
from openpilot.selfdrive.turnpilot.detector.light_state import LIGHT_MODE_DEFAULT, LightHead, LightReader, camera_lights_mode
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LanePaintClassifier, LinePaint, fill_lane_paint
from openpilot.selfdrive.turnpilot.detector.sideview import (RECALIB_DEG, SIDE_VIEWS, SectorQuality, SideGeometry, SideMap,
                                                             calib_moved_deg, compute_geometry)
from openpilot.selfdrive.turnpilot.scene.ranging import Lens, calib_from_view, camera_height, road_lens, wide_lens
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
# the stop sweep (SideViews). Chestnut, 2026-10-28, yolo26n on the 768x512 side view alone: 7.0 ms (the full pass 5.9 ms alone,
# 7.9 ms beside the big model)
SWEEPS = ('stopSweep', 'stopSweepGo')
SIDE_VIEW_V_MAX = 2.0      # m/s (STOP_GO_V_MAX): side views only at or under this, inside a sweep
SIDE_BUDGET_S = 0.015      # a median side-view run over this (the odd step's room beside the driving model): off for the drive
SIDE_OVERRUNS = 3          # of the last TIME_WINDOW side runs pushing the step past STEP_BUDGET_S: off for the drive
SIDE_STATUS_S = 0.5        # sideViewStatus noRoom: the gate open this long with no side run
SWEEP_SIGNAL_M = 60.       # during a sweep the light reader reads colour only with a signal mapped within this
STOP_SIGNS_DEFAULT = 'shadow'  # TurnPilotCameraStopSigns: off, or shadow (logged, never acted on)
LanePaintStatus = custom.TurnpilotDetections.LanePaintStatus
LightStatus = custom.TurnpilotDetections.LightStatus
FarStatus = custom.TurnpilotDetections.FarStatus
SideViewStatus = custom.TurnpilotDetections.SideViewStatus


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


def camera_stop_signs_mode(params) -> str:
  """TurnPilotCameraStopSigns: off, or shadow (the default, and anything else: nothing acts on a stop sign seen this week)."""
  try:
    value = params.get("TurnPilotCameraStopSigns")
  except UnknownKeyName:
    cloudlog.error("turnpilot: TurnPilotCameraStopSigns unknown to this build's params, stop signs in shadow; run scons")
    return STOP_SIGNS_DEFAULT
  value = value.decode() if isinstance(value, bytes) else value
  return 'off' if value == 'off' else STOP_SIGNS_DEFAULT


def plan_inputs(plan) -> tuple[str, float, bool]:
  """What modeld takes from fresh_plan's turnpilotPlan (shadow too): the detector's view, the far crop's bearing, and whether
  a signal is mapped within SWEEP_SIGNAL_M (a sweep's light reader reads then)."""
  if plan is None:
    return 'auto', 0., False
  mc = plan.mapControl
  return str(plan.detectorView), float(plan.farCropBearing), str(mc.kind) == 'signal' and 0. <= mc.distance <= SWEEP_SIGNAL_M


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


def upload_maps(name: str, side_map: SideMap) -> tuple:
  """A side view's maps on the device, as its graph takes them (luma_idx, chroma_idx, weights)."""
  from tinygrad import Tensor
  return tuple(Tensor(a).realize() for a in (side_map.luma_idx, side_map.chroma_idx, side_map.weights))


class SideViews:
  """The stop sweep's side views (module docstring): their compiled graph run(input_frame=, luma_idx=, chroma_idx=,
  weights=), the wide frame's device view, their maps for the bus's calibration (geometry and its device copies),
  recomputed off the step by the executor, each view's own time estimate, and their budget for the drive."""
  def __init__(self, run, cam_w: int, cam_h: int, frame, sm=None, executor=None, upload=None):
    self.run, self.cam_w, self.cam_h, self.frame, self.sm = run, cam_w, cam_h, frame, sm
    self.executor = executor
    self.upload = upload or upload_maps
    self.lens: Lens | None = None
    self.times: dict[str, deque] = {name: deque(maxlen=TIME_WINDOW) for name in SIDE_VIEWS}  # each view's estimate
    self.recent: deque[float] = deque(maxlen=TIME_WINDOW)    # every side run's time, for SIDE_BUDGET_S
    self.overruns: deque[bool] = deque(maxlen=TIME_WINDOW)
    self.quality_times: deque[float] = deque(maxlen=TIME_WINDOW)
    self.warmup_s = 0.              # the load's warm-up runs' time, the estimate before a view's first run
    self.disabled = False           # failed or over budget: off for the drive
    self.geometry: SideGeometry | None = None  # the maps on the device are for this calibration
    self.device: dict[str, tuple] = {}
    self.pending = None             # (future, rpy, wide euler) of a recompute under way
    self.staged: dict[str, tuple] = {}  # its views' maps uploaded so far, one an odd step
    self.why = SideViewStatus.noCalibration  # why the maps aren't ready
    self.next = 0                   # the side the next odd step runs (SIDE_VIEWS index)
    self.last_run_t: float | None = None  # time.monotonic() of the last side run
    self.started_t: float | None = None   # when the speed gate last opened
    self.quality: SectorQuality | None = SectorQuality()  # None: failed, off for the drive (no sectors: none ok)

  def calibration(self) -> tuple[list, list] | None:
    """(rpyCalib, wideFromDeviceEuler) when calibrated (and the wide lens, once the device is known), else None."""
    sm = self.sm
    sm.update(0)
    if self.lens is None:
      if not (sm.seen['deviceState'] and sm.seen['narrowRoadCameraState']):
        return None
      device_type = str(sm['deviceState'].deviceType)
      self.lens = wide_lens(DEVICE_CAMERAS[(device_type, str(sm['narrowRoadCameraState'].sensor))].wide_road, device_type)
      cloudlog.info(f"turnpilot side views: wide lens {self.lens.spec}")
    cal = sm['extrinsicsCalibration']
    if not (sm.seen['extrinsicsCalibration'] and cal.calStatus == log.ExtrinsicsCalibration.Status.calibrated):
      return None
    if len(cal.rpyCalib) != 3 or len(cal.wideFromDeviceEuler) != 3:
      return None
    return list(cal.rpyCalib), list(cal.wideFromDeviceEuler)

  def prepare(self, upload: bool = True) -> bool:
    """Whether the maps on the device are for the calibration (within RECALIB_DEG). If not, the views are invalid at once,
    their recompute runs off the step, and once it finishes they are uploaded one view a step (upload: this step may; it
    then runs no view; both views' maps at once took 27 ms on the chestnut, more than an odd step's room). why says what
    they wait for."""
    cal = self.calibration()
    if cal is None or self.lens is None:
      self.why = SideViewStatus.noCalibration
      return False
    rpy, euler = cal
    geom = self.geometry
    if geom is not None:
      moved = calib_moved_deg(geom.rpy, geom.wide_euler, rpy, euler)
      if moved <= RECALIB_DEG and geom.lens == self.lens:
        return True
      cloudlog.warning(f"turnpilot side views: calibration moved {moved:.2f} deg, invalid until their maps are recomputed")
      self.geometry, self.device = None, {}
    self.why = SideViewStatus.mapsStale
    if self.pending is not None and calib_moved_deg(self.pending[1], self.pending[2], rpy, euler) > RECALIB_DEG:
      self.pending, self.staged = None, {}  # for a calibration since left behind: never used
    if self.pending is None:
      if self.executor is None:
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='side_maps')
      self.pending = (self.executor.submit(compute_geometry, self.lens, rpy, euler, self.cam_w, self.cam_h), rpy, euler)
    future = self.pending[0]
    if upload and future.done():
      geom = future.result()  # a failed recompute raises: the side views' failure, off for the drive
      name = next(n for n in SIDE_VIEWS if n not in self.staged)
      t0 = time.perf_counter()
      self.staged[name] = self.upload(name, geom.maps[name])
      rpy_deg = np.round(np.degrees(geom.rpy), 2).tolist()
      cloudlog.warning(f"turnpilot side views: {name}'s maps for rpy {rpy_deg} deg on the device in {1e3 * (time.perf_counter() - t0):.1f} ms")
      if len(self.staged) == len(SIDE_VIEWS):
        self.pending, self.device, self.staged, self.geometry = None, self.staged, {}, geom
    return False

  def estimate(self, name: str) -> float:
    est = float(np.median(self.times[name])) if self.times[name] else self.warmup_s
    if self.quality is not None and self.quality_times:
      est += float(np.median(self.quality_times))
    return est

  def gate(self, open_: bool, now: float) -> None:
    """The speed gate this step: when it opens, noRoom's clock starts."""
    if not open_:
      self.started_t = None
    elif self.started_t is None:
      self.started_t = now

  def status(self, now: float, sweep: bool, gated: bool) -> int:
    if self.disabled:
      return SideViewStatus.disabled
    if not sweep:
      return SideViewStatus.idle
    if not gated:
      return SideViewStatus.speedGated
    if self.geometry is None:
      return self.why
    last = max(t for t in (self.last_run_t, self.started_t, -np.inf) if t is not None)
    return SideViewStatus.running if now - last <= SIDE_STATUS_S else SideViewStatus.noRoom

  def disable(self, why: str) -> None:
    if not self.disabled:
      cloudlog.error(f"turnpilot side views {why}, off for this drive: stop sweeps run the full wide pass and the road camera")
    self.disabled = True

  def ran(self, name: str, now: float, dt: float, overran: bool) -> None:
    """A run of view name, dt s at now; overran: with the model's time and its sectors it pushed the step past the budget."""
    self.last_run_t = now
    self.times[name].append(dt)
    self.recent.append(dt)
    self.overruns.append(overran)
    if len(self.recent) == TIME_WINDOW and float(np.median(self.recent)) > SIDE_BUDGET_S:
      self.disable(f"over budget ({np.median(self.recent) * 1e3:.1f} ms)")
    elif sum(self.overruns) >= SIDE_OVERRUNS:
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
  side: SideViews | None = None   # the side views (None: no wide camera, no side pkl, or it failed to load)
  signs_mode = STOP_SIGNS_DEFAULT  # TurnPilotCameraStopSigns, set by DetectorHost
  sweep: str | None = None        # the stop sweep running (SWEEPS), or None
  sweep_runs = 0                  # forward runs in this sweep, for the held cycle's full wide and road alternation
  signal_near = False             # a signal mapped within SWEEP_SIGNAL_M (plan_inputs), this step

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
    bus = frame_bus()
    self.lane_paint = LanePaintReader(cam_w, cam_h, bus) if 'road' in cameras else None
    self.lights = CameraLightReader(bus) if 'road' in cameras else None
    self.far = load_far(model_state, cam_w, cam_h, cameras['road'], bus) if 'road' in cameras else None
    self.side = load_side(model_state, cam_w, cam_h, cameras['wide'], bus) if 'wide' in cameras else None


  def choose_camera(self, desired_curvature: float, desire: int, now: float, view: str = 'auto') -> str:
    """The camera for this run: the wide one in a turn or on a curve and WIDE_HOLD_S after, else the road one; the plan's
    view (WIDE_EVERY, the sweeps) overrides that. A device with one camera runs it."""
    if abs(desired_curvature) >= WIDE_CURVATURE or desire in TURN_DESIRES:
      self.wide_until = now + WIDE_HOLD_S
    if view == 'stopSweep':  # held: the full wide pass and the road camera alternating, the wide first
      want = 'wide' if self.sweep_runs % 2 == 0 else 'road'
      self.sweep_runs += 1
    elif view == 'stopSweepGo':  # crossing: the full wide pass every time
      want = 'wide'
    elif view == 'road':
      want = 'road'
    elif view == 'roadAndWide':
      self.view_runs = (self.view_runs + 1) % WIDE_EVERY
      want = 'wide' if self.view_runs == 0 else 'road'
    else:
      want = 'wide' if now < self.wide_until else 'road'
    return want if want in self.frames else next(iter(self.frames))

  def room(self, model_execution_time: float) -> bool:
    """Whether this frame has room for a run: the driving model's time plus the estimate (the detector's, and the lane
    paint read's and the lights' while they are on: a sweep reads no paint, and lights only near a mapped signal) fit the
    budget, or the detector has gone FORCE_RUN_FRAMES or FORCE_RUN_S without and runs to measure again."""
    estimate = float(np.median(self.times)) if self.times else 0.
    if self.sweep is None and self.lane_paint is not None and self.paint_times:
      estimate += float(np.median(self.paint_times))
    if self.lights_read() and self.lights is not None and self.light_mode != 'off' and self.light_times:
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
           view: str = 'auto', far_bearing: float = 0., v_ego: float | None = None, signal_near: bool = False) -> None:
    """Called once per model frame after modelV2 is sent, with the driving model this detector was loaded for.
    metas: FrameMeta of packed frames 0 and 1; view: turnpilotPlan.detectorView (choose_camera, the sweeps); far_bearing:
    turnpilotPlan.farCropBearing (FarCrop); v_ego: carState.vEgo for the side views' speed gate (None: unknown, closed);
    signal_near: a signal mapped within SWEEP_SIGNAL_M (plan_inputs), for the light reader during a sweep."""
    if model_state is not self.model_state:
      # DetectorHost picks the detector by model, so this is a bug: raise for the host to log and retry, never a quiet off
      raise ValueError("turnpilot detector stepped with a driving model it wasn't loaded for")
    now = time.monotonic()
    sweep = view if view in SWEEPS else None
    if sweep != self.sweep:
      self.sweep, self.sweep_runs = sweep, 0
      if self.side is not None:
        self.side.next = 0
    gated = sweep is not None and v_ego is not None and v_ego <= SIDE_VIEW_V_MAX
    if self.side is not None:
      self.side.gate(gated, now)
    self.since_run += 1
    if self.since_run < RUN_EVERY:  # a frame between the full runs: the sweep's side views, else the far crop
      if sweep is not None:
        self.side_step(pm, model_state, model_execution_time, metas, gated)
      else:
        self.far_step(pm, model_state, model_execution_time, metas, far_bearing)
      return
    if sweep is not None and self.side is not None and not self.side.disabled:
      self.side_ready(upload=False)  # the maps' recompute starts with the sweep, above the gate too; uploaded on an odd step
    self.signal_near = signal_near
    lights_read = self.lights_read()
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
      td = msg.turnpilotDetections
      fill_detections(td, dets, meta.frame_id, meta.timestamp_eof, cam, dt)
      self.times.append(dt)
      if cam == 'road' and sweep is None:
        self.read_lane_paint(td, model_state, metas, dets)
      elif cam == 'road':
        td.lanePaintStatus = LanePaintStatus.sweep if self.lane_paint is not None else LanePaintStatus.disabled
      else:
        td.lanePaintStatus = LanePaintStatus.wideCamera if 'road' in self.cameras else LanePaintStatus.disabled
      if cam == 'road' and lights_read:
        self.read_lights(td, model_state, metas, raw)
      elif cam == 'road':
        td.lightStatus = LightStatus.sweep if self.light_status() == LightStatus.read else self.light_status()
      else:
        td.lightStatus = self.light_status() if self.light_status() != LightStatus.read else LightStatus.wideCamera
      boxes = decode_lights(raw, self.lb)
      for o, (x0, y0, x1, y1, conf, cls) in zip(td.init('lightBoxes', len(boxes)), boxes, strict=True):
        o.cls, o.conf, o.x0, o.y0, o.x1, o.y1 = int(cls), float(conf), float(x0), float(y0), float(x1), float(y1)
      signs = decode_signs(raw, self.lb) if self.signs_mode != 'off' else ()
      if len(signs):
        fill_signs(td, signs, model_state.frames[self.cameras[cam]], self.lb.cam_w, self.lb.cam_h)
      td.farStatus = self.far_status()
      td.sideViewStatus = self.side_status(gated)
      pm.send('turnpilotDetections', msg)
      self.failures = 0
    except Exception:
      self.failures += 1
      delay = min(RETRY_S * 2 ** (self.failures - 1), RETRY_MAX_S)
      self.retry_at = time.monotonic() + delay
      cloudlog.exception(f"turnpilot detector failed ({self.failures} in a row), retrying in {delay:.0f} s")

  def lights_read(self) -> bool:
    """Whether a road run reads the lights' colour: always, but during a sweep only with a signal mapped near."""
    return self.sweep is None or self.signal_near

  def far_status(self) -> int:
    """farStatus for the full pass's messages: off with TurnPilotFarCrop off or no far crop, sweep while a stop sweep
    suspends it, else the far crop's own."""
    if self.far is None or (not self.far_on and not self.far.disabled):
      return FarStatus.off
    if self.sweep is not None and not self.far.disabled:
      return FarStatus.sweep
    return self.far.status(time.monotonic())

  def side_status(self, gated: bool) -> int:
    """sideViewStatus for every message: off with no side views, else theirs (SideViews.status)."""
    if self.side is None:
      return SideViewStatus.off
    return self.side.status(time.monotonic(), self.sweep is not None, gated)

  def side_ready(self, upload: bool) -> bool:
    """SideViews.prepare, its failures the side views' (off for the drive, loudly), never the full pass's."""
    side = self.side
    assert side is not None
    try:
      return side.prepare(upload)
    except Exception:
      cloudlog.exception("turnpilot side views: their maps failed")
      side.disable("failed (maps)")
      return False

  def side_step(self, pm, model_state, model_execution_time: float, metas: list, gated: bool) -> None:
    """A sweep's odd step: the next side view, while the speed gate is open, its maps are ready and it has room (module
    docstring). Its failures turn the side views off for the drive and touch nothing of the full pass's."""
    side = self.side
    if side is None or side.disabled or not gated or not self.side_ready(upload=True):
      return
    name = SIDE_VIEWS[side.next]
    if model_execution_time + side.estimate(name) > STEP_BUDGET_S:
      return  # stale, said once it outlasts SIDE_STATUS_S (noRoom)
    try:
      geom = side.geometry
      assert geom is not None
      luma, chroma, weights = side.device[name]
      t0 = time.perf_counter()
      raw = side.run(input_frame=side.frame, luma_idx=luma, chroma_idx=chroma, weights=weights).numpy()
      dt = time.perf_counter() - t0
      side.next = (side.next + 1) % len(SIDE_VIEWS)
      i = self.cameras['wide']
      meta = metas[i]
      msg = messaging.new_message('turnpilotDetections', valid=bool(np.isfinite(raw).all()))
      td = msg.turnpilotDetections
      fill_detections(td, decode_side(raw, geom, name), meta.frame_id, meta.timestamp_eof, 'wide', dt)
      td.view, td.lens = name, geom.lens.spec
      quality_dt = self.read_sectors(td, model_state.frames[i], meta, name)
      td.lanePaintStatus = LanePaintStatus.sweep if self.lane_paint is not None else LanePaintStatus.disabled
      td.lightStatus = self.light_status() if self.light_status() != LightStatus.read else LightStatus.wideCamera
      td.farStatus, td.sideViewStatus = self.far_status(), SideViewStatus.running
      pm.send('turnpilotDetections', msg)
      side.ran(name, time.monotonic(), dt, model_execution_time + dt + quality_dt > STEP_BUDGET_S)
    except Exception:
      cloudlog.exception("turnpilot side views failed")
      side.disable("failed")

  def read_sectors(self, td, frame: np.ndarray, meta, name: str) -> float:
    """td.sectorQuality and sectorTime for side view name from the wide frame's host copy. A failure turns only the sector
    quality off, for the drive (no sectors: none is ok). Returns the time it took."""
    side = self.side
    assert side is not None and side.geometry is not None
    if side.quality is None:
      return 0.
    t0 = time.perf_counter()
    try:
      sectors = side.quality.update(side.geometry, name, frame, meta.timestamp_eof * 1e-9)
    except Exception:
      cloudlog.exception("turnpilot side views: sector quality failed, off for this drive (no sector reads ok)")
      side.quality = None
      return time.perf_counter() - t0
    dt = time.perf_counter() - t0
    for o, (inner, outer, quality) in zip(td.init('sectorQuality', len(sectors)), sectors, strict=True):
      o.inner, o.outer, o.quality = float(inner), float(outer), quality
    td.sectorTime = dt
    side.quality_times.append(dt)
    return dt

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


def check_side_specs(art: dict, frame_size: int) -> None:
  """A side-view pkl's input_specs and metadata against what modeld gives it (compile_detector.side_input_specs): the wide
  frame's view and a SIDE_INPUT_SIZE view's maps, by shape and dtype. Raises ValueError on a mismatch."""
  from openpilot.selfdrive.turnpilot.detector.compile_detector import side_input_specs
  meta, specs = art['metadata'], art['input_specs']
  want = side_input_specs(*meta.get('camera_size', (0, 0)), '')
  got = {k: tuple(v[:2]) for k, v in specs.items()}
  if (meta.get('kind') != 'side' or tuple(meta.get('input_size', ())) != SIDE_INPUT_SIZE or
      got != {k: (tuple(v[0]), v[1]) for k, v in want.items()} or specs['input_frame'][0] != (frame_size,)):
    raise ValueError(f"side views compiled for {meta}, {specs}; modeld has frames of {frame_size} bytes and {SIDE_INPUT_SIZE} views")


def load_side(model_state, cam_w: int, cam_h: int, index: int, bus) -> SideViews | None:
  """The side views on the wide camera, packed frame `index`, or None: no side pkl for this device (logged), or it failed
  to load (logged); the forward views run either way, and a sweep then runs only them (sideViewStatus off)."""
  try:
    from tinygrad import Tensor
    from tinygrad.engine.realize import lower_and_compile
    from openpilot.selfdrive.modeld.helpers import load_oob
    path = side_detector_pkl_path(cam_w, cam_h, model_state.chestnut)
    if not path.is_file():
      cloudlog.warning(f"turnpilot side views: no {path.name}, stop sweeps run the full wide pass and the road camera only")
      return None
    art = load_oob(path, model_state.chestnut)
    check_side_specs(art, model_state.frame_copy_size)
    run = art['run']
    run.captured._linear = lower_and_compile(run.captured._linear)
    side = SideViews(run, cam_w, cam_h, frame_view(model_state, index), bus)
    n = SIDE_INPUT_SIZE[0] * SIDE_INPUT_SIZE[1]
    blank = (Tensor(np.zeros(n, np.int32)).realize(), Tensor(np.zeros(n, np.int32)).realize(), Tensor(np.zeros((3, n), np.uint8)).realize())
    times = []
    for _ in range(3):  # warm up, and a first time estimate
      t0 = time.perf_counter()
      run(input_frame=side.frame, luma_idx=blank[0], chroma_idx=blank[1], weights=blank[2]).numpy()
      times.append(time.perf_counter() - t0)
    side.warmup_s = float(np.median(times))
    cloudlog.warning(f"turnpilot side views loaded ({1e3 * side.warmup_s:.1f} ms warm)")
    return side
  except Exception:
    cloudlog.exception("turnpilot side views failed to load, stop sweeps run the full wide pass and the road camera only")
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
    self.signs_mode = STOP_SIGNS_DEFAULT  # TurnPilotCameraStopSigns, read with TurnPilotEnabled
    self.next_check = 0.0
    self.missing = None  # the driving model last reported as having no detector
    self.failures = 0    # host failures in a row

  def check_enabled(self, now: float) -> bool:
    if now >= self.next_check:
      self.next_check = now + PARAM_CHECK_S
      self.enabled = self.params.get_bool("TurnPilotEnabled")
      self.light_mode = camera_lights_mode(self.params)
      self.far_on = far_crop_on(self.params)
      self.signs_mode = camera_stop_signs_mode(self.params)
    return self.enabled

  def load(self) -> None:
    self.detectors = [(m, self.load_fn(m, *self.args)) for m in self.models]

  def step(self, pm, model_state, model_execution_time: float, desired_curvature: float, desire: int, metas: list,
           engaged: bool, view: str = 'auto', far_bearing: float = 0., v_ego: float | None = None, signal_near: bool = False) -> None:
    """Called once per model frame after modelV2 is sent, with the model that made it. view, far_bearing, signal_near:
    plan_inputs of turnpilotPlan; v_ego: carState.vEgo."""
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
      det.light_mode, det.far_on, det.signs_mode = self.light_mode, self.far_on, self.signs_mode
      det.step(pm, model_state, model_execution_time, desired_curvature, desire, metas, view, far_bearing, v_ego, signal_near)
      self.failures = 0
    except Exception:
      # never take modeld down: off until the next TurnPilotEnabled check, after a wait doubling from RETRY_S. turnpilotd
      # hands back to the driver if that outlasts its deadline
      self.failures += 1
      delay = min(RETRY_S * 2 ** (self.failures - 1), RETRY_MAX_S)
      cloudlog.exception(f"turnpilot detector host failed ({self.failures} in a row), retrying in {delay:.0f} s")
      self.enabled, self.next_check = False, time.monotonic() + delay
