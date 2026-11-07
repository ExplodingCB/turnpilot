#!/usr/bin/env python3
from collections.abc import Callable
import base64
import ctypes
import gc
from functools import cached_property
import os
os.environ['GMMU'] = '0' # for chestnut fast loading, noop for qcom
os.environ.setdefault('AM_POWER_LIMIT', '100')
from tinygrad.device import Buffer, Device
from tinygrad.dtype import DType, dtypes
from tinygrad.engine.realize import lower_and_compile
from tinygrad.tensor import Tensor
from tinygrad.helpers import round_up
from tinygrad.uop.ops import UOp
import math
import pickle
import threading
import time
import numpy as np
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.cereal.messaging import PubMaster, SubMaster
from openpilot.cereal.services import SERVICE_LIST
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.hardware.usb import cable_connected, usb_host_connected
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.turnpilot.hooks import active_plan, fresh_plan, plan_desire
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, should_stop, smooth_value, get_curvature_from_plan
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_driving_model_data, fill_pose_msg, PublishState
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.helpers import (MODELS_DIR, chestnut_present, chestnut_compiled, modeld_pkl_path, load_oob, reset_chestnut,
                                              wait_for_chestnut)

SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')

LAT_SMOOTH_SECONDS = 0.0
LONG_SMOOTH_SECONDS = 0.3
MIN_LAT_CONTROL_SPEED = 0.3
BIG_MODEL_TIMEOUT = 60

# TurnPilot: the desires a finished turn's clear takes out of the model's desire queue (hooks.plan_desire's clear)
TURN_DESIRES = [log.Desire.turnLeft, log.Desire.turnRight]
# TurnPilot: a frame whose work (frames in hand to the detector step's end) took longer than this is logged as a
# modeld.slow_frame event, at most one each SLOW_FRAME_LOG_S. 100 ms is two model frames: one past it drops a frame, and
# selfdrived's modeldLagging (frameDropPerc > 1) is a few of them close together (1e seg 1, 11-06: one 419 ms frame)
SLOW_FRAME_S = 0.1
SLOW_FRAME_LOG_S = 1.0


def note_big_model(params, kind, exc, took_s, retry, waited_s, wait_error) -> None:
  """TurnPilot diagnostics: one TurnPilotDiagEvents event for a big model failure, so the phone's report says why it is
  the small model. Called on the failure paths only; never raises."""
  try:
    from openpilot.selfdrive.turnpilot.diagnostics import note_big_model as note
    note(params, kind, exc, took_s, retry, waited_s, wait_error)
  except Exception:
    pass


def note_recovery(params, kind: str, why: str) -> None:
  """TurnPilot diagnostics: the mid-drive recovery's events (chestnut_recovery.py: a retry that failed, the big model back).
  Never raises."""
  try:
    from openpilot.selfdrive.turnpilot.diagnostics import note_big_model_event
    note_big_model_event(params, kind, why)
  except Exception:
    pass


def successor_heard() -> Callable[[], bool]:
  """Whether a successor modeld published modelV2 since the last call (its liveness, chestnut_recovery.STALL_S)."""
  sock = messaging.sub_sock('modelV2', conflate=True, timeout=0)
  return lambda: sock.receive(non_blocking=True) is not None


def clear_turn_desires(model) -> tuple[float, bool]:
  """TurnPilot: a finished turn's desires out of the model's desire queue (hooks.plan_desire's clear), timed and logged
  with the time it took; returns (seconds, ok): those seconds and whether the desires came out (the frame's slow_frame
  clear_ms, cleared and clear_failed). A failure is logged with its time and the pulse ages out of the queue as before.
  The time is on the log line because the first clear of 1e (11-06) froze modeld for ~370 ms and nothing said so but the
  modelV2 gap (ModelState.warm_desire_clear). ok is there because, through the 11-06 round, a clear that raised returned
  its time like one that worked, and the slow frame it caused logged cleared=True: a slow clear, not a failed one."""
  t0 = time.perf_counter()
  try:
    model.clear_desire_history(TURN_DESIRES)
  except Exception:
    took = time.perf_counter() - t0
    cloudlog.exception(f"turnpilot: desire clear failed after {took * 1e3:.0f} ms")  # the pulse ages out of the queue as before
    return took, False
  took = time.perf_counter() - t0
  cloudlog.info(f"turnpilot: turn desires cleared from the model's desire queue in {took * 1e3:.0f} ms")
  return took, True


class SlowFrameLog:
  """TurnPilot: a modeld.slow_frame event for a frame whose work took over SLOW_FRAME_S, at most one each
  SLOW_FRAME_LOG_S, so the next stall says where its time went instead of showing only as a modelV2 gap. In 1e seg 1
  (11-06) frame 1872 published 419 ms after its EOF on a 42.1 ms model run, seven frames were dropped, frameDropPerc
  reached 3.37 against selfdrived's > 1 and modeldLagging soft-disabled openpilot mid-left at 11 mph; only the timing of
  the 'turn desires cleared' line tied it to the clear.

  The frame's work runs from its frames in hand (the vipc receives done) to the detector step's end: the wait for
  camerad's next frame is not modeld's own time, and a hand-over to a successor (chestnut_recovery) is not a frame.
  Fields: ms (the work), model_ms (model.run), cleared (a turn-desire clear ran on this frame and took them out),
  clear_failed (one ran and raised: clear_turn_desires' ok False) and clear_ms (its time either way, 0 with no clear),
  detector_ms (DetectorHost.step, 0 when it didn't run), frame_id (the main camera's), unlogged (the slow frames the rate
  limit held back, all in the SLOW_FRAME_LOG_S after the last event) and since_last_s (the seconds since that event, None
  for the first): in the 1e replay the stall's event carried the 8 held back from the startup burst 40 s before, which
  without since_last_s reads as 8 more stalls at the clear. event: cloudlog.event, injectable for the tests."""

  def __init__(self, event: Callable | None = None):
    self.event = event if event is not None else cloudlog.event
    self.last_t: float | None = None  # time.monotonic() of the last event
    self.unlogged = 0

  def update(self, now: float, frame_s: float, model_s: float, clear: tuple[float, bool] | None, detector_s: float,
             frame_id: int) -> bool:
    """One frame's times in seconds, clear clear_turn_desires' (seconds, ok) (None: no clear on this frame); whether it
    was logged."""
    if frame_s <= SLOW_FRAME_S:
      return False
    if self.last_t is not None and now - self.last_t < SLOW_FRAME_LOG_S:
      self.unlogged += 1
      return False
    clear_s, ok = clear if clear is not None else (0., None)
    self.event('modeld.slow_frame', ms=round(frame_s * 1e3, 1), model_ms=round(model_s * 1e3, 1), cleared=ok is True,
               clear_failed=ok is False, clear_ms=round(clear_s * 1e3, 1), detector_ms=round(detector_s * 1e3, 1),
               frame_id=int(frame_id), unlogged=self.unlogged, since_last_s=None if self.last_t is None else round(now - self.last_t, 1))
    self.last_t, self.unlogged = now, 0
    return True


def load_detector(params, link, cam_w: int, cam_h: int, main_wide_camera: bool, use_extra_client: bool, models: list,
                  host_cls=None):
  """TurnPilot: YOLO26 after modelV2, on the frames already uploaded. The host loads it now or once TurnPilotEnabled is
  set mid-drive, for the small model too; with it unset or no pkl nothing runs, and None (an import failure) changes
  nothing. host_cls: DetectorHost, injectable for the tests.

  Then ChestnutLoading clears (modeld's own, never a successor's: its parent drives meanwhile), loaded or not: the
  detector's ~19.5 s load (far crop, side views, native schedule) comes before the first modelV2, so selfdrived's big
  model settling (bigModelLoading, then its 5 s warm-up) covers it. Cleared before it, as through 11-06, selfdrived saw
  modelV2 not alive and raised commIssue for those 15 s at every start (1e seg 0: 0+34.7 to 0+50.2; 1d the same)."""
  detector = None
  try:
    if host_cls is None:
      from openpilot.selfdrive.turnpilot.detector.modeld_detector import DetectorHost as host_cls
    host = host_cls(params, cam_w, cam_h, main_wide_camera, use_extra_client, models)
    if host.check_enabled(time.monotonic()):
      host.load()
    detector = host
  except Exception:
    cloudlog.exception("turnpilot detector failed to load")
  if link is None:
    params.put_bool("ChestnutLoading", False)
  return detector


def get_action_from_model(model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                          lat_action_t: float, long_action_t: float, v_ego: float) -> log.ModelDataV2.Action:
  if 'action' not in model_output:
    plan = model_output['plan'][0]
    desired_accel = get_accel_from_plan(plan[:,Plan.VELOCITY][:,0],
                                        plan[:,Plan.ACCELERATION][:,0],
                                        ModelConstants.T_IDXS,
                                        action_t=long_action_t)
    desired_curvature = get_curvature_from_plan(plan[:,Plan.T_FROM_CURRENT_EULER][:,2],
                                                plan[:,Plan.ORIENTATION_RATE][:,2],
                                                ModelConstants.T_IDXS,
                                                v_ego,
                                                lat_action_t)
  else:
    desired_accel = model_output['action'][0,1]
    desired_curvature = model_output['action'][0,0] / (max(1.0, v_ego))**2
  stop = should_stop(v_ego, desired_accel)
  desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)
  if v_ego > MIN_LAT_CONTROL_SPEED:
    desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, LAT_SMOOTH_SECONDS)
  else:
    desired_curvature = prev_action.desiredCurvature

  return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature),
                                desiredAcceleration=float(desired_accel),
                                shouldStop=bool(stop))


class ChestnutGpuState:
  # GPU metrics require modeld's GPU context
  def __init__(self, pm: PubMaster, big: bool):
    self.pm = pm
    self.big = big
    self.valid = True
    self.sends = 0
    self.metrics = {}

  @cached_property
  def power_limit(self) -> int:
    smu = Device["AMD"].iface.dev_impl.smu
    return smu._send_msg(smu.smu_mod.PPSMC_MSG_GetPptLimit, 0, read_back_arg=True, timeout=100)

  def send(self) -> None:
    msg = messaging.new_message('chestnutGpuState')
    state = msg.chestnutGpuState
    self.sends += 1
    if self.big and "AMD" in Device._opened_devices and self.sends % 100 == 1:
      try:
        smu = Device["AMD"].iface.dev_impl.smu
        metrics_t = smu.smu_mod.SmuMetricsExternal_t
        smu._send_msg(smu.smu_mod.PPSMC_MSG_TransferTableSmu2Dram, smu.smu_mod.TABLE_SMU_METRICS, timeout=100)
        metrics_buf = bytearray(smu.adev.vram.view(smu.driver_table_paddr, ctypes.sizeof(metrics_t))[:])
        metrics = metrics_t.from_buffer(metrics_buf).SmuMetrics
        self.metrics = {'tempC': metrics.AvgTemperature[smu.smu_mod.TEMP_HOTSPOT],
                        'memoryTempC': metrics.AvgTemperature[smu.smu_mod.TEMP_MEM],
                        'powerDrawW': metrics.AverageSocketPower,
                        'powerLimitW': self.power_limit,
                        'gpuUsagePercent': metrics.AverageGfxActivity,
                        'gpuClockMhz': metrics.AverageGfxclkFrequencyPostDs,
                        'fanSpeedRpm': metrics.AvgFanRpm}
        self.valid = True
      except Exception:
        if self.valid:
          cloudlog.exception("chestnut state read failed")
        self.valid = False
        self.metrics.clear()
    if self.big:
      for k, v in self.metrics.items():
        setattr(state, k, v)

    msg.valid = not self.big or (self.valid and bool(self.metrics))
    self.pm.send('chestnutGpuState', msg)


class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof


def input_view(buffer: Buffer, shape: tuple[int, ...], dtype: DType, offset: int) -> Tensor:
  view = buffer.view(math.prod(shape) * dtype.itemsize, offset).ensure_allocated()
  return Tensor(UOp.from_buffer(view, dtype)).reshape(shape)


class ModelState:
  prev_desire: np.ndarray  # for tracking the rising edge of the pulse

  def __init__(self, cam_w: int, cam_h: int, chestnut: bool):
    jits = load_oob(modeld_pkl_path(chestnut), chestnut)
    self.model_device = jits['input_specs']['new_img'][2]
    self.input_shapes = {name: (shape, np.dtype(dtype)) for name, (shape, dtype, _) in jits['input_specs'].items()}
    self.state_pairs = {name: f'next_{name}' for name in self.input_shapes if f'next_{name}' in jits['metadata']['output_shapes']}
    self.vision_input_names = ('img', 'big_img')
    self.output_slices = pickle.loads(base64.b64decode(jits['metadata']['metadata']['output_slices']))

    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    self.chestnut = chestnut

    stride, y_height, uv_height, _ = get_nv12_info(cam_w, cam_h)
    self.frame_copy_size = stride * (y_height + uv_height)
    self.pack_inputs()
    with open(MODELS_DIR / f'{"big_" if chestnut else ""}driving_warp_{cam_w}x{cam_h}_tinygrad.pkl', 'rb') as f:
      self.run_warp = pickle.load(f)['run']
    self.run_warp.captured._linear = lower_and_compile(self.run_warp.captured._linear)
    self.run_model = jits['run']
    self.run_model.captured._linear = lower_and_compile(self.run_model.captured._linear)
    self.outputs = {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize() for name, (shape, dtype, device) in jits['output_specs'].items()}
    for name, next_name in self.state_pairs.items():
      state = self.input_queues[name]
      self.outputs[next_name] = input_view(state._buffer(), state.shape, state.dtype, 0)
    self.parser = Parser()

  def pack_inputs(self) -> None:
    # Pack host inputs into one upload to reduce USB transfer overhead for the eGPU.
    self.input_queues = {name: Tensor(np.zeros(shape, dtype=dtype), device=self.model_device).realize()
                         for name, (shape, dtype) in self.input_shapes.items() if name in self.state_pairs}
    shapes = {'tfm': (2, 3, 3)} | {name: shape for name, (shape, _) in self.input_shapes.items()
                                   if name not in self.state_pairs and name != 'new_img'}
    npy_size = sum(round_up(math.prod(shape) * 4, 128) for shape in shapes.values())
    self.packed_input = np.zeros(npy_size + 2 * self.frame_copy_size, dtype=np.uint8)
    self.input_host = Tensor(self.packed_input, device='NPY')._buffer()
    self.input_device = Tensor(self.packed_input, device=self.model_device)._buffer()
    self.npy = {}
    offset = 0
    for name, shape in shapes.items():
      self.npy[name] = np.ndarray(shape, dtype=np.float32, buffer=self.packed_input, offset=offset)
      self.input_queues[name] = input_view(self.input_device, shape, dtypes.float32, offset)
      offset += round_up(self.npy[name].nbytes, 128)
    self.frames = self.packed_input[npy_size:].reshape(2, self.frame_copy_size)
    self.warp_inputs = {'input_frame': input_view(self.input_device, self.frames.shape, dtypes.uint8, npy_size), 'M_inv': self.input_queues.pop('tfm')}

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    return {k: model_outputs[np.newaxis, v] for k,v in output_slices.items()}

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
          inputs: dict[str, np.ndarray], after_enqueue: Callable[[], None] | None = None) -> dict[str, np.ndarray]:
    for i, key in enumerate(self.vision_input_names):
      np.copyto(self.frames[i], np.frombuffer(bufs[key].data, dtype=np.uint8, count=self.frame_copy_size))
      self.npy['tfm'][i] = transforms[key]

    # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
    inputs['desire_pulse'][0] = 0
    self.npy['desire'][:] = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']
    self.npy['traffic_convention'][:] = inputs['traffic_convention']
    self.npy['action_t'][:] = inputs['action_t']

    self.input_device.copy_from(self.input_host)
    self.input_queues['new_img'] = self.run_warp(**self.warp_inputs)
    self.run_model(output_buffers=self.outputs, **self.input_queues)
    if self.chestnut and (err := Device[self.outputs['outputs'].device].error_state.host.view(fmt='q')[0]):
      raise RuntimeError(f"chestnut failed with {err}")
    if after_enqueue is not None:
      after_enqueue()
    model_output = self.outputs['outputs'].numpy()[0]
    if self.chestnut and not np.all(np.isfinite(model_output)):
      raise RuntimeError("model output not finite")
    outputs_dict = self.parser.parse_outputs(self.slice_outputs(model_output, self.output_slices))

    if SEND_RAW_PRED:
      outputs_dict['raw_pred'] = model_output.copy()
    return outputs_dict

  def clear_desire_history(self, desires: list[int]) -> None:
    """TurnPilot: drop these desires from the model's desire queue (the rising edges of its last frames)."""
    queue = self.input_queues.get('state_desire_q')
    if queue is None:
      return
    buf = queue._buffer()
    q = np.frombuffer(buf.as_memoryview(), dtype=np.float32).reshape(queue.shape).copy()
    q[..., desires] = 0
    buf.copy_from(Tensor(q, device='NPY')._buffer())
    self.prev_desire[desires] = 0

  def warm_desire_clear(self) -> None:
    """TurnPilot: one clear_desire_history of both turn desires at load, so the frame loop's first clear doesn't pay
    tinygrad's first use of its read-back and copy-in (lowering and compiling them for this device) mid-drive.

    1e seg 1 (11-06), the first clear since modeld started: modelV2 frame 1871 at 1+33.27, the 'turn desires cleared'
    line at 1+33.66, frame 1872 at 1+33.67 published 419 ms after its EOF on a 42.1 ms model run, then frame 1880. Seven
    frames dropped, frameDropPerc 3.37 against selfdrived's > 1, and modeldLagging soft-disabled openpilot ('TAKE CONTROL
    IMMEDIATELY') mid-left at 11 mph, 12 s into the night's first engagement. The three later clears that drive cost at
    most ~80 ms of publish latency against a p99 of ~75 ms: the cost is a first use, not the copy, so one call here takes
    it. Each real clear makes new host buffers and only the first was slow, so what that first use builds isn't tied to
    them: a call on the same queue and device takes it.

    Called on a zeroed queue (the end of warmup(), or a model fresh from its constructor): a clear of zeros is zeros, so
    it changes no state.

    A failure on the chestnut is sorted by what the chestnut does next, not by its exception: warmup()'s dummy frame runs
    again (frame_fault), the frame that just ran on the same inputs. If that raises too, the big model can't run its frame
    (a fault, or state the failure left behind) and the clear's exception raises out of warmup(), so the load fails as a
    load: load_big's 'big model load failed' and note_big_model 'failed to load' (the clear's
    exception), then the startup ChestnutRetry onroad cycle from a freshly reset GPU; a successor's link.failed('load:
    ...'). Swallowed, as when this warm call went in (11-06), warmup() returned, ChestnutActive went True and the fault
    came out on the first frame as 'failed while running' instead (review, 11-06). If the frame runs, the chestnut works:
    the failure is logged, the state zeroed again (the probe frame dirtied it, and a copy-in the failure cut short may
    have too) and the load goes on; the first real clear pays its first use as before, and if it fails too the pulse ages
    out. A fault the GPU came back from (AMD's on_device_hang resets it before it raises 'Device hang detected') goes on
    the same way: the chestnut ran a frame after it, as the frame loop's first frame would.
    The exception can't sort it. Not by the error word: on_device_hang zeroes it in that reset. Not by type: tinygrad
    raises its own code errors as RuntimeError like its device faults (tensor.py's 'assign dtype mismatch' and "can't
    create Tensor from", uop/ops.py's checks, device.py's 'buffer of size ... is too large', beside device.py's 'signal
    wait timed out' and 'failed with N' and support/usb.py's libusb errors). The type rule that first fixed the swallow
    (11-06) failed the load on any RuntimeError, MemoryError or OSError, so a deterministic code bug in the clear (a
    tinygrad bump, a change to the desire queue) cost the big model at every start, its retry and every successor, where
    swallowed it costs a pulse ageing out at each turn (review, 11-06 round 2). The probe runs on this failure path only:
    one frame, or on a wedged chestnut up to AMDDevice.wait_timeout_ms (main()'s 3 s) before it raises, after the clear's
    own 3 s; 11-06's loads took 29.1 s and 32.8 s, so ~39 s at worst against BIG_MODEL_TIMEOUT's 60.

    On the small model anything is logged and swallowed, with no probe: main() calls this outside any load-failure path,
    and comma's model stays the fallback; a real fault on comma's GPU comes out on its first frame (main()'s 'if not
    model.chestnut: raise')."""
    t0 = time.perf_counter()
    try:
      self.clear_desire_history(TURN_DESIRES)
    except Exception as e:
      if not self.chestnut:
        cloudlog.exception("turnpilot: desire clear warm-up failed, the first turn's clear pays its first use")
        return
      if (fault := self.frame_fault()) is not None:
        failed = f"{type(e).__name__}: {e}"
        cloudlog.error(f"turnpilot: the chestnut faulted in the desire clear warm-up ({failed}) and a frame after it ({fault}), the big model's load fails")
        raise
      went_on = "but the chestnut ran a frame after it, so the load goes on; the first turn's clear pays its first use"
      cloudlog.exception(f"turnpilot: desire clear warm-up failed on the big model ({type(e).__name__}) {went_on}")
      self.zero_state()
      return
    took_ms = (time.perf_counter() - t0) * 1e3
    cloudlog.info(f"turnpilot: desire clear warmed on the {'big' if self.chestnut else 'small'} model in {took_ms:.0f} ms")

  def frame_fault(self) -> str | None:
    """TurnPilot: warm_desire_clear's probe of the chestnut after a failure, warmup()'s dummy frame again: None when it
    ran, else what it raised ('Type: message'). Its state is the caller's to zero."""
    try:
      self.run_dummy_frame()
    except Exception as e:
      return f"{type(e).__name__}: {e}"
    return None

  def run_dummy_frame(self) -> None:
    """warmup()'s frame: zero frames, identity transforms, zero inputs (TurnPilot: shared with frame_fault)."""
    dummy_frames = {k: np.zeros(self.frame_copy_size, dtype=np.uint8) for k in self.vision_input_names}
    eye = np.eye(3, dtype=np.float32)
    dims = {'desire_pulse': ModelConstants.DESIRE_LEN, 'traffic_convention': 2, 'action_t': 2}
    self.run(dummy_frames, dict.fromkeys(self.vision_input_names, eye), {k: np.zeros(v, dtype=np.float32) for k, v in dims.items()})

  def zero_state(self) -> None:
    """warmup()'s reset after its frame: the packed inputs, the state queues and prev_desire (TurnPilot: shared with
    warm_desire_clear, after frame_fault's frame)."""
    self.packed_input[:] = 0
    for key in self.state_pairs:
      self.input_queues[key].assign(0).realize()
    self.prev_desire[:] = 0

  def warmup(self) -> None:
    self.run_dummy_frame()
    self.zero_state()
    # TurnPilot: the turn-desire clear's first use, on the zeroed queue (warm_desire_clear); a successor's warmup too. A
    # chestnut that fails it and then a frame raises out of here: a load failure, as the dummy run's would be
    self.warm_desire_clear()


def main(demo=False, successor: str | None = None):
  """successor: TurnPilot's mid-drive recovery (chestnut_recovery.py), the pipe fds 'R,W' of a successor modeld, which
  loads the big model alone, runs nothing until its parent probes it and publishes nothing until its parent hands over."""
  cloudlog.warning("modeld init" + (" (successor)" if successor else ""))
  link = None
  if successor is not None:
    from openpilot.selfdrive.modeld.chestnut_recovery import SuccessorLink
    cloudlog.bind(daemon="modeld")
    link = SuccessorLink(successor)

  # a cable is a chestnut on its way, unless a USB host is on it
  CHESTNUT = link is not None or (chestnut_compiled() and (chestnut_present() or (cable_connected() and not usb_host_connected())))
  if CHESTNUT:
    from tinygrad.runtime.ops_amd import AMDDevice
    AMDDevice.wait_timeout_ms = 3000
  params = Params()
  if link is None:  # a successor's parent drives on the small model meanwhile, and says what the big model is doing
    params.put_bool("ChestnutLoading", CHESTNUT)
    params.remove("ChestnutActive")

  gc.disable()

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams and VisionStreamType.VISION_STREAM_NARROW_ROAD in available_streams
      main_wide_camera = VisionStreamType.VISION_STREAM_NARROW_ROAD not in available_streams
      break
    time.sleep(.1)

  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_NARROW_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True)
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False)
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  st = time.monotonic()
  cloudlog.warning("loading model")
  model = None
  retry = False  # TurnPilot diagnostics: this load is the onroad cycle's retry
  big_loaded = True  # TurnPilot: False once the big model failed its load on the onroad retry too (the recovery takes over)
  load: dict = {"waited_s": None, "wait_error": None}
  if link is not None:
    # the successor's load holds nothing up (its parent publishes meanwhile, and gives it LOAD_DEADLINE_S); a failure ends it
    try:
      wait_for_chestnut()
      model = ModelState(vipc_client_main.width, vipc_client_main.height, True)
      model.warmup()
    except Exception as e:
      cloudlog.exception("successor: big model load failed")
      link.failed(f"load: {type(e).__name__}: {e}")
      os._exit(1)  # no finalize: a chestnut that failed to load waits out its timeouts there
  elif CHESTNUT:
    big_model = None
    retry = params.get_bool("ChestnutRetry")
    def load_big():
      nonlocal big_model
      t0 = time.monotonic()
      try:
        try:
          wait_for_chestnut()
        except Exception as e:
          load["wait_error"] = type(e).__name__
          raise
        load["waited_s"] = time.monotonic() - t0
        m = ModelState(vipc_client_main.width, vipc_client_main.height, True)
        m.warmup()
        big_model = m
      except Exception as e:
        cloudlog.exception("big model load failed")
        if not loader_timed_out.is_set():  # TurnPilot diagnostics: after the timeout, that event says it
          note_big_model(params, "failed to load", e, time.monotonic() - t0, retry, load["waited_s"], load["wait_error"])
    loader_timed_out = threading.Event()
    loader = threading.Thread(target=load_big, daemon=True)
    loader.start()
    loader.join(BIG_MODEL_TIMEOUT)
    model = big_model
    if model is None and loader.is_alive():
      loader_timed_out.set()
      note_big_model(params, "timed out", None, BIG_MODEL_TIMEOUT, retry, load["waited_s"], load["wait_error"])
    if model is None and not params.get_bool("ChestnutRetry"):
      # a crank can cut the chestnut power during the load, retry once with an onroad cycle. TurnPilot: from a freshly
      # powered GPU (helpers.reset_chestnut: a Tesla never cuts the chestnut's 12 V while occupied)
      cloudlog.error(f"big model load failed, cycling onroad to retry; {reset_chestnut()}")
      params.put_bool("ChestnutRetry", True)
      params.put_bool("OnroadCycleRequested", True)
    else:
      params.put_bool("ChestnutActive", model is not None)
      big_loaded = model is not None

  small_model = ModelState(vipc_client_main.width, vipc_client_main.height, False) if link is None and (model is None or CHESTNUT) else None
  if small_model is not None:
    # TurnPilot: the small model is never warmup()'d (comma's), so its clear's first use is taken here, on its fresh zeroed
    # queue: the model without a chestnut, and the fallback's first turn after a big model failure
    small_model.warm_desire_clear()
  if model is None:
    model = small_model
  cloudlog.warning(f"models loaded in {time.monotonic() - st:.1f}s, modeld starting")
  loaded_at = time.monotonic()

  # TurnPilot: the detector, then ChestnutLoading cleared (load_detector: selfdrived's big model settling covers the load)
  detector = load_detector(params, link, vipc_client_main.width, vipc_client_main.height, main_wide_camera, use_extra_client,
                           [model, small_model])

  if link is None:  # a successor takes modeld's realtime slot only when it goes live, off its parent's core until then
    config_realtime_process(7, 54)

  # messaging
  pub_socks = ["modelV2", "drivingModelData", "cameraOdometry"] + (["chestnutGpuState"] if CHESTNUT else [])
  pub_socks += ["turnpilotDetections", "turnpilotFarDetections"] if detector is not None else []
  pm = PubMaster(pub_socks if link is None else [])  # one publisher a topic: a successor's sockets open when it goes live

  # TurnPilot: the big model back mid-drive after a runtime failure, in a successor modeld (chestnut_recovery.py)
  recovery = None
  if CHESTNUT and link is None:
    import atexit
    import sys
    from openpilot.selfdrive.modeld.chestnut_recovery import ChestnutRecovery, Successor
    cmd = [sys.executable, '-m', 'openpilot.selfdrive.modeld.modeld'] + (['--demo'] if demo else []) + ['--successor']
    recovery = ChestnutRecovery(spawn=lambda: Successor.spawn(cmd), present=chestnut_present,
                                note=lambda kind, why: note_recovery(params, kind, why), reset=reset_chestnut)
    atexit.register(recovery.close)
    if not big_loaded:
      # the load failed on its onroad retry too: the successor keeps trying in the background (each attempt from a power
      # cycled GPU) instead of the small model for the rest of the drive
      recovery.failed(time.monotonic(), why='the big model failed to load at start')
  sm = SubMaster(["deviceState", "carState", "narrowRoadCameraState", "extrinsicsCalibration", "driverMonitoringState",
                  "carControl", "controlsState", "lateralDelay", "turnpilotPlan"],
                 ignore_alive=["turnpilotPlan"], ignore_avg_freq=["turnpilotPlan"], ignore_valid=["turnpilotPlan"])

  publish_state = PublishState()
  params = Params()
  chestnut_state = ChestnutGpuState(pm, model.chestnut) if CHESTNUT else None

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_RUN_FREQ)
  frame_id = 0
  last_vipc_frame_id = 0
  run_count = 0

  model_transform_main = np.zeros((3, 3), dtype=np.float32)
  model_transform_extra = np.zeros((3, 3), dtype=np.float32)
  extrinsics_calibration_seen = False
  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()

  if demo:
    CP = get_demo_car_params()
  else:
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("modeld got CarParams: %s", CP.brand)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  # TODO Move smooth seconds to action function
  long_delay = CP.longitudinalActuatorDelay + LONG_SMOOTH_SECONDS
  prev_action = log.ModelDataV2.Action()

  DH = DesireHelper()
  desire_cleared = False
  slow_frames = SlowFrameLog()
  if link is not None:
    link.loaded()

  while True:
    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_client_main.recv()
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        break

    if buf_main is None:
      cloudlog.debug("vipc_client_main no frame")
      continue

    if use_extra_client:
      # Keep receiving extra frames until frame id matches main camera
      while True:
        buf_extra = vipc_client_extra.recv()
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break

      if buf_extra is None:
        cloudlog.debug("vipc_client_extra no frame")
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000:
        cloudlog.error(f"frames out of sync! main: {meta_main.frame_id} ({meta_main.timestamp_sof / 1e9:.5f}),\
                         extra: {meta_extra.frame_id} ({meta_extra.timestamp_sof / 1e9:.5f})")

    else:
      # Use single camera
      buf_extra = buf_main
      meta_extra = meta_main

    frame_t0 = time.perf_counter()  # TurnPilot: this frame's work starts with its frames in hand (SlowFrameLog)
    sm.update(0)
    desire = DH.desire
    # TurnPilot: nav turn desires go in while DesireHelper is idle (a nav lane change goes through DesireHelper) and
    # openpilot is steering, and a finished turn's desire comes out of the model's desire queue, timed (clear_turn_desires)
    tp = active_plan(sm)
    tp_desire, clear = plan_desire(tp.desire, sm['carControl'].latActive) if tp is not None else (log.Desire.none, False)
    clear_result = clear_turn_desires(model) if clear and not desire_cleared else None  # (seconds, ok), None: no clear
    desire_cleared = clear
    if desire == log.Desire.none:
      desire = tp_desire
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["narrowRoadCameraState"].frameId
    v_ego = max(sm["carState"].vEgo, 0.)
    lat_delay = sm["lateralDelay"].lateralDelay + LAT_SMOOTH_SECONDS
    if sm.updated["extrinsicsCalibration"] and sm.seen['narrowRoadCameraState'] and sm.seen['deviceState']:
      device_from_calib_euler = np.array(sm["extrinsicsCalibration"].rpyCalib, dtype=np.float32)
      dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))]
      main_intrinsics = dc.wide_road.intrinsics if main_wide_camera else dc.narrow_road.intrinsics
      model_transform_main = get_warp_matrix(device_from_calib_euler, main_intrinsics, False).astype(np.float32)
      has_wide_camera = use_extra_client or main_wide_camera
      extra_intrinsics = dc.wide_road.intrinsics if has_wide_camera else dc.narrow_road.intrinsics
      model_transform_extra = get_warp_matrix(device_from_calib_euler, extra_intrinsics, True).astype(np.float32)
      extrinsics_calibration_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if desire >= 0 and desire < ModelConstants.DESIRE_LEN:
      vec_desire[desire] = 1

    # tracked dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count = run_count + 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)

    bufs = {name: buf_extra if 'big' in name else buf_main for name in model.vision_input_names}
    transforms = {name: model_transform_extra if 'big' in name else model_transform_main for name in model.vision_input_names}
    frame_delay = DT_MDL # compensate for time passed since the frame was captured: current_time - timestamp_eof is 50ms on average
    action_delay = DT_MDL / 2 # middle of the interval between model output (current state) and next frame (expected state)
    lat_action_t = lat_delay + frame_delay + action_delay
    long_action_t = long_delay + frame_delay + action_delay
    inputs: dict[str, np.ndarray] = {
      'desire_pulse': vec_desire,
      'traffic_convention': traffic_convention,
      'action_t': np.array([lat_action_t, long_action_t], dtype=np.float32),
    }

    # a successor runs once its parent probes it, and publishes once its parent has stopped (chestnut_recovery.SuccessorLink)
    if link is not None and link.poll():
      pm.sock.update({s: messaging.pub_sock(s) for s in pub_socks})
      config_realtime_process(7, 54)
      params.put_bool("ChestnutActive", True)
      run_count = 0
      cloudlog.warning("successor: live, the big model publishes")
    publishing = link is None or link.live

    mt1 = time.perf_counter()
    model_output = None
    try:
      if link is None or link.runs:
        send_chestnut = (publishing and chestnut_state is not None and
                         run_count % round(ModelConstants.MODEL_RUN_FREQ / SERVICE_LIST['chestnutGpuState'].frequency) == 0)
        model_output = model.run(bufs, transforms, inputs, chestnut_state.send if send_chestnut else None)
    except Exception as e:
      if link is not None:  # a successor has no small model: its parent publishes again at once, and notes why
        cloudlog.exception("successor: big model failed")
        link.failed(f"{'run' if link.live else 'probe'}: {type(e).__name__}: {e}")
        os._exit(1)  # at once: its parent waits on the exit, and a finalize would wait out the dead chestnut's timeouts
      if not model.chestnut:
        raise
      # fallback to small model, and the big model retried in the background (chestnut_recovery.py)
      cloudlog.exception("big model failed, fall back to small")
      note_big_model(params, "failed while running", e, time.monotonic() - loaded_at, retry, load["waited_s"], load["wait_error"])
      params.put_bool("ChestnutActive", False)
      if recovery is not None:
        recovery.failed(time.monotonic(), model)
      model = small_model
      if chestnut_state is not None:
        chestnut_state.big = False
      run_count = 0
      model_output = None
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1
    if link is not None and model_output is not None:
      link.ran()

    if model_output is not None and publishing:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')

      if not sm['carControl'].latActive:
        prev_action.desiredCurvature = sm['controlsState'].curvature
      action = get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego)
      prev_action = action
      fill_model_msg(modelv2_send, model_output, action,
                     publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id,
                     frame_drop_ratio, meta_main.timestamp_eof, model_execution_time, extrinsics_calibration_seen)
      modelv2_send.modelV2.big = model.chestnut

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob,
                tp.laneChange.raw if tp is not None else log.LaneChangeDirection.none,
                tp.turnSignal.side.raw if tp is not None else log.LaneChangeDirection.none)
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction

      fill_driving_model_data(drivingdata_send, modelv2_send)
      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, extrinsics_calibration_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('drivingModelData', drivingdata_send)
      pm.send('cameraOdometry', posenet_send)
    last_vipc_frame_id = meta_main.frame_id

    detector_s = 0.
    if detector is not None and model_output is not None and publishing:
      from openpilot.selfdrive.turnpilot.detector.modeld_detector import plan_inputs
      # the camera TurnPilot asks for (DESIGN phase 4 G4, the stop sweeps), where the far crop looks (G5) and whether a signal
      # is mapped near: read in shadow too, where it steers nothing
      view, far_bearing, signal_near = plan_inputs(fresh_plan(sm))
      detector_t0 = time.perf_counter()
      detector.step(pm, model, model_execution_time, prev_action.desiredCurvature, desire, [meta_main, meta_extra], sm['carControl'].enabled,
                    view, far_bearing, v_ego, signal_near)
      detector_s = time.perf_counter() - detector_t0

    # TurnPilot: a slow frame says where its time went (SlowFrameLog); a successor not yet live publishes nothing to lag
    if publishing:
      slow_frames.update(time.monotonic(), time.perf_counter() - frame_t0, model_execution_time, clear_result, detector_s,
                         meta_main.frame_id)

    # TurnPilot: the big model back. With openpilot disengaged and the successor sane on these frames, this process stops
    # publishing, and waits on the successor while it publishes; when it stops, this process publishes again on the small
    # model at once, and retries later
    if recovery is not None and recovery.active and recovery.step(time.monotonic(), sm['carControl'].enabled):
      pm.sock.clear()
      heard = successor_heard()
      recovery.go(time.monotonic())
      cloudlog.warning("big model: handed over to a successor modeld")
      why = recovery.standby(heard)
      recovery.failed(time.monotonic(), why=why)  # killed first if it stalled: one publisher a topic
      pm.sock.update({s: messaging.pub_sock(s) for s in pub_socks})
      params.put_bool("ChestnutActive", False)
      cloudlog.error(f"big model: {why}, the small model publishes again")
      prev_action, run_count = log.ModelDataV2.Action(), 0  # minutes stale: a fresh start, as at startup

if __name__ == "__main__":
  try:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
    parser.add_argument('--successor', help="TurnPilot's mid-drive recovery: a successor modeld's pipe fds, R,W (chestnut_recovery.py)")
    args = parser.parse_args()
    main(demo=args.demo, successor=args.successor)
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
