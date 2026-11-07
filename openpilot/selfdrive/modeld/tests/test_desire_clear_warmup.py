"""modeld's turn-desire clear (11-06, drive 1e seg 1): the first clear_desire_history since modeld started froze the
frame loop for ~370 ms (frame 1872 published 419 ms after its EOF on a 42.1 ms model run, seven frames dropped,
frameDropPerc 3.37 against selfdrived's > 1), and modeldLagging soft-disabled openpilot mid-left at 11 mph. Covered here:
warmup() takes that first use on its zeroed queue; a failure in it on the chestnut is sorted by the frame warmup() runs
after it (a chestnut that can't run it fails the big model's load as a load, through main() and in a successor; one that
runs it goes on, whatever the failure raised); the small model's warm call; the timed clear in the frame loop and its
(seconds, ok); the modeld.slow_frame event (cleared or clear_failed) and its rate limit; and ChestnutLoading cleared only
after the detector's load (selfdrived's big model settling covers it, no startup commIssue). The faults and code errors
are tinygrad's own, raised by its code on stand-ins for the device.

The cost itself is the chestnut's (AMD over USB): on a laptop's devices the first clear is already cheap, so the timing is
checked on the comma, by the bench at the end (TURNPILOT_CHESTNUT_BENCH=1) and on the car by the 'desire clear warmed ...
in N ms' and 'turn desires cleared ... in N ms' lines."""
import gc
import os
import time
from collections import defaultdict
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest
from tinygrad.device import Allocator, BufferSpec, Compiled, CompileError
from tinygrad.dtype import dtypes
from tinygrad.runtime.ops_amd import PCIIface
from tinygrad.runtime.support import usb
from tinygrad.runtime.support.system import FileIOInterface
from tinygrad.tensor import Tensor

from openpilot.cereal import log
from openpilot.selfdrive.modeld import modeld
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.modeld import ModelState, SlowFrameLog, clear_turn_desires, load_detector

TURNS = [log.Desire.turnLeft, log.Desire.turnRight]
QUEUE_SHAPE = (1, 25, ModelConstants.DESIRE_LEN)  # the shape is the test's: the clear works on any (..., DESIRE_LEN)


def bare_model(queue: np.ndarray | None = None, chestnut: bool = True) -> ModelState:
  """A ModelState with only what warmup() and the clear touch: the state desire queue on the CPU (with a second state
  queue beside it, which the clear must leave alone), the packed host inputs and prev_desire. run is patched per test."""
  m = ModelState.__new__(ModelState)
  q = np.zeros(QUEUE_SHAPE, dtype=np.float32) if queue is None else queue.astype(np.float32)
  m.input_queues = {'state_desire_q': Tensor(q, device='CPU').realize(),
                    'state_features': Tensor(np.full((1, 4, 8), 2., dtype=np.float32), device='CPU').realize()}
  m.state_pairs = {'state_desire_q': 'next_state_desire_q', 'state_features': 'next_state_features'}
  m.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
  m.packed_input = np.ones(64, dtype=np.uint8)
  m.frame_copy_size = 16
  m.vision_input_names = ('img', 'big_img')
  m.chestnut = chestnut
  return m


def queue_of(m: ModelState, name: str = 'state_desire_q') -> np.ndarray:
  return m.input_queues[name].numpy()


def dirty_run(m: ModelState):
  """warmup()'s dummy run: it leaves the state queues, prev_desire and the packed inputs non-zero, as a real model run does."""
  def run(bufs, transforms, inputs, after_enqueue=None):
    m.input_queues['state_desire_q'].assign(Tensor(np.ones(QUEUE_SHAPE, dtype=np.float32), device='CPU')).realize()
    m.input_queues['state_features'].assign(Tensor(np.full((1, 4, 8), 3., dtype=np.float32), device='CPU')).realize()
    m.prev_desire[:] = 1.
    m.packed_input[:] = 7
    return {}
  return run


def runs(m: ModelState, frame: BaseException | None = None):
  """m.run for a warmup() whose warm clear fails: warmup's own dummy frame works (dirty_run); every frame after it, the
  warm clear's probe (ModelState.frame_fault), raises frame, or works too with frame None."""
  dirty, calls = dirty_run(m), []
  def run(bufs, transforms, inputs, after_enqueue=None):
    calls.append(1)
    if len(calls) > 1 and frame is not None:
      raise frame
    return dirty(bufs, transforms, inputs, after_enqueue)
  return run


def test_warmup_warms_both_turn_desires_clear_on_the_zeroed_queue():
  m = bare_model()
  seen = []
  real_clear = ModelState.clear_desire_history
  def spy(desires):
    # the warm call comes after the queues are zeroed: a clear of zeros, which changes nothing
    seen.append((list(desires), queue_of(m).copy(), m.prev_desire.copy()))
    real_clear(m, desires)
  with mock.patch.object(m, 'run', side_effect=dirty_run(m)) as run, mock.patch.object(m, 'clear_desire_history', side_effect=spy):
    m.warmup()
  assert run.call_count == 1
  assert len(seen) == 1
  desires, q_at_clear, prev_at_clear = seen[0]
  assert desires == TURNS
  assert not q_at_clear.any() and not prev_at_clear.any()
  assert not queue_of(m).any() and not m.prev_desire.any()
  assert not queue_of(m, 'state_features').any()  # warmup's own zeroing, untouched by the clear
  assert not m.packed_input.any()


def test_warmup_logs_the_warm_clear_and_its_time():
  m = bare_model(chestnut=True)
  with mock.patch.object(m, 'run', side_effect=dirty_run(m)), mock.patch.object(modeld.cloudlog, 'info') as info:
    m.warmup()
  lines = [c.args[0] for c in info.call_args_list]
  assert any(s.startswith("turnpilot: desire clear warmed on the big model in ") and s.endswith(" ms") for s in lines), lines


def raised(fn) -> BaseException:
  """fn's exception, raised for real by tinygrad's own code (on stand-ins for the device)."""
  try:
    fn()
  except BaseException as e:
    return e
  raise AssertionError(f"{fn} raised nothing")


def hcq(err: int = 0) -> SimpleNamespace:
  """device.py's HCQ device as _wait_signal and on_device_hang see it: no recovery, a 1 ms wait, this error word."""
  return SimpleNamespace(device='AMD', can_recover=False, wait_timeout_ms=1, sleep_timeout_ms=None,
                         error_state=SimpleNamespace(host=SimpleNamespace(view=lambda fmt: [err])))


def amd_iface() -> SimpleNamespace:
  """ops_amd's PCIIface as on_device_hang and sleep see it: the interrupts collected (the reset), the error state set."""
  return SimpleNamespace(_collect_interrupts=lambda reset=False: None, dev=SimpleNamespace(can_recover=True),
                         pci_dev=SimpleNamespace(), dev_impl=SimpleNamespace(is_err_state=True))


def vram_oom() -> None:
  """device.py's allocator over an AMD _alloc with no VRAM left: its MemoryError."""
  def no_vram(size, spec):
    raise MemoryError(f"Cannot allocate {size} bytes: no memory is available.")
  Allocator.alloc(SimpleNamespace(cache=defaultdict(list), default_buffer_spec=BufferSpec(), _alloc=no_vram,
                                  free_cache=lambda: None, dev=SimpleNamespace(device='AMD')), 4096)


def usb_short_read() -> None:
  """support/usb.py's bulk IN that came back short (the transfer itself returning success)."""
  dev = SimpleNamespace(_bulk_mv=memoryview(bytearray(8192)), _bulk_buf=None, handle=None, _transferred=SimpleNamespace(value=0))
  with mock.patch.object(usb, 'checked', lambda fn, msg=None: lambda *a: 0):
    usb.USB3.bulk_read(dev, 4096)


def copy_size_mismatch() -> None:
  """The clear's copy-in (Buffer.copy_from) from a host queue of the wrong size."""
  q = Tensor(np.zeros(QUEUE_SHAPE, dtype=np.float32), device='CPU').realize()
  q._buffer().copy_from(Tensor(np.zeros((1, 24, ModelConstants.DESIRE_LEN), dtype=np.float32), device='NPY')._buffer())


def compile_error() -> None:
  raise CompileError("copy kernel")


# what tinygrad raises on a chestnut fault in the clear's read-back or copy-in, from its own code (device.py, ops_amd,
# support/usb.py, support/system.py)
CHESTNUT_FAULTS = {
  'signal wait timed out': lambda: Compiled._wait_signal(hcq(), [0], 1),
  'failed with 3': lambda: Compiled._wait_signal(hcq(3), [0], 1),
  'hang detected': lambda: Compiled.on_device_hang(hcq()),
  'Device hang detected': lambda: PCIIface.on_device_hang(amd_iface()),
  'Device is in error state': lambda: PCIIface.sleep(amd_iface(), 1),
  'bulk IN short read': usb_short_read,
  'VRAM allocation': vram_oom,
  'sysfs open': lambda: FileIOInterface('/nonexistent/turnpilot/chestnut'),
}
# what tinygrad's own code raises that isn't the device: the review's two RuntimeErrors first (11-06 round 2), which a
# type rule took for faults, then a shape or size mismatch, a compile error and an unimplemented path
CODE_ERRORS = {
  'assign dtype mismatch': lambda: Tensor.zeros(4).realize().assign(Tensor.zeros(4, dtype=dtypes.int32)).realize(),
  "can't create Tensor from": lambda: Tensor(object()),
  'copy size mismatch': copy_size_mismatch,
  "can't reshape": lambda: Tensor.ones(3, 4).reshape(5, 5),
  'CompileError': compile_error,
  'need alloc': lambda: Allocator._alloc(None, 4096, BufferSpec()),
}


def test_the_stand_ins_raise_what_the_device_and_the_code_raise():
  """The faults and code errors above, as tinygrad raises them: the review's point (11-06 round 2) is that a code error
  ('assign dtype mismatch') is a RuntimeError like a device fault, so neither the type nor the message can sort them."""
  faults = {k: raised(v) for k, v in CHESTNUT_FAULTS.items()}
  bugs = {k: raised(v) for k, v in CODE_ERRORS.items()}
  for want, e in (faults | bugs).items():
    assert want in str(e) or want == type(e).__name__ or (want, type(e)) in (('VRAM allocation', MemoryError), ('sysfs open', FileNotFoundError)), (want, e)
  assert type(faults['signal wait timed out']) is type(bugs['assign dtype mismatch']) is type(bugs["can't create Tensor from"]) is RuntimeError


def warm(m: ModelState, clear_error: BaseException, frame: BaseException | None):
  """warmup() on m with its warm clear raising clear_error and the probe's frame raising frame (None: it runs). Returns
  the run, clear and log mocks and the exception warmup() raised (None for none)."""
  run = mock.Mock(side_effect=runs(m, frame))
  with mock.patch.object(m, 'run', run), mock.patch.object(m, 'clear_desire_history', side_effect=clear_error) as clear, \
       mock.patch.object(modeld.cloudlog, 'error') as err, mock.patch.object(modeld.cloudlog, 'exception') as exc, \
       mock.patch.object(modeld.cloudlog, 'info') as info:
    try:
      m.warmup()
      out = None
    except BaseException as e:  # noqa: B036: what warmup() raised is the test's
      out = e
  return run, clear, err, exc, info, out


@pytest.mark.parametrize("name", CHESTNUT_FAULTS)
def test_a_chestnut_that_faults_in_the_warm_clear_and_then_fails_a_frame_fails_warmup_as_a_load(name):
  """Review 11-06: swallowed, warmup() returned, ChestnutActive went True and the fault came out on the first frame as
  'failed while running'. The chestnut can't run the frame after it, so the clear's own exception raises out of warmup()
  and load_big's except and the startup ChestnutRetry (a successor's link.failed('load: ...')) take it as a load failure."""
  fault, frame = raised(CHESTNUT_FAULTS[name]), raised(CHESTNUT_FAULTS['signal wait timed out'])
  m = bare_model(chestnut=True)
  run, clear, err, exc, info, out = warm(m, fault, frame)
  assert out is fault  # the clear's own exception: note_big_model's 'failed to load' says what it was
  assert clear.call_count == 1 and run.call_count == 2  # warmup's frame, then the probe's
  line = err.call_args.args[0]
  assert 'the chestnut faulted in the desire clear warm-up' in line and type(fault).__name__ in line
  assert 'AMD signal wait timed out' in line  # what the probe's frame raised
  assert not any('desire clear warmed' in c.args[0] for c in info.call_args_list)


@pytest.mark.parametrize("name", CODE_ERRORS)
def test_a_warm_clear_failure_on_a_chestnut_that_then_runs_a_frame_does_not_fail_warmup(name):
  """Review 11-06 round 2: round 1 sorted the failure by type, and tinygrad raises its own code errors as RuntimeError
  ('assign dtype mismatch', "can't create Tensor from"), so a code bug in the clear failed the load at every start, the
  retry and every successor. The chestnut ran the frame after it, so it works: logged, the state the probe's frame
  dirtied zeroed again, and the load goes on. The first real clear pays its first use (the pulse ages out if it fails)."""
  bug = raised(CODE_ERRORS[name])
  m = bare_model(chestnut=True)
  run, clear, err, exc, info, out = warm(m, bug, None)
  assert out is None
  assert clear.call_count == 1 and run.call_count == 2
  assert exc.call_count == 1 and 'but the chestnut ran a frame after it' in exc.call_args.args[0]
  assert type(bug).__name__ in exc.call_args.args[0]
  assert err.call_count == 0
  assert not queue_of(m).any() and not queue_of(m, 'state_features').any()
  assert not m.prev_desire.any() and not m.packed_input.any()


def test_a_fault_the_chestnut_came_back_from_lets_the_load_go_on():
  """AMD's on_device_hang resets the GPU before it raises 'Device hang detected': the frame after it runs, as the frame
  loop's first would, so the big model goes on. What the chestnut does next decides, not what it raised."""
  fault = raised(CHESTNUT_FAULTS['Device hang detected'])
  m = bare_model(chestnut=True)
  run, clear, err, exc, info, out = warm(m, fault, None)
  assert out is None and run.call_count == 2 and err.call_count == 0
  assert 'RuntimeError' in exc.call_args.args[0]
  assert not queue_of(m).any() and not m.prev_desire.any() and not m.packed_input.any()


def test_a_code_error_on_a_chestnut_that_then_fails_a_frame_fails_warmup():
  """The other way round: a code error in the clear, then a frame the chestnut can't run (its USB gone), is a load
  failure, with the clear's exception."""
  bug, frame = raised(CODE_ERRORS['copy size mismatch']), raised(usb_short_read)
  m = bare_model(chestnut=True)
  run, clear, err, exc, info, out = warm(m, bug, frame)
  assert out is bug and run.call_count == 2
  assert 'bulk IN short read' in err.call_args.args[0] and exc.call_count == 0


class StopMain(Exception):
  pass


class FakeVipc:
  """VisionIpcClient for main()'s startup: the narrow road stream only, connected at once."""
  width, height, buffer_len = 1928, 1208, 4

  def __init__(self, *a):
    pass

  @staticmethod
  def available_streams(name, block):
    return {modeld.VisionStreamType.VISION_STREAM_NARROW_ROAD}

  def connect(self, blocking):
    return True


def big_model_state(clear_error: BaseException | None, frame: BaseException | None):
  """main()'s ModelState: a bare one, the big model's warmup frame working, its warm clear raising clear_error (None: it
  works) and the probe's frame after it raising frame (None: it runs)."""
  def model_state(w, h, chestnut):
    m = bare_model(chestnut=chestnut)
    if chestnut:
      m.run = runs(m, frame)
      if clear_error is not None:
        m.clear_desire_history = mock.Mock(side_effect=clear_error)
    return m
  return model_state


def run_main_to_the_detector(clear_error, retry: bool, frame=None):
  """main() for real from its start to load_detector (which ends it): the chestnut there and compiled, the big model
  big_model_state's, the small model a bare one. Returns the params, note_big_model's calls and whether load_detector got
  the big model."""
  from tinygrad.runtime.ops_amd import AMDDevice
  params = FakeParams(ChestnutRetry=retry)
  params.remove = lambda key: params.values.pop(key, None)
  got = {}

  def load_detector(params_, link, w, h, wide, extra, models):
    got['models'] = models
    raise StopMain

  with mock.patch.object(modeld, 'chestnut_compiled', return_value=True), mock.patch.object(modeld, 'chestnut_present', return_value=True), \
       mock.patch.object(modeld, 'Params', return_value=params), mock.patch.object(modeld, 'VisionIpcClient', FakeVipc), \
       mock.patch.object(modeld, 'wait_for_chestnut'), mock.patch.object(modeld, 'ModelState', side_effect=big_model_state(clear_error, frame)), \
       mock.patch.object(modeld, 'reset_chestnut', return_value='reset'), mock.patch.object(modeld, 'note_big_model') as note, \
       mock.patch.object(modeld, 'load_detector', side_effect=load_detector), mock.patch.object(AMDDevice, 'wait_timeout_ms', 30000.), \
       mock.patch.object(modeld.cloudlog, 'exception'), mock.patch.object(modeld.cloudlog, 'error'), \
       mock.patch.object(modeld.cloudlog, 'warning'), mock.patch.object(modeld.cloudlog, 'info'):
    try:
      with pytest.raises(StopMain):
        modeld.main()
    finally:
      gc.enable()  # main() disables it for the process
  return params.values, note.call_args_list, got['models'][0].chestnut


@pytest.mark.parametrize("retry", [False, True], ids=['first load', 'onroad retry'])
def test_mains_load_takes_a_chestnut_fault_in_the_warm_clear_as_a_failed_load(retry):
  """Through main(): a fault in warmup's clear that the chestnut can't run a frame after is load_big's 'failed to load'
  (the clear's exception noted), so the first load cycles onroad to retry from a freshly reset GPU and the retry leaves
  ChestnutActive False (the recovery takes over), and modeld drives on the small model. Through the first 11-06 round
  warmup() returned, ChestnutActive went True on the faulted chestnut and the fault came out on the first frame instead."""
  fault, frame = raised(CHESTNUT_FAULTS['signal wait timed out']), raised(CHESTNUT_FAULTS['signal wait timed out'])
  values, notes, big = run_main_to_the_detector(fault, retry, frame)
  assert len(notes) == 1
  assert notes[0].args[1] == "failed to load" and notes[0].args[2] is fault and notes[0].args[4] is retry
  assert big is False  # the small model is the model
  if retry:
    assert values["ChestnutActive"] is False and not values.get("OnroadCycleRequested")
  else:
    assert values["ChestnutRetry"] is True and values["OnroadCycleRequested"] is True and "ChestnutActive" not in values


def test_mains_load_still_goes_big_when_the_warm_clear_works():
  values, notes, big = run_main_to_the_detector(None, False)
  assert notes == [] and big is True and values["ChestnutActive"] is True and not values.get("OnroadCycleRequested")


@pytest.mark.parametrize("retry", [False, True], ids=['first load', 'onroad retry'])
def test_mains_load_goes_big_through_a_tinygrad_code_error_in_the_warm_clear(retry):
  """Review 11-06 round 2, through main(): tinygrad's 'assign dtype mismatch' (a RuntimeError) in the clear of a chestnut
  that runs a frame after it. Round 1 noted 'failed to load' and drove the small model; the big model is the model."""
  values, notes, big = run_main_to_the_detector(raised(CODE_ERRORS['assign dtype mismatch']), retry)
  assert notes == [] and big is True and values["ChestnutActive"] is True and not values.get("OnroadCycleRequested")


class FakeLink:
  """chestnut_recovery's SuccessorLink as main() uses it: what the successor reported. loaded() ends main()."""
  def __init__(self, fds):
    self.events = []

  def failed(self, why):
    self.events.append(('failed', why))

  def loaded(self):
    self.events.append(('loaded',))
    raise StopMain


class Exited(Exception):
  pass


def run_successor(clear_error, frame=None):
  """A successor modeld's main() for real, its big model big_model_state's: the link's reports and how main() ended."""
  from openpilot.selfdrive.modeld import chestnut_recovery
  from tinygrad.runtime.ops_amd import AMDDevice
  links = []
  params = FakeParams()
  params.remove = lambda key: params.values.pop(key, None)
  params.get = lambda key, block=False: None
  with mock.patch.object(chestnut_recovery, 'SuccessorLink', side_effect=lambda fds: links.append(FakeLink(fds)) or links[-1]), \
       mock.patch.object(modeld.cloudlog, 'bind'), mock.patch.object(modeld, 'Params', return_value=params), \
       mock.patch.object(modeld, 'VisionIpcClient', FakeVipc), mock.patch.object(modeld, 'wait_for_chestnut'), \
       mock.patch.object(modeld, 'ModelState', side_effect=big_model_state(clear_error, frame)), \
       mock.patch.object(modeld, 'load_detector', return_value=None), mock.patch.object(AMDDevice, 'wait_timeout_ms', 30000.), \
       mock.patch.object(modeld.os, '_exit', side_effect=Exited), mock.patch.object(modeld, 'PubMaster'), \
       mock.patch.object(modeld, 'SubMaster'), mock.patch.object(modeld, 'get_demo_car_params'), \
       mock.patch.object(modeld.cloudlog, 'exception'), mock.patch.object(modeld.cloudlog, 'error'), \
       mock.patch.object(modeld.cloudlog, 'warning'), mock.patch.object(modeld.cloudlog, 'info'):
    try:
      with pytest.raises((Exited, StopMain)) as ended:
        modeld.main(demo=True, successor='3,4')
    finally:
      gc.enable()
  return links[0].events, ended.type


def test_a_successor_whose_chestnut_faults_in_the_warm_clear_reports_a_failed_load():
  fault = raised(CHESTNUT_FAULTS['signal wait timed out'])
  events, ended = run_successor(fault, raised(CHESTNUT_FAULTS['signal wait timed out']))
  assert ended is Exited and events == [('failed', 'load: RuntimeError: AMD signal wait timed out')]


def test_a_successor_loads_through_a_tinygrad_code_error_in_the_warm_clear():
  """Review 11-06 round 2: round 1's successor reported link.failed('load: RuntimeError: assign dtype mismatch ...') at
  every backoff slot of the drive."""
  events, ended = run_successor(raised(CODE_ERRORS['assign dtype mismatch']))
  assert ended is StopMain and events == [('loaded',)]


@pytest.mark.parametrize("name", ['signal wait timed out', 'Device hang detected'])
def test_the_small_models_warm_clear_never_raises_and_runs_no_frame(name):
  """main() calls the small model's warm clear outside any load-failure path, and comma's model stays the fallback: a real
  fault on comma's GPU comes out on its first frame (main()'s 'if not model.chestnut: raise'). No probe frame either."""
  m = bare_model(chestnut=False)
  run = mock.Mock()
  with mock.patch.object(m, 'run', run), mock.patch.object(m, 'clear_desire_history', side_effect=raised(CHESTNUT_FAULTS[name])), \
       mock.patch.object(modeld.cloudlog, 'exception') as exc, mock.patch.object(modeld.cloudlog, 'error') as err:
    m.warm_desire_clear()
  assert exc.call_count == 1 and 'desire clear warm-up failed' in exc.call_args.args[0]
  assert run.call_count == 0 and err.call_count == 0


def test_the_probe_is_warmups_own_frame():
  """frame_fault runs warmup()'s dummy frame (run_dummy_frame): zero frames of frame_copy_size, identity transforms, zero
  inputs; None when it runs, 'Type: message' when it raises."""
  m = bare_model(chestnut=True)
  seen = []
  with mock.patch.object(m, 'run', side_effect=lambda bufs, transforms, inputs: seen.append((bufs, transforms, inputs))):
    assert m.frame_fault() is None
  (bufs, transforms, inputs), = seen
  assert set(bufs) == set(transforms) == set(m.vision_input_names)
  assert all(b.shape == (m.frame_copy_size,) and not b.any() for b in bufs.values())
  assert all((t == np.eye(3)).all() for t in transforms.values())
  assert all(not v.any() for v in inputs.values()) and inputs['desire_pulse'].shape == (ModelConstants.DESIRE_LEN,)
  with mock.patch.object(m, 'run', side_effect=raised(usb_short_read)):
    assert m.frame_fault() == "RuntimeError: bulk IN short read: 0/4096 bytes"


def test_the_warm_clear_on_a_fresh_small_model_changes_nothing():
  """main()'s call on the small model, which is never warmup()'d: its queue is the constructor's zeros."""
  m = bare_model(chestnut=False)
  with mock.patch.object(modeld.cloudlog, 'info') as info:
    m.warm_desire_clear()
  assert not queue_of(m).any() and not m.prev_desire.any()
  assert (queue_of(m, 'state_features') == 2.).all()
  assert info.call_args.args[0].startswith("turnpilot: desire clear warmed on the small model in ")


def test_the_clear_takes_out_only_the_turn_desires():
  """What the warm call and the frame loop's clear do to a live queue: the turn columns and their prev_desire out, the
  rest (lane changes, keep) kept."""
  q = np.random.default_rng(0).uniform(0.1, 1., QUEUE_SHAPE).astype(np.float32)
  m = bare_model(q)
  m.prev_desire[:] = 1.
  m.clear_desire_history(TURNS)
  after = queue_of(m)
  keep = [d for d in range(ModelConstants.DESIRE_LEN) if d not in TURNS]
  assert not after[..., TURNS].any()
  np.testing.assert_array_equal(after[..., keep], q[..., keep])
  assert not m.prev_desire[TURNS].any() and (m.prev_desire[keep] == 1.).all()


def test_the_frame_loops_clear_is_timed_on_its_log_line():
  m = mock.Mock()
  with mock.patch.object(modeld.cloudlog, 'info') as info:
    took, ok = clear_turn_desires(m)
  m.clear_desire_history.assert_called_once_with(TURNS)
  assert took >= 0. and ok is True
  line = info.call_args.args[0]
  assert line.startswith("turnpilot: turn desires cleared from the model's desire queue in ") and line.endswith(" ms")


def test_a_failing_clear_is_logged_timed_and_says_it_failed():
  """A clear that raised still ran on the frame (its time is the slow frame's clear_ms), and the pulse ages out; ok is
  False, so its slow frame says clear_failed, not cleared (review 11-06: it logged cleared=True, a slow clear)."""
  m = mock.Mock()
  m.clear_desire_history.side_effect = RuntimeError("boom")
  with mock.patch.object(modeld.cloudlog, 'exception') as exc, mock.patch.object(modeld.cloudlog, 'info') as info:
    took, ok = clear_turn_desires(m)
  assert isinstance(took, float) and took >= 0. and ok is False
  line = exc.call_args.args[0]
  assert line.startswith("turnpilot: desire clear failed after ") and line.endswith(" ms")
  assert info.call_count == 0


class Events:
  def __init__(self):
    self.got = []

  def __call__(self, name, **fields):
    self.got.append((name, fields))


def test_a_frame_at_or_under_100_ms_is_not_logged():
  ev = Events()
  log_ = SlowFrameLog(ev)
  assert not log_.update(10., 0.042, 0.040, None, 0.0, 1)
  assert not log_.update(10.05, modeld.SLOW_FRAME_S, 0.040, None, 0.0, 2)
  assert ev.got == []


def test_1e_seg1s_stalled_frame_says_the_clear_took_its_time():
  """Frame 1872 of 1e seg 1: ~419 ms of work, the model's 42.1 ms of it, ~370 ms in the clear, the detector's step."""
  ev = Events()
  assert SlowFrameLog(ev).update(1000., 0.419, 0.0421, (0.370, True), 0.004, 1872)
  assert ev.got == [('modeld.slow_frame', {'ms': 419.0, 'model_ms': 42.1, 'cleared': True, 'clear_failed': False, 'clear_ms': 370.0,
                                           'detector_ms': 4.0, 'frame_id': 1872, 'unlogged': 0, 'since_last_s': None})]


def test_a_slow_frame_whose_clear_raised_says_it_failed_not_cleared():
  """The same frame had the clear raised after its 370 ms: not a successful clear that was slow."""
  ev = Events()
  assert SlowFrameLog(ev).update(1000., 0.419, 0.0421, (0.370, False), 0.004, 1872)
  assert ev.got == [('modeld.slow_frame', {'ms': 419.0, 'model_ms': 42.1, 'cleared': False, 'clear_failed': True, 'clear_ms': 370.0,
                                           'detector_ms': 4.0, 'frame_id': 1872, 'unlogged': 0, 'since_last_s': None})]


def test_the_frame_loop_hands_the_clears_result_to_the_slow_frame_log_as_is():
  """clear_turn_desires' (seconds, ok) is what SlowFrameLog.update takes, for both outcomes."""
  for side_effect, cleared, failed in ((None, True, False), (RuntimeError("AMD signal wait timed out"), False, True)):
    m = mock.Mock()
    m.clear_desire_history.side_effect = side_effect
    with mock.patch.object(modeld.cloudlog, 'exception'), mock.patch.object(modeld.cloudlog, 'info'):
      result = clear_turn_desires(m)
    ev = Events()
    assert SlowFrameLog(ev).update(5., 0.2, 0.04, result, 0., 7)
    assert ev.got[0][1]['cleared'] is cleared and ev.got[0][1]['clear_failed'] is failed


def test_slow_frames_are_logged_at_most_once_a_second_and_count_the_rest():
  ev = Events()
  log_ = SlowFrameLog(ev)
  assert log_.update(100.0, 0.15, 0.04, None, 0.09, 10)
  assert not log_.update(100.3, 0.12, 0.04, None, 0.07, 16)  # held back
  assert not log_.update(100.9, 0.20, 0.04, None, 0.15, 28)  # held back
  assert not log_.update(100.95, 0.05, 0.04, None, 0.0, 29)  # not slow: not counted
  assert log_.update(101.0, 0.11, 0.04, None, 0.06, 30)
  assert [f['frame_id'] for _, f in ev.got] == [10, 30]
  assert [f['unlogged'] for _, f in ev.got] == [0, 2]
  assert [f['since_last_s'] for _, f in ev.got] == [None, 1.0]  # the 2 held back belong to the second after frame 10
  assert all(f['cleared'] is False and f['clear_failed'] is False and f['clear_ms'] == 0. for _, f in ev.got)
  assert not log_.update(101.5, 0.3, 0.04, (0.25, True), 0.0, 40)  # a clear's slow frame inside the second is held back too
  assert log_.update(102.1, 0.3, 0.04, (0.25, True), 0.0, 52)
  assert ev.got[-1][1]['cleared'] is True and ev.got[-1][1]['unlogged'] == 1 and ev.got[-1][1]['since_last_s'] == 1.1


class FakeParams:
  def __init__(self, **values):
    self.values = dict(values)

  def put_bool(self, key, value):
    self.values[key] = bool(value)

  def get_bool(self, key):
    return bool(self.values.get(key))


class FakeHost:
  """DetectorHost's startup surface: check_enabled and load, noting ChestnutLoading as load() saw it."""
  instances: list = []

  def __init__(self, params, cam_w, cam_h, main_wide_camera, use_extra_client, models, enabled=True, fail=False):
    self.params, self.args, self.models = params, (cam_w, cam_h, main_wide_camera, use_extra_client), models
    self.enabled, self.fail, self.loading_at_load = enabled, fail, None
    FakeHost.instances.append(self)

  def check_enabled(self, now):
    return self.enabled

  def load(self):
    self.loading_at_load = self.params.get_bool("ChestnutLoading")
    if self.fail:
      raise RuntimeError("far pkl compile failed")


def host_cls(**kw):
  return lambda *a: FakeHost(*a, **kw)


@pytest.fixture(autouse=True)
def _fresh_hosts():
  FakeHost.instances = []


def test_chestnut_loading_stays_set_through_the_detector_load_then_clears():
  params = FakeParams(ChestnutLoading=True)
  det = load_detector(params, None, 1344, 760, False, True, ['big', 'small'], host_cls=host_cls())
  host = FakeHost.instances[0]
  assert det is host and host.args == (1344, 760, False, True) and host.models == ['big', 'small']
  assert host.loading_at_load is True  # selfdrived's big model settling covers the ~19.5 s load: no commIssue
  assert params.values["ChestnutLoading"] is False


def test_a_failed_detector_load_still_clears_chestnut_loading():
  """Otherwise selfdrived's bigModelLoading (no entry) would stand for the drive."""
  params = FakeParams(ChestnutLoading=True)
  with mock.patch.object(modeld.cloudlog, 'exception') as exc:
    det = load_detector(params, None, 1344, 760, False, True, ['big'], host_cls=host_cls(fail=True))
  assert det is None
  assert exc.call_args.args[0] == "turnpilot detector failed to load"
  assert params.values["ChestnutLoading"] is False


def test_turnpilot_off_loads_nothing_and_clears_chestnut_loading():
  params = FakeParams(ChestnutLoading=True)
  det = load_detector(params, None, 1344, 760, False, True, ['big'], host_cls=host_cls(enabled=False))
  assert det is FakeHost.instances[0] and det.loading_at_load is None
  assert params.values["ChestnutLoading"] is False


def test_a_successor_leaves_chestnut_loading_to_its_parent():
  params = FakeParams(ChestnutLoading=True)
  load_detector(params, object(), 1344, 760, False, True, ['big'], host_cls=host_cls())
  assert params.values["ChestnutLoading"] is True


def test_main_clears_chestnut_loading_only_through_load_detector():
  """The order lives in load_detector; main() must not clear the param before it, as it did through 11-06."""
  import inspect
  src = inspect.getsource(modeld.main)
  assert 'put_bool("ChestnutLoading", False)' not in src
  assert src.index('load_detector(') > src.index('models loaded in')
  assert 'small_model.warm_desire_clear()' in src


def test_bench_on_the_chestnut_the_first_two_clears_after_load_are_under_20_ms():
  """The bench, on the comma with the chestnut and nothing else holding it (the manager stopped):
    TURNPILOT_CHESTNUT_BENCH=1 pytest -q -s openpilot/selfdrive/modeld/tests/test_desire_clear_warmup.py -k bench
  The big model loaded and warmed as modeld does, then a frame run and a timed clear, twice: the frame loop's first and
  second clear of a drive. Without the warm call the first was ~370 ms (1e seg 1); the device sync is in the time."""
  if os.environ.get('TURNPILOT_CHESTNUT_BENCH') != '1':
    pytest.skip("the chestnut bench: TURNPILOT_CHESTNUT_BENCH=1 on the comma with the chestnut")
  from tinygrad.device import Device
  from tinygrad.runtime.ops_amd import AMDDevice
  from openpilot.selfdrive.modeld.helpers import chestnut_compiled, wait_for_chestnut
  AMDDevice.wait_timeout_ms = 3000
  wait_for_chestnut()
  assert chestnut_compiled()
  m = ModelState(1344, 760, True)
  m.warmup()
  frames = {k: np.zeros(m.frame_copy_size, dtype=np.uint8) for k in m.vision_input_names}
  eye = dict.fromkeys(m.vision_input_names, np.eye(3, dtype=np.float32))
  took = []
  for _ in range(2):
    pulse = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    pulse[log.Desire.turnLeft] = 1.
    m.run(frames, eye, {'desire_pulse': pulse, 'traffic_convention': np.array([1, 0], dtype=np.float32),
                        'action_t': np.array([0.3, 0.5], dtype=np.float32)})
    t0 = time.perf_counter()
    m.clear_desire_history(TURNS)
    Device[m.model_device].synchronize()
    took.append(time.perf_counter() - t0)
  print(f"first and second clear after load: {took[0] * 1e3:.1f} ms, {took[1] * 1e3:.1f} ms")
  assert max(took) < 0.020, took
