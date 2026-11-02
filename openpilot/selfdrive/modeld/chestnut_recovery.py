"""TurnPilot: bringing the big model back after the chestnut fails mid-drive, while the small model drives.

modeld falls back to the small model on a runtime failure of the big one (a moved USB cable: "chestnut failed with -1"). Until
this, that was the drive's end of the big model, and of TurnPilot. Now modeld retries in the background, RETRY_DELAYS_S after
the failure and then every RETRY_EVERY_S, and never in its main loop.

The retry runs in a fresh process, a successor modeld (modeld --successor), because tinygrad can't open the chestnut twice in
one process. Measured on the chestnut on a Mac (2026-11-01, big model at 1344x760):
- tinygrad's own recovery on the device it has open (AMDDevice.on_device_hang: the AM driver's recover, its queues set up
  again, the link's error word cleared) brought it back after a forced error word, the model's output bit-exact, in 9 s of
  USB register traffic. It needs the link it already has; a cable moved re-enumerates the chestnut, and that link is gone.
- A second AMDDevice in the same process, after closing the first's USB handle and lock and dropping it from tinygrad's
  caches (the device registry, Compiled.pm_bufferize's rules, the runtime cache, the USB device list), loaded the model in
  2.4 s and then faulted: GCVM_L2 protection faults, illegal instructions, 4.3 s a frame, the output off by 177. tinygrad
  keeps per-process state for a device it can't let go of (its VA space and caches).
- A fresh process (os.execv, same pid, after the error word was set and with no finalize) loaded it in 3.4 s, bit-exact
  against the old process's output, 20.9 ms a frame, as at startup.
So a successor is spawned. The parent (the process the manager watches) keeps the small model and goes on publishing until
the successor has the big model loaded and has run it on the current camera frames: then, with openpilot disengaged, the
parent stops publishing and the successor starts, the next frame. msgq hands a topic to a new publisher (its readers rejoin;
one message is lost at the change, measured), so nothing downstream restarts. modeld restarting itself (exec) would stall
modelV2 for the restart and the small model's load, seconds of NO_ENTRY and of no camera odometry while the car moves; a helper
holding only the big model would have to take the detector, which runs on the model's frames on the chestnut, and every
publish with it. A successor is modeld itself, so it takes them as they are. The manager still sees one modeld, the
parent. However the parent ends, its end of their pipe closes and the successor exits on that EOF (SuccessorLink): the
manager's SIGINT ends a multiprocessing child through os._exit, so no atexit runs there, and a SIGKILL runs nothing; the
atexit stop only covers modeld run on its own.
End to end on the bench (tools/turnpilot/chestnut_recovery_check.py --modeld: modeld --demo on fake cameras, its runner
raising once): modelV2 never paused more than 101 ms, the big model back 6.8 s after the failure (9.5 s with the detector,
whose detections the successor published 0.1 s after the handoff), ChestnutActive with it, one successor live past STALL_S.

The handoff waits for openpilot to be disengaged: the model never changes under an engaged openpilot (upstream never brings a
model back while driving), and TurnPilot resumes only at the driver's next engagement anyway (turnpilotd.py). A successor
that dies or stalls (STALL_S without its modelV2) gives the topics back: the parent publishes on the small model at once,
says the big model stopped (ChestnutActive False), and retries again. A successor live STABLE_S resets the backoff.

Each failed attempt is noted for Diagnostics (big model retry failed: why), each recovery (big model recovered), and each
successor that stopped (big model stopped); the in-process failure as before (big model failed while running).

Not covered here, and older than this: a chestnut that hangs rather than errors is caught inside tinygrad's synchronize, which
waits out AMDDevice.wait_timeout_ms (3 s) and runs the AM recover inline (the 9 s above, on a live link) before modeld's
fallback ever sees the exception. A link error (the cable's -1) is the error word ModelState.run raises on at once.
"""
import os
import queue
import subprocess
import sys
import threading
import time
from collections.abc import Callable

RETRY_DELAYS_S = (2., 5., 10.)  # after the failure, then after each failed attempt
RETRY_EVERY_S = 30.             # ...from then on
STABLE_S = 60.                  # a successor live this long resets the backoff: its failure is a new one
LOAD_DEADLINE_S = 90.           # the successor's load: modeld's 60 s for the big model (BIG_MODEL_TIMEOUT), and the detector's
PROBE_DEADLINE_S = 2.           # a probe answered (a sane output on a live frame) within this, or the successor is dropped
STALL_S = 10.                   # live, no modelV2 from the successor this long: stopped. Above a detector load, which holds
                                # modeld's loop for seconds while disengaged (DetectorHost), and tinygrad's 3 s wait timeout,
                                # past which a hung chestnut raises and the successor exits on its own
LITTLE_CORES = [0, 1, 2, 3]     # the attempt's thread, and so the successor it spawns: off modeld's realtime core 7

# the states of an attempt
IDLE, WAITING, STARTING, LOADING, LOADED, PROBING, LIVE = 'idle', 'waiting', 'starting', 'loading', 'loaded', 'probing', 'live'


def retry_delay(failed: int) -> float:
  """s to the next attempt after failed attempts since the big model stopped: 2, 5, 10, then 30 for good."""
  return RETRY_DELAYS_S[failed] if failed < len(RETRY_DELAYS_S) else RETRY_EVERY_S


def _noop(*args, **kwargs) -> None:
  return None


def chestnut_device(model_state):
  """tinygrad's device the model runs on (the registry's, opened already: never a new one)."""
  from tinygrad.device import Device
  return Device[Device.canonicalize(model_state.model_device)]


def make_inert(model_state) -> None:
  """The big model failed: nothing in this process touches its device again. Its synchronize, finalize and frees become
  no-ops, so a buffer of the dead model's whose last reference drops (the failed run's temporaries as its exception goes,
  or later, on any thread) never waits out a dead link's timeouts in modeld's loop, nor calls libusb on a handle let go of
  (abandon_chestnut); its device memory is the successor's to reclaim. Dropped from tinygrad's opened devices: no finalize
  at exit. Plain attribute sets, at the failure, in modeld's loop; never raises."""
  try:
    from tinygrad.device import Device
    dev = chestnut_device(model_state)
    Device._opened_devices.discard(Device.canonicalize(model_state.model_device))
    dev.synchronize = dev.finalize = _noop
    for fn in ('free', 'do_free', '_unmap', 'free_cache'):
      setattr(dev.allocator, fn, _noop)
  except Exception:
    pass


def abandon_chestnut(model_state, close_handle: bool = sys.platform == 'darwin') -> str:
  """Let go of the chestnut this process opened (make_inert's, already), so a successor can open it: tinygrad's USB
  interface claim and its lock file. In the attempt's thread, off modeld's loop. The caller keeps model_state referenced for
  the drive.
  close_handle: on Linux (the comma) a released interface is enough for another process to claim it (usbfs refuses
  SETCONFIGURATION only while an interface is claimed), and the handle stays open: a thread still in a transfer on it (the
  detector's side maps) never finds it freed. macOS's IOKit keeps the device itself open to one process, so there (the
  bench) the handle is closed. What was done, for the log; never raises."""
  done = []
  try:
    pci = getattr(getattr(chestnut_device(model_state), 'iface', None), 'pci_dev', None)
    usb = getattr(getattr(pci, 'usb', None), 'usb', None)
    if usb is not None:
      from tinygrad.runtime.autogen import libusb
      rc = libusb.libusb_release_interface(usb.handle, 0)
      done.append('interface released' if rc == 0 else f'interface release returned {rc}')
      if close_handle:
        libusb.libusb_close(usb.handle)
        done.append('handle closed')
    if getattr(pci, 'lock_fd', None) is not None:
      os.close(pci.lock_fd)
      pci.lock_fd = None
      done.append('lock closed')
  except Exception as e:
    done.append(f'{type(e).__name__}: {e}')
  return ', '.join(done) or 'nothing open'


class Successor:
  """The parent's end of a successor modeld: the process and two pipes, a line each way (the successor's stdout carries
  tinygrad's prints, so not that). Lines from it: loaded, sane, failed <why>. To it: probe, hold, go."""

  def __init__(self, proc, to_child: int, from_child: int):
    self.proc = proc
    self.to_child = to_child
    self.from_child = from_child
    os.set_blocking(from_child, False)
    self.buf = b''
    self.why = ''  # the successor's last 'failed' line

  @classmethod
  def spawn(cls, cmd: list[str] | None = None) -> 'Successor':
    """A fresh interpreter (never a fork: tinygrad's state and modeld's threads don't survive one), on the caller's
    scheduling and cores. cmd: the command, its pipe fds appended as 'R,W' (tests); default modeld --successor."""
    child_r, to_child = os.pipe()
    from_child, child_w = os.pipe()
    cmd = (cmd or [sys.executable, '-m', 'openpilot.selfdrive.modeld.modeld', '--successor']) + [f'{child_r},{child_w}']
    try:
      proc = subprocess.Popen(cmd, pass_fds=(child_r, child_w), close_fds=True)
    except Exception:
      for fd in (child_r, to_child, from_child, child_w):
        os.close(fd)
      raise
    os.close(child_r)
    os.close(child_w)
    return cls(proc, to_child, from_child)

  def lines(self) -> list[str]:
    """What it said since the last call; never blocks."""
    try:
      while chunk := os.read(self.from_child, 4096):
        self.buf += chunk
    except BlockingIOError:
      pass
    except OSError:
      return []
    *done, self.buf = self.buf.split(b'\n')
    out = [d.decode(errors='replace').strip() for d in done]
    for line in out:
      if line.startswith('failed'):
        self.why = line[len('failed'):].strip()
    return out

  def send(self, word: str) -> None:
    try:
      os.write(self.to_child, f'{word}\n'.encode())
    except OSError:
      pass  # it's gone: exited() says so

  def exited(self) -> int | None:
    return self.proc.poll()

  def stop(self) -> None:
    """Gone for good, without waiting (modeld's loop never blocks on it): its pipes closed and SIGKILL. It holds nothing a
    kill leaves behind (the kernel frees its USB claim and lock; msgq shows a message only once it is whole), and the
    parent publishes next, so it must be gone, not asked. Reaped in a thread."""
    for fd in (self.to_child, self.from_child):
      try:
        os.close(fd)
      except OSError:
        pass
    self.to_child = self.from_child = -1
    if self.proc.poll() is None:
      try:
        self.proc.kill()
      except OSError:
        pass
      threading.Thread(target=self.proc.wait, daemon=True, name='successor_reap').start()


class SuccessorLink:
  """The successor's end (modeld --successor R,W). It loads, says loaded, and runs nothing until the parent sends probe (with
  openpilot disengaged); then it runs the big model on every frame without publishing, and says sane after the first that
  ran clean (ModelState.run raises on the chestnut's error word and on output that isn't finite). On go it publishes from that
  frame; on hold it stops running and waits for the next probe. The parent's end closing (the parent exited or was killed)
  ends this process."""

  def __init__(self, fds: str, exit_fn: Callable[[int], None] = os._exit):
    r, w = (int(x) for x in fds.split(','))
    self.w = w
    self.cmds: queue.SimpleQueue = queue.SimpleQueue()
    self.exit_fn = exit_fn
    self.state = LOADING
    self.sane = False
    threading.Thread(target=self._read, args=(r,), daemon=True, name='successor_link').start()

  def _read(self, r: int) -> None:
    with os.fdopen(r, 'rb') as f:
      for line in f:
        self.cmds.put(line.decode(errors='replace').strip())
    self.exit_fn(0)  # EOF: the parent is gone, and the chestnut must be free for the next modeld

  def say(self, word: str, why: str = '') -> None:
    try:
      os.write(self.w, (f'{word} {why}'.strip() + '\n').encode())
    except OSError:
      pass

  def loaded(self) -> None:
    self.state = IDLE
    self.say('loaded')

  def poll(self) -> bool:
    """The parent's commands since the last frame. True on the frame it goes live (go, after a sane probe)."""
    went_live = False
    while not self.cmds.empty():
      cmd = self.cmds.get()
      if cmd == 'probe' and self.state != LIVE:
        self.state, self.sane = PROBING, False
      elif cmd == 'hold' and self.state == PROBING:
        self.state = IDLE
      elif cmd == 'go' and self.state == PROBING and self.sane:
        self.state, went_live = LIVE, True
    return went_live

  @property
  def runs(self) -> bool:
    return self.state in (PROBING, LIVE)

  @property
  def live(self) -> bool:
    return self.state == LIVE

  def ran(self) -> None:
    """The big model ran clean on this frame."""
    if self.state == PROBING and not self.sane:
      self.sane = True
      self.say('sane')

  def failed(self, why: str) -> None:
    self.say('failed', why.replace('\n', ' ')[:300])


def attempt_thread() -> None:
  """An attempt's thread leaves modeld's realtime slot (as the side maps' worker does), and the successor it spawns inherits
  that: at SCHED_FIFO 54 on core 7, the successor's imports and load would hold modeld's own loop off its core for seconds."""
  from openpilot.common.realtime import drop_realtime, set_core_affinity
  drop_realtime()
  set_core_affinity(LITTLE_CORES)


class ChestnutRecovery:
  """The parent's side: after a runtime failure (failed()), an attempt at each backoff slot, and the handoff. step() runs once
  a frame in modeld's loop and never blocks: the chestnut check and the spawn run in a thread of their own (run_async).

  spawn, present, note, run_async: stand-ins for tests (Successor.spawn, helpers.chestnut_present, a Diagnostics note
  taking (kind, why), a thread per attempt)."""

  def __init__(self, spawn: Callable[[], Successor] | None = None, present: Callable[[], bool] | None = None,
               note: Callable[[str, str], None] | None = None, run_async: Callable[[Callable[[], None]], None] | None = None):
    self.spawn = spawn or Successor.spawn
    self.present = present
    self.note = note or (lambda kind, why: None)
    self.run_async = run_async or self._thread
    self.state = IDLE
    self.failures = 0          # attempts failed since the big model stopped (the backoff's index)
    self.next_t = 0.
    self.deadline = 0.
    self.stopped_t = 0.        # when the big model stopped, for the recovery's note
    self.live_t = 0.           # when the last successor went live
    self.successor: Successor | None = None
    self.abandon: list = []    # in-process models to let go of before the next spawn (abandon_chestnut)
    self.dead: list = []       # ...and kept referenced for the drive
    self._result: tuple | None = None  # the attempt thread's (successor, why)

  @staticmethod
  def _thread(fn: Callable[[], None]) -> None:
    threading.Thread(target=fn, daemon=True, name='chestnut_retry').start()

  @property
  def active(self) -> bool:
    return self.state != IDLE

  def failed(self, now: float, model_state=None, why: str = '') -> None:
    """The big model stopped: in this process (model_state: its ModelState, let go of before the next spawn; modeld notes
    it as before, failed while running) or in the successor (why: standby's, noted). The next attempt per the backoff."""
    if model_state is not None:
      make_inert(model_state)
      self.abandon.append(model_state)
      self.dead.append(model_state)
    if self.state == LIVE:  # one that held is a new failure; one that didn't is this one's failed attempt
      self.note('stopped', f'after {now - self.live_t:.1f} s live: {why}')
      self.failures = 0 if now - self.live_t >= STABLE_S else self.failures + 1
    if self.state in (IDLE, LIVE):
      self.stopped_t = now
    self.drop()
    self.state, self.next_t = WAITING, now + retry_delay(self.failures)

  def drop(self) -> None:
    if self.successor is not None:
      self.successor.stop()
      self.successor = None

  def attempt_failed(self, now: float, why: str) -> None:
    self.note('retry failed', f'attempt {self.failures + 1}: {why}')
    self.failures += 1
    self.drop()
    self.state, self.next_t = WAITING, now + retry_delay(self.failures)

  def _attempt(self) -> None:
    """In the attempt's thread: off modeld's slot, let go of the dead chestnut, check it enumerated, spawn. A failure anywhere
    is the attempt's (noted, retried at the next slot): a thread that died before its result held the recovery in STARTING
    for the drive. Off the slot first: if that fails, nothing is spawned onto modeld's core at its priority."""
    try:
      try:
        attempt_thread()
      except Exception as e:
        self._result = (None, f'thread priority failed: {type(e).__name__}: {e}')
        return
      for m in self.abandon:
        abandon_chestnut(m)
      self.abandon = []
      if self.present is not None and not self.present():
        self._result = (None, 'chestnut not enumerated')
        return
      self._result = (self.spawn(), '')
    except Exception as e:
      self._result = (None, f'spawn failed: {type(e).__name__}: {e}')

  def step(self, now: float, engaged: bool) -> bool:
    """Once a frame. True: hand off now (openpilot disengaged, the successor sane on a live frame): the caller stops
    publishing, then calls go()."""
    if self.state == WAITING and now >= self.next_t - 1e-6:
      self.state, self._result = STARTING, None
      self.run_async(self._attempt)
    if self.state == STARTING and self._result is not None:
      self.successor, why = self._result
      if self.successor is None:
        self.attempt_failed(now, why)
      else:
        self.state, self.deadline = LOADING, now + LOAD_DEADLINE_S
    elif self.state in (LOADING, LOADED, PROBING):
      s = self.successor
      said = s.lines()
      if s.exited() is not None:
        self.attempt_failed(now, f'successor exited with {s.exited()}' + (f': {s.why}' if s.why else ''))
      elif self.state == LOADING:
        if 'loaded' in said:
          self.state = LOADED
        elif now > self.deadline:
          self.attempt_failed(now, f'successor not loaded in {LOAD_DEADLINE_S:.0f} s')
      if self.state == LOADED and not engaged:
        s.send('probe')
        self.state, self.deadline = PROBING, now + PROBE_DEADLINE_S
      elif self.state == PROBING:
        if 'sane' in said:
          if not engaged:
            return True
          s.send('hold')  # engaged meanwhile: the model never changes under an engaged openpilot
          self.state = LOADED
        elif now > self.deadline:
          self.attempt_failed(now, f'no sane output on live frames in {PROBE_DEADLINE_S:.0f} s')
    return False

  def go(self, now: float) -> None:
    """The caller has stopped publishing: the successor publishes from its next frame."""
    self.successor.send('go')
    self.state, self.live_t = LIVE, now
    self.note('recovered', f'big model back after {now - self.stopped_t:.1f} s, attempt {self.failures + 1}, in a successor modeld')

  def standby(self, heard: Callable[[], bool], now_fn: Callable[[], float] = time.monotonic,
              sleep: Callable[[float], None] = time.sleep, poll_s: float = 0.02) -> str:
    """Live: block until the successor stops (it exited, or STALL_S passed without its modelV2: heard() says whether it
    published since the last call; then it is stopped). Why, for the log and the note."""
    s = self.successor
    last = now_fn()
    while True:
      s.lines()
      if (code := s.exited()) is not None:
        return f'successor exited with {code}' + (f': {s.why}' if s.why else '')
      now = now_fn()
      if heard():
        last = now
      elif now - last > STALL_S:
        return f'successor stalled: no modelV2 for {STALL_S:.0f} s'
      sleep(poll_s)

  def close(self) -> None:
    """modeld exiting: the successor with it."""
    self.drop()
    self.state = IDLE
