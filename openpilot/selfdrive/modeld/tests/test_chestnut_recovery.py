"""modeld's mid-drive recovery of the chestnut (chestnut_recovery.py): the backoff, an attempt's states against a fake
successor and clock, the successor's end of the protocol, and the pipes with a real process. tools/turnpilot's
chestnut_recovery_check runs it on a real chestnut."""
import os
import sys
import time
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.selfdrive.modeld import chestnut_recovery as cr


class FakeSuccessor:
  def __init__(self):
    self.said, self.sent, self.code, self.why, self.stopped = [], [], None, '', False

  def lines(self):
    out, self.said = self.said, []
    return out

  def send(self, word):
    self.sent.append(word)

  def exited(self):
    return self.code

  def stop(self):
    self.stopped = True


class Rig:
  """A ChestnutRecovery on a fake clock, its attempts run inline, its successors fakes."""
  def __init__(self, present=True):
    self.successors, self.notes, self.present, self.t = [], [], present, 100.
    def spawn():
      s = FakeSuccessor()
      self.successors.append(s)
      return s
    self.rec = cr.ChestnutRecovery(spawn=spawn, present=lambda: self.present, note=lambda kind, why: self.notes.append((self.t, kind, why)),
                                   run_async=lambda fn: fn())

  def step(self, engaged=False, dt=0.05) -> bool:
    self.t += dt
    return self.rec.step(self.t, engaged)

  def run(self, seconds, engaged=False) -> list[float]:
    """Steps for seconds; the times a handoff was asked for."""
    asked = []
    for _ in range(round(seconds / 0.05)):
      if self.step(engaged):
        asked.append(self.t)
    return asked

  def loaded(self):
    """The big model just stopped in this process: the first attempt spawns at 2 s and its successor loads."""
    self.rec.failed(self.t, model_state='dead')
    self.run(2.05)
    assert self.rec.state == cr.LOADING and len(self.successors) == 1
    self.successors[-1].said.append('loaded')
    return self.successors[-1]

  def failures(self):
    return [(round(t - 100., 2), why) for t, kind, why in self.notes if kind == 'retry failed']


def test_the_backoff_is_2_5_10_then_every_30_s():
  assert [cr.retry_delay(n) for n in range(6)] == [2., 5., 10., 30., 30., 30.]


def test_attempts_follow_the_backoff_while_the_chestnut_is_gone():
  rig = Rig(present=False)
  rig.rec.failed(rig.t)
  rig.run(140.)
  times = [t for t, _ in rig.failures()]
  # each attempt at its slot after the last one failed: 2, +5, +10, then every 30 s
  assert [round(b - a) for a, b in zip([0.] + times[:-1], times, strict=True)] == [2, 5, 10, 30, 30, 30, 30]
  assert all(why.endswith('chestnut not enumerated') for _, why in rig.failures()) and not rig.successors


def test_a_loaded_successor_is_probed_only_disengaged_and_handed_over_on_a_sane_frame():
  rig = Rig()
  s = rig.loaded()
  assert rig.run(5., engaged=True) == [] and s.sent == []  # engaged: loaded, but never probed
  assert rig.step(engaged=False) is False and s.sent == ['probe']
  s.said.append('sane')
  assert rig.step(engaged=False) is True  # the caller stops publishing, then go()
  rig.rec.go(rig.t)
  assert s.sent == ['probe', 'go'] and rig.rec.state == cr.LIVE
  kind, why = rig.notes[-1][1:]
  assert (kind, why) == ('recovered', 'big model back after 7.1 s, attempt 1, in a successor modeld')  # from the failure


def test_a_sane_answer_after_the_driver_engaged_holds_and_probes_again_later():
  rig = Rig()
  s = rig.loaded()
  rig.step(engaged=False)
  s.said.append('sane')
  assert rig.step(engaged=True) is False and s.sent == ['probe', 'hold'] and rig.rec.state == cr.LOADED
  rig.run(3., engaged=True)
  assert s.sent == ['probe', 'hold']
  rig.step(engaged=False)
  assert s.sent == ['probe', 'hold', 'probe']


def test_a_successor_that_dies_loading_is_a_failed_attempt_with_its_why_and_the_next_waits_its_slot():
  rig = Rig()
  rig.rec.failed(rig.t)
  rig.run(2.05)
  s = rig.successors[0]
  s.code, s.why = 1, 'load: TimeoutError: chestnut did not enumerate'
  rig.step()
  assert rig.failures()[-1][1] == 'attempt 1: successor exited with 1: load: TimeoutError: chestnut did not enumerate'
  assert s.stopped and rig.rec.state == cr.WAITING
  rig.run(4.9)
  assert len(rig.successors) == 1
  rig.run(0.2)
  assert len(rig.successors) == 2  # 5 s after the failed attempt


@pytest.mark.parametrize("stage", ["load", "probe"])
def test_deadlines_drop_a_successor_that_never_answers(stage):
  rig = Rig()
  s = rig.loaded() if stage == "probe" else (rig.rec.failed(rig.t), rig.run(2.05), rig.successors[0])[-1]
  rig.run((cr.PROBE_DEADLINE_S if stage == "probe" else cr.LOAD_DEADLINE_S) + 0.2)
  assert s.stopped and rig.rec.state in (cr.WAITING, cr.STARTING, cr.LOADING)
  assert ('no sane output' if stage == "probe" else 'not loaded in') in rig.failures()[0][1]


def test_the_attempt_lets_go_of_the_dead_chestnut_before_spawning():
  rig = Rig()
  with mock.patch.object(cr, 'abandon_chestnut') as abandon, mock.patch.object(cr, 'make_inert') as inert:
    rig.rec.failed(rig.t, model_state='the dead big model')
    inert.assert_called_once_with('the dead big model')  # at the failure
    rig.run(1.9)
    abandon.assert_not_called()
    rig.run(0.2)
    abandon.assert_called_once_with('the dead big model')
    assert rig.rec.dead == ['the dead big model']  # referenced for the drive: its buffers never freed on a gone link


def test_standby_returns_when_the_successor_exits_or_stalls():
  rig = Rig()
  s = rig.loaded()
  rig.step()
  s.said.append('sane')
  rig.step()
  rig.rec.go(rig.t)
  clock = SimpleNamespace(t=0.)
  def sleep(dt):
    clock.t += dt
    if clock.t > 3.:
      s.code, s.why = 1, 'run: RuntimeError: chestnut failed with -1'
  why = rig.rec.standby(lambda: True, now_fn=lambda: clock.t, sleep=sleep)
  assert why == 'successor exited with 1: run: RuntimeError: chestnut failed with -1' and 3. < clock.t < 3.1
  s.code, clock.t = None, 0.
  heard = iter([True] * 10 + [False] * 10000)
  why = rig.rec.standby(lambda: next(heard), now_fn=lambda: clock.t, sleep=lambda dt: setattr(clock, 't', clock.t + dt))
  assert why.startswith('successor stalled') and abs(clock.t - (10 * 0.02 + cr.STALL_S)) < 0.05


@pytest.mark.parametrize("live_s,next_s", [(5., 5.), (cr.STABLE_S + 1., 2.)])
def test_a_live_successor_that_stops_is_retried_by_how_long_it_held(live_s, next_s):
  # one that held STABLE_S is a new failure (the backoff starts over); one that didn't is its attempt's failure
  rig = Rig()
  s = rig.loaded()
  rig.step()
  s.said.append('sane')
  rig.step()
  rig.rec.go(rig.t)
  rig.t += live_s
  rig.rec.failed(rig.t, why='successor exited with 1')
  assert s.stopped and rig.notes[-1][1] == 'stopped' and rig.notes[-1][2].startswith(f'after {live_s:.1f} s live')
  assert rig.rec.next_t - rig.t == next_s


def test_step_never_waits_on_the_spawn():
  # the chestnut check and the spawn run in the attempt's thread: modeld's loop goes on at 20 Hz meanwhile
  rec = cr.ChestnutRecovery(spawn=lambda: time.sleep(0.5) or FakeSuccessor(), present=lambda: time.sleep(0.2) or True)
  rec.failed(0.)
  took = []
  for i in range(60):
    t0 = time.perf_counter()
    rec.step(2. + i * 0.05, False)
    took.append(time.perf_counter() - t0)
    time.sleep(0.02)
  assert max(took) < 0.01 and rec.state in (cr.LOADING, cr.LOADED)


def test_an_attempt_thread_that_cannot_leave_modelds_slot_is_a_failed_attempt_not_a_stuck_recovery():
  # on the comma attempt_thread's sched_setscheduler/sched_setaffinity can raise (EINVAL from a cpuset): in the real thread
  # that once killed it before its result, the recovery held in STARTING for the drive with no note and no retry. Now it
  # spawns nothing (the successor would inherit core 7 at FIFO 54) and the next slot retries
  spawned, notes = [], []
  rec = cr.ChestnutRecovery(spawn=lambda: spawned.append(1) or FakeSuccessor(), present=lambda: True,
                            note=lambda kind, why: notes.append((kind, why)))
  with mock.patch.object(cr, 'attempt_thread', side_effect=OSError(22, 'Invalid argument')):
    rec.failed(0.)
    t = 2.
    while rec.state != cr.WAITING or not notes:
      rec.step(t, False)
      t += 0.05
      time.sleep(0.005)
      assert t < 5., f"stuck in {rec.state}"
  assert spawned == [] and notes[0][0] == 'retry failed' and 'thread priority failed' in notes[0][1]
  assert rec.failures == 1 and rec.next_t == pytest.approx(t - 0.05 + cr.retry_delay(1), abs=0.06)


@pytest.mark.parametrize("close_handle", [False, True])
def test_the_dead_chestnut_goes_inert_then_is_let_go_of(close_handle):
  fd = os.open(os.devnull, os.O_RDONLY)
  usb = SimpleNamespace(handle='the handle')
  allocator = SimpleNamespace(free=mock.Mock(), do_free=mock.Mock(), _unmap=mock.Mock(), free_cache=mock.Mock())
  dev = SimpleNamespace(iface=SimpleNamespace(pci_dev=SimpleNamespace(usb=SimpleNamespace(usb=usb), lock_fd=fd)), allocator=allocator,
                        synchronize=mock.Mock(), finalize=mock.Mock())
  device = mock.MagicMock()
  device.canonicalize.side_effect = lambda name: name.upper()
  device.__getitem__.side_effect = lambda name: dev
  device._opened_devices = {'AMD', 'CPU'}
  libusb = SimpleNamespace(libusb_release_interface=mock.Mock(return_value=0), libusb_close=mock.Mock())
  with mock.patch('tinygrad.device.Device', device), mock.patch.dict(sys.modules, {'tinygrad.runtime.autogen.libusb': libusb}), \
       mock.patch('tinygrad.runtime.autogen.libusb', libusb, create=True):
    cr.make_inert(SimpleNamespace(model_device='amd'))
    # a stray free or synchronize from now on (a dead buffer's last reference) does nothing: no wait on a dead link
    dev.synchronize(), dev.finalize(), allocator.free('storage', 8), allocator.do_free('storage', None)
    assert device._opened_devices == {'CPU'} and libusb.libusb_release_interface.call_count == 0
    done = cr.abandon_chestnut(SimpleNamespace(model_device='amd'), close_handle=close_handle)
  assert done == ('interface released, handle closed, lock closed' if close_handle else 'interface released, lock closed')
  libusb.libusb_release_interface.assert_called_once_with('the handle', 0)
  assert libusb.libusb_close.call_count == int(close_handle)
  assert dev.iface.pci_dev.lock_fd is None
  with pytest.raises(OSError):
    os.fstat(fd)
  cr.make_inert(None)  # never raises
  assert cr.abandon_chestnut(None).startswith('AttributeError')


def link_pair(exit_fn):
  to_child_r, to_child_w = os.pipe()
  from_child_r, from_child_w = os.pipe()
  link = cr.SuccessorLink(f'{to_child_r},{from_child_w}', exit_fn=exit_fn)
  return link, to_child_w, from_child_r


def read_lines(fd, n, timeout=2.):
  buf, end = b'', time.monotonic() + timeout
  while buf.count(b'\n') < n and time.monotonic() < end:
    buf += os.read(fd, 4096)
  return buf.decode().splitlines()


def wait_for(cond, timeout=2.):
  end = time.monotonic() + timeout
  while not cond() and time.monotonic() < end:
    time.sleep(0.005)
  return cond()


def test_the_successors_end_runs_only_when_probed_and_goes_live_only_after_a_sane_frame():
  exited = []
  link, to_child, from_child = link_pair(exited.append)
  link.loaded()
  assert read_lines(from_child, 1) == ['loaded'] and not link.runs
  os.write(to_child, b'go\n')  # a go before any sane probe goes nowhere
  assert wait_for(lambda: not link.cmds.empty()) and not link.poll() and not link.live
  os.write(to_child, b'probe\n')
  assert wait_for(lambda: not link.cmds.empty())
  assert not link.poll() and link.runs and not link.live
  link.ran()
  link.ran()
  assert read_lines(from_child, 1) == ['sane']  # once a probe
  os.write(to_child, b'hold\n')
  assert wait_for(lambda: not link.cmds.empty())
  link.poll()
  assert not link.runs
  os.write(to_child, b'probe\n')
  assert wait_for(lambda: not link.cmds.empty())
  link.poll()
  link.ran()
  os.write(to_child, b'go\n')
  assert wait_for(lambda: not link.cmds.empty())
  assert link.poll() and link.live and link.runs
  link.failed('run: RuntimeError: chestnut failed\nwith -1')
  assert read_lines(from_child, 2) == ['sane', 'failed run: RuntimeError: chestnut failed with -1']  # one line, whatever the why
  os.close(to_child)  # the parent gone: the successor ends, so the chestnut is free
  assert wait_for(lambda: exited == [0])


CHILD = """
import sys, time
from openpilot.selfdrive.modeld.chestnut_recovery import SuccessorLink
link = SuccessorLink(sys.argv[-1])
time.sleep(float(sys.argv[1]))  # its load
link.loaded()
while True:
  went = link.poll()
  if link.runs:
    link.ran()
  if went:
    break
  time.sleep(0.01)
time.sleep(0.3)  # live
link.failed('run: synthetic')
sys.exit(3)
"""


def test_a_real_successor_process_through_load_probe_handoff_and_exit():
  cmd = lambda load_s: [sys.executable, '-c', CHILD, str(load_s)]  # noqa: E731
  rec = cr.ChestnutRecovery(spawn=lambda: cr.Successor.spawn(cmd(0.3)), present=lambda: True)
  rec.failed(time.monotonic() - 2.)
  end = time.monotonic() + 20.
  while not rec.step(time.monotonic(), False):
    assert time.monotonic() < end, rec.state
    time.sleep(0.02)
  rec.go(time.monotonic())
  why = rec.standby(lambda: True)
  assert why == 'successor exited with 3: run: synthetic'
  rec.failed(time.monotonic(), why=why)
  assert rec.state == cr.WAITING and rec.failures == 1


def test_stop_kills_a_successor_at_once():
  s = cr.Successor.spawn([sys.executable, '-c', 'import time; time.sleep(60)'])
  t0 = time.monotonic()
  s.stop()
  assert time.monotonic() - t0 < 0.2
  assert wait_for(lambda: s.exited() is not None, 5.) and s.exited() < 0
