#!/usr/bin/env python3
"""The mid-drive recovery (selfdrive/modeld/chestnut_recovery.py) on a real chestnut: load the big model as modeld does, force a
runtime failure by raising from its runner, and bring it back through ChestnutRecovery in a successor process, which must open
the chestnut this process still holds the dead model of (abandon_chestnut), reload it, match this process's output on the same
inputs, and run on frames once probed and handed over. Synthetic frames stand in for camerad's, and the parent's loop for
modeld's: its step() times say whether the recovery ever held it up.

With --modeld it runs the real modeld --demo instead (as gate2_modeld does, against fake 1344x760 cameras in its own
OPENPILOT_PREFIX), its big model's runner raising once after --fail-after runs: modelV2 must go big, small (at once, no gap),
then big again from the successor modeld it spawns, ChestnutActive with it; and stopping modeld must leave no successor and
the chestnut free. It needs the small model's pkls (driving_tinygrad.pkl and its warp) in selfdrive/modeld/models.

Run it with nothing else holding the chestnut (modeld, the manager, another check):
  python -m openpilot.tools.turnpilot.chestnut_recovery_check [--width 1344 --height 760]
  python -m openpilot.tools.turnpilot.chestnut_recovery_check --modeld [--detector] [--fail-after 200] [--seconds 120]
"""
import argparse
import os
import sys
import tempfile
import time

os.environ['SEND_RAW_PRED'] = '1'  # modeld reads it at import: the raw output, to compare the two processes' bit for bit

import numpy as np

from openpilot.selfdrive.modeld.modeld import ModelState  # sets modeld's tinygrad env before tinygrad loads
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.helpers import chestnut_compiled, chestnut_present, wait_for_chestnut
from openpilot.selfdrive.modeld import chestnut_recovery as cr

LIVE_FRAMES = 40  # the successor's frames once handed over, then it exits
BIG_AGAIN_S = 15.  # --modeld: the big model back this long, past the parent's stall watchdog (cr.STALL_S)


def load(width: int, height: int) -> ModelState:
  from tinygrad.runtime.ops_amd import AMDDevice
  AMDDevice.wait_timeout_ms = 3000
  wait_for_chestnut()
  m = ModelState(width, height, True)
  m.warmup()
  return m


def reference(m: ModelState) -> np.ndarray:
  """The raw output on zero frames from a zero state, as warmup() runs it: the same in any process with the weights intact."""
  frames = {k: np.zeros(m.frame_copy_size, dtype=np.uint8) for k in m.vision_input_names}
  dims = {'desire_pulse': ModelConstants.DESIRE_LEN, 'traffic_convention': 2, 'action_t': 2}
  out = m.run(frames, dict.fromkeys(m.vision_input_names, np.eye(3, dtype=np.float32)), {k: np.zeros(v, dtype=np.float32) for k, v in dims.items()})
  m.packed_input[:] = 0
  for key in m.state_pairs:
    m.input_queues[key].assign(0).realize()
  m.prev_desire[:] = 0
  return out['raw_pred'].copy()


def frame(m: ModelState, rng) -> tuple:
  frames = {k: rng.integers(0, 255, m.frame_copy_size, dtype=np.uint8) for k in m.vision_input_names}
  inputs = {'desire_pulse': np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32), 'traffic_convention': np.array([1, 0], dtype=np.float32),
            'action_t': np.array([0.3, 0.5], dtype=np.float32)}
  return frames, dict.fromkeys(m.vision_input_names, np.eye(3, dtype=np.float32)), inputs


def child(fds: str, width: int, height: int, ref_path: str) -> None:
  """The successor's part, as modeld --successor plays it: load, say loaded, run only when probed, go live on go."""
  link = cr.SuccessorLink(fds)
  t0 = time.monotonic()
  try:
    m = load(width, height)
  except Exception as e:
    link.failed(f"load: {type(e).__name__}: {e}")
    os._exit(1)
  diff = float(np.abs(reference(m) - np.load(ref_path)).max())
  print(f"[successor pid {os.getpid()}] reloaded in {time.monotonic() - t0:.1f} s, output vs the parent's: max diff {diff}", flush=True)
  if diff > 1e-3:
    link.failed(f"output differs from the parent's by {diff}")
    os._exit(1)
  link.loaded()
  rng, live, ts = np.random.default_rng(1), 0, []
  while live < LIVE_FRAMES:
    t = time.perf_counter()
    if link.poll():
      print("[successor] go: live", flush=True)
    if link.runs:
      try:
        m.run(*frame(m, rng))
      except Exception as e:
        link.failed(f"run: {type(e).__name__}: {e}")
        os._exit(1)
      link.ran()
      if link.live:
        live += 1
        ts.append(time.perf_counter() - t)
    time.sleep(max(0., 0.05 - (time.perf_counter() - t)))
  print(f"[successor] {live} live frames, model.run median {np.median(ts) * 1e3:.1f} ms, max {max(ts) * 1e3:.1f} ms", flush=True)
  os._exit(0)


def parent(width: int, height: int) -> int:
  print(f"chestnut_present={chestnut_present()} chestnut_compiled={chestnut_compiled()}")
  assert chestnut_present() and chestnut_compiled()
  t0 = time.monotonic()
  m = load(width, height)
  print(f"[parent pid {os.getpid()}] big model loaded in {time.monotonic() - t0:.1f} s on {m.model_device}")
  ref = reference(m)
  rng = np.random.default_rng(0)
  ts = []
  for _ in range(20):
    t = time.perf_counter()
    m.run(*frame(m, rng))
    ts.append(time.perf_counter() - t)
  print(f"[parent] 20 frames, model.run median {np.median(ts) * 1e3:.1f} ms")

  def forced(*args, **kwargs):
    raise RuntimeError("forced: chestnut failed with -1")
  m.run_model = forced  # the runner raises, as a moved cable's link error does in modeld
  try:
    m.run(*frame(m, rng))
    print("[parent] FAIL: the forced runner didn't raise")
    return 1
  except RuntimeError as e:
    print(f"[parent] big model failed: {e}")

  with tempfile.NamedTemporaryFile(suffix='.npy', delete=False) as f:
    ref_path = f.name
  np.save(ref_path, ref)
  cmd = [sys.executable, '-m', 'openpilot.tools.turnpilot.chestnut_recovery_check', '--width', str(width), '--height', str(height),
         '--ref', ref_path, '--child']
  notes: list = []
  rec = cr.ChestnutRecovery(spawn=lambda: cr.Successor.spawn(cmd), present=chestnut_present, note=lambda kind, why: notes.append((kind, why)))
  failed_t = time.monotonic()
  rec.failed(failed_t, m)  # what modeld does on the failure: the model kept, the chestnut let go of before the spawn
  steps, handed, said = [], None, 0
  try:
    while time.monotonic() - failed_t < cr.LOAD_DEADLINE_S + 30.:
      t = time.perf_counter()
      go = rec.step(time.monotonic(), False)  # openpilot disengaged
      steps.append(time.perf_counter() - t)
      if go:
        handed = time.monotonic()
        rec.go(handed)
        break
      while len(notes) > said:
        print(f"[parent] {notes[said]}")
        said += 1
      time.sleep(0.05)  # modeld's loop at 20 Hz, on the small model meanwhile
    if handed is None:
      print(f"[parent] FAIL: no handoff, state {rec.state}, notes {notes}")
      return 1
    why = rec.standby(lambda: True)
  finally:
    rec.close()
    os.unlink(ref_path)
  print(f"[parent] handed over {handed - failed_t:.1f} s after the failure; standby ended: {why}")
  print(f"[parent] step() over {len(steps)} loop frames: max {max(steps) * 1e3:.2f} ms")
  print(f"[parent] notes: {notes}")
  ok = why == 'successor exited with 0' and max(steps) < 0.01 and notes[-1][0] == 'recovered'
  print("PASS" if ok else "FAIL")
  return 0 if ok else 1


# modeld itself, its big model's runner raising once: what a moved cable's link error does in modeld.run
FORCED_MODELD = """
import sys
import openpilot.selfdrive.modeld.modeld as md
fail_at, runs, run = int(sys.argv[1]), [0], md.ModelState.run
def forced(self, *args, **kwargs):
  if self.chestnut:
    runs[0] += 1
    if runs[0] == fail_at:
      raise RuntimeError("forced: chestnut failed with -1")
  return run(self, *args, **kwargs)
md.ModelState.run = forced
md.main(demo=True)
"""


def successors_alive() -> list[str]:
  import subprocess
  out = subprocess.run(['ps', '-axo', 'pid,command'], capture_output=True, text=True).stdout
  return [line.strip() for line in out.splitlines() if 'modeld' in line and '--successor' in line and 'ps -axo' not in line]


def modeld_check(fail_after: int, seconds: float, detector: bool) -> int:
  """The real modeld --demo: big, then small at the forced failure, then big again from its successor for BIG_AGAIN_S, past
  the parent's stall watchdog (chestnut_recovery.STALL_S: it hears the successor's modelV2, or kills it), one successor
  throughout; then stopped clean. detector: TurnPilotEnabled, so modeld runs the TurnPilot detector, and the successor's
  detections must follow the handoff."""
  import signal
  import subprocess
  from openpilot.cereal import messaging
  from openpilot.common.params import Params
  from openpilot.common.realtime import Ratekeeper
  from openpilot.tools.turnpilot.fake_cameras import FakeCameras, gray_nv12
  from openpilot.tools.turnpilot.gate2_modeld import chestnut_user_clients

  print(f"chestnut_present={chestnut_present()} chestnut_compiled={chestnut_compiled()}")
  assert chestnut_present() and chestnut_compiled()
  clients = chestnut_user_clients()  # macOS's own, with nothing of ours open (3 on the bench Mac)
  cams = FakeCameras()
  pm = messaging.PubMaster(['deviceState', 'carState', 'extrinsicsCalibration'])
  model_sock = messaging.sub_sock('modelV2', conflate=False)
  det_sock = messaging.sub_sock('turnpilotDetections', conflate=False)
  params = Params()
  params.remove("ChestnutRetry")
  params.put_bool("TurnPilotEnabled", detector)
  env = {k: v for k, v in os.environ.items() if k != 'SEND_RAW_PRED'}
  modeld = subprocess.Popen([sys.executable, '-c', FORCED_MODELD, str(fail_after)], env=env)
  road, wide = gray_nv12(seed=0), gray_nv12(seed=1)
  rk = Ratekeeper(20, print_delay_threshold=None)
  st = time.monotonic()
  seen: list[tuple[float, bool]] = []  # (time, modelV2.big)
  active: list[tuple[float, object]] = []  # ChestnutActive as it changes
  pids: set[str] = set()  # the successors seen while the big model is back
  dets: list[float] = []  # turnpilotDetections' times
  try:
    while time.monotonic() - st < seconds and modeld.poll() is None:
      cams.send(road, wide)
      if rk.frame % 10 == 0:
        ds = messaging.new_message('deviceState', valid=True)
        ds.deviceState.deviceType = "pc"
        pm.send('deviceState', ds)
        a = params.get("ChestnutActive")
        if not active or active[-1][1] != a:
          active.append((round(time.monotonic() - st, 1), a))
      if rk.frame % 5 == 0:
        cal = messaging.new_message('extrinsicsCalibration', valid=True)
        cal.extrinsicsCalibration.rpyCalib = [0., 0., 0.]
        cal.extrinsicsCalibration.wideFromDeviceEuler = [0., 0., 0.]
        cal.extrinsicsCalibration.calStatus = "calibrated"
        pm.send('extrinsicsCalibration', cal)
      pm.send('carState', messaging.new_message('carState', valid=True))
      for m in messaging.drain_sock(model_sock):
        big = bool(m.modelV2.big)
        if not seen or seen[-1][1] != big:
          print(f"[check] {time.monotonic() - st:6.1f} s: modelV2 from the {'big' if big else 'small'} model", flush=True)
        seen.append((time.monotonic() - st, big))
      dets += [time.monotonic() - st for _ in messaging.drain_sock(det_sock)]
      smalls = [t for t, b in seen if not b]
      if smalls and seen[-1][1]:
        if rk.frame % 20 == 0:
          pids |= {line.split()[0] for line in successors_alive()}
        if seen[-1][0] - smalls[-1] > BIG_AGAIN_S:
          break
      rk.keep_time()
  finally:
    modeld.send_signal(signal.SIGINT)
    try:
      modeld.wait(15)
    except subprocess.TimeoutExpired:
      modeld.kill()
      modeld.wait()
  time.sleep(1.)
  runs = [b for _, b in seen]
  order = [b for i, b in enumerate(runs) if i == 0 or runs[i - 1] != b]
  t_fail = next((t for t, b in seen if not b), None)
  t_back = next((t for t, b in seen if b and t_fail is not None and t > t_fail), None)
  gaps = [(b[0], b[0] - a[0]) for a, b in zip(seen, seen[1:], strict=False)]
  around = [g for t, g in gaps if t_fail is not None and t_back is not None and t_fail - 1. < t < t_back + 2.] or [float('inf')]
  print(f"[check] modelV2: {len(seen)} messages, models in order {['big' if b else 'small' for b in order]}; ChestnutActive {active}")
  if t_fail is not None and t_back is not None:
    print(f"[check] big model back {t_back - t_fail:.1f} s after the failure; largest modelV2 gap from the failure to past the " +
          f"handoff {max(around) * 1e3:.0f} ms (overall {max(g for _, g in gaps) * 1e3:.0f} ms)")
  left = successors_alive()
  big_again = seen[-1][0] - t_back if t_back is not None else 0.
  print(f"[check] big again {big_again:.1f} s (the stall watchdog: {cr.STALL_S:.0f} s), successor pids then {sorted(pids)}, " +
        f"after modeld stopped {len(left)}; modeld exit {modeld.returncode}; chestnut user clients {chestnut_user_clients()} ({clients} before)")
  after = [t for t in dets if t_back is not None and t > t_back]
  if detector:
    before = [t for t in dets if t_fail is not None and t < t_fail]
    first = f" (the first {after[0] - t_back:.1f} s after it)" if after else ""
    print(f"[check] turnpilotDetections: {len(before)} before the failure, {len(dets) - len(before) - len(after)} on the small model, " +
          f"{len(after)} after the handoff{first}")
  values = [a for _, a in active]
  ok = order == [True, False, True] and max(around) < 0.5 and big_again > cr.STALL_S and len(pids) == 1 and not left and \
       chestnut_user_clients() == clients and False in values and values[-1] is True and (not detector or len(after) > 0)
  print("PASS" if ok else "FAIL")
  return 0 if ok else 1


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--width', type=int, default=1344)
  ap.add_argument('--height', type=int, default=760)
  ap.add_argument('--modeld', action='store_true', help='the real modeld --demo against fake cameras')
  ap.add_argument('--fail-after', type=int, default=200, help='--modeld: the big model runs before its runner raises')
  ap.add_argument('--seconds', type=float, default=120.)
  ap.add_argument('--detector', action='store_true', help='--modeld: set TurnPilotEnabled, so modeld runs the TurnPilot detector too')
  ap.add_argument('--ref')
  ap.add_argument('--child', help=argparse.SUPPRESS)
  args = ap.parse_args()
  if args.child:
    child(args.child, args.width, args.height, args.ref)
  elif args.modeld:
    from openpilot.common.prefix import OpenpilotPrefix
    with OpenpilotPrefix("tprecovery"):  # its own params and msgq, cleaned up on exit
      code = modeld_check(args.fail_after, args.seconds, args.detector)
    sys.exit(code)
  else:
    code = parent(args.width, args.height)
    sys.stdout.flush()
    os._exit(code)  # the dead model's device was let go of: nothing to finalize


if __name__ == "__main__":
  main()
