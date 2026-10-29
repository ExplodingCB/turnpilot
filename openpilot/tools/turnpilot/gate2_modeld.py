#!/usr/bin/env python3
"""Gate 2: run the real modeld against fake 1344x760 os04c10 cameras and measure modelV2.modelExecutionTime.

It publishes the minimal inputs modeld needs (road and wide frames, camera states, deviceState pc,
carState, extrinsicsCalibration), starts modeld --demo in its own OPENPILOT_PREFIX, collects modelV2,
then stops modeld and checks that chestnut is released. Nothing else may hold chestnut while it runs.
With --detector it sets TurnPilotEnabled, so modeld also runs the TurnPilot detector, and reports its time and rate, and
the far crop's (turnpilotFarDetections, DESIGN phase 4 G5) when its pkl is in selfdrive/turnpilot/models: its runs on the
frames between the full pass's, its time, and what the full pass's messages say it is doing (farStatus).
With --sweep stopSweep or stopSweepGo it also publishes a turnpilotPlan asking for that stop sweep (STOP_SIGNS.md section
1.3) and carState at --v-ego, and reports the side views' runs (view sideLeft or sideRight), their time and sectors' time,
and what every message says they are doing (sideViewStatus), when the side pkl is in selfdrive/turnpilot/models.
  python -m openpilot.tools.turnpilot.gate2_modeld [--frames 600] [--detector] [--sweep stopSweep] [--v-ego 0]
"""
import argparse
import signal
import subprocess
import sys
import time

import numpy as np

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.common.realtime import Ratekeeper
from openpilot.tools.turnpilot.fake_cameras import FakeCameras, gray_nv12

WARMUP = 20


def chestnut_user_clients() -> int:
  out = subprocess.run(["/usr/sbin/ioreg", "-p", "IOService", "-l", "-w0", "-r", "-n", "custom 8516b768-CLEAN"],
                       capture_output=True, text=True).stdout
  return out.count("UserClient")


def pct(a, q):
  return float(np.percentile(a, q))


def run(n_frames: int, timeout: float, detector: bool, sweep: str | None = None, v_ego: float = 0.) -> int:
  cams = FakeCameras()
  pm = messaging.PubMaster(['deviceState', 'carState', 'extrinsicsCalibration', 'turnpilotPlan'])
  model_sock = messaging.sub_sock('modelV2', conflate=False)
  gpu_sock = messaging.sub_sock('chestnutGpuState', conflate=False)
  det_sock = messaging.sub_sock('turnpilotDetections', conflate=False)
  far_sock = messaging.sub_sock('turnpilotFarDetections', conflate=False)
  params = Params()
  params.remove("ChestnutRetry")
  params.put_bool("TurnPilotEnabled", detector)

  modeld = subprocess.Popen([sys.executable, "-m", "openpilot.selfdrive.modeld.modeld", "--demo"])
  road, wide = gray_nv12(seed=0), gray_nv12(seed=1)
  msgs, dets, fars, gpu_msgs, sent_at_first = [], [], [], 0, None
  rk = Ratekeeper(20, print_delay_threshold=None)
  st = time.monotonic()
  try:
    while len(msgs) < WARMUP + n_frames and time.monotonic() - st < timeout and modeld.poll() is None:
      cams.send(road, wide)
      if rk.frame % 10 == 0:
        ds = messaging.new_message('deviceState', valid=True)
        ds.deviceState.deviceType = "pc"
        pm.send('deviceState', ds)
      if rk.frame % 5 == 0:
        cal = messaging.new_message('extrinsicsCalibration', valid=True)
        cal.extrinsicsCalibration.rpyCalib = [0., 0., 0.]
        cal.extrinsicsCalibration.wideFromDeviceEuler = [0., 0., 0.]
        cal.extrinsicsCalibration.calStatus = "calibrated"
        pm.send('extrinsicsCalibration', cal)
      cs = messaging.new_message('carState', valid=True)
      cs.carState.vEgo = v_ego
      pm.send('carState', cs)
      if sweep is not None:
        plan = messaging.new_message('turnpilotPlan', valid=True)
        plan.turnpilotPlan.shadow, plan.turnpilotPlan.detectorView = True, sweep
        pm.send('turnpilotPlan', plan)

      for m in messaging.drain_sock(model_sock):
        if not msgs:
          sent_at_first = cams.frame_id
          print(f"first modelV2 after {time.monotonic() - st:.1f} s, big={m.modelV2.big}", flush=True)
        msgs.append(m)
      gpu_msgs += len(messaging.drain_sock(gpu_sock))
      dets += [m for m in messaging.drain_sock(det_sock) if msgs and len(msgs) > WARMUP]
      fars += [m for m in messaging.drain_sock(far_sock) if msgs and len(msgs) > WARMUP]
      rk.keep_time()
    chestnut_active = params.get("ChestnutActive")
  finally:
    modeld.send_signal(signal.SIGINT)
    try:
      modeld.wait(15)
    except subprocess.TimeoutExpired:
      modeld.kill()
      modeld.wait()

  if len(msgs) < WARMUP + n_frames:
    print(f"FAIL: got {len(msgs)} modelV2 messages, wanted {WARMUP + n_frames}")
    return 1

  meas = msgs[WARMUP:]
  ex = np.array([m.modelV2.modelExecutionTime for m in meas]) * 1e3
  age = np.array([(m.logMonoTime - m.modelV2.timestampEof) for m in meas]) / 1e6
  drop = np.array([m.modelV2.frameDropPerc for m in meas])
  ids = np.array([m.modelV2.frameId for m in meas])
  gaps = int(np.sum(np.diff(ids) - 1))
  big = all(m.modelV2.big for m in meas)
  sent = cams.frame_id - sent_at_first
  print(f"modelV2.big all True: {big}  ChestnutActive: {chestnut_active}  chestnutGpuState msgs: {gpu_msgs}")
  print(f"modelExecutionTime over {len(ex)} frames: median {np.median(ex):.2f} ms, p95 {pct(ex, 95):.2f} ms, " +
        f"p99 {pct(ex, 99):.2f} ms, max {ex.max():.2f} ms")
  print(f"frame age (modelV2 logMonoTime - frame eof): median {np.median(age):.2f} ms, p95 {pct(age, 95):.2f} ms")
  if (slow := np.flatnonzero(ex > 45)).size or gaps:
    print(f"model frames over 45 ms at {slow.tolist()[:10]}, frameId gaps after {np.flatnonzero(np.diff(ids) > 1).tolist()[:10]}")
  print(f"frame drop: frameId gaps {gaps} of {ids[-1] - ids[0] + 1}, frameDropPerc max {drop.max():.2f}, " +
        f"modelV2 {len(msgs)} for {sent} frames sent since the first output")
  if detector:
    det_ex = np.array([d.turnpilotDetections.executionTime for d in dets]) * 1e3
    rate = len(dets) / len(meas) * 20
    print(f"turnpilotDetections: {len(dets)} over {len(meas)} model frames ({rate:.2f} Hz at 20 Hz frames)" +
          (f", executionTime median {np.median(det_ex):.2f} ms, p95 {pct(det_ex, 95):.2f} ms, max {det_ex.max():.2f} ms" if len(dets) else ""))
    far_ex = np.array([f.turnpilotFarDetections.run.executionTime for f in fars]) * 1e3
    statuses = {}
    for d in dets:
      statuses[str(d.turnpilotDetections.farStatus)] = statuses.get(str(d.turnpilotDetections.farStatus), 0) + 1
    print(f"turnpilotFarDetections: {len(fars)} over {len(meas)} model frames ({len(fars) / len(meas) * 20:.2f} Hz)" +
          (f", executionTime median {np.median(far_ex):.2f} ms, p95 {pct(far_ex, 95):.2f} ms, max {far_ex.max():.2f} ms" if len(fars) else "") +
          f"; farStatus on the full pass's messages {statuses}")
    views = {}
    for d in dets:
      td = d.turnpilotDetections
      views.setdefault(f"{td.camera}/{td.view}", []).append(td)
    for key, tds in sorted(views.items()):
      vex = np.array([td.executionTime for td in tds]) * 1e3
      sec = np.array([td.sectorTime for td in tds]) * 1e3
      print(f"  {key}: {len(tds)} runs ({len(tds) / len(meas) * 20:.2f} Hz), executionTime median {np.median(vex):.2f} ms, " +
            f"p95 {pct(vex, 95):.2f} ms, max {vex.max():.2f} ms" + (f", sectorTime median {np.median(sec):.2f} ms" if sec.any() else ""))
    side = {}
    for d in dets:
      side[str(d.turnpilotDetections.sideViewStatus)] = side.get(str(d.turnpilotDetections.sideViewStatus), 0) + 1
    print(f"sideViewStatus on every message {side}")
  print(f"modeld exit code {modeld.returncode}, chestnut user clients after exit: {chestnut_user_clients()}")
  return 0 if big and chestnut_active and (not detector or dets) else 1


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--frames', type=int, default=600)
  parser.add_argument('--timeout', type=float, default=240.)
  parser.add_argument('--detector', action='store_true', help='set TurnPilotEnabled, so modeld runs the detector too')
  parser.add_argument('--sweep', choices=('stopSweep', 'stopSweepGo'), help='ask for this stop sweep (a shadow turnpilotPlan)')
  parser.add_argument('--v-ego', type=float, default=0., help='carState.vEgo, m/s (the side views run at 2.0 m/s or under)')
  args = parser.parse_args()
  # its own prefix: the exit cleanup deletes the prefix's params, msgq and logs
  with OpenpilotPrefix("gate2"):
    sys.exit(run(args.frames, args.timeout, args.detector, args.sweep, args.v_ego))


if __name__ == "__main__":
  main()
