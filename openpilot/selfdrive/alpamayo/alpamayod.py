#!/usr/bin/env python3
"""alpamayod: run Alpamayo 2 Edge on the comma road cameras and publish its plan.

Publishes alpamayoModelV2 (modelV2 format, device point in the calib frame like
stock modeld) and alpamayoOutput (reasoning text, raw rear-axle trajectory).
controlsd decides whether the plan steers (AlpamayoMode) and handles hold,
time shift, staleness and fallback.

Backends (ALPAMAYO_BACKEND):
  tinygrad  in process; DEV 'USB+AMD:LLVM' when a chestnut is present (default on device)
  torch     PC only: sends samples to alpacomma/torch_server.py over a unix socket
"""
from __future__ import annotations

import collections
import io
import math
import os
import json
import socket
import struct
import threading
import time

import numpy as np

import openpilot.cereal.messaging as messaging
from msgq.visionipc import VisionIpcClient
from openpilot.cereal.visionipc import VisionStreamType
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.locationd.helpers import Pose, PoseCalibrator
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.alpamayo import camera_adapter as ca

# chestnut setup like modeld, set before anything imports tinygrad. On the device, kernel search
# results persist across drives in /data. 2 cameras: about 25% faster plans; latency matters most here.
_ENV = {"GMMU": "0", "AM_POWER_LIMIT": "100", "JITBEAM": "2", "ALPA_CAMERAS": "2"}
if os.path.isdir("/data/openpilot"):
  _ENV["CACHEDB"] = "/data/tinygrad_alpamayo_cache.db"
for _k, _v in _ENV.items():
  os.environ.setdefault(_k, _v)

HISTORY_S = 2.2
# rear axle to comma mount, Tesla Model 3 default (wheelbase 2.875 m, mount ~1 m behind the front axle)
DEV_TO_REAR_M = float(os.environ.get("ALPAMAYO_DEV_TO_REAR_M", "1.9"))
NAV_DEFAULT = "Continue straight"


# ------------------------------------------------------------------ ego motion
class EgoTrack:
  """Planar rear-axle dead reckoning (FLU, yaw left positive) from vEgo and calibrated yaw rate."""

  def __init__(self):
    self.t = collections.deque()
    self.pose = collections.deque()
    self.x = self.y = self.yaw = 0.0
    self.last_t: int | None = None
    self.yaw_rate_flu = 0.0

  def update(self, t_ns: int, v: float):
    if self.last_t is not None:
      dt = (t_ns - self.last_t) * 1e-9
      if 0 < dt < 0.5:
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += self.yaw_rate_flu * dt
    self.last_t = t_ns
    self.t.append(t_ns)
    self.pose.append((self.x, self.y, self.yaw))
    while self.t and t_ns - self.t[0] > 3e9:
      self.t.popleft()
      self.pose.popleft()

  def history(self, t0_ns: int):
    """20 poses at -1.9..0 s in the rear-axle frame at t0, or None if not covered."""
    if not self.t or self.t[0] > t0_ns - 1.95e9:
      return None
    ts = np.array(self.t, dtype=np.float64)
    p = np.array(self.pose)
    q = t0_ns + np.arange(-19, 1) * 1e8
    xs, ys, ws = (np.interp(q, ts, p[:, i]) for i in range(3))
    c, s = math.cos(-ws[-1]), math.sin(-ws[-1])
    dx, dy = xs - xs[-1], ys - ys[-1]
    xyz = np.stack([c * dx - s * dy, s * dx + c * dy, np.zeros(20)], -1)
    yaw = ws - ws[-1]
    rot = np.zeros((20, 3, 3))
    rot[:, 0, 0] = rot[:, 1, 1] = np.cos(yaw)
    rot[:, 0, 1], rot[:, 1, 0] = -np.sin(yaw), np.sin(yaw)
    rot[:, 2, 2] = 1
    return xyz, rot


# ------------------------------------------------------------------ frames
class FrameRing:
  """Recent camera frames (NV12 copies at ~10 Hz) and a cache of their slot images."""

  def __init__(self):
    self.frames: collections.deque = collections.deque()  # (t_sof_ns, frame_id, nv12, meta)
    self.slot_cache: dict[tuple[int, int], np.ndarray] = {}

  def push(self, t_ns, frame_id, buf):
    if self.frames and t_ns - self.frames[-1][0] < 0.08e9:
      return  # keep ~10 Hz
    self.frames.append((t_ns, frame_id, np.array(buf.data, copy=True), (buf.width, buf.height, buf.stride, buf.uv_offset)))
    while self.frames and t_ns - self.frames[0][0] > HISTORY_S * 1e9:
      self.frames.popleft()
    alive = {f[1] for f in self.frames}
    for k in [k for k in self.slot_cache if k[0] not in alive]:
      del self.slot_cache[k]

  def at_or_before(self, t_ns):
    best = None
    for f in self.frames:
      if f[0] <= t_ns + 2e6:
        best = f
    return best

  def slot_image(self, frame, slot, maps: ca.SlotMaps):
    key = (frame[1], slot, maps._key)
    if key not in self.slot_cache:
      w, h, stride, uv_off = frame[3]
      x, y = maps.maps[slot]
      self.slot_cache[key] = ca.remap_nv12(frame[2], w, h, stride, uv_off, x, y)
    return self.slot_cache[key]


def build_inputs(maps: ca.SlotMaps, rings, t0_ns):
  fill = np.full((*ca.SLOT_HW, 3), maps.filler_value(), np.uint8)
  imgs, cams, stamps, keys = [], [], [], []
  for cam in maps.slots():
    offsets = ca.FRONT_OFFSETS_US if cam in (ca.FRONT, ca.TELE) else ca.OTHER_OFFSETS_US
    for off in offsets:
      t = t0_ns + off * 1000
      if cam in (ca.FRONT, ca.TELE):
        f = rings[maps.source(cam)].at_or_before(t)
        if f is None:
          return None
        imgs.append(rings[maps.source(cam)].slot_image(f, cam, maps))
        stamps.append(f[0] // 1000)
        keys.append((maps.source(cam), f[1], maps._key))
      else:
        imgs.append(fill)
        stamps.append(t // 1000)
        keys.append(("fill", maps.filler_value()))
      cams.append(cam)
  return np.stack(imgs), np.array(cams, dtype=np.int64), np.array(stamps, dtype=np.int64), keys


# ------------------------------------------------------------------ backends
class TorchServerRunner:
  """PC development backend: alpacomma/torch_server.py holds the PyTorch model."""

  def __init__(self, path="/tmp/alpamayo_torch.sock"):
    self.path = path

  def run(self, imgs, cams, stamps, keys, ego_xyz, ego_rot, nav_text, mapping):
    buf = io.BytesIO()
    np.savez(buf, imgs=imgs, cams=cams, stamps=stamps, ego_xyz=ego_xyz, ego_rot=ego_rot,
             nav=np.array(nav_text), mapping=np.array(mapping))
    data = buf.getvalue()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
      s.connect(self.path)
      s.sendall(struct.pack("<Q", len(data)) + data)
      n = struct.unpack("<Q", _recv(s, 8))[0]
      out = np.load(io.BytesIO(_recv(s, n)))
      return {k: out[k] for k in out.files} | {"cot": str(out["cot"]), "backend": "torch"}


def _recv(s, n):
  b = bytearray()
  while len(b) < n:
    chunk = s.recv(n - len(b))
    if not chunk:
      raise ConnectionError("torch server closed")
    b += chunk
  return bytes(b)


class TinygradRunner:
  def __init__(self, dev: str):
    from tinygrad import Context  # noqa: F401
    from openpilot.selfdrive.alpamayo.tg.alpamayo_tg import AlpamayoTG
    self.dev = dev
    self.m = AlpamayoTG(os.environ.get("ALPAMAYO_WEIGHTS", "/data/models/alpamayo2_edge"), dev)

  def run(self, imgs, cams, stamps, keys, ego_xyz, ego_rot, nav_text, mapping):
    out = self.m.run(imgs, cams, stamps, ego_xyz, ego_rot, nav_text, frame_keys=keys)
    return out | {"cot": out["cot_text"], "backend": f"tinygrad:{self.dev}"}


def warmup(runner) -> float:
  """One inference on black frames from a straight 5 m/s history: compiles, and with JITBEAM searches kernels."""
  cams, stamps = [], []
  for cam in ca.CAM_IDS:
    for off in (ca.FRONT_OFFSETS_US if cam in (ca.FRONT, ca.TELE) else ca.OTHER_OFFSETS_US):
      cams.append(cam)
      stamps.append(10_000_000 + off)
  imgs = np.zeros((len(cams), *ca.SLOT_HW, 3), np.uint8)
  xyz = np.stack([np.arange(-19, 1) * 0.5, np.zeros(20), np.zeros(20)], -1)
  rot = np.repeat(np.eye(3)[None], 20, 0)
  st = time.monotonic()
  runner.run(imgs, np.array(cams), np.array(stamps), None, xyz, rot, NAV_DEFAULT, "warmup")
  return time.monotonic() - st


class Loader:
  """Gets a runner ready off the main loop: weights on disk, chestnut check, model load.

  status is shown in the 'Alpamayo Not Running' alert and logged in alpamayoOutput;
  report is the self-test JSON (also kept in the AlpamayoSelfTest param).
  """

  def __init__(self, params: Params):
    self.params = params
    self.status = "starting"
    self.report: dict = {}
    self.runner = None
    self.thread = threading.Thread(target=self._load, daemon=True)
    self.thread.start()

  def _load(self):
    try:
      backend = os.environ.get("ALPAMAYO_BACKEND", "tinygrad")
      if backend == "torch":
        self.runner = TorchServerRunner()
        return
      from openpilot.selfdrive.alpamayo.fetch import weights_ready
      while not weights_ready():
        self.status = self.params.get("AlpamayoStatus") or "waiting for weights download"
        time.sleep(1)
      from openpilot.selfdrive.modeld.helpers import chestnut_present
      dev = os.environ.get("ALPAMAYO_DEV") or ("USB+AMD:LLVM" if chestnut_present() else None)
      if dev is None:
        self.status = "no chestnut found"
        return
      self.report["dev"] = dev
      if dev.startswith("USB"):
        from openpilot.selfdrive.alpamayo.selftest import usb_check
        self.status = "checking chestnut USB"
        usb = usb_check(dev)
        self.report["usb"] = usb
        self._save()
        if not usb["ok"]:
          self.status = "chestnut USB check failed"
          return
      self.status = "loading model onto chestnut"
      t0 = time.monotonic()
      runner = TinygradRunner(dev)
      self.report["load_s"] = round(time.monotonic() - t0, 1)
      self._save()
      self.status = "tuning chestnut kernels (first start only, can take a long time)"
      t0 = time.monotonic()
      self.report["warmup_s"] = [round(warmup(runner), 2)]
      self.report["warmup_s"].append(round(warmup(runner), 2))
      self.report["tune_s"] = round(time.monotonic() - t0, 1)
      self._save()
      self.status = "warming up"
      self.runner = runner
    except Exception as e:
      cloudlog.exception("alpamayod: load failed")
      self.status = f"load failed: {type(e).__name__}: {e}"[:120]
      self.report["error"] = self.status
      self._save()

  def note_inference(self, exec_time: float, ok: bool):
    times = self.report.setdefault("infer_s", [])
    if len(times) < 10:
      times.append(round(exec_time, 2))
      self.report["infer_ok"] = self.report.get("infer_ok", 0) + int(ok)
      self._save()
    if ok:
      self.status = "ready"

  def _save(self):
    self.params.put("AlpamayoSelfTest", json.dumps(self.report))
    cloudlog.warning(f"alpamayod: self-test {json.dumps(self.report)}")


# ------------------------------------------------------------------ output
def fill_model_v2(msg, out, frame_id, t_eof, exec_time, lat_delay):
  """Rear-axle FLU trajectory -> device-point path in openpilot's calib frame on T_IDXS."""
  md = msg.alpamayoModelV2
  md.frameId = md.frameIdExtra = frame_id
  md.timestampEof = t_eof
  md.modelExecutionTime = exec_time
  xyz = np.asarray(out["xyz"], dtype=np.float64)
  t = np.arange(1, len(xyz) + 1) * 0.1
  t = np.concatenate([[0.0], t])
  xr = np.concatenate([[0.0], xyz[:, 0]])
  yr = np.concatenate([[0.0], xyz[:, 1]])
  if "rot" in out:
    rot = np.asarray(out["rot"], dtype=np.float64)
    yaw = np.concatenate([[0.0], np.unwrap(np.arctan2(rot[:, 1, 0], rot[:, 0, 0]))])
  else:  # heading from the path; fine above walking speed
    yaw = np.unwrap(np.arctan2(np.gradient(yr), np.gradient(xr)))
    yaw -= yaw[0]
  # device point sits DEV_TO_REAR_M ahead of the rear axle along the heading
  xd = xr + DEV_TO_REAR_M * np.cos(yaw) - DEV_TO_REAR_M
  yd = yr + DEV_TO_REAR_M * np.sin(yaw)
  T = np.array(ModelConstants.T_IDXS)
  # beyond the 6.4 s horizon continue at the last velocity and yaw rate
  v_end = np.hypot(xd[-1] - xd[-2], yd[-1] - yd[-2]) / 0.1
  te = np.concatenate([t, T[T > t[-1]]])
  extra = te[len(t):] - t[-1]
  yaw_e = yaw[-1] + (yaw[-1] - yaw[-2]) / 0.1 * extra
  xe = np.concatenate([xd, xd[-1] + v_end * extra * np.cos(yaw_e)])
  ye = np.concatenate([yd, yd[-1] + v_end * extra * np.sin(yaw_e)])
  ywe = np.concatenate([yaw, yaw_e])
  px, py, pyaw = np.interp(T, te, xe), -np.interp(T, te, ye), -np.interp(T, te, ywe)  # FLU -> calib (y right)
  vx = np.gradient(px, T)
  for field, vals in (("position", (px, py, np.zeros_like(T))), ("orientation", (np.zeros_like(T), np.zeros_like(T), pyaw)),
                      ("velocity", (vx, np.zeros_like(T), np.zeros_like(T)))):
    f = getattr(md, field)
    f.x, f.y, f.z = [float(v) for v in vals[0]], [float(v) for v in vals[1]], [float(v) for v in vals[2]]
    f.t = [float(v) for v in T]
  # curvature at the lateral delay, right positive like modeld
  kappa = np.asarray(out.get("kappa", np.zeros(len(xyz))), dtype=np.float64)
  md.action.desiredCurvature = float(-np.interp(lat_delay, np.arange(len(kappa)) * 0.1, kappa))


def nav_from_instruction(sm) -> str:
  if not sm.alive["navInstruction"]:
    return NAV_DEFAULT
  ni = sm["navInstruction"]
  d = ni.maneuverDistance
  m = (ni.maneuverType or "") + " " + (ni.maneuverModifier or "")
  if 1 <= d <= 40 and "turn" in m:
    if "left" in m:
      return f"Turn left in {round(d)}m"
    if "right" in m:
      return f"Turn right in {round(d)}m"
  return NAV_DEFAULT


# ------------------------------------------------------------------ main
def main():
  config_realtime_process([5], 1)
  params = Params()
  # exact f-theta ("fov") won on comma four + os04c10 + Model 3 footage (ADE@3 s 0.046 m better than fov_pinhole,
  # 95% CI 0.030 to 0.063); fov_pinhole had won on a comma 3 Corolla route. NOTES.md.
  mapping = os.environ.get("ALPAMAYO_MAPPING", "fov")
  pm = messaging.PubMaster(["alpamayoModelV2", "alpamayoOutput"])
  sm = messaging.SubMaster(["carState", "deviceMotion", "extrinsicsCalibration", "deviceState", "narrowRoadCameraState",
                            "lateralDelay", "navInstruction"], poll="carState")
  pose_cal = PoseCalibrator()
  ego = EgoTrack()

  clients = {"narrow": VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD, False),
             "wide": VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False)}
  for c in clients.values():
    while not c.connect(False):
      time.sleep(0.1)
  rings = {k: FrameRing() for k in clients}
  loader = Loader(params)
  maps = None
  last_status_t = 0.0
  cloudlog.warning(f"alpamayod: streams up, mapping {mapping}")

  while True:
    sm.update(0)
    if sm.updated["extrinsicsCalibration"]:
      pose_cal.feed_extrinsics_calibration(sm["extrinsicsCalibration"])
    if sm.updated["deviceMotion"]:
      cp = pose_cal.build_calibrated_pose(Pose.from_device_motion(sm["deviceMotion"]))
      ego.yaw_rate_flu = -float(cp.angular_velocity.yaw)
    if sm.updated["carState"]:
      ego.update(sm.logMonoTime["carState"], max(sm["carState"].vEgo, 0.0))

    for k, c in clients.items():
      while (buf := c.recv(timeout_ms=0)) is not None:
        rings[k].push(c.timestamp_sof, c.frame_id, buf)

    if maps is None and sm.seen["deviceState"] and sm.seen["narrowRoadCameraState"] and rings["wide"].frames:
      dc = DEVICE_CAMERAS[(str(sm["deviceState"].deviceType), str(sm["narrowRoadCameraState"].sensor))]
      sim = str(sm["deviceState"].deviceType) == "pc"
      maps = ca.SlotMaps(ca.CommaCams(dc.wide_road.intrinsics, dc.narrow_road.intrinsics,
                                      (dc.narrow_road.width, dc.narrow_road.height)), mapping,
                         wide_model="pinhole" if sim else "equidistant")
    if loader.runner is None:
      if time.monotonic() - last_status_t > 1.0:  # logged in qlogs and shown in the alert
        last_status_t = time.monotonic()
        o = messaging.new_message("alpamayoOutput")
        o.valid = False
        o.alpamayoOutput.status = loader.status
        o.alpamayoOutput.selfTest = json.dumps(loader.report)
        pm.send("alpamayoOutput", o)
      time.sleep(0.01)
      continue
    runner = loader.runner
    if maps is None or not sm.seen["extrinsicsCalibration"]:
      time.sleep(0.01)
      continue

    maps.update(np.array(sm["extrinsicsCalibration"].rpyCalib))
    latest = rings["wide"].frames[-1] if rings["wide"].frames else None
    if latest is None:
      continue
    t0 = latest[0]
    hist = ego.history(t0)
    inp = build_inputs(maps, rings, t0)
    if hist is None or inp is None:
      time.sleep(0.01)
      continue
    nav = nav_from_instruction(sm)
    st = time.monotonic()
    try:
      out = runner.run(*inp, hist[0], hist[1], nav, mapping)
      valid = bool(np.all(np.isfinite(out["xyz"])))
    except Exception:
      cloudlog.exception("alpamayod: inference failed")
      out, valid = None, False
    exec_time = time.monotonic() - st
    loader.note_inference(exec_time, valid)

    lat_delay = sm["lateralDelay"].lateralDelay if sm.seen["lateralDelay"] else 0.2
    msg = messaging.new_message("alpamayoModelV2")
    msg.valid = valid
    if out is not None and valid:
      fill_model_v2(msg, out, latest[1], latest[0], exec_time, lat_delay)
    pm.send("alpamayoModelV2", msg)

    o = messaging.new_message("alpamayoOutput")
    o.valid = valid
    ao = o.alpamayoOutput
    ao.frameId, ao.timestampEof, ao.inferenceTimeS = latest[1], latest[0], exec_time
    ao.navText, ao.mapping = nav, mapping
    ao.status = loader.status
    if len(loader.report.get("infer_s", [])) <= 10:
      ao.selfTest = json.dumps(loader.report)
    if out is not None:
      ao.cot = out["cot"][:512]
      ao.backend = out["backend"]
      xyz = np.asarray(out["xyz"])
      ao.x, ao.y = [float(v) for v in xyz[:, 0]], [float(v) for v in xyz[:, 1]]
      ao.kappa = [float(v) for v in np.asarray(out.get("kappa", []))]
      ao.accel = [float(v) for v in np.asarray(out.get("accel", []))]
    pm.send("alpamayoOutput", o)


if __name__ == "__main__":
  main()
