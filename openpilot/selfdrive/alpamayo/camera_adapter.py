"""comma road cameras -> Alpamayo 2 Edge camera slots. numpy only (runs on device).

Single source of truth for the camera gap: open-loop replay, the AlpaSim
comma-like rig and alpamayod all build slot images through these maps.

Frames: comma device (x fwd, y right, z down) at the camera; openpilot calib =
device rotated by rpyCalib; view frame (x right, y down, z fwd).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

CAM_IDS = (0, 1, 2, 3, 5, 6)
FRONT, TELE = 1, 6
FRONT_OFFSETS_US = (-2_000_000, -1_000_000, -500_000, -100_000, 0)
OTHER_OFFSETS_US = (-1_000_000, -100_000, 0)
# All slots share one size (the reference validator stacks 22 images). 540x960 has
# NVIDIA's 1080x1920 aspect, so the processor's smart_resize gives the same grids.
SLOT_HW = (540, 960)
NV_FRONT_HFOV_DEG = 120.0
NV_TELE_HFOV_DEG = 30.0
MAPPINGS = ("direct", "fov", "fov_gray", "fov_drop", "wide_only", "raw_drop", "fov_pinhole")
# raw_drop reproduces commaai/openpilot@alpamayo2-edge-egpu: raw frames at native size,
# slots 1 + 6 only, the processor's smart_resize does the rest.

VIEW_FROM_DEVICE = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])


def rot_from_euler(rpy) -> np.ndarray:
  r, p, y = rpy
  cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
  rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
  ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
  rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
  return rz @ ry @ rx


# NVIDIA Hyperion front cameras the model was trained on (PhysicalAI-AV calibration,
# clip ea7bbd31-b7a5-4972-8dbd-7089e6b53de4). f-theta: pixel radius = th2r(angle).
# Camera frame x right, y down, z fwd; R_rig_cam rotates it into the rig (FLU) frame.
NV_FRONT_WIDE = dict(native_wh=(1920, 1080), pp=(963.159, 755.471),
                th2r=(0, 924.952, 43.4724, -82.6641, 26.1146),
                R_rig_cam=((-0.010559, -0.001013, 0.999944), (-0.999922, 0.006750, -0.010552), (-0.006739, -0.999977, -0.001085)))
NV_FRONT_TELE = dict(native_wh=(1920, 1080), pp=(773.456, 575.914),
                th2r=(0, 3730.84, 110.34, -401.81, 463.483),
                R_rig_cam=((0.042976, 0.005189, 0.999063), (-0.999070, 0.003854, 0.042956), (-0.003628, -0.999979, 0.005349)))


@dataclass
class TargetCam:
  """Virtual camera an Alpamayo slot expects: a pinhole, or NVIDIA's f-theta calibration."""
  hw: tuple[int, int]
  hfov_deg: float = 0.0
  model: str = "pinhole"
  calib: dict | None = None  # NV_FRONT_WIDE / NV_FRONT_TELE for model 'ftheta'

  def rays(self) -> np.ndarray:
    """Unit rays per output pixel in a view frame aligned with openpilot's calib frame."""
    h, w = self.hw
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    if self.model == "pinhole":
      f = (w / 2) / np.tan(np.radians(self.hfov_deg) / 2)
      return _unit(np.stack([(u - w / 2) / f, (v - h / 2) / f, np.ones_like(u)], -1))
    c = self.calib
    s = c["native_wh"][0] / w  # output is a uniform downscale of the native image
    du, dv = u * s - c["pp"][0], v * s - c["pp"][1]
    ang = np.linspace(0, np.radians(100), 8000)
    theta = np.interp(np.hypot(du, dv), np.polynomial.polynomial.polyval(ang, c["th2r"]), ang)
    phi = np.arctan2(dv, du)
    cam = np.stack([np.sin(theta) * np.cos(phi), np.sin(theta) * np.sin(phi), np.cos(theta)], -1)
    rig = cam @ np.asarray(c["R_rig_cam"]).T             # FLU
    frd = rig * np.array([1.0, -1.0, -1.0])              # openpilot calib frame (x fwd, y right, z down)
    return frd @ VIEW_FROM_DEVICE.T                      # view frame (x right, y down, z fwd)


def _unit(d):
  return d / np.linalg.norm(d, axis=-1, keepdims=True)


def default_targets() -> dict[int, TargetCam]:
  return {FRONT: TargetCam(SLOT_HW, model="ftheta", calib=NV_FRONT_WIDE),
          TELE: TargetCam(SLOT_HW, model="ftheta", calib=NV_FRONT_TELE)}


def pinhole_targets() -> dict[int, TargetCam]:
  return {FRONT: TargetCam(SLOT_HW, NV_FRONT_HFOV_DEG), TELE: TargetCam(SLOT_HW, NV_TELE_HFOV_DEG)}


def build_remap(src_K, src_wh, device_from_calib_rpy, target: TargetCam, src_model: str):
  """Source pixel coords (x, y) per target pixel, plus the fraction that lands on the sensor.

  src_model: 'pinhole' (comma narrow) or 'equidistant' (comma wide: the vignetted
  image circle, ~195 deg across the AR/OX sensor, fits r = f * theta with
  openpilot's wide focal length).
  """
  rays = target.rays()
  M = VIEW_FROM_DEVICE @ rot_from_euler(device_from_calib_rpy) @ VIEW_FROM_DEVICE.T
  r = rays @ M.T
  z = r[..., 2]
  if src_model == "equidistant":
    theta = np.arctan2(np.hypot(r[..., 0], r[..., 1]), z)
    phi = np.arctan2(r[..., 1], r[..., 0])
    x = src_K[0, 0] * theta * np.cos(phi) + src_K[0, 2]
    y = src_K[1, 1] * theta * np.sin(phi) + src_K[1, 2]
    behind = theta > np.radians(95)
  else:
    with np.errstate(divide="ignore", invalid="ignore"):
      x = src_K[0, 0] * r[..., 0] / z + src_K[0, 2]
      y = src_K[1, 1] * r[..., 1] / z + src_K[1, 2]
    behind = z <= 1e-3
  bad = behind | (x < 0) | (y < 0) | (x > src_wh[0] - 1) | (y > src_wh[1] - 1)
  x, y = x.astype(np.float32), y.astype(np.float32)
  x[bad], y[bad] = -1, -1
  return x, y, float(1 - bad.mean())


def direct_remap(src_wh, out_hw=SLOT_HW):
  """Plain resize of the whole sensor into a slot (mapping 'direct')."""
  h, w = out_hw
  u, v = np.meshgrid((np.arange(w) + 0.5) * src_wh[0] / w - 0.5, (np.arange(h) + 0.5) * src_wh[1] / h - 0.5)
  return u.astype(np.float32), v.astype(np.float32), 1.0


@dataclass
class CommaCams:
  wide_K: np.ndarray
  narrow_K: np.ndarray
  wh: tuple[int, int]


class SlotMaps:
  """Remap tables for slots 1 and 6 for a mapping, rebuilt when calibration changes."""

  def __init__(self, cams: CommaCams, mapping: str = "fov", targets: dict[int, TargetCam] | None = None,
               wide_model: str = "equidistant"):
    """wide_model: 'equidistant' on comma hardware, 'pinhole' in the openpilot simulator."""
    assert mapping in MAPPINGS, mapping
    self.cams, self.mapping, self.wide_model = cams, mapping, wide_model
    self.targets = targets or (pinhole_targets() if mapping == "fov_pinhole" else default_targets())
    self._key = None
    self.maps: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    self.coverage: dict[int, float] = {}

  def source(self, slot: int) -> str:
    """Which comma camera feeds a slot."""
    return "wide" if (slot == FRONT or self.mapping == "wide_only") else "narrow"

  def update(self, rpy) -> None:
    key = tuple(np.round(np.asarray(rpy, dtype=np.float64), 3))  # 1e-3 rad is under 1 px; finer keys defeat the frame caches
    if key == self._key:
      return
    self._key = key
    for slot in (FRONT, TELE):
      src = self.source(slot)
      if self.mapping == "direct":
        x, y, cov = direct_remap(self.cams.wh)
      elif self.mapping == "raw_drop":
        x, y, cov = direct_remap(self.cams.wh, (self.cams.wh[1], self.cams.wh[0]))
      else:
        K = self.cams.wide_K if src == "wide" else self.cams.narrow_K
        x, y, cov = build_remap(K, self.cams.wh, rpy, self.targets[slot], self.wide_model if src == "wide" else "pinhole")
      self.maps[slot], self.coverage[slot] = (x, y), cov

  def filler_value(self) -> int:
    return 128 if self.mapping == "fov_gray" else 0

  def slots(self) -> tuple[int, ...]:
    return (FRONT, TELE) if self.mapping.endswith("_drop") else CAM_IDS


def _bilinear_plane(plane: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
  h, w = plane.shape[:2]
  x0 = np.clip(np.floor(x).astype(np.int32), 0, w - 2)
  y0 = np.clip(np.floor(y).astype(np.int32), 0, h - 2)
  fx, fy = (x - x0)[..., None] if plane.ndim == 3 else (x - x0), (y - y0)[..., None] if plane.ndim == 3 else (y - y0)
  p = plane.astype(np.float32)
  top = p[y0, x0] * (1 - fx) + p[y0, x0 + 1] * fx
  bot = p[y0 + 1, x0] * (1 - fx) + p[y0 + 1, x0 + 1] * fx
  return top * (1 - fy) + bot * fy


def remap_rgb(rgb: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
  out = _bilinear_plane(rgb, x, y)
  out[x < 0] = 0
  return np.clip(out + 0.5, 0, 255).astype(np.uint8)


def remap_nv12(buf: np.ndarray, width: int, height: int, stride: int, uv_offset: int,
               x: np.ndarray, y: np.ndarray) -> np.ndarray:
  """Sample an NV12 VisionBuf at remap coords and convert only those pixels to RGB."""
  Y = buf[:uv_offset].reshape(-1, stride)[:height, :width]
  UV = buf[uv_offset:uv_offset + stride * (height // 2)].reshape(-1, stride)[:height // 2, :width].reshape(height // 2, width // 2, 2)
  yv = _bilinear_plane(Y, x, y)
  uv = _bilinear_plane(UV, x * 0.5 - 0.25, y * 0.5 - 0.25) - 128.0
  yv = (yv - 16.0) * 1.164
  r = yv + 1.596 * uv[..., 1]
  g = yv - 0.392 * uv[..., 0] - 0.813 * uv[..., 1]
  b = yv + 2.017 * uv[..., 0]
  out = np.stack([r, g, b], -1)
  out[x < 0] = 0
  return np.clip(out + 0.5, 0, 255).astype(np.uint8)
