"""The stop sweep's side views (STOP_SIGNS.md sections 1.1 and 1.3; stopsigns/visibility/R_SEEN.md section 5).

At a stop line cross traffic sits 60-88 deg off the car's axis, where the full wide pass (the whole fisheye letterboxed into
640x384) finds few cars: it never reaches 80% per frame past 40 deg. Two virtual rectilinear cameras fix most of that: a
level pinhole at yaw 75 deg left and right of the calibrated forward axis, f twice the wide camera's (904 px on a comma
four), 768x512 (46 x 31.6 deg), the horizon at row 205 (40% from the top), covering 52 deg to the frame edge. These are
the C2x views R_SEEN measured (0.77-1.00 of clear frames past 70 deg for cars 30-80 px tall in their own pixels).

The view is a gather inside the jitted detector graph (graph.make_prepare_side) through maps computed here from the
calibration (rpyCalib and wideFromDeviceEuler) and the wide lens (ranging.wide_lens: visibility.json's S1 fit, else the
repo's), so one pkl serves any calibration: per view pixel, the base index of its luma's 2x2 bilinear taps, their two
weights and whether it lies inside the fisheye frame (a pixel past the frame edge is the letterbox's gray), and the index of
its chroma pair, nearest on the half-resolution grid. modeld recomputes them off its step when the calibration moves more
than RECALIB_DEG, and the side views are invalid until then.

A box found in a view goes back to the wide camera's fisheye pixels (SideGeometry.box_to_fisheye: its edges through the
lens, the bounding box), with camera wide, so the tracker's measurement and same-camera IoU need nothing new.

Sector quality (SectorQuality): per 5 deg of bearing on each view, from the host copy of the wide frame, each sector's
horizon band (elevation -1 to +1 deg, envelope.md's H band) mapped to fisheye pixels through the views' geometry, against
the full wide frame's 10-35 deg band as envelope_thresholds.json defined it: ok, degraded, washed, dark, frozen or blocked.
The contrast thresholds were derived on moving frames (envelope.md: stopped-frame ratios swung 0.04-2.9 with the scene and
were unused), and at a stop line the car is standing, so their validity there waits on step S3's logs. This reads the
same-frame gradient (no temporal mean exists at a standstill).
"""
import json
from collections import deque
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import numpy as np

from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import view_frame_from_device_frame
from openpilot.selfdrive.turnpilot.scene.ranging import VISIBILITY_JSON, Lens, calib_from_view
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

SIDE_VIEWS = ('sideLeft', 'sideRight')
SIDE_YAW_DEG = {'sideLeft': -75., 'sideRight': 75.}  # about the calibrated vertical, positive right (device y)
SIDE_SIZE = (768, 512)                 # (w, h), multiples of 32
SIDE_F_SCALE = 2.0                     # the views' focal length: twice the wide lens's (904 px at 1344x760)
SIDE_HORIZON_ROW = 0.4 * SIDE_SIZE[1]  # 204.8: the calibrated horizon 40% from the top
RECALIB_DEG = 0.3                      # the maps are recomputed when the calibration moves more than this
MAX_OFF_AXIS_DEG = 100.                # a view ray further off the wide camera's axis is never gathered (behind its lens)
BOX_EDGE_SAMPLES = 9                   # points per box edge taken back through the lens

# sector quality: thresholds from stopsigns/visibility/envelope_thresholds.json (ratio_usable, washed_lift, washed_ratio,
# washed_sat, static_std, static_mean, static_frac) and STOP_SIGNS.md section 9 (dark, frozen); luma in full range DN, as
# envelope.md measured it (ffmpeg's gray from the camera's limited-range luma)
SECTOR_DEG = 5.
BAND_DEG = 1.                 # the horizon band: elevation -BAND_DEG to +BAND_DEG
REF_DEG = (10., 35.)          # the contrast reference: the full wide frame's horizon band at these bearings, both sides
MIN_SECTOR_PX = 20            # a sector with fewer band pixels inside the frame isn't listed
RATIO_USABLE = 0.5            # contrast ratio under this: degraded
WASHED_LIFT_DN = 50.          # 5th percentile this far over the reference's...
WASHED_RATIO = 0.35           # ...with contrast under this: washed
WASHED_SAT = 0.3              # or this share of pixels saturated (SAT_DN and up): washed
SAT_DN = 250.
DARK_LUMA_DN = 45.            # median under this: dark, until step S1 sets visibility.json's darkThreshold
FROZEN_MAD_DN, FROZEN_S = 0.5, 1.0  # mean absolute change from the sector's last sample under this for this long: frozen
STATIC_STD_DN, STATIC_MEAN_DN, STATIC_FRAC = 4.0, 45., 0.5  # this share of pixels static (temporal std) and dark: blocked
HISTORY_S = 1.0               # the samples blocked's temporal std is taken over
GAP_S = 1.0                   # a sector's samples further apart than this start its history over (a new sweep)
MIN_STATIC_SAMPLES = 3


def side_f(lens: Lens) -> float:
  return SIDE_F_SCALE * float(lens.f)


def rot_z(deg: float) -> np.ndarray:
  a = np.radians(deg)
  return np.array([[np.cos(a), -np.sin(a), 0.], [np.sin(a), np.cos(a), 0.], [0., 0., 1.]])


def calib_moved_deg(rpy_a, euler_a, rpy_b, euler_b) -> float:
  """The largest change in any of the calibration's six angles, deg."""
  return float(np.degrees(np.max(np.abs(np.r_[rpy_a, euler_a] - np.r_[rpy_b, euler_b]))))


def dark_threshold(path: Path = VISIBILITY_JSON) -> float:
  """visibility.json's darkThreshold (median luma, DN) when step S1 has set it, else DARK_LUMA_DN."""
  try:
    if path.is_file():
      value = json.loads(path.read_text()).get('darkThreshold')
      if value is not None and float(value) > 0.:
        return float(value)
  except Exception:
    cloudlog.exception(f"turnpilot: {path.name} darkThreshold unreadable, taking {DARK_LUMA_DN}")
  return DARK_LUMA_DN


@dataclass(frozen=True)
class Sector:
  inner: float       # deg left of the forward axis of its edge nearer the axis (a right one's negative)
  outer: float
  idx: np.ndarray    # flat luma indices in the NV12 frame of its horizon band's fisheye pixels (none on the frame's border)


@dataclass
class SideMap:
  """One view's gather maps (graph.make_prepare_side's inputs) and its sectors, for one calibration."""
  luma_idx: np.ndarray    # int32 (w*h,): the top-left luma tap of each view pixel (0 where masked)
  chroma_idx: np.ndarray  # int32 (w*h,): its chroma pair, nearest on the half-resolution grid
  weights: np.ndarray     # uint8 (3, w*h): the bilinear x and y weights (of 255) and inside the frame (1) or not (0)
  sectors: list[Sector] = field(default_factory=list)


@dataclass
class SideGeometry:
  lens: Lens
  rpy: tuple
  wide_euler: tuple
  cam_w: int
  cam_h: int
  stride: int
  y_height: int
  calib_from_view: np.ndarray        # the wide camera's view frame to the calibrated frame
  maps: dict[str, SideMap] = field(default_factory=dict)
  ref_idx: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
  cover_deg: dict[str, tuple[float, float]] = field(default_factory=dict)  # each view's (inner, outer) |bearing| on the horizon

  @property
  def f(self) -> float:
    return side_f(self.lens)

  def wide_from_view(self, name: str) -> np.ndarray:
    """Rotation from the side view's camera frame (x right, y down, z forward) to the wide camera's view frame."""
    return self.calib_from_view.T @ rot_z(SIDE_YAW_DEG[name]) @ view_frame_from_device_frame.T

  def to_fisheye(self, name: str, u, v) -> tuple[np.ndarray, np.ndarray]:
    """Side view pixels (continuous: pixel i spans [i, i+1]) to the wide frame's fisheye pixels."""
    d = view_rays(np.asarray(u, float), np.asarray(v, float), self.f)
    return self.lens.project_many(d @ self.wide_from_view(name).T)

  def to_view(self, name: str, u, v) -> tuple[np.ndarray, np.ndarray]:
    """The wide frame's fisheye pixels to side view pixels."""
    d = self.lens.rays(u, v) @ self.wide_from_view(name)  # wide_from_view.T @ d, row-wise
    cx, cy = SIDE_SIZE[0] / 2, SIDE_HORIZON_ROW
    with np.errstate(divide='ignore', invalid='ignore'):
      return cx + self.f * d[..., 0] / d[..., 2], cy + self.f * d[..., 1] / d[..., 2]

  def box_to_fisheye(self, name: str, box) -> tuple[float, float, float, float]:
    """A box (x0, y0, x1, y1) in side view pixels to the bounding box of its edges in fisheye pixels, clipped to the frame."""
    x0, y0, x1, y1 = (float(b) for b in box)
    t = np.linspace(0., 1., BOX_EDGE_SAMPLES)
    xs = np.r_[x0 + (x1 - x0) * t, x0 + (x1 - x0) * t, np.full_like(t, x0), np.full_like(t, x1)]
    ys = np.r_[np.full_like(t, y0), np.full_like(t, y1), y0 + (y1 - y0) * t, y0 + (y1 - y0) * t]
    u, v = self.to_fisheye(name, xs, ys)
    u, v = np.clip(u, 0, self.cam_w), np.clip(v, 0, self.cam_h)
    return float(u.min()), float(v.min()), float(u.max()), float(v.max())


def view_rays(u: np.ndarray, v: np.ndarray, f: float) -> np.ndarray:
  """Side view pixels to rays in its camera frame (x right, y down, z forward)."""
  return np.stack([(u - SIDE_SIZE[0] / 2) / f, (v - SIDE_HORIZON_ROW) / f, np.ones(np.broadcast(u, v).shape)], axis=-1)


def compute_geometry(lens: Lens, rpy, wide_euler, cam_w: int, cam_h: int) -> SideGeometry:
  """Both views' maps and sectors, and the contrast reference, for this lens and calibration (about 0.1-0.3 s: modeld
  runs it off its step)."""
  return _compute_geometry(lens, tuple(float(a) for a in rpy), tuple(float(a) for a in wide_euler), int(cam_w), int(cam_h))


@lru_cache(maxsize=2)
def _compute_geometry(lens: Lens, rpy: tuple, wide_euler: tuple, cam_w: int, cam_h: int) -> SideGeometry:
  stride, y_height, _, _ = get_nv12_info(cam_w, cam_h)
  geom = SideGeometry(lens, rpy, wide_euler, cam_w, cam_h, stride, y_height, calib_from_view(list(rpy), list(wide_euler)))
  for name in SIDE_VIEWS:
    geom.maps[name], geom.cover_deg[name] = _view_map(geom, name)
  geom.ref_idx = _band_idx(geom, np.r_[np.arange(*REF_DEG, 0.05), -np.arange(*REF_DEG, 0.05)])
  return geom


def _view_map(geom: SideGeometry, name: str) -> tuple[SideMap, tuple[float, float]]:
  w, h = SIDE_SIZE
  cam_w, cam_h, stride = geom.cam_w, geom.cam_h, geom.stride
  jj, ii = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
  d = view_rays(jj, ii, geom.f) @ geom.wide_from_view(name).T
  u, v = geom.lens.project_many(d)
  xs, ys = u - 0.5, v - 0.5  # sample positions in pixel index coordinates (pixel i's centre at i)
  off_axis = np.degrees(np.arccos(np.clip(d[..., 2] / np.linalg.norm(d, axis=-1), -1., 1.)))
  valid = (xs >= 0) & (xs <= cam_w - 1) & (ys >= 0) & (ys <= cam_h - 1) & (off_axis < MAX_OFF_AXIS_DEG)
  x0 = np.clip(np.floor(xs), 0, cam_w - 2)
  y0 = np.clip(np.floor(ys), 0, cam_h - 2)
  fx, fy = np.clip(xs - x0, 0., 1.), np.clip(ys - y0, 0., 1.)
  luma_idx = np.where(valid, y0 * stride + x0, 0).astype(np.int32).reshape(-1)
  cxh = np.clip(np.floor(u / 2), 0, cam_w // 2 - 1)
  cyh = np.clip(np.floor(v / 2), 0, cam_h // 2 - 1)
  chroma_idx = (stride * geom.y_height + np.where(valid, cyh * stride + 2 * cxh, 0)).astype(np.int32).reshape(-1)
  weights = np.stack([np.where(valid, np.rint(fx * 255), 0), np.where(valid, np.rint(fy * 255), 0), valid]).astype(np.uint8)
  m = SideMap(luma_idx, chroma_idx, weights.reshape(3, -1))

  # bearing (left positive) and elevation of each view pixel, for the sectors and the coverage
  xv, yv = (jj - w / 2) / geom.f, (ii - SIDE_HORIZON_ROW) / geom.f
  bearing = -(SIDE_YAW_DEG[name] + np.degrees(np.arctan(xv)))
  elev = -np.degrees(np.arctan(yv / np.sqrt(1. + xv * xv)))
  row = int(np.floor(SIDE_HORIZON_ROW))
  on_horizon = np.abs(bearing[row][valid[row]])
  cover = (float(on_horizon.min()), float(on_horizon.max())) if len(on_horizon) else (np.nan, np.nan)
  band = valid & (np.abs(elev) <= BAND_DEG)
  # each band pixel's nearest fisheye pixel, away from the frame's border (the gradient takes its neighbours)
  bx, by = np.rint(xs).astype(np.int64), np.rint(ys).astype(np.int64)
  band &= (bx >= 1) & (bx <= cam_w - 2) & (by >= 1) & (by <= cam_h - 2)
  sign = 1. if SIDE_YAW_DEG[name] < 0 else -1.
  mag = np.abs(bearing)
  if np.isfinite(cover[0]):
    lo = np.floor(cover[0] / SECTOR_DEG) * SECTOR_DEG
    while lo < 90.:
      sel = band & (mag >= lo) & (mag < lo + SECTOR_DEG)
      idx = np.unique(by[sel] * stride + bx[sel])
      if len(idx) >= MIN_SECTOR_PX:
        m.sectors.append(Sector(float(sign * lo), float(sign * (lo + SECTOR_DEG)), idx))
      lo += SECTOR_DEG
  return m, cover


def _band_idx(geom: SideGeometry, bearings_left: np.ndarray) -> np.ndarray:
  """Flat luma indices of the fisheye pixels on the horizon band at these bearings (deg, left positive)."""
  b = np.radians(bearings_left)[:, None]
  e = np.radians(np.arange(-BAND_DEG, BAND_DEG + 1e-9, 0.05))[None, :]
  rays = np.stack([np.cos(b) * np.cos(e), -np.sin(b) * np.cos(e), -np.sin(e) * np.ones_like(b)], axis=-1)
  u, v = geom.lens.project_many(rays @ geom.calib_from_view)  # calib_from_view.T @ ray, row-wise
  x, y = np.rint(u - 0.5).astype(np.int64), np.rint(v - 0.5).astype(np.int64)
  ok = (x >= 1) & (x <= geom.cam_w - 2) & (y >= 1) & (y <= geom.cam_h - 2)
  return np.unique(y[ok] * geom.stride + x[ok])


def full_range(y: np.ndarray) -> np.ndarray:
  """The camera's limited-range luma (16-235) as full range DN, the scale the thresholds were measured on."""
  return np.clip((y.astype(np.float32) - 16.) * (255. / 219.), 0., 255.)


def gradient(frame: np.ndarray, idx: np.ndarray, stride: int) -> np.ndarray:
  """Central-difference gradient magnitude of the luma at these flat indices, full range DN per pixel."""
  f = frame
  gx = f[idx + 1].astype(np.float32) - f[idx - 1]
  gy = f[idx + stride].astype(np.float32) - f[idx - stride]
  return np.hypot(gx, gy) * (0.5 * 255. / 219.)


class SectorQuality:
  """Each side-view sector's quality, from the wide frame's host copy on that view's runs. Frozen and blocked look back over
  the sector's own recent samples (the view's runs, 5 Hz while the sweep runs), so a gap in them starts those over."""
  def __init__(self, dark_dn: float | None = None):
    self.dark_dn = dark_threshold() if dark_dn is None else dark_dn
    self.history: dict[tuple, deque] = {}    # (view, inner) -> (t, luma) samples within HISTORY_S
    self.changed_t: dict[tuple, float] = {}  # (view, inner) -> when it last changed by FROZEN_MAD_DN or more

  def update(self, geom: SideGeometry, name: str, frame: np.ndarray, t: float) -> list[tuple[float, float, str]]:
    """frame: the wide camera's NV12 host copy (flat uint8); t: its camera time, s. Returns (inner, outer, quality) per
    sector of the view."""
    stride = geom.stride
    ref = full_range(frame[geom.ref_idx])
    ref_g = float(gradient(frame, geom.ref_idx, stride).mean()) if len(ref) else 0.
    ref_p5 = float(np.percentile(ref, 5)) if len(ref) else 0.
    out = []
    for s in geom.maps[name].sectors:
      key = (name, s.inner)
      vals = full_range(frame[s.idx])
      ratio = float(gradient(frame, s.idx, stride).mean()) / ref_g if ref_g > 1e-6 else 0.
      hist = self.history.setdefault(key, deque())
      if hist and t - hist[-1][0] > GAP_S:
        hist.clear()
      if not hist or float(np.abs(vals - hist[-1][1]).mean()) >= FROZEN_MAD_DN:
        self.changed_t[key] = t
      hist.append((t, vals))
      while hist and t - hist[0][0] > HISTORY_S + 1e-6:
        hist.popleft()
      out.append((s.inner, s.outer, self._quality(vals, ratio, ref_p5, hist, t - self.changed_t[key])))
    return out

  def _quality(self, vals: np.ndarray, ratio: float, ref_p5: float, hist: deque, still_s: float) -> str:
    if float((vals >= SAT_DN).mean()) >= WASHED_SAT:
      return 'washed'
    if len(hist) >= MIN_STATIC_SAMPLES:
      stack = np.stack([h[1] for h in hist])
      static = (stack.std(axis=0) < STATIC_STD_DN) & (stack.mean(axis=0) < STATIC_MEAN_DN)
      if float(static.mean()) >= STATIC_FRAC:
        return 'blocked'
    if still_s >= FROZEN_S:
      return 'frozen'
    if float(np.median(vals)) < self.dark_dn:
      return 'dark'
    if float(np.percentile(vals, 5)) - ref_p5 >= WASHED_LIFT_DN and ratio < WASHED_RATIO:
      return 'washed'
    if ratio < RATIO_USABLE:
      return 'degraded'
    return 'ok'
