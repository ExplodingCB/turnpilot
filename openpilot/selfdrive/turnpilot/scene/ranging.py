"""Monocular ranging: project a box's bottom-center onto a flat ground plane.

Frames: openpilot's device and calib frames are x forward, y right, z down. Positions out of here are x forward and
y LEFT, in meters over the ground from the point under the road camera, in the calibrated frame.

The wide road camera on a real device is a fisheye. Matching road-camera features (pinhole, trusted focal length) to the
wide camera on the public demo routes fits an equidistant lens, r = f * theta, far better than a pinhole: rms 1.4 px vs
3.3 px over 0-33 deg on a comma four (os04c10, f 452), 1.1 px vs 1.4 px over 0-22 deg on the 1928x1208 camera (f 633).
Only the one point per box is undistorted. The sim renders a pinhole, so the lens follows the device type.

One lens serves every consumer of the wide camera (STOP_SIGNS.md section 1.1): the S1 fit in maps/visibility.json when the
file has one ({model: equidistant | equisolid, f, k}: r = f * theta, or r = f * sin(k * theta) / k), else the repo's
equidistant fit above, which is validated only to 33 deg. Each consumer says which lens it uses (Lens.spec: the tracker's
lens_spec, the side views' turnpilotDetections.lens) so the stop-sign decider can refuse a mismatch.
"""
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from openpilot.common.swaglog import cloudlog

from openpilot.common.transformations.camera import CameraConfig, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler

DEFAULT_HEIGHT = 1.22   # m, calibrationd's initial camera height
MIN_HEIGHT = 0.5
MAX_RANGE = 200.        # m; farther ground points are noise
PINHOLE_DEVICE_TYPES = ('pc',)  # the sim's wide camera is a pinhole
# equidistant focal length of the wide road camera by its frame size, fitted on the demo routes (see above)
WIDE_FISHEYE_F = {(1344, 760): 452.0, (1928, 1208): 633.0}
VISIBILITY_JSON = Path(__file__).resolve().parents[1] / 'maps' / 'visibility.json'  # the S1 fit (SG2's schema)
LENS_MODELS = ('equidistant', 'equisolid')


@dataclass(frozen=True)
class Lens:
  width: int
  height: int
  f: float
  fisheye: bool = False  # equidistant: radius = f * angle off the optical axis
  k: float = 0.0         # fisheye only: 0 equidistant, else radius = f * sin(k * angle) / k (verify.md: k 0.3, bracket 0-0.4)
  source: str = 'repo'   # 'repo': the fit above; 'visibility.json': step S1's

  @property
  def center(self) -> tuple[float, float]:
    return self.width / 2, self.height / 2

  @property
  def spec(self) -> str:
    """What the lens is, for comparing consumers: 'equidistant f=452.0 k=0.0 repo'."""
    model = 'pinhole' if not self.fisheye else 'equidistant' if self.k == 0. else 'equisolid'
    return f"{model} f={float(self.f)} k={float(self.k)} {self.source}"

  def _theta(self, rn):
    return rn if self.k == 0. else np.arcsin(np.clip(self.k * rn, -1., 1.)) / self.k

  def _radius(self, theta):
    return theta if self.k == 0. else np.sin(self.k * theta) / self.k

  def ray(self, u: float, v: float) -> np.ndarray:
    """Pixel -> ray in the camera's view frame (x right, y down, z forward)."""
    cx, cy = self.center
    dx, dy = (u - cx) / self.f, (v - cy) / self.f
    if not self.fisheye:
      return np.array([dx, dy, 1.0])
    rn = np.hypot(dx, dy)
    if rn < 1e-9:
      return np.array([0.0, 0.0, 1.0])
    theta = self._theta(rn)
    s = np.sin(theta) / rn
    return np.array([dx * s, dy * s, np.cos(theta)])

  def project(self, ray: np.ndarray) -> tuple[float, float]:
    """Ray in the view frame -> pixel (the inverse of ray)."""
    cx, cy = self.center
    x, y, z = ray
    if not self.fisheye:
      return cx + self.f * x / z, cy + self.f * y / z
    r = np.hypot(x, y)
    theta = np.arctan2(r, z)
    k = self.f * self._radius(theta) / r if r > 1e-12 else 0.0
    return cx + k * x, cy + k * y

  def rays(self, u, v) -> np.ndarray:
    """ray() for arrays of pixels: (..., 3)."""
    cx, cy = self.center
    dx, dy = (np.asarray(u, float) - cx) / self.f, (np.asarray(v, float) - cy) / self.f
    if not self.fisheye:
      return np.stack([dx, dy, np.ones_like(dx)], axis=-1)
    rn = np.hypot(dx, dy)
    theta = self._theta(rn)
    s = np.where(rn > 1e-9, np.sin(theta) / np.maximum(rn, 1e-9), 1.0)
    return np.stack([dx * s, dy * s, np.cos(theta)], axis=-1)

  def project_many(self, rays: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """project() for (..., 3) rays."""
    cx, cy = self.center
    x, y, z = rays[..., 0], rays[..., 1], rays[..., 2]
    if not self.fisheye:
      with np.errstate(divide='ignore', invalid='ignore'):
        return cx + self.f * x / z, cy + self.f * y / z
    r = np.hypot(x, y)
    theta = np.arctan2(r, z)
    k = np.where(r > 1e-12, self.f * self._radius(theta) / np.maximum(r, 1e-12), 0.0)
    return cx + k * x, cy + k * y


def file_lens(path: Path = VISIBILITY_JSON) -> dict | None:
  """visibility.json's lens ({model, f, k}) when the file has a fitted one, else None (no file, or S1 hasn't run). A file
  that is there but unreadable, or a lens it can't use, is logged and taken as none: SG2's schema check refuses it loudly."""
  try:
    if not path.is_file():
      return None
    lens = json.loads(path.read_text()).get('lens') or None
    if lens is None or lens.get('model') not in LENS_MODELS or not float(lens.get('f') or 0.) > 0.:
      if lens is not None:
        cloudlog.error(f"turnpilot: {path.name} lens {lens} not usable, taking the repo's")
      return None
    return lens
  except Exception:
    cloudlog.exception(f"turnpilot: {path.name} unreadable, taking the repo's lens")
    return None


def road_lens(cam: CameraConfig) -> Lens:
  return Lens(cam.width, cam.height, cam.focal_length)


def wide_lens(cam: CameraConfig, device_type: str, path: Path = VISIBILITY_JSON) -> Lens:
  """The wide camera's lens: a pinhole in the sim, else visibility.json's fit when it has one, else the repo's."""
  if device_type in PINHOLE_DEVICE_TYPES or (cam.width, cam.height) not in WIDE_FISHEYE_F:
    return Lens(cam.width, cam.height, cam.focal_length)
  fit = file_lens(path)
  if fit is not None:
    k = float(fit.get('k') or 0.) if fit['model'] == 'equisolid' else 0.
    return Lens(cam.width, cam.height, float(fit['f']), fisheye=True, k=k, source='visibility.json')
  return Lens(cam.width, cam.height, WIDE_FISHEYE_F[(cam.width, cam.height)], fisheye=True)


def calib_from_view(rpy_calib, wide_from_device_euler=None) -> np.ndarray:
  """Rotation from a camera's view frame to the calibrated frame; the wide camera adds its offset from the road camera,
  as the UI's augmented_road_view does."""
  view_from_calib = view_frame_from_device_frame @ rot_from_euler(rpy_calib)
  if wide_from_device_euler is not None:
    view_from_calib = view_frame_from_device_frame @ rot_from_euler(wide_from_device_euler) @ rot_from_euler(rpy_calib)
  return view_from_calib.T


def ground_point(lens: Lens, calib_from_view_rot: np.ndarray, height: float, u: float, v: float) -> tuple[float, float] | None:
  """(x forward, y left) in m where the ray through pixel (u, v) meets flat ground `height` below the camera, or None when
  the ray doesn't go down (at or above the horizon) or the point is beyond MAX_RANGE."""
  ray = calib_from_view_rot @ lens.ray(u, v)   # calib frame: x fwd, y right, z down
  if ray[2] <= 1e-6:
    return None
  x, y = ray[0] * height / ray[2], ray[1] * height / ray[2]
  if x <= 0 or np.hypot(x, y) > MAX_RANGE:
    return None
  return float(x), float(-y)


def camera_height(calibration) -> float:
  h = list(calibration.height) if calibration is not None else []
  return float(h[0]) if len(h) and np.isfinite(h[0]) and h[0] > MIN_HEIGHT else DEFAULT_HEIGHT
