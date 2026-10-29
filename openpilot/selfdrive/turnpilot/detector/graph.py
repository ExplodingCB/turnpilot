"""YOLO26 on a full camera frame as one tinygrad graph: modeld's NV12 buffer in, the (1, 300, 6) end-to-end output out.

The whole frame is letterboxed (no crop, no warp), so a box maps back to sensor pixels with one scale and one offset.
Luma is box-filtered 2x2 onto the chroma grid and both are sampled nearest-neighbor onto the model input: for a
1344x760 frame into 640x384 that is close to an area resize. YOLO26 takes RGB in 0-1.

The far crop (DESIGN phase 4 G5) is a second graph: a FAR_CROP_SIZE window of the road frame at native resolution, which
is the model input as it is (twice the full pass's pixel density on a comma four: a car 1.8 m wide is 9 px at 230 m, as
it is at 115 m in the full pass). A box maps back with the window's offset alone (Crop). The window moves without a
recompile: the graph takes two views of modeld's frame buffer, the crop's luma rows and its chroma rows (crop_views),
so where it looks is which views it is given, and inside the graph every index is a constant (reshapes and slices, no
gather). The luma is at native resolution, the chroma nearest on its half-resolution grid.

The side views (STOP_SIGNS.md section 1.3, sideview.py) are a third graph: a 768x512 level pinhole of the wide frame, the
model input as it is, gathered through maps that are graph inputs (luma_idx, chroma_idx, weights: the luma's bilinear
base index and weights and whether the pixel is inside the frame, the chroma's nearest index), so one compiled graph serves
both views and any calibration. A box maps back through sideview.SideGeometry.box_to_fisheye.
"""
from dataclasses import dataclass
from pathlib import Path

from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

MODELS_DIR = Path(__file__).resolve().parents[1] / 'models'
DEFAULT_MODEL = 'yolo26n'
# model input (w, h) per camera frame (w, h); multiples of 32, the long side 640
INPUT_SIZES = {(1344, 760): (640, 384), (1928, 1208): (640, 416)}
PAD_VALUE = 114  # Ultralytics' letterbox gray
FAR_CROP_SIZE = (640, 384)  # the far crop (w, h): native resolution, the YOLO input as it is
FAR_HORIZON_ROW = 256       # the calibrated horizon this many rows down the crop (2/3): the ground from about 11 m out
                            # below it, signal heads from about 21 m out above it (4.8 m over the camera)
FAR_ALIGN_X = 32            # the crop's x0 a multiple of this, so its views' offsets stay aligned (strides are of 128)
SIDE_INPUT_SIZE = (768, 512)  # the side views (sideview.SIDE_SIZE): the YOLO input as it is


@dataclass(frozen=True)
class Letterbox:
  cam_w: int
  cam_h: int
  in_w: int
  in_h: int

  @property
  def scale(self) -> float:
    return min(self.in_w / self.cam_w, self.in_h / self.cam_h)

  @property
  def content(self) -> tuple[int, int]:
    """(w, h) of the scaled frame inside the model input."""
    return round(self.cam_w * self.scale), round(self.cam_h * self.scale)

  @property
  def pad(self) -> tuple[int, int]:
    """(left, top) padding, rounded as Ultralytics' LetterBox rounds it."""
    w, h = self.content
    return round((self.in_w - w) / 2 - 0.1), round((self.in_h - h) / 2 - 0.1)

  def to_frame(self, x, y):
    """Model input pixels to full-frame pixels (continuous coordinates, pixel i spans [i, i+1])."""
    px, py = self.pad
    return (x - px) / self.scale, (y - py) / self.scale

  def to_input(self, x, y):
    px, py = self.pad
    return x * self.scale + px, y * self.scale + py


@dataclass(frozen=True)
class Crop:
  """The far crop: model input pixel (x, y) is frame pixel (x0 + x, y0 + y). x0 a multiple of FAR_ALIGN_X, y0 even."""
  cam_w: int
  cam_h: int
  x0: int
  y0: int
  in_w: int = FAR_CROP_SIZE[0]
  in_h: int = FAR_CROP_SIZE[1]

  @property
  def scale(self) -> float:
    return 1.0

  @property
  def rect(self) -> tuple[int, int, int, int]:
    """x0, y0, x1, y1 in frame pixels."""
    return self.x0, self.y0, self.x0 + self.in_w, self.y0 + self.in_h

  def to_frame(self, x, y):
    return x + self.x0, y + self.y0

  def to_input(self, x, y):
    return x - self.x0, y - self.y0


def crop_at(cam_w: int, cam_h: int, u: float, v_horizon: float, size: tuple[int, int] = FAR_CROP_SIZE) -> Crop:
  """The far crop centred on frame column u with the horizon (frame row v_horizon) at FAR_HORIZON_ROW, inside the frame."""
  w, h = size
  x0 = int(round((u - w / 2) / FAR_ALIGN_X)) * FAR_ALIGN_X
  x0 = min(max(x0, 0), (cam_w - w) // FAR_ALIGN_X * FAR_ALIGN_X)
  y0 = 2 * int(round((v_horizon - FAR_HORIZON_ROW) / 2))
  y0 = min(max(y0, 0), (cam_h - h) // 2 * 2)
  return Crop(cam_w, cam_h, x0, y0, w, h)


def crop_views(crop: Crop) -> tuple[tuple[int, int], tuple[int, int]]:
  """(offset, size) in bytes, within one camerad-layout NV12 frame, of the crop's luma rows and of its chroma rows: what
  make_prepare_crop's graph takes. The rows are whole strides; the graph slices the crop's columns out of them."""
  stride, y_height, _, _ = get_nv12_info(crop.cam_w, crop.cam_h)
  return (crop.y0 * stride + crop.x0, crop.in_h * stride), (stride * y_height + crop.y0 // 2 * stride + crop.x0, crop.in_h // 2 * stride)


def make_prepare_crop(cam_w: int, cam_h: int, size: tuple[int, int] = FAR_CROP_SIZE):
  """(luma, chroma) uint8 views (crop_views) -> (1, 3, h, w) RGB 0-1 at native resolution: luma as it is, chroma nearest."""
  from tinygrad import Tensor, dtypes
  stride, _, _, _ = get_nv12_info(cam_w, cam_h)
  w, h = size

  def prepare(luma: Tensor, chroma: Tensor) -> Tensor:
    y = luma.reshape(h, stride)[:, :w].cast(dtypes.float32)
    uv = chroma.reshape(h // 2, stride)[:, :w].reshape(h // 2, w // 2, 2).cast(dtypes.float32) - 128

    def up(p: Tensor) -> Tensor:  # the half-resolution grid to every pixel
      return p.reshape(h // 2, 1, w // 2, 1).expand(h // 2, 2, w // 2, 2).reshape(h, w)
    u, v = up(uv[..., 0]), up(uv[..., 1])
    # BT.601 limited range, the inverse of camerad's RGB to NV12 (as make_prepare)
    c = (y - 16) * 1.164
    return Tensor.stack(c + 1.596 * v, c - 0.392 * u - 0.813 * v, c + 2.017 * u).div(255).clip(0, 1).unsqueeze(0)
  return prepare


def make_prepare_side(cam_w: int, cam_h: int, size: tuple[int, int] = SIDE_INPUT_SIZE):
  """(input_frame, luma_idx, chroma_idx, weights) -> (1, 3, h, w) RGB 0-1: the frame (modeld's NV12 view of the wide camera)
  gathered through a side view's maps (sideview.SideMap): luma bilinear from its base index (the taps +1, +stride,
  +stride+1) with weights x, y of 255, chroma nearest, a pixel whose weights[2] is 0 the letterbox's gray."""
  from tinygrad import Tensor, dtypes
  stride, _, _, _ = get_nv12_info(cam_w, cam_h)
  w, h = size

  def prepare(input_frame: Tensor, luma_idx: Tensor, chroma_idx: Tensor, weights: Tensor) -> Tensor:
    frame = input_frame.cast(dtypes.float32)
    wt = weights.cast(dtypes.float32)
    fx, fy = wt[0] / 255, wt[1] / 255
    top = frame[luma_idx] * (1 - fx) + frame[luma_idx + 1] * fx
    bottom = frame[luma_idx + stride] * (1 - fx) + frame[luma_idx + stride + 1] * fx
    luma = top * (1 - fy) + bottom * fy
    u, v = frame[chroma_idx] - 128, frame[chroma_idx + 1] - 128
    # BT.601 limited range, the inverse of camerad's RGB to NV12 (as make_prepare)
    c = (luma - 16) * 1.164
    rgb = Tensor.stack(c + 1.596 * v, c - 0.392 * u - 0.813 * v, c + 2.017 * u).div(255).clip(0, 1)
    return (wt[2] > 0.5).where(rgb, PAD_VALUE / 255).reshape(3, h, w).unsqueeze(0)
  return prepare


def letterbox(cam_w: int, cam_h: int, in_size: tuple[int, int] | None = None) -> Letterbox:
  in_w, in_h = in_size or INPUT_SIZES[(cam_w, cam_h)]
  return Letterbox(cam_w, cam_h, in_w, in_h)


def source_index(n: int, pad: int, content: int, scale: float, n_half: int):
  """Per model-input column (or row): the half-resolution frame pixel holding its center, and whether it's inside the frame."""
  from tinygrad import Tensor, dtypes
  i = Tensor.arange(n, dtype=dtypes.float32)
  src = ((i + 0.5 - pad) / (2 * scale)).floor().clip(0, n_half - 1).cast(dtypes.int32)
  return src, (i >= pad) & (i < pad + content)


def make_prepare(lb: Letterbox):
  """NV12 buffer (uint8, flat, camerad's padded layout) -> (1, 3, in_h, in_w) RGB 0-1, letterboxed."""
  from tinygrad import Tensor, dtypes
  stride, y_height, _, _ = get_nv12_info(lb.cam_w, lb.cam_h)
  uv_offset = stride * y_height
  (cw, ch), (px, py) = lb.content, lb.pad

  def prepare(input_frame: Tensor) -> Tensor:
    sx, vx = source_index(lb.in_w, px, cw, lb.scale, lb.cam_w // 2)
    sy, vy = source_index(lb.in_h, py, ch, lb.scale, lb.cam_h // 2)
    sx, vx, sy, vy = sx.reshape(1, -1), vx.reshape(1, -1), sy.reshape(-1, 1), vy.reshape(-1, 1)
    frame = input_frame.cast(dtypes.float32)
    y_idx = 2 * sy * stride + 2 * sx
    luma = (frame[y_idx] + frame[y_idx + 1] + frame[y_idx + stride] + frame[y_idx + stride + 1]) * 0.25
    uv_idx = uv_offset + sy * stride + 2 * sx
    u, v = frame[uv_idx] - 128, frame[uv_idx + 1] - 128
    # BT.601 limited range, the inverse of camerad's RGB to NV12
    c = (luma - 16) * 1.164
    rgb = Tensor.stack(c + 1.596 * v, c - 0.392 * u - 0.813 * v, c + 2.017 * u).div(255).clip(0, 1)
    return (vx & vy).where(rgb, PAD_VALUE / 255).unsqueeze(0)
  return prepare


def onnx_path(model: str = DEFAULT_MODEL) -> Path:
  return MODELS_DIR / f'{model}.onnx'


def detector_pkl_path(cam_w: int, cam_h: int, chestnut: bool, models_dir: Path = MODELS_DIR) -> Path:
  """The compiled detector for a camera size, named like modeld's pkls: big_ for the chestnut."""
  return models_dir / f'{"big_" if chestnut else ""}detector_{cam_w}x{cam_h}_tinygrad.pkl'


def far_detector_pkl_path(cam_w: int, cam_h: int, chestnut: bool, models_dir: Path = MODELS_DIR) -> Path:
  """The compiled far crop for a camera size (detector_pkl_path's naming)."""
  return models_dir / f'{"big_" if chestnut else ""}far_detector_{cam_w}x{cam_h}_tinygrad.pkl'


def side_detector_pkl_path(cam_w: int, cam_h: int, chestnut: bool, models_dir: Path = MODELS_DIR) -> Path:
  """The compiled side views for a camera size (detector_pkl_path's naming)."""
  return models_dir / f'{"big_" if chestnut else ""}side_detector_{cam_w}x{cam_h}_tinygrad.pkl'


def make_side_detector(cam_w: int, cam_h: int, model: str = DEFAULT_MODEL, size: tuple[int, int] = SIDE_INPUT_SIZE):
  """detect(input_frame, luma_idx, chroma_idx, weights) -> (1, 300, 6) Tensor in side view input pixels, conf, COCO class."""
  from tinygrad import Context
  from tinygrad.nn.onnx import OnnxRunner
  prepare = make_prepare_side(cam_w, cam_h, size)
  runner = OnnxRunner(onnx_path(model))
  input_name = next(iter(runner.graph_inputs))

  def detect(input_frame, luma_idx, chroma_idx, weights):
    with Context(SPLIT_REDUCEOP=0):  # the gathers lower to plain indexed loads
      img = prepare(input_frame, luma_idx, chroma_idx, weights).realize()
    return next(iter(runner({input_name: img}).values()))
  return detect


def make_far_detector(cam_w: int, cam_h: int, model: str = DEFAULT_MODEL, size: tuple[int, int] = FAR_CROP_SIZE):
  """detect(luma, chroma) -> (1, 300, 6) Tensor in crop input pixels (Crop.to_frame maps them back), conf, COCO class."""
  from tinygrad import Context
  from tinygrad.nn.onnx import OnnxRunner
  prepare = make_prepare_crop(cam_w, cam_h, size)
  runner = OnnxRunner(onnx_path(model))
  input_name = next(iter(runner.graph_inputs))

  def detect(luma, chroma):
    with Context(SPLIT_REDUCEOP=0):
      img = prepare(luma, chroma).realize()
    return next(iter(runner({input_name: img}).values()))
  return detect


def make_detector(cam_w: int, cam_h: int, model: str = DEFAULT_MODEL, in_size: tuple[int, int] | None = None):
  """(letterbox, detect): detect(input_frame) -> (1, 300, 6) Tensor, x0 y0 x1 y1 in model input pixels, conf, COCO class."""
  from tinygrad import Context
  from tinygrad.nn.onnx import OnnxRunner
  lb = letterbox(cam_w, cam_h, in_size)
  prepare = make_prepare(lb)
  runner = OnnxRunner(onnx_path(model))
  input_name = next(iter(runner.graph_inputs))

  def detect(input_frame):
    # as modeld's warp: no split reduce, so the gathers lower to plain indexed loads
    with Context(SPLIT_REDUCEOP=0):
      img = prepare(input_frame).realize()
    return next(iter(runner({input_name: img}).values()))
  return lb, detect
