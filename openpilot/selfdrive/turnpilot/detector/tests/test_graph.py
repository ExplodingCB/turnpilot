import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.turnpilot.detector.graph import PAD_VALUE, Letterbox, letterbox, make_prepare
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info


def solid_nv12(w: int, h: int, y: int, u: int, v: int) -> np.ndarray:
  """A camerad-layout NV12 buffer (as modeld copies it) of one color; the stride padding is garbage."""
  stride, y_height, uv_height, _ = get_nv12_info(w, h)
  buf = np.full((y_height + uv_height, stride), 255, dtype=np.uint8)
  buf[:h, :w] = y
  uv = buf[y_height:y_height + h // 2, :w].reshape(h // 2, w // 2, 2)
  uv[..., 0], uv[..., 1] = u, v
  return buf.reshape(-1)


class TestLetterbox(OpenpilotTestCase):
  def test_comma_cameras(self):
    # as Ultralytics' LetterBox(auto=False) pads them
    lb = letterbox(1344, 760)
    assert (lb.in_w, lb.in_h, lb.content, lb.pad) == (640, 384, (640, 362), (0, 11))
    lb = letterbox(1928, 1208)
    assert (lb.in_w, lb.in_h, lb.content, lb.pad) == (640, 416, (640, 401), (0, 7))

  def test_round_trip(self):
    rng = np.random.default_rng(0)
    for lb in (letterbox(1344, 760), letterbox(1928, 1208), Letterbox(1000, 1000, 640, 384)):
      x, y = rng.uniform(0, lb.cam_w, 100), rng.uniform(0, lb.cam_h, 100)
      np.testing.assert_allclose(lb.to_frame(*lb.to_input(x, y)), (x, y), atol=1e-9)
      # the frame's corners land on the content's corners
      (cw, ch), (px, py) = lb.content, lb.pad
      np.testing.assert_allclose(lb.to_input(0, 0), (px, py))
      np.testing.assert_allclose(lb.to_input(lb.cam_w, lb.cam_h), (px + cw, py + ch), atol=1.0)

  def test_portrait_frame_pads_sides(self):
    lb = Letterbox(760, 1344, 640, 384)
    assert lb.content[1] == 384 and lb.pad[1] == 0 and lb.pad[0] > 0


class TestPrepare(OpenpilotTestCase):
  def test_solid_red_and_padding(self):
    lb = Letterbox(256, 144, 64, 48)  # content 64x36, 6 rows of padding top and bottom
    from tinygrad import Tensor
    img = make_prepare(lb)(Tensor(solid_nv12(256, 144, 82, 90, 240))).numpy()
    assert img.shape == (1, 3, 48, 64)
    (_, ch), (_, py) = lb.content, lb.pad
    inside = img[0, :, py:py + ch].reshape(3, -1)
    np.testing.assert_allclose(inside.mean(axis=1), [1.0, 0.0, 0.0], atol=0.02)  # BT.601 limited (82, 90, 240) is red
    assert np.ptp(inside, axis=1).max() < 1e-5
    np.testing.assert_allclose(img[0, :, :py], PAD_VALUE / 255, atol=1e-6)
    np.testing.assert_allclose(img[0, :, py + ch:], PAD_VALUE / 255, atol=1e-6)

  def test_matches_numpy(self):
    # luma box-filtered 2x2, then nearest on the chroma grid, against a direct numpy version on a random frame
    w, h = 256, 144
    lb = Letterbox(w, h, 96, 64)
    stride, y_height, uv_height, _ = get_nv12_info(w, h)
    frame = np.random.default_rng(1).integers(0, 256, stride * (y_height + uv_height), dtype=np.uint8)
    from tinygrad import Tensor
    got = make_prepare(lb)(Tensor(frame)).numpy()[0]

    planes = frame.reshape(-1, stride).astype(np.float32)
    luma = planes[:h, :w].reshape(h // 2, 2, w // 2, 2).mean(axis=(1, 3))
    uv = planes[y_height:y_height + h // 2, :w].reshape(h // 2, w // 2, 2) - 128
    (cw, ch), (px, py) = lb.content, lb.pad
    expected = np.full((3, lb.in_h, lb.in_w), PAD_VALUE / 255, dtype=np.float32)
    for iy in range(py, py + ch):
      for ix in range(px, px + cw):
        sx = min(int((ix + 0.5 - px) / (2 * lb.scale)), w // 2 - 1)
        sy = min(int((iy + 0.5 - py) / (2 * lb.scale)), h // 2 - 1)
        c, (u, v) = (luma[sy, sx] - 16) * 1.164, uv[sy, sx]
        expected[:, iy, ix] = np.clip(np.array([c + 1.596 * v, c - 0.392 * u - 0.813 * v, c + 2.017 * u]) / 255, 0, 1)
    np.testing.assert_allclose(got, expected, atol=1e-4)


def crop_reference(frame: np.ndarray, w: int, h: int, crop) -> np.ndarray:
  """The far crop's model input straight from the NV12 planes: luma at native resolution (no box filter), chroma nearest."""
  stride, y_height, _, _ = get_nv12_info(w, h)
  planes = frame.reshape(-1, stride).astype(np.float32)
  rows, cols = np.arange(crop.y0, crop.y0 + crop.in_h), np.arange(crop.x0, crop.x0 + crop.in_w)
  luma = planes[rows][:, cols]
  u = planes[y_height + rows // 2][:, 2 * (cols // 2)] - 128
  v = planes[y_height + rows // 2][:, 2 * (cols // 2) + 1] - 128
  c = (luma - 16) * 1.164
  return np.clip(np.stack([c + 1.596 * v, c - 0.392 * u - 0.813 * v, c + 2.017 * u]) / 255, 0, 1)


class TestFarCrop(OpenpilotTestCase):
  def test_geometry_maps_input_pixels_one_to_one(self):
    # DESIGN phase 4 G5: a native-resolution window of the road frame, so a box maps back with an offset alone
    from openpilot.selfdrive.turnpilot.detector.graph import FAR_ALIGN_X, FAR_CROP_SIZE, Crop, crop_at
    assert FAR_CROP_SIZE == (640, 384)
    crop = Crop(1344, 760, 352, 120)
    assert crop.scale == 1.0 and (crop.in_w, crop.in_h) == FAR_CROP_SIZE and crop.rect == (352, 120, 992, 504)
    np.testing.assert_allclose(crop.to_frame(np.array([0., 639.5]), np.array([0., 10.])), ([352., 991.5], [120., 130.]))
    np.testing.assert_allclose(crop.to_input(*crop.to_frame(17.25, 300.5)), (17.25, 300.5))
    # aimed: centred on (u, v) with v at the crop's horizon row, x0 aligned, y0 even, always inside the frame
    from openpilot.selfdrive.turnpilot.detector.graph import FAR_HORIZON_ROW
    c = crop_at(1344, 760, 672., 380.)
    assert c.x0 % FAR_ALIGN_X == 0 and c.y0 % 2 == 0 and abs(c.x0 + 320 - 672) <= FAR_ALIGN_X / 2
    assert abs(c.y0 + FAR_HORIZON_ROW - 380) <= 1
    for u, v in ((-500., -500.), (5000., 5000.), (10., 700.)):
      c = crop_at(1344, 760, u, v)
      assert 0 <= c.x0 <= 1344 - 640 and 0 <= c.y0 <= 760 - 384 and c.x0 % FAR_ALIGN_X == 0 and c.y0 % 2 == 0

  def test_prepare_matches_numpy_at_any_offset(self):
    # the same compiled graph serves every aim: the crop is which views of the frame it is given (luma, chroma)
    from tinygrad import Tensor
    from openpilot.selfdrive.turnpilot.detector.graph import Crop, crop_views, make_prepare_crop
    w, h = 1344, 760
    stride, y_height, uv_height, _ = get_nv12_info(w, h)
    frame = np.random.default_rng(2).integers(0, 256, stride * (y_height + uv_height), dtype=np.uint8)
    prepare = make_prepare_crop(w, h)
    for x0, y0 in ((0, 0), (352, 120), (704, 376), (32, 2)):
      crop = Crop(w, h, x0, y0)
      (lo, ln), (co, cn) = crop_views(crop)
      got = prepare(Tensor(frame[lo:lo + ln]), Tensor(frame[co:co + cn])).numpy()
      assert got.shape == (1, 3, 384, 640)
      np.testing.assert_allclose(got[0], crop_reference(frame, w, h, crop), atol=1e-4, err_msg=f'crop at {x0}, {y0}')

  def test_views_stay_inside_the_frame_copy(self):
    from openpilot.selfdrive.turnpilot.detector.graph import Crop, crop_views
    for w, h in ((1344, 760), (1928, 1208)):
      stride, y_height, uv_height, _ = get_nv12_info(w, h)
      corner = Crop(w, h, (w - 640) // 32 * 32, h - 384)
      (lo, ln), (co, cn) = crop_views(corner)
      assert lo + ln <= stride * y_height and stride * y_height <= co and co + cn <= stride * (y_height + uv_height)
      assert ln == 384 * stride and cn == 192 * stride and lo % 32 == 0 and co % 32 == 0


class TestSidePrepare(OpenpilotTestCase):
  def test_gathers_through_the_maps(self):
    # the side views' gather (graph.make_prepare_side): luma bilinear from a base index and two weights, chroma nearest from
    # its own index, a masked pixel the letterbox's gray; against numpy on a random frame and random maps
    from tinygrad import Tensor
    from openpilot.selfdrive.turnpilot.detector.graph import make_prepare_side
    w, h, out_w, out_h = 64, 48, 16, 8
    stride, y_height, uv_height, _ = get_nv12_info(w, h)
    rng = np.random.default_rng(2)
    frame = rng.integers(0, 256, stride * (y_height + uv_height), dtype=np.uint8)
    n = out_w * out_h
    x0, y0 = rng.integers(0, w - 1, n), rng.integers(0, h - 1, n)
    luma_idx = (y0 * stride + x0).astype(np.int32)
    chroma_idx = (stride * y_height + rng.integers(0, h // 2, n) * stride + 2 * rng.integers(0, w // 2, n)).astype(np.int32)
    weights = np.stack([rng.integers(0, 256, n), rng.integers(0, 256, n), rng.random(n) < 0.8]).astype(np.uint8)
    luma_idx[weights[2] == 0] = 0
    img = make_prepare_side(w, h, (out_w, out_h))(Tensor(frame), Tensor(luma_idx), Tensor(chroma_idx), Tensor(weights)).numpy()
    assert img.shape == (1, 3, out_h, out_w)
    f = frame.astype(np.float32)
    fx, fy = weights[0] / 255., weights[1] / 255.
    y = (f[luma_idx] * (1 - fx) * (1 - fy) + f[luma_idx + 1] * fx * (1 - fy) + f[luma_idx + stride] * (1 - fx) * fy +
         f[luma_idx + stride + 1] * fx * fy)
    u, v = f[chroma_idx] - 128, f[chroma_idx + 1] - 128
    c = (y - 16) * 1.164
    rgb = np.clip(np.stack([c + 1.596 * v, c - 0.392 * u - 0.813 * v, c + 2.017 * u]) / 255, 0, 1)
    rgb = np.where(weights[2] == 1, rgb, PAD_VALUE / 255)
    np.testing.assert_allclose(img[0], rgb.reshape(3, out_h, out_w), atol=1e-4)
