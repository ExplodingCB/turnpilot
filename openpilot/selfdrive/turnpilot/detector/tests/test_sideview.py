"""The virtual side views (STOP_SIGNS.md section 1.3): level pinholes at yaw +-75 deg, f 904 px, 768x512, the horizon at row
205, gathered from the wide fisheye through index and weight maps computed from the calibration and the lens."""
from functools import cache

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.detector.sideview import (SIDE_HORIZON_ROW, SIDE_SIZE, SIDE_YAW_DEG, calib_moved_deg,
                                                             compute_geometry, side_f)
from openpilot.selfdrive.turnpilot.scene.ranging import wide_lens
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

WIDE = DEVICE_CAMERAS[('mici', 'os04c10')].wide_road  # the comma four: 1344x760
RPY, EULER = (-0.0004, 0.16485, 0.004535), (-2.1e-05, 0.00551, -0.00276)  # the demo route's calibration


@cache
def geometry(rpy=RPY, euler=EULER):
  return compute_geometry(wide_lens(WIDE, 'mici'), rpy, euler, WIDE.width, WIDE.height)


def textured_frame(geom, rng) -> np.ndarray:
  """A camerad-layout NV12 wide frame of fresh random texture (limited range luma 40-200)."""
  stride, y_height, uv_height, _ = get_nv12_info(geom.cam_w, geom.cam_h)
  frame = np.full(stride * (y_height + uv_height), 128, np.uint8)
  frame[:stride * y_height] = rng.integers(40, 200, stride * y_height, dtype=np.uint8)
  return frame


def horizon_pixel(geom, bearing_left_deg: float, elev_deg: float = 0.):
  """The wide frame's pixel of the calibrated ray bearing_left_deg left of forward, elev_deg above the horizon."""
  b, e = np.radians(bearing_left_deg), np.radians(elev_deg)
  ray = np.array([np.cos(b) * np.cos(e), -np.sin(b) * np.cos(e), -np.sin(e)])  # calib: x forward, y right, z down
  return geom.lens.project(geom.calib_from_view.T @ ray)


class TestSideView(OpenpilotTestCase):
  def test_the_c2x_views(self):
    assert SIDE_SIZE == (768, 512) and SIDE_YAW_DEG == {'sideLeft': -75., 'sideRight': 75.} and abs(SIDE_HORIZON_ROW - 204.8) < 1e-9
    assert side_f(wide_lens(WIDE, 'mici')) == 904.

  def test_a_fisheye_point_at_75_deg_maps_into_a_side_view_and_back(self):
    geom = geometry()
    for name, sign in (('sideLeft', 1.), ('sideRight', -1.)):
      for bearing, elev in ((75., 0.), (75., -3.), (62., 1.), (84., -1.)):
        u, v = horizon_pixel(geom, sign * bearing, elev)
        assert 0 < u < geom.cam_w and 0 < v < geom.cam_h
        vu, vv = geom.to_view(name, np.array([u]), np.array([v]))
        assert 0 <= vu[0] < SIDE_SIZE[0] and 0 <= vv[0] < SIDE_SIZE[1], (name, bearing, elev)
        bu, bv = geom.to_fisheye(name, vu, vv)
        assert abs(bu[0] - u) < 0.5 and abs(bv[0] - v) < 0.5, (name, bearing, bu[0] - u, bv[0] - v)
      # 75 deg on the horizon is the view's centre column and its horizon row: level, whatever the camera's pitch
      vu, vv = geom.to_view(name, *map(np.atleast_1d, horizon_pixel(geom, sign * 75.)))
      assert abs(vu[0] - SIDE_SIZE[0] / 2) < 0.05 and abs(vv[0] - SIDE_HORIZON_ROW) < 0.05

  def test_the_maps_sample_where_the_view_looks(self):
    # the gather's maps: the base luma index and the bilinear weights land on the fisheye point each view pixel looks at
    geom = geometry()
    stride, y_height, _, _ = get_nv12_info(geom.cam_w, geom.cam_h)
    w, h = SIDE_SIZE
    for name in ('sideLeft', 'sideRight'):
      m = geom.maps[name]
      assert m.luma_idx.shape == (w * h,) and m.luma_idx.dtype == np.int32 and m.weights.shape == (3, w * h)
      assert m.weights.dtype == np.uint8 and m.chroma_idx.dtype == np.int32
      rng = np.random.default_rng(3)
      ii = rng.integers(0, w * h, 2000)
      valid = m.weights[2, ii] == 1
      assert valid.mean() > 0.5
      ii = ii[valid]
      u, v = geom.to_fisheye(name, ii % w + 0.5, ii // w + 0.5)
      x0, y0 = m.luma_idx[ii] % stride, m.luma_idx[ii] // stride
      np.testing.assert_allclose(x0 + m.weights[0, ii] / 255. + 0.5, u, atol=0.01)
      np.testing.assert_allclose(y0 + m.weights[1, ii] / 255. + 0.5, v, atol=0.01)
      cy, cx = (m.chroma_idx[ii] - stride * y_height) // stride, (m.chroma_idx[ii] - stride * y_height) % stride // 2
      assert np.all(cx == np.floor(u / 2)) and np.all(cy == np.floor(v / 2))
      # past the fisheye frame's edge the view is masked, never gathered from outside the frame
      assert np.all(m.luma_idx[m.weights[2] == 0] == 0)
      assert m.luma_idx.max() + stride + 1 < stride * y_height

  def test_coverage_from_the_inner_edge_to_the_frame_edge(self):
    # the views see 52 deg to the frame edge on the horizon; their sectors are 5 deg of bearing, left positive
    geom = geometry()
    left, right = geom.maps['sideLeft'].sectors, geom.maps['sideRight'].sectors
    assert [s.inner for s in left][:2] == [50., 55.] and left[-1].inner >= 80. and len(left) >= 7
    assert [s.inner for s in right][:2] == [-50., -55.] and all(s.outer == s.inner - 5. for s in right)
    assert all(s.outer == s.inner + 5. for s in left)
    assert all(len(s.idx) >= 20 for s in left + right) and len(geom.ref_idx) > 1000
    assert 50. < geom.cover_deg['sideLeft'][0] < 54. and geom.cover_deg['sideLeft'][1] > 82.

  def test_calibration_movement(self):
    assert abs(calib_moved_deg(RPY, EULER, (RPY[0], RPY[1] + np.radians(0.31), RPY[2]), EULER) - 0.31) < 1e-6
    assert calib_moved_deg(RPY, EULER, RPY, (EULER[0], EULER[1], EULER[2] - np.radians(0.5))) > 0.3
    # recomputing for another pitch moves the horizon in the fisheye and keeps it on row 205 of the views
    other = geometry((RPY[0], RPY[1] + np.radians(2.), RPY[2]))
    u, v = horizon_pixel(other, 75.)
    vu, vv = other.to_view('sideLeft', np.array([u]), np.array([v]))
    assert abs(vv[0] - SIDE_HORIZON_ROW) < 0.05 and abs(v - horizon_pixel(geometry(), 75.)[1]) > 5.

  def test_box_to_fisheye_is_the_bounding_box_of_its_edges(self):
    geom = geometry()
    x0, y0, x1, y1 = geom.box_to_fisheye('sideRight', (300., 160., 380., 215.))
    u, v = geom.to_fisheye('sideRight', np.linspace(300., 380., 9), np.full(9, 160.))
    assert x0 <= u.min() + 1e-6 and x1 >= u.max() - 1e-6 and y0 <= v.min() + 1e-6
    assert x0 > geom.cam_w / 2  # the right view's boxes are right of centre
    # a box hanging past the frame's edge (85 deg is about column 543) is clipped to the frame
    x0, _, x1, _ = geom.box_to_fisheye('sideRight', (500., 160., 600., 215.))
    assert x1 == geom.cam_w and x0 < geom.cam_w - 5
