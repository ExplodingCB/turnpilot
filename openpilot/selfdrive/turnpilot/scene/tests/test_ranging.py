import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.transformations.camera import DEVICE_CAMERAS, get_view_frame_from_calib_frame, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.turnpilot.scene.ranging import Lens, calib_from_view, ground_point, road_lens, wide_lens

COMMA_FOUR = DEVICE_CAMERAS[('mici', 'os04c10')]
RPY = [0.01, 0.04, -0.02]
WIDE_EULER = [0.0, 0.005, -0.003]
HEIGHT = 1.35


def ground(*args) -> np.ndarray:
  xy = ground_point(*args)
  assert xy is not None
  return np.array(xy)


def to_pixel(lens: Lens, x: float, y_left: float, rpy, height, wide_euler=None):
  """Forward model: a ground point (x forward, y left) to a pixel."""
  view_from_calib = calib_from_view(rpy, wide_euler).T
  p_calib = np.array([x, -y_left, height])  # calib frame: y right, z down; the ground is `height` below the camera
  return lens.project(view_from_calib @ p_calib)


class TestRanging(OpenpilotTestCase):
  def test_pinhole_against_openpilot_projections(self):
    # project with upstream's own matrices, then range back
    lens = road_lens(COMMA_FOUR.narrow_road)
    K = COMMA_FOUR.narrow_road.intrinsics
    E = get_view_frame_from_calib_frame(0, 0, 0, HEIGHT)  # its height offset is in the view frame, so level only
    view_from_calib = view_frame_from_device_frame @ rot_from_euler(RPY)  # as the UI projects the path, z + height
    for x, y in ((8.0, 0.0), (20.0, 3.5), (45.0, -5.0), (5.0, 2.0)):
      p = K @ E @ np.array([x, -y, 0.0, 1.0])
      np.testing.assert_allclose(ground(lens, calib_from_view([0, 0, 0]), HEIGHT, *(p[:2] / p[2])), [x, y], rtol=1e-9, atol=1e-9)
      p = K @ view_from_calib @ np.array([x, -y, HEIGHT])
      np.testing.assert_allclose(ground(lens, calib_from_view(RPY), HEIGHT, *(p[:2] / p[2])), [x, y], rtol=1e-9, atol=1e-9)

  def test_fisheye_wide_round_trip(self):
    lens = wide_lens(COMMA_FOUR.wide_road, 'mici')
    assert lens.fisheye and lens.f == 452.0
    for x, y in ((3.0, 4.0), (10.0, -8.0), (6.0, 12.0), (30.0, 1.0)):
      u, v = to_pixel(lens, x, y, RPY, HEIGHT, WIDE_EULER)
      assert 0 <= u < lens.width and 0 <= v < lens.height
      np.testing.assert_allclose(ground(lens, calib_from_view(RPY, WIDE_EULER), HEIGHT, u, v), [x, y], rtol=1e-6, atol=1e-6)

  def test_fisheye_vs_pinhole_far_off_axis(self):
    # 45 deg to the side the two lens models disagree by a lot: the sim's pinhole must not be read as a fisheye
    fisheye, pinhole = wide_lens(COMMA_FOUR.wide_road, 'mici'), wide_lens(COMMA_FOUR.wide_road, 'pc')
    assert not pinhole.fisheye and pinhole.f == COMMA_FOUR.wide_road.focal_length
    u, v = to_pixel(fisheye, 6.0, 6.0, [0, 0, 0], HEIGHT)
    x_pin, _ = ground(pinhole, calib_from_view([0, 0, 0]), HEIGHT, u, v)
    assert abs(x_pin - 6.0) > 1.0

  def test_horizon_and_above(self):
    lens = road_lens(COMMA_FOUR.narrow_road)
    rot = calib_from_view([0, 0, 0])
    cx, cy = lens.center
    assert ground_point(lens, rot, HEIGHT, cx, cy) is None        # the horizon
    assert ground_point(lens, rot, HEIGHT, cx, cy - 50) is None   # sky
    np.testing.assert_allclose(ground(lens, rot, HEIGHT, cx, cy + lens.f * HEIGHT / 10.0), [10.0, 0.0], atol=1e-9)

  def test_pitch_moves_range(self):
    # positive rpyCalib pitch is a camera pitched down, which sees the same pixel nearer
    lens = road_lens(COMMA_FOUR.narrow_road)
    cx, cy = lens.center
    assert ground(lens, calib_from_view([0, 0.02, 0]), HEIGHT, cx, cy + 60)[0] < ground(lens, calib_from_view([0, 0, 0]), HEIGHT, cx, cy + 60)[0]


class TestLensFile(OpenpilotTestCase):
  # one lens for every consumer (STOP_SIGNS.md section 1.1): the S1 fit in maps/visibility.json when it has one, else the
  # repo's equidistant fit; each consumer says which (Lens.spec) so the stop-sign decider can refuse a mismatch
  def test_equisolid_round_trip_and_bearings(self):
    lens = Lens(1344, 760, 452., fisheye=True, k=0.3)
    for u, v in ((10., 380.), (672., 100.), (1300., 700.)):
      np.testing.assert_allclose(lens.project(lens.ray(u, v)), (u, v), atol=1e-6)
    us, vs = np.array([10., 672., 1300.]), np.array([380., 100., 700.])
    np.testing.assert_allclose(np.stack(lens.project_many(lens.rays(us, vs))), np.stack([us, vs]), atol=1e-6)
    np.testing.assert_allclose(lens.rays(us, vs)[1], lens.ray(672., 100.), atol=1e-12)
    # k 0.3 reads a frame-edge pixel wider than the equidistant fit: repo 80 deg is about 82.5 true (verify.md finding 1)
    repo = Lens(1344, 760, 452., fisheye=True)
    u = 672 - 452. * np.radians(80.)
    off_axis = lambda lens: np.degrees(np.arccos(lens.ray(u, 380.)[2]))  # noqa: E731
    assert abs(off_axis(repo) - 80.) < 1e-6 and 82. < off_axis(lens) < 83.

  def test_the_file_lens_or_the_repo_fit(self):
    import json
    import tempfile
    from pathlib import Path
    from openpilot.selfdrive.turnpilot.scene.ranging import file_lens
    with tempfile.TemporaryDirectory() as d:
      path = Path(d) / 'visibility.json'
      assert file_lens(path) is None  # no file: the repo fit
      lens = wide_lens(COMMA_FOUR.wide_road, 'mici', path)
      assert (lens.f, lens.k, lens.source) == (452., 0., 'repo') and lens.spec == 'equidistant f=452.0 k=0.0 repo'
      path.write_text(json.dumps({'lens': {}}))
      assert wide_lens(COMMA_FOUR.wide_road, 'mici', path).source == 'repo'  # an empty lens: not fitted yet
      path.write_text(json.dumps({'lens': {'model': 'equisolid', 'f': 450.5, 'k': 0.3, 'edgeDeg': {'left': 88., 'right': 87.}}}))
      lens = wide_lens(COMMA_FOUR.wide_road, 'mici', path)
      assert (lens.f, lens.k, lens.source, lens.fisheye) == (450.5, 0.3, 'visibility.json', True)
      assert lens.spec == 'equisolid f=450.5 k=0.3 visibility.json'
      path.write_text(json.dumps({'lens': {'model': 'equidistant', 'f': 451.}}))
      assert wide_lens(COMMA_FOUR.wide_road, 'mici', path).spec == 'equidistant f=451.0 k=0.0 visibility.json'
      # the sim's pinhole stays a pinhole whatever the file says (a comma four's lens is not the sim's)
      assert not wide_lens(COMMA_FOUR.wide_road, 'pc', path).fisheye
