"""Lane paint on synthetic road frames: a flat road rendered through the comma four road camera's real projection, in camerad's
padded NV12, with modelV2-style lane lines where the paint is."""
import time

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.scene.lane_paint import (UNKNOWN, LanePaintClassifier, LinePaint, fill_lane_paint, lane_paint_from_msg,
                                                            project, runs, PAINT, ROAD, INVALID)
from openpilot.selfdrive.turnpilot.scene.ranging import calib_from_view, road_lens
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

LENS = road_lens(DEVICE_CAMERAS[('mici', 'os04c10')].narrow_road)
W, H = LENS.width, LENS.height
STRIDE, Y_HEIGHT, UV_HEIGHT, _ = get_nv12_info(W, H)
RPY = [0.0, 0.03, 0.0]
HEIGHT = 1.35
LINE_YS = (-5.4, -1.8, 1.8, 5.4)   # modelV2's four lines, y right
LINE_W = 0.12
ASPHALT, CONCRETE = (90., 128., 128.), (150., 122., 130.)
WHITE, YELLOW = (205., 128., 128.), (150., 96., 150.)
YELLOW_ON_CONCRETE = (138., 106., 142.)  # the demo route's freeway: darker than the concrete, low U, high V
V_EGO, DT = 15., 0.1


def _ground():
  """Ground (x forward, y right) under every pixel, nan above the horizon."""
  u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
  rays = np.stack([(u - LENS.center[0]) / LENS.f, (v - LENS.center[1]) / LENS.f, np.ones_like(u)], axis=-1) @ calib_from_view(RPY).T
  down = rays[..., 2] > 1e-3
  scale = np.where(down, HEIGHT / np.where(down, rays[..., 2], 1.), np.nan)
  return rays[..., 0] * scale, rays[..., 1] * scale


GX, GY = _ground()


def solid(color, y0=0., width=LINE_W):
  return lambda x, y: (np.abs(y - y0) < width / 2, color)


def dashed(color, dash=3., cycle=12., phase=0.):
  return lambda x, y: ((np.abs(y) < LINE_W / 2) & (np.mod(x + phase, cycle) < dash), color)


def double(color, gap=0.12):
  half = (LINE_W + gap) / 2
  return lambda x, y: ((np.abs(np.abs(y) - half) < LINE_W / 2), color)


def botts_dots():
  """California's Botts' dots: 10 cm white dots, groups of 4 at 0.91 m, a 7.3 m cycle."""
  def mark(x, y):
    k = np.mod(x, 7.3)
    near = np.min([np.hypot(k - 0.91 * j, y) for j in range(4)], axis=0)
    return near < 0.05, WHITE
  return mark


def render(marks, s, road=ASPHALT, gain=1., crosswalk=False, car=None, seed=0, extra=(), shade=None, noise=2.):
  """NV12 frame with the car at s m along the road. marks: per modelV2 line, f(x, y about the line) -> (mask, yuv) or None.
  car: (x0, x1, y0, y1) m of a dark box standing on the road, drawn as its pixel rectangle; returns (frame, its box).
  extra: f(x, y) -> (mask, yuv) over the whole road, after the marks; shade: f(x, y) -> luma factor (shadows, glare)."""
  rng = np.random.default_rng(seed)
  img = np.empty((H, W, 3), np.float32)
  img[:] = road
  x = GX + s
  for y0, mark in zip(LINE_YS, marks, strict=True):
    if mark is not None:
      m, color = mark(x, GY - y0)
      img[m] = color
  for mark in extra:
    m, color = mark(x, GY)
    img[m] = color
  if crosswalk:  # continental bars along the lanes, 0.6 m wide and apart, 3 m deep, 12-15 m ahead of where the car starts
    img[(np.mod(GY, 1.2) < 0.6) & (np.abs(x - 13.5) < 1.5)] = WHITE
  box = None
  if car is not None:
    pts = np.array([[cx, cy, HEIGHT - dz] for cx in car[:2] for cy in car[2:] for dz in (0., 1.5)])
    u, v, _ = project(LENS, calib_from_view(RPY).T, pts)
    box = np.array([[max(u.min(), 0), max(v.min(), 0), min(u.max(), W - 1), min(v.max(), H - 1)]])
    x0, y0, x1, y1 = box[0].astype(int)
    img[y0:y1, x0:x1] = (30., 128., 128.)
  if shade is not None:
    img[..., 0] *= np.where(np.isnan(GX), 1., shade(np.nan_to_num(x), np.nan_to_num(GY)))
  img[np.isnan(GX)] = (170., 140., 120.)  # sky
  img[..., 0] *= gain
  img[..., 0] += rng.normal(0., noise, (H, W))
  img = np.clip(np.rint(img), 0, 255).astype(np.uint8)
  out = np.zeros((Y_HEIGHT + UV_HEIGHT, STRIDE), np.uint8)
  out[:H, :W] = img[..., 0]
  uv = img[..., 1:].reshape(H // 2, 2, W // 2, 2, 2).astype(np.float32).mean(axis=(1, 3))
  out[Y_HEIGHT:Y_HEIGHT + H // 2, :W] = np.rint(uv).astype(np.uint8).reshape(H // 2, W)
  return out.reshape(-1), box


def lines_xyz(ys=LINE_YS):
  x = np.linspace(0., 100., 33)
  return [(x, np.full_like(x, y), np.full_like(x, HEIGHT)) for y in ys]


def drive(marks, seconds=2., probs=(0.9,) * 4, calibrated=True, boxes_at=None, **kw):
  """The classifier's answer after seconds of driving at V_EGO over the marks (as render's)."""
  clf = LanePaintClassifier(LENS)
  paint = [UNKNOWN] * 4
  for k in range(int(seconds / DT)):
    frame, box = render(marks, V_EGO * DT * k, seed=k, **kw)
    paint = clf.update(k * DT, frame, STRIDE, Y_HEIGHT, RPY, calibrated, lines_xyz(), list(probs), box if boxes_at is None else boxes_at)
  return paint


def kinds(paint):
  return [(p.kind, p.color) for p in paint]


class TestLanePaint(OpenpilotTestCase):
  def test_road_markings(self):
    paint = drive([double(YELLOW), dashed(WHITE), solid(WHITE), None])
    assert kinds(paint) == [('solid', 'yellow'), ('broken', 'white'), ('solid', 'white'), ('none', 'unknown')], paint
    assert paint[1].confidence > 0.8 and paint[2].confidence > 0.8 and paint[0].confidence > 0.6
    assert 0.2 < paint[1].coverage < 0.5 and paint[2].coverage > 0.9
    # on concrete: yellow no brighter than the concrete, Botts' dots, a dash. The dots read unknown, not broken: a 10 cm dot
    # falls between samples more often than not, and marks that short are what parking T-marks look like
    paint = drive([solid(YELLOW_ON_CONCRETE), botts_dots(), dashed(WHITE, phase=5.), None], road=CONCRETE)
    assert kinds(paint)[0] == ('solid', 'yellow') and paint[1].kind == 'unknown' and kinds(paint)[2] == ('broken', 'white'), paint

  def test_brightness(self):
    for gain in (0.45, 1.15):
      paint = drive([None, dashed(WHITE), solid(WHITE), double(YELLOW)], gain=gain)
      assert kinds(paint)[1:] == [('broken', 'white'), ('solid', 'white'), ('solid', 'yellow')], (gain, paint)

  def test_car_over_a_solid_line(self):
    car = (14., 18.5, 0.9, 2.7)  # a car astride the ego right line, 14-18.5 m ahead (it keeps pace)
    paint = drive([None, dashed(WHITE), solid(WHITE), None], car=car)
    assert kinds(paint)[1:3] == [('broken', 'white'), ('solid', 'white')], paint
    # without the detector's box the line under the car mustn't read broken
    paint = drive([None, dashed(WHITE), solid(WHITE), None], car=car, boxes_at=np.zeros((0, 6)))
    assert paint[2].kind != 'broken', paint

  def test_crosswalk_bars_are_not_a_broken_line(self):
    for s0 in (0., 6.):
      clf = LanePaintClassifier(LENS)
      for k in range(15):  # the car drives up to the crosswalk and over its near edge
        s = s0 + 1. * k
        frame, _ = render([None, None, solid(WHITE), None], s, crosswalk=True, seed=k)
        paint = clf.update(k * DT, frame, STRIDE, Y_HEIGHT, RPY, True, lines_xyz(), [0.9] * 4)
        assert paint[1].kind != 'broken' and paint[2].kind != 'broken', (s, paint)

  def test_unknown_when_unsure(self):
    marks = [None, dashed(WHITE), solid(WHITE), None]
    assert drive(marks, calibrated=False) == [UNKNOWN] * 4
    assert drive(marks, probs=(0.9, 0.2, 0.9, 0.9))[1] == UNKNOWN
    assert drive(marks, seconds=0.25) == [UNKNOWN] * 4  # too few frames yet
    # lines beyond the frame's edge
    clf = LanePaintClassifier(LENS)
    for k in range(20):
      frame, _ = render(marks, V_EGO * DT * k, seed=k)
      paint = clf.update(k * DT, frame, STRIDE, Y_HEIGHT, RPY, True, lines_xyz((-30., -1.8, 1.8, 30.)), [0.9] * 4)
    assert paint[0] == UNKNOWN and paint[3] == UNKNOWN and paint[2].kind == 'solid'
    # stale frames start over; so does a line that jumps sideways (the model renumbering lines in a lane change)
    assert clf.update(20 * DT + 1., frame, STRIDE, Y_HEIGHT, RPY, True, lines_xyz((-30., -1.8, 1.8, 30.)), [0.9] * 4)[2] == UNKNOWN
    clf = LanePaintClassifier(LENS)
    for k in range(20):
      frame, _ = render(marks, V_EGO * DT * k, seed=k)
      ys = (-30., -1.8, 1.8, 30.) if k < 19 else (-30., -1.8, -1.8 + 3.6, 30.)
      paint = clf.update(k * DT, frame, STRIDE, Y_HEIGHT, RPY, True, lines_xyz(), [0.9] * 4) if k < 19 else \
        clf.update(k * DT, frame, STRIDE, Y_HEIGHT, RPY, True, lines_xyz(ys), [0.9] * 4)
    assert paint[1].kind == 'broken'

  def test_runs(self):
    s = np.array([PAINT] * 12 + [ROAD] * 36 + [PAINT] * 12 + [ROAD] * 9)
    assert runs(s) == (3., True)
    assert runs(np.array([PAINT] * 12 + [ROAD] * 20 + [INVALID] + [ROAD] * 15 + [PAINT] * 12)) == (3., False)  # not closed
    assert runs(np.array([PAINT] * 10 + [ROAD] * 60)) == (2.5, False)
    assert runs(np.array([ROAD] * 69)) == (0., False)

  def test_message_round_trip_and_old_logs(self):
    from openpilot.cereal import messaging
    msg = messaging.new_message('turnpilotDetections')
    paint = [UNKNOWN, LinePaint('broken', 'white', 0.75, 0.3), LinePaint('solid', 'yellow', 0.5, 0.9), LinePaint('none', 'unknown', 1., 0.)]
    fill_lane_paint(msg.turnpilotDetections.init('lanePaint', 4), paint)
    back = lane_paint_from_msg(msg.as_reader().turnpilotDetections.lanePaint)
    assert [(p.kind, p.color) for p in back] == kinds(paint) and abs(back[1].confidence - 0.75) < 1e-6
    assert lane_paint_from_msg(messaging.new_message('turnpilotDetections').as_reader().turnpilotDetections.lanePaint) == (UNKNOWN,) * 4

  def test_cost(self):
    clf = LanePaintClassifier(LENS)
    frame, _ = render([double(YELLOW), dashed(WHITE), solid(WHITE), None], 0.)
    lines = lines_xyz()
    for _ in range(3):
      clf.update(0., frame, STRIDE, Y_HEIGHT, RPY, True, lines, [0.9] * 4)
    t0 = time.perf_counter()
    for k in range(20):
      clf.update(k * DT, frame, STRIDE, Y_HEIGHT, RPY, True, lines, [0.9] * 4)
    assert (time.perf_counter() - t0) / 20 < 0.01  # ~1 ms on an M-series Mac; generous for CI


def not_broken_white(p):
  return not p.is_('broken', 'white', 0.)


def worn(color, keep=0.6, seed=1):
  """A solid line with paint gone in random 0.3-2 m stretches, keep of it left."""
  rng = np.random.default_rng(seed)
  edges = np.cumsum(rng.uniform(0.3, 2., 400))
  painted = rng.uniform(size=400) < keep
  def mark(x, y):
    idx = np.clip(np.searchsorted(edges, x), 0, 399)
    return (np.abs(y) < LINE_W / 2) & painted[idx], color
  return mark


class TestAdversarialFootage(OpenpilotTestCase):
  """Hard cases synthesized on the road camera's projection: none of them may read broken white where there is no
  broken white line, and a real broken white line may read unknown (no change), never solid or yellow-free broken elsewhere."""

  def test_parking_t_marks(self):
    # 0.9 m bars every 6.5 m along the right line, a parking strip beyond: not a lane line
    def t_marks(x, y):
      return ((np.abs(y) < LINE_W / 2) & (np.mod(x, 6.5) < 0.9), WHITE)
    for _ in range(2):
      paint = drive([None, dashed(WHITE), t_marks, None])
      assert not_broken_white(paint[2]) and paint[1].is_('broken', 'white', 0.6), paint

  def test_lane_drop_dotted_line(self):
    # MUTCD's dotted line (0.9 m dashes, 2.7 m gaps) beside a lane that ends: not a lane to move into
    paint = drive([None, None, dashed(WHITE, dash=0.9, cycle=3.6), None])
    assert not_broken_white(paint[2]), paint

  def test_shadows_over_a_solid_line(self):
    # dappled shade from trees: 1-4 m patches across the whole road at 45% luma
    rng = np.random.default_rng(3)
    edges = np.cumsum(rng.uniform(1., 4., 200))
    dark = rng.uniform(size=200) < 0.5
    def dappled(x, y):
      return np.where(dark[np.clip(np.searchsorted(edges, x), 0, 199)], 0.45, 1.)
    paint = drive([None, None, solid(WHITE), None], shade=dappled)
    assert not_broken_white(paint[2]), paint
    # poles' shadows across the road every 8 m, 0.4 m wide
    def poles(x, y):
      return np.where(np.mod(x, 8.) < 0.4, 0.4, 1.)
    paint = drive([None, None, solid(WHITE), double(YELLOW)], shade=poles)
    assert not_broken_white(paint[2]) and not_broken_white(paint[3]), paint
    # a hedge's shadow edge running along the solid line: one side dark
    def hedge(x, y):
      return np.where(y > LINE_YS[2] + 0.05, 0.5, 1.)
    paint = drive([None, None, solid(WHITE), None], shade=hedge)
    assert not_broken_white(paint[2]), paint

  def test_worn_solid_line(self):
    # lightly worn (a fifth of the paint gone in random stretches): never broken white
    for seed in range(4):
      paint = drive([None, worn(WHITE, keep=0.8, seed=seed), None, None])
      assert not_broken_white(paint[1]), (seed, paint)
    # KNOWN LIMIT: with most of the paint gone in random stretches a solid line looks like a broken one, and it can read
    # broken white. What keeps that from a change into a shoulder is the lane planner's check that the model sees a lane
    # there (test_lane_planner: a worn edge line beside a shoulder is no lane); between two travel lanes the driver confirms
    heavily = [drive([None, worn(WHITE, keep=0.4, seed=seed), None, None])[1] for seed in range(4)]
    assert any(p.is_('broken', 'white', 0.6) for p in heavily)  # if this fails, the limit is gone: update the docstring

  def test_tar_seam_and_crack_sealant(self):
    # a dark tar seam where the model puts a line: no paint
    seam = solid((45., 128., 128.), width=0.05)
    paint = drive([None, None, seam, None])
    assert not_broken_white(paint[2]), paint
    # light crack sealant in irregular streaks, a line the model is unsure of: never broken
    sealant = worn((150., 128., 128.), keep=0.35, seed=7)
    paint = drive([None, None, sealant, None], probs=(0.9, 0.9, 0.6, 0.9))
    assert paint[2].kind != 'broken', paint
    # and a real dashed line the model is unsure of reads unknown, not broken: costs a change, never adds one
    paint = drive([None, dashed(WHITE), None, None], probs=(0.9, 0.6, 0.9, 0.9))
    assert paint[1].kind == 'unknown', paint

  def test_stop_bar_and_gore_chevrons(self):
    # a stop bar across the lanes 13 m ahead of the start, crossed as the car drives up
    for s0 in (0., 5.):
      clf = LanePaintClassifier(LENS)
      def bar(x, y, s0=s0):
        return ((np.abs(x - 13.5 - s0) < 0.3) & (np.abs(y) < 6.), WHITE)
      for k in range(15):
        frame, _ = render([None, None, solid(WHITE), None], 1.0 * k, seed=k, extra=[bar])
        paint = clf.update(k * DT, frame, STRIDE, Y_HEIGHT, RPY, True, lines_xyz(), [0.9] * 4)
        assert not_broken_white(paint[2]), (s0, k, paint)
    # a gore: chevrons between the right line and the far right one (a solid white edge each side of the hatched area)
    def chevrons(x, y):
      return ((y > LINE_YS[2] + 0.2) & (y < LINE_YS[3] - 0.2) & (np.mod(x + (y - LINE_YS[2]) * 2., 6.) < 0.45), WHITE)
    paint = drive([None, dashed(WHITE), solid(WHITE), solid(WHITE)], extra=[chevrons])
    assert not_broken_white(paint[2]) and not_broken_white(paint[3]), paint

  def test_night_and_glare(self):
    # night: a dim road and a noisy sensor. The dashes may still read; solid and yellow never read broken white
    for gain, noise in ((0.3, 4.), (0.2, 6.)):
      paint = drive([double(YELLOW), dashed(WHITE), solid(WHITE), None], gain=gain, noise=noise)
      assert not_broken_white(paint[0]) and not_broken_white(paint[2]), (gain, paint)
      assert paint[1].kind in ('broken', 'unknown'), (gain, paint)
    # rain glare: a bright smear down the lane, the road near saturation where it is
    def glare(x, y):
      return np.where(np.abs(y - 0.5) < 2.5, 2.4, 1.)
    paint = drive([None, dashed(WHITE), solid(WHITE), None], shade=glare)
    assert not_broken_white(paint[2]), paint

  def test_double_yellow_at_the_ego_line_stops_the_crossing(self):
    # BeamNG shakedown lane_change (2026-10-27): after the change into the turn lane, the double solid yellow centreline is the
    # ego left line, with broken white between the oncoming lanes beyond it, broken white on the right and the curb lane's
    # unpainted edge. Lines where modelV2 logged them then (test_lane_planner XING_POST); the paint is this file's
    # synthetic palette: no road-camera frames were recorded in that run, so BeamNG's own yellow is not checked here
    from openpilot.selfdrive.turnpilot.lane_planner import LEFT, change_blocker, model_lanes
    from openpilot.selfdrive.turnpilot.tests.test_lane_planner import XING_POST
    ys = XING_POST[0]
    marks = [dashed(WHITE), double(YELLOW), dashed(WHITE), None]
    def at(mark, y0):
      return lambda x, y: mark(x, y - y0)
    clf = LanePaintClassifier(LENS)
    for k in range(20):
      frame, _ = render([None] * 4, V_EGO * DT * k, seed=k, extra=[at(m, y0) for m, y0 in zip(marks, ys, strict=True) if m is not None])
      paint = clf.update(k * DT, frame, STRIDE, Y_HEIGHT, RPY, True, lines_xyz(ys), list(XING_POST[1]))
    assert (paint[1].kind, paint[1].color) == ('solid', 'yellow') and paint[1].confidence >= 0.3, paint
    assert change_blocker(LEFT, model_lanes(*XING_POST), paint) == 'lineNotBroken'

  def test_white_paint_in_warm_sun_is_not_yellow(self):
    # comma's public comma four route 79d0f04c82980004/00000003--fb0b5d9cfc (a Model 3, California afternoon sun), segment 15:
    # the solid white bike-lane line on the car's right, median of 30608 painted samples: asphalt Y 76, U 126, V 130, the
    # paint Y 125 with its U 4 below the road's and V 3 above, the light's tint on a neutral surface. Without the
    # chromaticity test it read yellow (worn yellow's U dip) and a replay raised wrongSideTakeControl 18 times in that minute.
    # Segment 1's double yellow in the same light (U 12 below, V 9 above) still reads yellow
    from openpilot.selfdrive.turnpilot.turnpilotd import yellow_on_the_right
    warm_road, warm_white, warm_yellow = (76., 126., 130.), (125., 122., 133.), (108., 114., 139.)
    paint = drive([double(warm_yellow), dashed(warm_white), solid(warm_white), None], road=warm_road)
    assert kinds(paint)[0] == ('solid', 'yellow') and paint[0].confidence >= 0.5, paint
    assert paint[1].color != 'yellow' and paint[2].color != 'yellow' and paint[2].kind == 'solid', paint
    assert not yellow_on_the_right(paint, list(LINE_YS), [0.9] * 4, [-7., 3.5], [0.5, 0.5])[0]
    # a worn double yellow in neutral light (the demo route's: U 4-10 below the road) still reads yellow
    paint = drive([double((120., 121., 134.)), dashed(WHITE), solid(WHITE), None])
    assert kinds(paint)[0] == ('solid', 'yellow'), paint

  def test_a_tint_without_yellows_chromaticity_reads_neither_colour(self):
    # the same public route, segment 17, the sun ahead: its double yellow centre line washed out to a U dip of 4 below the
    # road at a bright peak, which yellow's chromaticity test refuses. It must read colour-unknown, never white: white would
    # let the lane planner treat a centre line as a lane line
    paint = drive([None, solid((200., 124., 129.)), dashed(WHITE), None])
    assert paint[1].color == 'unknown' and paint[1].kind == 'solid', paint
    assert kinds(paint)[2] == ('broken', 'white'), paint
