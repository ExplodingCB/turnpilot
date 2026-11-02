"""Lane paint on synthetic road frames: a flat road rendered through the comma four road camera's real projection, in camerad's
padded NV12, with modelV2-style lane lines where the paint is."""
import time

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.turnpilot.scene.lane_paint import (COLOR_PROB_MIN, LINE_PROB_MIN, MIN_FRAMES, UNKNOWN, LanePaintClassifier, LinePaint,
                                                            fill_lane_paint, lane_paint_from_msg,
                                                            project, runs, PAINT, ROAD, INVALID)
from openpilot.selfdrive.turnpilot.scene import lane_paint as lp
from openpilot.selfdrive.turnpilot.scene.lane_paint import sample_lines
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


# The Saturday re-run's route_demo at 6ba8582 (sim/rerun/runs/R8_route_demo/route_demo-20261031-125919): out of the
# reroute's third right the model held the oncoming lane, 3.9-4.1 m past the divider, at 5.1 down to 3.0 m/s on the road
# camera, every lanePaint read unknown, the right line's laneLineProbs 0.22-0.47. modelV2's four lines and two road edges at
# SAMPLE_IDX (12 m), laneLineProbs and roadEdgeStds of each road camera read from 120.45 s to 123.06 s, with carState's
# vEgo: (t, v, line ys, probs, edge ys, edge stds). The wrong-side handback came at 144.2 s, 109.9 m on, once the prob rose
# past 0.5 and the reader read the line yellow 1.0 within three frames
R8_ROUTE_DEMO_ONCOMING = (
  (120.45, 5.1, (-4.26, -1.78, 1.45, 4.27), (0.0, 0.33, 0.36, 0.28), (-2.3, 4.86), (0.51, 0.69)),
  (120.55, 5.12, (-4.18, -1.75, 1.43, 4.0), (0.0, 0.26, 0.32, 0.17), (-2.16, 4.33), (0.54, 0.72)),
  (120.67, 5.14, (-4.27, -1.77, 1.54, 4.09), (0.0, 0.35, 0.44, 0.2), (-2.27, 5.07), (0.54, 0.71)),
  (120.75, 5.16, (-4.05, -1.73, 1.42, 3.95), (0.0, 0.36, 0.47, 0.21), (-2.22, 4.79), (0.59, 0.78)),
  (120.86, 5.18, (-4.1, -1.7, 1.51, 4.05), (0.0, 0.29, 0.34, 0.22), (-2.06, 4.9), (0.57, 0.88)),
  (120.95, 5.16, (-4.02, -1.71, 1.57, 4.0), (0.0, 0.27, 0.36, 0.11), (-2.18, 4.66), (0.66, 0.87)),
  (121.06, 5.13, (-4.19, -1.73, 1.73, 4.37), (0.0, 0.39, 0.44, 0.25), (-2.35, 5.33), (0.69, 0.82)),
  (121.16, 5.11, (-4.2, -1.76, 1.88, 4.66), (0.0, 0.2, 0.36, 0.19), (-2.24, 5.56), (0.57, 0.81)),
  (121.25, 4.98, (-4.32, -1.76, 1.94, 4.75), (0.0, 0.3, 0.31, 0.19), (-2.35, 5.31), (0.63, 0.85)),
  (121.35, 4.71, (-4.21, -1.76, 1.61, 4.2), (0.0, 0.31, 0.26, 0.09), (-2.29, 4.74), (0.62, 0.74)),
  (121.46, 4.4, (-4.11, -1.73, 1.63, 4.18), (0.0, 0.26, 0.28, 0.06), (-2.2, 5.0), (0.65, 0.89)),
  (121.56, 4.2, (-4.29, -1.75, 1.71, 4.46), (0.0, 0.25, 0.24, 0.11), (-2.25, 5.29), (0.59, 0.95)),
  (121.65, 4.0, (-4.18, -1.76, 1.72, 4.18), (0.0, 0.3, 0.32, 0.13), (-2.15, 5.26), (0.57, 0.85)),
  (121.76, 3.91, (-4.18, -1.78, 1.72, 4.22), (0.0, 0.25, 0.31, 0.17), (-2.4, 5.32), (0.59, 0.8)),
  (121.86, 3.84, (-4.37, -1.73, 1.8, 4.61), (0.0, 0.31, 0.33, 0.31), (-2.36, 5.79), (0.55, 0.77)),
  (121.96, 3.84, (-4.27, -1.67, 1.79, 4.54), (0.0, 0.33, 0.26, 0.21), (-2.41, 5.46), (0.58, 0.76)),
  (122.05, 3.81, (-4.36, -1.76, 1.89, 4.68), (0.0, 0.26, 0.33, 0.25), (-2.57, 5.76), (0.6, 0.8)),
  (122.16, 3.76, (-4.46, -1.78, 1.89, 4.77), (0.0, 0.23, 0.32, 0.25), (-2.6, 5.82), (0.59, 0.81)),
  (122.26, 3.71, (-4.26, -1.76, 1.89, 4.51), (0.0, 0.19, 0.3, 0.24), (-2.44, 5.18), (0.65, 0.77)),
  (122.35, 3.61, (-4.27, -1.79, 1.92, 4.6), (0.0, 0.27, 0.26, 0.19), (-2.49, 4.85), (0.6, 0.7)),
  (122.45, 3.5, (-4.48, -1.81, 2.03, 4.75), (0.0, 0.25, 0.35, 0.25), (-2.5, 6.33), (0.61, 0.92)),
  (122.56, 3.43, (-4.47, -1.82, 1.98, 4.74), (0.0, 0.22, 0.27, 0.19), (-2.43, 5.84), (0.59, 0.9)),
  (122.65, 3.35, (-4.45, -1.82, 1.88, 4.58), (0.0, 0.2, 0.27, 0.19), (-2.34, 5.78), (0.6, 0.98)),
  (122.76, 3.26, (-4.33, -1.77, 1.82, 4.66), (0.0, 0.17, 0.22, 0.18), (-2.3, 5.68), (0.58, 0.89)),
  (122.86, 3.2, (-4.37, -1.76, 1.79, 4.59), (0.0, 0.22, 0.26, 0.17), (-2.29, 5.68), (0.58, 0.96)),
  (122.96, 3.12, (-4.45, -1.78, 1.79, 4.47), (0.0, 0.23, 0.32, 0.24), (-2.28, 5.8), (0.6, 0.9)),
  (123.06, 3.02, (-4.32, -1.76, 1.78, 4.43), (0.0, 0.21, 0.25, 0.16), (-2.12, 5.43), (0.6, 0.9)),
)
CENTRE_Y = 1.75  # the run's centre line, where its right line sat (1.42-2.03 m, inside SEARCH_HALF_M of it on every read)


def read_rows(rows, marks_at, probs_of=lambda probs: probs):
  """The reader over the rows' own lines and probs (probs_of may change them), the paint fixed on the road at marks_at's
  (mark, y) and the car moving at each row's v: the reader's four LinePaints per row."""
  clf, s, out = LanePaintClassifier(LENS), 0., []
  for k, (t, v, line_ys, probs, _edge_ys, _stds) in enumerate(rows):
    if k:
      s += v * (t - rows[k - 1][0])
    extra = [(lambda x, y, m=m, y0=y0: m(x, y - y0)) for m, y0 in marks_at]
    frame, _ = render([None] * 4, s, seed=k, extra=extra)
    out.append(clf.update(t, frame, STRIDE, Y_HEIGHT, RPY, True, lines_xyz(line_ys), list(probs_of(probs))))
  return out


def test_the_centre_line_under_a_line_the_model_is_unsure_of_reads_yellow_and_the_wrong_side_handback_comes():
  """R8 route_demo at 6ba8582: after a right, the model held the oncoming lane for 109.9 m and stood in it, the yellow centre
  line on its right under a right line the model gave 0.2-0.47, and the reader read no line under LINE_PROB_MIN, so the
  wrong-side handback waited 25 s for the prob to pass 0.5 (it read yellow at once then). Yellow is the cautious answer:
  a line the model gives COLOR_PROB_MIN or more reads yellow when its paint is yellow, kind unknown, and the handback
  comes on the run's own lines, probs, edges and speeds within 2 s of the first read, before the car stood at 128 s."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import wrong_side_rows
  from openpilot.selfdrive.turnpilot.turnpilotd import WRONG_SIDE_CONF
  rows = R8_ROUTE_DEMO_ONCOMING
  paint = read_rows(rows, [(double(YELLOW), CENTRE_Y)])
  assert all(max(r[3]) < LINE_PROB_MIN for r in rows)  # every line under the old floor on every read
  yellow = [p[2] for p in paint if p[2].color == 'yellow']
  assert len(yellow) >= len(rows) - 6 and all(p.kind == 'unknown' for p in yellow), paint
  assert max(p.confidence for p in yellow) >= WRONG_SIDE_CONF
  assert all(p[i] == UNKNOWN for p in paint for i in (0, 1, 3)), paint  # nothing painted there: nothing read
  fired = wrong_side_rows([(t, v, True, [(q.kind, q.color, q.confidence) for q in p], ys, probs, eys, stds)
                           for (t, v, ys, probs, eys, stds), p in zip(rows, paint, strict=True)])
  assert fired and fired[0] <= rows[0][0] + 2., fired
  # the planner never plans on an unsure line's yellow: the guard's alone
  from openpilot.selfdrive.turnpilot.turnpilotd import sure_paint
  assert all(sure_paint(p, r[3])[2] == UNKNOWN for p, r in zip(paint, rows, strict=True))


# comma's public comma four route 79d0f04c82980004/00000003--fb0b5d9cfc (California), segment 4, 50.2-54.0 s, the car on
# its own side at 5-9 m/s: its right line at 0.2-0.37 read yellow 0.6-1.0 (the reader at COLOR_PROB_MIN), the left road
# edge 5.5-8.5 m out past a solid white left line. The reads every 0.2 s: (t, v, paint conf, line ys, probs, edge ys, stds)
TESLA_4_OWN_SIDE = (
  (50.2, 5.1, 0.62, (-4.21, -1.15, 1.88, 4.59), (0.22, 0.74, 0.37, 0.1), (-6.97, 6.23), (1.72, 1.13)),
  (50.4, 5.3, 0.61, (-4.25, -1.15, 1.97, 4.24), (0.22, 0.72, 0.27, 0.04), (-7.28, 5.56), (1.6, 1.02)),
  (50.6, 5.5, 0.63, (-4.15, -1.17, 1.92, 4.57), (0.19, 0.65, 0.32, 0.07), (-7.15, 5.97), (1.75, 1.15)),
  (50.8, 5.8, 0.67, (-4.34, -1.2, 1.88, 4.44), (0.21, 0.66, 0.39, 0.08), (-7.54, 5.85), (1.71, 1.14)),
  (51.0, 6.0, 0.65, (-4.21, -1.17, 1.92, 4.36), (0.2, 0.61, 0.35, 0.07), (-7.16, 5.63), (1.69, 1.13)),
  (51.2, 6.3, 0.68, (-4.08, -1.19, 1.82, 4.08), (0.19, 0.54, 0.33, 0.05), (-7.39, 5.27), (1.62, 1.12)),
  (51.4, 6.5, 0.7, (-4.21, -1.18, 1.81, 4.5), (0.18, 0.52, 0.29, 0.07), (-7.2, 5.79), (1.75, 1.11)),
  (51.6, 6.8, 0.81, (-4.23, -1.21, 1.87, 4.52), (0.15, 0.44, 0.24, 0.04), (-7.5, 5.61), (1.75, 1.09)),
  (51.8, 7.1, 0.85, (-4.37, -1.26, 1.77, 4.44), (0.17, 0.52, 0.28, 0.05), (-7.82, 5.79), (1.85, 1.17)),
  (52.0, 7.3, 0.81, (-4.47, -1.33, 1.8, 4.74), (0.13, 0.47, 0.23, 0.05), (-7.09, 6.0), (1.77, 1.18)),
  (52.2, 7.5, 0.64, (-4.38, -1.33, 1.71, 4.59), (0.11, 0.39, 0.25, 0.04), (-7.47, 6.03), (1.69, 1.18)),
  (52.4, 7.8, 1.0, (-4.25, -1.34, 1.71, 4.57), (0.11, 0.35, 0.23, 0.04), (-6.9, 6.02), (1.73, 1.26)),
  (52.6, 7.9, 1.0, (-4.29, -1.35, 1.73, 4.25), (0.14, 0.38, 0.2, 0.03), (-7.12, 5.53), (1.68, 1.34)),
  (52.8, 8.1, 1.0, (-4.44, -1.55, 1.6, 4.23), (0.08, 0.36, 0.24, 0.04), (-6.6, 5.9), (1.89, 1.42)),
  (53.0, 8.4, 1.0, (-4.35, -1.58, 1.81, 4.05), (0.09, 0.35, 0.21, 0.03), (-6.71, 5.87), (1.62, 1.25)),
  (53.2, 8.6, 1.0, (-4.44, -1.67, 1.68, 4.07), (0.08, 0.37, 0.22, 0.03), (-6.77, 5.88), (1.79, 1.25)),
  (53.4, 8.8, 1.0, (-4.21, -1.61, 1.65, 3.79), (0.05, 0.27, 0.2, 0.02), (-6.27, 5.59), (1.66, 1.19)),
  (53.6, 9.0, 1.0, (-4.06, -1.54, 1.64, 3.7), (0.08, 0.36, 0.22, 0.03), (-5.8, 5.54), (1.47, 1.2)),
  (53.8, 9.2, 1.0, (-4.19, -1.63, 1.48, 3.85), (0.05, 0.24, 0.19, 0.03), (-6.08, 5.76), (1.64, 1.34)),
  (54.0, 9.4, 1.0, (-3.89, -1.48, 1.51, 3.78), (0.08, 0.3, 0.24, 0.04), (-5.54, 5.86), (1.58, 1.24)),
)


def test_an_unsure_lines_yellow_with_the_left_road_edge_far_is_no_wrong_side():
  """The reader's unsure yellow alone is not enough for the wrong-side handback: on comma's California route the car on
  its own side read its unsure right line yellow for 4 s, the left road edge 5.5-8.5 m out, and nothing may fire. The same
  reads with the left road edge where R8's was (0.3-0.85 m past the left line: the leftmost lane) do fire."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import wrong_side_rows
  from openpilot.selfdrive.turnpilot.turnpilotd import WRONG_SIDE_UNSURE_EDGE_M
  def rows(left_edge_past=None):
    out = []
    for t, v, conf, ys, probs, eys, stds in TESLA_4_OWN_SIDE:
      eys = eys if left_edge_past is None else (ys[1] - left_edge_past, eys[1])
      for dt in (0., 0.1):  # the reader's 10 Hz
        out.append((t + dt, v, True, [('unknown', 'unknown', 0.), ('solid', 'white', 0.9), ('unknown', 'yellow', conf),
                                      ('unknown', 'unknown', 0.)], ys, probs, eys, stds))
    return out
  assert wrong_side_rows(rows()) == []
  assert wrong_side_rows(rows(left_edge_past=WRONG_SIDE_UNSURE_EDGE_M + 0.5)) == []
  fired = wrong_side_rows(rows(left_edge_past=0.85))
  assert fired and fired[0] <= TESLA_4_OWN_SIDE[0][0] + 1.5, fired


def test_a_line_the_model_is_unsure_of_reads_only_yellow():
  """Under LINE_PROB_MIN a line reads its paint's colour only when that is yellow (the cautious answer: never crossed,
  the wrong-side guard's), always kind unknown: white or no paint there reads unknown as before, and under COLOR_PROB_MIN
  nothing. The kind of a line the model is sure of comes from its sure frames only, as before."""
  rows = R8_ROUTE_DEMO_ONCOMING
  for mark in (solid(WHITE), dashed(WHITE)):
    assert all(p[2] == UNKNOWN for p in read_rows(rows, [(mark, CENTRE_Y)])), mark
  low = read_rows(rows, [(double(YELLOW), CENTRE_Y)], lambda probs: [min(q, COLOR_PROB_MIN - 0.01) for q in probs])
  assert all(p[2] == UNKNOWN for p in low), low
  # unsure reads of the centre line, then the model sure of it: its kind and colour wait for MIN_FRAMES of the sure reads,
  # as they did when unsure reads were never kept
  sure_from, clf, s = 15, LanePaintClassifier(LENS), 0.
  for k, (t, v, line_ys, probs, _edge_ys, _stds) in enumerate(rows):
    s += v * (t - rows[k - 1][0]) if k else 0.
    frame, _ = render([None] * 4, s, seed=k, extra=[lambda x, y: double(YELLOW)(x, y - CENTRE_Y)])
    sure = [probs[0], probs[1], 0.9 if k >= sure_from else probs[2], probs[3]]
    paint = clf.update(t, frame, STRIDE, Y_HEIGHT, RPY, True, lines_xyz(line_ys), sure)
    if sure_from <= k < sure_from + MIN_FRAMES - 1:
      assert paint[2] == UNKNOWN, (k, paint)
  assert paint[2].color == 'yellow' and paint[2].kind != 'broken', paint


def reference_sample_lines(nv12, stride, y_height, lens, rpy_calib, lines_xyz, boxes=None):
  """lane_paint.sample_lines as it was before its vectorizing (2026-11-01), every point of every profile rotated and every
  median and point taken one channel and window at a time: the plain statement of what the fast one must give."""
  def flanks(profile, at):
    return (np.median(np.take_along_axis(profile, at[..., None] - lp._FLANK, axis=2), axis=2),
            np.median(np.take_along_axis(profile, at[..., None] + lp._FLANK, axis=2), axis=2))

  def at_(profile, idx):
    return np.take_along_axis(profile, idx[..., None], axis=2)[..., 0]

  centers = np.empty((len(lines_xyz), len(lp.XS), 3))
  for i, (x, y, z) in enumerate(lines_xyz):
    centers[i, :, 0], centers[i, :, 1], centers[i, :, 2] = lp.XS, np.interp(lp.XS, x, y), np.interp(lp.XS, x, z)
  pts = np.repeat(centers[:, :, None, :], len(lp.OFFSETS), axis=2)
  pts[..., 1] += lp.OFFSETS
  u, v, front = project(lens, calib_from_view(rpy_calib).T, pts)
  ui, vi = np.rint(u).astype(np.int64), np.rint(v).astype(np.int64)
  inside = front & (ui >= 0) & (ui < lens.width) & (vi >= 0) & (vi < lens.height)
  ok = inside.all(axis=2)
  ui, vi = np.where(inside, ui, 0), np.where(inside, vi, 0)
  luma = nv12[vi * stride + ui].astype(np.float32)
  uv = (y_height + vi // 2) * stride + (ui // 2) * 2
  chroma_u, chroma_v = nv12[uv].astype(np.float32), nv12[uv + 1].astype(np.float32)
  peak = lp._SEARCH[np.argmax(luma[..., lp._SEARCH], axis=2)]
  road_l, road_r = flanks(luma, peak)
  road = np.maximum(road_l, road_r)
  step = np.abs(road_l - road_r) > np.maximum(lp.LUMA_MIN, lp.STEP_REL * road)
  ridge = at_(luma, peak) - road
  above = np.take_along_axis(luma, peak[..., None] + lp._WIDTH, axis=2) >= (road + ridge / 2)[..., None]
  bright = ridge >= np.maximum(lp.LUMA_MIN, lp.LUMA_REL * road)
  wide = bright & (above.sum(axis=2) * lp.PROFILE_STEP_M > lp.WIDTH_MAX_M)
  dip_idx = lp._SEARCH[np.argmin(chroma_u[..., lp._SEARCH], axis=2)]
  u_road, v_road, y_road = (flanks(c, dip_idx) for c in (chroma_u, chroma_v, luma))
  u_dip, v_dip = at_(chroma_u, dip_idx), at_(chroma_v, dip_idx)
  dip = u_dip - np.minimum(*u_road)
  dv = v_dip - 0.5 * np.add(*v_road)
  y_paint, y_flank = np.maximum(at_(luma, dip_idx) - 16, 1), np.maximum(0.5 * np.add(*y_road) - 16, 1)
  cu = (u_dip - 128) / y_paint - (0.5 * np.add(*u_road) - 128) / y_flank
  cv = (v_dip - 128) / y_paint - (0.5 * np.add(*v_road) - 128) / y_flank
  tint = ((dip <= lp.YELLOW_DU) | (bright & ~wide & (dip <= lp.WORN_YELLOW_DU))) & (dv >= lp.YELLOW_DV_MIN)
  yellow = tint & (cu <= lp.YELLOW_CU_MAX) & (cv >= lp.YELLOW_CV_MIN)
  du_peak = at_(chroma_u, peak) - 0.5 * np.add(*flanks(chroma_u, peak))
  dv_peak = at_(chroma_v, peak) - 0.5 * np.add(*flanks(chroma_v, peak))
  white = bright & ~wide & ~tint & (du_peak >= lp.WHITE_DU) & (dv_peak <= lp.WHITE_DV_MAX)
  ok &= road < lp.LUMA_SATURATED
  if boxes is not None and len(boxes):
    b = np.asarray(boxes, dtype=np.float32)[:, :4]
    row = vi[..., lp._C][..., None]
    u0, u1 = ui[..., 0][..., None], ui[..., -1][..., None]
    ok &= ~((row >= b[:, 1]) & (row <= b[:, 3]) & (np.maximum(u0, u1) >= b[:, 0]) & (np.minimum(u0, u1) <= b[:, 2])).any(axis=-1)
  painted = (bright & ~wide) | yellow
  state = np.where(~ok | (wide & ~yellow) | (step & ~painted), INVALID, np.where(painted, PAINT, ROAD))
  return state, np.where(state != PAINT, lp.NO_COLOR, np.where(yellow, lp.YELLOW, np.where(white, lp.WHITE, lp.NO_COLOR)))


def random_reads(n, seed=1):
  """n (frame, rpy, lines, boxes): rendered roads and random bytes, calibrations off nominal, lines straight to sharply
  curved and off the frame's edge, and boxes over them or none."""
  rng = np.random.default_rng(seed)
  frames = [render([double(YELLOW), dashed(WHITE), solid(WHITE), None], 3. * k, seed=k)[0] for k in range(2)]
  frames += [render([solid(YELLOW_ON_CONCRETE), botts_dots(), dashed(WHITE, phase=5.), None], 3., road=CONCRETE)[0]]
  frames += [rng.integers(0, 256, frames[0].shape, dtype=np.uint8) for _ in range(2)]  # every branch, whatever the pixels
  x = np.linspace(0., 100., 33)
  for k in range(n):
    rpy = [rng.normal(0, 0.02), rng.normal(0.03, 0.03), rng.normal(0, 0.05)]
    lines = [(x, y0 + rng.normal(0, 0.5) + rng.normal(0, 0.002) * rng.choice((0, 1, 5)) * x ** 2,
              HEIGHT + rng.normal(0, 0.1) + rng.normal(0, 0.002) * x) for y0 in LINE_YS]
    boxes = None
    if k % 3:
      us, vs = np.sort(rng.uniform(0, W, (3, 2)), axis=1), np.sort(rng.uniform(300, H, (3, 2)), axis=1)
      boxes = np.c_[us[:, 0], vs[:, 0], us[:, 1], vs[:, 1], np.ones((3, 2))]
    yield frames[k % len(frames)], rpy, lines, boxes


def test_sample_lines_reads_what_the_reference_reads():
  """The vectorized reader (one gather, uint8 profiles, the median network, the rays by linearity) against the plain one,
  sample for sample: rays summed in another order could round a pixel the other way at a half, and never did here."""
  for frame, rpy, lines, boxes in random_reads(150):
    got = sample_lines(frame, STRIDE, Y_HEIGHT, LENS, rpy, lines, boxes)
    want = reference_sample_lines(frame, STRIDE, Y_HEIGHT, LENS, rpy, lines, boxes)
    assert np.array_equal(got[0], want[0]) and np.array_equal(got[1], want[1]) and got[0].dtype == want[0].dtype


def test_the_reader_is_three_times_the_reference():
  """The comma four read the reference's 0.79 ms (on an M-series Mac) in 7.5-8 ms, over its 5 ms budget, and turned the
  paint off every drive (detector/modeld_detector.py PAINT_BUDGET_S). The fast reader is about 3x it on any machine (0.21
  against 0.68 ms, sample_lines alone, on that Mac): a ratio, so a slow CI host doesn't fail it, and 2x, so noise doesn't."""
  reads = list(random_reads(10, seed=2))

  def best_of(f, n=5):
    times = []
    for _ in range(n):
      t0 = time.perf_counter()
      for frame, rpy, lines, boxes in reads:
        f(frame, STRIDE, Y_HEIGHT, LENS, rpy, lines, boxes)
      times.append(time.perf_counter() - t0)
    return min(times)
  best_of(sample_lines, 1)
  assert best_of(reference_sample_lines) > 2. * best_of(sample_lines)


def test_python_calls_per_read():
  """On the comma four the read is Python call overhead more than arithmetic, so its calls per update (cProfile's count) are
  a stand-in for its time there that no host's speed moves: 1803 before the vectorizing, 661 after, at a full window."""
  import cProfile
  import pstats
  clf, frame, lines = LanePaintClassifier(LENS), render([double(YELLOW), dashed(WHITE), solid(WHITE), None], 0.)[0], lines_xyz()
  box = np.array([[600., 400., 700., 470., 0.9, 2.]])
  for k in range(20):  # a full WINDOW_S of frames
    clf.update(k * DT, frame, STRIDE, Y_HEIGHT, RPY, True, lines, [0.9] * 4, box)
  pr = cProfile.Profile()
  pr.enable()
  clf.update(20 * DT, frame, STRIDE, Y_HEIGHT, RPY, True, lines, [0.9] * 4, box)
  pr.disable()
  assert pstats.Stats(pr).total_calls <= 860  # 661 and 30%, for numpy's own wrappers moving between versions
