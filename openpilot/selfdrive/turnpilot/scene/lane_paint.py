"""Lane paint: whether each of modelV2's four lane lines is painted broken or solid, white or yellow, read off the road camera.

modelV2 says where the lines are, not what they are, and the lane planner needs to know what is beside the car before it
moves over toward a turn (lane_planner.py). US markings (MUTCD) carry the answer:
- broken white: the lane beyond goes the same way, and crossing is allowed
- solid white: a road edge, shoulder, bike or parking lane, or a lane change discouraged near a junction
- yellow, solid or broken: oncoming traffic beyond it, or the left edge of a divided road
- California's Botts' dots (raised dots in place of painted dashes) read unknown: a 10 cm dot falls between samples more
  often than not, so a group of them doesn't make a mark, and short marks are what parking T-marks look like

Each line is sampled every X_STEP_M from X_MIN_M to X_MAX_M ahead where modelV2 puts it: its own x, y and z in the
calibrated frame (z is the ground below the camera), projected through the road camera as the UI draws laneLines. At each
sample a short profile is read across the line, on the ground. Paint is a narrow ridge of luma above the road on both
sides of it, by a margin that scales with the road's brightness. A wide bright band (a crosswalk bar, sun on concrete) is
neither paint nor road, and neither is a sample without paint whose two sides differ by more than a shadow could leave
on one road surface (STEP_REL): a shadow edge along a line darkens the paint below the sunlit road, and if that counted
as road, a solid line under a hedge's shadow would read broken. Yellow on concrete can be darker than the concrete, so a
narrow dip in U (the NV12 UV plane) below the road on both sides is paint too, and yellow; so is luma paint with a
smaller dip in U (worn yellow). Either needs the paint's chromaticity, (U - 128) / (Y - 16) and (V - 128) / (Y - 16), off
the road's beside it (YELLOW_CU_MAX, YELLOW_CV_MIN): in low or warm sun white paint takes the light's tint and, being
bright, dips in U as far as worn yellow does, but a neutral surface keeps the light's chromaticity whatever its brightness.
A sample with the U dip but not the chromaticity votes neither colour, so a yellow line the light washes out (the sun
ahead) reads colour-unknown, never white. Luma paint with U near the road's and V not above it is white.

Per frame a line has paint at some share of its samples, in marks: painted runs with less than GAP_MIN_M of road between
them, so a worn dash makes one mark. Over WINDOW_S that share, the longest mark and the
gaps between marks decide the kind: a solid line covers nearly every sample; a US broken line (3 m dashes on a 12 m
cycle) covers a quarter to a half, with marks of DASH_MIN_M to DASH_MAX_M and a closed gap between them in most frames.
Shorter marks are not lane lines: parking stall T-marks (a 0.6-1.2 m bar every 6-7 m, a parking strip beyond) and
crosswalk or stop bars crossed in an intersection. The color is read whatever the kind, so a line too worn or hidden to
tell broken from solid can still read yellow. A line is unknown, never a guess, when its samples fall outside the frame
or under a detected vehicle, its probability is low (under BROKEN_PROB_MIN it never reads broken: crack sealant and seams
are what the model is unsure of), calibration is invalid, the frames are stale, or there aren't
enough samples. A line jumping sideways (the model renumbering them in a lane change) starts its window over.

On openpilot's demo route (lane_paint_video.py --score), no solid or yellow ego line read broken white on the 135 labeled
stills or the 36 tuning stills; 49% and 39% of broken white ones did, and the rest mostly read unknown. Over every 10 Hz
reading of segments 1, 3 and 6-15 (verify_run.py), the four stretches the previous tuning read broken white wrongly (a solid
edge line beside an exit gore, a ramp edge, crosswalk and stop bars inside an intersection, crack sealant with no lane line)
read it 0 times. The misses that matter: on concrete freeways a light slab joint runs along some dashed lines and fills
their gaps, so they read solid, which only costs a lane change; a solid line with most of its paint gone in random
stretches looks broken and can read broken white (synthesized in test_lane_paint.py); and one route is not every road. So
the lane planner also needs the model to see a lane beyond the line, and the driver confirms each change (lane_planner.py).

The chromaticity test (YELLOW_CU_MAX, YELLOW_CV_MIN) is from comma's public comma four routes with video, 2026-10-30: a
Model 3 in California (79d0f04c82980004/00000003), a car in a European city (49d0fc1df3e7320b/00000002), one in Australia
(5afee5161ea1dcde/00000003) and the demo route, 81 min moving, 41 of them in right-hand traffic. There the ego right line,
never yellow on those drives, read yellow at 0.5 or more on 6.1% of 20969 moving 10 Hz reads without it and on 0.4% with
it, and turnpilotd's wrong-side guard, replayed, fired 9 times without it (white bike-lane and edge lines in low sun read
yellow) and once with it. 2004 reads of lines labelled yellow read yellow 86% and 85% of the time, and none white. Of the
18371 line reads yellow at 0.5 or more without it, 6 read white with it, all in Australia, where every line is white. The
thresholds were chosen on the same footage, with no held-out set; over 1.5 s windows of it the ego right line read yellow
in 1955 of 14985 without the test, and in 43 to 113 with any of -0.05 to -0.08 and 0.02 to 0.04 (55 as set), the
labelled yellow in 93-98% (96%).
"""
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from openpilot.selfdrive.turnpilot.scene.ranging import Lens, calib_from_view

X_MIN_M, X_MAX_M, X_STEP_M = 8., 25., 0.25
XS = np.arange(X_MIN_M, X_MAX_M + 1e-6, X_STEP_M)
PROFILE_STEP_M = 0.05
PROFILE_HALF_M = 1.1                  # the profile across a line runs this far either side of it
OFFSETS = np.arange(-PROFILE_HALF_M, PROFILE_HALF_M + 1e-6, PROFILE_STEP_M)
SEARCH_HALF_M = 0.4                   # paint is looked for this far either side of where the model puts the line
FLANK_M = (0.3, 0.7)                  # the road level is the median luma this far either side of the brightest point
WIDTH_HALF_M = 0.25                   # a ridge above half its height over more than WIDTH_MAX_M of the +-WIDTH_HALF_M around
WIDTH_MAX_M = 0.4                     # its peak isn't a line: a crosswalk bar, a patch of sun
LUMA_MIN = 12.                        # paint is at least this much brighter than the road on both sides,
LUMA_REL = 0.1                        # and this share of the road's own luma
LUMA_SATURATED = 235.                 # a road this bright can't show paint
STEP_REL = 0.3                        # the two sides of a sample differing by more than this share of the brighter (and
                                      # LUMA_MIN) is a step, a shadow edge or the road's edge: unreadable unless painted
# Yellow on concrete is often no brighter than the concrete (the demo route's freeway: luma 100-110 on 120-135), so a dip in
# U below the road on both sides is paint too. U and V at the paint minus the road's:
YELLOW_DU = -8.                       # yellow at or below this (the freeway's yellow edge: -12 to -19, its V +9 to +12),
YELLOW_DV_MIN = -3.                   # and V not below this (a green bike lane is low U and low V)
WORN_YELLOW_DU = -4.                  # luma paint with U this far below the road is yellow too (a worn double yellow on the
                                      # demo route: -4 to -10; white paint there: -2 to 0)
WHITE_DU = -4.                        # luma paint with U at or above this at its peak,
WHITE_DV_MAX = 3.                     # and V not more than this above the road's (yellow's V is above it), is white
# Yellow is a colour, not a tint: under low or warm sun white paint takes the light's tint, and being bright its U falls
# below the grey road's by as much as worn yellow's (comma four, a Model 3 in California afternoon sun: white lines -4 to
# -7, read yellow on 57% of samples). A neutral surface keeps the light's chromaticity, (U - 128) / (Y - 16), whatever its
# brightness, so a yellow vote also needs the paint's chromaticity off the road's by these, at the U dip:
YELLOW_CU_MAX = -0.06                 # U ratio at or below the road's minus this (white in warm sun: median -0.025,
                                      # yellow centre lines -0.13; demo route freeway edge -0.17)
YELLOW_CV_MIN = 0.03                  # and V ratio at or above the road's plus this (white +0.01, yellow +0.10)
YELLOW_SHARE = 0.3                    # yellow takes this share of the painted samples (yellow is the cautious answer),
WHITE_SHARE, WHITE_MAX_YELLOW = 0.7, 0.1  # white this share, with no more than this share yellow
COLOR_MIN_PAINT = 20                  # painted samples in the window before a color is read
LINE_PROB_MIN = 0.5
BROKEN_PROB_MIN = 0.65                # a line the model is less sure of than this never reads broken (it reads unknown): crack
                                      # sealant on the demo route (s15, line prob 0.56-0.62) read broken white at 0.62-0.88
WINDOW_S = 1.5                        # paint is accumulated over this long
STALE_S = 0.5                         # no frame for this long starts every window over
JUMP_M = 1.0                          # a line moving this much sideways between frames is a different line
MIN_FRAMES = 4                        # in the window, before anything but unknown
MIN_VALID = 60                        # valid samples in the window
FULL_FRAMES = 8                       # confidence is full from this many frames on
SOLID_COV = 0.8                       # paint at this share of the samples or more is solid
BROKEN_COV = (0.1, 0.5)               # broken in between (a solid line in dappled shade on the demo route read 0.51-0.59,
                                      # intersection bars crossed 0.05-0.07),
GAP_MIN_M, GAP_MAX_M = 1., 12.        # with a gap this long between marks
GAP_SHARE, GAP_SHARE_FULL = 0.4, 0.6  # in this share of the frames (a 3 m dash on a 12 m cycle shows a closed gap in at most 2/3),
DASH_MIN_M, DASH_MAX_M = 1.5, 6.      # and the longest mark per frame this long (median over the window): parking T-marks
                                      # and intersection bars are shorter, the demo route's dashes 2.2-4.5 m
NONE_COV = 0.02                       # paint at no more than this share is no line

KINDS = ('unknown', 'none', 'broken', 'solid')
COLORS = ('unknown', 'white', 'yellow')

_C = len(OFFSETS) // 2
_SEARCH = np.arange(_C - round(SEARCH_HALF_M / PROFILE_STEP_M), _C + round(SEARCH_HALF_M / PROFILE_STEP_M) + 1)
_FLANK = np.arange(round(FLANK_M[0] / PROFILE_STEP_M), round(FLANK_M[1] / PROFILE_STEP_M) + 1)
_WIDTH = np.arange(-round(WIDTH_HALF_M / PROFILE_STEP_M), round(WIDTH_HALF_M / PROFILE_STEP_M) + 1)
PAINT, ROAD, INVALID = 1, 0, -1
NO_COLOR, WHITE, YELLOW = 0, 1, 2


@dataclass(frozen=True)
class LinePaint:
  kind: str = 'unknown'     # KINDS
  color: str = 'unknown'    # COLORS
  confidence: float = 0.
  coverage: float = 0.      # share of the window's valid samples with paint

  def is_(self, kind: str, color: str, min_confidence: float) -> bool:
    return self.kind == kind and self.color == color and self.confidence >= min_confidence


UNKNOWN = LinePaint()


@dataclass
class _Frame:
  t: float
  valid: int
  paint: int
  gap: bool              # a closed gap of GAP_MIN_M to GAP_MAX_M between painted runs
  long_run_m: float      # the longest painted run
  white: int             # painted samples read white,
  yellow: int            # and yellow


@dataclass
class _Line:
  frames: deque = field(default_factory=deque)
  y: float | None = None


def project(lens: Lens, view_from_calib: np.ndarray, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  """(u, v, in front) of calibrated-frame points (..., 3), x forward, y right, z down, through a pinhole lens."""
  ray = pts @ view_from_calib.T
  z = ray[..., 2]
  front = z > 0.5
  zs = np.where(front, z, 1.)
  cx, cy = lens.center
  return cx + lens.f * ray[..., 0] / zs, cy + lens.f * ray[..., 1] / zs, front


def _median_flanks(profile: np.ndarray, at: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """Median of the profile FLANK_M either side of index `at`, per sample: (left, right)."""
  return (np.median(np.take_along_axis(profile, at[..., None] - _FLANK, axis=2), axis=2),
          np.median(np.take_along_axis(profile, at[..., None] + _FLANK, axis=2), axis=2))


def _at(profile: np.ndarray, idx: np.ndarray) -> np.ndarray:
  return np.take_along_axis(profile, idx[..., None], axis=2)[..., 0]


def sample_lines(nv12: np.ndarray, stride: int, y_height: int, lens: Lens, rpy_calib, lines_xyz, boxes=None):
  """Per line and sample (len(XS)): its state (PAINT, ROAD, INVALID) and color vote (NO_COLOR, WHITE, YELLOW).
  nv12: camerad's padded NV12 frame, flat uint8. lines_xyz: per line (x, y, z) arrays as in modelV2.laneLines.
  boxes: (N, >=4) x0 y0 x1 y1 of detected objects in this frame; samples behind them are INVALID."""
  centers = np.empty((len(lines_xyz), len(XS), 3))
  for i, (x, y, z) in enumerate(lines_xyz):
    centers[i, :, 0] = XS
    centers[i, :, 1] = np.interp(XS, x, y)
    centers[i, :, 2] = np.interp(XS, x, z)
  pts = np.repeat(centers[:, :, None, :], len(OFFSETS), axis=2)
  pts[..., 1] += OFFSETS
  u, v, front = project(lens, calib_from_view(rpy_calib).T, pts)
  ui, vi = np.rint(u).astype(np.int64), np.rint(v).astype(np.int64)
  inside = front & (ui >= 0) & (ui < lens.width) & (vi >= 0) & (vi < lens.height)
  ok = inside.all(axis=2)
  ui, vi = np.where(inside, ui, 0), np.where(inside, vi, 0)
  luma = nv12[vi * stride + ui].astype(np.float32)
  uv = (y_height + vi // 2) * stride + (ui // 2) * 2
  chroma_u, chroma_v = nv12[uv].astype(np.float32), nv12[uv + 1].astype(np.float32)

  # luma: a narrow ridge above the road on both sides
  peak_idx = _SEARCH[np.argmax(luma[..., _SEARCH], axis=2)]
  road_l, road_r = _median_flanks(luma, peak_idx)
  road = np.maximum(road_l, road_r)
  step = np.abs(road_l - road_r) > np.maximum(LUMA_MIN, STEP_REL * road)
  ridge = _at(luma, peak_idx) - road
  above = np.take_along_axis(luma, peak_idx[..., None] + _WIDTH, axis=2) >= (road + ridge / 2)[..., None]
  bright = ridge >= np.maximum(LUMA_MIN, LUMA_REL * road)
  wide = bright & (above.sum(axis=2) * PROFILE_STEP_M > WIDTH_MAX_M)
  # chroma: a dip in U below the road on both sides, V not dipping with it; smaller with luma paint (worn yellow)
  dip_idx = _SEARCH[np.argmin(chroma_u[..., _SEARCH], axis=2)]
  u_road, v_road, y_road = (_median_flanks(c, dip_idx) for c in (chroma_u, chroma_v, luma))
  u_dip, v_dip = _at(chroma_u, dip_idx), _at(chroma_v, dip_idx)
  dip = u_dip - np.minimum(*u_road)
  dv = v_dip - 0.5 * np.add(*v_road)
  # chromaticity at the U dip, where the yellow vote is found, against the road's beside it (YELLOW_CU_MAX, YELLOW_CV_MIN)
  y_paint, y_flank = np.maximum(_at(luma, dip_idx) - 16, 1), np.maximum(0.5 * np.add(*y_road) - 16, 1)
  cu = (u_dip - 128) / y_paint - (0.5 * np.add(*u_road) - 128) / y_flank
  cv = (v_dip - 128) / y_paint - (0.5 * np.add(*v_road) - 128) / y_flank
  tint = ((dip <= YELLOW_DU) | (bright & ~wide & (dip <= WORN_YELLOW_DU))) & (dv >= YELLOW_DV_MIN)  # the paint's or the light's
  yellow = tint & (cu <= YELLOW_CU_MAX) & (cv >= YELLOW_CV_MIN)
  du_peak = _at(chroma_u, peak_idx) - 0.5 * np.add(*_median_flanks(chroma_u, peak_idx))
  dv_peak = _at(chroma_v, peak_idx) - 0.5 * np.add(*_median_flanks(chroma_v, peak_idx))
  white = bright & ~wide & ~tint & (du_peak >= WHITE_DU) & (dv_peak <= WHITE_DV_MAX)  # a tint without yellow's chromaticity: neither

  ok &= road < LUMA_SATURATED
  if boxes is not None and len(boxes):
    b = np.asarray(boxes, dtype=np.float32)[:, :4]
    row = vi[..., _C][..., None]
    u0, u1 = ui[..., 0][..., None], ui[..., -1][..., None]
    behind = (row >= b[:, 1]) & (row <= b[:, 3]) & (np.maximum(u0, u1) >= b[:, 0]) & (np.minimum(u0, u1) <= b[:, 2])
    ok &= ~behind.any(axis=-1)

  painted = (bright & ~wide) | yellow
  state = np.where(~ok | (wide & ~yellow) | (step & ~painted), INVALID, np.where(painted, PAINT, ROAD))
  color = np.where(state != PAINT, NO_COLOR, np.where(yellow, YELLOW, np.where(white, WHITE, NO_COLOR)))
  return state, color


def _runs(state: np.ndarray):
  starts = np.r_[0, np.flatnonzero(np.diff(state)) + 1]
  vals = state[starts]
  closed = np.zeros(len(vals), bool)  # a ROAD run with PAINT on both sides
  closed[1:-1] = (vals[1:-1] == ROAD) & (vals[:-2] == PAINT) & (vals[2:] == PAINT)
  return starts, np.diff(np.r_[starts, len(state)]) * X_STEP_M, vals, closed


def runs(state: np.ndarray) -> tuple[float, bool]:
  """(longest mark in m, whether a closed gap of GAP_MIN_M..GAP_MAX_M lies between marks) for one line's samples. A mark is
  painted runs with road gaps shorter than GAP_MIN_M between them. An INVALID sample ends a mark, and a gap with one in it
  isn't closed."""
  starts, lengths, vals, closed = _runs(state)
  short = closed & (lengths < GAP_MIN_M)
  if short.any():
    state = state.copy()
    for s0, n in zip(starts[short], np.rint(lengths[short] / X_STEP_M).astype(int), strict=True):
      state[s0:s0 + n] = PAINT
    starts, lengths, vals, closed = _runs(state)
  painted = lengths[vals == PAINT]
  gap = bool((closed & (lengths >= GAP_MIN_M) & (lengths <= GAP_MAX_M)).any())
  return float(painted.max()) if len(painted) else 0., gap


def _color(frames) -> tuple[str, float]:
  """(color, its share) from the window's votes: yellow on YELLOW_SHARE of the painted samples (the cautious answer), white on
  WHITE_SHARE with hardly any yellow, otherwise unknown."""
  painted = sum(f.paint for f in frames)
  if painted < COLOR_MIN_PAINT:
    return 'unknown', 0.
  yellow, white = sum(f.yellow for f in frames) / painted, sum(f.white for f in frames) / painted
  if yellow >= YELLOW_SHARE:
    return 'yellow', min(1., yellow / (2 * YELLOW_SHARE))
  if white >= WHITE_SHARE and yellow <= WHITE_MAX_YELLOW:
    return 'white', white
  return 'unknown', 0.


def classify(frames) -> LinePaint:
  """One line's paint from its window of frames. The color is read whatever the kind: kind unknown with a color has the
  color's confidence."""
  if len(frames) < MIN_FRAMES:
    return UNKNOWN
  valid = sum(f.valid for f in frames)
  if valid < MIN_VALID:
    return UNKNOWN
  coverage = sum(f.paint for f in frames) / valid
  evidence = min(1., len(frames) / FULL_FRAMES)
  seen = [f for f in frames if f.valid >= len(XS) // 2]
  color, share = _color(frames)
  if coverage >= SOLID_COV:
    kind, margin = 'solid', min(1., (coverage - SOLID_COV) / (1. - SOLID_COV) + 0.5)
  elif BROKEN_COV[0] <= coverage <= BROKEN_COV[1] and seen and (gap_share := np.mean([f.gap for f in seen])) >= GAP_SHARE and \
       DASH_MIN_M <= np.median([f.long_run_m for f in seen]) <= DASH_MAX_M:
    kind, margin = 'broken', min(1., gap_share / GAP_SHARE_FULL)
  elif coverage <= NONE_COV:
    return LinePaint('none', 'unknown', evidence, coverage)
  else:
    kind, margin = 'unknown', 1.
  if color == 'unknown':
    return LinePaint(kind, color, float(evidence * margin) if kind != 'unknown' else 0., coverage)
  return LinePaint(kind, color, float(evidence * margin * share), coverage)


class LanePaintClassifier:
  """Accumulates sample_lines() over time into one LinePaint per modelV2 lane line."""
  def __init__(self, lens: Lens):
    assert not lens.fisheye, "the road camera is a pinhole"
    self.lens = lens
    self.lines = [_Line() for _ in range(4)]
    self.last_t: float | None = None

  def reset(self) -> None:
    self.lines = [_Line() for _ in range(4)]
    self.last_t = None

  def update(self, t: float, nv12: np.ndarray, stride: int, y_height: int, rpy_calib, calibrated: bool, lines_xyz, line_probs,
             boxes=None) -> list[LinePaint]:
    """t: s, the frame's time. lines_xyz: modelV2.laneLines as (x, y, z) per line, of this very frame."""
    if self.last_t is not None and not (0. <= t - self.last_t <= STALE_S):
      self.reset()
    self.last_t = t
    if not calibrated or len(lines_xyz) != 4 or len(line_probs) != 4:
      self.reset()
      return [UNKNOWN] * 4
    state, color = sample_lines(nv12, stride, y_height, self.lens, rpy_calib, lines_xyz, boxes)
    out = []
    for i, line in enumerate(self.lines):
      y = float(np.interp(X_MIN_M, lines_xyz[i][0], lines_xyz[i][1]))
      if line.y is not None and abs(y - line.y) > JUMP_M:
        line.frames.clear()
      line.y = y
      while line.frames and t - line.frames[0].t > WINDOW_S:
        line.frames.popleft()
      if line_probs[i] >= LINE_PROB_MIN:
        s = state[i]
        long_run, gap = runs(s)
        line.frames.append(_Frame(t, int((s != INVALID).sum()), int((s == PAINT).sum()), gap, long_run,
                                  int((color[i] == WHITE).sum()), int((color[i] == YELLOW).sum())))
      p = classify(line.frames) if line_probs[i] >= LINE_PROB_MIN else UNKNOWN
      out.append(LinePaint('unknown', p.color, p.confidence, p.coverage) if p.kind == 'broken' and line_probs[i] < BROKEN_PROB_MIN else p)
    return out


def fill_lane_paint(builder_list, paint: list[LinePaint]) -> None:
  """Fill a List(TurnpilotDetections.LanePaint) builder, initialized to len(paint)."""
  for b, p in zip(builder_list, paint, strict=True):
    b.kind, b.color, b.confidence, b.coverage = p.kind, p.color, float(p.confidence), float(p.coverage)


def lane_paint_from_msg(lane_paint) -> tuple[LinePaint, ...]:
  """The four lines' paint from a turnpilotDetections.lanePaint reader; unknown for a log without it."""
  if len(lane_paint) != 4:
    return (UNKNOWN,) * 4
  return tuple(LinePaint(str(p.kind), str(p.color), float(p.confidence), float(p.coverage)) for p in lane_paint)
