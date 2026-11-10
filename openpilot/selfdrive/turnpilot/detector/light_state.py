"""Traffic light state from the detector's class-9 boxes, read on the CPU copy of the road camera's full-resolution frame
(DESIGN phase 4 G2). What it reads is published and logged; under TurnPilotCameraLights addOnly (the default since
2026-11-06, LIGHT_MODE_DEFAULT) a governing red adds a stop at its signal (map_controls), under shadow nothing acts on it.

A box is gated before any colour is read, so that tail and brake lights, red signs and glare low in the frame never
become a red (CRITIQUE REQUIRED 8):
- small: its long side under HEAD_MIN_H_PX full-frame px. Below that a lamp is under about 8 px and a ball can't be told from
  an arrow, so it is never read (a 1.07 m head at about 50 m on a comma four).
- belowHorizon: its bottom edge is less than MIN_ELEVATION_DEG above the calibrated horizon. A signal head hangs 4-6 m
  over the road, above the camera; a car's lights are below it.
Gated boxes are counted, never published (modeld logs the counts once a minute: LightReader.take_counts). A box past the
gates is a head, and its state is read (its reason the published Light.reason: read or one of these):
- aspect: not a vertical head (height/width outside MIN_ASPECT-MAX_ASPECT): horizontal, a doghouse, a pedestrian signal.
  Unknown.
- A lamp pixel is bright (HSV value) and either saturated in a lamp hue or blown out white. dark: no third of the box
  holds LIT_FRAC_MIN of them. Unknown.
- The lit lamp is a run of lit rows (lamp_runs: rows holding ROW_REL of the fullest row's lamp pixels, gaps of GAP_REL
  closed), the run with the most; its third is the one its core's centre (its pixels at CORE_REL of its brightest) lies in,
  its colour and shape read on its rows grown through the rows next to them that hold any lamp pixel. twoLit: a second,
  separate run with SECOND_LIT_REL of its pixels (and LIT_FRAC_MIN of a third's). streak: the run's core (first to last
  row) is taller than STREAK_MAX of the run's widest row, or the run than RUN_MAX_REL of the box: a lit pole, a sign filling
  the box, glare, never a lamp (a lamp's glow trailing below its face is the lamp's: STREAK_MAX). Unknown either way. A
  glow that fills its box stays a streak however round: read as a lamp it made a governing red of something red and round
  on 210 of 561 frames of route 29's approach to a green (its video through the detector and this reader), and added no red
  read at any of the drive's stands at a red.
  Until 2026-11-07 the lit third was the third holding the most lamp pixels and any other third with half its share was
  twoLit, and a lamp's glow made it two. On the 11-06 night drives 667 head reads gave no green and no
  colour on any head governing the car's lane (heads 36-86 px tall: not small; 32 reds, all on side heads 15-30 deg off
  the lane, off the lamps' beam and so dimmer, it seems: there are no frames). The one governing head a frame shows (1d
  4:05, standing 31 m from it, box 24x46 px, no colour in 84 reads over 48 s; the qlog thumbnail, a quarter the size):
  the red lamp's face blown pink-white (S 0.15-0.29, no hue) with a saturated red glow (V 150-200, S 0.5-0.9) spreading
  below it under the visor, so the lamp straddles the top and middle thirds and the old read said twoLit (0.43 and 0.47
  of the thirds lit, the thumbnail upscaled); its core's centre is at 0.28 of the box: red. Read through its glow (the
  run), one lamp is one lamp. On the comma demo route's 878 daylight head reads (2026-11-07 replay) it
  changed no colour read and read 45 more, every one the colour its crop shows (green and yellow balls the sky had made
  arrows or two lamps, greens on a third's edge); voted (the lane taken straight), 36 more road runs had a governing green
  and 1 a yellow, the reds the same 371.
- noColor: the lit lamp's saturated pixels don't agree on red, yellow or green. Unknown. HUES green ends at 195 deg: on the
  demo route the green lamps' pixels lie at 166-181 (1-99%) and the sky's at 195-216, which up to 210 counted as a green
  lamp (a box over a head with sky in its bottom third read green; sky beside a green ball called it an arrow). The night
  green lamp's glow (1d 5:05 thumbnail) is 159-168.
- positionMismatch: a yellow whose lit lamp isn't in the box's middle third (YELLOW_SPAN). Unknown. Only a yellow: its hue
  is a streetlight's and an amber sign's too. A red's and a green's colour is the hue's alone, wherever the lamp sits: at
  night the box is drawn round the lamp's glow, the housing as dark as the sky, and the lamp sits anywhere in it.
- arrow: the lit lamp's pixels aren't mirror-symmetric left to right about their own centre, as a ball is and a turn
  arrow is not. A green arrow is unknown: it is never a permission. A yellow arrow is unknown too: a flashing yellow
  arrow is a permissive left, and blocking on it would stall the autonomous left in the phase it is built for. A red
  arrow keeps its colour (reason arrow): it never permits a go, so the autonomous left's go (map_controls.turn_light)
  and every lift (release_light) at its junction see it as a red; a green ball beside a red left arrow is not a left's
  green. Under add-only (the default) a camera red adds a stop at its signal, so a red arrow alone over the car's lane
  stops it there too, as a red: a stop and a loud handback, never a run. This is a proxy at these
  sizes: on the comma demo route it caught 24-78% of two red left arrows' reads, and called 10 reads of a red ball, 30
  of a yellow ball and 25 of a green ball arrows, so a red arrow must never turn unknown on it alone (the sky in the
  lamp's third made most of the green and yellow balls' arrows: since 2026-11-07 15 of the green and 2 of the yellow
  reads there read their colour).
The reader reads boxes down to decoder.LIGHT_READ_CONF, under the detector's MIN_CONF (route 29, 2026-11-08: a red read only
at its line, its heads first boxed at MIN_CONF 40 m out and then not for 1.7 s; the full pass had them at 0.2-0.33 from about
60 m), and a head from a box under MIN_CONF is published only once it reads a colour or holds a vote.
A head's state is its vote: the colour most of its last VOTE_M reads read, once VOTE_N did (a frame it wasn't seen in
counts as unknown), which also rides out LED flicker. A head governs the car's lane when its bearing lies within GOVERN_BEARING_DEG of the lane's (the
driving model's path at the head's distance), and the governing state is what all governing heads with a state agree
on, else unknown.

Distance is to the head, from its elevation and an assumed height: the stop line is not known from the camera.

What tunes it next (2026-11-07): each read carries a detail (HeadRead.detail: the thirds' lamp shares, the lit run, its
core's centre and rows, its pixels and widest row, its peak value, the saturated pixels per colour), published with the head
(LightHead.detail), and every SAMPLE_S the reader logs a sample (turnpilot.light_sample, debug level, so the rlog alone):
up to SAMPLE_HEADS heads, the governing first, each with its detail and its box's NV12 crop at full resolution (rows over
SAMPLE_MAX_PX subsampled). The 11-06 drives left no frames: the next night's rlog has the lamps themselves.
"""
import base64
import math
from collections import Counter, deque
from dataclasses import dataclass, field

import numpy as np

from openpilot.common.params import UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.detector.decoder import MIN_CONF
from openpilot.selfdrive.turnpilot.scene.ranging import Lens, calib_from_view

HEAD_MIN_H_PX = 24          # full-frame px
MIN_ELEVATION_DEG = 1.0     # the box's bottom edge above the calibrated horizon
# a vertical 3-lamp head's height over width. 1.8 until 2026-11-08: at night the box is drawn round the lit lamp's glow, the
# housing as dark as the sky, and is wider than the head (route 29: the governing red the car waited 25 s at boxed 1.65-1.77
# on every run, a governing green 1.43, a red at 99 m 1.53); a doghouse, a pedestrian signal and a horizontal head stay under
# 1.4 (about 1.0-1.3, and under 1)
MIN_ASPECT, MAX_ASPECT = 1.4, 4.5
INNER_X = 0.15              # each side of the box left out of the lamp counts (pole, backplate edge, sky)
LIT_V = 150                 # HSV value (0-255) of a lamp pixel...
LIT_S = 0.35                # ...with this saturation in a lamp hue...
BLOWN_V = 235               # ...or blown out this bright
LIT_FRAC_MIN = 0.08         # share of a third's pixels that are lamp pixels for it to be lit
SECOND_LIT_REL = 0.5        # another run with this share of the lit one's pixels (and LIT_FRAC_MIN of a third's) is a second lamp
ROW_REL = 0.25              # a row is lit with this share of the fullest row's lamp pixels
GAP_REL = 1 / 24            # lit rows this close (a share of the box's height, at least a row) are one lamp's
CORE_REL = 0.9              # a lamp's core: its pixels this bright of its brightest (a blown face at night, the lens by day)
# the lit run's core's rows over the run's widest row's lamp pixels (across the whole box), and the whole run's rows over the
# box's height: past either is a streak, never a lamp. Demo route daylight colour reads: at most 1.64 and 0.35; the 1d 4:05
# night red with its glow 0.35 (its whole run 1.1-1.3) and 0.39-0.43; a pole lit by a streetlight in a class-9 box (1e 7:05,
# read twoLit before) 4.2 (its core 25 of its 29 rows) and 0.66; the demo route's yellow warning sign filling its box 0.93 of
# its height. The core, not the whole run, since the 2026-11-07 review: a small or far lamp's glow trailing below its face
# made the whole run a streak, and of 1500 synthetic night lamps, 14 reds, 17 yellows and 3 greens that the read before
# 2026-11-07 read went unknown on the whole run (0, 2 and 0 on the core). Its cost: a strip lit brighter at one end, a pole
# under a streetlight, has a core shorter than its run, and of 980 synthetic lit strips 430 read yellow (240 on the whole
# run, 392 on the read before 2026-11-07). The real pole and red above read the same either way; the night samples (below)
# are the real test of both
STREAK_MAX = 2.5
RUN_MAX_REL = 0.6
MIN_COLOR_PX = 3            # saturated pixels in the lit lamp to read a colour from
COLOR_SHARE = 0.6           # of them, this share in one hue range
# green to 195, not 210: the sky is 195-216 and green lamps 166-181 (module docstring, noColor)
HUES = {'red': ((330., 360.), (0., 18.)), 'yellow': ((18., 65.),), 'green': ((120., 195.),)}
# where a yellow's lit lamp may sit: its core's centre, a share of the box's height down from its top, the middle third. Only a
# yellow's: its hue is a streetlight's, an amber sign's and a pedestrian hand's too, and the head's middle lamp is what tells it.
# A red's and a green's colour is the hue's alone, wherever the lamp sits in its box: at night the box is drawn round the lamp's
# glow and whatever is lit near it, never the housing, and the lamp sits anywhere in it (route 29's crops: a governing red ball
# 23 m out at the bottom of its box, a governing green ball 35 m out at 0.30 of its, the greens the car waited at 0.55-0.59)
YELLOW_SPAN = (1 / 3, 2 / 3)
ARROW_MIN_PX = 10           # lamp pixels needed to judge the lamp's shape; fewer is unknown (tooFewLampPx)
SYMMETRY_MIN = 0.6          # the lamp's mask against its mirror image about its own centre (IoU): a ball, not an arrow
ASSUMED_HEAD_M = 5.5        # m, a head's centre over the road (MUTCD clearance 15-19 ft to its bottom)
MAX_RANGE = 150.            # m, the farthest distance it says
# a head's state is the colour most of its last VOTE_M reads read, once VOTE_N of them did (a tie is unknown). Two, not four: at
# night a head is boxed in some runs only and forgotten after HEAD_DROP_S unseen, and four reads in five runs came 2.3 s after
# the first red read of route 29's late red (16 m from its line then, at it by the vote), 0.8 s after the other's
VOTE_M, VOTE_N = 5, 2
MATCH_HEIGHTS = 1.5         # a read is the same head when its box centre is within this many box heights
HEAD_DROP_S = 0.6           # a head not seen this long is forgotten
# a head within this of the lane's bearing governs it. Kept at 10 after the 11-06 night, though every red it read was a side
# head 15-30 deg off: at 1e 4:45-4:48 three right-side heads 16-25 deg off read red, 4-5 votes, as the car went through at
# 10.5 m/s, its driver not braking (not the car's lights), so a wider gate would have braked it hard on a through road; at
# 1e 11:04-11:06 a head 17 deg off read red until the car went and nothing after, likely the car's own. Wider needs frames
# to tell the two apart
GOVERN_BEARING_DEG = 10.
MAX_READ_HEADS = 8          # heads read a frame, the most confident first (decode_lights' order): the read's time stays bounded
SAMPLE_S = 2.0              # s between a reader's logged samples (turnpilot.light_sample, module docstring)
SAMPLE_HEADS = 2            # heads a sample carries at most: the governing first, then the unread, then the largest
SAMPLE_MAX_PX = 96          # a sample's crop rows at most: a taller crop is subsampled by 2, 4 or 8
STATES = ('red', 'yellow', 'green')
LIGHT_MODES = ('off', 'shadow', 'addOnly')  # TurnPilotCameraLights
# a camera red adds a stop, never removes one (map_controls: add-only). Shadow until 2026-11-06, when the real car rolled a right
# on red at a junction the map has no signal for, the reader reading the red and nothing acting on it (Kellen: a red light is
# a hard rail; never ship a feature in shadow)
LIGHT_MODE_DEFAULT = 'addOnly'


def camera_lights_mode(params) -> str:
  """TurnPilotCameraLights, read by modeld and turnpilotd: off, shadow or addOnly (the default, and anything unknown)."""
  try:
    value = params.get("TurnPilotCameraLights")
  except UnknownKeyName:
    cloudlog.error("turnpilot: TurnPilotCameraLights unknown to this build's params, camera lights add-only; run scons")
    return LIGHT_MODE_DEFAULT
  value = value.decode() if isinstance(value, bytes) else value
  return value if value in LIGHT_MODES else LIGHT_MODE_DEFAULT


@dataclass
class HeadRead:
  state: str                  # 'red', 'yellow', 'green' or 'unknown'
  reason: str                 # 'read', or why not (module docstring)
  gated: bool = False         # stopped at a gate: not a head, never published
  bearing: float = 0.         # deg left of the calibrated forward axis
  distance: float = MAX_RANGE  # m to the head
  # what the read saw, for the logs (module docstring): h box rows, fr the thirds' lamp shares, run the lit run's rows, lamp
  # them grown, c its core's centre over the box's height, cr its core's rows, n its lamp pixels, wd its widest row's, runs
  # how many, pk its peak value, hue its saturated pixels per colour; empty for a gated box or a bad aspect
  detail: str = ''


def lamp_pixels(nv12: np.ndarray, stride: int, y_height: int, x0: int, y0: int, x1: int, y1: int):
  """(lamp mask, saturated hue or nan, HSV value 0-255) per pixel of the box, from NV12 (BT.601 full range)."""
  rows = np.arange(y0, y1)
  cols = np.arange(x0, x1)
  y = nv12.reshape(-1, stride)[rows[:, None], cols[None, :]].astype(np.float32)
  uv = nv12.reshape(-1, stride)[y_height + rows[:, None] // 2, (cols[None, :] // 2) * 2 + np.array([[0], [1]])[:, :, None]]
  u, v = uv[0].astype(np.float32) - 128., uv[1].astype(np.float32) - 128.
  r, g, b = y + 1.402 * v, y - 0.344136 * u - 0.714136 * v, y + 1.772 * u
  rgb = np.clip(np.stack([r, g, b], axis=-1), 0., 255.)
  mx, mn = rgb.max(axis=-1), rgb.min(axis=-1)
  s = np.where(mx > 0, (mx - mn) / np.maximum(mx, 1e-6), 0.)
  hue = np.full(mx.shape, np.nan)
  d = np.maximum(mx - mn, 1e-6)
  rr, gg, bb = rgb[..., 0], rgb[..., 1], rgb[..., 2]
  h = np.where(mx == rr, ((gg - bb) / d) % 6., np.where(mx == gg, (bb - rr) / d + 2., (rr - gg) / d + 4.)) * 60.
  colored = (s >= LIT_S) & (mx >= LIT_V)
  hue[colored] = h[colored]
  in_lamp_hue = np.zeros(mx.shape, bool)
  for ranges in HUES.values():
    for lo, hi in ranges:
      in_lamp_hue |= (h >= lo) & (h < hi)
  lamp = (colored & in_lamp_hue) | (mx >= BLOWN_V)
  return lamp, hue, mx


def lamp_runs(lamp: np.ndarray, value: np.ndarray) -> list[tuple[int, int, int, float, int]]:
  """The box's lit lamps, top to bottom (module docstring): (first row, last row, lamp pixels, centre row, core rows) of
  each run of lit rows, its centre the mean row of its core and its core rows the core's first to last row. lamp, value:
  the box's inner columns (lamp_pixels). A lamp's glow joins its run, so a lamp is one run wherever the thirds' edges fall;
  two lit lamps are two runs, the dark housing between them."""
  prof = lamp.sum(axis=1)
  lit = np.nonzero(prof >= max(1., ROW_REL * float(prof.max())))[0]
  if not len(lit):
    return []
  gap = max(1, int(round(lamp.shape[0] * GAP_REL)))
  bounds, start = [], int(lit[0])
  for a, b in zip(lit[:-1], lit[1:], strict=True):
    if b - a - 1 > gap:
      bounds.append((start, int(a)))
      start = int(b)
  bounds.append((start, int(lit[-1])))
  out = []
  for r0, r1 in bounds:
    v = np.where(lamp[r0:r1 + 1], value[r0:r1 + 1], 0.)
    # its brightest: the 90th percentile of its lamp pixels, not one pixel (a blown glint on a visor by day would be the core)
    core = (v >= CORE_REL * float(np.percentile(v[v > 0], 90))).sum(axis=1)
    centre = float((np.arange(r0, r1 + 1) + 0.5) @ core / max(int(core.sum()), 1))
    cr = np.nonzero(core)[0]
    out.append((r0, r1, int(prof[r0:r1 + 1].sum()), centre, int(cr[-1] - cr[0] + 1) if len(cr) else r1 - r0 + 1))
  return out


def hue_counts(hues: np.ndarray) -> dict[str, int]:
  """The saturated pixels (finite hues) in each lamp colour's hue range."""
  hues = hues[np.isfinite(hues)]
  return {name: sum(int(((hues >= lo) & (hues < hi)).sum()) for lo, hi in ranges) for name, ranges in HUES.items()}


def hue_color(hues: np.ndarray) -> str | None:
  n = int(np.isfinite(hues).sum())
  if n < MIN_COLOR_PX:
    return None
  for name, k in hue_counts(hues).items():
    if k >= COLOR_SHARE * n:
      return name
  return None


def symmetric(mask: np.ndarray) -> bool:
  """The mask against its own mirror image left to right, about its centroid column."""
  ys, xs = np.nonzero(mask)
  cx = xs.mean()
  mirrored = np.rint(2 * cx - xs).astype(int)
  ok = (mirrored >= 0) & (mirrored < mask.shape[1])
  mirror = np.zeros_like(mask)
  mirror[ys[ok], mirrored[ok]] = True
  inter = (mask & mirror).sum()
  union = (mask | mirror).sum()
  return union > 0 and inter / union >= SYMMETRY_MIN


def read_head(nv12: np.ndarray, stride: int, y_height: int, lens: Lens, rpy_calib, cam_height: float, box) -> HeadRead:
  """One class-9 box's read (module docstring). box: a decode() row, x0 y0 x1 y1 in full-frame pixels."""
  x0, y0, x1, y1 = (float(b) for b in box[:4])
  rot = calib_from_view(rpy_calib)
  def angles(u, v):
    ray = rot @ lens.ray(u, v)  # calib frame: x forward, y right, z down
    return math.degrees(math.atan2(-ray[2], math.hypot(ray[0], ray[1]))), math.degrees(math.atan2(-ray[1], ray[0]))
  bottom_elev, bearing = angles((x0 + x1) / 2, y1)
  centre_elev, _ = angles((x0 + x1) / 2, (y0 + y1) / 2)
  rise = max(ASSUMED_HEAD_M - cam_height, 0.5)
  distance = min(rise / math.tan(math.radians(centre_elev)), MAX_RANGE) if centre_elev > 0. else MAX_RANGE
  out = HeadRead('unknown', 'read', bearing=bearing, distance=distance)
  bw, bh = x1 - x0, y1 - y0
  if max(bh, bw) < HEAD_MIN_H_PX:
    out.reason, out.gated = 'small', True
    return out
  if bottom_elev < MIN_ELEVATION_DEG:
    out.reason, out.gated = 'belowHorizon', True
    return out
  if not (MIN_ASPECT <= bh / max(bw, 1e-6) <= MAX_ASPECT):
    out.reason = 'aspect'
    return out
  ix0, iy0 = int(math.floor(x0)), int(math.floor(y0))
  ix1, iy1 = min(int(math.ceil(x1)), lens.width), min(int(math.ceil(y1)), lens.height)
  lamp, hue, value = lamp_pixels(nv12, stride, y_height, ix0, iy0, ix1, iy1)
  h, w = lamp.shape
  edges = np.rint(np.linspace(0, h, 4)).astype(int)
  inner = slice(int(round(w * INNER_X)), max(int(round(w * (1 - INNER_X))), int(round(w * INNER_X)) + 1))
  fracs = np.array([lamp[edges[k]:edges[k + 1], inner].mean() if edges[k + 1] > edges[k] else 0. for k in range(3)])
  out.detail = f"h{h} fr{fracs[0]:.2f}/{fracs[1]:.2f}/{fracs[2]:.2f}"
  if fracs.max() < LIT_FRAC_MIN:
    out.reason = 'dark'
    return out
  runs = lamp_runs(lamp[:, inner], value[:, inner])
  r0, r1, n, centre, core_rows = max(runs, key=lambda r: r[2])
  # the lamp's colour and shape are read on its run grown through the rows next to it that hold any lamp pixel, never into
  # another run: an arrow's thin head and its glow's thin edge are under ROW_REL, and are the lamp's
  prof = lamp[:, inner].sum(axis=1)
  e0, e1 = r0, r1
  lo, hi = max((r[1] + 1 for r in runs if r[1] < r0), default=0), min((r[0] - 1 for r in runs if r[0] > r1), default=h - 1)
  while e0 > lo and prof[e0 - 1] > 0:
    e0 -= 1
  while e1 < hi and prof[e1 + 1] > 0:
    e1 += 1
  rows = slice(e0, e1 + 1)
  peak = int(value[rows, inner][lamp[rows, inner]].max())
  widest = int(lamp[r0:r1 + 1].sum(axis=1).max())  # across the whole box: a narrow box's inner columns clip a glow
  out.detail += f" run{r0}-{r1} lamp{e0}-{e1} c{centre / h:.2f} cr{core_rows} n{n} wd{widest} runs{len(runs)} pk{peak}"
  third_px = h / 3 * lamp[:, inner].shape[1]
  if any(r[2] >= max(LIT_FRAC_MIN * third_px, SECOND_LIT_REL * n) for r in runs if r[0] != r0):
    out.reason = 'twoLit'
    return out
  # the whole run against the box (a glow's tail is the lamp's, and never past RUN_MAX_REL of a head), its core against its
  # widest row (a lamp's face is round however far its glow trails; a lit pole is its core end to end): STREAK_MAX, above
  if r1 - r0 + 1 > RUN_MAX_REL * h or core_rows > STREAK_MAX * widest:
    out.reason = 'streak'
    return out
  out.detail += " hue " + " ".join(f"{k[0]}{v}" for k, v in hue_counts(hue[rows, inner]).items())
  color = hue_color(hue[rows, inner])
  if color is None:
    out.reason = 'noColor'
    return out
  if color == 'yellow' and not YELLOW_SPAN[0] <= centre / h <= YELLOW_SPAN[1]:
    out.reason = 'positionMismatch'
    return out
  mask = lamp[rows]
  if mask.sum() < ARROW_MIN_PX:
    out.reason = 'tooFewLampPx'
    return out
  if not symmetric(mask):
    out.reason = 'arrow'
    if color != 'red':
      return out  # a green arrow is never a permission; a yellow one may be a flashing yellow arrow's permissive left
  out.state = color  # a red arrow keeps its colour (module docstring)
  return out


@dataclass
class Head:
  id: int
  box: tuple                 # normalized x0 y0 x1 y1
  t: float
  reads: deque = field(default_factory=lambda: deque(maxlen=VOTE_M))

  @property
  def state(self) -> str:
    counts = Counter(self.reads)
    first, second = sorted((counts[s] for s in STATES), reverse=True)[:2]
    return max(STATES, key=lambda s: counts[s]) if first >= VOTE_N and first > second else 'unknown'

  @property
  def votes(self) -> int:
    counts = Counter(self.reads)
    return max((counts[s] for s in STATES), default=0)


def centre(box) -> tuple[float, float]:
  return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2


class HeadVoter:
  """Heads across frames by box centre, each with its last VOTE_M reads."""
  def __init__(self):
    self.heads: list[Head] = []
    self.next_id = 0

  def update(self, t: float, reads: list) -> list[Head]:
    """reads: (normalized box, state) of this frame's heads. Returns the heads seen this frame, in reads' order."""
    for hd in self.heads:
      hd.reads.append('unknown')  # replaced below when it is seen
    pairs = []
    for i, (box, _) in enumerate(reads):
      cx, cy = centre(box)
      reach = MATCH_HEIGHTS * max(box[3] - box[1], 1e-3)
      for hd in self.heads:
        hx, hy = centre(hd.box)
        dist = math.hypot(cx - hx, cy - hy)
        if dist <= reach:
          pairs.append((dist, i, hd))
    taken_reads, taken_heads, out = set(), set(), {}
    for _, i, hd in sorted(pairs, key=lambda p: p[0]):
      if i in taken_reads or hd.id in taken_heads:
        continue
      taken_reads.add(i)
      taken_heads.add(hd.id)
      out[i] = hd
    for i, (box, state) in enumerate(reads):
      hd = out.get(i)
      if hd is None:
        hd = Head(self.next_id, box, t)
        self.next_id += 1
        self.heads.append(hd)
        out[i] = hd
      else:
        hd.reads.pop()  # this frame's placeholder
      hd.box, hd.t = tuple(box), t
      hd.reads.append(state)
    self.heads = [hd for hd in self.heads if t - hd.t <= HEAD_DROP_S]
    return [out[i] for i in range(len(reads))]


@dataclass
class LightHead:
  """A published head: its vote and this frame's read."""
  id: int
  box: tuple          # normalized x0 y0 x1 y1
  conf: float
  state: str          # the vote
  votes: int
  read: str           # this frame's state
  reason: str         # this frame's reason
  bearing: float
  distance: float
  governs: bool
  detail: str = ''    # this frame's read's (HeadRead.detail)


def nv12_crop(nv12: np.ndarray, stride: int, y_height: int, width: int, height: int, box) -> dict:
  """A box's NV12 for a sample (module docstring): the box with a sixth of its height around it, on even rows and columns
  (the chroma's 2x2), its rows subsampled by step (1, 2, 4 or 8) past SAMPLE_MAX_PX. x y w h: the crop in full-frame px. y:
  base64 of the luma, ceil(h/step) rows of ceil(w/step); uv: of the chroma, ceil(h/2/uvStep) rows of ceil(w/2/uvStep) u,v
  pairs (BT.601 full range, as lamp_pixels reads it)."""
  x0, y0, x1, y1 = (float(b) for b in box[:4])
  m = max(2, int(round((y1 - y0) / 6)))
  cx0, cy0 = max(0, (int(x0) - m) // 2 * 2), max(0, (int(y0) - m) // 2 * 2)
  cx1, cy1 = min(width, (int(math.ceil(x1)) + m + 1) // 2 * 2), min(height, (int(math.ceil(y1)) + m + 1) // 2 * 2)
  step = 1
  while (cy1 - cy0) / step > SAMPLE_MAX_PX and step < 8:
    step *= 2
  uv_step = max(1, step // 2)
  img = nv12.reshape(-1, stride)
  luma = img[cy0:cy1:step, cx0:cx1:step]
  chroma = img[y_height + cy0 // 2:y_height + cy1 // 2, cx0:cx1].reshape(-1, (cx1 - cx0) // 2, 2)[::uv_step, ::uv_step]
  return {'x': cx0, 'y': cy0, 'w': cx1 - cx0, 'h': cy1 - cy0, 'step': step, 'uvStep': uv_step,
          'yb64': base64.b64encode(np.ascontiguousarray(luma).tobytes()).decode(),
          'uvb64': base64.b64encode(np.ascontiguousarray(chroma).tobytes()).decode()}


class LightReader:
  """read_head() on each class-9 box of a road camera frame, the voter, and which heads govern the lane. It counts the
  funnel since the last take_counts(), which modeld logs (modeld_detector.log_light_gates): frames read, boxes that never
  became heads (gate_counts: small, belowHorizon, overCap) and heads by this frame's reason (head_counts) and colour
  (colour_counts). The night tuning needs the gated ones: they are never published (Light.reason is the heads' alone).
  Every SAMPLE_S with a head in view it logs a sample of them, crops and all (module docstring; sample())."""
  def __init__(self, lens: Lens):
    self.lens = lens
    self.voter = HeadVoter()
    self.frames = 0
    self.gate_counts: Counter = Counter()
    self.head_counts: Counter = Counter()
    self.colour_counts: Counter = Counter()
    self.sample_t: float | None = None  # the last sample's frame time (None: none yet)

  def boxes_seen(self) -> bool:
    """Whether a class-9 box has come in since the last take_counts()."""
    return bool(self.gate_counts or self.head_counts)

  def take_counts(self) -> tuple[int, Counter, Counter, Counter]:
    """(frames, gate_counts, head_counts, colour_counts) since the last take, the counts starting over."""
    out = (self.frames, self.gate_counts, self.head_counts, self.colour_counts)
    self.frames, self.gate_counts, self.head_counts, self.colour_counts = 0, Counter(), Counter(), Counter()
    return out

  def read(self, t: float, nv12: np.ndarray, stride: int, y_height: int, rpy_calib, cam_height: float, boxes,
           path=None) -> list[LightHead]:
    """boxes: decode_lights() rows; path: (x, y left) of the lane ahead (tracker.predicted_path), None for straight."""
    reads = []
    self.frames += 1
    for box in boxes:
      if len(reads) >= MAX_READ_HEADS:
        self.gate_counts['overCap'] += 1
        continue
      r = read_head(nv12, stride, y_height, self.lens, rpy_calib, cam_height, box)
      if r.gated:
        self.gate_counts[r.reason] += 1
        continue
      self.head_counts[r.reason] += 1
      if r.state != 'unknown':
        self.colour_counts[r.state] += 1
      norm = (float(box[0]) / self.lens.width, float(box[1]) / self.lens.height, float(box[2]) / self.lens.width,
              float(box[3]) / self.lens.height)
      reads.append((norm, float(box[4]), r))
    heads = self.voter.update(t, [(norm, r.state) for norm, _, r in reads])
    out = []
    for hd, (norm, conf, r) in zip(heads, reads, strict=True):
      lane = 0.
      if path is not None:
        xs, ys = path
        lane = math.degrees(math.atan2(float(np.interp(r.distance, xs, ys)), r.distance))
      out.append(LightHead(hd.id, norm, conf, hd.state, hd.votes, r.state, r.reason, r.bearing, r.distance,
                           abs(r.bearing - lane) <= GOVERN_BEARING_DEG, r.detail))
    if out and (self.sample_t is None or not 0. <= t - self.sample_t < SAMPLE_S):
      self.sample_t = t
      self.sample(t, nv12, stride, y_height, out)
    # a box under the detector's MIN_CONF (decoder.LIGHT_READ_CONF, read for an earlier red) is a head only once it reads a
    # colour or holds a vote: the boxes that read no lamp stay unpublished, as before, so nothing downstream counts them as heads
    return [h for h in out if h.conf >= MIN_CONF or h.read != 'unknown' or h.state != 'unknown']

  def sample(self, t: float, nv12: np.ndarray, stride: int, y_height: int, heads: list[LightHead]) -> None:
    """Log up to SAMPLE_HEADS of this frame's heads, the governing first, then the unread, then the tallest: each one's read,
    vote and detail, and its box's NV12 (nv12_crop), so a drive's rlog shows the lamps each read saw (turnpilot.light_sample,
    debug: the rlog, not the swaglog files). A failure here is logged and never fails the read."""
    try:
      W, H = self.lens.width, self.lens.height
      chosen = sorted(heads, key=lambda h: (not h.governs, h.read != 'unknown', -(h.box[3] - h.box[1])))[:SAMPLE_HEADS]
      out = []
      for h in chosen:
        box = (h.box[0] * W, h.box[1] * H, h.box[2] * W, h.box[3] * H)
        out.append({'id': h.id, 'governs': h.governs, 'state': h.state, 'votes': h.votes, 'read': h.read, 'reason': h.reason,
                    'detail': h.detail, 'bearing': round(h.bearing, 1), 'distance': round(h.distance, 1),
                    'conf': round(h.conf, 2), 'box': [round(b, 1) for b in box],
                    'crop': nv12_crop(nv12, stride, y_height, W, H, box)})
      cloudlog.event("turnpilot.light_sample", debug=True, t=round(t, 3), heads=out)
    except Exception:
      cloudlog.exception("turnpilot light sample failed")

  @staticmethod
  def governing_state(heads: list[LightHead]) -> str:
    states = {h.state for h in heads if h.governs and h.state != 'unknown'}
    return states.pop() if len(states) == 1 else 'unknown'
