"""Traffic light state from the detector's class-9 boxes, read on the CPU copy of the road camera's full-resolution frame
(DESIGN phase 4 G2). Shadow only this week: what it reads is published and logged, and nothing acts on it.

A box is gated before any colour is read, so that tail and brake lights, red signs and glare low in the frame never
become a red (CRITIQUE REQUIRED 8):
- small: its long side under HEAD_MIN_H_PX full-frame px. Below that a lamp is under about 8 px and a ball can't be told from
  an arrow, so it is never read (a 1.07 m head at about 50 m on a comma four).
- belowHorizon: its bottom edge is less than MIN_ELEVATION_DEG above the calibrated horizon. A signal head hangs 4-6 m
  over the road, above the camera; a car's lights are below it.
Gated boxes are counted, never published. A box past the gates is a head, and its state is read:
- aspect: not a vertical head (height/width outside MIN_ASPECT-MAX_ASPECT): horizontal, a doghouse, a pedestrian signal.
  Unknown.
- The box is split in thirds; a lamp pixel is bright (HSV value) and either saturated in a lamp hue or blown out white.
  The lit third holds the most; none past LIT_FRAC_MIN is dark, two is twoLit. Unknown either way.
- noColor: the lit lamp's saturated pixels don't agree on red, yellow or green. Unknown.
- positionMismatch: the colour isn't the one its lamp shows (red top, yellow middle, green bottom). Unknown.
- arrow: the lit lamp's pixels aren't mirror-symmetric left to right about their own centre, as a ball is and a turn
  arrow is not. Unknown: a red left arrow is never a red for the through lane, a green arrow never a permission. This is
  a proxy at these sizes; the replay measures it.
A head's state is its vote: VOTE_N of its last VOTE_M reads (a frame it wasn't seen in counts as unknown), which also
rides out LED flicker. A head governs the car's lane when its bearing lies within GOVERN_BEARING_DEG of the lane's (the
driving model's path at the head's distance), and the governing state is what all governing heads with a state agree
on, else unknown.

Distance is to the head, from its elevation and an assumed height: the stop line is not known from the camera.
"""
import math
from collections import Counter, deque
from dataclasses import dataclass, field

import numpy as np

from openpilot.common.params import UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.scene.ranging import Lens, calib_from_view

HEAD_MIN_H_PX = 24          # full-frame px
MIN_ELEVATION_DEG = 1.0     # the box's bottom edge above the calibrated horizon
MIN_ASPECT, MAX_ASPECT = 1.8, 4.5  # a vertical 3-lamp head's height over width
INNER_X = 0.15              # each side of the box left out of the lamp counts (pole, backplate edge, sky)
LIT_V = 150                 # HSV value (0-255) of a lamp pixel...
LIT_S = 0.35                # ...with this saturation in a lamp hue...
BLOWN_V = 235               # ...or blown out this bright
LIT_FRAC_MIN = 0.08         # share of a third's pixels that are lamp pixels for it to be lit
SECOND_LIT_REL = 0.5        # another third with this share of the lit one's (and LIT_FRAC_MIN) is a second lit lamp
MIN_COLOR_PX = 3            # saturated pixels in the lit lamp to read a colour from
COLOR_SHARE = 0.6           # of them, this share in one hue range
HUES = {'red': ((330., 360.), (0., 18.)), 'yellow': ((18., 65.),), 'green': ((120., 210.),)}
LAMP = {'red': 0, 'yellow': 1, 'green': 2}  # top, middle, bottom of a vertical head
ARROW_MIN_PX = 10           # lamp pixels needed to judge the lamp's shape; fewer is unknown (tooFewLampPx)
SYMMETRY_MIN = 0.6          # the lamp's mask against its mirror image about its own centre (IoU): a ball, not an arrow
ASSUMED_HEAD_M = 5.5        # m, a head's centre over the road (MUTCD clearance 15-19 ft to its bottom)
MAX_RANGE = 150.            # m, the farthest distance it says
VOTE_M, VOTE_N = 5, 4       # a state takes VOTE_N of a head's last VOTE_M reads
MATCH_HEIGHTS = 1.5         # a read is the same head when its box centre is within this many box heights
HEAD_DROP_S = 0.6           # a head not seen this long is forgotten
GOVERN_BEARING_DEG = 10.    # a head within this of the lane's bearing governs it
MAX_READ_HEADS = 8          # heads read a frame, the most confident first (decode_lights' order): the read's time stays bounded
STATES = ('red', 'yellow', 'green')
LIGHT_MODES = ('off', 'shadow', 'addOnly')  # TurnPilotCameraLights
LIGHT_MODE_DEFAULT = 'shadow'  # read and logged, never acted on (CRITIQUE: no stop authority until the replay gate passes)


def camera_lights_mode(params) -> str:
  """TurnPilotCameraLights, read by modeld and turnpilotd: off, shadow (the default, and anything unknown) or addOnly."""
  try:
    value = params.get("TurnPilotCameraLights")
  except UnknownKeyName:
    cloudlog.error("turnpilot: TurnPilotCameraLights unknown to this build's params, camera lights in shadow; run scons")
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


def lamp_pixels(nv12: np.ndarray, stride: int, y_height: int, x0: int, y0: int, x1: int, y1: int):
  """(lamp mask, saturated hue or nan) per pixel of the box, from NV12 (BT.601 full range)."""
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
  return lamp, hue


def hue_color(hues: np.ndarray) -> str | None:
  hues = hues[np.isfinite(hues)]
  if len(hues) < MIN_COLOR_PX:
    return None
  for name, ranges in HUES.items():
    n = sum(int(((hues >= lo) & (hues < hi)).sum()) for lo, hi in ranges)
    if n >= COLOR_SHARE * len(hues):
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
  lamp, hue = lamp_pixels(nv12, stride, y_height, ix0, iy0, ix1, iy1)
  h, w = lamp.shape
  edges = np.rint(np.linspace(0, h, 4)).astype(int)
  inner = slice(int(round(w * INNER_X)), max(int(round(w * (1 - INNER_X))), int(round(w * INNER_X)) + 1))
  fracs = np.array([lamp[edges[k]:edges[k + 1], inner].mean() if edges[k + 1] > edges[k] else 0. for k in range(3)])
  lit = int(np.argmax(fracs))
  if fracs[lit] < LIT_FRAC_MIN:
    out.reason = 'dark'
    return out
  others = np.delete(fracs, lit)
  if (others >= max(LIT_FRAC_MIN, SECOND_LIT_REL * fracs[lit])).any():
    out.reason = 'twoLit'
    return out
  rows = slice(edges[lit], edges[lit + 1])
  color = hue_color(hue[rows, inner])
  if color is None:
    out.reason = 'noColor'
    return out
  if LAMP[color] != lit:
    out.reason = 'positionMismatch'
    return out
  mask = lamp[rows]
  if mask.sum() < ARROW_MIN_PX:
    out.reason = 'tooFewLampPx'
    return out
  if not symmetric(mask):
    out.reason = 'arrow'
    return out
  out.state = color
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
    best = max(STATES, key=lambda s: counts[s])
    return best if counts[best] >= VOTE_N else 'unknown'

  @property
  def votes(self) -> int:
    counts = Counter(self.reads)
    return counts[self.state] if self.state != 'unknown' else max((counts[s] for s in STATES), default=0)


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


class LightReader:
  """read_head() on each class-9 box of a road camera frame, the voter, and which heads govern the lane."""
  def __init__(self, lens: Lens):
    self.lens = lens
    self.voter = HeadVoter()
    self.gate_counts: Counter = Counter()

  def read(self, t: float, nv12: np.ndarray, stride: int, y_height: int, rpy_calib, cam_height: float, boxes,
           path=None) -> list[LightHead]:
    """boxes: decode_lights() rows; path: (x, y left) of the lane ahead (tracker.predicted_path), None for straight."""
    reads = []
    for box in boxes:
      if len(reads) >= MAX_READ_HEADS:
        self.gate_counts['overCap'] += 1
        continue
      r = read_head(nv12, stride, y_height, self.lens, rpy_calib, cam_height, box)
      if r.gated:
        self.gate_counts[r.reason] += 1
        continue
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
                           abs(r.bearing - lane) <= GOVERN_BEARING_DEG))
    return out

  @staticmethod
  def governing_state(heads: list[LightHead]) -> str:
    states = {h.state for h in heads if h.governs and h.state != 'unknown'}
    return states.pop() if len(states) == 1 else 'unknown'
