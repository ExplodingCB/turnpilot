import base64
from unittest import mock

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.turnpilot.detector import light_state
from openpilot.selfdrive.turnpilot.detector.light_state import GOVERN_BEARING_DEG, HEAD_MIN_H_PX, LIT_FRAC_MIN, MAX_READ_HEADS, \
  MIN_ELEVATION_DEG, RUN_MAX_REL, SAMPLE_HEADS, SAMPLE_MAX_PX, SAMPLE_S, SECOND_LIT_REL, STREAK_MAX, VOTE_M, VOTE_N, HeadVoter, \
  LightReader, hue_color, lamp_pixels, lamp_runs, read_head
from openpilot.selfdrive.turnpilot.scene.ranging import Lens
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

W, H = 1344, 760
LENS = Lens(W, H, 1141.5)
STRIDE, Y_HEIGHT, UV_HEIGHT, _ = get_nv12_info(W, H)
LEVEL = [0., 0., 0.]  # calibrated level: the horizon is the middle row
CY = H / 2
COLORS = {'red': (255, 40, 30), 'yellow': (255, 190, 0), 'green': (40, 255, 170), 'white': (255, 255, 255)}
LAMPS = {'red': 0, 'yellow': 1, 'green': 2}


def nv12(rgb: np.ndarray) -> np.ndarray:
  """An (H, W, 3) RGB frame into camerad's padded NV12, flat (BT.601 full range)."""
  r, g, b = (rgb[..., i].astype(np.float32) for i in range(3))
  y = 0.299 * r + 0.587 * g + 0.114 * b
  u = (b - y) / 1.772 + 128
  v = (r - y) / 1.402 + 128
  out = np.zeros((Y_HEIGHT + UV_HEIGHT, STRIDE), np.uint8)
  out[:H, :W] = np.clip(y, 0, 255)
  uv = np.stack([u[::2, ::2], v[::2, ::2]], axis=-1).reshape(H // 2, W)
  out[Y_HEIGHT:Y_HEIGHT + H // 2, :W] = np.clip(uv, 0, 255)
  return out.reshape(-1)


def scene(heads=(), bg=(90, 110, 140)):
  """A frame with vertical heads drawn: (x0, y0, x1, y1, lit, color) with lit 0 top, 1 middle, 2 bottom, or None for
  dark; color a COLORS key; lit may be a list of lamps. Returns (nv12, boxes as decode() rows)."""
  img = np.zeros((H, W, 3), np.uint8)
  img[:] = bg
  boxes = []
  for x0, y0, x1, y1, lit, color in heads:
    img[y0:y1, x0:x1] = (25, 25, 25)
    third = (y1 - y0) / 3
    for lamp in ([] if lit is None else lit if isinstance(lit, list) else [lit]):
      cx, cy, rad = (x0 + x1) / 2, y0 + third * (lamp + 0.5), 0.38 * min(x1 - x0, third)
      yy, xx = np.mgrid[y0:y1, x0:x1]
      disc = (xx + 0.5 - cx) ** 2 + (yy + 0.5 - cy) ** 2 <= rad ** 2
      img[y0:y1, x0:x1][disc] = COLORS[color]
    boxes.append([x0, y0, x1, y1, 0.6, 9])
  return nv12(img), np.array(boxes, np.float32).reshape(-1, 6)


def head(lit, color, x0=660, y0=200, w=14, h=42):
  return (x0, y0, x0 + w, y0 + h, lit, color)


def read(heads, rpy=LEVEL):
  frame, boxes = scene(heads)
  return [read_head(frame, STRIDE, Y_HEIGHT, LENS, rpy, 1.22, b) for b in boxes]


# A lamp at night as the 11-06 drive's camera saw one facing the car (1d 4:05, the qlog thumbnail of a governing head 31 m off,
# box 24x46): its face blown to a tinted white (red: S 0.15-0.29, no hue at LIT_S), ringed by a saturated glow (V 150-200,
# S 0.5-0.9) that reaches about 3 px above the face, 4-5 px to its sides and 8 px below it (the visor shades it above); the
# housing as dark as the sky. CORE the face's colour, GLOW the glow's hue at V 1.0
NIGHT_SKY = (20, 17, 15)
CORE = {'red': (255, 205, 200), 'yellow': (255, 238, 185), 'green': (200, 255, 238)}
GLOW = {'red': (0.95, 0.15, 0.22), 'yellow': (0.95, 0.62, 0.08), 'green': (0.12, 0.92, 0.78)}
SKY = (105, 175, 225)  # a day sky: hue 205, S 0.53, V 225 (the demo route's sky in its boxes: hue 195-216)


def night_lamp(img: np.ndarray, cx: float, cy: float, color: str, r=5.5, up=3., side=4.5, down=8.) -> None:
  yy, xx = np.mgrid[0:H, 0:W]
  dx, dy = xx + 0.5 - cx, yy + 0.5 - cy
  d = np.hypot(dx, dy)
  ext = np.where(dy < 0, np.hypot(dx / (r + side), dy / (r + up)), np.hypot(dx / (r + side), dy / (r + down)))
  glow = (d > r) & (ext <= 1.)
  fade = np.clip((ext - r / (r + side)) / (1 - r / (r + side)), 0, 1)  # 0 at the face, 1 at the glow's edge
  value = 205. - 95. * fade
  img[glow] = (np.array(GLOW[color])[None, :] * value[glow][:, None]).astype(np.uint8)
  img[d <= r] = CORE[color]


def night_read(lamps, box, **kw):
  """read_head on a night frame: lamps (centre x, centre y, colour) drawn by night_lamp(**kw), box x0 y0 x1 y1."""
  img = np.zeros((H, W, 3), np.uint8)
  img[:] = NIGHT_SKY
  for cx, cy, color in lamps:
    night_lamp(img, cx, cy, color, **kw)
  frame = nv12(img)
  return frame, read_head(frame, STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, np.array(list(box) + [0.5, 9], np.float32))


def thirds(frame, box) -> list[float]:
  """The box's thirds' lamp shares over their inner columns, as the reader takes them."""
  x0, y0, x1, y1 = box
  lamp = lamp_pixels(frame, STRIDE, Y_HEIGHT, x0, y0, x1, y1)[0]
  h, w = lamp.shape
  e = np.rint(np.linspace(0, h, 4)).astype(int)
  inner = slice(int(round(w * 0.15)), int(round(w * 0.85)))
  return [float(lamp[e[k]:e[k + 1], inner].mean()) for k in range(3)]


def lit_run(frame, box):
  """The box's runs (lamp_runs) and its lit run's widest row across the box, as the reader takes them."""
  x0, y0, x1, y1 = box
  lamp, _, value = lamp_pixels(frame, STRIDE, Y_HEIGHT, x0, y0, x1, y1)
  w = lamp.shape[1]
  inner = slice(int(round(w * 0.15)), int(round(w * 0.85)))
  runs = lamp_runs(lamp[:, inner], value[:, inner])
  r0, r1 = max(runs, key=lambda r: r[2])[:2]
  return runs, int(lamp[r0:r1 + 1].sum(axis=1).max())


def disc(img, cx, cy, rad, color):
  yy, xx = np.mgrid[0:H, 0:W]
  img[(xx + 0.5 - cx) ** 2 + (yy + 0.5 - cy) ** 2 <= rad ** 2] = color


class TestReadHead(OpenpilotTestCase):
  def test_lamp_and_colour_agree(self):
    for color, lamp in LAMPS.items():
      r = read([head(lamp, color)])[0]
      assert (r.state, r.reason) == (color, 'read'), (color, r)

  def test_smallest_readable_head(self):
    for color, lamp in LAMPS.items():
      assert read([head(lamp, color, w=9, h=HEAD_MIN_H_PX)])[0].state == color

  def test_colour_in_the_wrong_lamp_is_unknown(self):
    # a red lit at the bottom is not a head's red: a tail light, a sign, glare
    r = read([head(2, 'red')])[0]
    assert (r.state, r.reason) == ('unknown', 'positionMismatch')
    assert read([head(0, 'green')])[0].state == 'unknown'

  def test_two_lamps_or_none_is_unknown(self):
    assert read([head([0, 2], 'red')])[0].reason == 'twoLit'
    assert read([head(None, 'red')])[0].reason == 'dark'

  def test_blown_out_lamp_without_colour_is_unknown(self):
    assert read([head(0, 'white')])[0].state == 'unknown'

  @staticmethod
  def left_arrow(color, lamp):
    """A head with a left arrow lit in lamp (0 top, 2 bottom): a head on the left, the shaft to the right."""
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = (90, 110, 140)
    x0, y0, w, h = 640, 150, 30, 90
    img[y0:y0 + h, x0:x0 + w] = (25, 25, 25)
    tip_x, cy = x0 + 4, y0 + 15 + 30 * lamp
    for dy in range(-9, 10):
      img[cy + dy, tip_x + abs(dy): tip_x + abs(dy) + 3] = COLORS[color]
    img[cy - 2:cy + 3, tip_x:x0 + w - 4] = COLORS[color]
    box = np.array([x0, y0, x0 + w, y0 + h, 0.6, 9], np.float32)
    return read_head(nv12(img), STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, box)

  def test_red_left_arrow_keeps_its_colour(self):
    # never a go for any lane: a red arrow reads red (reason arrow), so turn_light and release_light block on it
    r = self.left_arrow('red', 0)
    assert (r.state, r.reason) == ('red', 'arrow'), r

  def test_green_and_yellow_left_arrows_are_unknown(self):
    # a green arrow is never a permission; a yellow one may be a flashing yellow arrow's permissive left
    r = self.left_arrow('green', 2)
    assert (r.state, r.reason) == ('unknown', 'arrow'), r
    r = self.left_arrow('yellow', 1)
    assert (r.state, r.reason) == ('unknown', 'arrow'), r

  def test_gates_before_the_colour_read(self):
    # a red head's colours, but too small to read, below the horizon (a tail light), or lying flat (a horizontal head)
    small = read([head(0, 'red', h=HEAD_MIN_H_PX - 3, w=7)])[0]
    assert (small.state, small.reason, small.gated) == ('unknown', 'small', True)
    low_v = int(CY - LENS.f * np.tan(np.radians(MIN_ELEVATION_DEG)) + 5)
    low = read([head(0, 'red', y0=low_v - 42)])[0]
    assert (low.state, low.reason, low.gated) == ('unknown', 'belowHorizon', True)
    flat = read([(600, 200, 642, 214, 0, 'red')])[0]
    assert (flat.state, flat.reason) == ('unknown', 'aspect')
    assert not flat.gated  # a horizontal head is a head, read as unknown

  def test_horizon_follows_calibration(self):
    # pitched 3 deg the horizon moves ~60 px: a head whose bottom is 2.4 deg over the middle row is then below it
    y0 = int(CY - 90)
    assert read([head(0, 'red', y0=y0)])[0].reason == 'read'
    assert read([head(0, 'red', y0=y0)], rpy=[0., np.radians(3.), 0.])[0].reason == 'belowHorizon'

  def test_bearing_and_distance(self):
    r = read([head(0, 'red', x0=900)])[0]
    assert r.bearing < 0  # right of the axis
    near, far = read([head(0, 'red', y0=100)])[0], read([head(0, 'red', y0=300)])[0]
    assert near.distance < far.distance


class TestNightLamps(OpenpilotTestCase):
  BOX = (660, 200, 684, 246)  # 24x46, the 1d 4:05 governing head's box

  def at(self, frac: float) -> float:
    return self.BOX[1] + frac * (self.BOX[3] - self.BOX[1])

  def test_a_bloomed_red_reads_red(self):
    # 11-06 night: no colour in 84 reads of the governing head at 1d 4:05, standing 31 m from it (module docstring). Its
    # red's glow fills the middle third about as much as the lamp fills the top, and the read before 2026-11-07 (another
    # third with half the lit one's share) called it two lamps. One lamp is one run, its core's centre in the top third
    frame, r = night_read([(672, self.at(0.27), 'red')], self.BOX)
    top, middle, _ = thirds(frame, self.BOX)
    assert middle >= max(LIT_FRAC_MIN, SECOND_LIT_REL * top)  # the old read's twoLit
    assert (r.state, r.reason) == ('red', 'read'), r

  def test_bloomed_yellow_and_green_read_their_colour(self):
    for color, frac in (('yellow', 0.5), ('green', 0.8)):
      _, r = night_read([(672, self.at(frac), color)], self.BOX)
      assert (r.state, r.reason) == (color, 'read'), (color, r)
    # a brighter yellow's glow reaching 18 px below it fills the bottom third past half the middle's share: the old read's
    # twoLit, one lamp read as runs
    frame, r = night_read([(672, self.at(0.5), 'yellow')], self.BOX, down=18.)
    _, middle, bottom = thirds(frame, self.BOX)
    assert bottom >= max(LIT_FRAC_MIN, SECOND_LIT_REL * middle)
    assert (r.state, r.reason) == ('yellow', 'read'), r

  def test_two_bloomed_lamps_are_two_lit(self):
    # a red ball and a lit lamp below it (a green arrow's section): two runs, the dark housing between, never a colour
    _, r = night_read([(672, self.at(0.18), 'red'), (672, self.at(0.84), 'green')], self.BOX)
    assert (r.state, r.reason) == ('unknown', 'twoLit'), r
    assert read([head([0, 1], 'red', h=60)])[0].reason == 'twoLit'  # adjacent lamps by day: the housing between them

  def test_a_glow_with_no_head_around_it_is_unknown(self):
    # a box drawn round the glow alone (the housing unseen, its top and bottom guessed): the lamp sits in its middle, and no
    # red or green is read there
    for color in ('red', 'green'):
      _, r = night_read([(672, self.at(0.5), color)], self.BOX, r=8., up=8., side=4., down=10.)
      assert r.state == 'unknown', (color, r)

  def test_the_core_not_the_fuller_third_places_the_lamp(self):
    # a lamp's glow trailing 22 px below it fills the next third more than the lamp's own: the third with the most lamp
    # pixels put a red in the middle and a yellow at the bottom (positionMismatch); the core's centre is the lamp's, and so is
    # the core and not the mean row of all its pixels, which the glow drags down a third
    for color, frac, lamp_third in (('red', 0.21, 0), ('yellow', 0.56, 1)):
      frame, r = night_read([(672, self.at(frac), color)], self.BOX, r=5.5 * .75, up=3. * .75, side=4.5 * .75, down=22.)
      fr = thirds(frame, self.BOX)
      assert fr[lamp_third + 1] > fr[lamp_third], (color, fr)  # the fuller third is the next one down
      assert (r.state, r.reason) == (color, 'read'), (color, r)

  def test_a_blown_face_reads_its_colour_from_its_glow(self):
    # the 1d 4:05 red's face was blown pink-white, no hue (S 0.15-0.29): its colour was in its glow. A face filling its third
    # (a near head, the visor shading the glow above it) leaves that third too few saturated pixels to say a colour, and the
    # read on the third alone said noColor; the lamp's run, its glow below the face included, is red
    frame, r = night_read([(672, self.at(0.22), 'red')], self.BOX, r=8., up=0., side=4.5, down=6.)
    hue = lamp_pixels(frame, STRIDE, Y_HEIGHT, *self.BOX)[1]
    assert hue_color(hue[:15, 4:20]) is None  # the top third's inner columns alone: no colour
    assert (r.state, r.reason) == ('red', 'read'), r

  def test_a_glow_trailing_a_small_face_is_the_lamp(self):
    # a far head's red (14x36), its blown core 3 px (the lens's centre) and its glow trailing 14 px below it (a wet or smeared
    # windscreen; the 1d 4:05 glow reaches 8 px): its run is taller than STREAK_MAX of its widest row, and the shape test on
    # the whole run said streak where the read before 2026-11-07 read red (the review's grid: 14 reds, 17 yellows, 3 greens).
    # Its core is round: a lamp
    box = (660, 200, 674, 236)
    frame, r = night_read([(667, 203.5, 'red')], box, r=1.6, up=0.9, side=1.3, down=14.)
    runs, widest = lit_run(frame, box)
    (r0, r1, _, _, core_rows), = runs
    assert r1 - r0 + 1 > STREAK_MAX * widest and r1 - r0 + 1 <= RUN_MAX_REL * (box[3] - box[1])
    assert core_rows <= STREAK_MAX * widest
    assert (r.state, r.reason) == ('red', 'read'), r

  def test_a_seam_across_a_lamp_is_one_lamp(self):
    # two dark rows across a lit lamp (an LED's dark ring, a louvre's slat, a chroma row pair off its hue) are within GAP_REL
    # of the box: one lamp, never two
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = (90, 110, 140)
    img[200:242, 660:674] = (25, 25, 25)
    disc(img, 667, 207, 5.32, COLORS['red'])
    img[206:208, 660:674] = (25, 25, 25)
    frame = nv12(img)
    box = (660, 200, 674, 242)
    lamp = lamp_pixels(frame, STRIDE, Y_HEIGHT, *box)[0]
    assert not lamp[6:8].any() and lamp[5].any() and lamp[8].any()  # the seam: two rows of nothing
    assert len(lit_run(frame, box)[0]) == 1
    r = read_head(frame, STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, np.array(list(box) + [0.5, 9], np.float32))
    assert (r.state, r.reason) == ('red', 'read'), r


class TestNotALamp(OpenpilotTestCase):
  def test_a_streak_is_not_a_lamp(self):
    # 1e 7:05 (qlog thumbnail): a pole lit by a streetlight, boxed as a light 14x44 and governing, a strip 8 px wide and 30
    # tall of the streetlight's orange (hue 32, in yellow's range). Read as runs it is one lamp in the middle third: yellow, a
    # stop on a through road; the read before said twoLit. Its rows are 4-5 times its width: a streak
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = NIGHT_SKY
    img[211:241, 584:592] = (190, 130, 60)
    r = read_head(nv12(img), STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, np.array([578, 201, 592, 245, 0.5, 9], np.float32))
    assert (r.state, r.reason) == ('unknown', 'streak'), r
    # the same pole lit 24 rows, under RUN_MAX_REL of its box: its core, lit end to end, is 3 times its width
    img[:] = NIGHT_SKY
    img[214:238, 584:592] = (190, 130, 60)
    pole = (578, 201, 592, 245)
    runs, widest = lit_run(nv12(img), pole)
    (r0, r1, _, _, core_rows), = runs
    assert r1 - r0 + 1 <= RUN_MAX_REL * (pole[3] - pole[1]) and core_rows > STREAK_MAX * widest
    r = read_head(nv12(img), STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, np.array(list(pole) + [0.5, 9], np.float32))
    assert (r.state, r.reason) == ('unknown', 'streak'), r
    # a yellow warning sign filling its box (the demo route, 22x42): lit rows over 0.6 of the box
    img[:] = (90, 110, 140)
    img[202:241, 661:681] = (230, 190, 20)
    r = read_head(nv12(img), STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, np.array([660, 200, 682, 242, 0.5, 9], np.float32))
    assert (r.state, r.reason) == ('unknown', 'streak'), r

  def test_the_sky_is_not_a_green_lamp(self):
    # the demo route's sky in its boxes is hue 195-216, its green lamps 166-181: up to 210 the sky was a green lamp. A box
    # running down past a red head into the sky (as the demo route's run past a head by a quarter of it) read green
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = SKY
    img[200:228, 660:674] = (25, 25, 25)  # the head, the box's top two thirds
    box = np.array([660, 200, 674, 242, 0.5, 9], np.float32)
    r = read_head(nv12(img), STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, box)
    assert (r.state, r.reason) == ('unknown', 'dark'), r  # was green
    disc(img, 667, 207, 4.2, COLORS['red'])
    r = read_head(nv12(img), STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, box)
    assert (r.state, r.reason) == ('red', 'read'), r  # was green: a green on a red light

  def test_the_sky_beside_a_ball_is_not_an_arrow(self):
    # 15 of the demo route's green balls read as arrows (unknown): the box overhung the head on one side, and the sky there
    # was lamp pixels in the lamp's rows
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = SKY
    img[200:242, 660:671] = (25, 25, 25)
    disc(img, 665.5, 235, 4.2, COLORS['green'])
    r = read_head(nv12(img), STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, np.array([660, 200, 674, 242, 0.5, 9], np.float32))
    assert (r.state, r.reason) == ('green', 'read'), r

  def test_a_glint_is_not_the_lamps_core(self):
    # the sun on the visor's lip just above a lit yellow, blown white, joins the lamp's run: its core is the run's brightest
    # tenth (the 90th percentile), not its brightest pixel, which would put the yellow in the top third (positionMismatch)
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = (90, 110, 140)
    img[200:242, 660:674] = (25, 25, 25)
    disc(img, 667, 221, 5.32, (210, 160, 0))
    img[212:214, 665:669] = (255, 255, 255)
    frame = nv12(img)
    box = (660, 200, 674, 242)
    (r0, *_), = lit_run(frame, box)[0]
    assert r0 <= 12  # the glint's rows are the lamp's run
    r = read_head(frame, STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, np.array(list(box) + [0.5, 9], np.float32))
    assert (r.state, r.reason) == ('yellow', 'read'), r

  def test_a_read_says_what_it_saw(self):
    # the detail the logs carry (HeadRead.detail): the thirds' shares, the lit run and its core's centre, the colours
    r = read([head(0, 'red')])[0]
    for part in ('h42', 'fr', 'run', 'lamp', 'c0.1', 'wd', 'runs1', 'pk', 'hue r'):
      assert part in r.detail, (part, r.detail)
    assert read([head(None, 'red')])[0].detail.startswith('h42 fr')  # dark: the shares alone


class TestVote(OpenpilotTestCase):
  def votes(self, states, box=(0.49, 0.25, 0.5, 0.3)):
    v = HeadVoter()
    out = None
    for i, s in enumerate(states):
      out = v.update(i * 0.1, [(box, s)] if s is not None else [])
    return out

  def test_n_of_m(self):
    assert VOTE_N == 4 and VOTE_M == 5
    assert self.votes(['red'] * 3)[0].state == 'unknown'
    assert self.votes(['red'] * 4)[0].state == 'red'
    assert self.votes(['red', 'red', 'green', 'red', 'red'])[0].state == 'red'
    assert self.votes(['red', 'green', 'red', 'green', 'red', 'red'])[0].state == 'unknown'

  def test_a_missed_frame_counts_against(self):
    assert self.votes(['red', 'red', None, 'red', 'red'])[0].state == 'red'
    out = self.votes(['red', 'red', None, None, 'red'])
    assert out == [] or out[0].state == 'unknown'

  def test_heads_kept_apart_and_dropped(self):
    v = HeadVoter()
    a, b = (0.30, 0.25, 0.31, 0.3), (0.60, 0.25, 0.61, 0.3)
    for i in range(5):
      out = v.update(i * 0.1, [(a, 'red'), (b, 'green')])
    assert sorted(h.state for h in out) == ['green', 'red']
    assert v.update(2.0, []) == []


class TestReader(OpenpilotTestCase):
  def test_governing_heads_vote(self):
    reader = LightReader(LENS)
    frame, boxes = scene([head(0, 'red'), head(2, 'green', x0=1250)])
    for i in range(VOTE_N):
      out = reader.read(i * 0.1, frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, boxes)
    gov = [h for h in out if h.governs]
    assert [h.state for h in gov] == ['red']
    side = next(h for h in out if not h.governs)
    assert abs(side.bearing) > GOVERN_BEARING_DEG and side.state == 'green'
    assert reader.governing_state(out) == 'red'

  def test_disagreeing_governing_heads_are_unknown(self):
    reader = LightReader(LENS)
    frame, boxes = scene([head(0, 'red', x0=640), head(2, 'green', x0=700)])
    for i in range(VOTE_N):
      out = reader.read(i * 0.1, frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, boxes)
    assert reader.governing_state(out) == 'unknown'

  def test_gated_boxes_are_not_published(self):
    reader = LightReader(LENS)
    frame, boxes = scene([head(0, 'red', y0=500)])  # well below the horizon
    assert reader.read(0., frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, boxes) == []
    assert reader.gate_counts['belowHorizon'] == 1

  def test_the_funnel_is_counted_and_taken(self):
    # modeld logs the reader's funnel once a minute (modeld_detector.log_light_gates): the frames read, the boxes the gates
    # stop (never heads, never published), and the heads by this frame's reason and colour; a take starts the counts over
    reader = LightReader(LENS)
    frame, boxes = scene([head(0, 'red'), head(2, 'red', x0=400), head(0, 'red', x0=900, w=6, h=18), head(0, 'red', x0=300, y0=500)])
    assert not reader.boxes_seen()
    for i in range(3):
      reader.read(i * 0.1, frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, boxes)
    assert reader.boxes_seen()
    frames, gates, heads, colours = reader.take_counts()
    assert frames == 3 and gates == {'small': 3, 'belowHorizon': 3} and heads == {'read': 3, 'positionMismatch': 3}
    assert colours == {'red': 3}
    assert reader.take_counts() == (0, {}, {}, {}) and not reader.boxes_seen()
    reader.read(1., frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, np.zeros((0, 6), np.float32))
    assert reader.take_counts()[0] == 1 and not reader.boxes_seen()

  def test_at_most_max_read_heads_a_frame(self):
    # the read's time stays bounded however many heads the junction has: the most confident first, the rest counted
    reader = LightReader(LENS)
    frame, boxes = scene([head(0, 'red', x0=100 + 60 * i) for i in range(MAX_READ_HEADS + 4)])
    assert len(reader.read(0., frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, boxes)) == MAX_READ_HEADS
    assert reader.gate_counts['overCap'] == 4


def decode_crop(crop: dict) -> tuple[np.ndarray, np.ndarray]:
  """A sample's crop back to (luma rows x cols, chroma rows x pairs x 2)."""
  step, uv_step = crop['step'], crop['uvStep']
  rows, cols = -(-crop['h'] // step), -(-crop['w'] // step)
  luma = np.frombuffer(base64.b64decode(crop['yb64']), np.uint8).reshape(rows, cols)
  chroma = np.frombuffer(base64.b64decode(crop['uvb64']), np.uint8).reshape(-(-crop['h'] // 2 // uv_step), -(-crop['w'] // 2 // uv_step), 2)
  return luma, chroma


class TestSamples(OpenpilotTestCase):
  """The 11-06 night drives left no frames to tell why no governing head read a colour: every SAMPLE_S with a head in view
  the reader logs its heads' reads and their boxes' pixels (turnpilot.light_sample), so the next night's rlog has them."""
  @staticmethod
  def samples(event) -> list[dict]:
    return [c.kwargs for c in event.call_args_list if c.args and c.args[0] == 'turnpilot.light_sample']

  def test_a_sample_every_sample_s_the_governing_first(self):
    reader = LightReader(LENS)
    frame, boxes = scene([head(0, 'red', x0=1250), head(None, 'red', x0=400), head(0, 'red')])  # side, side, governing
    none = np.zeros((0, 6), np.float32)
    with mock.patch.object(light_state.cloudlog, 'event') as event:
      out = []
      for i in range(51):  # 5 s of road runs at 10 Hz, then 3 s with no head in view
        out = reader.read(i * 0.1, frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, boxes)
      for i in range(51, 81):
        reader.read(i * 0.1, frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, none)
    got = self.samples(event)
    assert SAMPLE_S == 2.0 and [s['t'] for s in got] == [0.0, 2.0, 4.0]
    for s in got:
      assert s['debug'] and len(s['heads']) == SAMPLE_HEADS
      first, second = s['heads']
      assert first['governs'] and not second['governs'] and second['read'] == 'unknown'  # the governing, then the unread
      assert first['read'] == 'red' and 'run' in first['detail'] and first['box'] == [660., 200., 674., 242.]
    assert all(h.detail for h in out)  # published with the head (LightHead.detail)

  def test_a_sample_crop_is_the_frame(self):
    reader = LightReader(LENS)
    frame, boxes = scene([head(0, 'red'), head(2, 'green', x0=300, y0=20, w=60, h=200)])
    with mock.patch.object(light_state.cloudlog, 'event') as event:
      reader.read(0., frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, boxes)
    crops = {tuple(h['box']): h['crop'] for h in self.samples(event)[0]['heads']}
    img = frame.reshape(-1, STRIDE)
    small, tall = crops[(660., 200., 674., 242.)], crops[(300., 20., 360., 220.)]
    # the small head at full resolution, a sixth of its height around it
    assert small['step'] == 1 and (small['x'], small['y']) == (652, 192) and small['x'] + small['w'] >= 674 + 7
    luma, chroma = decode_crop(small)
    x, y, w, h = small['x'], small['y'], small['w'], small['h']
    assert np.array_equal(luma, img[y:y + h, x:x + w])
    assert np.array_equal(chroma.reshape(h // 2, w), img[Y_HEIGHT + y // 2:Y_HEIGHT + (y + h) // 2, x:x + w])
    # a tall one subsampled to at most SAMPLE_MAX_PX rows
    luma, chroma = decode_crop(tall)
    x, y, w, h, step = tall['x'], tall['y'], tall['w'], tall['h'], tall['step']
    assert step in (2, 4) and luma.shape[0] <= SAMPLE_MAX_PX and np.array_equal(luma, img[y:y + h:step, x:x + w:step])
    assert chroma.shape[:2] == luma.shape  # step 2 or more: a chroma pair per luma pixel
    # a head filling the frame's height (a few metres off) still fits: step 8
    luma, chroma = decode_crop(light_state.nv12_crop(frame, STRIDE, Y_HEIGHT, W, H, (200., 10., 400., 510.)))
    assert luma.shape[0] <= SAMPLE_MAX_PX and chroma.shape[:2] == luma.shape

  def test_a_failed_sample_never_fails_the_read(self):
    reader = LightReader(LENS)
    frame, boxes = scene([head(0, 'red')])
    with mock.patch.object(light_state, 'nv12_crop', side_effect=ValueError('bad crop')), \
         mock.patch.object(light_state.cloudlog, 'exception') as failed, mock.patch.object(light_state.cloudlog, 'event') as event:
      for i in range(VOTE_N):
        out = reader.read(i * 0.1, frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, boxes)
    assert [h.state for h in out] == ['red'] and failed.call_count == 1 and not self.samples(event)  # said once, read on
