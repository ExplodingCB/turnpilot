import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.turnpilot.detector.light_state import GOVERN_BEARING_DEG, HEAD_MIN_H_PX, MAX_READ_HEADS, MIN_ELEVATION_DEG, VOTE_M, \
  VOTE_N, HeadVoter, LightReader, read_head
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

  def test_left_arrow_is_unknown(self):
    frame, _ = scene([])
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = (90, 110, 140)
    x0, y0, w, h = 640, 150, 30, 90
    img[y0:y0 + h, x0:x0 + w] = (25, 25, 25)
    # a red left arrow in the top lamp: a head on the left, the shaft to the right
    tip_x, cy = x0 + 4, y0 + 15
    for dy in range(-9, 10):
      img[cy + dy, tip_x + abs(dy): tip_x + abs(dy) + 3] = COLORS['red']
    img[cy - 2:cy + 3, tip_x:x0 + w - 4] = COLORS['red']
    box = np.array([x0, y0, x0 + w, y0 + h, 0.6, 9], np.float32)
    r = read_head(nv12(img), STRIDE, Y_HEIGHT, LENS, LEVEL, 1.22, box)
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

  def test_at_most_max_read_heads_a_frame(self):
    # the read's time stays bounded however many heads the junction has: the most confident first, the rest counted
    reader = LightReader(LENS)
    frame, boxes = scene([head(0, 'red', x0=100 + 60 * i) for i in range(MAX_READ_HEADS + 4)])
    assert len(reader.read(0., frame, STRIDE, Y_HEIGHT, LEVEL, 1.22, boxes)) == MAX_READ_HEADS
    assert reader.gate_counts['overCap'] == 4
