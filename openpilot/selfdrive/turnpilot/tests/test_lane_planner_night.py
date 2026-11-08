"""The lane planner at night (2026-11-07): drives 1d and 1e (2026-11-06, after dark) offered no lane change on 32,640 plan
frames between them, and the driver changed lanes himself for the turns. Kellen: "we wanna get over to the right before a
right turn and the left before a left", "a line the reader can't read is not a reason to stay put when the model's lane
lines, road edges and the map's lane count agree a lane is there", no give-ups for lack of a read, the rails kept (never
across a line read yellow or solid, never into the road edge, a parking strip or an oncoming lane, never into a car seen
there or in the blind spot), and the driver signals each change (the car can't light its own lamps)."""
import json
from pathlib import Path
from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.lane_planner import (BROKEN_WHITE_CONF_MIN, LANE_WIDTH, LATCHED_YELLOW, LC_CONFIRM_FRAMES,
                                                        LC_UNSURE_MAX_S, LC_WARN_S, WHITE_CLEAR_FRAMES, WHITE_HOLD_S,
                                                        YELLOW_OUT_FRAMES, LanePlanner, LineMarks, Phase, YellowHold, change_blocker,
                                                        lane_agreed, lane_change_window, model_lanes, oncoming_in_lane, soft_block)
from openpilot.selfdrive.turnpilot.route_follower import LEFT, RIGHT, Maneuver
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint
from openpilot.selfdrive.turnpilot.tests.synthetic import road, scene
from openpilot.selfdrive.turnpilot.tests.test_lane_planner import (BW, BY, COLORLESS, CONTRARY, SW, SY, THREE_LANES_LEFT_TURN,
                                                                   TWO_LANES_RIGHT_TURN, UNREADABLE, UNSURE_BW, Drive, LaneChange,
                                                                   maneuver, paint)

LaneChangeState = log.LaneChangeState
WHITE_UNKNOWN_KIND = LinePaint('unknown', 'white', 0.9, 0.3)
NONE_READ = LinePaint('none', 'unknown', 1., 0.)  # the reader sees no line there (1e 304-306.7 s: the bay's taper)
BAY = json.loads(Path(__file__).with_name('lane_bay_night.json').read_text())['drives']
LEFT_BAY = ((("left",), True), (("straight",), False))  # a far_bay list: the junction's, its bay further on


def night_reads(white_first=WHITE_CLEAR_FRAMES, then=UNKNOWN, **lines):
  """Drive paint: the left ego line read white (kind unknown, the usual night read of a dashed line) on its first
  white_first frames, then `then`; the far-left line the same unless given (lines: paint()'s other lines)."""
  frames = {'n': 0}

  def read(left, right):
    frames['n'] += 1
    p = WHITE_UNKNOWN_KIND if frames['n'] <= white_first else then
    return paint(**{'left': p, 'far_left': p, 'right': SW, **lines})
  return read


def night_road(lanes_left, lanes_right, width=3.0, far_prob=0.6, curb_prob=0.08, edge_room=None):
  """modelV2 at night on a city road of width-wide lanes: the curb-side (right) ego line unsure (curb_prob: 1e, a median 0.08
  moving), the far-left line at far_prob. edge_room: m from the left ego line to the left road edge, else the road's."""
  line_ys, probs, edges, stds = road(lanes_left, lanes_right, lane_width=width)
  probs[2] = curb_prob
  probs[0] = far_prob if lanes_left else 0.05
  if edge_room is not None:
    edges[0] = line_ys[1] - edge_room
  return line_ys, probs, edges, stds


def test_a_lane_beside_counts_against_the_lanes_measured_width_with_one_ego_line_unsure():
  """model_lanes: the next line out a lane's width beyond, the width the ego lines' spacing when one of them is sure. Before,
  both had to be sure, else LANE_WIDTH: at night the curb side's line is unsure on most frames, and a 3.0 m city lane's far
  line, 3.0 m beyond the ego line, needed 3.06 m (1d 575.9-577.4 s, 1e 364-367 s: the far-left line 2.6-2.8 m out, no lane)."""
  lanes = model_lanes(*night_road(1, 0))
  assert lanes.left == 1 and lanes.left_known and lanes.left_seen and not lanes.right_seen
  assert lanes.width == LANE_WIDTH  # the shoulder, the room and the lane bands as before
  # neither ego line sure, or the spacing no lane's: LANE_WIDTH's bar, as before
  line_ys, probs, edges, stds = night_road(1, 0)
  probs[1] = 0.3
  assert model_lanes(line_ys, probs, edges, stds).left == 0
  line_ys, probs, edges, stds = night_road(1, 0, width=2.2)
  assert model_lanes(line_ys, probs, edges, stds).left == 0, "2.2 m is no lane's spacing"
  # a narrower strip beside the lane is still none: a 7 ft parking strip beside a 10 ft lane is 0.7 of it
  line_ys, probs, edges, stds = night_road(1, 0)
  line_ys[0] = line_ys[1] - 0.7 * 3.0
  assert model_lanes(line_ys, probs, edges, stds).left == 0
  # the spacing never raises the bar: a wide lane measured with one line sure keeps LANE_WIDTH's
  assert model_lanes(*night_road(1, 0, width=4.2)).left == 1


def test_lane_agreed():
  """A line the reader read but couldn't make out is crossed where the model counts a painted lane beyond it and one sign
  says the lane is ours: the line read white, the line beyond read yellow (toward the far side), or the map's lane list."""
  lanes = model_lanes(*night_road(1, 0))
  for p in UNREADABLE:
    assert lane_agreed(LEFT, lanes, paint(left=p), nav_room=1, cleared=True), p
    assert not lane_agreed(LEFT, lanes, paint(left=p), nav_room=0, cleared=True), p
  # the far side's list only over a line its reads cleared (LineMarks): there the model counts the oncoming lane beyond a
  # centre line as readily, and a far_bay list is the junction's (2026-11-07 review)
  for p in (UNKNOWN, COLORLESS, UNSURE_BW):
    assert not lane_agreed(LEFT, lanes, paint(left=p), nav_room=1), p
  curb = model_lanes(*road(0, 1))
  for p in UNREADABLE:  # toward the curb the room is the road's own: the list as it was
    assert lane_agreed(RIGHT, curb, paint(right=p), nav_room=1), p
  assert lane_agreed(LEFT, lanes, paint(left=WHITE_UNKNOWN_KIND)), "white: no centre line"
  assert not lane_agreed(LEFT, lanes, paint(left=LinePaint('unknown', 'white', BROKEN_WHITE_CONF_MIN - 0.1, 0.3)))
  for p in (UNKNOWN, COLORLESS, UNSURE_BW):  # no list, no white: nothing says the lane is ours
    assert not lane_agreed(LEFT, lanes, paint(left=p)), p
  for beyond in (SY, BY, LATCHED_YELLOW):  # the centre line beyond the lane (a bay opened)
    assert lane_agreed(LEFT, lanes, paint(left=UNKNOWN, far_left=beyond)), beyond
  curb = model_lanes(*road(0, 1))
  assert not lane_agreed(RIGHT, curb, paint(right=UNKNOWN, far_right=SY)), "a yellow beyond is the far side's sign only"
  # the rails: never across a contrary read, held yellow included, the list and the model notwithstanding
  for p in CONTRARY + (LATCHED_YELLOW, LinePaint('unknown', 'yellow', 0.1, 0.)):
    assert not lane_agreed(LEFT, lanes, paint(left=p), nav_room=1, cleared=True), p
  # nothing read: no agreement (turnpilotd hands over none unless the reader read the frame)
  assert not lane_agreed(LEFT, lanes, (), nav_room=1, cleared=True)
  # the model must see the ego line and count a painted lane: room to the edge alone is no lane (a divided road's median,
  # a parking strip), nor a line the model doesn't see
  room_only = model_lanes(*night_road(1, 0, far_prob=0.05, edge_room=6.))
  assert room_only.left == 0 and room_only.left_open and not lane_agreed(LEFT, room_only, paint(left=WHITE_UNKNOWN_KIND), nav_room=1)
  line_ys, probs, edges, stds = night_road(1, 0)
  probs[1] = 0.3
  assert not lane_agreed(LEFT, model_lanes(line_ys, probs, edges, stds), paint(left=UNKNOWN), nav_room=1, cleared=True)


def test_change_blocker_crosses_an_unreadable_line_only_when_the_planner_says_unmarked():
  lanes = model_lanes(*night_road(1, 0))
  for p in UNREADABLE:
    assert change_blocker(LEFT, lanes, paint(left=p), nav_room=1) == 'lineNotBroken', p  # as before, off by default
    assert change_blocker(LEFT, lanes, paint(left=p), nav_room=1, unread_ok=True, line_cleared=True) == '', p
  for p in (UNKNOWN, COLORLESS, UNSURE_BW):  # the far side's list over a line never cleared: no agreement
    assert change_blocker(LEFT, lanes, paint(left=p), nav_room=1, unread_ok=True) == 'lineNotBroken', p
  for p in CONTRARY:
    assert change_blocker(LEFT, lanes, paint(left=p), nav_room=1, unread_ok=True, line_cleared=True) == 'lineNotBroken', p
  # the rest of the checks as on broken white
  parked = scene([{'cls': 'car', 'x': 30., 'y': 3.0, 'vx': 0.}]).objects
  assert change_blocker(LEFT, lanes, paint(left=UNKNOWN), parked, nav_room=1, unread_ok=True, line_cleared=True) == 'laneParked'
  assert change_blocker(RIGHT, lanes, paint(right=UNKNOWN), nav_room=1, unread_ok=True, right_hand_traffic=False) == 'noLane'


def test_an_oncoming_car_in_the_target_lane_makes_it_no_lane():
  """oncoming_in_lane: anything coming toward the car in the lane that way, to STOPPED_X_M, whatever the line reads (1e
  364-367 s: cars at 2.7-3.1 m left closing at 3-6 m/s, 7-25 m ahead). Before, broken white with one 60 m out went."""
  lanes = model_lanes(*road(1, 0))
  oncoming = [SimpleNamespace(x=60., y=3.6, vx=-8.)]
  assert oncoming_in_lane(oncoming, LEFT, lanes.width) and not oncoming_in_lane(oncoming, RIGHT, lanes.width)
  assert change_blocker(LEFT, lanes, paint(left=BW)) == ''
  assert change_blocker(LEFT, lanes, paint(left=BW), oncoming) == 'noLane'
  assert change_blocker(LEFT, lanes, paint(left=BW), [SimpleNamespace(x=60., y=3.6, vx=12.)]) == '', "ours, going our way"
  assert change_blocker(LEFT, lanes, paint(left=BW), [SimpleNamespace(x=60., y=7.2, vx=-8.)]) == '', "two lanes over"
  # an offer up aborts on it
  drive = Drive(lanes_left=1, lanes_right=0)
  m = maneuver("turn", "left")
  distance = drive.run(m, 290., LC_CONFIRM_FRAMES * DT_MDL + 0.5)
  assert drive.lp.phase == Phase.WARN
  drive.run(m, distance, DT_MDL, objects=scene([{'cls': 'car', 'x': 70., 'y': 3.6, 'vx': -10.}]).objects)
  assert [a for a, _ in drive.aborts] == ['noLane'] and not drive.requests


def test_line_marks():
  """Three ways: marked by a yellow or solid read, cleared by WHITE_CLEAR_FRAMES white reads since (or the yellow moving out a
  lane from it), or neither. One white read cleared a mark before (2026-11-07 review, 1e 243.35 s: a centre line read white
  0.61 on 2 frames, and lane_agreed crossed it on 4 frames 1.0-1.4 s on)."""
  marks = LineMarks()
  assert marks.marked == marks.cleared == [False] * 4, "from the start: neither"
  marks.update(paint(left=SY, right=BW), False)
  assert marks.marked == [False, True, False, False] and not any(marks.cleared)
  marks.update(paint(left=UNKNOWN), False)
  assert marks.marked[1], "an unread frame keeps it"
  marks.update((), False)
  assert marks.marked[1]
  marks.update(paint(left=LinePaint('broken', 'white', BROKEN_WHITE_CONF_MIN - 0.1, 0.3)), False)
  assert marks.marked[1], "a white read short of the bar doesn't clear it"
  for _ in range(WHITE_CLEAR_FRAMES - 1):
    marks.update(paint(left=WHITE_UNKNOWN_KIND), False)
    marks.update(paint(left=UNKNOWN), False)
  assert marks.marked[1] and not marks.cleared[1], "fewer white reads than WHITE_CLEAR_FRAMES: still marked"
  marks.update(paint(left=WHITE_UNKNOWN_KIND), False)
  assert not marks.marked[1] and marks.cleared[1], "WHITE_CLEAR_FRAMES white reads clear it, gaps between them or not"
  marks.update(paint(left=BY), False)
  assert marks.marked[1] and not marks.cleared[1], "a yellow read marks it again, its white reads counted afresh"
  for _ in range(WHITE_CLEAR_FRAMES - 1):
    marks.update(paint(left=WHITE_UNKNOWN_KIND), False)
  assert marks.marked[1]
  # the yellow moving out a lane (YellowHold.moved_out) clears it at once: a bay opening
  marks.update(paint(left=UNKNOWN, far_left=SY), False, (False, True, False, False))
  assert not marks.marked[1] and marks.cleared[1]
  for p in (SW, LinePaint('solid', 'unknown', 0.3, 0.9), BY, LinePaint('unknown', 'yellow', 0.1, 0.)):
    marks = LineMarks()
    marks.update(paint(left=p), False)
    assert marks.marked[1], p
  marks = LineMarks()
  marks.update(paint(left=SY), True)
  assert not any(marks.marked), "frozen: the lines may be renumbering"
  marks.update(paint(far_left=SY), False)
  marks.shift(LEFT)  # the car moved left: the far-left line is its left ego line now
  assert marks.marked == [False, True, False, False]
  marks = LineMarks()
  for _ in range(WHITE_CLEAR_FRAMES):
    marks.update(paint(far_left=WHITE_UNKNOWN_KIND, left=WHITE_UNKNOWN_KIND), False)
  marks.shift(LEFT)  # a line's reads move with it
  assert marks.cleared == [False, True, True, False]
  marks.update(paint(far_left=SY), False)
  marks.shift(0)
  assert marks.marked == [True] * 4 and not any(marks.cleared), "a change of unknown direction: every line marked, none cleared"


def test_a_centre_line_read_yellow_once_is_never_crossed_unread_after_its_hold():
  """The oncoming case lane_agreed must never take: an undivided road, its yellow centre line read once, then unread for
  longer than the yellow hold, with the map's list wanting a lane left (its bay further on) and the model counting the
  oncoming lane (its edge line painted a lane out). The mark keeps it; the bay latch is no help here (one read)."""
  m = maneuver("turn", "left", LEFT_BAY)
  for second in (UNKNOWN, COLORLESS, UNSURE_BW):
    reads = {'n': 0}

    def centre(left, right, reads=reads, second=second):
      reads['n'] += 1
      return paint(left=SY if reads['n'] == 1 else second, right=UNKNOWN)
    drive = Drive(lanes_left=1, lanes_right=0, paint=centre, driver_signals=True)
    drive.run(m, 380., 30., blinker=None)
    assert not drive.offers and not drive.requests, second
    # the bay opening (1e 305-307 s): the ego line read white after the yellow, and the yellow read a lane out on the frames
    # after, YELLOW_OUT_FRAMES of the two: the yellow moved out, the line cleared, and it goes. One white read alone of a line
    # just read yellow, the list and the model's count of what lies beyond it agreeing, no longer does (2026-11-07 review)
    for opened in (True, False):
      reads = {'n': 0}

      def bay(left, right, reads=reads, second=second, opened=opened):
        reads['n'] += 1
        n = reads['n']
        return paint(left=SY if n == 1 else WHITE_UNKNOWN_KIND if n <= 3 else second, right=UNKNOWN,
                     far_left=SY if opened and n > 3 else UNKNOWN)
      drive = Drive(lanes_left=1, lanes_right=0, paint=bay)
      drive.run(m, 380., 30.)
      assert bool(drive.offers and drive.requests) == opened, (second, opened)


def test_the_yellow_moving_out_a_lane_ends_the_ego_lines_hold():
  """YellowHold: the ego line read yellow, then white, and the line beyond it yellow, YELLOW_OUT_FRAMES frames of the two:
  the centre line now lies a lane out (a turn bay opening: 1d 575.5-576.30 s, 8 white frames and the yellow beyond on 2; 1e
  306.82 s, 1 white frame and the yellow beyond on 33), and the ego line's hold ends, moved_out said for LineMarks. Before,
  it held 100 m, past the window's end; and one frame of each ended it (2026-11-07 review, 1e 243.35-244.30 s)."""
  def run(reads, v=11.7):
    hold = YellowHold()
    out = ()
    for p in reads:
      out = hold.update(p, v, DT_MDL, False)
    return out, hold
  white = [paint(left=WHITE_UNKNOWN_KIND)] * 8
  beyond = [paint(left=UNKNOWN, far_left=BY)] * (YELLOW_OUT_FRAMES - 8)
  white_then_out = [paint(left=SY)] * 5 + [paint(left=UNKNOWN)] * 20 + white + [paint(left=UNKNOWN)] * 5 + beyond
  out, hold = run(white_then_out[:-1])
  assert out[1] == LATCHED_YELLOW, "a frame short of YELLOW_OUT_FRAMES: held"
  assert not hold.moved_out[1]
  out, hold = run(white_then_out)
  assert out[1] == UNKNOWN and out[0] == BY and hold.moved_out[1], "released, the yellow on the line beyond"
  # the 1e bay: one white frame, the yellow beyond read on the frames before it
  out, hold = run([paint(left=SY)] * 5 + [paint(left=NONE_READ, far_left=BY)] * (YELLOW_OUT_FRAMES - 1) +
                  [paint(left=WHITE_UNKNOWN_KIND, far_left=BY)])
  assert out[1] == WHITE_UNKNOWN_KIND and hold.moved_out[1]
  # the yellow beyond alone, no white read of the ego line: held, however long it reads
  assert run([paint(left=SY)] * 5 + [paint(left=UNKNOWN, far_left=BY)] * 3 * YELLOW_OUT_FRAMES)[0][1] == LATCHED_YELLOW
  # the white read too long before the yellow beyond: held
  stale = white_then_out[:-len(beyond) - 5] + [paint(left=UNKNOWN)] * int(WHITE_HOLD_S / DT_MDL + 2) + beyond
  assert run(stale)[0][1] == LATCHED_YELLOW
  # a yellow or solid read of the ego line after the white starts the count over
  for contrary in (SY, LinePaint('solid', 'unknown', 0.5, 0.9), LinePaint('unknown', 'yellow', 0.1, 0.)):
    assert run(white_then_out[:-len(beyond)] + [paint(left=contrary)] + beyond)[0][1] == LATCHED_YELLOW, contrary
  # the yellow beyond is held as any yellow, and moves to the ego line with a change that way
  _, hold = run(white_then_out)
  hold.shift(LEFT)
  assert hold.apply(paint())[1] == LATCHED_YELLOW
  # the right side the same way
  out, hold = run([paint(right=SY)] * 5 + [paint(right=WHITE_UNKNOWN_KIND)] * 3 +
                  [paint(right=UNKNOWN, far_right=BY)] * (YELLOW_OUT_FRAMES - 3))
  assert out[2] == UNKNOWN and hold.moved_out[2]


def test_one_white_read_and_one_yellow_beyond_are_no_bay_opening():
  """2026-11-07 review, 1e 243.35-244.85 s (the seg 5 approach, one lane, the yellow centre line on the left): the ego left
  line read yellow, then 3.3 s on unknown kind white 0.61 on 2 frames, then the far-left line yellow 1.0 on 3 frames among
  unknown ones, the model counting a lane left on some. One white read cleared the mark, one yellow read beyond ended the
  hold, and change_blocker passed left on 4 frames until the line read solid; with that held on, every unknown frame after
  it agreed by the yellow beyond. Now neither: marked and held, nothing passes, nothing is offered."""
  white_061 = LinePaint('unknown', 'white', 0.61, 0.)
  yellow_beyond = paint(left=UNKNOWN, far_left=LinePaint('unknown', 'yellow', 1., 0.))
  frames = [paint(left=SY)] * 2 + [paint(left=UNKNOWN)] * 66 + [paint(left=white_061)] * 2 + [paint(left=UNKNOWN)] * 18 + \
    [yellow_beyond, paint(left=UNKNOWN), yellow_beyond, paint(left=UNKNOWN)] + [paint(left=UNKNOWN)] * 2 + [yellow_beyond] + \
    [paint(left=UNKNOWN)] * 100
  lanes = model_lanes(*night_road(1, 0))
  lp, m = LanePlanner(), maneuver("turn", "left")
  passed = []
  for k, p in enumerate(frames):
    d = lp.update(m, 155. - k * 0.4, lanes, LaneChangeState.off, 8., False, False, (), True, True, DT_MDL, p)
    assert d.offer == LaneChange.none and d.lane_change == LaneChange.none, k
    passed += [k] if d.wait == 'confirming' else []  # change_blocker passed this frame (inside the window)
  assert lp.marks.marked[1] and not lp.marks.cleared[1] and not passed


def test_soft_block():
  lanes = model_lanes(*night_road(1, 0))
  assert soft_block(LEFT, 'lineNotBroken', lanes, paint(left=UNKNOWN))
  assert soft_block(LEFT, 'lineNotBroken', lanes, ())  # nobody read the frame
  for p in CONTRARY + (LATCHED_YELLOW,):
    assert not soft_block(LEFT, 'lineNotBroken', lanes, paint(left=p)), p
  unseen = model_lanes(*night_road(1, 0, far_prob=0.3))  # the far line's prob dipping under LINE_PROB_MIN: count unknown
  assert unseen.left == 0 and not unseen.left_known and soft_block(LEFT, 'noLane', unseen, paint(left=BW))
  at_edge = model_lanes(*road(0, 0))  # the edge beside the line: a known none
  assert at_edge.left_known and not soft_block(LEFT, 'noLane', at_edge, paint(left=BW))
  assert not soft_block(LEFT, 'noLane', unseen, paint(left=BW), nav_room=0)
  assert not soft_block(LEFT, 'noLane', unseen, paint(left=BW), [SimpleNamespace(x=40., y=3.6, vx=-8.)])
  assert not soft_block(RIGHT, 'noLane', model_lanes(*night_road(0, 1, far_prob=0.3)), paint(right=BW), right_hand_traffic=False)
  for blocker in ('laneParked', 'carInLane', ''):
    assert not soft_block(LEFT, blocker, lanes, paint(left=BW)), blocker


def offer_frame(frames, lanes_fn, m=None, blindspots=()):
  """The frame index the planner offers a left on, frames: paint per frame, lanes_fn(k): the model's lanes on frame k."""
  lp = LanePlanner()
  m = m or maneuver("turn", "left")
  for k, p in enumerate(frames):
    d = lp.update(m, 230. - k * 0.6, lanes_fn(k), LaneChangeState.off, 12., False, False, (), True, True, DT_MDL, p,
                  left_blindspot=k in blindspots)
    if d.offer == LaneChange.left:
      return k
  return None


def test_a_soft_frame_inside_a_live_confirm_count_keeps_the_change_pending():
  """Review 2026-11-07 (LC-4): a soft frame's wait is its blocker (lineNotBroken, noLane), not one turnpilotd counts as a
  change pending (LANE_PENDING_WAITS), so with the model's count flickering the wait alternated with 'confirming' frame to
  frame (1d 576.65-577.45 s, a left 119-110 m out), the turn's prompt to signal flickered with it (its chime cut to a blip),
  and a lamp lit for it was the change's go-ahead at the offer's first frame. Such a frame, the count live, is pending
  (confirm_live): turnpilotd's lane_pending never drops between the frames that pass and the offer."""
  from openpilot.selfdrive.turnpilot.lane_planner import Phase
  from openpilot.selfdrive.turnpilot.turnpilotd import LANE_PENDING_WAITS
  seen = model_lanes(*night_road(1, 0))
  line_ys, probs, edges, stds = night_road(1, 0, far_prob=0.3)
  stds[0] = 1.5
  unseen = model_lanes(line_ys, probs, edges, stds)
  lp, m, pending = LanePlanner(), maneuver("turn", "left"), []
  for k in range(40):
    d = lp.update(m, 230. - k * 0.6, seen if k % 2 == 0 else unseen, LaneChangeState.off, 12., False, False, (), True, True, DT_MDL,
                  paint(left=BW))
    pending.append(lp.phase != Phase.IDLE or (d.want != 0 and (d.wait in LANE_PENDING_WAITS or lp.confirm_live)))
    if d.offer == LaneChange.left:
      break
  assert d.offer == LaneChange.left
  first = pending.index(True)
  assert all(pending[first:]), pending  # never a frame between the first that passed and the offer the prompt could show on


def test_the_want_of_a_read_or_a_count_keeps_the_confirm_count():
  """Ten frames that pass by their own read make an offer, frames between them whose blocker is the want of a read or of
  the model's count (soft_block) neither counting nor breaking them, for LC_UNSURE_MAX_S past the last that passed; a
  contrary read starts the count over. Before, any frame that didn't pass did: at night the far line's prob flicked around
  LINE_PROB_MIN and the count never got to ten running (1d 576.4-577.4 s: eight at most)."""
  seen = model_lanes(*night_road(1, 0))
  line_ys, probs, edges, stds = night_road(1, 0, far_prob=0.3)
  stds[0] = 1.5  # the far line under LINE_PROB_MIN, the edge's std over EDGE_STD_MAX (1e 535.67 s): no count, no room
  unseen = model_lanes(line_ys, probs, edges, stds)
  assert not unseen.left_known and not unseen.left_open and change_blocker(LEFT, unseen, paint(left=BW)) == 'noLane'
  # broken white read, the lane counted on every other frame
  flicker = lambda k: seen if k % 2 == 0 else unseen  # noqa: E731
  assert offer_frame([paint(left=BW)] * 40, flicker) == 2 * (LC_CONFIRM_FRAMES - 1)
  # frames nobody read between the reads: kept
  frames = [paint(left=BW)] * 5 + [()] * 20 + [paint(left=BW)] * 20
  assert offer_frame(frames, lambda k: seen) == 25 + LC_CONFIRM_FRAMES - 5 - 1
  # a contrary read starts it over; so does LC_UNSURE_MAX_S with nothing passing
  frames = [paint(left=BW)] * 5 + [paint(left=SW)] + [paint(left=BW)] * 20
  assert offer_frame(frames, lambda k: seen) == 6 + LC_CONFIRM_FRAMES - 1
  gap = int(round(LC_UNSURE_MAX_S / DT_MDL)) + 1
  frames = [paint(left=BW)] * 5 + [()] * gap + [paint(left=BW)] * 20
  assert offer_frame(frames, lambda k: seen) == 5 + gap + LC_CONFIRM_FRAMES - 1
  # never offered on a soft frame, the count made: the next frame that passes
  frames = [paint(left=BW)] * (LC_CONFIRM_FRAMES - 1) + [()] * 5 + [paint(left=BW)]
  assert offer_frame(frames, lambda k: seen) == LC_CONFIRM_FRAMES + 4


THREE_LANES_RIGHT_TURN = ((("straight",), False), (("straight",), False), (("right",), True))


def test_an_unreadable_line_goes_where_the_map_and_the_model_agree():
  """The 1e/1d night: the ego line read but not made out, the map's lane list wanting the car two lanes over, the model
  counting the lanes: both changes offered and made. Before, lineNotBroken on every frame: nothing offered. Toward the curb
  the list is the sign as it was; toward the far side only over lines whose own reads cleared them (LineMarks: white on
  WHITE_CLEAR_FRAMES frames, here at the drive's start before the window), never over one never read white (2026-11-07
  review: test_the_far_sides_list_alone_never_crosses_a_line_never_read_white)."""
  m = maneuver("turn", "right", THREE_LANES_RIGHT_TURN)
  for p in UNREADABLE:
    drive = Drive(lanes_left=0, lanes_right=2, paint=lambda left, right, p=p: paint(left=p if left else SW, right=p if right else SW))
    drive.run(m, 500., 40.)
    assert [r for r, _, _ in drive.requests] == [LaneChange.right] * 2 and drive.lp.changes == 2, p
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  for p in UNREADABLE:
    drive = Drive(lanes_left=2, lanes_right=0, paint=night_reads(then=p))
    drive.run(m, 500., 40.)
    assert [r for r, _, _ in drive.requests] == [LaneChange.left] * 2 and drive.lp.changes == 2, p
    if p in (UNKNOWN, COLORLESS, UNSURE_BW):  # nothing read white: the list alone is no sign toward the far side
      drive = Drive(lanes_left=2, lanes_right=0, paint=lambda left, right, p=p: paint(left=p, right=SW))
      drive.run(m, 500., 40.)
      assert not drive.offers, p
  # with no list: only a white read says the lane is ours
  for p, goes in ((WHITE_UNKNOWN_KIND, True), (UNKNOWN, False), (COLORLESS, False)):
    drive = Drive(lanes_left=1, lanes_right=0, paint=lambda left, right, p=p: paint(left=p, right=SW))
    drive.run(maneuver("turn", "left"), 400., 30.)
    assert bool(drive.requests) == goes, p


def respond(drive, react_s=1.0):
  """The driver: the blinker the prompt asks for, react_s after it, until the change starts (the stalk let go then)."""
  prompt = drive.prompts[-1] if drive.prompts else LaneChange.none
  starting = drive.dh.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
  since = drive._prompt_since if prompt != LaneChange.none and prompt == getattr(drive, '_prompt_side', None) else 0.
  drive._prompt_side, drive._prompt_since = prompt, since + DT_MDL
  return prompt if prompt != LaneChange.none and since >= react_s and not starting else None


def test_two_changes_the_same_way_each_get_their_own_prompt_and_go():
  """Task 3 (2026-11-07): where the driver signals (the Tesla can't light its lamps), two lanes over before a left: each
  change is offered, its side prompted (the chime per prompt), the driver signals it and it goes; the second gets its own
  prompt after the first's cooldown, not the first's stalk carried over. Night lanes (the curb line unsure, 3.0 m lanes);
  the lines read broken white, or read white early on and unknown after (the map's list the sign over them: LineMarks)."""
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  for p in ('broken white', 'white, then unknown'):
    reads = (lambda left, right: paint(left=BW, right=SW)) if p == 'broken white' else night_reads(then=UNKNOWN)
    drive = Drive(lanes_left=2, lanes_right=0, driver_signals=True, paint=reads)
    distance = 500.
    for _ in range(int(40. / DT_MDL)):
      drive.step(m, distance, blinker=respond(drive))
      distance -= drive.v * DT_MDL
    rising = [k for k, (a, b) in enumerate(zip([LaneChange.none] + drive.prompts, drive.prompts, strict=False))
              if b == LaneChange.left and a != LaneChange.left]
    assert len(rising) == 2 and len(drive.offers) == 2, p
    assert [r for r, _, _ in drive.requests] == [LaneChange.left] * 2 and drive.lp.changes == 2 and not drive.aborts, p
    assert (drive.lanes_left, drive.lanes_right) == (0, 2), p
    # each request on the driver's signal, after its own prompt: none before it
    for k, (_, _, t_req) in zip(rising, drive.requests, strict=True):
      assert t_req > k * DT_MDL + 0.9, p
    start, end = lane_change_window(drive.v, 2)
    assert all(end < d < start for _, d, _ in drive.offers), p


def test_the_driver_holding_the_stalk_through_both_changes_is_asked_for_each():
  """The driver holds the stalk on from the first change through the second (a human's two-lane move): the second is still
  offered and prompted on its own, and goes on that same stalk as the countdown's go-ahead (WARN's: the driver's blinker
  any time in the countdown), never before its offer."""
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  drive = Drive(lanes_left=2, lanes_right=0, driver_signals=True)
  distance, held = 500., False
  for _ in range(int(40. / DT_MDL)):
    held = held or (drive.prompts and drive.prompts[-1] == LaneChange.left)
    drive.step(m, distance, blinker=LaneChange.left if held else None)
    distance -= drive.v * DT_MDL
  assert len(drive.offers) == 2 and len(drive.requests) == 2 and drive.lp.changes == 2 and not drive.aborts
  for (_, _, t_offer), (_, _, t_req) in zip(drive.offers, drive.requests, strict=True):
    assert 0. <= t_req - t_offer < LC_WARN_S


def test_an_unread_wait_says_why_and_never_requests():
  """An offer up with the paint unread (the wide camera): it waits, the prompt kept, nothing requested on an unread frame,
  even with the driver signalling; it goes on the first frame read broken white."""
  drive = Drive(lanes_left=1, lanes_right=0, driver_signals=True)
  m = maneuver("turn", "left")
  distance = drive.run(m, 290., LC_CONFIRM_FRAMES * DT_MDL + 0.5)
  assert drive.lp.phase == Phase.WARN
  drive.paint = None
  distance = drive.run(m, distance, 2., blinker=LaneChange.left)
  assert not drive.requests and not drive.aborts and drive.prompts[-1] == LaneChange.left
  from openpilot.selfdrive.turnpilot.tests.test_lane_planner import striped
  drive.paint = striped
  drive.run(m, distance, 5., blinker=LaneChange.left)
  assert len(drive.requests) == 1 and drive.lp.changes == 1
  # waiting on a line unread or not made out, the lane ends (the road edge the model is sure of comes in beside the line):
  # aborted noLane at once, as on a frame read broken white, never a silent wait
  for p in (None, lambda left, right: paint(left=UNKNOWN, right=SW)):
    drive = Drive(lanes_left=1, lanes_right=0, driver_signals=True)
    distance = drive.run(m, 290., LC_CONFIRM_FRAMES * DT_MDL + 0.5)
    drive.paint, drive.lanes_left = p, 0
    drive.run(m, distance, DT_MDL)
    assert [a for a, _ in drive.aborts] == ['noLane'] and not drive.requests, p


def test_a_frame_nobody_read_is_no_agreement_whatever_hold_pads_it():
  """2026-11-07 review: with any yellow hold live, YellowHold pads a frame nobody read to four lines (the held one yellow, the
  rest unknown), and lane_agreed took the padded ego line for one read but not made out: a held far-left yellow (its sign
  beyond) or the map's list agreed every such frame. It counted toward the confirm, carried the offer and the request.
  Now lane_agreed is asked only on paint read (as curb_lane_unpainted): such a frame waits, nothing offered or requested."""
  # the far side, no list: one frame read (the ego line white, the far-left yellow), then nothing read. Before: offered 0.45
  # s on, requested 3.5 s on, on a frame with no paint at all
  frames = {'n': 0}

  def once(left, right):
    frames['n'] += 1
    return paint(left=WHITE_UNKNOWN_KIND, far_left=SY) if frames['n'] == 1 else ()
  drive = Drive(lanes_left=1, lanes_right=0, paint=once)
  drive.run(maneuver("turn", "left"), 290., 8.)
  assert not drive.offers and not drive.requests
  # the curb side with a list: the right ego line never read, the centre line read yellow 1 frame in 90 (its hold pads the
  # rest). Before: the right offered and requested on unread frames; with nothing ever read, nothing (as now, both ways)
  frames['n'] = 0

  def centre(left, right):
    frames['n'] += 1
    return paint(left=SY, right=UNKNOWN) if frames['n'] % 90 == 1 else ()
  drive = Drive(lanes_left=0, lanes_right=1, paint=centre)
  drive.run(maneuver("turn", "right", TWO_LANES_RIGHT_TURN), 300., 12.)
  assert not drive.offers and not drive.requests
  # an offer up on broken white with the yellow beyond (a bay), then the paint gone (the wide camera): the held yellow beyond
  # pads every frame. It waits, nothing requested, and LC_UNSURE_MAX_S on aborts lineNotBroken, said
  drive = Drive(lanes_left=1, lanes_right=0, paint=lambda left, right: paint(left=BW, far_left=SY, right=SW))
  m = maneuver("turn", "left")
  distance = drive.run(m, 290., LC_CONFIRM_FRAMES * DT_MDL + 0.2)
  assert drive.lp.phase == Phase.WARN
  drive.paint = None
  drive.run(m, distance, LC_UNSURE_MAX_S + 1.)
  assert not drive.requests and [a for a, _ in drive.aborts] == ['lineNotBroken']


def test_the_far_sides_list_alone_never_crosses_a_line_never_read_white():
  """2026-11-07 review: a far_bay list ([left (active), straight], its bay further on) places the car a lane short of the
  turn; on the approach the model counts the oncoming lane beyond the centre line, which reads neither yellow nor white
  (worn, or its colour unknown at night). Before: offered left at the window's start (293 m), requested 247 m out on the
  car's own lamp, prompted 222 frames with the driver signalling. Now the far side's list is a sign only over a line its
  reads cleared (LineMarks: white on WHITE_CLEAR_FRAMES frames, or the yellow moved out a lane)."""
  m = maneuver("turn", "left", LEFT_BAY)
  for p in (UNKNOWN, COLORLESS, UNSURE_BW):
    for driver_signals in (False, True):
      drive = Drive(lanes_left=1, lanes_right=0, driver_signals=driver_signals,
                    paint=lambda left, right, p=p: paint(left=p, right=SW))
      drive.run(m, 400., 25.)
      assert not drive.offers and not drive.requests and all(q == LaneChange.none for q in drive.prompts), (p, driver_signals)
  # the same list over a line read white (a lane of ours beside the car), then unknown: it goes
  drive = Drive(lanes_left=1, lanes_right=0, paint=night_reads(then=UNKNOWN))
  drive.run(m, 400., 25.)
  assert drive.offers and drive.requests


def test_a_line_cleared_on_the_road_before_never_clears_the_next_roads_centre_line():
  """Review 2026-11-07 (LC-1): LineMarks.cleared never aged nor reset, so a clear earned by one road's line (2 s of broken
  white, or the junction's own paint read unknown-kind white on 10 frames: 1d 370.75 s, carried through the 374.3 s turn)
  carried onto the next road's centre line, and the far side's list alone agreed a change over it read colourless or unknown:
  offered at 293 m, requested at 292 m (the driver signalling) or 247 m (the car's lamp), the car into the oncoming lane.
  A new maneuver clears nothing: no offer, no request, no prompt, whoever signals; a fresh clear on the new road still goes."""
  m = maneuver("turn", "left", LEFT_BAY)
  uw = LinePaint('unknown', 'white', 0.7, 0.6)
  for prior, prior_s in ((BW, 2.), (uw, 0.6)):
    for centre in (UNKNOWN, COLORLESS):
      for driver_signals in (False, True):
        st = {'p': prior}
        drive = Drive(lanes_left=1, lanes_right=0, driver_signals=driver_signals, paint=lambda left, right, st=st: paint(left=st['p'], right=SW))
        drive.run(None, 0., prior_s)  # the road before, no maneuver in hand
        assert drive.lp.marks.cleared[1], (prior, centre)  # cleared there, as before
        st['p'] = centre
        d, blink = 400., None
        for _ in range(int(round(25. / DT_MDL))):
          dec = drive.step(m, d, blinker=blink, maneuver_id=1)
          if dec.prompt != LaneChange.none and driver_signals:
            blink = LaneChange.left  # the driver answers the prompt
          d -= drive.v * DT_MDL
        case = (prior, centre, driver_signals)
        assert not drive.offers and not drive.requests and drive.lp.changes == 0, case
        assert all(q == LaneChange.none for q in drive.prompts), case
  # the new road's own lane line read white clears it again: the far side's list agrees and the change goes
  st = {'p': BW}
  drive = Drive(lanes_left=1, lanes_right=0, paint=lambda left, right: paint(left=st['p'], right=SW))
  drive.run(None, 0., 2.)
  drive.run(m, 400., 25., maneuver_id=1)
  assert drive.offers and drive.requests


def test_a_parking_strip_never_places_the_car_for_lc03():
  """LC-03 without a lane list: the room beyond the curb-side line read broken white, the line beyond unseen, and parked cars
  in that room now and then. Before, the room between them placed the car (ten frames running), and at the window's end
  the right was skipped "Can't Reach Turn Lane" (1e 722-737 s, HEAD replay: blocked). Now a stopped car seen there on any
  frame of the window undoes a placement by room; one by a painted lane counted stands (stopped traffic in a real lane)."""
  m = maneuver("turn", "right")
  for parked_every, blocked in ((None, True), (40, False)):
    drive = Drive(lanes_left=0, lanes_right=1, night=True)
    distance, k = 300., 0
    while distance > 30.:
      k += 1
      objs = scene([{'cls': 'car', 'x': 25., 'y': -3.6, 'vx': 0.}]).objects if parked_every and k % parked_every < 4 else ()
      drive.step(m, distance, objects=objs, blindspot=LaneChange.right)  # held to the end: never goes
      distance -= drive.v * DT_MDL
    assert drive.decisions[-1].blocked == blocked, parked_every
  # a painted lane counted (by day): placed, a stopped car in it or not
  drive = Drive(lanes_left=0, lanes_right=1)
  distance, k = 300., 0
  while distance > 30.:
    k += 1
    objs = scene([{'cls': 'car', 'x': 25., 'y': -3.6, 'vx': 0.}]).objects if k % 40 < 4 else ()
    drive.step(m, distance, objects=objs, blindspot=LaneChange.right)
    distance -= drive.v * DT_MDL
  assert drive.decisions[-1].blocked


def replay_bay(name, react_s=1.0):
  """A logged night bay approach (lane_bay_night.json) through the planner, the driver signalling each prompt react_s after
  it (and his own logged blinker): (decisions with their t, the planner)."""
  drive = BAY[name]
  kind, side, mtype, modifier, angle, lanes = drive['maneuver']
  m = Maneuver(kind, side, mtype, modifier, angle, tuple((tuple(d), a) for d, a in lanes), geometry=True)
  lp, out, since, side = LanePlanner(), [], 0., LaneChange.none
  for t, md, v, lb, rb, lcs, ly, lpr, ey, es, pnt, bs, objs in drive['rows']:
    state = getattr(LaneChangeState, lcs)
    pt = tuple(LinePaint(k, c, conf, 0.) for k, c, conf in pnt) if pnt is not None else ()
    left = bool(lb) or (side == LaneChange.left and since >= react_s)
    right = bool(rb) or (side == LaneChange.right and since >= react_s)
    d = lp.update(m, md, model_lanes(ly, lpr, ey, es), state, v, left, right,
                  tuple(SimpleNamespace(x=x, y=y, vx=vx) for x, y, vx in objs), True, True, DT_MDL, pt,
                  left_blindspot=bool(bs[0]), right_blindspot=bool(bs[1]), maneuver_id=0)
    since = since + DT_MDL if d.prompt == side and d.prompt != LaneChange.none else 0.
    side = d.prompt
    out.append((t, md, d))
  return out, lp


def test_the_1d_left_bay_at_night_is_offered_and_requested():
  """1d (2026-11-06, after dark) 564-580.1 s: a left, its list [left, straight/right]; the bay opened about 127 m out, the
  centre line's yellow moving to the far-left line, the ego line read white then nothing; the driver signalled at 577.7 s
  and changed lanes himself at 580.2 s. HEAD: no offer (lineNotBroken and noLane throughout, the yellow hold to 582.5 s).
  Now: offered and requested before the driver's own change, inside the window, nothing aborted."""
  out, lp = replay_bay('1d_left_bay')
  offers = [(t, md) for t, md, d in out if d.offer == LaneChange.left]
  requests = [(t, md) for t, md, d in out if d.lane_change == LaneChange.left]
  assert offers and requests, "no offer: the 11-06 night"
  assert offers[0][0] > 575.4, "nothing before the bay opened: the centre line beside the car"
  _, end = lane_change_window(11.)
  assert requests[0][1] > end and requests[0][0] < 580.1
  assert not [d.abort for _, _, d in out if d.abort != 'none']


def test_the_1e_left_bay_at_night_is_offered_after_the_yellow_moves_out():
  """1e (2026-11-06, after dark) 298-311 s: a slight left, its list [left (active), straight, straight]; the ego left line
  read yellow to 303 s, no line, then white (306.9 s) and broken of unknown colour, the far-left line yellow 2.5-3.3 m out
  from 305 s. HEAD: no offer (lineNotBroken). Now: offered once the yellow lies beyond the lane, requested, never while the
  ego line was the centre line."""
  out, lp = replay_bay('1e_left_bay')
  offers = [t for t, _, d in out if d.offer == LaneChange.left]
  requests = [t for t, _, d in out if d.lane_change == LaneChange.left]
  assert offers and requests and offers[0] > 306.8 and requests[0] > offers[0]


def test_the_night_bays_want_no_change_while_shut():
  """The bay rule kept (desired_changes, BayLatch): once the centre line's reads shut the latch (BAY_YELLOW_READS of them),
  the plan wants no change until it opens: 1d on the ego line's white read as the bay opened (575.5 s); 1e at 304.2 s, as
  the lane widened 4.2-5.2 m into the bay and the model's left line jumped back in, a line crossed by LineWatch (the lane
  state reset, as on HEAD: its replay over the whole drive opened it at 302.15 s the same way), nothing offered on it."""
  for name, opened in (('1d_left_bay', 575.4), ('1e_left_bay', 304.1)):
    out, _ = replay_bay(name)
    shut = next(t for t, _, d in out if d.want == 0)
    assert shut < opened - 5. and all(d.want == 0 for t, _, d in out if shut <= t < opened), name


def test_a_far_side_unreadable_line_with_the_line_beyond_unseen_waits():
  """The far side, no list, the line unreadable and nothing beyond seen (the usual night read): no change; the room to the
  edge is no lane of ours by itself (a divided road's median, the oncoming lanes)."""
  drive = Drive(lanes_left=1, lanes_right=0, night=True, paint=lambda left, right: paint(left=UNKNOWN, right=SW))
  drive.run(maneuver("turn", "left"), 400., 30.)
  assert not drive.offers


def night_model(lanes_left, lanes_right, width=3.0, curb_prob=0.08):
  """turnpilotd Sim's modelV2 at night: the curb-side ego line unsure (1e, a median 0.08 moving), the lanes width wide."""
  from openpilot.selfdrive.turnpilot.tests.synthetic import model_v2

  def model(sim):
    b = model_v2(lanes_left, lanes_right, lane_width=width).as_builder()
    probs = list(b.laneLineProbs)
    b.laneLineProbs = probs[:2] + [curb_prob, probs[3]]
    return b.as_reader()
  return model


def test_through_turnpilotd_a_night_left_two_lanes_over_is_prompted_and_goes_on_the_drivers_signal():
  """The plan end to end (turnpilotd, the Sim): the map's list [left, straight, straight/right], the model at night, the left
  lines read white for a second well before the window (the list's sign over them toward the far side: LineMarks) and
  unknown after, the driver signalling. The plan offers the left with its prompt, and the driver's stalk is the request.
  Before (HEAD): lineNotBroken, no offer, no prompt. Never read white, the far side's list alone offers nothing (2026-11-07
  review)."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import THREE_LANES_LEFT_TURN as LIST, Sim, rising
  never = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=12., nav_lanes={0: LIST}, model_lanes=(2, 0),
              driver_signals=True, paint=(UNKNOWN, UNKNOWN, SW, UNKNOWN))
  never.model = night_model(2, 0)
  never.run(20.)
  assert not rising(never.trace, 'laneChangeOffer', LaneChange.left) and not rising(never.trace, 'signalPrompt', LaneChange.left)
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=12., nav_lanes={0: LIST}, model_lanes=(2, 0),
            driver_signals=True, paint=(WHITE_UNKNOWN_KIND, WHITE_UNKNOWN_KIND, SW, UNKNOWN))
  sim.model = night_model(2, 0)
  sim.run(1.)
  sim.paint = (UNKNOWN, UNKNOWN, SW, UNKNOWN)
  sim.run(19.)
  offers, prompts = rising(sim.trace, 'laneChangeOffer', LaneChange.left), rising(sim.trace, 'signalPrompt', LaneChange.left)
  assert offers and prompts and offers[0] == prompts[0]
  start, _ = lane_change_window(12., 2)
  assert offers[0] > 500. - start - 10.
  assert all(p.laneChange == LaneChange.none for _, _, p in sim.trace), "nothing until the driver signals"
  plan = sim.step(blinker=True)
  assert plan.laneChange == LaneChange.left and plan.laneChangeAbort == 'none'
  # nothing read at all (the detector's paint never comes): no offer, the night or not
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=12., nav_lanes={0: LIST}, model_lanes=(2, 0), driver_signals=True)
  sim.model = night_model(2, 0)
  sim.run(25.)
  assert not rising(sim.trace, 'laneChangeOffer', LaneChange.left)


def test_through_turnpilotd_paint_lost_under_an_offer_waits_then_says_why():
  """test_turnpilotd's paint-lost case as it now goes: the offer up, the paint gone (the wide camera), nothing requested and
  nothing aborted until LC_UNSURE_MAX_S with nothing read, then lineNotBroken, said on the plan for selfdrived's alert."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import BROKEN_WHITE_LEFT, Sim
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., model_lanes=(1, 0), paint=BROKEN_WHITE_LEFT)
  while not sim.trace or sim.trace[-1][2].laneChangeOffer == LaneChange.none:
    sim.step()
  sim.paint = ()
  waited = []
  for _ in range(int((LC_UNSURE_MAX_S + 1.) / DT_MDL)):
    plan = sim.step()
    if plan.laneChangeAbort != 'none':
      break
    waited.append(plan)
  assert all(p.laneChangeOffer == LaneChange.left and p.laneChange == LaneChange.none for p in waited)
  assert len(waited) * DT_MDL >= LC_UNSURE_MAX_S - 0.1
  assert plan.laneChangeAbort == 'lineNotBroken' and plan.laneChangeOffer == LaneChange.none and plan.laneChange == LaneChange.none
