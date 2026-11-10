import json
import math
from pathlib import Path

import openpilot.cereal.messaging as messaging
import pytest
from openpilot.cereal import custom, log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.turnpilot.hooks import ABORT_HOLD_S
from openpilot.selfdrive.turnpilot.lane_planner import (LANE_UNKNOWN_REASON, LANE_WIDTH, LATCHED_YELLOW, LC_HOLD_CLEAR_S, LC_RELEASE_S,
                                                        LC_ABORT_COOLDOWN_S, LC_ABORT_SETTLE_S, LC_CONFIRM_FRAMES, LC_COOLDOWN_S,
                                                        LC_REQUEST_TIMEOUT_S,
                                                        LC_UNSURE_MAX_S, LC_WARN_S, YELLOW_HOLD_S, LanePlanner,
                                                        ModelLanes, Phase, offer_room_s,
                                                        slow_zone, BROKEN_WHITE_CONF_MIN, LINE_UNSURE_REASON, WHITE_HOLD_M, WHITE_HOLD_S,
                                                        BAY_CONTEST_S, BAY_YELLOW_CONF_MIN, BAY_YELLOW_READS, YELLOW_HOLD_M,
                                                        BayLatch, WhiteHold, far_bay, lanes_toward, line_contrary,
                                                        YellowHold, change_blocker, desired_changes, ego_lane_index, lane_change_window,
                                                        lane_mismatch,
                                                        lane_changes_needed, model_lanes, model_lanes_from_msg, nav_lane_span, stopped_in_lane,
                                                        target_lane_blocked, keep_blocker)
from openpilot.selfdrive.turnpilot.route_follower import LEFT, NOMINAL_ANGLE_DEG, RIGHT, Maneuver, classify
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint
from openpilot.selfdrive.turnpilot.tests.synthetic import car_state, model_v2, road, scene
LC_SIGNAL_TIMEOUT_S = 8.  # WAIT's own clock before 2026-11-08 (route 29): now the wait lasts to the window's end

LaneChange = custom.TurnpilotPlan.LaneChange
THREE_LANES_LEFT_TURN = ((("left",), True), (("straight",), False), (("straight", "right"), False))
TWO_LANES_RIGHT_TURN = ((("straight",), False), (("right",), True))
BW, SW = LinePaint('broken', 'white', 0.9, 0.3), LinePaint('solid', 'white', 0.9, 0.95)
SY, BY = LinePaint('solid', 'yellow', 0.9, 0.95), LinePaint('broken', 'yellow', 0.5, 0.3)
NO_LINE = LinePaint('none', 'unknown', 1., 0.)


def paint(left=UNKNOWN, right=UNKNOWN, far_left=UNKNOWN, far_right=UNKNOWN):
  return far_left, left, right, far_right


def counts(lanes):
  return lanes.left, lanes.right


def maneuver(mtype, modifier, lanes=()):
  return Maneuver(*classify(mtype, modifier), mtype, modifier, NOMINAL_ANGLE_DEG.get(modifier, 0.), lanes)


def test_model_lanes_counts_room_over_lane_width():
  assert model_lanes(*road(2, 0)) == ModelLanes(2, 0, True, False)
  assert counts(model_lanes(*road(1, 1))) == (1, 1)
  # a wide shoulder with no painted line beyond it is not a lane, but leaves room to keep toward
  lanes = model_lanes(*road(0, 0, shoulder=3.))
  assert (lanes.left, lanes.right, lanes.keep_left_ok, lanes.keep_right_ok) == (0, 0, True, True)
  # an unsure road edge: paint alone says one lane
  assert counts(model_lanes(*road(2, 0, edge_std=2.))) == (1, 0)
  # no ego line on a side: no keep that way
  line_ys, probs, edges, stds = road(1, 1)
  assert not model_lanes(line_ys, [probs[0], 0.1, probs[2], probs[3]], edges, stds).keep_left_ok


def test_a_keep_toward_the_far_side_needs_a_lane_going_our_way_there():
  """BUILD_FINDINGS D5 (smoke S19): a keep toward the far side (left where traffic keeps right) went on the ego line alone
  with room beyond it, and on an undivided road that room is the oncoming lane: keepLeft at 19 m/s put the car 2.5 m over
  the centre line. Toward the far side a keep needs a lane counted beyond the ego line (its next line out painted, a
  same-direction lane) and that ego line not read yellow, now or held (YellowHold); toward the curb side it is as before."""
  undivided = model_lanes(*road(0, 1, shoulder=4.5))  # the left ego line seen, no line beyond it, the far edge 4.5 m out
  assert undivided.keep_left_ok and undivided.left == 0, "the old gate: the room beyond is enough"
  assert keep_blocker(undivided, LEFT) == 'keepFarSide'
  assert keep_blocker(undivided, LEFT, paint=paint(left=BW)) == 'keepFarSide', "no lane counted beyond it: no keep, whatever the paint"
  two_lanes = model_lanes(*road(1, 0))
  assert keep_blocker(two_lanes, LEFT, paint=paint(left=BW)) == ''
  assert keep_blocker(two_lanes, LEFT, paint=paint(left=BW, far_left=SY)) == '', "a lane ours, then a divided road's left edge"
  for yellow in (SY, BY, LinePaint('unknown', 'yellow', 0.35, 0.)):
    assert keep_blocker(two_lanes, LEFT, paint=paint(left=yellow)) == 'keepFarSide', yellow  # oncoming beyond it
  hold = YellowHold()
  hold.update(paint(left=SY), 15., 0.05, False)
  held = hold.update(paint(), 15., 0.05, False)  # worn the next frame: still yellow for the hold
  assert keep_blocker(two_lanes, LEFT, paint=held) == 'keepFarSide'
  assert hold.apply(paint()) == held and hold.update(paint(), 15., 0.05, False) == held, "apply reads the holds without counting a frame"
  # the model's count alone is no lane of ours (skeptic SIMFIX D5): an undivided road whose oncoming lane has its outer edge
  # line painted counts a lane to the left, so the paint unread, the ego line's colour unknown or anything but broken white
  # there is no keep, said
  for unread in ((), paint(), (UNKNOWN,) * 4, paint(left=SW), paint(left=LinePaint('broken', 'unknown', 0.9, 0.3)),
                 paint(left=LinePaint('broken', 'white', 0.2, 0.3))):
    assert keep_blocker(two_lanes, LEFT, paint=unread) == 'keepFarSide', unread
  # no line that way is said as before, first
  line_ys, probs, edges, stds = road(1, 1)
  assert keep_blocker(model_lanes(line_ys, [probs[0], 0.1, probs[2], probs[3]], edges, stds), LEFT) == 'keepNoLine'
  # toward the curb side the room is the road's own: a keep right with no lane beyond goes (an exit's, a fork's)
  curb = model_lanes(*road(1, 0, shoulder=3.))
  assert curb.right == 0 and keep_blocker(curb, RIGHT) == '' and keep_blocker(curb, RIGHT, paint=paint(right=SY)) == ''
  # traffic keeping left: the far side is the right
  assert keep_blocker(model_lanes(*road(1, 0, shoulder=4.5)), RIGHT, right_hand_traffic=False) == 'keepFarSide'
  assert keep_blocker(undivided, LEFT, right_hand_traffic=False) == ''


def test_no_keep_left_on_s19s_two_way_road():
  """S19's reroute slight left (tests/keep_far_side.json, every frame from the reroute to 5 m past its point): keepLeft went out
  at 10.76 s with the left ego line read at 0.62 and no line beyond it (lanes 0, the far edge 4.3 m out); the far side's
  rule never lets it go there, on the lanes alone (the paint read no yellow on that line after the reroute)."""
  data = json.loads((Path(__file__).parent / 'keep_far_side.json').read_text())['s19_too_fast_right_smoke']['rows']
  went = [r for r in data if r[3] == log.Desire.keepLeft]
  assert went and went[0][0] == 10.76, "the fixture's keepLeft as it went out"
  for t, _v, _d, _des, pnt, ys, probs, eys, stds in data:
    lanes = model_lanes(ys, probs, eys, stds)
    lp = tuple(LinePaint(k, c, conf, 0.) for k, c, conf in pnt)
    assert not any(q.color == 'yellow' for q in lp), t
    assert keep_blocker(lanes, LEFT, paint=lp) != '', (t, lanes)
    assert keep_blocker(lanes, LEFT, paint=()) != '', (t, lanes)  # nor with no paint read at all


def test_model_lanes_from_msg():
  assert counts(model_lanes_from_msg(model_v2(2, 1))) == (2, 1)
  assert model_lanes_from_msg(messaging.new_message('modelV2').modelV2) == ModelLanes()  # nothing from modeld yet


UNSURE_BW = LinePaint('broken', 'white', 0.4, 0.3)
COLORLESS = LinePaint('broken', 'unknown', 0.9, 0.3)
NOT_BW = (SW, SY, BY, NO_LINE, UNKNOWN, UNSURE_BW, COLORLESS, LinePaint('unknown', 'white', 0.9, 0.3))
LaneChangeState = log.LaneChangeState


def striped(lanes_left, lanes_right):
  """A multi-lane road: broken white between the travel lanes, solid white at the edges."""
  return paint(left=BW if lanes_left else SW, right=BW if lanes_right else SW)


def test_desired_changes():
  left = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  assert desired_changes(left, model_lanes(*road(2, 0))) == 2
  assert desired_changes(left, model_lanes(*road(1, 1))) == 1
  assert desired_changes(left, model_lanes(*road(0, 2))) == 0
  assert desired_changes(maneuver("turn", "right", TWO_LANES_RIGHT_TURN), model_lanes(*road(0, 1))) == -1
  # no map lanes: one toward a turn's side at a time; an exit to the curb by the model's count, not across solid white
  assert desired_changes(maneuver("turn", "left"), model_lanes(*road(2, 0))) == 1
  assert desired_changes(maneuver("turn", "right"), model_lanes(*road(0, 2))) == -1
  assert desired_changes(maneuver("off ramp", "slight right"), model_lanes(*road(0, 2))) == -2
  assert desired_changes(maneuver("off ramp", "slight right"), model_lanes(*road(0, 2)), paint=paint(right=SW)) == 0
  assert desired_changes(maneuver("off ramp", "slight right"), model_lanes(*road(0, 2)), paint=paint(right=BW, far_right=SW)) == -1
  assert desired_changes(maneuver("fork", "slight right"), model_lanes(*road(0, 1))) == 0
  assert desired_changes(None, model_lanes(*road(1, 1))) == 0


def test_every_change_needs_broken_white_into_a_travel_lane():
  lanes = model_lanes(*road(1, 1))
  assert change_blocker(LEFT, lanes, paint(left=BW)) == '' and change_blocker(RIGHT, lanes, paint(right=BW)) == ''
  for p in NOT_BW:  # solid, yellow, no line, unknown, unsure, colorless: no change
    assert change_blocker(LEFT, lanes, paint(left=p)) in ('lineNotBroken', 'noLane'), p
  assert change_blocker(LEFT, lanes, ()) == 'lineNotBroken'  # no paint read at all (no detector, wide camera, stale scene)
  # the paint the other way doesn't count: no left/right mixup
  assert change_blocker(RIGHT, lanes, paint(left=BW, right=SW)) == 'lineNotBroken'
  assert change_blocker(LEFT, lanes, paint(left=SW, right=BW)) == 'lineNotBroken'
  # broken white, but the model sees no lane that way (a bike lane's line turns broken before a junction)
  assert change_blocker(RIGHT, model_lanes(*road(1, 0)), paint(right=BW)) == 'noLane'
  # a worn solid edge line read broken white beside a shoulder or a bike lane: the model sees no lane there
  assert change_blocker(RIGHT, model_lanes(*road(0, 0, shoulder=2.)), paint(right=BW)) == 'noLane'
  line_ys, probs, edges, stds = road(0, 1)
  line_ys[3] = line_ys[2] + 0.7 * 3.6  # the next line out is a bike lane's width away, not a lane's
  assert change_blocker(RIGHT, model_lanes(line_ys, probs, edges, stds), paint(right=BW)) == 'noLane'
  # parked cars ahead in that lane: a parking strip
  parked = scene([{'cls': 'car', 'x': 30., 'y': -3.6, 'vx': 0.}]).objects
  assert change_blocker(RIGHT, lanes, paint(right=BW), parked) == 'laneParked'
  assert change_blocker(LEFT, lanes, paint(left=BW), parked) == ''
  # left-hand traffic: never away from the curb (the center line is broken white there)
  assert change_blocker(RIGHT, lanes, paint(right=BW), right_hand_traffic=False) == 'noLane'
  assert change_blocker(LEFT, lanes, paint(left=BW), right_hand_traffic=False) == ''


# less than paint, never solid, yellow or no line: the kind unknown, or broken short of the bar, white or of unknown colour,
# on a frame whose paint was read (a read line can come back UNKNOWN; paint nobody read is ())
LESS_THAN_PAINT = (UNKNOWN, LinePaint('unknown', 'white', 0.9, 0.3), UNSURE_BW, COLORLESS)


def test_a_curb_side_change_on_less_than_paint_needs_section_4_10s_conditions():
  """ALWAYS_DRIVE.md 4.10, the owner's yes for the curb side only (unpainted_curb, under TurnPilotAlwaysDrive): the ego line
  toward the curb read short of broken white but never solid, yellow or no line, the model's next line out 0.85-1.5 lane
  widths beyond it with a trusted road edge, the lane not parked (carInLane and the blind spot are the planner's, at the offer
  and on every frame of it). Never toward the far side, never with nothing read (no scene checks the lane then), never held
  yellow, and never without unpainted_curb."""
  lanes = model_lanes(*road(1, 1))
  for p in LESS_THAN_PAINT:
    assert change_blocker(RIGHT, lanes, paint(right=p)) == 'lineNotBroken', p  # without it: as today
    assert change_blocker(RIGHT, lanes, paint(right=p), unpainted_curb=True) == '', p
    assert change_blocker(LEFT, lanes, paint(left=p), unpainted_curb=True) == 'lineNotBroken', p  # the far side: never
  for p in (SW, SY, BY, NO_LINE, LATCHED_YELLOW, LinePaint('solid', 'unknown', 0.9, 0.95), LinePaint('unknown', 'yellow', 0.2, 0.)):
    assert change_blocker(RIGHT, lanes, paint(right=p), unpainted_curb=True) == 'lineNotBroken', p
  assert change_blocker(RIGHT, lanes, (), unpainted_curb=True) == 'lineNotBroken'
  # the next line out a lane's width beyond, painted, with a trusted edge: else no lane of less than paint
  assert change_blocker(RIGHT, model_lanes(*road(1, 0)), paint(right=UNKNOWN), unpainted_curb=True) == 'lineNotBroken'
  assert change_blocker(RIGHT, model_lanes(*road(1, 1, edge_std=2.)), paint(right=UNKNOWN), unpainted_curb=True) == 'lineNotBroken'
  line_ys, probs, edges, stds = road(1, 1)
  line_ys[3] = line_ys[2] + 0.7 * LANE_WIDTH  # a bike lane's width out
  assert change_blocker(RIGHT, model_lanes(line_ys, probs, edges, stds), paint(right=UNKNOWN), unpainted_curb=True) == 'lineNotBroken'
  # parked cars ahead in it: a parking strip; the map's lane list none that way
  parked = scene([{'cls': 'car', 'x': 30., 'y': -3.6, 'vx': 0.}]).objects
  assert change_blocker(RIGHT, lanes, paint(right=UNKNOWN), parked, unpainted_curb=True) == 'laneParked'
  assert change_blocker(RIGHT, lanes, paint(right=UNKNOWN), nav_room=0, unpainted_curb=True) == 'noLane'
  # left-hand traffic: the curb is the left
  assert change_blocker(LEFT, lanes, paint(left=UNKNOWN), right_hand_traffic=False, unpainted_curb=True) == ''
  assert change_blocker(RIGHT, lanes, paint(right=UNKNOWN), right_hand_traffic=False, unpainted_curb=True) == 'noLane'


def test_lanes_needed_every_path_gated():
  left = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  # the map's lanes say two over, but only across broken white
  assert lane_changes_needed(left, model_lanes(*road(2, 0)), paint=paint(left=BW)) == 2
  for p in NOT_BW + ((),):
    pt = paint(left=p) if p != () else ()
    assert lane_changes_needed(left, model_lanes(*road(2, 0)), paint=pt) == 0, p
  # an exit by the model's count: also only across broken white (it used to go on unknown paint)
  ramp = maneuver("off ramp", "slight right")
  assert lane_changes_needed(ramp, model_lanes(*road(0, 2)), paint=paint(right=BW)) == -2
  assert lane_changes_needed(ramp, model_lanes(*road(0, 2)), paint=()) == 0
  assert lane_changes_needed(ramp, model_lanes(*road(0, 2)), paint=paint()) == 0
  # turns without the map's lanes, either side; left-hand traffic only toward its curb
  assert lane_changes_needed(maneuver("turn", "left"), model_lanes(*road(2, 0)), paint=paint(left=BW)) == 1
  assert lane_changes_needed(maneuver("turn", "right"), model_lanes(*road(0, 2)), paint=paint(right=BW)) == -1
  assert lane_changes_needed(maneuver("turn", "left"), model_lanes(*road(1, 0)), False, paint=paint(left=BW)) == 1
  assert lane_changes_needed(maneuver("turn", "right"), model_lanes(*road(0, 1)), False, paint=paint(right=BW)) == 0


def test_yellow_veto_and_lane_count():
  left = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  assert lane_changes_needed(left, model_lanes(*road(2, 0)), paint=paint(left=SY)) == 0
  assert lane_changes_needed(left, model_lanes(*road(2, 0)), paint=paint(left=BY)) == 0
  # lanes beyond a yellow line don't count toward where the car is: two lanes seen to the right, the far one beyond yellow,
  # puts the car in the middle of the map's three lanes, not the left one
  assert desired_changes(left, model_lanes(*road(1, 2)), paint=paint(far_right=SY)) == 1
  # broken white at the ego line, yellow at the far one: still one lane over
  assert lane_changes_needed(maneuver("turn", "left"), model_lanes(*road(2, 0)), paint=paint(left=BW, far_left=SY)) == 1


def test_yellow_hold():
  hold = YellowHold()
  out = hold.update(paint(left=SY), 15., 0.05, False)
  assert out[1] == SY
  # worn for a stretch: the line reads unknown, but stays yellow for the hold
  for _ in range(int((YELLOW_HOLD_S - 0.5) / 0.05)):
    out = hold.update(paint(left=UNKNOWN), 30., 0.05, False)
  assert out[1].color == 'yellow' and change_blocker(LEFT, model_lanes(*road(1, 1)), out) == 'lineNotBroken'
  # and when nothing is read at all
  assert hold.update((), 30., 0.05, False)[1].color == 'yellow'
  for _ in range(40):
    out = hold.update(paint(left=BW), 30., 0.05, False)
  assert out[1] == BW  # held long and far enough
  # a change moves the holds with the lines; a change of unknown direction holds all four
  hold = YellowHold()
  hold.update(paint(far_left=SY), 15., 0.05, False)
  hold.shift(LEFT)
  assert hold.update(paint(), 15., 0.05, False)[1].color == 'yellow'
  hold.shift(RIGHT)
  assert hold.update(paint(), 15., 0.05, False)[0].color == 'yellow'
  hold.shift(0)
  assert all(p.color == 'yellow' for p in hold.update(paint(), 15., 0.05, False))
  hold = YellowHold()
  hold.shift(0)  # nothing held, nothing to spread
  assert hold.update(paint(), 15., 0.05, False) == paint()


def test_target_lane_blocked_and_stopped_sides():
  sc = scene([{'cls': 'car', 'x': 5., 'y': 3.6, 'vx': 15.}])
  assert target_lane_blocked(sc.objects, LEFT, 15.)
  assert not target_lane_blocked(sc.objects, RIGHT, 15.)
  # well ahead and pulling away vs closing fast from behind
  assert not target_lane_blocked(scene([{'cls': 'car', 'x': 40., 'y': 3.6, 'vx': 20.}]).objects, LEFT, 15.)
  assert target_lane_blocked(scene([{'cls': 'car', 'x': -40., 'y': 3.6, 'vx': 25.}]).objects, LEFT, 15.)
  assert not target_lane_blocked(scene([{'cls': 'car', 'x': -40., 'y': 3.6, 'vx': 16.}]).objects, LEFT, 15.)
  # two lanes over doesn't count
  assert not target_lane_blocked(scene([{'cls': 'car', 'x': 0., 'y': 7.5, 'vx': 15.}]).objects, LEFT, 15.)
  # TrackedObject y is left: a parked car on the left is LEFT's, never RIGHT's
  parked_left = scene([{'cls': 'car', 'x': 20., 'y': 3.6, 'vx': 0.}]).objects
  assert stopped_in_lane(parked_left, LEFT) and not stopped_in_lane(parked_left, RIGHT)


class Drive:
  """A car on a road of lanes, with the real DesireHelper executing nav lane change requests."""
  def __init__(self, lanes_left, lanes_right, v=15., driver_signals=False, paint=striped, paint_rule=True, lag=0, curb_unpainted=False,
               night=False):
    """paint: (lanes left, lanes right) -> the four lines' paint where the car is now, or None for none read. lag: frames
    the planner sees DesireHelper's state late by (turnpilotd isn't synced to modeld, whose modelV2 carries it).
    curb_unpainted: the curb lane has no edge line, so one lane from the curb the model sees room there but no line.
    night: the model sees neither line beyond the ego lane's (drive 1e: the far lines over LINE_PROB_MIN on 17% and 0%)."""
    self.lanes_left, self.lanes_right, self.v = lanes_left, lanes_right, v
    self.curb_unpainted, self.night = curb_unpainted, night
    self.lag, self._lc_seen = lag, []
    self.paint = paint
    self.lp, self.dh = LanePlanner(paint_rule), DesireHelper()
    self.driver_signals = driver_signals
    self.lc_t = self.t = 0.
    self.offset = 0.  # the car's lateral offset from its lane's centre, positive left
    self.requests, self.prompts, self.offers, self.aborts, self.decisions = [], [], [], [], []
    self.car_kw = {}  # the car's own signal (car_signals.py): lit_m, lead_m, car_block, signals_failed
    self.car_lamp = False  # with car_kw: the car's lamp lights with the offer's warning, and lit_m counts from it

  def step(self, m, distance, objects=(), blinker=None, blindspot=None, maneuver_id=0, inputs_valid=True, stop_d=None):
    line_ys, probs, edges, stds = road(self.lanes_left, self.lanes_right)
    if self.curb_unpainted and self.lanes_right == 1:
      probs[3] = 0.05
    if self.night:
      probs[0] = probs[3] = 0.05
    # the car's offset in its lane: the lines and edges slide the other way (modelV2 y is right)
    line_ys, edges = [y + self.offset for y in line_ys], [y + self.offset for y in edges]
    lanes = model_lanes(line_ys, probs, edges, stds)
    left_blinker, right_blinker = blinker == LaneChange.left, blinker == LaneChange.right
    lc_state = self.dh.lane_change_state if not self.lag else \
      (self._lc_seen[-self.lag] if len(self._lc_seen) >= self.lag else LaneChangeState.off)
    d = self.lp.update(m, distance, lanes, lc_state, self.v, left_blinker, right_blinker, objects,
                       self.driver_signals, True, DT_MDL, self.paint(self.lanes_left, self.lanes_right) if self.paint else (),
                       blindspot == LaneChange.left, blindspot == LaneChange.right, maneuver_id, inputs_valid, stop_d=stop_d,
                       **self.car_kw)
    if self.car_lamp:
      lit = self.car_kw.get('lit_m', 0.) + self.v * DT_MDL if self.lp.signal_side != LaneChange.none else 0.
      self.car_kw['lit_m'] = lit
    prev = self.decisions[-1] if self.decisions else None
    if d.lane_change != LaneChange.none and (prev is None or prev.lane_change == LaneChange.none):
      self.requests.append((d.lane_change, distance, self.t))
    if d.offer != LaneChange.none and (prev is None or prev.offer == LaneChange.none):
      self.offers.append((d.offer, distance, self.t))
    if d.abort != 'none' and (prev is None or prev.abort == 'none'):
      self.aborts.append((d.abort, self.t))
    self.decisions.append(d)
    self.prompts.append(d.prompt)
    # the model takes 3 s to change lanes once DesireHelper asks it to
    # the car moves over a lane in that time, and the model relabels its lines halfway, once the car is across the line
    starting = self.dh.lane_change_state == LaneChangeState.laneChangeStarting
    self.lc_t = self.lc_t + DT_MDL if starting else 0.
    prob = 1. if starting and self.lc_t < 3. else 0.
    k = 1 if self.dh.lane_change_direction == log.LaneChangeDirection.left else -1
    if starting and self.lc_t >= 1.5 and self.lc_t - DT_MDL < 1.5:
      self.lanes_left, self.lanes_right = self.lanes_left - k, self.lanes_right + k
    frac = min(self.lc_t / 3., 1.)
    self.offset = 0. if not starting else k * LANE_WIDTH * (frac if frac < 0.5 else frac - 1.)
    cs = car_state(self.v, left_blinker=left_blinker, right_blinker=right_blinker, left_blindspot=blindspot == LaneChange.left,
                   right_blindspot=blindspot == LaneChange.right)  # DesireHelper's own blind spot check, as on the car
    self._lc_seen.append(self.dh.lane_change_state)
    self.dh.update(cs, True, prob, d.lane_change)
    self.t += DT_MDL
    return d

  def run(self, m, distance, seconds, moving=True, **kw):
    for _ in range(int(round(seconds / DT_MDL))):
      self.step(m, distance, **kw)
      distance -= self.v * DT_MDL if moving else 0.
    return distance


def test_left_turn_on_three_lane_road_warns_then_changes_twice():
  drive = Drive(lanes_left=2, lanes_right=0)
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  distance = drive.run(m, 500., 40.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.left] * 2 and drive.lp.changes == 2
  assert (drive.lanes_left, drive.lanes_right) == (0, 2)
  # each request comes the full warning after its offer, inside the window
  assert len(drive.offers) == 2
  for (_, _, t_offer), (_, _, t_req) in zip(drive.offers, drive.requests, strict=True):
    assert abs((t_req - t_offer) - LC_WARN_S) < 2 * DT_MDL
  start, end = lane_change_window(drive.v, 2)
  assert all(end < d < start for _, d, _ in drive.offers)
  assert drive.offers[0][1] > lane_change_window(drive.v)[0]  # two changes to make: the first starts earlier
  # the countdown runs down during the warning
  counting = [d.countdown for d in drive.decisions if d.offer != LaneChange.none and d.lane_change == LaneChange.none]
  assert counting[0] > LC_WARN_S - 0.1 and min(counting) < 0.1
  assert not drive.aborts and distance < 100.


CONTRARY = (SW, SY, BY, NO_LINE)
UNREADABLE = (UNKNOWN, UNSURE_BW, COLORLESS, LinePaint('unknown', 'white', 0.9, 0.3))


def test_never_across_a_contrary_read_nor_on_paint_nobody_read():
  """Solid, yellow or no line read: never, the map's lane list and the model's lanes notwithstanding. Paint nobody read: never
  either (lane_agreed needs a read; the lines unreadable, test_an_unreadable_line_goes_where_the_map_and_the_model_agree)."""
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  for p in CONTRARY:
    drive = Drive(lanes_left=2, lanes_right=0, paint=lambda left, right, p=p: paint(left=p, right=SW))
    drive.run(m, 500., 40.)
    assert not drive.offers and not drive.requests, p
  drive = Drive(lanes_left=2, lanes_right=0, paint=None)  # the detector's paint never arrives
  drive.run(m, 500., 40.)
  assert not drive.offers
  drive = Drive(lanes_left=0, lanes_right=2, paint=None)
  drive.run(maneuver("off ramp", "slight right"), 500., 40.)
  assert not drive.offers  # an exit by the model's lane count no longer goes on unknown paint


def test_turned_off_plans_nothing():
  drive = Drive(lanes_left=2, lanes_right=0, paint_rule=False)
  drive.run(maneuver("turn", "left", THREE_LANES_LEFT_TURN), 500., 40.)
  assert not drive.offers and not drive.requests


def test_tesla_driver_confirms_with_the_stalk():
  drive = Drive(lanes_left=0, lanes_right=1, driver_signals=True)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 300., 2.)
  assert drive.offers and drive.prompts[-1] == LaneChange.right and not drive.requests
  # the countdown ends: still nothing until the driver signals
  distance = drive.run(m, distance, LC_WARN_S)
  d = drive.decisions[-1]
  assert d.offer == LaneChange.right and d.countdown == 0. and d.lane_change == LaneChange.none and drive.lp.phase == Phase.WAIT
  drive.run(m, distance, 5., blinker=LaneChange.right)
  assert [r for r, _, _ in drive.requests] == [LaneChange.right] and drive.lp.changes == 1 and not drive.aborts


def test_tesla_stalk_during_the_countdown_is_the_go_ahead():
  drive = Drive(lanes_left=0, lanes_right=1, driver_signals=True)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 300., 1.5)
  assert drive.lp.phase == Phase.WARN
  drive.run(m, distance, 5., blinker=LaneChange.right)
  assert drive.requests and drive.requests[0][2] - drive.offers[0][2] < LC_WARN_S and drive.lp.changes == 1


def test_tesla_no_signal_waits_to_the_windows_end_then_offers_again():
  """Route 29 (2026-11-08, 187-198 s): the wait for the driver's signal had its own 8 s clock (LC_SIGNAL_TIMEOUT_S) and
  aborted noSignal at 80 m with 150 m of window left, a driver who hadn't yet noticed the prompt (or, there, whose lamp the
  port hid) out of time. Now the prompt stays up, its chime repeating, for as long as the window is open: a driver who
  signals 20 s on still gets the change, and one who never does is aborted at the window's end (tooLate, then LC-03)."""
  drive = Drive(lanes_left=0, lanes_right=1, driver_signals=True)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 250., LC_CONFIRM_FRAMES * DT_MDL + LC_WARN_S + 2. * LC_SIGNAL_TIMEOUT_S, moving=False)
  assert not drive.aborts and not drive.requests and drive.lp.phase == Phase.WAIT
  assert drive.decisions[-1].prompt == LaneChange.right  # still asking
  drive.run(m, distance, 1., blinker=LaneChange.right, moving=False)
  assert [r for r, _, _ in drive.requests] == [LaneChange.right] and not drive.aborts
  # never signalled, moving: the prompt lasts to the window's end, then tooLate
  drive = Drive(lanes_left=0, lanes_right=1, driver_signals=True)
  start, end = lane_change_window(drive.v)
  distance = drive.run(m, start - 1., (start - end) / drive.v + ABORT_HOLD_S + 1.)
  assert [a for a, _ in drive.aborts] == ['tooLate'] and not drive.requests
  t_abort = drive.aborts[0][1]
  prompts = [d.prompt for d, t in zip(drive.decisions, np_times(drive), strict=True) if t < t_abort - 1e-6]
  assert prompts[-1] == LaneChange.right
  held = [d.abort for d, t in zip(drive.decisions, np_times(drive), strict=True) if t >= t_abort - 1e-6]
  n = int(round(ABORT_HOLD_S / DT_MDL))
  assert held[:n] == ['tooLate'] * n  # held for the alert
  # standing still with the offer aborted another way (the stalk the other way), the change is offered again after the cooldown
  drive = Drive(lanes_left=0, lanes_right=1, driver_signals=True)
  distance = drive.run(m, 250., LC_CONFIRM_FRAMES * DT_MDL + 1., moving=False)
  drive.lp.cancel('handedBack')
  t_abort = drive.t
  drive.run(m, distance, LC_ABORT_COOLDOWN_S + 1., moving=False)
  assert len(drive.offers) == 2 and drive.offers[1][2] - t_abort >= LC_ABORT_COOLDOWN_S - 2 * DT_MDL


def np_times(drive):
  return [k * DT_MDL for k in range(len(drive.decisions))]


def test_tesla_blinker_off_before_the_change_starts_aborts():
  drive = Drive(lanes_left=0, lanes_right=1, driver_signals=True)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 290., 1.5)
  # the driver flicks the stalk; the blinker runs out before DesireHelper starts (as when the blind spot holds it)
  drive.dh.lane_change_state = LaneChangeState.preLaneChange
  drive.lp.update(m, distance, model_lanes(*road(0, 1)), LaneChangeState.preLaneChange, 15., False, True, (), True, True, DT_MDL,
                  paint(right=BW), maneuver_id=0)
  assert drive.lp.phase == Phase.REQUEST
  drive.dh.lane_change_state = LaneChangeState.preLaneChange
  drive.step(m, distance)
  assert drive.decisions[-1].lane_change == LaneChange.none  # withdrawn at once; said once DesireHelper is seen not to start
  drive.run(m, distance, LC_ABORT_SETTLE_S + DT_MDL)
  assert drive.aborts[-1][0] == 'noSignal' and drive.lp.phase == Phase.IDLE


def test_aborts_before_the_change_starts():
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)

  def offered(m=m, **kw):
    drive = Drive(lanes_left=0, lanes_right=1, **kw)
    distance = drive.run(m, 290., 1.5)
    assert drive.lp.phase == Phase.WARN
    return drive, distance

  # the line turns solid, yellow or none; read unknown, or not read at all, it waits (test_an_offer_waits_out_an_unsure_line)
  for p in (SY, BY, NO_LINE, LinePaint('solid', 'unknown', 0.3, 0.9), LinePaint('unknown', 'yellow', 0.2, 0.)):
    drive, distance = offered()
    drive.paint = lambda left, right, p=p: paint(right=p)
    drive.run(m, distance, 1.)
    assert [a for a, _ in drive.aborts] == ['lineNotBroken'] and not drive.requests, p
  # solid white: an abort where no lane list names the lane beyond it; into the turn lane the list names, the offer stands
  drive, distance = offered(maneuver("turn", "right"))
  drive.paint = lambda left, right: paint(right=SW)
  drive.run(maneuver("turn", "right"), distance, 1.)
  assert [a for a, _ in drive.aborts] == ['lineNotBroken'] and not drive.requests
  drive, distance = offered()
  drive.paint = lambda left, right: paint(right=SW)
  drive.run(m, distance, 1.)
  assert not drive.aborts and drive.lp.phase == Phase.WARN
  drive, distance = offered()
  drive.paint = None  # no paint read (the wide camera): no give-up for lack of a read, nothing requested
  drive.run(m, distance, 1.)
  assert not drive.aborts and not drive.requests and drive.lp.phase != Phase.IDLE
  # the model stops seeing a lane that way
  drive, distance = offered()
  drive.lanes_right = 0
  drive.paint = lambda left, right: paint(right=BW)
  drive.run(m, distance, 1.)
  assert [a for a, _ in drive.aborts] == ['noLane'] and not drive.requests
  # the blind spot fills
  drive, distance = offered()
  drive.run(m, distance, 1., blindspot=LaneChange.right)
  assert [a for a, _ in drive.aborts] == ['blindspot'] and not drive.requests
  # a car in the target lane
  drive, distance = offered()
  drive.run(m, distance, 1., objects=scene([{'cls': 'car', 'x': 2., 'y': -3.6, 'vx': 15.}]).objects)
  assert [a for a, _ in drive.aborts] == ['carInLane']
  # parked cars come into view in that lane
  drive, distance = offered()
  drive.run(m, distance, 1., objects=scene([{'cls': 'car', 'x': 50., 'y': -3.6, 'vx': 0.}]).objects)
  assert [a for a, _ in drive.aborts] == ['laneParked']
  # the route changes, or goes
  drive, distance = offered()
  drive.run(m, distance, 1., maneuver_id=1)
  assert [a for a, _ in drive.aborts] == ['routeChanged']
  drive, distance = offered()
  drive.run(None, distance, 1.)
  assert [a for a, _ in drive.aborts] == ['routeChanged']
  # the car slows below the lane change speed
  drive, distance = offered()
  drive.v = 2.
  drive.run(m, distance, 1.)
  assert [a for a, _ in drive.aborts] == ['tooSlow']
  # inputs go invalid, the planner is turned off
  drive, distance = offered()
  drive.run(m, distance, 1., inputs_valid=False)
  assert [a for a, _ in drive.aborts] == ['badInputs']
  drive, distance = offered()
  drive.lp.paint_rule = False
  drive.run(m, distance, 1.)
  assert [a for a, _ in drive.aborts] == ['turnedOff']
  # already in a lane the turn leaves from (the model's count changed under the offer)
  two_right = maneuver("turn", "right", ((("straight",), False), (("straight", "right"), True), (("right",), True)))
  drive = Drive(lanes_left=0, lanes_right=2)
  distance = drive.run(two_right, 290., 1.5)
  assert drive.lp.phase == Phase.WARN and drive.lp.offer == LaneChange.right
  drive.lanes_left, drive.lanes_right = 1, 1
  drive.run(two_right, distance, 1.)
  assert [a for a, _ in drive.aborts] == ['notNeeded']


def test_too_close_to_the_turn_aborts():
  drive = Drive(lanes_left=0, lanes_right=1, driver_signals=True)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  _, end = lane_change_window(drive.v)
  drive.run(m, end + 3.5 * drive.v, 6.)  # offered just before the end of the window, never signaled
  assert drive.offers and [a for a, _ in drive.aborts] == ['tooLate']


def test_desire_helper_waiting_too_long_aborts():
  lp = LanePlanner()
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  lanes = model_lanes(*road(0, 1))
  args = {'lanes': lanes, 'v_ego': 15., 'left_blinker': False, 'right_blinker': False, 'objects': (), 'driver_signals': False,
          'right_hand_traffic': True, 'dt': DT_MDL, 'paint': paint(right=BW)}
  distance, d = 290., None
  for _ in range(int((LC_CONFIRM_FRAMES * DT_MDL + LC_WARN_S + LC_REQUEST_TIMEOUT_S + 1.) / DT_MDL)):
    d = lp.update(m, distance, lc_state=LaneChangeState.preLaneChange, **args)
    if d.abort != 'none':
      break
    distance -= 15. * DT_MDL
  assert d.abort == 'timedOut' and d.lane_change == LaneChange.none


def test_never_cancelled_once_started():
  drive = Drive(lanes_left=0, lanes_right=1)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 290., LC_CONFIRM_FRAMES * DT_MDL + LC_WARN_S + 0.3)
  assert drive.dh.lane_change_state == LaneChangeState.laneChangeStarting
  # mid-change everything goes wrong: the paint, the blind spot, the route
  drive.paint = lambda left, right: paint(right=SY)
  drive.run(None, distance, 1., blindspot=LaneChange.right, maneuver_id=7)
  assert not drive.aborts and drive.dh.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)


def requested_with_lag(fail=None, cancel=None):
  """A Tesla's change left with turnpilotd seeing DesireHelper a frame late: on the frame after the request a check fails
  (fail: update() keywords), or TurnPilot is cancelled (cancel: the reason; with fail too, a frame later still)."""
  drive = Drive(lanes_left=1, lanes_right=0, driver_signals=True, lag=1)
  m, distance, requested = maneuver("turn", "left"), 310., None
  for i in range(int(12. / DT_MDL)):
    if cancel and requested is not None and i == requested + (2 if fail else 1):
      drive.lp.cancel(cancel)
    kw = fail if fail and requested is not None and i == requested + 1 else {}
    d = drive.step(m, distance, blinker=LaneChange.left, **kw)
    if d.lane_change != LaneChange.none and requested is None:
      requested = i
    distance -= drive.v * DT_MDL
  return drive


def test_no_abort_announced_once_desire_helper_may_have_started():
  # DesireHelper starts the change on the request's first frame; turnpilotd only sees it a frame later. A check failing in
  # between must not say "Lane Change Cancelled" while the car moves over: the change is ours, and it goes on
  for kw in ({'fail': {'inputs_valid': False}}, {'fail': {'objects': scene([{'cls': 'car', 'x': 60., 'y': 3.6, 'vx': 0.}]).objects}},
             {'cancel': 'handedBack'}, {'cancel': 'handedBack', 'fail': {'inputs_valid': False}}):
    drive = requested_with_lag(**kw)
    assert not drive.aborts and drive.lp.changes == 1 and (drive.lanes_left, drive.lanes_right) == (0, 1), kw


def test_an_abort_before_desire_helper_started_still_goes_out():
  # the blind spot fills in the request: the request is withdrawn at once, the abort said once turnpilotd has seen that
  # DesireHelper didn't start
  drive = Drive(lanes_left=0, lanes_right=1, driver_signals=True, lag=1)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 290., 1.5)
  drive.dh.lane_change_state = LaneChangeState.preLaneChange  # held by the blind spot: never starts
  drive.lp.update(m, distance, model_lanes(*road(0, 1)), LaneChangeState.preLaneChange, 15., False, True, (), True, True, DT_MDL,
                  paint(right=BW), maneuver_id=0)
  assert drive.lp.phase == Phase.REQUEST
  d = drive.step(m, distance, blindspot=LaneChange.right)
  assert d.lane_change == LaneChange.none and d.abort == 'none'
  drive.run(m, distance, 1., blindspot=LaneChange.right)
  assert [a for a, _ in drive.aborts] == ['blindspot'] and drive.aborts[0][1] - d_time(drive, d) < LC_ABORT_SETTLE_S + DT_MDL / 2
  assert drive.lp.changes == 0 and (drive.lanes_left, drive.lanes_right) == (0, 1)


def d_time(drive, d):
  return next(k * DT_MDL for k, x in enumerate(drive.decisions) if x is d)


@pytest.mark.parametrize("always_drive", [False, True])
def test_blind_spot_or_car_at_the_offer_holds_it_back(always_drive):
  """Before an offer: the pre-offer waits stay under TurnPilotAlwaysDrive too (LC-01 changes only an offer's hold)."""
  drive = Drive(lanes_left=0, lanes_right=1)
  drive.lp.always_drive = always_drive
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 300., 3., blindspot=LaneChange.right)
  assert not drive.offers and drive.decisions[-1].reason == "car in blind spot"
  beside = scene([{'cls': 'car', 'x': -3., 'y': -3.6, 'vx': 15.}]).objects
  distance = drive.run(m, distance, 2., objects=beside)
  assert not drive.offers and drive.decisions[-1].reason == "car in target lane"
  drive.run(m, distance, 6.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.right]


# ------------------------------------------------ LC-01, LC-02 (ALWAYS_DRIVE.md step S4, TurnPilotAlwaysDrive): the blind spot hold

def always(drive):
  drive.lp.always_drive = True
  return drive


def to_request(drive, m, distance):
  """Drive on, nothing in the way, to the offer's first request frame (the car signalling: its countdown out); the distance."""
  while not drive.requests:
    drive.step(m, distance)
    distance -= drive.v * DT_MDL
  assert drive.lp.phase == Phase.REQUEST
  return distance


def to_phase(drive, m, phase, distance=290.):
  """An offer in phase, nothing in the way: WARN (its countdown under way), WAIT (the countdown out, the driver signalling and
  not yet signalled), REQUEST (the car signalling, the request's first frame). The distance."""
  distance = drive.run(m, distance, LC_CONFIRM_FRAMES * DT_MDL + (LC_WARN_S + 0.2 if phase == Phase.WAIT else 0.5))
  if phase == Phase.REQUEST:
    distance = to_request(drive, m, distance)
  assert drive.lp.phase == phase and len(drive.offers) == 1
  return distance


def test_a_blind_spot_under_an_offer_holds_it_under_the_switch_never_aborts_it():
  """LC-01: under TurnPilotAlwaysDrive a car in the blind spot during an offer (WARN, WAIT or REQUEST) no longer cancels it
  ('blindspot', a 6 s cooldown and a fresh offer chime, over and over): the offer stays up, any request withdrawn so DesireHelper
  never goes on its own the instant it clears, for longer than LC_REQUEST_TIMEOUT_S and LC_SIGNAL_TIMEOUT_S (LC-02: neither
  runs while the blind spot holds it). WARN and REQUEST are held (hold 'blindspot', the status line saying why); WAIT, the
  driver's blinker still to come, keeps asking for it. With the switch off it aborts as today."""
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  for on in (False, True):
    for phase in (Phase.WARN, Phase.WAIT, Phase.REQUEST):
      drive = Drive(0, 1, driver_signals=phase == Phase.WAIT)
      drive.lp.always_drive = on
      distance = to_phase(drive, m, phase)
      n, requests = len(drive.decisions), len(drive.requests)
      drive.run(m, distance, max(LC_REQUEST_TIMEOUT_S, LC_SIGNAL_TIMEOUT_S) + 0.5, blindspot=LaneChange.right)
      held = drive.decisions[n:]
      if not on:
        assert [a for a, _ in drive.aborts] == ['blindspot'] and all(d.hold == 'none' for d in drive.decisions), phase
        continue
      assert not drive.aborts and len(drive.offers) == 1 and len(drive.requests) == requests, phase
      assert all(d.offer == LaneChange.right and d.lane_change == LaneChange.none for d in held), phase
      assert drive.dh.lane_change_state != LaneChangeState.laneChangeStarting and drive.lp.changes == 0, phase
      if phase == Phase.WAIT:
        assert drive.lp.phase == Phase.WAIT and all(d.hold == 'none' and d.prompt == LaneChange.right for d in held)
      else:
        assert all(d.hold == 'blindspot' and d.countdown == 0. and d.reason == "car in blind spot" for d in held), phase


def test_the_blind_spot_clearing_says_the_change_again_and_counts_down_before_the_request():
  """LC-01's release: the blind spot read clear for LC_HOLD_CLEAR_S (0.5 s), the change is said again (hold 'clear': the lane
  change chime, selfdrived) with an LC_RELEASE_S (1.5 s) countdown, and only then the request, which DesireHelper makes. One
  offer throughout, never an abort; no request on any frame before the countdown is out."""
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  for phase in (Phase.WARN, Phase.REQUEST):
    drive = always(Drive(0, 1))
    distance = to_phase(drive, m, phase)
    distance = drive.run(m, distance, 4., blindspot=LaneChange.right)
    n = len(drive.decisions)
    drive.run(m, distance, 6.)
    after = drive.decisions[n:]
    k_clear = next(i for i, d in enumerate(after) if d.hold == 'clear')
    k_req = next(i for i, d in enumerate(after) if d.lane_change != LaneChange.none)
    assert all(d.hold == 'blindspot' for d in after[:k_clear]) and k_clear * DT_MDL == pytest.approx(LC_HOLD_CLEAR_S, abs=DT_MDL), phase
    assert all(d.hold == 'clear' for d in after[k_clear:k_req]), phase
    assert (k_req - k_clear) * DT_MDL == pytest.approx(LC_RELEASE_S, abs=DT_MDL / 2), phase
    counting = [d.countdown for d in after[k_clear:k_req]]
    assert counting[0] == pytest.approx(LC_RELEASE_S) and all(a > b for a, b in zip(counting, counting[1:], strict=False)), phase
    assert after[k_req].hold == 'none' and after[k_req].countdown == 0.
    assert len(drive.offers) == 1 and not drive.aborts and drive.lp.changes == 1, phase
    assert (drive.lanes_left, drive.lanes_right) == (1, 0), phase


def test_a_blind_spot_back_inside_the_release_countdown_holds_again_with_no_request_sent():
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  drive = always(Drive(0, 1))
  distance = to_phase(drive, m, Phase.WARN)
  distance = drive.run(m, distance, 2., blindspot=LaneChange.right)
  distance = drive.run(m, distance, LC_HOLD_CLEAR_S + 0.5)  # clear: into the release countdown
  assert drive.decisions[-1].hold == 'clear' and drive.decisions[-1].countdown < LC_RELEASE_S
  n = len(drive.decisions)
  distance = drive.run(m, distance, 1., blindspot=LaneChange.right)  # back before the countdown is out
  assert all(d.hold == 'blindspot' and d.lane_change == LaneChange.none for d in drive.decisions[n:])
  n = len(drive.decisions)
  drive.run(m, distance, 6.)
  k_req = next(i for i, d in enumerate(drive.decisions[n:]) if d.lane_change != LaneChange.none)
  assert k_req * DT_MDL == pytest.approx(LC_HOLD_CLEAR_S + LC_RELEASE_S, abs=DT_MDL)  # the full release again, from the start
  assert [r for r, _, _ in drive.requests] == [LaneChange.right] and not drive.aborts and drive.lp.changes == 1


def test_no_timed_out_while_the_blind_spot_holds_the_request_timed_out_kept_with_it_clear():
  """LC-02: DesireHelper waiting LC_REQUEST_TIMEOUT_S aborts timedOut only while the request is out: held for the blind spot it
  is withdrawn and the clock stops; released, with the blind spot clear and DesireHelper still not starting the change, it runs
  on and aborts as before. Off the switch the blind spot aborts the request."""
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  for on in (False, True):
    lp = LanePlanner()
    lp.always_drive = on
    args = {'lanes': model_lanes(*road(0, 1)), 'v_ego': 15., 'left_blinker': False, 'right_blinker': False, 'objects': (),
            'driver_signals': False, 'right_hand_traffic': True, 'dt': DT_MDL, 'paint': paint(right=BW), 'lc_state': LaneChangeState.preLaneChange}
    distance, d = 290., None
    while d is None or d.lane_change == LaneChange.none:
      d = lp.update(m, distance, **args)
      distance -= 15. * DT_MDL
    distance = 250.  # standing in for a long road: the window's end never comes
    held = [lp.update(m, distance, right_blindspot=True, **args) for _ in range(int(12. / DT_MDL))]
    if not on:
      assert any(x.abort == 'blindspot' for x in held)
      continue
    assert all(x.abort == 'none' and x.hold == 'blindspot' and x.lane_change == LaneChange.none for x in held)
    clear = [lp.update(m, distance, **args) for _ in range(int((LC_HOLD_CLEAR_S + LC_RELEASE_S + LC_REQUEST_TIMEOUT_S + 1.) / DT_MDL))]
    k_req = next(i for i, x in enumerate(clear) if x.lane_change != LaneChange.none)
    k_out = next(i for i, x in enumerate(clear) if x.abort == 'timedOut')
    assert (k_out - k_req) * DT_MDL == pytest.approx(LC_REQUEST_TIMEOUT_S, abs=LC_ABORT_SETTLE_S + 2 * DT_MDL)


def test_a_blind_spot_held_to_the_windows_end_aborts_too_late_and_the_turn_is_skipped():
  """The hold lasts until the window's end (tooLate, the request never sent), then LC-03 skips the turn out loud."""
  drive = always(Drive(0, 1))
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = to_phase(drive, m, Phase.WARN)
  frames = through_the_window(drive, m, distance, blindspot=LaneChange.right)
  assert [a for a, _ in drive.aborts] == ['tooLate'] and len(drive.offers) == 1 and not drive.requests
  assert_blocked_at_the_end(drive, frames)


def test_the_drivers_blinker_ending_under_the_hold_asks_again_never_no_signal():
  """The driver signalling (car signals off): the stalk let go while only the blind spot holds the change returns it to WAIT
  ('Signal Right to Change Lanes', its chime: hold none again) instead of aborting noSignal; WAIT's LC_SIGNAL_TIMEOUT_S doesn't
  run while the blind spot reads occupied; signalled again it is held, then released and made."""
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  drive = always(Drive(0, 1, driver_signals=True))
  distance = to_phase(drive, m, Phase.WARN)
  distance = drive.run(m, distance, 2., blinker=LaneChange.right, blindspot=LaneChange.right)  # the go-ahead, held
  assert drive.lp.phase == Phase.REQUEST and drive.decisions[-1].hold == 'blindspot' and drive.decisions[-1].prompt == LaneChange.right
  distance = drive.run(m, distance, 1., blindspot=LaneChange.right)  # the stalk let go
  d = drive.decisions[-1]
  assert drive.lp.phase == Phase.WAIT and d.hold == 'none' and d.prompt == LaneChange.right and d.offer == LaneChange.right
  drive.run(m, distance, LC_SIGNAL_TIMEOUT_S + 1., moving=False, blindspot=LaneChange.right)
  assert not drive.aborts and drive.lp.phase == Phase.WAIT
  distance = drive.run(m, distance, 1., blinker=LaneChange.right, blindspot=LaneChange.right)  # signalled again: held
  assert drive.lp.phase == Phase.REQUEST and drive.decisions[-1].hold == 'blindspot' and not drive.requests
  drive.run(m, distance, 6., blinker=LaneChange.right)
  assert len(drive.offers) == 1 and not drive.aborts and drive.lp.changes == 1 and len(drive.requests) == 1


def test_the_cars_lamp_stays_lit_through_the_hold_and_a_car_in_the_target_lane_still_aborts():
  """Where the car signals, its lamp stays on its side through the hold (signal_side), the change going at the release once
  the lamp has led it by the legal distance. A car TurnPilot's cameras see in the target lane still aborts the offer (R13)."""
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  drive = always(Drive(0, 1))
  drive.car_kw, drive.car_lamp = {'lit_m': 0., 'lead_m': 91.44}, True
  distance = to_phase(drive, m, Phase.WARN)
  sides = []
  for _ in range(int(6. / DT_MDL)):
    drive.step(m, distance, blindspot=LaneChange.right)
    sides.append(drive.lp.signal_side)
    distance -= drive.v * DT_MDL
  assert set(sides) == {LaneChange.right} and drive.car_kw['lit_m'] > 91.44 and not drive.requests
  drive.run(m, distance, 6.)
  assert len(drive.requests) == 1 and drive.lp.changes == 1 and not drive.aborts
  drive = always(Drive(0, 1))
  distance = to_phase(drive, m, Phase.WARN)
  distance = drive.run(m, distance, 1., blindspot=LaneChange.right)
  drive.run(m, distance, 1., blindspot=LaneChange.right, objects=scene([{'cls': 'car', 'x': 2., 'y': -3.6, 'vx': 15.}]).objects)
  assert [a for a, _ in drive.aborts] == ['carInLane']


def test_the_planner_changes_toward_the_curb_on_less_than_paint_only_under_the_switch():
  """4.10 in the planner: a right turn without a lane list, the curb lane beyond a line the paint can't read. Off the switch no
  offer (lineNotBroken, as today); on it the offer, its LC_WARN_S countdown said first, then the change. A line read solid white
  or yellow on any frame of the window is never less than paint: no such change for the rest of it. The blind spot and a car
  in the target lane hold it back as any change; toward the far side nothing changes."""
  m = maneuver("turn", "right")
  def unread(left, right):
    return paint(left=SW, right=UNKNOWN if right else SW)
  for on in (False, True):
    drive = Drive(0, 1, paint=unread)
    drive.lp.always_drive = on
    drive.run(m, 290., 8.)
    if not on:
      assert not drive.offers and drive.decisions[-1].wait == 'lineNotBroken'
      continue
    (side, _, t_offer), = drive.offers
    (_, _, t_request), = drive.requests
    assert side == LaneChange.right and t_request - t_offer == pytest.approx(LC_WARN_S, abs=1.5 * DT_MDL)
    assert drive.lp.changes == 1 and (drive.lanes_left, drive.lanes_right) == (1, 0) and not drive.aborts
  for marked in (SW, SY):
    frames = {'n': 0}
    def once(left, right, marked=marked, frames=frames):
      frames['n'] += 1
      return paint(left=SW, right=marked if frames['n'] <= 3 else UNKNOWN)
    drive = always(Drive(0, 1, paint=once))
    drive.run(m, 290., 9.)
    assert not drive.offers and drive.decisions[-1].wait == 'lineNotBroken', marked
    # review 2026-11-07 (LC-2): the same line read white of unknown kind after the solid or yellow frames (a solid line in
    # dappled shade reads so) cleared its mark, and lane_agreed's white sign crossed it, the switch on or off: offered at 274
    # m and changed. The latch is the window's for every less-than-paint change toward the curb
    for on in (False, True):
      frames = {'n': 0}
      def then_white(left, right, marked=marked, frames=frames):
        frames['n'] += 1
        return paint(left=SW, right=marked if frames['n'] <= 3 else LinePaint('unknown', 'white', 0.9, 0.6))
      drive = Drive(0, 1, paint=then_white)
      drive.lp.always_drive = on
      drive.run(m, 290., 9.)
      assert not drive.offers and not drive.requests and drive.lp.changes == 0, (marked, on)
  drive = always(Drive(0, 1, paint=unread))
  distance = drive.run(m, 290., 3., blindspot=LaneChange.right)
  drive.run(m, distance, 2., objects=scene([{'cls': 'car', 'x': -3., 'y': -3.6, 'vx': 15.}]).objects)
  assert not drive.offers
  drive = always(Drive(1, 0, paint=lambda left, right: paint(left=UNKNOWN if left else SY, right=SW)))
  drive.run(maneuver("turn", "left"), 290., 8.)
  assert not drive.offers and drive.decisions[-1].wait == 'lineNotBroken'


def test_a_curb_side_change_on_less_than_paint_needs_this_frames_paint_read():
  """ad-s4's block on 4.10, in the planner: the yellow hold (YellowHold.apply) gives a frame with no paint read four entries, the
  held line yellow and the rest unknown. So with a centre line read yellow on the car's left, then nothing read (turnpilotd's
  plan_paint: the wide camera on a bend or with a turn desire out, a stop sweep, the paint off for the drive), the curb line came
  out 'unknown' on paint nobody read, and 4.10 changed lanes on it. 4.10 needs this frame's paint read, before the hold: no
  offer (lineNotBroken), and LC-03 doesn't place the car on such frames, as off the switch. An offer of it under way aborts
  lineNotBroken on the first frame nothing is read, as a change on broken white does on such a frame."""
  m = maneuver("turn", "right")
  start, end = lane_change_window(15.)
  def yellow_then_unread(frames):
    def read(left, right):  # read on the first three frames, 30 m before the window: the hold runs 100 m into it
      frames['n'] += 1
      return paint(left=SY, right=UNKNOWN) if frames['n'] <= 3 else ()
    return read
  for on in (False, True):
    drive = Drive(0, 1, paint=yellow_then_unread({'n': 0}))
    drive.lp.always_drive = on
    drive.run(m, start + 30., 8.)
    assert not drive.offers and drive.lp.changes == 0 and drive.decisions[-1].wait == 'lineNotBroken', on
    # the blind spot occupied through the window, so the change never goes: nothing read placed the car that way (LC-03),
    # so the window's end skips nothing, under the switch as off it
    drive = Drive(0, 1, paint=yellow_then_unread({'n': 0}))
    drive.lp.always_drive = on
    distance = drive.run(m, start + 30., 19., blindspot=LaneChange.right)
    assert distance < end and not drive.offers and not drive.decisions[-1].blocked, on
  # the curb line read unknown on every frame (4.10's less than paint), the blind spot held: placed, and skipped at the end
  drive = always(Drive(0, 1, paint=lambda left, right: paint(left=SY, right=UNKNOWN)))
  assert drive.run(m, start + 30., 19., blindspot=LaneChange.right) < end
  assert not drive.offers and drive.decisions[-1].blocked
  # an offer on less than paint under way, then nothing read: it waits, never requested on a frame nobody read (before
  # 2026-11-07 the first such frame aborted it), and aborts once LC_UNSURE_MAX_S have passed with nothing read
  drive = always(Drive(0, 1))
  drive.paint = lambda left, right: paint(left=SY, right=UNKNOWN) if not drive.offers else ()
  drive.run(m, start - 10., LC_UNSURE_MAX_S + 1.)
  assert len(drive.offers) == 1 and [a for a, _ in drive.aborts] == ['lineNotBroken'] and not drive.requests
  # (the last read is carried PAINT_CARRY_S first, and the countdown over by then, the abort settles LC_ABORT_SETTLE_S)
  from openpilot.selfdrive.turnpilot.lane_planner import LC_ABORT_SETTLE_S, PAINT_CARRY_S
  assert drive.aborts[0][1] - drive.offers[0][2] == pytest.approx(LC_UNSURE_MAX_S + PAINT_CARRY_S + LC_ABORT_SETTLE_S, abs=2.5 * DT_MDL)


def test_driver_lane_change_gets_a_cooldown_too():
  lp = LanePlanner()
  m = maneuver("turn", "left")
  lanes = model_lanes(*road(1, 1))
  kw = {'v_ego': 15., 'objects': (), 'driver_signals': True, 'right_hand_traffic': True, 'dt': DT_MDL, 'paint': paint(left=BW, right=BW)}
  lp.update(m, 300., lanes, LaneChangeState.laneChangeStarting, left_blinker=False, right_blinker=False, **kw)  # direction unknown
  d = None
  for k in range(LC_CONFIRM_FRAMES + 2):
    d = lp.update(m, 300. - k, lanes, LaneChangeState.off, left_blinker=False, right_blinker=False, **kw)
  assert d.offer == LaneChange.none  # still cooling down: the lines may have renumbered


def test_no_request_below_lane_change_speed():
  drive = Drive(lanes_left=0, lanes_right=2, v=2.)
  drive.run(maneuver("turn", "right", TWO_LANES_RIGHT_TURN), 200., 10.)
  assert not drive.offers and not drive.requests


def three_lanes_and_a_bike_lane(lanes_left, lanes_right):
  """Double yellow on the left, broken white between the three lanes, a solid white bike lane line on the right."""
  return paint(left=BW if lanes_left else SY, right=BW if lanes_right else SW)


def test_turns_move_over_while_the_line_is_broken_white():
  # left turn, no map lanes: over twice, then the line on the left is yellow
  drive = Drive(lanes_left=2, lanes_right=0, paint=three_lanes_and_a_bike_lane)
  drive.run(maneuver("turn", "left"), 500., 40.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.left] * 2 and (drive.lanes_left, drive.lanes_right) == (0, 2)
  # right turn: over to the right lane, never into the bike lane
  drive = Drive(lanes_left=0, lanes_right=2, paint=three_lanes_and_a_bike_lane)
  drive.run(maneuver("turn", "right"), 500., 40.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.right] * 2 and (drive.lanes_left, drive.lanes_right) == (2, 0)


def test_yellow_line_stops_map_lane_changes():
  # the map says the leftmost of three lanes, but the model's two lanes on the left are beyond a yellow line
  drive = Drive(lanes_left=2, lanes_right=0, paint=lambda left, right: paint(left=SY, right=SW))
  drive.run(maneuver("turn", "left", THREE_LANES_LEFT_TURN), 500., 30.)
  assert not drive.offers


LaneMismatch = custom.TurnpilotPlan.LaneMismatch
LEFT_ONLY_AND_THROUGH = ((("left",), False), (("straight",), True), (("straight", "right"), True))


def test_turn_only_lane_where_the_route_goes_straight():
  # three lanes at an intersection the route goes straight through, 40 m ahead at 10 m/s (the planner changes no lane
  # closer than 60 m): in the left-turn-only lane it is flagged, in the through lanes not
  in_left = model_lanes(*road(0, 2))
  assert lane_mismatch(None, 500., LEFT_ONLY_AND_THROUGH, 40., in_left, 10.) == LaneMismatch.turnOnlyGoingStraight
  for lanes in (model_lanes(*road(1, 1)), model_lanes(*road(2, 0))):
    assert lane_mismatch(None, 500., LEFT_ONLY_AND_THROUGH, 40., lanes, 10.) == LaneMismatch.none
  # further out the lane planner still has time; past it, nothing; a turn before it comes first
  assert lane_mismatch(None, 500., LEFT_ONLY_AND_THROUGH, 100., in_left, 10.) == LaneMismatch.none
  assert lane_mismatch(None, 500., LEFT_ONLY_AND_THROUGH, -5., in_left, 10.) == LaneMismatch.none
  assert lane_mismatch(maneuver("turn", "left", THREE_LANES_LEFT_TURN), 20., LEFT_ONLY_AND_THROUGH, 40., in_left, 10.) == \
         LaneMismatch.none
  # without the model reading the ego lane's lines the lane count means nothing
  unsure = model_lanes(*road(0, 2, line_prob=0.3))
  assert lane_mismatch(None, 500., LEFT_ONLY_AND_THROUGH, 40., unsure, 10.) == LaneMismatch.none


def test_wrong_lane_for_the_turn():
  # the route turns left from the left-turn lane; the car is still in a through lane 40 m out
  left_turn = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  assert lane_mismatch(left_turn, 40., (), 0., model_lanes(*road(1, 1)), 10.) == LaneMismatch.wrongLaneForTurn
  assert lane_mismatch(left_turn, 40., (), 0., model_lanes(*road(0, 2)), 10.) == LaneMismatch.none
  assert lane_mismatch(left_turn, 150., (), 0., model_lanes(*road(1, 1)), 10.) == LaneMismatch.none  # the planner's turn
  # a right from a lane marked straight-and-right is fine
  right_turn = maneuver("turn", "right", ((("straight",), False), (("straight", "right"), False)))
  assert lane_mismatch(right_turn, 40., (), 0., model_lanes(*road(1, 0)), 10.) == LaneMismatch.none
  assert lane_mismatch(right_turn, 40., (), 0., model_lanes(*road(0, 1)), 10.) == LaneMismatch.wrongLaneForTurn



# BeamNG shakedown lane_change (wpTown_5_left, 2026-10-27): a left turn from a road of two lanes each way, a double solid
# yellow centreline and no edge line on the curb lane. TurnPilot changed left once, into the turn lane, then again across
# the double yellow, and drove ~100 m in the oncoming lane to the junction. Rows: modelV2 at SAMPLE_IDX, as logged.
XING = json.loads(Path(__file__).with_name('lane_change_xing.json').read_text())['rows']
XING_PRE = ([-4.68, -1.3, 1.96, 3.73], [0.79, 0.88, 0.11, 0.0], [-10.05, 2.74], [1.38, 0.5])   # 22.0 s, in the curb lane
XING_POST = ([-4.94, -1.44, 2.0, 4.92], [0.55, 0.98, 0.69, 0.15], [-7.62, 6.39], [1.09, 0.88])  # 31.0 s, in the turn lane
TWO_LANES_LEFT_TURN = ((("left",), True), (("straight", "right"), False))


def test_an_unpainted_curb_side_is_unknown():
  pre, post = model_lanes(*XING_PRE), model_lanes(*XING_POST)
  # in the curb lane the trusted road edge is right beside the ego line: nothing that way, known
  assert pre.right_known and ego_lane_index(2, pre, True) == 1
  # one lane in, the next line out is unpainted but the trusted edge is a lane away: the model can't count that lane
  assert counts(post) == (2, 0) and not post.right_known and ego_lane_index(2, post, True) is None
  assert model_lanes(*road(1, 1)).right_known and model_lanes(*road(0, 0)).right_known
  assert not model_lanes(*road(1, 1, edge_std=2.)).right_known  # an unsure edge: the paint alone can't say how many
  assert not model_lanes_from_msg(messaging.new_message('modelV2').modelV2).right_known


def test_logged_crossing_rows():
  m = maneuver("turn", "left", TWO_LANES_LEFT_TURN)
  pre, post = model_lanes(*XING_PRE), model_lanes(*XING_POST)
  misread = paint(left=BW, right=BW)  # the double yellow misread broken white: the paint gate is the only other barrier
  assert lane_changes_needed(m, pre, paint=misread) == 1  # the first, right, change
  # after it: the curb side can't be counted, and the one change the two lanes ever need is done
  assert desired_changes(m, post) == 0
  assert desired_changes(m, post, done=1) == 0 and lane_changes_needed(m, post, paint=misread, done=1) == 0
  # the model's two lanes to the left are the oncoming ones: the map's lane list has none there
  assert change_blocker(LEFT, post, misread) == ''
  assert change_blocker(LEFT, post, misread, nav_room=0) == 'noLane'


def replay_xing(paint_fn):
  lp, m = LanePlanner(), maneuver("turn", "left", TWO_LANES_LEFT_TURN)
  out = []
  for t, md, v, lb, rb, lcs, ly, lpr, ey, es in XING:
    state = getattr(LaneChangeState, lcs if lcs in ('off', 'preLaneChange') else 'laneChange' + lcs)
    d = lp.update(m, md, model_lanes(ly, lpr, ey, es), state, v, bool(lb), bool(rb), (), False, True, DT_MDL, paint_fn(t),
                  maneuver_id=0)
    out.append((t, d))
  return lp, out


def test_logged_crossing_replay_changes_once():
  # every line misread broken white: only the lane planner itself stands between the car and the oncoming lane
  lp, out = replay_xing(lambda t: paint(left=BW, right=BW))
  first = [t for t, d in out if d.offer == LaneChange.left and t < 22.6]
  assert first  # the change into the turn lane is still offered
  first_end = next(t for t, _, _, _, _, lcs, *_ in XING if t > 23. and lcs == 'off')
  second_start = next(t for t, _, _, _, _, lcs, *_ in XING if t > first_end and lcs == 'Starting')
  between = [(t, d) for t, d in out if first_end <= t < second_start]
  assert not [t for t, d in between if d.offer != LaneChange.none or d.lane_change != LaneChange.none], \
    'a second change offered across the centreline'
  assert not lp.lane_unknown  # dead reckoned: the turn lane, confirmed


def test_unpainted_curb_two_way_road_changes_once():
  # two lanes each way, the model's left edge beyond the oncoming lanes, every line read broken white
  drive = Drive(lanes_left=3, lanes_right=0, curb_unpainted=True, paint=lambda left, right: paint(left=BW, right=BW))
  drive.run(maneuver("turn", "left", TWO_LANES_LEFT_TURN), 400., 30.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.left] and (drive.lanes_left, drive.lanes_right) == (2, 1)


def test_unpainted_curb_three_lanes_dead_reckons_both_changes():
  drive = Drive(lanes_left=2, lanes_right=0, curb_unpainted=True)
  drive.run(maneuver("turn", "left", THREE_LANES_LEFT_TURN), 500., 40.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.left] * 2 and (drive.lanes_left, drive.lanes_right) == (0, 2)
  assert not drive.aborts


def test_lane_not_confirmed_is_said_not_changed():
  # one lane in from an unpainted curb with nothing to dead reckon from: the curb lane or the turn lane, it can't say
  drive = Drive(lanes_left=1, lanes_right=1, curb_unpainted=True, paint=lambda left, right: paint(left=BW, right=BW))
  m = maneuver("turn", "left", ((("left",), True), (("straight",), False), (("straight", "right"), False)))
  drive.run(m, 250., 3.)
  assert not drive.offers and drive.lp.lane_unknown and drive.decisions[-1].reason == LANE_UNKNOWN_REASON
  # past the window too, to the junction: nothing else says the lane there
  drive = Drive(lanes_left=1, lanes_right=1, curb_unpainted=True)
  drive.run(m, 40., 1.)
  assert drive.lp.lane_unknown
  # outside the window it says nothing
  drive = Drive(lanes_left=1, lanes_right=1, curb_unpainted=True)
  drive.run(m, 900., 1.)
  assert not drive.lp.lane_unknown


def test_completed_changes_are_counted_per_maneuver():
  drive = Drive(lanes_left=1, lanes_right=0)
  drive.run(maneuver("turn", "left", TWO_LANES_LEFT_TURN), 300., 15.)
  assert drive.lp.done == 1 and drive.lp.changes == 1
  drive.step(maneuver("turn", "left", TWO_LANES_LEFT_TURN), 100., maneuver_id=1)  # a new maneuver: counted afresh
  assert drive.lp.done == 0


class DroppedLateralDrive(Drive):
  """Drive, with openpilot's lateral control dropping for drop_s (a brake tap, a steering disengage) just after the
  change number `which` (from 1) starts: DesireHelper ends it, the car still in its lane."""
  def __init__(self, *a, which=1, drop_s=0.5, **kw):
    super().__init__(*a, **kw)
    self.which, self.drop_s, self.starts, self.drop_left = which, drop_s, 0, 0.
    self._was_starting = False

  def step(self, m, distance, **kw):
    starting = self.dh.lane_change_state == LaneChangeState.laneChangeStarting
    if starting and not self._was_starting:
      self.starts += 1
      if self.starts == self.which:
        self.drop_left = self.drop_s
    self._was_starting = starting
    lat = self.drop_left <= 0.
    self.drop_left -= DT_MDL
    update = self.dh.update
    self.dh.update = lambda cs, active, prob, req: update(cs, active and lat, prob, req)
    try:
      return super().step(m, distance, **kw)
    finally:
      self.dh.update = update


def test_a_change_that_ends_with_the_car_in_its_lane_isnt_counted():
  # lateral control drops just after one of the two changes into a three-lane road's left-turn lane starts, the curb lane
  # unpainted: DesireHelper ends the change with the car still in its lane. It isn't counted as made: the car is never
  # taken for in the turn lane while it isn't, and its lane can't be confirmed, said out loud
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  for which in (1, 2):
    drive = DroppedLateralDrive(lanes_left=2, lanes_right=0, curb_unpainted=True, which=which)
    drive.run(m, 500., 28.)  # to 80 m before the turn, past the last lane change
    in_turn_lane = drive.lanes_left == 0
    assert in_turn_lane or drive.lp.lane_unknown, (which, drive.lanes_left, drive.lp.done, drive.lp._start)
  # without the drop both changes are counted, and the car is in the turn lane, confirmed
  drive = Drive(lanes_left=2, lanes_right=0, curb_unpainted=True)
  drive.run(m, 500., 28.)
  assert drive.lanes_left == 0 and drive.lp.done == 2 and not drive.lp.lane_unknown


def test_a_change_that_didnt_move_the_car_never_sends_it_across_the_centreline():
  # in the left-turn lane of two (double yellow to the left, misread broken white). The driver signals right and
  # DesireHelper starts the change for 1 s, then ends it with the car still in its lane: no change left is wanted after it,
  # whether the model can place the car (the far-right line painted) or not
  m = maneuver("turn", "left", TWO_LANES_LEFT_TURN)
  known = ([-4.94, -1.44, 2.0, 5.3], [0.55, 0.98, 0.69, 0.75], XING_POST[2], XING_POST[3])
  for rows in (XING_POST, known):
    lp, dist, v = LanePlanner(), 260., 12.
    lanes = model_lanes(*rows)
    def step(state, right=False, lp=lp, lanes=lanes, v=v):
      nonlocal dist
      dist -= v * DT_MDL
      return lp.update(m, dist, lanes, state, v, False, right, (), False, True, DT_MDL, paint(left=BW, right=BW), maneuver_id=7)
    for _ in range(10):
      step(LaneChangeState.off)
    for _ in range(5):
      step(LaneChangeState.preLaneChange, True)
    for _ in range(20):
      step(LaneChangeState.laneChangeStarting, True)
    out = [step(LaneChangeState.off) for _ in range(200)]
    assert not [d for d in out if d.offer == LaneChange.left or d.lane_change == LaneChange.left], rows is known


def test_a_lane_the_model_places_is_never_moved_by_the_count_of_changes():
  # the model places the car in the curb lane of two (index 1), but one change left is counted from there: the two
  # disagree, so the car may be in either, never only where the count says
  pre = model_lanes(*XING_PRE)
  assert ego_lane_index(2, pre, True) == 1
  lo, hi = nav_lane_span(2, pre, True, 1, (1, 0))
  assert lo <= 1 <= hi
  # one change left counted, with the model placing the car in the right-hand lane: the two can't both hold, it may be anywhere
  assert nav_lane_span(2, pre, True, 1, None) == (0, 1)


def test_a_lane_crossed_without_desirehelper_leaves_the_lane_unknown():
  # three lanes our way and two oncoming, every line misread broken white, the curb lane unpainted and the line two over
  # faint: in the lanes away from the curb the model can't count to it. After one change of ours, the driver moves over to
  # the turn lane himself with openpilot's lateral off (DesireHelper never changes). The model's lines show the car cross:
  # the count of changes no longer places the car, so no change is asked for toward the oncoming lanes
  oncoming = 2
  class FaintCurb(Drive):
    def step(self, m, distance, **kw):
      global road
      plain = road
      def faint(left, right, **k):
        ys, probs, edges, stds = plain(left, right, **k)
        if right >= 1:
          probs[3] = 0.4 if right == 2 else 0.05
        return ys, probs, edges, stds
      road = faint
      try:
        return super().step(m, distance, **kw)
      finally:
        road = plain
  drive = FaintCurb(lanes_left=oncoming + 2, lanes_right=0, paint=lambda left, right: paint(left=BW, right=BW if right else SW))
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  dist = 500.
  while drive.lp.done != 1:
    dist = drive.run(m, dist, DT_MDL)
    assert dist > 0.
  dist = drive.run(m, dist, 0.5)
  frames = int(3. / DT_MDL)
  for i in range(frames):  # the driver's change: the lines slide right, and relabel as the car crosses
    frac = (i + 1) / frames
    if i == frames // 2:
      drive.lanes_left, drive.lanes_right = drive.lanes_left - 1, drive.lanes_right + 1
    drive.offset = LANE_WIDTH * (frac if i < frames // 2 else frac - 1.)
    drive.step(m, dist)
    dist -= drive.v * DT_MDL
  assert drive.lanes_left == oncoming
  n = len(drive.requests)
  drive.run(m, dist, 15.)
  assert not drive.requests[n:], 'a change asked for into the oncoming lane'
  assert drive.lp.done is None


class StopDrive(Drive):
  """Drive with a map stop (a signal) stop_at m before the maneuver: the car slows for it on the map stop's curve
  (MAP_STOP_DECEL, a second's lag), braking no harder than 2 m/s^2, stands hold_s at its line, then pulls away at 1 m/s^2
  to cruise. The planner gets the stop's distance until the go, as MapControlPolicy.stop_ahead() gives it."""
  def __init__(self, lanes_left, lanes_right, cruise, stop_at, hold_s=3., v=None, **kw):
    super().__init__(lanes_left, lanes_right, v=cruise if v is None else v, **kw)
    self.cruise, self.stop_at, self.hold_s = cruise, stop_at, hold_s
    self.held_t, self.gone = 0., False
    self.waits, self.min_v_before_stop = [], cruise

  def drive(self, m, distance, seconds, **kw):
    for _ in range(int(round(seconds / DT_MDL))):
      stop_d = None if self.gone else max(distance - self.stop_at, 0.)
      d = self.step(m, distance, stop_d=stop_d, **kw)
      self.waits.append((distance, self.v, d.wait))
      if stop_d is not None and stop_d < 0.5 and self.v < 0.3:
        self.held_t += DT_MDL
        self.gone = self.held_t >= self.hold_s
        self.v = 0.
        continue
      target = self.cruise if stop_d is None else min(self.cruise, math.sqrt(3. * max(stop_d - self.v - 0.5, 0.)))
      self.v = max(min(target, self.v + 1. * DT_MDL), self.v - 2. * DT_MDL, 0.)
      distance -= self.v * DT_MDL
    return distance


def test_a_signal_inside_the_window_gets_the_change_offered_before_it_at_speed():
  """BeamNG lane_change on the signal tiles, 2026-10-28: a signal 164 m before the turn sat inside the lane change window,
  and braking for it kept the car under DesireHelper's 9.44 m/s from 227 m in: no change was offered, and nothing in the
  plan said why. The window opens early enough for the change to start before the stop's braking takes the car below it."""
  drive = StopDrive(1, 0, cruise=11.35, stop_at=164.)
  m = maneuver("turn", "left")
  drive.drive(m, 450., 40.)
  assert drive.offers and drive.offers[0][1] > 164. + slow_zone(11.35), drive.offers
  req = drive.requests[0][1]
  assert req > 164. + slow_zone(11.35) and drive.lp.changes >= 1, (drive.requests, drive.aborts)
  assert not [a for a, _ in drive.aborts if a == 'tooSlow']
  # before its window, the plan said why there was no offer yet
  assert any(w == 'beforeWindow' for d, _, w in drive.waits if d > drive.offers[0][1] + 5.)


def test_with_no_room_before_the_stop_the_change_goes_after_its_go():
  """The stop seen too close to change lanes before it at speed, with room after it: no offer, said in the plan
  (afterStop), and once the car is away and up to speed, offered and made."""
  drive = StopDrive(1, 0, cruise=12., stop_at=175., hold_s=1000.)
  m = maneuver("turn", "left")
  distance = drive.drive(m, 215., 25.)  # up to the stop and held there
  assert not drive.offers and drive.v == 0. and drive.held_t > 1.
  waits = {w for d, _, w in drive.waits if d < 210.}  # afterStop at speed, beforeWindow once slow (the window shrinks)
  assert 'afterStop' in waits and waits <= {'afterStop', 'beforeWindow'}
  drive.cruise, drive.hold_s = 13., 0.
  drive.drive(m, distance, 25.)
  assert drive.offers and drive.lp.changes == 1, (drive.offers, drive.aborts, drive.waits[-1])
  assert all(60. < d < 180. for _, d, _ in drive.offers)


def test_with_no_room_before_the_stop_nor_after_it_the_lane_is_unreachable_and_said():
  """A stop 48 m before the turn, seen 52 m ahead: not enough road after its go to get up to speed and change. Too close
  to it at speed for one before it. The turn's lane can't be reached: unreachable, never a late change."""
  drive = StopDrive(2, 0, cruise=10., stop_at=48.)
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  drive.drive(m, 100., 4.)
  d = drive.decisions[-1]
  assert not drive.offers and d.wait == 'unreachable' and d.unreachable and d.want == 2
  assert 'reach' in d.reason
  # without the map's lane list the car's lane isn't known to be wrong: said, never a skip
  drive = StopDrive(1, 0, cruise=10., stop_at=48.)
  drive.drive(maneuver("turn", "left"), 100., 4.)
  d = drive.decisions[-1]
  assert d.wait == 'unreachable' and not d.unreachable


def test_lane_change_wait_says_why_no_change_is_offered():
  drive = Drive(1, 0, v=2.)  # below the lane change speed (NAV_LANE_CHANGE_SPEED_MIN)
  m = maneuver("turn", "left")
  drive.run(m, 120., 1.)
  assert drive.decisions[-1].wait == 'tooSlow' and drive.decisions[-1].want == 1
  drive = Drive(1, 0, v=15., paint=lambda l, r: paint(left=SW, right=SW))
  drive.run(m, 200., 1.)
  assert drive.decisions[-1].wait == 'lineNotBroken'
  drive = Drive(1, 0, v=15.)
  drive.run(m, 600., 0.2)
  assert drive.decisions[-1].wait == 'beforeWindow'
  drive.run(m, 200., 0.2)
  assert drive.decisions[-1].wait == 'confirming'
  drive = Drive(0, 0, v=15., paint=lambda l, r: paint(left=BW, right=SW))  # broken white, and no lane beyond it
  drive.run(m, 200., 1.)
  assert drive.decisions[-1].wait == 'noLane'
  drive = Drive(1, 0, v=15., paint_rule=False)
  drive.run(m, 200., 1.)
  assert drive.decisions[-1].wait == 'paintRuleOff'


def test_an_offer_made_with_room_before_the_stop_is_never_called_unreachable_on_its_way():
  """The map's lane list wants one change left; the stop 100 m before the turn leaves no room after its go. The change
  offered before it with room goes on to be made: the room running short during its own warning isn't an unreachable
  lane (that would skip the turn the car is changing lanes for)."""
  drive = StopDrive(1, 1, cruise=11.35, stop_at=100.)
  drive.drive(maneuver("turn", "left", THREE_LANES_LEFT_TURN), 300., 30.)
  assert drive.offers and drive.lp.changes == 1, (drive.offers, drive.aborts)
  assert not [a for a, _ in drive.aborts if a == 'tooSlow']
  assert not any(d.unreachable for d in drive.decisions)


# ---------------------------------------------------------------------------------------- the car's own signal (car_signals.py)

def car_drive(v=15., lead=91.44):
  drive = Drive(lanes_left=0, lanes_right=1, v=v)
  drive.car_kw = {'lit_m': 0., 'lead_m': lead}
  drive.car_lamp = True
  return drive


def test_car_signal_waits_for_the_legal_lead():
  # the car's lamp goes on with the warning; the request waits, past the countdown, until it has led by 300 ft
  drive = car_drive()
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  drive.run(m, 400., 20.)
  assert drive.requests and not drive.aborts
  (_, _, t_offer), (_, _, t_req) = drive.offers[0], drive.requests[0]
  assert t_req - t_offer >= 91.44 / drive.v - 2 * DT_MDL and t_req - t_offer > LC_WARN_S
  assert all(p == LaneChange.none for p in drive.prompts)  # never prompts


def test_car_signal_short_window_is_too_late():
  drive = car_drive(v=12.)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  drive.run(m, 140., 12.)
  assert not drive.requests and [a for a, _ in drive.aborts][:1] in (['tooLate'], [])


def test_where_the_car_signals_the_window_has_the_car_in_its_lane_by_its_end():
  """The first drive (2026-11-01): the car into its turn lane only at the light. The lamp's 300 ft took most of a city window,
  the change starting by its end at best (at 18 mph the offer at 181 m, the change at 84 m, in the lane 50 m out), and a
  second change got no time at all. With the offer's room the window opens early enough for the whole of the change, and a
  driver's warning (LC_OFFER_ROOM_S) leaves every window as it was; nor does a highway's past LC_START_MAX_M."""
  from openpilot.selfdrive.turnpilot.lane_planner import (LC_CHANGE_S, LC_OFFER_ROOM_S, LC_SPARE_S, LC_START_MAX_M, LC_EXTRA_LANE_S,
                                                          offer_room_s)
  for v in (3., 5.4, 8., 11.2, 13.4, 15.6, 20., 25., 31.):
    for n in (1, 2):
      assert lane_change_window(v, n, LC_OFFER_ROOM_S) == lane_change_window(v, n), (v, n)
    start, end = lane_change_window(v, 1, offer_room_s(v, 91.44))
    assert start >= lane_change_window(v)[0] and lane_change_window(v, 1, offer_room_s(v, 91.44))[1] == lane_change_window(v)[1]
    need = end + v * (LC_CONFIRM_FRAMES * DT_MDL + offer_room_s(v, 91.44) + LC_CHANGE_S + LC_SPARE_S)
    assert start == pytest.approx(max(lane_change_window(v)[0], min(need, LC_START_MAX_M))), v
    assert lane_change_window(v, 2, offer_room_s(v, 91.44))[0] == pytest.approx(start + LC_EXTRA_LANE_S * v)
  assert lane_change_window(31., 1, offer_room_s(31., 91.44)) == lane_change_window(31.)  # a highway's: as it was
  # closed loop at 18 mph, the car's lamp leading the change by 300 ft: in the turn lane before the window's end
  for v in (5.4, 8.):
    drive = car_drive(v=v)
    m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
    start, end = lane_change_window(v, 1, offer_room_s(v, 91.44))
    distance = start + 10.
    while drive.lp.changes == 0 and distance > 0.:
      drive.step(m, distance)
      distance -= v * DT_MDL
    assert drive.lp.changes == 1 and not drive.aborts and distance > end, (v, distance, end)
    assert drive.offers[0][1] > lane_change_window(v)[0]  # earlier than the window was


def centre_line(lanes_left, lanes_right):
  """Our lanes beside a solid yellow centre line: broken white between them, solid white at the curb, the next line out of
  each ego line read too."""
  left, far_left = (BW, SY if lanes_left == 1 else BW) if lanes_left else (SY, UNKNOWN)
  right, far_right = (BW, SW if lanes_right == 1 else BW) if lanes_right else (SW, UNKNOWN)
  return far_left, left, right, far_right


def test_a_centre_line_read_as_a_change_ends_is_held_on_its_own_line():
  """test_fundamentals' three-lane left: the centre line comes into view as the far-left line when the first change ends. The
  yellow holds moved with the lines after that frame's paint went into them, so its yellow came to sit on the ego line: the
  second change toward it read lineNotBroken for YELLOW_HOLD_M, and the turn was skipped. On a two-lane road the change
  into the lane by the centre line put its yellow on the ego right line: the curb side counted 0, and the turn lane the car
  was in said "Can't Confirm Lane". The holds now move before the frame's paint goes in."""
  left = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  drive = Drive(lanes_left=2, lanes_right=0, paint=centre_line)
  drive.run(left, 500., 40.)
  assert drive.lp.changes == 2 and (drive.lanes_left, drive.lanes_right) == (0, 2) and not drive.aborts
  assert 'lineNotBroken' not in {d.wait for d in drive.decisions}
  two = maneuver("turn", "left", ((("left",), True), (("straight",), False)))
  drive = Drive(lanes_left=1, lanes_right=0, paint=centre_line)
  drive.run(two, 400., 30.)
  assert drive.lp.changes == 1 and (drive.lanes_left, drive.lanes_right) == (0, 1)
  assert not drive.lp.lane_unknown and 'laneUnknown' not in {d.wait for d in drive.decisions}


def test_the_cars_lamp_stays_lit_between_two_changes_the_same_way():
  """Where the car signals, its lamp stays on its side from the first of two changes the same way to the second (signal_side
  through the cooldown): one signal across both lanes, and the second change's lead is the lamp's whole run, so it goes at its
  warning's end, never another 300 ft on. Dark between them, its lead started over and a three-lane road's second change had
  no window left. The carry ends with the changes the route wants that way."""
  drive = Drive(lanes_left=2, lanes_right=0, v=11.2)
  drive.car_kw, drive.car_lamp = {'lit_m': 0., 'lead_m': 91.44}, True
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  distance, sides = 500., []
  while drive.lp.changes < 2 and distance > 0.:
    drive.step(m, distance)
    sides.append(drive.lp.signal_side)
    distance -= drive.v * DT_MDL
  assert drive.lp.changes == 2 and not drive.aborts and len(drive.offers) == 2
  first = next(i for i, s in enumerate(sides) if s != LaneChange.none)
  assert set(sides[first:-1]) == {LaneChange.left}  # (the last: the second change over, nothing more wanted that way)
  (_, _, t_offer), (_, _, t_req) = drive.offers[1], drive.requests[1]
  assert abs((t_req - t_offer) - LC_WARN_S) < 2 * DT_MDL  # its lead already lit: the warning alone
  drive.run(m, distance, 5.)
  assert drive.lp.signal_side == LaneChange.none  # in the turn lane: nothing carried


def first_of_two_lefts():
  """A three-lane road at 25 mph, a left wanting both changes, the car working its own lamp with the 300 ft lead: driven to
  the frame its first change ends, the lamp carried for the second (signal_side), the cooldown begun."""
  drive = Drive(lanes_left=2, lanes_right=0, v=11.2)
  drive.car_kw, drive.car_lamp = {'lit_m': 0., 'lead_m': 91.44}, True
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  distance = 500.
  while drive.lp.changes < 1:
    drive.step(m, distance)
    distance -= drive.v * DT_MDL
  assert drive.lp.phase == Phase.IDLE and drive.lp.signal_side == LaneChange.left and len(drive.offers) == 1
  assert drive.decisions[-1].wait == 'cooldown'
  return drive, m, distance


def lamp_run(drive, m, distance, seconds, **kw):
  """Drive.run, with the planner's side for the car's lamp (signal_side) on every frame."""
  sides = []
  for _ in range(int(round(seconds / DT_MDL))):
    drive.step(m, distance, **kw)
    distance -= drive.v * DT_MDL
    sides.append(drive.lp.signal_side)
  return distance, sides


def test_the_lamp_carried_between_two_changes_goes_out_on_a_line_unread():
  """The review's carry probe (2026-11-06): with the left ego line read unknown after the first of two changes left, the lamp
  carried for the second (signal_side) stayed lit 13.35 s with no offer and no change, to the window's end. The unsure cap
  bounded only an offer's wait; the carry is no offer. Now LC_UNSURE_MAX_S past the cooldown with no frame passing
  change_blocker that way by its own read puts it out, silently (nothing was offered, so nothing aborts: the wait says why):
  on a line whose kind reads unknown (no hold reaches it), and on broken of unknown colour valid by the white hold alone."""
  drive, m, distance = first_of_two_lefts()
  drive.night = True  # the line beyond the next lane unseen: nothing agrees a lane is there (lane_agreed)
  drive.paint = lambda left, right: paint(left=UNKNOWN, right=BW)
  distance, sides = lamp_run(drive, m, distance, LC_COOLDOWN_S + 2 * LC_UNSURE_MAX_S)
  lit = sides.index(LaneChange.none)
  assert lit * DT_MDL == pytest.approx(LC_COOLDOWN_S + LC_UNSURE_MAX_S, abs=2 * DT_MDL), "lit 13.35 s before"
  assert set(sides[lit:]) == {LaneChange.none}, "dark stays dark: nothing offered on the unread line"
  assert len(drive.offers) == 1 and len(drive.requests) == 1 and not drive.aborts and drive.lp.changes == 1
  assert {d.offer for d in drive.decisions[-len(sides):]} == {LaneChange.none}  # IDLE throughout, the carry no offer
  assert drive.decisions[-1].wait == 'lineNotBroken' and drive.decisions[-1].want == 1
  # broken of unknown colour after broken white read through the cooldown: valid by the white hold alone (wait confirming,
  # nothing confirmed on it), which no more keeps the lamp lit than the unread line
  drive, m, distance = first_of_two_lefts()
  drive.night = True
  while drive.decisions[-1].wait == 'cooldown':
    drive.step(m, distance)
    distance -= drive.v * DT_MDL
  drive.paint = lambda left, right: paint(left=COLORLESS, right=BW)
  distance, sides = lamp_run(drive, m, distance, 2 * LC_UNSURE_MAX_S)
  lit = sides.index(LaneChange.none)
  assert lit * DT_MDL == pytest.approx(LC_UNSURE_MAX_S, abs=2 * DT_MDL)
  assert {d.wait for d in drive.decisions[-len(sides):][:lit]} == {'confirming'}, "the hold's frames, not the line's own"
  assert len(drive.offers) == 1 and not drive.aborts


def test_the_carried_lamp_waits_out_a_short_dropout_and_a_car_beside():
  """The carry still does its job across what an offer's wait waits out. The line unread for LC_UNSURE_MAX_S - 0.2 s past the
  cooldown, then broken white: the second change is offered with the lamp never dark, and it goes at its warning's end, its
  lead the lamp's whole run (dark for a frame, its 300 ft would start over). The clock runs from the last frame passing
  change_blocker, as an offer's wait does: two unsure stretches under the cap, with broken white read between them on fewer
  frames than an offer is confirmed on, keep the lamp lit too. A car in the blind spot that way stops the clock, the car
  beside hiding the line (as LC-01's hold): the lamp stays lit while it is there, and goes out LC_UNSURE_MAX_S after it
  clears with the line still unread."""
  unread = lambda left, right: paint(left=UNKNOWN, right=BW)  # noqa: E731
  read = lambda left, right: paint(left=BW, right=BW)  # noqa: E731
  for stretches in ((unread, LC_UNSURE_MAX_S - 0.2),), \
                   ((unread, 2.), (read, (LC_CONFIRM_FRAMES - 3) * DT_MDL), (unread, 2.)):
    drive, m, distance = first_of_two_lefts()
    drive.night = True  # nothing agreeing a lane beyond the unread line (lane_agreed); broken white still counts the room
    drive.paint = unread
    sides = []
    while drive.decisions[-1].wait == 'cooldown':
      distance, s = lamp_run(drive, m, distance, DT_MDL)
      sides += s
    for p, seconds in stretches:
      drive.paint = p
      distance, s = lamp_run(drive, m, distance, seconds)
      sides += s
    assert len(drive.offers) == 1, stretches
    drive.paint = striped
    while len(drive.requests) < 2 and distance > 0.:
      distance, s = lamp_run(drive, m, distance, DT_MDL)
      sides += s
    assert set(sides) == {LaneChange.left}, ("lit from the first change to the second's request", stretches)
    assert len(drive.offers) == 2 and len(drive.requests) == 2 and not drive.aborts, stretches
    (_, _, t_offer), (_, _, t_req) = drive.offers[1], drive.requests[1]
    assert abs((t_req - t_offer) - LC_WARN_S) < 2 * DT_MDL, stretches  # its lead already lit: the warning alone
    drive.run(m, distance, 6.)
    assert drive.lp.changes == 2 and (drive.lanes_left, drive.lanes_right) == (0, 2), stretches
  # held by the blind spot: the clock stops while the car beside is there, and runs once it clears
  drive, m, distance = first_of_two_lefts()
  drive.night = True
  drive.paint = lambda left, right: paint(left=UNKNOWN, right=BW)
  distance, beside = lamp_run(drive, m, distance, LC_COOLDOWN_S + 1.5 * LC_UNSURE_MAX_S, blindspot=LaneChange.left)
  assert set(beside) == {LaneChange.left} and len(drive.offers) == 1 and not drive.aborts
  distance, sides = lamp_run(drive, m, distance, LC_UNSURE_MAX_S + 0.5)
  lit = sides.index(LaneChange.none)
  assert lit * DT_MDL == pytest.approx(LC_UNSURE_MAX_S, abs=2 * DT_MDL) and set(sides[lit:]) == {LaneChange.none}
  assert len(drive.offers) == 1 and not drive.aborts


def test_car_signal_failed_mid_offer_prompts_the_driver():
  drive = car_drive()
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 300., 2.)
  assert drive.offers and drive.prompts[-1] == LaneChange.none
  drive.car_lamp = False
  drive.car_kw = {'lit_m': 0., 'lead_m': 91.44, 'signals_failed': True}
  distance = drive.run(m, distance, LC_WARN_S)
  assert drive.lp.phase == Phase.WAIT and drive.prompts[-1] == LaneChange.right
  drive.run(m, distance, 3., blinker=LaneChange.right)
  assert drive.requests


def test_driver_offer_keeps_the_driver_path_when_ready():
  drive = Drive(lanes_left=0, lanes_right=1, driver_signals=True)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 300., LC_WARN_S + 1.)
  assert drive.lp.phase == Phase.WAIT
  drive.driver_signals = False  # the car's signal turns ready mid-WAIT: this offer stays the driver's
  drive.car_kw = {'lit_m': 999., 'lead_m': 91.44}
  drive.run(m, distance, 1.)
  assert drive.lp.phase == Phase.WAIT and drive.prompts[-1] == LaneChange.right and not drive.requests


def test_car_block_aborts_no_signal():
  # the driver's lamp the other way, or hazards, while the car signals the change: the driver declines it
  drive = car_drive()
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 300., 1.5)
  assert drive.lp.phase == Phase.WARN
  drive.car_kw = {**drive.car_kw, 'car_block': True}
  drive.run(m, distance, 1.)
  assert [a for a, _ in drive.aborts] == ['noSignal'] and not drive.requests


def test_offer_room_uses_the_lead():
  from openpilot.selfdrive.turnpilot.lane_planner import LC_OFFER_ROOM_S, offer_room_s
  assert offer_room_s(11.2, None) == LC_OFFER_ROOM_S
  assert offer_room_s(11.2, 91.44) == pytest.approx(91.44 / 11.2 + 1.5)  # 25 mph: 9.7 s
  assert offer_room_s(40., 91.44) == pytest.approx(LC_WARN_S + 1.5)  # fast enough that the warning is longer


# ----------------------------------------------------------------------- LC-03: a change that never went (ALWAYS_DRIVE 4.11)

def through_the_window(drive, m, start, stop=20., **kw):
  """Drive from start m to stop m before the maneuver: [(distance, decision)] for every frame."""
  out, distance = [], start
  while distance > stop:
    out.append((distance, drive.step(m, distance, **kw)))
    distance -= drive.v * DT_MDL
  return out


def first_blocked(frames):
  return next((d for d, x in frames if x.blocked), None)


def assert_blocked_at_the_end(drive, frames):
  """LaneDecision.blocked from the first frame at or past the window's end on which nothing is offered, and on every frame
  after it for the maneuver; never before the window's end."""
  _, end = lane_change_window(drive.v)
  at = first_blocked(frames)
  assert at is not None and end - (LC_ABORT_SETTLE_S + 3. * DT_MDL) * drive.v < at <= end + 1e-6, (at, end)
  assert all(x.blocked for d, x in frames if d <= at)
  assert not any(x.blocked for d, x in frames if d > end + 1e-6)


TWO_LANE_RIGHT = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)


@pytest.mark.parametrize("always_drive", [False, True])
def test_a_change_the_lane_list_still_wants_at_the_windows_end_is_blocked_for_any_reason_but_no_lane(always_drive):
  """LC-03 (ALWAYS_DRIVE.md 4.11, unconditional): the map's lane list places the car in a lane the right turn doesn't leave
  from, and the change it needs hasn't gone by the window's end, whatever held it: the turn is skipped and rerouted
  (turnpilotd, skipReason laneBlocked), never pulsed from that lane. Each reason here: a car in the blind spot, a car in the
  target lane, too slow for DesireHelper (never offered), no signal from the driver, DesireHelper waiting too long, the
  lane parked, the line solid (of no colour read: one read solid white into the turn lane the list names may be crossed,
  change_blocker's bay, and that change is made). With TurnPilotAlwaysDrive off as well as on (step S4)."""
  beside = scene([{'cls': 'car', 'x': -3., 'y': -3.6, 'vx': 15.}]).objects
  parked = scene([{'cls': 'car', 'x': 50., 'y': -3.6, 'vx': 0.}]).objects
  cases = {
    'blindspot': (Drive(0, 1), {'blindspot': LaneChange.right}),
    'carInLane': (Drive(0, 1), {'objects': beside}),
    'tooSlow': (Drive(0, 1, v=2.), {}),
    'noSignal': (Drive(0, 1, driver_signals=True), {}),
    'laneParked': (Drive(0, 1), {'objects': parked}),
    'lineNotBroken': (Drive(0, 1, paint=lambda left, right: paint(right=LinePaint('solid', 'unknown', 0.9, 0.95))), {}),
  }
  for why, (drive, kw) in cases.items():
    drive.lp.always_drive = always_drive
    frames = through_the_window(drive, TWO_LANE_RIGHT, lane_change_window(drive.v)[0] + 30., **kw)
    assert not drive.requests or why == 'noSignal', why
    assert drive.lp.changes == 0, why
    assert_blocked_at_the_end(drive, frames)
  # DesireHelper never starting the change it was asked for (timedOut), on the planner alone
  lp = LanePlanner()
  lp.always_drive = always_drive
  args = {'lanes': model_lanes(*road(0, 1)), 'v_ego': 15., 'left_blinker': False, 'right_blinker': False, 'objects': (),
          'driver_signals': False, 'right_hand_traffic': True, 'dt': DT_MDL, 'paint': paint(right=BW)}
  distance, decisions = 320., []
  while distance > 20.:
    decisions.append((distance, lp.update(TWO_LANE_RIGHT, distance, lc_state=LaneChangeState.preLaneChange, **args)))
    distance -= 15. * DT_MDL
  assert any(x.abort == 'timedOut' for _, x in decisions)
  at = first_blocked(decisions)
  assert at is not None and at <= lane_change_window(15.)[1] and all(x.blocked for d, x in decisions if d <= at)


def test_no_lane_that_way_is_never_blocked():
  """The one exception: the model sees no lane the way the list says to change (noLane: the list may be wrong, or the car is
  at the edge already). The turn goes as today."""
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  drive = Drive(0, 0, paint=lambda left, right: paint(left=BW, right=SW))
  frames = through_the_window(drive, m, 350.)
  assert {x.wait for d, x in frames if 100. < d < 300.} == {'noLane'} and drive.decisions[-1].want == 2
  assert not any(x.blocked for _, x in frames)


@pytest.mark.parametrize("always_drive", [False, True])
def test_without_a_lane_list_the_car_is_placed_only_by_a_travel_lane_seen_that_way(always_drive):
  """No lane list: the car is placed in a lane the turn doesn't leave from only by a frame of the window on which the model
  read the lane that way and change_blocker found it a travel lane. A curb lane before a right (solid white beyond, or no
  lane), a bike lane behind solid white or a parked strip never places it: the turn goes as today. With TurnPilotAlwaysDrive
  off as well as on (step S4: 4.10's less than paint is never solid white)."""
  def made(drive):
    drive.lp.always_drive = always_drive
    return drive
  m = maneuver("turn", "right")
  start = lane_change_window(15.)[0] + 30.
  drive = made(Drive(0, 1))
  frames = through_the_window(drive, m, start, blindspot=LaneChange.right)
  assert not drive.offers and {x.wait for d, x in frames if 100. < d < 280.} == {'blindspot'}
  assert_blocked_at_the_end(drive, frames)
  drive = made(Drive(0, 1, v=2.))  # too slow, a travel lane read that way: never offered, blocked
  assert_blocked_at_the_end(drive, through_the_window(drive, m, lane_change_window(7.)[0] + 20.))
  for drive, kw in ((made(Drive(0, 0)), {}),  # the curb lane: solid white beyond it
                    (made(Drive(0, 1, paint=lambda left, right: paint(right=SW))), {}),  # a bike lane behind solid white
                    (made(Drive(0, 1)), {'objects': scene([{'cls': 'car', 'x': 50., 'y': -3.6, 'vx': 0.}]).objects})):  # a parked strip
    frames = through_the_window(drive, m, start, **kw)
    assert not any(x.blocked for _, x in frames), {x.wait for _, x in frames}


def test_a_change_made_in_the_window_is_never_blocked():
  """The change goes: with the lane list the car is in the turn's lane; without it, the travel lane read before the change is
  no evidence about the lane the car is in now (the curb lane, solid white beyond it)."""
  for m in (TWO_LANE_RIGHT, maneuver("turn", "right")):
    drive = Drive(0, 1)
    frames = through_the_window(drive, m, lane_change_window(15.)[0] + 30.)
    assert drive.lp.changes == 1 and not any(x.blocked for _, x in frames), m


def test_blocked_only_with_the_planner_on_and_a_lane_it_can_place():
  """TurnPilotLanePaintActive off plans no change, so blocks none; nor does a lane the list can't place (lane_unknown, 'Can't
  Confirm Lane' instead: R12)."""
  drive = Drive(0, 1, paint_rule=False, v=2.)
  assert not any(x.blocked for _, x in through_the_window(drive, TWO_LANE_RIGHT, 200.))
  three_right = maneuver("turn", "right", ((("straight",), False), (("straight",), False), (("right",), True)))
  drive = Drive(0, 1, curb_unpainted=True, v=2.)
  frames = through_the_window(drive, three_right, 200.)
  assert any(x.wait == 'laneUnknown' for _, x in frames) and not any(x.blocked for _, x in frames)


def test_blocked_holds_for_its_maneuver_and_ends_with_it():
  drive = Drive(0, 1, v=2.)
  frames = through_the_window(drive, TWO_LANE_RIGHT, 200.)
  assert frames[-1][1].blocked
  assert not drive.step(TWO_LANE_RIGHT, 300., maneuver_id=1).blocked  # the next maneuver (a reroute's): judged afresh


# Night (drive 1e, 2026-11-06, after dark): no lane change offered all drive. The model rarely sees the line beyond the ego
# lane's (lanes_toward), a dashed line's colour drops out between dashes (WhiteHold), one unsure frame killed an offer
# (LanePlanner._follow_up), and a far-side turn's bay not open yet wanted changes against the centre line (the bay rule).
WEAK_BW = LinePaint('broken', 'white', 0.65, 0.3)


def test_white_hold():
  """A white read keeps the line white across later reads of unknown colour, kind broken or unknown, for WHITE_HOLD_S or
  WHITE_HOLD_M, whichever ends first, at no more than the white read's confidence; never over solid, none or yellow, never
  on a frame nobody read. 1e seg 8-9: broken white at 0.8-1.0, then broken of unknown colour at 527.5 s."""
  hold = WhiteHold()
  assert hold.update(paint(left=BW), 15., DT_MDL, False)[1] == BW
  for _ in range(3):  # three reads of unknown colour after it: still white, so still broken white
    out = hold.update(paint(left=COLORLESS), 15., DT_MDL, False)
    assert out[1].is_('broken', 'white', BROKEN_WHITE_CONF_MIN) and out[1].confidence == BW.confidence
    assert broken_white(out, LEFT)
  out = hold.update(paint(left=UNKNOWN), 15., DT_MDL, False)
  assert (out[1].kind, out[1].color) == ('unknown', 'white') and not broken_white(out, LEFT), "the kind is never held"
  for p in (SW, LinePaint('solid', 'unknown', 0.9, 0.95), NO_LINE, SY, BY, LinePaint('unknown', 'yellow', 0.2, 0.), LATCHED_YELLOW):
    assert hold.update(paint(left=p), 15., DT_MDL, False)[1] == p, p
  assert hold.update(paint(right=COLORLESS), 15., DT_MDL, False)[2] == COLORLESS, "each line its own"
  assert hold.update((), 15., DT_MDL, False, read=False) == (), "nothing read: nothing made up"
  assert hold.update(paint(left=COLORLESS), 15., DT_MDL, False, read=False)[1] == COLORLESS, "the yellow hold's padding isn't a read"
  # a white read at the bar holds at its own confidence; one under it holds nothing
  weak = WhiteHold()
  weak.update(paint(left=WEAK_BW), 15., DT_MDL, False)
  assert weak.update(paint(left=COLORLESS), 15., DT_MDL, False)[1].confidence == WEAK_BW.confidence
  weak = WhiteHold()
  weak.update(paint(left=UNSURE_BW), 15., DT_MDL, False)
  assert weak.update(paint(left=COLORLESS), 15., DT_MDL, False)[1] == COLORLESS
  # until WHITE_HOLD_S at a crawl, WHITE_HOLD_M at speed: whichever comes first
  for v, last in ((2., WHITE_HOLD_S), (25., WHITE_HOLD_M / 25.)):
    hold = WhiteHold()
    hold.update(paint(left=BW), v, DT_MDL, False)
    n = int(round(last / DT_MDL))
    outs = [hold.update(paint(left=COLORLESS), v, DT_MDL, False)[1].color for _ in range(n + 2)]
    assert outs[:n - 1] == ['white'] * (n - 1) and outs[-1] == 'unknown', v
  # frozen in a lane change: no read counts; a change moves the holds with the lines, one of unknown direction drops them
  hold = WhiteHold()
  hold.update(paint(left=BW), 15., DT_MDL, True)
  assert hold.update(paint(left=COLORLESS), 15., DT_MDL, False)[1] == COLORLESS
  hold.update(paint(left=BW), 15., DT_MDL, False)
  hold.shift(LEFT)
  assert hold.update(paint(right=COLORLESS), 15., DT_MDL, False)[2].color == 'white'
  hold.shift(RIGHT)
  assert hold.update(paint(left=COLORLESS), 15., DT_MDL, False)[1].color == 'white'
  hold.shift(0)
  assert all(p == COLORLESS for p in hold.update((COLORLESS,) * 4, 15., DT_MDL, False))


def broken_white(p, side):
  return change_blocker(side, model_lanes(*road(1, 1)), p) == ''


def test_a_yellow_read_beats_the_white_hold():
  """The planner applies the yellow hold first: a yellow read after a white one is yellow, and stays yellow for the yellow
  hold whatever the white hold says (the yellow rail as it was)."""
  lp = LanePlanner()
  lanes = model_lanes(*road(1, 1))
  m = maneuver("turn", "left")
  args = (lanes, LaneChangeState.off, 15., False, False, (), False, True, DT_MDL)
  lp.update(m, 900., *args, paint(left=BW))
  lp.update(m, 900., *args, paint(left=SY))
  for p in (COLORLESS, BW, UNKNOWN):
    lp.update(m, 900., *args, paint(left=p))
    out = lp.white.apply(lp.yellow.apply(paint(left=p)))
    assert out[1] == LATCHED_YELLOW and change_blocker(LEFT, lanes, out) == 'lineNotBroken', p


def test_line_contrary():
  for p in (SW, SY, BY, NO_LINE, LATCHED_YELLOW, LinePaint('solid', 'unknown', 0.3, 0.9), LinePaint('unknown', 'yellow', 0.1, 0.)):
    assert line_contrary(paint(left=p), LEFT) and not line_contrary(paint(left=p), RIGHT), p
  for p in (UNKNOWN, COLORLESS, UNSURE_BW, LinePaint('unknown', 'white', 0.9, 0.3), BW):
    assert not line_contrary(paint(right=p), RIGHT), p
  assert not line_contrary((), LEFT)  # nothing read: the caller's (an offer aborts on it)


def offered_left(**kw):
  """A two-lane road, the car in the right lane, broken white between: a left offered, its countdown under way."""
  drive = Drive(lanes_left=1, lanes_right=0, **kw)
  m = maneuver("turn", "left")
  distance = drive.run(m, 290., LC_CONFIRM_FRAMES * DT_MDL + 0.5)
  assert drive.lp.phase == Phase.WARN and [o for o, _, _ in drive.offers] == [LaneChange.left]
  return drive, m, distance


def test_an_offer_waits_out_an_unsure_line_and_aborts_only_on_a_contrary_read():
  """_follow_up: an offer confirmed on broken white goes on through a frame read short of it with nothing contrary (the kind
  unknown, broken of unknown colour, broken white under BROKEN_WHITE_CONF_MIN), never requested on one, and aborts on solid,
  yellow or no line (1e seg 8-9 replay: one broken frame of unknown colour aborted the offer at 527.5 s), or once unsure for
  LC_UNSURE_MAX_S (test_an_unsure_wait_is_bounded_and_the_lamp_goes_out)."""
  for unsure in (UNKNOWN, COLORLESS, UNSURE_BW, LinePaint('unknown', 'white', 0.9, 0.3)):
    drive, m, distance = offered_left()
    drive.paint = lambda left, right, p=unsure: paint(left=p, right=SW)
    distance = drive.run(m, distance, DT_MDL)
    assert drive.lp.phase == Phase.WARN and drive.decisions[-1].offer == LaneChange.left, unsure
    drive.paint = striped
    drive.run(m, distance, 10.)
    assert not drive.aborts and [r for r, _, _ in drive.requests] == [LaneChange.left] and drive.lp.changes == 1, unsure
  # a run of them through the countdown's end, short of LC_UNSURE_MAX_S: the offer stays up, its countdown runs, and the
  # request waits for broken white
  drive, m, distance = offered_left()
  assert drive.lp.countdown < LC_UNSURE_MAX_S - 0.2
  drive.paint = lambda left, right: paint(left=UNKNOWN, right=SW)
  distance = drive.run(m, distance, LC_UNSURE_MAX_S - 0.2)
  assert not drive.aborts and not drive.requests and drive.lp.phase == Phase.REQUEST
  assert drive.decisions[-1].offer == LaneChange.left and drive.decisions[-1].reason == LINE_UNSURE_REASON
  drive.paint = striped
  drive.run(m, distance, 6.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.left] and drive.lp.changes == 1 and not drive.aborts
  # the contrary reads abort at once
  for contrary in (SW, SY, BY, NO_LINE, LinePaint('unknown', 'yellow', 0.2, 0.)):
    drive, m, distance = offered_left()
    drive.paint = lambda left, right, p=contrary: paint(left=p, right=SW)
    drive.run(m, distance, DT_MDL)
    assert [a for a, _ in drive.aborts] == ['lineNotBroken'] and not drive.requests, contrary
  # a frame with no paint read at all (the wide camera, a sweep, a stale scene) waits as an unsure one does: before
  # 2026-11-07 it aborted the offer (1e 782.5 s), Kellen: no give-ups for lack of a read. Never requested on it
  drive, m, distance = offered_left()
  drive.paint = None
  distance = drive.run(m, distance, LC_UNSURE_MAX_S - 0.2)
  assert not drive.aborts and not drive.requests and drive.decisions[-1].offer == LaneChange.left
  drive.paint = striped
  drive.run(m, distance, 6.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.left] and drive.lp.changes == 1 and not drive.aborts
  # unsure, the rest of the lane's checks still run: a parked lane aborts
  drive, m, distance = offered_left()
  drive.paint = lambda left, right: paint(left=UNKNOWN, right=SW)
  drive.run(m, distance, DT_MDL, objects=scene([{'cls': 'car', 'x': 40., 'y': 3.6, 'vx': 0.}]).objects)
  assert [a for a, _ in drive.aborts] == ['laneParked']
  # unsure all the way: before, only the window's end bounded it (tooLate, 14 s on); now LC_UNSURE_MAX_S after the last
  # frame read broken white, the request withdrawn LC_ABORT_SETTLE_S first, nothing ever requested
  drive, m, distance = offered_left()
  drive.paint = lambda left, right: paint(left=UNKNOWN, right=SW)
  last_read = drive.t
  drive.run(m, distance, 20.)
  assert [a for a, _ in drive.aborts] == ['lineNotBroken'] and not drive.requests
  assert drive.aborts[0][1] - last_read == pytest.approx(LC_UNSURE_MAX_S + LC_ABORT_SETTLE_S, abs=2.5 * DT_MDL)


def test_an_unsure_frame_withdraws_a_request_out():
  """In REQUEST (DesireHelper holding the change in preLaneChange): a frame read short of broken white withdraws the request
  for that frame, the offer up; the next broken white frame asks again. DesireHelper starts a change only on a request."""
  lp = LanePlanner()
  m = maneuver("turn", "left")
  args = {'lanes': model_lanes(*road(1, 0)), 'v_ego': 15., 'left_blinker': False, 'right_blinker': False, 'objects': (),
          'driver_signals': False, 'right_hand_traffic': True, 'dt': DT_MDL}
  distance, d = 290., None
  for _ in range(int((LC_CONFIRM_FRAMES * DT_MDL + LC_WARN_S + 0.2) / DT_MDL)):
    d = lp.update(m, distance, lc_state=LaneChangeState.preLaneChange, paint=paint(left=BW), **args)
    distance -= 15. * DT_MDL
  assert lp.phase == Phase.REQUEST and d.lane_change == LaneChange.left
  d = lp.update(m, distance, lc_state=LaneChangeState.preLaneChange, paint=paint(left=UNKNOWN), **args)
  assert d.lane_change == LaneChange.none and d.offer == LaneChange.left and d.abort == 'none' and d.reason == LINE_UNSURE_REASON
  d = lp.update(m, distance, lc_state=LaneChangeState.off, paint=paint(left=BW), **args)
  assert d.lane_change == LaneChange.left


def night_two_lanes(edge_m=5.4, edge_std=0.3, far_prob=0.05):
  """The 1e seg 8-9 approach: a two-lane one-way, the car in the right lane, the far-left line unseen (prob far_prob), the
  left road edge edge_m beyond the left ego line (median 5.4-5.5 m)."""
  line_ys, probs, edges, stds = road(1, 0)
  probs[0] = far_prob
  edges[0] = line_ys[1] - edge_m
  stds[0] = edge_std
  return model_lanes(line_ys, probs, edges, stds)


def test_a_lane_beyond_broken_white_counts_by_the_room_to_a_trusted_edge():
  """lanes_toward: the next line out unseen, the ego line read broken white (white hold included) and a trusted road edge a
  lane's width or more beyond it is one lane; nothing less, never across a yellow, never for keeps."""
  night = night_two_lanes()
  assert night.left == 0 and night.left_open
  assert change_blocker(LEFT, night, paint(left=BW)) == '' and lanes_toward(LEFT, night, paint(left=BW)) == 1
  hold = WhiteHold()
  hold.update(paint(left=BW), 10., DT_MDL, False)
  assert change_blocker(LEFT, night, hold.update(paint(left=COLORLESS), 10., DT_MDL, False)) == '', "the white hold's broken white"
  for p in NOT_BW:  # no lane by room alone, on anything short of broken white
    assert lanes_toward(LEFT, night, paint(left=p)) == 0 and change_blocker(LEFT, night, paint(left=p)) != '', p
  # never across yellow: the ego line read or held yellow; the line beyond read or held yellow
  assert change_blocker(LEFT, night, paint(left=SY)) == 'lineNotBroken' and lanes_toward(LEFT, night, paint(left=LATCHED_YELLOW)) == 0
  yellow = YellowHold()
  yellow.update(paint(left=SY), 10., DT_MDL, False)
  held = yellow.update(paint(left=BW), 10., DT_MDL, False)  # read broken white under the hold
  assert held[1] == LATCHED_YELLOW and change_blocker(LEFT, night, held) == 'lineNotBroken'
  # the line beyond read or held yellow (a one-way road's yellow left edge line, 1e seg 8 534.7 s): the one lane before it, as
  # paint_lanes counts, never one past it
  for beyond in (SY, BY, LATCHED_YELLOW):
    assert lanes_toward(LEFT, night, paint(left=BW, far_left=beyond)) == 1, beyond
    assert lanes_toward(LEFT, model_lanes(*road(2, 0)), paint(left=BW, far_left=beyond)) == 1, beyond
  assert lanes_toward(LEFT, model_lanes(*road(2, 0)), paint(left=BW)) == 2, "the model's own count as before"
  # room for less than a lane (a parking strip, a shoulder), an unsure edge, a line seen out there nearer than a lane: noLane
  for lanes in (night_two_lanes(edge_m=0.8 * LANE_WIDTH), night_two_lanes(edge_std=2.), night_two_lanes(far_prob=0.9, edge_m=7.)):
    assert not lanes.left_open
  line_ys, probs, edges, stds = road(1, 0, shoulder=3.)
  line_ys[0] = line_ys[1] - 0.7 * LANE_WIDTH  # a strip's line, seen
  strip = model_lanes(line_ys, probs, edges, stds)
  assert strip.left == 0 and not strip.left_open and change_blocker(LEFT, strip, paint(left=BW)) == 'noLane'
  assert change_blocker(LEFT, night_two_lanes(edge_m=0.8 * LANE_WIDTH), paint(left=BW)) == 'noLane'
  assert change_blocker(LEFT, night_two_lanes(edge_std=2.), paint(left=BW)) == 'noLane'
  # the curb side the same; parked cars ahead still make it a parking strip; left-hand traffic never away from the curb
  line_ys, probs, edges, stds = road(0, 1)
  probs[3] = 0.05
  curb = model_lanes(line_ys, probs, edges, stds)
  assert curb.right == 0 and change_blocker(RIGHT, curb, paint(right=BW)) == ''
  parked = scene([{'cls': 'car', 'x': 30., 'y': -3.6, 'vx': 0.}]).objects
  assert change_blocker(RIGHT, curb, paint(right=BW), parked) == 'laneParked'
  assert change_blocker(RIGHT, curb, paint(right=BW), right_hand_traffic=False) == 'noLane'
  # the map's lane list none that way still says no lane
  assert change_blocker(LEFT, night, paint(left=BW), nav_room=0) == 'noLane'
  # keeps aren't touched: a far-side keep still wants the next line out painted (keep_blocker, BeamNG S19)
  assert keep_blocker(night, LEFT, paint=paint(left=BW)) == 'keepFarSide'


def test_at_night_a_turn_changes_lanes_over_broken_white_with_no_line_seen_beyond():
  """The planner end to end with the far lines unseen: before, noLane on every frame and no offer all night."""
  drive = Drive(lanes_left=1, lanes_right=0, night=True)
  drive.run(maneuver("turn", "left"), 400., 30.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.left] and (drive.lanes_left, drive.lanes_right) == (0, 1)
  assert not drive.aborts
  drive = Drive(lanes_left=0, lanes_right=1, night=True)
  drive.run(maneuver("turn", "right"), 400., 30.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.right] and (drive.lanes_left, drive.lanes_right) == (1, 0)
  # an undivided road: the centre line yellow, its far edge beyond the oncoming lane; never across it
  drive = Drive(lanes_left=1, lanes_right=0, night=True, paint=lambda left, right: paint(left=SY, right=SW))
  drive.run(maneuver("turn", "left"), 400., 30.)
  assert not drive.offers and not drive.requests
  # the centre line read broken white on a frame under its hold (a misread): still held yellow, still nothing
  reads = iter([SY] * 10 + [BW] * 2000)
  drive = Drive(lanes_left=1, lanes_right=0, night=True, paint=lambda left, right: paint(left=next(reads), right=SW))
  drive.run(maneuver("turn", "left"), 400., YELLOW_HOLD_S - 0.6)
  assert not drive.offers


LEFT_BAY = ((("left",), True), (("straight",), False), (("straight",), False))  # drive 1e seg 5's lane list


def latch_reads(latch, p, n=BAY_YELLOW_READS, i=1, bay=True, counted=False, frozen=False, dt=DT_MDL, line=None):
  """n frames of paint p on the far-side ego line (index i, the left), unknown elsewhere; line: a whole paint tuple instead."""
  for _ in range(n):
    latch.update(bay, line if line is not None else tuple(p if k == i else UNKNOWN for k in range(4)), i, counted, dt, frozen)
  return latch.shut


def test_bay_latch():
  """BayLatch (2026-11-06 review): shut on BAY_YELLOW_READS reads of the far-side ego line yellow at BAY_YELLOW_CONF_MIN, none
  of them within BAY_CONTEST_S after a white read of that line; open on any white read of it (any kind but none), a lane the
  model counts that way, no far_bay, or a lane change (shift)."""
  assert latch_reads(BayLatch(), SY) and latch_reads(BayLatch(), BY), "BY reads at the bar, 0.5"
  assert not latch_reads(BayLatch(), SY, BAY_YELLOW_READS - 1), "one read short"
  for weak in (LinePaint('unknown', 'yellow', 0.35, 0.3), LinePaint('broken', 'yellow', BAY_YELLOW_CONF_MIN - 0.01, 0.3)):
    assert not latch_reads(BayLatch(), weak, 100), "under the bar: the yellow hold's YELLOW_CONF_MIN vetoes, it doesn't latch"
  assert not latch_reads(BayLatch(), LinePaint('none', 'yellow', 1., 0.), 100)
  # any white read of the line opens it, and its reads count afresh
  for white in (BW, SW, LinePaint('unknown', 'white', 0.6, 0.3)):
    latch = BayLatch()
    assert latch_reads(latch, SY) and not latch_reads(latch, white, 1) and latch.reads == 0, white
  latch = BayLatch()
  assert latch_reads(latch, SY) and latch_reads(latch, LinePaint('broken', 'white', 0.55, 0.3), 1), "white under the bar"
  assert latch_reads(latch, COLORLESS, 1000) and latch_reads(latch, UNKNOWN, 1000), "latched past the yellow hold"
  assert latch_reads(latch, None, 1, line=paint(left=SY, right=SW, far_left=BW)), "another line's white isn't this one's"
  # contested: a yellow read within BAY_CONTEST_S after a white read of the line doesn't count
  latch = BayLatch()
  latch_reads(latch, BW, 1)
  assert not latch_reads(latch, SY, int(round(BAY_CONTEST_S / DT_MDL)) - 2)
  assert latch.reads == 0 and latch_reads(latch, SY, BAY_YELLOW_READS + 2)
  # the model counting a lane that way, no far_bay, a frame frozen in a lane change
  assert not latch_reads(BayLatch(), SY, counted=True) and not latch_reads(BayLatch(), SY, bay=False)
  assert not latch_reads(BayLatch(), SY, frozen=True)
  latch = BayLatch()
  assert latch_reads(latch, SY) and not latch_reads(latch, UNKNOWN, 1, counted=True) and latch.reads == 0
  # a lane change opens it; the white reads move with the lines, and with the direction unknown the latest covers all four
  latch = BayLatch()
  latch_reads(latch, SY)
  latch.shift(LEFT)
  assert not latch.shut and latch.reads == 0 and latch_reads(latch, SY), "open, its reads counted afresh in the new lane"
  latch = BayLatch()
  latch_reads(latch, None, 1, line=paint(far_left=BW))  # the far-left line read white, the car changes left: its ego left now
  latch.shift(LEFT)
  assert not latch_reads(latch, SY, BAY_YELLOW_READS + 5)
  latch = BayLatch()
  latch_reads(latch, None, 1, line=paint(far_right=BW))  # the far-right line white, a crossing of unknown direction: any line
  latch.shift(0)
  assert not latch_reads(latch, SY, BAY_YELLOW_READS + 5)
  latch = BayLatch()
  latch_reads(latch, None, 1, line=paint(far_right=BW))
  latch.shift(RIGHT)  # the car changed right: the far-right line is its ego right line now, the left one uncontested
  assert latch_reads(latch, SY)


def test_a_far_side_turn_bay_not_open_yet_wants_no_change():
  """desired_changes' bay rule: a far-side turn whose valid lanes are the list's far-most, the far-side ego line read or held
  yellow: none (the car beside the centre line is in the far-most lane of its direction; the turn desire takes the bay). 1e
  seg 5: [left (active), straight, straight] on a one-lane approach wanted +2 against the yellow the whole approach."""
  m = maneuver("turn", "left", LEFT_BAY)
  one_lane = model_lanes(*road(0, 0))
  assert far_bay(m) and desired_changes(m, one_lane) == 2  # counted from the curb: the list's curb lane
  for yellow in (SY, BY, LATCHED_YELLOW):
    assert desired_changes(m, one_lane, paint=paint(left=yellow)) == 0, yellow
  for other in (BW, UNKNOWN, SW):
    assert desired_changes(m, one_lane, paint=paint(left=other)) == 2, other
  assert desired_changes(m, one_lane, bay_shut=True) == 0 and desired_changes(m, one_lane, paint=paint(left=SY), bay_shut=False) == 2
  # in the far lane of a two-lane approach with the bay's list: none wanted either way
  assert desired_changes(maneuver("turn", "left", THREE_LANES_LEFT_TURN), model_lanes(*road(0, 1)), paint=paint(left=SY)) == 0
  # not a bay: the lanes it leaves from not the far-most, a curb-side turn, no list; nor away from the far side
  assert not far_bay(maneuver("turn", "left", ((("straight",), False), (("left",), True), (("straight",), False))))
  assert not far_bay(maneuver("turn", "left", ((("left",), True), (("left",), True))))  # every lane the turn's: nothing to want
  assert not far_bay(maneuver("turn", "right", TWO_LANES_RIGHT_TURN)) and not far_bay(maneuver("turn", "left"))
  assert desired_changes(maneuver("turn", "left"), one_lane, paint=paint(left=SY)) == 1, "no list: as before"
  assert desired_changes(maneuver("turn", "right", TWO_LANES_RIGHT_TURN), model_lanes(*road(0, 1)), paint=paint(left=SY)) == -1
  # left-hand traffic: the far side is the right
  rht_bay = maneuver("turn", "right", ((("straight",), False), (("straight",), False), (("right",), True)))
  assert far_bay(rht_bay, right_hand_traffic=False) and not far_bay(rht_bay)
  assert desired_changes(rht_bay, one_lane, False, paint=paint(right=SY)) == 0 and desired_changes(rht_bay, one_lane, False) == -2


def test_the_bay_shut_lasts_until_broken_white_and_is_never_blocked():
  """Through the planner: shut from the yellow, latched past the yellow hold through unknown reads to the window's end, so
  LC-03 never skips the turn "Can't Reach Turn Lane" for a bay that hasn't opened; broken white read that way (a lane of
  ours opened) and the normal rule is back."""
  m = maneuver("turn", "left", LEFT_BAY)
  drive = Drive(0, 0, night=True, paint=lambda left, right: paint(left=SY, right=SW))
  start = lane_change_window(drive.v, 2)[0] + 30.
  frames = through_the_window(drive, m, start, stop=start - 30.)
  drive.paint = lambda left, right: paint(left=UNKNOWN, right=SW)  # the centre line unread for the rest of the window
  frames += through_the_window(drive, m, start - 30.)
  assert drive.lp.yellow.apply(paint()) == paint(), "the yellow hold ran out long before the window's end"
  assert all(x.want == 2 for _, x in frames[:BAY_YELLOW_READS - 1]), "shut on its BAY_YELLOW_READS-th read (BayLatch)"
  assert all(x.want == 0 for _, x in frames[BAY_YELLOW_READS - 1:]) and not any(x.blocked for _, x in frames) and not drive.offers
  assert desired_changes(m, model_lanes(*road(0, 0)), paint=paint(left=UNKNOWN, right=SW)) == 2, "without the latch: +2 again"
  drive.paint = lambda left, right: paint(left=BW, right=SW)
  assert drive.step(m, 15.).want == 2
  # the same approach with no yellow ever read: +2 throughout, and LC-03 blocks it at the window's end, placed by the list
  control = Drive(0, 0, night=True, paint=lambda left, right: paint(left=UNKNOWN, right=SW))
  frames = through_the_window(control, m, start)
  assert all(x.want == 2 for _, x in frames) and any(x.blocked for _, x in frames)
  # a hold that is only shift(0)'s cover shuts nothing
  drive = Drive(0, 0, night=True, paint=lambda left, right: paint(left=UNKNOWN, right=SW))
  drive.lp.yellow.update(paint(far_right=SY), 15., DT_MDL, False)
  drive.lp.yellow.shift(0)
  assert drive.step(m, 200.).want == 2 and not drive.lp.bay.shut


def test_the_bay_shut_places_the_car():
  """lane_unknown ('Can't Confirm Lane') isn't said with the bay shut: beside the centre line the car is in the far-most lane
  of its direction, wherever the model's curb-side count is unsure."""
  m = maneuver("turn", "left", LEFT_BAY)
  lanes = model_lanes(*road(0, 0, edge_std=2.))  # the curb edge unsure: the list can't place the car
  for p, unknown in ((paint(left=BW), True), (paint(left=SY), False)):
    lp = LanePlanner()
    for k in range(20):
      lp.update(m, 250. - k * 0.75, lanes, LaneChangeState.off, 15., False, False, (), False, True, DT_MDL, p)
    assert lp.lane_unknown == unknown, p


def test_a_lane_counted_by_room_places_the_car_for_lc03_only_on_an_offers_streak():
  """LC-03 without a lane list places the car by a travel lane read that way on a frame of the window: by the model's painted
  count on one frame, as ever; by the room beyond broken white (lanes_toward) only on LC_CONFIRM_FRAMES frames in a row, the
  bar an offer needs. A single broken white read with no line seen beyond it is no evidence to skip a turn on."""
  m = maneuver("turn", "right")
  start = lane_change_window(15.)[0] + 30.
  _, end = lane_change_window(15.)

  def once(n, night):
    """The right ego line read broken white on n frames in a row mid-window, unknown on the rest; the blind spot occupied
    throughout, so no change is ever offered."""
    k = [0]
    def pnt(left, right):
      k[0] += 1
      return paint(left=SW, right=BW if 200 <= k[0] < 200 + n else UNKNOWN)
    drive = Drive(0, 1, night=night, paint=pnt)
    return through_the_window(drive, m, start, blindspot=LaneChange.right)
  for n, night, blocked in ((1, True, False), (LC_CONFIRM_FRAMES - 1, True, False), (LC_CONFIRM_FRAMES, True, True),
                            (1, False, True)):
    frames = once(n, night)
    assert any(x.blocked for d, x in frames if d <= end) == blocked, (n, night)


# The 2026-11-06 review: the bay latch from one misread yellow sent a left from the curb lane across two through lanes, and
# a frame broken white by the white hold alone was the frame a request went out on.

def bay_drive(reads, lanes_left=2, start_extra=30., blindspot=None):
  """A night approach with LEFT_BAY's list, the car in the curb lane with lanes_left lanes of ours to its left (the far lines
  unseen), the left ego line read reads[k] on frame k (UNKNOWN past their end; the centre line, yellow, once the car has
  changed into the far lane). [(distance, decision)], and per frame the latch and the lanes left of the car."""
  m = maneuver("turn", "left", LEFT_BAY)
  drive = Drive(lanes_left, 0, night=True)
  k = [0]
  def pnt(left, right):
    k[0] += 1
    return paint(left=SY if lanes_left and not left else reads[k[0] - 1] if k[0] <= len(reads) else UNKNOWN, right=SW)
  drive.paint = pnt
  distance = lane_change_window(drive.v, lanes_left or 2)[0] + start_extra
  frames, shut = [], []
  while distance > 20.:
    frames.append((distance, drive.step(m, distance, blindspot=blindspot)))
    shut.append((drive.lp.bay.shut, drive.lanes_left))
    distance -= drive.v * DT_MDL
  return drive, frames, shut


def test_a_lane_divider_misread_yellow_once_never_shuts_the_bay():
  """The review's probe: a three-lane approach at night, the car in the curb lane, the left lane divider read yellow on one
  frame and unknown on every other. Before, the latch shut on that one read at YELLOW_CONF_MIN (0.35 here): want 0 to the
  window's end, no 'Can't Confirm Lane', never blocked, the left pulsed from the curb lane across two through lanes. Now
  under BAY_YELLOW_CONF_MIN it counts for nothing, and one read at 0.9 is short of BAY_YELLOW_READS: +2 throughout and LC-03
  skips the turn at the window's end, as before the bay rule."""
  for misread in ([LinePaint('unknown', 'yellow', 0.35, 0.3)], [SY], [SY] * (BAY_YELLOW_READS - 1)):
    drive, frames, shut = bay_drive([UNKNOWN] * 19 + misread)
    assert not any(s for s, _ in shut) and all(x.want == 2 for _, x in frames), misread
    assert_blocked_at_the_end(drive, frames)


def test_a_yellow_misread_amid_white_reads_of_the_line_never_shuts_the_bay():
  """Drive 1e seg 8-9 (a two-lane one-way): the left lane divider read white on 621 of 1041 frames of 518-570 s and broken
  yellow 0.55-0.75 on 28 over 536.0-537.3 s, white again within 5 s either side. Replayed with the bay's list the old latch shut on
  it for 10.5 s, wanting nothing. Contested by the white reads before it (BAY_CONTEST_S) it never shuts, and both changes
  go once the yellow hold has run out; the yellow vetoes them until then, as it always did."""
  misread = [LinePaint('broken', 'yellow', 0.55 + 0.2 * (k % 2), 0.3) for k in range(28)]
  drive, frames, shut = bay_drive([BW] * 40 + misread + [BW] * 2000, start_extra=150.)
  assert not any(s for s, left in shut if left) and drive.lp.changes == 2 and (drive.lanes_left, drive.lanes_right) == (0, 2)
  assert not any(x.blocked for _, x in frames)
  # a white read of the line opens a shut bay at once, even under the yellow hold (which writes it yellow for every other rule),
  # and its yellow after that is contested: the changes wanted back
  m = maneuver("turn", "left", LEFT_BAY)
  for white in (BW, LinePaint('unknown', 'white', 0.7, 0.3), LinePaint('solid', 'white', 0.8, 0.9)):
    lp = LanePlanner()
    args = (model_lanes(*road(0, 0)), LaneChangeState.off, 15., False, False, (), False, True, DT_MDL)
    for _ in range(BAY_YELLOW_READS):
      lp.update(m, 400., *args, paint(left=SY, right=SW))
    assert lp.bay.shut and lp.update(m, 400., *args, paint(left=SY, right=SW)).want == 0
    d = lp.update(m, 400., *args, paint(left=white, right=SW))
    assert not lp.bay.shut and d.want == 2 and lp.yellow.apply(paint())[1] == LATCHED_YELLOW, white
    for _ in range(int(BAY_CONTEST_S / DT_MDL) - 2):
      assert lp.update(m, 400., *args, paint(left=SY, right=SW)).want == 2, white


def test_a_lane_the_model_counts_beyond_the_line_opens_the_bay():
  """The review's rule (c), on the yellow hold's paint: a painted lane the model counts beyond the far-side ego line, the count
  known, is a lane of ours there once nothing yellow takes it out. Under the hold it counts nothing: on an undivided road the
  model counts the oncoming lane beyond the centre line (1e seg 5: 106 frames of 270-327 s, every one under the hold)."""
  m = maneuver("turn", "left", LEFT_BAY)
  one, beyond = model_lanes(*road(0, 0)), model_lanes(*road(1, 0))
  assert beyond.left == 1 and beyond.left_known
  lp = LanePlanner()
  args = (LaneChangeState.off, 15., False, False, (), False, True, DT_MDL)
  for _ in range(BAY_YELLOW_READS):
    lp.update(m, 400., one, *args, paint(left=SY, right=SW))
  assert lp.bay.shut
  hold = int(round(max(YELLOW_HOLD_S, YELLOW_HOLD_M / 15.) / DT_MDL))
  for _ in range(hold - 2):
    assert lp.update(m, 400., beyond, *args, paint(left=UNKNOWN, right=SW)).want == 0
  assert lp.bay.shut
  for _ in range(4):  # the hold run out, a frame nobody read: the raw count is no lane of ours (the oncoming lane)
    lp.update(m, 400., beyond, *args, ())
  assert lp.bay.shut and lp.yellow.apply(()) == ()
  # the hold run out, the paint read: the line is still the one read yellow (marked), so the lane counted beyond it is the
  # oncoming one and the latch stays shut; white reads of the line (a bay's own line, where the centre line was) open it
  lp.update(m, 400., beyond, *args, paint(left=UNKNOWN, right=SW))
  assert lp.bay.shut
  lp.update(m, 400., beyond, *args, paint(left=BW, right=SW))
  assert not lp.bay.shut
  # a line never read yellow or solid: the counted lane opens the latch, as before
  lp = LanePlanner()
  lp.bay.shut = True
  lp.update(m, 400., beyond, *args, paint(left=UNKNOWN, right=SW))
  assert not lp.bay.shut


def test_a_centre_line_read_solid_with_no_colour_keeps_the_bay_shut():
  """Route 29 (2026-11-08, 216-243 s), a left with [left, straight] on a one-lane approach at night: the centre line read
  yellow 330 m out (the latch shut), then solid of no colour from 305 m to 88 m, the model counting the oncoming lane beyond
  it. With the yellow hold run out that count opened the latch 156 m out, the turn wanted a change across the solid line for
  the rest of the approach, and LC-03 skipped the left at the window's end. A line marked yellow or solid is never counted
  over: no change wanted, nothing skipped."""
  m = maneuver("turn", "left", LEFT_BAY)
  one, beyond = model_lanes(*road(0, 0)), model_lanes(*road(1, 0))
  solid = LinePaint('solid', 'unknown', 1., 0.95)
  lp = LanePlanner()
  args = (LaneChangeState.off, 13., False, False, (), True, True, DT_MDL)
  distance = 330.
  for _ in range(BAY_YELLOW_READS):
    lp.update(m, distance, one, *args, paint(left=SY, right=UNKNOWN))
    distance -= 13. * DT_MDL
  assert lp.bay.shut
  decisions = []
  while distance > 20.:
    decisions.append(lp.update(m, distance, beyond, *args, paint(left=solid, right=UNKNOWN)))
    distance -= 13. * DT_MDL
  assert lp.bay.shut and all(d.want == 0 for d in decisions) and not any(d.blocked for d in decisions)


def test_the_real_bays_yellow_still_shuts_it():
  """1e seg 5's centre line: yellow on a third of frames (329 of 332 at 0.5 or better), unknown on the rest, white on none:
  shut on its reads and latched through the unknown ones to the window's end, never blocked."""
  reads = [SY if k % 3 == 0 else UNKNOWN for k in range(400)]
  drive, frames, shut = bay_drive(reads, lanes_left=0)
  assert all(s for s, _ in shut[3 * BAY_YELLOW_READS:])
  assert all(x.want == 0 for _, x in frames[3 * BAY_YELLOW_READS:]) and not any(x.blocked for _, x in frames)


def test_nothing_is_requested_on_a_frame_broken_white_by_the_white_hold_alone():
  """Drive 1d's left ego line: broken white 0.70-0.87 over 315.6-321.4 s, broken of unknown colour 321.52-322.41 s (the
  white hold made it broken white, room to the edge with no line seen beyond), broken yellow from 322.67 s. The countdown
  ending inside the colourless stretch: before, the request went out on its first frame, and nothing cancels a change once
  DesireHelper starts it. Now the offer waits through it (LINE_UNSURE_REASON), and the yellow aborts it."""
  drive, m, distance = offered_left(night=True)
  distance = drive.run(m, distance, drive.lp.countdown - 0.3)
  assert drive.lp.phase == Phase.WARN
  drive.paint = lambda left, right: paint(left=COLORLESS, right=SW)
  distance = drive.run(m, distance, 0.9)
  assert drive.lp.phase == Phase.REQUEST and not drive.requests and drive.decisions[-1].reason == LINE_UNSURE_REASON
  assert drive.lp.white.apply(paint(left=COLORLESS, right=SW))[1].color == 'white', "the hold covered the stretch"
  drive.paint = lambda left, right: paint(left=LinePaint('broken', 'yellow', 0.5, 0.3), right=SW)
  drive.run(m, distance, 1.)
  assert not drive.requests and [a for a, _ in drive.aborts] == ['lineNotBroken'] and drive.lp.changes == 0
  # by day (the next line out painted, so the model's count, not the room) the same
  drive, m, distance = offered_left()
  distance = drive.run(m, distance, drive.lp.countdown - 0.3)
  drive.paint = lambda left, right: paint(left=COLORLESS, right=SW)
  drive.run(m, distance, 0.9)
  assert drive.lp.phase == Phase.REQUEST and not drive.requests
  # a request already out is withdrawn on such a frame, the offer up, and asked again on the next frame read broken white
  lp = LanePlanner()
  args = {'lanes': model_lanes(*road(1, 0)), 'v_ego': 15., 'left_blinker': False, 'right_blinker': False, 'objects': (),
          'driver_signals': False, 'right_hand_traffic': True, 'dt': DT_MDL}
  distance, d = 290., None
  for _ in range(int((LC_CONFIRM_FRAMES * DT_MDL + LC_WARN_S + 0.2) / DT_MDL)):
    d = lp.update(maneuver("turn", "left"), distance, lc_state=LaneChangeState.preLaneChange, paint=paint(left=BW), **args)
    distance -= 15. * DT_MDL
  assert d.lane_change == LaneChange.left
  d = lp.update(maneuver("turn", "left"), distance, lc_state=LaneChangeState.preLaneChange, paint=paint(left=COLORLESS), **args)
  assert d.lane_change == LaneChange.none and d.offer == LaneChange.left and d.reason == LINE_UNSURE_REASON
  d = lp.update(maneuver("turn", "left"), distance, lc_state=LaneChangeState.off, paint=paint(left=BW), **args)
  assert d.lane_change == LaneChange.left


def test_a_white_read_every_few_seconds_never_makes_a_change():
  """The review's probe: the line read broken white once every 2.5 s, broken of unknown colour between (mixed votes), never
  yellow. Before, the white hold made every frame broken white: offered and changed. Now only the reads count toward the
  offer (the hold keeps the count between them): ten take 22.5 s, past the window, so nothing is offered, by day or night."""
  for night in (False, True):
    def pnt(left, right, k=[0]):  # noqa: B006 (its frame count, per drive)
      k[0] += 1
      return paint(left=BW if k[0] % 50 == 1 else COLORLESS, right=SW)
    drive = Drive(1, 0, paint=pnt, night=night)
    drive.run(maneuver("turn", "left"), 400., 25.)
    assert not drive.offers and not drive.requests, night


def test_the_white_hold_keeps_the_confirm_count_and_the_offer_goes_on_a_read():
  """A frame valid by the white hold alone neither counts toward LC_CONFIRM_FRAMES nor breaks it, and is never the frame an
  offer is made on, even with the count made and a blind spot clearing on it. A frame read short of broken white the hold
  doesn't cover (the kind unknown) keeps it too since 2026-11-07 (soft_block: the want of a read, nothing against the
  lane); a contrary read (solid) still starts the count over."""
  def offer_frame(reads, blindspots=()):
    lp = LanePlanner()
    it = iter(reads)
    for k in range(len(reads)):
      d = lp.update(maneuver("turn", "left"), 250. - k * 0.75, night_two_lanes(), LaneChangeState.off, 15., False, False, (), False,
                    True, DT_MDL, paint(left=next(it), right=SW), left_blindspot=k in blindspots)
      if d.offer == LaneChange.left:
        return k
    return None
  assert offer_frame([BW] * 20) == LC_CONFIRM_FRAMES - 1
  assert offer_frame([BW] * 5 + [COLORLESS] * 3 + [BW] * 20) == LC_CONFIRM_FRAMES + 2
  assert offer_frame([BW] * 5 + [LinePaint('unknown', 'white', 0.9, 0.3)] + [BW] * 20) == LC_CONFIRM_FRAMES
  assert offer_frame([BW] * 5 + [SW] + [BW] * 20) == 5 + LC_CONFIRM_FRAMES
  assert offer_frame([BW] * 15 + [COLORLESS] * 3 + [BW] * 5, blindspots=range(15)) == 18


def test_a_lane_by_the_white_hold_alone_never_places_the_car_for_lc03():
  """LC-03 without a lane list, at night: one frame read broken white, the white hold's frames after it. Before, they counted
  toward the room streak (LC_CONFIRM_FRAMES of them placed the car, and the turn was skipped); now they keep it, uncounted."""
  m = maneuver("turn", "right")
  start = lane_change_window(15.)[0] + 30.
  _, end = lane_change_window(15.)
  k = [0]
  def pnt(left, right):
    k[0] += 1
    return paint(left=SW, right=BW if k[0] == 200 else COLORLESS if 200 < k[0] < 260 else UNKNOWN)
  drive = Drive(0, 1, night=True, paint=pnt)
  frames = through_the_window(drive, m, start, blindspot=LaneChange.right)
  assert not any(x.blocked for d, x in frames if d <= end)


# The 2026-11-06 review's low findings on the bay rule: 'Wrong Lane' said on the approach the bay rule drives the turn from,
# and a lane list that marks no lane for the turn taken for a bay on _valid_lanes' guess.

def test_a_shut_bay_is_no_wrong_lane_for_its_turn():
  """lane_mismatch said wrongLaneForTurn ('Wrong Lane: This lane doesn't turn with the route') on a shut-bay approach that
  TurnPilot drives the turn from: [left (active), straight, straight], one lane, 40 m out at 10 m/s (inside the window's end,
  45 m), the left line yellow, the plan wanting no change. Now the same judgement as desired_changes' bay rule (_bay_waits):
  nothing said while it holds, and as before once it doesn't."""
  m = maneuver("turn", "left", LEFT_BAY)
  one_lane = model_lanes(*road(0, 0))
  assert lane_change_window(10.)[1] > 40.
  assert lane_mismatch(m, 40., (), 0., one_lane, 10.) == LaneMismatch.wrongLaneForTurn, "nothing said of the bay: as before"
  for yellow in (SY, BY, LATCHED_YELLOW):
    assert desired_changes(m, one_lane, paint=paint(left=yellow)) == 0, yellow
    assert lane_mismatch(m, 40., (), 0., one_lane, 10., paint=paint(left=yellow)) == LaneMismatch.none, yellow
  for other in (BW, UNKNOWN, SW, LinePaint('unknown', 'yellow', 0.35, 0.3)):
    assert desired_changes(m, one_lane, paint=paint(left=other)) == 2, other
    assert lane_mismatch(m, 40., (), 0., one_lane, 10., paint=paint(left=other)) == LaneMismatch.wrongLaneForTurn, other
  # the planner's latch, as turnpilotd hands it over, over the frame's own paint
  assert lane_mismatch(m, 40., (), 0., one_lane, 10., bay_shut=True) == LaneMismatch.none
  assert lane_mismatch(m, 40., (), 0., one_lane, 10., paint=paint(left=SY), bay_shut=False) == LaneMismatch.wrongLaneForTurn
  # only for the bay's turn: another list's wrong lane is said whatever the latch, and so is a turn-only lane going straight
  mid = maneuver("turn", "left", ((("straight",), False), (("left",), True), (("straight",), False)))
  assert lane_mismatch(mid, 40., (), 0., one_lane, 10., paint=paint(left=SY), bay_shut=True) == LaneMismatch.wrongLaneForTurn
  assert lane_mismatch(None, 500., LEFT_ONLY_AND_THROUGH, 40., model_lanes(*road(0, 2)), 10., paint=paint(left=SY), bay_shut=True) == \
         LaneMismatch.turnOnlyGoingStraight
  # left-hand traffic: the far side is the right
  rht_bay = maneuver("turn", "right", ((("straight",), False), (("straight",), False), (("right",), True)))
  assert lane_mismatch(rht_bay, 40., (), 0., one_lane, 10., False, paint=paint(right=SY)) == LaneMismatch.none
  assert lane_mismatch(rht_bay, 40., (), 0., one_lane, 10., False) == LaneMismatch.wrongLaneForTurn
  # through the planner, as turnpilotd asks every frame: the centre line yellow on a frame in three and unknown on the rest
  # (1e seg 5), then a broken white read opening the bay. Inside the window's end the banner and the plan agree on every
  # frame: wrong lane exactly when a change is wanted
  lp = LanePlanner()
  distance, k, said = 200., 0, []
  while distance > 20.:
    left = BW if distance < 35. else SY if k % 3 == 0 else UNKNOWN
    d = lp.update(m, distance, one_lane, LaneChangeState.off, 10., False, False, (), False, True, DT_MDL, paint(left=left, right=SW))
    found = lane_mismatch(m, distance, (), 0., one_lane, 10., bay_shut=lp.bay.shut)
    if distance < lane_change_window(10.)[1]:
      said.append((distance, d.want, found))
    assert not d.blocked, distance
    distance, k = distance - 10. * DT_MDL, k + 1
  assert all((found == LaneMismatch.wrongLaneForTurn) == (want != 0) for _, want, found in said)
  assert {found for dd, _, found in said if dd >= 35.} == {LaneMismatch.none}, "shut to the white read: no 'Wrong Lane'"
  assert {found for dd, _, found in said if dd < 35.} == {LaneMismatch.wrongLaneForTurn}, "the bay open: the list's lane again"


def test_a_list_that_marks_no_lane_for_the_turn_is_no_bay():
  """far_bay went by _valid_lanes, whose guess for a list that marks no lane for the turn (none active, none marked for its
  side: an incomplete OSM turn:lanes) is the outer lane on the turn's side, so any far-side turn with such a list counted as
  a bay, and the bay rule (the permissive answer for LC-03) could zero its changes on that guess. Now only the list's own
  marks: an unmarked three-lane list goes by the normal rule, its latch never shut."""
  one_lane = model_lanes(*road(0, 0))
  for unmarked in (((), False),) * 3, ((("none",), False),) * 3, ((("straight",), False),) * 3, \
                   ((("straight",), False), (("straight",), False), (("straight", "right"), False)):
    m = maneuver("turn", "left", unmarked)
    assert not far_bay(m), unmarked
    assert desired_changes(m, one_lane, paint=paint(left=SY)) == 2 and desired_changes(m, one_lane, bay_shut=True) == 2, unmarked
    lp = LanePlanner()
    for _ in range(3 * BAY_YELLOW_READS):
      d = lp.update(m, 400., one_lane, LaneChangeState.off, 15., False, False, (), False, True, DT_MDL, paint(left=SY, right=SW))
    assert not lp.bay.shut and d.want == 2, unmarked
  # the list's own marks, active or for the turn's side, are a bay as before
  for marked in (LEFT_BAY, ((("left",), False), (("straight",), False), (("straight",), False)),
                 ((("none",), True), (("straight",), False), (("straight",), False))):
    assert far_bay(maneuver("turn", "left", marked)), marked
  # left-hand traffic: the far side is the right
  assert not far_bay(maneuver("turn", "right", ((("none",), False),) * 3), right_hand_traffic=False)
  assert far_bay(maneuver("turn", "right", ((("straight",), False), (("straight",), False), (("right",), False))), right_hand_traffic=False)


def test_an_unsure_wait_is_bounded_and_the_lamp_goes_out():
  """The review's third probe (2026-11-06), left open last round: the car working its own lamp with the legal 300 ft lead, an
  offer confirmed on broken white, then the line read unknown to the window's end. The lamp stayed lit 14.2 s with no change
  and the offer ended tooLate; now it aborts lineNotBroken LC_UNSURE_MAX_S after the last frame read broken white, said,
  and the lamp goes out. A dash's colour dropping out for 1.95 s (the longest such stretch an ego line came back from broken
  white on 1d and 1e: 1e 525.5-527.4 s) is waited out and the change goes; a blind spot holding the change (LC-01, under
  the switch) stops the clock, the car beside hiding the line."""
  drive = Drive(lanes_left=1, lanes_right=0)
  drive.car_kw, drive.car_lamp = {'lit_m': 0., 'lead_m': 91.44}, True
  m = maneuver("turn", "left")
  distance = drive.run(m, lane_change_window(drive.v, 1, offer_room_s(drive.v, 91.44))[0] - 1., LC_CONFIRM_FRAMES * DT_MDL + 0.5)
  assert drive.lp.phase == Phase.WARN and drive.lp.signal_side == LaneChange.left
  drive.paint = lambda left, right: paint(left=UNKNOWN, right=SW)
  lit = 0
  while distance > 20.:
    drive.step(m, distance)
    distance -= drive.v * DT_MDL
    lit += drive.lp.signal_side != LaneChange.none
  assert [a for a, _ in drive.aborts] == ['lineNotBroken'] and not drive.requests
  assert lit * DT_MDL == pytest.approx(LC_UNSURE_MAX_S, abs=2 * DT_MDL), "lit 14.2 s before"
  # the colour of a dash out 1.95 s across the countdown's end: waited out, the change made on the next broken white read
  drive, m, distance = offered_left()
  distance = drive.run(m, distance, drive.lp.countdown - 1.)
  drive.paint = lambda left, right: paint(left=COLORLESS, right=SW)
  distance = drive.run(m, distance, 1.95)
  assert drive.lp.phase == Phase.REQUEST and not drive.requests and not drive.aborts
  drive.paint = striped
  drive.run(m, distance, 8.)
  assert not drive.aborts and [r for r, _, _ in drive.requests] == [LaneChange.left] and drive.lp.changes == 1
  # the driver's stalk path: past the countdown WAIT asks for the blinker up to LC_SIGNAL_TIMEOUT_S; unsure, the prompt goes
  # LC_UNSURE_MAX_S after the last read, aborted lineNotBroken, not noSignal 8 s on
  drive, m, distance = offered_left(driver_signals=True)
  drive.paint = lambda left, right: paint(left=UNKNOWN, right=SW)
  last_read = drive.t
  distance = drive.run(m, distance, drive.lp.countdown + 0.2)
  assert drive.lp.phase == Phase.WAIT and drive.decisions[-1].prompt == LaneChange.left and not drive.aborts
  drive.run(m, distance, LC_SIGNAL_TIMEOUT_S)
  assert [a for a, _ in drive.aborts] == ['lineNotBroken'] and not drive.requests and drive.decisions[-1].prompt == LaneChange.none
  assert drive.aborts[0][1] - last_read == pytest.approx(LC_UNSURE_MAX_S, abs=2 * DT_MDL)
  # held for the blind spot (LC-01): its clock stops while the car beside holds the change, and runs again once it's clear
  drive, m, distance = offered_left()
  always(drive)
  drive.paint = lambda left, right: paint(left=UNKNOWN, right=SW)
  distance = drive.run(m, distance, 2. * LC_UNSURE_MAX_S, blindspot=LaneChange.left)
  assert not drive.aborts and drive.decisions[-1].hold == 'blindspot' and drive.decisions[-1].offer == LaneChange.left
  clear = drive.t
  drive.run(m, distance, 2. * LC_UNSURE_MAX_S)
  assert [a for a, _ in drive.aborts] == ['lineNotBroken'] and not drive.requests
  assert LC_UNSURE_MAX_S - 2 * DT_MDL < drive.aborts[0][1] - clear < LC_UNSURE_MAX_S + LC_ABORT_SETTLE_S + 2 * DT_MDL


def test_frames_between_two_paint_reads_go_by_the_last_read():
  """Route 29 (2026-11-08): the lane paint came on 35% of plan frames, two read then two to four not (the detector's road
  camera runs). A frame between two reads counted as one nobody read, so the confirm count took twice as long (521 s: a left
  100 m out, offered 1.0 s after its first passing frame) and, with the driver signalling, the request went out on each
  read and was withdrawn on each frame between, 11 times in 2.1 s. Such a frame goes by the last read (PAINT_CARRY_S)."""
  from openpilot.selfdrive.turnpilot.lane_planner import PAINT_CARRY_S
  m = maneuver("turn", "left")
  for unread in (2, 4):
    drive = Drive(lanes_left=1, lanes_right=0, driver_signals=True)
    n = {'k': 0}
    def cadence(left, right, n=n, unread=unread):
      n['k'] += 1
      return striped(left, right) if (n['k'] - 1) % (2 + unread) < 2 else ()
    drive.paint = cadence
    assert unread * DT_MDL <= PAINT_CARRY_S
    distance = drive.run(m, 290., LC_CONFIRM_FRAMES * DT_MDL)
    assert len(drive.offers) == 1 and drive.offers[0][2] == pytest.approx((LC_CONFIRM_FRAMES - 1) * DT_MDL), unread
    k = len(drive.decisions)
    drive.run(m, distance, 6., blinker=LaneChange.left)
    # asked for once and held until DesireHelper starts it: never withdrawn on a frame between two reads
    asked = [d.lane_change == LaneChange.left for d in drive.decisions[k:] if d.offer == LaneChange.left]
    assert asked and all(asked) and len(drive.requests) == 1 and drive.lp.changes == 1 and not drive.aborts, unread


def test_a_read_older_than_the_carry_is_no_read():
  """The carry bridges the reader's own cadence, nothing longer: a wide camera run past PAINT_CARRY_S is a frame nobody
  read, as before (kept, never counted), and the last read of the lane the car left is never carried into the next."""
  from openpilot.selfdrive.turnpilot.lane_planner import PAINT_CARRY_S
  m = maneuver("turn", "left")
  drive = Drive(lanes_left=1, lanes_right=0, driver_signals=True)
  n = {'k': 0}
  def two_reads(left, right):
    n['k'] += 1
    return striped(left, right) if n['k'] <= 2 else ()
  drive.paint = two_reads
  drive.run(m, 290., 2.)
  assert not drive.offers and drive.decisions[-1].wait == 'lineNotBroken'
  assert drive.lp._confirm == 2 + int(round(PAINT_CARRY_S / DT_MDL))


def test_a_change_may_start_as_late_as_a_driver_starts_one():
  """Route 29, 519-530 s: a left on a 100 m leg, the driver's lamp on 92 m out and the line crossed 40-25 m out at 6-7 m/s.
  The window's end was 60 m at any city speed, so the offer made 82 m out died tooLate 2 s later and the left was skipped.
  The end is LC_END_S of travel, LC_END_MIN_M at least."""
  from openpilot.selfdrive.turnpilot.lane_planner import LC_END_MIN_M, LC_END_S
  assert lane_change_window(10.)[1] == pytest.approx(LC_END_S * 10.) and lane_change_window(10.)[1] <= 45.
  assert lane_change_window(5.)[1] == LC_END_MIN_M <= 35.
  m = maneuver("turn", "left")
  drive = Drive(lanes_left=1, lanes_right=0, v=10., driver_signals=True)
  distance = drive.run(m, 100., LC_CONFIRM_FRAMES * DT_MDL)
  drive.run(m, distance, 7., blinker=LaneChange.left)
  assert drive.offers and drive.requests and drive.lp.changes == 1 and not drive.aborts
  assert not any(d.blocked for d in drive.decisions)


def test_an_aborted_change_is_offered_again_soon():
  """An abort keeps the same change back LC_ABORT_COOLDOWN_S, then the confirm count: 3 s, not 6 (80 m at 13 m/s)."""
  from openpilot.selfdrive.turnpilot.lane_planner import LC_ABORT_COOLDOWN_S
  m = maneuver("turn", "left")
  drive = Drive(lanes_left=1, lanes_right=0, v=13., driver_signals=True)
  from types import SimpleNamespace
  distance = drive.run(m, 250., LC_CONFIRM_FRAMES * DT_MDL)
  assert len(drive.offers) == 1
  car = [SimpleNamespace(x=5., y=LANE_WIDTH, vx=13.)]
  distance = drive.run(m, distance, DT_MDL, objects=car)
  assert [a for a, _ in drive.aborts] == ['carInLane']
  drive.run(m, distance, 4.)
  assert len(drive.offers) == 2
  assert drive.offers[1][2] - drive.aborts[0][1] == pytest.approx(LC_ABORT_COOLDOWN_S + LC_CONFIRM_FRAMES * DT_MDL, abs=2 * DT_MDL)
  assert LC_ABORT_COOLDOWN_S <= 3.


TURN_BAY = ((("left",), True), (("straight",), False))  # route 29's 216-243 s left: the bay and the one through lane
SOLID_NO_COLOUR = LinePaint('solid', 'unknown', 1., 0.95)


def test_which_change_is_into_a_named_bay():
  from openpilot.selfdrive.turnpilot.lane_planner import bay_named
  assert bay_named(maneuver("turn", "left", TURN_BAY), 1)
  assert bay_named(maneuver("turn", "right", TWO_LANES_RIGHT_TURN), -1)
  assert not bay_named(maneuver("turn", "left", THREE_LANES_LEFT_TURN), 2), "the first of two lands in a through lane"
  assert not bay_named(maneuver("turn", "left", TURN_BAY), 0)
  assert not bay_named(maneuver("turn", "left"), 1), "no lane list names it"
  assert not bay_named(maneuver("turn", "left", (((), False), ((), False))), 1), "a list that marks no lane for the turn"
  assert not bay_named(maneuver("off ramp", "slight right", TWO_LANES_RIGHT_TURN), -1), "an exit is no turn bay"
  assert not bay_named(None, 1)


def test_a_named_bays_solid_white_line_may_be_crossed_and_nothing_else():
  """The owner's ruling (2026-11-09): a change into a lane the lane list marks for the coming turn may cross a single line
  read solid white where the model sees the lane there and it is free. Never yellow, never a solid line of no colour (toward
  the far side it can be the centre line), and with no bay named a solid line is no change, as before."""
  lane, night = model_lanes(*road(1, 0)), model_lanes(*road(1, 0, shoulder=0.5)[:1], [0.05, 0.9, 0.9, 0.05], *road(1, 0)[2:])
  assert lane.left == 1 and night.left == 0 and night.left_open
  for lanes in (lane, night):  # the lane counted from its painted line, or a lane's room to a trusted edge beyond the white
    assert change_blocker(LEFT, lanes, paint(left=SW), nav_room=1, bay=True) == ''
    assert change_blocker(LEFT, lanes, paint(left=SW), nav_room=1) == 'lineNotBroken', "no bay named: unchanged"
    assert change_blocker(LEFT, lanes, paint(left=SW)) == 'lineNotBroken'
  for line in (SY, BY, LATCHED_YELLOW, LinePaint('solid', 'yellow', 0.3, 0.95), LinePaint('unknown', 'yellow', 0.35, 0.)):
    assert change_blocker(LEFT, lane, paint(left=line), nav_room=1, bay=True) == 'lineNotBroken', line
    assert change_blocker(RIGHT, model_lanes(*road(0, 1)), paint(right=line), nav_room=1, bay=True) == 'lineNotBroken', line
  assert change_blocker(LEFT, lane, paint(left=SOLID_NO_COLOUR), nav_room=1, bay=True) == 'lineNotBroken', "colourless: maybe the centre line"
  assert change_blocker(LEFT, lane, paint(left=LinePaint('solid', 'white', 0.4, 0.95)), nav_room=1, bay=True) == 'lineNotBroken'
  assert change_blocker(LEFT, lane, paint(left=NO_LINE), nav_room=1, bay=True) == 'lineNotBroken'
  # the yellow hold over a line now read solid white: still yellow
  hold = YellowHold()
  hold.update(paint(left=SY), 10., DT_MDL, False)
  assert change_blocker(LEFT, lane, hold.update(paint(left=SW), 10., DT_MDL, False), nav_room=1, bay=True) == 'lineNotBroken'
  # the model sees no lane there, the list has none, or it isn't free
  assert change_blocker(LEFT, model_lanes(*road(0, 0)), paint(left=SW), nav_room=1, bay=True) == 'noLane'
  assert change_blocker(LEFT, lane, paint(left=SW), nav_room=0, bay=True) == 'noLane'
  oncoming, parked = scene_obj(30., LANE_WIDTH, -8.), scene_obj(30., LANE_WIDTH, 0.)
  assert change_blocker(LEFT, lane, paint(left=SW), (oncoming,), nav_room=1, bay=True) == 'noLane'
  assert change_blocker(LEFT, lane, paint(left=SW), (parked,), nav_room=1, bay=True) == 'laneParked'
  # toward the curb side the same: a right turn's bay behind solid white
  assert change_blocker(RIGHT, model_lanes(*road(0, 1)), paint(right=SW), nav_room=1, bay=True) == ''
  assert change_blocker(RIGHT, model_lanes(*road(0, 1)), paint(right=SOLID_NO_COLOUR), nav_room=1, bay=True) == 'lineNotBroken'


def scene_obj(x, y, vx):
  from types import SimpleNamespace
  return SimpleNamespace(x=x, y=y, vx=vx)


def solid_bay_drive(line, lanes=TURN_BAY, v=8.):
  """A left 90 m ahead from the through lane beside its bay, the bay's line reading `line`, the driver signalling left."""
  drive = Drive(1, 0, v=v, driver_signals=True, paint=lambda left, right: paint(left=line if left else SY, right=SW))
  m = maneuver("turn", "left", lanes)
  distance = 90.
  while distance > 20.:
    drive.step(m, distance, blinker=LaneChange.left)
    distance -= v * DT_MDL
  return drive


def test_the_car_changes_into_a_named_bay_across_solid_white():
  """Route 29 (2026-11-08, 239-241 s): a left's bay opened 48 m out behind a line read solid white 0.76-0.91, the lane list
  [left, straight]; the planner never crosses solid, wanted the change to the window's end and LC-03 skipped the left. The
  driver crossed it."""
  drive = solid_bay_drive(SW)
  assert [o for o, _, _ in drive.offers] == [LaneChange.left] and [r for r, _, _ in drive.requests] == [LaneChange.left]
  assert drive.lp.changes == 1 and not drive.aborts and not any(d.blocked for d in drive.decisions)


FAINT_YELLOW = LinePaint('solid', 'yellow', 0.4, 0.95)  # under the bay latch's bar: the change stays wanted, the line refuses it


@pytest.mark.parametrize("line, lanes", [(SY, TURN_BAY), (BY, TURN_BAY), (FAINT_YELLOW, TURN_BAY), (SOLID_NO_COLOUR, TURN_BAY), (SW, ())])
def test_no_change_across_yellow_colourless_or_unnamed_solid(line, lanes):
  """Yellow: never. A solid line of no colour leftwards: never. Solid white with no bay named by the lane list: as before."""
  drive = solid_bay_drive(line, lanes)
  if line is FAINT_YELLOW:
    assert not drive.lp.bay.shut and any(d.want == 1 for d in drive.decisions)
  assert not drive.offers and not drive.requests and drive.lp.changes == 0


def test_a_bay_never_entered_is_still_skipped_at_the_windows_end():
  """The wrong-lane rail (LC-03) with the bay's grant: the bay's line read solid white, the model seeing no lane beyond it,
  is noLane under the grant, and noLane at the window's end lets a turn go from the lane the car is in. LC-03 judges without
  the grant: the left's lane not reached, the left is skipped."""
  lp = LanePlanner()
  m = maneuver("turn", "left", TURN_BAY)
  no_lane = model_lanes(*road(0, 0))
  assert change_blocker(LEFT, no_lane, paint(left=SW), nav_room=1, bay=True) == 'noLane'
  distance, decisions = 60., []
  while distance > 25.:
    decisions.append(lp.update(m, distance, no_lane, LaneChangeState.off, 8., True, False, (), True, True, DT_MDL, paint(left=SW, right=SW)))
    distance -= 8. * DT_MDL
  assert all(d.want == 1 for d in decisions) and not any(int(d.offer) for d in decisions)
  assert decisions[-1].blocked
