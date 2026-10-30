import json
import math
from pathlib import Path

import openpilot.cereal.messaging as messaging
import pytest
from openpilot.cereal import custom, log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.turnpilot.hooks import ABORT_HOLD_S
from openpilot.selfdrive.turnpilot.lane_planner import (LANE_UNKNOWN_REASON, LANE_WIDTH,
                                                        LC_ABORT_COOLDOWN_S, LC_ABORT_SETTLE_S, LC_CONFIRM_FRAMES, LC_REQUEST_TIMEOUT_S,
                                                        LC_SIGNAL_TIMEOUT_S, LC_WARN_S, YELLOW_HOLD_S, LanePlanner, ModelLanes, Phase,
                                                        slow_zone,
                                                        YellowHold, change_blocker, desired_changes, ego_lane_index, lane_change_window,
                                                        lane_mismatch,
                                                        lane_changes_needed, model_lanes, model_lanes_from_msg, nav_lane_span, stopped_in_lane,
                                                        target_lane_blocked)
from openpilot.selfdrive.turnpilot.route_follower import LEFT, NOMINAL_ANGLE_DEG, RIGHT, Maneuver, classify
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint
from openpilot.selfdrive.turnpilot.tests.synthetic import car_state, model_v2, road, scene

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
  def __init__(self, lanes_left, lanes_right, v=15., driver_signals=False, paint=striped, paint_rule=True, lag=0, curb_unpainted=False):
    """paint: (lanes left, lanes right) -> the four lines' paint where the car is now, or None for none read. lag: frames
    the planner sees DesireHelper's state late by (turnpilotd isn't synced to modeld, whose modelV2 carries it).
    curb_unpainted: the curb lane has no edge line, so one lane from the curb the model sees room there but no line."""
    self.lanes_left, self.lanes_right, self.v = lanes_left, lanes_right, v
    self.curb_unpainted = curb_unpainted
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
    cs = car_state(self.v, left_blinker=left_blinker, right_blinker=right_blinker)
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


def test_never_without_broken_white():
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  for p in NOT_BW:
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


def test_tesla_no_signal_aborts_and_offers_again_later():
  drive = Drive(lanes_left=0, lanes_right=1, driver_signals=True)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 250., LC_CONFIRM_FRAMES * DT_MDL + LC_WARN_S + LC_SIGNAL_TIMEOUT_S + 0.5, moving=False)
  assert [a for a, _ in drive.aborts] == ['noSignal'] and not drive.requests
  t_abort = drive.aborts[0][1]
  drive.run(m, distance, LC_ABORT_COOLDOWN_S + 1., moving=False)
  held = [d.abort for d, t in zip(drive.decisions, np_times(drive), strict=True) if t >= t_abort - 1e-6]
  n = int(round(ABORT_HOLD_S / DT_MDL))
  assert held[:n] == ['noSignal'] * n and held[n + 1] == 'none'  # held for the alert, then gone
  assert len(drive.offers) == 2 and drive.offers[1][2] - t_abort >= LC_ABORT_COOLDOWN_S


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

  def offered(**kw):
    drive = Drive(lanes_left=0, lanes_right=1, **kw)
    distance = drive.run(m, 290., 1.5)
    assert drive.lp.phase == Phase.WARN
    return drive, distance

  # the line turns solid, unknown, or stops being read
  for p in (SW, UNKNOWN, None):
    drive, distance = offered()
    drive.paint = (lambda left, right, p=p: paint(right=p)) if p is not None else None
    drive.run(m, distance, 1.)
    assert [a for a, _ in drive.aborts] == ['lineNotBroken'] and not drive.requests, p
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
  # the car slows below DesireHelper's speed
  drive, distance = offered()
  drive.v = 7.
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


def test_blind_spot_or_car_at_the_offer_holds_it_back():
  drive = Drive(lanes_left=0, lanes_right=1)
  m = maneuver("turn", "right", TWO_LANES_RIGHT_TURN)
  distance = drive.run(m, 300., 3., blindspot=LaneChange.right)
  assert not drive.offers and drive.decisions[-1].reason == "car in blind spot"
  beside = scene([{'cls': 'car', 'x': -3., 'y': -3.6, 'vx': 15.}]).objects
  distance = drive.run(m, distance, 2., objects=beside)
  assert not drive.offers and drive.decisions[-1].reason == "car in target lane"
  drive.run(m, distance, 6.)
  assert [r for r, _, _ in drive.requests] == [LaneChange.right]


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
  drive = Drive(lanes_left=0, lanes_right=2, v=7.)
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
  distance = drive.drive(m, 255., 25.)  # up to the stop and held there
  assert not drive.offers and drive.v == 0. and drive.held_t > 1.
  assert {w for d, _, w in drive.waits if d < 250.} == {'afterStop'}
  drive.cruise, drive.hold_s = 13., 0.
  drive.drive(m, distance, 25.)
  assert drive.offers and drive.lp.changes == 1, (drive.offers, drive.aborts, drive.waits[-1])
  assert all(60. < d < 180. for _, d, _ in drive.offers)


def test_with_no_room_before_the_stop_nor_after_it_the_lane_is_unreachable_and_said():
  """A stop 100 m before the turn: not enough road after its go to get up to speed and change. Too close to it at speed
  for one before it. The turn's lane can't be reached: unreachable, never a late change."""
  drive = StopDrive(2, 0, cruise=11., stop_at=100.)
  m = maneuver("turn", "left", THREE_LANES_LEFT_TURN)
  drive.drive(m, 190., 4.)
  d = drive.decisions[-1]
  assert not drive.offers and d.wait == 'unreachable' and d.unreachable and d.want == 2
  assert 'reach' in d.reason
  # without the map's lane list the car's lane isn't known to be wrong: said, never a skip
  drive = StopDrive(1, 0, cruise=11., stop_at=100.)
  drive.drive(maneuver("turn", "left"), 190., 4.)
  d = drive.decisions[-1]
  assert d.wait == 'unreachable' and not d.unreachable


def test_lane_change_wait_says_why_no_change_is_offered():
  drive = Drive(1, 0, v=8.)  # below DesireHelper's speed
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
