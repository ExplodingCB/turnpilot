"""Junction turns only, and too fast to turn means skip the turn (turn_skip.py): the feasibility numbers, the skip with
its alert, and never a cancel once the desire is out."""
import math
import time

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.desire_timing import TURN_DESIRE_LEAD_M, DesireTiming
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE, plan_events
from openpilot.selfdrive.turnpilot.route_follower import LEFT, NONE, TURN, RouteFollower, maneuver_from_instruction
from openpilot.selfdrive.turnpilot.tests.synthetic import FakeNavd, junction_of, nav_instruction
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
from openpilot.selfdrive.turnpilot.turn_skip import MIN_FIRST_PULSE_M, SKIP_CONFIRM_FRAMES, TurnSkip, can_make_turn, safe_turn_speed
from openpilot.selfdrive.turnpilot.turn_speed import NO_SPEED_CAP

Desire = log.Desire
State = custom.TurnpilotPlan.State


def latest_feasible(distance, angle, curb_side=True):
  """The highest speed can_make_turn allows, by bisection."""
  lo, hi = 0., 60.
  for _ in range(60):
    mid = (lo + hi) / 2
    lo, hi = (mid, hi) if can_make_turn(mid, distance, angle, curb_side) else (lo, mid)
  return lo


def test_safe_turn_speeds():
  # sqrt(2 m/s^2 * R), R = 8 m (curb side) or 12 m (across the road) / tan(angle / 2)
  assert math.isclose(safe_turn_speed(90., curb_side=True), 4.0)
  assert math.isclose(safe_turn_speed(90., curb_side=False), math.sqrt(24.))
  assert math.isclose(safe_turn_speed(45., curb_side=True), math.sqrt(16. / math.tan(math.radians(22.5))))  # 6.2
  assert 2.5 < safe_turn_speed(135., curb_side=True) < 2.6
  assert safe_turn_speed(5., curb_side=True) == 15.  # V_TURN_MAX


@pytest.mark.parametrize("distance, angle, curb_side, v_max", [
  (40., 90., True, 11.416),   # the desire's lead
  (100., 90., True, 18.494),
  (150., 90., True, 22.900),
  (40., 90., False, 11.711),  # a left across the road is a wider arc
  (40., 135., True, 11.062),  # sharper is slower
  (40., 45., True, 12.235),
  (10., 90., True, 5.746),    # near the junction only the corner speed is left
])
def test_feasibility_at_speeds_and_angles(distance, angle, curb_side, v_max):
  # v^2 <= v_safe^2 + 2 * 2.0 * (d - v * 1.0)
  assert abs(latest_feasible(distance, angle, curb_side) - v_max) < 0.01
  assert can_make_turn(v_max - 0.05, distance, angle, curb_side)
  assert not can_make_turn(v_max + 0.05, distance, angle, curb_side)


def test_the_real_angle_decides():
  # 14 m/s 60 m out: a 90 degree right can be made, a 135 degree one can't
  assert can_make_turn(14., 60., 90., True) and not can_make_turn(14., 60., 135., True)


def left_at(s, angle=90., branches=2, modifier="left"):
  return (s, "turn", modifier, junction_of(modifier, branches, angle))


def plan_reader(plan):
  """A SubMaster stand-in holding this plan, received just now."""
  class SM(dict):
    pass
  sm = SM(turnpilotPlan=plan)
  sm.valid, sm.recv_time = {'turnpilotPlan': True}, {'turnpilotPlan': time.monotonic()}
  return sm


def test_too_fast_skips_the_turn_alerts_and_never_brakes_for_it():
  # a new route puts a left 50 m ahead of a car at 15 m/s: it can't get down to 4.9 m/s by then at 2 m/s^2
  sim = Sim([left_at(50.), left_at(900.)], cruise=15.)
  sim.run(10.)
  assert sim.s > 120.
  near = [plan for s, _, plan in sim.trace if s < 60.]
  assert all(plan.desire == Desire.none for plan in near)
  skipped = [plan.tooFastToTurn for plan in near]
  assert not any(skipped[:SKIP_CONFIRM_FRAMES - 1]) and all(skipped[SKIP_CONFIRM_FRAMES - 1:])
  # once skipped, no speed cap for it (no hard braking for a turn), and no turn state; until then the cap braked at its
  # own 2 m/s^2 limit for the frames the skip takes to confirm
  assert all(plan.speedCap == NO_SPEED_CAP for plan in near[SKIP_CONFIRM_FRAMES - 1:])
  assert min(v for s, v, _ in sim.trace) > 15. - 2. * SKIP_CONFIRM_FRAMES * DT_MDL - 1e-6
  assert all(plan.state != State.turning for plan in near)
  # the alert, as its own event
  assert plan_events(plan_reader(near[-1])) == ['turnpilotTooFastToTurn']


def test_skip_also_without_junction_data():
  # nominal 90 degrees when the route has no bearings: the same rule
  sim = Sim([(50., "turn", "left"), (900., "turn", "right")], cruise=15.)
  sim.run(3.)
  assert any(plan.tooFastToTurn for _, _, plan in sim.trace)
  assert all(plan.desire == Desire.none for _, _, plan in sim.trace)


def test_skip_ends_with_the_maneuver():
  # navd moves on (here, to the next turn): the skip was for the one maneuver
  sim = Sim([left_at(50.), left_at(400.)], cruise=15.)
  sim.run(14.)
  assert any(plan.tooFastToTurn for s, _, plan in sim.trace if s < 50.)
  assert not any(plan.tooFastToTurn for s, _, plan in sim.trace if s > 90.)


@pytest.mark.parametrize("modifier", ["left", "right"])  # across the road (v_safe 4.9) and to the curb side (4.0, stricter)
@pytest.mark.parametrize("angle", [70., 90., 135.])
@pytest.mark.parametrize("cruise", [13., 25.])
def test_following_the_cap_never_skips(modifier, angle, cruise):
  # from town or highway speed, a car that follows the turn speed cap is always under the feasibility curve
  sim = Sim([left_at(800., angle, modifier=modifier), left_at(2500., angle, modifier=modifier)], cruise=cruise)
  sim.run(80., turn_at=800.)
  assert sim.s > 820.
  assert not any(plan.tooFastToTurn for _, _, plan in sim.trace)
  desire = Desire.turnLeft if modifier == "left" else Desire.turnRight
  assert any(plan.desire == desire for _, _, plan in sim.trace)


def test_a_road_that_bends_is_the_models():
  # the route bends left where no other road meets it: no desire, no speed cap, nothing to skip (the left 1200 m past
  # the bend is looked ahead to, and its cap is far above the cruise speed)
  sim = Sim([left_at(300., branches=0), left_at(1500.)], cruise=15.)
  sim.run(25., turn_at=300.)
  assert sim.s > 320.
  before = [plan for s, _, plan in sim.trace if s < 310.]
  assert all(plan.desire == Desire.none and plan.speedCap > 30. and not plan.tooFastToTurn for plan in before)
  assert min(v for s, v, _ in sim.trace if s < 310.) > 14.9
  assert all(plan.maneuverBend and plan.state == State.following for plan in before)
  # at a junction the same left gets both
  sim = Sim([left_at(300.), left_at(1500.)], cruise=15.)
  sim.run(40., turn_at=300.)
  assert any(plan.desire == Desire.turnLeft for _, _, plan in sim.trace)
  assert any(plan.speedCap < 15. for _, _, plan in sim.trace)


def test_bend_classified_from_branches():
  bend = maneuver_from_instruction(nav_instruction(100., "turn", "left", junction=junction_of("left", branches=0)))
  turn = maneuver_from_instruction(nav_instruction(100., "turn", "left", junction=junction_of("left", branches=1, angle=60.)))
  nominal = maneuver_from_instruction(nav_instruction(100., "turn", "left"))
  assert (bend.kind, bend.bend) == (NONE, True)
  assert (turn.kind, turn.side, turn.angle_deg, turn.geometry) == (TURN, LEFT, 60., True)
  assert (nominal.kind, nominal.angle_deg, nominal.geometry) == (TURN, 90., False)
  keep = maneuver_from_instruction(nav_instruction(100., "fork", "slight left", junction=(0., 340., 0)))
  assert keep.kind == NONE  # a "fork" with no other branch is a bend too


def drive_committed(speed_after, turn=True):
  """A left at 200 m approached at 6 m/s; the desire goes out at the lead, then the driver's foot takes the car to
  speed_after. Returns [(s, desire, skipped)] per frame."""
  navd, rf, timing, skip = FakeNavd([left_at(200.), left_at(900.)]), RouteFollower(), DesireTiming(), TurnSkip()
  s, out = 0., []
  for _ in range(int(60. / DT_MDL)):
    v = 6. if timing.started is False or 200. - s > TURN_DESIRE_LEAD_M - 5. else speed_after
    instr = navd.update(s)
    rf.update(instr, instr is not None, True, v, DT_MDL)
    yaw = v / 8. if turn and rf.upcoming is not None and rf.distance < 8. else 0.
    state = skip.update(rf.upcoming, rf.distance, rf.maneuver_id, v, timing.maneuver_id == rf.maneuver_id and timing.started)
    desire = timing.update(None if state.skipped else rf.upcoming, rf.distance, rf.maneuver_id, rf.passed, v, 0., True, True,
                           DT_MDL, yaw, may_start=state.feasible_now)
    out.append((s, desire, state.skipped))
    s += v * DT_MDL
    if s > 260.:
      break
  return out


def test_no_cancel_once_the_desire_is_out():
  # 14 m/s from 35 m out is far too fast for the left, but the desire is out: it is held to the junction, never cleared
  # before the car turns, and the turn is never skipped
  out = drive_committed(14.)
  assert not any(skipped for _, _, skipped in out)
  first = next(s for s, desire, _ in out if desire == Desire.turnLeft)
  assert 200. - first <= TURN_DESIRE_LEAD_M
  assert all(desire == Desire.turnLeft for s, desire, _ in out if first < s < 192.)  # no clear, no gap, no repeat edge
  # the clear comes only once the car has turned the turn's angle
  turned_at = next(s for s, desire, _ in out if desire == CLEAR_DESIRE)
  assert turned_at > 192.
  # and with the car never turning, the desire is held into the junction and never cleared
  out = drive_committed(14., turn=False)
  assert all(desire == Desire.turnLeft for s, desire, _ in out if first < s < 200.)
  assert not any(desire == CLEAR_DESIRE for _, desire, _ in out)


def test_turn_skip_unit():
  skip = TurnSkip()
  m = maneuver_from_instruction(nav_instruction(30., "turn", "left", junction=junction_of("left")))
  # infeasible, but the desire is out: never skipped, only no new pulse
  for _ in range(50):
    state = skip.update(m, 30., 1, 15., desire_out=True)
    assert not state.skipped and not state.feasible_now
  # a new maneuver: SKIP_CONFIRM_FRAMES infeasible frames in a row, and one feasible frame resets the count
  for _ in range(SKIP_CONFIRM_FRAMES - 1):
    assert not skip.update(m, 30., 2, 15., desire_out=False).skipped
  assert skip.update(m, 30., 2, 4., desire_out=False).feasible_now
  for _ in range(SKIP_CONFIRM_FRAMES - 1):
    assert not skip.update(m, 30., 2, 15., desire_out=False).skipped
  assert skip.update(m, 30., 2, 15., desire_out=False).skipped
  # latched for this maneuver, even slowed right down
  assert skip.update(m, 30., 2, 0., desire_out=False).skipped
  assert not skip.update(m, 30., 3, 0., desire_out=False).skipped


def test_turn_ahead_of_a_bend_too_close_for_its_first_pulse_is_announced_skipped():
  # a left MIN_FIRST_PULSE_M or less past where the car is, after a bend: it becomes the maneuver only past the bend,
  # too late for its first pulse, so its skip is announced now; the bend itself isn't skipped
  skip = TurnSkip()
  bend = maneuver_from_instruction(nav_instruction(0., "turn", "right", junction=junction_of("right", 0, 60.)))
  left = maneuver_from_instruction(nav_instruction(30., "turn", "left", junction=junction_of("left")))
  assert bend.kind == NONE and left.kind == TURN
  state = skip.update(bend, 0., 1, 4., False, ahead=left, ahead_distance=30.)
  assert not state.skipped and state.feasible_now and not state.ahead_skipped
  state = skip.update(bend, 0., 1, 4., False, ahead=left, ahead_distance=MIN_FIRST_PULSE_M - 1.)
  assert not state.skipped and state.feasible_now and state.ahead_skipped
  # not while openpilot isn't steering (no first pulse would go, nor is the turn skipped under the driver)
  assert not skip.update(bend, 0., 1, 4., False, lat_active=False, ahead=left, ahead_distance=5.).ahead_skipped
  # once it is the maneuver, it is skipped as too late
  assert skip.update(left, MIN_FIRST_PULSE_M - 6., 2, 4., False).skipped


def test_shadow_plan_raises_no_alert():
  msg = messaging.new_message('turnpilotPlan')
  msg.turnpilotPlan.tooFastToTurn, msg.turnpilotPlan.shadow = True, True
  assert plan_events(plan_reader(msg.as_reader().turnpilotPlan)) == []


def test_turn_only_lane_flagged_and_shown():
  # the route goes straight through an intersection at 300 m (the left at 900): the car sits in its left-turn-only lane
  lanes = ((("left",), False), (("straight",), True), (("straight", "right"), True))
  sim = Sim([left_at(900.), left_at(2000.)], cruise=13., model_lanes=(0, 2))
  sim.navd.through = {0: (lanes, 300.)}
  sim.run(26.)
  mismatch = [(s, plan.laneMismatch) for s, _, plan in sim.trace]
  flagged = [s for s, m in mismatch if m == custom.TurnpilotPlan.LaneMismatch.turnOnlyGoingStraight]
  assert flagged and 300. - 78. < flagged[0] < 300. - 78. + 20 * DT_MDL * 13. + 1.  # 1 s after the planner's last chance
  assert max(flagged) < 301.
  assert all(m == custom.TurnpilotPlan.LaneMismatch.none for s, m in mismatch if s > 301.)
  plan = next(plan for s, _, plan in sim.trace if s == flagged[-1])
  assert plan_events(plan_reader(plan)) == ['turnpilotWrongLane']
  # in a through lane, nothing
  sim = Sim([left_at(900.), left_at(2000.)], cruise=13., model_lanes=(1, 1))
  sim.navd.through = {0: (lanes, 300.)}
  sim.run(26.)
  assert all(plan.laneMismatch == custom.TurnpilotPlan.LaneMismatch.none for _, _, plan in sim.trace)


def test_stop_before_the_turn_means_not_too_fast():
  # a map stop sign or signal the car stops at within 30 m before the junction: the turn is taken from a stop
  turn = maneuver_from_instruction(nav_instruction(60., "turn", "right", junction=junction_of('right')))
  assert not can_make_turn(20., 60., turn.angle_deg, True)
  for stop_d, skipped in ((52., False), (35., False), (25., True), (None, True), (65., True)):
    skip = TurnSkip()
    for _ in range(SKIP_CONFIRM_FRAMES + 1):
      state = skip.update(turn, 60., 1, 20., False, stop_d=stop_d)
    assert state.skipped == skipped, stop_d
    assert state.feasible_now == (not skipped), stop_d


def test_a_turn_whose_first_pulse_is_its_go_is_never_too_late():
  # the autonomous unprotected left sends no pulse before its hold line (turnpilotd: may_start off), so no first pulse by
  # MIN_FIRST_PULSE_M is no reason to skip it: its first pulse is the hold's go, from the line
  left = maneuver_from_instruction(nav_instruction(40., "turn", "left", junction=junction_of("left")))
  skip = TurnSkip()
  for d in (MIN_FIRST_PULSE_M + 5., MIN_FIRST_PULSE_M - 1., 9.):
    state = skip.update(left, d, 1, 3., False, pulse_at_go=True)
    assert not state.skipped, d
  # without it, as before
  assert TurnSkip().update(left, MIN_FIRST_PULSE_M - 1., 1, 3., False).skipped
