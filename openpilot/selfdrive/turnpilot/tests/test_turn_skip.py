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
from openpilot.selfdrive.turnpilot.turn_skip import (CURB_FIRST_PULSE_M, FAR_FIRST_PULSE_MIN_M, MIN_FIRST_PULSE_M, SKIP_CONFIRM_FRAMES,
                                                      STANDING_PULSE_MIN_M, STANDING_PULSE_V, TurnSkip, can_make_turn, first_pulse_latest,
                                                      safe_turn_speed)
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
  # at 7 m/s, where a far-side turn's deadline is MIN_FIRST_PULSE_M (first_pulse_latest)
  state = skip.update(bend, 0., 1, 7., False, ahead=left, ahead_distance=30.)
  assert not state.skipped and state.feasible_now and not state.ahead_skipped
  state = skip.update(bend, 0., 1, 7., False, ahead=left, ahead_distance=MIN_FIRST_PULSE_M - 1.)
  assert not state.skipped and state.feasible_now and state.ahead_skipped
  # not while openpilot isn't steering (no first pulse would go, nor is the turn skipped under the driver)
  assert not skip.update(bend, 0., 1, 7., False, lat_active=False, ahead=left, ahead_distance=5.).ahead_skipped
  # nor at or past its junction, nor standing or crawling STANDING_PULSE_MIN_M or more short of it (as tooLate itself)
  for ahead_distance, v in ((0., 7.), (-4.8, 7.), (12.3, 0.39), (STANDING_PULSE_MIN_M, 0.)):
    assert not TurnSkip().update(bend, 0., 1, v, False, ahead=left, ahead_distance=ahead_distance).ahead_skipped, (ahead_distance, v)
  # once it is the maneuver, it is skipped as too late
  assert skip.update(left, MIN_FIRST_PULSE_M - 6., 2, 7., False).skipped


def test_a_turn_waiting_past_a_wrong_way_opening_is_too_late_only_past_it():
  """A turn whose pulse waits past an opening on its side the car may not enter (navd's maneuverWrongWayM; desire_timing:
  wrong_way_wait) can't pulse by MIN_FIRST_PULSE_M: its tooLate deadline, and the announced skip of one ahead of a bend,
  move in to that point less WRONG_WAY_GRACE_M."""
  from openpilot.selfdrive.turnpilot.desire_timing import WRONG_WAY_GRACE_M, WRONG_WAY_PAST_M
  w = 16.
  left = maneuver_from_instruction(nav_instruction(30., "turn", "left", junction=junction_of("left"), wrong_way=w))
  deadline = w - WRONG_WAY_PAST_M - WRONG_WAY_GRACE_M
  skip = TurnSkip()
  assert not skip.update(left, deadline + 1., 1, 4., False).skipped
  assert skip.update(left, deadline - 1., 1, 4., False).skipped
  bend = maneuver_from_instruction(nav_instruction(0., "turn", "right", junction=junction_of("right", 0, 60.)))
  assert not TurnSkip().update(bend, 0., 1, 4., False, ahead=left, ahead_distance=deadline + 1.).ahead_skipped
  assert TurnSkip().update(bend, 0., 1, 4., False, ahead=left, ahead_distance=deadline - 1.).ahead_skipped


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
  # inside the lane planner's last chance (lane_change_window's end at the car's speed, slowing from 13 m/s)
  from openpilot.selfdrive.turnpilot.lane_planner import LC_END_MIN_M, LC_END_S
  assert flagged and 300. - LC_END_S * 13. - 1. < flagged[0] < 300. - LC_END_MIN_M
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
  assert TurnSkip().update(left, MIN_FIRST_PULSE_M - 1., 1, 7., False).skipped


@pytest.mark.parametrize("stop_infeasible", [False, True], ids=["tooFast", "unprotectedStop"])
def test_too_fast_latches_only_while_openpilot_steers_under_always_drive(stop_infeasible):
  """ALWAYS_DRIVE.md R4b (S3), under TurnPilotAlwaysDrive: tooFast (and its unprotectedStop, on the same count) latches only
  while openpilot steers, as tooLate already does. The gate's tooFast skips were 14 of 15 at the spawn, before engagement:
  a car driven by hand inside a turn's stopping distance had the turn skipped for good under it. Disengaged, the check still
  withholds the pulse (feasible_now false); once openpilot steers it runs on the live speed and skips as before."""
  m = maneuver_from_instruction(nav_instruction(30., "turn", "left", junction=junction_of("left")))

  def frames(skip, n, v, lat_active, mid=1):
    return [skip.update(m, 30., mid, v, False, lat_active=lat_active, stop_infeasible=stop_infeasible) for _ in range(n)]

  # switch on, openpilot not steering: never skipped however long, and no pulse may start
  skip = TurnSkip()
  skip.always_drive = True
  states = frames(skip, 5 * SKIP_CONFIRM_FRAMES, 15., lat_active=False)
  assert not any(s.skipped for s in states) and not any(s.feasible_now for s in states) and not skip.skipped
  # engaged at a speed it can make the turn from: feasible, nothing skipped (the hand-driven frames left no count behind)
  if not stop_infeasible:
    assert all(s.feasible_now and not s.skipped for s in frames(skip, SKIP_CONFIRM_FRAMES, 4., lat_active=True))
  # engaged still too fast: the confirming frames from the engagement on, then skipped as before
  states = frames(skip, SKIP_CONFIRM_FRAMES, 15., lat_active=True)
  assert not any(s.skipped for s in states[:-1]) and states[-1].skipped
  assert skip.reason == ('unprotectedStop' if stop_infeasible else 'tooFast')
  # switch off: today's latch, openpilot steering or not
  skip = TurnSkip()
  states = frames(skip, SKIP_CONFIRM_FRAMES, 15., lat_active=False)
  assert states[-1].skipped and skip.reason == ('unprotectedStop' if stop_infeasible else 'tooFast')
  # and the switch on with openpilot steering skips exactly as it does off
  on, off = TurnSkip(), TurnSkip()
  on.always_drive = True
  assert [s.skipped for s in frames(on, 2 * SKIP_CONFIRM_FRAMES, 15., True)] == \
    [s.skipped for s in frames(off, 2 * SKIP_CONFIRM_FRAMES, 15., True)]


def test_a_turn_engaged_for_inside_its_stopping_distance_is_driven_under_always_drive():
  """R4b closed loop: the driver drives a car at 15 m/s toward a left 60 m ahead, too fast to make it from there, openpilot
  not steering, and brakes by hand to 5 m/s, engaging about 30 m out. Switch on, the turn isn't skipped under the driver,
  and once openpilot steers it is driven (its desire goes); switch off, it was skipped for good before the driver braked."""
  for always_drive, want_skip in ((False, True), (True, False)):
    sim = Sim([left_at(60.), left_at(900.)], cruise=15.)
    sim.tp.always_drive = always_drive
    sim.v = 15.
    sim.accel = lambda sim, plan: -3.5 if sim.v > 5. else 0.  # the driver's brake, the plan not applied
    plans = []
    while sim.v > 5.:  # by hand: openpilot not steering
      plans.append(sim.step(lat_active=False))
    assert 25. < 60. - sim.s < 40.
    assert any(p.tooFastToTurn for p in plans) == want_skip, always_drive
    assert not any(p.desire == Desire.turnLeft for p in plans)
    sim.accel = None
    while sim.s < 55.:
      plans.append(sim.step(engaged=True))
    assert any(p.tooFastToTurn for p in plans) == want_skip, always_drive
    assert any(p.desire == Desire.turnLeft for p in plans) == (not want_skip), always_drive


RIGHT_M = maneuver_from_instruction(nav_instruction(40., "turn", "right", junction=junction_of("right")))
LEFT_M = maneuver_from_instruction(nav_instruction(40., "turn", "left", junction=junction_of("left")))
END_LEFT_M = maneuver_from_instruction(nav_instruction(40., "end of road", "left", junction=junction_of("left", opposing='none')))


def test_engaged_standing_short_of_a_turn_is_never_too_late():
  """1d 4:54.0 (engaged standing 9.9 m from a right, v 0.0) and 1e 2:07.7 (12.3 m from an end-of-road left, v 0.39): both
  were skipped tooLate at once, 'Too Fast to Turn' at 0 mph, and both drives ended in a disengagement (gas at 2:14.9, brake
  and steer at 4:57.1). A car under STANDING_PULSE_V STANDING_PULSE_MIN_M or more short of its junction is never too late:
  it pulses from where it is, a turn from the line as a hold's go is."""
  for m, d, v in ((RIGHT_M, 9.9, 0.), (END_LEFT_M, 12.3, 0.39), (LEFT_M, 20., 1.4), (RIGHT_M, STANDING_PULSE_MIN_M, 0.), (LEFT_M, 24., 0.)):
    state = TurnSkip().update(m, d, 1, v, False)
    assert not state.skipped and state.feasible_now, (m.side, d, v)
  # moving, inside its deadline: still too late (a far-side turn 20 m out at turning speed, 9 m out at 1.6 m/s; a curb-side
  # one 4 m out). The plan's "1.6 m/s at 20 m" was written against the flat 40 m lead: a far-side turn at 1.6 m/s now pulses
  # 15 m out (desire_timing.turn_lead) and a curb-side one 8 m out, so 20 m isn't late for either
  for m, d, v in ((LEFT_M, 20., 7.), (LEFT_M, 9., STANDING_PULSE_V + 0.1), (RIGHT_M, 4., STANDING_PULSE_V + 0.1), (RIGHT_M, 4., 3.)):
    assert TurnSkip().update(m, d, 1, v, False).skipped, (m.side, d, v)
  assert not TurnSkip().update(LEFT_M, 20., 1, STANDING_PULSE_V + 0.1, False).skipped
  # standing or crawling, but under STANDING_PULSE_MIN_M: still too late (a first pulse there turns the car into the curb)
  for m in (LEFT_M, RIGHT_M, END_LEFT_M):
    assert TurnSkip().update(m, 3., 1, 0.3, False).skipped, m.side


@pytest.mark.parametrize("v", [0., 3., 7.])
@pytest.mark.parametrize("distance", [0., -4.8, -12.1])
def test_never_too_late_at_or_past_the_junction(distance, v):
  """1e 8:36.47: tooLate at -12.1 m for a right finished 4 s before (a one-message navd blip made it a new maneuver behind
  the car), and 1e 9:43.5 at -4.8 m (a stale reroute's first step): both said 'Too Fast to Turn / Rerouting' for a turn the
  car had made. No first pulse goes at or past a junction (desire_timing.py), so tooLate never fires there; nor the announced
  skip of one ahead of a bend."""
  for m in (LEFT_M, RIGHT_M):
    skip = TurnSkip()
    skip.update(m, distance, 1, v, False)
    assert skip.reason != 'tooLate', (m.side, distance, v)
  bend = maneuver_from_instruction(nav_instruction(0., "turn", "right", junction=junction_of("right", 0, 60.)))
  assert not TurnSkip().update(bend, 0., 1, v, False, ahead=RIGHT_M, ahead_distance=distance).ahead_skipped


def test_the_first_pulse_deadline_is_always_short_of_its_lead():
  """turn_skip's tooLate deadline follows the pulse's own timing (desire_timing.turn_lead, 2026-11-06), never ahead of it at
  any speed: a far-side turn's lead is FAR_LEAD_S out (15-40 m) and its deadline FIRST_PULSE_S out (10-25 m); a curb-side
  turn's one pulse goes 8 m out and its deadline is CURB_FIRST_PULSE_M. Mirrored where traffic keeps left."""
  from openpilot.selfdrive.turnpilot.desire_timing import CURB_TURN_PULSE_M, turn_lead
  for rht in (True, False):
    far, curb = (LEFT_M, RIGHT_M) if rht else (RIGHT_M, LEFT_M)
    for k in range(300):
      v = k * 0.1
      for m in (far, curb):
        assert first_pulse_latest(m, v, rht) <= turn_lead(m, v, rht) - 3., (rht, m.side, v)
      assert first_pulse_latest(curb, v, rht) == CURB_FIRST_PULSE_M < CURB_TURN_PULSE_M
      assert FAR_FIRST_PULSE_MIN_M <= first_pulse_latest(far, v, rht) <= MIN_FIRST_PULSE_M
    assert first_pulse_latest(far, 7., rht) == MIN_FIRST_PULSE_M  # at turning speed, the 25 m it was


def start_standing(maneuvers, stand_s=3., pull_away=4., seconds=10.):
  """Sim: engaged standing at s 0 (the model holding the car) for stand_s, then pulling away toward pull_away m/s. The
  plans."""
  sim = Sim(maneuvers, cruise=pull_away)
  sim.v = 0.
  sim.accel = lambda sim, plan: 0.
  plans = [sim.step(engaged=True) for _ in range(int(stand_s / DT_MDL))]
  sim.accel = None
  turning = maneuvers[0][0]
  while sim.t < seconds:
    yaw = -sim.v / 8. if maneuvers[0][2] == 'right' and abs(turning - sim.s) < 6. else \
      (sim.v / 10. if maneuvers[0][2] == 'left' and abs(turning - sim.s) < 8. else 0.)
    plans.append(sim.step(engaged=True, yaw_rate=yaw))
  return sim, plans


def test_engaged_at_a_standstill_short_of_a_turn_it_is_driven_from_where_the_car_stands():
  """Closed loop, as 1d 4:54 and 1e 2:07 were engaged: a right 12 m out standing is taken, its one pulse 8 m out once the car
  pulls away (desire_timing.CURB_TURN_PULSE_M); the end-of-road left 12.3 m out pulses at once, standing, inside its lead's
  15 m floor. Neither is skipped, nor said 'Too Fast to Turn'."""
  from openpilot.selfdrive.turnpilot.desire_timing import CURB_TURN_PULSE_M
  sim, plans = start_standing([(12., "turn", "right", junction_of("right")), (900., "turn", "left", junction_of("left"))])
  assert not any(p.tooFastToTurn for p in plans)
  edges = [(s, v) for (s, v, p), (_, _, q) in zip(sim.trace[1:], sim.trace, strict=False) if p.desire == Desire.turnRight and q.desire != Desire.turnRight]
  assert len(edges) == 1 and CURB_TURN_PULSE_M - 0.5 < 12. - edges[0][0] < CURB_TURN_PULSE_M + 0.5, edges
  sim, plans = start_standing([(12.3, "end of road", "left", junction_of("left", opposing='none')),
                               (900., "turn", "right", junction_of("right"))])
  assert not any(p.tooFastToTurn for p in plans)
  first = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert first < 5, first  # at once, standing


def test_a_curb_side_turn_is_judged_by_the_reading_its_pulse_goes_by():
  """A curb-side turn's one pulse goes by the furthest reading of its junction (desire_timing.pulse_distance), so turn_skip
  judges it by the same reading (update's far): a reading 20 m short at 28 m out at the cap's 5.4 m/s is no tooFast skip
  (test_turnpilotd's GPS bias case: the pulse waits for the true 8 m, the cap brakes for the nearest meanwhile), nor is a
  4 m nearest with the furthest 12 m out too late; once the furthest is inside its deadline it is, and past the junction by
  the nearest it never is. Without a furthest, the nearest as before. A far-side turn is judged by the nearest."""
  for _ in range(SKIP_CONFIRM_FRAMES + 1):
    skip = TurnSkip()
    state = skip.update(RIGHT_M, 8.4, 1, 5.42, False, far=28.2)
  assert not state.skipped and state.feasible_now
  skip = TurnSkip()
  states = [skip.update(RIGHT_M, 8.4, 1, 5.42, False) for _ in range(SKIP_CONFIRM_FRAMES)]
  assert states[-1].skipped and skip.reason == 'tooFast'  # by the nearest alone
  assert not TurnSkip().update(RIGHT_M, 4., 1, 3., False, far=12.).skipped
  assert TurnSkip().update(RIGHT_M, 3., 1, 3., False, far=4.5).skipped
  assert TurnSkip().update(RIGHT_M, 4., 1, 3., False).skipped  # no far: the nearest
  skip = TurnSkip()
  skip.update(RIGHT_M, -1., 1, 3., False, far=-0.5)
  assert skip.reason != 'tooLate'  # first seen past the junction by every reading: never
  skip = TurnSkip()
  skip.update(RIGHT_M, -1., 1, 3., False, far=4.)
  assert skip.reason == 'tooLate'  # ...but split across it, no pulse can go by the nearest: said at once (2026-11-06)
  # standing: by the furthest too (5 m or more short of it, never too late)
  assert not TurnSkip().update(RIGHT_M, 3., 1, 0.2, False, far=6.).skipped
  # a far-side turn: the nearest, whatever the furthest
  assert TurnSkip().update(LEFT_M, 20., 1, 7., False, far=40.).skipped


def test_a_turn_whose_nearest_crosses_its_junction_with_no_first_pulse_is_skipped_out_loud():
  """No first pulse goes past a junction by the nearest (desire_timing.py: a late pulse is the dangerous one), so a turn that
  gets there unpulsed is lost: 1e 8:32 (a 91 deg right, one follower over the whole drive) had its furthest reading still
  8.8 m out with the nearest at the junction, and its pulse by the furthest never went, with no skip, alert or prompt. A turn
  whose nearest crosses the junction with openpilot steering and no first pulse out is skipped tooLate there and then (navd
  reroutes, the driver hears it). Only on the crossing: a turn first seen past its junction (1e 8:36's phantom, 9:43's stale
  reroute) or first steered for there (a re-engage after the driver's own turn) is not, nor one whose desire went out, nor
  one whose first pulse is its hold line's go (pulse_at_go)."""
  for kw in ({'far': 9.}, {}, {'far': 9., 'last': 7.}):
    skip = TurnSkip()
    assert not skip.update(RIGHT_M, 20., 1, 3., False, **kw).skipped  # approached from outside its deadline
    assert not skip.update(RIGHT_M, 0.3, 1, 3., False, **kw).skipped
    state = skip.update(RIGHT_M, -0.1, 1, 3., False, **kw)
    assert state.skipped and not state.feasible_now and skip.reason == 'tooLate', kw
    assert skip.update(RIGHT_M, -0.3, 1, 3., False, **kw).skipped  # latched
  # the desire out: never
  skip = TurnSkip()
  skip.update(RIGHT_M, 0.3, 1, 3., True, far=9.)
  assert not skip.update(RIGHT_M, -0.1, 1, 3., True, far=9.).skipped
  # openpilot not steering on the update before (the driver's turn, engaged in the junction): never
  skip = TurnSkip()
  skip.update(RIGHT_M, 0.3, 1, 3., False, lat_active=False, far=9.)
  assert not skip.update(RIGHT_M, -0.1, 1, 3., False, far=9.).skipped
  # nor once openpilot is off
  skip = TurnSkip()
  skip.update(RIGHT_M, 0.3, 1, 3., False, far=9.)
  assert not skip.update(RIGHT_M, -0.1, 1, 3., False, lat_active=False, far=9.).skipped
  # nor a turn whose first pulse is its hold line's go
  skip = TurnSkip()
  skip.update(LEFT_M, 0.3, 1, 0., False, pulse_at_go=True)
  assert not skip.update(LEFT_M, -0.1, 1, 0., False, pulse_at_go=True).skipped
  # a new maneuver past its junction by every reading from its first: never (test_never_too_late_at_or_past_the_junction)
  skip = TurnSkip()
  skip.update(RIGHT_M, 0.3, 1, 3., False, far=9.)
  assert not skip.update(RIGHT_M, -0.1, 2, 3., False, far=-0.1).skipped
  # ...but first seen with its readings split across the junction (a reroute's same junction carrying a short outlier as its
  # nearest: test_route_follower), the furthest short of it: no pulse can go by the nearest, so it is said at once
  skip = TurnSkip()
  skip.update(RIGHT_M, 0.3, 1, 3., False, far=9.)
  state = skip.update(RIGHT_M, -0.1, 2, 3., False, far=9.)
  assert state.skipped and skip.reason == 'tooLate'
  skip = TurnSkip()  # only on its first update: a later one past the junction with no crossing seen (openpilot off before) is not
  skip.update(RIGHT_M, -0.1, 3, 3., False, lat_active=False, far=9.)
  assert not skip.update(RIGHT_M, -0.2, 3, 3., False, far=9.).skipped
  # standing: a new reading puts the nearest past the junction under a car stopped short of it (a queue, a red): skipped too
  skip = TurnSkip()
  skip.update(RIGHT_M, 6., 1, 0., False, far=9.)
  assert skip.update(RIGHT_M, -1., 1, 0., False, far=9.).skipped and skip.reason == 'tooLate'
  # a far-side turn too: one waiting past a wrong-way opening 4 m short of its junction has no deadline short of it
  # (desire_timing.first_pulse_by: 4 - 2 - 3 m, at least 0)
  left = maneuver_from_instruction(nav_instruction(30., "turn", "left", junction=junction_of("left"), wrong_way=4.))
  skip = TurnSkip()
  assert not skip.update(left, 0.4, 1, 3., False).skipped
  assert skip.update(left, -0.1, 1, 3., False).skipped and skip.reason == 'tooLate'


def test_without_the_reading_its_pulse_goes_by_a_curb_side_turn_approached_under_openpilot_is_its_pulses_to_time():
  """turnpilotd hands TurnSkip neither the furthest nor the latest reading (DesireTiming has the furthest): judged by the
  nearest's 5 m deadline, a right whose readings spread 3 m or more was skipped tooLate before its pulse by the furthest was
  due (closed loop, the probe's +4/+6/+9 m long and -4/-6 m short: all skipped; 3 of the 8 logged rights). So without them
  the nearest's deadline holds for a curb-side turn only until openpilot has steered for it, without a break, from outside it:
  a turn that comes up inside it (navd past a bend 10 m before it) or that openpilot engages for there is still skipped; one
  approached from outside it is its pulse's to time, and its nearest crossing the junction unpulsed is skipped out loud.
  Given the furthest, the deadline is by the pulse's reading on every update, as before."""
  def approach(skip, to_m, kw=None, lat_from=None):
    for k in range(60):
      d = 20. - k * (20. - to_m) / 59.
      state = skip.update(RIGHT_M, d, 1, 3., False, lat_active=lat_from is None or d <= lat_from, **(kw or {}))
    return state
  assert not approach(TurnSkip(), 4.).skipped
  assert not approach(TurnSkip(), 0.).skipped  # at the junction itself: the crossing's
  # given the furthest: the deadline by it (5 m) on every update
  skip = TurnSkip()
  state = approach(skip, 4., {'far': 4.5})
  assert state.skipped and skip.reason == 'tooLate'
  # first seen inside the deadline, or first steered for there (engaged late, or again after a break): skipped
  assert TurnSkip().update(RIGHT_M, 4., 1, 3., False).skipped
  skip = TurnSkip()
  assert approach(skip, 4., lat_from=4.).skipped and skip.reason == 'tooLate'
  skip = TurnSkip()
  approach(skip, 10.)
  skip.update(RIGHT_M, 9., 1, 3., False, lat_active=False)
  assert skip.update(RIGHT_M, 4., 1, 3., False).skipped
  # a far-side turn: its deadline by the nearest as ever (25 m at turning speed)
  skip = TurnSkip()
  for d in (40., 30., 26.):
    assert not skip.update(LEFT_M, d, 1, 7., False).skipped
  assert skip.update(LEFT_M, 24., 1, 7., False).skipped


def test_a_curb_side_turn_is_too_late_by_the_latest_reading_short_of_where_its_pulse_is_due_by_it():
  """The curb-side pulse is due by the furthest at CURB_TURN_PULSE_M or by navd's latest reading at CURB_LATEST_PULSE_M
  (desire_timing.pulse_distance); its tooLate deadline (CURB_FIRST_PULSE_M, 5 m, of that same reading) is 3 m by the latest,
  always after the latest's pulse point, so a pulse it calls due is never one it calls too late."""
  from openpilot.selfdrive.turnpilot.desire_timing import CURB_LATEST_PULSE_M, CURB_TURN_PULSE_M, pulse_distance
  latest_deadline = CURB_FIRST_PULSE_M - (CURB_TURN_PULSE_M - CURB_LATEST_PULSE_M)
  assert 0. < latest_deadline < CURB_LATEST_PULSE_M
  assert TurnSkip().update(RIGHT_M, 2., 1, 3., False, far=12., last=latest_deadline - 0.5).skipped
  assert not TurnSkip().update(RIGHT_M, 2., 1, 3., False, far=12., last=latest_deadline + 0.5).skipped
  assert not TurnSkip().update(RIGHT_M, 2., 1, 3., False, far=12.).skipped  # by the furthest alone: 12 m, not late
  for far in (3., 6., 8., 12., 30.):
    for last_ in (2., 3.5, 5., 6., 9.):
      d = pulse_distance(RIGHT_M, min(2., last_), far, last=last_)
      due = d < CURB_TURN_PULSE_M
      late = d < CURB_FIRST_PULSE_M
      assert due or not late, (far, last_)
  # a far-side turn: the latest no matter
  assert pulse_distance(LEFT_M, 20., 40., last=3.) == 20.


def test_a_curb_side_turn_is_short_of_its_junction_or_past_it_by_its_nearest_reading_alone():
  """2026-11-06 review: a curb-side turn's crossing, its split and its deadline's own bound at the junction go by the nearest
  reading, one reading alone included, as its first pulse does (desire_timing.py): with the furthest and the latest 9 m
  ahead (one short outlier the nearest keeps, or one TRUE reading inside a lasting long bias: the same readings), the nearest
  crossing the junction unpulsed is skipped tooLate out loud there. Judged by the second-nearest that round, the bias's right
  was pulsed 0.3-2.3 m past its junction, nothing said (route_follower module docstring)."""
  from openpilot.selfdrive.turnpilot.turn_skip import too_late
  # the nearest crosses with the furthest and the latest 9 m ahead: skipped
  skip = TurnSkip()
  assert not skip.update(RIGHT_M, 0.3, 1, 3., False, far=9.3, last=9.3).skipped
  state = skip.update(RIGHT_M, -0.1, 1, 3., False, far=8.9, last=8.9)
  assert state.skipped and skip.reason == 'tooLate'
  # first seen split across the junction by the nearest alone: skipped at once
  skip = TurnSkip()
  skip.update(RIGHT_M, 0.3, 1, 3., False, far=9.)
  assert skip.update(RIGHT_M, -0.1, 2, 3., False, far=9.).skipped and skip.reason == 'tooLate'
  # its deadline (CURB_FIRST_PULSE_M by the reading its pulse goes by) holds short of the junction, never past it
  skip = TurnSkip()
  skip.update(RIGHT_M, 20., 1, 3., False, far=29.)
  assert skip.update(RIGHT_M, 4.6, 1, 3., False, far=CURB_FIRST_PULSE_M - 0.5, last=CURB_FIRST_PULSE_M - 0.5).skipped
  assert skip.reason == 'tooLate'
  assert not too_late(RIGHT_M, -0.1, 3., far=CURB_FIRST_PULSE_M - 0.5, last=CURB_FIRST_PULSE_M - 0.5)
  # a far-side turn: the nearest's crossing as ever
  skip = TurnSkip()
  skip.update(LEFT_M, 0.3, 1, 3., False, far=9.)
  assert skip.update(LEFT_M, -0.1, 1, 3., False, far=9.).skipped

def test_a_crossing_is_seen_only_on_consecutive_updates_of_the_turn_itself():
  """The stop-sign ladder's other turn stands in for the route's turn (turnpilotd hands TurnSkip None meanwhile: alt_replaced);
  the route's turn coming back past its junction after it is no crossing seen with no pulse out, so no 'Too Fast to Turn' for
  a turn the other one replaced (stop_go's turnMissed says that one)."""
  skip = TurnSkip()
  assert not skip.update(RIGHT_M, 20., 1, 3., False, far=29.).skipped
  assert not skip.update(RIGHT_M, 0.4, 1, 3., False, far=9.4).skipped  # the furthest 9 m long: no pulse yet, nor too late
  skip.update(None, 0.2, 1, 3., False)
  assert not skip.update(RIGHT_M, -0.2, 1, 3., False, far=8.8).skipped
  # the same updates with the turn's own in between: the crossing, skipped
  skip = TurnSkip()
  skip.update(RIGHT_M, 20., 1, 3., False, far=29.)
  skip.update(RIGHT_M, 0.4, 1, 3., False, far=9.4)
  assert skip.update(RIGHT_M, -0.2, 1, 3., False, far=8.8).skipped


@pytest.mark.parametrize("pulse_at_go", [False, True], ids=["route", "decider"])
@pytest.mark.parametrize("follow", [0., 0.05])
def test_a_curb_side_pulse_held_while_the_model_steers_in_is_skipped_out_loud_at_its_deadline(pulse_at_go, follow):
  """2026-11-06: a curb-side turn's one pulse waits while the model steers into the turn at a turn's scale with the car rolling
  (desire_timing.py: STEER_IN_TURN_CURV; a fresh edge then restarts the model's turn, BeamNG right_green). Still waiting at
  CURB_FIRST_PULSE_M, the car not following the model round (under OWN_FOLLOW_DEG since its steer-in: the model at 0.06 1/m
  from 9 m at the cap's 3 m/s, the car yawing with none or a twentieth of it), it is skipped tooLate there, out loud, and no
  pulse ever goes: never late into a model mid-steer, never at or past the junction, never lost in silence. The stop-sign
  decider's right (pulse_at_go, rolling from a line further out than its pulse point) too: its go spares it only while the car
  stands (test_a_curb_side_turns_go_spares_it_only_while_the_car_stands)."""
  from openpilot.selfdrive.turnpilot.desire_timing import CURB_TURN_PULSE_M, OWN_FOLLOW_DEG, STEER_IN_TURN_CURV
  from openpilot.selfdrive.turnpilot.tests.test_desire_timing import curb_turn
  trace, dt_, skip = curb_turn(lambda d: 0.06 if d < 9. else 0., pulse_at_go=pulse_at_go, follow=follow)
  at = next(i for i, (*_, sk) in enumerate(trace) if sk)
  d, _, _, deg, _ = trace[at]
  assert CURB_FIRST_PULSE_M - 3. * DT_MDL - 1e-6 <= d < CURB_FIRST_PULSE_M and skip.reason == 'tooLate', trace[at]
  assert trace[at - 1][3] < OWN_FOLLOW_DEG and (follow == 0. or trace[at - 1][3] > 0.5), trace[at - 1]
  assert all(desire != Desire.turnRight for _, desire, *_ in trace) and not dt_.started
  assert all(not sk for dd, *_, sk in trace if dd >= CURB_FIRST_PULSE_M)  # nothing said before its deadline...
  assert any(c >= STEER_IN_TURN_CURV for dd, _, c, *_ in trace if dd < CURB_TURN_PULSE_M)  # ...the pulse waiting from 8 m


def brake_to(line, v0=3., a=2.):
  """A speed profile v(d, v) braking at a to a stand at a line m before the junction, from v0."""
  def f(d, _v):
    return min(v0, math.sqrt(2. * a * max(d - line, 0.)))
  return f


def to_a_line(line, curv=0., follow=0., hold=True, d0=30., seconds=20., go_after_s=2.):
  """TurnSkip then DesireTiming each frame as turnpilotd runs them, a right d0 m ahead braking to a map stop at line m before
  its junction (stop_d: MapControlPolicy.stop_ahead, the line's distance while the car is to stop there, 0 held at it), the
  model steering in at curv from 10 m (the car yawing follow * v * curv); hold: the stop-sign decider's turn (its pulse held to
  its go, pulse_at_go), else a red (the pulse not held). go_after_s standing at the line, the stop is released (stop_d None)
  and the hold's go comes. Returns ([(d, v, desire, skipped)], TurnSkip, DesireTiming)."""
  from openpilot.selfdrive.turnpilot.route_follower import Maneuver, classify
  m = Maneuver(*classify("turn", "right"), "turn", "right", 90.)
  dt_, skip, d, v, out, stood, released, gone = DesireTiming(), TurnSkip(), d0, 3., [], 0., False, False
  speed = brake_to(line)
  for _ in range(int(seconds / DT_MDL)):
    v = 3. if gone else speed(d, v)
    stood = stood + DT_MDL if v < 0.05 else stood
    released = released or stood > go_after_s
    go, gone = released and not gone, gone or released  # the hold's go: the one frame it lets go, standing
    c = curv if d < 10. else 0.
    stop_d = None if released else max(d - line, 0.)
    st = skip.update(m, d, 1, v, dt_.started, stop_d=stop_d, pulse_at_go=hold and not dt_.done, far=d, last=d)
    des = dt_.update(None if st.skipped else m, d, 1, False, v, 0., True, True, DT_MDL, -follow * v * c,
                     may_start=st.feasible_now and not (hold and not released), go=go, holding=not released and v < 0.05,
                     far=d, last=d, curvature=c)
    out.append((round(d, 2), round(v, 2), des, st.skipped, go))
    d -= v * DT_MDL
  return [o[:4] for o in out], skip, dt_


@pytest.mark.parametrize("line", [3., 4., 4.5, 6.])
@pytest.mark.parametrize("hold, curv", [(True, 0.), (False, 0.), (False, 0.06)], ids=["decider", "red", "red_steer_in"])
def test_a_right_braking_to_a_stop_line_inside_its_deadline_is_never_skipped_short_of_it(line, hold, curv):
  """The 2026-11-06 review: TurnSkip judged a right by its 5 m deadline rolling whatever stood between the car and the junction,
  so a right braking to a stop line 3-4 m from navd's node was skipped tooLate at about 4.9 m, rolling at 1.9-2.8 m/s, short
  of the line it was stopping at: the stop-sign decider's right (its pulse held to the go, pulse_at_go: since 2026-11-06 a
  curb-side one rolling is judged by its deadline), and a right braking to a red with the model steering in at a turn's scale
  (its pulse waiting, the car not following it round). A map stop the car will stop at between it and the junction
  (stop_d, MapControlPolicy.stop_ahead) puts the deadline off: the turn pulses from the line (standing: whatever the model
  steers; the decider's go a pulse at once), never skipped short of it, never pulsed at or past the junction."""
  trace, skip, dt_ = to_a_line(line, curv=curv, hold=hold)
  assert not skip.skipped and not any(sk for *_, sk in trace), (skip.reason, next(o for o in trace if o[3]))
  first = next(i for i, (_, _, des, _) in enumerate(trace) if des == Desire.turnRight)
  d, v, _, _ = trace[first]
  from openpilot.selfdrive.turnpilot.desire_timing import STANDSTILL_SPEED
  assert 0. < d and (d >= CURB_FIRST_PULSE_M or v < STANDSTILL_SPEED), trace[first]  # rolling by its deadline, or standing
  assert all(des == Desire.none for _, _, des, _ in trace[:first]) and dt_.started
  # with no map stop between it and the junction (one past it, none, one behind the car) the deadline holds, as before
  if line < CURB_FIRST_PULSE_M:
    for stop_d in (None, 2., -1.):
      st = TurnSkip()
      st.update(RIGHT_M, 6., 1, 2.5, False, stop_d=stop_d, far=6., last=6.)
      assert st.update(RIGHT_M, 4.9, 1, 2.5, False, stop_d=None if stop_d is None else stop_d + 1.1 if stop_d > 0. else stop_d,
                       far=4.9, last=4.9).skipped == (stop_d is None or stop_d < 0. or stop_d + 1.1 > 4.9), stop_d


def test_a_curb_side_turns_go_spares_it_only_while_the_car_stands():
  """pulse_at_go (the stop-sign decider's turn, the autonomous left) made a turn never too late and never skipped on its
  crossing: its first pulse is its go. A curb-side turn's go is its pulse only from a line within the pulse point
  (desire_timing.CURB_TURN_PULSE_M + CURB_GO_SLACK_M); from one further out the pulse waits for the pulse point as the car
  rolls, and waits there while the model steers in, so with pulse_at_go blocking both rails such a right was lost in silence.
  Standing or crawling (under STANDING_PULSE_V) at a line inside its deadline it is spared, its go pulsing it at once; rolling,
  its deadline holds; its nearest crossing the junction unpulsed is skipped however slow. A far-side turn's go is its first
  pulse from wherever its line is: unchanged."""
  # standing at a line 4 m out (inside the 5 m deadline), waiting for the decider's go: spared
  skip = TurnSkip()
  for _ in range(10):
    assert not skip.update(RIGHT_M, 4., 1, 0., False, pulse_at_go=True, far=4., last=4.).skipped
  # rolling past its deadline with no pulse out: skipped tooLate
  skip = TurnSkip()
  assert not skip.update(RIGHT_M, 6., 1, 3., False, pulse_at_go=True, far=6., last=6.).skipped
  assert skip.update(RIGHT_M, 4.9, 1, 3., False, pulse_at_go=True, far=4.9, last=4.9).skipped and skip.reason == 'tooLate'
  # crawling (1 m/s) inside it: spared to the junction, then skipped on the crossing
  skip = TurnSkip()
  assert not skip.update(RIGHT_M, 3., 1, 1., False, pulse_at_go=True, far=3., last=3.).skipped
  assert not skip.update(RIGHT_M, 0.05, 1, 1., False, pulse_at_go=True, far=0.05, last=0.05).skipped
  assert skip.update(RIGHT_M, -0.01, 1, 1., False, pulse_at_go=True, far=-0.01, last=-0.01).skipped and skip.reason == 'tooLate'
  # its pulse out (the go's at the line, or the pulse point's): never
  skip = TurnSkip()
  for d in (4.9, 2., 0.05, -0.1):
    assert not skip.update(RIGHT_M, d, 1, 3., True, pulse_at_go=True, far=d, last=d).skipped
  # a far-side turn's go is its first pulse wherever its line is: never too late, never skipped on its crossing (as before)
  skip = TurnSkip()
  for d in (MIN_FIRST_PULSE_M - 1., 9., 0.3, -0.1):
    assert not skip.update(LEFT_M, d, 1, 3., False, pulse_at_go=True).skipped, d


def test_out_of_range_skip_is_gone_and_its_old_reason_still_read():
  """turnpilotd stopped passing out_of_range on 2026-11-06 (lefts to the line: never route around or skip a left for range),
  so its unprotectedRange branch was dead: removed. Logs before then can carry skipReason unprotectedRange; the module says
  what it meant."""
  import inspect
  import openpilot.selfdrive.turnpilot.turn_skip as turn_skip
  assert 'out_of_range' not in inspect.signature(TurnSkip.update).parameters
  with pytest.raises(TypeError):
    TurnSkip().update(LEFT_M, 100., 1, 10., False, out_of_range=True)  # pylint: disable=unexpected-keyword-arg
  assert 'unprotectedRange' in turn_skip.__doc__
  assert not TurnSkip().update(LEFT_M, 100., 1, 10., False).skipped


@pytest.mark.parametrize("curvature", [0.05, 0.07, 0.12])
@pytest.mark.parametrize("start", [12., 8.5])
def test_the_models_own_curb_side_turn_with_the_car_following_it_is_never_skipped(curvature, start):
  """Replayed over 1e with the logged model's curvature (a stress case: it ran under the old 40 m pulse and was turning), the
  two engaged rights whose pulse waited had the car 10.7 and 13.4 deg round at the deadline, skipped 'Too Fast to Turn' by a
  TURN_STARTED_DEG mark while the model drove them. The model steering in at a turn's scale with the car turning with it is
  its own turn: OWN_TURN_DEG round on its approach (from 12 m: the car's turn counted from the model's steer-in, not from the
  pulse point), or following it OWN_FOLLOW_DEG round once its pulse waits (from 8.5 m: at 0.05, a 20 m radius, 8.2 deg by the
  deadline counted from 8 m, skipped there before 2026-11-06's review): never skipped, never pulsed."""
  from openpilot.selfdrive.turnpilot.tests.test_desire_timing import curb_turn
  trace, dt_, skip = curb_turn(lambda d: curvature if d < start else 0.)
  assert all(desire != Desire.turnRight for _, desire, *_ in trace)
  assert dt_.own and not skip.skipped and not any(sk for *_, sk in trace)
  assert dt_.own_at > CURB_FIRST_PULSE_M
