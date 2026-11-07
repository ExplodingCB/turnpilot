import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.desire_timing import (CLEAR_FRAMES, CLEAR_MARGIN_DEG, CLEAR_MIN_DEG, CLEAR_WAIT_S, CURB_GO_SLACK_M,
                                                         CURB_LATEST_PULSE_M, CURB_TURN_PULSE_M,
                                                         ENTRY_GAP_S, ENTRY_M, FAR_LEAD_MIN_M, PULSE_GAP_FRAMES, REPULSE_S,
                                                         STANDSTILL_REARM_S, STEER_IN_CURV, STEER_IN_TURN_CURV, TURN_DESIRE_LEAD_M,
                                                         TURN_STARTED_DEG, DesireTiming, keep_lead, pulse_distance, turn_lead)
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE
from openpilot.selfdrive.turnpilot.route_follower import RouteFollower, maneuver_from_instruction
from openpilot.selfdrive.turnpilot.turn_skip import TurnSkip
from openpilot.selfdrive.turnpilot.tests.synthetic import FakeNavd, nav_instruction

Desire = log.Desire
PASSED_M = 15.  # route_follower.PASSED_DIST_M


def drive(maneuvers, v=8., seconds=40., lines=(True, True), turns_at=8., speed_profile=None, noise=None, steer_sign=1.):
  """Drive along a route; the car steers into a turn once it is turns_at m from it (None: never turns).
  noise(i) m is added to the position navd sees on frame i. Returns [(s, distance, desire)] per frame."""
  navd, rf, dt_, skip = FakeNavd(maneuvers), RouteFollower(), DesireTiming(), TurnSkip()
  s, out = 0., []
  for i in range(int(seconds / DT_MDL)):
    v_now = speed_profile(i * DT_MDL) if speed_profile else v
    instr = navd.update(s + (noise(i) if noise else 0.))
    rf.update(instr, instr is not None, True, v_now, DT_MDL)
    steer, yaw = 0., 0.
    if turns_at is not None and rf.upcoming is not None and rf.distance < turns_at:
      steer, yaw = 90. * steer_sign, v_now / 8. * steer_sign  # an 8 m radius
    # as turnpilotd: a turn the car is too fast for is skipped, and no pulse starts while it can't make it
    state = skip.update(rf.upcoming, rf.distance, rf.maneuver_id, v_now, dt_.maneuver_id == rf.maneuver_id and dt_.started)
    desire = dt_.update(None if state.skipped else rf.upcoming, rf.distance, rf.maneuver_id, rf.passed, v_now, steer, *lines,
                        DT_MDL, yaw, may_start=state.feasible_now)
    out.append((s, rf.distance, desire))
    s += v_now * DT_MDL
  return out


def under_the_cap(route, cruise=8.):
  """A speed profile for drive(): the car following the turn speed cap of the route's next turn exactly (no lag), from
  cruise; route as drive()'s maneuvers. It integrates its own position, one call a frame."""
  from openpilot.selfdrive.turnpilot.route_follower import Maneuver, classify
  from openpilot.selfdrive.turnpilot.turn_speed import turn_speed_cap
  turns = [(s, Maneuver(*classify(t, m), t, m, 90.)) for s, t, m, *_ in route]
  state = {'s': 0., 'v': cruise}

  def profile(t):
    s = state['s']
    ahead = [(at, m) for at, m in turns if at - s > -10.]
    v = cruise if not ahead else min(cruise, turn_speed_cap(ahead[0][1], ahead[0][0] - s, state['v']))
    state['v'], state['s'] = v, s + v * DT_MDL
    return v
  return profile


def rising_edges(trace, desire):
  prev, edges = Desire.none, []
  for s, d, cur in trace:
    if cur == desire and prev != desire:
      edges.append((s, d))
    prev = cur
  return edges


def test_left_turn_desire_from_the_lead():
  # under the turn cap (6.9 m/s by 40 m, then down to 4.3 by 15 m) a far-side turn's lead is the full 40 m: FAR_LEAD_S at
  # 6.9 m/s is past it (desire_timing.turn_lead)
  route = [(200., "turn", "left"), (900., "turn", "right")]
  trace = drive(route, speed_profile=under_the_cap(route))
  lead = TURN_DESIRE_LEAD_M
  edges = rising_edges(trace, Desire.turnLeft)
  assert len(edges) == 3 and 0. < edges[-1][1] < ENTRY_M  # the pulse at the lead, a repeat, and one for the turn itself
  s, d = edges[0]
  assert lead - 7. * DT_MDL - 1. < d < lead  # within a frame of the lead, dead-reckoned
  assert all(desire == Desire.none for s, d, desire in trace if s < 200. - lead - 1.)
  # held into the junction (the car turns 8 m out, at 64 deg/s), cleared once it has turned 80 degrees
  held = [desire for s, d, desire in trace if 200. - lead + 1. < s < 202.]
  assert held.count(Desire.turnLeft) == len(held) - 2 * PULSE_GAP_FRAMES  # but for the drops before the repeat pulses
  assert [desire for s, d, desire in trace if 202. < s < 210.].count(CLEAR_DESIRE) == CLEAR_FRAMES
  assert all(desire != Desire.turnLeft for s, d, desire in trace if s > 230.)
  assert not rising_edges(trace, Desire.turnRight)  # the next turn, 700 m on, isn't reached


def test_navd_distance_noise_adds_no_rising_edges():
  # navd's 1 Hz distance jumps by 3 m either way around the lead: the desire latches on the first crossing
  for maneuver, desire in (("left", Desire.turnLeft), ("slight right", Desire.keepRight)):
    route = [(200., "turn" if maneuver == "left" else "fork", maneuver), (900., "turn", "right")]
    noisy = rising_edges(drive(route, v=4., seconds=60., noise=lambda i: 3. if (i // 20) % 2 else -3.), desire)
    assert noisy, maneuver
    # repeats only on the REPULSE_S clock, never from the noise
    assert all((s1 - s0) / 4. > REPULSE_S for (s0, _), (s1, _) in zip(noisy[:-1], noisy[1:], strict=True)), maneuver
    assert len(noisy) == 1 or desire != Desire.keepRight


def test_consecutive_turns_each_get_a_desire():
  route = [(200., "turn", "left"), (320., "turn", "right"), (2000., "turn", "left")]
  trace = drive(route, seconds=70., speed_profile=under_the_cap(route))
  left, right = rising_edges(trace, Desire.turnLeft), rising_edges(trace, Desire.turnRight)
  assert 2 <= len(left) <= 3  # the left's 40 m lead under its cap is more than REPULSE_S: repeats until turned
  assert len(right) == 1 and 320. - CURB_TURN_PULSE_M - 1. < right[0][0] < 320. - CURB_TURN_PULSE_M + 0.5  # the right's one, at 8 m
  assert all(s < 325. for s, _ in right) and all(s < 205. for s, _ in left)  # none once the car has turned


def test_slight_right_exit_sends_keep_right():
  v = 25.
  trace = drive([(1500., "off ramp", "slight right"), (3000., "turn", "left")], v=v, seconds=70.)
  edges = rising_edges(trace, Desire.keepRight)
  assert len(edges) == 1
  assert keep_lead(v) - v * 1.1 < edges[0][1] <= keep_lead(v)
  assert not rising_edges(trace, Desire.turnRight)


def test_no_turn_pulse_at_road_speed():
  # 12 m/s to 25 m out, then 8: too fast for the left at its lead, so the turn is skipped, and no pulse goes even once the
  # car has slowed (turn_skip.py; the old fixed 10 m/s gate waited and pulsed 25 m out)
  def slower(t):
    return 12. if t < 175. / 12. else 8.
  assert not rising_edges(drive([(200., "turn", "left"), (900., "turn", "right")], seconds=30., speed_profile=slower),
                          Desire.turnLeft)
  # a pulse already out is held when the car speeds up, rather than dropped and sent again
  def faster(t):
    return 8. if t < 165. / 8. else 12.
  trace = drive([(200., "turn", "left"), (900., "turn", "right")], seconds=30., speed_profile=faster, turns_at=None)
  assert len(rising_edges(trace, Desire.turnLeft)) == 1
  assert all(desire == Desire.turnLeft for s, d, desire in trace if 166. < s < 200.)
  # a keep goes at any speed: test_slight_right_exit_sends_keep_right drives at 25 m/s


def test_keep_suppressed_without_lane_line_on_that_side():
  trace = drive([(300., "fork", "slight left"), (3000., "turn", "left")], v=15., seconds=30., lines=(False, True))
  assert not rising_edges(trace, Desire.keepLeft)


def test_uturn_and_straight_send_nothing():
  trace = drive([(100., "turn", "uturn"), (300., "continue", "straight"), (3000., "turn", "left")], v=10., seconds=40.)
  assert all(desire == Desire.none for s, d, desire in trace if s < 2900.)


def slowed_after_the_lead(t, s0=165., v0=7., v1=3.):
  """v0 to s0 (a left at 200 m: pulsed at its 40 m lead at turning speed), then v1 (a queue), as a speed profile."""
  return v0 if t < s0 / v0 else v1


def test_turn_desire_pulses_again_until_through_the_junction():
  # the model forgets a pulse after 132 frames (6.6 s); a far-side turn pulsed at its lead and then slowed to 3 m/s (a queue)
  # gets a new one every REPULSE_S up to the junction, and one at the entry
  trace = drive([(200., "turn", "left"), (900., "turn", "right")], turns_at=None, seconds=80., speed_profile=slowed_after_the_lead)
  edges = rising_edges(trace, Desire.turnLeft)
  assert TURN_DESIRE_LEAD_M - 1. < edges[0][1] < TURN_DESIRE_LEAD_M
  assert len(edges) == 4 and 0. < edges[-1][1] <= ENTRY_M  # every REPULSE_S on the approach, and one at the entry
  for (s0, _), (s1, _) in zip(edges[1:-2], edges[2:-1], strict=True):
    assert abs((s1 - s0) / 3. - REPULSE_S) < 0.3  # plus the gap
  # the drop between two pulses is long enough for modeld to see it
  gap = [desire for s, d, desire in trace if edges[1][0] < s < edges[2][0] and desire == Desire.none]
  assert len(gap) == PULSE_GAP_FRAMES
  # a curb-side turn at the same crawl: its one pulse at CURB_TURN_PULSE_M, nothing before it and no repeat after it
  # (2026-11-06, 1e: every fresh edge started the right where the car then was, 11-25 m out, into the inside curb)
  trace = drive([(200., "turn", "right"), (900., "turn", "left")], v=3., turns_at=None, seconds=80.)
  edges = rising_edges(trace, Desire.turnRight)
  assert len(edges) == 1 and CURB_TURN_PULSE_M - 3. * DT_MDL - 1. < edges[0][1] < CURB_TURN_PULSE_M, edges
  assert all(desire == Desire.none for s, d, desire in trace if s < 200. - CURB_TURN_PULSE_M - 1.)


def test_repeat_pulses_stop_once_the_car_has_turned():
  # turning at 3 m/s on an 8 m radius from 20 m out: 60 degrees take 2.8 s, after which a pulse would turn the car a
  # second time; before that, a slow turn still needs them (a far-side one pulsed at its lead, then slowed)
  trace = drive([(200., "turn", "left"), (900., "turn", "right")], turns_at=20., seconds=80., speed_profile=slowed_after_the_lead)
  edges = rising_edges(trace, Desire.turnLeft)
  assert len(edges) >= 2 and all(d > 20. - 2.8 * 3. for _, d in edges)


def crawl_into(steer_in, modifier="left", v=1.5, d0=30., seconds=24.):
  """A turn at d0 m crawled at v; the model steering into it at steer_in(d) 1/m at d m out (desiredCurvature, positive right,
  toward the turn's side); the car's yaw from it (v * curvature), so at a crawl it turns little while the model steers hard.
  [(d, desire, steer_in, turned_deg)] per frame."""
  from openpilot.selfdrive.turnpilot.route_follower import Maneuver, classify
  m = Maneuver(*classify("turn", modifier), "turn", modifier, 90.)
  dt_, out, d = DesireTiming(), [], d0
  for _ in range(int(seconds / DT_MDL)):
    curv = -m.side * steer_in(d)
    out.append((d, dt_.update(m, d, 1, False, v, 0., True, True, DT_MDL, -v * curv, curvature=curv), steer_in(d), dt_.turned_deg))
    d -= v * DT_MDL
  return out


def edges_of(trace, desire):
  return [d for (_, a, _, _), (d, b, _, _) in zip(trace, trace[1:], strict=False) if b == desire and a != desire]


def test_no_pulse_reaches_the_model_while_it_steers_into_the_turn():
  """BeamNG right_green at cbdf5f4 (both runs): crawling into the right at 1-2 m/s the entry pulse went with the model
  steering -46 and -75 deg into it but the car under TURN_STARTED_DEG of heading, and the fresh rising edge restarted the
  model's turn: it unwound within 0.4 s and took the corner late, into the oncoming half of the exit road. No repeat and no
  entry pulse while its curvature is STEER_IN_CURV or more toward the turn; the entry pulse goes once it lets go short of
  TURN_STARTED_DEG, and a steady approach still gets its repeats and entry. A far-side turn's repeats and entry (a curb-side
  turn has one pulse, 8 m out: below); STEER_IN_CURV is 0.005 since 2026-11-06, so a model steering in at 0.006, a 170 m
  radius, already holds the entry pulse back."""
  # the model steers in from 12 m out, through ENTRY_M, at 0.03 1/m (and at 0.006): nothing new reaches it
  for steer in (0.03, 0.006):
    trace = crawl_into(lambda d, steer=steer: steer if d < 12. else 0.)
    first = next(i for i, (_, desire, _, _) in enumerate(trace) if desire == Desire.turnLeft)
    steering = [(d, desire) for d, desire, curv, deg in trace[first:] if curv >= STEER_IN_CURV and deg < TURN_STARTED_DEG and d > 0.]
    assert any(d < ENTRY_M - 3. for d, _ in steering), steer  # deep inside ENTRY_M, still under TURN_STARTED_DEG by heading
    assert all(desire == Desire.turnLeft for _, desire in steering), f"a pulse went into the turn at {steer}"
  # it lets go of the corner 6 m out, short of TURN_STARTED_DEG: the entry pulse goes then
  edges = edges_of(crawl_into(lambda d: 0.03 if 6. < d < 12. else 0.), Desire.turnLeft)
  assert any(d <= 6. for d in edges) and not any(6. < d < 12. for d in edges), edges
  # never steering in, or under STEER_IN_CURV (0.004, a 250 m radius): the entry pulse inside ENTRY_M as before
  for steer in (0., 0.004):
    edges = edges_of(crawl_into(lambda d, steer=steer: steer if d < 12. else 0.), Desire.turnLeft)
    assert any(0. < d < ENTRY_M for d in edges), (steer, edges)
  # a curb-side turn crawled into: its one pulse 8 m out with the model not steering in, and no other edge
  edges = edges_of(crawl_into(lambda d: 0., modifier="right"), Desire.turnRight)
  assert len(edges) == 1 and CURB_TURN_PULSE_M - 0.2 < edges[0] < CURB_TURN_PULSE_M, edges
  # ...steered into through its pulse point at a turn's scale (0.06 from 12 m at right_green's crawl, the car yawing with it),
  # that one pulse is its only edge and none goes into it: the car OWN_TURN_DEG round on its approach, the turn is the
  # model's own; at a lean's or a bend's (0.03: under STEER_IN_TURN_CURV) it never waits, its one pulse at 8 m as with none
  trace = crawl_into(lambda d: 0.06 if d < 12. else 0., modifier="right")
  assert not edges_of(trace, Desire.turnRight) and all(desire != Desire.turnRight for _, desire, _, _ in trace)
  edges = edges_of(crawl_into(lambda d: 0.03 if d < 12. else 0., modifier="right"), Desire.turnRight)
  assert len(edges) == 1 and CURB_TURN_PULSE_M - 0.2 < edges[0] < CURB_TURN_PULSE_M, edges


def curb_turn(steer_in, v=3., d0=30., seconds=16., pulse_at_go=False, follow=1.):
  """A right d0 m ahead driven at v, TurnSkip and DesireTiming as turnpilotd runs them (the furthest and the latest given,
  both the nearest here), the model steering into it at steer_in(d) 1/m (desiredCurvature, positive right) and the car
  yawing with it (follow * v * curvature: the model's steer turns the car; follow 0, a car not following it round);
  pulse_at_go: TurnSkip's (the stop-sign decider's turn, rolling from its line). Returns ([(d, desire, curvature, turned_deg,
  skipped)] per frame, turned_deg the car's turn toward it as DesireTiming counts it (from the pulse, or from the model's
  turn-scale steer-in before it: approach_deg), the DesireTiming (own_at: d where the turn became the model's own, or None),
  the TurnSkip)."""
  from openpilot.selfdrive.turnpilot.route_follower import Maneuver, classify
  m = Maneuver(*classify("turn", "right"), "turn", "right", 90.)
  dt_, skip, out, d = DesireTiming(), TurnSkip(), [], d0
  dt_.own_at = None
  for _ in range(int(seconds / DT_MDL)):
    curv = steer_in(d)
    st = skip.update(m, d, 1, v, dt_.started, pulse_at_go=pulse_at_go, far=d, last=d)
    desire = dt_.update(None if st.skipped else m, d, 1, False, v, 0., True, True, DT_MDL, -follow * v * curv,
                        may_start=st.feasible_now, far=d, last=d, curvature=curv)
    if dt_.own and dt_.own_at is None:
      dt_.own_at = d
    out.append((d, desire, curv, dt_.turned_deg if dt_.started else dt_.approach_deg, st.skipped))
    d -= v * DT_MDL
  return out, dt_, skip


def right_edges(trace):
  return [(d, c) for (_, a, *_), (d, b, c, *_) in zip(trace, trace[1:], strict=False) if b == Desire.turnRight and a != Desire.turnRight]


def test_a_curb_side_turns_one_pulse_waits_while_the_model_steers_in_at_a_turns_scale_and_goes_when_it_lets_go():
  """2026-11-06 (the rights review): a curb-side turn's one pulse at CURB_TURN_PULSE_M, its only edge, skipped the steer-in
  guard the entry pulse has, and a fresh rising edge into a model already turning restarts its turn (BeamNG right_green:
  unwound within 0.4 s, then round late into the oncoming half of the exit road). Rolling, it waits while the model steers in
  at a turn's scale (STEER_IN_TURN_CURV) and goes the frame the model lets go of the corner (under it), the car short of
  OWN_FOLLOW_DEG round since that steer-in and short of turn_skip's deadline (CURB_FIRST_PULSE_M): never while it steers in,
  never at or past the junction."""
  from openpilot.selfdrive.turnpilot.desire_timing import OWN_FOLLOW_DEG
  from openpilot.selfdrive.turnpilot.turn_skip import CURB_FIRST_PULSE_M
  # at the cap's 3 m/s the model steers in at 0.06 (a 17 m radius) from 12 m and lets go 6.5 m out, the car not following it
  # round: the pulse waits from 8 m and goes as it lets go
  trace, dt_, skip = curb_turn(lambda d: 0.06 if 6.5 < d < 12. else 0., follow=0.)
  edges = right_edges(trace)
  assert len(edges) == 1 and CURB_FIRST_PULSE_M < 6.5 - 3. * DT_MDL - 1e-6 <= edges[0][0] <= 6.5 and edges[0][1] < STEER_IN_TURN_CURV, edges
  assert not any(sk for *_, sk in trace) and dt_.started and not dt_.own
  # nothing before it: no pulse while the model steers in (8 m to 6.5 m), none at or past the junction
  first = next(i for i, (d, desire, *_) in enumerate(trace) if desire == Desire.turnRight)
  assert all(desire == Desire.none for _, desire, *_ in trace[:first]) and trace[first][0] > 0.
  # letting go to a lean's steer (0.02, under the bar, still toward the turn) is letting go: the pulse goes then, its turned
  # count carried on from the steer-in (the car following a tenth of it: 1.8 deg, under OWN_FOLLOW_DEG)
  trace, dt_, skip = curb_turn(lambda d: 0.06 if 6.5 < d < 12. else 0.02 if d <= 6.5 else 0., follow=0.1)
  edges = right_edges(trace)
  assert len(edges) == 1 and 6.5 - 3. * DT_MDL - 1e-6 <= edges[0][0] <= 6.5 and not dt_.own and not skip.skipped, edges
  at = next(i for i, (d, *_) in enumerate(trace) if d == edges[0][0])
  assert 1. < trace[at][3] < OWN_FOLLOW_DEG, trace[at]
  # a far-side turn's first pulse is its lead's, 15-40 m out, and isn't held (its repeats and entry are: above)
  trace = crawl_into(lambda d: 0.03 if d < 20. else 0.)
  first = next(d for d, desire, _, _ in trace if desire == Desire.turnLeft)
  assert abs(first - FAR_LEAD_MIN_M) < 0.2, first


@pytest.mark.parametrize("v", [1., 2., 3., 4.])
def test_a_curb_side_turns_one_pulse_never_waits_on_a_road_bend_or_a_lean(v):
  """The 2026-11-06 review of the wait: at STEER_IN_CURV (0.005, the repeat and entry guard's bar) the one pulse waited on any
  steer toward the turn, so a right off a road bending its way, or with the model leaning to the curb, waited to turn_skip's
  deadline and was skipped 'Too Fast to Turn' (0.006-0.03 1/m from 40 m, the car yawing with it, at 1-4 m/s: every one
  skipped at 5 m). The wait's bar is a turn's (STEER_IN_TURN_CURV, 0.04): a road's bend (0.006-0.03 1/m from 40 m, a 33-170 m
  radius) and the leans the logged model made toward the right with no desire in its queue (1e: 0.038 for 2.75 s, 0.024 for
  1.5 s, here from a second before the pulse point) all get the pulse at 8 m, and none is skipped or taken as its own."""
  for curv in (0.006, 0.01, 0.02, 0.03):
    trace, dt_, skip = curb_turn(lambda d, curv=curv: curv if d < 40. else 0., v=v, d0=45., seconds=45. / v + 2.)
    edges = right_edges(trace)
    assert len(edges) == 1 and CURB_TURN_PULSE_M - v * DT_MDL - 1e-6 <= edges[0][0] < CURB_TURN_PULSE_M, (curv, edges)
    assert not skip.skipped and dt_.started and not dt_.own, curv
  for peak, dur in ((0.038, 2.75), (0.024, 1.5)):
    start = CURB_TURN_PULSE_M + v
    trace, dt_, skip = curb_turn(lambda d, peak=peak, dur=dur, start=start: peak if start - v * dur < d < start else 0., v=v,
                                 d0=start + 10., seconds=(start + 10.) / v + 2.)
    edges = right_edges(trace)
    assert len(edges) == 1 and CURB_TURN_PULSE_M - v * DT_MDL - 1e-6 <= edges[0][0] < CURB_TURN_PULSE_M, (peak, edges)
    assert not skip.skipped and dt_.started and not dt_.own, peak


def test_a_curb_side_turn_the_model_takes_on_its_own_while_its_pulse_waits_is_started_with_no_pulse():
  """The model steering into the right by itself (an end-of-road T, a right-turn-only lane) and the car turning with it: the
  turn is under way, so no pulse goes into it (a pulse in the turn turns the car on past the exit road), and it counts as
  started (own): never skipped (turn_skip never skips a turn out), cleared at its angle as any turn. At 3 m/s on 0.12 1/m from
  9 m (20.6 deg/s) the car is 6.9 deg round at the pulse point: its pulse waits, the car following the model OWN_FOLLOW_DEG
  round, and it is the model's own a frame later; from 10 m, OWN_TURN_DEG round on its approach before the pulse point.
  Turning back under TURN_STARTED_DEG past its deadline (the model straightening out of it) it goes back to the wait and gets
  no pulse (test_a_curb_side_turn_taken_as_the_models_own_goes_back...). Not following it round by the deadline it is skipped
  out loud (test_turn_skip: ..._skipped_out_loud_at_its_deadline)."""
  from openpilot.selfdrive.turnpilot.desire_timing import OWN_TURN_DEG
  for start, at_most, at_least in ((9., CURB_TURN_PULSE_M, CURB_TURN_PULSE_M - 0.5), (10., 10., CURB_TURN_PULSE_M)):
    trace, dt_, skip = curb_turn(lambda d, start=start: 0.12 if d < start else 0.)
    assert not right_edges(trace) and all(desire != Desire.turnRight for _, desire, *_ in trace), start
    assert at_least < dt_.own_at <= at_most, (start, dt_.own_at)
    assert dt_.own and dt_.started and dt_.done and not any(sk for *_, sk in trace) and not skip.skipped
    assert [desire for _, desire, *_ in trace].count(CLEAR_DESIRE) == CLEAR_FRAMES  # cleared at its angle (nothing queued: harmless)
    assert start == 9. or next(deg for d, _, _, deg, _ in trace if d <= dt_.own_at) >= OWN_TURN_DEG  # its approach counted
  # taken on its own, then the model straightens it back under TURN_STARTED_DEG, 2-3 m out (past its deadline): given back to
  # the wait, no pulse into it
  from openpilot.selfdrive.turnpilot.route_follower import Maneuver, classify
  m = Maneuver(*classify("turn", "right"), "turn", "right", 90.)
  dt_, d, out = DesireTiming(), 7., []
  for k in range(int(3. / DT_MDL)):
    yaw = -0.4 if k < 20 else 0.3  # 23 deg/s right for 1 s, then back left
    out.append(dt_.update(m, d, 1, False, 3., 0., True, True, DT_MDL, yaw, far=d, last=d, curvature=0.1 if k < 20 else -0.01))
    d -= 3. * DT_MDL
  assert not dt_.own and not dt_.started and dt_.turned_deg < TURN_STARTED_DEG and Desire.turnRight not in out


@pytest.mark.parametrize("start, curv", [(12., 0.05), (14., 0.05), (16., 0.04), (12., 0.055), (10., 0.05), (9., 0.05),
                                         (8.5, 0.05), (8., 0.05)])
def test_a_wide_right_the_model_starts_before_its_pulse_point_is_its_own_never_skipped(start, curv):
  """The 2026-11-06 review: the car's turn toward a right was counted only from its pulse's wait (8 m), so a right the model
  starts on its own on a wide arc before the pulse point, the car really 20-26 deg round by the 5 m deadline, read 8.6 deg
  and was skipped tooLate mid-turn ('Too Fast to Turn', navd's reroute, its corner cap and its crosswalk's walker hold dropped
  under it); so was any own right wider than a 17 m radius, wherever it began (0.05 from 8-10 m: 8.2 deg counted). The count
  runs from the model's turn-scale steer-in inside APPROACH_M: OWN_TURN_DEG round on its approach, or following the model
  OWN_FOLLOW_DEG round once its pulse waits, the turn is the model's own, never pulsed into, never skipped, and its count
  carries the turn it made before (cleared at its angle by it)."""
  import math
  from openpilot.selfdrive.turnpilot.desire_timing import APPROACH_M, OWN_FOLLOW_DEG, OWN_TURN_DEG
  from openpilot.selfdrive.turnpilot.turn_skip import CURB_FIRST_PULSE_M
  assert start < APPROACH_M
  trace, dt_, skip = curb_turn(lambda d: curv if d < start else 0., seconds=25.)
  assert not right_edges(trace) and not skip.skipped and not any(sk for *_, sk in trace), (dt_.own_at, skip.reason)
  assert dt_.own and dt_.own_at > CURB_FIRST_PULSE_M, dt_.own_at
  real = math.degrees(curv * (start - dt_.own_at))  # the car's real turn toward it when it became the model's own
  own_deg = next(deg for d, _, _, deg, _ in trace if d <= dt_.own_at)
  assert abs(own_deg - real) < 1.5 and own_deg >= (OWN_TURN_DEG if dt_.own_at > CURB_TURN_PULSE_M else OWN_FOLLOW_DEG), (own_deg, real)
  assert [desire for _, desire, *_ in trace].count(CLEAR_DESIRE) == CLEAR_FRAMES


def test_a_reroutes_same_junction_carries_the_models_steer_in_into_its_right():
  """A reroute while the model has the car turning into a right on its own (navd's new route, its first step the same
  junction: RouteFollower's span carry, same_bearings within SAME_TURN_M) is a new maneuver id, and DesireTiming's state for
  it starts over; the car's turn toward it since the model's steer-in began carries on (desire_timing._carry), so the turn
  is the model's own as it would have been, not counted from nothing again (nor refused as a steer-in under way when its step
  came up). Another junction (other bearings) carries nothing."""
  from openpilot.selfdrive.turnpilot.desire_timing import OWN_TURN_DEG
  from openpilot.selfdrive.turnpilot.route_follower import maneuver_from_instruction
  from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
  same = maneuver_from_instruction(nav_instruction(14., "turn", "right", junction=junction_of("right")))
  other = maneuver_from_instruction(nav_instruction(14., "turn", "right", junction=junction_of("right", angle=50.)))
  assert same.geometry and other.geometry
  for second, carried in ((same, True), (other, False)):
    dt_, d, own_at = DesireTiming(), 15., None
    for _ in range(int(3. / DT_MDL)):  # 0.06 toward it from 14 m at 3 m/s, the car with it (0.52 deg a frame); rerouted 12 m out
      mid = 1 if 14. - d < 2. - 1e-6 else 2
      c = 0.06 if d < 14. else 0.
      dt_.update(same if mid == 1 else second, d, mid, False, 3., 0., True, True, DT_MDL, -3. * c, far=d, last=d,
                 curvature=c, replaced=mid == 2 and dt_.maneuver_id == 1)
      if dt_.own and own_at is None:
        own_at = d
      d -= 3. * DT_MDL
    first_step = 14. - 3. * DT_MDL - OWN_TURN_DEG / (0.06 * 57.2958)  # where the car is OWN_TURN_DEG round, counted from 14 m
    if carried:
      assert own_at is not None and dt_.turned_deg >= OWN_TURN_DEG and abs(own_at - first_step) < 0.2, own_at
    else:
      # another junction: a steer-in already under way when its step came up is not its (test_a_steer_in_already_under_way...)
      assert own_at is None and dt_.approach_deg == 0., own_at


def test_a_steer_in_already_under_way_when_a_curb_side_turn_comes_up_is_not_its_own():
  """The tail of the turn before (a right, then another 15 m on: the second the maneuver while the model still has the car
  turning out of the first) is a turn-scale steer-in toward it from the moment it comes up; counted, it took the second right
  as the model's own with no pulse, and the model, straightening out of the first, never turned it: missed in silence. The
  count begins only once the model has been seen not steering in for the turn (desire_timing._approach): straightening out of
  the first, then into the second at a turn's scale, it counts from there."""
  from openpilot.selfdrive.turnpilot.desire_timing import OWN_TURN_DEG
  from openpilot.selfdrive.turnpilot.route_follower import Maneuver, classify
  m = Maneuver(*classify("turn", "right"), "turn", "right", 90.)
  dt_, d, out, own_at = DesireTiming(), 15., [], None
  for k in range(int(4. / DT_MDL)):
    c = 0.1 if k < 30 else 0. if k < 34 else 0.12  # 1.5 s of the first right's tail (17 deg), straight, then into the second
    out.append(dt_.update(m, d, 1, False, 3., 0., True, True, DT_MDL, -3. * c, far=d, last=d, curvature=c))
    if k == 29:
      assert not dt_.own and dt_.approach_deg == 0. and d > CURB_TURN_PULSE_M, (d, dt_.approach_deg)  # never counted
    if own_at is None and dt_.own:
      own_at = d
    d -= 3. * DT_MDL
  # into the second from 15 - 34 * 0.15 = 9.9 m: OWN_TURN_DEG round 1.5 m on, the model's own there, no pulse into it
  assert own_at is not None and abs(own_at - (15. - 34 * 3. * DT_MDL - OWN_TURN_DEG / (0.12 * 57.2958))) < 0.5, own_at
  assert Desire.turnRight not in out


def test_a_curb_side_turns_go_standing_is_its_pulse_whatever_the_model_steers():
  """Standing, the curb-side turn's one pulse goes whatever the model's desiredCurvature reads: a hold's go at its junction's
  own hold line (8.26 m, ped_crosswalk_right: the model didn't pull away on its own with no fresh edge), a go inside the pulse
  point, and a first pulse due with the car standing. The car isn't turning, and standing the model's curvature is noise."""
  from openpilot.selfdrive.turnpilot.route_follower import Maneuver, classify
  m = Maneuver(*classify("turn", "right"), "turn", "right", 90.)
  for d, go in ((CURB_TURN_PULSE_M + 0.26, True), (6., True), (6., False)):
    dt_ = DesireTiming()
    out = dt_.update(m, d, 1, False, 0., 0., True, True, DT_MDL, 0., go=go, far=d, last=d, curvature=0.05)
    assert out == Desire.turnRight and dt_.started and not dt_.own, (d, go)


def test_pulse_again_after_a_standstill():
  # stopped at 15 m from a left turn (a red light) for 10 s, then on through the turn
  def speed(t):
    return 8. if t < 185. / 8. else (0. if t < 185. / 8. + 10. else 5.)
  trace = drive([(200., "turn", "left"), (900., "turn", "right")], seconds=50., speed_profile=speed)
  edges = rising_edges(trace, Desire.turnLeft)
  assert abs(edges[-2][0] - 185.) < 1.  # a pulse goes as the car pulls away, 15 m out...
  assert 0. < edges[-1][1] < ENTRY_M and edges[-1][0] - edges[-2][0] >= ENTRY_GAP_S * 5. - 0.5  # ...and the entry pulse after it
  stopped = [desire for s, d, desire in trace if abs(s - 185.) < 0.5]
  assert stopped.count(Desire.none) >= int((10. - STANDSTILL_REARM_S) / DT_MDL) - 2


def turned(trace_yaw, until):
  return sum(yaw for t, yaw in trace_yaw if t < until)


def test_turn_desire_cleared_once_turned():
  # turning at 4 m/s on an 8 m radius from its pulse 8 m out: the clear goes out at 80 degrees, before navd moves on
  trace = drive([(200., "turn", "right"), (900., "turn", "left")], v=4., turns_at=CURB_TURN_PULSE_M, seconds=60., steer_sign=-1.)
  desires = [desire for s, d, desire in trace]
  assert desires.count(CLEAR_DESIRE) == CLEAR_FRAMES
  i = desires.index(CLEAR_DESIRE)
  s, d, _ = trace[i]
  deg = (CURB_TURN_PULSE_M - d) / 8. * 57.3  # turned since the pulse
  assert abs(deg - (90. - CLEAR_MARGIN_DEG)) < 3. and d > -10.
  assert Desire.turnRight not in desires[i:]  # and no pulse after it
  assert rising_edges(trace, Desire.turnRight)


def step_through(dt_, legs, dt=DT_MDL):
  """Feed DesireTiming a scripted drive: legs of (seconds, maneuver_id, maneuver, distance at the start, v, yaw rate,
  passed on the first frame). Returns the desires."""
  out = []
  for seconds, mid, maneuver, d0, v, yaw, passed in legs:
    for k in range(int(round(seconds / dt))):
      out.append(dt_.update(maneuver, d0 - v * k * dt, mid, passed and k == 0, v, 0., True, True, dt, yaw))
  return out


LEFT_TURN = maneuver_from_instruction(nav_instruction(40., "turn", "left"))
RIGHT_TURN = maneuver_from_instruction(nav_instruction(40., "turn", "right"))
KEEP_LEFT = maneuver_from_instruction(nav_instruction(40., "fork", "slight left"))


def test_clear_waits_for_the_turn_past_the_junction():
  # a slow corner: navd moves on with the car 30 degrees round, the clear goes once it has turned CLEAR_MIN_DEG
  yaw = 0.3  # rad/s, 17 deg/s
  dt_ = DesireTiming()
  out = step_through(dt_, [(10., 1, LEFT_TURN, 40., 4., 0., False),            # the approach: pulses
                           (30. / 17.2, 1, LEFT_TURN, 0., 4., yaw, False),     # 30 degrees round the corner
                           (0.05, 2, RIGHT_TURN, 300., 4., yaw, True),         # navd moves on
                           (4., 2, RIGHT_TURN, 300., 4., yaw, False)])
  assert Desire.turnLeft in out
  i = out.index(CLEAR_DESIRE)
  t_clear = i * DT_MDL - 10. - 30. / 17.2
  assert abs(t_clear - (CLEAR_MIN_DEG - 30.) / 17.2) < 0.15
  assert out.count(CLEAR_DESIRE) == CLEAR_FRAMES


def test_no_clear_without_a_turn():
  # passed straight on (a missed turn): nothing to clear, and no clear for a keep desire
  for maneuver, desire in ((LEFT_TURN, Desire.turnLeft), (KEEP_LEFT, Desire.keepLeft)):
    out = step_through(DesireTiming(), [(10., 1, maneuver, 40., 4., 0., False),
                                        (CLEAR_WAIT_S + 2., 2, RIGHT_TURN, 300., 4., 0., True),
                                        (5., 2, RIGHT_TURN, 300., 4., 0.3, False)])
    assert desire in out and CLEAR_DESIRE not in out


def test_next_turn_pulse_cancels_a_pending_clear():
  # navd moves on with the car 20 degrees round the left, and the next turn is a right 7 m on (inside its one pulse's 8 m):
  # its pulse goes out at once, and the rest of the left doesn't clear it
  out = step_through(DesireTiming(), [(10., 1, LEFT_TURN, 40., 4., 0., False),
                                      (20. / 17.2, 1, LEFT_TURN, 0., 4., 0.3, False),
                                      (3., 2, RIGHT_TURN, 7., 4., 0.3, True)])
  assert CLEAR_DESIRE not in out and Desire.turnRight in out


def engage_through(dt_, legs, dt=DT_MDL):
  """step_through with openpilot steering or not: legs of (seconds, maneuver, distance at the start, v, yaw rate, lat_active),
  all one maneuver. Returns the desires."""
  out = []
  for seconds, maneuver, d0, v, yaw, lat_active in legs:
    for k in range(int(round(seconds / dt))):
      out.append(dt_.update(maneuver, d0 - v * k * dt, 1, False, v, 0., True, True, dt, yaw, lat_active=lat_active))
  return out


def pulses(out, desire):
  return sum(cur == desire and prev != desire for prev, cur in zip([Desire.none] + out[:-1], out, strict=True))


def test_no_desire_until_openpilot_steers():
  # a slow approach with openpilot off from 40 m (longer than the timeout and every repeat), engaged 8 m out: one fresh
  # pulse then, none before
  out = engage_through(DesireTiming(), [(32. / 1.5, LEFT_TURN, 40., 1.5, 0., False), (2., LEFT_TURN, 8., 1.5, 0., True)])
  i = int(round(32. / 1.5 / DT_MDL))
  assert Desire.turnLeft not in out[:i] and CLEAR_DESIRE not in out
  assert out[i] == Desire.turnLeft and pulses(out, Desire.turnLeft) == 1
  # a keep the same way
  out = engage_through(DesireTiming(), [(2., KEEP_LEFT, 20., 5., 0., False), (1., KEEP_LEFT, 10., 5., 0., True)])
  assert Desire.keepLeft not in out[:40] and out[40] == Desire.keepLeft


def test_disengaged_on_the_approach_pulses_again_when_engaged():
  # at 7 m/s, where a far-side turn's lead is the full 40 m
  out = engage_through(DesireTiming(), [(1., LEFT_TURN, 40., 7., 0., True), (1., LEFT_TURN, 33., 7., 0., False),
                                        (1., LEFT_TURN, 26., 7., 0., True)])
  assert pulses(out, Desire.turnLeft) == 2 and out[20:40] == [Desire.none] * 20 and out[40] == Desire.turnLeft


def test_no_pulse_on_engaging_in_the_turn():
  # the driver took over 15 m out and turned the car 30 degrees by hand before engaging again: a pulse now would turn it
  # on past the exit road
  yaw = 0.3  # rad/s, 17 deg/s
  out = engage_through(DesireTiming(), [(4., LEFT_TURN, 40., 7., 0., True), (30. / 17.2, LEFT_TURN, 15., 4., yaw, False),
                                        (1., LEFT_TURN, 8., 4., yaw, True)])
  assert pulses(out, Desire.turnLeft) == 2  # the one at the lead and a repeat on the REPULSE_S clock, both before the takeover
  assert Desire.turnLeft not in out[int(round(4. / DT_MDL)):]


def test_turn_desire_cleared_with_openpilot_off():
  # pulsed with openpilot steering, then the driver took over and drove the turn: the clear still goes at 80 degrees
  yaw = 0.5
  out = engage_through(DesireTiming(), [(5., LEFT_TURN, 40., 4., 0., True), (3.5, LEFT_TURN, 20., 4., yaw, False)])
  assert Desire.turnLeft in out and out.count(CLEAR_DESIRE) == CLEAR_FRAMES
  i = out.index(CLEAR_DESIRE)
  assert abs((i * DT_MDL - 5.) * yaw * 57.3 - (90. - CLEAR_MARGIN_DEG)) < 2.



def drive_into_new_route(marked: bool, v=7.):
  """A left at 300 m, pulsed from its lead; 25 m before it a new route arrives that starts with a left 284 m on, after
  one invalid instruction from navd (marked) or none. Returns the switch point and [(s, distance, desire)] per frame."""
  navd, rf, dt_ = FakeNavd([(300., "turn", "left"), (900., "turn", "right")]), RouteFollower(), DesireTiming()
  s, switch, trace = 0., None, []
  for _ in range(int(90. / DT_MDL)):
    if switch is None and 300. - s < 25.:
      switch, navd = s, FakeNavd([(s + 284., "turn", "left"), (s + 900., "turn", "right")])
      instr, updated, valid = nav_instruction(0., valid=False), marked, False
      navd.frame = int(marked)  # the new route's first instruction a second after the invalid one, or now
    else:
      instr = navd.update(s)
      updated, valid = instr is not None, True
    rf.update(instr, updated, valid, v, DT_MDL)
    trace.append((s, rf.distance, dt_.update(rf.upcoming, rf.distance, rf.maneuver_id, rf.passed, v, 0., True, True, DT_MDL,
                                             replaced=rf.replaced)))
    s += v * DT_MDL
  return switch, trace


def test_new_route_clears_the_old_turn_at_once():
  # the old pulse is cleared at once, and the next goes at the new left's lead, not at 284, 260 or 220 m
  for marked in (False, True):
    switch, trace = drive_into_new_route(marked)
    assert any(desire == Desire.turnLeft for s, _, desire in trace if s < switch), marked
    cleared = [s for s, _, desire in trace if desire == CLEAR_DESIRE]
    assert len(cleared) == CLEAR_FRAMES and switch <= cleared[0] < switch + 2., marked
    after = [(d, desire) for s, d, desire in trace if s > switch]
    assert all(desire != Desire.turnLeft for d, desire in after if d > TURN_DESIRE_LEAD_M), marked
    assert any(desire == Desire.turnLeft for d, desire in after if d < TURN_DESIRE_LEAD_M), marked


def test_navd_going_quiet_mid_turn_still_clears_the_desire():
  # the pulse is out and the car is 20 degrees into the turn when navd stops publishing: the route is lost (no maneuver),
  # but the turn's desire is still cleared once the car has turned CLEAR_MIN_DEG, not left in the model's queue
  navd, rf, dt_ = FakeNavd([(100., "turn", "left"), (900., "turn", "right")]), RouteFollower(), DesireTiming()
  s, v, out, quiet_at = 0., 4., [], None
  for _ in range(int(40. / DT_MDL)):
    instr = navd.update(s) if quiet_at is None else None
    rf.update(instr, instr is not None, True, v, DT_MDL)
    yaw = 0.25 if 100. - s < 8. else 0.  # 14 deg/s from 8 m out
    if quiet_at is None and dt_.turned_deg >= 20.:
      quiet_at = s
    out.append((rf.valid, dt_.update(rf.upcoming, rf.distance, rf.maneuver_id, rf.passed, v, 0., True, True, DT_MDL, yaw,
                                     replaced=rf.replaced)))
    s += v * DT_MDL
  assert quiet_at is not None and Desire.turnLeft in [d for _, d in out]
  assert [d for _, d in out].count(CLEAR_DESIRE) == CLEAR_FRAMES
  first_clear = [d for _, d in out].index(CLEAR_DESIRE)
  assert not out[first_clear][0]  # after the route was lost


def test_abandoned_turn_sends_nothing_more_but_is_still_cleared():
  # TurnPilot hands back 10 degrees into a turn: no further pulse, and the desire still out in the model's queue is cleared
  # once the car has turned CLEAR_MIN_DEG, once
  yaw = 0.3  # rad/s, 17 deg/s
  dt_ = DesireTiming()
  out = step_through(dt_, [(10., 1, LEFT_TURN, 40., 4., 0., False), (10. / 17.2, 1, LEFT_TURN, 0., 4., yaw, False)])
  assert Desire.turnLeft in out
  dt_.abandon()
  after = step_through(dt_, [(5., 1, None, 0., 4., yaw, False)])  # turnpilotd sends no maneuver once it has handed back
  assert set(after) == {Desire.none, CLEAR_DESIRE} and after.count(CLEAR_DESIRE) == CLEAR_FRAMES
  t_clear = after.index(CLEAR_DESIRE) * DT_MDL
  assert abs(t_clear - (CLEAR_MIN_DEG - 10.) / 17.2) < 0.15
  # handed back before any turn: nothing to clear, nothing sent
  dt_ = DesireTiming()
  dt_.abandon()
  assert set(step_through(dt_, [(5., 1, None, 30., 4., yaw, False)])) == {Desire.none}


def test_no_pulse_after_a_standstill_in_the_turn():
  # stopped 40 degrees into a left for longer than STANDSTILL_REARM_S (a queue, a pedestrian on the exit crosswalk): the
  # car pulls away with no new pulse (one now turns it on past the exit road), and the turn is still cleared at 80 degrees
  yaw = 0.5  # rad/s, 29 deg/s
  dt_ = DesireTiming()
  out = step_through(dt_, [(9., 1, LEFT_TURN, 45., 5., 0., False),            # the approach: pulses
                           (40. / 28.65, 1, LEFT_TURN, 0., 4., yaw, False),    # 40 degrees round the corner
                           (STANDSTILL_REARM_S + 1., 1, LEFT_TURN, -2., 0., 0., False),  # stopped
                           (3., 1, LEFT_TURN, -2., 4., yaw, False)])           # on round the corner
  turning = int(round(9. / DT_MDL))
  assert Desire.turnLeft in out[:turning]
  assert pulses(out, Desire.turnLeft) == pulses(out[:turning], Desire.turnLeft)
  assert out.count(CLEAR_DESIRE) == CLEAR_FRAMES
  # stopped before turning at all (a red light), the pulse still goes again as the car pulls away
  dt_ = DesireTiming()
  out = step_through(dt_, [(5., 1, LEFT_TURN, 40., 5., 0., False), (STANDSTILL_REARM_S + 1., 1, LEFT_TURN, 15., 0., 0., False),
                           (1., 1, LEFT_TURN, 15., 4., 0., False)])
  stop_end = int(round((5. + STANDSTILL_REARM_S + 1.) / DT_MDL))
  assert out[stop_end] == Desire.turnLeft


def test_pulse_after_a_standstill_early_in_the_turn():
  # waiting in the junction for oncoming traffic 18-22 degrees into an unprotected left, longer than the model's desire
  # queue: the turn would lapse with no pulse as the car pulls away (H2: with the pulse gone at 5-16 degrees the model
  # gave up 3 turns of 3), so one goes. 40 degrees in, none does (it turns the car on past the exit road)
  yaw = 0.5  # rad/s, 29 deg/s
  for turned_deg, stop_s, pulse in ((18., 8., True), (22., 10., True), (40., 3., False)):
    dt_ = DesireTiming()
    out = step_through(dt_, [(9., 1, LEFT_TURN, 45., 5., 0., False),
                             (turned_deg / 28.65, 1, LEFT_TURN, 0., 4., yaw, False),
                             (stop_s, 1, LEFT_TURN, -2., 0., 0., False),
                             (3., 1, LEFT_TURN, -2., 4., yaw, False)])
    after = out[int(round((9. + turned_deg / 28.65 + stop_s) / DT_MDL)) - 1:]
    assert (pulses(after, Desire.turnLeft) == 1) == pulse, turned_deg
    assert out.count(CLEAR_DESIRE) == CLEAR_FRAMES, turned_deg


def glitch_through(mode, modifier="left", glitch_from=14., v_floor=6.):
  """A committed turn (a left's desire out at its lead, a right's at its one 8 m pulse), braking to v_floor m/s; from
  glitch_from m out navd's next three updates are a glitch (mode), as turnpilotd runs RouteFollower, TurnSkip and
  DesireTiming. Returns [(distance, desire, id, skipped)]."""
  rf, dt_, skip = RouteFollower(), DesireTiming(), TurnSkip()
  turn_modifier = modifier
  right = (0., 270. if modifier == "left" else 90., 2)
  dist, v, rc, k, out = 80., 8., 0, 0, []
  for i in range(600):
    v = max(v_floor, v - DT_MDL)
    dist -= v * DT_MDL
    updated = i % 20 == 0
    reported, valid, mtype, modifier = dist, True, "turn", turn_modifier
    if updated and dist < glitch_from and k < 3:
      k += 1
      if mode == 'invalid':
        valid = False
      elif mode == 'reroute_far':  # one invalid, then a new route whose same right reads 25 m off
        valid, rc, reported = (False, rc, reported) if k == 1 else (True, 1, dist + 25.)
      elif mode == 'reroute_past':  # one invalid, then a new route starting past the junction
        valid, rc, mtype, modifier, reported = (False, rc, mtype, modifier, reported) if k == 1 else (True, 1, "new name", "straight", 200.)
      elif mode == 'jump_close':
        reported = dist + 60.
    instr = nav_instruction(reported, mtype, modifier, valid=valid, junction=right if mtype == "turn" else (90., 90., 2),
                            route_count=rc)
    committed = dt_.maneuver_id == rf.maneuver_id and dt_.started and not dt_.done and dt_.side != 0
    rf.update(instr, updated, valid, v, DT_MDL, committed=committed)
    state = skip.update(rf.upcoming, rf.distance, rf.maneuver_id, v, dt_.maneuver_id == rf.maneuver_id and dt_.started)
    desire = dt_.update(None if state.skipped else rf.upcoming, rf.distance, rf.maneuver_id, rf.passed, v, 0., True, True, DT_MDL,
                        0., replaced=rf.replaced, may_start=state.feasible_now)
    out.append((dist, desire, rf.maneuver_id, state.skipped))
    if dist < -20.:
      break
  return out


def test_committed_turn_rides_out_route_glitches():
  # navd off the step for three updates, a reroute onto the same turn from 25 m off, a reroute starting past it, a 60 m
  # GPS jump 12 m out: the committed turn is never cleared, skipped or pulsed again under a new id before it is over. A left
  # committed at its lead, glitched from 14 m; a right committed at its one pulse, 8 m out, glitched from just inside it
  # (the right braked to 3 m/s, the curb-side cap's speed by 20 m: at 6 m/s 8 m out it is too fast for it, turn_skip.py)
  for modifier, glitch_from, v_floor, desire_out in (("left", 14., 6., Desire.turnLeft),
                                                    ("right", CURB_TURN_PULSE_M - 0.5, 3., Desire.turnRight)):
    for mode in ('invalid', 'reroute_far', 'reroute_past', 'jump_close'):
      out = glitch_through(mode, modifier, glitch_from, v_floor)
      first, at = next((mid, d) for d, desire, mid, _ in out if desire == desire_out)
      before = [(d, desire, mid, skipped) for d, desire, mid, skipped in out if d > -PASSED_M]
      assert all(mid == first and not skipped and desire != CLEAR_DESIRE for d, desire, mid, skipped in before if d < at), (modifier, mode)


def test_go_at_standstill_pulses_now():
  # held at the line past the model's queue (a walker, a stop sign), and the hold lets go: the turn pulses at once, still
  # standing, not once the car moves (the model won't pull away from a standstill with its pulse forgotten)
  dt_ = DesireTiming()
  step_through(dt_, [(5., 1, LEFT_TURN, 40., 5., 0., False), (STANDSTILL_REARM_S + 3., 1, LEFT_TURN, 15., 0., 0., False)])
  assert dt_.update(LEFT_TURN, 15., 1, False, 0., 0., True, True, DT_MDL, 0., go=True) == Desire.turnLeft
  # and the car gets STANDSTILL_REARM_S to move off on it before it re-arms
  out = [dt_.update(LEFT_TURN, 15., 1, False, 0., 0., True, True, DT_MDL, 0.) for _ in range(int(1.5 / DT_MDL))]
  assert set(out) == {Desire.turnLeft}
  # a go soon after a pulse (stopped under STANDSTILL_REARM_S): a repeat pulse, through a short gap
  dt_ = DesireTiming()
  step_through(dt_, [(5., 1, LEFT_TURN, 40., 5., 0., False), (1., 1, LEFT_TURN, 15., 0., 0., False)])
  out = [dt_.update(LEFT_TURN, 15., 1, False, 0., 0., True, True, DT_MDL, 0., go=k == 0) for k in range(10)]
  assert out[:PULSE_GAP_FRAMES] == [Desire.none] * PULSE_GAP_FRAMES and out[PULSE_GAP_FRAMES] == Desire.turnLeft
  # a curb-side turn's line 6 m out, inside its one pulse's 8 m: pulsed on the approach, held, and the go pulses at once
  dt_ = DesireTiming()
  out = step_through(dt_, [(4., 1, RIGHT_TURN, 18., 3., 0., False), (STANDSTILL_REARM_S + 3., 1, RIGHT_TURN, 6., 0., 0., False)])
  assert pulses(out, Desire.turnRight) == 1 and dt_.started
  assert dt_.update(RIGHT_TURN, 6., 1, False, 0., 0., True, True, DT_MDL, 0., go=True) == Desire.turnRight
  # 40 degrees into the turn: no pulse on a go either (it turns the car on past the exit road)
  dt_ = DesireTiming()
  step_through(dt_, [(9., 1, LEFT_TURN, 45., 5., 0., False), (40. / 28.65, 1, LEFT_TURN, 0., 4., 0.5, False),
                     (STANDSTILL_REARM_S + 1., 1, LEFT_TURN, -2., 0., 0., False)])
  assert dt_.update(LEFT_TURN, -2., 1, False, 0., 0., True, True, DT_MDL, 0., go=True) == Desire.none


def test_a_curb_side_turns_go_from_a_line_further_out_waits_for_its_pulse_point():
  """2026-11-06 (1e's rights cut the inside curb from pulses 11-25 m out): a hold's go standing further from a curb-side turn's
  junction than CURB_TURN_PULSE_M (a stop line, a red, the stop-sign decider's go) sends no desire at the go; the car pulls
  away straight under the model and the turn's one pulse goes once it is inside 8 m, as it would rolling in."""
  dt_ = DesireTiming()
  out = step_through(dt_, [(5., 1, RIGHT_TURN, 40., 5., 0., False)])  # to 15 m: nothing yet
  out += [dt_.update(RIGHT_TURN, 15., 1, False, 0., 0., True, True, DT_MDL, 0., holding=True) for _ in range(int(10. / DT_MDL))]
  assert set(out) == {Desire.none} and not dt_.started
  out = [dt_.update(RIGHT_TURN, 15., 1, False, 0., 0., True, True, DT_MDL, 0., go=True)]
  out += step_through(dt_, [(4., 1, RIGHT_TURN, 15., 2., 0., False)])
  edges = [15. - 2. * (k - 1) * DT_MDL for k in range(1, len(out)) if out[k] == Desire.turnRight and out[k - 1] != Desire.turnRight]
  assert out[0] == Desire.none and len(edges) == 1 and CURB_TURN_PULSE_M - 0.2 < edges[0] < CURB_TURN_PULSE_M, edges
  # standing at the junction's own hold line (scene_policy.HOLD_LINE_M, 8 m: test_turnpilotd's walker hold stood 8.26 m out):
  # within CURB_GO_SLACK_M of the pulse point, the go is the pulse, at once, standing
  for stand_m, pulsed in ((8.26, True), (CURB_TURN_PULSE_M + CURB_GO_SLACK_M - 0.05, True), (CURB_TURN_PULSE_M + CURB_GO_SLACK_M + 0.5, False)):
    dt_ = DesireTiming()
    step_through(dt_, [(4., 1, RIGHT_TURN, stand_m + 12., 3., 0., False)])
    held = [dt_.update(RIGHT_TURN, stand_m, 1, False, 0., 0., True, True, DT_MDL, 0., holding=True) for _ in range(40)]
    assert set(held) == {Desire.none}
    assert (dt_.update(RIGHT_TURN, stand_m, 1, False, 0., 0., True, True, DT_MDL, 0., go=True) == Desire.turnRight) == pulsed, stand_m


def test_a_curb_side_turns_go_is_its_pulse_only_within_the_slack_by_the_reading_its_pulse_goes_by():
  """2026-11-07 review (rights R1): the go standing at a hold was a curb-side turn's one pulse whenever the NEAREST reading had
  the car inside CURB_TURN_PULSE_M + CURB_GO_SLACK_M, while its rolling pulse goes by pulse_distance (the furthest, at the
  latest by navd's latest). The nearest keeps one reading 3-5 m short to the end (1d 5:02, 1e 7:15), so a go from a red line
  10-12 m out (nearest 5.6-7.6) or from the walker's hold line placed by the nearest (8.25 by it, 12.95 m from the junction)
  sent the right's only edge standing there: the early turn-in that clipped 1e's curbs. It goes by pulse_distance now: such a
  go sends nothing, and the pulse goes rolling once that reading is inside CURB_TURN_PULSE_M. A stand whose readings agree
  within the slack is still its go's pulse (steady GPS, ped_crosswalk_right's 8.26), as is one with no furthest given."""
  # (nearest, furthest, latest) at the stand, and whether the go is the pulse: 1d 5:02's readings at the walker's hold and at
  # red lines 10 and 12 m out (the closed loop's, test_turnpilotd_curb_go_readings); the furthest 2 m long with the nearest
  # true (the cost: the readings can't tell it from the first); steady; spread inside the slack; no furthest given
  cases = (((8.25, 14.95, 10.45), False), ((5.56, 12.26, 7.76), False), ((7.56, 14.26, 9.76), False),
           ((8.26, 10.26, 8.26), False), ((8.26, 8.26, 8.26), True), ((8.26, 8.5, 8.3), True), ((8.26, None, None), True))
  for (near, far, last), pulsed in cases:
    dt_ = DesireTiming()
    v = 3.
    k_in = int(round(12. / v / DT_MDL))
    out = [dt_.update(RIGHT_TURN, near + 12. - v * k * DT_MDL, 1, False, v, 0., True, True, DT_MDL, 0.,
                      far=None if far is None else far + 12. - v * k * DT_MDL, last=None if last is None else last + 12. - v * k * DT_MDL)
           for k in range(k_in)]
    out += [dt_.update(RIGHT_TURN, near, 1, False, 0., 0., True, True, DT_MDL, 0., far=far, last=last, holding=True) for _ in range(40)]
    assert set(out) == {Desire.none} and not dt_.started, (near, far, last)
    go = dt_.update(RIGHT_TURN, near, 1, False, 0., 0., True, True, DT_MDL, 0., far=far, last=last, go=True)
    assert (go == Desire.turnRight) == pulsed, (near, far, last, go)
    if pulsed:
      continue
    # pulling away at 2 m/s: the one pulse goes rolling where its reading comes inside CURB_TURN_PULSE_M, not before
    edge, k = None, 0
    while edge is None:
      k += 1
      dd = 2. * k * DT_MDL
      pd = pulse_distance(RIGHT_TURN, near - dd, far - dd, True, last - dd)
      if dt_.update(RIGHT_TURN, near - dd, 1, False, 2., 0., True, True, DT_MDL, 0., far=far - dd, last=last - dd) == Desire.turnRight:
        edge = pd
      assert k < 400 and near - dd > 0., (near, far, last)
    assert CURB_TURN_PULSE_M - 0.15 < edge < CURB_TURN_PULSE_M, (near, far, last, edge)


def test_held_standing_the_turn_drops_to_none_so_the_go_is_an_edge_at_once():
  # a hold that has the car standing (shouldStop) drops the turn desire to none; the go is a fresh edge in its own frame,
  # however soon. Held fewer than PULSE_GAP_FRAMES frames, the drop is made up to that before the edge
  for held_frames in (40, PULSE_GAP_FRAMES, 1):
    dt_ = DesireTiming()
    step_through(dt_, [(5., 1, LEFT_TURN, 40., 5., 0., False), (0.5, 1, LEFT_TURN, 10., 0., 0., False)])
    held = [dt_.update(LEFT_TURN, 10., 1, False, 0., 0., True, True, DT_MDL, 0., holding=True) for _ in range(held_frames)]
    assert set(held) == {Desire.none}, held_frames
    out = [dt_.update(LEFT_TURN, 10., 1, False, 0., 0., True, True, DT_MDL, 0., go=k == 0) for k in range(6)]
    first = out.index(Desire.turnLeft)
    assert first == max(PULSE_GAP_FRAMES - held_frames, 0) and set(out[first:]) == {Desire.turnLeft}, held_frames


def test_entry_pulse_goes_once_the_gap_after_a_late_repulse_has_passed():
  # at 4.5 m/s the pulses go at 27 m (the far-side lead, FAR_LEAD_S out) and 11.25 m: the car crosses ENTRY_M 0.3 s after
  # that repeat, inside ENTRY_GAP_S. The entry pulse waits out the gap and then goes, so the last pulse is fresh for the
  # whole turn; it is never used up by crossing ENTRY_M while it couldn't go
  v = 4.5
  trace = drive([(200., "turn", "left"), (900., "turn", "right")], v=v, turns_at=None, seconds=60.)
  edges = rising_edges(trace, Desire.turnLeft)
  late = [d for _, d in edges if ENTRY_M < d < ENTRY_M + ENTRY_GAP_S * v]
  assert late, edges  # the case: a repeat within ENTRY_GAP_S of the entry
  entry = [d for _, d in edges if 0. < d < ENTRY_M]
  assert len(entry) == 1, edges
  assert late[-1] - entry[0] >= ENTRY_GAP_S * v - 0.5, edges


def test_a_turn_whose_first_pulse_is_its_go():
  # the autonomous unprotected left (DESIGN phase 4 section 2): no pulse on the approach (may_start false), so the car stands
  # straight at its hold line with the road camera up the oncoming approach; the go is its first pulse, at once, standing
  dt_ = DesireTiming()
  out = []
  for k in range(int(6. / DT_MDL)):
    out.append(dt_.update(LEFT_TURN, 40. - 31. * k * DT_MDL / 6., 1, False, 5., 0., True, True, DT_MDL, 0., may_start=False))
  for _ in range(int(30. / DT_MDL)):  # held at the line, waiting for a gap
    out.append(dt_.update(LEFT_TURN, 9., 1, False, 0., 0., True, True, DT_MDL, 0., may_start=False, holding=True))
  assert set(out) == {Desire.none} and not dt_.started
  assert dt_.update(LEFT_TURN, 9., 1, False, 0., 0., True, True, DT_MDL, 0., go=True) == Desire.turnLeft
  assert dt_.started and dt_.side != 0
  # and the car gets STANDSTILL_REARM_S to move off on it
  out = [dt_.update(LEFT_TURN, 9., 1, False, 0., 0., True, True, DT_MDL, 0.) for _ in range(int(1.5 / DT_MDL))]
  assert set(out) == {Desire.turnLeft}


def test_a_stall_pulse_is_one_fresh_edge_of_the_turn_in_progress_whatever_it_has_turned():
  """ALWAYS_DRIVE.md LEFT-21: turnpilotd's stall watch asks for one fresh pulse of a turn the car stands partway round (past
  STANDSTILL_REPULSE_MAX_DEG, where no standstill pulse goes): the next update sends the turn's desire as a new rising edge,
  once; a turn already done (cleared at its angle) gets none, nor one never started."""
  dt_ = DesireTiming()
  out = step_through(dt_, [(1., 1, RIGHT_TURN, 15., 7.5, 0., False),      # the approach: pulsed 8 m out
                           (1.5, 1, RIGHT_TURN, 7., 4., -0.6, False)])    # into the turn, ~50 deg round
  assert dt_.started and not dt_.done and 30. < dt_.turned_deg < 70.
  out = step_through(dt_, [(4., 1, RIGHT_TURN, 5., 0., 0., False)])      # standing in it: re-armed to none
  assert out[-1] == Desire.none
  assert dt_.stall_pulse()
  out = step_through(dt_, [(1., 1, RIGHT_TURN, 5., 0., 0., False)])
  assert out[0] == Desire.turnRight and rising_edges([(0., 0., d) for d in out], Desire.turnRight) and \
    sum(1 for a, b in zip([Desire.none] + out, out, strict=False) if b == Desire.turnRight and a != Desire.turnRight) == 1
  done = DesireTiming()
  step_through(done, [(1., 1, RIGHT_TURN, 15., 7.5, 0., False), (3., 1, RIGHT_TURN, 7., 4., -0.8, False)])
  assert done.done and not done.stall_pulse()
  assert not DesireTiming().stall_pulse()


def drive_wrong_way(wrong_way_m, speed_profile, seconds=60.):
  """drive() for a left at 200 m with an opening on its left the car may not enter wrong_way_m before it (navd's
  maneuverWrongWayM; 0 none), the car standing where speed_profile(t) is 0 and turning 8 m out; as turnpilotd, the pulse
  waits by the furthest reading of the junction (far). Returns ([(s, distance, desire)], TurnSkip)."""
  navd = FakeNavd([(200., "turn", "left", (0., 270., 2)), (900., "turn", "right")], wrong_way={0: wrong_way_m})
  rf, dt_, skip = RouteFollower(), DesireTiming(), TurnSkip()
  s, out = 0., []
  for i in range(int(seconds / DT_MDL)):
    v = speed_profile(i * DT_MDL)
    instr = navd.update(s)
    rf.update(instr, instr is not None, True, v, DT_MDL)
    yaw = v / 8. if rf.upcoming is not None and rf.distance < 8. else 0.
    state = skip.update(rf.upcoming, rf.distance, rf.maneuver_id, v, dt_.maneuver_id == rf.maneuver_id and dt_.started)
    desire = dt_.update(None if state.skipped else rf.upcoming, rf.distance, rf.maneuver_id, rf.passed, v, 90. if yaw else 0., True, True,
                        DT_MDL, yaw, may_start=state.feasible_now, far=rf.distance + rf.junction_spread)
    out.append((s, rf.distance, desire))
    s += v * DT_MDL
  return out, skip


def test_no_turn_pulse_short_of_an_opening_the_car_may_not_enter():
  """The first drive: at a red light the car planned a left onto a one-way. Valhalla's route was right (its left is at the
  far carriageway of a divided road, 8 to 20 m on), but the pulse went 40 m out and again as the car pulled away from the
  light standing at the near carriageway, a one-way toward it, and the model takes a turn desire at the first opening that
  way. With navd's maneuverWrongWayM the pulse waits until the car is WRONG_WAY_PAST_M past that opening, and the turn isn't
  skipped as too late for it (turn_skip's deadline moves in with it)."""
  from openpilot.selfdrive.turnpilot.desire_timing import WRONG_WAY_PAST_M
  plain, _ = drive_wrong_way(0., lambda t: 5.5)
  assert rising_edges(plain, Desire.turnLeft)[0][1] > turn_lead(LEFT_TURN, 5.5) - 1.5  # as before: from the lead
  gated, skip = drive_wrong_way(16., lambda t: 5.5)
  edges = rising_edges(gated, Desire.turnLeft)
  assert edges and not skip.skipped
  assert all(desire == Desire.none for s, d, desire in gated if s < 200. - 16. + WRONG_WAY_PAST_M)
  assert 16. - WRONG_WAY_PAST_M - 1. < edges[0][1] <= 16. - WRONG_WAY_PAST_M  # the first frame past it
  # standing at the opening's red light, 10 m short of it, for 20 s: no pulse while standing or pulling away, one past it
  stop_s = 200. - 16. - 10.
  red, skip = drive_wrong_way(16., lambda t: 8. if t < stop_s / 8. else 0. if t < stop_s / 8. + 20. else 4.)
  edges = rising_edges(red, Desire.turnLeft)
  assert edges and not skip.skipped
  assert all(desire == Desire.none for s, d, desire in red if s < 200. - 16. + WRONG_WAY_PAST_M)
  # the same light with no such opening: the pulse is out on the approach and again as the car pulls away
  red, _ = drive_wrong_way(0., lambda t: 8. if t < stop_s / 8. else 0. if t < stop_s / 8. + 20. else 4.)
  assert sum(1 for s, d in rising_edges(red, Desire.turnLeft) if s < 200. - 16.) >= 2


def test_a_curb_side_turn_gets_one_pulse_8_m_out_and_nothing_before_it():
  """2026-11-06, 1e after dark: the model started each right 0.3-1.5 s after every fresh rising edge, wherever the car was; the
  40 m pulse, its repeat about 20 m out and the 10 m entry pulse each started the right 11-25 m before the junction on a 12-15
  m radius and cut the inside curb (the driver took over twice, one curb clip), where the human turned in about 6.7 m out on
  an 8 m radius. A right approached from 80 m under its turn cap (3 m/s by 20 m) sends exactly one rising edge, at
  CURB_TURN_PULSE_M, and nothing before it; it is never skipped (turn_skip's deadline for it is 5 m), and its one pulse is
  held through the turn to its clear."""
  route = [(80., "turn", "right"), (900., "turn", "left")]
  trace = drive(route, speed_profile=under_the_cap(route), turns_at=6.7, steer_sign=-1., seconds=40.)
  edges = rising_edges(trace, Desire.turnRight)
  assert len(edges) == 1, edges
  s, d = edges[0]
  assert CURB_TURN_PULSE_M - 4. * DT_MDL - 0.1 < d < CURB_TURN_PULSE_M, d
  assert all(desire == Desire.none for s_, d_, desire in trace if s_ < s)
  held = [desire for s_, d_, desire in trace if s <= s_ and desire != CLEAR_DESIRE]
  assert held[:held.index(Desire.none)] == [Desire.turnRight] * held.index(Desire.none)  # one edge, held to the clear
  assert [desire for s_, d_, desire in trace].count(CLEAR_DESIRE) == CLEAR_FRAMES
  # mirrored where traffic keeps left: the left is the curb side's, the right the far side's
  left, right = LEFT_TURN, RIGHT_TURN
  assert turn_lead(left, 3., right_hand_traffic=False) == CURB_TURN_PULSE_M == turn_lead(right, 3.)
  assert turn_lead(right, 7., right_hand_traffic=False) == TURN_DESIRE_LEAD_M == turn_lead(left, 7.)
  dt_ = DesireTiming()
  out = [dt_.update(left, 20. - 3. * k * DT_MDL, 1, False, 3., 0., True, True, DT_MDL, 0., right_hand_traffic=False)
         for k in range(int(5. / DT_MDL))]
  first = out.index(Desire.turnLeft)
  assert CURB_TURN_PULSE_M - 0.2 < 20. - 3. * first * DT_MDL < CURB_TURN_PULSE_M and pulses(out, Desire.turnLeft) == 1


def first_pulse_at(maneuver, v, d0=120.):
  """m before the junction a turn's first pulse goes approached at a steady v (may_start: no skip in the way)."""
  dt_ = DesireTiming()
  for k in range(int(d0 / v / DT_MDL) + 2):
    d = d0 - v * k * DT_MDL
    if dt_.update(maneuver, d, 1, False, v, 0., True, True, DT_MDL, 0.) == Desire.turnLeft:
      return d
  return None


def test_a_far_side_turns_lead_comes_in_with_speed():
  """2026-11-06 (1e: a 40 m desire at 0.7 mph held for 10-30 s took a parking-lot opening 25 m early): a far-side turn's first
  pulse goes FAR_LEAD_S before its junction, never more than TURN_DESIRE_LEAD_M (40 m, gate 1's, at the 6.9 m/s the cap has
  the car at there) nor under FAR_LEAD_MIN_M (15 m)."""
  for v, lead in ((15., TURN_DESIRE_LEAD_M), (6.9, TURN_DESIRE_LEAD_M), (5., 30.), (4., 24.), (2., FAR_LEAD_MIN_M), (1., FAR_LEAD_MIN_M),
                  (0.4, FAR_LEAD_MIN_M)):
    assert turn_lead(LEFT_TURN, v) == lead, v
    d = first_pulse_at(LEFT_TURN, v, d0=60.)
    assert d is not None and lead - v * DT_MDL - 1e-6 <= d < lead, (v, d, lead)
  # a car crawling up from far out: no desire held 10-30 s, the first pulse 15 m out, at most FAR_LEAD_MIN_M / v before it
  d = first_pulse_at(LEFT_TURN, 0.3, d0=40.)
  assert FAR_LEAD_MIN_M - 0.1 < d < FAR_LEAD_MIN_M
  # a right is unchanged by speed: its one pulse 8 m out at any speed it can make it from
  assert all(turn_lead(RIGHT_TURN, v) == CURB_TURN_PULSE_M for v in (0., 1., 3., 5., 15.))


def approach_right(bias=None, last=True, J=200.):
  """A right J m ahead approached at 8 m/s, at its cap's 3 m/s from 40 m out, through RouteFollower, TurnSkip and DesireTiming
  as turnpilotd runs them, both given the furthest reading (far) and, if last, navd's latest confirmed
  (RouteFollower.junction_latest). bias(true m to the junction) is m added to that navd update's reading (None: true). Returns (the first pulse's true m to the
  junction or None, (skip reason, true m) or None, how many updates were biased)."""
  navd, rf, dt_, skip = FakeNavd([(J, "turn", "right", (0., 90., 2)), (900., "turn", "left")]), RouteFollower(), DesireTiming(), TurnSkip()
  s, first, skipped, biased = 0., None, None, 0
  while s < J + 5.:
    v = 8. if J - s > 40. else 3.
    instr = navd.update(s)
    off = bias(J - s) if instr is not None and bias is not None else None
    if off:
      instr, biased = nav_instruction(J - s + off, "turn", "right", junction=(0., 90., 2), remaining=900. - s), biased + 1
    rf.update(instr, instr is not None, True, v, DT_MDL)
    kw = {'far': rf.distance + rf.junction_spread, 'last': rf.junction_latest if last else None}
    state = skip.update(rf.upcoming, rf.distance, rf.maneuver_id, v, dt_.maneuver_id == rf.maneuver_id and dt_.started, **kw)
    if state.skipped and skipped is None:
      skipped = (skip.reason, J - s)
    out = dt_.update(None if state.skipped else rf.upcoming, rf.distance, rf.maneuver_id, rf.passed, v, 0., True, True, DT_MDL, 0.,
                     may_start=state.feasible_now, **kw)
    if out == Desire.turnRight and first is None:
      first = J - s
    s += v * DT_MDL
  return first, skipped, biased


def once_inside(at, off):
  """A bias for approach_right: off m on the first navd update inside at m, true otherwise."""
  done = []
  def bias(d):
    if d < at and not done:
      done.append(d)
      return off
    return None
  return bias


def test_a_curb_side_pulse_goes_by_the_furthest_reading_never_early():
  """The curb-side turn's one pulse starts the turn where the car is, so it goes by the furthest of navd's readings of the
  junction (far: RouteFollower distance + junction_spread), the nearest still short of it (desire_timing.pulse_distance). In
  1d and 1e the readings spread 0.7-6.7 m where the nearest came inside 8 m, and where the car's arc showed where the
  junction was the nearest was 3-5 m short. A far-side turn's lead is unchanged (the nearest), and with no furthest given a
  curb-side pulse goes by the nearest as before.
  One reading off short, at 60 m out or 35 (test_turnpilotd's GPS outliers), holds the nearest that far short to the
  junction (it is a minimum). 4-7 m short, the pulse goes at the true 8 m by the furthest. 20 m short (by the nearest the
  right pulsed 28 m out) or 12, the next true reading jumps up more than NEW_MANEUVER_JUMP_M from a nearest inside
  ADVANCE_NEAR_M, which RouteFollower takes for navd moving on: a new maneuver, its readings fresh, pulsed at 8 m. 9-10 m
  short, the nearest is past the junction before the furthest (or the latest) comes inside its pulse point, and no first
  pulse goes past it (the late pulse is the dangerous one): the right is skipped out loud as its nearest crosses the junction
  (turn_skip: tooLate), never pulsed early nor lost in silence. (This test's first version put its outlier in a 2 m window
  navd's 1 Hz updates at 8 m/s stepped over: it never went out, and the 20 m case it claimed was never run.)"""
  for last in (True, False):
    first, skipped, _ = approach_right(last=last)
    assert first is not None and CURB_TURN_PULSE_M - 0.5 < first <= CURB_TURN_PULSE_M + 0.05 and skipped is None, (last, first)
    for at in (61., 35.):
      for off in (-20., -12., -7., -4.):
        first, skipped, biased = approach_right(once_inside(at, off), last=last)
        assert biased == 1 and first is not None and CURB_TURN_PULSE_M - 0.5 < first <= CURB_TURN_PULSE_M + 0.05, (last, at, off, first)
        assert skipped is None, (last, at, off, skipped)
      for off in (-10., -9.):
        first, skipped, biased = approach_right(once_inside(at, off), last=last)
        assert biased == 1 and first is None, (last, at, off, first)
        assert skipped is not None and skipped[0] == 'tooLate' and -off - 1.5 < skipped[1] < -off + 0.5, (last, at, off, skipped)
  # the furthest still 8 m or more out, no latest given: no pulse whatever the nearest says (the latest: below)
  m = RIGHT_TURN
  assert DesireTiming().update(m, 3., 1, False, 3., 0., True, True, DT_MDL, 0., far=12.) == Desire.none
  assert DesireTiming().update(m, 3., 1, False, 3., 0., True, True, DT_MDL, 0., far=7.5) == Desire.turnRight
  assert DesireTiming().update(m, 7.5, 1, False, 3., 0., True, True, DT_MDL, 0.) == Desire.turnRight  # no far: the nearest
  # a far-side turn: its lead by the nearest, the furthest and the latest no matter
  assert DesireTiming().update(LEFT_TURN, 39., 1, False, 7., 0., True, True, DT_MDL, 0., far=60., last=60.) == Desire.turnLeft
  assert DesireTiming().update(LEFT_TURN, 41., 1, False, 7., 0., True, True, DT_MDL, 0., far=60., last=3.) == Desire.none


def test_a_curb_side_pulse_is_due_by_the_latest_reading_when_the_furthest_holds_long():
  """1e 8:32, a 91 deg right replayed through one follower for the whole drive: the furthest reading of its junction (from
  far out, 8-26 s old) was still 8.8 m out with the nearest at the junction, about 10 m beyond where the car's arc put it,
  so by the furthest alone the right got no pulse; navd's latest reading was 1.5 m from the arc's junction. A curb-side
  pulse is due at the latest at CURB_LATEST_PULSE_M by the latest reading (RouteFollower.junction_latest, dead-reckoned),
  whichever of the two comes first; the nearest past the junction still sends none (a late pulse turns the car into the exit
  road's far lanes)."""
  m = RIGHT_TURN
  assert DesireTiming().update(m, 3., 1, False, 3., 0., True, True, DT_MDL, 0., far=12., last=CURB_LATEST_PULSE_M - 0.5) == Desire.turnRight
  assert DesireTiming().update(m, 3., 1, False, 3., 0., True, True, DT_MDL, 0., far=12., last=CURB_LATEST_PULSE_M + 0.5) == Desire.none
  assert DesireTiming().update(m, 3., 1, False, 3., 0., True, True, DT_MDL, 0., far=7.5, last=7.5) == Desire.turnRight  # the furthest
  assert DesireTiming().update(m, -0.5, 1, False, 3., 0., True, True, DT_MDL, 0., far=6., last=2.) == Desire.none
  # readings 9 m long from 120 m out to 40 m, true from there: the furthest 9 m long to the junction
  long_far = lambda d: 9. if 40. < d < 120. else None  # noqa: E731
  first, skipped, biased = approach_right(long_far)
  assert biased and first is not None and CURB_LATEST_PULSE_M - 0.5 < first <= CURB_LATEST_PULSE_M + 0.05 and skipped is None, first
  # by the furthest alone, never: its nearest crosses the junction with no pulse, and the turn is skipped out loud there
  first, skipped, _ = approach_right(long_far, last=False)
  assert first is None and skipped is not None and skipped[0] == 'tooLate' and -0.5 < skipped[1] < 0.5, (first, skipped)
  # one reading long (the furthest 4 m long): by the furthest alone the pulse went 3.7 m out, by the latest at 6
  first, skipped, _ = approach_right(once_inside(61., 4.))
  assert first is not None and CURB_LATEST_PULSE_M - 0.5 < first <= CURB_LATEST_PULSE_M + 0.05 and skipped is None, first
  # the latest long too, 4 m, to the last few m, then jumping in past CURB_LATEST_PULSE_M (1e 8:32: 9.7 m to 5.4 m in one
  # update): the pulse goes at once there, short of turn_skip's deadline by the latest (3 m)
  for J in (200., 201., 202., 203.5):
    first, skipped, _ = approach_right(lambda d: 9. if 40. < d < 120. else (4. if 6. < d <= 40. else None), J=J)
    assert first is not None and 3. < first < CURB_LATEST_PULSE_M and skipped is None, (J, first)


def long_but_once_true(bias_m, at):
  """A bias for approach_right: every navd reading bias_m long, but for the first one inside at m, which is true."""
  done = []
  def bias(d):
    if d < at and not done:
      done.append(d)
      return None
    return bias_m
  return bias


def test_a_curb_side_turn_is_past_its_junction_by_any_one_reading_so_never_pulsed_past_it():
  """2026-11-06 review: a curb-side turn's first pulse is never let go past the junction by the NEAREST reading, one reading
  alone included, and turn_skip skips the turn out loud as it crosses. One reading 8-10 m short 20-61 m out (an outlier the
  nearest keeps to the end) and one TRUE reading inside a long bias of 8-10 m that lasts are the same readings. Judged by the
  second-nearest (that round's JunctionSpan.firm, since removed), the outlier's right pulsed at the true 8 m and the bias's at
  0.3 m past its junction (8 m) or 2.3 m past (10 m), nothing said, the late pulse being the dangerous one. By the nearest
  the outlier's right is skipped tooLate out loud 7.6-10 m short of its junction (rerouted, safe) and the bias's at its
  junction, never pulsed past it. One 11-15 m short is taken at 8 m (the next true reading's jump up from inside
  ADVANCE_NEAR_M), as is one 4-7 m short; one 9-10 m short about 20 m out, navd's latest until its next true reading, brought
  the pulse out 14.8-15.7 m from the junction before the latest had to be confirmed (route_follower LATEST_AGREE_M), and is
  in the band now: skipped out loud, never turned in early. Two readings that agree on a junction behind the car hold the
  pulse the same way."""
  for at in (61., 35., 20.):
    for off in (-15., -12., -11., -7., -4.):
      first, skipped, biased = approach_right(once_inside(at, off))
      assert biased == 1 and first is not None and CURB_TURN_PULSE_M - 0.5 < first <= CURB_TURN_PULSE_M + 0.05, (at, off, first)
      assert skipped is None, (at, off, skipped)
    for off in (-10., -9., -8.):
      first, skipped, biased = approach_right(once_inside(at, off))
      assert biased == 1 and first is None, (at, off, first)  # never early: no pulse by the outlier
      assert skipped is not None and skipped[0] == 'tooLate' and -off - 1.5 < skipped[1] < -off + 0.5, (at, off, skipped)
  # one true reading inside a lasting long bias of 8 or 10 m: no pulse past the junction, skipped out loud at it
  for bias_m in (8., 10.):
    for at in (30., 9., 6.):
      first, skipped, biased = approach_right(long_but_once_true(bias_m, at))
      assert biased > 10 and first is None, (bias_m, at, first)
      assert skipped is not None and skipped[0] == 'tooLate' and -0.5 < skipped[1] < 0.5, (bias_m, at, skipped)
  # two that agree, 10 m short 35 m out: no pulse, skipped out loud as the nearest crosses, 10 m short of the junction
  done = []
  def twice(d):
    if d < 35. and len(done) < 2:
      done.append(d)
      return -10.
    return None
  first, skipped, biased = approach_right(twice)
  assert biased == 2 and first is None and skipped is not None and skipped[0] == 'tooLate' and 9. < skipped[1] < 11., (first, skipped)
  # unit: the nearest past the junction, the furthest and the latest inside the pulse point: none
  m = RIGHT_TURN
  assert DesireTiming().update(m, -1., 1, False, 3., 0., True, True, DT_MDL, 0., far=7.5, last=7.5) == Desire.none
  assert DesireTiming().update(m, 0.5, 1, False, 3., 0., True, True, DT_MDL, 0., far=7.5, last=7.5) == Desire.turnRight
  # a far-side turn: by the nearest too
  assert DesireTiming().update(LEFT_TURN, -1., 1, False, 7., 0., True, True, DT_MDL, 0., far=12., last=12.) == Desire.none


def test_no_first_pulse_while_a_hold_has_the_car_standing():
  """A turn's first pulse never goes while a hold has the car standing (holding: turnpilotd's shouldStop): it goes on the hold's
  go, the fresh edge in the plan that lets the car go. 2026-11-06 review: a right handed back by the stop-sign decider 7.25 m
  from its junction, its one 8 m pulse due where the car stood, got a fresh turnRight on the handback frame with the car held
  standing (the decider's 'ladder' state no longer holding the pulse), into the model's 6.6 s queue on a turn just handed back.
  A queue's stand with no hold of TurnPilot's still pulses standing (the model doesn't pull away on its own without the edge)."""
  m = RIGHT_TURN
  dt_ = DesireTiming()
  for _ in range(int(3. / DT_MDL)):
    assert dt_.update(m, 7.25, 1, False, 0., 0., True, True, DT_MDL, 0., holding=True, far=7.25, last=7.25) == Desire.none
  assert not dt_.started
  assert dt_.update(m, 7.25, 1, False, 0., 0., True, True, DT_MDL, 0., go=True, far=7.25, last=7.25) == Desire.turnRight
  # standing with no hold (a queue): the pulse goes
  assert DesireTiming().update(m, 7.25, 1, False, 0., 0., True, True, DT_MDL, 0., far=7.25, last=7.25) == Desire.turnRight
  # a far-side turn held standing inside its lead: none either, until the go
  dt_ = DesireTiming()
  assert dt_.update(LEFT_TURN, 12., 1, False, 0., 0., True, True, DT_MDL, 0., holding=True) == Desire.none
  assert dt_.update(LEFT_TURN, 12., 1, False, 0., 0., True, True, DT_MDL, 0., go=True) == Desire.turnLeft


def test_a_persistent_short_bias_brings_a_curb_side_pulse_in_by_the_bias_less_two_m():
  """The trade the latest reading's deadline makes: navd reading the junction 4 m short over the last 30 m (the latest with
  the nearest, the furthest from before it) is told apart by nothing from the furthest 4 m long (above), so the pulse goes by
  the latest, CURB_LATEST_PULSE_M by it, 4 - (CURB_TURN_PULSE_M - CURB_LATEST_PULSE_M) m early; by the furthest alone it would
  go at the true 8 m. On the road (1d, 1e) the latest was the reading that was right (module docstring)."""
  short = lambda d: -4. if d < 30. else None  # noqa: E731
  first, skipped, biased = approach_right(short)
  early = 4. - (CURB_TURN_PULSE_M - CURB_LATEST_PULSE_M)
  assert biased and first is not None and abs(first - (CURB_TURN_PULSE_M + early)) < 0.5 and skipped is None, first
  first, skipped, _ = approach_right(short, last=False)
  assert first is not None and abs(first - CURB_TURN_PULSE_M) < 0.5 and skipped is None, first


def reengaged_at(maneuver, off_at, back_at, v=3., v_back=None, far=None):
  """maneuver from 20 m at v (its first pulse on its lead), openpilot not steering from off_at m to back_at m (the car still
  rolling at v), then steering again at v_back (default v) for 2 s. far: the furthest reading, distance plus this, for a
  curb-side turn's pulse (None: the nearest). Returns (the desires from the re-engage, the DesireTiming)."""
  dt_, d, out = DesireTiming(), 20., []
  off = far if far is not None else 0.
  while d > back_at:
    lat = not (back_at < d <= off_at)
    out.append(dt_.update(maneuver, d, 1, False, v, 0., True, True, DT_MDL, 0., lat_active=lat, far=d + off))
    d -= v * DT_MDL
  assert dt_.started, ("its first pulse went before openpilot stopped steering", off_at, back_at, far)
  vb = v if v_back is None else v_back
  back = []
  for _ in range(int(2. / DT_MDL)):
    back.append(dt_.update(maneuver, d, 1, False, vb, 0., True, True, DT_MDL, 0., far=d + off))
    d -= vb * DT_MDL
  return back, dt_


def test_a_curb_side_turns_reengage_repeat_is_bounded_as_its_first_pulse_is():
  """2026-11-06 review (turnpilotd-followups' open item): a started right whose openpilot steering went off and came back, with
  no handback and no take-up, was pulsed afresh wherever it came back under TURN_STARTED_DEG: 1.95 m and 0.45 m out, a fresh
  edge the model starts the turn 0.3-1.5 s after, into the exit road's far lanes. Its re-engage repeat is bounded as its first
  pulse: rolling, never past CURB_FIRST_PULSE_M by the reading it goes by, never once the nearest has the junction at or behind
  the car (standing too); refused, none goes and repeat_refused says it (turnpilotd skips it out loud). Inside the bound it goes
  as before, and standing inside the deadline (a stand, the model holding the car) it goes as any standing pulse."""
  from openpilot.selfdrive.turnpilot.desire_timing import CURB_FIRST_PULSE_M
  for back_at in (4., 1.95, 0.45, -0.5):
    back, dt_ = reengaged_at(RIGHT_TURN, 6., back_at)
    assert Desire.turnRight not in back and dt_.repeat_refused, back_at
  back, dt_ = reengaged_at(RIGHT_TURN, 7., CURB_FIRST_PULSE_M + 0.5)
  assert back[0] == Desire.turnRight and not dt_.repeat_refused
  # by the furthest reading: 4 m by the nearest with the furthest 2 m further is still short of the deadline by it
  back, dt_ = reengaged_at(RIGHT_TURN, 5.5, 4., far=2.)
  assert back[0] == Desire.turnRight and not dt_.repeat_refused
  # standing inside the deadline it goes; standing past the junction it never does
  back, dt_ = reengaged_at(RIGHT_TURN, 6., 3., v_back=0.)
  assert back[0] == Desire.turnRight and not dt_.repeat_refused
  back, dt_ = reengaged_at(RIGHT_TURN, 6., -0.5, v_back=0.)
  assert Desire.turnRight not in back and dt_.repeat_refused
  # a far-side turn's re-engage is as before (its arc runs past the node): it pulses inside 5 m and past the junction
  for back_at in (4., -2.):
    back, dt_ = reengaged_at(LEFT_TURN, 6., back_at)
    assert back[0] == Desire.turnLeft and not dt_.repeat_refused, back_at


def blipped(maneuver, at, frames, v=3., yaw_from=None, fault=False, then=5., stand_s=0.):
  """maneuver from 20 m at v (its first pulse on its lead), openpilot not steering for frames frames from at m (fault:
  turnpilotd faulting instead, hold_off and track_yaw as its fault frames do), the car yawing toward the turn at an 8 m radius
  from yaw_from m (None: straight on), then stand_s s standing where it is and then s more at v. Returns (the desires from the
  blip on, the DesireTiming)."""
  dt_, d, out = DesireTiming(), 20., []

  def yaw(d, v):
    return maneuver.side * v / 8. if yaw_from is not None and d <= yaw_from else 0.
  while d > at:
    dt_.update(maneuver, d, 1, False, v, 0., True, True, DT_MDL, yaw(d, v))
    d -= v * DT_MDL
  assert dt_.started and dt_.active, "its first pulse went before the blip"
  for _ in range(frames):
    if fault:
      dt_.hold_off()
      dt_.track_yaw(yaw(d, v), DT_MDL)
    else:
      out.append(dt_.update(maneuver, d, 1, False, v, 0., True, True, DT_MDL, yaw(d, v), lat_active=False))
    d -= v * DT_MDL
  for _ in range(int(stand_s / DT_MDL)):
    out.append(dt_.update(maneuver, d, 1, False, 0., 0., True, True, DT_MDL, 0.))
  for _ in range(int(then / DT_MDL)):
    out.append(dt_.update(maneuver, d, 1, False, v, 0., True, True, DT_MDL, yaw(d, v)))
    d -= v * DT_MDL
  return out, dt_


def test_a_blip_in_openpilots_steering_inside_a_curb_side_turns_deadline_leaves_its_first_pulse_to_drive_it():
  """The 2026-11-06 review of the re-engage bound above: it refused, and turnpilotd skipped out loud with its pulse cleared from
  the model's queue, on any frame openpilot wasn't steering, so one fault frame (turnpilotd's hold_off) or one latActive frame
  lost 2.5-4.5 m out skipped a right the car was 0-12.9 degrees into on its pulse: 'Too Fast to Turn' and CLEAR_DESIRE
  mid-entry, its crosswalk hold gone with the maneuver. A blip (under REPEAT_BLIP_FRAMES), or a longer gap the car comes out
  of following the first pulse round (OWN_FOLLOW_DEG or more), is spared: none goes (no fresh edge inside the deadline), and
  repeat_refused stays off, the first pulse in the model's queue driving the turn on, cleared at its angle as with no gap. A
  gap of REPEAT_BLIP_FRAMES with the car straight is the bound's own case, refused. Spared, the turn is any turn under way: a
  standstill re-arms it, and it pulses afresh as the car pulls away."""
  from openpilot.selfdrive.turnpilot.desire_timing import OWN_FOLLOW_DEG, REPEAT_BLIP_FRAMES
  for fault in (False, True):
    for yaw_from in (6., 4., None):
      for at in (4.5, 3.5, 2.5):
        for frames in (1, 3, REPEAT_BLIP_FRAMES - 1):
          case = (fault, yaw_from, at, frames)
          out, dt_ = blipped(RIGHT_TURN, at, frames, yaw_from=yaw_from, fault=fault)
          assert Desire.turnRight not in out and not dt_.repeat_refused, case
          if yaw_from is not None:
            assert CLEAR_DESIRE in out and dt_.done, case  # cleared at its angle, the turn taken on its first pulse
          else:
            assert CLEAR_DESIRE not in out and not dt_.done, case  # nothing cleared: the first pulse stays in the queue
  # a longer gap, the car following the first pulse round through it: spared too
  # (from 5 m for REPEAT_BLIP_FRAMES + 1 frames, the car yawing from 5.5: back 11 degrees round, under TURN_STARTED_DEG)
  out, dt_ = blipped(RIGHT_TURN, 5., REPEAT_BLIP_FRAMES + 1, yaw_from=5.5)
  assert Desire.turnRight not in out and not dt_.repeat_refused and CLEAR_DESIRE in out
  # ...but back under OWN_FOLLOW_DEG round (yawing from 4 m: 2.1 degrees), it is refused, and stays refused as the car
  # turns on past OWN_FOLLOW_DEG after (turnpilotd skips it out loud on the frame it is refused)
  from unittest import mock
  with mock.patch('openpilot.selfdrive.turnpilot.desire_timing.cloudlog') as log:
    out, dt_ = blipped(RIGHT_TURN, 5., REPEAT_BLIP_FRAMES + 1, yaw_from=4.)
  events = [c.args[0] for c in log.event.call_args_list]
  assert Desire.turnRight not in out and dt_.repeat_refused and dt_.turned_deg > OWN_FOLLOW_DEG
  assert 'turnpilot.curb_repeat_refused' in events and 'turnpilot.curb_repeat_spared' not in events, events
  # a gap of REPEAT_BLIP_FRAMES with the car straight on, back inside the deadline: refused (turnpilotd says it)
  out, dt_ = blipped(RIGHT_TURN, 4.5, REPEAT_BLIP_FRAMES)
  assert Desire.turnRight not in out and dt_.repeat_refused
  # spared, then standing past STANDSTILL_REARM_S: the fresh pulse goes as the car pulls away, as for any turn under way
  out, dt_ = blipped(RIGHT_TURN, 4.5, 1, stand_s=STANDSTILL_REARM_S + 1., then=1.)
  assert Desire.turnRight in out and not dt_.repeat_refused
