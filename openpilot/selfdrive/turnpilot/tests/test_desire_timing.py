from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.desire_timing import (CLEAR_FRAMES, CLEAR_MARGIN_DEG, CLEAR_MIN_DEG, CLEAR_WAIT_S, ENTRY_GAP_S, ENTRY_M,
                                                         PULSE_GAP_FRAMES, REPULSE_S, STANDSTILL_REARM_S, TURN_DESIRE_LEAD_M,
                                                         DesireTiming, keep_lead)
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


def rising_edges(trace, desire):
  prev, edges = Desire.none, []
  for s, d, cur in trace:
    if cur == desire and prev != desire:
      edges.append((s, d))
    prev = cur
  return edges


def test_left_turn_desire_from_the_lead():
  # at 5.5 m/s, about what the turn cap leaves the car at by then (at 9 m/s the entry pulse waits: turn_skip.py)
  v = 5.5
  trace = drive([(200., "turn", "left"), (900., "turn", "right")], v=v)
  edges = rising_edges(trace, Desire.turnLeft)
  assert len(edges) == 3 and 0. < edges[-1][1] < ENTRY_M  # the pulse at the lead, a repeat, and one for the turn itself
  s, d = edges[0]
  assert TURN_DESIRE_LEAD_M - v * DT_MDL - 1. < d < TURN_DESIRE_LEAD_M  # within a frame of the lead, dead-reckoned
  assert all(desire == Desire.none for s, d, desire in trace if s < 200. - TURN_DESIRE_LEAD_M - 1.)
  # held into the junction (the car turns 8 m out, at 64 deg/s), cleared once it has turned 80 degrees
  held = [desire for s, d, desire in trace if 200. - TURN_DESIRE_LEAD_M + 1. < s < 202.]
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
  trace = drive([(200., "turn", "left"), (320., "turn", "right"), (2000., "turn", "left")], v=6., seconds=70.)
  left, right = rising_edges(trace, Desire.turnLeft), rising_edges(trace, Desire.turnRight)
  assert 2 <= len(left) <= 3 and 2 <= len(right) <= 3  # 40 m at 6 m/s is more than REPULSE_S: repeats until turned
  assert 320. - TURN_DESIRE_LEAD_M - 1. < right[0][0] < 320. - TURN_DESIRE_LEAD_M + 1.
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


def test_turn_desire_pulses_again_until_through_the_junction():
  # the model forgets a pulse after 132 frames (6.6 s); a slow approach gets a new one every REPULSE_S up to the junction
  trace = drive([(200., "turn", "right"), (900., "turn", "left")], v=3., turns_at=None, seconds=80.)
  edges = rising_edges(trace, Desire.turnRight)
  assert len(edges) == 4 and 0. < edges[-1][1] < ENTRY_M  # every REPULSE_S on the approach, and one at the entry
  for (s0, _), (s1, _) in zip(edges[:-2], edges[1:-1], strict=True):
    assert abs((s1 - s0) / 3. - REPULSE_S) < 0.3  # plus the gap
  # the drop between two pulses is long enough for modeld to see it
  gap = [desire for s, d, desire in trace if edges[0][0] < s < edges[1][0] and desire == Desire.none]
  assert len(gap) == PULSE_GAP_FRAMES


def test_repeat_pulses_stop_once_the_car_has_turned():
  # turning at 3 m/s on an 8 m radius from 20 m out: 60 degrees take 2.8 s, after which a pulse would turn the car a
  # second time; before that, a slow turn still needs them
  trace = drive([(200., "turn", "right"), (900., "turn", "left")], v=3., turns_at=20., seconds=80., steer_sign=-1.)
  edges = rising_edges(trace, Desire.turnRight)
  assert len(edges) >= 2 and all(d > 20. - 2.8 * 3. for _, d in edges)


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
  # turning at 4 m/s on an 8 m radius from 20 m out: the clear goes out at 80 degrees, before navd moves on
  trace = drive([(200., "turn", "right"), (900., "turn", "left")], v=4., turns_at=20., seconds=60., steer_sign=-1.)
  desires = [desire for s, d, desire in trace]
  assert desires.count(CLEAR_DESIRE) == CLEAR_FRAMES
  i = desires.index(CLEAR_DESIRE)
  s, d, _ = trace[i]
  deg = (20. - d) / 8. * 57.3  # turned since 20 m out
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
  # navd moves on with the car 20 degrees round the left, and the next turn is 30 m on: its pulse goes out at once,
  # and the rest of the left doesn't clear it
  out = step_through(DesireTiming(), [(10., 1, LEFT_TURN, 40., 4., 0., False),
                                      (20. / 17.2, 1, LEFT_TURN, 0., 4., 0.3, False),
                                      (3., 2, RIGHT_TURN, 30., 4., 0.3, True)])
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
  out = engage_through(DesireTiming(), [(1., LEFT_TURN, 40., 4., 0., True), (1., LEFT_TURN, 36., 4., 0., False),
                                        (1., LEFT_TURN, 32., 4., 0., True)])
  assert pulses(out, Desire.turnLeft) == 2 and out[20:40] == [Desire.none] * 20 and out[40] == Desire.turnLeft


def test_no_pulse_on_engaging_in_the_turn():
  # the driver took over 15 m out and turned the car 30 degrees by hand before engaging again: a pulse now would turn it
  # on past the exit road
  yaw = 0.3  # rad/s, 17 deg/s
  out = engage_through(DesireTiming(), [(25. / 4., LEFT_TURN, 40., 4., 0., True), (30. / 17.2, LEFT_TURN, 15., 4., yaw, False),
                                        (1., LEFT_TURN, 8., 4., yaw, True)])
  assert pulses(out, Desire.turnLeft) == 2  # the one at the lead and a repeat on the REPULSE_S clock, both before the takeover
  assert Desire.turnLeft not in out[int(round(25. / 4. / DT_MDL)):]


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


def glitch_through(mode):
  """A committed right, desire out at 40 m, braking to 6 m/s; from 14 m out navd's next three updates are a glitch
  (mode), as turnpilotd runs RouteFollower, TurnSkip and DesireTiming. Returns [(distance, desire, id, skipped)]."""
  rf, dt_, skip = RouteFollower(), DesireTiming(), TurnSkip()
  right = (0., 90., 2)
  dist, v, rc, k, out = 80., 8., 0, 0, []
  for i in range(600):
    v = max(6., v - DT_MDL)
    dist -= v * DT_MDL
    updated = i % 20 == 0
    reported, valid, mtype, modifier = dist, True, "turn", "right"
    if updated and dist < 14. and k < 3:
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
  # navd off the step for three updates, a reroute onto the same right from 25 m off, a reroute starting past it, a 60 m
  # GPS jump 12 m out: the committed right is never cleared, skipped or pulsed again under a new id before it is over
  for mode in ('invalid', 'reroute_far', 'reroute_past', 'jump_close'):
    out = glitch_through(mode)
    first = next(mid for d, desire, mid, _ in out if desire == Desire.turnRight)
    before = [(d, desire, mid, skipped) for d, desire, mid, skipped in out if d > -PASSED_M]
    assert all(mid == first and not skipped and desire != CLEAR_DESIRE for d, desire, mid, skipped in before if d < 40.), mode


def test_go_at_standstill_pulses_now():
  # held at the line past the model's queue (a walker, a stop sign), and the hold lets go: the turn pulses at once, still
  # standing, not once the car moves (the model won't pull away from a standstill with its pulse forgotten)
  dt_ = DesireTiming()
  step_through(dt_, [(5., 1, RIGHT_TURN, 40., 5., 0., False), (STANDSTILL_REARM_S + 3., 1, RIGHT_TURN, 15., 0., 0., False)])
  assert dt_.update(RIGHT_TURN, 15., 1, False, 0., 0., True, True, DT_MDL, 0., go=True) == Desire.turnRight
  # and the car gets STANDSTILL_REARM_S to move off on it before it re-arms
  out = [dt_.update(RIGHT_TURN, 15., 1, False, 0., 0., True, True, DT_MDL, 0.) for _ in range(int(1.5 / DT_MDL))]
  assert set(out) == {Desire.turnRight}
  # a go soon after a pulse (stopped under STANDSTILL_REARM_S): a repeat pulse, through a short gap
  dt_ = DesireTiming()
  step_through(dt_, [(5., 1, RIGHT_TURN, 40., 5., 0., False), (1., 1, RIGHT_TURN, 15., 0., 0., False)])
  out = [dt_.update(RIGHT_TURN, 15., 1, False, 0., 0., True, True, DT_MDL, 0., go=k == 0) for k in range(10)]
  assert out[:PULSE_GAP_FRAMES] == [Desire.none] * PULSE_GAP_FRAMES and out[PULSE_GAP_FRAMES] == Desire.turnRight
  # 40 degrees into the turn: no pulse on a go either (it turns the car on past the exit road)
  dt_ = DesireTiming()
  step_through(dt_, [(9., 1, LEFT_TURN, 45., 5., 0., False), (40. / 28.65, 1, LEFT_TURN, 0., 4., 0.5, False),
                     (STANDSTILL_REARM_S + 1., 1, LEFT_TURN, -2., 0., 0., False)])
  assert dt_.update(LEFT_TURN, -2., 1, False, 0., 0., True, True, DT_MDL, 0., go=True) == Desire.none


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
  # at 4 m/s the pulses go at 40, 26 and 12 m: the car crosses ENTRY_M 0.5 s after that last repeat, inside ENTRY_GAP_S.
  # The entry pulse waits out the gap and then goes, so the last pulse is fresh for the whole turn; it is never used up
  # by crossing ENTRY_M while it couldn't go
  v = 4.
  trace = drive([(200., "turn", "left"), (900., "turn", "right")], v=v, turns_at=None, seconds=60.)
  edges = rising_edges(trace, Desire.turnLeft)
  late = [d for _, d in edges if ENTRY_M < d < ENTRY_M + ENTRY_GAP_S * v]
  assert late, edges  # the case: a repeat within ENTRY_GAP_S of the entry
  entry = [d for _, d in edges if 0. < d < ENTRY_M]
  assert len(entry) == 1, edges
  assert late[-1] - entry[0] >= ENTRY_GAP_S * v - 0.5, edges
