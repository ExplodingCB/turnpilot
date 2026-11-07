"""turnpilotd's curb-side turns after the rights' one 8 m pulse (desire_timing.CURB_TURN_PULSE_M, 2026-11-06), closed loop on
test_turnpilotd's Sim: a go from a stand further out than the pulse is the turn's go all the same (watch_curb_go: go_id, the
post-go profile, the miss at the pass), a dropped pulse is taken up again at the turn's own deadline (_take_up_dropped), a
turn taken up at a signal it can no longer stop for is skipped out loud, and under left-hand traffic the left is the
curb-side turn and the right the unprotected one (right_hand_traffic handed to the follower and DesireTiming as to TurnSkip)."""
from unittest import mock

import pytest

from openpilot.cereal import log, messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import turnpilotd
from openpilot.selfdrive.turnpilot.desire_timing import CURB_GO_SLACK_M, CURB_TURN_PULSE_M, DesireTiming
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE
from openpilot.selfdrive.turnpilot.route_follower import RouteFollower
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import RIGHT_AT_A_CORNER, T_LEFT, Sim, oracle_lights, rising, stop_at
from openpilot.selfdrive.turnpilot.turn_skip import CURB_FIRST_PULSE_M, STOP_BEFORE_TURN_M
from openpilot.selfdrive.turnpilot.turn_speed import NO_SPEED_CAP, POST_GO_PROFILES, post_go_caps
from openpilot.selfdrive.turnpilot.turnpilotd import TURN_MISSED_REASON

Desire = log.Desire
JUNCTION = 300.


def events(event, name):
  return [c.kwargs for c in event.call_args_list if c.args[0] == name]


def held_at_a_red(s_line, cruise=13.):
  """RIGHT_AT_A_CORNER with a mapped signal at s_line, read red in the sim's light oracle: the car brought to rest at it. Returns
  the sim and the light, whose 'state' the test turns green."""
  sim = Sim(RIGHT_AT_A_CORNER, cruise=cruise, controls=[(s_line, 'signal')])
  sim.go_taps = False
  sim.tp.oracle_ok = True
  light = {'state': 'red'}

  def lights(sim):
    msg = messaging.new_message('turnpilotLights')
    msg.turnpilotLights.source = 'simOracle'
    msg.turnpilotLights.lights = [{'state': light['state'], 'distance': s_line - sim.s, 'instance': 'trafficLight 1'}]
    return msg.as_reader().turnpilotLights
  sim.lights = lights
  stop_at(sim, s_line)
  sim.run(1.)
  return sim, light


def from_a_red(s_line, turn=True, blip=False, stop_again_at=None):
  """A right from a red at s_line: green, then engaged through the junction. turn: the model takes it (an 8 m radius once its
  desire is out); else straight on. blip: the model steers toward it at 0.02 1/m for 0.5 s after the pulse and gives up (R8b's
  curvature bound broken, so the go's own miss is what says it). stop_again_at: the model brings the car to rest there (no
  TurnPilot hold) for 2 s, then drives on. Returns (sim, plans, the go's index, the pulse's index, [(turned deg, post-go on)])."""
  sim, light = held_at_a_red(s_line)
  assert not sim.tp.desires.started, "no desire standing at the red"
  light['state'] = 'green'
  plans, state, pulse, stopped_t = [], [], None, 0.
  while sim.s < JUNCTION + 45.:
    d = sim.tp.desires
    if pulse is not None and blip and len(plans) - pulse < int(0.5 / DT_MDL):
      sim.model_kw = {'curvature': 0.02}
    else:
      sim.model_kw = {}
    if stop_again_at is not None and sim.s >= stop_again_at and stopped_t < 2.:
      sim.v, stopped_t = 0., stopped_t + DT_MDL
    turning = turn and d.started and not d.done and sim.v > 0.3
    plans.append(sim.step(engaged=True, yaw_rate=-sim.v / 8. if turning else 0.))
    if pulse is None and plans[-1].desire == Desire.turnRight:
      pulse = len(plans) - 1
    state.append((sim.tp.desires.turned_deg, sim.tp.post_go))
    assert sim.t < 120.
  go = next(i for i, p in enumerate(plans) if not p.shouldStop)
  return sim, plans, go, pulse, state


def test_a_right_from_a_red_at_a_12_m_line_the_model_doesnt_take_is_missed_out_loud():
  """The go from a red 12 m out sends no desire (its one pulse waits for 8 m: desire_timing), so turn_go was false and go_id
  never armed: check_missed never said 'Turn Missed' for a right the model didn't take (2026-11-06 review). Its first pulse
  arms the go's bookkeeping now, so it is said at the pass, as for a go with its pulse: here with the model having steered
  toward it for half a second (R8b's straight-drive watch broken by its curvature), so only the go's miss can say it."""
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    sim, plans, go, pulse, _ = from_a_red(288., turn=False, blip=True)
  assert sim.trace[0][0] < 100. and plans[go].maneuverDistance > CURB_TURN_PULSE_M + CURB_GO_SLACK_M, "a line 9 m+ out"
  assert plans[go].desire == Desire.none and pulse is not None and pulse > go, "no desire at the go, the pulse after it"
  assert CURB_FIRST_PULSE_M < plans[pulse].maneuverDistance <= CURB_TURN_PULSE_M + 0.5
  armed, = events(event, 'turnpilot.curb_go_armed')
  assert armed['go_distance'] == pytest.approx(12.25, abs=0.3) and armed['distance'] <= CURB_TURN_PULSE_M + 0.5
  assert events(event, 'turnpilot.curb_go')[0]['distance'] == pytest.approx(12.25, abs=0.3)
  missed = [i for i, p in enumerate(plans) if p.turnMissed]
  assert missed and plans[missed[0]].maneuverDistance < -10., "said at the pass, never before it"
  assert all(plans[i].reason == TURN_MISSED_REASON for i in missed) and 2.5 <= len(missed) * DT_MDL <= 3.5
  assert events(event, 'turnpilot_turn_missed') and not any(p.tooFastToTurn for p in plans)
  # without the curvature blip R8b's watch says it too, and only once
  sim, plans, go, pulse, _ = from_a_red(288., turn=False)
  assert 2.5 <= sum(p.turnMissed for p in plans) * DT_MDL <= 3.5


def test_a_right_from_a_red_at_a_12_m_line_taken_gets_the_post_go_profile_of_a_go_at_its_line():
  """Taken, the right from the 12 m line is driven under the post-go profile from its first pulse on (graduated: 1.0 m/s^2 and
  3.0 m/s to 15 deg, 4.3 by 45), as the same right from a line within CURB_GO_SLACK_M of its pulse point (its go is its pulse:
  turn_go) is from its go: before 2026-11-06's arming it had none (no accel cap at all), and its stall watch after a go with it."""
  profile = POST_GO_PROFILES['graduated']
  runs = {}
  for s_line in (288., JUNCTION - CURB_TURN_PULSE_M - 0.5):  # 12 m out, and a line the go pulses at (8.5 m, standing 8.75 m)
    sim, plans, go, pulse, state = from_a_red(s_line)
    assert pulse is not None and max(t for t, _ in state) > 75., s_line
    on = [i for i, (_, pg) in enumerate(state) if pg is not None]
    assert on and on[0] <= pulse + 1, (s_line, on[:1], pulse)
    # under the profile's accel cap and its speed ramp by the deg turned, from the pulse until it ends
    capped = [(state[i][0], plans[i]) for i in on if i > pulse and state[i][0] < 40.]
    assert capped, s_line
    for turned, p in capped:
      accel, speed = post_go_caps(profile, turned, NO_SPEED_CAP)
      assert p.accelCap <= accel + 1e-6 and p.speedCap <= speed + 1e-3, (s_line, turned, p.accelCap, p.speedCap)
    assert not any(p.turnMissed or p.tooFastToTurn for p in plans), s_line
    runs[s_line] = [round(p.accelCap, 2) for _, p in capped]
  assert all(a == profile.accel_cap for a in runs[288.]), "the 12 m line's right is under the same 1.0 m/s^2 as the go at its line"


def test_a_go_further_out_than_the_turns_own_stop_or_stood_again_on_the_way_is_not_the_turns():
  """Bounded: a go from a stand more than STOP_BEFORE_TURN_M short of the junction (a signal there is its own, a straight one)
  is not the turn's go, nor a go the model brought the car to rest from again short of the pulse (its next go is a new one): the
  pulse then is an approach's, armed nothing (R8b's watch is what checks it)."""
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    sim, plans, go, pulse, state = from_a_red(JUNCTION - STOP_BEFORE_TURN_M - 10., turn=False)
  assert pulse is not None and not events(event, 'turnpilot.curb_go') and not events(event, 'turnpilot.curb_go_armed')
  assert all(pg is None for _, pg in state[pulse:])
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    sim, plans, go, pulse, state = from_a_red(288., turn=False, stop_again_at=290.)
  assert events(event, 'turnpilot.curb_go') and not events(event, 'turnpilot.curb_go_armed'), "stood again on the way: spent"
  assert pulse is not None and all(pg is None for _, pg in state[pulse:])


def right_dropped_at(take_up_m, outage_s=2., steers_at=None):
  """RIGHT_AT_A_CORNER approached under the cap; its one 8 m pulse out, the car crawls at 1 m/s and the detector stops with
  openpilot off steering (lat_active false) until the car is take_up_m from the junction, then back: TurnPilot takes the
  dropped right up there (_take_up_dropped). steers_at: openpilot steers again only once the car is that far from the
  junction (None: with the detector back). Returns (sim, the plans from the take-up)."""
  sim = Sim(RIGHT_AT_A_CORNER, cruise=13.)
  while sim.tp.desires.pulses == 0:
    sim.step(engaged=True)
    assert sim.t < 120.
  assert sim.trace[-1][2].desire == Desire.turnRight and sim.distance() <= CURB_TURN_PULSE_M + 0.5
  sim.cruise = 1.
  out_t = 0.
  while sim.distance() > take_up_m or out_t < outage_s:
    sim.v = min(sim.v, 1.)
    sim.step(available=False, lat_active=False, engaged=True)
    out_t += DT_MDL
    assert sim.t < 120.
  assert any(p.handback for _, _, p in sim.trace[-int(outage_s / DT_MDL):]), "handed back in the outage"
  back = len(sim.trace)
  while steers_at is not None and sim.distance() > steers_at:
    sim.v = min(sim.v, 1.)
    sim.step(lat_active=False, engaged=True)
    assert sim.t < 200.
  for _ in range(int(3. / DT_MDL)):
    sim.step(engaged=True)
  return sim, [p for _, _, p in sim.trace[back:]]


def test_a_curb_side_turns_dropped_pulse_is_taken_up_again_short_of_its_own_deadline():
  """A right whose one 8 m pulse was dropped (handed back, openpilot off steering) gets a fresh pulse taken up still short of
  its own pulse's deadline (turn_skip.CURB_FIRST_PULSE_M, 5 m, by the reading the pulse goes by), never only from ENTRY_M
  out: so a dropped 8 m right was always skipped (desireDropped). Inside the deadline it is skipped out loud as before."""
  sim, after = right_dropped_at(6.5)
  assert rising([(0., 0., p) for p in after], 'desire', Desire.turnRight), "a fresh pulse"
  assert not any(p.tooFastToTurn for p in after) and sim.tp.skip.reason == ''
  sim, after = right_dropped_at(3.5)
  assert any(p.tooFastToTurn for p in after[:3]) and sim.tp.skip.reason == 'desireDropped'
  assert not rising([(0., 0., p) for p in after], 'desire', Desire.turnRight)
  assert any(p.desire == CLEAR_DESIRE for p in after[:3]), "its pulse cleared from the model's queue"


def test_a_dropped_pulse_taken_up_while_openpilot_isnt_steering_is_skipped_never_pulsed_late():
  """Review 2026-11-06: the take-up's room is judged at the take-up, but the fresh pulse goes when openpilot steers again,
  and nothing bounds that re-engage repeat but TURN_STARTED_DEG. Taken up at 6.5 m with openpilot steering again only at 4.0,
  2.0 or 0.5 m, a fresh turnRight went at 3.95, 1.95 and 0.45 m, with nothing said: the late pulse into the exit road's far
  lanes the 5 m deadline is for. Taken up with openpilot not steering it is skipped there, out loud, its pulse cleared from
  the model's queue and its cap kept, as every dropped right was before (desireDropped); with openpilot steering at the take-up
  it is pulsed again (test above)."""
  for steers_at in (4.0, 2.0, 0.5):
    sim, after = right_dropped_at(6.5, steers_at=steers_at)
    assert any(p.tooFastToTurn for p in after[:3]) and sim.tp.skip.reason == 'desireDropped', steers_at
    assert not rising([(0., 0., p) for p in after], 'desire', Desire.turnRight), f"a late pulse, openpilot back at {steers_at} m"
    assert any(p.desire == CLEAR_DESIRE for p in after[:3]), "its pulse cleared from the model's queue"
    assert all(p.speedCap < NO_SPEED_CAP for p in after), "its cap kept to the pass"


def right_at_a_signal_taken_up(s_line, back_s, green=False, cruise=10., known_from=None):
  """RIGHT_AT_A_CORNER with a mapped signal at s_line: TurnPilot hands back 150 m out (the detector stops, openpilot still
  steering) and acts again with the car at back_s. green: the oracle reads it green. known_from: navd sends the signal only
  once the car is there (a tile loaded late: first known in the handback; FakeNavd's next instruction forced that frame, off
  its 1 Hz schedule, so it arrives there), None from the start. Returns (sim, the plans from the take-up, the cloudlog events)."""
  sim = Sim(RIGHT_AT_A_CORNER, cruise=cruise, controls=[(s_line, 'signal')] if known_from is None else [])
  sim.go_taps = False
  if green:
    sim.tp.oracle_ok = True
    sim.lights = oracle_lights('green', s_line)
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    while sim.s < JUNCTION - 150.:
      sim.step(engaged=True)
    while sim.s < back_s:
      if known_from is not None and not sim.navd.controls and sim.s >= known_from:
        sim.navd.controls, sim.navd.frame = [(s_line, 'signal')], 0  # navd's next instruction, this frame, carries it
      sim.step(available=False, engaged=True)
      assert sim.t < 120.
    assert not sim.tp.acting
    back = len(sim.trace)
    while sim.s < JUNCTION + 20. and sim.t < 120.:
      sim.step(engaged=True)
  return sim, [p for _, _, p in sim.trace[back:]], event


def test_a_turn_taken_up_at_a_signal_it_can_no_longer_stop_for_is_skipped_out_loud():
  """A right taken up after a handback at a signal 25 m short of its junction, its line 22 m ahead at 10 m/s (a stop there
  needs -2.3), the signal first known in the handback already too close to stop for at -2.0 (map_controls refuses to mark it:
  signal_turn_late), and the right makeable (47 m at 10 m/s): it was turned through a light nobody read with nothing said. Now
  it is skipped out loud (signalStop, "Can't Stop for the Light", navd reroutes), as an unprotected left too close for its line is
  (unprotectedStop): no turn desire, the car on through the junction on the model. A signal known all along keeps its stop
  through the handback (update()'s junction: a handback doesn't take the turn from TurnPilot), so its take-up loses none."""
  sim, after, event = right_at_a_signal_taken_up(JUNCTION - 25., JUNCTION - 47., known_from=JUNCTION - 49.)
  assert events(event, 'turnpilot.signal_turn_late'), "first known too close: never the turn's stop"
  late, = events(event, 'turnpilot.signal_turn_taken_up_late')
  assert late['needed'] < -2.0 and late['d'] == pytest.approx(22., abs=1.5) and not late['started'] and late['v'] > 9.5
  assert [e['reason'] for e in events(event, 'turnpilot_turn_skipped')] == ['signalStop']
  assert after[1].tooFastToTurn and after[1].skipReason == 'signalStop'
  assert not any(p.desire == Desire.turnRight for p in after), "never turned through the unread light"
  # its light read green (an accepted source): nothing to stop for, the right taken as before
  sim, after, event = right_at_a_signal_taken_up(JUNCTION - 25., JUNCTION - 47., green=True, known_from=JUNCTION - 49.)
  assert not events(event, 'turnpilot.signal_turn_taken_up_late') and sim.tp.skip.reason == ''
  assert any(p.desire == Desire.turnRight for p in after)


class LeftHandSim(Sim):
  """test_turnpilotd's Sim under left-hand traffic (main(): IsRhdDetected): right_hand_traffic False on every plan."""
  def __init__(self, *args, **kw):
    super().__init__(*args, **kw)
    plan = self.tp.plan

    def lht(sm, scene, enabled, shadow, driver_signals, *rest, **kwargs):
      return plan(sm, scene, enabled, shadow, driver_signals, False, *rest, **kwargs)
    self.tp.plan = lht


def through(sim, side, until=JUNCTION + 25.):
  """Engaged on to until, turning at an 8 m radius toward side once its desire is out within 12 m of the junction. The plans,
  and the most it turned."""
  out, turned = [], 0.
  while sim.s < until:
    d = sim.tp.desires
    turning = d.started and not d.done and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    out.append(sim.step(engaged=True, yaw_rate=side * sim.v / 8. if turning else 0.))
    turned = max(turned, sim.tp.desires.turned_deg)
    assert sim.t < 120.
  return out, turned


def test_left_hand_traffic_the_left_is_the_curb_side_turn_and_the_right_the_unprotected_one():
  """Under left-hand traffic the follower, DesireTiming and TurnSkip all take the side traffic keeps to (2026-11-06 review:
  TurnSkip had it and the other two didn't, so a right was DesireTiming's curb-side 8 m pulse and TurnSkip's far-side 10-25 m
  deadline: every right skipped tooLate). A left there is the curb-side turn: one pulse, 5-9 m out, nothing before it; a right
  across the road is the unprotected one: stopped at its hold line and handed back (TurnPilotAutoLeft off: G1's), its desire
  on the far side's lead."""
  sim = LeftHandSim([(JUNCTION, "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))], cruise=10.)
  plans, turned = through(sim, 1)
  edges = rising([(0., 0., p) for p in plans], 'desire', Desire.turnLeft)
  first = next(p for p in plans if p.desire == Desire.turnLeft)
  assert len(edges) == 1 and CURB_FIRST_PULSE_M < first.maneuverDistance <= CURB_TURN_PULSE_M + 1., first.maneuverDistance
  assert turned > 75. and not any(p.tooFastToTurn for p in plans)
  sim = LeftHandSim(RIGHT_AT_A_CORNER, cruise=10.)
  sim.go_taps = False  # nobody takes over: held at the line
  plans = []
  while not plans or str(plans[-1].controlPrompt) != 'unprotectedTurnTakeControl':
    plans.append(sim.step(engaged=True))
    assert sim.t < 120.
  first = next(p for p in plans if p.desire == Desire.turnRight)
  assert first.maneuverDistance > 15., "the far side's lead, not the curb side's 8 m"
  assert plans[-1].shouldStop and str(plans[-1].mapControl.kind) == 'unprotectedTurn', "held at its hold line"
  assert not any(p.tooFastToTurn for p in plans)
  assert JUNCTION - HOLD_LINE_M - 3. < sim.s < JUNCTION - HOLD_LINE_M and sim.v < 0.3, (sim.s, sim.v)
  # the same right where traffic keeps right is the curb-side one: one pulse at 8 m, no stop
  sim = Sim(RIGHT_AT_A_CORNER, cruise=10.)
  plans, _ = through(sim, -1)
  assert len(rising([(0., 0., p) for p in plans], 'desire', Desire.turnRight)) == 1 and not any(p.shouldStop for p in plans)


def test_left_hand_traffic_reaches_the_follower_and_desire_timing():
  # the wiring itself: the follower's slight far-side turns and DesireTiming's lead take this frame's right_hand_traffic
  sim = LeftHandSim(RIGHT_AT_A_CORNER, cruise=10.)
  with mock.patch.object(RouteFollower, 'update', autospec=True, side_effect=RouteFollower.update) as rf_update, \
       mock.patch.object(DesireTiming, 'update', autospec=True, side_effect=DesireTiming.update) as dt_update:
    sim.step(engaged=True)
  assert rf_update.call_args.kwargs['right_hand_traffic'] is False
  assert dt_update.call_args.kwargs['right_hand_traffic'] is False
