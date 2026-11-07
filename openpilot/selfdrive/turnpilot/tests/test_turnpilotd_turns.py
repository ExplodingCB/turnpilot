"""ALWAYS_DRIVE.md S3's turns (R8b): a route turn the model drives straight through is a turn missed and a reroute at the
follower's pass, not a 'Turn abandoned' handback, while the car stays inside both of R8b's bounds on every frame: turned
under 15 deg toward the turn and under TURN_MISSED_AWAY_DEG away from it (turned_deg is signed toward the turn), and
modelV2.action.desiredCurvature under TURN_MISSED_CURV either way, but on frames the model steers into the turn (2026-11-06: a
steer-in latched the watch off, and a right steered into and then driven dead straight was said by nothing). Under
TurnPilotAlwaysDrive R8's fire inside the bounds clears the desire (CLEAR_DESIRE) and hands nothing back; either bound broken
after it is R8's handback, latched as today. A
committed turn driven dead straight that never fires R8 is missed at the pass with the switch off as well as on (R9's detector
catching more), its last pulse cleared from the model's queue on that frame (WARGAME_FIXES WF-11)."""
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import RIGHT_AT_A_CORNER, Sim, stand
from openpilot.selfdrive.turnpilot.turnpilotd import (MISSED_HOLD_S, STALL_PROMPT_S, STALLED_REASON, TURN_ABANDON_S,
                                                       TURN_ABANDONED_REASON, TURN_MISSED_AWAY_DEG, TURN_MISSED_CURV,
                                                       TURN_MISSED_REASON)

Desire = log.Desire
JUNCTION = 300.  # RIGHT_AT_A_CORNER's right
PASS_S = JUNCTION + 20.  # past the follower's pass (15 m past the furthest reading of the junction)


def into_a_right(sim, deg, junction=300., curvature=0.03):
  """RIGHT_AT_A_CORNER's turn driven deg in, the model's curvature toward it (positive: right) once its one pulse is out, the
  approach straight: a model steering into the right at a turn's scale from the approach with the car going straight on is one
  whose pulse waits (desire_timing.STEER_IN_TURN_CURV) and is skipped at its deadline
  (test_a_right_the_model_steers_into_with_no_turn_is_skipped_out_loud...); the default 0.03, under it, would be pulsed at 8 m
  anyway (test_a_right_off_a_road_bending_its_way_or_a_lean_is_pulsed_at_8_m_never_skipped)."""
  sim.model_kw = {'curvature': 0.}
  while junction - sim.s > 3. or not sim.tp.desires.started:
    if sim.tp.desires.started:
      sim.model_kw = {'curvature': curvature}
    sim.step(engaged=True)
    assert sim.t < 120. and junction - sim.s > 0.
  sim.model_kw = {'curvature': curvature}
  while sim.tp.desires.turned_deg < deg:
    sim.step(engaged=True, yaw_rate=-0.4)
    assert sim.t < 140.


def right_sim(always_drive: bool) -> Sim:
  sim = Sim(RIGHT_AT_A_CORNER, cruise=8.)
  sim.tp.always_drive = always_drive
  return sim


def drive(sim, until_s: float, yaw_rate: float = 0., curvature=None) -> list:
  """Engaged on to until_s, the model's desiredCurvature held (curvature a value or a function of the sim)."""
  plans = []
  while sim.s < until_s:
    if curvature is not None:
      sim.model_kw = {'curvature': curvature(sim) if callable(curvature) else curvature}
    plans.append(sim.step(engaged=True, yaw_rate=yaw_rate))
    assert sim.t < 200.
  return plans


def prompts(plans) -> set:
  return {str(p.controlPrompt) for p in plans}


def test_the_bounds():
  assert TURN_MISSED_CURV == 0.015 and 0. < TURN_MISSED_AWAY_DEG < 10.


@pytest.mark.parametrize("always_drive", [False, True])
def test_a_right_driven_straight_inside_the_bounds_after_r8_fires_is_missed_at_the_pass_under_always_drive(always_drive):
  """BeamNG gate S3_right_green: the model drove straight through the right with its desire out, 7.4 deg turned and
  -0.0092 1/m away from it, and R8 handed back ('Turn abandoned') at 13.2 s. Under the switch R8's fire inside both bounds
  clears the desire from the fire and hands nothing back, the car drives on and the turn is missed at the follower's pass
  (turnMissed, 'Turn Missed', 'Rerouting': navd reroutes at once), never before it. With the switch off, today's handback."""
  sim = right_sim(always_drive)
  into_a_right(sim, 7.)
  plans = drive(sim, PASS_S + 30., curvature=-0.0092)
  if not always_drive:
    abandoned = next(i for i, p in enumerate(plans) if p.controlPrompt == 'turnAbandonedTakeControl')
    assert abs(abandoned * DT_MDL - TURN_ABANDON_S) <= 2 * DT_MDL and plans[abandoned].reason == TURN_ABANDONED_REASON
    assert not any(p.turnMissed for p in plans)
    return
  assert 'turnAbandonedTakeControl' not in prompts(plans)
  fire = next(i for i, p in enumerate(plans) if p.desire == CLEAR_DESIRE)
  assert abs(fire * DT_MDL - TURN_ABANDON_S) <= 2 * DT_MDL  # from the fire, the fire's own frame included
  assert not any(p.desire == Desire.turnRight for p in plans[fire:])  # never pulsed again: the queued pulse can't swing the car
  missed = next(i for i, p in enumerate(plans) if p.turnMissed)
  assert missed > fire and plans[missed].maneuverDistance < -10.  # at the pass, not before it
  assert not any(p.turnMissed or p.tooFastToTurn for p in plans[:missed])
  held = [p for p in plans[missed:] if p.turnMissed]
  assert abs(len(held) * DT_MDL - MISSED_HOLD_S) <= 2 * DT_MDL and all(p.reason == TURN_MISSED_REASON for p in held)
  assert not any(p.shouldStop for p in plans) and sim.tp.post_go is None


@pytest.mark.parametrize("always_drive", [False, True])
@pytest.mark.parametrize("past", [1., 3.2, 8.])
def test_a_stand_in_the_junction_after_r8bs_fire_is_the_stall_handback_at_3_s(always_drive, past):
  """ad-s3 skeptic (blocking): R8b's fire cancels the turn's desire, so turn_committed is False to the pass, and the stall
  watch keyed on it alone stopped watching the turn. The model standing in the junction past the turn's point after the fire
  got only stopped_unheld: the 5 s note and carStoppedTakeControl at 60 s (90 s near a mapped control), silent in the junction
  box, where before S3 R8's handback had it at once. A fired straight (straight_fired) is the stall's as a committed turn is:
  stalledTakeControl at STALL_PROMPT_S, held while it stands, never the note, and the turn's desire never sent again. With
  the switch off, R8's handback throughout, as today."""
  sim = right_sim(always_drive)
  into_a_right(sim, 7.)
  plans = drive(sim, sim.s + 0.05, curvature=-0.0092)
  while sim.tp.follower.distance > -past:
    plans += drive(sim, sim.s + 0.05, curvature=-0.0092)
  assert -past - 1. < sim.tp.follower.distance and not sim.tp.follower.passed and not any(p.turnMissed for p in plans)
  st = stand(sim, STALL_PROMPT_S + 7.)
  if not always_drive:
    assert all(p.controlPrompt == 'turnAbandonedTakeControl' for p in st)
    return
  assert any(p.desire == CLEAR_DESIRE for p in plans) and sim.tp.straight_fired and not sim.tp.turn_committed
  assert 'turnAbandonedTakeControl' not in prompts(plans + st)
  stalled = next(i for i, p in enumerate(st) if p.controlPrompt == 'stalledTakeControl')
  assert abs(stalled * DT_MDL - STALL_PROMPT_S) <= 2 * DT_MDL
  assert all(p.controlPrompt == 'none' for p in st[:stalled])
  assert all(p.controlPrompt == 'stalledTakeControl' and p.reason == STALLED_REASON for p in st[stalled:])
  assert not any(p.controlAlert == 'carStoppedNote' for p in st)
  assert not any(p.desire in (Desire.turnRight, Desire.turnLeft) for p in st)  # the turn R8b cleared is never pulsed again


@pytest.mark.parametrize("past", [1., 3.2, 8.])
def test_a_stand_after_r8bs_fire_gets_none_of_left_21s_finish(past):
  """tp/ad-s2 and tp/ad-s3 merged: S3 made a stand after R8b's fire the stall's (model_turn takes straight_fired), and S2's
  stall under the switch runs finish_turn (LEFT-21) for any curb-side route turn. A right R8b cleared was driven straight, so
  there is no turn to finish: no fresh pulse, no clearBox lift said "Stalled: finishing the turn", no carStoppedNote between
  lifts, only S3's stalledTakeControl at STALL_PROMPT_S. On the merge without the fix it got a lift at 3.0 s and 9.0 s and
  the note at 5 s. TurnPilotGoOverride is on, so the lift is not refused for the param."""
  sim = right_sim(True)
  assert sim.tp.go_override
  into_a_right(sim, 7.)
  plans = drive(sim, sim.s + 0.05, curvature=-0.0092)
  while sim.tp.follower.distance > -past:
    plans += drive(sim, sim.s + 0.05, curvature=-0.0092)
  assert sim.tp.straight_fired and not sim.tp.turn_committed
  st = stand(sim, STALL_PROMPT_S + 9.)
  assert not any(p.controlAlert in ('stallLift', 'carStoppedNote') for p in st)
  assert not any(str(p.goCertified.cause) == 'clearBox' for p in st) and sim.tp.stall_lifts == 0 and not sim.tp.stall_pulsed
  assert {str(p.controlPrompt) for p in st[int(round((STALL_PROMPT_S + 0.1) / DT_MDL)):]} == {'stalledTakeControl'}


@pytest.mark.parametrize("always_drive", [False, True])
def test_a_committed_right_driven_dead_straight_is_missed_at_the_pass_and_its_pulse_cleared(always_drive):
  """R8b's dead-straight half, unconditional: a committed right the model drives dead straight, under TURN_ABANDON_CURV away
  so R8 never fires, said nothing today (check_missed reads only a turn a hold let go of). From the hold line to the
  follower's pass inside both bounds, it is missed at the pass with the switch off as well as on, and the desire sent on
  that frame is CLEAR_DESIRE (WF-11: the last pulse otherwise stays in the model's queue past the junction)."""
  sim = right_sim(always_drive)
  plans = drive(sim, PASS_S + 30., curvature=0.)
  assert sim.tp.desires.maneuver_id >= 0 and any(p.desire == Desire.turnRight for p in plans)  # committed: its desire went out
  assert not ({'turnAbandonedTakeControl', 'stalledTakeControl'} & prompts(plans))
  missed = next(i for i, p in enumerate(plans) if p.turnMissed)
  assert plans[missed].maneuverDistance < -10. and not any(p.turnMissed for p in plans[:missed])
  assert plans[missed].desire == CLEAR_DESIRE and CLEAR_DESIRE not in [p.desire for p in plans[:missed]]
  assert abs(sum(p.turnMissed for p in plans) * DT_MDL - MISSED_HOLD_S) <= 2 * DT_MDL
  assert not any(p.tooFastToTurn for p in plans)


@pytest.mark.parametrize("always_drive", [False, True])
def test_a_car_yawed_away_from_the_turn_breaks_the_bound_and_is_abandoned(always_drive):
  """turned_deg is signed toward the turn: a car yawed 10 deg away from a right in the junction reads -10 and is heading for
  the oncoming lanes, a mess-up, never a straight drive. R8's fire there is R8's handback with the switch on as well as off,
  and driven on that way without R8 firing it is no miss either."""
  sim = right_sim(always_drive)
  drive(sim, JUNCTION - 3., curvature=0.)
  while sim.tp.desires.turned_deg > -10.:
    sim.step(engaged=True, yaw_rate=0.4)  # left: away from the right
  plans = drive(sim, PASS_S, curvature=-0.0092)
  assert 'turnAbandonedTakeControl' in prompts(plans) and not any(p.turnMissed for p in plans)
  # the same yaw, the model's curvature dead ahead: R8 never fires, and nothing is missed at the pass
  sim = right_sim(always_drive)
  drive(sim, JUNCTION - 3., curvature=0.)
  while sim.tp.desires.turned_deg > -10.:
    sim.step(engaged=True, yaw_rate=0.4)
  plans = drive(sim, PASS_S + 10., curvature=0.)
  assert 'turnAbandonedTakeControl' not in prompts(plans) and not any(p.turnMissed for p in plans)


def test_an_unwind_that_grows_past_the_curvature_bound_after_the_fire_is_abandoned_on_that_frame():
  """S20's unwind read -0.012 to -0.019 1/m for 0.5-0.75 s: a curvature read once at the fire proves nothing. Under the
  switch R8 fires inside the bounds at -0.012 (the desire cleared, no handback), then on the frame the curvature passes
  TURN_MISSED_CURV R8's handback fires and latches for the maneuver, the desire staying cleared, until the driver takes over."""
  sim = right_sim(True)
  into_a_right(sim, 7.)
  fired = drive(sim, sim.s + 0.1, curvature=-0.012)
  while not any(p.desire == CLEAR_DESIRE for p in fired):
    fired.append(drive(sim, sim.s + 0.01, curvature=-0.012)[-1])
  assert 'turnAbandonedTakeControl' not in prompts(fired)
  ramp, plans = [], []
  for i in range(int(0.5 / DT_MDL)):
    c = -0.012 - 0.007 * (i + 1) / (0.5 / DT_MDL)
    ramp.append(c)
    sim.model_kw = {'curvature': c}
    plans.append(sim.step(engaged=True))
  first = next(i for i, c in enumerate(ramp) if abs(c) >= TURN_MISSED_CURV)
  assert all(p.controlPrompt == 'none' for p in plans[:first])
  assert all(p.controlPrompt == 'turnAbandonedTakeControl' for p in plans[first:])
  sim.model_kw = {'curvature': -0.005}  # back inside the bound: the handback is latched for the maneuver
  later = [sim.step(engaged=True) for _ in range(int(1. / DT_MDL))]
  assert all(p.controlPrompt == 'turnAbandonedTakeControl' and p.reason == TURN_ABANDONED_REASON for p in later)
  assert not any(p.desire == Desire.turnRight for p in plans + later) and not any(p.turnMissed for p in plans + later)
  assert sim.step(engaged=False, gas=True).controlPrompt == 'none'  # the driver takes over


@pytest.mark.parametrize("always_drive", [False, True])
@pytest.mark.parametrize("turned, curvature", [(20., -0.012), (7., -0.02), (24., -0.015)], ids=["turned", "curvature", "S20"])
def test_an_unwind_past_either_bound_at_the_fire_is_abandoned_as_today(always_drive, turned, curvature):
  sim = right_sim(always_drive)
  into_a_right(sim, turned)
  plans = drive(sim, sim.s + 6., curvature=curvature)
  abandoned = next(i for i, p in enumerate(plans) if p.controlPrompt == 'turnAbandonedTakeControl')
  assert abs(abandoned * DT_MDL - TURN_ABANDON_S) <= 2 * DT_MDL
  assert all(p.controlPrompt == 'turnAbandonedTakeControl' for p in plans[abandoned:])
  assert not any(p.turnMissed for p in plans)


@pytest.mark.parametrize("always_drive", [False, True])
def test_a_right_taken_is_never_missed_nor_abandoned(always_drive):
  sim = right_sim(always_drive)
  into_a_right(sim, 70.)  # cleared at its angle less CLEAR_MARGIN_DEG on the way
  plans = drive(sim, sim.s + 30., yaw_rate=-0.4, curvature=0.03)
  assert not any(p.turnMissed for p in plans) and 'turnAbandonedTakeControl' not in prompts(plans)
  assert not any(p.turnMissed for _, _, p in sim.trace)


def test_a_right_gets_one_pulse_8_m_out_in_closed_loop():
  """2026-11-06, 1e after dark: every fresh rising edge started the right where the car then was (0.3-1.5 s after it), and the
  40 m pulse, its repeat about 20 m out and the 10 m entry pulse each started it 11-25 m out on a 12-15 m radius, into the
  inside curb. turnpilotd under its own turn cap, from road speed: the right's one pulse goes CURB_TURN_PULSE_M out, nothing
  before it, never said too late or too fast, and it is held through the turn to its clear (no repeat, no entry pulse)."""
  from openpilot.selfdrive.turnpilot.desire_timing import CURB_TURN_PULSE_M
  sim = Sim(RIGHT_AT_A_CORNER, cruise=13.)
  sim.v = 13.
  plans = []
  while sim.s < JUNCTION + 25.:
    turning = sim.tp.desires.started and abs(JUNCTION - 6.7 - sim.s) < 12.5 and not sim.tp.desires.done
    plans.append(sim.step(engaged=True, yaw_rate=-sim.v / 8. if turning and sim.s > JUNCTION - 6.7 else 0.))
    assert sim.t < 120.
  assert not any(p.tooFastToTurn for p in plans), [str(p.skipReason) for p in plans if p.tooFastToTurn][:1]
  edges = [i for i in range(1, len(plans)) if plans[i].desire == Desire.turnRight and plans[i - 1].desire != Desire.turnRight]
  assert len(edges) == 1, [sim.trace[i][0] for i in edges]
  i = edges[0]
  s, v, _ = sim.trace[i]
  assert CURB_TURN_PULSE_M - 0.5 < JUNCTION - s < CURB_TURN_PULSE_M + 0.5 and v < 3.5, (JUNCTION - s, v)  # under its cap
  assert all(p.desire == Desire.none for p in plans[:i])
  assert any(p.desire == CLEAR_DESIRE for p in plans[i:])


SPREAD_J = 300.  # the spread probe's right


def spread_route():
  from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
  return [(SPREAD_J, "turn", "right", junction_of("right")), (SPREAD_J + 1200., "turn", "left", junction_of("left", 2))]


def biased_navd(route, off, from_m, to_m):
  """A nav_filter: navd reads the right's junction off m (+ long, - short) while the car is from_m..to_m m short of it."""
  from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of, nav_instruction

  def f(sim, instr):
    d = SPREAD_J - sim.s
    if not (to_m <= d <= from_m):
      return instr, True
    return nav_instruction(d + off, "turn", "right", remaining=SPREAD_J + 1300. - sim.s + off, junction=junction_of("right"),
                           ahead=[(route[1][0] - sim.s + off,) + tuple(route[1][1:])]), True
  return f


@pytest.mark.parametrize("off,from_m,to_m", [
  (0., 0., 0.),     # no spread
  (4., 300., 30.),  # an early bias long, true from 30 m in: the furthest long to the junction (1e 8:32's shape)
  (6., 300., 30.),
  (9., 300., 30.),
  (-4., 30., 0.),   # a late bias short over the last 30 m: the nearest (and the latest) short, the furthest true
  (-6., 30., 0.),
  (-9., 30., 0.),
])
def test_a_right_whose_readings_spread_is_pulsed_or_skipped_out_loud_never_lost_in_silence(off, from_m, to_m):
  """The adversarial review's closed-loop probe (2026-11-06): a right whose navd readings spread 4-9 m. With TurnSkip given
  neither the furthest nor the latest reading every biased case was skipped tooLate before its pulse was due (the nearest's
  5 m deadline, the pulse waiting for the furthest), and with the furthest alone a 9 m long furthest or a 9 m short nearest
  left the right with no pulse and no word at all. turnpilotd now hands both TurnSkip and DesireTiming the furthest and the
  latest: never lost in silence, never pulsed at or past the junction, and every one driven, by the furthest at
  CURB_TURN_PULSE_M or the latest at CURB_LATEST_PULSE_M, a persistent short bias bringing it in by the bias less 2 m
  (test_desire_timing)."""
  from openpilot.selfdrive.turnpilot.desire_timing import CURB_LATEST_PULSE_M, CURB_TURN_PULSE_M
  route = spread_route()
  sim = Sim(route, cruise=13.)
  sim.v = 13.
  if off:
    sim.nav_filter = biased_navd(route, off, from_m, to_m)
  plans, pulse_d, skipped_d = [], None, None
  while sim.s < SPREAD_J + 25. and sim.t < 120.:
    d = sim.tp.desires
    turning = d.started and not d.done and sim.s > SPREAD_J - 6.7
    p = sim.step(engaged=True, yaw_rate=-sim.v / 8. if turning else 0.)
    if pulse_d is None and p.desire == Desire.turnRight:
      pulse_d = SPREAD_J - sim.s
    if skipped_d is None and p.tooFastToTurn:
      skipped_d = SPREAD_J - sim.s
    plans.append(p)
  case = (off, pulse_d, skipped_d, [str(p.skipReason) for p in plans if p.tooFastToTurn][:1])
  assert pulse_d is not None or skipped_d is not None, case  # never lost in silence
  assert pulse_d is None or pulse_d > 0., case                 # never a first pulse at or past the junction
  assert pulse_d is None or skipped_d is None, case            # one or the other
  early = max(-off - (CURB_TURN_PULSE_M - CURB_LATEST_PULSE_M), 0.)
  assert pulse_d is not None and CURB_LATEST_PULSE_M - 0.6 < pulse_d < CURB_TURN_PULSE_M + early + 0.6, case


def test_a_right_the_model_takes_on_its_own_while_its_pulse_waits_is_driven_never_skipped():
  """2026-11-06 (the rights review): the right's one pulse waits while the model steers into it with the car rolling
  (desire_timing.py: a fresh edge then restarts the model's turn, BeamNG right_green). In closed loop under turnpilotd, the
  model steering into the right from 9 m by itself (an end-of-road T, a right-turn-only lane) and the car turning with it:
  no pulse goes into it, it is never said too late or too fast, it counts as the turn under way (turn_committed) and is
  cleared at its angle, never missed or abandoned."""
  from openpilot.selfdrive.turnpilot.desire_timing import CURB_TURN_PULSE_M
  from openpilot.selfdrive.turnpilot.turn_skip import CURB_FIRST_PULSE_M
  sim = Sim(RIGHT_AT_A_CORNER, cruise=8.)
  plans, own_at, committed = [], None, False
  while sim.s < JUNCTION + 25. and sim.t < 120.:
    d = sim.tp.desires
    steering = JUNCTION - sim.s < 9. and not d.done
    sim.model_kw = {'curvature': 0.12 if steering else 0.}
    plans.append(sim.step(engaged=True, yaw_rate=-sim.v * 0.12 if steering else 0.))
    if own_at is None and sim.tp.desires.own:
      own_at = JUNCTION - sim.s
    committed = committed or (sim.tp.desires.own and sim.tp.turn_committed)
  assert not any(p.tooFastToTurn for p in plans), [str(p.skipReason) for p in plans if p.tooFastToTurn][:1]
  assert not any(p.desire == Desire.turnRight for p in plans)
  # following the model OWN_FOLLOW_DEG round once its pulse waits (at the pulse point, 8 m), before its deadline
  assert own_at is not None and CURB_FIRST_PULSE_M < own_at < CURB_TURN_PULSE_M and committed, own_at
  assert any(p.desire == CLEAR_DESIRE for p in plans)  # cleared at its angle
  assert not any(p.turnMissed for p in plans) and not ({'turnAbandonedTakeControl', 'stalledTakeControl'} & prompts(plans))


def test_a_right_the_model_steers_into_with_no_turn_is_skipped_out_loud_at_its_deadline_never_pulsed_late():
  """The other end of the wait: the model steering into the right at a turn's scale (0.06 1/m, a 17 m radius) from 9 m, new at
  its pulse point, and the car not turning with it (the sim's car yaws only as told): at its deadline
  (turn_skip.CURB_FIRST_PULSE_M by the furthest, 3 m by the latest) the right is skipped tooLate out loud ('Too Fast to
  Turn', navd's reroute), and no pulse ever goes: never into a model mid-steer, never at or past the junction, never lost in
  silence."""
  from openpilot.selfdrive.turnpilot.desire_timing import CURB_TURN_PULSE_M
  from openpilot.selfdrive.turnpilot.turn_skip import CURB_FIRST_PULSE_M
  sim = Sim(RIGHT_AT_A_CORNER, cruise=8.)
  plans, skipped_d = [], None
  while sim.s < JUNCTION + 25. and sim.t < 120.:
    sim.model_kw = {'curvature': 0.06 if JUNCTION - sim.s < 9. else 0.}
    p = sim.step(engaged=True)
    if skipped_d is None and p.tooFastToTurn:
      skipped_d = JUNCTION - sim.s
    plans.append(p)
  assert skipped_d is not None and str(next(p.skipReason for p in plans if p.tooFastToTurn)) == 'tooLate'
  assert CURB_FIRST_PULSE_M - 1. < skipped_d < CURB_TURN_PULSE_M, skipped_d
  assert not any(p.desire == Desire.turnRight for p in plans) and not sim.tp.desires.started


@pytest.mark.parametrize("curv", [0.006, 0.01, 0.02, 0.03, 0.035])
def test_a_right_off_a_road_bending_its_way_or_a_lean_is_pulsed_at_8_m_never_skipped(curv):
  """The 2026-11-06 review's closed-loop probe: RIGHT_AT_A_CORNER with the model reading curv toward the right from 60 m out
  (a road bending the turn's way; the old deadline test's constant 0.03 from the start is the same input), the car yawing with
  it, was skipped tooLate at 4.8 m and 3.1 m/s, every one, its pulse waiting on a steer-in at STEER_IN_CURV's 0.005. Under a
  turn's scale (desire_timing.STEER_IN_TURN_CURV) the right's one pulse goes CURB_TURN_PULSE_M out as on a straight
  approach, never said too late or too fast, never taken as the model's own; and turned on an 8 m radius from 6.7 m, it is
  turned and cleared."""
  from openpilot.selfdrive.turnpilot.desire_timing import CURB_TURN_PULSE_M
  sim = Sim(RIGHT_AT_A_CORNER, cruise=8.)
  plans, pulse_d = [], None
  while sim.s < JUNCTION + 25. and sim.t < 120.:
    d = sim.tp.desires
    turning = d.started and not d.done and sim.s > JUNCTION - 6.7
    k = 1. / 8. if turning else (curv if JUNCTION - sim.s < 60. else 0.)
    sim.model_kw = {'curvature': k}
    p = sim.step(engaged=True, yaw_rate=-sim.v * k)
    if pulse_d is None and p.desire == Desire.turnRight:
      pulse_d = JUNCTION - sim.s
    plans.append(p)
  assert not any(p.tooFastToTurn for p in plans), [str(p.skipReason) for p in plans if p.tooFastToTurn][:1]
  assert pulse_d is not None and CURB_TURN_PULSE_M - 0.6 < pulse_d < CURB_TURN_PULSE_M + 0.1, pulse_d
  assert not sim.tp.desires.own and any(p.desire == CLEAR_DESIRE for p in plans)
  assert not any(p.turnMissed for p in plans) and not ({'turnAbandonedTakeControl', 'stalledTakeControl'} & prompts(plans))


@pytest.mark.parametrize("start, curv, walker_at", [(16., 0.045, True), (14., 0.05, True), (12., 0.05, False)])
def test_a_wide_right_the_model_starts_early_keeps_its_cap_and_its_crosswalks_walker_hold(start, curv, walker_at):
  """The 2026-11-06 review: a right the model starts on its own on a wide arc before the pulse point (0.04-0.055 from 12-16 m)
  was counted only from the pulse's wait, read under OWN_TURN_DEG at the deadline and was skipped tooLate with the car 20-26 deg
  round: turnpilotd then handed the maneuver on as None, so the right's corner cap and the scene's hold for a walker on its
  crosswalk dropped mid-turn, and 'Too Fast to Turn' sounded with navd rerouting. Counted from the model's steer-in, it is the
  model's own on its approach: never skipped or pulsed, its corner cap held to the junction, and a walker stepping onto its
  crosswalk as it becomes the model's own, short of its hold line, stops the car short of her (a hard rail: people)."""
  from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M
  from openpilot.selfdrive.turnpilot.turn_speed import NO_SPEED_CAP
  sim = Sim(RIGHT_AT_A_CORNER, cruise=8.)
  plans, own_at = [], None
  walker = {'on': False}

  def objects(sim):
    return ({'cls': 'pedestrian', 'x': JUNCTION - sim.s + 3., 'y': -5., 'age': 5 + int(sim.t * 10.)},) if walker['on'] else ()
  while sim.s < JUNCTION + 25. and sim.t < 120.:
    d = sim.tp.desires
    steering = JUNCTION - sim.s < start and not d.done and sim.v > 0.3
    sim.model_kw = {'curvature': curv if steering else 0.}
    p = sim.step(objects(sim), engaged=True, yaw_rate=-sim.v * curv if steering else 0.)
    plans.append((JUNCTION - sim.s, p))
    if own_at is None and sim.tp.desires.own:
      own_at, walker['on'] = JUNCTION - sim.s, walker_at  # she steps onto the right's crosswalk as the turn is the model's
    if walker['on'] and sim.v < 0.05:
      break
  assert own_at is not None and own_at > (HOLD_LINE_M if walker_at else 5.), own_at
  assert not any(p.tooFastToTurn for _, p in plans), [str(p.skipReason) for _, p in plans if p.tooFastToTurn][:1]
  assert not any(p.desire == Desire.turnRight for _, p in plans)
  after = [(dd, p) for dd, p in plans if dd <= own_at]
  assert after and all(p.speedCap < NO_SPEED_CAP or p.shouldStop or p.reason == "pedestrian in crosswalk" for dd, p in after if dd > 0.), \
    "its corner cap through the turn"
  if walker_at:
    assert sim.v < 0.05 and JUNCTION - sim.s > 0. and plans[-1][1].shouldStop and plans[-1][1].reason == "pedestrian in crosswalk", \
      (sim.v, JUNCTION - sim.s, plans[-1][1].reason)
  else:
    assert any(p.desire == CLEAR_DESIRE for _, p in plans) and not any(p.turnMissed for _, p in plans)  # turned, cleared at its angle


@pytest.mark.parametrize("always_drive", [False, True])
def test_a_right_taken_as_the_models_own_that_the_model_straightens_out_of_is_said(always_drive):
  """The model steering into the right from 9 m (0.07 1/m, the car with it) until the turn is its own (OWN_TURN_DEG), then
  straightening out of it (0.02 1/m away) and driving through: given back to the wait (desire_timing), its pulse goes as the
  model lets go, short of the deadline, and the turn under way is turnpilotd's to watch: R8's 'Turn abandoned' handback, or
  under TurnPilotAlwaysDrive R8b's miss at the pass. Never silent, never too fast or too late, never a pulse while the model
  steers in. (Driven dead straight after a steer-in it was said by nothing, R8b's watch latched off on the steer-in's
  curvature; a steer-in no longer breaks it: test_a_committed_right_the_model_steers_into_and_then_drives_dead_straight...)"""
  sim = right_sim(always_drive)
  plans, own_at, pulses_while_in = [], None, 0
  while sim.s < PASS_S + 30. and sim.t < 150.:
    d = sim.tp.desires
    if own_at is None and d.own:
      own_at = JUNCTION - sim.s
    steering = JUNCTION - sim.s < 9. and own_at is None
    sim.model_kw = {'curvature': 0.07 if steering else (-0.02 if own_at is not None else 0.)}
    p = sim.step(engaged=True, yaw_rate=-sim.v * 0.07 if steering else 0.)
    pulses_while_in += steering and p.desire == Desire.turnRight
    plans.append(p)
  assert own_at is not None and pulses_while_in == 0
  assert any(p.desire == Desire.turnRight for p in plans)  # its pulse as the model let go
  said = 'turnAbandonedTakeControl' in prompts(plans) or any(p.turnMissed for p in plans)
  assert said and not any(p.tooFastToTurn for p in plans)


@pytest.mark.parametrize("gap", [3., 4.])
def test_a_deciders_right_whose_stop_line_is_inside_its_deadline_is_never_skipped_braking_to_it(tmp_path, gap):
  """The 2026-11-06 review: the stop-sign decider's right (pulse_at_go; its pulse held to the go) is judged by its rolling
  5 m deadline since this round, and TurnSkip called it too late before its stop: in closed loop through the decider
  (test_turnpilotd_stop's StopSim) a right whose stop line sits 3 m short of navd's node was skipped tooLate braking to it,
  4.9 m out at 1.9 m/s, still 'approach'. MapControlPolicy.stop_ahead() has the stop sign between the car and the junction
  (its line's distance, 0 held at it), which puts the deadline off: never skipped, its first pulse the decider's go, standing
  at the line."""
  from openpilot.selfdrive.turnpilot.desire_timing import STANDSTILL_SPEED
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import S_LINE, StopSim, go_frame
  sim = StopSim(tmp_path, model='goes', info_kw={'maneuver': 'right'}, maneuvers=[(S_LINE + gap, 'turn', 'right'), (2000., 'turn', 'left')])
  read, pulse = [], None
  while sim.t < 40. and sim.s < S_LINE + gap + 15.:
    p = sim.step()
    if 0. < S_LINE + gap - sim.s < 5. and str(p.stopGo.state) == 'approach':
      read.append(sim.tp.map_controls.stop_ahead())
    if pulse is None and p.desire == Desire.turnRight:
      pulse = (S_LINE + gap - sim.s, sim.v, str(p.stopGo.state))
  plans = sim.plans()
  assert not any(p.tooFastToTurn for p in plans), [(str(p.skipReason), str(p.stopGo.state)) for p in plans if p.tooFastToTurn][:1]
  assert read and all(r is not None and r >= 0. for r in read), read  # the stop sign between the car and the junction
  assert go_frame(sim) is not None and pulse is not None and pulse[1] < STANDSTILL_SPEED and pulse[2] == 'going', pulse


@pytest.mark.parametrize("always_drive", [False, True])
@pytest.mark.parametrize("steer_s,curvature", [(0.6, 0.03), (1.5, 0.08), (0., 0.03)], ids=["mild", "hard", "toThePass"])
def test_a_committed_right_the_model_steers_into_and_then_drives_dead_straight_is_missed_at_the_pass(always_drive, steer_s, curvature):
  """curb-turn-timing risk 3 (2026-11-06): R8b's watch latched off on the first frame the model steered into the turn past
  TURN_MISSED_CURV, so a committed right the model steered into (its pulse out, the car yawing a few degrees with it) and then
  drove dead straight past its junction under MISSED_TURN_DEG was said by nothing: turn_abandoned needs the model steering
  away, check_missed a go. A steer-in no longer breaks the watch: at the follower's pass (PASSED_DIST_M past the furthest
  reading, where a right taken is far round) the car inside the deg bounds is the turn missed, with the switch off as well as
  on: turnMissed ('Turn Missed', 'Rerouting'), the pulse cleared from the model's queue on that frame. 'toThePass': the model
  still steering into it at the pass with the car dead straight, said all the same."""
  sim = right_sim(always_drive)
  plans = drive(sim, JUNCTION - 3., curvature=0.)
  assert sim.tp.desires.started and sim.tp.turn_committed
  t0 = sim.t
  while sim.t - t0 < (steer_s or 60.) and sim.s < PASS_S + 30.:
    sim.model_kw = {'curvature': curvature}
    plans.append(sim.step(engaged=True, yaw_rate=-0.15 if steer_s else 0.))
  assert sim.tp.desires.turned_deg < 15., sim.tp.desires.turned_deg
  plans += drive(sim, PASS_S + 30., curvature=0.)
  assert not ({'turnAbandonedTakeControl', 'stalledTakeControl'} & prompts(plans))
  missed = next(i for i, p in enumerate(plans) if p.turnMissed)
  assert plans[missed].maneuverDistance < -10. and not any(p.turnMissed for p in plans[:missed])
  assert plans[missed].desire == CLEAR_DESIRE and plans[missed].reason == TURN_MISSED_REASON
  assert not any(p.tooFastToTurn for p in plans)


@pytest.mark.parametrize("walker_at", [True, False])
def test_a_right_skipped_too_late_that_the_model_turns_anyway_is_adopted_with_its_cap_and_walker_hold(walker_at):
  """curb-turn-timing risk 1 (2026-11-06): the model steering into a right at a turn's scale from 9 m with the car not following
  (under OWN_FOLLOW_DEG by the 5 m deadline) has the right skipped tooLate out loud, its pulse never sent. Then the model turns
  it anyway: skipped, the right was None to the scene and the desire, so the walker hold on its crosswalk was dropped mid-turn
  and nothing watched it. Turned TURN_STARTED_DEG into before the follower's pass, it is adopted as the model's own
  (turnpilotd.adopt_skipped): no longer skipped (tooFastToTurn off), started and own (never pulsed into), its corner cap held,
  and a walker on its crosswalk stops the car short of her at the right's 1.8 m/s (a hard rail: people); with nobody there it
  is turned and cleared at its angle, never missed or abandoned."""
  from openpilot.selfdrive.turnpilot.turn_speed import NO_SPEED_CAP
  sim = Sim(RIGHT_AT_A_CORNER, cruise=1.8)
  sim.s, sim.v = 250., 1.8
  plans, skipped_d, adopted_d = [], None, None
  walker = {'on': False}

  def objects(sim):
    return ({'cls': 'pedestrian', 'x': JUNCTION - sim.s + 3., 'y': -5., 'age': 5 + int(sim.t * 10.)},) if walker['on'] else ()
  while sim.s < JUNCTION + 25. and sim.t < 120.:
    turning = skipped_d is not None and not sim.tp.desires.done and sim.v > 0.3
    sim.model_kw = {'curvature': 0.12 if turning else (0.06 if JUNCTION - sim.s < 9. else 0.)}
    p = sim.step(objects(sim), engaged=True, yaw_rate=-0.4 if turning else 0.)
    plans.append((JUNCTION - sim.s, p))
    if skipped_d is None and p.tooFastToTurn:
      skipped_d = JUNCTION - sim.s
    if adopted_d is None and sim.tp.desires.own:
      adopted_d, walker['on'] = JUNCTION - sim.s, walker_at
    if walker['on'] and sim.v < 0.05:
      break
  assert skipped_d is not None and str(next(p.skipReason for _, p in plans if p.tooFastToTurn)) == 'tooLate'
  assert adopted_d is not None and 0. < adopted_d < skipped_d, (skipped_d, adopted_d)
  after = [p for dd, p in plans if dd <= adopted_d]
  assert not any(p.tooFastToTurn for p in after) and not any(p.desire == Desire.turnRight for _, p in plans)
  assert all(p.speedCap < NO_SPEED_CAP or p.shouldStop for dd, p in plans if dd <= adopted_d and dd > 0.), "its corner cap"
  if walker_at:
    assert sim.v < 0.05 and JUNCTION - sim.s > 0. and plans[-1][1].shouldStop and plans[-1][1].reason == "pedestrian in crosswalk", \
      (sim.v, JUNCTION - sim.s, plans[-1][1].reason)
  else:
    assert any(p.desire == CLEAR_DESIRE for p in after) and not any(p.turnMissed for _, p in plans)
    assert not ({'turnAbandonedTakeControl', 'stalledTakeControl'} & prompts(after))
