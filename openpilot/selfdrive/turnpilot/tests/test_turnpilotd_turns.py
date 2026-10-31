"""ALWAYS_DRIVE.md S3's turns (R8b): a route turn the model drives straight through is a turn missed and a reroute at the
follower's pass, not a 'Turn abandoned' handback, while the car stays inside both of R8b's bounds on every frame: turned
under 15 deg toward the turn and under TURN_MISSED_AWAY_DEG away from it (turned_deg is signed toward the turn), and
modelV2.action.desiredCurvature under TURN_MISSED_CURV either way. Under TurnPilotAlwaysDrive R8's fire inside the bounds
clears the desire (CLEAR_DESIRE) and hands nothing back; either bound broken after it is R8's handback, latched as today. A
committed turn driven dead straight that never fires R8 is missed at the pass with the switch off as well as on (R9's detector
catching more), its last pulse cleared from the model's queue on that frame (WARGAME_FIXES WF-11)."""
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import RIGHT_AT_A_CORNER, Sim, into_a_right, stand
from openpilot.selfdrive.turnpilot.turnpilotd import (MISSED_HOLD_S, STALL_PROMPT_S, STALLED_REASON, TURN_ABANDON_S,
                                                       TURN_ABANDONED_REASON, TURN_MISSED_AWAY_DEG, TURN_MISSED_CURV,
                                                       TURN_MISSED_REASON)

Desire = log.Desire
JUNCTION = 300.  # RIGHT_AT_A_CORNER's right
PASS_S = JUNCTION + 20.  # past the follower's pass (15 m past the furthest reading of the junction)


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
