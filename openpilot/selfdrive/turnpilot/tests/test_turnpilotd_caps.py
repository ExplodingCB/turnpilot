"""TurnPilot's speed caps that outlasted their maneuver (2026-11-06, the 11-06 drives): a keep the model was left to drive
(keepFollow) kept its corner cap to its pass, binding 8.1 s (1e) and 16.7 s (1d) with the model asking +0.3 to +1.0 m/s^2 over
it; and a junction turn kept its corner floor (V_CORNER, a right's V_CORNER_CURB) out to d -6.5 to -8.8 m after its desire had
cleared at its angle, binding 7.5 s of 1e. Now a keepFollow keeps no cap (comma first: the model drives the fork at its own
pace), and a turn turned to its angle at its junction (turn_speed.turn_done_deg, DesireTiming's clear within ENTRY_M of it)
lets its cap go; a turn cancelled or abandoned short of its angle, one cleared early on its approach, and the plain keep skip,
keep theirs to the pass. Closed loop on test_turnpilotd's Sim."""
from unittest import mock

from openpilot.cereal import log
from openpilot.selfdrive.turnpilot import turnpilotd
from openpilot.selfdrive.turnpilot.desire_timing import ENTRY_M
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE, KEEP_FOLLOW
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_keep import KEEP_J, follow_sim, keep_sim, run_keep
from openpilot.selfdrive.turnpilot.turn_speed import CORNER_M, NO_SPEED_CAP, V_CORNER, V_CORNER_CURB, turn_done_deg, turn_speed_cap

Desire = log.Desire
JUNCTION = 300.
RIGHT = [(JUNCTION, "turn", "right", junction_of("right")), (1500., "turn", "left", junction_of("left", opposing='none'))]
T_LEFT = [(JUNCTION, "turn", "left", junction_of("left", opposing='none')), (1500., "turn", "right", junction_of("right"))]


def test_a_keep_the_model_is_left_to_drive_keeps_no_cap():
  # under TurnPilotAlwaysDrive the keep that can't be sent is the model's to drive (keepFollow): from its skip on, no corner cap
  sim = follow_sim(line_prob=0.1)
  run_keep(sim, KEEP_J + 40.)
  said = [i for i, (_, _, p) in enumerate(sim.trace) if p.tooFastToTurn]
  assert said and str(sim.trace[said[0]][2].skipReason) == KEEP_FOLLOW
  first_s = sim.trace[said[0]][0]
  before = [p for s, _, p in sim.trace if s < first_s and str(p.maneuverModifier) == 'slight right' and 0. < p.maneuverDistance < 60.]
  assert before and any(p.speedCap < NO_SPEED_CAP for p in before), "the town keep's cap on its approach, as before"
  # from the frame after the skip's (the skip is decided after that frame's cap, as every skip is), the fork's included
  on_keep = [p for s, _, p in sim.trace[said[0] + 1:] if str(p.maneuverModifier) == 'slight right' and p.routeValid]
  assert on_keep and all(p.speedCap == NO_SPEED_CAP for p in on_keep), "no cap past the skip"
  # the model's own pace: the car gets back up toward cruise past the skip (the cap held it to the pass before)
  v_skip = sim.trace[said[0]][1]
  assert max(v for s, v, _ in sim.trace if s >= first_s) > v_skip + 0.5


def test_a_plain_keep_skip_keeps_its_cap_to_its_pass():
  # the switch off: the keep that can't be sent is skipped out loud and rerouted, its cap kept to its pass (never sped up into it)
  sim = keep_sim(line_prob=0.1)
  run_keep(sim, KEEP_J + 40.)
  said = [i for i, (_, _, p) in enumerate(sim.trace) if p.tooFastToTurn]
  assert said and str(sim.trace[said[0]][2].skipReason) == 'keepNoLine'
  first_s = sim.trace[said[0]][0]
  on_keep = [p for s, _, p in sim.trace if s >= first_s and str(p.maneuverModifier) == 'slight right' and p.routeValid]
  assert on_keep and all(p.speedCap < NO_SPEED_CAP for p in on_keep if p.maneuverDistance > 0.)


def through_a_turn(maneuvers, side, cancel_at_start=False, yaw=True):
  """Engaged at 8 m/s through the route's first turn at an 8 m radius once its desire is out within 12 m of the junction (no
  yaw: straight on), until 12 m past it: [(distance, desire done, turned deg, plan, v, the follower's maneuver)]. cancel_at_start:
  the desire cancelled
  (DesireTiming.cancel, a veto's skip with it out) on the frame it first goes out."""
  sim = Sim(maneuvers, cruise=8.)
  out, cancelled = [], False
  while sim.s < JUNCTION + 12.:
    d = sim.tp.desires
    if cancel_at_start and d.started and not cancelled:
      d.cancel()
      cancelled = True
    turning = yaw and d.started and not d.done and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    plan = sim.step(engaged=True, yaw_rate=-side * sim.v / 8. if turning else 0.)
    d = sim.tp.desires
    out.append((plan.maneuverDistance, d.done, d.turned_deg, plan, sim.v, sim.tp.follower.upcoming))
    assert sim.t < 120.
  return sim, out


def test_a_turn_turned_to_its_angle_lets_its_cap_go_before_its_pass():
  for maneuvers, side, floor in ((RIGHT, 1, V_CORNER_CURB), (T_LEFT, -1, V_CORNER)):  # a right's corner speed: 2.97, derived (2026-11-07)
    sim, out = through_a_turn(maneuvers, side)
    done = [(dist, plan, v, up) for dist, is_done, turned, plan, v, up in out if is_done and turned >= turn_done_deg(90.) - 1e-6 and
            str(plan.maneuverType) == 'turn' and dist < 0.]
    assert len(done) > 20, "turned to its angle a second and more before navd's pass"
    # its corner floor would bind there (the turn's own cap at d < 0), and it is gone: the model's pace from its angle on, from
    # the frame after the one it gets there on (the cap reads last frame's desire state, a frame late at most)
    for dist, plan, v, up in done[1:]:
      assert plan.speedCap == NO_SPEED_CAP, (side, dist, plan.speedCap)
      assert abs(turn_speed_cap(up, dist, v, True) - floor) < 1e-6, (side, dist)  # what bound it there before 2026-11-06
    # before its angle the cap is on, as before
    turning = [plan for dist, is_done, turned, plan, v, up in out if 0. < dist < 10. and not is_done]
    assert turning and all(p.speedCap < NO_SPEED_CAP for p in turning), side
    assert done[-1][2] > done[0][2] + 0.5 and done[-1][2] > floor  # free to speed up out of the turn, past its old floor


def test_a_turn_cleared_early_on_an_approach_bend_keeps_its_corner_cap():
  # review 2026-11-06: DesireTiming counts the yaw from the first pulse, so a bend toward the turn's side on the approach (or
  # an opening taken on the way: 1e's first left cleared at d 21.8 m) clears its desire at its angle short of the junction. The
  # release needs the clear within ENTRY_M of its junction and the car there: cleared 20 m and more out, the T-left keeps its
  # corner floor (V_CORNER) to its pass, never lifted as the car reaches the junction it hasn't turned at
  sim = Sim(T_LEFT, cruise=8.)
  out, cleared_at = [], None
  with mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    while sim.s < JUNCTION + 12.:
      d = sim.tp.desires
      bend = d.started and not d.done and 22. < JUNCTION - sim.s < 36. and sim.v > 0.3  # a 12 m arc at an 8 m radius
      plan = sim.step(engaged=True, yaw_rate=sim.v / 8. if bend else 0.)
      if cleared_at is None and sim.tp.desires.done:
        cleared_at = plan.maneuverDistance
      out.append((plan.maneuverDistance, sim.v, plan, sim.tp.follower.upcoming))
      assert sim.t < 120.
  assert cleared_at is not None and cleared_at > ENTRY_M + 5., cleared_at
  assert sim.tp.desires.turned_deg >= turn_done_deg(90.) - 1e-6 or any(p.desire == CLEAR_DESIRE for _, _, p, _ in out)
  near = [(dist, v, plan, up) for dist, v, plan, up in out if str(plan.maneuverType) == 'end of road' or str(plan.maneuverType) == 'turn']
  near = [(dist, v, plan, up) for dist, v, plan, up in near if -8. < dist < ENTRY_M and up is not None]
  assert near and all(plan.speedCap < NO_SPEED_CAP for _, _, plan, _ in near), "its corner cap at the junction"
  assert any(abs(plan.speedCap - turn_speed_cap(up, dist, v, True)) < 1e-6 and dist < CORNER_M for dist, v, plan, up in near)
  assert not [c for c in event.call_args_list if c.args[0] == 'turnpilot.turn_cap_released']


def test_a_turn_cancelled_short_of_its_angle_keeps_its_cap_to_its_pass():
  # its desire cancelled (a veto's skip with it out: DesireTiming.cancel sets done) and the car driven straight on: done, never
  # turned to its angle, so its corner floor stays to the pass (never sped up into a junction it didn't turn at)
  sim, out = through_a_turn(RIGHT, 1, cancel_at_start=True, yaw=False)
  past = [(dist, plan) for dist, is_done, turned, plan, v, up in out if is_done and str(plan.maneuverType) == 'turn' and -8. < dist < 0.]
  assert past and all(plan.speedCap < NO_SPEED_CAP for _, plan in past), [round(p.speedCap, 2) for _, p in past]
  assert all(turned < turn_done_deg(90.) for _, is_done, turned, _, _, _ in out)
