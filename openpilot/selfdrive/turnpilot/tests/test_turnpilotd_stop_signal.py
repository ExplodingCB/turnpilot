"""The turn signal at a stop sign the decider acts for (car_signals.py with stop_go.py; DESIGN phase4/blinker 6.6, 8 and
STOP_SIGNS.md D15): the car's lamp, or the driver's prompt, from the turn's arming through the stop, the decider's go and the
turn; the go never waiting on the lamp (D15); and at a stop-sign left the autonomous left standing aside, so the decider's go
is the turn's first pulse and the turn is never skipped mid-launch with the lamp going dark. test_turnpilotd_stop's StopSim:
the surveyed all-way, the car following the plan."""
import pytest

from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Desire
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import S_LINE, StopSim, go_frame, run_to_line

LaneChange = custom.TurnpilotPlan.LaneChange
Status = custom.TurnpilotPlan.TurnSignal.Status
WANT = {'left': (LaneChange.left, Desire.turnLeft, 1.), 'right': (LaneChange.right, Desire.turnRight, -1.)}
OTHER = {'left': 'right', 'right': 'left'}


def stop_turn_sim(tmp_path, maneuver, car_signals=True, auto_left='auto', model='goes'):
  """A turn at the stop sign, 13 m past its stop point, at a four-way (an oncoming road: the autonomous left's turn), under
  TurnPilotAutoLeft auto as on the car. car_signals: a verified Tesla whose lamp lights on the plan's side at once; else mode
  off, the driver prompted (and not answering unless the test says)."""
  sim = StopSim(tmp_path, model=model, info_kw={'maneuver': maneuver}, driver_signals=None,
                maneuvers=[(S_LINE + 13., 'turn', maneuver, (0., 270. if maneuver == 'left' else 90., 2, 'present', 11.2, 1)),
                           (2000., 'turn', OTHER[maneuver], (0., 90. if maneuver == 'left' else 270., 2, 'present'))])
  sim.tp.auto_left, sim.tp.left_range_m = auto_left, 300.
  sim.turn_done = False
  if car_signals:
    sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'

    def port(s):
      side = s.tp.signal_out.side
      return {'status': 'lit' if side != LaneChange.none else 'ready', 'left': side == LaneChange.left,
              'right': side == LaneChange.right, 'leftLamp': side == LaneChange.left, 'rightLamp': side == LaneChange.right}
    sim.car_signal = port
  else:
    sim.tp.car_signal_mode = 'off'
  return sim


def turn_step(sim, maneuver):
  """One frame, the car yawing into the turn on a 9 m radius while its desire is out, from the decider's commit point on (the
  sim's views and visibility file are for a car straight on its approach, so a launch yawed before it reads the lanes short)."""
  d, sg = sim.tp.desires, sim.tp.sg_now
  turning = d.started and not d.done and (sg is None or sg.state in ('committed', 'inLane', 'done', 'off'))
  plan = sim.step(yaw_rate=WANT[maneuver][2] * sim.v / 9. if turning else 0.)
  sim.turn_done = sim.turn_done or (d.maneuver_id == sim.tp.follower.maneuver_id and d.done and d.turned_deg >= 45.)
  return plan


def run(sim, maneuver, seconds):
  for _ in range(round(seconds / DT_MDL)):
    turn_step(sim, maneuver)


@pytest.mark.parametrize('auto_left', ['auto', 'off'])
@pytest.mark.parametrize('maneuver', ['left', 'right'])
def test_a_signalled_turn_at_a_stop_sign_is_lit_from_its_arming_through_the_go_and_the_turn(tmp_path, maneuver, auto_left):
  side, desire, _ = WANT[maneuver]
  sim = stop_turn_sim(tmp_path, maneuver, auto_left=auto_left)
  run_to_line(sim)
  run(sim, maneuver, 20.)
  plans = sim.plans()
  k = go_frame(sim)
  assert k is not None, "the decider never went"
  armed = next(i for i, p in enumerate(plans) if p.turnSignal.side == side)
  assert plans[armed].maneuverDistance > 91.44  # 300 ft before the turn plus the lamp's lag, well before the stop sign
  done = next(i for i in range(k, len(plans)) if plans[i].turnSignal.side == LaneChange.none)
  # lit without a gap from its arming, through the stop, the wait, the go and the launch, until the turn is done
  assert all(p.turnSignal.side == side and p.turnSignal.status == Status.ready for p in plans[armed:done])
  assert sim.turn_done  # cleared at its angle, 45 deg or more into it
  assert all(p.turnSignal.prompt == LaneChange.none for p in plans)  # the car signals: the driver is never asked
  # the decider's go is the turn's first pulse, and the turn is never skipped (the autonomous left's unprotectedStop among
  # them: it stands aside at a stop sign the decider acts for, so its pulse hold and its hold line never come into it)
  assert all(p.desire == Desire.none for p in plans[:k]) and plans[k].desire == desire
  assert not any(p.tooFastToTurn for p in plans) and not sim.tp.skip.skipped
  assert all(str(p.stopGo.maneuver) == maneuver for p in plans[k:done] if str(p.stopGo.state) != 'off')
  # 300 ft lit before the car moves into the turn
  assert plans[k].turnSignal.litM >= plans[k].turnSignal.leadM
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


@pytest.mark.parametrize('maneuver', ['left', 'right'])
def test_mode_off_prompts_a_turn_at_a_stop_sign_and_the_prompt_never_gates_the_go(tmp_path, maneuver):
  # D15: the driver is asked from under 20 mph, through the stop and the wait; the go comes when the junction is clear,
  # whether or not the driver signals (with LEFT_GO_NEEDS_SIGNAL the autonomous left's own line is another matter: a stop
  # sign's left is the decider's, never 'unsignalled')
  side, desire, _ = WANT[maneuver]
  frames = {}
  for answers in (False, True):
    sim = stop_turn_sim(tmp_path, maneuver, car_signals=False)
    sim.answers_prompt = answers
    run_to_line(sim)
    run(sim, maneuver, 14.)
    plans = sim.plans()
    k = frames[answers] = go_frame(sim)
    assert k is not None and plans[k].desire == desire
    held = [p for p in plans[:k] if p.shouldStop]
    assert held and all(p.turnSignal.prompt == (LaneChange.none if answers else side) for p in held)  # answered: lit, asked no more
    assert all(p.turnSignal.status == Status.off for p in plans)
    assert not any(p.tooFastToTurn for p in plans)
    assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or str(p.unprotectedLeft.state) == 'undecided' for p in plans)
  assert frames[False] == frames[True]
