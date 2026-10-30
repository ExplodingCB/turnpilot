"""The turn signal at a stop sign the decider acts for (car_signals.py with stop_go.py; DESIGN phase4/blinker 6.6, 8 and
STOP_SIGNS.md D15): the car's lamp, or the driver's prompt, from the turn's arming through the stop, the decider's go and the
turn; the go never waiting on the lamp (D15); a veto at the stop skipping the turn out loud with the decider then judging
every path the car might take, never the vetoed turn's alone; a veto after the go finishing the turn; and at a stop-sign
left the autonomous left standing aside, so the decider's go is the turn's first pulse and the turn is never skipped
mid-launch with the lamp going dark. test_turnpilotd_stop's StopSim: the surveyed all-way, the car following the plan."""
import pytest

from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Desire
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import S_LINE, StopSim, go_frame, run_to_line

LaneChange = custom.TurnpilotPlan.LaneChange
Status = custom.TurnpilotPlan.TurnSignal.Status
WANT = {'left': (LaneChange.left, Desire.turnLeft, 1.), 'right': (LaneChange.right, Desire.turnRight, -1.)}
OTHER = {'left': 'right', 'right': 'left'}
VETOES = {
  'driverSignal': lambda m: {'status': 'ready', 'block': 'otherSide', f'{OTHER[m]}Lamp': True},
  'hazards': lambda m: {'status': 'ready', 'block': 'hazard', 'hazard': True, 'leftLamp': True, 'rightLamp': True},
  'signalLost': lambda m: {'status': 'failed', 'fault': 'lampLost'},
}


def stop_turn_sim(tmp_path, maneuver, car_signals=True, auto_left='auto', model='goes'):
  """A turn at the stop sign, 13 m past its stop point, at a four-way (an oncoming road: the autonomous left's turn), under
  TurnPilotAutoLeft auto as on the car. car_signals: a verified Tesla whose lamp lights on the plan's side at once; else mode
  off, the driver prompted (and not answering unless the test says)."""
  sim = StopSim(tmp_path, model=model, info_kw={'maneuver': maneuver}, driver_signals=None,
                maneuvers=[(S_LINE + 13., 'turn', maneuver, (0., 270. if maneuver == 'left' else 90., 2, 'present', 11.2, 1)),
                           (2000., 'turn', OTHER[maneuver], (0., 90. if maneuver == 'left' else 270., 2, 'present'))])
  sim.tp.auto_left, sim.tp.left_range_m = auto_left, 300.
  sim.veto, sim.turn_done = None, False
  if car_signals:
    sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'

    def port(s):
      if s.veto is not None:
        return dict(s.veto)
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


def until(sim, maneuver, cond, limit_s=40.):
  t0 = sim.t
  while not cond(sim):
    turn_step(sim, maneuver)
    assert sim.t - t0 < limit_s, "never happened"


def observing(sim):
  return sim.tp.sg_now is not None and sim.tp.sg_now.state == 'observe' and sim.v < 0.05


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


@pytest.mark.parametrize('when', ['arrived', 'clearing'])
@pytest.mark.parametrize('veto', sorted(VETOES))
@pytest.mark.parametrize('maneuver', ['left', 'right'])
def test_a_veto_at_the_stop_sign_skips_the_turn_and_the_decider_then_judges_every_path(tmp_path, maneuver, veto, when):
  # held at the line, the decider observing for the turn (from its arrival, or with its lanes clear for most of CLEAR_S): the
  # driver's lamp the other way, hazards or our lamp put out skip the turn out loud (navd reroutes) and the hold's go is
  # straight on, never the vetoed turn. The decider judged the turn's conflicting lanes alone; it now judges the union of
  # every maneuver ('unknown', as off the route) through the launch, even once navd moves on to the next maneuver as the car
  # leaves the junction (arrived: inLane), its CLEAR_S from the veto, and goes on that, never on the turn's lanes with the car
  # crossing straight, nor half a second after the veto on the time the turn's lanes were clear (clearing)
  sim = stop_turn_sim(tmp_path, maneuver)
  run_to_line(sim)
  until(sim, maneuver, lambda s: observing(s) and (when == 'arrived' or s.tp.stop_go.clear_s >= sr.CLEAR_S - 0.5))
  n0 = len(sim.trace)
  sim.veto = VETOES[veto](maneuver)
  run(sim, maneuver, 0.2)
  assert sim.tp.skip.skipped and sim.tp.skip.reason == veto
  run(sim, maneuver, 16.)
  plans = sim.plans()
  after = plans[n0:]
  assert any(p.tooFastToTurn for p in after[:3]) and str(after[1].turnSignal.skip) == veto
  assert not any(p.desire in (Desire.turnLeft, Desire.turnRight) for p in plans)
  assert any(str(p.stopGo.state) == 'done' for p in after)  # across the junction and out of it
  assert all(str(p.stopGo.maneuver) == 'unknown' for p in after[2:] if str(p.stopGo.state) != 'off')
  k = go_frame(sim)
  assert k is not None and k > n0, "the decider never went straight on"
  assert (k - n0) * DT_MDL >= sr.CLEAR_S - 0.1  # the union's go conditions held CLEAR_S from the veto
  assert all(p.turnSignal.side == LaneChange.none for p in plans[k:])
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in after)


@pytest.mark.parametrize('maneuver', ['left', 'right'])
def test_a_veto_after_the_deciders_go_finishes_the_turn_out_loud(tmp_path, maneuver):
  # pulling away into the turn on the decider's go, its desire out and no stop in force: the turn has started, so a veto is
  # "Turn Signal Changed" and the turn finishes (DESIGN 6.6: one decision per maneuver), never a skip in the junction
  sim = stop_turn_sim(tmp_path, maneuver)
  run_to_line(sim)
  until(sim, maneuver, lambda s: go_frame(s) is not None)
  until(sim, maneuver, lambda s: s.v > 0.5)
  sim.veto = VETOES['driverSignal'](maneuver)
  run(sim, maneuver, 8.)
  plans = sim.plans()
  assert any(str(p.turnSignal.notice) == 'changedMidTurn' for p in plans)
  assert not sim.tp.skip.skipped and not any(p.tooFastToTurn for p in plans)
  assert sim.turn_done  # cleared at its angle, 45 deg or more into it


@pytest.mark.parametrize('maneuver', ['left', 'right'])
def test_a_veto_while_a_withdrawn_go_brings_the_car_to_rest_skips_the_turn(tmp_path, maneuver):
  # the model never takes over: the decider's go is withdrawn and its capped abort stops the car short of the first
  # conflicting lane (stop_go.capped_abort), the turn's desire out from the go. A veto while it brakes is a veto at a stop
  # (DESIGN 6.6), so the turn is skipped and its desire cleared, never "Finishing the turn" and then the decider's next go
  # taking the turn the driver overruled
  sim = stop_turn_sim(tmp_path, maneuver, model='holds')
  run_to_line(sim)

  def aborting(s):
    sg = s.tp.stop_go
    return sg.aborting is not None and s.v > 0.1 and s.tp.desires.started
  until(sim, maneuver, aborting)
  n0 = len(sim.trace)
  sim.veto = VETOES['driverSignal'](maneuver)
  run(sim, maneuver, 1.)
  plans = sim.plans()
  assert sim.tp.skip.skipped and sim.tp.skip.reason == 'driverSignal'
  assert any(p.tooFastToTurn for p in plans[n0:]) and not any(str(p.turnSignal.notice) == 'changedMidTurn' for p in plans)
  assert any(p.desire != Desire.none for p in plans[n0:]) and plans[-1].desire == Desire.none  # cleared from the model's queue
  run(sim, maneuver, 20.)
  assert not any(p.desire in (Desire.turnLeft, Desire.turnRight) for p in sim.plans()[n0 + round(1. / DT_MDL):])
