"""The turn signal at a stop sign the decider acts for (car_signals.py with stop_go.py; DESIGN phase4/blinker 6.6, 8 and
STOP_SIGNS.md D15): the car's lamp, or the driver's prompt, from the turn's arming through the stop, the decider's go and the
turn; the go never waiting on the lamp (D15); a veto at the stop skipping the turn out loud with the decider then judging
every path the car might take, never the vetoed turn's alone; a veto while the decider's go can still be withdrawn
withdrawing it and skipping the turn the same way, one past its commit point finishing the turn; and at a stop-sign
left the autonomous left standing aside, so the decider's go is the turn's first pulse and the turn is never skipped
mid-launch with the lamp going dark. test_turnpilotd_stop's StopSim: the surveyed all-way, the car following the plan."""
import pytest

from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Desire
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import S_LINE, StopSim, go_frame, other_turn_drive, run_to_line

LaneChange = custom.TurnpilotPlan.LaneChange
Status = custom.TurnpilotPlan.TurnSignal.Status
WANT = {'left': (LaneChange.left, Desire.turnLeft, 1.), 'right': (LaneChange.right, Desire.turnRight, -1.)}
OTHER = {'left': 'right', 'right': 'left'}
VETOES = {
  'driverSignal': lambda m: {'status': 'ready', 'block': 'otherSide', f'{OTHER[m]}Lamp': True},
  'hazards': lambda m: {'status': 'ready', 'block': 'hazard', 'hazard': True, 'leftLamp': True, 'rightLamp': True},
  'signalLost': lambda m: {'status': 'failed', 'fault': 'lampLost'},
}
# the opposing road mapped straight this far (navInstruction maneuverOpposingStraightM, 2c94f6b), as the lefts that go in
# test_turnpilotd_left carry: under TurnPilotAutoLeft auto a left whose opposing road isn't known straight past its R_req is
# skipped at route time (unprotectedRange), the four-way's included, before the decider ever acts there
STRAIGHT_M = 400.


def stop_turn_sim(tmp_path, maneuver, car_signals=True, auto_left='auto', model='goes'):
  """A turn at the stop sign, 13 m past its stop point, at a four-way (an oncoming road: the autonomous left's turn), under
  TurnPilotAutoLeft auto as on the car. car_signals: a verified Tesla whose lamp lights on the plan's side at once; else mode
  off, the driver prompted (and not answering unless the test says)."""
  sim = StopSim(tmp_path, model=model, info_kw={'maneuver': maneuver}, driver_signals=None,
                maneuvers=[(S_LINE + 13., 'turn', maneuver, (0., 270. if maneuver == 'left' else 90., 2, 'present', 11.2, 1, STRAIGHT_M)),
                           (2000., 'turn', OTHER[maneuver], (0., 90. if maneuver == 'left' else 270., 2, 'present', 0., 0, STRAIGHT_M))])
  # a range past route time's R_req for the left after a right here (its opposing road unmapped: 45 mph, two lanes, 350 m
  # from LINE_D plus ROUTE_REACH_MARGIN_M, BUILD_FINDINGS D4), so it isn't skipped for range once it is the maneuver
  sim.tp.auto_left, sim.tp.left_range_m = auto_left, 360.
  sim.veto, sim.turn_done = None, False
  if car_signals:
    car_signals_on(sim)
  else:
    sim.tp.car_signal_mode = 'off'
  return sim


def car_signals_on(sim):
  """A verified Tesla whose lamp lights on the plan's side at once; sim.veto, when set, is the port's carSignal instead."""
  sim.veto = getattr(sim, 'veto', None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'

  def port(s):
    if s.veto is not None:
      return dict(s.veto)
    side = s.tp.signal_out.side
    return {'status': 'lit' if side != LaneChange.none else 'ready', 'left': side == LaneChange.left,
            'right': side == LaneChange.right, 'leftLamp': side == LaneChange.left, 'rightLamp': side == LaneChange.right}
  sim.car_signal = port


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
  # ...and no more: the skipped turn's union never waits for navd's reroute (SG4b's off-route OFFROUTE_WAIT_S), the car
  # going straight on whatever it says (merged with tp/integrate: 10 s more at the line, "Waiting: the reroute")
  assert (k - n0) * DT_MDL < sr.CLEAR_S + 0.5 and not any("reroute" in p.reason for p in after)
  assert all(p.turnSignal.side == LaneChange.none for p in plans[k:])
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in after)


def vetoed_in_going(sim, maneuver, veto, n0):
  """After a veto at n0 in the decider's 'going': its outcome over the next 16 s (the plans from n0), checked the same way
  whenever the go could still be withdrawn: the go withdrawn for it (signalVeto), its capped abort bringing the car to rest
  short of the closest plausible edge (dEdgeMin), the turn skipped out loud for the veto's reason with its desire cleared,
  never "Turn Signal Changed" / "Finishing the turn", the decider then judging every path (unknown) and its next go straight
  on no sooner than CLEAR_S after the car came to rest, the turn never taken, no handback."""
  go_s = sim.trace[go_frame(sim)][0]
  d_min = sim.tp.stop_go.edges.d_min
  run(sim, maneuver, 0.2)
  assert sim.tp.skip.skipped and sim.tp.skip.reason == veto, (sim.tp.skip.skipped, sim.tp.skip.reason)
  run(sim, maneuver, 16.)
  plans = sim.plans()
  after = plans[n0:]
  notices = sorted({str(p.turnSignal.notice) for p in after})
  assert not any(str(p.turnSignal.notice) == 'changedMidTurn' for p in after), f"'Finishing the turn' for {veto} ({notices})"
  assert after[0].tooFastToTurn and str(after[1].turnSignal.skip) == veto
  assert any(str(p.stopGo.withdrawn) == 'signalVeto' for p in after[:3]), sorted({str(p.stopGo.withdrawn) for p in after})
  assert any(p.desire != Desire.none for p in after[:3])  # CLEAR_DESIRE: the go's pulse out of the model's queue
  rest = next(i for i in range(len(after)) if sim.trace[n0 + i][1] < 0.05)
  assert sim.trace[n0 + rest][0] - go_s < d_min - sr.M_EDGE + 0.05, "the abort stops the car short of the first conflicting lane"
  assert all(str(p.stopGo.maneuver) == 'unknown' for p in after[2:] if str(p.stopGo.state) != 'off')
  goes = [i for i in range(rest + 1, len(after)) if str(after[i].goCertified.cause) == 'stopSign' and
          str(after[i - 1].goCertified.cause) != 'stopSign']
  assert goes and (goes[0] - rest) * DT_MDL >= sr.CLEAR_S - 0.1, "the decider's next go, straight on, on the union's CLEAR_S"
  assert not any(p.desire in (Desire.turnLeft, Desire.turnRight) for p in after[1:]), "the go's pulse past the veto's frame"
  assert not sim.turn_done
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in after)


@pytest.mark.parametrize('veto', sorted(VETOES))
@pytest.mark.parametrize('maneuver', ['left', 'right'])
def test_a_veto_while_the_deciders_go_can_still_withdraw_skips_the_turn(tmp_path, maneuver, veto):
  # pulling away into the turn on the decider's go (state 'going': not committed, its capped abort still stopping short of
  # dEdgeMin, the closest plausible edge of the first conflicting lane): the go is withdrawable by the decider's own
  # definition, and the veto is one more thing it was made on that no longer holds. Withdrawn (signalVeto) and skipped out
  # loud as at the line, never "Finishing the turn" across the oncoming lanes with the other lamp, hazards or no lamp lit
  # (verification of tp/blinker, M2: B10's rule at every stop-sign left, the decider's go there being the turn's)
  sim = stop_turn_sim(tmp_path, maneuver)
  run_to_line(sim)
  until(sim, maneuver, lambda s: go_frame(s) is not None)
  until(sim, maneuver, lambda s: s.v > 0.5)
  sg = sim.tp.stop_go
  assert sg.state == 'going', sg.state
  room = sg.edges.d_min - (sim.s - sim.trace[go_frame(sim)][0])
  need = sr.abort_distance(sim.v, sim.tp.cp_delay, 0.5) + sr.M_EDGE
  assert need < room, f"the decider's capped abort still stops short of dEdgeMin: needs {need:.2f} m of {room:.2f}"
  n0 = len(sim.trace)
  sim.veto = VETOES[veto](maneuver)
  vetoed_in_going(sim, maneuver, veto, n0)


@pytest.mark.parametrize('maneuver', ['left', 'right'])
def test_a_veto_on_the_deciders_go_with_the_car_still_standing_withdraws_it_and_skips_the_turn(tmp_path, maneuver):
  # the decider's go is out (its pulse, goCertified stopSign) with the car not yet moving: standing where it was held, the
  # go can always be withdrawn, so the veto withdraws it and skips the turn out loud, the car held there; never a turn
  # finished on a veto the car could have stopped for
  sim = stop_turn_sim(tmp_path, maneuver)
  run_to_line(sim)
  until(sim, maneuver, lambda s: go_frame(s) is not None)
  assert sim.v < 0.3 and sim.tp.go_id == sim.tp.follower.maneuver_id, (sim.v, sim.tp.go_id)
  n0 = len(sim.trace)
  sim.veto = VETOES['driverSignal'](maneuver)
  vetoed_in_going(sim, maneuver, 'driverSignal', n0)
  assert sim.trace[n0][1] < 0.3, "the veto came with the car still standing"


@pytest.mark.parametrize('maneuver', ['left', 'right'])
def test_a_veto_after_the_deciders_commit_finishes_the_turn_out_loud(tmp_path, maneuver):
  # past the commit point (the capped abort can no longer stop short of dEdgeMin) the go can't be withdrawn: the turn has
  # started, so a veto is "Turn Signal Changed" and the turn finishes (DESIGN 6.6: one decision per maneuver), never a skip
  # nor a stop in the junction
  sim = stop_turn_sim(tmp_path, maneuver)
  run_to_line(sim)
  until(sim, maneuver, lambda s: go_frame(s) is not None)
  until(sim, maneuver, lambda s: s.tp.stop_go.state == 'committed')
  sim.veto = VETOES['driverSignal'](maneuver)
  run(sim, maneuver, 8.)
  plans = sim.plans()
  assert any(str(p.turnSignal.notice) == 'changedMidTurn' for p in plans)
  assert not sim.tp.skip.skipped and not any(p.tooFastToTurn for p in plans)
  assert not any(str(p.stopGo.withdrawn) == 'signalVeto' for p in plans)
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


# test_turnpilotd_stop.other_turn_drive's other turn for each route maneuver (route left: the right; right: the left; straight:
# the right), and the route's own turn's side
ALT = {'left': 'right', 'right': 'left', 'straight': 'right'}
SIDE = {'left': LaneChange.left, 'right': LaneChange.right, 'straight': LaneChange.none}


@pytest.mark.parametrize('route', ['left', 'right', 'straight'])
def test_the_ladders_other_turn_is_the_turn_the_car_signals_never_the_routes_it_replaced(tmp_path, route):
  # SG4c's other turn (STOP_SIGNS.md 7.1 rung 2, O12) came after tp/blinker merged e639725, and the merge left the turn signal
  # on the route's turn: the car turned right instead of the route's left with its left lamp lit through the go and the
  # turn. From the frame it is taken (otherTurn) the other turn is the one the car signals, armed at once (DESIGN
  # phase4/blinker 8, item 2), lit through the go and the commit until it is done; the route's turn it replaced at this
  # junction is never lit again, through the planned turn's miss
  plans, go, commit, _, _ = other_turn_drive(tmp_path, route, setup=car_signals_on)
  alt = next(k for k, p in enumerate(plans) if str(p.controlAlert) == 'otherTurn')
  want = SIDE[ALT[route]]
  done = next(k for k in range(go, len(plans)) if plans[k].turnSignal.side != want)
  assert done > commit, (alt, go, commit, done)
  assert all(p.turnSignal.side == want and p.turnSignal.status == Status.ready for p in plans[alt + 1:done])
  if route != 'straight':
    assert not any(p.turnSignal.side == SIDE[route] for p in plans[alt + 1:])
  assert not any(p.turnSignal.prompt != LaneChange.none for p in plans)


@pytest.mark.parametrize('route', ['left', 'right'])
def test_with_the_car_not_signalling_the_other_turn_is_the_one_the_driver_is_asked_to_signal(tmp_path, route):
  # mode off: "Signal Left" for the route's left until the other turn is taken, then "Signal Right" for the right it takes,
  # never the route's turn again (the driver not answering, so the prompt holds on every frame standing at the line)
  plans, go, _, _, _ = other_turn_drive(tmp_path, route, setup=lambda s: setattr(s.tp, 'car_signal_mode', 'off'))
  alt = next(k for k, p in enumerate(plans) if str(p.controlAlert) == 'otherTurn')
  assert any(p.turnSignal.prompt == SIDE[route] for p in plans[:alt])
  assert all(p.turnSignal.prompt == SIDE[ALT[route]] for p in plans[alt + 1:go])
  assert not any(p.turnSignal.prompt == SIDE[route] for p in plans[alt + 1:])


@pytest.mark.parametrize('when', ['taken', 'going'])
@pytest.mark.parametrize('veto', sorted(VETOES))
@pytest.mark.parametrize('route', ['left', 'straight'])
def test_a_veto_of_the_other_turn_before_it_starts_hands_back_held_and_the_car_never_takes_it(tmp_path, route, veto, when):
  # the driver's lamp the other way, hazards or our lamp put out once the other turn is taken (standing at the line, or on its
  # go while the decider can still withdraw it): the decider never takes it, withdraws a go out for it (signalVeto) and,
  # standing, hands back with the car held (H3, "turn cancelled"), the request for its lamp ended then and there, never
  # handed over; never a skip of the route's maneuver (route straight: a turn 1000 m on), nor the turn the driver overruled
  st = {'n0': None}

  def each(s):
    sg = s.tp.stop_go
    if st['n0'] is None and sg.alt is not None and (when == 'taken' or (sg.state == 'going' and s.tp.sg_now.withdrawable)):
      st['n0'] = len(s.trace)
      s.veto = VETOES[veto](ALT[route])
    return st['n0'] is not None and len(s.trace) - st['n0'] >= round(12. / DT_MDL)
  plans, _, _, _, _ = other_turn_drive(tmp_path, route, setup=car_signals_on, each=each)
  n0 = st['n0']
  assert n0 is not None, "the other turn was never taken"
  after = plans[n0:]
  assert not any(p.desire in (Desire.turnLeft, Desire.turnRight) for p in after[1:]), "a turn's pulse past the veto's frame"
  assert all(p.turnSignal.side == LaneChange.none and p.turnSignal.handedOver == LaneChange.none for p in after[1:])
  assert not any(p.tooFastToTurn for p in after), "a route maneuver skipped for the other turn's veto"
  held = [k for k, p in enumerate(after) if str(p.controlPrompt) == 'stopSignTakeControl']
  assert held and held[0] * DT_MDL < 6. and "turn cancelled" in after[held[0]].reason
  assert all(p.shouldStop for p in after[held[0] + 20:]), "the handback holds the car"
  assert all(p.accelCap >= -2. - 1e-9 for p in after)
  if when == 'going':
    assert any(str(p.stopGo.withdrawn) == 'signalVeto' for p in after[:3]), sorted({str(p.stopGo.withdrawn) for p in after})


def test_a_route_notice_never_covers_the_turn_signals_prompt_and_is_said_once_it_is_gone(tmp_path):
  # turnpilotRouteNotice (integrate, STOP_SIGNS.md 3.4) and turnpilotCarSignal (tp/blinker) are both selfdrived's
  # permanent alerts at one priority, so the later one shows: "Stop sign ahead needs you" falling due 300 m out while
  # "Signal Left" is up covered the prompt for NOTICE_S. The route notice waits for a free slot (route_notice.py), and the
  # turn signal's own line (its prompt, a notice, "Signal Off") holds it back: said in full once that is gone
  from openpilot.selfdrive.turnpilot.route_notice import NOTICE_S
  sim = StopSim(tmp_path, info_kw={'certification': 'unavoidable'})
  sim.engaged, sim.v = True, 13.
  update = sim.tp.turn_signal.update

  t0 = sim.t

  def prompting(**kw):  # the driver asked to signal for the drive's first 3 s, the notice due at once (the stop under 300 m)
    out = update(**kw)
    if sim.t - t0 < 3.:
      out.prompt = LaneChange.left
    return out
  sim.tp.turn_signal.update = prompting
  seen = []
  while sim.s < S_LINE - 20.:
    p = sim.step()
    seen.append((str(p.controlAlert), p.turnSignal.prompt))
  asked = [k for k, (_, prompt) in enumerate(seen) if prompt != LaneChange.none]
  said = [k for k, (alert, _) in enumerate(seen) if alert == 'stopAheadNeedsYou']
  assert asked and said and not set(asked) & set(said)
  assert said[0] > asked[-1] and abs(len(said) * DT_MDL - NOTICE_S) <= DT_MDL + 1e-6
