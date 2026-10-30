"""turnpilotd's turn signal in closed loop (car_signals.py, DESIGN phase4/blinker 11.3): the status from CarParams and the
port, the side it publishes for controlsd, the prompt, the driver overruling the car (a skip and a reroute), the side in
doubt, and the lane planner's lead gate on the car's own lamp."""
import pytest

from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import custom, log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.car_signals import ARM_LAG_S, LEAD_LONG_M
from openpilot.selfdrive.turnpilot.desire_timing import TURN_STARTED_DEG
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim

Status = custom.TurnpilotPlan.TurnSignal.Status
LaneChange = custom.TurnpilotPlan.LaneChange
Desire = log.Desire
RIGHT_TURN = [(400., "turn", "right", junction_of("right")), (2000., "turn", "left", junction_of("left"))]


def signal_sim(maneuvers=RIGHT_TURN, port=None, flags=int(TeslaFlags.CAR_SIGNALS), cruise=13.):
  sim = Sim(maneuvers, cruise=cruise, driver_signals=None)
  sim.tp.car_signal_flags = flags
  sim.tp.car_signal_mode = 'on'
  state = {'status': 'ready'}
  if port is not None:
    state.update(port)
  sim.port_state = state
  sim.car_signal = lambda s: dict(s.port_state)
  return sim


def drive_to(sim, distance, **kw):
  plans = []
  while sim.distance() > distance and sim.t < 120.:
    plans.append(sim.step(engaged=True, **kw))
  return plans


def test_status_rows():
  for flags, port, mode, status in (
    (0, None, 'off', Status.off),
    (0, None, 'on', Status.unverified),
    (int(TeslaFlags.CAR_SIGNALS | TeslaFlags.CAR_SIGNALS_CHECK), {'status': 'checking'}, 'off', Status.checking),
    (int(TeslaFlags.CAR_SIGNALS), {'status': 'verifying'}, 'on', Status.verifying),
    (int(TeslaFlags.CAR_SIGNALS), {'status': 'waiting'}, 'on', Status.waiting),
    (int(TeslaFlags.CAR_SIGNALS), {'status': 'lit', 'left': True}, 'on', Status.ready),
    (int(TeslaFlags.CAR_SIGNALS), {'status': 'failed', 'fault': 'lampTimeout'}, 'on', Status.failed),
  ):
    sim = signal_sim(port=port, flags=flags)
    sim.tp.car_signal_mode = mode
    assert sim.step(engaged=True).turnSignal.status == status, (flags, port, mode)


def test_the_car_signals_the_turn_from_its_arming_distance():
  sim = signal_sim()
  plans = drive_to(sim, 60.)
  armed = [p for p in plans if p.turnSignal.side == LaneChange.right]
  assert armed and armed[0].maneuverDistance <= LEAD_LONG_M + 13. * ARM_LAG_S + 1.
  assert armed[0].maneuverDistance > LEAD_LONG_M
  assert all(p.turnSignal.prompt == LaneChange.none for p in plans)  # the car signals: the driver isn't asked
  assert all(p.turnSignal.status == Status.ready for p in plans)


def test_mode_off_prompts_the_turn():
  sim = signal_sim(flags=0)
  sim.tp.car_signal_mode = 'off'
  plans = drive_to(sim, 5.)
  prompted = [p for p in plans if p.turnSignal.prompt == LaneChange.right]
  assert prompted and all(p.turnSignal.status == Status.off for p in plans)
  assert all(sim.trace[i][1] < 8.95 for i, p in enumerate(plans) if p.turnSignal.prompt != LaneChange.none)


def test_the_driver_signalling_the_other_way_skips_the_turn():
  sim = signal_sim()
  drive_to(sim, 80.)
  sim.port_state = {'status': 'ready', 'block': 'otherSide', 'leftLamp': True}
  plans = drive_to(sim, 60.)
  assert any(p.tooFastToTurn for p in plans)
  skipped = [p for p in plans if str(p.turnSignal.skip) == 'driverSignal']
  assert skipped and sim.tp.skip.reason == 'driverSignal'
  assert all(p.desire == 0 for p in plans[-10:])


def test_hazards_skip_the_turn_and_say_why():
  sim = signal_sim()
  drive_to(sim, 80.)
  sim.port_state = {'status': 'ready', 'block': 'hazard', 'hazard': True, 'leftLamp': True, 'rightLamp': True}
  plans = drive_to(sim, 60.)
  assert any(str(p.turnSignal.skip) == 'hazards' for p in plans)


def test_a_lamp_lit_on_the_other_side_with_wrong_side_skips_the_turn():
  """The left lamp lights in the right request's first second: the port faults wrongSide and releases with NONE, the lamp
  still lit (ours, the body obeying NONE late or never, or the driver's press the other way). TurnPilot never turns right
  with it lit: the turn is skipped out loud (driverSignal) and rerouted, never left to the "Signal Right" prompt."""
  sim = signal_sim()
  state = {'wrong': False}

  def port(s):
    if state['wrong']:
      return {'status': 'failed', 'fault': 'wrongSide', 'leftLamp': True}
    side = s.tp.signal_out.side
    return {'status': 'requesting' if side == LaneChange.right else 'ready', 'right': side == LaneChange.right}
  sim.car_signal = port
  plans = drive_to(sim, 110.)
  assert plans[-1].turnSignal.side == LaneChange.right
  state['wrong'] = True
  plans = drive_to(sim, -20.)
  assert plans[0].tooFastToTurn and str(plans[0].turnSignal.skip) == 'driverSignal'
  assert not any(p.desire == Desire.turnRight for p in plans), "TurnPilot turned right with the left lamp lit"
  assert not any(p.turnSignal.prompt != LaneChange.none for p in plans)


def test_a_turn_whose_bearings_disagree_is_skipped():
  # navd says right, the junction's bearings turn left by 90: the side the car would steer and signal is in doubt
  sim = signal_sim(maneuvers=[(400., "turn", "right", junction_of("left")), (2000., "turn", "left", junction_of("left"))],
                   flags=0)
  plans = drive_to(sim, 300.)
  assert any(p.tooFastToTurn and str(p.turnSignal.skip) == 'sideUncertain' for p in plans)
  assert sim.tp.skip.reason == 'sideUncertain'


def test_lane_change_waits_for_the_lamp_lead():
  # a lane change the car signals: the request waits for the port's lamp to lead it by 300 ft
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import BROKEN_WHITE_LEFT, THREE_LANES_LEFT_TURN
  sim = Sim([(700., "turn", "left"), (3000., "turn", "right")], cruise=15., nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(2, 0),
            paint=BROKEN_WHITE_LEFT, driver_signals=None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  lit = {'on': False}

  def port(s):
    side = s.tp.signal_out.side
    if side != LaneChange.none:
      lit['on'] = True
    return {'status': 'lit' if lit['on'] and side != LaneChange.none else 'ready', 'right': side == LaneChange.right,
            'left': side == LaneChange.left}
  sim.car_signal = port
  plans = [sim.step(engaged=True) for _ in range(int(35. / DT_MDL))]
  offers = [i for i, p in enumerate(plans) if p.laneChangeOffer != LaneChange.none]
  requests = [i for i, p in enumerate(plans) if p.laneChange != LaneChange.none]
  assert offers and requests
  assert plans[requests[0]].turnSignal.litM >= LEAD_LONG_M
  assert all(p.signalPrompt == LaneChange.none for p in plans)


# ---------------------------------------------------------------- a turn ended by its maneuver is never handed over (7.7)

def releasing_sim(maneuvers, cruise, driver_left=None):
  """The car signalling a right turn through a port that answers it: lit while asked, ready once not. driver_left: a dict
  whose 'on' puts the driver's left lamp up, our right lamp released with NONE for a few frames, then the port idle and
  blocked by the driver's lamp."""
  sim = Sim(maneuvers, cruise=cruise, driver_signals=None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = driver_left if driver_left is not None else {'on': False}
  state['release'] = 0

  def port(s):
    if state['on']:
      state['release'] += 1
      if state['release'] < 4:
        return {'status': 'releasing', 'right': True, 'rightLamp': state['release'] < 2, 'leftLamp': True, 'block': 'otherSide'}
      return {'status': 'ready', 'leftLamp': True, 'block': 'otherSide'}
    if s.tp.signal_out.side == LaneChange.right:
      return {'status': 'lit', 'right': True, 'rightLamp': True}
    return {'status': 'ready'}
  sim.car_signal = port
  return sim


def handed_over(sim, plans):
  """(v, plan) of each plan asking the driver to signal (turnSignal.handedOver)."""
  start = len(sim.trace) - len(plans)
  return [(sim.trace[start + i][1], p) for i, p in enumerate(plans) if p.turnSignal.handedOver != LaneChange.none]


def test_a_vetoed_turn_never_asks_for_the_side_the_driver_signalled_against():
  driver = {'on': False}
  sim = releasing_sim(RIGHT_TURN, 13., driver)
  plans = drive_to(sim, 80.)
  assert plans[-1].turnSignal.side == LaneChange.right
  driver['on'] = True
  plans = drive_to(sim, -40., blinker=True)
  assert any(str(p.turnSignal.skip) == 'driverSignal' for p in plans)
  told = handed_over(sim, plans)
  assert not told, f"skipped on the driver's left lamp, then 'Signal {told[0][1].turnSignal.handedOver} yourself' at {told[0][0]:.1f} m/s"


def too_fast_plans():
  # a right turn 90 m ahead at 20 m/s: armed at once (lead + lag is 119 m), signalled, then skipped as too fast
  sim = releasing_sim([(90., "turn", "right", junction_of("right")), (2000., "turn", "left")], 20.)
  sim.v = 20.
  plans = drive_to(sim, -40.)
  assert any(p.turnSignal.side == LaneChange.right for p in plans) and any(p.tooFastToTurn for p in plans)
  return sim, plans


def test_a_too_fast_skip_never_asks_the_driver_to_signal():
  sim, plans = too_fast_plans()
  told = handed_over(sim, plans)
  assert not told, f"skipped as too fast, then 'Signal {told[0][1].turnSignal.handedOver} yourself' at {told[0][0]:.1f} m/s"


def test_a_turn_the_car_took_ended_by_the_pass_is_never_handed_over():
  # the car round 66 deg of a 90 deg right (its clear is 80) when the follower counts the junction passed, 15 m past its
  # furthest reading: DesireTiming done and the maneuver None on the same frame
  sim = releasing_sim([(300., "turn", "right", junction_of("right")), (2000., "turn", "left")], 13.)
  drive_to(sim, 5.)
  plans, turned = [], 0.
  while sim.distance() > -40. and sim.t < 120.:
    plans.append(sim.step(engaged=True, yaw_rate=-0.25))
    turned = max(turned, sim.tp.desires.turned_deg)
  assert 45. < turned < 80., turned
  assert not handed_over(sim, plans)


def test_selfdrived_says_nothing_about_signalling_after_a_too_fast_skip():
  """Through selfdrived's own Events, the real alert callbacks and AlertManager (shown_alerts): what the screen shows and
  soundd plays after the too-fast skip, at about 44 mph."""
  sim, plans = too_fast_plans()
  start = len(sim.trace) - len(plans)
  shown = [(sim.trace[start + i][1], text1, text2) for i, text1, text2, _ in shown_alerts(plans)]
  assert any(text1 == "Too Fast to Turn" or text1.startswith("Turn") for _, text1, _ in shown), {s[1] for s in shown}
  bad = [s for s in shown if "yourself" in s[2]]
  assert not bad, f"after the too-fast skip selfdrived shows {bad[0][1]!r} / {bad[0][2]!r} at {bad[0][0]:.1f} m/s"


# ---------------------------------------------------------------- the driver overrules the car at a hold (6.6)

def held_at_walker(driver: str, turned_deg: float = 0.):
  """A right turn at 300 m, the car signalling it, its desire out on the approach, held short of the junction for a walker
  in the crosswalk. driver: 'left' (the driver's lamp the other way: our lamp released with NONE, the port held off) or
  'hazards', 2 s after the car stopped, for 3 s standing; then the walker leaves and the hold lets go. Returns (sim, the
  standing plans, the plans after the walker left). turned_deg: how far into the turn the car stands (yawed on the
  approach from its pulse)."""
  sim = Sim([(300., "turn", "right"), (1500., "turn", "left")], driver_signals=None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  walker = {'on': True}

  def port(s):
    if state['driver'] == 'left':
      return {'status': 'ready', 'block': 'otherSide', 'leftLamp': True}
    if state['driver'] == 'hazards':
      return {'status': 'ready', 'block': 'hazard', 'hazard': True, 'leftLamp': True, 'rightLamp': True}
    lit = s.tp.signal_out.side == LaneChange.right
    return {'status': 'lit' if lit else 'ready', 'right': lit, 'rightLamp': lit}
  sim.car_signal = port

  def objects(s):
    return ({'cls': 'pedestrian', 'x': 300. - s.s + 3., 'y': -5., 'age': 5 + int(s.t * 10.)},) if walker['on'] else ()

  while not (sim.v < 0.05 and sim.s > 250.):
    d = sim.tp.desires
    sim.step(objects(sim), engaged=True, yaw_rate=-0.3 if d.started and d.turned_deg < turned_deg else 0.)
    assert sim.t < 60.
  assert sim.tp.desires.started and sim.tp.turn_committed, "the right turn's desire went out on the approach"
  assert sim.trace[-1][2].shouldStop and sim.trace[-1][2].turnSignal.side == LaneChange.right
  for _ in range(int(2. / DT_MDL)):
    sim.step(objects(sim), engaged=True)
  state['driver'] = driver
  standing = [sim.step(objects(sim), engaged=True, blinker=driver == 'left') for _ in range(int(3. / DT_MDL))]
  walker['on'] = False
  after = [sim.step(objects(sim), engaged=True, blinker=driver == 'left') for _ in range(int(5. / DT_MDL))]
  return sim, standing, after


@pytest.mark.parametrize("driver,skip", [('left', 'driverSignal'), ('hazards', 'hazards')])
def test_the_driver_overruling_the_car_at_a_hold_skips_the_turn_out_loud(driver, skip):
  """DESIGN 6.6: a veto while the car stands at a line holding it skips the turn out loud, its desire out or not. The
  desire that went out on the approach is cleared from the model's queue, so the hold's go is never the vetoed turn."""
  sim, standing, after = held_at_walker(driver)
  said = [p for p in standing if p.tooFastToTurn and str(p.turnSignal.skip) == skip]
  assert said, f"{driver} at the line: nothing said, no skip, no handback while standing"
  assert said[0] is standing[0], "skipped on the frame the driver overruled it"
  assert all(p.tooFastToTurn and str(p.turnSignal.skip) == skip for p in standing), "said for as long as it stands"
  # the hold that had the car keeps it while the walker is there: the veto never pulls the car away, the hold's go does
  assert all(p.shouldStop for p in standing), "the car pulled away on the veto, the walker still in the crosswalk"
  assert not after[-1].shouldStop and sim.trace[-1][1] > 1.
  assert any(p.desire == CLEAR_DESIRE for p in standing), "the approach's pulse is cleared from the model's queue"
  assert not any(p.desire == Desire.turnRight for p in standing + after), "the hold let go into the vetoed turn"
  assert all(p.speedCap < 15. for p in after if p.tooFastToTurn), "the skipped turn's cap kept until it is passed"


def test_overruled_at_a_hold_standing_in_the_turn_finishes_it_out_loud():
  """Standing at the hold TURN_STARTED_DEG into the turn, the car has started it: never sent straight from there, it
  finishes the turn and says so ("Turn Signal Changed" / "Finishing the turn"), as a veto after the start does."""
  sim, standing, after = held_at_walker('left', turned_deg=25.)
  assert TURN_STARTED_DEG <= sim.tp.desires.turned_deg < 35.
  assert not any(p.tooFastToTurn or str(p.turnSignal.skip) != 'none' for p in standing + after)
  assert any(str(p.turnSignal.notice) == 'changedMidTurn' for p in standing)


def test_overruled_at_an_unprotected_turns_line_keeps_its_handback():
  """TurnPilotAutoLeft off (G1): the unprotected left stands at its hold line, handed back (take control), its desire out
  from the approach. A veto there is no skip: a skip would release the latch with the car in a left-turn position (6.6's
  exception); the car stays held and handed back."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import TURNS_AHEAD
  sim = Sim(TURNS_AHEAD, driver_signals=None)
  sim.go_taps = False
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  driver = {'right': False}

  def port(s):
    if driver['right']:
      return {'status': 'ready', 'block': 'otherSide', 'rightLamp': True}
    lit = s.tp.signal_out.side == LaneChange.left
    return {'status': 'lit' if lit else 'ready', 'left': lit, 'leftLamp': lit}
  sim.car_signal = port
  while not (sim.v < 0.05 and sim.s > 250.):
    sim.step(engaged=True)
    assert sim.t < 60.
  for _ in range(int(1. / DT_MDL)):
    sim.step(engaged=True)
  assert sim.trace[-1][2].controlPrompt == 'unprotectedTurnTakeControl' and sim.tp.desires.started
  driver['right'] = True
  plans = [sim.step(engaged=True) for _ in range(int(5. / DT_MDL))]
  assert not any(p.tooFastToTurn or str(p.turnSignal.skip) != 'none' for p in plans)
  assert all(p.shouldStop and p.controlPrompt == 'unprotectedTurnTakeControl' for p in plans)


# ---------------------------------------------------------------- ...and while a hold brakes the car to rest short of it (B7)

def overruling_port(state, side=LaneChange.right):
  """The port as TeslaCarSignal reports it at this level: state['driver'] 'left' or 'right' (the driver's lamp, the port
  held off), 'hazards', 'lost' (our lamp put out: lampLost, released), else lit on the plan's side while it asks."""
  def port(s):
    v = state['driver']
    if v in ('left', 'right'):
      return {'status': 'ready', 'block': 'otherSide', f'{v}Lamp': True}
    if v == 'hazards':
      return {'status': 'ready', 'block': 'hazard', 'hazard': True, 'leftLamp': True, 'rightLamp': True}
    if v == 'lost':
      return {'status': 'failed', 'fault': 'lampLost'}
    lit = s.tp.signal_out.side == side
    key = 'left' if side == LaneChange.left else 'right'
    return {'status': 'lit' if lit else 'ready', key: lit, f'{key}Lamp': lit}
  return port


def braking_to_walker(driver: str, veto_after_s: float = 1., walker_at: float = 38., veto_at: float | None = None):
  """A right turn at 300 m, the car signalling it, its desire out on the approach (40 m). A walker steps into its
  crosswalk walker_at m out and TurnPilot brakes the car to rest at the hold line on the speed cap's curve: shouldStop is
  false until about standstill (the plan 'holding' from about 33 m at 7 m/s). driver ('left', 'hazards', or 'lost': our
  lamp put out) overrules the car veto_after_s into that braking, the car still rolling; veto_at (m), instead, overrules it
  there, before the walker. Returns (sim, the plans from the veto to 3 s standing, the plans after the walker left)."""
  sim = Sim([(300., "turn", "right"), (1500., "turn", "left")], driver_signals=None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state)
  walker = {'on': False}

  def objects(s):
    return ({'cls': 'pedestrian', 'x': 300. - s.s + 3., 'y': -5., 'age': 5 + int(s.t * 10.)},) if walker['on'] else ()

  def step():
    return sim.step(objects(sim), engaged=True, blinker=state['driver'] == 'left')
  plans = []
  if veto_at is not None:
    while sim.distance() > veto_at:
      step()
    assert sim.tp.desires.started, "the right turn's desire went out on the approach (40 m)"
    state['driver'] = driver
    plans += [step() for _ in range(int(0.5 / DT_MDL))]
  while sim.distance() > walker_at:
    plans.append(step()) if veto_at is not None else step()
  assert sim.tp.desires.started, "the right turn's desire went out on the approach (40 m)"
  walker['on'] = True
  while str(sim.trace[-1][2].state) != 'holding':
    plans.append(step()) if veto_at is not None else step()
  if veto_at is None:
    for _ in range(int(veto_after_s / DT_MDL)):
      step()
    assert sim.v > 1.5 and not sim.trace[-1][2].shouldStop, "braking toward the hold, not yet standing"
    assert sim.trace[-1][2].turnSignal.side == LaneChange.right
    state['driver'] = driver
  while sim.v > 0.05:
    plans.append(step())
    assert sim.t < 60.
  plans += [step() for _ in range(int(3. / DT_MDL))]
  walker['on'] = False
  after = [step() for _ in range(int(8. / DT_MDL))]
  return sim, plans, after


@pytest.mark.parametrize("driver,skip", [('left', 'driverSignal'), ('hazards', 'hazards'), ('lost', 'signalLost')])
def test_the_driver_overruling_the_car_braking_for_a_hold_skips_the_turn_out_loud(driver, skip):
  """DESIGN 6.6: a veto while a TurnPilot stop brings the car to rest short of the junction (a walker's hold braking it on
  the speed cap's curve, shouldStop still false) is decided as a veto standing there is: skipped out loud from the veto's
  frame, the approach's desire cleared, the hold kept until it lets go, and its go straight on. Never "Finishing the turn"
  first, and our lamp put out (a one-frame veto on the port's ready to failed edge) is never lost before the car stands."""
  sim, plans, after = braking_to_walker(driver)
  assert plans[0].tooFastToTurn and str(plans[0].turnSignal.skip) == skip, \
    (plans[0].tooFastToTurn, str(plans[0].turnSignal.skip), str(plans[0].turnSignal.notice))
  assert all(p.tooFastToTurn and str(p.turnSignal.skip) == skip for p in plans), "said for as long as the hold has the car"
  assert not any(str(p.turnSignal.notice) == 'changedMidTurn' for p in plans + after), "'Finishing the turn' for a skipped turn"
  assert any(p.desire == CLEAR_DESIRE for p in plans), "the approach's pulse is cleared from the model's queue"
  stood = sim.trace[-len(after) - 1]
  assert stood[2].shouldStop and stood[1] < 0.05 and stood[0] < 300. - 5., "the walker's hold let go of the car on the veto"
  # the veto's own frame still carries the desire out since 40 m (its plan's desire is set before the veto is read); from
  # the next frame it is cleared, and the hold's go is straight on
  assert plans[1].desire == CLEAR_DESIRE
  assert not any(p.desire == Desire.turnRight for p in plans[1:] + after), "the hold let go into the vetoed turn"
  assert all(p.turnSignal.side == LaneChange.none for p in plans[1:] + after), "our request outlived the skipped turn"
  assert all(p.turnSignal.prompt == LaneChange.none for p in plans + after), "asked to signal a skipped turn"
  assert not after[-1].shouldStop and sim.trace[-1][1] > 1., "the hold's go never came"


def test_a_veto_moving_into_the_turn_is_never_a_skip_once_a_hold_stops_the_car():
  """Decided once per maneuver: the driver's left lamp with the right turn's desire out, the car rolling into it with no
  stop in force, is "Turn Signal Changed" / "Finishing the turn". A walker then stops the car short of the turn, the lamp
  still on: never a skip after it ("Turn Cancelled"), one veto never two contradictory decisions."""
  sim, plans, after = braking_to_walker('left', walker_at=34., veto_at=37.)
  assert str(plans[0].turnSignal.notice) == 'changedMidTurn', str(plans[0].turnSignal.notice)
  assert any(p.shouldStop for p in plans), "the walker's hold never stopped the car"
  assert not any(p.tooFastToTurn or str(p.turnSignal.skip) != 'none' for p in plans + after), \
    "'Finishing the turn', then 'Turn Cancelled' for the same veto"


def shown_alerts(plans):
  """What selfdrived shows and plays for these plans, through its own Events, the real alert callbacks and AlertManager at
  its 100 Hz over turnpilotd's 20 Hz: (plan index, alertText1, alertText2, audibleAlert) per selfdrived frame."""
  import types

  import openpilot.selfdrive.selfdrived.events as events_mod
  from openpilot.common.realtime import DT_CTRL
  from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
  from openpilot.selfdrive.selfdrived.events import ET, Events
  from openpilot.selfdrive.turnpilot import hooks
  EventName = log.OnroadEvent.EventName
  ev = hooks.CarSignalEvents()
  current = {'plan': None}
  saved = events_mod.CAR_SIGNAL_EVENTS, events_mod.shown_plan
  events_mod.CAR_SIGNAL_EVENTS, events_mod.shown_plan = ev, lambda sm, now=None: current['plan']
  shown, frame = [], 0
  try:
    am = AlertManager()
    port = types.SimpleNamespace(status='ready', fault='none', check='idle', checkWhy='', stillOn=False, rejected=0)
    for i, p in enumerate(plans):
      current['plan'] = p
      for _ in range(5):
        events = Events()
        if p.tooFastToTurn or p.turnMissed:
          events.add(EventName.turnpilotTooFastToTurn)
        for name in ev.update(p, port, int(TeslaFlags.CAR_SIGNALS | TeslaFlags.CAR_SIGNALS_RECHECK), DT_CTRL):
          events.add(getattr(EventName, name))
        am.add_many(frame, events.create_alerts([ET.PERMANENT, ET.WARNING], [None, None, None, False, 0, None]))
        am.process_alerts(frame, set())
        a = am.current_alert
        shown.append((i, a.alert_text_1, a.alert_text_2, str(a.audible_alert)))
        frame += 1
  finally:
    events_mod.CAR_SIGNAL_EVENTS, events_mod.shown_plan = saved
  return shown


def test_selfdrived_never_says_finishing_then_cancelled_for_a_veto_braking_for_a_hold():
  sim, plans, after = braking_to_walker('left')
  shown = shown_alerts(plans + after)
  cancelled = [s for s in shown if s[1].startswith("Turn Cancelled")]
  assert cancelled, {s[1] for s in shown}
  assert not any(s[2] == "Finishing the turn" for s in shown), sorted({(s[1], s[2]) for s in shown})


# ---------------------------------------------------------------- a veto at the unprotected left's line ends its request (B8)

def auto_left_at_its_line(driver):
  """TurnPilotAutoLeft auto (the demo default), the car signalling: standing at the left's hold line through an oncoming
  stream (waitGap), the left lit. driver(state) overrules the car there: state['driver'] 'right' or 'hazards' for 3 s.
  Returns (sim, state, the plans of the veto)."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import JUNCTION, V_25, drive, left_sim
  stream = [JUNCTION + 60. + k * 6. * V_25 for k in range(3)]
  t0 = [None]

  def cars(sim):
    if t0[0] is None and JUNCTION - sim.s < 60.:
      t0[0] = sim.t
    dt = sim.t - t0[0] if t0[0] is not None else 0.
    return [(s - V_25 * dt, 3.6) for s in stream]
  sim = left_sim(cars)
  sim.answers_prompt = False
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state, LaneChange.left)
  while not (sim.trace and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap'):
    drive(sim, DT_MDL)
    assert sim.t < 90.
  drive(sim, 1.)
  assert sim.v < 0.3 and sim.trace[-1][2].turnSignal.side == LaneChange.left and sim.trace[-1][2].shouldStop
  state['driver'] = driver
  vetoed = drive(sim, 3.)
  return sim, state, vetoed


@pytest.mark.parametrize("driver", ['right', 'hazards'])
def test_a_veto_at_the_autonomous_lefts_line_never_signals_the_left_again(driver):
  """The left hands back at its line on the driver's veto (6.6's exception), the car held. The request for the left ends
  with it: when the driver puts their lamp out (still deciding, held, handed back) nothing claims the left again, so the
  car never signals the side the driver just overruled."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import drive
  sim, state, vetoed = auto_left_at_its_line(driver)
  assert any(str(p.controlPrompt) == 'leftUndecidedTakeControl' for p in vetoed), {str(p.controlPrompt) for p in vetoed}
  assert all(p.shouldStop and not p.tooFastToTurn for p in vetoed), "the latch let go of the car at the line"
  assert all(p.turnSignal.side == LaneChange.none for p in vetoed), "the request for the left outlived the veto"
  state['driver'] = 'none'  # the driver puts their lamp out, still held at the line, handed back
  later = drive(sim, 5.)
  relit = [i for i, p in enumerate(later) if p.turnSignal.side == LaneChange.left]
  assert not relit, f"the car signals left again {relit[0] * DT_MDL:.2f} s after the driver's {driver} went out"
  assert not any(p.turnSignal.prompt != LaneChange.none for p in vetoed + later)
  assert not any(p.desire == Desire.turnLeft for p in vetoed + later), "the left went after the driver overruled it"
  assert all(p.shouldStop for p in later)


def test_taking_over_after_a_veto_at_the_lefts_line_never_asks_for_the_overruled_side():
  sim, state, _ = auto_left_at_its_line('right')
  state['driver'] = 'none'
  [sim.step(engaged=True) for _ in range(int(2. / DT_MDL))]
  told = [p for p in (sim.step(engaged=False) for _ in range(int(4. / DT_MDL))) if p.turnSignal.handedOver != LaneChange.none]
  assert not told, f"'Signal Off / Signal {told[0].turnSignal.handedOver} yourself' after the driver overruled the left"


def test_a_veto_at_g1s_line_never_relights_the_left():
  """TurnPilotAutoLeft off (G1): the latch holds the car at the left's line, handed back (unprotectedTurnTakeControl). The
  driver's right lamp there ends the request for the left: once their lamp is out the car never signals left again."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import TURNS_AHEAD
  sim = Sim(TURNS_AHEAD, driver_signals=None)
  sim.go_taps = False
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state, LaneChange.left)
  while not (sim.v < 0.05 and sim.s > 250.):
    sim.step(engaged=True)
    assert sim.t < 60.
  for _ in range(int(1. / DT_MDL)):
    sim.step(engaged=True)
  assert sim.trace[-1][2].controlPrompt == 'unprotectedTurnTakeControl' and sim.trace[-1][2].turnSignal.side == LaneChange.left
  state['driver'] = 'right'
  vetoed = [sim.step(engaged=True) for _ in range(int(3. / DT_MDL))]
  state['driver'] = 'none'
  later = [sim.step(engaged=True) for _ in range(int(3. / DT_MDL))]
  assert all(p.turnSignal.side == LaneChange.none for p in vetoed + later), \
    f"the car signals left on {sum(p.turnSignal.side == LaneChange.left for p in later)} of {len(later)} plans after the veto"
  assert all(p.shouldStop and p.controlPrompt == 'unprotectedTurnTakeControl' for p in vetoed + later)
  assert not any(p.tooFastToTurn or p.turnSignal.prompt != LaneChange.none for p in vetoed + later)


# ---------------------------------------------------------------- a veto during the autonomous left's go, while it can still stop (B10)

def veto_after_go(driver, delay_frames):
  """TurnPilotAutoLeft auto, the car signalling, an empty road: the left goes from its line (state 'go'), and delay_frames
  later, the car still able to stop short of the oncoming lanes (_after_go's own test, with a metre to spare), the driver
  overrules the car's left: their right lamp, hazards, or our lamp put out. Returns (sim, the 6 s of plans from the veto)."""
  from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M, STOP_DECEL
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import JUNCTION, drive, left_sim
  sim = left_sim()
  sim.answers_prompt = False
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state, LaneChange.left)
  while not (sim.trace and str(sim.trace[-1][2].unprotectedLeft.state) == 'go'):
    drive(sim, DT_MDL)
    assert sim.t < 90.
  drive(sim, delay_frames * DT_MDL)
  assert str(sim.trace[-1][2].unprotectedLeft.state) == 'go'
  stop_m = sim.v ** 2 / (2. * STOP_DECEL)
  assert stop_m < JUNCTION - sim.s - ONCOMING_ENTRY_M - 1., "the go still withdrawable (_after_go): it can stop short of oncoming"
  state['driver'] = driver
  return sim, drive(sim, 6.)


@pytest.mark.parametrize("delay_frames", [0, 5, 15])
@pytest.mark.parametrize("driver", ['right', 'hazards', 'lost'])
def test_a_veto_at_the_lefts_go_before_it_cannot_stop_withdraws_it(driver, delay_frames):
  """DESIGN 6.6, 6.7: the left's go is withdrawn for anything it was certified on that breaks while the car can still stop
  short of the oncoming lanes; a person overruling the car's left is one more. Withdrawn and handed back through the left's
  undecided (driverSignal), the car held short of the oncoming lanes, the request for the left ended on the veto's frame (as
  at its line, B8): never "Finishing the turn" across the oncoming lanes with the driver's right lamp or hazards flashing, or
  with no lamp at all."""
  from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import JUNCTION
  sim, plans = veto_after_go(driver, delay_frames)
  states = [str(p.unprotectedLeft.state) for p in plans]
  assert not any(str(p.turnSignal.notice) == 'changedMidTurn' for p in plans), \
    f"'Finishing the turn' for the driver's {driver} at the left's go, the car able to stop short of the oncoming lanes"
  assert 'committed' not in states, f"the left crossed the oncoming lanes after the driver's {driver}"
  assert max(s for s, _, _ in sim.trace[-len(plans):]) < JUNCTION - ONCOMING_ENTRY_M, "into the oncoming lanes"
  assert states[1] == 'undecided' and str(plans[1].unprotectedLeft.why) == 'driverSignal', (states[:3], str(plans[1].unprotectedLeft.why))
  assert any(str(p.controlPrompt) == 'leftUndecidedTakeControl' for p in plans), sorted({str(p.controlPrompt) for p in plans})
  assert all(p.turnSignal.side == LaneChange.none for p in plans), "the request for the left outlived the veto"
  assert not any(p.tooFastToTurn or p.turnSignal.prompt != LaneChange.none for p in plans)
  assert sim.v < 0.05 and plans[-1].shouldStop, "never held where the go was withdrawn"
  assert not any(p.shouldStop for p, (_, v, _) in zip(plans, sim.trace[-len(plans):], strict=True) if v > 0.5), \
    "shouldStop on a car still rolling: the held stop is the comfortable curve"


def test_selfdrived_never_says_finishing_for_a_veto_at_the_lefts_go():
  sim, plans = veto_after_go('hazards', 0)
  shown = shown_alerts(plans)
  assert not any(s[2] == "Finishing the turn" for s in shown), sorted({(s[1], s[2]) for s in shown})


def test_a_veto_at_the_lefts_go_past_its_point_of_no_return_finishes_it_out_loud():
  """Past the point where the car can stop short of the oncoming lanes the left goes on (committed_past_line commits it):
  a veto there is "Turn Signal Changed" / "Finishing the turn", never a withdrawal in the oncoming lanes."""
  from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M, STOP_DECEL
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import JUNCTION, drive, left_sim
  sim = left_sim()
  sim.answers_prompt = False
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state, LaneChange.left)
  while not (sim.trace and str(sim.trace[-1][2].unprotectedLeft.state) == 'go' and sim.v > 0.3 and
             sim.v ** 2 / (2. * STOP_DECEL) >= JUNCTION - sim.s - ONCOMING_ENTRY_M):
    drive(sim, DT_MDL)
    assert sim.t < 90.
  state['driver'] = 'right'
  plans = drive(sim, 6.)
  assert any(str(p.turnSignal.notice) == 'changedMidTurn' for p in plans)
  assert not any(str(p.unprotectedLeft.state) == 'undecided' for p in plans)
  assert sim.tp.desires.done or any(p.desire == Desire.turnLeft for p in plans) or sim.tp.desires.started


# ---------------------------------------------------------------- a veto at rest short of the turn, whoever stopped the car (B9)

def lead_stand(driver, stand_at=25.):
  """A right turn at 300 m, the car signalling it, its desire out since 40 m. A lead car stops in the queue: openpilot's
  longitudinal (the Sim's cruise 0, the model's lead 6 m ahead) brings the car to rest about 19 m short of the junction, no
  TurnPilot stop in force. 2 s standing, driver ('left', 'hazards', or 'lost': our lamp put out) overrules the car for 3 s;
  then the lead moves on (our lamp put out stays failed: the port's lampLost lasts the drive; the driver's lamp or hazards
  stay on). Returns (sim, the plans of the veto, the plans after)."""
  sim = Sim([(300., "turn", "right"), (1500., "turn", "left")], driver_signals=None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state)

  def step():
    return sim.step(engaged=True, blinker=state['driver'] == 'left')
  while sim.distance() > stand_at + 8.:
    step()
  sim.cruise, sim.model_kw = 0., {'lead_prob': 0.9, 'lead_x': 6.}
  while sim.v > 0.01:
    step()
    assert sim.t < 60.
  stood = [step() for _ in range(int(2. / DT_MDL))]
  assert sim.tp.desires.started and sim.tp.desires.turned_deg < 1., "the right turn's desire is out, the car not turned"
  assert not stood[-1].shouldStop and stood[-1].turnSignal.side == LaneChange.right and sim.distance() > 10.
  assert not stood[-1].tooFastToTurn
  state['driver'] = driver
  vetoed = [step() for _ in range(int(3. / DT_MDL))]
  sim.cruise, sim.model_kw = 13., {}
  after = [step() for _ in range(int(8. / DT_MDL))]
  return sim, vetoed, after


@pytest.mark.parametrize("driver,skip", [('left', 'driverSignal'), ('hazards', 'hazards'), ('lost', 'signalLost')])
def test_a_veto_at_rest_behind_a_lead_short_of_the_turn_skips_it_out_loud(driver, skip):
  """DESIGN 6.6's standing clause: at rest short of the junction under TURN_STARTED_DEG into the turn, the car hasn't
  started it, whatever stopped it (here openpilot's longitudinal behind a lead, no TurnPilot stop in force). A veto there
  skips the turn out loud from its own frame, the approach's desire cleared, never "Finishing the turn"."""
  sim, vetoed, after = lead_stand(driver)
  notices = sorted({str(p.turnSignal.notice) for p in vetoed + after})
  assert not any(str(p.turnSignal.notice) == 'changedMidTurn' for p in vetoed + after), \
    f"standing at rest 19 m short, turned 0 deg: 'Finishing the turn' for the driver's {driver} ({notices})"
  assert vetoed[0].tooFastToTurn and str(vetoed[0].turnSignal.skip) == skip, \
    (vetoed[0].tooFastToTurn, str(vetoed[0].turnSignal.skip))
  assert any(p.desire == CLEAR_DESIRE for p in vetoed[:3]), "the approach's pulse is cleared from the model's queue"


@pytest.mark.parametrize("driver", ['left', 'hazards', 'lost'])
def test_the_car_never_takes_the_vetoed_turn_when_the_lead_moves_on(driver):
  sim, vetoed, after = lead_stand(driver)
  pulses = [i for i, p in enumerate(after) if p.desire == Desire.turnRight]
  why = f"into the turn the driver's {driver} overruled"
  assert not pulses, f"a fresh turnRight pulse {pulses[0] * DT_MDL:.2f} s after the lead moved on, {why}"
  assert not any(str(p.state) == 'turning' for p in after)
  assert all(p.turnSignal.side == LaneChange.none for p in vetoed[1:] + after), "the request outlived the veto"
  assert all(p.turnSignal.prompt == LaneChange.none for p in vetoed + after), "asked to signal the overruled turn"


def test_selfdrived_never_says_finishing_the_turn_standing_behind_a_lead():
  sim, vetoed, after = lead_stand('left')
  shown = shown_alerts(vetoed + after)
  assert not any(s[2] == "Finishing the turn" for s in shown), sorted({(s[1], s[2]) for s in shown})
  assert any(s[1].startswith("Turn Cancelled") for s in shown), sorted({(s[1], s[2]) for s in shown})


def test_a_veto_in_a_crawling_queue_still_finishes_the_turn():
  """Only the car at rest is in scope: rolling at 1 to 2 m/s in a queue with the desire out, the turn has started by the
  design's call ("Turn Signal Changed" / "Finishing the turn"), and a stand behind the lead after that never skips it."""
  sim = Sim([(300., "turn", "right"), (1500., "turn", "left")], driver_signals=None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state)
  while sim.distance() > 70.:
    sim.step(engaged=True)
  sim.cruise = 1.5
  while not sim.tp.desires.started or sim.v > 1.6:
    sim.step(engaged=True)
    assert sim.t < 90.
  assert sim.v > 1. and sim.distance() > 20.
  state['driver'] = 'left'
  rolling = [sim.step(engaged=True, blinker=True) for _ in range(int(1. / DT_MDL))]
  sim.cruise, sim.model_kw = 0., {'lead_prob': 0.9, 'lead_x': 6.}
  stood = []
  while sim.v > 0.01 or len(stood) < int(3. / DT_MDL):
    stood.append(sim.step(engaged=True, blinker=True))
    assert len(stood) < int(20. / DT_MDL)
  assert sim.distance() > 5. and sim.tp.desires.turned_deg < 1., "at rest short of the turn"
  assert str(rolling[0].turnSignal.notice) == 'changedMidTurn'
  assert not any(p.tooFastToTurn for p in rolling + stood), "'Finishing the turn', then 'Turn Cancelled' for one veto"


def test_a_veto_just_after_a_holds_go_finishes_the_turn():
  """TurnPilot's go from a standstill is out (a hold let go of the car with the turn's pulse): a veto in the frames before
  the car reaches STANDSTILL_SPEED is after the go, so the turn finishes and says so, as one a moment later does; never a
  skip that sends the car straight across the junction on the go it made for the turn."""
  sim = Sim([(300., "turn", "right"), (1500., "turn", "left")], driver_signals=None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state)
  walker = {'on': True}

  def objects(s):
    return ({'cls': 'pedestrian', 'x': 300. - s.s + 3., 'y': -5., 'age': 5 + int(s.t * 10.)},) if walker['on'] else ()
  while not (sim.v < 0.05 and sim.s > 250.):
    sim.step(objects(sim), engaged=True)
    assert sim.t < 60.
  for _ in range(int(2. / DT_MDL)):
    sim.step(objects(sim), engaged=True)
  walker['on'] = False
  while sim.trace[-1][2].desire != Desire.turnRight:
    sim.step(objects(sim), engaged=True)
    assert sim.t < 80.
  sim.step(objects(sim), engaged=True)
  assert sim.v < 0.3 and sim.tp.go_id == sim.tp.follower.maneuver_id, (sim.v, sim.tp.go_id)
  state['driver'] = 'left'
  plans = [sim.step(objects(sim), engaged=True, blinker=True) for _ in range(int(6. / DT_MDL))]
  assert sim.trace[-len(plans)][1] < 0.3, "the veto came with the car still standing"
  assert str(plans[0].turnSignal.notice) == 'changedMidTurn', str(plans[0].turnSignal.notice)
  assert not any(p.tooFastToTurn or p.desire == CLEAR_DESIRE for p in plans), "skipped after the hold's go"


def test_a_veto_at_rest_in_the_junction_the_scene_committed_to_finishes_the_turn():
  """Past the hold line faster than COMMIT_SPEED the scene commits to the junction (stopping there is worse than
  finishing the turn): a lead that then stops the car in the junction, under TURN_STARTED_DEG into the turn and within
  PROTECT_PAST_M of its maneuver point, leaves it started. A veto there finishes the turn and says so, never a skip that
  sends the car straight on from inside the junction."""
  from openpilot.selfdrive.turnpilot.map_controls import PROTECT_PAST_M
  sim = Sim([(300., "turn", "right"), (1500., "turn", "left")], driver_signals=None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state)
  while sim.distance() > 7.8:
    sim.step(engaged=True)
  sim.cruise, sim.model_kw = 0., {'lead_prob': 0.9, 'lead_x': 2.}
  while sim.v > 0.01:
    sim.step(engaged=True)
    assert sim.t < 90.
  [sim.step(engaged=True) for _ in range(int(1. / DT_MDL))]
  assert sim.tp.scene.committed_id == sim.tp.follower.maneuver_id
  assert -PROTECT_PAST_M < sim.distance() < 7.8 and sim.tp.desires.turned_deg < TURN_STARTED_DEG and sim.tp.desires.started
  state['driver'] = 'left'
  plans = [sim.step(engaged=True, blinker=True) for _ in range(int(2. / DT_MDL))]
  assert str(plans[0].turnSignal.notice) == 'changedMidTurn', str(plans[0].turnSignal.notice)
  assert not any(p.tooFastToTurn or p.desire == CLEAR_DESIRE for p in plans), "skipped inside the junction"

