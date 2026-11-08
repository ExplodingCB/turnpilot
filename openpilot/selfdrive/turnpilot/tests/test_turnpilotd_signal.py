"""turnpilotd's turn signal in closed loop (car_signals.py, DESIGN phase4/blinker 11.3): the status from CarParams and the
port, the side it publishes for controlsd, the prompt, the driver overruling the car (a skip and a reroute), the side in
doubt, and the lane planner's lead gate on the car's own lamp."""
import math
from types import SimpleNamespace

import pytest

from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import custom, log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.car_signals import ARM_LAG_S, LEAD_LONG_M
from openpilot.selfdrive.turnpilot.desire_timing import TURN_STARTED_DEG
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE
from openpilot.selfdrive.turnpilot.route_follower import KEEP, PASSED_DIST_M
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of, nav_instruction
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import T_LEFT, TURN_R_M, Sim
from openpilot.selfdrive.turnpilot.turn_speed import V_CORNER

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
  # at any speed (2026-11-07: no speed floor), from the turn's arming on
  assert all(p.turnSignal.side == LaneChange.right for p in plans if p.turnSignal.prompt != LaneChange.none)


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
    plans.append(sim.step(engaged=True, yaw_rate=-sim.v / (2. * TURN_R_M)))  # per metre (TURN_R_M): 0.25 rad/s at 4.3 m/s
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

# 2026-11-06 (drive 1e after dark, rights cutting the inside curb): a curb-side turn (a right where traffic keeps right) gets
# ONE desire pulse, about 8 m before its junction (desire_timing.CURB_TURN_PULSE_M by the furthest reading, or
# CURB_LATEST_PULSE_M by the latest), nothing before it; a far-side turn keeps its lead (40 m at the turn cap's 6.9 m/s). The
# vetoes below were all a right's with its desire out from 40 m. A right now has none out at its 8 m hold line (the car
# stands 8.26 m out), braking to it, or at rest in a queue short of it: a veto there skips a turn the model was never told
# to take (turnpilotd's `not committed` branch), and the rail is that its pulse, which the hold's go or the 8 m point rolling
# on would send, never goes. A desire out short of the junction is now a far-side turn's, so each scene also runs on a left
# from a T's stem (T_LEFT: no unprotected-turn stop), every assertion of the desire-out premise kept there. The T-left is
# held at its hold line by an oncoming stream, not a walker: the crosswalk hold is a curb-side turn's (scene_policy).
TLEFT_TURN = [(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))]
ONCOMING_FROM_M = 60.  # m before the T-left's junction the oncoming stream is seen (auto_left_at_its_line's)
TURN_IN_FROM_M = 18.  # m before the junction the car turns in, standing at the hold turned: for the right the model starting
#                      it by itself, inside desire_timing.APPROACH_M (20)
OWN_RIGHT_CURV = 0.05  # 1/m toward the right (desiredCurvature is positive right), over STEER_IN_TURN_CURV (0.04): a turn's


def turn_of(turn: str):
  """The turn at 300 m and what holds the car at its hold line. 'right': the curb-side right (no junction data: the nominal
  90 deg), held by a walker in its crosswalk; its one pulse is about 8 m out, so none is out at the line or short of it.
  'tleft': a far-side left from a T's stem (T_LEFT), its desire out from its lead (40 m at 6.9 m/s), held by an oncoming
  stream inside ONCOMING_FROM_M: a car always 30 m past the junction at 9 m/s, 3.3 s out (scene_policy.ONCOMING_TTA_S 5).
  other: the driver's lamp the other way (overruling_port's state['driver']); yaw: the car's yaw rate turning it. holder(sim,
  a): the objects in the car's frame, the car turned a rad toward the turn (the stream is up the road: scene_policy.RoadFrame
  turns it back by the car's heading)."""
  if turn == 'right':
    def walker(s, a=0.):
      return ({'cls': 'pedestrian', 'x': 300. - s.s + 3., 'y': -5., 'age': 5 + int(s.t * 10.)},)
    return SimpleNamespace(maneuvers=[(300., "turn", "right"), (1500., "turn", "left")], side=LaneChange.right,
                           desire=Desire.turnRight, other='left', yaw=-0.3, holder=walker)

  def oncoming(s, a=0.):
    if 300. - s.s > ONCOMING_FROM_M:
      return ()
    x, y, c, n = 300. - s.s + 30., 5., math.cos(a), math.sin(a)
    return ({'cls': 'car', 'x': x * c + y * n, 'y': -x * n + y * c, 'vx': -9. * c, 'vy': 9. * n, 'age': 5 + int(s.t * 10.)},)
  return SimpleNamespace(maneuvers=TLEFT_TURN, side=LaneChange.left, desire=Desire.turnLeft, other='right', yaw=0.3,
                         holder=oncoming)


def signal_veto_sim(t):
  """The car signalling turn t (turn_of) through overruling_port: (sim, the port's state)."""
  sim = Sim(t.maneuvers, driver_signals=None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state, t.side)
  return sim, state


# The corner cap after a signal veto's skip (the veto branch: skip_cap_id): kept to the skipped turn's pass when its desire went
# out, which the model may still be steering for, and for a curb-side turn in hand (turnpilotd.in_hand_now, 2026-11-06 ruling:
# HEAD parity), whose one pulse 8 m out isn't out yet: before 2026-11-06 a right was committed from its 40 m pulse, so a veto at
# its walker hold line (8.26 m out), braking to it, or at rest behind a lead short of it kept the right's 3 m/s cap to the pass;
# with its pulse moved to 8 m the same vetoes kept none, and from its hold line the car went from 0.9 m/s to 7.5 m/s 5 s after
# the hold let go, straight on through the junction it was slowing to turn at (tests-signal probe1). Kept, both turns hold the
# car to their corner caps after the go (assert_skip_cap); a veto further out than turn_speed.TURN_CAP_LEAD_M keeps none, as at
# HEAD. The drive-through windows below are sized to pass the junction at 3 m/s.
SKIP_CAP_PIN_MS = 15.  # m/s: over any corner cap (a right's 3.0, V_CORNER 4.3), under NO_SPEED_CAP and the road's own


def assert_skip_cap(turn: str, after):
  """The skip's corner cap on the plans after the car is let go (the hold's go, the lead moving on), up to the skipped turn's
  pass (the plans saying it skipped: tooFastToTurn): kept for both, the T-left's its desire out from its lead, the right's in
  hand (the ruling above), from 1 s after the go for the right (the hold's own cap lets go on the go's frame)."""
  skipped = [p for p in (after if turn == 'tleft' else after[int(1. / DT_MDL):]) if p.tooFastToTurn]
  assert skipped, "the skip not said after the car was let go"
  if turn == 'tleft':
    assert all(p.speedCap < SKIP_CAP_PIN_MS for p in skipped), "the skipped turn's cap kept until it is passed (its desire went out)"
  else:
    assert all(p.speedCap < SKIP_CAP_PIN_MS for p in skipped), \
      f"the corner cap dropped ({max(p.speedCap for p in skipped):.1f} m/s) for a right in hand: sped up through its junction"


def held_at_walker(driver: str, turned_deg: float = 0., turn: str = 'right'):
  """The turn at 300 m (turn_of), the car signalling it, held short of the junction at its hold line (8.26 m out): the
  right for a walker in its crosswalk, its one pulse not yet due there; the T-left for an oncoming stream, its desire out
  on the approach. driver: 'other' (the driver's lamp the other way: our lamp released with NONE, the port held off) or
  'hazards', 2 s after the car stopped, for 3 s standing; then the walker (or the stream) leaves and the hold lets go.
  Returns (sim, the standing plans, the plans after it left). turned_deg: how far into the turn the car stands, yawed on
  the approach from TURN_IN_FROM_M: the T-left with its desire out; the right, with no pulse before its line, by the model
  starting it itself (its turn-scale steer-in, the car following it round: desire_timing's own turn, counted as started)."""
  t = turn_of(turn)
  sim, state = signal_veto_sim(t)
  holder = {'on': True, 'a': 0.}

  def objects(s):
    return t.holder(s, holder['a']) if holder['on'] else ()
  approach = []
  while not (sim.v < 0.05 and sim.s > 250.):
    d = sim.tp.desires
    turning_in = turned_deg > 0. and sim.distance() < TURN_IN_FROM_M
    if turning_in and turn == 'right':
      sim.model_kw = {'curvature': OWN_RIGHT_CURV}
    yaw = t.yaw if turning_in and (turn == 'right' or d.started) and max(d.turned_deg, d.approach_deg) < turned_deg else 0.
    approach.append(sim.step(objects(sim), engaged=True, yaw_rate=yaw))
    holder['a'] += abs(yaw) * DT_MDL
    assert sim.t < 60.
  if turn == 'right' and not turned_deg:
    assert not sim.tp.desires.started and not any(p.desire == t.desire for p in approach), \
      "a right's one pulse is 8 m out, not yet due at its hold line"
  else:
    assert sim.tp.desires.started and sim.tp.turn_committed, "the turn's desire went out on the approach (the model's own right)"
  assert sim.trace[-1][2].shouldStop and sim.trace[-1][2].turnSignal.side == t.side
  for _ in range(int(2. / DT_MDL)):
    sim.step(objects(sim), engaged=True)
  state['driver'] = t.other if driver == 'other' else driver
  standing = [sim.step(objects(sim), engaged=True, blinker=state['driver'] == 'left') for _ in range(int(3. / DT_MDL))]
  holder['on'] = False
  after = [sim.step(objects(sim), engaged=True, blinker=state['driver'] == 'left') for _ in range(int(5. / DT_MDL))]
  return sim, standing, after


@pytest.mark.parametrize("driver,skip", [('other', 'driverSignal'), ('hazards', 'hazards')])
@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_the_driver_overruling_the_car_at_a_hold_skips_the_turn_out_loud(turn, driver, skip):
  """DESIGN 6.6: a veto while the car stands at a line holding it skips the turn out loud, its desire out or not. The
  T-left's desire, out from its lead, is cleared from the model's queue and its cap kept to the pass; the right's one
  pulse, not yet due at its line (it would be the hold's go: desire_timing.CURB_GO_SLACK_M), never goes, so nothing is
  sent to clear, and its corner cap is kept to the pass, the right in hand (assert_skip_cap's ruling). Either way the hold's
  go is never the vetoed turn."""
  t = turn_of(turn)
  sim, standing, after = held_at_walker(driver, turn=turn)
  said = [p for p in standing if p.tooFastToTurn and str(p.turnSignal.skip) == skip]
  assert said, f"{driver} at the line: nothing said, no skip, no handback while standing"
  assert said[0] is standing[0], "skipped on the frame the driver overruled it"
  assert all(p.tooFastToTurn and str(p.turnSignal.skip) == skip for p in standing), "said for as long as it stands"
  # the hold that had the car keeps it while the walker (the stream) is there: the veto never pulls the car away, the
  # hold's go does
  assert all(p.shouldStop for p in standing), "the car pulled away on the veto, the walker still in the crosswalk"
  assert not after[-1].shouldStop and sim.trace[-1][1] > 1.
  assert not any(p.desire == t.desire for p in standing + after), "the hold let go into the vetoed turn"
  if turn == 'tleft':
    assert any(p.desire == CLEAR_DESIRE for p in standing), "the approach's pulse is cleared from the model's queue"
  else:
    # nothing went to the model: no pulse, so nothing to clear
    assert all(p.desire == Desire.none for p in standing + after), "a desire for a right never pulsed"
  # both caps kept to the pass: the T-left's (its desire went out), the right's (in hand: never sped up through its junction)
  assert_skip_cap(turn, after)


@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_overruled_at_a_hold_standing_in_the_turn_finishes_it_out_loud(turn):
  """Standing at the hold TURN_STARTED_DEG into the turn, the car has started it: never sent straight from there, it
  finishes the turn and says so ("Turn Signal Changed" / "Finishing the turn"), as a veto after the start does. The
  T-left is round from its pulse; the right, whose pulse isn't due at its line, by the model taking it itself (an early
  wide right, a slip lane), which counts as started and never gets a pulse into it."""
  t = turn_of(turn)
  sim, standing, after = held_at_walker('other', turned_deg=25., turn=turn)
  assert TURN_STARTED_DEG <= sim.tp.desires.turned_deg < 35.
  assert not any(p.tooFastToTurn or str(p.turnSignal.skip) != 'none' for p in standing + after)
  assert any(str(p.turnSignal.notice) == 'changedMidTurn' for p in standing)
  if turn == 'right':
    assert not any(p.desire == t.desire for _, _, p in sim.trace), "a pulse into the model's own right"


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


def braking_to_walker(driver: str, veto_after_s: float = 1., walker_at: float = 38., veto_at: float | None = None,
                      turn: str = 'right', after_s: float = 8.):
  """The turn at 300 m (turn_of), the car signalling it: the right with no desire out short of its 8 m pulse, the T-left
  with its desire out from its lead (40 m). Its holder (the walker into the right's crosswalk, the oncoming stream across
  the T-left) comes walker_at m out and TurnPilot brakes the car to rest at the hold line on the speed cap's curve:
  shouldStop is false until about standstill (the plan 'holding' from about 33 m at 7 m/s). driver ('other': the driver's
  lamp the other way, 'hazards', or 'lost': our lamp put out) overrules the car veto_after_s into that braking, the car
  still rolling; veto_at (m), instead, overrules it there, before the holder. Returns (sim, the plans from the veto to 3 s
  standing, the after_s s of plans after the holder left)."""
  t = turn_of(turn)
  sim, state = signal_veto_sim(t)
  holder = {'on': False}

  def objects(s):
    return t.holder(s) if holder['on'] else ()

  def step():
    return sim.step(objects(sim), engaged=True, blinker=state['driver'] == 'left')

  def premise():
    if turn == 'right':
      assert not sim.tp.desires.started and not any(p.desire == t.desire for _, _, p in sim.trace), \
        f"a right's one pulse is 8 m out, none at {sim.distance():.1f} m"
    else:
      assert sim.tp.desires.started, "the T-left's desire went out on the approach (40 m)"
  plans = []
  if veto_at is not None:
    while sim.distance() > veto_at:
      step()
    premise()
    state['driver'] = t.other if driver == 'other' else driver
    plans += [step() for _ in range(int(0.5 / DT_MDL))]
  while sim.distance() > walker_at:
    plans.append(step()) if veto_at is not None else step()
  premise()
  holder['on'] = True
  while str(sim.trace[-1][2].state) != 'holding':
    plans.append(step()) if veto_at is not None else step()
  if veto_at is None:
    for _ in range(int(veto_after_s / DT_MDL)):
      step()
    assert sim.v > 1.5 and not sim.trace[-1][2].shouldStop, "braking toward the hold, not yet standing"
    assert sim.trace[-1][2].turnSignal.side == t.side
    premise()
    state['driver'] = t.other if driver == 'other' else driver
  while sim.v > 0.05:
    plans.append(step())
    assert sim.t < 60.
  plans += [step() for _ in range(int(3. / DT_MDL))]
  holder['on'] = False
  after = [step() for _ in range(int(after_s / DT_MDL))]
  return sim, plans, after


@pytest.mark.parametrize("driver,skip", [('other', 'driverSignal'), ('hazards', 'hazards'), ('lost', 'signalLost')])
@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_the_driver_overruling_the_car_braking_for_a_hold_skips_the_turn_out_loud(turn, driver, skip):
  """DESIGN 6.6: a veto while a TurnPilot stop brings the car to rest short of the junction (a walker's or the oncoming
  stream's hold braking it on the speed cap's curve, shouldStop still false) is decided as a veto standing there is:
  skipped out loud from the veto's frame, the hold kept until it lets go, and its go straight on. Never "Finishing the
  turn" first, and our lamp put out (a one-frame veto on the port's ready to failed edge) is never lost before the car
  stands. The T-left's desire, out from its lead, is cleared from the model's queue; the right's one pulse (the hold's go
  at its line) never goes."""
  t = turn_of(turn)
  # 10 s after the go: past the follower's pass at the right's 3 m/s cap too (12 m past in 8 s), so the drive-through below
  # holds under either ruling on that cap (assert_skip_cap)
  sim, plans, after = braking_to_walker(driver, turn=turn, after_s=10.)
  assert plans[0].tooFastToTurn and str(plans[0].turnSignal.skip) == skip, \
    (plans[0].tooFastToTurn, str(plans[0].turnSignal.skip), str(plans[0].turnSignal.notice))
  assert all(p.tooFastToTurn and str(p.turnSignal.skip) == skip for p in plans), "said for as long as the hold has the car"
  assert not any(str(p.turnSignal.notice) == 'changedMidTurn' for p in plans + after), "'Finishing the turn' for a skipped turn"
  stood = sim.trace[-len(after) - 1]
  assert stood[2].shouldStop and stood[1] < 0.05 and stood[0] < 300. - 5., "the hold let go of the car on the veto"
  if turn == 'tleft':
    assert any(p.desire == CLEAR_DESIRE for p in plans), "the approach's pulse is cleared from the model's queue"
    # the veto's own frame still carries the desire out since 40 m (its plan's desire is set before the veto is read); from
    # the next frame it is cleared, and the hold's go is straight on
    assert plans[1].desire == CLEAR_DESIRE
  else:
    assert all(p.desire == Desire.none for p in plans + after), "a desire for a right never pulsed"
  assert not any(p.desire == t.desire for p in plans[1:] + after), "the hold let go into the vetoed turn"
  assert all(p.turnSignal.side == LaneChange.none for p in plans[1:] + after), "our request outlived the skipped turn"
  assert all(p.turnSignal.prompt == LaneChange.none for p in plans + after), "asked to signal a skipped turn"
  assert not after[-1].shouldStop and sim.trace[-1][1] > 1., "the hold's go never came"
  assert_skip_cap(turn, after)
  # past the follower's pass (PASSED_DIST_M), where a turn driven straight through is said missed (check_missed), so the
  # rail after it reads the whole pass
  assert sim.s > 300. + PASSED_DIST_M, f"driven straight on through the junction, but {sim.s - 300.:.1f} m past it at the end"
  assert not any(p.turnMissed for p in plans + after), "the vetoed turn, driven straight past, said as missed"


def test_a_veto_moving_into_the_turn_is_never_a_skip_once_a_hold_stops_the_car():
  """Decided once per maneuver: the driver's lamp the other way with the turn's desire out, the car rolling into it with no
  stop in force, is "Turn Signal Changed" / "Finishing the turn". A hold (the oncoming stream) then stops the car short of
  the turn, the lamp still on: never a skip after it ("Turn Cancelled"), one veto never two contradictory decisions. A
  far-side turn's: a right's desire is out only from 8 m, its own hold line, which the car crosses at its 3 m/s cap, over
  scene_policy.COMMIT_SPEED, so the scene commits to the turn there and a stand after it is in the junction (test_a_veto_
  at_rest_in_the_junction_the_scene_committed_to_finishes_the_turn), not short of the turn."""
  sim, plans, after = braking_to_walker('other', walker_at=34., veto_at=37., turn='tleft')
  assert str(plans[0].turnSignal.notice) == 'changedMidTurn', str(plans[0].turnSignal.notice)
  assert any(p.shouldStop for p in plans), "the oncoming stream's hold never stopped the car"
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


@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_selfdrived_never_says_finishing_then_cancelled_for_a_veto_braking_for_a_hold(turn):
  sim, plans, after = braking_to_walker('other', turn=turn)
  shown = shown_alerts(plans + after)
  cancelled = [s for s in shown if s[1].startswith("Turn Cancelled")]
  assert cancelled, {s[1] for s in shown}
  assert not any(s[2] == "Finishing the turn" for s in shown), sorted({(s[1], s[2]) for s in shown})


# ---------------------------------------------------------------- a veto at the unprotected left's line ends its request (B8)

def auto_left_at_its_line(driver, veto_s=3.):
  """TurnPilotAutoLeft auto (the demo default), the car signalling: standing at the left's hold line through an oncoming
  stream (waitGap), the left lit. driver(state) overrules the car there: state['driver'] 'right' or 'hazards' (or 'lost':
  our lamp put out) for veto_s. Returns (sim, state, the plans of the veto)."""
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
  vetoed = drive(sim, veto_s)
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

def lead_stand(driver, stand_at=25., turn='right', after_s=8.):
  """The turn at 300 m (turn_of), the car signalling it: the right with no desire out short of its 8 m pulse, the T-left
  with its desire out since its lead (40 m). A lead car stops in the queue: openpilot's longitudinal (the Sim's cruise 0,
  the model's lead 6 m ahead) brings the car to rest about 19 m short of the junction, no TurnPilot stop in force. 2 s
  standing, driver ('other': the driver's lamp the other way, 'hazards', or 'lost': our lamp put out) overrules the car for
  3 s; then the lead moves on, the right's car rolling through its 8 m pulse point (our lamp put out stays failed: the
  port's lampLost lasts the drive; the driver's lamp or hazards stay on). Returns (sim, the plans of the veto, the after_s
  s of plans after)."""
  t = turn_of(turn)
  sim, state = signal_veto_sim(t)

  def step():
    return sim.step(engaged=True, blinker=state['driver'] == 'left')
  while sim.distance() > stand_at + 8.:
    step()
  sim.cruise, sim.model_kw = 0., {'lead_prob': 0.9, 'lead_x': 6.}
  while sim.v > 0.01:
    step()
    assert sim.t < 60.
  stood = [step() for _ in range(int(2. / DT_MDL))]
  if turn == 'right':
    assert not sim.tp.desires.started and not any(p.desire == t.desire for _, _, p in sim.trace), \
      "a right's one pulse is 8 m out: none at rest 19 m short"
  else:
    assert sim.tp.desires.started and sim.tp.desires.turned_deg < 1., "the T-left's desire is out, the car not turned"
  assert not stood[-1].shouldStop and stood[-1].turnSignal.side == t.side and sim.distance() > 10.
  assert not stood[-1].tooFastToTurn
  state['driver'] = t.other if driver == 'other' else driver
  vetoed = [step() for _ in range(int(3. / DT_MDL))]
  sim.cruise, sim.model_kw = 13., {}
  after = [step() for _ in range(int(after_s / DT_MDL))]
  return sim, vetoed, after


@pytest.mark.parametrize("driver,skip", [('other', 'driverSignal'), ('hazards', 'hazards'), ('lost', 'signalLost')])
@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_a_veto_at_rest_behind_a_lead_short_of_the_turn_skips_it_out_loud(turn, driver, skip):
  """DESIGN 6.6's standing clause: at rest short of the junction under TURN_STARTED_DEG into the turn, the car hasn't
  started it, whatever stopped it (here openpilot's longitudinal behind a lead, no TurnPilot stop in force). A veto there
  skips the turn out loud from its own frame, never "Finishing the turn": the T-left's desire, out from its lead, cleared
  from the model's queue; the right's, not yet pulsed, never sent."""
  sim, vetoed, after = lead_stand(driver, turn=turn)
  notices = sorted({str(p.turnSignal.notice) for p in vetoed + after})
  assert not any(str(p.turnSignal.notice) == 'changedMidTurn' for p in vetoed + after), \
    f"standing at rest 19 m short, turned 0 deg: 'Finishing the turn' for the driver's {driver} ({notices})"
  assert vetoed[0].tooFastToTurn and str(vetoed[0].turnSignal.skip) == skip, \
    (vetoed[0].tooFastToTurn, str(vetoed[0].turnSignal.skip))
  if turn == 'tleft':
    assert any(p.desire == CLEAR_DESIRE for p in vetoed[:3]), "the approach's pulse is cleared from the model's queue"
  else:
    assert all(p.desire == Desire.none for p in vetoed + after), "a desire for a right never pulsed"


@pytest.mark.parametrize("driver", ['other', 'hazards', 'lost'])
@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_the_car_never_takes_the_vetoed_turn_when_the_lead_moves_on(turn, driver):
  """The lead moves on and the car drives through the junction it was overruled at: the T-left's desire, cleared, never
  pulsed again; the right's one pulse, due as the car rolls inside 8 m, never sent for a skipped turn."""
  t = turn_of(turn)
  # 14 s after the lead moves on: past the follower's pass at the right's 3 m/s cap too (only at the junction in 8 s), so
  # the drive-through holds under either ruling on that cap (assert_skip_cap) and the rails below read the whole pass
  sim, vetoed, after = lead_stand(driver, turn=turn, after_s=14.)
  assert_skip_cap(turn, after)
  assert sim.trace[-len(after)][0] < 300. - 10. and sim.s > 300. + PASSED_DIST_M, \
    f"driven on through the junction, but {sim.s - 300.:.1f} m past it at the end"
  pulses = [i for i, p in enumerate(after) if p.desire == t.desire]
  why = f"into the turn the driver's {driver} overruled"
  assert not pulses, f"a fresh {t.desire} pulse {pulses[0] * DT_MDL:.2f} s after the lead moved on, {why}"
  assert not any(str(p.state) == 'turning' for p in after)
  assert all(p.turnSignal.side == LaneChange.none for p in vetoed[1:] + after), "the request outlived the veto"
  assert all(p.turnSignal.prompt == LaneChange.none for p in vetoed + after), "asked to signal the overruled turn"
  assert not any(p.turnMissed for p in vetoed + after), "the vetoed turn, driven straight past, said as missed"


@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_selfdrived_never_says_finishing_the_turn_standing_behind_a_lead(turn):
  sim, vetoed, after = lead_stand('other', turn=turn)
  shown = shown_alerts(vetoed + after)
  assert not any(s[2] == "Finishing the turn" for s in shown), sorted({(s[1], s[2]) for s in shown})
  assert any(s[1].startswith("Turn Cancelled") for s in shown), sorted({(s[1], s[2]) for s in shown})


@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_a_veto_in_a_crawling_queue_still_finishes_the_turn(turn):
  """Only the car at rest is in scope: rolling at 1 to 2 m/s in a queue with the desire out, the turn has started by the
  design's call ("Turn Signal Changed" / "Finishing the turn"), and a stand behind the lead after that never skips it. The
  T-left crawls from about 29 m with its desire out from its lead (40 m); the right's is out from its one pulse 8 m out, so
  its crawl is inside 8 m, over its hold line under scene_policy.COMMIT_SPEED (the scene not committed: its stand, about
  3 m short, is at rest short of the turn as B9's is, but for the veto already decided)."""
  t = turn_of(turn)
  sim, state = signal_veto_sim(t)
  if turn == 'right':
    # from 30 m: since 2026-11-07 a right reaches its corner speed by 15 m (turn_speed CORNER_CURB_M), at about 4.5 m/s at
    # 20 m, so the queue's crawl starts earlier to be under 1.6 m/s by the 8 m pulse, as from 20 m before
    while sim.distance() > 30.:
      sim.step(engaged=True)
    sim.cruise = 1.5
  else:
    while not sim.tp.desires.started:
      sim.step(engaged=True)
      assert sim.t < 60.
    sim.cruise = 0.  # the queue ahead slows the car to its crawl from the turn cap's 6.9 m/s, its desire out at 40 m
    while sim.v > 1.6:
      sim.step(engaged=True)
    sim.cruise = 1.5
  while not sim.tp.desires.started or sim.v > 1.6:
    sim.step(engaged=True)
    assert sim.t < 90.
  assert sim.v > 1. and sim.distance() > (20. if turn == 'tleft' else 5.), (sim.v, sim.distance())
  state['driver'] = t.other
  rolling = [sim.step(engaged=True, blinker=state['driver'] == 'left') for _ in range(int(1. / DT_MDL))]
  sim.cruise, sim.model_kw = 0., {'lead_prob': 0.9, 'lead_x': 6.}
  stood = []
  while sim.v > 0.01 or len(stood) < int(3. / DT_MDL):
    stood.append(sim.step(engaged=True, blinker=state['driver'] == 'left'))
    assert len(stood) < int(20. / DT_MDL)
  assert sim.distance() > (5. if turn == 'tleft' else 1.) and sim.tp.desires.turned_deg < 1., "at rest short of the turn"
  assert sim.tp.follower.maneuver_id not in (sim.tp.scene.committed_id, sim.tp.scene.released_id), "the scene committed"
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


# a walker in the path 22 m short of the junction: the car stops about 27 m short of it, inside turn_skip.STOP_BEFORE_TURN_M
# (30), so a right's go there is the turn's go, remembered for its first pulse 8 m out (turnpilotd.watch_curb_go)
WALKER_MID_BLOCK = 300. - 22.


def mid_block_go_then_lead(driver, turn='right'):
  """The turn at 300 m (turn_of), the car signalling it. A walker in the path holds the car about 27 m short of the
  junction (a mid-block crosswalk); it leaves, the hold's go lets the car drive on at 5 m/s. A lead then stops the car in
  the queue about 18 m short, turned 0 deg, no TurnPilot stop in force: B9's scene, with an earlier go of TurnPilot's for
  the turn behind it. The T-left's desire is out from its lead (40 m) before the walker steps in, so the hold's go is the
  turn's (go_id), and the stand behind the lead spends it. The right's isn't (its one pulse is 8 m out): its go is
  remembered for that pulse (watch_curb_go) and forgotten at the stand, go_id never set. 2 s standing, driver ('other':
  the driver's lamp the other way, 'hazards', or 'lost': our lamp put out) overrules the car for 3 s; then the lead moves
  on. Returns (sim, where the walker held the car, the plans of the veto, the plans after)."""
  t = turn_of(turn)
  sim, state = signal_veto_sim(t)
  walker = {'on': False}

  def objects(s):
    if walker['on'] == 'clear':  # seen walking clear of the path, a new hit (WF-13: never lost in front of the car)
      return ({'cls': 'pedestrian', 'x': WALKER_MID_BLOCK - s.s, 'y': 3., 'age': 6 + int(s.t * 10.)},)
    return ({'cls': 'pedestrian', 'x': WALKER_MID_BLOCK - s.s, 'y': 0., 'age': 5 + int(s.t * 10.)},) if walker['on'] else ()

  def step():
    return sim.step(objects(sim), engaged=True, blinker=state['driver'] == 'left')
  if turn == 'right':
    while sim.distance() > 70.:
      step()
    sim.cruise = 5.
    while sim.distance() > 52.:
      step()
  else:
    while not sim.tp.desires.started:  # its lead: 40 m at the turn cap's 6.9 m/s (at 5 m/s it would be 30 m, past the stand)
      step()
      assert sim.t < 60.
    sim.cruise = 5.
  walker['on'] = True
  while not (sim.v < 0.05 and sim.trace[-1][2].shouldStop):
    step()
    assert sim.t < 90.
  for _ in range(int(2. / DT_MDL)):
    step()
  stood_at = sim.distance()
  if turn == 'right':
    assert not sim.tp.desires.started and not any(p.desire == t.desire for _, _, p in sim.trace), \
      f"a right's one pulse is 8 m out: none at {stood_at:.1f} m short"
  else:
    assert sim.tp.desires.started, f"the T-left's desire is out ({stood_at:.1f} m short)"
  walker['on'] = 'clear'
  step()
  walker['on'] = False
  while sim.v < 0.5:
    step()
    assert sim.t < 120.
  if turn == 'right':
    assert sim.tp.go_id != sim.tp.follower.maneuver_id and sim.tp.curb_go is not None, \
      "the right's go remembered for its first pulse, go_id not yet armed (watch_curb_go)"
  else:
    assert sim.tp.go_id == sim.tp.follower.maneuver_id, "the walker hold's go set go_id for the turn"
  while sim.distance() > 24.:
    step()
  sim.cruise, sim.model_kw = 0., {'lead_prob': 0.9, 'lead_x': 6.}
  while sim.v > 0.01:
    step()
    assert sim.t < 150.
  stood = [step() for _ in range(int(2. / DT_MDL))]
  assert sim.tp.desires.turned_deg < 1. and sim.distance() > 10., (sim.tp.desires.turned_deg, sim.distance())
  assert sim.tp.desires.started == (turn == 'tleft'), "the T-left's desire out, the right's not yet pulsed"
  assert not stood[-1].shouldStop and stood[-1].turnSignal.side == t.side and not stood[-1].tooFastToTurn
  if turn == 'right':
    assert sim.tp.curb_go is None and sim.tp.go_id != sim.tp.follower.maneuver_id, "the stand forgets the right's go"
  state['driver'] = t.other if driver == 'other' else driver
  vetoed = [step() for _ in range(int(3. / DT_MDL))]
  sim.cruise, sim.model_kw = 13., {}
  after = [step() for _ in range(int(8. / DT_MDL))]
  return sim, stood_at, vetoed, after


@pytest.mark.parametrize("driver,skip", [('other', 'driverSignal'), ('hazards', 'hazards'), ('lost', 'signalLost')])
@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_a_veto_at_rest_behind_a_lead_after_an_earlier_holds_go_skips_the_turn(turn, driver, skip):
  """An earlier go of TurnPilot's for the turn, made well short of the junction (a mid-block walker's hold let go about
  27 m out), is spent once the car has rolled on from it and come to rest again short of the hold line: at rest behind a
  lead 18 m short, turned 0 deg, the car hasn't started the turn (B9's standing clause), so a veto skips it out loud,
  never "Finishing the turn" with the car taking the vetoed turn when the lead moves on (verification of tp/blinker, M1).
  The T-left's desire, out from its lead, is cleared from the model's queue; the right's was never sent."""
  sim, stood_at, vetoed, after = mid_block_go_then_lead(driver, turn=turn)
  notices = sorted({str(p.turnSignal.notice) for p in vetoed + after})
  assert not any(str(p.turnSignal.notice) == 'changedMidTurn' for p in vetoed + after), \
    f"at rest behind a lead, turned 0 deg, a hold's go spent {stood_at:.0f} m short: 'Finishing the turn' ({notices})"
  assert vetoed[0].tooFastToTurn and str(vetoed[0].turnSignal.skip) == skip, \
    (vetoed[0].tooFastToTurn, str(vetoed[0].turnSignal.skip))
  if turn == 'tleft':
    assert any(p.desire == CLEAR_DESIRE for p in vetoed[:3]), "the approach's pulse is cleared from the model's queue"
  else:
    assert all(p.desire == Desire.none for p in vetoed + after), "a desire for a right never pulsed"


@pytest.mark.parametrize("driver", ['other', 'hazards', 'lost'])
@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_the_car_never_takes_the_vetoed_turn_after_an_earlier_holds_go(turn, driver):
  """...and the lead moving on, the car drives through the junction straight: the right's one pulse, due as it rolls inside
  8 m (the earlier go's, had the stand not forgotten it), never sent for a skipped turn, its corner cap kept to the pass (in
  hand at rest behind the lead: assert_skip_cap). The T-left stays capped by its skip's cap (assert_skip_cap: its desire was
  out), its V_CORNER 4.3 m/s: the walker go's post-go profile (graduated, 3.0 m/s to 15 deg turned) is that turn's and is
  dropped with its skip (tests-signal's flag, 2026-11-06: it held the car to 3.0 m/s straight through the junction it no longer
  turned at, the car 0.4 m short of it after 8 s)."""
  t = turn_of(turn)
  sim, _, vetoed, after = mid_block_go_then_lead(driver, turn=turn)
  assert_skip_cap(turn, after)
  if turn == 'tleft':
    assert sim.tp.post_go is None and min(p.speedCap for p in after) >= V_CORNER - 1e-3, min(p.speedCap for p in after)
  assert sim.s > 300. - 1., "driven on to the junction, past where its pulse would go (the right's 8 m, the T-left's entry 10 m)"
  turning = [i for i, p in enumerate(after) if p.desire == t.desire or str(p.state) == 'turning']
  assert not turning, f"the car takes the turn the driver's {driver} overruled, {turning[0] * DT_MDL:.2f} s after the lead moved on"
  assert all(p.turnSignal.prompt == LaneChange.none for p in vetoed + after), "asked to signal the overruled turn"
  assert all(p.turnSignal.side == LaneChange.none for p in vetoed[1:] + after), "the request outlived the veto"
  assert not any(p.turnMissed for p in vetoed + after), "the vetoed turn, driven straight past, said as missed"


@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_selfdrived_never_says_finishing_after_an_earlier_holds_go(turn):
  _, _, vetoed, after = mid_block_go_then_lead('other', turn=turn)
  shown = shown_alerts(vetoed + after)
  assert not any(s[2] == "Finishing the turn" for s in shown), sorted({(s[1], s[2]) for s in shown})
  assert any(s[1].startswith("Turn Cancelled") for s in shown), sorted({(s[1], s[2]) for s in shown})


@pytest.mark.parametrize("scene", ['held', 'braking', 'lead', 'midBlock'])
@pytest.mark.parametrize("turn", ['right', 'tleft'])
def test_unvetoed_each_scenes_turn_is_pulsed_where_a_vetoed_one_never_is(turn, scene):
  """The control for the vetoes above (their 'never the vetoed turn' rails would hold of a turn never pulsed at all): with
  nobody overruling the car, the same scene's turn is pulsed after the point the vetoed one must not be, never skipped. The
  right's one pulse at its hold's go (CURB_GO_SLACK_M) or rolling inside 8 m, the T-left's repeats and entry pulse."""
  t = turn_of(turn)
  if scene == 'held':
    _, standing, after = held_at_walker('none', turn=turn)
    later = standing + after
  elif scene == 'braking':
    _, plans, after = braking_to_walker('none', turn=turn)
    later = plans[1:] + after
  elif scene == 'lead':
    _, _, later = lead_stand('none', turn=turn)
  else:
    _, _, _, later = mid_block_go_then_lead('none', turn=turn)
  assert any(p.desire == t.desire for p in later), f"{turn} {scene}: never pulsed with no veto"
  assert not any(p.tooFastToTurn for p in later), f"{turn} {scene}: skipped with no veto"


def test_a_veto_at_rest_in_the_junction_a_holds_go_rolled_into_finishes_the_turn():
  """A hold's go at the junction (the walker in the turn's crosswalk gone, the car let go at the hold line) that rolls the
  car past the line into the junction, slower than COMMIT_SPEED (the scene not committed), and a lead then stopping it
  there under TURN_STARTED_DEG into the turn: the go's launch is under way inside the junction, so a veto finishes the turn
  and says so, never a skip that sends the car straight on from inside the junction it pulled into for the turn. Only a
  rest again short of the hold line (GO_LINE_M out or more) spends the go."""
  from openpilot.selfdrive.turnpilot.turnpilotd import GO_LINE_M
  sim = Sim([(300., "turn", "right"), (1500., "turn", "left")], driver_signals=None)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state)
  walker = {'on': True}

  def objects(s):
    return ({'cls': 'pedestrian', 'x': 300. - s.s + 3., 'y': -5., 'age': 5 + int(s.t * 10.)},) if walker['on'] else ()

  def step():
    return sim.step(objects(sim), engaged=True, blinker=state['driver'] == 'left')
  while not (sim.v < 0.05 and sim.s > 250.):
    step()
    assert sim.t < 60.
  for _ in range(int(2. / DT_MDL)):
    step()
  assert sim.distance() >= GO_LINE_M, "held at the hold line"
  sim.cruise = 1.5
  walker['on'] = False
  while sim.distance() > GO_LINE_M - 2.:
    step()
    assert sim.t < 90.
  assert sim.tp.go_id == sim.tp.follower.maneuver_id
  sim.cruise, sim.model_kw = 0., {'lead_prob': 0.9, 'lead_x': 2.}
  while sim.v > 0.01:
    step()
    assert sim.t < 100.
  [step() for _ in range(int(1. / DT_MDL))]
  assert sim.tp.follower.maneuver_id not in (sim.tp.scene.committed_id, sim.tp.scene.released_id), "the scene committed"
  assert sim.distance() < GO_LINE_M and sim.tp.desires.turned_deg < TURN_STARTED_DEG and sim.tp.desires.started
  state['driver'] = 'left'
  plans = [step() for _ in range(int(2. / DT_MDL))]
  assert str(plans[0].turnSignal.notice) == 'changedMidTurn', str(plans[0].turnSignal.notice)
  assert not any(p.tooFastToTurn or p.desire == CLEAR_DESIRE for p in plans), "skipped inside the junction"


# ---------------------------------------------------------------- a reroute after a veto (M3)

REROUTE_J, REROUTE_NEXT = 300., 1500.


def veto_then_reroute(driver, rerouted_at=REROUTE_J, mtype="turn", modifier="right", angle=None, model_lanes=(0, 0)):
  """A right turn at 300 m (or mtype modifier, its junction's angle), the car signalling it; driver ('lost': our lamp put
  out, the port failed for the drive, or 'left': the driver's lamp the other way, off again a second later) overrules it
  about 95 m short: skipped out loud. navd then reroutes before the junction (an off-route drift, the stop-avoidance
  re-run, an ETA drift on a certified control): one invalid update, then routeCount 1 with the same maneuver at
  rerouted_at (the same junction, same bearings, by default), the next left after it. model_lanes: the model's lanes
  (a keep is pulsed only toward a lane it sees). Returns (sim, the maneuver id at the veto, the plans after the veto's
  second)."""
  jn = junction_of(modifier, angle=angle)
  sim = Sim([(REROUTE_J, mtype, modifier, jn), (REROUTE_NEXT, "turn", "left", junction_of("left"))], driver_signals=None,
            model_lanes=model_lanes)
  sim.tp.car_signal_flags, sim.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
  state = {'driver': 'none'}
  sim.car_signal = overruling_port(state)

  def step():
    return sim.step(engaged=True, blinker=state['driver'] == 'left')
  while sim.distance() > 95.:
    step()
  assert sim.trace[-1][2].turnSignal.side == LaneChange.right
  state['driver'] = driver
  vetoed = [step() for _ in range(int(1. / DT_MDL))]
  assert vetoed[0].tooFastToTurn and sim.tp.skip.skipped, "the veto skips the turn out loud"
  old_id = sim.tp.follower.maneuver_id
  if driver == 'left':
    state['driver'] = 'none'  # the driver's lamp out again: the port ready, nothing vetoing a turn taken up again
  k = [0]

  def reroute(s, instr):
    k[0] += 1
    if k[0] == 1:
      return nav_instruction(REROUTE_J - s.s, mtype, modifier, remaining=REROUTE_NEXT - s.s, junction=jn), False
    if s.s < rerouted_at + 10.:
      return nav_instruction(rerouted_at - s.s, mtype, modifier, remaining=REROUTE_NEXT - s.s, junction=jn,
                             route_count=1, ahead=[(REROUTE_NEXT - s.s, "turn", "left", junction_of("left"))]), True
    return nav_instruction(REROUTE_NEXT - s.s, "turn", "left", remaining=REROUTE_NEXT - s.s, junction=junction_of("left"),
                           route_count=1), True
  sim.nav_filter = reroute
  after = []
  while sim.s < rerouted_at + 30. and sim.t < 120.:
    after.append(step())
  return sim, old_id, after


@pytest.mark.parametrize("driver,skip", [('lost', 'signalLost'), ('left', 'driverSignal')])
def test_a_reroute_onto_the_same_junction_never_revives_a_vetoed_turn(driver, skip):
  """The veto's skip is the junction's, not the maneuver id's: navd's new route turning right at the same junction (same
  bearings, within SAME_TURN_M of where it was) under a new id is still the turn the driver overruled, skipped for the
  same reason out loud, never prompted, signalled or pulsed. With our lamp put out the port is failed for the drive, so
  nothing could veto it again (verification of tp/blinker, M3)."""
  sim, old_id, after = veto_then_reroute(driver)
  new_id = sim.tp.follower.maneuver_id
  assert new_id != old_id, "navd's reroute is a new maneuver"
  prompts = [p for p in after if p.turnSignal.prompt != LaneChange.none]
  sides = [p for p in after if p.turnSignal.side != LaneChange.none]
  pulses = [p for p in after if p.desire == Desire.turnRight]
  taken = f"{len(prompts)} frames prompting 'Signal Right', {len(sides)} with our side set"
  assert not pulses, f"maneuver id {old_id} -> {new_id} on the reroute: the turn the driver overruled ({skip}) taken ({taken})"
  assert not prompts and not sides
  rerouted = [p for p in after if p.routeValid and p.maneuverDistance > 0. and str(p.maneuverModifier) == 'right']
  assert rerouted and all(p.tooFastToTurn and str(p.turnSignal.skip) == skip for p in rerouted), \
    "the new route's turn at the vetoed junction is skipped out loud, for the veto's reason"
  assert not any(str(p.state) == 'turning' for p in after)


@pytest.mark.parametrize("mtype,modifier,angle", [("turn", "slight right", 40.), ("fork", "slight right", 25.)])
@pytest.mark.parametrize("driver,skip", [('lost', 'signalLost'), ('left', 'driverSignal')])
def test_a_reroute_onto_the_same_fork_never_revives_a_vetoed_keep(driver, skip, mtype, modifier, angle):
  """A keep is signalled and overruled as a turn is (car_signals: TURN and KEEP), and route_follower.classify makes every
  'slight' modifier a keep: a city 'turn slight right' at a Y junction, a fork, an off ramp. So the veto's skip carries
  across navd's reroute onto the same fork under a new id as a turn's does: skipped for the veto's reason out loud, never
  prompted, signalled or pulsed keepRight (verification of tp/blinker, K1: carry_veto took up only turns, and the keep came
  back for 58-72 frames of keepRight pulses and up to 181 frames with our side set)."""
  sim, old_id, after = veto_then_reroute(driver, mtype=mtype, modifier=modifier, angle=angle, model_lanes=(0, 1))
  assert sim.tp.vetoed is not None and sim.tp.vetoed.maneuver.kind == KEEP, "a slight right is a keep (route_follower.classify)"
  new_id = sim.tp.follower.maneuver_id
  assert new_id != old_id, "navd's reroute is a new maneuver"
  pulses = [p for p in after if p.desire == Desire.keepRight]
  sides = [p for p in after if p.turnSignal.side != LaneChange.none]
  prompts = [p for p in after if p.turnSignal.prompt != LaneChange.none]
  taken = f"{len(pulses)} keepRight frames, {len(sides)} frames our side set, {len(prompts)} prompting"
  assert not pulses and not sides and not prompts, \
    f"maneuver id {old_id} -> {new_id}: the {mtype} {modifier} the driver overruled ({skip}) taken up again: {taken}"
  rerouted = [p for p in after if p.routeValid and p.maneuverDistance > 0. and str(p.maneuverModifier) == modifier]
  assert rerouted and all(p.tooFastToTurn and str(p.turnSignal.skip) == skip for p in rerouted), \
    "the new route's keep at the vetoed fork is skipped out loud, for the veto's reason"


def test_a_reroute_onto_the_next_junction_the_same_way_takes_that_turn():
  """Only the vetoed junction stays skipped: a new route turning right at the next junction the same way, 100 m on (the
  same bearings on a grid, more than SAME_TURN_M from the vetoed one), is a turn of its own, signalled and taken."""
  sim, _, after = veto_then_reroute('left', rerouted_at=REROUTE_J + 100.)
  near = [p for p in after if p.routeValid and 0. < p.maneuverDistance < 60. and str(p.maneuverModifier) == 'right']
  assert near and not any(p.tooFastToTurn for p in near)
  assert any(p.desire == Desire.turnRight for p in after), "the next junction's right is taken"
  assert any(p.turnSignal.side == LaneChange.right for p in after)


# ---------------------------------------------------------------- a reroute after a veto at the left's line (K2)

def reroute_at_the_left(junction, nxt=1500.):
  """navd's reroute with the car standing at the left's line (its stand drifting the ETAs of certified controls ahead:
  check_eta_drift; a stop-avoidance input change): one invalid update, then routeCount 1 with the same left at REROUTE_J
  (same bearings, same distance), the right at nxt after it. A nav_filter."""
  k = [0]

  def nav(s, instr):
    k[0] += 1
    if k[0] == 1:
      return nav_instruction(REROUTE_J - s.s, "turn", "left", remaining=nxt - s.s, junction=junction), False
    if s.s < REROUTE_J + 10.:
      return nav_instruction(REROUTE_J - s.s, "turn", "left", remaining=nxt - s.s, junction=junction, route_count=1,
                             ahead=[(nxt - s.s, "turn", "right", junction_of("right"))]), True
    return nav_instruction(nxt - s.s, "turn", "right", remaining=nxt - s.s, junction=junction_of("right"), route_count=1), True
  return nav


def nearer_reading_at_the_left_line(driver, nearer_m):
  """auto_left_at_its_line through a longer oncoming stream, and once the car stands at the left's line (waitGap, at rest)
  one of navd's readings of the left nearer_m nearer than the rest (a GPS fix off at a standstill). The follower's distance
  is the nearest reading, so 10 m nearer puts the junction behind the car standing 9.3 m short of it, under UNCERTAIN_M
  of spread (no junctionUncertain), and the left keeps judging (test_turnpilotd_left: test_a_nearer_reading_while_held_
  at_the_line_never_silently_freezes_the_left). 1.5 s on, the driver overrules the car's left for 1 s. Returns (sim, state,
  the plans of the veto)."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import JUNCTION, LEFT_25, V_25, drive, left_sim
  from openpilot.selfdrive.turnpilot.unprotected_left import HELD_MIN_M
  stream = [JUNCTION + 60. + k * 6. * V_25 for k in range(4)]
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
  jumped = {'on': False, 'n': 0}

  def nav(s, instr):
    if jumped['on'] and jumped['n'] < 1 and instr.maneuverModifier == 'left':
      jumped['n'] += 1
      return nav_instruction(instr.maneuverDistance - nearer_m, "turn", "left", remaining=instr.distanceRemaining,
                             junction=LEFT_25[0][3], ahead=[(LEFT_25[1][0] - s.s - nearer_m, "turn", "right", junction_of("right"))]), True
    return instr, True
  sim.nav_filter = nav
  while not (sim.trace and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap' and sim.v < 0.05):
    drive(sim, DT_MDL)
    assert sim.t < 90.
  d_line = sim.tp.follower.distance
  jumped['on'] = True
  drive(sim, 1.5)
  assert jumped['n'] == 1
  assert sim.tp.follower.distance < d_line - nearer_m + 0.5, (d_line, sim.tp.follower.distance)
  p = sim.trace[-1][2]
  # a reading that puts the junction behind the car standing at its line (10 m), or too near for the go's first-motion
  # re-check (5 m: under HELD_MIN_M, skeptic SIMFIX P2), is the left's position fault (skeptic m-behaviour3 P2): handed back,
  # held, still signalling; one that leaves it room, the left still waits for its gap
  waiting = ('undecided', 'position') if d_line - nearer_m < HELD_MIN_M else ('waitGap', '')
  assert (str(p.unprotectedLeft.state), p.unprotectedLeft.why if waiting[1] else '') == waiting and \
    p.turnSignal.side == LaneChange.left and p.shouldStop, (str(p.unprotectedLeft.state), p.unprotectedLeft.why, p.turnSignal.side, p.shouldStop)
  state['driver'] = driver
  return sim, state, drive(sim, 1.)


def left_line_veto_then_reroute(driver, reroute=True, nearer_m=0.):
  """auto_left_at_its_line (nearer_m: nearer_reading_at_the_left_line, a reading that much nearer before the veto): the
  driver overrules the car's left at its line for 1 s ('right', 'hazards': their lamp, out again after it, not yet having
  taken over; 'lost': our lamp put out, the port failed for the drive), the left handing back, the car held. A second
  later navd reroutes (reroute_at_the_left). Returns (sim, the maneuver id at the veto, 40 s of plans from the reroute)."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import LEFT_25, drive
  if nearer_m:
    sim, state, vetoed = nearer_reading_at_the_left_line(driver, nearer_m)
  else:
    sim, state, vetoed = auto_left_at_its_line(driver, veto_s=1.)
  assert any(str(p.controlPrompt) == 'leftUndecidedTakeControl' for p in vetoed), {str(p.controlPrompt) for p in vetoed}
  assert not any(p.tooFastToTurn for p in vetoed), "the left's handback, not a skip"
  assert sim.tp.vetoed is not None and sim.tp.vetoed.line, "the veto at the line is recorded, the junction's (VetoedTurn)"
  old_id = sim.tp.follower.maneuver_id
  if driver != 'lost':
    state['driver'] = 'none'
  drive(sim, 1.)
  assert sim.tp.vetoed is not None and sim.tp.vetoed.line, "the line record still there, the car standing, before the reroute"
  if reroute:
    sim.nav_filter = reroute_at_the_left(LEFT_25[0][3], LEFT_25[1][0])
  return sim, old_id, drive(sim, 40.)


# a reading of the left 5 or 10 m nearer than the rest while the car stands at its line, before the veto (both the left's
# position fault: 5 m leaves the follower under HELD_MIN_M): 10 m puts the follower's distance (the nearest reading) behind
# the car standing 9.3 m short of the junction (verification of tp/blinker,
# N1: the line record forgotten on the next frame, the car still standing, so navd's reroute took up the overruled left)
NEARER_M = [0., 5., 10.]


@pytest.mark.parametrize("nearer_m", NEARER_M)
@pytest.mark.parametrize("driver", ['right', 'hazards', 'lost'])
def test_a_reroute_after_a_veto_at_the_lefts_line_never_takes_up_the_overruled_left(driver, nearer_m):
  """A veto at the autonomous left's line is the left's handback (6.6's exception), not a skip, and it ends the request for
  the left (B8). It is the junction's too: navd's reroute giving the same left there a new maneuver id never arms it again,
  so the car never relights the left the driver overruled, never prompts 'Signal Left' for it with our lamp put out, and
  never goes (verification of tp/blinker, K2: id 1 -> 3, our side left on every frame of the 40 s, and with our lamp put
  out 'Signal Left' for the whole stand). So too after a reading nearer than the car standing at the line (N1)."""
  sim, old_id, after = left_line_veto_then_reroute(driver, nearer_m=nearer_m)
  new_id = sim.tp.follower.maneuver_id
  assert new_id != old_id, "navd's reroute is a new maneuver"
  went = [p for p in after if p.desire == Desire.turnLeft]
  relit = [p for p in after if p.turnSignal.side == LaneChange.left]
  prompts = [p for p in after if p.turnSignal.prompt != LaneChange.none]
  taken = f"{len(went)} turnLeft frames, {len(relit)} our side left, {len(prompts)} prompting"
  assert not went and not relit and not prompts, \
    f"id {old_id} -> {new_id}: the left the driver overruled ({driver}) at its line taken up again: {taken}"
  assert not any(p.tooFastToTurn for p in after), "the left's handback stands: never a skip that lets go of the car at its line"
  assert all(p.shouldStop for p in after), "held at the line"
  states = {str(p.unprotectedLeft.state) for p in after}
  assert states <= {'off', 'undecided'}, f"the left judged again after its handback: {sorted(states)}"  # off: navd's gap


@pytest.mark.parametrize("nearer_m", NEARER_M)
@pytest.mark.parametrize("driver", ['right', 'hazards'])
def test_taking_over_after_a_reroute_at_the_lefts_line_never_asks_for_the_overruled_side(driver, nearer_m):
  """B8's third guarantee across the reroute: the driver then takes over (the gas disengages, the car standing at the
  line), and nothing armed for the overruled left is handed over: never "Signal Off / Signal left yourself"."""
  sim, _, _ = left_line_veto_then_reroute(driver, nearer_m=nearer_m)
  told = [p for p in (sim.step(engaged=False) for _ in range(int(4. / DT_MDL))) if p.turnSignal.handedOver != LaneChange.none]
  assert not told, f"'Signal Off / Signal {told[0].turnSignal.handedOver} yourself' after the driver overruled the left and navd rerouted"


@pytest.mark.parametrize("nearer_m", [0., 10.])
@pytest.mark.parametrize("driver", ['right', 'lost'])
def test_without_a_reroute_the_overruled_left_stays_handed_back(driver, nearer_m):
  sim, _, after = left_line_veto_then_reroute(driver, reroute=False, nearer_m=nearer_m)
  assert not any(p.desire == Desire.turnLeft or p.turnSignal.side != LaneChange.none or
                 p.turnSignal.prompt != LaneChange.none for p in after)
  assert all(p.shouldStop for p in after)


def test_a_line_veto_is_forgotten_once_the_car_is_past_its_junction():
  """The carried end of a veto at the left's line is for the same left at the line the car stands at (navd's reroute
  there, a reading glitch's new id). Once the car is past that junction (the driver took over and drove it, then engaged
  again) it is forgotten: a left under a new id within SAME_TURN_M after it is a turn of its own, and without junction
  data same_junction can't tell the two apart, so ending its request would take it with no lamp and no prompt, nothing
  said (a skip's carry, M3's, is said out loud)."""
  from types import SimpleNamespace
  from openpilot.selfdrive.turnpilot.route_follower import LEFT, TURN, Maneuver
  from openpilot.selfdrive.turnpilot.turnpilotd import TurnPilot, VetoedTurn
  vetoed_left = Maneuver(TURN, LEFT, "turn", "left", 90., geometry=True, bearings=(0., 270.))
  next_left = Maneuver(TURN, LEFT, "turn", "left", 90.)  # no junction data
  tp = TurnPilot()
  tp.odo_m = 90.7  # standing at the left's line, 9.3 m short of its junction
  tp.vetoed = VetoedTurn(vetoed_left, 100., 'driverSignal', 1, False, line=True, veto_odo=90.7)
  tp.carry_veto(vetoed_left, SimpleNamespace(maneuver_id=3, distance=9.3), 0.)
  assert tp.turn_signal.ended_id == 3, "at the line the same left under a new id stays ended"
  tp.odo_m = 112.  # past the junction, engaged again, the next left 18 m on (30 m past the vetoed junction)
  tp.carry_veto(next_left, SimpleNamespace(maneuver_id=5, distance=18.), 8.)
  assert tp.turn_signal.ended_id != 5, "the next left's request ended with nothing said"
  assert tp.vetoed is None


def test_a_line_veto_stays_while_the_car_stands_where_it_was_vetoed():
  """The follower's distance is the nearest of navd's readings, so one reading off nearer while the car stands at the left's
  line (a fix off at a standstill, under UNCERTAIN_M of spread) can put the junction the line record keeps at or behind the
  car (verification of tp/blinker, N1: -0.75 m). The car standing where the veto found it is never past that junction: the
  record stays, and the same left under navd's new id stays ended, until the car has driven LINE_VETO_MOVED_M from there."""
  from types import SimpleNamespace
  from openpilot.selfdrive.turnpilot.route_follower import LEFT, TURN, Maneuver
  from openpilot.selfdrive.turnpilot.turnpilotd import LINE_VETO_MOVED_M, TurnPilot, VetoedTurn
  vetoed_left = Maneuver(TURN, LEFT, "turn", "left", 90., geometry=True, bearings=(0., 270.))
  next_left = Maneuver(TURN, LEFT, "turn", "left", 90.)  # no junction data
  tp = TurnPilot()
  tp.odo_m = 100.75  # standing at the line, the nearest reading 0.75 m behind the car
  tp.vetoed = VetoedTurn(vetoed_left, 100., 'driverSignal', 1, False, line=True, veto_odo=100.75)
  for k in range(int(40. / DT_MDL)):  # 40 s standing, and a reroute's new ids for the same left
    tp.odo_m += 0.001
    tp.carry_veto(vetoed_left, SimpleNamespace(maneuver_id=3 + k // 200, distance=-0.75), 0.)
    assert tp.vetoed is not None and tp.turn_signal.ended_id == 3 + k // 200, k
  assert tp.odo_m < 100.75 + LINE_VETO_MOVED_M
  # driven from the line (the driver took over and drove the left, then engaged again), past the junction as recorded too:
  # forgotten, the next left of a new id after it keeps its lamp and prompt
  tp.odo_m = 100.75 + LINE_VETO_MOVED_M + 0.1
  tp.carry_veto(next_left, SimpleNamespace(maneuver_id=9, distance=18.), 8.)
  assert tp.turn_signal.ended_id != 9 and tp.vetoed is None


def test_a_line_veto_ends_the_same_left_within_its_readings_spread_never_the_next_left_on():
  """A line veto's record is for the left at the line the car stands at: carried to navd's new id for that left wherever its
  readings put it within UNCERTAIN_M of spread and a margin (LINE_VETO_SAME_M: K2's reroute at 0 m, N1's nearer reading at
  10 m, the spread's whole 15 m), never to another left past it on the same road (skeptic m-behaviour3 P1: the next left 25
  m on, same bearings or none, was taken for the vetoed one within SAME_TURN_M, its request ended with nothing said, and the
  car reached it with no lamp, no 'Signal Left' and no handover)."""
  from types import SimpleNamespace
  from openpilot.selfdrive.turnpilot.route_follower import LEFT, TURN, UNCERTAIN_M, Maneuver
  from openpilot.selfdrive.turnpilot.turnpilotd import LINE_VETO_SAME_M, TurnPilot, VetoedTurn
  vetoed_left = Maneuver(TURN, LEFT, "turn", "left", 90., geometry=True, bearings=(0., 270.))
  assert UNCERTAIN_M < LINE_VETO_SAME_M < 25.
  for off, carried in ((0., True), (10., True), (UNCERTAIN_M, True), (-UNCERTAIN_M, True), (25., False), (40., False)):
    for nxt in (vetoed_left, Maneuver(TURN, LEFT, "turn", "left", 90.)):  # same bearings, or no junction data
      tp = TurnPilot()
      tp.odo_m = 90.7  # standing at the left's line, 9.3 m short of its junction
      tp.vetoed = VetoedTurn(vetoed_left, 100., 'driverSignal', 1, False, line=True, veto_odo=90.7)
      tp.carry_veto(nxt, SimpleNamespace(maneuver_id=3, distance=9.3 + off), 0.)
      assert (tp.turn_signal.ended_id == 3) == carried, (off, nxt.bearings)
      assert tp.vetoed is not None, "standing at the line, the record stays"


NEXT_LEFT_M = 25.
NEXT_LEFT_JUNCTIONS = {'nodata': None, 'samebearings_present': (0., 270., 2, 'present', 11.18, 1),
                       'samebearings_none': (0., 270., 2, 'none')}


@pytest.mark.parametrize("take_over", [False, True])
@pytest.mark.parametrize("kind", list(NEXT_LEFT_JUNCTIONS))
def test_a_different_left_after_a_line_veto_keeps_its_signal(kind, take_over):
  """Skeptic m-behaviour3 P1, closed loop: the driver's right lamp vetoes the autonomous left at its line (the left hands
  back, the car held), and a second later navd reroutes to a route whose first step is another left NEXT_LEFT_M past the
  vetoed junction (its junction data none, or the same bearings). Standing at the vetoed left's line, the car never lights
  its left lamp nor prompts 'Signal Left' (that would say it turns where the driver overruled it). Once the driver has
  taken over, driven through the vetoed junction and engaged again, the next left is the car's own: our lamp lit for it or
  the driver prompted to signal it, or skipped out loud (tooFastToTurn: the sim's unmeasured left range); never ended with
  nothing said (it was: carry_veto's line branch took it for the vetoed left within SAME_TURN_M)."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import JUNCTION, drive
  from openpilot.selfdrive.turnpilot.turnpilotd import LINE_VETO_MOVED_M
  junction, at, nxt = NEXT_LEFT_JUNCTIONS[kind], JUNCTION + NEXT_LEFT_M, 1500.
  k = [0]

  def nav(s, instr):
    k[0] += 1
    if k[0] == 1:
      return nav_instruction(at - s.s, "turn", "left", remaining=nxt - s.s, junction=junction), False
    if s.s < at + 10.:
      return nav_instruction(at - s.s, "turn", "left", remaining=nxt - s.s, junction=junction, route_count=1,
                             ahead=[(nxt - s.s, "turn", "right", junction_of("right"))]), True
    return nav_instruction(nxt - s.s, "turn", "right", remaining=nxt - s.s, junction=junction_of("right"), route_count=1), True

  sim, state, vetoed = auto_left_at_its_line('right', veto_s=1.)
  assert any(str(p.controlPrompt) == 'leftUndecidedTakeControl' for p in vetoed)
  state['driver'] = 'none'
  drive(sim, 1.)
  assert sim.tp.vetoed is not None and sim.tp.vetoed.line
  if take_over:
    sim.go_taps = True
    sim.take_over_on = ('unprotectedTurnTakeControl', 'leftUndecidedTakeControl')
  sim.nav_filter = nav
  i0 = len(sim.trace)
  drive(sim, 45.)
  after = sim.trace[i0:]
  at_line = [p for s, _, p in after if s < JUNCTION + LINE_VETO_MOVED_M - 9.3]
  lit = [p for p in at_line if p.turnSignal.side == LaneChange.left or p.turnSignal.prompt == LaneChange.left]
  assert at_line and not lit, f"{kind}: at the vetoed left's line, our left lamp or 'Signal Left' on {len(lit)} of {len(at_line)} frames"
  if not take_over:
    assert all(p.shouldStop for _, _, p in after), "held at the vetoed line"
    return
  near = [p for s, v, p in after if JUNCTION + LINE_VETO_MOVED_M < s < at and v > 0.5 and
          p.state != custom.TurnpilotPlan.State.unavailable]
  assert near, "the driver took over and drove on toward the next left"
  said = [p for p in near if p.turnSignal.side == LaneChange.left or p.turnSignal.prompt == LaneChange.left or p.tooFastToTurn]
  ids = f"ended id {sim.tp.turn_signal.ended_id}, maneuver id {sim.tp.follower.maneuver_id}"
  assert said, f"{kind}: the left {NEXT_LEFT_M} m past the vetoed one approached with no lamp, no prompt and no skip ({ids})"


@pytest.mark.parametrize("driver", ['right', 'lost'])
def test_a_reroute_after_a_veto_at_g1s_line_never_relights_the_left(driver):
  """TurnPilotAutoLeft off (G1): the latch holds the car at the left's line, handed back (unprotectedTurnTakeControl), and a
  veto there ends the request for the left. G1's left pulses on the approach, so at its line the turn is committed and the
  follower holds it through navd's reroute under its own id (route_follower: a committed turn): the request stays ended,
  nothing relights, prompts or hands over the left. (Passing before K2's fix too: the guard for G1's half of it.)"""
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
  state['driver'] = driver
  [sim.step(engaged=True) for _ in range(int(1. / DT_MDL))]
  if driver != 'lost':
    state['driver'] = 'none'
  [sim.step(engaged=True) for _ in range(int(1. / DT_MDL))]
  assert sim.tp.vetoed is not None and sim.tp.vetoed.line, "G1's line veto is recorded too"
  old_id = sim.tp.follower.maneuver_id
  sim.nav_filter = reroute_at_the_left(TURNS_AHEAD[0][3], TURNS_AHEAD[1][0])
  after = [sim.step(engaged=True) for _ in range(int(20. / DT_MDL))]
  assert sim.tp.follower.maneuver_id == old_id and sim.tp.turn_committed, "G1's committed left is held through the reroute"
  relit = [p for p in after if p.turnSignal.side == LaneChange.left]
  prompts = [p for p in after if p.turnSignal.prompt != LaneChange.none]
  assert not relit and not prompts, f"the car signals left on {len(relit)} plans, prompts on {len(prompts)}, after the reroute"
  assert not any(p.tooFastToTurn or p.desire == Desire.turnLeft for p in after)
  assert all(p.shouldStop for p in after)
  told = [p for p in (sim.step(engaged=False) for _ in range(int(4. / DT_MDL))) if p.turnSignal.handedOver != LaneChange.none]
  assert not told


# ---------------------------------------------------------------- hazards holding the port off: said on the reason line (7.7)

@pytest.mark.parametrize("turn,at", [('right', 6.), ('tleft', 30.)])
def test_hazards_holding_the_port_off_say_so_on_the_reason_line(turn, at):
  """DESIGN 7.7: hazards hold the port off (it claims nothing while they're on), so a turn the car finishes with them on
  (they came on after it started: "Finishing the turn") is not signalled by the car, and the plan's reason line says so
  while it lasts. A reason of higher rank (the scene's, a missed turn, the lane planner's, the map's) still outranks it.
  The turn has started once its desire is out: the T-left's from its lead (40 m), the right's from its one pulse 8 m out
  (desire_timing.CURB_TURN_PULSE_M), so the hazards come on 30 m and 6 m out."""
  t = turn_of(turn)
  sim = signal_sim(maneuvers=t.maneuvers if turn == 'tleft' else RIGHT_TURN)
  plans = drive_to(sim, at)
  assert sim.tp.desires.started and sim.v > 2. and plans[-1].turnSignal.side == t.side
  assert plans[-1].reason != "Hazards on: car not signalling"
  sim.port_state = {'status': 'ready', 'block': 'hazard', 'hazard': True, 'leftLamp': True, 'rightLamp': True}
  hazards = [sim.step(engaged=True) for _ in range(int(1. / DT_MDL))]
  assert sim.distance() > 0., "the hazards' second still short of the junction"
  assert str(hazards[0].turnSignal.notice) == 'changedMidTurn'
  assert all(p.turnSignal.side == t.side for p in hazards)
  assert all(p.reason == "Hazards on: car not signalling" for p in hazards), sorted({p.reason for p in hazards})
  sim.port_state = {'status': 'ready'}
  off = [sim.step(engaged=True) for _ in range(3)]
  assert all(p.reason != "Hazards on: car not signalling" for p in off)
