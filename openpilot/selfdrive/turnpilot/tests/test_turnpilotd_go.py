"""turnpilotd's certified go (go_override.GoCertifier; DESIGN phase 4 decision 1, G5): TurnPilot's go from a standstill is
asserted to plannerd (turnpilotPlan.goCertified) for its bounded lift only when nothing in the car's path within 15 m, no
lead and no hold of TurnPilot's is there, the driver isn't on a pedal, and TurnPilotGoOverride is on; it is withdrawn the
frame any of that stops holding, and it ends on its own at 3 s, 3 m/s or the model's takeover."""
from collections import deque

from openpilot.cereal import messaging
from openpilot.common.params import UnknownKeyName
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.go_override import GO_ACCEL, GO_CLEAR_M, GO_WINDOW_S, TAKEOVER_S
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.scene.tracker import RawBox, RawFrame
from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
from openpilot.selfdrive.turnpilot.tests.synthetic import model_v2
from openpilot.selfdrive.turnpilot.tests.test_go_override import WARM_S, new_planner, planner_sm
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import (LEFT_AT_A_RED, WALKER, FakeParams, Sim, go_from_the_hold, stand, stop_at,
                                                                 walker_stops_the_car)
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import JUNCTION, V_25, drive, left_sim
from openpilot.selfdrive.turnpilot.turnpilotd import GO_OVERRIDE_DEFAULT, NO_MOVE_REASON, NO_MOVE_S, go_override_for

Desire = messaging.log.Desire


def held_at_the_red(**kw):
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')], **kw)
  return sim


def go(plan):
  g = plan.goCertified
  return str(g.cause), g.id, g.t


def test_a_go_from_turnpilots_hold_is_certified_and_asserted_every_frame_of_it():
  sim = held_at_the_red()
  p = go_from_the_hold(sim)
  assert not p.shouldStop and p.desire == Desire.turnLeft
  cause, go_id, t = go(p)
  assert cause == 'turnRelease' and go_id >= 1 and t == 0.
  # the model holds the car: asserted on, its clock running, until the no-move limit withdraws it with the handback
  plans = stand(sim, NO_MOVE_S - 0.2)
  assert all(go(q)[:2] == ('turnRelease', go_id) for q in plans)
  assert abs(go(plans[-1])[2] - (NO_MOVE_S - 0.2)) < 1e-4
  plans = stand(sim, 0.4)
  withdrawn = next(q for q in plans if q.controlPrompt == 'cantStartTakeControl')
  assert go(withdrawn)[0] == 'none' and all(go(q)[0] == 'none' for q in plans[plans.index(withdrawn):])
  assert sim.tp.go_cert.end == 'noMove'


def test_the_go_ends_on_the_models_takeover():
  sim = held_at_the_red()
  go_from_the_hold(sim)
  # the model drives on (the Sim's car pulls away): its takeover holds TAKEOVER_S and the go is over, followed
  plans = [sim.step(engaged=True) for _ in range(round(TAKEOVER_S / DT_MDL) + 2)]
  assert go(plans[0])[0] == 'turnRelease' and go(plans[-1])[0] == 'none'
  assert sim.tp.go_cert.end in ('takeover', 'speed') and sim.tp.go_cert.taken_over


def test_the_go_ends_at_its_window():
  # in the junction (never withdrawn for not moving, G3): the model holds 3 s, the go's window ends it there
  sim = held_at_the_red()
  go_from_the_hold(sim)
  while sim.tp.desires.turned_deg < 45.:
    sim.step(engaged=True, yaw_rate=0.3 if sim.v > 0.5 else 0.)
    assert sim.t < 200.
  walker_stops_the_car(sim)
  plans = stand(sim, GO_WINDOW_S + 0.5)
  certified = [q for q in plans if go(q)[0] == 'turnRelease']
  assert abs(len(certified) * DT_MDL - GO_WINDOW_S) < 1e-6 and go(plans[-1])[0] == 'none' and sim.tp.go_cert.end == 'window'


def test_no_go_is_certified_with_something_in_the_path_a_lead_or_the_param_off():
  def certified(setup):
    sim = held_at_the_red()
    setup(sim)
    return go(go_from_the_hold(sim))[0]
  assert certified(lambda sim: None) == 'turnRelease'

  def param_off(sim):
    sim.tp.go_override = False
  assert certified(param_off) == 'none'

  def close_lead(sim):
    sim.model_kw = {'lead_prob': 0.9, 'lead_x': GO_CLEAR_M - 3.}
  assert certified(close_lead) == 'none'

  def far_lead(sim):
    sim.model_kw = {'lead_prob': 0.9, 'lead_x': GO_CLEAR_M + 5.}
  assert certified(far_lead) == 'turnRelease'


def raw_person(x, y, half=0.3):
  import math
  bearing = math.degrees(math.atan2(y, x))
  r = math.hypot(x, y)
  d = math.degrees(math.atan2(half, r))
  return RawBox('road', 'pedestrian', 0.4, bearing, (bearing - d, bearing + d), (x, y), r * 0.95, ())


def test_an_in_path_detection_within_15_m_blocks_the_go_and_withdraws_it_the_frame_it_comes():
  """Any detection in the corridor ahead within GO_CLEAR_M, a raw box of this frame's detector run (tracked or not, at any
  confidence the decoder keeps), blocks a go and withdraws one out."""
  def runs(boxes):
    return lambda sim: RawFrame(boxes(sim), 'road', sim.t)
  sim = held_at_the_red()
  sim.raw = runs(lambda sim: [raw_person(10., 0.5)])
  assert go(go_from_the_hold(sim))[0] == 'none'
  # a box off to the side, or past 15 m, doesn't
  for x, y in ((10., 4.), (GO_CLEAR_M + 3., 0.)):
    sim = held_at_the_red()
    sim.raw = runs(lambda sim, x=x, y=y: [raw_person(x, y)])
    assert go(go_from_the_hold(sim))[0] == 'turnRelease', (x, y)
  # certified, then a person comes into the path 8 m ahead mid-go: withdrawn that very frame
  box = {'on': False}
  sim = held_at_the_red()
  sim.raw = runs(lambda sim: [raw_person(8., -0.3)] if box['on'] else [])
  assert go(go_from_the_hold(sim))[0] == 'turnRelease'
  assert all(go(q)[0] == 'turnRelease' for q in stand(sim, 0.5))
  box['on'] = True
  q = stand(sim, DT_MDL)[0]
  assert go(q)[0] == 'none' and sim.tp.go_cert.end == 'inPath'


def test_a_tracked_walker_coming_into_the_path_or_a_hold_withdraws_it():
  sim = held_at_the_red()
  go_from_the_hold(sim)
  stand(sim, 0.3)
  q = stand(sim, DT_MDL, objects=WALKER)[0]  # a walker 4 m ahead in the path: TurnPilot holds, and the go is withdrawn
  assert go(q)[0] == 'none' and sim.tp.go_cert.end in ('inPath', 'hold')
  # the walker clears: TurnPilot's hold lets go at a standstill, a new go with a new id (its own window)
  first = sim.tp.go_cert.id
  plans = stand(sim, 1., objects=WALKER) + stand(sim, DT_MDL)
  assert all(go(q)[0] == 'none' for q in plans[:-1]) and go(plans[-1])[:2] == ('turnRelease', first + 1)


def test_a_pedal_or_a_disengage_withdraws_it():
  for kw in ({'engaged': False}, {'gas': True, 'engaged': False}):
    sim = held_at_the_red()
    go_from_the_hold(sim)
    sim.v = 0.
    q = sim.step(**kw)
    assert go(q)[0] == 'none' and sim.tp.go_cert.end == 'driver', kw


def test_the_unprotected_lefts_go_is_certified_as_its_own_and_withdrawn_with_it():
  """The autonomous left's go is certified with its own cause. A car coming into the oncoming band as the car first moves
  (the model not yet taking over, the lift moving it) withdraws the left's go short of the oncoming lanes: the certified go
  goes that same frame."""
  state = {'car': None}

  def cars(sim):
    return [] if state['car'] is None else [(state['car'] - V_25 * (sim.t - state['t']), 3.6)]
  sim = left_sim(cars)
  plans = []
  while not any(p.desire == Desire.turnLeft for p in plans[-1:]):
    plans += drive(sim, DT_MDL)
    assert sim.t < 60.
  assert go(plans[-1])[0] == 'unprotectedLeft' and str(plans[-1].unprotectedLeft.state) == 'go'
  sim.model_holds = True  # the model hasn't taken over: the car rolls on the lift alone
  state['car'], state['t'] = sim.s + 90., sim.t
  more = drive(sim, 1.5)
  out = next(i for i, p in enumerate(more) if str(p.unprotectedLeft.state) != 'go')
  assert all(go(p)[0] == 'unprotectedLeft' for p in more[:out]) and go(more[out])[0] == 'none'
  assert sim.tp.go_cert.end in ('left', 'hold') and not sim.tp.go_cert.taken_over


def test_a_faulted_or_disabled_plan_carries_no_go():
  sim = held_at_the_red()
  go_from_the_hold(sim)
  sim.long_control = False  # openpilot stops controlling speed: TurnPilot faults
  q = sim.step(engaged=True)
  assert go(q)[0] == 'none' and not sim.tp.go_cert.active


def test_go_override_param():
  assert GO_OVERRIDE_DEFAULT is True
  assert go_override_for(FakeParams()) is True  # unset: on (the autonomous left and the stop sign's go depend on it)
  for value, on in ((True, True), (False, False), ('1', True), ('0', False), (b'0', False), (b'1', True)):
    assert go_override_for(FakeParams(TurnPilotGoOverride=value)) is on, value

  class Unknown:
    def get(self, key):
      raise UnknownKeyName(key)
  assert go_override_for(Unknown()) is True


# --- plannerd in the loop: the real planner lifting the car for turnpilotd's go, a model that holds or takes over ---

def holding(sim):
  """G0's stand: the model holds the car (shouldStop under 0.3 m/s, no motion planned), braking gently when it rolls."""
  a = -0.4 if sim.v > 0.05 else 0.
  return model_v2(*sim.model_lanes, holds=True, accel=a, should_stop=sim.v < 0.3 and a < 0.1, **sim.model_kw)


def taking_over(sim):
  """Holds at a standstill; rolling, it means to go (shouldStop false, motion planned, 0.8 m/s^2)."""
  return holding(sim) if sim.v < 0.3 else model_v2(*sim.model_lanes, accel=0.8, **sim.model_kw)


def attach_planner(sim, model_fn, lag_s=0.3):
  """plannerd in sim's loop from now: the real LongitudinalPlanner takes each frame's turnpilotPlan, model_fn's modelV2 and
  the car's state, and the car's accel follows its aTarget lag_s late (-1.0 m/s^2 on its shouldStop: longcontrol's
  stopping). The car stands held when it is attached."""
  lp = new_planner(warm=False)
  queue = deque([0.] * max(round(lag_s / DT_MDL), 1))

  def accel(sim, plan):
    cs = sim.sm['carState']
    lp.update(planner_sm(sim.v, plan, sim.sm['modelV2'], gas=cs.gasPressed, enabled=sim.sm['carControl'].enabled))
    sim.planned.append((float(lp.output_a_target), bool(lp.output_should_stop), lp.go_lift.lifting))
    queue.append(-1. if lp.output_should_stop else float(lp.output_a_target))
    return queue.popleft()
  sim.model, sim.accel, sim.planned = model_fn, accel, []
  return lp


def held_at_its_red(model_fn, lag_s=0.3):
  """LEFT_AT_A_RED held at its red 10 m before the junction with plannerd in the loop, standing WARM_S, then its green.
  Returns (sim, planner, the index in sim.trace of the go)."""
  sim = held_at_the_red()
  sim.go_taps, sim.tp.oracle_ok = False, True
  light = {'state': 'red'}

  def lights(sim):
    msg = messaging.new_message('turnpilotLights')
    msg.turnpilotLights.source = 'simOracle'
    msg.turnpilotLights.lights = [{'state': light['state'], 'distance': 290. - sim.s, 'instance': 'trafficLight 1'}]
    return msg.as_reader().turnpilotLights
  sim.lights = lights
  stop_at(sim, 290.)
  lp = attach_planner(sim, model_fn, lag_s)
  for _ in range(round(WARM_S / DT_MDL)):
    assert sim.step(engaged=True).shouldStop
  light['state'] = 'green'
  while sim.trace[-1][2].shouldStop:
    sim.step(engaged=True)
    assert sim.t < 120.
  return sim, lp, len(sim.trace) - 1


def test_a_go_the_model_never_takes_over_is_withdrawn_at_its_window_and_handed_back():
  """The lift rolls the car on (+0.5 m/s^2, 2.25 m at most); the model never takes over, and the go's window ends: TurnPilot
  withdraws it at once (stopping the car at STOP_DECEL, never harder than -2.0) and hands back loudly, the car held standing,
  not left to creep on the model's gentle braking and stand silently."""
  for lag in (0.15, 0.3):
    sim, lp, go_i = held_at_its_red(holding, lag)
    s_go = sim.trace[go_i][0]
    assert go(sim.trace[go_i][2])[0] == 'turnRelease'
    plans = [sim.step(engaged=True) for _ in range(round((GO_WINDOW_S + 3.) / DT_MDL))]
    first = next(i for i, p in enumerate(plans) if p.controlPrompt == 'cantStartTakeControl')
    assert abs(first * DT_MDL - GO_WINDOW_S) <= DT_MDL + 1e-6 and plans[first].reason == NO_MOVE_REASON
    assert all(p.controlPrompt == 'cantStartTakeControl' for p in plans[first:]) and plans[-1].shouldStop
    assert sim.v < 0.05 and sim.s - s_go <= 0.5 * GO_ACCEL * GO_WINDOW_S ** 2 + 0.75 + 0.2, (lag, sim.s - s_go)
    assert all(a >= -2. for a, _, _ in sim.planned) and sum(lifted for _, _, lifted in sim.planned) <= round(GO_WINDOW_S / DT_MDL)
    assert sim.tp.go_cert.end == 'window' and not sim.tp.go_cert.taken_over


def test_a_go_the_model_takes_over_is_driven_on_with_nothing_said():
  sim, lp, go_i = held_at_its_red(taking_over)
  plans = [sim.step(engaged=True) for _ in range(round(6. / DT_MDL))]
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert sim.tp.go_cert.end == 'takeover' and sim.tp.go_cert.taken_over and sim.v > 2.
  lifted = sum(lifted for _, _, lifted in sim.planned)
  assert 0 < lifted < round(GO_WINDOW_S / DT_MDL)


def test_a_go_whose_lift_ended_early_stands_again_and_is_handed_back():
  """A phantom in the path for one detector run mid-lift withdraws the go (inPath), and the lift with it; the model holds and
  the car stands again short of the junction, nothing else holding it: withdrawn and handed back after NO_MOVE_S standing,
  never a silent stand (before G5's rule it was 90 s: stopped_unheld near a control)."""
  sim, lp, go_i = held_at_its_red(holding)
  phantom = {'on': False}
  sim.raw = lambda sim: RawFrame([raw_person(6., 0.)], 'road', sim.t) if phantom['on'] else None
  plans = [sim.step(engaged=True) for _ in range(round(1.2 / DT_MDL))]
  assert sim.v > 0.3 and all(go(p)[0] == 'turnRelease' for p in plans)
  phantom['on'] = True
  plans = [sim.step(engaged=True)]
  phantom['on'] = False
  assert go(plans[0])[0] == 'none' and sim.tp.go_cert.end == 'inPath'
  plans += [sim.step(engaged=True) for _ in range(round(8. / DT_MDL))]
  stood = next(i for i, p in enumerate(plans) if sim.trace[len(sim.trace) - len(plans) + i][1] < 0.3)
  handed = next(i for i, p in enumerate(plans) if p.controlPrompt == 'cantStartTakeControl')
  assert (handed - stood) * DT_MDL <= NO_MOVE_S + 0.2 and plans[-1].shouldStop and sim.v < 0.05


def test_the_unprotected_lefts_lifted_go_never_reaches_the_oncoming_lanes_unfollowed():
  """The autonomous left's go from its 9 m line, the model holding: the lift has the car past 0.3 m/s inside LEFT_NO_MOVE_S
  (1.0 s) at a 0.3 s lag, so its no-move withdrawal doesn't come; the model never takes over, and the window's end withdraws
  it: the car stops short of the oncoming lanes (ONCOMING_ENTRY_M before the maneuver point) and is handed back."""
  sim = left_sim()
  while not (sim.trace and sim.trace[-1][2].shouldStop and sim.v < 0.05 and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap'):
    drive(sim, DT_MDL)
    assert sim.t < 60.
  attach_planner(sim, holding)
  plans = []
  while not any(p.desire == Desire.turnLeft for p in plans[-1:]):
    plans += drive(sim, DT_MDL)
    assert sim.t < 90.
  assert go(plans[-1])[0] == 'unprotectedLeft'
  after = drive(sim, GO_WINDOW_S + 3.)
  first = next(i for i, p in enumerate(after) if p.controlPrompt == 'cantStartTakeControl')
  assert abs(first * DT_MDL - GO_WINDOW_S) <= DT_MDL + 1e-6, first * DT_MDL  # the window, not LEFT_NO_MOVE_S
  assert sim.v < 0.05 and JUNCTION - sim.s > ONCOMING_ENTRY_M and after[-1].shouldStop


def test_a_takeover_the_car_doesnt_move_on_is_still_the_no_move_withdrawal():
  """The model's plan says go (the takeover predicate holds, the lift ends), but the car stands (a brake hold, an actuator
  that doesn't answer): the go isn't followed until the car moves, so the no-move withdrawal still comes at NO_MOVE_S."""
  sim = held_at_the_red()
  go_from_the_hold(sim)
  plans = []
  for _ in range(round((NO_MOVE_S + 0.3) / DT_MDL)):
    sim.v = 0.  # the default model drives on: its takeover holds while the car stands
    plans.append(sim.step(engaged=True))
  assert sim.tp.go_cert.end == 'takeover' and sim.tp.go_cert.taken_over
  assert plans[-1].controlPrompt == 'cantStartTakeControl' and plans[-1].shouldStop
