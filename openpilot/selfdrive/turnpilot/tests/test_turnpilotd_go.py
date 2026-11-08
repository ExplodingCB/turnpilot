"""turnpilotd's certified go (go_override.GoCertifier; DESIGN phase 4 decision 1, G5): TurnPilot's go from a standstill is
asserted to plannerd (turnpilotPlan.goCertified) for its bounded lift only when nothing in the car's path within 15 m, no
lead and no hold of TurnPilot's is there, the driver isn't on a pedal, and TurnPilotGoOverride is on; it is withdrawn the
frame any of that stops holding, and it ends on its own at 3 s, 3 m/s or the model's takeover."""
import dataclasses
from collections import deque

import pytest

from openpilot.cereal import messaging
from openpilot.common.params import UnknownKeyName
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.desire_timing import CURB_FIRST_PULSE_M, CURB_GO_SLACK_M, CURB_TURN_PULSE_M
from openpilot.selfdrive.turnpilot.go_override import GO_ACCEL, GO_CLEAR_M, GO_WINDOW_S, TAKEOVER_S
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.map_controls import TURN_LIGHT_REASON
from openpilot.selfdrive.turnpilot.scene.tracker import RawBox, RawFrame
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M, ONCOMING_ENTRY_M, SceneDecision
from openpilot.selfdrive.turnpilot.tests.synthetic import model_v2
from openpilot.selfdrive.turnpilot.tests.test_go_override import WARM_S, new_planner, planner_sm
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import (LEFT_AT_A_RED, WALKER, WALKER_CLEAR, FakeParams, Sim, curb_turn_from_a_red,
                                                                 go_from_a_red, go_from_the_hold, junction_of, oracle_lights, stand, stop_at,
                                                                 walker_stops_the_car)
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import (JUNCTION, LEFT_25, TEST_RANGE_M, V_25, drive, left_sim, road_runs,
                                                                      run_objects, signal_sim)
from openpilot.selfdrive.turnpilot.turnpilotd import (GO_OVERRIDE_DEFAULT, LEFT_NO_MOVE_S, LEFT_LAUNCH_SPEED, LEFT_LIFT_ROOM_M, NO_MOVE_REASON, NO_MOVE_S,
                                                       RELEASE_UNSIGNALIZED, STALL_PROMPT_S, go_override_for)
from openpilot.selfdrive.turnpilot.unprotected_left import LINE_D, MODEL_HOLD_REASON

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
  # in the junction (never withdrawn for not moving, G3): the model holds 3 s, the go's window ends it there. A curb-side
  # turn's: a far-side one released in the junction is never certified (skeptic G5). The right's go from its 10 m line sends
  # no desire, its one pulse goes rolling 8 m out (2026-11-06: curb_turn_from_a_red), and the car turns from there
  sim = Sim(RIGHT_AT_A_SIGNAL, controls=[(290., 'signal')])
  curb_turn_from_a_red(sim)
  while sim.tp.desires.turned_deg < 45.:
    sim.step(engaged=True, yaw_rate=-0.3 if sim.v > 0.5 else 0.)
    assert sim.t < 200.
  walker_stops_the_car(sim)
  plans = stand(sim, DT_MDL, objects=WALKER_CLEAR) + stand(sim, GO_WINDOW_S + 0.5 - DT_MDL)
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
  # the walker walks clear: TurnPilot's hold lets go at a standstill, a new go with a new id (its own window)
  first = sim.tp.go_cert.id
  plans = stand(sim, 1., objects=WALKER) + stand(sim, DT_MDL, objects=WALKER_CLEAR)
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


def coasting(sim):
  """Holds at a standstill; rolling, it 'takes over' and only keeps the speed the lift gave it (shouldStop false, a plan at
  its speed, no accel): the takeover predicate holds from 0.5 m/s, but the car never launches."""
  if sim.v < 0.3:
    return holding(sim)
  m = model_v2(*sim.model_lanes, accel=0., **sim.model_kw).as_builder()
  m.velocity.x = [float(sim.v)] * len(m.velocity.x)
  return m.as_reader()


def launching(sim):
  """Holds at a standstill; rolling, it goes at the left's own launch (1.5 m/s^2)."""
  return holding(sim) if sim.v < 0.3 else model_v2(*sim.model_lanes, accel=1.5, **sim.model_kw)


def left_go_with(model_fn):
  sim = left_sim()
  while not (sim.trace and sim.trace[-1][2].shouldStop and sim.v < 0.05 and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap'):
    drive(sim, DT_MDL)
    assert sim.t < 60.
  attach_planner(sim, model_fn)
  plans = []
  while not any(p.desire == Desire.turnLeft for p in plans[-1:]):
    plans += drive(sim, DT_MDL)
    assert sim.t < 90.
  return sim


def test_the_lefts_go_is_followed_into_the_oncoming_lanes_only_by_a_launch():
  """Skeptic probe (advisor, G5): the lift has the car rolling at 0.78 m/s, the model's plan keeps that speed (the takeover
  predicate holds, the lift ends) and it coasts through the oncoming lanes at 0.78 m/s, 10 s across them, nothing said; the
  left's clear time assumed its 1.5 m/s^2 launch. So the left's go is followed into the oncoming lanes only at
  LEFT_LAUNCH_SPEED or more: short of it, while the car can still stop LEFT_LAUNCH_ROOM_M short of them, it is withdrawn and
  handed back. A model launching as the left assumes goes through."""
  sim = left_go_with(coasting)
  after = drive(sim, 15.)
  assert sim.tp.go_cert.end == 'takeover'
  handed = [p for p in after if p.controlPrompt == 'cantStartTakeControl']
  assert handed and not any(str(p.unprotectedLeft.state) == 'committed' for p in after)
  assert sim.v < 0.05 and JUNCTION - sim.s > ONCOMING_ENTRY_M and after[-1].shouldStop
  sim = left_go_with(launching)
  after = drive(sim, 8.)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in after)
  assert any(str(p.unprotectedLeft.state) == 'committed' for p in after) and sim.s > JUNCTION
  assert LEFT_LAUNCH_SPEED == 2.0


def test_a_left_go_nearer_the_oncoming_lanes_than_the_lifts_nudge_is_never_lifted():
  """A left crept for a view goes 1 m short of the oncoming lanes: the lift's 2.25 m nudge would carry the car past where it
  can stop short of them with the model never having gone, so that go is never certified (why 'room'): G4's go, as before.
  From its line (9 m, 5 m from the oncoming lanes) it is."""
  for distance, cause in ((LINE_D, 'unprotectedLeft'), (ONCOMING_ENTRY_M + LEFT_LIFT_ROOM_M - 0.1, 'none'),
                          (ONCOMING_ENTRY_M + 1., 'none')):
    sim = left_sim()
    while not (sim.trace and sim.v < 0.05 and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap'):
      drive(sim, DT_MDL)
      assert sim.t < 60.
    sim.tp.left_now.go = True
    sim.tp.certify_go(sim.sm, sim.sm['carState'], True, [], None, SceneDecision(), False, distance)
    assert sim.tp.go_cert.cause == cause and sim.tp.go_lifted == (cause != 'none'), distance
  assert LEFT_LIFT_ROOM_M == 4.5


# --- skeptic G5: turnRelease rests on its junction's own certification, a green an accepted source reads ---

# a right turn at a mapped signal (stop line 290 m, the junction 300 m): curb-side, no oncoming approach to cross
RIGHT_AT_A_SIGNAL = [(300., "turn", "right", junction_of("right")), (1500., "turn", "left", junction_of("left"))]


def model_stops_at(s_stop):
  """The car's accel when the model alone stops it at s_stop (a red no source TurnPilot accepts reads: on the car this week
  the camera's reader is shadow, so only the model stops for a red), at the speed it needs, cruising before."""
  def accel(sim, plan):
    d = s_stop - sim.s
    if d <= 0.3:
      return -3.
    need = sim.v ** 2 / (2. * d)
    return -need if need > 0.5 else min(1., (sim.cruise - sim.v) / 2.)
  return accel


def camera_reads(state, s_head=308.):
  """turnpilotLights with no source and, riding along, the camera's shadow reader (turnpilotDetections.lights, as modeld
  sends it) reading its governing head at s_head state, every frame."""
  from openpilot.selfdrive.turnpilot.tests.test_camera_lights import detections
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

  def lights(sim):
    sim.sm.put('turnpilotDetections', detections(state, s_head - sim.s, shadow=True))
    return none
  return lights


def walker_clears_at_the_models_red(lights=None, controls=((290., 'signal'),), maneuvers=RIGHT_AT_A_SIGNAL, s_stop=289.,
                                    always_drive=False):
  """Skeptic G5 probe: RIGHT_AT_A_SIGNAL (or maneuvers) on red, the model stopping the car at s_stop, its line (no source
  TurnPilot accepts need read the red), plannerd in the loop with the model holding for the red throughout. A walker crosses
  in front (they have the walk while the car has the red): TurnPilot's in-path hold. They clear: the hold lets go at a
  standstill, unless another of TurnPilot's holds stands (a route turn's stop for want of a green at its signal, 2026-11-06:
  map_controls' turn_stop). lights: the Sim's lights function, or None. always_drive: TurnPilotAlwaysDrive (the car ships it
  on, params_keys.h; the Sim's default is off). Returns (sim, the plans from the walker clearing, the car's s there)."""
  sim = Sim(maneuvers, controls=list(controls))
  sim.tp.always_drive = always_drive
  sim.go_taps = False
  sim.lights = lights
  sim.accel = model_stops_at(s_stop)
  while not (sim.v < 0.05 and sim.s > 280.):
    sim.step(engaged=True)
    assert sim.t < 80.
  attach_planner(sim, holding)
  held = [sim.step(WALKER, engaged=True) for _ in range(round(2. / DT_MDL))]
  assert held[-1].shouldStop
  s0 = sim.s
  return sim, [sim.step(WALKER_CLEAR, engaged=True)] + [sim.step(engaged=True) for _ in range(round(8. / DT_MDL) - 1)], s0


def test_a_walker_clearing_at_a_red_only_the_model_holds_for_is_no_certified_go():
  """Skeptic G5 blocker 1: turnRelease certified only that TurnPilot's walker left the path, and plannerd lifted the model's
  red-light hold: the car rolled 3.0 m past where the model stopped for the red, 1.75 m past the line, at 1.5 m/s. At a
  signal it is certified only on a green an accepted source reads (map_controls.release_light), and on the car no source is
  accepted, nor is the camera's shadow reader's green (a phantom green would lift the car into a red); its red keeps it off
  too. Uncertified, the model's hold stands: no go, no lift, the car still. Since 2026-11-07 (Kellen: no handbacks) a turn at
  a light with no colour read is no TurnPilot stop and no handback; a red the camera reads still stops the car at the line."""
  for lights in (None, camera_reads('green'), camera_reads('red')):
    sim, plans, s0 = walker_clears_at_the_models_red(lights)
    assert all(go(p)[0] == 'none' for p in plans) and not any(lifted for _, _, lifted in sim.planned)
    assert sim.s - s0 < 0.1 and max(v for _, v, _ in sim.trace[-len(plans):]) < 0.1
    assert not any(p.controlPrompt == 'signalUnseenTakeControl' or p.reason == TURN_LIGHT_REASON for p in plans), \
      "never a handback for the light"


def test_turn_release_is_certified_on_an_accepted_green_only():
  """The same walker clearing with the sim's oracle (an accepted source) reading the signal: green certifies the go, red
  doesn't (its own hold stays); the camera's shadow red over the oracle's green keeps it off (it only ever keeps a go off).
  Where navd maps no signal at the junction (an unmapped one, a stop sign, cross traffic would go unchecked): none
  (RELEASE_UNSIGNALIZED). Whose go: the walker's release is a turn's go (turn_go) only with the turn's desire out or pulsed
  by the go. LEFT_AT_A_RED's T-left (desire out from its lead) and the right stood at its own hold line (its one pulse at
  the go: desire_timing CURB_GO_SLACK_M) are; since 2026-11-06 a right stood further out (the model's stop 11 m out) has no
  desire at the go, its pulse coming rolling 8 m out. There the release is certified only under TurnPilotAlwaysDrive
  (turnpilotd-followups; the car ships it on): with the switch off nothing certifies at all, the oracle's green included
  (review 2026-11-06), so its negatives are checked with the switch on, where the green does certify and they can fail.
  Each light is checked where a green certifies."""
  def certified(lights, controls=((290., 'signal'),), maneuvers=RIGHT_AT_A_SIGNAL, s_stop=289.):
    sim, plans, _ = walker_clears_at_the_models_red(lights, controls, maneuvers, s_stop)
    return {go(p)[0] for p in plans} - {'none'}

  def oracle(state, camera=None, s_signal=290.):
    def lights(sim):
      if camera is not None:
        camera_reads(camera)(sim)
      sim.tp.oracle_ok = True
      return oracle_lights(state, s_signal)(sim)
    return lights
  # the right's: its signal's line at its hold line, 8 m out, the model stopping it just short of that
  for maneuvers, s_stop, s_signal in ((LEFT_AT_A_RED, 289., 290.), (RIGHT_AT_A_SIGNAL, 300. - HOLD_LINE_M - 0.5, 292.)):
    case, kw = (maneuvers[0][2], s_stop), {'controls': ((s_signal, 'signal'),), 'maneuvers': maneuvers, 's_stop': s_stop}
    assert certified(oracle('green', s_signal=s_signal), **kw) == {'turnRelease'}, case
    assert certified(oracle('red', s_signal=s_signal), **kw) == set(), case
    assert certified(oracle('green', camera='red', s_signal=s_signal), **kw) == set(), case
    assert certified(None, **{**kw, 'controls': ()}) == set(), case
  # the right stood 11 m out, its desire not out, under the switch: the green certifies it and lifts the model's hold (the
  # car rolls on toward its 8 m pulse), never a red, the camera's red over the oracle's green, nor a junction with no signal
  # mapped: no go, no lift, the car where the model stopped it
  def released(lights, controls=((290., 'signal'),)):
    sim, plans, s0 = walker_clears_at_the_models_red(lights, controls, always_drive=True)
    return {go(p)[0] for p in plans} - {'none'}, any(lifted for _, _, lifted in sim.planned), sim.s - s0
  causes, lifted, moved = released(oracle('green'))
  assert causes == {'turnRelease'} and lifted and moved > 1., (causes, lifted, moved)
  for name, lights, controls in (('red', oracle('red'), ((290., 'signal'),)), ('camera red', oracle('green', camera='red'), ((290., 'signal'),)),
                                 ('unmapped', None, ())):
    causes, lifted, moved = released(lights, controls)
    assert causes == set() and not lifted and moved < 0.1, (name, causes, lifted, moved)
  assert RELEASE_UNSIGNALIZED is False


def test_a_curb_side_turns_go_from_further_out_than_its_pulse_has_its_no_move_clock_with_the_switch_off():
  """tests-go-red risk 1 (2026-11-06): with TurnPilotAlwaysDrive off, a walker's hold let go 11 m short of a right at a green
  the oracle reads (accepted) is TurnPilot's go from a standstill, but the right's one pulse is 8 m out, so the go was no
  turn_go and nothing armed its no-move clock: the model holding the car, it stood silent ('Green (sim oracle)') until
  carStoppedTakeControl at 90 s, where HEAD (the desire out from 40 m) handed back at 3 s. The curb-side go now runs the
  no-move clock from the go as any go of TurnPilot's from a standstill: NO_MOVE_S standing with nothing else holding the car,
  then the go withdrawn, the car held standing (shouldStop) and handed back loudly (cantStartTakeControl). Never certified
  with the switch off (no lift: nothing rests on a turn whose desire isn't out)."""
  def lights(sim):
    sim.tp.oracle_ok = True
    return oracle_lights('green', 290.)(sim)
  sim, plans, s0 = walker_clears_at_the_models_red(lights)
  assert not sim.tp.always_drive and 300. - s0 > CURB_TURN_PULSE_M + CURB_GO_SLACK_M
  assert not any(p.desire == Desire.turnRight for p in plans), "the right's one pulse is 8 m out: none at the go 11 m out"
  handed = next(i for i, p in enumerate(plans) if p.controlPrompt == 'cantStartTakeControl')
  assert abs(handed * DT_MDL - NO_MOVE_S) <= 2 * DT_MDL and plans[handed].reason == NO_MOVE_REASON, handed * DT_MDL
  assert all(p.controlPrompt == 'none' for p in plans[:handed]), sorted({str(p.controlPrompt) for p in plans[:handed]})
  assert all(p.controlPrompt == 'cantStartTakeControl' and p.shouldStop for p in plans[handed:])
  assert all(go(p)[0] == 'none' for p in plans) and not any(lifted for _, _, lifted in sim.planned) and sim.s - s0 < 0.1
  # the control: the same go with the model pulling away (no planner holding it) is followed, nothing said, and the right's one
  # pulse goes rolling 8 m out
  sim = Sim(RIGHT_AT_A_SIGNAL, controls=[(290., 'signal')])
  sim.go_taps, sim.lights = False, lights
  sim.accel = model_stops_at(289.)
  while not (sim.v < 0.05 and sim.s > 280.):
    sim.step(engaged=True)
    assert sim.t < 80.
  stand(sim, 2., objects=WALKER)
  sim.accel = None
  after = [sim.step(WALKER_CLEAR, engaged=True)]
  while sim.s < 300. - CURB_FIRST_PULSE_M:
    after.append(sim.step(engaged=True))
    assert sim.t < 120.
  assert sim.tp.go_wait_s is None and not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in after)
  assert any(p.desire == Desire.turnRight for p in after) and not any(p.tooFastToTurn for p in after)


def test_a_shadow_camera_red_coming_mid_go_withdraws_it():
  """Certified on the oracle's green, then the camera's shadow reader reads red: the go is withdrawn that frame (light), the
  lift with it. Nothing brakes for the camera's read (shadow): only the lift is off."""
  sim = held_at_the_red()
  camera = {'state': None}
  p = go_from_the_hold(sim)
  base = sim.lights  # the oracle's, green now

  def lights(sim):
    if camera['state'] is not None:
      camera_reads(camera['state'], 300.)(sim)
    return base(sim)
  sim.lights = lights
  assert go(p)[0] == 'turnRelease'
  assert all(go(q)[0] == 'turnRelease' for q in stand(sim, 0.3))
  camera['state'] = 'red'
  q = stand(sim, DT_MDL)[0]
  assert go(q)[0] == 'none' and sim.tp.go_cert.end == 'light'


# --- skeptic G5: a far-side turn released in the junction is never lifted ---

def committed_left_stopped_by_a_walker(signal=False):
  """Skeptic G5 probe: the autonomous left committed (it can no longer stop short of the oncoming lanes on the comfortable
  curve), then a walker in the path stops it short of them, plannerd in the loop with the model holding. An oncoming car
  45 m up the road at 25 mph as the walker clears; the model holds for it. signal: the junction has a signal the sim's
  oracle reads green (an accepted source: the oncoming road has its green too). Returns (sim, the plans from the walker
  clearing, the car's s there, the oncoming car's s at sim.t)."""
  state = {'car': None}

  def cars(sim):
    return [] if state['car'] is None else [(state['car'] - V_25 * (sim.t - state['t']), 3.6)]

  def sim_cls(maneuvers):
    sim = Sim(maneuvers, controls=[(JUNCTION - HOLD_LINE_M - 2., 'signal')] if signal else ())
    if signal:
      sim.tp.oracle_ok, sim.lights = True, oracle_lights('green', JUNCTION - HOLD_LINE_M - 2.)
    return sim
  sim = left_sim(cars, sim_cls=sim_cls)
  plans = []
  while not (plans and str(plans[-1].unprotectedLeft.state) == 'committed'):
    plans += drive(sim, DT_MDL)
    assert sim.t < 90.
  assert go(plans[-1])[0] in ('unprotectedLeft', 'none')
  attach_planner(sim, holding)
  stand(sim, 2.5, objects=WALKER)  # stand pins the car where the walker stops it
  assert JUNCTION - sim.s - ONCOMING_ENTRY_M > 1.
  state['car'], state['t'] = sim.s + 45., sim.t
  s0 = sim.s
  return sim, [sim.step(WALKER_CLEAR, engaged=True)] + drive(sim, 6. - DT_MDL), s0, lambda: state['car'] - V_25 * (sim.t - state['t'])


def test_a_committed_far_side_turn_released_in_the_junction_is_never_lifted():
  """Skeptic G5 blocker 2: the walker cleared and turnRelease was certified in the junction (no 'room' check, no 'left'
  withdrawal: after the commit every oncoming hold drops, and the path corridor never sees a car in the oncoming lane), so
  plannerd lifted the model's hold for the oncoming car and moved the car 3.23 m, 0.9 m into the oncoming lane at 1.4 m/s,
  as the oncoming car reached it. A far-side turn released in the junction is never certified (why junction): the car stands
  where the walker stopped it, short of the oncoming lanes, and the stall's handback comes, as on 109215f. At a signal the
  oracle reads green too (the light certified, the oncoming road on its green)."""
  for signal in (False, True):
    sim, plans, s0, oncoming = committed_left_stopped_by_a_walker(signal)
    assert all(go(p)[0] == 'none' for p in plans), signal
    assert not any(lifted for _, _, lifted in sim.planned[-len(plans):]), signal
    assert sim.s - s0 < 0.1 and JUNCTION - sim.s > ONCOMING_ENTRY_M, signal
    assert oncoming() < sim.s  # the oncoming car went by
    stalled = next(i for i, p in enumerate(plans) if p.controlPrompt == 'stalledTakeControl')
    assert stalled * DT_MDL <= STALL_PROMPT_S + 0.2, signal


def test_a_curb_side_turn_released_in_the_junction_at_a_green_is_certified_and_a_far_side_one_isnt():
  """In the junction the go of a curb-side turn at a green (no oncoming lanes to cross, its cross traffic on its red) is
  certified; the far-side one (LEFT_AT_A_RED's, from a T's stem: its cross traffic crossed) isn't, and stands to the stall's
  handback."""
  def released_in_the_junction(maneuvers, desire, side):
    sim = Sim(maneuvers, controls=[(290., 'signal')])
    if side == 1:  # the right's go from its 10 m line: no desire, its one pulse rolling 8 m out (2026-11-06)
      curb_turn_from_a_red(sim, desire=desire)
    else:
      assert go(go_from_a_red(sim, desire=desire))[0] == 'turnRelease'  # from its line, on the oracle's green
    while sim.tp.desires.turned_deg < 45.:
      sim.step(engaged=True, yaw_rate=-0.3 * side if sim.v > 0.5 else 0.)
      assert sim.t < 200.
    walker_stops_the_car(sim)
    plans = stand(sim, DT_MDL, objects=WALKER_CLEAR)  # the walker walks clear: the hold lets go in the junction
    assert not plans[0].shouldStop and sim.tp.go_in_junction
    return sim, plans + stand(sim, STALL_PROMPT_S + 0.5)
  sim, plans = released_in_the_junction(RIGHT_AT_A_SIGNAL, Desire.turnRight, 1)
  assert go(plans[0])[0] == 'turnRelease'
  sim, plans = released_in_the_junction(LEFT_AT_A_RED, Desire.turnLeft, -1)
  assert all(go(p)[0] == 'none' for p in plans)
  assert any(p.controlPrompt == 'stalledTakeControl' for p in plans)


def test_a_release_of_a_turn_across_the_oncoming_road_is_never_certified():
  """A turn across the oncoming road goes only on the left's own go (unprotectedLeft, its gap certified): a release of
  TurnPilot's hold there, even from its line at a green, is never a turnRelease (why unprotected)."""
  sim = left_sim()
  while not (sim.trace and sim.v < 0.05 and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap'):
    drive(sim, DT_MDL)
    assert sim.t < 60.
  sim.tp.left_now.go = False
  sim.tp.map_controls.release_light = lambda distance: 'green'
  upcoming = sim.tp.follower.upcoming
  sim.tp.go_in_junction = False
  sim.tp.certify_go(sim.sm, sim.sm['carState'], True, [], None, SceneDecision(), False, LINE_D, upcoming, True)
  assert sim.tp.go_cert.cause == 'none' and not sim.tp.go_lifted
  # the same release of a T's stem left (no oncoming approach) is
  sim.tp.certify_go(sim.sm, sim.sm['carState'], True, [], None, SceneDecision(), False, LINE_D,
                    dataclasses.replace(upcoming, opposing='none'), True)
  assert sim.tp.go_cert.cause == 'turnRelease'


# --- skeptic G5 (residual): the unprotected left's lift, at a signal, on an accepted green only ---

def left_at_a_signal(controls, lights, model_fn=holding, seconds=40.):
  """LEFT_25 on the car's config: no accepted light source (the oracle off), the left's range set, an empty band (the oncoming
  road stopped at its own red), plannerd in the loop from the car's stand at its line with model_fn's model (holding for the
  red). Returns (sim, plans, the car's s at the stand)."""
  sim = Sim(LEFT_25, controls=controls)
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps = 'auto', TEST_RANGE_M, False
  sim.answers_prompt = True  # the driver signals the left when prompted: it goes only with a lamp lit (LEFT_GO_NEEDS_SIGNAL, tp/blinker)
  sim.tp.oracle_ok = False
  sim.lights = lights
  sim.raw = road_runs()
  plans, s_hold = [], None
  for _ in range(round(seconds / DT_MDL)):
    plans += run_objects(sim, DT_MDL)
    if s_hold is None and sim.v < 0.05 and str(plans[-1].unprotectedLeft.state) == 'waitGap':
      attach_planner(sim, model_fn)
      s_hold = sim.s
  assert s_hold is not None
  return sim, plans, s_hold


def held_then_goes(hold_s):
  """The model holds the car (G0's stand: as for a red it sees) hold_s from its first frame, then plans motion from rest (its
  green) at the left's own launch, 1.5 m/s^2 (left_launch)."""
  t0 = {}

  def model(sim):
    t0.setdefault('t', sim.t)
    return holding(sim) if sim.t - t0['t'] < hold_s else model_v2(*sim.model_lanes, accel=1.5, **sim.model_kw)
  return model


def test_a_signalized_lefts_go_on_the_cameras_shadow_green_is_never_lifted():
  """Skeptic G5 (residual): at a mapped signal the left goes on any source's green (turn_light), the camera's shadow reader's
  too, and certify_go checked the light only for turnRelease, so a shadow green certified unprotectedLeft and plannerd lifted
  the model's red-light hold: 60 frames lifted, the car rolled 3.01 m past its line at up to 1.5 m/s, then the handback; with
  a model that drives on once nudged it crossed the junction on the model's red, nothing said. The light net may only add a
  stop: the lift needs a green from an accepted source (release_light). Review 2026-11-07 (LS-2): the left's go then only let
  TurnPilot's own latch go, the model's hold stayed, and the no-move handback came 1 s on with the car held. Now the model's
  own hold there holds the left (LeftInput.model_holds): no go, no lift, no handback while the model plans no motion; its plan
  moving, the left goes on its gap, unlifted, and the car pulls away on the model."""
  for model_fn in (holding, launching):
    sim, plans, s_hold = left_at_a_signal([(JUNCTION - 8.5, 'signal')], camera_reads('green', s_head=JUNCTION + 8.), model_fn)
    held = plans[round(s_hold_t(plans) / DT_MDL) + 1:]
    assert not any(p.desire == Desire.turnLeft or str(p.unprotectedLeft.state) == 'go' for p in plans)
    assert not any(lifted for _, _, lifted in sim.planned) and all(go(p)[0] == 'none' for p in plans)
    assert sim.s - s_hold < 0.1 and sim.s < JUNCTION - ONCOMING_ENTRY_M
    assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
    assert held and all(p.reason == MODEL_HOLD_REASON['light'] for p in held[round(0.5 / DT_MDL):]), {p.reason for p in held}
  sim, plans, s_hold = left_at_a_signal([(JUNCTION - 8.5, 'signal')], camera_reads('green', s_head=JUNCTION + 8.), held_then_goes(20.),
                                        seconds=60.)
  assert any(p.desire == Desire.turnLeft for p in plans) and sim.s > JUNCTION
  assert not any(lifted for _, _, lifted in sim.planned) and all(go(p)[0] == 'none' for p in plans)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def s_hold_t(plans):
  """s into plans of the car's first stand at its line (left_at_a_signal's)."""
  return next(i for i, p in enumerate(plans) if p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap') * DT_MDL


@pytest.mark.parametrize('red_s', [0.5, 3., 10., 30.])
def test_a_signalized_left_waits_with_the_model_through_its_red_and_goes_on_its_green_never_handed_back(red_s):
  """Review 2026-11-07 (LS-2, its verifier's probe): a left at a mapped signal on the car's light config (no accepted source,
  no colour read), the model holding for a red red_s from the stand, then planning motion (its green). The go came at the
  red 2.3 s after the stand, unlifted, and cantStartTakeControl 1 s later held the car 9.2 m short until a disengage for any
  red of 3 s or more, the model's own green included. Now the left goes only once the model's plan moves, unlifted, and the
  car turns: never a take-control."""
  sim = signal_sim(oracle=False)
  while not (sim.trace and sim.trace[-1][2].shouldStop and sim.v < 0.05 and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap'):
    run_objects(sim, DT_MDL)
    assert sim.t < 60.
  attach_planner(sim, held_then_goes(red_s))
  plans = run_objects(sim, red_s + 20.)
  go_i = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert go_i * DT_MDL >= red_s - 1e-6  # never at the model's red
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert all(go(p)[0] == 'none' for p in plans) and sim.s > JUNCTION + 5.


def test_a_signalized_lefts_go_the_model_stops_following_goes_back_to_its_wait_never_handed_back():
  """LS-2's backstop: the model plans motion (the go goes, unlifted), then holds again before the car has moved (a red it now
  sees, a phantom): LEFT_NO_MOVE_S standing with no motion planned and the go goes back to a wait at the line, its turn
  pulse cleared, never cantStartTakeControl; its plan moving again, a fresh window and the go."""
  st = {'go_t': None}

  def model(sim):
    left = sim.tp.left_now
    if st['go_t'] is None and left is not None and left.state == 'go':
      st['go_t'] = sim.t
    if st['go_t'] is not None and sim.t < st['go_t'] + 15.:
      return holding(sim)
    return model_v2(*sim.model_lanes, accel=1.5, **sim.model_kw)  # the left's own launch (left_launch)
  sim = signal_sim(oracle=False)
  while not (sim.trace and sim.trace[-1][2].shouldStop and sim.v < 0.05 and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap'):
    run_objects(sim, DT_MDL)
    assert sim.t < 60.
  attach_planner(sim, model)
  t0 = sim.t
  plans = run_objects(sim, 40.)
  assert st['go_t'] is not None
  back = next((i for i, p in enumerate(plans) if t0 + (i + 1) * DT_MDL > st['go_t'] and str(p.unprotectedLeft.state) == 'waitGap'), None)
  assert back is not None and (t0 + back * DT_MDL) - st['go_t'] < LEFT_NO_MOVE_S + 0.6
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert all(p.desire != Desire.turnLeft for p in plans[back + 1:round((st['go_t'] + 15. - t0) / DT_MDL)])
  assert sim.s > JUNCTION + 5.  # gone again once the model's plan moves


def test_a_head_first_boxed_mid_lift_at_an_unsignalized_left_never_ends_its_lift():
  """HR-2 (review 2026-11-07) refuses the left's lift where the camera boxes a light with no signal mapped, decided at the
  go: a head first boxed mid-lift (a streetlight read as one as the car turns, a far-side head on the cross road coming into
  frame) never ends that lift at a junction with no signal, which would stand the car and hand it back (no_move); a red read
  there still withdraws it (test_a_shadow_camera_red_coming_mid_go_withdraws_it)."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import unmapped_camera_sim
  st = {'go_t': None}

  def read(t):
    return 'unknown' if st['go_t'] is not None and t >= st['go_t'] + 0.5 else None
  sim = unmapped_camera_sim(read)
  while not (sim.trace and sim.trace[-1][2].shouldStop and sim.v < 0.05 and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap'):
    drive(sim, DT_MDL)
    assert sim.t < 60.
  attach_planner(sim, launching)
  plans = []
  for _ in range(round(20. / DT_MDL)):
    plans += drive(sim, DT_MDL)
    if st['go_t'] is None and go(plans[-1])[0] == 'unprotectedLeft':
      st['go_t'] = sim.t
  assert st['go_t'] is not None
  assert sim.tp.go_cert.end in ('takeover', 'speed') and sim.s > JUNCTION, sim.tp.go_cert.end
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def test_a_left_with_the_cameras_red_at_an_unmapped_signal_is_never_gone_nor_lifted():
  """Skeptic G5 (residual, REQUIRED in the same change): no signal mapped, the camera's shadow reader reading the junction's
  red: the left went (turn_light 'none' before any light was read) and the lift rolled the car 3.01 m on the model's red. A red
  from any source at the junction keeps the left waiting and the lift off, mapped signal or not."""
  sim, plans, s_hold = left_at_a_signal([], camera_reads('red', s_head=JUNCTION + 8.))
  assert not any(str(p.unprotectedLeft.state) in ('go', 'committed') for p in plans)
  assert not any(lifted for _, _, lifted in sim.planned) and all(go(p)[0] == 'none' for p in plans)
  assert sim.s - s_hold < 0.1 and str(plans[-1].unprotectedLeft.state) == 'waitGap'


def test_the_lefts_lift_stays_on_an_accepted_green_or_where_no_signal_is():
  """The lift is unchanged where the light is an accepted source's green (the sim's oracle) or there is no signal at all."""
  for unsignalized in (True, False):
    sim = left_sim()
    if not unsignalized:
      sim = Sim(LEFT_25, controls=[(JUNCTION - 8.5, 'signal')])
      sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps = 'auto', TEST_RANGE_M, False
      sim.answers_prompt = True  # the driver signals the left when prompted: it goes only with a lamp lit (LEFT_GO_NEEDS_SIGNAL, tp/blinker)
      sim.tp.oracle_ok, sim.lights = True, oracle_lights('green', JUNCTION - 8.5)
      sim.raw = road_runs()
    while not (sim.trace and sim.v < 0.05 and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap'):
      drive(sim, DT_MDL)
      assert sim.t < 60.
    attach_planner(sim, holding)
    plans = []
    while not any(go(p)[0] != 'none' for p in plans[-1:]):
      plans += drive(sim, DT_MDL)
      assert sim.t < 90.
    assert go(plans[-1])[0] == 'unprotectedLeft'
