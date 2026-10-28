"""turnpilotd's certified go (go_override.GoCertifier; DESIGN phase 4 decision 1, G5): TurnPilot's go from a standstill is
asserted to plannerd (turnpilotPlan.goCertified) for its bounded lift only when nothing in the car's path within 15 m, no
lead and no hold of TurnPilot's is there, the driver isn't on a pedal, and TurnPilotGoOverride is on; it is withdrawn the
frame any of that stops holding, and it ends on its own at 3 s, 3 m/s or the model's takeover."""
from openpilot.cereal import messaging
from openpilot.common.params import UnknownKeyName
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.go_override import GO_CLEAR_M, GO_WINDOW_S, TAKEOVER_S
from openpilot.selfdrive.turnpilot.scene.tracker import RawBox, RawFrame
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import (LEFT_AT_A_RED, WALKER, FakeParams, Sim, go_from_the_hold, stand,
                                                                 walker_stops_the_car)
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import V_25, drive, left_sim
from openpilot.selfdrive.turnpilot.turnpilotd import GO_OVERRIDE_DEFAULT, NO_MOVE_S, go_override_for

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
