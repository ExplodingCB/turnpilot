"""turnpilotd's turn signal in closed loop (car_signals.py, DESIGN phase4/blinker 11.3): the status from CarParams and the
port, the side it publishes for controlsd, the prompt, the driver overruling the car (a skip and a reroute), the side in
doubt, and the lane planner's lead gate on the car's own lamp."""
from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.car_signals import ARM_LAG_S, LEAD_LONG_M
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim

Status = custom.TurnpilotPlan.TurnSignal.Status
LaneChange = custom.TurnpilotPlan.LaneChange
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
