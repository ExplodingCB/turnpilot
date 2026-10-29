"""CarControl's blinkers (car_signals.car_blinkers, controlsd's): TurnPilot's turn signal on a car that works its own (car_signals.py), stock
openpilot's DesireHelper blinkers everywhere else."""
import time
import types

from opendbc.car import structs
from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import log, messaging
from openpilot.selfdrive.turnpilot.car_signals import car_blinkers as blinkers


class SM(dict):
  def __init__(self, plan=None, age=0.):
    super().__init__()
    self.recv_time, self.valid = {}, {}
    if plan is not None:
      self['turnpilotPlan'] = plan
      self.recv_time['turnpilotPlan'] = time.monotonic() - age
      self.valid['turnpilotPlan'] = True


def plan(side='left', status='ready', shadow=False, state='following'):
  msg = messaging.new_message('turnpilotPlan')
  p = msg.turnpilotPlan
  p.state, p.shadow = state, shadow
  p.turnSignal.side, p.turnSignal.status = side, status
  return msg.as_reader().turnpilotPlan


def model(state=log.LaneChangeState.off, direction=log.LaneChangeDirection.none):
  return types.SimpleNamespace(meta=types.SimpleNamespace(laneChangeState=state, laneChangeDirection=direction))


def cp(brand='tesla', flags=0):
  CP = structs.CarParams()
  CP.brand, CP.flags = brand, flags
  return CP


SIGNALS = int(TeslaFlags.CAR_SIGNALS)
PRE_LEFT = model(log.LaneChangeState.preLaneChange, log.LaneChangeDirection.left)


def test_car_signal_tesla_commands_turnpilots_side():
  assert blinkers(cp(flags=SIGNALS), True, model(), SM(plan('left'))) == (True, False)
  assert blinkers(cp(flags=SIGNALS), True, model(), SM(plan('right'))) == (False, True)
  # only while ready and engaged, never from a stale, shadow or disabled plan
  assert blinkers(cp(flags=SIGNALS), False, model(), SM(plan('left'))) == (False, False)
  assert blinkers(cp(flags=SIGNALS), True, model(), SM(plan('left', status='failed'))) == (False, False)
  assert blinkers(cp(flags=SIGNALS), True, model(), SM(plan('left'), age=1.)) == (False, False)
  assert blinkers(cp(flags=SIGNALS), True, model(), SM(plan('left', shadow=True))) == (False, False)
  assert blinkers(cp(flags=SIGNALS), True, model(), SM(plan('left', state='disabled'))) == (False, False)
  assert blinkers(cp(flags=SIGNALS), True, model(), SM()) == (False, False)


def test_car_signal_tesla_ignores_desire_helper():
  # the driver's own lane change is the driver's, signalled by the driver: no car blinker on top of theirs
  assert blinkers(cp(flags=SIGNALS), True, PRE_LEFT, SM(plan('none'))) == (False, False)


def test_other_cars_unchanged():
  for CP in (cp(), cp('hyundai'), cp(flags=SIGNALS | int(TeslaFlags.CAR_SIGNALS_CHECK))):
    assert blinkers(CP, True, PRE_LEFT, SM(plan('right'))) == (True, False)
    assert blinkers(CP, True, model(), SM(plan('right'))) == (False, False)
