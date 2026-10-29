"""card's part of TurnPilot's car signals (turnpilot/car_signals.py): Start in Drive passed to the check and cleared, the
check's result written, the verified record deleted on a lamp lit on the wrong side."""
import types

from opendbc.car import structs
from openpilot.selfdrive.car.card import Car

VIN = "5YJ3E1EA8KF000001"


class FakeParams(dict):
  def get(self, key):
    return super().get(key)

  def get_bool(self, key):
    return bool(super().get(key, False))

  def put(self, key, value, block=False):
    self[key] = value

  def remove(self, key):
    self.pop(key, None)


def fake_card(check=None, fault='none', params=None):
  port = types.SimpleNamespace(active=True, check=check, fault=fault)
  started = []
  CI = types.SimpleNamespace(CS=types.SimpleNamespace(car_signal=port), start_signal_check=lambda: started.append(True))
  CP = structs.CarParams()
  CP.carVin = VIN
  card = types.SimpleNamespace(CI=CI, CP=CP, params=params if params is not None else FakeParams(), signal_check_written=None,
                               signal_record_deleted=False)
  return card, started


def test_start_in_drive_is_passed_and_cleared():
  check = types.SimpleNamespace(result=None)
  card, started = fake_card(check, params=FakeParams(TurnPilotCarSignalsCheckNow=True))
  Car.car_signals_tick(card)
  assert started == [True] and 'TurnPilotCarSignalsCheckNow' not in card.params
  Car.car_signals_tick(card)
  assert started == [True]


def test_check_result_written_once():
  check = types.SimpleNamespace(result={'pass': True, 'step': 'done', 'record': {'onReason': 8, 'gear': 'P', 'onSeparates': False}})
  card, _ = fake_card(check)
  Car.car_signals_tick(card)
  written = card.params['TurnPilotCarSignalsCheck']
  assert written['vin'] == VIN and written['exterior'] == 'pending' and written['checkId']
  card.params['TurnPilotCarSignalsCheck'] = 'answered'
  Car.car_signals_tick(card)
  assert card.params['TurnPilotCarSignalsCheck'] == 'answered'


def test_wrong_side_deletes_the_record():
  card, _ = fake_card(fault='wrongSide', params=FakeParams(TurnPilotCarSignalsVerified={'vin': VIN}))
  Car.car_signals_tick(card)
  assert 'TurnPilotCarSignalsVerified' not in card.params and card.signal_record_deleted


def test_inactive_port_does_nothing():
  card, started = fake_card(params=FakeParams(TurnPilotCarSignalsCheckNow=True, TurnPilotCarSignalsVerified={'vin': VIN}))
  card.CI.CS.car_signal.active = False
  Car.car_signals_tick(card)
  assert not started and 'TurnPilotCarSignalsVerified' in card.params
