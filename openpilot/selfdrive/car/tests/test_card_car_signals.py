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


ZERO_VIN = "0" * 17
DONGLE = "0123456789abcdef"
CHASE_EPS = b'TeM3_E014p10_0.0.0 (35),E014.21.2'
CHECK_KEYS = {'pass', 'step', 'record', 'vin', 'fw', 'when', 'checkId', 'exterior'}


def chase_card(params):
  check = types.SimpleNamespace(result={'pass': True, 'step': 'done', 'record': {'onReason': 9, 'gear': 'P', 'onSeparates': False}})
  card, _ = fake_card(check, params=params)
  card.CP.carVin, card.CP.carFingerprint, card.CP.fingerprintSource = ZERO_VIN, 'TESLA_MODEL_3', structs.CarParams.FingerprintSource.fw
  card.CP.carFw = [structs.CarParams.CarFw(ecu='eps', address=0x730, subAddress=0, fwVersion=CHASE_EPS, brand='tesla')]
  return card


def test_check_result_keeps_the_identity_only_with_vinless_on():
  """TurnPilotCarSignalsVinless off (unset or 0): the check card writes is be3c0ed's. On: it also keeps this comma's
  DongleId and the platform the car's firmware fingerprints exactly, what a record without a VIN matches on."""
  for params in (FakeParams(DongleId=DONGLE), FakeParams(DongleId=DONGLE, TurnPilotCarSignalsVinless=False)):
    card = chase_card(params)
    Car.car_signals_tick(card)
    assert set(card.params['TurnPilotCarSignalsCheck']) == CHECK_KEYS
  card = chase_card(FakeParams(DongleId=DONGLE, TurnPilotCarSignalsVinless=True))
  Car.car_signals_tick(card)
  written = card.params['TurnPilotCarSignalsCheck']
  assert set(written) == CHECK_KEYS | {'dongleId', 'carFingerprint'}
  assert (written['vin'], written['dongleId'], written['carFingerprint']) == (ZERO_VIN, DONGLE, 'TESLA_MODEL_3')
