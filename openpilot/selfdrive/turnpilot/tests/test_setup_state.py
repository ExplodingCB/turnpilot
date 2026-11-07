"""setup_state's car-signal words under TurnPilotCarSignalsForceOn (1 by default since 2026-11-06, Kellen: "never disable turn
signals for now they should always be enabled for this test"): the phone and the comma screen say what card's resolver
(car_signals.resolver) gives the next drive, never TurnPilotCarSignals alone, which reads off on every comma at its defaults
while the car signals."""
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.turnpilot import setup_state as ss
from openpilot.selfdrive.turnpilot.car_signals import resolver


class BlockingParams(Params):
  def put(self, key, dat, block=True):
    super().put(key, dat, block=True)

  def put_bool(self, key, val, block=True):
    super().put_bool(key, val, block=True)


ZERO_VIN = "0" * 17


class TestCarSignalState(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.params = BlockingParams()

  def resolved(self) -> str:
    """card's own answer at the next start, for a Tesla's zero VIN with no firmware (no record can match it)."""
    return resolver(self.params)(ZERO_VIN, []).mode

  def test_a_fresh_install_says_on_as_the_resolver_does(self):
    assert self.params.get('TurnPilotCarSignals') is None and self.params.get('TurnPilotCarSignalsForceOn') is None
    assert ss.signals_forced(self.params)
    assert ss.car_signal_state(self.params) == ('on', "on for every drive, with no lamp check")
    assert self.resolved() == 'on'

  def test_mode_off_or_on_reads_on_while_forced(self):
    for mode in ('off', 'on'):
      self.params.put('TurnPilotCarSignals', mode)
      assert ss.car_signal_state(self.params)[0] == 'on', mode
      assert self.resolved() == 'on', mode

  def test_a_queued_check_says_what_follows_it(self):
    self.params.put('TurnPilotCarSignals', 'check')
    state, why = ss.car_signal_state(self.params)
    assert state == 'check' and "lamp check runs in Park" in why and "then on for every drive, with no check" in why
    self.params.put_bool('TurnPilotCarSignalsForceOn', False)
    self.params.put('TurnPilotCarSignals', 'check')
    assert ss.car_signal_state(self.params)[1].endswith("then on once you confirm the lamps flashed")

  def test_with_force_on_0_the_mode_and_the_record_decide(self):
    self.params.put_bool('TurnPilotCarSignalsForceOn', False)
    assert not ss.signals_forced(self.params)
    assert ss.car_signal_state(self.params) == ('off', "off: you signal when TurnPilot asks")
    assert self.resolved() == 'off'
    self.params.put('TurnPilotCarSignals', 'on')
    assert ss.car_signal_state(self.params)[0] == 'off'  # no record: card resolves off
    assert self.resolved() == 'off'
    self.params.put('TurnPilotCarSignalsVerified', {'vin': ZERO_VIN, 'fw': []})
    assert ss.car_signal_state(self.params) == ('on', "on, from the lamp check you confirmed")

  def test_a_failed_check_leaves_the_car_signalling_while_forced(self):
    """signals_off (any failure in the guided check) writes TurnPilotCarSignals off: under TurnPilotCarSignalsForceOn the car
    signals all the same, which the 'failed' card said was off until 2026-11-06."""
    ss.start_signal_check(self.params)
    ss.signals_off(self.params)
    assert self.params.get('TurnPilotCarSignals') == 'off' and not self.params.get_bool('TurnPilotCarSignalsVinless')
    assert ss.car_signal_state(self.params)[0] == 'on' and self.resolved() == 'on'
    self.params.put_bool('TurnPilotCarSignalsForceOn', False)
    assert ss.car_signal_state(self.params)[0] == 'off' and self.resolved() == 'off'
