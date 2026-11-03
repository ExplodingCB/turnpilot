"""TurnPilot's faster GPS start in pigeond, without a receiver: the time and last position go to the receiver as soon as the
clock is valid, and the backup is saved at the controlled stop."""
from unittest import mock

import pytest

from openpilot.system.ubloxd import pigeond


class FakePigeon:
  def __init__(self):
    self.sent = []

  def send(self, dat):
    self.sent.append(bytes(dat))

  def send_with_ack(self, dat, ack=None, nack=None):
    self.sent.append(bytes(dat))

  def wait_for_ack(self, ack=None, nack=None, timeout=0.5):
    return True


def test_time_and_position_wait_for_a_valid_clock():
  p = FakePigeon()
  with mock.patch.object(pigeond, 'system_time_valid', return_value=False):
    assert not pigeond.send_time_and_position(p)
  assert p.sent == []
  with mock.patch.object(pigeond, 'system_time_valid', return_value=True), mock.patch.object(pigeond, 'send_position') as pos:
    assert pigeond.send_time_and_position(p)
  assert p.sent and p.sent[0][2:4] == b"\x13\x40" and pos.called  # UBX-MGA-INI-TIME_UTC, then the position


def test_the_stop_saves_the_backup_after_stopping_gnss_and_before_power_off():
  p, order = FakePigeon(), []
  with mock.patch.object(pigeond, 'set_power', side_effect=lambda on: order.append(('power', on))), \
       mock.patch.object(pigeond.time, 'sleep'), pytest.raises(SystemExit):
    p.send = lambda dat: order.append(bytes(dat)[2:4])
    pigeond.deinitialize_and_exit(p)
  assert order == [b"\x06\x04", b"\x09\x14", ('power', False)]  # CFG-RST GNSS stop, UPD-SOS create, power off
