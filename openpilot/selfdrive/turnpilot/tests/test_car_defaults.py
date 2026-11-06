"""The one-time defaults TurnPilot's web app writes at start (2026-11-02): the provisional left range and the queued lamp
check, each once per comma, never over a value someone set."""
import pytest

from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.turnpilot.car_signals import ensure_signal_check
from openpilot.selfdrive.turnpilot.maps.defaults import CAR_LEFT_RANGE_M, ensure_left_range


@pytest.fixture(autouse=True)
def own_params():
  with OpenpilotPrefix():
    yield


def fresh():
  return Params()


def test_an_unmeasured_left_range_becomes_the_provisional_one_once():
  p = fresh()
  p.put("TurnPilotLeftRangeM", 0., block=True)
  assert ensure_left_range(p) and p.get("TurnPilotLeftRangeM") == CAR_LEFT_RANGE_M == 210.
  p.put("TurnPilotLeftRangeM", 0., block=True)  # set back on purpose: left alone
  assert not ensure_left_range(p) and p.get("TurnPilotLeftRangeM") == 0.


def test_a_comma_on_the_earlier_150_m_default_takes_210():
  p = fresh()
  p.put("TurnPilotLeftRangeM", 150., block=True)
  p.put_bool("TurnPilotLeftRangeProvisional", True, block=True)
  assert ensure_left_range(p) and p.get("TurnPilotLeftRangeM") == 210.
  assert not ensure_left_range(p)


def test_a_measured_left_range_is_never_touched():
  p = fresh()
  p.put("TurnPilotLeftRangeM", 95., block=True)
  assert not ensure_left_range(p) and p.get("TurnPilotLeftRangeM") == 95.


def test_the_lamp_check_is_queued_once_with_the_vinless_match():
  p = fresh()
  p.put_bool("TurnPilotCarSignalsForceOn", False, block=True)
  assert ensure_signal_check(p)
  assert p.get("TurnPilotCarSignals") == "check" and p.get_bool("TurnPilotCarSignalsVinless")
  p.put("TurnPilotCarSignals", "off", block=True)  # card set it back after the check, or the driver turned it off: never queued again
  assert not ensure_signal_check(p) and p.get("TurnPilotCarSignals") == "off"


def test_a_verified_car_is_not_checked_again():
  p = fresh()
  p.put_bool("TurnPilotCarSignalsForceOn", False, block=True)
  p.put("TurnPilotCarSignalsVerified", {"vin": "x"}, block=True)
  assert not ensure_signal_check(p) and p.get("TurnPilotCarSignals") in (None, "off")


def test_car_signals_are_on_by_default_with_no_check_or_record():
  # Kellen, 2026-11-06: "never disable turn signals for now"
  from openpilot.selfdrive.turnpilot.car_signals import resolver
  p = fresh()
  assert not ensure_signal_check(p)  # nothing queued
  for mode in (None, "off", "on"):
    if mode is not None:
      p.put("TurnPilotCarSignals", mode, block=True)
    assert resolver(p)("00000000000000000", []).mode == "on"
  p.put("TurnPilotCarSignals", "check", block=True)
  assert resolver(p)("00000000000000000", []).mode == "check"  # asked for: the check still runs
  p.put_bool("TurnPilotCarSignalsForceOn", False, block=True)
  p.put("TurnPilotCarSignals", "off", block=True)
  assert resolver(p)("00000000000000000", []).mode == "off"
