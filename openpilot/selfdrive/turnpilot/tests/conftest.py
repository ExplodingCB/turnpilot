"""Shared fixtures for the TurnPilot tests. Nothing here is autouse: every test runs the settings the build ships unless
it asks for a fixture by name."""
import pytest

from openpilot.selfdrive.turnpilot import unprotected_left as ul


@pytest.fixture
def left_window_from_its_stand(monkeypatch):
  """Opt-in, for a scenario that is about the stand-and-judge path itself: the left's window starts over at its hold
  (WINDOW_FROM_APPROACH False, the public build's rule), so the car stands a whole CLEAR_WINDOW_S at its line before a go.
  The build ships it True. Never for a scenario about a hard rail (a seen car, a red, a person holding the left)."""
  monkeypatch.setattr(ul, 'WINDOW_FROM_APPROACH', False)
