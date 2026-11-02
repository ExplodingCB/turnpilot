"""navd's map and route status in TurnPilotMapStatus, for the web app and the comma UI.

{"state": "idle" | "checking" | "downloading" | "ready" | "error", "message": for the driver, "short": for the
onroad status line, "code": MapError code on errors, "destination": [lon, lat] it is about, plus "region", "source"
("server" or "local"), "progress" (0-1 or null), "bytes", "total" while downloading, and "at" (epoch seconds).

A status is about one destination. Until navd starts on a new one, the status still names the old one, so readers
show it only for the current NavDestination (for_destination).
"""
from openpilot.selfdrive.turnpilot.maps.paths import wall_time

STATUS_PARAM = 'TurnPilotMapStatus'
STATES = ('idle', 'checking', 'downloading', 'ready', 'error')


class NavStatus:
  """Writes the status param only when it changes."""

  def __init__(self, params):
    self.params = params
    self.last: dict | None = None

  def set(self, state: str, message: str = '', short: str = '', code: str = '', **extra):
    assert state in STATES, state
    status = {'state': state, 'message': message, 'short': short or message, 'code': code, **extra}
    if status != self.last:
      self.params.put(STATUS_PARAM, {**status, 'at': wall_time()})
      self.last = status

  def error(self, e, **extra):
    self.set('error', e.message, e.short, e.code, **extra)


def read_status(params) -> dict | None:
  status = params.get(STATUS_PARAM)
  return status if isinstance(status, dict) and status.get('state') in STATES else None


def same_destination(status: dict, destination) -> bool:
  """Whether status is about destination, a NavDestination ({"latitude", "longitude"})."""
  where = status.get('destination')
  if not isinstance(destination, dict) or not isinstance(where, (list, tuple)) or len(where) != 2:
    return False
  try:
    return abs(float(where[0]) - float(destination['longitude'])) < 1e-9 and abs(float(where[1]) - float(destination['latitude'])) < 1e-9
  except (KeyError, TypeError, ValueError):
    return False


def for_destination(status: dict | None, destination, navd_running: bool = True) -> dict | None:
  """navd's status for the current destination: its own if it is about it, else 'checking' while navd runs (it hasn't
  started on it yet), and None with no destination or with navd not running. navd runs offroad too while TurnPilot is
  enabled (process_config.navigation), so the phone sees the route made while the car is parked."""
  if destination is None:
    return None
  if status is not None and same_destination(status, destination):
    return status
  if not navd_running:
    return None
  return {'state': 'checking', 'message': 'Checking the mapped area', 'short': 'Checking the mapped area', 'code': '',
          'destination': [destination.get('longitude'), destination.get('latitude')] if isinstance(destination, dict) else None}
