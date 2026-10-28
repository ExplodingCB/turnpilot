"""What the driver is shown about a map stop control (map_controls.py), for the device's status line and the phone.
Plain strings in and out, no cereal: the web app's status formatting can't depend on a messaging build."""

TAKE_CONTROL = 'take control'


def hold_text(kind: str, action: str, light_state: str = 'unknown', light_source: str = 'none') -> tuple[str, str] | None:
  """(title, detail) while a map control holds the car, or can't stop gently for one; None otherwise.
  kind, action, light_state, light_source: turnpilotPlan.mapControl's, as strings."""
  if action in ('held', 'awaitingDriver'):
    if kind == 'stop':
      title = 'Stopped at stop sign'
    elif kind == 'unprotectedTurn':
      title = 'Stopped at unprotected turn'
    elif light_state == 'red':
      title = 'Red light'
    else:
      title = 'Stopped at signal'
    detail = TAKE_CONTROL if action == 'awaitingDriver' else ('sim oracle' if light_source == 'simOracle' and kind == 'signal' else '')
    return title, detail
  if action == 'infeasible':
    return 'Cannot stop gently', 'take control'
  return None


def controls_problem(controls_status: str) -> str:
  """The map controls' status as something wrong to say, or '' when there's nothing to say (ok, or no route)."""
  return '' if controls_status in ('ok', 'none', '') else controls_status


def is_map_reason(reason: str) -> bool:
  """A plan.reason that is the map control's own (map_controls.py), not another hold in force with it."""
  return reason.startswith(('Stopped at', 'Stopping for the signal', 'Red light', 'Yellow light', 'Stop sign', 'Map stop', 'Yield',
                            'Unprotected'))
