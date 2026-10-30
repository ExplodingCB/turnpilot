"""What the driver is shown about a map stop control (map_controls.py), for the device's status line and the phone.
Plain strings in and out, no cereal: the web app's status formatting can't depend on a messaging build."""

TAKE_CONTROL = 'take control'
# turnpilotd's plan.reason while the model stands the car with nothing of TurnPilot's holding it, before its handback
STOPPED_NOTE = "Car stopped, nothing ahead holding it"


# the stop-sign decider's actions on the plan's mapControl (stop_go.py through turnpilotd): said whatever the plan's state
DECIDER_ACTIONS = ('waitingCross', 'creeping', 'going', 'committed')
WAIT_TITLE = 'Waiting for cross traffic'


def _ledger_title(reason: str) -> str:
  """The stop-sign decider's wait or handback by its reason (STOP_SIGNS.md 4.6, 7.1): '' when it is none of the named ones."""
  r = reason.lower()
  if 'signal at junction' in r:
    return 'Signal at junction'
  if 'sunahead' in r or 'lowsun' in r or 'glare' in r:
    return 'Sun glare'
  if "can't see far enough" in r or 'view blocked' in r:
    return 'View blocked'
  return ''


def _detail(reason: str) -> str:
  """The decider's reason as the detail under its title: its 'Waiting: ' cut, the binding limit's word in brackets dropped."""
  r = reason[len('Waiting: '):] if reason.startswith('Waiting: ') else reason
  if r == WAIT_TITLE or r.lower() == 'signal at junction':
    return ''
  return r.split(' (')[0] if r.endswith(')') and "can't see far enough" in r else r


def hold_text(kind: str, action: str, light_state: str = 'unknown', light_source: str = 'none', reason: str = '') -> tuple[str, str] | None:
  """(title, detail) while a map control holds the car, or can't stop gently for one, or the stop-sign decider has it (waiting
  for cross traffic, creeping, going, committed); None otherwise. kind, action, light_state, light_source: turnpilotPlan.mapControl's,
  as strings; reason: the plan's (the decider's wait and handback are said by it: signal at junction, sun glare, view blocked)."""
  stop = kind in ('stop', 'yield')
  if stop and action == 'waitingCross':
    title = _ledger_title(reason) or WAIT_TITLE
    return title, '' if title == 'Signal at junction' else _detail(reason)
  if stop and action == 'creeping':
    if 'rolling up' in reason.lower():
      return 'Rolling up to stop sign', ''
    if 'our turn' in reason.lower():
      return 'Taking our turn', ''
    return 'Creeping for view', ''
  if stop and action == 'going':
    return 'Going', ''
  if stop and action == 'committed':
    return ('Stalled in junction', '') if 'stalled' in reason.lower() else ('Committed', '')
  if action in ('held', 'awaitingDriver'):
    if kind == 'stop':
      title = 'Stopped at stop sign'
    elif kind == 'yield':
      title = 'Stopped at yield'
    elif kind == 'unprotectedTurn':
      title = 'Stopped at unprotected turn'
    elif light_state == 'red':
      title = 'Red light'
    else:
      title = 'Stopped at signal'
    if stop and action == 'awaitingDriver':
      title = _ledger_title(reason) or title
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
                            'Unprotected', 'Waiting', 'Creeping for view', 'Rolling up', 'Going', 'Stopping short', 'Cross traffic clear',
                            'Stalled in junction', 'Turning right instead', 'Turning left instead', 'Going straight instead'))
