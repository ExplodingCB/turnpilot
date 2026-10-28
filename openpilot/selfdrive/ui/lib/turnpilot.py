"""What TurnPilot shows, from turnpilotPlan and carState. No raylib here: soundd uses it for the prompt's chime."""
from openpilot.cereal import custom, log
from openpilot.selfdrive.turnpilot.control_text import STOPPED_NOTE, controls_problem, hold_text, is_map_reason

State = custom.TurnpilotPlan.State
Desire = log.Desire

DESIRE_TEXT = {
  Desire.turnLeft: 'Turning left',
  Desire.turnRight: 'Turning right',
  Desire.keepLeft: 'Keeping left',
  Desire.keepRight: 'Keeping right',
}


def active_plan(sm, enabled: bool):
  """The turnpilotPlan reader when TurnPilot is on and turnpilotd is publishing, else None."""
  if not (enabled and sm.alive['turnpilotPlan'] and sm.valid['turnpilotPlan']):
    return None
  plan = sm['turnpilotPlan']
  return None if plan.state == State.disabled else plan


def signal_prompt(plan, car_state) -> str:
  """'left' or 'right' while the plan waits for the driver's blinker that way, else ''.

  turnpilotd keeps signalPrompt set after the driver signals, so the blinker check is here."""
  if plan is None or plan.state != State.laneChange:
    return ''
  side = str(plan.signalPrompt)
  if (side == 'left' and not car_state.leftBlinker) or (side == 'right' and not car_state.rightBlinker):
    return side
  return ''


def status_text(plan, engaged: bool) -> tuple[str, str]:
  """(title, detail) for the status line; ('', '') while TurnPilot is just following or has no route.

  It says what the car is doing, so it needs openpilot engaged, except in shadow mode, where the driver steers, and
  except when TurnPilot can't act at all (waiting for its detector, or handed back), which the driver needs to know."""
  if plan is not None and plan.state == State.unavailable:
    return 'TurnPilot unavailable', plan.reason
  if plan is None or not (engaged or plan.shadow):
    return '', ''
  detail = ''
  if plan.state == State.turning:
    modifier = plan.maneuverModifier
    side = 'left' if 'left' in modifier else 'right' if 'right' in modifier else ''
    title = DESIRE_TEXT.get(plan.desire) or (f'Turning {side}' if side else 'Turning')
  elif plan.state == State.laneChange:
    side = next((str(s) for s in (plan.laneChange, plan.signalPrompt) if str(s) != 'none'), '')
    title = f'Lane change {side}' if side else 'Lane change'
  elif plan.state == State.holding:
    mc = plan.mapControl
    held = hold_text(str(mc.kind), str(mc.action), str(mc.lightState), str(mc.lightSource))
    if held is not None:  # stopped at a stop sign or signal: and another hold in force with it (a pedestrian) is the detail
      title, detail = held[0], held[1] if is_map_reason(plan.reason) or not plan.reason else plan.reason
    else:
      title, detail = 'Holding', plan.reason
  elif plan.state == State.arrived:
    title = 'Arrived'
  elif plan.reason == STOPPED_NOTE:  # the model stands the car, nothing of TurnPilot's holding it: said before the handback
    title, detail = 'Car stopped', 'nothing ahead holding it'
  elif controls_problem(str(plan.mapControl.controlsStatus)):
    # no stop signs or signals from the map for this route: never shown as clear
    title, detail = 'Map stops unavailable', controls_problem(str(plan.mapControl.controlsStatus))
  else:
    return '', ''
  if plan.shadow:
    detail = f'shadow: {detail}' if detail else 'shadow'
  return title, detail


def map_status_text(status: dict | None, destination) -> tuple[str, str]:
  """(title, detail) for navd's TurnPilotMapStatus while a destination (NavDestination) is set: the reason there is no
  route, or the drive's map downloading. Shown whether or not openpilot is engaged, since it is about the route, not
  the car. A status about another destination (navd hasn't started on this one yet) is not this one's."""
  from openpilot.selfdrive.turnpilot.maps.status import same_destination
  if destination is None or not status:
    return '', ''
  if not same_destination(status, destination):
    return 'Checking', 'the mapped area'
  if status.get('state') == 'error':
    return 'No route', status.get('short') or status.get('message') or 'routing failed'
  if status.get('state') == 'downloading':
    progress = status.get('progress')
    return 'Downloading map', f'for this drive, {progress * 100:.0f}%' if isinstance(progress, (int, float)) else 'for this drive'
  return '', ''


def mapped_area_text(coverage_cache, position, override: str = '') -> str:
  """Which mapped area position ((latitude, longitude) or None) is in, for the TurnPilot settings screen."""
  from openpilot.selfdrive.turnpilot.maps.client import MapError
  if override:
    return 'local tiles (developer), no geofence'
  try:
    coverage = coverage_cache.get()
  except MapError:
    return 'saved mapped area is unreadable'
  if coverage is None:
    return 'no mapped area downloaded'
  if position is None:
    return 'car position unknown'  # the names would make it 3 lines, cut off in the button; the phone shows them
  region = coverage.region_at(position[1], position[0])
  return region.name if region else "Outside TurnPilot's mapped area"
