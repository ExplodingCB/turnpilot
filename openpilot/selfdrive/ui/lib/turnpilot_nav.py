"""What the nav view says (mici/onroad/nav_view.py): the maneuver, the time and distance left, what TurnPilot is doing or would do
now, and which onroad page the driver left the screen on. No raylib and no ui_state: the units come in as metric, so tests need
no window and no Params.

The camera view's status line (ui/lib/turnpilot.py: status_text) says only what the car is doing, engaged. The nav view is where
the driver looks to see what TurnPilot is thinking, so it always has a line: the same titles as if engaged, the plan's state
otherwise, and "not engaged" when the car isn't acting on it.
"""
from openpilot.cereal import custom
from openpilot.selfdrive.turnpilot.hooks import ABORT_TEXT, lane_change_text
from openpilot.selfdrive.turnpilot.web.status import STATE_LABELS, format_distance, format_duration, maneuver_text
from openpilot.selfdrive.ui.lib.turnpilot import map_status_text, status_text

State = custom.TurnpilotPlan.State

# the kinds of note under the state, which the view colors: a warning (orange), the plan's next move (green), a fact (grey)
WARN, PLAN, INFO = 'warn', 'plan', 'info'
MAX_NOTES = 2  # the view has room for two
STATE_LABELS_SHORT = {'approaching': 'Maneuver ahead'}  # the phone's label, which wraps on the comma

# turnpilotPlan.fault while the plan is disabled with TurnPilot on (selfdrived's "TurnPilot Unavailable" says plan.reason)
FAULT_TEXT = {
  'error': 'turnpilotd error, see Diagnostics',
  'noLongitudinal': "openpilot doesn't control speed on this car",
  'disengageOnAccelerator': 'needs Disengage on Accelerator on',
}
# turnpilotPlan.laneChangeWait: why a change the route wants isn't offered yet. Empty: nothing worth saying (not in its window
# yet, or confirming), so the line is just "planned"
WAIT_TEXT = {
  'paintRuleOff': 'lane changes off', 'inputsInvalid': 'inputs invalid', 'laneUnknown': "can't confirm lane",
  'pastWindow': 'too close to the turn', 'tooSlow': 'too slow', 'cooldown': 'just changed lanes', 'lineNotBroken': 'line not broken',
  'noLane': 'no lane there', 'laneParked': 'lane parked', 'blindspot': 'car in blind spot', 'carInLane': 'car in that lane',
  'afterStop': 'after the stop', 'unreachable': "can't reach the lane",
}
# turnpilotPlan.mapControl.kind -> its name; a signal is named by its light when one is read
CONTROL_NAMES = {'stop': 'Stop sign', 'yield': 'Yield', 'signal': 'Signal', 'unprotectedTurn': 'Unprotected turn'}
LIGHT_NAMES = {'red': 'Red light', 'yellow': 'Yellow light', 'green': 'Green light'}
# mapControl.action -> what the line adds. The holds (held, awaitingDriver) and the stop-sign decider's actions are the state's
# title already (status_text through hold_text), so they get no line of their own
CONTROL_ACTIONS = {'none': '', 'approaching': '', 'braking': 'stopping', 'released': 'going', 'proceedingOnYellow': 'going on yellow',
                   'overrun': 'passed it', 'infeasible': "can't stop gently"}


def sentence(text: str) -> str:
  """'Lane Change Right in 3' (an alert's title case) -> 'Lane change right in 3', as the rest of the view writes."""
  return text[:1].upper() + text[1:].lower()


def state_lines(plan, engaged: bool) -> tuple[str, str]:
  """(title, detail): what TurnPilot is doing, or would do if engaged, now. Never an empty title.

  plan: turnpilotPlan's reader while turnpilotd publishes it (alive and valid), else None. The view only shows with TurnPilotEnabled
  on, so a missing plan is turnpilotd down, and a disabled one is a fault."""
  if plan is None:
    return 'TurnPilot not running', 'no plan from turnpilotd'
  if plan.state == State.disabled:
    return 'TurnPilot off', FAULT_TEXT.get(str(plan.fault), '') or plan.reason
  title, detail = status_text(plan, True)  # as if engaged: the turn it would take, the stop it would hold at
  if not title:
    title, detail = STATE_LABELS_SHORT.get(str(plan.state)) or STATE_LABELS.get(str(plan.state), str(plan.state)), plan.reason
    if plan.shadow:
      detail = f'shadow: {detail}' if detail else 'shadow'
  if not engaged and not plan.shadow and plan.state != State.unavailable:
    detail = f'{detail}, not engaged' if detail else 'openpilot not engaged'
  return title, detail


def lane_change_line(plan) -> str:
  """The lane change in the plan: offered (its countdown, the prompt to signal, the wait for the blind spot, as the alert says it),
  just cancelled (and why, for the 2 s turnpilotd holds it), or wanted by the route but not offered yet (and why not)."""
  offer = str(plan.laneChangeOffer)
  if offer in ('left', 'right'):
    return sentence(lane_change_text(plan, offer))
  abort = str(plan.laneChangeAbort)
  if abort != 'none':
    return f'Lane change cancelled: {ABORT_TEXT.get(abort, abort).lower()}'
  want = int(plan.laneChangeWant)
  if not want:
    return ''
  side = 'left' if want > 0 else 'right'
  head = f'{abs(want)} lane changes {side} planned' if abs(want) > 1 else f'Lane change {side} planned'
  wait = WAIT_TEXT.get(str(plan.laneChangeWait), '')
  return f'{head}: {wait}' if wait else head


def control_line(mc, metric: bool) -> str:
  """The map stop control ahead (turnpilotPlan.mapControl): 'Stop sign in 300 ft', 'Red light in 80 m: stopping'. '' with none,
  and while it holds the car, which the title says ('Stopped at stop sign')."""
  kind, action = str(mc.kind), str(mc.action)
  if kind not in CONTROL_NAMES or action not in CONTROL_ACTIONS:
    return ''
  name = LIGHT_NAMES.get(str(mc.lightState), CONTROL_NAMES[kind]) if kind == 'signal' else CONTROL_NAMES[kind]
  if action in ('none', 'approaching', 'braking', 'infeasible') and mc.distance > 0:
    name = f'{name} in {format_distance(mc.distance, metric)}'
  word = CONTROL_ACTIONS[action]
  return f'{name}: {word}' if word else name


def calibration_text(status: str, percent: int) -> str:
  """extrinsicsCalibration's calStatus and calPerc as a note, '' once calibrated. TurnPilot's turns and stops wait on it."""
  if status == 'uncalibrated':
    return f'Calibrating {percent}%'
  if status == 'recalibrating':
    return f'Recalibrating {percent}%'
  return 'Calibration invalid: check the mount' if status == 'invalid' else ''


def notes(plan, *, gps_ok: bool, routing: bool, calibration: str, metric: bool) -> list[tuple[str, str]]:
  """[(text, WARN | PLAN | INFO)], most important first, at most MAX_NOTES: GPS, calibration, a skipped turn, the lane change, the
  map control ahead. routing: navInstruction alive and valid; navd keeps its last position through a GPS gap, so with a route and no
  fix it is holding the route on that. calibration: calibration_text's."""
  out = []
  if not gps_ok:
    out.append(('Holding route, GPS lost' if routing else 'No GPS fix', WARN))
  if calibration:
    out.append((calibration, WARN))
  if plan is not None and plan.state != State.disabled:
    if plan.tooFastToTurn:
      out.append(('Skipping the turn: too fast, rerouting', WARN))
    lane = lane_change_line(plan)
    if lane:
      out.append((lane, WARN if lane.startswith('Lane change cancelled') else PLAN))
    control = control_line(plan.mapControl, metric)
    if control:
      out.append((control, INFO))
  return out[:MAX_NOTES]


def maneuver_lines(instruction, metric: bool) -> tuple[str, str]:
  """(distance to the maneuver, the road it goes onto): ('600 ft', 'Example Ave'), as the camera view's nav panel, the arrow saying
  which way. The phone's whole sentence ('Arrive at Example') for the arrival, and for a maneuver with no road named."""
  road = instruction.maneuverPrimaryText
  if instruction.maneuverType == 'arrive' or not road:
    road = maneuver_text(instruction.maneuverType, instruction.maneuverModifier, road)
  return format_distance(max(0., instruction.maneuverDistance), metric), road


def remaining_text(instruction, metric: bool) -> str:
  """'10 min / 3.2 mi' left, as the camera view's nav panel writes it."""
  return f'{format_duration(max(0., instruction.timeRemaining))} / {format_distance(max(0., instruction.distanceRemaining), metric)}'


def route_status(destination, map_status: dict | None) -> tuple[str, str]:
  """(title, detail) in the maneuver's place while navd has no route: no destination, why there's no route (navd's
  TurnPilotMapStatus, as the camera view's status line says it), or the wait for one."""
  if destination is None:
    return 'No destination', 'set one from your phone'
  title, detail = map_status_text(map_status, destination)
  return (title, detail) if title else ('Waiting for route', '')


def resting_page(viewport_x: float, pages: dict[str, float], settled: bool) -> str | None:
  """The page the main scroller rests on: the one whose left edge is at the viewport's (the scroller snaps whole pages), while
  nothing moves it (settled: no swipe, fling or scroll_to under way, so a page passed on the way somewhere doesn't count)."""
  if not settled:
    return None
  return next((name for name, x in pages.items() if abs(x - viewport_x) < 1.), None)


class OnroadPage:
  """Which onroad page the driver chose, the camera view or the nav view, for main.py's scrolls back to onroad (the drive's
  start, leaving a standstill, the 5 s interactive timeout): without it each of those would swipe the nav view away."""
  CAMERA, NAV = 'camera', 'nav'

  def __init__(self):
    self.page = self.CAMERA

  def see(self, page: str | None) -> None:
    """resting_page's answer, onroad: a rest on either onroad page is the choice; home, settings and passing pages are not."""
    if page in (self.CAMERA, self.NAV):
      self.page = page
