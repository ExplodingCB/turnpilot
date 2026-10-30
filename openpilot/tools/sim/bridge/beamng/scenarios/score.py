#!/usr/bin/env python3
"""Scenario scoring from recorded rows, one row per 50 ms of a run (run.py records them from msgq and the game).

A row is a dict:
  t, wall            s since the run started, and wall-clock time
  x, y, heading, v   the ego's camera mount in BeamNG world meters, heading rad CCW from east, wheel speed m/s
  enabled            selfdriveState.enabled; alert, alert_text: its alertType and alertText1
  steer, gas, brake  carState steeringPressed, gasPressed, brakePressed
  state, reason      turnpilotPlan.state and .reason; lc, prompt: its laneChange and signalPrompt
  control_prompt     turnpilotPlan.controlPrompt ('none', 'waitingForGap', 'leftGo', 'stalledTakeControl', ...)
  left_state         turnpilotPlan.unprotectedLeft.state ('off', 'waitGap', 'go', 'undecided', ...); left_treq, left_rreq its
                     tReq and rReq; turn_missed turnpilotPlan.turnMissed
  onc                [[m to the junction, m it is ahead of the car along its own heading], ...] of each oncoming scenario car
                     (None before they recorded it)
  lc_wait, lc_want   turnpilotPlan.laneChangeWait (why a change the route wants isn't offered) and laneChangeWant
  desire             turnpilotPlan.desire (a log.Desire index); map_kind: its mapControl.kind ('none', 'signal', ...)
  lc_state           modelV2.meta.laneChangeState
  nav_valid, nav_type, nav_mod, nav_dist, nav_remaining   navInstruction; route: an id of the current navRoute
  dest_dist          m from the ego to the destination (None without one)
  ped_gap, car_gap   m from the camera mount to the nearest mannequin / scenario car (None when there is none)
  pacer_dx           m the pacer car is ahead of the ego along its heading (None without one)
  j_dist, car_j_dist m from the ego, and from the nearest oncoming car, to the scenario's junction (None without one)
  j_ahead            m the junction is ahead of the ego along its heading (negative: behind it; None without one)
  blinker            the driver's stalk: '', 'left' or 'right'; lamp: carState's blinkers, what openpilot read of it
  handback           turnpilotPlan.handback ('none', 'detectorStopped', ...); control_alert its controlAlert ('none',
                     'hardBrake', ...); too_fast its tooFastToTurn
  forced_light       the state the runner forced the scenario junction's own signal to ('red', 'green'; None: not forced)
  lights             [[instance, state, m to its line], ...] of the light oracle's turnpilotLights when fresh (None otherwise)

A scenario's expectations (EXPECTS: catalog Entry.expect, or the runner's --expect and the options that imply one) are
what its shot exists to show: a handback, a skip, a missed turn, the driver's gas press, hard braking. Each is required of
the run (not seen: a failure), and explains only the generic failure it causes (the disengagement after the handback, the
press itself, the route not finished), so a run's verdict means what its shot shows.

Rescore a saved result: python -m openpilot.tools.sim.bridge.beamng.scenarios.score result.json
Score a recorded drive: python -m openpilot.tools.sim.bridge.beamng.scenarios.score --scenario route_demo rlog...
"""
import argparse
import json

import numpy as np

from openpilot.selfdrive.turnpilot.lane_planner import LC_BLOCK_X_M
from openpilot.selfdrive.turnpilot.route_follower import KEEP, TURN, classify

STOPPED_V = 0.3        # m/s: standing
RESUMED_V = 2.0        # m/s: moving again after a hold
RESUME_WITHIN_S = 15.  # a hold counts as resumed if the car is moving again this soon after it ends
HOLD_MERGE_S = 1.0     # holds with the same reason this close together are one
ATTEMPT_M = 30.        # reaching this close to a maneuver on the route is an attempt
STEP_CHANGE_M = 2.     # the distance left after the current maneuver changing by more than this is a new step
REROUTE_WINDOW_S = 1.5 # a new navRoute this close to a step change means navd rerouted (it runs at 1 Hz)
CONFIRM_M = 20.        # after navd moves on, the car stays on the route this far along the next step: the step was taken
TURN_TAKEN_DEG = 45.   # a turn is taken only if the car's heading turned this far its way (the junctions are ~90 degrees)
ARRIVE_M = 15.         # route_follower.ARRIVE_DIST_M, measured to the destination in the game
CAUSE_WINDOW_S = 5.    # a disengagement's cause is the last alert this soon before it
COLLISION_GAP_M = 2.5  # camera mount to object: about where the bumper touches (the mount sits ~2 m behind it)
PED_STOP_GAP = (3., 8.)
CAR_MIN_GAP_M = 4.     # camera mount to the other car's centre
# the camera mount (the car's centreline) past this far left of the road's divider (negative: short of it): the car is in the
# oncoming lane. Half the car's width (0.95 m) less the road model's error (clean runs keep the centre 1.45-1.65 m short of
# it, p99): straddling the divider with the left wheels over it counts, not only a car wholly across
WRONG_SIDE_PAST_M = -0.5
WRONG_SIDE_MIN_M = 3.  # driving this far in the oncoming lane, outside every junction box, fails the run
ON_ROAD_M = 1.5        # within the road's half width plus this of a link's centreline: on that road
ALIGNED_DEG = 35.      # the car's heading within this of a link's direction (either way): driving along it
LINE_BOX_MARGIN_M = 2. # a junction's box reaches this far past its mapped stop line, for a car heading into the junction
INTO_JUNCTION_DEG = 60.  # ...within this of the bearing to it
NEAR_JUNCTION_M = 35.  # timing.GO_NEAR_M: a stop and the go within this of the junction are its own
PRESENT_CAR_M = 30.    # an oncoming car this close to the junction while the car is before it, at it: the conflict is presented
# The autonomous unprotected left (DESIGN phase 4 G4, G6; CRITIQUE REQUIRED 5): no gas tap anywhere (any press fails a run);
# the post-encroachment time with each oncoming car through the junction's conflict zone under PET_MIN_S fails; a go with an
# oncoming car under the plan's tReq from the zone at its own speed is a false go (fails); a gap of tReq or more offered
# while the car waited at its line without going is a missed gap (counted, not failed: the velocity-free rule's 1.3x margin
# and 2.5 s window wait out gaps a little longer than tReq by design); undecided passes only if no such gap was offered with
# the car at its line, before or after it (and a tReq is known); a stall or an unmoving go (stalledTakeControl,
# cantStartTakeControl) and a missed turn fail
PET_MIN_S = 2.
CONFLICT_M = 8.        # the conflict zone: within this of the junction's point
APPROACH_SPEED_S = 0.5  # an oncoming car's speed at a go: from its distance this long before
LEFT_STALLS = ('stalledTakeControl', 'cantStartTakeControl')
TURN_DESIRES = (1, 2)  # log.Desire.turnLeft, turnRight
# expectations (module docstring): 'handback:<kind>' (a turnpilotPlan.handback, or a take-control controlPrompt),
# 'gasDisengage', 'skip' (tooFastToTurn, or the left skipped at route time), 'turnMissed', 'hardBrake' (controlAlert)
EXPECTS = ('handback', 'gasDisengage', 'skip', 'turnMissed', 'hardBrake')
# the disengagement within this of the first row showing the gas is the press's: the shot's 0.2 s (carState.gasPressed to
# disengaged, in the rlog), plus the rows' 50 ms sampling of two messages that may each land a row late
GAS_DISENGAGE_S = 0.3
RED_STOP_SCENARIOS = ('red_right', 'red_right_g62')  # a forced red they must stand at and not pass (red_straight: a limit)
RED_PAST_LINE_M = 1.  # the camera mount this far past the junction's stop line (the bumper ~3 m past it): into the junction
RED_NEAR_M = 30.      # a stand within this before the stop line is at the red


def msg_fields(ss, cs, tp, nav, model) -> dict:
  """The message part of a row from selfdriveState, carState, turnpilotPlan (None before the first), navInstruction (None
  while navd has no valid route) and modelV2 readers."""
  return {
    'enabled': bool(ss.enabled), 'alert': str(ss.alertType), 'alert_text': str(ss.alertText1),
    'steer': bool(cs.steeringPressed), 'gas': bool(cs.gasPressed), 'brake': bool(cs.brakePressed),
    'state': '' if tp is None else str(tp.state), 'reason': '' if tp is None else str(tp.reason),
    'lc': '' if tp is None else str(tp.laneChange), 'prompt': '' if tp is None else str(tp.signalPrompt),
    'control_prompt': '' if tp is None else str(tp.controlPrompt),
    'left_state': 'off' if tp is None else str(tp.unprotectedLeft.state),
    'left_treq': 0. if tp is None else round(float(tp.unprotectedLeft.tReq), 2),
    'left_rreq': 0. if tp is None else round(float(tp.unprotectedLeft.rReq), 1),
    'turn_missed': False if tp is None else bool(tp.turnMissed),
    'handback': 'none' if tp is None else str(tp.handback), 'control_alert': 'none' if tp is None else str(tp.controlAlert),
    'too_fast': False if tp is None else bool(tp.tooFastToTurn),
    'lamp': 'left' if cs.leftBlinker else 'right' if cs.rightBlinker else '',
    'lc_wait': '' if tp is None else str(tp.laneChangeWait), 'lc_want': 0 if tp is None else int(tp.laneChangeWant),
    'desire': 0 if tp is None else int(tp.desire), 'map_kind': '' if tp is None else str(tp.mapControl.kind),
    'lc_state': str(model.meta.laneChangeState),
    'nav_valid': nav is not None, 'nav_type': '' if nav is None else str(nav.maneuverType),
    'nav_mod': '' if nav is None else str(nav.maneuverModifier),
    'nav_dist': 0. if nav is None else round(float(nav.maneuverDistance), 1),
    'nav_remaining': 0. if nav is None else round(float(nav.distanceRemaining), 1),
  }


def route_id(coords) -> int:
  """An id for a navRoute: navd resends the same route every 10 s, a reroute changes it."""
  n = len(coords)
  return hash((n, round(coords[0].latitude, 6), round(coords[n - 1].longitude, 6))) if n else 0


LOG_SERVICES = ('selfdriveState', 'carState', 'turnpilotPlan', 'navInstruction', 'navRoute', 'modelV2')


def rows_from_log(msgs) -> list[dict]:
  """Rows from a recorded message stream (log events in time order, e.g. an rlog's LogReader): one per turnpilotPlan,
  with the latest of the others. A log has no game, so positions, gaps and the destination distance are None."""
  latest: dict = {}
  rows: list[dict] = []
  t0, route = None, 0
  for m in msgs:
    w = m.which()
    if w not in LOG_SERVICES:
      continue
    latest[w] = m
    if w == 'navRoute':
      route = route_id(m.navRoute.coordinates)
    if w != 'turnpilotPlan' or not all(s in latest for s in ('selfdriveState', 'carState', 'modelV2')):
      continue
    t = m.logMonoTime * 1e-9
    t0 = t if t0 is None else t0
    cs = latest['carState'].carState
    nav = latest.get('navInstruction')
    rows.append({'t': round(t - t0, 3), 'wall': None, 'x': None, 'y': None, 'heading': None, 'v': round(cs.vEgo, 2), 'route': route,
                 'dest_dist': None, 'blinker': 'left' if cs.leftBlinker else 'right' if cs.rightBlinker else '',
                 'ped_gap': None, 'car_gap': None, 'pacer_dx': None, 'j_dist': None, 'car_j_dist': None, 'onc': None,
                 **msg_fields(latest['selfdriveState'].selfdriveState, cs, m.turnpilotPlan,
                              nav.navInstruction if nav is not None and nav.valid else None, latest['modelV2'].modelV2)})
  return rows


def disengagements(rows) -> list[dict]:
  """Every enabled to disabled transition, with the driver override or the last alert before it as the cause."""
  out, engaged, last_alert = [], False, None
  for r in rows:
    if r.get('alert'):
      last_alert = r
    if r['enabled']:
      engaged = True
      continue
    if not engaged:
      continue
    engaged = False
    override = next((k for k in ('steer', 'gas', 'brake') if r.get(k)), None)
    if override:
      cause = f"driver {override}"
    elif last_alert is not None and r['t'] - last_alert['t'] <= CAUSE_WINDOW_S:
      cause = f"{last_alert['alert']}: {last_alert.get('alert_text', '')}".rstrip(': ')
    else:
      cause = "unknown"
    out.append({'t': r['t'], 'cause': cause, 'x': r.get('x'), 'y': r.get('y'), 'v': r.get('v')})
  return out


def holds(rows) -> list[dict]:
  """Spans of turnpilotPlan.state holding, with reason, duration, the slowest speed in them and whether the car moved on."""
  spans: list[dict] = []
  for r in rows:
    if r.get('state') != 'holding':
      continue
    cur = spans[-1] if spans else None
    if cur is not None and r['t'] - cur['end'] <= HOLD_MERGE_S and r.get('reason') == cur['reason']:
      cur['end'] = r['t']
      cur['min_v'] = min(cur['min_v'], r['v'])
    else:
      spans.append({'reason': r.get('reason', ''), 'start': r['t'], 'end': r['t'], 'min_v': r['v']})
  for h in spans:
    after = [r for r in rows if h['end'] < r['t'] <= h['end'] + RESUME_WITHIN_S]
    h['duration_s'] = round(h['end'] - h['start'], 2)
    h['stopped'] = h['min_v'] < STOPPED_V
    h['resumed'] = any(r['v'] > RESUMED_V for r in after)
  return spans


def gas_taps(rows) -> list[dict]:
  """Each press of the gas while engaged (an override: the driver's go at a hold), with the go TurnPilot asked for and
  the hold's reason just before it, how long the car had stood, and how far it was from the scenario's junction. Pressing
  it to engage is not one."""
  out, stopped_t = [], None
  for a, b in zip(rows, rows[1:], strict=False):
    stopped_t = (a['t'] if stopped_t is None else stopped_t) if a['v'] < STOPPED_V else None
    if b['enabled'] and b.get('gas') and not (a['enabled'] and a.get('gas')):
      out.append({'t': b['t'], 'prompt': a.get('control_prompt', ''), 'reason': a.get('reason', ''),
                  'stopped_s': 0. if stopped_t is None else round(b['t'] - stopped_t, 2), 'j_dist': a.get('j_dist')})
  return out


def near_junction(r) -> bool:
  """The row is at the scenario's junction (within NEAR_JUNCTION_M), or doesn't say (no junction, a recorded drive)."""
  return r.get('j_dist') is None or r['j_dist'] <= NEAR_JUNCTION_M


def oncoming_presented(rows) -> bool | None:
  """An oncoming car came within PRESENT_CAR_M of the junction while the car was at it (NEAR_JUNCTION_M), still before
  it (the junction ahead along its heading): the conflict the left scenarios exist for. None when the rows don't say
  (no oncoming car, or rows from before they recorded it)."""
  if not any(r.get('car_j_dist') is not None for r in rows):
    return None
  return any(r.get('car_j_dist') is not None and r['car_j_dist'] <= PRESENT_CAR_M and r.get('j_dist') is not None and
             r['j_dist'] <= NEAR_JUNCTION_M and (r.get('j_ahead') or 0.) > 0. for r in rows)


def _after_advance(rows, i: int) -> tuple[bool, int]:
  """From row i, where navd moved on to a new step: (whether the car left the route, the row where that was settled).
  On the route, the new step's instruction stays valid on the same route until the car has driven CONFIRM_M along it.
  Off it (it went straight on past a turn: navd projects it past the step's end and moves on all the same), navd sends
  no instruction (OFF_ROUTE_DISTANCE) and reroutes seconds later, outside REROUTE_WINDOW_S."""
  base_route, base_left = rows[i].get('route'), rows[i]['nav_remaining']
  for j in range(i, len(rows)):
    r = rows[j]
    if not r.get('nav_valid') or r.get('route') != base_route:
      return True, j
    if base_left - r['nav_remaining'] >= CONFIRM_M:
      return False, j
  return False, len(rows) - 1


def _turn_deg(rows, a: int, b: int, side: int) -> float | None:
  """The most the car's heading turned toward `side` (route_follower.LEFT/RIGHT) over rows a..b, in degrees; None
  without headings (a recorded drive's rows)."""
  hs = [r['heading'] for r in rows[a:b + 1] if r.get('heading') is not None]
  if len(hs) < 2:
    return None
  turned = (np.unwrap(np.array(hs)) - hs[0]) * side
  return round(float(np.degrees(turned.max())), 1)


def maneuvers(rows) -> list[dict]:
  """The turns and keeps of the route in the order navd gave them: attempted once the car came within ATTEMPT_M while
  engaged. Taken if navd then moved on to the next step of the same route and the car stayed on it (_after_advance), and
  for a turn, where the rows have headings, if the car's heading turned TURN_TAKEN_DEG its way. A missed turn makes navd
  reroute, which shows as a new navRoute around the step change (navd sends the instruction and the route in either
  order), or as the car off the route just after it: a straight-through at a turn moves navd on, then off the route."""
  route_changes = []
  prev = None
  for r in rows:
    if r.get('nav_valid'):
      if prev is not None and prev.get('route') is not None and r.get('route') != prev.get('route'):
        route_changes.append(r['t'])
      prev = r
  steps: list[dict] = []
  cur = None
  for i, r in enumerate(rows):
    if not r.get('nav_valid'):
      continue
    future = r['nav_remaining'] - r['nav_dist']  # navd: the route left after this step's maneuver, fixed per step
    if cur is None or abs(future - cur['future']) > STEP_CHANGE_M:
      if cur is not None:
        rerouted = any(abs(t - r['t']) <= REROUTE_WINDOW_S for t in route_changes)
        cur['end'] = 'advanced' if future < cur['future'] and not rerouted else 'reroute'
        cur['end_i'] = i
      cur = {'future': future, 'type': r.get('nav_type', ''), 'modifier': r.get('nav_mod', ''), 'attempted': False, 't': None,
             'attempt_i': None}
      steps.append(cur)
    cur['type'], cur['modifier'] = r.get('nav_type', ''), r.get('nav_mod', '')
    if r['nav_dist'] < ATTEMPT_M and r['enabled'] and not cur['attempted']:
      cur['attempted'], cur['t'], cur['attempt_i'] = True, r['t'], i
  out = []
  for s in steps:
    kind, side = classify(s['type'], s['modifier'])
    if kind not in (TURN, KEEP):
      continue
    advanced = s['attempted'] and s.get('end') == 'advanced'
    left_route, turn_deg = False, None
    if advanced:
      left_route, settled = _after_advance(rows, s['end_i'])
      if kind == TURN:
        turn_deg = _turn_deg(rows, s['attempt_i'], settled, side)
    taken = advanced and not left_route and (turn_deg is None or turn_deg >= TURN_TAKEN_DEG)
    out.append({'type': s['type'], 'modifier': s['modifier'], 'attempted': s['attempted'], 't': s['t'], 'taken': taken,
                'left_route_after': left_route, 'turn_deg': turn_deg})
  return out


def min_gap(rows, key):
  vals = [r[key] for r in rows if r.get(key) is not None]
  return round(min(vals), 2) if vals else None


def oncoming_offsets(rows, road) -> list:
  """For each row, m the car's centreline is left of the divider between the two directions of the road it drives along
  (negative: on its own side); None inside a junction box (a turn crosses there), off the roads, across a road, or with
  no position. road: catalog.road_model's {'links': [[ax, ay, bx, by, ra, rb, lanes], ...], 'junctions': [[x, y, box, line],
  ...]}, lanes as BeamNG's lane string for travel a -> b, left to right ('+' that way, '-' the other). A road with no '-'
  lane our way has no oncoming lane; one with only '-' lanes is all oncoming (the wrong way down a one-way road). A
  junction's box reaches box m from its node, and for a car heading into it (within INTO_JUNCTION_DEG of the bearing to
  it) its farthest mapped stop line (line m from the node, 0 without one or in an older result's [x, y, box]) plus
  LINE_BOX_MARGIN_M: a car pulls away from the line toward its turn."""
  links = road['links']
  if not links:
    return [None] * len(rows)
  L = np.array([lk[:6] for lk in links], dtype=float)
  lanes = [lk[6] for lk in links]
  a, d = L[:, 0:2], L[:, 2:4] - L[:, 0:2]
  length = np.maximum(np.linalg.norm(d, axis=1), 1e-6)
  u = d / length[:, None]
  J = np.array([list(j[:4]) + [0.] * (4 - len(j[:4])) for j in road['junctions']], dtype=float).reshape(-1, 4)
  cos_into = np.cos(np.radians(INTO_JUNCTION_DEG))
  cos_aligned = np.cos(np.radians(ALIGNED_DEG))
  out = []
  for r in rows:
    if r.get('x') is None or r.get('y') is None or r.get('heading') is None:
      out.append(None)
      continue
    p = np.array([r['x'], r['y']])
    h = np.array([np.cos(r['heading']), np.sin(r['heading'])])
    if len(J):
      dj = np.hypot(J[:, 0] - p[0], J[:, 1] - p[1])
      into = ((J[:, 0] - p[0]) * h[0] + (J[:, 1] - p[1]) * h[1]) >= cos_into * dj
      if (dj < J[:, 2]).any() or ((dj < J[:, 3] + LINE_BOX_MARGIN_M) & (J[:, 3] > 0.) & into).any():
        out.append(None)
        continue
    f = np.clip(((p - a) * u).sum(axis=1) / length, 0., 1.)
    radius = L[:, 4] + f * (L[:, 5] - L[:, 4])
    rel = p - (a + f[:, None] * d)
    dist = np.linalg.norm(rel, axis=1)
    along = u @ h
    ok = (np.abs(along) >= cos_aligned) & (dist <= radius + ON_ROAD_M)
    if not ok.any():
      out.append(None)
      continue
    k = int(np.argmin(np.where(ok, dist, np.inf)))
    fwd = 1. if along[k] > 0 else -1.
    lane_str = lanes[k] if fwd > 0 else ''.join('+' if c == '-' else '-' for c in reversed(lanes[k]))
    n_against = lane_str.count('-')
    if n_against == 0:
      out.append(None)
      continue
    ux, uy = u[k] * fwd
    right = rel[k, 0] * uy - rel[k, 1] * ux  # m right of the centreline, for our travel
    divider = -radius[k] + 2 * radius[k] / len(lane_str) * n_against  # right of the centreline, for our travel
    out.append(round(float(divider - right), 2))
  return out


def wrong_side(rows, road) -> tuple[float | None, list[dict]]:
  """m driven in an oncoming lane (WRONG_SIDE_PAST_M past the divider, outside the junction boxes), and each stretch of
  it; None without positions (a recorded drive) or without a road model (or one with no roads in it)."""
  if road is None or not road.get('links') or not any(r.get('x') is not None for r in rows):
    return None, []  # with positions, check() fails the run for it: an empty road model checks nothing
  past = oncoming_offsets(rows, road)
  total, spans, cur = 0., [], None
  for i, r in enumerate(rows):
    if past[i] is not None and past[i] > WRONG_SIDE_PAST_M:
      dm = r['v'] * (r['t'] - rows[i - 1]['t']) if i else 0.
      total += dm
      if cur is None:
        cur = {'t': r['t'], 'x': r['x'], 'y': r['y'], 'm': 0., 'max_past_m': past[i]}
        spans.append(cur)
      cur['m'] = round(cur['m'] + dm, 1)
      cur['max_past_m'] = max(cur['max_past_m'], past[i])
    else:
      cur = None
  return round(total, 1), spans


def lane_changes(rows) -> list[dict]:
  """Each start of a DesireHelper lane change, with where the pacer car was then."""
  out, prev = [], None
  for r in rows:
    s = r.get('lc_state')
    if s == 'laneChangeStarting' and prev != s:
      out.append({'t': r['t'], 'pacer_dx': r.get('pacer_dx'), 'v': r['v']})
    prev = s
  return out


def stop_gap(rows, outcome: str = ''):
  """Camera-to-mannequin gap where the car stopped for one: where the run ended if it ended stopped, else where the car
  first stood after moving (a pause at the rolling start is not a stop)."""
  if outcome == 'stopped' and rows and rows[-1].get('ped_gap') is not None:
    return round(rows[-1]['ped_gap'], 2)
  moved = False
  for r in rows:
    moved = moved or r['v'] > RESUMED_V
    if moved and r['v'] < STOPPED_V and r.get('ped_gap') is not None:
      return round(r['ped_gap'], 2)
  return None


class EndCheck:
  """Decides live when a run is over: arrived, disengaged, stopped (standing end_on_stop_s after moving), or timeout."""
  def __init__(self, timeout_s: float, end_on_stop_s: float = 0.):
    self.timeout_s, self.end_on_stop_s = timeout_s, end_on_stop_s
    self.engaged = self.moved = False
    self.stopped_t = None

  def update(self, r) -> str | None:
    self.engaged = self.engaged or r['enabled']
    if self.engaged and not r['enabled']:
      return 'disengaged'
    if (r.get('dest_dist') is not None and r['dest_dist'] < ARRIVE_M) or r.get('state') == 'arrived':
      return 'arrived'
    self.moved = self.moved or r['v'] > RESUMED_V
    if self.end_on_stop_s > 0 and self.moved and r['v'] < STOPPED_V:
      self.stopped_t = r['t'] if self.stopped_t is None else self.stopped_t
      if r['t'] - self.stopped_t >= self.end_on_stop_s:
        return 'stopped'
    else:
      self.stopped_t = None
    if r['t'] >= self.timeout_s:
      return 'timeout'
    return None


def _hold_with(hs, word):
  return [h for h in hs if word in h['reason']]


def zone_spans(rows, dist) -> list[tuple[float, float]]:
  """[(t in, t out)] of the spans dist(row) is within CONFLICT_M (None: not known, out)."""
  spans, start = [], None
  for r in rows:
    d = dist(r)
    inside = d is not None and d <= CONFLICT_M
    if inside and start is None:
      start = r['t']
    elif not inside and start is not None:
      spans.append((start, r['t']))
      start = None
  if start is not None:
    spans.append((start, rows[-1]['t']))
  return spans


def span_gap(a, b) -> float:
  """s between two spans, 0 if they overlap."""
  return max(b[0] - a[1], a[0] - b[1], 0.)


def unprotected_left(rows) -> dict:
  """The autonomous left's numbers (module constants): the minimum post-encroachment time with any oncoming car, false gos,
  the gaps offered at the line, those missed, and whether it ended undecided (before or after a gap of tReq was offered)."""
  n = max((len(r['onc']) for r in rows if r.get('onc')), default=0)
  ego = zone_spans(rows, lambda r: r.get('j_dist'))
  cars = [zone_spans(rows, lambda r, k=k: r['onc'][k][0] if r.get('onc') and len(r['onc']) > k else None) for k in range(n)]
  pets = [span_gap(e, c) for e in ego for spans in cars for c in spans]
  gos = [i for i in range(1, len(rows)) if rows[i].get('left_state') == 'go' and rows[i - 1].get('left_state') != 'go']
  false_goes = []
  for i in gos:
    r = rows[i]
    j = next((j for j in range(i, -1, -1) if r['t'] - rows[j]['t'] >= APPROACH_SPEED_S), None)
    for k in range(n):
      if j is None or not r.get('onc') or len(r['onc']) <= k or not rows[j].get('onc') or len(rows[j]['onc']) <= k:
        continue
      d, ahead = r['onc'][k]
      v = (rows[j]['onc'][k][0] - d) / (r['t'] - rows[j]['t'])
      if ahead > 0. and d > CONFLICT_M and v > 0.5 and (d - CONFLICT_M) / v < r.get('left_treq', 0.):
        false_goes.append({'t': r['t'], 'car': k, 'tta_s': round((d - CONFLICT_M) / v, 2), 't_req': r.get('left_treq')})
  # gaps between the cars' passes through the zone (and after the last, to the run's end), against the plan's tReq
  passes = sorted(c for spans in cars for c in spans)
  bounds = [(a[1], b[0]) for a, b in zip(passes, passes[1:], strict=False) if b[0] > a[1]]
  if passes and rows[-1]['t'] > passes[-1][1]:
    bounds.append((passes[-1][1], rows[-1]['t']))
  # the tReq it judged with at its line: any held row's (waiting, creeping, or undecided at once)
  held = [r for r in rows if r.get('left_state') in ('waitGap', 'creep', 'undecided') and r['v'] < STOPPED_V]
  t_req = max((r.get('left_treq', 0.) for r in held), default=0.)
  t_line = held[0]['t'] if held else None
  offered = [(a, b) for a, b in bounds if t_req > 0. and b - a >= t_req]
  waiting = [r['t'] for r in rows if r.get('left_state') == 'waitGap' and r['v'] < STOPPED_V]
  go_t = [rows[i]['t'] for i in gos]
  missed = [(a, b) for a, b in offered if any(a <= t <= b - t_req for t in waiting) and not any(a <= t <= b - t_req for t in go_t)]
  undecided_t = next((r['t'] for r in rows if r.get('left_state') == 'undecided'), None)
  # undecided passes only where no gap of tReq was offered with the car at its line, before or after it: a stream that ends
  # in an empty road offers one. No tReq known at all is no pass either
  return {'pet_s': round(min(pets), 2) if pets else None, 'gos': go_t, 'false_goes': false_goes,
          'gaps_offered_s': [round(b - a, 1) for a, b in offered], 'missed_gaps': len(missed), 't_req': t_req,
          'undecided_t': undecided_t,
          'gap_before_undecided': undecided_t is not None and (t_req <= 0. or any(t_line is not None and b > t_line for a, b in offered)),
          'stalls': sorted({r['control_prompt'] for r in rows if r.get('control_prompt') in LEFT_STALLS}),
          'turn_missed': any(r.get('turn_missed') for r in rows)}


def _disengage_text(ds: list[dict], before: str = '') -> str:
  return f"{len(ds)} disengagement(s){before}: {ds[0]['cause']}"


def _fails(name: str, res: dict) -> list[tuple[str, str]]:
  """(key, why) of each way a scored run fails its scenario before its expectations: the key says which check, for
  expectations() to explain."""
  fails = []
  if res['disengagements']:
    fails.append(('disengage', _disengage_text(res['disengagements'])))
  if res['positions'] and res['wrong_side_m'] is None:
    fails.append(('road', "no road model: the oncoming lane was not checked"))
  elif res['wrong_side_m'] is not None and res['wrong_side_m'] >= WRONG_SIDE_MIN_M:
    w = res['wrong_side'][0]
    fails.append(('wrong_side', f"drove {res['wrong_side_m']:.0f} m in the oncoming lane outside the junctions, first at t={w['t']:.1f} s " +
                  f"({w['x']:.0f}, {w['y']:.0f}), up to {max(x['max_past_m'] for x in res['wrong_side']):.1f} m past the divider"))
  taps = res.get('gas_taps', [])
  if taps:  # no gas go anywhere (Kellen, 2026-10-28): any press while engaged is the driver's, a failed run
    x = taps[0]
    fails.append(('gas', f"{len(taps)} gas press(es), no taps anywhere, first at t={x['t']:.1f} s ({x['reason'] or 'no hold'}, " +
                  f"stood {x['stopped_s']} s)"))
  taken, attempted = res['maneuvers_taken'], res['maneuvers_attempted']
  hs = res['holds']
  if name == 'ped_crosswalk_right':
    h = _hold_with(hs, 'crosswalk')
    if not any(x['stopped'] for x in h):
      fails.append(('crosswalk', "never held to a stop for the pedestrian in the crosswalk"))
    elif not any(x['resumed'] for x in h if x['stopped']):
      fails.append(('stall', "did not move on after the crosswalk cleared"))
  elif name.startswith('unprotected_left'):  # and its variants at other approaches, gap streams and the occluded view
    lr = res['left']
    if lr['undecided_t'] is not None and lr['gap_before_undecided']:
      gap = f"a gap of {max(lr['gaps_offered_s'])} s" if lr['gaps_offered_s'] else "no tReq known"
      fails.append(('left_undecided', f"undecided at t={lr['undecided_t']:.1f} s with {gap} (tReq {lr['t_req']} s) offered at its line"))
    elif lr['undecided_t'] is None:
      # waiting is required wherever an oncoming car was presented; the wait is the left's own at its line
      h = _hold_with(hs, 'gap') + _hold_with(hs, 'oncoming') + _hold_with(hs, 'Oncoming')
      if res.get('oncoming_presented') is False:
        fails.append(('left_wait', f"no oncoming car came within {PRESENT_CAR_M:.0f} m of the junction while the car was at it: " +
                      "the wait went untested"))
      elif not h:
        fails.append(('left_wait', "never waited for oncoming traffic"))
      if not lr['gos']:
        fails.append(('left_go', "never went at the unprotected left"))
      if lr['false_goes']:
        x = lr['false_goes'][0]
        fails.append(('left_false_go', f"{len(lr['false_goes'])} false go(es): at t={x['t']:.1f} s an oncoming car {x['tta_s']} s from " +
                      f"the junction (tReq {x['t_req']} s)"))
      if lr['pet_s'] is not None and lr['pet_s'] < PET_MIN_S:
        fails.append(('left_pet', f"post-encroachment time {lr['pet_s']} s with an oncoming car, under {PET_MIN_S} s"))
    if lr['stalls']:
      fails.append(('stall', f"stalled or never moved on its go ({', '.join(lr['stalls'])})"))
    if lr['turn_missed']:
      fails.append(('turn_missed', "missed the turn"))
    if res['min_gap_car_m'] is not None and res['min_gap_car_m'] < CAR_MIN_GAP_M:
      fails.append(('car_gap', f"came within {res['min_gap_car_m']} m of an oncoming car"))
  elif name == 'ped_in_path':
    g = res['stop_gap_m']
    if res['outcome'] != 'stopped' or g is None:
      fails.append(('ped_stop', "did not stop for the pedestrian"))
    elif not PED_STOP_GAP[0] <= g <= PED_STOP_GAP[1]:
      fails.append(('ped_stop', f"stopped {g} m from the pedestrian, outside {PED_STOP_GAP[0]}-{PED_STOP_GAP[1]} m"))
  elif name == 'ped_step_out':
    if res['outcome'] != 'stopped':
      fails.append(('ped_stop', "did not stop for the pedestrian"))
  elif name == 'occupied_lane':
    if not res['lane_blocked']:
      fails.append(('lane_blocked', "the planner never saw the target lane as occupied"))
    if not res['lane_changes']:
      fails.append(('lane_change', "never changed lanes"))
    elif (dx := res['lane_changes'][0]['pacer_dx']) is not None and abs(dx) < LC_BLOCK_X_M:
      fails.append(('lane_change', f"changed lanes with the pacer {dx:.1f} m alongside"))
  elif name == 'lane_change':
    if not res['lane_changes']:
      fails.append(('lane_change', "never changed lanes"))
  if name == 'twoway_route_pair' and res.get('stop_met_t') is not None:
    fails.append(('stop_met', f"met a stop sign at t={res['stop_met_t']:.1f} s: the route was to go around it (with " +
                              "TurnPilotAvoidStops off, run it with --expect handback:stopSignTakeControl)"))
  red = res.get('red_light')
  if name in RED_STOP_SCENARIOS:
    if red is None:
      fails.append(('red', "no forced light in the rows: the red was not checked"))
    else:
      if not red['stood_at_red']:
        fails.append(('red', "never stood at the forced red"))
      if red['past_line_on_red_t'] is not None:
        fails.append(('red', f"past the stop line on the forced red at t={red['past_line_on_red_t']:.1f} s"))
  undecided_ok = name.startswith('unprotected_left') and res['left']['undecided_t'] is not None and not res['left']['gap_before_undecided']
  arrives = ('ped_crosswalk_right', 'lane_change', 'route_demo', 'right_green', 'left_t_junction', 'twoway_route_pair') + RED_STOP_SCENARIOS
  if (name in arrives or name.startswith('unprotected_left')) and not undecided_ok:
    if res['outcome'] != 'arrived':
      fails.append(('not_arrived', f"ended {res['outcome']}, not arrived"))
    if attempted == 0 or taken < attempted:
      fails.append(('maneuvers', f"took {taken} of {attempted} maneuvers"))
  if name in ('ped_crosswalk_right', 'ped_in_path', 'ped_step_out') and res['min_gap_ped_m'] is not None and \
     res['min_gap_ped_m'] < COLLISION_GAP_M:
    fails.append(('ped_contact', f"came within {res['min_gap_ped_m']} m of the pedestrian"))
  return fails


def parse_expect(expect) -> list[tuple[str, str]]:
  """[(kind, arg)] of expectation strings ('handback:detectorStopped', 'skip', ...); an unknown one is refused."""
  out = []
  for e in expect or ():
    kind, _, arg = str(e).partition(':')
    if kind not in EXPECTS or (kind == 'handback') != bool(arg):
      raise ValueError(f"unknown expectation {e!r}: one of {', '.join(EXPECTS)} ('handback:<kind>')")
    out.append((kind, arg))
  return out


def expect_facts(rows) -> dict:
  """What the expectations look for in the rows: the first time of each handback (turnpilotPlan.handback, or a take-control
  controlPrompt), control alert, skip and missed turn; the first row showing the gas once engaged; and the turn desires and
  TurnPilot prompts after them."""
  first: dict[str, dict] = {'handback': {}, 'alert': {}}
  skip_t = missed_t = gas_t = None
  engaged = False
  for r in rows:
    engaged = engaged or r['enabled']
    hb, cp = r.get('handback', 'none'), r.get('control_prompt', 'none')
    for k in (hb if hb not in ('', 'none') else None, cp if cp.endswith('TakeControl') else None):
      if k is not None:
        first['handback'].setdefault(k, r['t'])
    if r.get('control_alert', 'none') not in ('', 'none'):
      first['alert'].setdefault(r['control_alert'], r['t'])
    if skip_t is None and (r.get('too_fast') or r.get('left_state') == 'skipped'):
      skip_t = r['t']
    if missed_t is None and r.get('turn_missed'):
      missed_t = r['t']
    if gas_t is None and engaged and r.get('gas'):
      gas_t = r['t']
  return {'handbacks': first['handback'], 'control_alerts': first['alert'], 'skip_t': skip_t, 'turn_missed_t': missed_t,
          'gas_t': gas_t,
          'desire_after_skip_t': next((r['t'] for r in rows if skip_t is not None and r['t'] >= skip_t and r.get('desire') in TURN_DESIRES), None),
          'prompts_after_gas': sorted({k for k, t in first['handback'].items() if gas_t is not None and t >= gas_t})}


def expectations(name: str, res: dict, fails: list[tuple[str, str]], expect) -> tuple[list[tuple[str, str]], list[dict]]:
  """(the failures left, each expectation's {expect, met, t, explains}): an expectation not seen is a failure; one seen
  explains only the generic failures it causes (module docstring)."""
  facts = res['expect_facts']
  dis = list(res['disengagements'])
  explained_dis: list[dict] = []
  out, extra = [], []
  explains: dict[str, list[str]] = {}

  def explain(e: str, keys) -> None:
    for k, why in fails:
      if k in keys:
        explains.setdefault(e, []).append(why)
  for kind, arg in parse_expect(expect):
    e = f"{kind}:{arg}" if arg else kind
    t, met = None, False
    if kind == 'handback':
      t = facts['handbacks'].get(arg)
      met = t is not None
      if not met:
        extra.append(f"expected handback {arg}: none came")
      else:
        before = [d for d in dis if d['t'] < t]
        explained_dis += [d for d in dis if d['t'] >= t]
        if before:
          extra.append(_disengage_text(before, f" before the expected handback {arg}"))
        explain(e, ('not_arrived', 'maneuvers') + (('stall',) if arg in LEFT_STALLS else ()) +
                (('stop_met',) if arg == 'stopSignTakeControl' else ()) +
                (('left_go', 'left_wait') if arg == 'cantStartTakeControl' else ()))
    elif kind == 'gasDisengage':
      t = facts['gas_t']
      after = [d for d in dis if t is not None and d['t'] >= t]
      if t is None or not after:
        extra.append("expected gasDisengage: no disengagement on the gas")
      elif after[0]['t'] - t > GAS_DISENGAGE_S:
        extra.append(f"disengaged {after[0]['t'] - t:.2f} s after the gas press, over {GAS_DISENGAGE_S} s")
      else:
        met = True
        explained_dis.append(after[0])
        explain(e, ('gas', 'not_arrived', 'maneuvers'))
        if facts['prompts_after_gas']:
          extra.append(f"TurnPilot handed back after the gas press ({', '.join(facts['prompts_after_gas'])})")
    elif kind == 'skip':
      t = facts['skip_t']
      met = t is not None
      if not met:
        extra.append("expected skip: no turn skipped (tooFastToTurn)")
      else:
        explain(e, ('left_wait', 'left_go', 'not_arrived', 'maneuvers'))
        if facts['desire_after_skip_t'] is not None:
          extra.append(f"a turn desire after the skip, at t={facts['desire_after_skip_t']:.1f} s")
    elif kind == 'turnMissed':
      t = facts['turn_missed_t']
      met = t is not None
      if not met:
        extra.append("expected turnMissed: the turn was taken (no turnMissed)")
      else:
        explain(e, ('turn_missed', 'not_arrived', 'maneuvers'))
    elif kind == 'hardBrake':
      t = facts['control_alerts'].get('hardBrake')
      met = t is not None
      if not met:
        extra.append("expected hardBrake: no hardBrake alert")
    out.append({'expect': e, 'met': met, 't': t, 'explains': explains.get(e, [])})
  gone = {k for e in out for k, why in fails if why in e['explains']}
  left = [(k, why) for k, why in fails if k not in gone and k != 'disengage']
  if explained_dis:
    for e in out:
      if e['met'] and (e['expect'].startswith('handback:') or e['expect'] == 'gasDisengage'):
        e['explains'] = [_disengage_text([d]) for d in explained_dis if d['t'] >= (e['t'] or 0.)] + e['explains']
        break
  rest = [d for d in dis if d not in explained_dis and not any(d['t'] < (e['t'] or 0.) and e['expect'].startswith('handback:')
                                                                for e in out if e['met'])]
  if rest:
    left.insert(0, ('disengage', _disengage_text(rest)))
  return left + [('expect', x) for x in extra], out


def check(name: str, res: dict, expect=()) -> list[str]:
  """Why a scored run fails its scenario (empty: it passes): its checks, then its expectations (module docstring)."""
  fails = _fails(name, res)
  if expect:
    fails, res['expected'] = expectations(name, res, fails, expect)
  return [why for _, why in fails]


def red_light(rows, stop_line_m: float | None) -> dict | None:
  """The forced light (the runner's forced_light rows): whether the car stood at it red (within RED_NEAR_M before the
  junction's stop line, stop_line_m before the junction), when the camera mount was RED_PAST_LINE_M past the line with it
  red, the speed at the line, and when it went green. None without a forced light in the rows."""
  if not any(r.get('forced_light') for r in rows):
    return None
  line = stop_line_m if stop_line_m is not None else 0.
  stood = past = v_line = green = None
  for r in rows:
    a = r.get('j_ahead')
    if green is None and r.get('forced_light') == 'green':
      green = r['t']
    if a is None:
      continue
    if v_line is None and a <= line:
      v_line = r['v']
    if r.get('forced_light') == 'red':
      if stood is None and r['v'] < STOPPED_V and line - RED_PAST_LINE_M <= a <= line + RED_NEAR_M:
        stood = r['t']
      if past is None and a < line - RED_PAST_LINE_M:
        past = r['t']
  return {'stood_at_red': stood is not None, 'stood_t': stood, 'past_line_on_red_t': past, 'v_at_line': v_line, 'green_t': green,
          'stop_line_m': stop_line_m}


def summarize(name: str, rows, outcome: str, road: dict | None = None, junction_control: str | None = None, expect=(),
              stop_line_m: float | None = None) -> dict:
  """road: catalog.road_model's roads around the run, for the oncoming-lane check (rows with positions need one);
  junction_control: the scenario junction's own mapped control (catalog.junction_control's kind: 'signal', 'stop',
  'none'; None: not known), recorded with the result (a signal there no longer protects a left: map_controls);
  expect: the run's expectations (EXPECTS); stop_line_m: m before the junction of its own stop line (the plan's notes)."""
  ms = maneuvers(rows)
  wrong_m, wrong_spans = wrong_side(rows, road)
  res = {
    'outcome': outcome,
    'maneuvers_taken': sum(m['taken'] for m in ms),
    'maneuvers_attempted': sum(m['attempted'] for m in ms),
    'maneuvers': ms,
    'disengagements': disengagements(rows),
    'holds': holds(rows),
    'min_gap_ped_m': min_gap(rows, 'ped_gap'),
    'min_gap_car_m': min_gap(rows, 'car_gap'),
    'stop_gap_m': stop_gap(rows, outcome),
    'gap_reference': "camera mount (top of the windshield) to the mannequin, or to the other car's centre",
    'lane_changes': lane_changes(rows),
    'gas_taps': gas_taps(rows),
    'positions': any(r.get('x') is not None for r in rows),
    'wrong_side_m': wrong_m,
    'wrong_side': wrong_spans,
    # the prompts TurnPilot showed with the car standing at the junction (waitingForGap, a handback)
    'stopped_for_go': sorted({r['control_prompt'] for r in rows if r.get('control_prompt', 'none') not in ('', 'none') and
                              r['v'] < STOPPED_V and near_junction(r)}),
    'junction_control': junction_control,
    'oncoming_presented': oncoming_presented(rows),
    'left': unprotected_left(rows),
    'lane_blocked': any(r.get('reason') == 'car in target lane' for r in rows),
    'signal_prompts': sum(1 for a, b in zip(rows, rows[1:], strict=False) if b.get('blinker') and b['blinker'] != a.get('blinker')),
    'expect': list(expect or ()),
    'expect_facts': expect_facts(rows),
    'red_light': red_light(rows, stop_line_m),
    'stop_met_t': next((r['t'] for r in rows if r.get('map_kind') == 'stop' and r['enabled']), None),  # a stop sign the plan met
  }
  res['failures'] = check(name, res, expect)
  res['passed'] = not res['failures']
  return res


def report(label: str, res: dict) -> str:
  holds = [(h['reason'], h['duration_s']) for h in res['holds']]
  taps = [(x['prompt'], x['stopped_s']) for x in res.get('gas_taps', [])]
  line = f"{label}: {'PASS' if res['passed'] else 'FAIL'} {res['outcome']}, {res['maneuvers_taken']}/{res['maneuvers_attempted']} maneuvers, " + \
         f"{len(res['disengagements'])} disengagements, holds {holds}, gas taps {taps}, min gap ped {res['min_gap_ped_m']} car {res['min_gap_car_m']}, " + \
         f"oncoming lane {res.get('wrong_side_m')} m"
  lr = res.get('left')
  if lr is not None and (lr['gos'] or lr['undecided_t'] is not None or lr['pet_s'] is not None):
    line += f", left: PET {lr['pet_s']} s, false goes {len(lr['false_goes'])}, missed gaps {lr['missed_gaps']} of " + \
            f"{len(lr['gaps_offered_s'])} offered, gos {lr['gos']}, undecided {lr['undecided_t']}"
  red = res.get('red_light')
  if red is not None:
    line += f", forced red: stood {red['stood_at_red']}, {red['v_at_line']} m/s at the line, green at {red['green_t']}"
  met = [e for e in res.get('expected', []) if e['met']]
  if met:
    line += ''.join(f"\n  as expected: {e['expect']} at t={e['t']:.1f} s" + (f" (explains: {'; '.join(e['explains'])})" if e['explains'] else '')
                    for e in met)
  return line + ''.join(f"\n  {f}" for f in res['failures'])


def main():
  ap = argparse.ArgumentParser(description="Rescore scenario results, or score a recorded drive's rlogs.")
  ap.add_argument('paths', nargs='+', help='result.json files, or with --scenario, rlog segments in order')
  ap.add_argument('--scenario', default='', help='score these rlogs as this scenario (route_demo: arrival and maneuvers)')
  ap.add_argument('--expect', action='append', default=None,
                  help=f"an expectation ({', '.join(EXPECTS)}; handback:<kind>), in place of the result's own; repeatable")
  args = ap.parse_args()
  if args.scenario:
    from openpilot.tools.lib.logreader import LogReader
    rows = [r for p in args.paths for r in rows_from_log(LogReader(p, sort_by_time=True))]
    print(report(f"{args.scenario} from {len(args.paths)} log(s)", summarize(args.scenario, rows, 'arrived' if any(
      r['state'] == 'arrived' for r in rows) else 'log ended', expect=args.expect or ())))
    return
  for path in args.paths:
    with open(path) as f:
      result = json.load(f)
    notes = result.get('notes') or {}
    for run in result['runs']:
      expect = args.expect if args.expect is not None else run.get('expect', result.get('expect', ()))
      print(report(f"{result['scenario']} run {run['run']}", summarize(result['scenario'], run['rows'], run['outcome'], road=result.get('road'),
                                                                     junction_control=notes.get('junction_control'), expect=expect,
                                                                     stop_line_m=notes.get('stop_line_m'))))


if __name__ == "__main__":
  main()
