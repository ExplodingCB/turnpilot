#!/usr/bin/env python3
"""Scenario scoring from recorded rows, one row per 50 ms of a run (run.py records them from msgq and the game).

A row is a dict:
  t, wall            s since engagement (run.py; a recorded drive's: since its first turnpilotPlan), and wall-clock time
  x, y, heading, v   the ego's camera mount in BeamNG world meters, heading rad CCW from east, wheel speed m/s
  enabled            selfdriveState.enabled; alert, alert_text: its alertType and alertText1
  steer, gas, brake  carState steeringPressed, gasPressed, brakePressed
  state, reason      turnpilotPlan.state and .reason; lc, prompt: its laneChange and signalPrompt
  control_prompt     turnpilotPlan.controlPrompt ('none', 'waitingForGap', 'leftGo', 'stalledTakeControl', ...)
  left_state         turnpilotPlan.unprotectedLeft.state ('off', 'waitGap', 'go', 'undecided', ...); left_treq, left_rreq its
                     tReq and rReq; turn_missed turnpilotPlan.turnMissed
  go_cert            turnpilotPlan.goCertified.cause: the go plannerd's bounded lift is certified for ('none', 'unprotectedLeft', ...)
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
  accel_cap          turnpilotPlan.accelCap; tp_accel: TurnPilot's accel candidate in plannerd from it and the plan's speedCap
                     (hooks.plan_accel), m/s^2 (None before the first plan); a_target: longitudinalPlan.aTarget, what plannerd
                     asked of the car whoever asked it (None without one)
  det_age            s since the last turnpilotDetections (None before the first)

A scenario's expectations (EXPECTS: catalog Entry.expect, or the runner's --expect and the options that imply one) are
what its shot exists to show: a handback, a skip, a missed turn, the driver's gas press, hard braking. Each is required of
the run (not seen: a failure), and explains only the generic failure it causes (the disengagement after the handback, the
press itself, the route not finished), so a run's verdict means what its shot shows. One is met only as its shot's
pass_rule shows it (handback_problems, range_skip_problems, missed_problems, the skip's junction and kind): a handback the
car wasn't held after, or late after its go or its stand, a drop's handback late or with the detector still running, a
skip past its junction or held for after it, a missed turn said away from the pass or for a turn taken, is seen but unmet,
and explains nothing.

Rescore a saved result: python -m openpilot.tools.sim.bridge.beamng.scenarios.score result.json
Score a recorded drive: python -m openpilot.tools.sim.bridge.beamng.scenarios.score --scenario route_demo rlog...
"""
import argparse
import json

import numpy as np

from openpilot.selfdrive.turnpilot.handback import EVENTS as HANDBACK_EVENTS, EventName, Handback
from openpilot.selfdrive.turnpilot.hooks import SPEED_CAP_ACCEL_MIN, TAKE_CONTROL_PROMPTS, plan_accel
from openpilot.selfdrive.turnpilot.lane_planner import LC_BLOCK_X_M
from openpilot.selfdrive.turnpilot.route_follower import KEEP, TURN, classify
from openpilot.selfdrive.turnpilot.scene_policy import STOP_DECEL
from openpilot.selfdrive.turnpilot.unprotected_left import CREEP_MAX_M

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
# the occluded left (S08): undecided with the view blocked is its fault handback, a pass even with a gap offered it couldn't
# see; a go with the view blocked fails; its creep past where it first stood at its line is unprotected_left.CREEP_MAX_M,
# plus this for the rows' camera positions
OCCLUDED = 'unprotected_left_occluded'
CREEP_SLACK_M = 1.
TURN_DESIRES = (1, 2)  # log.Desire.turnLeft, turnRight
# expectations (module docstring): 'handback:<kind>' (a turnpilotPlan.handback, or a take-control controlPrompt),
# 'gasDisengage', 'skip' (tooFastToTurn, or the left skipped at route time), 'turnMissed', 'hardBrake' (controlAlert).
# An unprotected left's skip is its range skip (unprotectedLeft.state skipped: turnpilotd sets it for turn_skip's
# unprotectedRange only); tooFastToTurn alone there is another skip (tooLate, laneUnreachable or tooFast), not its shot.
# A skip's alert comes before its junction (S05, S19): one first seen with the junction behind the car (j_ahead) isn't met.
# The left's range skip drives on (S05 'no hold at the line ... a reroute', after_range_skip): no TurnPilot hold within
# NEAR_JUNCTION_M of the junction after it, and a new navRoute once the car is past the junction (navd reroutes past a
# skipped turn only: navd.skipped_turn_passed)
# A missed turn is said at the pass (S18 'the alert within 3 s of passing the junction', missed_turn): turnMissed first
# comes with the junction behind the car (the first row with j_ahead at or under 0) and within TURN_MISSED_S of it;
# turnpilotd says it only for a turn its car has turned under MISSED_TURN_DEG (its own), so the car's heading has turned
# under that (and HEADING_TOL_DEG, the game's heading against turnpilotd's yaw) from its go (the last stand before the
# pass) when it comes, and never TURN_TAKEN_DEG through its alert: a turn taken is never a miss (1c69d2e's false miss)
TURN_MISSED_S = 3.
MISSED_TURN_DEG = 15.
HEADING_TOL_DEG = 3.
EXPECTS = ('handback', 'gasDisengage', 'skip', 'turnMissed', 'hardBrake')
# a take-control handback holds the car while the driver is told to take control (hooks.TAKE_CONTROL_PROMPTS): from it,
# engaged, the car goes no further than a stop from its speed then, HELD_REACT_S at that speed and HELD_DECEL after it (a
# go withdrawn rolling stops at turnpilotd's STOP_DECEL, 1.5 m/s^2), plus HELD_SLACK_M. The stop sign's (FULL_STOP_HANDBACKS)
# comes after a full stop: the car standing at the handback's own row, never rolling (not a stand some time back and a roll
# up to the sign). The left's withdrawn go (SHORT_OF_LANES, S02 'stopped short of the oncoming lanes') is handed back
# outside the junction's conflict zone (CONFLICT_M) and held there: engaged after it the car never comes within CONFLICT_M
# of the junction, nor goes further than the zone was from it
HELD_REACT_S = 0.5
HELD_DECEL = STOP_DECEL
HELD_SLACK_M = 1.0
FULL_STOP_HANDBACKS = ('stopSignTakeControl',)
SHORT_OF_LANES = ('cantStartTakeControl',)
# ...and comes soon after the left's go (S02 'handback within 3.5 s of the go, 1.5 s with the lift off'): LIFT_HANDBACK_S
# from it with plannerd's lift certified for it (go_override.GO_WINDOW_S, 3 s, then withdrawn), NO_LIFT_HANDBACK_S without
# (turnpilotd's LEFT_NO_MOVE_S, 1 s, standing); rows from before go_cert are held to the former, the most either allows
GO_TIMED = ('cantStartTakeControl',)
LIFT_HANDBACK_S = 3.5
NO_LIFT_HANDBACK_S = 1.5
# a row or two: the plan's message and the car's each land in the 50 ms rows up to a row late
ROW_TOL_S = 0.1
# the stall's handback (S03 'handback 3 s after the stand') comes STALL_PROMPT_S (turnpilotd's) into the car's stand with
# nothing holding it (no TurnPilot hold, no pedal, engaged: the clock turnpilotd's stall counts on), a row early at most
# and STALL_LATE_S late (its count passing 3 s, the plan's frame, carState's speed a little behind the game's)
STALL_HANDBACKS = ('stalledTakeControl',)
STALL_PROMPT_S = 3.
STALL_LATE_S = 0.3
# turnpilotPlan.handback's kinds: a fault, handed back with openpilot's soft disable, which ends the drive (the car is not
# held: it coasts engaged through the soft disable); the disengagement after one is that soft disable's, of the event the
# handback raises (fault_event: handback.EVENTS), not another's
FAULT_HANDBACKS = ('detectorStopped', 'modelFellBack', 'noDetector')
SOFT_DISABLE = '/softDisable'
# S01's drop (run.py --drop-detections-at; summarize's drop): the detectorStopped handback within DROP_HANDBACK_S of the
# detector's last detections before it (the shot's rule, from the rows' det_age), which come after the drop (but for a
# frame or so: DROP_BEFORE_TOL_S); rows without det_age (results from before it) are held to the drop's own time, the
# runner's write of its param, plus the detector's read of it, up to DETECTOR_PARAM_CHECK_S later (modeld_detector's
# PARAM_CHECK_S; BeamNG smoke 2026-10-29: written at 3.04 s, handed back at 5.29 s)
DROP_HANDBACK_S = 2.0
DETECTOR_PARAM_CHECK_S = 1.0
DROP_BEFORE_TOL_S = 0.25
# ...and with det_age in the rows, the handback's own row shows the detector silent for turnpilotd's DETECTOR_DEADLINE_S,
# but a row or two (ROW_TOL_S): a detectorStopped with detections still coming isn't the drop's
DETECTOR_DEADLINE_S = 1.5
# the disengagement within this of the first row showing the gas is the press's: the shot's 0.2 s (carState.gasPressed to
# disengaged, in the rlog), plus the rows' 50 ms sampling of two messages that may each land a row late
GAS_DISENGAGE_S = 0.3
PROTECTED_TURNS = ('right_green', 'left_t_junction')  # S20: taken without TurnPilot holding the car to a stop
RED_STOP_SCENARIOS = ('red_right', 'red_right_g62')  # a forced red they must stand at and not pass (red_straight: a limit)
RED_PAST_LINE_M = 1.  # the camera mount this far past the junction's stop line (the bumper ~3 m past it): into the junction
RED_NEAR_M = 30.      # a stand within this before the stop line is at the red
# TurnPilot's accel request (the rows' tp_accel) below the -2.0 m/s^2 floor of every map, light and speed-cap stop
# (hooks.SPEED_CAP_ACCEL_MIN) fails a run (most shots' 'no TurnPilot accel below -2.0'): only a confirmed track's hard stop
# asks for more (scene_policy.py), said on the same plan (controlAlert hardBrake), and only where the scenario has a
# pedestrian or oncoming traffic to brake for (a run expecting the left's skip never meets its oncoming traffic); one
# without the alert is never anyone's (the float32 plan read back: ACCEL_TOL)
ACCEL_FLOOR = SPEED_CAP_ACCEL_MIN
ACCEL_TOL = 0.01
HARD_BRAKE_PREFIXES = ('ped_', 'unprotected_left')
# ...and in the hard stop's own shot (S10 'the braking past -2.0 m/s^2 starts only after the track confirmed', ped_step_out
# or any run expecting hardBrake) the planner's aTarget (longitudinalPlan, whoever asked for it) below ACCEL_FLOOR before
# TurnPilot's first hardBrake fails too, as does one with no hardBrake at all; rows with no a_target fail it as not checked
HARD_BRAKE_SHOTS = ('ped_step_out',)


def msg_fields(ss, cs, tp, nav, model, lp=None) -> dict:
  """The message part of a row from selfdriveState, carState, turnpilotPlan (None before the first), navInstruction (None
  while navd has no valid route), modelV2 and longitudinalPlan (None before the first) readers."""
  return {
    'enabled': bool(ss.enabled), 'alert': str(ss.alertType), 'alert_text': str(ss.alertText1),
    'steer': bool(cs.steeringPressed), 'gas': bool(cs.gasPressed), 'brake': bool(cs.brakePressed),
    'state': '' if tp is None else str(tp.state), 'reason': '' if tp is None else str(tp.reason),
    'lc': '' if tp is None else str(tp.laneChange), 'prompt': '' if tp is None else str(tp.signalPrompt),
    'control_prompt': '' if tp is None else str(tp.controlPrompt),
    'left_state': 'off' if tp is None else str(tp.unprotectedLeft.state),
    'left_treq': 0. if tp is None else round(float(tp.unprotectedLeft.tReq), 2),
    'left_rreq': 0. if tp is None else round(float(tp.unprotectedLeft.rReq), 1),
    'left_blocked': False if tp is None else bool(tp.unprotectedLeft.viewBlocked),
    'turn_missed': False if tp is None else bool(tp.turnMissed),
    'handback': 'none' if tp is None else str(tp.handback), 'control_alert': 'none' if tp is None else str(tp.controlAlert),
    'too_fast': False if tp is None else bool(tp.tooFastToTurn),
    'go_cert': 'none' if tp is None else str(tp.goCertified.cause),
    'accel_cap': None if tp is None else round(float(tp.accelCap), 2),
    'tp_accel': None if tp is None else round(float(plan_accel(tp.accelCap, tp.speedCap, cs.vEgo)), 2),
    'a_target': None if lp is None else round(float(lp.aTarget), 2),
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


LOG_SERVICES = ('selfdriveState', 'carState', 'turnpilotPlan', 'navInstruction', 'navRoute', 'modelV2', 'longitudinalPlan',
                'turnpilotDetections')


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
    det = latest.get('turnpilotDetections')
    rows.append({'t': round(t - t0, 3), 'wall': None, 'x': None, 'y': None, 'heading': None, 'v': round(cs.vEgo, 2), 'route': route,
                 'dest_dist': None, 'blinker': 'left' if cs.leftBlinker else 'right' if cs.rightBlinker else '',
                 'ped_gap': None, 'car_gap': None, 'pacer_dx': None, 'j_dist': None, 'car_j_dist': None, 'onc': None,
                 'det_age': None if det is None else round(t - det.logMonoTime * 1e-9, 3),
                 **msg_fields(latest['selfdriveState'].selfdriveState, cs, m.turnpilotPlan,
                              nav.navInstruction if nav is not None and nav.valid else None, latest['modelV2'].modelV2,
                              latest['longitudinalPlan'].longitudinalPlan if 'longitudinalPlan' in latest else None)})
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
  """Decides live when a run is over: arrived, disengaged, stopped (standing end_on_stop_s after moving), past junction
  (end_past_m past the scenario's junction, along the car's heading: a skip's shot ends at the junction it skipped, before
  the reroute's own turns), or timeout."""
  def __init__(self, timeout_s: float, end_on_stop_s: float = 0., end_past_m: float = 0.):
    self.timeout_s, self.end_on_stop_s, self.end_past_m = timeout_s, end_on_stop_s, end_past_m
    self.engaged = self.moved = False
    self.stopped_t = None

  def update(self, r) -> str | None:
    self.engaged = self.engaged or r['enabled']
    if self.engaged and not r['enabled']:
      return 'disengaged'
    if (r.get('dest_dist') is not None and r['dest_dist'] < ARRIVE_M) or r.get('state') == 'arrived':
      return 'arrived'
    if self.end_past_m > 0 and r.get('j_ahead') is not None and r['j_ahead'] < -self.end_past_m:
      return 'past junction'
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
  # the view (unprotectedLeft.viewBlocked): at the undecided, at each go, and how far the car crept past where it first stood
  blocked_at_undecided = any(r.get('left_blocked') for r in rows if undecided_t is not None and undecided_t - 0.5 <= r['t'] <= undecided_t)
  go_blocked = [rows[i]['t'] for i in gos if rows[i].get('left_blocked') or rows[i - 1].get('left_blocked')]
  first_go = rows[gos[0]]['t'] if gos else None
  line = [r for r in rows if r.get('left_state') in ('waitGap', 'creep', 'undecided') and r.get('j_ahead') is not None and
          (first_go is None or r['t'] < first_go)]
  creep_m = round(line[0]['j_ahead'] - min(r['j_ahead'] for r in line), 2) if line else None
  # undecided passes only where no gap of tReq was offered with the car at its line, before or after it: a stream that ends
  # in an empty road offers one. No tReq known at all is no pass either
  return {'pet_s': round(min(pets), 2) if pets else None, 'gos': go_t, 'false_goes': false_goes,
          'gaps_offered_s': [round(b - a, 1) for a, b in offered], 'missed_gaps': len(missed), 't_req': t_req,
          'undecided_t': undecided_t,
          'gap_before_undecided': undecided_t is not None and (t_req <= 0. or any(t_line is not None and b > t_line for a, b in offered)),
          'stalls': sorted({r['control_prompt'] for r in rows if r.get('control_prompt') in LEFT_STALLS}),
          'turn_missed': any(r.get('turn_missed') for r in rows),
          'blocked_at_undecided': blocked_at_undecided, 'go_blocked': go_blocked, 'creep_m': creep_m}


def tp_accel(rows) -> dict:
  """TurnPilot's accel request while engaged (rows' tp_accel): its minimum, and the first row below ACCEL_FLOOR without a
  confirmed track's hard stop (unannounced) and with one (hard), each {t, accel, alert}; the planner's minimum aTarget, its
  first below ACCEL_FLOOR before TurnPilot's first hardBrake (a_target_early: {t, accel}), and that hardBrake's t."""
  out: dict = {'min': None, 'unannounced': None, 'hard': None, 'a_target_min': None, 'a_target_early': None, 'hard_brake_t': None}
  for r in rows:
    if not r.get('enabled'):
      continue
    if out['hard_brake_t'] is None and r.get('control_alert') == 'hardBrake':
      out['hard_brake_t'] = r['t']
    if r.get('a_target') is not None:
      out['a_target_min'] = r['a_target'] if out['a_target_min'] is None else min(out['a_target_min'], r['a_target'])
      if out['hard_brake_t'] is None and out['a_target_early'] is None and r['a_target'] < ACCEL_FLOOR - ACCEL_TOL:
        out['a_target_early'] = {'t': r['t'], 'accel': r['a_target']}
    a = r.get('tp_accel')
    if a is None:
      continue
    out['min'] = a if out['min'] is None else min(out['min'], a)
    if a < ACCEL_FLOOR - ACCEL_TOL:
      key = 'hard' if r.get('control_alert') == 'hardBrake' else 'unannounced'
      if out[key] is None:
        out[key] = {'t': r['t'], 'accel': a, 'alert': r.get('control_alert', 'none')}
  return out


def take_control_expected(expect) -> list[str]:
  """The take-control handbacks (hooks.TAKE_CONTROL_PROMPTS) among a run's expectations."""
  return [arg for kind, _, arg in (str(e).partition(':') for e in expect or ()) if kind == 'handback' and arg in TAKE_CONTROL_PROMPTS]


def hard_brake_ok(name: str, expect) -> bool:
  """A confirmed track's hard stop is the scenario's to show (ACCEL_FLOOR's comment). Never in a run expecting a
  take-control handback: its shot is the car held while the driver is asked to take over (S03's 'no TurnPilot accel below
  -2.0 m/s^2', in ped_crosswalk_right), not a hard stop."""
  if take_control_expected(expect):
    return False
  kinds = {str(e).partition(':')[0] for e in expect or ()}
  return 'hardBrake' in kinds or (name.startswith(HARD_BRAKE_PREFIXES) and 'skip' not in kinds)


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
  acc = res.get('tp_accel') or {}
  if (x := acc.get('unannounced')) is not None:
    fails.append(('accel', f"TurnPilot asked for {x['accel']} m/s^2 at t={x['t']:.1f} s, below {ACCEL_FLOOR} m/s^2 without a " +
                  "confirmed track's hard stop (no hardBrake)"))
  if (x := acc.get('hard')) is not None and not hard_brake_ok(name, res.get('expect')):
    tc = take_control_expected(res.get('expect'))
    why = f"a run expecting a take-control handback (handback:{tc[0]}) shows no hard stop" if tc else f"{name} has nothing to brake for"
    fails.append(('accel', f"TurnPilot braked hard ({x['accel']} m/s^2, hardBrake) at t={x['t']:.1f} s: {why} past {ACCEL_FLOOR} m/s^2"))
  if name in HARD_BRAKE_SHOTS or 'hardBrake' in {str(e).partition(':')[0] for e in res.get('expect') or ()}:
    if acc.get('a_target_min') is None:
      fails.append(('accel', f"no planner aTarget in the rows: whether the braking past {ACCEL_FLOOR} m/s^2 started only after the " +
                    "track confirmed was not checked"))
    elif (x := acc.get('a_target_early')) is not None:
      when = f"before TurnPilot's confirmed hard stop (hardBrake at t={acc['hard_brake_t']:.1f} s)" if acc.get('hard_brake_t') is not None \
        else "with no confirmed hard stop of TurnPilot's (no hardBrake)"
      fails.append(('accel', f"the planner asked for {x['accel']} m/s^2 at t={x['t']:.1f} s, below {ACCEL_FLOOR} m/s^2 {when}"))
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
    if name == OCCLUDED:
      if lr.get('go_blocked'):
        fails.append(('left_go_blocked', f"went with the view blocked at t={lr['go_blocked'][0]:.1f} s"))
      if lr.get('creep_m') is not None and lr['creep_m'] > CREEP_MAX_M + CREEP_SLACK_M:
        fails.append(('left_creep', f"crept {lr['creep_m']} m past where it first stood at its line, over {CREEP_MAX_M} m"))
    if lr['undecided_t'] is not None and lr['gap_before_undecided'] and not (name == OCCLUDED and lr.get('blocked_at_undecided')):
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
    # every go's safety, whatever the left ended in: a go withdrawn before an undecided handback is still a go
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
  if name in PROTECTED_TURNS:
    stood = [h for h in hs if h['stopped']]
    if stood:
      fails.append(('held', f"TurnPilot held the car to a stop at a protected turn ({stood[0]['reason']}, {stood[0]['duration_s']} s)"))
  red = res.get('red_light')
  if name in RED_STOP_SCENARIOS:
    if red is None:
      fails.append(('red', "no forced light in the rows: the red was not checked"))
    else:
      if not red['stood_at_red']:
        fails.append(('red', "never stood at the forced red"))
      if red['past_line_on_red_t'] is not None:
        fails.append(('red', f"past the stop line on the forced red at t={red['past_line_on_red_t']:.1f} s"))
  undecided_ok = name.startswith('unprotected_left') and res['left']['undecided_t'] is not None and \
    (not res['left']['gap_before_undecided'] or (name == OCCLUDED and res['left'].get('blocked_at_undecided')))
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


def held_after(rows, t: float) -> dict:
  """From a take-control handback at t (HELD_*): the car's speed then, the metres it went engaged after it (to the first row
  not engaged: from there the driver has it), the fastest it went, and the most a stop from that speed allows; the
  junction's distance then (j_dist), the nearest the car came to it engaged after (min_j_dist), and the metres to the
  junction's conflict zone at the handback (zone_m), None without one in the rows."""
  i0 = next((i for i, r in enumerate(rows) if r['t'] >= t), len(rows) - 1)
  v0, j0 = rows[i0]['v'], rows[i0].get('j_dist')
  went, v_max, min_j = 0., v0, j0
  for a, b in zip(rows[i0:], rows[i0 + 1:], strict=False):
    if not b['enabled']:
      break
    went += 0.5 * (a['v'] + b['v']) * (b['t'] - a['t'])
    v_max = max(v_max, b['v'])
    if b.get('j_dist') is not None:
      min_j = b['j_dist'] if min_j is None else min(min_j, b['j_dist'])
  return {'v0': v0, 'went_m': round(went, 2), 'v_max': round(v_max, 2),
          'allowed_m': round(v0 * HELD_REACT_S + v0 ** 2 / (2 * HELD_DECEL) + HELD_SLACK_M, 2),
          'j_dist': j0, 'min_j_dist': None if min_j is None else round(min_j, 2),
          'zone_m': None if j0 is None else round(j0 - CONFLICT_M, 2)}


def go_before(rows, t: float) -> dict | None:
  """The left's last go (left_state into go) at or before t: when, the s from it to t, whether plannerd's lift was certified
  for it (a row from it to t with go_cert unprotectedLeft; None: rows from before go_cert) and the bound the handback is
  held to (GO_TIMED). None without a go."""
  i_t = next((i for i, r in enumerate(rows) if r['t'] >= t), len(rows) - 1)
  go_i = next((i for i in range(i_t, -1, -1) if rows[i].get('left_state') == 'go' and (i == 0 or rows[i - 1].get('left_state') != 'go')), None)
  if go_i is None:
    return None
  span = rows[go_i:i_t + 1]
  lift = any(r.get('go_cert') == 'unprotectedLeft' for r in span) if any('go_cert' in r for r in span) else None
  return {'go_t': rows[go_i]['t'], 'after_s': round(t - rows[go_i]['t'], 2), 'lift': lift,
          'bound_s': NO_LIFT_HANDBACK_S if lift is False else LIFT_HANDBACK_S}


def stand_before(rows, t: float) -> dict:
  """The car's stand with nothing holding it up to the handback at t (STALL_HANDBACKS): the first row of the run of rows
  before it standing (under STOPPED_V), engaged, no pedal and no TurnPilot hold (state holding), and the s from it to t; 0
  s from t itself without one."""
  i_t = next((i for i, r in enumerate(rows) if r['t'] >= t), len(rows) - 1)
  j = i_t
  while j > 0 and (r := rows[j - 1])['v'] < STOPPED_V and r['enabled'] and not (r.get('gas') or r.get('brake')) and \
        r.get('state') != 'holding':
    j -= 1
  return {'from_t': rows[j]['t'], 'stood_s': round(t - rows[j]['t'], 2)}


def stood_at(rows, t: float) -> bool:
  """The car standing (under STOPPED_V) at the handback's own first row, the row at t: its full stop (FULL_STOP_HANDBACKS)."""
  return next((r for r in rows if r['t'] >= t), rows[-1])['v'] < STOPPED_V


def expect_facts(rows) -> dict:
  """What the expectations look for in the rows: the first time of each handback (turnpilotPlan.handback, or a take-control
  controlPrompt), control alert, skip and missed turn; the first row showing the gas once engaged; and the turn desires and
  TurnPilot prompts after them."""
  first: dict[str, dict] = {'handback': {}, 'alert': {}}
  det_t: dict[str, float | None] = {}  # each handback's detector: its last detections at the handback's first row
  det_age: dict[str, float | None] = {}  # ...and their age then
  skip_t = range_skip_t = missed_t = gas_t = None
  ahead: dict[str, float | None] = {}  # j_ahead at the first skip row of each kind
  engaged = False
  for r in rows:
    engaged = engaged or r['enabled']
    hb, cp = r.get('handback', 'none'), r.get('control_prompt', 'none')
    for k in (hb if hb not in ('', 'none') else None, cp if cp.endswith('TakeControl') else None):
      if k is not None and k not in first['handback']:
        first['handback'][k] = r['t']
        det_t[k] = None if r.get('det_age') is None else round(r['t'] - r['det_age'], 3)
        det_age[k] = r.get('det_age')
    if r.get('control_alert', 'none') not in ('', 'none'):
      first['alert'].setdefault(r['control_alert'], r['t'])
    if skip_t is None and (r.get('too_fast') or r.get('left_state') == 'skipped'):
      skip_t, ahead['skip'] = r['t'], r.get('j_ahead')
    if range_skip_t is None and r.get('left_state') == 'skipped':
      range_skip_t, ahead['range_skip'] = r['t'], r.get('j_ahead')
    if missed_t is None and r.get('turn_missed'):
      missed_t = r['t']
    if gas_t is None and engaged and r.get('gas'):
      gas_t = r['t']
  return {'handbacks': first['handback'], 'control_alerts': first['alert'], 'skip_t': skip_t, 'range_skip_t': range_skip_t,
          'skip_ahead_m': ahead.get('skip'), 'range_skip_ahead_m': ahead.get('range_skip'), 'turn_missed_t': missed_t,
          'gas_t': gas_t,
          'handback_det_t': det_t, 'handback_det_age': det_age,
          'after_range_skip': after_range_skip(rows, range_skip_t), 'missed': missed_turn(rows, missed_t),
          'held': {k: held_after(rows, t) for k, t in first['handback'].items() if k in TAKE_CONTROL_PROMPTS},
          'stood_at': {k: stood_at(rows, t) for k, t in first['handback'].items() if k in FULL_STOP_HANDBACKS},
          'go_before': {k: go_before(rows, t) for k, t in first['handback'].items() if k in GO_TIMED},
          'stand_before': {k: stand_before(rows, t) for k, t in first['handback'].items() if k in STALL_HANDBACKS},
          'desire_after_skip_t': next((r['t'] for r in rows if skip_t is not None and r['t'] >= skip_t and r.get('desire') in TURN_DESIRES), None),
          'prompts_after_gas': sorted({k for k, t in first['handback'].items() if gas_t is not None and t >= gas_t})}


def after_range_skip(rows, t: float | None) -> dict | None:
  """After the left's range skip at t: the first TurnPilot hold near the junction ({t, reason, j_dist}, NEAR_JUNCTION_M),
  when the car first had the junction behind it (passed_t; None: never, or no junction in the rows: has_j), and the first
  row from then on with a navRoute other than the one the car came up to the junction on (rerouted_t). None without the
  skip."""
  if t is None:
    return None
  after = [r for r in rows if r['t'] >= t]
  hold = next(({'t': r['t'], 'reason': r.get('reason', ''), 'j_dist': r.get('j_dist')} for r in after
               if r.get('state') == 'holding' and near_junction(r)), None)
  i = next((i for i, r in enumerate(after) if r.get('j_ahead') is not None and r['j_ahead'] <= 0.), None)
  passed_t = None if i is None else after[i]['t']
  base = None if i is None else after[max(i - 1, 0)].get('route')
  rerouted_t = None if i is None else next((r['t'] for r in after[i:] if r.get('route') != base), None)
  return {'hold': hold, 'passed_t': passed_t, 'rerouted_t': rerouted_t, 'has_j': any(r.get('j_ahead') is not None for r in after)}


def _heading_turn(rows, a: int, b: int) -> float | None:
  """The most the car's heading turned either way from row a over rows a..b, in degrees; None without headings."""
  hs = [r['heading'] for r in rows[a:b + 1] if r.get('heading') is not None]
  if not hs or rows[a].get('heading') is None:
    return None
  return round(float(np.degrees(np.abs(np.unwrap(np.array(hs)) - hs[0]).max())), 1)


def missed_turn(rows, t: float | None) -> dict | None:
  """The first turnMissed, at t (TURN_MISSED_S): the junction's distance ahead then (ahead_m), when the car first had the
  junction behind it (passed_t; has_j: the rows have the junction), its go (from_t: the last stand before the pass or the
  miss, else its first row within NEAR_JUNCTION_M before the junction), and how far its heading had turned from there when
  the miss came (turned_at_deg) and through its alert (turned_through_deg; None without headings). None without one."""
  if t is None:
    return None
  i_m = next(i for i, r in enumerate(rows) if r['t'] >= t)
  end = i_m
  while end + 1 < len(rows) and rows[end + 1].get('turn_missed'):
    end += 1
  i_p = next((i for i, r in enumerate(rows) if r.get('j_ahead') is not None and r['j_ahead'] <= 0.), None)
  ref = i_m if i_p is None else min(i_p, i_m)
  i0 = next((i for i in range(ref, -1, -1) if rows[i]['v'] < STOPPED_V), None)
  if i0 is None:
    i0 = next((i for i, r in enumerate(rows) if r.get('j_dist') is not None and r['j_dist'] <= NEAR_JUNCTION_M and
               (r.get('j_ahead') or 0.) > 0.), 0)
  return {'t': t, 'ahead_m': rows[i_m].get('j_ahead'), 'passed_t': None if i_p is None else rows[i_p]['t'],
          'has_j': any(r.get('j_ahead') is not None for r in rows), 'from_t': rows[i0]['t'],
          'turned_at_deg': _heading_turn(rows, i0, i_m), 'turned_through_deg': _heading_turn(rows, i0, end)}


def missed_problems(m: dict | None) -> list[str]:
  """Why the turnMissed seen isn't its shot's (missed_turn): before the pass or too long after it, or with the turn taken."""
  if m is None:
    return []
  out = []
  if not m['has_j']:
    out.append("no junction in the rows: turnMissed's timing at the pass was not checked")
  elif m['passed_t'] is None or m['t'] < m['passed_t']:
    ahead = f"with the junction {m['ahead_m']:.0f} m ahead" if m['ahead_m'] is not None else "with the junction ahead"
    out.append(f"turnMissed at t={m['t']:.1f} s {ahead}: before the car passed it")
  elif m['t'] - m['passed_t'] > TURN_MISSED_S + ROW_TOL_S:
    out.append(f"turnMissed {m['t'] - m['passed_t']:.1f} s after the car passed the junction (t={m['passed_t']:.1f} s), over {TURN_MISSED_S} s")
  if m['turned_at_deg'] is None:
    out.append("no headings in the rows: whether the car turned before turnMissed was not checked")
  else:
    if m['turned_at_deg'] >= MISSED_TURN_DEG + HEADING_TOL_DEG:
      out.append(f"the car had turned {m['turned_at_deg']:.0f} deg from its go at t={m['from_t']:.1f} s when turnMissed came: a miss " +
                 f"is under turnpilotd's {MISSED_TURN_DEG:.0f} deg")
    if m['turned_through_deg'] >= TURN_TAKEN_DEG:
      out.append(f"the car turned {m['turned_through_deg']:.0f} deg from its go at t={m['from_t']:.1f} s through the turnMissed " +
                 "alert: the turn was taken, not missed")
  return out


def range_skip_problems(a: dict | None) -> list[str]:
  """Why the left's range skip met isn't its shot's (after_range_skip): a hold at the line after it, no reroute past it."""
  if a is None:
    return []
  out = []
  if (h := a['hold']) is not None:
    where = '' if h['j_dist'] is None else f", {h['j_dist']:.0f} m from the junction,"
    out.append(f"TurnPilot held the car at t={h['t']:.1f} s ({h['reason']}){where} after the range skip: no hold at the line")
  if not a['has_j']:
    out.append("no junction in the rows after the range skip: its reroute was not checked")
  elif a['passed_t'] is None:
    out.append("never passed the skipped junction: no reroute seen")
  elif a['rerouted_t'] is None:
    out.append(f"no reroute after passing the skipped junction at t={a['passed_t']:.1f} s (one navRoute to the run's end)")
  return out


def drop_problems(t: float, det_t: float | None, drop: dict | None, det_age: float | None = None) -> list[str]:
  """Why the detectorStopped handback at t isn't the drop's (DROP_HANDBACK_S): det_t, the detector's last detections
  before it, and det_age, their age at the handback's row (None: the rows don't say); drop, the runner's (summarize)."""
  drop_t = None if drop is None else drop.get('t')
  if drop is not None and drop_t is None:
    at = drop.get('at')
    return [f"the detection drop asked for at t={at:.1f} s never came (the run ended first)" if at is not None else
            "the detection drop never came (the run ended first)"]
  out = []
  if det_age is not None and det_age < DETECTOR_DEADLINE_S - ROW_TOL_S - 1e-6:
    out.append(f"handed back detectorStopped with the detector's last detections {det_age:.2f} s old, under turnpilotd's " +
               f"{DETECTOR_DEADLINE_S} s deadline: the detector hadn't gone silent")
  if det_t is not None:
    if t - det_t > DROP_HANDBACK_S + 1e-6:
      out.append(f"handed back {t - det_t:.1f} s after the detector's last detections (t={det_t:.1f} s), over {DROP_HANDBACK_S} s")
    if drop_t is not None and det_t < drop_t - DROP_BEFORE_TOL_S:
      out.append(f"the detector went silent at t={det_t:.1f} s, before the drop at t={drop_t:.1f} s")
  elif drop_t is not None:
    if t < drop_t:
      out.append(f"handed back detectorStopped at t={t:.1f} s, before the drop at t={drop_t:.1f} s")
    elif t - drop_t > DROP_HANDBACK_S + DETECTOR_PARAM_CHECK_S + 1e-6:
      out.append(f"handed back {t - drop_t:.1f} s after the drop at t={drop_t:.1f} s, over {DROP_HANDBACK_S + DETECTOR_PARAM_CHECK_S:.1f} s " +
                 f"({DROP_HANDBACK_S} s, and up to {DETECTOR_PARAM_CHECK_S} s for the detector to read the drop's param)")
  else:
    out.append(f"no detector times in the rows and no drop: the detectorStopped handback's {DROP_HANDBACK_S} s was not checked")
  return out


def fault_event(arg: str) -> str:
  """The selfdrived event a fault handback of kind arg raises (handback.EVENTS): its soft disable ends the drive."""
  names = {v: k for k, v in EventName.schema.enumerants.items()}
  return names[HANDBACK_EVENTS[Handback.schema.enumerants[arg]]]


def handback_problems(name: str, arg: str, t: float, facts: dict, res: dict) -> list[str]:
  """Why a handback of kind arg first seen at t isn't its shot's: another handback before it; a take-control handback the
  car wasn't held after (held_after), the left's withdrawn go not held short of the junction's conflict zone
  (SHORT_OF_LANES), the stop sign's without its full stop; a fault's disengagement not its soft disable; the detection
  drop's not within DROP_HANDBACK_S of the detector going silent after the drop (drop_problems)."""
  out = [f"handed back {k} at t={tk:.1f} s, before the expected handback {arg}"
         for k, tk in sorted(facts['handbacks'].items(), key=lambda kv: kv[1]) if k != arg and tk < t]
  h = facts.get('held', {}).get(arg)
  if h is not None and arg in SHORT_OF_LANES and name.startswith('unprotected_left'):
    if h['j_dist'] is None:
      out.append(f"no junction distance in the rows at the {arg} handback: 'short of the oncoming lanes' was not checked")
    elif h['j_dist'] <= CONFLICT_M:
      out.append(f"handed back {arg} {h['j_dist']:.1f} m from the junction, inside its {CONFLICT_M:.0f} m conflict zone: not short " +
                 "of the oncoming lanes")
    else:
      if h['min_j_dist'] <= CONFLICT_M:
        out.append(f"came within {h['min_j_dist']:.1f} m of the junction engaged after the {arg} handback, into its " +
                   f"{CONFLICT_M:.0f} m conflict zone: not held short of the oncoming lanes")
      if h['went_m'] > h['zone_m']:
        out.append(f"moved {h['went_m']:.1f} m engaged after the {arg} handback, past the {h['zone_m']:.1f} m to the junction's " +
                   f"{CONFLICT_M:.0f} m conflict zone")
  if arg in GO_TIMED:
    g = facts.get('go_before', {}).get(arg)
    if g is None:
      out.append(f"no go of the left (left_state go) before the {arg} handback")
    elif g['after_s'] > g['bound_s'] + ROW_TOL_S:
      lift = {True: "with plannerd's lift", False: "without the lift: no goCertified", None: "the lift not in the rows"}[g['lift']]
      out.append(f"handed back {arg} {g['after_s']:.1f} s after the left's go at t={g['go_t']:.1f} s, over {g['bound_s']} s ({lift})")
  if arg in STALL_HANDBACKS:
    st = facts.get('stand_before', {}).get(arg)
    if st is not None and not STALL_PROMPT_S - ROW_TOL_S <= st['stood_s'] <= STALL_PROMPT_S + STALL_LATE_S:
      out.append(f"handed back {arg} {st['stood_s']:.1f} s into the car's stand with nothing holding it (from t={st['from_t']:.1f} s), " +
                 f"not {STALL_PROMPT_S} s")
  if h is not None and h['went_m'] > h['allowed_m']:
    out.append(f"moved {h['went_m']:.1f} m engaged after the {arg} handback (up to {h['v_max']:.1f} m/s), past the " +
               f"{h['allowed_m']:.1f} m a stop from {h['v0']:.1f} m/s allows: not held")
  if arg in FULL_STOP_HANDBACKS and not facts.get('stood_at', {}).get(arg, False):
    out.append(f"handed back {arg} at t={t:.1f} s at {h['v0'] if h else 0.:.1f} m/s: no full stop before it")
  if arg in FAULT_HANDBACKS:
    after = [d for d in res['disengagements'] if d['t'] >= t]
    if not after:
      out.append(f"no disengagement after the {arg} handback: its soft disable never ended the drive")
    elif SOFT_DISABLE not in after[0]['cause']:
      who = "the driver's" if after[0]['cause'].startswith('driver ') else 'for something else'
      out.append(f"the disengagement after the {arg} handback was {who} ({after[0]['cause']}), not its soft disable")
    elif not after[0]['cause'].startswith(fault_event(arg) + SOFT_DISABLE):
      out.append(f"the disengagement after the {arg} handback was another event's soft disable ({after[0]['cause']}), not " +
                 f"{fault_event(arg)}'s")
  if arg == 'detectorStopped':
    out += drop_problems(t, facts['handback_det_t'].get(arg), res.get('detection_drop'), facts.get('handback_det_age', {}).get(arg))
  return out


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
      problems = [] if t is None else handback_problems(name, arg, t, facts, res)
      met = t is not None and not problems
      if t is None:
        extra.append(f"expected handback {arg}: none came")
        drop = res.get('detection_drop')
        if arg == 'detectorStopped' and drop is not None and drop.get('t') is None:
          extra += drop_problems(0., None, drop)  # the drop itself never came
      elif problems:
        extra += problems  # seen, but not the shot: it explains nothing
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
      range_skip = name.startswith('unprotected_left')  # its range skip (EXPECTS)
      t = facts['range_skip_t'] if range_skip else facts['skip_t']
      a = facts['range_skip_ahead_m' if range_skip else 'skip_ahead_m']
      late = t is not None and a is not None and a <= 0.
      problems = range_skip_problems(facts.get('after_range_skip')) if range_skip and t is not None and not late else []
      met = t is not None and not late and not problems
      if late:
        extra.append(f"the skip came at t={t:.1f} s with the junction {-a:.0f} m behind the car: its alert is before the junction")
      elif problems:
        extra += problems  # seen, but not the shot: it explains nothing
      elif not met and range_skip and facts['skip_t'] is not None:
        extra.append("expected skip: the left never skipped for range (unprotectedLeft.state skipped); tooFastToTurn at " +
                     f"t={facts['skip_t']:.1f} s is another skip (tooLate, laneUnreachable or tooFast)")
      elif not met:
        extra.append("expected skip: no turn skipped (tooFastToTurn)")
      else:
        explain(e, ('left_wait', 'left_go', 'not_arrived', 'maneuvers'))
        if facts['desire_after_skip_t'] is not None:
          extra.append(f"a turn desire after the skip, at t={facts['desire_after_skip_t']:.1f} s")
    elif kind == 'turnMissed':
      t = facts['turn_missed_t']
      problems = missed_problems(facts.get('missed'))
      met = t is not None and not problems
      if t is None:
        extra.append("expected turnMissed: none came (no turnMissed)")
      elif problems:
        extra += problems  # seen, but not the shot: it explains nothing
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
              stop_line_m: float | None = None, drop: dict | None = None) -> dict:
  """road: catalog.road_model's roads around the run, for the oncoming-lane check (rows with positions need one);
  junction_control: the scenario junction's own mapped control (catalog.junction_control's kind: 'signal', 'stop',
  'none'; None: not known), recorded with the result (a signal there no longer protects a left: map_controls);
  expect: the run's expectations (EXPECTS); stop_line_m: m before the junction of its own stop line (the plan's notes);
  drop: the runner's detection drop ({t: s since engagement, ...}; {t: None, at: s} asked for and never set; None: none)."""
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
    'tp_accel': tp_accel(rows),
    'detection_drop': drop,
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
  acc = res.get('tp_accel') or {}
  if acc.get('min') is not None:
    line += f", TurnPilot accel min {acc['min']} m/s^2 (planner {acc.get('a_target_min')})"
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
                                                                     stop_line_m=notes.get('stop_line_m'), drop=run.get('detection_drop'))))


if __name__ == "__main__":
  main()
