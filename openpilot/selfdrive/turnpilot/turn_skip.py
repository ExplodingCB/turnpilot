"""Too fast to turn: skip the turn rather than brake hard for it or take it at speed.

Kellen's rule (2026-10-27): during the approach to a junction turn, keep checking that the car can still get down to
a safe speed for the turn's real angle by the junction, braking comfortably. Once it can't, the turn is skipped for
good: its desire never goes out, its speed cap and the scene and lane logic for it are dropped (no hard braking for a
turn), the driver gets the turnpilotTooFastToTurn alert, and navd reroutes once the car is past the junction. The model
drives on as it would with no route.

The check, from the turn's angle (bearing_after less bearing_before) and the side of the road it turns across:
  v_safe = sqrt(A_LAT_SAFE * R),  R = corner tangent / tan(angle / 2)    (turn_speed.corner_tangent: 8 m to the curb side,
                                                                           12 m across the road)
  feasible  <=>  v_ego^2 <= v_safe^2 + 2 * A_COMFORT * max(d - v_ego * T_LAG, 0)
A_COMFORT is the hardest the turn speed cap may brake (hooks.SPEED_CAP_ACCEL_MIN, 2 m/s^2), so "feasible" means the
cap can get there within its own limit, and T_LAG is the planner's time constant from cap to accel (hooks.SPEED_CAP_TAU,
1 s), the distance covered before braking builds. A_LAT_SAFE is the turn cap's 2 m/s^2. At 90 degrees to the curb side
that is v_safe 4.0 m/s, and the latest feasible speeds are 11.4 m/s 40 m out, 18.5 m/s 100 m out and 22.9 m/s 150 m out.
The cap's own curve (1.5 m/s^2, leading the car by 2 s) stays under this one, so a car that follows the cap never skips.

The check only gates the desire's pulses. A turn is skipped after SKIP_CONFIRM_FRAMES infeasible frames in a row, so one
dead-reckoning jump doesn't skip it; meanwhile no pulse starts (feasible_now gates it). Once the desire is out the turn
is never skipped and nothing is cancelled: a turn the model has started planning is finished (cancelling mid-turn is how
you crash). Near the junction the check fails for a car rolling at the model's own corner speed, which is one more
reason it can't cancel. It still gates the repeat pulses and a fresh pulse on re-engaging (the old 10 m/s gate's job):
a car the driver sped up isn't sent into the turn again from road speed.
Nor does a turn's first pulse go out late: gate 1's model took a turn from a pulse 40 m out and not from 10-20 m, and a
pulse at or past the junction turns the car into the curb or the oncoming lanes. A turn whose desire hasn't gone out by
its deadline (first_pulse_latest) while openpilot is steering is skipped as above, reason tooLate: one that comes up
that close (navd moves past a bend or a straight-on step only 10 m beyond it), or one openpilot engages for that late. The
deadline follows the pulse's own timing (desire_timing.turn_lead, 2026-11-06), never ahead of it: a far-side turn's first
pulse goes FAR_LEAD_S before the junction (40 m at turning speed, 15 m at a crawl), so its deadline is FIRST_PULSE_S
before it (MIN_FIRST_PULSE_M, 25 m, at turning speed; FAR_FIRST_PULSE_MIN_M at a crawl); a curb-side turn's one pulse goes at
CURB_TURN_PULSE_M (8 m), so its deadline is CURB_FIRST_PULSE_M (5 m: from there the model, starting 0.3-1.5 s after the
edge, would start round at or past the corner's apex), and a curb-side turn the car is too fast for by then is the tooFast
skip's (can_make_turn), never a pulse at speed. It is also the bound on that pulse's wait while the model steers in at a
turn's scale (desire_timing.py, 2026-11-06: STEER_IN_TURN_CURV, never a road's bend or a lean's): one still waiting there,
the car not following the model round (under desire_timing.OWN_FOLLOW_DEG since its steer-in), is skipped here out loud,
never pulsed late into a model mid-steer; one the model has the car turning on by then is its own, started, so never
skipped. The deadline doesn't come short of a map stop the car will stop at between it and the junction (stop_d: a stop
sign not released, a red), since the turn pulses from that line or once past it: a decider's right braking to a line 3-4 m
from navd's node, its pulse held to the go, was skipped tooLate at about 4.9 m rolling at 1.9-2.8 m/s (the 2026-11-06
review's probe). Past the stop (released, the green) the deadline holds again.
A turn whose pulse waits past an opening on its side it may not enter (desire_timing.wrong_way_wait: a one-way toward it)
has its deadline moved in to that point less WRONG_WAY_GRACE_M
(desire_timing.first_pulse_by): a later pulse beats a turn into the one-way, and a turn the model won't take from there is
missed and rerouted.
The deadline never fires at or past the junction (distance 0 or less): every such skip on the road was a phantom for a turn
already made (1e 8:36, -12.1 m: a right finished 4 s before, after a one-message navd blip; 1e 9:43, -4.8 m: a stale reroute's
first step; route_follower.py now holds both). But no first pulse goes there either (desire_timing.py: by the nearest, a late
pulse is the dangerous one), so a turn whose nearest CROSSES its junction with openpilot steering and no first pulse out (the
last update short of it, this one past it, the same maneuver) is skipped tooLate there and then, out loud, never missed in
silence: on 1e's 91 deg right at 8:32 (one follower replayed over the whole drive) the furthest reading was still 8.8 m out
with the nearest at the junction, so its pulse by the furthest would never have gone. Every turn is short of its junction
or past it (the crossing, the split below, the deadline's own bound at the junction) by the nearest reading, one reading
alone included, as its first pulse is (desire_timing.py): one 8-10 m short 20-45 m out, which the nearest keeps to the end,
has a right skipped here 7.7-9.8 m short of its junction with every later reading saying it is ahead (HEAD, pulsing at
40 m, took it), its corner cap kept (turnpilotd keep_hand_cap). Kept on purpose (2026-11-06 review): those readings are
also one TRUE reading inside a lasting long bias of 8-10 m, and by the second-nearest (tried that round) that right was
pulsed 0.3-2.3 m past its junction with nothing said, where by the nearest it is skipped here at its junction, out loud.
A turn first seen or first steered for
past its junction (the phantoms, a re-engage in the junction after the driver's own turn) is not, unless it is first seen
with its readings split across the junction, the nearest past it and the furthest short of
it: then no first pulse can go by it either, and it is skipped tooLate at once. That is a reroute's same junction carrying the old span over with
one short outlier as its nearest (route_follower.py no longer passes such a turn from the start, in silence, for its nearest
alone), or a turn just past a bend taken up that way, already announced skipped (ahead_skipped, below). The phantoms read
behind by every reading are passed from the start by route_follower.py and never come here. Nor for a car standing or
crawling (under STANDING_PULSE_V) STANDING_PULSE_MIN_M or more short of it: engaged
at a standstill 12.3 m from an end-of-road left (1e 2:07.7, v 0.39) and 9.9 m from a right (1d 4:54.0, v 0.0), both were
skipped at once, 'Too Fast to Turn' at 0 mph, and both drives ended in a disengagement. Such a turn pulses when its lead
comes (at once for a far-side one inside its 15 m floor, at CURB_TURN_PULSE_M for a curb-side one), a turn from the line
as a hold's go already is.
A curb-side turn is judged by the reading its one pulse goes by (desire_timing.pulse_distance; update's far and last: the
furthest of navd's readings of its junction, or the latest CURB_TURN_PULSE_M - CURB_LATEST_PULSE_M further in): its
feasibility (can_make_turn), its tooLate deadline (5 m by the furthest, 3 m by the latest) and the standing check. By the
nearest, a reading 20 m short at 28 m out at the cap's 5.4 m/s skipped the right tooFast (test_turnpilotd's GPS bias case)
while the pulse would wait for the true 8 m anyway, and a spread of 3 m or more where the nearest comes inside 8 m (3 rights
of 8 in 1d and 1e) called the right too late before its pulse was due. The speed cap still brakes for the nearest
(turn_speed.py), so the car is slow by then whichever reading is right. Not given those readings (turnpilotd passing neither),
the nearest's deadline holds for a curb-side turn only until openpilot has steered for it, without a break, from outside it:
a turn that comes up inside it or is engaged for there is still skipped, and one approached from outside it is its pulse's
to time (DesireTiming has the furthest), its nearest crossing the junction unpulsed skipped out loud as above.
With openpilot not steering no desire goes out at all, and the driver turning there isn't skipped under them; under
TurnPilotAlwaysDrive neither is a turn too fast (tooFast, unprotectedStop: ALWAYS_DRIVE.md R4b), its count starting once
openpilot steers, so a re-engage inside its stopping distance from a speed it can be made at drives it. A pulse
again for a turn whose desire did go out (on the entry, re-engaging, pulling away) isn't a first pulse.
A turn just past a bend or a straight-on step (route_follower.py: RouteFollower.ahead) becomes the maneuver only once
the car is NAVD_ADVANCE_M past that step, at whatever distance it then has: one under MIN_FIRST_PULSE_M away already is
certain to be skipped tooLate then, which for a turn 10 m or less past the step is at or past its junction. So the skip
is announced (SkipState.ahead_skipped) from the moment the turn ahead is that close, before the junction.
The autonomous unprotected left (unprotected_left.py) sends no pulse before its hold line, its go being its first
(pulse_at_go): it is never skipped tooLate. Nor is a turn at a stop sign the stop-sign decider acts for, while the car
stands: but a curb-side one's go is its first pulse only from a line within desire_timing's CURB_TURN_PULSE_M +
CURB_GO_SLACK_M, and from one further out its pulse waits for the pulse point as the car rolls (and waits there while the
model steers in: desire_timing.py), so rolling (STANDING_PULSE_V or faster) it is judged by its deadline as any curb-side
turn, and its nearest crossing the junction unpulsed is skipped out loud however slow: pulse_at_go blocked both, and such a
right with no pulse out was lost in silence. (Logs before 2026-11-06 can show skipReason unprotectedRange: an autonomous left
whose required clear range was past the detector's proven range, skipped as soon as it was the maneuver and rerouted. The
lefts-to-the-line work drives every left to its line and judges it there; the skip is gone.)
An unprotected turn (map_controls.py) is taken from a stop at its hold line, so it is also too fast when that stop can't
be made at the map stops' -2.0 m/s^2 when it is first seen (stop_infeasible: turnpilotd, a stop not yet engaged): skipped
the same way, after the same confirming frames, never braked for at the floor.
Only junction turns (route_follower.TURN) are checked. A keep (slight turns, forks, ramps) is taken at speed by the
model, and its desire goes at any speed.
"""
import math
from dataclasses import dataclass

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot.desire_timing import (CURB_FIRST_PULSE_M, FAR_LEAD_S, TURN_DESIRE_LEAD_M, curb_side_turn, first_pulse_by,
                                                         pulse_distance)
from openpilot.selfdrive.turnpilot.hooks import SPEED_CAP_ACCEL_MIN, SPEED_CAP_TAU
from openpilot.selfdrive.turnpilot.route_follower import LEFT, TURN
from openpilot.selfdrive.turnpilot.turn_speed import A_LAT_MAX, V_TURN_MAX, corner_tangent

A_LAT_SAFE = A_LAT_MAX           # m/s^2 through the corner
A_COMFORT = -SPEED_CAP_ACCEL_MIN  # m/s^2: 2.0, the hardest the turn speed cap brakes
T_LAG = SPEED_CAP_TAU            # s before the braking builds
SKIP_CONFIRM_FRAMES = 5          # 0.25 s at 20 Hz
MIN_FIRST_PULSE_M = 25.          # m: a far-side turn's first pulse goes no later than this before the junction at turning speed...
FIRST_PULSE_S = FAR_LEAD_S * MIN_FIRST_PULSE_M / TURN_DESIRE_LEAD_M  # ...3.75 s before it slower (its lead is FAR_LEAD_S)...
FAR_FIRST_PULSE_MIN_M = 10.      # ...and never later than this (its lead's floor is 15 m)
# CURB_FIRST_PULSE_M (desire_timing's, 5 m): a curb-side turn's one pulse (CURB_TURN_PULSE_M, 8 m) no later than this
STANDING_PULSE_V = 1.5           # m/s: a car under this (standing, crawling)...
STANDING_PULSE_MIN_M = 5.        # ...this far or more short of the junction is never too late: it pulses from where it is
STOP_BEFORE_TURN_M = 30.         # a map stop sign or signal the car stops at this close before the junction: taken from a stop


def first_pulse_latest(maneuver, v_ego: float, right_hand_traffic: bool = True) -> float:
  """m before the junction a turn's first pulse must have gone by, at v_ego (module docstring): CURB_FIRST_PULSE_M for a
  curb-side turn; for a far-side one FIRST_PULSE_S at v_ego, MIN_FIRST_PULSE_M at most and FAR_FIRST_PULSE_MIN_M at least,
  always short of where its lead (desire_timing.turn_lead) has it pulse."""
  if curb_side_turn(maneuver, right_hand_traffic):
    return CURB_FIRST_PULSE_M
  return min(max(FIRST_PULSE_S * max(v_ego, 0.), FAR_FIRST_PULSE_MIN_M), MIN_FIRST_PULSE_M)


def too_late(maneuver, distance: float, v_ego: float, right_hand_traffic: bool = True, far: float | None = None,
             last: float | None = None) -> bool:
  """A turn whose first pulse hasn't gone is too late for one this close to its junction (module docstring): short of its
  deadline (first_pulse_latest, moved in past an opening it may not enter: desire_timing.first_pulse_by), never at or past
  the junction (the nearest reading), nor for a car under STANDING_PULSE_V STANDING_PULSE_MIN_M or more short of it. A
  curb-side turn's deadline and standing check are by the reading its pulse goes by (desire_timing.pulse_distance: far and
  last)."""
  d = pulse_distance(maneuver, distance, far, right_hand_traffic, last)
  return 0. < distance and d < first_pulse_by(maneuver, first_pulse_latest(maneuver, v_ego, right_hand_traffic)) and \
    not (v_ego < STANDING_PULSE_V and d >= STANDING_PULSE_MIN_M)


def safe_turn_speed(angle_deg: float, curb_side: bool) -> float:
  """m/s to take a junction turn of this unsigned angle, from the corner's radius."""
  half = math.radians(min(max(angle_deg, 1.), 179.)) / 2
  return min(math.sqrt(A_LAT_SAFE * corner_tangent(curb_side) / math.tan(half)), V_TURN_MAX)


def can_make_turn(v_ego: float, distance: float, angle_deg: float, curb_side: bool) -> bool:
  """Whether braking at A_COMFORT after T_LAG gets the car to safe_turn_speed by the junction, distance m ahead."""
  v_ego = max(v_ego, 0.)
  v_safe = safe_turn_speed(angle_deg, curb_side)
  return v_ego ** 2 <= v_safe ** 2 + 2. * A_COMFORT * max(distance - v_ego * T_LAG, 0.)


@dataclass
class SkipState:
  skipped: bool       # this maneuver is skipped for good
  feasible_now: bool  # a desire may start this frame
  ahead_skipped: bool = False  # the turn after this NONE step (RouteFollower.ahead) will be skipped tooLate: announced now


class TurnSkip:
  def __init__(self):
    self.maneuver_id = -1
    self.always_drive = False  # TurnPilotAlwaysDrive, handed over every frame (turnpilotd.hand_always_drive; ALWAYS_DRIVE.md section 1)
    self.skipped = False
    self.reason = ''  # why it was skipped
    self._infeasible = 0
    self._ahead_logged = -1  # the maneuver_id of the NONE step whose turn ahead was announced skipped
    self._short = False      # last update: openpilot steering, this turn's first pulse not out, the car short of its junction
    self._outside = False    # openpilot has steered without a break since this turn was outside its tooLate deadline
    self._first = False      # the next update is this maneuver's first

  def skip(self, maneuver, distance: float, maneuver_id: int, v_ego: float, reason: str, log: bool = True) -> None:
    """Skip this maneuver for good now, for reason: as when it is too fast to make (turnpilotd: a turn whose desire was
    dropped while TurnPilot had handed back, with too little road left for a fresh pulse to be taken; too late for its
    first pulse). log False: the same junction's skip carried to a new route's id (turnpilotd.carry_skip logs it), one
    turnpilot_turn_skipped per junction."""
    self.maneuver_id, self.skipped, self._infeasible, self.reason = maneuver_id, True, 0, reason
    if log:
      cloudlog.event("turnpilot_turn_skipped", error=True, reason=reason, type=maneuver.type, modifier=maneuver.modifier,
                     angle_deg=round(maneuver.angle_deg, 1), geometry=maneuver.geometry, distance=round(distance, 1),
                     v_ego=round(v_ego, 2))

  def adopt(self, maneuver_id: int) -> None:
    """The skipped turn taken by the car after all (turnpilotd.adopt_skipped: a curb-side turn skipped tooLate that the car has
    turned TURN_STARTED_DEG into since): no longer skipped; its desire counts as out from here (DesireTiming.adopt), so nothing
    judges it again."""
    if self.maneuver_id == maneuver_id and self.skipped:
      self.skipped, self.reason, self._infeasible, self._short = False, '', 0, False

  def update(self, maneuver, distance: float, maneuver_id: int, v_ego: float, desire_out: bool,
             right_hand_traffic: bool = True, lat_active: bool = True, ahead=None, ahead_distance: float = float('inf'),
             stop_d: float | None = None, stop_infeasible: bool = False, pulse_at_go: bool = False,
             far: float | None = None, last: float | None = None) -> SkipState:
    """maneuver: RouteFollower.upcoming; desire_out: this maneuver's desire has gone out (DesireTiming.started);
    lat_active: carControl.latActive; ahead, ahead_distance: RouteFollower.ahead past a NONE step, and the distance to it;
    stop_d: m to the map stop sign or signal the car will stop at next (MapControlPolicy.stop_ahead), or None. One within
    STOP_BEFORE_TURN_M before the junction means the turn is taken from a stop: never too fast (the stop itself brakes at
    no more than -2.0, and says so when it can't make it), and one anywhere between the car and the junction, never too
    late short of it (module docstring). stop_infeasible: the turn is unprotected and its hold-line stop,
    not yet engaged, can't be made at -2.0 (module docstring). pulse_at_go: the turn's first pulse is its hold line's go (the
    autonomous unprotected left: no pulse on its approach; the stop-sign decider's turn), so it is never too late for one; a
    curb-side one only while the car stands (module docstring). far: the furthest reading of the junction (distance plus
    RouteFollower.junction_spread), and last: navd's latest confirmed (RouteFollower.junction_latest), which a curb-side turn is judged
    by as its pulse goes by them (desire_timing.pulse_distance; module docstring); the nearest when not given. Every turn is
    short of its junction or past it, its crossing and its split, by distance, the nearest reading (module docstring). A turn
    first seen with the nearest past its junction and far short of it is skipped at once (module docstring)."""
    if maneuver_id != self.maneuver_id:
      self.maneuver_id, self.skipped, self._infeasible, self.reason = maneuver_id, False, 0, ''
      self._short = self._outside = False
      self._first = True
    first, self._first = self._first, False
    if self.skipped:
      return SkipState(True, False)
    if maneuver is None or maneuver.kind != TURN:
      # not this turn's update (none, a keep, the stop-sign ladder's other turn in its place): a crossing is seen only on
      # consecutive updates of the turn itself
      self._infeasible, self._short, self._outside = 0, False, False
      ahead_skipped = maneuver is not None and ahead is not None and ahead.kind == TURN and lat_active and \
        too_late(ahead, ahead_distance, v_ego, right_hand_traffic)
      if ahead_skipped and self._ahead_logged != maneuver_id:
        self._ahead_logged = maneuver_id
        cloudlog.event("turnpilot_turn_skipped", error=True, reason='tooLate', ahead=True, type=ahead.type,
                       modifier=ahead.modifier, angle_deg=round(ahead.angle_deg, 1), geometry=ahead.geometry,
                       distance=round(ahead_distance, 1), v_ego=round(v_ego, 2))
      return SkipState(False, True, ahead_skipped)
    # a turn whose pulse waits past an opening it may not enter (desire_timing.wrong_way_wait) is too late only past that;
    # never at or past the junction, nor standing or crawling short of it (module docstring)
    late = too_late(maneuver, distance, v_ego, right_hand_traffic, far, last)
    # ...nor short of a map stop the car will stop at between it and the junction (a stop sign, a red: stop_d): it pulses from
    # that line, or once past it, so a line inside the deadline (a narrow cross street's, 3-4 m from navd's node) is no reason
    # to skip it as the car brakes to it (module docstring)
    if late and stop_d is not None and 0. <= stop_d <= pulse_distance(maneuver, distance, far, right_hand_traffic, last):
      late = False
    curb_side = (maneuver.side == LEFT) != right_hand_traffic
    # without the reading a curb-side turn's pulse goes by (no far: turnpilotd not passing it), the nearest's deadline holds
    # only for a turn that comes up inside it or that openpilot steers for first there; one approached under openpilot from
    # outside it is its pulse's to time, and its nearest crossing the junction unpulsed is said below (module docstring)
    pulse_timed = far is None and curb_side and self._outside
    self._outside = lat_active and (self._outside or not late)
    # the nearest crossed the junction this update with openpilot steering and no first pulse out: none can go now
    # (desire_timing's guard), so the turn is skipped out loud, never missed in silence (module docstring)
    crossed = self._short and distance < 0.
    # ...or first seen that way with the furthest still short of the junction: its readings split across it (a reroute's same
    # junction carrying a short outlier as its nearest, route_follower.py; a turn just past a bend taken up), so no first pulse
    # can go by the nearest, and it is said at once (module docstring)
    split = first and far is not None and distance < 0. < far
    self._short = lat_active and not desire_out and distance >= 0.
    # the go is a curb-side turn's first pulse only standing at its line: rolling, its deadline holds, and its crossing does
    # whatever its go (module docstring)
    at_go = pulse_at_go and not (curb_side and v_ego >= STANDING_PULSE_V)
    if not desire_out and lat_active and ((late and not pulse_timed and not at_go) or
                                          ((crossed or split) and (curb_side or not pulse_at_go))):
      self.skip(maneuver, distance, maneuver_id, v_ego, 'tooLate')
      return SkipState(True, False)
    if stop_d is not None and 0. <= distance - stop_d <= STOP_BEFORE_TURN_M:
      self._infeasible = 0
      return SkipState(False, True)
    if can_make_turn(v_ego, pulse_distance(maneuver, distance, far, right_hand_traffic, last), maneuver.angle_deg, curb_side) and \
       not stop_infeasible:
      self._infeasible = 0
      return SkipState(False, True)
    if desire_out:  # committed: never skipped now, only no new pulse
      return SkipState(False, False)
    if self.always_drive and not lat_active:
      # TurnPilotAlwaysDrive (ALWAYS_DRIVE.md R4b): latched only while openpilot steers, as tooLate is; the driver's own
      # approach withholds the pulse, never skips the turn under them, and the count starts on the live speed once it steers
      self._infeasible = 0
      return SkipState(False, False)
    self._infeasible += 1
    if self._infeasible >= SKIP_CONFIRM_FRAMES:
      self.skipped, self.reason = True, 'unprotectedStop' if stop_infeasible else 'tooFast'
      cloudlog.event("turnpilot_turn_skipped", error=True, reason='unprotectedStop' if stop_infeasible else 'tooFast',
                     type=maneuver.type, modifier=maneuver.modifier,
                     angle_deg=round(maneuver.angle_deg, 1), geometry=maneuver.geometry, distance=round(distance, 1),
                     v_ego=round(v_ego, 2), v_safe=round(safe_turn_speed(maneuver.angle_deg, curb_side), 2))
    return SkipState(self.skipped, False)
