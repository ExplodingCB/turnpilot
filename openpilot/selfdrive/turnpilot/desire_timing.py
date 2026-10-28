"""Desire timing: which desire to send the driving model for the next maneuver, and when.

modeld feeds the model only the rising edge of a desire, so the desire is held through the maneuver and the
lead distance is the timing lever. Gate 1 (big model, 25 km/h, 2026-10-27) took 90-degree turns from a pulse
40 m before the junction and not from 10-20 m; a keep desire goes within max(2.5 s * v, 20 m). Once a desire
starts it is latched until the maneuver is passed (a keep: or the timeout; a turn is over only when the car has turned it
or is past it, never on a clock: a slow approach in a queue to a hold took longer, and the go then had no turn to pulse,
silently): navd's 1 Hz distance carries a few meters of noise, and dropping the desire whenever it read past the lead
again sent the model a new rising edge each time.
A keep toward a side where the model sees no lane line drove the car off the road there, so it is dropped.
The model forgets a pulse 132 frames after it (6.6 s at 20 Hz, its desire history; see below): where the slow
approach took longer than that, it planned the turn and then dropped it 10 m out, and a late retry pulse brought
the turn back at once. So a turn desire is pulsed again every REPULSE_S on the approach, and once more ENTRY_M
before the junction, so the last pulse is fresh for the whole turn: at 4 m/s a turn outlasts one pulse, and the
model let go of it halfway round. Crossing ENTRY_M within ENTRY_GAP_S of a repeat, the entry pulse goes once that gap is
past (it used to be used up then, and never went); a pulse that itself goes inside ENTRY_M is the entry pulse.
None once the car has turned TURN_STARTED_DEG (by its yaw rate): pulses during the turn turned the car on past the exit
road, into the curb. That holds for a repeat and one on engaging. One as the car
pulls away after a standstill (STANDSTILL_REARM_S) goes while it has turned less than STANDSTILL_REPULSE_MAX_DEG: waiting
in the junction for oncoming traffic, 15-25 degrees into a left, longer than the model's queue, the turn would lapse
(H2, below: with the pulse gone at 5-16 degrees the model gave up 3 turns of 3); further round (stopped halfway for a
pedestrian) none goes, and the model finishes it (H2: 9 of 10 from 25-42 degrees). While a hold has the car standing
(turnpilotd: shouldStop) the turn desire is none, its edge still in the model's queue, so the pulse on the hold's go is
a fresh edge in the very plan that lets the car go (BeamNG unprotected_left_visible: let go a frame or more before its
pulse, the car drove straight across the junction). A turn with no pulse before its hold (the autonomous unprotected left:
turnpilotd holds may_start off on its approach, so the car stands straight with the road camera up the oncoming approach)
takes its go as its first pulse, at once.
A repeat pulse is a short drop to none, long enough that modeld (unsynchronized, conflated subscriber) can't miss it.
The model keeps the rising edges of the last 132 frames (6.6 s at 20 Hz, the big model's state_desire_q), and
forgets a pulse when it leaves that queue. A pulse still in the queue once the car is round the corner turns it
on: at a 55 degree "left" the car turned 122-126 degrees into the curb, the last pulse 6.6 s from leaving.
So once the turn is done, CLEAR_DESIRE goes out and modeld drops the turn desires from the queue: at the
junction (navd past it) once the car has turned CLEAR_MIN_DEG, or anywhere within CLEAR_MARGIN_DEG of the turn's
angle (80 degrees for a 90 degree left or right), but not before CLEAR_MIN_FRACTION of it (a 40 degree left at 30).
The angle is the junction's real one from Valhalla's bearings (route_follower.py), else the modifier's nominal one.
Short of CLEAR_MIN_DEG it waits: in H2's single-pulse runs, with the pulse gone at 5-16 degrees the model gave up
3 turns of 3, and at 25-42 degrees it finished 9 of 10 on its own (the one it gave up was at 32).
No turn pulse starts while the car is too fast to make the turn (turn_skip.py, may_start false): the turn speed cap
has the car at 6.9 m/s by the lead, and faster than the check allows something else holds the speed (the driver's
foot), so the model would turn from road speed. Too fast for long, the turn is skipped and no pulse goes at all. A pulse
already out is held and never cancelled, since dropping it and sending it again would be a new rising edge, and a turn
under way is finished. A keep goes at any speed.
Nor does any desire go out while openpilot isn't steering (carControl.latActive): modeld would not feed it to the
model, and a pulse, repeat or timeout spent then is lost. Once openpilot engages, a fresh pulse goes, unless the driver
has turned the car TURN_STARTED_DEG toward a turn meanwhile.
When a new route replaces the maneuver (RouteFollower.replaced), a turn pulse the car hasn't started turning on is
cleared at once: the old junction may be just ahead, and the pulse would turn the car there.
"""
import math

from openpilot.cereal import log
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE
from openpilot.selfdrive.turnpilot.route_follower import KEEP, LEFT, TURN

Desire = log.Desire

TURN_DESIRE_LEAD_M = 40.      # gate 1's best lead on the big model
KEEP_DESIRE_LEAD_S = 2.5
KEEP_DESIRE_LEAD_MIN_M = 20.
KEEP_DESIRE_PAST_M = 5.       # a keep desire lasts this far past the maneuver point
DESIRE_TIMEOUT_S = 15.        # moving time a keep desire may be held before it is dropped for good
REPULSE_S = 3.5               # a turn desire pulses again this often on the approach...
ENTRY_M = 10.                 # ...and once more this far before the junction (not within ENTRY_GAP_S of the last)...
ENTRY_GAP_S = 1.5
TURN_STARTED_DEG = 15.        # ...until the car has turned this far toward the maneuver
STANDSTILL_REPULSE_MAX_DEG = 25.  # a pulse as the car pulls away after a standstill goes while it has turned less than this
MAX_PULSES = 5
PULSE_GAP_FRAMES = 3          # frames of none before a repeat pulse
STANDSTILL_SPEED = 0.3        # m/s
STANDSTILL_REARM_S = 2.       # stopped this long (a queue, a red light), the pulse goes again once the car moves
CLEAR_MIN_DEG = 45.           # past the junction, the turn desire is cleared once the car has turned this far...
CLEAR_MARGIN_DEG = 10.        # ...and before it, once it has turned the maneuver's angle less this...
CLEAR_MIN_FRACTION = 0.75     # ...and at least this much of it
CLEAR_WAIT_S = 6.             # past the junction short of CLEAR_MIN_DEG, the turn may get there this long after
CLEAR_FRAMES = 5              # frames CLEAR_DESIRE is published

DESIRES = {(TURN, LEFT): Desire.turnLeft, (TURN, -LEFT): Desire.turnRight,
           (KEEP, LEFT): Desire.keepLeft, (KEEP, -LEFT): Desire.keepRight}


def keep_lead(v_ego: float) -> float:
  return max(KEEP_DESIRE_LEAD_S * v_ego, KEEP_DESIRE_LEAD_MIN_M)


class DesireTiming:
  def __init__(self):
    self.maneuver_id = -1
    self.clear_frames = 0     # CLEAR_DESIRE goes out while this counts down
    self._pending = None      # [turned deg, side, s] of a passed turn not yet turned CLEAR_MIN_DEG
    self._reset()

  def _reset(self):
    self.active = False       # a desire is being held for the current maneuver
    self.started = False      # the desire went out for this maneuver: latched until passed or timed out
    self.done = False         # passed or timed out: nothing more for this maneuver
    self.pulses = 0
    self._since_pulse = 0.
    self.turned_deg = 0.      # heading change toward the turn since the first pulse
    self.entry = False        # the pulse at ENTRY_M went out
    self._gap = 0
    self._moving_t = 0.
    self._stopped_t = 0.
    self.side = 0             # set once a turn desire went out: the turn to clear once it is done
    self._held = False        # held off while openpilot wasn't steering, since the last pulse
    self._rearmed = False     # re-armed by a standstill, since the last pulse
    self._abandoned = False   # abandon()ed: TurnPilot handed back
    self._held_none = 0       # frames of none while a hold had the car standing, since the last pulse

  def _turn_over(self):
    """The maneuver was passed or replaced: clear its turn desire now, or once the car has turned CLEAR_MIN_DEG."""
    if self.side:
      self._pending = [self.turned_deg, self.side, 0.]
      self.side = 0

  def abandon(self):
    """TurnPilot handed back: no more desires for this maneuver. A turn desire already out is cleared from the model's
    queue once the car has turned CLEAR_MIN_DEG, as for a passed turn; the caller sends None as the maneuver from now on."""
    self._turn_over()
    self.done, self.active = True, False
    self._abandoned = True

  @property
  def held_off(self) -> bool:
    """The desire was dropped while openpilot wasn't steering, and no pulse has gone since."""
    return self._held

  def resume(self):
    """TurnPilot acts again after a handback: a maneuver abandoned before any desire went out for it is taken up again,
    as on a fresh engage. One whose desire went out stays done (turnpilotd decides about a held turn)."""
    if self._abandoned and not self.started:
      self.done = self._abandoned = False

  def hold_off(self):
    """The desire was dropped for a frame (turnpilotd faulted): as while openpilot wasn't steering, it pulses again only
    if the car hasn't turned TURN_STARTED_DEG."""
    self.active, self._held, self._gap = False, True, 0

  def track_yaw(self, yaw_rate: float, dt: float):
    """A frame turnpilotd faulted on, with this turn's desire out: the car's turn toward it still counts, so its desire is
    cleared at its angle once TurnPilot runs again, as if there had been no fault."""
    if self.started and self.side and not self.done:
      self.turned_deg += math.degrees(yaw_rate * dt) * self.side

  def carry_clear(self, old: "DesireTiming"):
    """Take over old's turn desire to clear (turnpilotd replaced it: a fault or TurnPilot off, its turn over meanwhile):
    cleared from the model's queue now if the car has turned CLEAR_MIN_DEG, else once it has, as for a passed turn."""
    old._turn_over()
    self._pending, self.clear_frames = old._pending, old.clear_frames
    if self._pending is not None and self._pending[0] >= CLEAR_MIN_DEG:
      self._clear()

  def cancel(self):
    """A turn skipped with its desire out but not yet turned on (turnpilotd: a dropped turn too close to pulse again):
    its desire comes out of the model's queue now, and nothing more goes for it."""
    self._clear()
    self.done, self.active = True, False

  def _clear(self):
    self.clear_frames, self._pending, self.side = CLEAR_FRAMES, None, 0

  def update(self, maneuver, distance: float, maneuver_id: int, passed: bool, v_ego: float,
             steering_angle_deg: float, keep_left_ok: bool, keep_right_ok: bool, dt: float, yaw_rate: float = 0.,
             lat_active: bool = True, replaced: bool = False, may_start: bool = True, go: bool = False,
             holding: bool = False) -> int:
    """The desire to publish, CLEAR_DESIRE once a turn is done. maneuver is RouteFollower.upcoming (None when there is
    none); keep_left_ok/keep_right_ok: the model sees a lane line and road that way (lane_planner.ModelLanes);
    yaw_rate: rad/s, left positive; lat_active: carControl.latActive; replaced: RouteFollower.replaced;
    may_start: the car can still make the turn (turn_skip.py); no turn pulse starts while it can't; go: a hold let go of
    the car at standstill (turnpilotd): the turn pulses again now, not once the car moves, since the model won't pull
    away from a standstill with its pulse forgotten; holding: a hold has the car (shouldStop): standing, the turn desire
    drops to none (its edge stays in the model's queue), so the go's pulse is a fresh edge in the very frame it lets go."""
    if maneuver_id != self.maneuver_id:
      if replaced and self.side and self.turned_deg < TURN_STARTED_DEG:
        self._clear()
      else:
        self._turn_over()
    if self._pending is not None:
      self._pending[0] += math.degrees(yaw_rate * dt) * self._pending[1]
      self._pending[2] += dt
      if self._pending[0] >= CLEAR_MIN_DEG:
        self._clear()
      elif self._pending[2] > CLEAR_WAIT_S:
        self._pending = None
    desire = self._update(maneuver, distance, maneuver_id, passed, v_ego, keep_left_ok, keep_right_ok, dt, yaw_rate, lat_active,
                          may_start, go, holding)
    if desire != Desire.none:
      self._pending = None  # the next maneuver's pulse is in the model's queue now
    if self.clear_frames > 0:
      self.clear_frames -= 1
      return CLEAR_DESIRE
    return desire

  def _update(self, maneuver, distance: float, maneuver_id: int, passed: bool, v_ego: float,
              keep_left_ok: bool, keep_right_ok: bool, dt: float, yaw_rate: float, lat_active: bool, may_start: bool,
              go: bool = False, holding: bool = False) -> int:
    if maneuver_id != self.maneuver_id:
      self.maneuver_id = maneuver_id
      self._reset()
      passed = False  # that was the maneuver before this one
    if passed or maneuver is None:
      self.done = self.done or passed
      self.active = False
      if passed:
        self._turn_over()
      return Desire.none
    desire = DESIRES.get((maneuver.kind, maneuver.side), Desire.none)
    if desire == Desire.none or self.done:
      return Desire.none

    if maneuver.kind == TURN:
      want = self.started or distance < TURN_DESIRE_LEAD_M
    else:
      if distance < -KEEP_DESIRE_PAST_M:
        self.done, self.active = True, False
        return Desire.none
      want = self.started or (distance < keep_lead(v_ego) and (keep_left_ok if maneuver.side == LEFT else keep_right_ok))
    if not want:
      self.active = False
      return Desire.none

    turn = maneuver.kind == TURN
    if not lat_active:
      self.active, self._held, self._gap = False, True, 0
    if turn and (self.active or self._held or self.started):  # a stop after a re-arm is still in the turn
      self.turned_deg += math.degrees(yaw_rate * dt) * maneuver.side
      if self.turned_deg >= max(maneuver.angle_deg - CLEAR_MARGIN_DEG, CLEAR_MIN_FRACTION * maneuver.angle_deg):
        if self.side:
          self._clear()
        self.done, self.active = True, False
        return Desire.none
    if not lat_active:
      return Desire.none

    if v_ego < STANDSTILL_SPEED:
      self._stopped_t += dt
      if go and turn and not self.started and may_start:
        # a turn whose first pulse is its go (an unprotected left taken from its hold line with no pulse on the approach,
        # turnpilotd): now, standing, and the car gets STANDSTILL_REARM_S to move off on it
        self._stopped_t = 0.
      elif holding and turn and self.started:
        # held standing: none until the go, which then pulses a fresh edge at once; the edge already sent stays queued
        self.active, self._rearmed, self._gap = False, True, 0
        self._held_none += 1
        return Desire.none
      if go and turn and self.started and self.turned_deg < STANDSTILL_REPULSE_MAX_DEG and may_start and \
         self._held_none and self._held_none < PULSE_GAP_FRAMES:
        self._gap = PULSE_GAP_FRAMES - self._held_none  # held under PULSE_GAP_FRAMES: the drop lasts that long all the same
        self._held_none = 0
      elif go and turn and self.started and self.turned_deg < STANDSTILL_REPULSE_MAX_DEG and may_start:
        # a hold let go of the car here: pulse now, at standstill (GO_PULSE). A fresh one if the standstill re-armed the
        # turn, else a repeat; then the standstill clock starts over, so the car gets STANDSTILL_REARM_S to move off on it
        self._stopped_t = 0.
        if self.active:
          self._gap, self._since_pulse = PULSE_GAP_FRAMES, 0.
          self.pulses += 1
          self.entry = self.entry or distance < ENTRY_M
      elif self._stopped_t > STANDSTILL_REARM_S:
        self.active = False  # the model has forgotten the pulse by now; send it again once the car moves
        self._rearmed = True
        return Desire.none
    else:
      self._stopped_t = 0.
      self._moving_t += dt
      if self._moving_t > DESIRE_TIMEOUT_S and not turn:
        self.done, self.active = True, False
        return Desire.none

    if self.active and turn:
      self._since_pulse += dt
      entry = not self.entry and distance < ENTRY_M and self._since_pulse >= ENTRY_GAP_S
      # used up once it can go: crossing ENTRY_M within ENTRY_GAP_S of a repeat, it waits out the gap and then goes
      self.entry = self.entry or entry
      if (entry or (self._since_pulse >= REPULSE_S and distance > ENTRY_M)) and self.turned_deg < TURN_STARTED_DEG and \
         self.pulses < MAX_PULSES and may_start:
        self._gap, self._since_pulse = PULSE_GAP_FRAMES, 0.
        self.pulses += 1
    if self._gap > 0:
      self._gap -= 1
      return Desire.none

    if not self.active:
      started_deg = STANDSTILL_REPULSE_MAX_DEG if self._rearmed and not self._held else TURN_STARTED_DEG
      if turn and (not may_start or self.turned_deg >= started_deg):
        return Desire.none  # a pulse in the turn (re-engaging, pulling away) turns the car on past the exit road
      if turn and not self.started and distance < 0.:
        return Desire.none  # a first pulse past the junction turns the car into the curb (turn_skip.py skips it)
      self.active = self.started = True
      if turn and distance < ENTRY_M:
        self.entry = True  # a pulse inside ENTRY_M (on engaging, pulling away) is the entry pulse
      self._held = self._rearmed = False
      self._held_none = 0
      self.pulses += 1
      self._since_pulse = 0.
      if turn:
        self.side = maneuver.side
    return desire
