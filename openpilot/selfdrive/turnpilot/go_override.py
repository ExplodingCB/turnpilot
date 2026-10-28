"""TurnPilot's bounded go (DESIGN phase 4 decision 1 and its stop-sign amendment, 2026-10-28; G5).

The longitudinal planner takes the min() of its accel candidates and the any() of their shouldStops, so TurnPilot can only
ever lower the car's accel. At a standstill the driving model holding the car (its shouldStop: under 0.3 m/s with its accel
under 0.1) keeps it there whatever TurnPilot lets go of: in BeamNG (G0, 2026-10-28) 14 of 24 tapless goes stood 20 s, the
model's shouldStop true in 0.95-1.0 of their frames. Owner (2026-10-28): no gas taps anywhere, and the autonomous left and the
stop sign's go go by themselves; "we don't want to kill Chase but we do want a good demo". This is the one upward path, and
it is bounded:

- Only for a go turnpilotd certifies (turnpilotPlan.goCertified: its cause and id, asserted on every frame of the go; GoCertifier
  below), read from an active plan (hooks.active_plan: fresh, valid, not shadow, not disabled), so a faulted plan, which
  carries the stops in force, never carries a go.
- Only under GO_END_SPEED (3 m/s), and only while the e2e model is the one candidate holding the car: the lead/mpc, cruise and
  TurnPilot candidates don't hold it (should_stop(v, a) false and their own stop false; TurnPilot's shouldStop is a walker's
  hold, a map stop, a withdrawn go), no lead is within GO_CLEAR_M (15 m: radarState's leadOne and leadTwo, modelV2's
  leadsV3[0] over GO_LEAD_PROB; turnpilotd blocks on its own tracks and raw boxes in the path within it), no pedal, engaged.
- The e2e candidate is lifted to GO_ACCEL (+0.5 m/s^2) at most, max(model, GO_ACCEL), and its shouldStop dropped. Lead, cruise
  and every TurnPilot cap stay in the min(), TurnPilot's shouldStop in the any(): the lift never raises the planner's output
  past max(its output without the lift, GO_ACCEL).
- For at most GO_WINDOW_S (3 s) from the go (plannerd's frames, its modelV2 clock, and the plan's own t since the go, the
  largest), ending at GO_END_SPEED or once the takeover predicate (model_takes_over) has held TAKEOVER_S. Anything else above
  failing ends it that frame, for good: a pedal, a disengage, a lead or a holder appearing, the model braking harder than
  GO_MODEL_BRAKE (it sees something; beyond DESIGN, narrowing only), and the go withdrawn (cause none, a stale plan) or
  replaced (a new id). The next go is a new id with its own window, never the same go resumed.

What it can do to the car: from rest, 0.5 m/s^2 for 3 s is 2.25 m at 1.5 m/s at most (an actuator lag only shortens it). From
the unprotected left's hold line (9 m before the maneuver point) that leaves the car 6.75 m out, 2.75 m short of the
oncoming lanes (ONCOMING_ENTRY_M, 4 m), and a stop from 1.5 m/s at 1.5 m/s^2 takes 0.75 m: the lift alone never puts the
car where a hard brake is needed, and the left can withdraw its go throughout it (unprotected_left: _after_go). Past that the
model drives, not the lift. A stop sign's go (STOP_SIGNS.md v3 section 2.5) stands 3.3 m or more from the closest plausible
edge, which the 2.25 m nudge doesn't reach.

The takeover predicate (model_takes_over) is the one function the stop-sign decider imports for its gate (section 2.6).
Every lift is logged at its start and its end with the reason (turnpilot.go_lift, turnpilot.go_lift_end), and every go
turnpilotd certifies or ends (turnpilot.go_certified, turnpilot.go_end), as every hard brake is.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import should_stop
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.turnpilot.hooks import active_plan

GO_ACCEL = 0.5          # m/s^2: the e2e candidate is lifted to this at most
GO_WINDOW_S = 3.        # s from the go
GO_END_SPEED = 3.       # m/s: the lift never acts at or above it
GO_CLEAR_M = 15.        # m: no lead (plannerd) and no in-path detection (turnpilotd) within this
GO_LEAD_PROB = 0.5      # modelV2.leadsV3[0].prob over this is a lead (plannerd's and turnpilotd's threshold)
GO_MODEL_BRAKE = -1.    # m/s^2: the model braking harder than this sees something: the go ends
# the takeover predicate (STOP_SIGNS.md v3 section 2.6): modelV2.action.shouldStop false and the model's planned speed at least
# TAKEOVER_V over the next TAKEOVER_T_S of its plan, held TAKEOVER_S. Read as every point of the plan's speed from
# TAKEOVER_FROM_S to TAKEOVER_T_S: its first samples start at the car's own speed, so from a standstill no plan is 0.5 m/s
# at once, and a plan that goes and stops again inside 2 s (a model creeping to its own stop line) isn't the model keeping the
# car moving. SG4a checks it against the G0 logs; step S4a measures it on the car
TAKEOVER_V = 0.5
TAKEOVER_FROM_S = 1.
TAKEOVER_T_S = 2.
TAKEOVER_S = 0.3
TAKEOVER_SAMPLES = np.linspace(TAKEOVER_FROM_S, TAKEOVER_T_S, 5)
CAUSES = ('turnRelease', 'unprotectedLeft', 'stopSign', 'rollUp', 'creep', 'clearBox')  # TurnpilotPlan.GoCertified.Cause


def model_takes_over(model) -> bool:
  """The takeover predicate, one frame of it (TAKEOVER_*): the model means to keep the car moving. modelV2 with no speed plan
  never takes over."""
  if model.action.shouldStop:
    return False
  v = model.velocity.x
  if len(v) != len(ModelConstants.T_IDXS):
    return False
  return bool(np.min(np.interp(TAKEOVER_SAMPLES, ModelConstants.T_IDXS, list(v))) >= TAKEOVER_V)


class Takeover:
  """model_takes_over held TAKEOVER_S, frame by frame; a frame without it starts over."""
  def __init__(self):
    self.held_s = 0.

  def reset(self) -> None:
    self.held_s = 0.

  def update(self, model, dt: float = DT_MDL) -> bool:
    self.held_s = self.held_s + dt if model_takes_over(model) else 0.
    return self.held_s >= TAKEOVER_S - 1e-6


def lead_within(sm, limit_m: float = GO_CLEAR_M) -> bool:
  """A lead within limit_m the planner sees: radarState's leadOne or leadTwo, or modelV2's first lead."""
  rs = sm['radarState']
  for lead in (rs.leadOne, rs.leadTwo):
    if lead.present and lead.dRel < limit_m:
      return True
  leads = sm['modelV2'].leadsV3
  return len(leads) > 0 and leads[0].prob > GO_LEAD_PROB and len(leads[0].x) > 0 and leads[0].x[0] < limit_m


def holds(v_ego: float, candidate) -> bool:
  """An (accel, shouldStop) candidate holds the car: its own stop, or it would keep a standing car standing."""
  accel, stop = candidate
  return bool(stop) or should_stop(v_ego, accel)


class GoLift:
  """plannerd's side (longitudinal_planner.py): the e2e candidate for this frame, lifted or not (module docstring)."""
  def __init__(self):
    self.key: tuple | None = None  # (cause, id) of the go last seen
    self.active = False            # ...still in its window, nothing having ended it
    self.lifting = False           # lifted this frame
    self.frames = 0
    self.t0: float | None = None   # its first frame's modelV2 time, s
    self.elapsed = 0.
    self.end = ''                  # why the last go's lift ended
    self.takeover = Takeover()

  def _end(self, reason: str, v_ego: float) -> None:
    if self.active:
      cloudlog.event("turnpilot.go_lift_end", cause=self.key[0], go_id=self.key[1], reason=reason, t=round(self.elapsed, 2),
                     v=round(v_ego, 2))
      self.active, self.end = False, reason

  def update(self, sm, v_ego: float, mpc, cruise, tp, e2e, dt: float = DT_MDL, now: float | None = None):
    """The e2e candidate (accel, shouldStop) for this frame. mpc, cruise, e2e: the planner's (accel, shouldStop) candidates,
    tp TurnPilot's (hooks.limits_plan's caps as an accel, its shouldStop) or None; now: modelV2's time, s, if known."""
    self.lifting = False
    plan = active_plan(sm)
    go = plan.goCertified if plan is not None else None
    if go is None or str(go.cause) == 'none':
      self._end('withdrawn', v_ego)  # the key stays: the same go asserted again is over, never resumed
      return e2e
    key = (str(go.cause), int(go.id))
    if key != self.key:
      self._end('replaced', v_ego)
      self.key, self.active, self.frames, self.t0, self.end = key, True, 0, now, ''
      self.takeover.reset()
      cloudlog.event("turnpilot.go_lift", cause=key[0], go_id=key[1], v=round(v_ego, 2), plan_t=round(float(go.t), 2))
    if not self.active:
      return e2e
    self.frames += 1
    clock = now - self.t0 if now is not None and self.t0 is not None else 0.
    self.elapsed = max((self.frames - 1) * dt, float(go.t), clock)
    cs = sm['carState']
    taken_over = self.takeover.update(sm['modelV2'], dt)
    if self.elapsed >= GO_WINDOW_S - 1e-6:
      reason = 'window'
    elif v_ego >= GO_END_SPEED:
      reason = 'speed'
    elif taken_over:
      reason = 'takeover'
    elif cs.brakePressed or cs.gasPressed or not sm['selfdriveState'].enabled:
      reason = 'driver'
    elif lead_within(sm):
      reason = 'lead'
    elif holds(v_ego, mpc) or holds(v_ego, cruise) or (tp is not None and holds(v_ego, tp)):
      reason = 'holder'
    elif e2e[0] < GO_MODEL_BRAKE:
      reason = 'modelBrake'
    else:
      self.lifting = True
      return max(float(e2e[0]), GO_ACCEL), False
    self._end(reason, v_ego)
    return e2e


class GoCertifier:
  """turnpilotd's side: the go in hand, one at a time, each a new id with its own GO_WINDOW_S, filled into every plan while
  it is out (turnpilotPlan.goCertified). It ends as the lift does on its own (GO_WINDOW_S; GO_END_SPEED or the takeover
  predicate, which is the go followed: taken_over), or when turnpilotd withdraws it (withdraw: its reason). A go asked for
  while another is out is refused: the next starts only after the previous one ended."""
  def __init__(self):
    self.id = 0                # the last go's id; never repeated while turnpilotd runs
    self.cause = 'none'        # the go out, 'none' with none
    self.t = 0.                # s since it
    self.taken_over = False    # the last go ended on the model taking over (or GO_END_SPEED): followed
    self.end = ''              # why the last go ended
    self.takeover = Takeover()

  @property
  def active(self) -> bool:
    return self.cause != 'none'

  def start(self, cause: str, **evidence) -> bool:
    """A certified go (cause: CAUSES) from now, or False while one is out."""
    assert cause in CAUSES, cause
    if self.active:
      cloudlog.event("turnpilot.go_refused", cause=cause, out=self.cause, go_id=self.id, t=round(self.t, 2))
      return False
    self.id += 1
    self.cause, self.t, self.taken_over, self.end = cause, 0., False, ''
    self.takeover.reset()
    cloudlog.event("turnpilot.go_certified", cause=cause, go_id=self.id, **evidence)
    return True

  def _end(self, reason: str, followed: bool = False) -> None:
    cloudlog.event("turnpilot.go_end", cause=self.cause, go_id=self.id, reason=reason, t=round(self.t, 2), followed=followed)
    self.cause, self.end, self.taken_over = 'none', reason, followed

  def update(self, model, v_ego: float, dt: float = DT_MDL) -> None:
    """The go out a frame on: its window, the speed, the model's takeover. Before this frame's start()."""
    if not self.active:
      return
    self.t += dt
    if self.takeover.update(model, dt):
      self._end('takeover', followed=True)
    elif v_ego >= GO_END_SPEED:
      self._end('speed', followed=True)
    elif self.t >= GO_WINDOW_S - 1e-6:
      self._end('window')

  def withdraw(self, reason: str) -> None:
    if self.active:
      self._end(reason)

  def fill(self, plan) -> None:
    if self.active:
      g = plan.goCertified
      g.cause, g.id, g.t = self.cause, self.id, float(self.t)
