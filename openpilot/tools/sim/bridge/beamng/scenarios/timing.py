"""Timing for scenario actors: where a walking mannequin is, when a walker starts and when oncoming cars go, and the
timestamps of a BeamNG AI script. Pure functions of positions (BeamNG world meters, x east, y north) and speeds, and
GoWatch, which follows the car at its junction.

On the signal tiles every scenario junction has a mapped signal, and the car stands at its line (BeamNG smoke 2026-10-28:
11-27 m out) until the driver's go. So the actors key on the car at the junction, not on its distance to the hold line:
the walker sets off on the car's go, and the oncoming cars go a fixed time after the car stands at its hold there, so the
first reaches the junction arrive_after s after the go (the sim driver's tap, driver.STOPPED_TAP_S after the stop).
"""
import math

import numpy as np

from openpilot.tools.sim.bridge.beamng.driver import STOPPED_TAP_S

WALK_SPEED = 1.4  # m/s, a brisk pedestrian
PROFILE_DT = 0.02  # s, integration step for speed profiles
NODE_SPACING = 4.0  # m between AI script nodes
GO_NEAR_M = 35.     # a stand or a turn desire this close to the junction (straight line) is at it: the car stood 27 m out
STOOD_V = 0.3       # m/s: standing
MOVING_V = 0.5      # m/s: moving again


def arc_lengths(pts) -> np.ndarray:
  pts = np.asarray(pts, dtype=float)
  return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1))]


def point_at(pts, s: float):
  """(point, heading) at arc length s along a polyline, clamped to its ends. Heading is radians counterclockwise from east."""
  pts = np.asarray(pts, dtype=float)
  seg = arc_lengths(pts)
  s = min(max(s, 0.0), float(seg[-1]))
  i = min(max(1, int(np.searchsorted(seg, s))), len(pts) - 1)
  f = (s - seg[i - 1]) / max(seg[i] - seg[i - 1], 1e-9)
  d = pts[i] - pts[i - 1]
  return pts[i - 1] + f * d, math.atan2(d[1], d[0])


def resample(pts, spacing: float = NODE_SPACING) -> np.ndarray:
  """The polyline with points every `spacing` m (and its last point)."""
  seg = arc_lengths(pts)
  s = np.r_[np.arange(0.0, seg[-1], spacing), seg[-1]]
  return np.array([point_at(pts, float(v))[0] for v in s])


def walker_pose(path, t: float, speed: float = WALK_SPEED):
  """(point, heading) of a walker t s after it set off along path; it stands at the end once there."""
  return point_at(path, speed * max(t, 0.0))


class GoWatch:
  """The car at its junction, from 20 Hz rows: held_t, when it came to a stand within GO_NEAR_M of the junction with
  TurnPilot holding (a map stop, a queue), and went, its go there: the hold let go with the car standing, the car moving
  again after a stand there, or, never having stood (a green), the turn's desire out there with nothing holding the car
  (braking for the line with the desire out is no go). A stand or a release further out (a signal before the junction's
  own) is no go."""

  def __init__(self, near_m: float = GO_NEAR_M):
    self.near_m = near_m
    self.held_t: float | None = None  # the first held stand near the junction
    self.stood = False                # stood near the junction
    self.held = False                 # ...with TurnPilot holding the car
    self.went = False

  def update(self, t: float, d_junction: float, v: float, holding: bool, turn_desire: bool) -> None:
    """d_junction: m from the car to the junction; holding: turnpilotPlan.state is holding; turn_desire: its desire is a
    turn."""
    if self.went or d_junction > self.near_m:
      return
    if v < STOOD_V:
      self.stood = True
      if holding:
        self.held = True
        self.held_t = t if self.held_t is None else self.held_t
      elif self.held:
        self.went = True  # let go at a standstill: the go, before the car moves
    elif v > MOVING_V and (self.stood or (turn_desire and not holding)):
      self.went = True


def step_out_distance(v: float, p: dict) -> float:
  """m ahead of the car a stepping-out walker (catalog: ped_step_out) sets off at, the car at v m/s: so that once she is
  in its corridor (enter_s after her first step) and react_s more has gone, stopping PED_STOP_GAP_M short of her (gap_m)
  needs decel m/s^2. p: the walker's params (decel, gap_m, enter_s, react_s)."""
  return v ** 2 / (2. * p['decel']) + p['gap_m'] + v * (p['enter_s'] + p['react_s'])


def profile_times(s_nodes, v: float, accel: float, s_change: float = math.inf, v_change: float = 0.0,
                  dt: float = PROFILE_DT) -> list[float]:
  """Seconds from a standing start to reach each arc length in s_nodes (increasing): speed up at `accel` to v, and from
  s_change on change to v_change at the same rate. A node the car never reaches (it stopped short) gets math.inf."""
  out = []
  t = s = vel = 0.0
  for target in s_nodes:
    while s < target:
      want = v if s < s_change else v_change
      vel = max(vel + float(np.clip(want - vel, -accel * dt, accel * dt)), 0.0)
      if vel < 1e-3 and want < 1e-3:
        return out + [math.inf] * (len(s_nodes) - len(out))
      s += vel * dt
      t += dt
    out.append(t - (s - target) / vel if vel > 1e-3 else t)
  return out


def script_nodes(path, v: float, accel: float, s_change: float = math.inf, v_change: float = 0.0,
                 spacing: float = NODE_SPACING) -> list[dict]:
  """A BeamNG AI script ({x, y, z, t} nodes, t from the release) along path from a standing start. Nodes the car would
  not reach are dropped; ai.set_script needs at least three."""
  pts = resample(path, spacing)
  times = profile_times(arc_lengths(pts), v, accel, s_change, v_change)
  return [{'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2]) if len(p) > 2 else 0.0, 't': float(t)}
          for p, t in zip(pts, times, strict=True) if math.isfinite(t)]


def travel_time(s: float, v: float, accel: float) -> float:
  """Seconds from a standing start to cover s m, speeding up at accel to v."""
  return profile_times([s], v, accel)[0]


def oncoming_release_delay(s_car: float, v_car: float, accel: float, arrive_after_s: float, tap_s: float = STOPPED_TAP_S) -> float:
  """Seconds after the car stands at its hold near the junction (GoWatch.held_t) to release oncoming cars standing s_car
  m out, so the first reaches the junction arrive_after_s after the go (the driver's tap tap_s after the stop). One too
  far out to make it from there is refused: it would come late, after the car had turned."""
  delay = tap_s + arrive_after_s - travel_time(s_car, v_car, accel)
  if delay < 0.:
    raise ValueError(f"an oncoming car {s_car:.0f} m out takes {travel_time(s_car, v_car, accel):.1f} s to the junction, " +
                     f"more than the {tap_s + arrive_after_s:.1f} s from the stop to {arrive_after_s} s after the go: start it nearer")
  return delay


def pacer_drop_s(s_start: float, s_junction: float, window_start_m: float, v: float, block_s: float) -> float:
  """Meters into its script (which starts s_start along its lane) the pacer starts dropping back: it keeps the target
  lane occupied for block_s at v after the car, alongside, comes within window_start_m of the junction (s_junction
  along the lane), or for block_s from the start if that is already past."""
  return max(s_junction - window_start_m - s_start, 0.0) + v * block_s


def catch_up_lead(v_ego: float, v_car: float, accel: float) -> float:
  """Meters a car released from rest beside an ego holding v_ego falls behind by the time it reaches v_car (speeding
  up at accel), so a pacer placed this far ahead ends up alongside."""
  t_ramp = v_car / accel
  return max(v_ego * t_ramp - v_car ** 2 / (2.0 * accel), 0.0)
