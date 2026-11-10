"""Pick the lateral source for controlsd: stock modelV2 or a held Alpamayo plan.

Alpamayo plans arrive at ~1-3 Hz. Between updates the last plan is held and
time-shifted: the ego motion since the plan's camera frame is dead reckoned
(vEgo + calibrated yaw rate) and pure pursuit on the shifted path gives the
desired curvature. Any problem falls back to stock with a reason.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# Fallback reasons, also published in alpamayoState.fallbackReason
OK = "ok"
NO_PLAN = "noPlan"
STALE = "stale"
DISAGREE = "disagree"
LANE_CHANGE = "laneChange"
INVALID = "invalid"
WARMUP = "warmup"
TOO_FAST = "tooFast"


@dataclass
class Plan:
  t_ns: int            # camera frame time the plan is relative to (monotonic ns)
  x: np.ndarray        # device-point path in the calib frame at t_ns, x fwd [m]
  y: np.ndarray        # y right [m] (openpilot modelV2 convention)
  t: np.ndarray        # seconds after t_ns


@dataclass
class Ego:
  """Device-point pose relative to the plan frame, integrated since the plan time."""
  t_ns: int
  x: float = 0.0
  y: float = 0.0
  yaw: float = 0.0  # right positive (calib frame, z down)


@dataclass
class LateralSourceConfig:
  max_plan_age_s: float = 1.5
  max_disagree_m: float = 1.5
  disagree_hold_s: float = 0.5
  disagree_horizon_s: float = 2.0
  good_plans_to_reengage: int = 2
  lookahead_s: float = 0.9        # plus the lateral delay
  min_lookahead_m: float = 5.0
  max_speed: float = 9.0          # m/s; slow plans (seconds old) are only safe at low speed


@dataclass
class LateralSource:
  cfg: LateralSourceConfig = field(default_factory=LateralSourceConfig)
  plan: Plan | None = None
  ego: Ego | None = None
  reason: str = NO_PLAN
  disagree_m: float = 0.0
  _disagree_since: float | None = None
  _good_plans: int = 0
  _latched: bool = True  # stock until enough good plans arrive

  def new_plan(self, plan: Plan, valid: bool) -> None:
    if not valid or not (np.all(np.isfinite(plan.x)) and np.all(np.isfinite(plan.y))):
      self._good_plans = 0
      self._latched = True
      self.reason = INVALID
      return
    self.plan = plan
    self.ego = Ego(plan.t_ns)
    self._good_plans += 1

  def _integrate(self, now_ns: int, v: float, yaw_rate: float) -> None:
    """yaw_rate in the calib frame (z down, right turn positive)."""
    e = self.ego
    dt = max(0.0, (now_ns - e.t_ns) * 1e-9)
    if dt > 0.2:  # missed updates: integrate in small steps at constant inputs
      n = int(dt / 0.01) + 1
      for _ in range(n):
        self._step(dt / n, v, yaw_rate)
    else:
      self._step(dt, v, yaw_rate)
    e.t_ns = now_ns

  def _step(self, dt, v, yaw_rate):
    e = self.ego
    e.x += v * math.cos(e.yaw) * dt
    e.y += v * math.sin(e.yaw) * dt
    e.yaw += yaw_rate * dt

  def path_in_ego(self) -> tuple[np.ndarray, np.ndarray]:
    """Plan path expressed in the current device frame (x fwd, y right)."""
    e, p = self.ego, self.plan
    c, s = math.cos(-e.yaw), math.sin(-e.yaw)
    dx, dy = p.x - e.x, p.y - e.y
    return c * dx - s * dy, s * dx + c * dy

  def update(self, now_ns: int, v: float, yaw_rate: float, lat_delay: float,
             stock_x: np.ndarray, stock_y: np.ndarray, lane_change: bool) -> tuple[float | None, str]:
    """Return (desired_curvature or None for stock, reason). Right positive, like controlsd and modeld."""
    if self.plan is None:
      self.reason = NO_PLAN
      return None, self.reason
    self._integrate(now_ns, v, yaw_rate)
    age = (now_ns - self.plan.t_ns) * 1e-9
    if age > self.cfg.max_plan_age_s:
      self._latched, self._good_plans = True, 0
      self.reason = STALE
      return None, self.reason
    px, py = self.path_in_ego()
    ahead = px > 0.0
    if ahead.sum() < 2:
      self.reason = STALE
      return None, self.reason
    px, py = px[ahead], py[ahead]

    # disagreement with stock at a fixed time horizon ahead
    x_cmp = max(v * self.cfg.disagree_horizon_s, self.cfg.min_lookahead_m)
    if x_cmp <= px[-1] and len(stock_x) > 1:
      self.disagree_m = abs(float(np.interp(x_cmp, px, py)) - float(np.interp(x_cmp, stock_x, stock_y)))
    else:
      self.disagree_m = 0.0
    now_s = now_ns * 1e-9
    if self.disagree_m > self.cfg.max_disagree_m:
      self._disagree_since = self._disagree_since or now_s
      if now_s - self._disagree_since > self.cfg.disagree_hold_s:
        self._latched, self._good_plans = True, 0
        self.reason = DISAGREE
        return None, self.reason
    else:
      self._disagree_since = None

    if lane_change:
      self.reason = LANE_CHANGE
      return None, self.reason
    if v > self.cfg.max_speed:
      self.reason = TOO_FAST
      return None, self.reason
    if self._latched:
      if self._good_plans < self.cfg.good_plans_to_reengage:
        self.reason = WARMUP
        return None, self.reason
      self._latched = False

    # pure pursuit on the time-shifted plan
    ld = max(self.cfg.min_lookahead_m, v * (self.cfg.lookahead_s + lat_delay))
    ld = min(ld, float(np.hypot(px[-1], py[-1])))
    d = np.hypot(px, py)
    i = int(np.searchsorted(d, ld))
    i = min(max(i, 1), len(d) - 1)
    f = (ld - d[i - 1]) / max(d[i] - d[i - 1], 1e-6)
    ty = py[i - 1] + f * (py[i] - py[i - 1])
    tx = px[i - 1] + f * (px[i] - px[i - 1])
    L2 = tx * tx + ty * ty
    curv_right = 2.0 * ty / max(L2, 1.0)
    self.reason = OK
    return curv_right, self.reason
