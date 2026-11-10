import numpy as np

from openpilot.selfdrive.alpamayo.lateral_source import (
  DISAGREE, LANE_CHANGE, OK, STALE, TOO_FAST, WARMUP, LateralSource, LateralSourceConfig, Plan)

V = 8.0
T = np.arange(0.1, 6.45, 0.1)


def arc(kappa_right, t0_ns=0, v=V):
  """Device-point path on a constant curvature arc, y right."""
  s = v * T
  if abs(kappa_right) < 1e-9:
    return Plan(t0_ns, s, np.zeros_like(s), T)
  return Plan(t0_ns, np.sin(kappa_right * s) / kappa_right, (1 - np.cos(kappa_right * s)) / kappa_right, T)


def stock_from(plan):
  return plan.x, plan.y


def ready_source(plan, **cfg):
  ls = LateralSource(LateralSourceConfig(**cfg))
  ls.new_plan(plan, True)
  ls.new_plan(plan, True)
  return ls


def test_straight_gives_zero():
  p = arc(0.0)
  ls = ready_source(p)
  c, r = ls.update(10_000_000, V, 0.0, 0.2, *stock_from(p), False)
  assert r == OK and abs(c) < 1e-6


def test_arc_curvature_recovered_and_time_shift():
  k = 0.01  # 100 m radius, right turn
  p = arc(k)
  ls = ready_source(p)
  # drive along the arc for 0.8 s at 100 Hz, curvature should stay near k
  for i in range(1, 81):
    c, r = ls.update(i * 10_000_000, V, V * k, 0.2, *stock_from(p), False)
    assert r == OK
    assert abs(c - k) < 0.1 * k, (i, c)


def test_drift_correction_sign():
  # ego drifted 0.5 m left of a straight plan -> steer right (positive)
  p = arc(0.0)
  ls = ready_source(p)
  ls.ego.y = -0.5
  c, r = ls.update(10_000_000, V, 0.0, 0.2, *stock_from(p), False)
  assert r == OK and c > 0


def test_stale_falls_back_and_needs_new_plans():
  p = arc(0.0)
  ls = ready_source(p, max_plan_age_s=1.0)
  assert ls.update(int(1.1e9), V, 0.0, 0.2, *stock_from(p), False) == (None, STALE)
  p2 = arc(0.0, t0_ns=int(1.2e9))
  ls.new_plan(p2, True)
  assert ls.update(int(1.25e9), V, 0.0, 0.2, *stock_from(p2), False)[1] == WARMUP
  ls.new_plan(p2, True)
  assert ls.update(int(1.26e9), V, 0.0, 0.2, *stock_from(p2), False)[1] == OK


def test_disagreement_needs_hold_time():
  p = arc(0.0)
  stock = arc(0.02)  # bends away by several meters at 2 s
  ls = ready_source(p, disagree_hold_s=0.5)
  assert ls.update(int(0.1e9), V, 0.0, 0.2, *stock_from(stock), False)[1] == OK
  assert ls.update(int(0.7e9), V, 0.0, 0.2, *stock_from(stock), False) == (None, DISAGREE)


def test_lane_change_uses_stock():
  p = arc(0.0)
  ls = ready_source(p)
  assert ls.update(10_000_000, V, 0.0, 0.2, *stock_from(p), True) == (None, LANE_CHANGE)


def test_invalid_plan_latches():
  p = arc(0.0)
  ls = ready_source(p)
  bad = Plan(0, p.x.copy(), p.y.copy(), T)
  bad.y[3] = np.nan
  ls.new_plan(bad, True)
  assert ls.update(10_000_000, V, 0.0, 0.2, *stock_from(p), False)[1] == WARMUP


def test_too_fast_uses_stock():
  p = arc(0.0, v=20.0)
  ls = ready_source(p, max_speed=9.0)
  assert ls.update(10_000_000, 20.0, 0.0, 0.2, *stock_from(p), False) == (None, TOO_FAST)
