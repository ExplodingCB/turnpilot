#!/usr/bin/env python3
"""Verify the stop-sign decider's launch on the real car from step S4a's logs (STOP_SIGNS.md 2.6, 8.3; SG4a) and stamp it.

  launch_verify.py --out OUT.json [--vis visibility.json] [--mode trial] [--date YYYY-MM-DD] RLOG_OR_DIR ...

S4a is 10 goes at an empty showcase all-way under TurnPilotStopSignGo=trial, foot over the brake. From the rlogs given (files,
or directories searched for rlog files) this reads:

- Every go of the decider's (turnpilotPlan.stopGo entering 'going' from a stop, in the --mode it acted in): its time, its
  takeover (stopGo.takeoverS, the G5 takeover predicate held TAKEOVER_S, while the go is out), its retries, where it was and
  how it ended (committed, withdrawn and why, a fault, the driver). A go is taken over when its takeover came while it was
  out, which is inside the gate: the decider withdraws a go not taken over by the gate point (or G5's window). A go withdrawn
  for anything but the model (a car, a person, a view; the driver's brake) said nothing about the model and isn't counted:
  it is listed. The launch passes with GOES_MIN or more counted goes and at least TAKEOVER_FRAC of them taken over (9 of 10).
- The longitudinal actuator delay: the lag of carState.aEgo behind carControl.actuators.accel (longActive) over the run's
  stops and launches (each between two standstills, its samples under DELAY_V_MAX, its command moving DELAY_ACCEL_RANGE or
  more), the lag in
  0-DELAY_MAX_S that best aligns them over all of them, and its spread (the per-stop best lags' standard deviation). It
  counts as measured with DELAY_SEGMENTS_MIN or more such windows spread under DELAY_SPREAD_MAX_S. T_ABORT takes the larger of
  it and CarParams' (stop_rules.t_abort), and a capped abort's end short of dEdgeMin rests on it, so an unmeasured delay is
  a failing stamp, never a pass.

It writes, into a copy of visibility.json (--out; never the file itself), launchVerified {date, goes, takenOver,
maxTakeoverS, verdict: pass or fail, tool, undecided} (a failing run's too: the stamp is never written by hand) and, when
measured, actuatorDelayS; then visibility_table.py --check on the copy. It never writes a pass it didn't measure. Each go is
printed with dEdgeMin and dEdgeMax and its position, for comparing with the bumper taped to the lane edge at 3 of the stops
(POS_ERR_M and the band). Exit 0 on a pass the check accepts, 1 otherwise.
"""
import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path

import numpy as np

from openpilot.selfdrive.turnpilot import visibility_table

GOES_MIN = 10                 # counted goes a pass needs
TAKEOVER_FRAC = 0.9           # ...and the share of them taken over inside the gate (9 of 10)
MODEL_ENDS = ('gate', 'window', 'stoodAgain')  # withdrawals that are the model's: it didn't take over, or stood again
DELAY_MAX_S = 1.5             # s: the lags tried, 0 to this
DELAY_STEP_S = 0.01
DELAY_V_MAX = 6.              # m/s: stops and launches are under this
DELAY_ACCEL_RANGE = 0.5       # m/s^2 the command must move by inside a window for it to say anything about the lag
DELAY_SEGMENT_MIN_S = 2.      # s a window must last
DELAY_SEGMENTS_MIN = 3
DELAY_SPREAD_MAX_S = 0.1
STILL_V, STILL_S = 0.05, 1.0  # m/s, s: a standstill between two stops and launches
GRID_S = 0.01                 # s: both series resampled to this


def rlog_paths(args: list[str]) -> list[Path]:
  out = []
  for a in args:
    p = Path(a)
    if p.is_dir():
      out += sorted(q for q in p.rglob('*') if q.is_file() and q.name.split('.')[0] == 'rlog')
    else:
      out.append(p)
  return out


@dataclass
class Go:
  t: float                    # s, log time of the go
  mode: str
  maneuver: str
  position_m: float
  d_edge_min: float
  d_edge_max: float
  retries: int
  takeover_s: float = -1.
  end: str = ''               # committed, withdrawn, fault, driver, open (the log ended)
  why: str = ''               # the withdrawal's reason, the fault
  stood_m: float | None = None  # m past the stop point where it ended

  @property
  def taken_over(self) -> bool:
    return self.takeover_s >= 0.

  @property
  def counted(self) -> bool:
    """It says whether the model takes over: taken over, or withdrawn for the model (MODEL_ENDS)."""
    return self.taken_over or (self.end == 'withdrawn' and self.why in MODEL_ENDS)


@dataclass
class Series:
  plans: list = field(default_factory=list)   # (t, turnpilotPlan)
  accel: list = field(default_factory=list)   # (t, actuators.accel) with longActive
  a_ego: list = field(default_factory=list)   # (t, aEgo, vEgo)


def read_logs(paths: list[Path]) -> Series:
  from openpilot.tools.lib.logreader import LogReader
  out = Series()
  for p in paths:
    for msg in LogReader(str(p)):
      w, t = msg.which(), msg.logMonoTime * 1e-9
      if w == 'turnpilotPlan':
        out.plans.append((t, msg.turnpilotPlan))
      elif w == 'carControl':
        cc = msg.carControl
        out.accel.append((t, float(cc.actuators.accel) if cc.longActive else None))
      elif w == 'carState':
        out.a_ego.append((t, float(msg.carState.aEgo), float(msg.carState.vEgo)))
  for s in (out.plans, out.accel, out.a_ego):
    s.sort(key=lambda x: x[0])
  return out


def goes(plans, mode: str) -> list[Go]:
  """The decider's goes in mode (module docstring), in order."""
  out: list[Go] = []
  cur: Go | None = None
  prev = 'off'
  for t, p in plans:
    g = p.stopGo
    state = str(g.state)
    if state == 'going' and prev != 'going':
      cur = Go(t, str(g.mode), str(g.maneuver), float(g.positionM), float(g.dEdgeMin), float(g.dEdgeMax), int(g.retries))
      if cur.mode == mode:
        out.append(cur)
      else:
        cur = None
    if cur is not None and cur.end == '':
      if state == 'going' and g.takeoverS >= 0. and not cur.taken_over:
        cur.takeover_s = float(g.takeoverS)
      if state != 'going':
        cur.stood_m = float(g.positionM)
        if state in ('committed', 'inLane'):
          cur.end = 'committed'
        elif int(g.retries) > cur.retries:
          # withdrawn (stopGo.retries counts each withdrawal; withdrawn is the last one's reason), H5 after it included
          cur.end, cur.why = 'withdrawn', str(g.withdrawn)
        elif state == 'ladder':
          cur.end, cur.why = 'fault', str(g.fault)
        else:
          cur.end = 'driver'  # done or off straight from going: openpilot disengaged (O6, O7)
    prev = state
  for go in out:
    if go.end == '':
      go.end = 'open'
  return out


def resample(pts, t0: float, t1: float):
  """[(t, value or None)] onto the GRID_S grid from t0 to t1 by linear interpolation; None where a value is missing."""
  n = int(math.floor((t1 - t0) / GRID_S)) + 1
  out, j = [], 0
  for k in range(n):
    t = t0 + k * GRID_S
    while j + 1 < len(pts) and pts[j + 1][0] <= t:
      j += 1
    if j + 1 >= len(pts) or pts[j][0] > t:
      out.append(None)
      continue
    (ta, a), (tb, b) = pts[j], pts[j + 1]
    if a is None or b is None or tb - ta > 0.2:
      out.append(None)
      continue
    out.append(a + (b - a) * (t - ta) / (tb - ta) if tb > ta else a)
  return out


def segments(accel, a_ego, v) -> list[tuple[int, int]]:
  """Grid index windows of the run's stops and launches (module docstring): cut in the middle of every standstill of
  STILL_S or more (a launch's command leads its motion by the delay, so the cut is never at the motion's start), each kept
  with DELAY_SEGMENT_MIN_S of usable samples under DELAY_V_MAX and its command moving DELAY_ACCEL_RANGE or more."""
  n = len(v)
  still = [x is not None and x < STILL_V for x in v]
  cuts, i = [0], 0
  while i < n:
    if not still[i]:
      i += 1
      continue
    j = i
    while j < n and still[j]:
      j += 1
    if (j - i) * GRID_S >= STILL_S:
      cuts.append((i + j) // 2)
    i = j
  cuts.append(n)
  out = []
  for lo, hi in zip(cuts, cuts[1:], strict=False):
    cmds = [accel[k] for k in range(lo, hi) if usable(accel, a_ego, v, k)]
    if len(cmds) * GRID_S >= DELAY_SEGMENT_MIN_S and max(cmds) - min(cmds) >= DELAY_ACCEL_RANGE:
      out.append((lo, hi))
  return out


def usable(accel, a_ego, v, k: int) -> bool:
  return accel[k] is not None and a_ego[k] is not None and v[k] is not None and v[k] <= DELAY_V_MAX


def lag_sse(accel: np.ndarray, a_ego: np.ndarray, ok: np.ndarray, lo: int, hi: int, k: int) -> tuple[float, int]:
  """(sum of squared errors, samples) of aEgo against the command k grid steps earlier, over [lo, hi) where ok (usable)."""
  idx = np.arange(max(lo, k), hi)
  if not len(idx):
    return 0., 0
  c = accel[idx - k]
  m = ok[idx] & np.isfinite(c)
  err = a_ego[idx][m] - c[m]
  return float(np.sum(err * err)), int(np.count_nonzero(m))


def actuator_delay(series: Series) -> dict:
  """{delayS, spreadS, segments, perSegment, measured} (module docstring)."""
  none = {'delayS': None, 'spreadS': None, 'segments': 0, 'perSegment': [], 'measured': False}
  if not series.accel or not series.a_ego:
    return none
  t0 = max(series.accel[0][0], series.a_ego[0][0])
  t1 = min(series.accel[-1][0], series.a_ego[-1][0])
  if t1 <= t0:
    return none
  accel = resample(series.accel, t0, t1)
  a_ego = resample([(t, a) for t, a, _ in series.a_ego], t0, t1)
  v = resample([(t, v) for t, _, v in series.a_ego], t0, t1)
  segs = segments(accel, a_ego, v)
  if not segs:
    return none
  ok = np.array([usable(accel, a_ego, v, k) for k in range(len(v))], dtype=bool)
  accel_np = np.array([np.nan if a is None else a for a in accel], dtype=np.float64)
  a_ego_np = np.array([np.nan if a is None else a for a in a_ego], dtype=np.float64)
  steps = [int(round(k * DELAY_STEP_S / GRID_S)) for k in range(int(round(DELAY_MAX_S / DELAY_STEP_S)) + 1)]
  total = np.zeros(len(steps))
  per = []
  for lo, hi in segs:
    errs = np.full(len(steps), np.inf)
    for i, k in enumerate(steps):
      sse, m = lag_sse(accel_np, a_ego_np, ok, lo, hi, k)
      if m:
        errs[i] = sse / m
        total[i] += sse / m
      else:
        total[i] = np.inf
    per.append(steps[int(np.argmin(errs))] * GRID_S)
  best = steps[int(np.argmin(total))] * GRID_S
  spread = float(np.std(per))
  return {'delayS': round(best, 3), 'spreadS': round(spread, 3), 'segments': len(segs), 'perSegment': [round(x, 3) for x in per],
          'measured': len(segs) >= DELAY_SEGMENTS_MIN and spread <= DELAY_SPREAD_MAX_S}


def verdict(gs: list[Go], delay: dict) -> tuple[str, list[str]]:
  """('pass' or 'fail', why it fails)."""
  counted = [g for g in gs if g.counted]
  taken = [g for g in counted if g.taken_over]
  why = []
  if len(counted) < GOES_MIN:
    why.append(f"{len(counted)} counted goes, {GOES_MIN} needed")
  elif len(taken) < math.ceil(TAKEOVER_FRAC * len(counted) - 1e-9):
    why.append(f"{len(taken)} of {len(counted)} goes taken over inside the gate, {math.ceil(TAKEOVER_FRAC * len(counted) - 1e-9)} needed")
  if not delay['measured']:
    why.append(f"the actuator delay isn't measured ({delay['segments']} stops and launches, {DELAY_SEGMENTS_MIN} needed; spread " +
               f"{delay['spreadS']} s, {DELAY_SPREAD_MAX_S} s at most)")
  return ('fail' if why else 'pass'), why


def stamp(vis_raw: dict, gs: list[Go], delay: dict, today: str) -> tuple[dict, str, list[str]]:
  """The copy of visibility.json with launchVerified (and, measured, actuatorDelayS), its verdict and why it fails."""
  v, why = verdict(gs, delay)
  counted = [g for g in gs if g.counted]
  taken = [g for g in counted if g.taken_over]
  out = dict(vis_raw)
  out['launchVerified'] = {'date': today, 'goes': len(counted), 'takenOver': len(taken),
                           'maxTakeoverS': round(max((g.takeover_s for g in taken), default=0.), 2), 'verdict': v,
                           'tool': 'launch_verify.py', 'undecided': len(gs) - len(counted)}
  if delay['measured']:
    out['actuatorDelayS'] = delay['delayS']
  return out, v, why


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('logs', nargs='+', help='rlog files, or directories to search for them')
  ap.add_argument('--vis', default=str(visibility_table.SHIPPED), help='the visibility.json to copy (default: the shipped one)')
  ap.add_argument('--out', required=True, help='where to write the stamped copy (never the visibility file itself)')
  ap.add_argument('--mode', default='trial', help="the TurnPilotStopSignGo mode whose goes count (default trial, step S4a's)")
  ap.add_argument('--date', default=None, help='the stamp date (default today)')
  args = ap.parse_args(argv)
  src, dst = Path(args.vis), Path(args.out)
  if dst.resolve() == src.resolve():
    raise SystemExit("--out must be a copy, not the visibility file itself")
  vis_raw = json.loads(src.read_text())
  series = read_logs(rlog_paths(args.logs))
  gs = goes(series.plans, args.mode)
  delay = actuator_delay(series)
  print(f"{len(gs)} {args.mode} goes in {len(series.plans)} plans")
  for k, g in enumerate(gs, 1):
    takeover = f"{g.takeover_s:.2f} s" if g.taken_over else 'none'
    ended = '' if g.stood_m is None else f", ended at {g.stood_m:.2f} m"
    print(f"  go {k} at {g.t:.2f} s ({g.maneuver}, retry {g.retries}): from {g.position_m:.2f} m, dEdgeMin {g.d_edge_min:.2f} " +
          f"dEdgeMax {g.d_edge_max:.2f}; takeover {takeover}; {g.end}{' (' + g.why + ')' if g.why else ''}{ended}" +
          ('' if g.counted else ': not counted (not the model)'))
  print(f"actuator delay: {delay['delayS']} s, spread {delay['spreadS']} s over {delay['segments']} stops and launches " +
        f"{delay['perSegment']}{'' if delay['measured'] else ' (not measured)'}")
  out, v, why = stamp(vis_raw, gs, delay, args.date or date.today().isoformat())
  dst.write_text(json.dumps(out, indent=1) + '\n')
  print(f"launchVerified: {v}" + ''.join(f"\n  {w}" for w in why) + f"\nwritten to {dst}")
  checked = visibility_table.main(['--check', str(dst)])
  return 0 if v == 'pass' and checked == 0 else 1


if __name__ == '__main__':
  sys.exit(main())
