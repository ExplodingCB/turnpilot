#!/usr/bin/env python3
"""Step S1's camera fit (STOP_SIGNS.md 1.1, 1.4-1.5, 8.3; STOP_SIGNS_SATURDAY.md T1): the wide lens, its frame edge, the
footprint mask and darkThreshold, from a person standing on taped marks beside the parked car and a daylight drive, written
into a copy of visibility.json with the tape's setback, offset and height.

  DEV=METAL camera_fit.py --tape TAPE.json --marks MARKS.json --s1 S1_DIR --drive DIR [DIR ...] --out OUT.json
                          [--vis visibility.json] [--save-detections FILE | --detections FILE]

TAPE.json  {cameraSetbackM (lens to the front bumper's face, along the car's axis), lateralOffsetM (lens to the car's
           centreline, + right), heightM (lens above the ground), date (YYYY-MM-DD)}
MARKS.json {rangeM: 10, marks: [{side: left | right, bearingDeg: 40 | 55 | 70 | 80 | 85 | 88}, ...]}: the marks in the
           order the person stood on them, each rangeM from the point on the ground under the lens, at that bearing off the
           car's axis
--s1       the parked S1 recording's segments (directories holding rlog and ecamera.hevc, searched below the paths given)
--drive    the daylight drive's segments (checklist step 16: calibration, then 5 min more on two headings)

S1 is checked first: a comma four (deviceType mici, os04c10, a 1344x760 ecamera), the car still (vEgo under STILL_V) and in
Park in every carState, calibration calibrated in every message and moving less than CALIB_MOVE_DEG. Then:

1. Detection. YOLO26n (tinygrad OnnxRunner, DEV=METAL) at DETECT_HZ on the full wide letterbox (detector/graph.py) and on
   both side views (detector/sideview.py's maps for S1's calibration, built with the repo lens). A person's feet are the
   bottom centre of their box; a side view's feet point goes back to fisheye pixels through that view's own index map
   (feet_from_map), the pixels the view was gathered from, so it doesn't depend on the lens the map was built with.
2. Dwells: one view's feet point within DWELL_PX of its median for DWELL_S or more (find_dwells); a dwell the full pass and
   a side view both saw is one, at the side view's point (merge_dwells).
3. Assignment. Each mark's pixel box over the lens bracket (f F_RANGE, k K_RANGE, yaw offset +-DELTA_MAX_DEG) gates which
   dwells may be it; the outer marks' boxes overlap, so the standing order decides: every order-keeping assignment of the
   dwells to the marks the boxes allow is fitted, and exactly one must pass every check below. A dwell no box takes, or
   two assignments that both fit, is refused.
4. The fit: f, k (r = f sin(k theta) / k, equidistant at k 0) and a yaw offset delta between the car's axis and the
   calibrated forward axis, least squares in angle: each feet pixel unprojected through the lens and the logged rpyCalib and
   wideFromDeviceEuler against the taped bearing and the ground elevation -atan(heightM / rangeM).
5. edgeDeg per side: the fitted lens's calibrated bearing at the frame's outer pixel column on the calibrated horizon.
6. The mask, from the drive's frames at MASK_HZ moving above MOVE_V in daylight (the sun up at the car) and calibrated
   within CALIB_MOVE_DEG of S1's, grouped by exposure (integLines x gain) into bins EXPOSURE_BAND wide; a group under
   MIN_GROUP_S is left out. In daylight the wide camera integrates 4-8 lines, so one line is a 15-25% step, enough to lift a
   static dark pixel's std over the threshold if the frames were pooled; rescaling luma by exposure would assume a linear
   tone curve, which the ISP's isn't. So each group is one exposure, in the full-range DN envelope_thresholds.json was
   measured in, and a pixel is blocked when, in any group, its temporal std is under STATIC_STD_DN and its mean under
   STATIC_MEAN_DN (sideview's values).
   Through the fitted lens each pixel has a calibrated bearing and elevation; cells are 1 deg of bearing by ELEV_STEP of
   elevation from ELEV_LO to ELEV_HI, blocked when CELL_FRAC or more of their pixels are, when they have none, or when any
   part of them is off the frame. A bearing's row is the run of clear cells through 0 deg elevation, for every whole
   bearing from 0 to edgeDeg - EDGE_MARGIN_DEG (stop_rules.footprint_ok keys rows by floor(bearing)); no clear run, no
   row, and stop_rules masks a bearing with no row.
7. darkThreshold: max(STATIC_MEAN_DN, the 99th percentile of the blocked pixels' means in the groups that blocked them +
   DARK_PAD_DN), DN.
8. veil: always unmeasured (sun ahead zeroes 67 deg on; S1 measures nothing about the veil).

Writes, only when everything passed: OUT.json, a copy of --vis (default the shipped file) with cameraSetbackM,
lateralOffsetM, heightM, lens, mask, darkThreshold and veil set and nothing else changed; OUT.report.txt (each mark's
residuals, delta, f, k, the edges, the mask's bands every 5 deg); OUT.provenance.json (the calibration and the segments).
The copy is checked with visibility_table.validate and camera_ready before it is written, and --check after. Exit 0 only
when both pass.

Refuses (exit 1, nothing written but a --save-detections file asked for): --out equal to --vis, the shipped file or an
input; a tape value missing or outside SETBACK_RANGE_M, HEIGHT_RANGE_M, LATERAL_MAX_M; S1 not still, not in Park, not
calibrated or its calibration moving, not a comma four; under MIN_INNER of the 40-80 deg marks on a side; a dwell no mark
takes, or the assignment not unique; bearing RMS over B_RMS_MAX, a residual over RESID_MAX, elevation RMS over E_RMS_MAX;
|delta| over DELTA_MAX_DEG (stop_rules.CAL_ANGLE_DEG, the calibration allowance in angle_bound); k or f outside the
bracket; a mark the fit puts EDGE_MARGIN_DEG or more inside the frame unseen, or a seen one it puts outside; under
MIN_MOVING_S of mask frames, or all of them within HEADING_DEG of one heading; the calibration moved between S1 and the
drive; S1's parked noise at the blocked pixels (or, with none, its dark ones) NOISE_MAX_DN or more. It never writes an
assumed lens, a bearing it has no data for, or a veil other than unmeasured.

Re-run it when the comma is re-mounted or calibration resets: the device recomputes the side views' maps after a
calibration move, but nothing re-checks the mask.
"""
import argparse
import json
import math
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from datetime import date
from pathlib import Path

import numpy as np

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot import visibility_table
from openpilot.selfdrive.turnpilot.detector import sideview
from openpilot.selfdrive.turnpilot.scene.ranging import Lens, calib_from_view

CAM_W, CAM_H = 1344, 760
DEVICE_TYPE, SENSOR = 'mici', 'os04c10'   # the comma four
REPO_F = 452.0                     # ranging.WIDE_FISHEYE_F: the lens the side-view maps are built with for detection
MARK_BEARINGS = (40, 55, 70, 80, 85, 88)
INNER_BEARINGS = (40, 55, 70, 80)  # inside the frame for every lens in the bracket
MIN_INNER = 4
RANGE_M = 10.
F_RANGE = (430., 475.)
K_RANGE = (0., 0.6)
K_EQUIDISTANT = 1e-3               # a fitted k under this is written as the equidistant lens, k 0
DELTA_MAX_DEG = sr.CAL_ANGLE_DEG
GATE_PX = 8.                       # px around a mark's bracket box a dwell may sit (the feet point's own error)
B_RMS_MAX, RESID_MAX, E_RMS_MAX = 0.5, 1.0, 0.5
DWELL_PX, DWELL_S, DWELL_GAP_S = 4., 4., 1.0
DWELL_MIN_POINTS = 8
MERGE_PX = 15.                     # px apart a full-pass and a side-view dwell at the same time are the same one
DETECT_HZ = 4.
MASK_HZ = 2.
CAMERA_HZ = 20.
MOVE_V = 3.                        # m/s
MIN_MOVING_S = 120.
HEADING_DEG = 30.
EXPOSURE_BAND = 0.10               # an exposure group's width (log bins)
MIN_GROUP_S = 20.                  # s of frames an exposure group needs to count (40 frames: the std to about 11%)
STATIC_STD_DN, STATIC_MEAN_DN = sideview.STATIC_STD_DN, sideview.STATIC_MEAN_DN
CELL_FRAC = 0.10
ELEV_LO, ELEV_HI, ELEV_STEP = -20., 5., 0.25
NOISE_MAX_DN = 3.0
DARK_PAD_DN = 5.
SETBACK_RANGE_M, HEIGHT_RANGE_M, LATERAL_MAX_M = (0.5, 3.0), (0.9, 1.8), 0.6
CALIB_MOVE_DEG = sideview.RECALIB_DEG
STILL_V = 0.1                      # m/s
PERSON = 0                         # COCO
SIDE_VIEWS = sideview.SIDE_VIEWS


class Refusal(Exception):
  """camera_fit refuses: the message says why. Nothing is written."""


@dataclass(frozen=True)
class Tape:
  setback: float
  lateral: float
  height: float
  date: str


@dataclass(frozen=True)
class Mark:
  side: str
  bearing: int

  @property
  def signed(self) -> float:
    """deg off the car's axis, right positive."""
    return float(self.bearing) if self.side == 'right' else -float(self.bearing)


@dataclass(frozen=True)
class Calib:
  rpy: tuple
  euler: tuple

  @property
  def rot(self) -> np.ndarray:
    return calib_from_view(list(self.rpy), list(self.euler))


@dataclass
class Dwell:
  t0: float
  t1: float
  u: float          # fisheye px (continuous: pixel i spans [i, i+1])
  v: float
  view: str         # 'full', 'sideLeft', 'sideRight'
  n: int


@dataclass
class Fit:
  f: float
  k: float
  delta: float      # deg: the car's axis is at calibrated bearing delta (right positive)
  assignment: list  # [(Dwell, Mark)]
  resid_b: list     # deg per assigned mark, in assignment order
  resid_e: list
  rms_b: float
  rms_e: float
  edge: dict = field(default_factory=dict)
  cost: float = 0.


@dataclass
class FrameMeta:
  t: float
  v: float
  heading: float | None
  sun_el: float
  exposure: float
  calibrated: bool
  calib: Calib | None
  seg: int = 0       # index of its segment
  n: int = 0         # its frame in the segment's video


# ------------------------------------------------------------------------------------------------------------ geometry

def calib_ray(signed_deg, elev_deg) -> np.ndarray:
  """Calibrated-frame ray (x forward, y right, z down) at a bearing (right positive) and elevation (up positive), deg."""
  b, e = np.radians(signed_deg), np.radians(elev_deg)
  return np.stack([np.cos(e) * np.cos(b), np.cos(e) * np.sin(b), -np.sin(e) * np.ones_like(b)], axis=-1)


def project(lens: Lens, rot: np.ndarray, ray) -> tuple[float, float]:
  u, v = lens.project_many(np.asarray(ray, float) @ rot)  # rot.T @ ray, row-wise: calib to view
  return float(u), float(v)


def unproject(lens: Lens, rot: np.ndarray, u: float, v: float) -> tuple[float, float]:
  """(bearing right positive, elevation up positive), deg, of a fisheye pixel in the calibrated frame."""
  x, y, z = rot @ lens.ray(u, v)
  return math.degrees(math.atan2(y, x)), math.degrees(math.atan2(-z, math.hypot(x, y)))


def mark_elev(height: float, range_m: float) -> float:
  return -math.degrees(math.atan2(height, range_m))


def mark_pixel(lens: Lens, rot: np.ndarray, side: str, bearing: float, height: float, range_m: float, delta: float):
  signed = (bearing if side == 'right' else -bearing) + delta
  return project(lens, rot, calib_ray(signed, mark_elev(height, range_m)))


def inside(u: float, v: float) -> bool:
  return 0. <= u <= CAM_W and 0. <= v <= CAM_H


def edge_deg(lens: Lens, rot: np.ndarray) -> dict:
  """{side: deg}: the calibrated bearing on the horizon at the outer pixel column's centre."""
  out = {}
  for side, sign, target in (('left', -1., 0.5), ('right', 1., CAM_W - 0.5)):
    lo, hi = 20., 135.
    for _ in range(60):
      mid = (lo + hi) / 2.
      u, _ = project(lens, rot, calib_ray(sign * mid, 0.))
      if (u > target) if side == 'left' else (u < target):
        lo = mid
      else:
        hi = mid
    out[side] = lo
  return out


def pixel_grid(lens: Lens, rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """(bearing right positive, elevation) deg of every pixel centre, (CAM_H, CAM_W)."""
  uu, vv = np.meshgrid(np.arange(CAM_W) + 0.5, np.arange(CAM_H) + 0.5)
  rays = lens.rays(uu, vv) @ rot.T
  x, y, z = rays[..., 0], rays[..., 1], rays[..., 2]
  return np.degrees(np.arctan2(y, x)), np.degrees(np.arctan2(-z, np.hypot(x, y)))


def lens_of(f: float, k: float) -> Lens:
  return Lens(CAM_W, CAM_H, float(f), fisheye=True, k=0. if k < K_EQUIDISTANT else float(k))


REPO_LENS = lens_of(REPO_F, 0.)


# ------------------------------------------------------------------------------------------------------------ dwells

def find_dwells(dets) -> list[Dwell]:
  """dets: (t, view, u, v) feet points. Per view, a run of points within DWELL_PX of its median, no gap over DWELL_GAP_S,
  lasting DWELL_S or more."""
  out = []
  for view in sorted({d[1] for d in dets}):
    pts = sorted((d for d in dets if d[1] == view), key=lambda d: d[0])
    clusters: list[list] = []
    for d in pts:
      best = None
      for c in clusters:
        if d[0] - c[-1][0] > DWELL_GAP_S:
          continue
        mu, mv = np.median([p[2] for p in c]), np.median([p[3] for p in c])
        dist = math.hypot(d[2] - mu, d[3] - mv)
        if dist <= DWELL_PX and (best is None or dist < best[0]):
          best = (dist, c)
      if best is None:
        clusters.append([d])
      else:
        best[1].append(d)
    for c in clusters:
      mu, mv = float(np.median([p[2] for p in c])), float(np.median([p[3] for p in c]))
      keep = [p for p in c if math.hypot(p[2] - mu, p[3] - mv) <= DWELL_PX]
      if len(keep) >= DWELL_MIN_POINTS and keep[-1][0] - keep[0][0] >= DWELL_S - 1e-9:
        out.append(Dwell(keep[0][0], keep[-1][0], float(np.median([p[2] for p in keep])), float(np.median([p[3] for p in keep])),
                         view, len(keep)))
  return sorted(out, key=lambda d: d.t0)


def merge_dwells(dwells: list[Dwell]) -> list[Dwell]:
  """One dwell per stand: a full-pass dwell overlapping a side view's in time (half the shorter) within MERGE_PX is dropped
  for the side view's, which has twice the pixels."""
  sides = [d for d in dwells if d.view != 'full']
  out = list(sides)
  for d in (d for d in dwells if d.view == 'full'):
    same = [s for s in sides if min(d.t1, s.t1) - max(d.t0, s.t0) >= 0.5 * min(d.t1 - d.t0, s.t1 - s.t0) and
            math.hypot(d.u - s.u, d.v - s.v) <= MERGE_PX]
    if not same:
      out.append(d)
  return sorted(out, key=lambda d: d.t0)


def feet_from_map(side_map, stride: int, x: float, y: float) -> tuple[float, float] | None:
  """A side-view point (continuous view pixels) to fisheye pixels through the view's own index map (sideview.SideMap: the
  luma tap each view pixel was gathered from and its bilinear weights), bilinear between the four view pixels around it;
  None where any of them lies off the fisheye frame."""
  w, h = sideview.SIDE_SIZE
  fx, fy = x - 0.5, y - 0.5
  j0, i0 = int(math.floor(fx)), int(math.floor(fy))
  a, b = fx - j0, fy - i0
  pts = {}
  for di in (0, 1):
    for dj in (0, 1):
      i, j = i0 + di, j0 + dj
      if not (0 <= i < h and 0 <= j < w):
        return None
      idx = i * w + j
      if side_map.weights[2, idx] == 0:
        return None
      yy, xx = divmod(int(side_map.luma_idx[idx]), stride)
      pts[(di, dj)] = (xx + side_map.weights[0, idx] / 255. + 0.5, yy + side_map.weights[1, idx] / 255. + 0.5)
  u = (pts[(0, 0)][0] * (1 - a) + pts[(0, 1)][0] * a) * (1 - b) + (pts[(1, 0)][0] * (1 - a) + pts[(1, 1)][0] * a) * b
  v = (pts[(0, 0)][1] * (1 - a) + pts[(0, 1)][1] * a) * (1 - b) + (pts[(1, 0)][1] * (1 - a) + pts[(1, 1)][1] * a) * b
  return float(u), float(v)


# ---------------------------------------------------------------------------------------------------- assignment and fit

def mark_box(rot: np.ndarray, m: Mark, height: float, range_m: float) -> tuple[float, float, float, float]:
  """(u0, u1, v0, v1): where the mark can be over the lens bracket and the yaw allowance, GATE_PX wider."""
  us, vs = [], []
  for f in np.linspace(*F_RANGE, 4):
    for k in np.linspace(*K_RANGE, 7):
      lens = lens_of(f, k)
      for d in (-DELTA_MAX_DEG, 0., DELTA_MAX_DEG):
        u, v = mark_pixel(lens, rot, m.side, m.bearing, height, range_m, d)
        us.append(u)
        vs.append(v)
  return min(us) - GATE_PX, max(us) + GATE_PX, min(vs) - GATE_PX, max(vs) + GATE_PX


def check_lens_bracket(f: float, k: float) -> None:
  if not (F_RANGE[0] <= f <= F_RANGE[1]) or not (K_RANGE[0] - 1e-9 <= k <= K_RANGE[1]):
    raise Refusal(f"the fitted lens (f {f:.1f} px, k {k:.3f}) is outside the bracket (f {F_RANGE[0]:.0f}-{F_RANGE[1]:.0f}, " +
                  f"k {K_RANGE[0]}-{K_RANGE[1]})")


def fit_lens(pairs, rot: np.ndarray, height: float, range_m: float) -> Fit:
  from scipy.optimize import least_squares
  el = mark_elev(height, range_m)

  def resid(x):
    lens = Lens(CAM_W, CAM_H, float(x[0]), fisheye=True, k=float(x[1]))
    rb, re = [], []
    for dw, m in pairs:
      b, e = unproject(lens, rot, dw.u, dw.v)
      rb.append(b - x[2] - m.signed)
      re.append(e - el)
    return np.r_[rb, re]
  sol = least_squares(resid, x0=[REPO_F, 0.3, 0.], bounds=([380., 0., -5.], [530., 0.95, 5.]), x_scale=[10., 0.1, 0.5])
  f, k, d = (float(x) for x in sol.x)
  r = resid(sol.x)
  n = len(pairs)
  rb, re = [float(x) for x in r[:n]], [float(x) for x in r[n:]]
  return Fit(f, k, d, list(pairs), rb, re, float(np.sqrt(np.mean(np.square(rb)))), float(np.sqrt(np.mean(np.square(re)))),
             cost=float(sol.cost))


def judge_fit(fit: Fit, marks: list[Mark], rot: np.ndarray, height: float, range_m: float) -> str:
  """Why the fitted assignment fails S1's checks, or ''."""
  seen = {(m.side, m.bearing) for _, m in fit.assignment}
  for side in ('left', 'right'):
    inner = sum(1 for s, b in seen if s == side and b in INNER_BEARINGS)
    if inner < MIN_INNER:
      return f"{inner} of the 40-80 deg marks seen on the {side} side, {MIN_INNER} needed"
  try:
    check_lens_bracket(fit.f, fit.k)
  except Refusal as e:
    return str(e)
  if abs(fit.delta) > DELTA_MAX_DEG + 1e-9:
    return f"a yaw offset of {fit.delta:+.2f} deg between the car's axis and calibration, over {DELTA_MAX_DEG} deg: recalibrate (step 16)"
  worst = max((abs(x) for x in fit.resid_b + fit.resid_e), default=0.)
  if fit.rms_b > B_RMS_MAX:
    return f"bearing RMS {fit.rms_b:.2f} deg, over {B_RMS_MAX}"
  if worst > RESID_MAX:
    return f"a residual of {worst:.2f} deg, over {RESID_MAX}"
  if fit.rms_e > E_RMS_MAX:
    return f"elevation RMS {fit.rms_e:.2f} deg, over {E_RMS_MAX}: check heightM and that the lot is level"
  lens = lens_of(fit.f, fit.k)
  el = mark_elev(height, range_m)
  for m in marks:
    signed = m.signed + fit.delta
    if (m.side, m.bearing) in seen:
      if not inside(*project(lens, rot, calib_ray(signed, el))):
        return f"the {m.side} {m.bearing} deg mark was seen but the fit puts it outside the frame"
    else:
      out = signed + (sr.EDGE_MARGIN_DEG if signed > 0 else -sr.EDGE_MARGIN_DEG)
      if inside(*project(lens, rot, calib_ray(out, el))):
        return f"the {m.side} {m.bearing} deg mark is unseen though the fit puts it {sr.EDGE_MARGIN_DEG} deg or more inside the frame"
  return ''


def assignments(dwells: list[Dwell], marks: list[Mark], ok) -> list[list[int]]:
  """Every order-keeping map of the dwells (time order) to distinct marks (standing order) that ok(dwell, mark) allows."""
  out = []

  def go(i, j0, acc):
    if i == len(dwells):
      out.append(list(acc))
      return
    for j in range(j0, len(marks)):
      if ok(i, j):
        acc.append(j)
        go(i + 1, j + 1, acc)
        acc.pop()
  go(0, 0, [])
  return out


def solve(dwells: list[Dwell], marks: list[Mark], cal: Calib, height: float, range_m: float = RANGE_M) -> Fit:
  """The fit of the one assignment of dwells to marks that passes every check, or a Refusal."""
  for side in ('left', 'right'):
    n = sum(1 for m in marks if m.side == side and m.bearing in INNER_BEARINGS)
    if n < MIN_INNER:
      raise Refusal(f"MARKS has {n} of the 40-80 deg marks on the {side} side, {MIN_INNER} needed")
  if not dwells:
    raise Refusal("no dwell found: nobody stood still on a mark for 4 s in S1's recording")
  rot = cal.rot
  boxes = [mark_box(rot, m, height, range_m) for m in marks]
  sides = ['right' if unproject(REPO_LENS, rot, d.u, d.v)[0] > 0. else 'left' for d in dwells]

  def ok(i, j):
    u0, u1, v0, v1 = boxes[j]
    return sides[i] == marks[j].side and u0 <= dwells[i].u <= u1 and v0 <= dwells[i].v <= v1
  for i, d in enumerate(dwells):
    if not any(ok(i, j) for j in range(len(marks))):
      raise Refusal(f"the dwell at {d.t0:.1f}-{d.t1:.1f} s ({d.view}, u {d.u:.1f} v {d.v:.1f}, {sides[i]}) matches no mark: " +
                    "someone else standing still, or a mark off its spot")
  cands = assignments(dwells, marks, ok)
  if not cands:
    raise Refusal("the dwells don't follow MARKS' standing order: fix the order in MARKS.json")
  judged = []
  for c in cands:
    fit = fit_lens([(dwells[i], marks[j]) for i, j in enumerate(c)], rot, height, range_m)
    judged.append((judge_fit(fit, marks, rot, height, range_m), fit))
  good = [f for why, f in judged if not why]
  if len(good) > 1:
    raise Refusal(f"{len(good)} assignments of the dwells to the marks fit, so the marks can't be told apart: " +
                  '; '.join(', '.join(f"{m.side[0]}{m.bearing}" for _, m in f.assignment) for f in good))
  if not good:
    why, fit = min(judged, key=lambda x: x[1].cost)
    raise Refusal(f"{why} (the best of {len(judged)} assignments: " + ', '.join(f"{m.side[0]}{m.bearing}" for _, m in fit.assignment) + ")")
  fit = good[0]
  lens = rounded_lens(fit)
  fit.f, fit.k = lens.f, lens.k
  fit.edge = {s: math.floor(e * 100.) / 100. for s, e in edge_deg(lens, rot).items()}
  return fit


def rounded_lens(fit: Fit) -> Lens:
  """The lens as written: f to 0.01 px, k to 1e-4 (0 under K_EQUIDISTANT)."""
  return lens_of(round(fit.f, 2), round(fit.k, 4))


def fitted_lens(fit: Fit) -> Lens:
  return lens_of(fit.f, fit.k)


# ------------------------------------------------------------------------------------------------------------ the mask

class Accumulator:
  """Per-pixel temporal mean and std of full-range DN frames."""
  def __init__(self, shape):
    self.s = np.zeros(shape, np.float64)
    self.ss = np.zeros(shape, np.float64)
    self.n = 0

  def add(self, frame) -> None:
    f = np.asarray(frame, np.float64)
    self.s += f
    self.ss += f * f
    self.n += 1

  def stats(self) -> tuple[np.ndarray, np.ndarray]:
    mean = self.s / max(self.n, 1)
    return np.sqrt(np.maximum(self.ss / max(self.n, 1) - mean * mean, 0.)), mean


def static_dark(std: np.ndarray, mean: np.ndarray) -> np.ndarray:
  return (std < STATIC_STD_DN) & (mean < STATIC_MEAN_DN)


def dark_threshold(mean: np.ndarray, blocked: np.ndarray) -> float:
  if not blocked.any():
    return float(STATIC_MEAN_DN)
  return float(max(STATIC_MEAN_DN, float(np.percentile(mean[blocked], 99)) + DARK_PAD_DN))


def _off_frame_cells(lens: Lens, rot: np.ndarray, sign: float, nb: int, ne: int) -> np.ndarray:
  """(nb, ne) bool: any of a cell's corners, edge midpoints or centre projects off the frame."""
  bs = np.arange(0., nb + 1e-9, 0.5)
  es = ELEV_LO + np.arange(0., ne + 1e-9, 0.5) * ELEV_STEP
  bb, ee = np.meshgrid(bs, es, indexing='ij')
  u, v = lens.project_many(calib_ray(sign * bb, ee) @ rot)
  off = ~((u >= 0.) & (u <= CAM_W) & (v >= 0.) & (v <= CAM_H))
  out = np.zeros((nb, ne), bool)
  for db in (0, 1, 2):
    for de in (0, 1, 2):
      out |= off[db:db + 2 * nb:2, de:de + 2 * ne:2][:nb, :ne]
  return out


def mask_rows(blocked: np.ndarray, bearing: np.ndarray, elev: np.ndarray, edge: dict, lens: Lens, rot: np.ndarray) -> dict:
  """{side: [[bearingDeg, elevLoDeg, elevHiDeg], ...]} (module docstring, 6)."""
  ne = int(round((ELEV_HI - ELEV_LO) / ELEV_STEP))
  zero = int(round(-ELEV_LO / ELEV_STEP))  # the first cell at or above 0 deg
  out = {}
  for side, sign in (('left', -1.), ('right', 1.)):
    nb = int(math.floor(edge[side] - sr.EDGE_MARGIN_DEG)) + 1
    s = sign * bearing
    sel = (s >= 0.) & (s < nb) & (elev >= ELEV_LO) & (elev < ELEV_HI)
    flat = np.floor(s[sel]).astype(np.int64) * ne + np.clip(np.floor((elev[sel] - ELEV_LO) / ELEV_STEP).astype(np.int64), 0, ne - 1)
    count = np.bincount(flat, minlength=nb * ne).reshape(nb, ne)
    hits = np.bincount(flat, weights=blocked[sel].astype(np.float64), minlength=nb * ne).reshape(nb, ne)
    bad = (count == 0) | (hits >= CELL_FRAC * count) | _off_frame_cells(lens, rot, sign, nb, ne)
    rows = []
    for b in range(nb):
      row = bad[b]
      if row[zero - 1] or row[zero]:
        continue
      lo, hi = zero - 1, zero
      while lo - 1 >= 0 and not row[lo - 1]:
        lo -= 1
      while hi + 1 < ne and not row[hi + 1]:
        hi += 1
      rows.append([b, ELEV_LO + lo * ELEV_STEP, ELEV_LO + (hi + 1) * ELEV_STEP])
    out[side] = rows
  return out


def exposure_key(exposure: float) -> int:
  return int(math.floor(math.log(exposure) / math.log1p(EXPOSURE_BAND)))


def group_blocked(accs: dict) -> tuple[np.ndarray, float]:
  """(blocked in any exposure group, darkThreshold over the groups' means at the pixels each blocked)."""
  blocked, means = None, []
  for acc in accs.values():
    std, mean = acc.stats()
    b = static_dark(std, mean)
    blocked = b if blocked is None else blocked | b
    means.append(mean[b])
  if blocked is None:
    raise Refusal("no exposure group to build the mask from")
  pooled = np.concatenate(means) if means else np.zeros(0)
  dark = float(STATIC_MEAN_DN) if not len(pooled) else float(max(STATIC_MEAN_DN, float(np.percentile(pooled, 99)) + DARK_PAD_DN))
  return blocked, dark


def noise_floor(parked: Accumulator, blocked: np.ndarray) -> float | None:
  """S1's parked temporal noise, DN: the median std at the blocked pixels, else at the parked dark ones; None with neither."""
  std, mean = parked.stats()
  where = blocked if blocked.any() else (mean < STATIC_MEAN_DN)
  return float(np.median(std[where])) if where.any() else None


def check_noise(sigma: float | None) -> None:
  if sigma is None:
    raise Refusal("nothing blocked and nothing dark in S1's parked frames: the noise floor under the static test can't be checked")
  if sigma >= NOISE_MAX_DN:
    raise Refusal(f"S1's parked noise at the blocked pixels is {sigma:.2f} DN, {NOISE_MAX_DN} or more: the static test " +
                  f"({STATIC_STD_DN} DN) can't tell a blocked pixel from noise")


def select_mask_frames(frames: list[FrameMeta], s1: Calib) -> dict[int, list[FrameMeta]]:
  """The mask's frames by exposure group (module docstring, 6), or a Refusal."""
  cand = [f for f in frames if f.calibrated and f.calib is not None and f.v > MOVE_V and f.sun_el > 0. and f.heading is not None
          and f.exposure > 0.]
  for f in cand:
    moved = sideview.calib_moved_deg(f.calib.rpy, f.calib.euler, s1.rpy, s1.euler)
    if moved >= CALIB_MOVE_DEG:
      raise Refusal(f"the calibration moved {moved:.2f} deg between S1 and the drive (at {f.t:.1f} s): rerun S1 after it settles")
  groups: dict[int, list[FrameMeta]] = {}
  for f in cand:
    groups.setdefault(exposure_key(f.exposure), []).append(f)
  groups = {k: g for k, g in sorted(groups.items()) if len(g) / MASK_HZ >= MIN_GROUP_S - 1e-9}
  kept = [f for g in groups.values() for f in g]
  secs = len(kept) / MASK_HZ
  if secs < MIN_MOVING_S:
    raise Refusal(f"{secs:.0f} s of moving daylight frames in exposure groups of {MIN_GROUP_S:.0f} s or more, {MIN_MOVING_S:.0f} s " +
                  "needed: drive longer (step 16)")
  hs = sorted(f.heading % 360. for f in kept)
  gaps = [b - a for a, b in zip(hs, hs[1:], strict=False)] + [hs[0] + 360. - hs[-1]]
  if 360. - max(gaps) <= 2. * HEADING_DEG:
    raise Refusal(f"every mask frame is within {HEADING_DEG:.0f} deg of one heading: add a road 60 deg or more off (step 16)")
  return groups


# ------------------------------------------------------------------------------------------------------------ S1 and tape

def check_tape(raw) -> Tape:
  if not isinstance(raw, dict):
    raise Refusal("TAPE.json isn't an object")
  vals = {}
  for k in ('cameraSetbackM', 'lateralOffsetM', 'heightM'):
    x = raw.get(k)
    if not isinstance(x, (int, float)) or isinstance(x, bool) or not math.isfinite(x):
      raise Refusal(f"TAPE.json {k} is missing or not a number")
    vals[k] = float(x)
  try:
    day = date.fromisoformat(str(raw.get('date')))
  except ValueError:
    raise Refusal("TAPE.json date must be YYYY-MM-DD") from None
  if not SETBACK_RANGE_M[0] <= vals['cameraSetbackM'] <= SETBACK_RANGE_M[1]:
    raise Refusal(f"cameraSetbackM {vals['cameraSetbackM']} m is outside {SETBACK_RANGE_M[0]}-{SETBACK_RANGE_M[1]} m")
  if not HEIGHT_RANGE_M[0] <= vals['heightM'] <= HEIGHT_RANGE_M[1]:
    raise Refusal(f"heightM {vals['heightM']} m is outside {HEIGHT_RANGE_M[0]}-{HEIGHT_RANGE_M[1]} m")
  if abs(vals['lateralOffsetM']) > LATERAL_MAX_M:
    raise Refusal(f"lateralOffsetM {vals['lateralOffsetM']} m is over {LATERAL_MAX_M} m")
  return Tape(vals['cameraSetbackM'], vals['lateralOffsetM'], vals['heightM'], day.isoformat())


def check_marks(raw) -> tuple[float, list[Mark]]:
  if not isinstance(raw, dict) or not isinstance(raw.get('marks'), list):
    raise Refusal("MARKS.json must be {rangeM, marks: [{side, bearingDeg}, ...]}")
  rng = raw.get('rangeM')
  if not isinstance(rng, (int, float)) or isinstance(rng, bool) or abs(float(rng) - RANGE_M) > 1e-9:
    raise Refusal(f"MARKS.json rangeM must be {RANGE_M:g} (the marks are taped {RANGE_M:g} m out)")
  marks = []
  for x in raw['marks']:
    if not isinstance(x, dict) or x.get('side') not in ('left', 'right') or x.get('bearingDeg') not in MARK_BEARINGS:
      raise Refusal(f"MARKS.json mark {x!r} isn't {{side: left | right, bearingDeg: one of {', '.join(map(str, MARK_BEARINGS))}}}")
    marks.append(Mark(x['side'], int(x['bearingDeg'])))
  if len(set(marks)) != len(marks):
    raise Refusal("MARKS.json lists a mark twice")
  return float(rng), marks


def check_s1(device_type: str, sensor: str, size, car, cal) -> Calib:
  """car: (t, vEgo, gearShifter, standstill); cal: (t, calStatus, rpyCalib, wideFromDeviceEuler). S1's calibration."""
  if device_type != DEVICE_TYPE:
    raise Refusal(f"S1 was recorded on a {device_type}, not a comma four ({DEVICE_TYPE})")
  if sensor != SENSOR:
    raise Refusal(f"S1's wide camera is {sensor}, not the comma four's {SENSOR}")
  if tuple(size) != (CAM_W, CAM_H):
    raise Refusal(f"S1's ecamera is {size[0]}x{size[1]}, not {CAM_W}x{CAM_H}")
  if not car:
    raise Refusal("S1 has no carState: the car's motion and gear can't be checked")
  for t, v, gear, _ in car:
    if abs(v) > STILL_V:
      raise Refusal(f"the car moved during S1 ({v:.2f} m/s at {t:.1f} s)")
    if gear != 'park':
      raise Refusal(f"the car wasn't in Park during S1 ({gear} at {t:.1f} s)")
  if not cal:
    raise Refusal("S1 has no calibration messages")
  for t, status, rpy, euler in cal:
    if status != 'calibrated':
      raise Refusal(f"calibration was {status} during S1 (at {t:.1f} s): drive step 16 first")
    if len(rpy) != 3 or len(euler) != 3:
      raise Refusal(f"S1's calibration at {t:.1f} s has no rpyCalib or wideFromDeviceEuler")
  first = cal[0]
  for t, _, rpy, euler in cal:
    moved = sideview.calib_moved_deg(rpy, euler, first[2], first[3])
    if moved >= CALIB_MOVE_DEG:
      raise Refusal(f"the calibration moved {moved:.2f} deg during S1 (at {t:.1f} s)")
  return Calib(tuple(float(np.median([c[2][i] for c in cal])) for i in range(3)),
               tuple(float(np.median([c[3][i] for c in cal])) for i in range(3)))


# ------------------------------------------------------------------------------------------------------------ output

def output(vis_raw: dict, tape: Tape, fit: Fit, mask: dict, dark: float) -> dict:
  out = dict(vis_raw)
  out.update({'cameraSetbackM': tape.setback, 'lateralOffsetM': tape.lateral, 'heightM': tape.height,
              'lens': {'model': 'equidistant' if fit.k < K_EQUIDISTANT else 'equisolid', 'f': round(fit.f, 2),
                       'k': 0. if fit.k < K_EQUIDISTANT else round(fit.k, 4), 'edgeDeg': dict(fit.edge)},
              'mask': {s: [[int(b), round(lo, 2), round(hi, 2)] for b, lo, hi in mask[s]] for s in ('left', 'right')},
              'darkThreshold': math.ceil(float(dark) * 10. - 1e-9) / 10., 'veil': 'unmeasured'})  # up: never under the measure
  return out


def report(fit: Fit, mask: dict, dark: float, sigma: float, n_mask: int, tape: Tape) -> str:
  lines = [f"camera_fit {tape.date}: f {fit.f:.2f} px, k {fit.k:.4f}, yaw offset {fit.delta:+.3f} deg, bearing RMS {fit.rms_b:.3f} deg, " +
           f"elevation RMS {fit.rms_e:.3f} deg", f"frame edge: left {fit.edge['left']:.2f} deg, right {fit.edge['right']:.2f} deg",
           "marks (residual bearing, elevation; deg):"]
  for (dw, m), rb, re in zip(fit.assignment, fit.resid_b, fit.resid_e, strict=True):
    lines.append(f"  {m.side:5s} {m.bearing:2d}: {rb:+.2f} {re:+.2f}  ({dw.view} u {dw.u:.1f} v {dw.v:.1f}, {dw.t0:.1f}-{dw.t1:.1f} s)")
  lines.append(f"mask from {n_mask} frames ({n_mask / MASK_HZ:.0f} s); darkThreshold {dark:.1f} DN; S1 parked noise {sigma:.2f} DN")
  for side in ('left', 'right'):
    rows = {int(b): (lo, hi) for b, lo, hi in mask[side]}
    top = max(rows, default=-1)
    lines.append(f"  {side}: rows 0-{top} deg, {sum(1 for b in range(top + 1) if b not in rows)} bearings masked inside that")
    for b in range(0, top + 1, 5):
      lines.append(f"    {b:2d} deg: " + (f"{rows[b][0]:+.2f} to {rows[b][1]:+.2f}" if b in rows else 'masked'))
  return '\n'.join(lines) + '\n'


# ------------------------------------------------------------------------------------------------------------ logs and video

@dataclass
class SegmentLog:
  path: Path                 # the segment's directory
  device_type: str = ''
  sensor: str = ''
  size: tuple = (0, 0)
  car: list = field(default_factory=list)     # (t, vEgo, gear, standstill)
  cal: list = field(default_factory=list)     # (t, status, rpy, euler)
  cam: dict = field(default_factory=dict)     # frameId -> (t, exposure)
  enc: dict = field(default_factory=dict)     # segmentId -> frameId
  gps: list = field(default_factory=list)     # (t, lat, lon, bearing, speed, unix s)


def segments(paths) -> list[Path]:
  """The segment directories (an rlog and an ecamera.hevc) under the paths, in route and segment order."""
  out = set()
  for p in map(Path, paths):
    if not p.exists():
      continue
    for r in ([p] if p.is_file() else p.rglob('rlog*')):
      if r.name.split('.')[0] == 'rlog' and (r.parent / 'ecamera.hevc').is_file():
        out.add(r.parent)

  def key(d: Path):
    name, _, n = d.name.rpartition('--')
    if name and n.isdigit():
      return (str(d.parent), name, int(n))
    return (str(d.parent), '', int(d.name)) if d.name.isdigit() else (str(d.parent), d.name, -1)
  return sorted(out, key=key)


def _rlog(seg: Path) -> Path:
  return next(p for p in sorted(seg.iterdir()) if p.name.split('.')[0] == 'rlog')


def read_segment(seg: Path) -> SegmentLog:
  from openpilot.tools.lib.logreader import LogReader
  out = SegmentLog(seg)
  out.size = video_size(seg / 'ecamera.hevc')
  for msg in LogReader(str(_rlog(seg))):
    w, t = msg.which(), msg.logMonoTime * 1e-9
    if w == 'deviceState':
      out.device_type = str(msg.deviceState.deviceType)
    elif w == 'wideRoadCameraState':
      c = msg.wideRoadCameraState
      out.sensor = str(c.sensor)
      exp = float(c.integLines) * float(c.gain) if c.integLines > 0 and c.gain > 0 else float(c.exposureValPercent)
      out.cam[int(c.frameId)] = (c.timestampEof * 1e-9, exp)
    elif w == 'wideRoadEncodeIdx':
      out.enc[int(msg.wideRoadEncodeIdx.segmentId)] = int(msg.wideRoadEncodeIdx.frameId)
    elif w == 'carState':
      cs = msg.carState
      out.car.append((t, float(cs.vEgo), str(cs.gearShifter), bool(cs.standstill)))
    elif w == 'extrinsicsCalibration':
      c = msg.extrinsicsCalibration
      out.cal.append((t, str(c.calStatus), tuple(float(x) for x in c.rpyCalib), tuple(float(x) for x in c.wideFromDeviceEuler)))
    elif w in ('gpsLocationExternal', 'gpsLocation'):
      g = getattr(msg, w)
      if g.hasFix if hasattr(g, 'hasFix') else g.flags % 2 == 1:
        out.gps.append((t, float(g.latitude), float(g.longitude), float(g.bearingDeg), float(g.speed), g.unixTimestampMillis * 1e-3))
  for s in (out.car, out.cal, out.gps):
    s.sort(key=lambda x: x[0])
  return out


def _at(series, t):
  if not series:
    return None
  i = int(np.searchsorted([x[0] for x in series], t)) - 1
  return series[max(i, 0)]


def frame_metas(log: SegmentLog, seg: int, step: int) -> list[FrameMeta]:
  """Every step-th frame of the segment's video, with what the logs say at its time."""
  out = []
  for n in range(0, max(log.enc, default=-1) + 1, step):
    fid = log.enc.get(n)
    if fid is None or fid not in log.cam:
      continue
    t, exposure = log.cam[fid]
    car, cal, gps = _at(log.car, t), _at(log.cal, t), _at(log.gps, t)
    sun_el, heading = -90., None
    if gps is not None and abs(gps[0] - t) < 2.:
      sun_el = sr.sun_position(gps[1], gps[2], gps[5] + (t - gps[0]))[0]
      heading = gps[3] if gps[4] > MOVE_V else None
    calib = Calib(cal[2], cal[3]) if cal is not None and len(cal[2]) == 3 and len(cal[3]) == 3 else None
    out.append(FrameMeta(t, car[1] if car else 0., heading, sun_el, exposure, cal is not None and cal[1] == 'calibrated', calib, seg, n))
  return out


def video_size(path: Path) -> tuple[int, int]:
  probe = subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries', 'stream=width,height',
                                   '-of', 'csv=p=0', str(path)]).decode()
  w, h = map(int, probe.strip().split(',')[:2])
  return w, h


def video_frames(path: Path, step: int):
  """(n, tight NV12 bytes as uint8) for frames 0, step, 2 step, ... of a segment's hevc."""
  w, h = CAM_W, CAM_H
  size = w * h * 3 // 2
  sel = f"select='not(mod(n\\,{step}))'"
  proc = subprocess.Popen(['ffmpeg', '-v', 'error', '-f', 'hevc', '-i', str(path), '-vf', sel, '-fps_mode', 'passthrough', '-f', 'rawvideo',
                           '-pix_fmt', 'nv12', 'pipe:1'], stdout=subprocess.PIPE)
  assert proc.stdout is not None
  k = 0
  try:
    while True:
      buf = proc.stdout.read(size)
      if len(buf) < size:
        break
      yield k * step, np.frombuffer(buf, np.uint8)
      k += 1
  finally:
    proc.stdout.close()
    proc.wait()


def luma(nv12: np.ndarray) -> np.ndarray:
  return sideview.full_range(nv12[:CAM_W * CAM_H].reshape(CAM_H, CAM_W))


def accumulate(paths: list[Path], groups: dict[int, list[FrameMeta]], step: int) -> dict[int, Accumulator]:
  """Each exposure group's luma (full-range DN), decoded from the segments' videos in one pass."""
  accs = {k: Accumulator((CAM_H, CAM_W)) for k in groups}
  want = {(f.seg, f.n): k for k, g in groups.items() for f in g}
  for i, p in enumerate(paths):
    if not any(s == i for s, _ in want):
      continue
    for n, buf in video_frames(p / 'ecamera.hevc', step):
      if (i, n) in want:
        accs[want[(i, n)]].add(luma(buf))
  return accs


def camerad_nv12(img: np.ndarray) -> np.ndarray:
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  stride, y_height, uv_height, _ = get_nv12_info(CAM_W, CAM_H)
  out = np.zeros((y_height + uv_height, stride), dtype=np.uint8)
  out[:CAM_H, :CAM_W] = img[:CAM_H * CAM_W].reshape(CAM_H, CAM_W)
  out[y_height:y_height + CAM_H // 2, :CAM_W] = img[CAM_H * CAM_W:].reshape(CAM_H // 2, CAM_W)
  return out.reshape(-1)


def detect(paths: list[Path], logs: list[SegmentLog], cal: Calib) -> list[tuple]:
  """(t, view, u, v, conf) for every person's feet in S1's frames at DETECT_HZ: the full letterbox and both side views."""
  from tinygrad import Tensor, TinyJit
  from openpilot.selfdrive.turnpilot.detector import decoder, graph
  lb, full_fn = graph.make_detector(CAM_W, CAM_H)
  full, side = TinyJit(full_fn, prune=True), TinyJit(graph.make_side_detector(CAM_W, CAM_H), prune=True)
  geom = sideview.compute_geometry(REPO_LENS, cal.rpy, cal.euler, CAM_W, CAM_H)
  maps = {name: tuple(Tensor(a).realize() for a in (geom.maps[name].luma_idx, geom.maps[name].chroma_idx, geom.maps[name].weights))
          for name in SIDE_VIEWS}
  step = int(round(CAMERA_HZ / DETECT_HZ))
  out = []
  for p, log in zip(paths, logs, strict=True):
    for n, buf in video_frames(p / 'ecamera.hevc', step):
      fid = log.enc.get(n)
      if fid is None or fid not in log.cam:
        continue
      t = log.cam[fid][0]
      frame = Tensor(camerad_nv12(buf)).realize()
      for x0, _, x1, y1, conf, cls in decoder.decode(full(input_frame=frame).numpy(), lb):
        if int(cls) == PERSON:
          out.append((t, 'full', float((x0 + x1) / 2.), float(y1), float(conf)))
      for name in SIDE_VIEWS:
        luma_idx, chroma_idx, weights = maps[name]
        raw = np.asarray(side(input_frame=frame, luma_idx=luma_idx, chroma_idx=chroma_idx, weights=weights).numpy(), np.float32).reshape(-1, 6)
        for x0, _, x1, y1, conf, _cls in raw[(raw[:, 4] >= decoder.MIN_CONF) & (np.rint(raw[:, 5]) == PERSON)]:
          feet = feet_from_map(geom.maps[name], geom.stride, float((x0 + x1) / 2.), float(y1))
          if feet is not None:
            out.append((t, name, feet[0], feet[1], float(conf)))
  return out


# ------------------------------------------------------------------------------------------------------------ main

def _same(a: Path, b: Path) -> bool:
  try:
    return a.resolve() == b.resolve()
  except OSError:
    return False


def run(args) -> int:
  out = Path(args.out)
  vis = Path(args.vis)
  inputs = [vis, visibility_table.SHIPPED, Path(args.tape), Path(args.marks)] + ([Path(args.detections)] if args.detections else [])
  if any(_same(out, p) for p in inputs):
    raise Refusal(f"--out {out} is an input or the shipped visibility.json: write a copy")
  stem = out.with_suffix('')
  side_files = [Path(f"{stem}.report.txt"), Path(f"{stem}.provenance.json")]
  if args.save_detections and any(_same(Path(args.save_detections), p) for p in inputs + [out] + side_files):
    raise Refusal("--save-detections would overwrite an input or an output")
  try:
    tape = check_tape(json.loads(Path(args.tape).read_text()))
    range_m, marks = check_marks(json.loads(Path(args.marks).read_text()))
    vis_raw = json.loads(vis.read_text())
  except (OSError, ValueError) as e:
    raise Refusal(f"an input doesn't read: {type(e).__name__}: {e}") from None
  visibility_table.validate(vis_raw)
  s1_paths, drive_paths = segments(args.s1), segments(args.drive)
  if not s1_paths:
    raise Refusal(f"no S1 segment (an rlog beside an ecamera.hevc) under {', '.join(args.s1)}")
  if not drive_paths:
    raise Refusal(f"no drive segment under {', '.join(args.drive)}")
  s1_logs = [read_segment(p) for p in s1_paths]
  for lg in s1_logs:
    check_s1(lg.device_type, lg.sensor, lg.size, lg.car, lg.cal)
  cal = check_s1(s1_logs[0].device_type, s1_logs[0].sensor, s1_logs[0].size, [c for lg in s1_logs for c in lg.car],
                 [c for lg in s1_logs for c in lg.cal])
  if args.detections:
    rec = json.loads(Path(args.detections).read_text())
    if rec.get('segments') != [str(p) for p in s1_paths]:
      raise Refusal(f"--detections {args.detections} was made from other segments than --s1's")
    dets = [tuple(d) for d in rec['dets']]
  else:
    dets = detect(s1_paths, s1_logs, cal)
    if args.save_detections:
      Path(args.save_detections).write_text(json.dumps({'tool': 'camera_fit.py', 'segments': [str(p) for p in s1_paths],
                                                        'calib': asdict(cal), 'dets': dets}) + '\n')
  dwells = merge_dwells(find_dwells([d[:4] for d in dets]))
  fit = solve(dwells, marks, cal, tape.height, range_m)
  lens, rot = fitted_lens(fit), cal.rot
  # the mask: the drive at MASK_HZ, and S1 parked for the noise floor
  step = int(round(CAMERA_HZ / MASK_HZ))
  drive_logs = [read_segment(p) for p in drive_paths]
  for lg in drive_logs:
    if lg.device_type != DEVICE_TYPE or lg.size != (CAM_W, CAM_H):
      raise Refusal(f"the drive segment {lg.path} isn't the comma four's ({lg.device_type}, {lg.size})")
  metas = [m for i, lg in enumerate(drive_logs) for m in frame_metas(lg, i, step)]
  groups = select_mask_frames(metas, cal)
  n_mask = sum(len(g) for g in groups.values())
  blocked, dark = group_blocked(accumulate(drive_paths, groups, step))
  # S1's parked noise, in its largest exposure group
  parked: dict[int, list[FrameMeta]] = {}
  for m in (m for i, lg in enumerate(s1_logs) for m in frame_metas(lg, i, step) if m.exposure > 0.):
    parked.setdefault(exposure_key(m.exposure), []).append(m)
  big = max(parked.items(), key=lambda kv: len(kv[1]), default=(None, []))
  sigma = noise_floor(accumulate(s1_paths, {big[0]: big[1]}, step)[big[0]], blocked) if len(big[1]) >= 10 else None
  check_noise(sigma)
  bearing, elev = pixel_grid(lens, rot)
  mask = mask_rows(blocked, bearing, elev, fit.edge, lens, rot)
  result = output(vis_raw, tape, fit, mask, dark)
  visibility_table.validate(result)
  ready, missing = visibility_table.Visibility(result).camera_ready()
  if not ready:
    raise Refusal(f"the copy's camera fields aren't ready: {', '.join(missing)} empty")
  out.write_text(json.dumps(result, indent=1) + '\n')
  side_files[0].write_text(report(fit, mask, dark, sigma, n_mask, tape))
  side_files[1].write_text(json.dumps({'tool': 'camera_fit.py', 'tape': asdict(tape), 'rpyCalib': list(cal.rpy),
                                       'wideFromDeviceEuler': list(cal.euler), 's1': [str(p) for p in s1_paths],
                                       'drive': [str(p) for p in drive_paths], 'maskFrames': n_mask,
                                       'exposureGroups': {str(k): len(g) for k, g in groups.items()},
                                       'dwells': [asdict(d) for d in dwells]}, indent=1) + '\n')
  print(report(fit, mask, dark, sigma, n_mask, tape), end='')
  print(f"written to {out}, {side_files[0].name}, {side_files[1].name}")
  return 0 if visibility_table.main(['--check', str(out)]) == 0 else 1


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('--tape', required=True)
  ap.add_argument('--marks', required=True)
  ap.add_argument('--s1', required=True, nargs='+', help="S1's parked segments (directories searched for rlog + ecamera.hevc)")
  ap.add_argument('--drive', required=True, nargs='+', help="step 16's drive segments")
  ap.add_argument('--out', required=True, help='the copy of visibility.json to write (never --vis or the shipped file)')
  ap.add_argument('--vis', default=str(visibility_table.SHIPPED), help='the visibility.json to copy (default: the shipped one)')
  det = ap.add_mutually_exclusive_group()
  det.add_argument('--save-detections', help="write S1's person detections here, to re-run the fit without YOLO")
  det.add_argument('--detections', help='read S1 person detections a --save-detections run wrote')
  args = ap.parse_args(argv)
  try:
    return run(args)
  except (Refusal, visibility_table.VisibilityError) as e:
    print(f"refused: {e}", file=sys.stderr)
    return 1


if __name__ == '__main__':
  sys.exit(main())
