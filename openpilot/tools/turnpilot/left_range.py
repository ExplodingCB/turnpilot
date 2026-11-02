#!/usr/bin/env python3
"""left_range: TurnPilotLeftRangeM and TurnPilotLeftFarRangeM, measured on rlogs this fork recorded on the car in shadow
mode (the checklist's step 17 drive), never set by it (LEFT_RANGE.md).

  python3 -m openpilot.tools.turnpilot.left_range [--allow-misses] <rlog.zst or a directory of them> [...]

What the range has to mean. The unprotected left certifies a gap from the absence of road camera boxes (unprotected_left.py:
band_blockers): every full-pass run's vehicle boxes at RAW_CONF or more in the oncoming band whose nearest plausible range
(RawBox.near_m, the box bottom NEAR_PX lower) is within R_req break the clear window, and CLEAR_WINDOW_S of runs with none is
the go. A left is attempted only where R_req fits under TurnPilotLeftRangeM (turnpilotd's route time, reach_at_line), and
past it under TurnPilotLeftFarRangeM while the far crop reads, whose boxes in the band block at any range. So the range is
safe only if an oncoming car inside it is boxed on the road runs, with no stretch of them long enough for the window to
fill: CRITIQUE REQUIRED 2 sets the window at least 3x the longest dropout at R_req, so a car is confirmed from the
farthest near_m from which it never went MAX_DROPOUT_S (CLEAR_WINDOW_S / 3) without a box at the left. The drive is not the
left: its full pass is on the road camera every run and the car closes at the oncoming car's speed plus its own, while held
at the line the left's road runs are 2 in WIDE_EVERY (roadAndWide) and the car closes at its own speed alone. So a gap on
the drive counts as the longer of the two it would be there (left_gap_s): the same missed runs LEFT_SCHEDULE x as long, the
same stretch of road at the car's speed. The far crop's runs aren't thinned at the left, but its chain takes the same
(the safe side). R_req's own margins (the opposing road's limit x SPEED_MARGIN 1.3, PET_S, the lanes) are route time's,
and this measures only the eyes.

How. Every oncoming car the drive saw near is a sample: a turnpilotScene vehicle track (car, truck, bus, motorcycle) on the
road camera, velocity-trusted (scene_policy.velocity_trusted), SEED_MIN_M..SEED_MAX_M ahead in the oncoming lanes
(ONCOMING_Y_NEAR_M..ONCOMING_Y_FAR_M to the left: right-hand traffic) and closing over the ground at ONCOMING_MIN_MPS or more.
A car parked by the road stands still over the ground and is never one, nor is a lead. From the track's box on its run
(the fork's tracker publishes the box it matched, at the run's camera time) the car is followed back run by run through
the logged boxes, ranged and borne by the fork's own SceneTracker.raw_frame with that moment's calibration: the box in
the band (band_blockers) where the car would have been. Its range comes from the seed's own motion (its ground speed and
the car's own, carState.vEgo), never re-centred on a hit, so the box of anything closing at another rate (a parked car, a
lead) drifts out of the gate, within the tracker's ground gate grown SPEED_ERR_MPS a second back from the seed, and the
same from its last box (the range rate since it); its bearing within BEARING_GATE_DEG of its last box's lateral offset.
Its confirmed range is the farthest near_m of that chain, which ends at what would be MAX_DROPOUT_S without its box at
the left: unboxed (runs went on, none boxed it: the detector's reach, or the car wasn't there yet), noRuns (no road run: the full
pass on the wide camera, the detector stopped) or uncalibrated (the runs can't be ranged), all of them no range: unknown is
never a clear road. A run with any other band box inside AMBIGUOUS_GATE x those gates, or its one box there but outside
them, ends it too (ambiguous): something else could be taken for it there (a car close ahead or behind, one parked by the
lane, a lead), so no credit for a guess, and no box of another object is ever its. The far crop's range is the same chain
through the far crop's runs as well (turnpilotFarDetections, view far).

The recommendation is conservative. With n samples the k-th shortest confirmed range is a one-sided 95% lower confidence
bound on the 5th percentile (Wilks: P(Bin(n, 0.05) >= k) >= 0.95), so at 95% confidence 95% of oncoming cars are confirmed
by it, DESIGN section 6's 95% recall. By default k is 1, the shortest of them all, which needs MIN_VEHICLES (59: the rule of
three's 3/n under 5% says the same): every sampled car was confirmed by it, so CRITIQUE REQUIRED 2 holds for each one it
saw. --allow-misses (off by default: it loosens) takes Wilks' largest k, which lets the k-1 shortest fall short; they are
listed. Fewer than MIN_VEHICLES recommends nothing. It is rounded down to ROUND_M, never past what the samples reached, and
capped at LEFT_RANGE_MAX_M (unprotected_left's: turnpilotd and navd clamp the params to it). The far crop's is given only with
a road range (turnpilotd extends the reach by it only then) and past it. The preflight --set command is printed only with a
recommendation, for the owner to run parked.

It reads only: rlogs through LogReader, no params, no sockets. It reads no location service, so it prints no position. A
sunnypilot log (initData.gitRemote) is refused: sunnypilot's own messages sit on turnpilotDetections' union slot.
Exit 0 with a road range recommended, 2 without one, 1 for a log it can't use.
"""
import argparse
import bisect
import math
import sys
from collections import Counter
from dataclasses import dataclass, field, replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.turnpilot.detector.modeld_detector import WIDE_EVERY
from openpilot.selfdrive.turnpilot.route_follower import LEFT
from openpilot.selfdrive.turnpilot.scene.tracker import GROUND_GATE_FRAC, GROUND_GATE_M, NEAR_PX, iou, is_calibrated
from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_Y_FAR_M, ONCOMING_Y_NEAR_M, velocity_trusted
from openpilot.selfdrive.turnpilot.unprotected_left import CLEAR_WINDOW_S, LEFT_RANGE_MAX_M, ROUTE_REACH_MARGIN_M, band_blockers, reach_at_line

MAX_DROPOUT_S = CLEAR_WINDOW_S / 3.  # CRITIQUE REQUIRED 2: the window at least 3x the longest dropout, at the left
LEFT_SCHEDULE = WIDE_EVERY / (WIDE_EVERY - 1)  # held at its line (roadAndWide) the full pass is on the road camera 2 runs in 3
COVERAGE, CONFIDENCE = 0.95, 0.95    # DESIGN section 6: the range where recall is 95%; at 95% confidence
MIN_VEHICLES = 59                    # the fewest for which the shortest is that bound (1 - 0.95^n >= 0.95)
ROUND_M = 10.                        # recommended ranges round down to this
MOTOR_VEHICLES = ('car', 'truck', 'bus', 'motorcycle')  # the samples: what comes at a left's R_req speeds
SEED_MIN_M, SEED_MAX_M = 10., 80.    # m ahead an oncoming track is taken as a sample: near, its velocity trusted
ONCOMING_MIN_MPS = 3.                # m/s toward the car over the ground: moving, not parked (velocity noise near is ~1 m/s)
SPEED_ERR_MPS = 3.                   # the range gates grow by this x the time back from the seed, and from its last box
BEARING_GATE_DEG = 1.                # deg either side of where the car would be...
BEARING_RATE_DEG_S = 1.              # ...plus this per s back from its last box (the car's own yaw, a lane change)
AMBIGUOUS_GATE = 2.                  # any other band box inside this x both gates may be taken for it: no credit past there
SEED_IOU = 0.5                       # the track's box is its run's box at this overlap
DRIVE_GAP_S = 5.                     # log time this far ahead, or back past LOG_JITTER_S, starts another drive (route, boot);
                                     # so does another route's initData (Drives)
LOG_JITTER_S = 10.                   # messages are logged as they come, not in logMonoTime order (procLog 0.98 s behind on
                                     # Chase's route); a reboot this close in log time has its own initData
TRACK_GAP_S = 1.                     # a track id unseen this long (or the scene this long) is another track: a track coasts
                                     # MAX_COAST_S (0.5 s), and a new tracker numbers its tracks from 1 again
MPH = 0.44704
PREFLIGHT = "python3 -m openpilot.selfdrive.turnpilot.preflight"
REASONS = {
  'unboxed': "no box of it for {gap} with runs going (the detector's reach, or it wasn't there yet)",
  'noRuns': "no road run for {gap} (the full pass on the wide camera, or the detector stopped): unknown, no range",
  'uncalibrated': "runs not calibrated for {gap}: unknown, no range",
  'ambiguous': "another box where it could be, or its one box off its motion: no credit",
  'logStart': "the log starts",
}


@dataclass
class Run:
  t: float            # camera time, s (timestampEof)
  t_log: float        # logMonoTime, s
  view: str           # 'full' (the road camera's full pass) or 'far'
  det: object         # the run's detections (raw_frame's input)
  cal: object         # extrinsicsCalibration then, None before one
  crop: tuple | None = None
  frame: object = None  # RawFrame, None uncalibrated


@dataclass
class Sample:
  track_id: int
  cls: str
  t: float            # s into its drive the track became a sample
  speed: float        # m/s over the ground toward the car
  road_m: float = 0.  # confirmed range on the road camera's full pass, near_m
  road_why: str = ''
  road_gap_s: float = 0.
  road_ground_m: float = 0.  # the ground range (hypot of the box's xy) of the road chain's farthest box: said, never used
  far_m: float = 0.   # ...on the full pass and the far crop together
  far_why: str = ''
  far_gap_s: float = 0.
  far_ground_m: float = 0.
  boxes: frozenset = frozenset()  # (run index, box index) of its chain, to tell one car's two tracks apart
  drive: int = 0      # its drive's index (Drives): its runs and boxes are that drive's alone


@dataclass
class Pick:
  key: str
  n: int = 0
  k: int = 0
  stat_m: float | None = None   # the k-th shortest confirmed range
  range_m: int | None = None    # the recommendation, None for none
  why: str = ''
  misses: list = field(default_factory=list)


@dataclass
class Result:
  refused: str = ''
  samples: list = field(default_factory=list)
  road: Pick = field(default_factory=lambda: Pick('TurnPilotLeftRangeM'))
  far: Pick = field(default_factory=lambda: Pick('TurnPilotLeftFarRangeM'))
  allow_misses: bool = False
  facts: dict = field(default_factory=dict)


def wilks_k(n: int, coverage: float = COVERAGE, confidence: float = CONFIDENCE) -> int:
  """The largest k for which the k-th smallest of n is a one-sided lower confidence bound on the (1 - coverage) quantile:
  P(Bin(n, 1 - coverage) >= k) >= confidence. 0 when even the smallest isn't."""
  p, k = 1. - coverage, 0
  pmf = [math.comb(n, i) * p ** i * (1. - p) ** (n - i) for i in range(n + 1)]
  tail = 1.
  for i in range(n + 1):  # tail = P(X >= i + 1) after taking pmf[i] off
    tail -= pmf[i]
    if tail >= confidence - 1e-12:
      k = i + 1
    else:
      break
  return k


def supported_mph(range_m: float, lanes: int | None = None) -> int:
  """The fastest mapped opposing limit (whole mph) whose left route time attempts under range_m, at a junction with no
  mapped stop line (reach_at_line with ROUTE_REACH_MARGIN_M), lanes None the unknown-lanes rule; 0 for none."""
  mph = 0
  while reach_at_line((mph + 1) * MPH, lanes, ROUTE_REACH_MARGIN_M) <= range_m and mph < 200:
    mph += 1
  return mph


def snapshot_detections(d) -> SimpleNamespace:
  dets = [SimpleNamespace(cls=int(x.cls), conf=float(x.conf), x0=float(x.x0), y0=float(x.y0), x1=float(x.x1), y1=float(x.y1))
          for x in d.detections]
  return SimpleNamespace(detections=dets, lightBoxes=(), signs=())


class Drive:
  """What one drive's logs hold that the measure needs, fed one message at a time (LogReader order): one stretch of
  continuous log time (Drives), so one time base. prev: the drive before it, whose software and camera it starts with (an
  initData or deviceState says otherwise), never its calibration."""
  def __init__(self, prev: 'Drive | None' = None):
    self.remote = prev.remote if prev is not None else None
    self.device_type = prev.device_type if prev is not None else ''
    self.sensor = prev.sensor if prev is not None else ''
    self.cal = None
    self.ego_t: list[float] = []
    self.ego_v: list[float] = []
    self.runs: list[Run] = []
    self.scene: list[tuple[float, list]] = []
    self.far_status = Counter()
    self.wide_runs = 0
    self.t0: float | None = None
    self.t1 = 0.
    self.scene_t: float | None = None  # the last turnpilotScene's log time
    self.epoch = 0                     # the tracker's: one more at an unavailable scene, or none for TRACK_GAP_S
    self.tracks: dict[int, tuple] = {} # track id -> (its key, log time last seen, epoch then)

  def feed(self, msg) -> None:
    which = msg.which()
    t_log = msg.logMonoTime * 1e-9
    if which == 'initData':  # the route's, built once and written first in every segment: its time is the route's start
      self.remote = str(msg.initData.gitRemote)
      if not self.device_type and str(msg.initData.deviceType) != 'unknown':
        self.device_type = str(msg.initData.deviceType)
      return
    self.t0 = t_log if self.t0 is None else self.t0
    self.t1 = max(self.t1, t_log)
    if self.sunnypilot:
      return  # its own messages on our union slots: never read
    if which == 'deviceState' and str(msg.deviceState.deviceType) != 'unknown':
      self.device_type = str(msg.deviceState.deviceType)
    elif which == 'narrowRoadCameraState':
      self.sensor = str(msg.narrowRoadCameraState.sensor)
    elif which == 'extrinsicsCalibration':
      c = msg.extrinsicsCalibration
      self.cal = SimpleNamespace(rpyCalib=list(c.rpyCalib), calStatus=str(c.calStatus), height=list(c.height),
                                 wideFromDeviceEuler=list(c.wideFromDeviceEuler))
    elif which == 'carState':
      self.ego_t.append(t_log)
      self.ego_v.append(float(msg.carState.vEgo))
    elif which == 'turnpilotDetections':
      d = msg.turnpilotDetections
      self.far_status[str(d.farStatus)] += 1
      if str(d.view) != 'full':
        return  # a stop sweep's side views
      if str(d.camera) != 'road':
        self.wide_runs += 1
        return
      if d.timestampEof > 0:
        self.runs.append(Run(d.timestampEof * 1e-9, t_log, 'full', snapshot_detections(d), self.cal))
    elif which == 'turnpilotFarDetections':
      f = msg.turnpilotFarDetections
      if f.run.timestampEof > 0 and str(f.run.view) == 'far':
        self.runs.append(Run(f.run.timestampEof * 1e-9, t_log, 'far', snapshot_detections(f.run), self.cal, tuple(f.crop)))
    elif which == 'turnpilotScene':
      sc = msg.turnpilotScene
      if not sc.available or (self.scene_t is not None and t_log - self.scene_t > TRACK_GAP_S):
        self.epoch += 1  # a scene that failed (turnpilotd starts a new tracker) or a turnpilotd that restarted
      self.scene_t = t_log
      if sc.available:
        self.scene.append((t_log, [SimpleNamespace(trackId=int(o.trackId), key=self.track_key(int(o.trackId), t_log),
                                                   cls=str(o.cls), x=float(o.x), y=float(o.y), vx=float(o.vx),
                                                   vy=float(o.vy), age=int(o.age), camera=str(o.camera), view=str(o.view),
                                                   box=tuple(o.box), tSeen=float(o.tSeen))
                                   for o in sc.objects]))

  def track_key(self, track_id: int, t_log: float) -> tuple:
    """Which track this id is: another one after the tracker started again (epoch) or the id went unseen TRACK_GAP_S. One
    car's track split in two is merged back by its boxes (merge), but two cars on one key would drop the second."""
    seen = self.tracks.get(track_id)
    key = seen[0] if seen is not None and seen[2] == self.epoch and t_log - seen[1] <= TRACK_GAP_S else \
      (track_id, self.epoch, t_log)
    self.tracks[track_id] = (key, t_log, self.epoch)
    return key

  @property
  def sunnypilot(self) -> bool:
    return self.remote is not None and 'sunnypilot' in self.remote.lower()

  @property
  def empty(self) -> bool:
    """Nothing to measure: no detector run and no scene."""
    return not self.runs and not self.scene and not self.wide_runs and not self.far_status


class Drives:
  """Every file's messages in order, split into one Drive per route and per stretch of continuous log time: another route
  (its own turnpilotd, whose tracker numbers its tracks from 1 again) or another boot (logMonoTime starts over) is measured on
  its own time base, so no car's chain runs through another drive's boxes and no track id is taken for another drive's.
  loggerd builds a route's initData once and writes it first in every segment: the same one is the same route, another one
  another route, and its time (the route's start) is never the log's time."""
  def __init__(self):
    self.drives: list[Drive] = []
    self.t_last: float | None = None
    self.init: tuple | None = None  # the current drive's initData (logMonoTime, wallTimeNanos)

  def new(self) -> None:
    self.drives.append(Drive(self.drives[-1] if self.drives else None))
    self.t_last = None

  def feed(self, msg) -> None:
    if msg.which() == 'initData':
      init = (msg.logMonoTime, msg.initData.wallTimeNanos)
      if not self.drives or (self.init is not None and init != self.init):
        self.new()
      self.init = init
    else:
      t = msg.logMonoTime * 1e-9
      if not self.drives or (self.t_last is not None and not (self.t_last - LOG_JITTER_S <= t <= self.t_last + DRIVE_GAP_S)):
        self.new()
      self.t_last = t if self.t_last is None else max(self.t_last, t)
    self.drives[-1].feed(msg)


class Ego:
  """The car's own distance travelled, from carState.vEgo over log time."""
  def __init__(self, t, v):
    order = np.argsort(np.asarray(t, dtype=np.float64), kind='stable')  # np.interp needs its times increasing
    self.t = np.asarray(t, dtype=np.float64)[order]
    v = np.asarray(v, dtype=np.float64)[order]
    self.s = np.concatenate([[0.], np.cumsum(np.diff(self.t) * (v[1:] + v[:-1]) / 2.)]) if len(self.t) else np.zeros(0)

  def between(self, a: float, b: float) -> float:
    return float(np.interp(b, self.t, self.s) - np.interp(a, self.t, self.s))


def left_gap_s(back: float, ego_m: float, speed: float) -> float:
  """How long a gap of back s with no box of the car on the drive would be at the left, held at its line: the longer of the
  same missed road runs at its schedule (roadAndWide: the full pass on the road camera WIDE_EVERY - 1 runs in WIDE_EVERY, so
  LEFT_SCHEDULE x as long; the drive's are all on it) and the same stretch of road at the car's own speed (the drive closed
  it at speed and the car's own, ego_m more; the car waiting at its line doesn't move)."""
  return max(back * LEFT_SCHEDULE, (speed * back + max(ego_m, 0.)) / speed)


def in_band(boxes) -> list:
  """The left's own band test on a straight approach (heading 0, its maneuver point at the car, reach unlimited)."""
  return band_blockers(boxes, 0., LEFT, math.inf, 0.)


def chain(runs: list[Run], seed: tuple, speed: float, ego: Ego, views: tuple) -> tuple[tuple[float, float], str, float, set]:
  """((confirmed near_m, that box's ground range), why it ends, s back to the run where it ended, its boxes as (run, box)
  indices) for the car whose
  box is seed (run index, box index, RawBox), closing at speed over the ground: back through the runs of these views
  (module docstring)."""
  i_seed, j_seed, last = seed
  t_seed, log_seed = runs[i_seed].t, runs[i_seed].t_log
  x_seed = last.xy[0]
  last_t, last_log = t_seed, log_seed
  far_box = last
  used = {(i_seed, j_seed)}
  seen_calibrated = seen_uncalibrated = False
  for i in range(i_seed - 1, -1, -1):
    r = runs[i]
    if r.view not in views:
      continue
    back = last_t - r.t
    if back <= 0.:
      continue
    if left_gap_s(back, ego.between(r.t_log, last_log), speed) > MAX_DROPOUT_S + 1e-6:
      why = 'unboxed' if seen_calibrated else 'uncalibrated' if seen_uncalibrated else 'noRuns'
      return (far_box.near_m, math.hypot(*far_box.xy)), why, back, used
    if r.frame is None:
      seen_uncalibrated = True
      continue
    seen_calibrated = True
    # where the car was, from its last box (the range rate since it, the bearing of its lateral offset: the car's own yaw)
    # and from the seed's own motion, never re-centred on a hit: a box closing at another rate drifts out of that gate
    y = last.xy[1]
    x = last.xy[0] + speed * back + ego.between(r.t_log, last_log)
    rng, bearing = math.hypot(x, y), math.degrees(math.atan2(y, x))
    gate_m = GROUND_GATE_M + GROUND_GATE_FRAC * rng + SPEED_ERR_MPS * back
    gate_deg = BEARING_GATE_DEG + BEARING_RATE_DEG_S * back
    since = t_seed - r.t
    rng_seed = math.hypot(x_seed + speed * since + ego.between(r.t_log, log_seed), y)
    gate_seed_m = GROUND_GATE_M + GROUND_GATE_FRAC * rng_seed + SPEED_ERR_MPS * since
    near = []  # every band box that could be taken for it: inside AMBIGUOUS_GATE x the gates
    for j, b in enumerate(r.frame.boxes):
      if b.xy is None or b.near_m is None or not in_band([b]):
        continue
      d_m, d_deg = abs(math.hypot(*b.xy) - rng), abs(b.bearing_deg - bearing)
      if d_m <= AMBIGUOUS_GATE * gate_m and d_deg <= AMBIGUOUS_GATE * gate_deg:
        near.append((j, b, d_m <= gate_m and d_deg <= gate_deg and abs(math.hypot(*b.xy) - rng_seed) <= gate_seed_m))
    if not near:
      continue
    if len(near) > 1 or not near[0][2]:
      return (far_box.near_m, math.hypot(*far_box.xy)), 'ambiguous', 0., used
    j, last, _ = near[0]
    last_t, last_log = r.t, r.t_log
    far_box = last if last.near_m > far_box.near_m else far_box
    used.add((i, j))
    seen_calibrated = seen_uncalibrated = False
  return (far_box.near_m, math.hypot(*far_box.xy)), 'logStart', 0., used


def seeds(drive: Drive) -> list:
  """The first snapshot of each track that is an oncoming car near (module docstring): (track, scene log time)."""
  out, done = [], set()
  for t_log, objects in drive.scene:
    for o in objects:
      if o.key in done or o.camera != 'road' or o.cls not in MOTOR_VEHICLES or not velocity_trusted(o):
        continue
      if SEED_MIN_M <= o.x <= SEED_MAX_M and ONCOMING_Y_NEAR_M <= o.y <= ONCOMING_Y_FAR_M and o.vx <= -ONCOMING_MIN_MPS and \
         abs(o.vy) <= abs(o.vx):
        done.add(o.key)
        out.append((o, t_log))
  return out


def seed_box(runs: list[Run], times: list[float], o):
  """(run index, box index, box) of the full-pass run the track's box came from (its tSeen), or None."""
  i = bisect.bisect_left(times, o.tSeen - 1e-4)
  while i < len(runs) and runs[i].t <= o.tSeen + 1e-4:
    r = runs[i]
    if r.view == 'full' and r.frame is not None and len(o.box) == 4 and r.frame.boxes:
      scores = iou(np.array([o.box]), np.array([b.box for b in r.frame.boxes]))[0]
      j = int(np.argmax(scores))
      b = r.frame.boxes[j]
      if scores[j] >= SEED_IOU and b.xy is not None and b.near_m is not None and in_band([b]):
        return i, j, b
    i += 1
  return None


def merge(samples: list[Sample]) -> list[Sample]:
  """One car's tracks (the tracker lost it and started again) as one sample: chains through a shared box, directly or
  through others, are one car, and it keeps the shortest road range and the shortest far range, each with its own chain's
  reason, so a count of n is n cars and no car reads longer than its shortest reading. In time order."""
  groups: list[list[Sample]] = []
  for s in samples:
    shares = [any(s.boxes & m.boxes for m in g) for g in groups]
    joined = [m for g, sh in zip(groups, shares, strict=True) if sh for m in g]
    groups = [g for g, sh in zip(groups, shares, strict=True) if not sh] + [[s] + joined]
  out = []
  for g in groups:
    road = min(g, key=lambda m: m.road_m)
    far = min(g, key=lambda m: m.far_m)
    out.append(replace(road, t=min(m.t for m in g), far_m=far.far_m, far_why=far.far_why, far_gap_s=far.far_gap_s,
                       far_ground_m=far.far_ground_m, boxes=frozenset().union(*(m.boxes for m in g))))
  return sorted(out, key=lambda s: s.t)


def pick(key: str, values: list[tuple[float, Sample]], allow_misses: bool, cap: float = LEFT_RANGE_MAX_M) -> Pick:
  """The recommendation over (confirmed range, sample) pairs (module docstring)."""
  n = len(values)
  p = Pick(key, n=n)
  if n < MIN_VEHICLES:
    p.why = f"{n} oncoming cars, {MIN_VEHICLES} needed: no range"
    return p
  k = wilks_k(n) if allow_misses else 1
  ordered = sorted(values, key=lambda v: v[0])
  p.k, p.stat_m = k, ordered[k - 1][0]
  p.range_m = int(math.floor(min(p.stat_m, cap) / ROUND_M) * ROUND_M)
  p.misses = [s for m, s in ordered if m < p.range_m]
  if p.range_m <= 0:
    p.range_m, p.why = None, f"the {ordinal(k)} shortest confirmed range is {p.stat_m:.0f} m: no range"
  return p


def ordinal(k: int) -> str:
  return {1: '', 2: 'second', 3: 'third'}.get(k, f"{k}th")


def refusal(drive: Drive) -> str:
  """Why this drive's log can't be used, '' if it can."""
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  if drive.sunnypilot:
    return f"a sunnypilot log ({drive.remote}): its own messages sit on turnpilotDetections' slot, so it is never read"
  if drive.remote is None:
    return "no initData: not a log this fork can be told from"
  if (drive.device_type, drive.sensor) not in DEVICE_CAMERAS:
    return f"no camera to range by (deviceType {drive.device_type or 'none'}, road sensor {drive.sensor or 'none'})"
  if not drive.ego_t:
    return "no carState: the car's own motion is unknown"
  return ''


def measure_drive(drive: Drive, index: int, facts: dict) -> list[Sample]:
  """One drive's samples on its own time base (its runs, its carState, its tracks), its counts added to facts."""
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.selfdrive.turnpilot.scene.tracker import SceneTracker
  tracker = SceneTracker(DEVICE_CAMERAS[(drive.device_type, drive.sensor)], drive.device_type)
  runs = sorted(drive.runs, key=lambda r: r.t)
  for r in runs:
    if is_calibrated(r.cal):
      r.frame = tracker.raw_frame(r.det, 'road', r.cal, r.t, r.view, r.crop)
  times = [r.t for r in runs]
  full = [r for r in runs if r.view == 'full']
  far_runs = [r for r in runs if r.view == 'far']
  for k, v in (('full', len(full)), ('calibrated', sum(r.frame is not None for r in full)), ('far', len(far_runs)),
               ('far_calibrated', sum(r.frame is not None for r in far_runs))):
    facts[k] = facts.get(k, 0) + v
  ego = Ego(drive.ego_t, drive.ego_v)
  samples = []
  for o, t_log in seeds(drive):
    found = seed_box(runs, times, o)
    if found is None:
      facts['seeds_without_box'] = facts.get('seeds_without_box', 0) + 1
      continue
    s = Sample(o.trackId, o.cls, t_log - (drive.t0 or 0.), -o.vx, drive=index)
    (s.road_m, s.road_ground_m), s.road_why, s.road_gap_s, used = chain(runs, found, s.speed, ego, ('full',))
    (s.far_m, s.far_ground_m), s.far_why, s.far_gap_s, used_far = chain(runs, found, s.speed, ego, ('full', 'far'))
    s.boxes = frozenset(used | used_far)
    samples.append(s)
  return merge(samples)


def measure(drives, allow_misses: bool = False) -> Result:
  """The recommendation over these drives (a Drive, or Drives.drives): each measured on its own, their samples pooled. Any
  drive refused (a sunnypilot log anywhere) refuses them all: a car of it unmeasured could be the shortest."""
  drives = [drives] if isinstance(drives, Drive) else list(drives)
  res = Result(allow_misses=allow_misses)
  measured = [d for d in drives if not d.empty] or drives[:1]
  far_status = sum((d.far_status for d in measured), Counter())
  res.facts = {'s': sum(d.t1 - (d.t0 or d.t1) for d in measured), 'drives': len(measured),
               'wide': sum(d.wide_runs for d in measured), 'far_status': dict(far_status), 'seeds_without_box': 0}
  sunny = next((d for d in drives if d.sunnypilot), None)
  if not drives:
    res.refused = "no messages"
  elif sunny is not None:
    res.refused = refusal(sunny)
  else:
    for k, d in enumerate(measured):
      if why := refusal(d):
        res.refused = (f"drive {k + 1} of {len(measured)}: " if len(measured) > 1 else '') + why
        break
  if res.refused:
    res.road.why = res.far.why = res.refused
    return res
  for k, d in enumerate(measured):
    res.samples += measure_drive(d, k, res.facts)
  res.samples.sort(key=lambda s: (s.drive, s.t))
  if res.facts['calibrated'] == 0:
    res.road.why = res.far.why = "no calibrated road run in the logs: nothing can be ranged"
    return res
  res.road = pick('TurnPilotLeftRangeM', [(s.road_m, s) for s in res.samples], allow_misses)
  far = Pick('TurnPilotLeftFarRangeM', n=len(res.samples))
  if not res.facts['far']:
    reading = far_status.get('read', 0)
    far.why = f"no turnpilotFarDetections in the logs, though the full pass's farStatus read on {reading} runs: the far runs were " + \
      "not logged, so nothing shows what the far crop boxed" if reading else \
      "far crop off: no turnpilotFarDetections in the logs, and the full pass's farStatus never read"
  elif res.road.range_m is None:
    far.why = "no road range: turnpilotd extends the reach by the far crop only past a measured TurnPilotLeftRangeM"
  else:
    far = pick('TurnPilotLeftFarRangeM', [(s.far_m, s) for s in res.samples], allow_misses)
    if far.range_m is not None and far.range_m <= res.road.range_m:
      far.why, far.range_m = f"the far crop adds no reach ({far.range_m} m, not past the road camera's {res.road.range_m} m)", None
  res.far = far
  return res


def sample_line(s: Sample, far: bool = False, drives: int = 1) -> str:
  m, why = (s.far_m, s.far_why) if far else (s.road_m, s.road_why)
  where = f"drive {s.drive + 1} " if drives > 1 else ""
  return f"    {where}t {s.t:7.1f} s  track {s.track_id} {s.cls} closing {s.speed:.0f} m/s: {m:.0f} m, " + \
    REASONS.get(why, why).format(gap=f"what would be over {MAX_DROPOUT_S:.2f} s at the left")


def pick_lines(p: Pick, label: str, far: bool, drives: int = 1) -> list[str]:
  out = [f"{label}: {p.n} oncoming cars"]
  if p.range_m is None:
    out.append(f"  no {p.key}: {p.why}")
    return out
  rule = f"the {ordinal(p.k) + ' ' if p.k > 1 else ''}shortest confirmed range of {p.n} (k={p.k})"
  passes = "the full pass or the far crop" if far else "the road camera's full pass"
  out.append(f"  recommend {p.key} {p.range_m} m: {rule}, {p.stat_m:.1f} m rounded down to {ROUND_M:.0f} m. A one-sided " +
             f"{CONFIDENCE:.0%} lower bound on the 5th percentile (Wilks): at {CONFIDENCE:.0%} confidence {COVERAGE:.0%} of oncoming " +
             f"cars are boxed by {passes} from there in, never {MAX_DROPOUT_S:.2f} s (CLEAR_WINDOW_S / 3) without a box " +
             "at the left (its road runs 2 in 3, the car closing at its own speed)")
  reading = " with the far crop reading" if far else ""
  out.append(f"  a left with no mapped stop line is attempted{reading} up to a mapped opposing limit of " +
             f"{supported_mph(p.range_m)} mph ({supported_mph(p.range_m, 1)} mph with one opposing lane mapped)")
  if p.misses:
    out.append(f"  short of it (--allow-misses): {len(p.misses)}")
    out += [sample_line(s, far, drives) for s in p.misses]
  return out


def render(res: Result) -> list[str]:
  f = res.facts
  if res.refused:
    return [f"left_range: refused: {res.refused}", "no recommendation"]
  drives = f.get('drives', 1)
  each = f" in {drives} drives, each on its own time base (a route, or a boot)" if drives > 1 else ""
  out = [f"left_range: {f.get('s', 0.):.0f} s of log{each}; road camera full-pass runs {f.get('full', 0)} " +
         f"({f.get('calibrated', 0)} calibrated), wide camera runs {f.get('wide', 0)}, far crop runs {f.get('far', 0)} " +
         f"({f.get('far_calibrated', 0)} calibrated)",
         f"oncoming cars sampled: {len(res.samples)} (tracks {SEED_MIN_M:.0f}-{SEED_MAX_M:.0f} m ahead in the oncoming lanes, " +
         f"closing {ONCOMING_MIN_MPS:.0f} m/s or more over the ground, velocity trusted); {MIN_VEHICLES} needed"]
  if f.get('seeds_without_box'):
    out.append(f"  {f['seeds_without_box']} more had no box on their own run (not counted)")
  if res.samples:
    road = sorted(s.road_m for s in res.samples)
    out.append(f"  confirmed range on the road camera, near_m: shortest {road[0]:.0f} m, 5th percentile " +
               f"{np.percentile(road, 5):.0f} m, median {np.median(road):.0f} m, longest {road[-1]:.0f} m")
    out.append(f"  (those boxes' own ground range, not used: median {np.median([s.road_ground_m for s in res.samples]):.0f} m; " +
               f"near_m takes the box bottom {NEAR_PX:.0f} px lower, as the left's gap rule does)")
    ends = Counter(s.road_why for s in res.samples)
    out.append("  road chains ended: " + ', '.join(f"{why} {n}" for why, n in ends.most_common()))
    if ends['ambiguous']:
      out.append("  (ambiguous: another box where the car could be, so no credit past there. Each one is that car's range, so " +
                 "drive where oncoming cars come singly, a few seconds apart, past no cars parked by the oncoming lanes)")
    out.append("  the shortest five:")
    out += [sample_line(s, drives=drives) for s in sorted(res.samples, key=lambda s: s.road_m)[:5]]
    if f.get('far'):
      far = sorted(s.far_m for s in res.samples)
      out.append(f"  confirmed range with the far crop too, near_m: shortest {far[0]:.0f} m, median {np.median(far):.0f} m, " +
                 f"longest {far[-1]:.0f} m")
  out += pick_lines(res.road, "road camera", False, drives)
  out += pick_lines(res.far, "far crop", True, drives)
  if res.road.range_m is not None:
    sets = [f"--set {res.road.key}={res.road.range_m}"]
    if res.far.range_m is not None:
      sets.append(f"--set {res.far.key}={res.far.range_m}")
    out.append("to set, on the comma, parked in P (the owner's call, not this tool's):")
    out.append(f"  {PREFLIGHT} {' '.join(sets)}")
    if res.far.range_m is not None:
      out.append("  the far range counts only with TurnPilotFarCrop on")
  else:
    out.append("nothing to set: TurnPilotLeftRangeM stays 0, and every unprotected left is skipped and rerouted")
  return out


def main(argv=None) -> int:
  from openpilot.tools.lib.logreader import LogReader
  from openpilot.tools.turnpilot.car_preflight import route_files
  ap = argparse.ArgumentParser(description="TurnPilotLeftRangeM and TurnPilotLeftFarRangeM from shadow-drive rlogs: read only")
  ap.add_argument('route', nargs='+', help="rlog files, or directories holding them")
  ap.add_argument('--allow-misses', action='store_true',
                  help="take Wilks' largest k, not the shortest: lets the k-1 shortest fall short (off: it loosens)")
  args = ap.parse_args(argv)
  try:
    paths = route_files(args.route)
  except FileNotFoundError as e:
    print(f"left_range: no such file or directory: {e}", file=sys.stderr)
    return 1
  if not paths:
    print("left_range: no rlog in " + ' '.join(args.route), file=sys.stderr)
    return 1
  drives = Drives()
  for p in paths:
    for msg in LogReader(str(Path(p)), only_union_types=True):
      drives.feed(msg)
  res = measure(drives.drives, args.allow_misses)
  print('\n'.join(render(res)))
  if res.refused:
    return 1
  return 0 if res.road.range_m is not None else 2


if __name__ == "__main__":
  sys.exit(main())
