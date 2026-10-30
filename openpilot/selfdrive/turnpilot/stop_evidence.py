"""The stop-sign decider's own track evidence (STOP_SIGNS.md 1.7, 2.3; SG3b): what each cross track did, on camera time.

StopEvidence keeps, per trackId (as scene_policy keys its tracks), a history of the track's matches: the camera time of each
(the tracker's Track.t_seen, TrackedObject.tSeen: never turnpilotd's clock), the bearing of its box's bottom centre, its box
width, the view that matched it, its class and its ground point, and where our car stood (d_stop, m short of the held
control's stop point). From it, per track, for SG4a's and SG4b's decider and for step S3's validation log:

- Its lane: the side its bearing is on, then its bearing against where each lane can be seen. A lane parallel to ours (the
  opposite approach's) is seen only under max_bearing plus ANGLE_BOUND (about 29 deg at the default junction), so a
  track past that is in a cross lane whatever its ground range says: ground range at 50-80 deg reads 1.5-2.5x long
  (STOP_SIGNS.md 1.2, verify.md finding 3), and by it a car through a cross lane at 12 m/s from 45 m read as an opposite
  car standing far off (SG3b skeptic p2), and with no cross lane on that side the track is in no listed lane. Under that
  bearing, where both can be, its ground position's forward distance decides (the opposite lanes beyond the cross road):
  inside 40 deg, the full pass's measured domain, where a cross car is within a few metres of our path. Which lane of a
  multi-lane cross approach it is in has no bearing cue at all (each lane is a line through the same bearings), so it is
  the nearest s band to its forward distance, and 15% short or 30% long moves a car one lane (p8): SG4b's to close before
  it reads a lane index. The position along the lane comes from the bearing ray against the map lane line, never from
  ground-plane range (off by about 30% at 50 m for half a degree of pitch).
- Distance and speed at the pessimistic ends of section 1.2 (stop_rules.fit_track): distance at sLaneMin with the bearing
  rotated toward the camera axis by ANGLE_BOUND, less 2 sigma; speed at sLaneMax with it rotated toward the frame edge, plus 2
  sigma; sigma from the fit's residuals over the trust window, the bearing noise floored at one pixel of the coarsest view
  among its samples (1/904 rad on a side view). Its speed is trusted after VEL_TRUST_S 1.0 s with 5 samples, its
  deceleration after DECEL_TRUST_S 1.5 s (0 before), tau-dot over that window, and the 1/w box expansion cross-checks it.
- Its nominal distance and speed (the middle of the s band, the bearing as measured), TTA to our conflict point (nominal and
  at the pessimistic ends), the s-free bearing-rate TTA to the camera's perpendicular foot, -sin(2 psi) / (2 dpsi/dt), as a
  cross-check, and the box expansion's time to contact (to the camera).
- U2 (stop_rules.u2_state), stopped at its line (trusted speed under SHIELD_V with its front within its line region, 3 m
  before its stop point to 2 m past it), the shield (stop_rules.shield_ok: a motor vehicle so for SHIELD_STOP_S, seen on
  every cycle of its view, not moved SHIELD_MOVE_M, on a stopping approach with no mapped bike lane and no shoulder of
  SHOULDER_PASS_M to pass it in: stop_rules.shield_lane_ok), and through at speed (a motor vehicle whose
  front passed its line at DOWNGRADE_V or more without dropping under it in the DOWNGRADE_WINDOW_M before: SG4b's downgrade,
  and a validation failure).

The opposite approach's lanes run parallel to ours, where a bearing says nothing about how far along one a car is (its
3.7 m offset is a few degrees at 50 m, inside the angle allowance): there the position is the ground range's at the near end of
an OPP_BAND band and the speed its fit's at the far end, and a car's box bottom is its front (head on).

Bearings taken from one place are only comparable from that place, so a track's fit uses only the samples taken with our
car where it stands now (within MOVE_TOL_M): moving restarts its trust, the conservative direction. Its first-track record
(the validation's first-track range) is taken where the car stood then, distance along the lane being where the car is on
the lane whoever looks. Nothing here acts: SG4a's decider reads it, and turnpilotd logs it for step S3.
"""
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.scene.tracker import MIN_HITS  # matches that make a track: its first-track record is taken there

SIDE_PIXEL_RAD = 1. / 904.   # rad, one pixel of a side view (f 904 px)
# one pixel of each view at its YOLO input on the comma four (1344x760 cameras, letterboxed to 640x384; the wide lens f 452 px,
# the road's about 1141 px): the full wide pass 2.1 sensor px, the road camera's full pass 2.1 of its px, the far crop native
PIXEL_RAD = {'sideLeft': SIDE_PIXEL_RAD, 'sideRight': SIDE_PIXEL_RAD, 'full': 2.1 / 452., 'road': 2.1 / 1141., 'far': 1. / 1141.}
PIXEL_RAD_UNKNOWN = 2.1 / 452.  # a view not listed: the coarsest above
HISTORY_S = 3.0              # s of matches kept per track (DECEL_TRUST_S and the shield's window inside it)
DROP_S = 2.0                 # s without a match before a track's history goes (its record stays for the log)
MOVE_TOL_M = 0.3             # m our car may have moved since a sample for the sample to count in a fit
CYCLE_GAP_S = 0.3            # s: a gap between matches over this missed a cycle of its view (5 Hz a view in a sweep)
CAR_HALF_LEN_M = 2.3         # m from a side-on car's box bottom centre to its front
DOWNGRADE_V = 5.0            # m/s, through its line at this or more (STOP_SIGNS.md 4.4, SG4b; validation fails the junction)
DOWNGRADE_WINDOW_M = 20.     # m before its line it must not have dropped under DOWNGRADE_V
SPEED_SPAN_S = 0.6           # s of matches behind each one its nominal speed is fitted over (through at speed)
APPROACH_V = 0.5             # m/s toward our path: moving toward the junction
DEFAULT_LINE_M = 10.         # m from the node along an approach to its stop point when nothing says (STOP_SIGNS.md 4.5 step 3)
CROSS_SKEW_DEG = 15.         # a lane within this of perpendicular is fitted as a cross lane (certify's skew check: 75-105)
OPP_BAND = 0.3               # a parallel lane's (the opposite approach's) position is the ground range's, within this fraction
MOTOR = ('car', 'truck', 'bus', 'motorcycle')

CLASS_NAMES = {0: 'pedestrian', 1: 'cyclist', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}  # COCO ids (the tracker's)


@dataclass
class Sample:
  t: float                   # camera time, s
  bearing: float             # deg left of the calibrated axis (the tracker's sign), its box's bottom centre
  width: float               # px
  view: str
  cls: str
  x: float                   # m forward (ground point, tracker frame)
  y: float                   # m left
  d_stop: float              # m our car stood short of the stop point


@dataclass
class First:
  """A track's first match on a lane: where the validation's first-track range comes from (STOP_SIGNS.md 4.3)."""
  t: float
  range_m: float             # m down its lane at the pessimistic distance end (sLaneMin, bearing toward the axis)
  view: str
  role: str
  index: int
  d_stop: float              # m our car stood short of the stop point then


@dataclass
class Record:
  """What the validation log keeps of a track over the encounter (turnpilotd: turnpilot.junction_evidence)."""
  track_id: int
  cls: str
  first: First
  need_m: float              # m, its lane's reach needed (R_req plus latency, all-way)
  last_t: float = 0.
  last_inside: bool = False
  max_gap_inside_s: float = 0.  # the longest gap between its matches while inside need_m
  approaching: bool | None = None  # moving toward our path when its speed was first trusted (None: never trusted)
  stopped_at_line: bool = False
  through_at_speed: bool = False
  v_max: float = 0.
  samples: int = 0


@dataclass
class TrackEvidence:
  track_id: int
  cls: str
  role: str
  index: int
  view: str
  t: float                   # camera time of its last match
  samples: int               # in its fit
  d_lo: float                # m to our conflict point, pessimistic (stop_rules.TrackFit)
  v_hi: float                # m/s toward it, pessimistic
  v_trusted: bool
  a_lo: float
  a_trusted: bool
  tau_dot: float | None
  expansion_agrees: bool | None
  d_nom: float               # m, nominal
  v_nom: float               # m/s toward our conflict point, nominal
  tta: float                 # s to our conflict point, nominal
  tta_lo: float              # s, pessimistic (d_lo / v_hi)
  tta_foot: float | None     # s to the camera's perpendicular foot from the bearing rate (cross lanes)
  ttc_expansion: float | None  # s to the camera from the box's expansion
  u2: str
  front_from_line_m: float   # m its front is past its stop point (+ toward our path)
  stopped_at_line: bool
  stopped_s: float
  shield: bool
  through_at_speed: bool
  lane: sr.Lane | None = None


@dataclass
class _State:
  stop_since: float | None = None
  stop_d: float = 0.
  speeds: list = field(default_factory=list)  # (front m down the lane, nominal speed) per match, for through at speed
  through: bool = False
  crossed: bool = False


def class_name(cls) -> str:
  return cls if isinstance(cls, str) else CLASS_NAMES.get(int(cls), 'other')


def lane_side(ln: sr.Lane) -> str:
  """The side of the frame its points are on (stop_rules' frame: y right)."""
  return 'left' if ln.point(10., 'min')[1] < 0. else 'right'


def is_cross(ln: sr.Lane) -> bool:
  return abs(abs(ln.rel_bearing) - 90.) <= CROSS_SKEW_DEG


def max_bearing(ln: sr.Lane, d_stop: float) -> float:
  """deg (unsigned): the largest bearing at which any point of ln, half a lane either side of its line, can be seen from
  d_stop m short of the stop point. The bearing of a point moving along a line changes monotonically, so it is the larger of
  its origin's, at sLaneMin and the far side of the lane, and the direction the lane leaves in: for the opposite approach's
  lanes (parallel to ours) atan((|y| + half a lane) / sLaneMin), 26.4 deg at the default 2-lane junction."""
  near = math.degrees(math.atan2(abs(ln.y) + sr.LANE_W_MAX / 2., max(ln.s_min + d_stop, 0.5)))
  return max(near, abs(ln.rel_bearing))


def junction_lanes(junction: sr.Junction, maneuver: str, setback: float, lane_edge_m: float | None = None,
                   yaw_deg: float = 0.) -> list[sr.Lane]:
  """The conflicting lanes of a held control's junction seen from its stop point (STOP_SIGNS.md 1.2, 2.1): one per physical
  lane (a right's swing movement shares the far lane's)."""
  from openpilot.selfdrive.turnpilot.routing.certify import cross_lanes
  edges = sr.edge_band(junction.stop_point_distance, cross_lanes(junction), lane_edge_m)
  out, seen = [], set()
  for ln in sr.conflict_lanes(junction, maneuver, edges, setback, yaw_deg):
    if (ln.role, ln.index) not in seen:
      seen.add((ln.role, ln.index))
      out.append(ln)
  return out


def _linfit(ts, ys) -> tuple[float, float]:
  """(value at the last t, slope) by least squares."""
  if len(ts) < 2 or ts[-1] - ts[0] <= 1e-6:
    return float(ys[-1]), 0.
  slope, icpt = np.polyfit(np.asarray(ts) - ts[-1], np.asarray(ys), 1)
  return float(icpt), float(slope)


def _rate_at_end(ts, ys) -> tuple[float, float]:
  """(value, rate) at the last t from a quadratic fit (a bearing's rate changes across a second at the side), a line
  with too few matches."""
  if len(ts) < 4 or ts[-1] - ts[0] <= 1e-6:
    return _linfit(ts, ys)
  c2, c1, c0 = np.polyfit(np.asarray(ts) - ts[-1], np.asarray(ys), 2)
  return float(c0), float(c1)


class StopEvidence:
  def __init__(self, pixel_rad: dict | None = None):
    self.pixel_rad = dict(PIXEL_RAD if pixel_rad is None else pixel_rad)
    self.lanes: list[sr.Lane] = []
    self.bound = 0.
    self.lines: dict[str, float] = {}
    self.reset()

  def reset(self) -> None:
    """A new held control: nothing carried over."""
    self.hist: dict[int, deque] = {}
    self.first: dict[int, First] = {}
    self.records: dict[int, Record] = {}
    self.states: dict[int, _State] = {}
    self.cache: dict[int, tuple] = {}  # trackId: (its last match's camera time, TrackEvidence): nothing changes between matches
    self.newest_t: float | None = None
    self.d_stop = 0.

  def set_junction(self, lanes: list[sr.Lane], bound_deg: float, lines: dict | None = None) -> None:
    """lanes: the held control's conflicting lanes seen from its stop point (junction_lanes); bound_deg: ANGLE_BOUND;
    lines: m from where each role's lanes cross our path to that approach's stop point (DEFAULT_LINE_M when not given)."""
    self.lanes, self.bound, self.lines = list(lanes), float(bound_deg), dict(lines or {})

  # ------------------------------------------------------------------------------------------------------------ input

  def observe(self, tracks, d_stop: float) -> None:
    """tracks: the tracker's tracks (Track: track_id, t_seen, bearing_seen, width_seen, view, cls, x); d_stop: m our car
    stands short of the stop point now. A match is taken once, by its camera time."""
    self.d_stop = float(d_stop)
    for trk in tracks:
      if getattr(trk, 'bearing_seen', None) is None or trk.x is None:
        continue
      t = float(trk.t_seen)
      h = self.hist.setdefault(trk.track_id, deque())
      if h and t <= h[-1].t + 1e-6:
        continue
      s = Sample(t, float(trk.bearing_seen), float(trk.width_seen or 0.), str(trk.view), class_name(trk.cls), float(trk.x[0]),
                 float(trk.x[1]), self.d_stop)
      h.append(s)
      while h and h[0].t < t - HISTORY_S:
        h.popleft()
      self.newest_t = t if self.newest_t is None else max(self.newest_t, t)
      self._note(trk.track_id, s, int(getattr(trk, 'hits', MIN_HITS)))
    if self.newest_t is not None:
      for tid in [k for k, h in self.hist.items() if not h or h[-1].t < self.newest_t - DROP_S]:
        del self.hist[tid]
        self.states.pop(tid, None)
        self.cache.pop(tid, None)

  def assign(self, s: Sample) -> sr.Lane | None:
    """The lane a match lies in (module docstring): by its bearing's side; then by its bearing, a lane that isn't a cross
    lane (the opposite approach's) only where its bearing can reach it (max_bearing plus ANGLE_BOUND); then, where both
    can, its forward distance against the lanes' s bands (with our car's offset then), the opposite approach's lanes beyond
    the cross road; among the cross lanes on its side, the nearest s band to its forward distance."""
    side = 'left' if s.bearing >= 0. else 'right'
    on_side = [ln for ln in self.lanes if lane_side(ln) == side]
    if not on_side:
      return None
    cross = [ln for ln in on_side if is_cross(ln)]
    other = [ln for ln in on_side if not is_cross(ln) and abs(s.bearing) <= max_bearing(ln, s.d_stop) + self.bound]
    if other and (not cross or s.x > max(ln.s_max for ln in cross) + s.d_stop + sr.LANE_W_MAX):
      return min(other, key=lambda ln: abs(abs(s.y) - abs(ln.y)))
    if not cross:
      return None
    return min(cross, key=lambda ln: abs(s.x - ((ln.s_min + ln.s_max) / 2. + s.d_stop)))

  def need(self, ln: sr.Lane) -> float:
    return sr.reach_needed_allway(ln.limit) if ln.limit > 0. else 0.

  def first_range(self, ln: sr.Lane, s: Sample) -> float:
    """m down ln at the pessimistic distance end (1.2): sLaneMin where our car stood, the bearing toward the axis; on a
    parallel lane the ground range's near end (module docstring)."""
    if not is_cross(ln):
      return max((1. - OPP_BAND) * s.x - (ln.s_min + s.d_stop), 0.) - ln.conflict_m
    psi = min(max(abs(s.bearing) - self.bound, 0.), 89.5)
    return (ln.s_min + s.d_stop) * math.tan(math.radians(psi)) - ln.conflict_m

  def _note(self, tid: int, s: Sample, hits: int) -> None:
    """The validation record of a new match: its first track (the match that made it a track, MIN_HITS: two hits make a
    track, 1.4), its gaps inside its lane's need."""
    rec = self.records.get(tid)
    if rec is None:
      if hits < MIN_HITS:
        return
      ln = self.assign(s)
      if ln is None:
        return
      first = First(s.t, self.first_range(ln, s), s.view, ln.role, ln.index, s.d_stop)
      self.first[tid] = first
      rec = self.records[tid] = Record(tid, s.cls, first, self.need(ln), s.t)
      rec.last_inside, rec.samples = first.range_m <= rec.need_m, 1
      return
    ln = next((x for x in self.lanes if (x.role, x.index) == (rec.first.role, rec.first.index)), None)
    inside = ln is not None and self.first_range(ln, s) <= rec.need_m
    if inside or rec.last_inside:
      rec.max_gap_inside_s = max(rec.max_gap_inside_s, s.t - rec.last_t)
    rec.last_t, rec.last_inside, rec.samples = s.t, inside, rec.samples + 1

  # ------------------------------------------------------------------------------------------------------- evidence

  def evaluate(self) -> dict[int, TrackEvidence]:
    """Each track's evidence from its matches so far (module docstring), and its validation record brought up to date."""
    out = {}
    for tid, h in self.hist.items():
      hit = self.cache.get(tid)
      if hit is not None and hit[0] == h[-1].t:
        if hit[1] is not None:
          out[tid] = hit[1]
        continue
      e = self._evidence(tid, h)
      self.cache[tid] = (h[-1].t, e)
      if e is None:
        continue
      out[tid] = e
      rec = self.records.get(tid)
      if rec is not None:
        if rec.approaching is None and e.v_trusted:
          rec.approaching = e.v_nom > APPROACH_V
        rec.stopped_at_line = rec.stopped_at_line or e.stopped_at_line
        rec.through_at_speed = rec.through_at_speed or e.through_at_speed
        rec.v_max = max(rec.v_max, e.v_nom if e.v_trusted else 0.)
    return out

  def _evidence(self, tid: int, h) -> TrackEvidence | None:
    last = h[-1]
    ln = self.assign(last)
    if ln is None:
      return None
    here = [s for s in h if abs(s.d_stop - last.d_stop) <= MOVE_TOL_M]
    s_min, s_max = ln.s_min + last.d_stop, ln.s_max + last.d_stop
    s_nom = (s_min + s_max) / 2.
    cross = is_cross(ln)
    st = self.states.setdefault(tid, _State())
    if cross:
      pix = max(self.pixel_rad.get(s.view, PIXEL_RAD_UNKNOWN) for s in here)
      fit = sr.fit_track([(s.t, abs(s.bearing), s.width or None) for s in here], s_min, s_max, self.bound, ln.conflict_m, pix)
      vel = [s for s in here if s.t >= last.t - sr.VEL_TRUST_S - 1e-9]
      ts = [s.t for s in vel]
      d_nom, slope = _linfit(ts, [s_nom * math.tan(math.radians(min(abs(s.bearing), 89.5))) - ln.conflict_m for s in vel])
      v_nom = -slope
      psi, dpsi = _rate_at_end(ts, [math.radians(abs(s.bearing)) for s in vel])
      tta_foot = math.sin(2. * psi) / (-2. * dpsi) if dpsi < -1e-6 else None
      ws = [s for s in vel if s.width > 0.]
      ttc = None
      if len(ws) >= 2:
        inv, inv_slope = _linfit([s.t for s in ws], [1. / s.width for s in ws])
        ttc = inv / -inv_slope if inv_slope < -1e-12 else None
    else:
      # a parallel lane (the opposite approach's): by ground range (module docstring)
      vel = [s for s in here if s.t >= last.t - sr.VEL_TRUST_S - 1e-9]
      d_nom, slope = _linfit([s.t for s in vel], [s.x - (s_nom + s.d_stop) for s in vel])
      v_nom = -slope
      trusted = last.t - here[0].t + 1e-9 >= sr.VEL_TRUST_S and len(vel) >= sr.VEL_TRUST_SAMPLES
      fit = sr.TrackFit(d_lo=max((1. - OPP_BAND) * last.x - s_min, 0.) - ln.conflict_m, v_hi=max((1. + OPP_BAND) * v_nom, 0.),
                        v_trusted=trusted)
      tta_foot, ttc = None, None
    line = self.lines.get(ln.role, DEFAULT_LINE_M)
    front = d_nom - CAR_HALF_LEN_M if cross else d_nom  # side on, its box's bottom centre is its middle; head on, its front
    front_from_line = line - front
    at_line = -sr.SHIELD_BEFORE_M <= front_from_line <= sr.SHIELD_PAST_M
    stopped = fit.v_trusted and fit.v_hi < sr.SHIELD_V
    if stopped:
      if st.stop_since is None:
        st.stop_since, st.stop_d = last.t, d_nom
    else:
      st.stop_since = None
    stopped_s = last.t - st.stop_since if st.stop_since is not None else 0.
    moved = abs(d_nom - st.stop_d) if st.stop_since is not None else 0.
    window = [s for s in here if s.t >= last.t - sr.SHIELD_STOP_S - 1e-9]
    gaps = [b.t - a.t for a, b in zip(window, window[1:], strict=False)]
    seen_every = (self.newest_t is None or self.newest_t - last.t <= CYCLE_GAP_S) and all(g <= CYCLE_GAP_S + 1e-9 for g in gaps)
    shield = ln.stopping and at_line and sr.shield_lane_ok(ln.approach) and \
      sr.shield_ok(last.cls, stopped_s, fit.v_hi, len([s for s in here if s.t >= last.t - sr.VEL_TRUST_S - 1e-9]), front_from_line,
                   seen_every, moved)
    through = self._through(st, ln, here, s_nom, line, last.cls, cross)
    return TrackEvidence(tid, last.cls, ln.role, ln.index, last.view, last.t, len(here), fit.d_lo, fit.v_hi, fit.v_trusted, fit.a_lo,
                         fit.a_trusted, fit.tau_dot, fit.expansion_agrees, d_nom, v_nom,
                         d_nom / v_nom if v_nom > 0.1 else float('inf'), fit.d_lo / fit.v_hi if fit.v_hi > 0.1 else float('inf'),
                         tta_foot, ttc, sr.u2_state(fit, at_line=at_line), front_from_line, at_line and stopped, stopped_s, shield,
                         through, ln)

  def _through(self, st: _State, ln: sr.Lane, here, s_nom: float, line: float, cls: str, cross: bool = True) -> bool:
    """Through its line at speed (module docstring), measured at the lane's nominal offset; latched."""
    if st.through or cls not in MOTOR:
      return st.through
    last = here[-1]
    span = [s for s in here if s.t >= last.t - SPEED_SPAN_S - 1e-9]
    if len(span) >= 3 and last.t - span[0].t >= SPEED_SPAN_S / 2.:
      if cross:
        ds = [s_nom * math.tan(math.radians(min(abs(s.bearing), 89.5))) - ln.conflict_m for s in span]
      else:
        ds = [s.x - (s_nom + s.d_stop) - ln.conflict_m for s in span]
      d, slope = _linfit([s.t for s in span], ds)
      st.speeds.append((d - CAR_HALF_LEN_M if cross else d, -slope))
    if not st.speeds or st.crossed:
      return st.through
    front, _ = st.speeds[-1]
    if front < line:
      st.crossed = True
      before = [v for f, v in st.speeds if line <= f <= line + DOWNGRADE_WINDOW_M]
      trusted = here[-1].t - here[0].t >= sr.VEL_TRUST_S - 1e-9 and len(here) >= sr.VEL_TRUST_SAMPLES
      st.through = trusted and bool(before) and min(before) >= DOWNGRADE_V and st.speeds[-1][1] >= DOWNGRADE_V
    return st.through
