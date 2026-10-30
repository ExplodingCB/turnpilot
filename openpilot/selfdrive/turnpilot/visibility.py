"""The live reach of STOP_SIGNS.md 1.4 (SG3b): per conflicting lane of the held control's junction, how far down it the camera
sees now, through stop_rules' walk with the live inputs. Nothing reads it to act yet (SG4a's decider will); turnpilotd logs it.

LiveVisibility is fed every detector run turnpilotd drains (observe_run: its view, camera time, raw boxes and, on a side
view's run, its sectors' quality) and, each frame, whether the scene is available (set_available: the tracker's staleness on
turnpilotd's clock, so a detector that stops leaves nothing fresh). lanes() then walks each lane with:

- Freshness on camera time: a view counts while its newest run is within DET_FRESH_S of the newest run of any view (the
  side view on the lane's side for 'side'), and only while the scene is available.
- Sector quality over the whole CLEAR_S window, not only the latest frame: a side-view bearing counts only if every run of
  that side's view in the window listed an ok sector covering it, at the point's bearing and at it unrotated (a run without a
  reading counts as not ok). The inner band (40 deg to the side views' inner edge) also needs the innermost sector ok.
- The sun: with the sun ahead and the veil present or unmeasured, 67 deg to the edge on both sides (sunAhead); low sun along
  an approach, that side's side-view sectors (lowSun); night and dusk have no table, so nothing.
- The footprint mask from visibility.json (stop_rules.footprint_ok, a car-sized footprint, not a ground point); a file
  without a mask (before step S1) sees nothing (noMask): a missing mask never leaves the pillar and housing at full reach.
- Occluders from every track, moving or not, and from every raw box held steady (STEADY_S with STEADY_SAMPLES runs of its
  view, bearing within STEADY_BEARING_DEG, width within STEADY_WIDTH): its bearing span caps each lane where it meets the
  lane at sLaneMin (a stopped car caps its own lane at itself and every lane behind it). A track's or box's ground range is
  trusted only under OCCLUDER_RANGE_TRUST_DEG, and then at its near end (OCCLUDER_NEAR_FRAC); past it, or unranged, it is
  bearing-only (1.2: an occluder whose depth is unknown).
- At a surveyed junction, the survey's sight distance and the parking on the lane's binding curb (routing.certify.lane_cap),
  and a passing validation stamp's minFirstTrackM in place of the tables' range bound (never in place of the coverage).
- A lane whose approach doesn't stop (section 4.1) has reach 0 (notSurveyed) unless the survey has that approach's sight
  distance (1.4 rule 5, row G2): YOLO reports cars, not hedges, fences, walls or crests, so with no survey nothing says the
  lane isn't hidden, whatever the tables' walk would reach (an unsurveyed junction's opposite lane read 127.5 m).

Per lane: reach, R_seen = reach - v_c T_LAT, the binding limit (what ended the walk), its need (all-way or two-way), the raw
boxes that block it (confidence RAW_BLOCK_CONF or more, not steady, whose bearing ray meets it within its need at either end
of the s band, or any within RAW_HORIZON_DEG of the horizon on its side: blockers()), and its shield candidates from stop_evidence (a
vehicle stopped at its line) and qualified shields. Besides, the near edge estimated from cross cars' forward distance at
bearings under EDGE_EST_MAX_DEG, which is logged and read by nothing (ground range there is unvalidated until step S4).
"""
import math
from collections import deque
from dataclasses import dataclass, field, replace

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.routing.certify import _per_lane, lane_cap, lane_need

SIDE_VIEW = {'left': 'sideLeft', 'right': 'sideRight'}
VIEW_KEY = {'road': 'road', 'full': 'full'}
OCCLUDER_RANGE_TRUST_DEG = 40.  # deg: a ground range is trusted under this bearing only (the full pass's measured domain)
OCCLUDER_NEAR_FRAC = 0.7        # and then at its near end
STEADY_S, STEADY_SAMPLES = 1.0, 5
STEADY_BEARING_DEG, STEADY_WIDTH = 0.3, 0.05
STEADY_PERIOD_S = 0.25          # s: the steady history must reach back STEADY_S less about one 5 Hz period
RAW_BLOCK_CONF = 0.35
RAW_HORIZON_DEG = 2.
RAW_LONG_DEG = 50.              # deg: past this a box's ground range reads up to RAW_LONG times long (verify.md finding 3)
RAW_LONG = 2.5
RAW_NEAR_FRAC = 0.7             # under it, its range at least this of what it reads
LANE_HALF_M = sr.LANE_W_MAX / 2.
EDGE_EST_MAX_DEG = 60.
EDGE_EST_V = 2.                 # m/s across our path a car must make to count for the edge estimate
MOTOR = ('car', 'truck', 'bus', 'motorcycle')
CLASS_NAMES = {0: 'pedestrian', 1: 'cyclist', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}


@dataclass
class LaneSeen:
  role: str
  index: int
  movement: str
  stopping: bool
  limit: float               # m/s, its approach's assumed limit
  need_m: float              # m of seen lane it needs (all-way or two-way, latency included)
  reach_m: float             # m seen from its conflict point
  r_seen_m: float            # reach less v_c T_LAT
  why: str                   # the binding limit: what ended the walk
  blockers: list = field(default_factory=list)  # raw boxes blocking it (descriptions)
  candidates: list = field(default_factory=list)  # tracks stopped at its line (trackIds)
  shields: list = field(default_factory=list)     # ...that qualify as its shield


@dataclass
class Report:
  lanes: list
  edge_estimate: dict
  occluders: int
  fresh: dict                # view key: fresh now


def ray_lane_d(lane: sr.Lane, psi_deg: float, which: str) -> float | None:
  """m along lane (from its origin) where a ray at psi_deg off the camera axis, on the lane's side, meets the lane line at
  its small ('min') or large ('max') offset; None if it never does (parallel, or behind)."""
  sx, y0 = lane.point(0., which)
  sigma = -1. if lane.point(10., which)[1] < 0. or (lane.rel_bearing == 0. and y0 < 0.) else 1.
  psi, r = math.radians(psi_deg), math.radians(lane.rel_bearing)
  den = math.cos(psi) * math.sin(r) - sigma * math.sin(psi) * math.cos(r)
  if abs(den) < 1e-9:
    return None
  d = (sigma * math.sin(psi) * sx - math.cos(psi) * y0) / den
  return d if d >= 0. else None


def class_name(cls) -> str:
  """A class as its name: a name, the tracker's class index (CLASS_NAMES), or a TurnpilotScene.ObjectClass enum as turnpilotd
  hands the scene's objects over (capnp readers), which reads as its own name (its ordinals aren't the tracker's indexes)."""
  if isinstance(cls, str):
    return cls
  try:
    return CLASS_NAMES.get(int(cls), 'other')
  except TypeError:
    return str(cls)


class LiveVisibility:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.view_t: dict[str, float] = {}
    self.raw: dict = {}                                  # view key: its newest RawFrame
    self.sectors = {'left': deque(), 'right': deque()}   # (camera time, [(lo, hi, quality)] in unsigned deg) per side-view run
    self.history: dict[str, deque] = {}                  # view key: (camera time, boxes) over STEADY_S
    self.newest_t: float | None = None
    self.available = True

  # ------------------------------------------------------------------------------------------------------------ input

  def observe_run(self, key: str, t: float, raw=None, sectors=None) -> None:
    """A detector run: its view key (scene.tracker.view_key), camera time, RawFrame, and a side view's sectorQuality as
    (inner, outer, quality) in deg left of the axis (or the capnp list)."""
    self.view_t[key] = max(t, self.view_t.get(key, -math.inf))
    self.newest_t = t if self.newest_t is None else max(self.newest_t, t)
    if raw is not None:
      self.raw[key] = raw
      h = self.history.setdefault(key, deque())
      h.append((t, list(raw.boxes)))
      while h and h[0][0] < t - STEADY_S - STEADY_PERIOD_S:
        h.popleft()
    side = {'sideLeft': 'left', 'sideRight': 'right'}.get(key)
    if side is not None:
      rows = []
      for sec in sectors or ():
        inner, outer, q = (sec.inner, sec.outer, str(sec.quality)) if hasattr(sec, 'inner') else sec
        rows.append((min(abs(inner), abs(outer)), max(abs(inner), abs(outer)), str(q)))
      w = self.sectors[side]
      w.append((t, rows))
      while w and w[0][0] < t - sr.CLEAR_S - 1e-9:
        w.popleft()

  def set_available(self, available: bool) -> None:
    self.available = bool(available)

  # -------------------------------------------------------------------------------------------------------- the inputs

  def fresh_key(self, key: str) -> bool:
    t = self.view_t.get(key)
    return self.available and t is not None and self.newest_t is not None and self.newest_t - t <= sr.DET_FRESH_S + 1e-9

  def fresh(self, view: str, side: str) -> bool:
    return self.fresh_key(SIDE_VIEW[side] if view == 'side' else VIEW_KEY.get(view, view))

  def sector_ok(self, side: str, bearing: float) -> bool:
    """Every run of side's view in the CLEAR_S window listed an ok sector covering bearing (deg, unsigned), and one did."""
    if self.newest_t is None:
      return False
    frames = [rows for t, rows in self.sectors[side] if t >= self.newest_t - sr.CLEAR_S - 1e-9]
    if not frames:
      return False
    for rows in frames:
      cover = [q for lo, hi, q in rows if lo - 1e-9 <= bearing <= hi + 1e-9]
      if not cover or any(q != 'ok' for q in cover):
        return False
    return True

  def steady(self, key: str, box) -> bool:
    """box held still in its view's runs for STEADY_S (module docstring)."""
    h = self.history.get(key)
    if not h:
      return False
    t_now = h[-1][0]
    width = abs(box.span_deg[1] - box.span_deg[0])
    hits = []
    for t, boxes in h:
      if t < t_now - STEADY_S - 1e-9:
        continue
      if not any(abs(b.bearing_deg - box.bearing_deg) <= STEADY_BEARING_DEG and
                 abs(abs(b.span_deg[1] - b.span_deg[0]) - width) <= STEADY_WIDTH * max(width, 1e-6) for b in boxes):
        return False
      hits.append(t)
    return len(hits) >= STEADY_SAMPLES and t_now - hits[0] >= STEADY_S - STEADY_PERIOD_S

  # ----------------------------------------------------------------------------------------------------------- lanes

  def lanes(self, lanes_at_stop: list, d_stop: float, vis, *, rot_deg: float, edges: sr.Edges, lighting: str = 'day',
            low_sun_sides=(), entry: dict | None = None, stamp: dict | None = None, tracks=(), evidence: dict | None = None,
            cam_height: float | None = None, a_launch: float = sr.A_LAUNCH_PLAN, ahead_m: float = 0.) -> Report:
    """Each lane's reach now (module docstring). lanes_at_stop: the held control's conflicting lanes seen from its stop
    point (stop_evidence.junction_lanes); d_stop: m our car stands short of the stop point; vis: visibility_table's
    Visibility; lighting: day, sunAhead, dusk or night at the line; low_sun_sides: the sides with low sun along an approach;
    entry: the matched survey; stamp: its passing validation stamp for our approach and this lighting (certify.pick_stamp);
    tracks: the tracker's tracks; evidence: stop_evidence's; edges: the edge band from the stop point (stop_rules.edge_band;
    a non-stopping lane's two-way need reads its far end). ahead_m: predict the walk from that much further on (the
    stop-sign decider's creep for a view, STOP_SIGNS.md 2.5): d_stop already the nearer point's, and every occluder with a
    ground position re-seen from there (occluders()); the views, sectors and sun as they are now."""
    height = cam_height if cam_height is not None else (vis.height if vis.height is not None else 1.3)
    lns = [replace(ln, s_min=ln.s_min + d_stop, s_max=ln.s_max + d_stop) for ln in lanes_at_stop]
    edges_now = sr.Edges(edges.d_min + d_stop, edges.d_max + d_stop, None if edges.nominal is None else edges.nominal + d_stop)
    occ = self.occluders(tracks, ahead_m)
    fresh = {k: self.fresh_key(k) for k in ('road', 'full', 'sideLeft', 'sideRight')}
    veil_ok = vis.veil in sr.VEIL_OK
    rows = vis.views(lighting, use='allWay') if lighting in ('day', 'sunAhead') else {}

    def zero(side: str, bearing: float):
      if lighting == 'sunAhead' and not veil_ok and sr.sun_ahead_zero(side, bearing):
        return 'sunAhead'
      if side in low_sun_sides and bearing >= sr.INNER_BAND_HI_DEG:
        return 'lowSun'
      if bearing >= sr.INNER_BAND_LO_DEG and self.fresh('side', side):  # a stale side view covers nothing anyway (noView)
        for b in (bearing, bearing - rot_deg):
          if not self.sector_ok(side, max(b, sr.INNER_BAND_HI_DEG + 0.1)):
            return 'sector'
      return False

    sight = vis.sight(rows, zero=zero, fresh=self.fresh, occluders=occ)
    out = []
    for ln in lns:
      need = lane_need(ln, edges_now, a_launch)
      if not vis.mask or vis.height is None:
        reach, why = 0., 'noMask'
      elif not self.available:
        reach, why = 0., 'noView'
      else:
        cap, cap_why = lane_cap(ln, entry) if entry is not None else (None, 'not surveyed')
        if not ln.stopping and cap_why == 'not surveyed':  # rule 5: nothing but a survey says a hedge or crest isn't there
          reach, why = 0., 'notSurveyed'
        else:
          validated = _per_lane(stamp.get('minFirstTrackM'), ln.role, ln.index) if stamp else None
          reach, why = sr.reach_walk_why(ln, sight, rot_deg, validated_m=validated, cap_m=cap)
      v_c = sr.V_CROSS_FACTOR * ln.limit
      seen = LaneSeen(ln.role, ln.index, ln.movement, ln.stopping, ln.limit, need, reach, max(reach - v_c * sr.T_LAT, 0.), why)
      seen.blockers = self.blockers(ln, need, rot_deg, height)
      for tid, e in (evidence or {}).items():
        if (e.role, e.index) == (ln.role, ln.index) and e.stopped_at_line:
          seen.candidates.append(tid)
          if e.shield:
            seen.shields.append(tid)
      out.append(seen)
    return Report(out, self.edge_estimate(tracks, vis.setback if vis.setback is not None else 0., d_stop), len(occ), fresh)

  def occluders(self, tracks, ahead_m: float = 0.) -> list:
    """Every track's box, and every steady raw box of a fresh view, as stop_rules Occluders (module docstring). ahead_m: each
    one with a ground position re-seen from that much further on (its span's edges taken at its own range), the trust rule
    applied to its bearing from there; one with none (unranged) keeps its bearings, the occluder at infinity, which a creep
    never helps (the prediction's safe side)."""
    out = []

    def shifted(span, bearing, rng, ground):
      if not ahead_m or ground is None:
        return span, bearing, rng

      def see(b):
        x, y = ground * math.cos(math.radians(b)) - ahead_m, ground * math.sin(math.radians(b))
        return math.degrees(math.atan2(y, x)), math.hypot(x, y)
      (b0, _), (b1, _), (bc, r) = see(span[0]), see(span[1]), see(bearing)
      return (b0, b1), bc, None if rng is None else max(rng + r - ground, 0.1)

    def add(span, bearing, rng, ground=None):
      span, bearing, rng = shifted(span, bearing, rng, ground)
      lo, hi = sorted(span)
      trusted = rng is not None and abs(bearing) < OCCLUDER_RANGE_TRUST_DEG
      r = rng * OCCLUDER_NEAR_FRAC if trusted else None
      if hi > 0.:
        out.append(sr.Occluder('left', max(lo, 0.), hi, r))
      if lo < 0.:
        out.append(sr.Occluder('right', max(-hi, 0.), -lo, r))
    for trk in tracks:
      span = getattr(trk, 'span_seen', None)
      if span is None or trk.x is None:
        continue
      rng = math.hypot(float(trk.x[0]), float(trk.x[1]))
      add(span, trk.bearing_seen, rng, rng)
    for key, frame in self.raw.items():
      if key == 'far' or not self.fresh_key(key):
        continue
      for b in frame.boxes:
        if self.steady(key, b):
          add(b.span_deg, b.bearing_deg, b.near_m if b.xy is not None else None, math.hypot(*b.xy) if b.xy is not None else None)
    return out

  def blockers(self, ln: sr.Lane, need: float, rot_deg: float, height: float) -> list:
    """The raw boxes of the fresh views' newest runs that block ln (STOP_SIGNS.md 1.7): a road user at RAW_BLOCK_CONF or
    more, not steady, whose plausible ground position (its range anywhere from RAW_NEAR_FRAC, or past RAW_LONG_DEG 1 /
    RAW_LONG, of what it reads, up to that) overlaps the lane's band at either end of the s band, half a lane either side,
    within its need along it; or any such box within RAW_HORIZON_DEG of the horizon (or above it: unranged) on the lane's
    side from where the lane starts, whatever band its nominal position falls in."""
    out = []
    side = 'left' if ln.point(10., 'min')[1] < 0. else 'right'
    p0 = ln.point(sr.WALK_STEP_M, 'min')
    origin = math.degrees(math.atan2(abs(p0[1]), p0[0]))
    cross = abs(abs(ln.rel_bearing) - 90.) <= 45.
    for key, frame in self.raw.items():
      if key == 'far' or not self.fresh_key(key):
        continue
      for b in frame.boxes:
        if b.conf < RAW_BLOCK_CONF or b.cls not in MOTOR + ('cyclist', 'pedestrian') or self.steady(key, b):
          continue
        if (b.bearing_deg >= 0.) != (side == 'left'):
          continue
        if b.xy is not None:
          x, y = float(b.xy[0]), abs(float(b.xy[1]))
          lo = 1. / RAW_LONG if abs(b.bearing_deg) >= RAW_LONG_DEG else RAW_NEAR_FRAC
          near_axis = max(min(abs(e) for e in b.span_deg) - rot_deg, 0.)
          d = ray_lane_d(ln, near_axis, 'min')
          along = d is not None and d - ln.conflict_m <= need
          if cross:
            in_band = x * lo <= ln.s_max + LANE_HALF_M and x >= ln.s_min - LANE_HALF_M
          else:
            in_band = y * lo <= abs(ln.y) + LANE_HALF_M and y >= abs(ln.y) - LANE_HALF_M and x >= ln.s_min - LANE_HALF_M
          if in_band and along:
            out.append(f"{key} {b.cls} {b.conf:.2f} at {b.bearing_deg:.1f} deg")
            continue
        elev = math.degrees(math.atan2(height, math.hypot(*b.xy))) if b.xy is not None else -1.
        if elev <= RAW_HORIZON_DEG and abs(b.bearing_deg) + 1e-9 >= origin:
          out.append(f"{key} {b.cls} {b.conf:.2f} at {b.bearing_deg:.1f} deg near the horizon")
    return out

  @staticmethod
  def edge_estimate(tracks, setback: float, d_stop: float) -> dict:
    """The near edge of the cross road from cross cars' forward distance (module docstring): per side, the median of the
    crossing motor vehicles' x at bearings under EDGE_EST_MAX_DEG, less the setback and half a lane: m from the stop point."""
    out = {}
    for side in ('left', 'right'):
      xs = sorted(float(t.x[0]) for t in tracks
                  if getattr(t, 'bearing_seen', None) is not None and t.x is not None and class_name(t.cls) in MOTOR and
                  (t.bearing_seen >= 0.) == (side == 'left') and abs(t.bearing_seen) < EDGE_EST_MAX_DEG and
                  abs(float(t.x[3])) >= EDGE_EST_V and abs(float(t.x[3])) > 2. * abs(float(t.x[2])))
      if xs:
        n = len(xs)
        med = xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.
        half = (sr.NEAR_LANE_MIN_M + sr.NEAR_LANE_MAX_M) / 2.
        out[side] = {'n': n, 'xMedM': med, 'edgeFromStopM': med - setback - half - d_stop}
    return out
