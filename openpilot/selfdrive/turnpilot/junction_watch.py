"""The stop sweep at every stop and yield, and step S3's evidence (STOP_SIGNS.md 1.3, 1.4, 1.7, 4.3; SG3b). Nothing here acts.

turnpilotd asks modeld for the held cycle (turnpilotPlan.detectorView stopSweep: full wide, side left, road, side right) for
every stop or yield control map_controls has engaged (sweep_control), from STOP_SWEEP_M before it until the car is
SWEEP_PAST_M past it, whether openpilot is engaged or not: step S3's dwell is driven with it off and TurnPilot on, and
modeld reads the field through fresh_plan, so a shadow drive sweeps too. The unprotected left's own view (the road camera
from its go to its commit, CRITIQUE REQUIRED 3; roadAndWide held at its line) outranks it (detector_view): the sweep suspends
the far crop the left reads. SG4a's decider takes the field over with its own cycles.

While a sweep runs, JunctionWatch keeps the held control's live visibility (visibility.py) and track evidence
(stop_evidence.py), fed every run turnpilotd drains, and logs:

- turnpilot.stop_visibility every VIS_LOG_S: each conflicting lane's reach, R_seen, need, binding limit, blockers and shields,
  the views' freshness and the lane-edge estimate (read by nothing).
- turnpilot.junction_evidence once per control, when its sweep ends (passed, dropped, another control, TurnPilot off): the
  junction (OSM node, our arriving heading approachBearingDeg, the stop point, its conflicting lanes and their needs), the
  camera setback and whether step S1
  measured it, how long the car stood at the stop point, and per cross track its class, lane (role, index), view, first-track
  range (the pessimistic distance end), its lane's need, whether it was moving toward our path when its speed was first
  trusted, its longest gap between matches inside that need, whether it stopped at its line or went through at speed, how
  long our car had been standing at the stop point when it was first tracked (settledS, -1 if it wasn't), and the lighting
  row then. openpilot/tools/turnpilot/junction_validate.py reads these from the rlogs (section 4.3).

The lanes are every conflicting lane of the junction for any maneuver (the union: a stamp is per approach, not per
maneuver), seen from the control's stop point; the lighting row is the sun's at the junction from the wall clock, only while
the system time is valid ('unknown' otherwise, which validates nothing).
"""
from types import SimpleNamespace

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.maps import surveys as survey_file
from openpilot.selfdrive.turnpilot.routing.certify import cross_lanes, pick_stamp
from openpilot.selfdrive.turnpilot.routing.junctions import survey_approach
from openpilot.selfdrive.turnpilot.stop_evidence import DEFAULT_LINE_M, StopEvidence, junction_lanes
from openpilot.selfdrive.turnpilot.visibility import LiveVisibility

STOP_SWEEP_M = 30.      # m before a stop or yield the held cycle starts (section 9)
SWEEP_PAST_M = 2.       # m past it the sweep ends (the latch's LATCH_PAST_M)
SWEEP_KINDS = ('stop', 'yield')
STAND_V = 0.1           # m/s: standing
LINE_M = 3.0            # m from the stop point the car stands at it (section 2.4 item 1)
VIS_LOG_S = 1.0
DEFAULT_SETBACK_M = 1.9  # m, the design's camera setback until step S1 measures it (the event says which)
LEFT_VIEWS = ('road', 'roadAndWide')


def sweep_control(controls):
  """The stop or yield whose sweep runs: the nearest map_controls has engaged within STOP_SWEEP_M before it to SWEEP_PAST_M
  past it, else None."""
  live = [c for c in controls if c.kind in SWEEP_KINDS and c.engaged and -SWEEP_PAST_M <= c.d <= STOP_SWEEP_M]
  return min(live, key=lambda c: abs(c.d)) if live else None


def detector_view(left_view: str, sweep: bool) -> str:
  """turnpilotPlan.detectorView: the unprotected left's own view, else the sweep, else modeld's choice."""
  if left_view in LEFT_VIEWS:
    return left_view
  return 'stopSweep' if sweep else 'auto'


class _Attr:
  """A capnp to_dict() dict read by attribute, missing fields as 0, for stop_rules.junction_from_capnp."""
  def __init__(self, d: dict):
    self._d = d

  def __getattr__(self, k):
    v = self._d.get(k, 0)
    if isinstance(v, dict):
      return _Attr(v)
    if isinstance(v, list):
      return [_Attr(x) if isinstance(x, dict) else x for x in v]
    return v


def junction_from_info(kind: str, info: dict | None) -> sr.Junction:
  """A tracked control's junction_info (MapControl.junctionInfo.to_dict()) as stop_rules' Junction; none is failed."""
  if not info:
    return sr.Junction(kind=kind, status='failed')
  return sr.junction_from_capnp(SimpleNamespace(kind=kind, junctionInfo=_Attr(info)))


def watch_lanes(j: sr.Junction, setback: float, lane_edge_m: float | None = None) -> list:
  """Every conflicting lane of the junction for any maneuver, from its stop point."""
  return junction_lanes(j, 'unknown', setback, lane_edge_m)


def lighting_at(j: sr.Junction, wall_t: float | None, veil: str | None) -> tuple[str, tuple]:
  """(row, low-sun sides) at the junction now: night, dusk, sunAhead (the sun within 90 deg of our heading), day, or
  unknown without a valid clock; and the sides whose approach the low sun lies along (section 1.6)."""
  if wall_t is None:
    return 'unknown', ()
  el, az = sr.sun_position(j.lat, j.lon, wall_t)
  if el < sr.NIGHT_SUN_EL:
    return 'night', ()
  if el < 0.:
    return 'dusk', ()
  sides = tuple(sorted({'left' if a.role == 'left' or (a.role == 'exit' and a.rel_bearing < 0.) else 'right'
                        for a in j.approaches if sr.effective_role(a) in ('left', 'right') and el < sr.SUN_EL_MAX and
                        sr.angle_diff(az, a.bearing) <= sr.SUN_AZ_TOL}))
  row = 'sunAhead' if sr.angle_diff(az, j.heading_in) < sr.SUN_AHEAD_DEG else 'day'
  return row, sides


class JunctionWatch:
  def __init__(self):
    self.live = LiveVisibility()
    self.evidence = StopEvidence()
    self.key = None
    self._reset(None)

  def _reset(self, control) -> None:
    self.key = control.key if control is not None else None
    self.control = control
    self.junction = junction_from_info(control.kind, control.junction_info) if control is not None else None
    self.lanes, self.entry, self.edges = [], None, None
    self.evidence.reset()
    self.notes: dict[int, dict] = {}       # trackId: settledS and lighting at its first track
    self.stand_t: float | None = None      # camera time our car came to stand at the stop point
    self.stand_s = 0.
    self.t_start: float | None = None
    self.vis_t: float | None = None
    self.row, self.low_sides = 'unknown', ()
    self.setback, self.setback_measured = DEFAULT_SETBACK_M, False

  # ------------------------------------------------------------------------------------------------------------ input

  def observe_runs(self, runs, available: bool) -> None:
    """runs: the tracker's new_runs this tick, (view key, camera time, RawFrame, sectors); available: the scene's."""
    for key, t, raw, sectors in runs:
      self.live.observe_run(key, t, raw=raw, sectors=sectors)
    self.live.set_available(available)

  def update(self, control, v_ego: float, tracks, vis, surveys, wall_t: float | None, dt: float = 0.05) -> None:
    """control: sweep_control()'s, or None; tracks: the tracker's; vis: visibility_table's Visibility (or None); surveys:
    the survey file (maps.surveys); wall_t: the wall clock while valid, else None."""
    if control is None or control.key != self.key:
      self.flush()
      self._reset(control)
      if control is None:
        return
      self._start(vis, surveys)
    self.control = control
    t_cam = self.live.newest_t
    if t_cam is not None and self.t_start is None:
      self.t_start = t_cam
    standing = v_ego < STAND_V and abs(control.d) <= LINE_M
    if standing:
      self.stand_s += dt
      if self.stand_t is None and t_cam is not None:
        self.stand_t = t_cam
    else:
      self.stand_t = None
    if wall_t is not None or self.row == 'unknown':
      self.row, self.low_sides = lighting_at(self.junction, wall_t, vis.veil if vis is not None else None)
    known = set(self.evidence.records)
    self.evidence.observe(tracks, d_stop=control.d)
    for tid in set(self.evidence.records) - known:
      first = self.evidence.records[tid].first
      settled = first.t - self.stand_t if self.stand_t is not None and first.t >= self.stand_t else -1.
      self.notes[tid] = {'settledS': settled, 'lighting': self.row}
    evidence = self.evidence.evaluate()
    if t_cam is not None and (self.vis_t is None or t_cam - self.vis_t >= VIS_LOG_S - 1e-6):
      self.vis_t = t_cam
      self._log_visibility(vis, tracks, evidence)

  def _start(self, vis, surveys) -> None:
    j = self.junction
    if vis is not None and vis.setback is not None:
      self.setback, self.setback_measured = float(vis.setback), True
    if j.status != 'ok':
      return
    entries = getattr(surveys, 'entries', surveys) or []
    self.entry = survey_file.match(list(entries), j.osm_node_id, (j.lat, j.lon)) if entries else None
    ours = survey_approach(self.entry, j.our_way, j.heading_in) if self.entry is not None else None
    lane_edge = (ours or {}).get('laneEdgeM')
    self.edges = sr.edge_band(j.stop_point_distance, cross_lanes(j), lane_edge)
    self.lanes = watch_lanes(j, self.setback, lane_edge)
    lines = {}
    for ln in self.lanes:
      a = ln.approach
      sv = survey_approach(self.entry, a.way_id, (a.bearing + 180.) % 360.) if self.entry is not None and a is not None else None
      line = (sv or {}).get('stopPointM') or (sv or {}).get('lineM')
      lines[ln.role] = float(line) if line else DEFAULT_LINE_M
    self.evidence.set_junction(self.lanes, sr.angle_bound(self.edges.nominal is not None), lines)

  # ------------------------------------------------------------------------------------------------------------- logs

  def _log_visibility(self, vis, tracks, evidence) -> None:
    if vis is None or not self.lanes:
      return
    j = self.junction
    stamp = pick_stamp(self.entry, self.row, j.heading_in) if self.entry is not None else None
    rep = self.live.lanes(self.lanes, self.control.d, vis, rot_deg=sr.angle_bound(self.edges.nominal is not None),
                          lighting=self.row, low_sun_sides=self.low_sides, entry=self.entry, stamp=stamp, tracks=tracks,
                          evidence=evidence, edges=self.edges)
    cloudlog.event("turnpilot.stop_visibility", osmNodeId=j.osm_node_id, controlKey=list(self.key), d=round(self.control.d, 2),
                   lighting=self.row, lowSun=list(self.low_sides), fresh=rep.fresh, occluders=rep.occluders,
                   edgeEstimate=rep.edge_estimate,
                   lanes=[{'role': x.role, 'index': x.index, 'stopping': x.stopping, 'reachM': round(x.reach_m, 2),
                           'rSeenM': round(x.r_seen_m, 2), 'needM': round(x.need_m, 2), 'why': x.why, 'blockers': x.blockers,
                           'candidates': x.candidates, 'shields': x.shields} for x in rep.lanes])

  def flush(self) -> None:
    """The held control's turnpilot.junction_evidence, once (module docstring)."""
    if self.key is None or self.control is None:
      return
    j = self.junction
    cars = []
    for tid, rec in sorted(self.evidence.records.items()):
      f = rec.first
      note = self.notes.get(tid, {})
      cars.append({'trackId': tid, 'cls': rec.cls, 'role': f.role, 'index': f.index, 'view': f.view, 'firstT': f.t,
                   'firstRangeM': f.range_m, 'firstDStopM': f.d_stop, 'needM': rec.need_m, 'approaching': rec.approaching,
                   'maxGapInsideS': rec.max_gap_inside_s, 'stoppedAtLine': rec.stopped_at_line, 'throughAtSpeed': rec.through_at_speed,
                   'vMax': rec.v_max, 'samples': rec.samples, 'lastT': rec.last_t, 'settledS': note.get('settledS', -1.),
                   'lighting': note.get('lighting', 'unknown')})
    cloudlog.event("turnpilot.junction_evidence", osmNodeId=j.osm_node_id, controlKey=list(self.key), kind=self.control.kind,
                   junctionStatus=j.status, lat=j.lat, lon=j.lon, approachBearingDeg=j.heading_in, ourWay=j.our_way,
                   stopPointDistance=j.stop_point_distance, surveyed=j.surveyed, setbackM=self.setback,
                   setbackMeasured=self.setback_measured, standS=round(self.stand_s, 2), lighting=self.row, tStart=self.t_start,
                   tEnd=self.live.newest_t, angleBoundDeg=self.evidence.bound, cars=cars,
                   lanes=[{'role': ln.role, 'index': ln.index, 'stopping': ln.stopping, 'needM': self.evidence.need(ln)} for ln in self.lanes])
    self.key = None
