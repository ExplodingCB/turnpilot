"""visibility.py, the live reach of STOP_SIGNS.md 1.4 (SG3b): per conflicting lane of the held control's junction, stop_rules'
walk with the live inputs (per-view freshness on camera time, sector quality over the whole CLEAR_S window, the footprint
mask, occluders from every track and from steady raw boxes, the survey's caps, the sun), R_seen after T_LAT, the binding
limit, raw-box blockers and the horizon rule, and the logged lane-edge estimate. Rows G1, G2, G3, G5, G8, G10, E1, E4, E11,
E12 and the inner band. Expected values come from the geometry and section 2.8 (repo tests can't read the artifacts)."""
import copy
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot import visibility_table as vt
from openpilot.selfdrive.turnpilot.scene.tracker import RawBox, RawFrame
from openpilot.selfdrive.turnpilot.stop_evidence import junction_lanes
from openpilot.selfdrive.turnpilot.tests.test_stop_evidence import junction
from openpilot.selfdrive.turnpilot.visibility import LiveVisibility, ray_lane_d

SETBACK, HEIGHT = 1.9, 1.3
ROT = sr.angle_bound(False)  # 3.5 deg: section 2.8's unsurveyed geometry
T = 10.0                     # camera time now


def open_mask():
  return {side: [[b, -40., 40.] for b in range(86)] for side in ('left', 'right')}


def vis_file(veil='unmeasured', mask=None):
  raw = json.loads(vt.SHIPPED.read_text())
  raw.update(cameraSetbackM=SETBACK, lateralOffsetM=0., heightM=HEIGHT, darkThreshold=45., veil=veil,
             lens={'model': 'equidistant', 'f': 452., 'k': 0., 'edgeDeg': {'left': 85.2, 'right': 85.2}},
             mask=open_mask() if mask is None else mask)
  return vt.Visibility(raw, tables=vt.validate(raw))


def sectors(view, bad=None, quality='degraded'):
  """A side view's sectorQuality: 5 deg sectors from its inner edge to past the frame edge, ok but the one containing bad."""
  sign = 1. if view == 'sideLeft' else -1.
  out = []
  for lo in np.arange(sr.INNER_BAND_HI_DEG, 90., 5.):
    q = quality if bad is not None and lo <= bad < lo + 5. else 'ok'
    out.append((sign * lo, sign * (lo + 5.), q))
  return out


def live(views=('road', 'full', 'sideLeft', 'sideRight'), until=T, bad=None, bad_at=None, quality='degraded', raw=None):
  """2 s of a held sweep's runs, each view at 5 Hz, ending at camera time until; the side views' sectors ok but the one
  containing bad (deg) on the run at camera time bad_at (every run when bad_at is None)."""
  lv = LiveVisibility()
  for k in range(11):
    for i, v in enumerate(('full', 'sideLeft', 'road', 'sideRight')):
      t = until - 2.0 + 0.2 * k - 0.05 * (3 - i)
      if v not in views:
        continue
      side = v.startswith('side')
      worse = bad is not None and (bad_at is None or abs(t - bad_at) < 1e-6)
      frame = (raw or {}).get(v, RawFrame([], 'road' if v == 'road' else 'wide', t, 'full' if not side else v))
      frame = RawFrame(frame.boxes, frame.camera, t, frame.view)
      lv.observe_run(v, t, raw=frame, sectors=sectors(v, bad if worse else None, quality) if side else None)
  return lv


def step(lv, t, boxes=None):
  """One more cycle of every view's runs at camera time t, view key: boxes in its run."""
  for v in ('full', 'sideLeft', 'road', 'sideRight'):
    side = v.startswith('side')
    lv.observe_run(v, t, raw=RawFrame((boxes or {}).get(v, []), 'road' if v == 'road' else 'wide', t, v if side else 'full'),
                   sectors=sectors(v) if side else None)


def lanes(surveyed=False):
  return junction_lanes(junction(25.), 'straight', SETBACK)


EDGES = sr.edge_band(13., 2)  # the junction's 2-lane cross road from its default stop point 13 m before the node


def report(lv, vis=None, **kw):
  kw.setdefault('rot_deg', ROT)
  return lv.lanes(lanes(), 0., vis or vis_file(), edges=EDGES, **kw)


def lane_of(rep, role, index=1):
  return next(x for x in rep.lanes if (x.role, x.index) == (role, index))


def walk(role, vis=None, rot=ROT, **kw):
  """The route-time walk on the same lane: every view fresh, nothing live."""
  vis = vis or vis_file()
  ln = next(x for x in lanes() if x.role == role)
  return sr.reach_walk(ln, vis.sight(vis.views('day')), rot, **kw)


def trk(tid, d, ln, view='sideLeft', cls=2, vx=0., vy=0.):
  """A tracker Track for a car standing d m down lane ln (bottom centre, 1.8 m wide)."""
  s = (ln.s_min + ln.s_max) / 2.
  left = ln.point(10., 'min')[1] < 0.
  x, y = (s, d) if ln.role != 'opposite' else (s + d, abs(ln.y))
  psi = math.degrees(math.atan2(y, x))
  half = math.degrees(math.atan2(0.9, math.hypot(x, y)))
  sign = 1. if left else -1.
  return SimpleNamespace(track_id=tid, t_seen=T, bearing_seen=sign * psi, width_seen=100., view=view, cls=cls, hits=5,
                         span_seen=tuple(sorted((sign * (psi - half), sign * (psi + half)))),
                         x=np.array([x, sign * y, vx, vy]))


def raw_box(bearing, conf=0.4, xy=None, width_deg=2.0, camera='wide', view='sideLeft'):
  return RawBox(camera, 'car', conf, bearing, (bearing - width_deg / 2., bearing + width_deg / 2.), xy,
                float(np.hypot(*xy)) * 0.8 if xy is not None else None, (0., 0., 0.1, 0.1), view)


# ---------------------------------------------------------------------------------------------------------- the open corner

def test_G1_open_corner_live_reach_is_the_tables_walk_and_r_seen_takes_t_lat():
  rep = report(live())
  for role in ('left', 'right'):
    x = lane_of(rep, role)
    assert x.reach_m == pytest.approx(walk(role), abs=1e-6) and x.why == 'range'
    v_c = sr.V_CROSS_FACTOR * 25. * sr.MPH
    assert x.r_seen_m == pytest.approx(max(x.reach_m - v_c * sr.T_LAT, 0.), abs=1e-6)
    assert x.need_m == pytest.approx(sr.reach_needed_allway(25. * sr.MPH), abs=1e-6)
  # section 2.8, unsurveyed at node - 13 m, PV1 conservative: 27 m near, 25 m far
  assert lane_of(rep, 'left').reach_m == pytest.approx(27., abs=1.5) and lane_of(rep, 'right').reach_m == pytest.approx(25., abs=1.5)


def test_freshness_is_on_camera_time_and_needs_the_scene():
  # the right side view's last run 0.6 s of camera time behind the newest: past DET_FRESH_S, so the right lane ends where
  # the side view's coverage begins (the inner band needs it too: 40 deg)
  lv = live()
  lv.observe_run('road', T + 0.6, raw=RawFrame([], 'road', T + 0.6))
  lv.observe_run('full', T + 0.6, raw=RawFrame([], 'wide', T + 0.6))
  lv.observe_run('sideLeft', T + 0.6, raw=RawFrame([], 'wide', T + 0.6, 'sideLeft'), sectors=sectors('sideLeft'))
  rep = report(lv)
  right = lane_of(rep, 'right')
  ln = next(x for x in lanes() if x.role == 'right')
  assert right.why == 'noView' and right.reach_m <= ln.s_min * math.tan(math.radians(sr.INNER_BAND_LO_DEG - ROT)) + sr.WALK_STEP_M
  assert lane_of(rep, 'left').why == 'range'
  # the scene unavailable (no runs for STALE_S on turnpilotd's clock): nothing fresh, nothing seen
  lv.set_available(False)
  assert all(x.reach_m == 0. for x in report(lv).lanes)


def test_inner_band_counts_only_with_the_full_pass_and_that_sides_view_fresh_and_ok():
  ln = next(x for x in lanes() if x.role == 'left')
  d45 = ln.s_min * math.tan(math.radians(45. - ROT))  # where the walk's bearing reaches 45 deg
  rep = report(live())
  assert lane_of(rep, 'left').reach_m > d45
  # the left side view stale: the lane ends at 40 deg
  rep = report(live(views=('road', 'full', 'sideRight')))
  d40 = ln.s_min * math.tan(math.radians(sr.INNER_BAND_LO_DEG - ROT))
  assert lane_of(rep, 'left').reach_m == pytest.approx(d40, abs=sr.WALK_STEP_M) and lane_of(rep, 'left').why == 'noView'
  # the full pass stale: the road camera's 20 deg ends it, short of the band
  rep = report(live(views=('road', 'sideLeft', 'sideRight')))
  d20 = ln.s_min * math.tan(math.radians(20. - ROT))
  assert lane_of(rep, 'left').reach_m == pytest.approx(d20, abs=sr.WALK_STEP_M) and lane_of(rep, 'left').why == 'noView'
  # the side view's innermost sector not ok on one frame of the window: the band isn't seen either
  rep = report(live(bad=53., bad_at=T - 1.0 - 0.1))
  assert lane_of(rep, 'left').reach_m == pytest.approx(d40, abs=sr.WALK_STEP_M)


# ------------------------------------------------------------------------------------------------------- sector quality

def test_E12_a_sector_degraded_on_one_frame_of_the_window_zeroes_it_for_the_window():
  ln = next(x for x in lanes() if x.role == 'left')
  bad_b = 62.
  d_bad = ln.s_min * math.tan(math.radians(bad_b - 2.5 - ROT))  # inside the 57.5-62.5 deg sector after rotation
  ok = report(live())
  assert lane_of(ok, 'left').reach_m > d_bad
  # degraded on one run of the left side view 1.5 s ago, ok since: still zero (inside CLEAR_S)
  rep = report(live(bad=bad_b - 2.5, bad_at=T - 1.5))
  x = lane_of(rep, 'left')
  assert x.why == 'sector' and x.reach_m < d_bad
  # the same run 2.5 s ago (out of the window): seen again
  lv = live(bad=bad_b - 2.5, bad_at=T - 1.5)
  for k in range(1, 6):
    t = T + 0.2 * k
    for v in ('full', 'sideLeft', 'road', 'sideRight'):
      side = v.startswith('side')
      lv.observe_run(v, t, raw=RawFrame([], 'road' if v == 'road' else 'wide', t, v if side else 'full'), sectors=sectors(v) if side else None)
  assert lane_of(report(lv), 'left').reach_m == pytest.approx(lane_of(ok, 'left').reach_m, abs=1e-6)


@pytest.mark.parametrize("quality", ['dark', 'frozen', 'blocked', 'washed'])
def test_E11_a_dark_frozen_blocked_or_washed_sector_has_no_reach(quality):
  ln = next(x for x in lanes() if x.role == 'right')
  rep = report(live(bad=57.5, quality=quality))
  x = lane_of(rep, 'right')
  assert x.why == 'sector' and x.reach_m <= ln.s_min * math.tan(math.radians(57.5 - ROT)) + sr.WALK_STEP_M


def test_E4_fog_a_degraded_sector_zeroes_reach():
  ln = next(x for x in lanes() if x.role == 'left')
  rep = report(live(bad=52.5, quality='degraded'))
  # the innermost sector degraded on every frame: the band and the side view's sectors from it are unseen
  x = lane_of(rep, 'left')
  assert x.why == 'sector' and x.reach_m <= ln.s_min * math.tan(math.radians(sr.INNER_BAND_LO_DEG - ROT)) + sr.WALK_STEP_M


def test_a_side_view_with_no_sector_reading_in_the_window_sees_nothing_past_the_band():
  lv = LiveVisibility()
  for k in range(11):
    t = T - 2.0 + 0.2 * k
    for v in ('full', 'sideLeft', 'road', 'sideRight'):
      lv.observe_run(v, t, raw=RawFrame([], 'wide', t, v if v.startswith('side') else 'full'))  # no sectorQuality at all
  assert all(x.why == 'sector' for x in report(lv).lanes if x.role in ('left', 'right'))


# ------------------------------------------------------------------------------------------------------------ the sun

def test_E1_sun_ahead_zeroes_67_deg_to_the_edge_until_the_veil_is_measured_none():
  ln = next(x for x in lanes() if x.role == 'left')
  rep = report(live(), lighting='sunAhead')
  x = lane_of(rep, 'left')
  assert x.why == 'sunAhead' and x.reach_m <= ln.s_min * math.tan(math.radians(sr.SUN_AHEAD_ZERO_DEG - ROT)) + sr.WALK_STEP_M
  # the drives showed no veil: the day row's reach
  rep = report(live(), vis=vis_file(veil='none'), lighting='sunAhead')
  assert lane_of(rep, 'left').reach_m == pytest.approx(walk('left'), abs=1e-6)


def test_E1_low_sun_along_an_approach_zeroes_that_sides_side_view_whatever_the_veil():
  rep = report(live(), vis=vis_file(veil='none'), low_sun_sides=('right',))
  assert lane_of(rep, 'right').why == 'lowSun' and lane_of(rep, 'left').why == 'range'


def test_night_or_dusk_has_no_table_so_no_reach():
  for row in ('night', 'dusk'):
    assert all(x.reach_m == 0. for x in report(live(), lighting=row).lanes)


# ---------------------------------------------------------------------------------------------------- the survey's caps

def survey_entry(sight=60., parking=40.):
  approaches = [{'wayId': w, 'bearingDeg': b, 'stops': True, 'plaque': True, 'postedMph': 25, 'laneEdgeM': 4.0, 'lineM': 7.0,
                 'nearestParkingM': {'left': parking, 'right': parking}, 'sightM': sight, 'stopPointM': None}
                for w, b in ((11, 90.), (12, 270.), (13, 180.), (0, 0.))]
  return {'osmNodeId': 99, 'lat': 39.8, 'lon': -86.15, 'date': '2026-10-01', 'source': 'inPerson', 'confirmedBy': [],
          'approaches': approaches, 'validations': []}


def test_G2_a_hedge_the_surveys_sight_distance_caps_every_lane_stopping_ones_too():
  rep = report(live(), entry=survey_entry(sight=12.))
  for role in ('left', 'right'):
    x = lane_of(rep, role)
    assert x.stopping and x.why == 'cap' and x.reach_m == pytest.approx(12., abs=sr.WALK_STEP_M)


def test_G5_a_curving_cross_road_is_capped_by_the_surveys_sight_distance():
  # no lane polyline reaches turnpilotd (JunctionApproach carries none): the survey's sight distance is what bounds a lane
  # that curves away (row G5's polyline walk is not built)
  rep = report(live(), entry=survey_entry(sight=18.))
  assert all(x.reach_m <= 18. + 1e-6 for x in rep.lanes if x.role in ('left', 'right'))


def test_G3_a_parked_car_caps_the_lane_at_the_parking_distance():
  # legal parking from 21 m on the binding curb (the left approach's right curb): a parked car assumed there, a bearing-only
  # occluder at sLaneMin
  rep = report(live(), entry=survey_entry(parking=21.))
  ln = next(x for x in lanes() if x.role == 'left')
  x = lane_of(rep, 'left')
  assert x.why == 'cap' and x.reach_m == pytest.approx(sr.parking_cap(ln.s_min, 21.), abs=sr.WALK_STEP_M)


# ----------------------------------------------------------------------------------------------------------- occluders

def test_a_moving_track_caps_reach_where_its_bearing_meets_the_lane():
  ln = next(x for x in lanes() if x.role == 'left')
  car = trk(1, 15., ln, vy=-8.)  # receding or approaching, it hides what is behind it
  rep = report(live(), tracks=[car])
  x = lane_of(rep, 'left')
  lo = min(abs(b) for b in car.span_seen)
  assert x.why == 'occluder' and x.reach_m == pytest.approx(ln.s_min * math.tan(math.radians(lo - ROT)), abs=sr.WALK_STEP_M)


def test_G8_a_stopped_car_in_lane_1_caps_lane_2_behind_it():
  j = junction(25.)
  j.approaches[0].lanes_in = 2
  lns = junction_lanes(j, 'straight', SETBACK)
  l1, l2 = (next(x for x in lns if (x.role, x.index) == ('left', i)) for i in (1, 2))
  car = trk(1, 9., l1)
  rep = live().lanes(lns, 0., vis_file(), rot_deg=ROT, edges=EDGES, tracks=[car])
  free = live().lanes(lns, 0., vis_file(), rot_deg=ROT, edges=EDGES)
  x1, x2 = (next(x for x in rep.lanes if (x.role, x.index) == ('left', i)) for i in (1, 2))
  lo = min(abs(b) for b in car.span_seen)
  assert x2.why == 'occluder' and x2.reach_m == pytest.approx(l2.s_min * math.tan(math.radians(lo - ROT)), abs=sr.WALK_STEP_M)
  assert x2.reach_m < next(x for x in free.lanes if (x.role, x.index) == ('left', 2)).reach_m
  assert x1.why == 'occluder'  # and its own lane at itself


def test_G10_a_car_stopped_in_a_centre_turn_lane_caps_the_far_lanes_behind_it():
  ln = next(x for x in lanes() if x.role == 'right')
  # the centre lane lies between the near and far lanes: 3.5 m nearer than the far lane's centre, 8 m down the road
  centre = SimpleNamespace(**vars(trk(1, 8., ln, view='full')))
  s = (ln.s_min + ln.s_max) / 2. - 3.5
  psi, half = math.degrees(math.atan2(8., s)), math.degrees(math.atan2(0.9, math.hypot(8., s)))
  centre.bearing_seen, centre.span_seen, centre.x = -psi, (-psi - half, -psi + half), np.array([s, -8., 0., 0.])
  rep = report(live(), tracks=[centre])
  x = lane_of(rep, 'right')
  assert x.why == 'occluder' and x.reach_m == pytest.approx(ln.s_min * math.tan(math.radians(psi - half - ROT)), abs=sr.WALK_STEP_M)


def test_a_steady_raw_box_becomes_an_occluder_and_stops_blocking():
  ln = next(x for x in lanes() if x.role == 'left')
  b = 60.
  box = raw_box(b, conf=0.36)  # never a track
  # seen in the left side view for 0.4 s: a blocker of the near lane, no occluder yet
  lv = live()
  for k in range(3):
    t = T + 0.2 * (k + 1)
    lv.observe_run('sideLeft', t, raw=RawFrame([box], 'wide', t, 'sideLeft'), sectors=sectors('sideLeft'))
    for v in ('full', 'road', 'sideRight'):
      lv.observe_run(v, t, raw=RawFrame([], 'wide', t, v if v.startswith('side') else 'full'), sectors=sectors(v) if v.startswith('side') else None)
  x = lane_of(report(lv), 'left')
  assert x.blockers and x.why == 'range'
  # held still (bearing within 0.3 deg, width within 5%) for 1.0 s: an occluder, no blocker
  for k in range(3, 7):
    t = T + 0.2 * (k + 1)
    lv.observe_run('sideLeft', t, raw=RawFrame([raw_box(b + 0.1, conf=0.36)], 'wide', t, 'sideLeft'), sectors=sectors('sideLeft'))
    for v in ('full', 'road', 'sideRight'):
      lv.observe_run(v, t, raw=RawFrame([], 'wide', t, v if v.startswith('side') else 'full'), sectors=sectors(v) if v.startswith('side') else None)
  x = lane_of(report(lv), 'left')
  assert not x.blockers and x.why == 'occluder'
  assert x.reach_m == pytest.approx(ln.s_min * math.tan(math.radians(b + 0.1 - 1.0 - ROT)), abs=sr.WALK_STEP_M)
  # it moves: a blocker again at once
  t = T + 1.6
  lv.observe_run('sideLeft', t, raw=RawFrame([raw_box(b - 1.5, conf=0.36)], 'wide', t, 'sideLeft'), sectors=sectors('sideLeft'))
  x = lane_of(report(lv), 'left')
  assert x.blockers and x.why != 'occluder'


def test_a_raw_box_near_the_horizon_outside_every_band_still_blocks():
  # a box 1 deg under the horizon (about 75 m) at 20 deg left: its ray meets the near lane far past its need, but a far box's
  # ground range can't be trusted, so near the horizon it blocks that side's lanes whatever its nominal band
  rng = HEIGHT / math.tan(math.radians(1.0))
  far = raw_box(20., xy=(rng * math.cos(math.radians(20.)), rng * math.sin(math.radians(20.))), view='full')
  lv = live()
  step(lv, T + 0.2, {'full': [far]})
  assert [b for b in lane_of(report(lv), 'left').blockers if 'horizon' in b]
  # the same box 8 m ahead: in the near lane's band by its ground position, a plain blocker
  low = raw_box(20., xy=(8., 8. * math.tan(math.radians(20.))), view='full')
  lv = live()
  step(lv, T + 0.2, {'full': [low]})
  blockers = lane_of(report(lv), 'left').blockers
  assert blockers and not [b for b in blockers if 'horizon' in b]
  # 30 m ahead at 20 deg (past the cross road, under the horizon by 2.4 deg): no band, not near the horizon, no blocker
  mid = raw_box(20., xy=(30., 30. * math.tan(math.radians(20.))), view='full')
  lv = live()
  step(lv, T + 0.2, {'full': [mid]})
  assert not lane_of(report(lv), 'left').blockers


# ----------------------------------------------------------------------------------------------------------------- mask

def test_the_footprint_mask_cuts_a_housing_the_ground_point_misses():
  # a housing at 60-65 deg left that leaves the band only up to 0.3 deg above the horizon: a car-sized footprint's top (1.5
  # m, over the 1.3 m camera) at 15-20 m sits 0.6-0.8 deg up, inside it; the ground point (-4 deg) never is
  mask = open_mask()
  for row in mask['left']:
    if 60 <= row[0] <= 65:
      row[2] = 0.3
  vis = vis_file(mask=mask)
  ln = next(x for x in lanes() if x.role == 'left')
  x = lane_of(report(live(), vis=vis), 'left')
  # the walk ends at the first point whose footprint (1.8 m wide) spans a housing bin
  d = 0.
  while True:
    xm, ym = ln.point(d + sr.WALK_STEP_M, 'max')
    b = math.degrees(math.atan2(d + sr.WALK_STEP_M, ln.s_min)) + ROT
    if math.floor(b + math.degrees(math.atan2(0.9, math.hypot(xm, ym)))) >= 60:
      break
    d += sr.WALK_STEP_M
  cut = d
  assert x.why == 'mask' and x.reach_m == pytest.approx(cut, abs=1e-6) and cut < ln.s_min * math.tan(math.radians(60. - ROT))
  xm, ym = ln.point(cut + 0.5, 'max')
  ground_el = -math.degrees(math.atan2(HEIGHT, math.hypot(xm, ym)))
  assert -40. < ground_el < 0.3  # the ground point alone would have passed the housing
  # no mask (before step S1): nothing is seen, never a free pass
  nomask = copy.deepcopy(vis.raw)
  nomask['mask'] = None
  rep = report(live(), vis=vt.Visibility(nomask, tables=vt.validate(nomask)))
  assert all(x.reach_m == 0. and x.why == 'noMask' for x in rep.lanes)


# ----------------------------------------------------------------------------------------------------------- the rest

def test_shield_candidates_and_the_lane_edge_estimate_are_logged():
  from openpilot.selfdrive.turnpilot.stop_evidence import TrackEvidence
  ln = next(x for x in lanes() if x.role == 'left')
  ev = TrackEvidence(7, 'car', 'left', 1, 'sideLeft', T, 8, 9., 0.1, True, 0., False, None, None, 11., 0., float('inf'),
                     float('inf'), None, None, 'stopped', 0.3, True, 1.4, True, False, ln)
  rep = report(live(), evidence={7: ev})
  x = lane_of(rep, 'left')
  assert x.shields == [7] and x.candidates == [7]
  # cross cars at bearings under 60 deg moving across: the near lane's edge estimate from their forward distance (logged only)
  cars = [SimpleNamespace(**{**vars(trk(i, d, ln, view='full', vy=-9.)), 'track_id': i}) for i, d in ((1, 6.), (2, 8.))]
  rep = report(live(), tracks=cars)
  est = rep.edge_estimate['left']
  s = (ln.s_min + ln.s_max) / 2.
  assert est['n'] == 2 and est['xMedM'] == pytest.approx(s, abs=1e-6)
  assert est['edgeFromStopM'] == pytest.approx(s - SETBACK - (sr.NEAR_LANE_MIN_M + sr.NEAR_LANE_MAX_M) / 2., abs=1e-6)


def test_ray_lane_d():
  ln = next(x for x in lanes() if x.role == 'left')
  assert ray_lane_d(ln, 60., 'min') == pytest.approx(ln.s_min * math.tan(math.radians(60.)), abs=1e-9)
  opp = next(x for x in lanes() if x.role == 'opposite')
  x = abs(opp.y) / math.tan(math.radians(10.))
  assert ray_lane_d(opp, 10., 'min') == pytest.approx(x - opp.s_min, abs=1e-9)


def test_a_non_stopping_lanes_need_is_the_two_way_requirement_from_the_edge_bands_far_end():
  j = junction(25., tier='mapStrong')  # not surveyed: non-stopping this week (D10)
  lns = junction_lanes(j, 'straight', SETBACK)
  rep = live().lanes(lns, 0., vis_file(), rot_deg=ROT, edges=EDGES)
  x = next(x for x in rep.lanes if x.role == 'left')
  assert not x.stopping and x.need_m == pytest.approx(sr.reach_needed_two_way(25. * sr.MPH, EDGES.d_max), abs=1e-6)


def test_rule_5_a_non_stopping_lane_without_a_surveyed_sight_distance_has_no_reach():
  # STOP_SIGNS.md 1.4 rule 5, row G2: YOLO reports cars, not hedges, fences, walls or crests, so a lane whose approach doesn't
  # stop has reach 0 unless its approach's sight distance was surveyed. SG3b skeptic p4: at an unsurveyed two-way (tier
  # mapStrong), zero detections, every view fresh and ok, the left lane read 27.2 m, the right 24.5 and the opposite 127.5
  # (R_seen 114.6, over the 61.9-93.7 m two-way need at 10-15 mph) by the tables' walk, why 'range'
  lns = junction_lanes(junction(25., tier='mapStrong'), 'straight', SETBACK)
  for entry in (None, {**survey_entry(), 'approaches': [a for a in survey_entry()['approaches'] if a['wayId'] == 0]}):
    rep = live().lanes(lns, 0., vis_file(), rot_deg=ROT, edges=EDGES, entry=entry)
    assert {x.role for x in rep.lanes} == {'left', 'right', 'opposite'}
    for x in rep.lanes:
      assert not x.stopping and (x.reach_m, x.r_seen_m, x.why) == (0., 0., 'notSurveyed')
  # its approach surveyed: the survey's sight distance bounds it like any lane's (rule 6)
  rep = live().lanes(lns, 0., vis_file(), rot_deg=ROT, edges=EDGES, entry=survey_entry(sight=20.))
  for x in rep.lanes:
    assert x.why in ('cap', 'range') and 0. < x.reach_m <= 20. + 1e-6
  # a stopping lane is untouched: without an entry it is the tables' walk (rule 3)
  assert lane_of(report(live()), 'left').why == 'range' and lane_of(report(live()), 'left').reach_m > 0.


def test_V11_a_near_lane_car_turning_into_our_road_caps_the_lane_behind_it_until_it_has_left_the_lane_band():
  # a car slowing in the near lane to turn into our road, a follower behind it: while it comes down the lane (moving or
  # slowing) its box caps the lane where its bearing meets the lane at sLaneMin (rule 4), so the stretch it hides is never
  # counted seen; once it has turned out of the lane and gone past us, the lane is walked in full again (and the follower, a
  # new track, is U2's: untrusted, never stopping)
  ln = next(x for x in lanes() if x.role == 'left')
  free = lane_of(report(live()), 'left').reach_m
  for d, vy in ((25., -8.), (15., -6.), (10., -3.)):
    car = trk(1, d, ln, vy=vy)
    x = lane_of(report(live(), tracks=[car]), 'left')
    lo = min(abs(b) for b in car.span_seen)
    assert x.why == 'occluder' and x.reach_m == pytest.approx(ln.s_min * math.tan(math.radians(lo - ROT)), abs=sr.WALK_STEP_M)
    assert x.reach_m < free
  # turned into our road and coming toward us in its oncoming lane 8 m ahead, its box may still lie nearer than the lane at
  # its bearing (ranging trusted under 40 deg at OCCLUDER_NEAR_FRAC): the lane's first metres capped, never more counted seen
  turned = SimpleNamespace(**vars(trk(1, 2., ln, vx=-3.)))
  turned.bearing_seen, turned.span_seen, turned.x = 20.6, (17.6, 23.6), np.array([8., 3.0, -3., 0.])
  assert lane_of(report(live(), tracks=[turned]), 'left').reach_m < free
  # gone past us: the lane walked in full again
  assert lane_of(report(live(), tracks=[]), 'left').reach_m == pytest.approx(free, abs=sr.WALK_STEP_M)


def test_V16_a_car_double_parked_in_the_near_lane_caps_it_and_isnt_its_shield():
  # standing 20 m down the left lane, not at its line: an occluder capping the lane at its bearing, never a shield candidate
  ln = next(x for x in lanes() if x.role == 'left')
  car = trk(1, 20., ln)
  x = lane_of(report(live(), tracks=[car]), 'left')
  lo = min(abs(b) for b in car.span_seen)
  assert x.why == 'occluder' and x.reach_m == pytest.approx(ln.s_min * math.tan(math.radians(lo - ROT)), abs=sr.WALK_STEP_M)
  assert not x.shields
