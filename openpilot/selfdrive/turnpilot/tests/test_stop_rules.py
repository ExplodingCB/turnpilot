"""The shared stop-sign rules (turnpilot/stop_rules.py; STOP_SIGNS.md 1.2, 1.4-1.6, 2.1-2.3, 2.6, 2.8 and 9).

Expected values are copied here because repo tests can't read ~/turnpilot-artifacts: the requirement rows and the reach
table of STOP_SIGNS.md section 2.8 (T_LAT 1.0 s, after measurement), and the abort distances and the parking caps of
stopsigns/design_v3/v3_tables.out (its requirement rows predate T_LAT 1.0 s, so they aren't expected values here).
"""
import math
from datetime import UTC, datetime
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.turnpilot import stop_rules as sr

MPH = 0.44704

# STOP_SIGNS.md 2.8: reach needed, including latency (m), and 2.3's table to one decimal
ALLWAY_NEED = {10: 14.5, 15: 22.6, 20: 32.9, 25: 45.4, 30: 60.1, 35: 77.0}
TWO_WAY_NUDGE = {10: 62, 15: 94, 20: 128, 25: 164, 30: 202}   # G5 nudge launch, default stop point (dEdgeMax 13.0 m)
TWO_WAY_MODEL = {10: 46, 15: 70, 20: 96, 25: 125, 30: 155}    # model launch 1.5 m/s^2
# v3_tables.out: the capped stop at -2.0 after T_d 0.55 s
ABORT_055 = {0.7: 0.51, 1.0: 0.80, 1.5: 1.39, 2.0: 2.10, 3.0: 3.90}

# 2.8's reach table: (near s_min, near s_max, far s_min, far s_max, rotation) as v3_tables.py places the lanes, and the
# cells (near, far) per column, PV1 side views conservative and optimistic, sun ahead, side views off; lens k = 0.3
GEOMETRY_28 = {
  'unsurveyed, node - 13': (6.7, 16.75, 9.7, 20.45, 3.5),
  'surveyed, node - 11': (8.4, 13.75, 11.9, 17.45, 2.5),
  'surveyed, node - 13': (10.4, 15.75, 13.9, 19.45, 2.5),
}
CELLS_28 = {
  'unsurveyed, node - 13': {'cons': (27, 25), 'opt': (40, 39), 'sun': (13, 19), 'off': (5, 7)},
  'surveyed, node - 11': {'cons': (29, 27), 'opt': (42, 40), 'sun': (18, 25), 'off': (6, 9)},
  'surveyed, node - 13': {'cons': (28, 25), 'opt': (41, 39), 'sun': (22, 25), 'off': (8, 11)},
}
LIMIT_28 = {'cons': 15, 'opt': 20}  # every geometry
SUN_LIMIT_28 = {'unsurveyed, node - 13': None, 'surveyed, node - 11': 10, 'surveyed, node - 13': 10}
EDGE_K03 = 88.1  # deg, the frame edge at k = 0.3 (verify.md)
EDGE_K0 = 85.2


def pv1(bound='cons', side=True):
  """PV1 (STOP_SIGNS.md 1.6) as views: road camera 150 m under 20 deg, full wide pass 28 m to 40 deg, side views 32 m
  (44 m optimistic) from their inner edge."""
  views = {'road': [[0., 150.], [20., 0.]], 'full': [[0., 28.], [40., 0.]]}
  if side:
    views['side'] = [[0., 0.], [sr.INNER_BAND_HI_DEG, 32. if bound == 'cons' else 44.]]
  return views


def sight(bound='cons', side=True, edge=EDGE_K03, **kw):
  return sr.Sight(edge_deg={'left': edge, 'right': edge}, views=pv1(bound, side), **kw)


def lanes_28(name):
  sn_min, sn_max, sf_min, sf_max, rot = GEOMETRY_28[name]
  return (sr.Lane('left', 1, 'cross', sn_min, sn_max, -90.), sr.Lane('right', 1, 'cross', sf_min, sf_max, 90.)), rot


class TestRequirements:
  @pytest.mark.parametrize('mph', sorted(ALLWAY_NEED))
  def test_allway_reach_needed(self, mph):
    assert sr.reach_needed_allway(mph * MPH) == pytest.approx(ALLWAY_NEED[mph], abs=0.06)

  def test_allway_uses_a_aw_3_and_m_stop_5(self):
    vc = sr.V_CROSS_FACTOR * 25 * MPH
    assert sr.r_req_allway(25 * MPH) == pytest.approx(vc * vc / (2 * 3.0) + 5.0)
    assert (sr.A_AW, sr.M_STOP, sr.T_LAT) == (3.0, 5.0, 1.0)

  @pytest.mark.parametrize('mph', sorted(TWO_WAY_NUDGE))
  def test_two_way_reach_needed(self, mph):
    assert sr.reach_needed_two_way(mph * MPH, 13.0) == pytest.approx(TWO_WAY_NUDGE[mph], abs=0.5)
    assert sr.reach_needed_two_way(mph * MPH, 13.0, a_launch=1.5) == pytest.approx(TWO_WAY_MODEL[mph], abs=0.5)

  def test_t_cue_is_the_go_latency_plus_the_launch_to_the_far_edge(self):
    assert sr.t_cue(13.0) == pytest.approx(0.5 + math.sqrt(2 * 13.0 / 0.5))
    assert sr.t_cue(13.0, 1.5) == pytest.approx(0.5 + math.sqrt(2 * 13.0 / 1.5))

  def test_a_launch_plan_from_the_param_clamped(self):
    def params(v):
      return SimpleNamespace(get=lambda key: v)
    assert sr.a_launch_plan(params(None)) == 0.5
    assert sr.a_launch_plan(params(1.5)) == 1.5
    assert sr.a_launch_plan(params(9.0)) == 2.0
    assert sr.a_launch_plan(params(0.1)) == 0.3
    assert sr.a_launch_plan(params('junk')) == 0.5

  def test_allway_limit_from_reach(self):
    assert sr.allway_limit_mph(45.4) == 25
    assert sr.allway_limit_mph(45.3) == 20
    assert sr.allway_limit_mph(14.0) is None


class TestGeometry:
  def test_unsurveyed_two_lane_edge_band_and_lane_offsets(self):
    e = sr.edge_band(13.0, 2)
    assert (e.d_min, e.d_max) == (pytest.approx(3.3), pytest.approx(13.0))
    assert sr.lane_offsets(e, 1.9, 1) == (pytest.approx(6.7), pytest.approx(16.75))
    assert sr.lane_offsets(e, 1.9, 2) == (pytest.approx(9.7), pytest.approx(20.45))

  def test_surveyed_edge_band(self):
    for stop, (lo, hi) in ((11.0, (5.0, 10.0)), (13.0, (7.0, 12.0))):
      e = sr.edge_band(stop, 2, lane_edge_m=3.5)
      assert (e.d_min, e.d_max, e.nominal) == (pytest.approx(lo), pytest.approx(hi), pytest.approx(stop - 3.5))
    assert sr.lane_offsets(sr.edge_band(13.0, 2, lane_edge_m=3.5), 1.9, 1) == (pytest.approx(10.4), pytest.approx(15.75))

  def test_four_lane_cross_road_moves_the_closest_edge(self):
    assert sr.edge_band(16.7, 4).d_min == pytest.approx(16.7 - (4 * 3.7 / 2 + 3.0 + 1.0) - 2.0)
    assert sr.edge_band(16.7, 4).d_max == pytest.approx(16.7 - (4 * 3.0 / 2 - 1.0) + 2.0)

  def test_angle_bound(self):
    assert sr.angle_bound(surveyed=False) == pytest.approx(3.5)  # route time: 1 deg of yaw, 2 deg map, 0.5 deg calibration
    assert sr.angle_bound(surveyed=True) == pytest.approx(2.5)
    assert sr.angle_bound(surveyed=True, yaw_offset_deg=3.0) == pytest.approx(4.5)


class TestReachWalk:
  @pytest.mark.parametrize('name', sorted(GEOMETRY_28))
  def test_section_2_8_cells_and_candidate_limits(self, name):
    (near, far), rot = lanes_28(name)
    cells = CELLS_28[name]
    for col, s in (('cons', sight('cons')), ('opt', sight('opt')), ('off', sight('cons', side=False)),
                   ('sun', sight('cons', zero=sr.sun_ahead_zero))):
      got = (sr.reach_walk(near, s, rot), sr.reach_walk(far, s, rot))
      assert got[0] == pytest.approx(cells[col][0], abs=0.5), (name, col, got)
      assert got[1] == pytest.approx(cells[col][1], abs=0.5), (name, col, got)
      if col in LIMIT_28:
        assert sr.allway_limit_mph(min(got)) == LIMIT_28[col]
      elif col == 'off':
        assert sr.allway_limit_mph(min(got)) is None  # nothing certifies with the side views off
      else:
        assert sr.allway_limit_mph(min(got)) == SUN_LIMIT_28[name]

  def test_k0_frame_edge_cuts_the_unsurveyed_near_lane(self):
    (near, far), rot = lanes_28('unsurveyed, node - 13')
    assert sr.reach_walk(near, sight('opt', edge=EDGE_K0), rot) == pytest.approx(39., abs=0.5)
    assert sr.reach_walk(far, sight('opt', edge=EDGE_K0), rot) == pytest.approx(39., abs=0.5)

  def test_bearing_at_s_min_and_range_at_s_max(self):
    lane = sr.Lane('left', 1, 'cross', 6.7, 16.75, -90.)
    # range-bound: sqrt(32^2 - 16.75^2) = 27.3 at s_max; at s_min alone it would be 31.3
    assert sr.reach_walk(lane, sight('cons'), 3.5) == pytest.approx(math.sqrt(32 ** 2 - 16.75 ** 2), abs=0.25)
    # bearing-bound: side views off stop at 40 deg of rotated bearing at s_min
    assert sr.reach_walk(lane, sight('cons', side=False), 3.5) == pytest.approx(6.7 * math.tan(math.radians(36.5)), abs=0.25)

  def test_inner_band_with_a_stale_side_view_ends_at_40_deg(self):
    lane = sr.Lane('left', 1, 'cross', 6.7, 16.75, -90.)
    stale = sight('cons', fresh=lambda view, side: view != 'side')
    assert sr.reach_walk(lane, stale, 3.5) == pytest.approx(6.7 * math.tan(math.radians(36.5)), abs=0.25)
    fresh_right_only = sight('cons', fresh=lambda view, side: view != 'side' or side == 'right')
    assert sr.reach_walk(lane, fresh_right_only, 3.5) == pytest.approx(6.7 * math.tan(math.radians(36.5)), abs=0.25)
    assert sr.reach_walk(lane, sight('cons', fresh=lambda view, side: True), 3.5) > 25.

  def test_validated_entry_bounds_the_distance_along_the_lane_capped_by_the_frame_edge(self):
    lane = sr.Lane('left', 1, 'cross', 10.4, 15.75, -90.)
    s = sight('cons')
    assert sr.reach_walk(lane, s, 2.5, validated_m=48.) == pytest.approx(48., abs=0.25)
    near_edge = sr.Lane('left', 1, 'cross', 4.0, 8.0, -90.)
    edge_d = 4.0 * math.tan(math.radians(EDGE_K03 - sr.EDGE_MARGIN_DEG - 2.5))
    assert sr.reach_walk(near_edge, s, 2.5, validated_m=200.) == pytest.approx(edge_d, abs=0.25)

  def test_caps_mask_and_occluders(self):
    lane = sr.Lane('left', 1, 'cross', 10.4, 15.75, -90.)
    assert sr.reach_walk(lane, sight('opt'), 2.5, cap_m=30.) == pytest.approx(30., abs=0.25)
    # a mask that blocks 70-75 deg on the left ends the walk at 70 deg (rotated)
    masked = sight('opt', mask=lambda side, bearing, rng: not (side == 'left' and 70. <= bearing < 75.))
    assert sr.reach_walk(lane, masked, 2.5) == pytest.approx(10.4 * math.tan(math.radians(67.5)), abs=0.3)
    # a bearing-only occluder at 60-65 deg caps the lane where its bearing meets the lane at s_min
    occ = sight('opt', occluders=[sr.Occluder('left', 60., 65., None)])
    assert sr.reach_walk(lane, occ, 2.5) == pytest.approx(10.4 * math.tan(math.radians(57.5)), abs=0.3)
    # one past the lane's range doesn't hide anything nearer
    occ_far = sight('opt', occluders=[sr.Occluder('left', 60., 65., 500.)])
    assert sr.reach_walk(lane, occ_far, 2.5) == pytest.approx(sr.reach_walk(lane, sight('opt'), 2.5), abs=0.01)

  def test_swing_zone_reach_counts_from_its_conflict_point(self):
    lane = sr.Lane('right', 1, 'swing', 13.9, 19.45, 90., conflict_m=sr.SWING_ZONE_M)
    full = sr.reach_walk(sr.Lane('right', 1, 'cross', 13.9, 19.45, 90.), sight('opt'), 2.5)
    assert sr.reach_walk(lane, sight('opt'), 2.5) == pytest.approx(full - sr.SWING_ZONE_M, abs=0.3)


class TestCaps:
  def test_parking_cap_matches_v3_tables(self):
    # v3_tables.out: bearing-only cap 20.5 / 29.3 / 44.0 m (s_min 8.4) and 20.6 / 29.4 / 44.2 m (s_min 10.4)
    for s_min, caps in ((8.4, (20.5, 29.3, 44.0)), (10.4, (20.6, 29.4, 44.2))):
      for p, cap in zip((21., 30., 45.), caps, strict=True):
        assert sr.parking_cap(s_min, p) == pytest.approx(cap, abs=0.06)

  def test_parking_legal_to_the_corner_caps_at_zero(self):
    assert sr.parking_cap(10.4, 0.) == 0.


class TestAbort:
  @pytest.mark.parametrize('v', sorted(ABORT_055))
  def test_abort_distance(self, v):
    assert sr.abort_distance(v, 0.55) == pytest.approx(ABORT_055[v], abs=0.01)

  def test_t_abort_takes_the_larger_delay(self):
    assert sr.t_abort(0.40) == pytest.approx(0.55)
    assert sr.t_abort(0.40, 0.60) == pytest.approx(0.75)
    assert sr.t_abort(0.40, 0.20) == pytest.approx(0.55)
    assert sr.t_abort(0.40, None) == pytest.approx(0.55)

  def test_go_room(self):
    assert sr.go_room_m(0.55) == pytest.approx(2.4, abs=0.01)
    assert sr.go_room_m(0.55, right=True) == pytest.approx(2.9, abs=0.01)

  def test_points_along_the_path(self):
    e = sr.edge_band(13.0, 2, lane_edge_m=3.5)  # dEdgeMin 7.0, dEdgeMax 12.0, nominal 9.5
    # at 1.5 m/s, abort 1.39 m: commit at 1.69 m to dEdgeMin, gate at 1.99 m (the first argument: m moved from the stop point)
    assert not sr.gate_reached(4.9, 1.5, 0.55, e)
    assert sr.gate_reached(5.1, 1.5, 0.55, e) and not sr.committed(5.1, 1.5, 0.55, e)
    assert sr.committed(5.4, 1.5, 0.55, e)
    assert sr.band_midpoint(e) == pytest.approx(9.5)  # the nominal surveyed edge
    assert sr.band_midpoint(sr.edge_band(13.0, 2)) == pytest.approx((3.3 + 13.0) / 2)
    assert not sr.in_lane(11.9, e) and sr.in_lane(12.1, e)


def approach(role, rel, tier='surveyed', lanes=1, inbound=True, limit=20 * MPH, exit_=False, **kw):
  return sr.Approach(bearing=0., rel_bearing=rel, role='exit' if exit_ else role, inbound=inbound, outbound=True, lanes_in=lanes,
                     lanes_out=lanes, control='stop', tier=tier, assumed_limit=limit, **kw)


def four_way(**kw):
  return sr.Junction(approaches=[approach('left', -90.), approach('right', 90.), approach('opposite', 0., exit_=True)], **kw)


class TestConflictSets:
  def moves(self, j, maneuver, yaw=0.):
    return sorted((m.role, m.movement) for m in sr.conflict_movements(j, maneuver, yaw))

  def test_straight(self):
    assert self.moves(four_way(), 'straight') == [('left', 'cross'), ('opposite', 'leftTurners'), ('right', 'cross')]

  def test_a_right_is_the_union_with_the_straight_set_until_15_deg(self):
    j = sr.Junction(approaches=[approach('left', -90.), approach('right', 90., exit_=True), approach('opposite', 0.)])
    assert self.moves(j, 'right') == [('left', 'cross'), ('opposite', 'leftTurners'), ('right', 'cross'), ('right', 'swing')]
    assert self.moves(j, 'right', yaw=16.) == [('left', 'cross'), ('opposite', 'leftTurners'), ('right', 'swing')]

  def test_a_left_and_its_exit_leg(self):
    # a T, the car on the stem turning left: the exit leg is the left approach (its inbound lanes are the near lanes)
    j = sr.Junction(approaches=[approach('left', -90., exit_=True), approach('right', 90.)])
    assert self.moves(j, 'left', yaw=20.) == [('left', 'cross'), ('right', 'cross')]
    assert self.moves(j, 'left') == [('left', 'cross'), ('right', 'cross')]

  def test_left_conflicts_with_opposing_straight_and_right(self):
    assert ('opposite', 'straightRight') in self.moves(four_way(), 'left')

  def test_unknown_is_the_union_of_all_three(self):
    assert self.moves(four_way(), 'unknown') == [('left', 'cross'), ('opposite', 'leftTurners'), ('opposite', 'straightRight'),
                                                 ('right', 'cross'), ('right', 'swing')]

  def test_an_approach_cars_only_leave_by_conflicts_with_nothing(self):
    j = sr.Junction(approaches=[approach('left', -90., inbound=False), approach('right', 90.)])
    assert self.moves(j, 'straight') == [('right', 'cross')]

  def test_lanes_offsets_and_swing(self):
    j = sr.Junction(stop_point_distance=13., approaches=[approach('left', -90.), approach('right', 90., exit_=True),
                                                          approach('opposite', 0.)])
    lanes = sr.conflict_lanes(j, 'right', sr.edge_band(13., 2), 1.9)
    by = {(ln.role, ln.movement): ln for ln in lanes}
    assert (by['left', 'cross'].s_min, by['right', 'cross'].s_min) == (pytest.approx(6.7), pytest.approx(9.7))
    assert by['right', 'swing'].conflict_m == sr.SWING_ZONE_M
    assert by['right', 'swing'].s_max == pytest.approx(20.45)

  def test_multi_lane_approach_has_a_lane_each(self):
    j = sr.Junction(approaches=[approach('left', -90., lanes=2), approach('right', 90., lanes=2)])
    lanes = sr.conflict_lanes(j, 'straight', sr.edge_band(16.7, 4), 1.9)
    s = sorted((ln.role, ln.index, round(ln.s_min, 2)) for ln in lanes)
    d_min = sr.edge_band(16.7, 4).d_min
    assert s == [('left', 1, round(d_min + 1.9 + 1.5, 2)), ('left', 2, round(d_min + 1.9 + 4.5, 2)),
                 ('right', 1, round(d_min + 1.9 + 7.5, 2)), ('right', 2, round(d_min + 1.9 + 10.5, 2))]


class TestTracks:
  @staticmethod
  def samples(d0, v0, a=0., s=10., dur=2.0, hz=5., conflict=0., width=True):
    """A car on a perpendicular lane at s, d0 m down it at t 0, moving toward us at v0 slowing at a."""
    out = []
    for k in range(int(dur * hz) + 1):
      t = k / hz
      tt = min(t, v0 / a) if a > 0 else t
      d = d0 - (v0 * tt - 0.5 * a * tt * tt)
      w = 1000. / math.hypot(d, s)
      out.append((t, math.degrees(math.atan2(d, s)), w if width else None))
    return out

  def test_a_far_car_is_far(self):
    fit = sr.fit_track(self.samples(60., 10.), 10., 10., 0.)
    assert fit.v_trusted and fit.v_hi == pytest.approx(10., abs=0.6)
    assert sr.u2_state(fit) == 'far'

  def test_a_close_car_at_speed_is_not_stopping(self):
    fit = sr.fit_track(self.samples(25., 10., dur=1.2), 10., 10., 0.)
    assert fit.v_trusted
    assert sr.u2_state(fit) == 'notStopping'

  def test_a_new_track_is_untrusted_whatever_its_range(self):
    fit = sr.fit_track(self.samples(80., 10., dur=0.6), 10., 10., 0.)
    assert not fit.v_trusted
    assert sr.u2_state(fit) == 'notStopping'

  def test_a_car_braking_hard_enough_is_proven_stopping(self):
    # 28 m out at 11 m/s braking at 5 m/s^2: too close to be 'far' (it needs more than A_U2), braking enough to stop short
    fit = sr.fit_track(self.samples(52., 19., a=5., dur=1.6), 10., 10., 0.)
    assert fit.a_trusted and fit.a_lo > 3.
    assert fit.v_hi ** 2 / (2. * (fit.d_lo - 3.)) > sr.A_U2
    assert sr.u2_state(fit) == 'stopping'

  def test_braking_without_the_box_cross_check_proves_nothing(self):
    fit = sr.fit_track(self.samples(52., 19., a=5., dur=1.6, width=False), 10., 10., 0.)
    assert sr.u2_state(fit) == 'notStopping'

  def test_braking_too_gently_for_its_range_is_not_stopping(self):
    fit = sr.fit_track(self.samples(30., 12., a=1.), 10., 10., 0.)
    assert sr.u2_state(fit) == 'notStopping'

  def test_s_band_and_angle_bound_are_pessimistic(self):
    exact = sr.fit_track(self.samples(40., 10.), 10., 10., 0.)
    band = sr.fit_track(self.samples(40., 10.), 9., 12., 2.5)
    assert band.d_lo < exact.d_lo and band.v_hi > exact.v_hi

  def test_stopped_at_its_line(self):
    fit = sr.fit_track(self.samples(8., 0.), 10., 10., 0.)
    assert sr.u2_state(fit, at_line=True) == 'stopped'

  def test_t_gap(self):
    assert sr.t_gap('straight') == 6.5 and sr.t_gap('right') == 6.5 and sr.t_gap('left') == 7.5
    assert sr.t_gap('left', extra_lanes=2) == 8.5

  def test_tracked_two_way_test_uses_a_detected_and_t_gap(self):
    t_cue = sr.t_cue(13.0)
    far = sr.fit_track(self.samples(200., 10., s=60.), 60., 60., 0.)
    near = sr.fit_track(self.samples(60., 10., s=60.), 60., 60., 0.)
    assert sr.tracked_ok(far, 'straight', t_cue, v_c=12.)
    assert not sr.tracked_ok(near, 'straight', t_cue, v_c=12.)
    untrusted = sr.fit_track(self.samples(200., 3., dur=0.6, s=60.), 60., 60., 0.)
    assert sr.tracked_ok(untrusted, 'straight', t_cue, v_c=12.) == (200. - 3. * 0.6 >= sr.tracked_r_req(12., t_cue))

  def test_shield(self):
    ok = {'kind': 'car', 'stopped_s': 1.2, 'v_hi': 0.2, 'samples': 6, 'front_from_line_m': -1.0, 'seen_every_cycle': True, 'moved_m': 0.1}
    assert sr.shield_ok(**ok)
    for k, v in (('kind', 'bicycle'), ('kind', 'motorcycle'), ('stopped_s', 0.8), ('v_hi', 0.6), ('samples', 4),
                 ('front_from_line_m', -3.5), ('front_from_line_m', 2.5), ('seen_every_cycle', False), ('moved_m', 0.35)):
      assert not sr.shield_ok(**dict(ok, **{k: v})), (k, v)
    assert sr.shield_ok(**dict(ok, kind='truck')) and sr.shield_ok(**dict(ok, kind='bus'))


def utc(*a):
  return datetime(*a, tzinfo=UTC).timestamp()


INDY = (39.7684, -86.1581)


class TestSun:
  def test_noaa_position_indianapolis_solar_noon(self):
    # 2026-10-28: declination about -13.3 deg, so a noon elevation of about 36.9 deg, near 17:28 UTC (13:28 EDT)
    el, az = sr.sun_position(*INDY, utc(2026, 10, 28, 17, 28))
    assert el == pytest.approx(36.9, abs=1.0)
    assert az == pytest.approx(180., abs=3.)

  def test_noaa_equinox_equator(self):
    el, _ = sr.sun_position(0., 0., utc(2026, 3, 20, 12, 7))
    assert el > 88.

  def test_lighting_rows(self):
    t_day = utc(2026, 10, 28, 19, 0)  # 15:00 EDT, sun in the south-west
    assert sr.lighting(*INDY, t_day, heading=0.) == 'day'          # sun behind
    assert sr.lighting(*INDY, t_day, heading=200.) == 'sunAhead'
    assert sr.lighting(*INDY, utc(2026, 10, 29, 3, 0), heading=0.) == 'night'
    # sunset about 18:43 EDT (22:43 UTC); civil dusk ends about 19:10 EDT
    assert sr.lighting(*INDY, utc(2026, 10, 28, 22, 55), heading=0.) == 'dusk'

  def test_low_sun_along_an_approach(self):
    t = utc(2026, 10, 28, 21, 30)  # 17:30 EDT: sun low in the south-west
    el, az = sr.sun_position(*INDY, t)
    assert 0. < el < 15.
    assert sr.lighting(*INDY, t, heading=20., look_bearings=[az + 10.]) == 'lowSun'
    assert sr.lighting(*INDY, t, heading=20., look_bearings=[az + 40.]) != 'lowSun'

  def test_eta_window(self):
    rows, why = sr.light_window(*INDY, utc(2026, 10, 28, 19, 0), heading=0., veil='unmeasured')
    assert why == '' and rows == {'day'}
    _, why = sr.light_window(*INDY, utc(2026, 10, 28, 22, 30), heading=0., veil='unmeasured')
    assert 'dusk' in why  # 18:30 EDT: the window's late end (18:50) reaches dusk
    _, why = sr.light_window(*INDY, utc(2026, 10, 28, 19, 0), heading=200., veil='unmeasured')
    assert 'sun ahead' in why
    rows, why = sr.light_window(*INDY, utc(2026, 10, 28, 19, 0), heading=200., veil='none')
    assert why == '' and rows == {'sunAhead'}


class TestConverters:
  def test_from_navd_control(self):
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    from openpilot.selfdrive.turnpilot.routing.junctions import Approach, Junction
    info = Junction(osm_node_id=77, lat=39.8, lon=-86.15, stop_point_distance=13., heading_in=90., maneuver='left', turn_angle=-90.,
                    rail_distance=-1., sidecar=True, surveyed=True, our_way=5,
                    approaches=[Approach(0., -90., 'exit', 'residential', 'road', True, True, 1, 1, 'stop', 'all', 'surveyed', 0., 8.9,
                                         'survey', 812)])
    j = sr.junction_from_control(Control(1, 'stop', 60., 0, 60., junction_info=info))
    assert (j.kind, j.osm_node_id, j.maneuver, j.stop_point_distance, j.surveyed, j.our_way) == ('stop', 77, 'left', 13., True, 5)
    a = j.approaches[0]
    assert (a.role, a.rel_bearing, a.tier, a.control, a.assumed_limit, a.way_id, a.lanes_in) == ('exit', -90., 'surveyed', 'stop', 8.9, 812, 1)

  def test_a_control_without_its_junction_is_failed(self):
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    assert sr.junction_from_control(Control(1, 'stop', 60., 0, 60.)).status == 'failed'

  def test_from_capnp(self):
    import openpilot.cereal.messaging as messaging
    msg = messaging.new_message('navInstruction')
    msg.navInstruction.controls = [{'id': 3, 'kind': 'yield', 'distance': 40., 'junctionInfo': {
      'status': 'ok', 'osmNodeId': 9, 'lat': 1., 'lon': 2., 'stopPointDistance': 13., 'headingIn': 45., 'maneuver': 'right',
      'turnAngle': 90., 'railDistance': -1., 'surveyed': False, 'approaches': [
        {'bearing': 135., 'relBearing': 90., 'role': 'exit', 'roadClass': 'tertiary', 'use': 'road', 'inbound': True, 'outbound': True,
         'lanesIn': 2, 'lanesOut': 1, 'valhallaControl': 'none', 'osmStop': 'none', 'tier': 'no', 'assumedLimit': 15.6, 'wayId': 4}]}}]
    reader = msg.navInstruction.as_reader().controls[0]
    j = sr.junction_from_capnp(reader)
    assert (j.kind, j.osm_node_id, j.maneuver, j.rail_distance, j.heading_in) == ('yield', 9, 'right', -1., 45.)
    a = j.approaches[0]
    assert (a.role, a.road_class, a.lanes_in, a.control, a.tier, a.way_id) == ('exit', 'tertiary', 2, 'none', 'no', 4)
    assert a.assumed_limit == pytest.approx(15.6)

  def test_class_speed_defaults(self):
    assert sr.class_limit_mph('residential') == 30 and sr.class_limit_mph('tertiary') == 35
    assert sr.class_limit_mph('secondary') == 45 and sr.class_limit_mph('primary') == 55 and sr.class_limit_mph('motorway') == 55
    assert sr.class_limit_mph('unclassified') == 30 and sr.class_limit_mph('service_other') == 30
