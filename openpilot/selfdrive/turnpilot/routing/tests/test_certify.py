"""The route-time certifier (routing/certify.py; STOP_SIGNS.md 3.2): one test per ordered check, and the ce test of each
in-scope catalogue row, named for it. The junction is made up: a 4-way all-way stop in Indianapolis, the car arriving
northbound, every approach surveyed (25 mph posted, the near travel-lane edge 3.5 m from the node, open corners), the stop
point at node - 13 m and a 1.9 m camera setback."""
import copy
import json
from dataclasses import replace
from datetime import UTC, datetime

import pytest

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot import visibility_table as vt
from openpilot.selfdrive.turnpilot.routing import certify as ce

MPH = 0.44704
LAT, LON = 39.7684, -86.1581
NODE = 4242


def utc(*a):
  return datetime(*a, tzinfo=UTC).timestamp()


ETA = utc(2026, 10, 28, 19, 0)  # 15:00 EDT: daylight, the sun in the south-west, behind a northbound car


def visibility(tmp_path, **changes):
  raw = json.loads(vt.SHIPPED.read_text())
  raw.update(cameraSetbackM=1.9, lateralOffsetM=0.0, heightM=1.3, darkThreshold=45, veil='unmeasured', actuatorDelayS=None,
             lens={'model': 'equisolid', 'f': 452., 'k': 0.3, 'edgeDeg': {'left': 88.1, 'right': 88.1}},
             mask={'left': [[b, -30., 30.] for b in range(89)], 'right': [[b, -30., 30.] for b in range(89)]},
             launchVerified={'date': '2026-10-30', 'goes': 10, 'takenOver': 10, 'maxTakeoverS': 1.1, 'verdict': 'pass'})
  raw.update(changes)
  p = tmp_path / 'visibility.json'
  p.write_text(json.dumps(raw))
  return vt.load(p)


def stamp(m=48., lighting='day', dropout=0.4, lanes=1, **kw):
  out = {'lighting': lighting, 'date': '2026-10-27', 'passes': 3, 'approachBearingDeg': 0., 'cars': {'left': 12, 'right': 12, 'opposite': 10},
         'minFirstTrackM': {'left': [m] * lanes, 'right': [m] * lanes, 'opposite': [80.] * lanes},
         'maxDropoutS': {'left': [dropout] * lanes, 'right': [dropout] * lanes, 'opposite': [0.3] * lanes}, 'allStopped': True,
         'verdict': 'pass'}
  out.update(kw)
  return out


def survey(mph=25, parking=None, sight=100., validations=None, lane_edge=3.5):
  def appr(way, toward):
    p = (parking or {}).get(way, {'left': 60., 'right': 60.})
    return {'wayId': way, 'bearingDeg': toward, 'stops': True, 'plaque': True, 'postedMph': mph, 'laneEdgeM': lane_edge, 'lineM': 7.,
            'nearestParkingM': p, 'sightM': sight if not isinstance(sight, dict) else sight.get(way, 100.), 'stopPointM': None}
  return {'osmNodeId': NODE, 'lat': LAT, 'lon': LON, 'date': '2026-10-27', 'source': 'inPerson', 'confirmedBy': ['inPerson'],
          'approaches': [appr(10, 0.), appr(11, 90.), appr(12, 270.), appr(13, 180.)],
          'validations': [stamp()] if validations is None else validations}


def approach(role, rel, bearing, way, tier='surveyed', mph=25, lanes=1, control='stop', exit_=False, **kw):
  return sr.Approach(bearing=bearing, rel_bearing=rel, role='exit' if exit_ else role, road_class='residential', use='road', inbound=True,
                     outbound=True, lanes_in=lanes, lanes_out=lanes, control=control, tier=tier, assumed_limit=mph * MPH, way_id=way, **kw)


def junction(mph=25, maneuver='straight', tiers=None, **kw):
  t = tiers or {}
  out = sr.Junction(kind='stop', status='ok', osm_node_id=NODE, lat=LAT, lon=LON, stop_point_distance=13., heading_in=0., maneuver=maneuver,
                    rail_distance=-1., sidecar=True, surveyed=True, our_way=10,
                    approaches=[approach('left', -90., 270., 11, t.get('left', 'surveyed'), mph, exit_=maneuver == 'left'),
                                approach('right', 90., 90., 12, t.get('right', 'surveyed'), mph, exit_=maneuver == 'right'),
                                approach('opposite', 0., 0., 13, t.get('opposite', 'surveyed'), mph, exit_=maneuver == 'straight')])
  return replace(out, **kw)


RUN = ce.Runtime(mode='auto', side_views=True)


def check(tmp_path, j=None, s=None, eta=ETA, runtime=RUN, maneuver=None, vis=None, **vis_changes):
  j = j or junction()
  v = vis if vis is not None else visibility(tmp_path, **vis_changes)
  return ce.certify(j, maneuver or j.maneuver, v, [s or survey()], eta, runtime)


class TestOrderedChecks:
  def test_a_surveyed_validated_25_mph_all_way_certifies(self, tmp_path):
    assert check(tmp_path) == ('certifiable', 'none')

  @pytest.mark.parametrize('maneuver', ['straight', 'left', 'right'])
  def test_10_a_stamp_without_the_opposite_side_validates_no_maneuver(self, tmp_path, maneuver):
    # junction_validate.py leaves the opposite approach out of every stamp until step S4's taped ranges bound its ground
    # range (STAMP_SIDES): every maneuver conflicts with the opposite approach, so none certifies on such a stamp
    st = stamp()
    for k in ('cars', 'minFirstTrackM', 'maxDropoutS'):
      del st[k]['opposite']
    v = ce.assess(junction(maneuver=maneuver), maneuver, visibility(tmp_path), [survey(validations=[st])], ETA, RUN)
    assert (v.certification, v.reason) == ('uncertifiable', 'notValidated') and 'opposite lane 1' in v.detail

  def test_1_no_visibility_file_is_no_table(self, tmp_path):
    assert check(tmp_path, vis=False) == ('uncertifiable', 'noTable')
    assert ce.certify(junction(), 'straight', None, [survey()], ETA, RUN) == ('uncertifiable', 'noTable')

  def test_O11_the_shipped_file_without_camera_measurements_is_no_table(self, tmp_path):
    assert ce.certify(junction(), 'straight', vt.load(vt.SHIPPED), [survey()], ETA, RUN) == ('uncertifiable', 'noTable')
    assert check(tmp_path, veil=None) == ('uncertifiable', 'noTable')

  @pytest.mark.parametrize('mode', ['off', 'shadow', 'nonsense'])
  def test_2_O10_mode_off_or_shadow(self, tmp_path, mode):
    assert check(tmp_path, runtime=ce.Runtime(mode=mode, side_views=True)) == ('uncertifiable', 'modeOff')

  def test_3_O14_no_launch_verified_except_under_trial(self, tmp_path):
    assert check(tmp_path, launchVerified=None) == ('uncertifiable', 'noLaunch')
    assert check(tmp_path, launchVerified=None, runtime=ce.Runtime(mode='trial', side_views=True)) == ('certifiable', 'none')
    assert check(tmp_path, runtime=ce.Runtime(mode='auto', side_views=True, launch_verified=False)) == ('uncertifiable', 'noLaunch')

  def test_3_a_failing_launch_stamp_is_no_launch(self, tmp_path):
    failed = {'date': '2026-10-30', 'goes': 10, 'takenOver': 3, 'maxTakeoverS': 4.0, 'verdict': 'fail'}
    assert check(tmp_path, launchVerified=failed) == ('uncertifiable', 'noLaunch')
    assert check(tmp_path, launchVerified=failed, runtime=ce.Runtime(mode='trial', side_views=True)) == ('certifiable', 'none')

  @pytest.mark.parametrize('changes', [{'roundabout': True}, {'divided': True}, {'rail_distance': 12.}, {'rail_distance': -2.},
                                       {'maneuver': 'uTurn'}])
  def test_4_geometry(self, tmp_path, changes):
    j = junction(**changes)
    assert check(tmp_path, j, maneuver=j.maneuver) == ('uncertifiable', 'geometry')

  def test_5_skew(self, tmp_path):
    j = junction()
    j.approaches[0].rel_bearing = -110.
    assert check(tmp_path, j) == ('uncertifiable', 'skew')

  def test_6_unknown_leg_and_a_failed_junction(self, tmp_path):
    j = junction()
    j.approaches[1].control = 'unknown'
    assert check(tmp_path, j) == ('uncertifiable', 'unknownLeg')
    assert check(tmp_path, junction(status='failed')) == ('uncertifiable', 'unknownLeg')

  def test_7_not_surveyed(self, tmp_path):
    assert check(tmp_path, junction(surveyed=False)) == ('uncertifiable', 'notSurveyed')
    assert check(tmp_path, junction(tiers={'right': 'mapStrong'})) == ('uncertifiable', 'notSurveyed')

  def test_8_N6_an_overridden_approach(self, tmp_path):
    j = junction(tiers={'right': 'no'})
    j.approaches[1].overridden = True
    assert check(tmp_path, j) == ('uncertifiable', 'override')

  def test_9_E2_light_at_dusk_and_night(self, tmp_path):
    assert check(tmp_path, eta=utc(2026, 10, 29, 2, 0)) == ('uncertifiable', 'light')
    assert check(tmp_path, eta=utc(2026, 10, 28, 22, 30)) == ('uncertifiable', 'light')  # 18:30 EDT: the window's end reaches dusk

  def test_9_light_without_a_valid_clock(self, tmp_path):
    assert check(tmp_path, eta=None) == ('uncertifiable', 'light')

  def test_10_not_validated_without_a_stamp(self, tmp_path):
    assert check(tmp_path, s=survey(validations=[])) == ('uncertifiable', 'notValidated')

  def test_11_reach(self, tmp_path):
    assert check(tmp_path, s=survey(validations=[stamp(40.)])) == ('uncertifiable', 'reach')

  def test_the_first_failure_wins(self, tmp_path):
    j = junction(roundabout=True, surveyed=False)
    assert check(tmp_path, j, launchVerified=None) == ('uncertifiable', 'noLaunch')
    assert check(tmp_path, j) == ('uncertifiable', 'geometry')


class TestReachAndValidation:
  def test_40_m_stamp_fails_at_25_mph_and_certifies_at_20(self, tmp_path):
    assert check(tmp_path, s=survey(validations=[stamp(40.)])) == ('uncertifiable', 'reach')
    assert check(tmp_path, junction(mph=20), survey(mph=20, validations=[stamp(40.)])) == ('certifiable', 'none')

  def test_45_m_stamp_fails_at_25_mph_which_needs_45_4(self, tmp_path):
    # the computed task's 45 m stamp predates T_LAT 1.0 s (STOP_SIGNS.md "Changes after measurement")
    assert check(tmp_path, s=survey(validations=[stamp(45.)])) == ('uncertifiable', 'reach')
    assert check(tmp_path, s=survey(validations=[stamp(45.5)])) == ('certifiable', 'none')

  def test_30_m_stamp_fails_at_20_mph_and_certifies_at_15(self, tmp_path):
    assert check(tmp_path, junction(mph=20), survey(mph=20, validations=[stamp(30.)])) == ('uncertifiable', 'reach')
    assert check(tmp_path, junction(mph=15), survey(mph=15, validations=[stamp(30.)])) == ('certifiable', 'none')

  def test_a_dropout_over_clear_s_over_3_fails_validation(self, tmp_path):
    assert check(tmp_path, s=survey(validations=[stamp(dropout=0.7)])) == ('uncertifiable', 'notValidated')
    assert check(tmp_path, s=survey(validations=[stamp(dropout=0.66)])) == ('certifiable', 'none')

  def test_a_stamp_that_did_not_pass_or_saw_a_car_through_does_not_validate(self, tmp_path):
    assert check(tmp_path, s=survey(validations=[stamp(verdict='fail')])) == ('uncertifiable', 'notValidated')
    assert check(tmp_path, s=survey(validations=[stamp(allStopped=False)])) == ('uncertifiable', 'notValidated')

  def test_a_stamp_for_another_approach_does_not_validate_ours(self, tmp_path):
    assert check(tmp_path, s=survey(validations=[stamp(approachBearingDeg=180.)])) == ('uncertifiable', 'notValidated')
    assert check(tmp_path, s=survey(validations=[stamp(approachBearingDeg=0.)])) == ('certifiable', 'none')

  def test_a_stamp_without_its_approach_validates_no_approach(self, tmp_path):
    # its reaches were measured from some approach's stop point and sightlines; without approachBearingDeg nobody can say whose
    st = stamp()
    del st['approachBearingDeg']
    v = ce.assess(junction(), 'straight', visibility(tmp_path), [survey(validations=[st])], ETA, RUN)
    assert (v.certification, v.reason) == ('uncertifiable', 'notValidated')
    assert 'without approachBearingDeg' in v.detail, v.detail
    # arriving from the west (heading 90), where the stamp's left, right and opposite reaches aren't ours
    j = junction(heading_in=90., our_way=12)
    assert check(tmp_path, j, survey(validations=[st])) == ('uncertifiable', 'notValidated')
    for bad in (None, 'north', True):
      assert check(tmp_path, s=survey(validations=[stamp(approachBearingDeg=bad)])) == ('uncertifiable', 'notValidated')

  def test_a_stamp_missing_a_binding_lane_does_not_validate(self, tmp_path):
    st = stamp()
    del st['minFirstTrackM']['right']
    assert check(tmp_path, s=survey(validations=[st])) == ('uncertifiable', 'notValidated')

  def test_G3_parking_from_21_m_on_a_binding_curb_fails(self, tmp_path):
    # the left approach's right-hand curb is the near side: parked cars there hide the near lane
    assert check(tmp_path, s=survey(parking={11: {'left': 60., 'right': 21.}})) == ('uncertifiable', 'reach')
    # its other curb is beyond both lanes: it binds nothing
    assert check(tmp_path, s=survey(parking={11: {'left': 21., 'right': 60.}})) == ('certifiable', 'none')
    assert check(tmp_path, s=survey(parking={12: {'left': 21., 'right': 60.}})) == ('uncertifiable', 'reach')

  def test_G2_a_hedge_the_survey_records_caps_the_lane(self, tmp_path):
    assert check(tmp_path, s=survey(sight={11: 30.})) == ('uncertifiable', 'reach')

  def test_G8_multi_lane_cross_road_each_lane_separately(self, tmp_path):
    j = junction()
    for a in j.approaches[:2]:
      a.lanes_in = a.lanes_out = 2
    s = survey(validations=[stamp(lanes=2)])
    assert check(tmp_path, j, s) == ('certifiable', 'none')
    st = stamp(lanes=2)
    st['minFirstTrackM']['right'] = [48., 40.]
    assert check(tmp_path, j, survey(validations=[st])) == ('uncertifiable', 'reach')

  def test_E9_side_views_off_nothing_certifies(self, tmp_path):
    assert check(tmp_path, runtime=ce.Runtime(mode='auto', side_views=False)) == ('uncertifiable', 'reach')
    assert check(tmp_path, junction(mph=10), survey(mph=10), runtime=ce.Runtime(mode='auto', side_views=False)) == ('uncertifiable', 'reach')

  def test_N4_an_all_way_whose_limit_exceeds_its_validation(self, tmp_path):
    assert check(tmp_path, junction(mph=30), survey(mph=30)) == ('uncertifiable', 'reach')

  def test_the_right_swing_zone_counts_from_15_m(self, tmp_path):
    # a right's swing zone puts the far lane's conflict point 15 m out: 48 - 15 = 33 m certifies 20 mph, not 25
    j = junction(maneuver='right')
    assert check(tmp_path, j, maneuver='right') == ('uncertifiable', 'reach')
    assert check(tmp_path, junction(mph=20, maneuver='right'), survey(mph=20), maneuver='right') == ('certifiable', 'none')


class TestLight:
  def test_E1_sun_ahead_with_the_veil_unmeasured(self, tmp_path):
    j = junction(heading_in=200.)
    assert check(tmp_path, j) == ('uncertifiable', 'light')

  def test_sun_ahead_with_no_veil_needs_its_own_stamp(self, tmp_path):
    j = junction(heading_in=200.)
    assert check(tmp_path, j, veil='none') == ('uncertifiable', 'notValidated')
    st = stamp(lighting='sunAhead', approachBearingDeg=200.)
    assert ce.certify(j, 'straight', visibility(tmp_path, veil='none'), [survey(validations=[st])], ETA, RUN)[1] != 'notValidated'

  def test_eta_window_at_15_00_certifies_and_fails_when_its_late_end_turns_the_sun_ahead(self, tmp_path):
    assert check(tmp_path) == ('certifiable', 'none')
    early = sr.sun_position(LAT, LON, ETA - sr.ETA_EARLY_S)[1]
    j = junction(heading_in=(early + 90.5) % 360.)  # behind the 90 deg line at the window's start, ahead of it by its end
    assert sr.lighting(LAT, LON, ETA - sr.ETA_EARLY_S, j.heading_in) == 'day'
    assert sr.lighting(LAT, LON, ETA + sr.ETA_LATE_S, j.heading_in) == 'sunAhead'
    assert check(tmp_path, j) == ('uncertifiable', 'light')
    # at the line the window is now
    assert check(tmp_path, j, eta=ETA - sr.ETA_EARLY_S, runtime=ce.Runtime(mode='auto', side_views=True, window=(0., 0.)))[1] != 'light'

  def test_eta_window_late_end_at_dusk(self, tmp_path):
    # eastbound, the legs north, south and east: no approach looks into the low western sun, which is behind the car
    j = junction(heading_in=90.)
    for a, b in zip(j.approaches, (0., 180., 90.), strict=True):
      a.bearing = b
    eta = utc(2026, 10, 28, 22, 30)  # 18:30 EDT, sunset about 18:44: the window's late end (18:50) is dusk
    v = ce.assess(j, 'straight', visibility(tmp_path), [survey()], eta, RUN)
    assert (v.reason, 'dusk' in v.detail) == ('light', True), v.detail
    at_line = ce.Runtime(mode='auto', side_views=True, window=(0., 0.))
    assert ce.assess(j, 'straight', visibility(tmp_path), [survey()], eta, at_line).reason != 'light'

  def test_low_sun_along_an_approach_whatever_the_veil(self, tmp_path):
    # 18:30 EDT: the sun about 2 deg up in the west-south-west, along the west leg (the left approach) of a northbound car
    v = ce.assess(junction(), 'straight', visibility(tmp_path, veil='none'), [survey()], utc(2026, 10, 28, 22, 30), RUN)
    assert (v.reason, 'low sun' in v.detail) == ('light', True), v.detail


class TestCatalogue:
  def test_C2_two_way_stop(self, tmp_path):
    assert check(tmp_path, junction(mph=15, tiers={'left': 'no', 'right': 'no'})) == ('uncertifiable', 'notSurveyed')

  def test_C3_G24_a_right_at_a_3_way_stop_whose_right_approach_does_not_stop(self, tmp_path):
    j = junction(maneuver='right', tiers={'right': 'no'})
    assert check(tmp_path, j, maneuver='right') == ('uncertifiable', 'notSurveyed')
    # the union with the straight set until 15 deg of yaw is what catches it: the right's own set touches only the swing zone
    assert {(m.role, m.movement) for m in sr.conflict_movements(j, 'right')} >= {('right', 'cross')}

  def test_C4_t_junction_on_the_stem(self, tmp_path):
    j = junction(maneuver='left', mph=20)
    j.approaches = j.approaches[:2]
    assert check(tmp_path, j, survey(mph=20), maneuver='left') == ('certifiable', 'none')
    j.approaches[1].control = 'unknown'
    assert check(tmp_path, j, survey(mph=20), maneuver='left') == ('uncertifiable', 'unknownLeg')

  def test_C5_yield(self, tmp_path):
    assert check(tmp_path, junction(kind='yield', tiers={'left': 'no', 'right': 'no', 'opposite': 'no'})) == ('uncertifiable', 'notSurveyed')

  def test_C6_slip_lane_and_roundabout(self, tmp_path):
    j = junction(maneuver='right')
    j.approaches[1].use = 'turnChannel'
    assert check(tmp_path, j, maneuver='right') == ('uncertifiable', 'geometry')
    assert check(tmp_path, junction(roundabout=True)) == ('uncertifiable', 'geometry')

  def test_C7_rail_crossing(self, tmp_path):
    assert check(tmp_path, junction(rail_distance=29.)) == ('uncertifiable', 'geometry')
    assert check(tmp_path, junction(rail_distance=-1.)) == ('certifiable', 'none')

  def test_C8_driveway_exit_onto_a_public_road(self, tmp_path):
    assert check(tmp_path, junction(tiers={'left': 'no', 'right': 'no', 'opposite': 'unknown'})) == ('uncertifiable', 'notSurveyed')

  def test_C9_two_way_mapped_as_all_way(self, tmp_path):
    assert check(tmp_path, junction(surveyed=False, tiers={'left': 'mapStrong', 'right': 'mapStrong', 'opposite': 'mapStrong'})) == \
      ('uncertifiable', 'notSurveyed')

  def test_C10_mapped_two_way_that_is_really_all_way(self, tmp_path):
    assert check(tmp_path, junction(surveyed=False, tiers={'left': 'no', 'right': 'no'})) == ('uncertifiable', 'notSurveyed')

  def test_C12_N5_suspected_unmapped_stop(self, tmp_path):
    assert check(tmp_path, junction(surveyed=False, suspected=True, tiers={'left': 'unknown', 'right': 'unknown', 'opposite': 'unknown'})) == \
      ('uncertifiable', 'notSurveyed')

  def test_C21_unsurveyed_map_tagged_all_way_20_mph(self, tmp_path):
    j = junction(mph=20, surveyed=False, tiers={'left': 'mapStrong', 'right': 'mapStrong', 'opposite': 'mapStrong'})
    assert check(tmp_path, j, survey(mph=20)) == ('uncertifiable', 'notSurveyed')

  def test_G6_skewed_approach_behind(self, tmp_path):
    j = junction()
    j.approaches[1].rel_bearing = 112.
    assert check(tmp_path, j) == ('uncertifiable', 'skew')

  def test_G7_skewed_approach_in_front(self, tmp_path):
    j = junction()
    j.approaches[0].rel_bearing = -70.
    assert check(tmp_path, j) == ('uncertifiable', 'skew')

  @pytest.mark.parametrize('maneuver', ['straight', 'left', 'right'])
  @pytest.mark.parametrize('skew', [40., 30., 20.])
  def test_G6_G7_a_cross_road_skewed_into_the_opposite_and_other_bins(self, tmp_path, skew, maneuver):
    # a skewed X: the cross road's legs at +skew (read opposite) and skew - 180 (read other), every one surveyed stopping and
    # validated. The 75-105 deg rule looked only at the left and right bins: the +skew leg was walked as an opposing lane
    # parallel to ours and the other one's traffic was in no conflict set
    j = junction()
    j.approaches = [approach('opposite', 0., 0., 13, exit_=maneuver == 'straight'), approach('opposite', skew, skew, 12),
                    approach('other', skew - 180., (skew + 180.) % 360., 11)]
    s = survey()
    s['approaches'][1]['bearingDeg'] = skew % 360.            # way 11 toward the node
    s['approaches'][2]['bearingDeg'] = (skew + 180.) % 360.   # way 12
    assert check(tmp_path, j, s, maneuver=maneuver) == ('uncertifiable', 'skew')

  @pytest.mark.parametrize('maneuver', ['straight', 'left', 'right'])
  @pytest.mark.parametrize('rel', [-150., 150.])
  def test_C3_a_fifth_road_behind_the_side_bins(self, tmp_path, rel, maneuver):
    # a 4-way all-way with a fifth road at rel deg, 45 mph, no stop: its traffic is in no conflict set, so nothing checks it
    j = junction(maneuver=maneuver)
    j.approaches.append(approach('other', rel, rel % 360., 14, tier='no', control='none', mph=45))
    assert check(tmp_path, j, maneuver=maneuver) == ('uncertifiable', 'skew')
    # cars only leaving by it: still a road the reach model doesn't have
    j.approaches[-1].inbound = False
    assert check(tmp_path, j, maneuver=maneuver) == ('uncertifiable', 'skew')
    # a footway or path there is no road
    j.approaches[-1].outbound = False
    assert check(tmp_path, j, maneuver=maneuver) == ('certifiable' if maneuver != 'right' else 'uncertifiable',
                                                     'none' if maneuver != 'right' else 'reach')

  def test_an_opposite_road_more_than_15_deg_off_straight_on(self, tmp_path):
    j = junction()
    j.approaches[2].rel_bearing = 20.
    assert check(tmp_path, j) == ('uncertifiable', 'skew')
    j.approaches[2].rel_bearing = -sr.OPPOSITE_SKEW_DEG
    assert check(tmp_path, j) == ('certifiable', 'none')

  @pytest.mark.parametrize('role, rel', [('right', 100.), ('left', -80.), ('opposite', 5.)])
  def test_a_second_road_in_one_role(self, tmp_path, role, rel):
    # each role is one road to the reach model: a second would be walked as the first
    j = junction()
    j.approaches.append(approach(role, rel, rel % 360., 14))
    v = ce.assess(j, 'straight', visibility(tmp_path), [survey()], ETA, RUN)
    assert (v.certification, v.reason, f'two {role} roads' in v.detail) == ('uncertifiable', 'skew', True), v.detail

  def test_G11_G12_divided_cross_road(self, tmp_path):
    assert check(tmp_path, junction(divided=True)) == ('uncertifiable', 'geometry')

  def test_G16_staggered_neighbour_is_not_matched_by_radius(self, tmp_path):
    # the neighbour 20 m on has its own node: navd matched no survey to it (by node id, never a radius), so it is unsurveyed
    j = junction(osm_node_id=NODE + 1, lat=LAT + 20. / 111195., surveyed=False)
    assert check(tmp_path, j) == ('uncertifiable', 'notSurveyed')

  def test_G23_exit_leg_unmatched(self, tmp_path):
    j = junction()
    j.approaches[2].control = 'unknown'
    assert check(tmp_path, j) == ('uncertifiable', 'unknownLeg')

  def test_N13_stale_or_implausible_survey(self, tmp_path):
    stale = copy.deepcopy(survey())
    stale['date'] = '2025-01-01'
    assert check(tmp_path, junction(surveyed=False), stale) == ('uncertifiable', 'notSurveyed')


class TestAssess:
  def test_assess_says_why_and_the_reach_per_lane(self, tmp_path):
    v = ce.assess(junction(), 'straight', visibility(tmp_path), [survey(validations=[stamp(40.)])], ETA, RUN)
    assert (v.certification, v.reason) == ('uncertifiable', 'reach')
    assert 'right' in v.detail
    near = next(lane for lane in v.lanes if lane.role == 'left')
    assert near.reach == pytest.approx(40., abs=0.3) and near.need == pytest.approx(45.4, abs=0.05)
    assert v.rows == {'day'} and v.validated

  def test_route_time_views_are_all_fresh_so_the_stamp_bounds_each_lane(self, tmp_path):
    # the walk keeps the views' coverage under a validated entry; at route time every view counts as fresh
    v = ce.assess(junction(), 'straight', visibility(tmp_path), [survey()], ETA, RUN)
    assert v.certification == 'certifiable'
    assert {(lane.role, round(lane.reach)) for lane in v.lanes} == {('left', 48), ('right', 48), ('opposite', 80)}

  def test_certify_control_fills_navd_junction(self, tmp_path):
    from openpilot.selfdrive.turnpilot.routing.controls import Control
    from openpilot.selfdrive.turnpilot.routing.junctions import Approach, Junction
    info = Junction(osm_node_id=NODE, lat=LAT, lon=LON, stop_point_distance=13., rail_distance=-1., surveyed=False)
    c = Control(1, 'stop', 60., 0, 60., junction_info=info)
    # no approaches read: nothing to judge cross traffic by
    assert ce.certify_control(c, visibility(tmp_path), [survey()], ETA, RUN) == ('uncertifiable', 'unknownLeg')
    assert (info.certification, info.cert_reason, info.validated) == ('uncertifiable', 'unknownLeg', False)
    info.approaches = [Approach(270., -90., 'left', 'residential', 'road', True, True, 1, 1, 'stop', 'stop', 'mapStrong', 0., 11.2,
                                'tagged', 11)]
    assert ce.certify_control(c, visibility(tmp_path), [survey()], ETA, RUN) == ('uncertifiable', 'notSurveyed')
    assert (info.certification, info.cert_reason, info.validated) == ('uncertifiable', 'notSurveyed', False)
    assert 'no counting survey' in info.cert_detail
    assert ce.certify_control(Control(2, 'signal', 80., 0, 80.), visibility(tmp_path), [], ETA, RUN) == ('unchecked', 'none')


class TestModeParam:
  def test_unknown_or_unreadable_acts_as_shadow_said_once(self):
    from types import SimpleNamespace
    from unittest.mock import patch

    from openpilot.common.params import UnknownKeyName

    def unknown(key):
      raise UnknownKeyName(key)
    ce._said.clear()
    with patch.object(ce.cloudlog, 'warning') as warn:
      for _ in range(3):  # navd asks every second
        assert ce.stop_sign_mode(SimpleNamespace(get=unknown)) == 'shadow'
        assert ce.stop_sign_mode(SimpleNamespace(get=lambda k: 'sometimes')) == 'shadow'
    assert warn.call_count == 2
    assert ce.stop_sign_mode(SimpleNamespace(get=lambda k: None)) == 'shadow'
    assert [ce.stop_sign_mode(SimpleNamespace(get=lambda k, m=m: m)) for m in ('off', 'trial', 'auto')] == ['off', 'trial', 'auto']
    assert ce.stop_sign_mode(SimpleNamespace(get=lambda k: b'auto')) == 'auto'
