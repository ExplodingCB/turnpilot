"""route_audit.py: every stop and yield approach on a route, with Valhalla's flag, the OSM sidecar's, the line source and
the OSM node id (STOP_SIGNS.md SG1a, step S2), on the stub Valhalla of routing/tests/test_route_controls.py."""
import json

import pytest

from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import LON0, M_PER_DEG, StubValhalla, entry, osm_dir, pt, steps
from openpilot.tools.turnpilot import route_audit


def test_every_stop_and_yield_approach_with_both_sources(tmp_path):
  v = StubValhalla([(0., 100., {'stop': True, 'way_id': 71}), (100., 200., {'way_id': 72}), (200., 250., {'node_signal': True}),
                    (250., 290., {}), (290., 300., {'stop': True, 'way_id': 74, 'forward': False}), (300., 400., {})])
  side = {'71:f': [entry('stop', 82., junction=901, stop_tag='minor')],
          '72:f': [entry('yield', 185., junction=902, evidence='mid_undirected')],
          '74:f': [entry('stop', 300., junction=904)]}  # directed the other way: 74:b's flag is Valhalla's phantom
  report = route_audit.audit(v, steps(), None, osm_dir(tmp_path, side))
  assert report.status == 'ok' and report.sidecar == 'ok'
  assert [c.kind for c in report.controls] == ['stop', 'yield', 'signal']  # the signal is a control, not a stop or yield row
  rows = [(r.key, r.valhalla, r.sidecar, r.line_source, r.sign_m, r.osm_node, r.control) for r in report.rows]
  assert rows == [  # the OSM sign is shown where it is; the stop point stays the default (SG1a moves none)
    ('71:f', 'stop', 'stop (mid_directed, stop=minor)', 'default', 82., 901, 'stop at 89 m'),
    ('72:f', '-', 'yield (mid_undirected, stop=none)', 'default', 185., 902, 'yield at 189 m'),
    ('74:b', 'stop (phantom: dropped)', '-', '-', None, 0, '-'),
  ]
  text = '\n'.join(route_audit.lines(report))
  for word in ('71:f', 'Valhalla', 'sidecar', 'line', 'sign m', 'OSM node', '901', 'phantom', 'yield (mid_undirected'):
    assert word in text
  assert json.loads(json.dumps(route_audit.as_json(report)))['rows'][0]['osm_node'] == 901


def test_a_missing_sidecar_is_said_at_the_top(tmp_path):
  v = StubValhalla([(0., 100., {'stop': True}), (100., 400., {})])
  report = route_audit.audit(v, steps(), None, osm_dir(tmp_path, shards=False))
  assert report.status == 'partial' and report.sidecar.startswith('missing')
  assert 'OSM sidecar: missing' in route_audit.lines(report)[0]
  assert [(r.valhalla, r.sidecar, r.control) for r in report.rows] == [('stop', 'not checked', 'stop at 89 m')]


def test_b3_osm_signs_near_the_route_that_no_row_covers_are_listed(tmp_path):
  """Step S2 is the gate before any real drive on a stop-sign route, so the audit can't list only what a source placed:
  every OSM stop or give_way node within RAW_M of the route that no row covers is listed, with who the sidecar holds it
  for (the other direction of our way: right, or a reversed direction tag; another way; no approach at all). One off
  the route, one the sidecar placed on its edge, and one on an edge Valhalla flags with its kind are not."""
  v = StubValhalla([(0., 100., {'way_id': 71}), (100., 200., {'way_id': 72, 'stop': True}), (200., 300., {'way_id': 73}),
                    (300., 400., {})])
  side = {'71:b': [entry('stop', 60., junction=900, node=501)],                 # the other direction's, on our unflagged edge
          '72:f': [entry('stop', 180., junction=901, node=502)],                # placed: its edge's row has it
          '72:b': [entry('stop', 120., junction=902, node=503)],                # on an edge Valhalla flags stop: its row covers it
          '81:f': [entry('stop', 250., junction=903, east=3., node=504)],       # a cross road's, 3 m off the route
          '82:f': [entry('stop', 260., junction=904, east=20., node=505)]}      # 20 m off: not near
  beside = {'node': 506, 'kind': 'stop', 'lat': pt(230.)[0], 'lon': LON0 + 2. / (M_PER_DEG * 0.7687), 'direction': 'forward',
            'stop_tag': 'none', 'why': 'not_on_a_drivable_way'}
  report = route_audit.audit(v, steps(), None, osm_dir(tmp_path, side, unplaced=[beside]))
  assert [(g.route_m, g.off_m, g.node, g.kind, g.holder, g.edge, g.to_node_m) for g in report.signs] == [
    (60., 0., 501, 'stop', 'other direction', '71:f', 40.),
    (230., 2., 506, 'stop', 'no approach (not_on_a_drivable_way)', '73:f', 70.),
    (250., 3., 504, 'stop', 'way 81:f', '73:f', 50.),
  ]
  assert [c.kind for c in report.controls] == ['stop']  # the car stops for none of the listed
  text = '\n'.join(route_audit.lines(report))
  assert 'no row covers' in text and '501' in text and 'not_on_a_drivable_way' in text and '505' not in text
  assert [g['node'] for g in route_audit.as_json(report)['signs']] == [501, 506, 504]


def test_each_junction_lists_its_legs_tiers_limits_the_line_source_and_the_survey(tmp_path):
  """STOP_SIGNS.md SG1b: route_audit prints each stop and yield's junction: its legs with their roles, tiers, limits and
  the limit's source, the stop point's source, and the survey it matched (or why one doesn't count)."""
  from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import StubJunction, allway, context, four, survey
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four({90: {'kph': 40}}))
  report = route_audit.audit(v, steps(), None, allway(tmp_path), context([survey(stops=(True, True, True, False))]))
  assert [(j.kind, j.osm_node, j.survey) for j in report.junctions] == [('stop', 900, 'matched, counts')]
  legs = {round(a['bearing']): (a['role'], a['tier'], a['speed_source'], round(a['assumed_mph'])) for a in report.junctions[0].legs}
  assert legs == {0: ('exit', 'surveyed', 'survey', 20), 90: ('right', 'surveyed', 'survey', 20), 270: ('left', 'no', 'survey', 20)}
  text = '\n'.join(route_audit.lines(report))
  for word in ('junction', 'node 900', 'exit', 'right', 'surveyed', 'mph', 'survey', 'line default'):
    assert word in text, word
  doc = json.loads(json.dumps(route_audit.as_json(report)))
  assert doc['junctions'][0]['osm_node'] == 900 and len(doc['junctions'][0]['legs']) == 3
  old = route_audit.audit(v, steps(), None, allway(tmp_path / 'old'), context([survey(date='2025-01-01')]))
  assert old.junctions[0].survey.startswith("matched, doesn't count")


def test_the_survey_template_fills_what_the_map_knows_and_leaves_the_rest_null(tmp_path):
  """route_audit --survey-template: for each candidate all-way on the trip (every other approach mapStrong or better), a
  surveyed_junctions.json entry with the OSM node, its position and per approach the way, the OSM bearing toward the
  junction and the tagged limit as hints; every field Kellen measures is null, so the entry doesn't load until he fills it
  (STOP_SIGNS.md 4.3, step S2). Filled in, it loads."""
  from openpilot.selfdrive.turnpilot.maps import surveys
  from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import StubJunction, allway, four
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four({90: {'kph': 40}}))
  report = route_audit.audit(v, steps(), None, allway(tmp_path))
  template = route_audit.survey_template(report)
  assert len(template) == 1
  entry = template[0]
  assert (entry['osmNodeId'], entry['date'], entry['source']) == (900, None, None)
  assert [(a['wayId'], round(a['bearingDeg']), a['laneEdgeM'], a['stops']) for a in entry['approaches']] == \
    [(50, 0, None, None), (790, 270, None, None), (970, 90, None, None), (700, 180, None, None)]
  assert entry['approaches'][1]['hintTaggedMph'] == round(40 / 3.6 / 0.44704) and entry['approaches'][2]['hintTaggedMph'] is None
  path = tmp_path / 'template.json'
  path.write_text(json.dumps(template))
  assert surveys.load(path).entries == []  # nulls: no survey yet
  entry |= {'date': '2026-10-29', 'source': 'inPerson'}
  for a in entry['approaches']:
    a |= {'stops': True, 'plaque': True, 'postedMph': 20, 'laneEdgeM': 4., 'lineM': 7., 'nearestParkingM': {'left': 40., 'right': 40.},
          'sightM': 60.}
  path.write_text(json.dumps([entry]))
  loaded = surveys.load(path)
  assert loaded.errors == [] and len(loaded.entries) == 1
  two_way = route_audit.audit(v, steps(), None, allway(tmp_path / 'two', (True, 'mid_directed', 'none'),
                                                      {700: (True, 'mid_directed', 'none'), 790: (False, 'none', 'none'),
                                                       970: (True, 'mid_directed', 'none')}))
  assert route_audit.survey_template(two_way) == []  # an approach that doesn't stop: no all-way to survey


def filled_visibility(tmp_path):
  from openpilot.selfdrive.turnpilot import visibility_table as vt
  raw = json.loads(vt.SHIPPED.read_text())
  raw.update(cameraSetbackM=1.9, lateralOffsetM=0.0, heightM=1.3, darkThreshold=45, veil='unmeasured',
             lens={'model': 'equisolid', 'f': 452., 'k': 0.3, 'edgeDeg': {'left': 88.1, 'right': 88.1}},
             mask={'left': [[b, -30., 30.] for b in range(89)], 'right': [[b, -30., 30.] for b in range(89)]},
             launchVerified={'date': '2026-10-30', 'goes': 10, 'takenOver': 10, 'maxTakeoverS': 1.1, 'verdict': 'pass'})
  p = tmp_path / 'visibility.json'
  p.write_text(json.dumps(raw))
  return vt.load(p)


def stub_allway(tmp_path, entries, edges=None):
  from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import StubJunction, allway, context, four
  v = StubJunction(edges or [(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four())
  return v, allway(tmp_path), context(entries)


ETA = 1_793_214_000.  # 2026-10-28 19:00 UTC (15:00 in Indianapolis): daylight, the sun behind a northbound car


def test_each_junction_has_its_certification_and_reason(tmp_path):
  """STOP_SIGNS.md SG2 (5): route_audit prints each control's certification and reason, with why. With the shipped
  visibility file (no camera measurements yet) every stop and yield is noTable, the stop-free floor; with one filled in,
  the surveyed junction has no validation stamp yet."""
  from openpilot.selfdrive.turnpilot.routing.certify import Runtime
  from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import survey
  v, side, ctx = stub_allway(tmp_path, [survey()])
  report = route_audit.audit(v, steps(), None, side, ctx, eta=ETA)
  assert [(j.certification, j.cert_reason) for j in report.junctions] == [('uncertifiable', 'noTable')]
  assert 'noTable' in '\n'.join(route_audit.lines(report))
  report = route_audit.audit(v, steps(), None, side, ctx, visibility=filled_visibility(tmp_path), runtime=Runtime(mode='auto', side_views=True),
                             eta=ETA)
  (j,) = report.junctions
  assert (j.certification, j.cert_reason) == ('uncertifiable', 'notValidated') and 'stamp' in j.cert_detail
  assert route_audit.as_json(report)['junctions'][0]['cert_reason'] == 'notValidated'


def test_predicted_reach_per_lane_from_pv1_at_the_closest_stop_point_and_at_node_13(tmp_path):
  """PV1's conservative and optimistic tables (STOP_SIGNS.md 1.6) give each conflicting lane's predicted reach, capped by
  the survey, at the closest stop point section 1.2 allows (the surveyed edge 4.0 m + 5.8 m) and at node - 13 m: the side
  views run out of pixels, not frame, so standing back loses reach."""
  from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import survey
  v, side, ctx = stub_allway(tmp_path, [survey()])
  (j,) = route_audit.audit(v, steps(), None, side, ctx, eta=ETA).junctions
  points = sorted({p['stop_m'] for p in j.predicted})
  assert points == [9.8, 13.0]
  near = {(p['stop_m'], p['bound']): p for p in j.predicted if p['role'] == 'left' and p['movement'] == 'cross'}
  assert near[9.8, 'conservative']['reach'] >= near[13.0, 'conservative']['reach']
  assert near[13.0, 'conservative']['reach'] < near[13.0, 'optimistic']['reach'] <= 60.  # the survey's 60 m sight distance caps it
  assert near[13.0, 'conservative']['need'] == pytest.approx(32.9, abs=0.05)  # 20 mph posted
  text = '\n'.join(route_audit.lines(route_audit.audit(v, steps(), None, side, ctx, eta=ETA)))
  assert 'predicted reach' in text and 'node - 13.0 m' in text and 'node - 9.8 m' in text


def test_check_survey_validates_the_file_and_prints_each_junctions_predicted_reach(tmp_path, capsys):
  """route_audit --check-survey FILE (step S2): the schema check, and each junction's predicted reach per lane with the
  survey's caps, so a field that kills a junction shows at once (STOP_SIGNS.md 4.3's made-up example: the approach heading
  180 has parking to the corner on one curb)."""
  from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import survey
  example = {'osmNodeId': 0, 'lat': 39.8000, 'lon': -86.1500, 'date': '2026-10-29', 'source': 'streetImagery', 'imageryDate': '2026-06',
             'confirmedBy': [], 'validations': [], 'approaches': [
               {'wayId': 0, 'bearingDeg': 90, 'stops': True, 'plaque': True, 'postedMph': 20, 'laneEdgeM': 4.0, 'lineM': 7.0,
                'nearestParkingM': {'left': 36.0, 'right': 40.0}, 'sightM': 60, 'stopPointM': 9.8},
               {'wayId': 0, 'bearingDeg': 180, 'stops': True, 'plaque': True, 'postedMph': 20, 'laneEdgeM': 4.0, 'lineM': 7.5,
                'nearestParkingM': {'left': 0.0, 'right': 38.0}, 'sightM': 55},
               {'wayId': 0, 'bearingDeg': 270, 'stops': True, 'plaque': True, 'postedMph': 20, 'laneEdgeM': 4.0, 'lineM': 7.0,
                'nearestParkingM': {'left': 35.0, 'right': 37.0}, 'sightM': 70},
               {'wayId': 0, 'bearingDeg': 0, 'stops': True, 'plaque': True, 'postedMph': 20, 'laneEdgeM': 4.0, 'lineM': 7.0,
                'nearestParkingM': {'left': 34.0, 'right': 41.0}, 'sightM': 45}]}
  path = tmp_path / 'surveyed_junctions.json'
  path.write_text(json.dumps([example]))
  assert route_audit.main(['--check-survey', str(path)]) == 0
  out = capsys.readouterr().out
  assert 'schema ok' in out and 'arriving heading 90' in out and '20 mph' in out
  assert 'parking 0.0 m' in out  # the curb parked to the corner binds a lane: shown where it caps it
  bad = dict(survey(), approaches=[dict(survey()['approaches'][0], laneEdgeM=None)])
  path.write_text(json.dumps([example, bad]))
  assert route_audit.main(['--check-survey', str(path)]) == 1
  assert 'laneEdgeM' in capsys.readouterr().out


def test_overrides_are_listed_with_their_evidence_and_cleared_after_looking(tmp_path, capsys):
  """D11: route_audit lists every TurnPilotJunctionOverrides entry with its evidence, and --clear removes one after Kellen
  has looked (from a file holding the param's value, or the car's Params with --params)."""
  path = tmp_path / 'overrides.json'
  path.write_text(json.dumps([{'approach': '900:790:f', 'kind': 'no', 'evidence': {'speed': 8.2, 'log': 'a1b2--3'}},
                              {'approach': '901:12:b', 'kind': 'signalSeen', 'evidence': None}]))
  assert route_audit.main(['--overrides', str(path), '--list-overrides']) == 0
  out = capsys.readouterr().out
  assert '900:790:f' in out and "'speed': 8.2" in out and 'signalSeen' in out
  assert route_audit.main(['--overrides', str(path), '--clear', '900:790:f']) == 0
  assert [o['approach'] for o in json.loads(path.read_text())] == ['901:12:b']
  assert route_audit.main(['--overrides', str(path), '--clear', '900:790:f']) == 1  # not there: said, nothing written
  assert 'not in' in capsys.readouterr().err

  class Params:
    def __init__(self):
      self.value = json.dumps([{'approach': '5:6:f', 'kind': 'no', 'evidence': 'x'}])

    def get(self, key):
      return self.value

    def put(self, key, value):
      self.value = value
  p = Params()
  assert route_audit.clear_override_param(p, '5:6:f') and json.loads(p.value) == []


def test_the_time_avoidance_added(tmp_path):
  """The audit runs navd's avoidance loop (routing/avoid.py) on the trip and says what it did and the time it added."""
  from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import survey
  # the stop's approach edge isn't the route's first (that one can't be avoided)
  v, side, ctx = stub_allway(tmp_path, [survey()], [(0., 50., {'way_id': 49}), (50., 100., {'stop': True, 'way_id': 50}), (100., 400., {})])
  report = route_audit.audit(v, steps(), None, side, ctx, eta=ETA)
  longer = [dict(s, duration=40.) for s in steps()]
  result = route_audit.audit_avoidance(report, steps(), reroute=lambda exclude: longer, route_controls=lambda s: ([], 'ok'), eta=ETA)
  assert (result.status, result.avoided, result.iterations) == ('clean', 1, 1)
  assert result.added_s == pytest.approx(80. - sum(s.get('duration', 0.) for s in steps()))
  assert 'avoidance: clean, 1 stop routed around' in '\n'.join(route_audit.avoidance_lines(result))


def test_the_straight_continuations_first_control_at_a_certified_all_way_where_the_route_turns(tmp_path):
  """N12: the car may miss a turn and run straight through; for each certified all-way where the route turns, the audit
  looks straight on from the junction and lists the first stop or yield there with its certification."""
  from openpilot.selfdrive.turnpilot.routing.controls import Control
  from openpilot.selfdrive.turnpilot.routing.junctions import Junction
  turning = Control(0, 'stop', 90., 0, 90., junction_info=Junction(lat=39.8, lon=-86.15, heading_in=0., maneuver='left',
                                                                   certification='certifiable'))
  straight = Control(1, 'stop', 300., 0, 300., junction_info=Junction(maneuver='straight', certification='certifiable'))
  ahead = Control(5, 'yield', 140., 0, 140., junction_info=Junction(maneuver='unknown'))
  looked = []

  def lookahead(position, heading):
    looked.append((position, heading))
    return [Control(4, 'signal', 60., 0, 60.), ahead], 'ok', [position]

  rows = route_audit.continuations([turning, straight], lookahead, certify=lambda c: ('uncertifiable', 'notSurveyed'))
  assert looked == [((39.8, -86.15), 0.)]
  assert rows == [{'control': 0, 'route_m': 90., 'maneuver': 'left', 'next': 'yield', 'next_m': 140., 'certification': 'uncertifiable',
                   'reason': 'notSurveyed'}]
  none_ahead = route_audit.continuations([turning], lambda p, h: ([], 'ok', [p]), certify=lambda c: ('certifiable', 'none'))
  assert none_ahead[0]['next'] == 'none'
