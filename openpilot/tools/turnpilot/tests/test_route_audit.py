"""route_audit.py: every stop and yield approach on a route, with Valhalla's flag, the OSM sidecar's, the line source and
the OSM node id (STOP_SIGNS.md SG1a, step S2), on the stub Valhalla of routing/tests/test_route_controls.py."""
import json

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
