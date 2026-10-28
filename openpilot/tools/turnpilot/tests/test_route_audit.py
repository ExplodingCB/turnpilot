"""route_audit.py: every stop and yield approach on a route, with Valhalla's flag, the OSM sidecar's, the line source and
the OSM node id (STOP_SIGNS.md SG1a, step S2), on the stub Valhalla of routing/tests/test_route_controls.py."""
import json

from openpilot.selfdrive.turnpilot.routing.tests.test_route_controls import StubValhalla, entry, osm_dir, steps
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
