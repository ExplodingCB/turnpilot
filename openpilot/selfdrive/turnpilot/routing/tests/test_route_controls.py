"""The route's stop signs, yields and signals from Valhalla (routing/controls.py): on a stub Valhalla, and on the sim's
tiles with controls when they're built (tools/turnpilot/build_sim_tiles.sh into ~/turnpilot_data/valhalla/west_coast_usa_signals)."""
import importlib.util
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from openpilot.selfdrive.turnpilot.maps.corridor import level2_index
from openpilot.selfdrive.turnpilot.routing import controls as controls_module
from openpilot.selfdrive.turnpilot.routing.controls import (CONTROL_LINE_M, MAP_MARGIN_M, MARKER, SIDECAR, actor_call, controls_ahead,
                                                            haversine, http_call, route_controls)

M_PER_DEG = 111195.
LAT0, LON0 = 39.77, -86.16
OFF = CONTROL_LINE_M + MAP_MARGIN_M  # a control's stop point before its edge's end, with no mapped line
FIXTURES = Path(__file__).parent / 'fixtures'
STAMP = '2026-10-26T21:24:46Z'


def pt(m):
  return LAT0 + m / M_PER_DEG, LON0


def steps(lengths=(200., 200.), every=10.):
  """Steps due north, back to back, their geojson geometry a point every `every` m."""
  out, start = [], 0.
  for length in lengths:
    n = int(length / every)
    out.append({'geometry': {'coordinates': [[pt(start + k * every)[1], pt(start + k * every)[0]] for k in range(n + 1)]},
                'distance': length})
    start += length
  return out


class StubValhalla:
  """trace_attributes and locate over a straight route: edges [(begin m, end m, fields...)], a shape point every 10 m.
  locate answers, for a point on an edge, that edge's candidate with its stop and yield signs. every: m between shape points."""
  def __init__(self, edges, total=400., fail_edge_walk=False, unmatched=(), every=10.):
    self.total, self.fail_edge_walk, self.unmatched, self.every = total, fail_edge_walk, set(unmatched), every
    self.edges = []
    for i, (b, e, f) in enumerate(edges):
      d = {'id': 1000 + i, 'way_id': f.get('way_id', 50 + i), 'forward': f.get('forward', True), 'begin_shape_index': round(b / every),
           'end_shape_index': round(e / every), 'traffic_signal': f.get('edge_signal', False), 'road_class': f.get('road_class', 'residential'),
           'internal_intersection': f.get('internal', False), 'length': (e - b) / 1000.,
           'end_node': {'traffic_signal': f.get('node_signal', False),
                        'intersecting_edges': [{'road_class': c} for c in f.get('crossing', ['residential'])]}}
      self.edges.append((d, f.get('stop', False), f.get('yield', False)))
    self.requests = []

  def __call__(self, endpoint, req):
    self.requests.append((endpoint, req))
    if endpoint == 'trace_attributes':
      if self.fail_edge_walk and req['shape_match'] == 'edge_walk':
        raise RuntimeError("no edge_walk path")
      return {'shape': [{'lat': pt(k * self.every)[0], 'lon': pt(k * self.every)[1]} for k in range(round(self.total / self.every) + 1)],
              'edges': [d for d, _, _ in self.edges]}
    return [self.answer(loc) for loc in req['locations']]

  def answer(self, loc):
    """locate's answer for one location: the route edge it is inside, with its signs."""
    m = (loc['lat'] - LAT0) * M_PER_DEG
    hit = next(((d, s, y) for d, s, y in self.edges if d['begin_shape_index'] * self.every < m < d['end_shape_index'] * self.every), None)
    if hit is None or hit[0]['id'] in self.unmatched:
      return {'edges': [{'edge_id': {'value': 1}, 'edge': {'stop_sign': True}}]}
    return {'edges': [{'edge_id': {'value': hit[0]['id']}, 'edge': {'stop_sign': hit[1], 'yield_sign': hit[2]}}]}


class StubRoad(StubValhalla):
  """StubValhalla, and locate by heading (controls_ahead) on its road due north: road edges [(begin m, end m)]. At a
  point inside an edge, that edge; at a node (radius 1), the edge ending there and the one starting there."""
  def __init__(self, edges, road, **kw):
    super().__init__(edges, **kw)
    self.road = road

  def __call__(self, endpoint, req):
    if endpoint != 'locate' or 'heading' not in req['locations'][0]:
      return super().__call__(endpoint, req)
    self.requests.append((endpoint, req))
    loc = req['locations'][0]
    if abs(loc['heading'] % 360) > 45 and abs(loc['heading'] % 360) < 315:
      return [{'edges': []}]  # nothing heads that way
    m = (loc['lat'] - LAT0) * M_PER_DEG
    out = []
    for b, e in self.road:
      shape = [{'lat': pt(x)[0], 'lon': pt(x)[1]} for x in (b, (b + e) / 2, e)]
      if loc['radius'] == 1 and (abs(m - b) < 0.5 or abs(m - e) < 0.5):
        out.append({'edge_info': {'shape': shape}, 'edge': {'forward': True}, 'percent_along': 0. if abs(m - b) < 0.5 else 1.,
                    'distance': 0., 'heading': 0.})
      elif loc['radius'] > 1 and b <= m < e:
        out.append({'edge_info': {'shape': shape}, 'edge': {'forward': True}, 'percent_along': (m - b) / (e - b), 'distance': 2.,
                    'heading': 0.})
    return [{'edges': out}]


def test_controls_ahead_follow_the_road_from_the_car_edge_after_edge():
  """Off the route (navd: offRouteControls): the road the car is on, from where it is, by its heading, then on from each
  node by the edge carrying on, walked like a route. Its end isn't a destination: a control there counts."""
  v = StubRoad([(0., 100., {'stop': True}), (100., 250., {'node_signal': True}), (250., 400., {'stop': True})],
               road=[(0., 100.), (100., 250.), (250., 400.)])
  controls, status, road = controls_ahead(v, pt(0.), 3.)
  assert status == 'ok'
  assert [(c.kind, round(c.route_m, 1)) for c in controls] == \
    [('stop', 100. - CONTROL_LINE_M - MAP_MARGIN_M), ('signal', 250. - CONTROL_LINE_M - MAP_MARGIN_M),
     ('stop', 400. - CONTROL_LINE_M - MAP_MARGIN_M)]
  assert abs(haversine(road[0], road[-1]) - 400.) < 0.5
  assert all(abs(haversine(c.point, pt(c.route_m))) < 0.5 for c in controls)
  heads = [r['locations'][0] for e, r in v.requests if e == 'locate' and 'heading' in r['locations'][0]]
  assert heads[0]['radius'] > 1 and all(h['radius'] == 1 for h in heads[1:]) and len(heads) == 3


def test_controls_ahead_unknown_is_failed_never_none():
  v = StubRoad([(0., 400., {'node_signal': True})], road=[(0., 400.)])
  assert controls_ahead(v, pt(0.), 180.)[:2] == ([], 'failed')  # no road heading that way
  assert controls_ahead(v, pt(0.), float('nan'))[:2] == ([], 'failed')  # no heading
  assert controls_ahead(lambda *_: 1 / 0, pt(0.), 0.)[:2] == ([], 'failed')


def test_controls_ahead_unsupported_without_the_marker(tmp_path):
  v = StubRoad([(0., 400., {'node_signal': True})], road=[(0., 400.)])
  assert controls_ahead(v, pt(0.), 0., tmp_path)[:2] == ([], 'unsupported')
  assert controls_ahead(v, pt(0.), 0., marked(tmp_path))[1] == 'ok'


def marked(tmp_path, sidecar=None):
  (tmp_path / MARKER).touch()
  if sidecar is not None:
    (tmp_path / SIDECAR).write_text(json.dumps({'controls': sidecar}))
  return tmp_path


def summary(controls):
  return [(c.kind, round(c.route_m, 1), c.step_idx, round(c.along_step_m, 1), c.line_mapped, c.junction, c.cross_suspect) for c in controls]


def test_kinds_at_the_edge_end_less_the_line_offset():
  v = StubValhalla([(0., 100., {'stop': True}), (100., 250., {'node_signal': True}), (250., 330., {'edge_signal': True}),
                    (330., 400., {})])
  controls, status = route_controls(v, steps())
  assert status == 'ok'
  off = CONTROL_LINE_M + MAP_MARGIN_M
  assert summary(controls) == [('stop', 100. - off, 0, 100. - off, False, True, False),
                               ('signal', 250. - off, 1, 50. - off, False, True, False),
                               ('signal', 330. - off, 1, 130. - off, False, True, False)]
  # one batched verbose locate, a metre back from each edge's end (Q1); then one more for the stop's junction (Q2, SG1b)
  locates = [r for e, r in v.requests if e == 'locate']
  assert len(locates) == 2 and locates[0]['verbose'] and len(locates[0]['locations']) == 4
  assert abs((locates[0]['locations'][0]['lat'] - LAT0) * M_PER_DEG - 99.) < 0.05
  assert locates[1]['verbose'] and locates[1]['locations'][0]['radius'] == 1  # the stop's node first


def test_yield_and_the_edge_at_the_destination():
  # the last edge ends at the route's end (the destination): a control there isn't driven to
  v = StubValhalla([(0., 150., {'yield': True}), (150., 400., {'stop': True})])
  controls, _ = route_controls(v, steps())
  assert [c.kind for c in controls] == ['yield']


def test_an_internal_edge_with_no_junction_before_it_on_the_route_keeps_its_own_end():
  # the route starts in the median: the control at the internal edge's end is still ahead of the car
  v = StubValhalla([(0., 100., {'node_signal': True, 'internal': True}), (100., 200., {'stop': True}), (200., 400., {})])
  assert [(c.kind, round(c.route_m, 1)) for c in route_controls(v, steps())[0]] == [('signal', 100. - OFF), ('stop', 200. - OFF)]


def test_kinds_at_one_edge_end_merge_and_controls_at_different_nodes_never_do():
  """One edge end is one node: its kinds are one control, the signal over the stop over the yield. A control at the next
  node 10 m on is its own (STOP_SIGNS.md G11, G18: no merging by distance, which once dropped a stop behind a signal)."""
  v = StubValhalla([(0., 100., {'stop': True, 'node_signal': True}), (100., 110., {'node_signal': True}), (110., 300., {'yield': True}),
                    (300., 400., {})])
  controls, _ = route_controls(v, steps())
  assert [(c.kind, c.id, round(c.route_m, 1)) for c in controls] == [('signal', 0, 100. - OFF), ('signal', 1, 110. - OFF), ('yield', 2, 300. - OFF)]


def test_g18_two_stops_at_different_nodes_12_m_apart_stay_two():
  """A frontage road: a stop at one node and another 12 m on, each its own control (they merged within MERGE_M before)."""
  v = StubValhalla([(0., 100., {'stop': True}), (100., 112., {'stop': True}), (112., 400., {})], every=2.)
  controls, status = route_controls(v, steps())
  assert status == 'ok' and [(c.kind, round(c.route_m, 1)) for c in controls] == [('stop', 100. - OFF), ('stop', 112. - OFF)]


def test_g11_a_stop_on_a_median_internal_edge_belongs_to_the_junction_before_it(tmp_path):
  """A divided cross road: Valhalla puts a stop or yield mapped in the median on an internal-intersection edge, which
  was never searched. It is now, and its control is the preceding junction's (STOP_SIGNS.md G11): the car stops before
  the first carriageway, never in the middle of the crossing."""
  v = StubValhalla([(0., 100., {}), (100., 120., {'internal': True, 'stop': True}), (120., 400., {})])
  controls, status = route_controls(v, steps())
  assert status == 'ok' and [(c.kind, round(c.route_m, 1), c.line_mapped) for c in controls] == [('stop', 100. - OFF, False)]
  assert [len(r['locations']) for e, r in v.requests if e == 'locate'][0] == 3  # the internal edge is located too (Q1)
  # the junction's own control and the median's merge there, the higher kind kept
  v = StubValhalla([(0., 100., {'yield': True}), (100., 120., {'internal': True, 'stop': True}), (120., 400., {})])
  assert [(c.kind, round(c.route_m, 1)) for c in route_controls(v, steps())[0]] == [('stop', 100. - OFF)]
  # a yield the OSM sidecar maps in the median, which Valhalla doesn't flag
  v = StubValhalla([(0., 100., {}), (100., 120., {'internal': True, 'way_id': 66}), (120., 400., {})])
  controls, status = route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path, {'66:f': [entry('yield', 115., junction=901)]}))
  assert status == 'ok' and [(c.kind, round(c.route_m, 1), c.osm_node) for c in controls] == [('yield', 100. - OFF, 901)]


def test_cross_suspect_and_junction():
  v = StubValhalla([(0., 100., {'stop': True, 'road_class': 'primary', 'crossing': ['residential', 'service_other']}),
                    (100., 200., {'yield': True, 'road_class': 'residential', 'crossing': ['residential']}),
                    (200., 300., {'stop': True, 'crossing': []}), (300., 400., {})])
  controls, _ = route_controls(v, steps())
  assert [(c.kind, c.junction, c.cross_suspect) for c in controls] == [('stop', True, True), ('yield', True, False), ('stop', False, False)]


def test_walk_or_snap_and_partial_said():
  v = StubValhalla([(0., 100., {'stop': True}), (100., 400., {})], fail_edge_walk=True)
  controls, status = route_controls(v, steps())
  assert status == 'walkOrSnap' and [c.kind for c in controls] == ['stop']
  v = StubValhalla([(0., 100., {'stop': True}), (100., 200., {}), (200., 400., {})], unmatched=(1001,))
  assert route_controls(v, steps())[1] == 'partial'


def test_failures_are_failed_and_unsupported_without_the_marker(tmp_path):
  def broken(endpoint, req):
    raise ConnectionError("no valhalla_service")  # pyvalhalla missing, and no service either: loud, never empty-and-ok
  assert route_controls(broken, steps()) == ([], 'failed')
  assert route_controls(StubValhalla([(0., 400., {})]), [{'geometry': {'coordinates': []}, 'distance': 0.}]) == ([], 'failed')
  assert route_controls(StubValhalla([(0., 100., {'stop': True}), (100., 400., {})]), steps(), tiles_dir=tmp_path) == ([], 'unsupported')
  assert route_controls(StubValhalla([(0., 100., {'stop': True}), (100., 400., {})]), steps(), tiles_dir=marked(tmp_path))[1] == 'ok'


def test_sidecar_line_is_projected(tmp_path):
  line = pt(93.5)
  side = {'77:f': {'kind': 'stop', 'instance': 'stop 7', 'line': list(line)}, '77:b': {'kind': 'stop', 'line': list(pt(20.))}}
  v = StubValhalla([(0., 100., {'stop': True, 'way_id': 77}), (100., 200., {'stop': True, 'way_id': 78}), (200., 400., {})])
  controls, status = route_controls(v, steps(), tiles_dir=marked(tmp_path, side))
  assert status == 'ok'
  assert abs(controls[0].route_m - (93.5 - MAP_MARGIN_M)) < 1. and controls[0].line_mapped and controls[0].instance == 'stop 7'
  assert not controls[1].line_mapped and abs(controls[1].route_m - (200. - CONTROL_LINE_M - MAP_MARGIN_M)) < 0.1


# --- the OSM sidecar (osm_controls/ shards, routing/osm_sidecar.py; STOP_SIGNS.md SG1a) ---

def entry(kind, m, junction=900, evidence='mid_directed', stop_tag='none', east=0., node=None):
  """A sidecar control whose line is m along the stub route (and east m off it), or None for a junction-node one; node:
  the sign's OSM node id, which shards carry (route_audit)."""
  line = None if m is None else [pt(m)[0], LON0 + east / (M_PER_DEG * 0.7687)]
  return {'kind': kind, 'line': line, 'junction_node': junction, 'stop_tag': stop_tag, 'evidence': evidence} | \
    ({} if node is None else {'node': node})


def osm_dir(path, controls=None, junctions=None, stamp=STAMP, tiles_stamp=STAMP, meta=True, shards=True, unplaced=None):
  """A directory with the sidecar's shard for the stub route's level-2 tile, stamped stamp, and the tiles' own stamp
  (meta.json, as a region's or the sim tiles' directory has; a corridor's manifest.json has it too)."""
  path.mkdir(parents=True, exist_ok=True)
  if meta:
    (path / 'meta.json').write_text(json.dumps({'id': 'test', 'osm_timestamp': tiles_stamp}))
  if shards:
    cell = level2_index(LON0, LAT0)
    (path / 'osm_controls').mkdir(exist_ok=True)
    (path / 'osm_controls' / f'{cell}.json').write_text(json.dumps({'version': 1, 'osm_timestamp': stamp, 'tile': cell,
                                                                    'controls': controls or {}, 'junctions': junctions or {}, 'rail': [],
                                                                    'unplaced': unplaced or []}))
  return path


def replay(fixture):
  """call() answering from a recorded trace_attributes and one batched locate (Q1); a later locate (the junctions' Q2,
  SG1b), which the fixture didn't record, finds nothing at its points."""
  calls = []

  def call(endpoint, req):
    if endpoint == 'trace_attributes':
      return fixture['trace_attributes']
    assert endpoint == 'locate'
    calls.append(req)
    if len(calls) > 1:
      return [{'edges': []} for _ in req['locations']]
    assert len(req['locations']) == len(fixture['locate'])
    return fixture['locate']
  return call


def test_c13_an_osm_mapped_stop_valhalla_does_not_flag_is_returned(tmp_path):
  """A quarter of the OSM-mapped stop approaches have no stop_sign flag in Valhalla's tiles (research_allway.md section 0):
  the sidecar's stop is unioned in, at the default stop point (the edge's end less CONTROL_LINE_M), its OSM stop node
  carried as data only (osm_sign_m, for route_audit; SG1b's stop-point rule may move it)."""
  fixture = json.loads((FIXTURES / 'valhalla_unflagged_stop.json').read_text())
  assert route_controls(replay(fixture), fixture['steps']) == ([], 'ok')  # Valhalla alone: no stop there
  side = tmp_path / 'pack'
  (side / 'osm_controls').mkdir(parents=True)
  (side / 'manifest.json').write_text(json.dumps({'region': 'indianapolis', 'version': 'v', 'files': {}, **fixture['meta']}))
  for cell, shard in fixture['osm_controls'].items():
    (side / 'osm_controls' / f'{cell}.json').write_text(json.dumps(shard))
  controls, status = route_controls(replay(fixture), fixture['steps'], sidecar_dir=side)
  assert status == 'ok'
  assert [(c.kind, c.line_mapped, c.line_source, c.osm_node, c.step_idx, c.junction_status) for c in controls] == \
    [('stop', False, 'default', 5001, 0, 'ok')]
  assert abs(controls[0].route_m - (150. - OFF)) < 0.5 and abs(controls[0].osm_sign_m - 135.) < 0.5


def test_c14_a_phantom_far_end_flag_is_overruled_by_the_sidecars_direction(tmp_path):
  """A stop on a node Valhalla can't direct (a crosswalk's, shared with a footway) is flagged both ways: the far direction
  gets a phantom stop at that node, 10 m past the junction before it, which looks like a stop on the approach to that
  junction. The sidecar's direction=forward says it is the other direction's: the phantom goes. An undirected stop's
  other end may be its real junction, so there Valhalla's flag stays (an extra stop is cheap; a dropped one isn't)."""
  edges = [(0., 90., {}), (90., 100., {'stop': True, 'way_id': 77, 'forward': False}), (100., 400., {})]
  directed = {'77:f': [entry('stop', 100., junction=900, evidence='mid_directed')]}
  with patch.object(controls_module, 'cloudlog') as log:
    controls, status = route_controls(StubValhalla(edges), steps(), sidecar_dir=osm_dir(tmp_path / 'directed', directed))
  assert status == 'ok' and controls == []
  assert any('phantom' in str(c.args[0]) for c in log.info.call_args_list + log.warning.call_args_list)
  undirected = {'77:f': [entry('stop', 100., junction=900, evidence='mid_undirected')]}
  controls, _ = route_controls(StubValhalla(edges), steps(), sidecar_dir=osm_dir(tmp_path / 'undirected', undirected))
  assert [(c.kind, round(c.route_m, 1)) for c in controls] == [('stop', 100. - OFF)]
  both = {**directed, '77:b': [entry('stop', 100., junction=901, evidence='mid_both')]}  # ours too: no phantom
  controls, _ = route_controls(StubValhalla(edges), steps(), sidecar_dir=osm_dir(tmp_path / 'both', both))
  assert [(c.kind, round(c.route_m, 1), c.osm_node) for c in controls] == [('stop', 100. - OFF, 901)]


def test_c14_a_phantom_on_a_way_split_is_overruled_by_the_next_ways_entry(tmp_path):
  """Our edge (way 77, against its order) ends at a way split (no other road meets there), where the route carries on
  along way 78. The other direction's directed stop on the split node is keyed to the way its traffic arrives on, way
  78 against ours (78:b), never 77:f, so the phantom check looks there too (Lexington KY: way 16169193 and 1334314513).
  A stop keyed to our own direction on the next way is no phantom's evidence, and where other roads meet (a junction,
  where the route may turn onto the cross road) the next way's entries are never read: a phantom lives on a node
  Valhalla can't direct, not at a junction."""
  split = [(0., 90., {}), (90., 100., {'stop': True, 'way_id': 77, 'forward': False, 'crossing': []}), (100., 400., {'way_id': 78})]
  theirs = {'78:b': [entry('stop', 100., junction=900)]}
  controls, status = route_controls(StubValhalla(split), steps(), sidecar_dir=osm_dir(tmp_path / 'theirs', theirs))
  assert status == 'ok' and controls == []
  ours = {'78:f': [entry('stop', 100., junction=900)]}
  controls, _ = route_controls(StubValhalla(split), steps(), sidecar_dir=osm_dir(tmp_path / 'ours', ours))
  assert [(c.kind, round(c.route_m, 1)) for c in controls] == [('stop', 100. - OFF)]
  junction = [(0., 90., {}), (90., 100., {'stop': True, 'way_id': 77, 'forward': False, 'crossing': ['residential']}),
              (100., 400., {'way_id': 78})]
  controls, _ = route_controls(StubValhalla(junction), steps(), sidecar_dir=osm_dir(tmp_path / 'junction', theirs))
  assert [(c.kind, round(c.route_m, 1)) for c in controls] == [('stop', 100. - OFF)]


@pytest.mark.parametrize('case', ['beside the node', 'mid-edge', 'ours an orphan', 'ours a yield', 'ours undirected'])
def test_b1_a_missing_sidecar_entry_for_us_never_drops_a_real_stop(tmp_path, case):
  """Augusta Way (Highland Park IL, way 550724913 westbound): a mid-block footway crossing with a directed stop each side
  of it, both inside our one Valhalla edge, which ends where the way does 30 m on (mid-edge). Ours has no junction
  within 60 m along its own way, so a sidecar built without orphans has no entry for it; the other direction's governs a
  junction and lies inside our edge. Valhalla's flag for us is the real stop. A missing entry for us doesn't prove there
  is no stop for us: only the other direction's directed stop ON our edge's end node, with nothing of ours (any kind,
  any direction tag, junction or none) on the edge, is the phantom (C14). Every other case keeps the flag (an extra stop
  is cheap)."""
  edges = [(0., 100., {}), (100., 150., {'stop': True, 'way_id': 77, 'forward': False}), (150., 400., {})]
  side = {'beside the node': {'77:f': [entry('stop', 152.3, junction=900)]},
          'mid-edge': {'77:f': [entry('stop', 120., junction=900)]},
          # on our end node, but ours is there too: an orphan (no junction within 60 m), a yield, an undirected stop
          'ours an orphan': {'77:f': [entry('stop', 150., junction=900)], '77:b': [entry('stop', 146., junction=0)]},
          'ours a yield': {'77:f': [entry('stop', 150., junction=900)], '77:b': [entry('yield', 146., junction=0)]},
          'ours undirected': {'77:f': [entry('stop', 150., junction=900), entry('stop', 140., junction=902, evidence='mid_undirected')]},
          }[case]
  with patch.object(controls_module, 'cloudlog') as log:
    controls, status = route_controls(StubValhalla(edges, every=2.), steps(every=2.), sidecar_dir=osm_dir(tmp_path, side))
  assert status == 'ok' and [(c.kind, round(c.route_m, 1)) for c in controls] == [('stop', 150. - OFF)]
  assert not any('phantom' in str(c.args[0]) for c in log.info.call_args_list)


def test_the_sidecar_entry_for_an_edge_is_the_one_whose_line_is_inside_it(tmp_path):
  """A way passes several junctions: its key has a list, and each edge takes the entry whose line projects inside its
  own span, within a few metres of the route (never one off to the side)."""
  v = StubValhalla([(0., 100., {'way_id': 77}), (100., 200., {'way_id': 77}), (200., 400., {'way_id': 77})])
  side = {'77:f': [entry('stop', 185., junction=902), entry('stop', 80., junction=901), entry('stop', 300., junction=903, east=30.)]}
  controls, status = route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path, side))
  assert status == 'ok'
  assert [(c.kind, round(c.route_m, 1), c.osm_node, c.line_mapped, round(c.osm_sign_m, 1)) for c in controls] == \
    [('stop', 100. - OFF, 901, False, 80.), ('stop', 200. - OFF, 902, False, 185.)]


@pytest.mark.parametrize('kind, sign_m', [('stop', 82.), ('stop', 70.), ('stop', 95.), ('yield', 66.)])
def test_b2_an_osm_stop_node_never_moves_the_stop_point(tmp_path, kind, sign_m):
  """An OSM stop node marks a sign, not a line, and SG1a moves no stop point: Valhalla's stop or yield stays at the
  default (its edge's end less CONTROL_LINE_M) whether the sign is 18, 30 or 5 m back, and takes the sidecar's junction
  node and the sign's place as data (osm_sign_m, route_audit). A yield moved back to its sign would end map_controls'
  YIELD_SPEED cap there, 30-40 m short of the merge (the I-70 East on-ramp's moved from 749 to 715 m), so the car could
  reach the conflict point well above 5 m/s. SG1b's stop-point rule, which knows the cross road's width, may move it."""
  v = StubValhalla([(0., 100., {kind: True, 'way_id': 77}), (100., 400., {})])
  controls, _ = route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path, {'77:f': [entry(kind, sign_m, junction=900)]}))
  assert [(c.kind, round(c.route_m, 1), c.line_mapped, c.line_source, c.osm_node, round(c.osm_sign_m, 1)) for c in controls] == \
    [(kind, 100. - OFF, False, 'default', 900, sign_m)]


def test_b3_a_sidecar_stop_tied_to_no_junction_near_its_edges_end_is_still_a_stop(tmp_path):
  """The sidecar keeps a stop it ties to no junction within 60 m (a mid-block trail or rail crossing) with junction_node
  0, an orphan: where Valhalla doesn't flag it either, the union is the car's only source for it. With its sign within
  ORPHAN_NEAR_M (CONTROL_LINE_M + 5 m) of its edge's end it stops at the edge's default stop point, at most 4 m past the
  sign, with no OSM junction node, its sign carried as data."""
  assert controls_module.ORPHAN_NEAR_M == CONTROL_LINE_M + 5.
  v = StubValhalla([(0., 100., {}), (100., 180., {'way_id': 77}), (180., 400., {})])
  for sign_m in (176., 166.):
    controls, status = route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path / str(sign_m), {'77:f': [entry('stop', sign_m, junction=0)]}))
    assert status == 'ok'
    assert [(c.kind, round(c.route_m, 1), c.osm_node, c.line_source, round(c.osm_sign_m, 1)) for c in controls] == \
      [('stop', 180. - OFF, 0, 'default', sign_m)]


@pytest.mark.parametrize('kind, sign_m, end_m, crossing', [
  ('stop', 140., 180., ['residential']),  # the old B3 shape: 40 m before a junction
  ('stop', 140., 500., []),               # 360 m before a node no other road meets: the stop would be mid-block
  ('stop', 140., 500., ['residential']),  # 360 m before a junction (Jim Town Road, 635 m)
  ('stop', 240., 300., ['residential']),  # 60 m (4th St NW, a reversed direction tag: 62 m)
  ('yield', 140., 180., ['residential']),
])
def test_sg1a_an_orphan_sign_far_before_its_edges_end_is_left_out(tmp_path, kind, sign_m, end_m, crossing):
  """An orphan's sign further than ORPHAN_NEAR_M before its Valhalla edge's end: the edge's end less CONTROL_LINE_M would
  put the stop (or the yield's cap) that far past the sign, where there is none, often mid-block, and the car would drive
  past the real sign unslowed and then stop there (Indianapolis: 146 of 176 such orphans more than 15 m past their sign,
  the furthest 1119 m). It is left out of the controls and logged, as the base had it (route_audit lists it as uncovered);
  never placed at its sign either, since a reversed direction tag would then stop the car past a junction (4th St NW)."""
  v = StubValhalla([(0., 100., {}), (100., end_m, {'way_id': 77, 'crossing': crossing}), (end_m, 800., {})], total=800.)
  with patch.object(controls_module, 'cloudlog') as log:
    controls, status = route_controls(v, steps((400., 400.)), sidecar_dir=osm_dir(tmp_path, {'77:f': [entry(kind, sign_m, junction=0)]}))
  assert status == 'ok' and controls == []
  assert any('left out' in str(c.args[0]) and '77:f' in str(c.args[0]) for c in log.info.call_args_list)


def test_sg1a_a_far_orphan_neither_drops_nor_moves_valhallas_own_flag(tmp_path):
  """Valhalla's own flag on the edge stays where it was (its edge's end less CONTROL_LINE_M), the far orphan's sign carried
  as data (osm_sign_m), and a far orphan of ours still counts as ours on the edge for the phantom check (C14)."""
  v = StubValhalla([(0., 100., {}), (100., 180., {'way_id': 77, 'stop': True}), (180., 400., {})])
  controls, _ = route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path / 'flag', {'77:f': [entry('stop', 140., junction=0)]}))
  assert [(c.kind, round(c.route_m, 1), c.osm_node, round(c.osm_sign_m, 1)) for c in controls] == [('stop', 180. - OFF, 0, 140.)]
  edges = [(0., 100., {}), (100., 150., {'stop': True, 'way_id': 77, 'forward': False}), (150., 400., {})]
  side = {'77:f': [entry('stop', 150., junction=900)], '77:b': [entry('stop', 120., junction=0)]}
  controls, _ = route_controls(StubValhalla(edges, every=2.), steps(every=2.), sidecar_dir=osm_dir(tmp_path / 'phantom', side))
  assert [(c.kind, round(c.route_m, 1)) for c in controls] == [('stop', 150. - OFF)]


def test_a_junction_node_stop_from_the_sidecar_is_on_the_edge_ending_there(tmp_path):
  v = StubValhalla([(0., 100., {'way_id': 77}), (100., 400., {'way_id': 78})])
  side = {'77:f': [entry('stop', None, junction=903, evidence='node_untagged')]}
  at = {'903': {'pos': list(pt(100.)), 'approaches': []}}
  controls, _ = route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path / 'a', side, at))
  assert [(c.kind, round(c.route_m, 1), c.line_mapped, c.osm_node) for c in controls] == [('stop', 100. - OFF, False, 903)]
  elsewhere = {'903': {'pos': list(pt(300.)), 'approaches': []}}  # that junction is further along the way: not this edge's end
  assert route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path / 'b', side, elsewhere))[0] == []


@pytest.mark.parametrize('case', ['missing', 'stale', 'no tiles stamp', 'no directory'])
def test_n7_a_missing_or_stale_sidecar_is_loud(tmp_path, case):
  """No sidecar where one is expected, or one whose osm_timestamp isn't the tiles': the stops Valhalla missed are unknown.
  Said out loud: the status is partial (the UI's "Map stops unavailable", map_controls' mapControlsUnavailable, off the
  route the stops-unknown cap), each stop and yield's junction status is failed, and it is logged. Valhalla's own
  controls still go out."""
  sidecar = {'missing': lambda: osm_dir(tmp_path, shards=False), 'stale': lambda: osm_dir(tmp_path, stamp='2026-09-01T00:00:00Z'),
             'no tiles stamp': lambda: osm_dir(tmp_path, meta=False), 'no directory': lambda: ''}[case]()
  v = StubValhalla([(0., 100., {'stop': True}), (100., 250., {'node_signal': True}), (250., 400., {})])
  with patch.object(controls_module, 'cloudlog') as log:
    controls, status = route_controls(v, steps(), sidecar_dir=sidecar)
  assert status == 'partial'
  assert [(c.kind, c.junction_status) for c in controls] == [('stop', 'failed'), ('signal', 'ok')]
  assert log.error.called and 'sidecar' in log.error.call_args.args[0]
  road = StubRoad([(0., 100., {'stop': True}), (100., 400., {})], road=[(0., 100.), (100., 400.)])
  assert controls_ahead(road, pt(0.), 3., sidecar_dir=sidecar)[1] == 'partial'
  good = osm_dir(tmp_path / 'good')
  assert route_controls(StubValhalla([(0., 100., {'stop': True}), (100., 400., {})]), steps(), sidecar_dir=good)[1] == 'ok'


def test_http_equals_actor():
  v = StubValhalla([(0., 100., {'stop': True}), (100., 250., {'node_signal': True}), (250., 400., {})])

  class Resp:
    def __init__(self, body):
      self.body = body

    def raise_for_status(self):
      pass

    def json(self):
      return json.loads(json.dumps(self.body))

  posted = []
  def post(url, json=None, timeout=None):
    posted.append((url, timeout))
    return Resp(v(url.rsplit('/', 1)[1], json))

  class Actor:
    def trace_attributes(self, req):
      return json.dumps(v('trace_attributes', req))  # some pyvalhalla versions answer a JSON string

    def locate(self, req):
      return v('locate', req)
  via_http = route_controls(http_call('http://127.0.0.1:8002', post), steps())
  via_actor = route_controls(actor_call(Actor()), steps())
  assert via_http == via_actor and via_http[0]
  assert posted[0] == ('http://127.0.0.1:8002/trace_attributes', 5.) and posted[1][0].endswith('/locate')


# --- the sim's tiles, with the level's stop signs and signals ---

SIGNALS_TILES = Path('~/turnpilot_data/valhalla/west_coast_usa_signals').expanduser()
SIGNALS_OSM = Path('~/turnpilot_data/osm/west_coast_usa_signals.osm').expanduser()
HAVE_TILES = importlib.util.find_spec('valhalla') is not None and (SIGNALS_TILES / MARKER).exists() and SIGNALS_OSM.exists()


@pytest.fixture(scope='module')
def sim_map():
  from openpilot.selfdrive.turnpilot.routing.valhalla_source import ValhallaSource
  root = ET.parse(SIGNALS_OSM).getroot()
  ll, ctrl = {}, {}
  for el in root.iter('node'):
    ll[el.get('id')] = (float(el.get('lat')), float(el.get('lon')))
    tags = {t.get('k'): t.get('v') for t in el.findall('tag')}
    if 'beamng:signal' in tags:
      ctrl[tags['beamng:signal']] = el.get('id')
  ways = [[nd.get('ref') for nd in w.findall('nd')] for w in root.iter('way')]
  return ValhallaSource(str(SIGNALS_TILES)), ll, ctrl, ways


def through(sim_map, instance):
  """{'along', 'against'}: the controls' instances on a route over the control's way each way, on past its far end."""
  src, ll, ctrl, ways = sim_map
  w = next(w for w in ways if ctrl[instance] in w)
  out = {}
  for label, (a, b) in (('along', (w[0], w[-1])), ('against', (w[-1], w[0]))):
    far = next((x[-1] if x[0] == b else x[0]) for x in ways if x is not w and b in (x[0], x[-1]))
    steps_ = src.route(ll[a][::-1], ll[far][::-1], None, None)
    controls, status = src.controls(steps_, tiles_dir=src.tiles_dir, sidecar_dir=src.tiles_dir)
    assert status == 'ok'
    out[label] = [c.instance for c in controls if c.line_mapped]
  return out


@pytest.mark.skipif(not HAVE_TILES, reason="no west_coast_usa_signals tiles (tools/turnpilot/build_sim_tiles.sh)")
@pytest.mark.parametrize("instance, along", [('stop 19', True), ('stop 35', False), ('stop 43', True), ('trafficLight 13', False)])
def test_sim_controls_only_on_their_approach(sim_map, instance, along):
  got = through(sim_map, instance)
  assert (instance in got['along']) == along and (instance in got['against']) == (not along), got


@pytest.mark.skipif(not HAVE_TILES, reason="no west_coast_usa_signals tiles (tools/turnpilot/build_sim_tiles.sh)")
def test_sim_controls_ahead_of_the_car_off_the_route(sim_map):
  """BeamNG route_demo, 2026-10-28: rerouting at (-332, 495) heading southwest, the car drove through the signal at
  (-382, 445). The lookup ahead of the car finds it on its mapped line; heading the other way, it isn't ahead."""
  src = sim_map[0]
  controls, status, _ = src.controls_ahead((39.7728582, -86.1619751), 225., tiles_dir=src.tiles_dir, sidecar_dir=src.tiles_dir)
  assert status == 'ok'
  first = controls[0]
  assert (first.kind, first.line_mapped) == ('signal', True) and 60. < first.route_m < 75., controls
  assert haversine(first.point, (39.7724078, -86.1625586)) < 6.
  controls, status, _ = src.controls_ahead((39.7724078, -86.1625586), 45., tiles_dir=src.tiles_dir, sidecar_dir=src.tiles_dir)
  assert status == 'ok' and all(c.kind != 'signal' or c.route_m > 200. for c in controls), controls


# --- the junction at a stop or yield (MapControl.junctionInfo, routing/junctions.py; STOP_SIGNS.md SG1b) ---

def toward(p, heading, m):
  """The point m from p (lat, lon) heading that way."""
  r = math.radians(heading)
  return p[0] + m * math.cos(r) / M_PER_DEG, p[1] + m * math.sin(r) / (M_PER_DEG * math.cos(math.radians(p[0])))


def leg(h, **kw):
  """A road leaving a stub junction heading h: class, use, drive (both, forward: cars only leave, backward: only arrive),
  lanes in and out, kph (its tagged limit), way, its inbound edge's signs (stop, yield, signal), fwd (the inbound edge's
  direction along its way), cycle (cycle_lane), cyclability, shoulder; match False: locate finds nothing at its probe or
  node; exit: the road the route leaves by."""
  return {'h': h, 'class': 'residential', 'use': 'road', 'drive': 'both', 'in': 1, 'out': 1, 'kph': None, 'way': 700 + int(h),
          'stop': False, 'yield': False, 'signal': False, 'fwd': True, 'cycle': 'none', 'cyclability': None, 'shoulder': False,
          'match': True, 'exit': False} | kw


class StubJunction(StubValhalla):
  """StubValhalla with junctions on its route due north: {node m: [leg(...), ...]}. trace_attributes gives each edge its
  headings (north, the begin heading of an edge leaving by an exit leg its heading) and its end node's intersecting edges
  (every leg but the exit, whose road the route's next edge is). locate answers a leg's probe point (radius 5, near the point LEG_PROBE_M out along it) with the
  leg's inbound and outbound edges, the node's own point (radius 1) with every leg's, and an opposing-road probe (with a
  heading) with the inbound edge of the leg it points at; anything else as StubValhalla."""

  def __init__(self, edges, junctions, **kw):
    super().__init__(edges, **kw)
    self.junctions = junctions
    for d, _, _ in self.edges:
      end_m, begin_m = d['end_shape_index'] * self.every, d['begin_shape_index'] * self.every
      d['end_heading'] = 0.
      out = next((x for x in junctions.get(begin_m, []) if x['exit']), None)
      d['begin_heading'] = out['h'] if out else 0.
      if out:  # the route's edge leaving by the exit leg is that road's
        d['road_class'], d['use'], d['way_id'], d['traversability'] = out['class'], out['use'], out['way'], out['drive']
      if end_m in junctions:
        d['end_node']['intersecting_edges'] = [{'begin_heading': x['h'], 'road_class': x['class'], 'use': x['use'],
                                                'driveability': x['drive'], 'cyclability': x['cyclability']}
                                               for x in junctions[end_m] if not x['exit']]

  def candidates(self, node_m, x):
    node = pt(node_m)
    far = toward(node, x['h'], 50.)
    out = []
    for inbound in (True, False):
      if x['drive'] not in (('backward', 'both') if inbound else ('forward', 'both')):
        continue
      travel = [far, node] if inbound else [node, far]
      forward = x['fwd'] if inbound else not x['fwd']
      edge = {'stop_sign': inbound and x['stop'], 'yield_sign': inbound and x['yield'], 'traffic_signal': inbound and x['signal'],
              'lane_count': x['in'] if inbound else x['out'], 'forward': forward, 'cycle_lane': x['cycle'],
              'classification': {'classification': x['class'], 'use': x['use']}}
      shape = [{'lat': q[0], 'lon': q[1]} for q in (travel if forward else travel[::-1])]
      out.append({'edge_id': {'value': 9000 + int(x['h']) * 2 + inbound}, 'edge': edge, 'shoulder': x['shoulder'],
                  'edge_info': {'way_id': x['way'], 'speed_limit': x['kph'] or 0, 'shape': shape}, 'distance': 0.5,
                  'percent_along': 1. if inbound else 0., 'heading': (x['h'] + 180.) % 360. if inbound else x['h']})
    return out

  def answer(self, loc):
    p = (loc['lat'], loc['lon'])
    for node_m, legs in self.junctions.items():
      node = pt(node_m)
      if 'heading' in loc:  # an opposing road's probe: the inbound edge of the leg it points at
        for x in legs:
          if x['match'] and haversine(p, toward(node, x['h'], 15.)) < 3.:
            return {'edges': [c for c in self.candidates(node_m, x) if c['percent_along'] == 1.]}
      elif loc['radius'] == 1 and haversine(p, node) < 0.5:
        return {'edges': [c for x in legs if x['match'] for c in self.candidates(node_m, x)]}
      elif loc['radius'] == 5:
        for x in legs:
          if haversine(p, toward(node, x['h'], 8.)) < 3.:
            return {'edges': self.candidates(node_m, x) if x['match'] else []}
    return super().answer(loc)


def turn_steps(boundary, modifier, kind='turn', total=400.):
  """steps() split at boundary m, the second starting with that maneuver."""
  out = steps((boundary, total - boundary))
  out[1]['maneuver'] = {'type': kind, 'modifier': modifier}
  return out


def junction_of(controls, kind='stop'):
  c = next(c for c in controls if c.kind == kind)
  return c, c.junction_info


def roles(j):
  return {round(a.bearing): (a.role, round(a.rel_bearing)) for a in j.approaches}


def test_g23_a_left_at_a_four_leg_junction_lists_the_exit_leg_with_role_exit():
  """The road the route leaves by isn't in trace_attributes' intersecting_edges: it is added as the exit leg (for a left
  it is the left approach, whose near lanes the left crosses first). Its inbound edge read by Q2 gives its control; one Q2
  can't match reads unknown, never none (STOP_SIGNS.md G23)."""
  legs = [leg(0.), leg(90.), leg(270., exit=True, stop=False, way=812)]
  v = StubJunction([(0., 100., {'stop': True}), (100., 400., {})], {100.: legs})
  controls, status = route_controls(v, turn_steps(100., 'left'))
  c, j = junction_of(controls)
  assert status == 'ok' and j.status == 'ok' and j.maneuver == 'left' and round(j.turn_angle) == -90
  assert roles(j) == {0: ('opposite', 0), 90: ('right', 90), 270: ('exit', -90)}
  ex = next(a for a in j.approaches if a.role == 'exit')
  assert (ex.inbound, ex.outbound, ex.valhalla_control, ex.way_id, ex.lanes_in, ex.lanes_out) == (True, True, 'none', 812, 1, 1)
  unmatched = [leg(0.), leg(90.), leg(270., exit=True, match=False)]
  _, j = junction_of(route_controls(StubJunction([(0., 100., {'stop': True}), (100., 400., {})], {100.: unmatched}),
                                    turn_steps(100., 'left'))[0])
  ex = next(a for a in j.approaches if a.role == 'exit')
  assert (ex.valhalla_control, ex.inbound, ex.lanes_in) == ('unknown', True, 0)
  # a one-way away from the junction (the route's exit edge traversable forward only): no car comes in by it, its
  # control still unknown (none to match)
  one_way = [leg(0.), leg(90.), leg(270., exit=True, drive='forward')]
  _, j = junction_of(route_controls(StubJunction([(0., 100., {'stop': True}), (100., 400., {})], {100.: one_way}),
                                    turn_steps(100., 'left'))[0])
  ex = next(a for a in j.approaches if a.role == 'exit')
  assert (ex.valhalla_control, ex.inbound, ex.outbound, ex.lanes_out) == ('unknown', False, True, 1)


def test_an_unmatched_leg_reads_unknown_never_none():
  """A leg whose inbound edge Q2 doesn't match has Valhalla control unknown: never read as no sign. A matched one with no
  sign is none, with a stop stop, and a leg cars only leave by has no inbound edge to match: unknown too (read it with
  inbound)."""
  legs = [leg(0., exit=True), leg(90., stop=True), leg(270., match=False), leg(300., drive='forward', **{'class': 'service_other'})]
  v = StubJunction([(0., 100., {'stop': True}), (100., 400., {})], {100.: legs})
  _, j = junction_of(route_controls(v, steps())[0])
  got = {round(a.bearing): (a.role, a.valhalla_control, a.inbound, a.outbound) for a in j.approaches}
  assert len(j.approaches) == 4 and got == {0: ('exit', 'none', True, True), 90: ('right', 'stop', True, True),
                                            270: ('left', 'unknown', True, True), 300: ('left', 'unknown', False, True)}
  assert all(c.kind != 'signal' for c in route_controls(v, steps())[0])


@pytest.mark.parametrize('boundary, modifier, kind, want', [(110., 'right', 'turn', 'right'), (125., 'sharp left', 'turn', 'left'),
                                                           (105., 'uturn', 'turn', 'uTurn'), (140., 'right', 'turn', 'straight'),
                                                           (100., 'straight', 'continue', 'straight'), (95., 'left', 'turn', 'left')])
def test_n8_the_maneuver_from_the_route_step_within_30_m_after_the_control(boundary, modifier, kind, want):
  """The maneuver at a control is the route step's that starts from its stop point to MANEUVER_AFTER_M 30 m past its node;
  none there is straight (STOP_SIGNS.md N8)."""
  v = StubJunction([(0., 100., {'stop': True}), (100., 400., {})], {100.: [leg(90.), leg(270.)]})
  _, j = junction_of(route_controls(v, turn_steps(boundary, modifier, kind))[0])
  assert j.maneuver == want


@pytest.mark.parametrize('lanes, want', [((1, 1), 13.0), ((2, 2), 16.7), ((3, 0), 14.85), (None, 11.0)])
def test_g15_the_cross_roads_lanes_move_the_stop_point_back(lanes, want):
  """A stop or yield's stop point is node - max(11 m, lanes_cross x 3.7 / 2 + 4.0 m + POS_ERR_M 2.0 m + 3.3 m) when the
  cross road's lanes are known: 13.0 m for 2 lanes, 16.7 m for 4 (STOP_SIGNS.md 1.2, G15). The cross road's lanes are
  its widest leg's, in and out, each way it carries matched (a one-way leg carries one); one with a direction unmatched
  isn't known, and the stop point stays."""
  if lanes is None:
    cross = [leg(90., match=False), leg(270., match=False)]
  elif lanes[1] == 0:
    cross = [leg(90., drive='forward', **{'in': 0, 'out': 3}), leg(270., drive='backward', **{'in': 3, 'out': 0})]
  else:
    cross = [leg(90., **{'in': lanes[0], 'out': lanes[1]}), leg(270., **{'in': lanes[0], 'out': lanes[1]})]
  for kind in ('stop', 'yield'):
    v = StubJunction([(0., 100., {kind: True}), (100., 400., {})], {100.: cross})
    c, j = junction_of(route_controls(v, steps())[0], kind)
    assert abs(100. - c.route_m - want) < 0.01 and abs(j.stop_point_distance - want) < 0.01 and j.stop_point_source == 'default'


def test_g15_a_signals_stop_point_never_moves(tmp_path):
  """Signals keep their stop point however wide the cross road, alone or where a stop or yield shares its node (the
  signal wins the node); a mapped line already further back than the rule stays where it is."""
  wide = [leg(90., **{'in': 2, 'out': 2}), leg(270., **{'in': 2, 'out': 2})]
  for flags in ({'node_signal': True}, {'node_signal': True, 'stop': True}, {'edge_signal': True, 'yield': True}):
    v = StubJunction([(0., 100., flags), (100., 400., {})], {100.: wide})
    controls, _ = route_controls(v, steps())
    assert [(c.kind, round(c.route_m, 1), c.junction_info) for c in controls] == [('signal', 100. - OFF, None)]
  side = {'77:f': {'kind': 'stop', 'line': list(pt(78.))}}  # the sim's line 22 m back: beyond the rule's 16.7 m
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 77}), (100., 400., {})], {100.: wide})
  c, j = junction_of(route_controls(v, steps(), tiles_dir=marked(tmp_path, side))[0])
  assert round(c.route_m, 1) == 78. - MAP_MARGIN_M and j.stop_point_source == 'mappedLine' and c.line_mapped
  side = {'77:f': {'kind': 'stop', 'line': list(pt(95.))}}  # 5 m back: the rule moves it to 16.7 m
  c, j = junction_of(route_controls(v, steps(), tiles_dir=marked(tmp_path, side))[0])
  assert round(100. - c.route_m, 1) == 16.7 and j.stop_point_source == 'mappedLine'


@pytest.mark.parametrize('ours, crossing, extra, suspected', [
  ('residential', 'secondary', {}, True), ('residential', 'tertiary', {}, True), ('residential', 'unclassified', {}, False),
  ('unclassified', 'secondary', {}, True), ('residential', 'secondary', {'node_signal': True}, False),
  ('residential', 'secondary', {'yield': True}, False)])
def test_n5_a_suspected_unmapped_stop_is_emitted_as_a_stop_control(tmp_path, ours, crossing, extra, suspected):
  """Nothing mapped on our approach (no Valhalla flag, nothing in the sidecar) where every road crossing outranks ours by two
  classes or more: a suspected unmapped stop, emitted as a stop control flagged suspected at the default stop point, its
  approaches' tiers unknown (nothing mapped there). A signal or a mapped yield there is no suspect (STOP_SIGNS.md N5)."""
  legs = [leg(90., **{'class': crossing}), leg(270., **{'class': crossing}), leg(0., exit=True, **{'class': ours})]
  v = StubJunction([(0., 100., {'road_class': ours} | extra), (100., 400., {'road_class': ours})], {100.: legs})
  controls, status = route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path))
  assert status == 'ok'
  got = [c for c in controls if c.junction_info is not None and c.junction_info.suspected]
  if not suspected:
    assert got == []
    return
  assert [(c.kind, round(c.route_m, 1)) for c in got] == [('stop', 100. - 13.)]  # the 2-lane rule moves it from 11 m
  j = got[0].junction_info
  assert j.status == 'ok' and all(a.tier == 'unknown' for a in j.approaches)


def test_n5_suspected_stops_have_a_switch():
  """SUSPECTED_STOPS (on, as the design has it) is the one constant that turns suspected stops off: until SG2 routes around
  them, each is a full stop the car makes."""
  legs = [leg(90., **{'class': 'secondary'}), leg(270., **{'class': 'secondary'})]
  v = StubJunction([(0., 100., {}), (100., 400., {})], {100.: legs})
  assert controls_module.SUSPECTED_STOPS is True and [c.kind for c in route_controls(v, steps())[0]] == ['stop']
  with patch.object(controls_module, 'SUSPECTED_STOPS', False):
    assert route_controls(v, steps())[0] == []


def test_n5_driveways_and_alleys_are_no_crossing_roads():
  """Only roads count as crossing ones: an alley or a driveway beside our residential street at a secondary doesn't make it
  less suspect, and a residential street crossing makes it no suspect."""
  base = [leg(90., **{'class': 'secondary'}), leg(270., **{'class': 'secondary'})]
  alley = base + [leg(100., use='alley', **{'class': 'service_other'})]
  v = StubJunction([(0., 100., {}), (100., 400., {})], {100.: alley})
  assert [(c.kind, c.junction_info.suspected) for c in route_controls(v, steps())[0]] == [('stop', True)]
  across = [leg(90., **{'class': 'secondary'}), leg(270., **{'class': 'residential'})]
  v = StubJunction([(0., 100., {}), (100., 400., {})], {100.: across})
  assert route_controls(v, steps())[0] == []


def kinds(controls):
  """(kind, its edge, suspected) per control."""
  return [(c.kind, c.edge, bool(c.junction_info and c.junction_info.suspected)) for c in controls]


def primary_cross(over=None):
  """Legs at a tertiary's junction with a primary road crossing it (left and right), the route straight on; over: {heading:
  leg fields}."""
  over = over or {}
  return [leg(90., **({'class': 'primary'} | over.get(90, {}))), leg(270., **({'class': 'primary'} | over.get(270, {}))),
          leg(0., exit=True, **{'class': 'tertiary'})]


@pytest.mark.parametrize('name, approach, extra, want', [
  # Indianapolis route 60 (and 66): OSM maps the junction's signal on our approach's stop-line node, 13 m before it,
  # a node no other road meets
  ('a signal on the stop-line node', [(0., 87., {'node_signal': True, 'crossing': []}), (87., 100., {})], {}, [('signal', 0, False)]),
  ('a directed signal there', [(0., 88., {'edge_signal': True, 'crossing': []}), (88., 100., {})], {}, [('signal', 0, False)]),
  # routes 33, 54 and 59: Valhalla's stop on an edge ending 8 m short of the junction, with nothing in the sidecar
  ('a stop at a node short of it', [(0., 92., {'stop': True, 'crossing': []}), (92., 100., {})], {}, [('stop', 0, False)]),
  ('a yield there', [(0., 92., {'yield': True, 'crossing': []}), (92., 100., {})], {}, [('yield', 0, False)]),
  # an alley or a driveway is no junction: the approach goes on back past it
  ('a signal back past an alley', [(0., 75., {'node_signal': True, 'crossing': []}), (75., 90., {}), (90., 100., {})],
   {90.: [leg(90., use='alley', **{'class': 'service_other'})]}, [('signal', 0, False)]),
  # the approach ends at the road junction before (40 m back) or JUNCTION_AHEAD_M back: what is mapped there isn't ours
  ('a signal at the junction before', [(0., 50., {'node_signal': True, 'crossing': []}), (50., 60., {}), (60., 100., {})],
   {60.: [leg(90.), leg(270.)]}, [('signal', 0, False), ('stop', 2, True)]),
  ('a signal 70 m back', [(0., 30., {'node_signal': True, 'crossing': []}), (30., 100., {})], {}, [('signal', 0, False), ('stop', 1, True)]),
  ('nothing', [(0., 87., {'crossing': []}), (87., 100., {})], {}, [('stop', 1, True)]),
])
def test_n5_anything_mapped_on_the_approach_means_no_suspected_stop(name, approach, extra, want):
  """N5's 'nothing mapped on our approach' holds over the whole approach: the junction's own edge and each one before it
  back to the previous road junction or JUNCTION_AHEAD_M (an alley, a driveway, a crosswalk or a signal's stop-line node is
  no junction). A signal, stop or yield anywhere on it means the junction is mapped, so no suspected stop: the car never
  stops at a green for one (Indianapolis route 60: a signal 13 m short of a primary, the suspected stop at its line)."""
  edges = [(b, e, {'road_class': 'tertiary'} | f) for b, e, f in approach] + [(100., 400., {'road_class': 'tertiary'})]
  v = StubJunction(edges, {100.: primary_cross()} | extra)
  controls, status = route_controls(v, steps())
  assert status == 'ok' and kinds(controls) == want, name


def test_n5_a_signal_on_the_stop_line_node_of_a_left_onto_a_major_road():
  """The left of the same shape (the skeptic's probe): a residential street's signal 20 m short of a secondary it turns left
  onto, the junction's crossing roads all secondaries."""
  legs = [leg(90., **{'class': 'secondary'}), leg(0., **{'class': 'secondary'}), leg(270., exit=True, **{'class': 'secondary'})]
  v = StubJunction([(0., 80., {'node_signal': True, 'crossing': []}), (80., 100., {}), (100., 400., {})], {100.: legs})
  assert kinds(route_controls(v, turn_steps(100., 'left'))[0]) == [('signal', 0, False)]


def test_n5_a_sidecar_stop_short_of_its_junction_is_its_only_stop(tmp_path):
  """Indianapolis node 1710705057's shape at a secondary: the sidecar's stop is on the edge ending at a crosswalk node 16 m
  short of the junction it governs. That junction has its mapped stop and no second, suspected one 14 m on."""
  edges = [(0., 84., {'way_id': 50, 'crossing': []}), (84., 100., {'way_id': 50}), (100., 400., {})]
  side = {'50:f': [entry('stop', 80., junction=900)]}
  at = {'900': {'pos': list(pt(100.)), 'approaches': []}}
  v = StubJunction(edges, four({90: {'class': 'secondary'}, 270: {'class': 'secondary'}}), every=2.)
  controls, _ = route_controls(v, steps(every=2.), sidecar_dir=osm_dir(tmp_path, side, at))
  assert kinds(controls) == [('stop', 0, False)]
  assert haversine((controls[0].junction_info.lat, controls[0].junction_info.lon), pt(100.)) < 0.5


@pytest.mark.parametrize('name, fields, want', [
  ('a signal on the stop-line node', {'node_signal': True}, [('signal', 0, False)]),
  ('nothing mapped', {}, [('stop', 1, True)]),
  ('a stop at the near carriageway', None, [('stop', 1, False)]),
])
def test_n5_one_junction_across_a_median_valhalla_does_not_flag(name, fields, want):
  """Indianapolis routes 60 and 73: a divided primary whose median edge a node splits (5 m and 5 m), so Valhalla flags
  neither half internal. The far carriageway 10 m on is the same junction: no suspected stop of its own, whether the near
  one is signalled, mapped, or suspected itself (one stop, never two 10 m apart)."""
  near = {'road_class': 'tertiary'} | ({'stop': True} if fields is None else {})
  edges = [(0., 87., {'road_class': 'tertiary', 'crossing': []} | (fields or {})), (87., 100., near),
           (100., 105., {'road_class': 'tertiary', 'crossing': []}), (105., 110., {'road_class': 'tertiary'}),
           (110., 400., {'road_class': 'tertiary'})]
  carriageways = {100.: primary_cross({90: {'drive': 'forward'}, 270: {'drive': 'backward'}}),
                  110.: primary_cross({90: {'drive': 'backward'}, 270: {'drive': 'forward'}})}
  v = StubJunction(edges, carriageways)
  assert kinds(route_controls(v, steps())[0]) == want, name


def test_n5_through_a_green_without_slowing_where_the_signal_is_on_the_approach_node():
  """Closed loop (map_controls' point-mass car, the sim's green): what navd sends for route 60's junction, a signal on our
  approach's stop-line node 13 m short of a primary, takes the car through at its speed, never stopped at the line."""
  from openpilot.common.realtime import DT_MDL
  from openpilot.selfdrive.turnpilot.tests.test_map_controls import Car, oracle
  edges = [(0., 87., {'road_class': 'tertiary', 'node_signal': True, 'crossing': []}), (87., 100., {'road_class': 'tertiary'}),
           (100., 400., {'road_class': 'tertiary'})]
  controls, _ = route_controls(StubJunction(edges, {100.: primary_cross()}), steps())
  car = Car([(120. + c.route_m, c.kind, c.line_mapped, c.junction, c.cross_suspect) for c in controls])
  v0 = car.v
  for _ in range(int(25. / DT_MDL)):
    _, dec, _ = car.step(lights=oracle('green'))
    assert not dec.should_stop
  assert car.s > 120. + 150. and min(r[2] for r in car.rows) > v0 - 0.1


class StubBehind(StubJunction):
  """StubJunction with the road behind the route's start, due north like the route: the first edge really begins at begin
  m (0 or less) and behind [(begin m, end m, fields)] runs up to it (fields: node_signal and edge_signal at its end node,
  crossing: the classes of other roads meeting there, none by default). locate (radius 1) at a point on the first edge
  answers it with its whole shape from begin; at a node behind (the end of a behind edge), the edges there: the one
  arriving, the one leaving, both reversed and each crossing road both ways, with the node's traffic_signal (found False:
  nothing at a node behind); anything else as StubJunction."""

  def __init__(self, edges, junctions, begin, behind, found=True, **kw):
    super().__init__(edges, junctions, **kw)
    self.begin, self.behind, self.found = begin, behind, found

  @staticmethod
  def cand(eid, b, e, pa, heading, fields=None, way=60):
    fields = fields or {}
    shape = [{'lat': pt(x)[0], 'lon': pt(x)[1]} for x in ((b, e) if heading == 0. else (e, b))]
    return {'edge_id': {'value': eid}, 'percent_along': pa, 'heading': heading, 'distance': 0.,
            'edge': {'forward': heading == 0., 'traffic_signal': bool(fields.get('edge_signal')) and pa == 1. and heading == 0.,
                     'classification': {'classification': fields.get('class', 'tertiary'), 'use': fields.get('use', 'road')}},
            'edge_info': {'way_id': way, 'shape': shape}}

  def answer(self, loc):
    m = (loc['lat'] - LAT0) * M_PER_DEG
    first = self.edges[0][0]
    end0 = first['end_shape_index'] * self.every
    if loc.get('radius') == 1 and 'heading' not in loc and m < 0.5:
      for k, (b, e, f) in enumerate(self.behind):
        if abs(m - e) < 0.5:
          if not self.found:
            return {'edges': [], 'nodes': []}
          nb, ne = (self.behind[k + 1][:2] if k + 1 < len(self.behind) else (self.begin, end0))
          out = [self.cand(5000 + k, b, e, 1., 0., f), self.cand(5000 + k, b, e, 0., 180.),
                 self.cand(5001 + k if k + 1 < len(self.behind) else first['id'], nb, ne, 0., 0.),
                 self.cand(5001 + k if k + 1 < len(self.behind) else first['id'], nb, ne, 1., 180.)]
          for c in f.get('crossing', []):
            for h in (90., 270.):
              x = toward(pt(e), h, 50.)
              out += [{'edge_id': {'value': 7000 + int(h)}, 'percent_along': pa, 'heading': h if pa == 0. else (h + 180.) % 360.,
                       'distance': 0., 'edge': {'classification': {'classification': c, 'use': 'road'}},
                       'edge_info': {'way_id': 71, 'shape': [{'lat': q[0], 'lon': q[1]} for q in ((pt(e), x) if pa == 0. else (x, pt(e)))]}}
                      for pa in (0., 1.)]
          return {'edges': out, 'nodes': [{'lat': pt(e)[0], 'lon': pt(e)[1], 'traffic_signal': bool(f.get('node_signal'))}]}
    out = super().answer(loc)
    if loc.get('radius') == 1 and 'heading' not in loc and 0. < m < end0:
      for c in out.get('edges') or []:
        if (c.get('edge_id') or {}).get('value') == first['id']:
          c |= self.cand(first['id'], self.begin, end0, (m - self.begin) / (end0 - self.begin), 0., way=first['way_id'])
          c['edge'] = {**c['edge'], 'stop_sign': out['edges'][0]['edge'].get('stop_sign', False)}
    return out


def pg_road(stub, behind, found=True, begin=-1.):
  """A tertiary whose route or lookup starts 1 m past a node (begin), the first edge ending at a primary stub m on from
  that node, the road behind it behind."""
  edges = [(0., stub + begin, {'road_class': 'tertiary'}), (stub + begin, 400., {'road_class': 'tertiary'})]
  return StubBehind(edges, {stub + begin: primary_cross()}, begin, behind, found=found, every=1.)


@pytest.mark.parametrize('ahead', [False, True])
@pytest.mark.parametrize('stub', [13., 20., 24.])
@pytest.mark.parametrize('name, behind', [
  ('a signal on the node', [(-80., -1., {'node_signal': True})]),
  ('a directed signal there', [(-80., -1., {'edge_signal': True})]),  # Indianapolis route 1's 11.8 m stub is mapped so
  ('a signal back past a crosswalk node', [(-80., -30., {'node_signal': True}), (-30., -1., {})]),
])
def test_pg_a_route_or_lookup_starting_past_a_signals_stop_line_node_sees_the_signal(name, behind, stub, ahead):
  """A route (a reroute starts at the car too) or a lookup ahead (controls_ahead's road starts at the car) starting 1 m past
  the stop-line node OSM maps a junction's signal on, stub m short of a primary: approach_edges reaches the trace's first
  edge with nothing mapped and the signal just behind the start. signal_behind walks back from that edge's begin node and
  finds it: no suspected stop, so no stop on a green (it was a suspected stop 0, 6 and 10 m ahead; on Indianapolis route 1,
  a lookup 1.5 m past the node of an 11.8 m stub gave one 0 m ahead)."""
  controls, status = controls_module._route_controls(pg_road(stub, behind), steps(every=1.), None, ahead=ahead)
  assert status == 'ok' and kinds(controls) == [], name


@pytest.mark.parametrize('name, behind, found', [
  ('nothing mapped back to a road junction', [(-80., -40., {'crossing': ['residential']}), (-40., -1., {})], True),
  ('a signal at the road junction behind is its own', [(-80., -40., {'node_signal': True, 'crossing': ['residential']}), (-40., -1., {})], True),
  ('a signal past JUNCTION_AHEAD_M', [(-120., -45., {'node_signal': True}), (-45., -1., {})], True),
  ('the road behind not found', [(-80., -1., {'node_signal': True})], False),
])
@pytest.mark.parametrize('ahead', [False, True])
def test_pg_the_suspected_stop_stays_with_nothing_mapped_behind_the_start_or_the_road_there_unknown(name, behind, found, ahead):
  """The same look behind the start keeps a real suspected stop: one the car is already inside the approach of (a lookup
  ahead every second must not drop it as the car closes in), one whose signal behind belongs to the road junction before,
  or past JUNCTION_AHEAD_M, and one whose road behind the start can't be read, logged (the safer side: a stop on a green
  is a -2.0 m/s^2 stop and a handback, rolling through an unmapped stop onto a primary isn't)."""
  with patch.object(controls_module, 'cloudlog') as log:
    controls, status = controls_module._route_controls(pg_road(20., behind, found), steps(every=1.), None, ahead=ahead)
  assert status == 'ok' and kinds(controls) == [('stop', 0, True)], name
  assert found or any('unknown' in str(c.args[0]) for c in log.warning.call_args_list)


def test_pg_a_lookup_ahead_reads_the_road_behind_once():
  """Off the route navd looks again every second: the look behind the start is kept in the LocateCache by the first edge
  and the junction's edge, so the next lookup asks nothing more for it."""
  v = pg_road(20., [(-80., -1., {'node_signal': True})])
  cache = controls_module.LocateCache()
  assert kinds(controls_module._route_controls(v, steps(every=1.), None, ahead=True, cache=cache)[0]) == []
  n = len(v.requests)
  assert kinds(controls_module._route_controls(v, steps(every=1.), None, ahead=True, cache=cache)[0]) == []
  assert [r for e, r in v.requests[n:] if e == 'locate'] == []


def test_pg_through_a_green_without_slowing_from_a_route_starting_past_the_stop_line_node():
  """Closed loop (map_controls' point-mass car): a route that starts 1 m past the stop-line node of a signal 20 m short of a
  primary (the light green, its line behind the car) takes the car through the junction at its speed; the suspected stop
  it had 6 m ahead braked it at -2.0 m/s^2 into cannotStopComfortably and stopOverrun."""
  from openpilot.common.realtime import DT_MDL
  from openpilot.selfdrive.turnpilot.tests.test_map_controls import Car
  controls, _ = route_controls(pg_road(20., [(-80., -1., {'node_signal': True})]), steps(every=1.))
  car = Car([(c.route_m, c.kind, c.line_mapped, c.junction, c.cross_suspect) for c in controls], v=11.)
  for _ in range(int(10. / DT_MDL)):
    _, dec, _ = car.step()
    assert not dec.should_stop
  assert car.s > 100. and min(r[2] for r in car.rows) > 11. - 0.1


def rail_dir(tmp_path, rail, controls=None):
  d = osm_dir(tmp_path, controls)
  cell = level2_index(LON0, LAT0)
  shard = json.loads((d / 'osm_controls' / f'{cell}.json').read_text())
  shard['rail'] = [{'lat': la, 'lon': lo, 'ways': [1]} for la, lo in rail]
  (d / 'osm_controls' / f'{cell}.json').write_text(json.dumps(shard))
  return d


def test_c7_rail_distance_from_the_sidecar(tmp_path):
  """The distance to the nearest mapped rail level crossing, from the OSM sidecar (Valhalla has no rail node type): along
  the route for one on the control's own edge however far back, else straight-line from the node within RAIL_M 30 m;
  -1 none; -2 unknown without a current sidecar, never none (STOP_SIGNS.md C7)."""
  v = StubJunction([(0., 100., {'stop': True}), (100., 400., {})], {100.: [leg(90.), leg(270.)]})
  cases = {'beside': ([toward(pt(100.), 90., 20.)], 20.), 'on our edge': ([pt(55.)], 45.), 'far': ([toward(pt(100.), 90., 40.)], -1.),
           'both': ([toward(pt(100.), 90., 20.), pt(88.)], 12.), 'none': ([], -1.)}
  for name, (rail, want) in cases.items():
    _, j = junction_of(route_controls(v, steps(), sidecar_dir=rail_dir(tmp_path / name, rail))[0])
    assert abs(j.rail_distance - want) < 0.5, name
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path / 'stale', stamp='2026-01-01T00:00:00Z'))[0])
  assert j.rail_distance == -2. and j.status == 'failed'
  _, j = junction_of(route_controls(v, steps())[0])  # no sidecar expected (synthetic): unknown all the same
  assert j.rail_distance == -2.


def allway(tmp_path, ours=None, others=None, record=None, maxspeed=None):
  """A sidecar for a four-leg junction at the stub route's node 100 m (OSM node 900): our approach (way 50 northbound) and
  legs at 0 (way 700), 90 (way 790) and 270 (way 970), each arriving forward along its way. ours, others: each approach's
  (stops, evidence, stop_tag) as the sidecar's junction record and controls give it."""
  ours = ours or (True, 'node_all', 'all')
  others = others or {700: ours, 790: ours, 970: ours}
  bearings = {50: 180., 700: 0., 790: 90., 970: 270.}
  approaches, controls = [], {}
  for way, (stops, evidence, tag) in {50: ours, **others}.items():
    approaches.append({'way_id': way, 'dir': 'f', 'bearing_deg': bearings[way], 'class': 'residential', 'maxspeed': maxspeed,
                       'oneway': False, 'stops': stops, 'kind': 'stop' if stops else None, 'evidence': evidence if stops else 'none',
                       'line': None})
    if stops:
      line = None if evidence.startswith('node') else list(pt(92.)) if way == 50 else list(toward(pt(100.), bearings[way], 8.))
      controls[f'{way}:f'] = [{'kind': 'stop', 'line': line, 'junction_node': 900, 'stop_tag': tag, 'evidence': evidence}]
  rec = {'900': {'pos': list(pt(100.)), 'approaches': record if record is not None else approaches}}
  return osm_dir(tmp_path, controls, rec)


def four(over=None):
  """Legs at the stub route's node 100 m: straight on (way 700, the road the route leaves by), right (790), left (970);
  over: {heading: leg fields}."""
  over = over or {}
  return {100.: [leg(0., way=700, exit=True, **over.get(0, {})), leg(90., way=790, **over.get(90, {})),
                 leg(270., way=970, **over.get(270, {}))]}


def tiers(j):
  return {round(a.bearing): a.tier for a in j.approaches}


@pytest.mark.parametrize('name, ours, others, want', [
  ('stop=all on the node', (True, 'node_all', 'all'), None, 'mapStrong'),
  ('directed signs', (True, 'mid_directed', 'none'), None, 'mapStrong'),
  ('plain node stop', (True, 'node_untagged', 'none'), None, 'mapWeak'),
  ('undirected', (True, 'mid_undirected', 'none'), None, 'mapWeak'),
])
def test_tiers_of_a_mapped_all_way(tmp_path, name, ours, others, want):
  """Each approach's tier (STOP_SIGNS.md 4.1): a directed stop or stop=all is mapStrong, a plain junction-node stop or an
  undirected one mapWeak; none of them stops this week, only surveyed does."""
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four())
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path, ours, others))[0])
  assert j.osm_node_id == 900 and j.sidecar and len(j.approaches) == 3 and tiers(j) == {0: want, 90: want, 270: want}, name
  assert {a.osm_stop for a in j.approaches} == {'all' if ours[2] == 'all' else 'stop'}


def test_no_wins_over_unknown_at_a_two_way_and_stop_minor(tmp_path):
  """A plain two-way: the cross approaches the sidecar shows unstopped are no, never unknown, since ours stops; so is one
  Q2 couldn't match (its Valhalla control unknown, its tier still no from the sidecar's record by bearing). stop=minor
  anywhere makes the stopping approaches mapWeak and the rest no; stop=all beside an unstopped approach contradicts:
  mapWeak."""
  unstopped = (False, 'none', 'none')
  two_way = {700: (True, 'mid_directed', 'none'), 790: unstopped, 970: unstopped}
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four({90: {'match': False}}))
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path / 'two', (True, 'mid_directed', 'none'), two_way))[0])
  assert tiers(j) == {0: 'mapStrong', 90: 'no', 270: 'no'}
  right = next(a for a in j.approaches if round(a.bearing) == 90)
  assert (right.valhalla_control, right.osm_stop, right.tier) == ('unknown', 'none', 'no')
  minor = {700: (True, 'mid_directed', 'minor'), 790: unstopped, 970: unstopped}
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four())
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path / 'minor', (True, 'mid_directed', 'minor'), minor))[0])
  assert tiers(j) == {0: 'mapWeak', 90: 'no', 270: 'no'}
  three = {700: (True, 'mid_directed', 'all'), 790: (True, 'mid_directed', 'all'), 970: unstopped}
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path / 'contra', (True, 'mid_directed', 'all'), three))[0])
  assert tiers(j) == {0: 'mapWeak', 90: 'mapWeak', 270: 'no'}


def test_without_a_current_sidecar_tiers_are_unknown_or_a_valhalla_flag(tmp_path):
  """No sidecar, or a stale one: an approach Valhalla flags is mapWeak (a Valhalla flag alone), every other unknown, never
  no (Valhalla misses a quarter of the stops: its silence proves nothing)."""
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four({0: {'stop': True}}))
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path, shards=False))[0])
  assert tiers(j) == {0: 'mapWeak', 90: 'unknown', 270: 'unknown'} and not j.sidecar and j.status == 'failed'
  assert {a.osm_stop for a in j.approaches} == {'unknown'}
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path / 'ok'))[0])  # current, nothing at the node
  assert tiers(j) == {0: 'mapWeak', 90: 'no', 270: 'no'} and {a.osm_stop for a in j.approaches} == {'none'}


@pytest.mark.parametrize('kph, maxspeed, cls, source, mph', [(48, None, 'residential', 'tagged', 48 / 3.6 / 0.44704),
                                                            (None, 11.176, 'residential', 'tagged', 25.),
                                                            (None, None, 'residential', 'classDefault', 30.),
                                                            (None, None, 'tertiary', 'classDefault', 35.),
                                                            (None, None, 'secondary', 'classDefault', 45.),
                                                            (None, None, 'primary', 'classDefault', 55.)])
def test_limits_tagged_else_the_class_default(tmp_path, kph, maxspeed, cls, source, mph):
  """An approach's assumed limit: the survey's posted sign, else the tagged maxspeed (Valhalla's speed_limit, else the
  sidecar's), else decision D8's class default (30 mph residential, 35 tertiary, 45 secondary, 55 primary and above),
  never Valhalla's classified speed."""
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})],
                   four({90: {'kph': kph, 'class': cls}, 0: {'class': cls}, 270: {'class': cls}}))
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path, maxspeed=maxspeed))[0])
  right = next(a for a in j.approaches if round(a.bearing) == 90)
  assert right.speed_source == source and abs(right.assumed_limit - mph * 0.44704) < 0.01
  assert right.tagged_limit == (0. if source == 'classDefault' else right.assumed_limit)


def survey(node=900, date='2026-10-20', stops=(True, True, True, True), posted=20, stop_point=None, confirmed=(), lane_edge=4.0):
  """A survey of the stub junction (OSM node node): our approach (way 50, heading 0 toward it) and the legs' (700 heading
  180, 790 heading 270, 970 heading 90), in person."""
  ways = [(50, 0.), (700, 180.), (790, 270.), (970, 90.)]
  return {'osmNodeId': node, 'lat': pt(100.)[0], 'lon': LON0, 'date': date, 'source': 'inPerson', 'confirmedBy': list(confirmed),
          'approaches': [{'wayId': w, 'bearingDeg': b, 'stops': st, 'plaque': st, 'postedMph': posted, 'laneEdgeM': lane_edge, 'lineM': 7.,
                          'nearestParkingM': {'left': 40., 'right': 40.}, 'sightM': 60.} | ({'stopPointM': stop_point} if w == 50 and stop_point else {})
                         for (w, b), st in zip(ways, stops, strict=True)],
          'validations': []}


def context(entries=(), overrides=None, ok=True, error='', today=(2026, 10, 30)):
  from datetime import date

  from openpilot.selfdrive.turnpilot.maps import surveys
  from openpilot.selfdrive.turnpilot.routing.junctions import Context
  loaded = []
  for e in entries:
    assert surveys.problem(e) == '', surveys.problem(e)
    loaded.append(dict(e, approaches=[dict(a, stopPointM=a.get('stopPointM')) for a in e['approaches']]))
  return Context(loaded, overrides or {}, ok, error, date(*today))


def test_a_counting_survey_sets_the_tiers_the_posted_speed_and_the_stop_point(tmp_path):
  """A survey matched by OSM node id and dated within a year: each approach it records stopping is surveyed (one it records
  unstopped is no), its posted sign is the assumed limit (source survey), and our approach's stopPointM is the stop point
  (source survey), closer than the 2-lane rule's 13 m since it is checked against the surveyed edge."""
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four())
  ctx = context([survey(stops=(True, True, True, False), stop_point=9.8)])
  c, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path), context=ctx)[0])
  assert j.surveyed and j.survey['osmNodeId'] == 900
  assert tiers(j) == {0: 'surveyed', 90: 'surveyed', 270: 'no'}
  assert {(a.speed_source, round(a.assumed_limit / 0.44704)) for a in j.approaches} == {('survey', 20)}
  assert (j.stop_point_source, round(j.stop_point_distance, 1), round(100. - c.route_m, 1)) == ('survey', 9.8, 9.8)


def test_a_survey_over_a_year_old_or_of_another_node_is_ignored(tmp_path):
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four())
  for name, entry in {'old': survey(date='2025-10-29'), 'another node': survey(node=901)}.items():
    c, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path / name), context=context([entry]))[0])
    assert not j.surveyed and tiers(j) == {0: 'mapStrong', 90: 'mapStrong', 270: 'mapStrong'}, name
    assert j.stop_point_source == 'default' and {a.speed_source for a in j.approaches} == {'classDefault'}, name
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path / 'again'), context=context([survey(date='2025-10-29')]))[0])
  assert j.survey is not None and 'year' in j.survey_note  # matched, said why it doesn't count


def test_the_plausibility_gate_needs_two_looks_at_a_primary_beside_a_minor_road(tmp_path):
  """A primary road meeting one two or more classes lower is implausible as an all-way (about 1% of such stop junctions,
  STOP_SIGNS.md 4.2): its survey counts only with both looks (in person and street imagery) in confirmedBy."""
  primary = four({0: {'class': 'primary'}})
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], primary)
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path / 'one'), context=context([survey()]))[0])
  assert not j.plausible and not j.surveyed and 'implausible' in j.survey_note and 'surveyed' not in tiers(j).values()
  both = survey(confirmed=('inPerson', 'streetImagery'))
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path / 'two'), context=context([both]))[0])
  assert not j.plausible and j.surveyed and set(tiers(j).values()) == {'surveyed'}
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four({0: {'class': 'tertiary'}}))
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path / 'tertiary'), context=context([survey()]))[0])
  assert j.plausible and j.surveyed


def test_an_override_makes_its_approach_no_and_an_unreadable_one_is_loud(tmp_path):
  """TurnPilotJunctionOverrides (an approach downgraded on site, or a signal seen there) puts its approach in the no tier,
  over a survey. A param this build doesn't know, or one that doesn't parse, is logged and said: every stop and yield
  junction's status is failed and the route's controls status partial (the UI's "Map stops/signals: partial")."""
  from openpilot.common.params import UnknownKeyName
  from openpilot.selfdrive.turnpilot.routing.junctions import read_overrides
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 50}), (100., 400., {})], four())
  params = {'TurnPilotJunctionOverrides': json.dumps([{'approach': '900:790:f', 'kind': 'no', 'evidence': {'speed': 8.2}},
                                                      {'approach': '900:970:f', 'kind': 'signalSeen', 'evidence': None}])}
  overrides, ok, _ = read_overrides(SimpleNamespace(get=params.get))
  assert ok and set(overrides) == {'900:790:f', '900:970:f'}
  ctx = context([survey()], overrides)
  _, j = junction_of(route_controls(v, steps(), sidecar_dir=allway(tmp_path / 'o'), context=ctx)[0])
  assert tiers(j) == {0: 'surveyed', 90: 'no', 270: 'no'} and j.status == 'ok'
  # and says so, for certify's override reason (SG2)
  assert {round(a.bearing) for a in j.approaches if a.overridden} == {90, 270}

  def unknown(key):
    raise UnknownKeyName(key)
  # the last a list, what a JSON key's get gives (it reads a value that doesn't parse as unset): not the STRING key's text
  for bad in (SimpleNamespace(get=unknown), SimpleNamespace(get=lambda k: '{"not": "a list"}'),
              SimpleNamespace(get=lambda k: json.dumps([{'approach': '900:790:f', 'kind': 'maybe'}])),
              SimpleNamespace(get=lambda k: [{'approach': '900:790:f', 'kind': 'no'}])):
    overrides, ok, why = read_overrides(bad)
    assert not ok and overrides == {} and 'TurnPilotJunctionOverrides' in why
    with patch.object(controls_module, 'cloudlog') as log:
      controls, status = route_controls(v, steps(), sidecar_dir=allway(tmp_path / 'bad'), context=context(ok=False, error=why))
    assert status == 'partial' and [c.junction_info.status for c in controls] == ['failed'] and controls[0].junction_status == 'failed'
    assert any('TurnPilotJunctionOverrides' in str(c.args[0]) for c in log.error.call_args_list)
  assert read_overrides(SimpleNamespace(get=lambda k: None)) == ({}, True, '')


def opposing_steps():
  """steps() whose second step starts with a left at 200 m, where a road carries on straight across (bearing 0)."""
  out = steps()
  lon, lat = out[1]['geometry']['coordinates'][0]
  out[1]['maneuver'] = {'type': 'turn', 'modifier': 'left', 'bearing_before': 0, 'bearing_after': 270}
  out[1]['intersections'] = [{'location': [lon, lat], 'bearings': [0, 90, 180, 270], 'in': 2, 'out': 3}]
  return out


def test_q2_rides_in_the_opposing_roads_batched_locate():
  """e7af1fb's one batched locate per route (the opposing roads, for the unprotected left's R_req) carries the junctions'
  probes too: a route makes Q1 and one Q2, and the opposing roads read there are opposing_roads' own answer."""
  from openpilot.selfdrive.turnpilot.routing.controls import Opposing, opposing_roads
  junctions = {100.: [leg(90.), leg(270.)], 200.: [leg(0., kph=56, **{'in': 2}), leg(90.), leg(270., exit=True)]}
  v = StubJunction([(0., 100., {'stop': True}), (100., 200., {'yield': True}), (200., 400., {})], junctions)
  alone = opposing_roads(v, opposing_steps())
  assert alone == {0: (56 / 3.6, 2)}
  v.requests.clear()
  out = Opposing()
  controls, status = route_controls(v, opposing_steps(), opposing=out)
  assert status == 'ok' and out.roads == alone and [c.kind for c in controls] == ['stop', 'yield']
  locates = [r for e, r in v.requests if e == 'locate']
  assert len(locates) == 2 and 'heading' in locates[1]['locations'][0]  # Q1, then Q2 led by the opposing probe
  assert all(c.junction_info.status == 'ok' for c in controls)
  # unsupported tiles: no batched locate ran, the roads stay None (ValhallaSource then reads them alone)
  out = Opposing()
  assert route_controls(v, opposing_steps(), tiles_dir='/nonexistent', opposing=out) == ([], 'unsupported') and out.roads is None


def test_q2_failing_leaves_every_leg_unknown_and_the_junction_failed():
  calls = []
  inner = StubJunction([(0., 100., {'stop': True}), (100., 400., {})], {100.: [leg(90.), leg(270.)]})

  def call(endpoint, req):
    calls.append(endpoint)
    if endpoint == 'locate' and calls.count('locate') == 2:
      raise ConnectionError("valhalla_service went away")
    return inner(endpoint, req)
  from openpilot.selfdrive.turnpilot.routing.controls import Opposing
  out = Opposing()
  controls, status = route_controls(call, opposing_steps(), opposing=out)
  assert status == 'ok' and [c.kind for c in controls] == ['stop'] and out.roads == {}
  j = controls[0].junction_info
  assert j.status == 'failed' and controls[0].junction_status == 'failed' and {a.valhalla_control for a in j.approaches} == {'unknown'}


def test_navds_own_control_carries_the_approach_key_and_exclusion_point():
  """For SG2's avoidance loop (navd only, not on the bus): the approach edge's key (way, direction, node), its exclusion
  point min(12 m, half the edge) back from the node with the route's bearing there, and whether the edge is the route's
  first or last."""
  v = StubJunction([(0., 100., {'stop': True, 'way_id': 61}), (100., 116., {'yield': True, 'way_id': 62, 'forward': False}),
                    (116., 300., {'stop': True, 'way_id': 63}), (300., 400., {})], {}, every=2.)
  controls, _ = route_controls(v, steps(every=2.))
  got = [(c.kind, c.approach_key[:2], c.first_edge, c.last_edge) for c in controls]
  assert got == [('stop', (61, True), True, False), ('yield', (62, False), False, False), ('stop', (63, True), False, False)]
  assert controls[0].approach_key[2] == tuple(round(x, 6) for x in pt(100.))
  for c, node, back in zip(controls, (100., 116., 300.), (12., 8., 12.), strict=True):
    lat, lon, b = c.exclusion
    assert abs(haversine((lat, lon), pt(node - back))) < 0.3 and angle(b, 0.) < 1.


def angle(a, b):
  return abs((a - b + 180.) % 360. - 180.)


class StubRoadJunction(StubRoad, StubJunction):
  """StubRoad's road ahead (locate by heading) with StubJunction's junctions."""


def test_controls_ahead_lists_the_straight_on_leg_with_role_exit():
  """Off the route the maneuver is unknown and the exit leg is the road the car carries on onto: the next edge of the road
  looked up, or, at its end (where intersecting_edges lists it), the leg nearest straight on."""
  junctions = {100.: [leg(90.), leg(270.)], 400.: [leg(2., way=744), leg(90.), leg(270.)]}
  v = StubRoadJunction([(0., 100., {'stop': True}), (100., 250., {}), (250., 400., {'stop': True})],
                       road=[(0., 100.), (100., 250.), (250., 400.)], junctions=junctions)
  controls, status, _ = controls_ahead(v, pt(0.), 3.)
  assert status == 'ok' and [round(c.route_m) for c in controls] == [87, 387]  # the 2-lane cross roads' 13 m
  first, last = (c.junction_info for c in controls)
  assert first.maneuver == last.maneuver == 'unknown'
  assert roles(first) == {0: ('exit', 0), 90: ('right', 90), 270: ('left', -90)}
  assert roles(last) == {2: ('exit', 2), 90: ('right', 90), 270: ('left', -90)}
  assert next(a for a in last.approaches if a.role == 'exit').way_id == 744


def test_controls_ahead_keeps_q1_and_q2_by_edge_id():
  """navd looks up the road ahead every second: with a LocateCache (one per tile set) the next lookup asks Valhalla again
  only for the road itself (locate by heading), not for the edges' signs (Q1) nor the junctions (Q2) it already has."""
  from openpilot.selfdrive.turnpilot.routing.controls import LocateCache
  junctions = {100.: [leg(90.), leg(270.)]}
  v = StubRoadJunction([(0., 100., {'stop': True}), (100., 250., {}), (250., 400., {})], road=[(0., 100.), (100., 250.), (250., 400.)],
                       junctions=junctions)
  cache = LocateCache()
  first = controls_ahead(v, pt(0.), 3., cache=cache)
  v.requests.clear()
  again = controls_ahead(v, pt(5.), 3., cache=cache)
  plain = [r for e, r in v.requests if e == 'locate' and 'heading' not in r['locations'][0]]
  assert plain == [] and [c.kind for c in again[0]] == [c.kind for c in first[0]] == ['stop']
  assert roles(again[0][0].junction_info) == roles(first[0][0].junction_info)
  v.requests.clear()
  controls_ahead(v, pt(5.), 3.)  # no cache: both asked again
  assert len([r for e, r in v.requests if e == 'locate' and 'heading' not in r['locations'][0]]) == 2


def test_a_stop_before_a_crosswalk_node_is_read_at_the_junction_it_governs(tmp_path):
  """Indianapolis, OSM node 1710705057: Valhalla splits our approach at a crosswalk node 16 m before the junction, and the
  sidecar's stop line lands on the edge ending there, where no other road meets. The junction is read at the node the
  sidecar says the stop governs, further on the route (within JUNCTION_AHEAD_M), with its legs; the stop point never moves
  forward for it (here it stays 27 m back, past the 2-lane rule's 13 m)."""
  edges = [(0., 84., {'way_id': 50, 'crossing': []}), (84., 100., {'way_id': 50}), (100., 400., {})]
  side = {'50:f': [entry('stop', 80., junction=900)]}
  at = {'900': {'pos': list(pt(100.)), 'approaches': []}}
  v = StubJunction(edges, {100.: four()[100.]}, every=2.)
  c, j = junction_of(route_controls(v, steps(every=2.), sidecar_dir=osm_dir(tmp_path, side, at))[0])
  assert (c.edge, round(c.route_m, 1), c.junction) == (0, 84. - OFF, False)
  assert haversine((j.lat, j.lon), pt(100.)) < 0.5 and j.osm_node_id == 900
  assert roles(j) == {0: ('exit', 0), 90: ('right', 90), 270: ('left', -90)}
  assert round(j.stop_point_distance, 1) == round(100. - (84. - OFF), 1) and j.stop_point_source == 'default'
  assert j.status == 'ok'
  # the junction not on the route ahead: read where the stop is, where no road meets, so it is failed (no cross road to
  # judge), logged; a stop Valhalla flags there with nothing in the sidecar is the same
  elsewhere = {'900': {'pos': list(toward(pt(100.), 90., 40.)), 'approaches': []}}
  with patch.object(controls_module, 'cloudlog') as log:
    _, j = junction_of(route_controls(v, steps(every=2.), sidecar_dir=osm_dir(tmp_path / 'b', side, elsewhere))[0])
  assert haversine((j.lat, j.lon), pt(84.)) < 0.5 and [a.role for a in j.approaches] == ['exit'] and j.status == 'failed'
  assert any('no other road' in str(c.args[0]) for c in log.warning.call_args_list)
  flagged = StubJunction([(0., 84., {'way_id': 50, 'crossing': [], 'stop': True}), (84., 100., {'way_id': 50}), (100., 400., {})],
                         {100.: four()[100.]}, every=2.)
  _, j = junction_of(route_controls(flagged, steps(every=2.), sidecar_dir=osm_dir(tmp_path / 'c'))[0])
  assert j.status == 'failed' and haversine((j.lat, j.lon), pt(84.)) < 0.5


def test_own_edge_is_the_located_edge_by_heading_with_its_end_node():
  """The car's own edge for SG2's avoidance loop (controls.own_edge): its way, direction and end node, so the exclusion of
  a control on it is never sent and that control is unavoidable (STOP_SIGNS.md 3.3, N10)."""
  from openpilot.selfdrive.turnpilot.routing.controls import own_edge, same_edge
  shape = [{'lat': 39.77, 'lon': -86.16}, {'lat': 39.7705, 'lon': -86.16}, {'lat': 39.771, 'lon': -86.16}]
  answers = {True: [{'edges': [{'edge_info': {'shape': shape, 'way_id': 50}, 'edge': {'forward': True}, 'distance': 1., 'percent_along': 0.4,
                               'heading': 0.}]}],
             False: [{'edges': [{'edge_info': {'shape': shape, 'way_id': 50}, 'edge': {'forward': False}, 'distance': 1., 'percent_along': 0.6,
                                'heading': 180.}]}]}
  requests = []

  def call(forward):
    def f(endpoint, req):
      requests.append((endpoint, req))
      return answers[forward]
    return f
  assert own_edge(call(True), (39.7704, -86.16), 0.) == (50, True, (39.771, -86.16))
  assert requests[-1][1]['locations'][0]['heading'] == 0
  assert own_edge(call(False), (39.7704, -86.16), 180.) == (50, False, (39.77, -86.16))  # its shape runs the way's direction
  assert own_edge(lambda e, r: [{'edges': []}], (39.77, -86.16), 0.) is None
  assert own_edge(lambda e, r: (_ for _ in ()).throw(RuntimeError('down')), (39.77, -86.16), 0.) is None
  # the same edge: way and direction equal, the end node within 1.5 m (a trace's node and a locate's shape end differ by
  # rounding); a split way's next edge is not
  assert same_edge((50, True, (39.771, -86.16)), (50, True, (39.771000005, -86.1600001)))
  assert not same_edge((50, True, (39.771, -86.16)), (50, False, (39.771, -86.16)))
  assert not same_edge((50, True, (39.771, -86.16)), (50, True, (39.7712, -86.16)))
  assert not same_edge((50, True, (39.771, -86.16)), None)


def test_exclusion_headings_are_valhallas_own_at_the_point():
  """Valhalla correlates an exclude_location by comparing its heading with the edge heading it computes itself, which on a
  short curved approach can be 50 deg off the route's local bearing at the point (Indianapolis way 17439639: 38 deg from the
  route's shape, 349 from locate), so the point is dropped. controls.exclusion_headings reads locate's heading of the
  candidate on the control's own way and direction, in one batched call; None where none is found (the shape's bearing stays)."""
  from openpilot.selfdrive.turnpilot.routing.controls import exclusion_headings
  answers = [{'edges': [{'edge_info': {'way_id': 7}, 'edge': {'forward': False}, 'heading': 170.}, {'edge_info': {'way_id': 9}, 'edge': {'forward': True},
                                                                                                    'heading': 349.4}]},
             {'edges': [{'edge_info': {'way_id': 5}, 'edge': {'forward': True}, 'heading': 10.}]}, None]
  requests = []

  def call(endpoint, req):
    requests.append((endpoint, req))
    return answers
  assert exclusion_headings(call, [(39.8244, -86.1928, 9, True), (39.8, -86.1, 6, True), (39.7, -86.0, 1, True)]) == [349.4, None, None]
  assert requests == [('locate', {'locations': [{'lat': 39.8244, 'lon': -86.1928}, {'lat': 39.8, 'lon': -86.1}, {'lat': 39.7, 'lon': -86.0}],
                                  'costing': 'auto', 'verbose': True})]
  assert exclusion_headings(lambda e, r: (_ for _ in ()).throw(RuntimeError('down')), [(1., 2., 3, True)]) == [None]
  assert exclusion_headings(call, []) == []
