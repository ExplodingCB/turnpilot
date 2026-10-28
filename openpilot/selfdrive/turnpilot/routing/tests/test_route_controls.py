"""The route's stop signs, yields and signals from Valhalla (routing/controls.py): on a stub Valhalla, and on the sim's
tiles with controls when they're built (tools/turnpilot/build_sim_tiles.sh into ~/turnpilot_data/valhalla/west_coast_usa_signals)."""
import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path
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
    out = []
    for loc in req['locations']:
      m = (loc['lat'] - LAT0) * M_PER_DEG
      hit = next(((d, s, y) for d, s, y in self.edges if d['begin_shape_index'] * self.every < m < d['end_shape_index'] * self.every), None)
      if hit is None or hit[0]['id'] in self.unmatched:
        out.append({'edges': [{'edge_id': {'value': 1}, 'edge': {'stop_sign': True}}]})
      else:
        out.append({'edges': [{'edge_id': {'value': hit[0]['id']}, 'edge': {'stop_sign': hit[1], 'yield_sign': hit[2]}}]})
    return out


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
  # one batched verbose locate, a metre back from each edge's end
  locates = [r for e, r in v.requests if e == 'locate']
  assert len(locates) == 1 and locates[0]['verbose'] and len(locates[0]['locations']) == 4
  assert abs((locates[0]['locations'][0]['lat'] - LAT0) * M_PER_DEG - 99.) < 0.05


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
  assert [len(r['locations']) for e, r in v.requests if e == 'locate'] == [3]  # the internal edge is located too
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
  """call() answering from a recorded trace_attributes and one batched locate."""
  def call(endpoint, req):
    if endpoint == 'trace_attributes':
      return fixture['trace_attributes']
    assert endpoint == 'locate' and len(req['locations']) == len(fixture['locate'])
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


def test_b3_a_sidecar_stop_tied_to_no_junction_is_still_a_stop(tmp_path):
  """The sidecar keeps a stop it ties to no junction within 60 m (a mid-block trail or rail crossing) with junction_node
  0: where Valhalla doesn't flag it either, the union is the car's only source for it. It stops at its edge's default
  stop point, with no OSM junction node, its sign carried as data."""
  v = StubValhalla([(0., 100., {}), (100., 180., {'way_id': 77}), (180., 400., {})])
  controls, status = route_controls(v, steps(), sidecar_dir=osm_dir(tmp_path, {'77:f': [entry('stop', 140., junction=0)]}))
  assert status == 'ok'
  assert [(c.kind, round(c.route_m, 1), c.osm_node, c.line_source, round(c.osm_sign_m, 1)) for c in controls] == \
    [('stop', 180. - OFF, 0, 'default', 140.)]


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
