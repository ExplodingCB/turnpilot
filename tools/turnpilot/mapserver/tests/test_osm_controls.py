"""The OSM stop-sign sidecar (osm_controls.py): its junction model on a few synthetic OSM ways (OPL, no osmium), and its
shards."""
import math

import pytest

from tools.turnpilot.mapserver import osm_controls

LAT0, LON0 = 39.77, -86.16
M_PER_DEG = 111195.


def ll(north, east):
  return LAT0 + north / M_PER_DEG, LON0 + east / (M_PER_DEG * math.cos(math.radians(LAT0)))


def opl_escape(s):
  return ''.join(c if c.isalnum() or c in '_:-.' else f"%{ord(c):x}%" for c in s)


def tags(t):
  """OPL's tag list: osmium escapes anything but plain characters ('25 mph' is '25%20%mph')."""
  return ','.join(f"{opl_escape(k)}={opl_escape(v)}" for k, v in t.items())


class Map:
  """Nodes at (m north, m east) of (LAT0, LON0) and ways over them, as osmium's OPL with way node locations."""
  def __init__(self):
    self.nodes, self.node_tags, self.ways = {}, {}, []

  def node(self, nid, north, east, **t):
    self.nodes[nid] = ll(north, east)
    if t:
      self.node_tags[nid] = t
    return nid

  def way(self, wid, nids, **t):
    self.ways.append((wid, nids, t))

  def control_lines(self):
    return [f"n{n} v1 dV c0 t2026-01-01T00:00:00Z i0 u T{tags(t)} x{self.nodes[n][1]:.7f} y{self.nodes[n][0]:.7f}"
            for n, t in self.node_tags.items()]

  def way_lines(self):
    return [f"w{w} v1 dV c0 t2026-01-01T00:00:00Z i0 u T{tags(t)} N" +
            ','.join(f"n{n}x{self.nodes[n][1]:.7f}y{self.nodes[n][0]:.7f}" for n in nids) for w, nids, t in self.ways]

  def build(self):
    return osm_controls.build_sidecar(self.control_lines(), self.way_lines)


def crossroads(**stop_at_4):
  """Way 10 (residential) from the south ends at node 4, where way 20 (tertiary) runs west to east through it and way 30
  (residential) leaves north: a four-leg junction at node 4."""
  m = Map()
  for nid, north in ((1, 0.), (2, 50.), (3, 90.)):
    m.node(nid, north, 0.)
  m.node(4, 100., 0., **stop_at_4)
  m.node(21, 100., -50.)
  m.node(22, 100., 50.)
  m.node(31, 150., 0.)
  m.node(32, 200., 0.)
  m.way(10, [1, 2, 3, 4], highway='residential', maxspeed='25 mph')
  m.way(20, [21, 4, 22], highway='tertiary')
  m.way(30, [4, 31, 32], highway='residential')
  return m


def test_a_directed_mid_way_stop_is_its_approach_with_the_real_line():
  m = crossroads()
  m.node_tags[3] = {'highway': 'stop', 'direction': 'forward', 'stop': 'minor'}
  side = m.build()
  assert side['controls'] == {'10:f': [{'kind': 'stop', 'line': [round(x, 7) for x in m.nodes[3]], 'junction_node': 4,
                                        'stop_tag': 'minor', 'evidence': 'mid_directed'}]}
  junction = side['junctions'][4]
  assert junction['pos'] == [round(m.nodes[4][0], 7), round(m.nodes[4][1], 7)]
  approaches = {(a['way_id'], a['dir']): a for a in junction['approaches']}
  assert set(approaches) == {(10, 'f'), (20, 'f'), (20, 'b'), (30, 'b')}  # way 30 leaves north: cars arrive against its order
  ours = approaches[(10, 'f')]
  assert (ours['stops'], ours['kind'], ours['evidence'], ours['class'], ours['oneway']) == (True, 'stop', 'mid_directed', 'residential', False)
  assert ours['line'] == side['controls']['10:f'][0]['line'] and abs(ours['maxspeed'] - 25 * 0.44704) < 1e-6
  assert round(ours['bearing_deg']) == 180  # the leg leaves the junction to the south
  assert [round(approaches[k]['bearing_deg']) % 360 for k in ((20, 'f'), (20, 'b'), (30, 'b'))] == [270, 90, 0]
  assert not any(approaches[k]['stops'] for k in ((20, 'f'), (20, 'b'), (30, 'b')))
  assert approaches[(20, 'f')]['evidence'] == 'none' and approaches[(20, 'f')]['line'] is None and approaches[(20, 'f')]['maxspeed'] is None


def test_an_undirected_mid_way_stop_goes_to_the_nearest_junction_and_both_to_each():
  m = crossroads()
  m.node_tags[3] = {'highway': 'stop'}
  assert [e['evidence'] for e in m.build()['controls']['10:f']] == ['mid_undirected']
  # a stop between two junctions with direction=both governs each, toward it
  m = crossroads()
  m.node(5, 125., 0., highway='stop', direction='both')
  m.ways[2] = (30, [4, 5, 31, 32], {'highway': 'residential'})
  m.node(41, 150., -50.)
  m.way(40, [41, 31], highway='residential')  # makes node 31 a junction: 3 legs
  controls = m.build()['controls']
  assert {k: [(e['junction_node'], e['evidence']) for e in v] for k, v in controls.items()} == \
    {'30:f': [(31, 'mid_both')], '30:b': [(4, 'mid_both')]}


@pytest.mark.parametrize('stop_tag, approaches', [
  ({'stop': 'all'}, {'10:f', '20:f', '20:b', '30:b'}),
  ({}, {'10:f', '20:f', '20:b', '30:b'}),
  ({'stop': 'minor'}, {'10:f', '30:b'}),  # only the approaches of a lower class than the top one (the tertiary)
])
def test_a_stop_on_the_junction_node_is_on_its_approaches_with_no_line(stop_tag, approaches):
  side = crossroads(highway='stop', **stop_tag).build()
  assert set(side['controls']) == approaches
  evidence = {'all': 'node_all', 'minor': 'node_minor'}.get(stop_tag.get('stop'), 'node_untagged')
  for key in approaches:
    assert side['controls'][key] == [{'kind': 'stop', 'line': None, 'junction_node': 4, 'stop_tag': stop_tag.get('stop', 'none'),
                                      'evidence': evidence}]
  stops = {f"{a['way_id']}:{a['dir']}" for a in side['junctions'][4]['approaches'] if a['stops']}
  assert stops == approaches


def test_give_way_is_a_yield_and_level_crossings_are_kept():
  m = crossroads()
  m.node_tags[31] = {'highway': 'give_way', 'direction': 'backward'}
  m.node_tags[2] = {'railway': 'level_crossing'}
  side = m.build()
  assert [(e['kind'], e['junction_node'], e['evidence']) for e in side['controls']['30:b']] == [('yield', 4, 'mid_directed')]
  assert side['rail'] == [{'lat': round(m.nodes[2][0], 7), 'lon': round(m.nodes[2][1], 7), 'ways': [10]}]


def test_a_stop_with_no_junction_near_it_is_left_out_and_counted():
  m = crossroads()
  m.node_tags[1] = {'highway': 'stop', 'direction': 'backward'}  # its way ends there: no junction that way
  side = m.build()
  assert side['controls'] == {} and side['stats']['no_junction_within_maxd'] == 1


def test_shards_on_the_level2_grid_stamped_one_per_tile():
  side = crossroads(highway='stop', stop='all').build()
  cell = osm_controls.level2_index(LON0, LAT0)
  empty = cell + 1
  out = osm_controls.shards(side, {cell, empty}, '2026-10-26T21:24:46Z')
  assert set(out) == {cell, empty}
  assert out[empty] == {'version': osm_controls.VERSION, 'osm_timestamp': '2026-10-26T21:24:46Z', 'tile': empty, 'controls': {},
                        'junctions': {}, 'rail': []}
  assert out[cell]['osm_timestamp'] == '2026-10-26T21:24:46Z' and set(out[cell]['controls']) == {'10:f', '20:f', '20:b', '30:b'}
  assert set(out[cell]['junctions']) == {'4'}  # JSON keys: the node id as text


def test_tile_cells_are_the_level2_tiles(tmp_path):
  for rel in ('tiles/2/000/756/425.gph', 'tiles/2/000/756/426.gph', 'tiles/1/047/701.gph', 'tiles/0/002/813.gph'):
    (tmp_path / rel).parent.mkdir(parents=True, exist_ok=True)
    (tmp_path / rel).write_bytes(b'x')
  assert osm_controls.tile_cells(tmp_path / 'tiles') == {756425, 756426}
