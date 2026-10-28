"""The OSM stop-sign sidecar (osm_controls.py): its junction model on a few synthetic OSM ways (OPL, no osmium), its shards,
and build_region.py --sidecar-only on a tiny region (with osmium, when the build machine has it)."""
import hashlib
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from tools.turnpilot.mapserver import osm_controls

LAT0, LON0 = 39.77, -86.16
M_PER_DEG = 111195.
MAPSERVER = Path(osm_controls.__file__).parent


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
                                        'stop_tag': 'minor', 'evidence': 'mid_directed', 'node': 3}]}
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
                                      'evidence': evidence, 'node': 4}]
  stops = {f"{a['way_id']}:{a['dir']}" for a in side['junctions'][4]['approaches'] if a['stops']}
  assert stops == approaches


def test_give_way_is_a_yield_and_level_crossings_are_kept():
  m = crossroads()
  m.node_tags[31] = {'highway': 'give_way', 'direction': 'backward'}
  m.node_tags[2] = {'railway': 'level_crossing'}
  side = m.build()
  assert [(e['kind'], e['junction_node'], e['evidence']) for e in side['controls']['30:b']] == [('yield', 4, 'mid_directed')]
  assert side['rail'] == [{'lat': round(m.nodes[2][0], 7), 'lon': round(m.nodes[2][1], 7), 'ways': [10]}]


def test_b3_a_stop_with_no_junction_near_it_is_kept_at_its_own_line_with_no_junction():
  """A stop the walk ties to no junction within MAXD (a mid-block trail or rail crossing, a dead end) is still a stop
  for its approach: it is emitted with junction_node 0 at its own line, so the car's union has it (and its phantom check
  sees it, C14), and counted. Directed: its direction; both: each; undirected: each direction its way allows."""
  m = crossroads()
  m.node_tags[1] = {'highway': 'stop', 'direction': 'backward'}  # its way ends there (a dead end): no junction that way
  side = m.build()
  line = [round(x, 7) for x in m.nodes[1]]
  assert side['controls'] == {'10:b': [{'kind': 'stop', 'line': line, 'junction_node': 0, 'stop_tag': 'none', 'evidence': 'mid_directed',
                                        'node': 1}]}
  assert side['stats']['no_junction_within_maxd'] == 1
  m = crossroads()
  m.node(5, -100., 0.)
  m.ways[0] = (10, [5, 1, 2, 3, 4], {'highway': 'residential'})
  m.node_tags[1] = {'highway': 'give_way'}  # undirected, 100 m from node 4 and a dead end the other way
  assert {k: [(e['kind'], e['junction_node'], e['evidence']) for e in v] for k, v in m.build()['controls'].items()} == \
    {'10:f': [('yield', 0, 'mid_undirected')], '10:b': [('yield', 0, 'mid_undirected')]}
  m.ways[0] = (10, [5, 1, 2, 3, 4], {'highway': 'residential', 'oneway': 'yes'})
  assert set(m.build()['controls']) == {'10:f'}  # a oneway's only arrival


def continuation(first_id, second_id, stop_at, **stop):
  """Way first_id runs north from node 1 to node 3 at 30 m, where way second_id carries on (a way split, a 2-leg node,
  not a junction) to node 4 at 40 m, a junction with way 20 (tertiary) across it. The OPL lists way `second_id` first
  when it is the lower id, as osmium does."""
  m = Map()
  for nid, north in ((1, 0.), (2, 20.), (3, 30.), (4, 40.), (31, 90.)):
    m.node(nid, north, 0.)
  m.node(21, 40., -50.)
  m.node(22, 40., 50.)
  ways = [(first_id, [1, 2, 3]), (second_id, [3, 4, 31])]
  for wid, nids in sorted(ways):
    m.way(wid, nids, highway='residential')
  m.way(20, [21, 4, 22], highway='tertiary')
  m.node_tags[stop_at] = {'highway': 'stop', **stop}
  return m


@pytest.mark.parametrize('first_id, second_id', [(15, 16), (16, 15)])
@pytest.mark.parametrize('stop_at', [2, 3])
def test_b3_the_walk_carries_on_across_a_way_split_to_the_junction(first_id, second_id, stop_at):
  """Grant Street (Evanston IL, stop node 11351528025, the last node of way 320636666, the junction a few metres on
  along way 1485056109): the walk to the junction crosses a 2-leg continuation node onto the way carrying on, whether
  the stop is mid-way or on the split node itself. The entry is keyed to the way arriving at the stop (its line is on
  that edge, where the car looks for it), and the junction lists the approach it arrives on as stopping."""
  m = continuation(first_id, second_id, stop_at, direction='forward')
  side = m.build()
  assert side['controls'] == {f'{first_id}:f': [{'kind': 'stop', 'line': [round(x, 7) for x in m.nodes[stop_at]], 'junction_node': 4,
                                                 'stop_tag': 'none', 'evidence': 'mid_directed', 'node': stop_at}]}
  stops = {(a['way_id'], a['dir']) for a in side['junctions'][4]['approaches'] if a['stops']}
  assert stops == {(second_id, 'f')} and side['stats'].get('no_junction_within_maxd', 0) == 0
  # the other direction's walk from the split node runs back down the first way: no junction, kept with none
  side = continuation(first_id, second_id, 3, direction='backward').build()
  assert {k: [e['junction_node'] for e in v] for k, v in side['controls'].items()} == {f'{second_id}:b': [0]}


def test_b3_signs_no_approach_takes_are_listed_unplaced():
  """Every stop and give_way node is somewhere in the sidecar, so route_audit can list one near a route that no control
  placed: one on no drivable way (a sign mapped beside the road, a footway's), and a stop=minor on a junction whose
  approaches are all of the top class, go in 'unplaced' with why."""
  m = crossroads()
  m.node(40, 60., 5., highway='stop', direction='forward')  # beside way 10, on no way
  m.node(41, 70., 8.)
  m.way(41, [40, 41], highway='footway')
  side = m.build()
  assert side['controls'] == {}
  assert side['unplaced'] == [{'node': 40, 'kind': 'stop', 'lat': round(m.nodes[40][0], 7), 'lon': round(m.nodes[40][1], 7),
                               'direction': 'forward', 'stop_tag': 'none', 'why': 'not_on_a_drivable_way'}]
  m = crossroads(highway='stop', stop='minor')
  m.ways[0] = (10, [1, 2, 3, 4], {'highway': 'tertiary'})
  m.ways[2] = (30, [4, 31, 32], {'highway': 'tertiary'})
  side = m.build()
  assert side['controls'] == {} and [(u['node'], u['why']) for u in side['unplaced']] == [(4, 'no_lower_approach')]
  cell = osm_controls.level2_index(LON0, LAT0)
  assert osm_controls.shards(side, {cell}, '2026-10-26T21:24:46Z')[cell]['unplaced'] == side['unplaced']


def test_shards_on_the_level2_grid_stamped_one_per_tile():
  side = crossroads(highway='stop', stop='all').build()
  cell = osm_controls.level2_index(LON0, LAT0)
  empty = cell + 1
  out = osm_controls.shards(side, {cell, empty}, '2026-10-26T21:24:46Z')
  assert set(out) == {cell, empty}
  assert out[empty] == {'version': osm_controls.VERSION, 'osm_timestamp': '2026-10-26T21:24:46Z', 'tile': empty, 'controls': {},
                        'junctions': {}, 'rail': [], 'unplaced': []}
  assert out[cell]['osm_timestamp'] == '2026-10-26T21:24:46Z' and set(out[cell]['controls']) == {'10:f', '20:f', '20:b', '30:b'}
  assert set(out[cell]['junctions']) == {'4'}  # JSON keys: the node id as text


def test_tile_cells_are_the_level2_tiles(tmp_path):
  for rel in ('tiles/2/000/756/425.gph', 'tiles/2/000/756/426.gph', 'tiles/1/047/701.gph', 'tiles/0/002/813.gph'):
    (tmp_path / rel).parent.mkdir(parents=True, exist_ok=True)
    (tmp_path / rel).write_bytes(b'x')
  assert osm_controls.tile_cells(tmp_path / 'tiles') == {756425, 756426}


# --- build_region.py --sidecar-only, with osmium ---

HAVE_OSMIUM = shutil.which('osmium') is not None


def osm_xml(m: Map) -> str:
  lines = ["<?xml version='1.0' encoding='UTF-8'?>", "<osm version='0.6' generator='test'>"]
  for nid, (lat, lon) in sorted(m.nodes.items()):
    t = ''.join(f"<tag k='{k}' v='{v}'/>" for k, v in m.node_tags.get(nid, {}).items())
    lines.append(f"<node id='{nid}' version='1' lat='{lat:.7f}' lon='{lon:.7f}'>{t}</node>")
  for wid, nids, t in sorted(m.ways):
    nds = ''.join(f"<nd ref='{n}'/>" for n in nids)
    tt = ''.join(f"<tag k='{k}' v='{v}'/>" for k, v in t.items())
    lines.append(f"<way id='{wid}' version='1'>{nds}{tt}</way>")
  lines.append('</osm>')
  return '\n'.join(lines)


@pytest.mark.skipif(not HAVE_OSMIUM, reason="no osmium (brew install osmium-tool)")
def test_sidecar_only_rebuilds_the_shards_stamped_with_the_regions_timestamp(tmp_path):
  m = crossroads()
  m.node_tags[3] = {'highway': 'stop', 'direction': 'forward'}
  region = tmp_path / 'indianapolis'
  region.mkdir()
  (tmp_path / 'region.osm').write_text(osm_xml(m))
  subprocess.run(['osmium', 'cat', '-O', '-o', region / 'indianapolis.osm.pbf', tmp_path / 'region.osm'], check=True)
  cell = osm_controls.level2_index(LON0, LAT0)
  digits = f"{cell:09d}"
  tile = f"tiles/2/{digits[:3]}/{digits[3:6]}/{digits[6:]}.gph"
  (region / tile).parent.mkdir(parents=True)
  (region / tile).write_bytes(b'tile')
  meta = {'id': 'indianapolis', 'osm_timestamp': '2026-10-26T21:24:46Z', 'built_at': '2026-10-27T23:49:49Z'}
  (region / 'meta.json').write_text(json.dumps(meta))
  stale = {'osm_controls/1.json': [2, 1, '0' * 64, 3]}
  (region / 'index.json').write_text(json.dumps({tile: [2, cell, hashlib.sha256(b'tile').hexdigest(), 4], **stale}))

  subprocess.run([sys.executable, MAPSERVER / 'build_region.py', 'indianapolis', '--sidecar-only', '--out', tmp_path], check=True)

  shard = json.loads((region / f'osm_controls/{cell}.json').read_text())
  assert shard['osm_timestamp'] == meta['osm_timestamp'] and [e['junction_node'] for e in shard['controls']['10:f']] == [4]
  assert json.loads((region / 'meta.json').read_text()) == meta  # the region's stamp and build time stay
  index = json.loads((region / 'index.json').read_text())
  data = (region / f'osm_controls/{cell}.json').read_bytes()
  assert index == {tile: [2, cell, hashlib.sha256(b'tile').hexdigest(), 4],
                   f'osm_controls/{cell}.json': [2, cell, hashlib.sha256(data).hexdigest(), len(data)]}
  assert (region / 'controls_built').exists()  # the region's tiles come from its .pbf, which carries its controls
