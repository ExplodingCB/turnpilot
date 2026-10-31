"""allway_candidates.py (STOP_SIGNS_SATURDAY.md T2): 3-way all-way stops arriving on the T's stem with the sun off the nose,
the legs from the ways (a one-way exit leg the sidecar's records don't list), junctions the sidecar builder itself made from
synthetic OPL (no osmium), and its refusals."""
import json
import math
from datetime import datetime, timedelta, timezone

import pytest

from openpilot.tools.turnpilot import allway_candidates as ac
from tools.turnpilot.mapserver import osm_controls

LAT0, LON0 = 39.77, -86.16
M_PER_DEG = 111195.
EDT = timezone(timedelta(hours=-4))
NOON = datetime(2026, 10, 31, 12, 0, tzinfo=EDT).timestamp()
STAMP = '2026-10-26T21:24:46Z'


def ll(north, east):
  return LAT0 + north / M_PER_DEG, LON0 + east / (M_PER_DEG * math.cos(math.radians(LAT0)))


def esc(s):
  return ''.join(c if c.isalnum() or c in '_:-.' else f"%{ord(c):x}%" for c in s)


class Map:
  def __init__(self):
    self.nodes, self.node_tags, self.ways = {}, {}, []

  def node(self, nid, north, east, **t):
    self.nodes[nid] = ll(north, east)
    if t:
      self.node_tags[nid] = t

  def way(self, wid, nids, **t):
    self.ways.append((wid, nids, t))

  def way_lines(self):
    return [f"w{w} v1 dV c0 t2026-01-01T00:00:00Z i0 u T{','.join(f'{esc(k)}={esc(v)}' for k, v in t.items())} N" +
            ','.join(f"n{n}x{self.nodes[n][1]:.7f}y{self.nodes[n][0]:.7f}" for n in nids) for w, nids, t in self.ways]

  def control_lines(self):
    return [f"n{n} v1 dV c0 t2026-01-01T00:00:00Z i0 u T{','.join(f'{esc(k)}={esc(v)}' for k, v in t.items())} " +
            f"x{self.nodes[n][1]:.7f} y{self.nodes[n][0]:.7f}" for n, t in self.node_tags.items()]

  def side(self):
    return osm_controls.build_sidecar(self.control_lines(), self.way_lines)

  def run(self, start=NOON, end=NOON + 3600., max_mph=20.):
    side = self.side()
    ways, occ = ac.parse_ways(self.way_lines())
    return ac.candidates(side['junctions'], side['rail'], ways, occ, (LAT0, LON0), 1., start, end, max_mph)


def tee(stem='south', cross='residential', cross_speed='20 mph', stem_class='residential', stop='all'):
  """A T at node 4 (an all-way stop on the node, stop=all): way 10 its stem from the south (or the north), way 20 across it
  west to east."""
  m = Map()
  sign = -1. if stem == 'south' else 1.
  for nid, d in ((1, 200.), (2, 100.), (3, 40.)):
    m.node(nid, sign * d, 0.)
  m.node(4, 0., 0., highway='stop', stop=stop)
  for nid, east in ((21, -200.), (22, -100.), (23, 100.), (24, 200.)):
    m.node(nid, 0., east)
  m.way(10, [1, 2, 3, 4], highway=stem_class, maxspeed='25 mph')
  t = {'highway': cross} | ({'maxspeed': cross_speed} if cross_speed else {})
  m.way(20, [21, 22, 4, 23, 24], **t)
  return m


def test_a_stem_south_t_is_listed_arriving_north_with_the_sun_behind():
  found, why = tee().run()
  assert len(found) == 1 and why == {}
  c = found[0]
  assert (c.node, c.stem_way, c.cross_limit_mph, c.cross_limit_source, c.map_strong, c.exit_only) == (4, 10, 20, 'tagged', True, [])
  assert abs(c.heading_in) < 0.5 or abs(c.heading_in - 360.) < 0.5
  assert c.need_m == 32.9 and c.rows == ['day'] and c.cross_tagged_mph == [20, 20]
  # the audit trip: 150 m back down the stem, 150 m out along each cross leg
  o = ac.point(c.commands['origin'])
  assert abs(o[0] - ll(-150., 0.)[0]) * M_PER_DEG < 1. and abs(o[1] - LON0) < 1e-5
  left, right = ac.point(c.commands['left']), ac.point(c.commands['right'])
  assert left[1] < LON0 < right[1]


def test_a_4_way_is_not_a_t():
  m = tee()
  m.node(31, 100., 0.)
  m.way(30, [4, 31], highway='residential')
  found, why = m.run()
  assert found == [] and why[4] == '4 legs carry arriving traffic, not 3'


def test_a_two_way_is_not_listed():
  m = tee(stop='all')
  del m.node_tags[4]
  m.node_tags[3] = {'highway': 'stop', 'direction': 'forward'}  # only the stem stops
  found, why = m.run()
  assert found == [] and 4 not in why


def test_a_stem_north_t_has_the_sun_ahead_at_noon():
  found, why = tee(stem='north').run()
  assert found == [] and 'sun ahead' in why[4]


def test_a_t_with_a_rail_crossing_within_30_m_is_out():
  m = tee()
  m.node_tags[3] = {'railway': 'level_crossing'}  # 40 m out: kept
  m.nodes[3] = ll(-25., 0.)                       # 25 m: out
  found, why = m.run()
  assert found == [] and 'rail crossing' in why[4]


def test_an_implausible_mix_is_out():
  found, why = tee(cross='primary', cross_speed='20 mph').run()
  assert found == [] and 'implausible' in why[4]


def test_a_one_way_exit_leg_straight_on_is_seen_from_the_ways_and_flagged():
  """181199929: the north leg only leaves, so the sidecar's records (arrivals) don't have it; the ways do."""
  m = tee()
  m.node(31, 100., 0.)
  m.way(30, [4, 31], highway='residential', oneway='yes')
  side = m.side()
  assert {round(a['bearing_deg']) % 360 for a in side['junctions'][4]['approaches']} == {180, 90, 270}  # no north leg in the record
  found, why = m.run()
  assert len(found) == 1 and [round(b) % 360 for b in found[0].exit_only] == [0]


def test_a_one_way_exit_leg_off_straight_is_not_a_t():
  m = tee()
  m.node(31, 70., 70.)
  m.way(30, [4, 31], highway='residential', oneway='yes')
  found, why = m.run()
  assert found == [] and 'besides the T' in why[4]


def test_an_untagged_cross_road_takes_the_class_default():
  found, why = tee(cross_speed=None).run()
  assert found == [] and '30 mph (classDefault)' in why[4]
  found, _ = tee(cross_speed=None).run(max_mph=30.)
  assert found[0].cross_limit_mph == 30 and found[0].cross_limit_source == 'classDefault'


def test_a_service_road_leg_is_out():
  found, why = tee(cross='service').run()
  assert found == [] and 'service' in why[4]


def write_region(tmp_path, m, stamp=STAMP, shard_stamp=STAMP):
  side = m.side()
  (tmp_path / 'meta.json').write_text(json.dumps({'osm_timestamp': stamp}))
  (tmp_path / 'osm_controls').mkdir()
  shard = {'version': 1, 'osm_timestamp': shard_stamp, 'tile': 1, 'controls': {}, 'junctions': {str(k): v for k, v in side['junctions'].items()},
           'rail': side['rail'], 'unplaced': []}
  (tmp_path / 'osm_controls' / '1.json').write_text(json.dumps(shard))
  (tmp_path / 'r.osm.pbf').write_text('')
  return tmp_path


ARGS = ['--near', f'{LAT0},{LON0}', '--radius-km', '1', '--start', '2026-10-31T12:00-04:00', '--end', '2026-10-31T13:00-04:00']


def test_main_lists_the_candidate_and_writes_nothing(tmp_path, monkeypatch, capsys):
  m = tee()
  region = write_region(tmp_path, m)
  monkeypatch.setattr(ac, 'osm_ways', lambda pbf, near, r: ac.parse_ways(m.way_lines()))
  before = sorted(p.name for p in region.rglob('*'))
  assert ac.main(['--region', str(region), *ARGS, '--json']) == 0
  out = json.loads(capsys.readouterr().out)
  assert [c['node'] for c in out['candidates']] == [4]
  assert set(out['candidates'][0]['commands']) == {'survey', 'audit left', 'audit right'}
  assert '--survey-template' in out['candidates'][0]['commands']['survey']
  assert sorted(p.name for p in region.rglob('*')) == before


def test_refuses_without_shards(tmp_path, capsys):
  (tmp_path / 'meta.json').write_text(json.dumps({'osm_timestamp': STAMP}))
  assert ac.main(['--region', str(tmp_path), *ARGS]) == 1
  assert 'no osm_controls/ shards' in capsys.readouterr().err


def test_refuses_a_shard_stamped_otherwise(tmp_path, capsys):
  region = write_region(tmp_path, tee(), shard_stamp='2026-09-01T00:00:00Z')
  assert ac.main(['--region', str(region), *ARGS]) == 1
  assert 'stamped' in capsys.readouterr().err


def test_refuses_a_window_that_touches_dusk(tmp_path, capsys):
  region = write_region(tmp_path, tee())
  args = ['--near', f'{LAT0},{LON0}', '--radius-km', '1', '--start', '2026-10-31T17:30-04:00', '--end', '2026-10-31T18:30-04:00']
  assert ac.main(['--region', str(region), *args]) == 1
  assert 'dusk' in capsys.readouterr().err
  with pytest.raises(ac.Refusal):
    ac.check_window(LAT0, LON0, NOON + 3600., NOON)
