"""The visibility file (turnpilot/maps/visibility.json) and its loader (turnpilot/visibility_table.py; STOP_SIGNS.md 1.5,
1.6, rows O11 and E2)."""
import copy
import json
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot import visibility_table as vt

REQUIRED = ('cameraSetbackM', 'lateralOffsetM', 'heightM', 'lens', 'mask', 'darkThreshold', 'veil', 'tables', 'allowProvisionalFor')


def filled() -> dict:
  """A visibility file as S1 and S4a would leave it (made-up numbers)."""
  raw = json.loads(vt.SHIPPED.read_text())
  raw.update(cameraSetbackM=1.9, lateralOffsetM=0.0, heightM=1.3, darkThreshold=45, veil='none', actuatorDelayS=0.45,
             lens={'model': 'equisolid', 'f': 452., 'k': 0.3, 'edgeDeg': {'left': 88.1, 'right': 88.1}},
             mask={'left': [[b, -30., 30.] for b in range(89)], 'right': [[b, -30., 30.] for b in range(89)]},
             launchVerified={'date': '2026-10-30', 'goes': 10, 'takenOver': 10, 'maxTakeoverS': 1.1, 'verdict': 'pass'})
  return raw


def write(tmp_path, raw, name='visibility.json'):
  p = tmp_path / name
  p.write_text(json.dumps(raw))
  return p


class TestShipped:
  def test_the_shipped_file_is_pv1_provisional_with_no_camera_measurements(self):
    v = vt.load(vt.SHIPPED)
    assert v.allow_provisional_for == 'allWay'
    assert v.launch_verified is None and v.actuator_delay_s is None
    ready, missing = v.camera_ready()
    assert not ready
    assert set(missing) == {'cameraSetbackM', 'lateralOffsetM', 'heightM', 'lens', 'mask', 'darkThreshold'}
    assert all(t.source == 'provisional' for t in v.tables)

  def test_pv1_bins(self):
    v = vt.load(vt.SHIPPED)
    cons = v.views('day', 'conservative', use='allWay')
    opt = v.views('day', 'optimistic', use='allWay')
    assert sr.bins_at(cons['road'], 10.) == 150. and sr.bins_at(cons['road'], 20.) == 0.
    assert sr.bins_at(cons['full'], 25.) == 28. and sr.bins_at(cons['full'], 40.) == 0.
    assert sr.bins_at(cons['side'], 52.4) == 0. and sr.bins_at(cons['side'], sr.INNER_BAND_HI_DEG) == 32.
    assert sr.bins_at(cons['side'], 85.) == 32.
    assert sr.bins_at(opt['side'], 60.) == 44.
    assert opt['road'] == cons['road'] and opt['full'] == cons['full']  # no optimistic table: the conservative one

  def test_provisional_tables_only_where_allowed(self):
    v = vt.load(vt.SHIPPED)
    assert v.views('day', use='twoWay') == {}
    assert v.views('day', use='twoWay', candidates=True)  # route_audit's predicted reach reads them anyway

  def test_no_table_at_night(self):
    assert vt.load(vt.SHIPPED).views('night', use='allWay') == {}

  def test_sun_ahead_and_overcast_read_the_day_rows(self):
    v = vt.load(vt.SHIPPED)
    assert v.views('sunAhead', use='allWay') == v.views('day', use='allWay')
    assert v.views('overcast', use='allWay') == v.views('day', use='allWay')


class TestSchema:
  @pytest.mark.parametrize('key', REQUIRED)
  def test_a_missing_field_is_a_loud_error_naming_it(self, tmp_path, key):
    raw = json.loads(vt.SHIPPED.read_text())
    del raw[key]
    with pytest.raises(vt.VisibilityError, match=key):
      vt.load(write(tmp_path, raw))

  @pytest.mark.parametrize('path, value, name', [
    (('lens',), {'model': 'equisolid', 'f': 452., 'k': 0.3}, 'lens.edgeDeg'),
    (('lens',), {'model': 'fisheye', 'f': 452., 'k': 0.3, 'edgeDeg': {'left': 88., 'right': 88.}}, 'lens.model'),
    (('mask',), {'left': [[0, 10., -10.]], 'right': []}, 'mask.left'),
    (('veil',), 'sometimes', 'veil'),
    (('cameraSetbackM',), 'far', 'cameraSetbackM'),
    (('allowProvisionalFor',), 'twoWay', 'allowProvisionalFor'),
    (('actuatorDelayS',), -1., 'actuatorDelayS'),
    (('launchVerified',), {'date': '2026-10-30', 'goes': 10}, 'launchVerified'),
  ])
  def test_a_bad_value_names_its_field(self, tmp_path, path, value, name):
    raw = filled()
    raw[path[0]] = value
    with pytest.raises(vt.VisibilityError, match=name.replace('.', r'\.')):
      vt.load(write(tmp_path, raw))

  @pytest.mark.parametrize('field, value', [('bins', [[20., 28.], [0., 150.]]), ('bins', []), ('view', 'rear'), ('lighting', 'dawn'),
                                            ('source', 'guess'), ('bound', 'middling'), ('bins', [[0., -1.]])])
  def test_a_bad_table_names_it(self, tmp_path, field, value):
    raw = filled()
    raw['tables'][0][field] = value
    with pytest.raises(vt.VisibilityError, match=rf'tables\[0\]\.{field}'):
      vt.load(write(tmp_path, raw))

  def test_a_filled_file_is_ready(self, tmp_path):
    v = vt.load(write(tmp_path, filled()))
    assert v.camera_ready() == (True, [])
    assert v.setback == 1.9 and v.height == 1.3 and v.veil == 'none' and v.actuator_delay_s == 0.45
    assert v.edge_deg == {'left': 88.1, 'right': 88.1}
    assert v.launch_verified['verdict'] == 'pass'

  def test_an_empty_veil_is_not_ready(self, tmp_path):
    raw = filled()
    raw['veil'] = None
    assert vt.load(write(tmp_path, raw)).camera_ready() == (False, ['veil'])

  def test_a_file_that_does_not_parse(self, tmp_path):
    p = tmp_path / 'visibility.json'
    p.write_text('{"cameraSetbackM": ')
    with pytest.raises(vt.VisibilityError, match="doesn't read"):
      vt.load(p)


class TestCheck:
  def test_check_accepts_the_shipped_file_and_says_what_is_empty(self, capsys):
    assert vt.main(['--check', str(vt.SHIPPED)]) == 0
    out = capsys.readouterr().out
    assert 'cameraSetbackM' in out and 'not ready' in out

  @pytest.mark.parametrize('key', REQUIRED)
  def test_check_refuses_a_file_missing_any_field(self, tmp_path, capsys, key):
    raw = filled()
    del raw[key]
    assert vt.main(['--check', str(write(tmp_path, raw))]) == 1
    assert key in capsys.readouterr().err

  def test_check_accepts_a_filled_file(self, tmp_path, capsys):
    assert vt.main(['--check', str(write(tmp_path, filled()))]) == 0
    assert 'ready' in capsys.readouterr().out


class TestTables:
  def test_measured_is_preferred_and_provisional_needs_its_allowance(self, tmp_path):
    raw = filled()
    raw['tables'].append({'source': 'measured', 'bound': 'conservative', 'view': 'side', 'lighting': 'day', 'bins': [[0, 0], [52.5, 37]],
                          'cars': 40, 'date': '2026-11-01'})
    v = vt.load(write(tmp_path, raw))
    assert sr.bins_at(v.views('day', use='allWay')['side'], 60.) == 37.
    raw['allowProvisionalFor'] = None
    v = vt.load(write(tmp_path, raw))
    assert set(v.views('day', use='allWay')) == {'side'}  # road and full were provisional only

  def test_a_sim_table_is_refused_off_the_sim(self, tmp_path):
    raw = filled()
    raw['tables'].append({'source': 'sim', 'bound': 'conservative', 'view': 'side', 'lighting': 'day', 'bins': [[0, 0], [52.5, 90]],
                          'cars': 0, 'date': '2026-11-01'})
    v = vt.load(write(tmp_path, raw))
    assert sr.bins_at(v.views('day', use='allWay')['side'], 60.) == 32.
    assert len(v.refused) == 1 and 'sim' in v.refused[0]
    v = vt.load(write(tmp_path, raw), sim_ok=True)
    assert sr.bins_at(v.views('day', use='allWay')['side'], 60.) == 90. and v.refused == []

  def test_sim_accepted_needs_simulation_the_param_and_a_sim_car(self):
    sim_car = SimpleNamespace(fingerprintSource='fixed', brand='tesla')
    real_car = SimpleNamespace(fingerprintSource='can', brand='tesla')
    assert vt.sim_accepted({'SIMULATION': '1'}, True, sim_car)
    assert not vt.sim_accepted({}, True, sim_car)
    assert not vt.sim_accepted({'SIMULATION': '1'}, False, sim_car)
    assert not vt.sim_accepted({'SIMULATION': '1'}, True, real_car)
    assert not vt.sim_accepted({'SIMULATION': '1'}, True, None)
    assert vt.sim_accepted({'SIMULATION': '1'}, True, SimpleNamespace(fingerprintSource='can', brand='mock'))

  def test_mask_and_sight(self, tmp_path):
    v = vt.load(write(tmp_path, filled()))
    s = v.sight(v.views('day', use='allWay'))
    assert s.edge_deg == {'left': 88.1, 'right': 88.1}
    assert s.mask('left', 60., 20.)
    raw = filled()
    raw['mask']['left'] = [row for row in raw['mask']['left'] if not 70 <= row[0] < 75]  # the pillar
    v = vt.load(write(tmp_path, raw))
    s = v.sight(v.views('day', use='allWay'))
    assert not s.mask('left', 72., 20.) and s.mask('right', 72., 20.)

  def test_cached_reloads_on_change(self, tmp_path):
    p = write(tmp_path, filled())
    a = vt.cached(p)
    assert vt.cached(p) is a
    raw = filled()
    raw['cameraSetbackM'] = 2.1
    p.write_text(json.dumps(raw) + ' ')
    assert vt.cached(p).setback == 2.1

  def test_cached_gives_none_for_a_bad_file(self, tmp_path):
    raw = copy.deepcopy(filled())
    del raw['lens']
    assert vt.cached(write(tmp_path, raw)) is None
    assert vt.cached(tmp_path / 'missing.json') is None


def test_a_missing_file_is_said_once_not_every_second(tmp_path):
  from unittest.mock import patch
  with patch.object(vt.cloudlog, 'error') as error:
    for _ in range(3):
      assert vt.cached(tmp_path / 'gone.json') is None
  assert error.call_count == 1
