"""sim_stop_files.py: the sim's own visibility file and survey of its all-way stops (STOP_SIGNS.md v3 8.2, SG4c), synthetic,
read by turnpilotd only under the sim gate (SIMULATION=1, TurnPilotSimLightOracle and a sim car: set_sim). Built from a sim
sidecar shard here; the shipped files are checked as they load."""
import datetime
import json
from pathlib import Path

from openpilot.selfdrive.turnpilot import visibility_table as vt
from openpilot.selfdrive.turnpilot.maps import surveys as survey_file
from openpilot.selfdrive.turnpilot.turnpilotd import SIM_SURVEYS, SIM_SUN_UTC_H, SIM_VISIBILITY, TurnPilot
from openpilot.tools.sim.bridge.beamng import sim_stop_files as ssf


def shard(tmp_path, junctions):
  d = tmp_path / 'osm_controls'
  d.mkdir()
  (d / '1.json').write_text(json.dumps({'controls': {}, 'junctions': junctions, 'osm_timestamp': 'sim-x', 'rail': [], 'tile': 1, 'version': 1}))
  return tmp_path


def leg(way, bearing, stops):
  return {'way_id': way, 'dir': 'f', 'bearing_deg': bearing, 'class': 'residential', 'maxspeed': 11.176, 'oneway': False, 'stops': stops,
          'kind': 'stop' if stops else None, 'evidence': 'mid_directed' if stops else 'none', 'line': None}


def test_every_all_way_of_the_sim_sidecar_is_surveyed_with_a_synthetic_day_stamp_and_two_ways_are_not(tmp_path):
  tiles = shard(tmp_path, {'11': {'pos': [39.76, -86.15], 'approaches': [leg(1, 0., True), leg(2, 90., True), leg(3, 180., True), leg(4, 270., True)]},
                           '12': {'pos': [39.77, -86.15], 'approaches': [leg(5, 0., True), leg(6, 90., False), leg(7, 270., False)]}})
  entries = ssf.surveys(tiles, today=datetime.date(2026, 10, 30))
  assert [e['osmNodeId'] for e in entries] == [11] and entries[0]['synthetic']
  e = entries[0]
  assert sorted(a['bearingDeg'] for a in e['approaches']) == [0., 90., 180., 270.]  # the heading arriving: the leg's, reversed
  assert all(a['postedMph'] == 25 and a['stops'] for a in e['approaches'])
  assert {v['approachBearingDeg'] for v in e['validations']} == {0., 90., 180., 270.}
  assert all(v['lighting'] == 'day' and v['verdict'] == 'pass' and v['synthetic'] for v in e['validations'])
  out = tmp_path / 'surveys.json'
  out.write_text(json.dumps(entries))
  loaded = survey_file.load(out)
  assert not loaded.errors and len(loaded.entries) == 1


def test_the_sim_visibility_file_is_ready_with_a_passing_launch_and_loads_only_as_the_sims(tmp_path):
  raw = ssf.visibility()
  p = tmp_path / 'v.json'
  p.write_text(json.dumps(raw))
  v = vt.load(p, sim_ok=True)
  assert v.camera_ready()[0] and v.launch_passed and raw['synthetic']
  assert raw['lens']['f'] == 425.25 and raw['veil'] == 'none'


def test_the_shipped_sim_files_load_and_turnpilotd_reads_them_only_under_the_sim_gate():
  assert Path(SIM_VISIBILITY).exists() and Path(SIM_SURVEYS).exists()
  v = vt.load(Path(SIM_VISIBILITY), sim_ok=True)
  assert v.camera_ready()[0] and v.launch_passed
  s = survey_file.load(Path(SIM_SURVEYS))
  assert not s.errors and s.entries and all(e.get('synthetic') for e in s.entries)
  tp = TurnPilot()
  shipped = (tp.vis_source, tp.survey_source)
  t = datetime.datetime(2026, 10, 30, 3, 0, tzinfo=datetime.UTC).timestamp()  # night in Indianapolis
  tp.wall_clock = lambda: t
  assert tp.decider_clock() == t
  tp.set_sim(True)
  assert tp.vis_source() is not None and tp.vis_source().launch_passed
  assert datetime.datetime.fromtimestamp(tp.decider_clock(), datetime.UTC).hour == SIM_SUN_UTC_H
  tp.set_sim(False)
  assert (tp.vis_source, tp.survey_source) == shipped and tp.decider_clock() == t
