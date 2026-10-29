"""surveyed_junctions.json and its loader (maps/surveys.py; STOP_SIGNS.md 4.3, SG1b): the schema, the 365-day rule, the
stopPointM rule, and matching by OSM node id, never by a radius."""
import copy
import json
from datetime import date
from unittest.mock import patch

from openpilot.selfdrive.turnpilot.maps import surveys
from openpilot.selfdrive.turnpilot.maps.surveys import SHIPPED, counts, load, match

# STOP_SIGNS.md 4.3's example entry, for a made-up junction (its node id can't match anything): a test fixture only, never
# in the shipped file
EXAMPLE = {"osmNodeId": 0, "lat": 39.8000, "lon": -86.1500, "date": "2026-10-29", "source": "streetImagery",
           "imageryDate": "2026-06", "confirmedBy": [],
           "approaches": [
             {"wayId": 0, "bearingDeg": 90, "stops": True, "plaque": True, "postedMph": 20, "laneEdgeM": 4.0, "lineM": 7.0,
              "nearestParkingM": {"left": 36.0, "right": 40.0}, "sightM": 60, "stopPointM": 9.8},
             {"wayId": 0, "bearingDeg": 180, "stops": True, "plaque": True, "postedMph": 20, "laneEdgeM": 4.0, "lineM": 7.5,
              "nearestParkingM": {"left": 0.0, "right": 38.0}, "sightM": 55},
             {"wayId": 0, "bearingDeg": 270, "stops": True, "plaque": True, "postedMph": 20, "laneEdgeM": 4.0, "lineM": 7.0,
              "nearestParkingM": {"left": 35.0, "right": 37.0}, "sightM": 70},
             {"wayId": 0, "bearingDeg": 0, "stops": True, "plaque": True, "postedMph": 20, "laneEdgeM": 4.0, "lineM": 7.0,
              "nearestParkingM": {"left": 34.0, "right": 41.0}, "sightM": 45}],
           "validations": []}
TODAY = date(2026, 10, 30)


def write(tmp_path, entries):
  path = tmp_path / 'surveyed_junctions.json'
  path.write_text(json.dumps(entries))
  return path


def test_the_shipped_file_is_an_empty_list_that_loads():
  assert json.loads(SHIPPED.read_text()) == []
  loaded = load(SHIPPED)
  assert loaded.entries == [] and loaded.errors == []


def test_the_example_entry_loads_and_counts(tmp_path):
  loaded = load(write(tmp_path, [EXAMPLE]))
  assert loaded.errors == [] and len(loaded.entries) == 1
  entry = loaded.entries[0]
  assert [a['stopPointM'] for a in entry['approaches']] == [9.8, None, None, None]  # 9.8 is exactly 3.3 + 2.0 + 0.5 m outside 4.0
  assert counts(entry, TODAY) == (True, '')
  assert entry['imageryDate'] == '2026-06'


def test_a_stop_point_inside_the_rule_is_rejected_and_the_rest_kept(tmp_path):
  """stopPointM must be at least STOP_CLEAR_M 3.3 m + POS_ERR_M 2.0 m + SURVEY_ERR_M 0.5 m outside the approach's
  laneEdgeM (STOP_SIGNS.md 1.2): closer, the G5 nudge could reach the lane under a position error. It is rejected, out loud,
  and the approach stops at the default point; the survey itself still counts."""
  bad = copy.deepcopy(EXAMPLE)
  bad['approaches'][0]['stopPointM'] = 9.7
  with patch.object(surveys, 'cloudlog') as log:
    loaded = load(write(tmp_path, [bad]))
  assert len(loaded.entries) == 1 and loaded.entries[0]['approaches'][0]['stopPointM'] is None
  assert len(loaded.errors) == 1 and 'stopPointM' in loaded.errors[0]
  assert log.error.called


def test_an_entry_with_a_missing_or_null_field_is_left_out_loudly(tmp_path):
  """The survey template prints null for every field Kellen measures: an entry still holding one is no survey. postedMph
  alone may be null (no sign visible: the class default applies)."""
  missing, null, posted = copy.deepcopy(EXAMPLE), copy.deepcopy(EXAMPLE), copy.deepcopy(EXAMPLE)
  del missing['date']
  null['approaches'][1]['laneEdgeM'] = None
  posted['approaches'][1]['postedMph'] = None
  with patch.object(surveys, 'cloudlog') as log:
    loaded = load(write(tmp_path, [missing, null, posted]))
  assert len(loaded.entries) == 1 and loaded.entries[0]['approaches'][1]['postedMph'] is None
  assert len(loaded.errors) == 2 and log.error.call_count == 2
  # a file that isn't a list, or doesn't read: no surveys, said
  (tmp_path / 'bad.json').write_text('{"osmNodeId": 1}')
  assert load(tmp_path / 'bad.json').entries == [] and load(tmp_path / 'bad.json').errors
  assert load(tmp_path / 'absent.json').errors


def test_over_a_year_old_or_from_imagery_over_a_year_old_does_not_count():
  old, old_imagery, in_person = copy.deepcopy(EXAMPLE), copy.deepcopy(EXAMPLE), copy.deepcopy(EXAMPLE)
  old['date'] = '2025-10-29'          # 366 days before TODAY
  old_imagery['imageryDate'] = '2025-10'  # the month's first day: over a year before TODAY
  in_person['source'], in_person['date'] = 'inPerson', '2025-10-31'
  del in_person['imageryDate']
  assert counts(old, TODAY)[0] is False and 'year' in counts(old, TODAY)[1]
  assert counts(old_imagery, TODAY)[0] is False and 'imagery' in counts(old_imagery, TODAY)[1]
  assert counts(in_person, TODAY) == (True, '')


def test_matched_by_osm_node_id_never_by_a_20_m_radius(tmp_path):
  """A survey is matched by OSM node id; with no id, only on the node itself (the precision OSM and Valhalla share for a
  node's position), never by a neighbourhood radius: a 20 m radius would attach an all-way's survey to a staggered or
  frontage neighbour 10-30 m away (STOP_SIGNS.md 4.3)."""
  surveyed = copy.deepcopy(EXAMPLE) | {'osmNodeId': 1001}
  m_per_deg = 111195.
  node = (39.8000, -86.1500)
  neighbour = (39.8000 + 12. / m_per_deg, -86.1500)  # 12 m north: another junction node
  entries = load(write(tmp_path, [surveyed])).entries
  assert match(entries, 1001, neighbour) is entries[0]   # its id: matched, wherever its lat/lon says
  assert match(entries, 1002, node) is None              # another id, on its very position: not this junction
  assert match(entries, 0, neighbour) is None            # no id, 12 m off: no radius match
  assert match(entries, 0, node) is entries[0]           # the junction's id not known: the entry on its node
  no_id = load(write(tmp_path, [copy.deepcopy(EXAMPLE)])).entries
  assert match(no_id, 0, node) is no_id[0]               # no id on either side: the node it is on
  assert match(no_id, 0, neighbour) is None
  assert match(no_id, 1002, node) is None                # the junction has an id: only the id matches
