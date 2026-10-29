"""junction_validate.py (STOP_SIGNS.md 4.3, step S3; row C9): turnpilot.junction_evidence events read from rlogs, grouped by
the held control's OSM node and arriving approach, judged per lighting row, and a passing stamp written into a copy of
surveyed_junctions.json that maps/surveys.py loads and routing/certify.py picks."""
import json

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.maps import surveys as survey_file
from openpilot.selfdrive.turnpilot.routing.certify import pick_stamp
from openpilot.tools.lib.logreader import save_log
from openpilot.tools.turnpilot import junction_validate as jv

NEED = sr.reach_needed_allway(25. * sr.MPH)  # 45.4 m: a 25 mph cross road's R_req plus latency
LANES = [{'role': r, 'index': 1, 'stopping': True, 'needM': NEED} for r in ('left', 'right', 'opposite')]


def car(tid, role, first_m, t, gap=0.3, stopped=True, through=False, approaching=True, settled=10., cls='car', lighting='day'):
  return {'trackId': tid, 'cls': cls, 'role': role, 'index': 1, 'view': 'sideLeft' if role == 'left' else 'sideRight',
          'firstT': t, 'firstRangeM': first_m, 'firstDStopM': 0., 'needM': NEED, 'approaching': approaching,
          'maxGapInsideS': gap, 'stoppedAtLine': stopped, 'throughAtSpeed': through, 'vMax': 9., 'samples': 30, 'lastT': t + 8.,
          'settledS': settled, 'lighting': lighting}


def traffic(n=10, first=52., t0=1000., **kw):
  """n cars a side, left, right and opposite, first tracked first m out, every 30 s from t0."""
  out, tid = [], 1
  for role in ('left', 'right', 'opposite'):
    for _ in range(n):
      out.append(car(tid, role, first + 0.1 * tid, t0 + 30. * tid, **kw))
      tid += 1
  return out


def event(cars, node=99, heading=0., setback_measured=True):
  return {'event': 'turnpilot.junction_evidence', 'osmNodeId': node, 'controlKey': [0, 1], 'kind': 'stop', 'junctionStatus': 'ok',
          'lat': 39.8, 'lon': -86.15, 'approachBearingDeg': heading, 'ourWay': 0, 'stopPointDistance': 13., 'surveyed': True,
          'setbackM': 1.9, 'setbackMeasured': setback_measured, 'standS': 900., 'lighting': 'day', 'tStart': 990., 'tEnd': 2000.,
          'angleBoundDeg': 2.5, 'cars': cars, 'lanes': LANES}


def write_log(path, events):
  """An rlog of logMessage events, as logmessaged publishes cloudlog's records."""
  msgs = [messaging.new_message(None, valid=True, logMessage=json.dumps({'msg': e, 'level': 'INFO', 'ctx': {}})).as_reader()
          for e in events]
  msgs.insert(0, messaging.new_message('carState').as_reader())  # other traffic in the log
  save_log(str(path), msgs, compress=False)
  return str(path)


def survey_file_with_entry(tmp_path):
  approaches = [{'wayId': 0, 'bearingDeg': b, 'stops': True, 'plaque': True, 'postedMph': 25, 'laneEdgeM': 4.0, 'lineM': 7.0,
                 'nearestParkingM': {'left': 40., 'right': 40.}, 'sightM': 60., 'stopPointM': None} for b in (0., 90., 180., 270.)]
  entry = {'osmNodeId': 99, 'lat': 39.8, 'lon': -86.15, 'date': '2026-10-20', 'source': 'inPerson', 'confirmedBy': [],
           'approaches': approaches, 'validations': []}
  p = tmp_path / 'surveyed_junctions.json'
  p.write_text(json.dumps([entry]))
  return p


def run(tmp_path, events, *extra):
  src = survey_file_with_entry(tmp_path)
  out = tmp_path / 'out.json'
  log = write_log(tmp_path / 'rlog', events)
  code = jv.main(['--surveys', str(src), '--out', str(out), *extra, log])
  return code, src, out


def stamps(out):
  return json.loads(out.read_text())[0]['validations']


def test_a_validated_junction_gets_a_stamp_that_surveys_loads_and_certify_picks(tmp_path, capsys):
  code, src, out = run(tmp_path, [event(traffic())])
  assert code == 0
  (st,) = stamps(out)
  assert st['verdict'] == 'pass' and st['lighting'] == 'day' and st['approachBearingDeg'] == 0. and st['allStopped'] is True
  assert st['cars'] == {'left': 10, 'right': 10, 'opposite': 10} and st['passes'] == 1
  assert st['minFirstTrackM'] == {'left': [pytest.approx(52.1)], 'right': [pytest.approx(53.1)], 'opposite': [pytest.approx(54.1)]}
  assert st['maxDropoutS'] == {'left': [0.3], 'right': [0.3], 'opposite': [0.3]}
  # the copy reads back through the survey loader and the certifier's stamp pick; the input is untouched
  entry = survey_file.load(out).entries[0]
  assert pick_stamp(entry, 'day', 0.) == st and pick_stamp(entry, 'day', 90.) is None
  assert json.loads(src.read_text())[0]['validations'] == []
  assert 'pass' in capsys.readouterr().out


def test_C9_a_vehicle_through_at_speed_fails_the_junction(tmp_path, capsys):
  cars = traffic()
  cars[3]['throughAtSpeed'], cars[3]['stoppedAtLine'] = True, False
  code, _, out = run(tmp_path, [event(cars)])
  assert code == 1 and stamps(out) == []
  text = capsys.readouterr().out
  assert 'through' in text and 'survey may be stale' in text


def test_a_car_that_never_stopped_at_its_line_fails(tmp_path, capsys):
  cars = traffic()
  cars[12]['stoppedAtLine'] = False
  code, _, out = run(tmp_path, [event(cars)])
  assert code == 1 and stamps(out) == [] and 'stop' in capsys.readouterr().out


def test_a_0_8_s_detection_gap_inside_r_req_fails_the_dropout_check(tmp_path, capsys):
  cars = traffic()
  cars[5]['maxGapInsideS'] = 0.8
  code, _, out = run(tmp_path, [event(cars)])
  assert code == 1 and stamps(out) == []
  assert 'dropout' in capsys.readouterr().out
  # 0.66 s is inside CLEAR_S / 3, and is written unrounded
  cars[5]['maxGapInsideS'] = 0.6612345
  code, _, out = run(tmp_path, [event(cars)])
  assert code == 0 and stamps(out)[0]['maxDropoutS']['left'] == [0.6612345]


def test_a_late_first_track_fails_unless_excluded_by_hand_with_a_reason(tmp_path, capsys):
  cars = traffic(n=11)
  late = cars[7]
  late['firstRangeM'] = 30.  # first tracked inside R_req plus latency while approaching
  code, _, out = run(tmp_path, [event(cars)])
  assert code == 1 and stamps(out) == []
  text = capsys.readouterr().out
  assert 'late' in text and f"{late['firstT']:.2f}" in text  # its log time, to find the clip by
  code, _, out = run(tmp_path, [event(cars)], '--exclude', f"{late['firstT']:.2f}=pulled out of a driveway inside the corner")
  assert code == 0
  st = stamps(out)[0]
  assert st['exclusions'] == [{'logT': pytest.approx(late['firstT']), 'reason': 'pulled out of a driveway inside the corner'}]
  assert st['cars']['left'] == 10  # an excluded car is no sample of reach (11 seen)
  # a car first tracked inside it that wasn't moving toward the junction is no late car
  late['approaching'] = False
  code, _, _ = run(tmp_path, [event(cars)])
  assert code == 0


def test_too_few_cars_on_a_binding_side_fails(tmp_path, capsys):
  cars = [c for c in traffic() if not (c['role'] == 'right' and c['trackId'] > 15)]
  code, _, out = run(tmp_path, [event(cars)])
  assert code == 1 and stamps(out) == [] and 'right' in capsys.readouterr().out


def test_cars_first_tracked_before_the_car_stood_at_its_stop_point_dont_count(tmp_path):
  cars = traffic()
  for c in cars[:2]:
    c['settledS'], c['firstRangeM'] = 1.0, 20.  # we were still rolling up: not the stop point's reach, never late
  code, _, out = run(tmp_path, [event(cars + [car(98, 'left', 50., 99000.), car(97, 'left', 50., 99100.)])])
  assert code == 0 and stamps(out)[0]['cars']['left'] == 10 and stamps(out)[0]['minFirstTrackM']['left'][0] >= 50.


def test_passes_from_several_logs_and_approaches_and_lighting_rows(tmp_path):
  # two passes from the south (heading 0) in two logs make one stamp; one from the west (heading 90) is its own approach;
  # cars in sunAhead light make their own row
  src = survey_file_with_entry(tmp_path)
  out = tmp_path / 'out.json'
  a = traffic(n=5)
  b = traffic(n=5, t0=5000.)
  west = traffic(t0=9000.)
  sun = traffic(t0=20000., lighting='sunAhead')
  logs = [write_log(tmp_path / 'rlog1', [event(a)]), write_log(tmp_path / 'rlog2', [event(b), event(west, heading=92.)]),
          write_log(tmp_path / 'rlog3', [event(sun)])]
  assert jv.main(['--surveys', str(src), '--out', str(out), *logs]) == 0
  got = sorted((s['approachBearingDeg'], s['lighting'], s['passes'], s['cars']['left']) for s in stamps(out))
  assert got == [(0., 'day', 2, 10), (0., 'sunAhead', 1, 10), (92., 'day', 1, 10)]


def test_refusals(tmp_path, capsys):
  # step S1's setback unmeasured: no reach was measured from a known camera position
  code, _, out = run(tmp_path, [event(traffic(), setback_measured=False)])
  assert code == 1 and stamps(out) == [] and 'setback' in capsys.readouterr().out
  # no survey entry for the node: nothing to stamp
  code, _, out = run(tmp_path, [event(traffic(), node=12345)])
  assert code == 1 and 'no survey entry' in capsys.readouterr().out
  # the output may not be the survey file itself
  src = survey_file_with_entry(tmp_path)
  with pytest.raises(SystemExit):
    jv.main(['--surveys', str(src), '--out', str(src), write_log(tmp_path / 'rlog', [event(traffic())])])


def test_the_watchs_own_event_reads_back(tmp_path, capsys):
  # the event junction_watch.py logs, as cloudlog records it, is what this tool reads: one car through at 8 m/s, its first
  # track settled 4 s into the stand, judged (and failed: through at speed, and one car is not ten)
  from unittest import mock
  from openpilot.selfdrive.turnpilot import junction_watch
  from openpilot.selfdrive.turnpilot.tests.test_junction_watch import NOON, run_watch
  from openpilot.selfdrive.turnpilot.tests.test_visibility import vis_file
  watch = junction_watch.JunctionWatch()
  with mock.patch.object(junction_watch.cloudlog, 'event') as event:
    run_watch(watch, 200, lambda t: 40. - 8. * (t - 4.) if 4. <= t <= 8.5 else None)
    watch.update(None, 0., [], vis_file(), None, NOON)
  (c,) = [c for c in event.call_args_list if c.args[0] == jv.EVENT]
  logged = json.loads(json.dumps({'event': c.args[0], **c.kwargs}))
  code, _, out = run(tmp_path, [logged])
  text = capsys.readouterr().out
  assert code == 1 and stamps(out) == []
  assert 'track 7 (car, left lane 1)' in text and 'through its line at speed' in text and 'left: 1 cross motor vehicles' in text
