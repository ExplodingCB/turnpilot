"""TurnPilot's cloudlog events in a run's rlogs (scenarios/events.py): logmessaged publishes cloudlog's records as logMessage
(launch_openpilot.sh keeps it with loggerd since b6ff0d9), and TurnPilot names its events two ways, 'turnpilot.<name>'
(turnpilot.go_not_certified, turnpilot.hard_brake) and 'turnpilot_<name>' (turnpilot_turn_skipped, turnpilot_turn_missed):
a reader matching only the first misses the skip and the missed turn (build findings H1)."""
import json

from openpilot.cereal import messaging
from openpilot.tools.lib.logreader import save_log
from openpilot.tools.sim.bridge.beamng.scenarios import events


def record(msg, created=1000., level='INFO'):
  return json.dumps({'msg': msg, 'ctx': {}, 'level': level, 'created': created})


def log_msg(text):
  return messaging.new_message(None, valid=True, logMessage=text).as_reader()


def test_both_spellings_are_turnpilots():
  for name in ('turnpilot.go_not_certified', 'turnpilot_turn_skipped', 'turnpilot_turn_missed', 'turnpilot.hard_brake'):
    assert events.is_turnpilot_event(name), name
  for name in ('turnpilotd: TurnPilotLeftRangeM unknown', 'turnpilot', 'selfdrive_event', 'go_not_certified', None, 3):
    assert not events.is_turnpilot_event(name), name


def test_an_event_is_read_with_its_fields_and_wall_time():
  e = events.turnpilot_event(record({'event': 'turnpilot_turn_missed', 'error': True, 'turned_deg': 9.9}, created=1234.5, level='ERROR'))
  assert e == {'event': 'turnpilot_turn_missed', 'wall': 1234.5, 'level': 'ERROR', 'error': True, 'turned_deg': 9.9}
  e = events.turnpilot_event(record({'event': 'turnpilot.go_not_certified', 'why': 'lead', 'lead_x': 14.7}))
  assert (e['event'], e['why'], e['lead_x']) == ('turnpilot.go_not_certified', 'lead', 14.7)


def test_anything_else_is_not_an_event():
  for text in (record({'event': 'controlsd_lag'}), record('turnpilotd: plain text'), record(['turnpilot.x', 'args']), '{not json',
               json.dumps(['turnpilot.go_end']), json.dumps({'level': 'INFO'})):
    assert events.turnpilot_event(text) is None, text


def test_events_in_a_log_stream_in_order_both_spellings():
  msgs = [messaging.new_message('carState').as_reader(),
          log_msg(record({'event': 'turnpilot.go_certified', 'cause': 'turnRelease'}, created=10.)),
          log_msg(record({'event': 'controlsd_lag'}, created=10.5)),
          log_msg(record({'event': 'turnpilot_turn_skipped', 'reason': 'tooFast'}, created=11.)),
          log_msg('{broken')]
  got = events.turnpilot_events(msgs)
  assert [(e['event'], e['wall']) for e in got] == [('turnpilot.go_certified', 10.), ('turnpilot_turn_skipped', 11.)]
  assert got[1]['reason'] == 'tooFast'


def test_errorlogmessage_duplicates_are_not_read_twice():
  # logmessaged sends an error record on logMessage and again on errorLogMessage: one event
  text = record({'event': 'turnpilot_turn_missed', 'error': True}, level='ERROR')
  msgs = [log_msg(text), messaging.new_message(None, valid=True, errorLogMessage=text).as_reader()]
  assert len(events.turnpilot_events(msgs)) == 1


def test_a_run_dirs_rlogs_read_with_t_since_engagement(tmp_path, capsys):
  seg = tmp_path / 'rlogs' / '00000009--abc--0'
  seg.mkdir(parents=True)
  (tmp_path / 'rlogs' / 'boot').mkdir()
  (tmp_path / 'rlogs' / 'boot' / 'bootlog').write_bytes(b'')  # the log root's boot dir, copied with the run: no rlog
  save_log(str(seg / 'rlog'), [log_msg(record({'event': 'turnpilot.go_not_certified', 'why': 'lead'}, created=105.25)),
                               log_msg(record({'event': 'turnpilot_turn_missed', 'turned_deg': 9.9}, created=112.)),
                               log_msg(record({'event': 'turnpilot.hard_brake', 'reason': 'x'}, created=95.))], compress=False)
  (tmp_path / 'result.json').write_text(json.dumps({'runs': [{'run': 1, 't_start_wall': 90., 't_engaged_wall': 100.}]}))
  got = events.run_events(tmp_path)
  assert [(e['event'], e.get('t')) for e in got] == [('turnpilot.hard_brake', -5.), ('turnpilot.go_not_certified', 5.25),
                                                    ('turnpilot_turn_missed', 12.)]
  assert all(e['run'] == 1 for e in got)
  events.main([str(tmp_path), '--match', 'turn_missed'])
  out = capsys.readouterr().out
  assert 'turnpilot_turn_missed' in out and 'go_not_certified' not in out and '12.00' in out


def test_each_event_is_timed_from_its_own_runs_engagement(tmp_path):
  seg = tmp_path / 'rlogs' / '00000001--abc--0'
  seg.mkdir(parents=True)
  save_log(str(seg / 'rlog'), [log_msg(record({'event': 'turnpilot.go_end'}, created=t)) for t in (101., 205., 50., 305.)], compress=False)
  (tmp_path / 'result.json').write_text(json.dumps({'runs': [{'run': 1, 't_engaged_wall': 100., 't_start_wall': 90.},
                                                             {'run': 2, 't_engaged_wall': 200., 't_start_wall': 190.},
                                                             {'run': 3, 't_engaged_wall': None, 't_start_wall': 300.}]}))
  got = events.run_events(tmp_path)
  # before every run's start (the previous segment, copied with the run): no run, no t; a run never engaged: no t
  assert [(e['run'], e.get('t')) for e in got] == [(None, None), (1, 1.), (2, 5.), (3, None)]
