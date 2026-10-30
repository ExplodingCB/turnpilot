#!/usr/bin/env python3
"""TurnPilot's cloudlog events in a run's rlogs: why TurnPilot did what it did (turnpilot.go_not_certified and its why,
turnpilot.hard_brake, turnpilot_turn_skipped, ...).

logmessaged publishes cloudlog's records as logMessage, which loggerd writes to the rlog (launch_openpilot.sh keeps both:
before b6ff0d9 the BeamNG launch blocked logmessaged and no record reached a run's rlogs). Each is cloudlog's JSON record;
an event is one whose msg is a dict with 'event' (cloudlog.event), and 'created' is its wall time, which the runner's
t_start_wall and t_engaged_wall are on. TurnPilot names its events two ways, 'turnpilot.<name>' and 'turnpilot_<name>'
(turnpilot_turn_skipped, turnpilot_turn_missed, turnpilot_lane_mismatch): a reader matching only one misses the others
(BeamNG build findings H1). An error record also goes out on errorLogMessage; only logMessage is read, so each event once.

  python -m openpilot.tools.sim.bridge.beamng.scenarios.events <run dir | rlog | dir of rlogs> ... [--match go_]
A run dir (the runner's, with result.json and the run's rlogs under it, as the demo smoke copies them) gives each event its
run and t, s since that run's engagement (None before the run engaged); anything else, wall time only.
"""
import argparse
import json
from pathlib import Path

PREFIXES = ('turnpilot.', 'turnpilot_')


def is_turnpilot_event(name) -> bool:
  return isinstance(name, str) and any(name.startswith(p) and len(name) > len(p) for p in PREFIXES)


def turnpilot_event(text: str) -> dict | None:
  """The TurnPilot event in a logMessage record, as {'event', 'wall', 'level', **its fields}; None for anything else."""
  try:
    rec = json.loads(text)
  except ValueError:
    return None
  m = rec.get('msg') if isinstance(rec, dict) else None
  if not isinstance(m, dict) or not is_turnpilot_event(m.get('event')):
    return None
  return {'event': m['event'], 'wall': rec.get('created'), 'level': rec.get('level'), **{k: v for k, v in m.items() if k != 'event'}}


def turnpilot_events(msgs) -> list[dict]:
  """Every TurnPilot event in a log stream (log events, e.g. an rlog's LogReader), in stream order."""
  out = []
  for m in msgs:
    if m.which() == 'logMessage' and (e := turnpilot_event(m.logMessage)) is not None:
      out.append(e)
  return out


def rlog_files(paths) -> list[Path]:
  """The rlogs in paths: files as given, dirs searched (a log root's boot dir has none)."""
  out: list[Path] = []
  for a in paths:
    p = Path(a)
    out += sorted(q for q in p.rglob('*') if q.is_file() and q.name.split('.')[0] == 'rlog') if p.is_dir() else [p]
  return out


def place(evs: list[dict], runs: list[dict]) -> list[dict]:
  """Each event's run (the last one started by its wall time: t_start_wall, else t_engaged_wall) and t, s since that run's
  engagement (None if it never engaged); before every run, None for both."""
  starts = sorted(((r.get('t_start_wall') or r.get('t_engaged_wall'), r) for r in runs
                   if (r.get('t_start_wall') or r.get('t_engaged_wall')) is not None), key=lambda x: x[0])
  for e in evs:
    run = None
    for t0, r in starts:
      if e['wall'] is not None and e['wall'] >= t0:
        run = r
    e['run'] = None if run is None else run.get('run')
    eng = None if run is None else run.get('t_engaged_wall')
    e['t'] = None if eng is None or e['wall'] is None else round(e['wall'] - eng, 3)
  return evs


def run_events(*paths) -> list[dict]:
  """The TurnPilot events in these rlogs (or dirs of them), in wall-time order; a dir with the runner's result.json has
  each placed in its run (place)."""
  from openpilot.tools.lib.logreader import LogReader
  evs = [e for f in rlog_files(paths) for e in turnpilot_events(LogReader(str(f), sort_by_time=True))]
  evs.sort(key=lambda e: e['wall'] if e['wall'] is not None else float('inf'))
  runs = []
  for p in map(Path, paths):
    if (p / 'result.json').is_file():
      runs += json.loads((p / 'result.json').read_text()).get('runs', [])
  return place(evs, runs) if runs else evs


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
  ap.add_argument('paths', nargs='+', help='a run dir (result.json and rlogs under it), rlogs, or dirs of rlogs')
  ap.add_argument('--match', default='', help='only events whose name contains this')
  args = ap.parse_args(argv)
  for e in run_events(*args.paths):
    if args.match not in e['event']:
      continue
    fields = {k: v for k, v in e.items() if k not in ('event', 'wall', 'level', 'run', 't')}
    when = f"run {e['run']} t={e['t']:7.2f}" if e.get('t') is not None else f"run {e.get('run')} wall={e['wall']}"
    print(f"{when}  {e['event']}  {json.dumps(fields, default=str)}")


if __name__ == "__main__":
  main()
