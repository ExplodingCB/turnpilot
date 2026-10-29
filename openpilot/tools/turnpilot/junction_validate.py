#!/usr/bin/env python3
"""Validate surveyed all-way stops from the car's own logs (STOP_SIGNS.md 4.3, step S3; SG3b) and write the stamps.

  junction_validate.py --out OUT.json [--surveys surveyed_junctions.json] [--exclude T=REASON ...] RLOG_OR_DIR ...

It reads every turnpilot.junction_evidence event turnpilotd logged (junction_watch.py: one per stop or yield it swept, with
openpilot engaged or not) from the rlogs given (files, or directories searched for rlog files), groups them by the held
control's OSM node and the approach it was held from (approachBearingDeg, within 35 deg), and judges each group per
lighting row (each car's row at its first track; day or sunAhead, the rows a stamp is written for). A row passes when:

- the camera setback was measured (step S1) in every pass, so first-track ranges were taken from a known camera position;
- every binding side (each conflicting approach the events list: left, right, opposite) had at least VALIDATE_CARS cross
  motor vehicles approaching it, counted only when our car had stood at its stop point SETTLE_S before their first track
  (they were seen from the stop point) and not excluded;
- every such vehicle first tracked inside its lane's R_req plus latency (needM) while moving toward the junction is late
  (a car the camera missed looks exactly like one that entered the lane late) and fails the row, unless --exclude names it
  by its log time (its first track's camera time, which this tool prints) with a reason, recorded in the stamp: only after
  looking at the clip and seeing it enter the lane inside that distance (a driveway, a turn in, a parked car pulling out);
- every motor vehicle that approached stopped at its line, and none went through it at speed (the survey may be stale: the
  junction fails for Kellen's review);
- per lane, the longest gap between consecutive detections of a counted vehicle while inside its need is at most CLEAR_S / 3
  (CRITIQUE REQUIRED 2), and every conflicting lane had a counted vehicle.

A passing row's stamp goes into the matched survey entry's validations, in a copy of the survey file (--out; never the file
itself): {lighting, date, passes, approachBearingDeg, cars {side: n}, minFirstTrackM and maxDropoutS {side: [per lane, nearest
first]} (first-track ranges at the pessimistic distance end of section 1.2, the gaps unrounded), allStopped, verdict pass,
exclusions [{logT, reason}], tool}. routing/certify.py reads it (pick_stamp). Why each failing row fails is printed. Exit 0
when every row with evidence passed and was stamped, 1 otherwise.
"""
import argparse
import json
import sys
from datetime import date
from pathlib import Path

from openpilot.selfdrive.turnpilot import stop_rules as sr
from openpilot.selfdrive.turnpilot.maps import surveys as survey_file

EVENT = 'turnpilot.junction_evidence'
MOTOR = ('car', 'truck', 'bus', 'motorcycle')
SIDES = ('left', 'right', 'opposite')
STAMP_ROWS = ('day', 'sunAhead')
SETTLE_S = 3.0          # s our car must have stood at its stop point before a car's first track counts toward reach
APPROACH_TOL_DEG = 35.  # deg: passes held from approaches this close are one approach (certify's LEG_HEADING_TOL)
EXCLUDE_TOL_S = 0.05    # s: --exclude's log time matches a first track this close


def rlog_paths(args: list[str]) -> list[Path]:
  out = []
  for a in args:
    p = Path(a)
    if p.is_dir():
      out += sorted(q for q in p.rglob('*') if q.is_file() and q.name.split('.')[0] == 'rlog')
    else:
      out.append(p)
  return out


def read_events(paths: list[Path]) -> list[dict]:
  """Every junction evidence event in the logs, each with the log it came from."""
  from openpilot.tools.lib.logreader import LogReader
  out = []
  for p in paths:
    for msg in LogReader(str(p)):
      if msg.which() != 'logMessage':
        continue
      try:
        rec = json.loads(msg.logMessage)
      except ValueError:
        continue
      m = rec.get('msg') if isinstance(rec, dict) else None
      if isinstance(m, dict) and m.get('event') == EVENT:
        out.append({**m, '_log': str(p)})
  return out


def group(events: list[dict]) -> list[dict]:
  """[{node, heading, events}] by OSM node, then by the approach held from (within APPROACH_TOL_DEG)."""
  groups: list[dict] = []
  for e in events:
    node = int(e.get('osmNodeId') or 0)
    if not node:
      continue
    h = float(e.get('approachBearingDeg', 0.))
    g = next((g for g in groups if g['node'] == node and sr.angle_diff(g['heading'], h) <= APPROACH_TOL_DEG), None)
    if g is None:
      groups.append({'node': node, 'heading': h, 'events': [e]})
    else:
      g['events'].append(e)
  return groups


def parse_excludes(items: list[str]) -> list[dict]:
  out = []
  for it in items or []:
    t, sep, reason = it.partition('=')
    if not sep or not reason.strip():
      raise SystemExit(f"--exclude {it!r}: give the log time and a reason, T=REASON")
    out.append({'logT': float(t), 'reason': reason.strip()})
  return out


def judge(g: dict, row: str, excludes: list[dict]) -> tuple[dict | None, list[str], list[str]]:
  """(the stamp or None, why it fails, notes) for one group's lighting row (module docstring)."""
  why, notes, used = [], [], []
  events = [e for e in g['events'] if any(c.get('lighting') == row for c in e.get('cars') or [])]
  if not all(e.get('setbackMeasured') for e in events):
    why.append("the camera setback wasn't measured (step S1) in every pass: no first-track range is from a known camera position")
  lanes: dict[tuple, float] = {}
  for e in events:
    for ln in e.get('lanes') or []:
      lanes[(ln['role'], int(ln['index']))] = float(ln['needM'])
  cars = [c for e in events for c in e.get('cars') or [] if c.get('lighting') == row and c.get('cls') in MOTOR]

  def excluded(c):
    hit = next((x for x in excludes if abs(x['logT'] - float(c['firstT'])) <= EXCLUDE_TOL_S), None)
    if hit is not None and hit not in used:
      used.append(hit)
    return hit is not None
  live = [c for c in cars if not excluded(c)]
  counted = [c for c in live if c.get('approaching') is True and float(c.get('settledS', -1.)) >= SETTLE_S]
  for c in live:
    tag = f"track {c['trackId']} ({c['cls']}, {c['role']} lane {c['index']}) at log time {float(c['firstT']):.2f}"
    if c.get('throughAtSpeed'):
      why.append(f"{tag} went through its line at speed (vMax {float(c.get('vMax', 0.)):.1f} m/s): the survey may be stale")
    elif c.get('approaching') is True and not c.get('stoppedAtLine'):
      why.append(f"{tag} approached and didn't stop at its line")
    settled = float(c.get('settledS', -1.)) >= SETTLE_S
    if settled and c.get('approaching') is not False and float(c['firstRangeM']) < float(c['needM']):
      why.append(f"{tag} was late: first tracked {float(c['firstRangeM']):.1f} m down its lane, inside {float(c['needM']):.1f} m " +
                 f"(R_req plus latency); --exclude {float(c['firstT']):.2f}=REASON only after the clip shows it entered there")
  sides = sorted({r for r, _ in lanes}, key=SIDES.index)
  count = {s: sum(1 for c in counted if c['role'] == s) for s in sides}
  for s in sides:
    if count[s] < sr.VALIDATE_CARS:
      why.append(f"{s}: {count[s]} cross motor vehicles seen from the stop point, {sr.VALIDATE_CARS} needed")
  first: dict[str, list] = {s: [] for s in sides}
  drop: dict[str, list] = {s: [] for s in sides}
  for (role, index) in sorted(lanes, key=lambda k: (SIDES.index(k[0]), k[1])):
    on = [c for c in counted if c['role'] == role and int(c['index']) == index]
    if not on:
      why.append(f"{role} lane {index}: no counted vehicle")
      continue
    gap = max(float(c['maxGapInsideS']) for c in on)
    if gap > sr.VALIDATE_DROPOUT_S + 1e-9:
      worst = max(on, key=lambda c: float(c['maxGapInsideS']))
      why.append(f"{role} lane {index}: dropout {gap:.2f} s inside R_req (track {worst['trackId']} at log time " +
                 f"{float(worst['firstT']):.2f}), over CLEAR_S / 3 ({sr.VALIDATE_DROPOUT_S:.2f} s)")
    while len(first[role]) < index - 1:  # a lane with nothing before it can't be skipped in a list
      first[role].append(None)
      drop[role].append(None)
    first[role].append(min(float(c['firstRangeM']) for c in on))
    drop[role].append(gap)
  unused = [x for x in excludes if x not in used]
  notes += [f"--exclude {x['logT']:.2f} matched no first track" for x in unused]
  if why:
    return None, why, notes
  stamp = {'lighting': row, 'date': date.today().isoformat(), 'passes': len(events), 'approachBearingDeg': g['heading'],
           'cars': count, 'minFirstTrackM': first, 'maxDropoutS': drop, 'allStopped': True, 'verdict': 'pass',
           'exclusions': [{'logT': x['logT'], 'reason': x['reason']} for x in used], 'tool': 'junction_validate.py'}
  return stamp, [], notes


def validate(events: list[dict], entries: list, excludes: list[dict]) -> tuple[list, bool]:
  """(the survey list with the passing stamps appended, all passed) and the report printed."""
  entries = json.loads(json.dumps(entries))
  ok = True
  for g in group(events):
    entry = next((e for e in entries if isinstance(e, dict) and e.get('osmNodeId') == g['node']), None)
    head = f"junction {g['node']} from {g['heading']:.0f} deg ({len(g['events'])} passes)"
    if entry is None:
      print(f"{head}: no survey entry for this node: nothing to stamp (survey it first, step S2)")
      ok = False
      continue
    rows = sorted({c.get('lighting', 'unknown') for e in g['events'] for c in e.get('cars') or []})
    for row in [r for r in rows if r not in STAMP_ROWS]:
      print(f"{head}: cars seen in {row} light aren't counted (no table for it)")
    for row in [r for r in rows if r in STAMP_ROWS] or ['day']:
      stamp, why, notes = judge(g, row, excludes)
      for n in notes:
        print(f"{head} {row}: note: {n}")
      if stamp is None:
        ok = False
        print(f"{head} {row}: FAIL")
        for w in why:
          print(f"  - {w}")
        continue
      entry.setdefault('validations', []).append(stamp)
      print(f"{head} {row}: pass, cars {stamp['cars']}, min first track {stamp['minFirstTrackM']}, max dropout {stamp['maxDropoutS']}")
  return entries, ok


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('logs', nargs='+', help='rlog files, or directories to search for them')
  ap.add_argument('--surveys', default=str(survey_file.SHIPPED), help='the survey file to copy (default: the shipped one)')
  ap.add_argument('--out', required=True, help='where to write the copy with the stamps (never the survey file itself)')
  ap.add_argument('--exclude', action='append', default=[], metavar='T=REASON',
                  help="a late car's log time (as printed) and why it doesn't count, after viewing its clip")
  args = ap.parse_args(argv)
  src, out = Path(args.surveys), Path(args.out)
  if out.resolve() == src.resolve():
    raise SystemExit("--out must be a copy, not the survey file itself")
  entries = json.loads(src.read_text())
  if not isinstance(entries, list):
    raise SystemExit(f"{src} isn't a list of survey entries")
  events = read_events(rlog_paths(args.logs))
  print(f"{len(events)} junction evidence events in {len(args.logs)} log arguments")
  stamped, ok = validate(events, entries, parse_excludes(args.exclude))
  out.write_text(json.dumps(stamped, indent=1) + '\n')
  return 0 if ok and events else 1


if __name__ == '__main__':
  sys.exit(main())
