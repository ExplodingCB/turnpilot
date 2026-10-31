"""The four stop-sign modes before vision (TurnPilotStopSignGo off, shadow, trial, auto at e93750b), pinned frame by frame: the
closed-loop StopSim scenarios of test_turnpilotd_stop.py, each frame's plan reduced to a tuple (the prompt, reason, stop, caps,
G5's go cause, the map control's action, the detector view and every turnpilotPlan.stopGo field of e93750b's schema) and
hashed, and routing/certify.assess's verdict over test_certify's fixtures. A later change that alters what any of the four
modes does fails here; vision is a mode of its own and isn't pinned.

Regenerate (only for a change meant to alter one of the four): TP_GOLDEN_WRITE=1 pytest this file."""
import hashlib
import json
import os
from pathlib import Path

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.routing import certify as ce
from openpilot.selfdrive.turnpilot.routing.tests import test_certify as tc
from openpilot.selfdrive.turnpilot.tests import test_turnpilotd_stop as ts

GOLDEN = Path(__file__).parent / 'stop_go_modes_golden.json'
MODES = ('off', 'shadow', 'trial', 'auto')
# turnpilotPlan.stopGo's fields at e93750b, named so a field added later never changes a hash
STOP_GO_FIELDS = ('state', 'reason', 'ledger', 'wouldGo', 'maneuver', 'arrivalRank', 'dEdgeMin', 'dEdgeMax', 'committed', 'inLane',
                  'retries', 'creptM', 'sideViewsLive', 'mapsValid', 'mode', 'clearS', 'goT', 'takeoverS', 'withdrawn', 'positionM',
                  'chime', 'fault', 'alternate')
LANE_FIELDS = ('role', 'index', 'approach', 'rSeen', 'rReq', 'binding', 'shielded')


def _v(x):
  if isinstance(x, bool):
    return x
  if isinstance(x, float):
    return round(x, 3)
  if isinstance(x, int):
    return x
  return str(x)


def frame_tuple(p) -> list:
  g = p.stopGo
  out = [_v(p.controlPrompt), _v(p.reason), _v(p.shouldStop), _v(p.speedCap), _v(p.accelCap), _v(p.goCertified.cause),
         _v(p.mapControl.action), _v(p.detectorView)]
  out += [_v(getattr(g, k)) for k in STOP_GO_FIELDS]
  out.append([[_v(getattr(ln, k)) for k in LANE_FIELDS] for ln in g.lanes])
  return out


def digest(plans) -> str:
  return hashlib.sha256(json.dumps([frame_tuple(p) for p in plans], separators=(',', ':')).encode()).hexdigest()


# ------------------------------------------------------------------------------------------------------------ scenarios

def c1_empty(tmp_path, mode):
  sim = ts.StopSim(tmp_path, mode=mode)
  ts.run_to_line(sim)
  sim.run(14.)
  return sim


def o11_no_camera(tmp_path, mode):
  sim = ts.StopSim(tmp_path, mode=mode, vis=tc.visibility(tmp_path, cameraSetbackM=None))
  ts.run_to_line(sim)
  sim.run(4.)
  return sim


def c1_v25_waiting_car(tmp_path, mode):
  sim = ts.StopSim(tmp_path, mode=mode, model='goes')
  ts.run_to_line(sim)
  sim.tracks = lambda s: ts.shield_tracks(s, 'left')
  sim.run(20.)
  return sim


def v9_v10_deadlock(tmp_path, mode):
  sim = ts.StopSim(tmp_path, mode=mode, model='goes')
  sim.tracks = lambda s: ts.shield_tracks(s, 'left')
  ts.run_to_line(sim)
  sim.run(20.)
  return sim


def o2_roll_up(tmp_path, mode):
  sim = ts.StopSim(tmp_path, mode=mode, model='holds')
  sim.tp.go_override = False
  base = sim.planner

  def planner(s, plan):
    a = base(s, plan)
    room = ts.S_LINE - 4.3 - s.s
    if s.v > 0.01 and room < 25.:
      a = min(a, -s.v * s.v / (2. * max(room, 0.2)))
    return a
  sim.accel = planner
  while not (sim.v < 0.02 and sim.s > ts.S_LINE - 30.):
    sim.step()
    assert sim.t < 80.
  sim.accel = base
  sim.run(22.)
  return sim


def o17_withdrawn_at_speed(tmp_path, mode):
  sim = ts.StopSim(tmp_path, mode=mode, lag_s=0.4, model='goes')
  ts.run_to_line(sim)
  for _ in range(round(14. / DT_MDL)):
    p = sim.step()
    if str(p.stopGo.state) == 'going' and sim.v > 1.2:
      break
  sim.boxes = lambda s: {v: [ts.raw_box(65., conf=0.5, xy=(8., 17.), view='sideLeft')] for v in ts.CYCLE}
  sim.run(5.)
  return sim


def v3_downgrade(tmp_path, mode):
  sim = ts.StopSim(tmp_path, mode=mode)
  sim.tp.params = ts.OverridesParam(None)
  ts.run_to_line(sim)
  t0 = sim.t_cam + 1.

  def tracks(s, t0=t0):
    d = 45. - 8. * (s.t_cam - t0)
    return ts.lane_tracks(s, 'left', 41, d) if s.t_cam >= t0 and d > -12. else []
  sim.tracks = tracks
  sim.run(20.)
  return sim


SCENARIOS = {'C1_empty': c1_empty, 'O11_no_camera': o11_no_camera, 'C1_V25_waiting_car': c1_v25_waiting_car,
             'V9_V10_deadlock': v9_v10_deadlock, 'O2_roll_up': o2_roll_up, 'O17_withdrawn_at_speed': o17_withdrawn_at_speed,
             'V3_downgrade': v3_downgrade}


# --------------------------------------------------------------------------------------------------------- the verdicts

def certify_cases(tmp_path) -> dict:
  """assess over test_certify's fixtures for each mode: the surveyed validated all-way, its two-way, the turns, no table, the
  shipped file, no launch, night, side views off, a 35 mph cross road, a downgrade's override, a divided road."""
  vis = tc.visibility(tmp_path)
  shipped = tc.vt.load(tc.vt.SHIPPED)
  night = tc.utc(2026, 10, 29, 3, 0)
  cases = {
    'allway': (tc.junction(), 'straight', vis, [tc.survey()], tc.ETA, {}),
    'left': (tc.junction(maneuver='left'), 'left', vis, [tc.survey()], tc.ETA, {}),
    'right': (tc.junction(maneuver='right'), 'right', vis, [tc.survey()], tc.ETA, {}),
    'twoway': (tc.junction(tiers={'left': 'no', 'right': 'no'}), 'straight', vis, [tc.survey()], tc.ETA, {}),
    'unsurveyed': (tc.junction(surveyed=False), 'straight', vis, [], tc.ETA, {}),
    'noTable': (tc.junction(), 'straight', None, [tc.survey()], tc.ETA, {}),
    'shipped': (tc.junction(), 'straight', shipped, [tc.survey()], tc.ETA, {}),
    'noLaunch': (tc.junction(), 'straight', tc.visibility(tmp_path, launchVerified=None), [tc.survey()], tc.ETA, {}),
    'night': (tc.junction(), 'straight', vis, [tc.survey()], night, {}),
    'noClock': (tc.junction(), 'straight', vis, [tc.survey()], None, {}),
    'sideOff': (tc.junction(), 'straight', vis, [tc.survey()], tc.ETA, {'side_views': False}),
    'mph35': (tc.junction(mph=35), 'straight', vis, [tc.survey(mph=35)], tc.ETA, {}),
    'divided': (tc.junction(divided=True), 'straight', vis, [tc.survey()], tc.ETA, {}),
    'unknown': (tc.junction(), 'unknown', vis, [tc.survey()], tc.ETA, {}),
  }
  out = {}
  for mode in MODES:
    for name, (j, m, v, s, eta, rt) in cases.items():
      kw = {'mode': mode, 'side_views': True}
      kw.update(rt)
      verdict = ce.assess(j, m, v, s, eta, ce.Runtime(**kw))
      out[f'{mode}:{name}'] = [verdict.certification, verdict.reason, verdict.detail, sorted(verdict.rows), verdict.validated,
                               [[x.role, x.index, x.movement, round(x.need, 3), round(x.reach, 3), x.stopping] for x in verdict.lanes]]
  return out


# ------------------------------------------------------------------------------------------------------------- the tests

def _golden() -> dict:
  return json.loads(GOLDEN.read_text()) if GOLDEN.exists() else {}


def _write(key: str, value) -> None:
  data = _golden()
  data[key] = value
  GOLDEN.write_text(json.dumps(data, indent=1, sort_keys=True) + '\n')


WRITE = os.environ.get('TP_GOLDEN_WRITE') == '1'


@pytest.mark.parametrize('mode', MODES)
@pytest.mark.parametrize('scenario', list(SCENARIOS))
def test_the_four_modes_closed_loop_traces_are_unchanged(tmp_path, scenario, mode):
  sim = SCENARIOS[scenario](tmp_path, mode)
  got = {'frames': len(sim.plans()), 'sha256': digest(sim.plans())}
  key = f'trace:{scenario}:{mode}'
  if WRITE:
    _write(key, got)
  assert _golden().get(key) == got, key


def test_the_four_modes_certify_verdicts_are_unchanged(tmp_path):
  got = certify_cases(tmp_path)
  if WRITE:
    _write('certify', got)
  assert _golden().get('certify') == got
