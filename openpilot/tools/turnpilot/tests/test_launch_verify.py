"""launch_verify.py (STOP_SIGNS.md 2.6, 8.3, step S4a; SG4a): the decider's trial goes read from turnpilotPlan.stopGo, a pass at
9 of 10 taken over inside the gate, the longitudinal delay from carControl.actuators.accel against carState.aEgo over the
stops and launches, and launchVerified and actuatorDelayS written into a copy of visibility.json that visibility_table.py
loads (launch_passed) and stop_rules.t_abort reads."""
import json

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.turnpilot import visibility_table as vt
from openpilot.tools.lib.logreader import save_log
from openpilot.tools.turnpilot import launch_verify as lv

PLAN_DT, CAR_DT = 0.05, 0.01


def stamped(msg, t):
  msg.logMonoTime = int(round(t * 1e9))
  return msg.as_reader()


def plan(t, state, mode='trial', takeover=-1., retries=0, withdrawn='', position=-0.5, fault=''):
  msg = messaging.new_message('turnpilotPlan')
  g = msg.turnpilotPlan.stopGo
  g.state, g.mode, g.maneuver, g.takeoverS, g.retries, g.withdrawn = state, mode, 'straight', takeover, retries, withdrawn
  g.positionM, g.dEdgeMin, g.dEdgeMax, g.fault = position, 7., 12., fault
  return stamped(msg, t)


def drive(n_goes=10, n_taken=9, lag=0.4, mode='trial', interferer=0):
  """A trial session: n_goes goes 20 s apart, the first n_taken taken over 0.8 s in (committed 2.5 s in), the rest withdrawn
  at G5's window; interferer more goes withdrawn for a car (not the model's). Each go is a launch at +1.0 m/s^2 for 3 s and a
  stop at -1.5 m/s^2, the car's aEgo the command lag s late."""
  msgs, t_end = [], 20. * (n_goes + interferer) + 12.
  spec = [('taken' if k < n_taken else 'held') for k in range(n_goes)] + ['car'] * interferer
  retries = 0
  for k, kind in enumerate(spec):
    t0 = 5. + 20. * k
    for i in range(round(18. / PLAN_DT)):
      t = t0 + i * PLAN_DT
      dt = t - (t0 + 1.)
      if dt < 0.:
        msgs.append(plan(t, 'observe', mode, retries=retries))
      elif kind == 'taken':
        state = 'going' if dt < 2.5 else 'committed' if dt < 6. else 'done' if dt < 6.5 else 'off'
        msgs.append(plan(t, state, mode, takeover=0.8 if dt >= 0.8 else -1., retries=retries, position=-0.5 + max(dt, 0.)))
      elif kind == 'held':
        if dt < 3.:
          msgs.append(plan(t, 'going', mode, retries=retries))
        else:
          if dt < 3. + PLAN_DT:
            retries += 1
          msgs.append(plan(t, 'observe', mode, retries=retries, withdrawn='window', position=1.5))
      else:
        if dt < 1.:
          msgs.append(plan(t, 'going', mode, retries=retries))
        else:
          if dt < 1. + PLAN_DT:
            retries += 1
          msgs.append(plan(t, 'observe', mode, retries=retries, withdrawn='raw:left1', position=0.5))

  def command(t):
    dt = (t - 6.) % 20.
    return 1.0 if dt < 3. else -1.5 if dt < 5. else 0.
  v = 0.
  for i in range(round(t_end / CAR_DT)):
    t = i * CAR_DT
    cc = messaging.new_message('carControl')
    cc.carControl.enabled = cc.carControl.longActive = True
    cc.carControl.actuators.accel = command(t)
    msgs.append(stamped(cc, t))
    a = command(t - lag) if t >= lag else 0.
    v = max(v + a * CAR_DT, 0.)
    cs = messaging.new_message('carState')
    cs.carState.aEgo, cs.carState.vEgo = a, v
    msgs.append(stamped(cs, t))
  msgs.sort(key=lambda m: m.logMonoTime)
  return msgs


def run(tmp_path, msgs, **kw):
  log = tmp_path / 'rlog'
  save_log(str(log), msgs, compress=False)
  src = tmp_path / 'visibility.json'
  raw = json.loads(vt.SHIPPED.read_text())
  raw.update(cameraSetbackM=1.9, lateralOffsetM=0.0, heightM=1.3, darkThreshold=45, veil='unmeasured', actuatorDelayS=None,
             lens={'model': 'equisolid', 'f': 452., 'k': 0.3, 'edgeDeg': {'left': 88.1, 'right': 88.1}},
             mask={'left': [[b, -30., 30.] for b in range(89)], 'right': [[b, -30., 30.] for b in range(89)]})
  src.write_text(json.dumps(raw))
  out = tmp_path / 'out.json'
  code = lv.main([str(log), '--vis', str(src), '--out', str(out), '--date', '2026-10-31', *kw.get('args', [])])
  return code, json.loads(out.read_text()), out


def test_ten_trial_goes_nine_taken_over_pass_and_the_stamp_is_what_the_certifier_reads(tmp_path):
  code, raw, out = run(tmp_path, drive(10, 9))
  assert code == 0
  stamp = raw['launchVerified']
  assert {k: stamp[k] for k in vt.LAUNCH_FIELDS} == {'date': '2026-10-31', 'goes': 10, 'takenOver': 9, 'maxTakeoverS': 0.8,
                                                     'verdict': 'pass'}
  v = vt.load(out)
  assert v.launch_passed and abs(v.actuator_delay_s - 0.4) <= 0.05


def test_eight_of_ten_fail_and_the_failing_stamp_is_written_too(tmp_path):
  code, raw, out = run(tmp_path, drive(10, 8))
  assert code == 1 and raw['launchVerified']['verdict'] == 'fail' and raw['launchVerified']['takenOver'] == 8
  assert not vt.load(out).launch_passed


@pytest.mark.parametrize('lag', [0.15, 0.4, 0.6])
def test_a_synthetic_lag_reads_back_within_0_05_s(tmp_path, lag):
  code, raw, _ = run(tmp_path, drive(10, 10, lag=lag))
  assert code == 0 and abs(raw['actuatorDelayS'] - lag) <= 0.05


def test_a_go_withdrawn_for_a_car_is_listed_never_counted_and_too_few_goes_never_pass(tmp_path):
  # two goes withdrawn for a box in the left lane say nothing about the model: 10 counted, 9 taken over, still a pass; with
  # only 9 goes the model could take over, it never is
  code, raw, _ = run(tmp_path, drive(10, 9, interferer=2))
  assert code == 0 and raw['launchVerified']['goes'] == 10 and raw['launchVerified']['undecided'] == 2
  code, raw, _ = run(tmp_path, drive(9, 9))
  assert code == 1 and raw['launchVerified']['verdict'] == 'fail'


def test_only_the_mode_asked_for_counts_and_an_unmeasured_delay_never_passes(tmp_path):
  # shadow goes don't happen (the decider never goes as shadow), but auto ones aren't step S4a's
  code, raw, _ = run(tmp_path, drive(10, 10, mode='auto'))
  assert code == 1 and raw['launchVerified']['goes'] == 0
  # no carControl or carState: the delay unmeasured, so no actuatorDelayS and no pass, however the goes went
  msgs = [m for m in drive(10, 10) if m.which() == 'turnpilotPlan']
  code, raw, _ = run(tmp_path, msgs)
  assert code == 1 and raw['launchVerified']['verdict'] == 'fail' and raw.get('actuatorDelayS') is None


def test_it_never_writes_over_the_visibility_file_itself(tmp_path):
  src = tmp_path / 'visibility.json'
  src.write_text(vt.SHIPPED.read_text())
  with pytest.raises(SystemExit):
    lv.main([str(tmp_path), '--vis', str(src), '--out', str(src)])
