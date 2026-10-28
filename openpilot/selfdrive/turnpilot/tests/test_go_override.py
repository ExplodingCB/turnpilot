"""TurnPilot's bounded go (go_override.py; DESIGN phase 4 decision 1 and its stop-sign amendment, G5): plannerd lifts the
e2e model's hold on a standing car only for a go turnpilotd certifies, at most +0.5 m/s^2, for at most 3 s, under 3 m/s,
and only while nothing else holds the car."""
import random
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.turnpilot.go_override import (GO_ACCEL, GO_CLEAR_M, GO_END_SPEED, GO_MODEL_BRAKE, GO_WINDOW_S, TAKEOVER_S,
                                                       TAKEOVER_V, GoCertifier, GoLift, Takeover, model_takes_over)

State = custom.TurnpilotPlan.State
T_IDXS = ModelConstants.T_IDXS


def test_schema_carries_the_certified_go_and_the_lift():
  # appended only: TurnpilotPlan @32 after unprotectedLeft @31, LongitudinalPlan @41-@42 after accelBoost @40
  causes = list(custom.TurnpilotPlan.GoCertified.Cause.schema.enumerants)
  assert causes == ['none', 'turnRelease', 'unprotectedLeft', 'stopSign', 'rollUp', 'creep', 'clearBox']
  fields = custom.TurnpilotPlan.schema.fields
  assert fields['goCertified'].proto.ordinal.explicit == 32
  assert list(custom.TurnpilotPlan.GoCertified.schema.fields) == ['cause', 'id', 't']
  lp = log.LongitudinalPlan.schema.fields
  assert lp['turnpilotGoLift'].proto.ordinal.explicit == 41 and lp['turnpilotGoEnd'].proto.ordinal.explicit == 42


# --- the inputs ---

def model(should_stop=False, plan=None, accel=0., lead_x=None, lead_prob=0.9):
  """A modelV2 reader: action (desiredAcceleration, shouldStop), a planned speed plan(t) on T_IDXS (none: no plan), and
  a lead lead_x m ahead."""
  msg = messaging.new_message('modelV2')
  m = msg.modelV2
  m.action.desiredAcceleration, m.action.shouldStop = float(accel), should_stop
  if plan is not None:
    m.velocity.x = [float(plan(t)) for t in T_IDXS]
  if lead_x is not None:
    m.init('leadsV3', 3)
    for lead in m.leadsV3:
      lead.prob, lead.x = float(lead_prob), [float(lead_x)] * 6
  return msg.as_reader().modelV2


HOLDING = {'should_stop': True, 'plan': lambda t: 0.}   # G0's stand: shouldStop true, no motion planned
GOING = {'should_stop': False, 'plan': lambda t: min(1.0 * t, 4.)}  # it means to go: 1 m/s^2 from rest


def go_plan(cause='turnRelease', go_id=1, t=0., shadow=False, state=State.turning, should_stop=False, accel_cap=10., speed_cap=100.):
  msg = messaging.new_message('turnpilotPlan')
  p = msg.turnpilotPlan
  p.state, p.shadow, p.shouldStop, p.accelCap, p.speedCap = state, shadow, should_stop, accel_cap, speed_cap
  if cause != 'none':
    p.goCertified.cause, p.goCertified.id, p.goCertified.t = cause, go_id, float(t)
  return msg.as_reader().turnpilotPlan


class FakeSM(dict):
  """SubMaster's dicts, every message received just now and valid."""
  def __init__(self, msgs, age=0., valid=True):
    super().__init__(msgs)
    self.alive = dict.fromkeys(msgs, True)
    self.valid = dict.fromkeys(msgs, valid)
    self.recv_time = dict.fromkeys(msgs, time.monotonic() - age)


def lift_sm(plan=None, model_v2=None, brake=False, gas=False, enabled=True, radar_lead=None, age=0.):
  msgs = {s: messaging.new_message(s) for s in ('carState', 'selfdriveState', 'radarState')}
  msgs['carState'].carState.brakePressed, msgs['carState'].carState.gasPressed = brake, gas
  msgs['selfdriveState'].selfdriveState.enabled = enabled
  if radar_lead is not None:
    lead = msgs['radarState'].radarState.leadOne
    lead.present, lead.dRel = True, float(radar_lead)
  sm = {s: getattr(m.as_reader(), s) for s, m in msgs.items()}
  sm['modelV2'] = model_v2 if model_v2 is not None else model(**HOLDING)
  if plan is not None:
    sm['turnpilotPlan'] = plan
  return FakeSM(sm, age=age)


FREE = (2.4, False)        # the mpc with no lead at a standstill (its fake lead 50 m on)
CRUISE = (2.0, False)      # the cruise candidate after a stand under a set speed
TP_FREE = (10., False)     # TurnPilot's candidate with no cap
E2E_HOLD = (0.0, True)     # the model holding the car


def lift(gl, sm, v=0., mpc=FREE, cruise=CRUISE, tp=TP_FREE, e2e=E2E_HOLD):
  return gl.update(sm, v, mpc, cruise, tp, e2e, DT_MDL)


# --- the takeover predicate ---

def test_takeover_predicate():
  """shouldStop false and the planned speed at least TAKEOVER_V from 1 s to 2 s into its plan (from a standstill no plan is
  at 0.5 m/s at once): the model means to keep the car moving."""
  assert TAKEOVER_V == 0.5
  assert not model_takes_over(model(**HOLDING))
  assert model_takes_over(model(**GOING))
  assert model_takes_over(model(plan=lambda t: 1.0))                  # moving and keeping on
  assert model_takes_over(model(plan=lambda t: 0.5 * t))              # 0.5 m/s^2 from rest: 0.5 m/s at 1 s
  assert not model_takes_over(model(plan=lambda t: 0.3 * t))          # lukewarm: 0.3 m/s at 1 s
  assert not model_takes_over(model(plan=lambda t: min(t, 2. - t)))   # goes, then stops by 2 s
  assert not model_takes_over(model(plan=lambda t: max(1. - t, 0.)))  # moving at 1 m/s, stopping within 2 s
  assert not model_takes_over(model(should_stop=True, plan=lambda t: 2.))  # its shouldStop vetoes the plan
  assert not model_takes_over(model(plan=None))                      # no plan: never a takeover


def test_takeover_holds_for_its_time():
  tk = Takeover()
  n = round(TAKEOVER_S / DT_MDL)
  assert [tk.update(model(**GOING), DT_MDL) for _ in range(n + 1)] == [False] * (n - 1) + [True, True]
  assert not tk.update(model(**HOLDING), DT_MDL)  # a frame of holding starts it over
  assert not any(tk.update(model(**GOING), DT_MDL) for _ in range(n - 1))


# --- plannerd's side: the lift ---

def test_lift_raises_the_held_e2e_candidate_to_go_accel_and_drops_its_stop():
  gl = GoLift()
  assert lift(gl, lift_sm(go_plan())) == (GO_ACCEL, False) == (0.5, False)
  assert gl.lifting
  # a model already asking for more keeps its own: the lift only ever raises, and never past GO_ACCEL
  assert lift(gl, lift_sm(go_plan()), e2e=(0.8, False)) == (0.8, False)
  assert lift(gl, lift_sm(go_plan()), e2e=(-0.3, True)) == (GO_ACCEL, False)


def test_no_go_no_lift():
  for sm in (lift_sm(None), lift_sm(go_plan('none')), lift_sm(go_plan(shadow=True)), lift_sm(go_plan(state=State.disabled)),
             lift_sm(go_plan(), age=1.), {'turnpilotPlan': go_plan()}):
    gl = GoLift()
    assert lift(gl, sm) == E2E_HOLD and not gl.lifting


def test_lift_ends_at_its_window_and_never_comes_back_for_that_go():
  gl = GoLift()
  n = round(GO_WINDOW_S / DT_MDL)
  out = [lift(gl, lift_sm(go_plan(t=i * DT_MDL))) for i in range(n + 40)]
  assert out[:n] == [(GO_ACCEL, False)] * n and all(o == E2E_HOLD for o in out[n:]) and gl.end == 'window'
  # the same go asserted on: nothing. A new go (a new id) gets its own window
  assert lift(gl, lift_sm(go_plan(go_id=2))) == (GO_ACCEL, False)
  # the plan's own clock bounds it too (a plannerd that came up mid-go)
  gl = GoLift()
  assert lift(gl, lift_sm(go_plan(go_id=3, t=GO_WINDOW_S))) == E2E_HOLD and gl.end == 'window'


def test_withdrawing_the_go_ends_the_lift_that_frame():
  gl = GoLift()
  assert lift(gl, lift_sm(go_plan())) == (GO_ACCEL, False)
  assert lift(gl, lift_sm(go_plan('none'))) == E2E_HOLD and gl.end == 'withdrawn' and not gl.lifting
  assert lift(gl, lift_sm(go_plan(go_id=1))) == E2E_HOLD  # the same go back again: over, never resumed
  # a stale plan (turnpilotd gone) is a withdrawal too
  gl = GoLift()
  lift(gl, lift_sm(go_plan()))
  assert lift(gl, lift_sm(go_plan(), age=1.)) == E2E_HOLD and gl.end == 'withdrawn'


def test_each_end_ends_it_for_good():
  """A pedal, a disengage, a lead inside GO_CLEAR_M, GO_END_SPEED, another candidate holding (TurnPilot's hold: a walker),
  or the model braking hard ends the go's lift at once, and it never comes back for that go."""
  # (end reason, the frame's lift_sm keywords, lift keywords)
  cases = [
    ('driver', {'brake': True}, {}), ('driver', {'gas': True}, {}), ('driver', {'enabled': False}, {}),
    ('lead', {'radar_lead': GO_CLEAR_M - 1.}, {}), ('lead', {'model_v2': model(**HOLDING, lead_x=GO_CLEAR_M - 1.)}, {}),
    ('speed', {}, {'v': GO_END_SPEED}), ('holder', {}, {'mpc': (0.07, True)}), ('holder', {}, {'mpc': (0.07, False)}),
    ('holder', {}, {'cruise': (-0.5, True)}), ('holder', {'plan': go_plan(should_stop=True)}, {'tp': (10., True)}),
    ('holder', {}, {'tp': (0., False)}), ('modelBrake', {}, {'e2e': (GO_MODEL_BRAKE - 0.1, False)}),
  ]
  for reason, sm_kw, kw in cases:
    gl = GoLift()
    assert lift(gl, lift_sm(go_plan())) == (GO_ACCEL, False)
    sm_kw = dict(sm_kw)
    plan = sm_kw.pop('plan', go_plan())
    out = lift(gl, lift_sm(plan, **sm_kw), **kw)
    assert out == kw.get('e2e', E2E_HOLD) and not gl.lifting and gl.end == reason, (reason, sm_kw, kw, gl.end)
    # whatever ended it going away: the go is over
    assert lift(gl, lift_sm(go_plan())) == E2E_HOLD, (reason, sm_kw, kw)


def test_a_lead_beyond_the_block_doesnt_end_it():
  gl = GoLift()
  assert lift(gl, lift_sm(go_plan(), radar_lead=GO_CLEAR_M + 5.)) == (GO_ACCEL, False)
  assert lift(gl, lift_sm(go_plan(), model_v2=model(**HOLDING, lead_x=GO_CLEAR_M + 5.))) == (GO_ACCEL, False)
  assert lift(gl, lift_sm(go_plan(), model_v2=model(**HOLDING, lead_x=5., lead_prob=0.2))) == (GO_ACCEL, False)


def test_takeover_ends_the_lift():
  gl = GoLift()
  n = round(TAKEOVER_S / DT_MDL)
  out = [lift(gl, lift_sm(go_plan(t=i * DT_MDL), model_v2=model(**GOING, accel=0.2)), e2e=(0.2, False)) for i in range(n + 5)]
  assert out[:n - 1] == [(GO_ACCEL, False)] * (n - 1) and all(o == (0.2, False) for o in out[n - 1:]) and gl.end == 'takeover'


def held(v, candidate):
  """An (accel, shouldStop) candidate that keeps a standing car standing (drive_helpers.should_stop) or says stop."""
  return candidate[1] or (v < 0.3 and candidate[0] < 0.1)


def test_lift_property_random():
  """Over random drives: the lift only ever raises the e2e candidate, to GO_ACCEL at most; never at or above GO_END_SPEED,
  with a lead inside GO_CLEAR_M, another candidate holding, a pedal, disengaged, for a go withdrawn, or past GO_WINDOW_S of
  its go (GO_WINDOW_S / DT_MDL frames of lift per go at most)."""
  rng = random.Random(5)
  lifted_frames = 0
  for _ in range(300):
    gl = GoLift()
    go_id, p_bad = rng.randint(1, 5), rng.choice([0., 0.01, 0.03, 0.1])
    lifts = {}
    for i in range(rng.randint(1, 120)):
      def bad(p=p_bad):
        return rng.random() < p
      v = rng.choice([0., 0.1, 0.5, 1.5, 2.9]) if not bad() else rng.choice([3.0, 6.])
      e2e = (rng.uniform(-0.9, 1.5) if not bad() else rng.uniform(-3., -0.9), rng.random() < 0.5)
      mpc = ((2.4, False) if not bad() else rng.choice([(0.07, False), (-1., False), (0.5, True)]))
      cruise = ((2., False) if not bad() else rng.choice([(-0.5, True), (0.05, False)]))
      tp_stop = bad()
      tp = (rng.choice([10., 1.5, 0.5]) if not bad() else rng.choice([0.05, -1.]), tp_stop)
      radar = rng.choice([None, 15.1, 40.]) if not bad() else rng.choice([8., 14.9])
      model_lead = rng.choice([None, 20.]) if not bad() else 8.
      brake, gas, enabled = bad(), bad(), not bad()
      cause = rng.choice(['turnRelease', 'unprotectedLeft', 'stopSign']) if not bad() else 'none'
      go_id += bad()
      sm = lift_sm(go_plan(cause, go_id, t=i * DT_MDL, should_stop=tp_stop), model(**HOLDING, lead_x=model_lead), brake, gas, enabled, radar)
      out = gl.update(sm, v, mpc, cruise, tp, e2e, DT_MDL)
      assert e2e[0] <= out[0] <= max(e2e[0], GO_ACCEL)
      if out != e2e:
        assert gl.lifting and out == (max(e2e[0], GO_ACCEL), False) and e2e[0] >= GO_MODEL_BRAKE
        assert v < GO_END_SPEED and cause != 'none' and not brake and not gas and enabled
        assert (radar is None or radar >= GO_CLEAR_M) and (model_lead is None or model_lead >= GO_CLEAR_M)
        assert not held(v, mpc) and not held(v, cruise) and not held(v, tp)
        assert gl.elapsed < GO_WINDOW_S and gl.key == (cause, go_id)
        lifts[gl.key] = lifts.get(gl.key, 0) + 1
      elif gl.lifting:
        assert out[0] >= GO_ACCEL and not out[1]  # lifted to its own accel: the model already asked for more
    assert all(n <= round(GO_WINDOW_S / DT_MDL) for n in lifts.values())
    lifted_frames += sum(lifts.values())
  assert lifted_frames > 3000  # the property was exercised, not vacuous


# --- turnpilotd's side: the certified go ---

def test_certifier_one_go_at_a_time_each_its_own_id_and_window():
  gc = GoCertifier()
  assert not gc.active and gc.cause == 'none'
  assert gc.start('turnRelease')
  first = gc.id
  assert not gc.start('stopSign') and gc.cause == 'turnRelease' and gc.id == first  # only after the previous one ended
  n = round(GO_WINDOW_S / DT_MDL)
  for _ in range(n - 1):
    gc.update(model(**HOLDING), 0., DT_MDL)
  assert gc.active and abs(gc.t - (n - 1) * DT_MDL) < 1e-6
  gc.update(model(**HOLDING), 0., DT_MDL)
  assert not gc.active and gc.end == 'window' and not gc.taken_over
  assert gc.start('stopSign') and gc.id == first + 1 and gc.t == 0.


def test_certifier_ends_on_the_takeover_or_the_speed_and_withdraws():
  gc = GoCertifier()
  gc.start('unprotectedLeft')
  for _ in range(round(TAKEOVER_S / DT_MDL)):
    gc.update(model(**GOING), 0.5, DT_MDL)
  assert not gc.active and gc.end == 'takeover' and gc.taken_over
  gc.start('turnRelease')
  gc.update(model(**HOLDING), GO_END_SPEED, DT_MDL)
  assert not gc.active and gc.end == 'speed' and gc.taken_over
  gc.start('turnRelease')
  gc.withdraw('inPath')
  assert not gc.active and gc.end == 'inPath' and not gc.taken_over
  gc.withdraw('again')  # nothing in hand: nothing changes
  assert gc.end == 'inPath'


def test_certifier_fills_the_plan_only_while_its_go_is_out():
  gc = GoCertifier()
  msg = messaging.new_message('turnpilotPlan')
  gc.fill(msg.turnpilotPlan)
  assert msg.turnpilotPlan.goCertified.cause == 'none'
  gc.start('turnRelease')
  gc.update(model(**HOLDING), 0., DT_MDL)  # the go's own frame isn't counted: started after this frame's update
  gc.start('turnRelease')  # refused
  msg = messaging.new_message('turnpilotPlan')
  gc.fill(msg.turnpilotPlan)
  g = msg.as_reader().turnpilotPlan.goCertified
  assert g.cause == 'turnRelease' and g.id == gc.id and abs(g.t - DT_MDL) < 1e-6
