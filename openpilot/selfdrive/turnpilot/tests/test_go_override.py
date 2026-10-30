"""TurnPilot's bounded go (go_override.py; DESIGN phase 4 decision 1 and its stop-sign amendment, G5): plannerd lifts the
e2e model's hold on a standing car only for a go turnpilotd certifies, at most +0.5 m/s^2, for at most 3 s, under 3 m/s,
and only while nothing else holds the car."""
import random
from collections import deque
import time

import numpy as np

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


# --- the planner: longitudinal_planner.py with the lift wired in ---

def tesla_cp():
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.tesla.values import CAR
  cp = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
  cp.openpilotLongitudinalControl = True  # the demo car: alpha longitudinal on (DESIGN decision 8)
  return cp


def planner_sm(v=0., plan=None, model_v2=None, brake=False, gas=False, enabled=True, radar_lead=None, age=0., cruise_kph=40.):
  from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
  msgs = {s: messaging.new_message(s) for s in ('carState', 'selfdriveState', 'radarState', 'controlsState')}
  cs = msgs['carState'].carState
  cs.vEgo, cs.standstill, cs.vCruise, cs.brakePressed, cs.gasPressed = float(v), v < 0.01, cruise_kph, brake, gas
  msgs['selfdriveState'].selfdriveState.enabled = enabled
  msgs['controlsState'].controlsState.longControlState = LongCtrlState.pid if enabled else LongCtrlState.off
  if radar_lead is not None:
    lead = msgs['radarState'].radarState.leadOne
    lead.present, lead.dRel, lead.modelProb, lead.aLeadTau = True, float(radar_lead), 0.9, 1.5
  sm = {s: getattr(m.as_reader(), s) for s, m in msgs.items()}
  sm['modelV2'] = model_v2 if model_v2 is not None else model(**HOLDING)
  if plan is not None:
    sm['turnpilotPlan'] = plan
  return FakeSM(sm, age=age)


WARM_S = 2.  # s standing held before a go: the cruise candidate ramps up from the planner's start (it is 2.0 after a stand)


def new_planner(warm=True):
  """The planner, standing WARM_S held by the model (no go) first, as a car at its line is."""
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  lp = LongitudinalPlanner(tesla_cp(), init_v=0.)
  for _ in range(round(WARM_S / DT_MDL) if warm else 0):
    lp.update(planner_sm(0., go_plan('none')))
  return lp


def published(lp, sm):
  pm = mock_pm()
  sm.logMonoTime = {'modelV2': 0}
  sm.all_checks = lambda: True
  lp.publish(sm, pm)
  return pm.sent['longitudinalPlan'].longitudinalPlan


def mock_pm():
  class PM:
    def __init__(self):
      self.sent = {}

    def send(self, s, msg):
      self.sent[s] = msg.as_reader()
  return PM()


def test_planner_lifts_the_models_standstill_hold_only_for_a_certified_go():
  lp = new_planner()  # standing, the model holding: TurnPilot's hold gone, no go
  assert lp.output_a_target == 0. and lp.output_should_stop
  lp.update(planner_sm(0., go_plan()))
  assert lp.output_a_target == GO_ACCEL and not lp.output_should_stop and lp.mpc.source == log.LongitudinalPlan.LongitudinalPlanSource.e2e
  lp_msg = published(lp, planner_sm(0., go_plan()))
  assert lp_msg.turnpilotGoLift and lp_msg.turnpilotGoEnd == '' and lp_msg.aTarget == GO_ACCEL and not lp_msg.shouldStop
  # withdrawn: that same frame the model's hold is back
  lp.update(planner_sm(0., go_plan('none')))
  assert lp.output_a_target == 0. and lp.output_should_stop
  lp_msg = published(lp, planner_sm(0., go_plan('none')))
  assert not lp_msg.turnpilotGoLift and lp_msg.turnpilotGoEnd == 'withdrawn'


def run(lp, frames):
  """frames: lists of planner_sm keywords, one a frame. (aTarget, shouldStop) of each."""
  out = []
  for kw in frames:
    lp.update(planner_sm(**kw))
    out.append((float(lp.output_a_target), bool(lp.output_should_stop)))
  return out


def test_planner_output_is_untouched_whenever_the_lift_may_not_act():
  """Twin planners, the same frames, one with a certified go: identical outputs at or above GO_END_SPEED, with a lead inside
  GO_CLEAR_M, under TurnPilot's hold (a walker's), braking, a shadow, faulted (disabled) or stale plan, and after its window."""
  base = {'plan': go_plan('none')}
  cases = {
    'speed': [{'v': GO_END_SPEED}] * 30, 'lead': [{'radar_lead': 8.}] * 30, 'leadAt14': [{'radar_lead': GO_CLEAR_M - 1.}] * 30,
    'modelLead': [{'model_v2': model(**HOLDING, lead_x=10.)}] * 30, 'brake': [{'brake': True}] * 30,
  }
  for name, frames in cases.items():
    plain = run(new_planner(), [{**base, **f} for f in frames])
    gone = run(new_planner(), [{**f, 'plan': go_plan(t=i * DT_MDL)} for i, f in enumerate(frames)])
    assert plain == gone, name
  held = go_plan(should_stop=True, speed_cap=0.)  # TurnPilot holds (a walker in front): the go means nothing
  assert run(new_planner(), [{'plan': go_plan('none', should_stop=True, speed_cap=0.)}] * 30) == run(new_planner(), [{'plan': held}] * 30)
  for plan in (go_plan(shadow=True), go_plan(state=State.disabled, should_stop=True)):
    assert run(new_planner(), [{'plan': go_plan('none', shadow=plan.shadow, state=plan.state, should_stop=plan.shouldStop)}] * 30) == \
      run(new_planner(), [{'plan': plan}] * 30)
  assert run(new_planner(), [{'plan': go_plan(), 'age': 1.}] * 30) == run(new_planner(), [{'plan': go_plan('none'), 'age': 1.}] * 30)
  # past the window: the same go asserted on lifts no more
  n = round(GO_WINDOW_S / DT_MDL)
  out = run(new_planner(), [{'plan': go_plan(t=i * DT_MDL)} for i in range(n + 20)])
  assert all(o == (GO_ACCEL, False) for o in out[:n]) and all(o == (0., True) for o in out[n:])


class Recorder(GoLift):
  """GoLift that keeps each frame's inputs and output, so the planner's own candidates are known to the test."""
  def __init__(self):
    super().__init__()
    self.log = []

  def update(self, sm, v_ego, mpc, cruise, tp, e2e, dt=DT_MDL, now=None):
    out = super().update(sm, v_ego, mpc, cruise, tp, e2e, dt, now)
    self.log.append((v_ego, mpc, cruise, tp, e2e, out, self.lifting))
    return out


def test_planner_property_random():
  """Random drives through the real planner: its output with the lift is the min() of the same candidates with only the e2e
  one lifted, so never above max(its output without the lift, GO_ACCEL), and the any() of their stops less only the
  model's; a frame not lifted is the stock blend exactly."""
  from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
  rng = random.Random(7)
  lifted = 0
  for _ in range(40):
    lp = new_planner()
    lp.go_lift = rec = Recorder()
    rec.log = []
    go_id, v, t = 1, 0., 0.
    for _ in range(rng.randint(20, 90)):
      kw = {'v': v, 'plan': go_plan('turnRelease', go_id, t=t, should_stop=rng.random() < 0.03,
                                    speed_cap=rng.choice([100., 100., 4.3, 0.5]), accel_cap=rng.choice([10., 10., 1.0, 0.3]))}
      if rng.random() < 0.05:
        kw['radar_lead'] = rng.choice([6., 14., 30.])
      if rng.random() < 0.03:
        kw['brake'] = True
      kw['model_v2'] = model(should_stop=v < 0.3, plan=lambda t_: 0., accel=rng.uniform(-0.5, 0.3)) if rng.random() < 0.8 else \
        model(**GOING, accel=0.6)
      if rng.random() < 0.03:
        kw['plan'] = go_plan('none')
      lp.update(planner_sm(**kw))
      v_ego, mpc, cruise, tp, e2e, out, lifting = rec.log[-1]
      cands = [mpc, cruise] + ([tp] if tp is not None else [])
      plain = float(np.clip(min([c[0] for c in cands] + [e2e[0]]), ACCEL_MIN, ACCEL_MAX))
      a = float(lp.output_a_target)
      assert a <= max(plain, GO_ACCEL) + 1e-6
      if not lifting:
        assert out == e2e and a == plain and lp.output_should_stop == any(c[1] for c in cands + [e2e])
      else:
        lifted += 1
        assert v_ego < GO_END_SPEED and a == float(np.clip(min([c[0] for c in cands] + [max(e2e[0], GO_ACCEL)]), ACCEL_MIN, ACCEL_MAX))
        assert lp.output_should_stop == any(c[1] for c in cands)
      v = max(v + a * DT_MDL, 0.)  # the car follows its plan
      t += DT_MDL
      if rng.random() < 0.02:
        go_id, t = go_id + 1, 0.
  assert lifted > 200


# --- closed loop: the planner driving a point mass, a model that holds a standstill ---

def holding_model(v):
  """G0's stand: the model holds the car (shouldStop, its plan to a stop), braking gently when it rolls."""
  a = -0.4 if v > 0.05 else 0.
  return model(should_stop=v < 0.3 and a < 0.1, plan=lambda t: max(v - 0.6 * t, 0.), accel=a)


def model_that_takes_over(v):
  """Holds at a standstill, then, rolling, means to go: shouldStop false and a plan 0.8 m/s^2 on."""
  if v < 0.3:
    return holding_model(v)
  return model(plan=lambda t: v + 0.8 * t, accel=0.8)


def closed_loop(model_fn, seconds=9., lag_s=0.3, go_at=WARM_S, withdraw_at=None, brake_at=None, lead_at=None):
  """The real planner closing the loop on a point mass whose accel follows aTarget lag_s late (a pure delay; -1.0 m/s^2 on a
  shouldStop, longcontrol's stopping), standing until go_at, then TurnPilot's go asserted as turnpilotd asserts it
  (GoCertifier: until its window, the model's takeover or 3 m/s; withdrawn at withdraw_at). The driver brakes from brake_at,
  a lead stands 8 m ahead from lead_at. Rows (t, v, s, aTarget, lifting, certified)."""
  lp = new_planner(warm=False)
  gc = GoCertifier()
  queue = deque([0.] * max(round(lag_s / DT_MDL), 1))
  v = s = 0.
  rows = []
  for i in range(round(seconds / DT_MDL)):
    t = i * DT_MDL
    m = model_fn(v)
    gc.update(m, v, DT_MDL)
    if abs(t - go_at) < DT_MDL / 2:
      assert gc.start('turnRelease')
    if withdraw_at is not None and abs(t - withdraw_at) < DT_MDL / 2:
      gc.withdraw('test')
    msg = messaging.new_message('turnpilotPlan')
    p = msg.turnpilotPlan
    p.state, p.accelCap, p.speedCap = State.turning, 10., 100.
    gc.fill(p)
    brake = brake_at is not None and t >= brake_at
    lead = 8. if lead_at is not None and t >= lead_at else None
    lp.update(planner_sm(v, msg.as_reader().turnpilotPlan, m, brake=brake, radar_lead=lead))
    a_cmd = -2. if brake else -1. if lp.output_should_stop else float(lp.output_a_target)
    queue.append(a_cmd)
    v = max(v + queue.popleft() * DT_MDL, 0.)
    s += v * DT_MDL
    rows.append((t, v, s, float(lp.output_a_target), lp.go_lift.lifting, gc.active))
  lp.certifier = gc
  return rows, lp


def test_closed_loop_a_model_that_never_takes_over_is_nudged_2_25_m_at_most():
  """The model holds throughout: the go lifts the car at +0.5 m/s^2 for its 3 s, no more: 1.5 m/s and 2.25 m at most (a lag
  only shortens it), then the model's hold is back and the car stops again on the model's own braking (here a gentle 0.4
  m/s^2: turnpilotd's withdrawal of a go not taken over stops it sooner, test_turnpilotd_go.py)."""
  for lag in (0.15, 0.3, 0.5):
    rows, lp = closed_loop(holding_model, seconds=11., lag_s=lag)
    lifted = [r for r in rows if r[4]]
    assert len(lifted) == round(GO_WINDOW_S / DT_MDL) and all(r[3] == GO_ACCEL for r in lifted)
    # turnpilotd's window and plannerd's agree: its end clears the go the frame plannerd's would have ended it
    assert lp.certifier.end == 'window' and not lp.certifier.taken_over and lp.go_lift.end == 'withdrawn'
    end_lift = lifted[-1]
    assert max(r[1] for r in rows) <= GO_ACCEL * GO_WINDOW_S + 1e-6
    assert end_lift[2] <= 0.5 * GO_ACCEL * GO_WINDOW_S ** 2 + 1e-6
    assert rows[-1][1] < 0.05
    assert all(r[3] >= -2. for r in rows)


def test_closed_loop_the_lift_ends_on_the_models_takeover():
  """A model that goes once the car rolls: the lift ends TAKEOVER_S after its plan says go, well inside the window, and the
  model drives on past 3 m/s by itself. With an actuator lag of up to 0.3 s the car is past 0.3 m/s inside the unprotected
  left's LEFT_NO_MOVE_S (1.0 s, CRITIQUE REQUIRED 4)."""
  from openpilot.selfdrive.turnpilot.turnpilotd import LEFT_NO_MOVE_S, STANDSTILL_SPEED
  for lag in (0.15, 0.3):
    rows, lp = closed_loop(model_that_takes_over, lag_s=lag)
    lifted = [r for r in rows if r[4]]
    assert 0 < len(lifted) < round(GO_WINDOW_S / DT_MDL) and lp.certifier.end == 'takeover' and lp.certifier.taken_over
    moving = next(r[0] for r in rows if r[1] >= STANDSTILL_SPEED)
    assert moving - WARM_S <= LEFT_NO_MOVE_S, (lag, moving - WARM_S)
    assert rows[-1][1] > GO_END_SPEED and not rows[-1][4]


def test_closed_loop_withdrawn_mid_lift_or_braked_ends_it_that_frame():
  rows, lp = closed_loop(holding_model, withdraw_at=WARM_S + 1.)
  at = next(i for i, r in enumerate(rows) if r[0] >= WARM_S + 1. - 1e-6)
  assert rows[at - 1][4] and not any(r[4] for r in rows[at:]) and lp.go_lift.end == 'withdrawn'
  assert rows[-1][1] < 0.05 and rows[-1][2] <= 0.5 * GO_ACCEL * 1. ** 2 + 0.5
  rows, lp = closed_loop(holding_model, brake_at=WARM_S + 1.)
  at = next(i for i, r in enumerate(rows) if r[0] >= WARM_S + 1. - 1e-6)
  assert rows[at - 1][4] and not any(r[4] for r in rows[at:]) and lp.go_lift.end == 'driver'
  # a lead stopping 8 m ahead mid-lift: over at once, the mpc holds the car behind it
  rows, lp = closed_loop(holding_model, lead_at=WARM_S + 1.)
  at = next(i for i, r in enumerate(rows) if r[0] >= WARM_S + 1. - 1e-6)
  assert not any(r[4] for r in rows[at:]) and lp.go_lift.end == 'lead'


def test_certifier_refuses_a_cause_the_schema_doesnt_have():
  import pytest
  gc = GoCertifier()
  with pytest.raises(ValueError):
    gc.start('gasTap')
  assert not gc.active and gc.id == 0
