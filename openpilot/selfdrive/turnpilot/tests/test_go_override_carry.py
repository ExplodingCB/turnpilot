"""The left's carry (go_override.py: GoLift's carry, GoCertifier's carry=True; review 2026-11-08, round 2's trade 'G5's
coast'): a second go of the unprotected left's own, certified with the left committed, that plannerd lifts to the 'left'
post-go profile's pace (CARRY_ACCEL, CARRY_SPEED) and no further, with no window, speed or takeover end and every other
end of the lift's kept."""
import random

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.go_override import (CARRY_ACCEL, CARRY_MODEL_SLOW, CARRY_SPEED, GO_ACCEL, GO_CLEAR_M, GO_END_SPEED,
                                                       GO_MODEL_BRAKE, GO_WINDOW_S, TAKEOVER_S, GoCertifier, GoLift, carry_accel)
from openpilot.selfdrive.turnpilot.hooks import SPEED_CAP_TAU, plan_accel
from openpilot.selfdrive.turnpilot.tests.test_go_override import CRUISE, FREE, held, lift_sm, model
from openpilot.selfdrive.turnpilot.turn_speed import LEFT_GO_PROFILE, POST_GO_PROFILES
from openpilot.selfdrive.turnpilot.unprotected_left import A_GO, V_GO

State = custom.TurnpilotPlan.State
COASTING = {'should_stop': False, 'plan': lambda t: 0.78}  # a model keeping the lift's speed: its plan is the takeover


def left_plan(left='committed', cause='unprotectedLeft', go_id=1, t=0., accel_cap=1.5, speed_cap=4.3, should_stop=False):
  """An active turnpilotPlan with the left in state left and its go out (cause none: no go), under the left's post-go caps."""
  msg = messaging.new_message('turnpilotPlan')
  p = msg.turnpilotPlan
  p.state, p.accelCap, p.speedCap, p.shouldStop = State.turning, accel_cap, speed_cap, should_stop
  p.unprotectedLeft.state = left
  if cause != 'none':
    p.goCertified.cause, p.goCertified.id, p.goCertified.t = cause, go_id, float(t)
  return msg.as_reader().turnpilotPlan


def tp_of(v, accel_cap=1.5, speed_cap=4.3, should_stop=False):
  return plan_accel(accel_cap, speed_cap, v), should_stop


def carry(gl, v=0.78, plan=None, e2e=(0., False), mpc=FREE, cruise=CRUISE, tp=None, **sm_kw):
  plan = plan if plan is not None else left_plan()
  tp = tp if tp is not None else tp_of(v, plan.accelCap, plan.speedCap, plan.shouldStop)
  sm_kw.setdefault('model_v2', model(**COASTING))
  return gl.update(lift_sm(plan, **sm_kw), v, mpc, cruise, tp, e2e, DT_MDL)


def test_the_carry_is_the_left_profiles_pace_never_more():
  """The 'left' post-go profile (turn_speed.py) is the left's own launch (unprotected_left: A_GO to V_GO, its clear time's):
  the carry never lifts past its accel cap nor toward more than its speed, whatever the plan's caps say."""
  assert (CARRY_ACCEL, CARRY_SPEED) == (POST_GO_PROFILES[LEFT_GO_PROFILE].accel_cap, POST_GO_PROFILES[LEFT_GO_PROFILE].v_end)
  assert (CARRY_ACCEL, CARRY_SPEED) == (A_GO, V_GO) == (1.5, 4.3)
  for v in (0.0, 0.78, 2., 3.5, 4.0, 4.3, 5.):
    for caps in ((1.5, 4.3), (10., 100.), (10., 4.65), (1.0, 3.0)):
      a = carry_accel(v, tp_of(v, *caps))
      assert a <= min(CARRY_ACCEL, (CARRY_SPEED - v) / SPEED_CAP_TAU, plan_accel(*caps, v)) + 1e-9, (v, caps)
  assert carry_accel(0.78, tp_of(0.78)) == CARRY_ACCEL
  assert abs(carry_accel(4.0, tp_of(4.0, 10., 100.)) - 0.3) < 1e-9  # past its speed cap's end: its own 4.3 m/s still


def test_a_coasting_models_carry_is_lifted_to_the_profile_with_no_window_speed_or_takeover_end():
  gl = GoLift()
  assert carry(gl) == (CARRY_ACCEL, False) and gl.lifting and gl.carry
  # the coasting model's plan is the takeover, GO_WINDOW_S goes by and the car passes GO_END_SPEED: still carried
  frames = round((GO_WINDOW_S + 2.) / DT_MDL)
  outs = [carry(gl, v=0.78 + 0.05 * i, plan=left_plan(t=i * DT_MDL)) for i in range(frames)]
  assert gl.active and gl.end == ''
  assert all(o[0] > 0. for o in outs[:20]) and any(0.78 + 0.05 * i >= GO_END_SPEED and o[0] > 0. for i, o in enumerate(outs))
  # at its speed nothing is lifted; the model faster than the profile keeps its own
  assert carry(gl, v=CARRY_SPEED) == (0., False) and not gl.lifting
  assert carry(gl, e2e=(2.0, False)) == (2.0, False) and not gl.lifting
  assert gl.active


def test_the_go_from_the_line_is_never_a_carry():
  """The same cause certified with the left in its go (the go from its line, which the lift's nudge alone never commits:
  LEFT_LIFT_ROOM_M) is the bounded lift as before: GO_ACCEL, ended by the takeover, GO_END_SPEED and its window."""
  gl = GoLift()
  out = carry(gl, v=0., plan=left_plan('go'), e2e=(0., True), model_v2=model(should_stop=True, plan=lambda t: 0.))
  assert out == (GO_ACCEL, False) and not gl.carry
  # committed later in that same go: still the bounded lift, never carried on
  assert carry(gl, v=1.0, plan=left_plan('committed'), e2e=(0., False), model_v2=model(should_stop=True, plan=lambda t: 0.)) \
    == (GO_ACCEL, False) and not gl.carry
  assert carry(gl, v=GO_END_SPEED, plan=left_plan('committed')) == (0., False) and gl.end == 'speed'
  gl = GoLift()
  n = round(TAKEOVER_S / DT_MDL)
  for i in range(n + 2):
    carry(gl, v=0.78, plan=left_plan('go', t=i * DT_MDL))
  assert gl.end == 'takeover' and not gl.carry
  # another cause, committed: never a carry
  gl = GoLift()
  assert carry(gl, v=0.5, plan=left_plan(cause='turnRelease')) == (GO_ACCEL, False) and not gl.carry


def test_every_rail_of_the_lift_ends_the_carry_for_good():
  """A pedal, a disengage, a lead within GO_CLEAR_M, a holder (TurnPilot's shouldStop: a person in the path), the model
  braking harder than GO_MODEL_BRAKE, planning a stop or slowing at all (its accel under CARRY_MODEL_SLOW: review
  2026-11-08, a -0.8 m/s^2 model was lifted to +1.5 for 1.8 s, its shouldStop not out in motion), the left no longer
  committed, the go withdrawn: the carry ends that frame, the model's own candidate back untouched, and never comes back
  for that go."""
  cases = [
    ('driver', {'brake': True}, {}), ('driver', {'gas': True}, {}), ('driver', {'enabled': False}, {}),
    ('lead', {'radar_lead': GO_CLEAR_M - 1.}, {}), ('lead', {'model_v2': model(**COASTING, lead_x=GO_CLEAR_M - 1.)}, {}),
    ('holder', {'plan': left_plan(should_stop=True)}, {}), ('holder', {}, {'mpc': (0.5, True)}),
    ('holder', {}, {'cruise': (-0.5, True)}),
    ('modelBrake', {}, {'e2e': (GO_MODEL_BRAKE - 0.1, False)}), ('modelStop', {}, {'e2e': (-0.2, True)}),
    ('modelSlow', {}, {'e2e': (CARRY_MODEL_SLOW - 0.01, False)}), ('modelSlow', {}, {'e2e': (-0.8, False)}),
    ('left', {'plan': left_plan('off')}, {}), ('left', {'plan': left_plan('waitGap')}, {}),
    ('withdrawn', {'plan': left_plan(cause='none')}, {}),
  ]
  for reason, sm_kw, kw in cases:
    gl = GoLift()
    assert carry(gl) == (CARRY_ACCEL, False)
    sm_kw = dict(sm_kw)
    plan = sm_kw.pop('plan', left_plan())
    e2e = kw.pop('e2e', (0., False))
    out = carry(gl, plan=plan, e2e=e2e, **kw, **sm_kw)
    assert out == e2e and not gl.lifting and gl.end == reason, (reason, gl.end)
    assert carry(gl) == (0., False) and not gl.lifting, reason  # the same go again: over


def test_only_a_model_coasting_or_speeding_up_is_carried():
  """The carry raises a model that keeps its speed or speeds up (CARRY_MODEL_SLOW and over: the coast, the real model's own
  launches, never under +0.28 m/s^2 on 11-06); one slowing is never lifted over, at any speed under CARRY_SPEED."""
  assert GO_MODEL_BRAKE < CARRY_MODEL_SLOW == -0.3
  for a in (CARRY_MODEL_SLOW, -0.2, 0., 0.5):
    gl = GoLift()
    assert carry(gl, e2e=(a, False)) == (CARRY_ACCEL, False) and gl.lifting, a
  for v in (0.6, 1.5, 3.0, 4.0):
    for a in (CARRY_MODEL_SLOW - 0.01, -0.5, -0.9):
      gl = GoLift()
      assert carry(gl, v=v) != (0., False)  # carried while it coasts...
      assert carry(gl, v=v, e2e=(a, False)) == (a, False) and not gl.lifting and gl.end == 'modelSlow', (v, a)
      assert carry(gl, v=v) == (0., False) and not gl.lifting  # ...and never again once it slowed


def test_the_certifiers_carry_is_out_until_withdrawn():
  gc = GoCertifier()
  assert gc.start('unprotectedLeft', carry=True) and gc.carry
  coast = model(**COASTING)
  for i in range(round((GO_WINDOW_S + 3.) / DT_MDL)):
    gc.update(coast, 0.78 + 0.05 * i)
  assert gc.active and gc.t > GO_WINDOW_S and not gc.taken_over
  gc.withdraw('carry across')
  assert not gc.active and gc.end == 'carry across' and not gc.taken_over
  # the next go is the bounded one again
  assert gc.start('unprotectedLeft') and not gc.carry
  for _ in range(round(TAKEOVER_S / DT_MDL) + 1):
    gc.update(coast, 0.78)
  assert not gc.active and gc.end == 'takeover'


def test_carry_property_random():
  """Random frames of a carry: the e2e candidate is only ever raised, to carry_accel at most (never past CARRY_ACCEL nor
  toward more than CARRY_SPEED, nor past the TurnPilot candidate's own accel), never with a rail failing, and never again
  once the model slowed (CARRY_MODEL_SLOW)."""
  rng = random.Random(11)
  lifted = 0
  for _ in range(200):
    gl = GoLift()
    slowed = False
    for i in range(rng.randint(1, 80)):
      v = rng.uniform(0., 6.)
      caps = rng.choice([(1.5, 4.3), (10., 4.65), (10., 100.), (1.0, 3.0), (-1.5, 0.)])
      stop = rng.random() < 0.03
      left = 'committed' if i == 0 else rng.choice(['committed'] * 40 + ['go', 'off'])  # a carry: committed at its first frame
      plan = left_plan(left, t=i * DT_MDL, accel_cap=caps[0], speed_cap=caps[1], should_stop=stop)
      tp = tp_of(v, *caps, stop)
      e2e = (rng.uniform(-1.5, 2.) if rng.random() < 0.03 else rng.uniform(CARRY_MODEL_SLOW, 2.), rng.random() < 0.05)
      mpc = rng.choice([FREE] * 20 + [(0.5, True)])
      lead = rng.choice([None] * 20 + [GO_CLEAR_M - 2.])
      brake = rng.random() < 0.02
      out = gl.update(lift_sm(plan, model(**COASTING), brake=brake, radar_lead=lead), v, mpc, CRUISE, tp, e2e, DT_MDL)
      assert out[0] >= e2e[0]
      if out != e2e:
        assert gl.lifting and gl.carry and left == 'committed' and not brake and lead is None and mpc == FREE
        assert not held(v, tp) and not held(v, e2e) and e2e[0] >= CARRY_MODEL_SLOW and not slowed
        assert out == (carry_accel(v, tp), False) and out[0] <= min(CARRY_ACCEL, (CARRY_SPEED - v) / SPEED_CAP_TAU, tp[0]) + 1e-9
        lifted += 1
      slowed = slowed or e2e[0] < CARRY_MODEL_SLOW
  assert lifted > 150  # the property was exercised, not vacuous
