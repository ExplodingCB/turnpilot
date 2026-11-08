"""The unprotected left's carry across the oncoming lanes (turnpilotd.left_carry, go_override's carry, unprotected_left's
LeftDecision.carry) in closed loop: the Sim's car, turnpilotd and the real longitudinal planner (plannerd's lift).

Review 2026-11-08, round 2's trade ('G5's coast'): a slow launch the left judged on keeps driving unless a car is seen in the
band its launch needs, so a model that only keeps the speed plannerd's lift gave it (0.78 m/s), nothing in the camera's
band, crossed the oncoming lanes at that speed for about 15 s, where the gap rule's R_req assumed the left's own launch
(A_GO to V_GO: under 3 s in them). A car outside the camera's band at the go could arrive while it was still in them. Now a
left committed (it can no longer stop short of them) short of a launch after such a judgement is carried across at the
left's post-go profile: plannerd lifts the model's candidate to the profile's accel cap and speed, never more, never
steering, every rail of the lift's kept."""
from unittest import mock

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot import go_override as go_mod, turnpilotd as tpd, unprotected_left as ul
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.turnpilot.go_override import CARRY_ACCEL, CARRY_MODEL_SLOW, CARRY_SPEED, GO_MODEL_BRAKE
from openpilot.selfdrive.turnpilot.hooks import SPEED_CAP_TAU, TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.scene_policy import ONCOMING_ENTRY_M
from openpilot.selfdrive.turnpilot.tests.synthetic import model_v2
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_go import attach_planner, coasting, holding, launching
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import JUNCTION, V_25, drive, left_sim, road_runs, signal_sim
from openpilot.selfdrive.turnpilot.turnpilotd import TurnPilot
from openpilot.selfdrive.turnpilot.unprotected_left import (A_GO, CLEAR_PATH_PAST_M, CONFLICT_PAST_M, FULL_PASS_RANGE_M, LEFT_LAUNCH_SPEED,
                                                           PET_S, SPEED_MARGIN, V_GO, WITHDRAWN_REASON)

LANES_M = ONCOMING_ENTRY_M + CLEAR_PATH_PAST_M  # the oncoming lanes on R_req's path: from their entry to its end (t_clear)
CRAWL_V = 0.78  # m/s: what the coasting model keeps (the lift's speed when its takeover predicate holds)


def profile_s(v0: float, d: float) -> float:
  """s to cover d m from v0 at the left's own launch (A_GO up to V_GO): what t_clear assumes of the car past v0."""
  if v0 >= V_GO:
    return d / V_GO
  d_ramp = (V_GO ** 2 - v0 ** 2) / (2. * A_GO)
  if d <= d_ramp:
    return ((v0 ** 2 + 2. * A_GO * d) ** 0.5 - v0) / A_GO
  return (V_GO - v0) / A_GO + (d - d_ramp) / V_GO


def run_left(model_fn, cars=lambda sim: (), seconds=25., lag_s=0.3, range_m=None, sim_fn=None, carry=True):
  """A left held at its line, plannerd attached with model_fn at lag_s, driven seconds with the Sim's yaw: (sim, the plans,
  the index in sim.trace of the first planned frame, the events of turnpilotd, go_override and the left). range_m: the full
  pass's (FULL_PASS_RANGE_M on the car); carry False: turnpilotd's left_carry a no-op (HEAD's crossing)."""
  events = []

  def said(name, *a, **kw):
    events.append((name, kw))
  with mock.patch.object(tpd.cloudlog, 'event', side_effect=said), mock.patch.object(go_mod.cloudlog, 'event', side_effect=said), \
       mock.patch.object(ul.cloudlog, 'event', side_effect=said), \
       mock.patch.object(TurnPilot, 'left_carry', TurnPilot.left_carry if carry else lambda self, *a, **kw: None):
    sim = left_sim(cars) if sim_fn is None else sim_fn()
    if range_m is not None:
      sim.tp.full_pass_range_m = range_m
    while not (sim.trace and sim.trace[-1][2].shouldStop and sim.v < 0.05 and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap'):
      drive(sim, DT_MDL)
      assert sim.t < 60.
    n0 = len(sim.trace)
    attach_planner(sim, model_fn, lag_s)
    plans = drive(sim, seconds)
  return sim, plans, n0, events


def named(events, name):
  return [kw for n, kw in events if n == name]


def in_lanes_s(sim, n0):
  return sum(JUNCTION - ONCOMING_ENTRY_M <= s <= JUNCTION + CLEAR_PATH_PAST_M for s, _, _ in sim.trace[n0:]) * DT_MDL


def lanes_v_max(sim, n0):
  return max(v for s, v, _ in sim.trace[n0:] if JUNCTION - ONCOMING_ENTRY_M <= s <= JUNCTION + CLEAR_PATH_PAST_M)


def entry_v(sim, n0):
  return next(v for s, v, _ in sim.trace[n0:] if s >= JUNCTION - ONCOMING_ENTRY_M)


@pytest.mark.parametrize('lag_s', [0.3, 1.0])
def test_a_coast_on_the_lifts_speed_is_carried_across_the_oncoming_lanes_in_about_the_time_r_req_assumed(lag_s):
  """The coasting model on the car's own full pass (98 m, the far crop off as on 11-06): judged on at the slow launch (the
  band it needs, past the camera's reach, judged on what the camera covers), committed at the oncoming lanes' entry at 0.78
  m/s. HEAD crossed them at that speed (15.5 s from their entry to R_req's path end). Now the left's carry lifts it at the
  profile's 1.5 m/s^2 to 4.3 m/s: across in about the profile's own time from the speed it entered at, so the rest of the
  crossing from the commit, with PET_S, fits inside the band the camera saw clear there. Never more accel than the profile,
  never past its speed, never a brake past -2.0, never a handback; at this car's own launch lag (11-06: 0.92-1.6 s from the
  plan's go to 0.3 m/s) as well."""
  sim, plans, n0, said = run_left(coasting, range_m=FULL_PASS_RANGE_M, lag_s=lag_s)
  slow = named(said, 'turnpilot.left_not_launched')
  assert len(slow) == 1 and slow[0]['on'] and slow[0]['r_req'] > slow[0]['range_m'] == FULL_PASS_RANGE_M, slow
  committed = named(said, 'turnpilot.left_committed')
  assert len(committed) == 1 and committed[0]['carry'] and committed[0]['v'] < LEFT_LAUNCH_SPEED, committed
  carries = named(said, 'turnpilot.left_carry')
  lifts = [kw for kw in named(said, 'turnpilot.go_lift') if kw['carry']]
  assert len(carries) == 1 and len(lifts) == 1 and lifts[0]['go_id'] == carries[0]['go_id'], (carries, lifts)
  ends = [kw for kw in named(said, 'turnpilot.go_end') if kw['go_id'] == carries[0]['go_id']]
  assert len(ends) == 1 and ends[0]['reason'] in ('carry across', 'carry turned'), ends

  v0, lanes = entry_v(sim, n0), in_lanes_s(sim, n0)
  crawl_v = committed[0]['v']  # what the coasting model keeps from the lift (0.78 m/s at 0.3 s of lag, more at 1 s)
  assert crawl_v < 1.2 and v0 < crawl_v + 0.1, (crawl_v, v0)
  assert lanes <= profile_s(v0, LANES_M) + 0.6 + lag_s, (lanes, profile_s(v0, LANES_M))  # the planner's 1 s cap and the lag
  assert lanes < 0.4 * LANES_M / crawl_v, lanes  # HEAD's crawl at it: 15.4 s at 0.78 m/s, 10.7 at 1.12
  # the rest of the crossing from the commit, as R_req has it (SPEED_MARGIN x the limit over it and PET_S, from the conflict
  # point), within the band the camera saw clear at the commit
  commit_i = next(i for i, (_, _, p) in enumerate(sim.trace[n0:]) if str(p.unprotectedLeft.state) == 'committed')
  end_i = next(i for i, (s, _, _) in enumerate(sim.trace[n0:]) if s > JUNCTION + CLEAR_PATH_PAST_M)
  rest_s = (end_i - commit_i) * DT_MDL
  if lag_s < 0.5:
    assert V_25 * SPEED_MARGIN * (rest_s + PET_S) + CONFLICT_PAST_M <= FULL_PASS_RANGE_M, rest_s

  # never more than the profile asks: every lifted frame's accel within its cap and its speed's (the frame's speed a frame on)
  lifted = [(a, sim.trace[n0 + i][1]) for i, (a, _, on) in enumerate(sim.planned) if on]  # the go's lift and the carry's
  assert any(v > 1.5 for _, v in lifted) and max(a for a, _ in lifted) <= CARRY_ACCEL + 1e-6
  assert all(a <= (CARRY_SPEED - v) / SPEED_CAP_TAU + 0.1 for a, v in lifted)
  # ...and never faster through the lanes than the left's own launch at the same lag (the Sim's pure delay carries an accel
  # on past the cap's end alike: 5.09 m/s at 1 s for both)
  own, _, o0, _ = run_left(launching, seconds=12., lag_s=lag_s)
  assert lanes_v_max(sim, n0) <= lanes_v_max(own, o0) + 0.05, (lanes_v_max(sim, n0), lanes_v_max(own, o0))
  assert min(a for a, _, _ in sim.planned) >= -2.
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert sim.s > JUNCTION + CLEAR_PATH_PAST_M


def test_at_head_the_coast_crawled_through_the_oncoming_lanes():
  """The same coast with turnpilotd's left_carry a no-op: HEAD's 15 s crawl, the case the carry exists for."""
  sim, plans, n0, said = run_left(coasting, range_m=FULL_PASS_RANGE_M, carry=False)
  assert not named(said, 'turnpilot.left_carry')
  assert in_lanes_s(sim, n0) > 0.9 * LANES_M / CRAWL_V


def test_a_car_seen_in_the_band_before_the_commit_still_withdraws_the_coast_and_it_waits_never_carried_nor_handed_back():
  """L3's rail kept: a car 250 m up as the coast pulls away (outside the A_GO launch's band, inside the coast's) withdraws
  the go to the left's own wait while the car can still stop short of the oncoming lanes; nothing is carried. The model never
  pulls away from a stand (no go over it from there, nearer the lanes than the lift's nudge), so the car waits short of the
  lanes saying so: a wait, never a handback."""
  st = {'t': None}

  def cars(sim):
    left = sim.tp.left_now
    if st['t'] is None and left is not None and left.state == 'go' and sim.v > 0.5:
      st['t'] = sim.t
    if st['t'] is None:
      return ()
    return ((sim.s + 250. - V_25 * (sim.t - st['t']), 3.6),) if sim.t - st['t'] < 6. else ()
  sim, plans, n0, said = run_left(coasting, cars, seconds=30.)
  slow = named(said, 'turnpilot.left_not_launched')
  assert len(slow) == 1 and not slow[0]['on'] and slow[0]['seen'], slow
  assert not named(said, 'turnpilot.left_carry') and not any(kw['carry'] for kw in named(said, 'turnpilot.go_lift'))
  assert not any(str(p.unprotectedLeft.state) == 'committed' for p in plans)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert sim.v < 0.05 and JUNCTION - sim.s > ONCOMING_ENTRY_M and plans[-1].shouldStop
  assert str(plans[-1].unprotectedLeft.state) == 'waitGap' and plans[-1].reason == WITHDRAWN_REASON['launch']


def ramp(a):
  """Holds at a standstill; rolling, it launches at a m/s^2."""
  return lambda sim: holding(sim) if sim.v < 0.3 else model_v2(*sim.model_lanes, accel=a, **sim.model_kw)


def from_rest(a):
  """Plans motion from rest at a m/s^2."""
  return lambda sim: model_v2(*sim.model_lanes, accel=a, **sim.model_kw)


@pytest.mark.parametrize('lag_s', [0.3, 1.0])
@pytest.mark.parametrize('model,judged', [('launching', False), ('ramp1.0', True), ('rest0.8', True), ('rest1.3', False)])
def test_a_normal_launch_is_unchanged_by_the_carry(model, judged, lag_s):
  """This car's own launches from a stand (11-06, 1d and 1e: four engaged, no gas; 0.36-0.55 s from first motion to 0.3 m/s,
  0.92-1.6 s from the plan's go) commit from LINE_D at 2.6-3.0 m/s and are never carried; one of them (1d, 1.1 m/s^2 to
  2.0 m/s in 1.85 m) is judged by L3 from LINE_D at 1.94 m/s. In the Sim, a launch at the left's own 1.5 m/s^2 is never
  judged; one at 0.8-1.0 m/s^2 is judged on and commits past LEFT_LAUNCH_SPEED. At the lag of this car's launches too, each
  drives exactly as with no carry at all, nothing certified, never a handback."""
  fn = {'launching': launching, 'ramp1.0': ramp(1.0), 'rest0.8': from_rest(0.8), 'rest1.3': from_rest(1.3)}[model]
  sim, plans, n0, said = run_left(fn, seconds=15., lag_s=lag_s)
  base, _, b0, _ = run_left(fn, seconds=15., lag_s=lag_s, carry=False)
  assert [(round(s, 6), round(v, 6)) for s, v, _ in sim.trace[n0:]] == [(round(s, 6), round(v, 6)) for s, v, _ in base.trace[b0:]]
  assert not named(said, 'turnpilot.left_carry') and not any(kw['carry'] for kw in named(said, 'turnpilot.go_lift'))
  slow = named(said, 'turnpilot.left_not_launched')
  assert bool(slow) == judged and all(kw['on'] for kw in slow), slow
  committed = named(said, 'turnpilot.left_committed')
  assert len(committed) == 1 and not committed[0]['carry'] and committed[0]['v'] >= LEFT_LAUNCH_SPEED, committed
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert sim.s > JUNCTION + CLEAR_PATH_PAST_M


def test_a_slow_launch_from_rest_is_carried_too_and_crosses_at_the_profiles_pace():
  """A model that pulls away by itself at 0.3 m/s^2 commits at about 1.6 m/s, short of a launch: carried from there."""
  sim, plans, n0, said = run_left(from_rest(0.3), seconds=20.)
  committed = named(said, 'turnpilot.left_committed')
  assert len(committed) == 1 and committed[0]['carry'], committed
  assert len(named(said, 'turnpilot.left_carry')) == 1
  v0 = entry_v(sim, n0)
  assert in_lanes_s(sim, n0) <= profile_s(v0, LANES_M) + 0.6
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def test_an_unlifted_lefts_slow_launch_is_never_carried():
  """A left plannerd wouldn't lift (a mapped signal with no accepted light source: every signal on the car) goes with the
  model, and so does its crossing: a slow launch there committing short of a launch is never carried (go_lifted), and that
  is said once (review 2026-11-08: a commit asking for a carry that never came said nothing)."""
  sim, plans, n0, said = run_left(from_rest(0.3), seconds=25., sim_fn=lambda: unlifted_sim())
  committed = named(said, 'turnpilot.left_committed')
  assert len(committed) == 1 and committed[0]['carry'], committed  # the left would have it carried...
  assert not named(said, 'turnpilot.left_carry') and not any(l for _, _, l in sim.planned)  # ...but nothing is lifted
  refused = named(said, 'turnpilot.left_carry_refused')
  assert [kw['why'] for kw in refused] == ['unlifted'] and not refused[0]['error'], refused
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def unlifted_sim():
  sim = signal_sim(oracle=False)
  sim.raw = road_runs()
  return sim


def braking_mid_carry(sim):
  """The coast, braking at 1.5 m/s^2 once rolling past 1.5 m/s (it sees something): comma's model, never lifted over."""
  st = sim.__dict__.setdefault('_brake', {'on': False})
  st['on'] = st['on'] or sim.v > 1.5
  if st['on']:
    return model_v2(*sim.model_lanes, accel=-1.5, **sim.model_kw)
  return coasting(sim)


def test_the_model_braking_mid_carry_ends_it_for_good():
  sim, plans, n0, said = run_left(braking_mid_carry, seconds=12.)
  carries = named(said, 'turnpilot.left_carry')
  assert len(carries) == 1
  ends = [kw for kw in named(said, 'turnpilot.go_lift_end') if kw['go_id'] == carries[0]['go_id']]
  assert len(ends) == 1 and ends[0]['reason'] == 'modelBrake', ends
  first = next(i for i, (a, _, _) in enumerate(sim.planned) if a < GO_MODEL_BRAKE)
  assert not any(on for _, _, on in sim.planned[first:])  # never lifted again over the model's brake
  assert -1.5 - 1e-6 <= min(a for a, _, _ in sim.planned[first:])  # the model's own braking, nothing harder from TurnPilot


def lifted_v(sim, n0):
  """The car's speed at each frame plannerd lifted (the go from the line's and the carry's)."""
  return [sim.trace[n0 + i][1] for i, (_, _, on) in enumerate(sim.planned) if on]


def slowing_mid_carry(a):
  """The coast, slowing at a m/s^2 once rolling past 1.5 m/s, its speed plan falling at a: a gentle stop for something it
  sees and the detector doesn't score, its shouldStop not out in motion (comma's own comes only near a standstill)."""
  def fn(sim):
    st = sim.__dict__.setdefault('_slow', {'on': False})
    st['on'] = st['on'] or sim.v > 1.5
    if not st['on']:
      return coasting(sim)
    m = model_v2(*sim.model_lanes, accel=a, **sim.model_kw).as_builder()
    m.velocity.x = [max(float(sim.v) + a * t, 0.) for t in ModelConstants.T_IDXS]
    return m.as_reader()
  return fn


@pytest.mark.parametrize('a', [-0.8, -0.4])
def test_a_model_slowing_gently_mid_carry_is_never_lifted_over(a):
  """Review 2026-11-08: the carry's only model end in motion was GO_MODEL_BRAKE (-1.0 m/s^2, the nudge's from a stand), so a
  model slowing at -0.8 from 1.75 m/s was lifted to +1.5 for 1.8 s, to 4.0 m/s, toward the exit crosswalk. Now its first
  slowing frame ends the carry for good ('modelSlow'): the model's own plan from there, nothing harder from TurnPilot."""
  sim, plans, n0, said = run_left(slowing_mid_carry(a), range_m=FULL_PASS_RANGE_M, seconds=14.)
  carries = named(said, 'turnpilot.left_carry')
  assert len(carries) == 1 and carries[0]['v'] < 1.5, carries
  ends = [kw for kw in named(said, 'turnpilot.go_lift_end') if kw['go_id'] == carries[0]['go_id']]
  assert len(ends) == 1 and ends[0]['reason'] == 'modelSlow' and ends[0]['v'] < 1.5 + CARRY_ACCEL * 2 * DT_MDL, ends
  # its first slowing frame (the model's own hold at the line before it, -0.4 to -0.5, is the stand's)
  first = next(i for i, (p, _, _) in enumerate(sim.planned) if p < CARRY_MODEL_SLOW and sim.trace[n0 + i][1] > 1.)
  assert sum(on for _, _, on in sim.planned[:first]) > 10 and not any(on for _, _, on in sim.planned[first:])
  assert max(lifted_v(sim, n0)) < 1.5 + 0.1
  assert a - 1e-6 <= min(p for p, _, _ in sim.planned[first:])  # the model's own slowing, nothing harder from TurnPilot
  # the carry before this fix: lifted on over the model's slowing to the profile's speed
  with mock.patch.object(go_mod, 'CARRY_MODEL_SLOW', GO_MODEL_BRAKE):
    old, _, o0, _ = run_left(slowing_mid_carry(a), range_m=FULL_PASS_RANGE_M, seconds=14.)
  assert max(lifted_v(old, o0)) > 3.5


def phantom_at_the_line(sim):
  """The coast with one frame of a model lead 8 m up (a phantom: BUILD_FINDINGS D1, the one at a green that ended the go)
  while the go from the line is out and the car rolls at 0.4 m/s or more: that go withdrawn before the model's takeover."""
  st = sim.__dict__.setdefault('_phantom', {'n': 0})
  gc = sim.tp.go_cert
  if st['n'] < 1 and gc.active and not gc.carry and sim.v >= 0.4:
    st['n'] += 1
    m = model_v2(*sim.model_lanes, accel=0., lead_prob=0.9, lead_x=8., **sim.model_kw).as_builder()
    m.velocity.x = [float(sim.v)] * len(m.velocity.x)
    return m.as_reader()
  return coasting(sim)


def test_a_coast_whose_go_from_the_line_was_withdrawn_before_the_takeover_is_still_carried():
  """Review 2026-11-08: the go from the line withdrawn by a one-frame lead before the model's takeover held kept no_move's
  clock running (go_wait_s, reset while the car rolls), which the carry's gate read as that go not over: committed with
  carry at 0.55 m/s, the carry never came and nothing said so, 21.8 s in the oncoming lanes. Now it comes once that go is
  over, however it ended: across in about the profile's own time, never a handback."""
  sim, plans, n0, said = run_left(phantom_at_the_line, range_m=FULL_PASS_RANGE_M)
  line = named(said, 'turnpilot.go_end')[0]
  assert line['reason'] == 'lead' and not line['followed'], line  # the go from the line withdrawn before its takeover
  committed = named(said, 'turnpilot.left_committed')
  assert len(committed) == 1 and committed[0]['carry'] and committed[0]['v'] < 0.7, committed
  assert len(named(said, 'turnpilot.left_carry')) == 1 and not named(said, 'turnpilot.left_carry_refused')
  v0 = entry_v(sim, n0)
  assert in_lanes_s(sim, n0) <= profile_s(v0, LANES_M) + 0.6 + 0.3, (in_lanes_s(sim, n0), profile_s(v0, LANES_M))
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert sim.s > JUNCTION + CLEAR_PATH_PAST_M


def phantom_at_the_commit(sim):
  """The coast with a model lead 8 m up (a phantom) for 0.5 s from the frame the left commits."""
  st = sim.__dict__.setdefault('_phantom', {'t': None})
  left = sim.tp.left_now
  if st['t'] is None and left is not None and left.state == 'committed':
    st['t'] = sim.t
  if st['t'] is not None and sim.t - st['t'] < 0.5:
    m = model_v2(*sim.model_lanes, accel=0., lead_prob=0.9, lead_x=8., **sim.model_kw).as_builder()
    m.velocity.x = [float(sim.v)] * len(m.velocity.x)
    return m.as_reader()
  return coasting(sim)


def test_a_carry_withdrawn_by_a_phantom_lead_comes_back_once_it_clears():
  """Review 2026-11-08: a carry withdrawn by turnpilotd's rails (withdraw_go: here a 0.5 s phantom lead within GO_CLEAR_M
  from the commit, which withdrew it on its first frame) was over for good, so the coast crawled 14.1 s through the oncoming
  lanes, the case the carry exists for. Now it is certified again, a go of its own, once the lead clears, the refusal said
  once while it holds: across in about the profile's time from there, never a handback. The model's own ends stay for good
  (test_a_model_slowing_gently_mid_carry_is_never_lifted_over)."""
  sim, plans, n0, said = run_left(phantom_at_the_commit, range_m=FULL_PASS_RANGE_M)
  carries = named(said, 'turnpilot.left_carry')
  assert len(carries) == 2 and carries[0]['go_id'] < carries[1]['go_id'], carries
  first = [kw for kw in named(said, 'turnpilot.go_end') if kw['go_id'] == carries[0]['go_id']]
  assert len(first) == 1 and first[0]['reason'] == 'lead', first  # the first carry withdrawn by the phantom
  refused = named(said, 'turnpilot.left_carry_refused')
  assert [kw['why'] for kw in refused] == ['lead'] and refused[0]['error'], refused  # said once while the lead held
  assert carries[1]['v'] < LEFT_LAUNCH_SPEED
  v0 = entry_v(sim, n0)
  assert in_lanes_s(sim, n0) <= profile_s(v0, LANES_M) + 0.5 + 0.6 + 0.3, (in_lanes_s(sim, n0), profile_s(v0, LANES_M))
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert sim.s > JUNCTION + CLEAR_PATH_PAST_M
