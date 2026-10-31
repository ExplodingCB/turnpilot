"""TurnPilotAlwaysDrive (ALWAYS_DRIVE.md section 1, step S1): turnpilotd hands the switch every frame to the lane planner, the
unprotected left, the map controls, the turn skip and the stop-sign decider's Frame, which act on it from steps S2 to S5; it
does nothing in shadow mode, and a component made afresh mid-frame (a reset on a fault or a handback) has it at once."""
import math
from unittest import mock

import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.turnpilot.go_override import GO_CLEAR_M, GO_END_SPEED, TAKEOVER_S
from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
from openpilot.selfdrive.turnpilot.scene_policy import NO_ACCEL_CAP, SceneDecision
from openpilot.selfdrive.turnpilot.stop_go import Frame
from openpilot.selfdrive.turnpilot.tests.test_go_override import WARM_S
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of, model_v2
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import (LEFT_AT_A_RED, RIGHT_AT_A_CORNER, T_LEFT, Sim, go_from_a_red,
                                                                 model_stands_in_the_turn, oracle_lights, stand, stop_at, walker_stops_the_car)
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_go import attach_planner, holding, raw_person
from openpilot.selfdrive.turnpilot.turnpilotd import (NO_MOVE_MAX_S, NO_MOVE_REASON, NO_MOVE_S, RELEASE_GOES, RELEASE_LIFT_M, STALL_LIFT_GAP_S,
                                                       STALL_LIFT_REASON, STALL_LIFTS, STALL_PROMPT_S, STALL_REPULSE_S, STALLED_REASON,
                                                       STANDSTILL_SPEED, STOPPED_NOTE, TurnPilot)

Desire = log.Desire


def parts(tp: TurnPilot) -> dict:
  return {'lanes': tp.lanes, 'paint_rule_lanes': tp.paint_rule_lanes, 'left': tp.left, 'map_controls': tp.map_controls,
          'skip': tp.skip}


def test_the_switch_is_off_by_default_everywhere():
  tp = TurnPilot()
  assert tp.always_drive is False and tp.always_drive_on is False
  assert all(part.always_drive is False for part in parts(tp).values())
  assert Frame().always_drive is False


@pytest.mark.parametrize("shadow", [False, True])
@pytest.mark.parametrize("on", [False, True])
def test_every_frame_hands_the_switch_on_and_never_in_shadow(on, shadow):
  sim = Sim([(400., 'turn', 'right')], shadow=shadow)
  sim.tp.always_drive = on
  sim.run(0.5)
  want = on and not shadow
  assert sim.tp.always_drive_on is want
  assert {k: part.always_drive for k, part in parts(sim.tp).items()} == dict.fromkeys(parts(sim.tp), want)
  sim.tp.always_drive = not on  # main()'s refresh flips it: the next frame has it everywhere
  sim.step(engaged=True)
  assert all(part.always_drive is ((not on) and not shadow) for part in parts(sim.tp).values())


def test_components_made_afresh_have_the_switch_at_once():
  sim = Sim([(400., 'turn', 'right')])
  sim.tp.always_drive = True
  sim.run(0.5)
  sim.tp.reset()  # a fault's or a turn-off's new components, before the next frame's hand-off
  assert all(part.always_drive is True for part in parts(sim.tp).values())


@pytest.mark.parametrize("on", [False, True])
def test_the_stop_sign_deciders_frame_carries_the_switch(on):
  sim = Sim([(1000., 'turn', 'right')], controls=[(200., 'stop')])
  sim.tp.always_drive = on
  frames = []
  real = sim.tp.stop_go.update

  def update(f, *a, **kw):
    frames.append(f)
    return real(f, *a, **kw)
  with mock.patch.object(sim.tp.stop_go, 'update', side_effect=update):
    stop_at(sim, 200.)
  assert frames and all(f.always_drive is on for f in frames)


# --- step S2, stands and stalls: a release the model doesn't follow (ST-04, ST-11, ST-13, ST-06's follow-up) ---

def go_id_of(plan):
  g = plan.goCertified
  return int(g.id) if str(g.cause) != 'none' else None


def released(on: bool, **kw):
  """LEFT_AT_A_RED (a T's stem left: no oncoming approach) held at its red 10 m before the junction by the sim's oracle, then
  its green: TurnPilot's go from the line (go_from_a_red), the switch on or off. Returns (sim, the plan that let go)."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')], **kw)
  sim.tp.always_drive = on
  return sim, go_from_a_red(sim)


def test_a_release_the_model_doesnt_follow_is_never_held_and_hands_back_loudly_at_20_s():
  """ALWAYS_DRIVE.md ST-04: under the switch a go from the line the model doesn't follow is never withdrawn into a TurnPilot
  hold: no shouldStop, no 0 speed cap, the turn's desire kept, and 'Can't start the turn: take control' only after
  NO_MOVE_MAX_S standing with nothing holding the car, still no hold. With the switch off: the hold and the handback at
  NO_MOVE_S, as today."""
  sim, p = released(True)
  assert go_id_of(p) is not None
  plans = stand(sim, NO_MOVE_MAX_S + 1.)
  first = next(i for i, q in enumerate(plans) if q.controlPrompt == 'cantStartTakeControl')
  assert abs((first + 1) * DT_MDL - NO_MOVE_MAX_S) <= 2 * DT_MDL + 1e-6, first * DT_MDL
  assert plans[first].reason == NO_MOVE_REASON and all(q.controlPrompt == 'cantStartTakeControl' for q in plans[first:])
  assert not any(q.shouldStop for q in plans) and min(q.speedCap for q in plans) > 1., "never a TurnPilot hold"
  assert not any(q.controlPrompt in ('carStoppedTakeControl', 'stalledTakeControl') for q in plans)
  assert not any(q.reason == STOPPED_NOTE for q in plans[:first]), "the no-move clock owns the stand"
  d = sim.tp.desires
  assert d.started and not d.done and d.maneuver_id == sim.tp.follower.maneuver_id, "the turn's desire is kept"

  sim, _ = released(False)
  plans = stand(sim, NO_MOVE_S + 0.3)
  assert plans[-1].controlPrompt == 'cantStartTakeControl' and plans[-1].shouldStop and plans[-1].speedCap == 0.


def test_the_no_hold_cant_start_clears_once_the_model_drives_off_on_its_own():
  """ALWAYS_DRIVE.md section 2, ST-04: the no-hold cantStartTakeControl is a WARNING the model drives on under, so it clears
  once the car moves on the model's takeover predicate: the car that then drives the turn isn't under 'Can't start the turn'.
  With the switch off it stays latched until the driver takes over."""
  sim, _ = released(True)
  assert stand(sim, NO_MOVE_MAX_S + 0.5)[-1].controlPrompt == 'cantStartTakeControl'
  plans = [sim.step(engaged=True) for _ in range(round(4. / DT_MDL))]  # the model drives on: its plan goes, the car moves
  cleared = next(i for i, q in enumerate(plans) if q.controlPrompt != 'cantStartTakeControl')
  assert cleared * DT_MDL <= TAKEOVER_S + 1.5 and all(q.controlPrompt == 'none' for q in plans[cleared:])
  assert sim.v > 1.


def test_a_go_refused_for_a_phantom_lead_is_certified_once_it_leaves_and_never_through_it():
  """ALWAYS_DRIVE.md ST-06's follow-up, ST-04: a model lead within GO_CLEAR_M refuses the go (kept: a RAIL); under the switch
  no TurnPilot hold follows, and a new certified go with a new id goes once the lead leaves 15 m with the car standing, at
  most RELEASE_GOES goes in a release; never one while the lead is within 15 m."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  sim.tp.always_drive = True
  sim.model_kw = {'lead_prob': 0.9, 'lead_x': GO_CLEAR_M - 5.}
  p = go_from_a_red(sim)
  assert go_id_of(p) is None  # refused: the lead
  plans = []
  for _ in range(5):
    plans += [(True, q) for q in stand(sim, 3., lead_prob=0.9, lead_x=GO_CLEAR_M - 5.)]
    plans += [(False, q) for q in stand(sim, 3.)]
  ids = {go_id_of(q) for _, q in plans} - {None}
  assert 1 <= len(ids) <= RELEASE_GOES, ids
  assert not any(go_id_of(q) is not None for lead, q in plans if lead)
  first_off = next(i for i, (lead, _) in enumerate(plans) if not lead)
  assert go_id_of(plans[first_off + 1][1]) is not None or go_id_of(plans[first_off + 2][1]) is not None
  assert not any(q.shouldStop for _, q in plans)


def ended_early(on=True, at_s=2.):
  """LEFT_AT_A_RED's go with plannerd in the loop and a model that holds (G0's stand): a phantom in the path for one detector
  run at_s into the lift withdraws it (inPath) with the car rolling. Returns (sim, s at the go, s at the withdrawal)."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  sim.tp.always_drive = on
  sim.go_taps, sim.tp.oracle_ok = False, True
  light = {'state': 'red'}
  sim.lights = lambda sim: oracle_lights(light['state'], 290.)(sim)
  stop_at(sim, 290.)
  attach_planner(sim, holding)
  for _ in range(round(WARM_S / DT_MDL)):
    sim.step(engaged=True)
  light['state'] = 'green'
  while sim.trace[-1][2].shouldStop:
    sim.step(engaged=True)
    assert sim.t < 120.
  s_go = sim.s
  phantom = {'on': False}
  sim.raw = lambda sim: RawFrame([raw_person(6., 0.)], 'road', sim.t) if phantom['on'] else None
  for _ in range(round(at_s / DT_MDL)):
    sim.step(engaged=True)
  phantom['on'] = True
  q = sim.step(engaged=True)
  phantom['on'] = False
  assert go_id_of(q) is None and sim.tp.go_cert.end == 'inPath'
  return sim, s_go, sim.s


def test_a_relift_starts_only_from_a_standstill_and_the_release_lifts_2_25_m_in_all():
  """ALWAYS_DRIVE.md ST-11, ST-13: a lift ended early (a phantom in the path) leaves the car rolling on the model's gentle
  braking: no new go while the car is at or over STANDSTILL_SPEED; once it stands (and RELIFT_GAP_S after the last go) a
  relift with a new id, withdrawn ('budget') once the release's lifts have moved the car RELEASE_LIFT_M in all, by odometer."""
  sim, s_go, s_end = ended_early()
  first_m = s_end - s_go
  assert 0.4 < first_m < RELEASE_LIFT_M - 0.5 and sim.v > STANDSTILL_SPEED
  lifted, starts = first_m, []
  prev = None
  for _ in range(round(15. / DT_MDL)):
    s0, v0 = sim.s, sim.v
    q = sim.step(engaged=True)
    gid = go_id_of(q)
    if gid is not None and gid != prev:
      starts.append((gid, v0))
    if gid is not None:
      lifted += sim.s - s0
    prev = gid
  assert starts, "a relift once the car stood"
  assert all(v < STANDSTILL_SPEED for _, v in starts), starts
  assert lifted <= RELEASE_LIFT_M + 0.1, lifted
  assert sim.tp.go_cert.end in ('budget', 'window') and lifted >= RELEASE_LIFT_M - 0.3, (sim.tp.go_cert.end, lifted)
  assert all(a >= -2. for a, _, _ in sim.planned)


def test_a_full_window_spends_the_release_and_no_second_lift_is_certified():
  """ALWAYS_DRIVE.md ST-11: a first lift whose window runs out moved the car its whole RELEASE_LIFT_M: the car comes back to
  rest, no second go is certified, and the release ends in the no-hold handback at NO_MOVE_MAX_S."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  sim.tp.always_drive = True
  sim.go_taps, sim.tp.oracle_ok = False, True
  light = {'state': 'red'}
  sim.lights = lambda sim: oracle_lights(light['state'], 290.)(sim)
  stop_at(sim, 290.)
  attach_planner(sim, holding, lag_s=DT_MDL)
  for _ in range(round(WARM_S / DT_MDL)):
    sim.step(engaged=True)
  light['state'] = 'green'
  while sim.trace[-1][2].shouldStop:
    sim.step(engaged=True)
  first = go_id_of(sim.trace[-1][2])
  assert first is not None
  plans = [sim.step(engaged=True) for _ in range(round((NO_MOVE_MAX_S + 10.) / DT_MDL))]  # its clock counts the stands only
  assert {go_id_of(q) for q in plans} - {None} <= {first}
  assert sim.tp.go_cert.end in ('window', 'budget')
  assert any(q.controlPrompt == 'cantStartTakeControl' for q in plans) and not any(q.shouldStop for q in plans)


def test_a_relifts_speed_end_keeps_the_no_move_clock():
  """ALWAYS_DRIVE.md ST-13: a relift ending on GO_END_SPEED without the takeover predicate leaves taken_over False: the
  release isn't followed, its no-move clock runs on."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  sim.tp.always_drive = True
  sim.model_kw = {'lead_prob': 0.9, 'lead_x': 5.}
  go_from_a_red(sim)
  stand(sim, 3., lead_prob=0.9, lead_x=5.)
  plans = stand(sim, 0.5)
  assert any(go_id_of(q) is not None for q in plans) and sim.tp.go_cert.relift
  sim.model_holds = True
  sim.v = GO_END_SPEED + 0.1  # the car at the lift's speed end, the model still holding
  sim.step(engaged=True)
  assert sim.tp.go_cert.end == 'speed' and not sim.tp.go_cert.taken_over
  assert sim.tp.go_wait_s is not None


# --- ST-12 (ST-09 (stops)): a straight release ---

STRAIGHT = [(2000., 'turn', 'left', T_LEFT)]  # nothing to turn at for 2 km


def straight_green(on: bool):
  """A mapped signal 290 m on along a straight, read red by the sim's oracle, then green: the map latch lets go with the car
  standing, no turn near. Returns (sim, the plan that let go)."""
  sim = Sim(STRAIGHT, controls=[(290., 'signal')])
  sim.tp.always_drive, sim.go_taps, sim.tp.oracle_ok = on, False, True
  light = {'state': 'red'}
  sim.lights = lambda sim: oracle_lights(light['state'], 290.)(sim)
  stop_at(sim, 290.)
  stand(sim, 1.)
  light['state'] = 'green'
  while sim.trace[-1][2].shouldStop:
    sim.step(engaged=True)
    assert sim.t < 80.
  return sim, sim.trace[-1][2]


def test_a_straight_release_certifies_like_a_turns_and_falls_to_the_release_rules():
  """ALWAYS_DRIVE.md ST-12: under the switch a map hold let go on a straight at an accepted green gets turn_go's certified
  lift (turnRelease through release_blocked and go_blocked, judged at the signal it stood at, not the route's next turn) and
  ST-04's rules: no hold, relifts, the no-hold handback at NO_MOVE_MAX_S. With the switch off: no lift, the 2 s hold."""
  sim, p = straight_green(True)
  assert str(p.goCertified.cause) == 'turnRelease'
  plans = stand(sim, 1.)
  assert all(str(q.goCertified.cause) == 'turnRelease' for q in plans), "kept: the junction it stands at is green"
  plans += stand(sim, NO_MOVE_MAX_S + 1.)
  assert not any(q.shouldStop for q in plans) and any(q.controlPrompt == 'cantStartTakeControl' for q in plans)
  sim, p = straight_green(False)
  assert str(p.goCertified.cause) == 'none'
  plans = stand(sim, NO_MOVE_S + 0.3)
  assert plans[-1].controlPrompt == 'cantStartTakeControl' and plans[-1].shouldStop


def test_a_scene_hold_let_go_on_a_straight_starts_the_release_clock():
  """ALWAYS_DRIVE.md ST-12: a walker's hold in the path on a straight, let go standing, starts the no-move clock under the
  switch (no lift: no signal there, RELEASE_UNSIGNALIZED off), the no-hold handback at NO_MOVE_MAX_S; with the switch off
  it starts none and the empty-road handback comes at STOPPED_HANDBACK_S, as today."""
  for on in (True, False):
    sim = Sim(STRAIGHT)
    sim.tp.always_drive = on
    sim.run(2.)
    walker_stops_the_car(sim)
    plans = stand(sim, NO_MOVE_MAX_S + 1.)
    assert not any(q.shouldStop for q in plans), on
    assert any(q.controlPrompt == 'cantStartTakeControl' for q in plans) is on
    assert all(str(q.goCertified.cause) == 'none' for q in plans)


# --- ST-08: a cap-only hold the car came to rest under ---

def test_a_cap_only_hold_the_car_came_to_rest_under_is_a_go_when_it_drops():
  """ALWAYS_DRIVE.md ST-08: a TurnPilot hold that only caps the speed (decision.holding, no shouldStop) with the car come to
  rest under it counts as a hold under the switch: its drop is TurnPilot's go (release_go: the turn's pulse again, go_id),
  keyed on the cap dropping, not the frame after the car stands. With the switch off nothing goes, as today."""
  for on in (True, False):
    sim = Sim(RIGHT_AT_A_CORNER, cruise=8.)
    sim.tp.always_drive = on
    while 300. - sim.s > 12.:
      sim.step(engaged=True)
      assert sim.t < 120.
    assert sim.tp.desires.started
    real = sim.tp.scene.update
    cap = {'on': True}

    def update(*a, real=real, cap=cap, **kw):
      d = real(*a, **kw)
      return SceneDecision(NO_ACCEL_CAP, 0.2, False, True, "pedestrian in crosswalk") if cap['on'] else d
    with mock.patch.object(sim.tp.scene, 'update', side_effect=update):
      held = stand(sim, 2.)
      assert all(not q.shouldStop for q in held) and sim.tp.go_id is None, on
      cap['on'] = False
      q = stand(sim, DT_MDL)[0]
    assert (sim.tp.go_id == sim.tp.follower.maneuver_id) is on, on
    assert (q.desire == Desire.turnRight) is on, on


# --- LEFT-21 (ST-03 (stalls)): a stall in a curb-side turn: a fresh pulse at 2 s, clearBox lifts from 3 s, then the handback ---

def stalled_right(on: bool, deg=45., **kw):
  """RIGHT_AT_A_CORNER's right driven deg in, then the model stands the car there (no TurnPilot hold, no go of its own)."""
  sim = Sim(RIGHT_AT_A_CORNER, cruise=8., **kw)
  sim.tp.always_drive = on
  model_stands_in_the_turn(sim, deg)
  return sim


def lifts_of(plans):
  """[(first index, last index)] of each clearBox go out in plans, by id."""
  spans = {}
  for i, q in enumerate(plans):
    if str(q.goCertified.cause) == 'clearBox':
      spans.setdefault(int(q.goCertified.id), [i, i])[1] = i
  return [tuple(v) for _, v in sorted(spans.items())]


def edges_of(plans, desire):
  """Indexes of desire's rising edges in plans (one already out at the first plan is no edge)."""
  return [i for i, q in enumerate(plans) if i > 0 and q.desire == desire and plans[i - 1].desire != desire]


def test_a_stall_in_a_curb_side_turn_is_pulsed_lifted_from_a_standstill_then_handed_back():
  """ALWAYS_DRIVE.md LEFT-21: under the switch a stand partway round a curb-side turn with nothing holding the car gets one
  fresh turn pulse at STALL_REPULSE_S and a clearBox lift from STALL_PROMPT_S (said: 'Stalled: finishing the turn' with a
  chime), up to STALL_LIFTS lifts STALL_LIFT_GAP_S apart, each from a standstill; then 'Stalled in the turn: take control',
  never a hold. Switch off: the handback at STALL_PROMPT_S, no pulse, no lift, as today."""
  sim = stalled_right(True)
  plans = stand(sim, 25.)
  pulses = edges_of(plans, Desire.turnRight)
  assert len(pulses) == 1 and abs(pulses[0] * DT_MDL - STALL_REPULSE_S) <= 0.15, [p * DT_MDL for p in pulses]
  spans = lifts_of(plans)
  assert len(spans) == STALL_LIFTS, spans
  assert abs(spans[0][0] * DT_MDL - STALL_PROMPT_S) <= 0.15
  assert all((b[0] - a[1]) * DT_MDL >= STALL_LIFT_GAP_S - 1e-6 for a, b in zip(spans, spans[1:], strict=False)), spans
  for start, _ in spans:
    assert str(plans[start].controlAlert) == 'stallLift' and plans[start].reason == STALL_LIFT_REASON
  handed = next(i for i, q in enumerate(plans) if q.controlPrompt == 'stalledTakeControl')
  assert handed > spans[-1][1] and plans[handed].reason == STALLED_REASON
  assert not any(q.controlPrompt == 'stalledTakeControl' for q in plans[:spans[-1][1]])
  assert not any(q.shouldStop for q in plans)
  assert all(str(sim.trace[len(sim.trace) - len(plans) + s][2].goCertified.cause) == 'clearBox' for s, _ in spans)

  sim = stalled_right(False)
  plans = stand(sim, STALL_PROMPT_S + 2.)
  assert not lifts_of(plans) and not edges_of(plans, Desire.turnRight)
  first = next(i for i, q in enumerate(plans) if q.controlPrompt == 'stalledTakeControl')
  assert abs(first * DT_MDL - STALL_PROMPT_S) < 0.15


def test_the_stalls_lifts_share_the_releases_2_25_m():
  """ALWAYS_DRIVE.md LEFT-21, ST-13: the stall's lifts move the car RELEASE_LIFT_M in all with any lift of the go that put it
  there: 1.5 m already lifted, the stall's lift is withdrawn ('budget') about 0.75 m on, and no more go with the car standing
  again: the handback."""
  sim = stalled_right(True)
  stand(sim, 1.)
  sim.tp.rel_lift_m = 1.5  # the release's lift that put the car in the junction
  attach_planner(sim, holding)
  plans = [sim.step(engaged=True) for _ in range(round(14. / DT_MDL))]
  spans = lifts_of(plans)
  assert len(spans) == 1 and sim.tp.go_cert.end == 'budget', (spans, sim.tp.go_cert.end)
  lifted = sum(sim.trace[len(sim.trace) - len(plans) + i][0] - sim.trace[len(sim.trace) - len(plans) + i - 1][0]
               for i in range(spans[0][0], spans[0][1] + 1))
  assert 0.5 < lifted <= RELEASE_LIFT_M - 1.5 + 0.1, lifted  # lifted metres; the model's own coast after it is the model's
  assert any(q.controlPrompt == 'stalledTakeControl' for q in plans[spans[0][1]:]) and not any(q.shouldStop for q in plans)


def car_frame(sim, xr, yr):
  """The car's (x, y left) of a point (xr along the approach, yr left of it) in the road's frame (scene_policy.road_frame) of a
  right turn the car has yawed approach_yaw_deg into."""
  a = math.radians(sim.tp.approach_yaw_deg) * -1
  c, s = math.cos(a), math.sin(a)
  return xr * c + yr * s, -xr * s + yr * c


def test_a_person_near_the_turn_bars_the_stalls_lift():
  """ALWAYS_DRIVE.md LEFT-21's person veto: go_blocked can't see the people a model stops for partway round. A person track of
  1 hit, or a raw person box alone, in the right's crosswalk zone in the road's frame 60 deg into the turn, or 4 m off the
  model's curved path 20 m on: no clearBox lift, the handback at STALL_PROMPT_S as today."""
  def run(objects=(), raw=None, model=None, deg=60.):
    sim = stalled_right(True, deg)
    d = sim.tp.follower.distance
    x, y = car_frame(sim, d + 3., -12.)
    assert math.hypot(x, y) > 5. and abs(y) > 4.6, (x, y)  # in the zone, clear of the straight corridor and the path
    if raw == 'box':
      sim.raw = lambda s: RawFrame([raw_person(x, y)], 'road', s.t)
    if model is not None:
      sim.model = model
    obj = [{'cls': 'pedestrian', 'x': x, 'y': y, 'vx': 0., 'vy': 0., 'inPath': False, 'age': 1}] if objects == 'zone' else list(objects)
    plans = stand(sim, STALL_PROMPT_S + 2., objects=obj)
    return plans
  for kind in ({'objects': 'zone'}, {'raw': 'box'}):
    plans = run(**kind)
    assert not lifts_of(plans), kind
    first = next(i for i, q in enumerate(plans) if q.controlPrompt == 'stalledTakeControl')
    assert abs(first * DT_MDL - STALL_PROMPT_S) < 0.15, kind

  def curved(sim):
    m = model_v2(holds=True).as_builder()
    xs = [float(v) for v in ModelConstants.X_IDXS]
    m.position.x = xs
    m.position.y = [0.02 * x * x for x in xs]  # modelV2's y is right: curving right, 8 m right of the axis 20 m on
    return m.as_reader()
  person = {'cls': 'pedestrian', 'x': 20., 'y': -4., 'vx': 0., 'vy': 0., 'inPath': False}  # 4 m left of the path there
  plans = run(objects=[person], model=curved, deg=30.)
  assert not lifts_of(plans)
  assert any(q.controlPrompt == 'stalledTakeControl' for q in plans)
  plans = run(objects=[dict(person, y=4.)], model=curved, deg=30.)  # 12 m off the path, out of the zone: lifted
  assert lifts_of(plans)


def test_a_person_coming_near_the_turn_mid_lift_withdraws_it():
  sim = stalled_right(True, 60.)
  plans = stand(sim, STALL_PROMPT_S + 0.5)
  assert lifts_of(plans) and str(plans[-1].goCertified.cause) == 'clearBox'
  x, y = car_frame(sim, sim.tp.follower.distance + 3., -12.)
  person = [{'cls': 'pedestrian', 'x': x, 'y': y, 'vx': 0., 'vy': 0., 'inPath': False, 'age': 1}]
  q = stand(sim, DT_MDL, objects=person)[0]
  assert str(q.goCertified.cause) == 'none' and sim.tp.go_cert.end == 'person'
  plans = stand(sim, 6., objects=person)
  assert not lifts_of(plans) and any(p.controlPrompt == 'stalledTakeControl' for p in plans)


def test_a_stall_with_a_model_lead_or_in_a_far_side_turn_is_never_lifted():
  """ALWAYS_DRIVE.md LEFT-21, ST-10: a model lead within 15 m (the g62 phantom, the s20 mailbox) refuses the lift, the handback
  at STALL_PROMPT_S as now; a far-side turn stalled in the junction is never lifted (nothing the lift checks sees the oncoming
  lanes), nor pulsed."""
  sim = stalled_right(True)
  plans = stand(sim, STALL_PROMPT_S + 2., lead_prob=0.95, lead_x=8.)
  assert not lifts_of(plans)
  assert abs(next(i for i, q in enumerate(plans) if q.controlPrompt == 'stalledTakeControl') * DT_MDL - STALL_PROMPT_S) < 0.15
  sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))], cruise=8.)
  sim.tp.always_drive = True
  model_stands_in_the_turn(sim, 45., side=1)
  plans = stand(sim, STALL_PROMPT_S + 2.)
  assert not lifts_of(plans) and not edges_of(plans, Desire.turnLeft)
  assert any(q.controlPrompt == 'stalledTakeControl' for q in plans)
