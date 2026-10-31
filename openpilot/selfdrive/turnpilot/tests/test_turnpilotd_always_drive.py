"""TurnPilotAlwaysDrive (ALWAYS_DRIVE.md section 1, step S1): turnpilotd hands the switch every frame to the lane planner, the
unprotected left, the map controls, the turn skip and the stop-sign decider's Frame, which act on it from steps S2 to S5; it
does nothing in shadow mode, and a component made afresh mid-frame (a reset on a fault or a handback) has it at once.
Step S2, stands and stalls, closed loop: a release the model doesn't follow (ST-04, ST-11, ST-13, ST-06's follow-up, ST-12),
a cap-only hold (ST-08), a stall in a curb-side turn (LEFT-21), a queue in a turn (ST-09 (stalls)), a stand at a signal
(ST-07 (stops)), a release where no signal is mapped (LEFT-20), section 4.5's unstick lift, and the crosswalk hold's standoff
(SC-02, unconditional)."""
import math
from unittest import mock

import pytest

from openpilot.cereal import log, messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.turnpilot.go_override import GO_CLEAR_M, GO_END_SPEED, TAKEOVER_S
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
from openpilot.selfdrive.turnpilot.scene_policy import (CROSSWALK_X_AFTER_M, CROSSWALK_X_BEFORE_M, CROSSWALK_Y_FAR_M, CROSSWALK_Y_NEAR_M,
                                                         NO_ACCEL_CAP, SceneDecision)
from openpilot.selfdrive.turnpilot.stop_go import Frame
from openpilot.selfdrive.turnpilot.tests.test_camera_lights import detections
from openpilot.selfdrive.turnpilot.tests.test_go_override import WARM_S
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of, model_v2
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import (LEFT_AT_A_RED, RIGHT_AT_A_CORNER, T_LEFT, WALKER, Sim, go_from_a_red,
                                                                 model_stands_in_the_turn, oracle_lights, stand, stop_at, walker_stops_the_car)
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_go import attach_planner, holding, model_stops_at, raw_person
from openpilot.selfdrive.turnpilot.unprotected_left import LIGHT_RED_MAX_S, STAND_UNDECIDED_S
from openpilot.selfdrive.turnpilot.turnpilotd import (LEAD_STAND_MAX_S, LIGHT_GREEN_REASON, LIGHT_WAIT_NOTE, NEAR_CONTROL_STAND_S, NO_MOVE_MAX_S,
                                                      NO_MOVE_REASON, NO_MOVE_S, PED_NOTE_S, PED_WAITING_NOTE, PED_WAITING_REASON, QUEUE_STILL_MAX_S,
                                                      RELEASE_GOES, RELEASE_LIFT_M, STALLED_REASON, STALL_LIFTS, STALL_LIFT_GAP_S, STALL_LIFT_REASON,
                                                      STALL_MIN_DEG, STALL_PROMPT_S, STALL_REPULSE_S, STANDSTILL_SPEED, STOPPED_HANDBACK_S,
                                                      STOPPED_NOTE, STOPPED_NOTE_S, TurnPilot, UNSTICK_S)

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
  # the model holding, the lifts never moving the car: a go at its window's end every RELIFT_GAP_S, RELEASE_GOES in all
  assert len({go_id_of(q) for q in [p] + plans} - {None}) == RELEASE_GOES
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
  person = {'cls': 'pedestrian', 'x': 20., 'y': -12., 'vx': 0., 'vy': 0., 'inPath': False}  # 4 m right of the path there
  plans = run(objects=[person], model=curved, deg=30.)
  assert not lifts_of(plans)
  assert any(q.controlPrompt == 'stalledTakeControl' for q in plans)
  plans = run(objects=[dict(person, y=4.)], model=curved, deg=30.)  # 12 m off the path, out of the zone: lifted
  assert lifts_of(plans)
  sim = stalled_right(True, 30.)
  assert not in_zone(sim, 20., -12.) and not in_zone(sim, 20., 4.), "the path's veto alone, never the zone's"


def in_zone(sim, x, y):
  """The car's (x, y left) is in the right's crosswalk zone in the road's frame (scene_policy's CROSSWALK_*)."""
  a = math.radians(sim.tp.approach_yaw_deg) * -1
  c, s = math.cos(a), math.sin(a)
  xr, yr, d = x * c - y * s, x * s + y * c, sim.tp.follower.distance
  return d - CROSSWALK_X_BEFORE_M < xr < d + CROSSWALK_X_AFTER_M and CROSSWALK_Y_NEAR_M < -yr < CROSSWALK_Y_FAR_M


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


# --- ST-09 (stalls): a queue in a turn ---

def queued(sim, seconds, moves_every=None):
  """The model stands the car in its turn behind a lead 6 m ahead that a vehicle track in the path corroborates (its velocity
  trusted). moves_every: the car ahead creeps on at 1.5 m/s for 1 s this often (a queue); None: it stands still."""
  plans, t0 = [], sim.t
  for _ in range(round(seconds / DT_MDL)):
    k = sim.t - t0
    vx = 1.5 if moves_every is not None and k % moves_every < 1. else 0.
    car = ({'cls': 'car', 'x': 6., 'y': 0.2, 'vx': vx, 'vy': 0., 'inPath': True, 'age': 40},)
    plans += stand(sim, DT_MDL, lead_prob=0.95, lead_x=6., objects=car)
  return plans


def test_a_queue_in_a_turn_behind_a_moving_lead_has_no_bound_and_a_still_one_hands_back_at_60_s():
  """ALWAYS_DRIVE.md ST-09 (stalls): under the switch a lead the live scene corroborates whose vehicle track moved within the
  last QUEUE_STILL_MAX_S keeps a stand in a turn with no bound (queueing is the model's); one still that long hands back
  ('Stalled in the turn'). Switch off: LEAD_STAND_MAX_S, as today."""
  sim = stalled_right(True, 30.)
  plans = queued(sim, 120., moves_every=20.)
  assert not any(q.controlPrompt in TAKE_CONTROL_PROMPTS for q in plans)
  sim = stalled_right(True, 30.)
  plans = queued(sim, QUEUE_STILL_MAX_S + 2.)
  first = next(i for i, q in enumerate(plans) if q.controlPrompt == 'stalledTakeControl')
  assert abs(first * DT_MDL - QUEUE_STILL_MAX_S) < 0.2, first * DT_MDL
  sim = stalled_right(False, 30.)
  plans = queued(sim, LEAD_STAND_MAX_S + 2., moves_every=5.)
  first = next(i for i, q in enumerate(plans) if q.controlPrompt == 'stalledTakeControl')
  assert abs(first * DT_MDL - LEAD_STAND_MAX_S) < 0.2


# --- ST-07 (stops), ST-01 (stalls) at a signal: the model owns the light ---

STAND_T0 = 0.2  # s: at_a_signal's sim.run(0.2), the sim's t when the stand starts


def at_a_signal(on: bool, camera=None):
  """Standing 20 m short of a mapped signal (no accepted light source: the car's case), the model holding the car; the camera's
  shadow reader reading camera(sim) (a state, or None for no head) at the signal's far-side head every other frame."""
  sim = Sim(STRAIGHT, controls=[(20., 'signal')])
  sim.tp.always_drive = on
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

  def lights(sim):
    state = camera(sim) if camera is not None else None
    if state is not None and int(round(sim.t / DT_MDL)) % 2 == 0:
      sim.sm.put('turnpilotDetections', detections(state, 45. - sim.s))
    return none
  sim.lights = lights
  sim.run(STAND_T0)
  return sim


def test_a_stand_at_a_signal_is_waiting_at_the_light_and_a_camera_red_holds_its_handback_to_150_s():
  """ALWAYS_DRIVE.md ST-07 (stops): under the switch within STOPPED_CONTROL_M of a mapped signal the 5 s note reads 'Waiting at
  the light' (no chime, as it already is); while the camera's shadow reader reads red there the NEAR_CONTROL_STAND_S handback
  waits, never past LIGHT_RED_MAX_S from the stand's start; with no red read it comes at NEAR_CONTROL_STAND_S. It never brakes
  for the read. Switch off: the note and the 90 s handback as today, red or not."""
  sim = at_a_signal(True, camera=lambda sim: 'red')
  plans = stand(sim, LIGHT_RED_MAX_S + 1.)
  assert plans[round((STOPPED_NOTE_S + 0.5) / DT_MDL)].reason == LIGHT_WAIT_NOTE
  assert not any(str(q.controlAlert) == 'carStoppedNote' for q in plans)
  first = next(i for i, q in enumerate(plans) if q.controlPrompt == 'carStoppedTakeControl')
  assert abs(first * DT_MDL - LIGHT_RED_MAX_S) < 0.2, first * DT_MDL
  assert not any(q.shouldStop for q in plans)
  sim = at_a_signal(True)
  plans = stand(sim, NEAR_CONTROL_STAND_S + 1.)
  first = next(i for i, q in enumerate(plans) if q.controlPrompt == 'carStoppedTakeControl')
  assert abs(first * DT_MDL - NEAR_CONTROL_STAND_S) < 0.2
  sim = at_a_signal(False, camera=lambda sim: 'red')
  plans = stand(sim, NEAR_CONTROL_STAND_S + 1.)
  assert plans[round((STOPPED_NOTE_S + 0.5) / DT_MDL)].reason == STOPPED_NOTE
  first = next(i for i, q in enumerate(plans) if q.controlPrompt == 'carStoppedTakeControl')
  assert abs(first * DT_MDL - NEAR_CONTROL_STAND_S) < 0.2


def test_a_steady_camera_green_the_model_doesnt_go_on_chimes():
  """ALWAYS_DRIVE.md ST-07 (stops): under the switch, standing at the signal with nothing in the path, once the camera reads a
  steady green and the model still stands: a chime and 'Light green: car not moving' within 5 s, once for that green; no lift.
  Switch off: nothing said."""
  t_green = 30.

  def camera(sim):
    return 'red' if sim.t - STAND_T0 < t_green else 'green'
  for on in (True, False):
    sim = at_a_signal(on, camera=camera)
    plans = stand(sim, t_green + 10.)
    said = [i for i, q in enumerate(plans) if str(q.controlAlert) == 'lightGreenNotMoving']
    assert bool(said) is on, on
    if on:
      assert t_green <= said[0] * DT_MDL <= t_green + 5. and plans[said[0]].reason == LIGHT_GREEN_REASON
      assert said == list(range(said[0], said[0] + len(said))) and len(said) * DT_MDL <= 3.1  # one chime, held a moment
    assert all(str(q.goCertified.cause) == 'none' for q in plans)


# --- LEFT-20 (ST-07 (stalls)): a release where no signal is (RELEASE_UNSIGNALIZED under section 4.5's conditions) ---

def camera_heads(*heads):
  """turnpilotLights with no source and the camera's shadow reader live (lightStatus read) reading heads [(state, s of the
  head)] every frame (none: it reads no head of any colour)."""
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

  def lights(sim):
    msg = messaging.new_message('turnpilotDetections')
    td = msg.turnpilotDetections
    td.camera, td.lightStatus = 'road', 'read'
    td.lights.source, td.lights.shadow = 'camera', True
    td.lights.lights = [{'state': st, 'distance': s - sim.s, 'governs': True, 'votes': 4, 'conf': 0.7} for st, s in heads]
    sim.sm.put('turnpilotDetections', msg.as_reader().turnpilotDetections)
    return none
  return lights


NO_HEADS = camera_heads()


def released_where_no_signal_is(on: bool, status='ok', controls=(), lights=NO_HEADS, signs=False):
  """RIGHT_AT_A_CORNER (no signal there), navd's controls status status, the model stopping the car short of the corner and
  holding it (plannerd in the loop) while a walker crosses in front; they clear and TurnPilot's hold lets go. lights: the
  camera's reads; signs: each road run has a stop-sign box. Returns the plans from the walker clearing."""
  sim = Sim(RIGHT_AT_A_CORNER, controls=list(controls), controls_status=status)
  sim.tp.always_drive, sim.go_taps = on, False
  sim.lights = lights
  sim.raw = lambda s: RawFrame([], 'road', s.t, signs=((0., 2., 0.8),) if signs else ())
  sim.accel = model_stops_at(289.)
  while not (sim.v < 0.05 and sim.s > 280.):
    sim.step(engaged=True)
    assert sim.t < 80.
  attach_planner(sim, holding)
  held = [sim.step(WALKER, engaged=True) for _ in range(round(2. / DT_MDL))]
  assert held[-1].shouldStop
  return [sim.step(engaged=True) for _ in range(round(3. / DT_MDL))]


def certified_causes(plans):
  return {str(q.goCertified.cause) for q in plans} - {'none'}


def test_a_release_where_no_signal_is_mapped_is_lifted_only_under_section_4_5s_conditions():
  """ALWAYS_DRIVE.md LEFT-20: under the switch RELEASE_UNSIGNALIZED holds for a curb-side or straight release where
  release_light reads 'none' (navd's controls ok: the sidecar served), no map control is within 30 m, the camera's shadow
  reader is live and reads no head of any colour, and no detector stop-sign box was seen; never with 'partial', a head, a
  sign box or a control near; switch off, never (RELEASE_UNSIGNALIZED False)."""
  assert certified_causes(released_where_no_signal_is(True)) == {'turnRelease'}
  assert certified_causes(released_where_no_signal_is(False)) == set()
  assert certified_causes(released_where_no_signal_is(True, status='partial')) == set()
  assert certified_causes(released_where_no_signal_is(True, lights=camera_heads(('unknown', 330.)))) == set()
  assert certified_causes(released_where_no_signal_is(True, lights=None)) == set()  # the reader not live: no head can't be known
  assert certified_causes(released_where_no_signal_is(True, signs=True)) == set()
  assert certified_causes(released_where_no_signal_is(True, controls=[(315., 'yield')])) == set()


# --- section 4.5 (owner's yes, under the switch): the unstick lift on a model stop off any hold ---

def stood_off_any_hold(on: bool, seconds: float, status='ok', lights=NO_HEADS, signs=False, controls=(), **kw):
  """The model stands the car on a straight with nothing of TurnPilot's holding it (a model stop off any hold), navd's controls
  status status, the camera's reads lights, a stop-sign box in every road run with signs. Returns the plans of the stand."""
  sim = Sim(STRAIGHT, controls=list(controls), controls_status=status)
  sim.tp.always_drive = on
  sim.lights = lights
  sim.raw = lambda s: RawFrame([], 'road', s.t, signs=((0., 2., 0.8),) if signs else ())
  sim.run(1.)
  return sim, stand(sim, seconds, **kw)


def unstick_spans(plans):
  ids = [int(q.goCertified.id) for q in plans if str(q.goCertified.cause) == 'unstick']
  return sorted(set(ids))


def test_a_model_stop_off_any_hold_gets_one_unstick_lift_at_10_s_under_section_4_5s_conditions():
  """ALWAYS_DRIVE.md section 4.5 (the owner's yes): under the switch one lift per stand at UNSTICK_S, a GoCertified cause of its
  own (unstick), said with a chime, only where release_light reads 'none' (controls ok), no map control within 30 m, the
  camera's shadow reader reads no head of any colour, no detector stop-sign box, nothing in the path and no model lead within
  GO_CLEAR_M; no second try; the stand's 60 s handback as before. Switch off: never."""
  sim, plans = stood_off_any_hold(True, STOPPED_HANDBACK_S + 1.)
  ids = unstick_spans(plans)
  assert len(ids) == 1, ids
  first = next(i for i, q in enumerate(plans) if str(q.goCertified.cause) == 'unstick')
  assert abs(first * DT_MDL - UNSTICK_S) <= 0.15, first * DT_MDL
  assert any(str(q.controlAlert) == 'unstickLift' for q in plans[first - 2:first + 2])
  assert plans[-1].controlPrompt == 'carStoppedTakeControl' and not any(q.shouldStop for q in plans)
  for kw in ({'on': False}, {'status': 'partial'}, {'lights': camera_heads(('unknown', 60.))}, {'lights': None}, {'signs': True},
             {'controls': [(25., 'yield')]}, {'lead_prob': 0.9, 'lead_x': GO_CLEAR_M - 5.}):
    args = {'on': True, **kw}
    _, plans = stood_off_any_hold(args.pop('on'), UNSTICK_S + 4., **args)
    assert not unstick_spans(plans), kw


def test_a_stand_in_a_turn_never_gets_the_unstick_lift():
  """Section 4.5's lift is for a model stop off any hold, never a stand with the route's turn desire out: a stall partway round
  is LEFT-21's (its person veto), and its handback isn't lifted under at UNSTICK_S."""
  sim = stalled_right(True, 45., controls_status='ok')
  sim.lights = NO_HEADS
  sim.raw = lambda s: RawFrame([], 'road', s.t)
  x, y = car_frame(sim, sim.tp.follower.distance + 3., -12.)
  person = [{'cls': 'pedestrian', 'x': x, 'y': y, 'vx': 0., 'vy': 0., 'inPath': False, 'age': 1}]
  plans = stand(sim, UNSTICK_S + 5., objects=person)
  assert not unstick_spans(plans) and not lifts_of(plans)
  assert any(q.controlPrompt == 'stalledTakeControl' for q in plans)


# --- SC-02 (unconditional): the right-turn crosswalk hold's standoff ---

def held_at_the_corner(on: bool, walker):
  """A right at 300 m, the car held at its hold line by the crosswalk hold for walker(sim) (a dict or None), steady on every
  10 Hz detector run. Returns the sim, standing held."""
  sim = Sim([(300., "turn", "right"), (1500., "turn", "left")])
  sim.tp.always_drive = on
  sim.walker = walker
  while not (sim.v < 0.05 and sim.s > 250.):
    sim.step(objects_of(sim), engaged=True)
    assert sim.t < 60.
  assert sim.trace[-1][2].reason == "pedestrian in crosswalk" and sim.trace[-1][2].shouldStop
  return sim


def objects_of(sim):
  w = sim.walker(sim)
  return (dict(w, age=5 + int(sim.t * 10.)),) if w is not None else ()


def hold_for(sim, seconds):
  plans = []
  for _ in range(round(seconds / DT_MDL)):
    sim.v = 0.
    plans.append(sim.step(objects_of(sim), engaged=True))
  return plans


@pytest.mark.parametrize("on", [False, True])
def test_a_walker_standing_on_the_corner_is_said_at_10_s_and_handed_back_at_45_s_held(on):
  """ALWAYS_DRIVE.md SC-02 (unconditional): the crosswalk hold has no bound today, so a person waiting on the corner held a right
  for good with only a status line. Once the hold has stood the car PED_NOTE_S, one chime and 'Waiting for pedestrian'; once
  every walker holding it has stood still (velocity trusted and under WALKER_STAND_SPEED, or not yet trusted) for
  STAND_UNDECIDED_S, 'Pedestrian waiting: take control' with the car held, never a go past them."""
  sim = held_at_the_corner(on, lambda sim: {'cls': 'pedestrian', 'x': 300. - sim.s + 3., 'y': -5.})
  plans = hold_for(sim, STAND_UNDECIDED_S + 3.)
  said = [i for i, q in enumerate(plans) if str(q.controlAlert) == 'pedestrianWaiting']
  assert said and abs(said[0] * DT_MDL - PED_NOTE_S) < 0.3 and said == list(range(said[0], said[0] + len(said))), said[:3]
  assert all(q.reason == PED_WAITING_NOTE for q in plans[said[0]:said[-1]])
  first = next(i for i, q in enumerate(plans) if q.controlPrompt == 'pedestrianWaitingTakeControl')
  assert abs(first * DT_MDL - STAND_UNDECIDED_S) < 0.5, first * DT_MDL
  assert plans[first].reason == PED_WAITING_REASON
  assert all(q.shouldStop for q in plans) and all(q.controlPrompt == 'pedestrianWaitingTakeControl' for q in plans[first:])
  assert all(str(q.goCertified.cause) == 'none' for q in plans)


@pytest.mark.parametrize("on", [False, True])
def test_a_walker_moving_in_the_crosswalk_is_waited_through_without_a_handback(on):
  """SC-02: walkers who move are waited through however long, as the left waits through moving traffic."""
  def pacing(sim):
    return {'cls': 'pedestrian', 'x': 300. - sim.s + 1., 'y': -5., 'vx': 1.2 if int(sim.t / 2.) % 2 else -1.2}  # up and down it
  sim = held_at_the_corner(on, pacing)
  plans = hold_for(sim, STAND_UNDECIDED_S + 20.)
  assert not any(q.controlPrompt in TAKE_CONTROL_PROMPTS for q in plans) and all(q.shouldStop for q in plans)


def test_a_stand_at_a_turns_line_with_its_desire_out_never_gets_the_unstick_lift():
  """Section 4.5's lift is never for a stand with the route's turn desire out, short of the turn too (a model stop at a right's
  line, no hold: an unmapped red, a walker the detector doesn't score): the turn's own rules have it."""
  sim = Sim(RIGHT_AT_A_CORNER, cruise=8., controls_status='ok')
  sim.tp.always_drive = True
  sim.lights = NO_HEADS
  sim.raw = lambda s: RawFrame([], 'road', s.t)
  while 300. - sim.s > 12.:
    sim.step(engaged=True)
    assert sim.t < 120.
  assert sim.tp.turn_committed and sim.tp.desires.turned_deg < STALL_MIN_DEG
  plans = stand(sim, UNSTICK_S + 5.)
  assert not unstick_spans(plans)
