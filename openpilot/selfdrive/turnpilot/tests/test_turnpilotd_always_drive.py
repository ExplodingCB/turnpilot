"""TurnPilotAlwaysDrive (ALWAYS_DRIVE.md section 1, step S1): turnpilotd hands the switch every frame to the lane planner, the
unprotected left, the map controls, the turn skip and the stop-sign decider's Frame, which act on it from steps S2 to S5; it
does nothing in shadow mode, and a component made afresh mid-frame (a reset on a fault or a handback) has it at once.
Step S2, stands and stalls, closed loop: a release the model doesn't follow (ST-04, ST-11, ST-13, ST-06's follow-up, ST-12),
a cap-only hold (ST-08), a stall in a curb-side turn (LEFT-21), a queue in a turn (ST-09 (stalls)), a stand at a signal
(ST-07 (stops)), a release where no signal is mapped (LEFT-20), section 4.5's unstick lift, the stop-sign memory both read
(metres driven since a forward run's box, never seconds standing), and the crosswalk hold's standoff (SC-02, unconditional)."""
import math
from unittest import mock

import pytest

from openpilot.cereal import log, messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.turnpilot.go_override import GO_CLEAR_M, GO_END_SPEED, TAKEOVER_S
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN
from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
from openpilot.selfdrive.turnpilot.scene_policy import (CROSSWALK_X_AFTER_M, CROSSWALK_X_BEFORE_M, CROSSWALK_Y_FAR_M, CROSSWALK_Y_NEAR_M,
                                                         NO_ACCEL_CAP, SceneDecision)
from openpilot.selfdrive.turnpilot.stop_go import Frame
from openpilot.selfdrive.turnpilot.tests.test_camera_lights import detections
from openpilot.selfdrive.turnpilot.tests.test_go_override import WARM_S
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of, model_v2
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import (LEFT_AT_A_RED, RIGHT_AT_A_CORNER, T_LEFT, WALKER, WALKER_CLEAR, WHITE, YELLOW, Sim,
                                                                 go_from_a_red, model_stands_in_the_turn, oracle_lights, stand, stop_at,
                                                                 walker_stops_the_car)
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_go import attach_planner, holding, model_stops_at, raw_person
from openpilot.selfdrive.turnpilot.unprotected_left import LIGHT_RED_MAX_S, STAND_UNDECIDED_S
from openpilot.selfdrive.turnpilot.turnpilotd import (LEAD_STAND_MAX_S, LIGHT_GREEN_REASON, LIGHT_UNREAD_REASON, LIGHT_UNREAD_S, LIGHT_WAIT_NOTE,
                                                      NEAR_CONTROL_STAND_S, NO_MOVE_MAX_S,
                                                      NO_MOVE_REASON, NO_MOVE_S, PED_NOTE_S, PED_WAITING_NOTE, PED_WAITING_REASON, QUEUE_STILL_MAX_S,
                                                      RELEASE_GOES, RELEASE_LIFT_M, STALLED_REASON, STALL_HANDED_MOVED_M, STALL_LIFTS,
                                                      STALL_LIFT_GAP_S, STALL_LIFT_REASON,
                                                      STALL_MIN_DEG, STALL_PROMPT_S, STALL_REPULSE_S, STANDSTILL_SPEED, STOPPED_HANDBACK_S,
                                                      SIGN_CLEAR_M, SIGN_RUN_FRESH_S, STOPPED_NOTE, STOPPED_NOTE_S, TurnPilot,
                                                      UNSTICK_S)

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
    plans = stand(sim, DT_MDL, objects=WALKER_CLEAR) + stand(sim, NO_MOVE_MAX_S + 1. - DT_MDL)
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


def test_a_stalls_handback_for_a_flickering_model_lead_stays_and_no_lift_follows_it():
  """SAT_RERUN R4/R5 (red_right_g62, red_right): the car stood 58 deg into the right against the far kerb, the model's lead
  (the building, 14.6 m) read p 0.72 one frame and under the lift's bar the next. The lift refused for it, 'Stalled in the turn:
  take control' showed 0.1 s, then 'Finishing the Turn' and a clearBox lift toward the lead (R5: lift 5 moved the car to
  1.71 m/s 8 s after the handback, the lead still read). Once handed back the stand stays handed back: the prompt on every frame
  and no lift, until the car moves, the driver acts or the turn changes."""
  sim = stalled_right(True, deg=58.)
  plans = []
  for k in range(round((STALL_PROMPT_S + 10.) / DT_MDL)):
    plans += stand(sim, DT_MDL, lead_prob=0.72 if k % 2 else 0.05, lead_x=14.6)
  handed = next(i for i, q in enumerate(plans) if q.controlPrompt == 'stalledTakeControl')
  assert all(q.controlPrompt == 'stalledTakeControl' for q in plans[handed:]), \
    [i * DT_MDL for i, q in enumerate(plans[handed:], handed) if q.controlPrompt != 'stalledTakeControl'][:5]
  assert not [s for s in lifts_of(plans) if s[0] > handed], lifts_of(plans)
  assert not any(q.shouldStop for q in plans)


def rolled(sim, speeds, flicker=False, **kw):
  """stand(), the car's speed speeds[k] on frame k instead of 0 (a lift's roll); flicker: lead_prob read on odd frames only."""
  lead, lead_x = kw.pop('lead_prob', None), kw.pop('lead_x', 30.)
  plans = []
  sim.model_holds = True
  for k, v in enumerate(speeds):
    on = lead is not None and (not flicker or k % 2)
    sim.model_kw = {'lead_prob': lead, 'lead_x': lead_x} if on else {}
    sim.v = v
    plans.append(sim.step(engaged=True, **kw))
  sim.model_holds = False
  return plans


def test_a_stalls_handback_stays_through_the_roll_of_the_lift_it_refused():
  """SAT_RERUN C7 (red_right_g62): clearBox lift 2 ended after 0.6 s for the model's lead (the building, 13.2 m, p 0.93) with the
  car rolling 0.25-0.5 m/s from that lift. 'Stalled in the turn: take control' came on the refusal and went 0.1 s later, the
  latch cleared by the roll's first frame at STANDSTILL_SPEED, and clearBox lift 3 went 3 s after toward the same building.
  The refused lift's roll is no move: the handback stays on every frame and no lift follows it. The car driving on
  STALL_HANDED_MOVED_M is: the stand it handed back is over."""
  n = round(1. / DT_MDL)
  sim = stalled_right(True, deg=58.)
  plans = stand(sim, STALL_PROMPT_S + 0.2)
  assert lifts_of(plans), "the stall's first lift"
  plans += rolled(sim, [0.02 * k for k in range(13)])  # the lift's first 0.6 s from a standstill, to 0.24 m/s
  plans += rolled(sim, [0.25 + 0.25 * k / n for k in range(n)] + [0.5 * (1. - k / n) for k in range(n)], flicker=True,
                  lead_prob=0.93, lead_x=13.2)  # the lead read, the lift refused, the roll up to 0.5 m/s and down again
  plans += rolled(sim, [0.] * round(8. / DT_MDL), flicker=True, lead_prob=0.93, lead_x=13.2)  # standing again, the lead flickering
  handed = next(i for i, q in enumerate(plans) if q.controlPrompt == 'stalledTakeControl')
  assert all(q.controlPrompt == 'stalledTakeControl' for q in plans[handed:]), \
    [round(i * DT_MDL, 2) for i, q in enumerate(plans[handed:], handed) if q.controlPrompt != 'stalledTakeControl'][:5]
  assert not [s for s in lifts_of(plans) if s[0] > handed], lifts_of(plans)
  assert sim.tp.odo_m - sim.trace[len(sim.trace) - len(plans) + handed][0] < STALL_HANDED_MOVED_M  # the roll, by the odometer
  # the model driving on: past STALL_HANDED_MOVED_M the stand is over, and so is its handback
  moved = rolled(sim, [1.] * round((STALL_HANDED_MOVED_M + 0.5) / DT_MDL))
  assert moved[-1].controlPrompt == 'none'
  assert not any(q.shouldStop for q in plans + moved)


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


def test_a_stand_at_a_signal_with_no_red_read_is_said_loudly_long_before_its_handback():
  """The confirm round's C8 (route_demo): the model stood 85 s at a mapped signal's green, 7 m short of a rerouted U-turn,
  nothing in its path, and no light source read anything there: 'Waiting at the light' on the status line alone until the
  90 s handback. A stand at a mapped signal with no red or yellow read for LIGHT_UNREAD_S is said with a chime, once a stand
  (lightNotRead, LIGHT_UNREAD_REASON); never a lift, and the handback still at NEAR_CONTROL_STAND_S. A camera red there
  explains the stand: nothing said. Switch on or off."""
  for on in (True, False):
    sim = at_a_signal(on)
    plans = stand(sim, NEAR_CONTROL_STAND_S + 1.)
    said = [i for i, q in enumerate(plans) if str(q.controlAlert) == 'lightNotRead']
    assert said and abs(said[0] * DT_MDL - LIGHT_UNREAD_S) < 0.15, (on, said[:1])
    assert said == list(range(said[0], said[0] + len(said))) and len(said) * DT_MDL <= 3.1  # one chime, held a moment
    assert plans[said[0]].reason == LIGHT_UNREAD_REASON and plans[said[0]].controlPrompt == 'none'
    assert LIGHT_UNREAD_S <= NEAR_CONTROL_STAND_S / 3.
    first = next(i for i, q in enumerate(plans) if q.controlPrompt == 'carStoppedTakeControl')
    assert abs(first * DT_MDL - NEAR_CONTROL_STAND_S) < 0.2, on
    assert all(str(q.goCertified.cause) == 'none' for q in plans) and not any(q.shouldStop for q in plans)
    sim = at_a_signal(on, camera=lambda sim: 'red')
    assert not any(str(q.controlAlert) == 'lightNotRead' for q in stand(sim, LIGHT_UNREAD_S + 5.)), on


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


SIGN_BOX = ((0., 2., 0.8),)  # a stop-sign box's (bearing, elevation, conf)


def sign_runs(signs, camera='road'):
  """The detector's forward run of each frame (sim.raw), a full pass on camera, for signs: False, no stop-sign box; True, a
  box in every run; 'approach', a box in every run while the car moves (STANDSTILL_SPEED or more) and none once it stands
  (the sign beside the car at the line, out of the road camera's view); 'wide', the road camera's run with no box and the
  wide camera's with the approach's box on alternate frames (as modeld alternates them in a turn)."""
  if signs == 'wide':
    return lambda s: RawFrame([], 'wide', s.t, signs=SIGN_BOX if s.v >= STANDSTILL_SPEED else ()) if round(s.t / DT_MDL) % 2 else \
      RawFrame([], 'road', s.t)
  if signs == 'approach':
    return lambda s: RawFrame([], camera, s.t, signs=SIGN_BOX if s.v >= STANDSTILL_SPEED else ())
  return lambda s: RawFrame([], camera, s.t, signs=SIGN_BOX if signs else ())


def released_where_no_signal_is(on: bool, status='ok', controls=(), lights=NO_HEADS, signs=False, hold_s=2., camera='road'):
  """RIGHT_AT_A_CORNER (no signal there), navd's controls status status, the model stopping the car short of the corner and
  holding it (plannerd in the loop) while a walker crosses in front for hold_s; they clear and TurnPilot's hold lets go.
  lights: the camera's reads; signs: the forward runs' stop-sign boxes (sign_runs, on camera). Returns the plans from the
  walker clearing."""
  sim = Sim(RIGHT_AT_A_CORNER, controls=list(controls), controls_status=status)
  sim.tp.always_drive, sim.go_taps = on, False
  sim.lights = lights
  sim.raw = sign_runs(signs, camera)
  sim.accel = model_stops_at(289.)
  while not (sim.v < 0.05 and sim.s > 280.):
    sim.step(engaged=True)
    assert sim.t < 80.
  attach_planner(sim, holding)
  held = [sim.step(WALKER, engaged=True) for _ in range(round(hold_s / DT_MDL))]
  assert held[-1].shouldStop
  return [sim.step(WALKER_CLEAR, engaged=True)] + [sim.step(engaged=True) for _ in range(round(3. / DT_MDL) - 1)]


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


@pytest.mark.parametrize('signs, camera', [('approach', 'road'), ('approach', 'wide'), ('wide', 'road')])
def test_a_stop_sign_seen_only_on_the_approach_bars_the_unsignalized_release(signs, camera):
  """LEFT-20's stop-sign check is for an unmapped stop sign, which at the line stands beside the car, out of the road
  camera's view: a box seen only while the car drove up to it, on the road camera's full pass or the wide camera's (alone,
  or between road runs with none), bars the release's lift however long the car has stood (the walker's 6 s hold here is past
  the 5 s the check used to forget it in); the same release with no box seen, on the same camera's runs, is lifted."""
  assert certified_causes(released_where_no_signal_is(True, signs=signs, hold_s=6., camera=camera)) == set()
  assert certified_causes(released_where_no_signal_is(True, hold_s=6., camera=camera)) == {'turnRelease'}


# --- section 4.5 (owner's yes, under the switch): the unstick lift on a model stop off any hold ---

# m: where stood_off_any_hold's and stood_at's model stands the car on STRAIGHT, past SIGN_CLEAR_M of road from turnpilotd's
# start (which counts as a stop sign just seen)
STAND_AT_M = 160.


def stood_off_any_hold(on: bool, seconds: float, status='ok', lights=NO_HEADS, signs=False, controls=(), model_lanes=(0, 0), paint=(),
                       **kw):
  """The model stands the car at STAND_AT_M on a straight with nothing of TurnPilot's holding it (a model stop off any hold),
  navd's controls status status, the camera's reads lights, the forward runs' stop-sign boxes signs (sign_runs), the model's
  lanes model_lanes (left, right of the car's) and the lane paint paint. Returns the plans of the stand."""
  sim = Sim(STRAIGHT, controls=list(controls), controls_status=status, model_lanes=model_lanes, paint=paint)
  sim.tp.always_drive = on
  sim.lights = lights
  sim.raw = sign_runs(signs)
  while sim.s < STAND_AT_M:
    sim.step()
    assert sim.t < 60.
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
             {'signs': 'approach'}, {'signs': 'wide'}, {'controls': [(STAND_AT_M + 12., 'yield')]},
             {'lead_prob': 0.9, 'lead_x': GO_CLEAR_M - 5.}):
    args = {'on': True, **kw}
    _, plans = stood_off_any_hold(args.pop('on'), UNSTICK_S + 4., **args)
    assert not unstick_spans(plans), kw


def test_a_stand_that_may_be_on_the_wrong_side_never_gets_the_unstick_lift():
  """SAT_RERUN R8 (route_demo): after a right's wide exit the model stood itself in the oncoming lane, every paint read unknown,
  its left road edge 0.3-0.85 m past the car's left line; 'Nothing ahead is holding the car: going on' lifted it 10 s on and
  the model drove on there at 5-7 m/s. A stand in the road's leftmost lane with a lane on its right and no yellow read on its
  left, or with a yellow line read on its right, gets no lift (the 60 s handback as with the switch off); on its own side of a
  two-way road (the oncoming lane on its left), on a one-lane road, or leftmost with the yellow line on its left (a one-way's
  or a divided road's left lane), the lift as before: an unknown read alone bars nothing."""
  unknown = (UNKNOWN,) * 4
  cases = {'R8: leftmost, a lane on its right, paint unknown': ((0, 1), unknown, False),
           'yellow on its right': ((1, 1), (UNKNOWN, WHITE, YELLOW, UNKNOWN), False),
           'leftmost, the right line white': ((0, 1), (UNKNOWN, UNKNOWN, WHITE, UNKNOWN), False),
           'its own side of a two-way road, paint unknown': ((1, 0), unknown, True),
           'a one-lane road, paint unknown': ((0, 0), unknown, True),
           "a one-way's left lane, the yellow edge line on its left": ((0, 1), (UNKNOWN, YELLOW, WHITE, UNKNOWN), True)}
  for why, (lanes, paint, lifted) in cases.items():
    _, plans = stood_off_any_hold(True, STOPPED_HANDBACK_S + 1., model_lanes=lanes, paint=paint)
    assert bool(unstick_spans(plans)) is lifted, why
    assert plans[-1].controlPrompt == 'carStoppedTakeControl' and not any(q.shouldStop for q in plans), why
  # one frame of the stand reading the car on the wrong side is the stand's
  sim, plans = stood_off_any_hold(True, 1., model_lanes=(1, 1), paint=(UNKNOWN, WHITE, YELLOW, UNKNOWN))
  sim.paint = unknown
  assert not unstick_spans(stand(sim, UNSTICK_S + 2.))
  # a doubt coming mid-lift withdraws it
  sim, plans = stood_off_any_hold(True, UNSTICK_S + 0.2, model_lanes=(1, 0), paint=unknown)
  assert unstick_spans(plans) and sim.tp.go_cert.active
  sim.paint = (UNKNOWN, WHITE, YELLOW, UNKNOWN)
  sim.model_lanes = (1, 1)
  stand(sim, DT_MDL)
  assert not sim.tp.go_cert.active and sim.tp.go_cert.end == 'wrongSide'


def stood_at(raw, seconds=UNSTICK_S + 2., long_control=lambda s: True, at_m=STAND_AT_M):
  """Under the switch, controls ok, the camera's reader live with no head: the car driven on STRAIGHT from turnpilotd's start
  to at_m with raw(sim) the forward run of each frame (None: none came) and long_control(sim) whether openpilot controls the
  speed (False: TurnPilot's plan is a disabled one, update() never reached), then the model stands it seconds. Returns the
  stand's plans."""
  sim = Sim(STRAIGHT, controls_status='ok')
  sim.tp.always_drive = True
  sim.lights = NO_HEADS
  sim.raw = raw
  while sim.s < at_m:
    sim.long_control = long_control(sim)
    sim.step(engaged=True)
    assert sim.t < 60.
  sim.long_control = True
  return stand(sim, seconds)


@pytest.mark.parametrize('behind_m', [SIGN_CLEAR_M - 10., SIGN_CLEAR_M + 10.])
def test_the_stop_sign_memory_is_metres_driven_never_seconds_standing(behind_m):
  """unsignalized_why's stop sign (LEFT-20, section 4.5): a box last seen behind_m before the car stands bars the unstick lift
  for the whole stand when that is under SIGN_CLEAR_M (standing wears none of it: the stand here runs to the 60 s handback),
  and doesn't once the car drove SIGN_CLEAR_M past it with forward runs that had none."""
  def raw(s):
    return RawFrame([], 'road', s.t, signs=SIGN_BOX if s.s < STAND_AT_M - behind_m else ())
  plans = stood_at(raw, STOPPED_HANDBACK_S + 1.)
  assert bool(unstick_spans(plans)) == (behind_m > SIGN_CLEAR_M), behind_m
  assert plans[-1].controlPrompt == 'carStoppedTakeControl'


@pytest.mark.parametrize('behind_m', [SIGN_CLEAR_M - 10., SIGN_CLEAR_M + 10.])
def test_metres_driven_with_no_forward_run_never_clear_the_stop_sign_memory(behind_m):
  """A stretch of road the car drove with no forward run for SIGN_RUN_FRESH_S (30 m, about 2.3 s: the detector down) can have
  hidden a stop sign, so it counts as one seen there: a stand under SIGN_CLEAR_M past it gets no unstick lift, one past
  SIGN_CLEAR_M of road the runs saw does; runs stopping while the car stands (here from the stand's start) change nothing."""
  end = STAND_AT_M - behind_m
  assert 30. / Sim(STRAIGHT).cruise > SIGN_RUN_FRESH_S
  def raw(s):
    return None if end - 30. <= s.s < end or s.v < STANDSTILL_SPEED else RawFrame([], 'road', s.t)
  assert bool(unstick_spans(stood_at(raw))) == (behind_m > SIGN_CLEAR_M), behind_m


@pytest.mark.parametrize('sign', [True, False])
def test_a_stop_sign_passed_while_turnpilot_isnt_planning_is_remembered(sign):
  """The stop-sign memory follows every frame, a disabled plan's too (no longitudinal control here, so update() isn't
  reached): a box seen then, inside SIGN_CLEAR_M of the stand, still bars the unstick lift once TurnPilot plans again; with no
  box the same drive gets it."""
  def raw(s):
    return RawFrame([], 'road', s.t, signs=SIGN_BOX if sign and s.s < STAND_AT_M - 30. else ())
  plans = stood_at(raw, long_control=lambda s: s.s >= STAND_AT_M - 20.)
  assert bool(unstick_spans(plans)) != sign, sign


@pytest.mark.parametrize('at_m', [SIGN_CLEAR_M - 10., SIGN_CLEAR_M + 10.])
def test_turnpilotds_start_counts_as_a_stop_sign_just_seen(at_m):
  """turnpilotd started (a restart, say) with the car short of an unmapped stop sign it can no longer see: nothing seen of the
  road behind, so the stop-sign memory starts as a box just seen, and a stand under SIGN_CLEAR_M from the start gets no unstick
  lift; one past SIGN_CLEAR_M of road the runs saw does, on the wide camera's runs as on the road camera's."""
  for camera in ('road', 'wide'):
    plans = stood_at(sign_runs(False, camera), at_m=at_m)
    assert bool(unstick_spans(plans)) == (at_m > SIGN_CLEAR_M), (at_m, camera)


def test_the_stop_sign_memory_reads_every_forward_run_of_the_tick():
  """watch_signs reads this tick's runs from the tracker (runs_now) beside the frame's raw run: a box on the wide camera's full
  pass (key 'full') that came with a road run without one is seen; a box only on a far crop or a side view is not a forward
  run's, and the frame's metres count as seen with the road run there."""
  tp = TurnPilot()
  road = RawFrame([], 'road', 0.)
  for runs, seen in (([('full', 0., RawFrame([], 'wide', 0., signs=SIGN_BOX), None)], True),
                     ([('road', 0., RawFrame([], 'road', 0., signs=SIGN_BOX), None)], True),
                     ([('far', 0., RawFrame([], 'road', 0., 'far', signs=SIGN_BOX), None),
                       ('sideLeft', 0., RawFrame([], 'wide', 0., 'sideLeft', signs=SIGN_BOX), None)], False)):
    tp.runs_now, tp.sign_clear_m = runs, 2. * SIGN_CLEAR_M
    tp.watch_signs(road, 10.)
    assert tp.sign_clear_m == (0. if seen else 2. * SIGN_CLEAR_M + 10. * DT_MDL), runs


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


# --- WF-13: never lift into a person TurnPilot lost (SAT_VALIDATION run 6, ped_step_out at 263065a); the latch is
# scene_policy.person_front (tp/sat-ped, test_turnpilotd_person_front), these are 6cb1d93's own cases on it ---

def stood_for_a_walker_at_a_green(walker=lambda sim: {'x': 3.8, 'y': -1.4, 'vx': 0., 'vy': 0., 'inPath': True}):
  """SAT_VALIDATION run 6: a walker stepped 2 m into the lane and stood there, about 1 m right of the car's centreline; TurnPilot
  stopped the car for her ('pedestrian in path') with her last track 3.8 m ahead and 1.4 m right, in the path, age 71, the
  junction ahead a signal read green. Here the same: the car stands for her 18 m short of the green signal, she stands in front
  of it (walker(sim): her track's fields, or None once the scene has none for her). Returns the sim, stood 2 s with her."""
  sim = Sim(STRAIGHT, controls=[(290., 'signal')], cruise=6.)
  sim.tp.always_drive, sim.go_taps, sim.tp.oracle_ok = True, False, True
  sim.lights = oracle_lights('green', 290.)
  s_her = 290. - 18.
  while sim.v > 0.05 or s_her - sim.s > 10.:
    sim.step(({'cls': 'pedestrian', 'x': s_her - sim.s, 'y': -0.5, 'vx': 0., 'vy': 0., 'inPath': True},), engaged=True)
    assert sim.t < 120.
  assert sim.trace[-1][2].shouldStop and 2. < s_her - sim.s < 8.
  stood = stand(sim, 2., objects=({'cls': 'pedestrian', **walker(sim), 'age': 71},))
  assert all(p.shouldStop for p in stood) and all(str(p.goCertified.cause) == 'none' for p in stood)
  return sim


def test_a_walker_whose_track_ends_in_front_of_the_standing_car_gets_no_lift():
  """SAT_VALIDATION run 6 (ped_step_out at 263065a, CONTACT): the car stood for a walker in its path; her track coasted on
  unmatched frames (the detector on the road camera, her below its view) and ended at 12.9 s with her still 2.8 m ahead of the
  camera; with no track, the straight release went at once (go_certified turnRelease, +0.5 m/s^2), the model drove on, and the
  car passed her at 3.5 m/s 0.93 m from its centreline. WARGAME_FIXES WF-13: a person TurnPilot stood the car for whose track
  ends in front of it may still be there (scene_policy.person_front): no lift of any cause is certified (go_blocked
  'personFront') and the car is held, "Person may be in front" on the status line, until she is seen leaving, the driver takes
  over, or PERSON_FRONT_MAX_S hands back (test_turnpilotd_person_front)."""
  from openpilot.selfdrive.turnpilot.scene_policy import PERSON_FRONT_NOTE
  sim = stood_for_a_walker_at_a_green()
  with mock.patch('openpilot.selfdrive.turnpilot.scene_policy.cloudlog') as log:
    plans = stand(sim, NO_MOVE_MAX_S + 1.)  # her track gone, she still there
  assert all(str(p.goCertified.cause) == 'none' for p in plans), "never a lift past a person TurnPilot lost"
  assert all(p.shouldStop for p in plans), "held by the latch"
  assert all(p.reason == PERSON_FRONT_NOTE for p in plans)
  assert 'turnpilot.person_front' in [c.args[0] for c in log.event.call_args_list]
  assert sim.tp.scene.person_front is not None


def test_a_walker_seen_leaving_the_path_before_her_track_ends_is_no_loss():
  """WF-13's other side: a walker the scene shows out of the car's path (she stepped back to the kerb) before her track
  ends left it, so the release lifts as before; and the driver taking over ends the latch."""
  out = {'on': False}
  sim = stood_for_a_walker_at_a_green(lambda sim: {'x': 3.8, 'y': -4.5 if out['on'] else -1.4, 'vx': 0., 'vy': 0.,
                                                   'inPath': not out['on']})
  out['on'] = True
  stand(sim, 0.5, objects=({'cls': 'pedestrian', 'x': 3.8, 'y': -4.5, 'vx': 0., 'vy': 0., 'inPath': False, 'age': 76},))
  plans = stand(sim, 3.)
  assert any(str(p.goCertified.cause) == 'turnRelease' for p in plans), "seen leaving: the release lifts"
  assert sim.tp.scene.person_front is None
  sim = stood_for_a_walker_at_a_green()
  stand(sim, 1.)
  assert sim.tp.scene.person_front is not None
  sim.step(engaged=False, gas=True)
  assert sim.tp.scene.person_front is None


# --- ST-04 after a creep (SAT_VALIDATION run 2, red_right at 263065a: 90 s standing at a green) ---

def crept_and_stood_again_at_a_green(creep_s=6., creep_v=0.4):
  """SAT_VALIDATION run 2: a right at a signal, released on its green 10.8 m short of the junction; the model's phantom lead
  (13 m, p 0.87, nothing in the scene) ended every lift at once (lead); the model then crept 0.2-0.45 m/s for 6 s with turnRight
  out, 5.8 deg into the turn, 7.9 m short of the junction (go_spent), and stood again behind the same phantom. Returns the sim
  after the creep, the car at rest again."""
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_go import RIGHT_AT_A_SIGNAL
  sim = Sim(RIGHT_AT_A_SIGNAL, controls=[(290., 'signal')])
  sim.tp.always_drive = True
  sim.model_kw = {'lead_prob': 0.87, 'lead_x': 13.}
  go_from_a_red(sim, desire=Desire.turnRight)
  stand(sim, 3., lead_prob=0.87, lead_x=13.)  # the lifts end for the lead, as in the run's three
  for _ in range(round(creep_s / DT_MDL)):  # the model creeps on its own, its plan going, the lead still out
    sim.model_kw = {'lead_prob': 0.87, 'lead_x': 13.}
    sim.v = creep_v
    sim.step(engaged=True)
  assert sim.tp.go_wait_s is None, "the creep counted as the model following the release"
  return sim


def test_a_release_the_model_creeps_on_and_stalls_again_short_of_the_junction_hands_back_at_20_s_not_90():
  """SAT_VALIDATION run 2 (red_right at 263065a): the creep counted as the model following the release (its no-move clock
  ended), the go was spent short of the junction (go_spent at 7.9 m), and nothing bounded the stand after it but the signal's
  NEAR_CONTROL_STAND_S: "Stopped at the signal: take control" 90 s into it, the car standing at a green with nothing in the road.
  ALWAYS_DRIVE.md ST-04: a release the model doesn't follow hands back at NO_MOVE_MAX_S, no hold. A creep that comes back to rest
  short of the junction (the go spent) is no following: the release's no-move clock runs again from the stand, and the same
  loud no-hold handback (cantStartTakeControl) comes NO_MOVE_MAX_S into it."""
  sim = crept_and_stood_again_at_a_green()
  plans = stand(sim, NO_MOVE_MAX_S + 1., lead_prob=0.87, lead_x=13.)
  assert sim.tp.go_spent and sim.tp.follower.distance > 5.
  first = next((i for i, q in enumerate(plans) if q.controlPrompt == 'cantStartTakeControl'), None)
  assert first is not None, "handed back within NO_MOVE_MAX_S of the stand, not the signal's 90 s"
  assert abs((first + 1) * DT_MDL - NO_MOVE_MAX_S) <= 3 * DT_MDL + 1e-6, first * DT_MDL
  assert plans[first].reason == NO_MOVE_REASON and not any(q.shouldStop for q in plans), "loud, never a hold"
  # the model drives off under it: the handback clears as the release's does
  plans = [sim.step(engaged=True) for _ in range(round(4. / DT_MDL))]
  assert plans[-1].controlPrompt == 'none' and sim.v > 1.


def test_the_clock_run_again_after_a_creep_adds_no_lift():
  """The same stand with the phantom gone after the creep: the restarted no-move clock bounds the stand and nothing more. No
  release lift is certified from it (before the restart none could be: the clock had ended, and relift needs it running), so
  the handback at NO_MOVE_MAX_S is the only thing the restart brings."""
  sim = crept_and_stood_again_at_a_green()
  sim.model_kw = {}
  plans = stand(sim, NO_MOVE_MAX_S + 1.)
  assert all(go_id_of(q) is None for q in plans), "no lift from the stand after the creep"
  assert any(q.controlPrompt == 'cantStartTakeControl' for q in plans) and not any(q.shouldStop for q in plans)


def test_a_second_creep_and_stall_in_the_same_go_hands_back_at_20_s_too():
  """Per stand, not once a go: after the first creep's stand (its clock run again), the model creeps on again, which ends that
  clock as followed, and stands again short of the junction behind the same phantom. The release's clock runs again from this
  stand as well, to the same loud no-hold handback NO_MOVE_MAX_S into it, never the signal's 90 s."""
  sim = crept_and_stood_again_at_a_green()
  stand(sim, 5., lead_prob=0.87, lead_x=13.)
  for _ in range(round(3. / DT_MDL)):  # the second creep, 1.2 m, still short of the junction
    sim.model_kw = {'lead_prob': 0.87, 'lead_x': 13.}
    sim.v = 0.4
    sim.step(engaged=True)
  assert sim.tp.go_wait_s is None and sim.tp.follower.distance > 5., "the creep ended the clock again, short of the junction"
  plans = stand(sim, NO_MOVE_MAX_S + 1., lead_prob=0.87, lead_x=13.)
  first = next((i for i, q in enumerate(plans) if q.controlPrompt == 'cantStartTakeControl'), None)
  assert first is not None, "handed back within NO_MOVE_MAX_S of the second stand, not the signal's 90 s"
  assert abs((first + 1) * DT_MDL - NO_MOVE_MAX_S) <= 3 * DT_MDL + 1e-6, first * DT_MDL
  assert plans[first].reason == NO_MOVE_REASON and not any(q.shouldStop for q in plans), "loud, never a hold"
  assert all(go_id_of(q) is None for q in plans), "no lift from it either"
