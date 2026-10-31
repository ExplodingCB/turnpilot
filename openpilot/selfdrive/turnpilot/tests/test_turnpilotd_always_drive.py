"""TurnPilotAlwaysDrive (ALWAYS_DRIVE.md section 1, step S1): turnpilotd hands the switch every frame to the lane planner, the
unprotected left, the map controls, the turn skip and the stop-sign decider's Frame, which act on it from steps S2 to S5; it
does nothing in shadow mode, and a component made afresh mid-frame (a reset on a fault or a handback) has it at once."""
from unittest import mock

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.go_override import GO_CLEAR_M, GO_END_SPEED, TAKEOVER_S
from openpilot.selfdrive.turnpilot.scene.tracker import RawFrame
from openpilot.selfdrive.turnpilot.stop_go import Frame
from openpilot.selfdrive.turnpilot.tests.test_go_override import WARM_S
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import (LEFT_AT_A_RED, T_LEFT, Sim, go_from_a_red, oracle_lights, stand, stop_at,
                                                              walker_stops_the_car)
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_go import attach_planner, holding, raw_person
from openpilot.selfdrive.turnpilot.turnpilotd import (NO_MOVE_MAX_S, NO_MOVE_REASON, NO_MOVE_S, RELEASE_GOES, RELEASE_LIFT_M, STANDSTILL_SPEED,
                                                       STOPPED_NOTE, TurnPilot)


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
