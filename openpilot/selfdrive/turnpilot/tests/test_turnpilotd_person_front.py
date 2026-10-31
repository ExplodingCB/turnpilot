"""A person TurnPilot saw who may still be in front of the car (WARGAME_FIXES WF-13, FREEZE.md item 15), closed loop.

BeamNG SAT validation row 6 (ped_step_out-20261031-084546, tp/integrate 263065a, TurnPilotAlwaysDrive on, a green signal ahead):
a walker stepped into the lane and stood about 1 m right of the car's centreline. TurnPilot's hard brake (track 1, confirmed,
25 hits) stood the car short of her; her track's last hit was at 3.31 m, 1.14 m right, then it coasted on unmatched frames (age
frozen at 71) out to 3.85 m, 1.41 m right for CLOSE_HOLD_S and ended at 12.9 s with her still standing there below the bonnet
line. With no track, S2's straight release went that frame (go_certified turnRelease), the model drove on, and the car passed
her at 3.5 m/s 0.93 m off her centre: on a road, the right front corner and the mirror strike her.

Here the same timeline on the point-mass Sim: the car brakes for her, stands, she stands 3.31 m ahead and 1.14 m right with
hits for 2 s, her track coasts 3 s with no hits and ends. Nothing of TurnPilot's goes and the car stays held, whatever the
switch says; she is said to be maybe in front on the status line from that frame; seen again and seen walking out of the
corridor, the release goes 2 s after she is clear; the driver's gas is the driver's go; 30 s on it hands back loudly with the car
held. A person who walks clear of the path while tracked releases as today."""
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim, oracle_lights

STRAIGHT = [(2000., 'turn', 'left')]  # nothing to turn at for 2 km
SIGNAL_S = 290.   # a mapped signal on the straight, green all along (the sim's oracle)
WALKER_S = 270.   # where she steps out, 20 m short of it: the release is judged at the signal (STOPPED_CONTROL_M)
STEP_OUT_M = 17.07  # run 6: she stepped out 17.07 m ahead of the car at 6.48 m/s
LAST_HIT = (3.31, -1.14)  # run 6: track 1's last hit, m ahead of the camera and m left (she stood right of the centreline)
COAST_V = (0.18, -0.09)   # ...and the velocity it coasted on, unmatched, for CLOSE_HOLD_S
COAST_S = 3.0
NOTE = "Person may be in front"
REASON = "Person may be in front: take control"
PROMPT = 'personFrontTakeControl'


class Walker:
  """Run 6's walker as the tracker published her: one track, a hit on every detector run (10 Hz) while seen."""
  def __init__(self, track_id=1, age=6):
    self.track_id, self.age, self.k = track_id, age, 0

  def seen(self, x, y, vx=0., vy=0.):
    self.k += 1
    if self.k % 2 == 0:
      self.age += 1
    return self.obj(x, y, vx, vy)

  def coasting(self, x, y, vx, vy):
    return self.obj(x, y, vx, vy)  # no new hit: its age stays

  def obj(self, x, y, vx, vy):
    return ({'trackId': self.track_id, 'cls': 'pedestrian', 'x': x, 'y': y, 'vx': vx, 'vy': vy, 'age': self.age, 'conf': 0.8,
             'inPath': abs(y) < 1.5, 'camera': 'road'},)


def run6(always_drive: bool):
  """The car engaged at 6.5 m/s on a straight toward a green signal; she steps out STEP_OUT_M ahead from the right kerb and
  walks 1.5 m into the lane; TurnPilot stands the car for her; she stands at LAST_HIT seen for 2 s. Returns (sim, walker)."""
  sim = Sim(STRAIGHT, cruise=6.5, controls=[(SIGNAL_S, 'signal')])
  sim.tp.always_drive, sim.go_taps, sim.tp.oracle_ok = always_drive, False, True
  sim.lights = oracle_lights('green', SIGNAL_S)
  w = Walker()
  while WALKER_S - sim.s > STEP_OUT_M + 10.:
    sim.step(engaged=True)
    assert sim.t < 60.
  y = -2.86  # on the kerb, by the road (the curb cap's track)
  while sim.v > 0.05 or sim.t < 1.:
    if WALKER_S - sim.s <= STEP_OUT_M:
      y = min(y + 0.8 * DT_MDL, LAST_HIT[1])  # stepping in at 0.8 m/s
    sim.step(w.seen(WALKER_S - sim.s, y, vy=0.8 if y < LAST_HIT[1] else 0.), engaged=True)
    assert sim.t < 90.
  assert any(p.reason == "pedestrian in path" and p.shouldStop for _, _, p in sim.trace[-5:])
  for _ in range(round(2. / DT_MDL)):  # she stands 3.31 m ahead, 1.14 m right, seen on every run
    sim.step(w.seen(*LAST_HIT), engaged=True)
  assert sim.v < 0.05 and sim.trace[-1][2].shouldStop
  return sim, w


def lose_her(sim, w):
  """Run 6 from her last hit: her track coasts COAST_S with no hits (age frozen), drifting on its last velocity, then ends.
  Returns the plans of the coast and of the frame it ended."""
  plans = []
  for k in range(round(COAST_S / DT_MDL)):
    t = k * DT_MDL
    plans.append(sim.step(w.coasting(LAST_HIT[0] + COAST_V[0] * t, LAST_HIT[1] + COAST_V[1] * t, *COAST_V), engaged=True))
  plans.append(sim.step(engaged=True))  # gone: run 6's 12.9 s
  return plans


def certified(plans):
  return [str(p.goCertified.cause) for p in plans if str(p.goCertified.cause) != 'none']


@pytest.mark.parametrize('always_drive', [True, False])
def test_run_6_her_track_ends_in_front_of_the_standing_car_and_nothing_goes(always_drive):
  """Run 6's own timeline: at 263065a the gone frame was S2's straight release (turnRelease, the lift, the model on). Now
  nothing of TurnPilot's goes, the car stays held standing (shouldStop, a 0 speed cap) with the model free to drive, and the
  status line says she may be in front from the gone frame on. With the switch off too (no lift there: the model alone drove
  on)."""
  sim, w = run6(always_drive)
  plans = lose_her(sim, w)
  gone = plans[-1]
  assert not certified(plans), certified(plans)
  assert gone.shouldStop and gone.speedCap == 0. and str(gone.state) == 'holding', (gone.shouldStop, gone.speedCap, gone.state)
  assert gone.reason == NOTE and str(gone.controlPrompt) not in TAKE_CONTROL_PROMPTS
  held = [sim.step(engaged=True) for _ in range(round(20. / DT_MDL))]  # nothing seen of her: the model would drive on
  assert all(p.shouldStop and p.reason == NOTE and str(p.goCertified.cause) == 'none' for p in held)
  assert max(v for _, v, _ in sim.trace[-len(held) - 1:]) < 0.05, "the car moved toward her"


@pytest.mark.parametrize('always_drive', [True, False])
def test_seen_again_and_walking_out_of_the_corridor_the_release_goes_2_s_after(always_drive):
  """She comes back into view (a new track, standing where she was), then walks out of the car's path to the kerb, seen on
  every run: the latch lets go 2 s after she is clear of the corridor by its margin with nothing person-like in it, and then
  the release is today's (the switch on: turnRelease at the green; off: the model's own go)."""
  sim, w = run6(always_drive)
  lose_her(sim, w)
  for _ in range(round(1. / DT_MDL)):  # nothing seen
    sim.step(engaged=True)
  back = Walker(track_id=2, age=2)
  plans = []
  y = LAST_HIT[1]
  for _ in range(round(1. / DT_MDL)):  # seen again where she was
    plans.append(sim.step(back.seen(LAST_HIT[0], y), engaged=True))
  clear_k = None
  for _ in range(round(6. / DT_MDL)):  # walks to the kerb at 1.2 m/s and stands there, seen
    y = max(y - 1.2 * DT_MDL, -3.5)
    plans.append(sim.step(back.seen(LAST_HIT[0], y, vy=-1.2 if y > -3.5 else 0.), engaged=True))
    if clear_k is None and y <= -2.5 - 1e-9:
      clear_k = len(plans) - 1
  assert clear_k is not None
  first_free = next(k for k, p in enumerate(plans) if not p.shouldStop)
  assert all(p.shouldStop for p in plans[:first_free])
  assert (first_free - clear_k) * DT_MDL == pytest.approx(2., abs=0.11), (first_free - clear_k) * DT_MDL
  assert all(p.reason == NOTE for p in plans[:first_free])
  after = plans[first_free:]
  if always_drive:
    assert certified(after)[:1] == ['turnRelease'] and certified(plans[:first_free]) == []
  assert sim.v > 0.3, "the release: the car pulls away once she has walked clear"


def test_the_drivers_gas_is_the_drivers_go():
  """The driver presses the gas (DisengageOnAccelerator: openpilot disengages a frame later): the latch ends, TurnPilot holds
  nothing and certifies no go of its own."""
  sim, w = run6(True)
  lose_her(sim, w)
  for _ in range(round(3. / DT_MDL)):
    sim.step(engaged=True)
  plans = [sim.step(gas=True, engaged=True)] + [sim.step(gas=True, engaged=False) for _ in range(round(1. / DT_MDL))]
  assert not plans[-1].shouldStop and plans[-1].reason != NOTE
  assert not certified(plans)


def test_30_s_on_it_hands_back_loudly_with_the_car_still_held():
  """Nothing seen of her for 30 s: "Person may be in front: take control", the handback's prompt (its repeating chime in
  selfdrived), the car still held; it stays until the driver takes over."""
  sim, w = run6(True)
  lose_her(sim, w)
  plans = [sim.step(engaged=True) for _ in range(round(35. / DT_MDL))]
  first = next(k for k, p in enumerate(plans) if str(p.controlPrompt) == PROMPT)
  assert (first + 1) * DT_MDL == pytest.approx(30., abs=0.11)
  assert PROMPT in TAKE_CONTROL_PROMPTS
  assert all(p.shouldStop for p in plans)
  assert all(str(p.controlPrompt) == PROMPT and p.reason == REASON for p in plans[first:])
  assert all(str(p.controlPrompt) not in TAKE_CONTROL_PROMPTS and p.reason == NOTE for p in plans[:first])
  assert not certified(plans)


@pytest.mark.parametrize('always_drive', [True, False])
def test_a_person_who_walks_clear_of_the_path_while_tracked_releases_as_today(always_drive):
  """She walks out of the path to the kerb, seen on every run, and only then is lost: no latch, and the hold lets go the frame
  her stop ends, as at 263065a (the switch on: turnRelease at the green)."""
  sim, w = run6(always_drive)
  plans, y = [], LAST_HIT[1]
  while y > -3.5:
    y -= 1.2 * DT_MDL
    plans.append(sim.step(w.seen(LAST_HIT[0], y, vy=-1.2), engaged=True))
  plans += [sim.step(engaged=True) for _ in range(round(3. / DT_MDL))]  # lost at the kerb
  free = next(k for k, p in enumerate(plans) if not p.shouldStop)
  assert plans[free - 1].reason == "pedestrian in path" and all(p.reason != NOTE for p in plans)
  if always_drive:
    assert str(plans[free].goCertified.cause) == 'turnRelease'
  assert sim.v > 0.3


def test_with_no_person_anywhere_the_release_is_todays():
  """A red, then the green, on the same straight, nobody anywhere: the map hold lets go and the release is certified that
  frame (ALWAYS_DRIVE.md ST-12), as at 263065a."""
  sim = Sim(STRAIGHT, controls=[(SIGNAL_S, 'signal')])
  sim.tp.always_drive, sim.go_taps, sim.tp.oracle_ok = True, False, True
  light = {'state': 'red'}
  sim.lights = lambda sim: oracle_lights(light['state'], SIGNAL_S)(sim)
  while not (sim.v < 0.05 and sim.s > SIGNAL_S - 50.):
    sim.step(engaged=True)
    assert sim.t < 60.
  for _ in range(round(1. / DT_MDL)):
    sim.step(engaged=True)
  light['state'] = 'green'
  plans = []
  while not plans or plans[-1].shouldStop:
    plans.append(sim.step(engaged=True))
    assert len(plans) < 400
  assert str(plans[-1].goCertified.cause) == 'turnRelease' and all(p.reason != NOTE for p in plans)



def test_a_fault_keeps_it_and_its_hold():
  """turnpilotd failing every frame with the latch set: the faulted plan carries its hold (fault_limits, the scene kept through
  the fault), the car never let go toward her; the driver's gas still ends it there."""
  from unittest import mock
  from openpilot.selfdrive.turnpilot.turnpilotd import TurnPilot
  sim, w = run6(True)
  lose_her(sim, w)
  with mock.patch.object(TurnPilot, 'update', side_effect=RuntimeError('boom')):
    plans = [sim.step(engaged=True) for _ in range(round(2. / DT_MDL))]
    assert all(str(p.state) == 'disabled' and p.shouldStop for p in plans) and sim.v < 0.05
    plans = [sim.step(gas=True, engaged=False) for _ in range(3)]
  assert not plans[-1].shouldStop and sim.tp.scene.person_front is None

def test_every_go_of_turnpilots_is_barred_while_it_is_set():
  """With the latch set and nothing else holding the car (a bare decision: no hold, no lead, no pedal, the scene live): S2's
  release and its relifts (turnRelease), the left's go (unprotectedLeft), the stall's clearBox lift, the unstick lift and the
  stop-sign decider's go are refused, 'personFront'; a go already out is withdrawn for it."""
  from openpilot.selfdrive.turnpilot.scene_policy import SceneDecision
  from openpilot.selfdrive.turnpilot.stop_go import StopGoDecision
  sim, w = run6(True)
  lose_her(sim, w)
  tp, sm = sim.tp, sim.sm
  cs, free = sm['carState'], SceneDecision()
  assert tp.scene.person_front is not None and sm['carControl'].enabled
  assert tp.go_blocked(sm, cs, True, [], None, free, False) == 'personFront'
  for cause in ('turnRelease', 'unprotectedLeft'):
    assert tp.go_refused(sm, cs, True, [], None, free, False, 20., cause) == 'personFront', cause
  assert tp.stall_refused(sm, cs, 0., free, [], None, True, False) == 'personFront'
  tp.unstick(sm, cs, 0., free, [], None, True, False, True)
  tp.certify_stop_go(sm, cs, True, [], None, free, False, StopGoDecision(go_cause='stopSign', state='going'))
  tp.certify_go(sm, cs, True, [], None, free, False, 20.)
  assert not tp.go_cert.active
  tp.go_cert.start('turnRelease')  # one out as it is set (the lift mid-window): withdrawn on the frame
  tp.withdraw_go(sm, cs, True, True, [], None, free, False, 20.)
  assert not tp.go_cert.active and tp.go_cert.end == 'personFront'


def test_the_stop_sign_decider_reads_it_as_its_front_zone():
  """turnpilotd hands the latch to the stop-sign decider's Frame every frame (its front zone occupied: no go, H7 at 30 s)."""
  from unittest import mock
  sim, w = run6(True)
  with mock.patch.object(sim.tp.stop_go, 'update', wraps=sim.tp.stop_go.update) as update:
    lose_her(sim, w)
    sim.step(engaged=True)
  frames = [c.args[0] for c in update.call_args_list]
  assert not frames[0].person_front and frames[-1].person_front


def test_the_autonomous_left_never_goes_while_it_is_set():
  """At the autonomous left's line, a walker in the path 6 m ahead from the left's go for 3 s, then lost there: the go is
  withdrawn and no other comes (its scene_clear false: the window never counts), the car held, the note said; 30 s on, the
  loud handback. (Seen walking clear instead, test_turnpilotd_left's fresh go after the hold.)"""
  from unittest import mock
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import left_sim, run_objects
  st = {'go_t': None}

  def ped(sim):
    if st['go_t'] is None and sim.tp.left_now is not None and sim.tp.left_now.state == 'go':
      st['go_t'] = sim.t
    if st['go_t'] is not None and sim.t < st['go_t'] + 3.:
      return ({'cls': 'pedestrian', 'x': 6., 'y': 0., 'vx': 0., 'vy': 0., 'age': 40 + round(10 * (sim.t - st['go_t'])), 'trackId': 5,
               'conf': 0.8, 'inPath': True, 'camera': 'road'},)
    return ()
  sim = left_sim()
  with mock.patch.object(sim.tp.left, 'update', wraps=sim.tp.left.update) as update:
    plans = run_objects(sim, 30., ped)
    while st['go_t'] is None or sim.t < st['go_t'] + 3. + 32.:
      plans += run_objects(sim, 1., ped)
      assert sim.t < 120.
  gos = [i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftGo' and str(plans[i - 1].controlPrompt) != 'leftGo']
  lost = round((st['go_t'] + 3.) / DT_MDL)
  assert len(gos) == 1 and gos[0] < lost, gos
  assert all(v < 0.05 for _, v, _ in sim.trace[lost + 20:]) and all(p.shouldStop for p in plans[lost + 1:])
  assert all(p.reason == NOTE for p in plans[lost + 1:lost + round(29. / DT_MDL)])
  assert any(str(p.controlPrompt) == PROMPT for p in plans[lost:])
  assert not any(c.args[0].scene_clear for c in update.call_args_list[lost + 2:] if c.args[0].mode != 'off')


def seen_back_walking_out(sim, seconds=6.):
  """After the loss: 1 s of nothing, a new track where she was for 1 s, then walking out to 3.5 m right and standing there."""
  for _ in range(round(1. / DT_MDL)):
    sim.step(engaged=True)
  back, y, plans = Walker(track_id=2, age=2), LAST_HIT[1], []
  for _ in range(round(1. / DT_MDL)):
    plans.append(sim.step(back.seen(LAST_HIT[0], y), engaged=True))
  for _ in range(round(seconds / DT_MDL)):
    y = max(y - 1.2 * DT_MDL, -3.5)
    plans.append(sim.step(back.seen(LAST_HIT[0], y), engaged=True))
  return plans


@pytest.mark.parametrize('box, held', [((5., 0.3), True), (None, True), ((5., -2.4), True), ((5., -4.5), False), ((9., 0.), False)])
def test_a_raw_person_box_in_front_keeps_it_set(box, held):
  """Her walking out seen, but a person box of the detector's runs in front of the car (in the corridor or its margin, up to
  PERSON_FRONT_X_M; one that can't be ranged can't be placed, so it is in front): nothing person-like is clear, so it stays
  set and the car held. Off the corridor's margin or further out, the release goes."""
  import math
  from openpilot.selfdrive.turnpilot.scene.tracker import RawBox, RawFrame

  def raw(sim):
    if box is None:
      return RawFrame([RawBox('road', 'pedestrian', 0.4, 0., (-1., 1.), None, None, ())], 'road', sim.t)
    x, y = box
    b, r = math.degrees(math.atan2(y, x)), math.hypot(x, y)
    d = math.degrees(math.atan2(0.3, r))
    return RawFrame([RawBox('road', 'pedestrian', 0.4, b, (b - d, b + d), (x, y), r, ())], 'road', sim.t)
  sim, w = run6(True)
  lose_her(sim, w)
  sim.raw = raw
  plans = seen_back_walking_out(sim)
  assert all(p.shouldStop for p in plans) is held, [p.shouldStop for p in plans][-40:]


@pytest.mark.parametrize('always_drive', [True, False])
def test_an_unconfirmed_blip_in_front_while_held_at_a_red_holds_nothing_past_the_green(always_drive):
  """B2 (skeptic round 1, blip_at_a_red): held at a red, a 2-hit person blip 6 m ahead in the path (a reflection: conf 0.8,
  never confirmed), coasting 0.5 s and gone, then the green. Its stop never brought the car to rest (the red had), so it is no
  person TurnPilot stopped for: no latch, and the release is today's (the switch on: turnRelease). At ed9e31d the latch held the
  car through the green to the 30 s personFrontTakeControl."""
  sim = Sim(STRAIGHT, controls=[(SIGNAL_S, 'signal')])
  sim.tp.always_drive, sim.go_taps, sim.tp.oracle_ok = always_drive, False, True
  light = {'state': 'red'}
  sim.lights = lambda sim: oracle_lights(light['state'], SIGNAL_S)(sim)
  while not (sim.v < 0.05 and sim.s > SIGNAL_S - 50.):
    sim.step(engaged=True)
    assert sim.t < 60.
  for _ in range(20):
    sim.step(engaged=True)
  blip = Walker(track_id=50, age=1)
  for _ in range(4):
    sim.step(blip.seen(6., 0.2), engaged=True)
  for _ in range(round(0.5 / DT_MDL)):
    sim.step(blip.coasting(6., 0.2, 0., 0.), engaged=True)
  light['state'] = 'green'
  plans = [sim.step(engaged=True) for _ in range(round(35. / DT_MDL))]
  assert all(p.reason != NOTE and str(p.controlPrompt) != PROMPT for p in plans)
  if always_drive:
    assert certified(plans)[:1] == ['turnRelease']
  assert sim.tp.scene.person_front is None and sim.v > 0.3
