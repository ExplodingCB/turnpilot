"""turnpilotd driving the autonomous unprotected left (unprotected_left.py, DESIGN phase 4 G4) in closed loop: the synthetic
route, model and scene of test_turnpilotd, raw road camera runs of the oncoming cars, a point-mass car following the plan."""
import math

import pytest

from openpilot.cereal import log, messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.scene.tracker import RawBox, RawFrame
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M, ONCOMING_ENTRY_M
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of, nav_instruction
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
from openpilot.selfdrive.turnpilot.turn_speed import POST_GO_PROFILES
from openpilot.selfdrive.turnpilot.turnpilotd import GO_LINE_M, LEFT_NO_MOVE_S, TurnPilot, auto_left_for, left_range_for
from openpilot.selfdrive.turnpilot.unprotected_left import (CLEAR_WINDOW_S, CREEP_STALL_S, HELD_MIN_M, LIGHT_UNKNOWN_S, ROAD_RANGE_M,
                                                           STALE_UNDECIDED_S, STAND_UNDECIDED_S,
                                                           r_req, t_req)

Desire = log.Desire
V_25 = 11.18
JUNCTION = 300.
# a left at JUNCTION across a 25 mph road with one lane each way, mapped (navd: maneuverOpposingSpeed, Lanes)
STRAIGHT_M = 400.  # the opposing road's straight span from our approach (maneuverOpposingStraightM): straight past every R_req here
LEFT_25 = [(JUNCTION, "turn", "left", (0., 270., 2, 'present', V_25, 1, STRAIGHT_M)), (1500., "turn", "right", junction_of("right"))]
LINE = JUNCTION - HOLD_LINE_M - 1.  # the map's hold line (MAP_MARGIN_M short of the scene's)
# TurnPilotLeftRangeM as a drive that measured it would set it (the default is unmeasured, 0: no left is attempted): past route
# time's R_req for a 25 mph one-lane left, 200 m from LINE_D plus ROUTE_REACH_MARGIN_M (BUILD_FINDINGS D4; 150 m before it)
TEST_RANGE_M = 210.


def raw_box(x, y, conf=0.6):
  bearing = math.degrees(math.atan2(y, x))
  half = math.degrees(math.atan2(0.9, math.hypot(x, y)))
  return RawBox('road', 'car', conf, bearing, (bearing - half, bearing + half), (x, y), math.hypot(x, y) * 0.95, ())


def road_runs(cars=lambda sim: ()):
  """A road camera run every other frame (the detector's 10 Hz), of cars at absolute road positions (s, y): their x is s
  less the car's."""
  frame = [0]

  def raw(sim):
    frame[0] += 1
    if frame[0] % 2:
      return None
    boxes = [raw_box(s - sim.s, y) for s, y in cars(sim) if -5. < s - sim.s < 400.]
    return RawFrame(boxes, 'road', sim.t)
  return raw


def left_sim(cars=lambda sim: (), runs=True, sim_cls=Sim):
  sim = sim_cls(LEFT_25)
  sim.tp.auto_left, sim.tp.left_range_m = 'auto', TEST_RANGE_M
  sim.go_taps = False
  sim.answers_prompt = True  # the driver signals the left when prompted: it goes only with a lamp lit (car_signals.py)
  sim.raw = road_runs(cars) if runs else None
  return sim


def drive(sim, seconds, yaw=True):
  """Engaged, turning at an 8 m radius through the junction once the turn is under way."""
  plans = []
  for _ in range(int(round(seconds / DT_MDL))):
    turning = yaw and sim.tp.desires.started and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    plans.append(sim.step(engaged=True, yaw_rate=sim.v / 8. if turning else 0.))
  return plans


def test_an_empty_road_goes_from_the_line_with_no_pulse_before_it():
  sim = left_sim()
  plans = []
  while sim.s < JUNCTION + 30. and sim.t < 90.:
    plans += drive(sim, DT_MDL)
  assert sim.s > JUNCTION + 30.
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  # no pulse on the approach: the first turn desire is the go, from the line, standing
  assert all(p.desire == Desire.none for p in plans[:go])
  assert sim.trace[go - 1][1] < 0.3 and LINE - 1. < sim.trace[go][0] <= LINE + 0.1  # at the line, latched (under 0.3 m/s)
  assert plans[go].controlPrompt == 'leftGo' and str(plans[go].unprotectedLeft.state) == 'go'
  assert sim.tp.post_go is not None or any(p.unprotectedLeft.state == 'committed' for p in plans[go:])
  # the road camera from the line, alone from the go; held at the line it waited silently
  held = [p for p in plans[:go] if p.shouldStop and p.mapControl.kind == 'unprotectedTurn']
  assert held and all(str(p.detectorView) in ('road', 'roadAndWide') for p in held)  # the latch is read a frame late
  assert str(plans[go].detectorView) == 'road' and str(plans[go + 5].detectorView) == 'road'
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert plans[go].unprotectedLeft.rReq > 100. and abs(plans[go].unprotectedLeft.vLimit - V_25) < 1e-3
  assert plans[go].unprotectedLeft.rangeM == TEST_RANGE_M
  # committed through the oncoming lanes, and the turn taken (no missed alert)
  assert any(str(p.unprotectedLeft.state) == 'committed' for p in plans[go:])
  assert not any(p.turnMissed for p in plans)


def test_it_waits_through_an_oncoming_stream_silently_and_goes_after_it():
  # four oncoming cars at the limit 6 s apart, the first 60 m past the junction when the car nears it: the car stands at its
  # line through all of them (no take-control in the wait: no intervention) and goes only once the band is clear
  stream = [JUNCTION + 60. + k * 6. * V_25 for k in range(4)]
  t0 = [None]

  def cars(sim):
    if t0[0] is None and JUNCTION - sim.s < 60.:
      t0[0] = sim.t
    dt = sim.t - t0[0] if t0[0] is not None else 0.
    return [(s - V_25 * dt, 3.6) for s in stream]
  sim = left_sim(cars)
  plans = []
  while sim.s < JUNCTION + 30. and sim.t < 120.:
    plans += drive(sim, DT_MDL)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  t_go = sim.trace[go]
  last_passed = t0[0] + (stream[-1] - JUNCTION + HOLD_LINE_M) / V_25  # the last car past the car's line
  assert plans[go].unprotectedLeft.clearS >= CLEAR_WINDOW_S - 1e-6
  assert go * DT_MDL > last_passed
  waited = [p for p in plans[:go] if p.shouldStop]
  assert len(waited) * DT_MDL > 15. and not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert all(str(p.controlPrompt) == 'waitingForGap' for p in waited[5:])
  assert sim.trace[go - 1][1] < 0.3 and t_go[0] <= LINE + 0.1 and sim.s > JUNCTION + 30.


def standing_run(raw_at, track_at, seconds, vx=0., stream=()):
  """LEFT_25 at its line with a car standing at (m past the junction, m left) raw_at (a road run box every other frame,
  None: none) and track_at (a velocity-trusted track at vx, None: none), and oncoming stream cars (absolute s, moving at
  V_25, a box and a track each). (sim, plans)"""
  frame = [0]

  def movers(sim):
    return [(s0 - V_25 * sim.t, 3.6) for s0 in stream if -5. < s0 - V_25 * sim.t - sim.s < 400.]

  def raw(sim):
    frame[0] += 1
    if frame[0] % 2:
      return None
    boxes = [raw_box(JUNCTION + raw_at[0] - sim.s, raw_at[1])] if raw_at is not None else []
    boxes += [raw_box(s - sim.s, y) for s, y in movers(sim)]
    return RawFrame([b for b in boxes if b.xy[0] > 0.], 'road', sim.t)
  sim = left_sim()
  sim.raw = raw
  plans = []
  for _ in range(int(seconds / DT_MDL)):
    turning = sim.tp.desires.started and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    objs = [{'cls': 'car', 'x': JUNCTION + track_at[0] - sim.s, 'y': track_at[1], 'vx': vx, 'vy': 0., 'age': 60, 'trackId': 40,
             'conf': 0.8, 'camera': 'road'}] if track_at is not None else []
    objs += [{'cls': 'car', 'x': s - sim.s, 'y': y, 'vx': -V_25, 'vy': 0., 'age': 60, 'trackId': 50 + k, 'conf': 0.8, 'camera': 'road'}
             for k, (s, y) in enumerate(movers(sim))]
    plans.append(sim.step(tuple(objs), engaged=True, yaw_rate=sim.v / 8. if turning else 0.))
  return sim, plans


def test_a_car_standing_in_the_way_is_a_loud_handback_never_an_endless_silent_wait():
  """Skeptic g4c (still open after SIMFIX; the same on 49359b2 and be3c0ed): the left had no bound on its wait. A car parked
  50 m up the oncoming curb (in the band: a box and a track) or at the curb 6 m past the junction, 5 m right (in the junction
  box: a track) held the car at its line the whole 240 s run with only 'waitingForGap' (shown, never heard): no chime, no
  handback, and stopped_unheld can't fire with TurnPilot holding the car. The owner's 'loud alert if undecided': waiting
  STAND_UNDECIDED_S on what the tracker says stands is the 'standing' fault, the car held, said."""
  for raw_at, track_at in (((50., 7.5), (50., 7.5)), (None, (6., -5.))):
    sim, plans = standing_run(raw_at, track_at, 90.)
    held = next(i for i, p in enumerate(plans) if p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap')
    first = next((i for i, p in enumerate(plans) if str(p.controlPrompt) in TAKE_CONTROL_PROMPTS), None)
    assert first is not None, f"stood at the line behind a car parked at {raw_at or track_at}, no chime, no handback"
    assert str(plans[first].controlPrompt) == 'leftUndecidedTakeControl' and plans[first].unprotectedLeft.why == 'standing'
    assert plans[first].reason == "Blocked by something standing: take control"
    assert abs((first - held) * DT_MDL - STAND_UNDECIDED_S) < 1., (first - held) * DT_MDL
    assert all(str(p.controlPrompt) == 'leftUndecidedTakeControl' and p.shouldStop for p in plans[first:])
    assert not any(p.desire == Desire.turnLeft for p in plans) and sim.s < LINE + 1.


def test_moving_traffic_with_no_gap_is_never_called_standing_and_a_wait_under_45_s_is_silent():
  # a stream of oncoming cars at the limit 3 s apart for 40 s, each a box and a moving velocity-trusted track: ordinary
  # waiting, silent, never the standing fault, and a go once it has passed; a car that moves on is no longer standing
  stream = [JUNCTION + 60. + V_25 * 25. + k * 3. * V_25 for k in range(13)]
  sim, plans = standing_run(None, None, 110., stream=stream)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  waited = sum(p.shouldStop for p in plans) * DT_MDL
  assert 30. < waited < STAND_UNDECIDED_S and any(p.desire == Desire.turnLeft for p in plans) and sim.s > JUNCTION + 10.


def test_a_wait_at_the_line_with_no_gap_is_a_loud_handback_at_45_s_whatever_the_tracker_says():
  """Owner decision (ii), 2026-10-30, Kellen's delegate (DESIGN's UNDECIDED row: no gap in 45 s): 154bdaa bounded the wait
  at the line only on what the tracker says stands, so a car parked in the band seen only as raw boxes (never tracked: a
  box nothing tracks can't be told standing from coming) held the car at its line silently with no limit, and an oncoming
  stream with no gap, or a car moving in the junction box, was waited through however long. Every wait at the line for a
  gap now ends at STAND_UNDECIDED_S in the loud handback (leftUndecidedTakeControl, "No gap in oncoming traffic: take
  control"; 'standing' where the tracker says so), the car held, never let go, never a go."""
  stream = [JUNCTION + 60. + V_25 * 25. + k * 3. * V_25 for k in range(24)]
  for raw_at, track_at, vx, movers in (((50., 7.5), None, 0., ()), (None, None, 0., stream), (None, (6., -5.), 3., ())):
    sim, plans = standing_run(raw_at, track_at, 110., vx=vx, stream=movers)
    case = (raw_at, track_at, vx, len(movers))
    held = next(i for i, p in enumerate(plans) if p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap')
    first = next((i for i, p in enumerate(plans) if str(p.controlPrompt) in TAKE_CONTROL_PROMPTS), None)
    assert first is not None, f"{case}: waited at the line {(len(plans) - held) * DT_MDL:.0f} s with no gap, silently"
    assert abs((first - held) * DT_MDL - STAND_UNDECIDED_S) < 1., (case, (first - held) * DT_MDL)
    assert str(plans[first].controlPrompt) == 'leftUndecidedTakeControl' and plans[first].unprotectedLeft.why == 'noGap', case
    assert plans[first].reason == "No gap in oncoming traffic: take control", case
    assert all(str(p.controlPrompt) == 'waitingForGap' for p in plans[held + 5:first]), case
    assert all(str(p.controlPrompt) == 'leftUndecidedTakeControl' and p.shouldStop for p in plans[first:]), case
    assert not any(p.desire == Desire.turnLeft for p in plans) and sim.s < LINE + 1. and sim.v < 0.05, case


def test_no_road_camera_runs_is_a_fault_never_a_go():
  # the detector never on the road camera (an old modeld ignoring detectorView, or no road camera): never clear, and a loud
  # handback at the line once the runs are STALE_UNDECIDED_S stale
  sim = left_sim(runs=False)
  plans = drive(sim, 60.)
  held = [i for i, p in enumerate(plans) if p.shouldStop and p.mapControl.kind == 'unprotectedTurn']
  assert held and sim.s < LINE + 0.1 and not any(p.desire == Desire.turnLeft for p in plans)
  undecided = [p for p in plans if str(p.controlPrompt) == 'leftUndecidedTakeControl']
  assert undecided and undecided[0].reason == "Camera not seeing oncoming traffic: take control" and undecided[0].shouldStop
  first = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftUndecidedTakeControl')
  assert (first - held[0]) * DT_MDL <= STALE_UNDECIDED_S + 0.2


def test_a_car_in_the_band_at_first_motion_withdraws_the_go_short_of_the_oncoming_lanes():
  # the go is certified; as the car first moves, a car comes into the band (over a crest, out of a driveway): the go is
  # withdrawn while the car can stop short of the oncoming lanes, said, and the car goes again once it is clear
  state = {'car': None}

  def cars(sim):
    return [] if state['car'] is None else [(state['car'] - V_25 * (sim.t - state['t']), 3.6)]
  sim = left_sim(cars)
  plans = []
  while not any(p.desire == Desire.turnLeft for p in plans[-1:]):
    plans += drive(sim, DT_MDL)
    assert sim.t < 60.
  # the go took the left's post-go profile and its shorter no-move limit (CRITIQUE REQUIRED 4)
  assert sim.tp.post_go is not None and sim.tp.post_go.profile == POST_GO_PROFILES['left'] and sim.tp.no_move_limit == LEFT_NO_MOVE_S
  state['car'], state['t'] = sim.s + 90., sim.t  # 90 m up the oncoming lane at the go
  more = drive(sim, 1.5)
  alerts = [p for p in more if str(p.controlAlert) == 'oncomingHolding']
  assert alerts and alerts[0].reason == "Oncoming, holding"
  assert min(JUNCTION - s for s, _, _ in sim.trace[-len(more):]) > ONCOMING_ENTRY_M
  stop_s = sim.s
  plans = drive(sim, 4.)
  assert sim.s - stop_s < 0.3 and all(p.shouldStop or sim.v < 0.3 for p in plans[-20:])
  # the car passes: it goes again, a fresh pulse, and the turn is taken
  plans = drive(sim, 25.)
  assert any(p.desire == Desire.turnLeft for p in plans) and sim.s > JUNCTION + 10.


def test_committed_it_is_never_stopped_for_the_band():
  state = {'car': None}

  def cars(sim):
    return [] if state['car'] is None else [(state['car'], 3.6)]
  sim = left_sim(cars)
  while not (sim.tp.left_now is not None and sim.tp.left_now.state == 'committed'):
    drive(sim, DT_MDL)
    assert sim.t < 60.
  state['car'] = JUNCTION + 25.  # standing in the oncoming lane ahead now
  plans = drive(sim, 3.)
  assert not any(p.shouldStop for p in plans) and not any(str(p.controlAlert) == 'oncomingHolding' for p in plans)
  assert sim.s > JUNCTION


def test_the_model_holding_the_car_after_the_go_is_withdrawn_and_handed_back():
  # G0: 14 of 24 tapless goes stood. The left's go not moving LEFT_NO_MOVE_S on: turnpilotd's withdrawal and handback; the
  # left never gives it again meanwhile
  sim = left_sim()
  plans = []
  while not any(p.desire == Desire.turnLeft for p in plans[-1:]):
    plans += drive(sim, DT_MDL)
  plans = []
  for _ in range(int((LEFT_NO_MOVE_S + 1.) / DT_MDL)):
    sim.v = 0.
    plans.append(sim.step(engaged=True))
  assert plans[-1].controlPrompt == 'cantStartTakeControl' and plans[-1].shouldStop
  edges = sum(p.desire == Desire.turnLeft and q.desire != Desire.turnLeft for q, p in zip(plans, plans[1:], strict=False))
  assert edges == 0 and plans[-1].desire == Desire.none


def test_shadow_mode_still_asks_for_the_road_camera():
  # modeld reads detectorView through fresh_plan: a shadow plan's ask is harmless and keeps the logs comparable
  sim = left_sim()
  sim.shadow = True
  plans = drive(sim, 40.)
  assert any(str(p.detectorView) in ('road', 'roadAndWide') for p in plans) and all(p.shadow for p in plans)


def test_off_and_announce_hand_back_at_the_line():
  for mode, reason in (('off', "Unprotected left: take control"), ('announce', "Unprotected left: would go: take control")):
    sim = left_sim()
    sim.tp.auto_left = mode
    plans = drive(sim, 40.)
    held = [p for p in plans if str(p.controlPrompt) == 'unprotectedTurnTakeControl']
    assert held and not any(p.desire == Desire.turnLeft and sim.s > LINE for p in plans), mode
    assert any(p.reason == reason for p in held), (mode, {p.reason for p in held})


def test_a_left_past_the_proven_range_is_never_taken_from_its_line():
  # 45 mph mapped: R_req past the road camera's range: skipped at route time (below), never a go from its line
  sim = Sim([(JUNCTION, "turn", "left", (0., 270., 2, 'present', 20.1, 1, STRAIGHT_M)), (1500., "turn", "right", junction_of("right"))])
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', TEST_RANGE_M, False, road_runs()
  sim.answers_prompt = True
  assert r_req(20.1, t_req(HOLD_LINE_M + 1., 1), HOLD_LINE_M + 1.) > TEST_RANGE_M
  plans = drive(sim, 60.)
  assert not any(p.desire == Desire.turnLeft for p in plans if p.maneuverDistance > 0.)


def test_main_reads_the_auto_left_and_its_range():
  class Params:
    def __init__(self, values):
      self.values = values

    def get(self, key):
      return self.values.get(key)
  assert auto_left_for(Params({})) == 'auto'
  assert auto_left_for(Params({'TurnPilotAutoLeft': 'announce'})) == 'announce'
  assert auto_left_for(Params({'TurnPilotAutoLeft': b'off'})) == 'off'
  assert auto_left_for(Params({'TurnPilotAutoLeft': 'yes'})) == 'auto'
  assert left_range_for(Params({})) == ROAD_RANGE_M
  assert left_range_for(Params({'TurnPilotLeftRangeM': 220.})) == 220.
  assert left_range_for(Params({'TurnPilotLeftRangeM': 'x'})) == ROAD_RANGE_M
  assert left_range_for(Params({'TurnPilotLeftRangeM': 1e4})) <= 400.
  assert TurnPilot().auto_left == 'auto'


def test_a_left_the_detector_cant_see_far_enough_for_is_skipped_at_route_time():
  # the mapped opposing speed sets R_req; past the detector's proven range the left is never attempted: skipped as soon as
  # it is the route's maneuver (chimed as a skip, navd reroutes), before any lane change toward it, and driven straight on
  for junction in ((0., 270., 2, 'present', 20.1, 1, STRAIGHT_M), (0., 270., 2, 'present', 0., 0, STRAIGHT_M)):  # 45 mph, unknown (45)
    sim = Sim([(600., "turn", "left", junction), (2000., "turn", "right", junction_of("right"))])
    sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', TEST_RANGE_M, False, road_runs()
    sim.answers_prompt = True
    plans, reasons = [], set()
    while sim.s < 650.:
      plans += drive(sim, DT_MDL, yaw=False)
      reasons.add(sim.tp.skip.reason)
      assert sim.t < 120.
    first = plans[2]
    assert first.tooFastToTurn and str(first.unprotectedLeft.state) == 'skipped' and first.maneuverDistance > 500.
    assert first.unprotectedLeft.rReq > first.unprotectedLeft.rangeM
    assert not any(p.shouldStop or p.desire != Desire.none or str(p.laneChange) != 'none' for p in plans)
    assert 'unprotectedRange' in reasons
  # within range (TurnPilotLeftRangeM measured further): taken
  sim = Sim([(600., "turn", "left", (0., 270., 2, 'present', 20.1, 1, STRAIGHT_M)), (2000., "turn", "right", junction_of("right"))])
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', 350., False, road_runs()  # past route time's 337 m
  sim.answers_prompt = True
  plans = drive(sim, 5.)
  assert not any(p.tooFastToTurn for p in plans)
  # off: the line's handback, as G1's, never skipped for range
  sim = Sim([(600., "turn", "left", (0., 270., 2, 'present', 20.1, 1)), (2000., "turn", "right", junction_of("right"))])
  sim.tp.auto_left, sim.go_taps = 'off', False
  assert not any(p.tooFastToTurn for p in drive(sim, 5.))


def test_a_left_whose_opposing_road_isnt_known_straight_over_r_req_is_skipped_at_route_time():
  # skeptic G4 (residual): an oncoming car up a road curving away from the turn was outside the straight band, and the left went
  # into it (300 m radius at the limit: PET 1.70 s; a 1.3x speeder 0.44-1.68 s). The straight span navd maps from our approach
  # (maneuverOpposingStraightM) must reach R_req from the line: short of it, or not known, the left is never attempted, skipped
  # out loud at route time (unprotectedRange: "Can't See Far Enough to Turn", navd reroutes) and logged as not straight
  from unittest import mock
  for straight in (0., 60.):
    sim = Sim([(600., "turn", "left", (0., 270., 2, 'present', V_25, 1, straight)), (2000., "turn", "right", junction_of("right"))])
    sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', TEST_RANGE_M, False, road_runs()
    with mock.patch('openpilot.selfdrive.turnpilot.turnpilotd.cloudlog') as log:
      plans = []
      while sim.s < 650.:
        plans += drive(sim, DT_MDL, yaw=False)
        assert sim.t < 120.
    first = plans[2]
    assert first.tooFastToTurn and str(first.unprotectedLeft.state) == 'skipped' and first.maneuverDistance > 500., straight
    assert not any(p.shouldStop or p.desire != Desire.none for p in plans), straight
    events = [c.kwargs for c in log.event.call_args_list if c.args[0] == 'turnpilot.left_not_straight']
    assert len(events) == 1 and events[0]['straight_m'] == straight and events[0]['r_req'] > straight + HOLD_LINE_M + 1., straight
  # straight past R_req: attempted as before
  sim = Sim([(600., "turn", "left", (0., 270., 2, 'present', V_25, 1, STRAIGHT_M)), (2000., "turn", "right", junction_of("right"))])
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', TEST_RANGE_M, False, road_runs()
  assert not any(p.tooFastToTurn for p in drive(sim, 5.))


def bend_run(bend_deg, standstill_yaw=0.):
  """LEFT_25 with the road bending bend_deg (positive toward the turn) over the car's last 40 to 12 m, the car following its
  lane there (no pulse before its line: the yaw is the road's), and one oncoming car at the limit timed to be 110-130 m out
  as the car stands at its line, in the oncoming lane straight ahead of the car as it stands there. standstill_yaw: rad/s
  of yaw rate read while the car stands (a biased model or gyro). (sim, plans, the oncoming car's m from the car at the go)"""
  frame = [0]

  def onc_s(sim):
    return JUNCTION + 420. - V_25 * sim.t

  def raw(sim):
    frame[0] += 1
    if frame[0] % 2:
      return None
    s = onc_s(sim)
    return RawFrame([raw_box(s - sim.s, 3.6)] if -5. < s - sim.s < 400. else [], 'road', sim.t)
  sim = left_sim()
  sim.raw = raw
  plans = []
  while sim.t < 90.:
    d = JUNCTION - sim.s
    yaw = math.radians(bend_deg) / (28. / sim.v) if 12. < d < 40. and sim.v > 0.5 else 0.
    if sim.v < 0.05:
      yaw = standstill_yaw
    turning = sim.tp.desires.started and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    plans.append(sim.step(engaged=True, yaw_rate=sim.v / 8. if turning else yaw))
    if plans[-1].desire == Desire.turnLeft:
      break
  return sim, plans, onc_s(sim) - sim.s


def test_a_bend_before_the_junction_is_the_road_not_the_car_turning():
  # skeptic G4 B1: the road bends 6 deg away from the turn over the car's last 40 m (a 270 m radius); the car follows its lane
  # and stands aligned with it at the line, the oncoming lane straight up the road ahead of it. Counted as the car turning,
  # that yaw turned the band away from the oncoming lane and the car went with a car 74 m out (6.6 s against a T_req of
  # 8.3 s). Before the turn's desire goes out the model follows its lane, so its yaw is the road's: the band is turned only
  # by the car's turn since its first pulse (the go, for the autonomous left)
  for bend_deg in (-6., 6.):
    sim, plans, onc = bend_run(bend_deg)
    assert plans[-1].desire == Desire.turnLeft, bend_deg
    assert onc < 0. or onc > plans[-1].unprotectedLeft.rReq, f"bend {bend_deg}: went with an oncoming car {onc:.0f} m out"
    assert sim.tp.approach_yaw_deg == 0.  # nothing counted before the go


def test_a_standing_car_never_counts_yaw():
  # a model or gyro yaw-rate bias read while the car stands (minutes at its line after a withdrawn go) is no turn
  sim, plans, _ = bend_run(0., standstill_yaw=math.radians(0.5))
  assert plans[-1].desire == Desire.turnLeft and sim.tp.approach_yaw_deg == 0.
  plans = drive(sim, 0.5)  # moving on the go, turning at the sim's 8 m radius: counted
  assert sim.tp.approach_yaw_deg > 0.


class HeldSim(Sim):
  """The model holding the car at a standstill once it has stood at the left's line (G0: the tapless go stood): it never
  moves again, whatever the plan allows."""
  stood = False

  def step(self, *a, **k):
    plan = super().step(*a, **k)
    ln = self.tp.left_now
    if ln is not None and ln.state in ('waitGap', 'creep') and self.v < 0.3:
      self.stood = True
    if self.stood:
      self.s -= self.v * DT_MDL
      self.v = 0.
      self.trace[-1] = (self.s, self.v, plan)
    return plan


def test_a_creep_for_a_view_the_car_never_makes_is_never_silent():
  # skeptic G4 (blocker 4): a standing opposing left-turner blocks the view, the car is asked to creep and never moves (the
  # model holding it): 270 s in 'creep' with only the visual creepingForView. Now a loud handback, the car held
  sim = HeldSim(LEFT_25)
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps = 'auto', TEST_RANGE_M, False
  sim.answers_prompt = True

  def objects(sim):
    return ({'cls': 'car', 'x': JUNCTION + 6. - sim.s, 'y': 3.6, 'vx': 0., 'vy': 0., 'age': 40, 'trackId': 7,
             'conf': 0.8, 'camera': 'road'},)
  frame = [0]

  def raw(sim):
    frame[0] += 1
    return None if frame[0] % 2 else RawFrame([raw_box(JUNCTION + 6. - sim.s, 3.6)], 'road', sim.t)
  sim.raw = raw
  plans = [sim.step(objects(sim), engaged=True) for _ in range(int(60. / DT_MDL))]
  creep = next(i for i, p in enumerate(plans) if str(p.unprotectedLeft.state) == 'creep')
  first = next((i for i, p in enumerate(plans) if str(p.controlPrompt) in TAKE_CONTROL_PROMPTS), None)
  assert first is not None, "stood creeping for a view it never got, silently"
  assert str(plans[first].controlPrompt) == 'leftUndecidedTakeControl' and plans[first].reason == "Can't creep for a view: take control"
  assert (first - creep) * DT_MDL <= CREEP_STALL_S + 1. and plans[first].shouldStop
  assert not any(p.desire == Desire.turnLeft for p in plans)


def test_a_creep_for_a_view_the_car_makes_then_undecided_keeps_the_car_held_and_said():
  # skeptic residual-check-1 (058df13): the same blocked view with the car moving as asked (the plain Sim, where HeldSim's
  # moved_m stays 0): it crept 3.8 m past its line, stayed blocked and went undecided 'blocked', and the latch was let go
  # on the first handback frame as the driver's with no driver input (shouldStop on 0 of 384 frames, the take-control
  # gone after 104 frames, the car 164 m past the junction at 13 m/s). Held short of the oncoming lanes, said throughout
  sim = Sim(LEFT_25)
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps = 'auto', TEST_RANGE_M, False
  sim.answers_prompt = True  # the driver signals the left when prompted: it goes only with a lamp lit (LEFT_GO_NEEDS_SIGNAL, tp/blinker)

  def objects(sim):
    return ({'cls': 'car', 'x': JUNCTION + 6. - sim.s, 'y': 3.6, 'vx': 0., 'vy': 0., 'age': 40, 'trackId': 7,
             'conf': 0.8, 'camera': 'road'},)
  frame = [0]

  def raw(sim):
    frame[0] += 1
    return None if frame[0] % 2 else RawFrame([raw_box(JUNCTION + 6. - sim.s, 3.6)], 'road', sim.t)
  sim.raw = raw
  plans = [sim.step(objects(sim), engaged=True) for _ in range(int(70. / DT_MDL))]
  states = [str(p.unprotectedLeft.state) for p in plans]
  und = states.index('undecided')
  assert 'creep' in states[:und] and sim.trace[und][0] - LINE > 2.  # it crept past the latch's reach first
  after = plans[und:]
  assert all(str(p.controlPrompt) == 'leftUndecidedTakeControl' and p.shouldStop for p in after)
  assert max(s for s, _, _ in sim.trace[und:]) < JUNCTION - ONCOMING_ENTRY_M and sim.v < 0.3
  assert not any(p.desire == Desire.turnLeft for p in plans)


def jump_run(jump_m, seconds, n_cars=12, sim_cls=Sim):
  """LEFT_25 held at its line through a stream of n_cars oncoming cars 3 s apart (no gap of T_req), the first 60 m past
  the junction as the car nears it; once the car stands waiting, one navd reading puts the junction jump_m nearer (GPS
  standing). (sim, plans, s at which the last car passed the junction's conflict point)"""
  stream = [JUNCTION + 60. + k * 3. * V_25 for k in range(n_cars)]
  t0 = [None]

  def cars(sim):
    if t0[0] is None and JUNCTION - sim.s < 60.:
      t0[0] = sim.t
    dt = sim.t - t0[0] if t0[0] is not None else 0.
    return [(s - V_25 * dt, 3.6) for s in stream]
  sim = left_sim(cars, sim_cls=sim_cls)
  jumped = {'on': False, 'n': 0}

  def nav(sim, instr):
    if jumped['on'] and jumped['n'] < 1 and instr.maneuverModifier == 'left':
      jumped['n'] += 1
      return nav_instruction(instr.maneuverDistance - jump_m, "turn", "left", remaining=instr.distanceRemaining,
                             junction=LEFT_25[0][3], ahead=[(1500. - sim.s - jump_m, "turn", "right", junction_of("right"))]), True
    return instr, True
  sim.nav_filter = nav
  plans = []
  while sim.t < seconds:
    plans += drive(sim, DT_MDL)
    if not jumped['on'] and sim.tp.left_now is not None and sim.tp.left_now.state == 'waitGap' and sim.v < 0.05:
      jumped['on'] = True
  assert jumped['n'] == 1
  return sim, plans, t0[0] + (stream[-1] - JUNCTION - 3.) / V_25


def test_a_nearer_reading_while_held_at_the_line_never_silently_freezes_the_left():
  # skeptic G4 (blocker 3): one reading 5.5-10 m nearer while the car stood at its line committed the scene to the junction
  # (the follower's distance under ONCOMING_ENTRY_M), the left took that as its commit with no go, and the latch held the
  # car forever ("waiting for a gap", no chime, 130 s past the stream's end). The left keeps judging and goes after it, and
  # its go is the turn: turnLeft goes out with it (skeptic m-behaviour3 P2: at 10 m the go put out no turnLeft and the car
  # drove straight across the junction, nothing said). A reading that puts the junction behind the car standing at its line
  # (10 m: the follower at -0.75 m, the car 9.25 m short) is the position fault instead, said, the car held: never a go. So
  # is one that leaves the go's first-motion re-check no room (5.5 and 6 m: the follower under HELD_MIN_M, skeptic SIMFIX
  # P2: a car appearing at first motion was never withdrawn for); 3 and 4 m still go, turnLeft with the go
  for jump in (5.5, 6.):
    sim, plans, _ = jump_run(jump, 60.)
    first = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftUndecidedTakeControl')
    assert plans[first].unprotectedLeft.why == 'position' and plans[first].reason == "Junction position unsure: take control"
    assert all(str(p.controlPrompt) == 'leftUndecidedTakeControl' and p.shouldStop for p in plans[first:]), jump
    assert not any(str(p.controlPrompt) == 'leftGo' or p.desire == Desire.turnLeft for p in plans) and sim.s < LINE + 1., jump
  for jump in (3., 4.):
    sim, plans, last = jump_run(jump, 90.)
    states = [str(p.unprotectedLeft.state) for p in plans]
    go = next((i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftGo'), None)
    assert go is not None, (jump, sorted(set(states)))
    assert 'committed' not in states[:go] and go * DT_MDL > last, jump
    assert any(p.desire == Desire.turnLeft for p in plans[go:go + 3]), f"{jump} m nearer: the left's go put out no turnLeft"
    assert sim.s > JUNCTION + 10., jump
    assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans), jump
  sim, plans, last = jump_run(10., 90.)
  first = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftUndecidedTakeControl')
  assert plans[first].unprotectedLeft.why == 'position' and plans[first].reason == "Junction position unsure: take control"
  assert all(str(p.controlPrompt) == 'leftUndecidedTakeControl' and p.shouldStop for p in plans[first:])
  assert not any(str(p.controlPrompt) == 'leftGo' or p.desire == Desire.turnLeft for p in plans) and sim.s < LINE + 1.
  # a reading further off than the follower's spread allows (UNCERTAIN_M; the reading after it then reads as a new maneuver
  # to the follower, the latch still holding the car at the old one's line): a fault, said for as long as it holds the car,
  # never a freeze
  sim, plans, _ = jump_run(20., 60.)
  first = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftUndecidedTakeControl')
  assert plans[first].reason == "Junction position unsure: take control" and sim.s < LINE + 1.
  assert all(str(p.controlPrompt) == 'leftUndecidedTakeControl' and p.shouldStop for p in plans[first:])


def test_the_lefts_go_after_a_nearer_reading_turns_or_says_why_not():
  """Skeptic m-behaviour3 P2 (reproduced on e639725): standing at the left's line, one of navd's readings puts the left 0, 5
  or 10 m nearer (under UNCERTAIN_M of spread: no junctionUncertain), and the stream ends with a gap. Either the left's go
  puts out turnLeft, or it never goes and says why (a take-control prompt, the car held): never a go with no turn desire,
  the car driving straight across the junction from its left-turn position with nothing said (no turnMissed: check_missed
  needs DesireTiming to have taken the turn)."""
  for nearer_m in (0., 5., 10.):
    stream = [JUNCTION + 60. + k * 6. * V_25 for k in range(4)]
    t0 = [None]

    def cars(sim, stream=stream, t0=t0):
      if t0[0] is None and JUNCTION - sim.s < 60.:
        t0[0] = sim.t
      dt = sim.t - t0[0] if t0[0] is not None else 0.
      return [(s - V_25 * dt, 3.6) for s in stream]
    sim = left_sim(cars)
    jumped = {'on': False, 'n': 0}

    def nav(s, instr, nearer_m=nearer_m, jumped=jumped):
      if jumped['on'] and jumped['n'] < 1 and instr.maneuverModifier == 'left':
        jumped['n'] += 1
        return nav_instruction(instr.maneuverDistance - nearer_m, "turn", "left", remaining=instr.distanceRemaining,
                               junction=LEFT_25[0][3], ahead=[(LEFT_25[1][0] - s.s - nearer_m, "turn", "right", junction_of("right"))]), True
      return instr, True
    sim.nav_filter = nav
    while not (sim.trace and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap' and sim.v < 0.05):
      drive(sim, DT_MDL)
      assert sim.t < 90.
    jumped['on'] = True
    i0 = len(sim.trace)
    drive(sim, 40.)
    after = [p for _, _, p in sim.trace[i0:]]
    went = next((i for i, p in enumerate(after) if str(p.controlPrompt) == 'leftGo'), None)
    if went is not None:
      assert any(p.desire == Desire.turnLeft for p in after[went:went + 3]), \
        f"{nearer_m} m nearer: the left went with no turnLeft, the car {sim.s - JUNCTION:.0f} m past the junction"
    else:
      said = [p for p in after if str(p.controlPrompt) in TAKE_CONTROL_PROMPTS]
      assert said and all(p.shouldStop for p in after[after.index(said[0]):]) and sim.s < JUNCTION, nearer_m


def first_motion_run(nearer_m, v_inject):
  """LEFT_25 held at its line behind a 4-car stream 6 s apart; standing there, one navd reading puts the junction nearer_m
  nearer (under UNCERTAIN_M of spread); once the stream has passed the left goes, and at its first motion (the car at
  v_inject) a car appears 90 m up the oncoming lane at V_25. (sim, plans after the reading, the go's index or None,
  where the car appeared at: its index or None)"""
  stream = [JUNCTION + 60. + k * 6. * V_25 for k in range(4)]
  t0, state = [None], {'car': None}

  def cars(sim):
    if t0[0] is None and JUNCTION - sim.s < 60.:
      t0[0] = sim.t
    dt = sim.t - t0[0] if t0[0] is not None else 0.
    out = [(s - V_25 * dt, 3.6) for s in stream]
    if state['car'] is not None:
      out.append((state['car'] - V_25 * (sim.t - state['t']), 3.6))
    return out
  sim = left_sim(cars)
  jumped = {'on': False, 'n': 0}

  def nav(s, instr):
    if jumped['on'] and jumped['n'] < 1 and instr.maneuverModifier == 'left':
      jumped['n'] += 1
      return nav_instruction(instr.maneuverDistance - nearer_m, "turn", "left", remaining=instr.distanceRemaining,
                             junction=LEFT_25[0][3], ahead=[(LEFT_25[1][0] - s.s - nearer_m, "turn", "right", junction_of("right"))]), True
    return instr, True
  sim.nav_filter = nav
  while not (sim.trace and str(sim.trace[-1][2].unprotectedLeft.state) == 'waitGap' and sim.v < 0.05):
    drive(sim, DT_MDL)
    assert sim.t < 90.
  jumped['on'] = True
  plans, go, appeared = [], None, None
  while sim.t < 130.:
    plans += drive(sim, DT_MDL)
    if go is None and plans[-1].desire == Desire.turnLeft:
      go = len(plans) - 1
    if go is not None and appeared is None and sim.v >= v_inject:
      state['car'], state['t'], appeared = sim.s + 90., sim.t, len(plans) - 1
    if appeared is not None and len(plans) - appeared > int(4.5 / DT_MDL):
      break
  return sim, plans, go, appeared


@pytest.mark.parametrize("v_inject", [0.3, 0.8])
@pytest.mark.parametrize("nearer_m", [0., 3., 4.5, 5.5, 6., 7., 8.])
def test_a_nearer_reading_at_the_line_never_leaves_the_first_motion_recheck_without_room(nearer_m, v_inject):
  """Skeptic SIMFIX P2 (633d8a3 partial, CRITIQUE REQUIRED 4; the same on 49359b2): standing at the left's line, a reading
  5.3-9.2 m nearer left the follower 0.1-4 m out, the junction still ahead (no position fault) and the spread under
  UNCERTAIN_M. The go's first-motion re-check withdraws only while the car can stop short of the oncoming lanes by the live
  reading, false at first motion there, so a car appearing 90 m up the oncoming lane then was ignored and the car committed,
  4.3-5.8 m past the junction at 4.2 m/s 4.5 s later. A reading under HELD_MIN_M is the position fault, the car held and
  said, never a go; one that leaves the re-check room (0, 3, 4.5 m nearer) still goes, and its re-check withdraws it."""
  sim, plans, go, appeared = first_motion_run(nearer_m, v_inject)
  if JUNCTION - LINE - nearer_m < HELD_MIN_M:
    assert go is None and not any(str(p.controlPrompt) == 'leftGo' for p in plans), f"{nearer_m} m nearer: it went"
    first = next(i for i, p in enumerate(plans) if str(p.controlPrompt) in TAKE_CONTROL_PROMPTS)
    assert str(plans[first].controlPrompt) == 'leftUndecidedTakeControl' and plans[first].unprotectedLeft.why == 'position'
    assert plans[first].reason == "Junction position unsure: take control"
    assert all(str(p.controlPrompt) == 'leftUndecidedTakeControl' and p.shouldStop for p in plans[first:])
    assert sim.s < LINE + 1. and sim.v < 0.05
    return
  assert go is not None and appeared is not None, nearer_m
  more = plans[appeared:]
  assert any(str(p.controlAlert) == 'oncomingHolding' for p in more[:int(1.5 / DT_MDL)]), \
    f"{nearer_m} m nearer: a car 90 m up the oncoming lane at the go's first motion never withdrew it"
  assert max(s for s, _, _ in sim.trace[-len(more):]) < JUNCTION - ONCOMING_ENTRY_M, f"{nearer_m} m nearer: into the oncoming lanes"


def test_a_go_whose_reading_jumps_nearer_before_it_moves_is_withdrawn_and_said():
  # the same reading after the go, before the car has moved on it (the model's launch latency): the go is withdrawn where the
  # car stands, said, held, never a first motion with no room to re-check
  sim = left_sim(sim_cls=GoHeldSim)
  jumped = {'on': False, 'n': 0}

  def nav(s, instr):
    if jumped['on'] and jumped['n'] < 1 and instr.maneuverModifier == 'left':
      jumped['n'] += 1
      return nav_instruction(instr.maneuverDistance - 7., "turn", "left", remaining=instr.distanceRemaining,
                             junction=LEFT_25[0][3], ahead=[(LEFT_25[1][0] - s.s - 7., "turn", "right", junction_of("right"))]), True
    return instr, True
  sim.nav_filter = nav
  plans = []
  while not any(p.desire == Desire.turnLeft for p in plans[-1:]):
    plans += drive(sim, DT_MDL)
    assert sim.t < 60.
  jumped['on'] = True
  plans = drive(sim, 1.)
  first = next((i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftUndecidedTakeControl'), None)
  assert first is not None and plans[first].unprotectedLeft.why == 'position', sorted({str(p.unprotectedLeft.state) for p in plans})
  assert all(p.shouldStop for p in plans[first + 1:]) and not sim.tp.go_cert.active and sim.s < LINE + 1.


def test_a_nearer_reading_on_the_roll_up_to_the_line_is_said_never_a_silent_hold():
  # skeptic G4 (residual): a reading 6-8 m nearer while the car rolled up to its line at 0.9-2.2 m/s, 1.5-3 m short of it,
  # committed the moving car's scene; the left entered 'committed' with no go, and the latch held the car at the line for good
  # (121 s of 150, controlPrompt none throughout) on an empty road. Now that is the 'position' fault, out loud, the car held
  for jump in (6., 8.):
    for at_line in (1.5, 3.):
      sim = left_sim()
      st = {'on': False, 'n': 0}

      def nav(sim, instr, jump=jump, at_line=at_line, st=st):
        if not st['on'] and JUNCTION - 9. - sim.s < at_line and sim.v > 0.3:
          st['on'] = True
        if st['on'] and st['n'] < 1 and instr.maneuverModifier == 'left':
          st['n'] += 1
          return nav_instruction(instr.maneuverDistance - jump, "turn", "left", remaining=instr.distanceRemaining,
                                 junction=LEFT_25[0][3], ahead=[(1500. - sim.s - jump, "turn", "right", junction_of("right"))]), True
        return instr, True
      sim.nav_filter = nav
      plans = drive(sim, 40.)
      assert st['n'] == 1, (jump, at_line)
      states = {str(p.unprotectedLeft.state) for p in plans}
      first = next((i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftUndecidedTakeControl'), None)
      assert first is not None and 'committed' not in states and 'go' not in states, (jump, at_line, sorted(states))
      assert plans[first].unprotectedLeft.why == 'position'
      assert any(p.reason == "Junction position unsure: take control" for p in plans[first:])
      assert sim.s < JUNCTION - ONCOMING_ENTRY_M and all(str(p.controlPrompt) == 'leftUndecidedTakeControl' for p in plans[first:])


def test_runs_before_calibration_never_let_the_left_go_with_a_car_in_the_band():
  # skeptic G4 (residual): uncalibrated runs read as empty road runs, and the left went at 3.5 s with a car 60 m up the
  # oncoming lane at 0.8. Now they are no runs: the left never certifies, and its stale fault says so
  from openpilot.cereal import messaging as msgs
  from openpilot.selfdrive.turnpilot.detector.decoder import CAR
  from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import CAMS, car_state, detections
  from openpilot.selfdrive.turnpilot.scene.tracker import SceneTracker
  from openpilot.selfdrive.turnpilot.unprotected_left import AUTO, LeftInput, UnprotectedLeft
  trk = SceneTracker(CAMS, 'mici')
  empty = msgs.new_message('extrinsicsCalibration').as_reader().extrinsicsCalibration
  left, decisions = UnprotectedLeft(), []
  for i in range(int(10. / DT_MDL)):
    t = 1. + i * DT_MDL
    raw = None
    if i % 2 == 0:
      trk.update_runs([detections(trk, 'road', t, [(CAR, 60., 3.6, 0.8)], frame_id=i)], empty, car_state(), now=t)
      raw = trk.raw if trk.raw_new else None
    decisions.append(left.update(LeftInput(AUTO, 1, 1, 9., 0., True, raw, [], 0., V_25, 1, TEST_RANGE_M), DT_MDL))
  assert not any(d.go for d in decisions)
  assert decisions[-1].state == 'undecided' and decisions[-1].why == 'stale'


def test_a_detector_run_modeld_marks_invalid_never_reaches_the_tracker_or_the_left():
  # skeptic G4 (residual, part b): modeld_detector publishes valid=False when the network's output isn't finite, and decode
  # drops the NaN rows, so the run carries no boxes: fed to the tracker it reads as a fresh, empty, available road run. On
  # turnpilotd's path it never is: DetectionDrain (cd25319) takes no run from an invalid message (logged once a spell,
  # turnpilot.detector_run_invalid), so the tracker sees none, the scene goes stale and the left's stale fault comes, never a go
  from unittest import mock
  import numpy as np
  from openpilot.cereal import messaging as msgs
  from openpilot.selfdrive.turnpilot import turnpilotd as tpd
  from openpilot.selfdrive.turnpilot.detector.decoder import fill_detections
  from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import CAMS, calibration, car_state
  from openpilot.selfdrive.turnpilot.scene.tracker import SceneTracker
  from openpilot.selfdrive.turnpilot.unprotected_left import AUTO, LeftInput, UnprotectedLeft

  def invalid_run(t, i):
    msg = msgs.new_message('turnpilotDetections', valid=False)
    fill_detections(msg.turnpilotDetections, np.zeros((0, 6), dtype=np.float32), i, int(t * 1e9), 'road', 0.005)
    return msg.as_reader()
  trk, left, decisions = SceneTracker(CAMS, 'mici'), UnprotectedLeft(), []
  drain = tpd.DetectionDrain(None)
  with mock.patch.object(tpd.messaging, 'drain_sock') as drain_sock, mock.patch.object(tpd, 'cloudlog') as log:
    for i in range(int(6. / DT_MDL)):
      t = 1. + i * DT_MDL
      drain_sock.return_value = [invalid_run(t, i)] if i % 2 == 0 else []
      trk.update_runs(drain.runs(), calibration(), car_state(), now=t)
      assert not trk.raw_new and trk.raw is None and not trk.new_views
      decisions.append(left.update(LeftInput(AUTO, 1, 1, 9., 0., True, trk.raw if trk.raw_new else None, [], 0., V_25, 1,
                                             TEST_RANGE_M), DT_MDL))
  assert not trk.available and not any(d.go for d in decisions)
  assert decisions[-1].state == 'undecided' and decisions[-1].why == 'stale'
  assert sum(c.args[0] == 'turnpilot.detector_run_invalid' for c in log.event.call_args_list) == 1
  tpd.INVALID_RUNS.clear()


class GoHeldSim(Sim):
  """The model holding the car at a standstill once TurnPilot's turn pulse (its go) has gone out: it never pulls away."""
  went = False

  def step(self, *a, **k):
    plan = super().step(*a, **k)
    self.went = self.went or plan.desire == Desire.turnLeft
    if self.went:
      self.s -= self.v * DT_MDL
      self.v = 0.
      self.trace[-1] = (self.s, self.v, plan)
    return plan


def test_a_go_after_a_nearer_reading_is_still_withdrawn_when_the_car_doesnt_move():
  # the left goes only standing held at its line: after a nearer reading (the follower's distance under GO_LINE_M) its go is
  # still a go from the line, withdrawn and handed back when the model holds the car (LEFT_NO_MOVE_S), never taken for a go
  # in the junction left to the stall's handback
  sim, plans, last = jump_run(4., 70., sim_cls=GoHeldSim)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert sim.distance() < GO_LINE_M and not sim.tp.go_in_junction
  after = plans[go:]
  withdrawn = next(i for i, p in enumerate(after) if str(p.controlPrompt) == 'cantStartTakeControl')
  assert withdrawn * DT_MDL <= LEFT_NO_MOVE_S + 0.2 and after[-1].shouldStop and sim.s < LINE + 1.


def signal_sim(red_until=None, oracle=True, line=JUNCTION - 8.5):
  """LEFT_25 with a mapped signal at line (the hold line's), its light from the sim oracle (accepted): red until red_until s
  (None: always), green after; oracle False: no light source at all. An empty band."""
  sim = Sim(LEFT_25, controls=[(line, 'signal')])
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps = 'auto', TEST_RANGE_M, False
  sim.answers_prompt = True
  sim.tp.oracle_ok = oracle

  def lights(sim):
    state = 'red' if red_until is None or sim.t < red_until else 'green'
    msg = messaging.new_message('turnpilotLights')
    msg.turnpilotLights.source = 'simOracle'
    msg.turnpilotLights.lights = [{'state': state, 'distance': line - sim.s, 'instance': 'trafficLight 1'}]
    return msg.as_reader().turnpilotLights
  sim.lights = lights if oracle else None
  sim.raw = road_runs()
  return sim


def run_objects(sim, seconds, objects=lambda sim: ()):
  plans = []
  for _ in range(int(round(seconds / DT_MDL))):
    turning = sim.tp.desires.started and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    plans.append(sim.step(objects(sim), engaged=True, yaw_rate=sim.v / 8. if turning else 0.))
  return plans


def test_a_red_at_the_junction_never_gets_a_go():
  # skeptic G4 (blocker 2), DESIGN G4 go condition 5: a left on a green ball counts as unprotected, so it stands at its line
  # under the red with the band empty (the opposing traffic stopped at its own red). It sent leftGo at 31.75 s and stood in
  # 'go' through the red. Now it waits, silently, and never pulses
  sim = signal_sim()
  plans = run_objects(sim, 60.)
  assert not any(str(p.controlPrompt) == 'leftGo' or p.desire == Desire.turnLeft for p in plans)
  assert sim.s < LINE + 1. and str(plans[-1].unprotectedLeft.state) == 'waitGap'
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def test_a_red_at_a_junction_with_no_mapped_signal_never_gets_a_go():
  # skeptic G4 (residual): map_controls.turn_light returned 'none' where no signal is mapped, before reading any light, so an
  # oracle red 12 m short of the junction (OSM missing the signal; on the car the camera's reader) with the band empty (the
  # oncoming road stopped at its own red) had leftGo at 31.8 s and the car committed through the red. Now it waits at its line
  sim = Sim(LEFT_25)
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps = 'auto', TEST_RANGE_M, False
  sim.answers_prompt = True  # the driver signals the left when prompted: it goes only with a lamp lit (LEFT_GO_NEEDS_SIGNAL, tp/blinker)
  sim.tp.oracle_ok = True

  def lights(sim):
    msg = messaging.new_message('turnpilotLights')
    msg.turnpilotLights.source = 'simOracle'
    msg.turnpilotLights.lights = [{'state': 'red', 'distance': JUNCTION - 12. - sim.s, 'instance': 'trafficLight 1'}]
    return msg.as_reader().turnpilotLights
  sim.lights = lights
  sim.raw = road_runs()
  plans = run_objects(sim, 60.)
  assert not any(str(p.controlPrompt) == 'leftGo' or p.desire == Desire.turnLeft for p in plans)
  assert not any(str(p.unprotectedLeft.state) in ('go', 'committed') for p in plans)
  assert sim.s < LINE + 1. and str(plans[-1].unprotectedLeft.state) == 'waitGap'


def test_after_the_red_the_go_waits_for_a_fresh_window_and_the_exit_crosswalk():
  # skeptic G4 (blocker 2): a walker came into the exit crosswalk at 40 s, late in the red; at the green the car drove off on
  # the stale go and committed at 47.4 s with the walker still in it. Now: no go before the green, none while the walker is
  # in the crosswalk, and the go after it has left, from the line
  def walker(sim):
    if sim.t < 40. or sim.t > 40. + (9. - 2.9) / 0.3:  # in the crosswalk from 40 s, slowly, and out of it onto the corner
      return ()
    return ({'cls': 'pedestrian', 'x': JUNCTION + 4. - sim.s, 'y': 9. - 0.3 * (sim.t - 40.), 'vx': 0., 'vy': -0.3, 'age': 40,
             'trackId': 9, 'conf': 0.8, 'camera': 'road'},)
  sim = signal_sim(red_until=45.)
  plans = run_objects(sim, 75., walker)
  go = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftGo')
  left_crosswalk = 40. + (9. - 3.) / 0.3  # its y under the exit crosswalk's near edge
  assert go * DT_MDL > left_crosswalk + CLEAR_WINDOW_S - 0.2
  assert not any(str(p.unprotectedLeft.state) == 'committed' for p in plans[:go])
  assert all(s < LINE + 1. for s, _, _ in sim.trace[:go]) and sim.s > JUNCTION + 10.


def test_a_hold_after_the_go_withdraws_it_and_the_car_waits_for_a_fresh_go():
  # skeptic G4 (blocker 2): any hold after the go (a walker in the path) left a stale go that ran when the hold let go. Now
  # the go is withdrawn while it can be, the car stands, and it goes again only on a fresh window after the hold
  st = {'go_t': None}

  def ped(sim):
    if st['go_t'] is None and sim.tp.left_now is not None and sim.tp.left_now.state == 'go':
      st['go_t'] = sim.t
    if st['go_t'] is not None and sim.t < st['go_t'] + 3.:
      return ({'cls': 'pedestrian', 'x': 6., 'y': 0., 'vx': 0., 'vy': 0., 'age': 40, 'trackId': 5, 'conf': 0.8, 'inPath': True,
               'camera': 'road'},)
    return ()
  sim = left_sim()
  plans = run_objects(sim, 60., ped)
  gos = [i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftGo' and str(plans[i - 1].controlPrompt) != 'leftGo']
  assert len(gos) >= 2 and gos[1] * DT_MDL >= st['go_t'] + 3. + CLEAR_WINDOW_S - 0.2
  assert all(v < 0.3 for _, v, _ in sim.trace[gos[0] + 2:gos[1]])  # never ran on the first go
  assert sim.s > JUNCTION + 10.


def test_a_signal_whose_light_nothing_reads_is_handed_back_never_gone():
  # a left at a mapped signal with no light read (no source, or the camera's reader can't see the head): it can't tell red
  # from green, so it never goes; said out loud after LIGHT_UNKNOWN_S at its line
  sim = signal_sim(oracle=False)
  plans = run_objects(sim, 50.)
  held = next(i for i, p in enumerate(plans) if p.shouldStop and p.mapControl.kind == 'unprotectedTurn')
  und = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftUndecidedTakeControl')
  assert (und - held) * DT_MDL <= LIGHT_UNKNOWN_S + 1. and plans[und].reason == "Can't read the light: take control"
  assert not any(p.desire == Desire.turnLeft for p in plans) and sim.s < LINE + 1.


def route_time_left(range_m, controls=()):
  """LEFT_25 with TurnPilotLeftRangeM range_m and navd's controls, driven until held at the junction's hold line or past it:
  its plans."""
  sim = Sim(LEFT_25, controls=controls)
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', range_m, False, road_runs()
  sim.answers_prompt = True
  plans = []
  for _ in range(int(40. / DT_MDL)):
    plans += drive(sim, DT_MDL, yaw=False)
    if plans[-1].shouldStop or sim.s > LINE + 1.:
      break
  return sim, plans


def test_route_time_takes_the_lefts_reach_where_the_car_may_stand():
  """BUILD_FINDINGS D4 (smoke S08, diag s02_nomove_handback): route time checked R_req at LINE_D with no margin (134 m at 25
  mph, one lane, under a 150 m range), and the live check at the line, from where the car stands and the furthest reading
  (S08: d_far 23.6 m, 204 m), ended the left undecided there: a takeover at any left with a set-back line or about 2 m of
  spread. With no mapped stop line route time now takes LINE_D plus ROUTE_REACH_MARGIN_M (UNCERTAIN_M, a spread just short of
  the junctionUncertain alert): 200 m, so at 150 m the left is skipped before its junction out loud (unprotectedRange) and
  rerouted, its plan's rReq the reach it was skipped for; with a range covering that it is attempted as before. The live
  check at the line is unchanged."""
  from openpilot.selfdrive.turnpilot.route_follower import UNCERTAIN_M
  from openpilot.selfdrive.turnpilot.unprotected_left import ROUTE_REACH_MARGIN_M
  from openpilot.selfdrive.turnpilot.unprotected_left import LINE_D, reach_at_line
  assert ROUTE_REACH_MARGIN_M == UNCERTAIN_M
  route_reach = reach_at_line(V_25, 1, ROUTE_REACH_MARGIN_M)
  assert reach_at_line(V_25, 1) < 150. < route_reach < TEST_RANGE_M, (reach_at_line(V_25, 1), route_reach)
  assert math.isclose(route_reach, r_req(V_25, t_req(LINE_D + UNCERTAIN_M, 1), LINE_D + UNCERTAIN_M))
  sim, plans = route_time_left(150.)
  skipped = [p for p in plans if str(p.unprotectedLeft.state) == 'skipped']
  assert skipped and all(p.tooFastToTurn and str(p.skipReason) == 'unprotectedRange' for p in skipped)
  assert skipped[0].maneuverDistance > 100., "skipped as soon as it is the maneuver, well before its junction"
  assert abs(skipped[0].unprotectedLeft.rReq - route_reach) < 0.1 and skipped[0].unprotectedLeft.rangeM == 150.
  assert not any(p.shouldStop or p.desire != Desire.none for p in plans)
  sim, plans = route_time_left(TEST_RANGE_M)
  assert not any(p.tooFastToTurn for p in plans)
  assert any(p.shouldStop and p.mapControl.kind == 'unprotectedTurn' for p in plans), "attempted: held at its line"


def test_route_time_takes_a_mapped_stop_lines_setback_in_place_of_the_margin():
  # a mapped stop line at the junction (a signal's, its line 10 m before it: 1 m back of the hold line) is where the car may
  # stand: route time takes R_req there, 138 m, under a 150 m range, so the left isn't skipped for range. One mapped 20 m
  # back (11 m) takes 182 m: skipped
  from openpilot.selfdrive.turnpilot.unprotected_left import reach_at_line
  for line_m, skip in ((10., False), (20., True)):
    sim, plans = route_time_left(150., controls=[(JUNCTION - line_m, 'signal', True)])
    assert any(p.tooFastToTurn and str(p.skipReason) == 'unprotectedRange' for p in plans) == skip, line_m
    if skip:
      rreq = next(p.unprotectedLeft.rReq for p in plans if str(p.unprotectedLeft.state) == 'skipped')
      assert abs(rreq - reach_at_line(V_25, 1, line_m - HOLD_LINE_M - 1.)) < 0.5, (line_m, rreq)


def test_an_unmeasured_detector_range_attempts_no_left_blind():
  # skeptic G4 (blocker 6): TurnPilotLeftRangeM defaulted to 150 m, pixel arithmetic never measured on real comma frames
  # (CRITIQUE SEV2: even a 25 mph R_req of 134-141 m is past the uncropped detector at 6.5-8 px), so 25 mph lefts were
  # attempted on an unmeasured range. Unset, the range is unmeasured (0): every autonomous left is past it, skipped at route
  # time out loud and rerouted, never taken from its line. A drive sets it from a measurement (TEST_RANGE_M here)
  assert ROAD_RANGE_M == 0.
  sim = Sim(LEFT_25)
  assert sim.tp.left_range_m == ROAD_RANGE_M
  sim.tp.auto_left, sim.go_taps, sim.raw = 'auto', False, road_runs()
  sim.answers_prompt = True
  plans, reasons = [], set()
  for _ in range(int(40. / DT_MDL)):
    plans += drive(sim, DT_MDL, yaw=False)
    reasons.add(sim.tp.skip.reason)
  assert any(p.tooFastToTurn for p in plans) and 'unprotectedRange' in reasons
  assert not any(p.shouldStop or p.desire != Desire.none for p in plans)
  assert all(str(p.unprotectedLeft.state) == 'skipped' for p in plans[2:] if p.maneuverDistance > 0. and str(p.maneuverModifier) == 'left')


# DESIGN phase 4 G5 (this run's G6): the far crop's runs, between the full pass's
V_45 = 20.1
LEFT_45 = [(JUNCTION, "turn", "left", (0., 270., 2, 'present', V_45, 1, STRAIGHT_M)), (1500., "turn", "right", junction_of("right"))]
# TurnPilotLeftFarRangeM as a drive that measured it would set it (the default is unmeasured, 0): past route time's R_req for a
# 45 mph one-lane left, 337 m from LINE_D plus ROUTE_REACH_MARGIN_M (BUILD_FINDINGS D4; 300 m before it)
FAR_RANGE_M = 350.


def far_runs(cars=lambda sim: (), cover=(-15.6, 15.6)):
  """A far crop run on the frames between road_runs' (modeld), of cars at absolute road positions (s, y), covering cover."""
  import dataclasses
  frame = [0]

  def raw(sim):
    frame[0] += 1
    if not frame[0] % 2:
      return None
    boxes = [dataclasses.replace(raw_box(s - sim.s, y), view='far') for s, y in cars(sim) if -5. < s - sim.s < 800.]
    return RawFrame(boxes, 'road', sim.t, 'far', cover)
  return raw


def far_sim(cars=lambda sim: (), status='read', far_range=FAR_RANGE_M):
  sim = Sim(LEFT_45)
  sim.tp.auto_left, sim.tp.left_range_m, sim.tp.left_far_range_m = 'auto', TEST_RANGE_M, far_range
  sim.go_taps = False
  sim.answers_prompt = True  # the driver signals the left when prompted: it goes only with a lamp lit (car_signals.py)
  sim.raw, sim.raw_far = road_runs(cars), far_runs(cars)
  sim.far_status = status if callable(status) else (lambda sim: status)
  return sim


def test_a_45_mph_left_is_taken_on_the_far_crops_runs():
  from openpilot.selfdrive.turnpilot.unprotected_left import far_aim
  sim = far_sim()
  plans = []
  while sim.s < JUNCTION + 30. and sim.t < 90.:
    plans += drive(sim, DT_MDL)
  assert sim.s > JUNCTION + 30. and not any(p.tooFastToTurn for p in plans)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  ul = plans[go].unprotectedLeft
  assert str(ul.state) == 'go' and ul.rReq > TEST_RANGE_M and ul.farNeeded and ul.farRangeM == FAR_RANGE_M
  assert sim.trace[go - 1][1] < 0.3 and LINE - 1. < sim.trace[go][0] <= LINE + 0.1
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  # the far crop aimed up the oncoming approach while the left is in hand, straight ahead once it isn't
  assert abs(plans[go - 1].farCropBearing - far_aim(0., 1, TEST_RANGE_M)) < 1e-4 and plans[go - 1].farCropBearing > 1.
  assert plans[0].farCropBearing != 0. and drive(sim, 1.)[-1].farCropBearing == 0.


def test_a_car_far_up_the_oncoming_road_seen_only_by_the_far_crop_holds_it():
  # one oncoming car at the limit, seen only by the far crop (the full pass never sees it): the car waits for it
  s0 = [None]

  def cars(sim):
    if s0[0] is None and JUNCTION - sim.s < 40.:
      s0[0] = sim.t
    dt = sim.t - s0[0] if s0[0] is not None else 0.
    return [(JUNCTION + 260. - V_45 * dt, 3.6)]
  sim = far_sim()
  sim.raw = road_runs()  # the full pass sees nothing
  sim.raw_far = far_runs(cars)
  plans = []
  while sim.s < JUNCTION + 30. and sim.t < 120.:
    plans += drive(sim, DT_MDL)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  passed = s0[0] + (260. + HOLD_LINE_M) / V_45  # the car past the line
  assert go * DT_MDL > passed and any(p.unprotectedLeft.farBlockers for p in plans[:go])


def test_without_the_far_crop_a_45_mph_left_is_skipped_and_its_loss_at_the_line_is_loud():
  # the far crop off (no pkl, the param, off for the drive) or its range unmeasured: skipped at route time and rerouted
  for status, far_range in (('off', FAR_RANGE_M), ('disabled', FAR_RANGE_M), ('read', 0.)):
    sim = far_sim(status=status, far_range=far_range)
    plans = drive(sim, 3.)
    assert plans[-1].tooFastToTurn and str(plans[-1].unprotectedLeft.state) == 'skipped', status
    assert plans[-1].unprotectedLeft.farRangeM == (far_range if status == 'read' else 0.) and plans[-1].unprotectedLeft.farNeeded
  # the full pass's range unmeasured: the far crop extends nothing (it covers the band only past that range)
  sim = far_sim()
  sim.tp.left_range_m = 0.
  assert drive(sim, 3.)[-1].tooFastToTurn
  # on but waiting for room or calibration at route time (a spell): not skipped, judged at the line
  for status in ('noRoom', 'noCalibration'):
    sim = far_sim(status=status)
    assert not any(p.tooFastToTurn for p in drive(sim, 3.)), status
  # it goes off for the drive while the car waits at its line: said once (farViewOff), and the left hands back (farOff)
  stream = [JUNCTION + 200. + k * 30. for k in range(40)]
  off_at = [None]

  def status(sim):
    return 'disabled' if off_at[0] is not None and sim.t >= off_at[0] else 'read'
  sim = far_sim(cars=lambda sim: [(s, 3.6) for s in stream], status=status)
  plans = []
  while sim.t < 60.:
    plans += drive(sim, DT_MDL)
    if off_at[0] is None and plans[-1].shouldStop and sim.v < 0.1 and str(plans[-1].unprotectedLeft.state) == 'waitGap':
      off_at[0] = sim.t + 2.
  assert off_at[0] is not None
  after = [p for p, (_, _, _) in zip(plans, sim.trace, strict=True) if p is not None][-int(round((sim.t - off_at[0]) / DT_MDL)):]
  assert any(str(p.controlAlert) == 'farViewOff' for p in after[:3])
  assert sum(str(p.controlAlert) == 'farViewOff' for p in plans) * DT_MDL <= 2.1  # said once
  # held at the line and handed back, never skipped and rerouted from it
  assert all(str(p.unprotectedLeft.state) == 'undecided' and p.unprotectedLeft.why == 'farOff' for p in after[2:])
  assert all(str(p.controlPrompt) == 'leftUndecidedTakeControl' and p.shouldStop and not p.tooFastToTurn for p in after[2:])
  assert not any(p.desire == Desire.turnLeft for p in plans) and sim.s < LINE + 0.1


def test_a_far_crop_spell_without_room_at_the_line_is_waited_through():
  # the far crop on but without room for 0.6 s (farStatus noRoom, no far runs: a slow spell of the model's) while the car
  # waits at its line: the window starts over and the car waits on, never a handback for a transient; it goes after
  spell = [None]
  runs = far_runs()

  def status(sim):
    return 'noRoom' if spell[0] is not None and spell[0] <= sim.t < spell[0] + 0.6 else 'read'

  def raw_far(sim):
    run = runs(sim)
    return None if status(sim) == 'noRoom' else run
  stream = [JUNCTION + 120. + k * 30. for k in range(3)]  # standing cars ahead in the band: it waits until they go
  sim = far_sim(cars=lambda sim: [(s, 3.6) for s in stream] if spell[0] is None or sim.t < spell[0] + 1. else [],
                status=status)
  sim.raw_far = raw_far
  sim.raw = road_runs(lambda sim: [(s, 3.6) for s in stream] if spell[0] is None or sim.t < spell[0] + 1. else [])
  plans = []
  while sim.s < JUNCTION + 30. and sim.t < 90.:
    plans += drive(sim, DT_MDL)
    if spell[0] is None and plans[-1].shouldStop and sim.v < 0.1 and str(plans[-1].unprotectedLeft.state) == 'waitGap':
      spell[0] = sim.t + 1.
  assert spell[0] is not None
  assert not any(str(p.unprotectedLeft.state) == 'undecided' or str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert not any(str(p.controlAlert) == 'farViewOff' for p in plans)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert go * DT_MDL > spell[0] + 0.6 + CLEAR_WINDOW_S - 0.1 and sim.s > JUNCTION + 30.


def test_main_reads_the_far_range():
  from openpilot.common.params import UnknownKeyName
  from openpilot.selfdrive.turnpilot.turnpilotd import left_far_range_for

  class Params:
    def __init__(self, values, unknown=False):
      self.values, self.unknown = values, unknown

    def get(self, key):
      if self.unknown:
        raise UnknownKeyName(key)
      return self.values.get(key)
  assert left_far_range_for(Params({})) == 0. and TurnPilot().left_far_range_m == 0.
  assert left_far_range_for(Params({'TurnPilotLeftFarRangeM': 280.})) == 280.
  assert left_far_range_for(Params({'TurnPilotLeftFarRangeM': b'bad'})) == 0.
  assert left_far_range_for(Params({'TurnPilotLeftFarRangeM': 1e4})) <= 400.
  assert left_far_range_for(Params({}, unknown=True)) == 0.


def test_a_car_standing_far_up_the_road_is_a_loud_handback_never_an_endless_wait():
  # skeptic G6: a car parked on the oncoming side 280 m up the road, in the band, found only by the far crop (which blocks
  # at any range): the 45 mph left stood at its line for good, silently (waitingForGap). Now, waiting on the far crop's boxes
  # alone FAR_BLOCKED_UNDECIDED_S, it is undecided out loud (farBlocked, the take-control prompt), held at its line within
  # -2.0, never a go
  from openpilot.selfdrive.turnpilot.unprotected_left import FAR_BLOCKED_UNDECIDED_S
  sim = far_sim()
  sim.raw, sim.raw_far = road_runs(), far_runs(lambda sim: [(JUNCTION + 280., 5.4)])
  plans = []
  while sim.t < 40. + FAR_BLOCKED_UNDECIDED_S:  # ~29 s to stand at the line
    plans += drive(sim, DT_MDL)
  states = [str(p.unprotectedLeft.state) for p in plans]
  stand = states.index('waitGap')
  first_und = states.index('undecided')
  assert FAR_BLOCKED_UNDECIDED_S - 0.1 <= (first_und - stand) * DT_MDL <= FAR_BLOCKED_UNDECIDED_S + 0.5
  assert all(p.controlPrompt == 'waitingForGap' for p in plans[stand:first_und])
  und = plans[first_und:]
  assert all(str(p.unprotectedLeft.state) == 'undecided' and p.unprotectedLeft.why == 'farBlocked' for p in und)
  assert all(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS and p.shouldStop for p in und)
  assert not any(p.desire == Desire.turnLeft for p in plans) and sim.s <= LINE + 0.1 and sim.v < 0.1
  assert min(p.accelCap for p in plans) >= -2.0 - 1e-6


def gas_crawl_at_the_line(crawl_m, v_crawl=0.7, tracks=False):
  """LEFT_25 held at its line waiting through an oncoming stream with no gap (cars 3 s apart); the driver presses the gas
  (openpilot disengaged), crawls crawl_m at v_crawl, lifts and re-engages rolling, the model driving straight on. tracks: the
  stream's cars tracked too (velocity trusted, closing), so the scene's own oncoming hold stands at the junction. (sim, the
  plans after the lift, m past the line at the lift)"""
  stream = [JUNCTION + 60. + k * 3. * V_25 for k in range(40)]
  t0 = [None]

  def cars(sim):
    if t0[0] is None and JUNCTION - sim.s < 60.:
      t0[0] = sim.t
    dt = sim.t - t0[0] if t0[0] is not None else 0.
    return [(s - V_25 * dt, 3.6) for s in stream]

  def objs():
    if not tracks:
      return ()
    return tuple({'cls': 'car', 'x': s - sim.s, 'y': y, 'vx': -V_25, 'vy': 0., 'age': 60, 'trackId': 50 + k, 'conf': 0.8,
                  'camera': 'road'} for k, (s, y) in enumerate(cars(sim)) if -5. < s - sim.s < 150.)
  sim = left_sim(cars)
  while not (sim.tp.left_now is not None and sim.tp.left_now.state == 'waitGap' and sim.v < 0.05):
    sim.step(objs(), engaged=True)
    assert sim.t < 60.
  for _ in range(int(3. / DT_MDL)):
    sim.step(objs(), engaged=True)
  s0 = sim.s
  sim.accel = lambda sim, plan: (0.5 if sim.v < v_crawl else 0.)
  while sim.s < s0 + crawl_m:
    sim.step(objs(), engaged=False, gas=True)
  past = sim.s - LINE
  sim.accel = None
  return sim, [sim.step(objs(), engaged=True) for _ in range(int(20. / DT_MDL))], past


def test_a_gas_crawl_into_the_left_is_said_through_the_junction_and_a_straight_drive_is_missed():
  """Skeptic SG-residual-main (owner decision (b)): held at the left's line waiting for a gap, a 2.3 m gas crawl and a re-engage
  drove straight through the junction; take-control showed about 4.5 s, then nothing, and turnMissed was never set. Now
  the crossing is said (unprotectedTurnTakeControl, "Unprotected left not checked") until the car is DRIVEN_THROUGH_M past
  the line, the re-engage hearing it; TurnPilot never pulses the left it didn't check; and driven straight past the
  junction the left is missed: turnMissed ("Turn Missed", "Rerouting")."""
  from openpilot.selfdrive.turnpilot.map_controls import DRIVEN_THROUGH_M
  sim, after, past = gas_crawl_at_the_line(2.3)
  rows = list(zip(sim.trace[-len(after):], after, strict=True))
  inside = [p for (s, _, _), p in rows if LINE + past + 0.5 < s < LINE + DRIVEN_THROUGH_M - 1.]
  assert inside and all(str(p.controlPrompt) == 'unprotectedTurnTakeControl' and p.reason == "Unprotected left not checked: take control"
                        for p in inside), sorted({(str(p.controlPrompt), p.reason) for p in inside})
  assert not any(p.desire == Desire.turnLeft for p in after), "TurnPilot pulsed the left it never checked"
  missed = [k for k, p in enumerate(after) if p.turnMissed]
  assert missed and sim.trace[-len(after) + missed[0]][0] > JUNCTION, "driven straight through the left, never missed"
  assert sim.s > JUNCTION + 30. and not any(p.shouldStop for p in after[5:])


def test_a_left_whose_line_the_driver_crossed_is_never_pulsed_when_the_scene_was_released_into_the_junction():
  """Found fixing SIMFIX round 2 (e93750b): the same 2.3 m gas crawl past the left's line, the stream's cars tracked, so
  the scene's oncoming hold stood at the junction and the driver's gas released it (scene_policy.released_id): the left took
  that for the driver's own release of the junction and committed, and TurnPilot pulsed the turn (turnLeft) on the
  re-engage, a left nobody checked, toward a stream 3 s apart. The latch let go as the driver's (map_controls.Driven) ends
  the left (undecided, 'driven'): never a pulse, the crossing said, the left driven straight past missed."""
  from openpilot.selfdrive.turnpilot.map_controls import DRIVEN_THROUGH_M
  sim, after, past = gas_crawl_at_the_line(2.3, tracks=True)
  assert sim.tp.scene.released_id is not None  # the scene's hold was the driver's to release
  assert not any(p.desire == Desire.turnLeft for p in after), "TurnPilot pulsed the left it never checked"
  assert all(str(p.unprotectedLeft.state) in ('undecided', 'off') for p in after)
  rows = list(zip(sim.trace[-len(after):], after, strict=True))
  inside = [p for (s, _, _), p in rows if LINE + past + 0.5 < s < LINE + DRIVEN_THROUGH_M - 1.]
  assert inside and all(str(p.controlPrompt) == 'unprotectedTurnTakeControl' for p in inside)
  assert any(p.turnMissed for p in after)


def test_a_gas_nudge_short_of_the_lefts_reach_stays_held_for_the_full_check():
  # owner decision (a), (c): a nudge under the latch's reach past the line (LATCH_PAST_M) lets nothing go; re-engaged, the car
  # is held where it stands, the left still waiting for its gap, never driven on unchecked
  sim, after, past = gas_crawl_at_the_line(1.5)
  assert past < 2.
  assert all(p.shouldStop for p in after[20:]) and sim.v < 0.05 and sim.s < LINE + 2.5
  assert {str(p.unprotectedLeft.state) for p in after[20:]} == {'waitGap'}
  assert not any(p.desire == Desire.turnLeft or str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in after)


def withdrawn_go():
  """LEFT_25 at its line through four oncoming cars 6 s apart; the go certified, and at its first motion a car 90 m up the
  oncoming lane at the limit withdraws it (skeptic SIMFIX round 2): the car stands on the left's own hold
  (unprotected_left: hold_here) just past its line, the hold line's latch already let go by the go (cause gap)."""
  stream = [JUNCTION + 60. + k * 6. * V_25 for k in range(4)]
  t0, late = [None], {}

  def cars(sim):
    if t0[0] is None and JUNCTION - sim.s < 60.:
      t0[0] = sim.t
    dt = sim.t - t0[0] if t0[0] is not None else 0.
    out = [(s - V_25 * dt, 3.6) for s in stream]
    if late:
      out.append((late['s'] - V_25 * (sim.t - late['t']), 3.6))
    return out
  sim = left_sim(cars)
  while not (sim.tp.left_now is not None and sim.tp.left_now.state == 'go'):
    drive(sim, DT_MDL)
    assert sim.t < 120.
  while sim.v < 0.3:
    drive(sim, DT_MDL)
  late['s'], late['t'] = sim.s + 90., sim.t
  drive(sim, 3.)
  assert sim.tp.left_now.state == 'waitGap' and sim.tp.left.hold_here and sim.v < 0.05 and LINE - 0.1 < sim.s < LINE + 1.
  return sim


def crossing_said(rows, upto):
  """The plans of rows (sim.trace's) short of upto m: all the driver's crossing's take-control (map_controls.Driven)."""
  inside = [p for s, _, p in rows if s < upto]
  return inside and all(str(p.controlPrompt) == 'unprotectedTurnTakeControl' and p.reason == "Unprotected left not checked: take control"
                        for p in inside)


@pytest.mark.parametrize("rolling", [True, False])
@pytest.mark.parametrize("crawl_to", [5., 3., 1.])
def test_after_a_withdrawn_go_the_drivers_crawl_on_is_theirs_never_held_in_the_oncoming_lanes(crawl_to, rolling):
  """Skeptic SIMFIX round 2 (still at e93750b): the go let the latch go (cause gap), and the left's own hold after its
  withdrawal (hold_here) stood the car wherever the comfortable curve stopped it, with nothing for the driver's metres: a
  crawl under the gas (disengaged) to 3 m or 1 m short of the junction centre and a re-engage held the car standing 2.25-2.48
  and 0.25-0.48 m from it, inside ONCOMING_ENTRY_M, a car closing. Owner decisions (a), (b): the car past its hold by the
  driver's own movement is the driver's: never a hold in the oncoming lanes, the crossing said (unprotectedTurnTakeControl,
  "Unprotected left not checked") until the car is DRIVEN_THROUGH_M past the line, a re-engage hearing it, never a pulse of
  the left nobody checked, and the left driven straight on missed (turnMissed). Re-engaged rolling, or standing."""
  from openpilot.selfdrive.turnpilot.map_controls import DRIVEN_THROUGH_M
  sim = withdrawn_go()
  sim.accel = lambda sim, plan: 0.5 if sim.v < 1. else 0.
  while JUNCTION - sim.s > crawl_to:
    sim.step(engaged=False, gas=True)
  if not rolling:
    sim.accel = lambda sim, plan: -1. if sim.v > 0. else 0.
    while sim.v > 0.:
      sim.step(engaged=False)
  sim.accel = None
  n = len(sim.trace)
  after = drive(sim, 15., yaw=False)
  rows = sim.trace[n:]
  in_lanes = [(round(JUNCTION - s, 2), round(v, 2)) for s, v, _ in rows if abs(JUNCTION - s) < ONCOMING_ENTRY_M and v < 0.05]
  assert not in_lanes, f"held standing in the oncoming lanes on the re-engage: {in_lanes[:3]}"
  assert not any(p.shouldStop for p in after)
  assert crossing_said(rows, LINE + DRIVEN_THROUGH_M - 1.), sorted({(str(p.controlPrompt), p.reason) for _, _, p in rows})
  assert not any(p.desire == Desire.turnLeft for p in after), "TurnPilot pulsed the left it never checked"
  missed = [s for s, _, p in rows if p.turnMissed]
  assert missed and missed[0] > JUNCTION, "driven straight through the left, never missed"


@pytest.mark.parametrize("after_m", [3., 6., 10.])
def test_after_a_withdrawn_go_the_drivers_own_left_is_never_held_in_the_junction_on_a_re_engage(after_m):
  """Skeptic SIMFIX round 2: the driver takes the left under the gas (disengaged, yawing) after the go was withdrawn and
  re-engages 7-58 deg round: the left's own hold stood the car in the junction at 3.62, -1.31 and -6.88 m from its centre
  (about -1.4 m/s^2), reason 'Oncoming, holding' from 3 s earlier. The driver's, said, never held; the left they turned is
  no missed turn."""
  from openpilot.selfdrive.turnpilot.map_controls import DRIVEN_THROUGH_M
  sim = withdrawn_go()
  s0, yawed = sim.s, [0.]

  def step(turn, **kw):
    yr = sim.v / 8. if turn else 0.
    yawed[0] += math.degrees(yr * DT_MDL)
    return sim.step(yaw_rate=yr, **kw)
  sim.accel = lambda sim, plan: 1. if sim.v < 4. else 0.
  while sim.s - s0 < after_m:
    step(JUNCTION - sim.s < 7., engaged=False, gas=True)
  sim.accel = None
  n = len(sim.trace)
  after = [step(yawed[0] < 90., engaged=True) for _ in range(int(8. / DT_MDL))]
  rows = sim.trace[n:]
  stood = [(round(JUNCTION - s, 2), round(v, 2), p.reason) for s, v, p in rows if v < 0.05]
  assert not stood, f"re-engaged {yawed[0]:.0f} deg round the driver's own left: held standing in the junction {stood[:2]}"
  assert not any(p.shouldStop for p in after)
  assert crossing_said(rows, LINE + DRIVEN_THROUGH_M - 1.), sorted({(str(p.controlPrompt), p.reason) for _, _, p in rows})
  assert not any(p.desire == Desire.turnLeft for p in after) and not any(p.turnMissed for p in after)


@pytest.mark.parametrize("reengage_at", [-2., -8., -25.])
def test_after_a_withdrawn_go_a_left_the_driver_drives_straight_through_is_said_and_missed(reengage_at):
  """Skeptic SIMFIX round 2: driven straight on through the junction under the gas after the withdrawn go and re-engaged 2 m
  or 8 m past its centre, the car was held standing 7.89 and 13.97 m past it and turnMissed was never set. The driver's:
  never held there, the crossing said until DRIVEN_THROUGH_M past the line, and the left missed (turnMissed, navd's reroute)."""
  from openpilot.selfdrive.turnpilot.map_controls import DRIVEN_THROUGH_M
  sim = withdrawn_go()
  n0 = len(sim.trace)
  sim.accel = lambda sim, plan: 1. if sim.v < 4. else 0.
  while JUNCTION - sim.s > reengage_at:
    sim.step(engaged=False, gas=True)
  sim.accel = None
  n1 = len(sim.trace)
  after = drive(sim, 10., yaw=False)
  rows = sim.trace[n1:]
  assert any(p.turnMissed for _, _, p in sim.trace[n0:]), "a left driven straight through after a withdrawn go: never missed"
  assert not any(v < 0.05 and abs(JUNCTION - s) < 12. for s, v, _ in rows) and not any(p.shouldStop for p in after)
  if LINE + DRIVEN_THROUGH_M - 1. > JUNCTION - reengage_at:
    assert crossing_said(rows, LINE + DRIVEN_THROUGH_M - 1.), sorted({(str(p.controlPrompt), p.reason) for _, _, p in rows})
  # said from where the driver took the car past the hold (its reach), disengaged, before any re-engage
  driven = [r for r in sim.trace[n0:n1] if r[0] > LINE + 2.5]
  assert crossing_said(driven, LINE + DRIVEN_THROUGH_M - 1.), sorted({(str(p.controlPrompt), p.reason) for _, _, p in driven})
  assert not any(p.desire == Desire.turnLeft for p in after)


def test_after_a_withdrawn_go_a_nudge_short_of_the_holds_reach_goes_back_to_the_full_check():
  # owner decision (c): a nudge under the hold's reach (LATCH_PAST_M past it, short of the oncoming lanes) is still the
  # left's: re-engaged standing, held where it stands, waiting for its gap, and it goes on a fresh certified go
  sim = withdrawn_go()
  s0 = sim.s
  sim.accel = lambda sim, plan: 0.4 if sim.v < 0.6 else 0.
  while sim.s < s0 + 1.2:
    sim.step(engaged=False, gas=True)
  sim.accel = lambda sim, plan: -1. if sim.v > 0. else 0.
  while sim.v > 0.:
    sim.step(engaged=False)
  sim.accel = None
  s1 = sim.s
  plans = drive(sim, 3.)
  assert all(p.shouldStop for p in plans[5:]) and sim.s - s1 < 0.1 and str(plans[-1].unprotectedLeft.state) == 'waitGap'
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans) and sim.tp.map_controls.driven is None
  plans = drive(sim, 40.)
  assert any(p.desire == Desire.turnLeft for p in plans) and sim.s > JUNCTION + 10.
  assert not any(p.turnMissed for p in plans) and sim.tp.map_controls.driven is None
