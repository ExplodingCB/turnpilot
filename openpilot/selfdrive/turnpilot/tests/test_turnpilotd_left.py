"""turnpilotd driving the autonomous unprotected left (unprotected_left.py, DESIGN phase 4 G4) in closed loop: the synthetic
route, model and scene of test_turnpilotd, raw road camera runs of the oncoming cars, a point-mass car following the plan."""
import math

import pytest

from openpilot.cereal import log, messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.detector.light_state import LIGHT_MODE_DEFAULT
from openpilot.selfdrive.turnpilot.hooks import CLEAR_DESIRE, TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.map_controls import CAMERA_GREEN_STEADY_S, SIGNAL_UNSEEN_REASON, SIGNAL_UNSEEN_S
from openpilot.selfdrive.turnpilot.maps.defaults import FAR_CROP_LEFT_RANGE_M
from openpilot.selfdrive.turnpilot.scene.tracker import RawBox, RawFrame
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M, ONCOMING_ENTRY_M
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of, nav_instruction
from openpilot.selfdrive.turnpilot.tests.test_camera_lights import detections as light_detections
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
from openpilot.selfdrive.turnpilot.turn_speed import POST_GO_PROFILES
from openpilot.selfdrive.turnpilot.turnpilotd import GO_LINE_M, LEFT_NO_MOVE_S, NO_MOVE_S, TurnPilot, auto_left_for, left_range_for
from openpilot.selfdrive.turnpilot import unprotected_left as ul_mod
from openpilot.selfdrive.turnpilot.unprotected_left import (CLEAR_WINDOW_S, CREEP_STALL_S, HELD_MIN_M, LIGHT_CHANGED_REASON, LIGHT_IN_JUNCTION_REASON,
                                                           LIGHT_RED_MAX_S, LIGHT_WAIT_REASON,
                                                           LONG_WAIT_REASON, ROAD_RANGE_M, STALE_UNDECIDED_S, STAND_UNDECIDED_S,
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


def car_frame(sim, x, y):
  """A road-frame position (x along the approach, y to the left) in the car's frame, the car yawed sim.heading_deg (the
  Sim's yaw_rate stepped, left positive): what the road camera sees of it once the car turns."""
  a = math.radians(getattr(sim, 'heading_deg', 0.))
  return x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a)


def road_runs(cars=lambda sim: ()):
  """A road camera run every other frame (the detector's 10 Hz), of cars at absolute road positions (s, y): their x is s
  less the car's, seen from the car's heading (car_frame: review 2026-11-08, until then boxed as if the car never yawed, so a
  car in the band left it once the Sim turned the car)."""
  frame = [0]

  def raw(sim):
    frame[0] += 1
    if frame[0] % 2:
      return None
    boxes = [raw_box(*car_frame(sim, s - sim.s, y)) for s, y in cars(sim) if -5. < s - sim.s < 400.]
    return RawFrame(boxes, 'road', sim.t)
  return raw


def with_wide(raw):
  """raw's runs, and on every third frame it has none a wide camera run seeing nothing (the left's roadAndWide: one wide run
  in WIDE_EVERY): the cross road's view a red's clearing needs (unprotected_left.CROSS_VIEW_FRESH_S)."""
  gaps = [0]

  def runs(sim):
    r = raw(sim)
    if r is not None:
      return r
    gaps[0] += 1
    return RawFrame([], 'wide', sim.t) if gaps[0] % 3 == 0 else None
  return runs


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


def test_a_left_whose_line_is_at_a_wrong_way_opening_pulses_only_past_it():
  """The autonomous left's go is its first pulse, from its hold line 9 m before the junction. Where a divided road's near
  carriageway, one-way toward the car (navd's maneuverWrongWayM), is that close before the junction, the line stands at it:
  the go lets the car roll with no desire, and the pulse goes once it is WRONG_WAY_PAST_M past the opening (desire_timing:
  wrong_way_wait), never a left desire at the one-way. It still commits and turns, said as a left go, no handback."""
  from openpilot.selfdrive.turnpilot.desire_timing import WRONG_WAY_PAST_M
  w = 10.
  sim = left_sim()
  sim.navd.wrong_way = {0: w}
  plans = []
  while sim.s < JUNCTION + 30. and sim.t < 90.:
    plans += drive(sim, DT_MDL)
  assert sim.s > JUNCTION + 30.
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert sim.trace[go][0] >= JUNCTION - w + WRONG_WAY_PAST_M - 0.1  # past the opening, not at the line
  assert any(p.controlPrompt == 'leftGo' for p in plans) and any(str(p.unprotectedLeft.state) == 'committed' for p in plans)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans) and not any(p.turnMissed for p in plans)


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


def standing_run(raw_at, track_at, seconds, vx=0., stream=(), leaves_t=math.inf):
  """LEFT_25 at its line with a car standing at (m past the junction, m left) raw_at (a road run box every other frame,
  None: none) and track_at (a velocity-trusted track at vx, None: none) until leaves_t s into the run, and oncoming stream
  cars (absolute s, moving at V_25, a box and a track each). (sim, plans)"""
  frame = [0]

  def movers(sim):
    return [(s0 - V_25 * sim.t, 3.6) for s0 in stream if -5. < s0 - V_25 * sim.t - sim.s < 400.]

  def raw(sim):
    frame[0] += 1
    if frame[0] % 2:
      return None
    boxes = [raw_box(JUNCTION + raw_at[0] - sim.s, raw_at[1])] if raw_at is not None and sim.t < leaves_t else []
    boxes += [raw_box(s - sim.s, y) for s, y in movers(sim)]
    return RawFrame([b for b in boxes if b.xy[0] > 0.], 'road', sim.t)
  sim = left_sim()
  sim.raw = raw
  plans = []
  for _ in range(int(seconds / DT_MDL)):
    turning = sim.tp.desires.started and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    objs = [{'cls': 'car', 'x': JUNCTION + track_at[0] - sim.s, 'y': track_at[1], 'vx': vx, 'vy': 0., 'age': 60, 'trackId': 40,
             'conf': 0.8, 'camera': 'road'}] if track_at is not None and sim.t < leaves_t else []
    objs += [{'cls': 'car', 'x': s - sim.s, 'y': y, 'vx': -V_25, 'vy': 0., 'age': 60, 'trackId': 50 + k, 'conf': 0.8, 'camera': 'road'}
             for k, (s, y) in enumerate(movers(sim))]
    plans.append(sim.step(tuple(objs), engaged=True, yaw_rate=sim.v / 8. if turning else 0.))
  return sim, plans


def test_a_car_standing_in_the_way_is_said_and_waited_on_never_handed_back():
  """Skeptic g4c (still open after SIMFIX; the same on 49359b2 and be3c0ed): the left had no bound on its wait. A car parked
  50 m up the oncoming curb (in the band: a box and a track) or at the curb 6 m past the junction, 5 m right (in the junction
  box: a track) held the car at its line the whole 240 s run with only 'waitingForGap' (shown, never heard). Until
  2026-11-07 waiting STAND_UNDECIDED_S on what the tracker says stands was the 'standing' handback. Kellen, 2026-11-07: "no,
  no handbacks". Past STAND_UNDECIDED_S its line says what holds it, in plain words, the car held at its line, no
  take-control however long; the car gone, the left goes on its gap."""
  for raw_at, track_at in (((50., 7.5), (50., 7.5)), (None, (6., -5.))):
    sim, plans = standing_run(raw_at, track_at, 150.)
    held = next(i for i, p in enumerate(plans) if p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap')
    assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans), raw_at or track_at
    said = next((i for i, p in enumerate(plans) if p.reason == LONG_WAIT_REASON['standing']), None)
    assert said is not None and abs((said - held) * DT_MDL - STAND_UNDECIDED_S) < 1., raw_at or track_at
    assert all(str(p.controlPrompt) == 'waitingForGap' and p.shouldStop and p.reason == LONG_WAIT_REASON['standing']
               for p in plans[said:]), raw_at or track_at
    assert not any(p.desire == Desire.turnLeft for p in plans) and sim.s < LINE + 1.
  # the parked car pulls away 100 s into the run: the left goes from its line, never having handed back
  sim, plans = standing_run((50., 7.5), (50., 7.5), 140., leaves_t=100.)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert go * DT_MDL > 100. and sim.s > JUNCTION + 10.


def test_moving_traffic_with_no_gap_is_never_called_standing_and_a_wait_under_45_s_is_silent():
  # a stream of oncoming cars at the limit 3 s apart for 40 s, each a box and a moving velocity-trusted track: ordinary
  # waiting, silent, never the standing fault, and a go once it has passed; a car that moves on is no longer standing
  stream = [JUNCTION + 60. + V_25 * 25. + k * 3. * V_25 for k in range(13)]
  sim, plans = standing_run(None, None, 110., stream=stream)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  waited = sum(p.shouldStop for p in plans) * DT_MDL
  assert 30. < waited < STAND_UNDECIDED_S and any(p.desire == Desire.turnLeft for p in plans) and sim.s > JUNCTION + 10.


def test_a_wait_at_the_line_with_no_gap_is_said_at_45_s_and_waited_through_whatever_the_tracker_says():
  """Owner decision (ii), 2026-10-30, Kellen's delegate (DESIGN's UNDECIDED row: no gap in 45 s): 154bdaa bounded the wait
  at the line only on what the tracker says stands, so a car parked in the band seen only as raw boxes (never tracked: a
  box nothing tracks can't be told standing from coming) held the car at its line silently with no limit, and an oncoming
  stream with no gap, or a car moving in the junction box, was waited through however long. Until 2026-11-07 every wait at
  the line for a gap ended at STAND_UNDECIDED_S in the loud handback ("No gap in oncoming traffic: take control"). Kellen,
  2026-11-07: "no, no handbacks". It is now logged once there (turnpilot.left_no_gap) and waited through, the car held at its
  line, waitingForGap and its line throughout, never a go into the traffic it sees, never a take-control."""
  from unittest import mock
  stream = [JUNCTION + 60. + V_25 * 25. + k * 3. * V_25 for k in range(40)]
  for raw_at, track_at, vx, movers in (((50., 7.5), None, 0., ()), (None, None, 0., stream), (None, (6., -5.), 3., ())):
    with mock.patch.object(ul_mod.cloudlog, 'event') as event:
      sim, plans = standing_run(raw_at, track_at, 140., vx=vx, stream=movers)
    case = (raw_at, track_at, vx, len(movers))
    held = next(i for i, p in enumerate(plans) if p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap')
    assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans), case
    assert all(str(p.controlPrompt) == 'waitingForGap' and p.shouldStop for p in plans[held + 5:]), case
    said = [c.kwargs for c in event.call_args_list if c.args and c.args[0] == 'turnpilot.left_no_gap']
    assert len(said) == 1 and abs(said[0]['wait_s'] - STAND_UNDECIDED_S) < 0.1, (case, said)
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


def test_a_left_past_the_proven_range_is_taken_from_its_line_on_the_band_it_sees():
  # 45 mph mapped: R_req past the road camera's range (the far crop's unmeasured). Kellen, 2026-11-07 ("no, no handbacks"; a
  # safety driver supervises): driven to its line and gone from it on the band the camera covers, never handed back for range
  sim = Sim([(JUNCTION, "turn", "left", (0., 270., 2, 'present', 20.1, 1, STRAIGHT_M)), (1500., "turn", "right", junction_of("right"))])
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', TEST_RANGE_M, False, road_runs()
  sim.answers_prompt = True
  assert r_req(20.1, t_req(HOLD_LINE_M + 1., 1), HOLD_LINE_M + 1.) > TEST_RANGE_M
  plans = drive(sim, 60.)
  assert any(p.desire == Desire.turnLeft for p in plans)
  assert not any(p.tooFastToTurn for p in plans)
  assert not any(str(p.unprotectedLeft.state) == 'undecided' or str(p.controlPrompt) == 'leftUndecidedTakeControl' for p in plans)


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


def held_at_the_line(sim, line, hold_s=4., limit_s=120.):
  """Drives sim (no turn yaw) until it has stood held at the hold line at line for hold_s, or limit_s: its plans and the skip
  reasons TurnPilot gave meanwhile."""
  plans, reasons, held_s = [], set(), 0.
  while held_s < hold_s:
    plans += drive(sim, DT_MDL, yaw=False)
    reasons.add(sim.tp.skip.reason)
    held_s = held_s + DT_MDL if plans[-1].shouldStop and sim.v < 0.05 else 0.
    assert sim.t < limit_s and sim.s < line + 1., (sim.t, sim.s)
  return plans, reasons


def test_a_left_the_detector_cant_see_far_enough_for_is_driven_to_its_line_and_gone_from_it():
  # 2026-11-06 (Kellen: never route around or skip a left for range): past the detector's proven range the left was skipped as
  # soon as it was the route's maneuver (unprotectedRange) and navd rerouted. 2026-11-07 ("no, no handbacks"): driven to its
  # hold line, never skipped, and gone from it on the band the camera covers, never handed back for the reach
  for junction in ((0., 270., 2, 'present', 20.1, 1, STRAIGHT_M), (0., 270., 2, 'present', 0., 0, STRAIGHT_M)):  # 45 mph, unknown (45)
    sim = Sim([(600., "turn", "left", junction), (2000., "turn", "right", junction_of("right"))])
    sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', TEST_RANGE_M, False, road_runs()
    sim.answers_prompt = True
    plans = drive(sim, 90., yaw=False)
    first = plans[2]
    assert str(first.unprotectedLeft.state) == 'approach' and first.maneuverDistance > 500. and not first.tooFastToTurn
    assert not any(p.tooFastToTurn or str(p.unprotectedLeft.state) == 'skipped' for p in plans), junction
    assert not any(str(p.skipReason) for p in plans)
    assert any(p.shouldStop and p.mapControl.kind == 'unprotectedTurn' for p in plans), "held at its line"
    assert not any(str(p.unprotectedLeft.state) == 'undecided' for p in plans), junction
    assert any(p.desire == Desire.turnLeft for p in plans), junction
    assert any(p.unprotectedLeft.rReq > p.unprotectedLeft.rangeM == TEST_RANGE_M for p in plans if str(p.unprotectedLeft.state) != 'off')
  # off: the line's handback, as G1's, never skipped for range (nor it ever was)
  sim = Sim([(600., "turn", "left", (0., 270., 2, 'present', 20.1, 1)), (2000., "turn", "right", junction_of("right"))])
  sim.tp.auto_left, sim.go_taps = 'off', False
  assert not any(p.tooFastToTurn for p in drive(sim, 5.))


def test_a_left_whose_opposing_road_isnt_known_straight_over_r_req_is_gone_on_its_straight_band():
  # skeptic G4 (residual): a car up a road curving away from the turn is outside the straight band. Until 2026-11-07 such a
  # left was handed back at its line ('curve'); Kellen, 2026-11-07 ("no, no handbacks"; the safety driver supervises): judged
  # on the band the camera covers and gone when it is clear, never skipped, still logged once on the way as not straight
  from unittest import mock
  for straight in (0., 60.):
    sim = Sim([(600., "turn", "left", (0., 270., 2, 'present', V_25, 1, straight)), (2000., "turn", "right", junction_of("right"))])
    sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', TEST_RANGE_M, False, road_runs()
    sim.answers_prompt = True
    with mock.patch('openpilot.selfdrive.turnpilot.turnpilotd.cloudlog') as log:
      plans = drive(sim, 90., yaw=False)
    assert not any(p.tooFastToTurn or str(p.unprotectedLeft.state) == 'skipped' for p in plans), straight
    assert not any(str(p.unprotectedLeft.state) == 'undecided' for p in plans), straight
    assert any(p.desire == Desire.turnLeft for p in plans), straight
    events = [c.kwargs for c in log.event.call_args_list if c.args[0] == 'turnpilot.left_not_straight']
    assert len(events) == 1 and events[0]['straight_m'] == straight and events[0]['r_req'] > straight + HOLD_LINE_M + 1., straight


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


def test_a_creep_for_a_view_the_car_never_makes_is_never_silent_nor_handed_back():
  # skeptic G4 (blocker 4): a standing opposing left-turner blocks the view, the car is asked to creep and never moves (the
  # model holding it): 270 s in 'creep' with only the visual creepingForView. Until 2026-11-07 a loud handback (creepStall,
  # "Can't creep for a view: take control"); Kellen, 2026-11-07: "no, no handbacks". Now its line says it waits for the view
  # from CREEP_STALL_S, the car held at its line, no take-control however long, never a go on the hidden lanes
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
  plans = [sim.step(objects(sim), engaged=True) for _ in range(int(120. / DT_MDL))]
  creep = next(i for i, p in enumerate(plans) if str(p.unprotectedLeft.state) == 'creep')
  said = next((i for i, p in enumerate(plans) if p.reason == LONG_WAIT_REASON['view']), None)
  assert said is not None, "stood creeping for a view it never got, silently"
  assert (said - creep) * DT_MDL <= CREEP_STALL_S + 1.
  # the creep stays asked (the latch's stop point on, shouldStop off: the model may yet take it), its line the wait
  assert all(p.reason == LONG_WAIT_REASON['view'] and str(p.controlPrompt) == 'waitingForGap' for p in plans[said:])
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert not any(p.desire == Desire.turnLeft for p in plans) and sim.s < LINE + 1.


def test_a_creep_for_a_view_the_car_makes_then_waits_keeps_the_car_held_and_said():
  # skeptic residual-check-1 (058df13): the same blocked view with the car moving as asked (the plain Sim, where HeldSim's
  # moved_m stays 0): it crept 3.8 m past its line, stayed blocked and went undecided 'blocked', and the latch was let go
  # on the first handback frame as the driver's with no driver input (shouldStop on 0 of 384 frames, the take-control
  # gone after 104 frames, the car 164 m past the junction at 13 m/s). Held short of the oncoming lanes, said throughout.
  # Since 2026-11-07 (Kellen: no handbacks) the view still blocked BLOCKED_UNDECIDED_S after the full creep is a wait where
  # it crept to, its line saying so, never a take-control: the latch never let go on it either
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
  assert 'undecided' not in states and not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  und = next(i for i, p in enumerate(plans) if p.reason == LONG_WAIT_REASON['view'])
  assert 'creep' in states[:und] and sim.trace[und][0] - LINE > 2.  # it crept past the latch's reach first
  after = plans[und:]
  assert all(str(p.controlPrompt) == 'waitingForGap' and p.reason == LONG_WAIT_REASON['view'] and p.shouldStop for p in after)
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
  # still a go from the line, withdrawn and handed back when the car doesn't move, never taken for a go in the junction left
  # to the stall's handback. Unlifted there (too near the oncoming lanes for the lift's nudge), with the model planning
  # motion (the Sim's drives on) and the car standing: a fault, said at NO_MOVE_S (review 2026-11-07, LS-2: the model's launch
  # from a stand takes 0.9-1.6 s on this car, so LEFT_NO_MOVE_S is no bound for it)
  sim, plans, last = jump_run(4., 70., sim_cls=GoHeldSim)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert sim.distance() < GO_LINE_M and not sim.tp.go_in_junction
  after = plans[go:]
  withdrawn = next(i for i, p in enumerate(after) if str(p.controlPrompt) == 'cantStartTakeControl')
  assert withdrawn * DT_MDL <= NO_MOVE_S + 0.2 and after[-1].shouldStop and sim.s < LINE + 1.


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
  # (drive 29, 2026-11-08: the signal's own stop at its line holds it there, the left's hold in the junction's mouth beyond)
  sim = signal_sim()
  plans = run_objects(sim, 60.)
  assert not any(str(p.controlPrompt) == 'leftGo' or p.desire == Desire.turnLeft for p in plans)
  assert sim.s < LINE + 1. and str(plans[-1].unprotectedLeft.state) in ('approach', 'waitGap') and plans[-1].shouldStop
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
  st = {'go_t': None, 'clear': False}

  def ped(sim):
    if st['go_t'] is None and sim.tp.left_now is not None and sim.tp.left_now.state == 'go':
      st['go_t'] = sim.t
    if st['go_t'] is not None and sim.t < st['go_t'] + 3.:
      return ({'cls': 'pedestrian', 'x': 6., 'y': 0., 'vx': 0., 'vy': 0., 'age': 40, 'trackId': 5, 'conf': 0.8, 'inPath': True,
               'camera': 'road'},)
    if st['go_t'] is not None and not st['clear']:  # seen walking clear of the path (WF-13: never lost in front of the car)
      st['clear'] = True
      return ({'cls': 'pedestrian', 'x': 6., 'y': 3., 'vx': 0., 'vy': 1.2, 'age': 41, 'trackId': 5, 'conf': 0.8, 'inPath': False,
               'camera': 'road'},)
    return ()
  sim = left_sim()
  plans = run_objects(sim, 60., ped)
  gos = [i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftGo' and str(plans[i - 1].controlPrompt) != 'leftGo']
  assert len(gos) >= 2 and gos[1] * DT_MDL >= st['go_t'] + 3. + CLEAR_WINDOW_S - 0.2
  assert all(v < 0.3 for _, v, _ in sim.trace[gos[0] + 2:gos[1]])  # never ran on the first go
  assert sim.s > JUNCTION + 10.


def test_a_signal_whose_light_nothing_reads_is_gone_on_the_clear_band():
  # a left at a mapped signal with no light read (no source, or the camera's reader can't see the head). Kellen, 2026-11-07
  # (no handbacks): an unread light is no reason to wait; the left goes on the clear band, never handed back for the light
  # (a red or yellow read still holds it: the tests above)
  sim = signal_sim(oracle=False)
  plans = run_objects(sim, 50.)
  assert any(p.desire == Desire.turnLeft for p in plans)
  assert not any(str(p.controlPrompt) == 'leftUndecidedTakeControl' or str(p.unprotectedLeft.state) == 'undecided' for p in plans)


def camera_signal_sim(read):
  """LEFT_25 at a signal mapped at its hold line, lit as on the car: no accepted light source (the oracle off), the camera's
  reader in its default add-only mode (LIGHT_MODE_DEFAULT) reading the head 15 m past the junction as read(t) on every
  other frame (None: nothing read), and an oncoming car every 3 s at 25 mph until 50 s, the band empty after."""
  sim = Sim(LEFT_25, controls=[(JUNCTION - 8.5, 'signal')])
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps = 'auto', TEST_RANGE_M, False
  sim.answers_prompt, sim.tp.oracle_ok, sim.tp.camera_mode = True, False, LIGHT_MODE_DEFAULT
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

  def lights(sim):
    state = read(sim.t)
    if state is not None and int(round(sim.t / DT_MDL)) % 2 == 0:
      sim.sm.put('turnpilotDetections', light_detections(state, JUNCTION + 15. - sim.s, shadow=False))
    return none
  sim.lights = lights
  sim.raw = with_wide(road_runs(lambda sim: ((JUNCTION + 60. + V_25 * (3. * (int(sim.t / 3.) + 1) - sim.t), 3.6),) if sim.t < 50. else ()))
  return sim


def clearing_go(sim, red_at, words, clear_at, objects=lambda sim: (), seconds=80.):
  """Kellen, 2026-11-08: a left waiting past its line (the junction's mouth at a mapped signal, or crept for a view) when a red
  or yellow is first read clears the junction like a driver, on its gap: run sim to seconds, the model stopping from red_at
  for the red it sees (its shouldStop: on the car it does; turnpilotd lifts the left's go over it, left_clearing). Asserts
  the car stood held (a blocked view crept for, short of the oncoming lanes), its line words, from the red to its go; the
  go only once the band was clear CLEAR_WINDOW_S from
  clear_at (an oncoming car seen holds it until it passes; a view blocked until it opens), lifted over the model's hold
  (unprotectedLeft certified), the car turned across and out; heard once at the red, never a handback. Returns the plans."""
  lifted = []
  plans = []
  t0, n0 = sim.t, len(sim.trace)
  for _ in range(int(round(seconds / DT_MDL))):
    sim.model_holds = sim.t >= red_at and sim.s < JUNCTION  # the model stops for the red it sees, until in the turn
    turning = sim.tp.desires.started and abs(JUNCTION - sim.s) < 12. and sim.v > 0.3
    plans.append(sim.step(objects(sim), engaged=True, yaw_rate=sim.v / 8. if turning else 0.))
    if sim.tp.go_cert.active and sim.tp.go_cert.cause == 'unprotectedLeft':
      lifted.append(len(plans) - 1)
  i_red = max(int(round((red_at + 0.5 - t0) / DT_MDL)), 0)
  go = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftGo')
  assert clear_at + CLEAR_WINDOW_S - 0.2 < t0 + go * DT_MDL < clear_at + CLEAR_WINDOW_S + 4., t0 + go * DT_MDL
  held = plans[i_red:go]
  # held until the gap, saying it; a view blocked by a turner still creeps for it, short of the oncoming lanes
  view = ("Creeping for a view", ul_mod.LONG_WAIT_REASON['view'])
  assert held and all(p.shouldStop and p.reason in (words,) + view for p in held[2:]), sorted({p.reason for p in held})
  assert any(p.reason == words for p in held)
  assert all(JUNCTION - s > ONCOMING_ENTRY_M for s, _, _ in sim.trace[n0:n0 + go])
  assert lifted and lifted[0] >= go  # the go lifted over the model's red hold
  assert any(p.desire == Desire.turnLeft for p in plans[go:]) and sim.s > JUNCTION + 5.
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or str(p.unprotectedLeft.state) == 'undecided' for p in plans)
  spells = heard_spells(selfdrived_alerts(plans), max(int(round((red_at - t0) / DT_MDL)) - 1, 0))
  assert len(spells) >= 1 and abs(t0 + spells[0][0] * DT_MDL - red_at) < 1.5, spells
  return plans


@pytest.mark.parametrize('green_first', [True, False])
def test_a_camera_red_lost_at_the_line_is_waited_for_as_the_lights_said_once_and_gone_on_the_next_green(green_first, monkeypatch):
  # review 2026-11-07 (handbacks-rest): on the car (camera add-only, no accepted source) a camera red read at the left's line
  # raises map_controls' own signal stop there, and the read's loss keeps it (a light gone unread keeps a stop) until a
  # steady green releases it; but turn_light gave 'none' once the red was no longer read, so the left, held by that stop
  # (other_hold), said "Waiting for a gap" at an empty band for as long as it stood (150 s in the review's Sim run), never
  # logged the wait, and was never going on the gap its line promised. At HEAD green, then red, then lost handed back
  # (lightChange) at once; arrive on red, then lost, was the same silent "Waiting for a gap" wait. Now the stop in force is
  # the left's light: its line the light's wait throughout, the change from the green said once, the wait said once as
  # LIGHT_RED_MAX_S passes, never a go nor a handback, and it goes on its gap once a camera green releases the stop
  red_at, lost_at, green_at = 45. if green_first else 0., 55., 230.

  def read(t):
    return 'green' if t < red_at or t >= green_at else 'red' if t < lost_at else None
  sim = camera_signal_sim(read)
  said = []
  monkeypatch.setattr(ul_mod.cloudlog, 'event', lambda name, *a, **kw: said.append((round(sim.t, 1), name)))
  if green_first:
    # drive 29 (2026-11-08): waiting for its gap on the green in the junction's mouth (SIGNAL_LEFT_HOLD_M), past the signal's
    # line, where the signal's own stop is never made. Until 2026-11-08 the red read there held the car in the junction
    # until a steady green (LS-4's words). Kellen's ruling: like a driver it clears the junction on its gap, the model
    # stopping for the red it sees (its hold lifted: left_clearing), the oncoming car every 3 s to 50 s holding it until
    # the band is clear, said once (left_light_changed), never a handback
    clearing_go(sim, red_at, LIGHT_CHANGED_REASON, 50.)
    assert [name for _, name in said if name in ('turnpilot.left_light_changed', 'turnpilot.left_light_wait',
                                                  'turnpilot.left_no_gap')] == ['turnpilot.left_light_changed']
    return
  plans = run_objects(sim, green_at)
  unseen_i = int(round((lost_at + SIGNAL_UNSEEN_S + 2.) / DT_MDL))  # (a lost read is carried CAMERA_STALE_S first)
  waits = [(t, name) for t, name in said if name in ('turnpilot.left_light_wait', 'turnpilot.left_light_changed', 'turnpilot.left_light_unseen',
                                                     'turnpilot.left_no_gap', 'turnpilot.left_far_wait')]
  # arrived on the red (read before the car passed its line): the signal's own stop at its line holds the car, short of the
  # junction (the left's hold in the mouth beyond it is never reached), its read lost said as any through signal's is, held
  # for a green
  stood = int(round(30. / DT_MDL))
  assert all(p.shouldStop and str(p.mapControl.kind) == 'signal' for p in plans[stood:]), {str(p.mapControl.kind) for p in plans[stood:]}
  assert all(p.reason == SIGNAL_UNSEEN_REASON for p in plans[unseen_i:]), {p.reason for p in plans[unseen_i:]}
  assert sim.s < LINE + 1.
  assert waits == []
  assert not any(p.desire == Desire.turnLeft or str(p.controlPrompt) == 'leftGo' for p in plans)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or str(p.unprotectedLeft.state) == 'undecided' for p in plans)
  # the camera's steady green releases TurnPilot's own stop (CAMERA_GREEN_STEADY_S), then a fresh window on the empty band (from
  # the signal's line, after the roll up to the mouth)
  plans = run_objects(sim, 20.)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert CAMERA_GREEN_STEADY_S + CLEAR_WINDOW_S - 0.2 < go * DT_MDL < CAMERA_GREEN_STEADY_S + CLEAR_WINDOW_S + (2. if green_first else 8.)
  assert sim.s > JUNCTION + 10. and not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def unmapped_camera_sim(read, status=lambda t: 'read'):
  """LEFT_25 at a junction the map has no signal for (OSM missing one: 11-06 had one), lit as on the car: no accepted light
  source, the camera's reader add-only (LIGHT_MODE_DEFAULT) reading the governing head 15 m past the junction as read(t) on
  every other frame (None: nothing read), its lightStatus status(t); an empty band (the oncoming road at its own red)."""
  sim = Sim(LEFT_25)
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps = 'auto', TEST_RANGE_M, False
  sim.answers_prompt, sim.tp.oracle_ok, sim.tp.camera_mode = True, False, LIGHT_MODE_DEFAULT
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

  def lights(sim):
    state, st = read(sim.t), status(sim.t)
    if (state is not None or st != 'read') and int(round(sim.t / DT_MDL)) % 2 == 0:
      sim.sm.put('turnpilotDetections', light_detections(state or 'unknown', JUNCTION + 15. - sim.s, shadow=False, status=st))
    return none
  sim.lights = lights
  sim.raw = road_runs()
  return sim


@pytest.mark.parametrize('after', ['lost', 'unknown', 'disabled'])
def test_a_camera_red_at_a_junction_with_no_mapped_signal_holds_the_left_until_a_steady_green(after):
  # review 2026-11-07 (HR-1): at a junction the map has no signal for, the camera's own signal is never made at the left's own
  # junction (its own_turn gate), so a red the camera read there (add-only, the car's only source) was forgotten the moment it
  # went unread: nothing read (a truck in front of the head), read 'unknown' (the 11-06 night's usual read), or the reader
  # gone disabled. The left then went 2.6-3.5 s later on the oncoming band, empty at its own red, its go certified for
  # plannerd's lift over the model's own red hold. Now the red stays the left's light until a steady camera green
  lost_at, green_at = 40., 190.

  def read(t):
    return 'red' if t < lost_at else 'green' if t >= green_at else None if after != 'unknown' else 'unknown'
  sim = unmapped_camera_sim(read, lambda t: 'disabled' if after == 'disabled' and lost_at <= t < green_at else 'read')
  plans = run_objects(sim, green_at)
  at_line = next(i for i, p in enumerate(plans) if str(p.unprotectedLeft.state) == 'waitGap' and p.shouldStop)
  assert at_line * DT_MDL < lost_at
  held = plans[at_line:]
  from openpilot.selfdrive.turnpilot.map_controls import CAMERA_OFF_REASON  # the reader failing: its onset's own words (SF-2)
  # the red latched: the light's wait, then from SIGNAL_UNSEEN_S after the loss the unseen words (AL-2), still held
  waits = (LIGHT_WAIT_REASON['red'], SIGNAL_UNSEEN_REASON)
  assert all(p.reason in waits + (CAMERA_OFF_REASON,) and p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap'
             for p in held), sorted({(p.reason, str(p.unprotectedLeft.state)) for p in held})
  assert sum(p.reason in waits for p in held) > 0.95 * len(held)
  assert plans[-1].reason == SIGNAL_UNSEEN_REASON
  assert not any(p.desire == Desire.turnLeft or str(p.goCertified.cause) == 'unprotectedLeft' for p in plans)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans) and sim.s < LINE + 1.
  # a steady camera green lets it go on its gap: a fresh window from the release, never lifted (no accepted green there)
  plans = run_objects(sim, 15.)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  assert CAMERA_GREEN_STEADY_S + CLEAR_WINDOW_S - 0.2 < go * DT_MDL < CAMERA_GREEN_STEADY_S + CLEAR_WINDOW_S + 2.
  assert not any(str(p.goCertified.cause) == 'unprotectedLeft' for p in plans)
  assert sim.s > JUNCTION + 10. and not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def test_a_red_far_short_of_the_lefts_junction_is_no_light_of_its_own():
  # the latch takes only heads the junction's own line (the junction less 10 m) would: a red read 25 m short of the junction
  # (the previous signal, a mid-block crossing) holds the left only while it is read, never once the car is past it
  sim = Sim(LEFT_25)
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps = 'auto', TEST_RANGE_M, False
  sim.answers_prompt, sim.tp.oracle_ok, sim.tp.camera_mode = True, False, LIGHT_MODE_DEFAULT
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

  def lights(sim):
    if JUNCTION - 25. - sim.s > 0. and int(round(sim.t / DT_MDL)) % 2 == 0:
      sim.sm.put('turnpilotDetections', light_detections('red', JUNCTION - 25. - sim.s, shadow=False))
    return none
  sim.lights = lights
  sim.raw = road_runs()
  plans = run_objects(sim, 60.)
  assert any(p.desire == Desire.turnLeft for p in plans) and sim.s > JUNCTION + 10.


@pytest.mark.parametrize('dark', [False, True])
def test_a_left_at_a_junction_the_camera_boxes_a_light_at_is_never_lifted_and_goes_with_the_model(dark):
  # review 2026-11-07 (HR-2): no signal mapped, the governing head boxed on every road run with no colour read (the 11-06
  # night's every governing head, 667 reads): the left's go was certified for plannerd's lift at 31.75 s exactly as on an
  # empty road, by day and after dark, while unsignalized_why's camera clause kept turnRelease and the unstick off. Now the
  # lift is refused there; the left goes unlifted, with the model (the Sim's drives on)
  from openpilot.selfdrive.turnpilot.darkness import DarkFilter
  sim = unmapped_camera_sim(lambda t: 'unknown')
  sim.tp.dark_filter = DarkFilter(dark=dark)
  plans = run_objects(sim, 60.)
  assert any(p.desire == Desire.turnLeft for p in plans) and sim.s > JUNCTION + 10.
  assert not any(str(p.goCertified.cause) == 'unprotectedLeft' for p in plans)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  # no head read at all: an unsignalized left is lifted as before (its companion: test_turnpilotd_go's lift test)
  sim = left_sim()
  plans = run_objects(sim, 60.)
  assert any(str(p.goCertified.cause) == 'unprotectedLeft' for p in plans) and sim.s > JUNCTION + 10.


class Selfdrived:
  """selfdrived's alerts for turnpilotd's plans, its own way, one plan at a time (selfdrived.py: hooks.plan_events, then
  hooks.CarSignalEvents for the turn signal, into Events and AlertManager at its 100 Hz over turnpilotd's 20 Hz; the car's
  signal not fitted, so the driver signals; the plan's SubMaster passed to the alert callbacks, as selfdrived's): step(plan)
  gives the (alertText1, alertText2, audibleAlert name) of each of its five frames."""
  def __init__(self):
    import types

    from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
    from openpilot.selfdrive.turnpilot import hooks
    self.ev, self.current, self.am, self.frame = hooks.CarSignalEvents(), {'plan': None}, AlertManager(), 0
    self.port = types.SimpleNamespace(status='off', fault='none', check='idle', checkWhy='', stillOn=False, rejected=0, left=False,
                                      right=False, hazard=False, block='none')

  def step(self, p):
    import time

    import openpilot.selfdrive.selfdrived.events as events_mod
    from openpilot.common.realtime import DT_CTRL
    from openpilot.selfdrive.selfdrived.events import ET, Events
    from openpilot.selfdrive.turnpilot import hooks
    EventName = log.OnroadEvent.EventName
    AudibleAlert = log.SelfdriveState.AudibleAlert
    sound_name = {v: k for k, v in AudibleAlert.schema.enumerants.items()}

    class PlanSM(dict):
      def __init__(self, plan):
        super().__init__({'turnpilotPlan': plan})
        self.recv_time, self.valid = {'turnpilotPlan': time.monotonic()}, {'turnpilotPlan': True}
    self.current['plan'] = p
    psm, out = PlanSM(p), []
    saved = events_mod.CAR_SIGNAL_EVENTS, events_mod.shown_plan
    events_mod.CAR_SIGNAL_EVENTS, events_mod.shown_plan = self.ev, lambda sm, now=None: self.current['plan']
    try:
      for _ in range(5):
        events = Events()
        for name in hooks.plan_events(psm) + list(self.ev.update(p, self.port, 0, DT_CTRL)):
          events.add(getattr(EventName, name))
        self.am.add_many(self.frame, events.create_alerts([ET.PERMANENT, ET.WARNING], [None, None, psm, False, 0, None]))
        self.am.process_alerts(self.frame, set())
        a = self.am.current_alert
        out.append((a.alert_text_1, a.alert_text_2, sound_name.get(int(a.audible_alert), str(a.audible_alert))))
        self.frame += 1
    finally:
      events_mod.CAR_SIGNAL_EVENTS, events_mod.shown_plan = saved
    return out


def selfdrived_alerts(plans):
  """What selfdrived shows and plays for these plans (Selfdrived): (plan index, alertText1, alertText2, audibleAlert name)
  per selfdrived frame."""
  sd = Selfdrived()
  return [(i, *a) for i, p in enumerate(plans) for a in sd.step(p)]


def drive_answering_what_is_shown(sim, seconds, react_s, step=None):
  """sim driven seconds (step(sim): one plan, drive's by default) with a driver who lights the lamp the prompt asks for once
  "Signal LEFT Now" has been on selfdrived's screen react_s in all (None: never), not on the plan's field the screen may not
  show (Sim.answers_prompt). Returns (plans, shown): shown per plan, its five selfdrived frames."""
  from openpilot.common.realtime import DT_CTRL
  sim.answers_prompt = False
  sd, plans, shown, seen = Selfdrived(), [], [], 0.
  for _ in range(int(round(seconds / DT_MDL))):
    p = step(sim) if step is not None else drive(sim, DT_MDL)[0]
    frames = sd.step(p)
    plans.append(p)
    shown.append(frames)
    seen += sum(f[0] == "Signal LEFT Now" for f in frames) * DT_CTRL
    if react_s is not None and seen >= react_s - 1e-9:
      sim._signal = 'left'
  return plans, shown


def test_the_light_readers_failing_at_a_signalized_lefts_line_is_heard_before_the_left_goes():
  """Review 2026-11-07 (SF-2): the camera's light reader failing (lightStatus disabled) 5 s after the car is held at a
  signalized left's line. Its chimed onset ran down where map_controls' own prompt was none, but the shown prompt there is
  the left's silent waitingForGap (turnpilotd.fill_left adds it, and overwrote the plan's reason), which plan_events raises
  first: 18 s of a silent "Waiting for a gap", then the left went through the signal no camera could read, and "Lights
  Unknown" came only after the go, silent. Now the onset is raised over the left's wait in its own words, with its chime,
  before the go, then the wait's line again."""
  from openpilot.common.realtime import DT_CTRL
  from openpilot.selfdrive.turnpilot.hooks import ABORT_HOLD_S
  from openpilot.selfdrive.turnpilot.map_controls import CAMERA_OFF_REASON
  sim = camera_signal_sim(lambda t: None)
  held_t = [None]
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights

  def lights(s):
    if held_t[0] is None and s.trace and s.trace[-1][2].shouldStop and s.v < 0.1 and str(s.trace[-1][2].unprotectedLeft.state) == 'waitGap':
      held_t[0] = s.t
    failing = held_t[0] is not None and s.t >= held_t[0] + 5.
    if int(round(s.t / DT_MDL)) % 2 == 0:
      s.sm.put('turnpilotDetections', light_detections('green', JUNCTION + 15. - s.s, shadow=False,
                                                       status='disabled' if failing else 'read', governs=False))
    return none
  sim.lights = lights
  plans = run_objects(sim, 75.)
  go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
  fail_i = int(round((held_t[0] + 5.) / DT_MDL))
  assert fail_i < go
  shown = selfdrived_alerts(plans)
  onset = [f for f in shown if fail_i <= f[0] < go and f[1] == 'Lights Unknown' and f[2] == CAMERA_OFF_REASON]
  assert onset and all(f[3] == 'prompt' for f in onset[:5])
  assert abs(len(onset) * DT_CTRL - ABORT_HOLD_S) < 0.2, len(onset) * DT_CTRL
  assert any(f[1] == 'Waiting for a gap' or f[1] == LIGHT_WAIT_REASON['red'] for f in shown if onset[-1][0] < f[0] < go)


def heard_spells(shown, start):
  """The audible spells in selfdrived's frames shown from plan index start: [(first plan index, its sound)] of each run of
  frames with a sound."""
  spells, prev = [], 'none'
  for i, _, _, sound in shown:
    if i >= start and sound != 'none' and prev == 'none':
      spells.append((i, sound))
    prev = sound if i >= start else 'none'
  return spells


def test_a_long_wait_at_the_line_that_wont_end_on_its_own_is_heard_once_never_a_handback():
  """Review 2026-11-07 (SF-4): every wait that ended in a handback's chime until that day (STAND_UNDECIDED_S on something
  standing, the light waits, the view and creep waits, farBlocked) was visual only: a car parked 50 m up the oncoming curb
  had 0 audible frames after the hold ('none' on 12,075 selfdrived frames) under "Waiting: something stopped is in the way"
  for 75.8 s; the camera's red lost at the line, 'Waiting for the light' 190.8 s, never heard. The state skeptic g4c rejected,
  with other words. Now each is heard once, as its old bound passes, a chime and no take-control; the plain wait for a gap
  in a moving stream stays silent (the busy road's ordinary wait)."""
  sim, plans = standing_run((50., 7.5), (50., 7.5), 150.)
  held = next(i for i, p in enumerate(plans) if p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap')
  spells = heard_spells(selfdrived_alerts(plans), held)
  assert len(spells) == 1 and spells[0][1] == 'prompt', spells
  assert abs((spells[0][0] - held) * DT_MDL - STAND_UNDECIDED_S) < 1.
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  # the camera's red read at the line all along: the light's wait, heard once at its old bound (LIGHT_RED_MAX_S); lost, it is
  # said as a through signal's is, at SIGNAL_UNSEEN_S from the loss (the next test), and only that once. At a junction with no
  # signal mapped, where the left's own hold at its line is the junction's (a mapped signal's own stop holds the car at its
  # line on a red, the left's hold being in the junction's mouth beyond it: drive 29, 2026-11-08)
  for read, at_s in ((lambda t: 'red', None), (lambda t: 'red' if t < 55. else None, 55. + SIGNAL_UNSEEN_S)):
    sim = unmapped_camera_sim(read)
    plans = run_objects(sim, 230.)
    held = next(i for i, p in enumerate(plans) if p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap')
    spells = heard_spells(selfdrived_alerts(plans), held)
    want = held * DT_MDL + LIGHT_RED_MAX_S if at_s is None else at_s  # (a lost read is carried CAMERA_STALE_S first)
    assert len(spells) == 1 and -0.1 < spells[0][0] * DT_MDL - want < 2., spells
  # an oncoming stream with no gap for 140 s: waited through, never heard
  stream = [JUNCTION + 60. + V_25 * 25. + k * 3. * V_25 for k in range(40)]
  sim, plans = standing_run(None, None, 140., stream=stream)
  held = next(i for i, p in enumerate(plans) if p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap')
  assert heard_spells(selfdrived_alerts(plans), held) == []


def test_a_light_changing_with_the_car_crept_into_the_junction_clears_it_on_the_gap():
  """Review 2026-11-07 (LS-4): a signalized left on a green with an opposing left-turner standing in the junction: the car
  crept 4 m for a view, the nose about 3.3 m from the junction's centre; the light went yellow, then red, and it was held
  there for the whole red (the read red's rail), never heard: on a cross road of two lanes or more each way that is in the
  near cross lanes through their green. LS-4 kept the hold, said and heard once. Kellen, 2026-11-08: clear the junction like
  a driver: the turner's view held until it leaves, then the gap, the go lifted over the model's red hold; said once."""
  yellow_at, red_at, green_at, occ_leaves = 50., 54., 130., 57.

  def light_at(t):
    return 'green' if t < yellow_at or t >= green_at else 'yellow' if t < red_at else 'red'
  sim = camera_signal_sim(light_at)
  sim.raw = None
  frame = [0]

  def raw(sim):
    frame[0] += 1
    if frame[0] % 2:
      return None
    return RawFrame([raw_box(JUNCTION + 6. - sim.s, 3.6)] if sim.t < occ_leaves else [], 'road', sim.t)
  sim.raw = with_wide(raw)

  def occluder(sim):
    return ({'cls': 'car', 'x': JUNCTION + 6. - sim.s, 'y': 3.6, 'vx': 0., 'vy': 0., 'age': 40, 'trackId': 7, 'conf': 0.8,
             'camera': 'road'},) if sim.t < occ_leaves else ()
  plans = run_objects(sim, yellow_at, occluder)
  assert sim.s > LINE + 2.  # crept for the view on the green
  plans += clearing_go(sim, yellow_at, LIGHT_CHANGED_REASON, occ_leaves, occluder, seconds=25.)
  assert not any(p.desire == Desire.turnLeft for p in plans[:round((occ_leaves + CLEAR_WINDOW_S - 0.2) / DT_MDL)])


@pytest.mark.parametrize('after', [None, 'unknown'])
@pytest.mark.parametrize('mapped', [True, False])
def test_a_lefts_red_no_longer_read_is_said_and_heard_as_a_through_signals_is_and_still_held(after, mapped):
  """Review 2026-11-07 (AL-2): at a signalized left a camera red read, then lost (nothing read, or the night's usual 'unknown'),
  stayed the left's red (turn_light: the stop it raised still in force, or the HR-1 latch with no signal mapped), counted
  against LIGHT_RED_MAX_S from the hold: "Waiting for the light" and silent for 124 s after the loss, where a through signal
  in the same state says "Light not seen: waiting for green" with a chime at SIGNAL_UNSEEN_S. A safety driver at a green the
  reader never read had a car that wouldn't go and nothing heard for two minutes. Now the left says it as the through signal
  does, from the loss (LeftInput.light_read): its line the unseen words and one chime at SIGNAL_UNSEEN_S, the car still held
  for a green (the hard rail); a red still read keeps LIGHT_RED_MAX_S (the long-wait test)."""
  def read(t):
    return 'red' if t < 55. else after
  sim = camera_signal_sim(read) if mapped else unmapped_camera_sim(read)
  plans = run_objects(sim, 120.)
  lost = int(round(55. / DT_MDL))
  if mapped:
    # drive 29 (2026-11-08): a mapped signal's own stop holds the car at its line on the red (the left's hold is in the
    # junction's mouth beyond it): its lost read said as the through signal's own, heard once, the car held for a green
    held = next(i for i, p in enumerate(plans) if p.shouldStop and sim.trace[i][1] < 0.05)
    spells = heard_spells(selfdrived_alerts(plans), held)
    assert len(spells) == 1 and 0. < (spells[0][0] - lost) * DT_MDL < SIGNAL_UNSEEN_S + 2., spells
    assert plans[-1].reason == SIGNAL_UNSEEN_REASON and str(plans[-1].mapControl.kind) == 'signal'
    assert all(p.shouldStop for p in plans[held:]) and sim.s < LINE + 1.
    assert not any(p.desire == Desire.turnLeft or str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
    return
  held = next(i for i, p in enumerate(plans) if p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap')
  assert held < lost
  shown = selfdrived_alerts(plans)
  spells = heard_spells(shown, held)
  assert len(spells) == 1 and 0. < (spells[0][0] - lost) * DT_MDL < SIGNAL_UNSEEN_S + 2., spells
  onset = next(f for f in shown if f[0] >= lost and f[3] != 'none')
  assert onset[1] == SIGNAL_UNSEEN_REASON, onset
  assert plans[lost + int(round(5. / DT_MDL))].reason == LIGHT_WAIT_REASON['red']  # the words change at SIGNAL_UNSEEN_S...
  assert plans[-1].reason == SIGNAL_UNSEEN_REASON  # ...and stay while it is not read
  assert all(p.shouldStop for p in plans[held:]) and sim.s < LINE + 1.
  assert not any(p.desire == Desire.turnLeft or str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def crept_at_a_light(before):
  """camera_signal_sim with an opposing left-turner standing 6 m past the junction (an occluder: the car creeps 3.8 m for the
  view) until 57 s; the light read as before until 50 s (None: nothing read), yellow to 54 s, red to 130 s, then green.
  Returns (sim, its occluder's objects, plans to 50 s)."""
  yellow_at, red_at, green_at, occ_leaves = 50., 54., 130., 57.

  def light_at(t):
    return before if t < yellow_at else 'yellow' if t < red_at else 'red' if t < green_at else 'green'
  sim = camera_signal_sim(light_at)
  frame = [0]

  def raw(sim):
    frame[0] += 1
    if frame[0] % 2:
      return None
    return RawFrame([raw_box(JUNCTION + 6. - sim.s, 3.6)] if sim.t < occ_leaves else [], 'road', sim.t)
  sim.raw = with_wide(raw)

  def occluder(sim):
    return ({'cls': 'car', 'x': JUNCTION + 6. - sim.s, 'y': 3.6, 'vx': 0., 'vy': 0., 'age': 40, 'trackId': 7, 'conf': 0.8,
             'camera': 'road'},) if sim.t < occ_leaves else ()
  return sim, occluder, run_objects(sim, yellow_at, occluder)


@pytest.mark.parametrize('before', [None, 'unknown'])
def test_a_red_read_with_the_car_crept_into_the_junction_on_an_unread_light_clears_it_on_the_gap(before):
  """Review 2026-11-07 (LEFTS-3): LS-4's cue fired only after a green was read at the line, but a mapped signal with no colour
  read is 'none' to the left (turn_light), so it creeps for a view on an unread light, at night the norm. The yellow, then
  the red, read with the nose 3.3 m from the junction's centre: "Waiting for the light", nothing heard, the car stood in the
  near cross lanes through their green. LEFTS-3 said it and kept the hold. Kellen, 2026-11-08: it clears the junction on
  the gap, "Red light: clearing the junction" (no change was read) heard once, the turner's view held until it leaves."""
  sim, occluder, plans = crept_at_a_light(before)
  assert sim.s > LINE + 2.  # crept for the view on the unread light
  clearing_go(sim, 50., LIGHT_IN_JUNCTION_REASON, 57., occluder, seconds=25.)


def test_a_red_read_with_the_car_standing_at_its_line_is_no_hold_in_the_junction():
  """LEFTS-3's key is the car past its line (the nearer of its first latch and LINE_D), never the metres since the latch: a
  car held at its line when the red comes (nothing crept) is waiting for the light, not holding in the junction, and heard
  only at the long wait's bound."""
  # at a junction with no signal mapped the hold is at its line (a mapped one's is in the junction's mouth: below)
  sim = unmapped_camera_sim(lambda t: None if t < 40. else 'red')
  sim.raw = road_runs(lambda sim: ((JUNCTION + 60. + V_25 * (3. * (int(sim.t / 3.) + 1) - sim.t), 3.6),) if sim.t < 50. else ())
  plans = run_objects(sim, 80.)
  held = plans[round(41. / DT_MDL):]
  assert sim.s < LINE + 1. and all(p.reason == LIGHT_WAIT_REASON['red'] for p in held), {p.reason for p in held}
  assert heard_spells(selfdrived_alerts(plans), round(40. / DT_MDL)) == []


def test_a_red_read_with_the_car_waiting_in_a_signalized_junctions_mouth_clears_the_junction_on_the_gap():
  """Drive 29 (2026-11-08): at a mapped signal the left waits for its gap in the junction's mouth (SIGNAL_LEFT_HOLD_M), past
  the signal's line: a red read there (no green read before it: the car's night) is a red with the car in the junction.
  Until 2026-11-08 held where it stood (LS-4's words, heard once). Kellen: like a driver, clear the junction: the oncoming
  car every 3 s to 50 s holds it until the band is clear, then it goes, lifted over the model's red hold, said and heard
  once (LIGHT_IN_JUNCTION_REASON), no handback."""
  sim = camera_signal_sim(lambda t: None if t < 40. else 'red')
  run_objects(sim, 40.)
  assert sim.s > LINE + 1. and JUNCTION - sim.s > ONCOMING_ENTRY_M  # in the mouth, past the signal's line
  clearing_go(sim, 40., LIGHT_IN_JUNCTION_REASON, 50., seconds=25.)


def test_a_signal_held_with_its_light_unseen_is_heard_once_and_still_held():
  """Review 2026-11-07 (SF-4): a through signal on the car's light config (no accepted source, the camera add-only), its red
  read at the stand and then never read again (no green read: 0 greens in 667 reads on 11-06): held at the green it never
  read, "Light not seen: waiting for green" on TurnPilot's status line only, selfdrived silent and blank on every frame of a
  200 s run. Now its onset at SIGNAL_UNSEEN_S is heard once (signalUnseen), the car still held for a green, never handed
  back; a green read releases it."""
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(300., 'signal')])
  sim.go_taps, sim.tp.oracle_ok, sim.tp.camera_mode = False, False, LIGHT_MODE_DEFAULT
  none = messaging.new_message('turnpilotLights').as_reader().turnpilotLights
  read = {'state': 'red'}

  def lights(sim):
    if read['state'] is not None and int(round(sim.t / DT_MDL)) % 2 == 0:
      sim.sm.put('turnpilotDetections', light_detections(read['state'], 315. - sim.s, shadow=False))
    return none
  sim.lights = lights
  plans = []
  while not (plans and plans[-1].shouldStop and sim.v < 0.05):
    plans.append(sim.step(engaged=True))
    assert sim.t < 60.
  stood = len(plans)
  read['state'] = None
  plans += [sim.step(engaged=True) for _ in range(int(60. / DT_MDL))]
  assert all(p.shouldStop for p in plans[stood:]) and not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  shown = selfdrived_alerts(plans)
  spells = heard_spells(shown, stood)
  assert len(spells) == 1, spells
  onset = next(f for f in shown if f[0] >= stood and f[3] != 'none')
  assert onset[2] == SIGNAL_UNSEEN_REASON and (onset[0] - stood) * DT_MDL < SIGNAL_UNSEEN_S + 2.
  read['state'] = 'green'
  plans = [sim.step(engaged=True) for _ in range(int(5. / DT_MDL))]
  assert not plans[-1].shouldStop and sim.v > 1.


def test_a_left_the_driver_hasnt_signalled_waits_at_its_line_prompting_the_side_on_repeat_and_goes_once_lit():
  """IC 9-21-8-25 and LEFT_GO_NEEDS_SIGNAL: the left goes only with its side lit, and on this car (no vehicle-CAN tap) only
  the driver can light it. Until 2026-11-07 it handed back after LEFT_SIGNAL_WAIT_S dark at its line ('unsignalled', "No turn
  signal at the line: take control"), and from the line on selfdrived showed the left's own silent waitingForGap prompt over
  "Signal LEFT Now", so the side's chime stopped as the car stood. Now (Kellen: no give-up handbacks; the driver prompted per
  side with a repeating chime at any speed) the car waits at its line for as long as it is dark, "Signal LEFT Now" and its
  own chime up the whole wait, never a take-control and never a go unsignalled; lit at last, it goes on its gap."""
  sim = left_sim()
  sim.answers_prompt = False  # the driver never signals
  plans = []
  while sim.t < 120.:
    plans += drive(sim, DT_MDL)
  held = next(i for i, p in enumerate(plans) if p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap')
  wait = plans[held:]
  assert len(wait) * DT_MDL > 80. and all(p.shouldStop and str(p.unprotectedLeft.state) == 'waitGap' for p in wait)
  assert all(str(p.turnSignal.prompt) == 'left' and p.reason == "Unprotected left: waiting for the turn signal" for p in wait)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or p.desire == Desire.turnLeft for p in plans)
  shown = [s for s in selfdrived_alerts(plans) if s[0] >= held]
  assert all(s[1] == "Signal LEFT Now" and s[3] == 'tpSignalLeft' for s in shown), sorted({s[1:] for s in shown})
  # the driver lights it: the window from the lamp, then the go
  sim.answers_prompt = True
  after = []
  while sim.s < JUNCTION + 10. and sim.t < 180.:
    after += drive(sim, DT_MDL)
  assert any(p.desire == Desire.turnLeft for p in after) and sim.s > JUNCTION + 10.
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in after)


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


def test_route_times_reach_refuses_nothing_and_the_line_judges_from_where_the_car_stands():
  """BUILD_FINDINGS D4 (smoke S08, diag s02_nomove_handback): route time checked R_req at LINE_D with no margin (134 m at 25
  mph, one lane, under a 150 m range), and the live check at the line, from where the car stands and the furthest reading
  (S08: d_far 23.6 m, 204 m), ended the left undecided there. Route time then took LINE_D plus ROUTE_REACH_MARGIN_M (200 m)
  and skipped the left out loud (unprotectedRange) under a 150 m range: on the 11-06 night that margin alone refused the two
  lefts the line could judge (1e L1: 267.8 m at route time, 182.1 m at the line, under 210 m). Since 2026-11-06 route time
  refuses nothing: the left is driven to its line under the same 150 m range, held there, and judged from where it stands
  (134 m at LINE_D: in range), so it goes on the empty band."""
  from openpilot.selfdrive.turnpilot.route_follower import UNCERTAIN_M
  from openpilot.selfdrive.turnpilot.unprotected_left import ROUTE_REACH_MARGIN_M
  from openpilot.selfdrive.turnpilot.unprotected_left import LINE_D, reach_at_line
  assert ROUTE_REACH_MARGIN_M == UNCERTAIN_M
  route_reach = reach_at_line(V_25, 1, ROUTE_REACH_MARGIN_M)
  assert reach_at_line(V_25, 1) < 150. < route_reach < TEST_RANGE_M, (reach_at_line(V_25, 1), route_reach)
  assert math.isclose(route_reach, r_req(V_25, t_req(LINE_D + UNCERTAIN_M, 1), LINE_D + UNCERTAIN_M))
  for range_m in (150., TEST_RANGE_M):
    sim, plans = route_time_left(range_m)
    assert not any(p.tooFastToTurn or str(p.unprotectedLeft.state) == 'skipped' for p in plans), range_m
    assert any(p.shouldStop and p.mapControl.kind == 'unprotectedTurn' for p in plans), "attempted: held at its line"
    plans += drive(sim, 25.)
    go = next(i for i, p in enumerate(plans) if p.desire == Desire.turnLeft)
    assert str(plans[go].unprotectedLeft.state) == 'go' and plans[go].unprotectedLeft.rReq <= range_m, range_m
    assert not any(p.tooFastToTurn or str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans) and sim.s > JUNCTION + 10.


def test_a_mapped_stop_lines_setback_never_skips_the_left_and_it_is_judged_at_the_line():
  # a mapped stop line at the junction (a signal's, its line 10 m before it: 1 m back of the hold line; or 20 m back, 11 m) is
  # where the car may stand: route time's R_req there (138 m, 182 m) was its refusal under a 150 m range for the 20 m one.
  # Neither is skipped now: each is driven to its line and judged there, never skipped out loud nor rerouted; its light unread,
  # it goes on the clear band (2026-11-07: no handbacks)
  for line_m in (10., 20.):
    sim, plans = route_time_left(150., controls=[(JUNCTION - line_m, 'signal', True)])
    plans += drive(sim, 20., yaw=False)
    assert not any(p.tooFastToTurn or str(p.skipReason) or str(p.unprotectedLeft.state) == 'skipped' for p in plans), line_m
    assert any(p.desire == Desire.turnLeft for p in plans), line_m
    assert not any(str(p.unprotectedLeft.state) == 'undecided' for p in plans), line_m


def test_an_unmeasured_detector_range_attempts_no_left_blind():
  # skeptic G4 (blocker 6): TurnPilotLeftRangeM defaulted to 150 m, pixel arithmetic never measured on real comma frames
  # (CRITIQUE SEV2: even a 25 mph R_req of 134-141 m is past the uncropped detector at 6.5-8 px), so 25 mph lefts were
  # attempted on an unmeasured range. Unset, the range is unmeasured (0): every autonomous left is past it, never taken from its
  # line. Since 2026-11-06 it is driven to its line, never skipped and rerouted; since 2026-11-07 (Kellen: no handbacks) it is
  # gone from there on the band the camera covers, never handed back for the range. A drive sets the range from a measurement
  assert ROAD_RANGE_M == 0.
  sim = Sim(LEFT_25)
  assert sim.tp.left_range_m == ROAD_RANGE_M
  sim.tp.auto_left, sim.go_taps, sim.raw = 'auto', False, road_runs()
  sim.answers_prompt = True
  plans, reasons = [], set()
  while sim.s < JUNCTION + 30. and sim.t < 120.:
    plans += drive(sim, DT_MDL)
    reasons.add(sim.tp.skip.reason)
  assert not any(p.tooFastToTurn for p in plans) and 'unprotectedRange' not in reasons
  assert not any(str(p.unprotectedLeft.state) in ('skipped', 'undecided') for p in plans)
  assert any(p.desire == Desire.turnLeft for p in plans) and sim.s > JUNCTION + 30.


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


def test_without_the_far_crop_a_45_mph_left_goes_on_the_band_it_covers_and_its_loss_there_is_loud():
  # the far crop off (no pkl, the param, off for the drive) or its range unmeasured: driven to its line (never skipped at route
  # time and rerouted since 2026-11-06) and, since 2026-11-07 (Kellen: no handbacks), gone from it on the band the full pass
  # covers, never handed back for what it can't see. Its range unmeasured with the far crop reading is its provisional
  # FAR_CROP_LEFT_RANGE_M since 2026-11-07 (turnpilotd.left_ranges), and the left still goes on an empty road
  for status, far_range in (('off', FAR_RANGE_M), ('disabled', FAR_RANGE_M), ('read', 0.)):
    sim = far_sim(status=status, far_range=far_range)
    plans = drive(sim, 3.)
    assert not plans[-1].tooFastToTurn and str(plans[-1].unprotectedLeft.state) == 'approach', status
    want = 0. if status != 'read' else (far_range or FAR_CROP_LEFT_RANGE_M)
    assert plans[-1].unprotectedLeft.farRangeM == want and not plans[-1].unprotectedLeft.farNeeded
    while sim.s < JUNCTION + 30. and sim.t < 120.:
      plans += drive(sim, DT_MDL)
    assert any(p.desire == Desire.turnLeft for p in plans), status
    assert not any(p.tooFastToTurn or str(p.unprotectedLeft.state) == 'undecided' for p in plans), status
  # the full pass's range unmeasured: the far crop extends nothing (it covers the band only past that range): not skipped either
  sim = far_sim()
  sim.tp.left_range_m = 0.
  assert not drive(sim, 3.)[-1].tooFastToTurn
  # on but waiting for room or calibration at route time (a spell): not skipped, judged at the line
  for status in ('noRoom', 'noCalibration'):
    sim = far_sim(status=status)
    assert not any(p.tooFastToTurn for p in drive(sim, 3.)), status
  # it goes off for the drive while the car waits at its line for its cars: said once (farViewOff), and the left, judged from
  # then on on the full pass's band, never hands back
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
  # ...and shown: selfdrived raises it over the left's silent waitingForGap for its whole time, with its chime (review
  # 2026-11-07, SF-3: its time ran down under that prompt and 'Far View Off' was never shown at the line)
  from openpilot.common.realtime import DT_CTRL
  from openpilot.selfdrive.turnpilot.turnpilotd import FAR_OFF_ALERT_S
  shown = [f for f in selfdrived_alerts(plans) if f[1] == 'Far View Off']
  assert abs(len(shown) * DT_CTRL - FAR_OFF_ALERT_S) < 0.2 and shown[0][3] == 'prompt', len(shown) * DT_CTRL
  assert all(str(plans[f[0]].controlPrompt) == 'waitingForGap' for f in shown)
  # the far crop's cars held it at the line while it read them; never handed back, never skipped and rerouted from it
  before = plans[:len(plans) - len(after)]
  assert not any(p.desire == Desire.turnLeft for p in before) and any(p.unprotectedLeft.farBlockers for p in before)
  assert not any(str(p.unprotectedLeft.state) == 'undecided' or str(p.controlPrompt) == 'leftUndecidedTakeControl' for p in plans)
  assert not any(p.tooFastToTurn for p in plans)


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


def test_a_car_standing_far_up_the_road_is_said_and_holds_the_left_never_an_endless_silent_wait():
  # skeptic G6: a car parked on the oncoming side 280 m up the road, in the band, found only by the far crop (which blocks
  # at any range): the 45 mph left stood at its line for good, silently (waitingForGap). Until 2026-11-07, waiting on the far
  # crop's boxes alone FAR_BLOCKED_UNDECIDED_S was the farBlocked handback; now (Kellen: "no, no handbacks") the car the
  # detector sees holds the left and the line says what it waits for from there, held at its line within -2.0, never a go,
  # never a take-control
  from openpilot.selfdrive.turnpilot.unprotected_left import FAR_BLOCKED_UNDECIDED_S
  sim = far_sim()
  sim.raw, sim.raw_far = road_runs(), far_runs(lambda sim: [(JUNCTION + 280., 5.4)])
  plans = []
  while sim.t < 70. + FAR_BLOCKED_UNDECIDED_S:  # ~29 s to stand at the line
    plans += drive(sim, DT_MDL)
  states = [str(p.unprotectedLeft.state) for p in plans]
  stand = states.index('waitGap')
  said = next(i for i, p in enumerate(plans) if p.reason == LONG_WAIT_REASON['farBlocked'])
  assert FAR_BLOCKED_UNDECIDED_S - 0.1 <= (said - stand) * DT_MDL <= FAR_BLOCKED_UNDECIDED_S + 0.5
  assert all(p.controlPrompt == 'waitingForGap' and p.shouldStop for p in plans[stand + 5:])
  assert all(p.reason == LONG_WAIT_REASON['farBlocked'] for p in plans[said:])
  assert 'undecided' not in states and not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
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



def test_a_quicker_gas_crawl_short_of_the_reach_re_engaged_rolling_is_held_on_the_latchs_own_stop():
  """Skeptic round 1 (tp/left-hold), through turnpilotd: held at the left's line, the driver's gas (disengaged) takes the car
  1.8 m on at up to 1.5 m/s, short of the latch's reach, and they re-engage rolling. The latch's own stop carries it about
  0.7 m further, engaged: TurnPilot's metres (turnpilotd hands map_controls openpilot's engaged), never the driver's, so
  the latch holds the car short of the oncoming lanes, the left waiting for its gap (c), never let go on its own braking."""
  sim, after, past = gas_crawl_at_the_line(1.8, v_crawl=1.5)
  lift_v = sim.trace[-len(after) - 1][1]
  assert past < 2. and lift_v > 1.2, (past, lift_v)
  assert all(p.shouldStop for p in after[30:]) and sim.v < 0.05 and JUNCTION - sim.s > ONCOMING_ENTRY_M + 1.
  assert {str(p.unprotectedLeft.state) for p in after[30:]} == {'waitGap'} and sim.tp.map_controls.driven is None
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


def late_withdrawn_go(v_w):
  """LEFT_25 at its line through four oncoming cars 6 s apart; the go certified, and later in its launch, at about v_w - 0.35
  m/s, a car 90 m up the oncoming lane at the limit withdraws it while the car can still stop short of the oncoming lanes:
  the left's own hold (hold_here) from there, TurnPilot's stop asking about v^2 / (2 STOP_DECEL) m on (skeptic round 1)."""
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
    sim.step(engaged=True)
    assert sim.t < 120.
  while sim.v < v_w - 0.35:
    sim.step(engaged=True)
  late['s'], late['t'] = sim.s + 90., sim.t
  while sim.tp.left_now.state == 'go':
    sim.step(engaged=True)
  assert sim.tp.left.hold_here and sim.v > 1. and JUNCTION - sim.s > ONCOMING_ENTRY_M + sim.tp.left.hold_asked
  return sim


@pytest.mark.parametrize("gas", [True, False])
@pytest.mark.parametrize("frac", [0.6, 0.9])
@pytest.mark.parametrize("v_w", [1.8, 2.3, 2.7])
def test_after_a_go_withdrawn_in_its_launch_a_re_engage_is_held_short_of_the_lanes_or_let_go_at_it(v_w, frac, gas):
  """Skeptic round 1 (tp/left-hold): the go withdrawn later in its launch, the driver (openpilot disengaged: the gas, or a
  cancel and a coast) keeps the car's speed over frac of the stop TurnPilot asked, then re-engages rolling. The left's own
  hold let go a frame or two into the re-engage, engaged, on TurnPilot's own braking metres: past = min(hold_m - asked,
  driver metres) took them for the driver's past the ask, and the lanes test at 0.5 m/s^2 for one the hold couldn't stop
  short. Openpilot then drove the car on into the junction under only the take-control, the full check (c) skipped, where
  the hold would have stood it 4.83-5.21 m from the centre (e93750b). At 2.7 e93750b stood it 3.47 and 2.83 m out, inside
  the lanes. Never let go on TurnPilot's metres: held short of the lanes, waiting for its gap, where the hold can stand the
  car there; let go at the re-engage itself, judged from its frame's speed, before any stop of TurnPilot's, where it can't
  (the invariant), its crossing said; never standing in the oncoming lanes."""
  from openpilot.selfdrive.turnpilot.map_controls import DRIVEN_THROUGH_M
  sim = late_withdrawn_go(v_w)
  asked = sim.tp.left.hold_asked
  s0, rows = sim.s, []  # (openpilot engaged, the left's own hold still standing after it, the plan)
  sim.accel = lambda sim, plan: 0.
  while sim.s - s0 < frac * asked - 1e-3 and sim.tp.left.hold_here:
    p = sim.step(engaged=False, gas=gas)
    rows.append((False, sim.tp.left.hold_here, p))
  sim.accel = None
  n = len(sim.trace)
  for _ in range(int(8. / DT_MDL)):
    p = sim.step(engaged=True)
    rows.append((True, sim.tp.left.hold_here, p))
  after = sim.trace[n:]
  stood = [JUNCTION - s for s, v, _ in after if v < 0.05]
  assert not [d for d in stood if abs(d) < ONCOMING_ENTRY_M], f"held standing in the oncoming lanes: {stood[:1]}"
  let_go = next((k for k, (_, here, _) in enumerate(rows) if not here), None)
  if v_w > 2.5:  # from where no hold can stand it short of the lanes: the driver's, at the re-engage
    reengage = next(k for k, (engaged, _, _) in enumerate(rows) if engaged)
    assert let_go == reengage, f"let go on row {let_go}, the re-engage {reengage}"
    assert not any(p.shouldStop for e, _, p in rows if e), "let go on TurnPilot's own braking"
    assert not any(p.desire == Desire.turnLeft for _, _, p in rows[let_go:]), "the go's turnLeft out on a left let go as the driver's"
    assert crossing_said(after, LINE + DRIVEN_THROUGH_M - 1.), sorted({(str(p.controlPrompt), p.reason) for *_, p in after})
  else:  # held where it stands, short of the lanes, the full check going on
    assert let_go is None, f"let go {'engaged' if rows[let_go][0] else 'disengaged'}: the full check (c) skipped"
    assert stood and stood[0] > ONCOMING_ENTRY_M and sim.v < 0.05
    plans = [p for e, _, p in rows if e]
    assert all(str(p.unprotectedLeft.state) == 'waitGap' for p in plans[:int(3. / DT_MDL)])
    assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


@pytest.mark.parametrize("frac", [0.3, 0.6, 0.9])
@pytest.mark.parametrize("v_w", [2.5, 2.7])
def test_after_a_go_withdrawn_in_its_launch_a_stop_short_of_the_lanes_and_a_re_engage_standing_is_held(v_w, frac):
  """Skeptic round 2 (tp/left-hold), through turnpilotd: the go withdrawn later in its launch (a car 90 m up the oncoming lane),
  the driver presses the gas (disengaged), keeps the speed over frac of the stop TurnPilot asked, brakes the car to a stop
  short of the oncoming lanes and re-engages standing. The lanes test, judged on the driver's frame from the speed then, had
  let the left's own hold go for good: the car pulled away from rest 19 m through the junction, 'driven', no stop, the go's
  turnLeft out on every frame (at 2.7 and 0.3 the gas lasted 0.24 s and the car stood 5.37 m from the centre, short of
  TurnPilot's own stop point). e93750b held it where it stood. Owner decision (c), the invariant: judged at the re-engage,
  standing short of the lanes: held there, waiting for its gap (or the position handback), never pulled away uncertified."""
  sim = late_withdrawn_go(v_w)
  asked = sim.tp.left.hold_asked
  s0 = sim.s
  sim.accel = lambda sim, plan: 0.
  while sim.s - s0 < frac * asked:
    sim.step(engaged=False, gas=True)
  sim.accel = lambda sim, plan: -2.5
  while sim.v > 0.:
    sim.step(engaged=False)
  at = JUNCTION - sim.s
  assert at > ONCOMING_ENTRY_M
  sim.accel = None
  plans = [sim.step(engaged=True) for _ in range(int(6. / DT_MDL))]
  moved = at - (JUNCTION - sim.s)
  states = sorted({(str(p.unprotectedLeft.state), str(p.unprotectedLeft.why)) for p in plans})
  assert moved < 0.05 and all(p.shouldStop for p in plans[5:]), f"re-engaged standing {at:.2f} m out: moved {moved:.2f} m, {states}"
  assert sim.tp.left.hold_here and sim.tp.map_controls.driven is None, states
  assert {s for s, _ in states} <= {'waitGap', 'undecided'} and ('undecided', 'driven') not in states, states
  assert not any(p.desire == Desire.turnLeft for p in plans)


def test_a_nudge_from_the_lefts_full_creep_braked_short_of_the_lanes_and_re_engaged_standing_is_held():
  """Skeptic round 2 (tp/left-hold), through turnpilotd: LEFT_25 creeping for a view (a car parked in the junction box
  blocks it), the full creep made, the car standing about 5 m from the centre. The driver's gas nudges it 0.35 m at up to
  1.2 m/s, they brake it to a stop 0.6 m short of the oncoming entry and re-engage standing: the latch's lanes test, judged
  on the gas frame from its speed, had let go for good, and the car drove off 14.7 m through the junction at 5.1 m/s, no
  stop on any frame (e93750b held it). Owner decision (c): judged at the re-engage, standing short of the lanes, held there
  for the full check, never driven on unchecked."""
  sim = Sim(LEFT_25)
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
  while not (sim.s > LINE + 3.5 and sim.v < 0.02 and str(sim.tp.left_now.state) == 'creep'):
    sim.step(objects(sim), engaged=True)
    assert sim.t < 60.
  for _ in range(10):
    sim.step(objects(sim), engaged=True)
  s0 = sim.s
  sim.accel = lambda sim, plan: 2.0 if sim.v < 1.2 else 0.
  while sim.s < s0 + 0.35:
    sim.step(objects(sim), engaged=False, gas=True)
  sim.accel = lambda sim, plan: -3.5
  while sim.v > 0.:
    sim.step(objects(sim), engaged=False)
  at = JUNCTION - sim.s
  assert at > ONCOMING_ENTRY_M + 0.4
  sim.accel = None
  plans = [sim.step(objects(sim), engaged=True) for _ in range(int(6. / DT_MDL))]
  moved = at - (JUNCTION - sim.s)
  states = sorted({(str(p.unprotectedLeft.state), str(p.unprotectedLeft.why)) for p in plans})
  assert moved < 0.05 and all(p.shouldStop for p in plans[5:]), f"re-engaged standing {at:.2f} m out: moved {moved:.2f} m, {states}"
  assert sim.tp.map_controls.driven is None and ('undecided', 'driven') not in states, states
  assert not any(p.desire == Desire.turnLeft for p in plans)


@pytest.mark.parametrize("then", ['rolling', 'stop_then_standing'])
@pytest.mark.parametrize("gas", [True, False])
@pytest.mark.parametrize("frac", [0.6, 0.9])
@pytest.mark.parametrize("v_w", [2.5, 2.7])
def test_after_a_go_withdrawn_in_its_launch_a_left_let_go_as_the_drivers_is_never_turned_on_its_go(v_w, frac, gas, then):
  """Skeptic round 2 (tp/left-hold): the go withdrawn later in its launch, the driver (disengaged: the gas, or a cancel and a
  coast) keeps the speed over frac of the stop TurnPilot asked, then re-engages rolling, or brakes to a stop short of the
  oncoming lanes and re-engages standing. Let go as the driver's ('driven': no stop, no go, no pulse), the go's turnLeft
  stayed out on the engaged frames (110 of 120 re-engaged rolling, 41 of them inside ONCOMING_ENTRY_M of the centre): with
  no hold, an uncertified left across the lanes the go was withdrawn for, which the model steers in the car (the sim has no
  yaw). Owner decision (b): a left the driver took with no certified go goes straight on and is missed; its pulse comes out
  of the model's queue (CLEAR_DESIRE) on the frame it is let go, never turnLeft after. Held instead (short of the lanes),
  no turnLeft standing either."""
  sim = late_withdrawn_go(v_w)
  asked = sim.tp.left.hold_asked
  s0 = sim.s
  sim.accel = lambda sim, plan: 0.
  while sim.s - s0 < frac * asked - 1e-3 and sim.tp.left.hold_here:
    sim.step(engaged=False, gas=gas)
  if then == 'stop_then_standing':
    sim.accel = lambda sim, plan: -2.5
    while sim.v > 0.:
      sim.step(engaged=False)
  sim.accel = None
  rows = []  # (the left's own hold standing after it, the plan)
  for _ in range(int(6. / DT_MDL)):
    p = sim.step(engaged=True)
    rows.append((sim.tp.left.hold_here, p))
  let_go = next((k for k, (here, _) in enumerate(rows) if not here), None)
  if let_go is not None:
    out = [k for k, (_, p) in enumerate(rows[let_go:]) if p.desire == Desire.turnLeft]
    assert not out, f"turnLeft on {len(out)} of {len(rows) - let_go} engaged frames after the left was let go as the driver's"
    assert rows[let_go][1].desire == CLEAR_DESIRE  # out of the model's queue on the frame it is let go
    assert any(p.turnMissed for _, p in rows) or JUNCTION - sim.s > -2., "driven straight through, never missed"
  standing = [p for (_, p), (s, v, _) in zip(rows, sim.trace[-len(rows):], strict=True) if v < 0.05]
  assert not any(p.desire == Desire.turnLeft for p in standing)


@pytest.mark.parametrize("lat_active", [False, True])
@pytest.mark.parametrize("deg", [10., 13.])
def test_a_left_let_go_as_the_drivers_short_of_turn_started_deg_has_its_pulse_cleared(deg, lat_active):
  """The other side of TURN_STARTED_DEG (tp/left-hold's recheck, round 1, follow-up 1): the driver turns into the left from
  the left's own hold (under the gas, disengaged) and it is let go as theirs 10 to 13 deg round, under TURN_STARTED_DEG.
  The withdrawn go's turnLeft comes out of the model's queue on the let-go frame (CLEAR_DESIRE) and never goes out again,
  re-engaged too: the left is the driver's, never an uncertified one on the go's pulse. A clear only under 5 deg passed
  every other test."""
  from openpilot.selfdrive.turnpilot.desire_timing import TURN_STARTED_DEG
  sim = withdrawn_go()
  sim.accel = lambda sim, plan: 0.6 if sim.v < 1.5 else 0.
  radius = 2.03 / math.radians(deg)  # the reach lets the hold go about 2 m on: deg turned by then
  let_go = None
  for _ in range(int(10. / DT_MDL)):
    p = sim.step(engaged=False, gas=True, yaw_rate=sim.v / radius, lat_active=lat_active)
    if not sim.tp.left.hold_here:
      let_go = p
      break
  assert let_go is not None and str(let_go.unprotectedLeft.why) == 'driven'
  turned = sim.tp.desires.turned_deg
  assert 7. < turned < TURN_STARTED_DEG, turned
  assert let_go.desire == CLEAR_DESIRE
  plans = [sim.step(engaged=False, gas=True, lat_active=lat_active) for _ in range(int(0.5 / DT_MDL))]
  sim.accel = None
  plans += [sim.step(engaged=True) for _ in range(int(8. / DT_MDL))]
  assert not any(p.desire == Desire.turnLeft for p in plans)


def test_a_left_the_driver_has_turned_into_when_it_is_let_go_keeps_its_desire_to_finish():
  """turnpilotd.clear_driven_left takes a withdrawn go's pulse out of the model's queue only under TURN_STARTED_DEG into the
  left: the driver turning the left themselves from the left's own hold (under the gas, disengaged, a 6 m radius), let go
  as theirs about 20 deg round, is a turn under way. Its desire is never cleared there (a re-engage then finishes the turn,
  as one the driver started on re-engaging always does), the crossing said all the same."""
  from openpilot.selfdrive.turnpilot.desire_timing import TURN_STARTED_DEG
  sim = withdrawn_go()
  sim.accel = lambda sim, plan: 0.5 if sim.v < 1.5 else 0.
  let_go = None
  for _ in range(int(10. / DT_MDL)):
    p = sim.step(engaged=False, gas=True, yaw_rate=sim.v / 6.)
    if not sim.tp.left.hold_here:
      let_go = p
      break
  assert let_go is not None and str(let_go.unprotectedLeft.why) == 'driven'
  d = sim.tp.desires
  assert d.started and d.turned_deg >= TURN_STARTED_DEG + 2., d.turned_deg
  assert not d.done and let_go.desire != CLEAR_DESIRE
  plans = [sim.step(engaged=False, gas=True, yaw_rate=sim.v / 6.) for _ in range(5)]
  assert not any(p.desire == CLEAR_DESIRE for p in plans) and not d.done
  assert str(plans[-1].controlPrompt) == 'unprotectedTurnTakeControl'


@pytest.mark.parametrize("v_w", [2.5, 2.7])
def test_a_go_withdrawn_in_its_launch_has_its_pulse_cleared_while_the_hold_brakes_and_the_next_go_pulses_afresh(v_w):
  """tp/left-hold's recheck, round 1, follow-up 2 (as at e93750b): the go withdrawn later in its launch with openpilot engaged
  throughout, the go's turnLeft stayed out on 40 of 44 (2.5) and 44 of 48 (2.7) of the frames the left's own hold braked the
  car to its stand, dropping only once it stood: the model steering the left the go was withdrawn for, its pulse queued. The
  pulse comes out of the model's queue on the withdrawal frame (CLEAR_DESIRE, one run of it), and no turnLeft goes while
  the car is held; the hold still stands the car short of the oncoming lanes, and the next go pulses a fresh edge and the
  turn is taken."""
  from openpilot.selfdrive.turnpilot.desire_timing import CLEAR_FRAMES
  sim = late_withdrawn_go(v_w)
  withdrawal = sim.trace[-1][2]
  assert withdrawal.desire == CLEAR_DESIRE, withdrawal.desire
  n = len(sim.trace)
  for _ in range(int(6. / DT_MDL)):
    sim.step(engaged=True)
  held = sim.trace[n:]
  rolling = [p for _, v, p in held if v > 0.05]
  assert rolling and not any(p.desire == Desire.turnLeft for p in rolling), \
    f"turnLeft on {sum(p.desire == Desire.turnLeft for p in rolling)} of {len(rolling)} rolling frames under the hold"
  assert not any(p.desire == Desire.turnLeft for _, _, p in held)
  clears = [k for k, p in enumerate([withdrawal] + [p for _, _, p in held]) if p.desire == CLEAR_DESIRE]
  assert clears == list(range(CLEAR_FRAMES)), clears  # once, from the withdrawal frame
  stood = [JUNCTION - s for s, v, _ in held if v < 0.05]
  assert stood and min(stood) > ONCOMING_ENTRY_M and sim.tp.left.hold_here and str(held[-1][2].unprotectedLeft.state) == 'waitGap'
  plans = drive(sim, 25.)
  first = next((k for k, p in enumerate(plans) if p.desire == Desire.turnLeft), None)
  assert first is not None and str(plans[first].unprotectedLeft.state) == 'go', "no fresh pulse at the next go"
  assert not any(p.desire == CLEAR_DESIRE for p in plans[:first])
  assert sim.s > JUNCTION + 10.


# --- drive 29 (2026-11-08): a left at a signal rolls up to the junction's mouth on the model and judges there ---

def mouth():
  from openpilot.selfdrive.turnpilot.map_controls import SIGNAL_LEFT_HOLD_M
  return JUNCTION - SIGNAL_LEFT_HOLD_M


@pytest.mark.parametrize('oracle', [True, False], ids=['green', 'unread'])
@pytest.mark.parametrize('dark', [False, True], ids=['day', 'night'])
def test_a_left_on_a_green_ball_is_never_held_at_the_line_and_goes_from_the_junctions_mouth(oracle, dark):
  """Drive 29 (cause 1): every left at a signal was held at a hold line 9 m short (SIGNAL_PROTECTS_LEFT False, the
  unprotected hold's stop there on a green too), the oncoming lanes judged from behind the stop line, and the driver drove
  every one. Now at a mapped signal on a green, read (an accepted source's) or not read at all (the car's night: no accepted
  source, its reader's greens rejected), the car rolls past the line to the junction's mouth (SIGNAL_LEFT_HOLD_M), judges
  the oncoming band there and goes when it is clear; never a stand at the line, never a take-control."""
  from openpilot.selfdrive.turnpilot.darkness import DarkFilter
  sim = signal_sim(red_until=0., oracle=oracle)
  sim.tp.dark_filter = DarkFilter(dark=dark)
  plans = run_objects(sim, 60.)
  go = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftGo')
  stands = [s for s, v, _ in sim.trace[:go] if v < 0.3 and s > JUNCTION - 40.]
  assert stands and min(stands) > LINE + 1.5, f"stood {JUNCTION - min(stands):.1f} m short of the junction"
  assert mouth() - 1.5 <= sim.trace[go][0] < JUNCTION - ONCOMING_ENTRY_M
  assert sim.s > JUNCTION + 10. and not any(p.turnMissed for p in plans)
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  assert not any(str(p.unprotectedLeft.state) == 'undecided' for p in plans)


def test_a_left_at_a_signal_waits_for_its_gap_in_the_junctions_mouth_and_goes_after_the_stream():
  """Drive 29 (cause 1): the oncoming band busy at a mapped signal, no light read (the car's config: no accepted source):
  the car waits in the junction's mouth, short of the oncoming lanes, never at the stop line, and goes once the stream ends."""
  sim = signal_sim(oracle=False)
  stream_until = 40.
  sim.raw = road_runs(lambda sim: ((JUNCTION + 60. + V_25 * (3. * (int(sim.t / 3.) + 1) - sim.t), 3.6),) if sim.t < stream_until else ())
  plans = run_objects(sim, 75.)
  go = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftGo')
  assert go * DT_MDL > stream_until
  waited = [s for s, v, p in sim.trace[:go] if v < 0.05 and str(p.unprotectedLeft.state) == 'waitGap']
  assert waited and all(mouth() - 1.5 <= s < JUNCTION - ONCOMING_ENTRY_M for s in waited)
  assert sim.s > JUNCTION + 10.
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def test_a_red_read_on_the_approach_to_a_signalized_left_stops_the_car_at_the_stop_line():
  """The hard rail at the mouth's hold: a red read (an accepted source's here) stops the car at the signal's own line, short
  of the junction, and the left never goes while it is read (its line the light's wait); on the green it rolls up to the
  mouth and goes."""
  sim = signal_sim(red_until=50.)
  plans = run_objects(sim, 80.)
  red = sim.trace[:round(50. / DT_MDL)]
  assert not any(p.desire == Desire.turnLeft or str(p.unprotectedLeft.state) in ('go', 'committed') for _, _, p in red)
  assert max(s for s, _, _ in red) < JUNCTION - 8. and red[-1][1] < 0.05
  assert sim.s > JUNCTION + 10.
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)


def test_a_red_read_too_late_for_the_signals_line_holds_the_left_in_the_mouth_never_overrun():
  """Drive 29 (cause 6): the camera's red first read with the car at the signal's line, rolling (483 s: 'stop' decided 0.2 m
  from the line at 4.3 m/s, infeasible, overrun) and the left's hold crossed as an overrun, 'Unprotected left not checked'.
  A red read with no room left at the line still has room at the left's hold in the mouth: the car stops there, short of the
  oncoming lanes, the left waiting for the light, never driven nor gone, and it goes on the green."""
  line = JUNCTION - 8.5
  light = {'state': 'green'}

  def lights(sim):
    if light["state"] == "green" and sim.s >= line - 0.3 and sim.t < 40.:
      light['state'] = 'red'
    if sim.t >= 40.:
      light['state'] = 'green'
    msg = messaging.new_message('turnpilotLights')
    msg.turnpilotLights.source = 'simOracle'
    msg.turnpilotLights.lights = [{'state': light['state'], 'distance': line - sim.s, 'instance': 'trafficLight 1'}]
    return msg.as_reader().turnpilotLights
  sim = signal_sim(oracle=True)
  sim.lights = lights
  plans = run_objects(sim, 70.)
  red_i = next(i for i, (s, _, _) in enumerate(sim.trace) if s >= line - 0.3)
  assert sim.trace[red_i][1] > 1.  # rolling when the red came
  assert any(str(p.mapControl.kind) == 'signal' and str(p.mapControl.action) == 'infeasible' for p in plans)  # too late for its line
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)  # never 'Unprotected left not checked'
  during = sim.trace[red_i:round(40. / DT_MDL)]
  stood = [s for s, v, _ in during if v < 0.05]
  assert stood and max(stood) < JUNCTION - ONCOMING_ENTRY_M
  assert not any(p.desire == Desire.turnLeft or str(p.unprotectedLeft.state) in ('go', 'committed', 'undecided')
                 for _, _, p in during)
  assert sim.s > JUNCTION + 10.


@pytest.mark.parametrize('tap_s, tracks', [(0.1, False), (0.3, False), (1.0, False), (0.3, True)])
def test_a_gas_tap_at_the_line_in_an_oncoming_stream_never_releases_the_left(tap_s, tracks):
  """The review of 2026-11-08 (probe_gas_tap, a regression pin: no misbehaviour found): waiting for its gap at the line in an
  unbroken oncoming stream, a gas tap of 0.1-1 s with openpilot engaged moves the car nowhere, releases nothing (no driver's
  release, no turn desire), and the left goes on waiting for its gap, short of the oncoming lanes."""
  stream = [JUNCTION + 60. + k * 3. * V_25 for k in range(40)]
  t0 = [None]

  def cars(sim):
    if t0[0] is None and JUNCTION - sim.s < 60.:
      t0[0] = sim.t
    dt = sim.t - t0[0] if t0[0] is not None else 0.
    return [(s - V_25 * dt, 3.6) for s in stream]
  sim = left_sim(cars)

  def objs():
    if not tracks:
      return ()
    return tuple({'cls': 'car', 'x': s - sim.s, 'y': y, 'vx': -V_25, 'vy': 0., 'age': 60, 'trackId': 50 + k, 'conf': 0.8,
                  'camera': 'road'} for k, (s, y) in enumerate(cars(sim)) if -5. < s - sim.s < 150.)
  while not (sim.tp.left_now is not None and sim.tp.left_now.state == 'waitGap' and sim.v < 0.05):
    sim.step(objs(), engaged=True)
    assert sim.t < 120.
  for _ in range(int(3. / DT_MDL)):
    sim.step(objs(), engaged=True)
  s0 = sim.s
  tap = [sim.step(objs(), engaged=True, gas=True) for _ in range(int(tap_s / DT_MDL))]
  assert sim.s - s0 < 0.05 and all(p.shouldStop for p in tap)
  plans = [sim.step(objs(), engaged=True) for _ in range(int(20. / DT_MDL))]
  assert sim.tp.scene.released_id is None and sim.tp.left.state == 'waitGap'
  assert JUNCTION - sim.s > ONCOMING_ENTRY_M and not any(p.desire == Desire.turnLeft for p in plans)
