"""turnpilotd driving the autonomous unprotected left (unprotected_left.py, DESIGN phase 4 G4) in closed loop: the synthetic
route, model and scene of test_turnpilotd, raw road camera runs of the oncoming cars, a point-mass car following the plan."""
import math

from openpilot.cereal import log, messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS
from openpilot.selfdrive.turnpilot.scene.tracker import RawBox, RawFrame
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M, ONCOMING_ENTRY_M
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of, nav_instruction
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
from openpilot.selfdrive.turnpilot.turn_speed import POST_GO_PROFILES
from openpilot.selfdrive.turnpilot.turnpilotd import GO_LINE_M, LEFT_NO_MOVE_S, TurnPilot, auto_left_for, left_range_for
from openpilot.selfdrive.turnpilot.unprotected_left import (CLEAR_WINDOW_S, CREEP_STALL_S, LIGHT_UNKNOWN_S, ROAD_RANGE_M, STALE_UNDECIDED_S,
                                                           r_req, t_req)

Desire = log.Desire
V_25 = 11.18
JUNCTION = 300.
# a left at JUNCTION across a 25 mph road with one lane each way, mapped (navd: maneuverOpposingSpeed, Lanes)
LEFT_25 = [(JUNCTION, "turn", "left", (0., 270., 2, 'present', V_25, 1)), (1500., "turn", "right", junction_of("right"))]
LINE = JUNCTION - HOLD_LINE_M - 1.  # the map's hold line (MAP_MARGIN_M short of the scene's)
# TurnPilotLeftRangeM as a drive that measured it would set it (the default is unmeasured, 0: no left is attempted)
TEST_RANGE_M = 150.


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
  sim = Sim([(JUNCTION, "turn", "left", (0., 270., 2, 'present', 20.1, 1)), (1500., "turn", "right", junction_of("right"))])
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', TEST_RANGE_M, False, road_runs()
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
  for junction in ((0., 270., 2, 'present', 20.1, 1), (0., 270., 2, 'present')):  # 45 mph, and unknown (taken as 45)
    sim = Sim([(600., "turn", "left", junction), (2000., "turn", "right", junction_of("right"))])
    sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', TEST_RANGE_M, False, road_runs()
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
  sim = Sim([(600., "turn", "left", (0., 270., 2, 'present', 20.1, 1)), (2000., "turn", "right", junction_of("right"))])
  sim.tp.auto_left, sim.tp.left_range_m, sim.go_taps, sim.raw = 'auto', 300., False, road_runs()
  plans = drive(sim, 5.)
  assert not any(p.tooFastToTurn for p in plans)
  # off: the line's handback, as G1's, never skipped for range
  sim = Sim([(600., "turn", "left", (0., 270., 2, 'present', 20.1, 1)), (2000., "turn", "right", junction_of("right"))])
  sim.tp.auto_left, sim.go_taps = 'off', False
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
  # car forever ("waiting for a gap", no chime, 130 s past the stream's end). The left keeps judging and goes after it
  for jump in (5.5, 6., 10.):
    sim, plans, last = jump_run(jump, 90.)
    states = [str(p.unprotectedLeft.state) for p in plans]
    go = next((i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftGo'), None)
    assert go is not None, (jump, sorted(set(states)))
    assert 'committed' not in states[:go] and go * DT_MDL > last, jump
    assert sim.s > JUNCTION + 10., jump
    assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans), jump
  # a reading further off than the follower's spread allows (UNCERTAIN_M; the reading after it then reads as a new maneuver
  # to the follower, the latch still holding the car at the old one's line): a fault, said for as long as it holds the car,
  # never a freeze
  sim, plans, _ = jump_run(20., 60.)
  first = next(i for i, p in enumerate(plans) if str(p.controlPrompt) == 'leftUndecidedTakeControl')
  assert plans[first].reason == "Junction position unsure: take control" and sim.s < LINE + 1.
  assert all(str(p.controlPrompt) == 'leftUndecidedTakeControl' and p.shouldStop for p in plans[first:])



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
  sim, plans, last = jump_run(6., 70., sim_cls=GoHeldSim)
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


def test_an_unmeasured_detector_range_attempts_no_left_blind():
  # skeptic G4 (blocker 6): TurnPilotLeftRangeM defaulted to 150 m, pixel arithmetic never measured on real comma frames
  # (CRITIQUE SEV2: even a 25 mph R_req of 134-141 m is past the uncropped detector at 6.5-8 px), so 25 mph lefts were
  # attempted on an unmeasured range. Unset, the range is unmeasured (0): every autonomous left is past it, skipped at route
  # time out loud and rerouted, never taken from its line. A drive sets it from a measurement (TEST_RANGE_M here)
  assert ROAD_RANGE_M == 0.
  sim = Sim(LEFT_25)
  assert sim.tp.left_range_m == ROAD_RANGE_M
  sim.tp.auto_left, sim.go_taps, sim.raw = 'auto', False, road_runs()
  plans, reasons = [], set()
  for _ in range(int(40. / DT_MDL)):
    plans += drive(sim, DT_MDL, yaw=False)
    reasons.add(sim.tp.skip.reason)
  assert any(p.tooFastToTurn for p in plans) and 'unprotectedRange' in reasons
  assert not any(p.shouldStop or p.desire != Desire.none for p in plans)
  assert all(str(p.unprotectedLeft.state) == 'skipped' for p in plans[2:] if p.maneuverDistance > 0. and str(p.maneuverModifier) == 'left')


# DESIGN phase 4 G5 (this run's G6): the far crop's runs, between the full pass's
V_45 = 20.1
LEFT_45 = [(JUNCTION, "turn", "left", (0., 270., 2, 'present', V_45, 1)), (1500., "turn", "right", junction_of("right"))]
FAR_RANGE_M = 300.  # TurnPilotLeftFarRangeM as a drive that measured it would set it (the default is unmeasured, 0)


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
