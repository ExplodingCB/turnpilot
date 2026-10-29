"""turnpilotd closed loop: synthetic navd, model and scene around a point-mass car that follows the plan's limits."""
import math
from unittest import mock

from opendbc.car.structs import car
import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import ABORT_HOLD_S, CLEAR_DESIRE, lane_change_events, plan_accel
from openpilot.selfdrive.turnpilot.lane_planner import LANE_UNKNOWN_REASON, LC_WARN_S, lane_change_window
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint
from openpilot.selfdrive.turnpilot.scene_policy import HOLD_LINE_M, NO_ACCEL_CAP, ONCOMING_TTA_S, PED_STOP_GAP_M
from openpilot.selfdrive.turnpilot.tests.synthetic import FakeNavd, car_control, car_state, junction_of, model_v2, nav_instruction, scene
from openpilot.selfdrive.turnpilot.desire_timing import TURN_DESIRE_LEAD_M
from openpilot.selfdrive.turnpilot.turn_skip import MIN_FIRST_PULSE_M
from openpilot.selfdrive.turnpilot.route_follower import Maneuver, classify
from openpilot.selfdrive.turnpilot.turn_speed import NO_SPEED_CAP, POST_GO_PROFILES, V_TURN_MIN
from openpilot.selfdrive.turnpilot.turnpilotd import (CAR_UNKNOWN_REASON, DETECTOR_DEADLINE_S, DETECTOR_START_DEADLINE_S, NO_LONG_REASON,
                                                       GAS_DISENGAGE_REASON, LEFT_NO_MOVE_S, NEAR_CONTROL_STAND_S, NO_MOVE_REASON, NO_MOVE_S,
                                                       SIGNAL_STOPPED_REASON, STALL_PROMPT_S, STALLED_REASON,
                                                       STOPPED_HANDBACK_S, STOPPED_LEAD_M, STOPPED_NOTE, STOPPED_NOTE_S, STOPPED_REASON,
                                                       TURN_MISSED_REASON, WAITING_REASON, TurnPilot, long_control_for)
from openpilot.selfdrive.turnpilot.handback import Handback
from openpilot.selfdrive.turnpilot.scene.tracker import STALE_S

State = custom.TurnpilotPlan.State
Fault = custom.TurnpilotPlan.Fault
LaneChange = custom.TurnpilotPlan.LaneChange
Desire = log.Desire
THREE_LANES_LEFT_TURN = ((("left",), True), (("straight",), False), (("straight", "right"), False))


TAKE_OVER_S = 1.  # s after TurnPilot hands back at an unprotected turn's hold line that the driver takes over
TAKE_OVER_ACCEL = 1.5  # m/s^2: the driver driving away from the hold, the plan not applied...
TAKE_OVER_M = 4.  # ...until this far on and faster than map_controls.COMMIT_SPEED (its hold released: driverWent),
TAKE_OVER_SPEED = 2.5  # then openpilot engaged again
# a left from a T's stem (no road ahead): no unprotected-turn stop (map_controls.py), for the tests of everything else
T_LEFT = junction_of("left", opposing='none')


class FakeSM:
  def __init__(self):
    self.data = {'carState': car_state(0.), 'carControl': car_control(), 'navInstruction': nav_instruction(0., valid=False),
                 'modelV2': model_v2()}
    self.updated = dict.fromkeys(self.data, False)
    self.valid = dict.fromkeys(self.data, True)
    self.alive = dict.fromkeys(self.data, True)

  def __getitem__(self, s):
    return self.data[s]

  def put(self, s, msg, valid=True):
    self.data[s], self.updated[s], self.valid[s] = msg, True, valid


class Sim:
  def __init__(self, maneuvers, cruise=13., nav_lanes=None, model_lanes=(0, 0), shadow=False, driver_signals=False, paint=(),
               paint_rule=True, long_control=True, controls=(), controls_status=None, disengage_on_accel=True):
    self.navd, self.tp, self.sm = FakeNavd(maneuvers, nav_lanes, controls=controls, controls_status=controls_status), TurnPilot(), FakeSM()
    # the hold line's handback at an unprotected left (G1's), for the tests here; the autonomous left (TurnPilotAutoLeft auto,
    # the default) is test_turnpilotd_left's
    self.tp.auto_left = 'off'
    self.raw = None  # the detector's raw boxes (scene.tracker.RawFrame) of this frame's run, a function of the sim, or None
    self.raw_far = None  # the far crop's run (RawFrame, view far) of this frame, a function of the sim, or None
    self.far_status = None  # turnpilotDetections.farStatus to send each frame (a full pass message), a function of the sim
    self.lights = None  # turnpilotLights to send each frame, a function of the sim
    self.paint, self.paint_rule, self.inputs_valid = paint, paint_rule, True
    self.s, self.v, self.cruise, self.t = 0., cruise, cruise, 0.
    self.model_lanes, self.shadow, self.driver_signals, self.long_control = model_lanes, shadow, driver_signals, long_control
    self.disengage_on_accel = disengage_on_accel
    self.big = False
    self.model_kw = {}  # synthetic.road() keywords for the model's lanes
    self.model_holds = False  # the model holds the car (stand()): its shouldStop, no motion planned; otherwise it drives on
    self.model = None  # (sim) -> the modelV2 to send instead of the synthetic one
    self.accel = None  # (sim, plan) -> the car's accel instead of following the plan's limits (a planner in the loop)
    self.model_running = True  # modeld publishing modelV2 (it doesn't while it loads the detector)
    self.nav_filter = None  # (sim, instruction) -> (instruction, valid): what navd sends instead
    # the driver takes over TAKE_OVER_S after TurnPilot stops the car at an unprotected turn and hands back (no gas go:
    # the press disengages, the driver drives through the hold line, and engages again): every left here without junction
    # data, or across a road ahead, is one. False: nobody takes over, for the tests of it
    self.go_taps = True
    self.take_over_on = ('unprotectedTurnTakeControl',)
    self._prompted_s = 0.
    self._take_over_from: float | None = None  # s where the driver's take-over began
    self.trace = []

  def step(self, objects=(), available=True, steer=0., blinker=False, lat_active=True, yaw_rate=0., gas=False,
           engaged=False):
    prompted = self.go_taps and self.trace and self.trace[-1][2].controlPrompt in self.take_over_on and self.v < 0.3
    self._prompted_s = self._prompted_s + DT_MDL if prompted else 0.
    if self._prompted_s >= TAKE_OVER_S and self._take_over_from is None:
      self._take_over_from, self._prompted_s = self.s, 0.
    driver = self._take_over_from is not None
    if driver and self.s - self._take_over_from > TAKE_OVER_M and self.v > TAKE_OVER_SPEED:
      self._take_over_from, driver = None, False
    gas = gas or driver
    self.sm.updated = dict.fromkeys(self.sm.data, False)
    instr = self.navd.update(self.s)
    if instr is not None:
      instr, valid = self.nav_filter(self, instr) if self.nav_filter is not None else (instr, True)
      self.sm.put('navInstruction', instr, valid)
    self.sm.put('carState', car_state(self.v, steer, left_blinker=blinker, yaw_rate=yaw_rate, gas_pressed=gas))
    self.sm.put('carControl', car_control(lat_active, engaged))
    if self.model_running:
      self.sm.put('modelV2', self.model(self) if self.model is not None else
                  model_v2(*self.model_lanes, big=self.big, holds=self.model_holds, **self.model_kw))
    if self.lights is not None:
      self.sm.put('turnpilotLights', self.lights(self))
    if self.far_status is not None:
      td = messaging.new_message('turnpilotDetections')
      td.turnpilotDetections.farStatus = self.far_status(self)
      self.sm.put('turnpilotDetections', td.as_reader().turnpilotDetections)
    plan = self.tp.plan(self.sm, scene(objects, available), True, self.shadow, self.driver_signals, paint=self.paint,
                        paint_rule=self.paint_rule, inputs_valid=self.inputs_valid, raw=self.raw(self) if self.raw is not None else None,
                        raw_far=self.raw_far(self) if self.raw_far is not None else None,
                        long_control=self.long_control, disengage_on_accel=self.disengage_on_accel).as_reader().turnpilotPlan
    a = min(plan_accel(plan.accelCap, plan.speedCap, self.v), (self.cruise - self.v) / 2., 1.5)
    if plan.shouldStop:
      a = min(a, -1.)
    if self.accel is not None:
      a = self.accel(self, plan)
    if driver:
      a = TAKE_OVER_ACCEL  # the driver's foot, openpilot disengaged: the plan isn't applied
    self.v = max(self.v + max(a, -3.5) * DT_MDL, 0.)
    self.s += self.v * DT_MDL
    self.t += DT_MDL
    self.trace.append((self.s, self.v, plan))
    return plan

  def distance(self):
    return self.tp.follower.distance

  def run(self, seconds, objects=lambda sim: (), turn_at=None):
    for _ in range(int(seconds / DT_MDL)):
      steer = 90. if turn_at is not None and abs(self.s - turn_at) < 8. else 0.
      self.step(objects(self), steer=steer)


def rising(trace, field, value):
  prev, edges = None, []
  for s, _, plan in trace:
    cur = getattr(plan, field)
    if cur == value and prev != value:
      edges.append(s)
    prev = cur
  return edges


def test_disabled_and_no_route():
  sm = FakeSM()
  plan = TurnPilot().update(sm, scene(available=False), False, False).as_reader().turnpilotPlan
  assert plan.state == State.disabled and plan.desire == Desire.none
  assert (plan.accelCap, plan.speedCap, plan.shouldStop) == (NO_ACCEL_CAP, NO_SPEED_CAP, False)
  sm.put('navInstruction', nav_instruction(100., valid=False), valid=False)
  plan = TurnPilot().update(sm, scene(), True, False).as_reader().turnpilotPlan
  assert plan.state == State.noRoute and not plan.routeValid and plan.handback == Handback.none


def test_turnpilot_faults_out_loud_while_a_gas_press_does_not_disengage():
  # skeptic G1: DisengageOnAccelerator off (a device that had it written off for the old gas go) and TurnPilot ran on, a gas
  # press while engaged not disengaging: the driver driving off a handed-back stop sign did it still engaged, silently.
  # It faults instead (turnpilotUnavailable), and a pedestrian in the path is still stopped for on the faulted plan
  x_ped = 80.
  sim = Sim([(300., "turn", "left")], cruise=10., disengage_on_accel=False)
  sim.run(20., objects=lambda sim: ({'cls': 'pedestrian', 'x': x_ped - sim.s, 'y': 0.1, 'inPath': True},))
  for _, _, plan in sim.trace:
    assert plan.state == State.disabled and plan.fault == Fault.disengageOnAccelerator and plan.reason == GAS_DISENGAGE_REASON
    assert plan.desire == Desire.none
  assert sim.v == 0. and 0. < x_ped - sim.s < PED_STOP_GAP_M + 2.
  # on: TurnPilot runs
  sim = Sim([(300., "turn", "left")], cruise=10.)
  sim.run(2.)
  assert sim.trace[-1][2].fault == Fault.none and sim.trace[-1][2].state != State.disabled


def test_nothing_without_openpilot_longitudinal():
  # stock ACC holds the speed: the caps would do nothing, and a turn desire would go out at road speed
  sim = Sim([(300., "turn", "left"), (1500., "turn", "right")], cruise=15., nav_lanes={0: THREE_LANES_LEFT_TURN},
            model_lanes=(2, 0), long_control=False)
  sim.run(30., turn_at=300.)
  assert sim.s > 320.
  for _, _, plan in sim.trace:
    assert plan.state == State.disabled and plan.reason == NO_LONG_REASON and plan.fault == Fault.noLongitudinal
    assert (plan.desire, plan.laneChange, plan.signalPrompt) == (Desire.none, LaneChange.none, LaneChange.none)
    assert (plan.accelCap, plan.speedCap, plan.shouldStop) == (NO_ACCEL_CAP, NO_SPEED_CAP, False)


def test_left_turn_drive():
  sim = Sim([(300., "turn", "left"), (1500., "turn", "right")])
  sim.run(45., turn_at=300.)
  states = [str(plan.state) for _, _, plan in sim.trace]
  assert states[0] == 'following' and 'approaching' in states and 'turning' in states
  assert 1 <= len(rising(sim.trace, 'desire', Desire.turnLeft)) <= 4  # repeats until turned; this sim has no yaw rate
  at_desire = [v for s, v, _ in sim.trace if 300. - TURN_DESIRE_LEAD_M < s < 300.]  # gate 1's speed from the pulse on
  assert max(at_desire) < V_TURN_MIN + 0.3
  after = [plan for s, _, plan in sim.trace if s > 320.]
  assert after and all(plan.maneuverModifier == "right" for plan in after)
  assert all(plan.desire == Desire.none for plan in after)


def test_consecutive_turns_drive():
  sim = Sim([(300., "turn", "right"), (420., "turn", "left"), (2000., "turn", "right")])
  for _ in range(int(60. / DT_MDL)):
    near = [m for m in (300., 420.) if abs(sim.s - m) < 8.]
    sim.step(available=True, steer=90. if near else 0.)
  # a pulse each, and more on the REPULSE_S clock while the car hasn't turned (this sim has no yaw rate)
  assert 1 <= len(rising(sim.trace, 'desire', Desire.turnRight)) <= 4
  assert 1 <= len(rising(sim.trace, 'desire', Desire.turnLeft)) <= 4
  assert sim.s > 440.


def test_highway_bends_are_left_alone():
  # "continue slight right" and "new name slight left" at 30 m/s: no keep desire, no braking
  sim = Sim([(400., "continue", "slight right"), (900., "new name", "slight left"), (5000., "turn", "right")], cruise=30.)
  sim.run(40.)
  assert sim.s > 1000. and min(v for _, v, _ in sim.trace) > 29.9
  assert all(plan.desire == Desire.none and plan.speedCap == NO_SPEED_CAP for _, _, plan in sim.trace)


def test_turn_desire_waits_for_openpilot_to_steer():
  # the driver engages 30 m before the left: no desire before, a fresh pulse then
  sim = Sim([(300., "turn", "left"), (1500., "turn", "right")])
  for _ in range(int(45. / DT_MDL)):
    sim.step(lat_active=sim.s > 270.)
  edges = rising(sim.trace, 'desire', Desire.turnLeft)
  assert edges and 270. < edges[0] < 271.
  assert all(plan.desire == Desire.none for s, _, plan in sim.trace if s < 270.)
  assert not any(plan.tooFastToTurn for _, _, plan in sim.trace)
  # engaged 20 m before it, too late for a first pulse to be taken: skipped out loud. Driving it with openpilot off
  # isn't skipped under the driver
  for engage in (280., None):
    sim = Sim([(300., "turn", "left"), (1500., "turn", "right")])
    for _ in range(int(45. / DT_MDL)):
      sim.step(lat_active=engage is not None and sim.s > engage)
    assert all(plan.desire == Desire.none for _, _, plan in sim.trace), engage
    assert any(plan.tooFastToTurn for _, _, plan in sim.trace) == (engage is not None), engage


def test_destination_change_keeps_a_committed_turn():
  # a new destination from the phone 25 m before a left whose desire is out, its route starting with a left 284 m on
  # (navd's next route, routeCount 1): a committed turn is never cancelled. The old left is held to its end, its desire
  # never cleared and its cap kept until the car is past it, then the new route is taken up and its left pulses at its lead
  sim = Sim([(300., "turn", "left"), (1500., "turn", "right")])
  switch = None
  for _ in range(int(80. / DT_MDL)):
    if switch is None and sim.distance() < 25.:
      switch = sim.s
      assert sim.trace[-1][2].desire == Desire.turnLeft
      sim.navd = FakeNavd([(switch + 284., "turn", "left"), (switch + 1500., "turn", "right")], route_count=1)
    sim.step()
  after = [(s, v, plan) for s, v, plan in sim.trace if s > switch]
  before = [(v, plan) for s, v, plan in after if s < 300.]
  assert all(plan.desire != CLEAR_DESIRE and plan.speedCap < 6. for _, plan in before)
  # never speeding up into it, short of pulling away from its unprotected stop at the hold line on the driver's go
  stopped = next(i for i, (v, plan) in enumerate(before) if v < 0.05)
  assert before[stopped][1].mapControl.kind == 'unprotectedTurn'
  assert all(b[0] <= a[0] + 1e-6 for a, b in zip(before[:stopped], before[1:stopped], strict=False))
  assert all(plan.desire != Desire.turnLeft for s, _, plan in after if s > 316. and plan.maneuverDistance > TURN_DESIRE_LEAD_M)
  new_left = [s for s in rising(sim.trace, 'desire', Desire.turnLeft) if s > 316.]
  assert new_left and switch + 284. - new_left[0] > MIN_FIRST_PULSE_M and max(v for _, v, _ in after) > 12.
  # not yet committed (60 m out, no desire out): the old left is dropped at once, and the car speeds up again
  sim = Sim([(300., "turn", "left"), (1500., "turn", "right")])
  sim.cruise = 13.
  while sim.distance() > 60. or sim.s < 10.:
    sim.step()
  switch = sim.s
  sim.navd = FakeNavd([(switch + 284., "turn", "left"), (switch + 1500., "turn", "right")], route_count=1)
  for _ in range(int(10. / DT_MDL)):
    sim.step()
  assert not any(p.desire == Desire.turnLeft for s, _, p in sim.trace if switch < s < switch + 200.)
  assert sim.v > 12.


def test_oncoming_car_holds_unprotected_left():
  spawn = {}

  def oncoming(sim):
    if 'x' not in spawn and sim.distance() < 40.:
      spawn['x'], spawn['t'] = 300. + 72., sim.t  # reaches the junction 8 s later, just as the car would
    if 'x' not in spawn:
      return ()
    x_world = spawn['x'] - 9. * (sim.t - spawn['t'])
    return ({'cls': 'car', 'x': x_world - sim.s, 'y': 5., 'vx': -9.},)

  # the scene's hold alone (no unprotected-turn stop at a T's stem): test_go_at_an_unprotected_left_* for one with it
  sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right")])
  sim.run(60., objects=oncoming, turn_at=300.)
  assert any(plan.reason == "waiting for oncoming" and plan.state == State.holding for _, _, plan in sim.trace)
  # the car stopped at the hold line while the oncoming car was less than 5 s out, then took the turn
  arrive = spawn['t'] + 72. / 9.
  for i, (s, _, _) in enumerate(sim.trace):
    t = (i + 1) * DT_MDL
    if arrive - ONCOMING_TTA_S < t < arrive - 1.:
      assert s < 300. - HOLD_LINE_M + 1.5
  assert any(v < 0.1 and plan.shouldStop for _, v, plan in sim.trace)
  assert sim.s > 320.


def test_pedestrian_in_path_stops_then_goes():
  def pedestrian(sim):
    return ({'cls': 'pedestrian', 'x': 200. - sim.s, 'y': 0.3, 'inPath': True},) if sim.t < 30. else ()

  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")])
  sim.run(25., objects=pedestrian)
  assert sim.v < 0.1 and 200. - PED_STOP_GAP_M - 2. < sim.s < 200.
  assert sim.trace[-1][2].reason == "pedestrian in path" and sim.trace[-1][2].shouldStop
  sim.run(10., objects=pedestrian)
  assert sim.s > 205.


BW, SW = LinePaint('broken', 'white', 0.9, 0.3), LinePaint('solid', 'white', 0.9, 0.95)
BROKEN_WHITE_LEFT = (UNKNOWN, BW, SW, UNKNOWN)


def test_lane_change_warned_then_requested():
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(2, 0),
            paint=BROKEN_WHITE_LEFT)
  sim.run(25.)
  offers = rising(sim.trace, 'laneChangeOffer', LaneChange.left)
  requests = [(s, plan) for s, _, plan in sim.trace if plan.laneChange == LaneChange.left]
  assert offers and requests and all(plan.state == State.laneChange for _, plan in requests)
  start, end = lane_change_window(15., 2)
  assert 500. - start < offers[0] < 500. - end
  assert abs((requests[0][0] - offers[0]) - LC_WARN_S * 15.) < 2.  # the warning's 3 s at 15 m/s
  counting = [plan.laneChangeCountdown for s, _, plan in sim.trace if offers[0] <= s < requests[0][0]]
  assert counting[0] > LC_WARN_S - 0.1 and counting[-1] < 0.1 and all(a >= b for a, b in zip(counting, counting[1:], strict=False))
  assert all(plan.laneChangeAbort == 'none' for _, _, plan in sim.trace)

  # the Tesla: the prompt and countdown, nothing until the driver's blinker
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(2, 0),
            driver_signals=True, paint=BROKEN_WHITE_LEFT)
  sim.run(10.)
  assert any(plan.signalPrompt == LaneChange.left and plan.laneChangeOffer == LaneChange.left and plan.state == State.laneChange
             for _, _, plan in sim.trace)
  assert all(plan.laneChange == LaneChange.none for _, _, plan in sim.trace)
  plan = sim.step(blinker=True)
  assert plan.laneChange == LaneChange.left


def test_no_paint_no_lane_change_and_aborts_reach_the_plan():
  # the map's lanes alone, with no paint read, no longer move the car
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(2, 0))
  sim.run(25.)
  assert all(plan.laneChange == LaneChange.none and plan.laneChangeOffer == LaneChange.none for _, _, plan in sim.trace)
  # paint lost under an offer: the plan says why, for selfdrived's alert
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., model_lanes=(1, 0), paint=BROKEN_WHITE_LEFT)
  while not sim.trace or sim.trace[-1][2].laneChangeOffer == LaneChange.none:
    sim.step()
  sim.paint = ()
  plan = sim.step()
  assert plan.laneChangeAbort == 'lineNotBroken' and plan.laneChangeOffer == LaneChange.none and plan.laneChange == LaneChange.none
  # inputs failing their checks abort too
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., model_lanes=(1, 0), paint=BROKEN_WHITE_LEFT)
  while not sim.trace or sim.trace[-1][2].laneChangeOffer == LaneChange.none:
    sim.step()
  sim.inputs_valid = False
  assert sim.step().laneChangeAbort == 'badInputs'


def test_lane_change_from_lane_paint():
  # no map lanes: the broken white line on the left lets the car over toward a left turn; none read, no change
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., model_lanes=(1, 0), paint=BROKEN_WHITE_LEFT)
  sim.run(25.)
  requests = [s for s, _, plan in sim.trace if plan.laneChange == LaneChange.left]
  start, end = lane_change_window(15.)
  assert requests and 500. - start < requests[0] < 500. - end
  assert all(plan.laneChange != LaneChange.right for _, _, plan in sim.trace)
  assert all(plan.laneChangePaintRule == plan.laneChange for _, _, plan in sim.trace)
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., model_lanes=(1, 0))
  sim.run(25.)
  assert all(plan.laneChange == LaneChange.none for _, _, plan in sim.trace)
  # TurnPilotLanePaintActive off: nothing acts or is offered, and laneChangePaintRule says what would have been asked for
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., model_lanes=(1, 0), paint=BROKEN_WHITE_LEFT, paint_rule=False)
  sim.run(25.)
  assert all(plan.laneChange == LaneChange.none and plan.laneChangeOffer == LaneChange.none for _, _, plan in sim.trace)
  assert any(plan.laneChangePaintRule == LaneChange.left for _, _, plan in sim.trace)



def test_lane_not_confirmed_is_said_not_changed():
  # one lane in from a curb lane with no edge line: the model sees room on the right but no line, so it can't say whether
  # the car is in the middle lane or the curb lane. No change from a lane it can't place: the plan says so instead
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(1, 0),
            paint=(UNKNOWN, BW, BW, UNKNOWN))
  sim.model_kw = {'shoulder': 4.}
  sim.run(25.)
  assert all(plan.laneChange == LaneChange.none and plan.laneChangeOffer == LaneChange.none for _, _, plan in sim.trace)
  unknown = [plan for _, _, plan in sim.trace if plan.laneMismatch == custom.TurnpilotPlan.LaneMismatch.laneUnknown]
  assert unknown and all(plan.reason == LANE_UNKNOWN_REASON for plan in unknown)
  # the curb lane's line read: the same road plans its changes
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(1, 0),
            paint=(UNKNOWN, BW, BW, UNKNOWN))
  sim.run(25.)
  assert any(plan.laneChange == LaneChange.left for _, _, plan in sim.trace)
  assert all(plan.laneMismatch == custom.TurnpilotPlan.LaneMismatch.none for _, _, plan in sim.trace)

def test_shadow_computes_the_same_plan():
  def oncoming(sim):
    return ({'cls': 'car', 'x': 330. - sim.s, 'y': 5., 'vx': -9.},) if sim.t > 18. else ()

  runs = []
  for shadow in (False, True):
    sim = Sim([(300., "turn", "left"), (1500., "turn", "right")], nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(1, 0), shadow=shadow)
    sim.run(40., objects=oncoming, turn_at=300.)
    runs.append(sim.trace)
  for (_, _, p0), (_, _, p1) in zip(*runs, strict=True):
    assert p1.shadow and not p0.shadow
    d0, d1 = p0.to_dict(), p1.to_dict()
    d0.pop('shadow', None)
    d1.pop('shadow', None)
    assert d0 == d1


class FakeParams(dict):
  def get(self, key):
    return super().get(key)


def test_long_control_from_car_params():
  assert long_control_for(FakeParams()) is None  # card hasn't written CarParams yet: not known
  for long_control in (False, True):
    cp = car.CarParams.new_message(openpilotLongitudinalControl=long_control)
    assert long_control_for(FakeParams(CarParams=cp.to_bytes())) == long_control


def test_unknown_longitudinal_is_disabled_without_a_fault():
  # every drive starts before card has written CarParams: no "TurnPilot unavailable" flash, and nothing acted on
  sim = Sim([(300., "turn", "left")], long_control=None)
  sim.run(2.)
  for _, _, plan in sim.trace:
    assert plan.state == State.disabled and plan.reason == CAR_UNKNOWN_REASON and plan.fault == Fault.none
    assert plan.desire == Desire.none and plan.speedCap == NO_SPEED_CAP


def test_enabled_plan_has_no_fault():
  sim = Sim([(300., "turn", "left")])
  sim.run(2.)
  assert all(plan.fault == Fault.none for _, _, plan in sim.trace)


def test_waits_for_the_detector_before_acting():
  # TurnPilot turned on with no detections yet (modeld loads the detector only while disengaged): no desire, no lane
  # change, no turn cap, and no handback either, since nothing has stopped
  sim = Sim([(150., "turn", "left"), (1500., "turn", "right")])
  for _ in range(int(DETECTOR_START_DEADLINE_S / DT_MDL) - 1):
    plan = sim.step(available=False)
    assert plan.state == State.unavailable and plan.reason == WAITING_REASON and plan.handback == Handback.none
    assert plan.desire == Desire.none and plan.laneChange == LaneChange.none and plan.speedCap == NO_SPEED_CAP
  assert sim.s > 170.  # drove past the turn, nothing held it
  plan = sim.step()  # first detections: acting
  assert plan.state != State.unavailable and plan.handback == Handback.none


def test_detector_stops_hand_back_without_a_hold():
  sim = Sim([(400., "turn", "left"), (1500., "turn", "right")])
  sim.run(5.)
  assert sim.trace[-1][2].handback == Handback.none
  # a slow spell: modeld's forced run comes within 1 s of the last detection, the scene unavailable from STALE_S: no handback
  for _ in range(int((1.0 - STALE_S) / DT_MDL)):
    assert sim.step(available=False).handback == Handback.none
  sim.step()
  # stopped: handed back once the deadline passes, with no desire, no lane change and no stop-and-wait at the turn
  plans = [sim.step(available=False) for _ in range(int(60. / DT_MDL))]
  first = next(i for i, p in enumerate(plans) if p.handback != Handback.none)
  assert abs((first + 1) * DT_MDL - (DETECTOR_DEADLINE_S - STALE_S)) <= 1.5 * DT_MDL
  assert all(p.handback == Handback.detectorStopped and p.state == State.unavailable for p in plans[first:])
  assert all(p.desire == Desire.none and p.laneChange == LaneChange.none and not p.shouldStop for p in plans)
  assert plans[-1].reason == "detector stopped: take control"
  assert sim.s > 420. and min(v for _, v, _ in sim.trace[-len(plans):]) > 5.
  # the detector comes back: acting again
  assert sim.step().handback == Handback.none and sim.trace[-1][2].state != State.unavailable


def test_handback_mid_turn_keeps_the_turn_cap_and_sends_no_new_pulse():
  # the turn desire went out and the car is slowing for the turn when the detector stops: the cap stays (a handback must
  # not speed the car up), the desire out is never cancelled, and no new pulse goes out
  sim = Sim([(300., "turn", "left"), (1500., "turn", "right")])
  while sim.distance() > 15.:
    sim.step()
  assert any(p.desire == Desire.turnLeft for _, _, p in sim.trace)
  capped = sim.trace[-1][2].speedCap
  assert capped < NO_SPEED_CAP
  n = len(sim.trace)
  plans = [sim.step(available=False) for _ in range(int(1.5 / DT_MDL))]
  assert plans[-1].handback == Handback.detectorStopped and plans[-1].speedCap <= capped + 1.
  first = next(i for i, p in enumerate(plans) if p.handback != Handback.none)
  assert all(s < sim.trace[n + first][0] for s in rising(sim.trace, 'desire', Desire.turnLeft))
  assert plans[-1].desire == Desire.turnLeft  # held, not cancelled
  assert all(p.desire in (Desire.none, Desire.turnLeft, CLEAR_DESIRE) for p in plans)


def test_handback_in_the_turn_holds_its_desire_to_the_clear():
  # never cancel a turn under way: handed back as the car starts turning, the desire stays out until the turn's own
  # clear (its angle less CLEAR_MARGIN_DEG), as when TurnPilot keeps acting, with no new rising edge
  def drive(stop_detector):
    sim = Sim([(300., "turn", "left"), (1500., "turn", "right")])
    while sim.distance() > 5.:
      sim.step()
    for _ in range(int(10. / DT_MDL)):
      sim.step(available=not stop_detector, yaw_rate=0.3)  # 17 deg/s
    return sim
  kept, handed = drive(False), drive(True)
  assert handed.trace[-1][2].handback == Handback.detectorStopped
  def desires(sim):
    return [p.desire for _, _, p in sim.trace]
  assert CLEAR_DESIRE in desires(handed)
  assert desires(handed).index(CLEAR_DESIRE) == desires(kept).index(CLEAR_DESIRE)
  assert len(rising(handed.trace, 'desire', Desire.turnLeft)) == len(rising(kept.trace, 'desire', Desire.turnLeft))


def test_fault_carries_a_model_fallback():
  # a fault is a disabled plan, but a model fallback's handback still goes out on it: the handback outranks the fault
  sim = Sim([(400., "turn", "left")], long_control=False)
  sim.big = True
  sim.run(1.)
  assert sim.trace[-1][2].handback == Handback.none and sim.trace[-1][2].fault == Fault.noLongitudinal
  sim.big = False
  plan = sim.step()
  assert plan.state == State.disabled and plan.fault == Fault.noLongitudinal and plan.handback == Handback.modelFellBack
  assert sim.tp.cancel_route


def test_skip_and_wrong_lane_unset_while_not_acting():
  # too fast for a near left with the detector stopped: no skip (navd would reroute) and no wrong-lane flag, since the
  # handback owns the alert slot and TurnPilot isn't driving the route
  sim = Sim([(400., "turn", "left"), (1500., "turn", "right")], cruise=20., nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(0, 2))
  sim.run(2.)
  for _ in range(int(30. / DT_MDL)):
    p = sim.step(available=False)
    if p.handback != Handback.none:
      assert not p.tooFastToTurn and p.laneMismatch == custom.TurnpilotPlan.LaneMismatch.none


def test_model_fallback_hands_back_and_cancels_the_route():
  sim = Sim([(400., "turn", "left"), (1500., "turn", "right")])
  sim.big = True
  sim.run(5.)
  assert sim.trace[-1][2].handback == Handback.none and not sim.tp.cancel_route
  sim.big = False  # modeld fell back to the small model
  plan = sim.step()
  assert plan.handback == Handback.modelFellBack and plan.state == State.unavailable
  assert sim.tp.cancel_route  # main() removes NavDestination once, which ends navd's route
  sim.tp.cancel_route = False
  plans = [sim.step() for _ in range(int(60. / DT_MDL))]
  assert all(p.handback == Handback.modelFellBack and p.desire == Desire.none and p.laneChange == LaneChange.none for p in plans)
  assert not sim.tp.cancel_route  # once
  # for the rest of the drive: turning TurnPilot off and on doesn't bring it back on the small model
  sim.tp.update(sim.sm, scene(), False, False)
  assert sim.step().handback == Handback.modelFellBack


def test_small_model_from_the_start_is_not_a_fallback():
  # no chestnut: modeld runs the small model from the start. That is no fallback; with no detector pkl for the small
  # model there are no detections: TurnPilot waits for them, then hands back as a detector that never started
  sim = Sim([(400., "turn", "left")])
  for _ in range(int(5. / DT_MDL)):
    plan = sim.step(available=False)
  assert plan.handback == Handback.none and plan.reason == WAITING_REASON and not sim.tp.cancel_route
  for _ in range(int(DETECTOR_START_DEADLINE_S / DT_MDL)):
    plan = sim.step(available=False)
  assert plan.handback == Handback.noDetector and not sim.tp.cancel_route


def offered_sim(**kwargs):
  sim = Sim([(500., "turn", "left"), (2000., "turn", "right")], cruise=15., model_lanes=(1, 0), paint=BROKEN_WHITE_LEFT, **kwargs)
  while not sim.trace or sim.trace[-1][2].laneChangeOffer == LaneChange.none:
    sim.step()
  return sim


def test_handback_cancels_an_offered_lane_change_out_loud():
  sim = offered_sim()
  plans = [sim.step(available=False) for _ in range(int(3. / DT_MDL))]
  first = next(i for i, p in enumerate(plans) if p.handback != Handback.none)
  assert all(p.laneChangeOffer == LaneChange.left for p in plans[:first])
  held = [p for p in plans[first:] if p.laneChangeAbort == 'handedBack']
  assert plans[first].laneChangeAbort == 'handedBack' and abs(len(held) * DT_MDL - ABORT_HOLD_S) < 2 * DT_MDL
  assert all(p.laneChangeOffer == LaneChange.none and p.laneChange == LaneChange.none and p.signalPrompt == LaneChange.none
             for p in plans[first:])


def test_fault_or_turning_off_cancels_an_offered_lane_change_out_loud():
  for stop in ('no_long', 'error', 'off'):
    sim = offered_sim()
    if stop == 'no_long':
      sim.long_control = False
      plans = [sim.step() for _ in range(int(3. / DT_MDL))]
    elif stop == 'error':
      sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
      plans = [sim.step() for _ in range(int(3. / DT_MDL))]
    else:
      plans = [sim.tp.plan(sim.sm, scene(), False, False, long_control=True).as_reader().turnpilotPlan for _ in range(int(3. / DT_MDL))]
    reason = 'turnedOff' if stop == 'off' else 'fault'
    assert all(p.state == State.disabled and p.laneChangeOffer == LaneChange.none for p in plans), stop
    held = [p for p in plans if p.laneChangeAbort == reason]
    assert plans[0].laneChangeAbort == reason and abs(len(held) * DT_MDL - ABORT_HOLD_S) < 2 * DT_MDL, stop
    # selfdrived raises the abort from a disabled plan turned off; a fault's own alert (turnpilotUnavailable, with its
    # chime) outranks it and says what happened, so the abort isn't raised under it (hooks.plan_events)
    if stop == 'off':
      assert lane_change_events(plans[0]) == ('turnpilotLaneChangeAborted',), stop
    else:
      assert plans[0].fault != Fault.none and lane_change_events(plans[0]) == (), stop


def invalid_nav(sim):
  """What navd sends off the route or rerouting: an invalid instruction for the same route."""
  return nav_instruction(0., valid=False, route_count=sim.navd.route_count), False


def approach_with_desire_out(cruise=13., to=25.):
  sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))], cruise=cruise)
  while sim.distance() > to or sim.s < 10.:
    sim.step()
  assert Desire.turnLeft in [p.desire for _, _, p in sim.trace] and sim.trace[-1][2].speedCap < NO_SPEED_CAP
  return sim


def assert_turn_held(sim, n):
  """From trace index n on: the turn's desire never cancelled or skipped, its cap kept, the car not sped up before the
  junction at s=300."""
  before = sim.trace[n - 1][2]
  after = [(s, v, p) for s, v, p in sim.trace[n:] if s < 300.]
  assert after
  assert all(p.desire != CLEAR_DESIRE and not p.tooFastToTurn for _, _, p in after)
  assert all(p.speedCap <= before.speedCap + 0.5 for _, _, p in after)
  assert max(v for _, v, _ in after) <= sim.trace[n - 1][1] + 0.05


def test_one_off_route_update_keeps_the_committed_turn():
  # the desire is out 12 m before the left when one navd update has a fix 20 m off the step: no CLEAR, no skip of the
  # same turn coming back under a new id, the cap stays, and no new pulse goes out from 10 m
  sim = approach_with_desire_out(cruise=13., to=12.)
  n, glitched = len(sim.trace), []
  def once(sim, instr):
    if not glitched:
      glitched.append(sim.s)
      return invalid_nav(sim)
    return instr, True
  sim.nav_filter = once
  while sim.s < 330.:
    sim.step()
  assert glitched
  assert_turn_held(sim, n)
  assert all(p.desire != CLEAR_DESIRE and not p.tooFastToTurn for _, _, p in sim.trace)


def test_model_fallback_on_the_approach_holds_the_turn_and_defers_the_route_cancel():
  # the big model falls back 25 m before the left, its desire out: the handback holds the desire and the cap, and the
  # route isn't cancelled until the turn is over. Were navd's route gone at once (every instruction invalid), the turn
  # is still held: no CLEAR, the cap stays, the car doesn't speed up into the junction
  for route_gone in (False, True):
    sim = approach_with_desire_out()
    sim.big = True
    sim.step()
    n = len(sim.trace)
    sim.big = False
    if route_gone:
      sim.nav_filter = lambda sim, instr: invalid_nav(sim)
    held = []
    while sim.s < 330.:
      plan = sim.step()
      assert plan.handback == Handback.modelFellBack
      if sim.s < 300.:
        held.append(sim.tp.cancel_route and sim.tp.turn_committed)
    assert all(held) and sim.tp.cancel_route and not sim.tp.turn_committed, route_gone
    assert_turn_held(sim, n)


def test_reroute_onto_the_same_turn_keeps_it():
  # a same-destination reroute starts 25 m before the left (GPS drift): two invalid updates while it runs, one between
  # the routes, then the new route with the same left. The turn is held throughout, under the same maneuver
  sim = approach_with_desire_out()
  n, first, frames = len(sim.trace), sim.tp.follower.maneuver_id, []
  def reroute(sim, instr):
    frames.append(sim.s)
    if len(frames) <= 3:
      if len(frames) == 3:
        sim.navd.route_count = 1  # the new route is in: one invalid instruction marks it, then its own
      return invalid_nav(sim)
    return instr, True
  sim.nav_filter = reroute
  while sim.s < 330.:
    sim.step()
    if sim.s < 290.:
      assert sim.tp.follower.maneuver_id == first
  assert_turn_held(sim, n)


def test_navd_going_quiet_on_the_approach_keeps_the_turn_cap():
  sim = approach_with_desire_out()
  n = len(sim.trace)
  sim.nav_filter = lambda sim, instr: (instr, True)
  update = sim.navd.update
  sim.navd.update = lambda s: None if s < 310. else update(s)
  while sim.s < 330.:
    sim.step()
  assert_turn_held(sim, n)


def test_turn_just_after_a_bend_or_a_new_name_is_slowed_for_not_skipped():
  # a left junction turn 40-50 m after a step that is NONE (a bend no other road meets, a straight-on name change): the
  # turn cap brakes for the left before navd moves past that step, so the left isn't skipped as too fast, and its pulse
  # goes once the car is where navd moves past the step, not up to a second later
  for first in (("turn", "right", junction_of("right", 0, 60.)), ("new name", "straight", (0., 0., 2))):
    for gap in (40., 50.):
      sim = Sim([(300.,) + first, (300. + gap, "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))])
      while sim.s < 305. + gap:
        sim.step()
      plans = [p for _, _, p in sim.trace]
      assert not any(p.tooFastToTurn for p in plans), (first, gap)
      edges = rising(sim.trace, 'desire', Desire.turnLeft)
      assert 300. + 9. < edges[0] < 300. + gap - MIN_FIRST_PULSE_M, (first, gap)


def test_turn_too_close_after_a_bend_for_its_first_pulse_is_skipped_out_loud():
  # navd moves past a NONE step only 10 m beyond it, so a left 5-30 m after it comes up inside the lead gate 1's model
  # takes a turn from (and under 10 m, past the junction). Its first pulse never goes that late: the turn is skipped out
  # loud (tooFastToTurn, navd reroutes), not pulsed into a missed turn, or into the curb past the junction. The skip is
  # announced before the junction, however close it comes after the step
  for first in (("turn", "right", junction_of("right", 0, 60.)), ("new name", "straight", (0., 0., 2))):
    for gap in (5., 8., 10., 12., 15., 20., 25., 30.):
      sim = Sim([(300.,) + first, (300. + gap, "turn", "left", junction_of("left", 2, opposing='none')), (1500., "turn", "right", junction_of("right", 2))])
      while sim.s < 300. + gap + 20.:
        sim.step()
      assert not rising(sim.trace, 'desire', Desire.turnLeft), (first, gap)
      announced = rising(sim.trace, 'tooFastToTurn', True)
      assert announced and announced[0] < 300. + gap, (first, gap)
      # and once announced, it stays announced through the junction
      assert all(p.tooFastToTurn for s, _, p in sim.trace if announced[0] <= s < 300. + gap), (first, gap)


SOFT_DISABLE_S = 3.  # selfdrived/state.py SOFT_DISABLE_TIME: lat stays active this long after the handback


def recover_after_outage(cruise, outage, reengage=1.):
  """The turn desire goes out, then the detector stops for outage s: handback, soft disable (lat active SOFT_DISABLE_S),
  then openpilot disengaged until the driver engages again reengage s after the detector comes back. Returns the sim
  and the trace index at which TurnPilot acts again."""
  sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))], cruise=cruise)
  while sim.distance() > TURN_DESIRE_LEAD_M - 1. or sim.s < 10.:
    sim.step()
  assert sim.trace[-1][2].desire == Desire.turnLeft
  t, handback_t, back_t, back_i = 0., None, None, None
  while sim.s < 310.:
    available = t >= outage
    if handback_t is None and not sim.tp.acting:
      handback_t = t
    if available and back_t is None:
      back_t, back_i = t, len(sim.trace)
    disengaged = handback_t is not None and t - handback_t >= SOFT_DISABLE_S
    lat = not disengaged or (back_t is not None and t - back_t >= reengage)
    sim.step(available=available, lat_active=lat)
    t += DT_MDL
  return sim, back_i


def test_turn_dropped_while_disengaged_is_not_left_half_driven():
  # the detector is back after a long outage and the driver engages again before the left, whose desire was dropped
  # while openpilot was off: once openpilot steers again, TurnPilot never drives on toward it capped, 'approaching' and
  # with no desire and no alert (a fresh pulse, or the turn skipped out loud)
  for cruise in (13., 3.5):
    sim, back = recover_after_outage(cruise, outage=5.)
    engaged = back + int(round(1. / DT_MDL))
    assert any(p.desire == Desire.turnLeft or p.tooFastToTurn for _, _, p in sim.trace[engaged:engaged + 5]), cruise
    after = [p for s, _, p in sim.trace[engaged + 5:] if s < 300.]
    assert after, cruise
    silent = [p for p in after if p.desire == Desire.none and p.speedCap < NO_SPEED_CAP and not p.tooFastToTurn and
              p.state == State.approaching]
    assert not silent, cruise


def test_dropped_turn_taken_up_or_skipped_with_its_pulse_cleared():
  # the left's desire went out 38 m before it, then the detector stopped and openpilot stopped steering lat_off s in;
  # the detector is back loss s in, with openpilot steering again. The pulse may still be in the model's queue. Room to
  # make the turn and for a pulse to be taken (ENTRY_M out): a fresh pulse, and the turn driven. Too close: skipped out
  # loud, its pulse cleared from the model, and the turn cap kept until the car is past the junction (never turning
  # from the old pulse at speed, never speeding up into the junction)
  for lat_off, loss, pulse in ((1.5, 2.5, True), (3.0, 4.0, True), (3.2, 3.6, True), (4.4, 5.0, False)):
    sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))], cruise=13.)
    while sim.distance() > 38. or sim.s < 10.:
      sim.step()
    for i in range(int(round(loss / DT_MDL))):
      sim.step(available=False, lat_active=i * DT_MDL < lat_off)
    back = len(sim.trace)
    while sim.s < 310.:
      sim.step()
    after = [(s, v, p) for s, v, p in sim.trace[back:]]
    to_junction = [(v, p) for s, v, p in after if s < 300.]
    assert all(b[0] <= a[0] + 1e-6 for a, b in zip(to_junction, to_junction[1:], strict=False)), (lat_off, loss)
    if pulse:
      assert after[0][2].desire == Desire.turnLeft and not any(p.tooFastToTurn for _, _, p in after), (lat_off, loss)
    else:
      assert after[0][2].tooFastToTurn and after[0][2].desire == CLEAR_DESIRE, (lat_off, loss)
      assert not any(p.desire == Desire.turnLeft for _, _, p in after), (lat_off, loss)
      assert all(p.speedCap < 5. for _, p in to_junction), (lat_off, loss)


def test_detector_back_within_the_soft_disable_keeps_the_turn():
  # back before openpilot disengaged: the held desire stays out with no new rising edge, and TurnPilot drives the turn on
  sim, back = recover_after_outage(13., outage=2.)
  after = [p for s, _, p in sim.trace[back:] if s < 300.]
  assert after and all(p.desire in (Desire.turnLeft, Desire.none) and not p.tooFastToTurn for p in after)
  assert after[0].desire == Desire.turnLeft and all(p.state == State.turning for p in after[:5])



def test_detector_back_after_a_stop_on_the_approach_keeps_the_turn():
  # stopped at a light 20 m before a committed left, the detector drops for 2.2 s and comes back with openpilot still
  # engaged (inside the soft disable): the standstill re-armed the desire, it wasn't dropped, so the turn isn't skipped
  # and its pulse goes again as the car pulls away
  sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))], cruise=13.)
  while sim.distance() > 20. or sim.s < 10.:
    sim.step()
  assert Desire.turnLeft in [p.desire for _, _, p in sim.trace]
  sim.cruise = 0.
  for _ in range(int(3. / DT_MDL)):  # brake to a stop
    sim.step()
  for i in range(int(3.7 / DT_MDL)):  # stopped past STANDSTILL_REARM_S, the detector down from 0.5 s to 2.7 s
    sim.v = 0.
    sim.step(available=not 0.5 <= i * DT_MDL < 2.7)
  assert any(p.handback == Handback.detectorStopped for _, _, p in sim.trace)
  assert sim.trace[-1][2].handback == Handback.none
  stop = len(sim.trace)
  sim.cruise = 13.
  for _ in range(int(10. / DT_MDL)):
    sim.step()
  assert sim.s > 300.
  assert not any(p.tooFastToTurn for _, _, p in sim.trace)
  assert rising(sim.trace[stop:], 'desire', Desire.turnLeft)


def test_detector_that_never_starts_hands_back():
  # no pkl, a detector that fails to load, or a driving model with no detector: never a fresh detection. Past
  # DETECTOR_START_DEADLINE_S of modeld running, that is a handback (selfdrived: soft disable while engaged, no engaging)
  sim = Sim([(400., "turn", "left"), (1500., "turn", "right")])
  sim.model_running = False  # modeld loading the detector: not counted
  for _ in range(int(2 * DETECTOR_START_DEADLINE_S / DT_MDL)):
    assert sim.step(available=False).handback == Handback.none
  sim.model_running = True
  plans = [sim.step(available=False) for _ in range(int((DETECTOR_START_DEADLINE_S + 5.) / DT_MDL))]
  first = next(i for i, p in enumerate(plans) if p.handback != Handback.none)
  assert abs((first + 1) * DT_MDL - DETECTOR_START_DEADLINE_S) <= 1.5 * DT_MDL
  assert all(p.handback == Handback.noDetector and p.state == State.unavailable for p in plans[first:])
  assert plans[-1].reason == "detector not running: take control"
  assert all(p.desire == Desire.none and p.laneChange == LaneChange.none for p in plans)
  # it starts after all: acting
  assert sim.step().handback == Handback.none and sim.trace[-1][2].state != State.unavailable


def test_fault_in_a_detector_gap_keeps_the_stop_and_the_handback():
  # stopped short of a pedestrian, the detector goes quiet and the stop is carried. One failed frame in the gap must not
  # drop the carried stop, nor forget that the detector ran (its handback would never come)
  x_ped = 200.
  sim = Sim([(1500., "turn", "right")])
  def pedestrian(sim):
    return ({'cls': 'pedestrian', 'x': x_ped - sim.s, 'y': 0.1, 'vx': 0., 'vy': 0., 'inPath': True},)
  for _ in range(int(40. / DT_MDL)):
    sim.step(pedestrian(sim))
  assert sim.trace[-1][2].shouldStop and sim.v == 0.
  for _ in range(6):
    assert sim.step(available=False).shouldStop
  sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
  assert sim.step(available=False).fault == Fault.error
  del sim.tp.update
  plans = [sim.step(available=False) for _ in range(int(10. / DT_MDL))]
  assert all(p.shouldStop for p in plans) and sim.s < x_ped - PED_STOP_GAP_M + 1.
  assert plans[-1].handback == Handback.detectorStopped
  # failing on every frame: the disabled plans still carry the stopped detector's handback, by its deadline
  sim = Sim([(1500., "turn", "right")])
  sim.run(3.)
  sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
  plans = [sim.step(available=False) for _ in range(int(3. / DT_MDL))]
  assert all(p.fault == Fault.error and p.state == State.disabled for p in plans)
  first = next(i for i, p in enumerate(plans) if p.handback != Handback.none)
  assert abs((first + 1) * DT_MDL - (DETECTOR_DEADLINE_S - STALE_S)) <= 1.5 * DT_MDL
  assert all(p.handback == Handback.detectorStopped for p in plans[first:])
  # with the detector running, a failing frame is only the fault
  assert sim.step().handback == Handback.none


def test_model_fallback_outlives_turnpilotd():
  # TurnPilot turned off and on runs a new turnpilotd, which never saw the big model: modeld's ChestnutActive (False:
  # the big model isn't running this drive) still hands back, with no route cancel (the first process did that)
  sim = Sim([(400., "turn", "left"), (1500., "turn", "right")])
  sim.tp.big_model_failed(enabled=True)
  plans = [sim.step() for _ in range(int(5. / DT_MDL))]
  assert all(p.handback == Handback.modelFellBack and p.desire == Desire.none and p.laneChange == LaneChange.none for p in plans)
  assert not sim.tp.cancel_route
  # in the process that drove the route, the param read first is the fallback as much as modelV2 is: the route is cancelled
  sim = Sim([(400., "turn", "left"), (1500., "turn", "right")])
  sim.big = True
  sim.run(2.)
  sim.tp.big_model_failed(enabled=True)
  assert sim.step().handback == Handback.modelFellBack and sim.tp.cancel_route


def oncoming_at(junction_s: float, after_s: float):
  """An oncoming car that reaches the left at junction_s 8 s after the car is 40 m from it (none before after_s)."""
  spawn = {}
  def objects(sim):
    if sim.s < after_s:
      return ()
    if 'x' not in spawn and junction_s - sim.s < 40.:
      spawn['x'], spawn['t'] = junction_s + 72., sim.t
    if 'x' not in spawn:
      return ()
    return ({'cls': 'car', 'x': spawn['x'] - 9. * (sim.t - spawn['t']) - sim.s, 'y': 5., 'vx': -9.},)
  return objects


def test_fault_keeps_the_next_turns_junction_hold():
  # a left is committed (the car passed its hold line faster than COMMIT_SPEED), then one frame faults. The next left,
  # with an oncoming car, still holds for it: maneuver ids never repeat within a drive, so the first left's commitment
  # (or a gas release) can't be read as the next one's
  for fault in (False, True):
    sim = Sim([(300., "turn", "left", T_LEFT), (700., "turn", "left", (270., 180., 2, 'none')), (2500., "turn", "right")])
    oncoming = oncoming_at(700., 400.)
    for _ in range(int(100. / DT_MDL)):
      if fault and 500. < sim.s and not getattr(sim, 'faulted', False):
        sim.faulted = True
        sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
        assert sim.step().fault == Fault.error
        del sim.tp.update
      near = any(abs(sim.s - m) < 8. for m in (300., 700.))
      sim.step(oncoming(sim), steer=90. if near else 0.)
      if sim.s > 730.:
        break
    assert sim.tp.scene.committed_id is not None, fault
    assert any(p.reason == "waiting for oncoming" for s, _, p in sim.trace if s > 400.), fault
    assert any(p.shouldStop for s, v, p in sim.trace if 400. < s < 700. and v < 0.1), fault


def held_for_pedestrian(x_ped=200.):
  sim = Sim([(1500., "turn", "right")])
  def pedestrian(sim):
    return ({'cls': 'pedestrian', 'x': x_ped - sim.s, 'y': 0.1, 'vx': 0., 'vy': 0., 'inPath': True},)
  for _ in range(int(40. / DT_MDL)):
    sim.step(pedestrian(sim))
  assert sim.trace[-1][2].shouldStop and sim.v == 0.
  return sim, pedestrian


def test_persistent_fault_keeps_the_scenes_stops():
  # turnpilotd failing on every frame: the faulted plans still carry the pedestrian stop (live, or carried through a
  # detector gap, which then hands back). The car never drives on through the pedestrian
  for gap in (False, True):
    sim, pedestrian = held_for_pedestrian()
    sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
    plans = [sim.step(() if gap else pedestrian(sim), available=not gap) for _ in range(int(12. / DT_MDL))]
    assert all(p.fault == Fault.error and p.state == State.disabled for p in plans), gap
    assert all(p.shouldStop and p.speedCap < 1. for p in plans), gap
    assert sim.s < 200. - PED_STOP_GAP_M + 1. and sim.v < 0.1, gap
    assert (plans[-1].handback == Handback.detectorStopped) == gap


def test_fault_keeps_a_junction_hold():
  # waiting at the hold line for an oncoming car, turnpilotd faults (once in a detector gap, or on every frame): the
  # route is gone with the fault, but the hold in force stays where it was until the scene says otherwise or the
  # driver presses the gas
  for mode in ('every frame', 'in a gap'):
    sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right")])
    def car(sim):  # an oncoming car edging up to it
      return ({'cls': 'car', 'x': 305. - sim.s, 'y': 5., 'vx': -1.5},)
    while not (sim.v < 0.05 and sim.s > 250.):
      sim.step(car(sim))
      assert sim.t < 120.
    assert sim.trace[-1][2].reason == "waiting for oncoming" and sim.trace[-1][2].shouldStop
    stopped = sim.s
    if mode == 'every frame':
      sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
      plans = [sim.step(car(sim)) for _ in range(int(5. / DT_MDL))]
    else:
      plans = [sim.step(available=False) for _ in range(4)]
      sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
      plans.append(sim.step(available=False))
      del sim.tp.update
      plans += [sim.step(available=False) for _ in range(int(5. / DT_MDL))]
    assert all(p.shouldStop for p in plans), mode
    assert sim.s < stopped + 0.5, mode
    # the driver looks and goes: the gas releases it
    released = sim.step(car(sim) if mode == 'every frame' else (), available=mode == 'every frame', gas=True)
    assert not released.shouldStop and released.accelCap == NO_ACCEL_CAP, mode
    # the turn cap at most; handed back (the gap outlasted the detector's deadline), no faster than the driver takes it
    # (a committed turn faulting, or handed back: held to the speed the driver's gas takes it to)
    assert released.speedCap > 2. or released.speedCap >= sim.v - 1e-3, mode


def test_detector_not_loaded_while_engaged_isnt_a_detector_that_never_started():
  # TurnPilot turned on while disengaged, and openpilot engaged before modeld read TurnPilotEnabled: modeld loads the
  # detector only while disengaged, so it hasn't tried yet. Engaged frames don't count against its modelV2 start
  # deadline until modeld had its chance; TurnPilot waits (not acting). But it never waits silently: from the first
  # engaged frame with no detection yet, DETECTOR_START_DEADLINE_S on (modeld loading, or not publishing at all: a load
  # that failed while the driver engaged during it) it hands back noDetector, out loud
  for model_running in (True, False):
    sim = Sim([(4000., "turn", "left"), (5000., "turn", "right")])
    for _ in range(int(0.5 / DT_MDL)):
      sim.step(available=False)
    sim.model_running = model_running
    plans = [sim.step(available=False, engaged=True) for _ in range(int(2 * DETECTOR_START_DEADLINE_S / DT_MDL))]
    first = next(i for i, p in enumerate(plans) if p.handback == Handback.noDetector)
    assert abs((first + 1) * DT_MDL - DETECTOR_START_DEADLINE_S) <= 1.5 * DT_MDL, model_running
    assert all(p.handback == Handback.none and p.state == State.unavailable and p.reason == WAITING_REASON
               for p in plans[:first]), model_running
    assert all(p.handback == Handback.noDetector for p in plans[first:]), model_running
  # ...whether or not openpilot stays engaged
  sim = Sim([(4000., "turn", "left"), (5000., "turn", "right")])
  sim.model_running = False
  plans = [sim.step(available=False, engaged=True)] + [sim.step(available=False) for _ in range(int(20. / DT_MDL))]
  first = next(i for i, p in enumerate(plans) if p.handback == Handback.noDetector)
  assert abs((first + 1) * DT_MDL - DETECTOR_START_DEADLINE_S) <= 1.5 * DT_MDL
  # disengaged long enough for modeld to have tried: engaged frames count from then on
  sim = Sim([(4000., "turn", "left"), (5000., "turn", "right")])
  for _ in range(int(3. / DT_MDL)):
    sim.step(available=False)
  plans = [sim.step(available=False, engaged=True) for _ in range(int(DETECTOR_START_DEADLINE_S / DT_MDL))]
  first = next(i for i, p in enumerate(plans) if p.handback == Handback.noDetector)
  assert abs((first + 1) * DT_MDL - (DETECTOR_START_DEADLINE_S - 3.)) <= 1.5 * DT_MDL


def test_handback_never_speeds_the_car_up():
  # the driving model falls back 60 m before a left, its desire not out yet: TurnPilot hands back and the route is
  # cancelled, but through the soft disable (openpilot still steering) the car never speeds up. Once openpilot is
  # disengaged the hold goes with it
  sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))], cruise=13.)
  sim.big = True
  while sim.distance() > 60. or sim.s < 10.:
    sim.step()
  assert sim.trace[-1][2].desire == Desire.none and sim.trace[-1][2].speedCap < 12.
  sim.big = False
  sim.nav_filter = lambda sim, instr: (nav_instruction(0., valid=False), False)  # the route cancelled
  start = len(sim.trace)
  for _ in range(int(SOFT_DISABLE_S / DT_MDL)):
    sim.step()
  held = sim.trace[start:]
  assert all(p.handback == Handback.modelFellBack for _, _, p in held)
  assert all(b[1] <= a[1] + 1e-6 for a, b in zip(held, held[1:], strict=False))
  assert sim.step(lat_active=False).speedCap == NO_SPEED_CAP
  assert sim.step().speedCap == NO_SPEED_CAP  # engaged again later (on the small model): no hold left over


def test_fault_keeps_the_commitment_to_the_turn_under_way():
  # past the hold line committed to a left (or released into it by the gas), one frame faults and navd brings the same
  # left back under a new id. Slow in the junction with an oncoming car in range, the car is never held there again
  def oncoming(sim):
    return ({'cls': 'car', 'x': 306. - sim.s, 'y': 5., 'vx': -1.5},)
  for how in ('committed', 'released'):
    sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right")])
    if how == 'committed':
      while sim.tp.scene.committed_id is None:
        sim.step()
        assert sim.s < 300.
    else:
      while not (sim.v < 0.05 and sim.s > 250.):
        sim.step(oncoming(sim))
        assert sim.t < 120.
      sim.cruise = 1.
      sim.step(oncoming(sim), gas=True)
      assert sim.tp.scene.released_id is not None
      while sim.s < 300. - HOLD_LINE_M + 1.:
        sim.step(oncoming(sim))
    for _ in range(14):  # 20 degrees round the corner
      sim.step(oncoming(sim), yaw_rate=0.5)
    pulses, turn = sim.tp.desires.pulses, sim.tp.follower.maneuver_id
    sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
    assert sim.step(oncoming(sim)).fault == Fault.error
    del sim.tp.update
    sim.cruise = sim.v = 1.
    start = len(sim.trace)
    while sim.s < 305.:
      sim.step(oncoming(sim), yaw_rate=0.05)
    after = [p for _, _, p in sim.trace[start:]]
    assert not any(p.shouldStop or p.reason == "waiting for oncoming" for p in after), how
    # nor is the turn under way dropped: not skipped, its cap kept, its desire not pulsed afresh
    assert not any(p.tooFastToTurn for p in after) and all(p.speedCap < 5. for p in after), how
    assert sim.tp.desires.pulses == pulses and sim.tp.follower.maneuver_id == turn, how


def drive_through_the_left(sim, n):
  """Drive the committed left at s=300 from trace index n, turning from 2 m before it, and check it was held: the same
  maneuver until the junction, its cap kept and the car not sped up into it, no CLEAR until the car is round it, no
  skip. The left is round (80 of its 90 degrees) about 10 m past the junction."""
  turn = sim.tp.follower.maneuver_id
  while sim.s < 325.:
    sim.step(yaw_rate=0.5 if 298. < sim.s < 312. else 0.)
    if sim.s < 300.:
      assert sim.tp.follower.maneuver_id == turn
  assert_turn_held(sim, n)
  v_n = sim.trace[n - 1][1]
  assert max(v for s, v, _ in sim.trace[n:] if s < 305.) <= v_n + 0.05
  assert all(p.desire != CLEAR_DESIRE for s, _, p in sim.trace[n:] if s < 305.)
  assert not any(p.tooFastToTurn for _, _, p in sim.trace)


def test_reroute_onto_the_same_turn_keeps_its_junction_where_it_was():
  # a reroute near m before the committed left (one invalid update, then routeCount 1), whose same left reads off m
  # further on (its origin snapped back) or closer (snapped forward) in every update after: the junction is where it
  # was dead-reckoned, so the cap doesn't lift and the turn isn't read as passed before the car gets there
  for off, near in ((25., 14.), (-20., 14.), (-25., 14.), (-20., 25.), (25., 6.)):
    sim = approach_with_desire_out(to=near + 5.)
    while sim.distance() > near:
      sim.step()
    n = len(sim.trace)
    def snapped(sim, instr, off=off, k=[0], J=300., NEXT=1500.):  # noqa: B006 (k counts the updates of this sim)
      k[0] += 1
      if k[0] == 1:
        return invalid_nav(sim)
      if instr.maneuverDistance > -10.:
        return nav_instruction(J - sim.s + off, "turn", "left", remaining=NEXT - sim.s, junction=T_LEFT,
                               route_count=1, ahead=[(NEXT - sim.s + off, "turn", "right", junction_of("right"))]), True
      return nav_instruction(NEXT - sim.s + off, "turn", "right", remaining=NEXT - sim.s, junction=junction_of("right"),
                             route_count=1), True
    sim.nav_filter = snapped
    drive_through_the_left(sim, n)


def test_navd_moving_on_early_keeps_the_committed_turn():
  # navd moves on to the next step (other bearings) at m before the committed left: a sharp turn's next step, or GPS
  # noise forward. The left isn't passed until the car has turned it or is dead-reckoned past it: its id and cap stay,
  # and no CLEAR goes before the car is round it
  for at, nxt in ((10., 1200.), (6., 1200.), (15., 1200.), (20., 1200.), (8., 60.)):
    sim = approach_with_desire_out(to=at + 5.)
    sim.navd.maneuvers[1] = (300. + nxt, "turn", "right", junction_of("right", 2))
    n = len(sim.trace)
    def early(sim, instr, at=at):
      if sim.s >= 300. - at:
        pos, mtype, modifier, junction = sim.navd.maneuvers[1]
        return nav_instruction(pos - sim.s, mtype, modifier, remaining=pos - sim.s, junction=junction), True
      return instr, True
    sim.nav_filter = early
    drive_through_the_left(sim, n)
    # once the left is over, the next step is taken up
    while sim.s < 340.:
      sim.step()
    assert sim.tp.follower.maneuver.side == -1, (at, nxt)



def test_gps_jump_on_the_committed_turns_step_keeps_its_junction_where_it_was():
  # the same route, the same step: navd's distance to the committed left reads off m from near m out, in one update or in
  # every update after (a GPS jump, closer when off is negative). The junction is where it was dead-reckoned, whichever
  # way it jumped: the turn isn't read as passed before the car gets there, so the cap doesn't lift into it
  for off, near, stay in ((-20., 14., True), (-20., 14., False), (-25., 25., True), (-10., 14., True), (12., 12., True),
                          (30., 12., True)):
    sim = approach_with_desire_out(to=near + 5.)
    while sim.distance() > near:
      sim.step()
    n = len(sim.trace)
    def jumped(sim, instr, off=off, stay=stay, k=[0]):  # noqa: B006 (k counts the updates of this sim)
      k[0] += 1
      if (k[0] > 1 and not stay) or instr.maneuverModifier != 'left':
        return instr, True
      return nav_instruction(instr.maneuverDistance + off, "turn", "left", remaining=instr.distanceRemaining, junction=T_LEFT,
                             ahead=[(1500. - sim.s + off, "turn", "right", junction_of("right"))]), True
    sim.nav_filter = jumped
    drive_through_the_left(sim, n)


def test_gps_outlier_that_brings_the_turns_first_pulse_out_doesnt_hold_its_junction_short():
  # one navd update reads the turn off m closer from near m out, and that reading brings the turn's first pulse out.
  # Which reading is right can't be told: the junction is held at the nearest (off m short), so the turn isn't read as
  # passed short of it, its cap stays to the junction, and the car isn't sped up into it. On an unprotected left the car
  # stops at or short of its hold line, never past it, and short by more than 15 m only said out loud
  J = 400.
  for side, off, near in (('right', -20., 60.), ('right', -12., 50.), ('right', -15., 55.), ('right', -25., 70.),
                          ('right', -8., 45.), ('left', -20., 60.)):
    nxt = 'left' if side == 'right' else 'right'
    mans = [(J, "turn", side, junction_of(side)), (J + 1200., "turn", nxt, junction_of(nxt, 2))]
    sim = Sim(mans)
    def outlier(sim, instr, side=side, off=off, near=near, mans=mans, done=[]):  # noqa: B006 (once per sim)
      if sim.s < J - near or done:
        return instr, True
      done.append(sim.s)
      return nav_instruction(J - sim.s + off, "turn", side, remaining=J + 1300. - sim.s + off, junction=junction_of(side),
                             ahead=[(mans[1][0] - sim.s + off,) + tuple(mans[1][1:])]), True
    sim.nav_filter = outlier
    committed_at, stops, errors = None, [], []
    while sim.s < J + 30.:
      sim.step(yaw_rate=(0.5 if side == 'left' else -0.5) if J - 2. < sim.s < J + 12. else 0.)
      if committed_at is None and sim.tp.turn_committed:
        committed_at = sim.s
      if sim.v < 0.3:
        stops.append(J - sim.s)
      if committed_at is not None and committed_at + 15. < sim.s < J:  # navd's next update in
        errors.append(sim.distance() - (J - sim.s))
    case = (side, off, near)
    assert committed_at is not None and committed_at >= J - near - 1., case  # the outlier is what brought the pulse out
    before = [(s, v, p) for s, v, p in sim.trace if committed_at <= s < J + 8.]
    assert all(p.speedCap < NO_SPEED_CAP and p.desire != CLEAR_DESIRE for _, _, p in before), case
    assert errors and max(errors) < 1., (case, max(errors))  # the junction never further than it is
    assert max(v for s, v, _ in before if s > J - 15.) < 5., case  # held to the turn's cap, 4.3 m/s here
    if side == 'left':
      assert_hold_line_kept(sim, stops, case)


def assert_hold_line_kept(sim, stops, case):
  """An unprotected turn's stop: at its hold line or short of it, never past it, and more than 2 m short of it only
  with the junction said to be uncertain (junctionUncertain, by its readings' spread)."""
  assert stops and HOLD_LINE_M < min(stops), (case, stops[:1], stops[-1:])
  if max(stops) >= HOLD_LINE_M + 2.:
    assert any(p.controlAlert == 'junctionUncertain' for _, _, p in sim.trace), (case, max(stops))


def assert_junction_kept(sim, J, side, case, from_s, ids=None):
  """From from_s to the junction at J: the turn's cap and desire kept (no CLEAR), the car held to its cap through it
  (4.3 m/s here), the follower's junction never further than it is in the last 30 m (ids: follower less true, m), and on
  an unprotected left the car stopped at its hold line or short of it (assert_hold_line_kept)."""
  before = [(s, v, p) for s, v, p in sim.trace if from_s <= s < J + 8.]
  assert before and all(p.speedCap < NO_SPEED_CAP and p.desire != CLEAR_DESIRE for _, _, p in before), case
  assert max(v for s, v, _ in sim.trace if J - 15. < s < J + 8.) < 5., case
  if ids is not None:
    assert ids and max(ids) < 1., (case, max(ids))
  if side == 'left':
    assert_hold_line_kept(sim, [J - s for s, v, _ in sim.trace if v < 0.3 and J - 40. < s < J], case)


def test_gps_outliers_around_the_turns_first_pulse_never_hold_its_junction_short():
  # navd's readings from near m out carry offs (one a second), then are true again. The reading that brings the turn's
  # first pulse out is an outlier, and so may the ones either side of it be (noise of 5 m or more before it, the outlier
  # lasting two fixes or decaying), or it is the first reading of a reroute (route_count 1 from it on). However many
  # readings it is off for, the readings before and after it outvote it: the junction is never held where it put it,
  # the turn read as passed, its cap lifted, short of the junction
  J = 400.
  for side, cruise, near, offs, route_count in (
      ('right', 13., 72., (-5., -20.), 0), ('right', 13., 72., (12., -20.), 0), ('right', 9., 65., (12., -20.), 0),
      ('left', 13., 72., (12., -20.), 0), ('left', 13., 72., (-8., -20.), 0), ('right', 13., 72., (-10., -20., -30.), 0),
      ('right', 13., 72., (-20., -20.), 0), ('right', 6., 55., (-12., -15.), 0), ('right', 13., 60., (-20., -17., -12., -6.), 0),
      ('left', 13., 60., (-20., -15.), 0),
      ('right', 13., 60., (-20.,), 1), ('right', 13., 70., (-25.,), 1), ('right', 13., 50., (-12.,), 1), ('left', 13., 60., (-20.,), 1)):
    nxt = 'left' if side == 'right' else 'right'
    mans = [(J, "turn", side, junction_of(side)), (J + 1200., "turn", nxt, junction_of(nxt, 2))]
    sim = Sim(mans, cruise=cruise)
    def outliers(sim, instr, side=side, offs=offs, near=near, mans=mans, route_count=route_count, k=[0]):  # noqa: B006
      if sim.s < J - near or sim.s > J + 5.:
        return instr, True
      off = offs[k[0]] if k[0] < len(offs) else 0.
      k[0] += 1
      return nav_instruction(J - sim.s + off, "turn", side, remaining=J + 1300. - sim.s + off, junction=junction_of(side),
                             route_count=route_count, ahead=[(mans[1][0] - sim.s + off,) + tuple(mans[1][1:])]), True
    sim.nav_filter = outliers
    committed_at, errors = None, []
    while sim.s < J + 30.:
      sim.step(yaw_rate=(0.5 if side == 'left' else -0.5) if J - 2. < sim.s < J + 12. else 0.)
      if committed_at is None and sim.tp.turn_committed:
        committed_at = sim.s
      if J - 30. < sim.s < J:
        errors.append(sim.distance() - (J - sim.s))
    case = (side, cruise, near, offs, route_count)
    assert committed_at is not None and committed_at >= J - near - 1., case  # an outlier is what brought the pulse out
    assert_junction_kept(sim, J, side, case, committed_at, errors)


def test_a_gps_bias_long_before_the_turn_or_lasting_into_it_never_holds_its_junction_short():
  # navd's readings from near m out carry offs (one a second), then are true again: a bias over the long approach that
  # outnumbers the true readings after it (p_seq2), one that starts close in and lasts, or a jump close in. No reading
  # outvotes another: the cap, the desire's lead and the hold line are at the nearest any reading put the junction, the
  # turn passed only past the furthest, so never passed or its cap lifted short of the junction, never a hold line past
  # its true one, and on an unprotected left a junction read more than 15 m apart is said out loud
  J = 400.
  for side, near, offs in (('right', 395., (-20.,) * 16), ('left', 395., (-20.,) * 16), ('left', 395., (20.,) * 16),
                           ('right', 395., (-12.,) * 16), ('left', 30., (-20.,)), ('left', 25., (-20.,) * 4),
                           ('left', 20., (-15.,)), ('right', 30., (-20.,) * 6), ('left', 38., (20.,) * 6)):
    nxt = 'left' if side == 'right' else 'right'
    mans = [(J, "turn", side, junction_of(side)), (J + 1200., "turn", nxt, junction_of(nxt, 2))]
    sim = Sim(mans)
    def biased(sim, instr, side=side, offs=offs, near=near, mans=mans, k=[0]):  # noqa: B006
      if sim.s < J - near or sim.s > J + 5. or k[0] >= len(offs):
        return instr, True
      off = offs[k[0]]
      k[0] += 1
      return nav_instruction(J - sim.s + off, "turn", side, remaining=J + 1300. - sim.s + off, junction=junction_of(side),
                             ahead=[(mans[1][0] - sim.s + off,) + tuple(mans[1][1:])]), True
    sim.nav_filter = biased
    committed_at, errors = None, []
    while sim.s < J + 30.:
      sim.step(yaw_rate=(0.5 if side == 'left' else -0.5) if J - 2. < sim.s < J + 12. else 0.)
      if committed_at is None and sim.tp.turn_committed:
        committed_at = sim.s
      if J - 30. < sim.s < J:
        errors.append(sim.distance() - (J - sim.s))
    case = (side, near, offs[:2], len(offs))
    assert committed_at is not None, case
    assert_junction_kept(sim, J, side, case, committed_at, errors)
    if side == 'left' and min(offs) <= -20.:
      assert any(p.controlAlert in ('junctionUncertain', 'cannotStopComfortably') for _, _, p in sim.trace), case


def test_gps_outlier_that_commits_the_turn_and_a_fault_after_it_dont_pass_the_turn_short():
  # navd's reading 60 m out is 20 m short and brings the turn's first pulse out; turnpilotd faults from 56 m for
  # frames frames, dead-reckoning the committed turn. It is on the track navd's readings agree on, not on the outlier's:
  # not read as passed short of the junction once the fault is over, its cap kept to it
  J = 400.
  for frames in (60, 100, 120, 160):
    mans = [(J, "turn", "right", junction_of("right")), (J + 1200., "turn", "left", junction_of("left", 2))]
    sim = Sim(mans)
    def outlier(sim, instr, mans=mans, done=[]):  # noqa: B006
      if sim.s < J - 60. or done:
        return instr, True
      done.append(sim.s)
      return nav_instruction(J - sim.s - 20., "turn", "right", remaining=J + 1300. - sim.s - 20., junction=junction_of("right"),
                             ahead=[(mans[1][0] - sim.s - 20.,) + tuple(mans[1][1:])]), True
    sim.nav_filter = outlier
    update, faulted = sim.tp.follower.update, [0]
    def failing(*args, sim=sim, frames=frames, update=update, faulted=faulted, **kwargs):
      if sim.s >= J - 56. and faulted[0] < frames:
        faulted[0] += 1
        raise RuntimeError("test fault")
      return update(*args, **kwargs)
    sim.tp.follower.update = failing
    committed_at = None
    while sim.s < J + 30.:
      sim.step(yaw_rate=-0.5 if J - 2. < sim.s < J + 12. else 0.)
      if committed_at is None and sim.tp.turn_committed:
        committed_at = sim.s
    assert committed_at is not None and committed_at >= J - 61., frames
    assert faulted[0] == frames
    caps = [p.speedCap for s, _, p in sim.trace if J - 1. < s < J]
    assert caps and max(caps) < NO_SPEED_CAP, frames  # the cap kept to the junction
    assert all(p.desire != CLEAR_DESIRE for s, _, p in sim.trace if committed_at <= s < J), frames
    assert max(v for s, v, _ in sim.trace if J - 15. < s < J) < 9., frames  # never sped up past the cruise it slowed from


def test_gps_outlier_on_the_first_reading_of_the_next_turns_step_doesnt_hold_its_junction_short():
  # a right, and a second turn gap m on. navd's first reading on the second turn's step, which brings its first pulse
  # out, is off m short; every reading after it is true. The second turn's junction isn't held there: two readings
  # agree on the true one, and on a tie the junction stays where it was, never passed on the outlier's one reading
  for off, gap, second in ((-20., 60., 'right'), (-20., 70., 'right'), (-25., 80., 'right'), (-20., 60., 'left'),
                           (-15., 65., 'left')):
    J1, J2 = 300., 300. + gap
    j2 = (90., 180., 2, 'present') if second == 'right' else (90., 0., 2, 'present')
    mans = [(J1, "turn", "right", junction_of("right")), (J2, "turn", second, j2), (J2 + 1200., "turn", "right", junction_of("right", 3))]
    sim = Sim(mans)
    def outlier(sim, instr, off=off, J2=J2, second=second, j2=j2, mans=mans, done=[]):  # noqa: B006
      if sim.navd.step != 1 or done:
        return instr, True
      done.append(sim.s)
      return nav_instruction(J2 - sim.s + off, "turn", second, remaining=2000. - sim.s + off, junction=j2,
                             ahead=[(mans[2][0] - sim.s + off, "turn", "right", junction_of("right", 3))]), True
    sim.nav_filter = outlier
    errors, y2 = [], (0.5 if second == 'left' else -0.5)
    while sim.s < J2 + 30.:
      sim.step(yaw_rate=-0.5 if J1 - 2. < sim.s < J1 + 12. else (y2 if J2 - 2. < sim.s < J2 + 12. else 0.))
      if J2 - 30. < sim.s < J2:
        errors.append(sim.distance() - (J2 - sim.s))
    assert_junction_kept(sim, J2, second, (off, gap, second), J2 - 30., errors)


def test_gps_outlier_on_the_last_reading_of_a_straight_on_step_doesnt_hold_the_turn_after_it_short():
  # a straight-on step ends 70 m before a right. navd's last reading on it, 75 m before the right, is 20 m short, and
  # the follower moves on to the right from it. The right is on the track navd's readings of it agree on, those before
  # the outlier (allManeuvers) and after it, not held 20 m short to its pass
  J, gap = 400., 70.
  mans = [(J - gap, "new name", "straight", (0., 0., 2)), (J, "turn", "right", junction_of("right")),
          (J + 1200., "turn", "left", junction_of("left", 2))]
  sim = Sim(mans)
  def outlier(sim, instr, done=[]):  # noqa: B006
    if J - sim.s > 75. or done:
      return instr, True
    done.append(sim.s)
    assert sim.navd.step == 0
    return nav_instruction(J - gap - sim.s - 20., "new name", "straight", remaining=1800. - sim.s - 20., junction=(0., 0., 2),
                           ahead=[(J - sim.s - 20., "turn", "right", junction_of("right")),
                                  (mans[2][0] - sim.s - 20., "turn", "left", junction_of("left", 2))]), True
  sim.nav_filter = outlier
  errors = []
  while sim.s < J + 30.:
    sim.step(yaw_rate=-0.5 if J - 2. < sim.s < J + 12. else 0.)
    if J - 30. < sim.s < J:
      errors.append(sim.distance() - (J - sim.s))
  assert_junction_kept(sim, J, 'right', 'none step', J - 30., errors)


def test_navd_moving_on_early_to_a_turn_the_same_way_without_junction_data_keeps_the_committed_turn():
  # a route without junction data: a right, and another right gap m on. navd moves on to the second at m before the
  # first: with no bearings the two read as the same junction, but the committed right's junction stays where it was
  # dead-reckoned: the same maneuver to the junction, its cap kept, the car not sped up into it, no CLEAR before it
  for at, gap in ((8., 40.), (14., 50.)):
    sim = Sim([(300., "turn", "right"), (300. + gap, "turn", "right"), (1500., "turn", "left", T_LEFT)])
    while sim.distance() > at + 3. or sim.s < 10.:
      sim.step()
    assert sim.tp.turn_committed
    n, turn = len(sim.trace), sim.tp.follower.maneuver_id
    def early(sim, instr, at=at, gap=gap):
      if 300. - at <= sim.s < 300. + gap - 10. and instr.maneuverModifier == 'right':
        return nav_instruction(300. + gap - sim.s, "turn", "right", remaining=1500. - sim.s), True
      return instr, True
    sim.nav_filter = early
    while sim.s < 305.:
      sim.step(yaw_rate=-0.5 if sim.s > 298. else 0.)
      if sim.s < 300.:
        assert sim.tp.follower.maneuver_id == turn, (at, gap, sim.s)
    assert_turn_held(sim, n)
    assert all(p.desire != CLEAR_DESIRE for s, _, p in sim.trace[n:] if s < 302.), (at, gap)


def test_a_fault_in_a_committed_turn_doesnt_hold_it_past_its_junction():
  # a fault (turnpilotd failing, or the car's longitudinal unknown) for 3 s from 3 m before the committed left, while the
  # car drives round it: the turn's distance and its yaw run on through the fault, so its desire is cleared or it is
  # passed where it would have been. No turnLeft is held, nor pulsed afresh, out on the road after it
  for how in ('error', 'noLong'):
    sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))])
    while sim.distance() > 3. or sim.s < 10.:
      sim.step()
    assert sim.tp.turn_committed
    sim.cruise = 4.3
    for _ in range(int(3. / DT_MDL)):
      if how == 'error':
        sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
      else:
        sim.long_control = None
      sim.step(yaw_rate=0.5 if sim.s < 312. else 0.)
    if how == 'error':
      del sim.tp.update
    sim.long_control = True
    while sim.s < 360.:
      sim.step(yaw_rate=0.5 if sim.s < 312. else 0.)
    late = [(s, p.desire) for s, _, p in sim.trace if s > 300. + 15. and p.desire == Desire.turnLeft]
    assert not late, (how, late[:3])
    assert sim.tp.follower.maneuver.side == -1, how  # the right after it is the maneuver now
  # a fault long enough that the turn is round and passed while it lasts: the turn's pulse still comes out of the model's
  # queue once TurnPilot runs again, and nothing is skipped or pulsed afresh
  sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))])
  while sim.distance() > 3. or sim.s < 10.:
    sim.step()
  sim.cruise = 4.3
  sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
  while sim.s < 316.:
    sim.step(yaw_rate=0.5 if sim.s < 312. else 0.)
  del sim.tp.update
  plans = [sim.step() for _ in range(int(2. / DT_MDL))]
  assert any(p.desire == CLEAR_DESIRE for p in plans)
  assert not any(p.desire == Desire.turnLeft or p.tooFastToTurn for p in plans)

# --- map stop control (map_controls.py) ---

def stop_at(sim, s_line, limit=60.):
  while not (sim.v < 0.05 and sim.s > s_line - 50.):
    sim.step()
    assert sim.t < limit, "never stopped"
  return sim.step()


def test_stop_sign_on_the_route_stops_and_hands_back():
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(200., 'stop')])
  sim.go_taps = False
  plan = stop_at(sim, 200.)
  assert -2. < 200. - sim.s < 5.
  assert plan.shouldStop and plan.state == State.holding and plan.reason == "Stopped at stop sign"
  assert plan.mapControl.kind == 'stop' and str(plan.mapControl.action) == 'held' and plan.mapControl.controlsStatus == 'ok'
  sim.run(1.5)
  plan = sim.trace[-1][2]
  assert plan.controlPrompt == 'stopSignTakeControl' and plan.reason == "Stop sign: take control to proceed"
  assert str(plan.mapControl.action) == 'awaitingDriver'
  held_s = sim.s
  sim.run(20.)
  assert sim.s - held_s < 0.05 and sim.trace[-1][2].shouldStop
  plan = sim.step(gas=True, engaged=True)  # a gas press is no go: the hold stays (the press disengages)
  assert plan.shouldStop and plan.controlPrompt == 'stopSignTakeControl'
  sim.go_taps, sim.take_over_on = True, ('stopSignTakeControl',)  # the driver takes over and drives away
  sim.run(10.)
  assert sim.s > 220. and not sim.trace[-1][2].shouldStop
  assert not any(p.controlAlert == 'stopOverrun' for _, _, p in sim.trace)


def test_signal_with_no_light_source_adds_nothing():
  # the car this week: no light source, so a mapped signal neither slows the car nor asks for anything (the model's own
  # braking handles the light)
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(200., 'signal')])
  plans = [sim.step() for _ in range(int(25. / DT_MDL))]
  assert sim.s > 300. and min(v for _, v, _ in sim.trace) >= 13. - 1e-6
  assert all(not p.shouldStop and p.speedCap > 20. and p.accelCap == NO_ACCEL_CAP and p.controlPrompt == 'none' for p in plans)


def test_sim_oracle_green_passes_and_needs_simulation_and_the_param():
  def lights(state):
    def make(sim):
      msg = messaging.new_message('turnpilotLights')
      msg.turnpilotLights.source = 'simOracle'
      msg.turnpilotLights.lights = [{'state': state, 'distance': 200. - sim.s, 'instance': 'trafficLight 1'}]
      return msg.as_reader().turnpilotLights
    return make
  # accepted: through on the green, with the reason saying where the light came from
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(200., 'signal')])
  sim.tp.oracle_ok = True
  sim.lights = lights('green')
  reasons = [sim.step().reason for _ in range(int(25. / DT_MDL))]
  assert sim.s > 210. and min(v for _, v, _ in sim.trace) > 12. and "Green (sim oracle)" in reasons
  # accepted, a red: stopped at the line, and held
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(200., 'signal')])
  sim.tp.oracle_ok = True
  sim.lights = lights('red')
  plan = stop_at(sim, 200.)
  assert plan.shouldStop and -2. < 200. - sim.s < 5. and plan.reason == "Red light (sim oracle)"
  # not accepted (no SIMULATION, or the param off): rejected out loud, and the light is unknown: nothing to stop for
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(200., 'signal')])
  sim.lights = lights('red')
  sim.run(25.)
  assert sim.s > 300. and not any(p.shouldStop for _, _, p in sim.trace)
  assert any(p.controlAlert == 'lightSourceRejected' for _, _, p in sim.trace)


def test_oracle_accepted_only_in_the_sim_with_the_param():
  from openpilot.selfdrive.turnpilot.turnpilotd import oracle_accepted
  assert oracle_accepted({'SIMULATION': '1'}, True)
  assert not oracle_accepted({}, True)
  assert not oracle_accepted({'SIMULATION': '1'}, False)
  assert not oracle_accepted({'SIMULATION': '0'}, True)


def test_fault_keeps_the_map_stop_latch_and_turning_off_clears_it():
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(200., 'stop')])
  stop_at(sim, 200.)
  held_s = sim.s
  sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
  plans = [sim.step() for _ in range(int(5. / DT_MDL))]
  assert all(p.fault == Fault.error and p.shouldStop for p in plans)
  assert sim.s - held_s < 0.05
  del sim.tp.update
  plan = sim.step()
  assert plan.shouldStop and plan.fault == Fault.none
  sim.tp.plan(sim.sm, scene(), False, False, long_control=True)  # TurnPilot turned off
  assert not sim.tp.map_controls.held


def test_skipped_turn_keeps_the_stop_after_it():
  # too fast for the turn at 90 m: skipped out loud, and the stop sign past it on the route still stops the car
  sim = Sim([(90., "turn", "right", junction_of("right")), (2000., "turn", "left")], cruise=20., controls=[(260., "stop")])
  sim.v = 20.
  plan = stop_at(sim, 260.)
  assert any(p.tooFastToTurn for _, _, p in sim.trace)
  assert plan.shouldStop and -2. < 260. - sim.s < 5.


def test_stop_sign_before_the_turn_means_the_turn_is_not_too_fast():
  # a stop sign 8 m before the junction: the car stops there anyway, so the turn isn't skipped as too fast
  sim = Sim([(150., "turn", "right", junction_of('right')), (2000., "turn", "left")], cruise=20., controls=[(142., 'stop')])
  sim.v = 20.
  plan = stop_at(sim, 142.)
  assert not any(p.tooFastToTurn for _, _, p in sim.trace)
  assert plan.shouldStop


def test_released_hold_at_standstill_pulses_the_turn_again():
  # BeamNG ped_crosswalk_right: held at the hold line for a walker, standing longer than the model's pulse lasts; the walker
  # leaves and the hold lets go. The model doesn't pull away on its own: the turn pulses again at once, at standstill
  sim = Sim([(300., "turn", "right"), (1500., "turn", "left")])
  walker = {'on': True}

  def objects(sim):
    # detected on every 10 Hz detector run: a steady walker
    return ({'cls': 'pedestrian', 'x': 300. - sim.s + 3., 'y': -5., 'age': 5 + int(sim.t * 10.)},) if walker['on'] else ()
  while not (sim.v < 0.05 and sim.s > 250.):
    sim.step(objects(sim))
    assert sim.t < 60.
  assert sim.trace[-1][2].reason == "pedestrian in crosswalk"
  for _ in range(int(5. / DT_MDL)):
    sim.step(objects(sim))
  walker['on'] = False
  start = len(sim.trace)
  sim.run(3.)
  after = sim.trace[start:]
  pulse = next(i for i, (_, _, p) in enumerate(after) if p.desire == Desire.turnRight)
  assert after[pulse][1] < 0.3, "the turn pulsed only once the car moved"


# --- unprotected turns (map_controls.py): a left across the road ahead's traffic stops at the hold line for the driver ---

TURNS_AHEAD = [(300., "turn", "left", junction_of("left")), (1500., "turn", "right", junction_of("right"))]


def test_unprotected_left_stops_before_the_hold_line_and_hands_back():
  sim = Sim(TURNS_AHEAD)
  sim.go_taps = False
  plan = stop_at(sim, 300.)
  assert sim.distance() >= HOLD_LINE_M and 300. - sim.s >= HOLD_LINE_M  # stopped before the line, never past it
  assert plan.shouldStop and plan.state == State.holding and plan.controlPrompt == 'unprotectedTurnTakeControl'
  assert plan.mapControl.kind == 'unprotectedTurn' and str(plan.mapControl.action) == 'awaitingDriver'
  assert plan.reason == "Unprotected left: take control"
  assert any(p.reason == "Unprotected left ahead" for _, _, p in sim.trace)
  held_s = sim.s
  sim.run(15.)
  assert sim.s - held_s < 0.05 and sim.trace[-1][2].shouldStop  # no detection is no clearance, and no go of its own
  plan = sim.step(gas=True, engaged=True)
  assert plan.shouldStop and plan.controlPrompt == 'unprotectedTurnTakeControl'  # nor a gas go
  sim.go_taps = True  # the driver takes over
  sim.run(15., turn_at=300.)
  assert sim.s > 320.


def test_the_oncoming_hold_counts_a_car_up_the_road_with_the_car_yawed_into_the_left():
  # DESIGN phase 4 section 2 (the road-frame band): the car stands at its hold line 30 deg into the left (yawed there on the
  # approach by its pulse); an oncoming car 50 m up the road is 22 m to its right in its own frame. turnpilotd turns the
  # scene back by the car's heading change toward the turn since its desire went out, so it still holds
  sim = Sim(TURNS_AHEAD)
  sim.go_taps = False
  while not sim.tp.desires.started:
    sim.step()
  assert sim.distance() <= TURN_DESIRE_LEAD_M + 1.
  yaw_rate = math.radians(30.) / ((sim.distance() - HOLD_LINE_M) / sim.v)  # 30 deg left by the hold line at this speed
  while sim.v > 0.05 or sim.distance() > HOLD_LINE_M + 5.:
    sim.step(yaw_rate=yaw_rate if sim.tp.approach_yaw_deg < 30. else 0.)
  assert abs(sim.tp.approach_yaw_deg - 30.) < 1.
  a = math.radians(30.)
  x_road, y_road = 50., 3.6
  car_frame = {'cls': 'car', 'x': x_road * math.cos(a) + y_road * math.sin(a), 'y': -x_road * math.sin(a) + y_road * math.cos(a),
               'vx': -12. * math.cos(a), 'vy': 12. * math.sin(a), 'inPath': False, 'age': 30}
  assert car_frame['y'] < -20.
  sim.step([car_frame])
  assert sim.tp.scene.junction is not None and sim.tp.scene.junction.reason == "waiting for oncoming"


def test_left_without_junction_data_is_unprotected():
  sim = Sim([(300., "turn", "left"), (1500., "turn", "right")])
  sim.go_taps = False
  plan = stop_at(sim, 300.)
  assert plan.controlPrompt == 'unprotectedTurnTakeControl' and 300. - sim.s >= HOLD_LINE_M


def test_left_from_a_t_stem_or_a_right_has_no_unprotected_stop():
  for maneuvers in ([(300., "turn", "left", junction_of("left", opposing='none')), (1500., "turn", "right", junction_of("right"))],
                    [(300., "turn", "right", junction_of("right")), (1500., "turn", "left", junction_of("left"))]):
    sim = Sim(maneuvers)
    sim.go_taps = False
    sim.run(40., turn_at=300.)
    assert sim.s > 320. and not any(p.shouldStop or p.controlPrompt != 'none' for _, _, p in sim.trace if p.maneuverDistance > -20.)


def test_a_left_at_a_signal_is_unprotected_and_stops_at_its_hold_line():
  # a signal at the junction holds nothing (SIGNAL_PROTECTS_LEFT): the left stops at its own hold line, not the signal's
  sim = Sim(TURNS_AHEAD, controls=[(289., 'signal')])
  sim.go_taps = False
  plan = stop_at(sim, 289.)
  assert plan.mapControl.kind == 'unprotectedTurn' and 300. - HOLD_LINE_M - 4. < sim.s < 300. - HOLD_LINE_M
  assert not any(p.mapControl.kind == 'signal' and p.shouldStop for _, _, p in sim.trace)


def test_left_too_fast_to_make_is_skipped_out_loud_not_braked_for():
  sim = Sim([(90., "turn", "left", junction_of("left")), (2000., "turn", "right")], cruise=20.)
  sim.v = 20.
  sim.go_taps = False
  sim.run(10.)
  assert any(p.tooFastToTurn for _, _, p in sim.trace)
  assert not any(p.shouldStop or p.controlAlert == 'cannotStopComfortably' or p.mapControl.kind == 'unprotectedTurn'
                 for _, _, p in sim.trace[5:])
  assert sim.s > 120.


def test_left_whose_stop_cant_be_made_is_skipped_though_the_turn_could_be():
  # first seen 31 m out at 10 m/s: slowing to the turn's speed by the junction is fine, but stopping at its hold line
  # needs -2.3 m/s^2. An unprotected left is only taken from that stop: skipped out loud and rerouted, never braked for
  for junction, skipped in ((junction_of("left"), True), (T_LEFT, False)):
    sim = Sim([(31., "turn", "left", junction), (2000., "turn", "right")], cruise=10.)
    sim.v = 10.
    sim.go_taps = False
    sim.run(6., turn_at=31.)
    assert any(p.tooFastToTurn for _, _, p in sim.trace) == skipped
    assert not any(p.shouldStop or p.controlAlert == 'cannotStopComfortably' or p.mapControl.kind == 'unprotectedTurn'
                   for _, _, p in sim.trace)
    assert any(p.desire == Desire.turnLeft for _, _, p in sim.trace) != skipped


def oncoming_from(sim, s_world, speed=10.):
  """A car coming down the oncoming lanes from s_world (along the route) now, at speed: objects(sim) for the scene."""
  t0 = sim.t

  def objects(sim):
    return ({'cls': 'car', 'x': s_world - speed * (sim.t - t0) - sim.s, 'y': 5., 'vx': -speed},)
  return objects


def go(sim, objects=lambda sim: ()):
  """The go at an unprotected left's hold line (the driver's tap, until the gap logic gives it)."""
  sim.step(objects(sim), gas=True)


def test_the_driver_taking_over_at_the_hold_line_is_never_stopped_again_in_the_junction():
  # a car coming, the driver takes over at the unprotected left's hold line and goes, then engages again inside the
  # junction: the detector's oncoming hold for that maneuver is the driver's now (scene_policy: released), never a stop in
  # the junction
  sim = Sim(TURNS_AHEAD)
  sim.go_taps = False
  stop_at(sim, 300.)
  car = oncoming_from(sim, 400.)
  sim.run(1., objects=car)
  sim.go_taps = True
  plans = []
  for _ in range(int(4. / DT_MDL)):
    plans.append(sim.step(car(sim)))
  assert sim.s > 300. - HOLD_LINE_M + 2. and sim.tp.scene.released_id is not None
  assert not any(p.shouldStop for p in plans[-20:])


def test_the_turn_pulses_on_the_go_frame_itself():
  # BeamNG unprotected_left_visible: let go after its hold, the car drove straight across. The model only takes a rising
  # edge, so the plan that first lets the car go carries a fresh turnLeft edge, whether the hold let go at once or long
  # after (here a red's, on the green)
  for wait in (0.3, 10.):
    sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
    plan = go_from_a_red(sim, wait=wait)
    assert sim.tp.desires.started and plan.desire == Desire.turnLeft and sim.trace[-2][2].desire != Desire.turnLeft, wait


def test_fault_on_the_approach_keeps_the_unprotected_stop():
  # turnpilotd failing on every frame while braking for an unprotected left: the faulted plans carry its stop on, dead
  # reckoned, to the latch before the line (map stops outlive faults), never dropped with the route the fault lost
  sim = Sim(TURNS_AHEAD)
  sim.go_taps = False
  while not (sim.trace and sim.trace[-1][2].mapControl.kind == 'unprotectedTurn' and sim.trace[-1][2].speedCap < sim.v):
    sim.step()
    assert sim.s < 290.
  sim.tp.update = mock.Mock(side_effect=RuntimeError("synthetic failure"))
  plans = [sim.step() for _ in range(int(25. / DT_MDL))]
  assert all(p.fault == Fault.error for p in plans)
  assert sim.v < 0.05 and 300. - sim.s >= HOLD_LINE_M and plans[-1].shouldStop



def test_a_gas_brush_on_the_approach_never_releases_the_unprotected_lefts_stop():
  # oncoming traffic in a steady stream; the driver brushes the gas on the final approach to the unprotected left (a
  # disengage, then engaged again): the map's stop at the hold line still stops the car and hands back, whatever the
  # brush did to the detector's hold
  def stream(sim):
    return ({'cls': 'car', 'x': 330. - sim.s, 'y': 5., 'vx': -10., 'age': 30},)
  for brush_at in (12., 10.5):
    sim = Sim(TURNS_AHEAD)
    sim.go_taps = False
    frames = 0
    while not (sim.v < 0.05 and sim.s > 250.):
      brush = 300. - sim.s <= brush_at and frames < 3
      frames += brush
      sim.step(stream(sim), gas=brush)
      assert sim.t < 120.
    assert frames == 3
    sim.run(2., objects=stream)
    plan = sim.trace[-1][2]
    assert plan.shouldStop and plan.controlPrompt == 'unprotectedTurnTakeControl', brush_at
    held_s = sim.s
    sim.run(4., objects=stream)
    assert sim.s - held_s < 0.05, brush_at


def test_a_handback_on_the_approach_keeps_the_unprotected_lefts_stop():
  # the detector stops 12, 6 or 3 m before the unprotected left's stop point: openpilot steers on through the 3 s soft
  # disable with the turn's desire out. The stop at the line stays, as a stop sign's would: the car never rolls past the
  # hold line into the blind junction on the plan
  for before_stop in (12., 6., 3.):
    sim = Sim(TURNS_AHEAD)
    sim.go_taps = False
    while 291. - sim.s > before_stop or sim.s < 10.:
      sim.step()
    plans = []
    for _ in range(int((6. + DETECTOR_DEADLINE_S) / DT_MDL)):
      plans.append(sim.step(available=False))
    assert any(p.handback != Handback.none for p in plans)
    assert 300. - sim.s > HOLD_LINE_M, before_stop
    assert str(plans[-1].mapControl.kind) == 'unprotectedTurn' and plans[-1].shouldStop, before_stop


def test_the_detector_back_after_a_short_outage_still_stops_at_the_unprotected_left():
  # the detector drops out on the approach to an unprotected left and comes back before openpilot lets go: the turn is
  # taken up, never driven across the blind junction without its stop at the line and the driver's go
  for outage, start in ((2., 25.), (2.5, 20.), (2., 16.), (1.5, 14.)):
    sim = Sim(TURNS_AHEAD)
    sim.go_taps = False
    while 300. - sim.s > start or sim.s < 10.:
      sim.step()
    for _ in range(int(outage / DT_MDL)):
      sim.step(available=False)
    while not (sim.v < 0.05 or sim.s > 300.):
      sim.step()
    assert sim.s < 300. - HOLD_LINE_M, (outage, start, 300. - sim.s)
    plan = sim.run(2.) or sim.trace[-1][2]
    assert plan.shouldStop and plan.controlPrompt == 'unprotectedTurnTakeControl', (outage, start)


def test_an_unprotected_left_taken_up_too_close_to_stop_for_is_skipped_out_loud():
  # the detector drops out with the left's desire out but its stop not yet engaged (the car fast), and comes back where
  # the stop at the line can't be made at -2.0: the turn isn't driven across the blind junction without its stop, it is
  # skipped out loud with its desire cleared, as a dropped turn too close to pulse again is
  sim = Sim(TURNS_AHEAD, cruise=8.)
  sim.go_taps = False
  while not (sim.tp.desires.started and sim.tp.desires.maneuver_id == sim.tp.follower.maneuver_id):
    sim.step()
  sim.tp.map_controls.turn = None  # its stop not engaged yet when the detector goes (a car faster than its engage distance)
  for _ in range(int(2.5 / DT_MDL)):
    sim.step(available=False)
    sim.tp.map_controls.turn = None
  sim.v = 8.
  plans = [sim.step() for _ in range(int(1. / DT_MDL))]
  assert any(p.tooFastToTurn for p in plans)
  assert any(p.desire == CLEAR_DESIRE for p in plans) or not sim.tp.desires.side
  assert not any(p.desire == Desire.turnLeft for p in plans[3:])


def test_a_slow_approach_never_times_the_turn_out_before_the_go():
  # creeping to a red before a left at 2 m/s (a queue): far more than 15 s of moving time on the turn's desire before
  # the car stands at the line. The turn is still on at the green's go: its pulse goes in the go frame, and it stays out
  # to the junction. A turn desire is over when the turn is (turned, or passed), never on a clock
  for cruise in (2., 1.5):
    sim = Sim(LEFT_AT_A_RED, cruise=cruise, controls=[(290., 'signal')])
    sim.v = cruise
    plan = go_from_a_red(sim, limit=400.)
    assert plan.desire == Desire.turnLeft, cruise
    while sim.s < 300.:
      plan = sim.step()
      assert plan.desire in (Desire.turnLeft, Desire.none), cruise
    assert sim.tp.turn_committed, cruise


def test_tiles_without_controls_are_said_out_loud():
  # navd on tiles with no control data (the plain west_coast_usa sim tiles, no controls_built marker) reports
  # 'unsupported': every plan on the route says map stops are unavailable, which selfdrived chimes and the UI shows,
  # never a route that just looks clear of stops
  from openpilot.selfdrive.ui.lib.turnpilot import status_text
  sim = Sim([(600., "turn", "right"), (1500., "turn", "left")], controls_status='unsupported')
  plans = [sim.step() for _ in range(int(20. / DT_MDL))]
  routed = [p for p in plans if p.routeValid]
  assert len(routed) > len(plans) // 2
  assert all(p.controlAlert == 'mapControlsUnavailable' and str(p.mapControl.controlsStatus) == 'unsupported' for p in routed)
  assert any(p.reason == 'Map stops/signals: unsupported' for p in routed)
  assert status_text(routed[-1], True) == ('Map stops unavailable', 'unsupported')
  # a navd from before controls ('none') has nothing to say
  sim = Sim([(600., "turn", "right"), (1500., "turn", "left")])
  plans = [sim.step() for _ in range(int(20. / DT_MDL))]
  assert not any(p.controlAlert == 'mapControlsUnavailable' for p in plans)


def stand(sim, seconds, **kw):
  """The model holds the car standing for seconds, engaged (nothing in the plan does): its shouldStop, no motion planned.
  lead_prob, lead_x: its lead."""
  lead, lead_x = kw.pop('lead_prob', None), kw.pop('lead_x', 30.)
  plans = []
  sim.model_holds = True
  for _ in range(int(round(seconds / DT_MDL))):
    sim.model_kw = {} if lead is None else {'lead_prob': lead, 'lead_x': lead_x}
    sim.v = 0.
    plans.append(sim.step(engaged=True, **kw))
  sim.model_holds = False
  return plans


def test_stood_engaged_with_nothing_holding_it_hands_back_and_never_goes():
  """BeamNG route_demo, 2026-10-28: after the reroute the model stopped on its own (shouldStop, no lead) on an empty road,
  and the car stood 470 s. Standing engaged with no TurnPilot hold, no lead and no map control within 30 m: said on the
  status line after STOPPED_NOTE_S, and handed back loudly after STOPPED_HANDBACK_S (carStoppedTakeControl), long enough
  for a red the map doesn't know (CRITIQUE REQUIRED 6: no handback at every unmapped red). It never goes itself."""
  sim = Sim([(2000., 'turn', 'left', T_LEFT)])
  sim.run(1.)
  plans = stand(sim, STOPPED_NOTE_S + 0.2)
  assert all(p.controlPrompt == 'none' for p in plans) and plans[-1].reason == STOPPED_NOTE
  plans = stand(sim, STOPPED_HANDBACK_S - STOPPED_NOTE_S - 0.4)
  assert all(p.controlPrompt == 'none' for p in plans)
  p = stand(sim, 0.4)[-1]
  assert p.controlPrompt == 'carStoppedTakeControl' and p.reason == STOPPED_REASON == "Car stopped: take control"
  assert not p.shouldStop and p.speedCap > 20. and p.accelCap == NO_ACCEL_CAP and p.desire == Desire.none  # never pushes
  sim.step(engaged=False, gas=True)  # the driver took over (the press disengages): the wait starts over
  assert all(p.controlPrompt == 'none' for p in stand(sim, STOPPED_HANDBACK_S - 0.2))
  assert stand(sim, 0.4)[-1].controlPrompt == 'carStoppedTakeControl'

  # not while disengaged, with a lead, or with the brake on
  sim = Sim([(2000., 'turn', 'left', T_LEFT)])
  sim.run(1.)
  plans = []
  for _ in range(int((STOPPED_HANDBACK_S + 1.) / DT_MDL)):
    sim.v = 0.
    plans.append(sim.step(engaged=False))
  assert all(p.controlPrompt == 'none' for p in plans)
  assert all(p.controlPrompt == 'none' for p in stand(sim, STOPPED_HANDBACK_S + 1., lead_prob=0.9, lead_x=7.))  # queued
  assert all(p.controlPrompt == 'none' for p in stand(sim, STOPPED_HANDBACK_S + 1., lead_prob=0.6, lead_x=STOPPED_LEAD_M - 1.))


def test_stood_at_a_mapped_signal_the_model_wont_leave_hands_back_after_the_long_stand():
  """CRITIQUE REQUIRED 6: a mapped signal TurnPilot doesn't hold for (no light source) used to silence the stopped
  handback for good: a model stall at a green was a silent indefinite stand. Within STOPPED_CONTROL_M of a map control,
  standing NEAR_CONTROL_STAND_S (longer than a red) hands back (carStoppedTakeControl, "Stopped at the signal: take
  control"). A stop sign queue the same."""
  sim = Sim([(2000., 'turn', 'left', T_LEFT)], controls=[(20., 'signal')])
  sim.run(0.2)
  plans = stand(sim, NEAR_CONTROL_STAND_S - 0.2)
  assert all(p.controlPrompt == 'none' for p in plans)
  p = stand(sim, 0.4)[-1]
  assert p.controlPrompt == 'carStoppedTakeControl' and p.reason == SIGNAL_STOPPED_REASON
  assert NEAR_CONTROL_STAND_S > STOPPED_HANDBACK_S
  sim = Sim([(2000., 'turn', 'left', T_LEFT)], controls=[(28., 'stop')])
  sim.run(0.2)
  assert all(p.controlPrompt == 'none' for p in stand(sim, NEAR_CONTROL_STAND_S - 0.2))  # a stop sign 25 m on: its queue
  assert stand(sim, 0.4)[-1].controlPrompt == 'carStoppedTakeControl'


def test_a_lead_too_far_ahead_to_hold_the_car_doesnt_keep_it_standing_silently():
  """The same route_demo stand: the model saw a lead of prob ~0.6 at 21-28 m the whole 470 s (radarState too), standing.
  openpilot stands STOP_DISTANCE (6 m) behind a lead it is queued for: one that far ahead is not what holds the car, so
  it is still handed back."""
  sim = Sim([(2000., 'turn', 'left', T_LEFT)])
  sim.run(1.)
  plans = stand(sim, STOPPED_HANDBACK_S + 0.2, lead_prob=0.6, lead_x=25.)
  assert plans[-1].controlPrompt == 'carStoppedTakeControl' and plans[-1].reason == STOPPED_REASON
  assert 6. < STOPPED_LEAD_M < 21.  # past long_mpc.STOP_DISTANCE, short of the recorded lead


def test_a_turn_lane_a_map_stop_leaves_no_room_for_is_skipped_out_loud_never_turned_from_the_wrong_lane():
  """The map's lane list puts the car two lanes right of the left turn's lane, and a red 100 m before the turn leaves no
  room to change before it at speed nor after its go: the turn is skipped as too fast to turn is (navd reroutes), the plan
  saying why (laneChangeWait unreachable), and its desire never goes out."""
  sim = Sim([(400., 'turn', 'left', T_LEFT)], cruise=11., nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(2, 0),
            controls=[(300., 'signal')])
  sim.tp.oracle_ok = True
  sim.lights = oracle_lights('red', 300.)  # read red from the start: a stop the car makes
  sim.run(25.)
  plans = [p for _, _, p in sim.trace]
  first = next(i for i, p in enumerate(plans) if p.tooFastToTurn)
  s_skip = sim.trace[first][0]
  assert 400. - s_skip > 150., s_skip  # decided once it was so, well before the junction
  assert plans[first].laneChangeWait == 'unreachable' and plans[first].laneChangeWant == 2
  assert any(p.laneChangeWait == 'beforeWindow' for p in plans[:first])
  assert all(p.desire == Desire.none for p in plans)
  assert all(p.tooFastToTurn for p in plans[first:] if p.maneuverDistance > 0.)


# a left at a signal from a T's stem (no oncoming approach: no unprotected-turn stop), read red, then green
LEFT_AT_A_RED = [(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))]


def go_from_a_red(sim, s_signal=290., wait=1., limit=60., desire=Desire.turnLeft):
  """Held at the signal s_signal, red in the sim's light oracle, for wait s, then its green: the hold lets go at
  standstill, pulsing desire. Returns the plan that let go."""
  sim.go_taps = False
  sim.tp.oracle_ok = True
  light = {'state': 'red'}

  def lights(sim):
    msg = messaging.new_message('turnpilotLights')
    msg.turnpilotLights.source = 'simOracle'
    msg.turnpilotLights.lights = [{'state': light['state'], 'distance': s_signal - sim.s, 'instance': 'trafficLight 1'}]
    return msg.as_reader().turnpilotLights
  sim.lights = lights
  stop_at(sim, s_signal, limit=limit)
  sim.run(wait)
  light['state'] = 'green'
  while sim.trace[-1][2].shouldStop:
    sim.step(engaged=True)
    assert sim.t < limit + wait + 5.
  plan = sim.trace[-1][2]
  assert plan.desire == desire  # the go pulse, at standstill
  return plan


def go_from_the_hold(sim):
  """LEFT_AT_A_RED held at its red 10 m before the junction, then the green: the go from a standstill."""
  return go_from_a_red(sim)


def test_a_turn_from_a_standstill_driven_straight_past_its_junction_is_missed_out_loud():
  """BeamNG standstill sweep, 2026-10-28: released from a stop, the car went straight on 7 of 21 times, with the model's
  turn desire out and nothing said, then drove on displaced into the oncoming lane. Past the junction (the follower's pass)
  having turned under 15 deg, the turn is missed: said with a chime ('Turn Missed', 'Rerouting', the too-fast-to-turn
  alert) and navd reroutes at once. Nothing brakes for it."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go_from_the_hold(sim)
  plans = []
  while sim.s < 340.:
    plans.append(sim.step(engaged=True))  # straight on: no yaw
  passed = next(i for i, p in enumerate(plans) if p.turnMissed)
  assert plans[passed].maneuverDistance < -10.  # at the pass, not before it
  assert not any(p.turnMissed or p.tooFastToTurn for p in plans[:passed])
  missed = [p for p in plans[passed:] if p.turnMissed]
  assert 2.5 <= len(missed) * DT_MDL <= 3.5 and all(p.reason == TURN_MISSED_REASON for p in missed)
  assert not any(p.tooFastToTurn for p in plans)  # navd's skip match is for a turn ahead, never one already passed
  # never braked for: no stop, no accel cap under the post-go profile's (positive), and no speed cap under the turn's own
  # while it is said (the profile's ends at the pass)
  assert all(not p.shouldStop and p.accelCap >= 1.0 for p in plans)
  assert all(p.speedCap >= plans[passed - 1].speedCap - 1e-3 for p in missed)
  assert not any(p.turnMissed for p in [sim.step(engaged=True) for _ in range(int(10. / DT_MDL))])  # said once

  # taken: turned through it, nothing said
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go_from_the_hold(sim)
  plans, turned = [], 0.
  while sim.s < 340.:
    plans.append(sim.step(engaged=True, yaw_rate=0.5 if 300. - sim.s < 10. and turned < 90. else 0.))
    turned = max(turned, sim.tp.desires.turned_deg)
  assert turned > 75. and not any(p.turnMissed for p in plans)


def test_stalled_in_the_junction_after_the_go_hands_back_and_never_goes():
  """BeamNG standstill sweep, 2026-10-28: 2 of 21 turns released from a stop stood 20 s mid-junction, 46-78 deg round,
  with nothing said: the stop line it was released from is within 30 m, so the empty-road handback never came. Standing
  more than 3 s after the go, 20-80 deg into the turn, with no TurnPilot hold and no close lead, it hands back loudly
  (stalledTakeControl) and says why. It never goes itself."""
  def into_the_turn(sim, deg):
    go_from_the_hold(sim)
    while sim.tp.desires.turned_deg < deg:
      sim.step(engaged=True, yaw_rate=0.5)
      assert sim.t < 120.

  def stand(sim, seconds, **kw):
    plans = []
    for _ in range(int(round(seconds / DT_MDL))):
      sim.v = 0.  # the model stands the car: nothing in the plan does
      plans.append(sim.step(engaged=True, **kw))
    return plans

  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  into_the_turn(sim, 45.)
  plans = stand(sim, STALL_PROMPT_S - 0.2)
  assert all(p.controlPrompt == 'none' for p in plans)
  before = plans[-1]
  p = stand(sim, 0.4)[-1]
  assert p.controlPrompt == 'stalledTakeControl' and p.reason == STALLED_REASON == "Stalled in the turn: take control"
  assert not p.shouldStop and p.accelCap == NO_ACCEL_CAP and p.speedCap >= before.speedCap - 1e-3  # never pushes
  assert p.desire == Desire.none
  sim.step(engaged=False, gas=True)  # the driver took over (the press disengages): the wait starts over
  assert all(p.controlPrompt == 'none' for p in stand(sim, STALL_PROMPT_S - 0.2))
  assert stand(sim, 0.4)[-1].controlPrompt == 'stalledTakeControl'

  # not with a lead close ahead, nor under 20 deg into the turn (the model may still be waiting to start it)
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  into_the_turn(sim, 45.)
  sim.model_kw = {'lead_prob': 0.9, 'lead_x': STOPPED_LEAD_M - 5.}
  assert all(p.controlPrompt == 'none' for p in stand(sim, STALL_PROMPT_S + 1.))
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  into_the_turn(sim, 10.)
  assert all(p.controlPrompt == 'none' for p in stand(sim, STALL_PROMPT_S + 1.))

  # nor while TurnPilot holds the car: a pedestrian in the turn's path
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  into_the_turn(sim, 45.)
  walker = ({'cls': 'pedestrian', 'x': 4., 'y': 0.1, 'vx': 0., 'vy': 0., 'inPath': True},)
  plans = stand(sim, STALL_PROMPT_S + 1., objects=walker)
  assert any(p.shouldStop for p in plans) and all(p.controlPrompt == 'none' for p in plans if p.shouldStop)


def test_a_shallow_turn_taken_from_a_stop_is_not_called_missed():
  """Skeptic probe, 2026-10-28: a turn under 20 deg clears its desire at max(angle - 10, 0.75 angle), under 15 deg, and
  turned_deg stops there. Released from a stop (a red, here) 20 m before it at cruise 10 m/s and yawed through its full angle, the
  car passed it 'turned under 15 deg' and the turn was called missed: the chime, 'Rerouting', and navd's reroute. A turn
  whose desire cleared at its angle was taken; a car that goes straight past the same junction is still missed."""
  def run(angle, yaw):
    sim = Sim([(300., "turn", "left", junction_of("left", angle=angle, opposing='none')), (1500., "turn", "right", junction_of("right"))],
              cruise=10., controls=[(280., 'signal')])
    go_from_a_red(sim, 280., wait=2.5, limit=200.)
    assert sim.tp.go_id is not None
    plans, yawed = [], 0.
    while sim.s < 345.:
      yr = 0.3 if yaw and 300. - sim.s < 6. and yawed < angle else 0.
      yawed += math.degrees(yr * DT_MDL)
      plans.append(sim.step(engaged=True, yaw_rate=yr))
      assert sim.t < 400.
    return plans, yawed

  for angle in (12., 15., 17., 19.):
    plans, yawed = run(angle, True)
    assert yawed >= angle and not any(p.turnMissed for p in plans), angle
  plans, _ = run(15., False)  # straight on past it: missed
  assert any(p.turnMissed for p in plans)


def test_a_hard_brake_is_raised_only_for_a_confirmed_pedestrian():
  # one detector frame of a pedestrian 10 m ahead at 10 m/s: braked for on the curve, no hard brake raised; a confirmed
  # one 10 m ahead: braked hard, and the plan says so (controlAlert hardBrake)
  sim = Sim([(2000., "turn", "right")], cruise=10.)
  sim.v = 10.
  plans = [sim.step(objects=[{'cls': 'pedestrian', 'x': 10., 'y': 0., 'inPath': True, 'age': 2, 'conf': 0.95}])]
  plans += [sim.step() for _ in range(20)]
  assert all(plan_accel(p.accelCap, p.speedCap, 10.) >= -2. - 1e-6 and p.controlAlert != 'hardBrake' for p in plans)
  sim = Sim([(2000., "turn", "right")], cruise=10.)
  sim.v = 10.
  plans = [sim.step(objects=[{'cls': 'pedestrian', 'x': 10. - 0.5 * k, 'y': 0., 'inPath': True, 'age': 5 + k, 'conf': 0.9}])
           for k in range(6)]
  assert plans[-1].controlAlert == 'hardBrake' and plans[-1].reason == "pedestrian in path"
  assert plan_accel(plans[-1].accelCap, plans[-1].speedCap, 10.) < -2.


def oracle_lights(state, s_signal):
  def make(sim):
    msg = messaging.new_message('turnpilotLights')
    msg.turnpilotLights.source = 'simOracle'
    msg.turnpilotLights.lights = [{'state': state, 'distance': s_signal - sim.s, 'instance': 'trafficLight 1'}]
    return msg.as_reader().turnpilotLights
  return make


def test_a_too_fast_turn_just_past_a_green_or_unchecked_signal_is_still_skipped():
  # a signal 8 m before the junction the car passes through on a green (or with no light source): it doesn't stop there,
  # so the turn is too fast and is skipped out loud, as with no signal (a stop sign there would be a stop: 1505)
  for state in (None, 'green'):
    sim = Sim([(100., "turn", "right", junction_of('right')), (2000., "turn", "left")], cruise=20., controls=[(92., 'signal')])
    sim.v = 20.
    if state is not None:
      sim.tp.oracle_ok = True
      sim.lights = oracle_lights(state, 92.)
    sim.run(4.)
    assert any(p.tooFastToTurn for _, _, p in sim.trace), state
    assert not any(p.shouldStop for _, _, p in sim.trace)


def test_the_lane_window_ignores_a_signal_the_car_passes_through():
  # 1875's road with the signal on a green: no stop there, so the window is the one at speed, and the turn lane is
  # reachable before the turn (never laneChangeWait unreachable, never skipped)
  sim = Sim([(400., 'turn', 'left', T_LEFT)], cruise=11., nav_lanes={0: THREE_LANES_LEFT_TURN}, model_lanes=(2, 0),
            controls=[(300., 'signal')])
  sim.tp.oracle_ok = True
  sim.lights = oracle_lights('green', 300.)
  sim.run(12.)
  plans = [p for _, _, p in sim.trace]
  assert not any(p.tooFastToTurn or p.laneChangeWait == 'unreachable' or p.laneChangeWait == 'afterStop' for p in plans)
  assert sim.tp.map_controls.stop_ahead() is None


def test_disengage_on_accelerator_on_is_the_default_and_turnpilot_runs_with_it():
  # any gas press disengages (DisengageOnAccelerator, default on): TurnPilot runs with it, never faulted for it, and a gas
  # press at a held stop sign is the driver taking over, never a go
  import re
  from pathlib import Path
  header = (Path(__file__).parents[3] / 'common' / 'params_keys.h').read_text()
  assert re.search(r'\{"DisengageOnAccelerator", \{PERSISTENT, BOOL, "1"\}\}', header)
  sim = Sim([(1000., "turn", "right"), (2000., "turn", "left")], controls=[(200., 'stop')])
  plan = stop_at(sim, 200.)
  assert plan.fault == Fault.none and plan.shouldStop
  sim.run(1.5)
  assert sim.trace[-1][2].controlPrompt == 'stopSignTakeControl'
  plans = [sim.step(gas=True)] + [sim.step() for _ in range(10)]
  assert all(p.shouldStop and p.fault == Fault.none and p.desire == Desire.none for p in plans)


def test_preflight_fails_with_disengage_on_accelerator_off():
  from openpilot.selfdrive.turnpilot.preflight import FAIL, PASS, check_gas_disengages
  assert check_gas_disengages(True).status == PASS
  assert check_gas_disengages(False).status == FAIL


def test_a_gas_press_while_engaged_is_never_a_go_through_the_detectors_hold():
  # held at a left's hold line for a car coming (the detector's hold: a T's stem, no map stop): a press with openpilot
  # still engaged (DisengageOnAccelerator off, or the frame before its disengage) releases nothing; the driver driving
  # (openpilot disengaged) at the line does, as taking over
  sim = Sim([(300., "turn", "left", T_LEFT), (1500., "turn", "right", junction_of("right"))])
  def car(sim):  # a steady stream: always one 30 m past the junction, coming at 10 m/s
    return ({'cls': 'car', 'x': 330. - sim.s, 'y': 5., 'vx': -10., 'age': 30},)
  while not (sim.v < 0.05 and sim.s > 250.):
    sim.step(car(sim))
    assert sim.t < 120.
  plans = [sim.step(car(sim), gas=True, engaged=True) for _ in range(20)]
  assert all(p.shouldStop and p.reason == "waiting for oncoming" for p in plans) and sim.tp.scene.released_id is None
  plan = sim.step(car(sim), gas=True, engaged=False)
  assert not plan.shouldStop and sim.tp.scene.released_id is not None


def test_the_go_from_a_standstill_drives_the_graduated_post_go_profile():
  """DESIGN phase 4 section 3 (G3): from the go a turn let go of at standstill is capped by the heading it has turned,
  DesireTiming.turned_deg: accel 1.0 m/s^2 until 45 deg, speed 3.0 m/s until 15 deg ramping to 4.3 at 45, blended into the
  corner cap by 60, then nothing. It only lowers speed: no stop, no braking past -2.0."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go = go_from_the_hold(sim)
  assert go.accelCap == 1.0 and math.isclose(go.speedCap, 3.0) and not go.shouldStop
  rows = []
  while sim.tp.post_go is not None:
    v0, pg = sim.v, sim.tp.post_go
    p = sim.step(engaged=True, yaw_rate=0.3 if sim.v > 0.5 else 0.)
    d, turned = sim.tp.desires, pg.turned_deg
    if d.maneuver_id == sim.tp.follower.maneuver_id and d.started and not d.done:
      assert math.isclose(turned, d.turned_deg, abs_tol=1e-6)  # keyed on DesireTiming's heading while it tracks the turn
    rows.append((turned, p, (sim.v - v0) / DT_MDL))
    assert sim.t < 200.
  for turned, p, a in rows:
    assert not p.shouldStop and plan_accel(p.accelCap, p.speedCap, sim.v) >= -2.0
    if turned < 45.:
      assert p.accelCap == 1.0 and a <= 1.0 + 1e-6, turned
    else:
      assert p.accelCap == NO_ACCEL_CAP, turned
    if turned < 15.:
      assert p.speedCap <= 3.0 + 1e-6, turned
    elif turned < 45.:
      assert p.speedCap <= 3.0 + 1.3 * (turned - 15.) / 30. + 0.03, turned
  assert any(math.isclose(t, 30., abs_tol=1.) and math.isclose(p.speedCap, 3.65, abs_tol=0.05) for t, p, _ in rows)
  assert max(t for t, _, _ in rows) >= 60.  # it ended at its blend's end, not before
  p = sim.step(engaged=True)
  assert p.accelCap == NO_ACCEL_CAP and p.speedCap > 4.3


def test_post_go_profiles_by_param_for_the_sweep():
  """TurnPilotPostGoProfile picks the standstill turn's profile (turnpilotd main()), so the BeamNG sweep's four columns need
  no code edit: none (V1 tapless) leaves the turn's own corner cap alone; flat3, graduated and flat43 cap it from the go."""
  caps = {}
  for name in ('none', 'flat3', 'graduated', 'flat43'):
    sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
    sim.tp.post_go_profile = name
    go = go_from_the_hold(sim)
    while sim.tp.post_go is not None and sim.tp.post_go.turned_deg < 30.:
      p = sim.step(engaged=True, yaw_rate=0.3 if sim.v > 0.5 else 0.)
      assert sim.t < 200.
    caps[name] = (go.accelCap, round(go.speedCap, 2), round(p.speedCap, 2) if name != 'none' else None)
  assert caps['none'][0] == NO_ACCEL_CAP and caps['none'][1] > 4.0  # the corner cap only (V_CORNER)
  assert caps['flat3'] == (1.0, 3.0, 3.0)
  assert caps['graduated'][:2] == (1.0, 3.0) and math.isclose(caps['graduated'][2], 3.65, abs_tol=0.05)
  assert caps['flat43'][:2] == (1.0, 4.3) and caps['flat43'][2] <= 4.3


def test_no_post_go_profile_without_a_standstill_go():
  """A turn driven through without stopping (signals pass through: no light source, no stop) keeps its own caps only."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  plans = []
  while sim.s < 340.:
    plans.append(sim.step(engaged=True, yaw_rate=0.4 if 300. - sim.s < 10. and sim.tp.desires.turned_deg < 85. else 0.))
    assert sim.tp.post_go is None
  assert all(p.accelCap == NO_ACCEL_CAP and not p.shouldStop for p in plans)


def test_a_go_missed_straight_on_drops_its_post_go_cap_at_the_pass():
  """A turn from a standstill driven straight past its junction is missed (turnMissed, rerouted): its 3.0 m/s post-go cap
  ends there and doesn't hold the car on the road after it."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go_from_the_hold(sim)
  plans = []
  while not (plans and plans[-1].turnMissed):
    plans.append(sim.step(engaged=True))
    assert sim.t < 200.
  assert all(p.speedCap <= 3.0 + 1e-6 for p in plans[:-1])
  assert sim.tp.post_go is None and plans[-1].speedCap > 3.0 and plans[-1].accelCap == NO_ACCEL_CAP


def test_a_hold_reached_already_into_the_turn_starts_the_ramp_where_it_is():
  """G0: a 10 m hold was reached 39-49 deg into the turn. The profile is keyed on DesireTiming.turned_deg itself, so a go
  40 deg in starts at 4.08 m/s (the ramp at 40 deg), with the accel cap, not back at 3.0."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  sim.go_taps = False
  sim.tp.oracle_ok = True
  light = {'state': 'red'}

  def lights(sim):
    msg = messaging.new_message('turnpilotLights')
    msg.turnpilotLights.source = 'simOracle'
    msg.turnpilotLights.lights = [{'state': light['state'], 'distance': 290. - sim.s, 'instance': 'trafficLight 1'}]
    return msg.as_reader().turnpilotLights
  sim.lights = lights
  stop_at(sim, 290.)
  sim.run(1.)
  sim.tp.desires.turned_deg = 40.  # crept into the turn on the way to the line
  light['state'] = 'green'
  while sim.trace[-1][2].shouldStop:
    sim.step(engaged=True)
    assert sim.t < 120.
  go = sim.trace[-1][2]
  assert sim.tp.post_go is not None and go.accelCap == 1.0
  assert math.isclose(go.speedCap, 3.0 + 1.3 * 25. / 30., abs_tol=1e-3)


def test_the_unprotected_left_takes_its_own_post_go_profile():
  """A go into an unprotected left (G4's autonomous go; today it hands back at its line) takes the left's profile, 1.5 m/s^2
  and 4.3 m/s to 45 deg (DESIGN section 2), or TurnPilotLeftGoProfile's. Any other turn takes TurnPilotPostGoProfile's."""
  left = Maneuver(*classify("turn", "left"), "turn", "left", 90., geometry=True, opposing='present')
  t_left = Maneuver(*classify("turn", "left"), "turn", "left", 90., geometry=True, opposing='none')
  right = Maneuver(*classify("turn", "right"), "turn", "right", 90., geometry=True, opposing='present')
  tp = TurnPilot()
  tp.start_post_go(left, 9., 0., True)
  assert tp.post_go.profile == POST_GO_PROFILES['left']
  assert tp.post_go.caps(0.) == (1.5, 4.3)
  for m in (t_left, right):
    tp.start_post_go(m, 9., 0., True)
    assert tp.post_go.profile == POST_GO_PROFILES['graduated']
  tp.start_post_go(left, 9., 0., False)  # traffic keeping left: the right is the unprotected one
  assert tp.post_go.profile == POST_GO_PROFILES['graduated']
  tp.start_post_go(right, 9., 0., False)
  assert tp.post_go.profile == POST_GO_PROFILES['left']
  tp.left_go_profile, tp.post_go_profile = 'graduated', 'flat43'
  tp.start_post_go(left, 9., 0., True)
  assert tp.post_go.profile == POST_GO_PROFILES['graduated']
  tp.start_post_go(t_left, 9., 0., True)
  assert tp.post_go.profile == POST_GO_PROFILES['flat43']


def test_no_move_after_the_go_withdraws_it_and_hands_back():
  """DESIGN section 2/3: the car not at 0.3 m/s NO_MOVE_S (2.0 s) after the go is the model holding it; TurnPilot withdraws
  the go before the car moves (holds it standing: shouldStop, a 0 speed cap braking no harder than -2.0, no turn pulse)
  and hands back loudly (cantStartTakeControl, "Can't start the turn: take control"), until the driver takes over."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go_from_the_hold(sim)
  plans = stand(sim, NO_MOVE_S - 0.2)
  assert all(p.controlPrompt == 'none' and not p.shouldStop for p in plans)
  p = stand(sim, 0.4)[-1]
  assert p.controlPrompt == 'cantStartTakeControl' and p.reason == NO_MOVE_REASON == "Can't start the turn: take control"
  assert p.shouldStop and p.speedCap == 0. and plan_accel(p.accelCap, p.speedCap, 0.3) >= -2.0
  assert p.desire == Desire.none and p.state == State.holding
  # held so while engaged, the plan applied: the car stays put, and no other handback replaces it
  plans = [sim.step(engaged=True) for _ in range(int(10. / DT_MDL))]
  assert all(p.controlPrompt == 'cantStartTakeControl' and p.shouldStop for p in plans) and sim.v < 0.05
  # the driver takes over (any gas press disengages): released, and nothing comes back
  p = sim.step(engaged=False, gas=True)
  assert p.controlPrompt == 'none' and not p.shouldStop
  assert all(p.controlPrompt == 'none' and not p.shouldStop for p in [sim.step(engaged=True) for _ in range(int(3. / DT_MDL))])


def test_no_move_handback_not_for_a_car_that_pulls_away_late_or_waits_behind_a_lead():
  # at 0.3 m/s 1.5 s after the go (G0: 50 left at 10 m took 1.95-3.11 s; those hand back): nothing said
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go_from_the_hold(sim)
  stand(sim, 1.5)
  plans = [sim.step(engaged=True) for _ in range(int(4. / DT_MDL))]
  assert sim.v > 0.3 and all(p.controlPrompt == 'none' and not p.shouldStop for p in plans)
  # stopped again once moving, under 20 deg in (the stall's range): not this handback
  assert all(p.controlPrompt != 'cantStartTakeControl' for p in stand(sim, 4.))
  # a lead close ahead holds the car: the go's clock waits for it
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go_from_the_hold(sim)
  assert all(p.controlPrompt == 'none' for p in stand(sim, NO_MOVE_S + 2., lead_prob=0.9, lead_x=STOPPED_LEAD_M - 5.))
  assert stand(sim, NO_MOVE_S + 0.2)[-1].controlPrompt == 'cantStartTakeControl'


def test_an_unprotected_turns_go_is_withdrawn_after_a_shorter_no_move():
  """CRITIQUE REQUIRED 4: a go across the oncoming road with no movement is withdrawn at LEFT_NO_MOVE_S (1.0 s), not 2.0:
  the gap it was certified for runs out while the car stands. Any other turn waits NO_MOVE_S (2.0 s)."""
  left = Maneuver(*classify("turn", "left"), "turn", "left", 90., geometry=True, opposing='present')
  t_left = Maneuver(*classify("turn", "left"), "turn", "left", 90., geometry=True, opposing='none')
  tp = TurnPilot()
  tp.start_post_go(left, 9., 0., True)
  assert tp.no_move_limit == LEFT_NO_MOVE_S == 1.0
  tp.start_post_go(t_left, 9., 0., True)
  assert tp.no_move_limit == NO_MOVE_S == 2.0
  tp.left_go_profile = 'graduated'  # the limit goes with the turn, not the profile
  tp.start_post_go(left, 9., 0., True)
  assert tp.no_move_limit == LEFT_NO_MOVE_S


def test_a_withdrawn_go_is_not_given_again_by_a_hold_that_comes_and_goes():
  """Withdrawn and handed back, the car held: a walker crossing in front (a scene hold) and clearing is no new go. The
  driver was told to take control, and only the driver taking over ends it."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go_from_the_hold(sim)
  assert stand(sim, NO_MOVE_S + 0.2)[-1].controlPrompt == 'cantStartTakeControl'
  walker = ({'cls': 'pedestrian', 'x': 4., 'y': 0.1, 'vx': 0., 'vy': 0., 'inPath': True},)
  held = stand(sim, 2., objects=walker)
  assert all(p.shouldStop for p in held)
  plans = [sim.step(engaged=True) for _ in range(int(5. / DT_MDL))]
  assert all(p.controlPrompt == 'cantStartTakeControl' and p.shouldStop and p.desire == Desire.none for p in plans)
  assert sim.v < 0.05


WALKER = ({'cls': 'pedestrian', 'x': 4., 'y': 0.1, 'vx': 0., 'vy': 0., 'inPath': True},)


def walker_stops_the_car(sim, n_max=400):
  """A walker on the turn's exit crosswalk, in the path, stops the car where it is (scene_policy: a pedestrian in the path
  still stops a committed turn), and stands in front of it 2 s. Then they clear: the hold lets go at standstill, a go."""
  n = 0
  while sim.v > 0.05:
    sim.step(WALKER, engaged=True)
    n += 1
    assert n < n_max
  assert all(p.shouldStop for p in stand(sim, 2., objects=WALKER))


def finish_the_turn(sim, seconds=8.):
  """The model goes at last: the car drives the turn round, yawing while it moves."""
  plans, turned = [], sim.tp.desires.turned_deg
  for _ in range(int(seconds / DT_MDL)):
    plans.append(sim.step(engaged=True, yaw_rate=0.3 if sim.v > 0.5 and turned < 90. else 0.))
    turned = max(turned, sim.tp.desires.turned_deg)
  return plans, turned


def test_a_go_in_the_junction_is_never_withdrawn_and_the_turn_is_finished():
  """Skeptic G3: the no-move withdrawal froze the car in the junction. A walker stops the car 45 deg into the turn, past the
  hold line and committed to it, and clears: TurnPilot's go. The model stands 3.2 s (G0: its shouldStop held about 3 s after
  a release). That go is never withdrawn: TurnPilot never holds the car in the junction (stopping in it is worse than
  finishing it), the stall's handback without a hold says so after STALL_PROMPT_S, and the car finishes the turn."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go_from_the_hold(sim)
  while sim.tp.desires.turned_deg < 45.:
    sim.step(engaged=True, yaw_rate=0.3 if sim.v > 0.5 else 0.)
    assert sim.t < 200.
  walker_stops_the_car(sim)
  assert sim.distance() < HOLD_LINE_M - 2. and sim.tp.scene.committed_id == sim.tp.follower.maneuver_id
  plans = stand(sim, 0.05)  # the walker gone: TurnPilot's go, its post-go caps (only lowering speed) from where it is
  assert sim.tp.release_go and sim.tp.go_id == sim.tp.follower.maneuver_id and sim.tp.post_go.turned_deg >= 45.
  assert sim.tp.go_wait_s is None and not sim.tp.go_withdrawn
  plans += stand(sim, 3.15)  # the model hesitates
  assert not any(p.shouldStop or p.controlPrompt == 'cantStartTakeControl' or p.speedCap < 1. for p in plans)
  stalled = [p for p in plans if p.controlPrompt == 'stalledTakeControl']
  assert stalled and len(stalled) * DT_MDL < 0.3 and stalled[-1].reason == STALLED_REASON  # handed back, nothing held
  after, turned = finish_the_turn(sim)
  assert not any(p.shouldStop or p.controlPrompt == 'cantStartTakeControl' for p in after)
  assert sim.v > 3. and turned >= 80.


def test_a_go_in_an_unprotected_left_across_the_oncoming_lanes_is_never_withdrawn():
  """Skeptic G3's worst case: a left across an oncoming road. TurnPilot hands back at its hold line (G1), the driver drives
  through the line and engages again, a walker stops the car 40 deg in, in the oncoming lanes, and clears. The go takes the
  left's profile and its 1.0 s no-move limit, and the model stands 1.5 s: the car is never held there, nothing TurnPilot
  does keeps it in the oncoming lanes, and it finishes the turn once the model goes."""
  sim = Sim([(300., "turn", "left"), (1500., "turn", "right")])
  while sim.s < 295.:
    sim.step(engaged=sim._take_over_from is None)
    assert sim.t < 200., (sim.s, sim.v)
  while sim.tp.desires.turned_deg < 40.:
    sim.step(engaged=sim._take_over_from is None, yaw_rate=0.3 if sim.v > 0.5 else 0.)
    assert sim.t < 300.
  sim.go_taps = False
  walker_stops_the_car(sim)
  assert sim.distance() < HOLD_LINE_M
  plans = stand(sim, LEFT_NO_MOVE_S + 0.5)
  assert sim.tp.no_move_limit == LEFT_NO_MOVE_S and sim.tp.post_go.profile == POST_GO_PROFILES['left']
  assert not any(p.shouldStop or p.controlPrompt == 'cantStartTakeControl' for p in plans) and not sim.tp.go_withdrawn
  plans = stand(sim, STALL_PROMPT_S - LEFT_NO_MOVE_S)  # handed back without a hold as a stall in the turn
  assert plans[-1].controlPrompt == 'stalledTakeControl' and not any(p.shouldStop for p in plans)
  after, turned = finish_the_turn(sim)
  assert not any(p.shouldStop for p in after) and sim.v > 3. and turned >= 80.


def test_a_go_past_the_hold_line_under_20_deg_in_is_handed_back_as_a_stall_never_held():
  """A walker stops the car past the hold line before it has turned 20 deg (a left still crossing the near lanes) and clears.
  The go isn't withdrawn there (no hold in the junction), and the model standing is still never silent: the go was in the
  junction, so the stall's handback (stalledTakeControl, no hold) covers it from 0 deg, not only from STALL_MIN_DEG."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go_from_the_hold(sim)
  while sim.distance() > HOLD_LINE_M - 2.5:
    sim.step(engaged=True, yaw_rate=0.05 if sim.v > 0.5 else 0.)
    assert sim.t < 200.
  walker_stops_the_car(sim)
  assert sim.distance() < HOLD_LINE_M - 2. and sim.tp.desires.turned_deg < 15.
  plans = stand(sim, STALL_PROMPT_S - 0.2)
  assert not any(p.shouldStop or p.controlPrompt != 'none' for p in plans)
  p = stand(sim, 0.4)[-1]
  assert p.controlPrompt == 'stalledTakeControl' and not p.shouldStop and p.desire == Desire.none
  # a go at the hold line under 20 deg in is withdrawn as before, not a stall
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  go_from_the_hold(sim)
  assert sim.distance() >= HOLD_LINE_M
  assert stand(sim, NO_MOVE_S + 0.2)[-1].controlPrompt == 'cantStartTakeControl'


def test_a_go_at_the_hold_line_already_into_the_turn_is_still_withdrawn():
  """G0: a 10 m hold was reached 39-49 deg into the turn, the desire pulsed on the approach. The car stands at the hold line,
  not in the junction, so its go is still withdrawn after NO_MOVE_S (by the car's place, not its heading)."""
  sim = Sim(LEFT_AT_A_RED, controls=[(290., 'signal')])
  sim.go_taps = False
  sim.tp.oracle_ok = True
  light = {'state': 'red'}

  def lights(sim):
    msg = messaging.new_message('turnpilotLights')
    msg.turnpilotLights.source = 'simOracle'
    msg.turnpilotLights.lights = [{'state': light['state'], 'distance': 290. - sim.s, 'instance': 'trafficLight 1'}]
    return msg.as_reader().turnpilotLights
  sim.lights = lights
  stop_at(sim, 290.)
  sim.run(1.)
  sim.tp.desires.turned_deg = 40.
  light['state'] = 'green'
  while sim.trace[-1][2].shouldStop:
    sim.step(engaged=True)
    assert sim.t < 120.
  assert sim.distance() >= HOLD_LINE_M and sim.tp.go_wait_s is not None
  p = stand(sim, NO_MOVE_S + 0.2)[-1]
  assert p.controlPrompt == 'cantStartTakeControl' and p.shouldStop and p.speedCap == 0.


def test_a_go_into_a_junction_the_scene_committed_to_or_the_driver_released_is_never_withdrawn():
  """The scene never holds the car again in a junction it committed to, or one the driver took over into (committed_id,
  released_id): nor does a no-move withdrawal, wherever the follower places the car."""
  t_left = Maneuver(*classify("turn", "left"), "turn", "left", 90., geometry=True, opposing='none')
  for field in ('committed_id', 'released_id', None):
    tp = TurnPilot()
    if field is not None:
      setattr(tp.scene, field, tp.follower.maneuver_id)
    tp.start_post_go(t_left, 12., 0., True)
    assert (tp.go_wait_s is None) == (field is not None), field
    tp.scene.committed_id = tp.scene.released_id = tp.follower.maneuver_id + 1  # another maneuver's: no matter
    tp.start_post_go(t_left, 12., 0., True)
    assert tp.go_wait_s == 0.
    tp.start_post_go(t_left, HOLD_LINE_M - 2., 0., True)  # past the hold line
    assert tp.go_wait_s is None



def test_the_no_move_limit_is_a_param_for_the_sweep():
  """TurnPilotNoMoveS (main()) sets a standstill turn's no-move limit, NO_MOVE_S to NO_MOVE_MAX_S, so the sweep's V1 tapless
  column can measure a pull-away later than 2 s with no code edit. The unprotected turn's LEFT_NO_MOVE_S stays."""
  left = Maneuver(*classify("turn", "left"), "turn", "left", 90., geometry=True, opposing='present')
  t_left = Maneuver(*classify("turn", "left"), "turn", "left", 90., geometry=True, opposing='none')
  tp = TurnPilot()
  tp.no_move_s = 20.
  tp.start_post_go(t_left, 9., 0., True)
  assert tp.no_move_limit == 20.
  tp.start_post_go(left, 9., 0., True)
  assert tp.no_move_limit == LEFT_NO_MOVE_S
