"""Fundamentals: the basic city turns driven closed loop, as a passenger judges them. turnpilotd on test_turnpilotd's Sim, with
modeld's DesireHelper in the loop and a car that moves over a lane on a lane change desire and turns on a turn desire.

Why: on the first real drive (2026-11-01) the car made no turn on its own. It reached the light in the straight lane, the
turn was given up, and the driver took over. The tests before these are mostly unit tests, and the Sim has no DesireHelper:
a requested lane change never happens there, so nothing drove the whole loop from the plan's request to the car in the turn
lane, signalling, stopped at the light and turning on its green. Each test here asserts what a passenger sees (the lane the
car turns from, its lamp, a stop at the line, the turn taken, no skip or stall), never the planner's internal state.

The loop, frame by frame in modeld's order: the car's carState (its speed, the lamps, the blind spot monitor) and the
modelV2 of the road where the car is (its lane lines offset by where it sits in its lane, the lane counts and paint of the
lane it is in, DesireHelper's state from its last update) go to turnpilotd; its plan's laneChange and desire go to
DesireHelper and the model's desire input as modeld puts them together (DesireHelper's desire first, the plan's turn desire
while it is idle, plan_desire's latActive gate and CLEAR_DESIRE); turnpilotd sees DesireHelper's state a frame later, as on
the car. The model's lane change: a rising edge of laneChangeLeft/Right moves the car one lane over in LC_MOVE_S, its lane
lines relabelled as its centre crosses the line, its lane change probability high until it is there (DesireHelper ends the
change on it). The model's turn: a turn desire's rising edge kept DESIRE_QUEUE frames (the model's desire history; a
CLEAR_DESIRE takes it out) turns the car round the junction it reaches TURN_ENTRY_M before, at TURN_RADIUS_M, to the
junction's 90 degrees. Longitudinal: the Sim's point mass under the plan's caps.

Who signals is the Tesla's two ways (car_signals.py), each test run under both: 'car', its own lamp (a verified port, the
lamp lit LAMP_ON_S after turnpilotd asks for it, a lane change waiting for 300 ft of it, navd sending no limit), and
'driver', the driver answering every prompt on the stalk REACT_S later and holding it through the change and, armed, the
turn. The switches are the shipped ones (params_keys.h): TurnPilotAlwaysDrive on, the autonomous left on (with a measured
range: the default 0 skips every left). Lights come from the sim's oracle, red on arrival and green after a wait."""
import math
from typing import NamedTuple

import pytest

from opendbc.car.tesla.values import TeslaFlags
from openpilot.cereal import custom, log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS, plan_desire
from openpilot.selfdrive.turnpilot.lane_planner import lane_change_window
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint
from openpilot.selfdrive.turnpilot.tests.synthetic import LANE_WIDTH, junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import THREE_LANES_LEFT_TURN, Sim, oracle_lights
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_left import STRAIGHT_M, TEST_RANGE_M, V_25, road_runs
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_stop import S_LINE, StopSim
from openpilot.selfdrive.turnpilot.handback import Handback

LaneChange = custom.TurnpilotPlan.LaneChange
LaneChangeState = log.LaneChangeState
Desire = log.Desire
MPH = 0.44704
W = LANE_WIDTH

LC_MOVE_S = 4.          # s a lane change desire takes the car one lane over
DESIRE_QUEUE = 132      # frames the model keeps a desire's rising edge (6.6 s at 20 Hz: desire_timing.py)
TURN_ENTRY_M = {'right': 5., 'left': 1.}  # the model starts round the corner this far before the junction point...
TURN_RADIUS_M = {'right': 9., 'left': 14.}  # ...at this radius, to the junction's angle
TURN_DEG = 90.
LAMP_ON_S = 0.5         # s from turnpilotd's ask to the Tesla's lamp lit (the port's next frame and the lamp's on latency)
REACT_S = 0.8           # s from a prompt to the driver's stalk
WAIT_S = 6.             # s the light stays red once the car stands at it

SW, BW = LinePaint('solid', 'white', 0.9, 0.95), LinePaint('broken', 'white', 0.9, 0.3)
YELLOW = LinePaint('solid', 'yellow', 0.9, 0.95)
TWO_LANES_RIGHT_TURN = ((("straight",), False), (("right",), True))
TWO_LANES_LEFT_TURN = ((("left",), True), (("straight",), False))
THREE_LANES_RIGHT_TURN = ((("straight",), False), (("straight",), False), (("right",), True))
SIDE = {'left': LaneChange.left, 'right': LaneChange.right}
YAW_SIGN = {'left': 1., 'right': -1.}  # carState.yawRate is left positive


class Loop(Sim):
  """test_turnpilotd's Sim with DesireHelper, a car that changes lanes and turns, and who signals (module docstring).
  lanes: the road's lanes our way, the leftmost bordered by the yellow centre line, the curb lane by a solid white edge
  line, broken white between; lane: the one the car starts in, 0 the leftmost."""
  def __init__(self, maneuvers, lanes=2, lane=0, signals='car', **kw):
    super().__init__(maneuvers=maneuvers, driver_signals=None, **kw)
    self.go_taps = False
    self.model = None  # the modelV2 from model_lanes and model_kw (step()), StopSim's own too
    self.tp.always_drive = True  # TurnPilotAlwaysDrive's shipped default (params_keys.h)
    self.n_lanes, self.y = lanes, (lane + 0.5) * W  # y: m right of the road's left line, the car's centre
    self.signals = signals
    if signals == 'car':
      self.tp.car_signal_flags, self.tp.car_signal_mode = int(TeslaFlags.CAR_SIGNALS), 'on'
      self.car_signal = Loop.port
    self.dh = DesireHelper()
    self.lc_prob = 0.
    self.move = None        # the lane change under way: [s into it, y it started at, +1 right / -1 left]
    self.fed = Desire.none  # the desire modeld fed the model last frame
    self.queue = None       # [frames since the turn desire's rising edge, its side], while the model has it
    self.turn = None        # the turn under way: [side, junction s]
    self.turned_at = set()  # junctions turned at
    self.heading = 0.       # deg turned, left positive
    self.lamp_ask, self.lamp_t = LaneChange.none, 0.
    self.stalk, self.ask, self.ask_t = 'none', 'none', 0.
    self.frames = []        # Frame, per frame

  # -------------------------------------------------------------------- the road and the car on it

  def lane(self) -> int:
    return min(max(int(self.y // W), 0), self.n_lanes - 1)

  def line_paint(self, k: int) -> LinePaint:
    if k < 0 or k > self.n_lanes:
      return UNKNOWN
    return YELLOW if k == 0 else SW if k == self.n_lanes else BW

  def road_paint(self) -> tuple:
    """The lane paint the detector reads for modelV2's four lines [far left, left, right, far right] from the car's lane."""
    i = self.lane()
    return tuple(self.line_paint(k) for k in (i - 1, i, i + 1, i + 2))

  def lamp(self) -> int:
    """The side lit, as a passenger sees it: the car's own lamp or the driver's."""
    if self.signals == 'car':
      return self.lamp_ask if self.lamp_ask != LaneChange.none and self.lamp_t >= LAMP_ON_S else LaneChange.none
    return SIDE.get(self.stalk, LaneChange.none)

  @staticmethod
  def port(sim) -> dict:
    """carState.carSignal of a verified Tesla port: the side turnpilotd asked for last frame, lit LAMP_ON_S on."""
    ask = sim.tp.signal_out.side
    if ask != sim.lamp_ask:
      sim.lamp_ask, sim.lamp_t = ask, 0.
    else:
      sim.lamp_t += DT_MDL
    lit = sim.lamp()
    return {'status': 'lit' if lit != LaneChange.none else 'requesting' if ask != LaneChange.none else 'ready',
            'left': ask == LaneChange.left, 'right': ask == LaneChange.right,
            'leftLamp': lit == LaneChange.left, 'rightLamp': lit == LaneChange.right}

  def driver(self) -> str:
    """The driver's stalk ('none', 'left', 'right'): a prompt answered REACT_S on, held while prompted, while a change is
    offered or under way that way, and while the turn armed that way is (car_signals.py's intent)."""
    if self.signals != 'driver' or not self.trace:
      return 'none'
    plan = self.trace[-1][2]
    prompt = next((str(p) for p in (plan.signalPrompt, plan.turnSignal.prompt) if p != LaneChange.none), 'none')
    if prompt != self.ask:
      self.ask, self.ask_t = prompt, 0.
    else:
      self.ask_t += DT_MDL
    if prompt != 'none' and self.ask_t >= REACT_S - 1e-6:
      self.stalk = prompt
    if self.stalk != 'none' and prompt == 'none':
      side = SIDE[self.stalk]
      changing = self.dh.lane_change_state == LaneChangeState.laneChangeStarting
      armed = self.tp.turn_signal.armed_id is not None and self.tp.turn_signal.arm_side == side
      if plan.laneChangeOffer != side and not changing and not armed:
        self.stalk = 'none'
    return self.stalk

  def junction_ahead(self, side: str):
    """The s of the route's turn that way the car is about to reach, or None."""
    for m in self.navd.maneuvers:
      if m[1] == 'turn' and m[2] == side and m[0] not in self.turned_at and -15. < m[0] - self.s < 60.:
        return m[0]
    return None

  def yaw_rate(self) -> float:
    """This frame's yaw: the model turning round the junction with a turn desire in its queue (module docstring)."""
    if self.turn is None and self.queue is not None and self.v > 0.3:
      side = self.queue[1]
      j = self.junction_ahead(side)
      if j is not None and self.s >= j - TURN_ENTRY_M[side]:
        self.turn = [side, j]
    if self.turn is None:
      return 0.
    side, j = self.turn
    if abs(self.heading) >= TURN_DEG:
      self.turned_at.add(j)
      self.turn = None
      return 0.
    return YAW_SIGN[side] * self.v / TURN_RADIUS_M[side]

  # -------------------------------------------------------------------- one frame

  def step(self, objects=(), **kw):
    i = self.lane()
    offset = self.y - (i + 0.5) * W
    self.model_lanes = (i, self.n_lanes - 1 - i)
    curvature = 0. if self.turn is None else -YAW_SIGN[self.turn[0]] / TURN_RADIUS_M[self.turn[0]]  # modelV2: positive right
    self.model_kw = {'offset': offset, 'lane_change_state': self.dh.lane_change_state,
                     'lane_change_direction': self.dh.lane_change_direction, 'curvature': curvature}
    self.paint = self.road_paint()
    self._signal = self.driver()
    yaw = self.yaw_rate()
    plan = super().step(objects, engaged=True, yaw_rate=yaw, **kw)  # the car's port (port()) ran in it: its lamp this frame
    self.heading += math.degrees(yaw) * DT_MDL
    self.frames.append(Frame(self.t, self.s, self.v, i, self.dh.lane_change_state, self.lamp(), self.heading, plan))
    # modeld: DesireHelper on this plan, then the model's desire input
    self.dh.update(self.sm['carState'], self.sm['carControl'].latActive, self.lc_prob, plan.laneChange.raw)
    tp_desire, clear = plan_desire(plan.desire, self.sm['carControl'].latActive)
    fed = self.dh.desire if self.dh.desire != Desire.none else tp_desire
    self.drive_model(fed, clear)
    return plan

  def drive_model(self, fed, clear: bool) -> None:
    """The driving model on modeld's desire: the turn desire queue and the lane change move."""
    if clear:
      self.queue = None
    if fed in (Desire.turnLeft, Desire.turnRight) and fed != self.fed:
      self.queue = [0, 'left' if fed == Desire.turnLeft else 'right']
    elif self.queue is not None:
      self.queue[0] += 1
      if self.queue[0] >= DESIRE_QUEUE:
        self.queue = None
    if self.move is None and fed in (Desire.laneChangeLeft, Desire.laneChangeRight) and fed != self.fed:
      self.move = [0., (self.lane() + 0.5) * W, -1. if fed == Desire.laneChangeLeft else 1.]
    self.fed = fed
    if self.move is not None:
      self.move[0] += DT_MDL
      f = min(self.move[0] / LC_MOVE_S, 1.)
      self.y = self.move[1] + self.move[2] * W * (1. - math.cos(math.pi * f)) / 2.
      if f >= 1.:
        self.move = None
    self.lc_prob = 1. if self.move is not None else 0.

  # -------------------------------------------------------------------- driving

  def drive(self, until, limit_s: float, objects=lambda sim: ()):
    """Steps until until(sim), failing with the trace past limit_s."""
    t0 = self.t
    while not until(self):
      self.step(objects(self))
      assert self.t - t0 < limit_s, trace(self)


class StopLoop(Loop, StopSim):
  """Loop at test_turnpilotd_stop's stop sign under the decider (its surveyed all-way, the visibility file, the clock):
  StopSim's model holds the car standing and drives once it rolls, its planner a lag late. One lane, so no change."""
  def __init__(self, tmp_path, maneuvers, signals='car', **kw):
    super().__init__(tmp_path=tmp_path, maneuvers=maneuvers, lanes=1, lane=0, signals=signals, **kw)

  def step(self, objects=(), **kw):
    self.model_holds = self.v < 0.3  # StopSim's 'goes' model: it holds a standstill and drives once rolling
    return super().step(objects, **kw)


class Frame(NamedTuple):
  """One frame as a passenger sees it (after its step), and its plan."""
  t: float
  s: float
  v: float
  lane: int       # the lane the car was in, 0 the leftmost
  dh: int         # DesireHelper's state as turnpilotd read it this frame
  lamp: int       # the side lit (Loop.lamp)
  heading: float  # deg turned, left positive
  plan: object


def trace(sim, every: int = 10) -> str:
  """The drive, a line every few frames, for a failing assertion."""
  rows = []
  for f in sim.frames[::every]:
    p = f.plan
    rows.append(" ".join((
      f"{f.t:5.1f} s={f.s:6.1f} v={f.v:4.1f} lane={f.lane} dh={int(f.dh)} lamp={str(f.lamp):5} hdg={f.heading:5.1f}",
      f"d={p.maneuverDistance:6.1f} wait={p.laneChangeWait} offer={p.laneChangeOffer} lc={p.laneChange}",
      f"cd={p.laneChangeCountdown:.1f} abort={p.laneChangeAbort} hold={p.laneChangeHold} des={p.desire}",
      f"stop={int(p.shouldStop)} skip={int(p.tooFastToTurn)}{p.skipReason} sig={p.turnSignal.side}/{p.turnSignal.prompt}",
      f"lit={p.turnSignal.litM:.0f} mm={p.laneMismatch} prompt={p.controlPrompt} {p.reason}")))
  return "\n" + "\n".join(rows[-120:])


# ------------------------------------------------------------------------------------------- what a passenger judges

def lane_changes(sim) -> list:
  """[(frame index the change started, side, the lane it left, the lane it reached)] for each lane change DesireHelper started."""
  out = []
  for k in range(1, len(sim.frames)):
    a, b = sim.frames[k - 1], sim.frames[k]
    if b.dh == LaneChangeState.laneChangeStarting and a.dh != LaneChangeState.laneChangeStarting:
      out.append([k, None, b.lane, None])
  for c in out:
    after = next((f.lane for f in sim.frames[c[0]:] if f.lane != c[2]), c[2])
    c[1], c[3] = ('left' if after < c[2] else 'right'), after
  return out


def assert_signalled_changes(sim, side: str, n: int, before_m: float):
  """n lane changes toward side, each with its lamp lit that way when it started, the car settled in the lane it changed to
  more than before_m short of the junction. The tests pass the window's end at the approach speed (60 m and more): the car
  in its turn lane before it brakes for the light at 35 mph, and well before at 12, never at the light. On the car's own
  lamp it stays lit from the first change's start through the last (one signal across the lanes)."""
  changes = lane_changes(sim)
  assert len(changes) == n and all(c[1] == side and abs(c[3] - c[2]) == 1 for c in changes), (changes, trace(sim))
  for k, _, _, _ in changes:
    assert sim.frames[k].lamp == SIDE[side], ("changed lanes unsignalled", k, trace(sim))
  last = changes[-1][0]
  settled = next(k for k in range(last, len(sim.frames)) if sim.frames[k].dh != LaneChangeState.laneChangeStarting)
  assert sim.frames[settled].plan.maneuverDistance > before_m, (sim.frames[settled].plan.maneuverDistance, before_m, trace(sim))
  if sim.signals == 'car':
    assert all(f.lamp == SIDE[side] for f in sim.frames[changes[0][0]:settled]), ("the lamp went dark between changes", trace(sim))
  return changes


def assert_turned(sim, side: str, lane: int, junction: float, past_m: float = 20.):
  """The car turned at junction toward side from lane, its lamp lit that way from the turn's start, and drove on past it:
  never skipped, missed, handed back, prompted to take control, told its lane is wrong or can't be confirmed (laneMismatch:
  'Wrong Lane', 'Can't Confirm Lane'), nor turned twice."""
  turning = [f for f in sim.frames if abs(f.heading) > 1.]
  assert turning, ("never turned", trace(sim))
  start = turning[0]
  assert start.lane == lane, ("turned from the wrong lane", start.lane, lane, trace(sim))
  assert abs(start.s - junction) < 15., (start.s, junction, trace(sim))
  assert YAW_SIGN[side] * sim.frames[-1].heading >= TURN_DEG - 1., (sim.frames[-1].heading, trace(sim))
  through = [f for f in sim.frames if f.s >= start.s and abs(f.heading) < 45.]
  assert all(f.lamp == SIDE[side] for f in through), ("turned with its lamp off", trace(sim))
  assert sim.s > junction + past_m, trace(sim)
  plans = [f.plan for f in sim.frames]
  assert not any(p.tooFastToTurn or p.skipReason or p.turnMissed for p in plans), ("skipped", trace(sim))
  assert not any(str(p.controlPrompt) in TAKE_CONTROL_PROMPTS or p.handback != Handback.none for p in plans), trace(sim)
  assert not any(str(p.laneMismatch) != 'none' for p in plans), ("told its lane is wrong", trace(sim))


def stood_at(sim, line: float) -> int:
  """The frame the car came to rest at the light: short of its line and within 15 m of it."""
  k = next((k for k, f in enumerate(sim.frames) if f.v < 0.05 and line - 15. < f.s), None)
  assert k is not None, ("never stood at the light", trace(sim))
  assert sim.frames[k].s <= line + 0.5, ("stood past the line", trace(sim))
  return k


# ------------------------------------------------------------------------------------------- the drives

def right_at_a_signal(mph: float, signals: str, lanes: int = 2, lane: int = 0, nav=TWO_LANES_RIGHT_TURN, junction: float = 450.,
                      green: bool = False) -> tuple:
  """A right at junction from the curb lane of lanes, a signal 10 m before it, the car starting in lane at mph: red on arrival,
  green WAIT_S after it stands (green: green throughout). (sim, the light's line)."""
  line = junction - 10.
  sim = Loop([(junction, "turn", "right", junction_of("right")), (junction + 1600., "turn", "left", junction_of("left"))],
             lanes=lanes, lane=lane, signals=signals, cruise=mph * MPH, nav_lanes={0: nav}, controls=[(line, 'signal')])
  red_light(sim, line, green)
  return sim, line


def left_at_a_signal(mph: float, signals: str, lanes: int = 2, lane: int = 1, nav=TWO_LANES_LEFT_TURN, junction: float = 450.,
                     green: bool = False) -> tuple:
  """A left at junction across a 25 mph road with an oncoming lane (the unprotected left, nobody coming: the road camera's
  runs empty), from the lane by the centre line, a signal at its hold line; as right_at_a_signal otherwise."""
  line = junction - 8.5
  left = (0., 270., 2, 'present', V_25, 1, STRAIGHT_M)
  sim = Loop([(junction, "turn", "left", left), (junction + 1600., "turn", "right", junction_of("right"))],
             lanes=lanes, lane=lane, signals=signals, cruise=mph * MPH, nav_lanes={0: nav}, controls=[(line, 'signal')])
  sim.tp.auto_left, sim.tp.left_range_m = 'auto', TEST_RANGE_M
  sim.raw = road_runs()
  red_light(sim, line, green)
  return sim, line


def red_light(sim, line: float, green: bool) -> None:
  """The sim's light oracle at line: red until the car has stood WAIT_S, then green (green: green throughout)."""
  sim.tp.oracle_ok = True
  state = {'stood': None}

  def lights(sim):
    if state['stood'] is None and sim.v < 0.05 and sim.s > line - 15.:
      state['stood'] = sim.t
    red = not green and (state['stood'] is None or sim.t < state['stood'] + WAIT_S)
    return oracle_lights('red' if red else 'green', line)(sim)
  sim.lights = lights
  sim.light = state


def through(sim, junction: float, limit_s: float = 120.) -> None:
  sim.drive(lambda sim: sim.s > junction + 30., limit_s)


def assert_went_on_green(sim, line: float, go_s: float = 4.):
  """Stood at the light's line through its red, never creeping, and moving within go_s of its green: no stall."""
  k = stood_at(sim, line)
  green_t = sim.light['stood'] + WAIT_S
  held = [f for f in sim.frames[k:] if f.t < green_t]
  assert held and all(f.v < 0.05 for f in held), ("crept on the red", trace(sim))
  moving = next((f for f in sim.frames if f.t > green_t and f.v > 1.), None)
  assert moving is not None and moving.t - green_t < go_s, ("stalled on the green", trace(sim))
  return k


# ------------------------------------------------------------------------------------------- 1, 2: a turn lane at a light

@pytest.mark.parametrize("signals", ['car', 'driver'])
@pytest.mark.parametrize("mph", [25, 35])
def test_a_right_at_a_red_changes_into_its_turn_lane_early_stops_and_turns_on_the_green(mph, signals):
  """One change toward the curb into the right-turn lane, made early in the window (settled in the lane with the window's
  end still ahead), signalled; the car stands at the red's line in the turn lane, goes on the green and turns with its lamp
  on. The first drive's failure: in the straight lane at the light, the turn given up."""
  sim, line = right_at_a_signal(mph, signals)
  through(sim, 450.)
  assert_signalled_changes(sim, 'right', 1, before_m=lane_change_window(mph * MPH)[1])
  k = assert_went_on_green(sim, line)
  assert sim.frames[k].lane == 1
  assert_turned(sim, 'right', 1, 450.)


@pytest.mark.parametrize("signals", ['car', 'driver'])
@pytest.mark.parametrize("mph", [25, 35])
def test_a_left_at_a_red_changes_into_its_turn_lane_and_takes_the_unprotected_left_on_the_green(mph, signals):
  """The same as a left from the left-turn lane by the centre line, the autonomous unprotected left's wait at its hold line
  through the red and its go on the green with nobody coming."""
  sim, line = left_at_a_signal(mph, signals)
  through(sim, 450.)
  assert_signalled_changes(sim, 'left', 1, before_m=lane_change_window(mph * MPH)[1])
  k = assert_went_on_green(sim, line, go_s=6.)
  assert sim.frames[k].lane == 0
  assert_turned(sim, 'left', 0, 450.)
  assert any(str(f.plan.controlPrompt) == 'leftGo' for f in sim.frames)


# ------------------------------------------------------------------------------------------- 3: slow traffic

@pytest.mark.parametrize("signals", ['car', 'driver'])
@pytest.mark.parametrize("mph", [12, 18])
def test_a_right_approached_in_slow_traffic_under_the_old_20_mph_floor_still_gets_its_lane(mph, signals):
  """Scenario 1 at 12-18 mph, under stock's 20 mph floor for a lane change (which TurnPilot's own changes no longer have:
  NAV_LANE_CHANGE_SPEED_MIN). The change still goes in time, and the turn from the turn lane."""
  sim, line = right_at_a_signal(mph, signals, junction=350.)
  through(sim, 350., limit_s=150.)
  assert_signalled_changes(sim, 'right', 1, before_m=lane_change_window(mph * MPH)[1])
  assert sim.frames[assert_went_on_green(sim, line)].lane == 1
  assert_turned(sim, 'right', 1, 350.)


# ------------------------------------------------------------------------------------------- 4: no change needed

@pytest.mark.parametrize("signals", ['car', 'driver'])
@pytest.mark.parametrize("mph", [25, 35])
def test_a_right_from_the_curb_lane_at_a_green_is_taken_cleanly_without_a_stop(mph, signals):
  """Already in the turn lane: no change, the turn's speed cap and no stop (through greens unslowed), the lamp on, turned.
  It comes to the corner at 3.5 m/s or less from 15 m out (turn_speed.V_CORNER_CURB): BeamNG's rights taken came so, and
  the ones at 4.6-4.7 m/s there were started 4-6 m out and lost (F3 right_green, 2026-11-01: abandoned at 14 deg)."""
  sim, _ = right_at_a_signal(mph, signals, lane=1, green=True)
  through(sim, 450.)
  assert not lane_changes(sim) and all(f.lane == 1 for f in sim.frames), trace(sim)
  assert min(f.v for f in sim.frames if f.s < 450.) > 2., ("stopped at a green", trace(sim))
  assert max(f.v for f in sim.frames if 435. <= f.s <= 450.) <= 3.5, ("into the corner too fast", trace(sim))
  assert_turned(sim, 'right', 1, 450.)


@pytest.mark.parametrize("signals", ['car', 'driver'])
@pytest.mark.parametrize("side", ['right', 'left'])
def test_a_turn_at_a_stop_sign_stops_goes_by_itself_and_turns(tmp_path, side, signals):
  """A one-lane road (no change) to test_turnpilotd_stop's surveyed all-way: the car stops at the sign, the decider's go,
  the turn with its lamp on."""
  junction = S_LINE + 13.
  other = 'left' if side == 'right' else 'right'
  sim = StopLoop(tmp_path, [(junction, 'turn', side), (2000., 'turn', other)], signals=signals,
                 info_kw={'maneuver': side})
  through(sim, junction, limit_s=90.)
  assert not lane_changes(sim), trace(sim)
  k = next((k for k, f in enumerate(sim.frames) if f.v < 0.05 and f.s > S_LINE - 15.), None)
  assert k is not None and abs(sim.frames[k].s - S_LINE) < 6., ("never stopped at the sign", trace(sim))
  assert_turned(sim, side, 0, junction)


# ------------------------------------------------------------------------------------------- 5: two changes

@pytest.mark.parametrize("signals", ['car', 'driver'])
@pytest.mark.parametrize("mph", [25, 35])
def test_two_changes_across_a_three_lane_road_into_the_far_sides_left_lane(mph, signals):
  """The curb lane of three to the left-turn lane by the centre line: two changes, each signalled, the second after the
  first has settled, both early enough; the stop at the red and the unprotected left on the green."""
  sim, line = left_at_a_signal(mph, signals, lanes=3, lane=2, nav=THREE_LANES_LEFT_TURN, junction=550.)
  through(sim, 550., limit_s=150.)
  assert_signalled_changes(sim, 'left', 2, before_m=lane_change_window(mph * MPH)[1])
  assert sim.frames[assert_went_on_green(sim, line, go_s=6.)].lane == 0
  assert_turned(sim, 'left', 0, 550.)


@pytest.mark.parametrize("signals", ['car', 'driver'])
def test_two_changes_toward_the_curb_into_a_right_turn_lane(signals):
  """The mirror: the left lane of three to a right-turn lane at the curb, two changes right, the red and the turn."""
  sim, line = right_at_a_signal(30, signals, lanes=3, lane=0, nav=THREE_LANES_RIGHT_TURN, junction=550.)
  through(sim, 550., limit_s=150.)
  assert_signalled_changes(sim, 'right', 2, before_m=lane_change_window(30 * MPH)[1])
  assert sim.frames[assert_went_on_green(sim, line)].lane == 2
  assert_turned(sim, 'right', 2, 550.)


# ------------------------------------------------------------------------------------------- 6: the blind spot

def blind_spot(sim, side: str, for_s: float | None):
  """The car's blind spot monitor on side occupied from the change's first offer for for_s (None: for good)."""
  state = {'from': None}

  def monitor(s):
    if state['from'] is None and s.trace and s.trace[-1][2].laneChangeOffer == SIDE[side]:
      state['from'] = s.t
    on = state['from'] is not None and (for_s is None or s.t < state['from'] + for_s)
    return (on and side == 'left', on and side == 'right')
  sim.blindspot = monitor
  return state


@pytest.mark.parametrize("signals", ['car', 'driver'])
def test_a_blind_spot_occupied_for_a_while_holds_the_change_then_it_goes_and_the_turn_is_taken(signals):
  """Scenario 1 at 30 mph with a car in the right blind spot from the change's offer for 4 s: no change while it is there
  (DesireHelper's own check, and under TurnPilotAlwaysDrive the offer held, LC-01), then the change, early enough, and the
  turn from the turn lane."""
  sim, line = right_at_a_signal(30, signals)
  occupied = blind_spot(sim, 'right', 4.)
  through(sim, 450.)
  changes = assert_signalled_changes(sim, 'right', 1, before_m=lane_change_window(30 * MPH)[1])
  assert occupied['from'] is not None and sim.frames[changes[0][0]].t >= occupied['from'] + 4., trace(sim)
  assert sim.frames[assert_went_on_green(sim, line)].lane == 1
  assert_turned(sim, 'right', 1, 450.)


@pytest.mark.parametrize("signals", ['car', 'driver'])
def test_a_blind_spot_that_never_clears_skips_the_turn_out_loud_never_turned_from_the_wrong_lane(signals):
  """The blind spot occupied from the offer on: the change never goes, the turn is skipped out loud before the light
  ("Can't Reach Turn Lane", skipReason laneBlocked: navd reroutes, LC-03), and the car goes straight on through the light
  from the lane it is in, never turning across the next one."""
  sim, line = right_at_a_signal(30, signals)
  blind_spot(sim, 'right', None)
  through(sim, 450.)
  assert not lane_changes(sim) and all(f.lane == 0 for f in sim.frames), trace(sim)
  plans = [f.plan for f in sim.frames]
  first = next((k for k, p in enumerate(plans) if p.tooFastToTurn), None)
  assert first is not None and plans[first].skipReason == 'laneBlocked', trace(sim)
  assert sim.frames[first].s < line, ("skipped only at the light", trace(sim))
  assert all(abs(f.heading) < 1. for f in sim.frames), ("turned from the wrong lane", trace(sim))
  assert not any(p.desire in (Desire.turnLeft, Desire.turnRight) for p in plans), trace(sim)
  assert_went_on_green(sim, line)  # straight on through the light, the route recomputed by navd
