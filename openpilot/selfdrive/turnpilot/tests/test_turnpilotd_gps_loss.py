"""TurnPilot through a GPS loss, navd and turnpilotd in one loop (navigation/hold.py, turnpilotd.py gps_lost). The first real
drive lost GPS standing at a red before a left and dropped back to plain openpilot. Here navd itself (navd_loop) runs at its
1 Hz in the closed-loop Sim: the car stops at the red, the fix wanders, goes and comes back wrong while it stands, stays out
through the left after the green, and returns past it. The left is driven on the same maneuver, never dropped or rerouted;
navd's position lost hands back loudly instead of dropping silently to stock."""
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import TAKE_CONTROL_PROMPTS, plan_accel
from openpilot.selfdrive.turnpilot.tests.navd_loop import JUNCTION_M, NavdLoop, yaw_rate
from openpilot.selfdrive.turnpilot.tests.synthetic import nav_instruction
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim, rising
from openpilot.selfdrive.turnpilot.turnpilotd import GPS_LOST_REASON

Desire = log.Desire
RED_S = 30.  # s the car stands at the red
WANDERING = [(35., 0., 45.), (-30., 20., 60.), (0., 35., 45.), (25., -25., 80.), (40., 10., 45.), (-10., -35., 50.)]
CLAIMING_GOOD = [(20., -15., 8.), (18., 22., 6.), (-22., 5., 9.), (14., 30., 8.)]


class RedLight:
  """The car stops at line and stands RED_S engaged, the model holding it, while the GPS goes as the first drive's did: wandering
  past GPS_ACCURACY_MAX_M, then no fix, then fixes claiming a good accuracy metres off; after the green no fix until
  back_at, then the true place. Every other frame it follows the plan's limits, as Sim does."""
  def __init__(self, sim, line, back_at):
    self.sim, self.line, self.back_at = sim, line, back_at
    self.stood_t: float | None = None
    self.green = False

  def fix(self):
    if self.stood_t is None:
      return 'true' if not self.green else None
    t = self.sim.t - self.stood_t
    if t < 6.:
      return WANDERING[int(t) % len(WANDERING)]
    if t > RED_S - 4.:
      return CLAIMING_GOOD[int(t) % len(CLAIMING_GOOD)]
    return None

  def after(self):
    return None if self.sim.s < self.back_at else 'true'

  def accel(self, sim, plan):
    follow = min(plan_accel(plan.accelCap, plan.speedCap, sim.v), (sim.cruise - sim.v) / 2., 1.5)
    if plan.shouldStop:
      follow = min(follow, -1.)
    if self.green:
      return follow
    gap = self.line - sim.s
    if gap <= 2. or (self.stood_t is not None):
      if sim.v < 0.05 and self.stood_t is None:
        self.stood_t = sim.t
      if self.stood_t is not None and sim.t - self.stood_t >= RED_S:
        self.green, self.stood_t = True, None
        sim.model_holds = False
        return follow
      sim.model_holds = self.stood_t is not None
      return -3.5
    need = sim.v ** 2 / (2. * max(gap - 1., 0.3))
    return -need if need > 1.2 else follow


def drive_the_red(line):
  loop = NavdLoop()
  sim = Sim([(JUNCTION_M, 'turn', 'left')], cruise=10.)
  red = RedLight(sim, line, JUNCTION_M + 40.)
  navd = []

  def nav(sim, _fake):
    ni, valid = loop.tick(sim.s, sim.v, fix=red.fix() if not red.green else red.after())
    navd.append((sim.t, sim.s, ni, valid, red.stood_t is not None))
    return ni, valid
  sim.nav_filter, sim.accel = nav, red.accel
  ids = []
  while sim.s < JUNCTION_M + 80.:
    sim.step(engaged=True, yaw_rate=yaw_rate(sim.s, sim.v))
    ids.append((sim.s, sim.tp.follower.maneuver_id, sim.trace[-1][2]))
    assert sim.t < 200.
  return loop, sim, navd, ids


@pytest.mark.parametrize("line", [JUNCTION_M - 8., JUNCTION_M - 60.])
def test_gps_lost_at_a_red_before_a_left_keeps_the_route_and_drives_the_left_after_the_green(line):
  """line: first at the stop line, the left's desire already out (committed); or queued 60 m back, before it (not committed,
  where any invalid instruction used to drop the left)."""
  loop, sim, navd, ids = drive_the_red(line)
  standing = [(ni, valid) for _, _, ni, valid, stood in navd if stood]
  assert len(standing) >= RED_S - 1
  assert all(valid and ni.maneuverModifier == 'left' for ni, valid in standing)
  assert all(abs(ni.maneuverDistance - standing[0][0].maneuverDistance) < 0.5 for ni, _ in standing)
  assert {str(ni.positionSource) for ni, _ in standing} == {'deadReckoned'}
  # the left is the same maneuver from the approach to the junction: never dropped, replaced or rerouted
  left_ids = {i for s, i, _ in ids if JUNCTION_M - 150. < s < JUNCTION_M}
  assert len(left_ids) == 1 and len(loop.router.calls) == 1 and 'NavDestination' in loop.values
  plans = [p for _, _, p in ids]
  assert rising([(s, 0., p) for s, _, p in ids], 'desire', Desire.turnLeft)
  assert not any(p.turnMissed or p.tooFastToTurn or str(p.controlPrompt) in TAKE_CONTROL_PROMPTS for p in plans)
  # navd carried round the left with no fix, then back on GPS on the right's step
  through = [(s, ni) for _, s, ni, valid, _ in navd if JUNCTION_M + 15. < s < JUNCTION_M + 40.]
  assert through and all(ni.maneuverModifier == 'right' and str(ni.positionSource) == 'deadReckoned' for _, ni in through)
  assert str(navd[-1][2].positionSource) == 'gps' and navd[-1][3] and loop.engine.step_idx == 1
  after = [p for s, _, p in ids if s > JUNCTION_M + 30.]
  assert after and all(p.maneuverModifier == 'right' and p.desire == Desire.none for p in after)


def lost_sim(enabled_at=0.):
  """A Sim on a route whose navd says its position is lost from 5 s on (positionSource lost, invalid, as navd sends it),
  engaged from enabled_at."""
  sim = Sim([(2000., 'turn', 'left')], cruise=10.)

  def nav(sim, ni):
    if sim.t < 5. or sim.t > sim.back_t:
      return ni, True
    lost = nav_instruction(ni.maneuverDistance, valid=False)
    msg = lost.as_builder()
    msg.positionSource = 'lost'
    return msg.as_reader(), False
  sim.back_t = 1e9
  sim.nav_filter = nav
  return sim


def run(sim, seconds, engaged=True):
  return [sim.step(engaged=engaged) for _ in range(int(round(seconds / DT_MDL)))]


def test_navds_position_lost_hands_back_loudly_once_a_spell_until_the_driver_takes_over():
  """navd can't place the car (GPS out past the hold's limits, or the car off the route it carried): engaged and acting,
  gpsLostTakeControl (TAKE_CONTROL_PROMPTS: take control, the handback's chime) from its first frame, held until the driver
  disengages. Engaging again in the same spell isn't nagged again; a new spell after the fix came back is said again."""
  sim = lost_sim()
  plans = run(sim, 16.)
  onset = next(i for i, p in enumerate(plans) if str(p.controlPrompt) != 'none')
  assert 5. <= onset * DT_MDL <= 5. + 1. + DT_MDL  # navd's first lost message (1 Hz)
  assert 'gpsLostTakeControl' in TAKE_CONTROL_PROMPTS
  assert all(str(p.controlPrompt) == 'gpsLostTakeControl' and p.reason == GPS_LOST_REASON for p in plans[onset:])
  plans = run(sim, 2., engaged=False)  # the driver takes over
  assert all(str(p.controlPrompt) == 'none' for p in plans[1:])
  plans = run(sim, 3.)  # engaged again, the GPS still out: not said again
  assert all(str(p.controlPrompt) == 'none' for p in plans)
  sim.back_t = sim.t + 1.  # the fix back...
  run(sim, 3.)
  sim.back_t = 1e9         # ...and lost again: a new spell
  plans = run(sim, 3.)
  assert any(str(p.controlPrompt) == 'gpsLostTakeControl' for p in plans)


def test_navds_position_lost_while_disengaged_is_said_on_engaging():
  sim = lost_sim()
  plans = run(sim, 8., engaged=False)
  assert all(str(p.controlPrompt) == 'none' for p in plans)
  plans = run(sim, 2.)
  assert all(str(p.controlPrompt) == 'gpsLostTakeControl' for p in plans[1:])
