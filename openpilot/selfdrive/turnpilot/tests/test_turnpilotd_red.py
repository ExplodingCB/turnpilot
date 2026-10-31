"""No turn or go on red, ever (SAT_VALIDATION row 14, red_right_g62; Kellen: "We do not have to support right on red, it
should always stop on red"): a right, a left and straight on at a red, with and without the sim's 2 m creep over the line."""
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.hooks import plan_accel
from openpilot.selfdrive.turnpilot.map_controls import RED_PAST_REASON
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import LEFT_AT_A_RED, Sim, oracle_lights

Desire = log.Desire
TURNS = (Desire.turnLeft, Desire.turnRight)
S_SIGNAL = 290.
ROUTES = {'right': ([(300., "turn", "right", junction_of("right")), (2000., "turn", "left")], Desire.turnRight),
          'left': (LEFT_AT_A_RED, Desire.turnLeft),
          'straight': ([(2000., "turn", "right")], Desire.none)}
CREEP_V = 1.1  # m/s: BeamNG's light brake below 3 m/s gains speed (g62: 0.68-1.1 m/s under -0.67 to -1.11 asked)


@pytest.mark.parametrize("overrun", [0., 2.])
@pytest.mark.parametrize("maneuver", ['right', 'left', 'straight'])
def test_no_turn_desire_lift_or_go_comes_on_a_red(maneuver, overrun):
  maneuvers, want = ROUTES[maneuver]
  sim = Sim(maneuvers, controls=[(S_SIGNAL, 'signal')])
  sim.go_taps = False
  sim.tp.oracle_ok = True
  light = {'state': 'red'}
  sim.lights = lambda sim: oracle_lights(light['state'], S_SIGNAL)(sim)

  def creep(sim, plan):
    a = min(plan_accel(plan.accelCap, plan.speedCap, sim.v), (sim.cruise - sim.v) / 2., 1.5)
    if overrun and S_SIGNAL - 20. < sim.s < S_SIGNAL + overrun and sim.v < CREEP_V and a < 0.:
      return (CREEP_V - sim.v) / 0.5  # it creeps on whatever is asked, over the line
    return a
  sim.accel = creep
  plans = []
  while not (sim.v < 0.01 and sim.s > S_SIGNAL - 30.):
    plans.append(sim.step(engaged=True))
    assert sim.t < 90., "never stood"
  s_stand = sim.s
  plans += [sim.step(engaged=True) for _ in range(int(15. / DT_MDL))]
  assert sim.v == 0. and sim.s - s_stand < 0.05 and s_stand < S_SIGNAL + overrun + 1.
  assert not any(p.desire in TURNS for p in plans), "a turn desire on red"
  assert not sim.tp.go_cert.active and plans[-1].shouldStop and plans[-1].controlPrompt == 'none'
  if overrun:
    assert plans[-1].reason == RED_PAST_REASON
  light['state'] = 'green'
  go = []
  while sim.trace[-1][2].shouldStop:
    go.append(sim.step(engaged=True))
    assert len(go) < 100
  assert go[-1].desire == want  # the go's pulse at standstill on the green (none straight on)


def test_the_cameras_shadow_red_alone_holds_the_turn_desire_until_it_reads_green():
  # Saturday's car: no source TurnPilot accepts reads the light (the model alone stops for it), the camera's shadow reader
  # reads it red. No turn desire while it does, standing or rolling; it reads green and the turn's desire goes out
  from openpilot.selfdrive.turnpilot.tests.test_turnpilotd_go import RIGHT_AT_A_SIGNAL, camera_reads, model_stops_at
  sim = Sim(RIGHT_AT_A_SIGNAL, controls=[(S_SIGNAL, 'signal')])
  sim.go_taps = False
  state = {'s': 'red'}
  sim.lights = lambda sim: camera_reads(state['s'])(sim)
  sim.accel = model_stops_at(289.)
  plans = []
  while not (sim.v < 0.05 and sim.s > 280.):
    plans.append(sim.step(engaged=True))
    assert sim.t < 90.
  plans += [sim.step(engaged=True) for _ in range(int(10. / DT_MDL))]
  sim.accel = lambda sim, plan: min(1., (sim.cruise - sim.v) / 2.)  # the model pulls away, the camera still red
  while sim.s < 291.:
    plans.append(sim.step(engaged=True))
  assert not any(p.desire in TURNS for p in plans), "a turn desire on the camera's red"
  state['s'] = 'green'
  later = [sim.step(engaged=True) for _ in range(int(1.5 / DT_MDL))]
  assert any(p.desire == Desire.turnRight for p in later)
