"""An off-route turn is a mistake said loudly (SAT_VALIDATION row 1, route_demo: the route went straight on, TurnPilot sent no
desire and the model turned right on its own into the oncoming lane and a roadside object)."""
import math

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.turnpilotd import OFF_ROUTE_TURN_DEG, OFF_ROUTE_TURN_M, OFF_ROUTE_TURN_REASON
from openpilot.selfdrive.turnpilot.tests.synthetic import junction_of
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim

V = 5.


def turn_at(sim, s0, deg, rate=0.35, until=None):
  """Drive on, the car yawing deg (+ right) from s0 at rate rad/s, to s = until. Returns the plans."""
  plans, turned = [], 0.
  until = s0 + 60. if until is None else until
  while sim.s < until:
    yaw = 0.
    if sim.s >= s0 and turned < abs(deg):
      yaw = -math.copysign(rate, deg)  # left positive
      turned += math.degrees(rate * DT_MDL)
    plans.append(sim.step(engaged=True, yaw_rate=yaw))
  return plans


def test_the_model_turning_off_a_straight_route_hands_back_loudly_and_never_brakes():
  sim = Sim([(400., "turn", "right", junction_of("right")), (2000., "turn", "left")], cruise=V)
  sim.v = V
  while sim.s < 80.:
    sim.step(engaged=True)
  plans = turn_at(sim, 100., 90.)
  fired = [i for i, p in enumerate(plans) if p.controlPrompt == 'offRouteTakeControl']
  assert fired and all(plans[i].reason == OFF_ROUTE_TURN_REASON for i in fired)
  start = int(round(20. / (V * DT_MDL)))  # the yaw starts 100 m in: 20 m after turn_at's first frame at 80 m
  assert (fired[0] - start) * V * DT_MDL <= OFF_ROUTE_TURN_M  # within its 30 m
  assert fired[-1] == len(plans) - 1  # held until the driver takes over
  assert not any(p.shouldStop for p in plans) and all(p.accelCap >= 0. for p in plans)


def test_the_routes_own_turn_and_a_lane_change_sized_yaw_never_fire():
  sim = Sim([(150., "turn", "right", junction_of("right")), (2000., "turn", "left")], cruise=V)
  sim.v = V
  plans = turn_at(sim, 145., 90., until=260.)  # the route's right, its desire out
  plans += turn_at(sim, 300., OFF_ROUTE_TURN_DEG - 10., until=380.)  # a drift under the threshold, far from any maneuver
  assert not any(p.controlPrompt == 'offRouteTakeControl' for p in plans)


@pytest.mark.parametrize("bend, deg, fires", [(0., 45., True), (60., 45., False), (60., -45., True)])
def test_a_bend_the_route_makes_that_way_in_its_step_is_no_off_route_turn(bend, deg, fires):
  # the step from a straight-on junction at 100 m (bearing out 0) to a right at 400 m whose bearing in is the bend's
  sim = Sim([(100., "continue", "straight", (0., 0., 2)), (400., "turn", "right", (bend, (bend + 90.) % 360., 2))], cruise=V)
  sim.v = V
  plans = turn_at(sim, 200., deg, until=280.)
  assert any(p.controlPrompt == 'offRouteTakeControl' for p in plans) == fires
