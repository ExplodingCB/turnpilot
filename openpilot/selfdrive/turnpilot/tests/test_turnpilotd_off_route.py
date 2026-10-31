"""An off-route turn is a mistake said loudly (SAT_VALIDATION row 1, route_demo: the route went straight on, TurnPilot sent no
desire and the model turned right on its own into the oncoming lane and a roadside object)."""
import math

import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.navigation.route import METERS_PER_DEGREE
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


@pytest.mark.parametrize("bend, deg, fires", [(0., 60., True), (60., 60., False), (60., -60., True)])
def test_a_bend_the_route_makes_that_way_in_its_step_is_no_off_route_turn(bend, deg, fires):
  # the step from a straight-on junction at 100 m (bearing out 0) to a right at 400 m whose bearing in is the bend's
  sim = Sim([(100., "continue", "straight", (0., 0., 2)), (400., "turn", "right", (bend, (bend + 90.) % 360., 2))], cruise=V)
  sim.v = V
  plans = turn_at(sim, 200., deg, until=280.)
  assert any(p.controlPrompt == 'offRouteTakeControl' for p in plans) == fires


# the route's own shape (navInstruction.routeTurns): the road bending with no maneuver is no off-route turn
R = 30.  # m: a suburban street's tight bend


def arcs(*legs):
  """A route's bearing (deg, unwrapped, + right) at s from legs [(s0, deg, radius), ...]: straight on, then each bend of deg
  at radius from its s0."""
  def bearing(s):
    b = 0.
    for s0, deg, radius in legs:
      length = math.radians(abs(deg)) * radius
      b += math.copysign(min(max(s - s0, 0.), length) / radius, deg) * 180. / math.pi
    return b
  return bearing


def polyline(bearing_of, length, step=2.):
  """The route's (longitude, latitude) polyline from bearing_of, s = 0 to length, at the equator."""
  pts, x, y, s = [(0., 0.)], 0., 0., 0.
  while s < length:
    b = math.radians(bearing_of(s + step / 2))
    x, y, s = x + step * math.sin(b), y + step * math.cos(b), s + step
    pts.append((x / METERS_PER_DEGREE, y / METERS_PER_DEGREE))
  return np.array(pts)


def drive(sim, bearing_of, until, extra=lambda s: 0.):
  """The car follows the road bearing_of (its yaw rate the road's curvature at its speed), plus extra(s) deg of its own."""
  plans = []
  while sim.s < until:
    h = 0.5
    deg_per_m = (bearing_of(sim.s + h) + extra(sim.s + h) - bearing_of(sim.s - h) - extra(sim.s - h)) / (2 * h)
    plans.append(sim.step(engaged=True, yaw_rate=-math.radians(deg_per_m) * sim.v))
  return plans


def fired(plans):
  return [i for i, p in enumerate(plans) if p.controlPrompt == 'offRouteTakeControl']


ROADS = {
  's_bend': arcs((200., -90., R), (200. + math.pi / 2 * R, 90., R)),
  'street_bend': arcs((200., 90., R)),
  'gentle_curve': arcs((150., 90., 300.)),
}


# navd's place on the route against the car's (Sim's lag, m behind): its 1 Hz fix 0.5 s behind, or ahead of the car, as GPS's
# along-track error and along_path's bias at a bend put it
PLACES = {'lag_0.5s': lambda v: 0.5 * v, 'ahead_5m': lambda v: -5., 'ahead_10m': lambda v: -10.}


@pytest.mark.parametrize("place", sorted(PLACES))
@pytest.mark.parametrize("v", [8., 15.])
@pytest.mark.parametrize("road", sorted(ROADS))
def test_a_bend_the_road_makes_with_no_maneuver_never_fires(road, v, place):
  bearing = ROADS[road]
  sim = Sim([(2000., "turn", "left")], cruise=v, shape=polyline(bearing, 2100.), lag=PLACES[place](v))
  sim.v = v
  assert not fired(drive(sim, bearing, 800.))


def test_without_the_routes_shape_the_s_bend_is_judged_as_before():
  # navd's message without routeTurns (a navd before it): only the step's bend, which an S-bend nets to nothing
  sim = Sim([(2000., "turn", "left")], cruise=8.)
  sim.v = 8.
  assert fired(drive(sim, ROADS['s_bend'], 400.))


def test_a_lane_change_on_a_straight_route_never_fires():
  straight = arcs()
  sim = Sim([(2000., "turn", "left")], cruise=13., shape=polyline(straight, 2100.), lag=6.)
  sim.v = 13.
  change = arcs((200., 8., 60.), (200. + math.radians(8.) * 60., -8., 60.))  # one lane right over about 17 m
  assert not fired(drive(sim, straight, 400., extra=change))


@pytest.mark.parametrize("lag", [0., 7.])
def test_row_1_the_model_turning_off_a_straight_route_still_fires_at_the_same_point(lag):
  def row_1(shape):
    sim = Sim([(400., "turn", "right", junction_of("right")), (2000., "turn", "left")], cruise=V,
              shape=polyline(arcs(), 2100.) if shape else None, lag=lag)
    sim.v = V
    while sim.s < 80.:
      sim.step(engaged=True)
    return fired(turn_at(sim, 100., 90.))
  with_shape, without = row_1(True), row_1(False)
  assert with_shape and with_shape[0] == without[0]
