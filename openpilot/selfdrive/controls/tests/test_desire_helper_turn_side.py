"""TurnPilot, 2026-11-07: the driver prompted to signal a route turn at any speed. Their blinker on the side of the turn
TurnPilot has armed (turnpilotPlan.turnSignal.side) is that turn's signal, never a lane change request: no preLaneChange
however fast the car goes, and none once the turn is no longer armed while the same lamp stays on. The other side's
blinker is still the driver's lane change, as in stock."""
from opendbc.car import structs
from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper

LCS = log.LaneChangeState
LCD = log.LaneChangeDirection


def cs(v: float, left: bool = False, right: bool = False):
  return structs.CarState(vEgo=v, leftBlinker=left, rightBlinker=right)


def run(dh, n, carstate, turn_side=LCD.none):
  for _ in range(n):
    dh.update(carstate, True, 0., LCD.none, turn_side)
  return dh.lane_change_state


def test_the_armed_turns_blinker_is_never_a_lane_change_at_speed():
  dh = DesireHelper()
  assert run(dh, 50, cs(20., right=True), LCD.right) == LCS.off
  assert dh.desire == log.Desire.none


def test_the_same_lamp_left_on_after_the_turn_is_no_new_blinker():
  dh = DesireHelper()
  run(dh, 50, cs(20., right=True), LCD.right)
  assert run(dh, 50, cs(20., right=True)) == LCS.off


def test_the_other_sides_blinker_is_still_the_drivers_lane_change():
  dh = DesireHelper()
  assert run(dh, 5, cs(20., left=True), LCD.right) == LCS.preLaneChange
  assert dh.lane_change_direction == LCD.left


def test_with_nothing_armed_a_blinker_at_speed_is_stocks_lane_change():
  dh = DesireHelper()
  assert run(dh, 5, cs(20., right=True)) == LCS.preLaneChange
  assert dh.lane_change_direction == LCD.right
