from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
# TurnPilot: its own lane changes are commands, not a blinker read as intent, so they need only a car moving fast enough to
# move over; the 20 mph floor (a driver's blinker under it is a turn) stays for the driver's
NAV_LANE_CHANGE_SPEED_MIN = 5 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

  @staticmethod
  def get_lane_change_direction(left_blinker):
    return LaneChangeDirection.left if left_blinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, nav_lane_change=LaneChangeDirection.none,
             turn_side=LaneChangeDirection.none):
    v_ego = carstate.vEgo
    # TurnPilot: a nav lane change request is the lane change itself, in its direction, with no blinker, nudge or 20 mph
    # floor; the blind spot check stays. The driver's blinker the other way cancels it
    driver_dir = LaneChangeDirection.none if carstate.leftBlinker == carstate.rightBlinker else \
      self.get_lane_change_direction(carstate.leftBlinker)
    # TurnPilot: the driver's blinker on the side of the route turn TurnPilot has armed (turnpilotPlan.turnSignal.side) is
    # that turn's signal, which TurnPilot asked for, never a lane change request, at any speed (Kellen, 2026-11-07: no speed
    # floor on the prompt to signal; a stock heuristic reading the blinker as intent is not TurnPilot's)
    turn_lamp = turn_side != LaneChangeDirection.none and driver_dir == turn_side and nav_lane_change == LaneChangeDirection.none
    if turn_lamp:
      driver_dir = LaneChangeDirection.none
    nav = nav_lane_change != LaneChangeDirection.none and driver_dir in (LaneChangeDirection.none, nav_lane_change)
    left_blinker = nav_lane_change == LaneChangeDirection.left if nav else driver_dir == LaneChangeDirection.left
    one_blinker = nav or driver_dir != LaneChangeDirection.none
    below_lane_change_speed = v_ego < (NAV_LANE_CHANGE_SPEED_MIN if nav else LANE_CHANGE_SPEED_MIN)

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and (nav or not self.prev_one_blinker) and \
         not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(left_blinker)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(left_blinker)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))
        torque_applied = torque_applied or nav

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        elif torque_applied and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(left_blinker)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    # the turn's lamp counts as seen: still on once the turn is no longer armed, it is no new blinker (no rising edge)
    self.prev_one_blinker = (one_blinker or turn_lamp) and lateral_active

    self.desire = log.Desire.none
    if self.lane_change_state == LaneChangeState.laneChangeStarting:
      if self.lane_change_direction == LaneChangeDirection.left:
        self.desire = log.Desire.laneChangeLeft
      elif self.lane_change_direction == LaneChangeDirection.right:
        self.desire = log.Desire.laneChangeRight
