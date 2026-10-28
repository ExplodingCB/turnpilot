#!/usr/bin/env python3
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.turnpilot.go_override import GoLift
from openpilot.selfdrive.turnpilot.hooks import limits_plan, plan_accel
from openpilot.common.swaglog import cloudlog

J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
J_CRUISE_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN = -0.5
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]


def get_cruise_accel(v_cruise, v_ego, a_cruise_prev, a_target_prev, dt):
  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, ACCEL_MAX)
  j_cruise = np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, min(a_cruise_prev, a_target_prev) - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.accel_boost = AccelBoost()
    self.go_lift = GoLift()  # TurnPilot's bounded go (turnpilot/go_override.py)

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    if self.CP.openpilotLongitudinalControl:
      self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    else:
      self.mpc.set_cur_state(v_ego, sm['carState'].aEgo)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    self.a_cruise = get_cruise_accel(v_cruise, v_ego, self.a_cruise, self.output_a_target, self.dt)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    output_a_target_e2e = self.accel_boost.update(sm, output_a_target_e2e, output_a_target_mpc, self.a_cruise)

    tp = limits_plan(sm)
    # TurnPilot's turn speed and scene limits (a faulted plan's too), reported as a cruise limit
    tp_candidate = (plan_accel(tp.accelCap, tp.speedCap, v_ego), tp.shouldStop) if tp is not None else None
    # TurnPilot's bounded go (turnpilot/go_override.py): for a go turnpilotd certifies, and only while the model alone holds
    # the car, the e2e candidate lifted to +0.5 m/s^2 at most and its stop dropped, 3 s at most, under 3 m/s. Every other
    # candidate stays in the min() and the any()
    model_t = getattr(sm, 'logMonoTime', {}).get('modelV2')
    output_a_target_e2e, output_should_stop_e2e = self.go_lift.update(
      sm, v_ego, (output_a_target_mpc, output_should_stop_mpc), (self.a_cruise, cruise_should_stop), tp_candidate,
      (output_a_target_e2e, output_should_stop_e2e), self.dt, model_t * 1e-9 if model_t else None)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop),
                  (output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e)]
    if tp_candidate is not None:
      candidates.append((tp_candidate[0], LongitudinalPlanSource.cruise, tp_candidate[1]))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.accelBoost = float(self.accel_boost.total_boost)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.turnpilotGoLift = bool(self.go_lift.lifting)
    longitudinalPlan.turnpilotGoEnd = self.go_lift.end

    pm.send('longitudinalPlan', plan_send)
