"""controlsd glue: choose the lateral curvature source and publish alpamayoState.

Only the lateral target changes. Longitudinal, curvature clipping, panda safety,
torque/rate limits and driver monitoring are untouched.
"""
from __future__ import annotations

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.alpamayo.gate import alpamayo_mode
from openpilot.selfdrive.alpamayo.lateral_source import LateralSource, LateralSourceConfig, Plan, OK

Mode = custom.AlpamayoState.Mode
MODES = {"off": Mode.off, "shadow": Mode.shadow, "active": Mode.active}
PARAMS_EVERY = int(1.0 / DT_CTRL)


def read_mode(params: Params) -> int:
  return MODES[alpamayo_mode(params)]


def plan_from_msg(md) -> Plan:
  t = np.array(md.position.t, dtype=np.float64)
  return Plan(int(md.timestampEof), np.array(md.position.x, dtype=np.float64),
              np.array(md.position.y, dtype=np.float64), t)


class AlpamayoLateral:
  SUBS = ["alpamayoModelV2"]
  PUBS = ["alpamayoState"]

  def __init__(self, params: Params):
    self.params = params
    self.mode = read_mode(params)
    self.src = LateralSource(self._cfg())
    self.frame = 0
    self.last_plan_frame = -1
    self.desired_curvature = 0.0
    self.using = False
    self.inference_time = 0.0

  def _cfg(self) -> LateralSourceConfig:
    def f(key, default):
      try:
        v = self.params.get(key)
        return float(v) if v is not None else default
      except (TypeError, ValueError):
        return default
    return LateralSourceConfig(max_plan_age_s=f("AlpamayoMaxPlanAgeS", 1.5), max_disagree_m=f("AlpamayoMaxDisagreeM", 1.5),
                               max_speed=f("AlpamayoMaxSpeed", 9.0))

  def update(self, sm, CS, now_ns: int, lat_delay: float, calibrated_pose, lat_active: bool) -> float | None:
    """Curvature to use instead of modelV2.action.desiredCurvature, or None for stock."""
    self.frame += 1
    if self.frame % PARAMS_EVERY == 0:
      self.mode = read_mode(self.params)
      self.src.cfg = self._cfg()
    if self.mode == Mode.off:
      self.using = False
      return None

    if sm.updated["alpamayoModelV2"]:
      md = sm["alpamayoModelV2"]
      self.src.new_plan(plan_from_msg(md), sm.valid["alpamayoModelV2"] and len(md.position.x) > 1)
      self.inference_time = md.modelExecutionTime

    yaw_rate = float(calibrated_pose.angular_velocity.yaw) if calibrated_pose is not None else 0.0
    stock = sm["modelV2"]
    lane_change = stock.meta.laneChangeState != log.LaneChangeState.off
    curv, _ = self.src.update(now_ns, max(CS.vEgo, 0.0), yaw_rate, lat_delay,
                              np.array(stock.position.x), np.array(stock.position.y), lane_change)
    if curv is not None:
      self.desired_curvature = curv
    self.using = self.mode == Mode.active and curv is not None and lat_active
    return curv if self.using else None

  def publish(self, pm, sm) -> None:
    msg = messaging.new_message("alpamayoState")
    st = msg.alpamayoState
    st.mode = self.mode
    st.usingAlpamayo = self.using
    st.fallbackReason = self.src.reason if self.mode != Mode.off else "off"
    st.planAgeS = float((sm.logMonoTime["carState"] - self.src.plan.t_ns) * 1e-9) if self.src.plan else -1.0
    st.disagreeM = float(self.src.disagree_m)
    st.desiredCurvature = float(self.desired_curvature)
    st.inferenceTimeS = float(self.inference_time)
    pm.send("alpamayoState", msg)

  @property
  def ok(self) -> bool:
    return self.src.reason == OK
