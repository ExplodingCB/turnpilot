"""Driver monitoring through TurnPilot's maneuvers (hooks.dm_maneuver_scan, policy.DriverMonitoring's maneuver_scan): an
attentive driver checking a junction or the mirrors isn't distracted, and everything else DM catches it still catches."""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.policy import DRIVER_MONITOR_SETTINGS, DriverMonitoring
from openpilot.selfdrive.turnpilot.hooks import JUNCTION_SCAN_M, JUNCTION_SCAN_V_MAX, active_plan, dm_maneuver_scan

AlertLevel = log.DriverMonitoringState.AlertLevel
State = custom.TurnpilotPlan.State
S = DRIVER_MONITOR_SETTINGS()


class FakeSM(dict):
  def __init__(self, msgs, valid=True):
    super().__init__(msgs)
    self.valid = dict.fromkeys(msgs, valid)
    self.recv_time = dict.fromkeys(msgs, time.monotonic())


def plan(shadow=False, state=State.following, **fields):
  msg = messaging.new_message('turnpilotPlan')
  p = msg.turnpilotPlan
  p.state, p.shadow = state, shadow
  p.maneuverDistance = 500.
  for path, value in fields.items():
    *parents, leaf = path.split('__')
    obj = p
    for name in parents:
      obj = getattr(obj, name)
    setattr(obj, leaf, value)
  return msg.as_reader().turnpilotPlan


def face(yaw=0., pitch=0., phone=0., blink=0.):
  """A face looking yaw rad (negative left, a left-hand drive car) and pitch rad (negative down) from the camera's axis."""
  ds = log.DriverStateV2.new_message()
  d = ds.leftDriverData
  d.faceOrientation, d.facePosition = [pitch, -yaw, 0.], [0., 0.]
  d.faceOrientationStd, d.facePositionStd = [0.01, 0.01, 0.01], [0.01, 0.01]
  d.faceProb, d.leftEyeProb, d.rightEyeProb = 1., 1., 1.
  d.leftBlinkProb = d.rightBlinkProb = blink
  d.phoneProb, d.sleepProb = phone, 0.
  return ds


def run(msgs, scan, lowspeed=False):
  """The alert levels of a drive, engaged and moving, with scan the maneuver's flag every frame."""
  DM, levels = DriverMonitoring(), []
  for msg in msgs:
    DM._update_states(msg, [0., 0., 0.], 8., True, lowspeed, maneuver_scan=scan)
    DM._update_events(False, True, lowspeed, False)
    levels.append(DM.alert_level)
  return levels, DM


def seconds(msg, s):
  return [msg] * int(s / DT_DMON)


ALERT_2 = S._VISION_POLICY_ALERT_2_TIMEOUT + 1.


def test_a_junction_glance_is_attentive_and_stock_without_the_maneuver():
  glance = face(yaw=-0.6)  # 34 deg left of the camera's axis: past stock's yaw threshold, inside its steering allowance
  stock, _ = run(seconds(glance, ALERT_2), scan=False)
  assert stock[int((S._VISION_POLICY_ALERT_1_TIMEOUT + 1.) / DT_DMON)] == AlertLevel.one
  assert stock[-1] == AlertLevel.two
  scanned, dm = run(seconds(glance, 60.), scan=True)
  assert all(level == AlertLevel.none for level in scanned)
  assert not dm.driver_distracted
  right, _ = run(seconds(face(yaw=0.6), 60.), scan=True)  # either way: cross traffic comes from both
  assert all(level == AlertLevel.none for level in right)


def test_the_allowance_is_bounded_by_stocks_steering_cap():
  # a head turned past the threshold plus _POSE_YAW_STEER_MAX_OFFSET (looking out of the side window) still alerts on time
  away = face(yaw=-(S._POSE_YAW_THRESHOLD + S._POSE_YAW_STEER_MAX_OFFSET + S._YAW_NATURAL_OFFSET + 0.1))
  levels, _ = run(seconds(away, ALERT_2), scan=True)
  assert levels[int((S._VISION_POLICY_ALERT_1_TIMEOUT + 1.) / DT_DMON)] == AlertLevel.one
  assert levels[-1] == AlertLevel.two


def test_head_down_phone_and_closed_eyes_still_alert_in_a_maneuver():
  for distracted in (face(pitch=-0.6), face(phone=0.9), face(blink=0.95)):
    levels, _ = run(seconds(distracted, S._VISION_POLICY_ALERT_3_TIMEOUT + 1.), scan=True)
    assert levels[int((S._VISION_POLICY_ALERT_1_TIMEOUT + 1.) / DT_DMON)] == AlertLevel.one
    assert levels[-1] == AlertLevel.three


def test_a_stop_signs_checks_leave_the_go_unnagged():
  # held at a stop sign checking both ways (low speed), then the go and the crossing, still checking: stock carries the
  # standstill's spent awareness into an alert at the launch; TurnPilot's junction leaves the attentive driver alone
  checks = (seconds(face(yaw=-0.6), 2.) + seconds(face(yaw=0.6), 2.)) * 5
  go = (seconds(face(yaw=-0.6), 1.5) + seconds(face(), 0.5)) * 3
  for scan, want in ((False, True), (True, False)):
    DM, levels = DriverMonitoring(), []
    for i, msg in enumerate(checks + go):
      lowspeed = i < len(checks)
      DM._update_states(msg, [0., 0., 0.], 0. if lowspeed else 5., True, lowspeed, maneuver_scan=scan)
      DM._update_events(False, True, lowspeed, False)
      levels.append(DM.alert_level)
    assert any(level != AlertLevel.none for level in levels[len(checks):]) == want


def test_maneuver_glances_never_teach_the_natural_pose():
  DM = DriverMonitoring()
  for _ in range(int(5. / DT_DMON)):
    DM._update_states(face(yaw=-0.6), [0., 0., 0.], 20., True, False, maneuver_scan=True)
  assert DM.pose.yaw_offsetter.filtered_stat.n == 0
  DM._update_states(face(), [0., 0., 0.], 20., True, False)
  assert DM.pose.yaw_offsetter.filtered_stat.n == 1


def test_demo_mode_ignores_the_maneuver():
  DM = DriverMonitoring()
  DM._update_states(face(yaw=-0.6), [0., 0., 0.], 8., True, False, demo_mode=True, maneuver_scan=True)
  assert DM.pose.scan_yaw_offset == 0. and DM.distracted_types['pose']


def test_which_maneuvers_scan():
  scan = lambda p, v=5.: dm_maneuver_scan(p, v)  # noqa: E731
  assert not scan(None) and not scan(plan())
  # a lane change, warned of or under way, at any speed: the mirrors
  assert scan(plan(laneChangeOffer='left'), 30.) and scan(plan(state=State.laneChange, laneChange='right'), 30.)
  # a junction TurnPilot is working, under JUNCTION_SCAN_V_MAX
  junctions = [plan(controlPrompt='waitingForCross'), plan(controlPrompt='stopGo'), plan(controlPrompt='waitingForGap'),
               plan(controlPrompt='creepingForView'), plan(controlPrompt='leftGo'), plan(controlPrompt='yieldAhead'),
               plan(detectorView='stopSweep'), plan(detectorView='stopSweepGo'),
               plan(mapControl__kind='stop', mapControl__action='held'), plan(mapControl__kind='yield', mapControl__action='creeping'),
               plan(stopGo__state='observe'), plan(stopGo__state='committed'), plan(unprotectedLeft__state='waitGap'),
               plan(unprotectedLeft__state='go'), plan(state=State.turning),
               plan(state=State.approaching, maneuverDistance=JUNCTION_SCAN_M - 1.)]
  for p in junctions:
    assert scan(p), p
    assert not scan(p, JUNCTION_SCAN_V_MAX + 0.5), p  # through it: the eyes go back ahead
  # nothing to check: a far turn, a bend, a stop being approached, a signal released, the left still far off
  for p in (plan(state=State.approaching, maneuverDistance=JUNCTION_SCAN_M + 10.),
            plan(state=State.approaching, maneuverDistance=10., maneuverBend=True),
            plan(mapControl__kind='stop', mapControl__action='approaching'), plan(mapControl__kind='signal', mapControl__action='released'),
            plan(unprotectedLeft__state='approach'), plan(stopGo__state='approach'), plan(controlPrompt='stopSignTakeControl')):
    assert not scan(p), p


def test_only_a_plan_turnpilot_drives_with_relaxes_dm():
  turning = plan(state=State.turning)
  assert dm_maneuver_scan(active_plan(FakeSM({'turnpilotPlan': turning})), 5.)
  for sm in (FakeSM({'turnpilotPlan': plan(shadow=True, state=State.turning)}),
             FakeSM({'turnpilotPlan': plan(state=State.disabled, controlPrompt='stopGo')}),
             FakeSM({'turnpilotPlan': turning}, valid=False),
             messaging.SubMaster(['turnpilotPlan'])):  # TurnPilot off: none ever sent
    assert not dm_maneuver_scan(active_plan(sm), 5.)
