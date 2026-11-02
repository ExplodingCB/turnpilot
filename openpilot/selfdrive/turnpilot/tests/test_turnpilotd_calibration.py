"""turnpilotd while the cameras calibrate (turnpilotd.calibration_wait): after an OS update reset calibration, the phone and
the comma said "detector not running: take control" while the car was only calibrating. The tracker takes no run before
calibration, so the scene is unavailable whatever the detector does: TurnPilot waits and says it calibrates, with
calibrationd's progress, hands nothing back, and its detector deadlines hold through it, to run on from where they were."""
from unittest import mock

from openpilot.cereal import custom, log, messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.turnpilot.handback import Handback
from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import calibration
from openpilot.selfdrive.turnpilot.scene.tracker import STALE_S
from openpilot.selfdrive.turnpilot.tests.test_turnpilotd import Sim
from openpilot.selfdrive.turnpilot.turnpilotd import (DETECTOR_DEADLINE_S, DETECTOR_START_DEADLINE_S, HANDBACK_REASON, WAITING_REASON,
                                                       calibration_wait)

State = custom.TurnpilotPlan.State
LaneChange = custom.TurnpilotPlan.LaneChange
Desire = log.Desire


def cal(status, percent=0, rpy=True):
  msg = messaging.new_message('extrinsicsCalibration')
  msg.extrinsicsCalibration.calStatus, msg.extrinsicsCalibration.calPerc = status, percent
  if rpy:
    msg.extrinsicsCalibration.rpyCalib = [0., 0.03, 0.]
  return msg.as_reader().extrinsicsCalibration


def calibrating_sim(*args, **kwargs):
  """test_turnpilotd's Sim, its plans given sim.calibrating (calibration_wait's reason, '' once calibrated), as main() does."""
  sim = Sim(*args, **kwargs)
  plan = sim.tp.plan
  sim.calibrating = ''
  sim.tp.plan = lambda *a, **k: plan(*a, calibrating=sim.calibrating, **k)
  return sim


def run(sim, seconds, **kw):
  return [sim.step(**kw) for _ in range(round(seconds / DT_MDL))]


def test_calibration_wait_says_calibrationds_state():
  assert calibration_wait(calibration()) == ''
  assert calibration_wait(cal('uncalibrated', 45)) == "calibrating cameras: 45%"
  assert calibration_wait(cal('recalibrating', 12)) == "recalibrating cameras: 12%"
  assert calibration_wait(cal('invalid', 100)) == "camera calibration invalid"
  assert calibration_wait(None) == "waiting for camera calibration"  # calibrationd never published
  assert calibration_wait(cal('calibrated', 100, rpy=False)) == "waiting for camera calibration"  # the tracker's test: no rpyCalib


def test_calibrating_from_the_start_is_no_detector_that_never_started():
  # the drive after an OS update: calibrationd starts at 0%, and the detector's runs are no runs until it's done. Two start
  # deadlines of it: unavailable, its reason the progress, nothing handed back, and nothing driven
  sim = calibrating_sim([(3000., "turn", "left"), (5000., "turn", "right")])
  n = round(2 * DETECTOR_START_DEADLINE_S / DT_MDL)
  for k in range(n):
    sim.calibrating = calibration_wait(cal('uncalibrated', k * 100 // n))
    p = sim.step(available=False)
    assert p.state == State.unavailable and p.handback == Handback.none and p.reason == sim.calibrating, (k, p.reason)
    assert p.desire == Desire.none and p.speedCap >= 99.
  assert sim.trace[-1][2].reason == "calibrating cameras: 99%"
  # calibrated, and the detector's first runs: acting at once
  sim.calibrating = ''
  p = sim.step()
  assert p.state != State.unavailable and p.handback == Handback.none


def test_the_start_deadline_holds_through_calibration_and_runs_on_after():
  # waiting for the detector 5 s, then calibration reset (30 s), then calibrated with still no detections: the start deadline
  # is handed back DETECTOR_START_DEADLINE_S of waiting in all, the calibration not counted, nor its clock started over
  sim = calibrating_sim([(3000., "turn", "left"), (5000., "turn", "right")])
  plans = run(sim, 5., available=False)
  assert all(p.reason == WAITING_REASON and p.handback == Handback.none for p in plans)
  sim.calibrating = calibration_wait(cal('recalibrating', 30))
  plans = run(sim, 30., available=False)
  assert all(p.reason == "recalibrating cameras: 30%" and p.handback == Handback.none for p in plans)
  sim.calibrating = ''
  plans = run(sim, DETECTOR_START_DEADLINE_S, available=False)
  first = next(i for i, p in enumerate(plans) if p.handback != Handback.none)
  assert abs((first + 1) * DT_MDL - (DETECTOR_START_DEADLINE_S - 5.)) <= 1.5 * DT_MDL
  assert plans[first].handback == Handback.noDetector and plans[first].reason == HANDBACK_REASON[Handback.noDetector]
  assert all(p.reason == WAITING_REASON for p in plans[:first])


def test_a_recalibration_mid_drive_is_no_stopped_detector():
  # acting, then a remount found mid-drive (recalibrating): the tracker drops every run, the scene goes unavailable at once.
  # Long past the stopped detector's deadline nothing is handed back: TurnPilot stops acting (the turn it was approaching
  # dropped, no desire), says it recalibrates, and drives again once calibrated with fresh detections
  sim = calibrating_sim([(600., "turn", "left"), (2000., "turn", "right")])
  run(sim, 5.)
  assert sim.trace[-1][2].state != State.unavailable
  sim.calibrating = calibration_wait(cal('recalibrating', 3))
  plans = run(sim, 5 * DETECTOR_DEADLINE_S, available=False)
  assert all(p.state == State.unavailable and p.handback == Handback.none and p.reason == "recalibrating cameras: 3%" for p in plans)
  assert all(p.desire == Desire.none and p.laneChange == LaneChange.none for p in plans)
  sim.calibrating = ''
  p = sim.step()
  assert p.state != State.unavailable and p.handback == Handback.none
  # ...and the stopped detector's deadline runs as ever after it
  plans = run(sim, 2 * DETECTOR_DEADLINE_S, available=False)
  first = next(i for i, p in enumerate(plans) if p.handback != Handback.none)
  assert abs((first + 1) * DT_MDL - (DETECTOR_DEADLINE_S - STALE_S)) <= 1.5 * DT_MDL
  assert plans[first].handback == Handback.detectorStopped


def test_a_model_fallback_while_calibrating_still_hands_back():
  # calibration holds the detector's deadlines, not a fallback: the small model can't navigate however calibrated it is
  sim = calibrating_sim([(3000., "turn", "left")])
  sim.big = True
  run(sim, 2.)
  sim.calibrating, sim.big = calibration_wait(cal('uncalibrated', 10)), False
  p = sim.step(available=False)
  assert p.handback == Handback.modelFellBack and p.reason == HANDBACK_REASON[Handback.modelFellBack]


def test_a_fault_while_calibrating_hands_nothing_back():
  # turnpilotd's update() failing every frame still runs the deadlines (plan()'s fault path): with the calibration held too
  sim = calibrating_sim([(3000., "turn", "left")])
  sim.calibrating = calibration_wait(cal('uncalibrated', 10))
  with mock.patch.object(type(sim.tp), 'update', side_effect=RuntimeError("synthetic failure")), \
       mock.patch('openpilot.selfdrive.turnpilot.turnpilotd.log_error'):
    plans = run(sim, 2 * DETECTOR_START_DEADLINE_S, available=False)
  assert all(p.handback == Handback.none for p in plans)
