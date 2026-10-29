"""turnpilotd as a process: when the manager runs it, and its loop living through exceptions, since a dead process
soft-disables openpilot for the rest of the drive."""
from unittest import mock

import pytest

from opendbc.car.structs import car
from openpilot.cereal import custom, messaging
from openpilot.selfdrive.turnpilot import turnpilotd
from openpilot.selfdrive.turnpilot.tests.synthetic import car_control, car_state, model_v2, nav_instruction
from openpilot.system.manager.process_config import managed_processes

State = custom.TurnpilotPlan.State


class FakeParams(dict):
  def get(self, key):
    return super().get(key)

  def get_bool(self, key):
    return bool(super().get(key, False))


def car_params_bytes(long_control=True):
  return car.CarParams.new_message(openpilotLongitudinalControl=long_control).to_bytes()


@pytest.mark.parametrize("started", [False, True])
@pytest.mark.parametrize("enabled", [False, True])
def test_runs_onroad_only_with_turnpilot_enabled(started, enabled):
  params = FakeParams(TurnPilotEnabled=enabled)
  assert managed_processes['turnpilotd'].should_run(started, params, car.CarParams.new_message()) == (started and enabled)


def test_unreadable_car_params_fail_safe():
  params = FakeParams(CarParams=b"not capnp")
  assert not turnpilotd.long_control_for(params)
  # the driver signals unless the car's own signal is ready, whatever the brand (car_signals.py): unreadable CarParams
  # carry no car signal flags, so the status is off or unverified and the driver is prompted
  assert turnpilotd.car_params(params) is None
  assert turnpilotd.unverified_why_for(params, None) == "waiting for the car's parameters"
  assert turnpilotd.driver_signals_for(turnpilotd.custom.TurnpilotPlan.TurnSignal.Status.off)


def test_error_log_is_throttled():
  log = turnpilotd.ErrorLog()
  with mock.patch.object(turnpilotd.cloudlog, 'exception') as exception, mock.patch.object(turnpilotd.time, 'monotonic') as now:
    for t in (100., 101., 100. + turnpilotd.ERROR_LOG_PERIOD_S):
      now.return_value = t
      log("a")
    log("b")
  assert [c.args[0] for c in exception.call_args_list] == ["a", "a", "b"]


class Stop(BaseException):
  """Ends the loop from Ratekeeper.keep_time; a BaseException, so the loop's own guards don't take it."""


class LoopSM:
  """What turnpilotd's main loop reads from its SubMaster. modelV2 raises for the first frames, and the input checks
  pass on the frames checks(frame) says."""
  def __init__(self, failing_frames, checks):
    self.data = {'carState': car_state(10.), 'carControl': car_control(), 'modelV2': model_v2(),
                 'navInstruction': nav_instruction(0., valid=False)}
    for service in ('deviceState', 'narrowRoadCameraState', 'extrinsicsCalibration'):
      self.data[service] = getattr(messaging.new_message(service).as_reader(), service)
    self.updated = dict.fromkeys(self.data, False)
    self.valid = dict.fromkeys(self.data, True)
    self.seen = {'deviceState': True, 'narrowRoadCameraState': True, 'turnpilotDetections': False}
    self.frame, self.failing_frames, self.checks = -1, failing_frames, checks

  def update(self, timeout):
    self.frame += 1

  def __getitem__(self, s):
    if s == 'modelV2' and self.frame < self.failing_frames:
      raise RuntimeError("synthetic failure")
    return self.data.get(s)

  def all_checks(self, services):
    return self.checks(self.frame)


def run_main(frames, failing_frames=0, tracker=None, checks=lambda frame: True, **params):
  sent, sm = [], LoopSM(failing_frames, checks)
  pm = mock.MagicMock()
  pm.send.side_effect = lambda s, msg: sent.append((s, msg.as_reader()))
  rk = mock.MagicMock()
  rk.keep_time.side_effect = [None] * (frames - 1) + [Stop()]
  params = FakeParams(**{'TurnPilotEnabled': True, 'CarParams': car_params_bytes(), 'DisengageOnAccelerator': True, **params})
  with mock.patch.object(messaging, 'SubMaster', return_value=sm), mock.patch.object(messaging, 'PubMaster', return_value=pm), \
       mock.patch.object(turnpilotd, 'Params', return_value=params), mock.patch.object(turnpilotd, 'Ratekeeper', return_value=rk), \
       mock.patch.object(turnpilotd, 'SceneTracker', return_value=tracker or mock.MagicMock()), \
       mock.patch.object(turnpilotd, 'DEVICE_CAMERAS', mock.MagicMock()), \
       mock.patch.object(turnpilotd, 'log_error') as log_error, pytest.raises(Stop):
    turnpilotd.main()
  plans = [msg for s, msg in sent if s == 'turnpilotPlan']
  scenes = [msg.turnpilotScene for s, msg in sent if s == 'turnpilotScene']
  return plans, scenes, [c.args[0] for c in log_error.call_args_list]


def test_plan_failure_publishes_a_disabled_plan_and_goes_on():
  plans, scenes, errors = run_main(frames=10, failing_frames=4)
  assert len(plans) == len(scenes) == 10
  plans = [p.turnpilotPlan for p in plans]
  assert all(p.state == State.disabled and p.reason == turnpilotd.ERROR_REASON and p.desire == 0 for p in plans[:4])
  assert all(p.fault == custom.TurnpilotPlan.Fault.error for p in plans[:4])  # selfdrived shows turnpilotUnavailable
  # recovered: no detections in this harness, so TurnPilot waits for the detector (tp/safety-scene) rather than acting
  assert all(p.state == State.unavailable and p.reason == turnpilotd.WAITING_REASON and p.fault == custom.TurnpilotPlan.Fault.none
             for p in plans[4:])
  assert errors == ["turnpilotd.scene_failed", "turnpilotd.plan_failed"] * 4  # the tracker reads modelV2 too


def test_disengage_on_accelerator_off_faults_out_loud():
  # a device with DisengageOnAccelerator written off (the old gas go wanted it off): a gas press while engaged wouldn't
  # disengage, so every plan is faulted for it (selfdrived: turnpilotUnavailable), never TurnPilot running on silently
  plans, _, errors = run_main(frames=5, DisengageOnAccelerator=False)
  plans = [p.turnpilotPlan for p in plans]
  assert all(p.state == State.disabled and p.fault == custom.TurnpilotPlan.Fault.disengageOnAccelerator and
             p.reason == turnpilotd.GAS_DISENGAGE_REASON for p in plans)
  assert not errors


def test_scene_failure_publishes_an_empty_scene_and_goes_on():
  tracker = mock.MagicMock()
  tracker.update.side_effect = RuntimeError("synthetic failure")
  plans, scenes, errors = run_main(frames=5, failing_frames=0, tracker=tracker)
  assert len(plans) == len(scenes) == 5
  # an empty scene: unavailable, so TurnPilot doesn't act (waiting for the detector; once it has run, a stopped detector)
  assert not any(s.available for s in scenes) and all(p.turnpilotPlan.state == State.unavailable for p in plans)
  assert errors == ["turnpilotd.scene_failed"] * 5


def test_a_bad_input_frame_keeps_the_plan_valid():
  # a plan invalid for one frame would drop its desire for that frame, and modeld would take it coming back as a pulse
  plans, _, _ = run_main(frames=40, checks=lambda frame: frame != 20)
  assert all(p.valid for p in plans)
  # inputs failing for 15 frames: invalid from the PLAN_INPUT_GRACE_FRAMES-th, valid again once they pass
  plans, _, _ = run_main(frames=40, checks=lambda frame: not 10 <= frame < 25)
  assert [i for i, p in enumerate(plans) if not p.valid] == list(range(10 + turnpilotd.PLAN_INPUT_GRACE_FRAMES - 1, 25))
  # and invalid from the start until the inputs first pass
  plans, _, _ = run_main(frames=5, checks=lambda frame: frame >= 3)
  assert [p.valid for p in plans] == [False, False, False, True, True]


def test_a_model_fallback_before_this_process_still_hands_back():
  # TurnPilot turned on after a fallback, or off and on after one: a new process. modeld's ChestnutActive says so
  Handback = custom.TurnpilotPlan.Handback
  plans, _, _ = run_main(frames=5, ChestnutActive=False)
  assert all(p.turnpilotPlan.handback == Handback.modelFellBack for p in plans)
  for active in (True, None):  # the big model running, or none expected (no chestnut): no fallback
    plans, _, _ = run_main(frames=5, ChestnutActive=active)
    assert all(p.turnpilotPlan.handback == Handback.none for p in plans)


@pytest.mark.parametrize("value,mode", [(None, 'shadow'), ('addOnly', 'addOnly'), ('off', 'off'), ('bogus', 'shadow')])
def test_main_reads_the_camera_lights_mode(value, mode):
  made, real = [], turnpilotd.TurnPilot
  def make():
    made.append(real())
    return made[-1]
  with mock.patch.object(turnpilotd, 'TurnPilot', side_effect=make):
    run_main(frames=3, TurnPilotCameraLights=value)
  assert made[0].camera_mode == mode


@pytest.mark.parametrize("value,profile", [(None, 'graduated'), ('none', 'none'), ('flat3', 'flat3'), ('flat43', 'flat43'),
                                           ('graduated', 'graduated'), ('left', 'left'), ('bogus', 'graduated'), (b'flat3', 'flat3')])
def test_main_reads_the_post_go_profiles(value, profile):
  """TurnPilotPostGoProfile and TurnPilotLeftGoProfile pick the post-go profiles by name (turn_speed.POST_GO_PROFILES), so the
  BeamNG sweep sets them without a code edit; unset or unknown is the default (graduated, left), logged."""
  made, real = [], turnpilotd.TurnPilot
  def make():
    made.append(real())
    return made[-1]
  with mock.patch.object(turnpilotd, 'TurnPilot', side_effect=make):
    run_main(frames=3, TurnPilotPostGoProfile=value, TurnPilotLeftGoProfile=value)
  assert made[0].post_go_profile == profile
  assert made[0].left_go_profile == (profile if value not in (None, 'bogus') else 'left')


def test_post_go_profile_unknown_to_the_build_is_the_default():
  class Old(FakeParams):
    def get(self, key):
      if key in ('TurnPilotPostGoProfile', 'TurnPilotLeftGoProfile'):
        raise turnpilotd.UnknownKeyName(key)
      return super().get(key)
  with mock.patch.object(turnpilotd, 'log_error') as log_error:
    assert turnpilotd.post_go_profile_for(Old(), 'TurnPilotPostGoProfile', 'graduated') == 'graduated'
    assert turnpilotd.post_go_profile_for(Old(), 'TurnPilotLeftGoProfile', 'left') == 'left'
  assert log_error.call_count == 2
  with mock.patch.object(turnpilotd, 'log_error') as log_error:
    assert turnpilotd.post_go_profile_for(FakeParams(TurnPilotPostGoProfile='nope'), 'TurnPilotPostGoProfile', 'graduated') == 'graduated'
  assert log_error.call_count == 1


@pytest.mark.parametrize("value,limit", [(None, 2.0), ('5.0', 5.0), (20.0, 20.0), ('0.5', 2.0), ('100', 20.0), ('x', 2.0)])
def test_main_reads_the_no_move_limit(value, limit):
  made, real = [], turnpilotd.TurnPilot
  def make():
    made.append(real())
    return made[-1]
  with mock.patch.object(turnpilotd, 'TurnPilot', side_effect=make):
    run_main(frames=3, TurnPilotNoMoveS=value)
  assert made[0].no_move_s == limit
