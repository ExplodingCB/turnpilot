"""turnpilotd as a process: when the manager runs it, and its loop living through exceptions, since a dead process
soft-disables openpilot for the rest of the drive."""
from types import SimpleNamespace
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
  assert turnpilotd.driver_signals_for(params)


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
    self.valid = dict.fromkeys([*self.data, 'turnpilotDetections', 'turnpilotFarDetections'], True)
    self.seen = {'deviceState': True, 'narrowRoadCameraState': True, 'turnpilotDetections': False, 'turnpilotFarDetections': False}
    self.frame, self.failing_frames, self.checks = -1, failing_frames, checks

  def update(self, timeout):
    self.frame += 1

  def __getitem__(self, s):
    if s == 'modelV2' and self.frame < self.failing_frames:
      raise RuntimeError("synthetic failure")
    return self.data.get(s)

  def all_checks(self, services):
    return self.checks(self.frame)

  def drain(self):
    """What turnpilotd's own turnpilotDetections socket drains this frame (messaging.drain_sock): the message the SubMaster
    holds while it is updated, as an Event with its valid."""
    if not self.updated.get('turnpilotDetections'):
      return []
    return [SimpleNamespace(valid=self.valid['turnpilotDetections'], turnpilotDetections=self.data['turnpilotDetections'])]


def run_main(frames, failing_frames=0, tracker=None, checks=lambda frame: True, **params):
  sent, sm = [], LoopSM(failing_frames, checks)
  pm = mock.MagicMock()
  pm.send.side_effect = lambda s, msg: sent.append((s, msg.as_reader()))
  rk = mock.MagicMock()
  rk.keep_time.side_effect = [None] * (frames - 1) + [Stop()]
  params = FakeParams(**{'TurnPilotEnabled': True, 'CarParams': car_params_bytes(), 'DisengageOnAccelerator': True, **params})
  with mock.patch.object(messaging, 'SubMaster', return_value=sm), mock.patch.object(messaging, 'PubMaster', return_value=pm), \
       mock.patch.object(messaging, 'sub_sock', return_value=mock.MagicMock()), \
       mock.patch.object(messaging, 'drain_sock', side_effect=lambda sock, wait_for_one=False: sm.drain()), \
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


def test_the_far_crops_runs_go_to_the_tracker_and_its_raw_run_to_the_plan():
  # DESIGN phase 4 G5: turnpilotFarDetections (optional) to SceneTracker.update_far with the calibration, never to update();
  # a new far run to the plan as raw_far (the unprotected left's), TurnPilotLeftFarRangeM read with the other params
  tracker = mock.MagicMock()
  tracker.raw_new, tracker.raw_far_new = False, True
  far = messaging.new_message('turnpilotFarDetections').as_reader().turnpilotFarDetections
  got = {}
  real_plan = turnpilotd.TurnPilot.plan

  def plan(self, *a, **k):
    got.update(k, far_range=self.left_far_range_m)
    return real_plan(self, *a, **k)
  orig = LoopSM.__init__

  def init(self, *a, **k):
    orig(self, *a, **k)
    self.data['turnpilotFarDetections'], self.seen['turnpilotFarDetections'] = far, True
  with mock.patch.object(LoopSM, '__init__', init), mock.patch.object(turnpilotd.TurnPilot, 'plan', plan):
    run_main(frames=3, tracker=tracker, TurnPilotLeftFarRangeM=260.)
  args = tracker.update_far.call_args.args
  assert args[0] is far and len(args) == 2
  assert got['raw_far'] is tracker.raw_far and got['raw'] is None and got['far_range'] == 260.


def nan_runs(trk):
  """A full pass run and a far crop run whose model output wasn't finite, as modeld publishes them: valid False, the
  non-finite rows dropped by the decoders (no boxes), the far run's crop still set."""
  import numpy as np
  from openpilot.selfdrive.turnpilot.detector.decoder import decode, decode_far, fill_detections, fill_far
  from openpilot.selfdrive.turnpilot.detector.graph import letterbox
  from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import far_crop_at_horizon
  nan = np.full((1, 300, 6), np.nan, np.float32)
  full = messaging.new_message('turnpilotDetections', valid=bool(np.isfinite(nan).all()))
  fill_detections(full.turnpilotDetections, decode(nan, letterbox(1344, 760)), 1, int(1e9), 'road', 0.006)
  crop, _ = far_crop_at_horizon(trk)
  far = messaging.new_message('turnpilotFarDetections', valid=bool(np.isfinite(nan).all()))
  fill_far(far.turnpilotFarDetections, decode_far(nan, crop), crop, 0., 2, int(1.05e9), 0.006)
  assert not full.valid and not far.valid
  return full.as_reader().turnpilotDetections, far.as_reader().turnpilotFarDetections


@pytest.mark.parametrize("valid", [True, False])
def test_an_invalid_detector_run_is_no_run_never_a_clear_one(valid):
  # skeptic G6: a run whose output wasn't finite carries no boxes, and a far one still its crop, so taken as a run it is a
  # clear one: the far crop's covering the band (the left's far_age reset, the run counted clear) and the full pass's a
  # fresh clear road run. SubMaster takes msg.valid into sm.valid even for the optional services (ignore_valid): an
  # invalid run never reaches the tracker, for either pass, so the left sees no run (never clear; stale is its fault)
  from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import CAMS, calibration
  from openpilot.selfdrive.turnpilot.scene.tracker import SceneTracker
  tracker = SceneTracker(CAMS, 'mici')
  full, far = nan_runs(tracker)
  got = []
  real_plan = turnpilotd.TurnPilot.plan

  def plan(self, *a, **k):
    got.append((k['raw'], k['raw_far']))
    return real_plan(self, *a, **k)
  orig = LoopSM.__init__

  def init(self, *a, **k):
    orig(self, *a, **k)
    self.data['extrinsicsCalibration'] = calibration()
    for s, msg in (('turnpilotDetections', full), ('turnpilotFarDetections', far)):
      self.data[s], self.seen[s], self.updated[s], self.valid[s] = msg, True, True, valid
  with mock.patch.object(LoopSM, '__init__', init), mock.patch.object(turnpilotd.TurnPilot, 'plan', plan), \
       mock.patch.dict(turnpilotd.INVALID_RUNS, clear=True), mock.patch.object(turnpilotd.cloudlog, 'event') as event:
    _, _, errors = run_main(frames=3, tracker=tracker)
    invalid = [c.kwargs['service'] for c in event.call_args_list if c.args[0] == "turnpilot.detector_run_invalid"]
    assert sorted(invalid) == ([] if valid else ['turnpilotDetections', 'turnpilotFarDetections'])  # once a spell, logged
    assert turnpilotd.INVALID_RUNS == ({} if valid else {'turnpilotDetections': 3, 'turnpilotFarDetections': 3})
  assert not errors and len(got) == 3
  raws, raw_fars = [r for r, _ in got if r is not None], [f for _, f in got if f is not None]
  if valid:  # the same runs, valid (no boxes, nothing there): a clear road run and a covering clear far run, once each
    assert len(raws) == len(raw_fars) == 1 and raws[0].boxes == raw_fars[0].boxes == [] and raw_fars[0].cover_deg is not None
  else:
    assert raws == raw_fars == [] and tracker.raw is None and tracker.raw_far is None and not tracker.available


def test_scene_failure_publishes_an_empty_scene_and_goes_on():
  tracker = mock.MagicMock()
  tracker.update_runs.side_effect = RuntimeError("synthetic failure")  # every run goes through update_runs (DetectionDrain)
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


@pytest.mark.parametrize("value,on", [(None, True), (True, True), (False, False), ('0', False)])
def test_main_reads_the_go_override(value, on):
  """TurnPilotGoOverride (go_override.py, G5), read once a second: unset is on."""
  made, real = [], turnpilotd.TurnPilot
  def make():
    made.append(real())
    return made[-1]
  with mock.patch.object(turnpilotd, 'TurnPilot', side_effect=make):
    run_main(frames=3, TurnPilotGoOverride=value)
  assert made[0].go_override is on


def test_a_sweep_coasts_the_trackers_tracks_longer_and_the_watch_sees_every_run():
  # STOP_SIGNS.md section 9: MAX_COAST_S 0.65 s while a stop sweep runs (the last plan asked for one), 0.5 s otherwise; and
  # the junction watch gets the tracker's runs and tracks every frame
  from openpilot.selfdrive.turnpilot.scene.tracker import MAX_COAST_S, SWEEP_COAST_S
  tracker = mock.MagicMock()
  tracker.raw_new, tracker.raw_far_new, tracker.new_runs, tracker.tracks = False, False, [('road', 1.0, None, None)], []
  coasts, observed = [], []
  real_plan = turnpilotd.TurnPilot.plan

  def plan(self, *a, **k):
    coasts.append(tracker.coast_s)
    msg = real_plan(self, *a, **k)
    self.sweeping = len(coasts) >= 2  # the second plan asks for a sweep
    return msg

  def observe(self, trk):
    observed.append(list(trk.new_runs))
  with mock.patch.object(turnpilotd.TurnPilot, 'plan', plan), mock.patch.object(turnpilotd.TurnPilot, 'observe_runs', observe):
    run_main(frames=4, tracker=tracker)
  assert coasts == [MAX_COAST_S, MAX_COAST_S, SWEEP_COAST_S, SWEEP_COAST_S]
  assert observed == [[('road', 1.0, None, None)]] * 4
