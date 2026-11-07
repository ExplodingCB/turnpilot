"""turnpilotd as a process: when the manager runs it, and its loop living through exceptions, since a dead process
soft-disables openpilot for the rest of the drive."""
from types import SimpleNamespace
from unittest import mock

import pytest

from opendbc.car.structs import car
from openpilot.cereal import custom, messaging
from openpilot.selfdrive.turnpilot import turnpilotd
from openpilot.selfdrive.turnpilot.scene.lane_paint import UNKNOWN, LinePaint, fill_lane_paint
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



def test_unverified_why_follows_the_vinless_opt_in():
  """turnpilotd says why mode on found no record ("Car Signals Not Verified") as card's resolver decided it: on a zero VIN,
  unreadable with TurnPilotCarSignalsVinless off; with it on, matched on the DongleId, the firmware's exact fingerprint and
  the exact carFw list, and void when the comma or the firmware differs."""
  from openpilot.selfdrive.turnpilot.car_signals import fw_list
  fw = [{'ecu': 'eps', 'address': 0x730, 'fwVersion': b'TeM3_E014p10_0.0.0 (35),E014.21.2', 'brand': 'tesla'}]
  cp = car.CarParams.new_message(carVin="0" * 17, carFingerprint='TESLA_MODEL_3', fingerprintSource='fw', carFw=fw).as_reader()
  record = {'onReason': 9, 'gear': 'P', 'onSeparates': False, 'vin': "0" * 17, 'fw': fw_list(cp.carFw), 'exteriorConfirmed': 1.,
            'dongleId': "0123456789abcdef", 'carFingerprint': 'TESLA_MODEL_3'}
  params = FakeParams(TurnPilotCarSignalsVerified=record, DongleId="0123456789abcdef")
  assert turnpilotd.unverified_why_for(params, cp) == "Car's VIN unreadable: can't match the record"
  params['TurnPilotCarSignalsVinless'] = False
  assert turnpilotd.unverified_why_for(params, cp) == "Car's VIN unreadable: can't match the record"
  params['TurnPilotCarSignalsVinless'] = True
  assert turnpilotd.unverified_why_for(params, cp) == ""
  params['DongleId'] = "fedcba9876543210"
  assert turnpilotd.unverified_why_for(params, cp) == "Recorded on another comma: run the check"
  params['DongleId'] = "0123456789abcdef"
  updated = car.CarParams.new_message(carVin="0" * 17, carFw=[{**fw[0], 'fwVersion': b'TeM3_E014p10_0.0.0 (24),E014.20.2'}]).as_reader()
  assert turnpilotd.unverified_why_for(params, updated) == "Car software changed: run the check again"

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
    from openpilot.selfdrive.turnpilot.scene.tests.test_tracker import calibration
    self.data = {'carState': car_state(10.), 'carControl': car_control(), 'modelV2': model_v2(),
                 'navInstruction': nav_instruction(0., valid=False)}
    for service in ('deviceState', 'narrowRoadCameraState'):
      self.data[service] = getattr(messaging.new_message(service).as_reader(), service)
    self.data['extrinsicsCalibration'] = calibration()  # calibrated: what TurnPilot waits for (test_calibrating_is_said_from_main)
    self.updated = dict.fromkeys(self.data, False)
    self.valid = dict.fromkeys([*self.data, 'turnpilotDetections', 'turnpilotFarDetections'], True)
    self.seen = {'deviceState': True, 'narrowRoadCameraState': True, 'extrinsicsCalibration': True, 'turnpilotDetections': False,
                 'turnpilotFarDetections': False}
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


@pytest.mark.parametrize("status,percent,seen,reason", [
  ('uncalibrated', 45, True, "calibrating cameras: 45%"), ('recalibrating', 12, True, "recalibrating cameras: 12%"),
  ('invalid', 100, True, "camera calibration invalid"), ('uncalibrated', 0, False, "waiting for camera calibration")])
def test_calibrating_is_said_from_main(status, percent, seen, reason):
  # after an OS update reset calibration the phone and the comma said "detector not running: take control" while the car
  # only calibrated: main() hands the plan extrinsicsCalibration's state, which it says, and nothing is handed back
  orig = LoopSM.__init__

  def init(self, *a, **k):
    orig(self, *a, **k)
    msg = messaging.new_message('extrinsicsCalibration')
    msg.extrinsicsCalibration.calStatus, msg.extrinsicsCalibration.calPerc = status, percent
    self.data['extrinsicsCalibration'], self.seen['extrinsicsCalibration'] = msg.as_reader().extrinsicsCalibration, seen
  with mock.patch.object(LoopSM, '__init__', init):
    plans, _, errors = run_main(frames=5)
  assert not errors
  assert all(p.turnpilotPlan.state == State.unavailable and p.turnpilotPlan.reason == reason and
             p.turnpilotPlan.handback == custom.TurnpilotPlan.Handback.none for p in plans)


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


@pytest.mark.parametrize("value,mode", [(None, 'addOnly'), ('addOnly', 'addOnly'), ('shadow', 'shadow'), ('off', 'off'),
                                        ('bogus', 'addOnly')])
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


@pytest.mark.parametrize("value,on", [(None, False), (True, True), (False, False), ('1', True), ('0', False), (b'1', True),
                                      (b'0', False), (' 1\n', True)])
def test_main_reads_the_always_drive_switch(value, on):
  """TurnPilotAlwaysDrive (ALWAYS_DRIVE.md section 1), read once a second beside TurnPilotGoOverride: unset is off, today's
  behaviour; "1" is on."""
  made, real = [], turnpilotd.TurnPilot
  def make():
    made.append(real())
    return made[-1]
  with mock.patch.object(turnpilotd, 'TurnPilot', side_effect=make):
    run_main(frames=3, TurnPilotAlwaysDrive=value)
  assert made[0].always_drive is on
  assert turnpilotd.always_drive_for(FakeParams()) is False


def test_always_drive_unknown_to_the_build_reads_off_and_says_so():
  """A build whose params library doesn't know TurnPilotAlwaysDrive yet (common/ not rebuilt after the merge): off, and
  logged, never an exception out of the param refresh."""
  class Old(FakeParams):
    def get(self, key):
      if key == 'TurnPilotAlwaysDrive':
        raise turnpilotd.UnknownKeyName(key)
      return super().get(key)
  with mock.patch.object(turnpilotd, 'log_error') as log_error:
    assert turnpilotd.always_drive_for(Old(TurnPilotAlwaysDrive=b'1')) is False
  assert log_error.call_count == 1 and 'TurnPilotAlwaysDrive' in log_error.call_args.args[0]
  assert 'unknown' in log_error.call_args.args[0] and 'off' in log_error.call_args.args[0]


def test_always_drive_is_a_persistent_bool_on_by_default():
  # Saturday's value as the fresh install's default (test_saturday_defaults.py); unset, turnpilotd still reads it off
  import re
  from pathlib import Path
  header = (Path(__file__).parents[3] / 'common' / 'params_keys.h').read_text()
  assert re.search(r'\{"TurnPilotAlwaysDrive", \{PERSISTENT, BOOL, "1"\}\}', header)


@pytest.mark.parametrize("value,on", [(None, True), (True, True), (False, False), ('0', False), (b'1', True)])
def test_main_reads_the_curb_pedestrian_cap(value, on):
  """TurnPilotCurbPedCap (scene_policy.py: the curb-pedestrian speed cap; owner decision, BUILD_FINDINGS D3-real option 2),
  read once a second: unset is on, and the scene policy takes it every frame."""
  made, real = [], turnpilotd.TurnPilot
  def make():
    made.append(real())
    return made[-1]
  with mock.patch.object(turnpilotd, 'TurnPilot', side_effect=make):
    run_main(frames=3, TurnPilotCurbPedCap=value)
  assert made[0].curb_ped_cap is on
  assert turnpilotd.curb_ped_cap_for(FakeParams()) is True


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


# confidences and coverages a Float32 carries exactly
READ_PAINT = [LinePaint('solid', 'yellow', 0.875, 0.9375), LinePaint('broken', 'white', 0.875, 0.25), UNKNOWN, LinePaint('none', 'unknown', 0.75, 0.)]
LanePaintStatus = custom.TurnpilotDetections.LanePaintStatus


@pytest.mark.parametrize("status, paint", [
  ('read', READ_PAINT),
  ('noModel', [UNKNOWN] * 4),  # LanePaintReader.read on a road camera frame it couldn't read: four unknowns, filled
  ('wideCamera', None), ('sweep', None), ('disabled', None),  # nothing filled: lane_paint_from_msg makes four unknowns of it
  ('unset', None), ('unset', [UNKNOWN] * 4),  # a log from before the field
])
def test_only_paint_the_reader_read_goes_to_the_plan(status, paint):
  """ad-s4's block on 4.10: the lane paint turnpilotd hands the plan is the reader's only when this message's lanePaintStatus
  is read. A run on the wide camera, a stop sweep, the paint off for the drive (a budget overrun, an exception) and a road
  frame the reader had no modelV2 for all come as LinePaints of kind and colour unknown (or none at all), which a line
  read unknown looks like too: the status alone tells them apart. Unread paint is (), as with the scene unavailable, so a
  curb-side change on less than paint (4.10) never goes on paint nobody read. Off the switch () and four unknowns plan
  the same (no line read broken white, none read yellow or solid)."""
  msg = messaging.new_message('turnpilotDetections')
  if paint is not None:
    fill_lane_paint(msg.turnpilotDetections.init('lanePaint', 4), paint)
  msg.turnpilotDetections.lanePaintStatus = status
  det = msg.as_reader().turnpilotDetections
  tracker = mock.MagicMock()
  tracker.raw_new, tracker.raw_far_new = False, False
  tracker.fill_scene.side_effect = lambda s: setattr(s, 'available', True)  # the scene fresh: only the status decides
  got = []

  def plan(self, *a, **k):
    got.append((a[1].available, a[6]))  # (scene available, the paint)
    return messaging.new_message('turnpilotPlan')
  orig = LoopSM.__init__

  def init(self, *a, **k):
    orig(self, *a, **k)
    self.data['turnpilotDetections'], self.seen['turnpilotDetections'] = det, True
  with mock.patch.object(LoopSM, '__init__', init), mock.patch.object(turnpilotd.TurnPilot, 'plan', plan):
    _, _, errors = run_main(frames=3, tracker=tracker)
  assert not errors and len(got) == 3
  assert all(available for available, _ in got)
  assert all(p == (tuple(READ_PAINT) if status == 'read' else ()) for _, p in got), got[0][1]


def test_paint_is_none_read_with_the_scene_unavailable_or_before_the_first_message():
  """plan_paint's other guards: the reader's read paint goes to the plan only while the scene is fresh (its detections are,
  so the paint is too), and nothing before the first turnpilotDetections; with four lines the reader read, never fewer."""
  msg = messaging.new_message('turnpilotDetections')
  fill_lane_paint(msg.turnpilotDetections.init('lanePaint', 4), READ_PAINT)
  msg.turnpilotDetections.lanePaintStatus = 'read'
  det = msg.as_reader().turnpilotDetections
  assert turnpilotd.plan_paint(det, True) == tuple(READ_PAINT)
  assert turnpilotd.plan_paint(det, False) == () and turnpilotd.plan_paint(None, True) == ()
  short = messaging.new_message('turnpilotDetections')
  fill_lane_paint(short.turnpilotDetections.init('lanePaint', 2), READ_PAINT[:2])
  short.turnpilotDetections.lanePaintStatus = 'read'
  assert turnpilotd.plan_paint(short.as_reader().turnpilotDetections, True) == ()


def test_the_plans_inputs_pass_their_frequency_check_at_turnpilotds_own_rate():
  """2026-11-02, the second drive: carState (100 Hz) read by turnpilotd's 20 Hz loop failed SubMaster's frequency check left at
  carState's rate, so every plan went out invalid and the car did nothing on a shown route. main's SubMaster takes the loop's
  rate, and at it carState and modelV2 as the loop reads them (the newest each frame) are freq_ok."""
  made = {}

  def capture(services, **kwargs):
    made.update(services=services, **kwargs)
    raise SystemExit
  with mock.patch.object(turnpilotd.messaging, 'SubMaster', side_effect=capture), mock.patch.object(turnpilotd, 'Params'), \
       pytest.raises(SystemExit):
    turnpilotd.main()
  assert made['frequency'] == turnpilotd.PLAN_FREQ == 20.

  sm = messaging.SubMaster(['carState', 'modelV2'], frequency=made['frequency'])
  for frame in range(400):  # 20 s of the loop: carState's newest of five, modelV2's one
    t = frame * turnpilotd.DT_MDL
    sm.update_msgs(t, [messaging.new_message('carState', valid=True), messaging.new_message('modelV2', valid=True)])
  assert sm.all_checks(['carState', 'modelV2'])
  unset = messaging.SubMaster(['carState', 'modelV2'])  # the old SubMaster, so the test fails without the fix
  for frame in range(400):
    unset.update_msgs(frame * turnpilotd.DT_MDL, [messaging.new_message('carState', valid=True), messaging.new_message('modelV2', valid=True)])
  assert not unset.freq_ok['carState']
