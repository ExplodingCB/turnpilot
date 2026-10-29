"""The detector comes back: after slow runs, after failures, after TurnPilot is turned on mid-drive, after a model fallback."""
import time
from collections import deque

from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.turnpilot.detector import modeld_detector
from openpilot.selfdrive.turnpilot.detector.modeld_detector import (FORCE_RUN_FRAMES, FORCE_RUN_S, RETRY_MAX_S, RETRY_S, RUN_EVERY,
                                                                     STEP_BUDGET_S, DetectorHost)
from openpilot.selfdrive.turnpilot.detector.tests.test_modeld_detector import PM, Meta, make_detector
from openpilot.selfdrive.turnpilot.scene.tracker import STALE_S
from openpilot.selfdrive.turnpilot.turnpilotd import DETECTOR_DEADLINE_S

METAS = [Meta(1), Meta(2)]


def run_gaps(det, pm, frames: int, model_time: float) -> list[int]:
  """Frames between the detector's runs over `frames` model frames."""
  gaps, last = [], 0
  for k in range(1, frames + 1):
    n = len(pm.sent)
    det.step(pm, det.model_state, model_time, 0.0, log.Desire.none, METAS)
    if len(pm.sent) > n:
      gaps.append(k - last)
      last = k
  return gaps


class FakeParams:
  def __init__(self, enabled=False, lights=None, far=None):
    self.enabled, self.lights, self.far = enabled, lights, far

  def get_bool(self, key):
    assert key == "TurnPilotEnabled"
    return self.enabled

  def get(self, key):
    assert key in ("TurnPilotCameraLights", "TurnPilotFarCrop")
    return self.lights if key == "TurnPilotCameraLights" else self.far


class FakeLoad:
  """Stands in for modeld_detector.load: a fake detector for each model in `have`, else None (no pkl for that device)."""
  def __init__(self, have):
    self.have, self.calls = have, []

  def __call__(self, model_state, *args):
    self.calls.append(model_state)
    if model_state not in self.have:
      return None
    det = make_detector()
    det.model_state = model_state
    return det


class TestDetectorRecovery(OpenpilotTestCase):
  def test_slow_burst_is_measured_again(self):
    # three slow runs while the Mac was loaded left a 60 ms median; the load has gone, so the next run is fast
    det, pm = make_detector(), PM()
    det.times = deque([0.06, 0.07, 0.09, 0.005, 0.005], maxlen=5)
    gaps = run_gaps(det, pm, 40, 0.020)
    assert gaps[0] == FORCE_RUN_FRAMES - RUN_EVERY  # forced, the estimate starting over from that run...
    assert set(gaps[1:]) == {RUN_EVERY}             # ...which fits, so back on the N-frame grid

  def test_no_room_still_runs_inside_the_deadline(self):
    # the driving model alone overruns the budget: a forced run every FORCE_RUN_FRAMES, never further apart, so
    # turnpilotd's deadline never runs out while the detector can still run (a 2.2 s gap at 18 Hz handed back in BeamNG)
    det, pm = make_detector(), PM()
    gaps = run_gaps(det, pm, 1200, STEP_BUDGET_S + 0.005)
    assert gaps[0] == FORCE_RUN_FRAMES - RUN_EVERY and set(gaps[1:]) == {FORCE_RUN_FRAMES}
    assert det.failures == 0

  def test_forced_run_is_inside_the_deadline(self):
    # turnpilotd hands back DETECTOR_DEADLINE_S after the last detections, counting from the scene going stale
    assert FORCE_RUN_S <= 0.8 * (DETECTOR_DEADLINE_S - STALE_S)
    assert FORCE_RUN_FRAMES * 0.05 <= FORCE_RUN_S + 1e-9  # at modeld's 20 Hz, the frame count comes first


  def test_failure_retries_with_backoff(self):
    det, pm = make_detector(), PM()
    det.run.fail = True
    delays = []
    for _ in range(8):
      det.retry_at = 0.0  # the wait is over
      det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, METAS)
      delays.append(det.retry_at)
      for _ in range(3 * RUN_EVERY):  # nothing while it waits
        det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, METAS)
      assert pm.sent == []
    waits = [b - a for a, b in zip(delays, delays[1:], strict=False)]
    assert [round(w) for w in waits[:5]] == [RETRY_S, 2 * RETRY_S, 4 * RETRY_S, 8 * RETRY_S, 16 * RETRY_S]
    assert waits[-1] <= RETRY_MAX_S + 1 and det.failures == 8
    det.run.fail, det.retry_at = False, 0.0
    det.step(pm, det.model_state, 0.010, 0.0, log.Desire.none, METAS)
    assert len(pm.sent) == 1 and det.failures == 0

  def test_host_follows_the_model(self):
    big, small = object(), object()
    loader, pm = FakeLoad({big, small}), PM()
    host = DetectorHost(FakeParams(True), 1344, 760, False, True, [big, small, small], load_fn=loader)
    host.load()
    assert loader.calls == [big, small]
    host.step(pm, big, 0.010, 0.0, log.Desire.none, METAS, True)
    host.step(pm, small, 0.010, 0.0, log.Desire.none, METAS, True)  # modeld fell back to the small model
    assert len(pm.sent) == 2
    big_det, small_det = (d for _, d in host.detectors)
    assert len(big_det.run.calls) == 1 and len(small_det.run.calls) == 1

  def test_host_passes_the_plans_view(self):
    # modeld hands the host turnpilotPlan.detectorView: an unprotected left's go runs on the road camera, its turn desire or not
    model, pm = object(), PM()
    host = DetectorHost(FakeParams(True), 1344, 760, False, True, [model], load_fn=FakeLoad({model}))
    host.load()
    (_, det), = host.detectors
    for _ in range(4):
      host.step(pm, model, 0.010, 0.05, log.Desire.turnLeft, METAS, True, 'road')
    assert det.run.calls and set(det.run.calls) == {'frame0'}
    host.step(pm, model, 0.010, 0.05, log.Desire.turnLeft, METAS, True)
    host.step(pm, model, 0.010, 0.05, log.Desire.turnLeft, METAS, True)
    assert det.run.calls[-1] == 'frame1'  # auto: the turn's wide camera

  def test_host_passes_the_light_mode(self):
    model, params, pm = object(), FakeParams(True, 'off'), PM()
    host = DetectorHost(params, 1344, 760, False, True, [model], load_fn=FakeLoad({model}))
    host.load()
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, True)
    (_, det), = host.detectors
    assert det.light_mode == 'off'
    params.lights, host.next_check = 'addOnly', 0.  # read again with TurnPilotEnabled, once a second
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, True)
    assert det.light_mode == 'addOnly'
    params.lights, host.next_check = None, 0.
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, True)
    assert det.light_mode == 'shadow'

  def test_host_passes_the_far_crop_param_and_aim(self):
    # TurnPilotFarCrop (unset: on) and turnpilotPlan.farCropBearing reach the detector
    model, params, pm = object(), FakeParams(True), PM()
    host = DetectorHost(params, 1344, 760, False, True, [model], load_fn=FakeLoad({model}))
    host.load()
    (_, det), = host.detectors
    got = []
    det.far_step = lambda pm, ms, t, metas, bearing: got.append(bearing)
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, True, 'auto', 6.5)
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, True, 'auto', 6.5)
    assert det.far_on and got == [6.5]  # the frame between the full runs
    params.far, host.next_check = False, 0.
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, True)
    assert not det.far_on
    params.far, host.next_check = b'1', 0.
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, True)
    assert det.far_on

  def test_host_without_a_detector_for_the_fallback(self):
    big, small = object(), object()
    pm = PM()
    host = DetectorHost(FakeParams(True), 1344, 760, False, True, [big, small], load_fn=FakeLoad({big}))
    host.load()
    for _ in range(4):
      host.step(pm, small, 0.010, 0.0, log.Desire.none, METAS, True)
    assert pm.sent == [] and host.missing is small

  def test_host_turned_on_mid_drive(self):
    model, params, pm = object(), FakeParams(False), PM()
    loader = FakeLoad({model})
    host = DetectorHost(params, 1344, 760, False, True, [model], load_fn=loader)
    assert not host.check_enabled(0.0)  # modeld's startup check: off, nothing loaded
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, False)
    assert loader.calls == [] and pm.sent == []
    params.enabled, host.next_check = True, 0.0  # turned on; read on the next check
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, True)
    assert loader.calls == []  # engaged: loading would stall the model, so wait
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, False)
    assert loader.calls == [model] and len(pm.sent) == 1
    params.enabled, host.next_check = False, 0.0  # turned off: loaded, but not run
    for _ in range(2 * RUN_EVERY):
      host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, False)
    assert len(pm.sent) == 1

  def test_host_never_raises_into_modeld(self):
    model, pm = object(), PM()
    host = DetectorHost(FakeParams(True), 1344, 760, False, True, [model], load_fn=FakeLoad({model}))
    host.load()
    det = host.detectors[0][1]
    det.room = None  # a bug outside the detector's own try
    t0 = time.monotonic()
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, False)
    assert not host.enabled and t0 + RETRY_S <= host.next_check < t0 + RETRY_S + 0.5  # retried after RETRY_S, not a minute
    host.next_check = 0.0
    host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, False)
    assert host.failures == 2 and host.next_check >= time.monotonic() + 2 * RETRY_S - 0.5  # doubling
    det.room = lambda _: True  # fixed: runs again, and the count starts over
    host.next_check = 0.0
    for _ in range(RUN_EVERY):
      host.step(pm, model, 0.010, 0.0, log.Desire.none, METAS, False)
    assert host.enabled and host.failures == 0 and len(pm.sent) == 1


@pytest.mark.parametrize("hz", [20., 18., 12., 6.])
def test_slow_model_still_runs_inside_the_deadline(monkeypatch, hz):
  # a loaded host runs the driving model below 20 Hz: the forced run goes by the clock, not by frames, so the gap
  # between runs stays inside turnpilotd's deadline whatever the rate
  clock = SimpleNamespace(t=1000.)
  monkeypatch.setattr(modeld_detector, 'time', SimpleNamespace(monotonic=lambda: clock.t, perf_counter=lambda: clock.t))
  det, pm = make_detector(), PM()
  runs = []
  for _ in range(int(60 * hz)):
    clock.t += 1. / hz
    n = len(pm.sent)
    det.step(pm, det.model_state, STEP_BUDGET_S + 0.005, 0.0, log.Desire.none, METAS)
    if len(pm.sent) > n:
      runs.append(clock.t)
  gaps = [b - a for a, b in zip(runs, runs[1:], strict=False)]
  assert len(runs) > 30 and max(gaps) <= FORCE_RUN_S + 1. / hz + 1e-6
  assert max(gaps) < DETECTOR_DEADLINE_S - STALE_S
